"""Bit-exact software twin of the TinyTapeout drone detector RTL.

The chip has no host: a PDM microphone bit-stream goes in one pin, an LED comes
out another. Everything between is fixed at tape-out.

Signal chain, all integer, no multipliers anywhere:

    PDM 1-bit @ CLK/PDM_DIV
      -> dyadic 1-pole cascade: state[b] += (in - state[b]) >> K,
         stage b clocked every 2^b PDM ticks (decimate-by-2 per octave)
      -> band[b] = state[b-1] - state[b]            (octave band-pass)
      -> feat = bit_length(|band|)                  (log2, i.e. a priority encoder)
      -> per-band max over a frame of 2^FRAME_LOG2 ticks
      -> 16 frames x NBAND ternary template, NPHASE staggered accumulators
      -> max over time > threshold -> LED

Only the template weights are learned. Everything upstream is fixed hardware,
so features are extracted once and cached; see train_ww.py.

Nothing here uses floating point except the sigma-delta *microphone model*,
which stands in for a real PDM mic and is not part of the chip.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict, field

import numpy as np

# ---------------------------------------------------------------------------
# Hardware parameters -- must match the RTL
# ---------------------------------------------------------------------------
CLK_HZ = 50_000_000
PDM_DIV = 32                       # mic clock = CLK / PDM_DIV
PDM_HZ = CLK_HZ // PDM_DIV         # 1_562_500

NSTAGE = 9                         # cascade depth
K_SHIFT = 2                        # 1-pole coefficient: y += (x-y) >> K
STATE_W = 10                       # signed state width
IN_AMP = 1 << (STATE_W - 3)        # PDM +-1 maps to +-IN_AMP

TAP0 = 4                           # first cascade stage used as a band
NBAND = 5                          # bands = stages TAP0 .. TAP0+NBAND-1
# Log resolution. MANT=0 -> a bare priority encoder, 6 dB per step.
# MANT=1 -> priority encoder plus the bit below the MSB, 3 dB per step,
# which costs one extra feature bit and one 2:1 mux in hardware.
MANT = 1
FEAT_W = 4                         # log-magnitude feature width
FEAT_MAX = (1 << FEAT_W) - 1

FRAME_LOG2 = 16                    # frame = 65_536 PDM ticks = 41.9 ms
NFRAME = 16                        # template depth -> 671 ms window
NPHASE = 2                         # staggered accumulators, hop = 8 frames
SCORE_W = 10                       # signed template score

AUDIO_HZ = 16_000                  # Speech Commands sample rate


@dataclass
class HWConfig:
    clk_hz: int = CLK_HZ
    pdm_div: int = PDM_DIV
    nstage: int = NSTAGE
    k_shift: int = K_SHIFT
    state_w: int = STATE_W
    in_amp: int = IN_AMP
    mant: int = MANT
    tap0: int = TAP0
    nband: int = NBAND
    feat_w: int = FEAT_W
    frame_log2: int = FRAME_LOG2
    nframe: int = NFRAME
    nphase: int = NPHASE
    score_w: int = SCORE_W
    feat_max: int = FEAT_MAX

    def __post_init__(self):
        # Derived exactly as the RTL derives them, so changing state_w or
        # feat_w in one place changes both the model and the silicon.
        self.in_amp = 1 << (self.state_w - 3)
        self.feat_max = (1 << self.feat_w) - 1

    def to_dict(self):
        d = asdict(self)
        d.pop("feat_max", None)
        return d

    @property
    def pdm_hz(self):
        return self.clk_hz // self.pdm_div

    @property
    def frame_ms(self):
        return 1000.0 * (1 << self.frame_log2) / self.pdm_hz

    def band_edges_hz(self):
        """Cutoff of cascade stage b, in Hz. Band b spans (fc[b], fc[b-1])."""
        return [self.pdm_hz / (2 ** b) / (2 * np.pi * (2 ** self.k_shift))
                for b in range(self.nstage)]


CFG = HWConfig()


# ---------------------------------------------------------------------------
# Microphone model: PCM -> 1-bit PDM (NOT part of the chip)
# ---------------------------------------------------------------------------
def pdm_phase_index(n_ticks: int, audio_len: int, cfg: HWConfig = CFG) -> np.ndarray:
    """Zero-order-hold index from PDM tick -> audio sample."""
    step = AUDIO_HZ / cfg.pdm_hz
    idx = (np.arange(n_ticks, dtype=np.float64) * step).astype(np.int32)
    return np.clip(idx, 0, audio_len - 1)


def pdm_phase_lerp(n_ticks: int, audio_len: int, cfg: HWConfig = CFG):
    """Linear-interpolation weights from PDM tick -> a pair of audio samples.

    The zero-order hold above repeats each 16 kHz sample for ~97.7 PDM ticks,
    which is a staircase: it plants an image of the whole spectrum at every
    multiple of 16 kHz, shaped by the hold's sinc. A real PDM microphone sees a
    continuous pressure waveform and produces no such images, so the ZOH is a
    property of the *simulation* rather than of the part -- and its artifacts
    land near Nyquist, in the two highest cascade bands.

    Returns (i0, i1, w) with sample = a[i0]*(1-w) + a[i1]*w.
    """
    step = AUDIO_HZ / cfg.pdm_hz
    pos = np.arange(n_ticks, dtype=np.float64) * step
    i0 = np.clip(pos.astype(np.int32), 0, audio_len - 1)
    i1 = np.clip(i0 + 1, 0, audio_len - 1)
    return i0, i1, (pos - i0).astype(np.float32)


def pdm_encode_batch(audio: np.ndarray, n_ticks: int, cfg: HWConfig = CFG,
                     gain: float = 0.5, interp: str = "zoh"):
    """Second-order sigma-delta, vectorised over the batch, yielded per tick.

    audio: (B, L) float in [-1, 1]. Yields (B,) arrays of +1 / -1 int8.
    A generator so the full B x n_ticks bitstream never has to exist.

    ``interp`` selects how the 16 kHz clip is resampled to the mic's 1.5625 MHz
    tick rate: "zoh" is what every cached feature set was built with, "linear"
    is the more faithful stand-in for a real microphone (see pdm_phase_lerp).
    ``gain`` sets how hard the modulator is driven; the integrators clip at
    +-3, so the default 0.5 -- on top of a clip already peak-normalised to 0.7
    and randomly attenuated -- leaves most of the modulator's range unused.
    """
    B = audio.shape[0]
    lerp = interp == "linear"
    if lerp:
        i0x, i1x, wx = pdm_phase_lerp(n_ticks, audio.shape[1], cfg)
    else:
        idx = pdm_phase_index(n_ticks, audio.shape[1], cfg)
    i1 = np.zeros(B, dtype=np.float32)
    i2 = np.zeros(B, dtype=np.float32)
    y = np.ones(B, dtype=np.float32)
    a = (audio * gain).astype(np.float32)
    for n in range(n_ticks):
        x = (a[:, i0x[n]] * (1.0 - wx[n]) + a[:, i1x[n]] * wx[n]) if lerp \
            else a[:, idx[n]]
        i1 += x - y
        i2 += i1 - y
        np.clip(i1, -3.0, 3.0, out=i1)
        np.clip(i2, -3.0, 3.0, out=i2)
        y = np.where(i2 >= 0, 1.0, -1.0).astype(np.float32)
        yield y


# ---------------------------------------------------------------------------
# The chip's front end -- integer, bit-exact
# ---------------------------------------------------------------------------
_POW2 = (1 << np.arange(0, 32, dtype=np.int64))


def _bit_length(v: np.ndarray) -> np.ndarray:
    """Position of the most significant set bit + 1; 0 for 0.

    A priority encoder in hardware -- which is why the log costs nothing.
    """
    return np.searchsorted(_POW2, v.astype(np.int64), side="right").astype(np.int32)


def log_feature(v, mant: int = MANT, feat_max: int = FEAT_MAX):
    """Integer log magnitude: priority-encoder exponent plus `mant` mantissa bits.

    mant=0 gives 6 dB steps, mant=1 gives 3 dB steps. Exactly what the RTL does.
    """
    v = np.asarray(v)
    e = _bit_length(np.abs(v))
    if mant == 0:
        return np.minimum(e, feat_max)
    sh = np.maximum(e - 1 - mant, 0)
    m = (np.abs(v).astype(np.int64) >> sh) & ((1 << mant) - 1)
    f = np.where(e > mant, ((e - mant) << mant) | m, e)
    return np.minimum(f, feat_max).astype(np.int32)


def log_feature_scalar(v: int, mant: int = MANT, feat_max: int = FEAT_MAX) -> int:
    v = abs(int(v))
    e = v.bit_length()
    if mant == 0:
        return min(e, feat_max)
    if e <= mant:
        return min(e, feat_max)
    m = (v >> max(e - 1 - mant, 0)) & ((1 << mant) - 1)
    return min(((e - mant) << mant) | m, feat_max)


def frontend_batch(audio: np.ndarray, cfg: HWConfig = CFG,
                   n_frames: int | None = None, gain: float = 0.5,
                   progress: bool = False, interp: str = "zoh") -> np.ndarray:
    """Run the exact integer front end over a batch of waveforms.

    audio: (B, L) float in [-1, 1] at AUDIO_HZ.
    returns: (B, n_frames, NBAND) uint8 log-magnitude features.
    """
    B = audio.shape[0]
    n_frames = n_frames or cfg.nframe
    n_ticks = n_frames << cfg.frame_log2

    state = np.zeros((cfg.nstage, B), dtype=np.int32)
    feat = np.zeros((B, n_frames, cfg.nband), dtype=np.uint8)
    fmax_c = cfg.feat_max
    frame_max = np.zeros((cfg.nband, B), dtype=np.int32)

    frame_mask = (1 << cfg.frame_log2) - 1
    taps = [cfg.tap0 + i for i in range(cfg.nband)]

    for n, bit in enumerate(pdm_encode_batch(audio, n_ticks, cfg, gain, interp)):
        x = (bit * cfg.in_amp).astype(np.int32)

        # Cascade. Stage b is clocked once every 2^b PDM ticks.
        prev = x
        for b in range(cfg.nstage):
            if n & ((1 << b) - 1):
                break                       # this stage (and all deeper) idle
            s = state[b]
            state[b] = s + ((prev - s) >> cfg.k_shift)
            prev = state[b]

        # Bands and per-frame max of the log magnitude.
        for i, b in enumerate(taps):
            if n & ((1 << b) - 1):
                continue                    # band only changes when its stage ticks
            band = state[b - 1] - state[b]
            np.maximum(frame_max[i], log_feature(band, cfg.mant, fmax_c),
                       out=frame_max[i])

        if (n & frame_mask) == frame_mask:
            f = n >> cfg.frame_log2
            feat[:, f, :] = np.minimum(frame_max, fmax_c).T.astype(np.uint8)
            frame_max[:] = 0
            if progress and (f % 4 == 3):
                print(f"    frame {f+1}/{n_frames}", end="\r", flush=True)

    return feat


def frontend_bits(bits, cfg: HWConfig = CFG, n_frames: int | None = None):
    """Scalar reference: same front end driven by an explicit bit sequence.

    bits: iterable of 0/1. Returns (n_frames, NBAND) list of lists.
    This is the version the cocotb testbench compares the RTL against.
    """
    n_frames = n_frames or cfg.nframe
    state = [0] * cfg.nstage
    frame_max = [0] * cfg.nband
    taps = [cfg.tap0 + i for i in range(cfg.nband)]
    out = []
    frame_mask = (1 << cfg.frame_log2) - 1

    for n, b01 in enumerate(bits):
        x = cfg.in_amp if b01 else -cfg.in_amp
        prev = x
        for b in range(cfg.nstage):
            if n & ((1 << b) - 1):
                break
            state[b] = state[b] + ((prev - state[b]) >> cfg.k_shift)
            prev = state[b]
        for i, b in enumerate(taps):
            if n & ((1 << b) - 1):
                continue
            e = log_feature_scalar(state[b - 1] - state[b], cfg.mant, cfg.feat_max)
            if e > frame_max[i]:
                frame_max[i] = e
        if (n & frame_mask) == frame_mask:
            out.append([min(v, cfg.feat_max) for v in frame_max])
            frame_max = [0] * cfg.nband
            if len(out) == n_frames:
                break
    return out


# ---------------------------------------------------------------------------
# The chip's back end -- ternary template + staggered argmax over time
# ---------------------------------------------------------------------------
def template_score(feat: np.ndarray, W: np.ndarray) -> np.ndarray:
    """feat: (..., NFRAME, NBAND) uint8. W: (NWORD, NFRAME, NBAND) in {-1,0,1}.

    Returns (..., NWORD) signed scores -- exactly the adder tree in the RTL.
    """
    f = feat.astype(np.int32)
    return np.tensordot(f, W.astype(np.int32), axes=([-2, -1], [-2, -1]))


def clip_score(v, score_w: int = SCORE_W):
    lo, hi = -(1 << (score_w - 1)), (1 << (score_w - 1)) - 1
    return np.clip(v, lo, hi)


class Detector:
    """Streaming detector matching the hardware implementation exactly.

    Per phase, per hidden unit: a saturating accumulator that resets to a
    hard-wired constant at the start of its window, accumulates the ternary
    template frame by frame, then requantises to 4 bits at the window end. The
    output layer is one ternary weight per hidden unit.
    """

    def __init__(self, W1, hbias, W2, thresh, cfg: HWConfig,
                 hacc_w: int = 7, hshift: int = 1, feat_off: int = 6,
                 refractory_frames: int = 16):
        self.W1 = np.asarray(W1, dtype=np.int64)          # (H, NFRAME, NBAND)
        self.hbias = np.asarray(hbias, dtype=np.int64)    # (H,)
        self.W2 = np.asarray(W2, dtype=np.int64).reshape(-1)
        self.thresh = int(thresh)
        self.cfg = cfg
        self.H = self.W1.shape[0]
        self.hacc_w = hacc_w
        self.hshift = hshift
        self.feat_off = feat_off
        self.lim = (1 << (hacc_w - 1)) - 1
        self.refractory = refractory_frames
        self.reset()

    def reset(self):
        self.acc = np.zeros((self.cfg.nphase, self.H), dtype=np.int64)
        self.frame = 0
        self.hold = 0
        self.fired = []

    def _sat(self, v):
        return np.clip(v, -self.lim - 1, self.lim)

    def push_frame(self, band_feat) -> int:
        """Feed one frame of NBAND features; returns the LED state."""
        c = self.cfg
        hop = c.nframe // c.nphase
        bf = np.asarray(band_feat, dtype=np.int64) - self.feat_off
        for p in range(c.nphase):
            slot = (self.frame - p * hop) % c.nframe
            base = self.hbias if slot == 0 else self.acc[p]
            self.acc[p] = self._sat(base + (self.W1[:, slot, :] * bf).sum(1))
            if slot == c.nframe - 1:
                h = np.clip(self.acc[p] >> self.hshift, 0, 15)
                h[self.acc[p] < 0] = 0
                score = int((h * self.W2).sum())
                if score > self.thresh:
                    self.hold = self.refractory
                    self.fired.append((self.frame, score))
        self.frame += 1
        led = 1 if self.hold > 0 else 0
        self.hold = max(self.hold - 1, 0)
        return led


def describe(cfg: HWConfig = CFG):
    e = cfg.band_edges_hz()
    lines = [f"PDM {cfg.pdm_hz/1e6:.4f} MHz  (clk {cfg.clk_hz/1e6:.1f} MHz / {cfg.pdm_div})",
             f"frame {cfg.frame_log2} bits = {cfg.frame_ms:.1f} ms, "
             f"{cfg.nframe} frames = {cfg.nframe*cfg.frame_ms:.0f} ms window",
             f"{cfg.nstage} cascade stages, K={cfg.k_shift}, state {cfg.state_w} b",
             "bands (stage: passband):"]
    for i in range(cfg.nband):
        b = cfg.tap0 + i
        lines.append(f"  band {i}  stage {b:2d}   {e[b]:8.1f} .. {e[b-1]:8.1f} Hz")
    return "\n".join(lines)


if __name__ == "__main__":
    print(describe())
