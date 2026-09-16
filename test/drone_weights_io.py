"""Single source of truth for the weight header and the RTL constants it assumes.

`test/test.py` (cocotb) and any live-microphone harness read the generated
`src/drone_weights.svh` through this module, so there is exactly one parser and
one copy of the classifier constants. cocotb is deliberately not imported here.

The one value that must never be hard-coded is the feature centre. The trainer
subtracts the training-set mean from every 4-bit band feature and writes it as
`centre=N` in the header comment (and as `localparam WW_CENTRE`); the RTL
subtracts the parameter `FEAT_OFF`. If the two differ the silicon runs a
different model from the one that was measured: an earlier build hardened
`FEAT_OFF=6` against a `centre=8` header and turned 98.9 % AUC into 81 %.
`FEAT_OFF` here therefore comes from the header, and the RTL's `A_CENTRE`
assertion checks the same equality at elaboration time.
"""

from __future__ import annotations

import os
import re

import numpy as np

HEADER_NAME = "drone_weights.svh"

# Classifier geometry the header was emitted for. test_cfg() in test.py owns
# the front-end geometry; these are the values the weight layout depends on.
NHID = 4          # hidden units: WW_ROW holds NHID x NFRAME x NBAND trits
HACC_W = 6        # hidden accumulator width: WW_HBIAS holds NHID x HACC_W bits
HSHIFT = 1        # hidden requantise shift

# LED hold, in frames -- must track HOLD_FRAMES in the RTL. `hold` is loaded in
# S_CLASS and decremented in the S_ROLL of the same frame, so the LED covers
# HOLD_FRAMES-1 whole frames.
HOLD_FRAMES = 2
HOLD_FRAMES_VISIBLE = HOLD_FRAMES - 1

_CENTRE_RE = re.compile(r"centre\s*=\s*(\d+)")
_WW_CENTRE_RE = re.compile(r"WW_CENTRE\s*=\s*(\d+)\s*;")


def header_path(src_dir: str) -> str:
    return os.path.join(src_dir, HEADER_NAME)


def read_header(src_dir: str) -> str:
    path = header_path(src_dir)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"weight header not found: {path}")
    with open(path) as f:
        return f.read()


def parse_centre(text: str) -> int:
    """The feature centre the header was trained at.

    Prefers the machine-readable `localparam WW_CENTRE = N;`, falls back to the
    `centre=N` in the generated comment, and refuses a header that has neither
    or where the two disagree.
    """
    lp = _WW_CENTRE_RE.search(text)
    cm = _CENTRE_RE.search(text)
    if lp is None and cm is None:
        raise ValueError(
            f"{HEADER_NAME} carries no centre: expected 'localparam WW_CENTRE = N;' "
            "or 'centre=N' in the header comment")
    if lp is not None and cm is not None and int(lp.group(1)) != int(cm.group(1)):
        raise ValueError(
            f"{HEADER_NAME}: WW_CENTRE={lp.group(1)} disagrees with comment "
            f"centre={cm.group(1)}")
    return int((lp or cm).group(1))


def _const(text: str, name: str) -> tuple[int, int]:
    m = re.search(name + r"\s*=\s*(\d+)'h([0-9a-fA-F]+)", text)
    if m is None:
        raise ValueError(f"{HEADER_NAME}: missing localparam {name}")
    return int(m.group(2), 16), int(m.group(1))


def _trit(code: int) -> int:
    return 1 if code == 0b01 else (-1 if code == 0b11 else 0)


def parse_weights(text: str, *, nhid: int, nframe: int, nband: int, hacc_w: int,
                  avg_n: int = 0):
    """Decode WW_ROW / WW_HBIAS / WW_W2 / WW_THRESH_PK -> (W1, HB, W2, thr).

    W1 is int64 [nhid, nframe, nfeat] in {-1, 0, +1}; HB is int64 [nhid];
    W2 is int64 [nhid] in {-1, 0, +1}; thr is a signed int. A WW_ROW row holds
    NFEAT = NBAND + AVG_N features (the RTL's NFEAT: every band maximum, then
    the AVG_N frame means), so `avg_n` must match the RTL or every row is
    sliced at the wrong stride.
    """
    nfeat = nband + avg_n
    v, row_w = _const(text, "WW_ROW")
    if row_w != 2 * nhid * nframe * nfeat:
        raise ValueError(
            f"{HEADER_NAME}: WW_ROW is {row_w} bits, expected "
            f"{2 * nhid * nframe * nfeat} for NHID={nhid} NFRAME={nframe} "
            f"NBAND={nband} AVG_N={avg_n}")
    w1 = np.zeros((nhid, nframe, nfeat), dtype=np.int64)
    for h in range(nhid):
        for f in range(nframe):
            row = (v >> (2 * nfeat * (h * nframe + f))) & ((1 << (2 * nfeat)) - 1)
            for b in range(nfeat):
                w1[h, f, b] = _trit((row >> (2 * b)) & 0b11)

    hv, hb_w = _const(text, "WW_HBIAS")
    if hb_w != hacc_w * nhid:
        raise ValueError(
            f"{HEADER_NAME}: WW_HBIAS is {hb_w} bits, expected {hacc_w * nhid}")
    hb = []
    for h in range(nhid):
        u = (hv >> (hacc_w * h)) & ((1 << hacc_w) - 1)
        hb.append(u - (1 << hacc_w) if u >> (hacc_w - 1) else u)

    wv, _ = _const(text, "WW_W2")
    w2 = [_trit((wv >> (2 * h)) & 0b11) for h in range(nhid)]

    tv, tw = _const(text, "WW_THRESH_PK")
    thr = tv - (1 << tw) if tv >> (tw - 1) else tv
    return w1, np.array(hb, dtype=np.int64), np.array(w2, dtype=np.int64), int(thr)


def load_weights(src_dir: str, *, nhid: int = NHID, nframe: int = 16, nband: int = 5,
                 hacc_w: int = HACC_W):
    """Parse src/drone_weights.svh -> (W1, HB, W2, thr)."""
    return parse_weights(read_header(src_dir), nhid=nhid, nframe=nframe,
                         nband=nband, hacc_w=hacc_w)


def load_centre(src_dir: str) -> int:
    return parse_centre(read_header(src_dir))


_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")

# The RTL's FEAT_OFF must equal this; test.py passes it to the golden model and
# the RTL asserts it against WW_CENTRE, so a header/RTL mismatch fails loudly
# at compile time instead of shipping a different model.
FEAT_OFF = load_centre(_SRC)
