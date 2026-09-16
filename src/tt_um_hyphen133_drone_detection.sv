// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: Apache-2.0
//
// tt_um_hyphen133_drone_detection -- self-contained acoustic drone detector
// for a TinyTapeout IHP 1x1 tile. No host, memory, or weights to load.
//
//   PDM microphone (1 bit)  ->  [ this chip ]  ->  LED
//
// Signal chain, integer throughout, and with no multiplier anywhere:
//
//   1. Generate the mic clock, CLK / PDM_DIV, and sample one bit per period.
//   2. Dyadic 1-pole cascade. stage[b] += (in - stage[b]) >>> K_SHIFT, with
//      stage b clocked once every 2^b mic ticks -- decimate-by-two per octave,
//      so one shared shift-and-add serves the whole filterbank.
//   3. band[b] = stage[b-1] - stage[b], an octave band-pass.
//   4. feat = log2|band| from a priority encoder plus MANT mantissa bits.
//      The log is free: it is the encoder's output, not a computation.
//   5. Per-band maximum over a frame of 2^FRAME_LOG2 ticks.
//   6. NHID hidden units, each an NFRAME x NBAND ternary template accumulated
//      frame by frame, then clamp(acc >> HSHIFT, 0, 15). A purely linear
//      template caps out near 84 % AUC on these features; one hidden layer
//      reaches the required capacity. NPHASE staggered copies mean detection
//      does not depend on alignment with a window boundary.
//   7. Output layer: one ternary weight per hidden unit; over threshold ->
//      hold the LED for HOLD_FRAMES-1 frames. A drone is a steady source
//      that is still there on the next window, so the output tracks it at
//      41.9 ms rather than latching: at the old 16 the LED lagged the
//      aircraft by 629 ms and ran two passes together into one.
//
// The fixed template weights come from drone_weights.svh (drone_dads_lvl12_s4:
// trained across a -12..0 dB level span, 98.98 % test AUC). Ternary weights
// cost nothing here: a zero drops that term from the adder tree.

`default_nettype none

module tt_um_hyphen133_drone_detection #(
    parameter PDM_DIV_LOG2 = 5,     // mic clock = clk / 2^PDM_DIV_LOG2
    parameter NSTAGE       = 9,    // cascade depth
    parameter K_SHIFT      = 2,     // 1-pole coefficient
    parameter STATE_W      = 10,    // signed cascade state
    // drone_2/drone_4 geometry: five octave bands and a 671 ms integration window.
    // This exact configuration completed the IHP sg13g2 1x1 flow at 94.60%
    // final core utilization with clean DRC, LVS, antenna, and timing checks.
    parameter TAP0         = 4,     // stages 3..8, keeping the lowest band
    parameter NBAND        = 5,
    parameter FRAME_LOG2   = 16,    // 65_536 mic ticks = 41.9 ms
    parameter NFRAME       = 16,    // 671 ms of integration under one window
    parameter MANT         = 1,     // mantissa bits in the log -> 3 dB steps
    parameter FEAT_W       = 4,
    // How many bands also keep a frame *mean* beside their frame maximum, out
    // of NBAND, counted from the deepest tap -- i.e. the AVG_N lowest-frequency
    // bands. 0 is what has always shipped: the maximum alone.
    //
    // The mean is what the maximum throws away, and on the eight detectors in
    // docs/task_optimization.md it is worth up to +7 test AUC, more than every
    // cascade change measured combined. It is also where the area goes, one
    // AVG_W accumulator per averaged band, so AVG_N is a parameter rather than
    // a flag: the mean of the three lowest bands captures most of the gain of
    // averaging all six, for half the accumulators.
    //
    // An exact mean would need a per-band divisor: band b ticks 2^(FRAME_LOG2-b)
    // times per frame, 8192 at band 3 against 256 at band 8. So sample instead
    // of averaging everything -- take 2^AVG_SHIFT samples per band per frame,
    // the same count for every band, and the divide is a constant >> AVG_SHIFT.
    //
    // The sampling instant is the same for every band, which is what makes
    // this nearly free: band b is due when cnt[b-1:0] is zero, and "every
    // 2^(FRAME_LOG2-b-AVG_SHIFT)-th tick of band b" reduces to
    // cnt[FRAME_LOG2-AVG_SHIFT-1:0] == 0 for all of them. One AND over counter
    // bits that already exist, shared across the ring.
    //
    // A leaky integrator was tried first and is not good enough: it averages
    // over ~2^K ticks rather than over the frame, and at the low bands that is
    // a small fraction of one frame. It recovers about a third of the gain.
    parameter AVG_N        = 0,
    parameter AVG_SHIFT    = 6,     // log2 of the samples per band per frame
    parameter NPHASE       = 2,     // staggered windows, hop = NFRAME/NPHASE
    parameter NHID         = 4,     // hidden units; 1 == the old linear template
    parameter HACC_W       = 6,     // saturating hidden accumulator
    parameter HSHIFT       = 1,     // hidden requantise: clamp(acc>>HSHIFT,0,15)
    parameter FEAT_OFF     = 8,     // constant subtracted from each band
                                    // feature before the adder tree; keeps
                                    // the accumulator and bias small. MUST
                                    // equal the header's centre (WW_CENTRE):
                                    // the trainer centred the features on
                                    // this value, and a mismatch runs a
                                    // different model from the one measured
                                    // (drone_4: FEAT_OFF 6 vs centre 8 turned
                                    // 98.9 % AUC into 81 %). Checked by
                                    // A_CENTRE below.
    parameter SCORE_W      = 10,
    // LED hold. `hold` is loaded in S_CLASS and decremented in the S_ROLL of
    // that same frame, so the LED stays up for HOLD_FRAMES-1 whole frames:
    // 2 is one frame, 41.9 ms. 1 is not the floor it looks like -- the load
    // and the decrement fall in the same frame, so it would blink for the few
    // hundred clocks of S_CLASS and never be seen.
    parameter HOLD_FRAMES  = 2,
    parameter DEBUG_PINS   = 1      // 0: uo_out[7:4] and uio_out driven low
) (
    input  wire [7:0] ui_in,    // [0] PDM data in, [7:1] threshold trim
    output wire [7:0] uo_out,   // [0] PDM clock out, [3:1] detections, [7:4] debug
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // Fixed weights. Keeping one header makes the submitted build
  // independent of command-line defines used by the original multi-model repo.
  `include "drone_weights.svh"

  localparam IN_AMP   = 1 << (STATE_W - 3);
  localparam FEAT_MAX = (1 << FEAT_W) - 1;
  // Features the template reads per frame: every band's maximum, then the
  // AVG_N means. AVG_W is one accumulator: 2^AVG_SHIFT samples of at most
  // FEAT_MAX cannot overflow FEAT_W+AVG_SHIFT bits. NAVG is 1 rather than 0
  // when AVG_N=0 because a zero-length array is not portable; the ring is
  // unread in that case and synthesis removes it.
  localparam NFEAT    = NBAND + AVG_N;
  localparam AVG_W    = FEAT_W + AVG_SHIFT;
  localparam NAVG     = (AVG_N > 0) ? AVG_N : 1;
  // The averaged bands are the last AVG_N the tap loop visits, so the ring
  // rotates only during those steps and is back in order for the classifier.
  localparam AVG_STG0 = TAP0 + NBAND - AVG_N;
  localparam FIDX_W   = $clog2(NFRAME);
  // Width of the LED hold counter. Sized from HOLD_FRAMES and NOT from FIDX_W,
  // which it used to share. That coupling held only by luck: FIDX_W+1 is 5
  // bits at this build's NFRAME=16, so 5'(16) fitted -- but the same source
  // with NFRAME=8 gives a 4-bit counter, 4'(16) truncates to zero and the LED
  // never asserts at all. See docs/hold_width.md.
  localparam HOLD_W   = $clog2(HOLD_FRAMES + 1);
  localparam HOP      = NFRAME / NPHASE;
  localparam CNT_W    = FRAME_LOG2 + FIDX_W;
  localparam STG_W    = $clog2(NSTAGE + 1);
  localparam NSLOT    = NPHASE * NHID;
  // NPHASE and NHID are powers of two, so the classifier step index splits
  // into {phase, hidden unit} by bit slicing instead of a divider.
  localparam PH_W     = (NPHASE <= 1) ? 1 : $clog2(NPHASE);
  localparam HD_W     = (NHID   <= 1) ? 1 : $clog2(NHID);
  localparam SLOT_W   = PH_W + HD_W;
  localparam BAND_W   = STATE_W + 1;
  localparam MANT_W   = (MANT < 1) ? 1 : MANT;   // avoid a [-1:0] vector

  // ---------------------------------------------------------------------
  // Microphone clock and input sampling
  // ---------------------------------------------------------------------
  logic [PDM_DIV_LOG2-1:0] div;
  logic                    pdm_bit;
  wire                     tick = (div == {{(PDM_DIV_LOG2-1){1'b0}}, 1'b1});

  // Async reset throughout: sg13g2 has no reset-less flop, so a synchronous
  // reset costs a tie-high cell on every RESET_B pin plus a reset mux in
  // front of every D input (~3 000 um^2 here). TinyTapeout deasserts rst_n
  // synchronously to clk, so the async form is safe.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      div     <= '0;
      pdm_bit <= 1'b0;
    end else begin
      div <= div + 1'b1;
      // Sample mid-way through the high phase of the emitted mic clock.
      if (div == {2'b11, {(PDM_DIV_LOG2-2){1'b0}}}) pdm_bit <= ui_in[0];
    end
  end

  assign uo_out[0] = div[PDM_DIV_LOG2-1];   // mic clock

  // ---------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------
  // Cascade states live in a ROTATING shift register, not an addressed array.
  // Every mic tick rotates all NSTAGE words past a single compute slot, so the
  // NSTAGE:1 read mux and the 1:NSTAGE write demux both disappear; the price
  // is that the cascade always takes NSTAGE clocks instead of only the due ones.
  logic signed [STATE_W-1:0] ring [NSTAGE];
  // Per-band frame maxima, also a ROTATING ring: the tap stages visit the
  // bands in order 0..NBAND-1 on every tick, so the band being updated is
  // always fmax[0] and the new value goes to the tail; after the NBAND tap
  // steps the ring is back in band order for the classifier's parallel read.
  logic        [FEAT_W-1:0]  fmax  [NBAND];
  // Per-band frame-mean accumulator, rotated in lockstep with fmax so the band
  // under update is always at the head, and cleared with it at the frame
  // boundary. It holds the sum of 2^AVG_SHIFT samples, so the mean is the top
  // FEAT_W bits.
  logic        [AVG_W-1:0]   favg  [NAVG];
  // Hidden accumulators, one per (phase, unit), also kept as a ROTATING ring:
  // S_CLASS visits the NSLOT slots in a fixed order every frame, so the
  // current slot's accumulator is always hacc[0] and the result goes to the
  // tail. No NSLOT:1 read mux, no 1:NSLOT write demux.
  logic signed [HACC_W-1:0]  hacc  [NSLOT];
  // Output-layer sum: NHID terms of at most 15 each, so it needs only
  // OSUM_W bits; the compare against the SCORE_W-bit threshold sign-extends.
  localparam OSUM_W = $clog2(NHID * 15 + 1) + 1;
  logic signed [OSUM_W-1:0]  osum;
  logic        [HOLD_W-1:0]  hold;

  logic [CNT_W-1:0]  cnt;          // mic-tick counter: framing + decimation
  logic [STG_W-1:0]  stg;          // cascade step
  logic [SLOT_W-1:0] slot;         // classifier step = {phase, word}
  logic [1:0]        st;
  localparam [1:0] S_IDLE = 2'd0, S_CASC = 2'd1, S_CLASS = 2'd2, S_ROLL = 2'd3;

  wire [FIDX_W-1:0] frame_idx = cnt[CNT_W-1:FRAME_LOG2];
  wire              frame_end = &cnt[FRAME_LOG2-1:0];

  // ---------------------------------------------------------------------
  // Cascade datapath -- one shared subtract-shift-add
  // ---------------------------------------------------------------------
  // No size casts here on purpose: yosys reads `-STATE_W'(IN_AMP)` as a cast
  // with a negated width and produces +IN_AMP, i.e. a rectified microphone.
  // iverilog and the Python model give -IN_AMP. Caught by gate-level sim.
  localparam signed [STATE_W-1:0] X_POS = IN_AMP;
  localparam signed [STATE_W-1:0] X_NEG = -IN_AMP;
  wire signed [STATE_W-1:0] x_in = pdm_bit ? X_POS : X_NEG;
  // The previous stage's output is whatever the last rotation wrote to the
  // ring tail: casc_nx if that stage was due, else its unchanged state. When
  // it was not due, this stage is not due either and casc_in is dead, so no
  // separate prev_v register is needed.
  wire signed [STATE_W-1:0] casc_in = (stg == 0) ? x_in : ring[NSTAGE-1];
  wire signed [STATE_W-1:0] casc_st = ring[0];              // head of the ring
  wire signed [STATE_W-1:0] casc_nx = casc_st + ((casc_in - casc_st) >>> K_SHIFT);

  // Stage b is idle unless the low b bits of the tick counter are zero.
  // Once a stage is idle every deeper stage is too, so the ring still rotates
  // but stops updating.
  wire casc_due = ((cnt & ((1 << stg) - 1)) == 0);
  wire casc_last = (stg == STG_W'(NSTAGE - 1));

  wire is_tap  = (stg >= STG_W'(TAP0)) && (stg < STG_W'(TAP0 + NBAND));

  // Explicit sign extension everywhere a signed value is widened: yosys
  // zero-extends `N'(signed_expr)` where the LRM (and iverilog, and the
  // Python model) sign-extend. Found by simulating the netlist.
  wire signed [BAND_W-1:0] casc_in_w = {casc_in[STATE_W-1], casc_in};
  wire signed [BAND_W-1:0] casc_nx_w = {casc_nx[STATE_W-1], casc_nx};
  wire signed [BAND_W-1:0] band = casc_in_w - casc_nx_w;
  wire        [BAND_W-2:0] bmag = band[BAND_W-1] ? (~band[BAND_W-2:0] + 1'b1)
                                                 : band[BAND_W-2:0];

  // Priority encoder: exp = index of the most significant set bit, +1.
  logic [4:0] bexp;
  always_comb begin
    bexp = 5'd0;
    for (int i = 0; i < BAND_W-1; i++) if (bmag[i]) bexp = 5'(i + 1);
  end

  // log2 with MANT mantissa bits below the leading one.
  logic [FEAT_W-1:0] feat;
  always_comb begin
    logic [4:0] sh;
    logic [MANT_W-1:0] mbits;
    logic [8:0] wide;
    sh    = 5'd0;
    mbits = '0;
    wide  = 9'd0;
    if (MANT == 0) begin
      wide = 9'(bexp);
    end else if (bexp <= 5'(MANT)) begin
      wide = 9'(bexp);
    end else begin
      sh    = bexp - 5'(1 + MANT);
      mbits = MANT_W'(bmag >> sh);
      wide  = (9'(bexp - 5'(MANT)) << MANT) | 9'(mbits);
    end
    feat = (wide > 9'(FEAT_MAX)) ? FEAT_W'(FEAT_MAX) : FEAT_W'(wide);
  end

  // Frame-mean sampling. Band b is due when cnt[b-1:0] is zero, and taking
  // every 2^(FRAME_LOG2-b-AVG_SHIFT)-th tick of band b reduces to the same
  // test for every band: the low FRAME_LOG2-AVG_SHIFT bits of the frame
  // counter are zero. So one AND over counter bits that already exist gates
  // the whole ring, and every band contributes exactly 2^AVG_SHIFT samples.
  //
  // This holds only while every tap is due at those instants, i.e. while
  // TAP0+NBAND-1 <= FRAME_LOG2-AVG_SHIFT. At FRAME_LOG2=16, AVG_SHIFT=6 that
  // is band 10, and the deepest tap in any build here is 8.
  localparam SAMP_W = FRAME_LOG2 - AVG_SHIFT;
  wire avg_due = (cnt[SAMP_W-1:0] == '0);
  // The accumulator cannot overflow: 2^AVG_SHIFT samples of at most FEAT_MAX
  // sum to less than 2^(FEAT_W+AVG_SHIFT) = 2^AVG_W.
  wire [AVG_W-1:0] avg_nx = favg[0] + AVG_W'(feat);

  // ---------------------------------------------------------------------
  // Classifier -- one shared ternary adder tree, time-multiplexed over
  // NPHASE x NHID accumulators once per frame.
  // ---------------------------------------------------------------------
  wire [PH_W-1:0]   c_ph   = slot[SLOT_W-1 -: PH_W];
  wire [HD_W-1:0]   c_hd   = slot[HD_W-1:0];
  wire [FIDX_W-1:0] c_slot = frame_idx - FIDX_W'(c_ph * HOP);
  wire              win_end = (c_slot == FIDX_W'(NFRAME-1));
  wire              last_h  = (c_hd == HD_W'(NHID-1));

  // 2 bits per weight: 01 = +1, 11 = -1, else 0. WW_ROW packs one
  // (hidden unit, frame slot) row of NBAND weights; see drone_weights.svh.
  wire [$clog2(NHID*NFRAME)-1:0] wsel = ($clog2(NHID*NFRAME))'(c_hd*NFRAME + c_slot);
  wire [2*NFEAT-1:0] wrow = WW_ROW[2*NFEAT*wsel +: 2*NFEAT];

  // Feature b of the row: every band's maximum first, in band order, then the
  // AVG_N means, also in band order -- [max0..max(NBAND-1), avg of band
  // NBAND-AVG_N .. avg of band NBAND-1]. Changing this order would silently
  // mis-decode every weight. A mean is read as
  // the top FEAT_W bits of its accumulator, which is the divide by
  // 2^AVG_SHIFT.
  //
  // The dot product is exact; only the accumulator saturates. That is the
  // arithmetic the trainer, eval_header.py and the golden model all use, so
  // the dot needs its own width: NFEAT centred features of up to 2^FEAT_W-1
  // each. At HACC_W=6 the five-band sum reaches +-40, and an earlier build
  // that held it in HACC_W bits wrapped it -- silence (every feature 0,
  // centred to -8) through a row of five -1 weights gave -24 instead of +40,
  // so the silicon ran a different model from the measured one whenever
  // |dot| > 31. test_detector_matches_model caught it on the lvl12 weights.
  localparam DOT_W = FEAT_W + $clog2(NFEAT + 1) + 1;
  localparam ACW_W = (DOT_W > HACC_W ? DOT_W : HACC_W) + 1;
  logic signed [DOT_W-1:0] dot;
  always_comb begin
    logic signed [DOT_W-1:0] fc;
    logic        [FEAT_W-1:0] fv;
    dot = '0;
    for (int b = 0; b < NFEAT; b++) begin
      logic [1:0] w2;
      w2 = wrow[2*b +: 2];
      fv = (b < NBAND) ? fmax[b]
                       : FEAT_W'(favg[b - NBAND][AVG_W-1 -: FEAT_W]);
      fc = DOT_W'($signed({1'b0, fv})) - DOT_W'(FEAT_OFF);
      if (w2[0]) dot = w2[1] ? dot - fc : dot + fc;
    end
  end

  // At slot 0 the accumulator resets to a hard-wired per-unit constant. That
  // constant carries both the learned bias and the feature-centring offset,
  // so centring costs nothing in silicon.
  wire signed [HACC_W-1:0] hb = $signed(WW_HBIAS[HACC_W*c_hd +: HACC_W]);
  wire signed [HACC_W-1:0] acc_cur = (c_slot == '0) ? hb : hacc[0];
  // Sign-extend both operands to ACW_W bits so acc_cur + dot cannot wrap
  // before the saturation compare below.
  wire signed [ACW_W-1:0]  acc_wide =
      $signed({{(ACW_W-HACC_W){acc_cur[HACC_W-1]}}, acc_cur}) +
      $signed({{(ACW_W-DOT_W){dot[DOT_W-1]}}, dot});
  localparam signed [ACW_W-1:0] HA_MAX =  (1 << (HACC_W-1)) - 1;
  localparam signed [ACW_W-1:0] HA_MIN = -(1 << (HACC_W-1));
  wire signed [HACC_W-1:0] acc_next =
      (acc_wide >  HA_MAX) ? HACC_W'(HA_MAX) :
      (acc_wide <  HA_MIN) ? HACC_W'(HA_MIN) : HACC_W'(acc_wide);

  // Hidden activation: clamp(acc >> HSHIFT, 0, 15) -- ReLU is free again.
  wire signed [HACC_W-1:0] hsh = acc_next >>> HSHIFT;
  wire [3:0] hval = acc_next[HACC_W-1]        ? 4'd0  :
                    (|hsh[HACC_W-1:4])        ? 4'd15 : hsh[3:0];

  // Output layer: one ternary constant per hidden unit, folded into the same
  // loop, so the window closes on the step that visits the last hidden unit.
  wire [1:0] w2c = WW_W2[2*c_hd +: 2];
  wire signed [OSUM_W-1:0] hval_s = OSUM_W'($signed({1'b0, hval}));
  wire signed [OSUM_W-1:0] o_term =
      w2c[0] ? (w2c[1] ? (~hval_s + 1'b1) : hval_s) : '0;
  wire signed [OSUM_W-1:0] osum_cur  = (c_hd == '0) ? '0 : osum;
  wire signed [OSUM_W-1:0] osum_next = osum_cur + o_term;

  // Threshold: the trained constant, trimmed by the board's DIP switches.
  // trim = (ui_in[7:1] - 64) * 4, range -256..+252, as a SCORE_W-bit signed value.
  wire signed [7:0]         trim8  = $signed({1'b0, ui_in[7:1]}) - 8'sd64;
  wire signed [SCORE_W-1:0] trim   = {{(SCORE_W-8){trim8[7]}}, trim8} <<< 2;
  wire signed [SCORE_W-1:0] thresh = $signed(WW_THRESH_PK) + trim;
  wire signed [SCORE_W-1:0] osum_w = {{(SCORE_W-OSUM_W){osum_next[OSUM_W-1]}}, osum_next};
  wire fire = win_end && last_h && (osum_w > thresh);

  // ---------------------------------------------------------------------
  // Sequencer
  // ---------------------------------------------------------------------
  integer i;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st   <= S_IDLE;
      cnt  <= '0;
      stg  <= '0;
      slot <= '0;
      for (i = 0; i < NSTAGE; i++) ring[i] <= '0;
      for (i = 0; i < NBAND;  i++) fmax[i]  <= '0;
      for (i = 0; i < NAVG;   i++) favg[i]  <= '0;
      for (i = 0; i < NSLOT;  i++) hacc[i]  <= '0;
      osum <= '0;
      hold <= '0;
    end else begin
      case (st)
        S_IDLE: if (tick) begin
          stg <= '0;
          st  <= S_CASC;
        end

        S_CASC: begin
          // Rotate one position; update the head only if this stage is due.
          for (i = 0; i < NSTAGE - 1; i++) ring[i] <= ring[i + 1];
          ring[NSTAGE-1] <= casc_due ? casc_nx : casc_st;
          // Tap stages rotate the fmax ring once each; the band under
          // update is at the head and its new maximum goes to the tail.
          if (is_tap) begin
            for (i = 0; i < NBAND - 1; i++) fmax[i] <= fmax[i + 1];
            fmax[NBAND-1] <= (casc_due && (feat > fmax[0])) ? feat : fmax[0];
          end
          if (AVG_N > 0 && stg >= STG_W'(AVG_STG0)
                        && stg < STG_W'(TAP0 + NBAND)) begin
            for (i = 0; i < NAVG - 1; i++) favg[i] <= favg[i + 1];
            favg[NAVG-1] <= (casc_due && avg_due) ? avg_nx : favg[0];
          end
          if (casc_last) begin
            if (frame_end) begin
              slot <= '0;
              st   <= S_CLASS;
            end else begin
              cnt <= cnt + 1'b1;
              st  <= S_IDLE;
            end
          end else begin
            stg <= stg + 1'b1;
          end
        end

        S_CLASS: begin
          // Rotate the accumulator ring; the slot just evaluated goes to
          // the tail so that after NSLOT steps the order is restored.
          for (i = 0; i < NSLOT - 1; i++) hacc[i] <= hacc[i + 1];
          hacc[NSLOT-1] <= acc_next;
          if (win_end) osum <= osum_next;
          if (fire) hold <= HOLD_W'(HOLD_FRAMES);
          if (slot == SLOT_W'(NSLOT - 1)) st <= S_ROLL;
          else                            slot <= slot + 1'b1;
        end

        S_ROLL: begin
          for (i = 0; i < NBAND; i++) fmax[i] <= '0;
          // The mean accumulator is a per-frame statistic like the max, so it
          // clears with it. (The leaky integrator this replaced did not, which
          // is part of why it measured the wrong thing.)
          if (AVG_N > 0) for (i = 0; i < NAVG; i++) favg[i] <= '0;
          if (hold != 0) hold <= hold - 1'b1;
          cnt <= cnt + 1'b1;
          st  <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------
  wire detect = (hold != 0);
  assign uo_out[1] = detect;
  assign uo_out[2] = detect;
  assign uo_out[3] = detect;                        // the LED
  generate
    if (DEBUG_PINS != 0) begin : g_dbg
      assign uo_out[7:4]  = fmax[0][FEAT_W-1 -: 4];  // scope hook: top band level
      assign uio_out[3:0] = 4'(frame_idx);
      assign uio_out[6:4] = {st, |detect};
      assign uio_out[7]   = tick;
      assign uio_oe       = 8'hFF;
    end else begin : g_nodbg
      assign uo_out[7:4]  = 4'b0;
      assign uio_out      = 8'b0;
      assign uio_oe       = 8'b0;
    end
  endgenerate

  wire _unused = &{ena, uio_in, 1'b0};

  // ---------------------------------------------------------------------
  // Assertions
  //
  // Enabled by -DWW_ASSERT, which test/Makefile sets on every RTL build and
  // src/config.json never sets, so this block does not reach synthesis --
  // verified by re-running the flow and diffing the netlist, which came out
  // byte-identical. These
  // are immediate assertions on purpose: iverilog rejects `property`,
  // `assert property` and `bind` outright, so concurrent SVA would mean
  // moving the whole suite to another simulator.
  //
  // The point of putting them here rather than writing more testbenches is
  // that they hold under *every* stimulus. All seven cocotb tests, both
  // weight builds and both frame lengths become scenario coverage for them,
  // and so does anything added later.
  //
  // What this is a reaction to: the wake-word sibling of this design shipped
  // a hardened part whose hold counter was too narrow for HOLD_FRAMES, so its
  // LED could never assert. This build escaped it only because NFRAME=16
  // happens to make FIDX_W+1 wide enough. A
  // -Wall lint pass (measured, verilator 5.052) does not catch it -- the
  // truncating cast is explicit, which silences WIDTHTRUNC -- and neither does
  // comparing against the golden model in a run that never fires. It is a
  // value invariant, and A_HOLD_FITS below is the check it needed.
  // ---------------------------------------------------------------------
`ifdef WW_ASSERT
  // Elaboration time: parameters that must survive being loaded into the
  // registers that hold them, and the arithmetic each one assumes.
  // iverilog rejects a label on an immediate assertion, so each check carries
  // its name in the failure message instead.
  initial begin
    // A_HOLD_FITS. This is the one that bug needed.
    // Measured against $bits(hold) -- the width the register was actually
    // declared with -- not against HOLD_W. Checking the localparam would pass
    // happily if the declaration were ever wired to some other parameter
    // again, which is precisely the bug this is here to stop.
    //
    // Stated as a range rather than a round-trip cast on purpose: iverilog
    // evaluates HOLD_W'(HOLD_FRAMES) as -2 in an expression context (it
    // assigns correctly, so the RTL above is fine), the same family of
    // width-cast bug as the yosys -N'(x) one that rectified the mic input.
    assert (HOLD_FRAMES < (1 << $bits(hold)))
      else $fatal(1, "A_HOLD_FITS: HOLD_FRAMES=%0d needs more than the %0d bits hold has",
                  HOLD_FRAMES, $bits(hold));
    // A_HOLD_VISIBLE. 1 loads and decrements in the same frame, so it would
    // blink for the few hundred clocks of S_CLASS and never be seen.
    assert (HOLD_FRAMES >= 2)
      else $fatal(1, "A_HOLD_VISIBLE: HOLD_FRAMES=%0d gives no visible output",
                  HOLD_FRAMES);
    // A_NFRAME_POW2. c_slot subtracts the phase offset in FIDX_W bits and
    // relies on the wrap, which is modulo NFRAME only for a power of two.
    assert (NFRAME == (1 << FIDX_W))
      else $fatal(1, "A_NFRAME_POW2: NFRAME=%0d is not a power of two", NFRAME);
    // A_HOP_EXACT. HOP = NFRAME/NPHASE must be exact or the phases drift.
    assert (NFRAME % NPHASE == 0)
      else $fatal(1, "A_HOP_EXACT: NPHASE=%0d does not divide NFRAME=%0d",
                  NPHASE, NFRAME);
    // A_BANDS_EXIST. Every band must be a real cascade tap.
    assert (TAP0 + NBAND <= NSTAGE)
      else $fatal(1, "A_BANDS_EXIST: TAP0+NBAND=%0d exceeds NSTAGE=%0d",
                  TAP0 + NBAND, NSTAGE);
    // A_FEAT_OFF. Subtracted from every feature before the adder tree.
    assert (FEAT_OFF < (1 << FEAT_W))
      else $fatal(1, "A_FEAT_OFF: FEAT_OFF=%0d exceeds the feature range",
                  FEAT_OFF);
    // A_CENTRE. The header was trained with its features centred on WW_CENTRE;
    // the silicon subtracts FEAT_OFF. Unequal means a different model ships.
    assert (FEAT_OFF == WW_CENTRE)
      else $fatal(1, "A_CENTRE: FEAT_OFF=%0d but the weight header was trained at centre=%0d",
                  FEAT_OFF, WW_CENTRE);
  end

  logic fire_q;
  logic [1:0] st_q;
  logic [HOLD_W-1:0] hold_q;
  always_ff @(posedge clk) begin
    fire_q <= rst_n && fire && (st == S_CLASS);
    st_q   <= rst_n ? st : S_IDLE;
    hold_q <= rst_n ? hold : '0;
  end

  always @(posedge clk) if (rst_n) begin
    // A_FIRE_LOADS_HOLD. A fire must light the LED with the whole hold
    // loaded, which is the runtime form of A_HOLD_FITS. Checked a cycle late
    // because `hold` is a register; at most one fire lands per frame (the two
    // phases reach win_end in different frames), so this never races itself.
    assert (!fire_q || hold == HOLD_FRAMES)
      else $error("A_FIRE_LOADS_HOLD: fire left hold=%0d, expected %0d",
                  hold, HOLD_FRAMES);
    // A_HOLD_BOUND. HOLD_W rounds up, so the counter has room to hold values
    // above HOLD_FRAMES and this can genuinely fail: at HOLD_FRAMES=2 the
    // register reaches 3.
    assert (hold <= HOLD_FRAMES)
      else $error("A_HOLD_BOUND: hold=%0d above HOLD_FRAMES=%0d",
                  hold, HOLD_FRAMES);
    // A_HOLD_STEP. Per cycle the only legal moves are load, decrement by one,
    // and unchanged -- unchanged is legal because `hold` only ever moves in
    // S_ROLL, so it sits still for the rest of the frame.
    assert (hold == hold_q || hold == HOLD_FRAMES
                           || (hold_q != 0 && hold == hold_q - 1'b1))
      else $error("A_HOLD_STEP: hold %0d -> %0d", hold_q, hold);
    // A_HOLD_EXPIRES. That leniency is not enough on its own: "unchanged"
    // also covers a counter stuck on forever, which is a permanently lit LED
    // and the one failure a user of the board would actually see. So pin the
    // frame boundary too -- after an S_ROLL cycle a nonzero hold must have
    // come down by exactly one. Nothing loads `hold` in S_ROLL (a fire is in
    // S_CLASS), so there is no legal exception.
    //
    // scripts/assert_mutations.sh found this gap: a `hold <= hold` mutation in
    // the S_ROLL arm passed A_HOLD_STEP cleanly.
    assert (st_q != S_ROLL || hold_q == 0 || hold == hold_q - 1'b1)
      else $error("A_HOLD_EXPIRES: S_ROLL left hold %0d -> %0d", hold_q, hold);
    // A_STG_BOUND. STG_W rounds up past NSTAGE, so running off the end of the
    // cascade is representable.
    assert (stg <= NSTAGE)
      else $error("A_STG_BOUND: stg=%0d past NSTAGE=%0d", stg, NSTAGE);
    // A_HVAL_SIGN. The requantise is clamp(acc >> HSHIFT, 0, 15); a negative
    // accumulator must come out as 0, not as a wrapped positive. This is the
    // sign handling the golden model assumes.
    assert (!acc_next[HACC_W-1] || hval == 4'd0)
      else $error("A_HVAL_SIGN: acc_next=%0d gave hval=%0d", acc_next, hval);
    // A_ROLL_TO_IDLE. S_ROLL clears the frame statistics and is always
    // followed by S_IDLE; a transition anywhere else would carry one frame's
    // maxima into the next.
    assert (st_q != S_ROLL || st == S_IDLE)
      else $error("A_ROLL_TO_IDLE: S_ROLL went to %0d, not S_IDLE", st);
    // A_NO_X_OUT. Nothing observable may be X once reset is released. Written
    // as an XOR-reduction identity test because iverilog's $isunknown returns
    // 1 for any concatenation, even an all-zero one -- it is only correct on a
    // single signal.
    assert ((^{uo_out, uio_out, uio_oe}) !== 1'bx)
      else $error("A_NO_X_OUT: uo=%b uio=%b oe=%b", uo_out, uio_out, uio_oe);
  end
`endif

endmodule

`default_nettype wire
