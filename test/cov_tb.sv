// SPDX-License-Identifier: Apache-2.0
//
// Line and toggle coverage harness for tt_um_hyphen133_drone_detection, built with
//   $ verilator --binary --coverage
// and driven by scripts/coverage.sh.
//
// What this measures, and what it does not: verilator is not in the cocotb
// image and cocotb's verilator backend needs it at build time, so this cannot
// instrument the cocotb suite itself. It drives the same *classes* of stimulus
// that suite uses -- a chirp across every band, both mic rails, full-scale
// alternation, the trim range, and a mid-frame reset -- and reports which
// lines and bits of the design nothing reaches. Read the result as "is any
// logic unexercised", not as a coverage figure for test.py.
//
// The stimulus is generated here rather than read from a file so the harness
// has no Python dependency. It does not need to be bit-identical to
// make_pdm(): coverage cares about which logic runs, and a chirp sweeps every
// cascade tap, which a fixed tone set does not.
//
// The harness itself is excluded from the count. Its 32-bit ints and the
// generator's branches are not the design, and left in they moved the
// headline both ways: 170 toggle points of which 93 never flipped, and 31
// always-run lines. The verilator_coverage summary is the DUT alone.

`default_nettype none
`timescale 1ns / 1ps

// verilator coverage_off
module cov_tb;

  localparam int FL    = 8;                  // short frames; coverage, not accuracy
  localparam int TICK  = 32;                 // clocks per mic period (PDM_DIV)
  localparam int FRAME = (1 << FL) * TICK;

  logic       clk = 0;
  logic       rst_n = 0;
  logic       ena = 1;
  logic [7:0] ui_in = 0;
  logic [7:0] uio_in = 0;
  wire  [7:0] uo_out, uio_out, uio_oe;

  tt_um_hyphen133_drone_detection #(.FRAME_LOG2(FL)) dut (
      .ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
      .uio_out(uio_out), .uio_oe(uio_oe),
      .ena(ena), .clk(clk), .rst_n(rst_n)
  );

  always #10 clk = ~clk;                     // 50 MHz, matching test.py

  // -------------------------------------------------------------------
  // First-order sigma-delta encoder, so a signal becomes a 1-bit stream the
  // way the real mic produces one.
  // -------------------------------------------------------------------
  localparam int FS = 1 << 14;               // full scale
  int sd_acc = 0;

  function automatic logic pdm(input int sample);
    sd_acc = sd_acc + sample - (sd_acc >= 0 ? FS : -FS);
    return sd_acc < 0;
  endfunction

  // Chirp: doubles its period every `octave_ticks`, so it walks down through
  // every cascade tap instead of sitting in one band.
  int phase = 0, per = 4, per_ctr = 0, sq = 1;

  function automatic int chirp(input int octave_ticks);
    phase = phase + 1;
    if (phase >= per) begin
      phase = 0;
      sq = -sq;
    end
    per_ctr = per_ctr + 1;
    if (per_ctr >= octave_ticks) begin
      per_ctr = 0;
      per = (per >= 512) ? 4 : per * 2;
    end
    return sq * (FS - (FS >> 3));            // 87 % of full scale
  endfunction

  // Amplitude ramp, quiet to loud, so the log encoder walks its whole
  // exponent range. Toggle coverage showed bexp/bmag/hval_s stuck part-way
  // when every scenario held one amplitude.
  int amp_ctr = 0;
  int amp_sh = 8;

  function automatic int ramped(input int octave_ticks, input int ramp_ticks);
    int v;
    v = chirp(octave_ticks) >>> amp_sh;
    amp_ctr = amp_ctr + 1;
    if (amp_ctr >= ramp_ticks) begin
      amp_ctr = 0;
      amp_sh  = (amp_sh == 0) ? 8 : amp_sh - 1;
    end
    return v;
  endfunction

  typedef enum int { M_CHIRP, M_DC0, M_DC1, M_ALT, M_QUIET, M_RAMP } mode_e;

  // wiggle: walk uio_in and toggle ena while driving. The design must not
  // notice; test_unused_inputs_ignored is the check, this makes the harness
  // exercise the same 17 toggle points it does.
  task automatic drive(input mode_e m, input int n_ticks, input logic [6:0] trim,
                       input bit wiggle = 0);
    logic b;
    for (int i = 0; i < n_ticks; i++) begin
      case (m)
        M_CHIRP: b = pdm(chirp(1024));
        M_DC0:   b = 1'b0;
        M_DC1:   b = 1'b1;
        M_ALT:   b = i[0];                   // Nyquist: maximum slew
        M_RAMP:  b = pdm(ramped(512, 2048));
        default: b = pdm(0);                 // dither around zero
      endcase
      ui_in = {trim, b};
      if (wiggle) begin
        uio_in = i[7:0];
        ena    = ~i[4];
      end
      repeat (TICK) @(posedge clk);
    end
    uio_in = '0;
    ena    = 1'b1;
  endtask

  task automatic reset_dut;
    rst_n = 0;
    repeat (8) @(posedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);
  endtask

  logic [6:0] trims[6];

  initial begin
    // Scenario 1: the chirp at the lowest trim, long enough for both phases
    // of the staggered window to reach win_end and fire.
    reset_dut();
    drive(M_CHIRP, 48 << FL, 7'd1);

    // Scenario 2: the trim range. The high end must reach no detection, which
    // is the arm of the threshold compare the low end never takes.
    trims = '{7'd0, 7'd127, 7'd85, 7'd42, 7'd64, 7'd1};
    for (int t = 0; t < 6; t++) begin
      reset_dut();
      drive(M_CHIRP, 20 << FL, trims[t]);
    end

    // Scenario 3: both mic rails, Nyquist, and near-silence. These drive the
    // cascade state to its limits, which is where STATE_W saturation and the
    // sign of the band difference live.
    reset_dut(); drive(M_DC0,   8 << FL, 7'd1);
    reset_dut(); drive(M_DC1,   8 << FL, 7'd1);
    reset_dut(); drive(M_ALT,   8 << FL, 7'd1);
    reset_dut(); drive(M_QUIET, 8 << FL, 7'd1);

    // Scenario 3b: the amplitude ramp, which is what walks the priority
    // encoder from its floor to its ceiling.
    reset_dut(); drive(M_RAMP, 24 << FL, 7'd1);

    // Scenario 4: reset part-way through a frame, then keep running, so the
    // reset arm of every register is taken from a populated state.
    reset_dut();
    drive(M_CHIRP, 3 << FL, 7'd1);
    repeat (FRAME / 2) @(posedge clk);
    reset_dut();
    drive(M_CHIRP, 4 << FL, 7'd1);

    // Scenario 5: the unused pins. uio_in walks every bit both ways and ena
    // drops for half of every 32 ticks. These were the only reachable points
    // the first measurement left at zero.
    reset_dut();
    drive(M_CHIRP, 4 << FL, 7'd1, 1);

    $display("cov_tb: done");
    $finish;
  end

endmodule
// verilator coverage_on

`default_nettype wire
