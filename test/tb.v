`default_nettype none
`timescale 1ns / 1ps

// Parameter overrides come in as +defines; iverilog's -P silently no-ops here.
// Defaults are the shipped RTL parameters; the Makefile shortens FRAME_LOG2
// for the fast RTL run. The gate-level netlist has no parameters, so under
// GL_TEST the design is instantiated as-is and must be the full-rate build.
`ifndef WW_FRAME_LOG2
  `define WW_FRAME_LOG2 16
`endif
// All drone geometry other than the shortened RTL frame is left at the exact
// tape-out defaults.

/* Testbench wrapper for tt_um_hyphen133_drone_detection. */
module tb ();

  // Waveforms for the RTL runs only: dumping every net of the gate-level
  // netlist at the full frame length writes tens of GB and dominates runtime.
`ifndef GL_TEST
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end
`endif

  reg        clk;
  reg        rst_n;
  reg        ena;
  reg  [7:0] ui_in;
  reg  [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

`ifdef GL_TEST
  // The IHP flow emits an unpowered netlist (no VPWR/VGND ports).
  tt_um_hyphen133_drone_detection user_project (
`else
  tt_um_hyphen133_drone_detection #(
      .FRAME_LOG2(`WW_FRAME_LOG2)
  ) user_project (
`endif
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

endmodule
