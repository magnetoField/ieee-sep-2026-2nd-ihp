# cocotb testbench for tt_um_hyphen133_drone_detection.
#   make                    RTL, short frames (FRAME_LOG2=8, fast)
#   FRAME_LOG2=16 make      RTL, full-rate frames (what tapes out)
#   GATES=yes make          complete gate-level suite (several hours; the
#                           netlist is copied in by the GDS action)

SIM ?= icarus
FST ?= -fst
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = tt_um_hyphen133_drone_detection.sv

FRAME_LOG2 ?= 8

ifneq ($(GATES),yes)

# The build directory carries the frame length: cocotb only recompiles when a
# source changes, so a shared directory could silently reuse a short-frame
# build for a full-rate run.
SIM_BUILD        = sim_build/rtl_f$(FRAME_LOG2)
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
COMPILE_ARGS    += -g2012
COMPILE_ARGS    += -DWW_FRAME_LOG2=$(FRAME_LOG2)
# Design assertions (the block at the bottom of the RTL). RTL only: they read
# internal signals, and src/config.json never defines this, so nothing
# synthesised is affected. WW_ASSERT=0 turns them off.
ifneq ($(WW_ASSERT),0)
COMPILE_ARGS    += -DWW_ASSERT
endif

else

# Gate level simulation: the netlist is the full-rate build, so force the
# golden model to match regardless of what FRAME_LOG2 was passed.
FRAME_LOG2       = 16
SIM_BUILD        = sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -g2012
# The stock sg13g2 models need TinyTapeout's patched iverilog (their specify
# blocks use `ifnone` on edge paths). With a stock iverilog, point GL_CELLS at
# a copy with the specify blocks removed and delayed_* nets tied through.
GL_CELLS ?= $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_stdcell/verilog/sg13g2_stdcell.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_io/verilog/sg13g2_io.v
VERILOG_SOURCES += $(GL_CELLS)

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS    += -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test
MODULE = test

export FRAME_LOG2
include $(shell cocotb-config --makefiles)/Makefile.sim
