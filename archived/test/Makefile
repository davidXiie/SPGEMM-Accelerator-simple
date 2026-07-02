#=============================================================================
# File     : Makefile
# Project  : SPGEMM-Accelerator
# Brief    : Cocotb Makefile — builds and runs SpGEMM accelerator simulation
#           Windows-compatible version (no grep/sed dependencies)
#
# Usage:
#   make OP=mul                       # Run SpGEMM (default)
#   make OP=add OP=sub                # Run SpAdd / SpSubtract
#   make SIM=icarus TESTCASE=test_quick_launch WAVES=0
#=============================================================================

# --- Waveform ---
WAVES ?= 1

# --- Simulation ---
SIM ?= icarus
TOPLEVEL_LANG = verilog

# --- DUT Top Module ---
TOPLEVEL = core_top

# --- Verilog Sources (hardcoded from filelist.f for Windows compatibility) ---
VERILOG_SOURCES = \
    ../rtl/include/defines.vh \
    ../rtl/include/isa.vh \
    ../rtl/infrastructure/axi_interface.v \
    ../rtl/infrastructure/decode.v \
    ../rtl/infrastructure/fetch.v \
    ../rtl/infrastructure/load.v \
    ../rtl/infrastructure/store.v \
    ../rtl/infrastructure/scratchpad.v \
    ../rtl/core/core_top.v \
    ../rtl/core/task_loader.v \
    ../rtl/core/pe_top.v \
    ../rtl/core/pe_decompress.v \
    ../rtl/core/pe_mul_array.v \
    ../rtl/core/pe_aggregation.v \
    ../rtl/core/sp_elementwise.v \
    ../rtl/core/c_csr_writer.v

# Add include directory explicitly
COMPILE_ARGS += -I../rtl/include

# For Icarus, use SV2012
ifeq ($(SIM),icarus)
    COMPILE_ARGS += -g2012
endif

# --- Cocotb ---
MODULE = test_dut
TESTCASE ?= test_single_op

# Enable waveform dumping (FST format)
ifeq ($(WAVES),1)
    ifeq ($(SIM),icarus)
        COMPILE_ARGS += -DCOCOTB_DUMP_WAVE
    endif
endif

# --- Cocotb Plusargs ---
export COCOTB_LOG_LEVEL=INFO

# --- Default target ---
all: gen_data sim

# --- Include Cocotb Makefiles ---
include $(shell cocotb-config --makefiles)/Makefile.sim

# --- Extra Targets ---

# Generate test data before simulation
OP ?= mul
gen_data:
	python gen_data.py --op $(OP)

# Clean build artifacts
clean::
	rm -rf __pycache__ *.pyc sim_build results.xml
	rm -rf data/ram.txt data/ideal_result.txt
	rm -f dump.fst dump.vcd
