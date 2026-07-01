# =============================================================================
# OOC synthesis: ONE PE at N_MAC=8 (8-bank accumulator, 64-col C bank).
#
# Purpose: get the real per-PE LUT/BRAM/DSP so we can extrapolate x N_PE and
# decide whether 8 PEs fit.  Config is taken from rtl/include/defines.vh:
#     N_MAC=8, `define ACC_8BANK, C_BANK_COLS=64 (64-col output tiling).
#
# This models ONE PE of an 8-PE cluster:
#   -verilog_define C_ROW_ADDR_BITS=6   -> 64 C rows/PE (512 rows / 8 PEs).
#
# CAVEAT on BRAM: A_NNZ_SLOT_PER_PE in defines.vh is still 28672 (sized for a
# 3-PE partition).  For an 8-PE cluster A/PE is ~78643/8 ~= 9830, so the A
# buffers here OVER-count; scale A BRAM down (~2.3x) for the 8-PE estimate.
# LUT (the binding constraint) is independent of A_NNZ_SLOT, so it is accurate.
#
# Run from repo root:
#     vivado -mode batch -source syn/synth_pe_top_n8.tcl
# Switch -part to xcku040-... to check the larger device (242400 LUT).
# =============================================================================

set RTL /home/yin/github/SPGEMM-Accelerator-simple/rtl/core

# fp16 datapath + both accumulator variants (only the 8-bank is instantiated
# because ACC_8BANK is defined; the 16-bank set is read but left unused).
read_verilog $RTL/fp16_mul.v
read_verilog $RTL/fp16_add.v
read_verilog $RTL/accum_bank.v
read_verilog $RTL/accum_bank_16.v
read_verilog $RTL/row_accumulator_8bank.v
read_verilog $RTL/row_accumulator_16bank.v
read_verilog $RTL/pe_mul_array.v
read_verilog $RTL/pe_top.v

# xcku035 = 203128 LUT / 540 BRAM36 / 1700 DSP  (matches earlier cluster synth)
# xcku040 = 242400 LUT / 600 BRAM36 / 1920 DSP  (swap -part below to compare)
synth_design -top pe_top -part xcku035-sfva784-1LV-i \
    -include_dirs rtl/include \
    -verilog_define C_ROW_ADDR_BITS=6 \
    -flatten_hierarchy none \
    -mode out_of_context

# Flat + hierarchical utilization so we can see the accumulator/FIFO/datapath split.
report_utilization             -file syn/pe_top_n8_utilization.rpt
report_utilization -hierarchical -file syn/pe_top_n8_util_hier.rpt
write_checkpoint -force syn/pe_top_n8_synth.dcp

puts "==== pe_top (N_MAC=8) OOC synth done ===="
puts "Read syn/pe_top_n8_utilization.rpt ; multiply LUT/BRAM/DSP by 8 and compare"
puts "to the part budget (LUT is the constraint: 8*perPE <= 203128 xcku035 / 242400 xcku040)."
