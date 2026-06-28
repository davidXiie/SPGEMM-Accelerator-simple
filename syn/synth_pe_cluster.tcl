# OOC synthesis: pe_cluster (multi-PE)
# Run from repo root:  vivado -mode batch -source syn/synth_pe_cluster.tcl

# ---- read sources (pe_top deps + cluster wrapper) ----
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/fp16_mul.v
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/fp16_add.v
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/accum_bank_16.v
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/row_accumulator_16bank.v
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/pe_mul_array.v
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/pe_top.v
read_verilog /home/yin/github/SPGEMM-Accelerator-simple/rtl/core/pe_cluster.v

# ---- synth ----
synth_design -top pe_cluster -part xcku035-sfva784-1LV-i \
    -flatten_hierarchy none \
    -mode out_of_context

# ---- reports ----
report_utilization -file syn/pe_cluster_utilization.rpt
write_checkpoint -force syn/pe_cluster_synth.dcp

