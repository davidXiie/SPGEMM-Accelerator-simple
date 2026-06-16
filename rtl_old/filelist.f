# SPGEMM-Accelerator Verilog RTL file list
# Updated: Phase 2 — added task_loader, sp_elementwise, removed scheduler

## include
rtl/include/defines.vh
rtl/include/isa.vh

## Infrastructure
rtl/infrastructure/axi_interface.v
rtl/infrastructure/decode.v
rtl/infrastructure/fetch.v
rtl/infrastructure/load.v
rtl/infrastructure/store.v
rtl/infrastructure/scratchpad.v

## Core
rtl/core/core_top.v
rtl/core/task_loader.v
rtl/core/pe_top.v
rtl/core/pe_decompress.v
rtl/core/pe_mul_array.v
rtl/core/pe_aggregation.v
rtl/core/sp_elementwise.v
rtl/core/c_csr_writer.v

## Top Wrapper
rtl/wrapper.v

## Simulation
rtl/sim/tb_core_top.v
