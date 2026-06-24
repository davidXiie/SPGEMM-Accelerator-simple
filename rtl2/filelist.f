# SPGEMM-Accelerator v2 Verilog RTL file list
# PE: 4-MAC + task_packer + task_group_fifo + product_group_fifo + serializer + accumulator

## Include
rtl/include/defines.vh

## Infrastructure
rtl/infrastructure/axi_interface.v
rtl/infrastructure/scratchpad.v

## Core
rtl/core/core_top.v
rtl/core/descriptor_loader.v
rtl/core/b_broadcast_loader.v
rtl/core/a_group_loader.v
rtl/core/pe_top.v
rtl/core/pe_task_packer.v
rtl/core/pe_mul_array.v
rtl/core/pe_serializer.v
rtl/core/pe_accumulator.v
rtl/core/c_dense_buffer.v
rtl/core/c_dense_write_arbiter.v
rtl/core/c_dense_ddr_writer.v

## Top Wrapper
rtl/wrapper.v

## Simulation
rtl/sim/tb_core_top.v
