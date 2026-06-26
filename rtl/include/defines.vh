//=============================================================================
// File     : defines.vh
// Project  : SPGEMM-Accelerator v2 (精简架构)
// Brief    : Global parameter definitions — SpGEMM-only, dense C output
//            PE: 4-MAC + task_packer + product_serializer + serial_acc
//=============================================================================

`ifndef DEFINES_VH
`define DEFINES_VH

`timescale 1ns/1ps

//=============================================================================
// Matrix Dimensions
//=============================================================================
`define MAX_M         512
`define MAX_K         512
`define MAX_N         512
`define MAX_DIM_BITS  10      // log2(512)

//=============================================================================
// PE & MAC Configuration
//=============================================================================
`define N_PE          16      // cluster size — change here to scale
`define N_MAC         8
`define N_MAC_BITS    3       // log2(8)

// FP16 multiplier pipeline latency (1 = registered output)
`define MUL_LAT       1

//=============================================================================
// Data Width
//=============================================================================
`define DATA_WIDTH    16
`define DATA_BYTES    2
`define DATA_BYTE_LOG2 1

// Index / offset widths
`define IDX_WIDTH     16
`define OFFSET_WIDTH  32

//=============================================================================
// AXI Bus Parameters
//=============================================================================
`define AXI_DATA_WIDTH 512
`define AXI_ADDR_WIDTH 64
`define AXI_LEN_WIDTH  8
`define AXI_STRB_WIDTH 64
`define AXI_ID_WIDTH   4

`define N_ELEM_PER_AXI_BEAT (`AXI_DATA_WIDTH / `DATA_WIDTH)
`define AXI_ELEM_PER_BEAT_LOG 5
`define AXI_BURST_MAX 256

//=============================================================================
// Fixed DDR Address Map
//=============================================================================
`define DDR_A_GROUPS_BASE   64'h0000_0000
`define DDR_B_BASE          64'h0020_0000
`define DDR_C_DENSE_BASE    64'h0030_0000
`define DDR_DESC_BASE       64'h0040_0000

`define A_GROUP_STRIDE      20'h2_0000

`define A_ROW_DESC_OFFSET   16'h0000
`define A_COL_OFFSET        16'h1000
`define A_VAL_OFFSET        16'h9000

`define B_ROW_DESC_OFFSET   16'h0000
`define B_COL_OFFSET        16'h1000
`define B_VAL_OFFSET        16'h0000

//=============================================================================
// PE Local Buffer Sizes
//   A buffer (per PE):
//     A_row_desc_buf: 128 × 64-bit = 1 KB
//     A_col_buf:      16384 × 16-bit = 32 KB
//     A_val_buf:      16384 × 16-bit = 32 KB
//   B buffer (per PE):
//     B_row_desc_buf: 512 × 64-bit = 4 KB
//     B_col_buf:      78848 × 16-bit ≈ 154 KB
//     B_val_buf:      78848 × 16-bit ≈ 154 KB
//   acc_buf: 512 × 16-bit = 1 KB
//=============================================================================
`define A_ROW_SLOT_PER_PE  256
`define A_NNZ_SLOT_PER_PE  16384
`define A_ROW_ADDR_BITS    8       // log2(256)
`define A_NNZ_ADDR_BITS    14      // log2(16384)

`define B_ROW_SLOT         512
`define B_NNZ_SLOT         78848
`define B_ROW_ADDR_BITS    9       // log2(512)
`define B_NNZ_ADDR_BITS    17      // log2(78848)

`define PE_ACC_DEPTH       512
`define PE_ACC_ADDR_BITS   9

// Instruction buffer (pre-computed schedule: b_group + a_val_ptr + lane_valid)
`define INSTR_SLOT         65536
`define INSTR_ADDR_BITS    16     // log2(65536)

//=============================================================================
// Task & Product Group FIFO parameters
//   task        = {b_val[15:0], a_val[15:0], col_id[8:0]}    41-bit
//   task_group  = {lane_valid[7:0], task7..task0}             336-bit
//   product     = {col_id[8:0], fp16_val[15:0]}              25-bit
//   prod_group  = {lane_valid[7:0], prod7..prod0}            208-bit
//
//   col_id is 9-bit because MAX_N=512 needs only log2(512)=9 bits.
//=============================================================================
`define TASK_WIDTH        41   // 9 + 16 + 16
`define TASK_GROUP_WIDTH  (`N_MAC + `N_MAC * `TASK_WIDTH)   // 336

`define PRODUCT_WIDTH       25   // {col_id[8:0], fp16_val[15:0]}
`define PRODUCT_GROUP_WIDTH (`N_MAC + `N_MAC * `PRODUCT_WIDTH)  // 208

`define TASK_FIFO_DEPTH     512
`define TASK_FIFO_DEPTH_LOG 9

`define PROD_FIFO_DEPTH     256
`define PROD_FIFO_DEPTH_LOG  8

// Pointer-task FIFO: one entry per A-nonzero (a_val[15:0], b_off[16:0], num_groups[6:0])
`define PTR_TASK_WIDTH      40
`define PTR_FIFO_DEPTH      512
`define PTR_FIFO_DEPTH_LOG  9

//=============================================================================
// C_dense_buffer
//=============================================================================
`define C_DENSE_DEPTH       (`MAX_M * `MAX_N)
`define C_DENSE_DEPTH_LOG   18

`define C_ROW_STRIDE        `MAX_N
`define C_ROW_STRIDE_LOG    `MAX_DIM_BITS

//=============================================================================
// Descriptor layout (in DDR)
//=============================================================================
`define DESC_ELEMENTS 12
`define DESC_BYTES    (`DESC_ELEMENTS * 2)

//=============================================================================
// Derived
//=============================================================================
`define TOTAL_MAC (`N_PE * `N_MAC)

`endif // DEFINES_VH
