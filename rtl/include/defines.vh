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
`define N_PE          4      // cluster size — change here to scale
`define N_MAC         8
`define N_MAC_BITS    3       // log2(8)
// Accumulator module = row_accumulator_<N_MAC>bank.  Define ACC_8BANK when N_MAC==8
// (leave it undefined when N_MAC==16).  `ifdef in pe_top.v selects the instance.
`define ACC_8BANK

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
//
// Problem constraints: M, K, N ∈ [16, 512]; A and B density ≤ 30%.
//   => max A nnz = max B nnz = floor(0.30 * 512 * 512) = 78643.
//
// A is row-partitioned across N_PE PEs (balanced by work); B is broadcast so
// every PE holds the full B.  C is per-PE, local-row indexed (see C bank below).
//
//   A buffer (per PE) — holds this PE's share of A (~total / N_PE, balanced):
//     A_row_desc_buf: 256  × 64-bit  =   2 KB   (>= 512/N_PE rows, +margin)
//     A_col_buf:      20480 × 16-bit =  40 KB   (>= 78643/N_PE = 19661 nnz)
//     A_val_buf:      20480 × 16-bit =  40 KB   (BRAM-aligned: 10 x 2K deep)
//   B buffer (per PE, broadcast = full B):
//     B_row_desc_buf: 512   × 64-bit =   4 KB   (>= K rows)
//     B_col_buf:      78848 × 16-bit ≈ 154 KB   (>= 78643 nnz)
//     B_val_buf:      78848 × 16-bit ≈ 154 KB
//   acc_buf: 512 × 16-bit = 1 KB   (>= N output columns)
//
// NOTE: per-PE A sizing assumes the host balances the row partition by work;
// 32768 gives ~1.67x headroom over the ideal 78643/4 = 19661 nnz/PE.
//=============================================================================
`define A_ROW_SLOT_PER_PE  256
`define A_NNZ_SLOT_PER_PE  28672   // >= 78643/N_PE(=26214 at N_PE=3); BRAM-aligned (14*2048)
`define A_ROW_ADDR_BITS    8       // log2(256)
`define A_NNZ_ADDR_BITS    15      // addr space 32768 >= 28672

`define B_ROW_SLOT         512     // >= max K
// OUTPUT-COLUMN TILING (fixed 64-col tiles): each pass loads only 64 output
// columns' worth of B, so the resident B buffer holds one 64-col tile instead of
// the full B.  T = ceil(N/64) passes; A stays resident and is reused every pass.
// Worst 64-col tile across all reference matrices = 9792 nnz (TC2_PEAK/TC6); the
// slot below (2048 deep/bank x 8 banks) holds it with margin AND still fits the
// small non-tiled cases (case1 full B = 4720).  Bump only if a 64-col tile can
// exceed 16384.
`define B_NNZ_SLOT         16384   // 8-bank aligned: 2048*8; one 64-col B tile
`define B_ROW_ADDR_BITS    9       // log2(512)
`define B_NNZ_ADDR_BITS    17      // addr port width (over-provisioned; depth = SLOT/N_MAC)

`define PE_ACC_DEPTH       512     // >= max N output columns
`define PE_ACC_ADDR_BITS   9

//=============================================================================
// C bank — independent on-chip C storage, indexed by LOCAL row.
//
//   Each PE stores only the rows it computes, densely (local row 0..count-1),
//   translated back to the global C row via C_row_map.  Depth is therefore set
//   by the max A-rows a single PE processes, NOT the global row range —
//   ~ceil(MAX_M / N_PE) for a balanced cluster.
//
//   C_ROW_ADDR_BITS is overridable on the iverilog command line
//   (-DC_ROW_ADDR_BITS=N).  Default is the CLUSTER configuration:
//     cluster (balanced, ceil(512/N_PE)=128 rows/PE): 7 → 128 slots (default)
//     single PE (processes all M rows, up to 256):     8 → 256 slots (override)
//=============================================================================
`ifndef C_ROW_ADDR_BITS
`define C_ROW_ADDR_BITS  7
`endif
`define C_ROW_SLOTS      (1 << `C_ROW_ADDR_BITS)

// On-chip C bank / accumulator column capacity per PASS = the output-column tile
// width.  With fixed 64-col output tiling every pass makes <=64 columns, so the C
// bank needs only 64 cols (8x less depth than the full MAX_N=512).  Overridable
// (-DC_BANK_COLS=512) for non-tiled runs where a single pass has N>64.
`ifndef C_BANK_COLS
`define C_BANK_COLS 64
`endif

// Instruction buffer (pre-computed schedule: b_group + a_val_ptr + lane_valid)
`define INSTR_SLOT         65536
`define INSTR_ADDR_BITS    16     // log2(65536)

//=============================================================================
// Task & Product Group FIFO parameters (widths are for N_MAC=16 lanes/group)
//   task        = {b_val[15:0], a_val[15:0], col_id[8:0]}     41-bit
//   task_group  = {comp_sel, lane_valid[15:0], task15..task0} 673-bit
//   product     = {col_id[8:0], fp16_val[15:0]}               25-bit
//   prod_group  = {lane_valid[15:0], prod15..prod0}           416-bit
//
//   col_id is 9-bit because MAX_N=512 needs only log2(512)=9 bits.
//=============================================================================
`define COL_ID_W          $clog2(`C_BANK_COLS)   // output-col width: 6 @64-col tiles, 9 @512
`define TASK_WIDTH        (`COL_ID_W + 32)   // {b_val[15:0], a_val[15:0], col_id}
// +1 MSB carries comp_sel (which ping-pong accumulator this group belongs to)
// so rows can be pipelined: the generator tags each group, and the product is
// routed by that tag instead of a global comp_sel.
`define TASK_GROUP_WIDTH  (`N_MAC + `N_MAC * `TASK_WIDTH + 1)   // 16+16*41+1 = 673

`define PRODUCT_WIDTH       (`COL_ID_W + 16)  // {col_id, fp16_val[15:0]}
`define PRODUCT_GROUP_WIDTH (`N_MAC + `N_MAC * `PRODUCT_WIDTH)  // 16+16*25 = 416

`define TASK_FIFO_DEPTH     512
`define TASK_FIFO_DEPTH_LOG 9

`define PROD_FIFO_DEPTH     256
`define PROD_FIFO_DEPTH_LOG  8

// Pointer-task FIFO: one entry per A-nonzero (a_val[15:0], b_off[16:0], num_groups[6:0])
// +1 MSB carries comp_sel (target ping-pong accumulator), see TASK_GROUP_WIDTH.
`define PTR_TASK_WIDTH      41
`define PTR_FIFO_DEPTH      512
`define PTR_FIFO_DEPTH_LOG  9

// Per-bank scatter FIFO depth (in the row accumulator).  This FIFO is multi-write-
// port -> registers+mux, NOT RAM-mappable, so it is a real LUT cost: x16 banks x2
// ping-pong accs x N_PE.  Any depth is FUNCTIONALLY SAFE (the 4-write-port scatter
// lands <=4 lanes/bank/cycle and free-gating throttles, never overflows); smaller
// just throttles same-bank bursts.  Measured: 8 vs 16 costs ~2% cluster cycles on
// C(251,121) (more on denser problems).  Default 8 (LUT-lean for PE count); bump to
// 16 for throughput.  BANK_FIFO_LOG is derived via $clog2 at instantiation, so only
// this one knob needs setting.  Overridable: -DBANK_FIFO_DEPTH=N.
`ifndef BANK_FIFO_DEPTH
`define BANK_FIFO_DEPTH 8
`endif

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
