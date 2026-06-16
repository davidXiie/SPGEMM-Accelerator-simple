//=============================================================================
// File     : defines.vh
// Project  : SPGEMM-Accelerator
// Brief    : Global parameter definitions for SPGEMM accelerator
//=============================================================================

`ifndef DEFINES_VH
`define DEFINES_VH

`timescale 1ns/1ps

//=============================================================================
// Matrix Dimensions
//=============================================================================
`define MAX_M         512     // A rows max
`define MAX_K         512     // A cols = B rows max
`define MAX_N         512     // B cols max
`define MAX_DIM_BITS  10      // log2(512) = 10

//=============================================================================
// PE & MAC Configuration
//=============================================================================
`define N_PE          8       // Number of PEs
`define N_MAC         4       // MAC units per PE
`define N_MAC_BITS    3       // log2(N_MAC)
`define N_PE_BITS     3       // log2(N_PE)

//=============================================================================
// Data Width (FP16: half-precision floating point)
//=============================================================================
`define DATA_WIDTH    16      // FP16 (IEEE 754 half precision: 1 sign, 5 exp, 10 mantissa)
`define DATA_BYTES    2       // DATA_WIDTH / 8
`define DATA_BITS_LOG2 4      // log2(16)
`define DATA_BYTE_LOG2 1      // log2(2) = log2(DATA_BYTES)

//=============================================================================
// AXI Bus Parameters 
//=============================================================================
`define AXI_DATA_WIDTH 512    // AXI data width                 AXI协议数据宽度
`define AXI_ADDR_WIDTH 64     // AXI address width              AXI协议地址宽度
`define AXI_LEN_WIDTH  8      // AXI burst length width         AXI协议burst长度宽度
`define AXI_STRB_WIDTH 64     // AXI strobe width (= AXI_DATA_WIDTH/8, byte-level)          AXI协议strobe宽度
`define AXI_ID_WIDTH   4      // AXI transaction ID width       AXI协议ID宽度

//=============================================================================
// Instruction Format (256-bit)
//=============================================================================
`define INST_WIDTH    256
`define OPCODE_WIDTH  3       // opcode [2:0]
`define MEMID_WIDTH   3       // memory type ID [5:3]

// Opcode values (3-bit)
`define OP_LOAD        3'b000    // Load CSR data from DRAM
`define OP_LOAD_TASK   3'b001    // Load host-computed task descriptors (NEW)
`define OP_STORE       3'b010    // Store result to DRAM
`define OP_COMPUTE     3'b011    // Trigger computation (type in bit[8:6])
`define OP_FINISH      3'b111    // Terminate

// Operation types (COMPUTE instruction bit[8:6])
`define OP_TYPE_MUL    3'b000    // Sparse Matrix × Sparse Matrix (SpGEMM)
`define OP_TYPE_ADD    3'b001    // Sparse Matrix + Sparse Matrix (SpAdd)
`define OP_TYPE_SUB    3'b010    // Sparse Matrix - Sparse Matrix (SpSubtract)

// Memory type IDs (for LOAD / LOAD_TASK / STORE)
`define MEM_ROW_PTR    3'b000    // CSR row pointer array
`define MEM_COL_IDX    3'b001    // CSR column index array
`define MEM_VAL        3'b010    // CSR value array
`define MEM_OUTPUT     3'b011    // Output buffer
`define MEM_TASK_DESC  3'b100    // Task descriptor (NEW: host→accelerator PE task table)

//=============================================================================
// Scratchpad / Buffer Sizes
//   Sizing rationale for 512×512 FP16 matrices:
//   - PE B Buffer: full B CSR, worst-case dense 512×512 nnz = 262144 entries
//     plus B_row_ptr (513 entries). In practice, sparsity >> 0%, so allocate 131K.
//   - PE A Buffer: holds this PE's assigned A rows (worst ~256 rows × dense ~512 nnz/row
//     = 131K elements). 32K is a practical buffer with spill handling assumed.
//   - Global Buffer: caches both A CSR + B CSR from DRAM.
//   - SPA depth = MAX_N = 512 (one accumulator per possible C column).
//=============================================================================
`define GBUF_DEPTH      65536   // Global Buffer depth (entries)
`define GBUF_DEPTH_LOG  16      // log2(65536)

`define PE_ABUF_DEPTH   32768   // PE A Buffer depth
`define PE_ABUF_DEPTH_LOG 15    // log2(32768)

`define PE_BBUF_DEPTH   131072  // PE B Buffer depth (full B CSR)
`define PE_BBUF_DEPTH_LOG 17    // log2(131072)

`define PE_SPA_DEPTH    512     // PE Partial Row Buffer / SPA depth (= MAX_N)
`define PE_SPA_DEPTH_LOG 9      // log2(512)

`define OUTBUF_DEPTH    65536   // Output Buffer depth (streaming to Store)
`define OUTBUF_DEPTH_LOG 16     // log2(65536)

//=============================================================================
// Scheduler Parameters (moved to host: these are for PE internal usage only)
//=============================================================================
`define WORKLOAD_BITS 20      // Bit width for workload counters

//=============================================================================
// Task Descriptor Parameters (host → accelerator)
//=============================================================================
`define TASK_DESC_ELEMENTS 5  // elements per task desc: row_start, row_end, a_ptr_start, a_ptr_end, valid
`define TASK_DESC_TOTAL     (`N_PE * `TASK_DESC_ELEMENTS)  // 8 PEs × 5 = 40 elements

//=============================================================================
// CSR Writer Parameters
//=============================================================================
`define CSR_ADDR_BITS 20      // CSR output address width
`define CSR_NNZ_BITS  10      // Per-row nnz counter bits

//=============================================================================
// Global Constants
//=============================================================================
`define AXI_BURST_MAX 256     // Max AXI burst length

//=============================================================================
// Derived Constants
//=============================================================================
`define N_MAC_PER_PE  `N_MAC
`define TOTAL_MAC     (`N_PE * `N_MAC)
`define PE_ID_BITS    `N_PE_BITS

// Bank block size for GlobalBuffer: DATA_WIDTH * N_MAC (16*4=64 bits per bank block)
// One bank block = one cycle of N_MAC parallel FP16 reads
`define BANK_BLOCK_SIZE (`DATA_WIDTH * `N_MAC)      // 64 bits
`define BANK_BLOCK_BYTES (`BANK_BLOCK_SIZE / 8)      // 8 bytes

// AXI beat carries N_ELEM_PER_AXI_BEAT elements (512/16=32 FP16 elements per beat)
`define N_ELEM_PER_AXI_BEAT (`AXI_DATA_WIDTH / `DATA_WIDTH)
`define AXI_ELEM_PER_BEAT_LOG 5   // log2(32)

`endif // DEFINES_VH
