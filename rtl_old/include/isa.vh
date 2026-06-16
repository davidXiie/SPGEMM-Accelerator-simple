//=============================================================================
// File     : isa.vh
// Project  : SPGEMM-Accelerator
// Brief    : ISA instruction format definitions for SPGEMM accelerator
//            Supports: SPGEMM (MUL), SPAdd, SPSubtract, host-side scheduling
//=============================================================================

`ifndef ISA_VH
`define ISA_VH

`include "defines.vh"

//=============================================================================
// 256-bit Instruction Layout
//=============================================================================
// bit[2:0]   : opcode
// bit[5:3]   : memory type ID (LOAD/STORE) / operation type (COMPUTE: MUL/ADD/SUB)
// bit[8:6]   : reserved (unused, formerly COMPUTE op_type, moved to avoid A_row_sram conflict)

//=============================================================================
// Opcode encoding (3-bit)
//=============================================================================
// `OP_LOAD      = 3'b000  : Load CSR data from DRAM to GlobalBuffer
// `OP_LOAD_TASK = 3'b001  : Load host-computed task descriptors
// `OP_STORE     = 3'b010  : Store result to DRAM
// `OP_COMPUTE   = 3'b011  : Trigger computation (type in bit[8:6])
// `OP_FINISH    = 3'b111  : Terminate instruction sequence

//=============================================================================
// Operation type (COMPUTE instruction bit[8:6])
//=============================================================================
// `OP_TYPE_MUL = 3'b000   : Sparse × Sparse (SpGEMM)
// `OP_TYPE_ADD = 3'b001   : Sparse + Sparse (SpAdd)
// `OP_TYPE_SUB = 3'b010   : Sparse - Sparse (SpSubtract)

//=============================================================================
// Load / LoadTask Instruction (bit format):
//   [2:0]    : opcode (LOAD=000 or LOAD_TASK=001)
//   [5:3]    : mem_id
//   [63:6]   : DRAM base address (58 bits)
//   [95:64]  : SRAM offset (32 bits)
//   [127:96] : xsize (transfer size in elements, 32 bits)
//   [255:128]: reserved (128 bits)
//=============================================================================
`define LOAD_DRAM_BASE_HI   63
`define LOAD_DRAM_BASE_LO   6
`define LOAD_SRAM_OFFSET_HI 95
`define LOAD_SRAM_OFFSET_LO 64
`define LOAD_XSIZE_HI       127
`define LOAD_XSIZE_LO       96

//=============================================================================
// Store Instruction (same layout as Load):
//=============================================================================
`define STORE_DRAM_BASE_HI  63
`define STORE_DRAM_BASE_LO  6
`define STORE_SRAM_OFFSET_HI 95
`define STORE_SRAM_OFFSET_LO 64
`define STORE_XSIZE_HI      127
`define STORE_XSIZE_LO      96

//=============================================================================
// COMPUTE Instruction (SpGEMM / SpAdd / SpSubtract) (bit format):
//   [2:0]    : opcode = 3'b011
//   [5:3]    : op_type (000=MUL, 001=ADD, 010=SUB)   ← moved from [8:6] to avoid A_row_sram conflict
//   [63:6]   : A_row_ptr SRAM base (58 bits)
//   [95:64]  : A_col_idx SRAM base (32 bits)
//   [127:96] : A_val SRAM base (32 bits)
//   [159:128]: B_row_ptr SRAM base (32 bits)
//   [191:160]: B_col_idx SRAM base (32 bits)
//   [223:192]: B_val SRAM base (32 bits)
//   [232:224]: M (A rows, 9 bits)
//   [241:233]: K (A cols = B rows, 9 bits)        ← N for ADD/SUB (element-wise)
//   [250:242]: N (B cols, 9 bits)
//   [255:251]: reserved (5 bits)
//
//   For ADD/SUB (element-wise on A and B):
//     M = number of non-zero elements in A (or row count for CSR)
//     K = reserved / used as operand B nnz hint
//     N = number of non-zero elements in B
//=============================================================================
`define COMPUTE_OP_TYPE_HI     5
`define COMPUTE_OP_TYPE_LO     3
`define SPGEMM_A_ROW_SRAM_HI    63
`define SPGEMM_A_ROW_SRAM_LO    6
`define SPGEMM_A_COL_SRAM_HI    95
`define SPGEMM_A_COL_SRAM_LO    64
`define SPGEMM_A_VAL_SRAM_HI    127
`define SPGEMM_A_VAL_SRAM_LO    96
`define SPGEMM_B_ROW_SRAM_HI    159
`define SPGEMM_B_ROW_SRAM_LO    128
`define SPGEMM_B_COL_SRAM_HI    191
`define SPGEMM_B_COL_SRAM_LO    160
`define SPGEMM_B_VAL_SRAM_HI    223
`define SPGEMM_B_VAL_SRAM_LO    192
`define SPGEMM_M_HI             232
`define SPGEMM_M_LO             224
`define SPGEMM_K_HI             241
`define SPGEMM_K_LO             233
`define SPGEMM_N_HI             250
`define SPGEMM_N_LO             242

//=============================================================================
// Finish Instruction:
//   bit[2:0] = 3'b111, rest don't care
//=============================================================================

`endif // ISA_VH
