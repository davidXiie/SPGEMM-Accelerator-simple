//=============================================================================
// File     : a_global_buffer.v
// Brief    : Global A-matrix buffer (CSR format).  Holds ALL A rows before
//            partition.  Read port drives pe_load_ctrl during the load phase.
//
//   A_desc_buf[r] = 64-bit : {a_off[31:0], a_nnz[15:0], c_row[15:0]}
//     a_off points into A_col_buf / A_val_buf (global offset).
//
//   A_col_buf[i]  = 16-bit : k_idx   (column index into B)
//   A_val_buf[i]  = 16-bit : FP16 value
//
//   Sizing:
//     desc_depth = MAX_M                           (512 rows)
//     nnz_depth  = N_PE * A_NNZ_SLOT_PER_PE        (3x28672 = 86016)
//     Host writes all data before start; accel reads during S_LOAD_A.
//=============================================================================

`include "defines.vh"

module a_global_buffer #(
    parameter DESC_DEPTH = `MAX_M,                     // 512
    parameter NNZ_DEPTH  = `N_PE * `A_NNZ_SLOT_PER_PE, // 86016
    parameter DESC_AW    = `MAX_DIM_BITS,               // log2(512) = 9..but MAX_DIM_BITS=10
    parameter NNZ_AW      = 17                           // log2(86016) < 17
) (
    input  wire clk,
    input  wire rst_n,

    // === Host write ports (single-cycle) ===
    input  wire                   host_desc_wr_en,
    input  wire [DESC_AW-1:0]     host_desc_wr_addr,
    input  wire [63:0]            host_desc_wr_data,

    input  wire                   host_col_wr_en,
    input  wire [NNZ_AW-1:0]      host_col_wr_addr,
    input  wire [15:0]            host_col_wr_data,

    input  wire                   host_val_wr_en,
    input  wire [NNZ_AW-1:0]      host_val_wr_addr,
    input  wire [15:0]            host_val_wr_data,

    // === Accel read ports (registered / synchronous read) ===
    input  wire                   rd_desc_en,
    input  wire [DESC_AW-1:0]     rd_desc_addr,
    output wire [63:0]            rd_desc_data,

    input  wire                   rd_col_en,
    input  wire [NNZ_AW-1:0]      rd_col_addr,
    output wire [15:0]            rd_col_data,

    input  wire                   rd_val_en,
    input  wire [NNZ_AW-1:0]      rd_val_addr,
    output wire [15:0]            rd_val_data
);

    // BRAM arrays — synchronous read, registered output
    reg [63:0] desc_buf [0:DESC_DEPTH-1];
    reg [15:0] col_buf  [0:NNZ_DEPTH-1];
    reg [15:0] val_buf  [0:NNZ_DEPTH-1];

    // Write side (host)
    always @(posedge clk) begin
        if (host_desc_wr_en) desc_buf[host_desc_wr_addr] <= host_desc_wr_data;
        if (host_col_wr_en) col_buf[host_col_wr_addr]   <= host_col_wr_data;
        if (host_val_wr_en) val_buf[host_val_wr_addr]   <= host_val_wr_data;
    end

    // Read side (registered output) — accel
    reg [63:0] desc_rd;
    reg [15:0] col_rd;
    reg [15:0] val_rd;
    always @(posedge clk) begin
        if (rd_desc_en) desc_rd <= desc_buf[rd_desc_addr];
        if (rd_col_en)  col_rd  <= col_buf[rd_col_addr];
        if (rd_val_en)  val_rd  <= val_buf[rd_val_addr];
    end
    assign rd_desc_data = desc_rd;
    assign rd_col_data  = col_rd;
    assign rd_val_data  = val_rd;

endmodule
