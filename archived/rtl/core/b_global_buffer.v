//=============================================================================
// File     : b_global_buffer.v
// Brief    : Global B-matrix buffer (CSR format).  Holds the full B matrix.
//            B is broadcast to all PEs during load phase.
//
//   B_desc_buf[r] = 32-bit : {b_off[16:0], b_nnz[9:0]}
//     b_off points into B_col_buf / B_val_buf.
//
//   B_col_buf[i]  = 16-bit : column_id
//   B_val_buf[i]  = 16-bit : FP16 value
//
//   Sizing:
//     desc_depth = MAX_K                   (512 rows)
//     nnz_depth  = B_NNZ_SLOT              (40960 — fits per-PE B bank with tiling)
//=============================================================================

`include "defines.vh"

module b_global_buffer #(
    parameter DESC_DEPTH = `MAX_K,
    parameter NNZ_DEPTH  = `B_NNZ_SLOT,
    parameter DESC_AW    = `B_ROW_ADDR_BITS,
    parameter NNZ_AW      = `B_NNZ_ADDR_BITS
) (
    input  wire clk,
    input  wire rst_n,

    // === Host write ports ===
    input  wire                   host_desc_wr_en,
    input  wire [DESC_AW-1:0]     host_desc_wr_addr,
    input  wire [31:0]            host_desc_wr_data,

    input  wire                   host_col_wr_en,
    input  wire [NNZ_AW-1:0]      host_col_wr_addr,
    input  wire [15:0]            host_col_wr_data,

    input  wire                   host_val_wr_en,
    input  wire [NNZ_AW-1:0]      host_val_wr_addr,
    input  wire [15:0]            host_val_wr_data,

    // === Accel read ports ===
    input  wire                   rd_desc_en,
    input  wire [DESC_AW-1:0]     rd_desc_addr,
    output wire [31:0]            rd_desc_data,

    input  wire                   rd_col_en,
    input  wire [NNZ_AW-1:0]      rd_col_addr,
    output wire [15:0]            rd_col_data,

    input  wire                   rd_val_en,
    input  wire [NNZ_AW-1:0]      rd_val_addr,
    output wire [15:0]            rd_val_data
);

    reg [31:0] desc_buf [0:DESC_DEPTH-1];
    reg [15:0] col_buf  [0:NNZ_DEPTH-1];
    reg [15:0] val_buf  [0:NNZ_DEPTH-1];

    always @(posedge clk) begin
        if (host_desc_wr_en) desc_buf[host_desc_wr_addr] <= host_desc_wr_data;
        if (host_col_wr_en) col_buf[host_col_wr_addr]   <= host_col_wr_data;
        if (host_val_wr_en) val_buf[host_val_wr_addr]   <= host_val_wr_data;
    end

    reg [31:0] desc_rd;
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
