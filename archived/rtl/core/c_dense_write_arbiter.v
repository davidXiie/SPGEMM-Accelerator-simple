//=============================================================================
// File     : c_dense_write_arbiter.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Arbitrates PE cbuf writes to C_dense_buffer.
//            Phase 1 (N_PE=1): direct pass-through with ready handshake.
//=============================================================================

`include "defines.vh"

module c_dense_write_arbiter (
    input  wire [`N_PE-1:0]                  pe_cbuf_valid,
    output wire [`N_PE-1:0]                  pe_cbuf_ready,
    input  wire [`N_PE*`C_DENSE_DEPTH_LOG-1:0] pe_cbuf_addr,
    input  wire [`N_PE*`DATA_WIDTH-1:0]      pe_cbuf_data,

    output reg                                cbuf_wr_en,
    output reg  [`C_DENSE_DEPTH_LOG-1:0]      cbuf_wr_addr,
    output reg  [`DATA_WIDTH-1:0]             cbuf_wr_data,

    input  wire                               aclk,
    input  wire                               aresetn
);

    // Phase 1: N_PE=1 — direct passthrough
    assign pe_cbuf_ready[0] = 1'b1;  // always ready (C_dense_buffer has 1 write port)

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            cbuf_wr_en   <= 1'b0;
            cbuf_wr_addr <= 0;
            cbuf_wr_data <= 0;
        end else begin
            cbuf_wr_en   <= pe_cbuf_valid[0];
            cbuf_wr_addr <= pe_cbuf_addr[0 +: `C_DENSE_DEPTH_LOG];
            cbuf_wr_data <= pe_cbuf_data[0 +: `DATA_WIDTH];
        end
    end

endmodule
