//=============================================================================
// sp_elementwise.v — single-read-per-state FSM, single always block, row_end output
//=============================================================================

`include "defines.vh"

module sp_elementwise (
    input  wire                      start,
    output reg                       done,
    input  wire [2:0]                op_type,
    input  wire [`MAX_DIM_BITS-1:0]  M,
    input  wire [`MAX_DIM_BITS-1:0]  N,
    input  wire [15:0]               a_row_sram, a_col_sram, a_val_sram,
    input  wire [15:0]               b_row_sram, b_col_sram, b_val_sram,
    output reg                       gbuf_rd_en,
    output reg  [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr,
    input  wire [`DATA_WIDTH-1:0]    gbuf_rd_data,
    input  wire                      gbuf_rd_valid,
    output reg                       out_valid,
    output reg  [`DATA_WIDTH-1:0]    out_col, out_val,
    output reg  [`MAX_DIM_BITS-1:0]  out_row_id, out_nnz,
    output reg                       out_row_end,   // NEW: row boundary signal
    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam S_IDLE=0, S_RAR0=1,S_RAR1=2,S_RBR0=3,S_RBR1=4,
               S_FAC=5,S_FAV=6,S_FBC=7,S_FBV=8,S_CMP=9,S_RDN=10,S_DON=11;

    reg [3:0] state;
    reg [`MAX_DIM_BITS-1:0] cur_row, row_nnz;
    reg [15:0] a_ptr, a_end, b_ptr, b_end;
    reg [`DATA_WIDTH-1:0] ac, av, bc, bv;
    reg a_rdy, b_rdy;
    reg [15:0] ar0_l, ar1_l, br0_l, br1_l;
    reg gbuf_req;
    reg gbuf_rvalid_r;
    reg [`DATA_WIDTH-1:0] gbuf_rdata_r;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state<=0; cur_row<=0; a_ptr<=0;a_end<=0; b_ptr<=0;b_end<=0; row_nnz<=0;
            ac<=0;av<=0;bc<=0;bv<=0; a_rdy<=0;b_rdy<=0;
            gbuf_req<=0; gbuf_rvalid_r<=0; gbuf_rdata_r<=0;
            out_valid<=0;out_col<=0;out_val<=0;out_row_id<=0;out_nnz<=0;
            out_row_end<=0; done<=0;
        end else begin
            out_valid<=0; out_row_end<=0; done<=0;
            gbuf_rvalid_r <= gbuf_rd_valid;
            if (gbuf_rd_valid) gbuf_rdata_r <= gbuf_rd_data;

            case (state)
                S_IDLE: if (start) begin
                    cur_row<=0; a_rdy<=0; b_rdy<=0; gbuf_req<=1; state<=S_RAR0; end
                S_RAR0: if (gbuf_rvalid_r) begin ar0_l<=gbuf_rdata_r[15:0]; state<=S_RAR1; end
                S_RAR1: if (gbuf_rvalid_r) begin ar1_l<=gbuf_rdata_r[15:0]; state<=S_RBR0; end
                S_RBR0: if (gbuf_rvalid_r) begin br0_l<=gbuf_rdata_r[15:0]; state<=S_RBR1; end
                S_RBR1: if (gbuf_rvalid_r) begin
                    br1_l<=gbuf_rdata_r[15:0];
                    a_ptr<=ar0_l; a_end<=ar1_l; b_ptr<=br0_l; b_end<=br1_l;
                    gbuf_req<=0; row_nnz<=0; state<=S_FAC;
                end
                S_FAC: if (a_ptr < a_end) begin gbuf_req<=1; if (gbuf_rvalid_r) begin ac<=gbuf_rdata_r; state<=S_FAV; end end
                       else begin gbuf_req<=0; state<=S_FAV; end
                S_FAV: if (gbuf_req && gbuf_rvalid_r) begin av<=gbuf_rdata_r; a_rdy<=1; a_ptr<=a_ptr+1; state<=(b_ptr<b_end)?S_FBC:S_CMP; end
                       else if (!gbuf_req) state<=(b_ptr<b_end)?S_FBC:S_CMP;
                S_FBC: if (b_ptr < b_end) begin gbuf_req<=1; if (gbuf_rvalid_r) begin bc<=gbuf_rdata_r; state<=S_FBV; end end
                       else begin gbuf_req<=0; state<=S_FBV; end
                S_FBV: if (gbuf_req && gbuf_rvalid_r) begin bv<=gbuf_rdata_r; b_rdy<=1; b_ptr<=b_ptr+1; state<=S_CMP; end
                       else if (!gbuf_req) state<=S_CMP;
                S_CMP: begin
                    if (a_rdy && b_rdy) begin
                        if (ac[`MAX_DIM_BITS-1:0] == bc[`MAX_DIM_BITS-1:0]) begin
                            if (op_type==`OP_TYPE_ADD) begin out_val<=av+bv; out_valid<=1; end
                            else if (av != bv) begin out_val<=av-bv; out_valid<=1; end
                            // else: A-B=0, skip output
                            out_col<=ac; out_row_id<=cur_row;
                            if (op_type!=`OP_TYPE_SUB || av!=bv) row_nnz<=row_nnz+1;
                            a_rdy<=0; b_rdy<=0;
                        end else if (ac[`MAX_DIM_BITS-1:0] < bc[`MAX_DIM_BITS-1:0]) begin
                            out_valid<=1; out_col<=ac; out_val<=av; out_row_id<=cur_row;
                            row_nnz<=row_nnz+1; a_rdy<=0;
                        end else begin
                            out_valid<=1; out_col<=bc;
                            out_val<=(op_type==`OP_TYPE_SUB)?(16'h0000-bv):bv;
                            out_row_id<=cur_row; row_nnz<=row_nnz+1; b_rdy<=0;
                        end
                    end else if (a_rdy && !b_rdy && b_ptr>=b_end) begin
                        out_valid<=1; out_col<=ac; out_val<=av; out_row_id<=cur_row;
                        row_nnz<=row_nnz+1; a_rdy<=0;
                    end else if (b_rdy && !a_rdy && a_ptr>=a_end) begin
                        out_valid<=1; out_col<=bc;
                        out_val<=(op_type==`OP_TYPE_SUB)?(16'h0000-bv):bv;
                        out_row_id<=cur_row; row_nnz<=row_nnz+1; b_rdy<=0;
                    end else if (!a_rdy && !b_rdy && a_ptr>=a_end && b_ptr>=b_end) begin
                        out_nnz<=row_nnz; out_row_end<=1; state<=S_RDN;
                    end
                end
                S_RDN: begin
                    cur_row<=cur_row+1; row_nnz<=0; a_rdy<=0; b_rdy<=0; gbuf_req<=1;
                    if (cur_row+1 >= M) state<=S_DON; else state<=S_RAR0;
                end
                S_DON: begin done<=1; state<=S_IDLE; end
            endcase
        end
    end

    always @(*) begin
        gbuf_rd_en=0; gbuf_rd_addr=0;
        case (state)
            S_RAR0,S_RBR0: begin gbuf_rd_en=gbuf_req; gbuf_rd_addr=(state==S_RAR0?a_row_sram:b_row_sram)+cur_row; end
            S_RAR1: begin gbuf_rd_en=1; gbuf_rd_addr=a_row_sram+cur_row+1; end
            S_RBR1: begin gbuf_rd_en=1; gbuf_rd_addr=b_row_sram+cur_row+1; end
            S_FAC: if (a_ptr<a_end) begin gbuf_rd_en=1; gbuf_rd_addr=a_col_sram+a_ptr; end
            S_FAV: if (gbuf_req) begin gbuf_rd_en=1; gbuf_rd_addr=a_val_sram+a_ptr; end
            S_FBC: if (b_ptr<b_end) begin gbuf_rd_en=1; gbuf_rd_addr=b_col_sram+b_ptr; end
            S_FBV: if (gbuf_req) begin gbuf_rd_en=1; gbuf_rd_addr=b_val_sram+b_ptr; end
        endcase
    end

endmodule
