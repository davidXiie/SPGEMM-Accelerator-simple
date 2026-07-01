//=============================================================================
// File     : pe_cluster.v
// Brief    : N_PE-wide PE cluster.
//
// A descriptors: per-PE streaming handshake (valid/ready/data).
// B is broadcast — one set of write ports drives all PE instances.
//=============================================================================

`include "defines.vh"

module pe_cluster #(
    parameter N_PE = `N_PE
) (
    input  wire aclk,
    input  wire aresetn,

    input  wire               start,
    input  wire [N_PE*16-1:0] row_count,   // [i*16 +: 16] = row_count for PE i
    output wire               done,

    input  wire [`MAX_DIM_BITS-1:0] M,
    input  wire [`MAX_DIM_BITS-1:0] K,
    input  wire [`MAX_DIM_BITS-1:0] N,

    // Operation mode (broadcast to all PEs): 0=SpGEMM, 1=elementwise; op_sub: add/sub
    input  wire                     op_mode,
    input  wire                     op_sub,

    //=========================================================================
    // A descriptor direct-write ports (per PE, packed)
    input  wire [N_PE-1:0]                        a_desc_we,
    input  wire [N_PE*`A_ROW_ADDR_BITS-1:0]      a_desc_waddr,
    input  wire [N_PE*36-1:0]                     a_desc_wdata,

    // A value/column write ports (per PE, packed)
    input  wire [N_PE-1:0]                    a_val_we,
    input  wire [N_PE*`A_NNZ_ADDR_BITS-1:0]  a_val_waddr,
    input  wire [N_PE*`DATA_WIDTH-1:0]        a_val_wdata,

    input  wire [N_PE-1:0]                    a_col_we,
    input  wire [N_PE*`A_NNZ_ADDR_BITS-1:0]  a_col_waddr,
    input  wire [N_PE*`DATA_WIDTH-1:0]        a_col_wdata,

    //=========================================================================
    // B write ports (broadcast)
    input  wire                          b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_col_wdata,
    input  wire                          b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_val_wdata,

    input  wire                          b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0]  b_desc_waddr,
    input  wire [31:0]                   b_desc_wdata,

    //=========================================================================
    // C bank read ports (per PE, packed).  Each PE owns an independent C bank;
    // addr = {local_row[C_ROW_ADDR_BITS-1:0], gaddr[4:0]}, data = 16 FP16 lanes,
    // c_rd_row = global C row of the addressed local slot.
    input  wire [N_PE-1:0]                          c_rd_en,  //读使能
    input  wire [N_PE*(`C_ROW_ADDR_BITS+5)-1:0]     c_rd_addr,   //读地址
    output wire [N_PE*16*16-1:0]                    c_rd_data,  //读数据  16lane
    output wire [N_PE*`MAX_DIM_BITS-1:0]            c_rd_row  //行号    
);

    localparam C_RD_ADDR_W = `C_ROW_ADDR_BITS + 5;

    wire [N_PE-1:0] done_vec;
    assign done = &done_vec;

    genvar i;
    generate
        for (i = 0; i < N_PE; i = i + 1) begin : gen_pe
            pe_top #(.PE_ID(i)) u_pe (
                .aclk    (aclk),
                .aresetn (aresetn),
                .start      (start),
                .row_count  (row_count[i*16 +: 16]),
                .done       (done_vec[i]),
                .M(M), .K(K), .N(N),
                .op_mode(op_mode), .op_sub(op_sub),

                .a_desc_we   (a_desc_we[i]),
                .a_desc_waddr(a_desc_waddr[i*`A_ROW_ADDR_BITS +: `A_ROW_ADDR_BITS]),
                .a_desc_wdata(a_desc_wdata[i*36 +: 36]),

                .a_val_we    (a_val_we[i]),
                .a_val_waddr (a_val_waddr[i*`A_NNZ_ADDR_BITS +: `A_NNZ_ADDR_BITS]),
                .a_val_wdata (a_val_wdata[i*`DATA_WIDTH +: `DATA_WIDTH]),

                .a_col_we    (a_col_we[i]),
                .a_col_waddr (a_col_waddr[i*`A_NNZ_ADDR_BITS +: `A_NNZ_ADDR_BITS]),
                .a_col_wdata (a_col_wdata[i*`DATA_WIDTH +: `DATA_WIDTH]),

                .b_col_we    (b_col_we),
                .b_col_waddr (b_col_waddr),
                .b_col_wdata (b_col_wdata),
                .b_val_we    (b_val_we),
                .b_val_waddr (b_val_waddr),
                .b_val_wdata (b_val_wdata),

                .b_desc_we   (b_desc_we),
                .b_desc_waddr(b_desc_waddr),
                .b_desc_wdata(b_desc_wdata),

                .c_rd_en   (c_rd_en[i]),
                .c_rd_addr (c_rd_addr[i*C_RD_ADDR_W +: C_RD_ADDR_W]),
                .c_rd_data (c_rd_data[i*16*16 +: 16*16]),
                .c_rd_row  (c_rd_row[i*`MAX_DIM_BITS +: `MAX_DIM_BITS])
            );
        end
    endgenerate

endmodule
