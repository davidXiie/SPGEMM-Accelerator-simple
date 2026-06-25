//=============================================================================
// File     : pe_cluster.v
// Project  : SPGEMM-Accelerator v2
// Brief    : N_PE-wide PE cluster.  Each PE owns its own C buffer (banked).
//
// Port packing convention (per-PE signals use packed buses):
//   bit/field for PE i is at bus[i*W +: W]  (PE0 at LSB, PEn-1 at MSB)
//
// B is broadcast — one set of write ports drives all PE instances.
// A and instruction buffers are per-PE.
// C readback is per-PE: host reads c_rd_addr into c_rd_data after done.
//=============================================================================

`include "defines.vh"

module pe_cluster #(
    parameter N_PE = `N_PE
) (
    input  wire aclk,
    input  wire aresetn,

    input  wire               start,
    // Per-PE row counts packed: [i*16 +: 16] = row_count for PE i
    input  wire [N_PE*16-1:0] row_count,
    output wire               done,   // AND of all PE dones

    input  wire [`MAX_DIM_BITS-1:0] M,
    input  wire [`MAX_DIM_BITS-1:0] K,
    input  wire [`MAX_DIM_BITS-1:0] N,

    //=========================================================================
    // A write ports (packed, per PE)
    input  wire [N_PE-1:0]                     a_desc_we,
    input  wire [N_PE*`A_ROW_ADDR_BITS-1:0]   a_desc_waddr,
    input  wire [N_PE*64-1:0]                  a_desc_wdata,
    input  wire [N_PE-1:0]                     a_val_we,
    input  wire [N_PE*`A_NNZ_ADDR_BITS-1:0]   a_val_waddr,
    input  wire [N_PE*`DATA_WIDTH-1:0]         a_val_wdata,

    // A column index buffer (per PE, packed)
    input  wire [N_PE-1:0]                     a_col_we,
    input  wire [N_PE*`A_NNZ_ADDR_BITS-1:0]   a_col_waddr,
    input  wire [N_PE*`DATA_WIDTH-1:0]         a_col_wdata,

    //=========================================================================
    // B write ports (broadcast — single set, wired to every PE)
    input  wire                          b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_col_wdata,
    input  wire                          b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_val_wdata,

    // B row descriptor (broadcast; {b_off[31:0], 0[31:16], b_nnz[15:0]})
    input  wire                          b_desc_we,
    input  wire [`MAX_DIM_BITS-1:0]     b_desc_waddr,
    input  wire [63:0]                   b_desc_wdata

    //=========================================================================
    // C buffer read (per PE, packed) — disabled (c_bank removed)
    // input  wire [N_PE-1:0]        c_rd_en,
    // input  wire [N_PE*17-1:0]     c_rd_addr,
    // output wire [N_PE*16-1:0]     c_rd_data   // FP16 per PE
);

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

                .a_desc_we   (a_desc_we[i]),
                .a_desc_waddr(a_desc_waddr[i*`A_ROW_ADDR_BITS +: `A_ROW_ADDR_BITS]),
                .a_desc_wdata(a_desc_wdata[i*64 +: 64]),
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
                .b_desc_wdata(b_desc_wdata)

                // .c_rd_en   (c_rd_en[i]),
                // .c_rd_addr (c_rd_addr[i*17 +: 17]),
                // .c_rd_data (c_rd_data[i*16 +: 16])
            );
        end
    endgenerate

endmodule
