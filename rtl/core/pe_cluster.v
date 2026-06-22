//=============================================================================
// File     : pe_cluster.v
// Project  : SPGEMM-Accelerator v2
// Brief    : 4-PE cluster. A rows are distributed round-robin (row % 4 == PE_ID).
//            B is broadcast (same data written to all PEs via shared write ports).
//            Each PE produces its own cbuf_wr output; addresses are global because
//            the A row descriptor [15:0] carries the global row ID.
//=============================================================================

`include "defines.vh"

module pe_cluster (
    input  wire aclk,
    input  wire aresetn,

    // Control — one start/done pair; all PEs start together
    input  wire        start,
    // Per-PE row counts (number of A rows assigned to each PE)
    input  wire [15:0] row_count_0,
    input  wire [15:0] row_count_1,
    input  wire [15:0] row_count_2,
    input  wire [15:0] row_count_3,
    output wire        done,            // high when ALL PEs are done

    // Matrix dimensions (shared)
    input  wire [`MAX_DIM_BITS-1:0] M,
    input  wire [`MAX_DIM_BITS-1:0] K,
    input  wire [`MAX_DIM_BITS-1:0] N,

    //=========================================================================
    // A write ports — separate per PE
    // PE 0
    input  wire                          a_desc_we_0,
    input  wire [`A_ROW_ADDR_BITS-1:0]  a_desc_waddr_0,
    input  wire [63:0]                   a_desc_wdata_0,
    input  wire                          a_col_we_0,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_col_waddr_0,
    input  wire [`DATA_WIDTH-1:0]        a_col_wdata_0,
    input  wire                          a_val_we_0,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr_0,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata_0,
    // PE 1
    input  wire                          a_desc_we_1,
    input  wire [`A_ROW_ADDR_BITS-1:0]  a_desc_waddr_1,
    input  wire [63:0]                   a_desc_wdata_1,
    input  wire                          a_col_we_1,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_col_waddr_1,
    input  wire [`DATA_WIDTH-1:0]        a_col_wdata_1,
    input  wire                          a_val_we_1,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr_1,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata_1,
    // PE 2
    input  wire                          a_desc_we_2,
    input  wire [`A_ROW_ADDR_BITS-1:0]  a_desc_waddr_2,
    input  wire [63:0]                   a_desc_wdata_2,
    input  wire                          a_col_we_2,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_col_waddr_2,
    input  wire [`DATA_WIDTH-1:0]        a_col_wdata_2,
    input  wire                          a_val_we_2,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr_2,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata_2,
    // PE 3
    input  wire                          a_desc_we_3,
    input  wire [`A_ROW_ADDR_BITS-1:0]  a_desc_waddr_3,
    input  wire [63:0]                   a_desc_wdata_3,
    input  wire                          a_col_we_3,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_col_waddr_3,
    input  wire [`DATA_WIDTH-1:0]        a_col_wdata_3,
    input  wire                          a_val_we_3,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr_3,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata_3,

    //=========================================================================
    // B write ports — broadcast (same data goes to all 4 PEs simultaneously)
    input  wire                          b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0]  b_desc_waddr,
    input  wire [63:0]                   b_desc_wdata,
    input  wire                          b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_col_wdata,
    input  wire                          b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_val_wdata,

    //=========================================================================
    // C dense buffer write — separate per PE (non-overlapping row addresses)
    output wire                              cbuf_wr_valid_0,
    input  wire                              cbuf_wr_ready_0,
    output wire [`C_DENSE_DEPTH_LOG-1:0]    cbuf_wr_addr_0,
    output wire [`DATA_WIDTH-1:0]           cbuf_wr_data_0,

    output wire                              cbuf_wr_valid_1,
    input  wire                              cbuf_wr_ready_1,
    output wire [`C_DENSE_DEPTH_LOG-1:0]    cbuf_wr_addr_1,
    output wire [`DATA_WIDTH-1:0]           cbuf_wr_data_1,

    output wire                              cbuf_wr_valid_2,
    input  wire                              cbuf_wr_ready_2,
    output wire [`C_DENSE_DEPTH_LOG-1:0]    cbuf_wr_addr_2,
    output wire [`DATA_WIDTH-1:0]           cbuf_wr_data_2,

    output wire                              cbuf_wr_valid_3,
    input  wire                              cbuf_wr_ready_3,
    output wire [`C_DENSE_DEPTH_LOG-1:0]    cbuf_wr_addr_3,
    output wire [`DATA_WIDTH-1:0]           cbuf_wr_data_3
);

    wire done_0, done_1, done_2, done_3;
    assign done = done_0 & done_1 & done_2 & done_3;

    pe_top #(.PE_ID(0)) u_pe_0 (
        .aclk(aclk), .aresetn(aresetn),
        .start(start), .row_count(row_count_0), .done(done_0),
        .M(M), .K(K), .N(N),
        .a_desc_we(a_desc_we_0), .a_desc_waddr(a_desc_waddr_0), .a_desc_wdata(a_desc_wdata_0),
        .a_col_we(a_col_we_0),   .a_col_waddr(a_col_waddr_0),   .a_col_wdata(a_col_wdata_0),
        .a_val_we(a_val_we_0),   .a_val_waddr(a_val_waddr_0),   .a_val_wdata(a_val_wdata_0),
        .b_desc_we(b_desc_we),   .b_desc_waddr(b_desc_waddr),   .b_desc_wdata(b_desc_wdata),
        .b_col_we(b_col_we),     .b_col_waddr(b_col_waddr),     .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),     .b_val_waddr(b_val_waddr),     .b_val_wdata(b_val_wdata),
        .cbuf_wr_valid(cbuf_wr_valid_0), .cbuf_wr_ready(cbuf_wr_ready_0),
        .cbuf_wr_addr(cbuf_wr_addr_0),   .cbuf_wr_data(cbuf_wr_data_0)
    );

    pe_top #(.PE_ID(1)) u_pe_1 (
        .aclk(aclk), .aresetn(aresetn),
        .start(start), .row_count(row_count_1), .done(done_1),
        .M(M), .K(K), .N(N),
        .a_desc_we(a_desc_we_1), .a_desc_waddr(a_desc_waddr_1), .a_desc_wdata(a_desc_wdata_1),
        .a_col_we(a_col_we_1),   .a_col_waddr(a_col_waddr_1),   .a_col_wdata(a_col_wdata_1),
        .a_val_we(a_val_we_1),   .a_val_waddr(a_val_waddr_1),   .a_val_wdata(a_val_wdata_1),
        .b_desc_we(b_desc_we),   .b_desc_waddr(b_desc_waddr),   .b_desc_wdata(b_desc_wdata),
        .b_col_we(b_col_we),     .b_col_waddr(b_col_waddr),     .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),     .b_val_waddr(b_val_waddr),     .b_val_wdata(b_val_wdata),
        .cbuf_wr_valid(cbuf_wr_valid_1), .cbuf_wr_ready(cbuf_wr_ready_1),
        .cbuf_wr_addr(cbuf_wr_addr_1),   .cbuf_wr_data(cbuf_wr_data_1)
    );

    pe_top #(.PE_ID(2)) u_pe_2 (
        .aclk(aclk), .aresetn(aresetn),
        .start(start), .row_count(row_count_2), .done(done_2),
        .M(M), .K(K), .N(N),
        .a_desc_we(a_desc_we_2), .a_desc_waddr(a_desc_waddr_2), .a_desc_wdata(a_desc_wdata_2),
        .a_col_we(a_col_we_2),   .a_col_waddr(a_col_waddr_2),   .a_col_wdata(a_col_wdata_2),
        .a_val_we(a_val_we_2),   .a_val_waddr(a_val_waddr_2),   .a_val_wdata(a_val_wdata_2),
        .b_desc_we(b_desc_we),   .b_desc_waddr(b_desc_waddr),   .b_desc_wdata(b_desc_wdata),
        .b_col_we(b_col_we),     .b_col_waddr(b_col_waddr),     .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),     .b_val_waddr(b_val_waddr),     .b_val_wdata(b_val_wdata),
        .cbuf_wr_valid(cbuf_wr_valid_2), .cbuf_wr_ready(cbuf_wr_ready_2),
        .cbuf_wr_addr(cbuf_wr_addr_2),   .cbuf_wr_data(cbuf_wr_data_2)
    );

    pe_top #(.PE_ID(3)) u_pe_3 (
        .aclk(aclk), .aresetn(aresetn),
        .start(start), .row_count(row_count_3), .done(done_3),
        .M(M), .K(K), .N(N),
        .a_desc_we(a_desc_we_3), .a_desc_waddr(a_desc_waddr_3), .a_desc_wdata(a_desc_wdata_3),
        .a_col_we(a_col_we_3),   .a_col_waddr(a_col_waddr_3),   .a_col_wdata(a_col_wdata_3),
        .a_val_we(a_val_we_3),   .a_val_waddr(a_val_waddr_3),   .a_val_wdata(a_val_wdata_3),
        .b_desc_we(b_desc_we),   .b_desc_waddr(b_desc_waddr),   .b_desc_wdata(b_desc_wdata),
        .b_col_we(b_col_we),     .b_col_waddr(b_col_waddr),     .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),     .b_val_waddr(b_val_waddr),     .b_val_wdata(b_val_wdata),
        .cbuf_wr_valid(cbuf_wr_valid_3), .cbuf_wr_ready(cbuf_wr_ready_3),
        .cbuf_wr_addr(cbuf_wr_addr_3),   .cbuf_wr_data(cbuf_wr_data_3)
    );

endmodule
