//=============================================================================
// File     : tb_pe_cluster.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Cocotb wrapper for 4-PE cluster test.
//=============================================================================

`include "defines.vh"

module tb_pe_cluster;

    reg aclk;
    reg aresetn;

    reg        start;
    reg [15:0] row_count_0, row_count_1, row_count_2, row_count_3;
    wire       done;

    reg [`MAX_DIM_BITS-1:0] M, K, N;

    // A write ports — PE 0
    reg        a_desc_we_0;
    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr_0;
    reg [63:0] a_desc_wdata_0;
    reg        a_col_we_0;
    reg [`A_NNZ_ADDR_BITS-1:0] a_col_waddr_0;
    reg [`DATA_WIDTH-1:0] a_col_wdata_0;
    reg        a_val_we_0;
    reg [`A_NNZ_ADDR_BITS-1:0] a_val_waddr_0;
    reg [`DATA_WIDTH-1:0] a_val_wdata_0;

    // A write ports — PE 1
    reg        a_desc_we_1;
    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr_1;
    reg [63:0] a_desc_wdata_1;
    reg        a_col_we_1;
    reg [`A_NNZ_ADDR_BITS-1:0] a_col_waddr_1;
    reg [`DATA_WIDTH-1:0] a_col_wdata_1;
    reg        a_val_we_1;
    reg [`A_NNZ_ADDR_BITS-1:0] a_val_waddr_1;
    reg [`DATA_WIDTH-1:0] a_val_wdata_1;

    // A write ports — PE 2
    reg        a_desc_we_2;
    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr_2;
    reg [63:0] a_desc_wdata_2;
    reg        a_col_we_2;
    reg [`A_NNZ_ADDR_BITS-1:0] a_col_waddr_2;
    reg [`DATA_WIDTH-1:0] a_col_wdata_2;
    reg        a_val_we_2;
    reg [`A_NNZ_ADDR_BITS-1:0] a_val_waddr_2;
    reg [`DATA_WIDTH-1:0] a_val_wdata_2;

    // A write ports — PE 3
    reg        a_desc_we_3;
    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr_3;
    reg [63:0] a_desc_wdata_3;
    reg        a_col_we_3;
    reg [`A_NNZ_ADDR_BITS-1:0] a_col_waddr_3;
    reg [`DATA_WIDTH-1:0] a_col_wdata_3;
    reg        a_val_we_3;
    reg [`A_NNZ_ADDR_BITS-1:0] a_val_waddr_3;
    reg [`DATA_WIDTH-1:0] a_val_wdata_3;

    // B write ports (broadcast)
    reg        b_desc_we;
    reg [`B_ROW_ADDR_BITS-1:0] b_desc_waddr;
    reg [63:0] b_desc_wdata;
    reg        b_col_we;
    reg [`B_NNZ_ADDR_BITS-1:0] b_col_waddr;
    reg [`DATA_WIDTH-1:0] b_col_wdata;
    reg        b_val_we;
    reg [`B_NNZ_ADDR_BITS-1:0] b_val_waddr;
    reg [`DATA_WIDTH-1:0] b_val_wdata;

    // C output ports — 4 PEs
    wire       cbuf_wr_valid_0; reg cbuf_wr_ready_0;
    wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr_0;
    wire [`DATA_WIDTH-1:0] cbuf_wr_data_0;

    wire       cbuf_wr_valid_1; reg cbuf_wr_ready_1;
    wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr_1;
    wire [`DATA_WIDTH-1:0] cbuf_wr_data_1;

    wire       cbuf_wr_valid_2; reg cbuf_wr_ready_2;
    wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr_2;
    wire [`DATA_WIDTH-1:0] cbuf_wr_data_2;

    wire       cbuf_wr_valid_3; reg cbuf_wr_ready_3;
    wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr_3;
    wire [`DATA_WIDTH-1:0] cbuf_wr_data_3;

`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    pe_cluster u_cluster (
        .aclk(aclk), .aresetn(aresetn),
        .start(start),
        .row_count_0(row_count_0), .row_count_1(row_count_1),
        .row_count_2(row_count_2), .row_count_3(row_count_3),
        .done(done),
        .M(M), .K(K), .N(N),

        .a_desc_we_0(a_desc_we_0), .a_desc_waddr_0(a_desc_waddr_0), .a_desc_wdata_0(a_desc_wdata_0),
        .a_col_we_0(a_col_we_0),   .a_col_waddr_0(a_col_waddr_0),   .a_col_wdata_0(a_col_wdata_0),
        .a_val_we_0(a_val_we_0),   .a_val_waddr_0(a_val_waddr_0),   .a_val_wdata_0(a_val_wdata_0),

        .a_desc_we_1(a_desc_we_1), .a_desc_waddr_1(a_desc_waddr_1), .a_desc_wdata_1(a_desc_wdata_1),
        .a_col_we_1(a_col_we_1),   .a_col_waddr_1(a_col_waddr_1),   .a_col_wdata_1(a_col_wdata_1),
        .a_val_we_1(a_val_we_1),   .a_val_waddr_1(a_val_waddr_1),   .a_val_wdata_1(a_val_wdata_1),

        .a_desc_we_2(a_desc_we_2), .a_desc_waddr_2(a_desc_waddr_2), .a_desc_wdata_2(a_desc_wdata_2),
        .a_col_we_2(a_col_we_2),   .a_col_waddr_2(a_col_waddr_2),   .a_col_wdata_2(a_col_wdata_2),
        .a_val_we_2(a_val_we_2),   .a_val_waddr_2(a_val_waddr_2),   .a_val_wdata_2(a_val_wdata_2),

        .a_desc_we_3(a_desc_we_3), .a_desc_waddr_3(a_desc_waddr_3), .a_desc_wdata_3(a_desc_wdata_3),
        .a_col_we_3(a_col_we_3),   .a_col_waddr_3(a_col_waddr_3),   .a_col_wdata_3(a_col_wdata_3),
        .a_val_we_3(a_val_we_3),   .a_val_waddr_3(a_val_waddr_3),   .a_val_wdata_3(a_val_wdata_3),

        .b_desc_we(b_desc_we), .b_desc_waddr(b_desc_waddr), .b_desc_wdata(b_desc_wdata),
        .b_col_we(b_col_we),   .b_col_waddr(b_col_waddr),   .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),   .b_val_waddr(b_val_waddr),   .b_val_wdata(b_val_wdata),

        .cbuf_wr_valid_0(cbuf_wr_valid_0), .cbuf_wr_ready_0(cbuf_wr_ready_0),
        .cbuf_wr_addr_0(cbuf_wr_addr_0),   .cbuf_wr_data_0(cbuf_wr_data_0),

        .cbuf_wr_valid_1(cbuf_wr_valid_1), .cbuf_wr_ready_1(cbuf_wr_ready_1),
        .cbuf_wr_addr_1(cbuf_wr_addr_1),   .cbuf_wr_data_1(cbuf_wr_data_1),

        .cbuf_wr_valid_2(cbuf_wr_valid_2), .cbuf_wr_ready_2(cbuf_wr_ready_2),
        .cbuf_wr_addr_2(cbuf_wr_addr_2),   .cbuf_wr_data_2(cbuf_wr_data_2),

        .cbuf_wr_valid_3(cbuf_wr_valid_3), .cbuf_wr_ready_3(cbuf_wr_ready_3),
        .cbuf_wr_addr_3(cbuf_wr_addr_3),   .cbuf_wr_data_3(cbuf_wr_data_3)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/pe_cluster_dump.vcd");
        $dumpvars(0, tb_pe_cluster.u_cluster.u_pe_0.state);
        $dumpvars(0, tb_pe_cluster.u_cluster.u_pe_1.state);
        $dumpvars(0, tb_pe_cluster.u_cluster.u_pe_2.state);
        $dumpvars(0, tb_pe_cluster.u_cluster.u_pe_3.state);
    end
`endif

endmodule
