//=============================================================================
// File     : tb_pe_cluster.v
// Brief    : Cocotb wrapper for the N_PE-wide PE cluster (N_PE <= 8).
//
//   Per-PE A descriptor streaming signals are exposed with individual names
//   (a_desc_valid_0 .. a_desc_valid_7) so Cocotb can drive them via
//   getattr(dut, f"a_desc_valid_{pid}").  Only the first N_PE signals are
//   wired into the cluster; the rest are declared but unused.
//
//   To change PE count: edit `define N_PE in defines.vh (1..8).
//   No changes to this file are needed.
//=============================================================================

`include "defines.vh"

module tb_pe_cluster;

    localparam N_PE = `N_PE;

    reg [7:0] n_pe_sig;
    initial n_pe_sig = N_PE;

    reg aclk;
    reg aresetn;
    reg  start;
    wire done;

    reg [N_PE*16-1:0]         row_count;
    reg [`MAX_DIM_BITS-1:0]   M, K, N;

    reg op_mode;   // 0 = SpGEMM, 1 = elementwise
    reg op_sub;    // elementwise: 0 = add, 1 = subtract

    //=========================================================================
    // Per-PE A descriptor direct-write signals.
    // Only indices 0..N_PE-1 are connected to the cluster.
    //=========================================================================
    reg  a_desc_we_0, a_desc_we_1, a_desc_we_2, a_desc_we_3;
    reg  a_desc_we_4, a_desc_we_5, a_desc_we_6, a_desc_we_7;

    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr_0, a_desc_waddr_1, a_desc_waddr_2, a_desc_waddr_3;
    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr_4, a_desc_waddr_5, a_desc_waddr_6, a_desc_waddr_7;

    reg [35:0] a_desc_wdata_0, a_desc_wdata_1, a_desc_wdata_2, a_desc_wdata_3;
    reg [35:0] a_desc_wdata_4, a_desc_wdata_5, a_desc_wdata_6, a_desc_wdata_7;

    // Collect into N_PE-wide packed buses (N_PE=3 from defines.vh).
    wire [N_PE-1:0] a_desc_we_bus;
    assign a_desc_we_bus[0] = a_desc_we_0;
    assign a_desc_we_bus[1] = a_desc_we_1;
    assign a_desc_we_bus[2] = a_desc_we_2;

    wire [N_PE*`A_ROW_ADDR_BITS-1:0] a_desc_waddr_bus;
    assign a_desc_waddr_bus[0*`A_ROW_ADDR_BITS +: `A_ROW_ADDR_BITS] = a_desc_waddr_0;
    assign a_desc_waddr_bus[1*`A_ROW_ADDR_BITS +: `A_ROW_ADDR_BITS] = a_desc_waddr_1;
    assign a_desc_waddr_bus[2*`A_ROW_ADDR_BITS +: `A_ROW_ADDR_BITS] = a_desc_waddr_2;

    wire [N_PE*36-1:0] a_desc_wdata_bus;
    assign a_desc_wdata_bus[0*36 +: 36] = a_desc_wdata_0;
    assign a_desc_wdata_bus[1*36 +: 36] = a_desc_wdata_1;
    assign a_desc_wdata_bus[2*36 +: 36] = a_desc_wdata_2;

    //=========================================================================
    // A value / column write ports (per PE, packed)
    //=========================================================================
    reg [N_PE-1:0]                    a_val_we;
    reg [N_PE*`A_NNZ_ADDR_BITS-1:0]  a_val_waddr;
    reg [N_PE*`DATA_WIDTH-1:0]        a_val_wdata;

    reg [N_PE-1:0]                    a_col_we;
    reg [N_PE*`A_NNZ_ADDR_BITS-1:0]  a_col_waddr;
    reg [N_PE*`DATA_WIDTH-1:0]        a_col_wdata;

    //=========================================================================
    // B write ports (broadcast)
    //=========================================================================
    reg                          b_col_we;
    reg [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr;
    reg [`DATA_WIDTH-1:0]        b_col_wdata;
    reg                          b_val_we;
    reg [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr;
    reg [`DATA_WIDTH-1:0]        b_val_wdata;

    reg                          b_desc_we;
    reg [`B_ROW_ADDR_BITS-1:0]  b_desc_waddr;
    reg [31:0]                   b_desc_wdata;

    //=========================================================================
    // C bank read ports (per PE, packed).  Cocotb drives the whole bus,
    // setting only the target PE's field per read.
    //=========================================================================
    localparam C_RD_ADDR_W = `C_ROW_ADDR_BITS + 5;
    reg  [N_PE-1:0]                  c_rd_en;
    reg  [N_PE*C_RD_ADDR_W-1:0]     c_rd_addr;
    wire [N_PE*16*16-1:0]           c_rd_data;
    wire [N_PE*`MAX_DIM_BITS-1:0]   c_rd_row;

`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    pe_cluster #(.N_PE(N_PE)) u_cluster (
        .aclk(aclk), .aresetn(aresetn),
        .start(start), .row_count(row_count), .done(done),
        .M(M), .K(K), .N(N),
        .op_mode(op_mode), .op_sub(op_sub),

        .a_desc_we   (a_desc_we_bus),
        .a_desc_waddr(a_desc_waddr_bus),
        .a_desc_wdata(a_desc_wdata_bus),

        .a_val_we(a_val_we),   .a_val_waddr(a_val_waddr),   .a_val_wdata(a_val_wdata),
        .a_col_we(a_col_we),   .a_col_waddr(a_col_waddr),   .a_col_wdata(a_col_wdata),

        .b_col_we(b_col_we),   .b_col_waddr(b_col_waddr),   .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),   .b_val_waddr(b_val_waddr),   .b_val_wdata(b_val_wdata),
        .b_desc_we(b_desc_we), .b_desc_waddr(b_desc_waddr), .b_desc_wdata(b_desc_wdata),

        .c_rd_en(c_rd_en), .c_rd_addr(c_rd_addr), .c_rd_data(c_rd_data),
        .c_rd_row(c_rd_row)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/pe_cluster_dump.vcd");
        $dumpvars(0, tb_pe_cluster);
    end
`endif

endmodule
