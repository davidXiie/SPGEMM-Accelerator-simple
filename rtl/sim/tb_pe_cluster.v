//=============================================================================
// File     : tb_pe_cluster.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Cocotb wrapper for the N_PE-wide PE cluster.
//            n_pe_sig is exposed so Python can read N_PE at runtime without
//            hardcoding the count in the test.
//=============================================================================

`include "defines.vh"

module tb_pe_cluster;

    localparam N_PE = `N_PE;

    // Expose N_PE so cocotb can read it without hardcoding the value in Python.
    reg [7:0] n_pe_sig;
    initial n_pe_sig = N_PE;

    reg aclk;
    reg aresetn;
    reg        start;
    wire       done;

    reg [N_PE*16-1:0]  row_count;
    reg [`MAX_DIM_BITS-1:0] M, K, N;

    // A write ports (packed)
    reg [N_PE-1:0]                    a_desc_we;
    reg [N_PE*`A_ROW_ADDR_BITS-1:0]  a_desc_waddr;
    reg [N_PE*64-1:0]                 a_desc_wdata;
    reg [N_PE-1:0]                    a_val_we;
    reg [N_PE*`A_NNZ_ADDR_BITS-1:0]  a_val_waddr;
    reg [N_PE*`DATA_WIDTH-1:0]        a_val_wdata;

    // A column index buffer (packed, per PE)
    reg [N_PE-1:0]                    a_col_we;
    reg [N_PE*`A_NNZ_ADDR_BITS-1:0]  a_col_waddr;
    reg [N_PE*`DATA_WIDTH-1:0]        a_col_wdata;

    // B write ports (broadcast)
    reg                          b_col_we;
    reg [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr;
    reg [`DATA_WIDTH-1:0]        b_col_wdata;
    reg                          b_val_we;
    reg [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr;
    reg [`DATA_WIDTH-1:0]        b_val_wdata;

    // B row descriptor (broadcast)
    reg                          b_desc_we;
    reg [`MAX_DIM_BITS-1:0]     b_desc_waddr;
    reg [63:0]                   b_desc_wdata;

    // C buffer read (per PE, packed) — disabled (c_bank removed)
    // reg  [N_PE-1:0]              c_rd_en;
    // reg  [N_PE*17-1:0]           c_rd_addr;
    // wire [N_PE*16-1:0]           c_rd_data;  // FP16 per PE


`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    pe_cluster #(.N_PE(N_PE)) u_cluster (
        .aclk(aclk), .aresetn(aresetn),
        .start(start), .row_count(row_count), .done(done),
        .M(M), .K(K), .N(N),

        .a_desc_we(a_desc_we), .a_desc_waddr(a_desc_waddr), .a_desc_wdata(a_desc_wdata),
        .a_val_we(a_val_we),   .a_val_waddr(a_val_waddr),   .a_val_wdata(a_val_wdata),
        .a_col_we(a_col_we),   .a_col_waddr(a_col_waddr),   .a_col_wdata(a_col_wdata),

        .b_col_we(b_col_we),   .b_col_waddr(b_col_waddr),   .b_col_wdata(b_col_wdata),
        .b_val_we(b_val_we),   .b_val_waddr(b_val_waddr),   .b_val_wdata(b_val_wdata),
        .b_desc_we(b_desc_we), .b_desc_waddr(b_desc_waddr), .b_desc_wdata(b_desc_wdata)

        // .c_rd_en  (c_rd_en),
        // .c_rd_addr(c_rd_addr),
        // .c_rd_data(c_rd_data)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/pe_cluster_dump.vcd");
        $dumpvars(0, tb_pe_cluster.u_cluster.gen_pe[0].u_pe.state);
    end
`endif

endmodule
