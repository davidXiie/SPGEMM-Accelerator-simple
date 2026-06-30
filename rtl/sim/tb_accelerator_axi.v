//=============================================================================
// tb_accelerator_axi.v — DDR-model testbench wrapper
// Instantiates both accelerator_axi_top and ddr_model, connects AXI bus,
// exposes ddr_model host ports for cocotb pre-load and C readback.
//=============================================================================

`include "defines.vh"

module tb_accelerator_axi;

    localparam N_PE   = `N_PE;
    localparam M_AW   = `MAX_DIM_BITS;
    localparam MEM_AW = 22;

    // Clock & reset
    reg  aclk;
    reg  aresetn;

    // Control
    reg                     start;
    wire                    done;
    reg  [M_AW-1:0]        M, K, N;
    reg                     op_mode, op_sub;

    // === AXI bus (between accelerator_axi_top and ddr_model) ===
    wire [3:0]              ddr_ARID;
    wire [63:0]             ddr_ARADDR;
    wire [7:0]              ddr_ARLEN;
    wire                    ddr_ARVALID;
    wire                    ddr_ARREADY;
    wire [3:0]              ddr_RID;
    wire [511:0]            ddr_RDATA;
    wire [1:0]              ddr_RRESP;
    wire                    ddr_RLAST;
    wire                    ddr_RVALID;
    wire                    ddr_RREADY;

    // === PE C read ports (for cocotb drain) ===
    // cocotb drives c_rd_en/c_rd_addr → reg
    reg  [N_PE-1:0]                      c_rd_en;
    reg  [N_PE*(`C_ROW_ADDR_BITS+5)-1:0] c_rd_addr;
    // PE returns c_rd_data/c_rd_row → wire
    wire [N_PE*16*16-1:0]                c_rd_data;
    wire [N_PE*`MAX_DIM_BITS-1:0]        c_rd_row;

    // === DDR model host ports (cocotb writes/reads) ===
    reg                     host_wr_en;
    reg  [MEM_AW-1:0]      host_wr_addr;
    reg  [15:0]             host_wr_data;
    reg  [MEM_AW-1:0]      host_rd_addr;
    wire [15:0]             host_rd_data;

`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    // ---- Accelerator top (AXI-direct, no global buffer) ----
    accelerator_axi_top #(.N_PE(N_PE), .M_AW(M_AW)) u_accel (
        .clk(aclk), .rst_n(aresetn),
        .start(start), .done(done),
        .M(M), .K(K), .N(N),
        .op_mode(op_mode), .op_sub(op_sub),

        .ddr_ARID   (ddr_ARID),   .ddr_ARADDR (ddr_ARADDR),
        .ddr_ARLEN  (ddr_ARLEN),  .ddr_ARVALID(ddr_ARVALID),
        .ddr_ARREADY(ddr_ARREADY),
        .ddr_RID    (ddr_RID),    .ddr_RDATA  (ddr_RDATA),
        .ddr_RRESP  (ddr_RRESP),  .ddr_RLAST  (ddr_RLAST),
        .ddr_RVALID (ddr_RVALID), .ddr_RREADY (ddr_RREADY),

        .c_rd_en  (c_rd_en),  .c_rd_addr (c_rd_addr),
        .c_rd_data(c_rd_data), .c_rd_row (c_rd_row)
    );

    // ---- DDR model (BRAM + AXI4 Slave) ----
    ddr_model #(.MEM_DEPTH(MEM_AW)) u_ddr (
        .clk(aclk), .rst_n(aresetn),

        .host_wr_en  (host_wr_en),
        .host_wr_addr(host_wr_addr),
        .host_wr_data(host_wr_data),
        .host_rd_addr(host_rd_addr),
        .host_rd_data(host_rd_data),

        .axi_arid   (ddr_ARID),   .axi_araddr (ddr_ARADDR),
        .axi_arlen  (ddr_ARLEN),  .axi_arvalid(ddr_ARVALID),
        .axi_arready(ddr_ARREADY),
        .axi_rid    (ddr_RID),    .axi_rdata  (ddr_RDATA),
        .axi_rresp  (ddr_RRESP),  .axi_rlast  (ddr_RLAST),
        .axi_rvalid (ddr_RVALID), .axi_rready (ddr_RREADY)
    );

    // ---- VCD dump ----
`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/accel_axi_dump.vcd");
        $dumpvars(0, tb_accelerator_axi.aclk);
        $dumpvars(0, tb_accelerator_axi.aresetn);
        $dumpvars(0, tb_accelerator_axi.start);
        $dumpvars(0, tb_accelerator_axi.done);
        $dumpvars(0, tb_accelerator_axi.u_accel.state);
    end
`endif

endmodule
