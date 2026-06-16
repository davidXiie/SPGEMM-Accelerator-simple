// Quick debug testbench for core_top state machine
`include "defines.vh"
`include "isa.vh"

module tb_core_top_debug;
    reg aclk;
    reg aresetn;
    reg cr_launch;
    reg [`AXI_ADDR_WIDTH-1:0] ins_baddr;
    reg [15:0] ins_count;
    wire cr_finish;
    wire [15:0] cycle_counter;

    // AXI signals
    wire m_axi_arvalid, m_axi_arready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0] m_axi_arlen;
    wire m_axi_rvalid, m_axi_rready;
    wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata;
    wire m_axi_rlast;
    wire m_axi_awvalid, m_axi_awready, m_axi_wvalid, m_axi_wready, m_axi_wlast, m_axi_bvalid, m_axi_bready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0] m_axi_awlen;
    wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata;
    wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb;
    wire [1:0] m_axi_bresp;

    core_top u_dut (
        .cr_launch(cr_launch),
        .ins_baddr(ins_baddr),
        .ins_count(ins_count),
        .cr_finish(cr_finish),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready),
        .m_axi_rdata(m_axi_rdata),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),
        .m_axi_bresp(m_axi_bresp),
        .cycle_counter(cycle_counter),
        .aclk(aclk),
        .aresetn(aresetn)
    );

    // AXI stubs: ready=1 for simplicity
    assign m_axi_arready = 1'b1;
    assign m_axi_rvalid  = 1'b0;
    assign m_axi_rdata   = 0;
    assign m_axi_rlast   = 1'b0;
    assign m_axi_rready  = 1'b0;
    assign m_axi_awready = 1'b0;
    assign m_axi_wready  = 1'b0;
    assign m_axi_bvalid  = 1'b0;

    always #5 aclk = ~aclk;

    initial begin
        $dumpfile("debug.vcd");
        $dumpvars(0, tb_core_top_debug);
        aclk = 0;
        aresetn = 0;
        cr_launch = 0;
        ins_baddr = 0;
        ins_count = 28;
        #20 aresetn = 1;
        #20 cr_launch = 1;
        $display("[%0t] Launch pulse asserted", $time);
        #10 cr_launch = 0;
        ins_baddr = 0;
        // Run for 500 cycles
        #5000;
        $display("[%0t] Simulation end, state=%0d", $time, u_dut.state);
        $finish;
    end
endmodule
