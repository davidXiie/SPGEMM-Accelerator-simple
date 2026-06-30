//=============================================================================
// tb_mmap.v — Minimal testbench for mmap-based DDR simulation.
// Only instantiates accelerator_axi_top, exposes AXI + C ports.
// Python's AXIReadResponder drives R channel, reads C via c_rd_*.
//=============================================================================

`include "defines.vh"

module tb_mmap;

    localparam N_PE   = `N_PE;
    localparam M_AW   = `MAX_DIM_BITS;

    // Clock & reset
    reg  aclk, aresetn;

    // Control
    reg                     start;
    wire                    done;
    reg  [M_AW-1:0]        M, K, N;
    reg                     op_mode, op_sub;

    // AXI Read Address Channel
    wire [3:0]              ddr_ARID;
    wire [63:0]             ddr_ARADDR;
    wire [7:0]              ddr_ARLEN;
    wire                    ddr_ARVALID;
    reg                     ddr_ARREADY;       // driven by Python AXI slave

    // AXI Read Data Channel (slave → master)
    reg  [3:0]              ddr_RID;
    reg  [511:0]            ddr_RDATA;
    reg  [1:0]              ddr_RRESP;
    reg                     ddr_RLAST;
    reg                     ddr_RVALID;
    wire                    ddr_RREADY;

    // PE C read ports
    reg  [N_PE-1:0]                      c_rd_en;
    reg  [N_PE*(`C_ROW_ADDR_BITS+5)-1:0] c_rd_addr;
    wire [N_PE*16*16-1:0]                c_rd_data;
    wire [N_PE*`MAX_DIM_BITS-1:0]        c_rd_row;

    // Clock
`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

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

    // VCD dump
`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/mmap_dump.vcd");
        $dumpvars(0, tb_mmap.aclk);
        $dumpvars(0, tb_mmap.aresetn);
        $dumpvars(0, tb_mmap.start);
        $dumpvars(0, tb_mmap.done);
        $dumpvars(0, tb_mmap.u_accel.state);
    end
`endif

endmodule
