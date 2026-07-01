//=============================================================================
// tb_mmap.v — Minimal testbench for mmap-based DDR simulation.
// Instantiates accelerator_axi_top, exposes AXI Read + Write + C ports.
// Python's AXIResponder drives R/B channels, accepts AR/AW/W channels.
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

    // === AXI4 Read Address Channel ===
    wire [3:0]              ddr_ARID;
    wire [63:0]             ddr_ARADDR;
    wire [7:0]              ddr_ARLEN;
    wire [2:0]              ddr_ARSIZE;
    wire                    ddr_ARVALID;
    reg                     ddr_ARREADY;       // driven by Python AXI slave

    // === AXI4 Read Data Channel (slave → master) ===
    reg  [3:0]              ddr_RID;
    reg  [511:0]            ddr_RDATA;
    reg  [1:0]              ddr_RRESP;
    reg                     ddr_RLAST;
    reg                     ddr_RVALID;
    wire                    ddr_RREADY;

    // === AXI4 Write Address Channel ===
    wire [3:0]              ddr_AWID;
    wire [63:0]             ddr_AWADDR;
    wire [7:0]              ddr_AWLEN;
    wire [2:0]              ddr_AWSIZE;
    wire                    ddr_AWVALID;
    reg                     ddr_AWREADY;       // driven by Python AXI slave

    // === AXI4 Write Data Channel ===
    wire [511:0]            ddr_WDATA;
    wire [63:0]             ddr_WSTRB;
    wire                    ddr_WLAST;
    wire                    ddr_WVALID;
    reg                     ddr_WREADY;        // driven by Python AXI slave

    // === AXI4 Write Response Channel ===
    reg  [3:0]              ddr_BID;
    reg  [1:0]              ddr_BRESP;
    reg                     ddr_BVALID;
    wire                    ddr_BREADY;

    // === PE C read ports (for cocotb simulation / debug) ===
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

        // AXI4 Read
        .ddr_ARID   (ddr_ARID),   .ddr_ARADDR (ddr_ARADDR),
        .ddr_ARLEN  (ddr_ARLEN),  .ddr_ARSIZE (ddr_ARSIZE),
        .ddr_ARVALID(ddr_ARVALID), .ddr_ARREADY(ddr_ARREADY),
        .ddr_RID    (ddr_RID),    .ddr_RDATA  (ddr_RDATA),
        .ddr_RRESP  (ddr_RRESP),  .ddr_RLAST  (ddr_RLAST),
        .ddr_RVALID (ddr_RVALID), .ddr_RREADY (ddr_RREADY),

        // AXI4 Write
        .ddr_AWID   (ddr_AWID),   .ddr_AWADDR (ddr_AWADDR),
        .ddr_AWLEN  (ddr_AWLEN),  .ddr_AWSIZE (ddr_AWSIZE),
        .ddr_AWVALID(ddr_AWVALID), .ddr_AWREADY(ddr_AWREADY),
        .ddr_WDATA  (ddr_WDATA),  .ddr_WSTRB  (ddr_WSTRB),
        .ddr_WLAST  (ddr_WLAST),  .ddr_WVALID (ddr_WVALID),
        .ddr_WREADY (ddr_WREADY),
        .ddr_BID    (ddr_BID),    .ddr_BRESP  (ddr_BRESP),
        .ddr_BVALID (ddr_BVALID), .ddr_BREADY (ddr_BREADY),

        // C read
        .c_rd_en  (c_rd_en),  .c_rd_addr (c_rd_addr),
        .c_rd_data(c_rd_data), .c_rd_row (c_rd_row)
    );

endmodule
