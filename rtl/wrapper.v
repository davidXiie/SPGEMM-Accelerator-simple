//=============================================================================
// File     : wrapper.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Top-level wrapper — CR slave + Core_top + AXI interfaces
//            Simplified: no ISA, fixed DDR addresses, dense C output
//=============================================================================

`include "defines.vh"

module spgemm_accelerator (
    // AXI-Lite Slave: CPU control interface
    input  wire                      s_axi_aclk,
    input  wire                      s_axi_aresetn,
    // Write
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [15:0]               s_axi_awaddr,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    input  wire [31:0]               s_axi_wdata,
    input  wire [3:0]                s_axi_wstrb,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,
    output wire [1:0]                s_axi_bresp,
    // Read
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    input  wire [15:0]               s_axi_araddr,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,
    output wire [31:0]               s_axi_rdata,
    output wire [1:0]                s_axi_rresp,

    // AXI-Full Master: DRAM interface
    // Read Address
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,
    output wire [`AXI_ID_WIDTH-1:0]  m_axi_arid,
    output wire [2:0]                m_axi_arsize,
    output wire [1:0]                m_axi_arburst,
    // Read Data
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [1:0]                m_axi_rresp,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,
    input  wire [`AXI_ID_WIDTH-1:0]  m_axi_rid,
    // Write Address
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,
    output wire [`AXI_ID_WIDTH-1:0]  m_axi_awid,
    // Write Data
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    output wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,
    // Write Response
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    input  wire [1:0]                m_axi_bresp,
    input  wire [`AXI_ID_WIDTH-1:0]  m_axi_bid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // Control Register signals (6 × 32-bit registers)
    //   Reg 0 (0x00): Control [bit0=start, bit1=clear]
    //   Reg 1 (0x04): Status  [bit0=done, bit1=busy, bit2=error]
    //   Reg 2 (0x08): M (A rows)
    //   Reg 3 (0x0C): K (A cols = B rows)
    //   Reg 4 (0x10): N (B cols)
    //   Reg 5 (0x14): pe_valid_mask [7:0]
    //=========================================================================
    localparam REG_COUNT = 6;
    localparam REG_BITS  = 32;

    wire [REG_COUNT*REG_BITS-1:0] cr_reg_out;
    wire [REG_COUNT*REG_BITS-1:0] cr_reg_in;
    wire [REG_COUNT-1:0]          cr_wr_pulse;
    wire [REG_COUNT-1:0]          cr_rd_valid;

    // Extract control fields
    wire        cr_start       = cr_reg_out[0];
    wire        cr_clear       = cr_reg_out[1];
    wire [15:0] cr_M           = cr_reg_out[79:64];   // Reg2
    wire [15:0] cr_K           = cr_reg_out[111:96];  // Reg3
    wire [15:0] cr_N           = cr_reg_out[143:128]; // Reg4
    wire [7:0]  cr_pe_mask     = cr_reg_out[167:160]; // Reg5

    wire core_finish;
    wire core_busy;

    // CR Reg In: status feedback
    assign cr_reg_in[31:0]    = cr_reg_out[31:0];       // echo control
    assign cr_reg_in[63:32]   = {30'b0, core_finish, core_busy}; // status
    assign cr_reg_in[95:64]   = cr_reg_out[95:64];      // echo M
    assign cr_reg_in[127:96]  = cr_reg_out[127:96];     // echo K
    assign cr_reg_in[159:128] = cr_reg_out[159:128];    // echo N
    assign cr_reg_in[191:160] = 32'd0;

    //=========================================================================
    // CR Slave (AXI-Lite)
    //=========================================================================
    cr_slave #(
        .REG_COUNT(REG_COUNT),
        .REG_BITS(REG_BITS)
    ) u_cr_slave (
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .reg_out       (cr_reg_out),
        .reg_in        (cr_reg_in),
        .reg_wr_pulse  (cr_wr_pulse),
        .reg_rd_valid  (cr_rd_valid),
        .s_axi_aclk    (aclk),
        .s_axi_aresetn (aresetn)
    );

    //=========================================================================
    // Core (SpGEMM Accelerator v2)
    //=========================================================================
    core_top u_core_top (
        .cr_start       (cr_start),
        .cr_clear       (cr_clear),
        .M              (cr_M[`MAX_DIM_BITS-1:0]),
        .K              (cr_K[`MAX_DIM_BITS-1:0]),
        .N              (cr_N[`MAX_DIM_BITS-1:0]),
        .pe_valid_mask  (cr_pe_mask),
        .cr_finish      (core_finish),
        .cr_busy        (core_busy),

        // AXI Read Master
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rlast    (m_axi_rlast),

        // AXI Write Master
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_bresp    (m_axi_bresp),

        .cycle_counter  (),

        .aclk           (aclk),
        .aresetn        (aresetn)
    );

    // AXI IDs
    assign m_axi_arid    = 4'b0;
    assign m_axi_arsize  = 3'b110;  // 64 bytes per beat
    assign m_axi_arburst = 2'b01;   // INCR
    assign m_axi_awid    = 4'b0;

endmodule
