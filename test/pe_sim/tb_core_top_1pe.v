//=============================================================================
// File     : tb_core_top_1pe.v
// Brief    : Cocotb AXI4 Slave DDR model for core_top_1pe test.
//
//   Simple behavioral DDR model:
//   - 512-bit data width, 64-bit address
//   - AR→R: reads memory, supports burst up to 256 beats
//   - AW+W→B: writes memory, single beat per AW transaction
//   - Memory: 1 MB (0x00000000 ~ 0x000FFFFF bytes)
//=============================================================================

`include "defines.vh"

module tb_core_top_1pe;

    // Clock and reset
    reg aclk;
    reg aresetn;
    wire cr_finish;
    wire cr_busy;

    // Core control
    reg  cr_start;
    reg  [`MAX_DIM_BITS-1:0] M, K, N;

    wire [15:0] cycle_counter;

    // AXI Read
    wire m_axi_arvalid;
    reg  m_axi_arready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0] m_axi_arlen;
    reg  m_axi_rvalid;
    wire m_axi_rready;
    reg  [`AXI_DATA_WIDTH-1:0] m_axi_rdata;
    reg  m_axi_rlast;

    // AXI Write
    wire m_axi_awvalid;
    reg  m_axi_awready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0] m_axi_awlen;
    wire m_axi_wvalid;
    reg  m_axi_wready;
    wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata;
    wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb;
    wire m_axi_wlast;
    reg  m_axi_bvalid;
    wire m_axi_bready;
    reg  [1:0] m_axi_bresp;

    //=========================================================================
    // DDR Memory Model
    //   Simple byte-addressable memory, 1 MB, 512-bit wide access.
    //   Higher 20 bits of address are ignored; only lower 20 bits used.
    //=========================================================================
    localparam MEM_DEPTH = 1024 * 1024 / (`AXI_DATA_WIDTH / 8);  // 16384 entries
    reg [`AXI_DATA_WIDTH-1:0] ddr_mem [0:MEM_DEPTH-1];

    integer _mi;
    initial begin
        for (_mi = 0; _mi < MEM_DEPTH; _mi = _mi + 1)
            ddr_mem[_mi] = 0;
    end

    // Read state
    reg [7:0]         rd_beat_cnt;
    reg [`AXI_ADDR_WIDTH-1:0] rd_addr;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axi_arready <= 1'b0;
            m_axi_rvalid  <= 1'b0;
            m_axi_rdata   <= 0;
            m_axi_rlast   <= 1'b0;
            rd_beat_cnt   <= 0;
            rd_addr       <= 0;
        end else begin
            // AR handshake: accept request
            if (m_axi_arvalid && m_axi_arready) begin
                m_axi_arready <= 1'b0;
                m_axi_rvalid  <= 1'b0;
            end
            if (m_axi_arvalid && !m_axi_arready && !m_axi_rvalid) begin
                m_axi_arready <= 1'b1;
                rd_addr       <= m_axi_araddr;
                rd_beat_cnt   <= 0;
                m_axi_rvalid  <= 1'b0;
            end

            // R channel: send data beats
            if (m_axi_arvalid && m_axi_arready) begin
                m_axi_rvalid <= 1'b1;
                m_axi_rdata  <= ddr_mem[rd_addr[19:6]];  // addr >> 6 (64-byte align)
                if (rd_beat_cnt == m_axi_arlen) begin
                    m_axi_rlast <= 1'b1;
                end else begin
                    rd_beat_cnt <= rd_beat_cnt + 1;
                    rd_addr     <= rd_addr + (`AXI_DATA_WIDTH / 8);
                    m_axi_rlast <= 1'b0;
                end
            end
            if (m_axi_rvalid && m_axi_rready) begin
                if (m_axi_rlast) begin
                    m_axi_rvalid <= 1'b0;
                    m_axi_rlast  <= 1'b0;
                end else begin
                    m_axi_rdata  <= ddr_mem[rd_addr[19:6]];
                    rd_addr      <= rd_addr + (`AXI_DATA_WIDTH / 8);
                    rd_beat_cnt  <= rd_beat_cnt + 1;
                end
            end
        end
    end

    // Write state
    reg [`AXI_ADDR_WIDTH-1:0] wr_addr;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axi_awready <= 1'b0;
            m_axi_wready  <= 1'b0;
            m_axi_bvalid  <= 1'b0;
            m_axi_bresp   <= 2'b00;
            wr_addr       <= 0;
        end else begin
            m_axi_awready <= 1'b1;
            m_axi_wready  <= 1'b1;
            m_axi_bvalid  <= 1'b0;

            if (m_axi_awvalid && m_axi_awready) begin
                wr_addr <= m_axi_awaddr;
            end

            if (m_axi_wvalid && m_axi_wready) begin
                ddr_mem[wr_addr[19:6]] <= m_axi_wdata;
                if (m_axi_bvalid) begin
                    // stagger B response by 1 cycle
                end else begin
                    m_axi_bvalid <= 1'b1;
                end
            end

            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid <= 1'b0;
            end
        end
    end

    //=========================================================================
    // Clock
    //=========================================================================
`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    //=========================================================================
    // DUT
    //=========================================================================
    core_top_1pe u_dut (
        .cr_start     (cr_start),
        .cr_clear     (1'b0),
        .M(M), .K(K), .N(N),
        .cr_finish    (cr_finish),
        .cr_busy      (cr_busy),

        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_araddr (m_axi_araddr),  .m_axi_arlen  (m_axi_arlen),
        .m_axi_rvalid (m_axi_rvalid),  .m_axi_rready (m_axi_rready),
        .m_axi_rdata  (m_axi_rdata),   .m_axi_rlast  (m_axi_rlast),

        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_awaddr (m_axi_awaddr),  .m_axi_awlen  (m_axi_awlen),
        .m_axi_wvalid (m_axi_wvalid),  .m_axi_wready (m_axi_wready),
        .m_axi_wdata  (m_axi_wdata),   .m_axi_wstrb  (m_axi_wstrb),
        .m_axi_wlast  (m_axi_wlast),   .m_axi_bvalid (m_axi_bvalid),
        .m_axi_bready (m_axi_bready),  .m_axi_bresp  (m_axi_bresp),

        .cycle_counter(cycle_counter),
        .aclk(aclk), .aresetn(aresetn)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/core_top_1pe_dump.vcd");
        $dumpvars(0, tb_core_top_1pe.u_dut.state);
        $dumpvars(0, tb_core_top_1pe.u_dut.cr_finish);
        $dumpvars(0, tb_core_top_1pe.u_dut.cycle_counter);
    end
`endif

endmodule
