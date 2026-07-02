//=============================================================================
// File     : c_dense_ddr_writer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Reads C_dense_buffer and writes to DDR at C_DDR_BASE.
//            Writes M rows × N cols of FP16 data.
//            DDR address: C_DDR_BASE + (row * N + col) * 2
//=============================================================================

`include "defines.vh"

module c_dense_ddr_writer (
    input  wire                      start,
    output reg                       done,

    input  wire [`MAX_DIM_BITS-1:0]  M,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // C_dense_buffer read
    output reg                       cbuf_rd_en,
    output reg  [`C_DENSE_DEPTH_LOG-1:0] cbuf_rd_addr,
    input  wire [`AXI_DATA_WIDTH-1:0] cbuf_rd_data,
    input  wire                      cbuf_rd_valid,

    // AXI Write Master
    output reg                       m_axi_awvalid,
    input  wire                      m_axi_awready,
    output reg  [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output reg  [7:0]                m_axi_awlen,

    output reg                       m_axi_wvalid,
    input  wire                      m_axi_wready,
    output reg  [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output reg  [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output reg                       m_axi_wlast,

    input  wire                      m_axi_bvalid,
    output reg                       m_axi_bready,
    input  wire [1:0]                m_axi_bresp,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // Write entire C_dense_buffer to DDR one AXI beat at a time.
    // Each beat: 64 bytes = 32 × FP16 elements.
    // Total elements: M × N
    // Total beats: ceil(M*N / 32)
    //=========================================================================
    localparam ST_IDLE      = 3'd0;
    localparam ST_WR_CMD    = 3'd1;
    localparam ST_WR_DATA   = 3'd2;
    localparam ST_WR_RESP   = 3'd3;
    localparam ST_DONE      = 3'd4;

    reg [2:0] state;

    reg [`C_DENSE_DEPTH_LOG-1:0] cbuf_addr_reg;   // current C_buffer read address
    reg [`AXI_ADDR_WIDTH-1:0]    ddr_addr_reg;     // current DDR write address
    reg [15:0]                   total_beats;       // total AXI beats
    reg [15:0]                   beat_cnt;          // beats written so far
    reg [7:0]                    burst_len;         // beats in current burst
    reg [7:0]                    burst_cnt;         // beats done in current burst
    reg [15:0]                   remaining;         // remaining beats

    reg started;
    wire [`C_DENSE_DEPTH_LOG-1:0] total_elements;
    assign total_elements = M * N;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state      <= ST_IDLE;
            done       <= 1'b0;
            started    <= 1'b0;
            cbuf_rd_en <= 1'b0;
            cbuf_rd_addr <= 0;
            m_axi_awvalid <= 1'b0;
            m_axi_awaddr  <= 0;
            m_axi_awlen   <= 0;
            m_axi_wvalid  <= 1'b0;
            m_axi_wdata   <= 0;
            m_axi_wstrb   <= 0;
            m_axi_wlast   <= 1'b0;
            m_axi_bready  <= 1'b0;
            cbuf_addr_reg <= 0;
            ddr_addr_reg  <= 0;
            total_beats   <= 0;
            beat_cnt      <= 0;
            burst_len     <= 0;
            burst_cnt     <= 0;
            remaining     <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start && !started) begin
                        started     <= 1'b1;
                        total_beats <= (total_elements + `N_ELEM_PER_AXI_BEAT - 1) / `N_ELEM_PER_AXI_BEAT;
                        beat_cnt    <= 0;
                        cbuf_addr_reg <= 0;
                        ddr_addr_reg  <= `DDR_C_DENSE_BASE;
                        state <= ST_WR_CMD;
                    end
                end

                ST_WR_CMD: begin
                    // Issue write command for a burst
                    remaining <= total_beats - beat_cnt;
                    if (remaining > `AXI_BURST_MAX)
                        burst_len <= `AXI_BURST_MAX - 1;
                    else
                        burst_len <= remaining - 1;
                    burst_cnt <= 0;

                    m_axi_awvalid <= 1'b1;
                    m_axi_awaddr  <= ddr_addr_reg;
                    m_axi_awlen   <= burst_len;

                    // Pre-fetch first beat from C buffer
                    cbuf_rd_en   <= 1'b1;
                    cbuf_rd_addr <= cbuf_addr_reg;
                    cbuf_addr_reg <= cbuf_addr_reg + `N_ELEM_PER_AXI_BEAT;

                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        state <= ST_WR_DATA;
                    end
                end

                ST_WR_DATA: begin
                    if (cbuf_rd_valid) begin
                        m_axi_wvalid <= 1'b1;
                        m_axi_wdata  <= cbuf_rd_data;
                        m_axi_wstrb  <= {`AXI_STRB_WIDTH{1'b1}};
                        burst_cnt    <= burst_cnt + 1;
                        beat_cnt     <= beat_cnt + 1;

                        if (burst_cnt == burst_len) begin
                            m_axi_wlast <= 1'b1;
                            cbuf_rd_en  <= 1'b0;
                        end else begin
                            m_axi_wlast <= 1'b0;
                            // Pre-fetch next beat
                            cbuf_rd_en   <= 1'b1;
                            cbuf_rd_addr <= cbuf_addr_reg;
                            cbuf_addr_reg <= cbuf_addr_reg + `N_ELEM_PER_AXI_BEAT;
                        end
                    end

                    if (m_axi_wvalid && m_axi_wready && m_axi_wlast) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;
                        m_axi_bready <= 1'b1;
                        state <= ST_WR_RESP;
                    end
                end

                ST_WR_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        ddr_addr_reg <= ddr_addr_reg +
                            (burst_len + 1) * (`AXI_DATA_WIDTH / 8);

                        if (beat_cnt >= total_beats) begin
                            state <= ST_DONE;
                        end else begin
                            state <= ST_WR_CMD;
                        end
                    end
                end

                ST_DONE: begin
                    done    <= 1'b1;
                    started <= 1'b0;
                    state   <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
