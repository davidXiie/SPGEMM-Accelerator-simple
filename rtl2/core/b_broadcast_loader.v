//=============================================================================
// File     : b_broadcast_loader.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Loads B (compact row-desc) from DDR, broadcasts to all PE B buffers.
//            Three separate write ports: desc (64bit), col (16bit), val (16bit).
//=============================================================================

`include "defines.vh"

module b_broadcast_loader (
    input  wire                      start,
    output reg                       done,

    // PE B buffer write ports (broadcast)
    output reg                       pe_b_desc_we,
    output reg  [`B_ROW_ADDR_BITS-1:0] pe_b_desc_waddr,
    output reg  [63:0]               pe_b_desc_wdata,
    output reg                       pe_b_col_we,
    output reg  [`B_NNZ_ADDR_BITS-1:0] pe_b_col_waddr,
    output reg  [`DATA_WIDTH-1:0]    pe_b_col_wdata,
    output reg                       pe_b_val_we,
    output reg  [`B_NNZ_ADDR_BITS-1:0] pe_b_val_waddr,
    output reg  [`DATA_WIDTH-1:0]    pe_b_val_wdata,

    // AXI Read Master
    output reg                       m_axi_arvalid,
    input  wire                      m_axi_arready,
    output reg  [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output reg  [7:0]                m_axi_arlen,
    input  wire                      m_axi_rvalid,
    output reg                       m_axi_rready,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam ST_IDLE    = 3'd0;
    localparam ST_RD_CMD  = 3'd1;
    localparam ST_RD_DATA = 3'd2;
    localparam ST_DONE    = 3'd3;

    reg [2:0] state;
    reg [1:0] phase;  // 0=desc, 1=col, 2=val

    reg [`AXI_ADDR_WIDTH-1:0] next_addr;
    reg [16:0] elem_cnt, elem_total, rd_cnt, rd_total;
    wire [`B_ROW_ADDR_BITS:0] K_val = 9'd3;  // TODO: parameterize

    reg started;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= ST_IDLE; phase <= 2'd0; done <= 1'b0; started <= 1'b0;
            m_axi_arvalid <= 1'b0; m_axi_araddr <= 0; m_axi_arlen <= 0; m_axi_rready <= 1'b0;
            pe_b_desc_we <= 1'b0; pe_b_col_we <= 1'b0; pe_b_val_we <= 1'b0;
        end else begin
            pe_b_desc_we <= 1'b0; pe_b_col_we <= 1'b0; pe_b_val_we <= 1'b0; done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start && !started) begin
                        started <= 1'b1; phase <= 2'd0;
                        // Phase 0: B_row_desc (64-bit each = 4×16-bit per row)
                        elem_total <= K_val * 4;
                        rd_total   <= ((K_val * 8 + `AXI_DATA_WIDTH/8 - 1) / (`AXI_DATA_WIDTH/8));
                        rd_cnt <= 0;
                        next_addr  <= `DDR_B_BASE + `B_ROW_DESC_OFFSET;
                        state <= ST_RD_CMD;
                    end
                end

                ST_RD_CMD: begin
                    m_axi_arvalid <= 1'b1; m_axi_araddr <= next_addr;
                    m_axi_arlen <= rd_total - 1; m_axi_rready <= 1'b1; elem_cnt <= 0;
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0; state <= ST_RD_DATA;
                    end
                end

                ST_RD_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        // Write desc as 64-bit (4 × 16-bit per entry)
                        if (phase == 2'd0) begin
                            for (integer e = 0; e < `N_ELEM_PER_AXI_BEAT; e = e + 4) begin
                                if (elem_cnt + e + 3 < elem_total) begin
                                    pe_b_desc_we <= 1'b1;
                                    pe_b_desc_waddr <= (elem_cnt + e) >> 2;
                                    pe_b_desc_wdata <= {
                                        m_axi_rdata[(e+3)*`DATA_WIDTH +: `DATA_WIDTH],
                                        m_axi_rdata[(e+2)*`DATA_WIDTH +: `DATA_WIDTH],
                                        m_axi_rdata[(e+1)*`DATA_WIDTH +: `DATA_WIDTH],
                                        m_axi_rdata[(e+0)*`DATA_WIDTH +: `DATA_WIDTH]
                                    };
                                end
                            end
                        end else if (phase == 2'd1) begin
                            for (integer e = 0; e < `N_ELEM_PER_AXI_BEAT; e = e + 1) begin
                                if (elem_cnt + e < elem_total) begin
                                    pe_b_col_we <= 1'b1;
                                    pe_b_col_waddr <= elem_cnt + e;
                                    pe_b_col_wdata <= m_axi_rdata[e*`DATA_WIDTH +: `DATA_WIDTH];
                                end
                            end
                        end else begin
                            for (integer e = 0; e < `N_ELEM_PER_AXI_BEAT; e = e + 1) begin
                                if (elem_cnt + e < elem_total) begin
                                    pe_b_val_we <= 1'b1;
                                    pe_b_val_waddr <= elem_cnt + e;
                                    pe_b_val_wdata <= m_axi_rdata[e*`DATA_WIDTH +: `DATA_WIDTH];
                                end
                            end
                        end

                        elem_cnt <= elem_cnt + `N_ELEM_PER_AXI_BEAT;
                        rd_cnt <= rd_cnt + 1;
                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            if (phase == 2'd0) begin phase <= 2'd1;
                                elem_total <= 17'd5; rd_total <= 1; rd_cnt <= 0;
                                next_addr <= `DDR_B_BASE + `B_COL_OFFSET;
                                state <= ST_RD_CMD;
                            end else if (phase == 2'd1) begin phase <= 2'd2;
                                elem_total <= 17'd5; rd_total <= 1; rd_cnt <= 0;
                                next_addr <= `DDR_B_BASE + 64'h40000;
                                state <= ST_RD_CMD;
                            end else begin state <= ST_DONE; end
                        end
                    end
                end

                ST_DONE: begin done <= 1'b1; started <= 1'b0; state <= ST_IDLE; end
            endcase
        end
    end

endmodule
