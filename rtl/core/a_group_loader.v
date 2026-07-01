//=============================================================================
// File     : a_group_loader.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Loads A_group (compact row-desc) from DDR into PE A buffers.
//            Three separate write ports: desc (64bit), col (16bit), val (16bit).
//
//   A is ROW-PARTITIONED (NOT broadcast): each PE owns its share of A rows,
//   laid out by the host in its own DDR region at
//       DDR_A_GROUPS_BASE + pe*A_GROUP_STRIDE
//   The loader walks the valid PEs one at a time (serial on the single AXI
//   read port), and drives `pe_sel` (one-hot) so core_top routes each PE's
//   writes to ONLY that PE.  Addresses are PE-local (reset to 0 per PE), so
//   every PE stores its A densely from local offset 0.  (B stays broadcast.)
//=============================================================================

`include "defines.vh"

module a_group_loader (
    input  wire                      start,
    input  wire [7:0]                pe_valid_mask,   // which PEs to load
    output reg                       done,

    output reg  [`N_PE-1:0]          pe_sel,          // one-hot: PE currently loading
    // PE A buffer write ports (routed to pe_sel by core_top)
    output reg                       pe_a_desc_we,
    output reg  [`A_ROW_ADDR_BITS-1:0] pe_a_desc_waddr,
    output reg  [63:0]               pe_a_desc_wdata,
    output reg                       pe_a_col_we,
    output reg  [`A_NNZ_ADDR_BITS-1:0] pe_a_col_waddr,
    output reg  [`DATA_WIDTH-1:0]    pe_a_col_wdata,
    output reg                       pe_a_val_we,
    output reg  [`A_NNZ_ADDR_BITS-1:0] pe_a_val_waddr,
    output reg  [`DATA_WIDTH-1:0]    pe_a_val_wdata,

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
    localparam ST_NEXT_PE = 3'd1;   // select next valid PE (or finish)
    localparam ST_RD_CMD  = 3'd2;
    localparam ST_RD_DATA = 3'd3;
    localparam ST_DONE    = 3'd4;

    reg [2:0] state;
    reg [2:0] phase;  // 0=desc, 1=col, 2=val

    reg [`AXI_ADDR_WIDTH-1:0] next_addr;
    reg [15:0] elem_cnt, elem_total, rd_cnt, rd_total;

    reg started;
    reg [3:0] pe_ld;                   // current PE index (0..N_PE)
    wire [15:0] a_row_count = 16'd16;  // TODO: from descriptor (per-PE row count)

    // Per-PE DDR region base = DDR_A_GROUPS_BASE + pe_ld * A_GROUP_STRIDE.
    wire [`AXI_ADDR_WIDTH-1:0] pe_base =
        `DDR_A_GROUPS_BASE + (pe_ld * `A_GROUP_STRIDE);

    integer e;  // loop index for AXI beat unrolling

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= ST_IDLE; phase <= 3'd0; done <= 1'b0; started <= 1'b0;
            pe_ld <= 4'd0; pe_sel <= {`N_PE{1'b0}};
            m_axi_arvalid <= 1'b0; m_axi_araddr <= 0; m_axi_arlen <= 0; m_axi_rready <= 1'b0;
            pe_a_desc_we <= 1'b0; pe_a_col_we <= 1'b0; pe_a_val_we <= 1'b0;
        end else begin
            pe_a_desc_we <= 1'b0; pe_a_col_we <= 1'b0; pe_a_val_we <= 1'b0; done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start && !started) begin
                        started <= 1'b1;
                        pe_ld   <= 4'd0;
                        state   <= ST_NEXT_PE;
                    end
                end

                // Advance to the next valid PE and start its phase-0 (desc) read;
                // finish once we've passed the last PE.
                ST_NEXT_PE: begin
                    if (pe_ld >= `N_PE) begin
                        pe_sel <= {`N_PE{1'b0}};
                        state  <= ST_DONE;
                    end else if (!pe_valid_mask[pe_ld]) begin
                        pe_ld <= pe_ld + 4'd1;          // skip disabled PE
                    end else begin
                        pe_sel     <= ({{(`N_PE-1){1'b0}}, 1'b1} << pe_ld);
                        phase      <= 3'd0;
                        elem_total <= a_row_count * 4;  // 4 x 16-bit per 64-bit desc
                        rd_total   <= ((a_row_count * 8 + `AXI_DATA_WIDTH/8 - 1) / (`AXI_DATA_WIDTH/8));
                        rd_cnt     <= 0;
                        next_addr  <= pe_base + `A_ROW_DESC_OFFSET;
                        state      <= ST_RD_CMD;
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
                        for (e = 0; e < `N_ELEM_PER_AXI_BEAT; e = e + 1) begin
                            if (elem_cnt + e < elem_total) begin
                                case (phase)
                                    3'd0: begin
                                        // desc handled below in 64-bit pairs
                                    end
                                    3'd1: begin
                                        pe_a_col_we <= 1'b1;
                                        pe_a_col_waddr <= elem_cnt + e;   // PE-local
                                        pe_a_col_wdata <= m_axi_rdata[e*`DATA_WIDTH +: `DATA_WIDTH];
                                    end
                                    3'd2: begin
                                        pe_a_val_we <= 1'b1;
                                        pe_a_val_waddr <= elem_cnt + e;   // PE-local
                                        pe_a_val_wdata <= m_axi_rdata[e*`DATA_WIDTH +: `DATA_WIDTH];
                                    end
                                endcase
                            end
                        end
                        if (phase == 3'd0) begin
                            for (e = 0; e < `N_ELEM_PER_AXI_BEAT; e = e + 4) begin
                                if (elem_cnt + e + 3 < elem_total) begin
                                    pe_a_desc_we <= 1'b1;
                                    pe_a_desc_waddr <= (elem_cnt + e) >> 2;
                                    pe_a_desc_wdata <= {
                                        m_axi_rdata[(e+3)*`DATA_WIDTH +: `DATA_WIDTH],
                                        m_axi_rdata[(e+2)*`DATA_WIDTH +: `DATA_WIDTH],
                                        m_axi_rdata[(e+1)*`DATA_WIDTH +: `DATA_WIDTH],
                                        m_axi_rdata[(e+0)*`DATA_WIDTH +: `DATA_WIDTH]
                                    };
                                end
                            end
                        end
                        elem_cnt <= elem_cnt + `N_ELEM_PER_AXI_BEAT;
                        rd_cnt <= rd_cnt + 1;
                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            if (phase == 3'd0) begin phase <= 3'd1;
                                elem_total <= 16'd64; rd_total <= 2; rd_cnt <= 0;
                                next_addr <= pe_base + `A_COL_OFFSET;
                                state <= ST_RD_CMD;
                            end else if (phase == 3'd1) begin phase <= 3'd2;
                                elem_total <= 16'd64; rd_total <= 2; rd_cnt <= 0;
                                next_addr <= pe_base + `A_VAL_OFFSET;
                                state <= ST_RD_CMD;
                            end else begin
                                // done with this PE -> move to the next one
                                pe_ld <= pe_ld + 4'd1;
                                state <= ST_NEXT_PE;
                            end
                        end
                    end
                end

                ST_DONE: begin done <= 1'b1; started <= 1'b0; state <= ST_IDLE; end
            endcase
        end
    end

endmodule
