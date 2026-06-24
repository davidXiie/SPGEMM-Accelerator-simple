//=============================================================================
// File     : descriptor_loader.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Reads descriptor (M/K/N/pe_valid_mask) from DDR at DDR_DESC_BASE
//            Uses simple AXI read (single burst, small size).
//=============================================================================

`include "defines.vh"

module descriptor_loader (
    input  wire                      start,
    output reg                       done,

    // Descriptor values (passed through from CR, validated from DDR if needed)
    output reg  [`MAX_DIM_BITS-1:0]  M,
    output reg  [`MAX_DIM_BITS-1:0]  K,
    output reg  [`MAX_DIM_BITS-1:0]  N,
    output reg  [7:0]                pe_valid_mask,

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

    // Phase 1: descriptor values come directly from CR registers via core_top.
    // This module is a pass-through that reads descriptor from DDR if needed.
    // For now, just issue one read to confirm DDR is accessible, then done.
    
    localparam ST_IDLE    = 2'd0;
    localparam ST_READ    = 2'd1;
    localparam ST_DONE    = 2'd2;

    reg [1:0] state;
    reg started;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state    <= ST_IDLE;
            done     <= 1'b0;
            started  <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_araddr  <= 0;
            m_axi_arlen   <= 0;
            m_axi_rready  <= 1'b0;
            M <= 0; K <= 0; N <= 0; pe_valid_mask <= 0;
        end else begin
            done <= 1'b0;
            case (state)
                ST_IDLE: begin
                    if (start && !started) begin
                        started <= 1'b1;
                        // Read descriptor from DDR (single burst, 12×2=24 bytes → 1 AXI beat)
                        m_axi_arvalid <= 1'b1;
                        m_axi_araddr  <= `DDR_DESC_BASE;
                        m_axi_arlen   <= 8'd0;  // 1 beat
                        m_axi_rready  <= 1'b1;
                        state <= ST_READ;
                    end
                end
                ST_READ: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                    end
                    if (m_axi_rvalid && m_axi_rready) begin
                        // Parse descriptor from rdata
                        // Layout: [0]=M, [1]=K, [2]=N, [3]=pe_valid_mask
                        M[`MAX_DIM_BITS-1:0] <= m_axi_rdata[15:0];
                        K[`MAX_DIM_BITS-1:0] <= m_axi_rdata[31:16];
                        N[`MAX_DIM_BITS-1:0] <= m_axi_rdata[47:32];
                        pe_valid_mask <= m_axi_rdata[71:64];
                        m_axi_rready <= 1'b0;
                        state <= ST_DONE;
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
