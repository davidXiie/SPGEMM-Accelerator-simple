//=============================================================================
// File     : c_drain_writer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Streams the row-accumulator drain straight to DDR through a SMALL
//            one-row line buffer, replacing the full on-chip dense-C bank.
//
//   Motivation: the C bank stored the whole per-PE dense C (128 rows x N) just
//   to reorder the sparse/padded drain into aligned DDR bursts.  A single dense
//   ROW is enough for that reorder — ~128x less BRAM — because the drain emits a
//   row's column-groups in order (gaddr 0..last, all groups incl. zeros), so we
//   only need to hold one row, then DMA exactly its N columns to DDR.
//
//   Flow:  FILL  — drain writes each 16-col group into line_buf[gaddr*16 +: 16];
//                  drain_ready is high while filling.
//          WRITE — on the row's last group, DMA line_buf[0..N-1] to DDR at
//                  C_DENSE_BASE + row*N*2 (exactly N cols -> handles N%16 / N%32
//                  via the last beat's wstrb); drain_ready low so the next row's
//                  drain waits (compute keeps running on the other ping-pong acc).
//
//   NOTE: single-row (no ping-pong) for clarity; drain and DMA don't overlap.
//   A 2-row ping-pong would overlap them at ~2x the (still tiny) buffer.
//   NOT yet wired into pe_top/core_top — the on-board PE integration is pending
//   (see core_top<->pe_top interface mismatch); this is the drain->DDR datapath.
//=============================================================================

`include "defines.vh"

module c_drain_writer #(
    parameter ROW_W   = `A_ROW_ADDR_BITS,   // drain_row_id width
    parameter N_MAX   = `MAX_N,             // max columns per row
    parameter NB      = `N_MAC              // drain lanes per group (16)
) (
    input  wire                      aclk,
    input  wire                      aresetn,

    input  wire [`MAX_DIM_BITS-1:0]  N,          // active column count this run
    input  wire [`AXI_ADDR_WIDTH-1:0] row_base_addr, // DDR byte addr of C row 0

    // Row-accumulator drain interface (one PE) + backpressure
    input  wire                      drain_active,   // high for each group beat
    input  wire [NB-1:0]             drain_valid,    // per-lane non-zero mask (unused: dense)
    input  wire [$clog2(N_MAX/NB)-1:0] drain_gaddr,  // column-group address
    input  wire [ROW_W-1:0]          drain_row_id,   // GLOBAL C row (apply C_row_map upstream)
    input  wire [NB*16-1:0]          drain_values,   // NB FP16 accumulators (0 where invalid)
    output wire                      drain_ready,    // low while DMA'ing a row

    // AXI-Full write master
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
    output reg                       m_axi_bready
);
    localparam EPB = `N_ELEM_PER_AXI_BEAT;   // 32 FP16 per 512-bit beat

    // One-row dense line buffer (fill by group, read by 32-elem beat for DDR).
    (* ram_style="block" *) reg [15:0] line_buf [0:N_MAX-1];

    localparam FILL = 1'b0, WRITE = 1'b1;
    reg        mode;
    reg [ROW_W-1:0]           row_lat;      // row being DMA'd
    reg [`AXI_ADDR_WIDTH-1:0] ddr_addr;
    reg [15:0]                elem_ptr;     // next element to stream (0..N)
    reg [15:0]                total_beats, beat_cnt;

    // ---- FILL: drain writes 16 cols per group; last group -> switch to WRITE ----
    wire [15:0] a_row_last_group = (N - 1) >> $clog2(NB);      // last gaddr for this row
    wire        fill_last = drain_active && (drain_gaddr == a_row_last_group[$clog2(N_MAX/NB)-1:0]);
    assign drain_ready = (mode == FILL);

    integer g;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            mode <= FILL; row_lat <= 0; ddr_addr <= 0;
            elem_ptr <= 0; total_beats <= 0; beat_cnt <= 0;
            m_axi_awvalid<=0; m_axi_awaddr<=0; m_axi_awlen<=0;
            m_axi_wvalid<=0; m_axi_wdata<=0; m_axi_wstrb<=0; m_axi_wlast<=0; m_axi_bready<=0;
        end else begin
            case (mode)
                FILL: begin
                    if (drain_active) begin
                        // scatter the group's NB lanes into the dense line at gaddr*NB
                        for (g = 0; g < NB; g = g + 1)
                            line_buf[drain_gaddr*NB + g] <= drain_values[g*16 +: 16];
                        if (fill_last) begin
                            row_lat     <= drain_row_id;
                            ddr_addr    <= row_base_addr + drain_row_id * N * 2;
                            total_beats <= (N + EPB - 1) / EPB;
                            beat_cnt    <= 0;
                            elem_ptr    <= 0;
                            m_axi_awvalid <= 1'b1;
                            m_axi_awaddr  <= row_base_addr + drain_row_id * N * 2;
                            m_axi_awlen   <= ((N + EPB - 1) / EPB) - 1;   // whole row = 1 burst
                            mode <= WRITE;
                        end
                    end
                end

                WRITE: begin
                    if (m_axi_awvalid && m_axi_awready) m_axi_awvalid <= 1'b0;

                    // present a 512-bit beat (32 FP16) from the line buffer
                    if (!m_axi_wvalid || (m_axi_wready && m_axi_wvalid)) begin
                        if (beat_cnt < total_beats) begin
                            for (g = 0; g < EPB; g = g + 1)
                                m_axi_wdata[g*16 +: 16] <=
                                    (elem_ptr + g < N) ? line_buf[elem_ptr + g] : 16'h0000;
                            // byte-enable only the columns that exist (last beat partial)
                            for (g = 0; g < EPB; g = g + 1)
                                m_axi_wstrb[g*2 +: 2] <= (elem_ptr + g < N) ? 2'b11 : 2'b00;
                            m_axi_wvalid <= 1'b1;
                            m_axi_wlast  <= (beat_cnt == total_beats - 1);
                            elem_ptr <= elem_ptr + EPB;
                            beat_cnt <= beat_cnt + 1;
                        end else begin
                            m_axi_wvalid <= 1'b0;
                            m_axi_wlast  <= 1'b0;
                        end
                    end

                    if (m_axi_wvalid && m_axi_wready && m_axi_wlast) m_axi_bready <= 1'b1;
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        mode <= FILL;                  // ready for the next row
                    end
                end
            endcase
        end
    end

endmodule
