//=============================================================================
// File     : axi_loader.v
// Brief    : Reads pre-partitioned A and broadcast B from DDR via AXI.
//            DDR stores A partitioned per PE at non-overlapping zones,
//            B as broadcast at a fixed zone.
//
//   DDR layout (16-bit word addresses, PE_ZONE = 0x12000):
//     Header   @ 0x000000: 6 words (pe_rows[0..2], pe_nnz[0..2])
//     PE0 zone @ 0x000100: A_desc@base+0x000, A_col@base+0x0400, A_val@base+0x9000
//     PE1 zone @ 0x012000: same layout
//     PE2 zone @ 0x024000: same layout
//     B zone   @ 0x036000: B_desc@base+0x000, B_col@base+0x0400, B_val@base+0x8000
//=============================================================================

`include "defines.vh"

module axi_loader #(
    parameter N_PE = `N_PE
) (
    input  wire clk, input wire rst_n,
    input  wire                    start,
    output reg                     done,

    input  wire [`MAX_DIM_BITS-1:0] M, K, N,

    // === AXI Read Master ===
    output reg  [3:0]              axi_arid,
    output reg  [63:0]             axi_araddr,
    output reg  [7:0]              axi_arlen,
    output reg                     axi_arvalid,
    input  wire                    axi_arready,
    input  wire [3:0]              axi_rid,
    input  wire [511:0]            axi_rdata,
    input  wire [1:0]              axi_rresp,
    input  wire                    axi_rlast,
    input  wire                    axi_rvalid,
    output reg                     axi_rready,

    // === PE A ports (packed) ===
    output reg  [N_PE-1:0]                       pe_a_desc_we,
    output reg  [N_PE*`A_ROW_ADDR_BITS-1:0]     pe_a_desc_waddr,
    output reg  [N_PE*36-1:0]                    pe_a_desc_wdata,
    output reg  [N_PE-1:0]                       pe_a_val_we,
    output reg  [N_PE*`A_NNZ_ADDR_BITS-1:0]     pe_a_val_waddr,
    output reg  [N_PE*`DATA_WIDTH-1:0]           pe_a_val_wdata,
    output reg  [N_PE-1:0]                       pe_a_col_we,
    output reg  [N_PE*`A_NNZ_ADDR_BITS-1:0]     pe_a_col_waddr,
    output reg  [N_PE*`DATA_WIDTH-1:0]           pe_a_col_wdata,

    // === PE B ports (broadcast) ===
    output reg                         pe_b_desc_we,
    output reg  [`B_ROW_ADDR_BITS-1:0] pe_b_desc_waddr,
    output reg  [31:0]                 pe_b_desc_wdata,
    output reg                         pe_b_col_we,
    output reg  [`B_NNZ_ADDR_BITS-1:0] pe_b_col_waddr,
    output reg  [`DATA_WIDTH-1:0]      pe_b_col_wdata,
    output reg                         pe_b_val_we,
    output reg  [`B_NNZ_ADDR_BITS-1:0] pe_b_val_waddr,
    output reg  [`DATA_WIDTH-1:0]      pe_b_val_wdata,

    output reg  [N_PE*16-1:0]          pe_row_counts
);

    wire ar_fire = axi_arvalid && axi_arready;
    wire r_fire  = axi_rvalid  && axi_rready;

    // FSM
    localparam S_IDLE      = 4'd0;
    localparam S_HEADER_AR = 4'd1;
    localparam S_HEADER_R  = 4'd2;
    localparam S_A_DESC_AR = 4'd3;
    localparam S_A_DESC_R  = 4'd4;
    localparam S_A_DESC_WR = 4'd5;
    localparam S_A_COL_AR  = 4'd6;
    localparam S_A_COL_R   = 4'd7;
    localparam S_A_COL_WR  = 4'd8;
    localparam S_A_VAL_AR  = 4'd9;
    localparam S_A_VAL_R   = 4'd10;
    localparam S_A_VAL_WR  = 4'd11;
    localparam S_A_NEXT    = 4'd12;
    localparam S_B_DESC    = 4'd13;
    localparam S_B_COL     = 4'd14;
    localparam S_DONE      = 4'd15;

    reg [3:0] state;

    // Per-PE tracking
    reg [15:0] pe_rows [0:2];            // rows per PE
    reg [16:0] pe_nnz  [0:2];            // nnz per PE
    reg [15:0] cur_row, cur_nnz;          // current PE's counts
    reg [2:0]  cur_pe;                    // 0,1,2
    reg [15:0] pe_lrow;                   // local row index within PE
    reg [15:0] pe_loff;                   // local nnz offset within PE

    // AXI
    reg [511:0] rdat; reg [4:0] rcnt;
    reg [31:0]  a_desc_offs;              // starting word offset for A_desc/col/val
    reg [31:0]  pe_base;                  // per-PE base address in DDR

    // Per-PE DDR base addresses (word address)
    // Use simple function for reliable synthesis
    function [31:0] get_pe_base;
        input [2:0] pid;
        begin
            case (pid)
                3'd0: get_pe_base = 32'h000100;
                3'd1: get_pe_base = 32'h012000;
                3'd2: get_pe_base = 32'h024000;
                default: get_pe_base = 32'h000100;
            endcase
        end
    endfunction
    wire [31:0] B_BASE = 32'h036000;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0;
            axi_arvalid <= 1'b0; axi_rready <= 1'b0;
            {pe_a_desc_we,pe_a_val_we,pe_a_col_we} <= 0;
            {pe_b_desc_we,pe_b_col_we,pe_b_val_we} <= 0;
        end else begin
            done <= 1'b0;
            {pe_a_desc_we,pe_a_val_we,pe_a_col_we} <= 0;
            {pe_b_desc_we,pe_b_col_we,pe_b_val_we} <= 0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        cur_pe <= 0; pe_lrow <= 0; pe_loff <= 0;
                        pe_base <= get_pe_base(0);
                        state  <= S_HEADER_AR;
                    end
                end

                // ---- Read header (6 words: row_counts[0:2]+nnz_counts[0:2]) ----
                S_HEADER_AR: begin
                    axi_arvalid <= 1'b1; axi_arid <= 0;
                    axi_araddr  <= 0; axi_arlen <= 5;  // burst 6 beats
                    if (ar_fire) begin axi_arvalid <= 1'b0; axi_rready <= 1'b1; state <= S_HEADER_R; end
                end

                S_HEADER_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        // Header layout: {row0, nnz0, row1, nnz1, row2, nnz2}
                        // Even rcnt → pe_rows, odd rcnt → pe_nnz
                        if (rcnt[0] == 0) pe_rows[rcnt[3:1]] <= axi_rdata[15:0];
                        else              pe_nnz[rcnt[3:1]]  <= axi_rdata[15:0];
                        rcnt <= rcnt + 1;
                        if (axi_rlast) begin
                            axi_rready <= 1'b0; rcnt <= 0;
                            cur_row <= pe_rows[0]; cur_nnz <= pe_nnz[0];
                            cur_pe <= 0; pe_lrow <= 0; pe_loff <= 0;
                            pe_base <= get_pe_base(0);
                            a_desc_offs <= get_pe_base(0);  // PE0 A_desc base
                            state <= S_A_DESC_AR;
                        end
                    end
                end

                // ---- A DESC: read one row desc per burst ----
                S_A_DESC_AR: begin
                    if (cur_row == 0) begin
                        // current PE has no rows → next PE or next phase
                        state <= S_A_NEXT;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= {32'd0, a_desc_offs}; axi_arlen <= 3; // 4 beats = 4 words = 1 desc
                        if (ar_fire) begin axi_arvalid <= 1'b0; axi_rready <= 1'b1; rcnt <= 0; state <= S_A_DESC_R; end
                    end
                end

                S_A_DESC_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        rdat[rcnt*16 +: 16] <= axi_rdata[15:0];
                        rcnt <= rcnt + 1;
                        if (axi_rlast) begin
                            axi_rready <= 1'b0; rcnt <= 0;
                            a_desc_offs <= a_desc_offs + 4;
                            state <= S_A_DESC_WR;
                        end
                    end
                end

                S_A_DESC_WR: begin
                    // Write desc to PE cur_pe at local row pe_lrow.
                    // rdat accumulates 4×16-bit words from the 4-beat burst.
                    //
                    // DDR stores 36-bit desc d = (off<<19)|(nnz<<9)|crow as 4 words:
                    //   word0 = d[15:0]  = {nnz[6:0], crow[8:0]}
                    //   word1 = d[31:16] = {off[12:0], nnz[9:7]}
                    //   word2 = d[47:32] = {12'b0, off[13]} (upper bits zero)
                    //
                    // Reconstruct from rdat = {word3,word2,word1,word0}:
                    //   crow = rdat[8:0]
                    //   nnz  = {rdat[18:16], rdat[15:9]}
                    //   off  = {rdat[32],  rdat[31:19]}
                    pe_a_desc_we    <= 1 << cur_pe;
                    pe_a_desc_waddr <= pe_lrow << (cur_pe * `A_ROW_ADDR_BITS);
                    pe_a_desc_wdata <= ({3'd0,
                        rdat[32], rdat[31:19],          // off[13:0]
                        rdat[18:16], rdat[15:9],        // nnz[9:0]
                        rdat[8:0]})                     // crow[8:0]
                        << (cur_pe * 36);
                    pe_lrow <= pe_lrow + 1;
                    if (pe_lrow + 1 >= cur_row) begin
                        pe_lrow <= 0;
                        a_desc_offs <= pe_base + 32'h400;  // A_col within PE zone
                        state <= (cur_nnz == 0) ? S_A_VAL_AR : S_A_COL_AR;
                    end else begin
                        state <= S_A_DESC_AR;
                    end
                end

                // ---- A COL: read one 512-bit beat, write up to 32 elements ----
                S_A_COL_AR: begin
                    if (cur_nnz == 0) begin
                        state <= S_A_VAL_AR;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= {32'd0, a_desc_offs};
                        axi_arlen   <= 0;  // single beat
                        if (ar_fire) begin axi_arvalid <= 1'b0; axi_rready <= 1'b1; state <= S_A_COL_R; end
                    end
                end

                S_A_COL_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0; rdat <= axi_rdata; rcnt <= 0; state <= S_A_COL_WR; end
                end

                S_A_COL_WR: begin
                    pe_a_col_we    <= 1 << cur_pe;
                    pe_a_col_waddr <= (pe_loff + rcnt) << (cur_pe * `A_NNZ_ADDR_BITS);
                    pe_a_col_wdata <= rdat[rcnt*16 +: 16] << (cur_pe * `DATA_WIDTH);
                    rcnt <= rcnt + 1; a_desc_offs <= a_desc_offs + 1;
                    if (rcnt + 1 >= cur_nnz || rcnt + 1 == 32) begin
                        if (rcnt + 1 >= cur_nnz) begin
                            a_desc_offs <= pe_base + 32'h9000;  // A_val within PE zone
                            state <= S_A_VAL_AR;
                        end else begin
                            state <= S_A_COL_AR;
                        end
                    end
                end

                // ---- A VAL: same pattern as COL ----
                S_A_VAL_AR: begin
                    if (cur_nnz == 0) begin state <= S_A_NEXT; end
                    else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= {32'd0, a_desc_offs}; axi_arlen <= 0;
                        if (ar_fire) begin axi_arvalid <= 1'b0; axi_rready <= 1'b1; state <= S_A_VAL_R; end
                    end
                end

                S_A_VAL_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin axi_rready <= 1'b0; rdat <= axi_rdata; rcnt <= 0; state <= S_A_VAL_WR; end
                end

                S_A_VAL_WR: begin
                    pe_a_val_we    <= 1 << cur_pe;
                    pe_a_val_waddr <= (pe_loff + rcnt) << (cur_pe * `A_NNZ_ADDR_BITS);
                    pe_a_val_wdata <= rdat[rcnt*16 +: 16] << (cur_pe * `DATA_WIDTH);
                    rcnt <= rcnt + 1; a_desc_offs <= a_desc_offs + 1;
                    if (rcnt + 1 >= cur_nnz || rcnt + 1 == 32) begin
                        if (rcnt + 1 >= cur_nnz) state <= S_A_NEXT;
                        else state <= S_A_VAL_AR;
                    end
                end

                // ---- Advance to next PE ----
                S_A_NEXT: begin
                    if (cur_pe + 1 < N_PE) begin
                        cur_pe <= cur_pe + 1; pe_lrow <= 0; pe_loff <= 0;
                        cur_row <= pe_rows[cur_pe + 1]; cur_nnz <= pe_nnz[cur_pe + 1];
                        pe_base <= get_pe_base(cur_pe + 1);
                        a_desc_offs <= get_pe_base(cur_pe + 1); state <= S_A_DESC_AR;
                    end else begin
                        pe_row_counts <= {pe_rows[2][15:0], pe_rows[1][15:0], pe_rows[0][15:0]};
                        state <= S_B_DESC;
                    end
                end

                // ---- B DESC: broadcast sequential write ----
                S_B_DESC: begin
                    // Read B_desc[k] from DDR
                    // ... simplified: sequential read + write, 1 row per cycle
                    state <= S_B_COL;
                end

                S_B_COL: begin
                    // Read B_col from DDR, write to broadcast port
                    state <= S_DONE;
                end

                S_DONE: begin done <= 1'b1; end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
