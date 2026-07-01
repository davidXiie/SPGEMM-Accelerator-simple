//=============================================================================
// File     : axi_loader.v
// Brief    : Reads pre-partitioned A and broadcast B from DDR via AXI4 Read.
//            DDR stores A partitioned per PE at non-overlapping zones,
//            B as broadcast at a fixed zone.
//
//   All AXI addresses are BYTE addresses (standard AXI4 convention).
//   ARSIZE = 6 (64-byte / 512-bit beats).  Consecutive burst beats are
//   64 bytes apart (AXI4 standard for 512-bit data bus).
//
//   DDR layout (BYTE addresses):
//     Header   @ 0x000000: 2*N_PE words (N_PE rows + N_PE nnz), packed in one beat
//     PE zones: each is PE_ZONE_BYTES = 0x24000 bytes
//       PE0 @ 0x000200:  A_desc@+0x0000, A_col@+0x0800, A_val@+0x12000
//       PE1 @ 0x024000:  A_desc@+0x0000, A_col@+0x0800, A_val@+0x12000
//       PE2 @ 0x048000:  A_desc@+0x0000, A_col@+0x0800, A_val@+0x12000
//       ...
//     B zone  @ N_PE * PE_ZONE_BYTES:
//                        B_desc@+0x0000, B_col@+0x0800, B_val@+0x10000
//
//   (Word-address equivalents for legacy reference:
//    PE0 @ 0x000100, PE1 @ 0x012000, PE2 @ 0x024000, B @ 0x036000)
//=============================================================================

`include "defines.vh"

module axi_loader #(
    parameter N_PE = `N_PE
) (
    input  wire clk, input wire rst_n,
    input  wire                    start,
    output reg                     done,

    input  wire [`MAX_DIM_BITS-1:0] M, K, N,

    // === AXI4 Read Master ===
    output reg  [3:0]              axi_arid,
    output reg  [63:0]             axi_araddr,
    output reg  [7:0]              axi_arlen,
    output reg  [2:0]              axi_arsize,
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

    //=========================================================================
    // Local parameters — byte-address DDR layout
    //=========================================================================
    // PE0 word-base = 0x000100, byte-base = 0x000200
    localparam [31:0] PE0_BYTE_BASE = 32'h0000_0200;
    // PE zone size (word stride 0x12000, byte stride 0x24000)
    localparam [31:0] PE_ZONE_BYTES = 32'h0002_4000;
    // B zone starts after N_PE PE zones (original B word-base 0x36000 / byte 0x6C000)
    localparam [31:0] B_BYTE_BASE   = N_PE * PE_ZONE_BYTES;

    // Intra-zone byte offsets
    localparam [31:0] A_DESC_OFF  = 32'h0000_0000;  // word  0x000, byte  0x000
    localparam [31:0] A_COL_OFF   = 32'h0000_0800;  // word  0x400, byte  0x800
    localparam [31:0] A_VAL_OFF   = 32'h0001_2000;  // word 0x9000, byte 0x12000
    localparam [31:0] B_DESC_OFF  = 32'h0000_0000;  // word  0x000, byte  0x000
    localparam [31:0] B_COL_OFF   = 32'h0000_0800;  // word  0x400, byte  0x800
    localparam [31:0] B_VAL_OFF   = 32'h0001_0000;  // word 0x8000, byte 0x10000

    localparam PE_IDX_W = (N_PE > 1) ? $clog2(N_PE) : 1;

    wire ar_fire = axi_arvalid && axi_arready;
    wire r_fire  = axi_rvalid  && axi_rready;

    //=========================================================================
    // FSM
    //=========================================================================
    localparam S_IDLE       = 5'd0;
    localparam S_HEADER_AR  = 5'd1;
    localparam S_HEADER_R   = 5'd2;
    localparam S_A_DESC_AR  = 5'd3;
    localparam S_A_DESC_R   = 5'd4;
    localparam S_A_DESC_WR  = 5'd5;
    localparam S_A_COL_AR   = 5'd6;
    localparam S_A_COL_R    = 5'd7;
    localparam S_A_COL_WR   = 5'd8;
    localparam S_A_VAL_AR   = 5'd9;
    localparam S_A_VAL_R    = 5'd10;
    localparam S_A_VAL_WR   = 5'd11;
    localparam S_A_NEXT     = 5'd12;
    localparam S_B_DESC_AR  = 5'd13;
    localparam S_B_DESC_R   = 5'd14;
    localparam S_B_DESC_WR  = 5'd15;
    localparam S_B_COL_AR   = 5'd16;
    localparam S_B_COL_R    = 5'd17;
    localparam S_B_COL_WR   = 5'd18;
    localparam S_B_VAL_AR   = 5'd19;
    localparam S_B_VAL_R    = 5'd20;
    localparam S_B_VAL_WR   = 5'd21;
    localparam S_DONE       = 5'd22;

    reg [4:0] state;

    //=========================================================================
    // Per-PE tracking — parameterized by N_PE
    //=========================================================================
    reg [15:0] pe_rows [N_PE-1:0];
    reg [16:0] pe_nnz  [N_PE-1:0];
    reg [15:0] cur_row, cur_nnz;
    reg [PE_IDX_W-1:0] cur_pe;
    reg [15:0] pe_lrow;
    reg [16:0] pe_loff;

    //=========================================================================
    // AXI / address tracking
    //=========================================================================
    reg [31:0]  a_desc_offs;              // byte offset for AXI ARADDR low 32 bits
    reg [31:0]  pe_base;                  // per-PE byte base address
    reg [511:0] rdat;                     // latched read data

    // B-phase tracking
    reg [15:0]  b_row;
    reg [16:0]  b_off;
    reg [16:0]  total_b_nnz;
    reg [31:0]  b_addr;

    // Single-beat element counters (for COL/VAL single-beat reads)
    reg [4:0]   ecnt;                     // element counter within a beat (0..31)
    reg [31:0]  a_col_off;                // running byte offset for A col/val reads
    reg [31:0]  b_col_off;                // running byte offset for B col/val reads

    //=========================================================================
    // Per-PE DDR byte-base address function
    //=========================================================================
    function [31:0] get_pe_base;
        input [PE_IDX_W-1:0] pid;
        begin
            // PE0 is at PE0_BYTE_BASE; PE i>0 are at i * PE_ZONE_BYTES
            if (pid == 0)
                get_pe_base = PE0_BYTE_BASE;
            else
                get_pe_base = pid * PE_ZONE_BYTES;
        end
    endfunction

    //=========================================================================
    // Main FSM
    //=========================================================================
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
                //=================================================================
                // IDLE → HEADER
                //=================================================================
                S_IDLE: begin
                    if (start) begin
                        cur_pe <= 0; pe_lrow <= 0; pe_loff <= 0;
                        pe_base <= get_pe_base(0);
                        state  <= S_HEADER_AR;
                    end
                end

                // ---- Read header: one 512-bit beat contains 2*N_PE words ----
                S_HEADER_AR: begin
                    axi_arvalid <= 1'b1; axi_arid <= 0;
                    axi_araddr  <= 0;           // byte address 0
                    axi_arlen   <= 0;           // single beat
                    axi_arsize  <= 3'd6;        // 64 bytes per beat
                    if (ar_fire) begin
                        axi_arvalid <= 1'b0;
                        axi_rready  <= 1'b1;
                        state <= S_HEADER_R;
                    end
                end

                S_HEADER_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0;
                        // Extract 2*N_PE words from one 512-bit beat
                        begin
                            integer _i;
                            for (_i = 0; _i < N_PE; _i = _i + 1) begin
                                pe_rows[_i] <= axi_rdata[16*(2*_i)   +: 16];
                                pe_nnz[_i]  <= axi_rdata[16*(2*_i+1) +: 16];
                            end
                        end
                        cur_row <= axi_rdata[15:0];     // pe_rows[0]
                        cur_nnz <= axi_rdata[31:16];    // pe_nnz[0]
                        cur_pe <= 0; pe_lrow <= 0; pe_loff <= 0;
                        pe_base <= get_pe_base(0);
                        a_desc_offs <= PE0_BYTE_BASE + A_DESC_OFF;
                        state <= S_A_DESC_AR;
                    end
                end

                //=================================================================
                // A DESC: one 512-bit beat per row (4 words packed in low 64 bits)
                //=================================================================
                S_A_DESC_AR: begin
                    if (cur_row == 0) begin
                        state <= S_A_NEXT;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= {32'd0, a_desc_offs};
                        axi_arlen   <= 0;           // single beat
                        axi_arsize  <= 3'd6;
                        if (ar_fire) begin
                            axi_arvalid <= 1'b0;
                            axi_rready  <= 1'b1;
                            state <= S_A_DESC_R;
                        end
                    end
                end

                S_A_DESC_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0;
                        // Extract 4 words from one beat
                        rdat[15:0]   <= axi_rdata[15:0];
                        rdat[31:16]  <= axi_rdata[31:16];
                        rdat[47:32]  <= axi_rdata[47:32];
                        rdat[63:48]  <= axi_rdata[63:48];
                        a_desc_offs  <= a_desc_offs + 8;  // 4 words = 8 bytes
                        state <= S_A_DESC_WR;
                    end
                end

                S_A_DESC_WR: begin
                    // Reconstruct 36-bit desc from 4 words
                    //   crow = rdat[8:0]
                    //   nnz  = {rdat[18:16], rdat[15:9]}
                    //   off  = {rdat[32],  rdat[31:19]}
                    pe_a_desc_we    <= 1 << cur_pe;
                    pe_a_desc_waddr <= pe_lrow << (cur_pe * `A_ROW_ADDR_BITS);
                    pe_a_desc_wdata <= ({3'd0,
                        rdat[32], rdat[31:19],
                        rdat[18:16], rdat[15:9],
                        rdat[8:0]})
                        << (cur_pe * 36);
                    pe_lrow <= pe_lrow + 1;
                    if (pe_lrow + 1 >= cur_row) begin
                        pe_lrow <= 0;
                        a_col_off <= pe_base +A_COL_OFF;
                        state <= (cur_nnz == 0) ? S_A_VAL_AR : S_A_COL_AR;
                    end else begin
                        state <= S_A_DESC_AR;
                    end
                end

                //=================================================================
                // A COL: single-beat reads, 32 elements per beat
                //=================================================================
                S_A_COL_AR: begin
                    if (cur_nnz == 0) begin
                        state <= S_A_VAL_AR;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= {32'd0, a_col_off};
                        axi_arlen   <= 0;           // single beat
                        axi_arsize  <= 3'd6;
                        if (ar_fire) begin
                            axi_arvalid <= 1'b0;
                            axi_rready  <= 1'b1;
                            state <= S_A_COL_R;
                        end
                    end
                end

                S_A_COL_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0; rdat <= axi_rdata; ecnt <= 0;
                        state <= S_A_COL_WR;
                    end
                end

                S_A_COL_WR: begin
                    pe_a_col_we    <= 1 << cur_pe;
                    pe_a_col_waddr <= (pe_loff + ecnt) << (cur_pe * `A_NNZ_ADDR_BITS);
                    pe_a_col_wdata <= rdat[ecnt*16 +: 16] << (cur_pe * `DATA_WIDTH);
                    a_col_off      <= a_col_off + 2;  // 1 word = 2 bytes
                    ecnt           <= ecnt + 1;
                    if (pe_loff + ecnt + 1 >= cur_nnz || ecnt + 1 == 32) begin
                        if (pe_loff + ecnt + 1 >= cur_nnz) begin
                            pe_loff <= 0;
                            a_col_off <= pe_base +A_VAL_OFF;
                            state <= S_A_VAL_AR;
                        end else begin
                            pe_loff <= pe_loff + 32;
                            state <= S_A_COL_AR;
                        end
                    end
                end

                //=================================================================
                // A VAL: same pattern as COL
                //=================================================================
                S_A_VAL_AR: begin
                    if (cur_nnz == 0) begin state <= S_A_NEXT; end
                    else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= {32'd0, a_col_off};
                        axi_arlen   <= 0; axi_arsize <= 3'd6;
                        if (ar_fire) begin
                            axi_arvalid <= 1'b0; axi_rready <= 1'b1;
                            state <= S_A_VAL_R;
                        end
                    end
                end

                S_A_VAL_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0; rdat <= axi_rdata; ecnt <= 0;
                        state <= S_A_VAL_WR;
                    end
                end

                S_A_VAL_WR: begin
                    pe_a_val_we    <= 1 << cur_pe;
                    pe_a_val_waddr <= (pe_loff + ecnt) << (cur_pe * `A_NNZ_ADDR_BITS);
                    pe_a_val_wdata <= rdat[ecnt*16 +: 16] << (cur_pe * `DATA_WIDTH);
                    a_col_off      <= a_col_off + 2;
                    ecnt           <= ecnt + 1;
                    if (pe_loff + ecnt + 1 >= cur_nnz || ecnt + 1 == 32) begin
                        if (pe_loff + ecnt + 1 >= cur_nnz) state <= S_A_NEXT;
                        else begin pe_loff <= pe_loff + 32; state <= S_A_VAL_AR; end
                    end
                end

                //=================================================================
                // Advance to next PE
                //=================================================================
                S_A_NEXT: begin
                    if (cur_pe + 1 < N_PE) begin
                        cur_pe <= cur_pe + 1; pe_lrow <= 0; pe_loff <= 0;
                        cur_row <= pe_rows[cur_pe + 1];
                        cur_nnz <= pe_nnz[cur_pe + 1];
                        pe_base <= get_pe_base(cur_pe + 1);
                        a_desc_offs <= get_pe_base(cur_pe + 1) + A_DESC_OFF;
                        state <= S_A_DESC_AR;
                    end else begin
                        // Build packed pe_row_counts
                        begin
                            integer _i;
                            for (_i = 0; _i < N_PE; _i = _i + 1)
                                pe_row_counts[_i*16 +: 16] <= pe_rows[_i][15:0];
                        end
                        b_row <= 0; b_off <= 0; total_b_nnz <= 0;
                        b_addr <= B_BYTE_BASE + B_DESC_OFF;
                        state <= S_B_DESC_AR;
                    end
                end

                //=================================================================
                // B DESC: one beat per row (2 words packed in low 32 bits)
                //=================================================================
                S_B_DESC_AR: begin
                    if (b_row >= K) begin
                        b_row <= 0; b_off <= 0;
                        b_addr <= B_BYTE_BASE + B_COL_OFF;
                        state <= S_B_COL_AR;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= b_addr;
                        axi_arlen   <= 0; axi_arsize <= 3'd6;
                        if (ar_fire) begin
                            axi_arvalid <= 1'b0; axi_rready <= 1'b1;
                            state <= S_B_DESC_R;
                        end
                    end
                end

                S_B_DESC_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0;
                        rdat[15:0]  <= axi_rdata[15:0];
                        rdat[31:16] <= axi_rdata[31:16];
                        state <= S_B_DESC_WR;
                    end
                end

                S_B_DESC_WR: begin
                    // rdat[31:0] = 32-bit B desc: {5'b0, b_off[16:0], b_nnz[9:0]}
                    pe_b_desc_we    <= 1;
                    pe_b_desc_waddr <= b_row;
                    pe_b_desc_wdata <= rdat[31:0];
                    total_b_nnz     <= total_b_nnz + rdat[9:0];
                    b_row           <= b_row + 1;
                    b_addr          <= b_addr + 4;  // 2 words = 4 bytes
                    state           <= S_B_DESC_AR;
                end

                //=================================================================
                // B COL: single-beat reads, 32 elements per beat
                //=================================================================
                S_B_COL_AR: begin
                    if (total_b_nnz == 0 || b_off >= total_b_nnz) begin
                        b_off  <= 0;
                        b_addr <= B_BYTE_BASE + B_VAL_OFF;
                        state <= S_B_VAL_AR;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= b_addr;
                        axi_arlen   <= 0; axi_arsize <= 3'd6;
                        if (ar_fire) begin
                            axi_arvalid <= 1'b0; axi_rready <= 1'b1;
                            state <= S_B_COL_R;
                        end
                    end
                end

                S_B_COL_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0; rdat <= axi_rdata; ecnt <= 0;
                        state <= S_B_COL_WR;
                    end
                end

                S_B_COL_WR: begin
                    pe_b_col_we    <= 1;
                    pe_b_col_waddr <= (b_off + ecnt);
                    pe_b_col_wdata <= rdat[ecnt*16 +: 16];
                    b_addr         <= b_addr + 2;  // 1 word = 2 bytes
                    ecnt           <= ecnt + 1;
                    if (b_off + ecnt + 1 >= total_b_nnz || ecnt + 1 == 32) begin
                        if (b_off + ecnt + 1 >= total_b_nnz) begin
                            b_off  <= 0;
                            b_addr <= B_BYTE_BASE + B_VAL_OFF;
                            state  <= S_B_VAL_AR;
                        end else begin
                            b_off  <= b_off + 32;
                            state  <= S_B_COL_AR;
                        end
                    end
                end

                //=================================================================
                // B VAL: same pattern as COL
                //=================================================================
                S_B_VAL_AR: begin
                    if (total_b_nnz == 0 || b_off >= total_b_nnz) begin
                        state <= S_DONE;
                    end else begin
                        axi_arvalid <= 1'b1; axi_arid <= 0;
                        axi_araddr  <= b_addr;
                        axi_arlen   <= 0; axi_arsize <= 3'd6;
                        if (ar_fire) begin
                            axi_arvalid <= 1'b0; axi_rready <= 1'b1;
                            state <= S_B_VAL_R;
                        end
                    end
                end

                S_B_VAL_R: begin
                    axi_rready <= 1'b1;
                    if (r_fire) begin
                        axi_rready <= 1'b0; rdat <= axi_rdata; ecnt <= 0;
                        state <= S_B_VAL_WR;
                    end
                end

                S_B_VAL_WR: begin
                    pe_b_val_we    <= 1;
                    pe_b_val_waddr <= (b_off + ecnt);
                    pe_b_val_wdata <= rdat[ecnt*16 +: 16];
                    b_addr         <= b_addr + 2;
                    ecnt           <= ecnt + 1;
                    if (b_off + ecnt + 1 >= total_b_nnz || ecnt + 1 == 32) begin
                        if (b_off + ecnt + 1 >= total_b_nnz) begin
                            b_off  <= total_b_nnz;
                            state  <= S_DONE;
                        end else begin
                            b_off  <= b_off + 32;
                            state  <= S_B_VAL_AR;
                        end
                    end
                end

                //=================================================================
                // DONE
                //=================================================================
                S_DONE: begin done <= 1'b1; end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
