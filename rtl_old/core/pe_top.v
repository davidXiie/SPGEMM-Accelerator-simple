//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator
// Brief    : PE Top-level — integrates pipeline-based Decompress, MUL Array,
//           and Aggregation with Chisel-style handshaking.
//
//   Pipeline:
//     pe_decompress (S0+S1) → pe_mul_array (3-stage) → pe_aggregation
//       entry_fifo              STREAM→MUL→SPA
//
//   Handshake signals:
//     pe_decompress → agg_stall (backpressure when SPA conflict/outputting)
//
//   Interface with core_top:
//     start/done: PE lifecycle
//     row_start/row_end/a_ptr_start/a_ptr_end: host task descriptor
//     out_*: single-element row output to C CSR Writer
//=============================================================================

`include "defines.vh"

module pe_top #(
    parameter integer PE_ID = 0
) (
    input  wire                      start,
    output wire                      done,

    // Task descriptor (from host scheduler via task_loader)
    input  wire [`MAX_DIM_BITS-1:0]  row_start,
    input  wire [`MAX_DIM_BITS-1:0]  row_end,
    input  wire [15:0]               a_ptr_start,
    input  wire [15:0]               a_ptr_end,
    input  wire [`MAX_DIM_BITS-1:0]  M,    // A rows
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,
    input  wire [2:0]                op_type,     // MUL/ADD/SUB

    // PE Local Buffer Load (STATE_LOAD_PE in core_top)
    input  wire                      load_en,     // asserted when this PE should load
    output reg                       load_done,   // pulsed when loading complete

    // SRAM base addresses in GlobalBuffer (from COMPUTE instruction latches)
    input  wire [15:0]               a_row_sram,
    input  wire [15:0]               a_col_sram,
    input  wire [15:0]               a_val_sram,
    input  wire [15:0]               b_row_sram,
    input  wire [15:0]               b_col_sram,
    input  wire [15:0]               b_val_sram,

    // GlobalBuffer read (shared, muxed by core_top)
    output reg                       gbuf_rd_en,
    output reg  [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr,
    input  wire [`DATA_WIDTH-1:0]    gbuf_rd_data,
    input  wire                      gbuf_rd_valid,

    // Output to CSR Writer (single-element stream)
    output wire [`MAX_DIM_BITS-1:0]  out_row_id,
    output wire [`MAX_DIM_BITS-1:0]  out_nnz,
    output wire [`DATA_WIDTH-1:0]    out_col,
    output wire [`DATA_WIDTH-1:0]    out_val,
    output wire                      out_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // A Buffer: stores this PE's assigned A rows
    //   Written during LOAD_PE state, read during COMPUTE
    //=========================================================================
    reg                         a_buf_wr_en;
    reg  [`PE_ABUF_DEPTH_LOG-1:0] a_buf_wr_addr;
    reg  [`DATA_WIDTH-1:0]       a_buf_wr_data;

    std_scratchpad #(
        .DEPTH(`PE_ABUF_DEPTH), .DEPTH_LOG(`PE_ABUF_DEPTH_LOG), .DATA_WIDTH(`DATA_WIDTH)
    ) u_a_buffer (
        .wr_en    (a_buf_wr_en),
        .wr_addr  (a_buf_wr_addr),
        .wr_data  (a_buf_wr_data),
        .rd_en    (a_buf_rd_en),
        .rd_addr  (a_buf_rd_addr),
        .rd_data  (a_buf_rd_data),
        .rd_valid (a_buf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );
    wire a_buf_rd_en;
    wire [`PE_ABUF_DEPTH_LOG-1:0] a_buf_rd_addr;
    wire [`DATA_WIDTH-1:0] a_buf_rd_data;
    wire a_buf_rd_valid;

    //=========================================================================
    // B Buffer: full B CSR copy (banked, N_MAC parallel lanes)
    //   Written during LOAD_PE state, read during COMPUTE
    //=========================================================================
    reg                         b_buf_wr_en;
    reg  [`PE_BBUF_DEPTH_LOG-1:0] b_buf_wr_addr;
    reg  [`N_MAC*`DATA_WIDTH-1:0] b_buf_wr_data;

    banked_scratchpad #(
        .N_BANKS(`N_MAC), .DEPTH(`PE_BBUF_DEPTH), .DEPTH_LOG(`PE_BBUF_DEPTH_LOG), .BANK_WIDTH(`DATA_WIDTH)
    ) u_b_buffer (
        .wr_en    (b_buf_wr_en),
        .wr_addr  (b_buf_wr_addr),
        .wr_data  (b_buf_wr_data),
        .rd_en    (b_buf_rd_en),
        .rd_addr  (b_buf_rd_addr),
        .rd_data  (b_buf_rd_data),
        .rd_valid (b_buf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );
    wire [`N_MAC-1:0]            b_buf_rd_en;
    wire [`N_MAC*`PE_BBUF_DEPTH_LOG-1:0] b_buf_rd_addr;
    wire [`N_MAC*`DATA_WIDTH-1:0] b_buf_rd_data;
    wire [`N_MAC-1:0]            b_buf_rd_valid;

    //=========================================================================
    // PE Load State Machine (STATE_LOAD_PE)
    //   Loads A_row_ptr, A_data (col/val interleaved), B_row_ptr, B_data
    //   from GlobalBuffer into local A/B buffers.
    //   A buffer layout:  [0..M] = A_row_ptr, [M+1..] = col/val interleaved pairs
    //   B buffer layout:  [0..K] = B_row_ptr (bank0 only),
    //                     [K+1..] = 64-bit words, each = {col[j+1],val[j+1],col[j],val[j]}
    //=========================================================================
    localparam LD_IDLE        = 4'd0;
    localparam LD_A_ROW       = 4'd1;
    localparam LD_A_DATA_COL  = 4'd2;
    localparam LD_A_DATA_VAL  = 4'd3;
    localparam LD_A_DATA_WR_V = 4'd4;
    localparam LD_B_ROW       = 4'd5;
    localparam LD_B_DATA_C0   = 4'd6;
    localparam LD_B_DATA_V0   = 4'd7;
    localparam LD_B_DATA_C1   = 4'd8;
    localparam LD_B_DATA_V1   = 4'd9;
    localparam LD_DONE        = 4'd10;

    reg [3:0] ld_state, ld_state_next;

    // Counters & latches
    reg [15:0] ld_abuf_base;     // A buffer address base for data section (M+1)
    reg [15:0] ld_cnt;           // element index within current section
    reg [15:0] ld_a_data_total;  // = a_ptr_end - a_ptr_start
    reg [15:0] ld_B_nnz;         // = B_row_ptr[K]
    reg [15:0] ld_B_Kp1;         // = K + 1
    reg [`DATA_WIDTH-1:0] ld_latch; // holds col while reading val (A data) or col0 (B data)
    reg [`DATA_WIDTH-1:0] ld_b_col0, ld_b_val0; // B data pair 0

    // Gbuf read request
    reg ld_gbuf_req;  // issue read this cycle

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            ld_state        <= LD_IDLE;
            ld_abuf_base    <= 0;
            ld_cnt          <= 0;
            ld_a_data_total <= 0;
            ld_B_nnz        <= 0;
            ld_B_Kp1        <= 0;
            ld_latch        <= 0;
            ld_b_col0       <= 0;
            ld_b_val0       <= 0;
            ld_gbuf_req     <= 1'b0;
            a_buf_wr_en     <= 1'b0; a_buf_wr_addr <= 0; a_buf_wr_data <= 0;
            b_buf_wr_en     <= 1'b0; b_buf_wr_addr <= 0; b_buf_wr_data <= 0;
            load_done       <= 1'b0;
        end else begin
            load_done <= 1'b0;
            a_buf_wr_en <= 1'b0; b_buf_wr_en <= 1'b0;
            ld_state_next <= ld_state;  // default: stay

            ld_state <= ld_state_next;

            case (ld_state)
                LD_IDLE: begin
                    if (load_en) begin
                        // Initialize: A row_ptr section size = M+1
                        ld_cnt          <= 0;
                        ld_a_data_total <= a_ptr_end - a_ptr_start;
                        ld_state_next   <= LD_A_ROW;
                        ld_gbuf_req     <= 1'b1;
                    end else begin
                        ld_gbuf_req     <= 1'b0;
                    end
                end

                // --- Load A_row_ptr: M+1 entries from a_row_sram ---
                LD_A_ROW: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        // Write to A buffer at address = ld_cnt (row index 0..M)
                        a_buf_wr_en   <= 1'b1;
                        a_buf_wr_addr <= ld_cnt[`PE_ABUF_DEPTH_LOG-1:0];
                        a_buf_wr_data <= gbuf_rd_data;
                        if (ld_cnt < M) begin
                            ld_cnt <= ld_cnt + 1;
                        end else begin
                            // Done: M+1 entries loaded (indices 0..M)
                            ld_abuf_base <= M + 1;  // A data section starts after row_ptr
                            ld_cnt <= 0;
                            ld_state_next <= LD_A_DATA_COL;
                            ld_gbuf_req   <= 1'b1;
                        end
                    end
                end

                // --- Load A_data: col/val interleaved ---
                LD_A_DATA_COL: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        // Latch col_idx
                        ld_latch      <= gbuf_rd_data;
                        ld_gbuf_req   <= 1'b1;
                        ld_state_next <= LD_A_DATA_VAL;
                    end
                end

                LD_A_DATA_VAL: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        // Write col_idx to A buffer
                        a_buf_wr_en   <= 1'b1;
                        a_buf_wr_addr <= (ld_abuf_base + 2*ld_cnt);
                        a_buf_wr_data <= ld_latch;
                        // Latch val for write next cycle
                        ld_latch      <= gbuf_rd_data;
                        // Don't issue next gbuf read yet (need 1 cycle to write val)
                        ld_gbuf_req   <= 1'b0;
                        ld_state_next <= LD_A_DATA_WR_V;
                    end
                end

                LD_A_DATA_WR_V: begin
                    // Write val_idx to A buffer
                    a_buf_wr_en   <= 1'b1;
                    a_buf_wr_addr <= (ld_abuf_base + 2*ld_cnt + 1);
                    a_buf_wr_data <= ld_latch;
                    if (ld_cnt + 1 < ld_a_data_total) begin
                        ld_cnt        <= ld_cnt + 1;
                        ld_state_next <= LD_A_DATA_COL;
                        ld_gbuf_req   <= 1'b1;
                    end else begin
                        ld_cnt        <= 0;
                        ld_state_next <= LD_B_ROW;
                        ld_gbuf_req   <= 1'b1;
                    end
                end

                // --- Load B_row_ptr: K+1 entries from b_row_sram ---
                LD_B_ROW: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        // Write to B buffer bank 0 at address = ld_cnt
                        b_buf_wr_en   <= 1'b1;
                        b_buf_wr_addr <= ld_cnt;
                        b_buf_wr_data <= {48'b0, gbuf_rd_data};  // bank0=row_ptr[i]
                        if (ld_cnt < K) begin
                            ld_cnt <= ld_cnt + 1;
                        end else begin
                            // K+1 entries loaded. Last entry = B_row_ptr[K] = nnz_B
                            ld_B_nnz  <= gbuf_rd_data[15:0];
                            ld_B_Kp1  <= K + 1;
                            ld_cnt    <= 0;
                            ld_state_next <= LD_B_DATA_C0;
                            ld_gbuf_req   <= 1'b1;
                        end
                    end
                end

                // --- Load B_data: interleaved col/val pairs into banked buffer ---
                LD_B_DATA_C0: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        ld_b_col0     <= gbuf_rd_data;
                        ld_gbuf_req   <= 1'b1;
                        ld_state_next <= LD_B_DATA_V0;
                    end
                end

                LD_B_DATA_V0: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        ld_b_val0     <= gbuf_rd_data;
                        ld_gbuf_req   <= 1'b1;
                        ld_state_next <= LD_B_DATA_C1;
                    end
                end

                LD_B_DATA_C1: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        // Latch col1 for pair 1
                        ld_latch      <= gbuf_rd_data;
                        ld_gbuf_req   <= 1'b1;
                        ld_state_next <= LD_B_DATA_V1;
                    end
                end

                LD_B_DATA_V1: begin
                    if (gbuf_rd_valid && ld_gbuf_req) begin
                        // Write 2 (col,val) pairs as 64-bit word to B buffer
                        // Format: {col1, val1, col0, val0}
                        b_buf_wr_en   <= 1'b1;
                        b_buf_wr_addr <= ld_B_Kp1 + (ld_cnt >> 1);
                        b_buf_wr_data <= {ld_latch, gbuf_rd_data, ld_b_col0, ld_b_val0};
                        if (ld_cnt + 2 < ld_B_nnz) begin
                            ld_cnt <= ld_cnt + 2;
                            ld_state_next <= LD_B_DATA_C0;
                            ld_gbuf_req   <= 1'b1;
                        end else begin
                            // Done loading B_data. Check for odd tail.
                            if (ld_cnt < ld_B_nnz) begin
                                // One element remaining (odd nnz): pad with 0
                                b_buf_wr_en   <= 1'b1;
                                b_buf_wr_addr <= ld_B_Kp1 + (ld_cnt >> 1) + 1;
                                b_buf_wr_data <= {32'b0, ld_b_col0, ld_b_val0};
                            end
                            ld_state_next <= LD_DONE;
                            ld_gbuf_req   <= 1'b0;
                        end
                    end
                end

                LD_DONE: begin
                    load_done   <= 1'b1;
                    ld_state_next <= LD_IDLE;
                    ld_gbuf_req   <= 1'b0;
                end

                default: begin
                    ld_gbuf_req   <= 1'b0;
                    ld_state_next <= LD_IDLE;
                end
            endcase
        end
    end

    // Drive gbuf_rd_en/gbuf_rd_addr from load FSM
    always @(*) begin
        gbuf_rd_en   = 1'b0;
        gbuf_rd_addr = 0;

        case (ld_state)
            LD_IDLE: ;
            LD_A_ROW: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = a_row_sram + ld_cnt;
            end
            LD_A_DATA_COL: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = a_col_sram + a_ptr_start + ld_cnt;
            end
            LD_A_DATA_VAL: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = a_val_sram + a_ptr_start + ld_cnt;
            end
            LD_A_DATA_WR_V: begin
                gbuf_rd_en   = 1'b0;  // writing, not reading
            end
            LD_B_ROW: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = b_row_sram + ld_cnt;
            end
            LD_B_DATA_C0: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = b_col_sram + ld_cnt;
            end
            LD_B_DATA_V0: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = b_val_sram + ld_cnt;
            end
            LD_B_DATA_C1: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = b_col_sram + ld_cnt + 1;
            end
            LD_B_DATA_V1: begin
                gbuf_rd_en   = ld_gbuf_req;
                gbuf_rd_addr = b_val_sram + ld_cnt + 1;
            end
        endcase
    end
    // Decompress → MUL array
    wire [`N_MAC-1:0]             lane_valid;
    wire [`N_MAC*`DATA_WIDTH-1:0] lane_a_val;
    wire [`N_MAC*`DATA_WIDTH-1:0] lane_b_val;
    wire [`N_MAC*`DATA_WIDTH-1:0] lane_col_idx;
    wire [`N_MAC*`DATA_WIDTH-1:0] lane_row_idx;

    // MUL array → Aggregation
    wire [`N_MAC-1:0]             mul_valid;
    wire [`N_MAC*`DATA_WIDTH-1:0] partial_value;
    wire [`N_MAC*`DATA_WIDTH-1:0] mul_col_idx;
    wire [`N_MAC*`DATA_WIDTH-1:0] mul_row_idx;

    // Decompress → Aggregation (Chisel-style row pulses)
    wire decomp_row_start;
    wire decomp_row_end;
    wire [`MAX_DIM_BITS-1:0] decomp_agg_row_id;

    // Aggregation → Decompress (backpressure)
    wire agg_stall;

    //=========================================================================
    // Sub-module Instantiations
    //=========================================================================

    // Pipeline-based Decompress Unit (S0 FETCH + S1 STREAM)
    pe_decompress u_decompress (
        .start           (start),
        .done            (decomp_done),
        .row_start       (row_start),
        .row_end         (row_end),
        .a_ptr_start     (a_ptr_start),
        .M               (M),
        .K               (K),
        .N               (N),
        .a_buf_rd_en     (a_buf_rd_en),
        .a_buf_rd_addr   (a_buf_rd_addr),
        .a_buf_rd_data   (a_buf_rd_data),
        .a_buf_rd_valid  (a_buf_rd_valid),
        .b_buf_rd_en     (b_buf_rd_en),
        .b_buf_rd_addr   (b_buf_rd_addr),
        .b_buf_rd_data   (b_buf_rd_data),
        .b_buf_rd_valid  (b_buf_rd_valid),
        .lane_valid      (lane_valid),
        .lane_a_val      (lane_a_val),
        .lane_b_val      (lane_b_val),
        .lane_col_idx    (lane_col_idx),
        .lane_row_idx    (lane_row_idx),
        .row_start_pulse (decomp_row_start),
        .row_end_pulse   (decomp_row_end),
        .agg_row_id      (decomp_agg_row_id),
        .agg_stall       (agg_stall),
        .aclk            (aclk),
        .aresetn         (aresetn)
    );

    // Configurable MUL Array (3-stage pipeline: reg→ALU→reg)
    // Row start/end/id pipelined to align with product latency
    wire               mul_row_start, mul_row_end;
    wire [`MAX_DIM_BITS-1:0] mul_row_id;

    pe_mul_array u_mul_array (
        .op_type       (op_type),
        .lane_valid    (lane_valid),
        .lane_a_val    (lane_a_val),
        .lane_b_val    (lane_b_val),
        .lane_col_idx  (lane_col_idx),
        .lane_row_idx  (lane_row_idx),
        .row_start_in  (decomp_row_start),
        .row_end_in    (decomp_row_end),
        .row_id_in     (decomp_agg_row_id),
        .mul_valid     (mul_valid),
        .partial_value (partial_value),
        .col_idx       (mul_col_idx),
        .row_idx       (mul_row_idx),
        .row_start_out (mul_row_start),
        .row_end_out   (mul_row_end),
        .row_id_out    (mul_row_id),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // Aggregation Unit + SPA — uses pipeline-delayed row signals
    pe_aggregation u_aggregation (
        .mul_valid       (mul_valid),
        .partial_value   (partial_value),
        .col_idx         (mul_col_idx),
        .row_idx         (mul_row_idx),
        .row_start_pulse (mul_row_start),
        .row_end_pulse   (mul_row_end),
        .agg_row_id      (mul_row_id),
        .agg_stall       (agg_stall),
        .agg_idle        (agg_idle),
        .out_valid       (out_valid),
        .out_col         (out_col),
        .out_val         (out_val),
        .out_row_id      (out_row_id),
        .out_nnz         (out_nnz),
        .aclk            (aclk),
        .aresetn         (aresetn)
    );

    // Done
    wire decomp_done;
    assign done = decomp_done;

endmodule
