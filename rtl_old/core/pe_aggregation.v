//=============================================================================
// File     : pe_aggregation.v
// Project  : SPGEMM-Accelerator
// Brief    : Pipeline-based Aggregation Unit + SPA (Sparse Accumulator)
//
//   Accepts FP16 partial products from MUL array, aggregates by column index j.
//   Uses Banked Partial Row Buffer with conflict detection and stalling.
//
//   Chisel patterns:
//     - Self-looping row output (drain touched_cols sequentially)
//     - m_acc_q-style accumulation registers
//     - Backpressure (agg_stall) when bank conflicts detected
//
//   Interface with pe_decompress:
//     row_start_pulse: clear SPA, start new row
//     row_end_pulse:   finalize current row, start output drained
//     agg_stall:       assertion stalls the upstream streamer
//=============================================================================

`include "defines.vh"

module pe_aggregation (
    // Input from MUL array (3-stage pipelined)
    input  wire [`N_MAC-1:0]             mul_valid,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] partial_value,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] col_idx,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] row_idx,

    // Control from pe_decompress (Chisel-style row pulses)
    input  wire                          row_start_pulse,
    input  wire                          row_end_pulse,
    input  wire [`MAX_DIM_BITS-1:0]      agg_row_id,

    // Backpressure output (to pe_decompress S1 stage)
    output wire                          agg_stall,
    output wire                          agg_idle,     // idle: no active row, no outputting, no in-flight

    // Output: completed C row (one element per cycle)
    output reg                           out_valid,
    output reg  [`DATA_WIDTH-1:0]        out_col,
    output reg  [`DATA_WIDTH-1:0]        out_val,
    output reg  [`MAX_DIM_BITS-1:0]      out_row_id,
    output reg  [`MAX_DIM_BITS-1:0]      out_nnz,

    input  wire                          aclk,
    input  wire                          aresetn
);

    //=========================================================================
    // SPA: Partial Row Buffer
    //   acc_val[512]: FP16 accumulated values
    //   acc_valid[512]: bit per column = touched
    //   touched_fifo[512]: column indices in insertion order
    //=========================================================================
    reg [`DATA_WIDTH-1:0] acc_val [0:`MAX_N-1];
    reg [`MAX_N-1:0] acc_valid_bits;  // bit vector for fast lookup
    reg [`MAX_DIM_BITS-1:0] touched_fifo [0:`MAX_N-1];
    reg [`MAX_DIM_BITS:0] touched_wr_ptr, touched_rd_ptr;
    wire touched_empty, touched_full;

    // Row tracking
    reg [`MAX_DIM_BITS-1:0] current_row;
    reg [`MAX_DIM_BITS-1:0] current_row_nnz;
    reg row_active;     // currently processing a row
    reg row_outputting;  // currently draining row output

    //=========================================================================
    // Banked access: bank = col_idx % N_MAC
    //=========================================================================
    wire [`N_MAC_BITS-1:0] bank_id [`N_MAC-1:0];
    wire [`MAX_DIM_BITS-1:0] col_int [`N_MAC-1:0];

    genvar m;
    generate
        for (m = 0; m < `N_MAC; m = m + 1) begin : gen_bank
            assign col_int[m] = col_idx[m*`DATA_WIDTH +: `MAX_DIM_BITS];
            assign bank_id[m] = col_int[m][`N_MAC_BITS-1:0];
        end
    endgenerate

    //=========================================================================
    // Serialized lane processing: one valid product per cycle
    //   Avoids multi-lane race on touched_wr_ptr / current_row_nnz.
    //   No conflict detection needed with serialization.
    //=========================================================================
    reg [`N_MAC_BITS-1:0] proc_lane;  // serialization: 0 = idle, 1,2 = pending lanes

    // Stall S1 when we have pending lanes to process (proc_lane != 0)
    assign agg_stall = row_outputting || (row_active && proc_lane != 0);

    wire agg_is_idle = !row_active && !row_outputting && (mul_valid == 0) && (proc_lane == 0);
    assign agg_idle = agg_is_idle;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            acc_valid_bits  <= 0;
            touched_wr_ptr  <= 0; touched_rd_ptr <= 0;
            current_row     <= 0; current_row_nnz <= 0;
            row_active      <= 1'b0; row_outputting <= 1'b0;
            out_valid <= 0; out_col <= 0; out_val <= 0; out_row_id <= 0; out_nnz <= 0;
            proc_lane <= 0;
        end else begin
            out_valid <= 1'b0;

            if (row_start_pulse) begin
                touched_wr_ptr <= 0; touched_rd_ptr <= 0;
                current_row <= agg_row_id; current_row_nnz <= 0;
                row_active <= 1'b1; row_outputting <= 1'b0;
                proc_lane <= 0;
                for (integer c = 0; c < `MAX_N; c = c + 1) acc_valid_bits[c] <= 1'b0;
            end

            // Serialized accumulation: one lane per cycle
            if (row_active && mul_valid != 0) begin
                // Process exactly ONE valid lane this cycle
                for (integer m = 0; m < `N_MAC; m = m + 1) begin
                    if (mul_valid[m] && proc_lane == m) begin
                        if (!acc_valid_bits[col_int[m]]) begin
                            acc_val[col_int[m]] <= partial_value[m*`DATA_WIDTH +: `DATA_WIDTH];
                            acc_valid_bits[col_int[m]] <= 1'b1;
                            touched_fifo[touched_wr_ptr] <= col_int[m];
                            touched_wr_ptr <= touched_wr_ptr + 1;
                            current_row_nnz <= current_row_nnz + 1;
                        end else begin
                            acc_val[col_int[m]] <= acc_val[col_int[m]] + partial_value[m*`DATA_WIDTH +: `DATA_WIDTH];
                        end
                        // Advance: next valid lane or done with this batch
                        if (m + 1 < `N_MAC && mul_valid[m+1])
                            proc_lane <= m + 1;
                        else
                            proc_lane <= 0;
                    end
                end
            end

            if (row_end_pulse && row_active) begin
                row_active <= 1'b0; row_outputting <= 1'b1; touched_rd_ptr <= 0;
            end

            if (row_outputting) begin
                if (!touched_empty) begin
                    out_valid <= 1'b1;
                    out_col   <= {{`DATA_WIDTH-`MAX_DIM_BITS{1'b0}}, touched_fifo[touched_rd_ptr]};
                    out_val   <= acc_val[touched_fifo[touched_rd_ptr]];
                    out_row_id <= current_row;  out_nnz <= current_row_nnz;
                    touched_rd_ptr <= touched_rd_ptr + 1;
                end else begin
                    row_outputting <= 1'b0; current_row_nnz <= 0;
                end
            end
        end
    end

    // FIFO status
    assign touched_empty = (touched_wr_ptr == touched_rd_ptr);
    assign touched_full  = (touched_wr_ptr - touched_rd_ptr >= `MAX_N);

endmodule
