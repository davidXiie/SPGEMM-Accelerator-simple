//=============================================================================
// File     : pe_decompress.v
// Project  : SPGEMM-Accelerator
// Brief    : Pipeline-based Decompress / Sparse Decode Unit
//
//   Architecture: 3-stage pipeline with Chisel-style handshaking
//     S0 (FETCH):  Read A_row_ptr, A_col_idx, B_row_ptr → build entry_fifo
//     S1 (STREAM): Read entries, stream B elements N_MAC per cycle → mac_fifo
//     S2 (output): Aggregation interface (row_start/row_end signaling)
//
//   Chisel patterns used:
//     d1_moving = !d2_nextValid           → upstream stalls until downstream free
//     Self-looping: each stage can iterate independently
//     pipe_empty: all stages idle and buffers empty → PE done
//
//   Row-Block: all B elements from one A row are concatenated.
//              Only the row's last batch may have idle lanes.
//=============================================================================

`include "defines.vh"

module pe_decompress (
    input  wire                      start,
    output wire                      done,

    input  wire [`MAX_DIM_BITS-1:0]  row_start,
    input  wire [`MAX_DIM_BITS-1:0]  row_end,
    input  wire [15:0]               a_ptr_start,
    input  wire [`MAX_DIM_BITS-1:0]  M,    // total A rows = A data base offset
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // A Buffer
    output reg                       a_buf_rd_en,
    output reg  [`PE_ABUF_DEPTH_LOG-1:0] a_buf_rd_addr,
    input  wire [`DATA_WIDTH-1:0]    a_buf_rd_data,
    input  wire                      a_buf_rd_valid,

    // B Buffer (banked: N_MAC banks, per-bank rd_en/addr)
    output reg  [`N_MAC-1:0]         b_buf_rd_en,
    output reg  [`N_MAC*`PE_BBUF_DEPTH_LOG-1:0] b_buf_rd_addr,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] b_buf_rd_data,
    input  wire [`N_MAC-1:0]         b_buf_rd_valid,

    // MAC lane output (to MUL array)
    output reg  [`N_MAC-1:0]         lane_valid,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_a_val,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_b_val,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_col_idx,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_row_idx,

    // Aggregation control
    output reg                        row_start_pulse,
    output reg                        row_end_pulse,
    output reg  [`MAX_DIM_BITS-1:0]   agg_row_id,

    // Backpressure from aggregation (bank conflict stall)
    input  wire                       agg_stall,
    input  wire                       agg_idle,     // aggregation is idle (no in-flight partials)

    input  wire                       aclk,
    input  wire                       aresetn
);

    // Width-cast N_MAC for use in arithmetic (N_MAC is a `define, needs explicit width)
    wire [9:0]  N_MAC_10 = `N_MAC;
    wire [`WORKLOAD_BITS-1:0] N_MAC_WL = `N_MAC;
    // Effective N_MAC: 2 elements per cycle (banks store interleaved col/val pairs)
    localparam N_MAC_EFF = `N_MAC / 2;  // = 2
    wire [9:0]  N_MAC_EFF_10 = N_MAC_EFF;
    wire [`WORKLOAD_BITS-1:0] N_MAC_EFF_WL = N_MAC_EFF;

    //=========================================================================
    // Entry FIFO (between S0 FETCH and S1 STREAM)
    // Entry 64-bit: {row_id[9:0], 1'b0, 5'b0, b_len[15:0], b_start[15:0], a_val[15:0]}
    //=========================================================================
    localparam ENTRY_WIDTH = 64;
    localparam ENTRY_DEPTH = 32;
    localparam ENTRY_ADDR  = 5;
    reg  [ENTRY_WIDTH-1:0] entry_mem [0:ENTRY_DEPTH-1];
    reg  [ENTRY_ADDR:0]    entry_wr_ptr, entry_rd_ptr;
    wire                   entry_empty, entry_full;
    wire [ENTRY_WIDTH-1:0] entry_rdata;

    localparam E_VAL_LO   = 0;
    localparam E_VAL_HI   = 15;
    localparam E_START_LO = 16;
    localparam E_START_HI = 31;
    localparam E_LEN_LO   = 32;
    localparam E_LEN_HI   = 47;
    localparam E_ROW_LO   = 54;
    localparam E_ROW_HI   = 63;

    assign entry_empty = (entry_wr_ptr == entry_rd_ptr);
    assign entry_full  = ((entry_wr_ptr + 1) == entry_rd_ptr) ||
                         ((entry_wr_ptr == ENTRY_DEPTH - 1) && (entry_rd_ptr == 0));
    assign entry_rdata = entry_mem[entry_rd_ptr[ENTRY_ADDR-1:0]];

    //=========================================================================
    // Pipeline registers
    //=========================================================================
    reg a_buf_rd_valid_r;
    reg [`DATA_WIDTH-1:0] a_buf_rd_data_r;
    reg b_buf_rd_valid_r;
    reg [`N_MAC*`DATA_WIDTH-1:0] b_buf_rd_data_r;

    //=========================================================================
    // S0 (FETCH): A element probe → B row resolve → entry_fifo write
    //
    //   Sub-states:
    //     S0_IDLE    : wait for start
    //     S0_A_PROBE : read A_row_ptr[(cur_row, cur_row+1)]
    //     S0_A_ELEM  : read A_col_idx[p], A_val[p] → get (k, a_val)
    //     S0_B_PROBE : read B_row_ptr[k], B_row_ptr[k+1] → get (b_start, b_end, b_len)
    //     S0_PUSH    : push entry to FIFO, advance p
    //=========================================================================
    localparam S0_IDLE      = 3'd0;
    localparam S0_A_PROBE   = 3'd1;
    localparam S0_A_ELEM_C  = 3'd2;  // read col_idx from interleaved A data
    localparam S0_A_ELEM_V  = 3'd3;  // read val from interleaved A data
    localparam S0_B_PROBE   = 3'd4;
    localparam S0_PUSH      = 3'd5;

    reg [2:0] s0_state;
    // Row tracking
    reg [`MAX_DIM_BITS-1:0] s0_cur_row;
    reg [15:0] s0_a_start, s0_a_end;
    reg [15:0] s0_a_ptr;
    reg a_ptr_end_loaded;
    // A element registers
    reg [`MAX_DIM_BITS-1:0] s0_k;
    reg [`DATA_WIDTH-1:0]   s0_a_val;
    // B row registers
    reg [15:0] s0_b_start, s0_b_end;
    // Row-Block accumulator (per A row)
    reg [`WORKLOAD_BITS-1:0] s0_total_B_nnz;
    // All rows done
    reg s0_all_rows_done;

    // S0 pipeline control
    // s0_moving: S0 can produce → when entry_fifo not full
    wire s0_stall = entry_full;
    wire s0_advance;  // S0 completed one entry

    //=========================================================================
    // S1 (STREAM): read entry → stream N_MAC B elements → MAC lane dispatch
    //
    //   Self-looping: continues streaming until all B elements consumed,
    //                 then reads next entry from FIFO.
    //=========================================================================
    localparam S1_IDLE     = 2'd0;
    localparam S1_STREAM   = 2'd1;
    localparam S1_DRAIN    = 2'd2;  // wait for last batch to be consumed by aggregation

    reg [1:0] s1_state;
    // Current entry
    reg [9:0]  s1_b_len;
    reg [14:0] s1_b_start;
    reg [`DATA_WIDTH-1:0] s1_a_val;
    // B stream position
    reg [`WORKLOAD_BITS-1:0] s1_b_rem;       // remaining B elements in current entry
    reg [`WORKLOAD_BITS-1:0] s1_batch_cnt;   // total batches dispatched
    reg [`WORKLOAD_BITS-1:0] s1_total_batches; // ceil(s0_total_B_nnz / N_MAC)
    reg [`WORKLOAD_BITS-1:0] s1_row_b_consumed; // B elements consumed so far
    // Preload next entry (1 cycle ahead)
    reg                       s1_next_valid;
    reg [9:0]                s1_next_b_len;
    reg [14:0]               s1_next_b_start;
    reg [`DATA_WIDTH-1:0]    s1_next_a_val;
    // Row ID tracking
    reg [`MAX_DIM_BITS-1:0]  s1_cur_row;
    // Row batch tracking
    reg [`WORKLOAD_BITS-1:0] s1_total_B_nnz;

    // S1 pipeline control
    // s1_stall: aggregation backpressure
    wire s1_stall = agg_stall;
    wire s1_advance; // S1 issued one MAC batch

    //=========================================================================
    // S0 Sequencer (Chisel-style: self-looping state machine)
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s0_state          <= S0_IDLE;
            s0_cur_row        <= 0;
            s0_a_start        <= 0;
            s0_a_end          <= 0;
            s0_a_ptr          <= 0;
            a_ptr_end_loaded  <= 1'b0;
            s0_k              <= 0;
            s0_a_val          <= 0;
            s0_b_start        <= 0;
            s0_b_end          <= 0;
            s0_total_B_nnz    <= 0;
            s0_all_rows_done  <= 1'b0;
            entry_wr_ptr      <= 0;
            entry_rd_ptr      <= 0;
        end else begin
            case (s0_state)

                S0_IDLE: begin
                    if (start) begin
                        s0_cur_row       <= row_start;
                        s0_a_start       <= 0;
                        s0_a_end         <= 0;
                        s0_a_ptr         <= 0;
                        a_ptr_end_loaded <= 1'b0;
                        s0_total_B_nnz   <= 0;
                        s0_all_rows_done <= 1'b0;
                        s0_state <= S0_A_PROBE;
                    end
                end

                // Read A_row_ptr[cur_row] and A_row_ptr[cur_row+1]
                S0_A_PROBE: begin
                    if (a_buf_rd_valid_r && !a_ptr_end_loaded) begin
                        s0_a_start       <= a_buf_rd_data_r[15:0];
                        a_ptr_end_loaded <= 1'b1;
                    end else if (a_buf_rd_valid_r && a_ptr_end_loaded) begin
                        s0_a_end         <= a_buf_rd_data_r[15:0];
                        a_ptr_end_loaded <= 1'b0;
                        s0_a_ptr         <= s0_a_start;  // init element pointer
                        if (a_buf_rd_data_r[15:0] > s0_a_start) begin
                            // Non-empty row: start scanning
                            s0_state <= S0_A_ELEM_C;
                        end else begin
                            // Empty row: skip to next
                            if (s0_cur_row >= row_end) begin
                                s0_all_rows_done <= 1'b1;
                                s0_state <= S0_IDLE;
                            end else begin
                                s0_cur_row <= s0_cur_row + 1;
                                s0_state   <= S0_A_PROBE;
                            end
                        end
                    end
                end

                // Read A_col_idx[p] from interleaved A data
                S0_A_ELEM_C: begin
                    if (a_buf_rd_valid_r) begin
                        s0_k     <= a_buf_rd_data_r[`MAX_DIM_BITS-1:0];
                        s0_state <= S0_A_ELEM_V;
                    end
                end

                // Read A_val[p] from interleaved A data
                S0_A_ELEM_V: begin
                    if (a_buf_rd_valid_r) begin
                        s0_a_val <= a_buf_rd_data_r;
                        s0_state <= S0_B_PROBE;
                    end
                end

                // Read B_row_ptr[k], B_row_ptr[k+1] → get (b_start, b_end, b_len)
                S0_B_PROBE: begin
                    if (b_buf_rd_valid_r) begin
                        if (!s0_b_start_loaded) begin
                            s0_b_start       <= b_buf_rd_data_r[15:0];
                            s0_b_start_loaded <= 1'b1;
                        end else begin
                            s0_b_end         <= b_buf_rd_data_r[15:0];
                            s0_b_start_loaded <= 1'b0;
                            s0_state <= S0_PUSH;
                        end
                    end
                end

                // Push entry to FIFO, advance to next A element
                S0_PUSH: begin
                    if (!s0_stall) begin
                        // Write entry: {row_id, b_len, b_start, a_val}
                        entry_mem[entry_wr_ptr[ENTRY_ADDR-1:0]] <= {
                            s0_cur_row[9:0],      // row_id [63:54]
                            10'b0,                  // padding [53:48]
                            (s0_b_end - s0_b_start),  // b_len [47:32]
                            s0_b_start[15:0],      // b_start [31:16]
                            s0_a_val               // a_val [15:0]
                        };
                        entry_wr_ptr <= entry_wr_ptr + 1;
                        s0_total_B_nnz <= s0_total_B_nnz + (s0_b_end - s0_b_start);

                        s0_a_ptr <= s0_a_ptr + 1;
                        if (s0_a_ptr + 1 >= s0_a_end) begin  // off-by-one: single-element row
                            // All elements in this row scanned → next row
                            // Send row-level info to S1 (via a separate signal)
                            if (s0_cur_row >= row_end) begin
                                s0_all_rows_done <= 1'b1;
                                s0_state <= S0_IDLE;
                            end else begin
                                s0_cur_row <= s0_cur_row + 1;
                                s0_total_B_nnz <= 0;
                                s0_state <= S0_A_PROBE;
                            end
                        end else begin
                            s0_state <= S0_A_ELEM_C;
                        end
                    end
                end

            endcase
        end
    end

    reg s0_b_start_loaded;  // sub-states within S0_B_PROBE

    // Reset s0_b_start_loaded
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s0_b_start_loaded <= 1'b0;
        end else if (s0_state != S0_B_PROBE) begin
            s0_b_start_loaded <= 1'b0;
        end
    end

    //=========================================================================
    // S1 Sequencer (streaming with preload, self-looping)
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s1_state          <= S1_IDLE;
            s1_b_len          <= 0;
            s1_b_start        <= 0;
            s1_a_val          <= 0;
            s1_b_rem          <= 0;
            s1_batch_cnt      <= 0;
            s1_total_batches  <= 0;
            s1_row_b_consumed <= 0;
            s1_next_valid     <= 1'b0;
            s1_next_b_len     <= 0;
            s1_next_b_start   <= 0;
            s1_next_a_val     <= 0;
            s1_cur_row        <= 0;
            s1_total_B_nnz    <= 0;
        end else begin
            case (s1_state)

                S1_IDLE: begin
                    // Wait for first entries in FIFO
                    if (!entry_empty) begin
                        // Load first entry: {row_id, b_len, b_start, a_val}
                        s1_cur_row <= entry_rdata[E_ROW_HI:E_ROW_LO];
                        s1_b_len   <= entry_rdata[E_LEN_HI:E_LEN_LO];
                        s1_b_start <= entry_rdata[E_START_HI:E_START_LO];
                        s1_a_val   <= entry_rdata[E_VAL_HI:E_VAL_LO];
                        s1_b_rem   <= entry_rdata[E_LEN_HI:E_LEN_LO];
                        entry_rd_ptr <= entry_rd_ptr + 1;
                        s1_batch_cnt      <= 0;
                        s1_row_b_consumed <= 0;
                        // Preload next entry if available
                        if (!entry_empty && entry_rd_ptr + 1 != entry_wr_ptr) begin
                            s1_next_valid   <= 1'b1;
                        end
                        s1_state <= S1_STREAM;
                    end
                end

                S1_STREAM: begin
                    if (s1_advance) begin
                        s1_batch_cnt      <= s1_batch_cnt + 1;
                        s1_row_b_consumed <= s1_row_b_consumed + N_MAC_EFF_WL;

                        if (s1_b_rem > N_MAC_EFF) begin
                            // More elements in current entry
                            s1_b_rem <= s1_b_rem - N_MAC_EFF_10;
                        end else begin
                            // Current entry exhausted → switch to next
                            if (s1_next_valid) begin
                                s1_b_len     <= s1_next_b_len;
                                s1_b_start   <= s1_next_b_start;
                                s1_a_val     <= s1_next_a_val;
                                s1_b_rem     <= s1_next_b_len - (N_MAC_EFF_10 - s1_b_rem);
                                entry_rd_ptr <= entry_rd_ptr + 1;
                                // Preload next-next
                                if (entry_rd_ptr + 2 != entry_wr_ptr) begin
                                    s1_next_valid <= 1'b1;
                                end else begin
                                    s1_next_valid <= 1'b0;
                                end
                            end else begin
                                // No more entries: row done
                                s1_b_rem <= 0;
                                s1_state <= S1_IDLE;
                            end
                        end

                        // Row boundary: detected by entry exhaustion (s1_next_valid=0)
                        // when current entry is exhausted and no next entry exists
                    end
                end

            endcase
        end
    end

    // Preload: intermediate reads for Icarus compatibility (no mem[addr][HI:LO])
    wire [ENTRY_ADDR-1:0]  entry_rd_next_addr;
    wire [ENTRY_ADDR-1:0]  entry_rd_cur_addr;
    wire [ENTRY_WIDTH-1:0] entry_rd_next_data;
    wire [ENTRY_WIDTH-1:0] entry_rd_cur_data;

    assign entry_rd_next_addr = (entry_rd_ptr + 1'b1);
    assign entry_rd_cur_addr  = entry_rd_ptr[ENTRY_ADDR-1:0];
    assign entry_rd_next_data = entry_mem[entry_rd_next_addr];
    assign entry_rd_cur_data  = entry_mem[entry_rd_cur_addr];

    // Preload logic for next entry (combinational read from entry_mem)
    always @(posedge aclk) begin
        if (s1_state == S1_STREAM && s1_advance && s1_b_rem <= N_MAC_EFF) begin
            if (s1_next_valid) begin
                // Read next entry from entry_mem (next position after current)
                s1_next_b_len   <= entry_rd_next_data[E_LEN_HI:E_LEN_LO];
                s1_next_b_start <= entry_rd_next_data[E_START_HI:E_START_LO];
                s1_next_a_val   <= entry_rd_next_data[E_VAL_HI:E_VAL_LO];
            end
        end
        if (s1_state == S1_IDLE && !entry_empty && entry_rd_ptr + 1 != entry_wr_ptr) begin
            // Preload next on first cycle of new row
            s1_next_valid   <= 1'b1;
            s1_next_b_len   <= entry_rd_cur_data[E_LEN_HI:E_LEN_LO];
            s1_next_b_start <= entry_rd_cur_data[E_START_HI:E_START_LO];
            s1_next_a_val   <= entry_rd_cur_data[E_VAL_HI:E_VAL_LO];
        end
    end

    // S1 advance: issued one MAC batch successfully
    assign s1_advance = (s1_state == S1_STREAM) && b_buf_rd_valid_r && !s1_stall;

    //=========================================================================
    // S0: Buffer read address generation
    //   A buffer layout: [0..row_end+1] = A_row_ptr, then interleaved (col,val)
    //   a_data_base = row_end + 2 = start of interleaved data section
    //=========================================================================
    wire [15:0] a_data_base_w;
    assign a_data_base_w = M + 1;  // interleaved A data starts after A_row_ptr

    wire [15:0] s0_a_data_rel;
    assign s0_a_data_rel = 2 * (s0_a_ptr - a_ptr_start);

    // S0: A buffer address generation only
    always @(*) begin
        a_buf_rd_en   = 1'b0;
        a_buf_rd_addr = 0;

        case (s0_state)
            S0_A_PROBE: begin
                a_buf_rd_en   = 1'b1;
                a_buf_rd_addr = s0_cur_row[`PE_ABUF_DEPTH_LOG-1:0]
                              + {{`PE_ABUF_DEPTH_LOG-1{1'b0}}, a_ptr_end_loaded};
            end
            S0_A_ELEM_C: begin
                a_buf_rd_en   = 1'b1;
                a_buf_rd_addr = a_data_base_w[`PE_ABUF_DEPTH_LOG-1:0]
                              + s0_a_data_rel[`PE_ABUF_DEPTH_LOG-1:0];
            end
            S0_A_ELEM_V: begin
                a_buf_rd_en   = 1'b1;
                a_buf_rd_addr = a_data_base_w[`PE_ABUF_DEPTH_LOG-1:0]
                              + s0_a_data_rel[`PE_ABUF_DEPTH_LOG-1:0]
                              + 1'b1;
            end
            S0_B_PROBE: begin
                // B reads handled in unified block below
            end
        endcase
    end

    //=========================================================================
    // Unified B buffer read address generation (S0 + S1)
    //=========================================================================
    wire [14:0] s1_b_elem_idx;
    assign s1_b_elem_idx = s1_b_start + (s1_b_len[14:0] - s1_b_rem[14:0]);

    wire [`PE_BBUF_DEPTH_LOG-1:0] s1_b_addr;
    assign s1_b_addr = K + 1 + (s1_b_elem_idx >> 1);

    always @(*) begin
        b_buf_rd_en   = {`N_MAC{1'b0}};
        b_buf_rd_addr = 0;

        // S1: streaming B data reads (all banks)
        if (s1_state == S1_STREAM && s1_b_rem > 0) begin
            for (integer m = 0; m < `N_MAC; m = m + 1) begin
                b_buf_rd_en[m] = 1'b1;
                b_buf_rd_addr[m*`PE_BBUF_DEPTH_LOG +: `PE_BBUF_DEPTH_LOG] = s1_b_addr;
            end
        end

        // S0: B_row_ptr probe (bank 0 only)
        if (s0_state == S0_B_PROBE) begin
            b_buf_rd_en[0] = 1'b1;
            b_buf_rd_addr[0 +: `PE_BBUF_DEPTH_LOG]
                = s0_k[`PE_BBUF_DEPTH_LOG-1:0]
                + {{`PE_BBUF_DEPTH_LOG-1{1'b0}}, s0_b_start_loaded};
        end
    end

    //=========================================================================
    // Pipeline: latch buffer reads
    //=========================================================================
    always @(posedge aclk) begin
        a_buf_rd_valid_r <= a_buf_rd_valid;
        a_buf_rd_data_r  <= a_buf_rd_data;
        b_buf_rd_valid_r <= |b_buf_rd_valid;
        b_buf_rd_data_r  <= b_buf_rd_data;
    end

    //=========================================================================
    // MAC Lane Dispatcher (output from S1 to MUL array)
    //   B buffer stores interleaved (col,val) pairs across 4 banks:
    //     bank0=col[2i], bank1=val[2i], bank2=col[2i+1], bank3=val[2i+1]
    //   Only lanes 0 and 2 are used (2 elements per cycle), each paired
    //   with its adjacent val bank (lanes 1 and 3 disabled).
    //=========================================================================
    reg [`DATA_WIDTH-1:0] s1_cur_row_vec;
    always @(*) begin
        s1_cur_row_vec = {{`DATA_WIDTH-`MAX_DIM_BITS{1'b0}}, s1_cur_row};
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            lane_valid   <= 0;
            lane_a_val   <= 0;
            lane_b_val   <= 0;
            lane_col_idx <= 0;
            lane_row_idx <= 0;
        end else begin
            lane_valid <= 0;

            if (s1_state == S1_STREAM && b_buf_rd_valid_r && !s1_stall) begin
                // Element 0: bank0=col, bank1=val → lane 0
                if (b_buf_rd_en[0] && 0 < s1_b_rem) begin
                    lane_valid[0] <= 1'b1;
                    lane_a_val[0*`DATA_WIDTH +: `DATA_WIDTH] <= s1_a_val;
                    lane_b_val[0*`DATA_WIDTH +: `DATA_WIDTH]
                        <= b_buf_rd_data_r[1*`DATA_WIDTH +: `DATA_WIDTH];  // bank1 = val
                    lane_col_idx[0*`DATA_WIDTH +: `DATA_WIDTH]
                        <= b_buf_rd_data_r[0*`DATA_WIDTH +: `DATA_WIDTH];  // bank0 = col
                    lane_row_idx[0*`DATA_WIDTH +: `DATA_WIDTH] <= s1_cur_row_vec;
                end
                // Element 1: bank2=col, bank3=val → lane 2
                if (b_buf_rd_en[2] && 1 < s1_b_rem) begin
                    lane_valid[2] <= 1'b1;
                    lane_a_val[2*`DATA_WIDTH +: `DATA_WIDTH]
                        <= (2 < s1_b_rem) ? s1_a_val : s1_next_a_val;
                    lane_b_val[2*`DATA_WIDTH +: `DATA_WIDTH]
                        <= b_buf_rd_data_r[3*`DATA_WIDTH +: `DATA_WIDTH];  // bank3 = val
                    lane_col_idx[2*`DATA_WIDTH +: `DATA_WIDTH]
                        <= b_buf_rd_data_r[2*`DATA_WIDTH +: `DATA_WIDTH];  // bank2 = col
                    lane_row_idx[2*`DATA_WIDTH +: `DATA_WIDTH] <= s1_cur_row_vec;
                end
            end
        end
    end

    //=========================================================================
    // Aggregation control signals (row_start / row_end pulses)
    //=========================================================================
    reg s1_first_batch;  // first batch of a new row
    reg s1_last_batch;   // last batch of a row

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            row_start_pulse <= 1'b0;
            row_end_pulse   <= 1'b0;
            agg_row_id      <= 0;
            s1_first_batch  <= 1'b0;
            s1_last_batch   <= 1'b0;
        end else begin
            row_start_pulse <= 1'b0;
            row_end_pulse   <= 1'b0;

            if (s1_state == S1_IDLE && !entry_empty) begin
                s1_first_batch <= 1'b1;
            end

            if (s1_advance) begin
                if (s1_first_batch) begin
                    row_start_pulse <= 1'b1;
                    agg_row_id      <= s1_cur_row;
                    s1_first_batch  <= 1'b0;
                end
                if (s1_batch_cnt + 1 >= s1_total_batches) begin
                    row_end_pulse <= 1'b1;
                end
            end
        end
    end

    //=========================================================================
    // Pipe empty & Done detection (Chisel-style)
    //=========================================================================
    wire s0_done = (s0_state == S0_IDLE);
    wire s1_done = (s1_state == S1_IDLE);
    wire pipe_empty = s0_done && s1_done && entry_empty && !start;

    assign done = pipe_empty && s0_all_rows_done && agg_idle;

endmodule
