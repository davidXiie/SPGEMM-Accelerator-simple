//=============================================================================
// File     : pe_load_ctrl.v
// Brief    : Loads A (round-robin partitioned) and B (broadcast) from global
//            buffers into the PE cluster.  Three sub-phases:
//
//   A loading (per-PE):
//     Row rid → PE (rid % N_PE).  Writes A_val / A_col to target PE's local
//     buffer, then streams the A_desc with LOCAL offset.
//
//   B loading (broadcast):
//     All PEs receive the same B_desc, B_col, B_val in sequence.
//=============================================================================

`include "defines.vh"

module pe_load_ctrl #(
    parameter N_PE       = `N_PE,
    parameter DESC_AW    = `MAX_DIM_BITS,       // A desc addr width
    parameter NNZ_AW     = 17,                  // A col/val addr width
    parameter B_DESC_AW  = `B_ROW_ADDR_BITS,
    parameter B_NNZ_AW   = `B_NNZ_ADDR_BITS,
    parameter B_NNZ_SLOT = `B_NNZ_SLOT
) (
    input  wire clk,
    input  wire rst_n,

    // === Start / Done ===
    input  wire                    start,          // 1-cycle pulse
    output wire                    a_done,         // A loading complete
    output wire                    b_done,         // B loading complete
    output wire                    all_done,       // both A and B complete

    // === Matrix dimensions ===
    input  wire [`MAX_DIM_BITS-1:0] M,            // A rows
    input  wire [`MAX_DIM_BITS-1:0] K,            // B rows
    input  wire [`MAX_DIM_BITS-1:0] N,

    // === A Global Buffer read ports ===
    output reg                     a_gbuf_desc_en,
    output reg  [DESC_AW-1:0]      a_gbuf_desc_addr,
    input  wire [63:0]             a_gbuf_desc_data,

    output reg                     a_gbuf_col_en,
    output reg  [NNZ_AW-1:0]       a_gbuf_col_addr,
    input  wire [15:0]             a_gbuf_col_data,

    output reg                     a_gbuf_val_en,
    output reg  [NNZ_AW-1:0]       a_gbuf_val_addr,
    input  wire [15:0]             a_gbuf_val_data,

    // === B Global Buffer read ports ===
    output reg                     b_gbuf_desc_en,
    output reg  [B_DESC_AW-1:0]    b_gbuf_desc_addr,
    input  wire [31:0]             b_gbuf_desc_data,

    output reg                     b_gbuf_col_en,
    output reg  [B_NNZ_AW-1:0]     b_gbuf_col_addr,
    input  wire [15:0]             b_gbuf_col_data,

    output reg                     b_gbuf_val_en,
    output reg  [B_NNZ_AW-1:0]     b_gbuf_val_addr,
    input  wire [15:0]             b_gbuf_val_data,

    // === PE Cluster A write ports (packed buses) ===
    output reg  [N_PE-1:0]                        pe_a_desc_we,
    output reg  [N_PE*`A_ROW_ADDR_BITS-1:0]      pe_a_desc_waddr,
    output reg  [N_PE*36-1:0]                     pe_a_desc_wdata,

    output reg  [N_PE-1:0]                        pe_a_val_we,
    output reg  [N_PE*`A_NNZ_ADDR_BITS-1:0]      pe_a_val_waddr,
    output reg  [N_PE*`DATA_WIDTH-1:0]            pe_a_val_wdata,

    output reg  [N_PE-1:0]                        pe_a_col_we,
    output reg  [N_PE*`A_NNZ_ADDR_BITS-1:0]      pe_a_col_waddr,
    output reg  [N_PE*`DATA_WIDTH-1:0]            pe_a_col_wdata,

    // === PE Cluster B write ports (broadcast) ===
    output reg                         pe_b_col_we,
    output reg  [`B_NNZ_ADDR_BITS-1:0] pe_b_col_waddr,
    output reg  [`DATA_WIDTH-1:0]      pe_b_col_wdata,
    output reg                         pe_b_val_we,
    output reg  [`B_NNZ_ADDR_BITS-1:0] pe_b_val_waddr,
    output reg  [`DATA_WIDTH-1:0]      pe_b_val_wdata,

    output reg                         pe_b_desc_we,
    output reg  [`B_ROW_ADDR_BITS-1:0] pe_b_desc_waddr,
    output reg  [31:0]                 pe_b_desc_wdata,

    // === Per-PE row counts (for the top-level FSM) ===
    output reg  [N_PE*16-1:0]        pe_row_counts
);

    //=========================================================================
    // Load-A FSM states
    //=========================================================================
    localparam LA_IDLE         = 3'd0;
    localparam LA_NEXT_ROW     = 3'd1;   // read A_desc[rid], determine target PE
    localparam LA_WAIT_DESC    = 3'd2;   // wait 1 cycle for registered A_desc read
    localparam LA_VAL_COL_LOOP = 3'd3;   // write A_val[t] + A_col[t] to PE[pid]
    localparam LA_DESC_STREAM  = 3'd4;   // stream a_desc to PE[pid]
    localparam LA_DONE         = 3'd5;

    // Load-B FSM states
    localparam LB_IDLE         = 3'd0;
    localparam LB_DESC         = 3'd1;
    localparam LB_DESC_WAIT    = 3'd2;
    localparam LB_COL          = 3'd3;
    localparam LB_COL_WAIT     = 3'd4;
    localparam LB_VAL          = 3'd5;
    localparam LB_VAL_WAIT     = 3'd6;
    localparam LB_DONE         = 3'd7;

    // Phase select: 0 = load A, 1 = load B
    reg phase;
    reg [2:0] la_state;
    reg [2:0] lb_state;

    // A-load tracking
    reg [`MAX_DIM_BITS-1:0]    a_rid;          // current global row index (0..M-1)
    reg [2:0]                  a_pid;          // target PE = rid % N_PE
    reg [`A_NNZ_ADDR_BITS-1:0] a_local_off [0:N_PE-1]; // next free local addr per PE
    reg [15:0]                 a_nnz;          // current row's nnz
    reg [8:0]                  a_crow;         // current row's global C row
    reg [16:0]                 a_global_off;   // global A offset (17-bit, up to 86016)
    reg [15:0]                 a_t;            // counter within current row
    reg [15:0]                 a_pe_rows [0:N_PE-1];  // rows per PE
    // B-load tracking
    reg [B_NNZ_AW-1:0]        b_idx;          // index for B col/val
    reg [`B_ROW_ADDR_BITS-1:0] b_k;            // row index for B desc
    wire [B_NNZ_AW-1:0]       b_nnz_total = B_NNZ_SLOT[B_NNZ_AW-1:0];

    // Done signal outputs
    assign a_done  = (la_state == LA_DONE);
    assign b_done  = phase && (lb_state == LB_DONE);
    assign all_done = a_done && b_done;

    //=========================================================================
    // Phase control
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 1'b0;
        end else if (la_state == LA_DONE) begin
            phase <= 1'b1;
        end
    end

    //=========================================================================
    // Load-A FSM
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            la_state      <= LA_IDLE;
            a_rid         <= 0;
            a_pid         <= 0;
            a_nnz         <= 0;
            a_crow        <= 0;
            a_global_off  <= 0;
            a_t           <= 0;
            a_gbuf_desc_en <= 1'b0;
            a_gbuf_col_en  <= 1'b0;
            a_gbuf_val_en  <= 1'b0;
            pe_a_desc_we    <= {N_PE{1'b0}};
            pe_a_val_we     <= {N_PE{1'b0}};
            pe_a_col_we     <= {N_PE{1'b0}};
            // Initialize local offsets and row counts
        end else begin
            // Defaults
            a_gbuf_desc_en <= 1'b0;
            a_gbuf_col_en  <= 1'b0;
            a_gbuf_val_en  <= 1'b0;
            pe_a_val_we    <= {N_PE{1'b0}};
            pe_a_col_we    <= {N_PE{1'b0}};
            pe_a_desc_we   <= {N_PE{1'b0}};

            case (la_state)
                LA_IDLE: begin
                    if (start) begin
                        la_state    <= LA_NEXT_ROW;
                        a_rid       <= 0;
                        a_t         <= 0;
                        a_local_off[0] <= 0;
                        a_local_off[1] <= 0;
                        a_local_off[2] <= 0;
                        a_pe_rows[0] <= 0;
                        a_pe_rows[1] <= 0;
                        a_pe_rows[2] <= 0;
                    end
                end

                LA_NEXT_ROW: begin
                    if (a_rid < M) begin
                        a_pid <= a_rid % N_PE;
                        // Issue read for A_desc[a_rid]
                        a_gbuf_desc_en   <= 1'b1;
                        a_gbuf_desc_addr <= a_rid[DESC_AW-1:0];
                        la_state <= LA_WAIT_DESC;
                    end else begin
                        // All rows processed → pack row_counts and done
                        la_state <= LA_DONE;
                    end
                end

                LA_WAIT_DESC: begin
                    // Registered read: a_gbuf_desc_data now valid
                    // Format matches Python a_desc(): (off<<19)|(nnz<<9)|crow
                    //   [35:19]=off (17b), [18:9]=nnz (10b), [8:0]=crow (9b)
                    a_global_off <= a_gbuf_desc_data[35:19];
                    a_nnz        <= a_gbuf_desc_data[18:9];
                    a_crow       <= a_gbuf_desc_data[8:0];
                    a_t          <= 0;
                    if (a_gbuf_desc_data[18:9] == 0) begin
                        // Zero-NNZ row → skip to desc streaming
                        la_state <= LA_NEXT_ROW;
                        a_rid    <= a_rid + 1;
                    end else begin
                        la_state <= LA_VAL_COL_LOOP;
                    end
                end

                LA_VAL_COL_LOOP: begin
                    // Read A_val[a_global_off + t] and A_col[a_global_off + t]
                    a_gbuf_val_en   <= 1'b1;
                    a_gbuf_val_addr <= a_global_off + a_t;
                    a_gbuf_col_en   <= 1'b1;
                    a_gbuf_col_addr <= a_global_off + a_t;

                    // Write to PE a_pid at local_addr = a_local_off[a_pid] + t
                    pe_a_val_we     <= (1 << a_pid);
                    pe_a_val_waddr  <= (a_local_off[a_pid] + a_t) << (a_pid * `A_NNZ_ADDR_BITS);
                    pe_a_val_wdata  <= a_gbuf_val_data << (a_pid * `DATA_WIDTH);
                    pe_a_col_we     <= (1 << a_pid);
                    pe_a_col_waddr  <= (a_local_off[a_pid] + a_t) << (a_pid * `A_NNZ_ADDR_BITS);
                    pe_a_col_wdata  <= a_gbuf_col_data << (a_pid * `DATA_WIDTH);

                    a_t <= a_t + 16'd1;
                    if (a_t + 16'd1 >= a_nnz) begin
                        // A_desc direct write: store per-PE descriptor with local offset
                        // Format: {3'b0, local_off[13:0], a_nnz[9:0], a_crow[8:0]}
                        pe_a_desc_we    <= (1 << a_pid);
                        pe_a_desc_waddr <= a_pe_rows[a_pid] << (a_pid * `A_ROW_ADDR_BITS);
                        pe_a_desc_wdata <= ({3'b0,
                            a_local_off[a_pid][13:0], a_nnz[9:0], a_crow[8:0]})
                            << (a_pid * 36);
                        // Update local offset and row count for this PE
                        a_local_off[a_pid] <= a_local_off[a_pid] + a_nnz;
                        a_pe_rows[a_pid]   <= a_pe_rows[a_pid] + 16'd1;
                        a_rid              <= a_rid + 1;
                        la_state           <= LA_NEXT_ROW;
                    end
                end

                LA_DONE: begin
                    // done signal driven by a_done output
                end

                default: la_state <= LA_IDLE;
            endcase
        end
    end

    //=========================================================================
    // Load-B FSM (broadcast B to all PEs)
    //=========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lb_state      <= LB_IDLE;
            b_k           <= 0;
            b_idx         <= 0;
            pe_b_desc_we   <= 1'b0;
            pe_b_col_we    <= 1'b0;
            pe_b_val_we    <= 1'b0;
            b_gbuf_desc_en <= 1'b0;
            b_gbuf_col_en  <= 1'b0;
            b_gbuf_val_en  <= 1'b0;
        end else if (phase) begin  // only active in phase B
            b_gbuf_desc_en <= 1'b0;
            b_gbuf_col_en  <= 1'b0;
            b_gbuf_val_en  <= 1'b0;
            pe_b_desc_we   <= 1'b0;
            pe_b_col_we    <= 1'b0;
            pe_b_val_we    <= 1'b0;

            case (lb_state)
                LB_IDLE: begin
                    b_k   <= 0;
                    b_idx <= 0;
                    lb_state <= LB_DESC;
                end

                LB_DESC: begin
                    b_gbuf_desc_en   <= 1'b1;
                    b_gbuf_desc_addr <= b_k;
                    lb_state <= LB_DESC_WAIT;
                end

                LB_DESC_WAIT: begin
                    pe_b_desc_we    <= 1'b1;
                    pe_b_desc_waddr <= b_k;
                    pe_b_desc_wdata <= b_gbuf_desc_data;
                    b_k <= b_k + 1;
                    if (b_k + 1 >= K) begin
                        b_idx    <= 0;       // reset for col phase
                        lb_state <= LB_COL;
                    end else begin
                        lb_state <= LB_DESC;
                    end
                end

                LB_COL: begin
                    b_gbuf_col_en   <= 1'b1;
                    b_gbuf_col_addr <= b_idx[B_NNZ_AW-1:0];
                    lb_state <= LB_COL_WAIT;
                end

                LB_COL_WAIT: begin
                    pe_b_col_we     <= 1'b1;
                    pe_b_col_waddr  <= b_idx;
                    pe_b_col_wdata  <= b_gbuf_col_data;
                    b_idx <= b_idx + 1;
                    if (b_idx + 1 >= B_NNZ_SLOT) begin
                        b_idx    <= 0;       // reset for val phase
                        lb_state <= LB_VAL;
                    end else begin
                        lb_state <= LB_COL;
                    end
                end

                LB_VAL: begin
                    b_gbuf_val_en   <= 1'b1;
                    b_gbuf_val_addr <= b_idx[B_NNZ_AW-1:0];
                    lb_state <= LB_VAL_WAIT;
                end

                LB_VAL_WAIT: begin
                    pe_b_val_we     <= 1'b1;
                    pe_b_val_waddr  <= b_idx;
                    pe_b_val_wdata  <= b_gbuf_val_data;
                    b_idx <= b_idx + 1;
                    if (b_idx + 1 >= B_NNZ_SLOT)
                        lb_state <= LB_DONE;
                    else
                        lb_state <= LB_VAL;
                end

                LB_DONE: begin
                    // done signal is handled by the top-level FSM
                end

                default: lb_state <= LB_IDLE;
            endcase
        end
    end

    // Pack row counts output
    always @(posedge clk) begin
        if (la_state == LA_DONE) begin
            pe_row_counts <= {a_pe_rows[2], a_pe_rows[1], a_pe_rows[0]};
        end
    end

endmodule
