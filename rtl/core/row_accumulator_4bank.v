//=============================================================================
// File     : row_accumulator_4bank.v
// Brief    : 4-bank row accumulator — replaces pe_serializer + pe_accumulator.
//
//   Receives 4-lane product bundles and accumulates C[row, col] using direct
//   col-id addressing. Four banks partition the column space by col_id[1:0].
//   Epoch/tag avoids acc_mem full-clear each row. Drain emits results in
//   strictly increasing col_id order.
//
//   FSM:
//     S_IDLE → S_ACCUM → S_WAIT_DRAIN → S_DRAIN → S_DONE
//                                                 → S_CLEAR_TAGS (epoch wrap)
//
//   Design constraint (enforced upstream via task_packer):
//     In any accepted bundle, all valid lanes have distinct col_id values.
//=============================================================================

module row_accumulator_4bank #(
    parameter OUT_COLS        = 512,   // max output columns (= MAX_N)
    parameter COL_W           = 9,     // ceil(log2(OUT_COLS))
    parameter PROD_W          = 16,    // product value width (signed)
    parameter ACC_W           = 32,    // accumulator width   (signed)
    parameter EPOCH_W         = 16,
    parameter BANK_FIFO_DEPTH = 8,
    parameter BANK_FIFO_LOG   = 3,     // log2(BANK_FIFO_DEPTH)
    parameter ROW_W           = 16
) (
    input  wire                   clk,
    input  wire                   rst_n,

    // Row control
    input  wire                   row_start,
    input  wire [ROW_W-1:0]       row_id_in,
    input  wire                   row_input_done,
    output reg                    busy,
    output reg                    row_done,

    // 4-lane product bundle (handshake)
    input  wire                   issue_valid,
    output wire                   issue_ready,
    input  wire [3:0]             lane_valid,
    input  wire [4*COL_W-1:0]     lane_col_id,     // {lane3,lane2,lane1,lane0}
    input  wire [4*PROD_W-1:0]    lane_product,    // {lane3,lane2,lane1,lane0}

    // Ordered sparse-row output
    output reg                    out_valid,
    input  wire                   out_ready,
    output reg  [ROW_W-1:0]       out_row_id,
    output reg  [COL_W-1:0]       out_col_id,
    output reg  [ACC_W-1:0]       out_value
);

    // =========================================================================
    // Derived parameters
    // =========================================================================
    localparam BANK_DEPTH  = OUT_COLS / 4;
    localparam BANK_ADDR_W = COL_W - 2;
    localparam BANK_LAST   = BANK_DEPTH - 1;    // last group_addr value

    // =========================================================================
    // FSM encoding
    // =========================================================================
    localparam S_IDLE       = 3'd0;
    localparam S_ACCUM      = 3'd1;
    localparam S_WAIT_DRAIN = 3'd2;
    localparam S_DRAIN      = 3'd3;
    localparam S_DONE       = 3'd4;
    localparam S_CLEAR_TAGS = 3'd5;

    reg [2:0] state;

    // =========================================================================
    // Row tracking
    // =========================================================================
    reg [ROW_W-1:0]   cur_row_id;
    reg [EPOCH_W-1:0] row_epoch;
    reg               input_done_latch;
    reg               clr_triggered;

    // =========================================================================
    // Drain counters
    // =========================================================================
    reg [BANK_ADDR_W-1:0] group_addr;
    reg [1:0]             bank_sel;
    reg                   drain_emit;    // 1 while holding out_valid for handshake

    // =========================================================================
    // Bundle routing — bank_id = col_id[1:0]
    // =========================================================================
    wire [1:0] bid0 = lane_col_id[0*COL_W +: 2];
    wire [1:0] bid1 = lane_col_id[1*COL_W +: 2];
    wire [1:0] bid2 = lane_col_id[2*COL_W +: 2];
    wire [1:0] bid3 = lane_col_id[3*COL_W +: 2];

    // Number of valid lanes mapping to each bank
    wire [2:0] mc0 = {2'b0, lane_valid[0] & (bid0==2'd0)}
                   + {2'b0, lane_valid[1] & (bid1==2'd0)}
                   + {2'b0, lane_valid[2] & (bid2==2'd0)}
                   + {2'b0, lane_valid[3] & (bid3==2'd0)};
    wire [2:0] mc1 = {2'b0, lane_valid[0] & (bid0==2'd1)}
                   + {2'b0, lane_valid[1] & (bid1==2'd1)}
                   + {2'b0, lane_valid[2] & (bid2==2'd1)}
                   + {2'b0, lane_valid[3] & (bid3==2'd1)};
    wire [2:0] mc2 = {2'b0, lane_valid[0] & (bid0==2'd2)}
                   + {2'b0, lane_valid[1] & (bid1==2'd2)}
                   + {2'b0, lane_valid[2] & (bid2==2'd2)}
                   + {2'b0, lane_valid[3] & (bid3==2'd2)};
    wire [2:0] mc3 = {2'b0, lane_valid[0] & (bid0==2'd3)}
                   + {2'b0, lane_valid[1] & (bid1==2'd3)}
                   + {2'b0, lane_valid[2] & (bid2==2'd3)}
                   + {2'b0, lane_valid[3] & (bid3==2'd3)};

    // All banks receive the same addr/data flat; wr_valid gates each port
    wire [4*BANK_ADDR_W-1:0] wr_addr_flat = {
        lane_col_id[3*COL_W+2 +: BANK_ADDR_W],
        lane_col_id[2*COL_W+2 +: BANK_ADDR_W],
        lane_col_id[1*COL_W+2 +: BANK_ADDR_W],
        lane_col_id[0*COL_W+2 +: BANK_ADDR_W]
    };
    wire [4*PROD_W-1:0] wr_data_flat = lane_product;

    wire do_enqueue = issue_valid & issue_ready;

    // Per-bank wr_valid vectors
    wire [3:0] bwv0 = {do_enqueue & lane_valid[3] & (bid3==2'd0),
                       do_enqueue & lane_valid[2] & (bid2==2'd0),
                       do_enqueue & lane_valid[1] & (bid1==2'd0),
                       do_enqueue & lane_valid[0] & (bid0==2'd0)};
    wire [3:0] bwv1 = {do_enqueue & lane_valid[3] & (bid3==2'd1),
                       do_enqueue & lane_valid[2] & (bid2==2'd1),
                       do_enqueue & lane_valid[1] & (bid1==2'd1),
                       do_enqueue & lane_valid[0] & (bid0==2'd1)};
    wire [3:0] bwv2 = {do_enqueue & lane_valid[3] & (bid3==2'd2),
                       do_enqueue & lane_valid[2] & (bid2==2'd2),
                       do_enqueue & lane_valid[1] & (bid1==2'd2),
                       do_enqueue & lane_valid[0] & (bid0==2'd2)};
    wire [3:0] bwv3 = {do_enqueue & lane_valid[3] & (bid3==2'd3),
                       do_enqueue & lane_valid[2] & (bid2==2'd3),
                       do_enqueue & lane_valid[1] & (bid1==2'd3),
                       do_enqueue & lane_valid[0] & (bid0==2'd3)};

    // =========================================================================
    // Bank signals
    // =========================================================================
    wire [3:0]         free_b0, free_b1, free_b2, free_b3;
    wire               rmw_b0,  rmw_b1,  rmw_b2,  rmw_b3;
    wire               emp_b0,  emp_b1,  emp_b2,  emp_b3;
    wire               clr_b0,  clr_b1,  clr_b2,  clr_b3;
    wire [EPOCH_W-1:0] dtag_b0, dtag_b1, dtag_b2, dtag_b3;
    wire [ACC_W-1:0]   dacc_b0, dacc_b1, dacc_b2, dacc_b3;

    wire tag_clear_pulse = (state == S_CLEAR_TAGS) && !clr_triggered;
    wire [BANK_ADDR_W-1:0] drain_rd_addr = group_addr;

    accum_bank #(
        .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W),
        .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W),
        .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG)
    ) u_bank0 (
        .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch),
        .wr_valid(bwv0), .wr_addr_flat(wr_addr_flat), .wr_data_flat(wr_data_flat),
        .free_count(free_b0), .rmw_busy(rmw_b0), .fifo_empty(emp_b0),
        .tag_clear_en(tag_clear_pulse), .tag_clear_busy(clr_b0),
        .drain_rd_addr(drain_rd_addr), .drain_tag(dtag_b0), .drain_acc(dacc_b0)
    );
    accum_bank #(
        .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W),
        .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W),
        .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG)
    ) u_bank1 (
        .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch),
        .wr_valid(bwv1), .wr_addr_flat(wr_addr_flat), .wr_data_flat(wr_data_flat),
        .free_count(free_b1), .rmw_busy(rmw_b1), .fifo_empty(emp_b1),
        .tag_clear_en(tag_clear_pulse), .tag_clear_busy(clr_b1),
        .drain_rd_addr(drain_rd_addr), .drain_tag(dtag_b1), .drain_acc(dacc_b1)
    );
    accum_bank #(
        .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W),
        .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W),
        .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG)
    ) u_bank2 (
        .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch),
        .wr_valid(bwv2), .wr_addr_flat(wr_addr_flat), .wr_data_flat(wr_data_flat),
        .free_count(free_b2), .rmw_busy(rmw_b2), .fifo_empty(emp_b2),
        .tag_clear_en(tag_clear_pulse), .tag_clear_busy(clr_b2),
        .drain_rd_addr(drain_rd_addr), .drain_tag(dtag_b2), .drain_acc(dacc_b2)
    );
    accum_bank #(
        .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W),
        .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W),
        .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG)
    ) u_bank3 (
        .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch),
        .wr_valid(bwv3), .wr_addr_flat(wr_addr_flat), .wr_data_flat(wr_data_flat),
        .free_count(free_b3), .rmw_busy(rmw_b3), .fifo_empty(emp_b3),
        .tag_clear_en(tag_clear_pulse), .tag_clear_busy(clr_b3),
        .drain_rd_addr(drain_rd_addr), .drain_tag(dtag_b3), .drain_acc(dacc_b3)
    );

    // =========================================================================
    // issue_ready — all banks must have enough free slots for this bundle
    // =========================================================================
    assign issue_ready = (state == S_ACCUM) && !input_done_latch
                       && (free_b0 >= {1'b0, mc0})
                       && (free_b1 >= {1'b0, mc1})
                       && (free_b2 >= {1'b0, mc2})
                       && (free_b3 >= {1'b0, mc3});

    wire all_fifos_empty = emp_b0 & emp_b1 & emp_b2 & emp_b3;
    wire all_rmw_done    = ~rmw_b0 & ~rmw_b1 & ~rmw_b2 & ~rmw_b3;
    wire all_clr_done    = ~clr_b0 & ~clr_b1 & ~clr_b2 & ~clr_b3;

    // =========================================================================
    // Drain — mux current bank's tag/acc outputs
    // =========================================================================
    wire [EPOCH_W-1:0] sel_tag = (bank_sel==2'd0) ? dtag_b0 :
                                 (bank_sel==2'd1) ? dtag_b1 :
                                 (bank_sel==2'd2) ? dtag_b2 : dtag_b3;
    wire [ACC_W-1:0]   sel_acc = (bank_sel==2'd0) ? dacc_b0 :
                                 (bank_sel==2'd1) ? dacc_b1 :
                                 (bank_sel==2'd2) ? dacc_b2 : dacc_b3;

    wire cur_valid_entry = (sel_tag == row_epoch) && (sel_acc != {ACC_W{1'b0}});
    wire last_cell = (bank_sel == 2'd3) &&
                     (group_addr == BANK_LAST[BANK_ADDR_W-1:0]);

    // =========================================================================
    // Sequential FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            cur_row_id       <= {ROW_W{1'b0}};
            row_epoch        <= {{(EPOCH_W-1){1'b0}}, 1'b1};  // start at 1
            input_done_latch <= 1'b0;
            clr_triggered    <= 1'b0;
            group_addr       <= {BANK_ADDR_W{1'b0}};
            bank_sel         <= 2'd0;
            drain_emit       <= 1'b0;
            busy             <= 1'b0;
            row_done         <= 1'b0;
            out_valid        <= 1'b0;
            out_row_id       <= {ROW_W{1'b0}};
            out_col_id       <= {COL_W{1'b0}};
            out_value        <= {ACC_W{1'b0}};
        end else begin
            row_done  <= 1'b0;
            out_valid <= 1'b0;

            case (state)
                // --------------------------------------------------------
                S_IDLE: begin
                    busy             <= 1'b0;
                    input_done_latch <= 1'b0;
                    if (row_start) begin
                        cur_row_id <= row_id_in;
                        group_addr <= {BANK_ADDR_W{1'b0}};
                        bank_sel   <= 2'd0;
                        drain_emit <= 1'b0;
                        busy       <= 1'b1;
                        state      <= S_ACCUM;
                    end
                end

                // --------------------------------------------------------
                S_ACCUM: begin
                    if (row_input_done)
                        input_done_latch <= 1'b1;
                    if (input_done_latch || row_input_done)
                        state <= S_WAIT_DRAIN;
                end

                // --------------------------------------------------------
                S_WAIT_DRAIN: begin
                    if (all_fifos_empty && all_rmw_done)
                        state <= S_DRAIN;
                end

                // --------------------------------------------------------
                S_DRAIN: begin
                    if (drain_emit) begin
                        // Holding output for downstream handshake
                        out_valid  <= 1'b1;
                        out_row_id <= cur_row_id;
                        out_col_id <= {group_addr, bank_sel};
                        out_value  <= sel_acc;

                        if (out_ready) begin
                            out_valid  <= 1'b0;
                            drain_emit <= 1'b0;
                            if (last_cell) begin
                                state <= S_DONE;
                            end else if (bank_sel == 2'd3) begin
                                bank_sel   <= 2'd0;
                                group_addr <= group_addr + {{(BANK_ADDR_W-1){1'b0}}, 1'b1};
                            end else begin
                                bank_sel <= bank_sel + 2'd1;
                            end
                        end

                    end else begin
                        // Scan: check current (group_addr, bank_sel)
                        if (cur_valid_entry) begin
                            drain_emit <= 1'b1;
                            out_valid  <= 1'b1;
                            out_row_id <= cur_row_id;
                            out_col_id <= {group_addr, bank_sel};
                            out_value  <= sel_acc;
                        end else begin
                            // Not valid: skip to next cell
                            if (last_cell) begin
                                state <= S_DONE;
                            end else if (bank_sel == 2'd3) begin
                                bank_sel   <= 2'd0;
                                group_addr <= group_addr + {{(BANK_ADDR_W-1){1'b0}}, 1'b1};
                            end else begin
                                bank_sel <= bank_sel + 2'd1;
                            end
                        end
                    end
                end

                // --------------------------------------------------------
                S_DONE: begin
                    row_done <= 1'b1;
                    if (row_epoch == {EPOCH_W{1'b1}}) begin
                        // Next epoch would be 0 (= initial tag value) → clear tags
                        clr_triggered <= 1'b0;
                        state         <= S_CLEAR_TAGS;
                    end else begin
                        row_epoch        <= row_epoch + {{(EPOCH_W-1){1'b0}}, 1'b1};
                        input_done_latch <= 1'b0;
                        state            <= S_IDLE;
                    end
                end

                // --------------------------------------------------------
                S_CLEAR_TAGS: begin
                    if (!clr_triggered)
                        clr_triggered <= 1'b1;
                    // tag_clear_pulse fires for exactly one cycle (!clr_triggered)
                    if (clr_triggered && all_clr_done) begin
                        row_epoch        <= {{(EPOCH_W-1){1'b0}}, 1'b1};
                        input_done_latch <= 1'b0;
                        clr_triggered    <= 1'b0;
                        state            <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Simulation-only assertions
    // =========================================================================
`ifdef SIMULATION
    integer ai, bi;
    always @(posedge clk) begin
        if (do_enqueue) begin
            for (ai = 0; ai < 4; ai = ai + 1)
                for (bi = ai+1; bi < 4; bi = bi + 1)
                    if (lane_valid[ai] && lane_valid[bi] &&
                        lane_col_id[ai*COL_W +:COL_W] == lane_col_id[bi*COL_W +:COL_W])
                        $error("RA4B: duplicate col_id in bundle lanes %0d and %0d", ai, bi);
        end
    end

    reg [COL_W-1:0] prev_col;
    reg             prev_col_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_col       <= {COL_W{1'b0}};
            prev_col_valid <= 1'b0;
        end else begin
            if (row_done)
                prev_col_valid <= 1'b0;
            if (out_valid && out_ready) begin
                if (prev_col_valid && out_col_id <= prev_col)
                    $error("RA4B: out_col_id %0d not > prev %0d", out_col_id, prev_col);
                prev_col       <= out_col_id;
                prev_col_valid <= 1'b1;
            end
        end
    end
`endif

endmodule
