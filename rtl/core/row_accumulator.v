//=============================================================================
// File     : row_accumulator.v
// Brief    : N_BANK-bank row accumulator with N_BANK-wide drain output.
//
//   Receives N_BANK-lane product bundles and accumulates C[row, col].
//   N_BANK banks partition the column space by col_id[BIDX_W-1:0].
//   BANK_DEPTH = OUT_COLS/N_BANK, BANK_ADDR_W = COL_W-BIDX_W.
//
//   Parameterized over N_BANK (= number of MAC lanes feeding it).  At N_BANK=32
//   one bank holds 512/32=16 columns; a contiguous SpGEMM group of 32 columns
//   maps to 32 distinct banks (no same-bank collision), so the 4-write-port
//   scatter only multi-cycles on the elementwise / carry path.
//=============================================================================

module row_accumulator #(
    parameter OUT_COLS        = 512,
    parameter COL_W           = 9,
    parameter N_BANK          = 16,
    parameter BIDX_W          = $clog2(N_BANK),   // bank-index bits = log2(N_BANK)
    parameter N_LANE          = N_BANK,            // # product lanes IN (>= N_BANK; default = N_BANK)
    parameter WR_PORTS        = 4,                 // same-bank lanes absorbed per cycle (2 or 4)
    parameter PROD_W          = 16,
    parameter ACC_W           = 32,
    parameter EPOCH_W         = 16,
    parameter BANK_FIFO_DEPTH = 8,
    parameter BANK_FIFO_LOG   = 3,
    parameter ROW_W           = 8
) (
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    row_start,
    input  wire [ROW_W-1:0]        row_id_in,
    input  wire [COL_W:0]          drain_cols,
    input  wire                    row_input_done,
    output reg                     busy,
    output reg                     row_done,

    input  wire                    issue_valid,
    output wire                    issue_ready,
    input  wire [N_LANE-1:0]       lane_valid,
    input  wire [N_LANE*COL_W-1:0] lane_col_id,
    input  wire [N_LANE*PROD_W-1:0] lane_product,

    output wire [N_BANK-1:0]       drain_valid,
    output wire [COL_W-BIDX_W-1:0] drain_gaddr,
    output wire [ROW_W-1:0]        drain_row_id,
    output wire [N_BANK*ACC_W-1:0] drain_values,
    // High for every S_DRAIN beat (one per column-group, incl. all-zero groups),
    // so the C bank can be fully written (zero-filled) without a separate clear.
    output wire                    drain_active
);

    localparam BANK_DEPTH  = OUT_COLS / N_BANK;
    localparam BANK_ADDR_W = COL_W - BIDX_W;
    localparam BANK_LAST   = BANK_DEPTH - 1;

    localparam S_IDLE       = 3'd0;
    localparam S_ACCUM      = 3'd1;
    localparam S_WAIT_DRAIN = 3'd2;
    localparam S_DRAIN      = 3'd3;
    localparam S_DONE       = 3'd4;
    localparam S_CLEAR_TAGS = 3'd5;

    reg [2:0] state;

    reg [ROW_W-1:0]   cur_row_id;
    reg [EPOCH_W-1:0] row_epoch;
    reg               input_done_latch;
    reg               clr_triggered;

    // Latches a row_start that arrives while the FSM is still busy with the
    // post-reset S_CLEAR_TAGS walk, so the start is serviced once we reach
    // S_IDLE instead of being lost (makes start-immediately-after-reset safe).
    reg               start_pending;
    reg [ROW_W-1:0]   pending_row_id;

    reg [BANK_ADDR_W-1:0] group_addr;

    //=========================================================================
    // Per-lane fields (N_LANE lanes IN): bank = col%N_BANK = col[BIDX_W-1:0],
    // in-bank addr = col[COL_W-1:BIDX_W].  N_LANE may exceed N_BANK -> several
    // lanes share a bank and the scatter spreads them over WR_PORTS/cycle.
    //=========================================================================
    wire [BIDX_W-1:0]      lbid  [0:N_LANE-1];
    wire [BANK_ADDR_W-1:0] laddr [0:N_LANE-1];
    wire [PROD_W-1:0]      lprod [0:N_LANE-1];
    genvar gi;
    generate for (gi = 0; gi < N_LANE; gi = gi + 1) begin : g_lane
        assign lbid [gi] = lane_col_id[gi*COL_W        +: BIDX_W];
        assign laddr[gi] = lane_col_id[gi*COL_W+BIDX_W +: BANK_ADDR_W];
        assign lprod[gi] = lane_product[gi*PROD_W +: PROD_W];
    end endgenerate

    // Per-bank free_count, indexable (assigned from the bank instances below).
    wire [BANK_FIFO_LOG:0] free_arr [0:N_BANK-1];

    //=========================================================================
    // Input scatter — drive AT MOST ONE lane per bank per cycle into the
    // single-write banks.  Groups whose lanes all land on distinct banks (the
    // SpGEMM rotation path) finish in one cycle; a group with same-bank
    // collisions (the Gen2 carry path) is absorbed over several cycles while
    // the upstream holds issue_valid (issue_ready stays low until done).
    // done_mask records which lanes of the current group are already enqueued.
    //=========================================================================
    reg [N_LANE-1:0] done_mask;

    // `accepting` does NOT depend on issue_valid: the upstream (pe_top) gates
    // its product-FIFO read on issue_ready and only then presents the data one
    // cycle later, so issue_ready must be assertable BEFORE issue_valid (like
    // AXI ready-before-valid).  `active` adds issue_valid for the enqueue path.
    wire accepting = (state == S_ACCUM) && !input_done_latch;
    wire active    = accepting && issue_valid;

    reg  [N_LANE-1:0]     eligible;
    reg  [5:0]            lt [0:N_LANE-1];  // # of lower-index eligible same-bank lanes
    reg  [N_LANE-1:0]     win0;       // lowest    eligible lane targeting its bank
    reg  [N_LANE-1:0]     win1;       // 2nd-lowest
    reg  [N_LANE-1:0]     win2;       // 3rd-lowest
    reg  [N_LANE-1:0]     win3;       // 4th-lowest
    reg  [N_LANE-1:0]     lane_enq;   // lane enqueued this cycle (gated by free)
    reg  [N_BANK-1:0]     bank_wr_en0,  bank_wr_en1,  bank_wr_en2,  bank_wr_en3;
    reg  [BANK_ADDR_W-1:0] bank_wr_addr0 [0:N_BANK-1];
    reg  [BANK_ADDR_W-1:0] bank_wr_addr1 [0:N_BANK-1];
    reg  [BANK_ADDR_W-1:0] bank_wr_addr2 [0:N_BANK-1];
    reg  [BANK_ADDR_W-1:0] bank_wr_addr3 [0:N_BANK-1];
    reg  [PROD_W-1:0]      bank_wr_data0 [0:N_BANK-1];
    reg  [PROD_W-1:0]      bank_wr_data1 [0:N_BANK-1];
    reg  [PROD_W-1:0]      bank_wr_data2 [0:N_BANK-1];
    reg  [PROD_W-1:0]      bank_wr_data3 [0:N_BANK-1];

    integer ji, ki, ni;
    always @(*) begin
        for (ji = 0; ji < N_LANE; ji = ji + 1)
            eligible[ji] = active && lane_valid[ji] && !done_mask[ji];

        // lt[j] = number of lower-index eligible lanes targeting the same bank;
        // lt==k -> this lane is the bank's (k+1)-th lane -> write port k.
        for (ji = 0; ji < N_LANE; ji = ji + 1) begin
            lt[ji] = 6'd0;
            for (ki = 0; ki < N_LANE; ki = ki + 1)
                if ((ki < ji) && eligible[ki] && (lbid[ki] == lbid[ji]))
                    lt[ji] = lt[ji] + 6'd1;
        end
        for (ji = 0; ji < N_LANE; ji = ji + 1) begin
            win0[ji] = eligible[ji] && (lt[ji] == 6'd0);
            win1[ji] = eligible[ji] && (lt[ji] == 6'd1);
            win2[ji] = eligible[ji] && (lt[ji] == 6'd2);
            win3[ji] = eligible[ji] && (lt[ji] == 6'd3);
        end

        // port k lane needs >=(k+1) free slots (shares the cycle with ports 0..k-1).
        // Ports 2,3 are compiled out when WR_PORTS<3/<4 (those ranks then wait a cycle).
        for (ji = 0; ji < N_LANE; ji = ji + 1)
            lane_enq[ji] = (win0[ji] && (free_arr[lbid[ji]] >= 1)) ||
                           (win1[ji] && (free_arr[lbid[ji]] >= 2)) ||
                           ((WR_PORTS >= 3) && win2[ji] && (free_arr[lbid[ji]] >= 3)) ||
                           ((WR_PORTS >= 4) && win3[ji] && (free_arr[lbid[ji]] >= 4));

        // per-bank four write ports (<=1 lane per win-rank per bank)
        for (ni = 0; ni < N_BANK; ni = ni + 1) begin
            bank_wr_en0[ni]   = 1'b0;
            bank_wr_addr0[ni] = {BANK_ADDR_W{1'b0}};
            bank_wr_data0[ni] = {PROD_W{1'b0}};
            bank_wr_en1[ni]   = 1'b0;
            bank_wr_addr1[ni] = {BANK_ADDR_W{1'b0}};
            bank_wr_data1[ni] = {PROD_W{1'b0}};
            bank_wr_en2[ni]   = 1'b0;
            bank_wr_addr2[ni] = {BANK_ADDR_W{1'b0}};
            bank_wr_data2[ni] = {PROD_W{1'b0}};
            bank_wr_en3[ni]   = 1'b0;
            bank_wr_addr3[ni] = {BANK_ADDR_W{1'b0}};
            bank_wr_data3[ni] = {PROD_W{1'b0}};
            for (ji = 0; ji < N_LANE; ji = ji + 1) begin
                if (win0[ji] && (lbid[ji] == ni[BIDX_W-1:0]) && (free_arr[ni] >= 1)) begin
                    bank_wr_en0[ni]   = 1'b1;
                    bank_wr_addr0[ni] = laddr[ji];
                    bank_wr_data0[ni] = lprod[ji];
                end
                if (win1[ji] && (lbid[ji] == ni[BIDX_W-1:0]) && (free_arr[ni] >= 2)) begin
                    bank_wr_en1[ni]   = 1'b1;
                    bank_wr_addr1[ni] = laddr[ji];
                    bank_wr_data1[ni] = lprod[ji];
                end
                if ((WR_PORTS >= 3) && win2[ji] && (lbid[ji] == ni[BIDX_W-1:0]) && (free_arr[ni] >= 3)) begin
                    bank_wr_en2[ni]   = 1'b1;
                    bank_wr_addr2[ni] = laddr[ji];
                    bank_wr_data2[ni] = lprod[ji];
                end
                if ((WR_PORTS >= 4) && win3[ji] && (lbid[ji] == ni[BIDX_W-1:0]) && (free_arr[ni] >= 4)) begin
                    bank_wr_en3[ni]   = 1'b1;
                    bank_wr_addr3[ni] = laddr[ji];
                    bank_wr_data3[ni] = lprod[ji];
                end
            end
        end
    end

    wire [N_LANE-1:0] next_done       = done_mask | lane_enq;
    // Only a presented group contributes "remaining" lanes; with no group
    // (issue_valid=0) remaining is 0 so issue_ready stays high (ready for next).
    wire [N_LANE-1:0] grp_lanes       = active ? lane_valid : {N_LANE{1'b0}};
    wire [N_LANE-1:0] remaining_after = grp_lanes & ~next_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            done_mask <= {N_LANE{1'b0}};
        else if (active && !issue_ready)   // partial: remember enqueued lanes
            done_mask <= next_done;
        else                               // group consumed, or no active group
            done_mask <= {N_LANE{1'b0}};
    end

    //=========================================================================
    // Bank instances
    //=========================================================================
    wire [N_BANK-1:0]  rmw_vec, emp_vec, clr_vec;
    wire [EPOCH_W-1:0] dtag_arr [0:N_BANK-1];
    wire [ACC_W-1:0]   dacc_arr [0:N_BANK-1];

    wire tag_clear_pulse = (state == S_CLEAR_TAGS) && !clr_triggered;
    wire [BANK_ADDR_W-1:0] drain_rd_addr = group_addr;

    generate for (gi = 0; gi < N_BANK; gi = gi + 1) begin : g_bank
        accum_bank_16 #(
            .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W),
            .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W),
            .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG)
        ) u_bank (
            .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch),
            .wr_en0(bank_wr_en0[gi]), .wr_addr0(bank_wr_addr0[gi]), .wr_data0(bank_wr_data0[gi]),
            .wr_en1(bank_wr_en1[gi]), .wr_addr1(bank_wr_addr1[gi]), .wr_data1(bank_wr_data1[gi]),
            .wr_en2(bank_wr_en2[gi]), .wr_addr2(bank_wr_addr2[gi]), .wr_data2(bank_wr_data2[gi]),
            .wr_en3(bank_wr_en3[gi]), .wr_addr3(bank_wr_addr3[gi]), .wr_data3(bank_wr_data3[gi]),
            .free_count(free_arr[gi]), .rmw_busy(rmw_vec[gi]), .fifo_empty(emp_vec[gi]),
            .tag_clear_en(tag_clear_pulse), .tag_clear_busy(clr_vec[gi]),
            .drain_rd_addr(drain_rd_addr), .drain_tag(dtag_arr[gi]), .drain_acc(dacc_arr[gi])
        );
    end endgenerate

    //=========================================================================
    // issue_ready — high on the cycle that enqueues the LAST remaining lane(s)
    // of the current group, so the upstream advances to the next group.
    //=========================================================================
    assign issue_ready = accepting && (remaining_after == {N_LANE{1'b0}});

    wire all_fifos_empty = &emp_vec;
    wire all_rmw_done    = ~|rmw_vec;
    wire all_clr_done    = ~|clr_vec;

    //=========================================================================
    // Drain output — N_BANK-wide, one group per cycle
    //=========================================================================
    wire [N_BANK-1:0] grp_v;
    generate for (gi = 0; gi < N_BANK; gi = gi + 1) begin : g_drain
        assign grp_v[gi] = (dtag_arr[gi] == row_epoch) && (dacc_arr[gi] != {ACC_W{1'b0}});
        assign drain_values[gi*ACC_W +: ACC_W] = dacc_arr[gi];
    end endgenerate

    assign drain_valid  = (state == S_DRAIN) ? grp_v : {N_BANK{1'b0}};
    assign drain_active = (state == S_DRAIN);
    assign drain_gaddr  = group_addr;
    assign drain_row_id = cur_row_id;

    wire [COL_W:0]         drain_cols_m1   = drain_cols - 1'b1;
    wire [BANK_ADDR_W-1:0] last_group_addr = drain_cols_m1[COL_W-1:BIDX_W];
    wire last_group = (group_addr == last_group_addr);

    //=========================================================================
    // FSM
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset INTO S_CLEAR_TAGS (not S_IDLE) so every reset scrubs the
            // tag arrays via the single-write clr walk.  This replaces the old
            // parallel tag_mem reset inside accum_bank_16 (which forced those
            // arrays into registers); the walk keeps back-to-back operations
            // free of cross-operation tag/epoch contamination while letting
            // tag_mem map to LUTRAM.  The PE's own reset/load sequence is longer
            // than the BANK_DEPTH-cycle walk, so first-row issue is never stalled.
            state            <= S_CLEAR_TAGS;
            cur_row_id       <= {ROW_W{1'b0}};
            row_epoch        <= {{(EPOCH_W-1){1'b0}}, 1'b1};
            input_done_latch <= 1'b0;
            clr_triggered    <= 1'b0;
            group_addr       <= {BANK_ADDR_W{1'b0}};
            busy             <= 1'b1;   // reset enters S_CLEAR_TAGS: stay BUSY until
                                        // the tag scrub finishes (->S_IDLE clears
                                        // it).  Else a fast producer (wide elem) races
                                        // multiple row_starts into the 1-deep start_pending
                                        // during the scrub -> rows merge/drop.
            row_done         <= 1'b0;
            start_pending    <= 1'b0;
            pending_row_id   <= {ROW_W{1'b0}};
        end else begin
            row_done <= 1'b0;

            // Capture a row_start that cannot be serviced yet (FSM not in
            // S_IDLE, e.g. still finishing the post-reset tag-clear walk).
            if (row_start && state != S_IDLE) begin
                start_pending  <= 1'b1;
                pending_row_id <= row_id_in;
            end

            case (state)
                S_IDLE: begin
                    busy             <= 1'b0;
                    input_done_latch <= 1'b0;
                    if (row_start || start_pending) begin
                        cur_row_id    <= row_start ? row_id_in : pending_row_id;
                        group_addr    <= {BANK_ADDR_W{1'b0}};
                        busy          <= 1'b1;
                        start_pending <= 1'b0;
                        state         <= S_ACCUM;
                    end
                end

                S_ACCUM: begin
                    if (row_input_done)
                        input_done_latch <= 1'b1;
                    if (input_done_latch || row_input_done)
                        state <= S_WAIT_DRAIN;
                end

                S_WAIT_DRAIN: begin
                    if (all_fifos_empty && all_rmw_done)
                        state <= S_DRAIN;
                end

                S_DRAIN: begin
                    if (last_group)
                        state <= S_DONE;
                    else
                        group_addr <= group_addr + {{(BANK_ADDR_W-1){1'b0}}, 1'b1};
                end

                S_DONE: begin
                    row_done <= 1'b1;
                    if (row_epoch == {EPOCH_W{1'b1}}) begin
                        clr_triggered <= 1'b0;
                        state         <= S_CLEAR_TAGS;
                    end else begin
                        row_epoch        <= row_epoch + {{(EPOCH_W-1){1'b0}}, 1'b1};
                        input_done_latch <= 1'b0;
                        state            <= S_IDLE;
                    end
                end

                S_CLEAR_TAGS: begin
                    if (!clr_triggered) clr_triggered <= 1'b1;
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

endmodule
