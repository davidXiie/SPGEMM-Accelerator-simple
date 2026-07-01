//=============================================================================
// File     : row_accumulator_16bank.v
// Brief    : 16-bank row accumulator with 16-wide drain output.
//
//   Receives 16-lane product bundles and accumulates C[row, col].
//   Sixteen banks partition the column space by col_id[3:0].
//   BANK_DEPTH = OUT_COLS/16, BANK_ADDR_W = COL_W-4.
//=============================================================================

module row_accumulator_16bank #(
    parameter OUT_COLS        = 512,
    parameter COL_W           = 9,
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
    input  wire [15:0]             lane_valid,
    input  wire [16*COL_W-1:0]    lane_col_id,
    input  wire [16*PROD_W-1:0]   lane_product,

    output wire [15:0]             drain_valid,
    output wire [COL_W-5:0]       drain_gaddr,
    output wire [ROW_W-1:0]       drain_row_id,
    output wire [16*ACC_W-1:0]    drain_values,
    // High for every S_DRAIN beat (one per column-group, incl. all-zero groups),
    // so the C bank can be fully written (zero-filled) without a separate clear.
    output wire                    drain_active
);

    localparam BANK_DEPTH  = OUT_COLS / 16;
    localparam BANK_ADDR_W = COL_W - 4;
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
    // Per-lane fields: bank = col[3:0], in-bank addr = col[COL_W-1:4]
    //=========================================================================
    wire [3:0]             lbid  [0:15];
    wire [BANK_ADDR_W-1:0] laddr [0:15];
    wire [PROD_W-1:0]      lprod [0:15];
    genvar gi;
    generate for (gi = 0; gi < 16; gi = gi + 1) begin : g_lane
        assign lbid [gi] = lane_col_id[gi*COL_W   +: 4];
        assign laddr[gi] = lane_col_id[gi*COL_W+4 +: BANK_ADDR_W];
        assign lprod[gi] = lane_product[gi*PROD_W +: PROD_W];
    end endgenerate

    // Per-bank free_count, indexable (assigned from the bank instances below).
    wire [BANK_FIFO_LOG:0] free_arr [0:15];

    //=========================================================================
    // Input scatter — drive AT MOST ONE lane per bank per cycle into the
    // single-write banks.  Groups whose lanes all land on distinct banks (the
    // SpGEMM rotation path) finish in one cycle; a group with same-bank
    // collisions (the Gen2 carry path) is absorbed over several cycles while
    // the upstream holds issue_valid (issue_ready stays low until done).
    // done_mask records which lanes of the current group are already enqueued.
    //=========================================================================
    reg [15:0] done_mask;

    // `accepting` does NOT depend on issue_valid: the upstream (pe_top) gates
    // its product-FIFO read on issue_ready and only then presents the data one
    // cycle later, so issue_ready must be assertable BEFORE issue_valid (like
    // AXI ready-before-valid).  `active` adds issue_valid for the enqueue path.
    wire accepting = (state == S_ACCUM) && !input_done_latch;
    wire active    = accepting && issue_valid;

    reg  [15:0]            eligible;
    reg  [4:0]             lt [0:15];  // # of lower-index eligible same-bank lanes
    reg  [15:0]            win0;       // lowest    eligible lane targeting its bank
    reg  [15:0]            win1;       // 2nd-lowest
    reg  [15:0]            win2;       // 3rd-lowest
    reg  [15:0]            win3;       // 4th-lowest
    reg  [15:0]            lane_enq;   // lane enqueued this cycle (gated by free)
    reg  [15:0]            bank_wr_en0,  bank_wr_en1,  bank_wr_en2,  bank_wr_en3;
    reg  [BANK_ADDR_W-1:0] bank_wr_addr0 [0:15];
    reg  [BANK_ADDR_W-1:0] bank_wr_addr1 [0:15];
    reg  [BANK_ADDR_W-1:0] bank_wr_addr2 [0:15];
    reg  [BANK_ADDR_W-1:0] bank_wr_addr3 [0:15];
    reg  [PROD_W-1:0]      bank_wr_data0 [0:15];
    reg  [PROD_W-1:0]      bank_wr_data1 [0:15];
    reg  [PROD_W-1:0]      bank_wr_data2 [0:15];
    reg  [PROD_W-1:0]      bank_wr_data3 [0:15];

    integer ji, ki, ni;
    always @(*) begin
        for (ji = 0; ji < 16; ji = ji + 1)
            eligible[ji] = active && lane_valid[ji] && !done_mask[ji];

        // lt[j] = number of lower-index eligible lanes targeting the same bank;
        // lt==k -> this lane is the bank's (k+1)-th lane -> write port k.
        for (ji = 0; ji < 16; ji = ji + 1) begin
            lt[ji] = 5'd0;
            for (ki = 0; ki < 16; ki = ki + 1)
                if ((ki < ji) && eligible[ki] && (lbid[ki] == lbid[ji]))
                    lt[ji] = lt[ji] + 5'd1;
        end
        for (ji = 0; ji < 16; ji = ji + 1) begin
            win0[ji] = eligible[ji] && (lt[ji] == 5'd0);
            win1[ji] = eligible[ji] && (lt[ji] == 5'd1);
            win2[ji] = eligible[ji] && (lt[ji] == 5'd2);
            win3[ji] = eligible[ji] && (lt[ji] == 5'd3);
        end

        // port k lane needs >=(k+1) free slots (shares the cycle with ports 0..k-1).
        for (ji = 0; ji < 16; ji = ji + 1)
            lane_enq[ji] = (win0[ji] && (free_arr[lbid[ji]] >= 1)) ||
                           (win1[ji] && (free_arr[lbid[ji]] >= 2)) ||
                           (win2[ji] && (free_arr[lbid[ji]] >= 3)) ||
                           (win3[ji] && (free_arr[lbid[ji]] >= 4));

        // per-bank four write ports (<=1 lane per win-rank per bank)
        for (ni = 0; ni < 16; ni = ni + 1) begin
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
            for (ji = 0; ji < 16; ji = ji + 1) begin
                if (win0[ji] && (lbid[ji] == ni[3:0]) && (free_arr[ni] >= 1)) begin
                    bank_wr_en0[ni]   = 1'b1;
                    bank_wr_addr0[ni] = laddr[ji];
                    bank_wr_data0[ni] = lprod[ji];
                end
                if (win1[ji] && (lbid[ji] == ni[3:0]) && (free_arr[ni] >= 2)) begin
                    bank_wr_en1[ni]   = 1'b1;
                    bank_wr_addr1[ni] = laddr[ji];
                    bank_wr_data1[ni] = lprod[ji];
                end
                if (win2[ji] && (lbid[ji] == ni[3:0]) && (free_arr[ni] >= 3)) begin
                    bank_wr_en2[ni]   = 1'b1;
                    bank_wr_addr2[ni] = laddr[ji];
                    bank_wr_data2[ni] = lprod[ji];
                end
                if (win3[ji] && (lbid[ji] == ni[3:0]) && (free_arr[ni] >= 4)) begin
                    bank_wr_en3[ni]   = 1'b1;
                    bank_wr_addr3[ni] = laddr[ji];
                    bank_wr_data3[ni] = lprod[ji];
                end
            end
        end
    end

    wire [15:0] next_done       = done_mask | lane_enq;
    // Only a presented group contributes "remaining" lanes; with no group
    // (issue_valid=0) remaining is 0 so issue_ready stays high (ready for next).
    wire [15:0] grp_lanes       = active ? lane_valid : 16'b0;
    wire [15:0] remaining_after = grp_lanes & ~next_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            done_mask <= 16'b0;
        else if (active && !issue_ready)   // partial: remember enqueued lanes
            done_mask <= next_done;
        else                               // group consumed, or no active group
            done_mask <= 16'b0;
    end

    //=========================================================================
    // Bank instances
    //=========================================================================
    wire [BANK_FIFO_LOG:0] free_b0,  free_b1,  free_b2,  free_b3,
                           free_b4,  free_b5,  free_b6,  free_b7,
                           free_b8,  free_b9,  free_b10, free_b11,
                           free_b12, free_b13, free_b14, free_b15;
    wire rmw_b0,  rmw_b1,  rmw_b2,  rmw_b3,  rmw_b4,  rmw_b5,  rmw_b6,  rmw_b7,
         rmw_b8,  rmw_b9,  rmw_b10, rmw_b11, rmw_b12, rmw_b13, rmw_b14, rmw_b15;
    wire emp_b0,  emp_b1,  emp_b2,  emp_b3,  emp_b4,  emp_b5,  emp_b6,  emp_b7,
         emp_b8,  emp_b9,  emp_b10, emp_b11, emp_b12, emp_b13, emp_b14, emp_b15;
    wire clr_b0,  clr_b1,  clr_b2,  clr_b3,  clr_b4,  clr_b5,  clr_b6,  clr_b7,
         clr_b8,  clr_b9,  clr_b10, clr_b11, clr_b12, clr_b13, clr_b14, clr_b15;
    wire [EPOCH_W-1:0] dtag_b0,  dtag_b1,  dtag_b2,  dtag_b3,
                       dtag_b4,  dtag_b5,  dtag_b6,  dtag_b7,
                       dtag_b8,  dtag_b9,  dtag_b10, dtag_b11,
                       dtag_b12, dtag_b13, dtag_b14, dtag_b15;
    wire [ACC_W-1:0]   dacc_b0,  dacc_b1,  dacc_b2,  dacc_b3,
                       dacc_b4,  dacc_b5,  dacc_b6,  dacc_b7,
                       dacc_b8,  dacc_b9,  dacc_b10, dacc_b11,
                       dacc_b12, dacc_b13, dacc_b14, dacc_b15;

    wire tag_clear_pulse = (state == S_CLEAR_TAGS) && !clr_triggered;
    wire [BANK_ADDR_W-1:0] drain_rd_addr = group_addr;

    `define BANK_INST(N, FREE, RMW, EMP, CLR, DTAG, DACC) \
    accum_bank_16 #( \
        .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W), \
        .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W), \
        .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG) \
    ) u_bank``N ( \
        .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch), \
        .wr_en0(bank_wr_en0[N]), .wr_addr0(bank_wr_addr0[N]), .wr_data0(bank_wr_data0[N]), \
        .wr_en1(bank_wr_en1[N]), .wr_addr1(bank_wr_addr1[N]), .wr_data1(bank_wr_data1[N]), \
        .wr_en2(bank_wr_en2[N]), .wr_addr2(bank_wr_addr2[N]), .wr_data2(bank_wr_data2[N]), \
        .wr_en3(bank_wr_en3[N]), .wr_addr3(bank_wr_addr3[N]), .wr_data3(bank_wr_data3[N]), \
        .free_count(FREE), .rmw_busy(RMW), .fifo_empty(EMP), \
        .tag_clear_en(tag_clear_pulse), .tag_clear_busy(CLR), \
        .drain_rd_addr(drain_rd_addr), .drain_tag(DTAG), .drain_acc(DACC) \
    )

    `BANK_INST(0,  free_b0,  rmw_b0,  emp_b0,  clr_b0,  dtag_b0,  dacc_b0);
    `BANK_INST(1,  free_b1,  rmw_b1,  emp_b1,  clr_b1,  dtag_b1,  dacc_b1);
    `BANK_INST(2,  free_b2,  rmw_b2,  emp_b2,  clr_b2,  dtag_b2,  dacc_b2);
    `BANK_INST(3,  free_b3,  rmw_b3,  emp_b3,  clr_b3,  dtag_b3,  dacc_b3);
    `BANK_INST(4,  free_b4,  rmw_b4,  emp_b4,  clr_b4,  dtag_b4,  dacc_b4);
    `BANK_INST(5,  free_b5,  rmw_b5,  emp_b5,  clr_b5,  dtag_b5,  dacc_b5);
    `BANK_INST(6,  free_b6,  rmw_b6,  emp_b6,  clr_b6,  dtag_b6,  dacc_b6);
    `BANK_INST(7,  free_b7,  rmw_b7,  emp_b7,  clr_b7,  dtag_b7,  dacc_b7);
    `BANK_INST(8,  free_b8,  rmw_b8,  emp_b8,  clr_b8,  dtag_b8,  dacc_b8);
    `BANK_INST(9,  free_b9,  rmw_b9,  emp_b9,  clr_b9,  dtag_b9,  dacc_b9);
    `BANK_INST(10, free_b10, rmw_b10, emp_b10, clr_b10, dtag_b10, dacc_b10);
    `BANK_INST(11, free_b11, rmw_b11, emp_b11, clr_b11, dtag_b11, dacc_b11);
    `BANK_INST(12, free_b12, rmw_b12, emp_b12, clr_b12, dtag_b12, dacc_b12);
    `BANK_INST(13, free_b13, rmw_b13, emp_b13, clr_b13, dtag_b13, dacc_b13);
    `BANK_INST(14, free_b14, rmw_b14, emp_b14, clr_b14, dtag_b14, dacc_b14);
    `BANK_INST(15, free_b15, rmw_b15, emp_b15, clr_b15, dtag_b15, dacc_b15);

    `undef BANK_INST

    assign free_arr[0]  = free_b0;  assign free_arr[1]  = free_b1;
    assign free_arr[2]  = free_b2;  assign free_arr[3]  = free_b3;
    assign free_arr[4]  = free_b4;  assign free_arr[5]  = free_b5;
    assign free_arr[6]  = free_b6;  assign free_arr[7]  = free_b7;
    assign free_arr[8]  = free_b8;  assign free_arr[9]  = free_b9;
    assign free_arr[10] = free_b10; assign free_arr[11] = free_b11;
    assign free_arr[12] = free_b12; assign free_arr[13] = free_b13;
    assign free_arr[14] = free_b14; assign free_arr[15] = free_b15;

    //=========================================================================
    // issue_ready — high on the cycle that enqueues the LAST remaining lane(s)
    // of the current group, so the upstream advances to the next group.
    //=========================================================================
    assign issue_ready = accepting && (remaining_after == 16'b0);

    wire all_fifos_empty = emp_b0 & emp_b1 & emp_b2 & emp_b3 & emp_b4 & emp_b5 & emp_b6 & emp_b7
                         & emp_b8 & emp_b9 & emp_b10& emp_b11& emp_b12& emp_b13& emp_b14& emp_b15;
    wire all_rmw_done    = ~rmw_b0 & ~rmw_b1 & ~rmw_b2 & ~rmw_b3 & ~rmw_b4 & ~rmw_b5 & ~rmw_b6 & ~rmw_b7
                         & ~rmw_b8 & ~rmw_b9 & ~rmw_b10& ~rmw_b11& ~rmw_b12& ~rmw_b13& ~rmw_b14& ~rmw_b15;
    wire all_clr_done    = ~clr_b0 & ~clr_b1 & ~clr_b2 & ~clr_b3 & ~clr_b4 & ~clr_b5 & ~clr_b6 & ~clr_b7
                         & ~clr_b8 & ~clr_b9 & ~clr_b10& ~clr_b11& ~clr_b12& ~clr_b13& ~clr_b14& ~clr_b15;

    //=========================================================================
    // Drain output — 16-wide, one group per cycle
    //=========================================================================
    wire grp_v0  = (dtag_b0  == row_epoch) && (dacc_b0  != {ACC_W{1'b0}});
    wire grp_v1  = (dtag_b1  == row_epoch) && (dacc_b1  != {ACC_W{1'b0}});
    wire grp_v2  = (dtag_b2  == row_epoch) && (dacc_b2  != {ACC_W{1'b0}});
    wire grp_v3  = (dtag_b3  == row_epoch) && (dacc_b3  != {ACC_W{1'b0}});
    wire grp_v4  = (dtag_b4  == row_epoch) && (dacc_b4  != {ACC_W{1'b0}});
    wire grp_v5  = (dtag_b5  == row_epoch) && (dacc_b5  != {ACC_W{1'b0}});
    wire grp_v6  = (dtag_b6  == row_epoch) && (dacc_b6  != {ACC_W{1'b0}});
    wire grp_v7  = (dtag_b7  == row_epoch) && (dacc_b7  != {ACC_W{1'b0}});
    wire grp_v8  = (dtag_b8  == row_epoch) && (dacc_b8  != {ACC_W{1'b0}});
    wire grp_v9  = (dtag_b9  == row_epoch) && (dacc_b9  != {ACC_W{1'b0}});
    wire grp_v10 = (dtag_b10 == row_epoch) && (dacc_b10 != {ACC_W{1'b0}});
    wire grp_v11 = (dtag_b11 == row_epoch) && (dacc_b11 != {ACC_W{1'b0}});
    wire grp_v12 = (dtag_b12 == row_epoch) && (dacc_b12 != {ACC_W{1'b0}});
    wire grp_v13 = (dtag_b13 == row_epoch) && (dacc_b13 != {ACC_W{1'b0}});
    wire grp_v14 = (dtag_b14 == row_epoch) && (dacc_b14 != {ACC_W{1'b0}});
    wire grp_v15 = (dtag_b15 == row_epoch) && (dacc_b15 != {ACC_W{1'b0}});

    assign drain_valid  = (state == S_DRAIN) ?
        {grp_v15,grp_v14,grp_v13,grp_v12,grp_v11,grp_v10,grp_v9,grp_v8,
         grp_v7, grp_v6, grp_v5, grp_v4, grp_v3, grp_v2, grp_v1, grp_v0} : 16'b0;
    assign drain_active = (state == S_DRAIN);
    assign drain_gaddr  = group_addr;
    assign drain_row_id = cur_row_id;
    assign drain_values = {dacc_b15,dacc_b14,dacc_b13,dacc_b12,dacc_b11,dacc_b10,dacc_b9,dacc_b8,
                           dacc_b7, dacc_b6, dacc_b5, dacc_b4, dacc_b3, dacc_b2, dacc_b1, dacc_b0};

    wire [COL_W:0]         drain_cols_m1   = drain_cols - 1'b1;
    wire [BANK_ADDR_W-1:0] last_group_addr = drain_cols_m1[COL_W-1:4];
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
            // than the 32-cycle walk, so first-row issue is never stalled.
            state            <= S_CLEAR_TAGS;
            cur_row_id       <= {ROW_W{1'b0}};
            row_epoch        <= {{(EPOCH_W-1){1'b0}}, 1'b1};
            input_done_latch <= 1'b0;
            clr_triggered    <= 1'b0;
            group_addr       <= {BANK_ADDR_W{1'b0}};
            busy             <= 1'b0;
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
