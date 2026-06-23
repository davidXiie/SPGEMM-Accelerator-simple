//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : PE Top — 4-wide B streaming for full MAC utilization.
//            B_col/val buffers are 4-banked (by absolute index[1:0]);
//            b_ptr advances by 4 each cycle, writing one task_group directly
//            to the task FIFO per cycle — no task_packer needed.
//
//   Requirement: Python must 4-align each B row's start offset (pad to
//   ceil(nnz/4)*4 with dummy col=0 val=0 elements) before loading.
//
//   FSM (10 states):
//     PE_IDLE → PE_LOAD_ROW_DESC → PE_CLEAR_ACC
//     → PE_LOAD_A_ELEM → PE_LOAD_B_DESC → PE_STREAM_B_ROW
//     → PE_WAIT_TASK_DRAIN → PE_WAIT_PRODUCT_DRAIN
//     → PE_NEXT_ROW → PE_DONE
//
//   A buffer:  A_row_desc_buf (64bit) + A_col_buf (16bit) + A_val_buf (16bit)
//   B buffer:  B_row_desc_buf (64bit) + B_col/val_b0..b3 (16bit × 4 banks)
//=============================================================================

`include "defines.vh"

module pe_top #(
    parameter PE_ID = 0
) (
    input  wire                     aclk,
    input  wire                     aresetn,

    input  wire                     start,
    input  wire [15:0]              row_count,
    output reg                      done,

    input  wire [`MAX_DIM_BITS-1:0]  M,
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // A buffer load ports
    input  wire                     a_desc_we,
    input  wire [`A_ROW_ADDR_BITS-1:0] a_desc_waddr,
    input  wire [63:0]              a_desc_wdata,
    input  wire                     a_col_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0] a_col_waddr,
    input  wire [`DATA_WIDTH-1:0]   a_col_wdata,
    input  wire                     a_val_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0] a_val_waddr,
    input  wire [`DATA_WIDTH-1:0]   a_val_wdata,

    // B buffer load ports
    input  wire                     b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0] b_desc_waddr,
    input  wire [63:0]              b_desc_wdata,
    input  wire                     b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]   b_col_wdata,
    input  wire                     b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]   b_val_wdata,

    // C dense buffer write (handshake)
    output wire                     cbuf_wr_valid,
    input  wire                     cbuf_wr_ready,
    output wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr,
    output wire [`DATA_WIDTH-1:0]   cbuf_wr_data
);

    //=========================================================================
    // A Buffer
    //=========================================================================
    reg [63:0]            A_row_desc_buf [0:`A_ROW_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf      [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_val_buf      [0:`A_NNZ_SLOT_PER_PE-1];

    always @(posedge aclk) begin
        if (a_desc_we) A_row_desc_buf[a_desc_waddr] <= a_desc_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]       <= a_col_wdata;
        if (a_val_we)  A_val_buf[a_val_waddr]       <= a_val_wdata;
    end

    //=========================================================================
    // B Buffer — 4 banks split by absolute index[1:0]
    // B rows must start at 4-aligned offsets (Python pads to ceil(nnz/4)*4).
    //=========================================================================
    localparam B_BANK_DEPTH = `B_NNZ_SLOT / 4;   // 19712

    reg [63:0]            B_row_desc_buf [0:`B_ROW_SLOT-1];
    reg [`DATA_WIDTH-1:0] B_col_b0       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b1       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b2       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b3       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b0       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b1       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b2       [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b3       [0:B_BANK_DEPTH-1];

    always @(posedge aclk) begin
        if (b_desc_we) B_row_desc_buf[b_desc_waddr] <= b_desc_wdata;
        if (b_col_we) case (b_col_waddr[1:0])
            2'd0: B_col_b0[b_col_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_col_wdata;
            2'd1: B_col_b1[b_col_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_col_wdata;
            2'd2: B_col_b2[b_col_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_col_wdata;
            2'd3: B_col_b3[b_col_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_col_wdata;
        endcase
        if (b_val_we) case (b_val_waddr[1:0])
            2'd0: B_val_b0[b_val_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_val_wdata;
            2'd1: B_val_b1[b_val_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_val_wdata;
            2'd2: B_val_b2[b_val_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_val_wdata;
            2'd3: B_val_b3[b_val_waddr[`B_NNZ_ADDR_BITS-1:2]] <= b_val_wdata;
        endcase
    end

    //=========================================================================
    // FSM States
    //=========================================================================
    localparam PE_IDLE               = 4'd0;
    localparam PE_LOAD_ROW_DESC      = 4'd1;
    localparam PE_CLEAR_ACC          = 4'd2;
    localparam PE_LOAD_A_ELEM        = 4'd3;
    localparam PE_LOAD_B_DESC        = 4'd4;
    localparam PE_STREAM_B_ROW       = 4'd5;
    localparam PE_WAIT_TASK_DRAIN    = 4'd6;
    localparam PE_WAIT_PRODUCT_DRAIN = 4'd7;
    localparam PE_NEXT_ROW           = 4'd8;
    localparam PE_DONE               = 4'd9;

    reg [3:0] state, state_next;

    //=========================================================================
    // Row-level registers
    //=========================================================================
    reg comp_sel;
    reg [`A_ROW_ADDR_BITS-1:0] row_idx;
    reg [63:0]            row_desc_reg;
    reg [`DATA_WIDTH-1:0] cur_global_row;
    reg [`DATA_WIDTH-1:0] cur_a_row_nnz;
    reg [`OFFSET_WIDTH-1:0] cur_a_start;

    //=========================================================================
    // A iterator registers
    //=========================================================================
    reg [`DATA_WIDTH-1:0]   a_nnz_left;
    reg [`OFFSET_WIDTH-1:0] a_ptr;
    wire [`A_NNZ_ADDR_BITS-1:0] a_ptr_next  = a_ptr[`A_NNZ_ADDR_BITS-1:0] + 1'b1;
    wire [`A_NNZ_ADDR_BITS-1:0] a_ptr_plus2 = a_ptr[`A_NNZ_ADDR_BITS-1:0] + 2'd2;
    reg [`DATA_WIDTH-1:0]   cur_k;
    reg [1:0]               b_lane_skip;    // leading lanes in cur B group already cross-filled
    reg [`DATA_WIDTH-1:0]   cur_a_val;

    //=========================================================================
    // B streamer registers (b_ptr always 4-aligned)
    //=========================================================================
    reg [63:0]            b_row_desc_reg;
    reg [`OFFSET_WIDTH-1:0] b_ptr;
    reg [`DATA_WIDTH-1:0]   b_nnz_left;

    //=========================================================================
    // 4-wide B read (combinational — b_ptr is always 4-aligned)
    //=========================================================================
    wire [`B_NNZ_ADDR_BITS-3:0] b_group = b_ptr[`B_NNZ_ADDR_BITS-1:2];

    wire [`DATA_WIDTH-1:0] bc0 = B_col_b0[b_group];
    wire [`DATA_WIDTH-1:0] bc1 = B_col_b1[b_group];
    wire [`DATA_WIDTH-1:0] bc2 = B_col_b2[b_group];
    wire [`DATA_WIDTH-1:0] bc3 = B_col_b3[b_group];
    wire [`DATA_WIDTH-1:0] bv0 = B_val_b0[b_group];
    wire [`DATA_WIDTH-1:0] bv1 = B_val_b1[b_group];
    wire [`DATA_WIDTH-1:0] bv2 = B_val_b2[b_group];
    wire [`DATA_WIDTH-1:0] bv3 = B_val_b3[b_group];

    //=========================================================================
    // Next-A-element prefetch for cross-element task filling
    //   3-level cascade: A_col_buf[a_ptr+1] → B_row_desc_buf → B_col/val banks
    //=========================================================================
    wire [`DATA_WIDTH-1:0]    nxt_k_w      = A_col_buf[a_ptr_next];
    wire [`DATA_WIDTH-1:0]    nxt_a_val_w  = A_val_buf[a_ptr_next];
    wire [63:0]               nxt_b_desc_w = B_row_desc_buf[nxt_k_w[`B_ROW_ADDR_BITS-1:0]];
    wire [`OFFSET_WIDTH-1:0]  nxt_b_ptr_w  = nxt_b_desc_w[63:32];
    wire [`DATA_WIDTH-1:0]    nxt_b_nnz_w  = nxt_b_desc_w[15:0];
    wire [`B_NNZ_ADDR_BITS-3:0] nb_group   = nxt_b_ptr_w[`B_NNZ_ADDR_BITS-1:2];
    wire [`DATA_WIDTH-1:0] nbc0 = B_col_b0[nb_group];
    wire [`DATA_WIDTH-1:0] nbc1 = B_col_b1[nb_group];
    wire [`DATA_WIDTH-1:0] nbc2 = B_col_b2[nb_group];
    wire [`DATA_WIDTH-1:0] nbc3 = B_col_b3[nb_group];
    wire [`DATA_WIDTH-1:0] nbv0 = B_val_b0[nb_group];
    wire [`DATA_WIDTH-1:0] nbv1 = B_val_b1[nb_group];
    wire [`DATA_WIDTH-1:0] nbv2 = B_val_b2[nb_group];
    wire [`DATA_WIDTH-1:0] nbv3 = B_val_b3[nb_group];

    //=========================================================================
    // Task Group generation — cross-element filling support
    //=========================================================================

    // Available slots in current group (4 minus skipped leading lanes)
    wire [15:0] avail_slots_w = 16'd4 - {14'd0, b_lane_skip};
    // Last group: all remaining elements fit in current group's available slots
    wire is_last_b_group = (b_nnz_left > 16'd0) && (b_nnz_left <= avail_slots_w);

    wire task_fifo_full;   // driven by u_task_fifo below
    wire stream_group_valid = (state == PE_STREAM_B_ROW) && (b_nnz_left != 0);
    wire task_group_wr_en   = stream_group_valid && !task_fifo_full;

    // Spare slots available for cross-fill (only meaningful when is_last_b_group)
    wire [2:0] spare_slots_w = avail_slots_w[2:0] - b_nnz_left[2:0];
    // How many slots to fill from next A element's B row (full 16-bit comparison avoids wrap-around)
    wire [2:0] fill_used_w = (nxt_b_nnz_w > {13'd0, spare_slots_w}) ? spare_slots_w
                                                                      : nxt_b_nnz_w[2:0];
    // Cross-fill: last group + more A elements + next B row non-empty + no existing skip + free slots
    wire do_cross_fill    = is_last_b_group && (a_nnz_left > 16'd1) && (nxt_b_nnz_w > 16'd0)
                          && (b_lane_skip == 2'd0) && (spare_slots_w > 3'd0) && task_group_wr_en;
    // fill_exhausts_nxt: entire next B row consumed by cross-fill (full 16-bit comparison)
    wire fill_exhausts_nxt = do_cross_fill && (nxt_b_nnz_w <= {13'd0, spare_slots_w});

    // Total valid lanes in last group (current + fill)
    wire [3:0] total_valid_lanes = {2'd0, b_lane_skip} + {1'b0, b_nnz_left[2:0]}
                                 + (do_cross_fill ? {1'b0, fill_used_w} : 4'd0);

    // Per-lane validity
    wire [3:0] stream_lane_valid;
    assign stream_lane_valid[0] = (b_lane_skip == 2'd0)
                                && (is_last_b_group ? (total_valid_lanes >= 4'd1) : 1'b1);
    assign stream_lane_valid[1] = (b_lane_skip <= 2'd1)
                                && (is_last_b_group ? (total_valid_lanes >= 4'd2) : 1'b1);
    assign stream_lane_valid[2] = (b_lane_skip <= 2'd2)
                                && (is_last_b_group ? (total_valid_lanes >= 4'd3) : 1'b1);
    assign stream_lane_valid[3] = (is_last_b_group ? (total_valid_lanes >= 4'd4) : 1'b1);

    // Cross-fill mux: lane i >= b_nnz_left uses nbc[i-b_nnz_left] / nxt_a_val_w
    // b_lane_skip controls VALIDITY only — each lane always reads its own bank[i].
    wire cf1 = do_cross_fill && (b_nnz_left == 16'd1);
    wire cf2 = do_cross_fill && (b_nnz_left <= 16'd2);
    wire cf3 = do_cross_fill && (b_nnz_left <= 16'd3);

    wire [`DATA_WIDTH-1:0] sg0_bc = bc0;
    wire [`DATA_WIDTH-1:0] sg0_bv = bv0;
    wire [`DATA_WIDTH-1:0] sg0_av = cur_a_val;

    wire [`DATA_WIDTH-1:0] sg1_bc = cf1 ? nbc0 : bc1;
    wire [`DATA_WIDTH-1:0] sg1_bv = cf1 ? nbv0 : bv1;
    wire [`DATA_WIDTH-1:0] sg1_av = cf1 ? nxt_a_val_w : cur_a_val;

    wire [`DATA_WIDTH-1:0] sg2_bc = !cf2 ? bc2 : (b_nnz_left == 16'd1) ? nbc1 : nbc0;
    wire [`DATA_WIDTH-1:0] sg2_bv = !cf2 ? bv2 : (b_nnz_left == 16'd1) ? nbv1 : nbv0;
    wire [`DATA_WIDTH-1:0] sg2_av = cf2 ? nxt_a_val_w : cur_a_val;

    wire [`DATA_WIDTH-1:0] sg3_bc = !cf3 ? bc3
                                  : (b_nnz_left == 16'd1) ? nbc2
                                  : (b_nnz_left == 16'd2) ? nbc1 : nbc0;
    wire [`DATA_WIDTH-1:0] sg3_bv = !cf3 ? bv3
                                  : (b_nnz_left == 16'd1) ? nbv2
                                  : (b_nnz_left == 16'd2) ? nbv1 : nbv0;
    wire [`DATA_WIDTH-1:0] sg3_av = cf3 ? nxt_a_val_w : cur_a_val;

    wire [`TASK_WIDTH-1:0] sg0 = {16'd0, sg0_bv, sg0_av, sg0_bc};
    wire [`TASK_WIDTH-1:0] sg1 = {16'd0, sg1_bv, sg1_av, sg1_bc};
    wire [`TASK_WIDTH-1:0] sg2 = {16'd0, sg2_bv, sg2_av, sg2_bc};
    wire [`TASK_WIDTH-1:0] sg3 = {16'd0, sg3_bv, sg3_av, sg3_bc};

    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data = {sg3, sg2, sg1, sg0, stream_lane_valid};

    wire b_last_group_fires = task_group_wr_en && is_last_b_group;
    wire b_batch_done       = (state == PE_STREAM_B_ROW) && (b_nnz_left == 0 || b_last_group_fires);

    //=========================================================================
    // Task Group FIFO → MAC Array
    //=========================================================================
    wire task_fifo_rd_en;
    wire [`TASK_GROUP_WIDTH-1:0] task_fifo_rd_data;
    wire task_fifo_empty;

    sync_fifo #(
        .WIDTH(`TASK_GROUP_WIDTH), .DEPTH(`TASK_FIFO_DEPTH),
        .DEPTH_LOG(`TASK_FIFO_DEPTH_LOG)
    ) u_task_fifo (
        .wr_en    (task_group_wr_en),
        .wr_data  (task_group_wr_data),
        .wr_full  (task_fifo_full),
        .rd_en    (task_fifo_rd_en),
        .rd_data  (task_fifo_rd_data),
        .rd_empty (task_fifo_empty),
        .count    (),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    //=========================================================================
    // MAC Array (4-lane)
    //=========================================================================
    wire [`N_MAC-1:0] mac_lane_valid;
    wire [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task;

    reg [`N_MAC-1:0] mac_lane_valid_r;
    reg [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task_r;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            mac_lane_valid_r <= 0;
            mac_lane_task_r  <= 0;
        end else if (task_fifo_rd_en) begin
            mac_lane_valid_r <= task_fifo_rd_data[3:0];
            mac_lane_task_r[0*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data[67:4];
            mac_lane_task_r[1*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data[131:68];
            mac_lane_task_r[2*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data[195:132];
            mac_lane_task_r[3*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data[259:196];
        end else begin
            mac_lane_valid_r <= 0;
        end
    end
    assign mac_lane_valid = mac_lane_valid_r;
    assign mac_lane_task  = mac_lane_task_r;

    wire [`N_MAC-1:0] mul_valid;
    wire [`N_MAC*`PRODUCT_WIDTH-1:0] mul_product;

    pe_mul_array u_mul_array (
        .lane_valid  (mac_lane_valid),
        .lane_task   (mac_lane_task),
        .mul_valid   (mul_valid),
        .mul_product (mul_product),
        .aclk        (aclk),
        .aresetn     (aresetn)
    );

    //=========================================================================
    // Product Group FIFO
    //=========================================================================
    wire product_group_wr_en;
    wire [`PRODUCT_GROUP_WIDTH-1:0] product_group_wr_data;
    wire product_fifo_full;
    wire [`PROD_FIFO_DEPTH_LOG:0] product_fifo_cnt;

    assign product_group_wr_en         = |mul_valid && !product_fifo_full;
    assign product_group_wr_data[3:0]  = mul_valid;
    assign product_group_wr_data[35:4]   = mul_product[0*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[67:36]  = mul_product[1*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[99:68]  = mul_product[2*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[131:100]= mul_product[3*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];

    wire prod_fifo_rd_en;
    wire [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data;
    wire prod_fifo_empty;

    sync_fifo #(
        .WIDTH(`PRODUCT_GROUP_WIDTH), .DEPTH(`PROD_FIFO_DEPTH),
        .DEPTH_LOG(`PROD_FIFO_DEPTH_LOG)
    ) u_product_fifo (
        .wr_en    (product_group_wr_en),
        .wr_data  (product_group_wr_data),
        .wr_full  (product_fifo_full),
        .rd_en    (prod_fifo_rd_en),
        .rd_data  (prod_fifo_rd_data),
        .rd_empty (prod_fifo_empty),
        .count    (product_fifo_cnt),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    //=========================================================================
    // 4-bank Row Accumulator — ping-pong pair
    //=========================================================================
    wire mac_pipeline_idle = !(|mac_lane_valid) && !(|mul_valid);

    wire        acc_busy_0,        acc_busy_1;
    wire        acc_row_done_0,    acc_row_done_1;
    wire        acc_issue_ready_0, acc_issue_ready_1;
    wire        acc_out_valid_0,   acc_out_valid_1;
    wire [8:0]  acc_out_col_id_0,  acc_out_col_id_1;
    wire [31:0] acc_out_value_0,   acc_out_value_1;
    wire [15:0] acc_out_row_id_0,  acc_out_row_id_1;

    wire acc_issue_ready = comp_sel ? acc_issue_ready_1 : acc_issue_ready_0;
    wire other_acc_busy  = comp_sel ? acc_busy_0        : acc_busy_1;

    wire acc_row_start_0 = (state == PE_CLEAR_ACC) && !comp_sel;
    wire acc_row_start_1 = (state == PE_CLEAR_ACC) &&  comp_sel;

    wire acc_inp_done = (state == PE_WAIT_PRODUCT_DRAIN)
                      && prod_fifo_empty && mac_pipeline_idle
                      && !other_acc_busy;
    wire acc_inp_done_0 = acc_inp_done && !comp_sel;
    wire acc_inp_done_1 = acc_inp_done &&  comp_sel;

    wire issue_valid_0 = !prod_fifo_empty && !comp_sel;
    wire issue_valid_1 = !prod_fifo_empty &&  comp_sel;

    wire acc_out_ready_0 =  comp_sel && cbuf_wr_ready;
    wire acc_out_ready_1 = !comp_sel && cbuf_wr_ready;

    assign prod_fifo_rd_en = !prod_fifo_empty && acc_issue_ready;

    wire [3:0]  acc_lane_valid;
    wire [35:0] acc_lane_col_id;
    wire [63:0] acc_lane_product;

    assign acc_lane_valid   = prod_fifo_rd_data[3:0];
    assign acc_lane_col_id  = {prod_fifo_rd_data[4+3*32+16 +: 9],
                               prod_fifo_rd_data[4+2*32+16 +: 9],
                               prod_fifo_rd_data[4+1*32+16 +: 9],
                               prod_fifo_rd_data[4+0*32+16 +: 9]};
    assign acc_lane_product = {prod_fifo_rd_data[4+3*32 +: 16],
                               prod_fifo_rd_data[4+2*32 +: 16],
                               prod_fifo_rd_data[4+1*32 +: 16],
                               prod_fifo_rd_data[4+0*32 +: 16]};

    row_accumulator_4bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(`DATA_WIDTH),
        .ACC_W(32), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5), .ROW_W(16)
    ) u_row_acc_0 (
        .clk(aclk), .rst_n(aresetn),
        .row_start(acc_row_start_0), .row_id_in(cur_global_row), .drain_cols(N),
        .row_input_done(acc_inp_done_0), .busy(acc_busy_0), .row_done(acc_row_done_0),
        .issue_valid(issue_valid_0), .issue_ready(acc_issue_ready_0),
        .lane_valid(acc_lane_valid), .lane_col_id(acc_lane_col_id), .lane_product(acc_lane_product),
        .out_valid(acc_out_valid_0), .out_ready(acc_out_ready_0),
        .out_row_id(acc_out_row_id_0), .out_col_id(acc_out_col_id_0), .out_value(acc_out_value_0)
    );

    row_accumulator_4bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(`DATA_WIDTH),
        .ACC_W(32), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5), .ROW_W(16)
    ) u_row_acc_1 (
        .clk(aclk), .rst_n(aresetn),
        .row_start(acc_row_start_1), .row_id_in(cur_global_row), .drain_cols(N),
        .row_input_done(acc_inp_done_1), .busy(acc_busy_1), .row_done(acc_row_done_1),
        .issue_valid(issue_valid_1), .issue_ready(acc_issue_ready_1),
        .lane_valid(acc_lane_valid), .lane_col_id(acc_lane_col_id), .lane_product(acc_lane_product),
        .out_valid(acc_out_valid_1), .out_ready(acc_out_ready_1),
        .out_row_id(acc_out_row_id_1), .out_col_id(acc_out_col_id_1), .out_value(acc_out_value_1)
    );

    //=========================================================================
    // Task FIFO read control
    //=========================================================================
    // mac_lane_valid_r adds 1 register stage before pe_mul_array, so the
    // effective write latency is MUL_LAT+1 cycles. Reserve that many slots.
    assign task_fifo_rd_en = !task_fifo_empty &&
                             (product_fifo_cnt < (`PROD_FIFO_DEPTH - `MUL_LAT - 1));

    //=========================================================================
    // Row writeback — drain accumulator drives cbuf_wr continuously
    //=========================================================================
    wire        drain_out_valid  = comp_sel ? acc_out_valid_0  : acc_out_valid_1;
    wire [8:0]  drain_out_col_id = comp_sel ? acc_out_col_id_0 : acc_out_col_id_1;
    wire [31:0] drain_out_value  = comp_sel ? acc_out_value_0  : acc_out_value_1;
    wire [15:0] drain_out_row_id = comp_sel ? acc_out_row_id_0 : acc_out_row_id_1;

    assign cbuf_wr_valid = drain_out_valid;
    assign cbuf_wr_addr  = (drain_out_row_id * `C_ROW_STRIDE) + drain_out_col_id;
    assign cbuf_wr_data  = drain_out_value[`DATA_WIDTH-1:0];

    //=========================================================================
    // FSM sequential logic
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= PE_IDLE;
        end else begin
            state <= state_next;
        end
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            comp_sel       <= 1'b0;
            row_idx        <= 0;
            row_desc_reg   <= 0;
            cur_global_row <= 0;
            cur_a_row_nnz  <= 0;
            cur_a_start    <= 0;
            a_nnz_left     <= 0;
            a_ptr          <= 0;
            cur_k          <= 0;
            cur_a_val      <= 0;
            b_row_desc_reg <= 0;
            b_ptr          <= 0;
            b_nnz_left     <= 0;
            b_lane_skip    <= 2'd0;
            done           <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                PE_IDLE: begin
                    if (start) row_idx <= 0;
                end

                PE_LOAD_ROW_DESC: begin
                    row_desc_reg   <= A_row_desc_buf[row_idx];
                    cur_global_row <= A_row_desc_buf[row_idx][15:0];
                    cur_a_row_nnz  <= A_row_desc_buf[row_idx][31:16];
                    cur_a_start    <= A_row_desc_buf[row_idx][63:32];
                    a_ptr          <= A_row_desc_buf[row_idx][63:32];
                    a_nnz_left     <= A_row_desc_buf[row_idx][31:16];
                end

                PE_CLEAR_ACC: begin
                    b_lane_skip <= 2'd0;   // reset any residual skip from previous row
                end

                PE_LOAD_A_ELEM: begin
                    cur_k          <= A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
                    cur_a_val      <= A_val_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
                    // Cascade: use combinational A_col_buf read as B_row_desc_buf index.
                    // Eliminates PE_LOAD_B_DESC state — 4-cycle setup → 1-cycle setup.
                    b_row_desc_reg <= B_row_desc_buf[A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]][`B_ROW_ADDR_BITS-1:0]];
                    b_ptr          <= B_row_desc_buf[A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]][`B_ROW_ADDR_BITS-1:0]][63:32];
                    b_nnz_left     <= B_row_desc_buf[A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]][`B_ROW_ADDR_BITS-1:0]][15:0];
                end

                PE_STREAM_B_ROW: begin
                    if (!task_fifo_full && b_nnz_left != 0) begin
                        if (!is_last_b_group) begin
                            // Middle group: consume avail_slots elements, reset b_lane_skip
                            b_ptr       <= b_ptr + 4;
                            b_nnz_left  <= b_nnz_left - avail_slots_w;
                            b_lane_skip <= 2'd0;
                        end else if (do_cross_fill && fill_exhausts_nxt) begin
                            // Cross-fill consumes BOTH current and next A element entirely
                            a_ptr      <= a_ptr_plus2;
                            a_nnz_left <= a_nnz_left - 16'd2;
                            b_nnz_left <= 16'd0;
                            b_lane_skip <= 2'd0;
                        end else if (do_cross_fill) begin
                            // Cross-fill: next A's B row partially consumed (fill_used_w elements)
                            a_ptr      <= a_ptr + 1;
                            a_nnz_left <= a_nnz_left - 1;
                            cur_k          <= nxt_k_w;
                            cur_a_val      <= nxt_a_val_w;
                            b_row_desc_reg <= nxt_b_desc_w;
                            b_ptr          <= nxt_b_ptr_w;
                            b_nnz_left     <= nxt_b_nnz_w - {13'd0, fill_used_w};
                            b_lane_skip    <= fill_used_w[1:0];
                        end else begin
                            // Normal last group: advance A pointer, optionally prefetch next
                            a_ptr      <= a_ptr + 1;
                            a_nnz_left <= a_nnz_left - 1;
                            b_lane_skip <= 2'd0;
                            if (a_nnz_left > 1) begin
                                // Prefetch next A element so LOAD_A_ELEM is skipped.
                                cur_k          <= A_col_buf[a_ptr_next];
                                cur_a_val      <= A_val_buf[a_ptr_next];
                                b_row_desc_reg <= B_row_desc_buf[A_col_buf[a_ptr_next][`B_ROW_ADDR_BITS-1:0]];
                                b_ptr          <= B_row_desc_buf[A_col_buf[a_ptr_next][`B_ROW_ADDR_BITS-1:0]][63:32];
                                b_nnz_left     <= B_row_desc_buf[A_col_buf[a_ptr_next][`B_ROW_ADDR_BITS-1:0]][15:0];
                            end else begin
                                b_nnz_left <= 16'd0;
                            end
                        end
                    end else if (b_nnz_left == 0 && a_nnz_left > 0) begin
                        // Empty B row: advance a_ptr, go through LOAD_A_ELEM
                        a_ptr      <= a_ptr + 1;
                        a_nnz_left <= a_nnz_left - 1;
                        b_lane_skip <= 2'd0;
                    end
                end

                PE_NEXT_ROW: begin
                    if (!state_stable) begin
                        row_idx  <= row_idx + 1;
                        comp_sel <= ~comp_sel;
                    end
                end

                PE_DONE: begin
                    if (!other_acc_busy) done <= 1'b1;
                end

            endcase
        end
    end

    //=========================================================================
    // state_stable: 0 on first cycle of a new state, 1 thereafter
    //=========================================================================
    reg state_stable;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) state_stable <= 1'b0;
        else if (state_next != state) state_stable <= 1'b0;
        else state_stable <= 1'b1;
    end

    //=========================================================================
    // FSM next-state logic
    //=========================================================================
    always @(*) begin
        state_next = state;
        case (state)
            PE_IDLE:
                if (start) state_next = PE_LOAD_ROW_DESC;

            PE_LOAD_ROW_DESC:
                if (state_stable) state_next = PE_CLEAR_ACC;

            PE_CLEAR_ACC:
                if (cur_a_row_nnz == 0)
                    state_next = PE_WAIT_PRODUCT_DRAIN;  // empty row
                else
                    state_next = PE_LOAD_A_ELEM;

            PE_LOAD_A_ELEM:
                state_next = PE_STREAM_B_ROW;  // 1-cycle setup: no state_stable wait

            PE_STREAM_B_ROW:
                if (b_batch_done) begin
                    if (fill_exhausts_nxt) begin
                        // Consumed two A elements simultaneously — check pre-dec by 2
                        if (a_nnz_left <= 2)
                            state_next = PE_WAIT_TASK_DRAIN;
                        else
                            state_next = PE_LOAD_A_ELEM;   // load a_ptr+2
                    end else begin
                        // a_nnz_left pre-decrement: value 1 → will become 0 after edge.
                        if (a_nnz_left <= 1)
                            state_next = PE_WAIT_TASK_DRAIN;
                        else if (b_last_group_fires)
                            state_next = PE_STREAM_B_ROW;  // prefetched/cross-filled, skip LOAD_A_ELEM
                        else
                            state_next = PE_LOAD_A_ELEM;   // empty B row still needs setup
                    end
                end

            PE_WAIT_TASK_DRAIN:
                if (task_fifo_empty) state_next = PE_WAIT_PRODUCT_DRAIN;

            PE_WAIT_PRODUCT_DRAIN:
                if (prod_fifo_empty && mac_pipeline_idle && !other_acc_busy)
                    state_next = PE_NEXT_ROW;

            PE_NEXT_ROW:
                if (state_stable) begin
                    if (row_idx < row_count)
                        state_next = PE_LOAD_ROW_DESC;
                    else
                        state_next = PE_DONE;
                end

            PE_DONE: ; // stay

            default: state_next = PE_IDLE;
        endcase
    end

    //=========================================================================
    // Debug trace (SIMULATION only) — target cols 53,56,69,73,30
    //=========================================================================
`ifdef SIMULATION
    // task_group written to FIFO — check any lane's col matches target
    wire [15:0] _tc0 = task_group_wr_data[19:4];    // sg0 col field
    wire [15:0] _tc1 = task_group_wr_data[83:68];   // sg1 col field
    wire [15:0] _tc2 = task_group_wr_data[147:132];
    wire [15:0] _tc3 = task_group_wr_data[211:196];
    wire _tg_hit = (_tc0==53||_tc0==56||_tc0==69||_tc0==73||_tc0==30||
                    _tc1==53||_tc1==56||_tc1==69||_tc1==73||_tc1==30||
                    _tc2==53||_tc2==56||_tc2==69||_tc2==73||_tc2==30||
                    _tc3==53||_tc3==56||_tc3==69||_tc3==73||_tc3==30);

    // product FIFO read side — what acc receives
    wire [15:0] _pc0 = prod_fifo_rd_data[4+0*32+16 +: 16];
    wire [15:0] _pc1 = prod_fifo_rd_data[4+1*32+16 +: 16];
    wire [15:0] _pc2 = prod_fifo_rd_data[4+2*32+16 +: 16];
    wire [15:0] _pc3 = prod_fifo_rd_data[4+3*32+16 +: 16];
    wire [15:0] _pv0 = prod_fifo_rd_data[4+0*32 +: 16];
    wire [15:0] _pv1 = prod_fifo_rd_data[4+1*32 +: 16];
    wire [15:0] _pv2 = prod_fifo_rd_data[4+2*32 +: 16];
    wire [15:0] _pv3 = prod_fifo_rd_data[4+3*32 +: 16];
    wire _pr_hit = (_pc0==53||_pc0==56||_pc0==69||_pc0==73||_pc0==30||
                    _pc1==53||_pc1==56||_pc1==69||_pc1==73||_pc1==30||
                    _pc2==53||_pc2==56||_pc2==69||_pc2==73||_pc2==30||
                    _pc3==53||_pc3==56||_pc3==69||_pc3==73||_pc3==30);

    always @(posedge aclk) begin
        // 0. Alert on dropped products (product_fifo_full when MAC output valid)
        if ((|mul_valid) && product_fifo_full)
            $display("[PROD_DROP @%0t] mul_valid=%b pf_cnt=%0d", $time, mul_valid, product_fifo_cnt);

        // 1. Task group into FIFO — check if target col is present
        if (task_group_wr_en && _tg_hit)
            $display("[TG_WR @%0t row=%0d] valid=%b cols=%0d/%0d/%0d/%0d bnz=%0d fifo_full=%b",
                $time, row_idx, stream_lane_valid,
                _tc0, _tc1, _tc2, _tc3, b_nnz_left, task_fifo_full);

        // 2. Product FIFO READ — what accumulator actually receives per cycle
        if (prod_fifo_rd_en && _pr_hit)
            $display("[PRD_RD @%0t row=%0d comp=%0d] valid=%b c0=%0d(v=%0d) c1=%0d(v=%0d) c2=%0d(v=%0d) c3=%0d(v=%0d) acc_rdy=%b",
                $time, row_idx, comp_sel, prod_fifo_rd_data[3:0],
                _pc0, _pv0, _pc1, _pv1, _pc2, _pv2, _pc3, _pv3, acc_issue_ready);

        // 3. C buffer write for target positions
        if (cbuf_wr_valid && cbuf_wr_ready &&
            ((drain_out_row_id==2 && (drain_out_col_id==53||drain_out_col_id==56||
                                      drain_out_col_id==69||drain_out_col_id==73)) ||
             (drain_out_row_id==6 && drain_out_col_id==30)))
            $display("[CBUF_WR @%0t] row=%0d col=%0d val=%0d",
                $time, drain_out_row_id, drain_out_col_id, drain_out_value);
    end
`endif

endmodule
