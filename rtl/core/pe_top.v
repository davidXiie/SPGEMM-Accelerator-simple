//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : PE Top — integrates A/B buffers, FSM, task_packer,
//            task_group_fifo, 4-MAC array, product_group_fifo,
//            serializer, accumulator, and row writeback.
//
//   FSM (12 states):
//     PE_IDLE → PE_LOAD_ROW_DESC → PE_CLEAR_ACC
//     → PE_LOAD_A_ELEM → PE_LOAD_B_DESC → PE_STREAM_B_ROW
//     → PE_FLUSH_TASK_PACK → PE_WAIT_TASK_DRAIN → PE_WAIT_PRODUCT_DRAIN
//     → PE_WRITE_ROW → PE_NEXT_ROW → PE_DONE
//
//   A buffer:  A_row_desc_buf (64bit) + A_col_buf (16bit) + A_val_buf (16bit)
//   B buffer:  B_row_desc_buf (64bit) + B_col_buf (16bit) + B_val_buf (16bit)
//=============================================================================

`include "defines.vh"

module pe_top #(
    parameter PE_ID = 0
) (
    input  wire                     aclk,
    input  wire                     aresetn,

    // Control
    input  wire                     start,
    input  wire [15:0]              row_count,
    output reg                      done,

    // Matrix dimensions
    input  wire [`MAX_DIM_BITS-1:0]  M,
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // === A buffer load ports (from a_group_loader) ===
    input  wire                     a_desc_we,
    input  wire [`A_ROW_ADDR_BITS-1:0] a_desc_waddr,
    input  wire [63:0]              a_desc_wdata,

    input  wire                     a_col_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0] a_col_waddr,
    input  wire [`DATA_WIDTH-1:0]   a_col_wdata,

    input  wire                     a_val_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0] a_val_waddr,
    input  wire [`DATA_WIDTH-1:0]   a_val_wdata,

    // === B buffer load ports (from b_broadcast_loader) ===
    input  wire                     b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0] b_desc_waddr,
    input  wire [63:0]              b_desc_wdata,

    input  wire                     b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]   b_col_wdata,

    input  wire                     b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]   b_val_wdata,

    // === C dense buffer write (handshake) ===
    output wire                     cbuf_wr_valid,
    input  wire                     cbuf_wr_ready,
    output wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr,
    output wire [`DATA_WIDTH-1:0]   cbuf_wr_data
);

    //=========================================================================
    // A Buffer (reg arrays)
    //=========================================================================
    reg [63:0] A_row_desc_buf [0:`A_ROW_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_val_buf [0:`A_NNZ_SLOT_PER_PE-1];

    always @(posedge aclk) begin
        if (a_desc_we) A_row_desc_buf[a_desc_waddr] <= a_desc_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]       <= a_col_wdata;
        if (a_val_we)  A_val_buf[a_val_waddr]       <= a_val_wdata;
    end

    //=========================================================================
    // B Buffer (reg arrays)
    //=========================================================================
    reg [63:0] B_row_desc_buf [0:`B_ROW_SLOT-1];
    reg [`DATA_WIDTH-1:0] B_col_buf [0:`B_NNZ_SLOT-1];
    reg [`DATA_WIDTH-1:0] B_val_buf [0:`B_NNZ_SLOT-1];

    always @(posedge aclk) begin
        if (b_desc_we) B_row_desc_buf[b_desc_waddr] <= b_desc_wdata;
        if (b_col_we)  B_col_buf[b_col_waddr]       <= b_col_wdata;
        if (b_val_we)  B_val_buf[b_val_waddr]       <= b_val_wdata;
    end

    //=========================================================================
    // FSM State
    //=========================================================================
    localparam PE_IDLE              = 4'd0;
    localparam PE_LOAD_ROW_DESC    = 4'd1;
    localparam PE_CLEAR_ACC        = 4'd2;
    localparam PE_LOAD_A_ELEM      = 4'd3;
    localparam PE_LOAD_B_DESC      = 4'd4;
    localparam PE_STREAM_B_ROW     = 4'd5;
    localparam PE_FLUSH_TASK_PACK  = 4'd6;
    localparam PE_WAIT_TASK_DRAIN  = 4'd7;
    localparam PE_WAIT_PRODUCT_DRAIN = 4'd8;
    localparam PE_WRITE_ROW        = 4'd9;
    localparam PE_NEXT_ROW         = 4'd10;
    localparam PE_DONE             = 4'd11;

    reg [3:0] state, state_next;

    //=========================================================================
    // Row-level registers
    //=========================================================================
    reg comp_sel;                          // ping-pong: 0→acc0 computing, 1→acc1 computing
    reg [`A_ROW_ADDR_BITS-1:0] row_idx;   // local row index (0..row_count-1)
    reg [63:0] row_desc_reg;
    reg [`DATA_WIDTH-1:0] cur_global_row;
    reg [`DATA_WIDTH-1:0] cur_a_row_nnz;
    reg [`OFFSET_WIDTH-1:0] cur_a_start;

    //=========================================================================
    // A iterator registers
    //=========================================================================
    reg [`DATA_WIDTH-1:0] a_nnz_left;
    reg [`OFFSET_WIDTH-1:0] a_ptr;
    reg [`DATA_WIDTH-1:0] cur_k;
    reg [`DATA_WIDTH-1:0] cur_a_val;

    //=========================================================================
    // B streamer registers
    //=========================================================================
    reg [63:0] b_row_desc_reg;
    reg [`OFFSET_WIDTH-1:0] b_ptr;
    reg [`DATA_WIDTH-1:0] b_nnz_left;
    // a_pending removed — intermediate flushes eliminated for MAC throughput

    //=========================================================================
    // Task generation
    //=========================================================================
    wire task_packer_ready;
    wire task_in_valid;
    wire [`TASK_WIDTH-1:0] task_in_data;
    wire task_row_done;  // all tasks for this A row generated

    wire b_batch_done = (state == PE_STREAM_B_ROW) && (b_nnz_left == 0);

    // Task data: {reserved, b_val, a_val, col}
    assign task_in_data = {
        16'd0,
        B_val_buf[b_ptr[`B_NNZ_ADDR_BITS-1:0]],
        cur_a_val,
        B_col_buf[b_ptr[`B_NNZ_ADDR_BITS-1:0]]
    };
    assign task_in_valid = (state == PE_STREAM_B_ROW) &&
                           (b_nnz_left != 0) &&
                           task_packer_ready;

    //=========================================================================
    // B streamer update
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            b_ptr <= 0;
            b_nnz_left <= 0;
        end else if (task_in_valid) begin
            b_ptr <= b_ptr + 1;
            b_nnz_left <= b_nnz_left - 1;
        end
    end

    //=========================================================================
    // Task Packer → Task Group FIFO
    //=========================================================================
    wire task_flush_pack;
    wire task_flush_done;
    wire task_group_wr_en;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data;
    wire task_fifo_full;

    assign task_flush_pack = (state == PE_FLUSH_TASK_PACK);

    pe_task_packer u_task_packer (
        .task_in_valid  (task_in_valid),
        .task_in_ready  (task_packer_ready),
        .task_in_data   (task_in_data),
        .group_wr_en    (task_group_wr_en),
        .group_wr_data  (task_group_wr_data),
        .group_fifo_full(task_fifo_full),
        .flush_pack     (task_flush_pack),
        .flush_done     (task_flush_done),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

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

    wire task_fifo_rd_en;
    wire [`TASK_GROUP_WIDTH-1:0] task_fifo_rd_data;
    wire task_fifo_empty;

    //=========================================================================
    // MAC Array (4-lane)
    //=========================================================================
    wire [`N_MAC-1:0] mac_lane_valid;
    wire [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task;

    // Register task data only when task FIFO is read (avoid stale data loop)
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
            mac_lane_valid_r <= 0;  // clear after one cycle
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

    assign product_group_wr_en = |mul_valid && !product_fifo_full;
    assign product_group_wr_data[3:0]       = mul_valid;
    assign product_group_wr_data[35:4]      = mul_product[0*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[67:36]     = mul_product[1*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[99:68]     = mul_product[2*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[131:100]   = mul_product[3*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];

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
        .count    (),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    wire prod_fifo_rd_en;
    wire [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data;
    wire prod_fifo_empty;

    //=========================================================================
    // 4-bank Row Accumulator — ping-pong pair
    //
    // comp_sel selects which instance is currently accumulating.
    // The other instance drains autonomously; its output drives cbuf_wr.
    //=========================================================================

    // MAC pipeline idle: both registered-valid stage and mul-output stage must be 0
    wire mac_pipeline_idle = !(|mac_lane_valid) && !(|mul_valid);

    // Per-accumulator outputs
    wire        acc_busy_0,        acc_busy_1;
    wire        acc_row_done_0,    acc_row_done_1;
    wire        acc_issue_ready_0, acc_issue_ready_1;
    wire        acc_out_valid_0,   acc_out_valid_1;
    wire [8:0]  acc_out_col_id_0,  acc_out_col_id_1;
    wire [31:0] acc_out_value_0,   acc_out_value_1;
    wire [15:0] acc_out_row_id_0,  acc_out_row_id_1;

    // Muxed compute-side signals (comp_sel accumulator)
    wire acc_issue_ready = comp_sel ? acc_issue_ready_1 : acc_issue_ready_0;
    wire other_acc_busy  = comp_sel ? acc_busy_0        : acc_busy_1;

    // row_start: 1-cycle pulse entering PE_CLEAR_ACC → goes to comp acc only
    wire acc_row_start_0 = (state == PE_CLEAR_ACC) && !comp_sel;
    wire acc_row_start_1 = (state == PE_CLEAR_ACC) &&  comp_sel;

    // row_input_done: fire when prod_fifo empty + MAC idle + drain acc also free.
    // Adding !other_acc_busy ensures we don't start the new row before the
    // previous-previous row has finished draining (2-row pipeline depth = 1 inflight drain).
    wire acc_inp_done = (state == PE_WAIT_PRODUCT_DRAIN)
                      && prod_fifo_empty && mac_pipeline_idle
                      && !other_acc_busy;
    wire acc_inp_done_0 = acc_inp_done && !comp_sel;
    wire acc_inp_done_1 = acc_inp_done &&  comp_sel;

    // issue_valid: only the compute accumulator receives products
    wire issue_valid_0 = !prod_fifo_empty && !comp_sel;
    wire issue_valid_1 = !prod_fifo_empty &&  comp_sel;

    // out_ready: drain accumulator (1-comp_sel) passes cbuf_wr_ready through
    wire acc_out_ready_0 =  comp_sel && cbuf_wr_ready;   // draining when comp_sel=1
    wire acc_out_ready_1 = !comp_sel && cbuf_wr_ready;   // draining when comp_sel=0

    // Consume from product FIFO when compute accumulator is ready
    assign prod_fifo_rd_en = !prod_fifo_empty && acc_issue_ready;

    // Extract 4-lane fields from product FIFO head
    wire [3:0]  acc_lane_valid;
    wire [35:0] acc_lane_col_id;   // 4 * 9 = 36 bits, {lane3..lane0}
    wire [63:0] acc_lane_product;  // 4 * 16 = 64 bits, {lane3..lane0}

    assign acc_lane_valid   = prod_fifo_rd_data[3:0];
    assign acc_lane_col_id  = {prod_fifo_rd_data[4+3*32+16 +: 9],
                               prod_fifo_rd_data[4+2*32+16 +: 9],
                               prod_fifo_rd_data[4+1*32+16 +: 9],
                               prod_fifo_rd_data[4+0*32+16 +: 9]};
    assign acc_lane_product = {prod_fifo_rd_data[4+3*32 +: 16],
                               prod_fifo_rd_data[4+2*32 +: 16],
                               prod_fifo_rd_data[4+1*32 +: 16],
                               prod_fifo_rd_data[4+0*32 +: 16]};

    localparam ACC_PARAMS = 0; // dummy — params listed inline below

    row_accumulator_4bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(`DATA_WIDTH),
        .ACC_W(32), .EPOCH_W(16), .BANK_FIFO_DEPTH(8), .BANK_FIFO_LOG(3), .ROW_W(16)
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
        .ACC_W(32), .EPOCH_W(16), .BANK_FIFO_DEPTH(8), .BANK_FIFO_LOG(3), .ROW_W(16)
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
    // Task FIFO read control (feed MAC when FIFO not empty, product FIFO not full)
    //=========================================================================
    assign task_fifo_rd_en = !task_fifo_empty && !product_fifo_full;

    //=========================================================================
    // Pipeline drain detection
    //=========================================================================
    wire task_drain_done = task_fifo_empty;
    // product drain: prod_fifo drained → accumulator receives row_input_done
    // and handles remaining bank-FIFO RMW internally before row_done asserts.

    //=========================================================================
    // Row writeback — drain acc outputs continuously; no state-gate needed.
    // The draining accumulator (1-comp_sel) drives cbuf_wr at all times.
    // out_row_id from the accumulator carries the correct global row ID.
    //=========================================================================
    wire        drain_out_valid  = comp_sel ? acc_out_valid_0  : acc_out_valid_1;
    wire [8:0]  drain_out_col_id = comp_sel ? acc_out_col_id_0 : acc_out_col_id_1;
    wire [31:0] drain_out_value  = comp_sel ? acc_out_value_0  : acc_out_value_1;
    wire [15:0] drain_out_row_id = comp_sel ? acc_out_row_id_0 : acc_out_row_id_1;

    assign cbuf_wr_valid = drain_out_valid;
    assign cbuf_wr_addr  = (drain_out_row_id * `C_ROW_STRIDE) + drain_out_col_id;
    assign cbuf_wr_data  = drain_out_value[`DATA_WIDTH-1:0];

    // Debug (uncomment for troubleshooting)
    // reg [7:0] dbg_cnt;

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
            comp_sel         <= 1'b0;
            row_idx          <= 0;
            row_desc_reg     <= 0;
            cur_global_row   <= 0;
            cur_a_row_nnz    <= 0;
            cur_a_start      <= 0;
            a_nnz_left       <= 0;
            a_ptr            <= 0;
            cur_k            <= 0;
            cur_a_val        <= 0;
            b_row_desc_reg   <= 0;
            done             <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                PE_IDLE: begin
                    if (start) begin
                        row_idx <= 0;
                    end
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
                    // acc_row_start wire pulses high for this one cycle
                end

                PE_LOAD_A_ELEM: begin
                    cur_k     <= A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
                    cur_a_val <= A_val_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
                end

                PE_LOAD_B_DESC: begin
                    b_row_desc_reg <= B_row_desc_buf[cur_k[`B_ROW_ADDR_BITS-1:0]];
                    b_ptr          <= B_row_desc_buf[cur_k[`B_ROW_ADDR_BITS-1:0]][63:32];
                    b_nnz_left     <= B_row_desc_buf[cur_k[`B_ROW_ADDR_BITS-1:0]][15:0];
                end

                PE_STREAM_B_ROW: begin
                    if (b_nnz_left == 0 && a_nnz_left > 0) begin
                        // Advance to next A element immediately (skip flush for throughput).
                        // Must NOT gate on task_packer_ready: when FIFO is full (ready=0),
                        // b_batch_done still fires and we go to PE_FLUSH_TASK_PACK, but
                        // a_ptr would not advance → same A element re-processed on return.
                        a_ptr      <= a_ptr + 1;
                        a_nnz_left <= a_nnz_left - 1;
                    end
                end

                PE_FLUSH_TASK_PACK: begin
                    // While waiting for flush_done, pre-load the next A element (cycle 1)
                    // and its B row descriptor (cycle 2) so LOAD_A_ELEM + LOAD_B_DESC
                    // states are skipped for intermediate A elements.
                    if (!state_stable && a_nnz_left > 0) begin
                        cur_k     <= A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
                        cur_a_val <= A_val_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
                    end
                    if (state_stable && a_nnz_left > 0) begin
                        b_ptr      <= B_row_desc_buf[cur_k[`B_ROW_ADDR_BITS-1:0]][63:32];
                        b_nnz_left <= B_row_desc_buf[cur_k[`B_ROW_ADDR_BITS-1:0]][15:0];
                    end
                end

                PE_WAIT_TASK_DRAIN: begin
                    // Wait for task_fifo_empty
                end

                PE_WAIT_PRODUCT_DRAIN: begin
                    // Nothing needed: acc_inp_done fires combinationally when
                    // prod_fifo_empty && mac_pipeline_idle && !other_acc_busy.
                end

                PE_NEXT_ROW: begin
                    if (!state_stable) begin
                        row_idx  <= row_idx + 1;
                        comp_sel <= ~comp_sel;  // switch accumulator for next row
                    end
                end

                PE_DONE: begin
                    // Wait for the last row's drain accumulator to finish
                    // before asserting done; otherwise the final row's writes
                    // are lost because the testbench stops capturing at done.
                    if (!other_acc_busy) done <= 1'b1;
                end

            endcase
        end
    end

    //=========================================================================
    // state_stable: ensures fast states stay at least 1 cycle for NB reads
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
                // always pass through PE_CLEAR_ACC to issue row_start pulse
                if (state_stable) state_next = PE_CLEAR_ACC;

            PE_CLEAR_ACC:
                // row_start pulses for this cycle; immediately advance
                if (cur_a_row_nnz == 0)
                    state_next = PE_WAIT_PRODUCT_DRAIN;  // empty row
                else
                    state_next = PE_LOAD_A_ELEM;

            PE_LOAD_A_ELEM:
                if (state_stable) state_next = PE_LOAD_B_DESC;

            PE_LOAD_B_DESC:
                if (state_stable) state_next = PE_STREAM_B_ROW;

            PE_STREAM_B_ROW:
                if (b_batch_done) state_next = PE_FLUSH_TASK_PACK;

            PE_FLUSH_TASK_PACK:
                if (task_flush_done) begin
                    if (a_nnz_left == 0)  // last A element → full drain
                        state_next = PE_WAIT_TASK_DRAIN;
                    else  // A+B pre-loaded during flush; skip LOAD_A_ELEM + LOAD_B_DESC
                        state_next = PE_STREAM_B_ROW;
                end

            PE_WAIT_TASK_DRAIN:
                if (task_drain_done) state_next = PE_WAIT_PRODUCT_DRAIN;

            PE_WAIT_PRODUCT_DRAIN:
                // Also wait for drain acc (other_acc_busy) to finish its previous
                // row before we can reuse it as the next compute accumulator.
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

endmodule
