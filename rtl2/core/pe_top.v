//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : PE Top — 4-wide B streaming, synchronous BRAM reads.
//            A_col/val: internal BRAM (per-PE, independent).
//            B_col/val: external shared buffer via handshake (b_shared_buffer).
//            B_row_desc: kept internal (512×64b LUT-RAM, small).
//
//   FSM (11 states):
//     PE_IDLE → PE_LOAD_ROW_DESC → PE_CLEAR_ACC
//     → PE_LOAD_A_ELEM → PE_LOAD_B_SETUP → PE_STREAM_B_ROW
//     → PE_WAIT_TASK_DRAIN → PE_WAIT_PRODUCT_DRAIN
//     → PE_NEXT_ROW → PE_DONE
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

    // External B data interface (→ b_shared_buffer)
    output wire                     ext_b_req,
    output wire [`B_NNZ_ADDR_BITS-3:0] ext_b_group,
    input  wire                     ext_b_rdy,
    input  wire [`DATA_WIDTH-1:0]   ext_bc0,
    input  wire [`DATA_WIDTH-1:0]   ext_bc1,
    input  wire [`DATA_WIDTH-1:0]   ext_bc2,
    input  wire [`DATA_WIDTH-1:0]   ext_bc3,
    input  wire [`DATA_WIDTH-1:0]   ext_bv0,
    input  wire [`DATA_WIDTH-1:0]   ext_bv1,
    input  wire [`DATA_WIDTH-1:0]   ext_bv2,
    input  wire [`DATA_WIDTH-1:0]   ext_bv3,

    // B_desc write port (from b_broadcast_loader — kept per-PE, not shared)
    input  wire                     b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0] b_desc_waddr,
    input  wire [63:0]              b_desc_wdata,

    // C dense buffer write (handshake)
    output wire                     cbuf_wr_valid,
    input  wire                     cbuf_wr_ready,
    output wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr,
    output wire [`DATA_WIDTH-1:0]   cbuf_wr_data
);

    //=========================================================================
    // A Buffer (per-PE, independent)
    //   Write port + dedicated continuous read port for clean BRAM inference.
    //=========================================================================
    reg [63:0]            A_row_desc_buf [0:`A_ROW_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf      [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_val_buf      [0:`A_NNZ_SLOT_PER_PE-1];

    // Write port (load phase only)
    always @(posedge aclk) begin
        if (a_desc_we) A_row_desc_buf[a_desc_waddr] <= a_desc_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]       <= a_col_wdata;
        if (a_val_we)  A_val_buf[a_val_waddr]       <= a_val_wdata;
    end

    //=========================================================================
    // Continuous Read Ports — pure BRAM template, reads every cycle
    //   The read address registers (row_idx, a_ptr, cur_k) are stable
    //   before the FSM needs the value, so the pipelined read is always
    //   valid by the time the consuming state is entered.
    //=========================================================================
    reg [63:0]            a_desc_rd;
    reg [`DATA_WIDTH-1:0] a_col_rd, a_val_rd;
    reg [63:0]            b_desc_rd;

    always @(posedge aclk) begin
        a_desc_rd <= A_row_desc_buf[row_idx];
        a_col_rd  <= A_col_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
        a_val_rd  <= A_val_buf[a_ptr[`A_NNZ_ADDR_BITS-1:0]];
        b_desc_rd <= B_row_desc_buf[cur_k[`B_ROW_ADDR_BITS-1:0]];
    end

    //=========================================================================
    // B_row_desc (per-PE, small: 512×64b → LUT-RAM, kept internal)
    //=========================================================================
    reg [63:0] B_row_desc_buf [0:`B_ROW_SLOT-1];

    always @(posedge aclk) begin
        if (b_desc_we)
            B_row_desc_buf[b_desc_waddr] <= b_desc_wdata;
    end

    //=========================================================================
    // FSM States
    //=========================================================================
    localparam PE_IDLE               = 4'd0;
    localparam PE_LOAD_ROW_DESC      = 4'd1;
    localparam PE_CLEAR_ACC          = 4'd2;
    localparam PE_LOAD_A_ELEM        = 4'd3;
    localparam PE_LOAD_B_SETUP       = 4'd10;
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
    wire [`A_NNZ_ADDR_BITS-1:0] a_ptr_next = a_ptr[`A_NNZ_ADDR_BITS-1:0] + 1'b1;
    reg [`DATA_WIDTH-1:0]   cur_k;       // registered A_col (sync from BRAM)
    reg [`DATA_WIDTH-1:0]   cur_a_val;   // registered A_val (sync from BRAM)

    //=========================================================================
    // B streamer registers
    //=========================================================================
    reg [63:0]            b_row_desc_reg;
    reg [`OFFSET_WIDTH-1:0] b_ptr;
    reg [`DATA_WIDTH-1:0]   b_nnz_left;

    //=========================================================================
    // B bank read — external interface (replaces internal B_col/val arrays)
    //
    //   Timing: ext_b_req + ext_b_group issued in cycle N
    //           ext_b_rdy + ext_bc0..bv3 valid in cycle N+1
    //           If ext_b_rdy = 0 (port conflict), PE stalls 1 cycle.
    //=========================================================================
    wire [`B_NNZ_ADDR_BITS-3:0] b_group = b_ptr[`B_NNZ_ADDR_BITS-1:2];

    // Request to shared B buffer
    wire b_bank_rd_en = (state == PE_LOAD_B_SETUP) || (state == PE_STREAM_B_ROW);
    assign ext_b_req   = b_bank_rd_en;
    assign ext_b_group = b_group;

    // Pipeline registers for B data (from external, 1 cycle after request)
    reg [`DATA_WIDTH-1:0] bc0_r, bc1_r, bc2_r, bc3_r;
    reg [`DATA_WIDTH-1:0] bv0_r, bv1_r, bv2_r, bv3_r;
    reg ext_b_rdy_r;   // 1 = data in bc0_r..bv3_r is valid this cycle

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            bc0_r <= 0; bc1_r <= 0; bc2_r <= 0; bc3_r <= 0;
            bv0_r <= 0; bv1_r <= 0; bv2_r <= 0; bv3_r <= 0;
            ext_b_rdy_r <= 1'b0;
        end else begin
            // Latch external data every cycle it might arrive
            if (ext_b_rdy) begin
                bc0_r <= ext_bc0; bc1_r <= ext_bc1;
                bc2_r <= ext_bc2; bc3_r <= ext_bc3;
                bv0_r <= ext_bv0; bv1_r <= ext_bv1;
                bv2_r <= ext_bv2; bv3_r <= ext_bv3;
            end
            ext_b_rdy_r <= ext_b_rdy;
        end
    end

    //=========================================================================
    // lane_valid pipeline — align with 1-cycle delayed B bank data
    //=========================================================================
    wire [3:0] stream_lane_valid;
    assign stream_lane_valid[0] = (b_nnz_left >= 16'd1);
    assign stream_lane_valid[1] = (b_nnz_left >= 16'd2);
    assign stream_lane_valid[2] = (b_nnz_left >= 16'd3);
    assign stream_lane_valid[3] = (b_nnz_left >= 16'd4);

    reg [3:0] lane_valid_r;
    always @(posedge aclk) begin
        if (b_bank_rd_en)
            lane_valid_r <= stream_lane_valid;
        else
            lane_valid_r <= 4'd0;
    end

    reg b_last_group_r;
    always @(posedge aclk) begin
        if (b_bank_rd_en)
            b_last_group_r <= (b_nnz_left <= 16'd4);
        else
            b_last_group_r <= 1'b0;
    end

    //=========================================================================
    // Task Group generation — uses registered B data (bc0_r..bv3_r)
    //=========================================================================
    wire [`TASK_WIDTH-1:0] sg0 = {16'd0, bv0_r, cur_a_val, bc0_r};
    wire [`TASK_WIDTH-1:0] sg1 = {16'd0, bv1_r, cur_a_val, bc1_r};
    wire [`TASK_WIDTH-1:0] sg2 = {16'd0, bv2_r, cur_a_val, bc2_r};
    wire [`TASK_WIDTH-1:0] sg3 = {16'd0, bv3_r, cur_a_val, bc3_r};

    wire task_fifo_full;
    wire stream_group_valid = (state == PE_STREAM_B_ROW) && ext_b_rdy_r && (|lane_valid_r);
    wire task_group_wr_en   = stream_group_valid && !task_fifo_full;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data = {sg3, sg2, sg1, sg0, lane_valid_r};

    wire b_last_group_fires = task_group_wr_en && b_last_group_r;
    wire b_batch_done = (state == PE_STREAM_B_ROW)
                     && (ext_b_rdy_r && (!(|lane_valid_r) || b_last_group_fires));

    //=========================================================================
    // Stall detection — if ext_b_rdy=0 but we're waiting for B data
    //=========================================================================
    wire b_stall = (state == PE_STREAM_B_ROW) && !ext_b_rdy_r && (b_nnz_left != 0);

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
    assign task_fifo_rd_en = !task_fifo_empty &&
                             (product_fifo_cnt < (`PROD_FIFO_DEPTH - `MUL_LAT - 1));

    //=========================================================================
    // Row writeback
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
            done           <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                PE_IDLE: begin
                    if (start) row_idx <= 0;
                end

                PE_LOAD_ROW_DESC: begin
                    row_desc_reg   <= a_desc_rd;
                    cur_global_row <= a_desc_rd[15:0];
                    cur_a_row_nnz  <= a_desc_rd[31:16];
                    cur_a_start    <= a_desc_rd[63:32];
                    a_ptr          <= a_desc_rd[63:32];
                    a_nnz_left     <= a_desc_rd[31:16];
                end

                PE_CLEAR_ACC: ;

                PE_LOAD_A_ELEM: begin
                    cur_k     <= a_col_rd;
                    cur_a_val <= a_val_rd;
                end

                PE_LOAD_B_SETUP: begin
                    b_row_desc_reg <= b_desc_rd;
                    b_ptr      <= b_desc_rd[63:32];
                    b_nnz_left <= b_desc_rd[15:0];
                end

                PE_STREAM_B_ROW: begin
                    if (!b_stall) begin
                        if (!task_fifo_full && ext_b_rdy_r && (|lane_valid_r)) begin
                            if (b_last_group_r) begin
                                a_ptr      <= a_ptr + 1;
                                a_nnz_left <= a_nnz_left - 1;
                            end else begin
                                b_ptr      <= b_ptr + 4;
                                b_nnz_left <= b_nnz_left - 16'd4;
                            end
                        end else if (!(|lane_valid_r) && a_nnz_left > 0) begin
                            a_ptr      <= a_ptr + 1;
                            a_nnz_left <= a_nnz_left - 1;
                        end
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
    // state_stable
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
                    state_next = PE_WAIT_PRODUCT_DRAIN;
                else
                    state_next = PE_LOAD_A_ELEM;

            PE_LOAD_A_ELEM:
                state_next = PE_LOAD_B_SETUP;

            PE_LOAD_B_SETUP: begin
                if (b_nnz_left == 0)
                    state_next = PE_LOAD_A_ELEM;
                else
                    state_next = PE_STREAM_B_ROW;
            end

            PE_STREAM_B_ROW: begin
                if (b_stall)
                    state_next = PE_STREAM_B_ROW;  // stall
                else if (b_batch_done) begin
                    if (a_nnz_left <= 1)
                        state_next = PE_WAIT_TASK_DRAIN;
                    else
                        state_next = PE_LOAD_A_ELEM;
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

            PE_DONE: ;

            default: state_next = PE_IDLE;
        endcase
    end

`ifdef SIMULATION
    wire [15:0] _tc0 = task_group_wr_data[19:4];
    wire [15:0] _tc1 = task_group_wr_data[83:68];
    wire [15:0] _tc2 = task_group_wr_data[147:132];
    wire [15:0] _tc3 = task_group_wr_data[211:196];
    wire _tg_hit = (_tc0==53||_tc0==56||_tc0==69||_tc0==73||_tc0==30||
                    _tc1==53||_tc1==56||_tc1==69||_tc1==73||_tc1==30||
                    _tc2==53||_tc2==56||_tc2==69||_tc2==73||_tc2==30||
                    _tc3==53||_tc3==56||_tc3==69||_tc3==73||_tc3==30);

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
        if ((|mul_valid) && product_fifo_full)
            $display("[PROD_DROP @%0t] mul_valid=%b pf_cnt=%0d", $time, mul_valid, product_fifo_cnt);

        if (task_group_wr_en && _tg_hit)
            $display("[TG_WR @%0t row=%0d] valid=%b cols=%0d/%0d/%0d/%0d bnz=%0d",
                $time, row_idx, lane_valid_r,
                _tc0, _tc1, _tc2, _tc3, b_nnz_left);

        if (prod_fifo_rd_en && _pr_hit)
            $display("[PRD_RD @%0t row=%0d comp=%0d] valid=%b c0=%0d(v=%0d) c1=%0d(v=%0d) c2=%0d(v=%0d) c3=%0d(v=%0d) acc_rdy=%b",
                $time, row_idx, comp_sel, prod_fifo_rd_data[3:0],
                _pc0, _pv0, _pc1, _pv1, _pc2, _pv2, _pc3, _pv3, acc_issue_ready);

        if (cbuf_wr_valid && cbuf_wr_ready &&
            ((drain_out_row_id==2 && (drain_out_col_id==53||drain_out_col_id==56||
                                      drain_out_col_id==69||drain_out_col_id==73)) ||
             (drain_out_row_id==6 && drain_out_col_id==30)))
            $display("[CBUF_WR @%0t] row=%0d col=%0d val=%0d",
                $time, drain_out_row_id, drain_out_col_id, drain_out_value);

        if (b_stall)
            $display("[B_STALL @%0t row=%0d PE=%0d] b_ptr=%0d b_nnz=%0d",
                $time, row_idx, PE_ID, b_ptr, b_nnz_left);
    end
`endif

endmodule
