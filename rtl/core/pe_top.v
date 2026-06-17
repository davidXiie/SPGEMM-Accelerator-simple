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
    // Product Serializer
    //=========================================================================
    wire acc_in_valid;
    wire acc_in_ready;
    wire [`PRODUCT_WIDTH-1:0] acc_in_data;
    wire serializer_idle;

    pe_serializer u_serializer (
        .prod_fifo_empty   (prod_fifo_empty),
        .prod_fifo_rd_en   (prod_fifo_rd_en),
        .prod_fifo_rd_data (prod_fifo_rd_data),
        .acc_in_valid      (acc_in_valid),
        .acc_in_ready      (acc_in_ready),
        .acc_in_data       (acc_in_data),
        .idle              (serializer_idle),
        .aclk              (aclk),
        .aresetn           (aresetn)
    );

    //=========================================================================
    // Accumulator
    //=========================================================================
    wire acc_idle;
    wire acc_drain_empty;
    wire acc_clear_done;
    wire acc_clear_en;

    wire [`PE_ACC_ADDR_BITS-1:0] wb_rd_addr;
    wire [`DATA_WIDTH-1:0]       wb_rd_data;

    assign acc_clear_en = (state == PE_CLEAR_ACC);

    pe_accumulator u_accumulator (
        .acc_in_valid    (acc_in_valid),
        .acc_in_ready    (acc_in_ready),
        .acc_in_data     (acc_in_data),
        .idle            (acc_idle),
        .all_drain_empty (acc_drain_empty),
        .clear_en        (acc_clear_en),
        .clear_done      (acc_clear_done),
        .N               (N),
        .wb_rd_addr      (wb_rd_addr),
        .wb_rd_data      (wb_rd_data),
        .aclk            (aclk),
        .aresetn         (aresetn)
    );

    //=========================================================================
    // Task FIFO read control (feed MAC when FIFO not empty, product FIFO not full)
    //=========================================================================
    assign task_fifo_rd_en = !task_fifo_empty && !product_fifo_full;

    //=========================================================================
    // Pipeline drain detection
    //=========================================================================
    wire task_drain_done;
    wire product_drain_done;

    assign task_drain_done    = task_fifo_empty;
    assign product_drain_done = prod_fifo_empty && serializer_idle && acc_idle;

    //=========================================================================
    // Row writeback
    //=========================================================================
    reg [`MAX_DIM_BITS-1:0] write_col;
    reg [`MAX_DIM_BITS-1:0] write_global_row;

    assign cbuf_wr_valid = (state == PE_WRITE_ROW);
    assign cbuf_wr_addr  = (write_global_row * `C_ROW_STRIDE) + write_col;
    assign cbuf_wr_data  = wb_rd_data;
    assign wb_rd_addr    = write_col[`PE_ACC_ADDR_BITS-1:0];

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
            write_col      <= 0;
            write_global_row <= 0;
            done           <= 1'b0;
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
                    // acc_clear_en drives accumulator clear
                    // wait for clear_done
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
                    if (b_nnz_left == 0 && a_nnz_left > 0 && task_packer_ready) begin
                        // Advance to next A element immediately (skip flush for throughput)
                        a_ptr      <= a_ptr + 1;
                        a_nnz_left <= a_nnz_left - 1;
                    end
                end

                PE_FLUSH_TASK_PACK: begin
                    // Final flush at end of row only (intermediate flushes removed)
                end

                PE_WAIT_TASK_DRAIN: begin
                    // Wait for task_fifo_empty
                end

                PE_WAIT_PRODUCT_DRAIN: begin
                    if (product_drain_done)
                        write_global_row <= cur_global_row;
                end

                PE_WRITE_ROW: begin
                    if (cbuf_wr_valid && cbuf_wr_ready) begin
                        write_col <= write_col + 1'b1;
                    end
                end

                PE_NEXT_ROW: begin
                    write_col <= 0;
                    if (!state_stable)
                        row_idx <= row_idx + 1;
                end

                PE_DONE: begin
                    done <= 1'b1;
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
                if (state_stable) begin
                    if (A_row_desc_buf[row_idx][31:16] == 0)
                        state_next = PE_WRITE_ROW;  // empty row
                    else
                        state_next = PE_CLEAR_ACC;
                end

            PE_CLEAR_ACC:
                if (acc_clear_done) state_next = PE_LOAD_A_ELEM;

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
                    else  // intermediate: flush packer only, no drain
                        state_next = PE_LOAD_A_ELEM;
                end

            PE_WAIT_TASK_DRAIN:
                if (task_drain_done) state_next = PE_WAIT_PRODUCT_DRAIN;

            PE_WAIT_PRODUCT_DRAIN:
                if (product_drain_done)
                    state_next = (a_nnz_left > 0) ? PE_LOAD_A_ELEM : PE_WRITE_ROW;

            PE_WRITE_ROW:
                if (write_col == N - 1 && cbuf_wr_ready)
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
