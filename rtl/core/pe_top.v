//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator v2  (inst-version)
// Brief    : PE Top — address-based instruction scheduling.
//
//   A_val_buf and B_col/val_b0..b3 hold raw A/B data in SRAM.
//   instr_buf holds a pre-computed (b_group, a_val_ptr, lane_valid) schedule.
//
//   Instruction format (64-bit):
//     [63:33] reserved
//     [32:18] b_group      index into B_col/val bank arrays  (15-bit)
//     [17: 4] a_val_ptr    index into A_val_buf              (14-bit)
//     [ 3: 0] lane_valid   active MAC lanes                  ( 4-bit)
//
//   Row descriptor stored in A_row_desc_buf (64-bit):
//     [63:32] instr_start  absolute start index into instr_buf
//     [31:16] instr_count  number of instruction groups for this row
//     [15: 0] c_row        global C output row id (for host readback only)
//
//   C buffer: 4-bank internal SRAM, banked by col[1:0].
//     Each PE owns its slice; drain writes 4 values simultaneously per cycle.
//     Read port c_rd_addr = {local_row_idx[7:0], col[8:0]} (17-bit).
//
//   FSM (8 states):
//     PE_IDLE → PE_LOAD_ROW_DESC → PE_CLEAR_ACC → PE_STREAM_INSTRS
//     → PE_WAIT_TASK_DRAIN → PE_WAIT_PRODUCT_DRAIN → PE_NEXT_ROW → PE_DONE
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

    // Row descriptor load  {instr_start[31:0], instr_count[15:0], c_row[15:0]}
    input  wire                          a_desc_we,
    input  wire [`A_ROW_ADDR_BITS-1:0]   a_desc_waddr,
    input  wire [63:0]                   a_desc_wdata,

    // A value buffer
    input  wire                          a_val_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata,

    // B col/val buffers (4-banked by absolute index[1:0])
    input  wire                          b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_col_wdata,
    input  wire                          b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_val_wdata,

    // Instruction buffer
    input  wire                          instr_we,
    input  wire [`INSTR_ADDR_BITS-1:0]  instr_waddr,
    input  wire [127:0]                  instr_wdata,

    // C buffer read port (synchronous, 1-cycle latency)
    // c_rd_addr = {local_row_idx[A_ROW_ADDR_BITS-1:0], col[8:0]}  (17-bit)
    input  wire                          c_rd_en,
    input  wire [16:0]                   c_rd_addr,
    output reg  [31:0]                   c_rd_data   // FP32 accumulator output
);

    localparam ACC_COL_W     = 9;                                  // log2(OUT_COLS=512)
    localparam BANK_ADDR_W   = ACC_COL_W - 2;                     // 7 bits (groups 0..127)
    localparam C_BANK_ADDR_W = `A_ROW_ADDR_BITS + BANK_ADDR_W;   // 15 bits
    localparam C_BANK_DEPTH  = 1 << C_BANK_ADDR_W;                // 32768
    localparam C_RD_ADDR_W   = `A_ROW_ADDR_BITS + ACC_COL_W;     // 17 bits

    //=========================================================================
    // SRAM declarations
    //=========================================================================
    reg [63:0]            A_row_desc_buf [0:`A_ROW_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_val_buf      [0:`A_NNZ_SLOT_PER_PE-1];

    localparam B_BANK_DEPTH = `B_NNZ_SLOT / 4;
    reg [`DATA_WIDTH-1:0] B_col_b0 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b1 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b2 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b3 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b0 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b1 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b2 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b3 [0:B_BANK_DEPTH-1];

    reg [127:0]           instr_buf [0:`INSTR_SLOT-1];

    // Per-PE banked C buffer: banked by col[1:0], stores FP32 (32-bit)
    // Address: {local_row_idx[7:0], col_group[6:0]}  (15-bit)
    reg [31:0] c_bank0 [0:C_BANK_DEPTH-1];
    reg [31:0] c_bank1 [0:C_BANK_DEPTH-1];
    reg [31:0] c_bank2 [0:C_BANK_DEPTH-1];
    reg [31:0] c_bank3 [0:C_BANK_DEPTH-1];

`ifdef COCOTB_SIM
    integer _ci;
    initial begin
        c_rd_data = 32'h0;  // prevent X in packed c_rd_data bus (32-bit FP32 per PE)
        for (_ci = 0; _ci < C_BANK_DEPTH; _ci = _ci + 1) begin
            c_bank0[_ci] = 32'h0; c_bank1[_ci] = 32'h0;
            c_bank2[_ci] = 32'h0; c_bank3[_ci] = 32'h0;
        end
    end
`endif

    //=========================================================================
    // SRAM write ports
    //=========================================================================
    always @(posedge aclk) begin
        if (a_desc_we) A_row_desc_buf[a_desc_waddr] <= a_desc_wdata;
        if (a_val_we)  A_val_buf[a_val_waddr]        <= a_val_wdata;
        if (instr_we)  instr_buf[instr_waddr]         <= instr_wdata;
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
    // FSM states
    //=========================================================================
    localparam PE_IDLE               = 3'd0;
    localparam PE_LOAD_ROW_DESC      = 3'd1;
    localparam PE_CLEAR_ACC          = 3'd2;
    localparam PE_STREAM_INSTRS      = 3'd3;
    localparam PE_WAIT_TASK_DRAIN    = 3'd4;
    localparam PE_WAIT_PRODUCT_DRAIN = 3'd5;
    localparam PE_NEXT_ROW           = 3'd6;
    localparam PE_DONE               = 3'd7;

    reg [2:0] state, state_next;

    //=========================================================================
    // Row-level registers
    //=========================================================================
    reg comp_sel;
    reg [`A_ROW_ADDR_BITS-1:0] row_idx;
    reg [31:0]                 cur_instr_start;
    reg [15:0]                 cur_instr_count;
    reg [15:0]                 instr_ptr;

    //=========================================================================
    // Instruction decode & task group formation (combinatorial)
    //=========================================================================
    wire [`INSTR_ADDR_BITS-1:0] instr_raddr =
        cur_instr_start[`INSTR_ADDR_BITS-1:0] + instr_ptr[`INSTR_ADDR_BITS-1:0];

    // Instruction format (128-bit, 4 × 32-bit per-lane words):
    //   Lane k word occupies bits [k*32+31 : k*32]:
    //     [31:16] a_val_fp16 — FP16 A-value embedded directly (no A_val_buf read)
    //     [15: 1] b_group    — B bank address = abs_B_pos / 4; lane k reads bank k
    //     [    0] valid
    wire [127:0] cur_instr    = instr_buf[instr_raddr];
    wire [3:0]   lane_valid_w = {cur_instr[96], cur_instr[64], cur_instr[32], cur_instr[0]};

    // Per-lane FP16 A values (embedded in instruction)
    wire [`DATA_WIDTH-1:0] a_val_0 = cur_instr[ 31:16];
    wire [`DATA_WIDTH-1:0] a_val_1 = cur_instr[ 63:48];
    wire [`DATA_WIDTH-1:0] a_val_2 = cur_instr[ 95:80];
    wire [`DATA_WIDTH-1:0] a_val_3 = cur_instr[127:112];

    // Per-lane B bank addresses — all 4 B SRAMs addressed independently
    wire [14:0] bg0 = cur_instr[ 15: 1];
    wire [14:0] bg1 = cur_instr[ 47:33];
    wire [14:0] bg2 = cur_instr[ 79:65];
    wire [14:0] bg3 = cur_instr[111:97];

    wire [`DATA_WIDTH-1:0] bc0 = B_col_b0[bg0];
    wire [`DATA_WIDTH-1:0] bv0 = B_val_b0[bg0];
    wire [`DATA_WIDTH-1:0] bc1 = B_col_b1[bg1];
    wire [`DATA_WIDTH-1:0] bv1 = B_val_b1[bg1];
    wire [`DATA_WIDTH-1:0] bc2 = B_col_b2[bg2];
    wire [`DATA_WIDTH-1:0] bv2 = B_val_b2[bg2];
    wire [`DATA_WIDTH-1:0] bc3 = B_col_b3[bg3];
    wire [`DATA_WIDTH-1:0] bv3 = B_val_b3[bg3];

    wire [`TASK_WIDTH-1:0] sg0 = {16'd0, bv0, a_val_0, bc0};
    wire [`TASK_WIDTH-1:0] sg1 = {16'd0, bv1, a_val_1, bc1};
    wire [`TASK_WIDTH-1:0] sg2 = {16'd0, bv2, a_val_2, bc2};
    wire [`TASK_WIDTH-1:0] sg3 = {16'd0, bv3, a_val_3, bc3};

    wire task_fifo_full;
    wire instr_active      = (state == PE_STREAM_INSTRS) && (instr_ptr < cur_instr_count);
    wire task_group_wr_en  = instr_active && !task_fifo_full;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data = {sg3, sg2, sg1, sg0, lane_valid_w};

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
    wire [`N_MAC-1:0]           mac_lane_valid;
    wire [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task;

    reg [`N_MAC-1:0]            mac_lane_valid_r;
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

    wire [`N_MAC-1:0]               mul_valid;
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

    assign product_group_wr_en            = |mul_valid && !product_fifo_full;
    assign product_group_wr_data[3:0]     = mul_valid;
    // Each product: 48 bits = {col_id[15:0], fp32_val[31:0]}
    assign product_group_wr_data[51:4]    = mul_product[0*48 +: 48];
    assign product_group_wr_data[99:52]   = mul_product[1*48 +: 48];
    assign product_group_wr_data[147:100] = mul_product[2*48 +: 48];
    assign product_group_wr_data[195:148] = mul_product[3*48 +: 48];

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
    // Ping-pong Row Accumulator
    //=========================================================================
    wire mac_pipeline_idle = !(|mac_lane_valid) && !(|mul_valid);

    wire acc_busy_0,        acc_busy_1;
    wire acc_row_done_0,    acc_row_done_1;
    wire acc_issue_ready_0, acc_issue_ready_1;

    // 4-wide drain outputs from each accumulator
    wire [3:0]             drain_valid_0,  drain_valid_1;
    wire [BANK_ADDR_W-1:0] drain_gaddr_0,  drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] drain_row_id_0, drain_row_id_1;
    wire [4*32-1:0]        drain_values_0, drain_values_1;

    wire acc_issue_ready = comp_sel ? acc_issue_ready_1 : acc_issue_ready_0;
    wire other_acc_busy  = comp_sel ? acc_busy_0        : acc_busy_1;

    wire acc_row_start_0 = (state == PE_CLEAR_ACC) && !comp_sel;
    wire acc_row_start_1 = (state == PE_CLEAR_ACC) &&  comp_sel;

    wire acc_inp_done    = (state == PE_WAIT_PRODUCT_DRAIN)
                         && prod_fifo_empty && mac_pipeline_idle && !other_acc_busy;
    wire acc_inp_done_0  = acc_inp_done && !comp_sel;
    wire acc_inp_done_1  = acc_inp_done &&  comp_sel;

    wire issue_valid_0 = !prod_fifo_empty && !comp_sel;
    wire issue_valid_1 = !prod_fifo_empty &&  comp_sel;

    assign prod_fifo_rd_en = !prod_fifo_empty && acc_issue_ready;

    wire [3:0]   acc_lane_valid;
    wire [35:0]  acc_lane_col_id;
    wire [127:0] acc_lane_product;  // 4 × 32-bit FP32 products

    assign acc_lane_valid   = prod_fifo_rd_data[3:0];
    // Each lane's product: bits [31:0]=fp32_val, bits [47:32]=col_id
    assign acc_lane_col_id  = {prod_fifo_rd_data[4+3*48+32 +: 9],  // lane3 col_id[8:0]
                               prod_fifo_rd_data[4+2*48+32 +: 9],
                               prod_fifo_rd_data[4+1*48+32 +: 9],
                               prod_fifo_rd_data[4+0*48+32 +: 9]};
    assign acc_lane_product = {prod_fifo_rd_data[4+3*48 +: 32],    // lane3 fp32_val
                               prod_fifo_rd_data[4+2*48 +: 32],
                               prod_fifo_rd_data[4+1*48 +: 32],
                               prod_fifo_rd_data[4+0*48 +: 32]};

    // row_id_in = local row_idx (used as C buffer row address)
    row_accumulator_4bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(32),
        .ACC_W(32), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5),
        .ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_0 (
        .clk(aclk), .rst_n(aresetn),
        .row_start(acc_row_start_0), .row_id_in(row_idx), .drain_cols(N),
        .row_input_done(acc_inp_done_0), .busy(acc_busy_0), .row_done(acc_row_done_0),
        .issue_valid(issue_valid_0), .issue_ready(acc_issue_ready_0),
        .lane_valid(acc_lane_valid), .lane_col_id(acc_lane_col_id), .lane_product(acc_lane_product),
        .drain_valid(drain_valid_0), .drain_gaddr(drain_gaddr_0),
        .drain_row_id(drain_row_id_0), .drain_values(drain_values_0)
    );

    row_accumulator_4bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(32),
        .ACC_W(32), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5),
        .ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_1 (
        .clk(aclk), .rst_n(aresetn),
        .row_start(acc_row_start_1), .row_id_in(row_idx), .drain_cols(N),
        .row_input_done(acc_inp_done_1), .busy(acc_busy_1), .row_done(acc_row_done_1),
        .issue_valid(issue_valid_1), .issue_ready(acc_issue_ready_1),
        .lane_valid(acc_lane_valid), .lane_col_id(acc_lane_col_id), .lane_product(acc_lane_product),
        .drain_valid(drain_valid_1), .drain_gaddr(drain_gaddr_1),
        .drain_row_id(drain_row_id_1), .drain_values(drain_values_1)
    );

    //=========================================================================
    // Task FIFO read control
    //=========================================================================
    assign task_fifo_rd_en = !task_fifo_empty &&
                             (product_fifo_cnt < (`PROD_FIFO_DEPTH - `MUL_LAT - 1));

    //=========================================================================
    // C buffer write — 4 banks updated simultaneously per drain group
    // comp_sel=0: acc0 computing, acc1 draining → drain from acc1
    // comp_sel=1: acc1 computing, acc0 draining → drain from acc0
    //=========================================================================
    wire [3:0]               c_wr_valid  = comp_sel ? drain_valid_0  : drain_valid_1;
    wire [BANK_ADDR_W-1:0]   c_wr_gaddr  = comp_sel ? drain_gaddr_0  : drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] c_wr_rid = comp_sel ? drain_row_id_0 : drain_row_id_1;
    wire [4*32-1:0]          c_wr_vals   = comp_sel ? drain_values_0 : drain_values_1;

    wire [C_BANK_ADDR_W-1:0] c_wr_addr = {c_wr_rid, c_wr_gaddr};

    always @(posedge aclk) begin
        if (c_wr_valid[0]) c_bank0[c_wr_addr] <= c_wr_vals[0*32 +: 32];
        if (c_wr_valid[1]) c_bank1[c_wr_addr] <= c_wr_vals[1*32 +: 32];
        if (c_wr_valid[2]) c_bank2[c_wr_addr] <= c_wr_vals[2*32 +: 32];
        if (c_wr_valid[3]) c_bank3[c_wr_addr] <= c_wr_vals[3*32 +: 32];
    end

    //=========================================================================
    // C buffer read (synchronous, 1-cycle latency)
    // c_rd_addr = {local_row_idx[A_ROW_ADDR_BITS-1:0], col[ACC_COL_W-1:0]}
    //=========================================================================
    wire [1:0]               rd_bank  = c_rd_addr[1:0];
    wire [C_BANK_ADDR_W-1:0] rd_baddr = {c_rd_addr[C_RD_ADDR_W-1:ACC_COL_W],
                                          c_rd_addr[ACC_COL_W-1:2]};

    always @(posedge aclk) begin
        if (c_rd_en) begin
            case (rd_bank)
                2'd0: c_rd_data <= c_bank0[rd_baddr];
                2'd1: c_rd_data <= c_bank1[rd_baddr];
                2'd2: c_rd_data <= c_bank2[rd_baddr];
                2'd3: c_rd_data <= c_bank3[rd_baddr];
            endcase
        end
    end

    //=========================================================================
    // FSM sequential
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) state <= PE_IDLE;
        else          state <= state_next;
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            comp_sel        <= 1'b0;
            row_idx         <= 0;
            cur_instr_start <= 0;
            cur_instr_count <= 0;
            instr_ptr       <= 0;
            done            <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                PE_IDLE: begin
                    if (start) row_idx <= 0;
                end

                PE_LOAD_ROW_DESC: begin
                    cur_instr_count <= A_row_desc_buf[row_idx][31:16];
                    cur_instr_start <= A_row_desc_buf[row_idx][63:32];
                    instr_ptr       <= 0;
                end

                PE_STREAM_INSTRS: begin
                    if (task_group_wr_en) instr_ptr <= instr_ptr + 1;
                end

                PE_NEXT_ROW: begin
                    row_idx  <= row_idx + 1;
                    comp_sel <= ~comp_sel;
                    instr_ptr <= 0;
                end

                PE_DONE: begin
                    if (!other_acc_busy) done <= 1'b1;
                end
            endcase
        end
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
                state_next = PE_CLEAR_ACC;

            PE_CLEAR_ACC:
                state_next = PE_STREAM_INSTRS;

            PE_STREAM_INSTRS:
                if (instr_ptr >= cur_instr_count)
                    state_next = PE_WAIT_TASK_DRAIN;

            PE_WAIT_TASK_DRAIN:
                if (task_fifo_empty)
                    state_next = PE_WAIT_PRODUCT_DRAIN;

            PE_WAIT_PRODUCT_DRAIN:
                if (prod_fifo_empty && mac_pipeline_idle && !other_acc_busy)
                    state_next = PE_NEXT_ROW;

            PE_NEXT_ROW:
                state_next = ((row_idx + 1) >= row_count) ? PE_DONE : PE_LOAD_ROW_DESC;

            PE_DONE:
                state_next = PE_DONE;
        endcase
    end

endmodule
