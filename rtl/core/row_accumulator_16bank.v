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

    reg [BANK_ADDR_W-1:0] group_addr;

    //=========================================================================
    // Bundle routing — bank = col_id[3:0]
    //=========================================================================
    wire [3:0] bid0  = lane_col_id[0 *COL_W +: 4];
    wire [3:0] bid1  = lane_col_id[1 *COL_W +: 4];
    wire [3:0] bid2  = lane_col_id[2 *COL_W +: 4];
    wire [3:0] bid3  = lane_col_id[3 *COL_W +: 4];
    wire [3:0] bid4  = lane_col_id[4 *COL_W +: 4];
    wire [3:0] bid5  = lane_col_id[5 *COL_W +: 4];
    wire [3:0] bid6  = lane_col_id[6 *COL_W +: 4];
    wire [3:0] bid7  = lane_col_id[7 *COL_W +: 4];
    wire [3:0] bid8  = lane_col_id[8 *COL_W +: 4];
    wire [3:0] bid9  = lane_col_id[9 *COL_W +: 4];
    wire [3:0] bid10 = lane_col_id[10*COL_W +: 4];
    wire [3:0] bid11 = lane_col_id[11*COL_W +: 4];
    wire [3:0] bid12 = lane_col_id[12*COL_W +: 4];
    wire [3:0] bid13 = lane_col_id[13*COL_W +: 4];
    wire [3:0] bid14 = lane_col_id[14*COL_W +: 4];
    wire [3:0] bid15 = lane_col_id[15*COL_W +: 4];

    // Products-per-bank counts (5-bit, max 16)
    `define MC_SUM(B) \
        {4'b0,lane_valid[0]&(bid0==(B))}+{4'b0,lane_valid[1]&(bid1==(B))}\
       +{4'b0,lane_valid[2]&(bid2==(B))}+{4'b0,lane_valid[3]&(bid3==(B))}\
       +{4'b0,lane_valid[4]&(bid4==(B))}+{4'b0,lane_valid[5]&(bid5==(B))}\
       +{4'b0,lane_valid[6]&(bid6==(B))}+{4'b0,lane_valid[7]&(bid7==(B))}\
       +{4'b0,lane_valid[8]&(bid8==(B))}+{4'b0,lane_valid[9]&(bid9==(B))}\
       +{4'b0,lane_valid[10]&(bid10==(B))}+{4'b0,lane_valid[11]&(bid11==(B))}\
       +{4'b0,lane_valid[12]&(bid12==(B))}+{4'b0,lane_valid[13]&(bid13==(B))}\
       +{4'b0,lane_valid[14]&(bid14==(B))}+{4'b0,lane_valid[15]&(bid15==(B))}

    wire [4:0] mc0  = `MC_SUM(4'd0);
    wire [4:0] mc1  = `MC_SUM(4'd1);
    wire [4:0] mc2  = `MC_SUM(4'd2);
    wire [4:0] mc3  = `MC_SUM(4'd3);
    wire [4:0] mc4  = `MC_SUM(4'd4);
    wire [4:0] mc5  = `MC_SUM(4'd5);
    wire [4:0] mc6  = `MC_SUM(4'd6);
    wire [4:0] mc7  = `MC_SUM(4'd7);
    wire [4:0] mc8  = `MC_SUM(4'd8);
    wire [4:0] mc9  = `MC_SUM(4'd9);
    wire [4:0] mc10 = `MC_SUM(4'd10);
    wire [4:0] mc11 = `MC_SUM(4'd11);
    wire [4:0] mc12 = `MC_SUM(4'd12);
    wire [4:0] mc13 = `MC_SUM(4'd13);
    wire [4:0] mc14 = `MC_SUM(4'd14);
    wire [4:0] mc15 = `MC_SUM(4'd15);

    `undef MC_SUM

    wire [16*BANK_ADDR_W-1:0] wr_addr_flat = {
        lane_col_id[15*COL_W+4 +: BANK_ADDR_W], lane_col_id[14*COL_W+4 +: BANK_ADDR_W],
        lane_col_id[13*COL_W+4 +: BANK_ADDR_W], lane_col_id[12*COL_W+4 +: BANK_ADDR_W],
        lane_col_id[11*COL_W+4 +: BANK_ADDR_W], lane_col_id[10*COL_W+4 +: BANK_ADDR_W],
        lane_col_id[9 *COL_W+4 +: BANK_ADDR_W], lane_col_id[8 *COL_W+4 +: BANK_ADDR_W],
        lane_col_id[7 *COL_W+4 +: BANK_ADDR_W], lane_col_id[6 *COL_W+4 +: BANK_ADDR_W],
        lane_col_id[5 *COL_W+4 +: BANK_ADDR_W], lane_col_id[4 *COL_W+4 +: BANK_ADDR_W],
        lane_col_id[3 *COL_W+4 +: BANK_ADDR_W], lane_col_id[2 *COL_W+4 +: BANK_ADDR_W],
        lane_col_id[1 *COL_W+4 +: BANK_ADDR_W], lane_col_id[0 *COL_W+4 +: BANK_ADDR_W]
    };
    wire [16*PROD_W-1:0] wr_data_flat = lane_product;

    wire do_enqueue = issue_valid & issue_ready;

    `define BWV(B) {do_enqueue&lane_valid[15]&(bid15==(B)),\
                    do_enqueue&lane_valid[14]&(bid14==(B)),\
                    do_enqueue&lane_valid[13]&(bid13==(B)),\
                    do_enqueue&lane_valid[12]&(bid12==(B)),\
                    do_enqueue&lane_valid[11]&(bid11==(B)),\
                    do_enqueue&lane_valid[10]&(bid10==(B)),\
                    do_enqueue&lane_valid[9] &(bid9 ==(B)),\
                    do_enqueue&lane_valid[8] &(bid8 ==(B)),\
                    do_enqueue&lane_valid[7] &(bid7 ==(B)),\
                    do_enqueue&lane_valid[6] &(bid6 ==(B)),\
                    do_enqueue&lane_valid[5] &(bid5 ==(B)),\
                    do_enqueue&lane_valid[4] &(bid4 ==(B)),\
                    do_enqueue&lane_valid[3] &(bid3 ==(B)),\
                    do_enqueue&lane_valid[2] &(bid2 ==(B)),\
                    do_enqueue&lane_valid[1] &(bid1 ==(B)),\
                    do_enqueue&lane_valid[0] &(bid0 ==(B))}

    wire [15:0] bwv0  = `BWV(4'd0);
    wire [15:0] bwv1  = `BWV(4'd1);
    wire [15:0] bwv2  = `BWV(4'd2);
    wire [15:0] bwv3  = `BWV(4'd3);
    wire [15:0] bwv4  = `BWV(4'd4);
    wire [15:0] bwv5  = `BWV(4'd5);
    wire [15:0] bwv6  = `BWV(4'd6);
    wire [15:0] bwv7  = `BWV(4'd7);
    wire [15:0] bwv8  = `BWV(4'd8);
    wire [15:0] bwv9  = `BWV(4'd9);
    wire [15:0] bwv10 = `BWV(4'd10);
    wire [15:0] bwv11 = `BWV(4'd11);
    wire [15:0] bwv12 = `BWV(4'd12);
    wire [15:0] bwv13 = `BWV(4'd13);
    wire [15:0] bwv14 = `BWV(4'd14);
    wire [15:0] bwv15 = `BWV(4'd15);

    `undef BWV

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

    `define BANK_INST(N, BWV, FREE, RMW, EMP, CLR, DTAG, DACC) \
    accum_bank_16 #( \
        .BANK_DEPTH(BANK_DEPTH), .BANK_ADDR_W(BANK_ADDR_W), \
        .PROD_W(PROD_W), .ACC_W(ACC_W), .EPOCH_W(EPOCH_W), \
        .FIFO_DEPTH(BANK_FIFO_DEPTH), .FIFO_DEPTH_LOG(BANK_FIFO_LOG) \
    ) u_bank``N ( \
        .clk(clk), .rst_n(rst_n), .row_epoch(row_epoch), \
        .wr_valid(BWV), .wr_addr_flat(wr_addr_flat), .wr_data_flat(wr_data_flat), \
        .free_count(FREE), .rmw_busy(RMW), .fifo_empty(EMP), \
        .tag_clear_en(tag_clear_pulse), .tag_clear_busy(CLR), \
        .drain_rd_addr(drain_rd_addr), .drain_tag(DTAG), .drain_acc(DACC) \
    )

    `BANK_INST(0,  bwv0,  free_b0,  rmw_b0,  emp_b0,  clr_b0,  dtag_b0,  dacc_b0);
    `BANK_INST(1,  bwv1,  free_b1,  rmw_b1,  emp_b1,  clr_b1,  dtag_b1,  dacc_b1);
    `BANK_INST(2,  bwv2,  free_b2,  rmw_b2,  emp_b2,  clr_b2,  dtag_b2,  dacc_b2);
    `BANK_INST(3,  bwv3,  free_b3,  rmw_b3,  emp_b3,  clr_b3,  dtag_b3,  dacc_b3);
    `BANK_INST(4,  bwv4,  free_b4,  rmw_b4,  emp_b4,  clr_b4,  dtag_b4,  dacc_b4);
    `BANK_INST(5,  bwv5,  free_b5,  rmw_b5,  emp_b5,  clr_b5,  dtag_b5,  dacc_b5);
    `BANK_INST(6,  bwv6,  free_b6,  rmw_b6,  emp_b6,  clr_b6,  dtag_b6,  dacc_b6);
    `BANK_INST(7,  bwv7,  free_b7,  rmw_b7,  emp_b7,  clr_b7,  dtag_b7,  dacc_b7);
    `BANK_INST(8,  bwv8,  free_b8,  rmw_b8,  emp_b8,  clr_b8,  dtag_b8,  dacc_b8);
    `BANK_INST(9,  bwv9,  free_b9,  rmw_b9,  emp_b9,  clr_b9,  dtag_b9,  dacc_b9);
    `BANK_INST(10, bwv10, free_b10, rmw_b10, emp_b10, clr_b10, dtag_b10, dacc_b10);
    `BANK_INST(11, bwv11, free_b11, rmw_b11, emp_b11, clr_b11, dtag_b11, dacc_b11);
    `BANK_INST(12, bwv12, free_b12, rmw_b12, emp_b12, clr_b12, dtag_b12, dacc_b12);
    `BANK_INST(13, bwv13, free_b13, rmw_b13, emp_b13, clr_b13, dtag_b13, dacc_b13);
    `BANK_INST(14, bwv14, free_b14, rmw_b14, emp_b14, clr_b14, dtag_b14, dacc_b14);
    `BANK_INST(15, bwv15, free_b15, rmw_b15, emp_b15, clr_b15, dtag_b15, dacc_b15);

    `undef BANK_INST

    //=========================================================================
    // issue_ready
    //=========================================================================
    assign issue_ready = (state == S_ACCUM) && !input_done_latch
        && (free_b0  >= {1'b0, mc0 }) && (free_b1  >= {1'b0, mc1 })
        && (free_b2  >= {1'b0, mc2 }) && (free_b3  >= {1'b0, mc3 })
        && (free_b4  >= {1'b0, mc4 }) && (free_b5  >= {1'b0, mc5 })
        && (free_b6  >= {1'b0, mc6 }) && (free_b7  >= {1'b0, mc7 })
        && (free_b8  >= {1'b0, mc8 }) && (free_b9  >= {1'b0, mc9 })
        && (free_b10 >= {1'b0, mc10}) && (free_b11 >= {1'b0, mc11})
        && (free_b12 >= {1'b0, mc12}) && (free_b13 >= {1'b0, mc13})
        && (free_b14 >= {1'b0, mc14}) && (free_b15 >= {1'b0, mc15});

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
            state            <= S_IDLE;
            cur_row_id       <= {ROW_W{1'b0}};
            row_epoch        <= {{(EPOCH_W-1){1'b0}}, 1'b1};
            input_done_latch <= 1'b0;
            clr_triggered    <= 1'b0;
            group_addr       <= {BANK_ADDR_W{1'b0}};
            busy             <= 1'b0;
            row_done         <= 1'b0;
        end else begin
            row_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy             <= 1'b0;
                    input_done_latch <= 1'b0;
                    if (row_start) begin
                        cur_row_id <= row_id_in;
                        group_addr <= {BANK_ADDR_W{1'b0}};
                        busy       <= 1'b1;
                        state      <= S_ACCUM;
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
