//=============================================================================
// pe_top.v — hybrid pointer-task + Gen2, 0-overhead executor
//
// For each A[i,k] nonzero:
//   aligned part  (floor(b_nnz/8) groups) → ptr_fifo → executor (autonomous)
//   remainder     (b_nnz%8 elements)      → Gen2 accumulate → task_fifo
//
// Executor uses the sync_fifo's registered output (rd_data always shows current
// head), so no EXEC_PTR_LOAD state is needed — 0 overhead between entries.
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

    input  wire                          a_desc_valid,
    output wire                          a_desc_ready,
    input  wire [35:0]                   a_desc_data,

    input  wire                          a_val_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata,

    input  wire                          a_col_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        a_col_wdata,

    input  wire                          b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_col_wdata,
    input  wire                          b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_val_wdata,

    input  wire                          b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0]  b_desc_waddr,
    input  wire [31:0]                   b_desc_wdata
);

    //=========================================================================
    // SRAM declarations
    //=========================================================================
    reg [`DATA_WIDTH-1:0] A_val_buf [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf [0:`A_NNZ_SLOT_PER_PE-1];

    localparam B_BANK_DEPTH = `B_NNZ_SLOT / 8;
    localparam B_DESC_DEPTH = `B_ROW_SLOT;

    reg [`DATA_WIDTH-1:0] B_col_b0 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b1 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b2 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b3 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b4 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b5 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b6 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b7 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b0 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b1 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b2 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b3 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b4 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b5 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b6 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b7 [0:B_BANK_DEPTH-1];

    reg [31:0] B_desc_buf [0:B_DESC_DEPTH-1];

    //=========================================================================
    // SRAM write ports
    //=========================================================================
    always @(posedge aclk) begin
        if (a_val_we)  A_val_buf[a_val_waddr]  <= a_val_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]  <= a_col_wdata;
        if (b_desc_we) B_desc_buf[b_desc_waddr] <= b_desc_wdata;
        if (b_col_we) case (b_col_waddr[2:0])
            3'd0: B_col_b0[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd1: B_col_b1[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd2: B_col_b2[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd3: B_col_b3[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd4: B_col_b4[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd5: B_col_b5[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd6: B_col_b6[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
            3'd7: B_col_b7[b_col_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_col_wdata;
        endcase
        if (b_val_we) case (b_val_waddr[2:0])
            3'd0: B_val_b0[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd1: B_val_b1[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd2: B_val_b2[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd3: B_val_b3[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd4: B_val_b4[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd5: B_val_b5[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd6: B_val_b6[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
            3'd7: B_val_b7[b_val_waddr[`B_NNZ_ADDR_BITS-1:3]] <= b_val_wdata;
        endcase
    end

    //=========================================================================
    // Main FSM states
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

    reg comp_sel;
    reg [`A_ROW_ADDR_BITS-1:0] row_idx;
    reg [31:0]                 cur_a_off;
    reg [15:0]                 cur_a_nnz;

    //=========================================================================
    // Generator sub-FSM
    //=========================================================================
    localparam GEN_IDLE     = 3'd0;
    localparam GEN_FETCH    = 3'd1;
    localparam GEN_EMIT     = 3'd2;
    localparam GEN_ROW_DONE = 3'd3;

    reg [2:0]  gen_state;
    reg [15:0] gen_t;
    reg [15:0] gen_a_val;
    reg [31:0] gen_b_off;
    reg [15:0] gen_b_nnz;

    //=========================================================================
    // A nonzero prefetch
    //=========================================================================
    wire [`A_NNZ_ADDR_BITS-1:0] fetch_a_addr =
        cur_a_off[`A_NNZ_ADDR_BITS-1:0] + gen_t[`A_NNZ_ADDR_BITS-1:0];
    wire [15:0] fetch_a_val  = A_val_buf[fetch_a_addr];
    wire [15:0] fetch_k_idx  = A_col_buf[fetch_a_addr];
    wire [31:0] fetch_b_desc = B_desc_buf[fetch_k_idx[`B_ROW_ADDR_BITS-1:0]];
    wire [31:0] fetch_b_off  = {15'b0, fetch_b_desc[26:10]};
    wire [15:0] fetch_b_nnz  = {6'b0,  fetch_b_desc[9:0]};

    //=========================================================================
    // Generator: aligned groups (→ ptr_fifo) and remainder (→ Gen2)
    //=========================================================================
    wire [15:0] gen_num_groups = {3'b0,  gen_b_nnz[15:3]};
    wire [2:0]  gen_remainder  = gen_b_nnz[2:0];

    wire [31:0] gen_abs_base = gen_b_off + {gen_num_groups[13:0], 3'b000};
    wire [2:0]  gen_r        = gen_abs_base[2:0];
    wire [13:0] gen_m        = gen_abs_base[16:3];

    wire [13:0] gen_bg0 = (gen_r == 3'd0) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg1 = (gen_r <= 3'd1) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg2 = (gen_r <= 3'd2) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg3 = (gen_r <= 3'd3) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg4 = (gen_r <= 3'd4) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg5 = (gen_r <= 3'd5) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg6 = (gen_r <= 3'd6) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg7 = gen_m;

    wire [15:0] bc0=B_col_b0[gen_bg0]; wire [15:0] bv0=B_val_b0[gen_bg0];
    wire [15:0] bc1=B_col_b1[gen_bg1]; wire [15:0] bv1=B_val_b1[gen_bg1];
    wire [15:0] bc2=B_col_b2[gen_bg2]; wire [15:0] bv2=B_val_b2[gen_bg2];
    wire [15:0] bc3=B_col_b3[gen_bg3]; wire [15:0] bv3=B_val_b3[gen_bg3];
    wire [15:0] bc4=B_col_b4[gen_bg4]; wire [15:0] bv4=B_val_b4[gen_bg4];
    wire [15:0] bc5=B_col_b5[gen_bg5]; wire [15:0] bv5=B_val_b5[gen_bg5];
    wire [15:0] bc6=B_col_b6[gen_bg6]; wire [15:0] bv6=B_val_b6[gen_bg6];
    wire [15:0] bc7=B_col_b7[gen_bg7]; wire [15:0] bv7=B_val_b7[gen_bg7];

    wire [15:0] ne_bv [0:7]; wire [15:0] ne_bc [0:7];
    assign ne_bv[0]=(gen_r==0)?bv0:(gen_r==1)?bv1:(gen_r==2)?bv2:(gen_r==3)?bv3:(gen_r==4)?bv4:(gen_r==5)?bv5:(gen_r==6)?bv6:bv7;
    assign ne_bc[0]=(gen_r==0)?bc0:(gen_r==1)?bc1:(gen_r==2)?bc2:(gen_r==3)?bc3:(gen_r==4)?bc4:(gen_r==5)?bc5:(gen_r==6)?bc6:bc7;
    assign ne_bv[1]=(gen_r==0)?bv1:(gen_r==1)?bv2:(gen_r==2)?bv3:(gen_r==3)?bv4:(gen_r==4)?bv5:(gen_r==5)?bv6:(gen_r==6)?bv7:bv0;
    assign ne_bc[1]=(gen_r==0)?bc1:(gen_r==1)?bc2:(gen_r==2)?bc3:(gen_r==3)?bc4:(gen_r==4)?bc5:(gen_r==5)?bc6:(gen_r==6)?bc7:bc0;
    assign ne_bv[2]=(gen_r==0)?bv2:(gen_r==1)?bv3:(gen_r==2)?bv4:(gen_r==3)?bv5:(gen_r==4)?bv6:(gen_r==5)?bv7:(gen_r==6)?bv0:bv1;
    assign ne_bc[2]=(gen_r==0)?bc2:(gen_r==1)?bc3:(gen_r==2)?bc4:(gen_r==3)?bc5:(gen_r==4)?bc6:(gen_r==5)?bc7:(gen_r==6)?bc0:bc1;
    assign ne_bv[3]=(gen_r==0)?bv3:(gen_r==1)?bv4:(gen_r==2)?bv5:(gen_r==3)?bv6:(gen_r==4)?bv7:(gen_r==5)?bv0:(gen_r==6)?bv1:bv2;
    assign ne_bc[3]=(gen_r==0)?bc3:(gen_r==1)?bc4:(gen_r==2)?bc5:(gen_r==3)?bc6:(gen_r==4)?bc7:(gen_r==5)?bc0:(gen_r==6)?bc1:bc2;
    assign ne_bv[4]=(gen_r==0)?bv4:(gen_r==1)?bv5:(gen_r==2)?bv6:(gen_r==3)?bv7:(gen_r==4)?bv0:(gen_r==5)?bv1:(gen_r==6)?bv2:bv3;
    assign ne_bc[4]=(gen_r==0)?bc4:(gen_r==1)?bc5:(gen_r==2)?bc6:(gen_r==3)?bc7:(gen_r==4)?bc0:(gen_r==5)?bc1:(gen_r==6)?bc2:bc3;
    assign ne_bv[5]=(gen_r==0)?bv5:(gen_r==1)?bv6:(gen_r==2)?bv7:(gen_r==3)?bv0:(gen_r==4)?bv1:(gen_r==5)?bv2:(gen_r==6)?bv3:bv4;
    assign ne_bc[5]=(gen_r==0)?bc5:(gen_r==1)?bc6:(gen_r==2)?bc7:(gen_r==3)?bc0:(gen_r==4)?bc1:(gen_r==5)?bc2:(gen_r==6)?bc3:bc4;
    assign ne_bv[6]=(gen_r==0)?bv6:(gen_r==1)?bv7:(gen_r==2)?bv0:(gen_r==3)?bv1:(gen_r==4)?bv2:(gen_r==5)?bv3:(gen_r==6)?bv4:bv5;
    assign ne_bc[6]=(gen_r==0)?bc6:(gen_r==1)?bc7:(gen_r==2)?bc0:(gen_r==3)?bc1:(gen_r==4)?bc2:(gen_r==5)?bc3:(gen_r==6)?bc4:bc5;
    assign ne_bv[7]=(gen_r==0)?bv7:(gen_r==1)?bv0:(gen_r==2)?bv1:(gen_r==3)?bv2:(gen_r==4)?bv3:(gen_r==5)?bv4:(gen_r==6)?bv5:bv6;
    assign ne_bc[7]=(gen_r==0)?bc7:(gen_r==1)?bc0:(gen_r==2)?bc1:(gen_r==3)?bc2:(gen_r==4)?bc3:(gen_r==5)?bc4:(gen_r==6)?bc5:bc6;

    wire [`TASK_WIDTH-1:0] pack_sg0={ne_bv[0],gen_a_val,ne_bc[0][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg1={ne_bv[1],gen_a_val,ne_bc[1][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg2={ne_bv[2],gen_a_val,ne_bc[2][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg3={ne_bv[3],gen_a_val,ne_bc[3][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg4={ne_bv[4],gen_a_val,ne_bc[4][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg5={ne_bv[5],gen_a_val,ne_bc[5][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg6={ne_bv[6],gen_a_val,ne_bc[6][8:0]};

    //=========================================================================
    // Gen2: accumulate cross-A-nnz remainders
    //=========================================================================
    reg [2:0]             carry2_cnt;
    reg [`TASK_WIDTH-1:0] carry2_task [0:6];

    wire [3:0] g2_combined = {1'b0, carry2_cnt} + {1'b0, gen_remainder};
    wire       g2_can_emit = g2_combined[3];
    wire [2:0] g2_overflow = g2_combined[2:0];

    wire [`TASK_WIDTH-1:0] g2_sg0=(carry2_cnt>=1)?carry2_task[0]:pack_sg0;
    wire [`TASK_WIDTH-1:0] g2_sg1=(carry2_cnt>=2)?carry2_task[1]:(carry2_cnt==1)?pack_sg0:pack_sg1;
    wire [`TASK_WIDTH-1:0] g2_sg2=(carry2_cnt>=3)?carry2_task[2]:(carry2_cnt==2)?pack_sg0:(carry2_cnt==1)?pack_sg1:pack_sg2;
    wire [`TASK_WIDTH-1:0] g2_sg3=(carry2_cnt>=4)?carry2_task[3]:(carry2_cnt==3)?pack_sg0:(carry2_cnt==2)?pack_sg1:(carry2_cnt==1)?pack_sg2:pack_sg3;
    wire [`TASK_WIDTH-1:0] g2_sg4=(carry2_cnt>=5)?carry2_task[4]:(carry2_cnt==4)?pack_sg0:(carry2_cnt==3)?pack_sg1:(carry2_cnt==2)?pack_sg2:(carry2_cnt==1)?pack_sg3:pack_sg4;
    wire [`TASK_WIDTH-1:0] g2_sg5=(carry2_cnt>=6)?carry2_task[5]:(carry2_cnt==5)?pack_sg0:(carry2_cnt==4)?pack_sg1:(carry2_cnt==3)?pack_sg2:(carry2_cnt==2)?pack_sg3:(carry2_cnt==1)?pack_sg4:pack_sg5;
    wire [`TASK_WIDTH-1:0] g2_sg6=(carry2_cnt>=7)?carry2_task[6]:(carry2_cnt==6)?pack_sg0:(carry2_cnt==5)?pack_sg1:(carry2_cnt==4)?pack_sg2:(carry2_cnt==3)?pack_sg3:(carry2_cnt==2)?pack_sg4:(carry2_cnt==1)?pack_sg5:pack_sg6;
    wire [`TASK_WIDTH-1:0] g2_sg7=(carry2_cnt==7)?pack_sg0:(carry2_cnt==6)?pack_sg1:(carry2_cnt==5)?pack_sg2:(carry2_cnt==4)?pack_sg3:(carry2_cnt==3)?pack_sg4:(carry2_cnt==2)?pack_sg5:(carry2_cnt==1)?pack_sg6:pack_sg0;

    wire [7:0] g2_flush_lane_valid = (8'd1 << carry2_cnt) - 8'd1;

    wire task_fifo_full;
    wire ptr_fifo_full;

    wire g1_to_g2_valid = (gen_state == GEN_EMIT) && (gen_remainder != 3'd0);
    wire g2_want_emit   = g1_to_g2_valid && g2_can_emit;
    wire g2_want_flush  = (gen_state == GEN_ROW_DONE) && (carry2_cnt != 3'd0);

    wire gen_emit_stall =
        (gen_num_groups != 16'd0 && ptr_fifo_full) ||
        (gen_remainder  != 3'd0  && g2_can_emit && task_fifo_full);
    wire gen_emit_can_advance = (gen_state == GEN_EMIT) && !gen_emit_stall;
    wire g1_acc_advances      = gen_emit_can_advance && g1_to_g2_valid;

    wire ptr_fifo_wr_en =
        (gen_state == GEN_EMIT) && gen_emit_can_advance && (gen_num_groups != 16'd0);
    wire [`PTR_TASK_WIDTH-1:0] ptr_fifo_wr_data =
        {gen_a_val, gen_b_off[16:0], gen_num_groups[6:0]};

    wire task_group_wr_en = (g2_want_emit || g2_want_flush) && !task_fifo_full;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data =
        g2_want_flush
        ? {carry2_task[6],carry2_task[5],carry2_task[4],carry2_task[3],
           carry2_task[2],carry2_task[1],carry2_task[0],g2_flush_lane_valid}
        : {g2_sg7,g2_sg6,g2_sg5,g2_sg4,g2_sg3,g2_sg2,g2_sg1,g2_sg0,8'hFF};

    //=========================================================================
    // Generator sub-FSM sequential
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            gen_state<=GEN_IDLE; gen_t<=0; gen_a_val<=0; gen_b_off<=0; gen_b_nnz<=0;
        end else case (gen_state)
            GEN_IDLE: begin
                if (state == PE_CLEAR_ACC) begin
                    gen_t <= 0;
                    gen_state <= (cur_a_nnz==0) ? GEN_ROW_DONE : GEN_FETCH;
                end
            end
            GEN_FETCH: begin
                gen_a_val<=fetch_a_val; gen_b_off<=fetch_b_off; gen_b_nnz<=fetch_b_nnz;
                gen_t<=gen_t+16'd1;
                if (fetch_b_nnz==0) begin
                    if (gen_t+16'd1 >= cur_a_nnz) gen_state<=GEN_ROW_DONE;
                end else gen_state<=GEN_EMIT;
            end
            GEN_EMIT: begin
                if (gen_emit_can_advance) begin
                    if (gen_t >= cur_a_nnz) begin
                        gen_state <= GEN_ROW_DONE;
                    end else if (fetch_b_nnz == 16'd0) begin
                        gen_t <= gen_t + 16'd1;
                        if (gen_t+16'd1 >= cur_a_nnz) gen_state<=GEN_ROW_DONE;
                        else gen_state<=GEN_FETCH;
                    end else begin
                        gen_a_val<=fetch_a_val; gen_b_off<=fetch_b_off;
                        gen_b_nnz<=fetch_b_nnz; gen_t<=gen_t+16'd1;
                    end
                end
            end
            GEN_ROW_DONE: begin
                if (state==PE_WAIT_TASK_DRAIN || state==PE_NEXT_ROW ||
                    state==PE_WAIT_PRODUCT_DRAIN)
                    gen_state<=GEN_IDLE;
            end
            default: gen_state<=GEN_IDLE;
        endcase
    end

    //=========================================================================
    // Gen2 sequential
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            carry2_cnt<=0;
            carry2_task[0]<=0; carry2_task[1]<=0; carry2_task[2]<=0;
            carry2_task[3]<=0; carry2_task[4]<=0; carry2_task[5]<=0; carry2_task[6]<=0;
        end else begin
            if (g1_acc_advances) begin
                if (g2_can_emit) begin
                    carry2_cnt <= g2_overflow;
                    case (carry2_cnt)
                        3'd2: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg6; end
                        3'd3: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg5; if(g2_overflow>=2) carry2_task[1]<=pack_sg6; end
                        3'd4: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg4; if(g2_overflow>=2) carry2_task[1]<=pack_sg5; if(g2_overflow>=3) carry2_task[2]<=pack_sg6; end
                        3'd5: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg3; if(g2_overflow>=2) carry2_task[1]<=pack_sg4; if(g2_overflow>=3) carry2_task[2]<=pack_sg5; if(g2_overflow>=4) carry2_task[3]<=pack_sg6; end
                        3'd6: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg2; if(g2_overflow>=2) carry2_task[1]<=pack_sg3; if(g2_overflow>=3) carry2_task[2]<=pack_sg4; if(g2_overflow>=4) carry2_task[3]<=pack_sg5; if(g2_overflow>=5) carry2_task[4]<=pack_sg6; end
                        3'd7: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg1; if(g2_overflow>=2) carry2_task[1]<=pack_sg2; if(g2_overflow>=3) carry2_task[2]<=pack_sg3; if(g2_overflow>=4) carry2_task[3]<=pack_sg4; if(g2_overflow>=5) carry2_task[4]<=pack_sg5; if(g2_overflow>=6) carry2_task[5]<=pack_sg6; end
                        default: carry2_cnt<=0;
                    endcase
                end else begin
                    carry2_cnt <= g2_combined[2:0];
                    case (carry2_cnt)
                        3'd0: begin if(gen_remainder>=1) carry2_task[0]<=pack_sg0; if(gen_remainder>=2) carry2_task[1]<=pack_sg1; if(gen_remainder>=3) carry2_task[2]<=pack_sg2; if(gen_remainder>=4) carry2_task[3]<=pack_sg3; if(gen_remainder>=5) carry2_task[4]<=pack_sg4; if(gen_remainder>=6) carry2_task[5]<=pack_sg5; if(gen_remainder==7) carry2_task[6]<=pack_sg6; end
                        3'd1: begin if(gen_remainder>=1) carry2_task[1]<=pack_sg0; if(gen_remainder>=2) carry2_task[2]<=pack_sg1; if(gen_remainder>=3) carry2_task[3]<=pack_sg2; if(gen_remainder>=4) carry2_task[4]<=pack_sg3; if(gen_remainder>=5) carry2_task[5]<=pack_sg4; if(gen_remainder>=6) carry2_task[6]<=pack_sg5; end
                        3'd2: begin if(gen_remainder>=1) carry2_task[2]<=pack_sg0; if(gen_remainder>=2) carry2_task[3]<=pack_sg1; if(gen_remainder>=3) carry2_task[4]<=pack_sg2; if(gen_remainder>=4) carry2_task[5]<=pack_sg3; if(gen_remainder>=5) carry2_task[6]<=pack_sg4; end
                        3'd3: begin if(gen_remainder>=1) carry2_task[3]<=pack_sg0; if(gen_remainder>=2) carry2_task[4]<=pack_sg1; if(gen_remainder>=3) carry2_task[5]<=pack_sg2; if(gen_remainder>=4) carry2_task[6]<=pack_sg3; end
                        3'd4: begin if(gen_remainder>=1) carry2_task[4]<=pack_sg0; if(gen_remainder>=2) carry2_task[5]<=pack_sg1; if(gen_remainder>=3) carry2_task[6]<=pack_sg2; end
                        3'd5: begin if(gen_remainder>=1) carry2_task[5]<=pack_sg0; if(gen_remainder>=2) carry2_task[6]<=pack_sg1; end
                        3'd6: begin if(gen_remainder>=1) carry2_task[6]<=pack_sg0; end
                        default: ;
                    endcase
                end
            end else if (g2_want_flush && !task_fifo_full) begin
                carry2_cnt <= 3'd0;
            end
        end
    end

    //=========================================================================
    // task_fifo (Gen2 output)
    //=========================================================================
    wire task_fifo_rd_en;
    wire [`TASK_GROUP_WIDTH-1:0] task_fifo_rd_data;
    wire task_fifo_empty;

    sync_fifo #(.WIDTH(`TASK_GROUP_WIDTH),.DEPTH(`TASK_FIFO_DEPTH),.DEPTH_LOG(`TASK_FIFO_DEPTH_LOG))
    u_task_fifo (
        .wr_en(task_group_wr_en),.wr_data(task_group_wr_data),.wr_full(task_fifo_full),
        .rd_en(task_fifo_rd_en),.rd_data(task_fifo_rd_data),.rd_empty(task_fifo_empty),
        .count(),.aclk(aclk),.aresetn(aresetn)
    );

    //=========================================================================
    // ptr_fifo (pointer tasks)
    //=========================================================================
    wire [`PTR_TASK_WIDTH-1:0] ptr_fifo_rd_data;
    wire ptr_fifo_empty;
    wire ptr_fifo_rd_en;

    sync_fifo #(.WIDTH(`PTR_TASK_WIDTH),.DEPTH(`PTR_FIFO_DEPTH),.DEPTH_LOG(`PTR_FIFO_DEPTH_LOG))
    u_ptr_fifo (
        .wr_en(ptr_fifo_wr_en),.wr_data(ptr_fifo_wr_data),.wr_full(ptr_fifo_full),
        .rd_en(ptr_fifo_rd_en),.rd_data(ptr_fifo_rd_data),.rd_empty(ptr_fifo_empty),
        .count(),.aclk(aclk),.aresetn(aresetn)
    );

    //=========================================================================
    // Product FIFOs — declared early so exec_prod_safe can reference count
    //=========================================================================
    wire [`PROD_FIFO_DEPTH_LOG:0] product_fifo_cnt_0, product_fifo_cnt_1;

    wire [`PROD_FIFO_DEPTH_LOG:0] active_prod_fifo_cnt =
        comp_sel ? product_fifo_cnt_1 : product_fifo_cnt_0;

    wire exec_prod_safe = active_prod_fifo_cnt < (`PROD_FIFO_DEPTH - `MUL_LAT - 1);

    //=========================================================================
    // MAC Executor — 2 states, 0 overhead between consecutive entries
    //
    // sync_fifo rd_data always reflects mem[rd_ptr] (registered head).
    // EXEC_IDLE samples it directly; EXEC_PTR samples it at exec_ptr_last.
    // ptr_fifo_rd_en is asserted that same cycle to advance rd_ptr.
    //=========================================================================
    localparam EXEC_IDLE = 1'd0;
    localparam EXEC_PTR  = 1'd1;

    reg        exec_state;
    reg [15:0] exec_a_val;
    reg [16:0] exec_b_off;
    reg [6:0]  exec_num_groups;
    reg [6:0]  exec_g;

    wire exec_idle = (exec_state == EXEC_IDLE);
    wire exec_busy = !exec_idle;

    wire exec_ptr_last = (exec_state == EXEC_PTR) &&
                         exec_prod_safe &&
                         (exec_g + 7'd1 >= {1'b0, exec_num_groups});

    assign ptr_fifo_rd_en = (exec_idle && !ptr_fifo_empty) ||
                            (exec_ptr_last && !ptr_fifo_empty);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            exec_state<=EXEC_IDLE; exec_a_val<=0; exec_b_off<=0; exec_num_groups<=0; exec_g<=0;
        end else case (exec_state)
            EXEC_IDLE: begin
                if (!ptr_fifo_empty) begin
                    exec_a_val      <= ptr_fifo_rd_data[39:24];
                    exec_b_off      <= ptr_fifo_rd_data[23:7];
                    exec_num_groups <= ptr_fifo_rd_data[6:0];
                    exec_g          <= 7'd0;
                    exec_state      <= EXEC_PTR;
                end
            end
            EXEC_PTR: begin
                if (exec_prod_safe) begin
                    exec_g <= exec_g + 7'd1;
                    if (exec_ptr_last) begin
                        if (!ptr_fifo_empty) begin
                            exec_a_val      <= ptr_fifo_rd_data[39:24];
                            exec_b_off      <= ptr_fifo_rd_data[23:7];
                            exec_num_groups <= ptr_fifo_rd_data[6:0];
                            exec_g          <= 7'd0;
                        end else begin
                            exec_state <= EXEC_IDLE;
                        end
                    end
                end
                // else: stall when prod_fifo near full
            end
            default: exec_state<=EXEC_IDLE;
        endcase
    end

    //=========================================================================
    // Executor B bank reads
    //=========================================================================
    wire [31:0] exec_abs_base = {15'b0, exec_b_off} + {22'b0, exec_g, 3'b000};
    wire [2:0]  exec_r        = exec_abs_base[2:0];
    wire [13:0] exec_m        = exec_abs_base[16:3];

    wire [13:0] exec_bg0=(exec_r==0)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg1=(exec_r<=1)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg2=(exec_r<=2)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg3=(exec_r<=3)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg4=(exec_r<=4)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg5=(exec_r<=5)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg6=(exec_r<=6)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg7=exec_m;

    wire [15:0] ebc0=B_col_b0[exec_bg0]; wire [15:0] ebv0=B_val_b0[exec_bg0];
    wire [15:0] ebc1=B_col_b1[exec_bg1]; wire [15:0] ebv1=B_val_b1[exec_bg1];
    wire [15:0] ebc2=B_col_b2[exec_bg2]; wire [15:0] ebv2=B_val_b2[exec_bg2];
    wire [15:0] ebc3=B_col_b3[exec_bg3]; wire [15:0] ebv3=B_val_b3[exec_bg3];
    wire [15:0] ebc4=B_col_b4[exec_bg4]; wire [15:0] ebv4=B_val_b4[exec_bg4];
    wire [15:0] ebc5=B_col_b5[exec_bg5]; wire [15:0] ebv5=B_val_b5[exec_bg5];
    wire [15:0] ebc6=B_col_b6[exec_bg6]; wire [15:0] ebv6=B_val_b6[exec_bg6];
    wire [15:0] ebc7=B_col_b7[exec_bg7]; wire [15:0] ebv7=B_val_b7[exec_bg7];

    wire [15:0] enebv [0:7]; wire [15:0] enebc [0:7];
    assign enebv[0]=(exec_r==0)?ebv0:(exec_r==1)?ebv1:(exec_r==2)?ebv2:(exec_r==3)?ebv3:(exec_r==4)?ebv4:(exec_r==5)?ebv5:(exec_r==6)?ebv6:ebv7;
    assign enebc[0]=(exec_r==0)?ebc0:(exec_r==1)?ebc1:(exec_r==2)?ebc2:(exec_r==3)?ebc3:(exec_r==4)?ebc4:(exec_r==5)?ebc5:(exec_r==6)?ebc6:ebc7;
    assign enebv[1]=(exec_r==0)?ebv1:(exec_r==1)?ebv2:(exec_r==2)?ebv3:(exec_r==3)?ebv4:(exec_r==4)?ebv5:(exec_r==5)?ebv6:(exec_r==6)?ebv7:ebv0;
    assign enebc[1]=(exec_r==0)?ebc1:(exec_r==1)?ebc2:(exec_r==2)?ebc3:(exec_r==3)?ebc4:(exec_r==4)?ebc5:(exec_r==5)?ebc6:(exec_r==6)?ebc7:ebc0;
    assign enebv[2]=(exec_r==0)?ebv2:(exec_r==1)?ebv3:(exec_r==2)?ebv4:(exec_r==3)?ebv5:(exec_r==4)?ebv6:(exec_r==5)?ebv7:(exec_r==6)?ebv0:ebv1;
    assign enebc[2]=(exec_r==0)?ebc2:(exec_r==1)?ebc3:(exec_r==2)?ebc4:(exec_r==3)?ebc5:(exec_r==4)?ebc6:(exec_r==5)?ebc7:(exec_r==6)?ebc0:ebc1;
    assign enebv[3]=(exec_r==0)?ebv3:(exec_r==1)?ebv4:(exec_r==2)?ebv5:(exec_r==3)?ebv6:(exec_r==4)?ebv7:(exec_r==5)?ebv0:(exec_r==6)?ebv1:ebv2;
    assign enebc[3]=(exec_r==0)?ebc3:(exec_r==1)?ebc4:(exec_r==2)?ebc5:(exec_r==3)?ebc6:(exec_r==4)?ebc7:(exec_r==5)?ebc0:(exec_r==6)?ebc1:ebc2;
    assign enebv[4]=(exec_r==0)?ebv4:(exec_r==1)?ebv5:(exec_r==2)?ebv6:(exec_r==3)?ebv7:(exec_r==4)?ebv0:(exec_r==5)?ebv1:(exec_r==6)?ebv2:ebv3;
    assign enebc[4]=(exec_r==0)?ebc4:(exec_r==1)?ebc5:(exec_r==2)?ebc6:(exec_r==3)?ebc7:(exec_r==4)?ebc0:(exec_r==5)?ebc1:(exec_r==6)?ebc2:ebc3;
    assign enebv[5]=(exec_r==0)?ebv5:(exec_r==1)?ebv6:(exec_r==2)?ebv7:(exec_r==3)?ebv0:(exec_r==4)?ebv1:(exec_r==5)?ebv2:(exec_r==6)?ebv3:ebv4;
    assign enebc[5]=(exec_r==0)?ebc5:(exec_r==1)?ebc6:(exec_r==2)?ebc7:(exec_r==3)?ebc0:(exec_r==4)?ebc1:(exec_r==5)?ebc2:(exec_r==6)?ebc3:ebc4;
    assign enebv[6]=(exec_r==0)?ebv6:(exec_r==1)?ebv7:(exec_r==2)?ebv0:(exec_r==3)?ebv1:(exec_r==4)?ebv2:(exec_r==5)?ebv3:(exec_r==6)?ebv4:ebv5;
    assign enebc[6]=(exec_r==0)?ebc6:(exec_r==1)?ebc7:(exec_r==2)?ebc0:(exec_r==3)?ebc1:(exec_r==4)?ebc2:(exec_r==5)?ebc3:(exec_r==6)?ebc4:ebc5;
    assign enebv[7]=(exec_r==0)?ebv7:(exec_r==1)?ebv0:(exec_r==2)?ebv1:(exec_r==3)?ebv2:(exec_r==4)?ebv3:(exec_r==5)?ebv4:(exec_r==6)?ebv5:ebv6;
    assign enebc[7]=(exec_r==0)?ebc7:(exec_r==1)?ebc0:(exec_r==2)?ebc1:(exec_r==3)?ebc2:(exec_r==4)?ebc3:(exec_r==5)?ebc4:(exec_r==6)?ebc5:ebc6;

    wire [`TASK_WIDTH-1:0] exec_sg0={enebv[0],exec_a_val,enebc[0][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg1={enebv[1],exec_a_val,enebc[1][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg2={enebv[2],exec_a_val,enebc[2][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg3={enebv[3],exec_a_val,enebc[3][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg4={enebv[4],exec_a_val,enebc[4][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg5={enebv[5],exec_a_val,enebc[5][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg6={enebv[6],exec_a_val,enebc[6][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg7={enebv[7],exec_a_val,enebc[7][8:0]};

    //=========================================================================
    // MAC array input: executor (ptr_fifo path) or Gen2 (task_fifo path)
    //
    // task_fifo_rd_en gated by ptr_fifo_empty to prevent conflict: if ptr_fifo
    // is non-empty, exec will transition to EXEC_PTR next cycle and win the mux.
    //=========================================================================
    assign task_fifo_rd_en = exec_idle && ptr_fifo_empty && !task_fifo_empty && exec_prod_safe;

    reg                         task_fifo_rd_en_d1;
    reg [`TASK_GROUP_WIDTH-1:0] task_fifo_rd_data_d1;
    always @(posedge aclk) begin
        if (!aresetn) begin
            task_fifo_rd_en_d1<=0; task_fifo_rd_data_d1<=0;
        end else begin
            task_fifo_rd_en_d1   <= task_fifo_rd_en;
            task_fifo_rd_data_d1 <= task_fifo_rd_data;
        end
    end

    reg [`N_MAC-1:0]             mac_lane_valid_r;
    reg [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task_r;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            mac_lane_valid_r<=0; mac_lane_task_r<=0;
        end else if (exec_state==EXEC_PTR && exec_prod_safe) begin
            mac_lane_valid_r <= 8'hFF;
            mac_lane_task_r[0*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg0;
            mac_lane_task_r[1*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg1;
            mac_lane_task_r[2*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg2;
            mac_lane_task_r[3*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg3;
            mac_lane_task_r[4*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg4;
            mac_lane_task_r[5*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg5;
            mac_lane_task_r[6*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg6;
            mac_lane_task_r[7*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg7;
        end else if (task_fifo_rd_en_d1) begin
            mac_lane_valid_r <= task_fifo_rd_data_d1[`N_MAC-1:0];
            mac_lane_task_r[0*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+0*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[1*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+1*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[2*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+2*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[3*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+3*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[4*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+4*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[5*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+5*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[6*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+6*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[7*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+7*`TASK_WIDTH+:`TASK_WIDTH];
        end else begin
            mac_lane_valid_r<=0;
        end
    end

    wire [`N_MAC-1:0]             mac_lane_valid = mac_lane_valid_r;
    wire [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task  = mac_lane_task_r;

    //=========================================================================
    // Multiplier array
    //=========================================================================
    wire [`N_MAC-1:0]                mul_valid;
    wire [`N_MAC*`PRODUCT_WIDTH-1:0] mul_product;

    pe_mul_array u_mul_array (
        .lane_valid(mac_lane_valid),.lane_task(mac_lane_task),
        .mul_valid(mul_valid),.mul_product(mul_product),
        .aclk(aclk),.aresetn(aresetn)
    );

    //=========================================================================
    // Dual product FIFOs (ping-pong)
    //=========================================================================
    wire [`PRODUCT_GROUP_WIDTH-1:0] product_group_wr_data;
    wire product_fifo_full_0, product_fifo_full_1;

    wire product_fifo_full   = comp_sel ? product_fifo_full_1 : product_fifo_full_0;
    wire product_group_wr_en = |mul_valid && !product_fifo_full;

    assign product_group_wr_data[`N_MAC-1:0]=mul_valid;
    assign product_group_wr_data[`N_MAC+0*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[0*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+1*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[1*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+2*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[2*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+3*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[3*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+4*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[4*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+5*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[5*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+6*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[6*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+7*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[7*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];

    wire prod_fifo_rd_en_0,prod_fifo_rd_en_1;
    wire [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data_0,prod_fifo_rd_data_1;
    wire prod_fifo_empty_0,prod_fifo_empty_1;

    sync_fifo #(.WIDTH(`PRODUCT_GROUP_WIDTH),.DEPTH(`PROD_FIFO_DEPTH),.DEPTH_LOG(`PROD_FIFO_DEPTH_LOG))
    u_product_fifo_0 (
        .wr_en(product_group_wr_en&&!comp_sel),.wr_data(product_group_wr_data),
        .wr_full(product_fifo_full_0),.rd_en(prod_fifo_rd_en_0),
        .rd_data(prod_fifo_rd_data_0),.rd_empty(prod_fifo_empty_0),
        .count(product_fifo_cnt_0),.aclk(aclk),.aresetn(aresetn)
    );

    sync_fifo #(.WIDTH(`PRODUCT_GROUP_WIDTH),.DEPTH(`PROD_FIFO_DEPTH),.DEPTH_LOG(`PROD_FIFO_DEPTH_LOG))
    u_product_fifo_1 (
        .wr_en(product_group_wr_en&&comp_sel),.wr_data(product_group_wr_data),
        .wr_full(product_fifo_full_1),.rd_en(prod_fifo_rd_en_1),
        .rd_data(prod_fifo_rd_data_1),.rd_empty(prod_fifo_empty_1),
        .count(product_fifo_cnt_1),.aclk(aclk),.aresetn(aresetn)
    );

    //=========================================================================
    // Row accumulators (ping-pong)
    //=========================================================================
    wire mac_pipeline_idle = !(|mac_lane_valid);

    wire acc_busy_0,acc_busy_1,acc_row_done_0,acc_row_done_1;
    wire acc_issue_ready_0,acc_issue_ready_1;
    wire [7:0] drain_valid_0,drain_valid_1;
    wire [5:0] drain_gaddr_0,drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] drain_row_id_0,drain_row_id_1;
    wire [8*16-1:0] drain_values_0,drain_values_1;

    wire other_acc_busy = comp_sel ? acc_busy_0 : acc_busy_1;

    assign a_desc_ready = (state == PE_LOAD_ROW_DESC);

    wire pe_drain_done = (state==PE_WAIT_PRODUCT_DRAIN) && mac_pipeline_idle && !other_acc_busy;

    reg mac_done_latch_0,mac_done_latch_1;
    always @(posedge aclk) begin
        if (!aresetn) begin mac_done_latch_0<=0; mac_done_latch_1<=0; end
        else begin
            if (pe_drain_done&&!comp_sel) mac_done_latch_0<=1;
            if (pe_drain_done&& comp_sel) mac_done_latch_1<=1;
            if (mac_done_latch_0&&prod_fifo_empty_0) mac_done_latch_0<=0;
            if (mac_done_latch_1&&prod_fifo_empty_1) mac_done_latch_1<=0;
        end
    end

    wire acc_inp_done_0 = mac_done_latch_0 && prod_fifo_empty_0;
    wire acc_inp_done_1 = mac_done_latch_1 && prod_fifo_empty_1;

    assign prod_fifo_rd_en_0 = !prod_fifo_empty_0 && acc_issue_ready_0;
    assign prod_fifo_rd_en_1 = !prod_fifo_empty_1 && acc_issue_ready_1;

    reg prd_rd_d1_0,prd_rd_d1_1;
    reg [`PRODUCT_GROUP_WIDTH-1:0] prd_dat_d1_0,prd_dat_d1_1;
    always @(posedge aclk) begin
        if (!aresetn) begin prd_rd_d1_0<=0; prd_dat_d1_0<=0; prd_rd_d1_1<=0; prd_dat_d1_1<=0; end
        else begin
            prd_rd_d1_0  <= prod_fifo_rd_en_0&&!prod_fifo_empty_0;
            prd_dat_d1_0 <= prod_fifo_rd_data_0;
            prd_rd_d1_1  <= prod_fifo_rd_en_1&&!prod_fifo_empty_1;
            prd_dat_d1_1 <= prod_fifo_rd_data_1;
        end
    end

    wire [7:0]    alv0=prd_dat_d1_0[`N_MAC-1:0];
    wire [8*9-1:0] alc0={prd_dat_d1_0[`N_MAC+7*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+6*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+5*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+4*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+3*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+2*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+1*`PRODUCT_WIDTH+16+:9],prd_dat_d1_0[`N_MAC+0*`PRODUCT_WIDTH+16+:9]};
    wire [8*16-1:0] alp0={prd_dat_d1_0[`N_MAC+7*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+6*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+5*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+4*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+3*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+2*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+1*`PRODUCT_WIDTH+:16],prd_dat_d1_0[`N_MAC+0*`PRODUCT_WIDTH+:16]};
    wire [7:0]    alv1=prd_dat_d1_1[`N_MAC-1:0];
    wire [8*9-1:0] alc1={prd_dat_d1_1[`N_MAC+7*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+6*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+5*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+4*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+3*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+2*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+1*`PRODUCT_WIDTH+16+:9],prd_dat_d1_1[`N_MAC+0*`PRODUCT_WIDTH+16+:9]};
    wire [8*16-1:0] alp1={prd_dat_d1_1[`N_MAC+7*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+6*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+5*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+4*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+3*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+2*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+1*`PRODUCT_WIDTH+:16],prd_dat_d1_1[`N_MAC+0*`PRODUCT_WIDTH+:16]};

    row_accumulator_8bank #(
        .OUT_COLS(512),.COL_W(9),.PROD_W(16),.ACC_W(16),.EPOCH_W(16),
        .BANK_FIFO_DEPTH(32),.BANK_FIFO_LOG(5),.ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_0 (
        .clk(aclk),.rst_n(aresetn),
        .row_start((state==PE_CLEAR_ACC)&&!comp_sel),.row_id_in(row_idx),.drain_cols(N),
        .row_input_done(acc_inp_done_0),.busy(acc_busy_0),.row_done(acc_row_done_0),
        .issue_valid(!prod_fifo_empty_0),.issue_ready(acc_issue_ready_0),
        .lane_valid(alv0),.lane_col_id(alc0),.lane_product(alp0),
        .drain_valid(drain_valid_0),.drain_gaddr(drain_gaddr_0),
        .drain_row_id(drain_row_id_0),.drain_values(drain_values_0)
    );

    row_accumulator_8bank #(
        .OUT_COLS(512),.COL_W(9),.PROD_W(16),.ACC_W(16),.EPOCH_W(16),
        .BANK_FIFO_DEPTH(32),.BANK_FIFO_LOG(5),.ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_1 (
        .clk(aclk),.rst_n(aresetn),
        .row_start((state==PE_CLEAR_ACC)&&comp_sel),.row_id_in(row_idx),.drain_cols(N),
        .row_input_done(acc_inp_done_1),.busy(acc_busy_1),.row_done(acc_row_done_1),
        .issue_valid(!prod_fifo_empty_1),.issue_ready(acc_issue_ready_1),
        .lane_valid(alv1),.lane_col_id(alc1),.lane_product(alp1),
        .drain_valid(drain_valid_1),.drain_gaddr(drain_gaddr_1),
        .drain_row_id(drain_row_id_1),.drain_values(drain_values_1)
    );

    //=========================================================================
    // Main FSM
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) state<=PE_IDLE;
        else          state<=state_next;
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin comp_sel<=0; row_idx<=0; cur_a_off<=0; cur_a_nnz<=0; done<=0; end
        else begin
            done<=0;
            case (state)
                PE_IDLE:          if (start) row_idx<=0;
                PE_LOAD_ROW_DESC: if (a_desc_valid) begin
                    cur_a_off<={18'b0,a_desc_data[32:19]};
                    cur_a_nnz<={6'b0, a_desc_data[18:9]};
                end
                PE_NEXT_ROW: begin row_idx<=row_idx+1; comp_sel<=~comp_sel; end
                PE_DONE: if (!acc_busy_0&&!acc_busy_1) done<=1;
            endcase
        end
    end

    always @(*) begin
        state_next=state;
        case (state)
            PE_IDLE:               if (start)        state_next=PE_LOAD_ROW_DESC;
            PE_LOAD_ROW_DESC:      if (a_desc_valid) state_next=PE_CLEAR_ACC;
            PE_CLEAR_ACC:                             state_next=PE_STREAM_INSTRS;
            PE_STREAM_INSTRS:      if (gen_state==GEN_ROW_DONE) state_next=PE_WAIT_TASK_DRAIN;
            PE_WAIT_TASK_DRAIN:    if (task_fifo_empty&&ptr_fifo_empty&&exec_idle) state_next=PE_WAIT_PRODUCT_DRAIN;
            PE_WAIT_PRODUCT_DRAIN: if (mac_pipeline_idle&&!other_acc_busy) state_next=PE_NEXT_ROW;
            PE_NEXT_ROW:           state_next=((row_idx+1)>=row_count)?PE_DONE:PE_LOAD_ROW_DESC;
            PE_DONE:               state_next=PE_DONE;
        endcase
    end

endmodule
