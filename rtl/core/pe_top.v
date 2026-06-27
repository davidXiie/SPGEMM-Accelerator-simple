//=============================================================================
// pe_top.v — hybrid pointer-task + Gen2, 0-overhead executor, N_MAC=16
//
// For each A[i,k] nonzero:
//   aligned part  (floor(b_nnz/16) groups) → ptr_fifo → executor (autonomous)
//   remainder     (b_nnz%16 elements)      → Gen2 accumulate → task_fifo
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
    input  wire [31:0]                   b_desc_wdata,

    // C buffer read port (independent C bank; synchronous 1-cycle read).
    // Address = {local_row[C_ROW_ADDR_BITS-1:0], gaddr[4:0]}; data = 16 FP16
    // lanes for column group gaddr (column j = gaddr*16 + lane).  c_rd_row
    // returns the global C row id of this local slot (from C_row_map).
    input  wire                          c_rd_en,
    input  wire [`C_ROW_ADDR_BITS+4:0]  c_rd_addr,
    output reg  [16*16-1:0]              c_rd_data,
    output reg  [`MAX_DIM_BITS-1:0]      c_rd_row
);

    //=========================================================================
    // SRAM declarations
    //=========================================================================
    reg [`DATA_WIDTH-1:0] A_val_buf [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf [0:`A_NNZ_SLOT_PER_PE-1];

    localparam B_BANK_DEPTH = `B_NNZ_SLOT / 16;
    localparam B_DESC_DEPTH = `B_ROW_SLOT;

    reg [`DATA_WIDTH-1:0] B_col_b0  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b1  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b2  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b3  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b4  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b5  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b6  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b7  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b8  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b9  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b10 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b11 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b12 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b13 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b14 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b15 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b0  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b1  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b2  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b3  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b4  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b5  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b6  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b7  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b8  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b9  [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b10 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b11 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b12 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b13 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b14 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b15 [0:B_BANK_DEPTH-1];

    reg [31:0] B_desc_buf [0:B_DESC_DEPTH-1];

    //=========================================================================
    // SRAM write ports
    //=========================================================================
    always @(posedge aclk) begin
        if (a_val_we)  A_val_buf[a_val_waddr]  <= a_val_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]  <= a_col_wdata;
        if (b_desc_we) B_desc_buf[b_desc_waddr] <= b_desc_wdata;
        if (b_col_we) case (b_col_waddr[3:0])
            4'd0:  B_col_b0 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd1:  B_col_b1 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd2:  B_col_b2 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd3:  B_col_b3 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd4:  B_col_b4 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd5:  B_col_b5 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd6:  B_col_b6 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd7:  B_col_b7 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd8:  B_col_b8 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd9:  B_col_b9 [b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd10: B_col_b10[b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd11: B_col_b11[b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd12: B_col_b12[b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd13: B_col_b13[b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd14: B_col_b14[b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
            4'd15: B_col_b15[b_col_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_col_wdata;
        endcase
        if (b_val_we) case (b_val_waddr[3:0])
            4'd0:  B_val_b0 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd1:  B_val_b1 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd2:  B_val_b2 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd3:  B_val_b3 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd4:  B_val_b4 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd5:  B_val_b5 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd6:  B_val_b6 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd7:  B_val_b7 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd8:  B_val_b8 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd9:  B_val_b9 [b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd10: B_val_b10[b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd11: B_val_b11[b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd12: B_val_b12[b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd13: B_val_b13[b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd14: B_val_b14[b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
            4'd15: B_val_b15[b_val_waddr[`B_NNZ_ADDR_BITS-1:4]] <= b_val_wdata;
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
    reg [`A_ROW_ADDR_BITS-1:0] row_idx;     // local row index (dense, per PE)
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
    wire [15:0] gen_num_groups = {4'b0,  gen_b_nnz[15:4]};
    wire [3:0]  gen_remainder  = gen_b_nnz[3:0];

    wire [31:0] gen_abs_base = gen_b_off + {gen_num_groups[13:0], 4'b0000};
    wire [3:0]  gen_r        = gen_abs_base[3:0];
    wire [13:0] gen_m        = gen_abs_base[17:4];

    wire [13:0] gen_bg0  = (gen_r == 4'd0)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg1  = (gen_r <= 4'd1)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg2  = (gen_r <= 4'd2)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg3  = (gen_r <= 4'd3)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg4  = (gen_r <= 4'd4)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg5  = (gen_r <= 4'd5)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg6  = (gen_r <= 4'd6)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg7  = (gen_r <= 4'd7)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg8  = (gen_r <= 4'd8)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg9  = (gen_r <= 4'd9)  ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg10 = (gen_r <= 4'd10) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg11 = (gen_r <= 4'd11) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg12 = (gen_r <= 4'd12) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg13 = (gen_r <= 4'd13) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg14 = (gen_r <= 4'd14) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg15 = gen_m;

    wire [15:0] bc0 =B_col_b0 [gen_bg0];  wire [15:0] bv0 =B_val_b0 [gen_bg0];
    wire [15:0] bc1 =B_col_b1 [gen_bg1];  wire [15:0] bv1 =B_val_b1 [gen_bg1];
    wire [15:0] bc2 =B_col_b2 [gen_bg2];  wire [15:0] bv2 =B_val_b2 [gen_bg2];
    wire [15:0] bc3 =B_col_b3 [gen_bg3];  wire [15:0] bv3 =B_val_b3 [gen_bg3];
    wire [15:0] bc4 =B_col_b4 [gen_bg4];  wire [15:0] bv4 =B_val_b4 [gen_bg4];
    wire [15:0] bc5 =B_col_b5 [gen_bg5];  wire [15:0] bv5 =B_val_b5 [gen_bg5];
    wire [15:0] bc6 =B_col_b6 [gen_bg6];  wire [15:0] bv6 =B_val_b6 [gen_bg6];
    wire [15:0] bc7 =B_col_b7 [gen_bg7];  wire [15:0] bv7 =B_val_b7 [gen_bg7];
    wire [15:0] bc8 =B_col_b8 [gen_bg8];  wire [15:0] bv8 =B_val_b8 [gen_bg8];
    wire [15:0] bc9 =B_col_b9 [gen_bg9];  wire [15:0] bv9 =B_val_b9 [gen_bg9];
    wire [15:0] bc10=B_col_b10[gen_bg10]; wire [15:0] bv10=B_val_b10[gen_bg10];
    wire [15:0] bc11=B_col_b11[gen_bg11]; wire [15:0] bv11=B_val_b11[gen_bg11];
    wire [15:0] bc12=B_col_b12[gen_bg12]; wire [15:0] bv12=B_val_b12[gen_bg12];
    wire [15:0] bc13=B_col_b13[gen_bg13]; wire [15:0] bv13=B_val_b13[gen_bg13];
    wire [15:0] bc14=B_col_b14[gen_bg14]; wire [15:0] bv14=B_val_b14[gen_bg14];
    wire [15:0] bc15=B_col_b15[gen_bg15]; wire [15:0] bv15=B_val_b15[gen_bg15];

    // Rotation mux: ne_bv[j] = bv at bank (gen_r+j)%16
    wire [15:0] ne_bv [0:15]; wire [15:0] ne_bc [0:15];
    assign ne_bv[0] =(gen_r==0)?bv0 :(gen_r==1)?bv1 :(gen_r==2)?bv2 :(gen_r==3)?bv3 :(gen_r==4)?bv4 :(gen_r==5)?bv5 :(gen_r==6)?bv6 :(gen_r==7)?bv7 :(gen_r==8)?bv8 :(gen_r==9)?bv9 :(gen_r==10)?bv10:(gen_r==11)?bv11:(gen_r==12)?bv12:(gen_r==13)?bv13:(gen_r==14)?bv14:bv15;
    assign ne_bc[0] =(gen_r==0)?bc0 :(gen_r==1)?bc1 :(gen_r==2)?bc2 :(gen_r==3)?bc3 :(gen_r==4)?bc4 :(gen_r==5)?bc5 :(gen_r==6)?bc6 :(gen_r==7)?bc7 :(gen_r==8)?bc8 :(gen_r==9)?bc9 :(gen_r==10)?bc10:(gen_r==11)?bc11:(gen_r==12)?bc12:(gen_r==13)?bc13:(gen_r==14)?bc14:bc15;
    assign ne_bv[1] =(gen_r==0)?bv1 :(gen_r==1)?bv2 :(gen_r==2)?bv3 :(gen_r==3)?bv4 :(gen_r==4)?bv5 :(gen_r==5)?bv6 :(gen_r==6)?bv7 :(gen_r==7)?bv8 :(gen_r==8)?bv9 :(gen_r==9)?bv10:(gen_r==10)?bv11:(gen_r==11)?bv12:(gen_r==12)?bv13:(gen_r==13)?bv14:(gen_r==14)?bv15:bv0;
    assign ne_bc[1] =(gen_r==0)?bc1 :(gen_r==1)?bc2 :(gen_r==2)?bc3 :(gen_r==3)?bc4 :(gen_r==4)?bc5 :(gen_r==5)?bc6 :(gen_r==6)?bc7 :(gen_r==7)?bc8 :(gen_r==8)?bc9 :(gen_r==9)?bc10:(gen_r==10)?bc11:(gen_r==11)?bc12:(gen_r==12)?bc13:(gen_r==13)?bc14:(gen_r==14)?bc15:bc0;
    assign ne_bv[2] =(gen_r==0)?bv2 :(gen_r==1)?bv3 :(gen_r==2)?bv4 :(gen_r==3)?bv5 :(gen_r==4)?bv6 :(gen_r==5)?bv7 :(gen_r==6)?bv8 :(gen_r==7)?bv9 :(gen_r==8)?bv10:(gen_r==9)?bv11:(gen_r==10)?bv12:(gen_r==11)?bv13:(gen_r==12)?bv14:(gen_r==13)?bv15:(gen_r==14)?bv0:bv1;
    assign ne_bc[2] =(gen_r==0)?bc2 :(gen_r==1)?bc3 :(gen_r==2)?bc4 :(gen_r==3)?bc5 :(gen_r==4)?bc6 :(gen_r==5)?bc7 :(gen_r==6)?bc8 :(gen_r==7)?bc9 :(gen_r==8)?bc10:(gen_r==9)?bc11:(gen_r==10)?bc12:(gen_r==11)?bc13:(gen_r==12)?bc14:(gen_r==13)?bc15:(gen_r==14)?bc0:bc1;
    assign ne_bv[3] =(gen_r==0)?bv3 :(gen_r==1)?bv4 :(gen_r==2)?bv5 :(gen_r==3)?bv6 :(gen_r==4)?bv7 :(gen_r==5)?bv8 :(gen_r==6)?bv9 :(gen_r==7)?bv10:(gen_r==8)?bv11:(gen_r==9)?bv12:(gen_r==10)?bv13:(gen_r==11)?bv14:(gen_r==12)?bv15:(gen_r==13)?bv0:(gen_r==14)?bv1:bv2;
    assign ne_bc[3] =(gen_r==0)?bc3 :(gen_r==1)?bc4 :(gen_r==2)?bc5 :(gen_r==3)?bc6 :(gen_r==4)?bc7 :(gen_r==5)?bc8 :(gen_r==6)?bc9 :(gen_r==7)?bc10:(gen_r==8)?bc11:(gen_r==9)?bc12:(gen_r==10)?bc13:(gen_r==11)?bc14:(gen_r==12)?bc15:(gen_r==13)?bc0:(gen_r==14)?bc1:bc2;
    assign ne_bv[4] =(gen_r==0)?bv4 :(gen_r==1)?bv5 :(gen_r==2)?bv6 :(gen_r==3)?bv7 :(gen_r==4)?bv8 :(gen_r==5)?bv9 :(gen_r==6)?bv10:(gen_r==7)?bv11:(gen_r==8)?bv12:(gen_r==9)?bv13:(gen_r==10)?bv14:(gen_r==11)?bv15:(gen_r==12)?bv0:(gen_r==13)?bv1:(gen_r==14)?bv2:bv3;
    assign ne_bc[4] =(gen_r==0)?bc4 :(gen_r==1)?bc5 :(gen_r==2)?bc6 :(gen_r==3)?bc7 :(gen_r==4)?bc8 :(gen_r==5)?bc9 :(gen_r==6)?bc10:(gen_r==7)?bc11:(gen_r==8)?bc12:(gen_r==9)?bc13:(gen_r==10)?bc14:(gen_r==11)?bc15:(gen_r==12)?bc0:(gen_r==13)?bc1:(gen_r==14)?bc2:bc3;
    assign ne_bv[5] =(gen_r==0)?bv5 :(gen_r==1)?bv6 :(gen_r==2)?bv7 :(gen_r==3)?bv8 :(gen_r==4)?bv9 :(gen_r==5)?bv10:(gen_r==6)?bv11:(gen_r==7)?bv12:(gen_r==8)?bv13:(gen_r==9)?bv14:(gen_r==10)?bv15:(gen_r==11)?bv0:(gen_r==12)?bv1:(gen_r==13)?bv2:(gen_r==14)?bv3:bv4;
    assign ne_bc[5] =(gen_r==0)?bc5 :(gen_r==1)?bc6 :(gen_r==2)?bc7 :(gen_r==3)?bc8 :(gen_r==4)?bc9 :(gen_r==5)?bc10:(gen_r==6)?bc11:(gen_r==7)?bc12:(gen_r==8)?bc13:(gen_r==9)?bc14:(gen_r==10)?bc15:(gen_r==11)?bc0:(gen_r==12)?bc1:(gen_r==13)?bc2:(gen_r==14)?bc3:bc4;
    assign ne_bv[6] =(gen_r==0)?bv6 :(gen_r==1)?bv7 :(gen_r==2)?bv8 :(gen_r==3)?bv9 :(gen_r==4)?bv10:(gen_r==5)?bv11:(gen_r==6)?bv12:(gen_r==7)?bv13:(gen_r==8)?bv14:(gen_r==9)?bv15:(gen_r==10)?bv0:(gen_r==11)?bv1:(gen_r==12)?bv2:(gen_r==13)?bv3:(gen_r==14)?bv4:bv5;
    assign ne_bc[6] =(gen_r==0)?bc6 :(gen_r==1)?bc7 :(gen_r==2)?bc8 :(gen_r==3)?bc9 :(gen_r==4)?bc10:(gen_r==5)?bc11:(gen_r==6)?bc12:(gen_r==7)?bc13:(gen_r==8)?bc14:(gen_r==9)?bc15:(gen_r==10)?bc0:(gen_r==11)?bc1:(gen_r==12)?bc2:(gen_r==13)?bc3:(gen_r==14)?bc4:bc5;
    assign ne_bv[7] =(gen_r==0)?bv7 :(gen_r==1)?bv8 :(gen_r==2)?bv9 :(gen_r==3)?bv10:(gen_r==4)?bv11:(gen_r==5)?bv12:(gen_r==6)?bv13:(gen_r==7)?bv14:(gen_r==8)?bv15:(gen_r==9)?bv0:(gen_r==10)?bv1:(gen_r==11)?bv2:(gen_r==12)?bv3:(gen_r==13)?bv4:(gen_r==14)?bv5:bv6;
    assign ne_bc[7] =(gen_r==0)?bc7 :(gen_r==1)?bc8 :(gen_r==2)?bc9 :(gen_r==3)?bc10:(gen_r==4)?bc11:(gen_r==5)?bc12:(gen_r==6)?bc13:(gen_r==7)?bc14:(gen_r==8)?bc15:(gen_r==9)?bc0:(gen_r==10)?bc1:(gen_r==11)?bc2:(gen_r==12)?bc3:(gen_r==13)?bc4:(gen_r==14)?bc5:bc6;
    assign ne_bv[8] =(gen_r==0)?bv8 :(gen_r==1)?bv9 :(gen_r==2)?bv10:(gen_r==3)?bv11:(gen_r==4)?bv12:(gen_r==5)?bv13:(gen_r==6)?bv14:(gen_r==7)?bv15:(gen_r==8)?bv0:(gen_r==9)?bv1:(gen_r==10)?bv2:(gen_r==11)?bv3:(gen_r==12)?bv4:(gen_r==13)?bv5:(gen_r==14)?bv6:bv7;
    assign ne_bc[8] =(gen_r==0)?bc8 :(gen_r==1)?bc9 :(gen_r==2)?bc10:(gen_r==3)?bc11:(gen_r==4)?bc12:(gen_r==5)?bc13:(gen_r==6)?bc14:(gen_r==7)?bc15:(gen_r==8)?bc0:(gen_r==9)?bc1:(gen_r==10)?bc2:(gen_r==11)?bc3:(gen_r==12)?bc4:(gen_r==13)?bc5:(gen_r==14)?bc6:bc7;
    assign ne_bv[9] =(gen_r==0)?bv9 :(gen_r==1)?bv10:(gen_r==2)?bv11:(gen_r==3)?bv12:(gen_r==4)?bv13:(gen_r==5)?bv14:(gen_r==6)?bv15:(gen_r==7)?bv0:(gen_r==8)?bv1:(gen_r==9)?bv2:(gen_r==10)?bv3:(gen_r==11)?bv4:(gen_r==12)?bv5:(gen_r==13)?bv6:(gen_r==14)?bv7:bv8;
    assign ne_bc[9] =(gen_r==0)?bc9 :(gen_r==1)?bc10:(gen_r==2)?bc11:(gen_r==3)?bc12:(gen_r==4)?bc13:(gen_r==5)?bc14:(gen_r==6)?bc15:(gen_r==7)?bc0:(gen_r==8)?bc1:(gen_r==9)?bc2:(gen_r==10)?bc3:(gen_r==11)?bc4:(gen_r==12)?bc5:(gen_r==13)?bc6:(gen_r==14)?bc7:bc8;
    assign ne_bv[10]=(gen_r==0)?bv10:(gen_r==1)?bv11:(gen_r==2)?bv12:(gen_r==3)?bv13:(gen_r==4)?bv14:(gen_r==5)?bv15:(gen_r==6)?bv0:(gen_r==7)?bv1:(gen_r==8)?bv2:(gen_r==9)?bv3:(gen_r==10)?bv4:(gen_r==11)?bv5:(gen_r==12)?bv6:(gen_r==13)?bv7:(gen_r==14)?bv8:bv9;
    assign ne_bc[10]=(gen_r==0)?bc10:(gen_r==1)?bc11:(gen_r==2)?bc12:(gen_r==3)?bc13:(gen_r==4)?bc14:(gen_r==5)?bc15:(gen_r==6)?bc0:(gen_r==7)?bc1:(gen_r==8)?bc2:(gen_r==9)?bc3:(gen_r==10)?bc4:(gen_r==11)?bc5:(gen_r==12)?bc6:(gen_r==13)?bc7:(gen_r==14)?bc8:bc9;
    assign ne_bv[11]=(gen_r==0)?bv11:(gen_r==1)?bv12:(gen_r==2)?bv13:(gen_r==3)?bv14:(gen_r==4)?bv15:(gen_r==5)?bv0:(gen_r==6)?bv1:(gen_r==7)?bv2:(gen_r==8)?bv3:(gen_r==9)?bv4:(gen_r==10)?bv5:(gen_r==11)?bv6:(gen_r==12)?bv7:(gen_r==13)?bv8:(gen_r==14)?bv9:bv10;
    assign ne_bc[11]=(gen_r==0)?bc11:(gen_r==1)?bc12:(gen_r==2)?bc13:(gen_r==3)?bc14:(gen_r==4)?bc15:(gen_r==5)?bc0:(gen_r==6)?bc1:(gen_r==7)?bc2:(gen_r==8)?bc3:(gen_r==9)?bc4:(gen_r==10)?bc5:(gen_r==11)?bc6:(gen_r==12)?bc7:(gen_r==13)?bc8:(gen_r==14)?bc9:bc10;
    assign ne_bv[12]=(gen_r==0)?bv12:(gen_r==1)?bv13:(gen_r==2)?bv14:(gen_r==3)?bv15:(gen_r==4)?bv0:(gen_r==5)?bv1:(gen_r==6)?bv2:(gen_r==7)?bv3:(gen_r==8)?bv4:(gen_r==9)?bv5:(gen_r==10)?bv6:(gen_r==11)?bv7:(gen_r==12)?bv8:(gen_r==13)?bv9:(gen_r==14)?bv10:bv11;
    assign ne_bc[12]=(gen_r==0)?bc12:(gen_r==1)?bc13:(gen_r==2)?bc14:(gen_r==3)?bc15:(gen_r==4)?bc0:(gen_r==5)?bc1:(gen_r==6)?bc2:(gen_r==7)?bc3:(gen_r==8)?bc4:(gen_r==9)?bc5:(gen_r==10)?bc6:(gen_r==11)?bc7:(gen_r==12)?bc8:(gen_r==13)?bc9:(gen_r==14)?bc10:bc11;
    assign ne_bv[13]=(gen_r==0)?bv13:(gen_r==1)?bv14:(gen_r==2)?bv15:(gen_r==3)?bv0:(gen_r==4)?bv1:(gen_r==5)?bv2:(gen_r==6)?bv3:(gen_r==7)?bv4:(gen_r==8)?bv5:(gen_r==9)?bv6:(gen_r==10)?bv7:(gen_r==11)?bv8:(gen_r==12)?bv9:(gen_r==13)?bv10:(gen_r==14)?bv11:bv12;
    assign ne_bc[13]=(gen_r==0)?bc13:(gen_r==1)?bc14:(gen_r==2)?bc15:(gen_r==3)?bc0:(gen_r==4)?bc1:(gen_r==5)?bc2:(gen_r==6)?bc3:(gen_r==7)?bc4:(gen_r==8)?bc5:(gen_r==9)?bc6:(gen_r==10)?bc7:(gen_r==11)?bc8:(gen_r==12)?bc9:(gen_r==13)?bc10:(gen_r==14)?bc11:bc12;
    assign ne_bv[14]=(gen_r==0)?bv14:(gen_r==1)?bv15:(gen_r==2)?bv0:(gen_r==3)?bv1:(gen_r==4)?bv2:(gen_r==5)?bv3:(gen_r==6)?bv4:(gen_r==7)?bv5:(gen_r==8)?bv6:(gen_r==9)?bv7:(gen_r==10)?bv8:(gen_r==11)?bv9:(gen_r==12)?bv10:(gen_r==13)?bv11:(gen_r==14)?bv12:bv13;
    assign ne_bc[14]=(gen_r==0)?bc14:(gen_r==1)?bc15:(gen_r==2)?bc0:(gen_r==3)?bc1:(gen_r==4)?bc2:(gen_r==5)?bc3:(gen_r==6)?bc4:(gen_r==7)?bc5:(gen_r==8)?bc6:(gen_r==9)?bc7:(gen_r==10)?bc8:(gen_r==11)?bc9:(gen_r==12)?bc10:(gen_r==13)?bc11:(gen_r==14)?bc12:bc13;
    assign ne_bv[15]=(gen_r==0)?bv15:(gen_r==1)?bv0:(gen_r==2)?bv1:(gen_r==3)?bv2:(gen_r==4)?bv3:(gen_r==5)?bv4:(gen_r==6)?bv5:(gen_r==7)?bv6:(gen_r==8)?bv7:(gen_r==9)?bv8:(gen_r==10)?bv9:(gen_r==11)?bv10:(gen_r==12)?bv11:(gen_r==13)?bv12:(gen_r==14)?bv13:bv14;
    assign ne_bc[15]=(gen_r==0)?bc15:(gen_r==1)?bc0:(gen_r==2)?bc1:(gen_r==3)?bc2:(gen_r==4)?bc3:(gen_r==5)?bc4:(gen_r==6)?bc5:(gen_r==7)?bc6:(gen_r==8)?bc7:(gen_r==9)?bc8:(gen_r==10)?bc9:(gen_r==11)?bc10:(gen_r==12)?bc11:(gen_r==13)?bc12:(gen_r==14)?bc13:bc14;

    wire [`TASK_WIDTH-1:0] pack_sg0 ={ne_bv[0], gen_a_val,ne_bc[0][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg1 ={ne_bv[1], gen_a_val,ne_bc[1][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg2 ={ne_bv[2], gen_a_val,ne_bc[2][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg3 ={ne_bv[3], gen_a_val,ne_bc[3][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg4 ={ne_bv[4], gen_a_val,ne_bc[4][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg5 ={ne_bv[5], gen_a_val,ne_bc[5][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg6 ={ne_bv[6], gen_a_val,ne_bc[6][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg7 ={ne_bv[7], gen_a_val,ne_bc[7][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg8 ={ne_bv[8], gen_a_val,ne_bc[8][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg9 ={ne_bv[9], gen_a_val,ne_bc[9][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg10={ne_bv[10],gen_a_val,ne_bc[10][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg11={ne_bv[11],gen_a_val,ne_bc[11][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg12={ne_bv[12],gen_a_val,ne_bc[12][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg13={ne_bv[13],gen_a_val,ne_bc[13][8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg14={ne_bv[14],gen_a_val,ne_bc[14][8:0]};

    //=========================================================================
    // Gen2: accumulate cross-A-nnz remainders (up to 15 carry elements)
    //=========================================================================
    reg [3:0]             carry2_cnt;
    reg [`TASK_WIDTH-1:0] carry2_task [0:14];

    wire [4:0] g2_combined = {1'b0, carry2_cnt} + {1'b0, gen_remainder};
    wire       g2_can_emit = g2_combined[4];
    wire [3:0] g2_overflow = g2_combined[3:0];

    // g2_sg_j = carry2_task[j] if j < carry2_cnt, else pack_sg[j - carry2_cnt]
    wire [`TASK_WIDTH-1:0] g2_sg0 =(carry2_cnt>=1)?carry2_task[0]:pack_sg0;
    wire [`TASK_WIDTH-1:0] g2_sg1 =(carry2_cnt>=2)?carry2_task[1]:(carry2_cnt==1)?pack_sg0:pack_sg1;
    wire [`TASK_WIDTH-1:0] g2_sg2 =(carry2_cnt>=3)?carry2_task[2]:(carry2_cnt==2)?pack_sg0:(carry2_cnt==1)?pack_sg1:pack_sg2;
    wire [`TASK_WIDTH-1:0] g2_sg3 =(carry2_cnt>=4)?carry2_task[3]:(carry2_cnt==3)?pack_sg0:(carry2_cnt==2)?pack_sg1:(carry2_cnt==1)?pack_sg2:pack_sg3;
    wire [`TASK_WIDTH-1:0] g2_sg4 =(carry2_cnt>=5)?carry2_task[4]:(carry2_cnt==4)?pack_sg0:(carry2_cnt==3)?pack_sg1:(carry2_cnt==2)?pack_sg2:(carry2_cnt==1)?pack_sg3:pack_sg4;
    wire [`TASK_WIDTH-1:0] g2_sg5 =(carry2_cnt>=6)?carry2_task[5]:(carry2_cnt==5)?pack_sg0:(carry2_cnt==4)?pack_sg1:(carry2_cnt==3)?pack_sg2:(carry2_cnt==2)?pack_sg3:(carry2_cnt==1)?pack_sg4:pack_sg5;
    wire [`TASK_WIDTH-1:0] g2_sg6 =(carry2_cnt>=7)?carry2_task[6]:(carry2_cnt==6)?pack_sg0:(carry2_cnt==5)?pack_sg1:(carry2_cnt==4)?pack_sg2:(carry2_cnt==3)?pack_sg3:(carry2_cnt==2)?pack_sg4:(carry2_cnt==1)?pack_sg5:pack_sg6;
    wire [`TASK_WIDTH-1:0] g2_sg7 =(carry2_cnt>=8)?carry2_task[7]:(carry2_cnt==7)?pack_sg0:(carry2_cnt==6)?pack_sg1:(carry2_cnt==5)?pack_sg2:(carry2_cnt==4)?pack_sg3:(carry2_cnt==3)?pack_sg4:(carry2_cnt==2)?pack_sg5:(carry2_cnt==1)?pack_sg6:pack_sg7;
    wire [`TASK_WIDTH-1:0] g2_sg8 =(carry2_cnt>=9)?carry2_task[8]:(carry2_cnt==8)?pack_sg0:(carry2_cnt==7)?pack_sg1:(carry2_cnt==6)?pack_sg2:(carry2_cnt==5)?pack_sg3:(carry2_cnt==4)?pack_sg4:(carry2_cnt==3)?pack_sg5:(carry2_cnt==2)?pack_sg6:(carry2_cnt==1)?pack_sg7:pack_sg8;
    wire [`TASK_WIDTH-1:0] g2_sg9 =(carry2_cnt>=10)?carry2_task[9]:(carry2_cnt==9)?pack_sg0:(carry2_cnt==8)?pack_sg1:(carry2_cnt==7)?pack_sg2:(carry2_cnt==6)?pack_sg3:(carry2_cnt==5)?pack_sg4:(carry2_cnt==4)?pack_sg5:(carry2_cnt==3)?pack_sg6:(carry2_cnt==2)?pack_sg7:(carry2_cnt==1)?pack_sg8:pack_sg9;
    wire [`TASK_WIDTH-1:0] g2_sg10=(carry2_cnt>=11)?carry2_task[10]:(carry2_cnt==10)?pack_sg0:(carry2_cnt==9)?pack_sg1:(carry2_cnt==8)?pack_sg2:(carry2_cnt==7)?pack_sg3:(carry2_cnt==6)?pack_sg4:(carry2_cnt==5)?pack_sg5:(carry2_cnt==4)?pack_sg6:(carry2_cnt==3)?pack_sg7:(carry2_cnt==2)?pack_sg8:(carry2_cnt==1)?pack_sg9:pack_sg10;
    wire [`TASK_WIDTH-1:0] g2_sg11=(carry2_cnt>=12)?carry2_task[11]:(carry2_cnt==11)?pack_sg0:(carry2_cnt==10)?pack_sg1:(carry2_cnt==9)?pack_sg2:(carry2_cnt==8)?pack_sg3:(carry2_cnt==7)?pack_sg4:(carry2_cnt==6)?pack_sg5:(carry2_cnt==5)?pack_sg6:(carry2_cnt==4)?pack_sg7:(carry2_cnt==3)?pack_sg8:(carry2_cnt==2)?pack_sg9:(carry2_cnt==1)?pack_sg10:pack_sg11;
    wire [`TASK_WIDTH-1:0] g2_sg12=(carry2_cnt>=13)?carry2_task[12]:(carry2_cnt==12)?pack_sg0:(carry2_cnt==11)?pack_sg1:(carry2_cnt==10)?pack_sg2:(carry2_cnt==9)?pack_sg3:(carry2_cnt==8)?pack_sg4:(carry2_cnt==7)?pack_sg5:(carry2_cnt==6)?pack_sg6:(carry2_cnt==5)?pack_sg7:(carry2_cnt==4)?pack_sg8:(carry2_cnt==3)?pack_sg9:(carry2_cnt==2)?pack_sg10:(carry2_cnt==1)?pack_sg11:pack_sg12;
    wire [`TASK_WIDTH-1:0] g2_sg13=(carry2_cnt>=14)?carry2_task[13]:(carry2_cnt==13)?pack_sg0:(carry2_cnt==12)?pack_sg1:(carry2_cnt==11)?pack_sg2:(carry2_cnt==10)?pack_sg3:(carry2_cnt==9)?pack_sg4:(carry2_cnt==8)?pack_sg5:(carry2_cnt==7)?pack_sg6:(carry2_cnt==6)?pack_sg7:(carry2_cnt==5)?pack_sg8:(carry2_cnt==4)?pack_sg9:(carry2_cnt==3)?pack_sg10:(carry2_cnt==2)?pack_sg11:(carry2_cnt==1)?pack_sg12:pack_sg13;
    wire [`TASK_WIDTH-1:0] g2_sg14=(carry2_cnt>=15)?carry2_task[14]:(carry2_cnt==14)?pack_sg0:(carry2_cnt==13)?pack_sg1:(carry2_cnt==12)?pack_sg2:(carry2_cnt==11)?pack_sg3:(carry2_cnt==10)?pack_sg4:(carry2_cnt==9)?pack_sg5:(carry2_cnt==8)?pack_sg6:(carry2_cnt==7)?pack_sg7:(carry2_cnt==6)?pack_sg8:(carry2_cnt==5)?pack_sg9:(carry2_cnt==4)?pack_sg10:(carry2_cnt==3)?pack_sg11:(carry2_cnt==2)?pack_sg12:(carry2_cnt==1)?pack_sg13:pack_sg14;
    wire [`TASK_WIDTH-1:0] g2_sg15=(carry2_cnt==15)?pack_sg0:(carry2_cnt==14)?pack_sg1:(carry2_cnt==13)?pack_sg2:(carry2_cnt==12)?pack_sg3:(carry2_cnt==11)?pack_sg4:(carry2_cnt==10)?pack_sg5:(carry2_cnt==9)?pack_sg6:(carry2_cnt==8)?pack_sg7:(carry2_cnt==7)?pack_sg8:(carry2_cnt==6)?pack_sg9:(carry2_cnt==5)?pack_sg10:(carry2_cnt==4)?pack_sg11:(carry2_cnt==3)?pack_sg12:(carry2_cnt==2)?pack_sg13:(carry2_cnt==1)?pack_sg14:pack_sg0;

    wire [15:0] g2_flush_lane_valid = (16'd1 << carry2_cnt) - 16'd1;

    wire task_fifo_full;
    wire ptr_fifo_full;

    wire g1_to_g2_valid = (gen_state == GEN_EMIT) && (gen_remainder != 4'd0);
    wire g2_want_emit   = g1_to_g2_valid && g2_can_emit;
    wire g2_want_flush  = (gen_state == GEN_ROW_DONE) && (carry2_cnt != 4'd0);

    wire gen_emit_stall =
        (gen_num_groups != 16'd0 && ptr_fifo_full) ||
        (gen_remainder  != 4'd0  && g2_can_emit && task_fifo_full);
    wire gen_emit_can_advance = (gen_state == GEN_EMIT) && !gen_emit_stall;
    wire g1_acc_advances      = gen_emit_can_advance && g1_to_g2_valid;

    wire ptr_fifo_wr_en =
        (gen_state == GEN_EMIT) && gen_emit_can_advance && (gen_num_groups != 16'd0);
    wire [`PTR_TASK_WIDTH-1:0] ptr_fifo_wr_data =
        {gen_a_val, gen_b_off[16:0], gen_num_groups[6:0]};

    wire task_group_wr_en = (g2_want_emit || g2_want_flush) && !task_fifo_full;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data =
        g2_want_flush
        ? {carry2_task[0],carry2_task[14],carry2_task[13],carry2_task[12],
           carry2_task[11],carry2_task[10],carry2_task[9],carry2_task[8],
           carry2_task[7],carry2_task[6],carry2_task[5],carry2_task[4],
           carry2_task[3],carry2_task[2],carry2_task[1],carry2_task[0],
           g2_flush_lane_valid}
        : {g2_sg15,g2_sg14,g2_sg13,g2_sg12,g2_sg11,g2_sg10,g2_sg9,g2_sg8,
           g2_sg7,g2_sg6,g2_sg5,g2_sg4,g2_sg3,g2_sg2,g2_sg1,g2_sg0,16'hFFFF};

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
            carry2_task[0]<=0; carry2_task[1]<=0; carry2_task[2]<=0; carry2_task[3]<=0;
            carry2_task[4]<=0; carry2_task[5]<=0; carry2_task[6]<=0; carry2_task[7]<=0;
            carry2_task[8]<=0; carry2_task[9]<=0; carry2_task[10]<=0; carry2_task[11]<=0;
            carry2_task[12]<=0; carry2_task[13]<=0; carry2_task[14]<=0;
        end else begin
            if (g1_acc_advances) begin
                if (g2_can_emit) begin
                    carry2_cnt <= g2_overflow;
                    // new carry: pack_sg[16-carry2_cnt .. gen_remainder-1]
                    case (carry2_cnt)
                        4'd2:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg14; end
                        4'd3:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg13; if(g2_overflow>=2) carry2_task[1]<=pack_sg14; end
                        4'd4:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg12; if(g2_overflow>=2) carry2_task[1]<=pack_sg13; if(g2_overflow>=3) carry2_task[2]<=pack_sg14; end
                        4'd5:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg11; if(g2_overflow>=2) carry2_task[1]<=pack_sg12; if(g2_overflow>=3) carry2_task[2]<=pack_sg13; if(g2_overflow>=4) carry2_task[3]<=pack_sg14; end
                        4'd6:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg10; if(g2_overflow>=2) carry2_task[1]<=pack_sg11; if(g2_overflow>=3) carry2_task[2]<=pack_sg12; if(g2_overflow>=4) carry2_task[3]<=pack_sg13; if(g2_overflow>=5) carry2_task[4]<=pack_sg14; end
                        4'd7:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg9;  if(g2_overflow>=2) carry2_task[1]<=pack_sg10; if(g2_overflow>=3) carry2_task[2]<=pack_sg11; if(g2_overflow>=4) carry2_task[3]<=pack_sg12; if(g2_overflow>=5) carry2_task[4]<=pack_sg13; if(g2_overflow>=6) carry2_task[5]<=pack_sg14; end
                        4'd8:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg8;  if(g2_overflow>=2) carry2_task[1]<=pack_sg9;  if(g2_overflow>=3) carry2_task[2]<=pack_sg10; if(g2_overflow>=4) carry2_task[3]<=pack_sg11; if(g2_overflow>=5) carry2_task[4]<=pack_sg12; if(g2_overflow>=6) carry2_task[5]<=pack_sg13; if(g2_overflow>=7) carry2_task[6]<=pack_sg14; end
                        4'd9:  begin if(g2_overflow>=1) carry2_task[0]<=pack_sg7;  if(g2_overflow>=2) carry2_task[1]<=pack_sg8;  if(g2_overflow>=3) carry2_task[2]<=pack_sg9;  if(g2_overflow>=4) carry2_task[3]<=pack_sg10; if(g2_overflow>=5) carry2_task[4]<=pack_sg11; if(g2_overflow>=6) carry2_task[5]<=pack_sg12; if(g2_overflow>=7) carry2_task[6]<=pack_sg13; if(g2_overflow>=8) carry2_task[7]<=pack_sg14; end
                        4'd10: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg6;  if(g2_overflow>=2) carry2_task[1]<=pack_sg7;  if(g2_overflow>=3) carry2_task[2]<=pack_sg8;  if(g2_overflow>=4) carry2_task[3]<=pack_sg9;  if(g2_overflow>=5) carry2_task[4]<=pack_sg10; if(g2_overflow>=6) carry2_task[5]<=pack_sg11; if(g2_overflow>=7) carry2_task[6]<=pack_sg12; if(g2_overflow>=8) carry2_task[7]<=pack_sg13; if(g2_overflow>=9) carry2_task[8]<=pack_sg14; end
                        4'd11: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg5;  if(g2_overflow>=2) carry2_task[1]<=pack_sg6;  if(g2_overflow>=3) carry2_task[2]<=pack_sg7;  if(g2_overflow>=4) carry2_task[3]<=pack_sg8;  if(g2_overflow>=5) carry2_task[4]<=pack_sg9;  if(g2_overflow>=6) carry2_task[5]<=pack_sg10; if(g2_overflow>=7) carry2_task[6]<=pack_sg11; if(g2_overflow>=8) carry2_task[7]<=pack_sg12; if(g2_overflow>=9) carry2_task[8]<=pack_sg13; if(g2_overflow>=10) carry2_task[9]<=pack_sg14; end
                        4'd12: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg4;  if(g2_overflow>=2) carry2_task[1]<=pack_sg5;  if(g2_overflow>=3) carry2_task[2]<=pack_sg6;  if(g2_overflow>=4) carry2_task[3]<=pack_sg7;  if(g2_overflow>=5) carry2_task[4]<=pack_sg8;  if(g2_overflow>=6) carry2_task[5]<=pack_sg9;  if(g2_overflow>=7) carry2_task[6]<=pack_sg10; if(g2_overflow>=8) carry2_task[7]<=pack_sg11; if(g2_overflow>=9) carry2_task[8]<=pack_sg12; if(g2_overflow>=10) carry2_task[9]<=pack_sg13; if(g2_overflow>=11) carry2_task[10]<=pack_sg14; end
                        4'd13: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg3;  if(g2_overflow>=2) carry2_task[1]<=pack_sg4;  if(g2_overflow>=3) carry2_task[2]<=pack_sg5;  if(g2_overflow>=4) carry2_task[3]<=pack_sg6;  if(g2_overflow>=5) carry2_task[4]<=pack_sg7;  if(g2_overflow>=6) carry2_task[5]<=pack_sg8;  if(g2_overflow>=7) carry2_task[6]<=pack_sg9;  if(g2_overflow>=8) carry2_task[7]<=pack_sg10; if(g2_overflow>=9) carry2_task[8]<=pack_sg11; if(g2_overflow>=10) carry2_task[9]<=pack_sg12; if(g2_overflow>=11) carry2_task[10]<=pack_sg13; if(g2_overflow>=12) carry2_task[11]<=pack_sg14; end
                        4'd14: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg2;  if(g2_overflow>=2) carry2_task[1]<=pack_sg3;  if(g2_overflow>=3) carry2_task[2]<=pack_sg4;  if(g2_overflow>=4) carry2_task[3]<=pack_sg5;  if(g2_overflow>=5) carry2_task[4]<=pack_sg6;  if(g2_overflow>=6) carry2_task[5]<=pack_sg7;  if(g2_overflow>=7) carry2_task[6]<=pack_sg8;  if(g2_overflow>=8) carry2_task[7]<=pack_sg9;  if(g2_overflow>=9) carry2_task[8]<=pack_sg10; if(g2_overflow>=10) carry2_task[9]<=pack_sg11; if(g2_overflow>=11) carry2_task[10]<=pack_sg12; if(g2_overflow>=12) carry2_task[11]<=pack_sg13; if(g2_overflow>=13) carry2_task[12]<=pack_sg14; end
                        4'd15: begin if(g2_overflow>=1) carry2_task[0]<=pack_sg1;  if(g2_overflow>=2) carry2_task[1]<=pack_sg2;  if(g2_overflow>=3) carry2_task[2]<=pack_sg3;  if(g2_overflow>=4) carry2_task[3]<=pack_sg4;  if(g2_overflow>=5) carry2_task[4]<=pack_sg5;  if(g2_overflow>=6) carry2_task[5]<=pack_sg6;  if(g2_overflow>=7) carry2_task[6]<=pack_sg7;  if(g2_overflow>=8) carry2_task[7]<=pack_sg8;  if(g2_overflow>=9) carry2_task[8]<=pack_sg9;  if(g2_overflow>=10) carry2_task[9]<=pack_sg10; if(g2_overflow>=11) carry2_task[10]<=pack_sg11; if(g2_overflow>=12) carry2_task[11]<=pack_sg12; if(g2_overflow>=13) carry2_task[12]<=pack_sg13; if(g2_overflow>=14) carry2_task[13]<=pack_sg14; end
                        default: ; // carry2_cnt 0 or 1: overflow always 0
                    endcase
                end else begin
                    carry2_cnt <= g2_combined[3:0];
                    // accumulate: carry2_task[carry2_cnt + j] = pack_sg[j] for j=0..gen_remainder-1
                    case (carry2_cnt)
                        4'd0:  begin if(gen_remainder>=1) carry2_task[0]<=pack_sg0; if(gen_remainder>=2) carry2_task[1]<=pack_sg1; if(gen_remainder>=3) carry2_task[2]<=pack_sg2; if(gen_remainder>=4) carry2_task[3]<=pack_sg3; if(gen_remainder>=5) carry2_task[4]<=pack_sg4; if(gen_remainder>=6) carry2_task[5]<=pack_sg5; if(gen_remainder>=7) carry2_task[6]<=pack_sg6; if(gen_remainder>=8) carry2_task[7]<=pack_sg7; if(gen_remainder>=9) carry2_task[8]<=pack_sg8; if(gen_remainder>=10) carry2_task[9]<=pack_sg9; if(gen_remainder>=11) carry2_task[10]<=pack_sg10; if(gen_remainder>=12) carry2_task[11]<=pack_sg11; if(gen_remainder>=13) carry2_task[12]<=pack_sg12; if(gen_remainder>=14) carry2_task[13]<=pack_sg13; if(gen_remainder>=15) carry2_task[14]<=pack_sg14; end
                        4'd1:  begin if(gen_remainder>=1) carry2_task[1]<=pack_sg0; if(gen_remainder>=2) carry2_task[2]<=pack_sg1; if(gen_remainder>=3) carry2_task[3]<=pack_sg2; if(gen_remainder>=4) carry2_task[4]<=pack_sg3; if(gen_remainder>=5) carry2_task[5]<=pack_sg4; if(gen_remainder>=6) carry2_task[6]<=pack_sg5; if(gen_remainder>=7) carry2_task[7]<=pack_sg6; if(gen_remainder>=8) carry2_task[8]<=pack_sg7; if(gen_remainder>=9) carry2_task[9]<=pack_sg8; if(gen_remainder>=10) carry2_task[10]<=pack_sg9; if(gen_remainder>=11) carry2_task[11]<=pack_sg10; if(gen_remainder>=12) carry2_task[12]<=pack_sg11; if(gen_remainder>=13) carry2_task[13]<=pack_sg12; if(gen_remainder>=14) carry2_task[14]<=pack_sg13; end
                        4'd2:  begin if(gen_remainder>=1) carry2_task[2]<=pack_sg0; if(gen_remainder>=2) carry2_task[3]<=pack_sg1; if(gen_remainder>=3) carry2_task[4]<=pack_sg2; if(gen_remainder>=4) carry2_task[5]<=pack_sg3; if(gen_remainder>=5) carry2_task[6]<=pack_sg4; if(gen_remainder>=6) carry2_task[7]<=pack_sg5; if(gen_remainder>=7) carry2_task[8]<=pack_sg6; if(gen_remainder>=8) carry2_task[9]<=pack_sg7; if(gen_remainder>=9) carry2_task[10]<=pack_sg8; if(gen_remainder>=10) carry2_task[11]<=pack_sg9; if(gen_remainder>=11) carry2_task[12]<=pack_sg10; if(gen_remainder>=12) carry2_task[13]<=pack_sg11; if(gen_remainder>=13) carry2_task[14]<=pack_sg12; end
                        4'd3:  begin if(gen_remainder>=1) carry2_task[3]<=pack_sg0; if(gen_remainder>=2) carry2_task[4]<=pack_sg1; if(gen_remainder>=3) carry2_task[5]<=pack_sg2; if(gen_remainder>=4) carry2_task[6]<=pack_sg3; if(gen_remainder>=5) carry2_task[7]<=pack_sg4; if(gen_remainder>=6) carry2_task[8]<=pack_sg5; if(gen_remainder>=7) carry2_task[9]<=pack_sg6; if(gen_remainder>=8) carry2_task[10]<=pack_sg7; if(gen_remainder>=9) carry2_task[11]<=pack_sg8; if(gen_remainder>=10) carry2_task[12]<=pack_sg9; if(gen_remainder>=11) carry2_task[13]<=pack_sg10; if(gen_remainder>=12) carry2_task[14]<=pack_sg11; end
                        4'd4:  begin if(gen_remainder>=1) carry2_task[4]<=pack_sg0; if(gen_remainder>=2) carry2_task[5]<=pack_sg1; if(gen_remainder>=3) carry2_task[6]<=pack_sg2; if(gen_remainder>=4) carry2_task[7]<=pack_sg3; if(gen_remainder>=5) carry2_task[8]<=pack_sg4; if(gen_remainder>=6) carry2_task[9]<=pack_sg5; if(gen_remainder>=7) carry2_task[10]<=pack_sg6; if(gen_remainder>=8) carry2_task[11]<=pack_sg7; if(gen_remainder>=9) carry2_task[12]<=pack_sg8; if(gen_remainder>=10) carry2_task[13]<=pack_sg9; if(gen_remainder>=11) carry2_task[14]<=pack_sg10; end
                        4'd5:  begin if(gen_remainder>=1) carry2_task[5]<=pack_sg0; if(gen_remainder>=2) carry2_task[6]<=pack_sg1; if(gen_remainder>=3) carry2_task[7]<=pack_sg2; if(gen_remainder>=4) carry2_task[8]<=pack_sg3; if(gen_remainder>=5) carry2_task[9]<=pack_sg4; if(gen_remainder>=6) carry2_task[10]<=pack_sg5; if(gen_remainder>=7) carry2_task[11]<=pack_sg6; if(gen_remainder>=8) carry2_task[12]<=pack_sg7; if(gen_remainder>=9) carry2_task[13]<=pack_sg8; if(gen_remainder>=10) carry2_task[14]<=pack_sg9; end
                        4'd6:  begin if(gen_remainder>=1) carry2_task[6]<=pack_sg0; if(gen_remainder>=2) carry2_task[7]<=pack_sg1; if(gen_remainder>=3) carry2_task[8]<=pack_sg2; if(gen_remainder>=4) carry2_task[9]<=pack_sg3; if(gen_remainder>=5) carry2_task[10]<=pack_sg4; if(gen_remainder>=6) carry2_task[11]<=pack_sg5; if(gen_remainder>=7) carry2_task[12]<=pack_sg6; if(gen_remainder>=8) carry2_task[13]<=pack_sg7; if(gen_remainder>=9) carry2_task[14]<=pack_sg8; end
                        4'd7:  begin if(gen_remainder>=1) carry2_task[7]<=pack_sg0; if(gen_remainder>=2) carry2_task[8]<=pack_sg1; if(gen_remainder>=3) carry2_task[9]<=pack_sg2; if(gen_remainder>=4) carry2_task[10]<=pack_sg3; if(gen_remainder>=5) carry2_task[11]<=pack_sg4; if(gen_remainder>=6) carry2_task[12]<=pack_sg5; if(gen_remainder>=7) carry2_task[13]<=pack_sg6; if(gen_remainder>=8) carry2_task[14]<=pack_sg7; end
                        4'd8:  begin if(gen_remainder>=1) carry2_task[8]<=pack_sg0; if(gen_remainder>=2) carry2_task[9]<=pack_sg1; if(gen_remainder>=3) carry2_task[10]<=pack_sg2; if(gen_remainder>=4) carry2_task[11]<=pack_sg3; if(gen_remainder>=5) carry2_task[12]<=pack_sg4; if(gen_remainder>=6) carry2_task[13]<=pack_sg5; if(gen_remainder>=7) carry2_task[14]<=pack_sg6; end
                        4'd9:  begin if(gen_remainder>=1) carry2_task[9]<=pack_sg0; if(gen_remainder>=2) carry2_task[10]<=pack_sg1; if(gen_remainder>=3) carry2_task[11]<=pack_sg2; if(gen_remainder>=4) carry2_task[12]<=pack_sg3; if(gen_remainder>=5) carry2_task[13]<=pack_sg4; if(gen_remainder>=6) carry2_task[14]<=pack_sg5; end
                        4'd10: begin if(gen_remainder>=1) carry2_task[10]<=pack_sg0; if(gen_remainder>=2) carry2_task[11]<=pack_sg1; if(gen_remainder>=3) carry2_task[12]<=pack_sg2; if(gen_remainder>=4) carry2_task[13]<=pack_sg3; if(gen_remainder>=5) carry2_task[14]<=pack_sg4; end
                        4'd11: begin if(gen_remainder>=1) carry2_task[11]<=pack_sg0; if(gen_remainder>=2) carry2_task[12]<=pack_sg1; if(gen_remainder>=3) carry2_task[13]<=pack_sg2; if(gen_remainder>=4) carry2_task[14]<=pack_sg3; end
                        4'd12: begin if(gen_remainder>=1) carry2_task[12]<=pack_sg0; if(gen_remainder>=2) carry2_task[13]<=pack_sg1; if(gen_remainder>=3) carry2_task[14]<=pack_sg2; end
                        4'd13: begin if(gen_remainder>=1) carry2_task[13]<=pack_sg0; if(gen_remainder>=2) carry2_task[14]<=pack_sg1; end
                        4'd14: begin if(gen_remainder>=1) carry2_task[14]<=pack_sg0; end
                        default: ;
                    endcase
                end
            end else if (g2_want_flush && !task_fifo_full) begin
                carry2_cnt <= 4'd0;
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

    // Only load next ptr entry when task_fifo is empty; otherwise yield to task path.
    assign ptr_fifo_rd_en = (exec_idle     && !ptr_fifo_empty && task_fifo_empty) ||
                            (exec_ptr_last && !ptr_fifo_empty && task_fifo_empty);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            exec_state<=EXEC_IDLE; exec_a_val<=0; exec_b_off<=0; exec_num_groups<=0; exec_g<=0;
        end else case (exec_state)
            EXEC_IDLE: begin
                if (!ptr_fifo_empty && task_fifo_empty) begin
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
                        if (!ptr_fifo_empty && task_fifo_empty) begin
                            // chain directly only when task_fifo is drained
                            exec_a_val      <= ptr_fifo_rd_data[39:24];
                            exec_b_off      <= ptr_fifo_rd_data[23:7];
                            exec_num_groups <= ptr_fifo_rd_data[6:0];
                            exec_g          <= 7'd0;
                        end else begin
                            exec_state <= EXEC_IDLE;
                        end
                    end
                end
            end
            default: exec_state<=EXEC_IDLE;
        endcase
    end

    //=========================================================================
    // Executor B bank reads (16-bank, group stride = 16)
    //=========================================================================
    wire [31:0] exec_abs_base = {15'b0, exec_b_off} + {21'b0, exec_g, 4'b0000};
    wire [3:0]  exec_r        = exec_abs_base[3:0];
    wire [13:0] exec_m        = exec_abs_base[17:4];

    wire [13:0] exec_bg0 =(exec_r==0)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg1 =(exec_r<=1)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg2 =(exec_r<=2)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg3 =(exec_r<=3)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg4 =(exec_r<=4)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg5 =(exec_r<=5)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg6 =(exec_r<=6)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg7 =(exec_r<=7)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg8 =(exec_r<=8)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg9 =(exec_r<=9)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg10=(exec_r<=10)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg11=(exec_r<=11)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg12=(exec_r<=12)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg13=(exec_r<=13)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg14=(exec_r<=14)?exec_m:exec_m+14'd1;
    wire [13:0] exec_bg15=exec_m;

    wire [15:0] ebc0 =B_col_b0 [exec_bg0];  wire [15:0] ebv0 =B_val_b0 [exec_bg0];
    wire [15:0] ebc1 =B_col_b1 [exec_bg1];  wire [15:0] ebv1 =B_val_b1 [exec_bg1];
    wire [15:0] ebc2 =B_col_b2 [exec_bg2];  wire [15:0] ebv2 =B_val_b2 [exec_bg2];
    wire [15:0] ebc3 =B_col_b3 [exec_bg3];  wire [15:0] ebv3 =B_val_b3 [exec_bg3];
    wire [15:0] ebc4 =B_col_b4 [exec_bg4];  wire [15:0] ebv4 =B_val_b4 [exec_bg4];
    wire [15:0] ebc5 =B_col_b5 [exec_bg5];  wire [15:0] ebv5 =B_val_b5 [exec_bg5];
    wire [15:0] ebc6 =B_col_b6 [exec_bg6];  wire [15:0] ebv6 =B_val_b6 [exec_bg6];
    wire [15:0] ebc7 =B_col_b7 [exec_bg7];  wire [15:0] ebv7 =B_val_b7 [exec_bg7];
    wire [15:0] ebc8 =B_col_b8 [exec_bg8];  wire [15:0] ebv8 =B_val_b8 [exec_bg8];
    wire [15:0] ebc9 =B_col_b9 [exec_bg9];  wire [15:0] ebv9 =B_val_b9 [exec_bg9];
    wire [15:0] ebc10=B_col_b10[exec_bg10]; wire [15:0] ebv10=B_val_b10[exec_bg10];
    wire [15:0] ebc11=B_col_b11[exec_bg11]; wire [15:0] ebv11=B_val_b11[exec_bg11];
    wire [15:0] ebc12=B_col_b12[exec_bg12]; wire [15:0] ebv12=B_val_b12[exec_bg12];
    wire [15:0] ebc13=B_col_b13[exec_bg13]; wire [15:0] ebv13=B_val_b13[exec_bg13];
    wire [15:0] ebc14=B_col_b14[exec_bg14]; wire [15:0] ebv14=B_val_b14[exec_bg14];
    wire [15:0] ebc15=B_col_b15[exec_bg15]; wire [15:0] ebv15=B_val_b15[exec_bg15];

    // Rotation mux: enebv[j] = ebv at bank (exec_r+j)%16
    wire [15:0] enebv [0:15]; wire [15:0] enebc [0:15];
    assign enebv[0] =(exec_r==0)?ebv0 :(exec_r==1)?ebv1 :(exec_r==2)?ebv2 :(exec_r==3)?ebv3 :(exec_r==4)?ebv4 :(exec_r==5)?ebv5 :(exec_r==6)?ebv6 :(exec_r==7)?ebv7 :(exec_r==8)?ebv8 :(exec_r==9)?ebv9 :(exec_r==10)?ebv10:(exec_r==11)?ebv11:(exec_r==12)?ebv12:(exec_r==13)?ebv13:(exec_r==14)?ebv14:ebv15;
    assign enebc[0] =(exec_r==0)?ebc0 :(exec_r==1)?ebc1 :(exec_r==2)?ebc2 :(exec_r==3)?ebc3 :(exec_r==4)?ebc4 :(exec_r==5)?ebc5 :(exec_r==6)?ebc6 :(exec_r==7)?ebc7 :(exec_r==8)?ebc8 :(exec_r==9)?ebc9 :(exec_r==10)?ebc10:(exec_r==11)?ebc11:(exec_r==12)?ebc12:(exec_r==13)?ebc13:(exec_r==14)?ebc14:ebc15;
    assign enebv[1] =(exec_r==0)?ebv1 :(exec_r==1)?ebv2 :(exec_r==2)?ebv3 :(exec_r==3)?ebv4 :(exec_r==4)?ebv5 :(exec_r==5)?ebv6 :(exec_r==6)?ebv7 :(exec_r==7)?ebv8 :(exec_r==8)?ebv9 :(exec_r==9)?ebv10:(exec_r==10)?ebv11:(exec_r==11)?ebv12:(exec_r==12)?ebv13:(exec_r==13)?ebv14:(exec_r==14)?ebv15:ebv0;
    assign enebc[1] =(exec_r==0)?ebc1 :(exec_r==1)?ebc2 :(exec_r==2)?ebc3 :(exec_r==3)?ebc4 :(exec_r==4)?ebc5 :(exec_r==5)?ebc6 :(exec_r==6)?ebc7 :(exec_r==7)?ebc8 :(exec_r==8)?ebc9 :(exec_r==9)?ebc10:(exec_r==10)?ebc11:(exec_r==11)?ebc12:(exec_r==12)?ebc13:(exec_r==13)?ebc14:(exec_r==14)?ebc15:ebc0;
    assign enebv[2] =(exec_r==0)?ebv2 :(exec_r==1)?ebv3 :(exec_r==2)?ebv4 :(exec_r==3)?ebv5 :(exec_r==4)?ebv6 :(exec_r==5)?ebv7 :(exec_r==6)?ebv8 :(exec_r==7)?ebv9 :(exec_r==8)?ebv10:(exec_r==9)?ebv11:(exec_r==10)?ebv12:(exec_r==11)?ebv13:(exec_r==12)?ebv14:(exec_r==13)?ebv15:(exec_r==14)?ebv0:ebv1;
    assign enebc[2] =(exec_r==0)?ebc2 :(exec_r==1)?ebc3 :(exec_r==2)?ebc4 :(exec_r==3)?ebc5 :(exec_r==4)?ebc6 :(exec_r==5)?ebc7 :(exec_r==6)?ebc8 :(exec_r==7)?ebc9 :(exec_r==8)?ebc10:(exec_r==9)?ebc11:(exec_r==10)?ebc12:(exec_r==11)?ebc13:(exec_r==12)?ebc14:(exec_r==13)?ebc15:(exec_r==14)?ebc0:ebc1;
    assign enebv[3] =(exec_r==0)?ebv3 :(exec_r==1)?ebv4 :(exec_r==2)?ebv5 :(exec_r==3)?ebv6 :(exec_r==4)?ebv7 :(exec_r==5)?ebv8 :(exec_r==6)?ebv9 :(exec_r==7)?ebv10:(exec_r==8)?ebv11:(exec_r==9)?ebv12:(exec_r==10)?ebv13:(exec_r==11)?ebv14:(exec_r==12)?ebv15:(exec_r==13)?ebv0:(exec_r==14)?ebv1:ebv2;
    assign enebc[3] =(exec_r==0)?ebc3 :(exec_r==1)?ebc4 :(exec_r==2)?ebc5 :(exec_r==3)?ebc6 :(exec_r==4)?ebc7 :(exec_r==5)?ebc8 :(exec_r==6)?ebc9 :(exec_r==7)?ebc10:(exec_r==8)?ebc11:(exec_r==9)?ebc12:(exec_r==10)?ebc13:(exec_r==11)?ebc14:(exec_r==12)?ebc15:(exec_r==13)?ebc0:(exec_r==14)?ebc1:ebc2;
    assign enebv[4] =(exec_r==0)?ebv4 :(exec_r==1)?ebv5 :(exec_r==2)?ebv6 :(exec_r==3)?ebv7 :(exec_r==4)?ebv8 :(exec_r==5)?ebv9 :(exec_r==6)?ebv10:(exec_r==7)?ebv11:(exec_r==8)?ebv12:(exec_r==9)?ebv13:(exec_r==10)?ebv14:(exec_r==11)?ebv15:(exec_r==12)?ebv0:(exec_r==13)?ebv1:(exec_r==14)?ebv2:ebv3;
    assign enebc[4] =(exec_r==0)?ebc4 :(exec_r==1)?ebc5 :(exec_r==2)?ebc6 :(exec_r==3)?ebc7 :(exec_r==4)?ebc8 :(exec_r==5)?ebc9 :(exec_r==6)?ebc10:(exec_r==7)?ebc11:(exec_r==8)?ebc12:(exec_r==9)?ebc13:(exec_r==10)?ebc14:(exec_r==11)?ebc15:(exec_r==12)?ebc0:(exec_r==13)?ebc1:(exec_r==14)?ebc2:ebc3;
    assign enebv[5] =(exec_r==0)?ebv5 :(exec_r==1)?ebv6 :(exec_r==2)?ebv7 :(exec_r==3)?ebv8 :(exec_r==4)?ebv9 :(exec_r==5)?ebv10:(exec_r==6)?ebv11:(exec_r==7)?ebv12:(exec_r==8)?ebv13:(exec_r==9)?ebv14:(exec_r==10)?ebv15:(exec_r==11)?ebv0:(exec_r==12)?ebv1:(exec_r==13)?ebv2:(exec_r==14)?ebv3:ebv4;
    assign enebc[5] =(exec_r==0)?ebc5 :(exec_r==1)?ebc6 :(exec_r==2)?ebc7 :(exec_r==3)?ebc8 :(exec_r==4)?ebc9 :(exec_r==5)?ebc10:(exec_r==6)?ebc11:(exec_r==7)?ebc12:(exec_r==8)?ebc13:(exec_r==9)?ebc14:(exec_r==10)?ebc15:(exec_r==11)?ebc0:(exec_r==12)?ebc1:(exec_r==13)?ebc2:(exec_r==14)?ebc3:ebc4;
    assign enebv[6] =(exec_r==0)?ebv6 :(exec_r==1)?ebv7 :(exec_r==2)?ebv8 :(exec_r==3)?ebv9 :(exec_r==4)?ebv10:(exec_r==5)?ebv11:(exec_r==6)?ebv12:(exec_r==7)?ebv13:(exec_r==8)?ebv14:(exec_r==9)?ebv15:(exec_r==10)?ebv0:(exec_r==11)?ebv1:(exec_r==12)?ebv2:(exec_r==13)?ebv3:(exec_r==14)?ebv4:ebv5;
    assign enebc[6] =(exec_r==0)?ebc6 :(exec_r==1)?ebc7 :(exec_r==2)?ebc8 :(exec_r==3)?ebc9 :(exec_r==4)?ebc10:(exec_r==5)?ebc11:(exec_r==6)?ebc12:(exec_r==7)?ebc13:(exec_r==8)?ebc14:(exec_r==9)?ebc15:(exec_r==10)?ebc0:(exec_r==11)?ebc1:(exec_r==12)?ebc2:(exec_r==13)?ebc3:(exec_r==14)?ebc4:ebc5;
    assign enebv[7] =(exec_r==0)?ebv7 :(exec_r==1)?ebv8 :(exec_r==2)?ebv9 :(exec_r==3)?ebv10:(exec_r==4)?ebv11:(exec_r==5)?ebv12:(exec_r==6)?ebv13:(exec_r==7)?ebv14:(exec_r==8)?ebv15:(exec_r==9)?ebv0:(exec_r==10)?ebv1:(exec_r==11)?ebv2:(exec_r==12)?ebv3:(exec_r==13)?ebv4:(exec_r==14)?ebv5:ebv6;
    assign enebc[7] =(exec_r==0)?ebc7 :(exec_r==1)?ebc8 :(exec_r==2)?ebc9 :(exec_r==3)?ebc10:(exec_r==4)?ebc11:(exec_r==5)?ebc12:(exec_r==6)?ebc13:(exec_r==7)?ebc14:(exec_r==8)?ebc15:(exec_r==9)?ebc0:(exec_r==10)?ebc1:(exec_r==11)?ebc2:(exec_r==12)?ebc3:(exec_r==13)?ebc4:(exec_r==14)?ebc5:ebc6;
    assign enebv[8] =(exec_r==0)?ebv8 :(exec_r==1)?ebv9 :(exec_r==2)?ebv10:(exec_r==3)?ebv11:(exec_r==4)?ebv12:(exec_r==5)?ebv13:(exec_r==6)?ebv14:(exec_r==7)?ebv15:(exec_r==8)?ebv0:(exec_r==9)?ebv1:(exec_r==10)?ebv2:(exec_r==11)?ebv3:(exec_r==12)?ebv4:(exec_r==13)?ebv5:(exec_r==14)?ebv6:ebv7;
    assign enebc[8] =(exec_r==0)?ebc8 :(exec_r==1)?ebc9 :(exec_r==2)?ebc10:(exec_r==3)?ebc11:(exec_r==4)?ebc12:(exec_r==5)?ebc13:(exec_r==6)?ebc14:(exec_r==7)?ebc15:(exec_r==8)?ebc0:(exec_r==9)?ebc1:(exec_r==10)?ebc2:(exec_r==11)?ebc3:(exec_r==12)?ebc4:(exec_r==13)?ebc5:(exec_r==14)?ebc6:ebc7;
    assign enebv[9] =(exec_r==0)?ebv9 :(exec_r==1)?ebv10:(exec_r==2)?ebv11:(exec_r==3)?ebv12:(exec_r==4)?ebv13:(exec_r==5)?ebv14:(exec_r==6)?ebv15:(exec_r==7)?ebv0:(exec_r==8)?ebv1:(exec_r==9)?ebv2:(exec_r==10)?ebv3:(exec_r==11)?ebv4:(exec_r==12)?ebv5:(exec_r==13)?ebv6:(exec_r==14)?ebv7:ebv8;
    assign enebc[9] =(exec_r==0)?ebc9 :(exec_r==1)?ebc10:(exec_r==2)?ebc11:(exec_r==3)?ebc12:(exec_r==4)?ebc13:(exec_r==5)?ebc14:(exec_r==6)?ebc15:(exec_r==7)?ebc0:(exec_r==8)?ebc1:(exec_r==9)?ebc2:(exec_r==10)?ebc3:(exec_r==11)?ebc4:(exec_r==12)?ebc5:(exec_r==13)?ebc6:(exec_r==14)?ebc7:ebc8;
    assign enebv[10]=(exec_r==0)?ebv10:(exec_r==1)?ebv11:(exec_r==2)?ebv12:(exec_r==3)?ebv13:(exec_r==4)?ebv14:(exec_r==5)?ebv15:(exec_r==6)?ebv0:(exec_r==7)?ebv1:(exec_r==8)?ebv2:(exec_r==9)?ebv3:(exec_r==10)?ebv4:(exec_r==11)?ebv5:(exec_r==12)?ebv6:(exec_r==13)?ebv7:(exec_r==14)?ebv8:ebv9;
    assign enebc[10]=(exec_r==0)?ebc10:(exec_r==1)?ebc11:(exec_r==2)?ebc12:(exec_r==3)?ebc13:(exec_r==4)?ebc14:(exec_r==5)?ebc15:(exec_r==6)?ebc0:(exec_r==7)?ebc1:(exec_r==8)?ebc2:(exec_r==9)?ebc3:(exec_r==10)?ebc4:(exec_r==11)?ebc5:(exec_r==12)?ebc6:(exec_r==13)?ebc7:(exec_r==14)?ebc8:ebc9;
    assign enebv[11]=(exec_r==0)?ebv11:(exec_r==1)?ebv12:(exec_r==2)?ebv13:(exec_r==3)?ebv14:(exec_r==4)?ebv15:(exec_r==5)?ebv0:(exec_r==6)?ebv1:(exec_r==7)?ebv2:(exec_r==8)?ebv3:(exec_r==9)?ebv4:(exec_r==10)?ebv5:(exec_r==11)?ebv6:(exec_r==12)?ebv7:(exec_r==13)?ebv8:(exec_r==14)?ebv9:ebv10;
    assign enebc[11]=(exec_r==0)?ebc11:(exec_r==1)?ebc12:(exec_r==2)?ebc13:(exec_r==3)?ebc14:(exec_r==4)?ebc15:(exec_r==5)?ebc0:(exec_r==6)?ebc1:(exec_r==7)?ebc2:(exec_r==8)?ebc3:(exec_r==9)?ebc4:(exec_r==10)?ebc5:(exec_r==11)?ebc6:(exec_r==12)?ebc7:(exec_r==13)?ebc8:(exec_r==14)?ebc9:ebc10;
    assign enebv[12]=(exec_r==0)?ebv12:(exec_r==1)?ebv13:(exec_r==2)?ebv14:(exec_r==3)?ebv15:(exec_r==4)?ebv0:(exec_r==5)?ebv1:(exec_r==6)?ebv2:(exec_r==7)?ebv3:(exec_r==8)?ebv4:(exec_r==9)?ebv5:(exec_r==10)?ebv6:(exec_r==11)?ebv7:(exec_r==12)?ebv8:(exec_r==13)?ebv9:(exec_r==14)?ebv10:ebv11;
    assign enebc[12]=(exec_r==0)?ebc12:(exec_r==1)?ebc13:(exec_r==2)?ebc14:(exec_r==3)?ebc15:(exec_r==4)?ebc0:(exec_r==5)?ebc1:(exec_r==6)?ebc2:(exec_r==7)?ebc3:(exec_r==8)?ebc4:(exec_r==9)?ebc5:(exec_r==10)?ebc6:(exec_r==11)?ebc7:(exec_r==12)?ebc8:(exec_r==13)?ebc9:(exec_r==14)?ebc10:ebc11;
    assign enebv[13]=(exec_r==0)?ebv13:(exec_r==1)?ebv14:(exec_r==2)?ebv15:(exec_r==3)?ebv0:(exec_r==4)?ebv1:(exec_r==5)?ebv2:(exec_r==6)?ebv3:(exec_r==7)?ebv4:(exec_r==8)?ebv5:(exec_r==9)?ebv6:(exec_r==10)?ebv7:(exec_r==11)?ebv8:(exec_r==12)?ebv9:(exec_r==13)?ebv10:(exec_r==14)?ebv11:ebv12;
    assign enebc[13]=(exec_r==0)?ebc13:(exec_r==1)?ebc14:(exec_r==2)?ebc15:(exec_r==3)?ebc0:(exec_r==4)?ebc1:(exec_r==5)?ebc2:(exec_r==6)?ebc3:(exec_r==7)?ebc4:(exec_r==8)?ebc5:(exec_r==9)?ebc6:(exec_r==10)?ebc7:(exec_r==11)?ebc8:(exec_r==12)?ebc9:(exec_r==13)?ebc10:(exec_r==14)?ebc11:ebc12;
    assign enebv[14]=(exec_r==0)?ebv14:(exec_r==1)?ebv15:(exec_r==2)?ebv0:(exec_r==3)?ebv1:(exec_r==4)?ebv2:(exec_r==5)?ebv3:(exec_r==6)?ebv4:(exec_r==7)?ebv5:(exec_r==8)?ebv6:(exec_r==9)?ebv7:(exec_r==10)?ebv8:(exec_r==11)?ebv9:(exec_r==12)?ebv10:(exec_r==13)?ebv11:(exec_r==14)?ebv12:ebv13;
    assign enebc[14]=(exec_r==0)?ebc14:(exec_r==1)?ebc15:(exec_r==2)?ebc0:(exec_r==3)?ebc1:(exec_r==4)?ebc2:(exec_r==5)?ebc3:(exec_r==6)?ebc4:(exec_r==7)?ebc5:(exec_r==8)?ebc6:(exec_r==9)?ebc7:(exec_r==10)?ebc8:(exec_r==11)?ebc9:(exec_r==12)?ebc10:(exec_r==13)?ebc11:(exec_r==14)?ebc12:ebc13;
    assign enebv[15]=(exec_r==0)?ebv15:(exec_r==1)?ebv0:(exec_r==2)?ebv1:(exec_r==3)?ebv2:(exec_r==4)?ebv3:(exec_r==5)?ebv4:(exec_r==6)?ebv5:(exec_r==7)?ebv6:(exec_r==8)?ebv7:(exec_r==9)?ebv8:(exec_r==10)?ebv9:(exec_r==11)?ebv10:(exec_r==12)?ebv11:(exec_r==13)?ebv12:(exec_r==14)?ebv13:ebv14;
    assign enebc[15]=(exec_r==0)?ebc15:(exec_r==1)?ebc0:(exec_r==2)?ebc1:(exec_r==3)?ebc2:(exec_r==4)?ebc3:(exec_r==5)?ebc4:(exec_r==6)?ebc5:(exec_r==7)?ebc6:(exec_r==8)?ebc7:(exec_r==9)?ebc8:(exec_r==10)?ebc9:(exec_r==11)?ebc10:(exec_r==12)?ebc11:(exec_r==13)?ebc12:(exec_r==14)?ebc13:ebc14;

    wire [`TASK_WIDTH-1:0] exec_sg0 ={enebv[0], exec_a_val,enebc[0][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg1 ={enebv[1], exec_a_val,enebc[1][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg2 ={enebv[2], exec_a_val,enebc[2][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg3 ={enebv[3], exec_a_val,enebc[3][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg4 ={enebv[4], exec_a_val,enebc[4][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg5 ={enebv[5], exec_a_val,enebc[5][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg6 ={enebv[6], exec_a_val,enebc[6][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg7 ={enebv[7], exec_a_val,enebc[7][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg8 ={enebv[8], exec_a_val,enebc[8][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg9 ={enebv[9], exec_a_val,enebc[9][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg10={enebv[10],exec_a_val,enebc[10][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg11={enebv[11],exec_a_val,enebc[11][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg12={enebv[12],exec_a_val,enebc[12][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg13={enebv[13],exec_a_val,enebc[13][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg14={enebv[14],exec_a_val,enebc[14][8:0]};
    wire [`TASK_WIDTH-1:0] exec_sg15={enebv[15],exec_a_val,enebc[15][8:0]};

    //=========================================================================
    // MAC array input: executor (ptr_fifo path) or Gen2 (task_fifo path)
    //=========================================================================
    // Fire task_fifo when exec is idle (ptr drained) OR at each ptr-entry boundary.
    // exec_ptr_last already implies exec_prod_safe, so no separate check needed there.
    assign task_fifo_rd_en = (exec_idle || exec_ptr_last) && !task_fifo_empty && exec_prod_safe;

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
            mac_lane_valid_r <= 16'hFFFF;
            mac_lane_task_r[0 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg0;
            mac_lane_task_r[1 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg1;
            mac_lane_task_r[2 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg2;
            mac_lane_task_r[3 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg3;
            mac_lane_task_r[4 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg4;
            mac_lane_task_r[5 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg5;
            mac_lane_task_r[6 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg6;
            mac_lane_task_r[7 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg7;
            mac_lane_task_r[8 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg8;
            mac_lane_task_r[9 *`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg9;
            mac_lane_task_r[10*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg10;
            mac_lane_task_r[11*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg11;
            mac_lane_task_r[12*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg12;
            mac_lane_task_r[13*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg13;
            mac_lane_task_r[14*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg14;
            mac_lane_task_r[15*`TASK_WIDTH+:`TASK_WIDTH]<=exec_sg15;
        end else if (task_fifo_rd_en_d1) begin
            mac_lane_valid_r <= task_fifo_rd_data_d1[`N_MAC-1:0];
            mac_lane_task_r[0 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+0 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[1 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+1 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[2 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+2 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[3 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+3 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[4 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+4 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[5 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+5 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[6 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+6 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[7 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+7 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[8 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+8 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[9 *`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+9 *`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[10*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+10*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[11*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+11*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[12*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+12*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[13*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+13*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[14*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+14*`TASK_WIDTH+:`TASK_WIDTH];
            mac_lane_task_r[15*`TASK_WIDTH+:`TASK_WIDTH]<=task_fifo_rd_data_d1[`N_MAC+15*`TASK_WIDTH+:`TASK_WIDTH];
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
    assign product_group_wr_data[`N_MAC+0 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[0 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+1 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[1 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+2 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[2 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+3 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[3 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+4 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[4 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+5 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[5 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+6 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[6 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+7 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[7 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+8 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[8 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+9 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[9 *`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+10*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[10*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+11*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[11*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+12*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[12*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+13*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[13*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+14*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[14*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+15*`PRODUCT_WIDTH+:`PRODUCT_WIDTH]=mul_product[15*`PRODUCT_WIDTH+:`PRODUCT_WIDTH];

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
    // Row accumulators (ping-pong, 16-bank)
    //=========================================================================
    wire mac_pipeline_idle = !(|mac_lane_valid);

    wire acc_busy_0,acc_busy_1,acc_row_done_0,acc_row_done_1;
    wire acc_issue_ready_0,acc_issue_ready_1;
    wire [15:0] drain_valid_0,drain_valid_1;
    wire [4:0]  drain_gaddr_0,drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] drain_row_id_0,drain_row_id_1;
    wire [16*16-1:0] drain_values_0,drain_values_1;
    wire drain_active_0,drain_active_1;

    wire other_acc_busy = comp_sel ? acc_busy_0 : acc_busy_1;

    assign a_desc_ready = (state == PE_LOAD_ROW_DESC);

    wire pe_drain_done = (state==PE_WAIT_PRODUCT_DRAIN) && mac_pipeline_idle && !other_acc_busy;

    reg mac_done_latch_0,mac_done_latch_1;
    always @(posedge aclk) begin
        if (!aresetn) begin mac_done_latch_0<=0; mac_done_latch_1<=0; end
        else begin
            if (pe_drain_done&&!comp_sel) mac_done_latch_0<=1;
            if (pe_drain_done&& comp_sel) mac_done_latch_1<=1;
            if (mac_done_latch_0&&prod_fifo_empty_0&&!prd_hold_0&&!prd_rd_d1_0) mac_done_latch_0<=0;
            if (mac_done_latch_1&&prod_fifo_empty_1&&!prd_hold_1&&!prd_rd_d1_1) mac_done_latch_1<=0;
        end
    end

    // Hold registers: save a product when accumulator stalls (issue_ready drops
    // after the FIFO read pointer already advanced). Cleared when re-issued.
    reg prd_hold_0, prd_hold_1;
    reg [`PRODUCT_GROUP_WIDTH-1:0] prd_hold_dat_0, prd_hold_dat_1;

    // Block FIFO reads only when a product is held waiting for re-issue.
    // The acc_issue_ready_0 gate already prevents reads when issue_ready=0,
    // so no extra !prd_rd_d1 blocking is needed (that would halve throughput).
    assign prod_fifo_rd_en_0 = !prod_fifo_empty_0 && acc_issue_ready_0 && !prd_hold_0;
    assign prod_fifo_rd_en_1 = !prod_fifo_empty_1 && acc_issue_ready_1 && !prd_hold_1;

    reg prd_rd_d1_0, prd_rd_d1_1;
    reg [`PRODUCT_GROUP_WIDTH-1:0] prd_dat_d1_0, prd_dat_d1_1;
    always @(posedge aclk) begin
        if (!aresetn) begin
            prd_rd_d1_0 <= 0; prd_dat_d1_0 <= 0;
            prd_rd_d1_1 <= 0; prd_dat_d1_1 <= 0;
            prd_hold_0  <= 0; prd_hold_dat_0 <= 0;
            prd_hold_1  <= 0; prd_hold_dat_1 <= 0;
        end else begin
            // Save product when accumulator not ready (issue_ready dropped in
            // the 1-cycle window between FIFO read and product application).
            if (prd_rd_d1_0 && !acc_issue_ready_0) begin
                prd_hold_0     <= 1'b1;
                prd_hold_dat_0 <= prd_dat_d1_0;
            end else if (prd_hold_0 && acc_issue_ready_0)
                prd_hold_0 <= 1'b0;

            if (prd_rd_d1_1 && !acc_issue_ready_1) begin
                prd_hold_1     <= 1'b1;
                prd_hold_dat_1 <= prd_dat_d1_1;
            end else if (prd_hold_1 && acc_issue_ready_1)
                prd_hold_1 <= 1'b0;

            prd_rd_d1_0  <= prod_fifo_rd_en_0 && !prod_fifo_empty_0;
            prd_dat_d1_0 <= prod_fifo_rd_data_0;
            prd_rd_d1_1  <= prod_fifo_rd_en_1 && !prod_fifo_empty_1;
            prd_dat_d1_1 <= prod_fifo_rd_data_1;
        end
    end

    // Effective product: held data takes priority over the just-latched data.
    // At most one of prd_hold and prd_rd_d1 is true at any cycle (guaranteed
    // by the !prd_hold && !prd_rd_d1 gate on prod_fifo_rd_en).
    wire eff_valid_0 = prd_hold_0 | prd_rd_d1_0;
    wire eff_valid_1 = prd_hold_1 | prd_rd_d1_1;
    wire [`PRODUCT_GROUP_WIDTH-1:0] eff_dat_0 = prd_hold_0 ? prd_hold_dat_0 : prd_dat_d1_0;
    wire [`PRODUCT_GROUP_WIDTH-1:0] eff_dat_1 = prd_hold_1 ? prd_hold_dat_1 : prd_dat_d1_1;

    // acc_inp_done must also wait for any in-flight or held product to drain.
    wire acc_inp_done_0 = mac_done_latch_0 && prod_fifo_empty_0
                          && !prd_hold_0 && !prd_rd_d1_0;
    wire acc_inp_done_1 = mac_done_latch_1 && prod_fifo_empty_1
                          && !prd_hold_1 && !prd_rd_d1_1;

    // Extract 16 lane_valid, 16 col_ids, 16 products from effective data
    wire [15:0]    alv0 = eff_dat_0[`N_MAC-1:0];
    wire [16*9-1:0] alc0 = {
        eff_dat_0[`N_MAC+15*`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+14*`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+13*`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+12*`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+11*`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+10*`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+9 *`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+8 *`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+7 *`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+6 *`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+5 *`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+4 *`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+3 *`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+2 *`PRODUCT_WIDTH+16+:9],
        eff_dat_0[`N_MAC+1 *`PRODUCT_WIDTH+16+:9],eff_dat_0[`N_MAC+0 *`PRODUCT_WIDTH+16+:9]};
    wire [16*16-1:0] alp0 = {
        eff_dat_0[`N_MAC+15*`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+14*`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+13*`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+12*`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+11*`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+10*`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+9 *`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+8 *`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+7 *`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+6 *`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+5 *`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+4 *`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+3 *`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+2 *`PRODUCT_WIDTH+:16],
        eff_dat_0[`N_MAC+1 *`PRODUCT_WIDTH+:16],eff_dat_0[`N_MAC+0 *`PRODUCT_WIDTH+:16]};
    wire [15:0]    alv1 = eff_dat_1[`N_MAC-1:0];
    wire [16*9-1:0] alc1 = {
        eff_dat_1[`N_MAC+15*`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+14*`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+13*`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+12*`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+11*`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+10*`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+9 *`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+8 *`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+7 *`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+6 *`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+5 *`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+4 *`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+3 *`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+2 *`PRODUCT_WIDTH+16+:9],
        eff_dat_1[`N_MAC+1 *`PRODUCT_WIDTH+16+:9],eff_dat_1[`N_MAC+0 *`PRODUCT_WIDTH+16+:9]};
    wire [16*16-1:0] alp1 = {
        eff_dat_1[`N_MAC+15*`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+14*`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+13*`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+12*`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+11*`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+10*`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+9 *`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+8 *`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+7 *`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+6 *`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+5 *`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+4 *`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+3 *`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+2 *`PRODUCT_WIDTH+:16],
        eff_dat_1[`N_MAC+1 *`PRODUCT_WIDTH+:16],eff_dat_1[`N_MAC+0 *`PRODUCT_WIDTH+:16]};

    row_accumulator_16bank #(
        .OUT_COLS(512),.COL_W(9),.PROD_W(16),.ACC_W(16),.EPOCH_W(16),
        .BANK_FIFO_DEPTH(32),.BANK_FIFO_LOG(5),.ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_0 (
        .clk(aclk),.rst_n(aresetn),
        .row_start((state==PE_CLEAR_ACC)&&!comp_sel),.row_id_in(row_idx),.drain_cols(N),
        .row_input_done(acc_inp_done_0),.busy(acc_busy_0),.row_done(acc_row_done_0),
        .issue_valid(eff_valid_0),.issue_ready(acc_issue_ready_0),
        .lane_valid(alv0),.lane_col_id(alc0),.lane_product(alp0),
        .drain_valid(drain_valid_0),.drain_gaddr(drain_gaddr_0),
        .drain_row_id(drain_row_id_0),.drain_values(drain_values_0),
        .drain_active(drain_active_0)
    );

    row_accumulator_16bank #(
        .OUT_COLS(512),.COL_W(9),.PROD_W(16),.ACC_W(16),.EPOCH_W(16),
        .BANK_FIFO_DEPTH(32),.BANK_FIFO_LOG(5),.ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_1 (
        .clk(aclk),.rst_n(aresetn),
        .row_start((state==PE_CLEAR_ACC)&&comp_sel),.row_id_in(row_idx),.drain_cols(N),
        .row_input_done(acc_inp_done_1),.busy(acc_busy_1),.row_done(acc_row_done_1),
        .issue_valid(eff_valid_1),.issue_ready(acc_issue_ready_1),
        .lane_valid(alv1),.lane_col_id(alc1),.lane_product(alp1),
        .drain_valid(drain_valid_1),.drain_gaddr(drain_gaddr_1),
        .drain_row_id(drain_row_id_1),.drain_values(drain_values_1),
        .drain_active(drain_active_1)
    );

    //=========================================================================
    // C bank — independent on-chip C storage (separate from A/B buffers).
    //
    //   Indexed by LOCAL row (the accumulator's drain_row_id is now row_idx,
    //   a dense 0..rows_per_PE-1 counter), so the bank depth is set by the
    //   number of rows THIS PE computes, not the global row range.  C_row_map
    //   records the global C row for each local slot so the host can translate
    //   on readback.
    //
    //   16 sub-banks (parallel with the 16 accumulator banks).  On every drain
    //   beat the full column group is written: bank b gets its accumulated
    //   value, or 0 when drain_valid[b]=0.  Because S_DRAIN visits every group
    //   0..ceil(N/16)-1 (incl. all-zero groups), each computed C row is fully
    //   written with no separate clear pass.
    //
    //   Address = {local_row[C_ROW_ADDR_BITS-1:0], gaddr[4:0]}.
    //   The two ping-pong accumulators drain serially (guarded by
    //   other_acc_busy), so a priority mux on drain_active is race-free.
    //=========================================================================
    localparam C_BANK_ADDR_W = `C_ROW_ADDR_BITS + 5;     // local_row + gaddr
    localparam C_BANK_DEPTH  = 1 << C_BANK_ADDR_W;

    reg [15:0]               C_bank [0:15][0:C_BANK_DEPTH-1];
    reg [`MAX_DIM_BITS-1:0]  C_row_map [0:`C_ROW_SLOTS-1];   // local → global C row

    // Record the global row for each local slot as descriptors are loaded.
    // Descriptor c_row field is a_desc_data[8:0] (nnz begins at bit 9).
    always @(posedge aclk) begin
        if ((state==PE_LOAD_ROW_DESC) && a_desc_valid)
            C_row_map[row_idx[`C_ROW_ADDR_BITS-1:0]] <= {{(`MAX_DIM_BITS-9){1'b0}}, a_desc_data[8:0]};
    end

    wire                        c_wr_en   = drain_active_0 | drain_active_1;
    wire                        c_wr_sel0 = drain_active_0;
    wire [`C_ROW_ADDR_BITS-1:0] c_wr_row  = c_wr_sel0 ? drain_row_id_0[`C_ROW_ADDR_BITS-1:0]
                                                      : drain_row_id_1[`C_ROW_ADDR_BITS-1:0];
    wire [4:0]                  c_wr_gaddr = c_wr_sel0 ? drain_gaddr_0  : drain_gaddr_1;
    wire [15:0]                 c_wr_dv    = c_wr_sel0 ? drain_valid_0  : drain_valid_1;
    wire [16*16-1:0]            c_wr_dat   = c_wr_sel0 ? drain_values_0 : drain_values_1;
    wire [C_BANK_ADDR_W-1:0]    c_wr_addr  = {c_wr_row, c_wr_gaddr};

    // Registered map read (same address timing as the C bank data read).
    always @(posedge aclk) begin
        if (c_rd_en)
            c_rd_row <= C_row_map[c_rd_addr[C_BANK_ADDR_W-1:5]];
    end

    genvar cb;
    generate
        for (cb = 0; cb < 16; cb = cb + 1) begin : gen_c_bank
            always @(posedge aclk) begin
                if (c_wr_en)
                    C_bank[cb][c_wr_addr] <= c_wr_dv[cb] ? c_wr_dat[cb*16 +: 16]
                                                         : 16'h0000;
            end
            always @(posedge aclk) begin
                if (c_rd_en)
                    c_rd_data[cb*16 +: 16] <= C_bank[cb][c_rd_addr];
            end
        end
    endgenerate

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
            PE_STREAM_INSTRS: begin
                if (gen_state==GEN_ROW_DONE && task_fifo_empty && !g2_want_flush && ptr_fifo_empty && exec_idle)
                    state_next=PE_WAIT_PRODUCT_DRAIN;
                else if (gen_state==GEN_ROW_DONE)
                    state_next=PE_WAIT_TASK_DRAIN;
            end
            PE_WAIT_TASK_DRAIN:    if (task_fifo_empty&&!g2_want_flush&&ptr_fifo_empty&&exec_idle) state_next=PE_WAIT_PRODUCT_DRAIN;
            PE_WAIT_PRODUCT_DRAIN: if (mac_pipeline_idle&&!other_acc_busy) state_next=PE_NEXT_ROW;
            PE_NEXT_ROW:           state_next=((row_idx+1)>=row_count)?PE_DONE:PE_LOAD_ROW_DESC;
            PE_DONE:               state_next=PE_DONE;
        endcase
    end

`ifdef SIMULATION
    // Debug: detect dropped products (issue_valid=1 but issue_ready=0)
    // and trace slot-0 product enqueues for acc_0
    integer _dbg_j;
    always @(posedge aclk) begin
        // Alert if product arrives but accumulator not ready
        if (eff_valid_0 && !acc_issue_ready_0) begin
            $display("[DBG DROP] t=%0t row=%0d acc0 product DROPPED (issue_ready=0)",
                     $time, row_idx);
            for (_dbg_j = 0; _dbg_j < 16; _dbg_j = _dbg_j + 1) begin
                if (alv0[_dbg_j])
                    $display("[DBG DROP]   lane%0d col_id=%0d val=0x%04x",
                             _dbg_j, alc0[_dbg_j*9+:9], alp0[_dbg_j*16+:16]);
            end
        end
        if (eff_valid_1 && !acc_issue_ready_1) begin
            $display("[DBG DROP] t=%0t row=%0d acc1 product DROPPED (issue_ready=0)",
                     $time, row_idx);
            for (_dbg_j = 0; _dbg_j < 16; _dbg_j = _dbg_j + 1) begin
                if (alv1[_dbg_j])
                    $display("[DBG DROP]   lane%0d col_id=%0d val=0x%04x",
                             _dbg_j, alc1[_dbg_j*9+:9], alp1[_dbg_j*16+:16]);
            end
        end
        // Trace every slot-0 accumulation for acc_0 (col_id 0..15)
        if (eff_valid_0 && acc_issue_ready_0) begin
            for (_dbg_j = 0; _dbg_j < 16; _dbg_j = _dbg_j + 1) begin
                if (alv0[_dbg_j] && alc0[_dbg_j*9 +: 9] < 16)
                    $display("[DBG S0] t=%0t row=%0d acc0 enq lane%0d col_id=%0d",
                             $time, row_idx, _dbg_j, alc0[_dbg_j*9 +: 9]);
            end
        end
    end
`endif

endmodule
