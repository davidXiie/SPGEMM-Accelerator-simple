//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : PE Top — hardware on-chip instruction generation with
//            cross-B-row task packing.
//
//   Eliminates pre-computed instr_buf.  Instead, the generator FSM walks
//   A_col_buf → B_desc_buf to produce 4-wide task groups directly.
//
//   Cross-B-row packing: a carry buffer (up to 3 elements) holds tail
//   elements from a partial B-row group.  When the next A nonzero is
//   prefetched its B-row head fills the remaining slots, forming a
//   complete 4-wide group.  Each lane carries its own a_val so different
//   A nonzeros can coexist in one task group (never crossing A rows).
//
//   A row descriptor (36-bit):
//     [35:33] reserved (3'b0)
//     [32:19] a_off   — start index into A_col/A_val buffers (local to PE, 14-bit, max=16383)
//     [18: 9] a_nnz   — number of A nonzeros in this row (10-bit, max=512)
//     [ 8: 0] c_row   — global C output row id (9-bit, max=511)
//
//   B descriptor (32-bit, broadcast):
//     [31:27] reserved (5'b0)
//     [26:10] b_off   — start of B row in 8-bank storage (17-bit, max=131071)
//     [ 9: 0] b_nnz   — number of B nonzeros in this row (10-bit, max=512)
//
//   Generator sub-FSM (5 states):
//     GEN_IDLE → GEN_FETCH → GEN_EMIT ↔ GEN_ROW_DONE
//                                     → GEN_FLUSH (drain carry at row end)
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

    // A row descriptor stream  {3'b0, a_off[13:0], a_nnz[9:0], c_row[8:0]}  (36-bit)
    // Host drives a_desc_valid + a_desc_data; PE asserts a_desc_ready when waiting.
    input  wire                          a_desc_valid,
    output wire                          a_desc_ready,
    input  wire [35:0]                   a_desc_data,

    // A value buffer (FP16)
    input  wire                          a_val_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        a_val_wdata,

    // A column index buffer (which B row each A nonzero points to)
    input  wire                          a_col_we,
    input  wire [`A_NNZ_ADDR_BITS-1:0]  a_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        a_col_wdata,

    // B col/val buffers (4-banked by absolute index[1:0])
    input  wire                          b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_col_wdata,
    input  wire                          b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0]  b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]        b_val_wdata,

    // B row descriptor (broadcast; {5'b0, b_off[16:0], b_nnz[9:0]}  32-bit)
    input  wire                          b_desc_we,
    input  wire [`B_ROW_ADDR_BITS-1:0]  b_desc_waddr,
    input  wire [31:0]                   b_desc_wdata


    // C buffer read port (synchronous, 1-cycle latency)
    // input  wire                          c_rd_en,
    // input  wire [16:0]                   c_rd_addr,
    // output reg  [15:0]                   c_rd_data
);

    localparam ACC_COL_W     = 9;
    localparam BANK_ADDR_W   = ACC_COL_W - 3;
    // localparam C_BANK_ADDR_W = `A_ROW_ADDR_BITS + BANK_ADDR_W;
    // localparam C_BANK_DEPTH  = 32'd1 << C_BANK_ADDR_W;
    // localparam C_RD_ADDR_W   = `A_ROW_ADDR_BITS + ACC_COL_W;

    localparam B_BANK_DEPTH  = `B_NNZ_SLOT / 8;
    localparam B_DESC_DEPTH  = `B_ROW_SLOT;   // 512

    //=========================================================================
    // SRAM declarations
    //=========================================================================
    reg [`DATA_WIDTH-1:0] A_val_buf      [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf      [0:`A_NNZ_SLOT_PER_PE-1];

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

    reg [31:0]            B_desc_buf [0:B_DESC_DEPTH-1];

    // reg [15:0] c_bank0 [0:C_BANK_DEPTH-1];
    // reg [15:0] c_bank1 [0:C_BANK_DEPTH-1];
    // reg [15:0] c_bank2 [0:C_BANK_DEPTH-1];
    // reg [15:0] c_bank3 [0:C_BANK_DEPTH-1];

//`ifdef COCOTB_SIM
//    integer _ci;
//    initial begin
//        c_rd_data = 16'h0;
//        for (_ci = 0; _ci < C_BANK_DEPTH; _ci = _ci + 1) begin
//            c_bank0[_ci] = 16'h0; c_bank1[_ci] = 16'h0;
//            c_bank2[_ci] = 16'h0; c_bank3[_ci] = 16'h0;
//        end
//    end
//`endif

    //=========================================================================
    // SRAM write ports
    //=========================================================================
    always @(posedge aclk) begin
        if (a_val_we)  A_val_buf[a_val_waddr]        <= a_val_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]         <= a_col_wdata;
        if (b_desc_we) B_desc_buf[b_desc_waddr]       <= b_desc_wdata;
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
    reg [31:0]                 cur_a_off;
    reg [15:0]                 cur_a_nnz;

    //=========================================================================
    // Generator sub-FSM
    //=========================================================================
    localparam GEN_IDLE     = 3'd0;
    localparam GEN_FETCH    = 3'd1;
    localparam GEN_EMIT     = 3'd2;
    localparam GEN_ROW_DONE = 3'd3;
    localparam GEN_FLUSH    = 3'd4;  // flush remaining carry at end of A row

    reg [2:0]  gen_state;
    reg [15:0] gen_t;
    reg [15:0] gen_g;
    reg [15:0] gen_a_val;
    reg [31:0] gen_b_off;
    reg [15:0] gen_b_nnz;

    // Carry buffer: up to 7 elements from the tail of the previous B row
    reg [2:0]  carry_cnt;          // 0..7
    reg [15:0] carry_av [0:6];     // a_val per carry slot
    reg [15:0] carry_bv [0:6];     // b_val per carry slot
    reg [15:0] carry_bc [0:6];     // col_id per carry slot

    // 2-level combinatorial read chain
    wire [`A_NNZ_ADDR_BITS-1:0] fetch_a_addr =
        cur_a_off[`A_NNZ_ADDR_BITS-1:0] + gen_t[`A_NNZ_ADDR_BITS-1:0];
    wire [15:0] fetch_k_idx   = A_col_buf[fetch_a_addr];
    wire [15:0] fetch_a_val   = A_val_buf[fetch_a_addr];
    wire [31:0] fetch_b_desc  = B_desc_buf[fetch_k_idx[`B_ROW_ADDR_BITS-1:0]];
    wire [31:0] fetch_b_off   = {15'b0, fetch_b_desc[26:10]};
    wire [15:0] fetch_b_nnz   = {6'b0,  fetch_b_desc[9:0]};

    // Current group decode
    wire [31:0] gen_abs_base  = gen_b_off + {gen_g[13:0], 3'b000};
    wire [2:0]  gen_r         = gen_abs_base[2:0];
    wire [13:0] gen_m         = gen_abs_base[16:3];

    wire [13:0] gen_bg0 = (gen_r == 3'd0) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg1 = (gen_r <= 3'd1) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg2 = (gen_r <= 3'd2) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg3 = (gen_r <= 3'd3) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg4 = (gen_r <= 3'd4) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg5 = (gen_r <= 3'd5) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg6 = (gen_r <= 3'd6) ? gen_m : gen_m + 14'd1;
    wire [13:0] gen_bg7 = gen_m;

    wire [15:0] gen_remaining = gen_b_nnz - {gen_g[13:0], 3'b000};
    wire [3:0]  gen_cnt       = (gen_remaining >= 16'd8) ? 4'd8 : gen_remaining[3:0];
    wire        gen_last_grp  = (gen_remaining <= 16'd8);

    // B bank reads (8 lanes, combinatorial)
    wire [`DATA_WIDTH-1:0] bc0 = B_col_b0[gen_bg0];
    wire [`DATA_WIDTH-1:0] bv0 = B_val_b0[gen_bg0];
    wire [`DATA_WIDTH-1:0] bc1 = B_col_b1[gen_bg1];
    wire [`DATA_WIDTH-1:0] bv1 = B_val_b1[gen_bg1];
    wire [`DATA_WIDTH-1:0] bc2 = B_col_b2[gen_bg2];
    wire [`DATA_WIDTH-1:0] bv2 = B_val_b2[gen_bg2];
    wire [`DATA_WIDTH-1:0] bc3 = B_col_b3[gen_bg3];
    wire [`DATA_WIDTH-1:0] bv3 = B_val_b3[gen_bg3];
    wire [`DATA_WIDTH-1:0] bc4 = B_col_b4[gen_bg4];
    wire [`DATA_WIDTH-1:0] bv4 = B_val_b4[gen_bg4];
    wire [`DATA_WIDTH-1:0] bc5 = B_col_b5[gen_bg5];
    wire [`DATA_WIDTH-1:0] bv5 = B_val_b5[gen_bg5];
    wire [`DATA_WIDTH-1:0] bc6 = B_col_b6[gen_bg6];
    wire [`DATA_WIDTH-1:0] bv6 = B_val_b6[gen_bg6];
    wire [`DATA_WIDTH-1:0] bc7 = B_col_b7[gen_bg7];
    wire [`DATA_WIDTH-1:0] bv7 = B_val_b7[gen_bg7];

    //=========================================================================
    // New elements in sequential order (rotation-corrected)
    //
    // ne[j] = j-th valid element in this group (lane (gen_r+j)%8).
    // Valid lanes are gen_r, (gen_r+1)%8, ..., (gen_r+gen_cnt-1)%8.
    // Extracting them in this fixed order allows uniform carry buffer filling.
    //=========================================================================
    wire [15:0] ne_bv [0:7];
    wire [15:0] ne_bc [0:7];

    // ne[k] = bv_{(gen_r+k)%8}
    assign ne_bv[0] = (gen_r==3'd0)?bv0:(gen_r==3'd1)?bv1:(gen_r==3'd2)?bv2:(gen_r==3'd3)?bv3:
                      (gen_r==3'd4)?bv4:(gen_r==3'd5)?bv5:(gen_r==3'd6)?bv6:bv7;
    assign ne_bc[0] = (gen_r==3'd0)?bc0:(gen_r==3'd1)?bc1:(gen_r==3'd2)?bc2:(gen_r==3'd3)?bc3:
                      (gen_r==3'd4)?bc4:(gen_r==3'd5)?bc5:(gen_r==3'd6)?bc6:bc7;
    assign ne_bv[1] = (gen_r==3'd0)?bv1:(gen_r==3'd1)?bv2:(gen_r==3'd2)?bv3:(gen_r==3'd3)?bv4:
                      (gen_r==3'd4)?bv5:(gen_r==3'd5)?bv6:(gen_r==3'd6)?bv7:bv0;
    assign ne_bc[1] = (gen_r==3'd0)?bc1:(gen_r==3'd1)?bc2:(gen_r==3'd2)?bc3:(gen_r==3'd3)?bc4:
                      (gen_r==3'd4)?bc5:(gen_r==3'd5)?bc6:(gen_r==3'd6)?bc7:bc0;
    assign ne_bv[2] = (gen_r==3'd0)?bv2:(gen_r==3'd1)?bv3:(gen_r==3'd2)?bv4:(gen_r==3'd3)?bv5:
                      (gen_r==3'd4)?bv6:(gen_r==3'd5)?bv7:(gen_r==3'd6)?bv0:bv1;
    assign ne_bc[2] = (gen_r==3'd0)?bc2:(gen_r==3'd1)?bc3:(gen_r==3'd2)?bc4:(gen_r==3'd3)?bc5:
                      (gen_r==3'd4)?bc6:(gen_r==3'd5)?bc7:(gen_r==3'd6)?bc0:bc1;
    assign ne_bv[3] = (gen_r==3'd0)?bv3:(gen_r==3'd1)?bv4:(gen_r==3'd2)?bv5:(gen_r==3'd3)?bv6:
                      (gen_r==3'd4)?bv7:(gen_r==3'd5)?bv0:(gen_r==3'd6)?bv1:bv2;
    assign ne_bc[3] = (gen_r==3'd0)?bc3:(gen_r==3'd1)?bc4:(gen_r==3'd2)?bc5:(gen_r==3'd3)?bc6:
                      (gen_r==3'd4)?bc7:(gen_r==3'd5)?bc0:(gen_r==3'd6)?bc1:bc2;
    assign ne_bv[4] = (gen_r==3'd0)?bv4:(gen_r==3'd1)?bv5:(gen_r==3'd2)?bv6:(gen_r==3'd3)?bv7:
                      (gen_r==3'd4)?bv0:(gen_r==3'd5)?bv1:(gen_r==3'd6)?bv2:bv3;
    assign ne_bc[4] = (gen_r==3'd0)?bc4:(gen_r==3'd1)?bc5:(gen_r==3'd2)?bc6:(gen_r==3'd3)?bc7:
                      (gen_r==3'd4)?bc0:(gen_r==3'd5)?bc1:(gen_r==3'd6)?bc2:bc3;
    assign ne_bv[5] = (gen_r==3'd0)?bv5:(gen_r==3'd1)?bv6:(gen_r==3'd2)?bv7:(gen_r==3'd3)?bv0:
                      (gen_r==3'd4)?bv1:(gen_r==3'd5)?bv2:(gen_r==3'd6)?bv3:bv4;
    assign ne_bc[5] = (gen_r==3'd0)?bc5:(gen_r==3'd1)?bc6:(gen_r==3'd2)?bc7:(gen_r==3'd3)?bc0:
                      (gen_r==3'd4)?bc1:(gen_r==3'd5)?bc2:(gen_r==3'd6)?bc3:bc4;
    assign ne_bv[6] = (gen_r==3'd0)?bv6:(gen_r==3'd1)?bv7:(gen_r==3'd2)?bv0:(gen_r==3'd3)?bv1:
                      (gen_r==3'd4)?bv2:(gen_r==3'd5)?bv3:(gen_r==3'd6)?bv4:bv5;
    assign ne_bc[6] = (gen_r==3'd0)?bc6:(gen_r==3'd1)?bc7:(gen_r==3'd2)?bc0:(gen_r==3'd3)?bc1:
                      (gen_r==3'd4)?bc2:(gen_r==3'd5)?bc3:(gen_r==3'd6)?bc4:bc5;
    assign ne_bv[7] = (gen_r==3'd0)?bv7:(gen_r==3'd1)?bv0:(gen_r==3'd2)?bv1:(gen_r==3'd3)?bv2:
                      (gen_r==3'd4)?bv3:(gen_r==3'd5)?bv4:(gen_r==3'd6)?bv5:bv6;
    assign ne_bc[7] = (gen_r==3'd0)?bc7:(gen_r==3'd1)?bc0:(gen_r==3'd2)?bc1:(gen_r==3'd3)?bc2:
                      (gen_r==3'd4)?bc3:(gen_r==3'd5)?bc4:(gen_r==3'd6)?bc5:bc6;

    //=========================================================================
    // Pack logic
    //
    // pack_total  = carry_cnt + gen_cnt  (range 1..15, fits in 4 bits)
    // pack_can_emit = pack_total >= 8    (bit [3] of the 4-bit sum)
    // overflow_cnt  = pack_total - 8     (pack_total[2:0] when can_emit)
    //
    // Output slot k (when emit):
    //   k < carry_cnt → carry buffer slot k
    //   k >= carry_cnt → ne[k - carry_cnt] with gen_a_val
    //=========================================================================
    wire [3:0] pack_total    = {1'b0, carry_cnt} + gen_cnt;
    wire       pack_can_emit = pack_total[3];          // >= 8
    wire [2:0] overflow_cnt  = pack_total[2:0];        // = pack_total-8 when can_emit

    // Output a_val per slot (slot 7 always new since carry_cnt <= 7)
    wire [15:0] out_av0 = (carry_cnt >= 3'd1) ? carry_av[0] : gen_a_val;
    wire [15:0] out_av1 = (carry_cnt >= 3'd2) ? carry_av[1] : gen_a_val;
    wire [15:0] out_av2 = (carry_cnt >= 3'd3) ? carry_av[2] : gen_a_val;
    wire [15:0] out_av3 = (carry_cnt >= 3'd4) ? carry_av[3] : gen_a_val;
    wire [15:0] out_av4 = (carry_cnt >= 3'd5) ? carry_av[4] : gen_a_val;
    wire [15:0] out_av5 = (carry_cnt >= 3'd6) ? carry_av[5] : gen_a_val;
    wire [15:0] out_av6 = (carry_cnt >= 3'd7) ? carry_av[6] : gen_a_val;
    wire [15:0] out_av7 = gen_a_val;

    // Output b_val per slot (slot k: carry if k < carry_cnt, else ne[k-carry_cnt])
    wire [15:0] out_bv0 = (carry_cnt >= 3'd1) ? carry_bv[0] : ne_bv[0];
    wire [15:0] out_bv1 = (carry_cnt >= 3'd2) ? carry_bv[1] :
                          (carry_cnt == 3'd1)  ? ne_bv[0]    : ne_bv[1];
    wire [15:0] out_bv2 = (carry_cnt >= 3'd3) ? carry_bv[2] :
                          (carry_cnt == 3'd2)  ? ne_bv[0]    :
                          (carry_cnt == 3'd1)  ? ne_bv[1]    : ne_bv[2];
    wire [15:0] out_bv3 = (carry_cnt >= 3'd4) ? carry_bv[3] :
                          (carry_cnt == 3'd3)  ? ne_bv[0]    :
                          (carry_cnt == 3'd2)  ? ne_bv[1]    :
                          (carry_cnt == 3'd1)  ? ne_bv[2]    : ne_bv[3];
    wire [15:0] out_bv4 = (carry_cnt >= 3'd5) ? carry_bv[4] :
                          (carry_cnt == 3'd4)  ? ne_bv[0]    :
                          (carry_cnt == 3'd3)  ? ne_bv[1]    :
                          (carry_cnt == 3'd2)  ? ne_bv[2]    :
                          (carry_cnt == 3'd1)  ? ne_bv[3]    : ne_bv[4];
    wire [15:0] out_bv5 = (carry_cnt >= 3'd6) ? carry_bv[5] :
                          (carry_cnt == 3'd5)  ? ne_bv[0]    :
                          (carry_cnt == 3'd4)  ? ne_bv[1]    :
                          (carry_cnt == 3'd3)  ? ne_bv[2]    :
                          (carry_cnt == 3'd2)  ? ne_bv[3]    :
                          (carry_cnt == 3'd1)  ? ne_bv[4]    : ne_bv[5];
    wire [15:0] out_bv6 = (carry_cnt >= 3'd7) ? carry_bv[6] :
                          (carry_cnt == 3'd6)  ? ne_bv[0]    :
                          (carry_cnt == 3'd5)  ? ne_bv[1]    :
                          (carry_cnt == 3'd4)  ? ne_bv[2]    :
                          (carry_cnt == 3'd3)  ? ne_bv[3]    :
                          (carry_cnt == 3'd2)  ? ne_bv[4]    :
                          (carry_cnt == 3'd1)  ? ne_bv[5]    : ne_bv[6];
    wire [15:0] out_bv7 = (carry_cnt == 3'd7)  ? ne_bv[0]   :
                          (carry_cnt == 3'd6)  ? ne_bv[1]    :
                          (carry_cnt == 3'd5)  ? ne_bv[2]    :
                          (carry_cnt == 3'd4)  ? ne_bv[3]    :
                          (carry_cnt == 3'd3)  ? ne_bv[4]    :
                          (carry_cnt == 3'd2)  ? ne_bv[5]    :
                          (carry_cnt == 3'd1)  ? ne_bv[6]    : ne_bv[7];

    // Output col_id per slot
    wire [15:0] out_bc0 = (carry_cnt >= 3'd1) ? carry_bc[0] : ne_bc[0];
    wire [15:0] out_bc1 = (carry_cnt >= 3'd2) ? carry_bc[1] :
                          (carry_cnt == 3'd1)  ? ne_bc[0]    : ne_bc[1];
    wire [15:0] out_bc2 = (carry_cnt >= 3'd3) ? carry_bc[2] :
                          (carry_cnt == 3'd2)  ? ne_bc[0]    :
                          (carry_cnt == 3'd1)  ? ne_bc[1]    : ne_bc[2];
    wire [15:0] out_bc3 = (carry_cnt >= 3'd4) ? carry_bc[3] :
                          (carry_cnt == 3'd3)  ? ne_bc[0]    :
                          (carry_cnt == 3'd2)  ? ne_bc[1]    :
                          (carry_cnt == 3'd1)  ? ne_bc[2]    : ne_bc[3];
    wire [15:0] out_bc4 = (carry_cnt >= 3'd5) ? carry_bc[4] :
                          (carry_cnt == 3'd4)  ? ne_bc[0]    :
                          (carry_cnt == 3'd3)  ? ne_bc[1]    :
                          (carry_cnt == 3'd2)  ? ne_bc[2]    :
                          (carry_cnt == 3'd1)  ? ne_bc[3]    : ne_bc[4];
    wire [15:0] out_bc5 = (carry_cnt >= 3'd6) ? carry_bc[5] :
                          (carry_cnt == 3'd5)  ? ne_bc[0]    :
                          (carry_cnt == 3'd4)  ? ne_bc[1]    :
                          (carry_cnt == 3'd3)  ? ne_bc[2]    :
                          (carry_cnt == 3'd2)  ? ne_bc[3]    :
                          (carry_cnt == 3'd1)  ? ne_bc[4]    : ne_bc[5];
    wire [15:0] out_bc6 = (carry_cnt >= 3'd7) ? carry_bc[6] :
                          (carry_cnt == 3'd6)  ? ne_bc[0]    :
                          (carry_cnt == 3'd5)  ? ne_bc[1]    :
                          (carry_cnt == 3'd4)  ? ne_bc[2]    :
                          (carry_cnt == 3'd3)  ? ne_bc[3]    :
                          (carry_cnt == 3'd2)  ? ne_bc[4]    :
                          (carry_cnt == 3'd1)  ? ne_bc[5]    : ne_bc[6];
    wire [15:0] out_bc7 = (carry_cnt == 3'd7)  ? ne_bc[0]   :
                          (carry_cnt == 3'd6)  ? ne_bc[1]    :
                          (carry_cnt == 3'd5)  ? ne_bc[2]    :
                          (carry_cnt == 3'd4)  ? ne_bc[3]    :
                          (carry_cnt == 3'd3)  ? ne_bc[4]    :
                          (carry_cnt == 3'd2)  ? ne_bc[5]    :
                          (carry_cnt == 3'd1)  ? ne_bc[6]    : ne_bc[7];

    // Task group slots for pack emit: {b_val[15:0], a_val[15:0], col_id[8:0]}  (41-bit)
    wire [`TASK_WIDTH-1:0] pack_sg0 = {out_bv0, out_av0, out_bc0[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg1 = {out_bv1, out_av1, out_bc1[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg2 = {out_bv2, out_av2, out_bc2[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg3 = {out_bv3, out_av3, out_bc3[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg4 = {out_bv4, out_av4, out_bc4[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg5 = {out_bv5, out_av5, out_bc5[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg6 = {out_bv6, out_av6, out_bc6[8:0]};
    wire [`TASK_WIDTH-1:0] pack_sg7 = {out_bv7, out_av7, out_bc7[8:0]};

    // Task group slots for flush emit (only carry_cnt lanes valid)
    wire [`TASK_WIDTH-1:0] flush_sg0 = {carry_bv[0], carry_av[0], carry_bc[0][8:0]};
    wire [`TASK_WIDTH-1:0] flush_sg1 = {carry_bv[1], carry_av[1], carry_bc[1][8:0]};
    wire [`TASK_WIDTH-1:0] flush_sg2 = {carry_bv[2], carry_av[2], carry_bc[2][8:0]};
    wire [`TASK_WIDTH-1:0] flush_sg3 = {carry_bv[3], carry_av[3], carry_bc[3][8:0]};
    wire [`TASK_WIDTH-1:0] flush_sg4 = {carry_bv[4], carry_av[4], carry_bc[4][8:0]};
    wire [`TASK_WIDTH-1:0] flush_sg5 = {carry_bv[5], carry_av[5], carry_bc[5][8:0]};
    wire [`TASK_WIDTH-1:0] flush_sg6 = {carry_bv[6], carry_av[6], carry_bc[6][8:0]};
    wire [7:0] flush_lane_valid = (carry_cnt == 3'd1) ? 8'b0000_0001 :
                                  (carry_cnt == 3'd2) ? 8'b0000_0011 :
                                  (carry_cnt == 3'd3) ? 8'b0000_0111 :
                                  (carry_cnt == 3'd4) ? 8'b0000_1111 :
                                  (carry_cnt == 3'd5) ? 8'b0001_1111 :
                                  (carry_cnt == 3'd6) ? 8'b0011_1111 : 8'b0111_1111;

    wire task_fifo_full;
    wire do_pack_emit  = (gen_state == GEN_EMIT)  && pack_can_emit;
    wire do_flush_emit = (gen_state == GEN_FLUSH);

    wire task_group_wr_en = (do_pack_emit || do_flush_emit) && !task_fifo_full;

    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data =
        do_flush_emit
        ? {flush_sg6, flush_sg5, flush_sg4, flush_sg3, flush_sg2, flush_sg1, flush_sg0, flush_lane_valid}
        : {pack_sg7, pack_sg6, pack_sg5, pack_sg4, pack_sg3, pack_sg2, pack_sg1, pack_sg0, 8'b1111_1111};

    //=========================================================================
    // Generator sub-FSM sequential
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            gen_state <= GEN_IDLE;
            gen_t     <= 0;
            gen_g     <= 0;
            gen_a_val <= 0;
            gen_b_off <= 0;
            gen_b_nnz <= 0;
            carry_cnt <= 3'd0;
            carry_av[0] <= 0; carry_av[1] <= 0; carry_av[2] <= 0;
            carry_av[3] <= 0; carry_av[4] <= 0; carry_av[5] <= 0; carry_av[6] <= 0;
            carry_bv[0] <= 0; carry_bv[1] <= 0; carry_bv[2] <= 0;
            carry_bv[3] <= 0; carry_bv[4] <= 0; carry_bv[5] <= 0; carry_bv[6] <= 0;
            carry_bc[0] <= 0; carry_bc[1] <= 0; carry_bc[2] <= 0;
            carry_bc[3] <= 0; carry_bc[4] <= 0; carry_bc[5] <= 0; carry_bc[6] <= 0;
        end else begin
            case (gen_state)

                GEN_IDLE: begin
                    if (state == PE_CLEAR_ACC) begin
                        gen_t     <= 0;
                        gen_g     <= 0;
                        carry_cnt <= 3'd0;  // reset carry at start of each A row
                        if (cur_a_nnz == 16'd0)
                            gen_state <= GEN_ROW_DONE;
                        else
                            gen_state <= GEN_FETCH;
                    end
                end

                GEN_FETCH: begin
                    gen_a_val <= fetch_a_val;
                    gen_b_off <= fetch_b_off;
                    gen_b_nnz <= fetch_b_nnz;
                    gen_t     <= gen_t + 16'd1;
                    gen_g     <= 0;
                    if (fetch_b_nnz == 16'd0) begin
                        // Skip empty B row
                        if (gen_t + 16'd1 >= cur_a_nnz) begin
                            // No more A nnzs — flush carry if any
                            if (carry_cnt != 3'd0)
                                gen_state <= GEN_FLUSH;
                            else
                                gen_state <= GEN_ROW_DONE;
                        end
                        // else stay in GEN_FETCH to try next A nnz
                    end else begin
                        gen_state <= GEN_EMIT;
                    end
                end

                GEN_EMIT: begin
                    if (pack_can_emit) begin
                        // -------------------------------------------------------
                        // Emit case: carry_cnt + gen_cnt >= 8 → write one group
                        // -------------------------------------------------------
                        if (!task_fifo_full) begin
                            // Update carry: new carry[j] = ne[8-carry_cnt+j]
                            carry_cnt <= overflow_cnt; // = pack_total[2:0]

                            // carry[0] = ne[8-carry_cnt] (when overflow_cnt >= 1)
                            if (overflow_cnt >= 3'd1) begin
                                carry_av[0] <= gen_a_val;
                                carry_bv[0] <= (carry_cnt==3'd1)?ne_bv[7]:(carry_cnt==3'd2)?ne_bv[6]:
                                               (carry_cnt==3'd3)?ne_bv[5]:(carry_cnt==3'd4)?ne_bv[4]:
                                               (carry_cnt==3'd5)?ne_bv[3]:(carry_cnt==3'd6)?ne_bv[2]:ne_bv[1];
                                carry_bc[0] <= (carry_cnt==3'd1)?ne_bc[7]:(carry_cnt==3'd2)?ne_bc[6]:
                                               (carry_cnt==3'd3)?ne_bc[5]:(carry_cnt==3'd4)?ne_bc[4]:
                                               (carry_cnt==3'd5)?ne_bc[3]:(carry_cnt==3'd6)?ne_bc[2]:ne_bc[1];
                            end
                            // carry[1] = ne[9-carry_cnt] (when overflow_cnt >= 2)
                            if (overflow_cnt >= 3'd2) begin
                                carry_av[1] <= gen_a_val;
                                carry_bv[1] <= (carry_cnt==3'd2)?ne_bv[7]:(carry_cnt==3'd3)?ne_bv[6]:
                                               (carry_cnt==3'd4)?ne_bv[5]:(carry_cnt==3'd5)?ne_bv[4]:
                                               (carry_cnt==3'd6)?ne_bv[3]:ne_bv[2];
                                carry_bc[1] <= (carry_cnt==3'd2)?ne_bc[7]:(carry_cnt==3'd3)?ne_bc[6]:
                                               (carry_cnt==3'd4)?ne_bc[5]:(carry_cnt==3'd5)?ne_bc[4]:
                                               (carry_cnt==3'd6)?ne_bc[3]:ne_bc[2];
                            end
                            // carry[2] = ne[10-carry_cnt] (when overflow_cnt >= 3)
                            if (overflow_cnt >= 3'd3) begin
                                carry_av[2] <= gen_a_val;
                                carry_bv[2] <= (carry_cnt==3'd3)?ne_bv[7]:(carry_cnt==3'd4)?ne_bv[6]:
                                               (carry_cnt==3'd5)?ne_bv[5]:(carry_cnt==3'd6)?ne_bv[4]:ne_bv[3];
                                carry_bc[2] <= (carry_cnt==3'd3)?ne_bc[7]:(carry_cnt==3'd4)?ne_bc[6]:
                                               (carry_cnt==3'd5)?ne_bc[5]:(carry_cnt==3'd6)?ne_bc[4]:ne_bc[3];
                            end
                            // carry[3] = ne[11-carry_cnt] (when overflow_cnt >= 4)
                            if (overflow_cnt >= 3'd4) begin
                                carry_av[3] <= gen_a_val;
                                carry_bv[3] <= (carry_cnt==3'd4)?ne_bv[7]:(carry_cnt==3'd5)?ne_bv[6]:
                                               (carry_cnt==3'd6)?ne_bv[5]:ne_bv[4];
                                carry_bc[3] <= (carry_cnt==3'd4)?ne_bc[7]:(carry_cnt==3'd5)?ne_bc[6]:
                                               (carry_cnt==3'd6)?ne_bc[5]:ne_bc[4];
                            end
                            // carry[4] = ne[12-carry_cnt] (when overflow_cnt >= 5)
                            if (overflow_cnt >= 3'd5) begin
                                carry_av[4] <= gen_a_val;
                                carry_bv[4] <= (carry_cnt==3'd5)?ne_bv[7]:(carry_cnt==3'd6)?ne_bv[6]:ne_bv[5];
                                carry_bc[4] <= (carry_cnt==3'd5)?ne_bc[7]:(carry_cnt==3'd6)?ne_bc[6]:ne_bc[5];
                            end
                            // carry[5] = ne[13-carry_cnt] (when overflow_cnt >= 6)
                            if (overflow_cnt >= 3'd6) begin
                                carry_av[5] <= gen_a_val;
                                carry_bv[5] <= (carry_cnt==3'd6)?ne_bv[7]:ne_bv[6];
                                carry_bc[5] <= (carry_cnt==3'd6)?ne_bc[7]:ne_bc[6];
                            end
                            // carry[6] = ne[14-carry_cnt] = ne[7] only when carry_cnt==7
                            if (overflow_cnt == 3'd7) begin
                                carry_av[6] <= gen_a_val;
                                carry_bv[6] <= ne_bv[7];
                                carry_bc[6] <= ne_bc[7];
                            end

                            // State / counter transitions
                            if (gen_last_grp) begin
                                if (gen_t >= cur_a_nnz) begin
                                    // All A nnzs consumed
                                    if (overflow_cnt != 3'b0)
                                        gen_state <= GEN_FLUSH;
                                    else
                                        gen_state <= GEN_ROW_DONE;
                                end else if (fetch_b_nnz == 16'd0) begin
                                    // Next A nnz has empty B row — skip
                                    gen_t <= gen_t + 16'd1;
                                    gen_g <= 0;
                                    if (gen_t + 16'd1 >= cur_a_nnz) begin
                                        if (overflow_cnt != 3'b0)
                                            gen_state <= GEN_FLUSH;
                                        else
                                            gen_state <= GEN_ROW_DONE;
                                    end else
                                        gen_state <= GEN_FETCH;
                                end else begin
                                    // Zero-overhead prefetch: load next A nnz
                                    gen_a_val <= fetch_a_val;
                                    gen_b_off <= fetch_b_off;
                                    gen_b_nnz <= fetch_b_nnz;
                                    gen_t     <= gen_t + 16'd1;
                                    gen_g     <= 0;
                                    // stay in GEN_EMIT
                                end
                            end else begin
                                gen_g <= gen_g + 16'd1;
                            end
                        end
                        // else task_fifo_full: stall, no updates

                    end else begin
                        // -------------------------------------------------------
                        // Accumulate case: carry_cnt + gen_cnt < 8.
                        // Only possible at last group (gen_cnt < 8).
                        // No FIFO write, always proceed.
                        // -------------------------------------------------------
                        carry_cnt <= pack_total[2:0]; // carry_cnt + gen_cnt (< 8)

                        // Append ne[0..gen_cnt-1] to carry starting at carry[carry_cnt]
                        case (carry_cnt)
                            3'd0: begin
                                if (gen_cnt >= 4'd1) begin carry_av[0] <= gen_a_val; carry_bv[0] <= ne_bv[0]; carry_bc[0] <= ne_bc[0]; end
                                if (gen_cnt >= 4'd2) begin carry_av[1] <= gen_a_val; carry_bv[1] <= ne_bv[1]; carry_bc[1] <= ne_bc[1]; end
                                if (gen_cnt >= 4'd3) begin carry_av[2] <= gen_a_val; carry_bv[2] <= ne_bv[2]; carry_bc[2] <= ne_bc[2]; end
                                if (gen_cnt >= 4'd4) begin carry_av[3] <= gen_a_val; carry_bv[3] <= ne_bv[3]; carry_bc[3] <= ne_bc[3]; end
                                if (gen_cnt >= 4'd5) begin carry_av[4] <= gen_a_val; carry_bv[4] <= ne_bv[4]; carry_bc[4] <= ne_bc[4]; end
                                if (gen_cnt >= 4'd6) begin carry_av[5] <= gen_a_val; carry_bv[5] <= ne_bv[5]; carry_bc[5] <= ne_bc[5]; end
                                if (gen_cnt >= 4'd7) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[6]; carry_bc[6] <= ne_bc[6]; end
                            end
                            3'd1: begin
                                if (gen_cnt >= 4'd1) begin carry_av[1] <= gen_a_val; carry_bv[1] <= ne_bv[0]; carry_bc[1] <= ne_bc[0]; end
                                if (gen_cnt >= 4'd2) begin carry_av[2] <= gen_a_val; carry_bv[2] <= ne_bv[1]; carry_bc[2] <= ne_bc[1]; end
                                if (gen_cnt >= 4'd3) begin carry_av[3] <= gen_a_val; carry_bv[3] <= ne_bv[2]; carry_bc[3] <= ne_bc[2]; end
                                if (gen_cnt >= 4'd4) begin carry_av[4] <= gen_a_val; carry_bv[4] <= ne_bv[3]; carry_bc[4] <= ne_bc[3]; end
                                if (gen_cnt >= 4'd5) begin carry_av[5] <= gen_a_val; carry_bv[5] <= ne_bv[4]; carry_bc[5] <= ne_bc[4]; end
                                if (gen_cnt >= 4'd6) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[5]; carry_bc[6] <= ne_bc[5]; end
                            end
                            3'd2: begin
                                if (gen_cnt >= 4'd1) begin carry_av[2] <= gen_a_val; carry_bv[2] <= ne_bv[0]; carry_bc[2] <= ne_bc[0]; end
                                if (gen_cnt >= 4'd2) begin carry_av[3] <= gen_a_val; carry_bv[3] <= ne_bv[1]; carry_bc[3] <= ne_bc[1]; end
                                if (gen_cnt >= 4'd3) begin carry_av[4] <= gen_a_val; carry_bv[4] <= ne_bv[2]; carry_bc[4] <= ne_bc[2]; end
                                if (gen_cnt >= 4'd4) begin carry_av[5] <= gen_a_val; carry_bv[5] <= ne_bv[3]; carry_bc[5] <= ne_bc[3]; end
                                if (gen_cnt >= 4'd5) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[4]; carry_bc[6] <= ne_bc[4]; end
                            end
                            3'd3: begin
                                if (gen_cnt >= 4'd1) begin carry_av[3] <= gen_a_val; carry_bv[3] <= ne_bv[0]; carry_bc[3] <= ne_bc[0]; end
                                if (gen_cnt >= 4'd2) begin carry_av[4] <= gen_a_val; carry_bv[4] <= ne_bv[1]; carry_bc[4] <= ne_bc[1]; end
                                if (gen_cnt >= 4'd3) begin carry_av[5] <= gen_a_val; carry_bv[5] <= ne_bv[2]; carry_bc[5] <= ne_bc[2]; end
                                if (gen_cnt >= 4'd4) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[3]; carry_bc[6] <= ne_bc[3]; end
                            end
                            3'd4: begin
                                if (gen_cnt >= 4'd1) begin carry_av[4] <= gen_a_val; carry_bv[4] <= ne_bv[0]; carry_bc[4] <= ne_bc[0]; end
                                if (gen_cnt >= 4'd2) begin carry_av[5] <= gen_a_val; carry_bv[5] <= ne_bv[1]; carry_bc[5] <= ne_bc[1]; end
                                if (gen_cnt >= 4'd3) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[2]; carry_bc[6] <= ne_bc[2]; end
                            end
                            3'd5: begin
                                if (gen_cnt >= 4'd1) begin carry_av[5] <= gen_a_val; carry_bv[5] <= ne_bv[0]; carry_bc[5] <= ne_bc[0]; end
                                if (gen_cnt >= 4'd2) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[1]; carry_bc[6] <= ne_bc[1]; end
                            end
                            3'd6: begin
                                if (gen_cnt >= 4'd1) begin carry_av[6] <= gen_a_val; carry_bv[6] <= ne_bv[0]; carry_bc[6] <= ne_bc[0]; end
                            end
                            default: ; // carry_cnt==7: gen_cnt must be 0, impossible here
                        endcase

                        // State transitions (always at last group)
                        if (gen_t >= cur_a_nnz) begin
                            // No more A nnzs — flush carry
                            gen_state <= GEN_FLUSH;
                        end else if (fetch_b_nnz == 16'd0) begin
                            gen_t <= gen_t + 16'd1;
                            gen_g <= 0;
                            if (gen_t + 16'd1 >= cur_a_nnz)
                                gen_state <= GEN_FLUSH;
                            else
                                gen_state <= GEN_FETCH;
                        end else begin
                            // Zero-overhead prefetch: next A nnz has non-empty B row
                            gen_a_val <= fetch_a_val;
                            gen_b_off <= fetch_b_off;
                            gen_b_nnz <= fetch_b_nnz;
                            gen_t     <= gen_t + 16'd1;
                            gen_g     <= 0;
                            // stay in GEN_EMIT
                        end
                    end
                end

                GEN_FLUSH: begin
                    // Emit partial group using carry buffer, then done
                    if (!task_fifo_full) begin
                        carry_cnt <= 3'd0;
                        gen_state <= GEN_ROW_DONE;
                    end
                end

                GEN_ROW_DONE: begin
                    if (state == PE_WAIT_TASK_DRAIN || state == PE_NEXT_ROW ||
                        state == PE_WAIT_PRODUCT_DRAIN)
                        gen_state <= GEN_IDLE;
                end

                default: gen_state <= GEN_IDLE;

            endcase
        end
    end

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
    wire [`N_MAC-1:0]             mac_lane_valid;
    wire [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task;

    // 1-cycle pipeline for synchronous (BRAM) FIFO read
    reg                          task_fifo_rd_en_d1;
    reg [`TASK_GROUP_WIDTH-1:0]  task_fifo_rd_data_d1;
    always @(posedge aclk) begin
        if (!aresetn) begin
            task_fifo_rd_en_d1   <= 1'b0;
            task_fifo_rd_data_d1 <= {`TASK_GROUP_WIDTH{1'b0}};
        end else begin
            task_fifo_rd_en_d1   <= task_fifo_rd_en && !task_fifo_empty;
            task_fifo_rd_data_d1 <= task_fifo_rd_data;
        end
    end

    reg [`N_MAC-1:0]              mac_lane_valid_r;
    reg [`N_MAC*`TASK_WIDTH-1:0]  mac_lane_task_r;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            mac_lane_valid_r <= 0;
            mac_lane_task_r  <= 0;
        end else if (task_fifo_rd_en_d1) begin
            mac_lane_valid_r <= task_fifo_rd_data_d1[`N_MAC-1:0];
            mac_lane_task_r[0*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 0*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[1*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 1*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[2*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 2*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[3*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 3*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[4*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 4*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[5*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 5*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[6*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 6*`TASK_WIDTH +: `TASK_WIDTH];
            mac_lane_task_r[7*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[`N_MAC + 7*`TASK_WIDTH +: `TASK_WIDTH];
        end else begin
            mac_lane_valid_r <= 0;
        end
    end
    assign mac_lane_valid = mac_lane_valid_r;
    assign mac_lane_task  = mac_lane_task_r;

    wire [`N_MAC-1:0]                mul_valid;
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
    // Dual Product Group FIFOs (one per accumulator)
    //
    // MAC writes to the ACTIVE FIFO (selected by comp_sel).  Each FIFO drains
    // independently into its own accumulator, so the next row can start as
    // soon as the MAC pipeline is idle — no need to wait for the FIFO to drain.
    //=========================================================================
    wire [`PRODUCT_GROUP_WIDTH-1:0] product_group_wr_data;
    wire product_fifo_full_0, product_fifo_full_1;
    wire [`PROD_FIFO_DEPTH_LOG:0] product_fifo_cnt_0, product_fifo_cnt_1;

    // Only write to the FIFO that belongs to the current accumulator
    wire product_fifo_full   = comp_sel ? product_fifo_full_1  : product_fifo_full_0;
    wire product_group_wr_en = |mul_valid && !product_fifo_full;

    assign product_group_wr_data[`N_MAC-1:0]                           = mul_valid;
    assign product_group_wr_data[`N_MAC+0*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[0*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+1*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[1*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+2*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[2*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+3*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[3*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+4*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[4*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+5*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[5*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+6*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[6*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    assign product_group_wr_data[`N_MAC+7*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = mul_product[7*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];

    wire prod_fifo_rd_en_0,  prod_fifo_rd_en_1;
    wire [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data_0, prod_fifo_rd_data_1;
    wire prod_fifo_empty_0,  prod_fifo_empty_1;

    sync_fifo #(
        .WIDTH(`PRODUCT_GROUP_WIDTH), .DEPTH(`PROD_FIFO_DEPTH),
        .DEPTH_LOG(`PROD_FIFO_DEPTH_LOG)
    ) u_product_fifo_0 (
        .wr_en    (product_group_wr_en && !comp_sel),
        .wr_data  (product_group_wr_data),
        .wr_full  (product_fifo_full_0),
        .rd_en    (prod_fifo_rd_en_0),
        .rd_data  (prod_fifo_rd_data_0),
        .rd_empty (prod_fifo_empty_0),
        .count    (product_fifo_cnt_0),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    sync_fifo #(
        .WIDTH(`PRODUCT_GROUP_WIDTH), .DEPTH(`PROD_FIFO_DEPTH),
        .DEPTH_LOG(`PROD_FIFO_DEPTH_LOG)
    ) u_product_fifo_1 (
        .wr_en    (product_group_wr_en &&  comp_sel),
        .wr_data  (product_group_wr_data),
        .wr_full  (product_fifo_full_1),
        .rd_en    (prod_fifo_rd_en_1),
        .rd_data  (prod_fifo_rd_data_1),
        .rd_empty (prod_fifo_empty_1),
        .count    (product_fifo_cnt_1),
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

    wire [7:0]             drain_valid_0,  drain_valid_1;
    wire [BANK_ADDR_W-1:0] drain_gaddr_0,  drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] drain_row_id_0, drain_row_id_1;
    wire [8*16-1:0]        drain_values_0, drain_values_1;

    wire other_acc_busy = comp_sel ? acc_busy_0 : acc_busy_1;

    assign a_desc_ready  = (state == PE_LOAD_ROW_DESC);

    wire acc_row_start_0 = (state == PE_CLEAR_ACC) && !comp_sel;
    wire acc_row_start_1 = (state == PE_CLEAR_ACC) &&  comp_sel;

    // When the PE FSM is satisfied (MAC idle + other acc done), record which FIFO
    // just finished being written.  Then fire acc_inp_done_X once that FIFO has
    // actually drained to empty, so row_accumulator never drops products.
    wire pe_drain_done = (state == PE_WAIT_PRODUCT_DRAIN)
                       && mac_pipeline_idle && !other_acc_busy;

    reg mac_done_latch_0, mac_done_latch_1;
    always @(posedge aclk) begin
        if (!aresetn) begin
            mac_done_latch_0 <= 1'b0;
            mac_done_latch_1 <= 1'b0;
        end else begin
            if (pe_drain_done && !comp_sel) mac_done_latch_0 <= 1'b1;
            if (pe_drain_done &&  comp_sel) mac_done_latch_1 <= 1'b1;
            if (mac_done_latch_0 && prod_fifo_empty_0) mac_done_latch_0 <= 1'b0;
            if (mac_done_latch_1 && prod_fifo_empty_1) mac_done_latch_1 <= 1'b0;
        end
    end

    wire acc_inp_done_0 = mac_done_latch_0 && prod_fifo_empty_0;
    wire acc_inp_done_1 = mac_done_latch_1 && prod_fifo_empty_1;

    // Each FIFO drains into its accumulator autonomously (independent of comp_sel)
    wire issue_valid_0 = !prod_fifo_empty_0;
    wire issue_valid_1 = !prod_fifo_empty_1;

    assign prod_fifo_rd_en_0 = !prod_fifo_empty_0 && acc_issue_ready_0;
    assign prod_fifo_rd_en_1 = !prod_fifo_empty_1 && acc_issue_ready_1;

    // 1-cycle pipeline for synchronous FIFO read (one register per FIFO)
    reg                            prod_fifo_rd_en_d1_0,  prod_fifo_rd_en_d1_1;
    reg [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data_d1_0, prod_fifo_rd_data_d1_1;
    always @(posedge aclk) begin
        if (!aresetn) begin
            prod_fifo_rd_en_d1_0   <= 1'b0;
            prod_fifo_rd_data_d1_0 <= {`PRODUCT_GROUP_WIDTH{1'b0}};
            prod_fifo_rd_en_d1_1   <= 1'b0;
            prod_fifo_rd_data_d1_1 <= {`PRODUCT_GROUP_WIDTH{1'b0}};
        end else begin
            prod_fifo_rd_en_d1_0   <= prod_fifo_rd_en_0 && !prod_fifo_empty_0;
            prod_fifo_rd_data_d1_0 <= prod_fifo_rd_data_0;
            prod_fifo_rd_en_d1_1   <= prod_fifo_rd_en_1 && !prod_fifo_empty_1;
            prod_fifo_rd_data_d1_1 <= prod_fifo_rd_data_1;
        end
    end

    wire [7:0]    acc_lane_valid_0;
    wire [8*9-1:0]  acc_lane_col_id_0;
    wire [8*16-1:0] acc_lane_product_0;
    assign acc_lane_valid_0   = prod_fifo_rd_data_d1_0[`N_MAC-1:0];
    assign acc_lane_col_id_0  = {prod_fifo_rd_data_d1_0[`N_MAC+7*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+6*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+5*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+4*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+3*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+2*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+1*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_0[`N_MAC+0*`PRODUCT_WIDTH+16 +: 9]};
    assign acc_lane_product_0 = {prod_fifo_rd_data_d1_0[`N_MAC+7*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+6*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+5*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+4*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+3*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+2*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+1*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_0[`N_MAC+0*`PRODUCT_WIDTH +: 16]};

    wire [7:0]    acc_lane_valid_1;
    wire [8*9-1:0]  acc_lane_col_id_1;
    wire [8*16-1:0] acc_lane_product_1;
    assign acc_lane_valid_1   = prod_fifo_rd_data_d1_1[`N_MAC-1:0];
    assign acc_lane_col_id_1  = {prod_fifo_rd_data_d1_1[`N_MAC+7*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+6*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+5*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+4*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+3*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+2*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+1*`PRODUCT_WIDTH+16 +: 9],
                                  prod_fifo_rd_data_d1_1[`N_MAC+0*`PRODUCT_WIDTH+16 +: 9]};
    assign acc_lane_product_1 = {prod_fifo_rd_data_d1_1[`N_MAC+7*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+6*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+5*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+4*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+3*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+2*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+1*`PRODUCT_WIDTH +: 16],
                                  prod_fifo_rd_data_d1_1[`N_MAC+0*`PRODUCT_WIDTH +: 16]};

    row_accumulator_8bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(16),
        .ACC_W(16), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5),
        .ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_0 (
        .clk(aclk), .rst_n(aresetn),
        .row_start(acc_row_start_0), .row_id_in(row_idx), .drain_cols(N),
        .row_input_done(acc_inp_done_0), .busy(acc_busy_0), .row_done(acc_row_done_0),
        .issue_valid(issue_valid_0), .issue_ready(acc_issue_ready_0),
        .lane_valid(acc_lane_valid_0), .lane_col_id(acc_lane_col_id_0), .lane_product(acc_lane_product_0),
        .drain_valid(drain_valid_0), .drain_gaddr(drain_gaddr_0),
        .drain_row_id(drain_row_id_0), .drain_values(drain_values_0)
    );

    row_accumulator_8bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(16),
        .ACC_W(16), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5),
        .ROW_W(`A_ROW_ADDR_BITS)
    ) u_row_acc_1 (
        .clk(aclk), .rst_n(aresetn),
        .row_start(acc_row_start_1), .row_id_in(row_idx), .drain_cols(N),
        .row_input_done(acc_inp_done_1), .busy(acc_busy_1), .row_done(acc_row_done_1),
        .issue_valid(issue_valid_1), .issue_ready(acc_issue_ready_1),
        .lane_valid(acc_lane_valid_1), .lane_col_id(acc_lane_col_id_1), .lane_product(acc_lane_product_1),
        .drain_valid(drain_valid_1), .drain_gaddr(drain_gaddr_1),
        .drain_row_id(drain_row_id_1), .drain_values(drain_values_1)
    );

    //=========================================================================
    // Task FIFO read control
    //=========================================================================
    // Throttle task reads based on the ACTIVE product FIFO to prevent overflow
    wire [`PROD_FIFO_DEPTH_LOG:0] active_prod_fifo_cnt =
        comp_sel ? product_fifo_cnt_1 : product_fifo_cnt_0;
    assign task_fifo_rd_en = !task_fifo_empty &&
                             (active_prod_fifo_cnt < (`PROD_FIFO_DEPTH - `MUL_LAT - 1));

    //=========================================================================
    // C buffer write (disabled — c_bank removed)
    //=========================================================================
    // wire [3:0]               c_wr_valid = comp_sel ? drain_valid_0  : drain_valid_1;
    // wire [BANK_ADDR_W-1:0]   c_wr_gaddr = comp_sel ? drain_gaddr_0  : drain_gaddr_1;
    // wire [`A_ROW_ADDR_BITS-1:0] c_wr_rid = comp_sel ? drain_row_id_0 : drain_row_id_1;
    // wire [4*16-1:0]          c_wr_vals  = comp_sel ? drain_values_0 : drain_values_1;
    // wire [C_BANK_ADDR_W-1:0] c_wr_addr  = {c_wr_rid, c_wr_gaddr};

    // always @(posedge aclk) begin
    //     if (c_wr_valid[0]) c_bank0[c_wr_addr] <= c_wr_vals[0*16 +: 16];
    //     if (c_wr_valid[1]) c_bank1[c_wr_addr] <= c_wr_vals[1*16 +: 16];
    //     if (c_wr_valid[2]) c_bank2[c_wr_addr] <= c_wr_vals[2*16 +: 16];
    //     if (c_wr_valid[3]) c_bank3[c_wr_addr] <= c_wr_vals[3*16 +: 16];
    // end

    //=========================================================================
    // C buffer read (disabled — c_bank removed)
    //=========================================================================
    // wire [1:0]               rd_bank  = c_rd_addr[1:0];
    // wire [C_BANK_ADDR_W-1:0] rd_baddr = {c_rd_addr[C_RD_ADDR_W-1:ACC_COL_W],
    //                                       c_rd_addr[ACC_COL_W-1:2]};

    // always @(posedge aclk) begin
    //     if (c_rd_en) begin
    //         case (rd_bank)
    //             2'd0: c_rd_data <= c_bank0[rd_baddr];
    //             2'd1: c_rd_data <= c_bank1[rd_baddr];
    //             2'd2: c_rd_data <= c_bank2[rd_baddr];
    //             2'd3: c_rd_data <= c_bank3[rd_baddr];
    //         endcase
    //     end
    // end

    //=========================================================================
    // Main FSM sequential
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) state <= PE_IDLE;
        else          state <= state_next;
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            comp_sel    <= 1'b0;
            row_idx     <= 0;
            cur_a_off   <= 0;
            cur_a_nnz   <= 0;
            done        <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                PE_IDLE: begin
                    if (start) row_idx <= 0;
                end

                PE_LOAD_ROW_DESC: begin
                    if (a_desc_valid) begin
                        cur_a_off <= {18'b0, a_desc_data[32:19]};
                        cur_a_nnz <= {6'b0,  a_desc_data[18:9]};
                    end
                end

                PE_NEXT_ROW: begin
                    row_idx  <= row_idx + 1;
                    comp_sel <= ~comp_sel;
                end

                PE_DONE: begin
                    if (!other_acc_busy) done <= 1'b1;
                end
            endcase
        end
    end

    //=========================================================================
    // Main FSM next-state logic
    //=========================================================================
    always @(*) begin
        state_next = state;
        case (state)
            PE_IDLE:
                if (start) state_next = PE_LOAD_ROW_DESC;

            PE_LOAD_ROW_DESC:
                if (a_desc_valid) state_next = PE_CLEAR_ACC;

            PE_CLEAR_ACC:
                state_next = PE_STREAM_INSTRS;

            PE_STREAM_INSTRS:
                if (gen_state == GEN_ROW_DONE) state_next = PE_WAIT_TASK_DRAIN;

            PE_WAIT_TASK_DRAIN:
                if (task_fifo_empty) state_next = PE_WAIT_PRODUCT_DRAIN;

            PE_WAIT_PRODUCT_DRAIN:
                // Each product FIFO drains into its own accumulator independently,
                // so only wait for the MAC pipeline to flush into the active FIFO.
                if (mac_pipeline_idle && !other_acc_busy)
                    state_next = PE_NEXT_ROW;

            PE_NEXT_ROW:
                state_next = ((row_idx + 1) >= row_count) ? PE_DONE : PE_LOAD_ROW_DESC;

            PE_DONE:
                state_next = PE_DONE;
        endcase
    end

endmodule
