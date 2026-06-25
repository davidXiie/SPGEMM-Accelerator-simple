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
//   A row descriptor (64-bit):
//     [63:32] a_off   — start index into A_col/A_val buffers (local to PE)
//     [31:16] a_nnz   — number of A nonzeros in this row
//     [15: 0] c_row   — global C output row id (host readback only)
//
//   B descriptor (64-bit, broadcast):
//     [63:32] b_off   — start of B row in 4-bank storage (may be non-4-aligned)
//     [15: 0] b_nnz   — number of B nonzeros in this row
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

    // Row descriptor load  {a_off[31:0], a_nnz[15:0], c_row[15:0]}
    input  wire                          a_desc_we,
    input  wire [`A_ROW_ADDR_BITS-1:0]   a_desc_waddr,
    input  wire [63:0]                   a_desc_wdata,

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

    // B row descriptor (broadcast; {b_off[31:0], 0[31:16], b_nnz[15:0]})
    input  wire                          b_desc_we,
    input  wire [`MAX_DIM_BITS-1:0]     b_desc_waddr,
    input  wire [63:0]                   b_desc_wdata


    // C buffer read port (synchronous, 1-cycle latency)
    // input  wire                          c_rd_en,
    // input  wire [16:0]                   c_rd_addr,
    // output reg  [15:0]                   c_rd_data
);

    localparam ACC_COL_W     = 9;
    localparam BANK_ADDR_W   = ACC_COL_W - 2;
    // localparam C_BANK_ADDR_W = `A_ROW_ADDR_BITS + BANK_ADDR_W;
    // localparam C_BANK_DEPTH  = 32'd1 << C_BANK_ADDR_W;
    // localparam C_RD_ADDR_W   = `A_ROW_ADDR_BITS + ACC_COL_W;

    localparam B_BANK_DEPTH  = `B_NNZ_SLOT / 4;
    localparam B_DESC_DEPTH  = 1 << `MAX_DIM_BITS;

    //=========================================================================
    // SRAM declarations
    //=========================================================================
    reg [63:0]            A_row_desc_buf [0:`A_ROW_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_val_buf      [0:`A_NNZ_SLOT_PER_PE-1];
    reg [`DATA_WIDTH-1:0] A_col_buf      [0:`A_NNZ_SLOT_PER_PE-1];

    reg [`DATA_WIDTH-1:0] B_col_b0 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b1 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b2 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_col_b3 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b0 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b1 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b2 [0:B_BANK_DEPTH-1];
    reg [`DATA_WIDTH-1:0] B_val_b3 [0:B_BANK_DEPTH-1];

    reg [63:0]            B_desc_buf [0:B_DESC_DEPTH-1];

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
        if (a_desc_we) A_row_desc_buf[a_desc_waddr] <= a_desc_wdata;
        if (a_val_we)  A_val_buf[a_val_waddr]        <= a_val_wdata;
        if (a_col_we)  A_col_buf[a_col_waddr]         <= a_col_wdata;
        if (b_desc_we) B_desc_buf[b_desc_waddr]       <= b_desc_wdata;
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

    // Carry buffer: up to 3 elements from the tail of the previous B row
    reg [1:0]  carry_cnt;          // 0..3
    reg [15:0] carry_av [0:2];     // a_val per carry slot
    reg [15:0] carry_bv [0:2];     // b_val per carry slot
    reg [15:0] carry_bc [0:2];     // col_id per carry slot

    // 2-level combinatorial read chain
    wire [`A_NNZ_ADDR_BITS-1:0] fetch_a_addr =
        cur_a_off[`A_NNZ_ADDR_BITS-1:0] + gen_t[`A_NNZ_ADDR_BITS-1:0];
    wire [15:0] fetch_k_idx   = A_col_buf[fetch_a_addr];
    wire [15:0] fetch_a_val   = A_val_buf[fetch_a_addr];
    wire [63:0] fetch_b_desc  = B_desc_buf[fetch_k_idx[`MAX_DIM_BITS-1:0]];
    wire [31:0] fetch_b_off   = fetch_b_desc[63:32];
    wire [15:0] fetch_b_nnz   = fetch_b_desc[15:0];

    // Current group decode
    wire [31:0] gen_abs_base  = gen_b_off + {gen_g[13:0], 2'b00};
    wire [1:0]  gen_r         = gen_abs_base[1:0];
    wire [14:0] gen_m         = gen_abs_base[16:2];

    wire [14:0] gen_bg0 = (gen_r == 2'd0) ? gen_m : gen_m + 15'd1;
    wire [14:0] gen_bg1 = (gen_r <= 2'd1) ? gen_m : gen_m + 15'd1;
    wire [14:0] gen_bg2 = (gen_r <= 2'd2) ? gen_m : gen_m + 15'd1;
    wire [14:0] gen_bg3 = gen_m;

    wire [15:0] gen_remaining = gen_b_nnz - {gen_g[13:0], 2'b00};
    wire [3:0]  gen_cnt       = (gen_remaining >= 16'd4) ? 4'd4 : gen_remaining[3:0];
    wire        gen_last_grp  = (gen_remaining <= 16'd4);

    // B bank reads (4 lanes, combinatorial)
    wire [`DATA_WIDTH-1:0] bc0 = B_col_b0[gen_bg0];
    wire [`DATA_WIDTH-1:0] bv0 = B_val_b0[gen_bg0];
    wire [`DATA_WIDTH-1:0] bc1 = B_col_b1[gen_bg1];
    wire [`DATA_WIDTH-1:0] bv1 = B_val_b1[gen_bg1];
    wire [`DATA_WIDTH-1:0] bc2 = B_col_b2[gen_bg2];
    wire [`DATA_WIDTH-1:0] bv2 = B_val_b2[gen_bg2];
    wire [`DATA_WIDTH-1:0] bc3 = B_col_b3[gen_bg3];
    wire [`DATA_WIDTH-1:0] bv3 = B_val_b3[gen_bg3];

    //=========================================================================
    // New elements in sequential order (rotation-corrected)
    //
    // ne[j] = j-th valid element in this group (lane (gen_r+j)%4).
    // Valid lanes are gen_r, (gen_r+1)%4, ..., (gen_r+gen_cnt-1)%4.
    // Extracting them in this fixed order allows uniform carry buffer filling.
    //=========================================================================
    wire [15:0] ne_bv [0:3];
    wire [15:0] ne_bc [0:3];

    assign ne_bv[0] = (gen_r==2'd0)?bv0:(gen_r==2'd1)?bv1:(gen_r==2'd2)?bv2:bv3;
    assign ne_bc[0] = (gen_r==2'd0)?bc0:(gen_r==2'd1)?bc1:(gen_r==2'd2)?bc2:bc3;
    assign ne_bv[1] = (gen_r==2'd0)?bv1:(gen_r==2'd1)?bv2:(gen_r==2'd2)?bv3:bv0;
    assign ne_bc[1] = (gen_r==2'd0)?bc1:(gen_r==2'd1)?bc2:(gen_r==2'd2)?bc3:bc0;
    assign ne_bv[2] = (gen_r==2'd0)?bv2:(gen_r==2'd1)?bv3:(gen_r==2'd2)?bv0:bv1;
    assign ne_bc[2] = (gen_r==2'd0)?bc2:(gen_r==2'd1)?bc3:(gen_r==2'd2)?bc0:bc1;
    assign ne_bv[3] = (gen_r==2'd0)?bv3:(gen_r==2'd1)?bv0:(gen_r==2'd2)?bv1:bv2;
    assign ne_bc[3] = (gen_r==2'd0)?bc3:(gen_r==2'd1)?bc0:(gen_r==2'd2)?bc1:bc2;

    //=========================================================================
    // Pack logic
    //
    // pack_total  = carry_cnt + gen_cnt  (range 1..7, fits in 3 bits)
    // pack_can_emit = pack_total >= 4    (bit [2] of the 3-bit sum)
    // overflow_cnt  = pack_total - 4     (pack_total[1:0] when can_emit)
    //
    // Output slot k (when emit):
    //   k < carry_cnt → carry buffer slot k
    //   k >= carry_cnt → ne[k - carry_cnt] with gen_a_val
    //=========================================================================
    wire [2:0] pack_total    = {1'b0, carry_cnt} + {1'b0, gen_cnt[2:0]};
    wire       pack_can_emit = pack_total[2];          // >= 4
    wire [1:0] overflow_cnt  = pack_total[1:0];        // valid when pack_can_emit

    // Output a_val per slot
    wire [15:0] out_av0 = (carry_cnt >= 2'd1) ? carry_av[0] : gen_a_val;
    wire [15:0] out_av1 = (carry_cnt >= 2'd2) ? carry_av[1] : gen_a_val;
    wire [15:0] out_av2 = (carry_cnt >= 2'd3) ? carry_av[2] : gen_a_val;
    wire [15:0] out_av3 = gen_a_val;  // slot 3 always from new (carry_cnt <= 3)

    // Output b_val per slot (slot k: carry if k < carry_cnt, else ne[k-carry_cnt])
    wire [15:0] out_bv0 = (carry_cnt >= 2'd1) ? carry_bv[0] : ne_bv[0];
    wire [15:0] out_bv1 = (carry_cnt >= 2'd2) ? carry_bv[1] :
                          (carry_cnt == 2'd1)  ? ne_bv[0]    : ne_bv[1];
    wire [15:0] out_bv2 = (carry_cnt == 2'd3) ? carry_bv[2] :
                          (carry_cnt == 2'd2)  ? ne_bv[0]    :
                          (carry_cnt == 2'd1)  ? ne_bv[1]    : ne_bv[2];
    wire [15:0] out_bv3 = (carry_cnt == 2'd3) ? ne_bv[0]    :
                          (carry_cnt == 2'd2)  ? ne_bv[1]    :
                          (carry_cnt == 2'd1)  ? ne_bv[2]    : ne_bv[3];

    // Output col_id per slot
    wire [15:0] out_bc0 = (carry_cnt >= 2'd1) ? carry_bc[0] : ne_bc[0];
    wire [15:0] out_bc1 = (carry_cnt >= 2'd2) ? carry_bc[1] :
                          (carry_cnt == 2'd1)  ? ne_bc[0]    : ne_bc[1];
    wire [15:0] out_bc2 = (carry_cnt == 2'd3) ? carry_bc[2] :
                          (carry_cnt == 2'd2)  ? ne_bc[0]    :
                          (carry_cnt == 2'd1)  ? ne_bc[1]    : ne_bc[2];
    wire [15:0] out_bc3 = (carry_cnt == 2'd3) ? ne_bc[0]    :
                          (carry_cnt == 2'd2)  ? ne_bc[1]    :
                          (carry_cnt == 2'd1)  ? ne_bc[2]    : ne_bc[3];

    // Task group slots for pack emit (all 4 lanes valid, per-lane a_val)
    wire [`TASK_WIDTH-1:0] pack_sg0 = {16'd0, out_bv0, out_av0, out_bc0};
    wire [`TASK_WIDTH-1:0] pack_sg1 = {16'd0, out_bv1, out_av1, out_bc1};
    wire [`TASK_WIDTH-1:0] pack_sg2 = {16'd0, out_bv2, out_av2, out_bc2};
    wire [`TASK_WIDTH-1:0] pack_sg3 = {16'd0, out_bv3, out_av3, out_bc3};

    // Task group slots for flush emit (only carry_cnt lanes valid)
    wire [`TASK_WIDTH-1:0] flush_sg0 = {16'd0, carry_bv[0], carry_av[0], carry_bc[0]};
    wire [`TASK_WIDTH-1:0] flush_sg1 = {16'd0, carry_bv[1], carry_av[1], carry_bc[1]};
    wire [`TASK_WIDTH-1:0] flush_sg2 = {16'd0, carry_bv[2], carry_av[2], carry_bc[2]};
    wire [3:0] flush_lane_valid = (carry_cnt == 2'd1) ? 4'b0001 :
                                  (carry_cnt == 2'd2) ? 4'b0011 : 4'b0111;

    wire task_fifo_full;
    wire do_pack_emit  = (gen_state == GEN_EMIT)  && pack_can_emit;
    wire do_flush_emit = (gen_state == GEN_FLUSH);

    wire task_group_wr_en = (do_pack_emit || do_flush_emit) && !task_fifo_full;

    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data =
        do_flush_emit
        ? {64'd0, flush_sg2, flush_sg1, flush_sg0, flush_lane_valid}
        : {pack_sg3, pack_sg2, pack_sg1, pack_sg0, 4'b1111};

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
            carry_cnt <= 2'd0;
            carry_av[0] <= 0; carry_av[1] <= 0; carry_av[2] <= 0;
            carry_bv[0] <= 0; carry_bv[1] <= 0; carry_bv[2] <= 0;
            carry_bc[0] <= 0; carry_bc[1] <= 0; carry_bc[2] <= 0;
        end else begin
            case (gen_state)

                GEN_IDLE: begin
                    if (state == PE_CLEAR_ACC) begin
                        gen_t     <= 0;
                        gen_g     <= 0;
                        carry_cnt <= 2'd0;  // reset carry at start of each A row
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
                            if (carry_cnt != 2'd0)
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
                        // Emit case: carry_cnt + gen_cnt >= 4 → write one group
                        // -------------------------------------------------------
                        if (!task_fifo_full) begin
                            // Update carry with overflow elements
                            // (new carry slots = ne[4-carry_cnt .. gen_cnt-1])
                            carry_cnt <= pack_total[1:0]; // = carry_cnt+gen_cnt-4

                            // Carry slot 0 (overflow_cnt >= 1, i.e., pack_total >= 5)
                            if (pack_total >= 3'd5) begin
                                carry_av[0] <= gen_a_val;
                                carry_bv[0] <= (carry_cnt == 2'd1) ? ne_bv[3] :
                                               (carry_cnt == 2'd2) ? ne_bv[2] : ne_bv[1];
                                carry_bc[0] <= (carry_cnt == 2'd1) ? ne_bc[3] :
                                               (carry_cnt == 2'd2) ? ne_bc[2] : ne_bc[1];
                            end
                            // Carry slot 1 (overflow_cnt >= 2, i.e., pack_total >= 6)
                            if (pack_total >= 3'd6) begin
                                carry_av[1] <= gen_a_val;
                                carry_bv[1] <= (carry_cnt == 2'd2) ? ne_bv[3] : ne_bv[2];
                                carry_bc[1] <= (carry_cnt == 2'd2) ? ne_bc[3] : ne_bc[2];
                            end
                            // Carry slot 2 (overflow_cnt == 3, pack_total == 7)
                            if (pack_total == 3'd7) begin
                                carry_av[2] <= gen_a_val;
                                carry_bv[2] <= ne_bv[3];
                                carry_bc[2] <= ne_bc[3];
                            end

                            // State / counter transitions
                            if (gen_last_grp) begin
                                if (gen_t >= cur_a_nnz) begin
                                    // All A nnzs consumed
                                    if (pack_total[1:0] != 2'b0)
                                        gen_state <= GEN_FLUSH;
                                    else
                                        gen_state <= GEN_ROW_DONE;
                                end else if (fetch_b_nnz == 16'd0) begin
                                    // Next A nnz has empty B row — skip
                                    gen_t <= gen_t + 16'd1;
                                    gen_g <= 0;
                                    if (gen_t + 16'd1 >= cur_a_nnz) begin
                                        if (pack_total[1:0] != 2'b0)
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
                        // Accumulate case: carry_cnt + gen_cnt < 4.
                        // Only possible at last group (gen_cnt < 4).
                        // No FIFO write, always proceed.
                        // -------------------------------------------------------
                        carry_cnt <= pack_total[1:0]; // carry_cnt + gen_cnt (< 4)

                        // Append ne[0..gen_cnt-1] to carry starting at carry[carry_cnt]
                        case (carry_cnt)
                            2'd0: begin
                                carry_av[0] <= gen_a_val;
                                carry_bv[0] <= ne_bv[0];
                                carry_bc[0] <= ne_bc[0];
                                if (gen_cnt >= 4'd2) begin
                                    carry_av[1] <= gen_a_val;
                                    carry_bv[1] <= ne_bv[1];
                                    carry_bc[1] <= ne_bc[1];
                                end
                                if (gen_cnt >= 4'd3) begin
                                    carry_av[2] <= gen_a_val;
                                    carry_bv[2] <= ne_bv[2];
                                    carry_bc[2] <= ne_bc[2];
                                end
                            end
                            2'd1: begin
                                carry_av[1] <= gen_a_val;
                                carry_bv[1] <= ne_bv[0];
                                carry_bc[1] <= ne_bc[0];
                                if (gen_cnt >= 4'd2) begin
                                    carry_av[2] <= gen_a_val;
                                    carry_bv[2] <= ne_bv[1];
                                    carry_bc[2] <= ne_bc[1];
                                end
                            end
                            2'd2: begin
                                carry_av[2] <= gen_a_val;
                                carry_bv[2] <= ne_bv[0];
                                carry_bc[2] <= ne_bc[0];
                            end
                            default: ; // carry_cnt=3: impossible when pack_total < 4
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
                        carry_cnt <= 2'd0;
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
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            task_fifo_rd_en_d1   <= 1'b0;
            task_fifo_rd_data_d1 <= 0;
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
            mac_lane_valid_r <= task_fifo_rd_data_d1[3:0];
            mac_lane_task_r[0*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[67:4];
            mac_lane_task_r[1*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[131:68];
            mac_lane_task_r[2*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[195:132];
            mac_lane_task_r[3*`TASK_WIDTH +: `TASK_WIDTH] <= task_fifo_rd_data_d1[259:196];
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
    // Product Group FIFO
    //=========================================================================
    wire product_group_wr_en;
    wire [`PRODUCT_GROUP_WIDTH-1:0] product_group_wr_data;
    wire product_fifo_full;
    wire [`PROD_FIFO_DEPTH_LOG:0] product_fifo_cnt;

    assign product_group_wr_en             = |mul_valid && !product_fifo_full;
    assign product_group_wr_data[3:0]      = mul_valid;
    assign product_group_wr_data[35:4]     = mul_product[0*32 +: 32];
    assign product_group_wr_data[67:36]    = mul_product[1*32 +: 32];
    assign product_group_wr_data[99:68]    = mul_product[2*32 +: 32];
    assign product_group_wr_data[131:100]  = mul_product[3*32 +: 32];

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

    wire [3:0]             drain_valid_0,  drain_valid_1;
    wire [BANK_ADDR_W-1:0] drain_gaddr_0,  drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] drain_row_id_0, drain_row_id_1;
    wire [4*16-1:0]        drain_values_0, drain_values_1;

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

    // 1-cycle pipeline for synchronous (BRAM) FIFO read
    reg                          prod_fifo_rd_en_d1;
    reg [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data_d1;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            prod_fifo_rd_en_d1   <= 1'b0;
            prod_fifo_rd_data_d1 <= 0;
        end else begin
            prod_fifo_rd_en_d1   <= prod_fifo_rd_en && !prod_fifo_empty;
            prod_fifo_rd_data_d1 <= prod_fifo_rd_data;
        end
    end

    wire [3:0]  acc_lane_valid;
    wire [35:0] acc_lane_col_id;
    wire [63:0] acc_lane_product;

    assign acc_lane_valid   = prod_fifo_rd_data_d1[3:0];
    // product format: {col_id[15:0], fp16_val[15:0]} per lane (32-bit each)
    assign acc_lane_col_id  = {prod_fifo_rd_data_d1[4+3*32+16 +: 9],
                               prod_fifo_rd_data_d1[4+2*32+16 +: 9],
                               prod_fifo_rd_data_d1[4+1*32+16 +: 9],
                               prod_fifo_rd_data_d1[4+0*32+16 +: 9]};
    assign acc_lane_product = {prod_fifo_rd_data_d1[4+3*32 +: 16],
                               prod_fifo_rd_data_d1[4+2*32 +: 16],
                               prod_fifo_rd_data_d1[4+1*32 +: 16],
                               prod_fifo_rd_data_d1[4+0*32 +: 16]};

    row_accumulator_4bank #(
        .OUT_COLS(512), .COL_W(9), .PROD_W(16),
        .ACC_W(16), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5),
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
        .OUT_COLS(512), .COL_W(9), .PROD_W(16),
        .ACC_W(16), .EPOCH_W(16), .BANK_FIFO_DEPTH(32), .BANK_FIFO_LOG(5),
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
                    cur_a_off <= A_row_desc_buf[row_idx][63:32];
                    cur_a_nnz <= A_row_desc_buf[row_idx][31:16];
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
                state_next = PE_CLEAR_ACC;

            PE_CLEAR_ACC:
                state_next = PE_STREAM_INSTRS;

            PE_STREAM_INSTRS:
                if (gen_state == GEN_ROW_DONE) state_next = PE_WAIT_TASK_DRAIN;

            PE_WAIT_TASK_DRAIN:
                if (task_fifo_empty) state_next = PE_WAIT_PRODUCT_DRAIN;

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
