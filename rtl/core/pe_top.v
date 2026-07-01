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

    // Operation mode: 0 = SpGEMM (C=A*B); 1 = elementwise (C=A op B).
    //   op_sub: in elementwise mode 0 = add (A+B), 1 = subtract (A-B).
    // In elementwise mode each input element is streamed straight into the row
    // accumulator (scatter-add by column); the MAC passes it through x(+/-1.0).
    input  wire                          op_mode,
    input  wire                          op_sub,

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
    input  wire [`C_ROW_ADDR_BITS + $clog2(`C_BANK_COLS/`N_MAC) - 1:0]  c_rd_addr,
    output reg  [`N_MAC*16-1:0]              c_rd_data,
    output reg  [`MAX_DIM_BITS-1:0]      c_rd_row
);

    //=========================================================================
    // SRAM declarations
    //=========================================================================
    // A val/col buffers, 16-BANKED by nnz-offset%16 (BRAM, sync read).  Banking lets
    // the elementwise path read 16 consecutive A elements/cycle (16x its old 1/cycle);
    // the SpGEMM generator reads one element/cycle by muxing the bank for its offset.
    // Same total depth as the old single buffers (A_NNZ_SLOT_PER_PE), 1/16 per bank.
    localparam A_BANK_DEPTH = `A_NNZ_SLOT_PER_PE / `N_MAC;
    localparam A_BADDR_W    = `A_NNZ_ADDR_BITS - `N_MAC_BITS;
    localparam GADDR_W      = $clog2(`C_BANK_COLS / `N_MAC);
    // A_val/A_col banks are declared per-bank in the gen_a_bank generate (end of module).

    localparam B_BANK_DEPTH = `B_NNZ_SLOT / `N_MAC;
    localparam B_DESC_DEPTH = `B_ROW_SLOT;

    // All B reads are now synchronous (executor/generator/elem), so force Block
    // RAM — these 32 arrays (~4928x16 each) were the dominant LUTRAM consumer.
    // B_col/B_val banks are declared per-bank in the gen_b_bank generate (end of module).

    reg [31:0] B_desc_buf [0:B_DESC_DEPTH-1];

    //=========================================================================
    // SRAM write ports
    //=========================================================================
    always @(posedge aclk) begin
        if (b_desc_we) B_desc_buf[b_desc_waddr] <= b_desc_wdata;
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
    reg gen_done_acc_0, gen_done_acc_1;     // generation finished for acc k's current row
    reg [`A_ROW_ADDR_BITS-1:0] row_idx;     // local row index (dense, per PE)
    reg [`MAX_DIM_BITS-1:0]    cur_c_row;    // global C row (elementwise B index)
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
    // Registered (BRAM) A read shared by SpGEMM and elem (driven below, after the
    // elem address is defined).  fetch_a_val/fetch_k_idx are now the data for the
    // address that was presented LAST cycle, so the generator FSM issues the
    // address one cycle early (GEN_AADDR / the prior GEN_EMIT) — see the FSM.
    wire [15:0] a_val_r, a_col_r;   // muxed single A element (== A[a_rd_addr]); see read block
    wire [15:0] fetch_a_val  = a_val_r;
    wire [15:0] fetch_k_idx  = a_col_r;
    wire [31:0] fetch_b_desc = B_desc_buf[fetch_k_idx[`B_ROW_ADDR_BITS-1:0]];
    wire [31:0] fetch_b_off  = {15'b0, fetch_b_desc[26:10]};
    wire [15:0] fetch_b_nnz  = {6'b0,  fetch_b_desc[9:0]};

    //=========================================================================
    // Generator: aligned groups (→ ptr_fifo) and remainder (→ Gen2)
    //=========================================================================
    wire [15:0] gen_num_groups = gen_b_nnz >> `N_MAC_BITS;
    wire [`N_MAC_BITS-1:0] gen_remainder = gen_b_nnz[`N_MAC_BITS-1:0];

    wire [31:0] gen_abs_base = gen_b_off + (gen_num_groups << `N_MAC_BITS);
    wire [`N_MAC_BITS-1:0] gen_r = gen_abs_base[`N_MAC_BITS-1:0];
    wire [13:0] gen_m        = gen_abs_base[`N_MAC_BITS+13:`N_MAC_BITS];

    // (aligned-group bank addresses gen_bg* were dead code; removed)

    // PREFETCH B-read address — computed from the prefetch descriptor (fetch_*,
    // i.e. the NEXT A-nnz) so the registered read lands exactly when that A-nnz
    // becomes current.  This pipelines the generator (1 A-nnz/cycle) without the
    // 2-phase stall; the rotation below still uses the CURRENT gen_r, which by
    // construction equals this cycle's gen_r_pf delayed one step.
    wire [15:0] fetch_num_groups = fetch_b_nnz >> `N_MAC_BITS;
    wire [31:0] gen_abs_base_pf  = fetch_b_off + (fetch_num_groups << `N_MAC_BITS);
    wire [`N_MAC_BITS-1:0] gen_r_pf = gen_abs_base_pf[`N_MAC_BITS-1:0];
    wire [13:0] gen_m_pf         = gen_abs_base_pf[`N_MAC_BITS+13:`N_MAC_BITS];

    wire [13:0] gen_bg_pf [0:`N_MAC-1];
    genvar gpf;
    generate for (gpf=0; gpf<`N_MAC; gpf=gpf+1) begin: g_gen_bg_pf
        assign gen_bg_pf[gpf] = (gen_r_pf <= gpf) ? gen_m_pf : gen_m_pf + 14'd1;
    end endgenerate

    // Registered (synchronous) B reads -> infer Block RAM.  Issued from the
    // PREFETCH address (next A-nnz) and clock-enabled only when the generator
    // advances (gen_b_read_en), so bcN/bvN hold the CURRENT A-nnz's data while
    // stalled and update exactly when the next A-nnz is loaded into gen_*.
    reg [`COL_ID_W-1:0] bc  [0:`N_MAC-1];
    reg [15:0] bv  [0:`N_MAC-1];
    reg [`COL_ID_W-1:0] ebc [0:`N_MAC-1];
    reg [15:0] ebv [0:`N_MAC-1];
    wire gen_b_read_en;   // driven below; consumed by gen_b_bank RW port (end of module)

    wire [15:0] ne_bv [0:`N_MAC-1]; wire [`COL_ID_W-1:0] ne_bc [0:`N_MAC-1];
    genvar nr;
    generate for (nr=0; nr<`N_MAC; nr=nr+1) begin: g_ne_rot
        assign ne_bv[nr] = bv[(gen_r + nr) & (`N_MAC-1)];
        assign ne_bc[nr] = bc[(gen_r + nr) & (`N_MAC-1)];
    end endgenerate

    wire [`TASK_WIDTH-1:0] pack_sg [0:`N_MAC-1];
    genvar ps;
    generate for (ps=0; ps<`N_MAC; ps=ps+1) begin: g_pack_sg
        assign pack_sg[ps] = {ne_bv[ps], gen_a_val, ne_bc[ps][`COL_ID_W-1:0]};
    end endgenerate

    //=========================================================================
    // Gen2: accumulate cross-A-nnz remainders (up to 15 carry elements)
    //=========================================================================
    reg [`N_MAC_BITS-1:0] carry2_cnt;
    reg [`TASK_WIDTH-1:0] carry2_task [0:`N_MAC-1];

    wire [`N_MAC_BITS:0] g2_combined = carry2_cnt + gen_remainder;
    wire       g2_can_emit = g2_combined[`N_MAC_BITS];
    wire [`N_MAC_BITS-1:0] g2_overflow = g2_combined[`N_MAC_BITS-1:0];

    // g2_sg[j] = carry2_task[j] if j < carry2_cnt else pack_sg[j - carry2_cnt]
    wire [`TASK_WIDTH-1:0] g2_sg [0:`N_MAC-1];
    genvar gs;
    generate for (gs=0; gs<`N_MAC; gs=gs+1) begin: g_g2_sg
        assign g2_sg[gs] = (carry2_cnt > gs) ? carry2_task[gs]
                                             : pack_sg[(gs - carry2_cnt) & (`N_MAC-1)];
    end endgenerate

    wire [`N_MAC-1:0] g2_flush_lane_valid = ((1 << carry2_cnt) - 1);

    wire task_fifo_full;
    wire ptr_fifo_full;

    wire g1_to_g2_valid = (gen_state == GEN_EMIT) && (gen_remainder != 4'd0);
    wire g2_want_emit   = g1_to_g2_valid && g2_can_emit;
    wire g2_want_flush  = (gen_state == GEN_ROW_DONE) && (carry2_cnt != 4'd0);

    wire gen_emit_stall =
        (gen_num_groups != 16'd0 && ptr_fifo_full) ||
        (gen_remainder  != 4'd0  && g2_can_emit && task_fifo_full);
    wire gen_emit_can_advance = (gen_state == GEN_EMIT) && !gen_emit_stall;

    // Clock-enable for the prefetched B reads: pull the next A-nnz's B data in
    // whenever the generator is about to load a new gen_b_off — i.e. every
    // GEN_FETCH (which loads gen_*) and every advancing GEN_EMIT cycle.  Held
    // off during a stall so bcN/bvN keep the current A-nnz's data.
    // B prefetch: read the NEXT A-nnz's B remainder whenever a new gen_b_off is
    // loaded (GEN_FETCH, and the chaining GEN_EMIT advance).  fetch_b_off is fed by
    // the registered A read (k_idx), which holds the next A-nnz via the gen_t_next
    // addressing, so bcN lands aligned with that A-nnz's GEN_EMIT.
    assign gen_b_read_en = (gen_state == GEN_FETCH) ||
                         (gen_state == GEN_EMIT && gen_emit_can_advance);
    wire g1_acc_advances      = gen_emit_can_advance && g1_to_g2_valid;

    wire ptr_fifo_wr_en =
        (gen_state == GEN_EMIT) && gen_emit_can_advance && (gen_num_groups != 16'd0);
    wire [`PTR_TASK_WIDTH-1:0] ptr_fifo_wr_data =
        {comp_sel, gen_a_val, gen_b_off[16:0], gen_num_groups[6:0]};   // MSB = target acc

    // ---- SpGEMM (Gen2) task-group source ----
    wire gen2_task_wr_en = (g2_want_emit || g2_want_flush) && !task_fifo_full;
    wire [`N_MAC*`TASK_WIDTH-1:0] g2_emit_lanes, g2_flush_lanes;
    genvar gg;
    generate for (gg=0; gg<`N_MAC; gg=gg+1) begin: g_g2_lanes
        assign g2_emit_lanes [gg*`TASK_WIDTH +: `TASK_WIDTH] = g2_sg[gg];
        assign g2_flush_lanes[gg*`TASK_WIDTH +: `TASK_WIDTH] =
            (gg==`N_MAC-1) ? carry2_task[0] : carry2_task[gg];
    end endgenerate
    wire [`TASK_GROUP_WIDTH-1:0] gen2_task_data =
        g2_want_flush ? {1'b0, g2_flush_lanes, g2_flush_lane_valid}
                      : {1'b0, g2_emit_lanes, {`N_MAC{1'b1}}};

    //=========================================================================
    // Elementwise generator (op_mode=1): stream A[row] then B[row] elements
    // straight into task_fifo as one-lane groups {value, +/-1.0, col}.  The MAC
    // passes value x(+/-1.0) through; the row accumulator scatter-adds by column
    // => C = A +/- B.  B's per-row descriptor is loaded into B_desc_buf[row_idx]
    // by the host (same {b_off[26:10], b_nnz[9:0]} layout as SpGEMM).
    //=========================================================================
    localparam ELEM_IDLE=2'd0, ELEM_A=2'd1, ELEM_B=2'd2, ELEM_DONE=2'd3;
    reg  [1:0]  elem_state;
    reg  [15:0] elem_j;        // elementwise window bank-address (16 elems/window)

    wire [31:0] elem_b_desc = B_desc_buf[cur_c_row];  // global row (B is broadcast)
    wire [16:0] elem_b_off  = elem_b_desc[26:10];
    wire [15:0] elem_b_nnz  = {6'b0, elem_b_desc[9:0]};

    // Elementwise reads A 16 ELEMENTS / cycle: elem_j is the WINDOW bank-address
    // (16 consecutive elements = the 16 banks at elem_j).  Window range = start..last.
    wire [12:0] a_win_start = cur_a_off[16:0] >> `N_MAC_BITS;
    wire [12:0] a_win_last  = (cur_a_off[16:0] + {1'b0,cur_a_nnz} - 17'd1) >> `N_MAC_BITS;
    wire [12:0] b_win_start = elem_b_off >> `N_MAC_BITS;
    wire [12:0] b_win_last  = (elem_b_off        + {1'b0,elem_b_nnz} - 17'd1) >> `N_MAC_BITS;
    // elem_j NEXT-state (mirrors the FSM).  Addressing the registered A/B reads with
    // it makes a_val_bank_r/ebcN hold the CURRENT window EVERY cycle -> 1 window/cycle,
    // no 2-phase (same trick as SpGEMM gen_t_next; survives task_fifo_full stalls).
    wire elem_start_w = (state==PE_CLEAR_ACC) && op_mode;
    wire [15:0] elem_j_next =
        (elem_state==ELEM_IDLE) ? (elem_start_w ? (cur_a_nnz!=0 ? {3'b0,a_win_start}
                                                                : {3'b0,b_win_start})
                                                : elem_j) :
        (elem_state==ELEM_A && !task_fifo_full) ?
            ((elem_j[12:0] >= a_win_last) ? {3'b0,b_win_start} : elem_j + 16'd1) :
        (elem_state==ELEM_B && !task_fifo_full && elem_j[12:0] < b_win_last) ? elem_j + 16'd1 :
        elem_j;
    // A read window address = NEXT window (so the registered read lands it in time).
    wire [`A_NNZ_ADDR_BITS-1:0] elem_a_addr = {elem_j_next[A_BADDR_W-1:0], {`N_MAC_BITS{1'b0}}};

    // 2-deep A prefetch: address the SpGEMM read with gen_t's NEXT-state value, so
    // a_val_r/a_col_r registered this cycle == A[gen_t] next cycle, holding the
    // invariant a_val_r==A[gen_t] every cycle (incl. across EMIT stalls).  That
    // lets the generator keep its 1-cyc/A-nnz chaining FSM with a BRAM (registered)
    // read — no GEN_AADDR fill, no throughput loss.  gen_t_next exactly mirrors the
    // FSM's gen_t update.  Elem keeps its own 2-phase (elem_a_addr, current elem_j).
    wire gen_start_w = (state == PE_CLEAR_ACC) && !op_mode;
    wire [15:0] gen_t_next =
        (gen_state==GEN_IDLE)  ? (gen_start_w ? 16'd0 : gen_t)        :
        (gen_state==GEN_FETCH) ? (gen_t + 16'd1)                      :
        (gen_state==GEN_EMIT && gen_emit_can_advance && (gen_t < cur_a_nnz))
                               ? (gen_t + 16'd1)                      :
        gen_t;

    // Single registered A read port, address muxed by op_mode (SpGEMM gen vs elem
    // are mutually exclusive).  ram_style="block" on A_val_buf/A_col_buf => Block
    // RAM (was ~29k LUT/PE of distributed RAM).
    wire [`A_NNZ_ADDR_BITS-1:0] a_rd_addr =
        op_mode ? elem_a_addr
                : (cur_a_off[`A_NNZ_ADDR_BITS-1:0] + gen_t_next[`A_NNZ_ADDR_BITS-1:0]);
    wire [A_BADDR_W-1:0] a_rd_baddr = a_rd_addr[`A_NNZ_ADDR_BITS-1:`N_MAC_BITS];
    wire [`N_MAC_BITS-1:0] a_rd_bsel = a_rd_addr[`N_MAC_BITS-1:0];
    reg  [`N_MAC_BITS-1:0] a_rd_bsel_d;
    // 16-bank registered read (one address, all banks).  Packed so the SpGEMM mux
    // is a simple part-select; the elem path (Step 2) consumes all 16 lanes.
    reg [`N_MAC*16-1:0] a_val_bank_r, a_col_bank_r;
    always @(posedge aclk) a_rd_bsel_d <= a_rd_bsel;
    // SpGEMM single-element read: mux the bank for a_rd_addr (1-cyc-delayed select).
    assign a_val_r = a_val_bank_r[a_rd_bsel_d*16 +: 16];
    assign a_col_r = a_col_bank_r[a_rd_bsel_d*16 +: 16];

    // 16-WIDE elementwise.  Window = the 16 banks at elem_j (offsets 16*elem_j..+15).
    // B reuses the executor's 16 registered ports (ebcN/ebvN, muxed to elem_b_addr by
    // op_mode); A uses the 16-bank read a_val_bank_r/a_col_bank_r.  No rotation: each
    // lane carries its own column and the row accumulator scatters by col%16.  A lane
    // is valid iff its global offset is inside the row [base, base+nnz) — masks the
    // unaligned A head/tail (B is 16-aligned).  Whole row -> one comp_sel (ping-pong
    // per row) via the existing task_group_wr_data tag + main FSM, like SpGEMM.
    wire [12:0] elem_b_addr = elem_j_next[12:0];            // B window bank addr (NEXT, like A)
    wire        elem_in_b   = (elem_state==ELEM_B);
    wire        elem_active = (elem_state==ELEM_A) || elem_in_b;
    wire [16:0] elem_win0   = {elem_j[12:0], {`N_MAC_BITS{1'b0}}};   // N_MAC*elem_j (window's first offset)
    wire [16:0] elem_base   = elem_in_b ? elem_b_off : cur_a_off[16:0];
    wire [16:0] elem_end    = elem_base + (elem_in_b ? {1'b0,elem_b_nnz} : {1'b0,cur_a_nnz});

    wire [`N_MAC*16-1:0] ebc_vec, ebv_vec;
    genvar ev;
    generate for (ev=0; ev<`N_MAC; ev=ev+1) begin: g_eb_vec
        assign ebc_vec[ev*16 +: 16] = ebc[ev];
        assign ebv_vec[ev*16 +: 16] = ebv[ev];
    end endgenerate

    wire [`N_MAC-1:0]          elem_lv;
    wire [`N_MAC*`TASK_WIDTH-1:0]  elem_tasks_flat;
    genvar ek;
    generate for (ek=0; ek<`N_MAC; ek=ek+1) begin : g_elem_lane
        wire [16:0] eabs  = elem_win0 + ek[16:0];
        assign elem_lv[ek] = (eabs >= elem_base) && (eabs < elem_end);
        wire [15:0] eval  = elem_in_b ? ebv_vec[ek*16+:16] : a_val_bank_r[ek*16+:16];
        wire [`COL_ID_W-1:0] ecol = elem_in_b ? ebc_vec[ek*16 +: `COL_ID_W] : a_col_bank_r[ek*16 +: `COL_ID_W];
        wire [15:0] ecoef = (elem_in_b && op_sub) ? 16'hBC00 : 16'h3C00;   // +/-1.0
        assign elem_tasks_flat[ek*`TASK_WIDTH +: `TASK_WIDTH] = {eval, ecoef, ecol};
    end endgenerate

    // Pipelined: emit one window/cycle (no 2-phase), the read holds the current window.
    wire elem_wr_en = elem_active && (|elem_lv) && !task_fifo_full;
    wire [`TASK_GROUP_WIDTH-1:0] elem_task_group = {1'b0, elem_tasks_flat, elem_lv};

    // Pipelined FSM: one window per cycle (the read holds the current window via the
    // elem_j_next addressing above; no ADDR/USE phase).  Stalls on task_fifo_full.
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin elem_state<=ELEM_IDLE; elem_j<=0; end
        else case (elem_state)
            ELEM_IDLE: if (state==PE_CLEAR_ACC && op_mode) begin
                if      (cur_a_nnz!=0) begin elem_j<={3'b0,a_win_start}; elem_state<=ELEM_A; end
                else if (elem_b_nnz!=0)begin elem_j<={3'b0,b_win_start}; elem_state<=ELEM_B; end
                else                        elem_state<=ELEM_DONE;
            end
            ELEM_A: if (!task_fifo_full) begin
                if (elem_j[12:0] >= a_win_last) begin           // last A window emitted
                    if (elem_b_nnz!=0) begin elem_j<={3'b0,b_win_start}; elem_state<=ELEM_B; end
                    else                    elem_state<=ELEM_DONE;
                end else elem_j <= elem_j + 16'd1;
            end
            ELEM_B: if (!task_fifo_full) begin
                if (elem_j[12:0] >= b_win_last) elem_state <= ELEM_DONE;
                else elem_j <= elem_j + 16'd1;
            end
            ELEM_DONE: if (state==PE_NEXT_ROW)
                elem_state <= ELEM_IDLE;
            default: elem_state <= ELEM_IDLE;
        endcase
    end

    // ---- task_fifo write source mux ----
    wire task_group_wr_en = op_mode ? elem_wr_en : gen2_task_wr_en;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_body = op_mode ? elem_task_group : gen2_task_data;
    wire [`TASK_GROUP_WIDTH-1:0] task_group_wr_data =
        {comp_sel, task_group_body[`TASK_GROUP_WIDTH-2:0]};   // MSB = target acc

    wire row_gen_done = op_mode ? (elem_state==ELEM_DONE) : (gen_state==GEN_ROW_DONE);

    //=========================================================================
    // Generator sub-FSM sequential
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            gen_state<=GEN_IDLE; gen_t<=0; gen_a_val<=0; gen_b_off<=0; gen_b_nnz<=0;
        end else case (gen_state)
            // A read is now registered (BRAM): GEN_AADDR presents A[gen_t]'s address,
            // GEN_FETCH consumes the registered result (a_val_r/a_col_r) and loads
            // gen_*.  An advancing GEN_EMIT already presents the NEXT A-nnz's address
            // (gen_t was incremented in FETCH), so EMIT->FETCH needs no extra AADDR;
            // only the first A-nnz and post-skip transitions go through GEN_AADDR.
            // A read is registered (BRAM), but a_rd_addr uses gen_t_next so the
            // invariant a_val_r/a_col_r == A[gen_t] holds every cycle (incl. across
            // stalls) — exactly like the old combinational A read.  So the original
            // 1-cyc/A-nnz CHAINING FSM is used unchanged (fetch_* = a_val_r/a_col_r).
            GEN_IDLE: begin
                if (state == PE_CLEAR_ACC && !op_mode) begin
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
                if (state==PE_NEXT_ROW)
                    gen_state<=GEN_IDLE;
            end
            default: gen_state<=GEN_IDLE;
        endcase
    end

    //=========================================================================
    // Gen2 sequential
    //=========================================================================
    integer ci2;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            carry2_cnt <= 0;
            for (ci2=0; ci2<`N_MAC; ci2=ci2+1) carry2_task[ci2] <= 0;
        end else begin
            if (g1_acc_advances) begin
                if (g2_can_emit) begin
                    carry2_cnt <= g2_overflow;
                    for (ci2=0; ci2<`N_MAC-1; ci2=ci2+1)
                        if (ci2 < g2_overflow)
                            carry2_task[ci2] <= pack_sg[(`N_MAC - carry2_cnt + ci2) & (`N_MAC-1)];
                end else begin
                    carry2_cnt <= g2_combined[`N_MAC_BITS-1:0];
                    for (ci2=0; ci2<`N_MAC-1; ci2=ci2+1)
                        if (ci2 < gen_remainder)
                            carry2_task[(carry2_cnt + ci2) & (`N_MAC-1)] <= pack_sg[ci2];
                end
            end else if (g2_want_flush && !task_fifo_full) begin
                carry2_cnt <= 0;
            end
        end
    end

    //=========================================================================
    // task_fifo (Gen2 output)
    //=========================================================================
    wire task_fifo_rd_en;
    wire [`TASK_GROUP_WIDTH-1:0] task_fifo_rd_data;
    wire task_fifo_empty;

    wire [`TASK_FIFO_DEPTH_LOG:0] task_fifo_cnt;
    sync_fifo #(.WIDTH(`TASK_GROUP_WIDTH),.DEPTH(`TASK_FIFO_DEPTH),.DEPTH_LOG(`TASK_FIFO_DEPTH_LOG))
    u_task_fifo (
        .wr_en(task_group_wr_en),.wr_data(task_group_wr_data),.wr_full(task_fifo_full),
        .rd_en(task_fifo_rd_en),.rd_data(task_fifo_rd_data),.rd_empty(task_fifo_empty),
        .count(task_fifo_cnt),.aclk(aclk),.aresetn(aresetn)
    );

    //=========================================================================
    // ptr_fifo (pointer tasks)
    //=========================================================================
    wire [`PTR_TASK_WIDTH-1:0] ptr_fifo_rd_data;
    wire ptr_fifo_empty;
    wire ptr_fifo_rd_en;

    wire [`PTR_FIFO_DEPTH_LOG:0] ptr_fifo_cnt;
    sync_fifo #(.WIDTH(`PTR_TASK_WIDTH),.DEPTH(`PTR_FIFO_DEPTH),.DEPTH_LOG(`PTR_FIFO_DEPTH_LOG))
    u_ptr_fifo (
        .wr_en(ptr_fifo_wr_en),.wr_data(ptr_fifo_wr_data),.wr_full(ptr_fifo_full),
        .rd_en(ptr_fifo_rd_en),.rd_data(ptr_fifo_rd_data),.rd_empty(ptr_fifo_empty),
        .count(ptr_fifo_cnt),.aclk(aclk),.aresetn(aresetn)
    );

    //=========================================================================
    // Product FIFOs — declared early so exec_prod_safe can reference count
    //=========================================================================
    wire [`PROD_FIFO_DEPTH_LOG:0] product_fifo_cnt_0, product_fifo_cnt_1;

    // -2 (not -1): the executor now has one extra pipeline stage (registered
    // B read for BRAM inference) between the prod_safe check and the product,
    // so one more group is in flight before it reaches the product FIFO.
    // Per-comp_sel because exec and the task path can target DIFFERENT ping-pong
    // accumulators concurrently (row pipelining): each must gate on ITS OWN fifo.
    wire prod_safe_0 = product_fifo_cnt_0 < (`PROD_FIFO_DEPTH - `MUL_LAT - 2);
    wire prod_safe_1 = product_fifo_cnt_1 < (`PROD_FIFO_DEPTH - `MUL_LAT - 2);
    wire exec_prod_safe = exec_comp ? prod_safe_1 : prod_safe_0;

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
    reg        exec_comp;        // comp_sel (target acc) of the ptr entry being expanded

    wire exec_idle = (exec_state == EXEC_IDLE);
    wire exec_busy = !exec_idle;

    wire exec_ptr_last = (exec_state == EXEC_PTR) &&
                         exec_prod_safe &&
                         (exec_g + 7'd1 >= {1'b0, exec_num_groups});

    // Executor chains ptr entries back-to-back (no longer blocked on task_fifo);
    // the task path now interleaves into the executor's idle/gap cycles via a
    // unified MAC-feed arbiter (task_fifo_rd_en gated by !exec_issuing), so there
    // is no exec<->task handoff bubble and the executor never drains its pipeline.
    assign ptr_fifo_rd_en = (exec_idle     && !ptr_fifo_empty) ||
                            (exec_ptr_last && !ptr_fifo_empty);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            exec_state<=EXEC_IDLE; exec_a_val<=0; exec_b_off<=0; exec_num_groups<=0; exec_g<=0; exec_comp<=0;
        end else case (exec_state)
            EXEC_IDLE: begin
                if (!ptr_fifo_empty) begin
                    exec_a_val      <= ptr_fifo_rd_data[39:24];
                    exec_b_off      <= ptr_fifo_rd_data[23:7];
                    exec_num_groups <= ptr_fifo_rd_data[6:0];
                    exec_comp       <= ptr_fifo_rd_data[`PTR_TASK_WIDTH-1];
                    exec_g          <= 7'd0;
                    exec_state      <= EXEC_PTR;
                end
            end
            EXEC_PTR: begin
                if (exec_prod_safe) begin
                    exec_g <= exec_g + 7'd1;
                    if (exec_ptr_last) begin
                        if (!ptr_fifo_empty) begin
                            // chain directly to the next ptr entry (task path
                            // interleaves separately, so no need to drain it first)
                            exec_a_val      <= ptr_fifo_rd_data[39:24];
                            exec_b_off      <= ptr_fifo_rd_data[23:7];
                            exec_num_groups <= ptr_fifo_rd_data[6:0];
                            exec_comp       <= ptr_fifo_rd_data[`PTR_TASK_WIDTH-1];
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
    wire [31:0] exec_abs_base = {15'b0, exec_b_off} + ({25'b0, exec_g} << `N_MAC_BITS);
    wire [`N_MAC_BITS-1:0] exec_r_addr = exec_abs_base[`N_MAC_BITS-1:0];
    wire [13:0] exec_m        = exec_abs_base[`N_MAC_BITS+13:`N_MAC_BITS];

    wire [13:0] exec_bg [0:`N_MAC-1];
    genvar xbg;
    generate for (xbg=0; xbg<`N_MAC; xbg=xbg+1) begin: g_exec_bg
        assign exec_bg[xbg] = (exec_r_addr <= xbg) ? exec_m : exec_m + 14'd1;
    end endgenerate

    // Stage-2 control, delayed 1 cycle to align with the registered B reads:
    //   exec_r        = rotation amount for the data now arriving
    //   exec_a_val_d1 = A value for that group
    //   exec_valid_d1 = "a B-read group is landing this cycle" (write enable)
    reg [`N_MAC_BITS-1:0] exec_r;
    reg [15:0] exec_a_val_d1;
    reg        exec_valid_d1;
    reg        exec_comp_d1;     // exec_comp delayed to match exec_valid_d1
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            exec_r <= 4'd0; exec_a_val_d1 <= 16'd0; exec_valid_d1 <= 1'b0; exec_comp_d1 <= 1'b0;
        end else begin
            exec_r        <= exec_r_addr;
            exec_a_val_d1 <= exec_a_val;
            exec_comp_d1  <= exec_comp;
            exec_valid_d1 <= (exec_state == EXEC_PTR) && exec_prod_safe;
        end
    end

    // ebc/ebv (executor + elementwise B reads) are produced by the gen_b_bank generate (end of module).

    wire [15:0] enebv [0:`N_MAC-1]; wire [`COL_ID_W-1:0] enebc [0:`N_MAC-1];
    genvar xr;
    generate for (xr=0; xr<`N_MAC; xr=xr+1) begin: g_eneb_rot
        assign enebv[xr] = ebv[(exec_r + xr) & (`N_MAC-1)];
        assign enebc[xr] = ebc[(exec_r + xr) & (`N_MAC-1)];
    end endgenerate

    wire [`TASK_WIDTH-1:0] exec_sg [0:`N_MAC-1];
    genvar xs;
    generate for (xs=0; xs<`N_MAC; xs=xs+1) begin: g_exec_sg
        assign exec_sg[xs] = {enebv[xs], exec_a_val_d1, enebc[xs][`COL_ID_W-1:0]};
    end endgenerate

    //=========================================================================
    // MAC array input: executor (ptr_fifo path) or Gen2 (task_fifo path)
    //=========================================================================
    // Unified MAC-feed arbiter.  Both paths reach the MAC with the SAME 2-cycle
    // latency, and the MAC takes one group/cycle, so they collide only if both
    // ISSUE in the same cycle.  exec issues when (EXEC_PTR && prod_safe); the task
    // path issues in every OTHER cycle (!exec_issuing) when it has work — this
    // packs Gen2 groups into the executor's gaps (ptr_fifo-empty / row boundaries)
    // with zero handoff bubble, instead of forcing the executor idle first.
    // task_prod_safe gates on the TASK head's own ping-pong fifo (it may target a
    // different accumulator than the executor's current comp).
    wire exec_issuing  = (exec_state == EXEC_PTR) && exec_prod_safe;
    wire task_head_comp = task_fifo_rd_data[`TASK_GROUP_WIDTH-1];
    wire task_prod_safe = task_head_comp ? prod_safe_1 : prod_safe_0;
    assign task_fifo_rd_en = !exec_issuing && !task_fifo_empty && task_prod_safe;

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
    reg                          mac_comp_sel_r;   // target acc of the current MAC group
    integer mi;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            mac_lane_valid_r<=0; mac_lane_task_r<=0; mac_comp_sel_r<=0;
        end else if (exec_valid_d1) begin
            mac_lane_valid_r <= {`N_MAC{1'b1}};
            mac_comp_sel_r   <= exec_comp_d1;
            for (mi=0; mi<`N_MAC; mi=mi+1)
                mac_lane_task_r[mi*`TASK_WIDTH +: `TASK_WIDTH] <= exec_sg[mi];
        end else if (task_fifo_rd_en_d1) begin
            mac_lane_valid_r <= task_fifo_rd_data_d1[`N_MAC-1:0];
            mac_comp_sel_r   <= task_fifo_rd_data_d1[`TASK_GROUP_WIDTH-1];
            for (mi=0; mi<`N_MAC; mi=mi+1)
                mac_lane_task_r[mi*`TASK_WIDTH +: `TASK_WIDTH] <=
                    task_fifo_rd_data_d1[`N_MAC + mi*`TASK_WIDTH +: `TASK_WIDTH];
        end else begin
            mac_lane_valid_r<=0;
        end
    end

    wire [`N_MAC-1:0]             mac_lane_valid = mac_lane_valid_r;
    wire [`N_MAC*`TASK_WIDTH-1:0] mac_lane_task  = mac_lane_task_r;

    // The multiplier array delays valid/col/val by MUL_LAT cycles, so the PRODUCT
    // for a group emerges MUL_LAT cycles after its comp_sel was at mac_comp_sel_r.
    // Route the product by this DELAYED comp_sel — otherwise, when two rows' groups
    // are back-to-back (row pipelining removed the drain gap), a row's product can
    // be steered into the next row's accumulator by the already-advanced comp_sel.
    // (MUL_LAT==1: a single delay stage.)
    reg mac_comp_sel_route;
    always @(posedge aclk or negedge aresetn)
        if (!aresetn) mac_comp_sel_route <= 1'b0;
        else          mac_comp_sel_route <= mac_comp_sel_r;

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

    wire product_fifo_full   = mac_comp_sel_route ? product_fifo_full_1 : product_fifo_full_0;
    wire product_group_wr_en = |mul_valid && !product_fifo_full;

    assign product_group_wr_data[`N_MAC-1:0]=mul_valid;
    genvar pg;
    generate for (pg=0; pg<`N_MAC; pg=pg+1) begin: g_prod_pack
        assign product_group_wr_data[`N_MAC + pg*`PRODUCT_WIDTH +: `PRODUCT_WIDTH]
             = mul_product[pg*`PRODUCT_WIDTH +: `PRODUCT_WIDTH];
    end endgenerate

    wire prod_fifo_rd_en_0,prod_fifo_rd_en_1;
    wire [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data_0,prod_fifo_rd_data_1;
    wire prod_fifo_empty_0,prod_fifo_empty_1;

    sync_fifo #(.WIDTH(`PRODUCT_GROUP_WIDTH),.DEPTH(`PROD_FIFO_DEPTH),.DEPTH_LOG(`PROD_FIFO_DEPTH_LOG))
    u_product_fifo_0 (
        .wr_en(product_group_wr_en&&!mac_comp_sel_route),.wr_data(product_group_wr_data),
        .wr_full(product_fifo_full_0),.rd_en(prod_fifo_rd_en_0),
        .rd_data(prod_fifo_rd_data_0),.rd_empty(prod_fifo_empty_0),
        .count(product_fifo_cnt_0),.aclk(aclk),.aresetn(aresetn)
    );

    sync_fifo #(.WIDTH(`PRODUCT_GROUP_WIDTH),.DEPTH(`PROD_FIFO_DEPTH),.DEPTH_LOG(`PROD_FIFO_DEPTH_LOG))
    u_product_fifo_1 (
        .wr_en(product_group_wr_en&&mac_comp_sel_route),.wr_data(product_group_wr_data),
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
    wire [`N_MAC-1:0] drain_valid_0,drain_valid_1;
    wire [GADDR_W-1:0]  drain_gaddr_0,drain_gaddr_1;
    wire [`A_ROW_ADDR_BITS-1:0] drain_row_id_0,drain_row_id_1;
    wire [`N_MAC*16-1:0] drain_values_0,drain_values_1;
    wire drain_active_0,drain_active_1;

    wire other_acc_busy = comp_sel ? acc_busy_0 : acc_busy_1;

    assign a_desc_ready = (state == PE_LOAD_ROW_DESC);

    //=========================================================================
    // Row-level pipelining: per-comp_sel "issue done" detection.
    //   Rows are pipelined across the two ping-pong accumulators.  acc k's row is
    //   fully ISSUED to it when the generator finished that row (gen_done_acc_k,
    //   set at PE_NEXT_ROW) AND no comp_sel=k task remains ANYWHERE upstream:
    //   ptr_fifo, task_fifo (covers the Gen2 path), the executor, the 2 mac-feed
    //   pipeline stages, and the MAC itself.  The exec arbitration (a new ptr is
    //   loaded only when task_fifo is empty) keeps the MAC comp_sel monotonic, so
    //   the per-comp_sel FIFO counters are exact.
    //=========================================================================
    reg [`PTR_FIFO_DEPTH_LOG:0]  ptr_cnt_0,  ptr_cnt_1;
    reg [`TASK_FIFO_DEPTH_LOG:0] task_cnt_0, task_cnt_1;
    wire ptr_rd_comp  = ptr_fifo_rd_data[`PTR_TASK_WIDTH-1];
    wire task_rd_comp = task_fifo_rd_data[`TASK_GROUP_WIDTH-1];
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            ptr_cnt_0<=0; ptr_cnt_1<=0; task_cnt_0<=0; task_cnt_1<=0;
        end else begin
            ptr_cnt_0  <= ptr_cnt_0  + (ptr_fifo_wr_en   && !comp_sel) - (ptr_fifo_rd_en  && !ptr_rd_comp);
            ptr_cnt_1  <= ptr_cnt_1  + (ptr_fifo_wr_en   &&  comp_sel) - (ptr_fifo_rd_en  &&  ptr_rd_comp);
            task_cnt_0 <= task_cnt_0 + (task_group_wr_en && !comp_sel) - (task_fifo_rd_en && !task_rd_comp);
            task_cnt_1 <= task_cnt_1 + (task_group_wr_en &&  comp_sel) - (task_fifo_rd_en &&  task_rd_comp);
        end
    end

    // comp_sel=k items in the executor / the 2 mac-feed stages / the MAC.
    wire k0_exec = exec_busy && !exec_comp;
    wire k1_exec = exec_busy &&  exec_comp;
    wire k0_ev   = exec_valid_d1 && !exec_comp_d1;
    wire k1_ev   = exec_valid_d1 &&  exec_comp_d1;
    wire k0_tk   = task_fifo_rd_en_d1 && !task_fifo_rd_data_d1[`TASK_GROUP_WIDTH-1];
    wire k1_tk   = task_fifo_rd_en_d1 &&  task_fifo_rd_data_d1[`TASK_GROUP_WIDTH-1];
    wire k0_mac  = (|mac_lane_valid_r) && !mac_comp_sel_r;
    wire k1_mac  = (|mac_lane_valid_r) &&  mac_comp_sel_r;
    // product still inside the multiplier (emerges MUL_LAT later, routed by the
    // delayed comp_sel) — must also be drained before the row is "issue done".
    wire k0_mul  = (|mul_valid) && !mac_comp_sel_route;
    wire k1_mul  = (|mul_valid) &&  mac_comp_sel_route;

    wire gen_issue_done_0 = gen_done_acc_0 && (ptr_cnt_0==0) && (task_cnt_0==0)
                            && !k0_exec && !k0_ev && !k0_tk && !k0_mac && !k0_mul;
    wire gen_issue_done_1 = gen_done_acc_1 && (ptr_cnt_1==0) && (task_cnt_1==0)
                            && !k1_exec && !k1_ev && !k1_tk && !k1_mac && !k1_mul;

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

    // Row fully drained into acc k = issuing done AND its product FIFO/hold empty.
    wire acc_inp_done_0 = gen_issue_done_0 && prod_fifo_empty_0
                          && !prd_hold_0 && !prd_rd_d1_0;
    wire acc_inp_done_1 = gen_issue_done_1 && prod_fifo_empty_1
                          && !prd_hold_1 && !prd_rd_d1_1;

    // Extract 16 lane_valid, 16 col_ids, 16 products from effective data
    wire [`N_MAC-1:0]    alv0 = eff_dat_0[`N_MAC-1:0];
    wire [`N_MAC-1:0]    alv1 = eff_dat_1[`N_MAC-1:0];
    wire [`N_MAC*`COL_ID_W-1:0]  alc0, alc1;
    wire [`N_MAC*16-1:0] alp0, alp1;
    genvar al;
    generate for (al=0; al<`N_MAC; al=al+1) begin: g_acc_lane
        assign alc0[al*`COL_ID_W +: `COL_ID_W] = eff_dat_0[`N_MAC + al*`PRODUCT_WIDTH + 16 +: `COL_ID_W];
        assign alp0[al*16+:16] = eff_dat_0[`N_MAC + al*`PRODUCT_WIDTH +: 16];
        assign alc1[al*`COL_ID_W +: `COL_ID_W] = eff_dat_1[`N_MAC + al*`PRODUCT_WIDTH + 16 +: `COL_ID_W];
        assign alp1[al*16+:16] = eff_dat_1[`N_MAC + al*`PRODUCT_WIDTH +: 16];
    end endgenerate

    // Per-bank scatter FIFO depth is the `BANK_FIFO_DEPTH knob (LUT vs throughput;
    // see defines.vh).  LOG is derived so only the one define needs setting.
    // Accumulator bank-count follows N_MAC (8 or 16): row_accumulator_<N_MAC>bank.
    // Same block name (g_acc) in both arms keeps the u_row_acc_* hierarchy stable.
    `define ACC_PARAMS .OUT_COLS(`C_BANK_COLS),.COL_W(`COL_ID_W),.PROD_W(16),.ACC_W(16),.EPOCH_W(16), \
        .BANK_FIFO_DEPTH(`BANK_FIFO_DEPTH),.BANK_FIFO_LOG($clog2(`BANK_FIFO_DEPTH)),.ROW_W(`A_ROW_ADDR_BITS)
    `define ACC0_PORTS \
        .clk(aclk),.rst_n(aresetn), \
        .row_start((state==PE_CLEAR_ACC)&&!comp_sel),.row_id_in(row_idx),.drain_cols(N), \
        .row_input_done(acc_inp_done_0),.busy(acc_busy_0),.row_done(acc_row_done_0), \
        .issue_valid(eff_valid_0),.issue_ready(acc_issue_ready_0), \
        .lane_valid(alv0),.lane_col_id(alc0),.lane_product(alp0), \
        .drain_valid(drain_valid_0),.drain_gaddr(drain_gaddr_0), \
        .drain_row_id(drain_row_id_0),.drain_values(drain_values_0), \
        .drain_ready(1'b1),.drain_active(drain_active_0)
    `define ACC1_PORTS \
        .clk(aclk),.rst_n(aresetn), \
        .row_start((state==PE_CLEAR_ACC)&&comp_sel),.row_id_in(row_idx),.drain_cols(N), \
        .row_input_done(acc_inp_done_1),.busy(acc_busy_1),.row_done(acc_row_done_1), \
        .issue_valid(eff_valid_1),.issue_ready(acc_issue_ready_1), \
        .lane_valid(alv1),.lane_col_id(alc1),.lane_product(alp1), \
        .drain_valid(drain_valid_1),.drain_gaddr(drain_gaddr_1), \
        .drain_row_id(drain_row_id_1),.drain_values(drain_values_1), \
        .drain_ready(1'b1),.drain_active(drain_active_1)
    // `ifdef (not generate) so u_row_acc_* stay directly under pe_top (stable
    // hierarchy for testbench probes).  ACC_8BANK is set in defines.vh when N_MAC==8.
`ifdef ACC_8BANK
    row_accumulator_8bank  #(`ACC_PARAMS) u_row_acc_0 (`ACC0_PORTS);
    row_accumulator_8bank  #(`ACC_PARAMS) u_row_acc_1 (`ACC1_PORTS);
`else
    row_accumulator_16bank #(`ACC_PARAMS) u_row_acc_0 (`ACC0_PORTS);
    row_accumulator_16bank #(`ACC_PARAMS) u_row_acc_1 (`ACC1_PORTS);
`endif

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
    localparam C_BANK_ADDR_W = `C_ROW_ADDR_BITS + GADDR_W;     // local_row + gaddr
    localparam C_BANK_DEPTH  = 1 << C_BANK_ADDR_W;

    // C_bank is declared per-sub-bank INSIDE the generate loop below so each is
    // a separate variable (<= C_BANK_DEPTH*16 bits) — a single 16x deep 2D array
    // exceeds Vivado's 1 Mbit per-variable behavioral-memory limit (Synth 8-4556).
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
    wire [GADDR_W-1:0]          c_wr_gaddr = c_wr_sel0 ? drain_gaddr_0  : drain_gaddr_1;
    wire [`N_MAC-1:0]           c_wr_dv    = c_wr_sel0 ? drain_valid_0  : drain_valid_1;
    wire [`N_MAC*16-1:0]        c_wr_dat   = c_wr_sel0 ? drain_values_0 : drain_values_1;
    wire [C_BANK_ADDR_W-1:0]    c_wr_addr  = {c_wr_row, c_wr_gaddr};

    // Registered map read (same address timing as the C bank data read).
    always @(posedge aclk) begin
        if (c_rd_en)
            c_rd_row <= C_row_map[c_rd_addr[C_BANK_ADDR_W-1:GADDR_W]];
    end

    genvar cb;
    generate
        for (cb = 0; cb < `N_MAC; cb = cb + 1) begin : gen_c_bank
            reg [15:0] mem [0:C_BANK_DEPTH-1];   // one sub-bank (own variable)
            always @(posedge aclk) begin
                if (c_wr_en)
                    mem[c_wr_addr] <= c_wr_dv[cb] ? c_wr_dat[cb*16 +: 16]
                                                  : 16'h0000;
            end
            always @(posedge aclk) begin
                if (c_rd_en)
                    c_rd_data[cb*16 +: 16] <= mem[c_rd_addr];
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
        if (!aresetn) begin comp_sel<=0; row_idx<=0; cur_c_row<=0; cur_a_off<=0; cur_a_nnz<=0; done<=0;
                            gen_done_acc_0<=0; gen_done_acc_1<=0; end
        else begin
            done<=0;
            case (state)
                PE_IDLE:          if (start) row_idx<=0;
                PE_LOAD_ROW_DESC: if (a_desc_valid) begin
                    cur_c_row<={{(`MAX_DIM_BITS-9){1'b0}}, a_desc_data[8:0]};
                    // a_off is 15 bits [33:19] (was 14 -> truncated above 16383,
                    // which a dense PE's offset exceeds: 153 nnz/row x >=108 rows).
                    // A_NNZ_SLOT_PER_PE=28672 needs 15 bits.
                    cur_a_off<={17'b0,a_desc_data[33:19]};
                    cur_a_nnz<={6'b0, a_desc_data[18:9]};
                end
                // New row starting on this acc -> its generation is not done yet.
                PE_CLEAR_ACC: if (!comp_sel) gen_done_acc_0<=0; else gen_done_acc_1<=0;
                // Row's generation finished (we only leave STREAM once it is) ->
                // mark this acc's row done so its row_input_done can fire as the
                // products drain, while we move on to the next row.
                PE_NEXT_ROW: begin
                    row_idx<=row_idx+1; comp_sel<=~comp_sel;
                    if (!comp_sel) gen_done_acc_0<=1; else gen_done_acc_1<=1;
                end
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
            // Row PIPELINING: advance as soon as this row's GENERATION is done
            // (incl. the Gen2 carry flush) and the NEXT row's accumulator is free.
            // We do NOT wait for this row's tasks/products to drain — they keep
            // flowing to their accumulator (routed by the per-task comp_sel) while
            // the next row generates.  The two WAIT_* states are now unreachable.
            PE_STREAM_INSTRS:      if (row_gen_done && !g2_want_flush && !other_acc_busy)
                                       state_next=PE_NEXT_ROW;
            PE_WAIT_TASK_DRAIN:    state_next=PE_NEXT_ROW;            // (dead) safety
            PE_WAIT_PRODUCT_DRAIN: state_next=PE_NEXT_ROW;            // (dead) safety
            PE_NEXT_ROW:           state_next=((row_idx+1)>=row_count)?PE_DONE:PE_LOAD_ROW_DESC;
            PE_DONE:               state_next=PE_DONE;
        endcase
    end

    //=========================================================================
    // A banks: per-bank BRAM (val 16b / col 9b), col%N_MAC banked.  Write demux
    // by load address; single registered read into packed a_*_bank_r.
    //=========================================================================
    genvar ab;
    generate for (ab=0; ab<`N_MAC; ab=ab+1) begin: gen_a_bank
        (* ram_style="block" *) reg [`DATA_WIDTH-1:0] vmem [0:A_BANK_DEPTH-1];
        (* ram_style="block" *) reg [8:0]             cmem [0:A_BANK_DEPTH-1];
        always @(posedge aclk) begin
            if (a_val_we && a_val_waddr[`N_MAC_BITS-1:0]==ab)
                vmem[a_val_waddr[`A_NNZ_ADDR_BITS-1:`N_MAC_BITS]] <= a_val_wdata;
            if (a_col_we && a_col_waddr[`N_MAC_BITS-1:0]==ab)
                cmem[a_col_waddr[`A_NNZ_ADDR_BITS-1:`N_MAC_BITS]] <= a_col_wdata;
            a_val_bank_r[ab*16 +: 16] <= vmem[a_rd_baddr];
            a_col_bank_r[ab*16 +: 16] <= cmem[a_rd_baddr];
        end
    end endgenerate

    //=========================================================================
    // B banks: per-bank BRAM.  Port A = write(load) OR gen-prefetch read(compute)
    // at ONE muxed address -> single RW BRAM port (they never overlap, no dup).
    // Port B = executor / elementwise read.
    //=========================================================================
    genvar bb;
    generate for (bb=0; bb<`N_MAC; bb=bb+1) begin: gen_b_bank
        (* ram_style="block" *) reg [`COL_ID_W-1:0]   cmem [0:B_BANK_DEPTH-1];
        (* ram_style="block" *) reg [`DATA_WIDTH-1:0] vmem [0:B_BANK_DEPTH-1];
        wire [13:0] colA = (b_col_we && b_col_waddr[`N_MAC_BITS-1:0]==bb)
                         ? b_col_waddr[`B_NNZ_ADDR_BITS-1:`N_MAC_BITS] : gen_bg_pf[bb];
        wire [13:0] valA = (b_val_we && b_val_waddr[`N_MAC_BITS-1:0]==bb)
                         ? b_val_waddr[`B_NNZ_ADDR_BITS-1:`N_MAC_BITS] : gen_bg_pf[bb];
        always @(posedge aclk) begin
            if (b_col_we && b_col_waddr[`N_MAC_BITS-1:0]==bb) cmem[colA] <= b_col_wdata;
            else if (gen_b_read_en) bc[bb] <= cmem[colA];
            if (b_val_we && b_val_waddr[`N_MAC_BITS-1:0]==bb) vmem[valA] <= b_val_wdata;
            else if (gen_b_read_en) bv[bb] <= vmem[valA];
        end
        always @(posedge aclk) begin
            ebc[bb] <= cmem[op_mode ? elem_b_addr : exec_bg[bb]];
            ebv[bb] <= vmem[op_mode ? elem_b_addr : exec_bg[bb]];
        end
    end endgenerate

`ifdef SIMULATION
    // MAC-idle profiler: categorize why no group is issued in STREAM_INSTRS.
    reg [31:0] mi_bp, mi_starve, mi_ping, mi_other, mi_oexbusy, mi_otask, mi_total, busy_cyc;
    reg [15:0] pk_ptr, pk_task, pk_p0, pk_p1;
    reg mi_printed;
    initial begin mi_bp=0; mi_starve=0; mi_ping=0; mi_other=0; mi_oexbusy=0; mi_otask=0; mi_total=0; busy_cyc=0; mi_printed=0;
                  pk_ptr=0; pk_task=0; pk_p0=0; pk_p1=0; end
    always @(posedge aclk) begin
        if (aresetn) begin
            if (ptr_fifo_cnt      > pk_ptr)  pk_ptr  <= ptr_fifo_cnt;
            if (task_fifo_cnt     > pk_task) pk_task <= task_fifo_cnt;
            if (product_fifo_cnt_0> pk_p0)   pk_p0   <= product_fifo_cnt_0;
            if (product_fifo_cnt_1> pk_p1)   pk_p1   <= product_fifo_cnt_1;
        end
        if (aresetn && state==PE_STREAM_INSTRS) begin
            if (|mac_lane_valid_r) busy_cyc <= busy_cyc + 1;
            else begin
                mi_total <= mi_total + 1;
                if (!exec_prod_safe)                            mi_bp     <= mi_bp + 1;       // accumulator backpressure
                else if (row_gen_done && other_acc_busy)        mi_ping   <= mi_ping + 1;     // waiting on ping-pong to advance
                else if (ptr_fifo_empty && task_fifo_empty)     mi_starve <= mi_starve + 1;   // generation can't keep up
                else if (exec_busy)                             mi_oexbusy<= mi_oexbusy+ 1;   // exec mid-expansion (B-read / latency bubble)
                else if (!task_fifo_empty)                      mi_otask  <= mi_otask + 1;    // exec idle, task pending (ptr<->task handoff)
                else                                            mi_other  <= mi_other + 1;    // exec idle, ptr pending (load/fill bubble)
            end
        end
        if (done && !mi_printed) begin
            mi_printed <= 1;
            $display("[MACPROF] STREAM busy=%0d idle=%0d | backpressure=%0d ping-pong=%0d starve=%0d exec-bubble=%0d task-handoff=%0d ptr-fill=%0d",
                     busy_cyc, mi_total, mi_bp, mi_ping, mi_starve, mi_oexbusy, mi_otask, mi_other);
            $display("[FIFOPEAK] ptr=%0d/%0d task=%0d/%0d prod0=%0d/%0d prod1=%0d/%0d",
                     pk_ptr, `PTR_FIFO_DEPTH, pk_task, `TASK_FIFO_DEPTH,
                     pk_p0, `PROD_FIFO_DEPTH, pk_p1, `PROD_FIFO_DEPTH);
        end
    end
`endif

endmodule
