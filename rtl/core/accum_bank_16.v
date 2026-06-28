//=============================================================================
// File     : accum_bank_16.v
// Brief    : Single-write-port accumulator bank.
//
//   One column-bank of the row accumulator.  Takes ONE (addr,product) write
//   per cycle, buffers it in a small FIFO, and applies it to acc_mem with a
//   2-stage tag-checked FP16 read-modify-write.
//
//   History: this used to expose 16 write ports (one per MAC lane) and pack a
//   whole burst into the FIFO in one cycle via a prefix-sum crossbar.  That
//   crossbar (16 ports x FIFO_DEPTH slots) was ~5k LUT PER BANK and the single
//   biggest LUT cost in the PE.  row_accumulator_16bank now scatters each
//   input group across the banks at <=1 lane/bank/cycle (multi-cycle only when
//   a group has same-bank collisions), so each bank needs just ONE write port.
//   The FIFO is now single-write/single-read -> trivial and RAM-mappable.
//=============================================================================

module accum_bank_16 #(
    parameter BANK_DEPTH     = 32,
    parameter BANK_ADDR_W    = 5,
    parameter PROD_W         = 32,
    parameter ACC_W          = 32,
    parameter EPOCH_W        = 16,
    parameter FIFO_DEPTH     = 32,
    parameter FIFO_DEPTH_LOG = 5
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire [EPOCH_W-1:0]           row_epoch,

    // Two write ports (port1 only used when port0 is too -> entries land at
    // tail, tail+1).  Two ports absorb the common 2-way same-bank collision
    // (Gen2 carry+current) in one cycle; deeper collisions are scattered over
    // multiple cycles by row_accumulator_16bank.  The per-bank scatter SELECT
    // network in the parent scales with the port count, so 2 (vs 4) roughly
    // halves the accumulator LUT at a ~1.42x (vs 1.0x) throughput cost.
    input  wire                         wr_en0,
    input  wire [BANK_ADDR_W-1:0]      wr_addr0,
    input  wire [PROD_W-1:0]           wr_data0,
    input  wire                         wr_en1,
    input  wire [BANK_ADDR_W-1:0]      wr_addr1,
    input  wire [PROD_W-1:0]           wr_data1,
    output wire [FIFO_DEPTH_LOG:0]     free_count,

    output wire                         rmw_busy,
    output wire                         fifo_empty,

    input  wire                         tag_clear_en,
    output wire                         tag_clear_busy,

    input  wire [BANK_ADDR_W-1:0]      drain_rd_addr,
    output wire [EPOCH_W-1:0]          drain_tag,
    output wire [ACC_W-1:0]            drain_acc
);

    localparam ENTRY_W   = BANK_ADDR_W + PROD_W;
    localparam BANK_LAST = BANK_DEPTH - 1;

    reg [ENTRY_W-1:0]        fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_DEPTH_LOG-1:0] fifo_tail;
    reg [FIFO_DEPTH_LOG-1:0] fifo_head;
    reg [FIFO_DEPTH_LOG:0]   fifo_cnt;

    assign fifo_empty = (fifo_cnt == {(FIFO_DEPTH_LOG+1){1'b0}});
    assign free_count = FIFO_DEPTH[FIFO_DEPTH_LOG:0] - fifo_cnt;

    // Force LUTRAM (distributed) so all 16 banks map identically.  Without an
    // explicit style Vivado's inference is non-deterministic: some banks keep
    // acc/tag in RAM32M while one or two per accumulator bail to registers +
    // address-decode muxes and balloon to ~100k LUTs (the "bank9" outliers).
    // Both arrays are 1-write / async-read, which LUTRAM supports directly.
    (* ram_style = "distributed" *) reg [EPOCH_W-1:0] tag_mem [0:BANK_DEPTH-1];
    (* ram_style = "distributed" *) reg [ACC_W-1:0]   acc_mem [0:BANK_DEPTH-1];

    assign drain_tag = tag_mem[drain_rd_addr];
    assign drain_acc = acc_mem[drain_rd_addr];

    reg                   s1_valid;
    reg [BANK_ADDR_W-1:0] s1_addr;
    reg [PROD_W-1:0]      s1_prod;

    reg                   s2_valid;
    reg [BANK_ADDR_W-1:0] s2_addr;
    reg [ACC_W-1:0]       s2_new_val;

    wire deq_fire = !fifo_empty;
    assign rmw_busy = s1_valid | s2_valid;

    wire             s12_hazard   = s1_valid && s2_valid && (s1_addr == s2_addr);
    wire [ACC_W-1:0] s1_old_acc   = s12_hazard ? s2_new_val : acc_mem[s1_addr];
    wire             s1_epoch_hit = s12_hazard ? 1'b1       : (tag_mem[s1_addr] == row_epoch);

    wire [15:0] fp16_sum;
    fp16_add u_fp16_add (
        .a(s1_old_acc[15:0]),
        .b(s1_prod[15:0]),
        .z(fp16_sum)
    );
    wire [ACC_W-1:0] s1_new_val = s1_epoch_hit ? fp16_sum : s1_prod;

    reg [BANK_ADDR_W-1:0] clr_idx;
    reg                   clr_active;
    assign tag_clear_busy = clr_active;

    // Two-write / single-read FIFO front-end.  waddr1 = tail+1 (port1 implies
    // port0), so each cycle appends 0, 1, or 2 entries.
    wire [FIFO_DEPTH_LOG-1:0] waddr0 = fifo_tail;
    wire [FIFO_DEPTH_LOG-1:0] waddr1 = fifo_tail + {{(FIFO_DEPTH_LOG-1){1'b0}}, 1'b1};
    wire [1:0] wr_cnt = {1'b0, wr_en0} + {1'b0, wr_en1};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_tail <= {FIFO_DEPTH_LOG{1'b0}};
            fifo_head <= {FIFO_DEPTH_LOG{1'b0}};
            fifo_cnt  <= {(FIFO_DEPTH_LOG+1){1'b0}};
        end else begin
            if (wr_en0) fifo_mem[waddr0] <= {wr_addr0, wr_data0};
            if (wr_en1) fifo_mem[waddr1] <= {wr_addr1, wr_data1};
            fifo_tail <= fifo_tail + {{(FIFO_DEPTH_LOG-2){1'b0}}, wr_cnt};

            if (deq_fire)
                fifo_head <= fifo_head + {{(FIFO_DEPTH_LOG-1){1'b0}}, 1'b1};

            fifo_cnt <= fifo_cnt
                      + {{(FIFO_DEPTH_LOG-1){1'b0}}, wr_cnt}
                      - {{FIFO_DEPTH_LOG{1'b0}}, deq_fire};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid   <= 1'b0; s1_addr  <= {BANK_ADDR_W{1'b0}}; s1_prod  <= {PROD_W{1'b0}};
            s2_valid   <= 1'b0; s2_addr  <= {BANK_ADDR_W{1'b0}}; s2_new_val <= {ACC_W{1'b0}};
        end else begin
            s2_valid   <= s1_valid; s2_addr  <= s1_addr; s2_new_val <= s1_new_val;
            if (deq_fire) begin
                s1_valid <= 1'b1;
                s1_addr  <= fifo_mem[fifo_head][ENTRY_W-1 -: BANK_ADDR_W];
                s1_prod  <= fifo_mem[fifo_head][PROD_W-1:0];
            end else begin
                s1_valid <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clr_active <= 1'b0;
            clr_idx    <= {BANK_ADDR_W{1'b0}};
        end else begin
            if (tag_clear_en && !clr_active) begin
                clr_active <= 1'b1;
                clr_idx    <= {BANK_ADDR_W{1'b0}};
            end
            if (clr_active) begin
                if (clr_idx == BANK_LAST[BANK_ADDR_W-1:0])
                    clr_active <= 1'b0;
                else
                    clr_idx <= clr_idx + {{(BANK_ADDR_W-1){1'b0}}, 1'b1};
            end
        end
    end

    // Single write port (clr walk OR accumulate), synchronous, NO content reset
    // -> infers LUTRAM cleanly.  Stale tags after a soft reset are scrubbed by
    // the S_CLEAR_TAGS walk that row_accumulator_16bank now runs out of reset
    // (clr_active drives clr_idx 0..LAST here), so we no longer need the parallel
    // rst_n clear that previously forced this array into registers.
    always @(posedge clk) begin
        if (clr_active)
            tag_mem[clr_idx] <= {EPOCH_W{1'b0}};
        else if (s2_valid)
            tag_mem[s2_addr] <= row_epoch;
    end

    always @(posedge clk) begin
        if (s2_valid)
            acc_mem[s2_addr] <= s2_new_val;
    end

`ifndef SYNTHESIS
    integer _ci, _fi;
    initial begin
        for (_ci = 0; _ci < BANK_DEPTH; _ci = _ci + 1) begin
            tag_mem[_ci] = {EPOCH_W{1'b0}};
            acc_mem[_ci] = {ACC_W{1'b0}};
        end
        for (_fi = 0; _fi < FIFO_DEPTH; _fi = _fi + 1)
            fifo_mem[_fi] = {ENTRY_W{1'b0}};
    end
`endif

`ifdef SIMULATION
    always @(posedge clk) begin
        if (fifo_cnt > FIFO_DEPTH[FIFO_DEPTH_LOG:0]) begin
            $display("ERROR accum_bank_16 FIFO overflow (cnt=%0d)", fifo_cnt);
            $stop;
        end
    end
`endif

endmodule
