//=============================================================================
// File     : accum_bank_16.v
// Brief    : 16-write-port accumulator bank (expanded from accum_bank.v).
//
//   Identical to accum_bank but with 16 independent write ports instead of 8.
//   Used by row_accumulator_16bank.
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

    // 16 independent write ports
    input  wire [15:0]                  wr_valid,
    input  wire [16*BANK_ADDR_W-1:0]   wr_addr_flat,
    input  wire [16*PROD_W-1:0]        wr_data_flat,
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
    localparam FIFO_MASK = FIFO_DEPTH - 1;
    localparam BANK_LAST = BANK_DEPTH - 1;

    reg [ENTRY_W-1:0]        fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_DEPTH_LOG-1:0] fifo_tail;
    reg [FIFO_DEPTH_LOG-1:0] fifo_head;
    reg [FIFO_DEPTH_LOG:0]   fifo_cnt;

    // Sum of wr_valid bits — max 16, needs 5 bits
    wire [4:0] wr_cnt =
          {4'b0, wr_valid[0]}  + {4'b0, wr_valid[1]}
        + {4'b0, wr_valid[2]}  + {4'b0, wr_valid[3]}
        + {4'b0, wr_valid[4]}  + {4'b0, wr_valid[5]}
        + {4'b0, wr_valid[6]}  + {4'b0, wr_valid[7]}
        + {4'b0, wr_valid[8]}  + {4'b0, wr_valid[9]}
        + {4'b0, wr_valid[10]} + {4'b0, wr_valid[11]}
        + {4'b0, wr_valid[12]} + {4'b0, wr_valid[13]}
        + {4'b0, wr_valid[14]} + {4'b0, wr_valid[15]};

    // Cumulative write offsets within burst (4-bit, max 15)
    wire [3:0] slot1  = {3'b0, wr_valid[0]};
    wire [3:0] slot2  = slot1  + {3'b0, wr_valid[1]};
    wire [3:0] slot3  = slot2  + {3'b0, wr_valid[2]};
    wire [3:0] slot4  = slot3  + {3'b0, wr_valid[3]};
    wire [3:0] slot5  = slot4  + {3'b0, wr_valid[4]};
    wire [3:0] slot6  = slot5  + {3'b0, wr_valid[5]};
    wire [3:0] slot7  = slot6  + {3'b0, wr_valid[6]};
    wire [3:0] slot8  = slot7  + {3'b0, wr_valid[7]};
    wire [3:0] slot9  = slot8  + {3'b0, wr_valid[8]};
    wire [3:0] slot10 = slot9  + {3'b0, wr_valid[9]};
    wire [3:0] slot11 = slot10 + {3'b0, wr_valid[10]};
    wire [3:0] slot12 = slot11 + {3'b0, wr_valid[11]};
    wire [3:0] slot13 = slot12 + {3'b0, wr_valid[12]};
    wire [3:0] slot14 = slot13 + {3'b0, wr_valid[13]};
    wire [3:0] slot15 = slot14 + {3'b0, wr_valid[14]};

    wire [FIFO_DEPTH_LOG-1:0] waddr0  = fifo_tail;
    wire [FIFO_DEPTH_LOG-1:0] waddr1  = fifo_tail + {1'b0, slot1};
    wire [FIFO_DEPTH_LOG-1:0] waddr2  = fifo_tail + {1'b0, slot2};
    wire [FIFO_DEPTH_LOG-1:0] waddr3  = fifo_tail + {1'b0, slot3};
    wire [FIFO_DEPTH_LOG-1:0] waddr4  = fifo_tail + {1'b0, slot4};
    wire [FIFO_DEPTH_LOG-1:0] waddr5  = fifo_tail + {1'b0, slot5};
    wire [FIFO_DEPTH_LOG-1:0] waddr6  = fifo_tail + {1'b0, slot6};
    wire [FIFO_DEPTH_LOG-1:0] waddr7  = fifo_tail + {1'b0, slot7};
    wire [FIFO_DEPTH_LOG-1:0] waddr8  = fifo_tail + {1'b0, slot8};
    wire [FIFO_DEPTH_LOG-1:0] waddr9  = fifo_tail + {1'b0, slot9};
    wire [FIFO_DEPTH_LOG-1:0] waddr10 = fifo_tail + {1'b0, slot10};
    wire [FIFO_DEPTH_LOG-1:0] waddr11 = fifo_tail + {1'b0, slot11};
    wire [FIFO_DEPTH_LOG-1:0] waddr12 = fifo_tail + {1'b0, slot12};
    wire [FIFO_DEPTH_LOG-1:0] waddr13 = fifo_tail + {1'b0, slot13};
    wire [FIFO_DEPTH_LOG-1:0] waddr14 = fifo_tail + {1'b0, slot14};
    wire [FIFO_DEPTH_LOG-1:0] waddr15 = fifo_tail + {1'b0, slot15};

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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_tail <= {FIFO_DEPTH_LOG{1'b0}};
            fifo_head <= {FIFO_DEPTH_LOG{1'b0}};
            fifo_cnt  <= {(FIFO_DEPTH_LOG+1){1'b0}};
        end else begin
            if (wr_valid[0])  fifo_mem[waddr0]  <= {wr_addr_flat[0 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[0 *PROD_W+:PROD_W]};
            if (wr_valid[1])  fifo_mem[waddr1]  <= {wr_addr_flat[1 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[1 *PROD_W+:PROD_W]};
            if (wr_valid[2])  fifo_mem[waddr2]  <= {wr_addr_flat[2 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[2 *PROD_W+:PROD_W]};
            if (wr_valid[3])  fifo_mem[waddr3]  <= {wr_addr_flat[3 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[3 *PROD_W+:PROD_W]};
            if (wr_valid[4])  fifo_mem[waddr4]  <= {wr_addr_flat[4 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[4 *PROD_W+:PROD_W]};
            if (wr_valid[5])  fifo_mem[waddr5]  <= {wr_addr_flat[5 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[5 *PROD_W+:PROD_W]};
            if (wr_valid[6])  fifo_mem[waddr6]  <= {wr_addr_flat[6 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[6 *PROD_W+:PROD_W]};
            if (wr_valid[7])  fifo_mem[waddr7]  <= {wr_addr_flat[7 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[7 *PROD_W+:PROD_W]};
            if (wr_valid[8])  fifo_mem[waddr8]  <= {wr_addr_flat[8 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[8 *PROD_W+:PROD_W]};
            if (wr_valid[9])  fifo_mem[waddr9]  <= {wr_addr_flat[9 *BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[9 *PROD_W+:PROD_W]};
            if (wr_valid[10]) fifo_mem[waddr10] <= {wr_addr_flat[10*BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[10*PROD_W+:PROD_W]};
            if (wr_valid[11]) fifo_mem[waddr11] <= {wr_addr_flat[11*BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[11*PROD_W+:PROD_W]};
            if (wr_valid[12]) fifo_mem[waddr12] <= {wr_addr_flat[12*BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[12*PROD_W+:PROD_W]};
            if (wr_valid[13]) fifo_mem[waddr13] <= {wr_addr_flat[13*BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[13*PROD_W+:PROD_W]};
            if (wr_valid[14]) fifo_mem[waddr14] <= {wr_addr_flat[14*BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[14*PROD_W+:PROD_W]};
            if (wr_valid[15]) fifo_mem[waddr15] <= {wr_addr_flat[15*BANK_ADDR_W+:BANK_ADDR_W], wr_data_flat[15*PROD_W+:PROD_W]};

            fifo_tail <= fifo_tail + wr_cnt[FIFO_DEPTH_LOG-1:0];

            if (deq_fire)
                fifo_head <= fifo_head + {{(FIFO_DEPTH_LOG-1){1'b0}}, 1'b1};

            fifo_cnt <= fifo_cnt + {1'b0, wr_cnt} - {{FIFO_DEPTH_LOG{1'b0}}, deq_fire};
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
