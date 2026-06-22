//=============================================================================
// File     : accum_bank.v
// Brief    : Single accumulator bank for row_accumulator_4bank.
//
//   FIFO (circular, FIFO_DEPTH deep, must be power-of-2) supports
//   up to 4 independent write ports and 1 read port per cycle.
//   RMW pipeline: IDLE → READ → ADD → WRITE (3 stages, one entry at a time).
//   Tag-based epoch mechanism: avoids clearing acc_mem each row.
//=============================================================================

module accum_bank #(
    parameter BANK_DEPTH     = 128,    // accumulators per bank (OUT_COLS/4)
    parameter BANK_ADDR_W    = 7,      // ceil(log2(BANK_DEPTH))
    parameter PROD_W         = 16,
    parameter ACC_W          = 32,
    parameter EPOCH_W        = 16,
    parameter FIFO_DEPTH     = 8,      // must be power of 2
    parameter FIFO_DEPTH_LOG = 3       // log2(FIFO_DEPTH)
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire [EPOCH_W-1:0]           row_epoch,

    // 4 independent write ports (only active when wr_valid[i]=1)
    input  wire [3:0]                   wr_valid,
    input  wire [4*BANK_ADDR_W-1:0]     wr_addr_flat,  // {addr3,addr2,addr1,addr0}
    input  wire [4*PROD_W-1:0]          wr_data_flat,  // {dat3,dat2,dat1,dat0}
    output wire [3:0]                   free_count,    // 0..FIFO_DEPTH

    output wire                         rmw_busy,
    output wire                         fifo_empty,

    // Sequential tag-clear (triggered by top on epoch wrap)
    input  wire                         tag_clear_en,
    output wire                         tag_clear_busy,

    // Combinational drain read (only valid after all RMW done)
    input  wire [BANK_ADDR_W-1:0]       drain_rd_addr,
    output wire [EPOCH_W-1:0]           drain_tag,
    output wire [ACC_W-1:0]             drain_acc
);

    // =========================================================================
    // Localparams
    // =========================================================================
    localparam ENTRY_W    = BANK_ADDR_W + PROD_W;
    localparam FIFO_MASK  = FIFO_DEPTH - 1;
    localparam BANK_LAST  = BANK_DEPTH - 1;   // last valid bank_addr

    // =========================================================================
    // FIFO — circular ring buffer with 4 write ports, 1 read port
    //
    //   slot_of[i] = number of wr_valid[0..i-1] bits set = write offset for port i
    //   All valid ports write contiguously from fifo_tail each cycle.
    // =========================================================================
    reg [ENTRY_W-1:0]         fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_DEPTH_LOG-1:0]  fifo_tail;
    reg [FIFO_DEPTH_LOG-1:0]  fifo_head;
    reg [FIFO_DEPTH_LOG:0]    fifo_cnt;    // 0 .. FIFO_DEPTH (needs one extra bit)

    // Burst write count this cycle
    wire [2:0] wr_cnt = {2'b0, wr_valid[0]} + {2'b0, wr_valid[1]}
                      + {2'b0, wr_valid[2]} + {2'b0, wr_valid[3]};

    // Per-port write offset within burst (compact the valid ports)
    wire [1:0] slot1 = {1'b0, wr_valid[0]};
    wire [1:0] slot2 = {1'b0, wr_valid[0]} + {1'b0, wr_valid[1]};
    wire [1:0] slot3 = slot2 + {1'b0, wr_valid[2]};

    // Write addresses (3-bit, wraps at FIFO_DEPTH)
    wire [FIFO_DEPTH_LOG-1:0] waddr0 = fifo_tail;
    wire [FIFO_DEPTH_LOG-1:0] waddr1 = fifo_tail + slot1;
    wire [FIFO_DEPTH_LOG-1:0] waddr2 = fifo_tail + slot2;
    wire [FIFO_DEPTH_LOG-1:0] waddr3 = fifo_tail + slot3;

    assign fifo_empty  = (fifo_cnt == {(FIFO_DEPTH_LOG+1){1'b0}});
    assign free_count  = FIFO_DEPTH[3:0] - fifo_cnt[3:0];

    // =========================================================================
    // Tag & accumulator memories
    // =========================================================================
    reg [EPOCH_W-1:0]  tag_mem [0:BANK_DEPTH-1];
    reg [ACC_W-1:0]    acc_mem [0:BANK_DEPTH-1];

    assign drain_tag = tag_mem[drain_rd_addr];
    assign drain_acc = acc_mem[drain_rd_addr];

    // =========================================================================
    // RMW pipeline
    // =========================================================================
    localparam RMW_IDLE  = 2'd0;
    localparam RMW_READ  = 2'd1;
    localparam RMW_ADD   = 2'd2;
    localparam RMW_WRITE = 2'd3;

    reg [1:0]              rmw_st;
    reg [BANK_ADDR_W-1:0]  rmw_addr;
    reg [PROD_W-1:0]       rmw_prod;
    reg [ACC_W-1:0]        rmw_old;
    reg [EPOCH_W-1:0]      rmw_old_tag;
    reg [ACC_W-1:0]        rmw_new;

    wire deq_fire = !fifo_empty && (rmw_st == RMW_IDLE);
    assign rmw_busy = (rmw_st != RMW_IDLE);

    // =========================================================================
    // Tag clear
    // =========================================================================
    reg [BANK_ADDR_W-1:0] clr_idx;
    reg                   clr_active;
    assign tag_clear_busy = clr_active;

    // =========================================================================
    // FIFO process
    // =========================================================================
    integer fi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_tail <= {FIFO_DEPTH_LOG{1'b0}};
            fifo_head <= {FIFO_DEPTH_LOG{1'b0}};
            fifo_cnt  <= {(FIFO_DEPTH_LOG+1){1'b0}};
            for (fi = 0; fi < FIFO_DEPTH; fi = fi + 1)
                fifo_mem[fi] <= {ENTRY_W{1'b0}};
        end else begin
            // Multi-write: each valid port appends to the burst sequence
            if (wr_valid[0])
                fifo_mem[waddr0] <= {wr_addr_flat[0*BANK_ADDR_W +: BANK_ADDR_W],
                                     wr_data_flat[0*PROD_W      +: PROD_W]};
            if (wr_valid[1])
                fifo_mem[waddr1] <= {wr_addr_flat[1*BANK_ADDR_W +: BANK_ADDR_W],
                                     wr_data_flat[1*PROD_W      +: PROD_W]};
            if (wr_valid[2])
                fifo_mem[waddr2] <= {wr_addr_flat[2*BANK_ADDR_W +: BANK_ADDR_W],
                                     wr_data_flat[2*PROD_W      +: PROD_W]};
            if (wr_valid[3])
                fifo_mem[waddr3] <= {wr_addr_flat[3*BANK_ADDR_W +: BANK_ADDR_W],
                                     wr_data_flat[3*PROD_W      +: PROD_W]};

            fifo_tail <= fifo_tail + wr_cnt[FIFO_DEPTH_LOG-1:0];

            if (deq_fire)
                fifo_head <= fifo_head + {{(FIFO_DEPTH_LOG-1){1'b0}}, 1'b1};

            // Count update: +wr_cnt, -deq_fire  (both happen independently)
            fifo_cnt  <= fifo_cnt
                       + {1'b0, wr_cnt}          // zero-extend 3→4 bits
                       - {{FIFO_DEPTH_LOG{1'b0}}, deq_fire};
        end
    end

    // =========================================================================
    // RMW pipeline
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rmw_st      <= RMW_IDLE;
            rmw_addr    <= {BANK_ADDR_W{1'b0}};
            rmw_prod    <= {PROD_W{1'b0}};
            rmw_old     <= {ACC_W{1'b0}};
            rmw_old_tag <= {EPOCH_W{1'b0}};
            rmw_new     <= {ACC_W{1'b0}};
        end else begin
            case (rmw_st)
                RMW_IDLE: begin
                    if (deq_fire) begin
                        rmw_addr <= fifo_mem[fifo_head][ENTRY_W-1 -: BANK_ADDR_W];
                        rmw_prod <= fifo_mem[fifo_head][PROD_W-1:0];
                        rmw_st   <= RMW_READ;
                    end
                end
                RMW_READ: begin
                    rmw_old_tag <= tag_mem[rmw_addr];
                    rmw_old     <= acc_mem[rmw_addr];
                    rmw_st      <= RMW_ADD;
                end
                RMW_ADD: begin
                    if (rmw_old_tag != row_epoch)
                        rmw_new <= {{(ACC_W-PROD_W){rmw_prod[PROD_W-1]}}, rmw_prod};
                    else
                        rmw_new <= rmw_old
                                 + {{(ACC_W-PROD_W){rmw_prod[PROD_W-1]}}, rmw_prod};
                    rmw_st <= RMW_WRITE;
                end
                RMW_WRITE: begin
                    tag_mem[rmw_addr] <= row_epoch;
                    acc_mem[rmw_addr] <= rmw_new;
                    rmw_st            <= RMW_IDLE;
                end
                default: rmw_st <= RMW_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Tag clear (sequential; only triggered after all RMW done)
    // =========================================================================
    integer ci;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clr_active <= 1'b0;
            clr_idx    <= {BANK_ADDR_W{1'b0}};
            for (ci = 0; ci < BANK_DEPTH; ci = ci + 1)
                tag_mem[ci] <= {EPOCH_W{1'b0}};
        end else begin
            if (tag_clear_en && !clr_active) begin
                clr_active <= 1'b1;
                clr_idx    <= {BANK_ADDR_W{1'b0}};
            end
            if (clr_active) begin
                tag_mem[clr_idx] <= {EPOCH_W{1'b0}};
                if (clr_idx == BANK_LAST[BANK_ADDR_W-1:0]) begin
                    clr_active <= 1'b0;
                end else begin
                    clr_idx <= clr_idx + {{(BANK_ADDR_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

    // =========================================================================
    // Simulation assertions
    // =========================================================================
`ifdef SIMULATION
    always @(posedge clk) begin
        if (fifo_cnt > FIFO_DEPTH[FIFO_DEPTH_LOG:0])
            $error("accum_bank FIFO overflow (cnt=%0d)", fifo_cnt);
    end
`endif

endmodule
