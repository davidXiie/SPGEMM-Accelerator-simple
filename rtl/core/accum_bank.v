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
    input  wire [EPOCH_W-1:0]           row_epoch,   //输入的16位tag

    // 4 independent write ports (only active when wr_valid[i]=1)
    input  wire [3:0]                   wr_valid,
    input  wire [4*BANK_ADDR_W-1:0]     wr_addr_flat,  // {addr3,addr2,addr1,addr0}
    input  wire [4*PROD_W-1:0]          wr_data_flat,  // {dat3,dat2,dat1,dat0}
    output wire [FIFO_DEPTH_LOG:0]      free_count,    // 0..FIFO_DEPTH   fifo剩余容量

    output wire                         rmw_busy,      //RMW 流水线是否有空闲
    output wire                         fifo_empty,     //fifo 是否为空

    // Sequential tag-clear (triggered by top on epoch wrap)
    input  wire                         tag_clear_en,    //清除tag
    output wire                         tag_clear_busy,  //tag_clear 是否有空闲

    // Combinational drain read (only valid after all RMW done)
    input  wire [BANK_ADDR_W-1:0]       drain_rd_addr, //上层选择读取的bank地址
    output wire [EPOCH_W-1:0]           drain_tag,  //判断tag是否相等
    output wire [ACC_W-1:0]             drain_acc     //输出的累加结果
);

    // =========================================================================
    // Localparams
    // =========================================================================
    localparam ENTRY_W    = BANK_ADDR_W + PROD_W;   //fifo存地址+值
    localparam FIFO_MASK  = FIFO_DEPTH - 1;             //深度是8
    localparam BANK_LAST  = BANK_DEPTH - 1;   //   bank深度128

    // =========================================================================
    // FIFO — circular ring buffer with 4 write ports, 1 read port
    //
    //   slot_of[i] = number of wr_valid[0..i-1] bits set = write offset for port i
    //   All valid ports write contiguously from fifo_tail each cycle.
    // =========================================================================
    (* ram_style = "distributed" *) reg [ENTRY_W-1:0]         fifo_mem [0:FIFO_DEPTH-1];       //8*23 fifo  
    reg [FIFO_DEPTH_LOG-1:0]  fifo_tail;                        //写指针
    reg [FIFO_DEPTH_LOG-1:0]  fifo_head;                        //读指针
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
    assign free_count  = FIFO_DEPTH[FIFO_DEPTH_LOG:0] - fifo_cnt;

    // =========================================================================
    // Tag & accumulator memories
    // =========================================================================
    (* ram_style = "distributed" *) reg [EPOCH_W-1:0]  tag_mem [0:BANK_DEPTH-1];     //存128个epoch
    (* ram_style = "distributed" *) reg [ACC_W-1:0]    acc_mem [0:BANK_DEPTH-1];      //128个累加器

    assign drain_tag = tag_mem[drain_rd_addr];
    assign drain_acc = acc_mem[drain_rd_addr];          //通过 drain_rd_addr 同时读出对应地址的 tag 和 acc，供上层判断数据有效性并获取累加结果

    // =========================================================================
    // RMW pipeline — 2 stages, 1 entry/cycle throughput
    //   S1: fetch addr+prod from FIFO, compute new value combinationally
    //   S2: write result to tag_mem/acc_mem
    //   Forwarding: if S1 and S2 target the same address, bypass memory reads
    // =========================================================================
    reg                   s1_valid;
    reg [BANK_ADDR_W-1:0] s1_addr;
    reg [PROD_W-1:0]      s1_prod;

    reg                   s2_valid;
    reg [BANK_ADDR_W-1:0] s2_addr;
    reg [ACC_W-1:0]       s2_new_val;

    wire deq_fire = !fifo_empty;   //FIFO 非空就取
    assign rmw_busy = s1_valid | s2_valid;

    wire               s12_hazard   = s1_valid && s2_valid && (s1_addr == s2_addr);  
    // RAW 冲突：S1 要读地址 X，S2 正要写地址 X（还没落地），S1 如果直接读 mem 会拿到旧值。
    wire [ACC_W-1:0]   s1_old_acc   = s12_hazard ? s2_new_val : acc_mem[s1_addr]; // 绕过内存，直接用 S2 还没写回的结果
    wire               s1_epoch_hit = s12_hazard ? 1'b1       : (tag_mem[s1_addr] == row_epoch);   
    wire [ACC_W-1:0]   s1_new_val   = s1_epoch_hit
        ? (s1_old_acc + {{(ACC_W-PROD_W){s1_prod[PROD_W-1]}}, s1_prod})
        : {{(ACC_W-PROD_W){s1_prod[PROD_W-1]}}, s1_prod};

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

            fifo_tail <= fifo_tail + wr_cnt;

            if (deq_fire)
                fifo_head <= fifo_head + {{(FIFO_DEPTH_LOG-1){1'b0}}, 1'b1};

            // Count update: +wr_cnt, -deq_fire  (both happen independently)
            fifo_cnt  <= fifo_cnt
                       + {1'b0, wr_cnt}          // zero-extend 3→4 bits
                       - {{FIFO_DEPTH_LOG{1'b0}}, deq_fire};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid   <= 1'b0;
            s1_addr    <= {BANK_ADDR_W{1'b0}};
            s1_prod    <= {PROD_W{1'b0}};
            s2_valid   <= 1'b0;
            s2_addr    <= {BANK_ADDR_W{1'b0}};
            s2_new_val <= {ACC_W{1'b0}};
        end else begin
            // S2: write result to memories
            // Tag-clear writes merged here (s2_valid and clr_active never
            // overlap: clr_active only set after all RMW done).
            if (s2_valid) begin
                tag_mem[s2_addr] <= row_epoch;
                acc_mem[s2_addr] <= s2_new_val;
            end else if (clr_active) begin
                tag_mem[clr_idx] <= {EPOCH_W{1'b0}};
            end
            // Advance S1 → S2
            s2_valid   <= s1_valid;
            s2_addr    <= s1_addr;
            s2_new_val <= s1_new_val;
            // Dequeue FIFO → S1
            if (deq_fire) begin
                s1_valid <= 1'b1;
                s1_addr  <= fifo_mem[fifo_head][ENTRY_W-1 -: BANK_ADDR_W];
                s1_prod  <= fifo_mem[fifo_head][PROD_W-1:0];
            end else begin
                s1_valid <= 1'b0;
            end
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
        end else begin
            if (tag_clear_en && !clr_active) begin
                clr_active <= 1'b1;
                clr_idx    <= {BANK_ADDR_W{1'b0}};
            end
            if (clr_active) begin
                // tag_mem write handled in main RMW always block
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
    integer si;
    initial begin
        for (si = 0; si < BANK_DEPTH; si = si + 1) begin
            tag_mem[si] = {EPOCH_W{1'b0}};
            acc_mem[si] = {ACC_W{1'b0}};
        end
        for (si = 0; si < FIFO_DEPTH; si = si + 1)
            fifo_mem[si] = {ENTRY_W{1'b0}};
    end
    always @(posedge clk) begin
        if (fifo_cnt > FIFO_DEPTH[FIFO_DEPTH_LOG:0])
            $error("accum_bank FIFO overflow (cnt=%0d)", fifo_cnt);
    end
`endif

endmodule
