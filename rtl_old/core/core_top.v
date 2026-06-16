//=============================================================================
// File     : core_top.v
// Project  : SPGEMM-Accelerator
// Brief    : Top-level core module - connects all sub-modules:
//           Fetch, Load, LoadTask, Store, GlobalBuffer, PE Array, ElementWise,
//           C CSR Writer.
//           Main state machine:
//             IDLE → LOAD_A(×3) → LOAD_B(×3) → LOAD_TASK → COMPUTE → WRITE_CSR → STORE → FINISH
//           COMPUTE branches on op_type:
//             MUL → PE Array (SpGEMM)
//             ADD/SUB → ElementWise Unit
//           Scheduler removed → host computes task descriptors, loaded via LOAD_TASK.
//=============================================================================

`include "defines.vh"
`include "isa.vh"

module core_top (
    // Control Register interface (AXI-Lite slave signals via Wrapper)
    input  wire                      cr_launch,     //启动信号
    input  wire [`AXI_ADDR_WIDTH-1:0] ins_baddr,     //指令起始地址
    input  wire [15:0]               ins_count,        ///指令数量
    output wire                      cr_finish,         //完成信号

    // AXI Read Master (for Fetch + Load)
    output wire                      m_axi_arvalid,     
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,

    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,

    // AXI Write Master (for Store)
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,

    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    output wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,

    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    input  wire [1:0]                m_axi_bresp,

    // Debug / Performance counters
    output wire [15:0]               cycle_counter,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // State Machine (with debug $display)
    //=========================================================================
    localparam STATE_IDLE           = 4'd0;
    localparam STATE_LOAD_A_ROW     = 4'd1;
    localparam STATE_LOAD_A_COL     = 4'd2;
    localparam STATE_LOAD_A_VAL     = 4'd3;
    localparam STATE_LOAD_B_ROW     = 4'd4;
    localparam STATE_LOAD_B_COL     = 4'd5;
    localparam STATE_LOAD_B_VAL     = 4'd6;
    localparam STATE_LOAD_TASK_DATA = 4'd7;
    localparam STATE_PARSE_TASK     = 4'd8;
    localparam STATE_LOAD_PE        = 4'd9;
    localparam STATE_COMPUTE        = 4'd10;
    localparam STATE_CSR_FINALIZE   = 4'd11;
    localparam STATE_STORE          = 4'd12;
    localparam STATE_FINISH         = 4'd13;

    reg [3:0] state, state_next;

    // Operation type (decoded from COMPUTE instruction bit[8:6])
    reg [2:0] op_type;

    //=========================================================================
    // Instruction Decode registers (from COMPUTE instruction)
    //=========================================================================
    reg [15:0] a_row_ptr_sram;
    reg [15:0] a_col_idx_sram;
    reg [15:0] a_val_sram;
    reg [15:0] b_row_ptr_sram;
    reg [15:0] b_col_idx_sram;
    reg [15:0] b_val_sram;
    reg [`MAX_DIM_BITS-1:0] M, K, N;

    //=========================================================================
    // Sub-module signals
    //=========================================================================

    // Fetch → Decode dispatches (4 channels)
    wire [`INST_WIDTH-1:0] fetch_ld_inst, fetch_cp_inst, fetch_st_inst;
    wire fetch_ld_valid, fetch_ld_ready;
    wire fetch_cp_valid, fetch_cp_ready;
    wire fetch_st_valid, fetch_st_ready;

    // Load
    wire load_done;
    wire load_gbuf_wr_en;
    wire [`GBUF_DEPTH_LOG-1:0] load_gbuf_wr_addr;
    wire [`DATA_WIDTH-1:0]   load_gbuf_wr_data;

    // GlobalBuffer
    wire gbuf_rd_en;
    wire [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr;
    wire [`DATA_WIDTH-1:0]     gbuf_rd_data;
    wire gbuf_rd_valid;

    // Task Descriptors (host-computed, loaded via LOAD_TASK)
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_row_start, pe_row_end;
    wire [`N_PE-1:0][15:0]              pe_a_ptr_start, pe_a_ptr_end;
    wire [`N_PE-1:0]                    pe_task_valid;
    wire task_load_done;

    // PE Load signals (STATE_LOAD_PE)
    reg  [`N_PE_BITS:0]                load_pe_idx;        // which PE is loading
    wire [`N_PE-1:0]                   pe_load_done;       // each PE signals when done
    wire                               pe_load_all_done;   // all PEs loaded
    wire [`N_PE-1:0]                   pe_gbuf_rd_en;      // per-PE gbuf read enable
    wire [`N_PE-1:0][`GBUF_DEPTH_LOG-1:0] pe_gbuf_rd_addr; // per-PE gbuf read addr

    // PE Array (for MUL / SpGEMM)
    wire [`N_PE-1:0] pe_done;
    wire pe_all_done;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_out_row_id;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_out_nnz;
    wire [`N_PE-1:0][`DATA_WIDTH-1:0]   pe_out_col;
    wire [`N_PE-1:0][`DATA_WIDTH-1:0]   pe_out_val;
    wire [`N_PE-1:0]                    pe_out_valid;

    // ElementWise Unit (for ADD/SUB)
    wire ew_done;
    wire [`MAX_DIM_BITS-1:0] ew_out_row_id;
    wire [`MAX_DIM_BITS-1:0] ew_out_nnz;
    wire [`DATA_WIDTH-1:0]   ew_out_col;
    wire [`DATA_WIDTH-1:0]   ew_out_val;
    wire                     ew_out_valid;
    wire                     ew_out_row_end;

    // C CSR Writer
    wire csr_done;

    // Store
    wire store_done;

    // AXI read mux
    wire fetch_arvalid, load_arvalid;
    wire fetch_arready, load_arready;
    wire [`AXI_ADDR_WIDTH-1:0] fetch_araddr, load_araddr;
    wire [7:0] fetch_arlen, load_arlen;
    wire fetch_rvalid, load_rvalid;
    wire fetch_rready, load_rready;
    wire [`AXI_DATA_WIDTH-1:0] fetch_rdata, load_rdata;
    wire fetch_rlast, load_rlast;

    //=========================================================================
    // Cycle counter
    //=========================================================================
    reg [15:0] cycle_cnt;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) cycle_cnt <= 0;
        else          cycle_cnt <= cycle_cnt + 1;
    end
    assign cycle_counter = cycle_cnt;

    //=========================================================================
    // State Machine (with debug $display)
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= STATE_IDLE;
            op_type <= `OP_TYPE_MUL;
        end else begin
            state <= state_next;
        end
    end

    // Next state logic: one state per LOAD, STORE→FINISH directly (single-op mode)
    always @(*) begin
        state_next = state;
        case (state)
            STATE_IDLE:           if (cr_launch)            state_next = STATE_LOAD_A_ROW;
            STATE_LOAD_A_ROW:     if (load_done)            state_next = STATE_LOAD_A_COL;
            STATE_LOAD_A_COL:     if (load_done)            state_next = STATE_LOAD_A_VAL;
            STATE_LOAD_A_VAL:     if (load_done)            state_next = STATE_LOAD_B_ROW;
            STATE_LOAD_B_ROW:     if (load_done)            state_next = STATE_LOAD_B_COL;
            STATE_LOAD_B_COL:     if (load_done)            state_next = STATE_LOAD_B_VAL;
            STATE_LOAD_B_VAL:     if (load_done)            state_next = STATE_LOAD_TASK_DATA;
            STATE_LOAD_TASK_DATA: if (load_done)            state_next = STATE_PARSE_TASK;
            STATE_PARSE_TASK:     if (task_load_done)       state_next = (op_type == `OP_TYPE_MUL) ? STATE_LOAD_PE : STATE_COMPUTE;
            STATE_LOAD_PE:        if (pe_load_all_done)     state_next = STATE_COMPUTE;
            STATE_COMPUTE:        if (pe_all_done || ew_done) state_next = STATE_CSR_FINALIZE;
            STATE_CSR_FINALIZE:   if (csr_done)             state_next = STATE_STORE;
            STATE_STORE:          if (store_done)           state_next = STATE_FINISH;
            STATE_FINISH: ;
            default: state_next = STATE_IDLE;
        endcase
    end

    assign cr_finish = (state == STATE_FINISH);

    //=========================================================================
    // Module Instantiation
    //=========================================================================

    // --- Fetch (3 dispatch channels: Load, Compute, Store) ---
    fetch #(
        .INST_QUEUE_DEPTH(16),
        .INST_QUEUE_DEPTH_LOG(4)
    ) u_fetch (
        .launch        (cr_launch),
        .ins_baddr     (ins_baddr),
        .ins_count     (ins_count),
        .m_axi_arvalid (fetch_arvalid),
        .m_axi_arready (fetch_arready),
        .m_axi_araddr  (fetch_araddr),
        .m_axi_arlen   (fetch_arlen),
        .m_axi_rvalid  (fetch_rvalid),
        .m_axi_rready  (fetch_rready),
        .m_axi_rdata   (fetch_rdata),
        .m_axi_rlast   (fetch_rlast),
        .ld_inst_valid (fetch_ld_valid),
        .ld_inst_ready (fetch_ld_ready),
        .ld_inst       (fetch_ld_inst),
        .sp_inst_valid (fetch_cp_valid),
        .sp_inst_ready (fetch_cp_ready),
        .sp_inst       (fetch_cp_inst),
        .st_inst_valid (fetch_st_valid),
        .st_inst_ready (fetch_st_ready),
        .st_inst       (fetch_st_inst),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // --- Decode COMPUTE instruction (SpGEMM / SpAdd / SpSubtract) ---
    wire [15:0] cp_a_row_sram, cp_a_col_sram, cp_a_val_sram;
    wire [15:0] cp_b_row_sram, cp_b_col_sram, cp_b_val_sram;
    wire [`MAX_DIM_BITS-1:0] cp_M, cp_K, cp_N;

    compute_decode u_compute_decode (
        .inst          (fetch_cp_inst),
        .a_row_ptr_sram(cp_a_row_sram),
        .a_col_idx_sram(cp_a_col_sram),
        .a_val_sram    (cp_a_val_sram),
        .b_row_ptr_sram(cp_b_row_sram),
        .b_col_idx_sram(cp_b_col_sram),
        .b_val_sram    (cp_b_val_sram),
        .M             (cp_M),
        .K             (cp_K),
        .N             (cp_N)
    );

    // Latch COMPUTE parameters on instruction
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            op_type <= `OP_TYPE_MUL;
            M <= 0; K <= 0; N <= 0;
            a_row_ptr_sram <= 0; a_col_idx_sram <= 0; a_val_sram <= 0;
            b_row_ptr_sram <= 0; b_col_idx_sram <= 0; b_val_sram <= 0;
        end else if (fetch_cp_valid && fetch_cp_ready) begin
            op_type        <= fetch_cp_inst[`COMPUTE_OP_TYPE_HI:`COMPUTE_OP_TYPE_LO];
            M <= cp_M; K <= cp_K; N <= cp_N;
            a_row_ptr_sram <= cp_a_row_sram;
            a_col_idx_sram <= cp_a_col_sram;
            a_val_sram     <= cp_a_val_sram;
            b_row_ptr_sram <= cp_b_row_sram;
            b_col_idx_sram <= cp_b_col_sram;
            b_val_sram     <= cp_b_val_sram;
        end
    end

    // Fetch instruction ready signals
    assign fetch_cp_ready = 1'b1;  // always accept COMPUTE instructions

    // --- AXI Read Mux ---
    axi_read_mux #(.N_CLIENTS(2)) u_read_mux (
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_arid    (),
        .m_axi_arsize  (),
        .m_axi_arburst (),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .m_axi_rresp   (2'b00),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_rid     (4'b0),
        .s_axi_arvalid ({load_arvalid, fetch_arvalid}),
        .s_axi_arready ({load_arready, fetch_arready}),
        .s_axi_araddr  ({load_araddr,  fetch_araddr}),
        .s_axi_arlen   ({load_arlen,   fetch_arlen}),
        .s_axi_rvalid  ({load_rvalid,  fetch_rvalid}),
        .s_axi_rready  ({load_rready,  fetch_rready}),
        .s_axi_rdata   ({load_rdata,   fetch_rdata}),
        .s_axi_rlast   ({load_rlast,   fetch_rlast}),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // --- GlobalBuffer Read MUX ---
    // During LOAD_TASK: task_loader drives gbuf_rd
    // During LOAD_PE:   selected PE drives gbuf_rd
    // During COMPUTE (ADD/SUB): elementwise drives gbuf_rd
    wire gbuf_rd_en_int;
    wire [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr_int;
    wire task_gbuf_rd_en;
    wire [`GBUF_DEPTH_LOG-1:0] task_gbuf_rd_addr;
    wire ew_gbuf_rd_en;
    wire [`GBUF_DEPTH_LOG-1:0] ew_gbuf_rd_addr;
    wire ew_active = (state == STATE_COMPUTE) && (op_type != `OP_TYPE_MUL);

    // MUX select: task_loader | PE | elementwise
    assign gbuf_rd_en_int   = (state == STATE_PARSE_TASK) ? task_gbuf_rd_en   :
                              (state == STATE_LOAD_PE)    ? pe_gbuf_rd_en[load_pe_idx] :
                              (ew_active)                 ? ew_gbuf_rd_en     : 1'b0;
    assign gbuf_rd_addr_int = (state == STATE_PARSE_TASK) ? task_gbuf_rd_addr :
                              (state == STATE_LOAD_PE)    ? pe_gbuf_rd_addr[load_pe_idx] :
                              (ew_active)                 ? ew_gbuf_rd_addr   : 0;

    // --- Global Buffer ---
    global_buffer #(
        .DEPTH(`GBUF_DEPTH), .DEPTH_LOG(`GBUF_DEPTH_LOG)
    ) u_global_buffer (
        .wr_en    (load_gbuf_wr_en),
        .wr_addr  (load_gbuf_wr_addr),
        .wr_data  (load_gbuf_wr_data),
        .rd_en    (gbuf_rd_en_int),
        .rd_addr  (gbuf_rd_addr_int),
        .rd_data  (gbuf_rd_data),
        .rd_valid (gbuf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    // --- Load ---
    load u_load (
        .inst_valid     (fetch_ld_valid),
        .inst_ready     (fetch_ld_ready),
        .inst_data      (fetch_ld_inst),
        .ext_valid      ((state == STATE_LOAD_A_ROW) || (state == STATE_LOAD_A_COL) || (state == STATE_LOAD_A_VAL) ||
                          (state == STATE_LOAD_B_ROW) || (state == STATE_LOAD_B_COL) || (state == STATE_LOAD_B_VAL) ||
                          (state == STATE_LOAD_TASK_DATA)),
        .done           (load_done),
        .m_axi_arvalid  (load_arvalid),
        .m_axi_arready  (load_arready),
        .m_axi_araddr   (load_araddr),
        .m_axi_arlen    (load_arlen),
        .m_axi_rvalid   (load_rvalid),
        .m_axi_rready   (load_rready),
        .m_axi_rdata    (load_rdata),
        .m_axi_rlast    (load_rlast),
        .gbuf_wr_en     (load_gbuf_wr_en),
        .gbuf_wr_addr   (load_gbuf_wr_addr),
        .gbuf_wr_data   (load_gbuf_wr_data),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

    // Latch task descriptor SRAM base from LOAD_TASK instruction
    reg [`GBUF_DEPTH_LOG-1:0] task_sram_base;
    always @(posedge aclk) begin
        if (fetch_ld_valid && fetch_ld_ready &&
            fetch_ld_inst[`LOAD_SRAM_OFFSET_HI:`LOAD_SRAM_OFFSET_LO] != 0)
            task_sram_base <= fetch_ld_inst[`LOAD_SRAM_OFFSET_HI:`LOAD_SRAM_OFFSET_LO];
    end

    // --- Task Loader: reads host-computed task descriptors from GlobalBuffer ---
    task_loader u_task_loader (
        .start          (state == STATE_PARSE_TASK),
        .done           (task_load_done),
        .pe_row_start   (pe_row_start),
        .pe_row_end     (pe_row_end),
        .pe_a_ptr_start (pe_a_ptr_start),
        .pe_a_ptr_end   (pe_a_ptr_end),
        .pe_task_valid  (pe_task_valid),
        .task_sram_base (task_sram_base),
        .gbuf_rd_en     (task_gbuf_rd_en),
        .gbuf_rd_addr   (task_gbuf_rd_addr),
        .gbuf_rd_data   (gbuf_rd_data),
        .gbuf_rd_valid  (gbuf_rd_valid),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

    //=========================================================================
    // PE Load Sequencer (STATE_LOAD_PE)
    //   Each PE loads its A slice and full B CSR from GlobalBuffer into
    //   local buffers, one PE at a time (sequential, single gbuf read port).
    //=========================================================================
    reg [`N_PE_BITS:0] load_pe_cycle_rem;  // cycles remaining for current PE
    wire load_pe_active = (state == STATE_LOAD_PE);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            load_pe_idx       <= 0;
            load_pe_cycle_rem <= 0;
        end else begin
            if (state == STATE_PARSE_TASK && task_load_done) begin
                // Enter LOAD_PE: start with PE 0
                load_pe_idx       <= 0;
                load_pe_cycle_rem <= `N_PE;  // N_PE PEs to load
            end else if (state == STATE_LOAD_PE) begin
                if (pe_load_done[load_pe_idx]) begin
                    // Current PE done, advance to next
                    if (load_pe_idx < `N_PE - 1) begin
                        load_pe_idx <= load_pe_idx + 1;
                    end else begin
                        load_pe_idx <= 0;
                    end
                    load_pe_cycle_rem <= load_pe_cycle_rem - 1;
                end
            end
        end
    end

    // PE Load done: all PEs completed their local buffer loading
    assign pe_load_all_done = (state == STATE_LOAD_PE) && (load_pe_cycle_rem == 0);

    // --- PE Array (for MUL / SpGEMM, disabled for ADD/SUB) ---
    genvar pe_idx;
    generate
        for (pe_idx = 0; pe_idx < `N_PE; pe_idx = pe_idx + 1) begin : gen_pe_array
            pe_top #(
                .PE_ID(pe_idx)
            ) u_pe (
                .start          (compute_start_pulse && (op_type == `OP_TYPE_MUL) && pe_task_valid[pe_idx]),
                .done           (pe_done[pe_idx]),
                .row_start      (pe_row_start[pe_idx]),
                .row_end        (pe_row_end[pe_idx]),
                .a_ptr_start    (pe_a_ptr_start[pe_idx]),
                .a_ptr_end      (pe_a_ptr_end[pe_idx]),
                .M              (M),
                .K              (K),
                .N              (N),
                .op_type        (op_type),
                .load_en        (load_pe_active && (load_pe_idx == pe_idx)),
                .load_done      (pe_load_done[pe_idx]),
                .a_row_sram     (a_row_ptr_sram),
                .a_col_sram     (a_col_idx_sram),
                .a_val_sram     (a_val_sram),
                .b_row_sram     (b_row_ptr_sram),
                .b_col_sram     (b_col_idx_sram),
                .b_val_sram     (b_val_sram),
                .gbuf_rd_en     (pe_gbuf_rd_en[pe_idx]),
                .gbuf_rd_addr   (pe_gbuf_rd_addr[pe_idx]),
                .gbuf_rd_data   (gbuf_rd_data),
                .gbuf_rd_valid  (gbuf_rd_valid),
                .out_row_id     (pe_out_row_id[pe_idx]),
                .out_nnz        (pe_out_nnz[pe_idx]),
                .out_col        (pe_out_col[pe_idx]),
                .out_val        (pe_out_val[pe_idx]),
                .out_valid      (pe_out_valid[pe_idx]),
                .aclk           (aclk),
                .aresetn        (aresetn)
            );
        end
    endgenerate

    // Mask invalid PEs: they count as done
    wire [`N_PE-1:0] pe_done_masked;
    genvar i_done;
    generate
        for (i_done = 0; i_done < `N_PE; i_done = i_done + 1) begin : gen_done_mask
            assign pe_done_masked[i_done] = (!pe_task_valid[i_done]) || pe_done[i_done];
        end
    endgenerate
    assign pe_all_done = &pe_done_masked;

    // Compute start pulse (one cycle, avoids pe_decompress done deadlock)
    reg compute_entered;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) compute_entered <= 1'b0;
        else if (state != STATE_COMPUTE) compute_entered <= 1'b0;
        else compute_entered <= 1'b1;
    end
    wire compute_start_pulse = (state == STATE_COMPUTE) && !compute_entered;

    // --- ElementWise Unit (for ADD/SUB, disabled for MUL) ---
    sp_elementwise u_elementwise (
        .start          (compute_start_pulse && (op_type != `OP_TYPE_MUL)),
        .done           (ew_done),
        .op_type        (op_type),
        .M              (M),
        .N              (N),
        .a_row_sram     (a_row_ptr_sram),
        .a_col_sram     (a_col_idx_sram),
        .a_val_sram     (a_val_sram),
        .b_row_sram     (b_row_ptr_sram),
        .b_col_sram     (b_col_idx_sram),
        .b_val_sram     (b_val_sram),
        .gbuf_rd_en     (ew_gbuf_rd_en),
        .gbuf_rd_addr   (ew_gbuf_rd_addr),
        .gbuf_rd_data   (gbuf_rd_data),
        .gbuf_rd_valid  (gbuf_rd_valid),
        .out_row_id     (ew_out_row_id),
        .out_nnz        (ew_out_nnz),
        .out_col        (ew_out_col),
        .out_val        (ew_out_val),
        .out_valid      (ew_out_valid),
        .out_row_end    (ew_out_row_end),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

    // Mux PE Array or ElementWise output to CSR Writer
    // CSR Writer expects N_PE-wide arrays; for ADD/SUB we only use PE[0]
    wire [`N_PE-1:0]                 csr_pe_valid_mux;
    wire [`N_PE-1:0]                 csr_pe_row_end_mux;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] csr_pe_row_id_mux;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] csr_pe_nnz_mux;
    wire [`N_PE-1:0][`DATA_WIDTH-1:0]   csr_pe_col_mux;
    wire [`N_PE-1:0][`DATA_WIDTH-1:0]   csr_pe_val_mux;

    // Zero vectors for padding when using ElementWise (single-output) instead of PE Array (multi-output)
    wire [`N_PE-2:0] zero_pe_vec     = {(`N_PE-1){1'b0}};
    wire [`MAX_DIM_BITS-1:0] zero_dim = 0;
    wire [`DATA_WIDTH-1:0]   zero_dw  = 0;

    assign csr_pe_valid_mux   = (op_type == `OP_TYPE_MUL) ? pe_out_valid   : {zero_pe_vec, ew_out_valid};
    assign csr_pe_row_end_mux = (op_type == `OP_TYPE_MUL) ? {`N_PE{1'b0}} : {zero_pe_vec, ew_out_row_end};
    // For row_id/nnz/col/val, replicate zero padding for N_PE-1 entries
    assign csr_pe_row_id_mux[0]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[0]  : ew_out_row_id;
    assign csr_pe_row_id_mux[1]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[1]  : zero_dim;
    assign csr_pe_row_id_mux[2]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[2]  : zero_dim;
    assign csr_pe_row_id_mux[3]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[3]  : zero_dim;
    assign csr_pe_row_id_mux[4]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[4]  : zero_dim;
    assign csr_pe_row_id_mux[5]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[5]  : zero_dim;
    assign csr_pe_row_id_mux[6]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[6]  : zero_dim;
    assign csr_pe_row_id_mux[7]  = (op_type == `OP_TYPE_MUL) ? pe_out_row_id[7]  : zero_dim;

    assign csr_pe_nnz_mux[0] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[0] : ew_out_nnz;
    assign csr_pe_nnz_mux[1] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[1] : zero_dim;
    assign csr_pe_nnz_mux[2] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[2] : zero_dim;
    assign csr_pe_nnz_mux[3] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[3] : zero_dim;
    assign csr_pe_nnz_mux[4] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[4] : zero_dim;
    assign csr_pe_nnz_mux[5] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[5] : zero_dim;
    assign csr_pe_nnz_mux[6] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[6] : zero_dim;
    assign csr_pe_nnz_mux[7] = (op_type == `OP_TYPE_MUL) ? pe_out_nnz[7] : zero_dim;

    assign csr_pe_col_mux[0] = (op_type == `OP_TYPE_MUL) ? pe_out_col[0] : ew_out_col;
    assign csr_pe_col_mux[1] = (op_type == `OP_TYPE_MUL) ? pe_out_col[1] : zero_dw;
    assign csr_pe_col_mux[2] = (op_type == `OP_TYPE_MUL) ? pe_out_col[2] : zero_dw;
    assign csr_pe_col_mux[3] = (op_type == `OP_TYPE_MUL) ? pe_out_col[3] : zero_dw;
    assign csr_pe_col_mux[4] = (op_type == `OP_TYPE_MUL) ? pe_out_col[4] : zero_dw;
    assign csr_pe_col_mux[5] = (op_type == `OP_TYPE_MUL) ? pe_out_col[5] : zero_dw;
    assign csr_pe_col_mux[6] = (op_type == `OP_TYPE_MUL) ? pe_out_col[6] : zero_dw;
    assign csr_pe_col_mux[7] = (op_type == `OP_TYPE_MUL) ? pe_out_col[7] : zero_dw;

    assign csr_pe_val_mux[0] = (op_type == `OP_TYPE_MUL) ? pe_out_val[0] : ew_out_val;
    assign csr_pe_val_mux[1] = (op_type == `OP_TYPE_MUL) ? pe_out_val[1] : zero_dw;
    assign csr_pe_val_mux[2] = (op_type == `OP_TYPE_MUL) ? pe_out_val[2] : zero_dw;
    assign csr_pe_val_mux[3] = (op_type == `OP_TYPE_MUL) ? pe_out_val[3] : zero_dw;
    assign csr_pe_val_mux[4] = (op_type == `OP_TYPE_MUL) ? pe_out_val[4] : zero_dw;
    assign csr_pe_val_mux[5] = (op_type == `OP_TYPE_MUL) ? pe_out_val[5] : zero_dw;
    assign csr_pe_val_mux[6] = (op_type == `OP_TYPE_MUL) ? pe_out_val[6] : zero_dw;
    assign csr_pe_val_mux[7] = (op_type == `OP_TYPE_MUL) ? pe_out_val[7] : zero_dw;

    // --- C CSR Writer (unified for PE Array and ElementWise output) ---
    wire csr_done_int;
    wire                                 csr_wr_en;
    wire [`OUTBUF_DEPTH_LOG-1:0]         csr_wr_addr;
    wire [`DATA_WIDTH-1:0]               csr_wr_data;

    wire compute_done = (op_type == `OP_TYPE_MUL) ? pe_all_done : ew_done;

    c_csr_writer u_csr_writer (
        .start       (state == STATE_COMPUTE),
        .done        (csr_done_int),
        .pe_row_valid(csr_pe_valid_mux),
        .pe_row_end (csr_pe_row_end_mux),
        .pe_row_id   (csr_pe_row_id_mux),
        .pe_nnz      (csr_pe_nnz_mux),
        .pe_col      (csr_pe_col_mux),
        .pe_val      (csr_pe_val_mux),
        .M           (M),
        .pe_all_done (pe_all_done),
        .compute_done(compute_done),
        .obuf_wr_en  (csr_wr_en),
        .obuf_wr_addr(csr_wr_addr),
        .obuf_wr_data(csr_wr_data),
        .aclk        (aclk),
        .aresetn     (aresetn)
    );
    assign csr_done = csr_done_int;

    // --- Output Scratchpad ---
    wire                                 osp_rd_en;
    wire [`OUTBUF_DEPTH_LOG-1:0]         osp_rd_addr;
    wire [`AXI_DATA_WIDTH-1:0]           osp_rd_data;
    wire                                 osp_rd_valid;

    output_scratchpad #(
        .DEPTH(`OUTBUF_DEPTH), .DEPTH_LOG(`OUTBUF_DEPTH_LOG)
    ) u_outbuf (
        .wr_en   (csr_wr_en),
        .wr_addr (csr_wr_addr),
        .wr_data (csr_wr_data),
        .rd_en   (osp_rd_en),
        .rd_addr (osp_rd_addr),
        .rd_data (osp_rd_data),
        .rd_valid(osp_rd_valid),
        .aclk    (aclk),
        .aresetn (aresetn)
    );

    // --- Store ---
    store u_store (
        .inst_valid     (fetch_st_valid),
        .inst_ready     (fetch_st_ready),
        .inst_data      (fetch_st_inst),
        .ext_valid      (state == STATE_STORE),
        .done           (store_done),
        .osp_rd_en      (osp_rd_en),
        .osp_rd_addr    (osp_rd_addr),
        .osp_rd_data    (osp_rd_data),
        .osp_rd_valid   (osp_rd_valid),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_bresp    (m_axi_bresp),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

endmodule
