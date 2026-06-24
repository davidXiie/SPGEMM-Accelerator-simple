//=============================================================================
// File     : core_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Top-level core — FSM:
//             IDLE → LOAD_DESC → LOAD_B → LOAD_A → START_PE
//                  → WAIT_PE → WRITE_C_DENSE → FINISH
//
//   B buffer: global shared (b_shared_buffer), 4 replicas.
//   B_row_desc: kept per-PE (512×64b LUT-RAM, small).
//=============================================================================

`include "defines.vh"

module core_top (
    input  wire                      cr_start,
    input  wire                      cr_clear,
    input  wire [`MAX_DIM_BITS-1:0]  M,
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,
    input  wire [7:0]                pe_valid_mask,
    output wire                      cr_finish,
    output wire                      cr_busy,

    // AXI Read Master
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,

    // AXI Write Master
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

    output wire [15:0]               cycle_counter,
    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // State Machine
    //=========================================================================
    localparam S_IDLE           = 4'd0;
    localparam S_LOAD_DESC      = 4'd1;
    localparam S_LOAD_B         = 4'd2;
    localparam S_LOAD_A         = 4'd3;
    localparam S_START_PE       = 4'd4;
    localparam S_WAIT_PE        = 4'd5;
    localparam S_WRITE_C_DENSE  = 4'd6;
    localparam S_FINISH         = 4'd7;

    reg [3:0] state, state_next;

    // Cycle counter
    reg [15:0] cycle_cnt;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) cycle_cnt <= 0;
        else          cycle_cnt <= cycle_cnt + 1;
    end
    assign cycle_counter = cycle_cnt;

    //=========================================================================
    // Sub-module signals
    //=========================================================================
    wire load_done_desc, load_done_b, load_done_a;
    wire load_done = (state == S_LOAD_DESC) ? load_done_desc :
                     (state == S_LOAD_B)    ? load_done_b    :
                     (state == S_LOAD_A)    ? load_done_a    : 1'b0;

    wire [`N_PE-1:0] pe_done;
    wire pe_all_done;
    wire c_ddr_done;

    // FSM
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) state <= S_IDLE;
        else          state <= state_next;
    end
    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:          if (cr_start)               state_next = S_LOAD_DESC;
            S_LOAD_DESC:     if (load_done_desc)          state_next = S_LOAD_B;
            S_LOAD_B:        if (load_done_b)             state_next = S_LOAD_A;
            S_LOAD_A:        if (load_done_a)             state_next = S_START_PE;
            S_START_PE:                                    state_next = S_WAIT_PE;
            S_WAIT_PE:       if (pe_all_done)            state_next = S_WRITE_C_DENSE;
            S_WRITE_C_DENSE: if (c_ddr_done)              state_next = S_FINISH;
            S_FINISH: ;
            default: state_next = S_IDLE;
        endcase
    end
    assign cr_finish = (state == S_FINISH);
    assign cr_busy   = (state != S_IDLE) && (state != S_FINISH);

    // PE start pulse
    reg pe_started;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) pe_started <= 1'b0;
        else if (state != S_START_PE) pe_started <= 1'b0;
        else pe_started <= 1'b1;
    end
    wire pe_start_pulse = (state == S_START_PE) && !pe_started;

    //=========================================================================
    // AXI Read Mux
    //=========================================================================
    wire desc_arvalid, b_arvalid, a_arvalid;
    wire desc_arready, b_arready, a_arready;
    wire [`AXI_ADDR_WIDTH-1:0] desc_araddr, b_araddr, a_araddr;
    wire [7:0] desc_arlen, b_arlen, a_arlen;
    wire desc_rvalid, b_rvalid, a_rvalid;
    wire desc_rready, b_rready, a_rready;
    wire [`AXI_DATA_WIDTH-1:0] desc_rdata, b_rdata, a_rdata;
    wire desc_rlast, b_rlast, a_rlast;

    assign m_axi_arvalid = (state == S_LOAD_DESC) ? desc_arvalid :
                           (state == S_LOAD_B)    ? b_arvalid    :
                           (state == S_LOAD_A)    ? a_arvalid    : 1'b0;
    assign m_axi_araddr  = (state == S_LOAD_DESC) ? desc_araddr  :
                           (state == S_LOAD_B)    ? b_araddr     :
                           (state == S_LOAD_A)    ? a_araddr     : 0;
    assign m_axi_arlen   = (state == S_LOAD_DESC) ? desc_arlen   :
                           (state == S_LOAD_B)    ? b_arlen      :
                           (state == S_LOAD_A)    ? a_arlen      : 0;

    assign desc_arready = (state == S_LOAD_DESC) ? m_axi_arready : 1'b0;
    assign b_arready    = (state == S_LOAD_B)    ? m_axi_arready : 1'b0;
    assign a_arready    = (state == S_LOAD_A)    ? m_axi_arready : 1'b0;

    assign desc_rvalid = (state == S_LOAD_DESC) ? m_axi_rvalid : 1'b0;
    assign b_rvalid    = (state == S_LOAD_B)    ? m_axi_rvalid : 1'b0;
    assign a_rvalid    = (state == S_LOAD_A)    ? m_axi_rvalid : 1'b0;
    assign desc_rdata = m_axi_rdata; assign b_rdata = m_axi_rdata; assign a_rdata = m_axi_rdata;
    assign desc_rlast = m_axi_rlast; assign b_rlast = m_axi_rlast; assign a_rlast = m_axi_rlast;
    assign m_axi_rready = (state == S_LOAD_DESC) ? desc_rready :
                          (state == S_LOAD_B)    ? b_rready    :
                          (state == S_LOAD_A)    ? a_rready    : 1'b0;

    //=========================================================================
    // Descriptor Loader
    //=========================================================================
    descriptor_loader u_desc_loader (
        .start         (state == S_LOAD_DESC),
        .done          (load_done_desc),
        .M             (), .K (), .N (), .pe_valid_mask (),
        .m_axi_arvalid (desc_arvalid), .m_axi_arready (desc_arready),
        .m_axi_araddr  (desc_araddr),  .m_axi_arlen   (desc_arlen),
        .m_axi_rvalid  (desc_rvalid),  .m_axi_rready  (desc_rready),
        .m_axi_rdata   (desc_rdata),   .m_axi_rlast   (desc_rlast),
        .aclk          (aclk),          .aresetn       (aresetn)
    );

    //=========================================================================
    // B Broadcast Loader — writes col/val → b_shared_buffer,
    //                      writes desc → PEs (B_row_desc, per-PE LUT-RAM)
    //=========================================================================
    wire b_desc_we, b_col_we, b_val_we;
    wire [`B_ROW_ADDR_BITS-1:0] b_desc_waddr;
    wire [63:0] b_desc_wdata;
    wire [`B_NNZ_ADDR_BITS-1:0] b_col_waddr, b_val_waddr;
    wire [`DATA_WIDTH-1:0] b_col_wdata, b_val_wdata;

    b_broadcast_loader u_b_loader (
        .start          (state == S_LOAD_B),
        .done           (load_done_b),
        .pe_b_desc_we   (b_desc_we),
        .pe_b_desc_waddr(b_desc_waddr), .pe_b_desc_wdata(b_desc_wdata),
        .pe_b_col_we    (b_col_we),
        .pe_b_col_waddr (b_col_waddr),  .pe_b_col_wdata(b_col_wdata),
        .pe_b_val_we    (b_val_we),
        .pe_b_val_waddr (b_val_waddr),  .pe_b_val_wdata(b_val_wdata),
        .m_axi_arvalid  (b_arvalid), .m_axi_arready (b_arready),
        .m_axi_araddr   (b_araddr),  .m_axi_arlen   (b_arlen),
        .m_axi_rvalid   (b_rvalid),  .m_axi_rready  (b_rready),
        .m_axi_rdata    (b_rdata),   .m_axi_rlast   (b_rlast),
        .aclk           (aclk),       .aresetn       (aresetn)
    );

    //=========================================================================
    // A Group Loader → PE A buffers
    //=========================================================================
    wire a_desc_we, a_col_we, a_val_we;
    wire [`A_ROW_ADDR_BITS-1:0] a_desc_waddr;
    wire [63:0] a_desc_wdata;
    wire [`A_NNZ_ADDR_BITS-1:0] a_col_waddr, a_val_waddr;
    wire [`DATA_WIDTH-1:0] a_col_wdata, a_val_wdata;

    a_group_loader u_a_loader (
        .start         (state == S_LOAD_A),
        .done          (load_done_a),
        .pe_a_desc_we  (a_desc_we),
        .pe_a_desc_waddr(a_desc_waddr), .pe_a_desc_wdata(a_desc_wdata),
        .pe_a_col_we   (a_col_we),
        .pe_a_col_waddr(a_col_waddr),   .pe_a_col_wdata(a_col_wdata),
        .pe_a_val_we   (a_val_we),
        .pe_a_val_waddr(a_val_waddr),   .pe_a_val_wdata(a_val_wdata),
        .m_axi_arvalid (a_arvalid), .m_axi_arready (a_arready),
        .m_axi_araddr  (a_araddr),  .m_axi_arlen   (a_arlen),
        .m_axi_rvalid  (a_rvalid),  .m_axi_rready  (a_rready),
        .m_axi_rdata   (a_rdata),   .m_axi_rlast   (a_rlast),
        .aclk          (aclk),       .aresetn       (aresetn)
    );

    //=========================================================================
    // B Shared Buffer — 4 replicas, TDP dual-read
    //=========================================================================
    localparam B_GW = `B_NNZ_ADDR_BITS - 3;  // B group address width (13)
    localparam B_GW_BUS = `N_PE * B_GW;       // flat bus width (208)

    wire [`N_PE-1:0]                pe_b_req;
    wire [B_GW_BUS-1:0]            pe_b_group_flat;
    wire [`N_PE-1:0]                pe_b_rdy;
    wire [`N_PE*`DATA_WIDTH-1:0]   pe_bc0_flat, pe_bc1_flat;
    wire [`N_PE*`DATA_WIDTH-1:0]   pe_bc2_flat, pe_bc3_flat;
    wire [`N_PE*`DATA_WIDTH-1:0]   pe_bv0_flat, pe_bv1_flat;
    wire [`N_PE*`DATA_WIDTH-1:0]   pe_bv2_flat, pe_bv3_flat;

    b_shared_buffer u_b_shared (
        .b_col_we        (b_col_we),
        .b_col_waddr     (b_col_waddr),
        .b_col_wdata     (b_col_wdata),
        .b_val_we        (b_val_we),
        .b_val_waddr     (b_val_waddr),
        .b_val_wdata     (b_val_wdata),
        .pe_b_req        (pe_b_req),
        .pe_b_group_flat (pe_b_group_flat),
        .pe_b_rdy        (pe_b_rdy),
        .pe_bc0_flat     (pe_bc0_flat),
        .pe_bc1_flat     (pe_bc1_flat),
        .pe_bc2_flat     (pe_bc2_flat),
        .pe_bc3_flat     (pe_bc3_flat),
        .pe_bv0_flat     (pe_bv0_flat),
        .pe_bv1_flat     (pe_bv1_flat),
        .pe_bv2_flat     (pe_bv2_flat),
        .pe_bv3_flat     (pe_bv3_flat),
        .aclk            (aclk),
        .aresetn         (aresetn)
    );

    //=========================================================================
    // PE Array
    //=========================================================================
    wire [`N_PE-1:0] pe_done_int;
    wire [`N_PE-1:0] pe_cbuf_valid;
    wire [`N_PE-1:0] pe_cbuf_ready;
    wire [`N_PE*`C_DENSE_DEPTH_LOG-1:0] pe_cbuf_addr;
    wire [`N_PE*`DATA_WIDTH-1:0] pe_cbuf_data;

    genvar pe_idx;
    generate
        for (pe_idx = 0; pe_idx < `N_PE; pe_idx = pe_idx + 1) begin : gen_pe
            // wire pe_valid = pe_valid_mask[pe_idx];  // mask is 8-bit, N_PE=16
            wire pe_valid = 1'b1;  // TODO: expand mask or use per-group enable

            pe_top #(.PE_ID(pe_idx)) u_pe (
                .aclk(aclk), .aresetn(aresetn),
                .start         (pe_start_pulse && pe_valid),
                .row_count     (16'd16),  // TODO: from descriptor
                .done          (pe_done_int[pe_idx]),

                // A buffer load (per-PE, independent)
                .a_desc_we     (a_desc_we && pe_valid),
                .a_desc_waddr  (a_desc_waddr),
                .a_desc_wdata  (a_desc_wdata),
                .a_col_we      (a_col_we && pe_valid),
                .a_col_waddr   (a_col_waddr),
                .a_col_wdata   (a_col_wdata),
                .a_val_we      (a_val_we && pe_valid),
                .a_val_waddr   (a_val_waddr),
                .a_val_wdata   (a_val_wdata),

                // B_desc load (per-PE, small LUT-RAM)
                .b_desc_we     (b_desc_we && pe_valid),
                .b_desc_waddr  (b_desc_waddr),
                .b_desc_wdata  (b_desc_wdata),

                // External B data interface → b_shared_buffer
                .ext_b_req     (pe_b_req[pe_idx]),
                .ext_b_group   (pe_b_group_flat[pe_idx*B_GW +: B_GW]),
                .ext_b_rdy     (pe_b_rdy[pe_idx]),
                .ext_bc0       (pe_bc0_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bc1       (pe_bc1_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bc2       (pe_bc2_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bc3       (pe_bc3_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bv0       (pe_bv0_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bv1       (pe_bv1_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bv2       (pe_bv2_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),
                .ext_bv3       (pe_bv3_flat[pe_idx*`DATA_WIDTH +: `DATA_WIDTH]),

                // C buffer write handshake
                .cbuf_wr_valid (pe_cbuf_valid[pe_idx]),
                .cbuf_wr_ready (pe_cbuf_ready[pe_idx]),
                .cbuf_wr_addr  (pe_cbuf_addr[pe_idx*`C_DENSE_DEPTH_LOG +: `C_DENSE_DEPTH_LOG]),
                .cbuf_wr_data  (pe_cbuf_data[pe_idx*`DATA_WIDTH +: `DATA_WIDTH])
            );
        end
    endgenerate

    // PE done masking (N_PE up to 16, mask is 8-bit — extended with 1'b0)
    wire [`N_PE-1:0] pe_done_masked;
    generate
        for (pe_idx = 0; pe_idx < `N_PE; pe_idx = pe_idx + 1) begin : gen_done
            assign pe_done_masked[pe_idx] = pe_done_int[pe_idx];
        end
    endgenerate
    assign pe_done = pe_done_int;
    assign pe_all_done = &pe_done_masked;

    //=========================================================================
    // C_dense_write_arbiter → C_dense_buffer
    //=========================================================================
    wire cbuf_wr_en;
    wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr;
    wire [`DATA_WIDTH-1:0] cbuf_wr_data;
    wire cbuf_rd_en;
    wire [`C_DENSE_DEPTH_LOG-1:0] cbuf_rd_addr;
    wire [`AXI_DATA_WIDTH-1:0] cbuf_rd_data;
    wire cbuf_rd_valid;

    c_dense_write_arbiter u_c_arbiter (
        .pe_cbuf_valid (pe_cbuf_valid),
        .pe_cbuf_ready (pe_cbuf_ready),
        .pe_cbuf_addr  (pe_cbuf_addr),
        .pe_cbuf_data  (pe_cbuf_data),
        .cbuf_wr_en    (cbuf_wr_en),
        .cbuf_wr_addr  (cbuf_wr_addr),
        .cbuf_wr_data  (cbuf_wr_data),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // C_dense_buffer (32-bank BRAM)
    c_dense_buffer u_c_buffer (
        .wr_en    (cbuf_wr_en),
        .wr_addr  (cbuf_wr_addr),
        .wr_data  (cbuf_wr_data),
        .rd_en    (cbuf_rd_en),
        .rd_addr  (cbuf_rd_addr),
        .rd_data  (cbuf_rd_data),
        .rd_valid (cbuf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    // C_dense_DDR_writer
    c_dense_ddr_writer u_c_ddr_writer (
        .start         (state == S_WRITE_C_DENSE),
        .done          (c_ddr_done),
        .M(M), .N(N),
        .cbuf_rd_en    (cbuf_rd_en),
        .cbuf_rd_addr  (cbuf_rd_addr),
        .cbuf_rd_data  (cbuf_rd_data),
        .cbuf_rd_valid (cbuf_rd_valid),
        .m_axi_awvalid (m_axi_awvalid), .m_axi_awready (m_axi_awready),
        .m_axi_awaddr  (m_axi_awaddr),  .m_axi_awlen   (m_axi_awlen),
        .m_axi_wvalid  (m_axi_wvalid),  .m_axi_wready  (m_axi_wready),
        .m_axi_wdata   (m_axi_wdata),   .m_axi_wstrb   (m_axi_wstrb),
        .m_axi_wlast   (m_axi_wlast),   .m_axi_bvalid  (m_axi_bvalid),
        .m_axi_bready  (m_axi_bready),  .m_axi_bresp   (m_axi_bresp),
        .aclk          (aclk),           .aresetn       (aresetn)
    );

endmodule
