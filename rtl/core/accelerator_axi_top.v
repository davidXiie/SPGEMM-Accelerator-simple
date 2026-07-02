//=============================================================================
// accelerator_axi_top.v — AXI-direct accelerator, 6-state FSM.
//   S_IDLE → S_LOAD → S_COMPUTE → S_DRAIN → S_DONE
//
// axi_loader reads A+B from DDR via AXI4 Read → PE cluster computes →
// axi_c_drain writes C back to DDR via AXI4 Write.
//=============================================================================

`include "defines.vh"

module accelerator_axi_top #(
    parameter N_PE  = `N_PE,
    parameter M_AW  = `MAX_DIM_BITS
) (
    input  wire clk, rst_n,
    input  wire start, output wire done,
    input  wire [M_AW-1:0] M, K, N,
    input  wire op_mode, op_sub,

    // AXI4 Read — AR channel
    output wire [3:0]   ddr_ARID,
    output wire [63:0]  ddr_ARADDR,
    output wire [7:0]   ddr_ARLEN,
    output wire [2:0]   ddr_ARSIZE,
    output wire         ddr_ARVALID,
    input  wire         ddr_ARREADY,
    // AXI4 Read — R channel
    input  wire [3:0]   ddr_RID,
    input  wire [511:0] ddr_RDATA,
    input  wire [1:0]   ddr_RRESP,
    input  wire         ddr_RLAST,
    input  wire         ddr_RVALID,
    output wire         ddr_RREADY,

    // AXI4 Write — AW channel
    output wire [3:0]   ddr_AWID,
    output wire [63:0]  ddr_AWADDR,
    output wire [7:0]   ddr_AWLEN,
    output wire [2:0]   ddr_AWSIZE,
    output wire         ddr_AWVALID,
    input  wire         ddr_AWREADY,
    // AXI4 Write — W channel
    output wire [511:0] ddr_WDATA,
    output wire [63:0]  ddr_WSTRB,
    output wire         ddr_WLAST,
    output wire         ddr_WVALID,
    input  wire         ddr_WREADY,
    // AXI4 Write — B channel
    input  wire [3:0]   ddr_BID,
    input  wire [1:0]   ddr_BRESP,
    input  wire         ddr_BVALID,
    output wire         ddr_BREADY,

    // PE C read (for cocotb debug access when not draining)
    input  wire [N_PE-1:0]                       c_rd_en,
    input  wire [N_PE*(`C_ROW_ADDR_BITS+5)-1:0] c_rd_addr,
    output wire [N_PE*16*16-1:0]                  c_rd_data,
    output wire [N_PE*`MAX_DIM_BITS-1:0]          c_rd_row
);

    //=========================================================================
    // FSM: IDLE → LOAD → COMPUTE → DRAIN → DONE
    //=========================================================================
    localparam S_IDLE       = 3'd0;
    localparam S_LOAD       = 3'd1;
    localparam S_COMPUTE    = 3'd2;
    localparam S_WAIT_DRAIN = 3'd3;
    localparam S_DRAIN      = 3'd4;
    localparam S_DONE       = 3'd5;

    reg [2:0] state, state_next;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE; else state <= state_next;

    wire load_done, cluster_done, drain_done;
    reg  prev_load, prev_comp, prev_drain;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_load  <= 1'b0;
            prev_comp  <= 1'b0;
            prev_drain <= 1'b0;
        end else begin
            prev_load  <= (state == S_LOAD);
            prev_comp  <= (state == S_COMPUTE);
            prev_drain <= (state == S_DRAIN);
        end
    end
    wire load_rise  = (state == S_LOAD)    && !prev_load;
    wire comp_rise  = (state == S_COMPUTE) && !prev_comp;
    wire drain_rise = (state == S_DRAIN)   && !prev_drain;

    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:       if (start)        state_next = S_LOAD;
            S_LOAD:       if (load_done)    state_next = S_COMPUTE;
            S_COMPUTE:    if (cluster_done) state_next = S_WAIT_DRAIN;
            S_WAIT_DRAIN:                   state_next = S_DRAIN;
            S_DRAIN:      if (drain_done)   state_next = S_DONE;
            S_DONE:       state_next = S_DONE;
            default:      state_next = S_IDLE;
        endcase
    end
    assign done = (state == S_DONE);

    //=========================================================================
    // AXI Loader → PE cluster
    //=========================================================================
    wire [N_PE-1:0]                    pe_a_desc_we;
    wire [N_PE*`A_ROW_ADDR_BITS-1:0]  pe_a_desc_waddr;
    wire [N_PE*36-1:0]                 pe_a_desc_wdata;
    wire [N_PE-1:0]                    pe_a_val_we;
    wire [N_PE*`A_NNZ_ADDR_BITS-1:0]  pe_a_val_waddr;
    wire [N_PE*`DATA_WIDTH-1:0]        pe_a_val_wdata;
    wire [N_PE-1:0]                    pe_a_col_we;
    wire [N_PE*`A_NNZ_ADDR_BITS-1:0]  pe_a_col_waddr;
    wire [N_PE*`DATA_WIDTH-1:0]        pe_a_col_wdata;
    wire                               pe_b_desc_we;
    wire [`B_ROW_ADDR_BITS-1:0]       pe_b_desc_waddr;
    wire [31:0]                        pe_b_desc_wdata;
    wire                               pe_b_col_we;
    wire [`B_NNZ_ADDR_BITS-1:0]       pe_b_col_waddr;
    wire [`DATA_WIDTH-1:0]             pe_b_col_wdata;
    wire                               pe_b_val_we;
    wire [`B_NNZ_ADDR_BITS-1:0]       pe_b_val_waddr;
    wire [`DATA_WIDTH-1:0]             pe_b_val_wdata;
    wire [N_PE*16-1:0]                 pe_row_counts;

    axi_loader #(.N_PE(N_PE)) u_loader (
        .clk(clk), .rst_n(rst_n), .start(load_rise), .done(load_done),
        .M(M), .K(K), .N(N),
        .axi_arid(ddr_ARID), .axi_araddr(ddr_ARADDR), .axi_arlen(ddr_ARLEN),
        .axi_arsize(ddr_ARSIZE), .axi_arvalid(ddr_ARVALID), .axi_arready(ddr_ARREADY),
        .axi_rid(ddr_RID), .axi_rdata(ddr_RDATA), .axi_rresp(ddr_RRESP),
        .axi_rlast(ddr_RLAST), .axi_rvalid(ddr_RVALID), .axi_rready(ddr_RREADY),
        .pe_a_desc_we(pe_a_desc_we), .pe_a_desc_waddr(pe_a_desc_waddr),
        .pe_a_desc_wdata(pe_a_desc_wdata),
        .pe_a_val_we(pe_a_val_we), .pe_a_val_waddr(pe_a_val_waddr),
        .pe_a_val_wdata(pe_a_val_wdata),
        .pe_a_col_we(pe_a_col_we), .pe_a_col_waddr(pe_a_col_waddr),
        .pe_a_col_wdata(pe_a_col_wdata),
        .pe_b_desc_we(pe_b_desc_we), .pe_b_desc_waddr(pe_b_desc_waddr), .pe_b_desc_wdata(pe_b_desc_wdata),
        .pe_b_col_we(pe_b_col_we),   .pe_b_col_waddr(pe_b_col_waddr),   .pe_b_col_wdata(pe_b_col_wdata),
        .pe_b_val_we(pe_b_val_we),   .pe_b_val_waddr(pe_b_val_waddr),   .pe_b_val_wdata(pe_b_val_wdata),
        .pe_row_counts(pe_row_counts)
    );

    //=========================================================================
    // PE Cluster — C read port mux between Python (sim) and axi_c_drain
    //=========================================================================
    // Wire pe_row_counts directly — it is stable well before comp_rise fires.
    wire [N_PE*16-1:0] cluster_row_count = pe_row_counts;

    // C read port: during DRAIN the axi_c_drain drives the read port;
    // at other times the external port (Python in sim) has access.
    localparam C_RD_ADDR_W = `C_ROW_ADDR_BITS + 5;
    wire [N_PE-1:0]               drain_c_rd_en;
    wire [N_PE*C_RD_ADDR_W-1:0]   drain_c_rd_addr;
    wire [N_PE-1:0]               pe_c_rd_en_w;
    wire [N_PE*C_RD_ADDR_W-1:0]   pe_c_rd_addr_w;
    assign pe_c_rd_en_w   = (state == S_DRAIN) ? drain_c_rd_en   : c_rd_en;
    assign pe_c_rd_addr_w = (state == S_DRAIN) ? drain_c_rd_addr : c_rd_addr;

    pe_cluster #(.N_PE(N_PE)) u_cluster (
        .aclk(clk), .aresetn(rst_n),
        .start(comp_rise), .row_count(cluster_row_count), .done(cluster_done),
        .M(M), .K(K), .N(N), .op_mode(op_mode), .op_sub(op_sub),
        .a_desc_we(pe_a_desc_we), .a_desc_waddr(pe_a_desc_waddr),
        .a_desc_wdata(pe_a_desc_wdata),
        .a_val_we(pe_a_val_we), .a_val_waddr(pe_a_val_waddr),
        .a_val_wdata(pe_a_val_wdata),
        .a_col_we(pe_a_col_we), .a_col_waddr(pe_a_col_waddr),
        .a_col_wdata(pe_a_col_wdata),
        .b_desc_we(pe_b_desc_we), .b_desc_waddr(pe_b_desc_waddr), .b_desc_wdata(pe_b_desc_wdata),
        .b_col_we(pe_b_col_we),   .b_col_waddr(pe_b_col_waddr),   .b_col_wdata(pe_b_col_wdata),
        .b_val_we(pe_b_val_we),   .b_val_waddr(pe_b_val_waddr),   .b_val_wdata(pe_b_val_wdata),
        .c_rd_en(pe_c_rd_en_w), .c_rd_addr(pe_c_rd_addr_w),
        .c_rd_data(c_rd_data), .c_rd_row(c_rd_row)
    );

    //=========================================================================
    // AXI C Drain — writes C results back to DDR via AXI4 Write
    //=========================================================================
    axi_c_drain #(
        .N_PE(N_PE),
        .MAX_DIM_BITS(`MAX_DIM_BITS),
        .C_ROW_ADDR_BITS(`C_ROW_ADDR_BITS)
    ) u_c_drain (
        .clk(clk), .rst_n(rst_n),
        .start(drain_rise), .done(drain_done),
        .M(M), .N(N),
        .row_counts(cluster_row_count),
        .c_rd_en  (drain_c_rd_en),
        .c_rd_addr(drain_c_rd_addr),
        .c_rd_data(c_rd_data),
        .c_rd_row (c_rd_row),
        .axi_awid   (ddr_AWID),
        .axi_awaddr (ddr_AWADDR),
        .axi_awlen  (ddr_AWLEN),
        .axi_awsize (ddr_AWSIZE),
        .axi_awvalid(ddr_AWVALID),
        .axi_awready(ddr_AWREADY),
        .axi_wdata  (ddr_WDATA),
        .axi_wstrb  (ddr_WSTRB),
        .axi_wlast  (ddr_WLAST),
        .axi_wvalid (ddr_WVALID),
        .axi_wready (ddr_WREADY),
        .axi_bid    (ddr_BID),
        .axi_bresp  (ddr_BRESP),
        .axi_bvalid (ddr_BVALID),
        .axi_bready (ddr_BREADY)
    );

endmodule
