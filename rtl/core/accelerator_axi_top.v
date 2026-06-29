//=============================================================================
// File     : accelerator_axi_top.v
// Brief    : AXI-direct accelerator top.  DDR simulated by cocotb mmap+AXI4Slave.
//            axi_loader reads via AXI4 read channel, writes directly to PE ports.
//
//   FSM:  S_IDLE → S_LOAD → S_COMPUTE → S_DONE
//
//   No BRAM DDR model — raw AXI bus exposed to cocotb via tb_accelerator_axi.
//=============================================================================

`include "defines.vh"

module accelerator_axi_top #(
    parameter N_PE  = `N_PE,
    parameter M_AW  = `MAX_DIM_BITS,
    parameter C_AW  = 18
) (
    input  wire clk,
    input  wire rst_n,

    // === Control ===
    input  wire                    start,
    output wire                    done,
    input  wire [M_AW-1:0]        M, K, N,
    input  wire                    op_mode, op_sub,

    // === AXI4 Read Address Channel (exposed for AXI4Slave) ===
    output wire [3:0]              ddr_ARID,
    output wire [63:0]             ddr_ARADDR,
    output wire [7:0]              ddr_ARLEN,
    output wire                    ddr_ARVALID,
    input  wire                    ddr_ARREADY,

    // === AXI4 Read Data Channel ===
    input  wire [3:0]              ddr_RID,
    input  wire [511:0]            ddr_RDATA,
    input  wire [1:0]              ddr_RRESP,
    input  wire                    ddr_RLAST,
    input  wire                    ddr_RVALID,
    output wire                    ddr_RREADY,

    // === PE C read ports (for cocotb drain) ===
    output wire [N_PE-1:0]                           c_rd_en,
    output wire [N_PE*(`C_ROW_ADDR_BITS+5)-1:0]     c_rd_addr,
    input  wire [N_PE*16*16-1:0]                     c_rd_data,
    input  wire [N_PE*`MAX_DIM_BITS-1:0]             c_rd_row
);

    //=========================================================================
    // Top-level FSM
    //=========================================================================
    localparam S_IDLE     = 2'd0;
    localparam S_LOAD     = 2'd1;
    localparam S_COMPUTE  = 2'd2;
    localparam S_DONE     = 2'd3;

    reg [1:0] state, state_next;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE; else state <= state_next;

    wire load_done, cluster_done;
    reg  prev_load, prev_comp;
    always @(posedge clk) begin prev_load<=(state==S_LOAD); prev_comp<=(state==S_COMPUTE); end
    wire load_rise = (state==S_LOAD)    && !prev_load;
    wire comp_rise = (state==S_COMPUTE) && !prev_comp;

    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:    if (start)        state_next = S_LOAD;
            S_LOAD:    if (load_done)    state_next = S_COMPUTE;
            S_COMPUTE: if (cluster_done) state_next = S_DONE;
            S_DONE:    state_next = S_DONE;
            default:   state_next = S_IDLE;
        endcase
    end
    assign done = (state == S_DONE);

    //=========================================================================
    // AXI Loader
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
        .clk(clk), .rst_n(rst_n),
        .start(load_rise), .done(load_done),
        .M(M), .K(K), .N(N),
        .axi_arid  (ddr_ARID),   .axi_araddr(ddr_ARADDR), .axi_arlen(ddr_ARLEN),
        .axi_arvalid(ddr_ARVALID), .axi_arready(ddr_ARREADY),
        .axi_rid   (ddr_RID),    .axi_rdata(ddr_RDATA),   .axi_rresp(ddr_RRESP),
        .axi_rlast (ddr_RLAST),  .axi_rvalid(ddr_RVALID), .axi_rready(ddr_RREADY),
        .pe_a_desc_we   (pe_a_desc_we),   .pe_a_desc_waddr(pe_a_desc_waddr), .pe_a_desc_wdata(pe_a_desc_wdata),
        .pe_a_val_we    (pe_a_val_we),    .pe_a_val_waddr (pe_a_val_waddr),  .pe_a_val_wdata (pe_a_val_wdata),
        .pe_a_col_we    (pe_a_col_we),    .pe_a_col_waddr (pe_a_col_waddr),  .pe_a_col_wdata (pe_a_col_wdata),
        .pe_b_desc_we  (pe_b_desc_we),   .pe_b_desc_waddr(pe_b_desc_waddr), .pe_b_desc_wdata(pe_b_desc_wdata),
        .pe_b_col_we   (pe_b_col_we),    .pe_b_col_waddr (pe_b_col_waddr),  .pe_b_col_wdata (pe_b_col_wdata),
        .pe_b_val_we   (pe_b_val_we),    .pe_b_val_waddr (pe_b_val_waddr),  .pe_b_val_wdata (pe_b_val_wdata),
        .pe_row_counts(pe_row_counts)
    );

    //=========================================================================
    // PE Cluster
    //=========================================================================
    reg [N_PE*16-1:0] cluster_row_count;
    always @(posedge clk) if (comp_rise) cluster_row_count <= pe_row_counts;

    pe_cluster #(.N_PE(N_PE)) u_cluster (
        .aclk(clk), .aresetn(rst_n),
        .start(comp_rise), .row_count(cluster_row_count), .done(cluster_done),
        .M(M), .K(K), .N(N), .op_mode(op_mode), .op_sub(op_sub),
        .a_desc_we   (pe_a_desc_we),   .a_desc_waddr(pe_a_desc_waddr), .a_desc_wdata(pe_a_desc_wdata),
        .a_val_we    (pe_a_val_we),    .a_val_waddr (pe_a_val_waddr),  .a_val_wdata (pe_a_val_wdata),
        .a_col_we    (pe_a_col_we),    .a_col_waddr (pe_a_col_waddr),  .a_col_wdata (pe_a_col_wdata),
        .b_desc_we  (pe_b_desc_we),   .b_desc_waddr(pe_b_desc_waddr), .b_desc_wdata(pe_b_desc_wdata),
        .b_col_we   (pe_b_col_we),    .b_col_waddr (pe_b_col_waddr),  .b_col_wdata (pe_b_col_wdata),
        .b_val_we   (pe_b_val_we),    .b_val_waddr (pe_b_val_waddr),  .b_val_wdata (pe_b_val_wdata),
        .c_rd_en(c_rd_en), .c_rd_addr(c_rd_addr), .c_rd_data(c_rd_data), .c_rd_row(c_rd_row)
    );

endmodule
