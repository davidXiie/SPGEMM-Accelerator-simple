//=============================================================================
// File     : accelerator_top.v
// Brief    : Complete SpGEMM accelerator built around pe_cluster.
//
//   Architecture:
//     ┌──────────────────────────────────────────────────────────┐
//     │  accelerator_top                                        │
//     │  ┌──────────┐  ┌──────────┐  ┌──────────┐              │
//     │  │ a_global │  │ b_global │  │ c_global │              │
//     │  │ _buffer  │  │ _buffer  │  │ _buffer  │              │
//     │  └────┬─────┘  └────┬─────┘  └────▲─────┘              │
//     │       │read          │read         │write               │
//     │  ┌────▼──────────────▼─────┐  ┌────┴────────┐          │
//     │  │    pe_load_ctrl         │  │ pe_drain    │          │
//     │  │ (A partition + B bcast) │  │ _ctrl       │          │
//     │  └────────┬────────────────┘  └────▲────────┘          │
//     │           │ A/B write ports       │ C read ports       │
//     │  ┌────────▼────────────────────────┴──────────┐       │
//     │  │           pe_cluster (N_PE=3)               │       │
//     │  │  ┌──────┐  ┌──────┐  ┌──────┐             │       │
//     │  │  │ PE0  │  │ PE1  │  │ PE2  │             │       │
//     │  │  └──────┘  └──────┘  └──────┘             │       │
//     │  └────────────────────────────────────────────┘       │
//     └──────────────────────────────────────────────────────┘
//
//   Execution flow (5-state FSM):
//     S_IDLE → S_LOAD_A → S_LOAD_B → S_COMPUTE → S_DRAIN → S_DONE
//=============================================================================

`include "defines.vh"

module accelerator_top #(
    parameter N_PE            = `N_PE,
    parameter M_AW            = `MAX_DIM_BITS,       // log2(512) = 10
    parameter A_DESC_DEPTH    = `MAX_M,               // 512
    parameter A_NNZ_DEPTH     = `N_PE * `A_NNZ_SLOT_PER_PE,  // 86016
    parameter A_NNZ_AW        = 17,                   // ceil(log2(86016))
    parameter A_DESC_AW       = 10,                   // ceil(log2(512))
    parameter B_DESC_DEPTH    = `MAX_K,               // 512
    parameter B_NNZ_DEPTH     = `B_NNZ_SLOT,          // 40960
    parameter C_AW             = `C_DENSE_DEPTH_LOG    // 18
) (
    input  wire clk,
    input  wire rst_n,

    // === Control (host interface) ===
    input  wire                    start,
    output wire                    done,

    input  wire [M_AW-1:0]        M,
    input  wire [M_AW-1:0]        K,
    input  wire [M_AW-1:0]        N,
    input  wire                    op_mode,          // 0=SpGEMM, 1=elementwise
    input  wire                    op_sub,           // 0=add, 1=sub (elementwise)

    // === A Global Buffer host write ports ===
    input  wire                    a_host_desc_wr_en,
    input  wire [A_DESC_AW-1:0]   a_host_desc_wr_addr,
    input  wire [63:0]            a_host_desc_wr_data,
    input  wire                    a_host_col_wr_en,
    input  wire [A_NNZ_AW-1:0]    a_host_col_wr_addr,
    input  wire [15:0]            a_host_col_wr_data,
    input  wire                    a_host_val_wr_en,
    input  wire [A_NNZ_AW-1:0]    a_host_val_wr_addr,
    input  wire [15:0]            a_host_val_wr_data,

    // === B Global Buffer host write ports ===
    input  wire                    b_host_desc_wr_en,
    input  wire [`B_ROW_ADDR_BITS-1:0] b_host_desc_wr_addr,
    input  wire [31:0]            b_host_desc_wr_data,
    input  wire                    b_host_col_wr_en,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_host_col_wr_addr,
    input  wire [15:0]            b_host_col_wr_data,
    input  wire                    b_host_val_wr_en,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_host_val_wr_addr,
    input  wire [15:0]            b_host_val_wr_data,

    // === C Global Buffer host read port ===
    input  wire [C_AW-1:0]        c_host_rd_addr,
    output wire [15:0]            c_host_rd_data
);

    //=========================================================================
    // Top-level FSM
    //=========================================================================
    localparam S_IDLE     = 3'd0;
    localparam S_LOAD_A   = 3'd1;
    localparam S_LOAD_B   = 3'd2;
    localparam S_COMPUTE  = 3'd3;
    localparam S_DRAIN    = 3'd4;
    localparam S_DONE     = 3'd5;

    reg [2:0] state, state_next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    reg  accel_start;     // internal start pulses
    reg  load_a_start;
    reg  load_b_start;
    reg  drain_start;
    wire load_a_done;
    wire load_b_done;
    wire load_all_done;
    wire cluster_done;
    wire drain_done;
    reg  comp_start_r;

    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:    if (start) state_next = S_LOAD_A;
            S_LOAD_A:  if (load_a_done) state_next = S_LOAD_B;
            S_LOAD_B:  if (load_b_done) state_next = S_COMPUTE;
            S_COMPUTE: if (cluster_done) state_next = S_DRAIN;
            S_DRAIN:   if (drain_done) state_next = S_DONE;
            S_DONE:    state_next = S_DONE;
            default:   state_next = S_IDLE;
        endcase
    end

    // Generate internal start pulses (1 cycle)
    reg prev_s_load_a, prev_s_load_b, prev_s_compute, prev_s_drain;
    always @(posedge clk) begin
        prev_s_load_a  <= (state == S_LOAD_A);
        prev_s_load_b  <= (state == S_LOAD_B);
        prev_s_compute <= (state == S_COMPUTE);
        prev_s_drain   <= (state == S_DRAIN);
    end
    wire s_load_a_rise  = (state == S_LOAD_A)  && !prev_s_load_a;
    wire s_load_b_rise  = (state == S_LOAD_B)  && !prev_s_load_b;
    wire s_compute_rise = (state == S_COMPUTE) && !prev_s_compute;
    wire s_drain_rise   = (state == S_DRAIN)   && !prev_s_drain;

    assign done = (state == S_DONE);

    //=========================================================================
    // A Global Buffer
    //=========================================================================
    wire        a_gbuf_desc_en;
    wire [9:0]  a_gbuf_desc_addr;   // A_DESC_AW-1 = 9
    wire [63:0] a_gbuf_desc_data;
    wire        a_gbuf_col_en;
    wire [16:0] a_gbuf_col_addr;    // A_NNZ_AW-1 = 16
    wire [15:0] a_gbuf_col_data;
    wire        a_gbuf_val_en;
    wire [16:0] a_gbuf_val_addr;
    wire [15:0] a_gbuf_val_data;

    a_global_buffer #(
        .DESC_DEPTH(A_DESC_DEPTH),
        .NNZ_DEPTH (A_NNZ_DEPTH),
        .DESC_AW   (10),
        .NNZ_AW    (17)
    ) u_a_global (
        .clk(clk), .rst_n(rst_n),
        .host_desc_wr_en  (a_host_desc_wr_en),
        .host_desc_wr_addr(a_host_desc_wr_addr),
        .host_desc_wr_data(a_host_desc_wr_data),
        .host_col_wr_en   (a_host_col_wr_en),
        .host_col_wr_addr (a_host_col_wr_addr),
        .host_col_wr_data (a_host_col_wr_data),
        .host_val_wr_en   (a_host_val_wr_en),
        .host_val_wr_addr (a_host_val_wr_addr),
        .host_val_wr_data (a_host_val_wr_data),
        .rd_desc_en (a_gbuf_desc_en),
        .rd_desc_addr(a_gbuf_desc_addr),
        .rd_desc_data(a_gbuf_desc_data),
        .rd_col_en  (a_gbuf_col_en),
        .rd_col_addr(a_gbuf_col_addr),
        .rd_col_data(a_gbuf_col_data),
        .rd_val_en  (a_gbuf_val_en),
        .rd_val_addr(a_gbuf_val_addr),
        .rd_val_data(a_gbuf_val_data)
    );

    //=========================================================================
    // B Global Buffer
    //=========================================================================
    wire        b_gbuf_desc_en;
    wire [`B_ROW_ADDR_BITS-1:0] b_gbuf_desc_addr;
    wire [31:0] b_gbuf_desc_data;
    wire        b_gbuf_col_en;
    wire [`B_NNZ_ADDR_BITS-1:0] b_gbuf_col_addr;
    wire [15:0] b_gbuf_col_data;
    wire        b_gbuf_val_en;
    wire [`B_NNZ_ADDR_BITS-1:0] b_gbuf_val_addr;
    wire [15:0] b_gbuf_val_data;

    b_global_buffer #(
        .DESC_DEPTH(B_DESC_DEPTH),
        .NNZ_DEPTH (B_NNZ_DEPTH)
    ) u_b_global (
        .clk(clk), .rst_n(rst_n),
        .host_desc_wr_en  (b_host_desc_wr_en),
        .host_desc_wr_addr(b_host_desc_wr_addr),
        .host_desc_wr_data(b_host_desc_wr_data),
        .host_col_wr_en   (b_host_col_wr_en),
        .host_col_wr_addr (b_host_col_wr_addr),
        .host_col_wr_data (b_host_col_wr_data),
        .host_val_wr_en   (b_host_val_wr_en),
        .host_val_wr_addr (b_host_val_wr_addr),
        .host_val_wr_data (b_host_val_wr_data),
        .rd_desc_en (b_gbuf_desc_en),
        .rd_desc_addr(b_gbuf_desc_addr),
        .rd_desc_data(b_gbuf_desc_data),
        .rd_col_en  (b_gbuf_col_en),
        .rd_col_addr(b_gbuf_col_addr),
        .rd_col_data(b_gbuf_col_data),
        .rd_val_en  (b_gbuf_val_en),
        .rd_val_addr(b_gbuf_val_addr),
        .rd_val_data(b_gbuf_val_data)
    );

    //=========================================================================
    // PE Load Controller
    //=========================================================================
    wire [N_PE-1:0]                    pe_a_desc_valid;
    wire [N_PE-1:0]                    pe_a_desc_ready;
    wire [N_PE*36-1:0]                 pe_a_desc_data;
    wire [N_PE-1:0]                    pe_a_val_we;
    wire [N_PE*`A_NNZ_ADDR_BITS-1:0]  pe_a_val_waddr;
    wire [N_PE*`DATA_WIDTH-1:0]        pe_a_val_wdata;
    wire [N_PE-1:0]                    pe_a_col_we;
    wire [N_PE*`A_NNZ_ADDR_BITS-1:0]  pe_a_col_waddr;
    wire [N_PE*`DATA_WIDTH-1:0]        pe_a_col_wdata;

    wire                               pe_b_col_we;
    wire [`B_NNZ_ADDR_BITS-1:0]       pe_b_col_waddr;
    wire [`DATA_WIDTH-1:0]             pe_b_col_wdata;
    wire                               pe_b_val_we;
    wire [`B_NNZ_ADDR_BITS-1:0]       pe_b_val_waddr;
    wire [`DATA_WIDTH-1:0]             pe_b_val_wdata;
    wire                               pe_b_desc_we;
    wire [`B_ROW_ADDR_BITS-1:0]       pe_b_desc_waddr;
    wire [31:0]                        pe_b_desc_wdata;

    wire [N_PE*16-1:0]                pe_row_counts;

    pe_load_ctrl #(
        .N_PE(N_PE)
    ) u_load (
        .clk(clk), .rst_n(rst_n),
        .start(s_load_a_rise),
        .a_done (load_a_done),
        .b_done (load_b_done),
        .all_done(load_all_done),
        .M(M), .K(K), .N(N),

        .a_gbuf_desc_en  (a_gbuf_desc_en),
        .a_gbuf_desc_addr(a_gbuf_desc_addr),
        .a_gbuf_desc_data(a_gbuf_desc_data),
        .a_gbuf_col_en   (a_gbuf_col_en),
        .a_gbuf_col_addr (a_gbuf_col_addr),
        .a_gbuf_col_data (a_gbuf_col_data),
        .a_gbuf_val_en   (a_gbuf_val_en),
        .a_gbuf_val_addr (a_gbuf_val_addr),
        .a_gbuf_val_data (a_gbuf_val_data),

        .b_gbuf_desc_en  (b_gbuf_desc_en),
        .b_gbuf_desc_addr(b_gbuf_desc_addr),
        .b_gbuf_desc_data(b_gbuf_desc_data),
        .b_gbuf_col_en   (b_gbuf_col_en),
        .b_gbuf_col_addr (b_gbuf_col_addr),
        .b_gbuf_col_data (b_gbuf_col_data),
        .b_gbuf_val_en   (b_gbuf_val_en),
        .b_gbuf_val_addr (b_gbuf_val_addr),
        .b_gbuf_val_data (b_gbuf_val_data),

        .pe_a_desc_valid(pe_a_desc_valid),
        .pe_a_desc_ready(pe_a_desc_ready),
        .pe_a_desc_data (pe_a_desc_data),
        .pe_a_val_we    (pe_a_val_we),
        .pe_a_val_waddr (pe_a_val_waddr),
        .pe_a_val_wdata (pe_a_val_wdata),
        .pe_a_col_we    (pe_a_col_we),
        .pe_a_col_waddr (pe_a_col_waddr),
        .pe_a_col_wdata (pe_a_col_wdata),

        .pe_b_col_we   (pe_b_col_we),
        .pe_b_col_waddr(pe_b_col_waddr),
        .pe_b_col_wdata(pe_b_col_wdata),
        .pe_b_val_we   (pe_b_val_we),
        .pe_b_val_waddr(pe_b_val_waddr),
        .pe_b_val_wdata(pe_b_val_wdata),
        .pe_b_desc_we   (pe_b_desc_we),
        .pe_b_desc_waddr(pe_b_desc_waddr),
        .pe_b_desc_wdata(pe_b_desc_wdata),

        .pe_row_counts(pe_row_counts)
    );

    //=========================================================================
    // PE Cluster
    //=========================================================================
    reg        cluster_start;
    reg [N_PE*16-1:0] cluster_row_count;

    // Compute start: one-cycle pulse
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cluster_start <= 1'b0;
        end else begin
            cluster_start <= s_compute_rise;
            if (s_compute_rise) begin
                cluster_row_count <= pe_row_counts;
            end
        end
    end

    wire [N_PE-1:0]                    c_rd_en;
    wire [N_PE*(`C_ROW_ADDR_BITS+5)-1:0] c_rd_addr;
    wire [N_PE*16*16-1:0]              c_rd_data;
    wire [N_PE*`MAX_DIM_BITS-1:0]      c_rd_row;

    pe_cluster #(.N_PE(N_PE)) u_cluster (
        .aclk(clk), .aresetn(rst_n),
        .start(cluster_start),
        .row_count(cluster_row_count),
        .done(cluster_done),
        .M(M), .K(K), .N(N),
        .op_mode(op_mode), .op_sub(op_sub),

        .a_desc_valid(pe_a_desc_valid),
        .a_desc_ready(pe_a_desc_ready),
        .a_desc_data (pe_a_desc_data),
        .a_val_we    (pe_a_val_we),
        .a_val_waddr (pe_a_val_waddr),
        .a_val_wdata (pe_a_val_wdata),
        .a_col_we    (pe_a_col_we),
        .a_col_waddr (pe_a_col_waddr),
        .a_col_wdata (pe_a_col_wdata),

        .b_col_we   (pe_b_col_we),
        .b_col_waddr(pe_b_col_waddr),
        .b_col_wdata(pe_b_col_wdata),
        .b_val_we   (pe_b_val_we),
        .b_val_waddr(pe_b_val_waddr),
        .b_val_wdata(pe_b_val_wdata),
        .b_desc_we   (pe_b_desc_we),
        .b_desc_waddr(pe_b_desc_waddr),
        .b_desc_wdata(pe_b_desc_wdata),

        .c_rd_en  (c_rd_en),
        .c_rd_addr(c_rd_addr),
        .c_rd_data(c_rd_data),
        .c_rd_row (c_rd_row)
    );

    //=========================================================================
    // PE Drain Controller
    //=========================================================================
    wire        c_gbuf_wr_en;
    wire [C_AW-1:0] c_gbuf_wr_addr;
    wire [15:0] c_gbuf_wr_lane_valid;
    wire [16*16-1:0] c_gbuf_wr_lane_data;

    pe_drain_ctrl #(
        .N_PE(N_PE),
        .C_AW(C_AW)
    ) u_drain (
        .clk(clk), .rst_n(rst_n),
        .start(s_drain_rise),
        .done (drain_done),
        .M(M), .N(N),
        .pe_row_counts(pe_row_counts),

        .pe_c_rd_en  (c_rd_en),
        .pe_c_rd_addr(c_rd_addr),
        .pe_c_rd_data(c_rd_data),
        .pe_c_rd_row (c_rd_row),

        .c_gbuf_wr_en       (c_gbuf_wr_en),
        .c_gbuf_wr_addr     (c_gbuf_wr_addr),
        .c_gbuf_wr_lane_valid(c_gbuf_wr_lane_valid),
        .c_gbuf_wr_lane_data (c_gbuf_wr_lane_data)
    );

    //=========================================================================
    // C Global Buffer
    //=========================================================================
    c_global_buffer #(
        .ROWS(`MAX_M), .COLS(`MAX_N), .C_AW(C_AW)
    ) u_c_global (
        .clk(clk), .rst_n(rst_n),
        .wr_en         (c_gbuf_wr_en),
        .wr_addr       (c_gbuf_wr_addr),
        .wr_lane_valid (c_gbuf_wr_lane_valid),
        .wr_lane_data  (c_gbuf_wr_lane_data),
        .rd_en         (1'b1),           // always reading for host convenience
        .rd_addr       (c_host_rd_addr),
        .rd_data       (c_host_rd_data)
    );

endmodule
