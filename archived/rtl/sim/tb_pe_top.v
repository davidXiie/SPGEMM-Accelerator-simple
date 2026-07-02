//=============================================================================
// File     : tb_pe_top.v  (inst-version)
// Brief    : Cocotb wrapper for pe_top unit test.
//=============================================================================

`include "defines.vh"

module tb_pe_top;

    reg aclk;
    reg aresetn;

    reg        start;
    reg [15:0] row_count;
    wire       done;

    reg [`MAX_DIM_BITS-1:0] M;
    reg [`MAX_DIM_BITS-1:0] K;
    reg [`MAX_DIM_BITS-1:0] N;

    reg op_mode;   // 0 = SpGEMM, 1 = elementwise
    reg op_sub;    // elementwise: 0 = add, 1 = subtract

    // A row descriptor stream port
    reg        a_desc_valid;
    wire       a_desc_ready;
    reg [35:0] a_desc_data;

    // A value buffer load port
    reg        a_val_we;
    reg [`A_NNZ_ADDR_BITS-1:0] a_val_waddr;
    reg [`DATA_WIDTH-1:0] a_val_wdata;

    // B col/val buffer load ports
    reg        b_col_we;
    reg [`B_NNZ_ADDR_BITS-1:0] b_col_waddr;
    reg [`DATA_WIDTH-1:0] b_col_wdata;
    reg        b_val_we;
    reg [`B_NNZ_ADDR_BITS-1:0] b_val_waddr;
    reg [`DATA_WIDTH-1:0] b_val_wdata;

    // A column index buffer load port
    reg        a_col_we;
    reg [`A_NNZ_ADDR_BITS-1:0] a_col_waddr;
    reg [`DATA_WIDTH-1:0] a_col_wdata;

    // B row descriptor load port
    reg        b_desc_we;
    reg [`B_ROW_ADDR_BITS-1:0] b_desc_waddr;
    reg [31:0] b_desc_wdata;

    // C buffer read port (independent C bank, local-row indexed)
    reg                          c_rd_en;
    reg  [`C_ROW_ADDR_BITS+4:0]  c_rd_addr;   // {local_row, gaddr}
    wire [16*16-1:0]             c_rd_data;   // 16 FP16 lanes per group
    wire [`MAX_DIM_BITS-1:0]     c_rd_row;    // global C row of this local slot


`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    pe_top #(.PE_ID(0)) u_pe (
        .aclk           (aclk),
        .aresetn        (aresetn),
        .start          (start),
        .row_count      (row_count),
        .done           (done),
        .M              (M),
        .K              (K),
        .N              (N),

        .op_mode        (op_mode),
        .op_sub         (op_sub),

        .a_desc_valid   (a_desc_valid),
        .a_desc_ready   (a_desc_ready),
        .a_desc_data    (a_desc_data),

        .a_val_we       (a_val_we),
        .a_val_waddr    (a_val_waddr),
        .a_val_wdata    (a_val_wdata),

        .a_col_we       (a_col_we),
        .a_col_waddr    (a_col_waddr),
        .a_col_wdata    (a_col_wdata),

        .b_col_we       (b_col_we),
        .b_col_waddr    (b_col_waddr),
        .b_col_wdata    (b_col_wdata),
        .b_val_we       (b_val_we),
        .b_val_waddr    (b_val_waddr),
        .b_val_wdata    (b_val_wdata),

        .b_desc_we      (b_desc_we),
        .b_desc_waddr   (b_desc_waddr),
        .b_desc_wdata   (b_desc_wdata),

        .c_rd_en        (c_rd_en),
        .c_rd_addr      (c_rd_addr),
        .c_rd_data      (c_rd_data),
        .c_rd_row       (c_rd_row)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/pe_dump.vcd");
        $dumpvars(0, tb_pe_top.u_pe.state);
        $dumpvars(0, tb_pe_top.u_pe.row_idx);
        $dumpvars(0, tb_pe_top.u_pe.gen_state);
        $dumpvars(0, tb_pe_top.u_pe.done);
    end
`endif

endmodule
