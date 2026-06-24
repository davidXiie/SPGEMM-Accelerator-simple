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

    // Row descriptor load port
    reg        a_desc_we;
    reg [`A_ROW_ADDR_BITS-1:0] a_desc_waddr;
    reg [63:0] a_desc_wdata;

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
    reg [`MAX_DIM_BITS-1:0] b_desc_waddr;
    reg [63:0] b_desc_wdata;

    // C buffer read port: addr = {local_row_idx[7:0], col[8:0]}  (17-bit)
    reg         c_rd_en;
    reg  [16:0] c_rd_addr;
    wire [31:0]  c_rd_data;    // FP32 output

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

        .a_desc_we      (a_desc_we),
        .a_desc_waddr   (a_desc_waddr),
        .a_desc_wdata   (a_desc_wdata),

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
        .c_rd_data      (c_rd_data)
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
