//=============================================================================
// File     : tb_accelerator.v
// Brief    : Cocotb testbench wrapper for accelerator_top.
//            Exposes host write/read ports and debug signals.
//=============================================================================

`include "defines.vh"

module tb_accelerator;

    localparam N_PE = `N_PE;
    localparam M_AW  = `MAX_DIM_BITS;
    localparam A_NNZ_AW = 17;
    localparam A_DESC_AW = 10;
    localparam C_AW = `C_DENSE_DEPTH_LOG;

    // Clock & reset
    reg  aclk;
    reg  aresetn;

    // Control
    reg                     start;
    wire                    done;
    reg  [M_AW-1:0]         M;
    reg  [M_AW-1:0]         K;
    reg  [M_AW-1:0]         N;
    reg                     op_mode;
    reg                     op_sub;

    // A host write ports
    reg                     a_host_desc_wr_en;
    reg  [A_DESC_AW-1:0]    a_host_desc_wr_addr;
    reg  [63:0]             a_host_desc_wr_data;
    reg                     a_host_col_wr_en;
    reg  [A_NNZ_AW-1:0]     a_host_col_wr_addr;
    reg  [15:0]             a_host_col_wr_data;
    reg                     a_host_val_wr_en;
    reg  [A_NNZ_AW-1:0]     a_host_val_wr_addr;
    reg  [15:0]             a_host_val_wr_data;

    // B host write ports
    reg                     b_host_desc_wr_en;
    reg  [`B_ROW_ADDR_BITS-1:0] b_host_desc_wr_addr;
    reg  [31:0]             b_host_desc_wr_data;
    reg                     b_host_col_wr_en;
    reg  [`B_NNZ_ADDR_BITS-1:0] b_host_col_wr_addr;
    reg  [15:0]             b_host_col_wr_data;
    reg                     b_host_val_wr_en;
    reg  [`B_NNZ_ADDR_BITS-1:0] b_host_val_wr_addr;
    reg  [15:0]             b_host_val_wr_data;

    // C host read port
    reg  [C_AW-1:0]         c_host_rd_addr;
    wire [15:0]             c_host_rd_data;

    // ---- Clock generator (Cocotb mode) ----
`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    // ---- Accelerator instantiation ----
    accelerator_top #(.N_PE(N_PE)) u_accel (
        .clk               (aclk),
        .rst_n             (aresetn),
        .start             (start),
        .done              (done),
        .M                 (M),
        .K                 (K),
        .N                 (N),
        .op_mode           (op_mode),
        .op_sub            (op_sub),

        .a_host_desc_wr_en  (a_host_desc_wr_en),
        .a_host_desc_wr_addr(a_host_desc_wr_addr),
        .a_host_desc_wr_data(a_host_desc_wr_data),
        .a_host_col_wr_en   (a_host_col_wr_en),
        .a_host_col_wr_addr (a_host_col_wr_addr),
        .a_host_col_wr_data (a_host_col_wr_data),
        .a_host_val_wr_en   (a_host_val_wr_en),
        .a_host_val_wr_addr (a_host_val_wr_addr),
        .a_host_val_wr_data (a_host_val_wr_data),

        .b_host_desc_wr_en  (b_host_desc_wr_en),
        .b_host_desc_wr_addr(b_host_desc_wr_addr),
        .b_host_desc_wr_data(b_host_desc_wr_data),
        .b_host_col_wr_en   (b_host_col_wr_en),
        .b_host_col_wr_addr (b_host_col_wr_addr),
        .b_host_col_wr_data (b_host_col_wr_data),
        .b_host_val_wr_en   (b_host_val_wr_en),
        .b_host_val_wr_addr (b_host_val_wr_addr),
        .b_host_val_wr_data (b_host_val_wr_data),

        .c_host_rd_addr    (c_host_rd_addr),
        .c_host_rd_data    (c_host_rd_data)
    );

    // ---- VCD dump ----
`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/accel_dump.vcd");
        // $dumpvars(0, tb_accelerator);  // disabled for speed
    end
`endif

endmodule
