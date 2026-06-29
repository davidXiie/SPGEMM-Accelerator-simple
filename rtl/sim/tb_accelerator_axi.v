//=============================================================================
// File     : tb_accelerator_axi.v
// Brief    : Cocotb testbench for accelerator_axi_top (DDR-direct).
//=============================================================================

`include "defines.vh"

module tb_accelerator_axi;

    localparam N_PE   = `N_PE;
    localparam M_AW   = `MAX_DIM_BITS;
    localparam MEM_AW = 22;       // DDR model depth

    reg  aclk;
    reg  aresetn;

    // Control
    reg                     start;
    wire                    done;
    reg  [M_AW-1:0]         M, K, N;
    reg                     op_mode, op_sub;

    // DDR host ports
    reg                     host_wr_en;
    reg  [MEM_AW-1:0]       host_wr_addr;
    reg  [15:0]             host_wr_data;
    reg  [MEM_AW-1:0]       host_rd_addr;
    wire [15:0]             host_rd_data;

    // PE C read ports
    wire [N_PE-1:0]                     c_rd_en;
    reg  [N_PE*(`C_ROW_ADDR_BITS+5)-1:0] c_rd_addr;
    wire [N_PE*16*16-1:0]               c_rd_data;
    wire [N_PE*`MAX_DIM_BITS-1:0]       c_rd_row;

`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    accelerator_axi_top #(.N_PE(N_PE), .M_AW(M_AW), .MEM_AW(MEM_AW)) u_accel (
        .clk(aclk), .rst_n(aresetn),
        .start(start), .done(done),
        .M(M), .K(K), .N(N),
        .op_mode(op_mode), .op_sub(op_sub),
        .host_wr_en  (host_wr_en),
        .host_wr_addr(host_wr_addr),
        .host_wr_data(host_wr_data),
        .host_rd_addr(host_rd_addr),
        .host_rd_data(host_rd_data),
        .c_rd_en  (c_rd_en),
        .c_rd_addr(c_rd_addr),
        .c_rd_data(c_rd_data),
        .c_rd_row (c_rd_row)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/accel_axi_dump.vcd");
        $dumpvars(0, tb_accelerator_axi);
    end
`endif

endmodule
