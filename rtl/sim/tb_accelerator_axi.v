//=============================================================================
// tb_accelerator_axi.v
//=============================================================================

`include "defines.vh"

module tb_accelerator_axi;

    localparam N_PE = `N_PE;
    localparam M_AW = `MAX_DIM_BITS;

    reg  aclk;
    reg  aresetn;
    reg  start;
    wire done;
    reg  [M_AW-1:0] M, K, N;
    reg  op_mode, op_sub;

    // PE C read ports
    reg  [N_PE*(`C_ROW_ADDR_BITS+5)-1:0] c_rd_addr;
    wire [N_PE-1:0]                     c_rd_en;
    wire [N_PE*16*16-1:0]               c_rd_data;
    wire [N_PE*`MAX_DIM_BITS-1:0]       c_rd_row;

`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    accelerator_axi_top #(.N_PE(N_PE), .M_AW(M_AW)) u_accel (
        .clk(aclk), .rst_n(aresetn),
        .start(start), .done(done), .M(M), .K(K), .N(N),
        .op_mode(op_mode), .op_sub(op_sub),
        .c_rd_en(c_rd_en), .c_rd_addr(c_rd_addr),
        .c_rd_data(c_rd_data), .c_rd_row(c_rd_row)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("sim_build/accel_axi_dump.vcd");
        $dumpvars(0, tb_accelerator_axi.u_accel.state);
        $dumpvars(0, tb_accelerator_axi.u_accel.u_loader.state);
        $dumpvars(0, tb_accelerator_axi.u_accel.u_cluster.gen_pe[0].u_pe.state);
        $dumpvars(0, tb_accelerator_axi.done);
    end
`endif

endmodule
