//=============================================================================
// File     : tb_pe_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Minimal cocotb wrapper for pe_top unit test.
//            Exposes all PE ports as module-level wires for cocotb to drive.
//=============================================================================

`include "defines.vh"

module tb_pe_top;

    reg aclk;
    reg aresetn;

    reg        start;
    reg [15:0] row_count;
    wire       done;

    // A buffer load ports
    reg        a_desc_we;
    reg [7:0]  a_desc_waddr;
    reg [63:0] a_desc_wdata;
    reg        a_col_we;
    reg [15:0] a_col_waddr;
    reg [`DATA_WIDTH-1:0] a_col_wdata;
    reg        a_val_we;
    reg [15:0] a_val_waddr;
    reg [`DATA_WIDTH-1:0] a_val_wdata;

    // B buffer load ports
    reg        b_desc_we;
    reg [9:0]  b_desc_waddr;
    reg [63:0] b_desc_wdata;
    reg        b_col_we;
    reg [17:0] b_col_waddr;
    reg [`DATA_WIDTH-1:0] b_col_wdata;
    reg        b_val_we;
    reg [17:0] b_val_waddr;
    reg [`DATA_WIDTH-1:0] b_val_wdata;

    // C buffer write handshake
    wire       cbuf_wr_valid;
    reg        cbuf_wr_ready;
    wire [17:0] cbuf_wr_addr;
    wire [`DATA_WIDTH-1:0] cbuf_wr_data;

`ifndef COCOTB_SIM
    always #5 aclk = ~aclk;
`endif

    pe_top #(.PE_ID(0)) u_pe (
        .aclk           (aclk),
        .aresetn        (aresetn),
        .start          (start),
        .row_count      (row_count),
        .done           (done),

        .a_desc_we      (a_desc_we),
        .a_desc_waddr   (a_desc_waddr[`A_ROW_ADDR_BITS-1:0]),
        .a_desc_wdata   (a_desc_wdata),
        .a_col_we       (a_col_we),
        .a_col_waddr    (a_col_waddr[`A_NNZ_ADDR_BITS-1:0]),
        .a_col_wdata    (a_col_wdata),
        .a_val_we       (a_val_we),
        .a_val_waddr    (a_val_waddr[`A_NNZ_ADDR_BITS-1:0]),
        .a_val_wdata    (a_val_wdata),

        .b_desc_we      (b_desc_we),
        .b_desc_waddr   (b_desc_waddr[`B_ROW_ADDR_BITS-1:0]),
        .b_desc_wdata   (b_desc_wdata),
        .b_col_we       (b_col_we),
        .b_col_waddr    (b_col_waddr[`B_NNZ_ADDR_BITS-1:0]),
        .b_col_wdata    (b_col_wdata),
        .b_val_we       (b_val_we),
        .b_val_waddr    (b_val_waddr[`B_NNZ_ADDR_BITS-1:0]),
        .b_val_wdata    (b_val_wdata),

        .cbuf_wr_valid  (cbuf_wr_valid),
        .cbuf_wr_ready  (cbuf_wr_ready),
        .cbuf_wr_addr   (cbuf_wr_addr),
        .cbuf_wr_data   (cbuf_wr_data)
    );

`ifndef COCOTB_SIM
    initial begin
        $dumpfile("tb_pe_top.fst");
        $dumpvars(0, tb_pe_top);
        #10000000 $finish;
    end
`endif

`ifdef COCOTB_SIM
    // VCD off for speed
`endif

endmodule
