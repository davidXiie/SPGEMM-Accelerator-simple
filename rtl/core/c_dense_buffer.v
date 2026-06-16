//=============================================================================
// File     : c_dense_buffer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : On-chip dense C buffer (512×512 FP16 = 512KB)
//            Written by c_dense_write_arbiter, read by c_dense_ddr_writer.
//=============================================================================

`include "defines.vh"

module c_dense_buffer (
    input  wire                      wr_en,
    input  wire [`C_DENSE_DEPTH_LOG-1:0] wr_addr,
    input  wire [`DATA_WIDTH-1:0]    wr_data,

    input  wire                      rd_en,
    input  wire [`C_DENSE_DEPTH_LOG-1:0] rd_addr,
    output wire [`AXI_DATA_WIDTH-1:0] rd_data,
    output wire                      rd_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    // Store as flat array: C[global_row_id * N + col]
    reg [`DATA_WIDTH-1:0] ram [0:`C_DENSE_DEPTH-1];

    // Write: 16-bit per element
    always @(posedge aclk) begin
        if (wr_en) ram[wr_addr] <= wr_data;
    end

    // Read: AXI_DATA_WIDTH wide (32×FP16 = 512-bit per read)
    reg [`AXI_DATA_WIDTH-1:0] rd_data_reg;
    reg rd_valid_reg;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rd_data_reg  <= 0;
            rd_valid_reg <= 1'b0;
        end else begin
            rd_valid_reg <= rd_en;
            if (rd_en) begin
                // Read 32 consecutive FP16 elements
                for (integer i = 0; i < `N_ELEM_PER_AXI_BEAT; i = i + 1) begin
                    rd_data_reg[i*`DATA_WIDTH +: `DATA_WIDTH]
                        <= ram[rd_addr + i];
                end
            end
        end
    end

    assign rd_data  = rd_data_reg;
    assign rd_valid = rd_valid_reg;

endmodule
