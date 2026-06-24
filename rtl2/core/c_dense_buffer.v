//=============================================================================
// File     : c_dense_buffer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : On-chip dense C buffer (512×512 FP16 = 512KB)
//            32-bank BRAM-friendly: write routes by addr[4:0],
//            read fans out 32 banks in parallel → 512-bit / cycle.
//=============================================================================

`include "defines.vh"

module c_dense_buffer (
    input  wire                      wr_en,
    input  wire [`C_DENSE_DEPTH_LOG-1:0] wr_addr,
    input  wire [`DATA_WIDTH-1:0]    wr_data,

    input  wire                      rd_en,
    input  wire [`C_DENSE_DEPTH_LOG-1:0] rd_addr,
    output wire [`AXI_DATA_WIDTH-1:0] rd_data,
    output reg                       rd_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam N_BANKS    = 32;
    localparam BANK_DEPTH = `C_DENSE_DEPTH / N_BANKS;
    localparam BANK_ADDR_W = `C_DENSE_DEPTH_LOG - 5;

    // Write: low 5 bits = bank select, high bits = bank-internal address
    wire [4:0]             wr_bank      = wr_addr[4:0];
    wire [BANK_ADDR_W-1:0] wr_bank_addr = wr_addr[`C_DENSE_DEPTH_LOG-1 : 5];

    // Read: all banks at the same internal address (rd_addr low 5 bits ignored;
    //   c_dense_ddr_writer always reads at 32-aligned addresses)
    wire [BANK_ADDR_W-1:0] rd_bank_addr = rd_addr[`C_DENSE_DEPTH_LOG-1 : 5];

    genvar i;
    generate
        for (i = 0; i < N_BANKS; i = i + 1) begin : gen_bram_bank
            reg [`DATA_WIDTH-1:0] ram [0:BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] rd_data_reg;

            // Write: one-hot bank select
            always @(posedge aclk) begin
                if (wr_en && (wr_bank == i))
                    ram[wr_bank_addr] <= wr_data;
            end

            // Read: all banks in parallel
            always @(posedge aclk) begin
                if (rd_en)
                    rd_data_reg <= ram[rd_bank_addr];
            end

            // Bit-slice assembly into 512-bit AXI data bus
            assign rd_data[i*`DATA_WIDTH +: `DATA_WIDTH] = rd_data_reg;
        end
    endgenerate

    // Read valid: 1-cycle latency matching BRAM read
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) rd_valid <= 1'b0;
        else          rd_valid <= rd_en;
    end

endmodule
