//=============================================================================
// File     : pe_serializer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Product Serializer — reads 4-lane product groups from
//            product_group_fifo, outputs one product per cycle via
//            registered valid/data handshake to accumulator.
//
//   Invalid lanes are automatically skipped.
//=============================================================================

`include "defines.vh"

module pe_serializer (
    input  wire                         prod_fifo_empty,
    output reg                          prod_fifo_rd_en,
    input  wire [`PRODUCT_GROUP_WIDTH-1:0] prod_fifo_rd_data,

    output reg                          acc_in_valid,
    input  wire                         acc_in_ready,
    output reg  [`PRODUCT_WIDTH-1:0]    acc_in_data,

    output wire                         idle,

    input  wire                         aclk,
    input  wire                         aresetn
);

    reg [`PRODUCT_GROUP_WIDTH-1:0] prod_group_reg;
    reg [3:0]                      prod_valid_reg;
    reg [1:0]                      prod_lane_ptr;
    reg                            prod_group_loaded;

    assign idle = !prod_group_loaded;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            prod_group_loaded <= 1'b0;
            prod_lane_ptr     <= 2'd0;
            prod_fifo_rd_en   <= 1'b0;
            prod_group_reg    <= 0;
            prod_valid_reg    <= 4'b0;
            acc_in_valid      <= 1'b0;
            acc_in_data       <= 0;
        end else begin
            prod_fifo_rd_en <= 1'b0;

            if (!prod_group_loaded && !prod_fifo_empty) begin
                // Load new group from FIFO
                prod_group_reg    <= prod_fifo_rd_data;
                prod_valid_reg    <= prod_fifo_rd_data[3:0];
                prod_lane_ptr     <= 2'd0;
                prod_group_loaded <= 1'b1;
                prod_fifo_rd_en   <= 1'b1;
                acc_in_valid      <= 1'b0;
                // Will output first valid lane on next cycle
            end else if (prod_group_loaded) begin
                // Skip invalid lanes immediately
                if (!prod_valid_reg[prod_lane_ptr]) begin
                    acc_in_valid <= 1'b0;
                    if (prod_lane_ptr == 2'd3) begin
                        prod_group_loaded <= 1'b0;
                        prod_lane_ptr     <= 2'd0;
                    end else begin
                        prod_lane_ptr <= prod_lane_ptr + 1'b1;
                    end
                // Valid lane: first raise acc_in_valid with data, then wait for acc_in_ready
                end else if (!acc_in_valid) begin
                    // Present product to accumulator
                    acc_in_valid <= 1'b1;
                    case (prod_lane_ptr)
                        2'd0: acc_in_data <= prod_group_reg[35:4];
                        2'd1: acc_in_data <= prod_group_reg[67:36];
                        2'd2: acc_in_data <= prod_group_reg[99:68];
                        2'd3: acc_in_data <= prod_group_reg[131:100];
                    endcase
                end else if (acc_in_ready) begin
                    // Accumulator accepted → advance to next lane
                    acc_in_valid <= 1'b0;
                    if (prod_lane_ptr == 2'd3) begin
                        prod_group_loaded <= 1'b0;
                        prod_lane_ptr     <= 2'd0;
                    end else begin
                        prod_lane_ptr <= prod_lane_ptr + 1'b1;
                    end
                end else begin
                    // Accumulator busy → hold acc_in_valid=1
                    acc_in_valid <= 1'b1;
                end
            end
        end
    end

endmodule
