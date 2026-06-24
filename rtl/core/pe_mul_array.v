//=============================================================================
// File     : pe_mul_array.v
// Brief    : 4-lane FP16 Multiplier Array with col-delay pipeline.
//
//   Each lane: FP16 × FP16 → FP32 (via fp16_mul combinatorial module).
//   MUL_LAT register stages delay col_id, fp32_val, and valid.
//
//   Product format per lane (PRODUCT_WIDTH=48): {col_id[15:0], fp32_val[31:0]}
//=============================================================================

`include "defines.vh"

module pe_mul_array (
    // Task group input (4 lanes)
    input  wire [`N_MAC-1:0]              lane_valid,
    input  wire [`N_MAC*`TASK_WIDTH-1:0]  lane_task,

    // Product group output (4 lanes); PRODUCT_WIDTH = 48
    output wire [`N_MAC-1:0]                        mul_valid,
    output wire [`N_MAC*`PRODUCT_WIDTH-1:0]         mul_product,

    input  wire                           aclk,
    input  wire                           aresetn
);

    // Unpack task: {reserved[15:0], b_val[15:0], a_val[15:0], col_id[15:0]}
    genvar m;
    generate
        for (m = 0; m < `N_MAC; m = m + 1) begin : gen_lane
            wire [`DATA_WIDTH-1:0] mac_a   = lane_task[m*`TASK_WIDTH + 31 -: `DATA_WIDTH];
            wire [`DATA_WIDTH-1:0] mac_b   = lane_task[m*`TASK_WIDTH + 47 -: `DATA_WIDTH];
            wire [`DATA_WIDTH-1:0] mac_col = lane_task[m*`TASK_WIDTH + 15 -: `DATA_WIDTH];

            // FP16 × FP16 → FP32 (combinatorial)
            wire [31:0] mul_fp32;
            fp16_mul u_fp16_mul (
                .a (mac_a),
                .b (mac_b),
                .z (mul_fp32)
            );

            // Pipeline: delay col_id, fp32 value, and valid by MUL_LAT cycles
            reg [`DATA_WIDTH-1:0] col_pipe   [0:`MUL_LAT-1];
            reg [31:0]            val_pipe   [0:`MUL_LAT-1];
            reg                   valid_pipe [0:`MUL_LAT-1];

            integer s;
            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn) begin
                    for (s = 0; s < `MUL_LAT; s = s + 1) begin
                        col_pipe[s]   <= 0;
                        val_pipe[s]   <= 0;
                        valid_pipe[s] <= 1'b0;
                    end
                end else begin
                    col_pipe[0]   <= mac_col;
                    val_pipe[0]   <= mul_fp32;
                    valid_pipe[0] <= lane_valid[m];

                    for (s = 1; s < `MUL_LAT; s = s + 1) begin
                        col_pipe[s]   <= col_pipe[s-1];
                        val_pipe[s]   <= val_pipe[s-1];
                        valid_pipe[s] <= valid_pipe[s-1];
                    end
                end
            end

            // Output: product = {col_id[15:0], fp32_val[31:0]}  (48-bit)
            wire [`PRODUCT_WIDTH-1:0] product;
            assign product[31:0]  = val_pipe[`MUL_LAT-1];   // FP32 product value
            assign product[47:32] = col_pipe[`MUL_LAT-1];   // col_id

            assign mul_valid[m]                                     = valid_pipe[`MUL_LAT-1];
            assign mul_product[m*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = product;
        end
    endgenerate

endmodule
