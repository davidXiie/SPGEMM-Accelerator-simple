//=============================================================================
// File     : pe_mul_array.v
// Brief    : N_MAC-lane FP16 Multiplier Array.
//
//   Each lane: FP16 × FP16 → FP16 (via fp16_mul combinatorial module).
//   MUL_LAT register stages delay col_id, fp16_val, and valid.
//
//   Task format per lane (TASK_WIDTH=41):
//     [8:0]   = col_id (9-bit, MAX_N=512)
//     [24:9]  = a_val  (FP16)
//     [40:25] = b_val  (FP16)
//
//   Product format per lane (PRODUCT_WIDTH=25):
//     [15:0]  = fp16_val (FP16 product)
//     [24:16] = col_id   (9-bit)
//=============================================================================

`include "defines.vh"

module pe_mul_array (
    // Task group input (4 lanes)
    input  wire [`N_MAC-1:0]              lane_valid,
    input  wire [`N_MAC*`TASK_WIDTH-1:0]  lane_task,

    // Product group output (4 lanes); PRODUCT_WIDTH = 32
    output wire [`N_MAC-1:0]                        mul_valid,
    output wire [`N_MAC*`PRODUCT_WIDTH-1:0]         mul_product,

    input  wire                           aclk,
    input  wire                           aresetn
);

    // Unpack task: {b_val[40:25], a_val[24:9], col_id[8:0]}  (41-bit)
    genvar m;
    generate
        for (m = 0; m < `N_MAC; m = m + 1) begin : gen_lane
            wire [8:0]             mac_col = lane_task[m*`TASK_WIDTH +: 9];       // [8:0]
            wire [`DATA_WIDTH-1:0] mac_a   = lane_task[m*`TASK_WIDTH + 9 +: 16]; // [24:9]
            wire [`DATA_WIDTH-1:0] mac_b   = lane_task[m*`TASK_WIDTH + 25 +: 16];// [40:25]

            // FP16 × FP16 → FP16 (combinatorial)
            wire [15:0] mul_fp16;
            fp16_mul u_fp16_mul (
                .a (mac_a),
                .b (mac_b),
                .z (mul_fp16)
            );

            // Pipeline: delay col_id, fp16 value, and valid by MUL_LAT cycles
            reg [8:0]  col_pipe   [0:`MUL_LAT-1];
            reg [15:0] val_pipe   [0:`MUL_LAT-1];
            reg        valid_pipe [0:`MUL_LAT-1];

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
                    val_pipe[0]   <= mul_fp16;
                    valid_pipe[0] <= lane_valid[m];

                    for (s = 1; s < `MUL_LAT; s = s + 1) begin
                        col_pipe[s]   <= col_pipe[s-1];
                        val_pipe[s]   <= val_pipe[s-1];
                        valid_pipe[s] <= valid_pipe[s-1];
                    end
                end
            end

            // Output: product = {col_id[8:0], fp16_val[15:0]}  (25-bit)
            wire [`PRODUCT_WIDTH-1:0] product;
            assign product[15:0]  = val_pipe[`MUL_LAT-1];   // FP16 product value
            assign product[24:16] = col_pipe[`MUL_LAT-1];   // col_id (9-bit)

            assign mul_valid[m]                                     = valid_pipe[`MUL_LAT-1];
            assign mul_product[m*`PRODUCT_WIDTH +: `PRODUCT_WIDTH] = product;
        end
    endgenerate

endmodule
