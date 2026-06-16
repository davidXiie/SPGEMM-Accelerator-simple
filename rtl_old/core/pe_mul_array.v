//=============================================================================
// File     : pe_mul_array.v
// Project  : SPGEMM-Accelerator
// Brief    : Configurable Arithmetic Array - N_MAC parallel FP16 ALUs with 3-stage pipeline.
//           Supports MUL (SpGEMM), ADD (SpAdd), SUB (SpSubtract).
//           Also pipelines row_start/row_end/row_id to align with product latency.
//=============================================================================

`include "defines.vh"

module pe_mul_array (
    input  wire [2:0]                          op_type,
    input  wire [`N_MAC-1:0]                  lane_valid,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_a_val,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_b_val,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_col_idx,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_row_idx,

    // Row control signals (pipelined 3 stages to match product latency)
    input  wire                               row_start_in,
    input  wire                               row_end_in,
    input  wire [`MAX_DIM_BITS-1:0]           row_id_in,

    // Pipelined output (3 stages: reg → ALU → reg)
    output reg  [`N_MAC-1:0]                  mul_valid,
    output reg  [`N_MAC*`DATA_WIDTH-1:0]      partial_value,
    output reg  [`N_MAC*`DATA_WIDTH-1:0]      col_idx,
    output reg  [`N_MAC*`DATA_WIDTH-1:0]      row_idx,

    // Row control output (delayed to match product)
    output wire                               row_start_out,
    output wire                               row_end_out,
    output wire [`MAX_DIM_BITS-1:0]           row_id_out,

    input  wire                               aclk,
    input  wire                               aresetn
);

    // Stage 0: input registers
    reg [`N_MAC-1:0]                  valid_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      a_val_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      b_val_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      col_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      row_s0;
    reg [2:0]                         op_type_s0;
    reg                               row_start_s0, row_end_s0;
    reg [`MAX_DIM_BITS-1:0]           row_id_s0;

    // Stage 1: ALU
    reg [`N_MAC-1:0]                  valid_s1;
    reg [`N_MAC*`DATA_WIDTH-1:0]      partial_s1;
    reg [`N_MAC*`DATA_WIDTH-1:0]      col_s1;
    reg [`N_MAC*`DATA_WIDTH-1:0]      row_s1;
    reg                               row_start_s1, row_end_s1;
    reg [`MAX_DIM_BITS-1:0]           row_id_s1;

    // Stage 2: output registers
    reg [`N_MAC-1:0]                  valid_s2;
    reg [`N_MAC*`DATA_WIDTH-1:0]      partial_s2;
    reg [`N_MAC*`DATA_WIDTH-1:0]      col_s2;
    reg [`N_MAC*`DATA_WIDTH-1:0]      row_s2;
    reg                               row_start_s2, row_end_s2;
    reg [`MAX_DIM_BITS-1:0]           row_id_s2;

    // Stage 0: register inputs
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_s0   <= 0; a_val_s0 <= 0; b_val_s0 <= 0; col_s0 <= 0; row_s0 <= 0;
            op_type_s0 <= `OP_TYPE_MUL;
            row_start_s0 <= 0; row_end_s0 <= 0; row_id_s0 <= 0;
        end else begin
            valid_s0   <= lane_valid;
            a_val_s0   <= lane_a_val;
            b_val_s0   <= lane_b_val;
            col_s0     <= lane_col_idx;
            row_s0     <= lane_row_idx;
            op_type_s0 <= op_type;
            row_start_s0 <= row_start_in;
            row_end_s0   <= row_end_in;
            row_id_s0    <= row_id_in;
        end
    end

    // Stage 1: ALU operation
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_s1   <= 0; partial_s1 <= 0; col_s1 <= 0; row_s1 <= 0;
            row_start_s1 <= 0; row_end_s1 <= 0; row_id_s1 <= 0;
        end else begin
            valid_s1 <= valid_s0; col_s1 <= col_s0; row_s1 <= row_s0;
            row_start_s1 <= row_start_s0; row_end_s1 <= row_end_s0; row_id_s1 <= row_id_s0;
            for (integer m = 0; m < `N_MAC; m = m + 1) begin
                if (valid_s0[m]) begin
                    case (op_type_s0)
                        `OP_TYPE_MUL: partial_s1[m*`DATA_WIDTH +: `DATA_WIDTH] <=
                            a_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH] * b_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH];
                        `OP_TYPE_ADD: partial_s1[m*`DATA_WIDTH +: `DATA_WIDTH] <=
                            a_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH] + b_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH];
                        `OP_TYPE_SUB: partial_s1[m*`DATA_WIDTH +: `DATA_WIDTH] <=
                            a_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH] - b_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH];
                        default: partial_s1[m*`DATA_WIDTH +: `DATA_WIDTH] <=
                            a_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH] * b_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH];
                    endcase
                end
            end
        end
    end

    // Stage 2: output registers
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_s2 <= 0; partial_s2 <= 0; col_s2 <= 0; row_s2 <= 0;
            row_start_s2 <= 0; row_end_s2 <= 0; row_id_s2 <= 0;
        end else begin
            valid_s2   <= valid_s1; partial_s2 <= partial_s1; col_s2 <= col_s1; row_s2 <= row_s1;
            row_start_s2 <= row_start_s1; row_end_s2 <= row_end_s1; row_id_s2 <= row_id_s1;
        end
    end

    assign mul_valid     = valid_s2;
    assign partial_value = partial_s2;
    assign col_idx       = col_s2;
    assign row_idx       = row_s2;
    assign row_start_out = row_start_s2;
    assign row_end_out   = row_end_s2;
    assign row_id_out    = row_id_s2;

endmodule
