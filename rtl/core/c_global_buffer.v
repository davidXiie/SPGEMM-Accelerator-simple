//=============================================================================
// File     : c_global_buffer.v
// Brief    : Global dense C-matrix buffer.  16 sub-banks, each holds one
//            lane of a 16-column group (col%16 = bank_id).
//
//   Address = {row[`MAX_DIM_BITS-1:0], gaddr[4:0]}
//     row   = global C row  (0 .. M-1)
//     gaddr = col / 16       (0 .. ceil(N/16)-1)
//
//   Write:  from pe_drain_ctrl, lane-granular (16 lanes/cycle).
//   Read:   from host after done, single 16-bit element per cycle.
//
//   Sizing: MAX_M * MAX_N = 512 * 512 = 262144 entries x 16-bit = 4 Mbit
//           (16 sub-banks, each 16384 x 16-bit)
//=============================================================================

`include "defines.vh"

module c_global_buffer #(
    parameter ROWS     = `MAX_M,
    parameter COLS     = `MAX_N,
    parameter C_AW     = `C_DENSE_DEPTH_LOG    // log2(M*N) = 18
) (
    input  wire clk,
    input  wire rst_n,

    // === Write port (from pe_drain_ctrl) ===
    // 16-lane group write — addr = {row[9:0], gaddr[4:0]}
    // Each lane valid independently (drain_valid[15:0])
    input  wire                   wr_en,
    input  wire [C_AW-1:0]       wr_addr,       // {row, gaddr}
    input  wire [15:0]           wr_lane_valid,
    input  wire [16*16-1:0]      wr_lane_data,  // 16 × 16-bit

    // === Read port (from host) ===
    input  wire                   rd_en,
    input  wire [C_AW-1:0]       rd_addr,       // = row*COLS + col
    output reg  [15:0]            rd_data
);

    localparam BANK_DEPTH = (ROWS * COLS) / 16;  // 16384
    localparam BANK_AW     = C_AW - 4;            // 14-bit

    // 16 sub-banks, each holds one column lane
    genvar cb;
    generate
        for (cb = 0; cb < 16; cb = cb + 1) begin : gen_bank
            reg [15:0] mem [0:BANK_DEPTH-1];
            always @(posedge clk) begin
                if (wr_en && wr_lane_valid[cb])
                    mem[wr_addr[BANK_AW-1:0]] <= wr_lane_data[cb*16 +: 16];
            end
        end
    endgenerate

    // Single-element read — select correct sub-bank from rd_addr[3:0]
    wire [3:0]  rd_bank = rd_addr[3:0];
    wire [BANK_AW-1:0] rd_bank_addr = rd_addr[C_AW-1:4];

    always @(posedge clk) begin
        if (rd_en) begin
            case (rd_bank)
                4'd0:  rd_data <= gen_bank[0 ].mem[rd_bank_addr];
                4'd1:  rd_data <= gen_bank[1 ].mem[rd_bank_addr];
                4'd2:  rd_data <= gen_bank[2 ].mem[rd_bank_addr];
                4'd3:  rd_data <= gen_bank[3 ].mem[rd_bank_addr];
                4'd4:  rd_data <= gen_bank[4 ].mem[rd_bank_addr];
                4'd5:  rd_data <= gen_bank[5 ].mem[rd_bank_addr];
                4'd6:  rd_data <= gen_bank[6 ].mem[rd_bank_addr];
                4'd7:  rd_data <= gen_bank[7 ].mem[rd_bank_addr];
                4'd8:  rd_data <= gen_bank[8 ].mem[rd_bank_addr];
                4'd9:  rd_data <= gen_bank[9 ].mem[rd_bank_addr];
                4'd10: rd_data <= gen_bank[10].mem[rd_bank_addr];
                4'd11: rd_data <= gen_bank[11].mem[rd_bank_addr];
                4'd12: rd_data <= gen_bank[12].mem[rd_bank_addr];
                4'd13: rd_data <= gen_bank[13].mem[rd_bank_addr];
                4'd14: rd_data <= gen_bank[14].mem[rd_bank_addr];
                4'd15: rd_data <= gen_bank[15].mem[rd_bank_addr];
            endcase
        end
    end

endmodule
