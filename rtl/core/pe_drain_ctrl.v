//=============================================================================
// File     : pe_drain_ctrl.v
// Brief    : Reads C rows from each PE's local C bank and writes them into
//            the global dense C buffer (c_global_buffer).
//
//   For each PE pid:
//     For local_row 0 .. row_counts[pid]-1:
//       For gaddr 0 .. ceil(N/16)-1:
//         1. Issue c_rd_en[pid], c_rd_addr = {local_row, gaddr}
//         2. Wait 1 cycle for registered read
//         3. c_rd_row[pid] = global C row, c_rd_data[pid] = 16 FP16 values
//         4. Write to c_global_buffer[global_row][gaddr*16 + lane]
//=============================================================================

`include "defines.vh"

module pe_drain_ctrl #(
    parameter N_PE   = `N_PE,
    parameter C_AW    = `C_DENSE_DEPTH_LOG       // log2(512*512) = 18
) (
    input  wire clk,
    input  wire rst_n,

    // === Start / Done ===
    input  wire                    start,
    output reg                     done,

    // === Matrix dimensions ===
    input  wire [`MAX_DIM_BITS-1:0] M,
    input  wire [`MAX_DIM_BITS-1:0] N,
    input  wire [N_PE*16-1:0]      pe_row_counts,  // [pid*16+:16] = rows for PE pid

    // === PE Cluster C read ports (packed) ===
    output reg  [N_PE-1:0]                          pe_c_rd_en,
    output reg  [N_PE*(`C_ROW_ADDR_BITS+5)-1:0]    pe_c_rd_addr,
    input  wire [N_PE*16*16-1:0]                    pe_c_rd_data,
    input  wire [N_PE*`MAX_DIM_BITS-1:0]            pe_c_rd_row,

    // === C Global Buffer write port ===
    output reg                     c_gbuf_wr_en,
    output reg  [C_AW-1:0]        c_gbuf_wr_addr,     // {global_row[9:0], gaddr[4:0]}
    output reg  [15:0]             c_gbuf_wr_lane_valid,
    output reg  [16*16-1:0]        c_gbuf_wr_lane_data
);

    localparam C_RD_ADDR_W = `C_ROW_ADDR_BITS + 5;  // per-PE C read addr width

    // FSM states
    localparam DR_IDLE      = 3'd0;
    localparam DR_ISSUE     = 3'd1;   // issue c_rd_en / c_rd_addr
    localparam DR_WAIT      = 3'd2;   // wait 1 cycle for registered C read
    localparam DR_WRITE     = 3'd3;   // write to c_global_buffer
    localparam DR_NEXT      = 3'd4;   // advance loop variables
    localparam DR_DONE      = 3'd5;

    reg [2:0] state;

    reg [2:0]                  dr_pid;        // 0..N_PE-1
    reg [`C_ROW_ADDR_BITS-1:0] dr_local_row;  // 0..row_counts[pid]-1
    reg [4:0]                  dr_gaddr;       // 0..ceil(N/16)-1
    reg [`MAX_DIM_BITS-1:0]    dr_global_row;  // latched from c_rd_row
    reg [16*16-1:0]            dr_rd_data;     // latched from c_rd_data

    wire [`MAX_DIM_BITS-1:0] pe_rows = pe_row_counts[dr_pid*16 +: 16];
    wire [5:0] ngroups = (N + 15) >> 4;   // ceil(N/16), max 32

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= DR_IDLE;
            dr_pid      <= 0;
            dr_local_row<= 0;
            dr_gaddr    <= 0;
            pe_c_rd_en  <= {N_PE{1'b0}};
            c_gbuf_wr_en<= 1'b0;
            done        <= 1'b0;
        end else begin
            // Defaults
            pe_c_rd_en  <= {N_PE{1'b0}};
            c_gbuf_wr_en<= 1'b0;
            done        <= 1'b0;

            case (state)
                DR_IDLE: begin
                    if (start) begin
                        dr_pid       <= 0;
                        dr_local_row <= 0;
                        dr_gaddr     <= 0;
                        state        <= DR_ISSUE;
                    end
                end

                DR_ISSUE: begin
                    // Issue read command to PE pid
                    pe_c_rd_en <= (1 << dr_pid);
                    pe_c_rd_addr <= ({dr_local_row, dr_gaddr}) << (dr_pid * C_RD_ADDR_W);
                    state <= DR_WAIT;
                end

                DR_WAIT: begin
                    // Registered read: data appears 1 cycle later
                    state <= DR_WRITE;
                end

                DR_WRITE: begin
                    // Latch PE output
                    dr_global_row <= pe_c_rd_row[dr_pid*`MAX_DIM_BITS +: `MAX_DIM_BITS];
                    dr_rd_data    <= pe_c_rd_data[dr_pid*16*16 +: 16*16];

                    // Write to c_global_buffer
                    c_gbuf_wr_en   <= 1'b1;
                    c_gbuf_wr_addr <= {pe_c_rd_row[dr_pid*`MAX_DIM_BITS +: `MAX_DIM_BITS],
                                       dr_gaddr};
                    c_gbuf_wr_lane_valid <= 16'hFFFF;  // drain writes all lanes
                    c_gbuf_wr_lane_data  <= pe_c_rd_data[dr_pid*16*16 +: 16*16];

                    state <= DR_NEXT;
                end

                DR_NEXT: begin
                    // Advance loop
                    if (dr_gaddr + 1 < ngroups[4:0]) begin
                        dr_gaddr <= dr_gaddr + 1;
                        state    <= DR_ISSUE;
                    end else begin
                        dr_gaddr <= 0;
                        if (dr_local_row + 1 < pe_rows) begin
                            dr_local_row <= dr_local_row + 1;
                            state        <= DR_ISSUE;
                        end else begin
                            dr_local_row <= 0;
                            if (dr_pid + 1 < N_PE) begin
                                dr_pid  <= dr_pid + 1;
                                state   <= DR_ISSUE;
                            end else begin
                                state <= DR_DONE;
                            end
                        end
                    end
                end

                DR_DONE: begin
                    done <= 1'b1;
                end

                default: state <= DR_IDLE;
            endcase
        end
    end

endmodule
