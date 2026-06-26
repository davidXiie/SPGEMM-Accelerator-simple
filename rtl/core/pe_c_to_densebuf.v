//=============================================================================
// File     : pe_c_to_densebuf.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Reads PE internal 4-bank C buffer (FP16) and writes to
//            c_dense_buffer.  c_rd_addr = {local_row[7:0], col[8:0]}.
//
//   FSM: IDLE → READ → WRITE → NEXT → DONE
//   c_rd_en is asserted during READ.  c_rd_data has 1-cycle latency
//   (registered read in pe_top), so WRITE state latches the result
//   and writes c_dense_buffer.
//=============================================================================

`include "defines.vh"

module pe_c_to_densebuf (
    input  wire                         start,
    output reg                          done,

    input  wire [`A_ROW_ADDR_BITS-1:0]  rows,
    input  wire [`MAX_DIM_BITS-1:0]     base_row,  // global C row offset for this PE
    input  wire [`MAX_DIM_BITS-1:0]     N,

    // PE C buffer read port
    output reg                          c_rd_en,
    output reg  [16:0]                  c_rd_addr,
    input  wire [15:0]                  c_rd_data,   // FP16

    // c_dense_buffer write port
    output reg                          cbuf_wr_en,
    output reg  [`C_DENSE_DEPTH_LOG-1:0] cbuf_wr_addr,
    output reg  [`DATA_WIDTH-1:0]       cbuf_wr_data,

    input  wire                         aclk,
    input  wire                         aresetn
);

    localparam S_IDLE  = 3'd0;
    localparam S_READ  = 3'd1;
    localparam S_WRITE = 3'd2;
    localparam S_NEXT  = 3'd3;
    localparam S_DONE  = 3'd4;

    reg [2:0] state;
    reg [`A_ROW_ADDR_BITS-1:0] row;
    reg [`MAX_DIM_BITS-1:0]    col;
    reg started_d;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= S_IDLE; done <= 1'b0; started_d <= 1'b0;
            c_rd_en <= 1'b0; c_rd_addr <= 0;
            cbuf_wr_en <= 1'b0; cbuf_wr_addr <= 0; cbuf_wr_data <= 0;
            row <= 0; col <= 0;
        end else begin
            done <= 1'b0; c_rd_en <= 1'b0; cbuf_wr_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start && !started_d) begin
                        started_d <= 1'b1;
                        row <= 0; col <= 0;
                        if (rows == 0) state <= S_DONE;
                        else           state <= S_READ;
                    end
                end

                S_READ: begin
                    c_rd_en   <= 1'b1;
                    c_rd_addr <= {row, col};
                    state     <= S_WRITE;
                end

                S_WRITE: begin
                    cbuf_wr_en   <= 1'b1;
                    cbuf_wr_addr <= (base_row + row) * N + col;
                    cbuf_wr_data <= c_rd_data;
                    state        <= S_NEXT;
                end

                S_NEXT: begin
                    if (col == N - 1) begin
                        col <= 0;
                        if (row == rows - 1)
                            state <= S_DONE;
                        else begin
                            row   <= row + 1'b1;
                            state <= S_READ;
                        end
                    end else begin
                        col   <= col + 1'b1;
                        state <= S_READ;
                    end
                end

                S_DONE: begin
                    done      <= 1'b1;
                    started_d <= 1'b0;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
