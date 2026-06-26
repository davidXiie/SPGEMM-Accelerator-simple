//=============================================================================
// File     : pe_task_packer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Task Packer — receives individual tasks from B streamer,
//            packs into 4-lane task groups, writes to task_group_fifo.
//            Supports flush of partial group at end of row.
//
//   Interface:
//     task_in_valid/ready/data (64-bit)
//     group_wr_en/data (260-bit)
//     flush_pack — signal to flush remaining 1~3 tasks
//=============================================================================

`include "defines.vh"

module pe_task_packer (
    input  wire                     task_in_valid,
    output wire                     task_in_ready,
    input  wire [`TASK_WIDTH-1:0]   task_in_data,

    output reg                      group_wr_en,
    output reg  [`TASK_GROUP_WIDTH-1:0] group_wr_data,
    input  wire                     group_fifo_full,

    input  wire                     flush_pack,
    output reg                      flush_done,

    input  wire                     aclk,
    input  wire                     aresetn
);

    reg [1:0]  pack_count;
    reg [3:0]  pack_valid;
    reg [`TASK_WIDTH-1:0] pack_task0;
    reg [`TASK_WIDTH-1:0] pack_task1;
    reg [`TASK_WIDTH-1:0] pack_task2;
    reg [`TASK_WIDTH-1:0] pack_task3;

    // Can accept if FIFO not full AND not currently flushing
    wire flush_active = flush_pack;
    assign task_in_ready = !group_fifo_full && !flush_active;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            pack_count   <= 2'd0;
            pack_valid   <= 4'b0000;
            pack_task0   <= 0;
            pack_task1   <= 0;
            pack_task2   <= 0;
            pack_task3   <= 0;
            group_wr_en  <= 1'b0;
            group_wr_data <= 0;
            flush_done   <= 1'b0;
        end else begin
            group_wr_en  <= 1'b0;
            flush_done   <= 1'b0;

            // Normal pack: receive individual tasks
            if (task_in_valid && task_in_ready) begin
                case (pack_count)
                    2'd0: begin
                        pack_task0     <= task_in_data;
                        pack_valid[0]  <= 1'b1;
                        pack_count     <= 2'd1;
                    end
                    2'd1: begin
                        pack_task1     <= task_in_data;
                        pack_valid[1]  <= 1'b1;
                        pack_count     <= 2'd2;
                    end
                    2'd2: begin
                        pack_task2     <= task_in_data;
                        pack_valid[2]  <= 1'b1;
                        pack_count     <= 2'd3;
                    end
                    2'd3: begin
                        pack_task3     <= task_in_data;
                        // Full group: write to FIFO
                        if (!group_fifo_full) begin
                            group_wr_en <= 1'b1;
                            group_wr_data[3:0]         <= 4'b1111;
                            group_wr_data[67:4]        <= pack_task0;
                            group_wr_data[131:68]      <= pack_task1;
                            group_wr_data[195:132]     <= pack_task2;
                            group_wr_data[259:196]     <= task_in_data;
                            pack_count   <= 2'd0;
                            pack_valid   <= 4'b0000;
                        end
                    end
                endcase
            end

            // Flush: remaining 1~3 tasks at end of A row
            if (flush_pack && pack_count != 0 && !group_fifo_full) begin
                group_wr_en <= 1'b1;
                group_wr_data[3:0]       <= pack_valid;
                group_wr_data[67:4]      <= pack_task0;
                group_wr_data[131:68]    <= pack_task1;
                group_wr_data[195:132]   <= pack_task2;
                group_wr_data[259:196]   <= pack_task3;
                pack_count   <= 2'd0;
                pack_valid   <= 4'b0000;
                flush_done   <= 1'b1;
            end else if (flush_pack && pack_count == 0) begin
                flush_done <= 1'b1;
            end
        end
    end

endmodule
