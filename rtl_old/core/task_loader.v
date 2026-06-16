//=============================================================================
// File     : task_loader.v
// Project  : SPGEMM-Accelerator
// Brief    : Task Descriptor Loader - reads host-computed PE task descriptors
//           from GlobalBuffer and distributes them to PEs.
//
//   Host (software) computes:
//     b_row_nnz[k] = B_row_ptr[k+1] - B_row_ptr[k]
//     row_cyc[i]   = ceil(sum(b_row_nnz[A_col_idx[p]]) / N_MAC)
//     dynamic_target = ceil(remaining_work / remaining_pe)
//     → PE task descriptors per row: {row_start, row_end, a_ptr_start, a_ptr_end}
//
//   Host writes task descriptors to DDR at known address.
//   LOAD_TASK instruction loads them into GlobalBuffer.
//   This module reads them from GlobalBuffer and distributes to PE ports.
//
//   Task descriptor format in GlobalBuffer (5 elements per PE):
//     [0]: row_start   (16-bit)
//     [1]: row_end     (16-bit)
//     [2]: a_ptr_start (16-bit)
//     [3]: a_ptr_end   (16-bit)
//     [4]: {15'd0, valid}  (bit[0] = valid flag)
//=============================================================================

`include "defines.vh"

module task_loader (
    input  wire                      start,
    output reg                       done,

    // PE Task outputs
    output reg  [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_row_start,
    output reg  [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_row_end,
    output reg  [`N_PE-1:0][15:0]              pe_a_ptr_start,
    output reg  [`N_PE-1:0][15:0]              pe_a_ptr_end,
    output reg  [`N_PE-1:0]                    pe_task_valid,

    // Task descriptor base in GlobalBuffer (from LOAD_TASK instruction sram_offset)
    input  wire [`GBUF_DEPTH_LOG-1:0] task_sram_base,

    // GlobalBuffer read
    output reg                       gbuf_rd_en,
    output reg  [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr,
    input  wire [`DATA_WIDTH-1:0]    gbuf_rd_data,
    input  wire                      gbuf_rd_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // State Machine
    //=========================================================================
    localparam ST_IDLE      = 2'd0;
    localparam ST_READ_DESC  = 2'd1;  // Read 5 elements per PE, N_PE times
    localparam ST_DONE      = 2'd2;

    reg [1:0] state, state_next;

    // Current PE being loaded (0..N_PE-1)
    reg [`N_PE_BITS:0] pe_idx;
    // Element within current PE's descriptor (0..4)
    reg [2:0] elem_idx;

    // Latch: accumulate descriptor elements
    reg [`MAX_DIM_BITS-1:0] latch_row_start;
    reg [`MAX_DIM_BITS-1:0] latch_row_end;
    reg [15:0] latch_a_ptr_start;
    reg [15:0] latch_a_ptr_end;
    reg latch_valid;

    // Read address offset: starts from task_sram_base
    wire [`GBUF_DEPTH_LOG-1:0] desc_offset;
    assign desc_offset = task_sram_base + (pe_idx * `TASK_DESC_ELEMENTS) + elem_idx;

    // Number of valid PEs loaded
    reg [`N_PE_BITS:0] valid_pe_count;

    //=========================================================================
    // State transition
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= ST_IDLE;
        end else begin
            state <= state_next;
        end
    end

    always @(*) begin
        state_next = state;
        case (state)
            ST_IDLE: begin
                if (start)
                    state_next = ST_READ_DESC;
            end
            ST_READ_DESC: begin
                // Done when all PEs * 5 elements read
                if (pe_idx == `N_PE - 1 && elem_idx == `TASK_DESC_ELEMENTS - 1 && gbuf_rd_valid)
                    state_next = ST_DONE;
            end
            ST_DONE: begin
                state_next = ST_IDLE;
            end
        endcase
    end

    //=========================================================================
    // Sequential logic
    //=========================================================================
    reg init_done;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            pe_idx    <= 0;
            elem_idx  <= 0;
            latch_row_start <= 0;
            latch_row_end   <= 0;
            latch_a_ptr_start <= 0;
            latch_a_ptr_end   <= 0;
            latch_valid  <= 0;
            valid_pe_count <= 0;
            init_done <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (start && !init_done) begin
                        pe_idx   <= 0;
                        elem_idx <= 0;
                        valid_pe_count <= 0;
                        init_done <= 1'b1;
                        // Clear all PE outputs
                        for (integer p = 0; p < `N_PE; p = p + 1) begin
                            pe_row_start[p]   <= 0;
                            pe_row_end[p]     <= 0;
                            pe_a_ptr_start[p] <= 0;
                            pe_a_ptr_end[p]   <= 0;
                            pe_task_valid[p]  <= 1'b0;
                        end
                    end
                end

                ST_READ_DESC: begin
                    if (gbuf_rd_valid) begin
                        case (elem_idx)
                            3'd0: latch_row_start <= gbuf_rd_data[`MAX_DIM_BITS-1:0];
                            3'd1: latch_row_end   <= gbuf_rd_data[`MAX_DIM_BITS-1:0];
                            3'd2: latch_a_ptr_start <= gbuf_rd_data[15:0];
                            3'd3: latch_a_ptr_end   <= gbuf_rd_data[15:0];
                            3'd4: begin
                                latch_valid <= gbuf_rd_data[0];
                                // Commit to PE output
                                pe_row_start[pe_idx]   <= latch_row_start;
                                pe_row_end[pe_idx]     <= latch_row_end;
                                pe_a_ptr_start[pe_idx] <= latch_a_ptr_start;
                                pe_a_ptr_end[pe_idx]   <= latch_a_ptr_end;
                                pe_task_valid[pe_idx]  <= gbuf_rd_data[0];
                                if (gbuf_rd_data[0])
                                    valid_pe_count <= valid_pe_count + 1;
                            end
                        endcase

                        // Advance element/PE index
                        if (elem_idx == `TASK_DESC_ELEMENTS - 1) begin
                            elem_idx <= 0;
                            pe_idx   <= pe_idx + 1;
                        end else begin
                            elem_idx <= elem_idx + 1;
                        end
                    end
                end

                ST_DONE: begin
                    init_done <= 1'b0;
                end
            endcase
        end
    end

    //=========================================================================
    // GlobalBuffer read
    //=========================================================================
    always @(*) begin
        gbuf_rd_en   = 1'b0;
        gbuf_rd_addr = 0;

        if (state == ST_READ_DESC) begin
            gbuf_rd_en   = 1'b1;
            gbuf_rd_addr = desc_offset;
        end
    end

    //=========================================================================
    // Done
    //=========================================================================
    always @(*) begin
        done = (state == ST_DONE);
    end

endmodule
