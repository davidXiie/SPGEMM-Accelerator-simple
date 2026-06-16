//=============================================================================
// File     : c_csr_writer.v
// Project  : SPGEMM-Accelerator
// Brief    : C CSR Writer — collects PE/ElementWise output during COMPUTE,
//           generates standard CSR in Output Buffer after compute done.
//
//   Output Buffer layout (standard CSR, 32-bit row_ptr):
//     [0..2*(M+1)-1] : C_row_ptr[0..M]  (M+1 entries, 32-bit = 2×16-bit each)
//     [2*(M+1)..]    : (col,val) pairs interleaved
//
//   Phases:
//     COLLECT:   round-robin scan PEs, write (col,val) to OBuf, track row_nnz
//     PREFIX:    after compute_done && collect idle, compute prefix sum row_ptr
//     DONE
//=============================================================================

`include "defines.vh"

module c_csr_writer (
    input  wire                          start,
    output reg                           done,

    input  wire [`N_PE-1:0]              pe_row_valid,
    input  wire [`N_PE-1:0]              pe_row_end,
    input  wire [`N_PE*`MAX_DIM_BITS-1:0] pe_row_id,
    input  wire [`N_PE*`MAX_DIM_BITS-1:0] pe_nnz,
    input  wire [`N_PE*`DATA_WIDTH-1:0]  pe_col,
    input  wire [`N_PE*`DATA_WIDTH-1:0]  pe_val,

    input  wire [`MAX_DIM_BITS-1:0]      M,
    input  wire                          pe_all_done,
    input  wire                          compute_done,

    output reg                           obuf_wr_en,
    output reg  [`OUTBUF_DEPTH_LOG-1:0]  obuf_wr_addr,
    output reg  [`DATA_WIDTH-1:0]        obuf_wr_data,

    input  wire                          aclk,
    input  wire                          aresetn
);

    localparam ST_IDLE        = 3'd0;
    localparam ST_COLLECT_SCAN = 3'd1;
    localparam ST_COLLECT_WRC  = 3'd2;
    localparam ST_COLLECT_WRV  = 3'd3;
    localparam ST_PREFIX_LO    = 3'd4;   // write low 16-bit of 32-bit row_ptr
    localparam ST_PREFIX_HI    = 3'd5;   // write high 16-bit, advance row
    localparam ST_DONE         = 3'd6;

    reg [2:0] state, state_next;

    // Round-robin
    reg [`N_PE_BITS-1:0] scan_pe;
    reg                  all_idle;
    reg [`DATA_WIDTH-1:0] val_latch;

    // Row tracking
    reg [`CSR_NNZ_BITS-1:0] c_row_nnz [`MAX_M-1:0];
    reg [`MAX_M-1:0]          row_seen;

    // 32-bit row_ptr area = 2*(M+1) entries; col/val starts after that
    wire [`OUTBUF_DEPTH_LOG-1:0] COL_VAL_BASE;
    assign COL_VAL_BASE = 2 * (M + 1);
    reg [`OUTBUF_DEPTH_LOG-1:0] wr_ptr;

    // Prefix sum: 2 × 16-bit writes per row
    reg [`MAX_DIM_BITS:0] prefix_idx;
    reg [`CSR_ADDR_BITS-1:0] prefix_acc;
    reg [`CSR_ADDR_BITS-1:0] prefix_val32;  // 32-bit value for current row

    wire [`MAX_DIM_BITS-1:0] rid;
    wire [`CSR_NNZ_BITS-1:0]  nnz;
    wire [`DATA_WIDTH-1:0]   col, val;
    assign rid = pe_row_id[scan_pe*`MAX_DIM_BITS +: `MAX_DIM_BITS];
    assign nnz = pe_nnz[scan_pe*`MAX_DIM_BITS +: `CSR_NNZ_BITS];
    assign col = pe_col[scan_pe*`DATA_WIDTH +: `DATA_WIDTH];
    assign val = pe_val[scan_pe*`DATA_WIDTH +: `DATA_WIDTH];

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
            ST_IDLE: if (start) state_next = ST_COLLECT_SCAN;
            ST_COLLECT_SCAN: begin
                if (compute_done && all_idle && scan_pe == `N_PE - 1 && !pe_row_valid[scan_pe])
                    state_next = ST_PREFIX_LO;
                else if (pe_row_valid[scan_pe])
                    state_next = ST_COLLECT_WRC;
            end
            ST_COLLECT_WRC:  state_next = ST_COLLECT_WRV;
            ST_COLLECT_WRV:  state_next = ST_COLLECT_SCAN;
            ST_PREFIX_LO:    state_next = ST_PREFIX_HI;
            ST_PREFIX_HI:    state_next = (prefix_idx > M) ? ST_DONE : ST_PREFIX_LO;
            ST_DONE:         state_next = ST_IDLE;
        endcase
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            scan_pe       <= 0;
            all_idle      <= 1'b1;
            val_latch     <= 0;
            wr_ptr        <= 2 * (`MAX_M + 1);
            prefix_idx    <= 0;
            prefix_acc    <= 0;
            obuf_wr_en    <= 1'b0;
            obuf_wr_addr  <= 0;
            obuf_wr_data  <= 0;
            done          <= 1'b0;
        end else begin
            obuf_wr_en <= 1'b0;
            done       <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        scan_pe       <= 0;
                        all_idle      <= 1'b1;
                        wr_ptr        <= COL_VAL_BASE;
                        prefix_idx    <= 0;
                        prefix_acc    <= 0;
                        for (integer i = 0; i < `MAX_M; i = i + 1) begin
                            row_seen[i]  <= 1'b0;
                            c_row_nnz[i] <= 0;
                        end
                    end
                end

                ST_COLLECT_SCAN: begin
                    if (pe_row_valid[scan_pe]) begin
                        obuf_wr_en   <= 1'b1;
                        obuf_wr_addr <= wr_ptr;
                        obuf_wr_data <= col;
                        val_latch    <= val;
                        if (!row_seen[rid] && pe_row_end[scan_pe]) begin
                            row_seen[rid]  <= 1'b1;
                            c_row_nnz[rid] <= nnz;
                        end
                        all_idle <= 1'b0;
                    end else begin
                        if (scan_pe == `N_PE - 1) begin
                            scan_pe  <= 0;
                            all_idle <= 1'b1;
                        end else
                            scan_pe <= scan_pe + 1;
                    end
                end

                ST_COLLECT_WRC: begin
                    obuf_wr_en   <= 1'b1;
                    obuf_wr_addr <= wr_ptr + 1;
                    obuf_wr_data <= val_latch;
                    wr_ptr <= wr_ptr + 2;
                end

                ST_COLLECT_WRV: begin
                    if (scan_pe == `N_PE - 1) begin
                        scan_pe  <= 0;
                        all_idle <= 1'b1;
                    end else
                        scan_pe <= scan_pe + 1;
                end

                //=== Prefix sum: 32-bit row_ptr = 2 × 16-bit writes ===
                ST_PREFIX_LO: begin
                    prefix_val32 <= prefix_acc;
                    obuf_wr_en   <= 1'b1;
                    obuf_wr_addr <= prefix_idx << 1;
                    obuf_wr_data <= prefix_acc[`DATA_WIDTH-1:0];
                    if (prefix_idx < M)
                        prefix_acc <= prefix_acc + c_row_nnz[prefix_idx];
                end

                ST_PREFIX_HI: begin
                    obuf_wr_en   <= 1'b1;
                    obuf_wr_addr <= (prefix_idx << 1) + 1;
                    obuf_wr_data <= prefix_val32[`CSR_ADDR_BITS-1:`DATA_WIDTH];
                    prefix_idx <= prefix_idx + 1;
                end

                ST_DONE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end

endmodule
