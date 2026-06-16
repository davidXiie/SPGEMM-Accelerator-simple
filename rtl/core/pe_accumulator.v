//=============================================================================
// File     : pe_accumulator.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Serial Accumulator with internal acc_buf[512] (16-bit per entry).
//            Accumulate: one product per cycle, read-modify-write acc_buf[col].
//            Clear: sequential 512-cycle zero-fill via clear_en.
//            Provides read port for row writeback (pe_top reads all 512 cols).
//
//   States: ACC_IDLE → ACC_ADD → ACC_WRITE → ACC_IDLE
//=============================================================================

`include "defines.vh"

module pe_accumulator (
    // Accumulation input (from serializer)
    input  wire                     acc_in_valid,
    output wire                     acc_in_ready,
    input  wire [`PRODUCT_WIDTH-1:0] acc_in_data,

    output wire                     idle,
    output wire                     all_drain_empty,

    // Clear control (from PE FSM)
    input  wire                     clear_en,
    output wire                     clear_done,

    // Writeback read port (PE_WRITE_ROW reads acc_buf[col])
    input  wire [`PE_ACC_ADDR_BITS-1:0] wb_rd_addr,
    output wire [`DATA_WIDTH-1:0]   wb_rd_data,

    input  wire                     aclk,
    input  wire                     aresetn
);

    // Internal accumulator buffer
    reg [`DATA_WIDTH-1:0] acc_buf [0:`PE_ACC_DEPTH-1];

    // FP16 add: product = {col_id[15:0], product_val[15:0]}
    wire [`DATA_WIDTH-1:0] product_val = acc_in_data[15:0];
    wire [`DATA_WIDTH-1:0] product_col = acc_in_data[31:16];

    localparam ACC_IDLE  = 2'd0;
    localparam ACC_ADD   = 2'd1;
    localparam ACC_WRITE = 2'd2;

    reg [1:0] acc_state;

    reg [`DATA_WIDTH-1:0] acc_col_reg;
    reg [`DATA_WIDTH-1:0] acc_old_reg;
    reg [`DATA_WIDTH-1:0] acc_delta_reg;
    reg [`DATA_WIDTH-1:0] acc_new_reg;

    // Clear
    reg [`PE_ACC_ADDR_BITS-1:0] clear_idx;
    reg clear_active;

    // Integer add (sufficient for small values [1..7]).
    // Replace with FP16 adder IP for synthesis with real FP16 data.
    wire [`DATA_WIDTH-1:0] fp16_add_y;
    assign fp16_add_y = acc_old_reg + acc_delta_reg;

    assign acc_in_ready = (acc_state == ACC_IDLE) && !clear_active;
    assign idle = (acc_state == ACC_IDLE) && !clear_active;
    assign all_drain_empty = idle;

    assign clear_done = clear_active && (clear_idx == `PE_ACC_DEPTH - 1);

    // Writeback read (combinational)
    assign wb_rd_data = acc_buf[wb_rd_addr];

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            acc_state    <= ACC_IDLE;
            acc_col_reg  <= 0;
            acc_old_reg  <= 0;
            acc_delta_reg <= 0;
            acc_new_reg  <= 0;
            clear_idx    <= 0;
            clear_active <= 1'b0;
        end else begin
            // Clear phase
            if (clear_en && !clear_active) begin
                clear_active <= 1'b1;
                clear_idx   <= 0;
            end
            if (clear_active) begin
                acc_buf[clear_idx] <= 16'd0;
                if (clear_idx < `PE_ACC_DEPTH - 1)
                    clear_idx <= clear_idx + 1'b1;
                else
                    clear_active <= 1'b0;
            end

            // Accumulation
            case (acc_state)
                ACC_IDLE: begin
                    if (acc_in_valid && acc_in_ready) begin
                        acc_col_reg   <= product_col;
                        acc_delta_reg <= product_val;
                        acc_old_reg   <= acc_buf[product_col[`PE_ACC_ADDR_BITS-1:0]];
                        acc_state     <= ACC_ADD;
                    end
                end

                ACC_ADD: begin
                    acc_new_reg <= fp16_add_y;
                    acc_state   <= ACC_WRITE;
                end

                ACC_WRITE: begin
                    acc_buf[acc_col_reg[`PE_ACC_ADDR_BITS-1:0]] <= acc_new_reg;
                    acc_state <= ACC_IDLE;
                end
            endcase
        end
    end

endmodule
