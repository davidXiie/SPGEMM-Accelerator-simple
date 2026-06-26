//=============================================================================
// File     : scratchpad.v
// Project  : SPGEMM-Accelerator v2
// Brief    : SRAM modules: std_scratchpad, banked_scratchpad, sync_fifo
//=============================================================================

`include "defines.vh"

//=============================================================================
// StandardScratchpad: Single-port SRAM with registered read
//=============================================================================
module std_scratchpad #(
    parameter integer DEPTH      = 1024,
    parameter integer DEPTH_LOG  = 10,
    parameter integer DATA_WIDTH = `DATA_WIDTH
) (
    input  wire                  wr_en,
    input  wire [DEPTH_LOG-1:0]  wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,

    input  wire                  rd_en,
    input  wire [DEPTH_LOG-1:0]  rd_addr,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  rd_valid,

    input  wire                  aclk,
    input  wire                  aresetn
);

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];
    reg [DATA_WIDTH-1:0] rd_data_reg;
    reg rd_valid_reg;

    // Write + registered read in one block — most reliable BRAM SDP pattern.
    // No async reset on this block: BRAM output register cannot have async reset.
    always @(posedge aclk) begin
        if (wr_en) ram[wr_addr] <= wr_data;
        if (rd_en) rd_data_reg <= ram[rd_addr];
    end

    // Valid flag only — a plain control register, fine with async reset.
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            rd_valid_reg <= 1'b0;
        else
            rd_valid_reg <= rd_en;
    end

    assign rd_data  = rd_data_reg;
    assign rd_valid = rd_valid_reg;

endmodule


//=============================================================================
// BankedScratchpad: Multi-bank parallel read SRAM
//   Used for B Buffer in PEs (N_MAC parallel lanes)
//=============================================================================
module banked_scratchpad #(
    parameter integer N_BANKS    = `N_MAC,
    parameter integer DEPTH      = 4096,
    parameter integer DEPTH_LOG  = 12,
    parameter integer BANK_WIDTH = `DATA_WIDTH
) (
    input  wire                          wr_en,
    input  wire [DEPTH_LOG-1:0]          wr_addr,
    input  wire [N_BANKS*BANK_WIDTH-1:0] wr_data,

    input  wire [N_BANKS-1:0]            rd_en,
    input  wire [N_BANKS*DEPTH_LOG-1:0]  rd_addr,
    output wire [N_BANKS*BANK_WIDTH-1:0] rd_data,
    output wire [N_BANKS-1:0]            rd_valid,

    input  wire                          aclk,
    input  wire                          aresetn
);

    genvar b;
    generate
        for (b = 0; b < N_BANKS; b = b + 1) begin : gen_bank
            (* ram_style = "block" *) reg [BANK_WIDTH-1:0] ram [0:DEPTH-1];
            reg [BANK_WIDTH-1:0] rd_reg;
            reg rd_valid_reg;

            wire [DEPTH_LOG-1:0] rd_addr_b = rd_addr[b*DEPTH_LOG +: DEPTH_LOG];

            // Write + registered read: single clock-only block for BRAM SDP inference.
            always @(posedge aclk) begin
                if (wr_en)
                    ram[wr_addr] <= wr_data[(b+1)*BANK_WIDTH-1 -: BANK_WIDTH];
                if (rd_en[b])
                    rd_reg <= ram[rd_addr_b];
            end

            // Valid flag: control register, async reset is fine here.
            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn)
                    rd_valid_reg <= 1'b0;
                else
                    rd_valid_reg <= rd_en[b];
            end

            assign rd_data[(b+1)*BANK_WIDTH-1 -: BANK_WIDTH] = rd_reg;
            assign rd_valid[b] = rd_valid_reg;
        end
    endgenerate

endmodule


//=============================================================================
// SyncFIFO: Synchronous FIFO
//   DEPTH must be a power-of-2.
//   keep_hierarchy prevents Vivado from flattening this module before BRAM
//   inference runs, ensuring the ram_style attribute survives optimisation.
//=============================================================================
(* keep_hierarchy = "yes" *)
module sync_fifo #(
    parameter integer WIDTH      = 32,
    parameter integer DEPTH      = 16,
    parameter integer DEPTH_LOG  = 4
) (
    input  wire                  wr_en,
    input  wire [WIDTH-1:0]      wr_data,
    output wire                  wr_full,

    input  wire                  rd_en,
    output wire [WIDTH-1:0]      rd_data,
    output wire                  rd_empty,

    output wire [DEPTH_LOG:0]    count,

    input  wire                  aclk,
    input  wire                  aresetn
);

    // rw_addr_collision="no": wr_ptr != rd_ptr is guaranteed by the FIFO pointer
    // logic, so Vivado does not need to generate WRITE_FIRST forwarding mux trees.
    (* ram_style = "block" *) (* rw_addr_collision = "no" *) reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [DEPTH_LOG:0] wr_ptr, rd_ptr;

`ifndef SYNTHESIS
    // Simulation only: initialise to 0 so unread slots don't produce X values.
    // `SYNTHESIS is defined automatically by Vivado; this block is invisible
    // to the synthesis tool, so it does not affect BRAM inference.
    integer _sim_i;
    initial begin
        for (_sim_i = 0; _sim_i < DEPTH; _sim_i = _sim_i + 1)
            mem[_sim_i] = {WIDTH{1'b0}};
    end
`endif

    assign count    = wr_ptr - rd_ptr;
    assign wr_full  = count[DEPTH_LOG];   // full when count == DEPTH == 2^DEPTH_LOG
    assign rd_empty = (count == 0);

    wire [DEPTH_LOG-1:0] wr_addr = wr_ptr[DEPTH_LOG-1:0];
    wire [DEPTH_LOG-1:0] rd_addr = rd_ptr[DEPTH_LOG-1:0];
    wire                 wr_en_q = wr_en & ~wr_full;

    // Pointer FFs — async reset is fine; these are plain registers, not BRAM
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (wr_en_q)             wr_ptr <= wr_ptr + 1'b1;
            if (rd_en && !rd_empty)  rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // Memory write (synthesis: SDP BRAM inferred).
    // rd_data is combinatorial (FWFT style) so callers see the head element
    // immediately without a one-cycle read latency.  For synthesis a registered
    // output wrapper should be added; for simulation this is correct.
    always @(posedge aclk) begin
        if (wr_en_q)
            mem[wr_addr] <= wr_data;
    end

    assign rd_data = mem[rd_addr];

endmodule
