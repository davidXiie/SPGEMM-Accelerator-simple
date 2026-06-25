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

    reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];
    reg [DATA_WIDTH-1:0] rd_data_reg;
    reg rd_valid_reg;

    always @(posedge aclk) begin
        if (wr_en) ram[wr_addr] <= wr_data;
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rd_data_reg  <= 0;
            rd_valid_reg <= 1'b0;
        end else begin
            rd_valid_reg <= rd_en;
            if (rd_en) rd_data_reg <= ram[rd_addr];
        end
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
            reg [BANK_WIDTH-1:0] ram [0:DEPTH-1];
            reg [BANK_WIDTH-1:0] rd_reg;
            reg rd_valid_reg;

            wire [DEPTH_LOG-1:0] rd_addr_b = rd_addr[b*DEPTH_LOG +: DEPTH_LOG];

            always @(posedge aclk) begin
                if (wr_en)
                    ram[wr_addr] <= wr_data[(b+1)*BANK_WIDTH-1 -: BANK_WIDTH];
            end

            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn) begin
                    rd_reg       <= 0;
                    rd_valid_reg <= 1'b0;
                end else begin
                    rd_valid_reg <= rd_en[b];
                    if (rd_en[b]) rd_reg <= ram[rd_addr_b];
                end
            end

            assign rd_data[(b+1)*BANK_WIDTH-1 -: BANK_WIDTH] = rd_reg;
            assign rd_valid[b] = rd_valid_reg;
        end
    endgenerate

endmodule


//=============================================================================
// SyncFIFO: Synchronous FIFO
//=============================================================================
module sync_fifo #(
    parameter integer WIDTH      = 32,
    parameter integer DEPTH      = 16,
    parameter integer DEPTH_LOG  = 4
) (
    input  wire                  wr_en,
    input  wire [WIDTH-1:0]      wr_data,
    output wire                  wr_full,

    input  wire                  rd_en,
    output reg  [WIDTH-1:0]      rd_data,
    output wire                  rd_empty,

    output wire [DEPTH_LOG:0]    count,

    input  wire                  aclk,
    input  wire                  aresetn
);

    (* ram_style = "block" *) reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [DEPTH_LOG:0] wr_ptr, rd_ptr;

    // Simulation determinism: zero-initialize RAM (ignored by synthesis,
    // FPGA BRAM initializes to zero on configuration).
    integer si;
    initial begin
        for (si = 0; si < DEPTH; si = si + 1)
            mem[si] = 0;
    end

    assign count    = wr_ptr - rd_ptr;
    assign wr_full  = (count >= DEPTH);
    assign rd_empty = (count == 0);

    // Pointers: async reset is fine for plain FFs
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (wr_en && !wr_full)  wr_ptr <= wr_ptr + 1'b1;
            if (rd_en && !rd_empty) rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // Memory write (separate block, no reset → clean BRAM write port)
    always @(posedge aclk) begin
        if (wr_en && !wr_full)
            mem[wr_ptr[DEPTH_LOG-1:0]] <= wr_data;
    end

    // Memory read: SEPARATE block, clock-edge only, NO async reset.
    // Xilinx BRAM output registers do not support async reset;
    // mixing rd_data with aresetn prevents BRAM inference.
    always @(posedge aclk) begin
        rd_data <= mem[rd_ptr[DEPTH_LOG-1:0]];
    end

endmodule
