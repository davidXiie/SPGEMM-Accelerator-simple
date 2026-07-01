//=============================================================================
// File     : axi_c_drain.v
// Brief    : Reads C results from PE banks and writes to DDR via AXI4 Write.
//            Replaces the simulation-only c_rd_* Python drain path.
//
//   Output format: dense C matrix, row-major, FP16 values.
//   Each 512-bit AXI beat holds 32 consecutive FP16 values (2 C-bank reads).
//   DDR byte address for C[r][c]: C_BASE + (r * N + c) * 2
//
//   FSM: IDLE → loop { RD_G0, RD_G1, WR_AW, WR_W, WR_B } → DONE
//   Reads two 16-element groups → packs into one 512-bit beat → AXI write.
//=============================================================================

`include "defines.vh"

module axi_c_drain #(
    parameter N_PE             = `N_PE,
    parameter MAX_DIM_BITS     = `MAX_DIM_BITS,
    parameter C_ROW_ADDR_BITS  = `C_ROW_ADDR_BITS
) (
    input  wire clk, input wire rst_n,
    input  wire                              start,
    output reg                               done,

    input  wire [MAX_DIM_BITS-1:0]           M, N,
    input  wire [N_PE*16-1:0]               row_counts,

    // === PE C read ports (master side) ===
    output reg  [N_PE-1:0]                           c_rd_en,
    output reg  [N_PE*(C_ROW_ADDR_BITS+5)-1:0]      c_rd_addr,
    input  wire [N_PE*16*16-1:0]                     c_rd_data,
    input  wire [N_PE*`MAX_DIM_BITS-1:0]             c_rd_row,

    // === AXI4 Write Master ===
    output reg  [3:0]              axi_awid,
    output reg  [63:0]             axi_awaddr,
    output reg  [7:0]              axi_awlen,
    output reg  [2:0]              axi_awsize,
    output reg                     axi_awvalid,
    input  wire                    axi_awready,
    output reg  [511:0]            axi_wdata,
    output reg  [63:0]             axi_wstrb,
    output reg                     axi_wlast,
    output reg                     axi_wvalid,
    input  wire                    axi_wready,
    input  wire [3:0]              axi_bid,
    input  wire [1:0]              axi_bresp,
    input  wire                    axi_bvalid,
    output reg                     axi_bready
);

    //=========================================================================
    // Local parameters
    //=========================================================================
    localparam C_RD_ADDR_W     = C_ROW_ADDR_BITS + 5;
    localparam PE_IDX_W        = (N_PE > 1) ? $clog2(N_PE) : 1;
    // C output DDR base (byte address); word 0x0030_0000 × 2 = 0x0060_0000
    localparam [63:0] C_BYTE_BASE = 64'h0060_0000;

    //=========================================================================
    // FSM
    //=========================================================================
    localparam S_IDLE      = 4'd0;
    localparam S_RD_PRIME1 = 4'd1;  // assert c_rd_en, addr=G0 (prime cycle 1)
    localparam S_RD_PRIME2 = 4'd2;  // hold (prime cycle 2, G0 data ready next)
    localparam S_RD_G0     = 4'd3;  // latch G0, set addr=G1
    localparam S_RD_G1     = 4'd4;  // hold addr=G1 (wait for G1 data)
    localparam S_RD_G2     = 4'd5;  // latch G1, deassert c_rd_en, pack & issue AW+W
    localparam S_WR_B      = 4'd6;  // wait for AW+W+B handshake
    localparam S_NEXT      = 4'd7;  // advance counters
    localparam S_DONE      = 4'd8;

    reg [3:0] state;

    //=========================================================================
    // Counters
    //=========================================================================
    reg [PE_IDX_W-1:0] pid;              // current PE
    reg [15:0]         lrow;             // local row within PE
    reg [15:0]         gpair;            // group-pair index (0, 1, 2, ...)
    reg [MAX_DIM_BITS-1:0] global_r;     // global C row
    reg [4:0]          ngroups;          // ceil(N/32) — groups of 32 cols per beat
    reg [4:0]          npairs;           // ceil(ngroups/2)

    // Data pipeline
    reg [255:0]        g0_data;          // latched first group data

    //=========================================================================
    // Derived (combinational)
    //=========================================================================
    // c_rd_addr = {local_row[7:0], group[4:0]} per PE
    function [C_RD_ADDR_W-1:0] make_c_addr;
        input [7:0] lr;
        input [4:0] grp;
        begin
            make_c_addr = {lr[C_ROW_ADDR_BITS-1:0], grp[4:0]};
        end
    endfunction

    wire aw_fire = axi_awvalid && axi_awready;
    wire w_fire  = axi_wvalid  && axi_wready;
    wire b_fire  = axi_bvalid  && axi_bready;

    //=========================================================================
    // Main FSM
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE; done <= 1'b0;
            c_rd_en  <= 0;
            axi_awvalid <= 1'b0; axi_wvalid <= 1'b0; axi_bready <= 1'b0;
            axi_awid <= 0; axi_awaddr <= 0; axi_awlen <= 0; axi_awsize <= 0;
            axi_wdata <= 0; axi_wstrb <= 0; axi_wlast <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                //=============================================================
                // IDLE: init counters
                //=============================================================
                S_IDLE: begin
                    if (start) begin
                        pid <= 0; lrow <= 0; gpair <= 0;
                        // ngroups = number of 32-element beats = ceil(N/32)
                        ngroups <= (N[4:0] == 0) ? N[9:5] : (N[9:5] + 1);
                        npairs  <= (N[4:0] == 0) ? N[9:5] : (N[9:5] + 1);
                        state <= S_RD_PRIME1;
                    end
                end

                //=============================================================
                // RD_PRIME1: assert c_rd_en/addr (prime cycle 1 of 2)
                //=============================================================
                S_RD_PRIME1: begin
                    c_rd_en   <= 1 << pid;
                    c_rd_addr <= make_c_addr(lrow, gpair << 1) << (pid * C_RD_ADDR_W);
                    state     <= S_RD_PRIME2;
                end

                //=============================================================
                // RD_PRIME2: hold c_rd_en/addr (prime cycle 2 of 2)
                //=============================================================
                S_RD_PRIME2: begin
                    state <= S_RD_G0;
                end

                //=============================================================
                // RD_G0: latch G0 data (valid after RD_PRIME primed the read),
                //        issue second group read for G1.
                //=============================================================
                S_RD_G0: begin
                    // Latch first group (now valid after RD_PRIME cycle)
                    g0_data   <= c_rd_data[pid*256 +: 256];
                    global_r  <= c_rd_row[pid*MAX_DIM_BITS +: MAX_DIM_BITS];
                    // Issue second group read
                    c_rd_addr <= make_c_addr(lrow, (gpair << 1) + 1) << (pid * C_RD_ADDR_W);
                    state     <= S_RD_G1;
                end

                //=============================================================
                // RD_G1: latch G1 data (valid from S_RD_G0's addr change),
                //        pack beat, deassert c_rd_en, issue AW+W.
                //=============================================================
                //=============================================================
                // RD_G1: hold c_rd_en/addr=G1 (wait for G1 data, 2nd cycle)
                //=============================================================
                S_RD_G1: begin
                    state <= S_RD_G2;
                end

                //=============================================================
                // RD_G2: latch G1 (valid after 2 cycles at G1 addr),
                //        deassert c_rd_en, pack beat, issue AW+W.
                //=============================================================
                S_RD_G2: begin
                    c_rd_en   <= 0;
                    // Pack: {G1, G0} → 512-bit beat
                    axi_awid    <= 4'd0;
                    axi_awaddr  <= C_BYTE_BASE + (global_r * ngroups * 32 + gpair * 32) * 2;
                    axi_awlen   <= 8'd0;         // single beat
                    axi_awsize  <= 3'd6;         // 64 bytes
                    axi_awvalid <= 1'b1;
                    // W channel (same cycle, single-beat)
                    axi_wdata   <= {c_rd_data[pid*256 +: 256], g0_data};
                    axi_wstrb   <= 64'hFFFF_FFFF_FFFF_FFFF;
                    axi_wlast   <= 1'b1;
                    axi_wvalid  <= 1'b1;
                    axi_bready  <= 1'b1;
                    state <= S_WR_B;
                end

                //=============================================================
                // WR_B: wait for AW+W handshake then B response
                //=============================================================
                S_WR_B: begin
                    if (aw_fire) begin axi_awvalid <= 1'b0; end
                    if (w_fire)  begin axi_wvalid  <= 1'b0; end
                    if (b_fire)  begin
                        axi_bready <= 1'b0;
                        state <= S_NEXT;
                    end
                end

                //=============================================================
                // NEXT: advance to next group-pair / row / PE
                //=============================================================
                S_NEXT: begin
                    if (gpair + 1 < npairs) begin
                        gpair <= gpair + 1;
                        state <= S_RD_PRIME1;
                    end else if (lrow + 1 < row_counts[pid*16 +: 16]) begin
                        lrow  <= lrow + 1;
                        gpair <= 0;
                        state <= S_RD_PRIME1;
                    end else if (pid + 1 < N_PE) begin
                        pid   <= pid + 1;
                        lrow  <= 0;
                        gpair <= 0;
                        state <= S_RD_PRIME1;
                    end else begin
                        state <= S_DONE;
                    end
                end

                //=============================================================
                // DONE
                //=============================================================
                S_DONE: begin done <= 1'b1; end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
