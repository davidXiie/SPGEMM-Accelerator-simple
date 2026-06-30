//=============================================================================
// File     : ddr_model.v
// Brief    : Simple AXI4 Slave BRAM to simulate DDR for Cocotb testing.
//           - Host write port: Cocotb pre-loads matrix data
//           - AXI4 Read port:  axi_loader reads A/B data during LOAD phase
//           - Host read port:  Cocotb reads back C results
//
//   AXI address map:
//     A desc  : 0x0000_0000  (512 × 64-bit = 4096 bytes)
//     A col/val: 0x0000_1000  (78643 × 16-bit × 2 interleaved)
//     B desc  : 0x0020_0000  (512 × 32-bit = 2048 bytes)
//     B col/val: 0x0020_1000  (40960 × 16-bit × 2 interleaved)
//=============================================================================

`include "defines.vh"

module ddr_model #(
    parameter MEM_DEPTH = 22,    // 2^22 = 4M entries × 16-bit = 8MB
    parameter MEM_AW    = 22
) (
    input  wire clk,
    input  wire rst_n,

    // === Host write port (Cocotb pre-loads test data) ===
    input  wire                     host_wr_en,
    input  wire [MEM_AW-1:0]       host_wr_addr,   // 16-bit word address
    input  wire [15:0]             host_wr_data,

    // === Host read port (Cocotb reads C results) ===
    input  wire [MEM_AW-1:0]       host_rd_addr,
    output reg  [15:0]             host_rd_data,

    // === AXI4 Read address channel ===
    input  wire [3:0]              axi_arid,
    input  wire [63:0]             axi_araddr,
    input  wire [7:0]              axi_arlen,     // burst length - 1
    input  wire                    axi_arvalid,
    output reg                     axi_arready,

    // === AXI4 Read data channel ===
    output reg  [3:0]              axi_rid,
    output reg  [511:0]            axi_rdata,
    output reg  [1:0]              axi_rresp,
    output reg                     axi_rlast,
    output reg                     axi_rvalid,
    input  wire                    axi_rready
);

    // 16-bit wide memory (Cocotb writes 16-bit, AXI reads 512-bit = 32 consecutive words)
    reg [15:0] mem [0:(1<<MEM_AW)-1];

    // Host write
    always @(posedge clk) begin
        if (host_wr_en)
            mem[host_wr_addr] <= host_wr_data;
    end

    // Host read (registered)
    always @(posedge clk) begin
        host_rd_data <= mem[host_rd_addr];
    end

    // AXI read state machine
    localparam AR_IDLE   = 2'd0;
    localparam AR_BURST  = 2'd1;

    reg [1:0]  ar_state;
    reg [63:0] ar_addr;
    reg [7:0]  ar_cnt;       // current beat count
    reg [7:0]  ar_len;       // total beats - 1

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_state  <= AR_IDLE;
            axi_arready <= 1'b0;
            axi_rvalid  <= 1'b0;
            axi_rlast   <= 1'b0;
            axi_rid     <= 4'd0;
            axi_rdata   <= 512'd0;
            axi_rresp   <= 2'b00;
            ar_addr     <= 64'd0;
            ar_cnt      <= 8'd0;
            ar_len      <= 8'd0;
        end else begin
            case (ar_state)
                AR_IDLE: begin
                    axi_arready <= 1'b1;
                    axi_rvalid  <= 1'b0;
                    if (axi_arvalid && axi_arready) begin
                        axi_arready <= 1'b0;
                        ar_addr <= axi_araddr;
                        ar_len  <= axi_arlen;
                        ar_cnt  <= 8'd0;
                        axi_rid <= axi_arid;
                        ar_state <= AR_BURST;
                    end
                end

                AR_BURST: begin
                    // Read 512-bit = 32 × 16-bit from consecutive 16-bit addresses
                    // ar_addr is a 16-bit word address
                    axi_rvalid <= 1'b1;
                    axi_rlast  <= (ar_cnt == ar_len);
                    axi_rresp  <= 2'b00;
                    // Compute rdata: 32 consecutive words
                    // Simple sequential 16-bit word read
`ifdef COCOTB_SIM
                    $display("[DDR] BURST beat=%0d addr=%0d word0=%h", ar_cnt, ar_addr+ar_cnt, mem[ar_addr + ar_cnt]);
`endif
                    axi_rdata[ 15:  0] <= mem[ar_addr[21:0] + {14'd0, ar_cnt} +  0];
                    axi_rdata[ 31: 16] <= mem[ar_addr[21:0] + {14'd0, ar_cnt} +  1];
                    axi_rdata[ 47: 32] <= mem[ar_addr + ar_cnt[21:0] + 21'd2];
                    axi_rdata[ 63: 48] <= mem[ar_addr + ar_cnt[21:0] + 21'd3];
                    axi_rdata[ 79: 64] <= mem[ar_addr + ar_cnt[21:0] + 21'd4];
                    axi_rdata[ 95: 80] <= mem[ar_addr + ar_cnt[21:0] + 21'd5];
                    axi_rdata[111: 96] <= mem[ar_addr + ar_cnt[21:0] + 21'd6];
                    axi_rdata[127:112] <= mem[ar_addr + ar_cnt[21:0] + 21'd7];
                    axi_rdata[143:128] <= mem[ar_addr + ar_cnt[21:0] + 21'd8];
                    axi_rdata[159:144] <= mem[ar_addr + ar_cnt[21:0] + 21'd9];
                    axi_rdata[175:160] <= mem[ar_addr + ar_cnt[21:0] + 21'd10];
                    axi_rdata[191:176] <= mem[ar_addr + ar_cnt[21:0] + 21'd11];
                    axi_rdata[207:192] <= mem[ar_addr + ar_cnt[21:0] + 21'd12];
                    axi_rdata[223:208] <= mem[ar_addr + ar_cnt[21:0] + 21'd13];
                    axi_rdata[239:224] <= mem[ar_addr + ar_cnt[21:0] + 21'd14];
                    axi_rdata[255:240] <= mem[ar_addr + ar_cnt[21:0] + 21'd15];
                    axi_rdata[271:256] <= mem[ar_addr + ar_cnt[21:0] + 21'd16];
                    axi_rdata[287:272] <= mem[ar_addr + ar_cnt[21:0] + 21'd17];
                    axi_rdata[303:288] <= mem[ar_addr + ar_cnt[21:0] + 21'd18];
                    axi_rdata[319:304] <= mem[ar_addr + ar_cnt[21:0] + 21'd19];
                    axi_rdata[335:320] <= mem[ar_addr + ar_cnt[21:0] + 21'd20];
                    axi_rdata[351:336] <= mem[ar_addr + ar_cnt[21:0] + 21'd21];
                    axi_rdata[367:352] <= mem[ar_addr + ar_cnt[21:0] + 21'd22];
                    axi_rdata[383:368] <= mem[ar_addr + ar_cnt[21:0] + 21'd23];
                    axi_rdata[399:384] <= mem[ar_addr + ar_cnt[21:0] + 21'd24];
                    axi_rdata[415:400] <= mem[ar_addr + ar_cnt[21:0] + 21'd25];
                    axi_rdata[431:416] <= mem[ar_addr + ar_cnt[21:0] + 21'd26];
                    axi_rdata[447:432] <= mem[ar_addr + ar_cnt[21:0] + 21'd27];
                    axi_rdata[463:448] <= mem[ar_addr + ar_cnt[21:0] + 21'd28];
                    axi_rdata[479:464] <= mem[ar_addr + ar_cnt[21:0] + 21'd29];
                    axi_rdata[495:480] <= mem[ar_addr + ar_cnt[21:0] + 21'd30];
                    axi_rdata[511:496] <= mem[ar_addr + ar_cnt[21:0] + 21'd31];

                    // Handshake — don't override rvalid on last beat!
                    if (axi_rready) begin
                        if (ar_cnt == ar_len) begin
                            // Last beat accepted → go idle (rvalid deasserts next cycle)
                            ar_state <= AR_IDLE;
                        end else begin
                            ar_cnt <= ar_cnt + 8'd1;
                        end
                    end
                end

                default: ar_state <= AR_IDLE;
            endcase
        end
    end

    // Build 512-bit read data from 32 consecutive 16-bit words
    integer i;
    always @(posedge clk) begin
        if (ar_state == AR_BURST) begin
            for (i = 0; i < 32; i = i + 1) begin
                axi_rdata[i*16 +: 16] <= mem[ar_addr[MEM_AW:0] + ar_cnt * 32 + i];
            end
        end
    end

endmodule
