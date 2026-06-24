//=============================================================================
// File     : b_shared_buffer.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Global shared B buffer — 4 replicas with TDP dual-read.
//            Each replica serves 4 PE via fixed-priority mux on 2 BRAM ports.
//            B_row_desc stays inside PE (512×64b LUT-RAM, small).
//
//   Write: b_broadcast_loader writes to ALL replicas identically.
//   Read:  PE asserts pe_b_req[i]=1 with pe_b_group[i] as address.
//          Data returns 1 cycle later on pe_bc0..bv3[i] with pe_b_rdy[i]=1.
//          If pe_b_rdy[i]=0, PE must stall (port conflict, retry next cycle).
//=============================================================================

`include "defines.vh"

module b_shared_buffer (
    // Write ports (from b_broadcast_loader, broadcast to all replicas)
    input  wire                     b_col_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_col_waddr,
    input  wire [`DATA_WIDTH-1:0]   b_col_wdata,
    input  wire                     b_val_we,
    input  wire [`B_NNZ_ADDR_BITS-1:0] b_val_waddr,
    input  wire [`DATA_WIDTH-1:0]   b_val_wdata,

    // Read request (16 PE) — address valid this cycle
    input  wire [`N_PE-1:0]         pe_b_req,
    input  wire [`N_PE*(`B_NNZ_ADDR_BITS-3)-1:0] pe_b_group_flat,

    // Read response (16 PE) — data + valid, 1 cycle after request
    output wire [`N_PE-1:0]         pe_b_rdy,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bc0_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bc1_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bc2_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bc3_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bv0_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bv1_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bv2_flat,
    output wire [`N_PE*`DATA_WIDTH-1:0] pe_bv3_flat,

    input  wire                     aclk,
    input  wire                     aresetn
);

    localparam B_BANK_DEPTH   = `B_NNZ_SLOT / 4;           // 12500
    localparam B_BANK_ADDR_W  = `B_NNZ_ADDR_BITS - 2;      // 14
    localparam PE_PER_REPLICA = `N_PE / `B_REPLICAS;        // 4
    localparam B_GW = `B_NNZ_ADDR_BITS - 3;                 // B group addr width (13)

    // Write address decode
    wire [1:0] wr_bank = b_col_waddr[1:0];
    wire [B_BANK_ADDR_W-1:0] wr_addr = b_col_waddr[`B_NNZ_ADDR_BITS-1:2];

    // Unpack flat pe_b_group bus
    wire [B_GW-1:0] pe_b_group [0:`N_PE-1];
    genvar ug;
    generate
        for (ug = 0; ug < `N_PE; ug = ug + 1) begin : gen_unpack
            assign pe_b_group[ug] = pe_b_group_flat[ug*B_GW +: B_GW];
        end
    endgenerate

    genvar rep, pi;
    generate
        for (rep = 0; rep < `B_REPLICAS; rep = rep + 1) begin : gen_replica

            //-----------------------------------------------------------------
            // 8 arrays per replica
            //-----------------------------------------------------------------
            reg [`DATA_WIDTH-1:0] col_b0_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] col_b1_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] col_b2_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] col_b3_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] val_b0_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] val_b1_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] val_b2_mem [0:B_BANK_DEPTH-1];
            reg [`DATA_WIDTH-1:0] val_b3_mem [0:B_BANK_DEPTH-1];

            // Read output registers (BRAM read latency = 1 cycle)
            reg [`DATA_WIDTH-1:0] col_b0_rA, col_b0_rB;
            reg [`DATA_WIDTH-1:0] col_b1_rA, col_b1_rB;
            reg [`DATA_WIDTH-1:0] col_b2_rA, col_b2_rB;
            reg [`DATA_WIDTH-1:0] col_b3_rA, col_b3_rB;
            reg [`DATA_WIDTH-1:0] val_b0_rA, val_b0_rB;
            reg [`DATA_WIDTH-1:0] val_b1_rA, val_b1_rB;
            reg [`DATA_WIDTH-1:0] val_b2_rA, val_b2_rB;
            reg [`DATA_WIDTH-1:0] val_b3_rA, val_b3_rB;

            //-----------------------------------------------------------------
            // Write (broadcast to all replicas)
            //-----------------------------------------------------------------
            always @(posedge aclk) begin
                if (b_col_we) case (wr_bank)
                    2'd0: col_b0_mem[wr_addr] <= b_col_wdata;
                    2'd1: col_b1_mem[wr_addr] <= b_col_wdata;
                    2'd2: col_b2_mem[wr_addr] <= b_col_wdata;
                    2'd3: col_b3_mem[wr_addr] <= b_col_wdata;
                endcase
                if (b_val_we) case (wr_bank)
                    2'd0: val_b0_mem[wr_addr] <= b_val_wdata;
                    2'd1: val_b1_mem[wr_addr] <= b_val_wdata;
                    2'd2: val_b2_mem[wr_addr] <= b_val_wdata;
                    2'd3: val_b3_mem[wr_addr] <= b_val_wdata;
                endcase
            end

            //-----------------------------------------------------------------
            // Priority encoder: 4 PE → 2 BRAM ports
            //   Port A: PE[0] > PE[2]
            //   Port B: PE[1] > PE[3]
            //-----------------------------------------------------------------
            localparam PE0 = rep * PE_PER_REPLICA + 0;
            localparam PE1 = rep * PE_PER_REPLICA + 1;
            localparam PE2 = rep * PE_PER_REPLICA + 2;
            localparam PE3 = rep * PE_PER_REPLICA + 3;

            wire pe0_win_A = pe_b_req[PE0];                     // PE0 always wins A if asking
            wire pe2_win_A = pe_b_req[PE2] && !pe_b_req[PE0];   // PE2 wins A only if PE0 idle

            wire pe1_win_B = pe_b_req[PE1];                     // PE1 always wins B if asking
            wire pe3_win_B = pe_b_req[PE3] && !pe_b_req[PE1];   // PE3 wins B only if PE1 idle

            wire port_A_used = pe0_win_A || pe2_win_A;
            wire port_B_used = pe1_win_B || pe3_win_B;

            // Port A address
            wire [B_BANK_ADDR_W-1:0] rd_addr_A = pe0_win_A ? pe_b_group[PE0] : pe_b_group[PE2];

            // Port B address
            wire [B_BANK_ADDR_W-1:0] rd_addr_B = pe1_win_B ? pe_b_group[PE1] : pe_b_group[PE3];

            //-----------------------------------------------------------------
            // TDP synchronous reads
            //-----------------------------------------------------------------
            always @(posedge aclk) begin
                if (port_A_used) begin
                    col_b0_rA <= col_b0_mem[rd_addr_A];
                    col_b1_rA <= col_b1_mem[rd_addr_A];
                    col_b2_rA <= col_b2_mem[rd_addr_A];
                    col_b3_rA <= col_b3_mem[rd_addr_A];
                    val_b0_rA <= val_b0_mem[rd_addr_A];
                    val_b1_rA <= val_b1_mem[rd_addr_A];
                    val_b2_rA <= val_b2_mem[rd_addr_A];
                    val_b3_rA <= val_b3_mem[rd_addr_A];
                end
                if (port_B_used) begin
                    col_b0_rB <= col_b0_mem[rd_addr_B];
                    col_b1_rB <= col_b1_mem[rd_addr_B];
                    col_b2_rB <= col_b2_mem[rd_addr_B];
                    col_b3_rB <= col_b3_mem[rd_addr_B];
                    val_b0_rB <= val_b0_mem[rd_addr_B];
                    val_b1_rB <= val_b1_mem[rd_addr_B];
                    val_b2_rB <= val_b2_mem[rd_addr_B];
                    val_b3_rB <= val_b3_mem[rd_addr_B];
                end
            end

            //-----------------------------------------------------------------
            // Pipeline: track which PE was served (1 cycle delay for BRAM)
            //-----------------------------------------------------------------
            reg pe0_served, pe1_served, pe2_served, pe3_served;
            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn) begin
                    pe0_served <= 1'b0;
                    pe1_served <= 1'b0;
                    pe2_served <= 1'b0;
                    pe3_served <= 1'b0;
                end else begin
                    pe0_served <= pe0_win_A;
                    pe1_served <= pe1_win_B;
                    pe2_served <= pe2_win_A;
                    pe3_served <= pe3_win_B;
                end
            end

            //-----------------------------------------------------------------
            // Output routing to each PE
            //-----------------------------------------------------------------
            // PE0: port A data, valid if served last cycle
            assign pe_b_rdy[PE0]     = pe0_served;
            assign pe_bc0_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = col_b0_rA;
            assign pe_bc1_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = col_b1_rA;
            assign pe_bc2_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = col_b2_rA;
            assign pe_bc3_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = col_b3_rA;
            assign pe_bv0_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = val_b0_rA;
            assign pe_bv1_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = val_b1_rA;
            assign pe_bv2_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = val_b2_rA;
            assign pe_bv3_flat[PE0*`DATA_WIDTH +: `DATA_WIDTH] = val_b3_rA;

            // PE1: port B data
            assign pe_b_rdy[PE1]     = pe1_served;
            assign pe_bc0_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = col_b0_rB;
            assign pe_bc1_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = col_b1_rB;
            assign pe_bc2_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = col_b2_rB;
            assign pe_bc3_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = col_b3_rB;
            assign pe_bv0_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = val_b0_rB;
            assign pe_bv1_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = val_b1_rB;
            assign pe_bv2_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = val_b2_rB;
            assign pe_bv3_flat[PE1*`DATA_WIDTH +: `DATA_WIDTH] = val_b3_rB;

            // PE2: port A data (only if PE0 idle)
            assign pe_b_rdy[PE2]     = pe2_served;
            assign pe_bc0_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = col_b0_rA;
            assign pe_bc1_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = col_b1_rA;
            assign pe_bc2_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = col_b2_rA;
            assign pe_bc3_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = col_b3_rA;
            assign pe_bv0_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = val_b0_rA;
            assign pe_bv1_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = val_b1_rA;
            assign pe_bv2_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = val_b2_rA;
            assign pe_bv3_flat[PE2*`DATA_WIDTH +: `DATA_WIDTH] = val_b3_rA;

            // PE3: port B data (only if PE1 idle)
            assign pe_b_rdy[PE3]     = pe3_served;
            assign pe_bc0_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = col_b0_rB;
            assign pe_bc1_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = col_b1_rB;
            assign pe_bc2_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = col_b2_rB;
            assign pe_bc3_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = col_b3_rB;
            assign pe_bv0_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = val_b0_rB;
            assign pe_bv1_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = val_b1_rB;
            assign pe_bv2_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = val_b2_rB;
            assign pe_bv3_flat[PE3*`DATA_WIDTH +: `DATA_WIDTH] = val_b3_rB;

        end  // gen_replica
    endgenerate

endmodule
