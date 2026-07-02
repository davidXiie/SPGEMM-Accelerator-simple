//=============================================================================
// File     : tb_core_top.v
// Project  : SPGEMM-Accelerator v2
// Brief    : Testbench for SpGEMM Accelerator v2
//            Phase 1: N_PE=1, N_MAC=1, small matrix test
//=============================================================================

`include "defines.vh"

module tb_core_top;

    reg aclk;
    reg aresetn;

    // CR interface
    reg  cr_start;
    reg  cr_clear;
    reg  [`MAX_DIM_BITS-1:0] M, K, N;
    reg  [7:0] pe_valid_mask;
    wire cr_finish;
    wire cr_busy;

    // AXI Read
    wire m_axi_arvalid;
    reg  m_axi_arready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]  m_axi_arlen;
    reg  m_axi_rvalid;
    wire m_axi_rready;
    reg  [`AXI_DATA_WIDTH-1:0] m_axi_rdata;
    reg  m_axi_rlast;

    // AXI Write
    wire m_axi_awvalid;
    reg  m_axi_awready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]  m_axi_awlen;
    wire m_axi_wvalid;
    reg  m_axi_wready;
    wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata;
    wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb;
    wire m_axi_wlast;
    reg  m_axi_bvalid;
    wire m_axi_bready;
    reg  [1:0] m_axi_bresp;

    wire [15:0] cycle_counter;

    // AXI model variables
    integer blen, beat, base_beat;

    //=========================================================================
    // DUT
    //=========================================================================
    core_top u_dut (
        .cr_start      (cr_start),
        .cr_clear      (cr_clear),
        .M             (M),
        .K             (K),
        .N             (N),
        .pe_valid_mask (pe_valid_mask),
        .cr_finish     (cr_finish),
        .cr_busy       (cr_busy),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_awvalid (m_axi_awvalid),
        .m_axi_awready (m_axi_awready),
        .m_axi_awaddr  (m_axi_awaddr),
        .m_axi_awlen   (m_axi_awlen),
        .m_axi_wvalid  (m_axi_wvalid),
        .m_axi_wready  (m_axi_wready),
        .m_axi_wdata   (m_axi_wdata),
        .m_axi_wstrb   (m_axi_wstrb),
        .m_axi_wlast   (m_axi_wlast),
        .m_axi_bvalid  (m_axi_bvalid),
        .m_axi_bready  (m_axi_bready),
        .m_axi_bresp   (m_axi_bresp),
        .cycle_counter (cycle_counter),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    //=========================================================================
    // Clock
    //=========================================================================
    always #5 aclk = ~aclk;  // 100MHz

    //=========================================================================
    // DDR Memory Model (simplified)
    //=========================================================================
    reg [`AXI_DATA_WIDTH-1:0] ddr_mem [0:8191];  // 512KB worth of 64-byte beats

    // Initialize memory with test data
    task init_memory;
        integer i, j;
        begin
            // Zero all
            for (i = 0; i < 8192; i = i + 1) ddr_mem[i] = 512'd0;

            // === Descriptor (at DDR_DESC_BASE = 0x0040_0000 → beat 0x8000) ===
            // M=2, K=3, N=4, pe_valid_mask=0x01
            ddr_mem[16'h8000 >> 6] = {
                448'd0,           // padding
                8'h01,            // pe_valid_mask at byte offset 8 (bit 64)
                16'd4,            // N at byte offset 4 (bit 32)
                16'd3,            // K at byte offset 2 (bit 16)
                16'd2             // M at byte offset 0 (bit 0)
            };
            // Note: above bit positions need verification. For little-endian:
            // byte0 = M[7:0], byte1 = M[15:8], byte2 = K[7:0], etc.
            // Let's pack more carefully:
            // M[15:0] at bits 15:0, K[15:0] at bits 31:16, N[15:0] at bits 47:32,
            // pe_valid_mask[7:0] at bits 71:64
            ddr_mem[16'h8000 >> 6] = { 448'd0, 8'h01, 16'd4, 16'd3, 16'd2 };

            // === B matrix (at DDR_B_BASE = 0x0020_0000) ===
            // B = 3×4, with compact row-desc format
            // B_row_desc[0..2]: each = {row_nnz[15:0], start_offset[15:0]}
            // Row 0: col=0,1 → row_nnz=2, start_offset=0
            // Row 1: col=2   → row_nnz=1, start_offset=2
            // Row 2: col=0,3 → row_nnz=2, start_offset=3
            j = (`DDR_B_BASE + `B_ROW_DESC_OFFSET) >> 6;
            ddr_mem[j] = { 480'd0, 16'd0, 16'd2,  16'd2, 16'd1,  16'd3, 16'd2 };  // desc[0], desc[1], desc[2]
            // Wait, we need to be careful about byte ordering.
            // Each row_desc = 2 × 16-bit. K=3 → 6 × 16-bit = 96 bits total.
            // Row_desc[0] = {start=0, nnz=2}  → bits 31:16=start, bits 15:0=nnz
            // Row_desc[1] = {start=2, nnz=1}  → bits 63:48=start, bits 47:32=nnz
            // Row_desc[2] = {start=3, nnz=2}  → bits 95:80=start, bits 79:64=nnz
            ddr_mem[j] = { 416'd0, 16'd3, 16'd2,  16'd2, 16'd1,  16'd0, 16'd2 };

            // B_col + B_val at B_COL_OFFSET (0x1000) → col then val interleaved
            // B_col: [0, 1, 2, 0, 3] → B_val: [v0, v1, v2, v3, v4]
            // For test, assign simple values: B_val = col+1
            j = (`DDR_B_BASE + `B_COL_OFFSET) >> 6;
            ddr_mem[j] = { 352'd0,
                16'd5, 16'd3,   // B_val[4]=5, B_col[4]=3
                16'd4, 16'd0,   // B_val[3]=4, B_col[3]=0
                16'd3, 16'd2,   // B_val[2]=3, B_col[2]=2
                16'd2, 16'd1,   // B_val[1]=2, B_col[1]=1
                16'd1, 16'd0 }; // B_val[0]=1, B_col[0]=0

            // === A matrix (at DDR_A_GROUPS_BASE = 0x0000_0000) ===
            // A = 2×3 (M=2, K=3), PE0 A_group
            // Row 0: A[0,0]=10, A[0,1]=20 → nnz=2, start=0
            // Row 1: A[1,2]=30 → nnz=1, start=2
            // A_row_desc per row = {global_row_id[15:0], row_nnz[15:0], start_offset[31:0]}
            // We store as 4 × 16-bit: [global_row_id, row_nnz, start_lo, start_hi]
            j = (`DDR_A_GROUPS_BASE + `A_ROW_DESC_OFFSET) >> 6;
            // Row_desc[0]: global_id=0, nnz=2, start=0
            // Row_desc[1]: global_id=1, nnz=1, start=2
            ddr_mem[j] = { 384'd0,
                16'd0, 16'd2, 16'd1, 16'd0,     // desc[1]: start_hi=0, start_lo=2, nnz=1, global_id=1
                16'd0, 16'd0, 16'd2, 16'd0 };    // desc[0]: start_hi=0, start_lo=0, nnz=2, global_id=0

            // A_col: [0, 1, 2]    (at A_COL_OFFSET = 0x1000)
            j = (`DDR_A_GROUPS_BASE + `A_COL_OFFSET) >> 6;
            ddr_mem[j] = { 448'd0, 16'd2, 16'd1, 16'd0 };  // col[2]=2, col[1]=1, col[0]=0

            // A_val: [10, 20, 30] (at A_VAL_OFFSET = 0x9000)
            j = (`DDR_A_GROUPS_BASE + `A_VAL_OFFSET) >> 6;
            // FP16: 10=0x4900, 20=0x4D00, 30=0x4F80
            ddr_mem[j] = { 448'd0, 16'h4F80, 16'h4D00, 16'h4900 };

            $display("[TB] Memory initialized");
            $display("[TB]   A: 2×3, B: 3×4, expected C: 2×4");
            $display("[TB]   A[0,:]={0:10, 1:20}, A[1,:]={2:30}");
            $display("[TB]   B[0,:]={0:1, 1:2}, B[1,:]={2:3}, B[2,:]={0:4, 3:5}");
            // C = A×B:
            // C[0,0] = A[0,0]*B[0,0] + A[0,1]*B[1,0] = 10*1 + 20*0 = 10
            // C[0,1] = A[0,0]*B[0,1] + A[0,1]*B[1,1] = 10*2 + 20*0 = 20
            // C[0,2] = A[0,0]*B[0,2] + A[0,1]*B[1,2] = 10*0 + 20*3 = 60
            // C[0,3] = A[0,0]*B[0,3] + A[0,1]*B[1,3] = 10*0 + 20*0 = 0
            // C[1,0] = A[1,2]*B[2,0] = 30*4 = 120
            // C[1,1] = 0
            // C[1,2] = 0
            // C[1,3] = A[1,2]*B[2,3] = 30*5 = 150
        end
    endtask

    //=========================================================================
    // Test sequence
    //=========================================================================
    initial begin
        aclk   = 1'b0;
        aresetn = 1'b0;
        cr_start = 1'b0;
        cr_clear = 1'b0;
        M = 0; K = 0; N = 0;
        pe_valid_mask = 8'h00;
        m_axi_arready = 1'b0;
        m_axi_rvalid  = 1'b0;
        m_axi_rdata   = 512'h0;
        m_axi_rlast   = 1'b0;
        m_axi_awready = 1'b0;
        m_axi_wready  = 1'b0;
        m_axi_bvalid  = 1'b0;
        m_axi_bresp   = 2'b00;

        // FST waveform dump
        $dumpfile("dump.fst");
        $dumpvars(0, tb_core_top);

        // Initialize memory
        init_memory();

        // Reset
        #100;
        aresetn = 1'b1;
        #50;

        // Configure and launch
        @(posedge aclk);
        M = 8'd2;
        K = 8'd3;
        N = 8'd4;
        pe_valid_mask = 8'h01;
        #10;
        cr_start = 1'b1;
        #10;
        cr_start = 1'b0;

        $display("[TB] Launched: M=%d, K=%d, N=%d", M, K, N);

        // Wait for finish with AXI model
        fork
            begin
                // AXI read responder
                forever begin
                    @(posedge aclk);
                    if (m_axi_arvalid && !m_axi_arready) begin
                        m_axi_arready <= 1'b1;
                        @(posedge aclk);
                        m_axi_arready <= 1'b0;
                        // Serve data beats
                        blen = m_axi_arlen + 1;
                        beat = 0;
                        base_beat = m_axi_araddr >> 6;
                        while (beat < blen) begin
                            m_axi_rvalid <= 1'b1;
                            m_axi_rdata  <= ddr_mem[base_beat + beat];
                            m_axi_rlast  <= (beat == blen - 1);
                            @(posedge aclk);
                            if (m_axi_rready) beat = beat + 1;
                        end
                        m_axi_rvalid <= 1'b0;
                        m_axi_rlast  <= 1'b0;
                    end
                end
            end
            begin
                // AXI write responder
                forever begin
                    @(posedge aclk);
                    if (m_axi_awvalid && !m_axi_awready) begin
                        m_axi_awready <= 1'b1;
                        @(posedge aclk);
                        m_axi_awready <= 1'b0;
                    end
                    if (m_axi_wvalid && !m_axi_wready) begin
                        m_axi_wready  <= 1'b1;
                    end else begin
                        m_axi_wready  <= 1'b0;
                    end
                    if (m_axi_wvalid && m_axi_wready && m_axi_wlast) begin
                        m_axi_bvalid <= 1'b1;
                        @(posedge aclk);
                        if (m_axi_bready) m_axi_bvalid <= 1'b0;
                    end
                end
            end
            begin
                // Wait for finish
                wait(cr_finish);
                $display("[TB] DUT finished at cycle %d", cycle_counter);
                #100;
                $display("[TB] Test PASSED (functional check TBD)");
                $finish;
            end
            begin
                // Timeout
                #1000000;
                $display("[TB] TIMEOUT at cycle %d", cycle_counter);
                $finish;
            end
        join
    end

endmodule
