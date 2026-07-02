//=============================================================================
// File     : tb_row_accumulator_4bank.v
// Brief    : Testbench for row_accumulator_4bank.
//
//   Test cases:
//     1. Four distinct banks hit simultaneously (col_id = {0,1,2,3})
//     2. All lanes to same bank (col_id = {1,5,9,13})
//     3. Cross-cycle repeated col_id (same col_id in consecutive bundles)
//     4. Partial lane valid (only lanes 0 and 3)
//     5. FIFO backpressure (rapidly fill one bank's FIFO)
//     6. Empty output row (no bundles)
//     7. Zero-value accumulation (positive + negative → 0, DROP_ZERO)
//     8. Random multi-row test with software reference model
//=============================================================================

`timescale 1ns/1ps
`define SIMULATION

// DUT parameters (match defaults in RTL)
`define OUT_COLS  512
`define COL_W     9
`define PROD_W    16
`define ACC_W     32
`define EPOCH_W   16
`define ROW_W     16
`define BANK_FIFO_DEPTH 8
`define BANK_FIFO_LOG   3

module tb_row_accumulator_4bank;

    // =========================================================================
    // Clock & reset
    // =========================================================================
    reg clk, rst_n;
    initial clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    // =========================================================================
    // DUT ports
    // =========================================================================
    reg                         row_start;
    reg [`ROW_W-1:0]            row_id_in;
    reg                         row_input_done;
    wire                        busy;
    wire                        row_done;

    reg                         issue_valid;
    wire                        issue_ready;
    reg  [3:0]                  lane_valid;
    reg  [4*`COL_W-1:0]         lane_col_id;
    reg  [4*`PROD_W-1:0]        lane_product;

    wire                        out_valid;
    reg                         out_ready;
    wire [`ROW_W-1:0]           out_row_id;
    wire [`COL_W-1:0]           out_col_id;
    wire [`ACC_W-1:0]           out_value;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    row_accumulator_4bank #(
        .OUT_COLS       (`OUT_COLS),
        .COL_W          (`COL_W),
        .PROD_W         (`PROD_W),
        .ACC_W          (`ACC_W),
        .EPOCH_W        (`EPOCH_W),
        .BANK_FIFO_DEPTH(`BANK_FIFO_DEPTH),
        .BANK_FIFO_LOG  (`BANK_FIFO_LOG),
        .ROW_W          (`ROW_W)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .row_start      (row_start),
        .row_id_in      (row_id_in),
        .row_input_done (row_input_done),
        .busy           (busy),
        .row_done       (row_done),
        .issue_valid    (issue_valid),
        .issue_ready    (issue_ready),
        .lane_valid     (lane_valid),
        .lane_col_id    (lane_col_id),
        .lane_product   (lane_product),
        .out_valid      (out_valid),
        .out_ready      (out_ready),
        .out_row_id     (out_row_id),
        .out_col_id     (out_col_id),
        .out_value      (out_value)
    );

    // =========================================================================
    // Software reference model
    // =========================================================================
    integer ref_acc   [0:`OUT_COLS-1];
    integer ref_nonzero;    // number of nonzero ref entries
    integer cur_test_row_id;

    task ref_clear;
        integer i;
        begin
            for (i = 0; i < `OUT_COLS; i = i + 1)
                ref_acc[i] = 0;
            ref_nonzero = 0;
        end
    endtask

    // Add one lane's contribution to the reference model (signed product)
    task ref_add;
        input integer col;
        input integer prod;  // signed 16-bit
        integer sp;
        begin
            // Sign-extend PROD_W-bit value
            sp = (prod[`PROD_W-1]) ? (prod | ~(({1'b0, {(`PROD_W){1'b1}}})>>1)*2+1-1) : prod;
            // Simpler: treat as signed integer directly (Verilog integer is 32-bit signed)
            // prod already comes in as a 16-bit reg; cast to signed
            ref_acc[col] = ref_acc[col] + $signed(prod[`PROD_W-1:0]);
        end
    endtask

    // =========================================================================
    // Helper tasks
    // =========================================================================

    // Start a new row
    task start_row;
        input [`ROW_W-1:0] rid;
        begin
            @(negedge clk);
            row_id_in   = rid;
            row_start   = 1'b1;
            @(posedge clk); #1;
            row_start   = 1'b0;
            cur_test_row_id = rid;
            ref_clear;
        end
    endtask

    // Issue one 4-lane bundle; waits for issue_ready with timeout
    task issue_bundle;
        input [3:0] vld;
        input [`COL_W-1:0] c0, c1, c2, c3;
        input signed [`PROD_W-1:0] p0, p1, p2, p3;
        integer timeout;
        begin
            @(negedge clk);
            issue_valid  = 1'b1;
            lane_valid   = vld;
            lane_col_id  = {c3, c2, c1, c0};
            lane_product = {p3, p2, p1, p0};

            // Wait for handshake
            timeout = 0;
            while (!issue_ready && timeout < 500) begin
                @(posedge clk); #1;
                timeout = timeout + 1;
            end
            if (timeout == 500) begin
                $error("TIMEOUT waiting for issue_ready in issue_bundle");
                $finish;
            end

            // Accepted this cycle (at negedge+#1 view, posedge is about to happen)
            // Update reference for accepted lanes
            if (vld[0]) ref_add(c0, p0);
            if (vld[1]) ref_add(c1, p1);
            if (vld[2]) ref_add(c2, p2);
            if (vld[3]) ref_add(c3, p3);

            @(posedge clk); #1;
            issue_valid = 1'b0;
        end
    endtask

    // Signal end of row input
    task done_input;
        begin
            @(negedge clk);
            row_input_done = 1'b1;
            @(posedge clk); #1;
            row_input_done = 1'b0;
        end
    endtask

    // Collect and verify DUT output for current row
    // Expects out_ready always high (can be changed to test backpressure separately)
    task verify_row;
        input [`ROW_W-1:0] expected_row_id;
        input integer       max_cycles;
        integer timeout;
        integer emit_count;
        integer last_col;
        integer col, val;
        begin
            timeout    = 0;
            emit_count = 0;
            last_col   = -1;
            out_ready  = 1'b1;

            // Count reference nonzero entries
            ref_nonzero = 0;
            begin : cnt_blk
                integer ii;
                for (ii = 0; ii < `OUT_COLS; ii = ii + 1)
                    if (ref_acc[ii] != 0) ref_nonzero = ref_nonzero + 1;
            end

            // Wait for row_done
            while (!row_done && timeout < max_cycles) begin
                @(posedge clk); #1;
                if (out_valid && out_ready) begin
                    col = out_col_id;
                    val = $signed(out_value);

                    // Check row_id
                    if (out_row_id !== expected_row_id)
                        $error("verify_row: wrong row_id %0d (expected %0d)", out_row_id, expected_row_id);

                    // Check strictly increasing col_id
                    if (col <= last_col)
                        $error("verify_row: col_id %0d not > previous %0d", col, last_col);
                    last_col = col;

                    // Check value matches reference
                    if ($signed(ref_acc[col]) !== val) begin
                        $error("verify_row: col %0d value mismatch: DUT=%0d REF=%0d",
                               col, val, $signed(ref_acc[col]));
                    end else begin
                        // Mark as checked
                        ref_acc[col] = 0;
                        emit_count   = emit_count + 1;
                    end
                end
                timeout = timeout + 1;
            end

            if (timeout == max_cycles) begin
                $error("verify_row: TIMEOUT waiting for row_done");
                $finish;
            end

            // Check emit count matches reference nonzero count
            if (emit_count !== ref_nonzero)
                $error("verify_row: emitted %0d entries, expected %0d", emit_count, ref_nonzero);

            // Check all reference entries were emitted (all should be 0 now)
            begin : chk_blk
                integer ii;
                for (ii = 0; ii < `OUT_COLS; ii = ii + 1) begin
                    if (ref_acc[ii] != 0)
                        $error("verify_row: col %0d in ref (val=%0d) not emitted by DUT",
                               ii, $signed(ref_acc[ii]));
                end
            end

            out_ready = 1'b0;
        end
    endtask

    // =========================================================================
    // Pass/fail counter
    // =========================================================================
    integer pass_cnt, fail_cnt;
    initial begin pass_cnt = 0; fail_cnt = 0; end

    // Hook $error to count failures (simple approach)
    // In practice use SVA, but for Verilog-2001 just rely on $error messages

    // =========================================================================
    // Test body
    // =========================================================================
    integer i, j;
    reg [`COL_W-1:0] rand_col [0:3];
    reg signed [`PROD_W-1:0] rand_prod [0:3];
    reg [3:0] rand_valid;
    integer used_cols [0:3];   // track col_ids used this bundle (uniqueness)

    initial begin
        // Initialize
        rst_n          = 0;
        row_start      = 0;
        row_id_in      = 0;
        row_input_done = 0;
        issue_valid    = 0;
        lane_valid     = 0;
        lane_col_id    = 0;
        lane_product   = 0;
        out_ready      = 0;
        ref_clear;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        // =================================================================
        // Test 1: Four distinct banks hit simultaneously
        //   col_id = {0,1,2,3} → bank 0,1,2,3 each get 1 entry
        // =================================================================
        $display("=== Test 1: Four distinct banks ===");
        start_row(16'd1);
        issue_bundle(4'b1111,
                     `COL_W'd0, `COL_W'd1, `COL_W'd2, `COL_W'd3,
                     16'sd1, 16'sd2, 16'sd3, 16'sd4);
        done_input;
        verify_row(16'd1, 5000);
        $display("Test 1 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 2: All lanes to same bank (bank conflict)
        //   col_id = {1,5,9,13} → all bank 1, different bank_addrs
        // =================================================================
        $display("=== Test 2: Same-bank conflict ===");
        start_row(16'd2);
        issue_bundle(4'b1111,
                     `COL_W'd1, `COL_W'd5, `COL_W'd9, `COL_W'd13,
                     16'sd10, 16'sd20, 16'sd30, 16'sd40);
        done_input;
        verify_row(16'd2, 5000);
        $display("Test 2 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 3: Cross-cycle repeated col_id
        //   Bundle 0: col 7 = p1
        //   Bundle 1: col 7 = p2   → acc[7] = p1+p2
        //   (distinct within each bundle since only lane 0 valid)
        // =================================================================
        $display("=== Test 3: Cross-cycle repeated col_id ===");
        start_row(16'd3);
        issue_bundle(4'b0001,
                     `COL_W'd7, `COL_W'd0, `COL_W'd0, `COL_W'd0,
                     16'sd5, 16'sd0, 16'sd0, 16'sd0);
        issue_bundle(4'b0001,
                     `COL_W'd7, `COL_W'd0, `COL_W'd0, `COL_W'd0,
                     16'sd3, 16'sd0, 16'sd0, 16'sd0);
        issue_bundle(4'b0001,
                     `COL_W'd7, `COL_W'd0, `COL_W'd0, `COL_W'd0,
                     -16'sd2, 16'sd0, 16'sd0, 16'sd0);
        // acc[7] should be 5+3-2=6
        done_input;
        verify_row(16'd3, 5000);
        $display("Test 3 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 4: Partial lane valid (only lanes 0 and 3)
        // =================================================================
        $display("=== Test 4: Partial lane valid ===");
        start_row(16'd4);
        issue_bundle(4'b1001,
                     `COL_W'd10, `COL_W'd0, `COL_W'd0, `COL_W'd15,
                     16'sd7, 16'sd0, 16'sd0, 16'sd9);
        done_input;
        verify_row(16'd4, 5000);
        $display("Test 4 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 5: FIFO backpressure — fill bank 0 repeatedly
        //   Send BANK_FIFO_DEPTH+2 bundles all to bank 0
        //   (col_ids: 0,4,8,12,...  all bank 0, distinct col_ids)
        // =================================================================
        $display("=== Test 5: FIFO backpressure ===");
        start_row(16'd5);
        for (i = 0; i < `BANK_FIFO_DEPTH + 4; i = i + 1) begin
            // One lane only, bank 0, increasing col_id
            issue_bundle(4'b0001,
                         (`COL_W'd0 + i*`COL_W'd4), `COL_W'd0, `COL_W'd0, `COL_W'd0,
                         16'sd1, 16'sd0, 16'sd0, 16'sd0);
        end
        done_input;
        verify_row(16'd5, 10000);
        $display("Test 5 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 6: Empty output row (no bundles issued)
        // =================================================================
        $display("=== Test 6: Empty row ===");
        start_row(16'd6);
        // No bundles
        done_input;
        verify_row(16'd6, 5000);  // should see row_done with no out_valid
        $display("Test 6 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 7: Zero-value accumulation (+ then − cancels to 0)
        //   col 20: +10 then -10 = 0 → should NOT appear in output (DROP_ZERO)
        //   col 21: +7 (should appear)
        // =================================================================
        $display("=== Test 7: Zero-value drop ===");
        start_row(16'd7);
        issue_bundle(4'b0011,
                     `COL_W'd20, `COL_W'd21, `COL_W'd0, `COL_W'd0,
                     16'sd10, 16'sd7, 16'sd0, 16'sd0);
        issue_bundle(4'b0001,
                     `COL_W'd20, `COL_W'd0, `COL_W'd0, `COL_W'd0,
                     -16'sd10, 16'sd0, 16'sd0, 16'sd0);
        // ref_acc[20] = 0 (cancelled), ref_acc[21] = 7
        done_input;
        verify_row(16'd7, 5000);
        $display("Test 7 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 8: Multi-bundle row with out_ready backpressure
        // =================================================================
        $display("=== Test 8: Output backpressure ===");
        out_ready = 1'b0;   // hold ready low
        start_row(16'd8);
        issue_bundle(4'b1111,
                     `COL_W'd100, `COL_W'd101, `COL_W'd102, `COL_W'd103,
                     16'sd1, 16'sd2, 16'sd3, 16'sd4);
        issue_bundle(4'b1111,
                     `COL_W'd200, `COL_W'd201, `COL_W'd202, `COL_W'd203,
                     16'sd5, 16'sd6, 16'sd7, 16'sd8);
        done_input;

        // Wait for row_done while toggling out_ready
        begin : bp_test
            integer bp_timeout;
            integer bp_last_col;
            integer bp_emit;
            bp_timeout  = 0;
            bp_last_col = -1;
            bp_emit     = 0;

            // Count reference nonzero
            ref_nonzero = 0;
            for (i = 0; i < `OUT_COLS; i = i + 1)
                if (ref_acc[i] != 0) ref_nonzero = ref_nonzero + 1;

            while (!row_done && bp_timeout < 20000) begin
                @(posedge clk); #1;
                // Toggle out_ready every 3 cycles to test backpressure
                if ((bp_timeout % 3) == 0)
                    out_ready = ~out_ready;

                if (out_valid && out_ready) begin
                    begin : bp_check
                        // Assign wire to integer first to force signed 32-bit comparison
                        integer bp_cur_col;
                        bp_cur_col = out_col_id;
                        if (bp_last_col >= 0 && bp_cur_col <= bp_last_col)
                            $error("Test 8: out_col_id not increasing: %0d <= %0d",
                                   bp_cur_col, bp_last_col);
                        if ($signed(ref_acc[bp_cur_col]) !== $signed(out_value))
                            $error("Test 8: col %0d value mismatch DUT=%0d REF=%0d",
                                   bp_cur_col, $signed(out_value), $signed(ref_acc[bp_cur_col]));
                        bp_last_col = bp_cur_col;
                        ref_acc[bp_cur_col] = 0;
                    end
                    bp_emit = bp_emit + 1;
                end
                bp_timeout = bp_timeout + 1;
            end

            if (bp_timeout == 20000) $error("Test 8: TIMEOUT");
            if (bp_emit !== ref_nonzero) $error("Test 8: emit count %0d != ref %0d", bp_emit, ref_nonzero);
        end
        out_ready = 1'b0;
        $display("Test 8 complete");
        repeat (5) @(posedge clk);

        // =================================================================
        // Test 9: Random multi-row test
        // =================================================================
        $display("=== Test 9: Random multi-row ===");
        begin : rand_test
            integer num_rows, row_i, bundle_i, num_bundles;
            integer la, lb, lc, ld;    // lane indices for col_id selection
            reg [`COL_W-1:0] c0r, c1r, c2r, c3r;
            reg signed [`PROD_W-1:0] p0r, p1r, p2r, p3r;
            reg [3:0] vr;

            num_rows = 10;
            for (row_i = 0; row_i < num_rows; row_i = row_i + 1) begin
                start_row(row_i + 16'd100);
                num_bundles = ($random % 16) + 1;  // 1..16 bundles per row

                for (bundle_i = 0; bundle_i < num_bundles; bundle_i = bundle_i + 1) begin
                    // Generate 4 distinct random col_ids (to satisfy uniqueness constraint)
                    // Use multiples of 4 + lane index to guarantee uniqueness across banks
                    c0r = (($random & 16'h007F) << 2) | 2'd0;  // bank 0
                    c1r = (($random & 16'h007F) << 2) | 2'd1;  // bank 1
                    c2r = (($random & 16'h007F) << 2) | 2'd2;  // bank 2
                    c3r = (($random & 16'h007F) << 2) | 2'd3;  // bank 3
                    // Clamp to OUT_COLS-1
                    if (c0r >= `OUT_COLS) c0r = c0r - 4;
                    if (c1r >= `OUT_COLS) c1r = c1r - 4;
                    if (c2r >= `OUT_COLS) c2r = c2r - 4;
                    if (c3r >= `OUT_COLS) c3r = c3r - 4;

                    p0r = $random & 16'hFF;
                    p1r = $random & 16'hFF;
                    p2r = $random & 16'hFF;
                    p3r = $random & 16'hFF;
                    vr  = ($random & 4'hF) | 4'b0001; // at least lane 0 valid

                    issue_bundle(vr, c0r, c1r, c2r, c3r, p0r, p1r, p2r, p3r);
                end

                done_input;
                out_ready = 1'b1;
                verify_row(row_i + 16'd100, 50000);
                out_ready = 1'b0;
                repeat (3) @(posedge clk);
            end
        end
        $display("Test 9 complete");

        // =================================================================
        $display("=== All tests done ===");
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #2000000;
        $error("Global simulation timeout");
        $finish;
    end

    // =========================================================================
    // Waveform dump
    // =========================================================================
    initial begin
        $dumpfile("sim_row_accum.vcd");
        $dumpvars(0, tb_row_accumulator_4bank);
    end

endmodule
