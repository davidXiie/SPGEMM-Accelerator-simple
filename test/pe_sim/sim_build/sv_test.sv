module sv_test;
    logic clk = 0;
    always #5 clk = ~clk;

    int cnt = 0;
    always_ff @(posedge clk) begin
        cnt <= cnt + 1;
        $display("SV OK at %0t, cnt=%0d", $time, cnt);
        if (cnt == 3) $finish;
    end
endmodule
