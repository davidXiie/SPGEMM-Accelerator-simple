//=============================================================================
// File     : fp32_add.v
// Brief    : FP32 + FP32 → FP32 combinatorial adder.
//
// Handles: zero, infinity, NaN, normal operands.
// Denormal inputs: flushed to zero (ea==0 treated as zero).
// Denormal output: flushed to zero.
// Rounding: round-to-zero (truncation) — sufficient for SpGEMM accumulation.
//=============================================================================
module fp32_add (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [31:0] z
);
    wire        sa = a[31], sb = b[31];
    wire [7:0]  ea = a[30:23], eb = b[30:23];
    wire [22:0] ma = a[22:0],  mb = b[22:0];

    // Special cases
    wire a_nan = (&ea) & (|ma);
    wire b_nan = (&eb) & (|mb);
    wire a_inf = (&ea) & ~(|ma);
    wire b_inf = (&eb) & ~(|mb);
    wire a_zer = ~(|ea) & ~(|ma);   // ±0.0 or denormal → treat as zero
    wire b_zer = ~(|eb) & ~(|mb);

    wire is_nan = a_nan | b_nan | (a_inf & b_inf & (sa ^ sb));  // +inf + -inf = NaN
    wire is_inf = ~is_nan & (a_inf | b_inf);
    wire sign_inf = a_inf ? sa : sb;

    // For zero-input short-circuits (handled in final mux; intermediate paths unused)
    wire a_only_zer = a_zer & ~b_zer;
    wire b_only_zer = b_zer & ~a_zer;
    wire both_zer   = a_zer & b_zer;

    //-------------------------------------------------------------------------
    // Main path: both operands normal (ea,eb ∈ [1,254])
    // Full 24-bit mantissas with implicit leading 1 (safe: zero short-circuited above)
    wire [23:0] ma_f = {1'b1, ma};
    wire [23:0] mb_f = {1'b1, mb};

    // Swap so the operand with the larger exponent is "big"
    wire a_ge = (ea > eb) | ((ea == eb) & (ma >= mb));
    wire [7:0]  e_big  = a_ge ? ea    : eb;
    wire [23:0] mf_big = a_ge ? ma_f  : mb_f;
    wire [23:0] mf_sml = a_ge ? mb_f  : ma_f;
    wire        s_big  = a_ge ? sa    : sb;
    wire        s_sml  = a_ge ? sb    : sa;

    // Alignment: right-shift the smaller operand
    wire [7:0]  shamt     = e_big - (a_ge ? eb : ea);
    wire [23:0] mf_aligned = (shamt >= 8'd24) ? 24'h0 : (mf_sml >> shamt);

    // Effective operation
    wire do_sub = s_big ^ s_sml;

    // 25-bit result (carry or borrow possible)
    wire [24:0] m_sum = do_sub
        ? ({1'b0, mf_big} - {1'b0, mf_aligned})
        : ({1'b0, mf_big} + {1'b0, mf_aligned});

    //-------------------------------------------------------------------------
    // Normalise
    // Addition: carry-out at bit 24 → shift right 1, exp+1
    wire add_carry = m_sum[24] & ~do_sub;

    // Subtraction: find leading 1 in m_sum[23:0] using LZC
    reg [4:0] lzc;
    always @(*) begin
        casez (m_sum[23:0])
            24'b1???????????????????????: lzc = 5'd0;
            24'b01??????????????????????: lzc = 5'd1;
            24'b001?????????????????????: lzc = 5'd2;
            24'b0001????????????????????: lzc = 5'd3;
            24'b00001???????????????????: lzc = 5'd4;
            24'b000001??????????????????: lzc = 5'd5;
            24'b0000001?????????????????: lzc = 5'd6;
            24'b00000001????????????????: lzc = 5'd7;
            24'b000000001???????????????: lzc = 5'd8;
            24'b0000000001??????????????: lzc = 5'd9;
            24'b00000000001?????????????: lzc = 5'd10;
            24'b000000000001????????????: lzc = 5'd11;
            24'b0000000000001???????????: lzc = 5'd12;
            24'b00000000000001??????????: lzc = 5'd13;
            24'b000000000000001?????????: lzc = 5'd14;
            24'b0000000000000001????????: lzc = 5'd15;
            24'b00000000000000001???????: lzc = 5'd16;
            24'b000000000000000001??????: lzc = 5'd17;
            24'b0000000000000000001?????: lzc = 5'd18;
            24'b00000000000000000001????: lzc = 5'd19;
            24'b000000000000000000001???: lzc = 5'd20;
            24'b0000000000000000000001??: lzc = 5'd21;
            24'b00000000000000000000001?: lzc = 5'd22;
            24'b000000000000000000000001: lzc = 5'd23;
            default:                      lzc = 5'd24; // exact cancellation → 0
        endcase
    end

    // Addition result (may overflow to infinity)
    wire [7:0]  e_add    = add_carry ? (e_big + 8'd1) : e_big;
    wire [22:0] m_add    = add_carry ? m_sum[23:1] : m_sum[22:0];
    wire        add_ovf  = add_carry & (&e_big);   // e_big == 0xFE and carry → inf

    // Subtraction result (may underflow or cancel)
    wire [8:0]  e_sub9   = {1'b0, e_big} - {4'b0, lzc};
    wire        sub_unf  = e_sub9[8] | (lzc == 5'd24);  // negative exp or exact cancel
    wire [7:0]  e_sub    = sub_unf ? 8'h0 : e_sub9[7:0];
    wire [23:0] m_shifted = m_sum[23:0] << lzc;
    wire [22:0] m_sub    = sub_unf ? 23'h0 : m_shifted[22:0];

    // Mux add vs sub
    wire [7:0]  e_res = do_sub ? e_sub : e_add;
    wire [22:0] m_res = do_sub ? m_sub : m_add;

    wire [31:0] z_norm = add_ovf           ? {s_big, 8'hFF, 23'h0}   // overflow → inf
                       : (do_sub & sub_unf) ? {s_big, 31'h0}            // subtraction underflow/cancel → 0
                       :                      {s_big, e_res, m_res};

    //-------------------------------------------------------------------------
    // Final output mux (special cases take priority)
    assign z = is_nan      ? {1'b0, 8'hFF, 23'h400000} :
               is_inf      ? {sign_inf, 8'hFF, 23'h0}   :
               a_only_zer  ? b                           :
               b_only_zer  ? a                           :
               both_zer    ? {s_big, 31'h0}              :
                              z_norm;

endmodule
