//=============================================================================
// File     : fp16_add.v
// Brief    : FP16 + FP16 → FP16 combinatorial adder (IEEE 754-2008).
//
// FP16 format: [15] sign,  [14:10] exp (bias=15),  [9:0] mantissa (hidden 1)
//
// Handles: ±zero, ±infinity, NaN, normal operands.
// Denormal inputs/outputs: flushed to zero.
// Rounding: round-toward-zero (truncation) — sufficient for SpGEMM accumulation.
// Overflow: result_exp == 31 or carry into exp-31 → ±infinity.
//=============================================================================
module fp16_add (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] z
);
    wire       sa = a[15], sb = b[15];
    wire [4:0] ea = a[14:10], eb = b[14:10];
    wire [9:0] ma = a[9:0],   mb = b[9:0];

    //-------------------------------------------------------------------------
    // Special-case detection
    //-------------------------------------------------------------------------
    wire a_nan = (&ea) & (|ma);
    wire b_nan = (&eb) & (|mb);
    wire a_inf = (&ea) & ~(|ma);
    wire b_inf = (&eb) & ~(|mb);
    wire a_zer = ~(|ea);     // zero or denormal → flush to zero
    wire b_zer = ~(|eb);

    wire is_nan    = a_nan | b_nan | (a_inf & b_inf & (sa ^ sb));
    wire is_inf    = ~is_nan & (a_inf | b_inf);
    wire sign_inf  = a_inf ? sa : sb;
    wire a_only_z  = a_zer & ~b_zer;
    wire b_only_z  = b_zer & ~a_zer;
    wire both_z    = a_zer & b_zer;

    //-------------------------------------------------------------------------
    // Main path: align and add/sub
    //-------------------------------------------------------------------------
    wire [10:0] ma_f = {1'b1, ma};
    wire [10:0] mb_f = {1'b1, mb};

    // Put larger-magnitude operand in "big" slot
    wire a_ge   = (ea > eb) | ((ea == eb) & (ma >= mb));
    wire [4:0]  e_big  = a_ge ? ea    : eb;
    wire [10:0] mf_big = a_ge ? ma_f  : mb_f;
    wire [10:0] mf_sml = a_ge ? mb_f  : ma_f;
    wire        s_big  = a_ge ? sa    : sb;
    wire        s_sml  = a_ge ? sb    : sa;

    // Align smaller operand by right-shifting
    // Zero out only when shift >= 11 (full 11-bit mantissa shifted away)
    wire [4:0]  shamt      = e_big - (a_ge ? eb : ea);
    wire [10:0] mf_aligned = (shamt >= 5'd11) ? 11'h0 : (mf_sml >> shamt);

    wire do_sub = s_big ^ s_sml;

    // 12-bit sum (addition can produce carry; subtraction is always smaller)
    wire [11:0] m_sum = do_sub
        ? ({1'b0, mf_big} - {1'b0, mf_aligned})
        : ({1'b0, mf_big} + {1'b0, mf_aligned});

    //-------------------------------------------------------------------------
    // Normalize
    //-------------------------------------------------------------------------
    // Addition: carry into bit 11 → shift right 1, increment exponent
    wire add_carry = m_sum[11] & ~do_sub;

    // Subtraction: leading-zero count of m_sum[10:0]
    reg [3:0] lzc;
    always @(*) begin
        casez (m_sum[10:0])
            11'b1??????????: lzc = 4'd0;
            11'b01?????????: lzc = 4'd1;
            11'b001????????: lzc = 4'd2;
            11'b0001???????: lzc = 4'd3;
            11'b00001??????: lzc = 4'd4;
            11'b000001?????: lzc = 4'd5;
            11'b0000001????: lzc = 4'd6;
            11'b00000001???: lzc = 4'd7;
            11'b000000001??: lzc = 4'd8;
            11'b0000000001?: lzc = 4'd9;
            11'b00000000001: lzc = 4'd10;
            default:         lzc = 4'd11; // exact cancellation → zero
        endcase
    end

    // Addition result (overflow when carry pushes exp to 31)
    wire [4:0] e_add   = add_carry ? (e_big + 5'd1) : e_big;
    wire [9:0] m_add   = add_carry ? m_sum[10:1]    : m_sum[9:0];
    wire       add_ovf = add_carry & (&e_big);  // e_big==30+carry → exp==31 → inf

    // Subtraction result (underflow when leading-zero shift goes below exp 1)
    wire [5:0] e_sub6   = {1'b0, e_big} - {2'b0, lzc};
    wire       sub_unf  = e_sub6[5] | (lzc == 4'd11);
    wire [4:0] e_sub    = sub_unf ? 5'h0 : e_sub6[4:0];
    wire [10:0] m_shift = m_sum[10:0] << lzc;
    wire [9:0]  m_sub   = sub_unf ? 10'h0 : m_shift[9:0];

    wire [4:0] e_res = do_sub ? e_sub : e_add;
    wire [9:0] m_res = do_sub ? m_sub : m_add;

    wire [15:0] z_norm = add_ovf           ? {s_big, 5'h1F, 10'h0} :
                         (do_sub & sub_unf) ? {s_big, 15'h0}        :
                                              {s_big, e_res, m_res};

    //-------------------------------------------------------------------------
    // Output mux (special cases have priority)
    //-------------------------------------------------------------------------
    assign z = is_nan    ? 16'h7E00                    :
               is_inf    ? {sign_inf, 5'h1F, 10'h0}   :
               a_only_z  ? b                           :
               b_only_z  ? a                           :
               both_z    ? {s_big, 15'h0}             :
                            z_norm;
endmodule
