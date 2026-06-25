//=============================================================================
// File     : fp16_mul.v
// Brief    : FP16 × FP16 → FP16 combinatorial multiplier (IEEE 754-2008).
//
// FP16 format: [15] sign,  [14:10] exp (bias=15),  [9:0] mantissa (hidden 1)
//
// Normal × Normal:
//   mp = {1,ma} × {1,mb}  → 22-bit product
//   mp[21]=1 → leading 1 at bit 21;  result_exp = ea+eb-14  (biased)
//   mp[21]=0 → leading 1 at bit 20;  result_exp = ea+eb-15
//   FP16 mantissa = upper 10 bits after the leading 1; remaining bits used for RNE round.
//
// Overflow  (result_exp >= 31) → ±infinity
// Underflow (result_exp <=  0) → ±zero  (denormals flushed)
// NaN / ±Inf inputs: IEEE 754 rules (NaN propagation, inf×zero = NaN)
// Denormal inputs: flushed to zero.
// Rounding: round-to-nearest-even (RNE).
//=============================================================================
module fp16_mul (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] z
);
    wire       sa = a[15], sb = b[15];
    wire [4:0] ea = a[14:10], eb = b[14:10];
    wire [9:0] ma = a[9:0],   mb = b[9:0];
    wire       sz = sa ^ sb;

    //-------------------------------------------------------------------------
    // Special-case detection (denormals flushed to zero via *_zero)
    //-------------------------------------------------------------------------
    wire a_nan  = (&ea) & (|ma);
    wire b_nan  = (&eb) & (|mb);
    wire a_inf  = (&ea) & ~(|ma);
    wire b_inf  = (&eb) & ~(|mb);
    wire a_zero = ~(|ea);
    wire b_zero = ~(|eb);

    wire is_nan  = a_nan | b_nan | (a_inf & b_zero) | (a_zero & b_inf);
    wire is_inf  = ~is_nan & (a_inf | b_inf);
    wire is_zero = ~is_nan & (a_zero | b_zero);

    //-------------------------------------------------------------------------
    // Mantissa multiplication: 11-bit × 11-bit → 22-bit
    //-------------------------------------------------------------------------
    wire [10:0] ma_f = {1'b1, ma};
    wire [10:0] mb_f = {1'b1, mb};
    wire [21:0] mp   = ma_f * mb_f;

    wire norm_21 = mp[21];  // leading 1 at bit 21 or 20

    // Post-leading-1 mantissa bits and rounding inputs
    wire [9:0] mant   = norm_21 ? mp[20:11] : mp[19:10];
    wire       guard  = norm_21 ? mp[10]    : mp[9];
    wire       sticky = norm_21 ? (|mp[9:0]): (|mp[8:0]);

    // Round-to-nearest-even
    wire round_up = guard & (sticky | mant[0]);
    wire [10:0] mant_rnd   = {1'b0, mant} + {10'b0, round_up};
    wire        mant_carry = mant_rnd[10];  // rounding overflowed mantissa

    //-------------------------------------------------------------------------
    // Exponent: 7-bit two's-complement arithmetic to detect over/underflow
    //   norm_21=1 → base = ea+eb-14;  norm_21=0 → base = ea+eb-15
    //   +1 if mant_carry (rounding bumped exponent)
    //-------------------------------------------------------------------------
    wire [6:0] e_sum  = {2'b00, ea} + {2'b00, eb};
    wire [6:0] e_base = norm_21 ? (e_sum - 7'd14) : (e_sum - 7'd15);
    wire [6:0] e_adj  = e_base + {6'b0, mant_carry};

    // e_adj is unsigned 7-bit; two's-complement wrap detects negatives:
    //   bit[6]=1 → negative exponent → underflow (flush to zero)
    //   e_adj >= 31 and bit[6]=0 → overflow → infinity
    wire e_neg  = e_adj[6];
    wire e_ovfl = ~e_adj[6] & (e_adj >= 7'd31);

    wire [4:0] e_fp16 = e_adj[4:0];
    wire [9:0] m_fp16 = mant_carry ? mant_rnd[9:0] : mant;

    //-------------------------------------------------------------------------
    // Output mux
    //-------------------------------------------------------------------------
    assign z = is_nan              ? 16'h7E00                   : // quiet NaN
               is_inf              ? {sz, 5'h1F, 10'h000}       : // ±inf
               is_zero             ? {sz, 15'h0}                : // ±zero
               e_ovfl              ? {sz, 5'h1F, 10'h000}       : // overflow → ±inf
               (e_neg | ~|e_adj)   ? {sz, 15'h0}                : // underflow → ±zero
                                     {sz, e_fp16, m_fp16};        // normal
endmodule
