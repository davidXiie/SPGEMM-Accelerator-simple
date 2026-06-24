//=============================================================================
// File     : fp16_mul.v
// Brief    : FP16 × FP16 → FP32 combinatorial multiplier.
//
// FP16 (IEEE 754-2008 binary16):
//   [15] sign,  [14:10] exp (bias=15),  [9:0] mantissa
// FP32 (IEEE 754 binary32):
//   [31] sign,  [30:23] exp (bias=127), [22:0] mantissa
//
// For normal × normal inputs (ea,eb ∈ [1,30]):
//   True exp of product = (ea-15)+(eb-15) = ea+eb-30
//   FP32 biased exp     = ea+eb-30+127 = ea+eb+97 (if leading bit at pos 20)
//                       = ea+eb+97+1 = ea+eb+98    (if leading bit at pos 21)
//
//   {1,ma_10b}×{1,mb_10b} = 22-bit product mp.
//   mp[21]=1 → product ≥ 2^21, leading bit at 21.
//   mp[21]=0 → product ≥ 2^20, leading bit at 20.
//
// Denormals: flushed to zero (ea==0 treated as zero).
// No rounding (exact for FP16→FP32 widening: at most 21 significant bits,
// well within FP32's 24-bit significand).
//=============================================================================
module fp16_mul (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [31:0] z
);
    wire        sa = a[15], sb = b[15];
    wire [4:0]  ea = a[14:10], eb = b[14:10];
    wire [9:0]  ma = a[9:0],   mb = b[9:0];

    wire s_z = sa ^ sb;

    // Special-case detection (flush denormals to zero via a_is_zero)
    wire a_is_nan  =  (&ea) & (|ma);
    wire b_is_nan  =  (&eb) & (|mb);
    wire a_is_inf  =  (&ea) & ~(|ma);
    wire b_is_inf  =  (&eb) & ~(|mb);
    wire a_is_zero = ~(|ea);            // ea==0: zero or denormal → flush to zero
    wire b_is_zero = ~(|eb);

    wire is_nan  = a_is_nan | b_is_nan | (a_is_inf & b_is_zero) | (a_is_zero & b_is_inf);
    wire is_inf  = ~is_nan & (a_is_inf | b_is_inf);
    wire is_zero = ~is_nan & (a_is_zero | b_is_zero);

    // 11-bit full mantissas with implicit leading 1 (denormals flushed above)
    wire [10:0] ma_f = {1'b1, ma};
    wire [10:0] mb_f = {1'b1, mb};

    // 22-bit product: min 2^20 (1024×1024), max ≈ 2^22-4096 (2047×2047)
    wire [21:0] mp = ma_f * mb_f;

    // Normalise: leading 1 at bit 21 (product ≥ 2^21) or bit 20
    wire norm_21 = mp[21];

    // FP32 biased exponent
    wire [9:0] e32 = {5'b0, ea} + {5'b0, eb} + (norm_21 ? 10'd98 : 10'd97);

    // FP32 mantissa (23-bit, bits after the leading 1)
    wire [22:0] m32 = norm_21 ? {mp[20:0], 2'b00} : {mp[19:0], 3'b00};

    wire [31:0] z_nan  = {1'b0, 8'hFF, 23'h400000};   // quiet NaN
    wire [31:0] z_inf  = {s_z,  8'hFF, 23'h000000};
    wire [31:0] z_zero = {s_z,  31'h0};
    wire [31:0] z_norm = {s_z,  e32[7:0], m32};

    assign z = is_nan  ? z_nan  :
               is_inf  ? z_inf  :
               is_zero ? z_zero : z_norm;

endmodule
