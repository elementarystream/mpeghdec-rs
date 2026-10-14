// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/fixpoint_math.h, libFDK/include/arm/fixpoint_math_arm.h,
// libFDK/src/fixpoint_math.cpp and libFDK/src/arm/fixpoint_math_arm.cpp.
//
// On AArch64 the reference implementation computes schur_div(), sqrtFixp(), invFixp() and
// invSqrtNorm2() with single precision floating point instructions (scvtf/fdiv/fsqrt/fcvtzs).
// Rust's `f32` arithmetic and saturating, truncating `as i32` casts reproduce them exactly.

//! Fixed-point specific mathematical functions.

use crate::common_fix::*;
use crate::scale::{saturate_shift, scale_value, scale_value_saturate};
use crate::tools_rom::{INV_COUNT, SQRT_TAB};
use crate::x87::F80;

pub const LD_DATA_SCALING: f32 = 64.0;
/// `pow(2, LD_DATA_SHIFT) = LD_DATA_SCALING`
pub const LD_DATA_SHIFT: i32 = 6;
pub const MAX_LD_PRECISION: usize = 10;
pub const LD_PRECISION: usize = 10;

/// Taylor series coefficients for ln(1-x), centered at 0 (MacLaurin polynomial). (`ldCoeff`)
pub static LD_COEFF: [FixpSgl; MAX_LD_PRECISION] = [
    fl2fxconst_sgl(-1.0),
    fl2fxconst_sgl(-1.0 / 2.0),
    fl2fxconst_sgl(-1.0 / 3.0),
    fl2fxconst_sgl(-1.0 / 4.0),
    fl2fxconst_sgl(-1.0 / 5.0),
    fl2fxconst_sgl(-1.0 / 6.0),
    fl2fxconst_sgl(-1.0 / 7.0),
    fl2fxconst_sgl(-1.0 / 8.0),
    fl2fxconst_sgl(-1.0 / 9.0),
    fl2fxconst_sgl(-1.0 / 10.0),
];

pub const SQRT_BITS: i32 = 7;
pub const SQRT_VALUES: usize = 128 + 2;
pub const SQRT_BITS_MASK: i32 = 0x7f;
pub const SQRT_FRACT_BITS_MASK: i32 = 0x007F_FFFF;

/// Convert a Q1.31 value to `f32` (`scvtf s, w, #31`).
#[inline(always)]
fn q31_to_f32(x: FixpDbl) -> f32 {
    x as f32 * (1.0 / 2147483648.0)
}

/// Convert an `f32` to Q1.31 with truncation and saturation (`fcvtzs w, s, #31`).
#[inline(always)]
fn f32_to_q31(x: f32) -> FixpDbl {
    (x * 2147483648.0) as i32
}

/// Conversion of a `double` to `INT` like the x86 build (`cvttsd2si`): truncation, and
/// `INT_MIN` for NaN and values out of range (where the Rust `as` conversion saturates).
#[inline(always)]
pub fn x86_f64_to_i32(x: f64) -> i32 {
    if x > -2147483649.0 && x < 2147483648.0 {
        x as i32
    } else {
        i32::MIN
    }
}

/// `(float)(1.0 / (float)op)` of the x86 builds: in double precision on x86-64; with the x87
/// unit on 32 bit x86 (`fld1`, `fidivl op` in extended precision without rounding `op` to
/// `float`, `fstps`).
#[inline(always)]
fn x86_inv_f32(op: i32) -> f32 {
    if crate::ARCH_X86_32 {
        F80::ONE.fdiv(F80::from_i32(op)).to_f32()
    } else {
        (1.0f64 / op as f32 as f64) as f32
    }
}

/// `frexpf(x, &e)` (C library): `x = m * 2^e` with `0.5 <= |m| < 1`; `e = 0` for 0, infinities
/// and NaN (which are returned unchanged).
fn frexpf(x: f32) -> (f32, i32) {
    if x == 0.0 || !x.is_finite() {
        return (x, 0);
    }
    let bits = x.to_bits();
    let biased = ((bits >> 23) & 0xff) as i32;
    if biased == 0 {
        // subnormal: normalize first
        let (m, e) = frexpf(x * 16777216.0); // 2^24
        return (m, e - 24);
    }
    (f32::from_bits((bits & 0x807f_ffff) | (126 << 23)), biased - 126)
}

/// `fIsLessThan(FIXP_DBL a_m, INT a_e, FIXP_DBL b_m, INT b_e)`: `a_m*2^a_e < b_m*2^b_e`
#[inline]
pub fn f_is_less_than(mut a_m: FixpDbl, mut a_e: i32, mut b_m: FixpDbl, mut b_e: i32) -> bool {
    let n = fixnorm_d(a_m);
    a_m <<= n;
    a_e -= n;
    let n = fixnorm_d(b_m);
    b_m <<= n;
    b_e -= n;
    if a_m == 0 {
        a_e = b_e;
    }
    if b_m == 0 {
        b_e = a_e;
    }
    if a_e > b_e {
        (b_m >> f_min(a_e - b_e, DFRACT_BITS - 1)) > a_m
    } else {
        (a_m >> f_min(b_e - a_e, DFRACT_BITS - 1)) < b_m
    }
}

/// `fIsLessThan(FIXP_SGL a_m, INT a_e, FIXP_SGL b_m, INT b_e)`
#[inline]
pub fn f_is_less_than_sgl(mut a_m: FixpSgl, mut a_e: i32, mut b_m: FixpSgl, mut b_e: i32) -> bool {
    let n = fixnorm_s(a_m);
    a_m = ((a_m as i32) << n) as i16;
    a_e -= n;
    let n = fixnorm_s(b_m);
    b_m = ((b_m as i32) << n) as i16;
    b_e -= n;
    if a_m == 0 {
        a_e = b_e;
    }
    if b_m == 0 {
        b_e = a_e;
    }
    if a_e > b_e {
        ((b_m as i32) >> f_min(a_e - b_e, FRACT_BITS - 1)) > a_m as i32
    } else {
        ((a_m as i32) >> f_min(b_e - a_e, FRACT_BITS - 1)) < b_m as i32
    }
}

/// `CalcLdData(op)`: deprecated alias of `fLog2(op, 0)`.
#[inline]
pub fn calc_ld_data(op: FixpDbl) -> FixpDbl {
    f_log2_noexp(op, 0)
}

/// `LdDataVector(FIXP_DBL* srcVector, FIXP_DBL* destVector, INT number)`
pub fn ld_data_vector(src: &[FixpDbl], dst: &mut [FixpDbl]) {
    for (d, &s) in dst.iter_mut().zip(src) {
        *d = f_log2_noexp(s, 0);
    }
}

/// `CalcInvLdData(x)`: delivers `2^(x * LD_DATA_SCALING)`, the inverse of `CalcLdData()`.
#[inline]
pub fn calc_inv_ld_data(x: FixpDbl) -> FixpDbl {
    let set_zero: u32 = if x < fl2fxconst_dbl(-31.0 / 64.0) { 0 } else { 1 };
    let set_max = (x >= fl2fxconst_dbl(31.0 / 64.0)) | (x == fl2fxconst_dbl(0.0));
    if set_max {
        MAXVAL_DBL
    } else {
        let frac = (x & 0x3FF) as FixpSgl;
        let index3 = ((x >> 10) as u32 & 0x1F) as usize;
        let index2 = ((x >> 15) as u32 & 0x1F) as usize;
        let index1 = ((x >> 20) as u32 & 0x1F) as usize;
        let exp = f_min(31, if x > 0 { 31 - (x >> 25) } else { -(x >> 25) });
        let lookup1 = EXP2_TAB_LONG[index1].wrapping_mul(set_zero);
        let lookup2 = EXP2W_TAB_LONG[index2];
        let lookup3 = EXP2X_TAB_LONG[index3];
        let lookup3f = lookup3.wrapping_add(f_mult_div2(0x0016302F_i32, frac) as u32);
        let lookup12 = f_mult(lookup1 as i32, lookup2 as i32) as u32;
        let lookup = f_mult(lookup12 as i32, lookup3f as i32) as u32;
        ((lookup << 3) >> exp) as FixpDbl
    }
}

/// `InitLdInt()`: nothing to do, the table is precomputed.
#[inline]
pub fn init_ld_int() {}

pub const LD_INT_TAB_LEN: i32 = 193;

/// `CalcLdInt(INT i)`: `ld(i) / LD_DATA_SCALING` for `0 < i < LD_INT_TAB_LEN`, 0 otherwise.
#[inline]
pub fn calc_ld_int(i: i32) -> FixpDbl {
    if i > 0 && i < LD_INT_TAB_LEN {
        LD_INT_COEFF[i as usize]
    } else {
        0
    }
}

/// `sqrtFixp_lookup(FIXP_DBL x)`
#[inline]
pub fn sqrt_fixp_lookup(x: FixpDbl) -> FixpDbl {
    let mut y = x as u32;
    let mut t: u32 = 0;
    if y != 0 {
        let zeros = fixnormz_d(y as i32) & 0x1e;
        y <<= zeros;
        let idx = ((y >> 26) - 16) as usize;
        let frac = ((y >> 10) & 0xffff) as u16;
        let nfrac = 0xffff ^ frac;
        t = (nfrac as u32)
            .wrapping_mul(SQRT_TAB[idx] as u32)
            .wrapping_add((frac as u32).wrapping_mul(SQRT_TAB[idx + 1] as u32));
        t >>= zeros >> 1;
    }
    t as FixpDbl
}

/// `sqrtFixp_lookup(FIXP_DBL x, INT* x_e)`
#[inline]
pub fn sqrt_fixp_lookup_e(x: FixpDbl, x_e: &mut i32) -> FixpDbl {
    let mut y = x as u32;
    if x == 0 {
        return x;
    }
    // Normalize
    let mut e = fixnormz_d(y as i32);
    y <<= e;
    e = *x_e - e + 2;
    // Correct odd exponent.
    if e & 1 != 0 {
        y >>= 1;
        e += 1;
    }
    // Get square root
    let idx = ((y >> 26) - 16) as usize;
    let frac = ((y >> 10) & 0xffff) as u16;
    let nfrac = 0xffff ^ frac;
    let t = (nfrac as u32)
        .wrapping_mul(SQRT_TAB[idx] as u32)
        .wrapping_add((frac as u32).wrapping_mul(SQRT_TAB[idx + 1] as u32));
    // Write back exponent
    *x_e = e >> 1;
    (t >> 1) as FixpDbl
}

/// `InitInvSqrtTab()`: nothing to do, the table is precomputed.
#[inline]
pub fn init_inv_sqrt_tab() {}

/// `invSqrtNorm2(FIXP_DBL op, INT* shift)`: delivers `1/sqrt(op)` normalized to .5...1 and the
/// shift value of the output, i.e. `1/sqrt(op) = result * 2^shift`.
///
/// AArch64: single precision square root and division of the normalized input. x86
/// (`fixpoint_math_x86.h`): double precision `1.0 / sqrt(0.5 * op)`, rounded to single
/// precision and normalized with `frexpf()`.
#[inline]
pub fn inv_sqrt_norm2(mut op: FixpDbl, shift: &mut i32) -> FixpDbl {
    if op == 0 {
        *shift = 16;
        return MAXVAL_DBL;
    }
    if crate::ARCH_X86_32 {
        // x87: `fild`, `fmuls 0.5f`, `fsqrt`, `fdivr 1` in extended precision, `fstps`
        let v = F80::from_i32(op).fmul(F80::from_f32(0.5)).sqrt();
        let (m, e) = frexpf(F80::ONE.fdiv(v).to_f32());
        *shift = e + 15;
        return x86_f64_to_i32(m as f64 * 2147483648.0);
    }
    if crate::ARCH_X86 {
        let result = (1.0f64 / ((0.5f32 * op as f32) as f64).sqrt()) as f32;
        let (m, e) = frexpf(result);
        *shift = e + 15;
        // ldexp(m, 31) is exact
        return x86_f64_to_i32(m as f64 * 2147483648.0);
    }
    let exponent = fixnormz_d(op) - 1;
    op <<= exponent & !1;
    *shift = 1 + (exponent >> 1);
    f32_to_q31(0.5f32 / q31_to_f32(op).sqrt())
}

/// `sqrtFixp(FIXP_DBL op)`: square root for values in `0 .. 0x7FFFFFFF`.
///
/// AArch64: single precision square root of `op / 2^31`. x86: double precision square root of
/// `(float)op`, scaled with `46340.9492f` (about `2^15.5`).
#[inline]
pub fn sqrt_fixp(op: FixpDbl) -> FixpDbl {
    if crate::ARCH_X86_32 {
        // x87: `fild`, `fsqrt`, `fmuls` in extended precision, `fistp`
        return F80::from_i32(op).sqrt().fmul(F80::from_f32(46340.9492)).to_i32_trunc();
    }
    if crate::ARCH_X86 {
        return x86_f64_to_i32((op as f32 as f64).sqrt() * 46340.9492f32 as f64);
    }
    f32_to_q31(q31_to_f32(op).sqrt())
}

/// `invFixp(FIXP_DBL op)`: `1.0 / op` where `op` is an integer; the result has an implicit
/// exponent of 31.
///
/// AArch64: single precision division. x86: double precision division rounded to single
/// precision (the scaling by `2^31` with `frexpf()` / `ldexpf()` is exact); 0 and 1 give
/// `MAXVAL_DBL`.
#[inline]
pub fn inv_fixp(op: FixpDbl) -> FixpDbl {
    if crate::ARCH_X86 {
        if op == 0 || op == 1 {
            return MAXVAL_DBL;
        }
        let result = x86_inv_f32(op);
        return x86_f64_to_i32(result as f64 * 2147483648.0);
    }
    f32_to_q31(1.0f32 / op as f32)
}

/// `invFixp(FIXP_DBL op_m, int* op_e)`: `1.0 / (op_m * 2^op_e)`.
///
/// AArch64: `schur_div()` of the normalized input (`MAXVAL_DBL` for `op_m <= 0`). x86: double
/// precision division rounded to single precision and normalized with `frexpf()`
/// (`MAXVAL_DBL` for 0 and 1).
#[inline]
pub fn inv_fixp_e(mut op_m: FixpDbl, op_e: &mut i32) -> FixpDbl {
    if crate::ARCH_X86 {
        if op_m == 0 || op_m == 1 {
            *op_e = 31 - *op_e;
            return MAXVAL_DBL;
        }
        let (m, e) = frexpf(x86_inv_f32(op_m));
        *op_e = e - *op_e + 31;
        return x86_f64_to_i32(m as f64 * 2147483648.0);
    }
    if op_m <= 0 {
        *op_e = 31 - *op_e;
        return MAXVAL_DBL;
    }
    let shl = count_leading_bits(op_m);
    op_m <<= shl;
    *op_e = shl + 1 - *op_e;
    schur_div(0x4000_0000, op_m, 16)
}

/// `schur_div(FIXP_DBL num, FIXP_DBL denum, INT count)`: `num / denum` (`count` is ignored).
///
/// AArch64: single precision division. x86 (`fixpoint_math_x86.h`): 64 bit integer division
/// `(num << 31) / denum`, `MAXVAL_DBL` for `num == denum`.
#[inline]
pub fn schur_div(num: FixpDbl, denum: FixpDbl, _count: i32) -> FixpDbl {
    if crate::ARCH_X86 {
        return if num == denum {
            MAXVAL_DBL
        } else {
            (((num as i64) << 31) / denum as i64) as i32
        };
    }
    f32_to_q31(q31_to_f32(num) / q31_to_f32(denum))
}

/// `fMultNorm(FIXP_DBL f1, FIXP_DBL f2, INT* result_e)`: multiply with normalization.
pub fn f_mult_norm(mut f1: FixpDbl, mut f2: FixpDbl, result_e: &mut i32) -> FixpDbl {
    if f1 == 0 || f2 == 0 {
        *result_e = 0;
        return 0;
    }
    let norm_f1 = count_leading_bits(f1);
    f1 <<= norm_f1;
    let norm_f2 = count_leading_bits(f2);
    f2 <<= norm_f2;
    if f1 == MINVAL_DBL && f2 == MINVAL_DBL {
        *result_e = -(norm_f1 + norm_f2 - 1);
        -(MINVAL_DBL >> 1)
    } else {
        *result_e = -(norm_f1 + norm_f2);
        f_mult(f1, f2)
    }
}

/// `fMultNorm(FIXP_DBL f1, FIXP_DBL f2)`: product with exponent 0 (saturated).
#[inline]
pub fn f_mult_norm_noexp(f1: FixpDbl, f2: FixpDbl) -> FixpDbl {
    let mut e = 0;
    let m = f_mult_norm(f1, f2, &mut e);
    scale_value_saturate(m, e)
}

/// `fMultNorm(FIXP_DBL f1_m, INT f1_e, FIXP_DBL f2_m, INT f2_e, INT result_e)`: product with the
/// given exponent `result_e`.
#[inline]
pub fn f_mult_norm_to(
    f1_m: FixpDbl,
    f1_e: i32,
    f2_m: FixpDbl,
    f2_e: i32,
    result_e: i32,
) -> FixpDbl {
    let mut e = 0;
    let m = f_mult_norm(f1_m, f2_m, &mut e);
    scale_value_saturate(m, e + f1_e + f2_e - result_e)
}

/// `fMultI(FIXP_DBL a, INT b)`: fractional times integer, rounded to nearest.
#[inline]
pub fn f_mult_i(a: FixpDbl, b: i32) -> i32 {
    let mut m_e = 0;
    let mut m = f_mult_norm(a, b, &mut m_e);
    if m_e < 0 {
        if m_e > -DFRACT_BITS {
            m >>= (-m_e) - 1;
            (m + 1) >> 1
        } else {
            0
        }
    } else {
        scale_value_saturate(m, m_e)
    }
}

/// `fMultIfloor(FIXP_DBL a, INT b)`: fractional times integer, rounded down.
#[inline]
pub fn f_mult_i_floor(a: FixpDbl, b: i32) -> i32 {
    let mut m_e = 0;
    let m = f_mult_norm(a, b, &mut m_e);
    if m_e < 0 {
        if m_e > -DFRACT_BITS {
            m >> (-m_e)
        } else if m < 0 {
            -1
        } else {
            0
        }
    } else {
        scale_value_saturate(m, m_e)
    }
}

/// `fMultIceil(FIXP_DBL a, INT b)`: fractional times integer, rounded up.
#[inline]
pub fn f_mult_i_ceil(a: FixpDbl, b: i32) -> i32 {
    let mut m_e = 0;
    let m = f_mult_norm(a, b, &mut m_e);
    if m_e < 0 {
        if m_e > -(DFRACT_BITS - 1) {
            let mut mi = m >> (-m_e);
            if m & ((1 << (-m_e)) - 1) != 0 {
                mi += 1;
            }
            mi
        } else if m > 0 {
            1
        } else if m_e == -(DFRACT_BITS - 1) && m == MINVAL_DBL {
            -1
        } else {
            0
        }
    } else {
        scale_value_saturate(m, m_e)
    }
}

/// `fDivNorm(FIXP_DBL num, FIXP_DBL denom, INT* result_e)`: `num / denom` for `num >= 0`,
/// `denom > 0` with normalization of the inputs.
pub fn f_div_norm(mut l_num: FixpDbl, mut l_denum: FixpDbl, result_e: &mut i32) -> FixpDbl {
    if l_num == 0 {
        *result_e = 0;
        return 0;
    }
    let norm_num = count_leading_bits(l_num);
    l_num <<= norm_num;
    l_num >>= 1;
    *result_e = -norm_num + 1;
    let norm_den = count_leading_bits(l_denum);
    l_denum <<= norm_den;
    *result_e -= -norm_den;
    schur_div(l_num, l_denum, FRACT_BITS)
}

/// `fDivNorm(FIXP_DBL num, FIXP_DBL denom)`: `num / denom` with exponent 0 for `denom >= num`.
pub fn f_div_norm_noexp(num: FixpDbl, denom: FixpDbl) -> FixpDbl {
    let mut e = 0;
    let res = f_div_norm(num, denom, &mut e);
    // Avoid overflow since we must output a value with exponent 0; there is no other choice
    // than saturating to almost 1.0f.
    if res == (1 << (DFRACT_BITS - 2)) && e == 1 {
        MAXVAL_DBL
    } else {
        scale_value(res, e)
    }
}

/// `fDivNormSigned(FIXP_DBL num, FIXP_DBL denom)`: signed division with exponent 0.
pub fn f_div_norm_signed_noexp(num: FixpDbl, denom: FixpDbl) -> FixpDbl {
    if denom == 0 {
        return MAXVAL_DBL;
    }
    let sign = (num >= 0) != (denom >= 0);
    let mut e = 0;
    let res = f_div_norm_signed(num, denom, &mut e);
    // Saturate since we must output a value with exponent 0
    if e > 0 && f_abs(res) >= fl2fxconst_dbl(0.5) {
        if sign {
            MINVAL_DBL
        } else {
            MAXVAL_DBL
        }
    } else {
        scale_value(res, e)
    }
}

/// `fDivNormSigned(FIXP_DBL L_num, FIXP_DBL L_denum, INT* result_e)`
pub fn f_div_norm_signed(mut l_num: FixpDbl, mut l_denum: FixpDbl, result_e: &mut i32) -> FixpDbl {
    let sign = (l_num >= 0) != (l_denum >= 0);
    if l_num == 0 {
        *result_e = 0;
        return 0;
    }
    if l_denum == 0 {
        *result_e = 14;
        return MAXVAL_DBL;
    }
    let norm_num = count_leading_bits(l_num);
    l_num <<= norm_num;
    l_num >>= 2;
    l_num = f_abs(l_num);
    *result_e = -norm_num + 1;
    let norm_den = count_leading_bits(l_denum);
    l_denum <<= norm_den;
    l_denum >>= 1;
    l_denum = f_abs(l_denum);
    *result_e -= -norm_den;
    let div = schur_div(l_num, l_denum, FRACT_BITS);
    if sign {
        -div
    } else {
        div
    }
}

/// `fDivNormHighPrec(FIXP_DBL num, FIXP_DBL denom, INT* result_e)`
pub fn f_div_norm_high_prec(mut num: FixpDbl, mut denom: FixpDbl, result_e: &mut i32) -> FixpDbl {
    if num == 0 {
        *result_e = 0;
        return 0;
    }
    let norm_num = count_leading_bits(num);
    num <<= norm_num;
    num >>= 1;
    *result_e = -norm_num + 1;
    let norm_den = count_leading_bits(denom);
    denom <<= norm_den;
    *result_e -= -norm_den;
    schur_div(num, denom, 31)
}

/// `fAdjust(FIXP_DBL a_m, INT* pA_e)`: adjust mantissa to exponent -1.
#[inline]
pub fn f_adjust(a_m: FixpDbl, p_a_e: &mut i32) -> FixpDbl {
    let shift = fixnorm_d(a_m) - 1;
    *p_a_e -= shift;
    scale_value(a_m, shift)
}

/// `fAddNorm(FIXP_DBL a_m, INT a_e, FIXP_DBL b_m, INT b_e, INT* pResult_e)`
#[inline]
pub fn f_add_norm(
    mut a_m: FixpDbl,
    mut a_e: i32,
    mut b_m: FixpDbl,
    mut b_e: i32,
    p_result_e: &mut i32,
) -> FixpDbl {
    // If one of the summands is zero, return the other. This is necessary for the summation of a
    // very small number to zero.
    if a_m == 0 {
        *p_result_e = b_e;
        return b_m;
    }
    if b_m == 0 {
        *p_result_e = a_e;
        return a_m;
    }
    a_m = f_adjust(a_m, &mut a_e);
    b_m = f_adjust(b_m, &mut b_e);
    if a_e > b_e {
        *p_result_e = a_e;
        a_m + (b_m >> f_min(a_e - b_e, DFRACT_BITS - 1))
    } else {
        *p_result_e = b_e;
        (a_m >> f_min(b_e - a_e, DFRACT_BITS - 1)) + b_m
    }
}

/// `fAddNorm(FIXP_DBL a_m, INT a_e, FIXP_DBL b_m, INT b_e, INT result_e)`
#[inline]
pub fn f_add_norm_to(a_m: FixpDbl, a_e: i32, b_m: FixpDbl, b_e: i32, result_e: i32) -> FixpDbl {
    let a_m = scale_value(a_m, a_e - result_e);
    let b_m = scale_value(b_m, b_e - result_e);
    a_m.wrapping_add(b_m)
}

pub const MAX_POW2_PRECISION: usize = 8;
pub const POW2_PRECISION: usize = 5;

/// Taylor series coefficients of the function 2^x (`pow2Coeff`):
/// `pow2Coeff[i-1] = ln(2)^i / i!`, `i = 1..MAX_POW2_PRECISION`.
static POW2_COEFF: [FixpSgl; MAX_POW2_PRECISION] = [
    fl2fxconst_sgl(0.693147180559945309417232121458177),
    fl2fxconst_sgl(0.240226506959100712333551263163332),
    fl2fxconst_sgl(0.0555041086648215799531422637686218),
    fl2fxconst_sgl(0.00961812910762847716197907157365887),
    fl2fxconst_sgl(0.00133335581464284434234122219879962),
    fl2fxconst_sgl(1.54035303933816099544370973327423e-4),
    fl2fxconst_sgl(1.52527338040598402800254390120096e-5),
    fl2fxconst_sgl(1.32154867901443094884037582282884e-6),
];

/// `f2Pow(const FIXP_DBL exp_m, const INT exp_e, INT* result_e)`: `2 ^ (exp_m * 2^exp_e)`
pub fn f2_pow(exp_m: FixpDbl, exp_e: i32, result_e: &mut i32) -> FixpDbl {
    let mut int_part: i32;
    let mut frac_part: FixpDbl;
    if exp_e > 0 {
        let exp_bits = DFRACT_BITS - 1 - exp_e;
        // Out-of-range shift amounts are masked modulo 32 like in the AArch64 build.
        int_part = exp_m.wrapping_shr(exp_bits as u32);
        frac_part = exp_m.wrapping_sub(int_part.wrapping_shl(exp_bits as u32));
        frac_part = frac_part.wrapping_shl(exp_e as u32);
    } else {
        int_part = 0;
        frac_part = exp_m.wrapping_shr(exp_e.wrapping_neg() as u32);
    }
    // Best accuracy is around 0, so try to get there with the fractional part.
    if frac_part > fl2fxconst_dbl(0.5) {
        int_part += 1;
        frac_part = frac_part.wrapping_add(fl2fxconst_dbl(-1.0));
    }
    if frac_part < fl2fxconst_dbl(-0.5) {
        int_part -= 1;
        frac_part = -(fl2fxconst_dbl(-1.0).wrapping_sub(frac_part));
    }
    // "+ 1" compensates fMultAddDiv2() of the polynomial evaluation below.
    *result_e = f_min(int_part, MAXVAL_DBL - 1) + 1;
    // Evaluate taylor polynomial which approximates 2^x
    let mut p = frac_part;
    // First taylor series coefficient a_0 = 1.0, scaled by 0.5 due to fMultDiv2().
    let mut result_m = fl2fxconst_dbl(1.0 / 2.0);
    for coeff in POW2_COEFF.iter().take(POW2_PRECISION) {
        // next taylor series term: a_i * x^i, x=0
        result_m = f_mult_add_div2(result_m, *coeff, p);
        p = f_mult(p, frac_part);
    }
    result_m
}

/// `f2Pow(const FIXP_DBL exp_m, const INT exp_e)`: mantissa with implicit exponent 0.
pub fn f2_pow_noexp(exp_m: FixpDbl, exp_e: i32) -> FixpDbl {
    let mut result_e = 0;
    let result_m = f2_pow(exp_m, exp_e, &mut result_e);
    let result_e = f_min(DFRACT_BITS - 1, f_max(-(DFRACT_BITS - 1), result_e));
    scale_value(result_m, result_e)
}

/// `fPow(FIXP_DBL base_m, INT base_e, FIXP_DBL exp_m, INT exp_e, INT* result_e)`
pub fn f_pow(
    base_m: FixpDbl,
    base_e: i32,
    mut exp_m: FixpDbl,
    mut exp_e: i32,
    result_e: &mut i32,
) -> FixpDbl {
    if base_m <= 0 {
        *result_e = 0;
        return 0;
    }
    // Calc log2 of base
    let mut baselg2_e = 0;
    let base_lg2 = f_log2(base_m, base_e, &mut baselg2_e);
    // Prepare exp
    let leading_bits = count_leading_bits(f_abs(exp_m));
    exp_m <<= leading_bits;
    exp_e -= leading_bits;
    // Calc base pow exp
    let ans_lg2 = f_mult(base_lg2, exp_m);
    let ans_lg2_e = exp_e + baselg2_e;
    // Calc antilog
    f2_pow(ans_lg2, ans_lg2_e, result_e)
}

/// `fLdPow(FIXP_DBL baseLd_m, INT baseLd_e, FIXP_DBL exp_m, INT exp_e, INT* result_e)`:
/// `x ^ (exp_m * 2^exp_e)` where `log2(x) = baseLd_m * 2^baseLd_e`.
pub fn f_ld_pow(
    base_ld_m: FixpDbl,
    base_ld_e: i32,
    mut exp_m: FixpDbl,
    mut exp_e: i32,
    result_e: &mut i32,
) -> FixpDbl {
    // Prepare exp
    let leading_bits = count_leading_bits(f_abs(exp_m));
    exp_m <<= leading_bits;
    exp_e -= leading_bits;
    // Calc base pow exp
    let ans_lg2 = f_mult(base_ld_m, exp_m);
    let ans_lg2_e = exp_e + base_ld_e;
    // Calc antilog
    f2_pow(ans_lg2, ans_lg2_e, result_e)
}

/// `fLdPow(FIXP_DBL baseLd_m, INT baseLd_e, FIXP_DBL exp_m, INT exp_e)`: implicit exponent 0.
pub fn f_ld_pow_noexp(base_ld_m: FixpDbl, base_ld_e: i32, exp_m: FixpDbl, exp_e: i32) -> FixpDbl {
    let mut result_e = 0;
    let result_m = f_ld_pow(base_ld_m, base_ld_e, exp_m, exp_e, &mut result_e);
    saturate_shift(result_m, -result_e, DFRACT_BITS)
}

/// `fPowInt(FIXP_DBL base_m, INT base_e, INT N, INT* result_e)`: `(base_m * 2^base_e) ^ N`
pub fn f_pow_int(mut base_m: FixpDbl, mut base_e: i32, exp: i32, p_result_e: &mut i32) -> FixpDbl {
    if exp != 0 {
        let mut result_e = 0;
        let mut result: FixpDbl;
        if base_m != 0 {
            let leading_bits = count_leading_bits(base_m);
            base_m <<= leading_bits;
            base_e -= leading_bits;
            result = base_m;
            let mut i = 1;
            while i < f_abs(exp) {
                result = f_mult(result, base_m);
                i += 1;
            }
            if exp < 0 {
                // 1.0 / ans
                result = f_div_norm(fl2fxconst_dbl(0.5), result, &mut result_e);
                result_e += 1;
            } else {
                let ans_scale = count_leading_bits(result);
                result <<= ans_scale;
                result_e -= ans_scale;
            }
            result_e += exp * base_e;
        } else {
            result = 0;
        }
        *p_result_e = result_e;
        result
    } else {
        *p_result_e = 1;
        fl2fxconst_dbl(0.5)
    }
}

/// `flog2_func1()` (AArch64 replacement for the Taylor loop in `fLog2`, `LD_PRECISION == 10`).
#[inline(always)]
fn flog2_func1(x2_m: FixpDbl) -> FixpDbl {
    let pld_coeff = &LD_COEFF[2..];
    let mut result_m = -(x2_m / 2); // 1st iteration with factor -0.5*1/1
    let sqx2_m = f_mult_div2(x2_m, x2_m) << 2; // sqx2_m: x2_m^2<<1
    result_m -= sqx2_m / 8; // 2nd iteration with factor -0.5*1/2
    let mut px2_m = f_mult_div2(x2_m, sqx2_m); // px2_m: x2_m^3
    let mut px3_m = f_mult_div2(sqx2_m, sqx2_m) >> 1; // px3_m: x2_m^4
    let mut coeff2 = pld_coeff[0]; // factor -0.5*1/3
    let mut coeff3 = pld_coeff[1]; // factor -0.5*1/4
    result_m = f_mult_add_div2(result_m, coeff2, px2_m);
    px2_m = f_mult_div2(px2_m, sqx2_m); // px2_m: x2_m^5
    result_m = f_mult_add_div2(result_m, coeff3, px3_m);
    px3_m = f_mult_div2(px3_m, sqx2_m); // px3_m: x2_m^6
    coeff2 = pld_coeff[2]; // factor -0.5*1/5
    result_m = f_mult_add_div2(result_m, coeff2, px2_m);
    // LD_PRECISION == 10
    coeff3 = pld_coeff[3]; // factor -0.5*1/6
    px2_m = f_mult_div2(px2_m, sqx2_m); // px2_m: x2_m^7
    result_m = f_mult_add_div2(result_m, coeff3, px3_m);
    px3_m = f_mult_div2(px3_m, sqx2_m); // px3_m: x2_m^8
    coeff2 = pld_coeff[4]; // factor -0.5*1/7
    coeff3 = pld_coeff[5]; // factor -0.5*1/8
    result_m = f_mult_add_div2(result_m, coeff2, px2_m);
    px2_m = f_mult_div2(px2_m, sqx2_m); // px2_m: x2_m^9
    result_m = f_mult_add_div2(result_m, coeff3, px3_m);
    px3_m = f_mult_div2(px3_m, sqx2_m); // px3_m: x2_m^10
    coeff2 = pld_coeff[6]; // factor -0.5*1/9
    coeff3 = pld_coeff[7]; // factor -0.5*1/10
    result_m = f_mult_add_div2(result_m, coeff2, px2_m);
    result_m = f_mult_add_div2(result_m, coeff3, px3_m);
    result_m
}

/// `fLog2(FIXP_DBL x_m, INT x_e, INT* result_e)`: logarithm of base 2 of `x_m * 2^x_e`.
#[inline]
pub fn f_log2(x_m: FixpDbl, mut x_e: i32, result_e: &mut i32) -> FixpDbl {
    // Short cut for zero and negative numbers.
    if x_m <= 0 {
        *result_e = DFRACT_BITS - 1;
        return fl2fxconst_dbl(-1.0);
    }
    // Move input value x_m * 2^x_e toward 1.0, where the taylor approximation of the function
    // log(1-x) centered at 0 is most accurate.
    let b_norm = fixnormz_d(x_m) - 1;
    let mut x2_m = x_m << b_norm;
    x_e -= b_norm;
    // map x from log(x) domain to log(1-x) domain.
    x2_m = -(x2_m.wrapping_add(fl2fxconst_dbl(-1.0)));
    // Taylor polynomial approximation of ln(1-x): unrolled on AArch64 (`flog2_func1()`), the
    // loop of the generic code on x86
    let mut result_m = if crate::ARCH_X86 {
        let mut result_m: FixpDbl = 0;
        let mut px2_m = x2_m;
        for &coeff in &LD_COEFF {
            result_m = f_mult_add_div2(result_m, coeff, px2_m);
            px2_m = f_mult(px2_m, x2_m);
        }
        result_m
    } else {
        flog2_func1(x2_m)
    };
    // Multiply result with 1/ln(2) = 1.0 + 0.442695040888 (get log2(x) from ln(x) result).
    result_m = f_mult_add_div2(
        result_m,
        result_m,
        fl2fxconst_dbl(2.0 * 0.4426950408889634073599246810019),
    );
    // Add exponent part. log2(x_m * 2^x_e) = log2(x_m) + x_e
    if x_e != 0 {
        let enorm = DFRACT_BITS - fixnorm_d(x_e);
        // The -1 in the right shift of result_m compensates the fMultDiv2() above in the taylor
        // polynomial evaluation loop.
        result_m = (result_m >> (enorm - 1)).wrapping_add(x_e << (DFRACT_BITS - 1 - enorm));
        *result_e = enorm;
    } else {
        // 1 compensates the fMultDiv2() above in the taylor polynomial evaluation loop.
        *result_e = 1;
    }
    result_m
}

/// `fLog2(FIXP_DBL x_m, INT x_e)`: result with implicit exponent `LD_DATA_SHIFT`.
#[inline]
pub fn f_log2_noexp(mut x_m: FixpDbl, x_e: i32) -> FixpDbl {
    if x_m <= 0 {
        x_m = fl2fxconst_dbl(-1.0);
    } else {
        let mut result_e = 0;
        x_m = f_log2(x_m, x_e, &mut result_e);
        x_m = scale_value(x_m, result_e - LD_DATA_SHIFT);
    }
    x_m
}

/// `CalcLog2(FIXP_DBL arg, INT arg_e, INT* result_e)`: deprecated alias of `fLog2`.
#[inline]
pub fn calc_log2(base_m: FixpDbl, base_e: i32, result_e: &mut i32) -> FixpDbl {
    f_log2(base_m, base_e, result_e)
}

/// `fAddSaturate(const FIXP_SGL a, const FIXP_SGL b)`
#[inline]
pub fn f_add_saturate_sgl(a: FixpSgl, b: FixpSgl) -> FixpSgl {
    let sum = a as i32 + b as i32;
    f_max(f_min(sum, MAXVAL_SGL), MINVAL_SGL) as i16
}

/// `fAddSaturate(const FIXP_DBL a, const FIXP_DBL b)`
#[inline]
pub fn f_add_saturate(a: FixpDbl, b: FixpDbl) -> FixpDbl {
    // The C++ code computes the wrapping sum and replaces it with MAXVAL_DBL / MINVAL_DBL (by the
    // sign of a) when a and b have the same sign and the sum has the other one: that is the
    // saturating addition (which compiles to a single instruction and vectorizes).
    a.saturating_add(b)
}

/// `fNegateSaturate(FIXP_DBL x)`: both `0x80000000` and `0x80000001` map to `0x7fffffff`.
#[inline]
pub fn f_negate_saturate(x: FixpDbl) -> FixpDbl {
    -f_max(MINVAL_DBL + 1, x)
}

/// `fNegateSaturate(FIXP_SGL x)`
#[inline]
pub fn f_negate_saturate_sgl(x: FixpSgl) -> FixpSgl {
    -f_max((MINVAL_SGL + 1) as i16, x)
}

/// `fixp_floorToInt(FIXP_DBL f_inp, INT sf)`
pub fn fixp_floor_to_int(f_inp: FixpDbl, sf: i32) -> i32 {
    f_inp >> ((DFRACT_BITS - 1) - sf)
}

/// `fixp_floor(FIXP_DBL f_inp, INT sf)`
pub fn fixp_floor(f_inp: FixpDbl, sf: i32) -> FixpDbl {
    let floor_int = fixp_floor_to_int(f_inp, sf);
    floor_int << ((DFRACT_BITS - 1) - sf)
}

/// `fixp_ceilToInt(FIXP_DBL f_inp, INT sf)`: `sf` mantissa bits left of the dot.
pub fn fixp_ceil_to_int(f_inp: FixpDbl, sf: i32) -> i32 {
    let sx = (DFRACT_BITS - 1) - sf; // mantissa bits right of dot
    let mask = (1i32 << sx).wrapping_sub(1);
    let mut ceil_int = f_inp >> sx;
    if f_inp & mask != 0 {
        ceil_int += 1; // increment only, if there is at least one set mantissa bit right of dot
    }
    ceil_int
}

/// `fixp_ceil(FIXP_DBL f_inp, INT sf)`
pub fn fixp_ceil(f_inp: FixpDbl, sf: i32) -> FixpDbl {
    let sx = (DFRACT_BITS - 1) - sf;
    let mut ceil_int = fixp_ceil_to_int(f_inp, sf);
    let mask = 1u32 << (DFRACT_BITS - 1);
    ceil_int <<= sx; // no fract warn bec. shift into saturation done on int side
    if f_inp > 0 && (ceil_int as u32 & mask) != 0 {
        ceil_int = ceil_int.wrapping_sub(1);
    }
    ceil_int
}

/// `fixp_truncateToInt(FIXP_DBL f_inp, INT sf)`: remove the fractional part (like a float cast).
pub fn fixp_truncate_to_int(f_inp: FixpDbl, sf: i32) -> i32 {
    let sx = (DFRACT_BITS - 1) - sf;
    let mut fbaccu = f_inp;
    let mask = 1i32 << sx;
    if fbaccu < 0 && (fbaccu & mask.wrapping_sub(1)) != 0 {
        fbaccu = fbaccu.wrapping_add(mask);
    }
    fbaccu >> sx
}

/// `fixp_truncate(FIXP_DBL f_inp, INT sf)`
pub fn fixp_truncate(f_inp: FixpDbl, sf: i32) -> FixpDbl {
    let truncate_int = fixp_truncate_to_int(f_inp, sf);
    truncate_int << ((DFRACT_BITS - 1) - sf)
}

/// `fixp_roundToInt(FIXP_DBL f_inp, INT sf)`
///
/// `sf` should be in `0..=31`; outside of that range the shift amounts are taken modulo 32 like
/// on the reference build (e.g. `pcmLimiter_GetMaxGainReduction()` passes 34 for a gain of 0).
pub fn fixp_round_to_int(f_inp: FixpDbl, sf: i32) -> i32 {
    let sx = (DFRACT_BITS - 1 - sf) as u32;
    let mut rnd = (1u32.wrapping_shl(sx) >> 1) as i32; // rnd becomes 0, if sf==31 and 0x40000000, if sf==0
    let inp = f_inp;
    if inp >= 0 {
        if inp > 0x7FFF_FFFF - rnd {
            rnd = inp.wrapping_shr(sx) + 1;
        } else {
            rnd = (inp + rnd).wrapping_shr(sx);
        }
    } else if inp >= MINVAL_DBL + rnd {
        rnd =
            ((inp.wrapping_neg().wrapping_add(rnd) as u32).wrapping_shr(sx) as i32).wrapping_neg();
    } else {
        rnd = MINVAL_DBL.wrapping_shr(sx);
    }
    rnd
}

/// `fixp_round(FIXP_DBL f_inp, INT sf)`
pub fn fixp_round(f_inp: FixpDbl, sf: i32) -> FixpDbl {
    let sx = DFRACT_BITS - 1 - sf;
    let mut r = fixp_round_to_int(f_inp, sf);
    let mask = 1u32 << (DFRACT_BITS - 1);
    r <<= sx;
    if f_inp > 0 && (r as u32 & mask) != 0 {
        r = r.wrapping_sub(1);
    }
    r
}

/// `GetInvInt(int intValue)`: `1/intValue` for `1 <= intValue < 80`.
#[inline]
pub fn get_inv_int(int_value: i32) -> FixpDbl {
    INV_COUNT[f_min(f_max(int_value, 0), 80 - 1) as usize]
}

pub static EXP2_TAB_LONG: [u32; 32] = [
    0x40000000, 0x4166C34C, 0x42D561B4, 0x444C0740, 0x45CAE0F2, 0x47521CC6, 0x48E1E9BA, 0x4A7A77D4,
    0x4C1BF829, 0x4DC69CDD, 0x4F7A9930, 0x51382182, 0x52FF6B55, 0x54D0AD5A, 0x56AC1F75, 0x5891FAC1,
    0x5A82799A, 0x5C7DD7A4, 0x5E8451D0, 0x60962665, 0x62B39509, 0x64DCDEC3, 0x6712460B, 0x69540EC9,
    0x6BA27E65, 0x6DFDDBCC, 0x70666F76, 0x72DC8374, 0x75606374, 0x77F25CCE, 0x7A92BE8B, 0x7D41D96E,
];
pub static EXP2W_TAB_LONG: [u32; 32] = [
    0x40000000, 0x400B1818, 0x4016321B, 0x40214E0C, 0x402C6BE9, 0x40378BB4, 0x4042AD6D, 0x404DD113,
    0x4058F6A8, 0x40641E2B, 0x406F479E, 0x407A7300, 0x4085A051, 0x4090CF92, 0x409C00C4, 0x40A733E6,
    0x40B268FA, 0x40BD9FFF, 0x40C8D8F5, 0x40D413DD, 0x40DF50B8, 0x40EA8F86, 0x40F5D046, 0x410112FA,
    0x410C57A2, 0x41179E3D, 0x4122E6CD, 0x412E3152, 0x41397DCC, 0x4144CC3B, 0x41501CA0, 0x415B6EFB,
];
pub static EXP2X_TAB_LONG: [u32; 32] = [
    0x40000000, 0x400058B9, 0x4000B173, 0x40010A2D, 0x400162E8, 0x4001BBA3, 0x4002145F, 0x40026D1B,
    0x4002C5D8, 0x40031E95, 0x40037752, 0x4003D011, 0x400428CF, 0x4004818E, 0x4004DA4E, 0x4005330E,
    0x40058BCE, 0x4005E48F, 0x40063D51, 0x40069613, 0x4006EED5, 0x40074798, 0x4007A05B, 0x4007F91F,
    0x400851E4, 0x4008AAA8, 0x4009036E, 0x40095C33, 0x4009B4FA, 0x400A0DC0, 0x400A6688, 0x400ABF4F,
];
pub static LD_INT_COEFF: [i32; 193] = [
    0x80000001u32 as i32,
    0x00000000,
    0x02000000,
    0x032B8034,
    0x04000000,
    0x04A4D3C2,
    0x052B8034,
    0x059D5DA0,
    0x06000000,
    0x06570069,
    0x06A4D3C2,
    0x06EB3A9F,
    0x072B8034,
    0x0766A009,
    0x079D5DA0,
    0x07D053F7,
    0x08000000,
    0x082CC7EE,
    0x08570069,
    0x087EF05B,
    0x08A4D3C2,
    0x08C8DDD4,
    0x08EB3A9F,
    0x090C1050,
    0x092B8034,
    0x0949A785,
    0x0966A009,
    0x0982809D,
    0x099D5DA0,
    0x09B74949,
    0x09D053F7,
    0x09E88C6B,
    0x0A000000,
    0x0A16BAD3,
    0x0A2CC7EE,
    0x0A423162,
    0x0A570069,
    0x0A6B3D79,
    0x0A7EF05B,
    0x0A92203D,
    0x0AA4D3C2,
    0x0AB7110E,
    0x0AC8DDD4,
    0x0ADA3F60,
    0x0AEB3A9F,
    0x0AFBD42B,
    0x0B0C1050,
    0x0B1BF312,
    0x0B2B8034,
    0x0B3ABB40,
    0x0B49A785,
    0x0B584822,
    0x0B66A009,
    0x0B74B1FD,
    0x0B82809D,
    0x0B900E61,
    0x0B9D5DA0,
    0x0BAA708F,
    0x0BB74949,
    0x0BC3E9CA,
    0x0BD053F7,
    0x0BDC899B,
    0x0BE88C6B,
    0x0BF45E09,
    0x0C000000,
    0x0C0B73CB,
    0x0C16BAD3,
    0x0C21D671,
    0x0C2CC7EE,
    0x0C379085,
    0x0C423162,
    0x0C4CABA8,
    0x0C570069,
    0x0C6130AF,
    0x0C6B3D79,
    0x0C7527B9,
    0x0C7EF05B,
    0x0C88983F,
    0x0C92203D,
    0x0C9B8926,
    0x0CA4D3C2,
    0x0CAE00D2,
    0x0CB7110E,
    0x0CC0052B,
    0x0CC8DDD4,
    0x0CD19BB0,
    0x0CDA3F60,
    0x0CE2C97D,
    0x0CEB3A9F,
    0x0CF39355,
    0x0CFBD42B,
    0x0D03FDA9,
    0x0D0C1050,
    0x0D140CA0,
    0x0D1BF312,
    0x0D23C41D,
    0x0D2B8034,
    0x0D3327C7,
    0x0D3ABB40,
    0x0D423B08,
    0x0D49A785,
    0x0D510118,
    0x0D584822,
    0x0D5F7CFF,
    0x0D66A009,
    0x0D6DB197,
    0x0D74B1FD,
    0x0D7BA190,
    0x0D82809D,
    0x0D894F75,
    0x0D900E61,
    0x0D96BDAD,
    0x0D9D5DA0,
    0x0DA3EE7F,
    0x0DAA708F,
    0x0DB0E412,
    0x0DB74949,
    0x0DBDA072,
    0x0DC3E9CA,
    0x0DCA258E,
    0x0DD053F7,
    0x0DD6753E,
    0x0DDC899B,
    0x0DE29143,
    0x0DE88C6B,
    0x0DEE7B47,
    0x0DF45E09,
    0x0DFA34E1,
    0x0E000000,
    0x0E05BF94,
    0x0E0B73CB,
    0x0E111CD2,
    0x0E16BAD3,
    0x0E1C4DFB,
    0x0E21D671,
    0x0E275460,
    0x0E2CC7EE,
    0x0E323143,
    0x0E379085,
    0x0E3CE5D8,
    0x0E423162,
    0x0E477346,
    0x0E4CABA8,
    0x0E51DAA8,
    0x0E570069,
    0x0E5C1D0B,
    0x0E6130AF,
    0x0E663B74,
    0x0E6B3D79,
    0x0E7036DB,
    0x0E7527B9,
    0x0E7A1030,
    0x0E7EF05B,
    0x0E83C857,
    0x0E88983F,
    0x0E8D602E,
    0x0E92203D,
    0x0E96D888,
    0x0E9B8926,
    0x0EA03232,
    0x0EA4D3C2,
    0x0EA96DF0,
    0x0EAE00D2,
    0x0EB28C7F,
    0x0EB7110E,
    0x0EBB8E96,
    0x0EC0052B,
    0x0EC474E4,
    0x0EC8DDD4,
    0x0ECD4012,
    0x0ED19BB0,
    0x0ED5F0C4,
    0x0EDA3F60,
    0x0EDE8797,
    0x0EE2C97D,
    0x0EE70525,
    0x0EEB3A9F,
    0x0EEF69FF,
    0x0EF39355,
    0x0EF7B6B4,
    0x0EFBD42B,
    0x0EFFEBCD,
    0x0F03FDA9,
    0x0F0809CF,
    0x0F0C1050,
    0x0F10113B,
    0x0F140CA0,
    0x0F18028D,
    0x0F1BF312,
    0x0F1FDE3D,
    0x0F23C41D,
    0x0F27A4C0,
    0x0F2B8034,
];
