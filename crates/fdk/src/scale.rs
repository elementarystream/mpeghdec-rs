// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/scale.h, libFDK/include/arm/scale_arm.h and libFDK/src/scale.cpp.

//! Scaling operations.

use crate::common_fix::*;
use crate::fixpoint_math::f_add_saturate;

/// `scaleValue(FIXP_DBL value, INT scalefactor)`
///
/// Shift amounts of 32 or more are undefined in C; the AArch64 build masks them modulo 32, which
/// `wrapping_shl`/`wrapping_shr` reproduce.
#[inline(always)]
pub fn scale_value(value: FixpDbl, scalefactor: i32) -> FixpDbl {
    if scalefactor > 0 {
        value.wrapping_shl(scalefactor as u32)
    } else {
        value.wrapping_shr(scalefactor.wrapping_neg() as u32)
    }
}

/// `scaleValue(FIXP_SGL value, INT scalefactor)` (computed in `int`, truncated to `SHORT`)
#[inline(always)]
pub fn scale_value_sgl(value: FixpSgl, scalefactor: i32) -> FixpSgl {
    if scalefactor > 0 {
        ((value as i32) << scalefactor) as i16
    } else {
        ((value as i32) >> (-scalefactor)) as i16
    }
}

/// `scaleValueInPlace(FIXP_DBL* value, INT scalefactor)` (AArch64 variant)
#[inline(always)]
pub fn scale_value_in_place(value: &mut FixpDbl, scalefactor: i32) {
    if scalefactor >= 0 {
        *value <<= scalefactor;
    } else {
        *value >>= -scalefactor;
    }
}

/// `scaleValueSaturate(FIXP_DBL val, INT shl)` (AArch64 `sqshl` variant).
///
/// The shift amount is taken from the low byte of `shl` as a signed value, left shifts saturate
/// and right shifts truncate. The result is limited to `MINVAL_DBL + 1`.
///
/// The x86 build uses the generic variant: the shift amount is the whole `shl`, and right shifts
/// that shift out all significant bits give 0 (also for negative values).
#[inline(always)]
pub fn scale_value_saturate(val: FixpDbl, shl: i32) -> FixpDbl {
    if crate::ARCH_X86 {
        let headroom = fixnormz_d(val ^ (val >> 31));
        return if shl >= 0 {
            if headroom <= shl {
                if val > 0 {
                    MAXVAL_DBL
                } else {
                    MINVAL_DBL + 1
                }
            } else {
                f_max(val << shl, MINVAL_DBL + 1)
            }
        } else {
            let sh = shl.wrapping_neg();
            if DFRACT_BITS - headroom < sh {
                0
            } else {
                f_max(val >> sh, MINVAL_DBL + 1)
            }
        };
    }
    let sh = shl as i8 as i32;
    let result = if sh >= 0 {
        if val == 0 {
            0
        } else if sh >= 32 {
            if val > 0 {
                MAXVAL_DBL
            } else {
                MINVAL_DBL
            }
        } else {
            let w = (val as i64) << sh;
            if w > MAXVAL_DBL as i64 {
                MAXVAL_DBL
            } else if w < MINVAL_DBL as i64 {
                MINVAL_DBL
            } else {
                w as i32
            }
        }
    } else if -sh >= 32 {
        val >> 31
    } else {
        val >> (-sh)
    };
    f_max(MINVAL_DBL + 1, result)
}

/// `SATURATE_RIGHT_SHIFT(src, scale, dBits)` (AArch64 variant)
#[inline(always)]
pub const fn saturate_right_shift(src: i32, scale: i32, dbits: i32) -> i32 {
    let max = ((1u32 << (dbits - 1)) - 1) as i32;
    if ((src ^ (src >> (DFRACT_BITS - 1))) >> scale) > max {
        (src >> (DFRACT_BITS - 1)) ^ max
    } else {
        src >> scale
    }
}

/// `SATURATE_LEFT_SHIFT(src, scale, dBits)` (AArch64 variant)
#[inline(always)]
pub const fn saturate_left_shift(src: i32, scale: i32, dbits: i32) -> i32 {
    let max = ((1u32 << (dbits - 1)) - 1) as i32;
    if (src ^ (src >> (DFRACT_BITS - 1))) > (max >> scale) {
        (src >> (DFRACT_BITS - 1)) ^ max
    } else {
        src << scale
    }
}

/// `SATURATE_SHIFT(src, scale, dBits)`: right shift for positive `scale`, left shift otherwise.
#[inline(always)]
pub const fn saturate_shift(src: i32, scale: i32, dbits: i32) -> i32 {
    if scale < 0 {
        saturate_left_shift(src, -scale, dbits)
    } else {
        saturate_right_shift(src, scale, dbits)
    }
}

/// `SATURATE_LEFT_SHIFT_ALT(src, scale, dBits)`
#[inline(always)]
pub const fn saturate_left_shift_alt(src: i32, scale: i32, dbits: i32) -> i32 {
    let max = ((1u32 << (dbits - 1)) - 1) as i32;
    let max2 = ((1u32 << (dbits - 1)) - 2) as i32;
    if src > (max >> scale) {
        max
    } else if src <= !(max >> scale) {
        !max2
    } else {
        src << scale
    }
}

/// `SATURATE_RIGHT_SHIFT_ALT(src, scale, dBits)`
#[inline(always)]
pub const fn saturate_right_shift_alt(src: i32, scale: i32, dbits: i32) -> i32 {
    let max = ((1u32 << (dbits - 1)) - 1) as i32;
    let max2 = ((1u32 << (dbits - 1)) - 2) as i32;
    if (src >> scale) > max {
        max
    } else if (src >> scale) < !max2 {
        !max2
    } else {
        src >> scale
    }
}

/// `SATURATE_RIGHT_SHIFT_RND(src, scale, dBits)`
#[inline(always)]
pub const fn saturate_right_shift_rnd(src: i32, scale: i32, dbits: i32) -> i32 {
    if scale != 0 {
        saturate_right_shift(src.wrapping_add(1 << (scale - 1)), scale, dbits)
    } else {
        saturate_right_shift(src, scale, dbits)
    }
}

/// `SATURATE_INT_PCM_RIGHT_SHIFT(src, scale)`
#[inline(always)]
pub const fn saturate_int_pcm_right_shift(src: i32, scale: i32) -> i32 {
    saturate_right_shift(src, scale, SAMPLE_BITS)
}

/// `scaleValuesWithFactor(FIXP_DBL* vector, FIXP_DBL factor, INT len, INT scalefactor)`
pub fn scale_values_with_factor(vector: &mut [FixpDbl], factor: FixpDbl, scalefactor: i32) {
    let shift = f_min(scalefactor + 1, DFRACT_BITS - 1);
    if shift >= 0 {
        for v in vector.iter_mut() {
            *v = f_mult_div2(*v, factor) << shift;
        }
    } else {
        // The AArch64 variant (`arm/scale_arm.cpp`) does not limit right shifts (shifts >= 32
        // are masked modulo 32 like in the AArch64 build), the generic one of the x86 build
        // limits them to 31.
        let shift = if crate::ARCH_X86 { f_min(-shift, DFRACT_BITS - 1) } else { -shift };
        for v in vector.iter_mut() {
            *v = f_mult_div2(*v, factor).wrapping_shr(shift as u32);
        }
    }
}

/// `scaleValues(FIXP_SGL* vector, INT len, INT scalefactor)`
pub fn scale_values_sgl(vector: &mut [FixpSgl], scalefactor: i32) {
    if scalefactor == 0 {
        return;
    }
    if scalefactor > 0 {
        let s = f_min(scalefactor, FRACT_BITS - 1);
        for v in vector.iter_mut() {
            *v = ((*v as i32) << s) as i16;
        }
    } else {
        let s = f_min(-scalefactor, FRACT_BITS - 1);
        for v in vector.iter_mut() {
            *v >>= s;
        }
    }
}

crate::avx2_clone! {
/// `scaleValues(FIXP_DBL* vector, INT len, INT scalefactor)`
pub fn scale_values(vector: &mut [FixpDbl], scalefactor: i32) {
    if scalefactor == 0 {
        return;
    }
    if scalefactor > 0 {
        let s = f_min(scalefactor, DFRACT_BITS - 1);
        for v in vector.iter_mut() {
            *v <<= s;
        }
    } else {
        let s = f_min(-scalefactor, DFRACT_BITS - 1);
        for v in vector.iter_mut() {
            *v >>= s;
        }
    }
}
}

/// `scaleValues(FIXP_DBL* dst, const FIXP_DBL* src, INT len, INT scalefactor)`
///
/// `dst` and `src` must have the same length. For in-place operation use [`scale_values`].
pub fn scale_values_to(dst: &mut [FixpDbl], src: &[FixpDbl], scalefactor: i32) {
    debug_assert_eq!(dst.len(), src.len());
    if scalefactor == 0 {
        dst.copy_from_slice(src);
    } else if scalefactor > 0 {
        let s = f_min(scalefactor, DFRACT_BITS - 1);
        for (d, &x) in dst.iter_mut().zip(src) {
            *d = x << s;
        }
    } else {
        let s = f_min(-scalefactor, DFRACT_BITS - 1);
        for (d, &x) in dst.iter_mut().zip(src) {
            *d = x >> s;
        }
    }
}

/// `scaleValues(FIXP_SGL* dst, const FIXP_SGL* src, INT len, INT scalefactor)`
///
/// Note: the C version clamps the shift to `DFRACT_BITS - 1` and computes in `int`.
pub fn scale_values_sgl_to(dst: &mut [FixpSgl], src: &[FixpSgl], scalefactor: i32) {
    debug_assert_eq!(dst.len(), src.len());
    if scalefactor == 0 {
        dst.copy_from_slice(src);
    } else if scalefactor > 0 {
        let s = f_min(scalefactor, DFRACT_BITS - 1);
        for (d, &x) in dst.iter_mut().zip(src) {
            *d = ((x as i32) << s) as i16;
        }
    } else {
        let s = f_min(-scalefactor, DFRACT_BITS - 1);
        for (d, &x) in dst.iter_mut().zip(src) {
            *d = ((x as i32) >> s) as i16;
        }
    }
}

crate::avx2_clone! {
/// `scaleValuesSaturate(FIXP_DBL* vector, INT len, INT scalefactor)`
pub fn scale_values_saturate(vector: &mut [FixpDbl], scalefactor: i32) {
    if scalefactor == 0 {
        return;
    }
    let s = f_max(f_min(scalefactor, DFRACT_BITS - 1), -(DFRACT_BITS - 1));
    map_saturate(vector, s, |v| v);
}
}

/// `scaleValueSaturate(v, s)` for `0 <= s <= 31`: the saturated left shift, limited to
/// `MINVAL_DBL + 1` (branch free, so the loops using it vectorize).
#[inline(always)]
fn shl_saturate(v: FixpDbl, s: i32) -> FixpDbl {
    let r = if v > (MAXVAL_DBL >> s) {
        MAXVAL_DBL
    } else if v < (MINVAL_DBL >> s) {
        MINVAL_DBL
    } else {
        v << s
    };
    f_max(MINVAL_DBL + 1, r)
}

/// `scaleValueSaturate(v, -s)` for `1 <= s <= 31`: the arithmetic right shift (the result is never
/// below `MINVAL_DBL + 1`). In the x86 build (generic variant), the result is 0 if all
/// significant bits are shifted out, also for negative values (where the shift gives -1).
#[inline(always)]
fn shr_saturate(v: FixpDbl, s: i32) -> FixpDbl {
    if crate::ARCH_X86 && (v ^ (v >> 31)) >> (s - 1) == 0 {
        0
    } else {
        v >> s
    }
}

/// Applies `f(scaleValueSaturate(x, s))` to all values (`-31 <= s <= 31`).
#[inline(always)]
fn map_saturate(vector: &mut [FixpDbl], s: i32, f: impl Fn(FixpDbl) -> FixpDbl) {
    if s >= 0 {
        for v in vector.iter_mut() {
            *v = f(shl_saturate(*v, s));
        }
    } else {
        let s = -s;
        for v in vector.iter_mut() {
            *v = f(shr_saturate(*v, s));
        }
    }
}

/// Like [`map_saturate`] from `src` to `dst`.
#[inline(always)]
fn map_saturate_to<T>(dst: &mut [T], src: &[FixpDbl], s: i32, f: impl Fn(FixpDbl) -> T) {
    debug_assert_eq!(dst.len(), src.len());
    if s >= 0 {
        for (d, &x) in dst.iter_mut().zip(src) {
            *d = f(shl_saturate(x, s));
        }
    } else {
        let s = -s;
        for (d, &x) in dst.iter_mut().zip(src) {
            *d = f(shr_saturate(x, s));
        }
    }
}

crate::avx2_clone! {
/// `scaleValuesSaturate(FIXP_DBL* dst, const FIXP_DBL* src, INT len, INT scalefactor)`
pub fn scale_values_saturate_to(dst: &mut [FixpDbl], src: &[FixpDbl], scalefactor: i32) {
    debug_assert_eq!(dst.len(), src.len());
    if scalefactor == 0 {
        dst.copy_from_slice(src);
        return;
    }
    let s = f_max(f_min(scalefactor, DFRACT_BITS - 1), -(DFRACT_BITS - 1));
    map_saturate_to(dst, src, s, |v| v);
}
}

/// `scaleValuesSaturate(FIXP_SGL* dst, const FIXP_DBL* src, INT len, INT scalefactor)`
pub fn scale_values_saturate_dbl_to_sgl(dst: &mut [FixpSgl], src: &[FixpDbl], scalefactor: i32) {
    debug_assert_eq!(dst.len(), src.len());
    let s = f_max(f_min(scalefactor, DFRACT_BITS - 1), -(DFRACT_BITS - 1));
    map_saturate_to(dst, src, s, |v| fx_dbl2fx_sgl(f_add_saturate(v, 0x8000)));
}

/// `scaleValuesSaturate(FIXP_SGL* vector, INT len, INT scalefactor)`
pub fn scale_values_saturate_sgl(vector: &mut [FixpSgl], scalefactor: i32) {
    if scalefactor == 0 {
        return;
    }
    let s = f_max(f_min(scalefactor, DFRACT_BITS - 1), -(DFRACT_BITS - 1));
    for v in vector.iter_mut() {
        *v = fx_dbl2fx_sgl(scale_value_saturate(fx_sgl2fx_dbl(*v), s));
    }
}

/// `scaleValuesSaturate(FIXP_SGL* dst, const FIXP_SGL* src, INT len, INT scalefactor)`
pub fn scale_values_saturate_sgl_to(dst: &mut [FixpSgl], src: &[FixpSgl], scalefactor: i32) {
    debug_assert_eq!(dst.len(), src.len());
    if scalefactor == 0 {
        dst.copy_from_slice(src);
        return;
    }
    let s = f_max(f_min(scalefactor, DFRACT_BITS - 1), -(DFRACT_BITS - 1));
    for (d, &x) in dst.iter_mut().zip(src) {
        *d = fx_dbl2fx_sgl(scale_value_saturate(fx_sgl2fx_dbl(x), s));
    }
}

/// `getScalefactorShort(const SHORT* vector, INT len)`
pub fn get_scalefactor_short(vector: &[i16]) -> i32 {
    let mut max_val: i16 = 0;
    for &temp in vector {
        max_val |= temp ^ (temp >> (FRACT_BITS - 1));
    }
    f_max(0, fixnormz_d(max_val as i32) - 1 - (DFRACT_BITS - FRACT_BITS))
}

/// `getScalefactorPCM(const INT_PCM* vector, INT len, INT stride)`
///
/// `vector` must contain at least `(len - 1) * stride + 1` elements.
pub fn get_scalefactor_pcm(vector: &[IntPcm], len: usize, stride: usize) -> i32 {
    let mut max_val: IntPcm = 0;
    for i in 0..len {
        let temp = vector[i * stride];
        max_val |= temp ^ (temp >> (SAMPLE_BITS - 1));
    }
    f_max(0, fixnormz_d(max_val) - 1 - (DFRACT_BITS - SAMPLE_BITS))
}

crate::avx2_clone! {
/// `getScalefactor(const FIXP_DBL* vector, INT len)`: headroom of the vector.
pub fn get_scalefactor(vector: &[FixpDbl]) -> i32 {
    let mut max_val: FixpDbl = 0;
    for &temp in vector {
        max_val |= temp ^ (temp >> (DFRACT_BITS - 1));
    }
    f_max(0, fixnormz_d(max_val) - 1)
}
}

/// `getScalefactor(const FIXP_SGL* vector, INT len)`
pub fn get_scalefactor_sgl(vector: &[FixpSgl]) -> i32 {
    let mut max_val: i16 = 0;
    for &temp in vector {
        max_val |= temp ^ (temp >> (FRACT_BITS - 1));
    }
    f_max(0, fixnormz_s(max_val) - 1)
}
