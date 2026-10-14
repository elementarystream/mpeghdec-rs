// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_trigFcts.h and libFDK/src/FDK_trigFcts.cpp.

//! Trigonometric functions, fixed-point fractional implementation.

use crate::common_fix::*;
use crate::tools_rom::SINE_TABLE_512;

/// `LD`: log2 of the length of the sine table [`SINE_TABLE_512`] (`SINETAB`).
const LD: i32 = 9;

/// `fixp_sin_cos_residual_inline(x, scale, &sine, &cosine)`: calculates the coarse lookup value of
/// sine and cosine and returns `(residual, sine, cosine)`, where `residual` is the delta x.
#[inline]
pub fn fixp_sin_cos_residual_inline(x: FixpDbl, scale: i32) -> (FixpDbl, FixpDbl, FixpDbl) {
    let shift = 31 - scale - LD - 1;
    let mut ssign: i32 = 1;
    let mut csign: i32 = 1;

    let mut residual = f_mult(x, fl2fxconst_dbl(1.0 / core::f64::consts::PI));
    let mut s = residual >> shift;

    residual &= (1 << shift) - 1;
    residual = f_mult(residual, fl2fxconst_dbl(core::f64::consts::PI / 4.0)) << 2;
    residual <<= scale;

    // Sine sign symmetry
    if s & ((1 << LD) << 1) != 0 {
        ssign = -ssign;
    }
    // Cosine sign symmetry
    if (s + (1 << LD)) & ((1 << LD) << 1) != 0 {
        csign = -csign;
    }

    s = f_abs(s);

    s &= ((1 << LD) << 1) - 1; // Modulo PI

    if s > (1 << LD) {
        s = ((1 << LD) << 1) - s;
    }

    let (sl, cl): (i32, i32);
    // Because of packed table
    if s > (1 << (LD - 1)) {
        // Cosine/Sine symmetry for angles greater than PI/4
        s = (1 << LD) - s;
        let tmp = SINE_TABLE_512[s as usize];
        sl = tmp.re as i32;
        cl = tmp.im as i32;
    } else {
        let tmp = SINE_TABLE_512[s as usize];
        sl = tmp.im as i32;
        cl = tmp.re as i32;
    }

    let sine = (sl * ssign) << (DFRACT_BITS - FRACT_BITS);
    let cosine = (cl * csign) << (DFRACT_BITS - FRACT_BITS);

    (residual, sine, cosine)
}

/// `inline_fixp_cos_sin(x1, x2, scale, out)`: calculate cosine and sine value each of 2 angles.
///
/// Returns `[cos(x1), sin(x1), cos(x2), sin(x2)]`; `scale` is the exponent of `x1` and `x2`.
#[inline]
pub fn inline_fixp_cos_sin(x1: FixpDbl, x2: FixpDbl, scale: i32) -> [FixpDbl; 4] {
    let (residual, sine, cosine) = fixp_sin_cos_residual_inline(x1, scale);
    let error0 = f_mult_div2(sine, residual);
    let error1 = f_mult_div2(cosine, residual);
    let out0 = cosine - (error0 << 1);
    let out1 = sine + (error1 << 1);

    let (residual, sine, cosine) = fixp_sin_cos_residual_inline(x2, scale);
    let error0 = f_mult_div2(sine, residual);
    let error1 = f_mult_div2(cosine, residual);
    let out2 = cosine - (error0 << 1);
    let out3 = sine + (error1 << 1);

    [out0, out1, out2, out3]
}

/// `fixp_cos(x, scale)`
pub fn fixp_cos(x: FixpDbl, scale: i32) -> FixpDbl {
    let (residual, sine, cosine) = fixp_sin_cos_residual_inline(x, scale);
    let error = f_mult(sine, residual);
    cosine - error
}

/// `fixp_sin(x, scale)`
pub fn fixp_sin(x: FixpDbl, scale: i32) -> FixpDbl {
    let (residual, sine, cosine) = fixp_sin_cos_residual_inline(x, scale);
    let error = f_mult(cosine, residual);
    sine + error
}

/// `fixp_cos_sin(x, scale, &cos, &sin)`: returns `(cos(x), sin(x))`.
pub fn fixp_cos_sin(x: FixpDbl, scale: i32) -> (FixpDbl, FixpDbl) {
    let (residual, sine, cosine) = fixp_sin_cos_residual_inline(x, scale);
    let error0 = f_mult(sine, residual);
    let error1 = f_mult(cosine, residual);
    (cosine - error0, sine + error1)
}
