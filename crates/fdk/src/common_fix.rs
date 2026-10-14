// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/common_fix.h, fixmul.h, fixmadd.h, cplx_mul.h, abs.h, clz.h,
// fixminmax.h and their arm/ variants.
//
// The arithmetic follows the AArch64 code paths of the reference implementation (Apple arm64,
// NEON disabled) or, with `crate::ARCH_X86`, those of the x86 build. The AArch64 paths differ from
// the generic C fallbacks in the least significant bits, e.g. `fixmul_DD` computes
// `(a * b) >> 31` instead of `((a * b) >> 32) << 1`.

//! Fixed-point data types and basic arithmetic.

use crate::digest::StateDigest;

/// Single precision fractional (Q1.15).
pub type FixpSgl = i16;
/// Double precision fractional (Q1.31).
pub type FixpDbl = i32;
/// Quad precision fractional (Q1.63).
pub type FixpQdl = i64;
/// PCM sample type (`INT_PCM`, `SAMPLE_BITS == 32`).
pub type IntPcm = i32;
/// Fixed-point equivalent type for PCM audio time domain data (`FIXP_PCM`).
pub type FixpPcm = FixpDbl;

/// Packed complex single precision value (`FIXP_SPK`).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct FixpSpk {
    pub re: FixpSgl,
    pub im: FixpSgl,
}

/// Packed complex double precision value (`FIXP_DPK`).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct FixpDpk {
    pub re: FixpDbl,
    pub im: FixpDbl,
}

impl FixpSpk {
    pub const fn new(re: FixpSgl, im: FixpSgl) -> Self {
        Self { re, im }
    }
}

impl FixpDpk {
    pub const fn new(re: FixpDbl, im: FixpDbl) -> Self {
        Self { re, im }
    }
}

pub const FRACT_BITS: i32 = 16;
pub const DFRACT_BITS: i32 = 32;
pub const ACCU_BITS: i32 = 40;
pub const SAMPLE_BITS: i32 = 32;
pub const WAV_BITS: i32 = 24;

pub const SGL_MASK: u32 = (1u32 << FRACT_BITS) - 1;
pub const MAX_SHIFT_SGL: i32 = FRACT_BITS - 1;
pub const MAX_SHIFT_DBL: i32 = DFRACT_BITS - 1;

pub const FRACT_FIX_SCALE: i64 = 1i64 << (FRACT_BITS - 1);
pub const DFRACT_FIX_SCALE: i64 = 1i64 << (DFRACT_BITS - 1);

pub const MAXVAL_SGL: i32 = 0x0000_7FFF;
pub const MINVAL_SGL: i32 = 0xFFFF_8000u32 as i32;
pub const MAXVAL_DBL: i32 = 0x7FFF_FFFF;
pub const MINVAL_DBL: i32 = 0x8000_0000u32 as i32;
pub const MAXVAL_PCM: i32 = MAXVAL_DBL;
pub const MINVAL_PCM: i32 = MINVAL_DBL;
pub const MAXVAL_FIXP_PCM: i32 = MAXVAL_DBL;
pub const MINVAL_FIXP_PCM: i32 = MINVAL_DBL;
pub const SAMPLE_MAX: IntPcm = ((1u32 << (SAMPLE_BITS - 1)) - 1) as i32;
pub const SAMPLE_MIN: IntPcm = !SAMPLE_MAX;

/// `FL2FXCONST_SGL`: compile-time conversion of a float constant to `FIXP_SGL`.
///
/// Note that C evaluates the argument in `float` precision when it is written with `f`-suffixed
/// literals; callers must reproduce that by computing the argument in `f32` and casting to `f64`.
#[inline]
pub const fn fl2fxconst_sgl(val: f64) -> FixpSgl {
    let s = FRACT_FIX_SCALE as f64;
    if val >= 0.0 {
        if val * s + 0.5 >= MAXVAL_SGL as f64 {
            MAXVAL_SGL as i16
        } else {
            (val * s + 0.5) as i16
        }
    } else if val * s - 0.5 <= MINVAL_SGL as f64 {
        MINVAL_SGL as i16
    } else {
        (val * s - 0.5) as i16
    }
}

/// `FL2FXCONST_DBL`: compile-time conversion of a float constant to `FIXP_DBL`.
///
/// See [`fl2fxconst_sgl`] regarding `float` precision of the argument.
#[inline]
pub const fn fl2fxconst_dbl(val: f64) -> FixpDbl {
    let s = DFRACT_FIX_SCALE as f64;
    if val >= 0.0 {
        if val * s + 0.5 >= MAXVAL_DBL as f64 {
            MAXVAL_DBL
        } else {
            (val * s + 0.5) as i32
        }
    } else if val * s - 0.5 <= MINVAL_DBL as f64 {
        MINVAL_DBL
    } else {
        (val * s - 0.5) as i32
    }
}

/// `FL2FX_SGL`: runtime float to `FIXP_SGL` conversion (no overflow check).
#[inline]
pub fn fl2fx_sgl(val: f32) -> FixpSgl {
    if val > 0.0 {
        (val * FRACT_FIX_SCALE as f32 + 0.5) as i16
    } else {
        (val * FRACT_FIX_SCALE as f32 - 0.5) as i16
    }
}

/// `FL2FX_DBL`: runtime float to `FIXP_DBL` conversion (no overflow check).
#[inline]
pub fn fl2fx_dbl(val: f32) -> FixpDbl {
    if val > 0.0 {
        (val * DFRACT_FIX_SCALE as f32 + 0.5) as i32
    } else {
        (val * DFRACT_FIX_SCALE as f32 - 0.5) as i32
    }
}

/// `FX_DBL2FXCONST_SGL`: rounded conversion of a `FIXP_DBL` to `FIXP_SGL` (usable in constants).
#[inline]
pub const fn fx_dbl2fxconst_sgl(val: FixpDbl) -> FixpSgl {
    if ((val >> (DFRACT_BITS - FRACT_BITS - 1)) + 1) > ((1i32 << FRACT_BITS) - 1) && val > 0 {
        ((1i32 << (FRACT_BITS - 1)) - 1) as i16
    } else {
        (((val >> (DFRACT_BITS - FRACT_BITS - 1)) + 1) >> 1) as i16
    }
}

/// `FX_SGL2FX_DBL`
#[inline(always)]
pub const fn fx_sgl2fx_dbl(val: FixpSgl) -> FixpDbl {
    (val as i32) << (DFRACT_BITS - FRACT_BITS)
}

/// `FX_DBL2FX_SGL` (truncating)
#[inline(always)]
pub const fn fx_dbl2fx_sgl(val: FixpDbl) -> FixpSgl {
    (val >> (DFRACT_BITS - FRACT_BITS)) as i16
}

/// `FX_DBL2FL`
#[inline]
pub fn fx_dbl2fl(val: FixpDbl) -> f32 {
    (val as f64 / DFRACT_FIX_SCALE as f64) as f32
}

/// `FX_DBL2DOUBLE`
#[inline]
pub fn fx_dbl2double(val: FixpDbl) -> f64 {
    val as f64 / DFRACT_FIX_SCALE as f64
}

/// `FX_PCM2FX_DBL`
#[inline(always)]
pub const fn fx_pcm2fx_dbl(val: FixpPcm) -> FixpDbl {
    val
}

/// `FX_DBL2FX_PCM`
#[inline(always)]
pub const fn fx_dbl2fx_pcm(val: FixpDbl) -> IntPcm {
    val
}

// ---------------------------------------------------------------------------------------------
// Multiplication (fixmul.h, arm/fixmul_arm.h, x86/fixmul_x86.h)
//
// The AArch64 build computes `fMult()` as `(a * b) >> 31` (`smull`, `asr #31`), the x86 build
// as `((a * b) >> 32) << 1` (`imul`, `shl`): the LSB of the x86 result is always 0. The `Div2`
// variants are the same on both. `crate::ARCH_X86` selects the variant.
// ---------------------------------------------------------------------------------------------

/// `fixmuldiv2_DD`: `(a * b) >> 32`
#[inline(always)]
pub const fn fixmuldiv2_dd(a: i32, b: i32) -> i32 {
    ((a as i64 * b as i64) >> 32) as i32
}

/// `fixmul_DD`: `(a * b) >> 31` (AArch64), `((a * b) >> 32) << 1` (x86)
#[inline(always)]
pub const fn fixmul_dd(a: i32, b: i32) -> i32 {
    if crate::ARCH_X86 {
        fixmuldiv2_dd(a, b) << 1
    } else {
        ((a as i64 * b as i64) >> 31) as i32
    }
}

/// `fixmuldiv2BitExact_DD`
#[inline(always)]
pub const fn fixmuldiv2_bit_exact_dd(a: i32, b: i32) -> i32 {
    fixmuldiv2_dd(a, b)
}

/// `fixmulBitExact_DD` (AArch64): `((a * b) >> 32) << 1`
#[inline(always)]
pub const fn fixmul_bit_exact_dd(a: i32, b: i32) -> i32 {
    fixmuldiv2_bit_exact_dd(a, b) << 1
}

/// `fixmuldiv2_SD` (AArch64): `(a * b) >> 16`
#[inline(always)]
pub const fn fixmuldiv2_sd(a: i16, b: i32) -> i32 {
    ((a as i64 * b as i64) >> 16) as i32
}

/// `fixmuldiv2_DS`
#[inline(always)]
pub const fn fixmuldiv2_ds(a: i32, b: i16) -> i32 {
    fixmuldiv2_sd(b, a)
}

/// `fixmul_SD`: `(a * b) >> 15` (AArch64), `((a * b) >> 16) << 1` (x86)
#[inline(always)]
pub const fn fixmul_sd(a: i16, b: i32) -> i32 {
    if crate::ARCH_X86 {
        fixmuldiv2_sd(a, b) << 1
    } else {
        ((a as i64 * b as i64) >> 15) as i32
    }
}

/// `fixmul_DS`
#[inline(always)]
pub const fn fixmul_ds(a: i32, b: i16) -> i32 {
    fixmul_sd(b, a)
}

/// `fixmuldiv2_SS`: `a * b`
#[inline(always)]
pub const fn fixmuldiv2_ss(a: i16, b: i16) -> i32 {
    a as i32 * b as i32
}

/// `fixmul_SS`: `(a * b) << 1`
#[inline(always)]
pub const fn fixmul_ss(a: i16, b: i16) -> i32 {
    (a as i32 * b as i32) << 1
}

/// `fixpow2div2_D`
#[inline(always)]
pub const fn fixpow2div2_d(a: i32) -> i32 {
    fixmuldiv2_dd(a, a)
}

/// `fixpow2_D`: `(a * a) >> 31` (AArch64), `((a * a) >> 32) << 1` (x86)
#[inline(always)]
pub const fn fixpow2_d(a: i32) -> i32 {
    if crate::ARCH_X86 {
        fixpow2div2_d(a) << 1
    } else {
        ((a as i64 * a as i64) >> 31) as i32
    }
}

/// `fixpow2div2_S`
#[inline(always)]
pub const fn fixpow2div2_s(a: i16) -> i32 {
    fixmuldiv2_ss(a, a)
}

/// `fixpow2_S`: saturates `0x8000 * 0x8000`
#[inline(always)]
pub const fn fixpow2_s(a: i16) -> i32 {
    let result = fixpow2div2_s(a) << 1;
    result ^ (result >> 31)
}

/// `fixmuldiv2_QQ`: `(a * b) >> 64` (AArch64 `smulh`, x86-64 128 bit product).
///
/// The 32 bit x86 build has no 128 bit type and uses the generic approximation of `fixmul.h`,
/// which omits the product of the low halves and shifts the mixed products individually.
#[inline(always)]
pub const fn fixmuldiv2_qq(a: i64, b: i64) -> i64 {
    if crate::ARCH_X86_32 {
        let (a_hi, a_lo) = (a >> 32, a as u32 as i64);
        let (b_hi, b_lo) = (b >> 32, b as u32 as i64);
        return a_hi
            .wrapping_mul(b_hi)
            .wrapping_add(a_hi.wrapping_mul(b_lo) >> 32)
            .wrapping_add(b_hi.wrapping_mul(a_lo) >> 32);
    }
    ((a as i128 * b as i128) >> 64) as i64
}

/// `fixmul_QQ` (AArch64)
#[inline(always)]
pub const fn fixmul_qq(a: i64, b: i64) -> i64 {
    fixmuldiv2_qq(a, b) << 1
}

/// `fixmuldiv2_QD`: `(a * b) >> 32` (AArch64: `smulh` with `b << 32`, x86: 128 bit product)
///
/// The 32 bit x86 build uses the generic approximation of `fixmul.h` (the high half times `b`
/// plus the shifted product of the low half and `b`).
#[inline(always)]
pub const fn fixmuldiv2_qd(a: i64, b: i32) -> i64 {
    if crate::ARCH_X86_32 {
        let (a_hi, a_lo, b_hi) = (a >> 32, a as u32 as i64, b as i64);
        return a_hi.wrapping_mul(b_hi).wrapping_add(b_hi.wrapping_mul(a_lo) >> 32);
    }
    fixmuldiv2_qq(a, (b as i64) << DFRACT_BITS)
}

/// `fixmul_QD`: `fixmuldiv2_QD(a, b) << 1`
#[inline(always)]
pub const fn fixmul_qd(a: i64, b: i32) -> i64 {
    fixmuldiv2_qd(a, b) << 1
}

/// Overloads of `fMult`, `fMultDiv2`, `fMultBitExact` and `fMultDiv2BitExact`.
///
/// The C++ code selects the implementation by the static operand types (`FIXP_DBL` = `i32`,
/// `FIXP_SGL` = `i16`, `FIXP_QDL` = `i64`); the trait reproduces that overload resolution.
pub trait FMult<B>: Copy {
    type Output;
    fn f_mult(self, b: B) -> Self::Output;
    fn f_mult_div2(self, b: B) -> Self::Output;
}

impl FMult<i32> for i32 {
    type Output = i32;
    #[inline(always)]
    fn f_mult(self, b: i32) -> i32 {
        fixmul_dd(self, b)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i32) -> i32 {
        fixmuldiv2_dd(self, b)
    }
}

impl FMult<i16> for i32 {
    type Output = i32;
    #[inline(always)]
    fn f_mult(self, b: i16) -> i32 {
        fixmul_ds(self, b)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i16) -> i32 {
        fixmuldiv2_ds(self, b)
    }
}

impl FMult<i32> for i16 {
    type Output = i32;
    #[inline(always)]
    fn f_mult(self, b: i32) -> i32 {
        fixmul_sd(self, b)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i32) -> i32 {
        fixmuldiv2_sd(self, b)
    }
}

impl FMult<i16> for i16 {
    type Output = i32;
    #[inline(always)]
    fn f_mult(self, b: i16) -> i32 {
        fixmul_ss(self, b)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i16) -> i32 {
        fixmuldiv2_ss(self, b)
    }
}

impl FMult<i64> for i64 {
    type Output = i64;
    #[inline(always)]
    fn f_mult(self, b: i64) -> i64 {
        fixmul_qq(self, b)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i64) -> i64 {
        fixmuldiv2_qq(self, b)
    }
}

impl FMult<i32> for i64 {
    type Output = i64;
    #[inline(always)]
    fn f_mult(self, b: i32) -> i64 {
        fixmul_qd(self, b)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i32) -> i64 {
        fixmuldiv2_qd(self, b)
    }
}

impl FMult<i64> for i32 {
    type Output = i64;
    #[inline(always)]
    fn f_mult(self, b: i64) -> i64 {
        fixmul_qd(b, self)
    }
    #[inline(always)]
    fn f_mult_div2(self, b: i64) -> i64 {
        fixmuldiv2_qd(b, self)
    }
}

/// `fMult(a, b)`
#[inline(always)]
pub fn f_mult<A: FMult<B>, B>(a: A, b: B) -> A::Output {
    a.f_mult(b)
}

/// `fMultDiv2(a, b)`
#[inline(always)]
pub fn f_mult_div2<A: FMult<B>, B>(a: A, b: B) -> A::Output {
    a.f_mult_div2(b)
}

/// Overloads of `fMultBitExact` / `fMultDiv2BitExact`.
pub trait FMultBitExact<B>: Copy {
    fn f_mult_bit_exact(self, b: B) -> i32;
    fn f_mult_div2_bit_exact(self, b: B) -> i32;
}

impl FMultBitExact<i32> for i32 {
    #[inline(always)]
    fn f_mult_bit_exact(self, b: i32) -> i32 {
        fixmul_bit_exact_dd(self, b)
    }
    #[inline(always)]
    fn f_mult_div2_bit_exact(self, b: i32) -> i32 {
        fixmuldiv2_bit_exact_dd(self, b)
    }
}

impl FMultBitExact<i16> for i32 {
    // AArch64: fixmulBitExact_DS == fixmul_DS, fixmuldiv2BitExact_DS == fixmuldiv2_DS
    #[inline(always)]
    fn f_mult_bit_exact(self, b: i16) -> i32 {
        fixmul_ds(self, b)
    }
    #[inline(always)]
    fn f_mult_div2_bit_exact(self, b: i16) -> i32 {
        fixmuldiv2_ds(self, b)
    }
}

impl FMultBitExact<i32> for i16 {
    #[inline(always)]
    fn f_mult_bit_exact(self, b: i32) -> i32 {
        fixmul_ds(b, self)
    }
    #[inline(always)]
    fn f_mult_div2_bit_exact(self, b: i32) -> i32 {
        fixmuldiv2_ds(b, self)
    }
}

/// `fMultBitExact(a, b)`
#[inline(always)]
pub fn f_mult_bit_exact<A: FMultBitExact<B>, B>(a: A, b: B) -> i32 {
    a.f_mult_bit_exact(b)
}

/// `fMultDiv2BitExact(a, b)`
#[inline(always)]
pub fn f_mult_div2_bit_exact<A: FMultBitExact<B>, B>(a: A, b: B) -> i32 {
    a.f_mult_div2_bit_exact(b)
}

/// Overloads of `fPow2` / `fPow2Div2`.
pub trait FPow2: Copy {
    fn f_pow2(self) -> i32;
    fn f_pow2_div2(self) -> i32;
}

impl FPow2 for i32 {
    #[inline(always)]
    fn f_pow2(self) -> i32 {
        fixpow2_d(self)
    }
    #[inline(always)]
    fn f_pow2_div2(self) -> i32 {
        fixpow2div2_d(self)
    }
}

impl FPow2 for i16 {
    #[inline(always)]
    fn f_pow2(self) -> i32 {
        fixpow2_s(self)
    }
    #[inline(always)]
    fn f_pow2_div2(self) -> i32 {
        fixpow2div2_s(self)
    }
}

/// `fPow2(a)`
#[inline(always)]
pub fn f_pow2<A: FPow2>(a: A) -> i32 {
    a.f_pow2()
}

/// `fPow2Div2(a)`
#[inline(always)]
pub fn f_pow2_div2<A: FPow2>(a: A) -> i32 {
    a.f_pow2_div2()
}

// ---------------------------------------------------------------------------------------------
// Multiply-accumulate (fixmadd.h, arm/fixmadd_arm.h)
//
// The `Div2` variants are the same on both builds. `fMultAdd()` / `fMultSub()` are
// `(2 * x + ((a * b) >> 31)` on AArch64 and `(x + ((a * b) >> 32)) << 1` on x86.
// ---------------------------------------------------------------------------------------------

/// `fixmadddiv2_DD` (AArch64): `x + ((a * b) >> 32)`
#[inline(always)]
pub const fn fixmadddiv2_dd(x: i32, a: i32, b: i32) -> i32 {
    (x as i64 + ((a as i64 * b as i64) >> 32)) as i32
}

/// `fixmadddiv2_SD` (AArch64): `x + ((a * b) >> 16)`
#[inline(always)]
pub const fn fixmadddiv2_sd(x: i32, a: i16, b: i32) -> i32 {
    (x as i64 + ((a as i64 * b as i64) >> 16)) as i32
}

/// `fixmadddiv2_DS`
#[inline(always)]
pub const fn fixmadddiv2_ds(x: i32, a: i32, b: i16) -> i32 {
    fixmadddiv2_sd(x, b, a)
}

/// `fixmadddiv2_SS`
#[inline(always)]
pub const fn fixmadddiv2_ss(x: i32, a: i16, b: i16) -> i32 {
    x.wrapping_add(fixmuldiv2_ss(a, b))
}

/// `fixmadddiv2_SS_Dual`
#[inline(always)]
pub const fn fixmadddiv2_ss_dual(x: i32, a: i32, b: i32) -> i32 {
    x.wrapping_add(fixmuldiv2_ss(a as i16, b as i16))
        .wrapping_add(fixmuldiv2_ss((a >> 16) as i16, (b >> 16) as i16))
}

/// `fixmadd_DD`: `(x << 1) + ((a * b) >> 31)` (AArch64), `(x + ((a * b) >> 32)) << 1` (x86)
#[inline(always)]
pub const fn fixmadd_dd(x: i32, a: i32, b: i32) -> i32 {
    if crate::ARCH_X86 {
        x.wrapping_add(fixmuldiv2_dd(a, b)) << 1
    } else {
        (((x as i64) << 1) + ((a as i64 * b as i64) >> 31)) as i32
    }
}

/// `fixmadd_SD`: `(x << 1) + ((a * b) >> 15)` (AArch64), `(x + ((a * b) >> 16)) << 1` (x86)
#[inline(always)]
pub const fn fixmadd_sd(x: i32, a: i16, b: i32) -> i32 {
    if crate::ARCH_X86 {
        x.wrapping_add(fixmuldiv2_sd(a, b)) << 1
    } else {
        (((x as i64) << 1) + ((a as i64 * b as i64) >> 15)) as i32
    }
}

/// `fixmadd_DS`
#[inline(always)]
pub const fn fixmadd_ds(x: i32, a: i32, b: i16) -> i32 {
    fixmadd_sd(x, b, a)
}

/// `fixmadd_SS`
#[inline(always)]
pub const fn fixmadd_ss(x: i32, a: i16, b: i16) -> i32 {
    x.wrapping_add(fixmuldiv2_ss(a, b)) << 1
}

/// `fixmsubdiv2_DD` (AArch64): `x - ((a * b) >> 32)`
#[inline(always)]
pub const fn fixmsubdiv2_dd(x: i32, a: i32, b: i32) -> i32 {
    (x as i64 - ((a as i64 * b as i64) >> 32)) as i32
}

/// `fixmsubdiv2_SD` (AArch64): `x - ((a * b) >> 16)`
#[inline(always)]
pub const fn fixmsubdiv2_sd(x: i32, a: i16, b: i32) -> i32 {
    (x as i64 - ((a as i64 * b as i64) >> 16)) as i32
}

/// `fixmsubdiv2_DS`
#[inline(always)]
pub const fn fixmsubdiv2_ds(x: i32, a: i32, b: i16) -> i32 {
    fixmsubdiv2_sd(x, b, a)
}

/// `fixmsubdiv2_SS`
#[inline(always)]
pub const fn fixmsubdiv2_ss(x: i32, a: i16, b: i16) -> i32 {
    x.wrapping_sub(fixmuldiv2_ss(a, b))
}

/// `fixmsub_DD`: `(x << 1) - ((a * b) >> 31)` (AArch64), `(x - ((a * b) >> 32)) << 1` (x86)
#[inline(always)]
pub const fn fixmsub_dd(x: i32, a: i32, b: i32) -> i32 {
    if crate::ARCH_X86 {
        x.wrapping_sub(fixmuldiv2_dd(a, b)) << 1
    } else {
        (((x as i64) << 1) - ((a as i64 * b as i64) >> 31)) as i32
    }
}

/// `fixmsub_SD`: `(x << 1) - ((a * b) >> 15)` (AArch64), `(x - ((a * b) >> 16)) << 1` (x86)
#[inline(always)]
pub const fn fixmsub_sd(x: i32, a: i16, b: i32) -> i32 {
    if crate::ARCH_X86 {
        x.wrapping_sub(fixmuldiv2_sd(a, b)) << 1
    } else {
        (((x as i64) << 1) - ((a as i64 * b as i64) >> 15)) as i32
    }
}

/// `fixmsub_DS`
#[inline(always)]
pub const fn fixmsub_ds(x: i32, a: i32, b: i16) -> i32 {
    fixmsub_sd(x, b, a)
}

/// `fixmsub_SS`
#[inline(always)]
pub const fn fixmsub_ss(x: i32, a: i16, b: i16) -> i32 {
    x.wrapping_sub(fixmuldiv2_ss(a, b)) << 1
}

/// Overloads of `fMultAdd`, `fMultAddDiv2`, `fMultSub`, `fMultSubDiv2` and their `BitExact`
/// variants, selected by the types of the factors `a` and `b`.
pub trait FMultAdd<B>: Copy {
    fn madd(x: i32, a: Self, b: B) -> i32;
    fn madd_div2(x: i32, a: Self, b: B) -> i32;
    fn msub(x: i32, a: Self, b: B) -> i32;
    fn msub_div2(x: i32, a: Self, b: B) -> i32;
    fn madd_div2_bit_exact(x: i32, a: Self, b: B) -> i32;
    fn msub_div2_bit_exact(x: i32, a: Self, b: B) -> i32;
}

impl FMultAdd<i32> for i32 {
    #[inline(always)]
    fn madd(x: i32, a: i32, b: i32) -> i32 {
        fixmadd_dd(x, a, b)
    }
    #[inline(always)]
    fn madd_div2(x: i32, a: i32, b: i32) -> i32 {
        fixmadddiv2_dd(x, a, b)
    }
    #[inline(always)]
    fn msub(x: i32, a: i32, b: i32) -> i32 {
        fixmsub_dd(x, a, b)
    }
    #[inline(always)]
    fn msub_div2(x: i32, a: i32, b: i32) -> i32 {
        fixmsubdiv2_dd(x, a, b)
    }
    #[inline(always)]
    fn madd_div2_bit_exact(x: i32, a: i32, b: i32) -> i32 {
        fixmadddiv2_dd(x, a, b)
    }
    #[inline(always)]
    fn msub_div2_bit_exact(x: i32, a: i32, b: i32) -> i32 {
        x.wrapping_sub(fixmuldiv2_bit_exact_dd(a, b))
    }
}

impl FMultAdd<i32> for i16 {
    #[inline(always)]
    fn madd(x: i32, a: i16, b: i32) -> i32 {
        fixmadd_sd(x, a, b)
    }
    #[inline(always)]
    fn madd_div2(x: i32, a: i16, b: i32) -> i32 {
        fixmadddiv2_sd(x, a, b)
    }
    #[inline(always)]
    fn msub(x: i32, a: i16, b: i32) -> i32 {
        fixmsub_sd(x, a, b)
    }
    #[inline(always)]
    fn msub_div2(x: i32, a: i16, b: i32) -> i32 {
        fixmsubdiv2_sd(x, a, b)
    }
    #[inline(always)]
    fn madd_div2_bit_exact(x: i32, a: i16, b: i32) -> i32 {
        fixmadddiv2_ds(x, b, a)
    }
    #[inline(always)]
    fn msub_div2_bit_exact(x: i32, a: i16, b: i32) -> i32 {
        x.wrapping_sub(fixmuldiv2_ds(b, a))
    }
}

impl FMultAdd<i16> for i32 {
    #[inline(always)]
    fn madd(x: i32, a: i32, b: i16) -> i32 {
        fixmadd_ds(x, a, b)
    }
    #[inline(always)]
    fn madd_div2(x: i32, a: i32, b: i16) -> i32 {
        fixmadddiv2_ds(x, a, b)
    }
    #[inline(always)]
    fn msub(x: i32, a: i32, b: i16) -> i32 {
        fixmsub_ds(x, a, b)
    }
    #[inline(always)]
    fn msub_div2(x: i32, a: i32, b: i16) -> i32 {
        fixmsubdiv2_ds(x, a, b)
    }
    #[inline(always)]
    fn madd_div2_bit_exact(x: i32, a: i32, b: i16) -> i32 {
        fixmadddiv2_ds(x, a, b)
    }
    #[inline(always)]
    fn msub_div2_bit_exact(x: i32, a: i32, b: i16) -> i32 {
        x.wrapping_sub(fixmuldiv2_ds(a, b))
    }
}

impl FMultAdd<i16> for i16 {
    #[inline(always)]
    fn madd(x: i32, a: i16, b: i16) -> i32 {
        fixmadd_ss(x, a, b)
    }
    #[inline(always)]
    fn madd_div2(x: i32, a: i16, b: i16) -> i32 {
        fixmadddiv2_ss(x, a, b)
    }
    #[inline(always)]
    fn msub(x: i32, a: i16, b: i16) -> i32 {
        fixmsub_ss(x, a, b)
    }
    #[inline(always)]
    fn msub_div2(x: i32, a: i16, b: i16) -> i32 {
        fixmsubdiv2_ss(x, a, b)
    }
    #[inline(always)]
    fn madd_div2_bit_exact(x: i32, a: i16, b: i16) -> i32 {
        fixmadddiv2_ss(x, a, b)
    }
    #[inline(always)]
    fn msub_div2_bit_exact(x: i32, a: i16, b: i16) -> i32 {
        fixmsubdiv2_ss(x, a, b)
    }
}

/// `fMultAdd(x, a, b)`: `2 * x + a * b`
#[inline(always)]
pub fn f_mult_add<A: FMultAdd<B>, B>(x: i32, a: A, b: B) -> i32 {
    A::madd(x, a, b)
}

/// `fMultAddDiv2(x, a, b)`: `x + a * b / 2`
#[inline(always)]
pub fn f_mult_add_div2<A: FMultAdd<B>, B>(x: i32, a: A, b: B) -> i32 {
    A::madd_div2(x, a, b)
}

/// `fMultSub(x, a, b)`: `2 * x - a * b`
#[inline(always)]
pub fn f_mult_sub<A: FMultAdd<B>, B>(x: i32, a: A, b: B) -> i32 {
    A::msub(x, a, b)
}

/// `fMultSubDiv2(x, a, b)`: `x - a * b / 2`
#[inline(always)]
pub fn f_mult_sub_div2<A: FMultAdd<B>, B>(x: i32, a: A, b: B) -> i32 {
    A::msub_div2(x, a, b)
}

/// `fMultAddDiv2BitExact(x, a, b)`
#[inline(always)]
pub fn f_mult_add_div2_bit_exact<A: FMultAdd<B>, B>(x: i32, a: A, b: B) -> i32 {
    A::madd_div2_bit_exact(x, a, b)
}

/// `fMultSubDiv2BitExact(x, a, b)`
#[inline(always)]
pub fn f_mult_sub_div2_bit_exact<A: FMultAdd<B>, B>(x: i32, a: A, b: B) -> i32 {
    A::msub_div2_bit_exact(x, a, b)
}

/// `fPow2AddDiv2(x, a)` for `FIXP_DBL a`
#[inline(always)]
pub const fn f_pow2_add_div2(x: i32, a: i32) -> i32 {
    fixmadddiv2_dd(x, a, a)
}

/// `fPow2AddDiv2(x, a)` for `FIXP_SGL a`
#[inline(always)]
pub const fn f_pow2_add_div2_sgl(x: i32, a: i16) -> i32 {
    x.wrapping_add(fixpow2div2_s(a))
}

/// `fPow2Add(x, a)` for `FIXP_DBL a`
#[inline(always)]
pub const fn f_pow2_add(x: i32, a: i32) -> i32 {
    x.wrapping_add(fixpow2_d(a))
}

/// `fPow2Add(x, a)` for `FIXP_SGL a`
#[inline(always)]
pub const fn f_pow2_add_sgl(x: i32, a: i16) -> i32 {
    x.wrapping_add(fixpow2_s(a))
}

// ---------------------------------------------------------------------------------------------
// Complex multiplication (cplx_mul.h, arm/cplx_mul_arm.h)
//
// The AArch64 build accumulates both products of a component in 64 bits and shifts the sum
// once (`smull`, `smsubl`/`smaddl`, `asr`); the x86 build (generic C code) adds or subtracts the
// individually shifted products `fMultDiv2()` / `fMult()` in 32 bits.
// ---------------------------------------------------------------------------------------------

/// Complex multiplications `c = a * b` with a `FIXP_DBL` input `a` and a `FIXP_DBL` or
/// `FIXP_SGL` coefficient `b`. All functions return `(c_re, c_im)`.
pub trait CplxCoef: Copy {
    fn cplx_mult_div2(a_re: i32, a_im: i32, b_re: Self, b_im: Self) -> (i32, i32);
    fn cplx_mult(a_re: i32, a_im: i32, b_re: Self, b_im: Self) -> (i32, i32);
    fn cplx_mult_add_div2(
        c_re: i32,
        c_im: i32,
        a_re: i32,
        a_im: i32,
        b_re: Self,
        b_im: Self,
    ) -> (i32, i32);
    fn cplx_mult_sub_div2(
        c_re: i32,
        c_im: i32,
        a_re: i32,
        a_im: i32,
        b_re: Self,
        b_im: Self,
    ) -> (i32, i32);
}

/// The x86 (generic C) complex product: `re = mul(a_re, b_re) - mul(a_im, b_im)`,
/// `im = mul(a_re, b_im) + mul(a_im, b_re)` in wrapping 32 bit arithmetic.
#[inline(always)]
fn cplx_prod_generic<B: Copy>(
    a_re: i32,
    a_im: i32,
    b_re: B,
    b_im: B,
    mul: impl Fn(i32, B) -> i32,
) -> (i32, i32) {
    (mul(a_re, b_re).wrapping_sub(mul(a_im, b_im)), mul(a_re, b_im).wrapping_add(mul(a_im, b_re)))
}

#[inline(always)]
const fn cplx_prod(a_re: i32, a_im: i32, b_re: i64, b_im: i64) -> (i64, i64) {
    let re = (a_re as i64).wrapping_mul(b_re).wrapping_sub((a_im as i64).wrapping_mul(b_im));
    let im = (a_re as i64).wrapping_mul(b_im).wrapping_add((a_im as i64).wrapping_mul(b_re));
    (re, im)
}

impl CplxCoef for i32 {
    #[inline(always)]
    fn cplx_mult_div2(a_re: i32, a_im: i32, b_re: i32, b_im: i32) -> (i32, i32) {
        if crate::ARCH_X86 {
            return cplx_prod_generic(a_re, a_im, b_re, b_im, fixmuldiv2_dd);
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((re >> 32) as i32, (im >> 32) as i32)
    }
    #[inline(always)]
    fn cplx_mult(a_re: i32, a_im: i32, b_re: i32, b_im: i32) -> (i32, i32) {
        if crate::ARCH_X86 {
            return cplx_prod_generic(a_re, a_im, b_re, b_im, fixmul_dd);
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((re >> 31) as i32, (im >> 31) as i32)
    }
    #[inline(always)]
    fn cplx_mult_add_div2(
        c_re: i32,
        c_im: i32,
        a_re: i32,
        a_im: i32,
        b_re: i32,
        b_im: i32,
    ) -> (i32, i32) {
        if crate::ARCH_X86 {
            let (re, im) = cplx_prod_generic(a_re, a_im, b_re, b_im, fixmuldiv2_dd);
            return (c_re.wrapping_add(re), c_im.wrapping_add(im));
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((c_re as i64).wrapping_add(re >> 32) as i32, (c_im as i64).wrapping_add(im >> 32) as i32)
    }
    #[inline(always)]
    fn cplx_mult_sub_div2(
        c_re: i32,
        c_im: i32,
        a_re: i32,
        a_im: i32,
        b_re: i32,
        b_im: i32,
    ) -> (i32, i32) {
        if crate::ARCH_X86 {
            let (re, im) = cplx_prod_generic(a_re, a_im, b_re, b_im, fixmuldiv2_dd);
            return (c_re.wrapping_sub(re), c_im.wrapping_sub(im));
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((c_re as i64).wrapping_sub(re >> 32) as i32, (c_im as i64).wrapping_sub(im >> 32) as i32)
    }
}

impl CplxCoef for i16 {
    #[inline(always)]
    fn cplx_mult_div2(a_re: i32, a_im: i32, b_re: i16, b_im: i16) -> (i32, i32) {
        if crate::ARCH_X86 {
            return cplx_prod_generic(a_re, a_im, b_re, b_im, fixmuldiv2_ds);
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((re >> 16) as i32, (im >> 16) as i32)
    }
    #[inline(always)]
    fn cplx_mult(a_re: i32, a_im: i32, b_re: i16, b_im: i16) -> (i32, i32) {
        if crate::ARCH_X86 {
            return cplx_prod_generic(a_re, a_im, b_re, b_im, fixmul_ds);
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((re >> 15) as i32, (im >> 15) as i32)
    }
    #[inline(always)]
    fn cplx_mult_add_div2(
        c_re: i32,
        c_im: i32,
        a_re: i32,
        a_im: i32,
        b_re: i16,
        b_im: i16,
    ) -> (i32, i32) {
        if crate::ARCH_X86 {
            let (re, im) = cplx_prod_generic(a_re, a_im, b_re, b_im, fixmuldiv2_ds);
            return (c_re.wrapping_add(re), c_im.wrapping_add(im));
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((c_re as i64).wrapping_add(re >> 16) as i32, (c_im as i64).wrapping_add(im >> 16) as i32)
    }
    #[inline(always)]
    fn cplx_mult_sub_div2(
        c_re: i32,
        c_im: i32,
        a_re: i32,
        a_im: i32,
        b_re: i16,
        b_im: i16,
    ) -> (i32, i32) {
        if crate::ARCH_X86 {
            let (re, im) = cplx_prod_generic(a_re, a_im, b_re, b_im, fixmuldiv2_ds);
            return (c_re.wrapping_sub(re), c_im.wrapping_sub(im));
        }
        let (re, im) = cplx_prod(a_re, a_im, b_re as i64, b_im as i64);
        ((c_re as i64).wrapping_sub(re >> 16) as i32, (c_im as i64).wrapping_sub(im >> 16) as i32)
    }
}

/// `cplxMultDiv2(&c_Re, &c_Im, a_Re, a_Im, b_Re, b_Im)` for `FIXP_DBL` inputs `a`.
/// For `FIXP_SPK`/`FIXP_DPK` twiddles pass `w.re, w.im`.
#[inline(always)]
pub fn cplx_mult_div2<B: CplxCoef>(a_re: i32, a_im: i32, b_re: B, b_im: B) -> (i32, i32) {
    B::cplx_mult_div2(a_re, a_im, b_re, b_im)
}

/// `cplxMult(&c_Re, &c_Im, a_Re, a_Im, b_Re, b_Im)` for `FIXP_DBL` inputs `a`.
#[inline(always)]
pub fn cplx_mult<B: CplxCoef>(a_re: i32, a_im: i32, b_re: B, b_im: B) -> (i32, i32) {
    B::cplx_mult(a_re, a_im, b_re, b_im)
}

/// `cplxMultAddDiv2(&c_Re, &c_Im, a_Re, a_Im, b_Re, b_Im)`: returns `c + a * b / 2`.
#[inline(always)]
pub fn cplx_mult_add_div2<B: CplxCoef>(
    c_re: i32,
    c_im: i32,
    a_re: i32,
    a_im: i32,
    b_re: B,
    b_im: B,
) -> (i32, i32) {
    B::cplx_mult_add_div2(c_re, c_im, a_re, a_im, b_re, b_im)
}

/// `cplxMultSubDiv2(&c_Re, &c_Im, a_Re, a_Im, b_Re, b_Im)`: returns `c - a * b / 2`.
#[inline(always)]
pub fn cplx_mult_sub_div2<B: CplxCoef>(
    c_re: i32,
    c_im: i32,
    a_re: i32,
    a_im: i32,
    b_re: B,
    b_im: B,
) -> (i32, i32) {
    B::cplx_mult_sub_div2(c_re, c_im, a_re, a_im, b_re, b_im)
}

/// `cplxMultDiv2(FIXP_DBL*, FIXP_DBL*, FIXP_SGL, FIXP_SGL, FIXP_SGL, FIXP_SGL)`
#[inline(always)]
pub const fn cplx_mult_div2_16x16(a_re: i16, a_im: i16, b_re: i16, b_im: i16) -> (i32, i32) {
    (
        fixmuldiv2_ss(a_re, b_re).wrapping_sub(fixmuldiv2_ss(a_im, b_im)),
        fixmuldiv2_ss(a_re, b_im).wrapping_add(fixmuldiv2_ss(a_im, b_re)),
    )
}

/// `cplxMultDiv2(FIXP_SGL*, FIXP_SGL*, FIXP_SGL, FIXP_SGL, FIXP_SGL, FIXP_SGL)`
#[inline(always)]
pub const fn cplx_mult_div2_16x16_sgl(a_re: i16, a_im: i16, b_re: i16, b_im: i16) -> (i16, i16) {
    let (re, im) = cplx_mult_div2_16x16(a_re, a_im, b_re, b_im);
    (fx_dbl2fx_sgl(re), fx_dbl2fx_sgl(im))
}

// Note: `cplxMult(FIXP_SGL*, FIXP_SGL*, FIXP_SGL, ...)` does not exist on AArch64 (the generic
// fallback is only compiled when FUNCTION_cplxMult_32x16X2 is undefined).

// ---------------------------------------------------------------------------------------------
// Absolute value (abs.h)
// ---------------------------------------------------------------------------------------------

/// `fixabs_D`: note that `fAbs(MINVAL_DBL) == MAXVAL_DBL`.
#[inline(always)]
pub const fn fixabs_d(x: i32) -> i32 {
    let y = (x as u32).wrapping_sub((x as u32) >> 31) as i32;
    y ^ (y >> 31)
}

/// `fixabs_S`
#[inline(always)]
pub const fn fixabs_s(x: i16) -> i16 {
    let y = (x as u16).wrapping_sub((x as u16) >> 15) as i16;
    y ^ (y >> 15)
}

/// `fixabs_Q`
#[inline(always)]
pub const fn fixabs_q(x: i64) -> i64 {
    let y = (x as u64).wrapping_sub((x as u64) >> 63) as i64;
    y ^ (y >> 63)
}

/// Overloads of `fAbs` / `fixp_abs`.
pub trait FAbs: Copy {
    fn f_abs(self) -> Self;
}

impl FAbs for i32 {
    #[inline(always)]
    fn f_abs(self) -> i32 {
        fixabs_d(self)
    }
}

impl FAbs for i16 {
    #[inline(always)]
    fn f_abs(self) -> i16 {
        fixabs_s(self)
    }
}

impl FAbs for i64 {
    #[inline(always)]
    fn f_abs(self) -> i64 {
        fixabs_q(self)
    }
}

/// `fAbs(x)` / `fixp_abs(x)`
#[inline(always)]
pub fn f_abs<T: FAbs>(x: T) -> T {
    x.f_abs()
}

// ---------------------------------------------------------------------------------------------
// Min / max (fixminmax.h)
// ---------------------------------------------------------------------------------------------

/// `fMin(a, b)` / `fixMin(a, b)` / `fixmin(a, b)`
#[inline(always)]
pub fn f_min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

/// `fMax(a, b)` / `fixMax(a, b)` / `fixmax(a, b)`
#[inline(always)]
pub fn f_max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else {
        b
    }
}

// ---------------------------------------------------------------------------------------------
// Count leading bits (clz.h, arm/clz_arm.h)
// ---------------------------------------------------------------------------------------------

/// `fixnormz_D` / `CntLeadingZeros`: number of leading zeros, 32 for 0.
#[inline(always)]
pub const fn fixnormz_d(value: i32) -> i32 {
    (value as u32).leading_zeros() as i32
}

/// `fixnorm_D` / `CountLeadingBits`: number of redundant sign bits, 0 for 0.
#[inline(always)]
pub const fn fixnorm_d(value: i32) -> i32 {
    if value == 0 {
        return 0;
    }
    let v = if value < 0 { !value } else { value };
    fixnormz_d(v) - 1
}

/// `fixnormz_S` (AArch64)
#[inline(always)]
pub const fn fixnormz_s(value: i16) -> i32 {
    let result = (value as i32) << 16;
    if result == 0 {
        16
    } else {
        fixnormz_d(result)
    }
}

/// `fixnorm_S` (AArch64)
#[inline(always)]
pub const fn fixnorm_s(value: i16) -> i32 {
    let lvalue = (value as i32) << 16;
    if lvalue == 0 {
        return 0;
    }
    let v = if lvalue < 0 { !lvalue } else { lvalue };
    fixnormz_d(v) - 1
}

/// Overloads of `fNorm` / `fNormz`.
pub trait FNorm: Copy {
    fn f_norm(self) -> i32;
    fn f_normz(self) -> i32;
}

impl FNorm for i32 {
    #[inline(always)]
    fn f_norm(self) -> i32 {
        fixnorm_d(self)
    }
    #[inline(always)]
    fn f_normz(self) -> i32 {
        fixnormz_d(self)
    }
}

impl FNorm for i16 {
    #[inline(always)]
    fn f_norm(self) -> i32 {
        fixnorm_s(self)
    }
    #[inline(always)]
    fn f_normz(self) -> i32 {
        fixnormz_s(self)
    }
}

impl FNorm for i64 {
    /// There is no `fNorm(INT64)` in C; provided for completeness only.
    #[inline(always)]
    fn f_norm(self) -> i32 {
        unimplemented!("fNorm(INT64) does not exist in the reference implementation")
    }
    #[inline(always)]
    fn f_normz(self) -> i32 {
        let mut clz = fixnormz_d((self >> 32) as i32);
        if clz == 32 {
            clz += fixnormz_d(self as i32);
        }
        clz
    }
}

/// `fNorm(x)`: redundant sign bits.
#[inline(always)]
pub fn f_norm<T: FNorm>(x: T) -> i32 {
    x.f_norm()
}

/// `fNormz(x)`: leading zeros.
#[inline(always)]
pub fn f_normz<T: FNorm>(x: T) -> i32 {
    x.f_normz()
}

/// `CntLeadingZeros(x)`
#[inline(always)]
pub const fn cnt_leading_zeros(x: i32) -> i32 {
    fixnormz_d(x)
}

/// `CountLeadingBits(x)`
#[inline(always)]
pub const fn count_leading_bits(x: i32) -> i32 {
    fixnorm_d(x)
}
