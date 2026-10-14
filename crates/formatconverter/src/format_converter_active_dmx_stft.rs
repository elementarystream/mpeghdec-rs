// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/src/FDK_formatConverter_activeDmx_stft.h,
// libFormatConverter/src/FDK_formatConverter_activeDmx_stft.cpp and
// libFormatConverter/src/arm/FDK_formatConverter_activeDmx_stft_arm.cpp (which contains no code
// for the reference configuration: FUNCTION_activeDmxProcess_STFT, FUNCTION_immersiveModeInit
// and FUNCTION_activeDmxProcess_STFT_func1..5 are not defined, so the generic C code is ported).
//
// Notice of the C++ source:
//
// This software module was originally developed by
//
// Alexander Adami (AudioLabs) AND Fraunhofer IIS
//
// in the course of development of the ISO/IEC 23008-3 for reference purposes and its
// performance may not have been optimized. This software module is an implementation
// of one or more tools as specified by the ISO/IEC 23008-3 standard. ISO/IEC gives
// you a royalty-free, worldwide, non-exclusive, copyright license to copy, distribute,
// and make derivative works of this software module or modifications  thereof for use
// in implementations or products claiming conformance to the ISO/IEC 23008-3 standard
// and which satisfy any specified conformance criteria. Those intending to use this
// software module in products are advised that its use may infringe existing patents.
// ISO/IEC have no liability for use of this software module or modifications thereof.
// Copyright is not released for products that do not conform to the ISO/IEC 23008-3
// standard.
//
// Fraunhofer IIS retains full right to modify and use the code for its own purpose,
// assign or donate the code to a third party and to inhibit third parties from using
// the code for products that do not conform to MPEG-related ITU Recommendations and/or
// ISO/IEC International Standards.
//
// This copyright notice must be included in all copies or derivative works.
//
// Copyright (c) ISO/IEC 2015.

//! Active downmix in the STFT domain: downmix with energy preserving adaptive equalization per
//! ERB band (and the 3D rendering of the immersive mode with delayed top channels).

use mpeghdec_fdk::digest::StateDigest;

use crate::format_converter_constants_stft::*;
use crate::format_converter_data::format_converter_dmx_matrix_control;
use crate::format_converter_intern::*;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_add_saturate, inv_sqrt_norm2, schur_div};
use mpeghdec_fdk::scale::{scale_value_saturate, scale_values};

pub const MAX_ERB_BANDS: usize = 58;

/// `activeDownmixer`: state of the active downmix. The STFT buffers (`inputBufferStft`,
/// `outputBufferStft`, `prevInputBufferStft`) are those of the format converter instance and
/// are not duplicated here.
#[derive(StateDigest, Clone, Debug)]
pub struct ActiveDownmixer {
    pub target_ene_prev: Vec<Vec<FixpDbl>>,
    pub realized_ene_prev: Vec<Vec<FixpDbl>>,

    /// number input channels
    pub num_in_chans: u32,
    /// number output channels
    pub num_out_chans: u32,
    /// format: Q3.29
    pub fixp_aes: FixpDbl,
    pub target_ene_prev_exp: Vec<i32>,
    pub in_buf_stft_headroom_prev: [i8; STFT_ERB_BANDS],
    pub realized_sig_headroom_prev: [i8; STFT_ERB_BANDS],
    /// Scratch buffer: per bin target energy of each output channel (not in C).
    pub target_ene_bins: Vec<[FixpDbl; NUM_BINS]>,
}

/// `activeDmxStftInit(handle, numInChans, numOutChans, inputBufferStft, prevInputBufferStft,
/// outputBufferStft, aes)`: returns 0 on success.
pub fn active_dmx_stft_init(
    handle: &mut Option<Box<ActiveDownmixer>>,
    num_in_chans: u32,
    num_out_chans: u32,
    aes: i32,
) -> i32 {
    let status = 0;

    debug_assert!(num_in_chans <= MAX_CHANNELS);
    debug_assert!(num_out_chans <= MAX_CHANNELS);

    let mut h = Box::new(ActiveDownmixer {
        num_in_chans,
        num_out_chans,
        in_buf_stft_headroom_prev: [31; STFT_ERB_BANDS], /* start value */
        realized_sig_headroom_prev: [31; STFT_ERB_BANDS], /* start value */
        target_ene_prev: vec![vec![0; STFT_ERB_BANDS]; num_out_chans as usize],
        realized_ene_prev: vec![vec![0; STFT_ERB_BANDS]; num_out_chans as usize],
        target_ene_prev_exp: vec![0; STFT_ERB_BANDS],
        fixp_aes: 0,
        target_ene_bins: vec![[0; NUM_BINS]; num_out_chans as usize],
    });

    active_dmx_set_aes(aes, &mut h); /* 7 active downmix, 0 passive downmix */

    *handle = Some(h);
    status
}

/// `activeDmxClose_STFT(handle)`
pub fn active_dmx_close_stft(handle: &mut Option<Box<ActiveDownmixer>>) {
    *handle = None;
}

/* The per frequency band loops of activeDmxProcess_STFT() run over all bins at once with the
per ERB band values (headroom, downmix coefficients, exponents) expanded to per bin arrays. Every
bin is computed with exactly the arithmetic of the C++ code; only the per ERB band sums are
formed afterwards (wrapping additions, so the order does not matter). */

/// Number of complex STFT bins (the Nyquist value is stored in the imaginary part of bin 0).
const NUM_BINS: usize = STFT_LENGTH as usize / 2;

/// Calls `$m!(erb, first bin, end bin)` for the ERB bands with more than one bin (erbs 33..57,
/// `ERB_FREQ_IDX_256_58`); the erbs 0..32 have the single bin `erb`. Literal ranges let the
/// compiler generate straight-line code.
macro_rules! for_each_wide_erb {
    ($m:ident) => {
        $m!(33, 33, 36);
        $m!(34, 36, 39);
        $m!(35, 39, 43);
        $m!(36, 43, 47);
        $m!(37, 47, 51);
        $m!(38, 51, 55);
        $m!(39, 55, 60);
        $m!(40, 60, 65);
        $m!(41, 65, 70);
        $m!(42, 70, 77);
        $m!(43, 77, 84);
        $m!(44, 84, 91);
        $m!(45, 91, 98);
        $m!(46, 98, 106);
        $m!(47, 106, 115);
        $m!(48, 115, 125);
        $m!(49, 125, 136);
        $m!(50, 136, 148);
        $m!(51, 148, 160);
        $m!(52, 160, 174);
        $m!(53, 174, 189);
        $m!(54, 189, 205);
        $m!(55, 205, 222);
        $m!(56, 222, 241);
        $m!(57, 241, 256);
    };
}

/// Number of single bin ERB bands (erb == bin).
const NARROW_ERBS: usize = 33;

#[test]
#[allow(unused_assignments)]
fn wide_erb_ranges() {
    let mut start = NARROW_ERBS;
    let mut n = NARROW_ERBS;
    macro_rules! check {
        ($e:literal, $a:literal, $b:literal) => {
            assert_eq!(($e, $a, $b), (n, start, ERB_FREQ_IDX_256_58[$e] as usize));
            start = $b;
            n += 1;
        };
    }
    for_each_wide_erb!(check);
    assert_eq!(n, STFT_ERB_BANDS);
    for erb in 0..NARROW_ERBS {
        assert_eq!(ERB_FREQ_IDX_256_58[erb] as usize, erb + 1);
    }
}

/// Expands the per ERB band values `src` to the bins of the bands.
#[inline(always)]
fn expand_erb<T: Copy>(dst: &mut [T; NUM_BINS], src: &[T; STFT_ERB_BANDS]) {
    dst[..NARROW_ERBS].copy_from_slice(&src[..NARROW_ERBS]);
    macro_rules! fill {
        ($e:literal, $a:literal, $b:literal) => {
            dst[$a..$b].fill(src[$e]);
        };
    }
    for_each_wide_erb!(fill);
}

/// Sums the per bin values `vals` over the bins of each ERB band (wrapping additions).
#[inline(always)]
fn sum_erb(vals: &[FixpDbl; NUM_BINS]) -> [FixpDbl; STFT_ERB_BANDS] {
    let mut out = [0 as FixpDbl; STFT_ERB_BANDS];
    out[..NARROW_ERBS].copy_from_slice(&vals[..NARROW_ERBS]);
    macro_rules! sum {
        ($e:literal, $a:literal, $b:literal) => {
            out[$e] = vals[$a..$b].iter().fold(0 as FixpDbl, |a, &v| a.wrapping_add(v));
        };
    }
    for_each_wide_erb!(sum);
    out
}

/// `ERB_FREQ_IDX_256_58_EXP` per bin.
static ERB_EXP_BIN: [u8; NUM_BINS] = {
    let mut t = [0u8; NUM_BINS];
    let mut erb = 0;
    let mut bin = 0;
    while bin < NUM_BINS {
        if bin >= ERB_FREQ_IDX_256_58[erb] as usize {
            erb += 1;
        }
        t[bin] = ERB_FREQ_IDX_256_58_EXP[erb] as u8;
        bin += 1;
    }
    t
};

/// Downmix of one input channel into one output channel for all bins: adds
/// `(coef * (x << hr)) >> hr` to `realized` (saturated) and the energy contribution
/// `(fPow2Div2(re) + fPow2Div2(im)) >> exp` of every bin to `ene` (wrapping). `hr` and `exp` are
/// the shift amounts modulo 32 (the C++ shifts on AArch64 use the amount modulo 32).
fn dmx_bins(
    input: &[FixpDbl],
    realized: &mut [FixpDbl],
    coef: &[FixpDbl; NUM_BINS],
    hr: &[u32; NUM_BINS],
    ch_out_exp: i32,
    ene: &mut [FixpDbl; NUM_BINS],
) {
    let input: &[FixpDbl; 2 * NUM_BINS] = input[..2 * NUM_BINS].try_into().unwrap();
    let realized: &mut [FixpDbl; 2 * NUM_BINS] =
        (&mut realized[..2 * NUM_BINS]).try_into().unwrap();
    for b in 0..NUM_BINS {
        let h = hr[b];
        let d = coef[b];
        let tmp_re = f_mult(d, input[2 * b] << h);
        let tmp_im = f_mult(d, input[2 * b + 1] << h);
        realized[2 * b] = f_add_saturate(realized[2 * b], tmp_re >> h);
        realized[2 * b + 1] = f_add_saturate(realized[2 * b + 1], tmp_im >> h);
        let e = (ERB_EXP_BIN[b] as i32 + ch_out_exp) as u32 & 31;
        ene[b] = ene[b].wrapping_add(f_pow2_div2(tmp_im).wrapping_add(f_pow2_div2(tmp_re)) >> e);
    }
}

/// Realized energy `|fPow2Div2(re << hdr) + fPow2Div2(im << hdr)| >> exp` of every bin (the
/// absolute value is the one's complement `s ^ (s >> 31)` of the C++ code). The value of bin 0
/// is not used (the caller computes DC and Nyquist separately).
fn realized_energy_bins(
    sig: &[FixpDbl],
    hdr: &[u32; NUM_BINS],
    ch_out_exp: i32,
    ene: &mut [FixpDbl; NUM_BINS],
) {
    let sig: &[FixpDbl; 2 * NUM_BINS] = sig[..2 * NUM_BINS].try_into().unwrap();
    for b in 0..NUM_BINS {
        let h = hdr[b];
        let tmp_re = f_pow2_div2(sig[2 * b] << h);
        let tmp_im = f_pow2_div2(sig[2 * b + 1] << h);
        let s = tmp_re.wrapping_add(tmp_im);
        let e = (ERB_EXP_BIN[b] as i32 + ch_out_exp) as u32 & 31;
        ene[b] = (s ^ (s >> 31)) >> e;
    }
}

/// Applies the equalizer: `x = fMult(eq, x) << exp` for both parts of every bin (`exp` modulo
/// 32). The Nyquist value (imaginary part of bin 0) is handled by the caller.
fn apply_eq_bins(sig: &mut [FixpDbl], eq: &[FixpDbl; NUM_BINS], exp: &[u32; NUM_BINS]) {
    let sig: &mut [FixpDbl; 2 * NUM_BINS] = (&mut sig[..2 * NUM_BINS]).try_into().unwrap();
    for b in 0..NUM_BINS {
        sig[2 * b] = f_mult(eq[b], sig[2 * b]) << exp[b];
        sig[2 * b + 1] = f_mult(eq[b], sig[2 * b + 1]) << exp[b];
    }
}

/// `computeEQAndClip()` for all erbs: the EQ `eq[erb]` and its exponent modulo 32 (the shift
/// amount of the C++ shift) `sh[erb]`.
fn compute_eq_erbs(
    target_ene: &[FixpDbl],
    realized_ene: &[FixpDbl],
    diff_exp: &[i32; STFT_ERB_BANDS],
    eq_exp: i32,
    eq: &mut [FixpDbl; STFT_ERB_BANDS],
    sh: &mut [u32; STFT_ERB_BANDS],
) {
    let target_ene = &target_ene[..STFT_ERB_BANDS];
    let realized_ene = &realized_ene[..STFT_ERB_BANDS];
    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86", target_arch = "x86_64")))]
    let start = 0;
    // The AVX2 code reproduces the x86-64 arithmetic of the reference.
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    let start =
        if mpeghdec_fdk::ARCH_X86 && !mpeghdec_fdk::ARCH_X86_32 && mpeghdec_fdk::simd::avx2() {
            // SAFETY: AVX2 is available; all slices hold STFT_ERB_BANDS values; the chunks cover
            // 0..start.
            let start = STFT_ERB_BANDS / 4 * 4;
            for i in (0..start).step_by(4) {
                unsafe {
                    eq_avx2::compute4(
                        target_ene.as_ptr().add(i),
                        realized_ene.as_ptr().add(i),
                        diff_exp.as_ptr().add(i),
                        eq_exp,
                        eq.as_mut_ptr().add(i),
                        sh.as_mut_ptr().add(i),
                    )
                };
            }
            start
        } else {
            0
        };
    // The NEON code reproduces the AArch64 arithmetic of the reference.
    #[cfg(target_arch = "aarch64")]
    let start = if mpeghdec_fdk::ARCH_X86 {
        0
    } else {
        // SAFETY: all slices hold STFT_ERB_BANDS values; the chunks cover 0..start.
        let start = STFT_ERB_BANDS / 4 * 4;
        for i in (0..start).step_by(4) {
            unsafe {
                eq_neon::compute4(
                    target_ene.as_ptr().add(i),
                    realized_ene.as_ptr().add(i),
                    diff_exp.as_ptr().add(i),
                    eq_exp,
                    eq.as_mut_ptr().add(i),
                    sh.as_mut_ptr().add(i),
                )
            };
        }
        start
    };
    for erb in start..STFT_ERB_BANDS {
        let mut e = 0;
        eq[erb] =
            compute_eq_and_clip(target_ene[erb], realized_ene[erb], diff_exp[erb], eq_exp, &mut e);
        sh[erb] = e as u32 & 31;
    }
}

/// NEON variant of `computeEQAndClip()` for 4 erbs. Every step is the lane-wise equivalent of
/// the scalar code: `fNormz()` is `clz`, the float division and square root of `schur_div()`
/// and `invSqrtNorm2()` are IEEE operations in single precision like the scalar code
/// (`scvtf`/`fcvtzs` conversions with the same scaling), `fMult()` is computed in 64 bits and
/// `scaleValueSaturate(x, s)` is `max(sqshl(x, (SCHAR)s), MINVAL_DBL + 1)` (`sqshl` saturates
/// for any shift amount of a non-zero value and shifts right arithmetically for negative
/// amounts, like the C++ code).
#[cfg(target_arch = "aarch64")]
mod eq_neon {
    use super::*;
    use core::arch::aarch64::*;

    #[inline(always)]
    unsafe fn q31_to_f32(x: int32x4_t) -> float32x4_t {
        vmulq_n_f32(vcvtq_f32_s32(x), 1.0 / 2147483648.0)
    }

    #[inline(always)]
    unsafe fn f32_to_q31(x: float32x4_t) -> int32x4_t {
        vcvtq_s32_f32(vmulq_n_f32(x, 2147483648.0))
    }

    /// `fMult(a, b)`: `(a * b) >> 31`, truncated to 32 bits.
    #[inline(always)]
    unsafe fn f_mult4(a: int32x4_t, b: int32x4_t) -> int32x4_t {
        let lo = vmull_s32(vget_low_s32(a), vget_low_s32(b));
        let hi = vmull_high_s32(a, b);
        vshrn_high_n_s64::<31>(vshrn_n_s64::<31>(lo), hi)
    }

    pub unsafe fn compute4(
        target_ene: *const i32,
        realized_ene: *const i32,
        diff_exp: *const i32,
        eq_exp: i32,
        eq: *mut i32,
        sh: *mut u32,
    ) {
        let one = vdupq_n_s32(1);
        let zero = vdupq_n_s32(0);
        let eq_limit_max = vdupq_n_s32(fl2fxconst_dbl((2.511886432f32 / 4.0f32) as f64));
        let eq_limit_min = vdupq_n_s32(fl2fxconst_dbl((0.316227766f32 / 4.0f32) as f64));

        let realized = vmaxq_s32(vld1q_s32(realized_ene), one);
        let target = vmaxq_s32(vld1q_s32(target_ene), one);

        /* fDivNormNE(target, realized) */
        let norm_num = vmaxq_s32(vsubq_s32(vclzq_s32(target), one), zero);
        let norm_den = vmaxq_s32(vsubq_s32(vclzq_s32(realized), one), zero);
        let num = vshrq_n_s32::<1>(vshlq_s32(target, norm_num));
        let den = vshlq_s32(realized, norm_den);
        let mut e = vaddq_s32(vsubq_s32(norm_den, norm_num), one);
        let mut eq_val = f32_to_q31(vdivq_f32(q31_to_f32(num), q31_to_f32(den)));
        e = vaddq_s32(e, vld1q_s32(diff_exp));

        /* make exponent even */
        let odd = vandq_s32(e, one);
        eq_val = vshlq_s32(eq_val, vnegq_s32(odd));
        e = vaddq_s32(e, odd);

        /* invSqrtNorm2(eq_val) */
        let exponent = vsubq_s32(vclzq_s32(eq_val), one);
        let op = vshlq_s32(eq_val, vandq_s32(exponent, vdupq_n_s32(!1)));
        let mut shift = vaddq_s32(one, vshrq_n_s32::<1>(exponent));
        let mut inv = f32_to_q31(vdivq_f32(vdupq_n_f32(0.5), vsqrtq_f32(q31_to_f32(op))));
        let is_zero = vceqq_s32(eq_val, zero);
        shift = vbslq_s32(is_zero, vdupq_n_s32(16), shift);
        inv = vbslq_s32(is_zero, vdupq_n_s32(MAXVAL_DBL), inv);

        eq_val = f_mult4(eq_val, inv);
        /* the exponent is even: e / 2 == e >> 1 */
        let mut eq_exp_out = vaddq_s32(shift, vshrq_n_s32::<1>(e));

        /* Normalize mantissa to exponent 2 */
        let scale = vsubq_s32(eq_exp_out, vdupq_n_s32(2));
        eq_val = vmaxq_s32(vqshlq_s32(eq_val, scale), vdupq_n_s32(MINVAL_DBL + 1));
        eq_val = vminq_s32(eq_val, eq_limit_max);
        eq_val = vmaxq_s32(eq_val, eq_limit_min);
        /* CountLeadingBits() of a positive value */
        let scale = vsubq_s32(vclzq_s32(eq_val), one);
        eq_val = vshlq_s32(eq_val, scale);

        /* Add equalizer exponent */
        eq_exp_out = vaddq_s32(vsubq_s32(vdupq_n_s32(2), scale), vdupq_n_s32(eq_exp));
        vst1q_s32(eq, eq_val);
        vst1q_u32(sh, vandq_u32(vreinterpretq_u32_s32(eq_exp_out), vdupq_n_u32(31)));
    }
}

/// AVX2 variant of `computeEQAndClip()` for 4 erbs with the arithmetic of the x86-64 build.
/// Every step is the lane-wise equivalent of the scalar code for the values that occur (the
/// normalized dividend is less than the divisor, all intermediate values are positive):
/// - `fNormz()` of a positive value is `31 - e`, where `e` is the exponent of the (exact) double
///   conversion.
/// - `schur_div()` is the truncated quotient `(num << 31) / den` (< 2^31): the correctly rounded
///   double division of the exact operands is at most 1 too large, which the exact 64 bit
///   product of quotient and divisor reveals.
/// - `invSqrtNorm2()` uses the same IEEE operations (`(float)op * 0.5f`, double square root
///   and division, rounding to float); `frexpf()` and the scaling by `2^31` extract the
///   exponent and the mantissa of the normal result.
/// - `fMult()` is `((a * b) >> 32) << 1`; `scaleValueSaturate()` saturates if the headroom is
///   not larger than a left shift and shifts right logically (counts >= 32 give 0).
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
mod eq_avx2 {
    use super::*;
    #[cfg(target_arch = "x86")]
    use core::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    use core::arch::x86_64::*;

    /// The low (`HI == false`) or high 32 bits of the 64 bit lanes of `v`.
    #[inline(always)]
    unsafe fn pack<const HI: bool>(v: __m256i) -> __m128i {
        let idx = if HI {
            _mm256_setr_epi32(1, 3, 5, 7, 1, 3, 5, 7)
        } else {
            _mm256_setr_epi32(0, 2, 4, 6, 0, 2, 4, 6)
        };
        _mm256_castsi256_si128(_mm256_permutevar8x32_epi32(v, idx))
    }

    /// `fNormz(x)` (leading zeros) of positive values.
    #[inline(always)]
    unsafe fn clz_pos(x: __m128i) -> __m128i {
        let e = _mm256_srli_epi64::<52>(_mm256_castpd_si256(_mm256_cvtepi32_pd(x)));
        _mm_sub_epi32(_mm_set1_epi32(31 + 1023), pack::<false>(e))
    }

    #[target_feature(enable = "avx2")]
    pub unsafe fn compute4(
        target_ene: *const i32,
        realized_ene: *const i32,
        diff_exp: *const i32,
        eq_exp: i32,
        eq: *mut i32,
        sh: *mut u32,
    ) {
        let one = _mm_set1_epi32(1);
        let eq_limit_max = _mm_set1_epi32(fl2fxconst_dbl((2.511886432f32 / 4.0f32) as f64));
        let eq_limit_min = _mm_set1_epi32(fl2fxconst_dbl((0.316227766f32 / 4.0f32) as f64));

        let realized = _mm_max_epi32(_mm_loadu_si128(realized_ene as *const __m128i), one);
        let target = _mm_max_epi32(_mm_loadu_si128(target_ene as *const __m128i), one);

        /* fDivNormNE(target, realized) */
        let norm_num = _mm_sub_epi32(clz_pos(target), one);
        let norm_den = _mm_sub_epi32(clz_pos(realized), one);
        let num = _mm_srai_epi32::<1>(_mm_sllv_epi32(target, norm_num));
        let den = _mm_sllv_epi32(realized, norm_den);
        let mut e = _mm_add_epi32(_mm_sub_epi32(norm_den, norm_num), one);
        let q = _mm256_div_pd(
            _mm256_mul_pd(_mm256_cvtepi32_pd(num), _mm256_set1_pd(2147483648.0)),
            _mm256_cvtepi32_pd(den),
        );
        let t = _mm256_cvttpd_epi32(q);
        let prod = _mm256_mul_epu32(_mm256_cvtepu32_epi64(t), _mm256_cvtepu32_epi64(den));
        let over = _mm256_cmpgt_epi64(prod, _mm256_slli_epi64::<31>(_mm256_cvtepu32_epi64(num)));
        let mut eq_val = _mm_add_epi32(t, pack::<false>(over));
        e = _mm_add_epi32(e, _mm_loadu_si128(diff_exp as *const __m128i));

        /* make exponent even */
        let odd = _mm_and_si128(e, one);
        eq_val = _mm_srav_epi32(eq_val, odd);
        e = _mm_add_epi32(e, odd);

        /* invSqrtNorm2(eq_val) */
        let f = _mm_mul_ps(_mm_cvtepi32_ps(eq_val), _mm_set1_ps(0.5));
        let r =
            _mm256_cvtpd_ps(_mm256_div_pd(_mm256_set1_pd(1.0), _mm256_sqrt_pd(_mm256_cvtps_pd(f))));
        let rb = _mm_castps_si128(r);
        let shift = _mm_sub_epi32(_mm_srli_epi32::<23>(rb), _mm_set1_epi32(126 - 15));
        let inv = _mm_slli_epi32::<7>(_mm_or_si128(
            _mm_and_si128(rb, _mm_set1_epi32(0x7f_ffff)),
            _mm_set1_epi32(0x80_0000),
        ));

        let prod = _mm256_mul_epi32(_mm256_cvtepi32_epi64(eq_val), _mm256_cvtepi32_epi64(inv));
        eq_val = _mm_slli_epi32::<1>(pack::<true>(prod));
        /* the exponent is even: e / 2 == e >> 1 */
        let eq_exp_out = _mm_add_epi32(shift, _mm_srai_epi32::<1>(e));

        /* Normalize mantissa to exponent 2 */
        let scale = _mm_sub_epi32(eq_exp_out, _mm_set1_epi32(2));
        let sat = _mm_cmpgt_epi32(scale, _mm_sub_epi32(clz_pos(eq_val), one));
        let left = _mm_blendv_epi8(_mm_sllv_epi32(eq_val, scale), _mm_set1_epi32(MAXVAL_DBL), sat);
        let right = _mm_srlv_epi32(eq_val, _mm_sub_epi32(_mm_setzero_si128(), scale));
        eq_val = _mm_blendv_epi8(left, right, scale);
        eq_val = _mm_min_epi32(eq_val, eq_limit_max);
        eq_val = _mm_max_epi32(eq_val, eq_limit_min);
        /* CountLeadingBits() of a positive value */
        let scale = _mm_sub_epi32(clz_pos(eq_val), one);
        eq_val = _mm_sllv_epi32(eq_val, scale);

        /* Add equalizer exponent */
        let eq_exp_out =
            _mm_add_epi32(_mm_sub_epi32(_mm_set1_epi32(2), scale), _mm_set1_epi32(eq_exp));
        _mm_storeu_si128(eq as *mut __m128i, eq_val);
        _mm_storeu_si128(sh as *mut __m128i, _mm_and_si128(eq_exp_out, _mm_set1_epi32(31)));
    }
}

/// Shift amounts of the energy smoothing for the exponent difference `d` (-31..31):
/// `summand1 + (summand2 >> d)` for `d >= 0`, `(summand1 >> -d) + summand2` otherwise.
#[derive(Clone, Copy)]
struct SmoothShift {
    s1: u32,
    s2: u32,
}

impl SmoothShift {
    fn new(d: i32) -> Self {
        debug_assert!((-31..=31).contains(&d));
        SmoothShift { s1: (-d).max(0) as u32, s2: d.max(0) as u32 }
    }
}

/// Energy smoothing and buffer update of one output channel: `prev = fMult(alpha, ene) +
/// fMult(1 - alpha, prev)` with the exponent alignment `shift`, and the minimum of the headroom
/// `CntLeadingZeros(prev) - 1` in `min_hr`.
#[inline(always)]
fn smooth_energy(
    ene: &[FixpDbl; STFT_ERB_BANDS],
    prev: &mut [FixpDbl],
    shift: &[SmoothShift; STFT_ERB_BANDS],
    min_hr: &mut [i32; STFT_ERB_BANDS],
) {
    let prev: &mut [FixpDbl; STFT_ERB_BANDS] = (&mut prev[..STFT_ERB_BANDS]).try_into().unwrap();
    for erb in 0..STFT_ERB_BANDS {
        let summand1 = f_mult(ALPHA_AEQ, ene[erb]);
        let summand2 = f_mult(MAXVAL_DBL - ALPHA_AEQ, prev[erb]);
        let e = (summand1 >> shift[erb].s1).wrapping_add(summand2 >> shift[erb].s2);
        prev[erb] = e;
        min_hr[erb] = f_min(min_hr[erb], cnt_leading_zeros(e) - 1);
    }
}

/// `scaleValue(v[erb], sf[erb])` for all erbs (`sf` in -1..31).
#[inline(always)]
fn scale_erbs(v: &mut [FixpDbl], sf: &[i32; STFT_ERB_BANDS]) {
    let v: &mut [FixpDbl; STFT_ERB_BANDS] = (&mut v[..STFT_ERB_BANDS]).try_into().unwrap();
    for erb in 0..STFT_ERB_BANDS {
        debug_assert!((-31..=31).contains(&sf[erb]));
        v[erb] = (v[erb] << sf[erb].max(0)) >> (-sf[erb]).max(0);
    }
}

mpeghdec_fdk::avx2_clone! {
/// Minimum headroom per ERB band of an STFT buffer, see `update_stft_headroom()`.
fn update_stft_headroom_bins(hr: &mut [i32; STFT_ERB_BANDS], buf: &[FixpDbl]) {
    let buf: &[FixpDbl; 2 * NUM_BINS] = buf[..2 * NUM_BINS].try_into().unwrap();
    // Per bin: the bits of |re| and |im| (one's complement); fNormz(a | b) is
    // min(fNormz(a), fNormz(b)).
    let mut m = [0 as FixpDbl; NUM_BINS];
    for b in 0..NUM_BINS {
        let re = buf[2 * b];
        let im = buf[2 * b + 1];
        m[b] = (re ^ (re >> 31)) | (im ^ (im >> 31));
    }
    hr[0] = f_min(hr[0], get1x_scalefactor(&buf[0..]));
    hr[STFT_ERB_BANDS - 1] = f_min(hr[STFT_ERB_BANDS - 1], get1x_scalefactor(&buf[1..]));
    for erb in 1..33 {
        /* These erb use only 1 complex frequency band */
        hr[erb] = f_min(hr[erb], f_normz(m[erb]) - 1);
    }
    /* These erb use 3..19 frequency bands: getScalefactor() */
    macro_rules! wide {
        ($e:literal, $a:literal, $b:literal) => {
            let v = m[$a..$b].iter().fold(0, |a, &x| a | x);
            hr[$e] = f_min(hr[$e], f_max(0, f_normz(v) - 1));
        };
    }
    for_each_wide_erb!(wide);
}
}

/*
  Computation of return value EQ:                                   EQ_val           EQ_exp
  - division targetEne / realizedEne                                0.250 ... 1.0[  -30 ... +32
  - inverse square root of division result                          0.500 ... 1.0[  -15 ... +16
  - multiply invSqrt result with division result                    0.500 ... 1.0[  -15 ... +16
  - limitation of result into range apporximately [0.32 ... 2.51]   0.316 ... 2.51    2
  - multiply with AES in range 1/7...7/7                            0.045 ... 2.51    2
  - add (1.0-AES) in range 0/7... 6/7                               0.316 ... 2.51    2
  Due to the range limitation, we choose format Q29 for the following variables:
  - h->eqLimitMax
  - h->eqLimitMin
  - h->fixpAES
  - return value
*/

/// `fDivNormNE(L_num, L_denum, result_e)`: copy of fDivNorm but both `L_num > 0` and
/// `L_denum > 0`.
#[inline]
fn f_div_norm_ne(mut l_num: FixpDbl, mut l_denum: FixpDbl, result_e: &mut i32) -> FixpDbl {
    let norm_num = f_max(0, cnt_leading_zeros(l_num) - 1);
    let norm_den = f_max(0, cnt_leading_zeros(l_denum) - 1);
    l_num <<= norm_num;
    l_num >>= 1;
    l_denum <<= norm_den;
    *result_e = norm_den - norm_num + 1;

    schur_div(l_num, l_denum, FRACT_BITS)
}

/// `computeEQAndClip(targetEne, realizedEne, diffExp, eq_exp, EQ_exp)`
#[inline]
fn compute_eq_and_clip(
    mut target_ene: FixpDbl,
    mut realized_ene: FixpDbl,
    diff_exp: i32,
    eq_exp: i32,
    eq_exp_out: &mut i32,
) -> FixpDbl {
    /* pow(10.f, 8.f/20.f) with exp = 2 */
    let eq_limit_max: FixpDbl = fl2fxconst_dbl((2.511886432f32 / 4.0f32) as f64);
    /* pow(10.f, -10.f/20.f) with exp = 2  */
    let eq_limit_min: FixpDbl = fl2fxconst_dbl((0.316227766f32 / 4.0f32) as f64);
    let mut eq_exp_sqrt = 0;

    /* EQ = (float)sqrt(*targetEne/(h->epsilon + *realizedEne)); */
    realized_ene = f_max(realized_ene, 1);
    target_ene = f_max(target_ene, 1); /* zeros are small numbers in reality (float). */
    /* if we let it at 0 then the result is always eqLimitMin */
    /* set it to 1 results in a EQ of 1 when realizedEne is also 0 */
    /* that is a better deviation from float code than the eqLimitMin */
    let mut eq_val = f_div_norm_ne(target_ene, realized_ene, eq_exp_out);
    *eq_exp_out += diff_exp;

    let odd = *eq_exp_out & 1;
    eq_val >>= odd; /* make exponent even */
    *eq_exp_out += odd;

    eq_val = f_mult(eq_val, inv_sqrt_norm2(eq_val, &mut eq_exp_sqrt));
    *eq_exp_out = eq_exp_sqrt + (*eq_exp_out / 2);

    /* Normalize mantissa to exponent 2 */
    let mut scale = *eq_exp_out - 2;
    eq_val = scale_value_saturate(eq_val, scale);
    /*
      if (*EQ > eqLimitMax) *EQ = eqLimitMax;
      if (*EQ < eqLimitMin) *EQ = eqLimitMin;
    */
    eq_val = f_min(eq_val, eq_limit_max);
    eq_val = f_max(eq_val, eq_limit_min);
    scale = count_leading_bits(eq_val);
    eq_val <<= scale;

    /* Add equalizer exponent */
    *eq_exp_out = 2 - scale + eq_exp;
    eq_val
}

/// `get1xScalefactor(buf)`
#[inline]
fn get1x_scalefactor(buf: &[FixpDbl]) -> i32 {
    let buf0 = buf[0] ^ (buf[0] >> 31);
    f_normz(buf0) - 1 /* in range 0..31 */
}

/// Updates the per ERB band headroom `hr` with the headroom of the STFT buffer `buf`: DC
/// (`get1xScalefactor(buf)`) for erb 0, Nyquist (`get1xScalefactor(buf + 1)`) for erb 57,
/// `get2xScalefactor()` for the single bin erbs 1..32 and `getScalefactor()` over the bins of the
/// erbs 33..57.
fn update_stft_headroom(hr: &mut [i32; STFT_ERB_BANDS], buf: &[FixpDbl]) {
    update_stft_headroom_bins(hr, buf);
}

/// `activeDmxProcess_STFT(handle)`: active downmix of one STFT time slot from
/// `inputBufferStft` into `outputBufferStft` of the format converter instance.
pub fn active_dmx_process_stft(p: &mut IisFormatConverterInternal) {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    if mpeghdec_fdk::simd::avx2() {
        /// The same code compiled for AVX2 (vectorized with 256 bit registers).
        #[target_feature(enable = "avx2")]
        unsafe fn avx2(p: &mut IisFormatConverterInternal) {
            active_dmx_process_stft_impl(p)
        }
        // SAFETY: AVX2 is available.
        return unsafe { avx2(p) };
    }
    active_dmx_process_stft_impl(p)
}

#[inline(always)]
fn active_dmx_process_stft_impl(p: &mut IisFormatConverterInternal) {
    /*********************************************/
    /* Immersive mode initializer                */
    /*********************************************/
    {
        let params = p.fc_params.as_ref().unwrap();
        let immersive_mode = params.immersive_mode;
        /* rendering3DTypeFlag_internal is only updated internally. rendering3DTypeFlag is updated
         * every frame. */
        let rendering3_d_type_flag_internal = params.rendering3_d_type_flag_internal;

        /* Update control for immersive mode. The rendering3D flag could change under immersive
         * mode. Transitions from 3D to 2D (or the other way round) rendering are possible. */
        if immersive_mode != 0 && rendering3_d_type_flag_internal as u32 != p.rendering3_d_type_flag
        {
            /* Update control */
            format_converter_dmx_matrix_control(p);
        }
    }

    let IisFormatConverterInternal {
        fc_params,
        fc_state,
        input_buffer_stft,
        prev_input_buffer_stft,
        output_buffer_stft: realized_sig,
        eq_gains,
        top_in,
        mid_out,
        erb_is4_gvh_l: p_erb_is4_gvh_l,
        erb_is4_gvh_h: p_erb_is4_gvh_h,
        stft_headroom_prescaling,
        amount_of_added_dmx_matrices_and_equalizers: amount,
        ..
    } = p;
    let params = fc_params.as_deref().unwrap();
    let h = fc_state.as_mut().unwrap().handle_active_dmx_stft.as_deref_mut().unwrap();
    let stft_headroom_prescaling = *stft_headroom_prescaling;

    let mut in_buf_stft_headroom = [0i32; STFT_ERB_BANDS];
    let mut realized_sig_headroom = [0i32; STFT_ERB_BANDS];
    let mut min_headroom_target_ene = [0i32; STFT_ERB_BANDS];
    let mut min_headroom_realized_ene = [0i32; STFT_ERB_BANDS];
    let mut exp_diff = [0i32; STFT_ERB_BANDS];
    let mut exp_diff_realized = [0i32; STFT_ERB_BANDS];
    let mut realized_energy = [0 as FixpDbl; STFT_ERB_BANDS];

    let mut target_ene_arr = [0 as FixpDbl; IIS_FORMATCONVERTER_MAX_OUT_CH * STFT_ERB_BANDS];

    let num_out_chans = h.num_out_chans as usize;
    let num_in_chans = h.num_in_chans as usize;

    let immersive_mode = params.immersive_mode;
    let mode3_drendering = params.mode3_drendering;

    let dmx_matrix_l_fdk = params.dmx_sorted(params.dmx_matrix_l_fdk);
    let dmx_matrix_h_fdk = params.dmx_sorted(params.dmx_matrix_h_fdk);
    let eq_index_fdk = params.eq_index_sorted(params.eq_index_fdk);

    let ch_out_exp = &params.ch_out_exp;
    let ch_out_count = &params.ch_out_count;
    let dmx_iterations = params.dmx_iterations;

    /* - init inputStftHeadroom[0..57] with (SCHAR) 32
    - init const_1dot0_times58 with 1.0f with scale=1 */
    let const_1dot0_times58: [FixpEqH; STFT_ERB_BANDS] = [FIXP_EQ_H_FORMAT_1_DOT_0; STFT_ERB_BANDS];

    /* Equalizers init */
    let mut eq: [Option<&[FixpEqH]>; N_EQ] = [None; N_EQ];
    let eq_e;
    if *amount != 0 {
        eq_e = EQ_BITSTREAM_H_EXP;
        for i in 0..*amount as usize {
            eq[i] = eq_gains[i].as_deref();
        }
    } else {
        let pr = params.format_converter_params_internal.as_deref().unwrap();
        eq[0] = Some(&const_1dot0_times58);
        eq_e = EQ_H_EXP;
        for i in RULE_EQ1 as usize..=RULE_EQ5 as usize {
            eq[i] = Some(&pr.eq[i - 1]);
        }
        if immersive_mode != 0 {
            eq[6] = Some(&const_1dot0_times58);
            for i in RULE_EQVF as usize..=RULE_18_11 as usize {
                eq[i] = Some(&pr.eq[i - 1]);
            }
        }
    }

    /*
       - for all input buffer: ch=0..numInChans-1 &&
             all prev  buffer: ch=TFC..TFRA,  if Mode3Drendering
         - get headroom from buffer[0] for erb = 0  (DC)
         - get headroom from buffer[1] for erb = 57 (Nyquist)
         - for all erb=1..57
           - get buffer`s headroom over all fftBands of this erb
           - get minimum headroom in inBufStftHeadroom[erb]
    */
    for erb in 0..STFT_ERB_BANDS {
        in_buf_stft_headroom[erb] = 32;
    }

    for ch_in in 0..num_in_chans {
        update_stft_headroom(&mut in_buf_stft_headroom, &input_buffer_stft[ch_in]);
    }
    if mode3_drendering != 0 {
        for ch_in in TFC..=TFRA {
            update_stft_headroom(&mut in_buf_stft_headroom, &prev_input_buffer_stft[ch_in]);
        }
    }

    /*********************************************/
    /*               Clear Buffer                */
    /*********************************************/
    for ch_out in 0..num_out_chans {
        realized_sig[ch_out][..STFT_LENGTH as usize].fill(0);
    }
    target_ene_arr[..STFT_ERB_BANDS * num_out_chans].fill(0);

    let mut erb_is4_gvh_l = 58u32;
    let mut erb_is4_gvh_h = 58u32;

    if mode3_drendering != 0 {
        erb_is4_gvh_l = *p_erb_is4_gvh_l;
        erb_is4_gvh_h = *p_erb_is4_gvh_h;
    }

    /*********************************************/
    /* Compute target energy and realized signal */
    /*********************************************/

    /* input headroom per bin (shift amount modulo 32 like the C++ shifts) */
    let mut hr_bin = [0u32; NUM_BINS];
    expand_erb(&mut hr_bin, &in_buf_stft_headroom.map(|hr| hr as u32 & 31));
    let mut coef_bin = [0 as FixpDbl; NUM_BINS];
    /* per bin target energy of each output channel; summed per erb after the downmix (the C++
    code sums per erb for every input channel; the wrapping sums are the same) */
    let ene_acc = &mut h.target_ene_bins;
    for row in ene_acc[..num_out_chans].iter_mut() {
        row.fill(0);
    }

    let mut mtx_off = 0usize;
    for ch_in in 0..num_in_chans {
        let mut input_buffer_index: i32 = -1; /* means: inputBuffer = h->inputBufferStft[chIn]; */
        if mode3_drendering != 0 {
            let c = ch_in as i32;
            if c == top_in[TFC] {
                input_buffer_index = TFC as i32;
            } else if c == top_in[TFL] {
                input_buffer_index = TFL as i32;
            } else if c == top_in[TFR] {
                input_buffer_index = TFR as i32;
            } else if c == top_in[TFLA] {
                input_buffer_index = TFLA as i32;
            } else if c == top_in[TFRA] {
                input_buffer_index = TFRA as i32;
            }
        }
        for ch_out in 0..num_out_chans {
            let coef_l = dmx_matrix_l_fdk[mtx_off + ch_out];
            let coef_h = dmx_matrix_h_fdk[mtx_off + ch_out];
            if coef_l != 0 || coef_h != 0 {
                /* pointer to equalizer */
                let eq_ptr: &[FixpEqH] =
                    match eq.get(eq_index_fdk[mtx_off + ch_out] as usize).copied().flatten() {
                        Some(e) => e,
                        // C reads an uninitialized pointer here (not possible with the decoder's
                        // configurations); use the neutral equalizer.
                        None => {
                            mpeghdec_fdk::deviation::hit();
                            &const_1dot0_times58
                        }
                    };
                let input_buffer: &mut [FixpDbl] =
                    if input_buffer_index != -1 && mid_out[ch_out] as usize > SW {
                        &mut prev_input_buffer_stft[input_buffer_index as usize]
                    } else {
                        &mut input_buffer_stft[ch_in] /* undelayed input signal */
                    };

                let realized_sig_ch_out = &mut realized_sig[ch_out];
                let target_energy =
                    &mut target_ene_arr[STFT_ERB_BANDS * ch_out..STFT_ERB_BANDS * (ch_out + 1)];
                let sav_im0 = input_buffer[1];
                input_buffer[1] = 0;

                /* product of equalizer coefficient with dmx matrix coefficient per erb: the C++
                code runs over the erbs 0..erbIs4GVH_L with the coefficient L, then up to
                erbIs4GVH_H with H and up to 58 with L again (dmx_iterations == 3), or over all
                erbs with L (dmx_iterations == 1, erbIs4GVH_L == 58). */
                let mut coef_erb = [0 as FixpDbl; STFT_ERB_BANDS];
                let mut max_erb = erb_is4_gvh_l as usize;
                let mut erb = 0usize;
                let mut dmx_coeff_mtx = coef_l;
                for it in 0..dmx_iterations {
                    while erb < max_erb {
                        coef_erb[erb] = f_mult(dmx_coeff_mtx, eq_ptr[erb]);
                        erb += 1;
                    }
                    if it == 0 {
                        max_erb = erb_is4_gvh_h as usize;
                        dmx_coeff_mtx = coef_h;
                    } else if it == 1 {
                        max_erb = 58;
                        dmx_coeff_mtx = coef_l;
                    }
                }
                assert_eq!(erb, STFT_ERB_BANDS, "active downmix: not all erbs processed");
                expand_erb(&mut coef_bin, &coef_erb);
                dmx_bins(
                    input_buffer,
                    realized_sig_ch_out,
                    &coef_bin,
                    &hr_bin,
                    ch_out_exp[ch_out],
                    &mut ene_acc[ch_out],
                );

                erb -= 1; /* switch back to last erb */
                let dmx_coeff = coef_erb[erb];
                let target_exp = ERB_FREQ_IDX_256_58_EXP[erb] + ch_out_exp[ch_out];
                input_buffer[1] = sav_im0;
                let hr = in_buf_stft_headroom[erb] as u32;
                let tmp_im = f_mult(dmx_coeff, sav_im0.wrapping_shl(hr));
                realized_sig_ch_out[1] =
                    f_add_saturate(realized_sig_ch_out[1], tmp_im.wrapping_shr(hr));
                target_energy[erb] = target_energy[erb]
                    .wrapping_add(f_pow2_div2(tmp_im).wrapping_shr(target_exp as u32));
            } /* if ( ( dmxMatrixL_FDK[chOut] != (FIXP_DMX_H)0 ) || ( dmxMatrixH_FDK[chOut] !=
              (FIXP_DMX_H)0 ) ) */
        } /* for(chOut = 0; chOut < numOutChans; chOut++) */
        mtx_off += num_out_chans;
    } /* for(chIn = 0; chIn < numInChans; chIn++) */
    for ch_out in 0..num_out_chans {
        let sums = sum_erb(&ene_acc[ch_out]);
        let target_energy =
            &mut target_ene_arr[STFT_ERB_BANDS * ch_out..STFT_ERB_BANDS * (ch_out + 1)];
        for erb in 0..STFT_ERB_BANDS {
            target_energy[erb] = target_energy[erb].wrapping_add(sums[erb]);
        }
    }

    /* headroom of realized signal */
    /* initialize headroom values with maximum  */
    realized_sig_headroom.copy_from_slice(&in_buf_stft_headroom);

    for ch_out in 0..num_out_chans {
        /* first fft band (DC) and last fft band (Nyquist), all other fft bands */
        update_stft_headroom(&mut realized_sig_headroom, &realized_sig[ch_out]);
    }

    for erb in 0..STFT_ERB_BANDS {
        /* diff = target exponent - previous target exponent (does not depend on erb) */
        /* exponent = e_chOut + 1 - (2*inBufStftHeadroom) + erb_freq_idx_256_58_exp[i] + 20 + 2; */
        exp_diff[erb] = 2 * (-(in_buf_stft_headroom[erb] + stft_headroom_prescaling))
            + h.in_buf_stft_headroom_prev[erb] as i32;
        exp_diff[erb] = f_max(f_min(exp_diff[erb], 31), -31);
        exp_diff_realized[erb] = 2 * (-(realized_sig_headroom[erb] + stft_headroom_prescaling))
            + h.realized_sig_headroom_prev[erb] as i32;
        exp_diff_realized[erb] = f_max(f_min(exp_diff_realized[erb], 31), -31);
    }
    let shift_target = exp_diff.map(SmoothShift::new);
    let shift_realized = exp_diff_realized.map(SmoothShift::new);

    /*********************************************/
    /*            Update delay buffer            */
    /*********************************************/
    if immersive_mode != 0 {
        for ch_in in 0..num_in_chans {
            let c = ch_in as i32;
            for idx in [TFL, TFC, TFR, TFLA, TFRA] {
                if c == top_in[idx] {
                    let (src, dst) = (&input_buffer_stft[ch_in], &mut prev_input_buffer_stft[idx]);
                    dst[..STFT_LENGTH as usize].copy_from_slice(&src[..STFT_LENGTH as usize]);
                }
            }
        }
    }

    /* realizedEnergy shares the memory of minHeadroomTargetEne in C (not used concurrently). */

    for erb in 0..STFT_ERB_BANDS {
        min_headroom_realized_ene[erb] = 31; /* start value */
    }
    let mut hdr_bin = [0u32; NUM_BINS];
    expand_erb(&mut hdr_bin, &realized_sig_headroom.map(|hr| hr as u32 & 31));
    let mut ene_bin = [0 as FixpDbl; NUM_BINS];

    for ch_out in 0..num_out_chans {
        if ch_out_count[ch_out] != 0 {
            let sig = &realized_sig[ch_out];
            let exp_ch = ch_out_exp[ch_out];

            /* DC frequency, only real part */
            let realized_exp = ERB_FREQ_IDX_256_58_EXP[0] + exp_ch;
            let hdr = realized_sig_headroom[0] as u32;
            let tmp_re = f_pow2_div2(sig[0].wrapping_shl(hdr)); /* 0...0x40000000 */
            realized_energy[0] = tmp_re.wrapping_shr(realized_exp as u32);

            /* all other bins; the erbs 1..32 have one bin, where |nrg| >> exp is not negative
            and the final one's complement absolute value of the sum does not change it */
            realized_energy_bins(sig, &hdr_bin, exp_ch, &mut ene_bin);
            let sums = sum_erb(&ene_bin);
            for erb in 1..STFT_ERB_BANDS {
                realized_energy[erb] = sums[erb] ^ (sums[erb] >> 31);
            }

            /* Add Nyquist energy to last erb, valid variables: NyqRe, Hdr, realized_exp */
            let erb = STFT_ERB_BANDS - 1;
            let hdr = realized_sig_headroom[erb] as u32;
            let realized_exp = ERB_FREQ_IDX_256_58_EXP[erb] + exp_ch;
            let nyq_re = f_pow2_div2(sig[1].wrapping_shl(hdr)); /* 0...0x40000000 */
            realized_energy[erb] =
                realized_energy[erb].wrapping_add(nyq_re.wrapping_shr(realized_exp as u32));

            /*********************************************/
            /*    Energy smoothing and buffer update     */
            /*********************************************/
            smooth_energy(
                &realized_energy,
                &mut h.realized_ene_prev[ch_out],
                &shift_realized,
                &mut min_headroom_realized_ene,
            );
        } /* chOut_count[chOut] != 0 */
    } /* chOut */
    for erb in 0..STFT_ERB_BANDS {
        min_headroom_target_ene[erb] = 31;
    }

    for ch_out in 0..num_out_chans {
        if ch_out_count[ch_out] != 0 {
            let target_energy: &[FixpDbl; STFT_ERB_BANDS] = target_ene_arr
                [ch_out * STFT_ERB_BANDS..(ch_out + 1) * STFT_ERB_BANDS]
                .try_into()
                .unwrap();
            /*********************************************/
            /*    Energy smoothing and buffer update     */
            /*********************************************/
            smooth_energy(
                target_energy,
                &mut h.target_ene_prev[ch_out],
                &shift_target,
                &mut min_headroom_target_ene,
            );
        } /* chOut_count[chOut] != 0 */
    } /* chOut */

    for ch_out in 0..num_out_chans {
        if ch_out_count[ch_out] != 0 {
            scale_erbs(&mut h.target_ene_prev[ch_out], &min_headroom_target_ene);
            scale_erbs(&mut h.realized_ene_prev[ch_out], &min_headroom_realized_ene);
        }
    }
    for erb in 0..STFT_ERB_BANDS {
        if exp_diff[erb] >= 0 {
            h.in_buf_stft_headroom_prev[erb] =
                (2 * (in_buf_stft_headroom[erb] + stft_headroom_prescaling)
                    + min_headroom_target_ene[erb]) as i8;
        } else {
            h.in_buf_stft_headroom_prev[erb] =
                (h.in_buf_stft_headroom_prev[erb] as i32 + min_headroom_target_ene[erb]) as i8;
        }

        if exp_diff_realized[erb] >= 0 {
            h.realized_sig_headroom_prev[erb] =
                (2 * (realized_sig_headroom[erb] + stft_headroom_prescaling)
                    + min_headroom_realized_ene[erb]) as i8;
        } else {
            h.realized_sig_headroom_prev[erb] =
                (h.realized_sig_headroom_prev[erb] as i32 + min_headroom_realized_ene[erb]) as i8;
        }
    }

    if h.fixp_aes == 0 {
        for ch_out in 0..num_out_chans {
            if ch_out_count[ch_out] != 0 {
                /* Passive downmix, AES == 0. EQ = 1.0 */
                scale_values(
                    &mut realized_sig[ch_out]
                        [..2 * ERB_FREQ_IDX_256_58[STFT_ERB_BANDS - 1] as usize],
                    eq_e,
                );
            }
        }
    } else {
        let mut eq_bin = [0 as FixpDbl; NUM_BINS];
        let mut sh_bin = [0u32; NUM_BINS];
        let diff_exp: [i32; STFT_ERB_BANDS] = core::array::from_fn(|erb| {
            h.realized_sig_headroom_prev[erb] as i32 - h.in_buf_stft_headroom_prev[erb] as i32
        });
        for ch_out in 0..num_out_chans {
            if ch_out_count[ch_out] != 0 {
                /*********************************************/
                /*          Compute and apply EQ             */
                /*********************************************/
                let target_ene_ch_out = &h.target_ene_prev[ch_out];
                let realized_ene_ch_out = &h.realized_ene_prev[ch_out];
                let realized_sig_ch_out = &mut realized_sig[ch_out];

                /* EQ per erb (the shift amount modulo 32 like the C++ shift) */
                let mut eq_erb = [0 as FixpDbl; STFT_ERB_BANDS];
                let mut sh_erb = [0u32; STFT_ERB_BANDS];
                compute_eq_erbs(
                    target_ene_ch_out,
                    realized_ene_ch_out,
                    &diff_exp,
                    eq_e,
                    &mut eq_erb,
                    &mut sh_erb,
                );
                expand_erb(&mut eq_bin, &eq_erb);
                expand_erb(&mut sh_bin, &sh_erb);
                /* the Nyquist value uses the EQ of the last erb */
                let nyq = realized_sig_ch_out[1];
                apply_eq_bins(realized_sig_ch_out, &eq_bin, &sh_bin);
                let last = STFT_ERB_BANDS - 1;
                realized_sig_ch_out[1] = f_mult(eq_erb[last], nyq) << sh_erb[last];
            } /* if ( chOut_count[chOut] != 0 ) */
        } /* for(chOut = 0; chOut < numOutChans; chOut++) */
    }
}

/// `activeDmxSetAES(AES, handle)`: `fixpAES = AES/7.0f` in Q2.29.
pub fn active_dmx_set_aes(aes: i32, h: &mut ActiveDownmixer) {
    debug_assert!(aes <= 7);
    h.fixp_aes = (aes.wrapping_mul(0x0924_9249).wrapping_add(1)) >> 1; /* fixpAES = AES/7.0f   change format from INT to Q2.29 */
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The vectorized EQ computation against the scalar `compute_eq_and_clip()`.
    #[test]
    fn compute_eq_erbs_matches_scalar() {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        let edge = [0, 1, -1, 2, 3, i32::MAX, i32::MIN, i32::MIN + 1, 0x4000_0000, 0x3fff_ffff];
        for iter in 0..200_000 {
            let mut val = || {
                let r = rnd();
                match r % 4 {
                    0 => edge[(r >> 8) as usize % edge.len()],
                    1 => ((r >> 8) as i32) >> ((r >> 40) % 32),
                    _ => (r >> 8) as i32,
                }
            };
            let target: [i32; STFT_ERB_BANDS] = core::array::from_fn(|_| val());
            let realized: [i32; STFT_ERB_BANDS] = core::array::from_fn(|_| val());
            let diff_exp: [i32; STFT_ERB_BANDS] =
                core::array::from_fn(|_| (rnd() % 601) as i32 - 300);
            let eq_exp = [EQ_H_EXP, EQ_BITSTREAM_H_EXP][iter % 2];
            let (mut eq, mut sh) = ([0; STFT_ERB_BANDS], [0; STFT_ERB_BANDS]);
            compute_eq_erbs(&target, &realized, &diff_exp, eq_exp, &mut eq, &mut sh);
            for erb in 0..STFT_ERB_BANDS {
                let mut e = 0;
                let v =
                    compute_eq_and_clip(target[erb], realized[erb], diff_exp[erb], eq_exp, &mut e);
                assert_eq!(
                    (eq[erb], sh[erb]),
                    (v, e as u32 & 31),
                    "{} {} {}",
                    target[erb],
                    realized[erb],
                    diff_exp[erb]
                );
            }
        }
    }
}
