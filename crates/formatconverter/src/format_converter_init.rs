// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_formatConverter_init.h and
// libFormatConverter/src/FDK_formatConverter_init.cpp.
// The floating point code of the C++ file is only contained in comments; everything is fixed
// point.
//
// Notice of the C++ source:
//
// This software module was originally developed by
//
// Fraunhofer IIS
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
// Fraunhofer IIS retains full right to modify and use the code for
// their own purpose, assign or donate the code to a third party and to inhibit third
// parties from using the code for products that do not conform to MPEG-related ITU
// Recommendations and/or ISO/IEC International Standards.
//
// This copyright notice must be included in all copies or derivative works.
//
// Copyright (c) ISO/IEC 2013.

//! Format converter initialization: downmix rules, generation of the downmix gains (amplitude
//! panning), static equalizers (peak filter cascades) and the spatial elevation rendering
//! parameters of the immersive mode.

use crate::format_converter_constants::*;
use crate::format_converter_constants_stft::STFT_ERB_BANDS;
use crate::format_converter_intern::*;
use crate::format_converter_lib::MAX_NUM_DMX_RULES;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{
    f_add_norm, f_div_norm_signed, f_pow, inv_fixp_e, inv_sqrt_norm2, sqrt_fixp,
};
use mpeghdec_fdk::scale::{scale_value, scale_value_sgl};
use mpeghdec_fdk::trig_fcts::{fixp_cos, fixp_sin};

include!("format_converter_init_tables.rs");

/// `converter_status_t`: error codes.
pub type ConverterStatus = i32;
/// success
pub const FORMAT_CONVERTER_STATUS_OK: ConverterStatus = 0;
/// generic error
pub const FORMAT_CONVERTER_STATUS_FAILED: ConverterStatus = -1;
/// missing downmix rule
pub const FORMAT_CONVERTER_STATUS_MISSING_RULE: ConverterStatus = -2;
/// invalid input format index
pub const FORMAT_CONVERTER_STATUS_INFORMAT: ConverterStatus = -3;
/// invalid output format index
pub const FORMAT_CONVERTER_STATUS_OUTFORMAT: ConverterStatus = -4;
/// invalid sampling frequency
pub const FORMAT_CONVERTER_STATUS_SFREQ: ConverterStatus = -5;
/// invalid block length
pub const FORMAT_CONVERTER_STATUS_BLOCKLENGTH: ConverterStatus = -6;
/// out-of-range trim values
pub const FORMAT_CONVERTER_STATUS_TRIM: ConverterStatus = -7;
/// randomization out-of-range
pub const FORMAT_CONVERTER_STATUS_RANDOMIZATION: ConverterStatus = -8;
/// bands out-of-range
pub const FORMAT_CONVERTER_STATUS_BANDS: ConverterStatus = -9;

/* hard coded parameters */

/// time constant for adaptive eq smoothing \[ms\]
pub const TC_AEQ_MS: f32 = 60.0;
/// adaptive equatization gain upper limit \[dB\]
pub const AEQ_MAX_LIMIT_DB: f32 = 8.0;
/// adaptive equatization gain upper limit \[dB\]
pub const AEQ_MIN_LIMIT_DB: f32 = -10.0;
/// speed of sound \[m/s\]
pub const SOS_M_S: f32 = 340.0;

/* parameter range limitations */
/// minimum sampling frequency
pub const SFREQ_HZ_MIN: i32 = 8000;
/// maximum sampling frequency
pub const SFREQ_HZ_MAX: i32 = 384000;
/// minimum number of bands
pub const NBANDS_MIN: u32 = 10;
/// maximum number of bands
pub const NBANDS_MAX: u32 = 2048;
/// maximum azi deviation \[deg\]
pub const RANDOMIZATION_AZI_MAX: i32 = 35;
/// maximum ele deviation \[deg\]
pub const RANDOMIZATION_ELE_MAX: i32 = 55;

/// Size of the temporary in-out conversion vectors. The first pass of `converter_init()` checks
/// against `IN_OUT_N` (and may write one entry beyond); the second pass of the immersive mode
/// does not check. The spare entries keep the port from panicking where C would write into the
/// adjacent vectors (only possible for setups the decoder never creates).
const IN_OUT_TMP_N: usize = 2 * IN_OUT_N + 2;

/// `find_channel(ch, channels, nchan)`
fn find_channel(ch: i32, channels: &[AfcFormatConverterChannelId], nchan: i32) -> i32 {
    for i in 0..nchan.max(0) as usize {
        if channels[i] == ch {
            return i as i32;
        }
    }
    -1
}

const DBL_16_DIV_20: FixpDbl = 0x6666_6666;
const DBL_10_DIV_16: FixpDbl = 0x5000_0000;

/// `peak_filter(f, f_e, q, q_e, g, g_e, G, G_e, b, b_e, result_e)`: magnitude response of a
/// peak filter (peak frequency `f` \[Hz\], Q factor `q`, peak gain `g` \[dB\], global gain
/// `G` \[dB\]) at the band center frequency `b` \[Hz\]. The exponent of the result is added to `*result_e`.
pub fn peak_filter(
    f: FixpDbl,
    f_e: i32,
    q: FixpDbl,
    q_e: i32,
    g: FixpDbl,
    g_e: i32,
    big_g: FixpDbl,
    big_g_e: i32,
    b: FixpDbl,
    b_e: i32,
    result_e: &mut i32,
) -> FixpDbl {
    let (mut b, mut b_e, mut q, mut q_e) = (b, b_e, q, q_e);
    let mut v0_e = 0;
    let mut tmp_v_e = 0;
    let mut tmp_q_e;
    let mut tmp_sum_e = 0;
    let mut tmp_g_v_e = 0;
    let mut tmp_g_1_e = 0;
    let mut gain_e = 0;

    let norm_b = count_leading_bits(b);
    b = b.wrapping_shl(norm_b as u32);
    b_e -= norm_b;

    let mut norm_q = count_leading_bits(q);
    q = q.wrapping_shl(norm_q as u32);
    q_e -= norm_q;

    /* V0 */

    /* peak gain in linear domain */
    let tmp_v0 = f_mult(DBL_16_DIV_20, f_abs(g));
    v0_e += g_e - 4;
    let v0 = f_pow(DBL_10_DIV_16, 4, tmp_v0, v0_e, &mut v0_e);

    /* f*f*b*b */
    let tmp_f_pow2_f = f_mult(f, f);
    let tmp_f_pow2_b = f_mult(b, b);
    let tmp_fb = f_mult(tmp_f_pow2_f, tmp_f_pow2_b);
    let tmp_fb_e = 2 * b_e + 2 * f_e;

    /* b*b*b*b */
    let tmp_b = f_mult(tmp_f_pow2_b, tmp_f_pow2_b);
    let tmp_b_e = 4 * b_e;

    /* f*f*f*f */
    let tmp_f = f_mult(tmp_f_pow2_f, tmp_f_pow2_f);
    let tmp_f_e = 4 * f_e;

    /* tmp_q = (1.0f/(q*q) - 2.0f) */
    /* tmp_v = (V0*V0/(q*q) - 2.0f) */
    let mut tmp_q = f_mult(q, q);
    tmp_q_e = 2 * q_e;
    norm_q = count_leading_bits(tmp_q);
    tmp_q = tmp_q.wrapping_shl(norm_q as u32);
    tmp_q_e -= norm_q;
    let mut tmp_v = f_mult(v0, v0);
    v0_e *= 2;
    tmp_v = f_div_norm_signed(tmp_v, tmp_q, &mut tmp_v_e);
    tmp_v_e += v0_e - tmp_q_e;
    tmp_q = inv_fixp_e(tmp_q, &mut tmp_q_e); // -fDivNormSigned( MINVAL_DBL, tmp_q, &tmp_q_e);
    tmp_q = f_add_norm(tmp_q, tmp_q_e, MINVAL_DBL, 1, &mut tmp_q_e);
    tmp_v = f_add_norm(tmp_v, tmp_v_e, MINVAL_DBL, 1, &mut tmp_v_e);

    /* b*b*b*b + f*f*f*f */
    let tmp_sum = f_add_norm(tmp_f, tmp_f_e, tmp_b, tmp_b_e, &mut tmp_sum_e);

    /* b*b*b*b + (V0*V0/(q*q) - 2.0f)*f*f*b*b + f*f*f*f */
    let tmp_g_v =
        f_add_norm(tmp_sum, tmp_sum_e, f_mult(tmp_v, tmp_fb), tmp_v_e + tmp_fb_e, &mut tmp_g_v_e);

    /* b*b*b*b + (1.0f/(q*q) - 2.0f)*f*f*b*b + f*f*f*f */
    let tmp_g_1 =
        f_add_norm(tmp_sum, tmp_sum_e, f_mult(tmp_q, tmp_fb), tmp_q_e + tmp_fb_e, &mut tmp_g_1_e);

    /* 2nd order peak filter magnitude response */
    let mut gain;
    if g < 0 {
        gain = f_div_norm_signed(tmp_g_1, tmp_g_v, &mut gain_e);
        gain_e = gain_e + tmp_g_1_e - tmp_g_v_e;
    } else {
        gain = f_div_norm_signed(tmp_g_v, tmp_g_1, &mut gain_e);
        gain_e = gain_e - tmp_g_1_e + tmp_g_v_e;
    }

    let mut tmp_big_g_e = 0;
    let mut tmp_big_g = f_mult(DBL_16_DIV_20, big_g);
    tmp_big_g_e = tmp_big_g_e + big_g_e - 4;
    tmp_big_g = f_pow(DBL_10_DIV_16, 4, tmp_big_g, tmp_big_g_e, &mut tmp_big_g_e);

    if gain_e & 0x1 != 0 {
        gain_e += 1;
        gain >>= 1;
    }

    let mut result = sqrt_fixp(gain);
    *result_e += gain_e >> 1;

    result = f_mult(result, tmp_big_g);
    *result_e += tmp_big_g_e;

    result
}

const DBL_0_DOT_8: FixpDbl = 0x6666_6666;
const DBL_0_DOT_5: FixpDbl = 0x4000_0000;
const DBL_0_DOT_3: FixpDbl = 0x2666_6666;
const DBL_0_DOT_25: FixpDbl = 0x2000_0000;

const DBL_N2_DIV_4: FixpDbl = 0xC000_0000u32 as i32;
const DBL_2_DIV_4: FixpDbl = 0x4000_0000;
const DBL_N6DOT5_DIV_8: FixpDbl = 0x9800_0000u32 as i32;
const DBL_4DOT5_DIV_8: FixpDbl = 0x4800_0000;
const DBL_N3DOT5_DIV_4: FixpDbl = 0x9000_0000u32 as i32;
const DBL_N3DOT1_DIV_4: FixpDbl = 0x9ccc_cccdu32 as i32;
const DBL_1DOT8_DIV_2: FixpDbl = 0x7333_3333;
const DBL_N1DOT3_DIV_2: FixpDbl = 0xaccc_cccdu32 as i32;
const DBL_0DOT7_DIV_1: FixpDbl = 0x5999_999a;

/// Parameters of one peak filter of `compute_eq()`: f, f_e, q, g, g_e, G, G_e.
#[derive(Clone, Copy, Default)]
struct PeakParams {
    f: FixpDbl,
    f_e: i32,
    q: FixpDbl,
    g: FixpDbl,
    g_e: i32,
    big_g: FixpDbl,
    big_g_e: i32,
}

impl PeakParams {
    fn apply(&self, b: FixpDbl, eq_e: &mut i32) -> FixpDbl {
        peak_filter(
            self.f,
            self.f_e,
            self.q,
            0,
            self.g,
            self.g_e,
            self.big_g,
            self.big_g_e,
            b,
            16 - 1,
            eq_e,
        )
    }
}

/// `compute_eq(bands_nrm, nbands, sfreq_Hz, eq_1_idx, eq_1_strength, eq_2_idx, eq_2_strength,
/// eq)`: static equalizer response of the rules `eq_1_idx`/`eq_2_idx` (exponent `EQ_H_EXP`).
fn compute_eq(
    bands_nrm: &[FixpDbl],
    nbands: i32,
    sfreq_hz: i32,
    eq_1_idx: i32,
    eq_1_strength: FixpDbl,
    eq_2_idx: i32,
    eq_2_strength: FixpDbl,
    eq: &mut [FixpEqH],
) {
    let mut eq_e;
    let nb = nbands.max(0) as usize;

    /* initialize static eqs */
    let mut e1 = [PeakParams::default(); 3];
    let mut e2 = [PeakParams::default(); 3];

    let tmp_sfreq_hz = (sfreq_hz as FixpDbl).wrapping_shl(15);

    let set_eq1 = |p: &mut PeakParams, s: FixpDbl| {
        p.f = 12000i32 << 16;
        p.f_e = 15;
        p.q = DBL_0_DOT_3;
        p.g = f_mult(DBL_N2_DIV_4, s);
        p.g_e = 2;
        p.big_g = s;
        p.big_g_e = 0;
    };
    let set_eq2 = |p: &mut PeakParams, s: FixpDbl| {
        p.f = 12000i32 << 16;
        p.f_e = 15;
        p.q = DBL_0_DOT_3;
        p.g = f_mult(DBL_N3DOT5_DIV_4, s);
        p.g_e = 2;
        p.big_g = s;
        p.big_g_e = 0;
    };
    let set_eq5 = |p: &mut PeakParams, s: FixpDbl| {
        p.f = 35i32 << 25;
        p.f_e = 6;
        p.q = DBL_0_DOT_25;
        p.g = f_mult(DBL_N1DOT3_DIV_2, s);
        p.g_e = 1;
        p.big_g = s;
        p.big_g_e = 0;
    };

    /* RULE_EQ1, RULE_EQ2, RULE_EQ5 */
    if eq_1_idx == RULE_EQ1 {
        set_eq1(&mut e1[0], eq_1_strength);
    }
    if eq_2_idx == RULE_EQ1 {
        set_eq1(&mut e2[0], eq_2_strength);
    }
    if eq_1_idx == RULE_EQ2 {
        set_eq2(&mut e1[0], eq_1_strength);
    }
    if eq_2_idx == RULE_EQ2 {
        set_eq2(&mut e2[0], eq_2_strength);
    }
    if eq_1_idx == RULE_EQ5 {
        set_eq5(&mut e1[0], eq_1_strength);
    }
    if eq_2_idx == RULE_EQ5 {
        set_eq5(&mut e2[0], eq_2_strength);
    }

    if eq_1_idx == RULE_EQ1
        || eq_1_idx == RULE_EQ2
        || eq_1_idx == RULE_EQ5
        || eq_2_idx == RULE_EQ1
        || eq_2_idx == RULE_EQ2
        || eq_2_idx == RULE_EQ5
    {
        for i in 0..nb {
            eq[i] = MAXVAL_EQ_H;
            eq_e = 0;
            if eq_1_idx != RULE_NOPROC {
                let f = f_mult(f_abs(bands_nrm[i]), tmp_sfreq_hz);
                let tmp = e1[0].apply(f, &mut eq_e);
                eq[i] = scale_value(f_mult(eq[i], tmp), eq_e - EQ_H_EXP);
            }
            if eq_2_idx != RULE_NOPROC {
                let f = f_mult(f_abs(bands_nrm[i]), tmp_sfreq_hz);
                let tmp = e2[0].apply(f, &mut eq_e);
                eq[i] = scale_value(f_mult(eq[i], tmp), eq_e - EQ_H_EXP);
            }
        }
    }

    /* RULE_EQ3 */
    let set_eq3 = |p: &mut [PeakParams; 3], s: FixpDbl| {
        p[0].f = 200i32 << 23;
        p[0].f_e = 8;
        p[0].q = DBL_0_DOT_3;
        p[0].g = f_mult(DBL_N6DOT5_DIV_8, s);
        p[0].g_e = 3;
        p[0].big_g = f_mult(DBL_0DOT7_DIV_1, s);
        p[0].big_g_e = 0;

        p[1].f = 1300i32 << 20;
        p[1].f_e = 11;
        p[1].q = DBL_0_DOT_5;
        p[1].g = f_mult(DBL_1DOT8_DIV_2, s);
        p[1].g_e = 1;
        p[1].big_g = 0;
        p[1].big_g_e = 0;

        p[2].f = 600i32 << 21;
        p[2].f_e = 10;
        p[2].q = MAXVAL_DBL;
        p[2].g = f_mult(DBL_2_DIV_4, s);
        p[2].g_e = 2;
        p[2].big_g = 0;
        p[2].big_g_e = 0;
    };
    if eq_1_idx == RULE_EQ3 {
        set_eq3(&mut e1, eq_1_strength);
    }
    if eq_2_idx == RULE_EQ3 {
        set_eq3(&mut e2, eq_2_strength);
    }

    if eq_1_idx == RULE_EQ3 || eq_2_idx == RULE_EQ3 {
        for i in 0..nb {
            eq[i] = MAXVAL_EQ_H;
            eq_e = 0;
            if eq_1_idx == RULE_EQ3 {
                let f = f_mult(f_abs(bands_nrm[i]), tmp_sfreq_hz);
                let tmp = e1[0].apply(f, &mut eq_e);
                eq[i] = f_mult(eq[i], tmp);
                let tmp = e1[1].apply(f, &mut eq_e);
                eq[i] = f_mult(eq[i], tmp);
                let tmp = e1[2].apply(f, &mut eq_e);
                eq[i] = scale_value(f_mult(eq[i], tmp), eq_e - EQ_H_EXP);
            }
            if eq_2_idx == RULE_EQ3 {
                let f = f_mult(f_abs(bands_nrm[i]), tmp_sfreq_hz);
                let tmp = e2[0].apply(f, &mut eq_e);
                eq[i] = f_mult(eq[i], tmp);
                let tmp = e2[1].apply(f, &mut eq_e);
                eq[i] = f_mult(eq[i], tmp);
                let tmp = e2[2].apply(f, &mut eq_e);
                eq[i] = scale_value(f_mult(eq[i], tmp), eq_e - EQ_H_EXP);
            }
        }
    }

    /* RULE_EQ4 */
    let set_eq4 = |p: &mut [PeakParams; 3], s: FixpDbl| {
        p[0].f = 5000i32 << 18;
        p[0].f_e = 13;
        p[0].q = MAXVAL_DBL;
        p[0].g = f_mult(DBL_4DOT5_DIV_8, s);
        p[0].g_e = 3;
        p[0].big_g = f_mult(DBL_N3DOT1_DIV_4, s);
        p[0].big_g_e = 2;

        p[1].f = 1100i32 << 20;
        p[1].f_e = 11;
        p[1].q = DBL_0_DOT_8;
        p[1].g = f_mult(DBL_1DOT8_DIV_2, s);
        p[1].g_e = 1;
        p[1].big_g = 0;
        p[1].big_g_e = 0;
    };
    if eq_1_idx == RULE_EQ4 {
        set_eq4(&mut e1, eq_1_strength);
    }
    if eq_2_idx == RULE_EQ4 {
        set_eq4(&mut e2, eq_2_strength);
    }
    if eq_1_idx == RULE_EQ4 || eq_2_idx == RULE_EQ4 {
        for i in 0..nb {
            eq[i] = MAXVAL_EQ_H;
            eq_e = 0;
            if eq_1_idx == RULE_EQ4 {
                let f = f_mult(f_abs(bands_nrm[i]), tmp_sfreq_hz);
                let tmp = e1[0].apply(f, &mut eq_e);
                eq[i] = f_mult(eq[i], tmp);
                let tmp = e1[1].apply(f, &mut eq_e);
                eq[i] = scale_value(f_mult(eq[i], tmp), eq_e - EQ_H_EXP);
            }
            if eq_2_idx == RULE_EQ4 {
                let f = f_mult(f_abs(bands_nrm[i]), tmp_sfreq_hz);
                let tmp = e2[0].apply(f, &mut eq_e);
                eq[i] = f_mult(eq[i], tmp);
                let tmp = e2[1].apply(f, &mut eq_e);
                eq[i] = scale_value(f_mult(eq[i], tmp), eq_e - EQ_H_EXP);
            }
        }
    }

    let mut table: Option<&[FixpDbl; 58]> = None;
    if nbands == STFT_ERB_BANDS as i32 {
        table = match eq_1_idx {
            RULE_EQVF => Some(&EQVF_STFT_ERB),
            RULE_EQVB => Some(&EQVB_STFT_ERB),
            RULE_EQVFC => Some(&EQVFC_STFT_ERB),
            RULE_EQVBC => Some(&EQVBC_STFT_ERB),
            RULE_EQVOG => Some(&EQVOG_STFT_ERB),
            RULE_EQVS => Some(&EQVS_STFT_ERB),
            RULE_EQBTM => Some(&EQBTM_STFT_ERB),
            RULE_EQVBA => Some(&EQVBA_STFT_ERB),
            RULE_06_03 => Some(&COLOR_060_030_STFT_ERB),
            RULE_09_03 => Some(&COLOR_090_030_STFT_ERB),
            RULE_06_11 => Some(&COLOR_060_110_STFT_ERB),
            RULE_09_11 => Some(&COLOR_090_110_STFT_ERB),
            RULE_13_11 => Some(&COLOR_135_110_STFT_ERB),
            RULE_18_11 => Some(&COLOR_180_110_STFT_ERB),
            _ => None,
        };
    }

    if let Some(t) = table {
        for i in 0..nb {
            eq[i] = scale_value(t[i], 1 - EQ_H_EXP);
        }
    }
}

/// `converter_set_inout_format(fcInt, inout, num_channels, channel_vector)`: define the input
/// (`inout == 0`) or output (`inout == 1`) format as list of channels.
pub fn converter_set_inout_format(
    fc_int: &mut IisFormatConverterInternal,
    inout: i32,
    num_channels: i32,
    channel_vector: &[AfcFormatConverterChannelId],
) {
    let n = num_channels.max(0) as usize;
    if inout == 0 {
        fc_int.format_in_list_of_channels[..n].copy_from_slice(&channel_vector[..n]);
        fc_int.format_in_list_of_channels_nchan = num_channels;
    } else {
        debug_assert!(inout == 1);
        fc_int.format_out_list_of_channels[..n].copy_from_slice(&channel_vector[..n]);
        fc_int.format_out_list_of_channels_nchan = num_channels;
    }
}

/// Amplitude panning gains of `converter_init()`: returns (a1_m, a1_e, nrm_m, nrm_e).
fn panning_gains(alpha0: FixpDbl, alpha: FixpDbl) -> (FixpDbl, i32, FixpDbl, i32) {
    let alpha0 = f_mult(alpha0, FX_PI); /* exp PI 2 and exp alpha 1 */
    let mut alpha0_e = 3;
    let alpha = f_mult(alpha, FX_PI);
    let mut alpha_e = 3;

    let sin_alpha0_m = fixp_sin(alpha0, alpha0_e);
    let cos_alpha0_m = fixp_cos(alpha0, alpha0_e);

    let sin_alpha_m = fixp_sin(alpha, alpha_e);
    let cos_alpha_m = fixp_cos(alpha, alpha_e);

    let tan_alpha0_m = f_div_norm_signed(sin_alpha0_m, cos_alpha0_m, &mut alpha0_e);
    let tan_alpha_m = f_div_norm_signed(sin_alpha_m, cos_alpha_m, &mut alpha_e);

    let mut num_e = 0;
    let mut denom_e = 0;

    let mut num_m = f_add_norm(tan_alpha0_m, alpha0_e, tan_alpha_m, alpha_e, &mut num_e);
    num_m = f_add_norm(num_m, num_e, 0x0000_0001, -3, &mut num_e);
    let mut denom_m =
        f_add_norm(tan_alpha0_m, alpha0_e, tan_alpha_m.wrapping_neg(), alpha_e, &mut denom_e);
    denom_m = f_add_norm(denom_m, denom_e, 0x0000_0001, -3, &mut denom_e);

    let mut a1_e = 0;
    let mut a1_m = f_div_norm_signed(num_m, denom_m, &mut a1_e);
    a1_e = a1_e + num_e - denom_e;

    let mut nrm_e = 0;
    let mut tmp_e;

    let mut tmp_m = f_mult(a1_m, a1_m);
    tmp_e = a1_e << 1;
    tmp_m = f_add_norm(tmp_m, tmp_e, fl2fxconst_dbl(0.5f32 as f64), 1, &mut tmp_e);
    if tmp_e & 0x1 != 0 {
        tmp_m >>= 1;
        tmp_e += 1;
    }

    let nrm_m = inv_sqrt_norm2(tmp_m, &mut nrm_e);
    nrm_e -= tmp_e >> 1;

    a1_m = f_mult(a1_m, nrm_m);
    a1_e += nrm_e;

    (a1_m, a1_e, nrm_m, nrm_e)
}

/// `converter_init(fcInt, params, params_tmp, input_format, output_format, randomization,
/// sfreq_Hz, blocklength, nbands, bands_nrm, p_buffer)`: initialize the downmix (gains and
/// equalizers of the conversion from the input to the output list of channels).
///
/// Randomization is not supported (`randomization` must be `None`); `blocklength` is unused.
pub fn converter_init(
    fc_int: &mut IisFormatConverterInternal,
    params: &mut ConverterPr,
    params_tmp: &mut ConverterPrTmp,
    input_format: FdkConverterFormatid,
    output_format: FdkConverterFormatid,
    randomization: Option<&[i32]>,
    sfreq_hz: i32,
    _blocklength: i32,
    nbands: u32,
    bands_nrm: &[FixpDbl],
) -> ConverterStatus {
    /* Random setups are not supported. */
    debug_assert!(randomization.is_none());

    let mut in_out_src = [0i32; IN_OUT_TMP_N];
    let mut in_out_dst = [0i32; IN_OUT_TMP_N];
    let mut in_out_gain = [0 as FixpDmxH; IN_OUT_TMP_N];
    let mut in_out_proc = [0i32; IN_OUT_TMP_N];

    /* Immersive variables */
    let mut in_out_gain_l = [0 as FixpDmxH; IN_OUT_TMP_N];
    let mut in_out_src2 = [0i32; IN_OUT_TMP_N];
    let mut in_out_dst2 = [0i32; IN_OUT_TMP_N];
    let mut in_out_gain2 = [0 as FixpDmxH; IN_OUT_TMP_N];
    // inOutGainI of the C code is written but never read; it is not ported.
    let mut in_out_proc2 = [0i32; IN_OUT_TMP_N];

    let mut azi_ele = [[0i32; 2]; (CH_LFE2 + 1) as usize];

    debug_assert!(nbands as usize <= MAXBANDS);

    /* init in_out vectors */
    for i in 0..IN_OUT_N {
        in_out_src[i] = 0;
        in_out_dst[i] = 0;
        in_out_gain[i] = 0;
        in_out_proc[i] = 0;
        in_out_gain_l[i] = -1;

        in_out_src2[i] = 0;
        in_out_dst2[i] = 0;
        in_out_gain2[i] = 0;
        in_out_proc2[i] = 0;
    }

    /* parameter range checking */

    if input_format >= FDK_NFORMATS - 1 {
        return FORMAT_CONVERTER_STATUS_INFORMAT;
    }

    if output_format >= FDK_NFORMATS || output_format == FDK_NFORMATS - 2 {
        return FORMAT_CONVERTER_STATUS_OUTFORMAT;
    }

    if !(SFREQ_HZ_MIN..=SFREQ_HZ_MAX).contains(&sfreq_hz) {
        return FORMAT_CONVERTER_STATUS_SFREQ;
    }

    if (nbands < NBANDS_MIN || nbands > NBANDS_MAX) && nbands != 0 {
        return FORMAT_CONVERTER_STATUS_BANDS;
    }

    for i in 0..nbands as usize {
        if bands_nrm[i] < 0 {
            // if ((bands_nrm[i] > 1.0f) || (bands_nrm[i] < 0.0f))
            return FORMAT_CONVERTER_STATUS_BANDS;
        }
    }

    /* get input and output format info */
    if input_format != FDK_FORMAT_IN_LISTOFCHANNELS {
        return FORMAT_CONVERTER_STATUS_INFORMAT;
    }
    let nchanin = fc_int.format_in_list_of_channels_nchan;

    if output_format != FDK_FORMAT_OUT_LISTOFCHANNELS {
        return FORMAT_CONVERTER_STATUS_OUTFORMAT;
    }
    let nchanout = fc_int.format_out_list_of_channels_nchan;
    let format_in = fc_int.format_in_list_of_channels;
    let format_out = fc_int.format_out_list_of_channels;

    /* copy channel angles (they may be modified for random setups) */
    for i in 0..(CH_LFE2 + 1) as usize {
        azi_ele[i][0] = CH_AZI_ELE[i][0];
        azi_ele[i][1] = CH_AZI_ELE[i][1];
    }

    let dmx_rules: &[[i32; 8]] =
        if fc_int.immersive_downmix_flag == 1 && (nchanout == 5 || nchanout == 6) {
            init_elev_sptl_parms(fc_int, sfreq_hz, nchanout);
            &DMX_RULES_IMMERSIVE
        } else {
            &DMX_RULES_CLASSIC
        };
    debug_assert!(dmx_rules.len() <= MAX_NUM_DMX_RULES);

    /* more parameter error checking */

    /* Define format conversion. Downmix matrices genration. */

    let mut ch: i32 = 0;
    for i in 0..nchanin {
        /* map EMPTY channel as silent channel, i.e. generate a dmx mtx line containing zeros */
        let input_channel_id = format_in[i as usize];
        if input_channel_id == CH_EMPTY {
            let c = ch as usize;
            in_out_src[c] = i; /* input channel index */
            in_out_dst[c] = 0; /* use first output channel. unimportant since gain is set to 0.0 */
            in_out_gain[c] = 0; /* gain: 0 gain, silence */
            in_out_proc[c] = 0; /* processing: 0 = no EQ */
            ch += 1;
            continue;
        }

        /* test if input channel exists in output format */
        let output_channel_idx = find_channel(input_channel_id, &format_out, nchanout);

        /* input channel exists in output format
        (just copy channel from input to output) */

        if output_channel_idx >= 0 {
            let c = ch as usize;
            in_out_src[c] = i; /* input channel index */
            in_out_dst[c] = output_channel_idx; /* output channel index */
            in_out_gain[c] = MAXVAL_DMX_H; /* gain */
            in_out_proc[c] = 0; /* processing */
            ch += 1;

            /* input channel does not exist in output format
            (apply downmix rules) */
        } else {
            /* search best downmix rule */
            let ch_start = ch;
            let mut k = 0usize;
            while dmx_rules[k][0] != -1 {
                let rule = &dmx_rules[k];
                /* when the downmix rule is RULE_VIRTUAL : perform the "spatial elevation
                 * rendering" */
                if rule[0] == input_channel_id && rule[4] == RULE_VIRTUAL {
                    let h = rule[1] as usize;
                    if nchanout == 6 || nchanout == 5 {
                        for mch in 0..6usize {
                            if fc_int.gvh[h][mch] == 0 {
                                continue;
                            }

                            let dst = if nchanout == 6 {
                                mch as i32
                            } else if mch > 3 {
                                mch as i32 - 1
                            } else {
                                mch as i32
                            };

                            if ch >= IN_OUT_N as i32 {
                                return FORMAT_CONVERTER_STATUS_MISSING_RULE;
                            }
                            let c = ch as usize;
                            in_out_src[c] = i;
                            in_out_dst[c] = dst;
                            in_out_gain[c] = scale_value_sgl(
                                fx_dbl2fx_dmx_h(f_mult(fc_int.gvh[h][mch], rule[3])),
                                fc_int.gvh_e[h][mch],
                            );
                            in_out_gain_l[c] = scale_value_sgl(
                                fx_dbl2fx_dmx_h(f_mult(fc_int.gvl[h][mch], rule[3])),
                                fc_int.gvl_e[h][mch],
                            );

                            in_out_proc[c] = rule[7];

                            fc_int.top_in[h] = i;

                            ch += 1;
                        }
                    }

                    break;
                }
                /* find rule with input == input_channel_id */
                if rule[0] == input_channel_id {
                    /* top loudspeaker to all height loudspeakers rule */
                    if rule[4] == RULE_TOP2ALLU || rule[4] == RULE_TOP2ALLM {
                        let (first, last) = if rule[4] == RULE_TOP2ALLU {
                            (CH_U_000, CH_U_180)
                        } else {
                            (CH_M_000, CH_M_180)
                        };
                        /* count number of height (horizontal) channels */
                        let mut ch_count = 0;
                        for uch in first..=last {
                            if find_channel(uch, &format_out, nchanout) != -1 {
                                ch_count += 1;
                            }
                        }
                        /* if there are height (horizontal) channels, top is mixed to these */
                        if ch_count > 0 {
                            let mut gain_e = 0;
                            let ch_count_e = 4;
                            let gain_m = fx_dbl2fx_dmx_h(inv_sqrt_norm2(
                                (ch_count as i32)
                                    .wrapping_shl((DFRACT_BITS - (ch_count_e + 1)) as u32),
                                &mut gain_e,
                            ));
                            gain_e -= ch_count_e >> 1;

                            for uch in first..=last {
                                if ch >= IN_OUT_N as i32 {
                                    return FORMAT_CONVERTER_STATUS_MISSING_RULE;
                                }
                                let c = ch as usize;
                                in_out_src[c] = i; /* input channel index */
                                in_out_dst[c] = find_channel(uch, &format_out, nchanout); /* output channel index */
                                in_out_gain[c] =
                                    fx_dbl2fx_dmx_h(scale_value(f_mult(rule[3], gain_m), gain_e));
                                in_out_proc[c] = rule[7]; /* processing */
                                if in_out_dst[c] != -1 {
                                    ch += 1;
                                }
                            }
                            break;
                        }
                        k += 1;
                        continue;
                    }
                    /* do target channels of rule exist? */
                    let mut idx = find_channel(rule[1], &format_out, nchanout);
                    let mut idx2 = idx;
                    if rule[2] != -1 {
                        /* rule with two output channels */
                        idx2 = find_channel(rule[2], &format_out, nchanout);
                    }
                    /* do all rule output channels exist? */
                    if idx >= 0 && idx2 >= 0 {
                        if rule[4] == RULE_PANNING || rule[4] == RULE_AUTOPAN {
                            /* add offest such that all angles are positive */
                            let mut azi_1 = azi_ele[format_out[idx as usize] as usize][0];
                            let mut azi_2 = azi_ele[format_out[idx2 as usize] as usize][0];
                            let mut azi_target = azi_ele[input_channel_id as usize][0];
                            let mn = f_min(f_min(azi_1, azi_2), azi_target);
                            azi_1 -= mn;
                            azi_2 -= mn;
                            azi_target -= mn;
                            let (alpha0, mut alpha);
                            /* manual and auto panning */
                            if rule[4] == RULE_PANNING {
                                /* manual definition of parameters */
                                alpha0 = rule[5]; /* angle of system +-alpha0 */
                                alpha = rule[6]; /* definition: negative is left */
                            } else
                            /* if (dmx_rules[k][4] == RULE_AUTOPAN) */
                            {
                                /* amplitude panning angle alpha0 */
                                alpha0 = f_abs(azi_1 - azi_2) >> 1;
                                /* center angle */
                                let center = (azi_1 >> 1) + (azi_2 >> 1);
                                /* amplitude panning angle alpha */
                                alpha = center - azi_target;
                                if azi_1 > azi_2 {
                                    alpha = -alpha;
                                }
                            }

                            let (a1_m, a1_e, nrm_m, nrm_e) = panning_gains(alpha0, alpha);

                            /* panning */
                            if ch >= IN_OUT_N as i32 {
                                return FORMAT_CONVERTER_STATUS_MISSING_RULE;
                            }
                            let c = ch as usize;
                            in_out_src[c] = i; /* input channel index */
                            in_out_dst[c] = idx; /* output channel index */
                            in_out_gain[c] =
                                fx_dbl2fx_dmx_h(scale_value(f_mult(rule[3], a1_m), a1_e));
                            in_out_proc[c] = rule[7]; /* processing */
                            ch += 1;
                            let c = ch as usize;
                            in_out_src[c] = i; /* input channel index */
                            in_out_dst[c] = idx2; /* output channel index */
                            in_out_gain[c] =
                                fx_dbl2fx_dmx_h(scale_value(f_mult(rule[3], nrm_m), nrm_e));
                            in_out_proc[c] = rule[7]; /* processing */
                            ch += 1;
                        } else {
                            /* only external processing option */
                            if ch >= IN_OUT_N as i32 {
                                return FORMAT_CONVERTER_STATUS_MISSING_RULE;
                            }
                            let c = ch as usize;
                            in_out_src[c] = i; /* input channel index */
                            in_out_dst[c] = idx; /* output channel index */
                            in_out_gain[c] = fx_dbl2fx_dmx_h(rule[3]); /* gain */
                            in_out_proc[c] = rule[7]; /* processing */
                            ch += 1;
                            /* phantom source (and thus 2nd destination channel)? */
                            if rule[2] != -1 {
                                idx = find_channel(rule[2], &format_out, nchanout);
                                if idx != -1 {
                                    let c = ch as usize;
                                    in_out_src[c] = i; /* input channel index */
                                    in_out_dst[c] = idx; /* output channel index */
                                    in_out_gain[c] = fx_dbl2fx_dmx_h(rule[3]); /* gain */
                                    in_out_proc[c] = rule[7]; /* processing */
                                    ch += 1;
                                }
                            }
                        }
                        /* done (break loop over k) */
                        break;
                    }
                }
                k += 1;
            } /* = loop over k end */
            if ch_start == ch {
                /* downmix rule missing */
                return FORMAT_CONVERTER_STATUS_MISSING_RULE;
            }
        }
    }
    if ch >= IN_OUT_N as i32 {
        return FORMAT_CONVERTER_STATUS_MISSING_RULE;
    }
    let c = ch as usize;
    in_out_src[c] = -1;
    in_out_dst[c] = -1;
    in_out_gain[c] = -1;
    in_out_proc[c] = -1;
    in_out_gain_l[c] = -1;

    /* re-order in_out such that ordering is by destination channel */
    let mut ptr = 0usize;
    for i in 0..nchanout {
        let mut c = 0usize;
        while in_out_src[c] >= 0 {
            if i == in_out_dst[c] {
                params_tmp.in_out_src[ptr] = in_out_src[c] as i8;
                params_tmp.in_out_dst[ptr] = in_out_dst[c] as i8;
                params_tmp.in_out_gain[ptr] = in_out_gain[c];
                params_tmp.in_out_proc[ptr] = in_out_proc[c] as i8;
                params_tmp.in_out_gain_l[ptr] = in_out_gain_l[c];
                ptr += 1;
            }
            c += 1;
        }
        ch = c as i32;
    }
    // After the loop `ch` is the number of entries (C reuses `ch` as loop variable); for an
    // output setup without channels it keeps its previous value, which is the same.
    let c = ch as usize;
    params_tmp.in_out_src[c] = -1;
    params_tmp.in_out_dst[c] = -1;
    params_tmp.in_out_gain[c] = -1; //-1
    params_tmp.in_out_proc[c] = -1;

    if fc_int.immersive_downmix_flag == 1
        && (fc_int.num_output_channels == 6 || fc_int.num_output_channels == 5)
    {
        params_tmp.in_out_gain_l[c] = -1;

        let mut ch: usize = 0;
        for i in 0..nchanin {
            let input_channel_id = format_in[i as usize];
            if input_channel_id == CH_EMPTY {
                continue;
            }

            /* test if input channel exists in output format */
            let output_channel_idx = find_channel(input_channel_id, &format_out, nchanout);

            /* input channel exists in output format
            (just copy channel from input to output) */
            if output_channel_idx >= 0 {
                in_out_src2[ch] = i; /* input channel index */
                in_out_dst2[ch] = output_channel_idx; /* output channel index */
                in_out_gain2[ch] = MAXVAL_DMX_H; /* gain */
                in_out_proc2[ch] = 0; /* processing */
                ch += 1;

                /* input channel does not exist in output format
                (apply downmix rules) */
            } else {
                /* search best downmix rule */
                let ch_start = ch;
                let mut k = 0usize;
                while dmx_rules[k][0] != -1 {
                    let rule = &dmx_rules[k];
                    if rule[4] == RULE_VIRTUAL {
                        k += 1;
                        continue;
                    }
                    if rule[0] == input_channel_id {
                        /* top loudspeaker to all height loudspeakers rule */
                        if rule[4] == RULE_TOP2ALLU || rule[4] == RULE_TOP2ALLM {
                            let (first, last) = if rule[4] == RULE_TOP2ALLU {
                                (CH_U_000, CH_U_180)
                            } else {
                                (CH_M_000, CH_M_180)
                            };
                            /* count number of height channels */
                            let mut ch_count = 0;
                            for uch in first..=last {
                                if find_channel(uch, &format_out, nchanout) != -1 {
                                    ch_count += 1;
                                }
                            }
                            /* if there are height channels, top is mixed to these */
                            if ch_count > 0 {
                                /*
                                  float gain;
                                  gain = 1.0f / (float)sqrt(ch_count);
                                */
                                let mut gain_e = 0;
                                let ch_count_e = 4;
                                let gain_m = fx_dbl2fx_dmx_h(inv_sqrt_norm2(
                                    (ch_count as i32)
                                        .wrapping_shl((DFRACT_BITS - (ch_count_e + 1)) as u32),
                                    &mut gain_e,
                                ));
                                gain_e -= ch_count_e >> 1;

                                for uch in first..=last {
                                    in_out_src2[ch] = i; /* input channel index */
                                    in_out_dst2[ch] = find_channel(uch, &format_out, nchanout); /* output channel index */
                                    in_out_gain2[ch] = fx_dbl2fx_dmx_h(scale_value(
                                        f_mult(rule[3], gain_m),
                                        gain_e,
                                    )); /* gain */
                                    in_out_proc2[ch] = rule[7]; /* processing */
                                    if in_out_dst2[ch] != -1 {
                                        ch += 1;
                                    }
                                }
                                break;
                            }
                            k += 1;
                            continue;
                        }
                        /* do target channels of rule exist? */
                        let mut idx = find_channel(rule[1], &format_out, nchanout);
                        let mut idx2 = idx;
                        if rule[2] != -1 {
                            /* rule with two output channels */
                            idx2 = find_channel(rule[2], &format_out, nchanout);
                        }
                        /* do all rule output channels exist? */
                        if idx >= 0 && idx2 >= 0 {
                            if rule[4] == RULE_PANNING || rule[4] == RULE_AUTOPAN {
                                let mut alpha: FixpDbl = 0;
                                let mut alpha0: FixpDbl = 0;

                                /* add offest such that all angles are positive */
                                let mut azi_1 = azi_ele[format_out[idx as usize] as usize][0];
                                let mut azi_2 = azi_ele[format_out[idx2 as usize] as usize][0];
                                let mut azi_target = azi_ele[input_channel_id as usize][0];
                                let mn = f_max(f_min(azi_1, azi_2), azi_target);
                                azi_1 -= mn;
                                azi_2 -= mn;
                                azi_target -= mn;
                                /* manual and auto panning */
                                if rule[4] == RULE_PANNING {
                                    /* manual definition of parameters */
                                    alpha0 = rule[5]; /* angle of system +-alpha0 */
                                    alpha = rule[6]; /* definition: negative is left */
                                } else if rule[4] == RULE_AUTOPAN {
                                    /* amplitude panning angle alpha0 */
                                    alpha0 = f_abs(azi_1 - azi_2) >> 1;
                                    /* center angle */
                                    let center = (azi_1 >> 1) + (azi_2 >> 1);
                                    /* amplitude panning angle alpha */
                                    alpha = center - azi_target;
                                    if azi_1 > azi_2 {
                                        alpha = -alpha;
                                    }
                                }

                                let (a1_m, a1_e, nrm_m, nrm_e) = panning_gains(alpha0, alpha);

                                /* panning */
                                in_out_src2[ch] = i; /* input channel index */
                                in_out_dst2[ch] = idx; /* output channel index */
                                in_out_gain2[ch] =
                                    fx_dbl2fx_dmx_h(scale_value(f_mult(rule[3], a1_m), a1_e)); /* gain */
                                in_out_proc2[ch] = rule[7]; /* processing */
                                ch += 1;
                                in_out_src2[ch] = i; /* input channel index */
                                in_out_dst2[ch] = idx2; /* output channel index */
                                in_out_gain2[ch] =
                                    fx_dbl2fx_dmx_h(scale_value(f_mult(rule[3], nrm_m), nrm_e)); /* gain */
                                in_out_proc2[ch] = rule[7]; /* processing */
                                ch += 1;
                            } else {
                                /* only external processing option */
                                in_out_src2[ch] = i; /* input channel index */
                                in_out_dst2[ch] = idx; /* output channel index */
                                in_out_gain2[ch] = fx_dbl2fx_dmx_h(rule[3]); /* gain */
                                in_out_proc2[ch] = rule[7]; /* processing */
                                ch += 1;
                                /* phantom source (and thus 2nd destination channel)? */
                                if rule[2] != -1 {
                                    idx = find_channel(rule[2], &format_out, nchanout);
                                    if idx != -1 {
                                        in_out_src2[ch] = i; /* input channel index */
                                        in_out_dst2[ch] = idx; /* output channel index */
                                        in_out_gain2[ch] = fx_dbl2fx_dmx_h(rule[3]); /* gain */
                                        in_out_proc2[ch] = rule[7]; /* processing */
                                        ch += 1;
                                    }
                                }
                            }
                            /* done (break loop over k) */
                            break;
                        }
                    }
                    k += 1;
                } /* = loop over k end */
                if ch_start == ch {
                    /* downmix rule missing */
                    return FORMAT_CONVERTER_STATUS_MISSING_RULE;
                }
            }
            if ch >= IN_OUT_TMP_N - 2 {
                // C writes beyond its vectors here (not reachable with the layouts of the decoder).
                mpeghdec_fdk::deviation::hit();
                return FORMAT_CONVERTER_STATUS_MISSING_RULE;
            }
        }
        in_out_src2[ch] = -1;
        in_out_dst2[ch] = -1;
        in_out_gain2[ch] = -1;
        in_out_proc2[ch] = -1;

        if ch >= IN_OUT_N {
            // C writes beyond params_tmp here (not reachable with the layouts of the decoder).
            mpeghdec_fdk::deviation::hit();
            return FORMAT_CONVERTER_STATUS_FAILED;
        }

        /* re-order in_out such that ordering is by destination channel */
        let mut ptr = 0usize;
        for i in 0..nchanout {
            ch = 0;
            while in_out_src2[ch] >= 0 {
                if i == in_out_dst2[ch] {
                    params_tmp.in_out_src2[ptr] = in_out_src2[ch] as i8;
                    params_tmp.in_out_dst2[ptr] = in_out_dst2[ch] as i8;
                    params_tmp.in_out_gain2[ptr] = in_out_gain2[ch];
                    if in_out_proc2[ch] == RULE_EQBTM {
                        params_tmp.in_out_proc2[ptr] = RULE_NOPROC as i8;
                    } else {
                        params_tmp.in_out_proc2[ptr] = in_out_proc2[ch] as i8;
                    }
                    ptr += 1;
                }
                ch += 1;
            }
        }
        if ch != ptr {
            return FORMAT_CONVERTER_STATUS_FAILED;
        }
        params_tmp.in_out_src2[ch] = -1;
        params_tmp.in_out_dst2[ch] = -1;
        params_tmp.in_out_gain2[ch] = -1;
        params_tmp.in_out_proc2[ch] = -1;
    }

    /* random formats: adjust gains and eqs */

    /* initialize static eqs: */
    let nb = nbands as i32;
    for (idx, rule) in [RULE_EQ1, RULE_EQ2, RULE_EQ3, RULE_EQ4, RULE_EQ5].into_iter().enumerate() {
        compute_eq(bands_nrm, nb, sfreq_hz, rule, MAXVAL_DBL, RULE_NOPROC, 0, &mut params.eq[idx]);
    }

    /* initialize static eqs for the immersive mode */
    for rule in RULE_EQVF..=RULE_18_11 {
        compute_eq(
            bands_nrm,
            nb,
            sfreq_hz,
            rule,
            MAXVAL_DBL,
            RULE_NOPROC,
            0,
            &mut params.eq[(rule - 1) as usize],
        );
    }

    /* adaptive eq parameter init */

    FORMAT_CONVERTER_STATUS_OK
}

/// `FREQ_MACRO(x)`
const fn freq_macro(x: i32) -> FixpDbl {
    x << (31 - 19)
}
pub const FREQ_E: i32 = 19;

/// `setActiveDownmixRange_StftErb(fcInt, fs)`: ERB bands (2.8 .. 10 kHz) that use the G_VH gains
/// in the 3D rendering of the immersive mode.
pub fn set_active_downmix_range_stft_erb(fc_int: &mut IisFormatConverterInternal, fs: i32) {
    for i in 0..STFT_ERB_BANDS {
        let f_tmp = f_mult(freq_macro(fs), FC_STFT_ERB[i]);
        fc_int.is4_gvh_stft_erb[i] =
            (f_tmp >= freq_macro(2800 << 1) && f_tmp <= freq_macro(10000 << 1)) as _;
    }

    let mut i = 0;
    while i < STFT_ERB_BANDS {
        /* search for first non-zero */
        if fc_int.is4_gvh_stft_erb[i] != 0 {
            fc_int.erb_is4_gvh_l = i as u32;
            break;
        }
        i += 1;
    }
    while i < STFT_ERB_BANDS {
        /* search for next zero */
        if fc_int.is4_gvh_stft_erb[i] == 0 {
            fc_int.erb_is4_gvh_h = i as u32;
            break;
        }
        i += 1;
    }
}

/// Power normalization of one panning gain table (`GVH` or `GVL`) of `normalizePG()`.
fn normalize_pg_table(g: &mut [[FixpDbl; 6]; 13], g_e: &mut [[i32; 6]; 13]) {
    for chin in TFC..=VOG {
        let mut p: FixpDbl = 0;
        let mut p_e = 0;
        for chout in FL..=SR {
            /* fAddNorm */
            // P  +=  GVH[chin][chout] * GVH[chin][chout];
            if g[chin][chout] == MAXVAL_DBL {
                p = f_add_norm(MAXVAL_DBL, 0, p, p_e, &mut p_e);
            } else {
                p = f_add_norm(f_pow2(g[chin][chout]), 0, p, p_e, &mut p_e);
            }
        }

        /* sqrtFixp */
        // P    = sqrt (P);

        if p_e & 0x1 != 0 {
            p_e += 1;
            p >>= 1;
        }

        if p != MAXVAL_DBL {
            p = sqrt_fixp(p);
        }

        p_e >>= 1;

        for chout in FL..=SR {
            /* fDivNorm */
            // GVH[chin][chout] /= P;
            g[chin][chout] = f_div_norm_signed(g[chin][chout], p, &mut g_e[chin][chout]);
            g_e[chin][chout] -= p_e;
        }
    }
}

/// `normalizePG(fcInt)`: power normalization of G_VH and G_VL.
pub fn normalize_pg(fc_int: &mut IisFormatConverterInternal) {
    /* power normalization of GVH */
    normalize_pg_table(&mut fc_int.gvh, &mut fc_int.gvh_e);
    /* power normalization of GVL */
    normalize_pg_table(&mut fc_int.gvl, &mut fc_int.gvl_e);
}

/// `randElevSptlParms(fcInt)`
fn rand_elev_sptl_parms(fc_int: &mut IisFormatConverterInternal) {
    /*  (6) power normalization */
    normalize_pg(fc_int);
}

/// `initElevSptlParms(fcInt, fs, nchanout)`: `fs` is the sampling frequency (an integer passed
/// as `FIXP_DBL`).
fn init_elev_sptl_parms(fc_int: &mut IisFormatConverterInternal, fs: FixpDbl, nchanout: i32) {
    set_active_downmix_range_stft_erb(fc_int, fs);

    for ch in 0..13 {
        fc_int.top_in[ch] = -1;
    }

    for ch in 0..nchanout as usize {
        fc_int.mid_out[ch] = ch as u32; /* 5.1 layout */
    }
    if nchanout == 5 {
        fc_int.mid_out[3] += 1;
        fc_int.mid_out[4] += 1;
    }
    /* update panning coefficients if random ( normalizePG is called anyways ) */
    rand_elev_sptl_parms(fc_int);
}
