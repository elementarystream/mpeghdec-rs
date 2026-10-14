// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/ltp_post.h, libMpeghDec/src/ltp_post.cpp and
// libMpeghDec/src/arm/ltp_post_arm.cpp (the ARM file only contains NEON/ARMv7 code, which is
// inactive in the reference configuration).

//! TCX/AAC long term prediction postfilter (LTPF).
//!
//! The C++ code works with pointers into the local buffers `buf_in` (input signal with the
//! `mem_in` history in front) and `buf_out` (output signal with the `mem_out` history in front)
//! and reads backwards from them (`x0 = &synth_ltp[-pitch_int]`, `y0[-i]`). Here the whole
//! buffers are passed together with the start index of the respective part.

use mpeghdec_fdk::digest::StateDigest;

use crate::usacdec_const::PIT_MAX_MAX;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::lpc::{clpc_auto_to_lpc, clpc_synthesis_sgl};
use mpeghdec_fdk::mdct::PCM_OUT_HEADROOM;
use mpeghdec_fdk::scale::{saturate_right_shift, scale_values_saturate_dbl_to_sgl};

/// `AAC_LTP_FILTERS_LENGTH`
pub const AAC_LTP_FILTERS_LENGTH: usize = 4;
/// `AAC_LTP_FILTERS2_LENGTH`
pub const AAC_LTP_FILTERS2_LENGTH: usize = 7;

/// `LTP_MEM_IN_SIZE`
pub const LTP_MEM_IN_SIZE: usize = AAC_LTP_FILTERS2_LENGTH - 1;
/// `LTP_MEM_OUT_SIZE`
pub const LTP_MEM_OUT_SIZE: usize = (2 * PIT_MAX_MAX as usize) + AAC_LTP_FILTERS_LENGTH;

/// `LTP_LPC_ORDER`
const LTP_LPC_ORDER: usize = 24;
/// `TCXLTP_MAX_DELAY`
const TCXLTP_MAX_DELAY: usize = 3;

/// `PIT_MIN_12k8`: minimum pitch lag with resolution 1/4
const PIT_MIN_12K8: i32 = 34;
/// `PIT_FR2_12k8`: minimum pitch lag with resolution 1/2
const PIT_FR2_12K8: i32 = 128;
/// `PIT_FR1_12k8`: minimum pitch lag with resolution 1
const PIT_FR1_12K8: i32 = 160;
/// `PIT_MAX_12k8`: maximum pitch lag
const PIT_MAX_12K8: i32 = 231;

/// `L_FRAME_1024`
const L_FRAME_1024: usize = 1024;
/// `L_FRAME_MAX`
const L_FRAME_MAX: i32 = 1024;
/// `FSCALE_DENOM`
const FSCALE_DENOM: i32 = 12800;

/// `ALPHA`: `FL2FXCONST_DBL(0.95)`
const ALPHA: FixpDbl = 0x7999999A;
/// `FL2FX_DBL(1.0 / 384)` (evaluated in double precision)
const ONE_BY_384: FixpDbl = 0x00555555;

/// `aac_ltp_post_inter`
static AAC_LTP_POST_INTER: [FixpSgl; 17] = [
    0x223C, 0x21D2, 0x209B, 0x1EA9, 0x1C1B, 0x1915, 0x15C3, 0x124F, 0x0EE2, 0x0BA1, 0x08A9, 0x0610,
    0x03E5, 0x0232, 0x00FA, 0x003E, 0x0000,
];

/// `aac_ltp_post_tilt`
static AAC_LTP_POST_TILT: [FixpSgl; 28] = [
    0x22C1,
    0x38B0,
    0x1D7A,
    0x075F,
    0xFFC8u16 as i16,
    0xFFF1u16 as i16,
    0xFFFEu16 as i16,
    0x234E,
    0x3931,
    0x1D2A,
    0x06ED,
    0xFF8Cu16 as i16,
    0xFFE2u16 as i16,
    0xFFFCu16 as i16,
    0x23E6,
    0x39BC,
    0x1CD3,
    0x0673,
    0xFF4Du16 as i16,
    0xFFD2u16 as i16,
    0xFFFAu16 as i16,
    0x2489,
    0x3A4F,
    0x1C75,
    0x05F0,
    0xFF0Bu16 as i16,
    0xFFC1u16 as i16,
    0xFFF8u16 as i16,
];

/// `AAC_LTP_FILTER`
#[derive(StateDigest)]
struct AacLtpFilter {
    filt: &'static [FixpSgl],
    length: i32,
}

/// `aacLtpFilters`
static AAC_LTP_FILTERS: [AacLtpFilter; 1] =
    [AacLtpFilter { filt: &AAC_LTP_POST_INTER, length: AAC_LTP_FILTERS_LENGTH as i32 }];
/// `aacLtpFilters2`
static AAC_LTP_FILTERS2: [AacLtpFilter; 1] =
    [AacLtpFilter { filt: &AAC_LTP_POST_TILT, length: AAC_LTP_FILTERS2_LENGTH as i32 }];

/// `ltp_get_lpc(input, length, A, A_e, lpcorder)`: LPC coefficients of `input` (`length` values).
fn ltp_get_lpc(input: &[FixpSgl], a: &mut [FixpSgl], a_e: &mut i32, lpcorder: i32) {
    let length = input.len() as i32;
    let mut r = [0 as FixpDbl; LTP_LPC_ORDER + 1];

    debug_assert!(length <= L_FRAME_MAX);

    /* calc r[0], determine shift */
    let mut s = 0;
    let mut rtmp: FixpDbl = 0;
    for &x in input {
        rtmp += f_pow2_div2(x) >> s;
        if rtmp >= 0x40000000 {
            s += 1;
            rtmp >>= 1;
        }
    }

    rtmp = f_max(rtmp, 100);

    let mut sh2 = f_norm(rtmp);
    const ONE_0001: FixpDbl = 0x4001a36e; /* constant 1.0001 in Q1.30 */
    rtmp = f_mult(rtmp << sh2, ONE_0001);
    sh2 -= 1;
    debug_assert!(sh2 >= 0);
    r[0] = rtmp;

    /* calc r[1...] */
    for i in 1..=lpcorder {
        let mut rtmp: FixpDbl = 0;
        for j in 0..(length - i) {
            rtmp += f_mult_div2(input[j as usize], input[(j + i) as usize]) >> s;
        }
        r[i as usize] = rtmp << sh2;
    }

    clpc_auto_to_lpc(a, a_e, &r, 0, lpcorder, None, 1);
}

/// `ltp_get_zir(zir, length, synth_ltp, synth, A, A_e, lpcorder, gain, gainIdx, pitch_int,
/// pitch_fr, pitres, filtIdx)`: zero input response of the LTP filter transition.
///
/// `synth_ltp` is `buf_out[ltp_pos..]`, `synth` is `buf_in[syn_pos..]`; `zir` holds `length`
/// values.
#[allow(clippy::too_many_arguments)]
fn ltp_get_zir(
    zir: &mut [FixpDbl],
    buf_out: &[FixpSgl],
    ltp_pos: usize,
    buf_in: &[FixpSgl],
    syn_pos: usize,
    a: &[FixpSgl],
    a_e: i32,
    lpcorder: i32,
    gain: FixpSgl,
    gain_idx: i32,
    pitch_int: i32,
    pitch_fr: i32,
    pitres: i32,
    filt_idx: i32,
) {
    let mut buf = [0 as FixpDbl; LTP_LPC_ORDER];
    let mut st_idx = 0;

    /* x0 = &synth_ltp[-pitch_int], x1 = x0 - 1, y0 = synth */
    let x_start = ltp_pos as isize - pitch_int as isize;
    let y_start = syn_pos as isize;

    debug_assert!(filt_idx >= 0);

    let f = &AAC_LTP_FILTERS[filt_idx as usize];
    let f2 = &AAC_LTP_FILTERS2[filt_idx as usize];
    let w0 = &f.filt[pitch_fr as usize..];
    let w1 = &f.filt[(pitres - pitch_fr) as usize..];
    let l = f.length;
    let l2 = f2.length;
    let v0 = &f2.filt[(gain_idx * l2) as usize..];

    for j in 0..lpcorder as usize {
        let x0 = x_start + j as isize;
        let y0 = y_start + j as isize;
        let mut s: FixpDbl = 0;
        let mut s2: FixpDbl = 0;

        let mut k = 0usize;
        for i in 0..l as isize {
            s += f_mult_div2(w0[k], buf_out[(x0 + i) as usize])
                + f_mult_div2(w1[k], buf_out[(x0 - 1 - i) as usize]);
            k += pitres as usize;
        }

        for i in 0..l2 as isize {
            s2 += f_mult_div2(v0[i as usize], buf_in[(y0 - i) as usize]);
        }

        s2 = f_mult(s2, ALPHA);

        let mut tmp: i32 = (s >> 1) - (s2 >> 1);
        tmp = ((buf_in[syn_pos + j] as i32 - buf_out[ltp_pos + j] as i32) << 14)
            + f_mult(gain, tmp as FixpDbl);
        buf[lpcorder as usize - 1 - j] = tmp;
    }

    zir.fill(0);

    clpc_synthesis_sgl(zir, 1, a, a_e, lpcorder, &mut buf, &mut st_idx);

    let length = zir.len() as i32;
    let mut alpha = MAXVAL_SGL as FixpSgl;
    let step = (MAXVAL_SGL / (length >> 1)) as FixpSgl;

    for z in &mut zir[(length >> 1) as usize..] {
        *z = f_mult(*z, alpha);
        alpha = (alpha as i32 - step as i32) as FixpSgl;
    }
}

/// `ltp_synth_filter(synth_ltp, synth, length, pitch_int, pitch_fr, gain, gainIdx, pitch_res,
/// zir, fade, filtIdx)`
///
/// `synth_ltp` is `buf_out[out_pos..]`, `synth` is `buf_in[in_pos..]`. `fade`: 0 = normal,
/// +1 = fade-in, -1 = fade-out.
#[allow(clippy::too_many_arguments)]
fn ltp_synth_filter(
    buf_out: &mut [FixpSgl],
    out_pos: usize,
    buf_in: &[FixpSgl],
    in_pos: usize,
    length: i32,
    pitch_int: i32,
    pitch_fr: i32,
    mut gain: FixpSgl,
    gain_idx: i32,
    pitch_res: i32,
    zir: Option<&[FixpDbl]>,
    fade: i32,
    filt_idx: i16,
) {
    if gain > 0 {
        /* x0 = &synth_ltp[-pitch_int], x1 = x0 - 1, y0 = synth */
        let x_start = out_pos as isize - pitch_int as isize;
        let y_start = in_pos as isize;

        debug_assert!(filt_idx >= 0);
        debug_assert!((length & 3) == 0);

        let f = &AAC_LTP_FILTERS[filt_idx as usize];
        let f2 = &AAC_LTP_FILTERS2[filt_idx as usize];
        let w0 = &f.filt[pitch_fr as usize..];
        let w1 = &f.filt[(pitch_res - pitch_fr) as usize..];
        let l = f.length;
        let l2 = f2.length;
        let v0 = &f2.filt[(gain_idx * l2) as usize..];

        let mut alpha: FixpDbl = if fade <= 0 { MINVAL_DBL } else { 0 };
        let step: FixpDbl = if length == 384 {
            fade * ONE_BY_384 /* +/- 0x0055.5555 */
        } else if length == 512 {
            fade << (6 + 16) /* +/- 0x0040.0000 */
        } else {
            /* if (length == 128) */
            fade << (8 + 16) /* +/- 0x0100.0000 */
        };
        gain = -gain;

        for j in 0..length as usize {
            let x0 = x_start + j as isize;
            let y0 = y_start + j as isize;
            let mut s: FixpDbl = 0;
            let mut s2: FixpDbl = 0;

            let mut k = 0usize;
            for i in 0..l as isize {
                s += f_mult_div2(w0[k], buf_out[(x0 + i) as usize])
                    + f_mult_div2(w1[k], buf_out[(x0 - 1 - i) as usize]);
                k += pitch_res as usize;
            }

            for i in 0..l2 as isize {
                s2 += f_mult_div2(v0[i as usize], buf_in[(y0 - i) as usize]);
            }

            s2 = f_mult_div2(s2, ALPHA);
            let mut tmp: FixpDbl = (s >> 1) - s2;
            tmp = ((buf_in[in_pos + j] as i32) << 13) + f_mult_div2(f_mult(gain, alpha), tmp);
            if let Some(zir) = zir {
                tmp -= zir[j] >> 1;
            }
            buf_out[out_pos + j] = saturate_right_shift(tmp, 13, FRACT_BITS) as FixpSgl;
            alpha -= step;
        }
    } else {
        let len = length as usize;
        buf_out[out_pos..out_pos + len].copy_from_slice(&buf_in[in_pos..in_pos + len]);
    }
}

/// `ltp_decode_params(ltp_param, pitch_int, pitch_fr, gain, pitmin, pitfr1, pitfr2, pitmax,
/// pitres)`: decode pitch and gain.
#[allow(clippy::too_many_arguments)]
fn ltp_decode_params(
    ltp_param: &[i32; 3],
    pitch_int: &mut i32,
    pitch_fr: &mut i32,
    gain: &mut FixpSgl,
    pitmin: i32,
    pitfr1: i32,
    pitfr2: i32,
    _pitmax: i32,
    pitres: i32,
) {
    let gainbits = 2;

    /* Decode Pitch and Gain */
    if ltp_param[0] != 0 {
        if ltp_param[1] < ((pitfr2 - pitmin) * pitres) {
            *pitch_int = pitmin + (ltp_param[1] / pitres);
            *pitch_fr = ltp_param[1] - (*pitch_int - pitmin) * pitres;
        } else if ltp_param[1] < ((pitfr2 - pitmin) * pitres + (pitfr1 - pitfr2) * (pitres >> 1)) {
            *pitch_int = pitfr2 + ((ltp_param[1] - (pitfr2 - pitmin) * pitres) / (pitres >> 1));
            *pitch_fr =
                (ltp_param[1] - (pitfr2 - pitmin) * pitres) - (*pitch_int - pitfr2) * (pitres >> 1);
            *pitch_fr <<= 1; /* was *= (pitres>>1); */
        } else {
            *pitch_int = ltp_param[1] + pitfr1
                - ((pitfr2 - pitmin) * pitres)
                - ((pitfr1 - pitfr2) * (pitres >> 1));
            *pitch_fr = 0;
        }

        if ltp_param[2] >= 3 {
            *gain = MAXVAL_SGL as FixpSgl;
        } else {
            *gain = (((ltp_param[2] + 1) as FixpSgl as i32) << (15 - gainbits)) as FixpSgl;
        }
    } else {
        *pitch_int = 0;
        *pitch_fr = 0;
        *gain = 0;
    }
}

/// `readLtpParam(hBs, ltp_param, isLFE)`: parse the LTPF parameters.
///
/// Returns 1 if LTPF data is present in an LFE channel element (an error), 0 otherwise.
pub fn read_ltp_param(hbs: &mut FdkBitStream, ltp_param: &mut [i32; 3], is_lfe: i32) -> i32 {
    ltp_param[0] = hbs.read_bit() as i32; /* ltpf_data_present */
    if ltp_param[0] != 0 {
        if is_lfe != 0 {
            return 1;
        }
        ltp_param[1] = hbs.read_bits(9) as i32; /* ltpf_pitch_lag_index */
        ltp_param[2] = hbs.read_bits(2) as i32; /* ltpf_gain_index */
    }
    0
}

/// `ltp_post(sig, L_frame, fs, ltp_param, pitch_int_past, pitch_fr_past, gain_past,
/// gainIdx_past, mem_in, mem_out)`: apply the LTP postfilter to `sig` (`L_frame` values) in
/// place.
///
/// `mem_in` holds [`LTP_MEM_IN_SIZE`] and `mem_out` [`LTP_MEM_OUT_SIZE`] values. Note that
/// `ltp_param[2]` is used (and stored in `gain_idx_past`) even if `ltp_param[0]` is 0.
///
/// In the pass-through case the C++ code reads `sig[L_frame - LTP_MEM_OUT_SIZE..]`, i.e. it
/// requires `L_frame >= LTP_MEM_OUT_SIZE` (MPEG-H always uses `L_frame == 1024`); this port
/// panics otherwise.
#[allow(clippy::too_many_arguments)]
pub fn ltp_post(
    sig: &mut [FixpDbl],
    l_frame: i32,
    fs: i32,
    ltp_param: &[i32; 3],
    pitch_int_past: &mut i32,
    pitch_fr_past: &mut i32,
    gain_past: &mut FixpSgl,
    gain_idx_past: &mut i32,
    mem_in: &mut [FixpSgl],
    mem_out: &mut [FixpSgl],
) {
    let lf = l_frame as usize;

    /* Check if ltp post filter can run in pass-through mode */
    if *gain_past == 0 && ltp_param[0] == 0 {
        debug_assert!(lf >= LTP_MEM_OUT_SIZE); /* LTP_MEM_OUT_SIZE > LTP_MEM_IN_SIZE */

        /* Update mem_in states */
        scale_values_saturate_dbl_to_sgl(
            &mut mem_in[..LTP_MEM_IN_SIZE],
            &sig[lf - LTP_MEM_IN_SIZE..lf],
            PCM_OUT_HEADROOM,
        );

        /* Update mem_out states */
        scale_values_saturate_dbl_to_sgl(
            &mut mem_out[..LTP_MEM_OUT_SIZE],
            &sig[lf - LTP_MEM_OUT_SIZE..lf],
            PCM_OUT_HEADROOM,
        );

        /* Update past values */
        *gain_past = 0;
        *gain_idx_past = 0;
        *pitch_fr_past = 0;
        *pitch_int_past = 0;

        return;
    }

    let mut zir = [0 as FixpDbl; L_FRAME_1024 / 4];
    let mut a = [0 as FixpSgl; LTP_LPC_ORDER + 1];
    let mut a_e = 0;
    /* need +3 for xtensa optimisation (length divisible by 4) */
    let mut buf_in = [0 as FixpSgl; L_FRAME_1024 + LTP_MEM_IN_SIZE + TCXLTP_MAX_DELAY + 3];
    let mut buf_out = [0 as FixpSgl; L_FRAME_1024 + LTP_MEM_OUT_SIZE];
    let mut pitch_int = 0;
    let mut pitch_fr = 0;
    let mut gain: FixpSgl = 0;

    /******** Init ********/

    /* Parameters */
    let filt_idx = 0;
    let enc_dec_delay = l_frame >> 1;
    let l_transition = l_frame / 8;
    let lpcorder = LTP_LPC_ORDER as i32;
    let i = ((((fs / 2) * PIT_MIN_12K8) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - PIT_MIN_12K8;
    let pit_min = PIT_MIN_12K8 + i;
    let pit_fr2 = PIT_FR2_12K8 - i;
    let pit_fr1 = PIT_FR1_12K8;
    let pit_max = PIT_MAX_12K8 + (6 * i);
    let pitres = 4;

    /* Input buffer: sig_in = buf_in + LTP_MEM_IN_SIZE */
    let sig_in = LTP_MEM_IN_SIZE;
    buf_in[..LTP_MEM_IN_SIZE].copy_from_slice(&mem_in[..LTP_MEM_IN_SIZE]);
    scale_values_saturate_dbl_to_sgl(
        &mut buf_in[sig_in..sig_in + lf],
        &sig[..lf],
        PCM_OUT_HEADROOM,
    );
    mem_in[..LTP_MEM_IN_SIZE].copy_from_slice(&buf_in[sig_in + lf - LTP_MEM_IN_SIZE..sig_in + lf]);

    /* Output buffer: sig_out = buf_out + LTP_MEM_OUT_SIZE */
    let sig_out = LTP_MEM_OUT_SIZE;
    buf_out[..LTP_MEM_OUT_SIZE].copy_from_slice(&mem_out[..LTP_MEM_OUT_SIZE]);

    /* LTP parameters: integer pitch, fractional pitch, gain */
    ltp_decode_params(
        ltp_param,
        &mut pitch_int,
        &mut pitch_fr,
        &mut gain,
        pit_min,
        pit_fr1,
        pit_fr2,
        pit_max,
        pitres,
    );
    let mut tmp = pitch_int * pitres + pitch_fr;
    tmp *= 2;
    pitch_int = tmp / pitres;
    pitch_fr = tmp % pitres;
    gain >>= 2;

    let edd = enc_dec_delay as usize;
    let lt = l_transition as usize;

    /******** Previous-frame part ********/
    if enc_dec_delay != 0 {
        ltp_synth_filter(
            &mut buf_out,
            sig_out,
            &buf_in,
            sig_in,
            enc_dec_delay,
            *pitch_int_past,
            *pitch_fr_past,
            *gain_past,
            *gain_idx_past,
            pitres,
            None,
            0,
            filt_idx,
        );
    }

    /******** Transition part ********/
    if gain == 0 && *gain_past == 0 {
        buf_out[sig_out + edd..sig_out + edd + lt]
            .copy_from_slice(&buf_in[sig_in + edd..sig_in + edd + lt]);
    } else if *gain_past == 0 {
        ltp_synth_filter(
            &mut buf_out,
            sig_out + edd,
            &buf_in,
            sig_in + edd,
            l_transition,
            pitch_int,
            pitch_fr,
            gain,
            ltp_param[2],
            pitres,
            None,
            1,
            filt_idx,
        );
    } else if gain == 0 {
        ltp_synth_filter(
            &mut buf_out,
            sig_out + edd,
            &buf_in,
            sig_in + edd,
            l_transition,
            *pitch_int_past,
            *pitch_fr_past,
            *gain_past,
            *gain_idx_past,
            pitres,
            None,
            -1,
            filt_idx,
        );
    } else if gain == *gain_past && pitch_int == *pitch_int_past && pitch_fr == *pitch_fr_past {
        ltp_synth_filter(
            &mut buf_out,
            sig_out + edd,
            &buf_in,
            sig_in + edd,
            l_transition,
            pitch_int,
            pitch_fr,
            gain,
            ltp_param[2],
            pitres,
            None,
            0,
            filt_idx,
        );
    } else {
        let lpc_len = (l_frame >> 2) as usize;
        ltp_get_lpc(&buf_out[sig_out + edd - lpc_len..sig_out + edd], &mut a, &mut a_e, lpcorder);

        ltp_get_zir(
            &mut zir[..lt],
            &buf_out,
            sig_out + edd - lpcorder as usize,
            &buf_in,
            sig_in + edd - lpcorder as usize,
            &a,
            a_e,
            lpcorder,
            gain,
            ltp_param[2],
            pitch_int,
            pitch_fr,
            pitres,
            filt_idx as i32,
        );

        ltp_synth_filter(
            &mut buf_out,
            sig_out + edd,
            &buf_in,
            sig_in + edd,
            l_transition,
            pitch_int,
            pitch_fr,
            gain,
            ltp_param[2],
            pitres,
            Some(&zir[..lt]),
            0,
            filt_idx,
        );
    }

    /******** Current-frame part ********/
    ltp_synth_filter(
        &mut buf_out,
        sig_out + edd + lt,
        &buf_in,
        sig_in + edd + lt,
        l_frame - enc_dec_delay - l_transition,
        pitch_int,
        pitch_fr,
        gain,
        ltp_param[2],
        pitres,
        None,
        0,
        filt_idx,
    );

    /******** Output ********/

    /* copy to output */
    for (s, &o) in sig[..lf].iter_mut().zip(&buf_out[sig_out..sig_out + lf]) {
        *s = fx_sgl2fx_dbl(o) >> PCM_OUT_HEADROOM;
    }

    /* Update */
    *pitch_int_past = pitch_int;
    *pitch_fr_past = pitch_fr;
    *gain_past = gain;
    *gain_idx_past = ltp_param[2];
    mem_out[..LTP_MEM_OUT_SIZE].copy_from_slice(&buf_out[lf..lf + LTP_MEM_OUT_SIZE]);
}
