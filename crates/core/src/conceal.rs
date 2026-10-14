// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/conceal.h and libMpeghDec/src/conceal.cpp.

//! Independent channel concealment (AAC core concealment).
//!
//! This AAC core implementation includes a concealment function. There are various tests inside
//! the core, starting with simple CRC tests and ending in a variety of plausibility checks. If
//! such a check indicates an invalid bitstream, then concealment is applied. Concealment is also
//! applied when the calling main program indicates a distorted or missing data frame using the
//! frameOK flag. This is used for error detection on the transport layer.
//!
//! The concealment modes used by the MPEG-H decoder are:
//!
//! 1. Muting: the spectral data is simply set to zero in case of an detected error.
//! 2. Noise substitution: in case of an detected error, concealment copies the last frame and
//!    attenuates the spectral data. Noise substitution adds no additional delay.
//!
//! Differences to the C++ interface:
//! - `CConcealmentInfo::pConcealParams` is not stored (see [`CConcealmentInfo`]); the functions
//!   that use it take the decoder's `concealCommonData` as `params: &CConcealParams`.
//! - `pAacDecoderChannelInfo->pSpectralCoefficient` is passed as a slice (`spectrum`, at least
//!   1024 values); `SPEC_LONG(ptr)` is the slice itself.
//! - `CConcealment_Store()`, `CConcealment_Apply()` and `CConcealment_TDFading()` do not take the
//!   static channel info: they (and the static helpers they call) only use its `concealmentInfo`,
//!   which is passed as `h_concealment_info`.

use crate::aac_rom::{
    PcmDec, AACDEC_RANDOM_SIGN, AAC_NF_NO_RANDOM_VAL, MAXVAL_PCM_DEC, MINVAL_PCM_DEC,
};
use crate::aacdecoder_lib::*;
use crate::channelinfo::{
    get_window_group_length_table, get_window_groups, get_window_sequence, get_window_shape,
    AacdecRenderMode, CAacDecoderChannelInfo, SamplingRateInfo, AACDEC_RENDER_IMDCT,
    AACDEC_RENDER_INVALID, AACDEC_RENDER_LPD,
};
use crate::conceal_types::*;
use mpeghdec_fdk::common_fix::{
    f_abs, f_max, f_mult, fl2fxconst_dbl, fl2fxconst_sgl, fx_dbl2fx_sgl, fx_sgl2fx_dbl, FixpDbl,
    FixpSgl, MAXVAL_DBL, MAXVAL_SGL, MINVAL_DBL,
};
use mpeghdec_fdk::fixpoint_math::f_ld_pow_noexp;
use mpeghdec_fdk::mdct::{BlockType, BLOCK_LONG, BLOCK_SHORT, BLOCK_START, BLOCK_STOP};

/// `AACDEC_CONCEAL_PARAM_NOT_SPECIFIED`
pub const AACDEC_CONCEAL_PARAM_NOT_SPECIFIED: i32 = 0xFFFE;

/// `CONCEAL_DFLT_COMF_NOISE_LEVEL`
const CONCEAL_DFLT_COMF_NOISE_LEVEL: FixpDbl = 0x100000;

/// `CONCEAL_NOT_DEFINED`: `(UCHAR)-1`
const CONCEAL_NOT_DEFINED: u8 = u8::MAX;

/* default settings */
/// `CONCEAL_DFLT_FADEOUT_FRAMES`
const CONCEAL_DFLT_FADEOUT_FRAMES: i32 = 6;
/// `CONCEAL_DFLT_FADEIN_FRAMES`
const CONCEAL_DFLT_FADEIN_FRAMES: i32 = 5;
/// `CONCEAL_DFLT_MUTE_RELEASE_FRAMES`
const CONCEAL_DFLT_MUTE_RELEASE_FRAMES: i32 = 0;

/// `FL2FXCONST_SGL(CONCEAL_DFLT_FADE_FACTOR)`: 1/sqrt(2) (a `float` literal)
#[allow(clippy::approx_constant, clippy::excessive_precision)] // the literal of the C++ code
const CONCEAL_DFLT_FADE_FACTOR_SGL: FixpSgl = fl2fxconst_sgl(0.707106781186548f32 as f64);

/* some often used constants: */
/// `FIXP_ONE`
const FIXP_ONE: FixpDbl = fl2fxconst_dbl(1.0);

/* For parameter conversion */
/// `CONCEAL_PARAMETER_BITS`
const CONCEAL_PARAMETER_BITS: i32 = 8;
/// `CONCEAL_MAX_QUANT_FACTOR`
const CONCEAL_MAX_QUANT_FACTOR: i32 = (1 << CONCEAL_PARAMETER_BITS) - 1;
/// `CONCEAL_MIN_ATTENUATION_FACTOR_025_LD`: ld of -0.25 dB
#[allow(clippy::excessive_precision)] // the literal of the C++ code
const CONCEAL_MIN_ATTENUATION_FACTOR_025_LD: FixpDbl =
    fl2fxconst_dbl(-0.041524101186092029596853445212299);

/// `CConcealment_GetWinSeq(prevWinSeq)`
fn cconcealment_get_win_seq(prev_win_seq: i32) -> BlockType {
    let mut new_win_seq = BLOCK_LONG;

    /* Try to have only long blocks */
    if prev_win_seq == BLOCK_START || prev_win_seq == BLOCK_SHORT {
        new_win_seq = BLOCK_STOP;
    }

    new_win_seq
}

/// Reads `fadeOutFactor[idx]` (`fade_in == false`) or `fadeInFactor[idx]` (`fade_in == true`)
/// like the C++ code does, i.e. an index past the end of `fadeOutFactor[]` reads `fadeInFactor[]`
/// (the arrays are adjacent in `CConcealParams`). This happens in `findEquiFadeFrame()` when the
/// fade-out counter has reached `CONCEAL_MAX_NUM_FADE_FACTORS` (possible with short blocks and
/// `numFadeOutFrames == 31`).
#[inline]
fn fade_factor(params: &CConcealParams, fade_in: bool, idx: i32) -> FixpSgl {
    let n = CONCEAL_MAX_NUM_FADE_FACTORS as i32;
    let i = if fade_in { idx + n } else { idx };
    if i < n {
        params.fade_out_factor[i as usize]
    } else {
        params.fade_in_factor[(i - n) as usize]
    }
}

/// `CConcealment_InitCommonData(pConcealCommonData)`: init common concealment information data.
pub fn cconcealment_init_common_data(p_conceal_common_data: &mut CConcealParams) {
    /* Set default error concealment technique */
    p_conceal_common_data.method = ConcealMethodNoise;

    p_conceal_common_data.num_fade_out_frames = CONCEAL_DFLT_FADEOUT_FRAMES;
    p_conceal_common_data.num_fade_in_frames = CONCEAL_DFLT_FADEIN_FRAMES;
    p_conceal_common_data.num_mute_release_frames = CONCEAL_DFLT_MUTE_RELEASE_FRAMES;

    p_conceal_common_data.comfort_noise_level = CONCEAL_DFLT_COMF_NOISE_LEVEL;

    /* Init fade factors (symetric) */
    p_conceal_common_data.fade_out_factor[0] = CONCEAL_DFLT_FADE_FACTOR_SGL;
    p_conceal_common_data.fade_in_factor[0] = p_conceal_common_data.fade_out_factor[0];

    for i in 1..CONCEAL_MAX_NUM_FADE_FACTORS {
        p_conceal_common_data.fade_out_factor[i] = fx_dbl2fx_sgl(f_mult(
            p_conceal_common_data.fade_out_factor[i - 1],
            CONCEAL_DFLT_FADE_FACTOR_SGL,
        ));
        p_conceal_common_data.fade_in_factor[i] = p_conceal_common_data.fade_out_factor[i];
    }
}

/// `CConcealment_GetMethod(pConcealCommonData)`: get current concealment method.
pub fn cconcealment_get_method(
    p_conceal_common_data: Option<&CConcealParams>,
) -> CConcealmentMethod {
    let mut method = ConcealMethodNone;

    if let Some(p) = p_conceal_common_data {
        method = p.method;
    }

    method
}

/// `CConcealment_InitChannelData(pConcealChannelInfo, pConcealCommonData, initRenderMode,
/// samplesPerFrame)`: init concealment information for each channel.
///
/// `p_conceal_common_data` is only stored as `pConcealParams` in C++ (see [`CConcealmentInfo`]);
/// `samples_per_frame` is unused.
pub fn cconcealment_init_channel_data(
    p_conceal_channel_info: &mut CConcealmentInfo,
    _p_conceal_common_data: &CConcealParams,
    init_render_mode: AacdecRenderMode,
    _samples_per_frame: i32,
) {
    let c = p_conceal_channel_info;
    c.td_noise_seed = 0;
    c.td_noise_states = [0; 3];
    c.td_noise_coef[0] = fl2fxconst_sgl(0.05f32 as f64);
    c.td_noise_coef[1] = fl2fxconst_sgl(0.5f32 as f64);
    c.td_noise_coef[2] = fl2fxconst_sgl(0.45f32 as f64);

    c.last_render_mode = init_render_mode as i8;

    c.window_shape = CONCEAL_NOT_DEFINED;
    c.window_sequence = BLOCK_LONG; /* default type */
    c.last_win_grp_len = 1;

    c.conceal_state = ConcealState_Ok;

    c.spectral_coefficient[..1024].fill(0);

    c.spec_scale = [0; 8];

    c.i_random_phase = 0;

    c.prev_frame_ok[0] = 1;
    c.prev_frame_ok[1] = 1;

    c.cnt_fade_frames = 0;
    c.cnt_valid_frames = 0;
    c.fade_old = MAXVAL_DBL;
    c.win_grp_offset[0] = 0;
    c.win_grp_offset[1] = 0;
    c.att_grp_offset[0] = 0;
    c.att_grp_offset[1] = 0;
}

/// `CConcealment_SetParams(concealParams, method, fadeOutSlope, fadeInSlope, muteRelease,
/// comfNoiseLevel)`: set error concealment parameters.
pub fn cconcealment_set_params(
    mut conceal_params: Option<&mut CConcealParams>,
    method: i32,
    fade_out_slope: i32,
    fade_in_slope: i32,
    mute_release: i32,
    comf_noise_level: FixpDbl,
) -> AacDecoderError {
    /* set concealment technique */
    if method != AACDEC_CONCEAL_PARAM_NOT_SPECIFIED {
        match method {
            ConcealMethodMute | ConcealMethodNoise => match conceal_params.as_deref_mut() {
                None => return AAC_DEC_INVALID_HANDLE,
                /* set param */
                Some(p) => p.method = method,
            },
            _ => return AAC_DEC_SET_PARAM_FAIL,
        }
    }

    /* set number of frames for fade-out slope */
    if fade_out_slope != AACDEC_CONCEAL_PARAM_NOT_SPECIFIED {
        if fade_out_slope < CONCEAL_MAX_NUM_FADE_FACTORS as i32 && fade_out_slope >= 0 {
            match conceal_params.as_deref_mut() {
                None => return AAC_DEC_INVALID_HANDLE,
                /* set param */
                Some(p) => p.num_fade_out_frames = fade_out_slope,
            }
        } else {
            return AAC_DEC_SET_PARAM_FAIL;
        }
    }

    /* set number of frames for fade-in slope */
    if fade_in_slope != AACDEC_CONCEAL_PARAM_NOT_SPECIFIED {
        if fade_in_slope < CONCEAL_MAX_NUM_FADE_FACTORS as i32 && fade_in_slope >= 0 {
            match conceal_params.as_deref_mut() {
                None => return AAC_DEC_INVALID_HANDLE,
                /* set param */
                Some(p) => p.num_fade_in_frames = fade_in_slope,
            }
        } else {
            return AAC_DEC_SET_PARAM_FAIL;
        }
    }

    /* set number of error-free frames after which the muting will be released */
    if mute_release != AACDEC_CONCEAL_PARAM_NOT_SPECIFIED {
        if mute_release < ((CONCEAL_MAX_NUM_FADE_FACTORS as i32) << 1) && mute_release >= 0 {
            match conceal_params.as_deref_mut() {
                None => return AAC_DEC_INVALID_HANDLE,
                /* set param */
                Some(p) => p.num_mute_release_frames = mute_release,
            }
        } else {
            return AAC_DEC_SET_PARAM_FAIL;
        }
    }

    /* set confort noise level which will be inserted while in state 'muting' */
    if comf_noise_level != AACDEC_CONCEAL_PARAM_NOT_SPECIFIED {
        if comf_noise_level < 0 {
            return AAC_DEC_SET_PARAM_FAIL;
        }
        match conceal_params {
            None => return AAC_DEC_INVALID_HANDLE,
            Some(p) => p.comfort_noise_level = comf_noise_level,
        }
    }

    AAC_DEC_OK
}

/// `CConcealment_SetAttenuation(concealParams, fadeOutAttenuationVector,
/// fadeInAttenuationVector)`: set fade-out/in attenuation factor vectors
/// (`CONCEAL_MAX_NUM_FADE_FACTORS` values each, `None` for `NULL`).
///
/// Returns 0 if OK, all other values indicate errors.
pub fn cconcealment_set_attenuation(
    mut conceal_params: Option<&mut CConcealParams>,
    fade_out_attenuation_vector: Option<&[i16]>,
    fade_in_attenuation_vector: Option<&[i16]>,
) -> AacDecoderError {
    if fade_out_attenuation_vector.is_none() && fade_in_attenuation_vector.is_none() {
        return AAC_DEC_SET_PARAM_FAIL;
    }

    /* Fade-out factors */
    if let Some(v) = fade_out_attenuation_vector {
        /* check quantized factors first */
        for &f in &v[..CONCEAL_MAX_NUM_FADE_FACTORS] {
            if f < 0 || f as i32 > CONCEAL_MAX_QUANT_FACTOR {
                return AAC_DEC_SET_PARAM_FAIL;
            }
        }
        let Some(p) = conceal_params.as_deref_mut() else {
            return AAC_DEC_INVALID_HANDLE;
        };

        /* now dequantize factors */
        for i in 0..CONCEAL_MAX_NUM_FADE_FACTORS {
            p.fade_out_factor[i] = fx_dbl2fx_sgl(f_ld_pow_noexp(
                CONCEAL_MIN_ATTENUATION_FACTOR_025_LD,
                0,
                (fl2fxconst_dbl(1.0 / 2.0) >> (CONCEAL_PARAMETER_BITS - 1)) * v[i] as i32,
                CONCEAL_PARAMETER_BITS,
            ));
        }
    }

    /* Fade-in factors */
    if let Some(v) = fade_in_attenuation_vector {
        /* check quantized factors first */
        for &f in &v[..CONCEAL_MAX_NUM_FADE_FACTORS] {
            if f < 0 || f as i32 > CONCEAL_MAX_QUANT_FACTOR {
                return AAC_DEC_SET_PARAM_FAIL;
            }
        }
        let Some(p) = conceal_params else {
            return AAC_DEC_INVALID_HANDLE;
        };

        /* now dequantize factors */
        for i in 0..CONCEAL_MAX_NUM_FADE_FACTORS {
            p.fade_in_factor[i] = fx_dbl2fx_sgl(f_ld_pow_noexp(
                CONCEAL_MIN_ATTENUATION_FACTOR_025_LD,
                0,
                (FIXP_ONE >> CONCEAL_PARAMETER_BITS) * v[i] as i32,
                CONCEAL_PARAMETER_BITS,
            ));
        }
    }

    AAC_DEC_OK
}

/// `CConcealment_GetState(pConcealChannelInfo)`: get state of concealment module.
pub fn cconcealment_get_state(
    p_conceal_channel_info: Option<&CConcealmentInfo>,
) -> CConcealmentState {
    let mut state = ConcealState_Ok;

    if let Some(c) = p_conceal_channel_info {
        state = c.conceal_state;
    }

    state
}

/// `CConcealment_Store(hConcealmentInfo, pAacDecoderChannelInfo, pAacDecoderStaticChannelInfo)`:
/// store data for concealment techniques applied later. Interface function to store data for
/// different concealment strategies.
///
/// `pAacDecoderStaticChannelInfo` is dropped (see the module documentation).
pub fn cconcealment_store(
    h_concealment_info: &mut CConcealmentInfo,
    params: &CConcealParams,
    p_aac_decoder_channel_info: &CAacDecoderChannelInfo,
    spectrum: &[FixpDbl],
) {
    if p_aac_decoder_channel_info.render_mode != AACDEC_RENDER_LPD {
        let p_spectral_coefficient = spectrum;
        let p_spec_scale = &p_aac_decoder_channel_info.spec_scale;

        /* store new scale factors */
        h_concealment_info.spec_scale = *p_spec_scale;

        if params.method < ConcealMethodInter {
            /* store new spectral bins */
            h_concealment_info.spectral_coefficient[..1024]
                .copy_from_slice(&p_spectral_coefficient[..1024]);
        }
    }

    if p_aac_decoder_channel_info.render_mode != AACDEC_RENDER_LPD {
        let p_ics_info = &p_aac_decoder_channel_info.ics_info;

        /* store new window infos */
        h_concealment_info.window_sequence = get_window_sequence(p_ics_info);
        h_concealment_info.window_shape = get_window_shape(p_ics_info);
        h_concealment_info.last_window_groups = get_window_groups(p_ics_info);
        h_concealment_info.last_window_group_length = *get_window_group_length_table(p_ics_info);
        let groups = get_window_groups(p_ics_info) as usize;
        h_concealment_info.last_win_grp_len = if groups > 0 {
            get_window_group_length_table(p_ics_info)[groups - 1]
        } else {
            /* WindowGroups == 0: the C++ code reads the byte before WindowGroupLength[], the high
            byte of the preceding specScale[7] of CAacDecoderChannelInfo (little-endian). */
            (p_aac_decoder_channel_info.spec_scale[7] as u16 >> 8) as u8
        };
    }
}

/// `CConcealment_Apply(hConcealmentInfo, pAacDecoderChannelInfo, pAacDecoderStaticChannelInfo,
/// pSamplingRateInfo, samplesPerFrame, lastLpdMode, frameOk, flags)`: apply concealment.
/// Interface function to different concealment strategies.
///
/// `pAacDecoderStaticChannelInfo` is dropped (see the module documentation);
/// `p_sampling_rate_info`, `last_lpd_mode` and `flags` are unused.
///
/// Returns -1 if the mute release is active, otherwise whether concealment processing was
/// applied.
#[allow(clippy::too_many_arguments)]
pub fn cconcealment_apply(
    h_concealment_info: &mut CConcealmentInfo,
    params: &CConcealParams,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    spectrum: &mut [FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    samples_per_frame: i32,
    _last_lpd_mode: u8,
    frame_ok: i32,
    flags: u32,
) -> i32 {
    let mut applied_processing = 0;
    let state_before = h_concealment_info.conceal_state;
    let mute_release_active = frame_ok != 0
        && h_concealment_info.conceal_state >= ConcealState_Mute
        && h_concealment_info.cnt_valid_frames < params.num_mute_release_frames;

    if h_concealment_info.window_shape == CONCEAL_NOT_DEFINED {
        /* Initialize window_shape with same value as in the current (parsed) frame.
        Because section 4.6.11.3.2 (Windowing and block switching) of ISO/IEC 14496-3:2009 says:
        For the first raw_data_block() to be decoded the window_shape of the left and right
        half of the window are identical. */
        h_concealment_info.window_shape = p_aac_decoder_channel_info.ics_info.window_shape;
    }

    if frame_ok != 0 && !mute_release_active {
        /* Update render mode if frameOk except for ongoing mute release state. */
        h_concealment_info.last_render_mode = p_aac_decoder_channel_info.render_mode as i8;

        /* Rescue current data for concealment in future frames */
        cconcealment_store(h_concealment_info, params, p_aac_decoder_channel_info, spectrum);
        /* Reset index to random sign vector to make sign calculation frame agnostic
        (only depends on number of subsequently concealed spectral blocks) */
        h_concealment_info.i_random_phase = 0;
    } else {
        if h_concealment_info.last_render_mode as i32 == AACDEC_RENDER_INVALID {
            h_concealment_info.last_render_mode = AACDEC_RENDER_IMDCT as i8;
        }
        p_aac_decoder_channel_info.render_mode = h_concealment_info.last_render_mode as i32;
    }

    /* hand current frame status to the state machine */
    cconcealment_update_state(
        h_concealment_info,
        params,
        frame_ok,
        samples_per_frame,
        p_aac_decoder_channel_info,
        spectrum,
    );

    {
        /* Create data for signal rendering according to the selected concealment method and
        decoder operating mode. */

        if p_aac_decoder_channel_info.render_mode != AACDEC_RENDER_LPD {
            match params.method {
                ConcealMethodNoise => {
                    /* Noise substitution error concealment technique */
                    applied_processing = cconcealment_apply_noise(
                        h_concealment_info,
                        params,
                        p_aac_decoder_channel_info,
                        spectrum,
                        p_sampling_rate_info,
                        samples_per_frame,
                        flags,
                    );
                }
                /* ConcealMethodMute and default */
                _ => {
                    if frame_ok == 0 {
                        /* Mute spectral data in case of errors */
                        spectrum[..samples_per_frame as usize].fill(0);
                        /* Set last window shape */
                        p_aac_decoder_channel_info.ics_info.window_shape =
                            h_concealment_info.window_shape;
                        applied_processing = 1;
                    }
                }
            }
        } else if frame_ok == 0 || mute_release_active {
            /* simply restore the buffer */
            let p_ics_info = &mut p_aac_decoder_channel_info.ics_info;

            p_ics_info.window_sequence = h_concealment_info.window_sequence;
            p_ics_info.window_shape = h_concealment_info.window_shape;

            if h_concealment_info.conceal_state != ConcealState_Mute {
                /* restore scale factors */
                p_aac_decoder_channel_info.spec_scale = h_concealment_info.spec_scale;

                /* restore spectral bins */
                spectrum[..1024].copy_from_slice(&h_concealment_info.spectral_coefficient[..1024]);
            } else {
                /* clear scale factors */
                p_aac_decoder_channel_info.spec_scale = [0; 8];

                /* clear buffer */
                spectrum[..1024].fill(0);
            }
        }
    }
    /* update history */
    h_concealment_info.prev_frame_ok[0] = h_concealment_info.prev_frame_ok[1];
    h_concealment_info.prev_frame_ok[1] = frame_ok;

    if frame_ok == 0
        || state_before != ConcealState_Ok
        || h_concealment_info.conceal_state != ConcealState_Ok
    {
        /* concealment uses state that is not part of the state digest */
        mpeghdec_fdk::digest::hazard();
    }

    if mute_release_active {
        -1
    } else {
        applied_processing
    }
}

/// `CConcealment_ApplyNoise()`: apply concealment noise substitution. In case of frame lost this
/// function produces a noisy frame with respect to the energies values of past frame.
fn cconcealment_apply_noise(
    p_concealment_info: &mut CConcealmentInfo,
    params: &CConcealParams,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    spectrum: &mut [FixpDbl],
    _p_sampling_rate_info: &SamplingRateInfo,
    samples_per_frame: i32,
    _flags: u32,
) -> i32 {
    let mut applied_processing = 0;

    match p_concealment_info.conceal_state {
        ConcealState_Ok => { /* Nothing to do here! */ }

        ConcealState_Single | ConcealState_FadeOut => {
            applied_processing = cconcealment_apply_fade_out(
                /*mode =*/ 1,
                p_concealment_info,
                params,
                samples_per_frame,
                p_aac_decoder_channel_info,
                spectrum,
            );
        }

        ConcealState_Mute => {
            let p_ics_info = &mut p_aac_decoder_channel_info.ics_info;
            /* set dummy window parameters */
            p_ics_info.valid = 0; /* Trigger the generation of a consitent IcsInfo */
            /* Prevent an invalid WindowShape (required for F/T transform) */
            p_ics_info.window_shape = p_concealment_info.window_shape;
            p_ics_info.window_sequence =
                cconcealment_get_win_seq(p_concealment_info.window_sequence);
            /* Store for next frame (spectrum in concealment buffer can't be used at all) */
            p_concealment_info.window_sequence = p_ics_info.window_sequence;
            p_concealment_info.win_grp_offset[0] = 0;
            p_concealment_info.win_grp_offset[1] = 0;

            /* mute spectral data */
            spectrum[..samples_per_frame as usize].fill(0);
            p_concealment_info.spectral_coefficient[..samples_per_frame as usize].fill(0);

            applied_processing = 1;
        }

        ConcealState_FadeIn => {
            /* TimeDomainFading:                                        */
            /* Attenuation of signal is done in CConcealment_TDFading() */

            applied_processing = 1;
        }

        _ => { /* we shouldn't come here anyway */ }
    }

    applied_processing
}

/// `findEquiFadeFrame(pConcealCommonData, actFadeIndex, direction)`: find next fading frame in
/// case of changing fading direction.
///
/// `act_fade_index` is the last index used for fading; `direction` is the direction of change:
/// 0: change from FADE-OUT to FADE-IN, 1: change from FADE-IN to FADE-OUT. This function
/// determines the next fading index to be used for the fading direction to be changed to.
fn find_equi_fade_frame(
    p_conceal_common_data: &CConcealParams,
    act_fade_index: i32,
    direction: i32,
) -> i32 {
    let reference_val: FixpSgl;
    let mut min_diff: FixpSgl = MAXVAL_SGL as FixpSgl;

    let mut next_fade_index = 0;

    /* init depending on direction */
    let p_factor: &[FixpSgl; CONCEAL_MAX_NUM_FADE_FACTORS] = if direction == 0 {
        /* FADE-OUT => FADE-IN */
        if act_fade_index < 0 {
            reference_val = MAXVAL_SGL as FixpSgl;
        } else {
            reference_val = fade_factor(p_conceal_common_data, false, act_fade_index) >> 1;
        }
        &p_conceal_common_data.fade_in_factor
    } else {
        /* FADE-IN => FADE-OUT */
        if act_fade_index < 0 {
            reference_val = MAXVAL_SGL as FixpSgl;
        } else {
            reference_val = fade_factor(p_conceal_common_data, true, act_fade_index) >> 1;
        }
        &p_conceal_common_data.fade_out_factor
    };

    /* search for minimum difference */
    for (i, &f) in p_factor.iter().enumerate() {
        let diff = f_abs((f as i32 >> 1) - reference_val as i32) as FixpSgl;
        if diff < min_diff {
            min_diff = diff;
            next_fade_index = i as i32;
        }
    }

    /* check and adjust depending on direction */
    if direction == 0 {
        /* FADE-OUT => FADE-IN */
        if (p_factor[next_fade_index as usize] >> 1) <= reference_val && next_fade_index > 0 {
            next_fade_index -= 1;
        }
        if next_fade_index > p_conceal_common_data.num_fade_in_frames - 1 {
            next_fade_index = f_max(p_conceal_common_data.num_fade_in_frames - 1, 0);
        }
    } else {
        /* FADE-IN => FADE-OUT */
        if (p_factor[next_fade_index as usize] >> 1) >= reference_val
            && next_fade_index < CONCEAL_MAX_NUM_FADE_FACTORS as i32 - 1
        {
            next_fade_index += 1;
        }
        if next_fade_index > p_conceal_common_data.num_fade_out_frames - 1 {
            next_fade_index = f_max(p_conceal_common_data.num_fade_out_frames - 1, 0);
        }
    }

    next_fade_index
}

/// `CConcealment_UpdateState()`: update the concealment state.
///
/// The function updates the state of the concealment state-machine. The states are: mute,
/// fade-in, fade-out, interpolate and frame-ok.
fn cconcealment_update_state(
    p_concealment_info: &mut CConcealmentInfo,
    p_conceal_common_data: &CConcealParams,
    frame_ok: i32,
    samples_per_frame: i32,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    spectrum: &mut [FixpDbl],
) {
    let c = p_concealment_info;
    let p = p_conceal_common_data;

    match p.method {
        ConcealMethodNoise => {
            if c.conceal_state != ConcealState_Ok {
                /* count the valid frames during concealment process */
                if frame_ok != 0 {
                    c.cnt_valid_frames += 1;
                } else {
                    c.cnt_valid_frames = 0;
                }
            }

            /* -- STATE MACHINE for Noise Substitution -- */
            match c.conceal_state {
                ConcealState_Ok => {
                    if frame_ok == 0 {
                        c.cnt_fade_frames = 0;
                        c.cnt_valid_frames = 0;
                        c.att_grp_offset[0] = 0;
                        c.att_grp_offset[1] = 0;
                        c.win_grp_offset[0] = 0;
                        c.win_grp_offset[1] = 0;
                        if p.num_fade_out_frames > 0 {
                            /* change to state SINGLE-FRAME-LOSS */
                            c.conceal_state = ConcealState_Single;
                            /* mode 0 just updates the Fading counter */
                            cconcealment_apply_fade_out(
                                /*mode =*/ 0,
                                c,
                                p,
                                samples_per_frame,
                                p_aac_decoder_channel_info,
                                spectrum,
                            );
                        } else {
                            /* change to state MUTE */
                            c.conceal_state = ConcealState_Mute;
                        }
                    }
                }

                ConcealState_Single => {
                    /* Just a pre-stage before fade-out begins. Stay here only one frame! */
                    if frame_ok != 0 {
                        /* change to state OK */
                        c.conceal_state = ConcealState_Ok;
                    } else if c.cnt_fade_frames >= p.num_fade_out_frames {
                        /* change to state MUTE */
                        c.conceal_state = ConcealState_Mute;
                    } else {
                        /* change to state FADE-OUT */
                        c.conceal_state = ConcealState_FadeOut;
                        /* mode 0 just updates the Fading counter */
                        cconcealment_apply_fade_out(
                            /*mode =*/ 0,
                            c,
                            p,
                            samples_per_frame,
                            p_aac_decoder_channel_info,
                            spectrum,
                        );
                    }
                }

                ConcealState_FadeOut => {
                    if c.cnt_valid_frames > p.num_mute_release_frames {
                        if p.num_fade_in_frames > 0 {
                            /* change to state FADE-IN */
                            c.conceal_state = ConcealState_FadeIn;
                            c.cnt_fade_frames = find_equi_fade_frame(
                                p,
                                c.cnt_fade_frames,
                                0, /* FadeOut -> FadeIn */
                            );
                        } else {
                            /* change to state OK */
                            c.conceal_state = ConcealState_Ok;
                        }
                    } else {
                        if frame_ok != 0 {
                            /* we have good frame information but stay fully in concealment -
                            reset winGrpOffset/attGrpOffset */
                            c.win_grp_offset[0] = 0;
                            c.win_grp_offset[1] = 0;
                            c.att_grp_offset[0] = 0;
                            c.att_grp_offset[1] = 0;
                        }
                        if c.cnt_fade_frames >= p.num_fade_out_frames {
                            /* change to state MUTE */
                            c.conceal_state = ConcealState_Mute;
                        } else {
                            /* Stay in FADE-OUT */
                            /* mode 0 just updates the Fading counter */
                            cconcealment_apply_fade_out(
                                /*mode =*/ 0,
                                c,
                                p,
                                samples_per_frame,
                                p_aac_decoder_channel_info,
                                spectrum,
                            );
                        }
                    }
                }

                ConcealState_Mute => {
                    if c.cnt_valid_frames > p.num_mute_release_frames {
                        if p.num_fade_in_frames > 0 {
                            /* change to state FADE-IN */
                            c.conceal_state = ConcealState_FadeIn;
                            c.cnt_fade_frames = p.num_fade_in_frames - 1;
                        } else {
                            /* change to state OK */
                            c.conceal_state = ConcealState_Ok;
                        }
                    } else if frame_ok != 0 {
                        /* we have good frame information but stay fully in concealment - reset
                        winGrpOffset/attGrpOffset */
                        c.win_grp_offset[0] = 0;
                        c.win_grp_offset[1] = 0;
                        c.att_grp_offset[0] = 0;
                        c.att_grp_offset[1] = 0;
                    }
                }

                ConcealState_FadeIn => {
                    c.cnt_fade_frames -= 1;
                    if frame_ok != 0 {
                        if c.cnt_fade_frames < 0 {
                            /* change to state OK */
                            c.conceal_state = ConcealState_Ok;
                        }
                    } else if p.num_fade_out_frames > 0 {
                        /* change to state FADE-OUT */
                        c.conceal_state = ConcealState_FadeOut;
                        c.cnt_fade_frames = find_equi_fade_frame(
                            p,
                            c.cnt_fade_frames + 1,
                            1, /* FadeIn -> FadeOut */
                        );
                        c.win_grp_offset[0] = 0;
                        c.win_grp_offset[1] = 0;
                        c.att_grp_offset[0] = 0;
                        c.att_grp_offset[1] = 0;

                        /* decrease because CConcealment_ApplyFadeOut() will increase,
                        accordingly */
                        c.cnt_fade_frames -= 1;
                        /* mode 0 just updates the Fading counter */
                        cconcealment_apply_fade_out(
                            /*mode =*/ 0,
                            c,
                            p,
                            samples_per_frame,
                            p_aac_decoder_channel_info,
                            spectrum,
                        );
                    } else {
                        /* change to state MUTE */
                        c.conceal_state = ConcealState_Mute;
                    }
                }

                _ => {}
            }
        }

        _ => { /* Don't need a state machine for other concealment methods. */ }
    }
}

/// `CConcealment_ApplyRandomSign(randomPhase, spec, samplesPerFrame)`: randomizes the sign of
/// the spectral data.
///
/// The function toggles the sign of the spectral data randomly. This is useful to ensure the
/// quality of the concealed frames.
fn cconcealment_apply_random_sign(
    mut random_phase: i32,
    spec: &mut [FixpDbl],
    samples_per_frame: i32,
) {
    /* random table 512x16bit has been reduced to 512 packed sign bits = 32x16 bit */

    /* read current packed sign word */
    let mut packed_sign: u16 = AACDEC_RANDOM_SIGN[(random_phase >> 4) as usize];
    packed_sign >>= random_phase & 0xf;

    for s in &mut spec[..samples_per_frame as usize] {
        if (random_phase & 0xf) == 0 {
            packed_sign = AACDEC_RANDOM_SIGN[(random_phase >> 4) as usize];
        }

        if packed_sign & 0x1 != 0 {
            *s = -f_max(*s, MINVAL_DBL + 1);
        }
        packed_sign >>= 1;

        random_phase = (random_phase + 1) & (AAC_NF_NO_RANDOM_VAL - 1);
    }
}

/// `CConcealment_GetLastFrameOk(hConcealmentInfo, fBeforeApply)`: frame OK flag of the previous
/// frame.
///
/// If called before the function `CConcealment_Apply()` set the `f_before_apply` flag to get the
/// correct value.
pub fn cconcealment_get_last_frame_ok(
    h_concealment_info: Option<&CConcealmentInfo>,
    f_before_apply: i32,
) -> i32 {
    let mut prev_frame_ok = 1;

    if let Some(c) = h_concealment_info {
        prev_frame_ok = c.prev_frame_ok[(f_before_apply & 0x1) as usize];
    }

    prev_frame_ok
}

/// `CConcealment_GetDelay(pConcealCommonData)`: number of delay frames introduced by the
/// concealment technique.
pub fn cconcealment_get_delay(p_conceal_common_data: Option<&CConcealParams>) -> u32 {
    let mut frame_delay = 0;

    if let Some(p) = p_conceal_common_data {
        match p.method {
            ConcealMethodTonal | ConcealMethodInter => frame_delay = 1,
            _ => {}
        }
    }

    frame_delay
}

/// `CConcealment_ApplyFadeOut(mode, pConcealmentInfo, pAacDecoderStaticChannelInfo,
/// samplesPerFrame, pAacDecoderChannelInfo)`
fn cconcealment_apply_fade_out(
    mode: i32,
    p_concealment_info: &mut CConcealmentInfo,
    params: &CConcealParams,
    samples_per_frame: i32,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    spectrum: &mut [FixpDbl],
) -> i32 {
    /* mode 1 = apply RandomSign and mute spectral coefficients if necessary,  *
     * mode 0 = Update cntFadeFrames                                            */

    /* restore frequency coefficients from buffer with a specific muting */
    let c = p_concealment_info;
    let mut num_windows = 1;
    let mut window_len = samples_per_frame;
    let mut src_grp_start = 0;
    let win_idx_stride = 1;

    let p_ics_info = &mut p_aac_decoder_channel_info.ics_info;
    let p_spec_scale = &mut p_aac_decoder_channel_info.spec_scale;

    /* set old window parameters */
    {
        p_ics_info.window_shape = c.window_shape;
        p_ics_info.window_sequence = c.window_sequence;

        if c.window_sequence == BLOCK_SHORT {
            /* short block handling */
            num_windows = 8;
            window_len = samples_per_frame >> 3;
            src_grp_start = num_windows - c.last_win_grp_len as i32;
        }
    }

    let att_idx_stride = f_max(1, num_windows / (c.last_win_grp_len as i32 + 1));

    /* load last state */
    let mut att_idx = c.cnt_fade_frames;
    let mut num_win_grp_per_fac = c.att_grp_offset[mode as usize];
    let mut src_win = src_grp_start + c.win_grp_offset[mode as usize];

    let wl = window_len as usize;
    for dst_win in 0..num_windows {
        let cncl_off = (src_win * window_len) as usize;
        let out_off = (dst_win * window_len) as usize;

        if mode == 1 {
            let p_cncl = &mut c.spectral_coefficient[cncl_off..cncl_off + wl];
            /* mute if attIdx gets large enaugh */
            if att_idx > params.num_fade_out_frames {
                p_cncl.fill(0);
            }

            /* restore frequency coefficients from buffer - attenuation is done later */
            let p_out = &mut spectrum[out_off..out_off + wl];
            p_out.copy_from_slice(p_cncl);

            /* apply random change of sign for spectral coefficients */
            cconcealment_apply_random_sign(c.i_random_phase, p_out, window_len);

            /* Increment random phase index to avoid repetition artifacts. */
            c.i_random_phase = (c.i_random_phase + 1) & (AAC_NF_NO_RANDOM_VAL - 1);

            /* set old scale factors */
            p_spec_scale[(dst_win * win_idx_stride) as usize] =
                c.spec_scale[(src_win * win_idx_stride) as usize];
        }

        src_win += 1;

        if src_win >= num_windows {
            /* end of sequence -> rewind to first window of group */
            src_win = src_grp_start;
            num_win_grp_per_fac += 1;
            if num_win_grp_per_fac >= att_idx_stride {
                num_win_grp_per_fac = 0;
                att_idx += 1;
            }
        }
    }

    /* store current state */

    c.win_grp_offset[mode as usize] = src_win - src_grp_start;
    c.att_grp_offset[mode as usize] = num_win_grp_per_fac;

    if mode == 0 {
        c.cnt_fade_frames = att_idx;
    }

    1
}

/// `CConcealment_TDFading(len, ppAacDecoderStaticChannelInfo, aacOutDataHeadroom, pcmdata,
/// pcmdata_1)`: do time domain fading (TDFading) in concealment case.
///
/// In case of concealment, this function takes care of the fading, after time domain signal has
/// been rendered by the respective signal rendering functions.
///
/// TimeDomain fading never creates jumps in energy / discontinuities, it always does a continuous
/// fading. To achieve this, fading is always done from a starting point to a target point, while
/// the starting point is always determined to be the last target point. By varying the target
/// point of a fading, the fading slope can be controlled.
///
/// This principle is applied to the fading within a frame and the fading from frame to frame.
///
/// One frame is divided into 8 subframes to obtain 8 parts of fading slopes within a frame, each
/// maybe with its own gradient.
///
/// Workflow:
/// 1. Determine Fading behavior and end-of-frame target fading level, based on concealmentState
///    (determined by `CConcealment_UpdateState()`) and the core mode.
///    - By _DEFAULT_, the target fading level is determined by `fadeOutFactor[cntFadeFrames]` in
///      case of fadeOut, or `fadeInFactor[cntFadeFrames]` in case of fadeIn. --> fading type is
///      `FADE_TIMEDOMAIN` in this case. Target fading level is determined by fading index
///      cntFadeFrames.
///    - If concealmentState is signalling a _MUTED SIGNAL_, TDFading decays to 0 within 1/8th of
///      a frame if `numFadeOutFrames == 0`. --> fading type is `FADE_TIMEDOMAIN_TOSPECTRALMUTE`
///      in this case.
///    - If concealmentState is signalling the _END OF MUTING_, TDFading fades to target fading
///      level within 1/8th of a frame if `numFadeInFrames == 0`. --> fading type is
///      `FADE_TIMEDOMAIN_FROMSPECTRALMUTE` in this case. Target fading level is determined by
///      fading index cntFadeFrames.
/// 2. Render fading levels within current frame and do the final fading: map fading slopes to
///    fading levels and apply to time domain signal.
///
/// `ppAacDecoderStaticChannelInfo` is dropped (see the module documentation); `pcmdata_1` is
/// unused (the only caller passes `NULL`).
pub fn cconcealment_td_fading(
    mut len: i32,
    h_concealment_info: &mut CConcealmentInfo,
    params: &CConcealParams,
    aac_out_data_headroom: i32,
    pcmdata: &mut [PcmDec],
) -> i32 {
    /*
    Do the fading in Time domain based on concealment states and core mode
    */
    let p_concealment_info = h_concealment_info;
    let p_conceal_params = params;
    if p_concealment_info.conceal_state != ConcealState_Ok
        || p_concealment_info.conceal_state_old != ConcealState_Ok
    {
        /* concealment uses state that is not part of the state digest */
        mpeghdec_fdk::digest::hazard();
    }
    let fade_stop: FixpDbl;
    let att_mute: FixpDbl = 0;
    let mut idx = 0;
    let conceal_state = p_concealment_info.conceal_state;
    let fading_type: TDfadingType;
    let mut fading_stations: [FixpDbl; 9] = [0; 9];
    let mut fading_steps: [i32; 8] = [0; 8];
    /* start fading at last end-of-frame attenuation */
    let fade_start = p_concealment_info.fade_old;
    let mut fade_factor_is_in = false; /* fadeFactor = pConcealParams->fadeOutFactor */
    let cnt_fade_frames = p_concealment_info.cnt_fade_frames;
    let td_fade_out_stop_before_mute = 1;
    let td_fade_in_stop_before_full_level = 1;

    /*
    determine Fading behaviour (end-of-frame attenuation and fading type) (1.)
    */

    match conceal_state {
        ConcealState_Single | ConcealState_Mute | ConcealState_FadeOut => {
            idx = if p_conceal_params.method == ConcealMethodNoise {
                cnt_fade_frames - 1
            } else {
                cnt_fade_frames
            };
            let mut t = FADE_TIMEDOMAIN;

            if conceal_state == ConcealState_Mute
                || (cnt_fade_frames + td_fade_out_stop_before_mute)
                    > p_conceal_params.num_fade_out_frames
            {
                t = FADE_TIMEDOMAIN_TOSPECTRALMUTE;
            }
            fading_type = t;
        }
        ConcealState_FadeIn | ConcealState_Ok => {
            if conceal_state == ConcealState_FadeIn {
                idx = cnt_fade_frames;
                idx -= td_fade_in_stop_before_full_level;
            }
            /* FDK_FALLTHROUGH */
            fade_factor_is_in = true;
            idx = if conceal_state == ConcealState_Ok { -1 } else { idx };
            fading_type = if p_concealment_info.conceal_state_old == ConcealState_Mute {
                FADE_TIMEDOMAIN_FROMSPECTRALMUTE
            } else {
                FADE_TIMEDOMAIN
            };
        }
        _ => {
            fading_type = FADE_TIMEDOMAIN_TOSPECTRALMUTE;
        }
    }

    /* determine Target end-of-frame fading level and fading slope */
    match fading_type {
        FADE_TIMEDOMAIN_FROMSPECTRALMUTE => {
            fade_stop = if idx < 0 {
                MAXVAL_DBL
            } else {
                fx_sgl2fx_dbl(fade_factor(p_conceal_params, fade_factor_is_in, idx))
            };
            if p_conceal_params.num_fade_in_frames == 0 {
                /* do step as fast as possible */
                fading_steps[0] = 1;
            } else {
                cconcealment_td_fading_do_linear_fading_steps(&mut fading_steps);
            }
        }
        FADE_TIMEDOMAIN => {
            fade_stop = if idx < 0 {
                MAXVAL_DBL
            } else {
                fx_sgl2fx_dbl(fade_factor(p_conceal_params, fade_factor_is_in, idx))
            };
            cconcealment_td_fading_do_linear_fading_steps(&mut fading_steps);
        }
        /* FADE_TIMEDOMAIN_TOSPECTRALMUTE */
        _ => {
            fade_stop = att_mute;
            if p_conceal_params.num_fade_out_frames == 0 {
                /* do step as fast as possible */
                fading_steps[0] = 1;
            } else {
                cconcealment_td_fading_do_linear_fading_steps(&mut fading_steps);
            }
        }
    }

    /*
    Render fading levels within current frame and do the final fading (2.)
    */

    len >>= 3;
    cconcealment_td_fade_fill_fading_stations(
        &mut fading_stations,
        &fading_steps,
        fade_stop,
        fade_start,
        fading_type,
    );

    if fading_stations.iter().any(|&s| s != MAXVAL_DBL)
    /* if there's something to fade */
    {
        let mut start = 0;
        for ii in 0..8 {
            cconcealment_td_fade_pcm_att(
                start,
                len,
                fading_stations[ii],
                fading_stations[ii + 1],
                pcmdata,
            );
            start += len;
        }
    }
    cconcealment_td_noise_apply(
        p_concealment_info,
        p_conceal_params,
        len,
        aac_out_data_headroom,
        pcmdata,
    );

    /* Save end-of-frame attenuation and fading type */
    p_concealment_info.last_fading_type = fading_type;
    p_concealment_info.fade_old = fade_stop;
    p_concealment_info.conceal_state_old = conceal_state;

    1
}

/// `CConcealment_TDFadePcmAtt(start, len, fadeStart, fadeStop, pcmdata)`: attenuate pcmdata in
/// Time Domain Fading process.
fn cconcealment_td_fade_pcm_att(
    start: i32,
    len: i32,
    fade_start: FixpDbl,
    fade_stop: FixpDbl,
    pcmdata: &mut [PcmDec],
) {
    /* set start energy */
    let mut d_gain = fade_start;
    /* determine energy steps from sample to sample */
    /* (len == 0: the loop below is empty, the division result is not used) */
    let d_step: FixpDbl =
        if len != 0 { (((fade_start >> 1) - (fade_stop >> 1)) / len) << 1 } else { 0 };

    for i in start..(start + len) {
        d_gain = d_gain.wrapping_sub(d_step);
        /* prevent gain from getting negative due to possible fixpoint inaccuracies */
        let d_gain_apply = f_max(0, d_gain);
        /* finally, attenuate samples */
        pcmdata[i as usize] = f_mult(pcmdata[i as usize], d_gain_apply);
    }
}

/// `CConcealment_TDFadeFillFadingStations(fadingStations, fadingSteps, fadeStop, fadeStart,
/// fadingType)`: fill FadingStations.
///
/// The fadingstations are the attenuation factors, being applied to its dedicated portions of pcm
/// data. They are calculated using the fadingsteps. One fadingstep is the weighted contribution
/// to the fading slope within its dedicated portion of pcm data.
///
/// ```text
/// *Fadingsteps  :      0  0  0  1  0  1  2  0
///
///                   |<-  1 Frame pcm data ->|
///       fadeStart-->|__________             |
///                   ^  ^  ^  ^ \____        |
///  Attenuation  :   |  |  |  |  ^  ^\__     |
///                   |  |  |  |  |  |  ^\    |
///                   |  |  |  |  |  |  | \___|<-- fadeStop
///                   |  |  |  |  |  |  |  ^  ^
///                   |  |  |  |  |  |  |  |  |
/// Fadingstations:  [0][1][2][3][4][5][6][7][8]
/// ```
///
/// (Fadingstations "`[0]`" is "`[8]` from previous frame", therefore its not meaningful to be edited)
fn cconcealment_td_fade_fill_fading_stations(
    fading_stations: &mut [FixpDbl; 9],
    fading_steps: &[i32; 8],
    fade_stop: FixpDbl,
    fade_start: FixpDbl,
    _fading_type: TDfadingType,
) {
    let fading_steps_sum: i32 = fading_steps.iter().sum();
    let fade_diff: i32 = fade_stop.wrapping_sub(fade_start) / f_max(fading_steps_sum, 1);
    fading_stations[0] = fade_start;
    for i in 1..8 {
        fading_stations[i] =
            fading_stations[i - 1].wrapping_add(fade_diff.wrapping_mul(fading_steps[i - 1]));
    }
    fading_stations[8] = fade_stop;
}

/// `CConcealment_TDFading_doLinearFadingSteps(fadingSteps)`
fn cconcealment_td_fading_do_linear_fading_steps(fading_steps: &mut [i32; 8]) {
    *fading_steps = [1; 8];
}

/* end of TimeDomainFading functions */

/// `CConcealment_TDNoise_Random(seed)`: derived from `int UsacRandomSign()`.
fn cconcealment_td_noise_random(seed: &mut u32) -> i32 {
    *seed = ((*seed as u64) * 69069 + 5) as u32;
    *seed as i32
}

/// `CConcealment_TDNoise_Apply(pConcealmentInfo, len, aacOutDataHeadroom, pcmdata)`
fn cconcealment_td_noise_apply(
    p_concealment_info: &mut CConcealmentInfo,
    params: &CConcealParams,
    len: i32,
    aac_out_data_headroom: i32,
    pcmdata: &mut [PcmDec],
) {
    let c = p_concealment_info;
    c.td_noise_seed = (cconcealment_td_noise_random(&mut c.td_noise_seed) as u32).wrapping_add(1);
    let mut seed = c.td_noise_seed;
    let states = &mut c.td_noise_states;
    let coef = &c.td_noise_coef;

    let td_noise_att: FixpDbl = params.comfort_noise_level;

    if (c.conceal_state != ConcealState_Ok || c.conceal_state_old != ConcealState_Ok)
        && td_noise_att != 0
    {
        /* the seed is not part of the state digest */
        mpeghdec_fdk::digest::hazard();
        for ii in 0..(len << 3) as usize {
            /* create filtered noise */
            states[2] = states[1];
            states[1] = states[0];
            states[0] = cconcealment_td_noise_random(&mut seed);
            let noise_val_long: FixpDbl = f_mult(states[0], coef[0])
                .wrapping_add(f_mult(states[1], coef[1]))
                .wrapping_add(f_mult(states[2], coef[2]));
            let mut noise_val: PcmDec =
                f_mult(noise_val_long, td_noise_att).wrapping_shr(aac_out_data_headroom as u32);

            /* add filtered noise - check for clipping, before */
            if (noise_val > 0 && pcmdata[ii] > MAXVAL_PCM_DEC - noise_val)
                || (noise_val < 0 && pcmdata[ii] < MINVAL_PCM_DEC - noise_val)
            {
                noise_val = noise_val.wrapping_mul(-1);
            }

            pcmdata[ii] = pcmdata[ii].wrapping_add(noise_val);
        }
    }
}
