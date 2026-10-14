// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_formatConverter.h,
// libFormatConverter/include/FDK_formatConverter_api.h and
// libFormatConverter/src/FDK_formatConverter.cpp.
// formatConverterGetNumInputChans() and formatConverterGetNumOutputChans() are declared but not
// implemented in C++; they are not ported.
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
// Fraunhofer IIS retains full right to modify and use the code for its own purpose,
// assign or donate the code to a third party and to inhibit third parties from using
// the code for products that do not conform to MPEG-related ITU Recommendations and/or
// ISO/IEC International Standards.
//
// This copyright notice must be included in all copies or derivative works.
//
// Copyright (c) ISO/IEC 2013.

//! Format converter API: channel matching, open/init/close of the parameters, post-processing
//! and setting of downmix matrices and bitstream equalizers.

use crate::dmx_matrix_parser::{EqParamsStruct, PkFilterParamsStruct};
use crate::format_converter_active_dmx_stft::active_dmx_set_aes;
use crate::format_converter_data::*;
use crate::format_converter_init::*;
use crate::format_converter_intern::*;
use crate::format_converter_lib::*;
use mpeghdec_fdk::cicp2geometry::{cicp2geometry_get_number_of_lfes, Cicp2GeometryChannelGeometry};
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_add_norm, f_div_norm_high_prec, sqrt_fixp};
use mpeghdec_fdk::scale::{scale_value, scale_value_saturate};

/* default parameters */
pub const FORMAT_CONVERTER_MAX_CHANNELS: u32 = 28;

/// `api_chid_t`: list of channels known to the format converter DMX matrix generator.
pub type ApiChid = i32;
/* horizontal channels */
pub const API_CH_M_000: i32 = 0;
pub const API_CH_M_L015: i32 = 1;
pub const API_CH_M_R015: i32 = 2;
pub const API_CH_M_L030: i32 = 3;
pub const API_CH_M_R030: i32 = 4;
pub const API_CH_M_L045: i32 = 5;
pub const API_CH_M_R045: i32 = 6;
pub const API_CH_M_L060: i32 = 7;
pub const API_CH_M_R060: i32 = 8;
pub const API_CH_M_L090: i32 = 9;
pub const API_CH_M_R090: i32 = 10;
pub const API_CH_M_L110: i32 = 11;
pub const API_CH_M_R110: i32 = 12;
pub const API_CH_M_L135: i32 = 13;
pub const API_CH_M_R135: i32 = 14;
pub const API_CH_M_L150: i32 = 15;
pub const API_CH_M_R150: i32 = 16;
pub const API_CH_M_180: i32 = 17;
/* height channels */
pub const API_CH_U_000: i32 = 18;
pub const API_CH_U_L045: i32 = 19;
pub const API_CH_U_R045: i32 = 20;
pub const API_CH_U_L030: i32 = 21;
pub const API_CH_U_R030: i32 = 22;
pub const API_CH_U_L090: i32 = 23;
pub const API_CH_U_R090: i32 = 24;
pub const API_CH_U_L110: i32 = 25;
pub const API_CH_U_R110: i32 = 26;
pub const API_CH_U_L135: i32 = 27;
pub const API_CH_U_R135: i32 = 28;
pub const API_CH_U_180: i32 = 29;
/* top channel */
pub const API_CH_T_000: i32 = 30;
/* low channels */
pub const API_CH_L_000: i32 = 31;
pub const API_CH_L_L045: i32 = 32;
pub const API_CH_L_R045: i32 = 33;
/* low frequency effects */
pub const API_CH_LFE1: i32 = 34;
pub const API_CH_LFE2: i32 = 35;
/* empty channel */
pub const API_CH_EMPTY: i32 = -1;

/// `formatConverterSetInOutFormat(fcInt, inout, numChannels, channel_vector)`: set input format
/// (`inout == 0`) or output format (`inout == 1`) as list of channels.
pub fn format_converter_set_in_out_format(
    fc_int: &mut IisFormatConverterInternal,
    inout: i32,
    num_channels: i32,
    channel_vector: &[AfcFormatConverterChannelId],
) {
    debug_assert!(inout == 0 || inout == 1);
    converter_set_inout_format(fc_int, inout, num_channels, channel_vector);
}

/// `azel` of `formatConverterMatchChConfig2Channels()`: azi, ele, azstart, azend, elstart, elend.
static AZEL: [[i32; 6]; (CH_LFE2 + 1) as usize] = [
    [0, 0, -7, 7, -9, 20],
    [22, 0, 8, 22, -9, 20],
    [-22, 0, -22, -8, -9, 20],
    [30, 0, 23, 37, -9, 20],
    [-30, 0, -37, -23, -9, 20],
    [45, 0, 38, 52, -9, 20],
    [-45, 0, -52, -38, -9, 20],
    [60, 0, 53, 75, -9, 20],
    [-60, 0, -75, -53, -9, 20],
    [90, 0, 76, 100, -45, 20],
    [-90, 0, -100, -76, -45, 20],
    [110, 0, 101, 124, -45, 20],
    [-110, 0, -124, -101, -45, 20],
    [135, 0, 125, 142, -45, 20],
    [-135, 0, -142, -125, -45, 20],
    [150, 0, 143, 157, -45, 20],
    [-150, 0, -157, -143, -45, 20],
    [180, 0, 158, -158, -45, 20],
    [0, 35, -10, 10, 21, 60],
    [45, 35, 38, 66, 21, 60],
    [-45, 35, -66, -38, 21, 60],
    [30, 35, 11, 37, 21, 60],
    [-30, 35, -37, -11, 21, 60],
    [90, 35, 67, 100, 21, 60],
    [-90, 35, -100, -67, 21, 60],
    [110, 35, 101, 124, 21, 60],
    [-110, 35, -124, -101, 21, 60],
    [135, 35, 125, 157, 21, 60],
    [-135, 35, -157, -125, 21, 60],
    [180, 35, 158, -158, 21, 60],
    [0, 90, -180, 180, 61, 90],
    [0, -15, -10, 10, -45, -10],
    [45, -15, 11, 75, -45, -10],
    [-45, -15, -75, -11, -45, -10],
    [45, 0, 0, 0, -20, 20],  /* LFE1 */
    [-45, 0, 0, 0, -20, 20], /* LFE2 */
];

/// `formatConverterMatchChConfig2Channels(chConfig, inout, numChannels, channel_vec,
/// numUnknownCh, unknownCh_vec)`: match input (`inout == 0`) / output (`inout == 1`) channel
/// configurations given by channel geometry data to channel labels known to the format converter
/// DMX matrix generator. Returns 0 on success, 1 if a known output channel is used twice.
pub fn format_converter_match_ch_config2_channels(
    ch_config: &[Cicp2GeometryChannelGeometry],
    inout: i32,
    num_channels: i32,
    channel_vec: &mut [AfcFormatConverterChannelId],
    num_unknown_ch: &mut u32,
    unknown_ch_vec: &mut [i32],
) -> i32 {
    let mut lfe_idx = [0usize; FDK_FORMAT_CONVERTER_MAX_LFE];
    let nch = num_channels.max(0) as usize;

    /* no known channel used in the beginning */
    let mut channel_used = [0i32; CH_LFE1 as usize];

    /* count number of LFEs */
    let mut num_lfes = 0usize;
    for k in 0..nch {
        if ch_config[k].lfe == 1 {
            num_lfes += 1;
        }
    }

    /* all channels are unknown at the beginning */
    *num_unknown_ch = num_channels as u32;

    /* ==== match non-LFE channels to known channels ==== */
    for k in 0..nch {
        /* initialize channel k as unknown channel, 0 offset */
        channel_vec[k] = CH_EMPTY;
        if ch_config[k].lfe == 0 {
            /* <== non-LFEs only */
            /* search for matching sector. sectors are non-overlapping, thus only
            1 sector may match (but multiple unknowns may match the 1 known). */
            for l in 0..CH_LFE1 as usize {
                let mut az = ch_config[k].az; /* read az inside l-loop since we may overwrite it below */
                let el = ch_config[k].el;
                let azstart = AZEL[l][2];
                let mut azend = AZEL[l][3];
                let elstart = AZEL[l][4];
                let elend = AZEL[l][5];
                if azstart > 0 && azend < 0 {
                    /* +-180 deg. wrap around */
                    azend += 360;
                    if az < 0 {
                        az = az.wrapping_add(360);
                    }
                }
                if az >= azstart && az <= azend && el >= elstart && el <= elend {
                    /* multiple use of known channels only allowed in input setup */
                    if channel_used[l] == 0 || inout == 0 {
                        channel_vec[k] = l as i32;
                        channel_used[l] = 1;
                        *num_unknown_ch = num_unknown_ch.wrapping_sub(1);
                    } else {
                        return 1;
                    }
                }
            }
        }
    } /* end of non-LFE matching */

    /* ==== match LFE channel(s) ==== */
    /* find the indices of the numLfes LFE(s) in the input config */
    if num_lfes > 0 {
        let mut l = 0;
        for k in 0..nch {
            if ch_config[k].lfe == 1 {
                lfe_idx[l] = k;
                l += 1;
            }
        }
    }

    /*
    LFE channel in config: => assign rule +azimuth -> LFE1
                                          -azimuth -> LFE2
    */
    for l in 0..num_lfes {
        /* loop over LFEs in config */
        let az = ch_config[lfe_idx[l]].az % 360;
        if (az >= 0 && az < 180) || (az < -180 && az >= -360) {
            channel_vec[lfe_idx[l]] = CH_LFE1;
        } else {
            channel_vec[lfe_idx[l]] = CH_LFE2;
        }
        *num_unknown_ch = num_unknown_ch.wrapping_sub(1);
    }

    /* signal list of the (*numUnknownCh) unknown channels to the caller, ignore unmatched LFEs */
    if *num_unknown_ch != 0 {
        let mut l = 0;
        for k in 0..nch {
            if channel_vec[k] == CH_EMPTY {
                unknown_ch_vec[l] = k as i32;
                l += 1;
            }
        }
    }

    0
}

/// `formatConverterOpen(formatConverterMode, fcInt)`: allocate the format converter parameter
/// and state structs. Returns 0 on success, otherwise 1.
pub fn format_converter_open(
    _format_converter_mode: IisFormatconverterMode,
    fc_int: &mut IisFormatConverterInternal,
) -> i32 {
    let mut status = 0;

    /* allocation */
    fc_int.fc_params = Some(Box::default());
    fc_int.fc_state = Some(Box::default());

    /* check input parameters */

    /* delay */
    fc_int.fc_params.as_mut().unwrap().format_converter_delay =
        format_converter_get_delay_samples(fc_int.mode);

    /* generic formats (for use with external DMXmtx) */
    if fc_int.fc_input_format == FDK_FORMAT_CONVERTER_INPUT_FORMAT_GENERIC
        && fc_int.fc_output_format != FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC
    {
        status = 1;
    }

    if fc_int.fc_output_format == FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC
        && fc_int.fc_input_format != FDK_FORMAT_CONVERTER_INPUT_FORMAT_GENERIC
    {
        status = 1;
    }

    /* input format */
    match fc_int.fc_input_format {
        FDK_FORMAT_CONVERTER_INPUT_FORMAT_LISTOFCHANNELS
        | FDK_FORMAT_CONVERTER_INPUT_FORMAT_GENERIC => {}
        _ => {
            fc_int.fc_input_format = FDK_FORMAT_CONVERTER_INPUT_FORMAT_INVALID;
            status = 1;
        }
    }

    /* output format */
    match fc_int.fc_output_format {
        FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_LISTOFCHANNELS
        | FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC => {}
        _ => {
            fc_int.fc_output_format = FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_INVALID;
            status = 1;
        }
    }

    /* sampling rate */
    if fc_int.sampling_rate < 8000 || fc_int.sampling_rate > 384000 {
        fc_int.sampling_rate = 0;
        status = 1;
    }

    /* on error clean up so far allocated structs and return */
    if status == 1 {
        fc_int.fc_params = None;
        fc_int.fc_state = None;
        return 1;
    }

    /* buffer allocation */
    if allocate_format_converter_params(fc_int) != 0 {
        status = 1;
    }

    if status != 0 {
        /* clean up */
        format_converter_close(fc_int);
    }

    status
}

/// `formatConverterInit(fcInt, centerFrequenciesNormalized, p_buffer)`: initialize the format
/// converter parameters (downmix matrix, EQs) and states. Returns 0 on success, otherwise an
/// error code (`converter_status_t` or 1 / -1).
pub fn format_converter_init(
    fc_int: &mut IisFormatConverterInternal,
    center_frequencies_normalized: Option<&'static [FixpDbl]>,
) -> i32 {
    debug_assert!(fc_int.fc_params.is_some());
    debug_assert!(fc_int.fc_state.is_some());

    /* set format converter parameters */
    let mut error_flag = set_format_converter_params(center_frequencies_normalized, fc_int);
    if error_flag != 0 {
        return 1;
    }

    if fc_int.fc_params.as_ref().unwrap().generic_io_fmt == 0 {
        /* format converter init internal */
        error_flag = format_converter_init_internal(fc_int);
        if error_flag != 0 {
            return error_flag;
        } else {
            fc_int.fc_params.as_mut().unwrap().dmx_mtx_is_set = 1;
        }
    } else {
        fc_int.fc_params.as_mut().unwrap().dmx_mtx_is_set = 0;
    }

    /* set format converter states */
    error_flag = set_format_converter_state(fc_int);
    if error_flag != 0 {
        return -1;
    }

    0
}

/// `formatConverterClose(fcInt)`: free the format converter parameter and state structs.
pub fn format_converter_close(fc_int: &mut IisFormatConverterInternal) {
    if fc_int.fc_state.is_some() {
        /* free buffers */
        free_format_converter_state(fc_int);
        fc_int.fc_state = None;
    }
    if fc_int.fc_params.is_some() {
        free_format_converter_params(fc_int);
        fc_int.fc_params = None;
    }
}

/// `formatConverterPostprocessDmxMtx(dmxMtx, numInputChans, numOutputChans)`: applies a
/// threshold to MTX gains to avoid excessive use of phantom sources and normalizes the gains to
/// preserve the energy. Returns 0 on success, 1 for a `NULL` matrix.
pub fn format_converter_postprocess_dmx_mtx(
    dmx_mtx: Option<&mut [Vec<FixpDmxH>]>,
    num_input_chans: u32,
    num_output_chans: u32,
) -> i32 {
    /* threshold for setting DMX gains to zero: 0.3=-10.5dB (approx 3/4 panpot) */
    let thr: FixpDmxH = fl2fxconst_sgl(0.3f32 as f64);

    let Some(dmx_mtx) = dmx_mtx else { return 1 };
    let nout = num_output_chans as usize;

    /* post-process gains for each input channel individually */
    for i in 0..num_input_chans as usize {
        let row = &mut dmx_mtx[i];
        /* post-process only if there is at least one gain
        larger than the threshold for the current input channel */
        let mut max_gain: FixpDmxH = 0;
        for j in 0..nout {
            if row[j] > max_gain {
                max_gain = row[j];
            }
        }
        if max_gain <= thr {
            continue;
        }

        /* calculate energy before post-processing */
        let mut orig_ene: FixpDbl = 0;
        let mut orig_ene_e = 0;
        for j in 0..nout {
            /* max output channels 24 */
            orig_ene = f_add_norm(orig_ene, orig_ene_e, f_pow2(row[j]), 0, &mut orig_ene_e);
        }

        /* apply threshold */
        for j in 0..nout {
            /* max output channels 24 */
            if row[j] <= thr {
                row[j] = 0;
            }
        }

        /* calculate power after modification */
        let mut mod_ene: FixpDbl = 0;
        let mut mod_ene_e = 0;
        for j in 0..nout {
            mod_ene = f_add_norm(mod_ene, mod_ene_e, f_pow2(row[j]), 0, &mut mod_ene_e);
        }

        /* normalize gains */
        let mut e = 0;
        let tmp = f_div_norm_high_prec(orig_ene, mod_ene, &mut e);
        e += orig_ene_e - mod_ene_e;
        let norm_factor;
        if e & 1 != 0 {
            norm_factor = sqrt_fixp(tmp >> 1);
            e += 1;
            e >>= 1;
        } else {
            norm_factor = sqrt_fixp(tmp);
            e >>= 1;
        }
        for j in 0..nout {
            row[j] = fx_dbl2fx_dmx_h(scale_value(f_mult(norm_factor, row[j]), e));
        }
    }
    0
}

/// `formatConverterAddDmxMtx(dmxMtx, fcInt)`: add a post-processed DMX gains matrix to the
/// downmix matrix. Returns 0 on success, otherwise 1.
pub fn format_converter_add_dmx_mtx(
    dmx_mtx: Option<&[Vec<FixpDmxH>]>,
    fc_int: &mut IisFormatConverterInternal,
) -> i32 {
    let mut num_out_lfes = 0u32;
    let mut lfe_index: i32 = -1;

    cicp2geometry_get_number_of_lfes(
        &fc_int.out_channel_geo,
        fc_int.num_output_channels,
        &mut num_out_lfes,
    );

    let Some(dmx_mtx) = dmx_mtx else { return 1 };

    if num_out_lfes == 1 {
        for out in 0..fc_int.num_output_channels as usize {
            if fc_int.out_channel_geo[out].lfe != 0 {
                lfe_index = out as i32;
            }
        }
    } /* End: ( numOutLfes == 1 ) */

    let nin = fc_int.num_total_input_channels as usize;
    let nout = fc_int.num_output_channels as usize;
    let params = fc_int.fc_params.as_mut().unwrap();
    if params.dmx_mtx_is_set != 1 {
        return 1;
    }

    /* add external dmx matrix: element-wise addition
    NOTE: the number of input/output channels must NOT change!
    */
    for i in 0..nin {
        for j in 0..nout {
            if lfe_index != -1 && fc_int.input_channel_geo[i].lfe != 0 {
                continue;
            }
            /* add update terms to current downmix matrix */
            params.dmx_mtx[i][j] = params.dmx_mtx[i][j].wrapping_add(dmx_mtx[i][j]);
        }
    }

    /* Resort the dmxMatrix in order to be applied in the downmix processing function.  */
    let mut j = 0;
    for column in 0..nin {
        for row in 0..nout {
            /* add update terms to sorted downmix matrix */
            params.dmx_mtx_sorted[j] = params.dmx_mtx[column][row];
            j += 1;
        }
    }

    0
}

/// `formatConverterSetDmxMtx(fcInt)`: set the sorted DMX matrix based on the DMX matrix.
pub fn format_converter_set_dmx_mtx(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let nin = fc_int.num_total_input_channels as usize;
    let nout = fc_int.num_output_channels as usize;
    let params = fc_int.fc_params.as_mut().unwrap();

    /* Resort the dmxMatrix in order to be applied in the downmix processing function.  */
    let mut j = 0;
    for column in 0..nin {
        for row in 0..nout {
            params.dmx_mtx_sorted[j] = params.dmx_mtx[column][row];
            j += 1;
        }
    }

    /* DMX matrix is valid now */
    params.dmx_mtx_is_set = 1;

    0
}

/// `formatConverterSetEQs(eqIndex_sorted, numEQs, eqParams, eqMap, grp, fcInt, p_buffer)`: set
/// the bitstream EQs of signal group `grp`.
///
/// `eqIndex_sorted` is `&fcInt->fcParams->eqIndexVec_sorted[eq_index_offset]` (the C++ caller
/// passes a pointer into that array); `eq_params` holds the `numEQs` EQ definitions (`None` for
/// `NULL`) and `eq_map` indicates which EQ to apply to which input channel of the group (0 = no
/// EQ). Returns 0 on success, otherwise -1.
pub fn format_converter_set_eqs(
    eq_index_offset: usize,
    num_eqs: u8,
    eq_params: Option<&[EqParamsStruct]>,
    eq_map: &[u32],
    grp: u32,
    fc_int: &mut IisFormatConverterInternal,
) -> i32 {
    /* Maximum number of EQs is the max input channels */
    let mut eq_gains_tmp = [[0 as FixpDbl; 58]; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS];
    let mut eq_gains_tmp_e = [[0i32; 58]; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS];

    if num_eqs as usize > FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS {
        return -1;
    }

    let n_grp = fc_int.num_input_channels[grp as usize] as usize;
    for i in 0..n_grp {
        if eq_map[i] > num_eqs as u32 {
            return -1;
        }
    }

    if allocate_format_converter_eqs(fc_int) == -1 {
        return -1;
    }

    /* calculate numEQs EQ responses from the numEQs eqParamsStructs */
    for n in 0..num_eqs as usize {
        for k in 0..fc_int.stft_num_erb_bands as usize {
            let mut tmp_sfreq_hz = fc_int.sampling_rate as FixpDbl;
            let headroom = f_normz(tmp_sfreq_hz) - 1;
            tmp_sfreq_hz = tmp_sfreq_hz.wrapping_shl(headroom as u32);
            let f = f_mult(f_abs(fc_int.fc_center_frequencies.unwrap()[k]), tmp_sfreq_hz);
            let f_e = 32 - headroom - 2;
            let Some(eq_params) = eq_params else { continue };
            /* These EQ parameters were parsed and decoded in EqualizerConfig() */
            /* init with first peak filter and global gain G[dB] */
            let pk: &PkFilterParamsStruct = &eq_params[n].pk_filter_params[0];
            eq_gains_tmp_e[n][k] = 0;
            eq_gains_tmp[n][k] = peak_filter(
                pk.f,
                pk.f_e as i32,
                pk.q,
                pk.q_e as i32,
                pk.g,
                pk.g_e as i32,
                eq_params[n].g,
                eq_params[n].g_e as i32,
                f,
                f_e,
                &mut eq_gains_tmp_e[n][k],
            );
            /* apply remaining filters of peak filter cascade with 0dB global gain */
            for m in 1..eq_params[n].n_pk_filter as usize {
                let pk = &eq_params[n].pk_filter_params[m];
                let pf = peak_filter(
                    pk.f,
                    pk.f_e as i32,
                    pk.q,
                    pk.q_e as i32,
                    pk.g,
                    pk.g_e as i32,
                    0,
                    0,
                    f,
                    f_e,
                    &mut eq_gains_tmp_e[n][k],
                );
                eq_gains_tmp[n][k] = f_mult(eq_gains_tmp[n][k], pf);
            }
        }
    }

    /* apply EQs to freq. dep. dmx matrix. This freq. dep. matrix is calculated on the fly in the
     * active downmix processing. */
    /* The EQs from the bitstream parser are in Q29 format (exponent equals to 2), Meaning that
     * 0.25 is actually 1. */

    let nout = fc_int.num_output_channels as usize;
    for m in 0..n_grp {
        /* m is the input channel index for the current signal group equalizers */
        /* i is the input channel index for the total of equalizers taking into account every
         * signal group which parses the eq and dmx matrix info. */
        let i = m + fc_int.amount_of_added_dmx_matrices_and_equalizers as usize;
        let eq_idx = eq_map[m] as i32;
        let gains = fc_int.eq_gains[i].as_mut().unwrap();
        if eq_idx > 0 {
            /* apply an EQ to this input channel */
            for k in 0..fc_int.stft_num_erb_bands as usize {
                gains[k] = scale_value_saturate(
                    eq_gains_tmp[(eq_idx - 1) as usize][k],
                    eq_gains_tmp_e[(eq_idx - 1) as usize][k] - EQ_BITSTREAM_H_EXP,
                );
            }
        } else {
            /* no EQ applied to this input channel - simply apply dmxMtx */
            for k in 0..fc_int.stft_num_erb_bands as usize {
                gains[k] = FIXP_EQ_BITSTREAM_H_FORMAT_1_DOT_0; /*fcInt->fcParams->dmxMtx[i][j];*/
            }
        }
        let eq_index_sorted = &mut fc_int.fc_params.as_mut().unwrap().eq_index_vec_sorted;
        for n in 0..nout {
            /* EQ index for the activeDmxProcess_STFT function */
            eq_index_sorted[eq_index_offset + m * nout + n] = i as i32; /* taking into account the 20 equalizers which are stored by default as defined in activeDmxProcess_STFT. RULE_18_11 is the last eq for the immersive mode.  */
        }
    }

    0
}

/// `formatConverterGetDelaySamples(mode)`: processing delay of the format converter in samples,
/// -1 for invalid modes.
pub fn format_converter_get_delay_samples(mode: IisFormatconverterMode) -> i32 {
    match mode {
        IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_PHASE_ALIGN
        | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_PHASE_ALIGN => 4096,
        IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN
        | IIS_FORMATCONVERTER_MODE_PASSIVE_FREQ_DOMAIN => 0,
        IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT => 256, /* same as HOPSIZE */
        _ => -1,
    }
}

/// `setCustomDownmixParameters(fcInt, adaptiveEQStrength, phaseAlignStrength)`
pub fn set_custom_downmix_parameters(
    fc_int: &mut IisFormatConverterInternal,
    adaptive_eq_strength: i32,
    _phase_align_strength: i32,
) -> i32 {
    if fc_int.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT {
        let h = fc_int.fc_state.as_mut().unwrap().handle_active_dmx_stft.as_mut().unwrap();
        active_dmx_set_aes(adaptive_eq_strength, h);
    }
    0
}
