// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_formatConverter_data.h and
// libFormatConverter/src/FDK_formatConverter_data.cpp.
// setFormatConverterParamsPreAlloc(), formatConverterSetInOutFormat_internal() and
// formatConverterSetInOutFormat_internal() are declared but not implemented in C++.
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

//! Allocation and initialization of the format converter parameters and states, generation of
//! the downmix matrices from the downmix rules and selection of the matrices for processing.

use crate::format_converter_active_dmx_stft::*;
use crate::format_converter_constants_stft::STFT_ERB_BANDS;
use crate::format_converter_init::*;
use crate::format_converter_intern::*;
use crate::format_converter_lib::*;
use mpeghdec_fdk::common_fix::*;

/// `setFormatConverterParams(centerFrequenciesNormalized, fcInt)`: set data of the format
/// converter parameter struct. Returns 0.
pub fn set_format_converter_params(
    center_frequencies_normalized: Option<&'static [FixpDbl]>,
    fc_int: &mut IisFormatConverterInternal,
) -> i32 {
    let status = 0;
    let nin = fc_int.num_total_input_channels as usize;
    let nout = fc_int.num_output_channels as usize;
    let mode = fc_int.mode;
    let params = fc_int.fc_params.as_mut().unwrap();

    /* freq domain params */
    if mode != IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN {
        if mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
            || mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
        {
            params.center_frequencies_normalized = center_frequencies_normalized;
        } else {
            params.center_frequencies_normalized = None;
        }
    }

    /* init DMX matrix */
    for i in 0..nin {
        for j in 0..nout {
            params.dmx_mtx[i][j] = 0;
        }
    }

    params.dmx_mtx_is_set = 0;

    status
}

/// `allocateFormatConverterParams(fcInt)`: allocate the parameter struct members (zeroed).
pub fn allocate_format_converter_params(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let nin = fc_int.num_total_input_channels as usize;
    let nout = fc_int.num_output_channels as usize;
    let params = fc_int.fc_params.as_mut().unwrap();

    /* dmx mtx */
    params.dmx_mtx = vec![vec![0; nout]; nin];
    /* dmx mtx sorted */
    params.dmx_mtx_sorted = vec![0; nin * nout];
    /* dmx mtx L */
    params.dmx_mtx_l = vec![vec![0; nout]; nin];
    /* dmx mtx L sorted */
    params.dmx_mtx_l_sorted = vec![0; nin * nout];
    /* dmx mtx 2 */
    params.dmx_mtx2 = vec![vec![0; nout]; nin];
    /* dmx mtx 2 sorted */
    params.dmx_mtx2_sorted = vec![0; nin * nout];
    /* Eq index vector */
    params.eq_index_vec = vec![vec![0; nout]; nin];
    /* Eq index vector 2 */
    params.eq_index_vec2 = vec![vec![0; nout]; nin];
    /* Eq index vector sorted */
    params.eq_index_vec_sorted = vec![0; nin * nout];
    /* Eq index vector sorted 2 */
    params.eq_index_vec2_sorted = vec![0; nin * nout];
    /* internal structure */
    params.format_converter_params_internal = Some(Box::default());

    0
}

/// `allocateFormatConverterEQs(fcInt)`: allocate the bitstream EQ gains (if not yet done).
pub fn allocate_format_converter_eqs(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let num_input_channels = fc_int.num_total_input_channels as usize;

    /* eqGains */
    if fc_int.eq_gains[0].is_none() {
        for i in 0..num_input_channels {
            fc_int.eq_gains[i] = Some(vec![0; STFT_ERB_BANDS]);
        }
    }

    0
}

/// `freeFormatConverterParams(fcInt)`: free the parameter struct members.
///
/// Unlike C, the freed `eqGains` pointers are reset (C leaves them dangling; they are never used
/// again in the decoder).
pub fn free_format_converter_params(fc_int: &mut IisFormatConverterInternal) {
    if let Some(params) = fc_int.fc_params.as_mut() {
        params.dmx_mtx = Vec::new();
        params.dmx_mtx_sorted = Vec::new();
        params.dmx_mtx_l = Vec::new();
        params.dmx_mtx_l_sorted = Vec::new();
        params.dmx_mtx2 = Vec::new();
        params.dmx_mtx2_sorted = Vec::new();
        params.eq_index_vec = Vec::new();
        params.eq_index_vec_sorted = Vec::new();
        params.eq_index_vec2 = Vec::new();
        params.eq_index_vec2_sorted = Vec::new();
        params.format_converter_params_internal = None;
    }

    /* eqGains */
    for g in fc_int.eq_gains.iter_mut() {
        *g = None;
    }
}

/// `setFormatConverterState(fcInt)`: initialize the active downmix of the STFT modes. Returns 0
/// on success, otherwise -1.
pub fn set_format_converter_state(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let error_flag1 = 0;
    let mut error_flag2 = 0;

    /* init phase align dmx */
    if fc_int.mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        || fc_int.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
    /* IIS_FORMATCONVERTER_MODE_PASSIVE_FREQ_DOMAIN_STFT: This only depends on the aes
    parameter, 7 for active and 0 for passive. */
    {
        let state = fc_int.fc_state.as_mut().unwrap();
        error_flag2 = active_dmx_stft_init(
            &mut state.handle_active_dmx_stft,
            fc_int.num_total_input_channels,
            fc_int.num_output_channels,
            fc_int.aes,
        );
    }

    if error_flag1 != 0 || error_flag2 != 0 {
        -1
    } else {
        0
    }
}

/// `freeFormatConverterState(fcInt)`
pub fn free_format_converter_state(fc_int: &mut IisFormatConverterInternal) {
    if let Some(state) = fc_int.fc_state.as_mut() {
        /* free stft dmx */
        if fc_int.mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
            || fc_int.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
        {
            active_dmx_close_stft(&mut state.handle_active_dmx_stft);
        }
    }
}

/// `formatConverterInit_internal(fcInt, p_buffer)`: generate the downmix matrices and EQ index
/// matrices from the downmix rules (`converter_init()`). Returns 0 on success, 1 for invalid
/// formats or the `converter_init()` error code.
pub fn format_converter_init_internal(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let mut params_tmp = Box::<ConverterPrTmp>::default();

    let mut converter_state = FORMAT_CONVERTER_STATUS_OK;

    /* input format */
    match fc_int.fc_input_format {
        FDK_FORMAT_CONVERTER_INPUT_FORMAT_LISTOFCHANNELS => {
            fc_int.fc_params.as_mut().unwrap().format_converter_input_format_internal =
                FDK_FORMAT_IN_LISTOFCHANNELS;
        }
        _ => return 1,
    }

    /* output format */
    match fc_int.fc_output_format {
        FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_LISTOFCHANNELS => {
            fc_int.fc_params.as_mut().unwrap().format_converter_output_format_internal =
                FDK_FORMAT_OUT_LISTOFCHANNELS;
        }
        _ => return 1,
    }

    /* init internal structure */
    if fc_int.mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        || fc_int.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
        || fc_int.mode == IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN
    {
        let params = fc_int.fc_params.as_mut().unwrap();
        let input_format = params.format_converter_input_format_internal;
        let output_format = params.format_converter_output_format_internal;
        let bands_nrm = params.center_frequencies_normalized;
        let mut pr = params.format_converter_params_internal.take().unwrap();
        let sfreq = fc_int.sampling_rate as i32;
        let blocklength = fc_int.frame_size as i32;
        let nbands = fc_int.stft_num_erb_bands;
        converter_state = converter_init(
            fc_int,
            &mut pr,
            &mut params_tmp,
            input_format,
            output_format,
            None,
            sfreq,
            blocklength,
            nbands,
            bands_nrm.unwrap_or(&[]),
        );
        fc_int.fc_params.as_mut().unwrap().format_converter_params_internal = Some(pr);
    }

    if converter_state != 0 {
        return converter_state;
    }

    let nin = fc_int.num_total_input_channels as usize;
    let nout = fc_int.num_output_channels as usize;
    let immersive = fc_int.immersive_downmix_flag != 0 && (nout == 6 || nout == 5);
    let stft_mode = fc_int.mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        || fc_int.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT;
    let params = fc_int.fc_params.as_mut().unwrap();
    let t = &*params_tmp;

    {
        /* generate dmx matrix */
        if nout != 1 {
            let mut i = 0;
            while t.in_out_src[i] >= 0 {
                params.dmx_mtx[t.in_out_src[i] as usize][t.in_out_dst[i] as usize] =
                    t.in_out_gain[i];
                i += 1;
            }
        } else {
            for i in 0..nin {
                params.dmx_mtx[i][0] = 0;
            }
            let mut i = 0;
            while t.in_out_src[i] >= 0 {
                let s = t.in_out_src[i] as usize;
                params.dmx_mtx[s][0] = params.dmx_mtx[s][0].wrapping_add(t.in_out_gain[i]);
                i += 1;
            }
        }
    }
    if immersive {
        /* generate dmx matrix gainL*/
        let mut i = 0;
        while t.in_out_src[i] >= 0 {
            let (s, d) = (t.in_out_src[i] as usize, t.in_out_dst[i] as usize);
            if t.in_out_gain_l[i] > -1 {
                params.dmx_mtx_l[s][d] = t.in_out_gain_l[i];
            } else {
                params.dmx_mtx_l[s][d] = t.in_out_gain[i];
            }
            i += 1;
        }

        /* generate dmx matrix 2 gain2*/
        let mut i = 0;
        while t.in_out_src2[i] >= 0 {
            params.dmx_mtx2[t.in_out_src2[i] as usize][t.in_out_dst2[i] as usize] =
                t.in_out_gain2[i];
            i += 1;
        }
    }

    if stft_mode {
        /**** generate eq index matrix ****/
        let mut i = 0;
        while t.in_out_src[i] >= 0 {
            params.eq_index_vec[t.in_out_src[i] as usize][t.in_out_dst[i] as usize] =
                t.in_out_proc[i] as i32;
            i += 1;
        }
        if immersive {
            /**** generate eq index matrix 2 ****/
            let mut i = 0;
            while t.in_out_src2[i] >= 0 {
                params.eq_index_vec2[t.in_out_src2[i] as usize][t.in_out_dst2[i] as usize] =
                    t.in_out_proc2[i] as i32;
                i += 1;
            }
        }
    }

    /* Resort the dmxMatrix in order to be applied in the passive downmix function
     * formatConverterProcess_passive_timeDomain_frameLength.  */
    let mut j = 0;
    for column in 0..nin {
        for row in 0..nout {
            params.eq_index_vec_sorted[j] = params.eq_index_vec[column][row];
            params.dmx_mtx_sorted[j] = params.dmx_mtx[column][row];
            if immersive {
                params.dmx_mtx_l_sorted[j] = params.dmx_mtx_l[column][row];
                params.eq_index_vec2_sorted[j] = params.eq_index_vec2[column][row];
                params.dmx_mtx2_sorted[j] = params.dmx_mtx2[column][row];
            }
            j += 1;
        }
    }

    0
}

/// `formatConverterDmxMatrixControl(fcInt)`: immersive mode control; selects the downmix
/// matrices and equalizer index vectors used by the processing.
pub fn format_converter_dmx_matrix_control(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let nout = fc_int.num_output_channels;
    let rendering3_d_type_flag = fc_int.rendering3_d_type_flag;
    let params = fc_int.fc_params.as_mut().unwrap();

    /*********************************************/
    /* Immersive mode initializer                */
    /*********************************************/
    params.immersive_mode = (fc_int.immersive_downmix_flag != 0 && (nout == 6 || nout == 5)) as i32;
    params.mode3_drendering = (rendering3_d_type_flag != 0 && params.immersive_mode != 0) as i32;
    params.mode2_drendering = (rendering3_d_type_flag == 0 && params.immersive_mode != 0) as i32;
    params.rendering3_d_type_flag_internal = rendering3_d_type_flag as i32;

    params.dmx_iterations = 1;

    /* Select right set of downmix matrices and equalizer vectors */
    if params.mode3_drendering != 0 {
        params.dmx_matrix_h_fdk = DmxMtxSel::Mtx;
        params.dmx_matrix_l_fdk = DmxMtxSel::MtxL;
        params.eq_index_fdk = EqIndexSel::Vec;
        params.dmx_iterations = 3;
    } else if params.mode2_drendering != 0 {
        params.dmx_matrix_l_fdk = DmxMtxSel::Mtx2;
        params.dmx_matrix_h_fdk = params.dmx_matrix_l_fdk; /* dummy */
        params.eq_index_fdk = EqIndexSel::Vec2;
    } else {
        params.dmx_matrix_l_fdk = DmxMtxSel::Mtx;
        params.dmx_matrix_h_fdk = params.dmx_matrix_l_fdk; /* dummy */
        params.eq_index_fdk = EqIndexSel::Vec;
    }

    0
}

/// `formatConverterDmxMatrixExponent(fcInt)`: calculates how many input channels have an impact
/// in each output channel and its exponential term in order to accumulate results.
pub fn format_converter_dmx_matrix_exponent(fc_int: &mut IisFormatConverterInternal) -> i32 {
    let nin = fc_int.num_total_input_channels as usize;
    let nout = fc_int.num_output_channels as usize;
    let params = fc_int.fc_params.as_mut().unwrap();

    for ch_out in 0..nout {
        params.ch_out_count[ch_out] = 0;
    }

    for ch_in in 0..nin {
        let o = ch_in * nout;
        for ch_out in 0..nout {
            if params.dmx_mtx_sorted[o + ch_out] != 0
                || params.dmx_mtx2_sorted[o + ch_out] != 0
                || params.dmx_mtx_l_sorted[o + ch_out] != 0
            {
                params.ch_out_count[ch_out] += 1;
            }
        }
    }

    for ch_out in 0..nout {
        if params.ch_out_count[ch_out] > 0 {
            params.ch_out_exp[ch_out] = 32 - cnt_leading_zeros(params.ch_out_count[ch_out] - 1);
        } else {
            params.ch_out_exp[ch_out] = 0;
        }
    }

    0
}
