// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/channel.h and libMpeghDec/src/channel.cpp.

//! Channel element parser (`CChannelElement_Read()`) and decoder (`CChannelElement_Decode()`).
//!
//! Both functions work on the channels of one element. The frame level calls them like
//! `CAacDecoder_DecodeFrame()` does, with the channel arrays starting at the first channel of the
//! element and a [`CChannelElementMem`] holding the memory the C++ code reaches through the
//! pointers of the channel structures (see the [crate documentation](crate)).

use crate::aacdec_tns::{
    ctns_read, ctns_read_data_present_flag, ctns_read_data_present_usac, ctns_reset,
};
use crate::aacdecoder_lib::*;
use crate::block::*;
use crate::channelinfo::*;
use crate::fdp::fdp_decode_bins;
use crate::ltp_post::read_ltp_param;
use crate::stereo::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::{f_max, f_min, FixpDbl};
use mpeghdec_fdk::mdct::{BLOCK_LONG, BLOCK_SHORT, BLOCK_START};
use mpeghdec_fdk::thread_pool::ThreadPool;
use mpeghdec_fdk::tools_rom::{get_bitstream_element_list, RbdId};
use mpeghdec_igf::igf_dec::*;
use mpeghdec_tpdec::TransportDec;

/// The memory of a channel element that the C++ code reaches through pointers of the channel
/// structures (see the [crate documentation](crate)).
pub struct CChannelElementMem<'a> {
    /// `pAacDecoderChannelInfo[ch]->pSpectralCoefficient`: `workBufferCore2` starting at the
    /// first channel of the element; channel `ch` of the element at `spec[ch * 1024..]`.
    pub spec: &'a mut [FixpDbl],
    /// `pAacDecoderChannelInfo[0]->pComStaticData` (shared by both channels of a CPE)
    pub com_static_data: &'a mut CAacDecoderCommonStaticData,
    /// `pComStaticData->pWorkBufferCore1` (shared by all channels)
    pub work_buffer_core1: &'a mut CWorkBufferCore1,
    /// `pAacDecoderStaticChannelInfo[ch]->pCpeStaticData` (`NULL` unless the element has
    /// `AC_EL_USAC_CP_POSSIBLE`)
    pub cpe_static_data: Option<&'a mut CpePersistentData>,
    /// `igf_private_data_common` (`IGFdata.IGF_Common_channel_data_handle` of channel `ch` of the
    /// element is `&igf_private_data_common[ch]`)
    pub igf_private_data_common: &'a mut [IgfPrivateDataCommon; 2],
    /// `pTimeData2` (at least 3072 values): IGF `virtualSpec` of channel `ch` of the element
    /// (`[1024 * ch..]`) and the complex prediction `scratchBuffer2` (`[2048..3072]`)
    pub time_data2: &'a mut [FixpDbl],
}

/// Two different elements of a slice.
fn pair_mut<T>(s: &mut [T]) -> (&mut T, &mut T) {
    let (a, b) = s.split_at_mut(1);
    (&mut a[0], &mut b[0])
}

/// `Clean_Complex_Prediction_coefficients(pJointStereoPersistentData, windowGroups, low_limit,
/// high_limit)`
fn clean_complex_prediction_coefficients(
    p_joint_stereo_persistent_data: &mut CJointStereoPersistentData,
    window_groups: i32,
    low_limit: i32,
    high_limit: i32,
) {
    for group in 0..window_groups.max(0) as usize {
        for sfb in low_limit.max(0) as usize..high_limit.max(0) as usize {
            p_joint_stereo_persistent_data.alpha_q_re_prev[group][sfb] = 0;
            p_joint_stereo_persistent_data.alpha_q_im_prev[group][sfb] = 0;
        }
    }
}

/// The IGF stereo processing of `CChannelElement_Decode()` (before or after TNS; the C++ code
/// contains the same block twice).
fn channel_element_igf_stereo(
    ch_info: &mut [Box<CAacDecoderChannelInfo>],
    st_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    mem: &mut CChannelElementMem,
    p_sampling_rate_info: &SamplingRateInfo,
    el_flags: u32,
) {
    let (st_l, st_r) = pair_mut(st_info);
    let (ch_l, ch_r) = pair_mut(ch_info);
    let (spec_l, spec_r) = mem.spec[..2048].split_at_mut(1024);
    let (vspec_l, vspec_r) = mem.time_data2[..2048].split_at_mut(1024);
    let (common_l, common_r) = pair_mut(&mut mem.igf_private_data_common[..]);
    let jsd = mem.com_static_data.joint_stereo_data.as_deref_mut().expect("pJointStereoData");
    let is_long = is_long_block(&ch_l.ics_info) == 1;

    {
        let (tnf_mask_l, tnf_mask_r) = mem.work_buffer_core1.tnf_masks();
        tnf_mask_l.fill(0);
        tnf_mask_r.fill(0);

        cigf_apply_stereo(
            &mut st_l.igf_static_data,
            &mut st_r.igf_static_data,
            &mut ch_l.igf_data,
            &mut ch_r.igf_data,
            common_l,
            common_r,
            vspec_l,
            vspec_r,
            spec_l,
            spec_r,
            &mut ch_l.spec_scale,
            &mut ch_r.spec_scale,
            if is_long { IGF_GRID_LONG_WINDOW } else { IGF_GRID_SHORT_WINDOW },
            get_window_groups(&ch_l.ics_info) as i32,
            if is_long_block(&ch_l.ics_info) != 0 { 1 } else { 8 },
            get_window_group_length_table(&ch_l.ics_info),
            &mut st_l.nf_random_seed,
            &mut st_r.nf_random_seed,
            &jsd.ms_used,
            tnf_mask_l,
            tnf_mask_r,
            (el_flags & AC_EL_IGF_USE_ENF != 0) as u8,
            IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
        );
    }

    let igf_start_sfb = if is_long_block(&ch_l.ics_info) != 0 {
        st_l.igf_static_data.igf_start_sfb_lb as i32
    } else {
        st_l.igf_static_data.igf_start_sfb_sb as i32
    };

    if jsd.igf_ms_mask_present != 3 {
        clean_complex_prediction_coefficients(
            &mut mem
                .cpe_static_data
                .as_deref_mut()
                .expect("pCpeStaticData")
                .joint_stereo_persistent_data,
            get_window_groups(&ch_l.ics_info) as i32,
            igf_start_sfb,
            64,
        );
    }

    let offsets = get_scale_factor_band_offsets(&ch_l.ics_info, p_sampling_rate_info);
    let cpd = mem.com_static_data.cplx_prediction_data.as_deref();
    cjoint_stereo_apply_ms_igf(
        jsd,
        cpd,
        &st_l.igf_static_data,
        &ch_l.ics_info,
        ch_l.granule_length,
        ch_r.granule_length,
        &ch_l.spec_scale,
        &ch_r.spec_scale,
        spec_l,
        spec_r,
        offsets,
        get_window_group_length_table(&ch_l.ics_info),
        get_window_groups(&ch_l.ics_info) as i32,
    );

    if is_long_block(&ch_l.ics_info) != 0 {
        /* Apply CP/MS over tha auxiliary array in case of TNF */
        if (ch_l.igf_data.bitstream_data[0].igf_use_enf_flat != 0)
            || (ch_r.igf_data.bitstream_data[0].igf_use_enf_flat != 0)
        {
            cjoint_stereo_apply_ms_igf(
                jsd,
                cpd,
                &st_l.igf_static_data,
                &ch_l.ics_info,
                ch_l.granule_length,
                ch_r.granule_length,
                &ch_l.spec_scale,
                &ch_r.spec_scale,
                vspec_l,
                vspec_r,
                offsets,
                get_window_group_length_table(&ch_l.ics_info),
                get_window_groups(&ch_l.ics_info) as i32,
            );
        }

        let (tnf_mask_l, tnf_mask_r) = mem.work_buffer_core1.tnf_masks();
        /* Apply IGF - TNF */
        cigf_tnf_apply(
            &st_l.igf_static_data,
            &ch_l.igf_data,
            vspec_l,
            spec_l,
            &ch_l.spec_scale,
            tnf_mask_l,
            0,
            0,
        );
        /* Apply IGF - TNF */
        cigf_tnf_apply(
            &st_l.igf_static_data,
            &ch_r.igf_data,
            vspec_r,
            spec_r,
            &ch_r.spec_scale,
            tnf_mask_r,
            0,
            0,
        );
    }
}

/// `CChannelElement_Decode(pAacDecoderChannelInfo[2], pAacDecoderStaticChannelInfo[2],
/// pSamplingRateInfo, flags, elFlags, el_channels)`: decode a channel element. To be called after
/// [`cchannel_element_read`].
///
/// The channel arrays start at the first channel of the element; `mem` holds the memory of the
/// element (see [`CChannelElementMem`]).
pub fn cchannel_element_decode(
    p_aac_decoder_channel_info: &mut [Box<CAacDecoderChannelInfo>],
    p_aac_decoder_static_channel_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    mem: &mut CChannelElementMem,
    p_sampling_rate_info: &SamplingRateInfo,
    flags: u32,
    el_flags: u32,
    el_channels: i32,
) {
    cchannel_element_decode_pool(
        p_aac_decoder_channel_info,
        p_aac_decoder_static_channel_info,
        mem,
        p_sampling_rate_info,
        flags,
        el_flags,
        el_channels,
        None,
    )
}

/// [`cchannel_element_decode`] with the per channel tools (IGF, TNS) of the two channels of a
/// CPE running in parallel on `pool` (not in C; the result is identical).
#[allow(clippy::too_many_arguments)]
pub fn cchannel_element_decode_pool(
    p_aac_decoder_channel_info: &mut [Box<CAacDecoderChannelInfo>],
    p_aac_decoder_static_channel_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    mem: &mut CChannelElementMem,
    p_sampling_rate_info: &SamplingRateInfo,
    flags: u32,
    el_flags: u32,
    el_channels: i32,
    pool: Option<&ThreadPool>,
) {
    const L: usize = 0;
    const R: usize = 1;
    let ch_info = p_aac_decoder_channel_info;
    let st_info = p_aac_decoder_static_channel_info;

    let mut max_sf_bands_l = 0;
    let mut max_sf_bands_r = 0;
    let maybe_jstereo = (el_channels > 1) as i32;

    if maybe_jstereo != 0 {
        max_sf_bands_l = get_scale_factor_bands_transmitted(&ch_info[L].ics_info) as i32;
        max_sf_bands_r = get_scale_factor_bands_transmitted(&ch_info[R].ics_info) as i32;

        /* apply ms */
        if ch_info[L].p_dyn_data.raw_data_info.common_window != 0 {
            let max_sfb_ste = ch_info[L].ics_info.max_sfb_ste as i32;
            let (st_l, st_r) = pair_mut(st_info);
            let (ch_l, ch_r) = pair_mut(ch_info);
            let (spec_l, spec_r) = mem.spec[..2048].split_at_mut(1024);
            let jsd =
                mem.com_static_data.joint_stereo_data.as_deref_mut().expect("pJointStereoData");
            let offsets = get_scale_factor_band_offsets(&ch_l.ics_info, p_sampling_rate_info);
            let wgl = ch_l.ics_info.window_group_length;
            let groups = get_window_groups(&ch_l.ics_info) as i32;

            {
                let cplx = if jsd.cplx_pred_flag != 0 {
                    Some(CJointStereoCplxCtx {
                        cplx_prediction_data: mem
                            .com_static_data
                            .cplx_prediction_data
                            .as_deref()
                            .expect("cplxPredictionData"),
                        joint_stereo_persistent_data: &mem
                            .cpe_static_data
                            .as_deref()
                            .expect("pCpeStaticData")
                            .joint_stereo_persistent_data,
                        static_spectral_coeffs_l: &st_l.concealment_info.spectral_coefficient,
                        static_spectral_coeffs_r: &st_r.concealment_info.spectral_coefficient,
                        static_spec_scale_l: &st_l.concealment_info.spec_scale,
                        static_spec_scale_r: &st_r.concealment_info.spec_scale,
                        scratch_buffer: mem.work_buffer_core1.work_buffer(),
                        scratch_buffer2: &mut mem.time_data2[2048..3072],
                    })
                } else {
                    None
                };
                cjoint_stereo_apply_ms(
                    jsd,
                    cplx,
                    &ch_l.ics_info,
                    ch_l.granule_length,
                    ch_r.granule_length,
                    ch_l.transform_splitting_active,
                    spec_l,
                    spec_r,
                    &mut ch_l.p_dyn_data.a_sfb_scale,
                    &mut ch_r.p_dyn_data.a_sfb_scale,
                    &mut ch_l.spec_scale,
                    &mut ch_r.spec_scale,
                    offsets,
                    &wgl,
                    groups,
                    max_sfb_ste,
                    max_sf_bands_l,
                    max_sf_bands_r,
                    1,
                );
            }

            if (el_flags & AC_EL_ENHANCED_NOISE != 0) && (el_flags & AC_EL_IGF_USE_ENF != 0) {
                /* Performs joint-stereo/MS for the IGF tiles (up to 4) */
                cjoint_stereo_apply_ms_igf_core(
                    ch_l,
                    ch_r,
                    st_l,
                    st_r,
                    jsd,
                    mem.com_static_data.cplx_prediction_data.as_deref(),
                    mem.cpe_static_data.as_deref(),
                    mem.work_buffer_core1,
                    mem.time_data2,
                    offsets,
                    &wgl,
                    groups,
                    max_sfb_ste,
                    max_sf_bands_l,
                    max_sf_bands_r,
                );
            }
        } /* if (pAacDecoderChannelInfo[L]->pDynData->RawDataInfo.CommonWindow)*/
    } /* maybe_stereo */

    for ch in 0..el_channels.max(0) as usize {
        let mut no_sfbs = get_scale_factor_bands_transmitted(&ch_info[ch].ics_info);
        /* For USAC common window: max_sfb of both channels may differ (common_max_sfb == 0). */
        if (maybe_jstereo == 1) && (ch_info[L].p_dyn_data.raw_data_info.common_window == 1) {
            no_sfbs = f_max(max_sf_bands_l, max_sf_bands_r) as u8;
        }
        let mut cp_active = 0;
        if el_flags & AC_EL_USAC_CP_POSSIBLE != 0 {
            cp_active = mem
                .com_static_data
                .joint_stereo_data
                .as_deref()
                .expect("pJointStereoData")
                .cplx_pred_flag as i32;
        }

        /* Omit writing of pAacDecoderChannelInfo[ch]->specScale for complex stereo prediction
        since scaling has already been carried out. */
        let max_sfb_ste = ch_info[L].ics_info.max_sfb_ste as i32;

        if !(cp_active != 0
            && max_sfb_ste == no_sfbs as i32
            && ch_info[ch].p_dyn_data.tns_data.active == 0)
        {
            cblock_scale_spectral_data(
                &mut ch_info[ch],
                &mut mem.spec[ch * 1024..(ch + 1) * 1024],
                no_sfbs,
                p_sampling_rate_info,
            );

            if (el_flags & AC_EL_ENHANCED_NOISE != 0) && (el_flags & AC_EL_IGF_USE_ENF != 0) {
                /* Scale tiles if Complex Prediction has not already been run */
                cblock_scale_tile_data(
                    &mut st_info[ch],
                    &mut ch_info[ch],
                    no_sfbs,
                    p_sampling_rate_info,
                );
            }
        }
    } /* End "for (ch = 0; ch < el_channels; ch++)" */

    if maybe_jstereo != 0 {
        /* apply ms */
        if ch_info[L].p_dyn_data.raw_data_info.common_window != 0 {
            /* Apply IGF stereo */
            if (el_flags & AC_EL_ENHANCED_NOISE != 0)
                && (el_flags & AC_EL_IGF_AFTER_TNS == 0)
                && (el_flags & AC_EL_IGF_INDEP_TILING == 0)
            {
                channel_element_igf_stereo(ch_info, st_info, mem, p_sampling_rate_info, el_flags);
            }
        }
        /* CommonWindow */
        else if el_flags & AC_EL_USAC_CP_POSSIBLE != 0 {
            let jspd = &mut mem
                .cpe_static_data
                .as_deref_mut()
                .expect("pCpeStaticData")
                .joint_stereo_persistent_data;
            jspd.alpha_q_re_prev = [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS];
            jspd.alpha_q_im_prev = [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS];
        }
    } /* if (maybe_jstereo) */

    let apply_tools_needed = flags & AC_USAC == 0
        || ch_info[L].p_dyn_data.specific_to_usac.tns_active == 1
        || maybe_jstereo == 0;
    if let Some(pool) = pool.filter(|_| apply_tools_needed && el_channels == 2) {
        /* The two channels use disjoint memory (their channel infos, spectra,
         * IGF_Common_channel_data_handle and IGF virtualSpec), so they are processed in
         * parallel (not in C). */
        let common_window = ch_info[L].p_dyn_data.raw_data_info.common_window as i32;
        let (st_l, st_r) = pair_mut(st_info);
        let (ch_l, ch_r) = pair_mut(ch_info);
        let (spec_l, spec_r) = mem.spec[..2048].split_at_mut(1024);
        let (common_l, common_r) = pair_mut(&mut mem.igf_private_data_common[..]);
        let (vspec_l, vspec_r) = mem.time_data2[..2048].split_at_mut(1024);
        let mut tasks = [
            (&mut **st_l, &mut **ch_l, spec_l, common_l, vspec_l),
            (&mut **st_r, &mut **ch_r, spec_r, common_r, vspec_r),
        ];
        pool.for_each_mut(&mut tasks, |_, (st, ch, spec, common, vspec)| {
            apply_tools_channel(
                st,
                ch,
                spec,
                common,
                vspec,
                p_sampling_rate_info,
                flags,
                el_flags,
                common_window,
            )
        });
    } else {
        for ch in 0..el_channels.max(0) {
            if apply_tools_needed {
                let common_window = ch_info[L].p_dyn_data.raw_data_info.common_window as i32;
                apply_tools(
                    st_info,
                    ch_info,
                    mem.spec,
                    p_sampling_rate_info,
                    flags,
                    el_flags,
                    ch,
                    common_window,
                    mem.igf_private_data_common,
                    mem.time_data2,
                );
            }
        } /* End "for (ch = 0; ch < el_channels; ch++)" */
    }

    if maybe_jstereo != 0 {
        /* apply ms */
        if ch_info[L].p_dyn_data.raw_data_info.common_window != 0 {
            /* Apply IGF stereo */
            if (el_flags & AC_EL_ENHANCED_NOISE != 0)
                && (el_flags & AC_EL_IGF_AFTER_TNS != 0)
                && (el_flags & AC_EL_IGF_INDEP_TILING == 0)
            {
                channel_element_igf_stereo(ch_info, st_info, mem, p_sampling_rate_info, el_flags);
            }
        } /* if (pAacDecoderChannelInfo[L]->pDynData->RawDataInfo.CommonWindow) */
    } /* if (maybe_jstereo) */

    for _ch in 0..el_channels.max(0) {
        if el_flags & AC_EL_USAC_CP_POSSIBLE != 0 {
            let jspd = &mut mem
                .cpe_static_data
                .as_deref_mut()
                .expect("pCpeStaticData")
                .joint_stereo_persistent_data;
            jspd.clear_spectral_coeffs = 0;
            if ch_info[L].p_dyn_data.raw_data_info.common_window != 0
                && ch_info[L].transform_splitting_active != ch_info[R].transform_splitting_active
            {
                jspd.clear_spectral_coeffs = 1;
            }
        }
    }
}

/// `CJointStereo_ApplyMSIGFcore(pAacDecoderChannelInfo[2], pAacDecoderStaticChannelInfo[2],
/// pScaleFactorBandOffsets, pWindowGroupLength, windowGroups, max_sfb_ste,
/// scaleFactorBandsTransmittedL, scaleFactorBandsTransmittedR)` (stereo.cpp): performs joint
/// stereo / M/S for the IGF tiles.
///
/// The C++ function reaches the element data through the channel structures; here they are passed
/// explicitly (the arguments of [`cjoint_stereo_apply_ms`]).
pub fn cjoint_stereo_apply_ms_igf_core(
    ch_l: &mut CAacDecoderChannelInfo,
    ch_r: &mut CAacDecoderChannelInfo,
    st_l: &mut CAacDecoderStaticChannelInfo,
    st_r: &mut CAacDecoderStaticChannelInfo,
    p_joint_stereo_data: &mut CJointStereoData,
    cplx_prediction_data: Option<&CCplxPredictionData>,
    cpe_static_data: Option<&CpePersistentData>,
    work_buffer_core1: &mut CWorkBufferCore1,
    time_data2: &mut [FixpDbl],
    p_scale_factor_band_offsets: &[i16],
    p_window_group_length: &[u8],
    window_groups: i32,
    max_sfb_ste: i32,
    scale_factor_bands_transmitted_l: i32,
    scale_factor_bands_transmitted_r: i32,
) {
    let is_short_l = (get_window_sequence(&ch_l.ics_info) == BLOCK_SHORT) as i32;
    let is_short_r = (get_window_sequence(&ch_r.ics_info) == BLOCK_SHORT) as i32;
    let number_of_tiles = iis_igf_dec_lib_get_number_of_tiles(&st_l.igf_static_data, is_short_l);

    /* Set pointers to the already computed previous DMX and its exaponent (PreviousDMX =
     * pJointStereoData->store_dmx_re_prev, see cjoint_stereo_apply_ms) */

    /* Apply MS over tiles */
    for tile_idx in 0..number_of_tiles {
        /* Get tile spectrum */
        let tile_spectrum_l = iis_igf_dec_lib_access_source_spectrum(
            &st_l.igf_static_data,
            &mut ch_l.igf_data,
            tile_idx,
            is_short_l,
            0,
        )
        .expect("tile L");
        let tile_spectrum_r = iis_igf_dec_lib_access_source_spectrum(
            &st_r.igf_static_data,
            &mut ch_r.igf_data,
            tile_idx,
            is_short_r,
            0,
        )
        .expect("tile R");

        /* Get tile SFB scaling data (fSpectrumTab_sfb_exp) and the spectrum scale values to be
         * computed (fSpectrumScale) */
        let s_l = &mut st_l.igf_static_data;
        let s_r = &mut st_r.igf_static_data;
        let left_scale = &mut s_l.f_spectrum_tab_sfb_exp[tile_idx as usize];
        let right_scale = &mut s_r.f_spectrum_tab_sfb_exp[tile_idx as usize];
        let spec_scale_l = &mut s_l.f_spectrum_scale[tile_idx as usize];
        let spec_scale_r = &mut s_r.f_spectrum_scale[tile_idx as usize];

        let cplx = if p_joint_stereo_data.cplx_pred_flag != 0 {
            Some(CJointStereoCplxCtx {
                cplx_prediction_data: cplx_prediction_data.expect("cplxPredictionData"),
                joint_stereo_persistent_data: &cpe_static_data
                    .expect("pCpeStaticData")
                    .joint_stereo_persistent_data,
                static_spectral_coeffs_l: &st_l.concealment_info.spectral_coefficient,
                static_spectral_coeffs_r: &st_r.concealment_info.spectral_coefficient,
                static_spec_scale_l: &st_l.concealment_info.spec_scale,
                static_spec_scale_r: &st_r.concealment_info.spec_scale,
                scratch_buffer: work_buffer_core1.work_buffer(),
                scratch_buffer2: &mut time_data2[2048..3072],
            })
        } else {
            None
        };

        /* Apply MS over the tiles */
        cjoint_stereo_apply_ms(
            p_joint_stereo_data,
            cplx,
            &ch_l.ics_info,
            ch_l.granule_length,
            ch_r.granule_length,
            ch_l.transform_splitting_active,
            tile_spectrum_l,
            tile_spectrum_r,
            left_scale,
            right_scale,
            spec_scale_l,
            spec_scale_r,
            p_scale_factor_band_offsets,
            p_window_group_length,
            window_groups,
            max_sfb_ste,
            scale_factor_bands_transmitted_l,
            scale_factor_bands_transmitted_r,
            0,
        );
    } /* for(int tileIdx=0;tileIdx<NumberOfTiles;tileIdx++) */
}

/// `CChannel_CodebookTableInit(pAacDecoderChannelInfo)`: init codeBook SFB indices (section
/// data) with HCB_ESC. Useful for bitstreams which do not have any section data, but still SFB's
/// (scale factor bands). This has the effect that upto the amount of transmitted SFB are treated
/// as non-zero.
pub fn cchannel_codebook_table_init(p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo) {
    let max_sfb = get_scale_factor_bands_transmitted(&p_aac_decoder_channel_info.ics_info) as usize;
    let (max_bands, max_windows) =
        if is_long_block(&p_aac_decoder_channel_info.ics_info) != 0 { (64, 1) } else { (16, 8) };
    let p_code_book = &mut p_aac_decoder_channel_info.p_dyn_data.a_code_book;

    /* Note: for long blocks the table (8 * 16 entries) is written up to index 63. */
    let mut off = 0;
    for _w in 0..max_windows {
        let mut b = 0;
        while b < max_sfb {
            p_code_book[off + b] = ESCBOOK;
            b += 1;
        }
        while b < max_bands {
            p_code_book[off + b] = ZERO_HCB;
            b += 1;
        }
        off += max_bands;
    }
}

/// `EnhancedNoiseFilling(hBs, pAacDecoderStaticChannelInfo, pAacDecoderChannelInfo, flags,
/// elFlags)`: reads the IGF scale factors and IGF data of a channel.
fn enhanced_noise_filling(
    hbs: &mut FdkBitStream,
    p_aac_decoder_static_channel_info: &mut CAacDecoderStaticChannelInfo,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    flags: u32,
    el_flags: u32,
) {
    let mut b_usac_indep_flag = 0;

    let (is_short_win, window_groups) = if is_long_block(&p_aac_decoder_channel_info.ics_info) != 0
    {
        (0, 1)
    } else {
        (1, p_aac_decoder_channel_info.ics_info.window_groups as i32)
    };

    if flags & AC_INDEP != 0 {
        b_usac_indep_flag = 1;
    }

    iis_igf_dec_lib_read_scf(
        &mut p_aac_decoder_static_channel_info.igf_static_data,
        &mut p_aac_decoder_channel_info.igf_data,
        hbs,
        b_usac_indep_flag,
        is_short_win,
        window_groups,
        IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
    );

    iis_igf_dec_lib_read_igf(
        &mut p_aac_decoder_static_channel_info.igf_static_data,
        &mut p_aac_decoder_channel_info.igf_data,
        hbs,
        is_short_win,
        b_usac_indep_flag,
        (el_flags & AC_EL_IGF_USE_ENF != 0) as i32,
        IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
    );
}

/// `CChannelElement_Read(hBs, pAacDecoderChannelInfo[], pAacDecoderStaticChannelInfo[], aot,
/// pSamplingRateInfo, flags, elFlags, frame_length, numberOfChannels, epConfig, pTpDec)`:
/// arbitrary order bitstream parser of a channel element (followed by inverse quantization,
/// noise filling, stereo filling and frequency domain prediction).
///
/// The channel arrays start at the first channel of the element; `mem` holds the memory of the
/// element (see [`CChannelElementMem`]). `p_tp_dec` is required for ADTS/DRM CRC checking only.
pub fn cchannel_element_read(
    hbs: &mut FdkBitStream,
    p_aac_decoder_channel_info: &mut [Box<CAacDecoderChannelInfo>],
    p_aac_decoder_static_channel_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    mem: &mut CChannelElementMem,
    aot: AudioObjectType,
    p_sampling_rate_info: &SamplingRateInfo,
    flags: u32,
    el_flags: u32,
    frame_length: u32,
    number_of_channels: u8,
    ep_config: i8,
    mut p_tp_dec: Option<&mut TransportDec>,
) -> AacDecoderError {
    let ch_info = p_aac_decoder_channel_info;
    let st_info = p_aac_decoder_static_channel_info;
    let mut error = AAC_DEC_OK;
    let mut crc_reg1 = -1;
    let mut crc_reg2 = -1;
    let mut common_ltpf = 0;

    debug_assert!((number_of_channels == 1) || (number_of_channels == 2));

    'bail: {
        /* Get channel element sequence table */
        let Some(mut list) =
            get_bitstream_element_list(aot, ep_config, number_of_channels, 0, el_flags)
        else {
            error = AAC_DEC_UNSUPPORTED_FORMAT;
            break 'bail;
        };

        ctns_reset(&mut ch_info[0].p_dyn_data.tns_data);
        /* Set common window to 0 by default. If signalized in the bit stream it will be overwritten
         * later explicitely */
        ch_info[0].p_dyn_data.raw_data_info.common_window = 0;
        if flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 {
            ch_info[0].p_dyn_data.specific_to_usac.tns_active = 0;
        }
        if number_of_channels == 2 {
            ctns_reset(&mut ch_info[1].p_dyn_data.tns_data);
            ch_info[1].p_dyn_data.raw_data_info.common_window = 0;
        }

        let mut cplx_pred = 0;
        if el_flags & AC_EL_USAC_CP_POSSIBLE != 0 {
            mem.com_static_data
                .joint_stereo_data
                .as_deref_mut()
                .expect("pJointStereoData")
                .cplx_pred_flag = 0;
            cplx_pred = 1;
        }

        /* Iterate through sequence table */
        let mut i: i32 = 0;
        let mut ch: usize = 0;
        let mut decision_bit: u32 = 0;
        loop {
            match list.id[i as usize] {
                RbdId::enhancedNoiseFilling => {
                    if el_flags & AC_EL_ENHANCED_NOISE != 0 {
                        if st_info[ch].igf_static_data.enhanced_noise_filling == 0 {
                            return AAC_DEC_UNKNOWN; /* robustness: unrecoverable init error */
                        }
                        enhanced_noise_filling(
                            hbs,
                            &mut st_info[ch],
                            &mut ch_info[ch],
                            flags,
                            el_flags,
                        );
                    }
                }
                RbdId::element_instance_tag => {
                    ch_info[0].element_instance_tag = hbs.read_bits(4) as u8;
                    if number_of_channels == 2 {
                        ch_info[1].element_instance_tag = ch_info[0].element_instance_tag;
                    }
                }
                RbdId::common_window => {
                    decision_bit = hbs.read_bit();
                    ch_info[ch].p_dyn_data.raw_data_info.common_window = decision_bit as u8;
                    if number_of_channels == 2 {
                        ch_info[1].p_dyn_data.raw_data_info.common_window =
                            ch_info[0].p_dyn_data.raw_data_info.common_window;
                    }
                }
                RbdId::ics_info => 'case: {
                    /* store last window sequence (utilized in complex stereo prediction) before
                     * reading new channel-info */
                    if cplx_pred != 0 && ch_info[0].p_dyn_data.raw_data_info.common_window != 0 {
                        let jspd = &mut mem
                            .cpe_static_data
                            .as_deref_mut()
                            .expect("pCpeStaticData")
                            .joint_stereo_persistent_data;
                        jspd.win_seq_prev = ch_info[0].ics_info.window_sequence as u8;
                        jspd.win_shape_prev = ch_info[0].ics_info.window_shape;
                    }
                    /* Read individual channel info */
                    if flags & AC_MPEGH3DA != 0 {
                        st_info[ch].prev_window_shape = ch_info[ch].ics_info.window_shape;
                    }
                    error = ics_read(hbs, &mut ch_info[ch].ics_info, p_sampling_rate_info, flags);

                    if (el_flags & AC_EL_LFE != 0)
                        && get_window_sequence(&ch_info[ch].ics_info) != BLOCK_LONG
                    {
                        error = AAC_DEC_PARSE_ERROR;
                        break 'case;
                    }

                    if number_of_channels == 2
                        && ch_info[0].p_dyn_data.raw_data_info.common_window != 0
                    {
                        ch_info[1].ics_info = ch_info[0].ics_info;
                    }
                }
                RbdId::common_max_sfb => {
                    if hbs.read_bit() == 0 {
                        error = AAC_DEC_PARSE_ERROR;
                    }
                }
                RbdId::ltp_data_present => {
                    if hbs.read_bit() != 0 {
                        error = AAC_DEC_UNSUPPORTED_PREDICTION;
                    }
                }
                RbdId::ms => {
                    let mut max_sfb_ste = get_scale_max_factor_bands_transmitted(
                        &ch_info[0].ics_info,
                        &ch_info[1].ics_info,
                    ) as i32;
                    let mut max_sfb_ste_clear = 64;
                    let mut igf_start_sfb = 0;
                    let mut igf_stop_sfb = 0;

                    let igf_use_enhances_noise = (el_flags & AC_EL_ENHANCED_NOISE != 0)
                        && (el_flags & AC_EL_IGF_INDEP_TILING == 0);
                    if igf_use_enhances_noise {
                        let igf = &st_info[ch].igf_static_data;
                        if is_long_block(&ch_info[ch].ics_info) != 0 {
                            igf_start_sfb = igf.igf_start_sfb_lb as i32;
                            igf_stop_sfb = igf.igf_stop_sfb_lb as i32;
                        } else {
                            igf_start_sfb = igf.igf_start_sfb_sb as i32;
                            igf_stop_sfb = igf.igf_stop_sfb_sb as i32;
                        }

                        max_sfb_ste = f_min(max_sfb_ste, igf_start_sfb);
                        max_sfb_ste_clear = f_min(max_sfb_ste_clear, igf_start_sfb);

                        if (max_sfb_ste > ch_info[0].ics_info.max_sf_bands as i32)
                            || (max_sfb_ste > ch_info[1].ics_info.max_sf_bands as i32)
                        {
                            error = AAC_DEC_PARSE_ERROR;
                        }
                    }

                    ch_info[0].ics_info.max_sfb_ste = max_sfb_ste as u8;
                    ch_info[1].ics_info.max_sfb_ste = max_sfb_ste as u8;

                    if flags & (AC_USAC | AC_MPEGH3DA) != 0
                        && ch_info[ch].p_dyn_data.raw_data_info.common_window == 0
                    {
                        clean_complex_prediction_coefficients(
                            &mut mem
                                .cpe_static_data
                                .as_deref_mut()
                                .expect("pCpeStaticData")
                                .joint_stereo_persistent_data,
                            get_window_groups(&ch_info[0].ics_info) as i32,
                            0,
                            64,
                        );
                    }

                    let jsd = mem
                        .com_static_data
                        .joint_stereo_data
                        .as_deref_mut()
                        .expect("pJointStereoData");
                    /* jointStereoPersistentData and cplxPredictionData are only available/allocated
                    if cplxPred is active. */
                    let jspd = if cplx_pred == 0 {
                        None
                    } else {
                        mem.cpe_static_data
                            .as_deref_mut()
                            .map(|c| &mut c.joint_stereo_persistent_data)
                    };
                    let cpd = if cplx_pred == 0 {
                        None
                    } else {
                        mem.com_static_data.cplx_prediction_data.as_deref_mut()
                    };
                    if cjoint_stereo_read(
                        hbs,
                        jsd,
                        get_window_groups(&ch_info[0].ics_info) as i32,
                        max_sfb_ste,
                        max_sfb_ste_clear,
                        jspd,
                        cpd,
                        cplx_pred,
                        get_scale_factor_bands_total(&ch_info[0].ics_info) as i32,
                        get_window_sequence(&ch_info[0].ics_info),
                        flags,
                    ) != 0
                    {
                        error = AAC_DEC_PARSE_ERROR;
                    }

                    if igf_use_enhances_noise {
                        if cjoint_stereo_read_igf(
                            hbs,
                            mem.com_static_data
                                .joint_stereo_data
                                .as_deref_mut()
                                .expect("pJointStereoData"),
                            get_window_groups(&ch_info[0].ics_info) as i32,
                            igf_start_sfb,
                            igf_stop_sfb,
                            st_info[ch].igf_static_data.use_high_res as i32,
                            &mut mem
                                .cpe_static_data
                                .as_deref_mut()
                                .expect("pCpeStaticData")
                                .joint_stereo_persistent_data,
                            mem.com_static_data
                                .cplx_prediction_data
                                .as_deref_mut()
                                .expect("cplxPredictionData"),
                            get_window_sequence(&ch_info[0].ics_info),
                            flags,
                        ) != 0
                        {
                            error = AAC_DEC_PARSE_ERROR;
                        }
                    } else if el_flags & AC_EL_USAC_CP_POSSIBLE != 0 {
                        let runs =
                            get_scale_factor_bands_total(&ch_info[0].ics_info) as i32 - max_sfb_ste;
                        if runs > 0 {
                            let c_pd = mem
                                .com_static_data
                                .cplx_prediction_data
                                .as_deref_mut()
                                .expect("cplxPredictionData");
                            let s = max_sfb_ste as usize;
                            let e = s + runs as usize;
                            for group in 0..8 {
                                c_pd.alpha_q_re[group][s..e].fill(0);
                                c_pd.alpha_q_im[group][s..e].fill(0);
                            }
                        }
                    }
                }
                RbdId::global_gain => {
                    ch_info[ch].p_dyn_data.raw_data_info.global_gain = hbs.read_bits(8) as u8;
                }
                RbdId::scale_factor_data_usac | RbdId::scale_factor_data => 'case: {
                    if list.id[i as usize] == RbdId::scale_factor_data_usac {
                        if aot == AOT_MPEGH3DA {
                            if common_ltpf == 0 {
                                if read_ltp_param(
                                    hbs,
                                    &mut st_info[ch].ltp_param,
                                    (el_flags & AC_EL_LFE) as i32,
                                ) != 0
                                {
                                    error = AAC_DEC_PARSE_ERROR;
                                    break 'case;
                                }
                            } else if ch == 1 {
                                st_info[1].ltp_param = st_info[0].ltp_param;
                            }
                            ch_info[ch].fdp_spacing_index = -1;
                            if (flags & AC_INDEP == 0)
                                && (get_window_sequence(&ch_info[ch].ics_info) != BLOCK_SHORT)
                            {
                                ch_info[ch].fdp_data_present = hbs.read_bit() as i32; /* fdp_data_present */
                                if ch_info[ch].fdp_data_present != 0 {
                                    if el_flags & AC_EL_LFE != 0 {
                                        error = AAC_DEC_PARSE_ERROR;
                                        break 'case;
                                    }
                                    ch_info[ch].fdp_spacing_index = hbs.read_bits(8) as i32;
                                    /* fdp_spacing_index */
                                }
                            }
                            if flags & AC_INDEP != 0 {
                                st_info[ch].i_mdct.prev_alias_symmetry = hbs.read_bit() as i32;
                                /* prev_aliasing_symmetry */
                            }
                            ch_info[ch].curr_aliasing_symmetry = hbs.read_bit() as i32; /* curr_aliasing_symmetry */
                            if (el_flags & AC_EL_LFE != 0)
                                && (st_info[ch].i_mdct.prev_alias_symmetry != 0
                                    || ch_info[ch].curr_aliasing_symmetry != 0)
                            {
                                error = AAC_DEC_PARSE_ERROR;
                                break 'case;
                            }
                        }
                        /* Set active sfb codebook indexes to HCB_ESC to make them "active" */
                        cchannel_codebook_table_init(&mut ch_info[ch]); /*  equals ReadSectionData(self, bs) in float soft. block.c line: ~599 */
                        /* Note: The missing "break" is intentional here, since we need to call
                         * CBlock_ReadScaleFactorData(). */
                    }
                    /* case scale_factor_data: */
                    if flags & AC_ER_RVLC != 0 {
                    } else {
                        error = cblock_read_scale_factor_data(&mut ch_info[ch], hbs, flags);
                    }
                }
                RbdId::tns_data_present => {
                    ctns_read_data_present_flag(hbs, &mut ch_info[ch].p_dyn_data.tns_data);
                    if (el_flags & AC_EL_LFE != 0)
                        && ch_info[ch].p_dyn_data.tns_data.data_present != 0
                    {
                        error = AAC_DEC_PARSE_ERROR;
                    }
                }
                RbdId::tns_data => {
                    /* tns_data_present is checked inside CTns_Read(). */
                    let c = &mut *ch_info[ch];
                    error = ctns_read(hbs, &mut c.p_dyn_data.tns_data, &c.ics_info, flags);
                }
                RbdId::gain_control_data => {}
                RbdId::gain_control_data_present => {
                    if hbs.read_bit() != 0 {
                        error = AAC_DEC_UNSUPPORTED_GAIN_CONTROL_DATA;
                    }
                }
                RbdId::tw_data => {}
                RbdId::common_tw => 'case: {
                    if aot == AOT_MPEGH3DA {
                        common_ltpf = hbs.read_bit();
                        if common_ltpf != 0
                            && read_ltp_param(
                                hbs,
                                &mut st_info[ch].ltp_param,
                                (el_flags & AC_EL_LFE) as i32,
                            ) != 0
                        {
                            error = AAC_DEC_PARSE_ERROR;
                            break 'case;
                        }
                    }
                }
                RbdId::tns_data_present_usac => {
                    if ch_info[0].p_dyn_data.specific_to_usac.tns_active != 0 {
                        let mut tns_on_lr = 0u8;
                        let (c0, c1) = pair_mut(ch_info);
                        let common_window = c0.p_dyn_data.raw_data_info.common_window as i32;
                        ctns_read_data_present_usac(
                            hbs,
                            &mut c0.p_dyn_data.tns_data,
                            &mut c1.p_dyn_data.tns_data,
                            &mut tns_on_lr,
                            &c0.ics_info,
                            flags,
                            el_flags,
                            common_window,
                        );
                        if tns_on_lr == 0 {
                            error = AAC_DEC_PARSE_ERROR;
                        }
                    }
                }
                RbdId::core_mode => {
                    decision_bit = hbs.read_bit();
                    if decision_bit != 0 {
                        error = AAC_DEC_PARSE_ERROR;
                    }
                }
                RbdId::tns_active => {
                    ch_info[0].p_dyn_data.specific_to_usac.tns_active = hbs.read_bit() as u8;
                }
                RbdId::noise => {
                    if el_flags & AC_EL_USAC_NOISE != 0 {
                        ch_info[ch].p_dyn_data.specific_to_usac.fd_noise_level_and_offset =
                            hbs.read_bits(3 + 5) as u8; /* Noise level */
                    }
                }
                RbdId::fac_data => {
                    let f_fac_dat_present = hbs.read_bit();

                    if f_fac_dat_present != 0 {
                        error = AAC_DEC_PARSE_ERROR;
                    }
                }
                RbdId::esc2_rvlc => if flags & AC_ER_RVLC != 0 {},
                RbdId::esc1_hcr => if flags & AC_ER_HCR != 0 {},
                RbdId::ac_spectral_data => {
                    let c = &*ch_info[ch];
                    error = cblock_read_ac_spectral_data(
                        hbs,
                        c,
                        &mut mem.spec[ch * 1024..(ch + 1) * 1024],
                        &mut st_info[ch],
                        p_sampling_rate_info,
                        frame_length,
                        flags,
                    );
                    ch_info[ch].render_mode = AACDEC_RENDER_IMDCT;
                }

                /* CRC handling */
                RbdId::adtscrc_start_reg1 => {
                    if let Some(tp) = p_tp_dec.as_deref_mut() {
                        crc_reg1 = tp.crc_start_reg(192);
                    }
                }
                RbdId::adtscrc_start_reg2 => {
                    if let Some(tp) = p_tp_dec.as_deref_mut() {
                        crc_reg2 = tp.crc_start_reg(128);
                    }
                }
                RbdId::adtscrc_end_reg1 | RbdId::drmcrc_end_reg => {
                    if let Some(tp) = p_tp_dec.as_deref_mut() {
                        tp.crc_end_reg(crc_reg1);
                        crc_reg1 = -1;
                    }
                }
                RbdId::adtscrc_end_reg2 => {
                    if crc_reg1 != -1 {
                        error = AAC_DEC_DECODE_FRAME_ERROR;
                    } else if let Some(tp) = p_tp_dec.as_deref_mut() {
                        tp.crc_end_reg(crc_reg2);
                        crc_reg2 = -1;
                    }
                }
                RbdId::drmcrc_start_reg => {
                    if let Some(tp) = p_tp_dec.as_deref_mut() {
                        crc_reg1 = tp.crc_start_reg(0);
                    }
                }

                /* Non data cases */
                RbdId::next_channel => {
                    ch = (ch + 1) % number_of_channels as usize;
                }
                RbdId::link_sequence => {
                    list = list.next[decision_bit as usize].expect("element list");
                    i = -1;
                }

                _ => {
                    error = AAC_DEC_UNSUPPORTED_FORMAT;
                }
            }

            if error != AAC_DEC_OK {
                break 'bail;
            }

            i += 1;

            if list.id[i as usize] == RbdId::end_of_sequence {
                break;
            }
        }

        for ch in 0..number_of_channels as usize {
            if ch_info[ch].render_mode == AACDEC_RENDER_IMDCT
                || ch_info[ch].render_mode == AACDEC_RENDER_ELDFB
            {
                /* Copy lower part of coded spectrum for later use in FD Prediction */
                let mut quant_spec_curr = [0 as FixpDbl; 172];
                quant_spec_curr.copy_from_slice(&mem.spec[ch * 1024..ch * 1024 + 172]);

                /* IGF Stereo Filling start/stop */
                let mut igf_stereo_filling = 0u8;
                /* Check for stereo filling only when MPEG-H stream is available */
                if aot == AOT_MPEGH3DA {
                    ch_info[ch].transform_splitting_active = 0;

                    if el_flags & AC_EL_USAC_NOISE != 0 {
                        let mut temp =
                            ch_info[ch].p_dyn_data.specific_to_usac.fd_noise_level_and_offset;
                        /* When "noise_level"=0 and "noise_offset"!=0 => check if transform
                         * splitting or stereo filling and rearrange noise bits */
                        if (temp > 0) && (temp < 32) {
                            /* Activate stereo filling when the window of the two channels is common
                             * and this is the right (second) channel */
                            if (ch_info[0].p_dyn_data.raw_data_info.common_window == 1) && (ch == 1)
                            {
                                temp <<= 3;
                                ch_info[ch].p_dyn_data.specific_to_usac.fd_noise_level_and_offset =
                                    temp;
                                /* Activate Stereo Filling */
                                igf_stereo_filling = 1;
                            }

                            /* Activate transform splitting for LONG_START_SEQUENCE and
                             * STOP_START_SEQUENCE only. */
                            if ch_info[ch].ics_info.window_sequence == BLOCK_START && (ch == 0) {
                                temp <<= 3;
                                ch_info[ch].p_dyn_data.specific_to_usac.fd_noise_level_and_offset =
                                    temp;
                                /* Signal transform splitting */
                                ch_info[ch].transform_splitting_active = 1;
                            }
                        } /* if((temp > 0)&&(temp < 32)) */

                        if (ch_info[0].p_dyn_data.raw_data_info.common_window != 0) && (ch == 1) {
                            ch_info[ch].transform_splitting_active =
                                ch_info[0].transform_splitting_active;
                        }
                    } /* if (elFlags & AC_EL_USAC_NOISE) */
                } /* if(aot==AOT_MPEGH3DA) */

                /* Shows which bands are empty. */
                ch_info[ch].p_dyn_data.band_is_noise = [1; 8 * 16];

                let e = cblock_inverse_quantize_spectral_data(
                    &mut ch_info[ch],
                    &mut mem.spec[ch * 1024..(ch + 1) * 1024],
                    p_sampling_rate_info,
                    1,
                );
                if e != AAC_DEC_OK {
                    return e;
                }

                /* Syncronize channel tiling and whitening info for IGF if necessary */
                /* Just for stereo */
                if (el_flags & AC_EL_ENHANCED_NOISE != 0)
                    && (el_flags & AC_EL_IGF_INDEP_TILING == 0)
                    && (number_of_channels == 2)
                    && (ch == 0)
                {
                    /* Sync once */
                    let (c0, c1) = pair_mut(ch_info);
                    iis_igf_sync_data(&mut c0.igf_data, &mut c1.igf_data);
                }

                /* Copy the base spectrum into tiles (up to 4) for subsequent IGF processing in
                case INF is active. Copying is implemented before the noise filling stage, i.e.
                tiles are sparsely populated */
                if (el_flags & AC_EL_ENHANCED_NOISE != 0) && (el_flags & AC_EL_IGF_USE_ENF != 0) {
                    let c = &mut *ch_info[ch];
                    iis_igf_dec_lib_inject_source_spectrum_new(
                        &mut st_info[ch].igf_static_data,
                        &mut c.igf_data,
                        &mem.spec[ch * 1024..(ch + 1) * 1024],
                        &c.p_dyn_data.a_sfb_scale,
                        (is_long_block(&c.ics_info) == 0) as i32,
                        IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
                    );
                }

                let mut max_noise_sfb: u8 = 0;
                if el_flags & AC_EL_USAC_NOISE != 0 {
                    max_noise_sfb = get_scale_factor_bands_transmitted(&ch_info[ch].ics_info);

                    if el_flags & AC_EL_ENHANCED_NOISE != 0 {
                        let igf = &st_info[ch].igf_static_data;
                        if igf.enhanced_noise_filling != 0 {
                            if is_long_block(&ch_info[ch].ics_info) != 0 {
                                if max_noise_sfb > igf.igf_start_sfb_lb {
                                    max_noise_sfb = igf.igf_start_sfb_lb;
                                }
                            } else if max_noise_sfb > igf.igf_start_sfb_sb {
                                max_noise_sfb = igf.igf_start_sfb_sb;
                            }
                        }

                        /* Noise filling is applied after the noise params for the empty sfb-bands
                         * have been modified. */
                        /* If INF is active, then fill the separate tiles with noise independently */
                        cblock_apply_noise(
                            &mut st_info[ch],
                            &mut ch_info[ch],
                            &mut mem.spec[ch * 1024..(ch + 1) * 1024],
                            p_sampling_rate_info,
                            (el_flags & AC_EL_IGF_USE_ENF != 0) as i32,
                            igf_stereo_filling as i32,
                            max_noise_sfb as i32,
                        );
                    }
                    /* if(elFlags & AC_EL_ENHANCED_NOISE) */
                    else {
                        cblock_apply_noise(
                            &mut st_info[ch],
                            &mut ch_info[ch],
                            &mut mem.spec[ch * 1024..(ch + 1) * 1024],
                            p_sampling_rate_info,
                            0,
                            0,
                            max_noise_sfb as i32,
                        );
                    }

                    /*Entered only for particular noise values and when Channel=1 */
                    if igf_stereo_filling != 0 && is_long_block(&ch_info[ch].ics_info) != 0 {
                        /* Stereo filling is active only for long frames*/
                        igf_stereo_filling_decode(
                            ch_info,
                            st_info,
                            mem,
                            p_sampling_rate_info,
                            el_flags,
                            max_noise_sfb as i32,
                        );
                    }
                } /* if (elFlags & AC_EL_USAC_NOISE) */

                if flags & AC_MPEGH3DA != 0 {
                    let prediction_bandwidth =
                        if (frame_length != 768) && (p_sampling_rate_info.sampling_rate >= 44100) {
                            132
                        } else {
                            (160 * frame_length as i32) / 1024
                        };

                    {
                        let c = &mut *ch_info[ch];
                        let offsets =
                            get_scale_factor_band_offsets(&c.ics_info, p_sampling_rate_info);
                        fdp_decode_bins(
                            c,
                            &mut mem.spec[ch * 1024..(ch + 1) * 1024],
                            &mut st_info[ch],
                            flags,
                            &quant_spec_curr,
                            offsets,
                            prediction_bandwidth,
                            0, /* isTcx */
                            0, /* tcx => i_gain_m */
                            0, /* tcx => i_gain_e */
                            0, /* tcx => g_div64 */
                            0, /* tcx => lg */
                        );
                    }

                    /* FDP for IGF tiles when INF is active */
                    if (el_flags & AC_EL_ENHANCED_NOISE != 0) && (el_flags & AC_EL_IGF_USE_ENF != 0)
                    {
                        if el_flags & AC_EL_USAC_NOISE == 0 {
                            max_noise_sfb =
                                get_scale_factor_bands_transmitted(&ch_info[ch].ics_info);
                        }
                        if (is_long_block(&ch_info[ch].ics_info) != 0)
                            && (ch_info[ch].fdp_spacing_index >= 0)
                            && (max_noise_sfb > 0)
                        {
                            let st = &mut *st_info[ch];
                            let c = &mut *ch_info[ch];
                            let number_of_tiles =
                                iis_igf_dec_lib_get_number_of_tiles(&st.igf_static_data, 0);
                            let sfb_offset =
                                get_scale_factor_band_offsets(&c.ics_info, p_sampling_rate_info);
                            let max_num_bins_copied = f_min(
                                prediction_bandwidth,
                                sfb_offset[max_noise_sfb as usize - 1] as i32,
                            );

                            /* determine sfb for max_num_bins_copied */
                            let mut sfb = 0usize;
                            while (sfb_offset[sfb] as i32) < max_num_bins_copied {
                                sfb += 1;
                            }
                            let sfb_max_num_bins_copied = sfb;
                            let n_bins = sfb_offset[sfb_max_num_bins_copied] as usize;

                            for tile_idx in 0..number_of_tiles {
                                /* Get the spectrum of the particular tile */
                                let p2_tile_spectrum = iis_igf_dec_lib_access_source_spectrum(
                                    &st.igf_static_data,
                                    &mut c.igf_data,
                                    tile_idx,
                                    0,
                                    0,
                                )
                                .expect("tile");
                                p2_tile_spectrum[..n_bins]
                                    .copy_from_slice(&mem.spec[ch * 1024..ch * 1024 + n_bins]);

                                /* Get pointer to the exponent of the particular tile */
                                let p2_tile_spectrum_exp =
                                    iis_igf_dec_lib_access_source_spectrum_exponent(
                                        &mut st.igf_static_data,
                                        tile_idx,
                                        0,
                                    );
                                p2_tile_spectrum_exp[..sfb_max_num_bins_copied].copy_from_slice(
                                    &c.p_dyn_data.a_sfb_scale[..sfb_max_num_bins_copied],
                                );
                            }
                        }
                    } /* if((elFlags & AC_EL_ENHANCED_NOISE) && (elFlags & AC_EL_IGF_USE_ENF)) */
                } /* if (flags & AC_MPEGH3DA) */
            }
        }
    }

    /* bail: */
    if crc_reg1 != -1 || crc_reg2 != -1 {
        if error == AAC_DEC_OK {
            error = AAC_DEC_DECODE_FRAME_ERROR;
        }
        if let Some(tp) = p_tp_dec {
            if crc_reg1 != -1 {
                tp.crc_end_reg(crc_reg1);
            }
            if crc_reg2 != -1 {
                tp.crc_end_reg(crc_reg2);
            }
        }
    }
    error
}

/// The IGF stereo filling of the right channel in `CChannelElement_Read()` (long blocks only).
fn igf_stereo_filling_decode(
    ch_info: &mut [Box<CAacDecoderChannelInfo>],
    st_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    mem: &mut CChannelElementMem,
    p_sampling_rate_info: &SamplingRateInfo,
    el_flags: u32,
    max_noise_sfb: i32,
) {
    let (ch_l, ch_r) = pair_mut(ch_info);
    let (st_l, st_r) = pair_mut(st_info);

    /* We use this array as a temporary one in "StereoFilling_GetPreviousDmx_IGF" and we fill it
     * again in "IGF_StereoFillingPrepare" */
    let dmx_prev_modified = mem.work_buffer_core1.mdct_out_temp();
    dmx_prev_modified.fill(0);
    let mut dmx_prev_modified_exp = [0i16; 64];

    /* We calculate the previous downmix for the fully noise-filled bands only and put it in
     * "dmx_prev_modified" */
    igf_stereo_filling_get_previous_dmx(
        &mem.cpe_static_data.as_deref().expect("pCpeStaticData").joint_stereo_persistent_data,
        mem.com_static_data.cplx_prediction_data.as_deref().expect("cplxPredictionData"),
        &st_l.concealment_info.spectral_coefficient,
        &st_r.concealment_info.spectral_coefficient,
        &st_l.concealment_info.spec_scale,
        &st_r.concealment_info.spec_scale,
        &ch_l.ics_info,
        ch_l.granule_length,
        ch_r.granule_length,
        dmx_prev_modified,
        &mut dmx_prev_modified_exp,
        get_scale_factor_band_offsets(&ch_l.ics_info, p_sampling_rate_info),
        get_window_group_length_table(&ch_l.ics_info),
        get_window_groups(&ch_l.ics_info) as i32,
        max_noise_sfb,
        &ch_r.p_dyn_data.band_is_noise,
    );

    /* Change frame-wise scaling factors to sfb-wise ones by copying them up for each and every
     * sfb */
    for i in 1..64 {
        dmx_prev_modified_exp[i] = dmx_prev_modified_exp[0];
    }

    let spec_r = &mut mem.spec[1024..2048];

    /* Stereo Filling parameter calculation */
    igf_stereo_filling_prepare(
        ch_r,
        spec_r,
        p_sampling_rate_info,
        max_noise_sfb,
        dmx_prev_modified,
        &mut dmx_prev_modified_exp,
        &ch_r.p_dyn_data.band_is_noise,
    );

    if el_flags & AC_EL_IGF_USE_ENF != 0 {
        /* Get the number of tiles */
        let number_of_tiles = iis_igf_dec_lib_get_number_of_tiles(&st_r.igf_static_data, 0);

        /* Over all tiles */
        for tile_idx in 0..number_of_tiles {
            /* Get the spectrum of the particular tile */
            let p2_tile_spectrum = iis_igf_dec_lib_access_source_spectrum(
                &st_r.igf_static_data,
                &mut ch_r.igf_data,
                tile_idx,
                0,
                0,
            )
            .expect("tile");

            /* Get pointer to the exponent of the particular tile */
            let p2_tile_spectrum_exp = iis_igf_dec_lib_access_source_spectrum_exponent(
                &mut st_r.igf_static_data,
                tile_idx,
                0,
            );

            /* Apply Stereo Filling to the particular tile */
            igf_stereo_filling_apply(
                &ch_r.ics_info,
                &ch_r.p_dyn_data.a_scale_factor,
                p2_tile_spectrum,
                p2_tile_spectrum_exp,
                p_sampling_rate_info,
                max_noise_sfb,
                dmx_prev_modified,
                &dmx_prev_modified_exp,
                &ch_r.p_dyn_data.band_is_noise,
            );
        }
    }

    /* Get the sfb scale data */
    let d = &mut *ch_r.p_dyn_data;

    /* Stereo Filling application */
    igf_stereo_filling_apply(
        &ch_r.ics_info,
        &d.a_scale_factor,
        spec_r,
        &mut d.a_sfb_scale,
        p_sampling_rate_info,
        max_noise_sfb,
        dmx_prev_modified,
        &dmx_prev_modified_exp,
        &d.band_is_noise,
    );
}
