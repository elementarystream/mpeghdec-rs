// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/stereo.h, libMpeghDec/src/stereo.cpp and
// libMpeghDec/src/arm/stereo_arm.cpp (its NEON CJointStereo_GenerateMSOutput is not compiled in
// the reference configuration).

//! Joint stereo processing: M/S, complex stereo prediction and IGF stereo.
//!
//! The C++ functions take the channel structures of the element and reach the joint stereo data
//! through pointers (`pAacDecoderChannelInfo[L]->pComStaticData->pJointStereoData` /
//! `cplxPredictionData`, `pAacDecoderStaticChannelInfo[L]->pCpeStaticData`). The Rust functions
//! take the used fields and structures as separate arguments; the argument documentation names
//! the C++ expression.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::{
    FixpFilt, AAC_CODE_BOOK_DESCRIPTION_SCL, MDST_FILT_COEF_CURR, MDST_FILT_COEF_PREV,
    SF_FNA_COEFFS,
};
use crate::block::cblock_decode_huffman_word;
use crate::channelinfo::{is_long_block, CIcsInfo};
use mpeghdec_fdk::audio::{AC_INDEP, AC_MPEGH3DA, AC_RSVD50, AC_USAC};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::f_negate_saturate;
use mpeghdec_fdk::mdct::{BLOCK_LONG, BLOCK_SHORT, BLOCK_START, BLOCK_STOP, SHAPE_KBD, SHAPE_SINE};
use mpeghdec_fdk::scale::{
    saturate_left_shift, saturate_left_shift_alt, saturate_shift, scale_values, scale_values_to,
};
use mpeghdec_igf::igf_dec::IgfPrivateStaticData;

/// `SFB_PER_PRED_BAND`
pub const SFB_PER_PRED_BAND: i32 = 2;

/// `SR_FNA_OUT`: additional scaling of the `CJointStereo_filterAndAdd()` output to avoid
/// overflows (0: the coefficients are prescaled, see `SF_FNA_COEFFS`).
pub const SR_FNA_OUT: i32 = 0;

/// `JointStereoMaximumGroups`
pub const JOINT_STEREO_MAXIMUM_GROUPS: usize = 8;
/// `JointStereoMaximumBands`
pub const JOINT_STEREO_MAXIMUM_BANDS: usize = 64;

/// `CCplxPredictionData`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CCplxPredictionData {
    /// `pred_dir`: 0 = prediction from mid to side channel, 1 = vice versa
    pub pred_dir: u8,
    /// `igf_pred_dir`: 0 = prediction from mid to side channel, 1 = vice versa
    pub igf_pred_dir: u8,
    /// `complex_coef`: 0 = `alpha_q_im[x]` is 0 for all prediction bands, 1 = `alpha_q_im[x]` is
    /// transmitted via bitstream
    pub complex_coef: u8,
    /// `use_prev_frame`: 0 = use current frame for MDST estimation, 1 = use current and previous
    /// frame
    pub use_prev_frame: u8,
    /// `alpha_q_re`
    ///
    /// Not part of the state digest: only read for the bands written in the same frame
    /// (`CJointStereo_Read()` writes the bands below `max_sfb_ste` of all groups, which
    /// `CJointStereo_ApplyMS()` reads; `CJointStereo_ReadIGF()` writes the IGF bands and above,
    /// which `CJointStereo_ApplyMS_IGF()` reads; both only with the prediction flag of the frame).
    #[digest(skip)]
    pub alpha_q_re: [[i16; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
    /// `alpha_q_im`
    ///
    /// State digest: only the IGF bands of the element, from the lowest IGF start band to the
    /// highest IGF stop band of the long and short blocks (fed by `AacDecoderInstance`, which
    /// knows the IGF configuration): `CJointStereo_ReadIGF()` copies them to the persistent
    /// `alpha_q_im_prev` without writing them (they may be left from an earlier frame). The other
    /// bands are only read by `CJointStereo_ApplyMS()` below `max_sfb_ste` (at most the IGF
    /// start band) after `CJointStereo_Read()` wrote them in the same frame.
    #[digest(skip)]
    pub alpha_q_im: [[i16; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
}

impl Default for CCplxPredictionData {
    fn default() -> Self {
        CCplxPredictionData {
            pred_dir: 0,
            igf_pred_dir: 0,
            complex_coef: 0,
            use_prev_frame: 0,
            alpha_q_re: [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
            alpha_q_im: [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
        }
    }
}

/// `CJointStereoData`: joint stereo scratch memory (valid for this frame), one for one CPE
/// (`pComStaticData->pJointStereoData`). Note: the C++ decoder allocates it with `FDKmalloc()`,
/// i.e. its initial content is undefined; here it is zero-initialized.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CJointStereoData {
    /// `MsMaskPresent`
    pub ms_mask_present: u8,
    /// `MsUsed`: every array element contains flags for up to 8 groups. This array is also
    /// utilized for complex stereo prediction.
    pub ms_used: [u8; JOINT_STEREO_MAXIMUM_BANDS],
    /// `cplx_pred_flag`: stereo complex prediction was signalled for this frame
    pub cplx_pred_flag: u8,
    /// `IGF_MsMaskPresent`
    pub igf_ms_mask_present: u8,
    /// `igf_cplx_pred_flag`
    pub igf_cplx_pred_flag: u8,
    /// `store_dmx_re_prev`: needed for the case when INF is active
    pub store_dmx_re_prev: [FixpDbl; 1024],
    /// `store_dmx_re_prev_e`
    pub store_dmx_re_prev_e: i16,
}

impl Default for CJointStereoData {
    fn default() -> Self {
        CJointStereoData {
            ms_mask_present: 0,
            ms_used: [0; JOINT_STEREO_MAXIMUM_BANDS],
            cplx_pred_flag: 0,
            igf_ms_mask_present: 0,
            igf_cplx_pred_flag: 0,
            store_dmx_re_prev: [0; 1024],
            store_dmx_re_prev_e: 0,
        }
    }
}

/// `CJointStereoPersistentData`: joint stereo persistent memory
/// (`CpePersistentData::jointStereoPersistentData`).
///
/// The C++ pointers are not stored (see the [crate documentation](crate)):
/// - `scratchBuffer` is `CWorkBufferCore1::workBuffer` (2048 values),
/// - `spectralCoeffs[ch]` / `specScale[ch]` are the concealment spectrum and scale of the
///   channels of the element (`CConcealmentInfo::spectral_coefficient` / `spec_scale`),
/// - `scratchBuffer2` is `pTimeData2[2 * 1024..3 * 1024]`.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CJointStereoPersistentData {
    /// `clearSpectralCoeffs`: indicates that the spectral coeffs must be cleared because the
    /// transform splitting active flag of the left and right channel was different
    pub clear_spectral_coeffs: u8,
    /// `alpha_q_re_prev`
    pub alpha_q_re_prev: [[i16; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
    /// `alpha_q_im_prev`
    pub alpha_q_im_prev: [[i16; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
    /// `winSeqPrev`
    pub win_seq_prev: u8,
    /// `winShapePrev`
    pub win_shape_prev: u8,
    /// `winGroupsPrev`
    pub win_groups_prev: u8,
}

impl Default for CJointStereoPersistentData {
    fn default() -> Self {
        CJointStereoPersistentData {
            clear_spectral_coeffs: 0,
            alpha_q_re_prev: [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
            alpha_q_im_prev: [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS],
            win_seq_prev: 0,
            win_shape_prev: 0,
            win_groups_prev: 0,
        }
    }
}

/// `CJointStereo_Read(bs, pJointStereoData, windowGroups, scaleFactorBandsTransmitted,
/// max_sfb_ste_clear, pJointStereoPersistentData, cplxPredictionData, cplxPredictionActiv,
/// scaleFactorBandsTotal, windowSequence, flags)`: reads joint stereo data from the bitstream.
///
/// `p_joint_stereo_persistent_data` and `cplx_prediction_data` are only available if
/// `cplx_prediction_activ` is set (`NULL` otherwise). Returns 0 on success, -1 on error.
pub fn cjoint_stereo_read(
    bs: &mut FdkBitStream,
    p_joint_stereo_data: &mut CJointStereoData,
    window_groups: i32,
    scale_factor_bands_transmitted: i32,
    max_sfb_ste_clear: i32,
    p_joint_stereo_persistent_data: Option<&mut CJointStereoPersistentData>,
    mut cplx_prediction_data: Option<&mut CCplxPredictionData>,
    cplx_prediction_activ: i32,
    scale_factor_bands_total: i32,
    window_sequence: i32,
    flags: u32,
) -> i32 {
    let jsd = p_joint_stereo_data;
    let sfb_tr = scale_factor_bands_transmitted.max(0) as usize;

    jsd.ms_mask_present = bs.read_2bits() as u8;

    jsd.ms_used[..sfb_tr].fill(0);

    jsd.cplx_pred_flag = 0;
    if cplx_prediction_activ != 0 {
        let cpd = cplx_prediction_data.as_deref_mut().expect("cplxPredictionData");
        cpd.pred_dir = 0;
        cpd.complex_coef = 0;
        cpd.use_prev_frame = 0;
        cpd.igf_pred_dir = 0;
    }

    match jsd.ms_mask_present {
        0 => { /* no M/S */ /* all flags are already cleared */ }
        1 => {
            /* read ms_used */
            for group in 0..window_groups {
                for band in 0..sfb_tr {
                    jsd.ms_used[band] |= (bs.read_bit() << group) as u8;
                }
            }
        }
        2 => {
            /* full spectrum M/S */
            for band in 0..sfb_tr {
                jsd.ms_used[band] = 255; /* set all flags to 1 */
            }
        }
        _ => {
            /* M/S coding is disabled, complex stereo prediction is enabled */
            if flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 {
                if cplx_prediction_activ != 0 {
                    /* 'if (stereoConfigIndex == 0)' */

                    jsd.cplx_pred_flag = 1;

                    /* cplx_pred_data()  cp. ISO/IEC FDIS 23003-3:2011(E)  Table 26 */
                    let cplx_pred_all = bs.read_bit(); /* local use only */

                    if cplx_pred_all != 0 {
                        for group in 0..window_groups {
                            let groupmask = 1u8.wrapping_shl(group as u32);
                            for band in 0..sfb_tr {
                                jsd.ms_used[band] |= groupmask;
                            }
                        }
                    } else {
                        for group in 0..window_groups {
                            let mut band = 0;
                            while band < sfb_tr {
                                jsd.ms_used[band] |= (bs.read_bit() << group) as u8;
                                if ((band + 1) as i32) < scale_factor_bands_total {
                                    jsd.ms_used[band + 1] |=
                                        jsd.ms_used[band] & 1u8.wrapping_shl(group as u32);
                                }
                                band += SFB_PER_PRED_BAND as usize;
                            }
                        }
                    }
                } else {
                    return -1;
                }
            }
        }
    }

    if cplx_prediction_activ != 0 {
        let jspd = p_joint_stereo_persistent_data.expect("pJointStereoPersistentData");
        let cpd = cplx_prediction_data.expect("cplxPredictionData");
        /* If all sfb are MS-ed then no complex prediction */
        if jsd.ms_mask_present == 3 {
            if jsd.cplx_pred_flag != 0 {
                /* set pointer to Huffman codebooks */
                let hcb = &AAC_CODE_BOOK_DESCRIPTION_SCL;
                /* set predictors to zero in case of a transition from long to short window
                 * sequences and vice versa */
                if ((window_sequence == BLOCK_SHORT) && (jspd.win_seq_prev as i32 != BLOCK_SHORT))
                    || ((jspd.win_seq_prev as i32 == BLOCK_SHORT)
                        && (window_sequence != BLOCK_SHORT))
                {
                    jspd.alpha_q_re_prev =
                        [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS];
                    jspd.alpha_q_im_prev =
                        [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS];
                }
                if flags & AC_MPEGH3DA != 0 {
                    /* Clean the array up to the maximum SFB transmitted. The rest is not cleaned,
                     * because it may be used by IGF complex prediction */
                    for group in 0..JOINT_STEREO_MAXIMUM_GROUPS {
                        for band in 0..sfb_tr {
                            cpd.alpha_q_re[group][band] = 0;
                            cpd.alpha_q_im[group][band] = 0;
                        }
                    }
                } else {
                    cpd.alpha_q_re = [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS];
                    cpd.alpha_q_im = [[0; JOINT_STEREO_MAXIMUM_BANDS]; JOINT_STEREO_MAXIMUM_GROUPS];
                }

                /* 0 = mid->side prediction, 1 = side->mid prediction */
                cpd.pred_dir = bs.read_bit() as u8;
                cpd.complex_coef = bs.read_bit() as u8;

                if cpd.complex_coef != 0 {
                    if flags & AC_INDEP != 0 {
                        cpd.use_prev_frame = 0;
                    } else {
                        cpd.use_prev_frame = bs.read_bit() as u8;
                    }
                }

                let delta_code_time = if flags & AC_INDEP != 0 { 0 } else { bs.read_bit() };

                {
                    let mut last_alpha_q_re: i32;
                    let mut last_alpha_q_im: i32;

                    for group in 0..window_groups.max(0) as usize {
                        let mut band = 0usize;
                        while band < sfb_tr {
                            if delta_code_time == 1 {
                                if group > 0 {
                                    last_alpha_q_re = cpd.alpha_q_re[group - 1][band] as i32;
                                    last_alpha_q_im = cpd.alpha_q_im[group - 1][band] as i32;
                                } else if (window_sequence == BLOCK_SHORT)
                                    && (jspd.win_seq_prev as i32 == BLOCK_SHORT)
                                {
                                    debug_assert!(jspd.win_groups_prev > 0);
                                    let gp = jspd.win_groups_prev as usize - 1;
                                    last_alpha_q_re = jspd.alpha_q_re_prev[gp][band] as i32;
                                    last_alpha_q_im = jspd.alpha_q_im_prev[gp][band] as i32;
                                } else {
                                    last_alpha_q_re = jspd.alpha_q_re_prev[group][band] as i32;
                                    last_alpha_q_im = jspd.alpha_q_im_prev[group][band] as i32;
                                }
                            } else if band > 0 {
                                last_alpha_q_re = cpd.alpha_q_re[group][band - 1] as i32;
                                last_alpha_q_im = cpd.alpha_q_im[group][band - 1] as i32;
                            } else {
                                last_alpha_q_re = 0;
                                last_alpha_q_im = 0;
                            } /* if (delta_code_time == 1) */

                            if jsd.ms_used[band] & 1u8.wrapping_shl(group as u32) != 0 {
                                let mut dpcm_alpha_re = cblock_decode_huffman_word(bs, hcb);
                                dpcm_alpha_re -= 60;
                                dpcm_alpha_re *= -1;

                                cpd.alpha_q_re[group][band] =
                                    (dpcm_alpha_re + last_alpha_q_re) as i16;

                                if cpd.complex_coef != 0 {
                                    let mut dpcm_alpha_im = cblock_decode_huffman_word(bs, hcb);
                                    dpcm_alpha_im -= 60;
                                    dpcm_alpha_im *= -1;

                                    cpd.alpha_q_im[group][band] =
                                        (dpcm_alpha_im + last_alpha_q_im) as i16;
                                } else {
                                    cpd.alpha_q_im[group][band] = 0;
                                }
                            } else {
                                cpd.alpha_q_re[group][band] = 0;
                                cpd.alpha_q_im[group][band] = 0;
                            } /* if (pJointStereoData->MsUsed[band] & ((UCHAR)1 << group)) */

                            if band + 1 < sfb_tr {
                                /* <= this should be the correct way (cp. ISO_IEC_FDIS_23003-0(E) */
                                /*    7.7.2.3.2 Decoding of prediction coefficients) */
                                cpd.alpha_q_re[group][band + 1] = cpd.alpha_q_re[group][band];
                                cpd.alpha_q_im[group][band + 1] = cpd.alpha_q_im[group][band];
                            } /* if ((band+1)<scaleFactorBandsTotal) */

                            jspd.alpha_q_re_prev[group][band] = cpd.alpha_q_re[group][band];
                            jspd.alpha_q_im_prev[group][band] = cpd.alpha_q_im[group][band];
                            band += SFB_PER_PRED_BAND as usize;
                        }

                        for band in sfb_tr..max_sfb_ste_clear.max(0) as usize {
                            cpd.alpha_q_re[group][band] = 0;
                            cpd.alpha_q_im[group][band] = 0;
                            jspd.alpha_q_re_prev[group][band] = 0;
                            jspd.alpha_q_im_prev[group][band] = 0;
                        }
                    }
                }
            }
        } else {
            for group in 0..window_groups.max(0) as usize {
                for band in 0..max_sfb_ste_clear.max(0) as usize {
                    jspd.alpha_q_re_prev[group][band] = 0;
                    jspd.alpha_q_im_prev[group][band] = 0;
                }
            }
        }

        jspd.win_groups_prev = window_groups as u8;
    }

    0
}

/// `CJointStereo_ReadIGF(bs, pJointStereoData, windowGroups, igfStartSfb, igfStopSfb,
/// igfUseHighRes, pJointStereoPersistentData, cplxPredictionData, windowSequence, flags)`: reads
/// the IGF joint stereo data. Returns 0.
pub fn cjoint_stereo_read_igf(
    bs: &mut FdkBitStream,
    p_joint_stereo_data: &mut CJointStereoData,
    window_groups: i32,
    igf_start_sfb: i32,
    igf_stop_sfb: i32,
    igf_use_high_res: i32,
    p_joint_stereo_persistent_data: &mut CJointStereoPersistentData,
    cplx_prediction_data: &mut CCplxPredictionData,
    window_sequence: i32,
    flags: u32,
) -> i32 {
    let jsd = p_joint_stereo_data;
    let jspd = p_joint_stereo_persistent_data;
    let cpd = cplx_prediction_data;
    let start = igf_start_sfb as usize;
    let stop = igf_stop_sfb.max(0) as usize;

    jsd.igf_ms_mask_present = bs.read_2bits() as u8;

    jsd.ms_used[start..JOINT_STEREO_MAXIMUM_BANDS].fill(0);

    jsd.igf_cplx_pred_flag = 0;

    match jsd.igf_ms_mask_present {
        0 => {
            /* no IGF M/S */
            /* all flags are already cleared */
            for sfb in start..stop {
                jsd.ms_used[sfb] = 0; /* set all flags to 0 */
            }
        }
        1 => {
            /* read IGF ms_used */
            for group in 0..window_groups {
                let mut sfb = start;
                while sfb < stop {
                    jsd.ms_used[sfb] |= (bs.read_bit() << group) as u8;
                    if igf_use_high_res == 0 && sfb + 1 < stop {
                        jsd.ms_used[sfb + 1] = jsd.ms_used[sfb];
                    }
                    sfb += (2 - igf_use_high_res) as usize;
                }
            }
        }
        2 => {
            /* IGF full spectrum M/S */
            for sfb in start..stop {
                jsd.ms_used[sfb] = 255; /* set all flags to 1 */
            }
        }
        _ => {
            jsd.igf_cplx_pred_flag = 1;

            let igf_stereo_pred_all = bs.read_bit();

            if igf_stereo_pred_all != 0 {
                for group in 0..window_groups {
                    let groupmask = 1u8.wrapping_shl(group as u32);
                    for sfb in start..stop {
                        jsd.ms_used[sfb] |= groupmask;
                    }
                }
            } else {
                for group in 0..window_groups {
                    let mut sfb = start;
                    while sfb < stop {
                        jsd.ms_used[sfb] |= (bs.read_bit() << group) as u8;
                        if sfb + 1 < stop {
                            jsd.ms_used[sfb + 1] = jsd.ms_used[sfb];
                        }
                        sfb += SFB_PER_PRED_BAND as usize;
                    }
                }
            }
        }
    } /* end switch */

    if jsd.igf_ms_mask_present != 3 {
        return 0;
    }

    if jsd.igf_cplx_pred_flag != 0 {
        /* set pointer to Huffman codebooks */
        let hcb = &AAC_CODE_BOOK_DESCRIPTION_SCL;
        /* set predictors to zero in case of a transition from long to short window sequences and
         * vice versa */
        if ((window_sequence == BLOCK_SHORT) && (jspd.win_seq_prev as i32 != BLOCK_SHORT))
            || ((jspd.win_seq_prev as i32 == BLOCK_SHORT) && (window_sequence != BLOCK_SHORT))
        {
            for group in 0..window_groups.max(0) as usize {
                for sfb in start..stop {
                    jspd.alpha_q_re_prev[group][sfb] = 0;
                }
            }
        }

        /* make sure prediction coefficients are reset */
        for group in 0..window_groups.max(0) as usize {
            /* Clean the IGF used part of the array. The lower part of the array is used by
             * standard MS/Complex stereo prediction */
            for sfb in start..stop {
                cpd.alpha_q_re[group][sfb] = 0;
            }
        }

        /* 0 = mid->side prediction, 1 = side->mid prediction */
        cpd.igf_pred_dir = bs.read_bit() as u8;

        let igf_delta_code_time = if flags & AC_INDEP != 0 { 0 } else { bs.read_bit() };

        let mut last_alpha_q_re: i32;

        for group in 0..window_groups.max(0) as usize {
            let mut sfb = start;
            while sfb < stop {
                if igf_delta_code_time == 1 {
                    if group > 0 {
                        last_alpha_q_re = cpd.alpha_q_re[group - 1][sfb] as i32;
                    } else {
                        debug_assert!(jspd.win_groups_prev > 0);
                        last_alpha_q_re =
                            jspd.alpha_q_re_prev[jspd.win_groups_prev as usize - 1][sfb] as i32;
                    }
                } else if sfb > start {
                    last_alpha_q_re = cpd.alpha_q_re[group][sfb - 1] as i32;
                } else {
                    last_alpha_q_re = 0;
                } /* if (delta_code_time == 1) */

                if jsd.ms_used[sfb] & 1u8.wrapping_shl(group as u32) != 0 {
                    let mut dpcm_alpha_re = cblock_decode_huffman_word(bs, hcb);
                    dpcm_alpha_re -= 60;
                    dpcm_alpha_re *= -1;

                    cpd.alpha_q_re[group][sfb] = (dpcm_alpha_re + last_alpha_q_re) as i16;
                } else {
                    cpd.alpha_q_re[group][sfb] = 0;
                } /* if (pJointStereoData->MsUsed[band] & ((UCHAR)1 << group)) */

                if sfb + 1 < stop {
                    /* <= this should be the correct way (cp. ISO_IEC_FDIS_23003-0(E) */
                    cpd.alpha_q_re[group][sfb + 1] = cpd.alpha_q_re[group][sfb];
                } /* (sfb+1)< igfStopSfb  */

                jspd.alpha_q_re_prev[group][sfb] = cpd.alpha_q_re[group][sfb];
                jspd.alpha_q_im_prev[group][sfb] = cpd.alpha_q_im[group][sfb];

                sfb += SFB_PER_PRED_BAND as usize;
            } /* for (sfb=igfStartSfb; sfb<igfStopSfb;sfb+=SFB_PER_PRED_BAND) */

            for sfb in stop..JOINT_STEREO_MAXIMUM_BANDS {
                cpd.alpha_q_re[group][sfb] = 0;
                cpd.alpha_q_im[group][sfb] = 0;
                jspd.alpha_q_re_prev[group][sfb] = 0;
                jspd.alpha_q_im_prev[group][sfb] = 0;
            }
        } /* for (group = 0; group<windowGroups; group++) */
    }

    jspd.win_groups_prev = window_groups as u8;

    0
}

const INDICES_1: [i32; 6] = [2, 1, 0, 1, 2, 3];
const INDICES_2: [i32; 6] = [1, 0, 0, 2, 3, 4];
const INDICES_3: [i32; 6] = [0, 0, 1, 3, 4, 5];

const SUBTR_1: [i32; 6] = [6, 5, 4, 2, 1, 1];
const SUBTR_2: [i32; 6] = [5, 4, 3, 1, 1, 2];
const SUBTR_3: [i32; 6] = [4, 3, 2, 1, 2, 3];

/// `fMultDiv2(coeff, x)` of the MDST filter (`FIXP_FILT` = `FIXP_DBL`).
#[inline(always)]
fn fmd2(c: FixpFilt, x: FixpDbl) -> FixpDbl {
    f_mult_div2(c, x)
}

/// `CJointStereo_filterAndAdd(in, len, windowLen, coeff, out, isCurrent)`: output values with
/// even index get a positive addon (`isCurrent == 1`) or a negative addon (`isCurrent == 0`).
///
/// `inp[..len]` and `out[..len]` are accessed (`in` and `out` never overlap in the C++ code).
fn cjoint_stereo_filter_and_add(
    inp: &[FixpDbl],
    len: i32,
    _window_len: i32,
    coeff: &[FixpFilt],
    out: &mut [FixpDbl],
    is_current: u8,
) {
    let l = len;
    let at = |i: i32| inp[i as usize];
    let sub = |o: &mut FixpDbl, v: FixpDbl| *o = o.wrapping_sub(v);
    let add = |o: &mut FixpDbl, v: FixpDbl| *o = o.wrapping_add(v);

    if is_current == 1 {
        /* exploit the symmetry of the table: coeff[6] = - coeff[0],
                                              coeff[5] = - coeff[1],
                                              coeff[4] = - coeff[2],
                                              coeff[3] = 0
        */

        for i in 0..3 {
            sub(&mut out[0], fmd2(coeff[i], at(INDICES_1[i])) >> SR_FNA_OUT);
            add(&mut out[0], fmd2(coeff[i], at(INDICES_1[5 - i])) >> SR_FNA_OUT);
        }

        for i in 0..3 {
            sub(&mut out[1], fmd2(coeff[i], at(INDICES_2[i])) >> SR_FNA_OUT);
            add(&mut out[1], fmd2(coeff[i], at(INDICES_2[5 - i])) >> SR_FNA_OUT);
        }

        for i in 0..3 {
            sub(&mut out[2], fmd2(coeff[i], at(INDICES_3[i])) >> SR_FNA_OUT);
            add(&mut out[2], fmd2(coeff[i], at(INDICES_3[5 - i])) >> SR_FNA_OUT);
        }

        for j in 3..(l - 3) {
            for i in 0..3 {
                sub(&mut out[j as usize], fmd2(coeff[i], at(j - 3 + i as i32)) >> SR_FNA_OUT);
                add(&mut out[j as usize], fmd2(coeff[i], at(j + 3 - i as i32)) >> SR_FNA_OUT);
            }
        }

        let o = (l - 3) as usize;
        for i in 0..3 {
            sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_1[i])) >> SR_FNA_OUT);
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_1[5 - i])) >> SR_FNA_OUT);
        }

        let o = (l - 2) as usize;
        for i in 0..3 {
            sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_2[i])) >> SR_FNA_OUT);
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_2[5 - i])) >> SR_FNA_OUT);
        }

        let o = (l - 1) as usize;
        for i in 0..3 {
            sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_3[i])) >> SR_FNA_OUT);
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_3[5 - i])) >> SR_FNA_OUT);
        }
    } else {
        /* exploit the symmetry of the table: coeff[6] = coeff[0],
                                              coeff[5] = coeff[1],
                                              coeff[4] = coeff[2]
        */

        for i in 0..3 {
            sub(&mut out[0], fmd2(coeff[i], at(INDICES_1[i]) >> SR_FNA_OUT));
            sub(&mut out[0], fmd2(coeff[i], at(INDICES_1[5 - i]) >> SR_FNA_OUT));
        }
        sub(&mut out[0], fmd2(coeff[3], at(0) >> SR_FNA_OUT));

        for i in 0..3 {
            add(&mut out[1], fmd2(coeff[i], at(INDICES_2[i]) >> SR_FNA_OUT));
            add(&mut out[1], fmd2(coeff[i], at(INDICES_2[5 - i]) >> SR_FNA_OUT));
        }
        add(&mut out[1], fmd2(coeff[3], at(1) >> SR_FNA_OUT));

        for i in 0..3 {
            sub(&mut out[2], fmd2(coeff[i], at(INDICES_3[i]) >> SR_FNA_OUT));
            sub(&mut out[2], fmd2(coeff[i], at(INDICES_3[5 - i]) >> SR_FNA_OUT));
        }
        sub(&mut out[2], fmd2(coeff[3], at(2) >> SR_FNA_OUT));

        let mut j = 3;
        while j < l - 4 {
            for i in 0..3 {
                add(&mut out[j as usize], fmd2(coeff[i], at(j - 3 + i as i32) >> SR_FNA_OUT));
                add(&mut out[j as usize], fmd2(coeff[i], at(j + 3 - i as i32) >> SR_FNA_OUT));
            }
            add(&mut out[j as usize], fmd2(coeff[3], at(j) >> SR_FNA_OUT));

            j += 1;

            for i in 0..3 {
                sub(&mut out[j as usize], fmd2(coeff[i], at(j - 3 + i as i32) >> SR_FNA_OUT));
                sub(&mut out[j as usize], fmd2(coeff[i], at(j + 3 - i as i32) >> SR_FNA_OUT));
            }
            sub(&mut out[j as usize], fmd2(coeff[3], at(j) >> SR_FNA_OUT));
            j += 1;
        }

        let o = (l - 3) as usize;
        for i in 0..3 {
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_1[i]) >> SR_FNA_OUT));
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_1[5 - i]) >> SR_FNA_OUT));
        }
        add(&mut out[o], fmd2(coeff[3], at(l - 3) >> SR_FNA_OUT));

        let o = (l - 2) as usize;
        for i in 0..3 {
            sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_2[i]) >> SR_FNA_OUT));
            sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_2[5 - i]) >> SR_FNA_OUT));
        }
        sub(&mut out[o], fmd2(coeff[3], at(l - 2) >> SR_FNA_OUT));

        let o = (l - 1) as usize;
        for i in 0..3 {
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_3[i]) >> SR_FNA_OUT));
            add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_3[5 - i]) >> SR_FNA_OUT));
        }
        add(&mut out[o], fmd2(coeff[3], at(l - 1) >> SR_FNA_OUT));
    }
}

/// `CJointStereo_filterAndAdd_Int(in, len, windowLen, halfLowpassLine, coeff, out, isCurrent)`:
/// [`cjoint_stereo_filter_and_add`] on the interleaved spectrum of transform splitting (every
/// second value of `in` and `out`).
fn cjoint_stereo_filter_and_add_int(
    inp: &[FixpDbl],
    len: i32,
    _window_len: i32,
    half_lowpass_line: i32,
    coeff: &[FixpFilt],
    out: &mut [FixpDbl],
    is_current: u8,
) {
    let l = len;
    let h = half_lowpass_line;
    let at = |i: i32| inp[(i << 1) as usize];
    let sub = |o: &mut FixpDbl, v: FixpDbl| *o = o.wrapping_sub(v);
    let add = |o: &mut FixpDbl, v: FixpDbl| *o = o.wrapping_add(v);
    let oi = |j: i32| (j << 1) as usize;

    if is_current == 1 {
        for i in 0..3 {
            if INDICES_1[i] < h {
                sub(&mut out[0], fmd2(coeff[i], at(INDICES_1[i])) >> SR_FNA_OUT);
            }
            if INDICES_1[5 - i] < h {
                add(&mut out[0], fmd2(coeff[i], at(INDICES_1[5 - i])) >> SR_FNA_OUT);
            }
        }

        for i in 0..3 {
            if INDICES_2[i] < h {
                sub(&mut out[2], fmd2(coeff[i], at(INDICES_2[i])) >> SR_FNA_OUT);
            }
            if INDICES_2[5 - i] < h {
                add(&mut out[2], fmd2(coeff[i], at(INDICES_2[5 - i])) >> SR_FNA_OUT);
            }
        }

        for i in 0..3 {
            if INDICES_3[i] < h {
                sub(&mut out[4], fmd2(coeff[i], at(INDICES_3[i])) >> SR_FNA_OUT);
            }
            if INDICES_3[5 - i] < h {
                add(&mut out[4], fmd2(coeff[i], at(INDICES_3[5 - i])) >> SR_FNA_OUT);
            }
        }

        for j in 3..(l - 3) {
            for i in 0..3 {
                let ii = i as i32;
                if (j - 3 + ii) < h {
                    sub(&mut out[oi(j)], fmd2(coeff[i], at(j - 3 + ii)) >> SR_FNA_OUT);
                }
                if (j + 3 - ii) < h {
                    add(&mut out[oi(j)], fmd2(coeff[i], at(j + 3 - ii)) >> SR_FNA_OUT);
                }
            }
        }

        for (k, subtr) in [(3, &SUBTR_1), (2, &SUBTR_2), (1, &SUBTR_3)] {
            let o = oi(l - k);
            for i in 0..3 {
                if (l - subtr[i]) < h {
                    sub(&mut out[o], fmd2(coeff[i], at(l - subtr[i])) >> SR_FNA_OUT);
                }
                if (l - subtr[5 - i]) < h {
                    add(&mut out[o], fmd2(coeff[i], at(l - subtr[5 - i])) >> SR_FNA_OUT);
                }
            }
        }
    } else {
        for i in 0..3 {
            if INDICES_1[i] < h {
                sub(&mut out[0], fmd2(coeff[i], at(INDICES_1[i]) >> SR_FNA_OUT));
            }
            if INDICES_1[5 - i] < h {
                sub(&mut out[0], fmd2(coeff[i], at(INDICES_1[5 - i]) >> SR_FNA_OUT));
            }
        }
        if 0 < h {
            sub(&mut out[0], fmd2(coeff[3], at(0) >> SR_FNA_OUT));
        }

        for i in 0..3 {
            if INDICES_2[i] < h {
                add(&mut out[2], fmd2(coeff[i], at(INDICES_2[i]) >> SR_FNA_OUT));
            }
            if INDICES_2[5 - i] < h {
                add(&mut out[2], fmd2(coeff[i], at(INDICES_2[5 - i]) >> SR_FNA_OUT));
            }
        }
        if 1 < h {
            add(&mut out[oi(1)], fmd2(coeff[3], at(1) >> SR_FNA_OUT));
        }

        for i in 0..3 {
            if INDICES_3[i] < h {
                sub(&mut out[4], fmd2(coeff[i], at(INDICES_3[i]) >> SR_FNA_OUT));
            }
            if INDICES_3[5 - i] < h {
                sub(&mut out[4], fmd2(coeff[i], at(INDICES_3[5 - i]) >> SR_FNA_OUT));
            }
        }
        if 2 < h {
            sub(&mut out[oi(2)], fmd2(coeff[3], at(2) >> SR_FNA_OUT));
        }

        let mut j = 3;
        while j < l - 4 {
            for i in 0..3 {
                let ii = i as i32;
                if (j - 3 + ii) < h {
                    add(&mut out[oi(j)], fmd2(coeff[i], at(j - 3 + ii) >> SR_FNA_OUT));
                }
                if (j + 3 - ii) < h {
                    add(&mut out[oi(j)], fmd2(coeff[i], at(j + 3 - ii) >> SR_FNA_OUT));
                }
            }
            if j < h {
                add(&mut out[oi(j)], fmd2(coeff[3], at(j) >> SR_FNA_OUT));
            }

            j += 1;

            for i in 0..3 {
                let ii = i as i32;
                if (j - 3 + ii) < h {
                    sub(&mut out[oi(j)], fmd2(coeff[i], at(j - 3 + ii) >> SR_FNA_OUT));
                }
                if (j + 3 - ii) < h {
                    sub(&mut out[oi(j)], fmd2(coeff[i], at(j + 3 - ii) >> SR_FNA_OUT));
                }
            }
            if j < h {
                sub(&mut out[oi(j)], fmd2(coeff[3], at(j) >> SR_FNA_OUT));
            }
            j += 1;
        }

        let o = oi(l - 3);
        for i in 0..3 {
            if (l - SUBTR_1[i]) < h {
                add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_1[i]) >> SR_FNA_OUT));
            }
            if (l - SUBTR_1[5 - i]) < h {
                add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_1[5 - i]) >> SR_FNA_OUT));
            }
        }
        if (l - 3) < h {
            add(&mut out[o], fmd2(coeff[3], at(l - 3) >> SR_FNA_OUT));
        }

        let o = oi(l - 2);
        for i in 0..3 {
            if (l - SUBTR_2[i]) < h {
                sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_2[i]) >> SR_FNA_OUT));
            }
            if (l - SUBTR_2[5 - i]) < h {
                sub(&mut out[o], fmd2(coeff[i], at(l - SUBTR_2[5 - i]) >> SR_FNA_OUT));
            }
        }
        if (l - 2) < h {
            sub(&mut out[o], fmd2(coeff[3], at(l - 2) >> SR_FNA_OUT));
        }

        let o = oi(l - 1);
        for i in 0..3 {
            if (l - SUBTR_3[i]) < h {
                add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_3[i]) >> SR_FNA_OUT));
            }
            if (l - SUBTR_3[5 - i]) < h {
                add(&mut out[o], fmd2(coeff[i], at(l - SUBTR_3[5 - i]) >> SR_FNA_OUT));
            }
        }
        if (l - 1) < h {
            add(&mut out[o], fmd2(coeff[3], at(l - 1) >> SR_FNA_OUT));
        }
    }
}

/// `CJointStereo_GenerateMSOutput(pSpecLCurrBand, pSpecRCurrBand, leftScale, rightScale,
/// nSfbBands)` (generic C++ variant, processes 4 values per iteration from the end).
#[inline]
fn cjoint_stereo_generate_ms_output(
    p_spec_l_curr_band: &mut [FixpDbl],
    p_spec_r_curr_band: &mut [FixpDbl],
    left_scale: u32,
    right_scale: u32,
    n_sfb_bands: u32,
) {
    for i in (0..n_sfb_bands as usize).rev() {
        /* MS output generation */
        let left_coefficient = p_spec_l_curr_band[i] >> left_scale;
        let right_coefficient = p_spec_r_curr_band[i] >> right_scale;

        p_spec_l_curr_band[i] = left_coefficient.wrapping_add(right_coefficient);
        p_spec_r_curr_band[i] = left_coefficient.wrapping_sub(right_coefficient);
    }
}

/// The complex prediction data of `CJointStereo_ApplyMS()` that the C++ code reaches through the
/// channel structures of the element.
pub struct CJointStereoCplxCtx<'a> {
    /// `pAacDecoderChannelInfo[L]->pComStaticData->cplxPredictionData`
    pub cplx_prediction_data: &'a CCplxPredictionData,
    /// `pAacDecoderStaticChannelInfo[L]->pCpeStaticData->jointStereoPersistentData`
    pub joint_stereo_persistent_data: &'a CJointStereoPersistentData,
    /// `jointStereoPersistentData.spectralCoeffs[L]` (concealment spectrum of the left channel)
    pub static_spectral_coeffs_l: &'a [FixpDbl],
    /// `jointStereoPersistentData.spectralCoeffs[R]`
    pub static_spectral_coeffs_r: &'a [FixpDbl],
    /// `jointStereoPersistentData.specScale[L]` (concealment spec scale of the left channel)
    pub static_spec_scale_l: &'a [i16],
    /// `jointStereoPersistentData.specScale[R]`
    pub static_spec_scale_r: &'a [i16],
    /// `jointStereoPersistentData.scratchBuffer` (`pWorkBufferCore1->workBuffer`, 2048 values:
    /// `dmx_re` and `dmx_re_prev`)
    pub scratch_buffer: &'a mut [FixpDbl],
    /// `jointStereoPersistentData.scratchBuffer2` (`pTimeData2[2048..3072]`: `dmx_im`)
    pub scratch_buffer2: &'a mut [FixpDbl],
}

/// `CJointStereo_ApplyMS(pAacDecoderChannelInfo[2], pAacDecoderStaticChannelInfo[2], spectrumL,
/// spectrumR, SFBleftScale, SFBrightScale, specScaleL, specScaleR, pScaleFactorBandOffsets,
/// pWindowGroupLength, windowGroups, max_sfb_ste_outside, scaleFactorBandsTransmittedL,
/// scaleFactorBandsTransmittedR, store_dmx_re_prev, store_dmx_re_prev_e, mainband_flag)`:
/// applies M/S stereo or complex stereo prediction.
///
/// Of the channel structures the C++ code uses `pJointStereoData` (`p_joint_stereo_data`), the
/// icsInfo, granuleLength and transform_splitting_active of the channels and the complex
/// prediction data (`cplx`, required if `cplx_pred_flag` is set). `store_dmx_re_prev` /
/// `store_dmx_re_prev_e` always point to `pJointStereoData->store_dmx_re_prev(_e)`; the parameters
/// are omitted.
pub fn cjoint_stereo_apply_ms(
    p_joint_stereo_data: &mut CJointStereoData,
    cplx: Option<CJointStereoCplxCtx>,
    ics_info_l: &CIcsInfo,
    granule_length_l: i32,
    granule_length_r: i32,
    transform_splitting_active_l: u8,
    spectrum_l: &mut [FixpDbl],
    spectrum_r: &mut [FixpDbl],
    sfb_left_scale: &mut [i16],
    sfb_right_scale: &mut [i16],
    spec_scale_l: &mut [i16],
    spec_scale_r: &mut [i16],
    p_scale_factor_band_offsets: &[i16],
    p_window_group_length: &[u8],
    window_groups: i32,
    max_sfb_ste_outside: i32,
    scale_factor_bands_transmitted_l: i32,
    scale_factor_bands_transmitted_r: i32,
    mainband_flag: i32,
) {
    let jsd = p_joint_stereo_data;
    let offs = p_scale_factor_band_offsets;
    let off = |b: i32| offs[b as usize] as usize;

    let max_sfb_ste = f_max(scale_factor_bands_transmitted_l, scale_factor_bands_transmitted_r);
    let min_sfb_ste = f_min(scale_factor_bands_transmitted_l, scale_factor_bands_transmitted_r);
    let scale_factor_bands_transmitted = min_sfb_ste;

    if jsd.cplx_pred_flag != 0 {
        let cx = cplx.expect("complex prediction data");
        let cpd = cx.cplx_prediction_data;
        let jspd = cx.joint_stereo_persistent_data;
        let static_spectral_coeffs_l = cx.static_spectral_coeffs_l;
        let static_spectral_coeffs_r = cx.static_spectral_coeffs_r;
        let static_spec_scale_l = cx.static_spec_scale_l;
        let static_spec_scale_r = cx.static_spec_scale_r;

        let (dmx_re, dmx_re_prev) = cx.scratch_buffer[..2048].split_at_mut(1024);

        /* When MS is applied over the main band this value gets computed. Otherwise (for the
         * tiles) it uses the assigned value */
        let mut dmx_re_prev_e: i16 = jsd.store_dmx_re_prev_e;

        let mut p_coeff_prev: &[FixpFilt];
        let mut p_coeff: &[FixpFilt];
        let mut coeff_pointer_offset: usize;

        let previous_shape = jspd.win_shape_prev as i32;
        let current_shape = ics_info_l.window_shape as i32;

        /* complex stereo prediction */

        /* 0. preparations */

        /* 0.0. get scratch buffer for downmix MDST */
        let dmx_im = &mut *cx.scratch_buffer2;

        /* 0.1. window lengths */

        /* get length of short window for current configuration */
        let mut window_len = granule_length_l; /* framelength 768 => 96, framelength 1024 => 128 */

        /* if this is no short-block set length for long-block */
        if ics_info_l.window_sequence != BLOCK_SHORT {
            window_len *= 8;
        }
        let wl = window_len as usize;

        /* 0.2. set pointer to filter-coefficients for MDST excitation including previous frame
         * portions */
        /*      cp. ISO/IEC FDIS 23003-3:2011(E) table 125 */

        /* set pointer to default-position */
        p_coeff_prev = &MDST_FILT_COEF_PREV[previous_shape as usize];

        if cpd.complex_coef == 1 {
            match ics_info_l.window_sequence {
                /* current window sequence */
                BLOCK_SHORT | BLOCK_LONG => {
                    p_coeff_prev = &MDST_FILT_COEF_PREV[previous_shape as usize];
                }
                BLOCK_START => {
                    if (jspd.win_seq_prev as i32 == BLOCK_SHORT)
                        || (jspd.win_seq_prev as i32 == BLOCK_START)
                    {
                        /* a stop-start-sequence can only follow on an eight-short-sequence or a
                         * start-sequence */
                        p_coeff_prev = &MDST_FILT_COEF_PREV[2 + previous_shape as usize];
                    } else {
                        p_coeff_prev = &MDST_FILT_COEF_PREV[previous_shape as usize];
                    }
                }
                BLOCK_STOP => {
                    p_coeff_prev = &MDST_FILT_COEF_PREV[2 + previous_shape as usize];
                }
                _ => {
                    p_coeff_prev = &MDST_FILT_COEF_PREV[previous_shape as usize];
                }
            }
        }

        /* 0.3. set pointer to filter-coefficients for MDST excitation */

        /* define offset of pointer to filter-coefficients for MDST exitation employing only the
         * current frame */
        coeff_pointer_offset = if (previous_shape == SHAPE_SINE) && (current_shape == SHAPE_SINE) {
            0
        } else if (previous_shape == SHAPE_SINE) && (current_shape == SHAPE_KBD) {
            2
        } else if (previous_shape == SHAPE_KBD) && (current_shape == SHAPE_KBD) {
            1
        } else {
            /* if ( (previousShape == SHAPE_KBD) && (currentShape == SHAPE_SINE) ) */
            3
        };

        /* set pointer to filter-coefficient table cp. ISO/IEC FDIS 23003-3:2011(E) table 124 */
        p_coeff = match ics_info_l.window_sequence {
            /* current window sequence */
            BLOCK_SHORT | BLOCK_LONG => &MDST_FILT_COEF_CURR[coeff_pointer_offset],
            BLOCK_START => {
                if (jspd.win_seq_prev as i32 == BLOCK_SHORT)
                    || (jspd.win_seq_prev as i32 == BLOCK_START)
                {
                    /* a stop-start-sequence can only follow on an eight-short-sequence or a
                     * start-sequence */
                    &MDST_FILT_COEF_CURR[12 + coeff_pointer_offset]
                } else {
                    &MDST_FILT_COEF_CURR[4 + coeff_pointer_offset]
                }
            }
            BLOCK_STOP => &MDST_FILT_COEF_CURR[8 + coeff_pointer_offset],
            _ => &MDST_FILT_COEF_CURR[coeff_pointer_offset],
        };

        /* 0.4. find maximum common (l/r) band-scaling-factor for whole sequence (all windows) */
        let mut frame_max_scale = 0;
        let mut window = 0usize;
        for group in 0..window_groups.max(0) as usize {
            for _groupwin in 0..p_window_group_length[group] {
                let left_scale = &sfb_left_scale[window * 16..];
                let right_scale = &sfb_right_scale[window * 16..];
                let mut window_max_scale = 0;

                /* find maximum scaling factor of all bands in this window */
                let mut band = 0;
                while band < min_sfb_ste {
                    let l_scale = left_scale[band as usize] as i32;
                    let r_scale = right_scale[band as usize] as i32;
                    let common_scale = if l_scale > r_scale { l_scale } else { r_scale };
                    window_max_scale = if window_max_scale < common_scale {
                        common_scale
                    } else {
                        window_max_scale
                    };
                    band += 1;
                }
                if scale_factor_bands_transmitted_l > min_sfb_ste {
                    /* i.e. scaleFactorBandsTransmittedL == max_sfb_ste */
                    while band < max_sfb_ste {
                        let l_scale = left_scale[band as usize] as i32;
                        window_max_scale =
                            if window_max_scale < l_scale { l_scale } else { window_max_scale };
                        band += 1;
                    }
                } else if scale_factor_bands_transmitted_r > min_sfb_ste {
                    /* i.e. scaleFactorBandsTransmittedR == max_sfb_ste */
                    while band < max_sfb_ste {
                        let r_scale = right_scale[band as usize] as i32;
                        window_max_scale =
                            if window_max_scale < r_scale { r_scale } else { window_max_scale };
                        band += 1;
                    }
                }

                /* find maximum common SF of all windows */
                frame_max_scale = if frame_max_scale < window_max_scale {
                    window_max_scale
                } else {
                    frame_max_scale
                };
                window += 1;
            }
        }

        /* add some headroom for overflow protection during filter and add operation */
        frame_max_scale += 2;

        /* process on window-basis (i.e. iterate over all groups and corresponding windows) */
        let mut window = 0usize;
        for group in 0..window_groups.max(0) as usize {
            let group_mask = 1u8.wrapping_shl(group as u32);

            for _groupwin in 0..p_window_group_length[group] {
                /* initialize the MDST with zeros */
                dmx_im[wl * window..wl * window + wl].fill(0);

                /* 1. calculate the previous downmix MDCT. We do this once just for the Main band. */
                if cpd.complex_coef == 1 && cpd.use_prev_frame == 1 && mainband_flag != 0 {
                    /* if this is a long-block or the first window of a short-block
                       calculate the downmix MDCT of the previous frame.
                       use_prev_frame is assumed not to change during a frame!
                    */

                    /* first determine shiftfactors to scale left and right channel */
                    if (ics_info_l.window_sequence != BLOCK_SHORT) || (window == 0) {
                        let mut index_offset = 0usize;
                        let mut sr_left_chan = 0i32;
                        let mut sr_right_chan = 0i32;
                        let k = if ics_info_l.window_sequence == BLOCK_SHORT {
                            /* use the last window of the previous frame for MDCT calculation if
                             * this is a short-block. */
                            index_offset = wl * 7;
                            7
                        } else {
                            0
                        };
                        if static_spec_scale_l[k] > static_spec_scale_r[k] {
                            sr_right_chan =
                                static_spec_scale_l[k] as i32 - static_spec_scale_r[k] as i32;
                            dmx_re_prev_e = static_spec_scale_l[k];
                        } else {
                            sr_left_chan =
                                static_spec_scale_r[k] as i32 - static_spec_scale_l[k] as i32;
                            dmx_re_prev_e = static_spec_scale_r[k];
                        }

                        /* now scale channels and determine downmix MDCT of previous frame */
                        if jspd.clear_spectral_coeffs == 1 {
                            dmx_re_prev[..wl].fill(0);
                            dmx_re_prev_e = 0;
                        } else {
                            let sl = f_min(DFRACT_BITS - 1, sr_left_chan + 1);
                            let sr = f_min(DFRACT_BITS - 1, sr_right_chan + 1);
                            if cpd.pred_dir == 0 {
                                for i in 0..wl {
                                    dmx_re_prev[i] = (static_spectral_coeffs_l[index_offset + i]
                                        >> sl)
                                        + (static_spectral_coeffs_r[index_offset + i] >> sr);
                                }
                            } else {
                                for i in 0..wl {
                                    dmx_re_prev[i] = (static_spectral_coeffs_l[index_offset + i]
                                        >> sl)
                                        - (static_spectral_coeffs_r[index_offset + i] >> sr);
                                }
                            }
                        }

                        /* In case that we use INF we have to preserve the state of the
                        "dmx_re_prev" (original or computed). This is necessary because we have to
                        apply MS over the separate IGF tiles. */
                        jsd.store_dmx_re_prev[..wl].copy_from_slice(&dmx_re_prev[..wl]);

                        /* Particular exponent of the computed/original "dmx_re_prev" must be kept
                         * for the tile MS calculations if necessary.*/
                        jsd.store_dmx_re_prev_e = dmx_re_prev_e;
                    } /* if ( (pAacDecoderChannelInfo[L]->icsInfo.WindowSequence != BLOCK_SHORT) ||
                      (window == 0) ) */
                } /* if ( pJointStereoData->complex_coef == 1 ) */

                /* 2. calculate downmix MDCT of current frame */

                /* set pointer to scale-factor-bands of current window */
                let ls_off = window * 16;

                spec_scale_l[window] = frame_max_scale as i16;
                spec_scale_r[window] = frame_max_scale as i16;

                /* adapt scaling-factors to previous frame */
                if cpd.use_prev_frame == 1 {
                    if window == 0 {
                        if (dmx_re_prev_e as i32) < frame_max_scale {
                            let s = -f_min(DFRACT_BITS - 1, frame_max_scale - dmx_re_prev_e as i32);
                            if mainband_flag == 0 {
                                scale_values_to(
                                    &mut dmx_re_prev[..wl],
                                    &jsd.store_dmx_re_prev[..wl],
                                    s,
                                );
                            } else {
                                scale_values(&mut dmx_re_prev[..wl], s);
                            }
                        } else {
                            if mainband_flag == 0 {
                                dmx_re_prev[..wl].copy_from_slice(&jsd.store_dmx_re_prev[..wl]);
                            }
                            spec_scale_l[0] = dmx_re_prev_e;
                            spec_scale_r[0] = dmx_re_prev_e;
                        }
                    } else {
                        /* window != 0 */
                        debug_assert!(ics_info_l.window_sequence == BLOCK_SHORT);
                        if (spec_scale_l[window - 1] as i32) < frame_max_scale {
                            let s = -f_min(
                                DFRACT_BITS - 1,
                                frame_max_scale - spec_scale_l[window - 1] as i32,
                            );
                            scale_values(&mut dmx_re[wl * (window - 1)..wl * window], s);
                        } else {
                            spec_scale_l[window] = spec_scale_l[window - 1];
                            spec_scale_r[window] = spec_scale_r[window - 1];
                        }
                    }
                } /* if ( pJointStereoData->use_prev_frame == 1 ) */

                /* scaling factors of both channels ought to be equal now */
                debug_assert!(spec_scale_l[window] == spec_scale_r[window]);

                /* rescale signal and calculate downmix MDCT */
                for band in 0..max_sfb_ste.max(0) as usize {
                    /* first adapt scaling of current band to scaling of current window => shift
                     * signal right */
                    let mut l_scale = sfb_left_scale[ls_off + band] as i32;
                    let mut r_scale = sfb_right_scale[ls_off + band] as i32;

                    l_scale = f_min(DFRACT_BITS - 1, spec_scale_l[window] as i32 - l_scale);
                    r_scale = f_min(DFRACT_BITS - 1, spec_scale_l[window] as i32 - r_scale); /* L or R doesn't matter, specScales are equal at this point */

                    /* Write back to sfb scale to cover the case when max_sfb_ste < max_sfb */
                    sfb_left_scale[ls_off + band] = spec_scale_l[window];
                    sfb_right_scale[ls_off + band] = spec_scale_l[window];

                    if l_scale < 0 || r_scale < 0 {
                        /* shifts by negative amounts: partly vectorized in the reference build
                         * (see the crate documentation, "Deviations from the C++ code") */
                        mpeghdec_fdk::deviation::hit();
                    }
                    let b0 = wl * window + offs[band] as usize;
                    let b1 = wl * window + offs[band + 1] as usize;
                    for i in b0..b1 {
                        spectrum_l[i] = spectrum_l[i].wrapping_shr(l_scale as u32);
                        spectrum_r[i] = spectrum_r[i].wrapping_shr(r_scale as u32);
                    }

                    /* now calculate downmix MDCT */
                    if jsd.ms_used[band] & group_mask != 0 {
                        dmx_re[b0..b1].copy_from_slice(&spectrum_l[b0..b1]);
                    } else if cpd.pred_dir == 0 {
                        for i in b0..b1 {
                            dmx_re[i] = spectrum_l[i].wrapping_add(spectrum_r[i]) >> 1;
                        }
                    } else {
                        for i in b0..b1 {
                            dmx_re[i] = spectrum_l[i].wrapping_sub(spectrum_r[i]) >> 1;
                        }
                    }
                } /* for ( band=0; band<max_sfb_ste; band++ ) */

                /* Clean until the end */
                for i in off(max_sfb_ste_outside)..wl {
                    dmx_re[wl * window + i] = 0;
                }

                /* 3. calculate MDST-portion corresponding to the current frame. */
                if cpd.complex_coef == 1 {
                    /* If transform splitting is active */
                    if transform_splitting_active_l != 0 {
                        /* Deinterleave */
                        let half_trans_length = 512;
                        let half_lowpass_line = offs[max_sfb_ste_outside as usize] as i32 >> 1;

                        /* The length of the filter processing must be extended because of filter
                         * boundary problems */
                        let extended_band = f_min(half_lowpass_line + 7, half_trans_length);

                        coeff_pointer_offset = if (previous_shape == SHAPE_SINE)
                            && (current_shape == SHAPE_SINE)
                        {
                            0
                        } else if (previous_shape == SHAPE_SINE) && (current_shape == SHAPE_KBD) {
                            2
                        } else if (previous_shape == SHAPE_KBD) && (current_shape == SHAPE_KBD) {
                            1
                        } else {
                            /* if ( (previousShape == SHAPE_KBD) && (currentShape == SHAPE_SINE) ) */
                            3
                        };

                        p_coeff = &MDST_FILT_COEF_CURR[16 + coeff_pointer_offset];
                        p_coeff_prev = &MDST_FILT_COEF_PREV[4 + previous_shape as usize];

                        cjoint_stereo_filter_and_add_int(
                            dmx_re,
                            extended_band,
                            half_trans_length,
                            half_lowpass_line,
                            p_coeff,
                            dmx_im,
                            1,
                        );

                        if cpd.use_prev_frame == 1 {
                            cjoint_stereo_filter_and_add_int(
                                &dmx_re_prev[1..],
                                extended_band,
                                half_trans_length,
                                extended_band,
                                p_coeff_prev,
                                dmx_im,
                                0,
                            );
                        }

                        coeff_pointer_offset = if current_shape == SHAPE_SINE { 0 } else { 1 };

                        p_coeff = &MDST_FILT_COEF_CURR[16 + coeff_pointer_offset];
                        p_coeff_prev = &MDST_FILT_COEF_PREV[4 + current_shape as usize];

                        cjoint_stereo_filter_and_add_int(
                            &dmx_re[1..],
                            extended_band,
                            half_trans_length,
                            half_lowpass_line,
                            p_coeff,
                            &mut dmx_im[1..],
                            1,
                        );

                        cjoint_stereo_filter_and_add_int(
                            dmx_re,
                            extended_band,
                            half_trans_length,
                            half_lowpass_line,
                            p_coeff_prev,
                            &mut dmx_im[1..],
                            0,
                        );
                    } else {
                        /* 3.1 move pointer in filter-coefficient table in case of short window
                         * sequence */
                        /*     (other coefficients are utilized for the last 7 short windows) */
                        if (ics_info_l.window_sequence == BLOCK_SHORT) && (window != 0) {
                            p_coeff = &MDST_FILT_COEF_CURR[current_shape as usize];
                            p_coeff_prev = &MDST_FILT_COEF_PREV[current_shape as usize];
                        }

                        /* The length of the filter processing must be extended because of filter
                         * boundary problems */
                        let extended_band =
                            f_min(offs[max_sfb_ste_outside as usize] as i32 + 7, window_len);

                        /* 3.2. estimate downmix MDST from current frame downmix MDCT */
                        if (ics_info_l.window_sequence == BLOCK_SHORT) && (window != 0) {
                            cjoint_stereo_filter_and_add(
                                &dmx_re[wl * window..],
                                extended_band,
                                window_len,
                                p_coeff,
                                &mut dmx_im[wl * window..],
                                1,
                            );

                            cjoint_stereo_filter_and_add(
                                &dmx_re[wl * (window - 1)..],
                                extended_band,
                                window_len,
                                p_coeff_prev,
                                &mut dmx_im[wl * window..],
                                0,
                            );
                        } else {
                            cjoint_stereo_filter_and_add(
                                dmx_re,
                                extended_band,
                                window_len,
                                p_coeff,
                                dmx_im,
                                1,
                            );

                            if cpd.use_prev_frame == 1 {
                                cjoint_stereo_filter_and_add(
                                    dmx_re_prev,
                                    extended_band,
                                    window_len,
                                    p_coeff_prev,
                                    &mut dmx_im[wl * window..],
                                    0,
                                );
                            }
                        }
                    } /* if(pAacDecoderChannelInfo[L]->transform_splitting_active) */
                } /* if ( pJointStereoData->complex_coef == 1 ) */

                /* 4. upmix process */
                let pred_dir: i32 = if cpd.pred_dir != 0 { -1 } else { 1 };
                /* 0.1 in Q-3.34 */
                let point_one: FixpDbl = 0x66666666; /* 0.8 */
                /* Shift value for the downmix */
                let shift_dmx = SF_FNA_COEFFS + 1;

                for band in 0..max_sfb_ste_outside.max(0) as usize {
                    if jsd.ms_used[band] & group_mask != 0 {
                        let temp_re = cpd.alpha_q_re[group][band];
                        let temp_im = cpd.alpha_q_im[group][band];

                        /* Find the minimum common headroom for alpha_re and alpha_im */
                        let mut alpha_re_headroom = count_leading_bits(temp_re as i32) - 16;
                        if temp_re == 0 {
                            alpha_re_headroom = 15;
                        }
                        let mut alpha_im_headroom = count_leading_bits(temp_im as i32) - 16;
                        if temp_im == 0 {
                            alpha_im_headroom = 15;
                        }
                        let val = f_min(alpha_re_headroom, alpha_im_headroom);

                        /* Multiply alpha by 0.1 with maximum precision */
                        debug_assert!(val >= 0);
                        let alpha_re_tmp = f_mult(((temp_re as i32) << val) as FixpSgl, point_one);
                        let alpha_im_tmp = f_mult(((temp_im as i32) << val) as FixpSgl, point_one);

                        /* Calculate alpha exponent */
                        /* (Q-3.34 * Q15.0) shifted left by "val" */
                        let alpha_re_exp = -3 + 15 - val;

                        let help3_shift = alpha_re_exp + 1;

                        let base = wl * window;
                        for i in offs[band] as usize..offs[band + 1] as usize {
                            /* Calculating helper term:
                                  side = specR[i] - alpha_re[i] * dmx_re[i] - alpha_im[i] * dmx_im[i];

                              Here "dmx_re" may be the same as "specL" or alternatively keep the
                              downmix. "dmx_re" and "specL" are two different pointers pointing to
                              separate arrays, which may or may not contain the same data (with
                              different scaling).

                              specL[i] =   + (specL[i] + side);
                              specR[i] = -/+ (specL[i] - side);
                            */
                            let mut side = f_mult_add_div2(
                                f_mult_div2(alpha_re_tmp, dmx_re[base + i]),
                                alpha_im_tmp,
                                dmx_im[base + i].wrapping_shl(shift_dmx as u32),
                            );
                            side = (spectrum_r[base + i] >> 2).wrapping_sub(saturate_shift(
                                side,
                                -(help3_shift - 2),
                                DFRACT_BITS - 2,
                            ));

                            let left = (spectrum_l[base + i] >> 2).wrapping_add(side);
                            let mut right = (spectrum_l[base + i] >> 2).wrapping_sub(side);
                            right = right.wrapping_mul(pred_dir);

                            spectrum_l[base + i] = saturate_left_shift_alt(left, 2, DFRACT_BITS);
                            spectrum_r[base + i] = saturate_left_shift_alt(right, 2, DFRACT_BITS);
                        }
                    }
                } /* for ( band=0; band < max_sfb_ste; band++ ) */
                window += 1;
            } /* for ( groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++ ) */
        } /* for ( window = 0, group = 0; group < windowGroups; group++ ) */
    } else {
        /* MS stereo */

        let mut window = 0i32;
        for group in 0..window_groups.max(0) as usize {
            let group_mask = 1u8.wrapping_shl(group as u32);

            for _groupwin in 0..p_window_group_length[group] {
                let ls_off = window as usize * 16;

                let l_base = (window * granule_length_l) as usize;
                let r_base = (window * granule_length_r) as usize;

                let mut band = 0i32;
                while band < max_sfb_ste_outside {
                    let b = band as usize;
                    if jsd.ms_used[b] & group_mask != 0 {
                        let l_scale0 = sfb_left_scale[ls_off + b] as i32;
                        let r_scale0 = sfb_right_scale[ls_off + b] as i32;
                        let mut common_scale =
                            if l_scale0 > r_scale0 { l_scale0 } else { r_scale0 };

                        /* ISO/IEC 14496-3 Chapter 4.6.8.1.1 :
                        M/S joint channel coding can only be used if common_window is 1. */

                        common_scale += 1;
                        sfb_left_scale[ls_off + b] = common_scale as i16;
                        sfb_right_scale[ls_off + b] = common_scale as i16;

                        let l_scale = f_min(DFRACT_BITS - 1, common_scale - l_scale0);
                        let r_scale = f_min(DFRACT_BITS - 1, common_scale - r_scale0);

                        debug_assert!(l_scale >= 0 && r_scale >= 0);

                        let offset_curr_band = offs[b] as usize;
                        let offset_next_band = offs[b + 1] as usize;

                        cjoint_stereo_generate_ms_output(
                            &mut spectrum_l[l_base + offset_curr_band..],
                            &mut spectrum_r[r_base + offset_curr_band..],
                            l_scale as u32,
                            r_scale as u32,
                            (offset_next_band - offset_curr_band) as u32,
                        );
                    }
                    band += 1;
                }
                if scale_factor_bands_transmitted_l > scale_factor_bands_transmitted {
                    while band < scale_factor_bands_transmitted_l {
                        let b = band as usize;
                        if jsd.ms_used[b] & group_mask != 0 {
                            sfb_right_scale[ls_off + b] = sfb_left_scale[ls_off + b];

                            for index in offs[b] as usize..offs[b + 1] as usize {
                                let left_coefficient = spectrum_l[l_base + index];
                                /* FIXP_DBL rightCoefficient = (FIXP_DBL)0; */
                                spectrum_r[r_base + index] = left_coefficient;
                            }
                        }
                        band += 1;
                    }
                } else if scale_factor_bands_transmitted_r > scale_factor_bands_transmitted {
                    while band < scale_factor_bands_transmitted_r {
                        let b = band as usize;
                        if jsd.ms_used[b] & group_mask != 0 {
                            sfb_left_scale[ls_off + b] = sfb_right_scale[ls_off + b];

                            for index in offs[b] as usize..offs[b + 1] as usize {
                                /* FIXP_DBL leftCoefficient  = (FIXP_DBL)0; */
                                let right_coefficient = spectrum_r[r_base + index];

                                spectrum_l[l_base + index] = right_coefficient;
                                spectrum_r[r_base + index] = f_negate_saturate(right_coefficient);
                            }
                        }
                        band += 1;
                    }
                }
                window += 1;
            }
        }

        /* Reset MsUsed flags if no explicit signalling was transmitted. Necessary for intensity
        coding. PNS correlation signalling was mapped before calling CJointStereo_ApplyMS(). */
    }
}

/// `CJointStereo_ApplyMS_IGF(pAacDecoderStaticChannelInfo[2], pAacDecoderChannelInfo[2], pSpecL,
/// pSpecR, pScaleFactorBandOffsets, pWindowGroupLength, windowGroups)`: M/S or complex prediction
/// in the IGF range.
///
/// Used fields of the channel structures: `pJointStereoData` and `cplxPredictionData` of the
/// element, `IGF_StaticData` of the left channel (IGF start/stop sfb), `icsInfo` of the left
/// channel and `granuleLength` / `specScale` of both channels.
pub fn cjoint_stereo_apply_ms_igf(
    p_joint_stereo_data: &CJointStereoData,
    cplx_prediction_data: Option<&CCplxPredictionData>,
    igf_static_data_l: &IgfPrivateStaticData,
    ics_info_l: &CIcsInfo,
    granule_length_l: i32,
    granule_length_r: i32,
    spec_scale_l: &[i16],
    spec_scale_r: &[i16],
    p_spec_l: &mut [FixpDbl],
    p_spec_r: &mut [FixpDbl],
    p_scale_factor_band_offsets: &[i16],
    p_window_group_length: &[u8],
    window_groups: i32,
) {
    let jsd = p_joint_stereo_data;
    let offs = p_scale_factor_band_offsets;

    /* Exit if the MS mask is all zero. Nothing to do */
    if jsd.igf_ms_mask_present == 0 {
        return;
    }

    let (igf_sfb_start, igf_sfb_stop) = if is_long_block(ics_info_l) != 0 {
        (igf_static_data_l.igf_start_sfb_lb as usize, igf_static_data_l.igf_stop_sfb_lb as usize)
    } else {
        (igf_static_data_l.igf_start_sfb_sb as usize, igf_static_data_l.igf_stop_sfb_sb as usize)
    };

    if (jsd.igf_cplx_pred_flag != 0) && (jsd.igf_ms_mask_present == 3) {
        let cpd = cplx_prediction_data.expect("cplxPredictionData");

        /* get length of short window for current configuration */
        let mut window_len = granule_length_l; /* framelength 768 => 96, framelength 1024 => 128 */
        /* if this is no short-block set length for long-block */
        if ics_info_l.window_sequence != BLOCK_SHORT {
            window_len *= 8;
        }
        let wl = window_len as usize;

        /* process on window-basis (i.e. iterate over all groups and corresponding windows) */
        let mut window = 0usize;
        for group in 0..window_groups.max(0) as usize {
            let group_mask = 1u8.wrapping_shl(group as u32);

            for _groupwin in 0..p_window_group_length[group] {
                let temp = spec_scale_l[window] as i32 - spec_scale_r[window] as i32;
                let (l_scale, r_scale) =
                    if temp >= 0 { (2, f_min(31, temp + 2)) } else { (f_min(31, -temp + 2), 2) };

                /* rescale signal if necessary*/
                for band in igf_sfb_start..igf_sfb_stop {
                    if jsd.ms_used[band] & group_mask != 0 {
                        for i in offs[band] as usize..offs[band + 1] as usize {
                            p_spec_l[wl * window + i] >>= l_scale;
                            p_spec_r[wl * window + i] >>= r_scale;
                        }
                    }
                } /* for ( band=0; band<max_sfb_ste; band++ ) */

                /* Upmix process */
                let igf_pred_dir: i32 = if cpd.igf_pred_dir != 0 { -1 } else { 1 };

                /* 0.1 in Q-3.34 */
                let point_one: FixpDbl = 0x66666666;

                for band in igf_sfb_start..igf_sfb_stop {
                    if jsd.ms_used[band] & group_mask != 0 {
                        let temp_re = cpd.alpha_q_re[group][band];

                        /* Find the minimum common headroom for alpha_re and alpha_im */
                        let alpha_re_headroom = count_leading_bits((temp_re as i32) << 16);

                        /* Multiply alpha by 0.1 with maximum precision */
                        let alpha_re_tmp =
                            f_mult(((temp_re as i32) << alpha_re_headroom) as FixpSgl, point_one);

                        /* Calculate shift exponent */
                        /* (Q-3.34 * Q15.0) shifted left by "val" and + 1 because of Div2 Mult */
                        let help1_shift = 13 - alpha_re_headroom;

                        for i in offs[band] as usize..offs[band + 1] as usize {
                            let il = wl * window + i;
                            /* Calculating: side = specR[i] - alpha_re[i] * specL[i]; */

                            /* alpha_re[i] * specL[i] */
                            let help1 = f_mult_div2(alpha_re_tmp, p_spec_l[il]);

                            /* side = specR[i] - alpha_re[i] * specL[i] */
                            let help2 = p_spec_r[il].wrapping_sub(saturate_shift(
                                help1,
                                -help1_shift,
                                DFRACT_BITS - 2,
                            ));

                            /* specR[i] = -/+ (specL[i] - side; */
                            p_spec_r[il] = saturate_left_shift(
                                p_spec_l[il].wrapping_sub(help2).wrapping_mul(igf_pred_dir),
                                r_scale,
                                DFRACT_BITS,
                            );

                            /* specL[i] = specL[i] + side; */
                            p_spec_l[il] = saturate_left_shift(
                                p_spec_l[il].wrapping_add(help2),
                                l_scale,
                                DFRACT_BITS,
                            );
                        }
                    } /* if ( pJointStereoData->MsUsed[band] & groupMask ) */
                } /* for ( band=0; band < max_sfb_ste; band++ ) */
                window += 1;
            } /* for ( groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++ ) */
        } /* for ( window = 0, group = 0; group < windowGroups; group++ ) */

    /* free scratch buffer */
    /* C_AALLOC_SCRATCH_END(dmx_im, FIXP_DBL, 1024); */
    } else
    /* Complex prediction end*/
    {
        /* MS stereo */

        let mut window = 0usize;
        for group in 0..window_groups.max(0) as usize {
            let group_mask = 1u8.wrapping_shl(group as u32);

            for _groupwin in 0..p_window_group_length[group] {
                /* Find if a difference between the scalings exists. Add 1-bit headroom because of
                 * the addtion/subtraction */

                let temp = spec_scale_l[window] as i32 - spec_scale_r[window] as i32;
                let (l_scale, r_scale) =
                    if temp >= 0 { (1, f_min(31, temp + 1)) } else { (f_min(31, -temp + 1), 1) };

                let l_base = window * granule_length_l as usize;
                let r_base = window * granule_length_r as usize;

                for band in igf_sfb_start..igf_sfb_stop {
                    if jsd.ms_used[band] & group_mask != 0 {
                        for index in offs[band] as usize..offs[band + 1] as usize {
                            /* MS output generation */
                            p_spec_l[l_base + index] >>= l_scale;
                            let temp_left = p_spec_l[l_base + index];
                            p_spec_r[r_base + index] >>= r_scale;
                            let temp_right = p_spec_r[r_base + index];

                            p_spec_l[l_base + index] = saturate_left_shift(
                                temp_left.wrapping_add(temp_right),
                                l_scale,
                                DFRACT_BITS,
                            );
                            p_spec_r[r_base + index] = saturate_left_shift(
                                temp_left.wrapping_sub(temp_right),
                                r_scale,
                                DFRACT_BITS,
                            );
                        }
                    }
                }
                window += 1;
            }
        }
    }
}

/// `IGF_StereoFilling_GetPreviousDmx(pAacDecoderStaticChannelInfo[2], pAacDecoderChannelInfo[2],
/// dmx_prev, dmx_prev_win_exp, pScaleFactorBandOffsets, pWindowGroupLength, windowGroups,
/// max_noise_sfb, band_is_noise)`: the downmix of the previous frame for the fully noise-filled
/// bands.
///
/// Used fields of the channel structures: `pCpeStaticData->jointStereoPersistentData`
/// (`clearSpectralCoeffs`, `spectralCoeffs[L/R]`, `specScale[L/R]`),
/// `cplxPredictionData->pred_dir`, `icsInfo` of the left channel and `granuleLength` of both
/// channels.
pub fn igf_stereo_filling_get_previous_dmx(
    p_joint_stereo_persistent_data: &CJointStereoPersistentData,
    cplx_prediction_data: &CCplxPredictionData,
    static_spectral_coeffs_l: &[FixpDbl],
    static_spectral_coeffs_r: &[FixpDbl],
    static_spec_scale_l: &[i16],
    static_spec_scale_r: &[i16],
    ics_info_l: &CIcsInfo,
    granule_length_l: i32,
    granule_length_r: i32,
    dmx_prev: &mut [FixpDbl],
    dmx_prev_win_exp: &mut [i16],
    p_scale_factor_band_offsets: &[i16],
    p_window_group_length: &[u8],
    window_groups: i32,
    max_noise_sfb: i32,
    band_is_noise: &[u8],
) {
    let offs = p_scale_factor_band_offsets;

    let noise_filling_start_offset =
        if ics_info_l.window_sequence == BLOCK_SHORT { 20 } else { 160 };

    /* determine sfb from where on noise filling is applied */
    let mut sfb = 0usize;
    while (offs[sfb] as i32) < noise_filling_start_offset {
        sfb += 1;
    }
    let nf_start_offset_sfb = sfb;
    let max_noise_sfb = max_noise_sfb.max(0) as usize;

    /* MS stereo */
    if p_joint_stereo_persistent_data.clear_spectral_coeffs == 1 {
        let mut window = 0usize;
        for group in 0..window_groups.max(0) as usize {
            for _groupwin in 0..p_window_group_length[group] {
                let out_base = window * granule_length_r as usize;

                for sfb in nf_start_offset_sfb..max_noise_sfb {
                    /* Run algorithm only if the particular band has been fully noise filled, i.e.
                     * that it has been empty before the noise-filling process */
                    if band_is_noise[group * 16 + sfb] == 0 {
                        continue;
                    }

                    dmx_prev[out_base + offs[sfb] as usize..out_base + offs[sfb + 1] as usize]
                        .fill(0);
                } /* for (band=0; band<scaleFactorBandsTransmitted; band++) */
                window += 1;
            } /*  for (int groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++) */
        }
        return;
    }

    let mut window = 0usize;
    for group in 0..window_groups.max(0) as usize {
        for _groupwin in 0..p_window_group_length[group] {
            /* Find if a difference between the scalings exists. Add 1-bit headroom because of the
             * addtion/subtraction */
            let l_scale;
            let r_scale;
            let temp = static_spec_scale_l[window] as i32 - static_spec_scale_r[window] as i32;
            if temp >= 0 {
                l_scale = 1;
                r_scale = f_min(31, temp + 1);
                dmx_prev_win_exp[window] = static_spec_scale_l[window];
            } else {
                l_scale = f_min(31, -temp + 1);
                r_scale = 1;
                dmx_prev_win_exp[window] = static_spec_scale_r[window];
            }

            let l_base = window * granule_length_l as usize;
            let r_base = window * granule_length_r as usize;
            let out_base = window * granule_length_r as usize;

            for sfb in nf_start_offset_sfb..max_noise_sfb {
                /* Run algorithm only if the particular band has been fully noise filled, i.e. that
                it has been empty before the noise-filling process */
                if band_is_noise[group * 16 + sfb] == 0 {
                    continue;
                }

                for index in offs[sfb] as usize..offs[sfb + 1] as usize {
                    let l = static_spectral_coeffs_l[l_base + index] >> l_scale;
                    let r = static_spectral_coeffs_r[r_base + index] >> r_scale;
                    dmx_prev[out_base + index] = if cplx_prediction_data.pred_dir == 0 {
                        l.wrapping_add(r)
                    } else {
                        l.wrapping_sub(r)
                    };
                }
            } /*for (sfb= nfStartOffset_sfb; sfb < max_noise_sfb; sfb++)*/
            window += 1;
        } /*  for (int groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++)*/
    }
}
