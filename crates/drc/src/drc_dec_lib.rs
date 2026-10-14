// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/include/FDK_drcDecLib.h, libDRCdec/src/FDK_drcDecLib.cpp.

//! Public interface of the MPEG-D DRC decoder.
//!
//! DRC decoder according to ISO/IEC 23003-4 (MPEG-D DRC), including ISO/IEC 23003-4/AMD1
//! (Amendment 1) and section 6 of ISO/IEC 23008-3 (MPEG-H 3D Audio).
//!
//! Calling sequence per frame:
//! [`DrcDecoder::read_uni_drc_gain`] (and config/loudness readers when present),
//! [`DrcDecoder::set_channel_gains`], [`DrcDecoder::preprocess`],
//! [`DrcDecoder::process_time`] / [`DrcDecoder::process_freq`].

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::FixpDbl;

use crate::drc_dec_gain_decoder::*;
use crate::drc_dec_reader::*;
use crate::drc_dec_selection_process::*;
use crate::drc_dec_tools::bitstream_contains_multiband_drc;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;

/// `DRC_DEC_LOUDNESS_NOT_PRESENT`
pub const DRC_DEC_LOUDNESS_NOT_PRESENT: i32 = 0x7FFF_FFFE;

/// `DRC_DEC_FUNCTIONAL_RANGE`
pub type DrcDecFunctionalRange = i32;
/// `DRC_DEC_SELECTION`: DRC decoder instance for DRC set selection only
pub const DRC_DEC_SELECTION: DrcDecFunctionalRange = 0x1;
/// `DRC_DEC_GAIN`: DRC decoder instance for applying DRC only
pub const DRC_DEC_GAIN: DrcDecFunctionalRange = 0x2;
/// `DRC_DEC_ALL`: DRC decoder with full functionality
pub const DRC_DEC_ALL: DrcDecFunctionalRange = 0x3;

/// `DRC_DEC_USERPARAM`
pub type DrcDecUserParam = i32;
/* get and set userparams */
/// `DRC_DEC_BOOST`
pub const DRC_DEC_BOOST: DrcDecUserParam = 0;
/// `DRC_DEC_COMPRESS`
pub const DRC_DEC_COMPRESS: DrcDecUserParam = 1;
/* set only userparams */
/// `DRC_DEC_LOUDNESS_NORMALIZATION_ON`
pub const DRC_DEC_LOUDNESS_NORMALIZATION_ON: DrcDecUserParam = 2;
/// `DRC_DEC_TARGET_LOUDNESS`: target loudness in dB, with exponent e = 7
pub const DRC_DEC_TARGET_LOUDNESS: DrcDecUserParam = 3;
/// `DRC_DEC_EFFECT_TYPE`
pub const DRC_DEC_EFFECT_TYPE: DrcDecUserParam = 4;
/// `DRC_DEC_EFFECT_TYPE_FALLBACK_CODE`
pub const DRC_DEC_EFFECT_TYPE_FALLBACK_CODE: DrcDecUserParam = 5;
/// `DRC_DEC_LOUDNESS_MEASUREMENT_METHOD`
pub const DRC_DEC_LOUDNESS_MEASUREMENT_METHOD: DrcDecUserParam = 6;
/// `DRC_DEC_ALBUM_MODE`
pub const DRC_DEC_ALBUM_MODE: DrcDecUserParam = 7;
/* set only system (not user) parameters */
/// `DRC_DEC_DOWNMIX_ID`
pub const DRC_DEC_DOWNMIX_ID: DrcDecUserParam = 8;
/// `DRC_DEC_TARGET_LAYOUT_REQUESTED`: CICP target layout notified to FDK_drcDecLib for choosing
/// an appropriate downmixInstruction
pub const DRC_DEC_TARGET_LAYOUT_REQUESTED: DrcDecUserParam = 9;
/// `DRC_DEC_TARGET_CHANNEL_COUNT_REQUESTED`: number of output channels notified to
/// FDK_drcDecLib for choosing an appropriate downmixInstruction
pub const DRC_DEC_TARGET_CHANNEL_COUNT_REQUESTED: DrcDecUserParam = 10;
/// `DRC_DEC_BASE_CHANNEL_COUNT`
pub const DRC_DEC_BASE_CHANNEL_COUNT: DrcDecUserParam = 11;
/// `DRC_DEC_FRAME_SIZE`
pub const DRC_DEC_FRAME_SIZE: DrcDecUserParam = 12;
/// `DRC_DEC_SAMPLE_RATE`
pub const DRC_DEC_SAMPLE_RATE: DrcDecUserParam = 13;
/* get only system parameters */
/// `DRC_DEC_IS_MULTIBAND_DRC_1`
pub const DRC_DEC_IS_MULTIBAND_DRC_1: DrcDecUserParam = 14;
/// `DRC_DEC_IS_MULTIBAND_DRC_2`
pub const DRC_DEC_IS_MULTIBAND_DRC_2: DrcDecUserParam = 15;
/// `DRC_DEC_IS_ACTIVE`: MPEG-D DRC payload is present
pub const DRC_DEC_IS_ACTIVE: DrcDecUserParam = 16;
/// `DRC_DEC_TARGET_LAYOUT_SELECTED`: selected CICP target layout if appropriate
/// downmixInstruction exists, or -1 otherwise
pub const DRC_DEC_TARGET_LAYOUT_SELECTED: DrcDecUserParam = 17;
/// `DRC_DEC_TARGET_CHANNEL_COUNT_SELECTED`: number of output channels if appropriate
/// downmixInstruction exists
pub const DRC_DEC_TARGET_CHANNEL_COUNT_SELECTED: DrcDecUserParam = 18;
/// `DRC_DEC_OUTPUT_LOUDNESS`: output loudness in dB, with exponent e = 7, or
/// DRC_DEC_LOUDNESS_NOT_PRESENT if no loudness is contained in the bitstream
pub const DRC_DEC_OUTPUT_LOUDNESS: DrcDecUserParam = 19;
/// `DRC_DEC_IS_MUTE`: audio output is mute, because no loudnessInfoSet is present yet in
/// uniDrc() payload. Only for codec mode DRC_DEC_MPEG_4_AAC
pub const DRC_DEC_IS_MUTE: DrcDecUserParam = 20;

/// `DRC_DEC_ERROR`
pub type DrcDecError = i32;
/// `DRC_DEC_OK`
pub const DRC_DEC_OK: DrcDecError = 0;
/// `DRC_DEC_NOT_OK`
pub const DRC_DEC_NOT_OK: DrcDecError = -10000;
/// `DRC_DEC_OUT_OF_MEMORY`
pub const DRC_DEC_OUT_OF_MEMORY: DrcDecError = -9999;
/// `DRC_DEC_NOT_OPENED`
pub const DRC_DEC_NOT_OPENED: DrcDecError = -9998;
/// `DRC_DEC_NOT_READY`
pub const DRC_DEC_NOT_READY: DrcDecError = -9997;
/// `DRC_DEC_PARAM_OUT_OF_RANGE`
pub const DRC_DEC_PARAM_OUT_OF_RANGE: DrcDecError = -9996;
/// `DRC_DEC_INVALID_PARAM`
pub const DRC_DEC_INVALID_PARAM: DrcDecError = -9995;
/// `DRC_DEC_UNSUPPORTED_FUNCTION`
pub const DRC_DEC_UNSUPPORTED_FUNCTION: DrcDecError = -9994;
/// `DRC_DEC_PARSE_ERROR`
pub const DRC_DEC_PARSE_ERROR: DrcDecError = -9993;

/// `DRC_DEC_CODEC_MODE`
pub type DrcDecCodecMode = i32;
/// `DRC_DEC_TEST_TIME_DOMAIN`
pub const DRC_DEC_TEST_TIME_DOMAIN: DrcDecCodecMode = -100;
/// `DRC_DEC_TEST_QMF_DOMAIN`
pub const DRC_DEC_TEST_QMF_DOMAIN: DrcDecCodecMode = -99;
/// `DRC_DEC_TEST_STFT_DOMAIN`
pub const DRC_DEC_TEST_STFT_DOMAIN: DrcDecCodecMode = -98;
/// `DRC_DEC_CODEC_MODE_UNDEFINED`
pub const DRC_DEC_CODEC_MODE_UNDEFINED: DrcDecCodecMode = -1;
/// `DRC_DEC_MPEG_4_AAC`
pub const DRC_DEC_MPEG_4_AAC: DrcDecCodecMode = 0;
/// `DRC_DEC_MPEG_D_USAC`
pub const DRC_DEC_MPEG_D_USAC: DrcDecCodecMode = 1;
/// `DRC_DEC_MPEG_H_3DA`
pub const DRC_DEC_MPEG_H_3DA: DrcDecCodecMode = 2;

/// `DRC_DEC_LOCATION`: apply only DRC sets dedicated to processing location.
/// DRC1: before downmix, DRC2/DRC3: after downmix
pub type DrcDecLocation = i32;
/// `DRC_DEC_DRC1`
pub const DRC_DEC_DRC1: DrcDecLocation = 0;
/// `DRC_DEC_DRC2_DRC3`
pub const DRC_DEC_DRC2_DRC3: DrcDecLocation = 1;

/* Decoder library info */
/// `DRCDEC_LIB_VL0`
pub const DRCDEC_LIB_VL0: i32 = 2;
/// `DRCDEC_LIB_VL1`
pub const DRCDEC_LIB_VL1: i32 = 8;
/// `DRCDEC_LIB_VL2`
pub const DRCDEC_LIB_VL2: i32 = 0;
/// `DRCDEC_LIB_TITLE`
pub const DRCDEC_LIB_TITLE: &str = "MPEG-D DRC Decoder Lib";

/// `DRC_DEC_STATUS`
type DrcDecStatus = i32;
const DRC_DEC_NOT_INITIALIZED: DrcDecStatus = 0;
const DRC_DEC_INITIALIZED: DrcDecStatus = 1;
const DRC_DEC_NEW_GAIN_PAYLOAD: DrcDecStatus = 2;
const DRC_DEC_INTERPOLATION_PREPARED: DrcDecStatus = 3;
const DRCDEC_INCOMPLETE_UNIDRC: DrcDecStatus = 4;

/// MPEG-D DRC decoder instance (`struct s_drc_decoder`, `HANDLE_DRC_DECODER`).
///
/// Created with [`DrcDecoder::open`] (`FDK_drcDec_Open`); dropping it corresponds to
/// `FDK_drcDec_Close`. The fields are public for inspection (e.g. by differential tests).
#[derive(StateDigest, Clone, Debug)]
pub struct DrcDecoder {
    pub codec_mode: DrcDecCodecMode,
    pub functional_range: DrcDecFunctionalRange,
    pub status: i32,

    /* handles of submodules */
    pub h_gain_dec: Option<Box<DrcGainDecoder>>,
    pub h_selection_proc: Option<Box<DrcSelectionProcess>>,
    pub sel_proc_input_diff: i32,

    /* data structs */
    pub uni_drc_config: Box<UniDrcConfig>,
    pub loudness_info_set: Box<LoudnessInfoSet>,
    pub uni_drc_gain: Box<UniDrcGain>,

    pub sel_proc_output: SelProcOutput,
}

/// `_compAssign(UCHAR* dest, const UCHAR src)`: compare and assign
fn comp_assign_u8(dest: &mut u8, src: u8) -> i32 {
    let diff = (*dest != src) as i32;
    *dest = src;
    diff
}

impl DrcDecoder {
    fn is_reset_needed(&self, old_sel_proc_output: &SelProcOutput) -> bool {
        let mut reset_needed = false;
        let out = &self.sel_proc_output;

        if out.num_selected_drc_sets != old_sel_proc_output.num_selected_drc_sets {
            reset_needed = true;
        } else {
            for i in 0..(out.num_selected_drc_sets as usize).min(MAX_ACTIVE_DRCS) {
                if out.selected_drc_set_ids[i] != old_sel_proc_output.selected_drc_set_ids[i] {
                    reset_needed = true;
                }
                if out.selected_downmix_ids[i] != old_sel_proc_output.selected_downmix_ids[i] {
                    reset_needed = true;
                }
            }
        }

        if out.boost != old_sel_proc_output.boost {
            reset_needed = true;
        }
        if out.compress != old_sel_proc_output.compress {
            reset_needed = true;
        }

        /* Note: Changes in downmix matrix are not caught, as they don't affect the DRC gain
         * decoder */

        reset_needed
    }

    /// `startSelectionProcess(hDrcDec)`
    fn start_selection_process(&mut self) {
        let mut uni_drc_config_has_changed = 0;

        if self.status == 0 {
            return;
        }

        let old_sel_proc_output = self.sel_proc_output.clone();

        if self.functional_range & DRC_DEC_SELECTION != 0 {
            uni_drc_config_has_changed = self.uni_drc_config.diff;
            if self.uni_drc_config.diff != 0
                || self.loudness_info_set.diff != 0
                || self.sel_proc_input_diff != 0
            {
                /* in case of an error, signal that selection process was not successful */
                self.sel_proc_output.num_selected_drc_sets = 0;

                if let Some(sp) = self.h_selection_proc.as_deref_mut() {
                    drc_dec_selection_process_process(
                        sp,
                        &mut self.uni_drc_config,
                        &self.loudness_info_set,
                        &mut self.sel_proc_output,
                    );
                }

                self.sel_proc_input_diff = 0;
                self.uni_drc_config.diff = 0;
                self.loudness_info_set.diff = 0;
            }
        }

        if self.functional_range & DRC_DEC_GAIN != 0
            && (self.is_reset_needed(&old_sel_proc_output) || uni_drc_config_has_changed != 0)
        {
            if let Some(gd) = self.h_gain_dec.as_deref_mut() {
                drc_dec_gain_decoder_config(
                    gd,
                    &self.uni_drc_config,
                    self.sel_proc_output.num_selected_drc_sets,
                    &self.sel_proc_output.selected_drc_set_ids,
                    &self.sel_proc_output.selected_downmix_ids,
                );
            }
        }
    }

    /// `FDK_drcDec_Open(phDrcDec, functionalRange)`
    pub fn open(functional_range: DrcDecFunctionalRange) -> Box<DrcDecoder> {
        let mut h = Box::new(DrcDecoder {
            codec_mode: DRC_DEC_CODEC_MODE_UNDEFINED,
            functional_range,
            status: DRC_DEC_NOT_INITIALIZED,
            h_gain_dec: None,
            h_selection_proc: None,
            sel_proc_input_diff: 0,
            uni_drc_config: Box::new(UniDrcConfig::ZERO),
            loudness_info_set: Box::new(LoudnessInfoSet::ZERO),
            uni_drc_gain: Box::new(UniDrcGain::ZERO),
            sel_proc_output: SelProcOutput::ZERO,
        });

        if h.functional_range & DRC_DEC_SELECTION != 0 {
            let mut sp = drc_dec_selection_process_create();
            drc_dec_selection_process_init(&mut sp);
            h.h_selection_proc = Some(sp);
            h.sel_proc_output.output_loudness = UNDEFINED_LOUDNESS_VALUE;
            h.sel_proc_input_diff = 1;
        }

        if h.functional_range & DRC_DEC_GAIN != 0 {
            h.h_gain_dec = Some(drc_dec_gain_decoder_open());
        }

        h
    }

    /// `FDK_drcDec_SetCodecMode(hDrcDec, codecMode)`
    pub fn set_codec_mode(&mut self, codec_mode: DrcDecCodecMode) -> DrcDecError {
        if self.codec_mode == DRC_DEC_CODEC_MODE_UNDEFINED {
            /* Set codec mode, if it is set for the first time */
            self.codec_mode = codec_mode;

            if self.functional_range & DRC_DEC_SELECTION != 0 {
                if let Some(sp) = self.h_selection_proc.as_deref_mut() {
                    let s_err = drc_dec_selection_process_set_codec_mode(sp, codec_mode);
                    if s_err != 0 {
                        return DRC_DEC_NOT_OK;
                    }
                }
                self.sel_proc_input_diff = 1;
            }

            if self.functional_range & DRC_DEC_GAIN != 0 {
                let delay_mode = DM_REGULAR_DELAY;
                let (time_domain_supported, subband_domain_supported) = match self.codec_mode {
                    DRC_DEC_MPEG_4_AAC | DRC_DEC_MPEG_D_USAC => (1, SDM_OFF),
                    DRC_DEC_MPEG_H_3DA => (1, SDM_STFT256),
                    DRC_DEC_TEST_TIME_DOMAIN => (1, SDM_OFF),
                    DRC_DEC_TEST_QMF_DOMAIN => (0, SDM_QMF64),
                    DRC_DEC_TEST_STFT_DOMAIN => (0, SDM_STFT256),
                    _ => (1, SDM_OFF),
                };

                if let Some(gd) = self.h_gain_dec.as_deref_mut() {
                    let d_err = drc_dec_gain_decoder_set_codec_dependent_parameters(
                        gd,
                        delay_mode,
                        time_domain_supported,
                        subband_domain_supported,
                    );
                    if d_err != 0 {
                        return DRC_DEC_NOT_OK;
                    }
                }
            }
        }

        /* Don't allow changing codecMode if it has already been set. */
        if self.codec_mode != codec_mode {
            return DRC_DEC_NOT_OK;
        }

        DRC_DEC_OK
    }

    /// `FDK_drcDec_Init(hDrcDec, frameSize, sampleRate, baseChannelCount)`
    pub fn init(
        &mut self,
        frame_size: i32,
        sample_rate: i32,
        base_channel_count: i32,
    ) -> DrcDecError {
        if frame_size == 0 || sample_rate == 0 || base_channel_count == 0 {
            return DRC_DEC_OK; /* return without doing anything */
        }

        if self.functional_range & DRC_DEC_SELECTION != 0 {
            if let Some(sp) = self.h_selection_proc.as_deref_mut() {
                let s_err = drc_dec_selection_process_set_param(
                    sp,
                    SEL_PROC_BASE_CHANNEL_COUNT,
                    base_channel_count,
                    Some(&mut self.sel_proc_input_diff),
                );
                if s_err != 0 {
                    return DRC_DEC_NOT_OK;
                }
                let s_err = drc_dec_selection_process_set_param(
                    sp,
                    SEL_PROC_SAMPLE_RATE,
                    sample_rate,
                    Some(&mut self.sel_proc_input_diff),
                );
                if s_err != 0 {
                    return DRC_DEC_NOT_OK;
                }
            }
        }

        if self.functional_range & DRC_DEC_GAIN != 0 {
            if let Some(gd) = self.h_gain_dec.as_deref_mut() {
                if drc_dec_gain_decoder_set_param(gd, GAIN_DEC_FRAME_SIZE, frame_size) != 0 {
                    return DRC_DEC_NOT_OK;
                }
                if drc_dec_gain_decoder_set_param(gd, GAIN_DEC_SAMPLE_RATE, sample_rate) != 0 {
                    return DRC_DEC_NOT_OK;
                }
                if drc_dec_gain_decoder_init(gd) != 0 {
                    return DRC_DEC_NOT_OK;
                }
            }
        }

        self.status = DRC_DEC_INITIALIZED;

        self.start_selection_process();

        DRC_DEC_OK
    }

    /// `FDK_drcDec_SetParam(hDrcDec, requestType, requestValue)`: set single user request
    pub fn set_param(
        &mut self,
        request_type: DrcDecUserParam,
        request_value: FixpDbl,
    ) -> DrcDecError {
        let mut invalid_parameter = 0;

        if self.functional_range & DRC_DEC_GAIN != 0 {
            match request_type {
                DRC_DEC_SAMPLE_RATE => {
                    if let Some(gd) = self.h_gain_dec.as_deref_mut() {
                        if drc_dec_gain_decoder_set_param(gd, GAIN_DEC_SAMPLE_RATE, request_value)
                            != 0
                        {
                            return DRC_DEC_PARAM_OUT_OF_RANGE;
                        }
                    }
                }
                DRC_DEC_FRAME_SIZE => {
                    if let Some(gd) = self.h_gain_dec.as_deref_mut() {
                        if drc_dec_gain_decoder_set_param(gd, GAIN_DEC_FRAME_SIZE, request_value)
                            != 0
                        {
                            return DRC_DEC_PARAM_OUT_OF_RANGE;
                        }
                    }
                }
                _ => invalid_parameter |= DRC_DEC_GAIN,
            }
        }

        if self.functional_range & DRC_DEC_SELECTION != 0 {
            let sel_param = match request_type {
                DRC_DEC_BOOST => Some(SEL_PROC_BOOST),
                DRC_DEC_COMPRESS => Some(SEL_PROC_COMPRESS),
                DRC_DEC_LOUDNESS_NORMALIZATION_ON => Some(SEL_PROC_LOUDNESS_NORMALIZATION_ON),
                DRC_DEC_TARGET_LOUDNESS => Some(SEL_PROC_TARGET_LOUDNESS),
                DRC_DEC_EFFECT_TYPE => Some(SEL_PROC_EFFECT_TYPE),
                DRC_DEC_EFFECT_TYPE_FALLBACK_CODE => Some(SEL_PROC_EFFECT_TYPE_FALLBACK_CODE),
                DRC_DEC_DOWNMIX_ID => Some(SEL_PROC_DOWNMIX_ID),
                DRC_DEC_ALBUM_MODE => Some(SEL_PROC_ALBUM_MODE),
                _ => None,
            };
            match sel_param {
                Some(sp_param) => {
                    if let Some(sp) = self.h_selection_proc.as_deref_mut() {
                        let s_err = drc_dec_selection_process_set_param(
                            sp,
                            sp_param,
                            request_value,
                            Some(&mut self.sel_proc_input_diff),
                        );
                        if s_err != 0 {
                            return DRC_DEC_PARAM_OUT_OF_RANGE;
                        }
                    }
                }
                None => invalid_parameter |= DRC_DEC_SELECTION,
            }
        }

        if invalid_parameter == self.functional_range {
            return DRC_DEC_INVALID_PARAM;
        }

        /* All parameters need a new start of the selection process */
        self.start_selection_process();

        DRC_DEC_OK
    }

    /// `FDK_drcDec_GetParam(hDrcDec, requestType)`
    pub fn get_param(&self, request_type: DrcDecUserParam) -> i32 {
        match request_type {
            DRC_DEC_IS_MULTIBAND_DRC_1 => bitstream_contains_multiband_drc(&self.uni_drc_config, 0),
            DRC_DEC_IS_ACTIVE => {
                /* MPEG-D DRC overrides MPEG-4 DRC, if uniDrc payload is present (loudnessInfoSet
                 * and/or uniDrcConfig) */
                let mut uni_drc_payload_present =
                    (self.loudness_info_set.loudness_info_count > 0) as i32;
                uni_drc_payload_present |=
                    (self.loudness_info_set.loudness_info_album_count > 0) as i32;
                uni_drc_payload_present |=
                    (self.uni_drc_config.drc_instructions_uni_drc_count > 0) as i32;
                uni_drc_payload_present |=
                    (self.uni_drc_config.downmix_instructions_count > 0) as i32;

                uni_drc_payload_present
            }
            DRC_DEC_TARGET_LAYOUT_SELECTED => self.sel_proc_output.target_layout as i32,
            DRC_DEC_TARGET_CHANNEL_COUNT_SELECTED => {
                self.sel_proc_output.target_channel_count as i32
            }
            DRC_DEC_OUTPUT_LOUDNESS => self.sel_proc_output.output_loudness,
            DRC_DEC_IS_MUTE => {
                /* If uniDrc() payload is present, but no loudnessInfoSet contained so far,
                DRC decoder output shall be mute for 2.5 seconds after startup. */
                if self.status == DRCDEC_INCOMPLETE_UNIDRC {
                    drc_dec_gain_decoder_get_startup_phase(self.h_gain_dec.as_deref())
                } else {
                    0
                }
            }
            _ => 0,
        }
    }

    /// `FDK_drcDec_SetInterfaceParameters(hDrcDec, uniDrcInterface)`: not supported.
    pub fn set_interface_parameters(&mut self) -> DrcDecError {
        DRC_DEC_UNSUPPORTED_FUNCTION
    }

    /// `FDK_drcDec_SetSelectionProcessMpeghParameters(hDrcDec, numGroupIdsRequested,
    /// groupIdsRequested, numGroupPresetIdsRequested, groupPresetIdsRequested,
    /// numMembersGroupPresetIdsRequested, groupPresetIdRequestedPreference)`
    pub fn set_selection_process_mpegh_parameters(
        &mut self,
        num_group_ids_requested: i32,
        group_ids_requested: &[i32],
        num_group_preset_ids_requested: i32,
        group_preset_ids_requested: &[i32],
        num_members_group_preset_ids_requested: &[i32],
        group_preset_id_requested_preference: i32,
    ) -> DrcDecError {
        if self.functional_range == DRC_DEC_GAIN {
            return DRC_DEC_NOT_OK;
        }

        if let Some(sp) = self.h_selection_proc.as_deref_mut() {
            let s_err = drc_dec_selection_process_set_mpegh_params(
                sp,
                num_group_ids_requested,
                group_ids_requested,
                num_group_preset_ids_requested,
                group_preset_ids_requested,
                num_members_group_preset_ids_requested,
                group_preset_id_requested_preference,
                Some(&mut self.sel_proc_input_diff),
            );
            if s_err != 0 {
                return DRC_DEC_NOT_OK;
            }
        }

        self.start_selection_process();

        DRC_DEC_OK
    }

    /// `FDK_drcDec_SetSelectionProcessMpeghParameters_simple(hDrcDec, groupPresetIdRequested,
    /// numGroupIdsRequested, groupIdsRequested)`
    pub fn set_selection_process_mpegh_parameters_simple(
        &mut self,
        group_preset_id_requested: i32,
        num_group_ids_requested: i32,
        group_ids_requested: &[i32],
    ) -> DrcDecError {
        let mut num_group_preset_ids_requested = 1;
        let num_members_group_preset_ids_requested = 1;

        if group_preset_id_requested < 0 {
            num_group_preset_ids_requested = 0;
        }

        self.set_selection_process_mpegh_parameters(
            num_group_ids_requested,
            group_ids_requested,
            num_group_preset_ids_requested,
            &[group_preset_id_requested],
            &[num_members_group_preset_ids_requested],
            group_preset_id_requested,
        )
    }

    /// `FDK_drcDec_SetDownmixInstructions(hDrcDec, numDowmixId, downmixId, targetLayout,
    /// targetChannelCount)`
    pub fn set_downmix_instructions(
        &mut self,
        num_downmix_id: i32,
        downmix_id: &[i32],
        target_layout: &[i32],
        target_channel_count: &[i32],
    ) -> DrcDecError {
        let mut diff = 0;

        if num_downmix_id > 32 {
            return DRC_DEC_NOT_OK;
        }

        diff |= comp_assign_u8(
            &mut self.uni_drc_config.downmix_instructions_count,
            num_downmix_id as u8,
        );

        for i in 0..num_downmix_id.max(0) as usize {
            let d = &mut self.uni_drc_config.downmix_instructions[i];
            diff |= comp_assign_u8(&mut d.downmix_id, downmix_id[i] as u8);
            diff |= comp_assign_u8(&mut d.target_layout, target_layout[i] as u8);
            diff |= comp_assign_u8(&mut d.target_channel_count, target_channel_count[i] as u8);
            diff |= comp_assign_u8(&mut d.downmix_coefficients_present, 0);
        }

        self.sel_proc_input_diff |= diff;

        self.start_selection_process();

        DRC_DEC_OK
    }

    /// `FDK_drcDec_SetSelectionProcessOutput(hDrcDec, hSelProcOutput)`: does nothing.
    pub fn set_selection_process_output(&mut self, _h_sel_proc_output: &SelProcOutput) {}

    /// `FDK_drcDec_GetSelectionProcessOutput(hDrcDec)`
    pub fn get_selection_process_output(&self) -> &SelProcOutput {
        &self.sel_proc_output
    }

    /// `FDK_drcDec_SetChannelGains(hDrcDec, updateLNgain, numChannels, frameSize, channelGainDb,
    /// audioBuffer, audioBufferChannelOffset)`
    ///
    /// `audio_buffer` starts at the first sample of the first channel.
    pub fn set_channel_gains(
        &mut self,
        update_ln_gain: i32,
        num_channels: i32,
        frame_size: i32,
        channel_gain_db: &[FixpDbl],
        audio_buffer: &mut [FixpDbl],
        audio_buffer_channel_offset: i32,
    ) {
        // The C code dereferences hGainDec unconditionally (a decoder without DRC_DEC_GAIN would
        // crash); nothing is done here in that case.
        let Some(gd) = self.h_gain_dec.as_deref_mut() else {
            return;
        };

        if update_ln_gain != 0 {
            let err = drc_dec_gain_decoder_set_loudness_normalization_gain_db(
                gd,
                self.sel_proc_output.loudness_normalization_gain_db,
            );
            if err != 0 {
                return;
            }
        }

        drc_dec_gain_decoder_set_channel_gains(
            gd,
            num_channels,
            frame_size,
            channel_gain_db,
            audio_buffer_channel_offset,
            audio_buffer,
        );
    }

    /// `FDK_drcDec_ReadUniDrcConfig(hDrcDec, hBitstream, subStreamIndex)`
    pub fn read_uni_drc_config(
        &mut self,
        h_bitstream: &mut FdkBitStream,
        sub_stream_index: i32,
    ) -> DrcDecError {
        self.uni_drc_config.loudness_info_set_present = 0; /* set to zero for error handling */
        let d_err = if self.codec_mode == DRC_DEC_MPEG_H_3DA {
            drc_dec_read_mpegh3da_uni_drc_config(
                h_bitstream,
                &mut self.uni_drc_config,
                &mut self.loudness_info_set,
                sub_stream_index,
            )
        } else {
            return DRC_DEC_NOT_OK;
        };

        if d_err != 0 {
            return DRC_DEC_PARSE_ERROR;
        }

        self.start_selection_process();

        DRC_DEC_OK
    }

    /// `FDK_drcDec_ReadLoudnessInfoSet(hDrcDec, hBitstream, subStreamIndex)`
    pub fn read_loudness_info_set(
        &mut self,
        h_bitstream: &mut FdkBitStream,
        sub_stream_index: i32,
    ) -> DrcDecError {
        let d_err = if self.codec_mode == DRC_DEC_MPEG_H_3DA {
            drc_dec_read_mpegh3da_loudness_info_set(
                h_bitstream,
                &mut self.loudness_info_set,
                sub_stream_index,
            )
        } else {
            return DRC_DEC_NOT_OK;
        };

        if d_err != 0 {
            return DRC_DEC_PARSE_ERROR;
        }

        self.start_selection_process();

        DRC_DEC_OK
    }

    /// `FDK_drcDec_ReadUniDrcGain(hDrcDec, hBitstream, subStreamIndex)`
    pub fn read_uni_drc_gain(
        &mut self,
        h_bitstream: &mut FdkBitStream,
        sub_stream_index: i32,
    ) -> DrcDecError {
        if self.status == 0 {
            return DRC_DEC_OK;
        }

        let gd = self.h_gain_dec.as_deref();
        let d_err = drc_dec_read_uni_drc_gain(
            h_bitstream,
            drc_dec_gain_decoder_get_drc_coefficients(gd),
            drc_dec_gain_decoder_get_frame_size(gd),
            drc_dec_gain_decoder_get_delta_tmin_default(gd),
            sub_stream_index,
            &mut self.uni_drc_gain,
        );
        if d_err != 0 {
            return DRC_DEC_PARSE_ERROR;
        }

        if self.uni_drc_gain.status != 0 {
            self.status = DRC_DEC_NEW_GAIN_PAYLOAD;
        }

        DRC_DEC_OK
    }

    /// A copy of the state that [`DrcDecoder::read_uni_drc_gain`] reads (not in C; used by the
    /// frame pipelining of the core decoder to parse gain payloads while the decoder is in use).
    pub fn gain_parser(&self) -> UniDrcGainParser {
        let gd = self.h_gain_dec.as_deref();
        UniDrcGainParser {
            active: self.status != 0,
            coef: drc_dec_gain_decoder_get_drc_coefficients(gd).cloned(),
            frame_size: drc_dec_gain_decoder_get_frame_size(gd),
            delta_tmin_default: drc_dec_gain_decoder_get_delta_tmin_default(gd),
            scratch: Box::new(UniDrcGain::ZERO),
        }
    }

    /// Whether `p` is a [`DrcDecoder::gain_parser`] of the current state.
    pub fn gain_parser_matches(&self, p: &UniDrcGainParser) -> bool {
        let gd = self.h_gain_dec.as_deref();
        p.active == (self.status != 0)
            && p.coef.as_ref() == drc_dec_gain_decoder_get_drc_coefficients(gd)
            && p.frame_size == drc_dec_gain_decoder_get_frame_size(gd)
            && p.delta_tmin_default == drc_dec_gain_decoder_get_delta_tmin_default(gd)
    }

    /// `FDK_drcDec_ReadUniDrc(hDrcDec, hBitstream)`: not supported.
    pub fn read_uni_drc(&mut self, _h_bitstream: &mut FdkBitStream) -> DrcDecError {
        DRC_DEC_UNSUPPORTED_FUNCTION
    }

    /// `FDK_drcDec_Preprocess(hDrcDec)`
    pub fn preprocess(&mut self) -> DrcDecError {
        if self.status == 0 {
            return DRC_DEC_NOT_READY;
        }
        if self.functional_range & DRC_DEC_GAIN == 0 {
            return DRC_DEC_NOT_OK;
        }
        let Some(gd) = self.h_gain_dec.as_deref_mut() else {
            return DRC_DEC_NOT_OK;
        };

        if self.status != DRC_DEC_NEW_GAIN_PAYLOAD {
            /* no new gain payload was read, e.g. during concealment or flushing.
            Generate DRC gains based on the stored DRC gains of last frames */
            drc_dec_gain_decoder_conceal(gd, &self.uni_drc_config, &mut self.uni_drc_gain);
        }

        let d_err = drc_dec_gain_decoder_preprocess(
            gd,
            &mut self.uni_drc_gain,
            self.sel_proc_output.loudness_normalization_gain_db,
            self.sel_proc_output.boost,
            self.sel_proc_output.compress,
        );
        if d_err != 0 {
            return DRC_DEC_NOT_OK;
        }
        self.status = DRC_DEC_INTERPOLATION_PREPARED;

        DRC_DEC_OK
    }

    /// `FDK_drcDec_ProcessTime(hDrcDec, delaySamples, drcLocation, channelOffset,
    /// drcChannelOffset, numChannelsProcessed, realBuffer, timeDataChannelOffset)`
    ///
    /// `real_buffer` holds the deinterleaved time signal; channel `c` starts at
    /// `c * time_data_channel_offset`.
    pub fn process_time(
        &mut self,
        delay_samples: i32,
        drc_location: DrcDecLocation,
        channel_offset: i32,
        drc_channel_offset: i32,
        num_channels_processed: i32,
        real_buffer: &mut [FixpDbl],
        time_data_channel_offset: i32,
    ) -> DrcDecError {
        if self.functional_range & DRC_DEC_GAIN == 0 {
            return DRC_DEC_NOT_OK;
        }
        if self.status != DRC_DEC_INTERPOLATION_PREPARED {
            return DRC_DEC_NOT_READY;
        }
        let Some(gd) = self.h_gain_dec.as_deref_mut() else {
            return DRC_DEC_NOT_OK;
        };

        let d_err = drc_dec_gain_decoder_process_time_domain(
            gd,
            delay_samples,
            drc_location,
            channel_offset,
            drc_channel_offset,
            num_channels_processed,
            time_data_channel_offset,
            real_buffer,
        );
        if d_err != 0 {
            return DRC_DEC_NOT_OK;
        }

        DRC_DEC_OK
    }

    /// `FDK_drcDec_ProcessFreq(hDrcDec, delaySamples, drcLocation, channelOffset,
    /// drcChannelOffset, numChannelsProcessed, processSingleTimeslot, realBuffer, imagBuffer)`
    ///
    /// `real_buffer[i]` / `imag_buffer[i]` hold the subband signal of the `i`-th processed
    /// channel. If `imag_buffer` is `None`, the signal in `real_buffer` is interleaved (real,
    /// imag, real, imag, ...).
    pub fn process_freq<R: AsMut<[FixpDbl]>>(
        &mut self,
        delay_samples: i32,
        drc_location: DrcDecLocation,
        channel_offset: i32,
        drc_channel_offset: i32,
        num_channels_processed: i32,
        process_single_timeslot: i32,
        real_buffer: &mut [R],
        imag_buffer: Option<&mut [R]>,
    ) -> DrcDecError {
        if self.functional_range & DRC_DEC_GAIN == 0 {
            return DRC_DEC_NOT_OK;
        }
        if self.status != DRC_DEC_INTERPOLATION_PREPARED {
            return DRC_DEC_NOT_READY;
        }
        let Some(gd) = self.h_gain_dec.as_deref_mut() else {
            return DRC_DEC_NOT_OK;
        };

        let d_err = drc_dec_gain_decoder_process_subband_domain(
            gd,
            delay_samples,
            drc_location,
            channel_offset,
            drc_channel_offset,
            num_channels_processed,
            process_single_timeslot,
            real_buffer,
            imag_buffer,
        );
        if d_err != 0 {
            return DRC_DEC_NOT_OK;
        }

        DRC_DEC_OK
    }

    /// `FDK_drcDec_ApplyDownmix(hDrcDec, reverseInChannelMap, reverseOutChannelMap, realBuffer,
    /// pNChannels)`: not supported.
    pub fn apply_downmix(&mut self) -> DrcDecError {
        DRC_DEC_UNSUPPORTED_FUNCTION
    }
}

/// The state of a [`DrcDecoder`] that `FDK_drcDec_ReadUniDrcGain()` reads (not in C, see
/// [`DrcDecoder::gain_parser`]): the parse of a gain payload with
/// [`UniDrcGainParser::read_uni_drc_gain`] reads the same bits and returns the same error as
/// [`DrcDecoder::read_uni_drc_gain`] of that decoder, which only differs by storing the gains.
#[derive(StateDigest, Clone, Debug)]
pub struct UniDrcGainParser {
    /// `hDrcDec->status != 0`
    active: bool,
    /// `drcDec_GainDecoder_GetDrcCoefficients(hGainDec)`
    coef: Option<DrcCoefficientsUniDrc>,
    frame_size: i32,
    delta_tmin_default: i32,
    /// Receives the parsed gains.
    scratch: Box<UniDrcGain>,
}

impl UniDrcGainParser {
    /// [`DrcDecoder::read_uni_drc_gain`] without storing the gains.
    pub fn read_uni_drc_gain(
        &mut self,
        h_bitstream: &mut FdkBitStream,
        sub_stream_index: i32,
    ) -> DrcDecError {
        if !self.active {
            return DRC_DEC_OK;
        }
        let d_err = drc_dec_read_uni_drc_gain(
            h_bitstream,
            self.coef.as_ref(),
            self.frame_size,
            self.delta_tmin_default,
            sub_stream_index,
            &mut self.scratch,
        );
        if d_err != 0 {
            return DRC_DEC_PARSE_ERROR;
        }
        DRC_DEC_OK
    }
}

/// `FDK_drcDec_GetGroupLoudness(hSelProcOutput, groupID, groupLoudnessAvailable)`: group loudness
/// (`FIXP_DBL`, e = 7)
pub fn get_group_loudness(
    h_sel_proc_output: Option<&SelProcOutput>,
    group_id: i32,
    group_loudness_available: &mut i32,
) -> i32 {
    *group_loudness_available = 0;
    let Some(out) = h_sel_proc_output else {
        return 0;
    };

    for j in 0..(out.group_id_loudness_count as usize).min(32) {
        if out.group_id[j] as i32 == group_id {
            *group_loudness_available = 1;
            return out.group_id_loudness[j];
        }
    }

    /* groupLoudness not found */
    0
}
