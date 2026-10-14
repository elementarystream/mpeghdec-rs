// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDec_selectionProcess.cpp, drcDec_selectionProcess.h.

//! DRC set selection according to section 6.2 of ISO/IEC 23003-4 (MPEG-D DRC), including
//! ISO/IEC 23003-4/AMD1 (Amendment 1) and section 6.4.4 of ISO/IEC 23008-3 (MPEG-H 3D Audio).

#![allow(clippy::nonminimal_bool)]

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::scale::saturate_left_shift;
use mpeghdec_fdk::{
    f_max, f_min, fl2fxconst_dbl, fl2fxconst_sgl, fx_dbl2fx_sgl, fx_sgl2fx_dbl, FixpDbl, FixpSgl,
};
use mpeghdec_fdk::{DFRACT_BITS, MAXVAL_DBL, MINVAL_DBL};

use crate::drc_dec_reader::{comp_assign, gain_set_at};
use crate::drc_dec_tools::*;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;

/// `MAX_REQUESTS_GROUP_ID`: mae_numGroups for MPEG-H LC Profile Level 4
pub const MAX_REQUESTS_GROUP_ID: usize = 28;
/// `MAX_REQUESTS_GROUP_PRESET_ID`: mae_numGroupPresets for MPEG-H LC Profile Level 4
pub const MAX_REQUESTS_GROUP_PRESET_ID: usize = 16;

/// `UNDEFINED_LOUDNESS_VALUE`
pub const UNDEFINED_LOUDNESS_VALUE: FixpDbl = MAXVAL_DBL - 1;

/// `DRCDEC_SELECTION_PROCESS_RETURN`
pub type DrcDecSelectionProcessReturn = i32;
/// `DRCDEC_SELECTION_PROCESS_NO_ERROR`
pub const DRCDEC_SELECTION_PROCESS_NO_ERROR: DrcDecSelectionProcessReturn = 0;
/// `DRCDEC_SELECTION_PROCESS_WARNING`
pub const DRCDEC_SELECTION_PROCESS_WARNING: DrcDecSelectionProcessReturn = -1000;
/// `DRCDEC_SELECTION_PROCESS_NOT_OK`
pub const DRCDEC_SELECTION_PROCESS_NOT_OK: DrcDecSelectionProcessReturn = -2000;
/// `DRCDEC_SELECTION_PROCESS_OUTOFMEMORY`
pub const DRCDEC_SELECTION_PROCESS_OUTOFMEMORY: DrcDecSelectionProcessReturn = -1999;
/// `DRCDEC_SELECTION_PROCESS_INVALID_HANDLE`
pub const DRCDEC_SELECTION_PROCESS_INVALID_HANDLE: DrcDecSelectionProcessReturn = -1998;
/// `DRCDEC_SELECTION_PROCESS_NOT_SUPPORTED`
pub const DRCDEC_SELECTION_PROCESS_NOT_SUPPORTED: DrcDecSelectionProcessReturn = -1997;
/// `DRCDEC_SELECTION_PROCESS_INVALID_PARAM`
pub const DRCDEC_SELECTION_PROCESS_INVALID_PARAM: DrcDecSelectionProcessReturn = -1996;
/// `DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE`
pub const DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE: DrcDecSelectionProcessReturn = -1995;

/// `SEL_PROC_CODEC_MODE`
pub type SelProcCodecMode = i32;
/// `SEL_PROC_TEST_TIME_DOMAIN`
pub const SEL_PROC_TEST_TIME_DOMAIN: SelProcCodecMode = -100;
/// `SEL_PROC_TEST_QMF_DOMAIN`
pub const SEL_PROC_TEST_QMF_DOMAIN: SelProcCodecMode = -99;
/// `SEL_PROC_TEST_STFT_DOMAIN`
pub const SEL_PROC_TEST_STFT_DOMAIN: SelProcCodecMode = -98;
/// `SEL_PROC_CODEC_MODE_UNDEFINED`
pub const SEL_PROC_CODEC_MODE_UNDEFINED: SelProcCodecMode = -1;
/// `SEL_PROC_MPEG_4_AAC`
pub const SEL_PROC_MPEG_4_AAC: SelProcCodecMode = 0;
/// `SEL_PROC_MPEG_D_USAC`
pub const SEL_PROC_MPEG_D_USAC: SelProcCodecMode = 1;
/// `SEL_PROC_MPEG_H_3DA`
pub const SEL_PROC_MPEG_H_3DA: SelProcCodecMode = 2;

/// `SEL_PROC_USER_PARAM`
pub type SelProcUserParam = i32;
/// `SEL_PROC_LOUDNESS_NORMALIZATION_ON` (set and get user param)
pub const SEL_PROC_LOUDNESS_NORMALIZATION_ON: SelProcUserParam = 0;
/// `SEL_PROC_DYNAMIC_RANGE_CONTROL_ON` (get only user param)
pub const SEL_PROC_DYNAMIC_RANGE_CONTROL_ON: SelProcUserParam = 1;
/// `SEL_PROC_TARGET_LOUDNESS` (set only user params from here on)
pub const SEL_PROC_TARGET_LOUDNESS: SelProcUserParam = 2;
/// `SEL_PROC_EFFECT_TYPE`
pub const SEL_PROC_EFFECT_TYPE: SelProcUserParam = 3;
/// `SEL_PROC_EFFECT_TYPE_FALLBACK_CODE`
pub const SEL_PROC_EFFECT_TYPE_FALLBACK_CODE: SelProcUserParam = 4;
/// `SEL_PROC_LOUDNESS_MEASUREMENT_METHOD`
pub const SEL_PROC_LOUDNESS_MEASUREMENT_METHOD: SelProcUserParam = 5;
/// `SEL_PROC_ALBUM_MODE`
pub const SEL_PROC_ALBUM_MODE: SelProcUserParam = 6;
/// `SEL_PROC_DOWNMIX_ID`
pub const SEL_PROC_DOWNMIX_ID: SelProcUserParam = 7;
/// `SEL_PROC_TARGET_LAYOUT`
pub const SEL_PROC_TARGET_LAYOUT: SelProcUserParam = 8;
/// `SEL_PROC_TARGET_CHANNEL_COUNT`
pub const SEL_PROC_TARGET_CHANNEL_COUNT: SelProcUserParam = 9;
/// `SEL_PROC_BASE_CHANNEL_COUNT`
pub const SEL_PROC_BASE_CHANNEL_COUNT: SelProcUserParam = 10;
/// `SEL_PROC_SAMPLE_RATE`
pub const SEL_PROC_SAMPLE_RATE: SelProcUserParam = 11;
/// `SEL_PROC_BOOST`
pub const SEL_PROC_BOOST: SelProcUserParam = 12;
/// `SEL_PROC_COMPRESS`
pub const SEL_PROC_COMPRESS: SelProcUserParam = 13;

/// `SEL_PROC_OUTPUT` (`struct s_selection_process_output`)
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct SelProcOutput {
    /// e = 7
    pub output_peak_level_db: FixpDbl,
    /// e = 7
    pub loudness_normalization_gain_db: FixpDbl,
    /// e = 7
    pub output_loudness: FixpDbl,

    pub num_selected_drc_sets: u8,
    pub selected_drc_set_ids: [i8; MAX_ACTIVE_DRCS],
    pub selected_downmix_ids: [u8; MAX_ACTIVE_DRCS],

    pub active_downmix_id: u8,
    pub base_channel_count: u8,
    pub target_channel_count: u8,
    pub target_layout: i16,
    pub downmix_matrix_present: u8,

    /// e = 1
    pub boost: FixpSgl,
    /// e = 1
    pub compress: FixpSgl,

    pub group_id_loudness_count: u8,
    pub group_id: [u8; 32],
    /// e = 7
    pub group_id_loudness: [FixpDbl; 32],

    /// e = 7
    pub mixing_level: FixpDbl,
}

impl SelProcOutput {
    pub const ZERO: Self = Self {
        output_peak_level_db: 0,
        loudness_normalization_gain_db: 0,
        output_loudness: 0,
        num_selected_drc_sets: 0,
        selected_drc_set_ids: [0; MAX_ACTIVE_DRCS],
        selected_downmix_ids: [0; MAX_ACTIVE_DRCS],
        active_downmix_id: 0,
        base_channel_count: 0,
        target_channel_count: 0,
        target_layout: 0,
        downmix_matrix_present: 0,
        boost: 0,
        compress: 0,
        group_id_loudness_count: 0,
        group_id: [0; 32],
        group_id_loudness: [0; 32],
        mixing_level: 0,
    };
}

impl Default for SelProcOutput {
    fn default() -> Self {
        Self::ZERO
    }
}

/// `DRC_EFFECT_TYPE_REQUEST`
type DrcEffectTypeRequest = i32;
const DETR_NONE: DrcEffectTypeRequest = 0;
const DETR_NIGHT: DrcEffectTypeRequest = 1;
const DETR_NOISY: DrcEffectTypeRequest = 2;
const DETR_LIMITED: DrcEffectTypeRequest = 3;
const DETR_LOWLEVEL: DrcEffectTypeRequest = 4;
const DETR_DIALOG: DrcEffectTypeRequest = 5;
const DETR_GENERAL_COMPR: DrcEffectTypeRequest = 6;
#[allow(dead_code)]
const DETR_EXPAND: DrcEffectTypeRequest = 7;
#[allow(dead_code)]
const DETR_ARTISTIC: DrcEffectTypeRequest = 8;
const DETR_COUNT: DrcEffectTypeRequest = 9;

/// `DRC_FEATURE_REQUEST_TYPE`
type DrcFeatureRequestType = i32;
const DFRT_EFFECT_TYPE: DrcFeatureRequestType = 0;

/// `METHOD_DEFINITION_REQUEST`
type MethodDefinitionRequest = i32;
#[allow(dead_code)]
const MDR_DEFAULT: MethodDefinitionRequest = 0;
const MDR_PROGRAM_LOUDNESS: MethodDefinitionRequest = 1;
const MDR_ANCHOR_LOUDNESS: MethodDefinitionRequest = 2;

/// `MEASUREMENT_SYSTEM_REQUEST`
type MeasurementSystemRequest = i32;
const MSR_EXPERT_PANEL: MeasurementSystemRequest = 3;

/// `TARGET_CONFIG_REQUEST_TYPE`
type TargetConfigRequestType = i32;
const TCRT_DOWNMIX_ID: TargetConfigRequestType = 0;
const TCRT_TARGET_LAYOUT: TargetConfigRequestType = 1;
const TCRT_TARGET_CHANNEL_COUNT: TargetConfigRequestType = 2;

/// `DRC_FEATURE_REQUEST` (union with the single member `drcEffectType`)
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DrcFeatureRequest {
    pub num_requests: u8,
    pub num_requests_desired: u8,
    pub request: [i32; MAX_REQUESTS_DRC_EFFECT_TYPE],
}

impl DrcFeatureRequest {
    const ZERO: Self = Self {
        num_requests: 0,
        num_requests_desired: 0,
        request: [0; MAX_REQUESTS_DRC_EFFECT_TYPE],
    };
}

/// `MPEGH_REQUEST`
///
/// Stored as the raw bytes of the C structure (all members are `UCHAR`/`SCHAR`):
/// `numGroupIdsRequested`, `groupIdRequested[28]`, `numGroupPresetIdsRequested`,
/// `groupPresetIdRequested[16]`, `numMembersGroupPresetIdsRequested[16]`,
/// `groupPresetIdRequestedPreference`. The request counts can exceed the array sizes (they are
/// stored before the range check of `drcDec_SelectionProcess_SetMpeghParams`); the C code then
/// reads the following members, which the accessors reproduce. Reads beyond the structure are
/// undefined behaviour in C and return 0 here.
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct MpeghRequest {
    pub raw: [u8; MPEGH_REQUEST_SIZE],
}

/// Size of `MPEGH_REQUEST` in bytes.
pub const MPEGH_REQUEST_SIZE: usize =
    1 + MAX_REQUESTS_GROUP_ID + 1 + 2 * MAX_REQUESTS_GROUP_PRESET_ID + 1;
const OFS_GROUP_ID: usize = 1;
const OFS_NUM_PRESET: usize = OFS_GROUP_ID + MAX_REQUESTS_GROUP_ID;
const OFS_PRESET_ID: usize = OFS_NUM_PRESET + 1;
const OFS_NUM_MEMBERS: usize = OFS_PRESET_ID + MAX_REQUESTS_GROUP_PRESET_ID;
const OFS_PREFERENCE: usize = OFS_NUM_MEMBERS + MAX_REQUESTS_GROUP_PRESET_ID;

impl Default for MpeghRequest {
    fn default() -> Self {
        MpeghRequest { raw: [0; MPEGH_REQUEST_SIZE] }
    }
}

impl MpeghRequest {
    #[inline]
    fn byte(&self, ofs: usize) -> u8 {
        self.raw.get(ofs).copied().unwrap_or_else(|| {
            mpeghdec_fdk::deviation::hit();
            0
        })
    }

    /// `numGroupIdsRequested`
    pub fn num_group_ids_requested(&self) -> u8 {
        self.raw[0]
    }
    /// `groupIdRequested[i]`
    pub fn group_id_requested(&self, i: usize) -> u8 {
        self.byte(OFS_GROUP_ID + i)
    }
    /// `numGroupPresetIdsRequested`
    pub fn num_group_preset_ids_requested(&self) -> u8 {
        self.raw[OFS_NUM_PRESET]
    }
    /// `groupPresetIdRequested[i]`
    pub fn group_preset_id_requested(&self, i: usize) -> u8 {
        self.byte(OFS_PRESET_ID + i)
    }
    /// `numMembersGroupPresetIdsRequested[i]`
    pub fn num_members_group_preset_ids_requested(&self, i: usize) -> u8 {
        self.byte(OFS_NUM_MEMBERS + i)
    }
    /// `groupPresetIdRequestedPreference`
    pub fn group_preset_id_requested_preference(&self) -> i8 {
        self.raw[OFS_PREFERENCE] as i8
    }
}

/// `SEL_PROC_INPUT`
#[derive(StateDigest, Clone, Debug, Default, PartialEq, Eq)]
pub struct SelProcInput {
    /* system parameters */
    pub base_channel_count: i8,
    /// not supported
    pub base_layout: i8,
    pub target_config_request_type: i32,
    pub num_downmix_id_requests: u8,
    pub downmix_id_requested: [u8; MAX_REQUESTS_DOWNMIX_ID],
    pub target_layout_requested: u8,
    pub target_channel_count_requested: u8,
    /// needed for complexity estimation, currently not supported
    pub audio_sample_rate: i32,

    /* loudness normalization parameters */
    pub loudness_normalization_on: u8,
    /// e = 7
    pub target_loudness: FixpDbl,
    pub album_mode: u8,
    pub peak_limiter_present: u8,
    /// resolution: 1 dB
    pub loudness_deviation_max: u8,
    /// e = 7
    pub loudness_normalization_gain_db_max: FixpDbl,
    /// e = 7
    pub loudness_normalization_gain_modification_db: FixpDbl,
    /// e = 7
    pub output_peak_level_max: FixpDbl,

    /* dynamic range control parameters */
    pub dynamic_range_control_on: u8,
    pub num_drc_feature_requests: u8,
    pub drc_feature_request_type: [i32; MAX_REQUESTS_DRC_FEATURE],
    pub drc_feature_request: [DrcFeatureRequest; MAX_REQUESTS_DRC_FEATURE],

    /* MPEG-H parameters */
    pub mpegh_request: MpeghRequest,

    /* other */
    /// e = 1
    pub boost: FixpSgl,
    /// e = 1
    pub compress: FixpSgl,
    /// not supported
    pub drc_characteristic_target: u8,
}

/* Table E.1 of ISO/IEC DIS 23003-4: Recommended order of fallback effect type requests */
static FALLBACK_EFFECT_TYPE_REQUESTS: [[DrcEffectTypeRequest; 5]; 6] = [
    /* Night */ [DETR_GENERAL_COMPR, DETR_NOISY, DETR_LIMITED, DETR_LOWLEVEL, DETR_DIALOG],
    /* Noisy */ [DETR_GENERAL_COMPR, DETR_NIGHT, DETR_LIMITED, DETR_LOWLEVEL, DETR_DIALOG],
    /* Limited */ [DETR_GENERAL_COMPR, DETR_NIGHT, DETR_NOISY, DETR_LOWLEVEL, DETR_DIALOG],
    /* LowLevel */ [DETR_GENERAL_COMPR, DETR_NOISY, DETR_NIGHT, DETR_LIMITED, DETR_DIALOG],
    /* Dialog */ [DETR_GENERAL_COMPR, DETR_NIGHT, DETR_NOISY, DETR_LIMITED, DETR_LOWLEVEL],
    /* General */ [DETR_NIGHT, DETR_NOISY, DETR_LIMITED, DETR_LOWLEVEL, DETR_DIALOG],
];

/*******************************************/

/// `DRCDEC_SELECTION_DATA`. `pInst` is an index into `drcInstructionsUniDrc` of the
/// `UNI_DRC_CONFIG` passed to [`drc_dec_selection_process_process`].
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DrcDecSelectionData {
    pub selection_flag: u8,
    pub downmix_id_request_index: u8,
    /// e = 7
    pub output_peak_level: FixpDbl,
    /// e = 7
    pub loudness_normalization_gain_db_adjusted: FixpDbl,
    /// e = 7
    pub output_loudness: FixpDbl,
    pub p_inst: usize,
}

impl DrcDecSelectionData {
    const ZERO: Self = Self {
        selection_flag: 0,
        downmix_id_request_index: 0,
        output_peak_level: 0,
        loudness_normalization_gain_db_adjusted: 0,
        output_loudness: 0,
        p_inst: 0,
    };
}

/// `DRCDEC_SELECTION`
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct DrcDecSelection {
    pub num_data: u8,
    pub data: [DrcDecSelectionData; 32 + 1],
}

impl DrcDecSelection {
    const ZERO: Self = Self { num_data: 0, data: [DrcDecSelectionData::ZERO; 32 + 1] };
}

/// `struct s_drcdec_selection_process`
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct DrcSelectionProcess {
    pub codec_mode: SelProcCodecMode,
    pub sel_proc_input: SelProcInput,
    /// 2 instances, one before and one after selection
    pub selection_data: [DrcDecSelection; 2],
}

/*******************************************/
/* helper functions                        */
/*******************************************/

fn is_error(x: i32) -> bool {
    x < DRCDEC_SELECTION_PROCESS_WARNING
}

/// The two selection buffers, addressed by index (the C code swaps pointers to them).
type Sel = [DrcDecSelection; 2];

/// Mutable references to the potential (`p`) and selected (`s`) buffers (always distinct).
fn pot_sel(sel: &mut Sel, p: usize, s: usize) -> (&mut DrcDecSelection, &mut DrcDecSelection) {
    debug_assert_ne!(p, s);
    let (a, b) = sel.split_at_mut(1);
    if p == 0 {
        (&mut a[0], &mut b[0])
    } else {
        (&mut b[0], &mut a[0])
    }
}

/// `_drcdec_selection_addNew(pSelection)`
fn selection_add_new(p_selection: &mut DrcDecSelection) -> Option<&mut DrcDecSelectionData> {
    if (p_selection.num_data as usize) < (32 + 1) {
        let n = p_selection.num_data as usize;
        p_selection.data[n] = DrcDecSelectionData::ZERO;
        p_selection.num_data += 1;
        Some(&mut p_selection.data[n])
    } else {
        None
    }
}

/// `_drcdec_selection_add(pSelection, pDataIn)`: `false` corresponds to a `NULL` return.
fn selection_add(p_selection: &mut DrcDecSelection, p_data_in: &DrcDecSelectionData) -> bool {
    if (p_selection.num_data as usize) < (32 + 1) {
        let n = p_selection.num_data as usize;
        p_selection.data[n] = *p_data_in;
        p_selection.num_data += 1;
        true
    } else {
        false
    }
}

/// `_drcdec_selection_clear(pSelection)`
fn selection_clear(p_selection: &mut DrcDecSelection) {
    p_selection.num_data = 0;
}

/// `_drcdec_selection_getNumber(pSelection)`
fn selection_get_number(p_selection: &DrcDecSelection) -> i32 {
    p_selection.num_data as i32
}

/// `_drcdec_selection_setNumber(pSelection, num)`
fn selection_set_number(p_selection: &mut DrcDecSelection, num: i32) {
    if num >= 0 && num < p_selection.num_data as i32 {
        p_selection.num_data = num as u8;
    }
}

/// `_drcdec_selection_getAt(pSelection, at)`
fn selection_get_at(p_selection: &DrcDecSelection, at: i32) -> Option<&DrcDecSelectionData> {
    if at >= 0 && at < (32 + 1) {
        Some(&p_selection.data[at as usize])
    } else {
        None
    }
}

/// `_swapSelectionAndClear(ppCandidatesPotential, ppCandidatesSelected)`
fn swap_selection_and_clear(
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
) {
    std::mem::swap(pp_candidates_potential, pp_candidates_selected);
    selection_clear(&mut sel[*pp_candidates_selected]);
}

/// `_swapSelection(ppCandidatesPotential, ppCandidatesSelected)`
fn swap_selection(pp_candidates_potential: &mut usize, pp_candidates_selected: &mut usize) {
    std::mem::swap(pp_candidates_potential, pp_candidates_selected);
}

/*******************************************/
/* public functions                        */
/*******************************************/

/// `drcDec_SelectionProcess_Create(phInstance)`
pub fn drc_dec_selection_process_create() -> Box<DrcSelectionProcess> {
    Box::new(DrcSelectionProcess {
        codec_mode: SEL_PROC_CODEC_MODE_UNDEFINED,
        sel_proc_input: SelProcInput::default(),
        selection_data: [DrcDecSelection::ZERO, DrcDecSelection::ZERO],
    })
}

/// `drcDec_SelectionProcess_Init(hInstance)`
pub fn drc_dec_selection_process_init(
    h_instance: &mut DrcSelectionProcess,
) -> DrcDecSelectionProcessReturn {
    init_default_params(&mut h_instance.sel_proc_input);
    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/// `drcDec_SelectionProcess_SetCodecMode(hInstance, codecMode)`
pub fn drc_dec_selection_process_set_codec_mode(
    h_instance: &mut DrcSelectionProcess,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    match codec_mode {
        SEL_PROC_MPEG_H_3DA
        | SEL_PROC_TEST_TIME_DOMAIN
        | SEL_PROC_TEST_QMF_DOMAIN
        | SEL_PROC_TEST_STFT_DOMAIN => {
            h_instance.codec_mode = codec_mode;
        }
        _ => return DRCDEC_SELECTION_PROCESS_NOT_OK,
    }

    h_instance.codec_mode = codec_mode;
    init_codec_mode_params(&mut h_instance.sel_proc_input, codec_mode)
}

/// `drcDec_SelectionProcess_SetParam(hInstance, requestType, requestValue, pDiff)`
pub fn drc_dec_selection_process_set_param(
    h_instance: &mut DrcSelectionProcess,
    request_type: SelProcUserParam,
    mut request_value: FixpDbl,
    p_diff: Option<&mut i32>,
) -> DrcDecSelectionProcessReturn {
    let request_value_int: i32 = request_value;
    let request_value_ulong: u32 = request_value as u32;
    let mut diff = 0;
    let p = &mut h_instance.sel_proc_input;

    match request_type {
        SEL_PROC_LOUDNESS_NORMALIZATION_ON => {
            if (request_value_int != 0) && (request_value_int != 1) {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            diff |= comp_assign(&mut p.loudness_normalization_on, request_value_int as u8);
        }
        SEL_PROC_TARGET_LOUDNESS => {
            /* Lower boundary: drcSetTargetLoudnessValueLower default value.
            Upper boundary: drcSetTargetLoudnessValueUpper default value */
            if (request_value < fl2fxconst_dbl((-63.0f32 / 128.0f32) as f64)) || (request_value > 0)
            {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            if request_value > fl2fxconst_dbl((-10.0f32 / 128.0f32) as f64) {
                /* recommended maximum value */
                request_value = fl2fxconst_dbl((-10.0f32 / 128.0f32) as f64);
            }
            diff |= comp_assign(&mut p.target_loudness, request_value);
        }
        SEL_PROC_EFFECT_TYPE => {
            if (request_value_int < -1) || (request_value_int >= DETR_COUNT) {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            /* Caution. This overrides all drcFeatureRequests requested so far! */
            if request_value_int == -1 {
                diff |= comp_assign(&mut p.dynamic_range_control_on, 0);
            } else if request_value_int == DETR_NONE {
                diff |= comp_assign(&mut p.dynamic_range_control_on, 1);
                diff |= comp_assign(&mut p.num_drc_feature_requests, 0);
            } else {
                diff |= comp_assign(&mut p.dynamic_range_control_on, 1);
                diff |= comp_assign(&mut p.num_drc_feature_requests, 1);
                diff |= comp_assign(&mut p.drc_feature_request_type[0], DFRT_EFFECT_TYPE);
                let req = &mut p.drc_feature_request[0];
                diff |= comp_assign(&mut req.num_requests_desired, 1);
                diff |= comp_assign(&mut req.request[0], request_value_int);
                if (request_value_int > DETR_NONE) && (request_value_int <= DETR_GENERAL_COMPR) {
                    /* use fallback effect type requests */
                    for i in 0..5 {
                        diff |= comp_assign(
                            &mut req.request[i + 1],
                            FALLBACK_EFFECT_TYPE_REQUESTS[(request_value_int - 1) as usize][i],
                        );
                    }
                    diff |= comp_assign(&mut req.num_requests, 6);
                } else {
                    diff |= comp_assign(&mut req.num_requests, 1);
                }
            }
        }
        SEL_PROC_EFFECT_TYPE_FALLBACK_CODE => {
            /* Caution. This overrides all drcFeatureRequests requested so far! */
            if request_value_ulong == 0xFFFF_FFFF {
                /* DRC off */
                diff |= comp_assign(&mut p.dynamic_range_control_on, 0);
            } else if request_value_ulong == 0 {
                /* DETR_NONE without fallbacks */
                diff |= comp_assign(&mut p.dynamic_range_control_on, 1);
                diff |= comp_assign(&mut p.num_drc_feature_requests, 0);
            } else {
                let mut num_requests: u8 = 1;
                for i in 0..8 {
                    let fallback_effect_type = (request_value_ulong >> (4 * i)) & 0xF;
                    if fallback_effect_type >= DETR_COUNT as u32 {
                        return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
                    }
                }
                diff |= comp_assign(&mut p.dynamic_range_control_on, 1);
                diff |= comp_assign(&mut p.num_drc_feature_requests, 1);
                diff |= comp_assign(&mut p.drc_feature_request_type[0], DFRT_EFFECT_TYPE);
                let req = &mut p.drc_feature_request[0];
                diff |= comp_assign(&mut req.num_requests_desired, 1);
                /* selected DRC effect type is located at the 4 LSB */
                diff |= comp_assign(&mut req.request[0], (request_value_ulong & 0xF) as i32);
                /* decode fallback effect type requests */
                for i in 1..8 {
                    /* fallback DRC effect types are sorted from right to left, 4 bits each. */
                    let fallback_effect_type = (request_value_ulong >> (4 * i)) & 0xF;
                    if fallback_effect_type == 0 {
                        break;
                    }
                    diff |= comp_assign(&mut req.request[i], fallback_effect_type as i32);
                    num_requests += 1;
                }
                diff |= comp_assign(&mut req.num_requests, num_requests);
            }
        }
        SEL_PROC_ALBUM_MODE => {
            if (request_value_int < 0) || (request_value_int > 1) {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            diff |= comp_assign(&mut p.album_mode, request_value_int as u8);
        }
        SEL_PROC_DOWNMIX_ID => {
            diff |= comp_assign(&mut p.target_config_request_type, TCRT_DOWNMIX_ID);
            if request_value_int < 0 {
                /* negative requests signal no downmixId */
                diff |= comp_assign(&mut p.num_downmix_id_requests, 0);
            } else {
                diff |= comp_assign(&mut p.num_downmix_id_requests, 1);
                diff |= comp_assign(&mut p.downmix_id_requested[0], request_value_int as u8);
            }
        }
        SEL_PROC_TARGET_LAYOUT => {
            /* Request target layout according to ChannelConfiguration in ISO/IEC 23001-8 (CICP) */
            if (request_value_int < 1) || (request_value_int > 63) {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            diff |= comp_assign(&mut p.target_config_request_type, TCRT_TARGET_LAYOUT);
            diff |= comp_assign(&mut p.target_layout_requested, request_value_int as u8);
        }
        SEL_PROC_TARGET_CHANNEL_COUNT => {
            if request_value_int < 1 {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            diff |= comp_assign(&mut p.target_config_request_type, TCRT_TARGET_CHANNEL_COUNT);
            diff |= comp_assign(&mut p.target_channel_count_requested, request_value_int as u8);
        }
        SEL_PROC_BASE_CHANNEL_COUNT => {
            if request_value_int < 0 {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            diff |= comp_assign(&mut p.base_channel_count, request_value_int as i8);
        }
        SEL_PROC_SAMPLE_RATE => {
            if request_value_int < 0 {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            diff |= comp_assign(&mut p.audio_sample_rate, request_value_int);
        }
        SEL_PROC_BOOST => {
            if (request_value < 0) || (request_value > fl2fxconst_dbl((1.0f32 / 2.0f32) as f64)) {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            /* convert to FIXP_SGL with rounding */
            diff |= comp_assign(&mut p.boost, fx_dbl2fx_sgl(request_value + (1 << 15)));
        }
        SEL_PROC_COMPRESS => {
            if (request_value < 0) || (request_value > fl2fxconst_dbl((1.0f32 / 2.0f32) as f64)) {
                return DRCDEC_SELECTION_PROCESS_PARAM_OUT_OF_RANGE;
            }
            /* convert to FIXP_SGL with rounding */
            diff |= comp_assign(&mut p.compress, fx_dbl2fx_sgl(request_value + (1 << 15)));
        }
        _ => return DRCDEC_SELECTION_PROCESS_INVALID_PARAM,
    }

    if let Some(p_diff) = p_diff {
        *p_diff |= diff;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/// `drcDec_SelectionProcess_GetParam(hInstance, requestType)`
pub fn drc_dec_selection_process_get_param(
    h_instance: &DrcSelectionProcess,
    request_type: SelProcUserParam,
) -> FixpDbl {
    let p = &h_instance.sel_proc_input;
    match request_type {
        SEL_PROC_LOUDNESS_NORMALIZATION_ON => p.loudness_normalization_on as FixpDbl,
        SEL_PROC_DYNAMIC_RANGE_CONTROL_ON => p.dynamic_range_control_on as FixpDbl,
        _ => 0,
    }
}

/// `drcDec_SelectionProcess_SetMpeghParams(hInstance, numGroupIdsRequested, groupIdRequested,
/// numGroupPresetIdsRequested, groupPresetIdRequested, numMembersGroupPresetIdsRequested,
/// groupPresetIdRequestedPreference, pDiff)`
pub fn drc_dec_selection_process_set_mpegh_params(
    h_instance: &mut DrcSelectionProcess,
    num_group_ids_requested: i32,
    group_id_requested: &[i32],
    num_group_preset_ids_requested: i32,
    group_preset_id_requested: &[i32],
    num_members_group_preset_ids_requested: &[i32],
    group_preset_id_requested_preference: i32,
    p_diff: Option<&mut i32>,
) -> DrcDecSelectionProcessReturn {
    let p = &mut h_instance.sel_proc_input.mpegh_request.raw;
    let mut diff = 0;

    /* groupId request */
    diff |= comp_assign(&mut p[0], num_group_ids_requested as u8);
    if num_group_ids_requested > MAX_REQUESTS_GROUP_ID as i32 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }
    // (C writes out of bounds for negative request counts; the loops are limited here)
    if p[0] as usize > MAX_REQUESTS_GROUP_ID {
        mpeghdec_fdk::deviation::hit();
    }
    for i in 0..(p[0] as usize).min(MAX_REQUESTS_GROUP_ID) {
        diff |= comp_assign(&mut p[OFS_GROUP_ID + i], group_id_requested[i] as u8);
    }

    /* groupPresetId request */
    diff |= comp_assign(&mut p[OFS_NUM_PRESET], num_group_preset_ids_requested as u8);
    if num_group_preset_ids_requested > MAX_REQUESTS_GROUP_PRESET_ID as i32 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }
    if p[OFS_NUM_PRESET] as usize > MAX_REQUESTS_GROUP_PRESET_ID {
        mpeghdec_fdk::deviation::hit();
    }
    for i in 0..(p[OFS_NUM_PRESET] as usize).min(MAX_REQUESTS_GROUP_PRESET_ID) {
        diff |= comp_assign(&mut p[OFS_PRESET_ID + i], group_preset_id_requested[i] as u8);
        diff |= comp_assign(
            &mut p[OFS_NUM_MEMBERS + i],
            num_members_group_preset_ids_requested[i] as u8,
        );
    }
    diff |= comp_assign(&mut p[OFS_PREFERENCE], group_preset_id_requested_preference as i8 as u8);

    if let Some(p_diff) = p_diff {
        *p_diff |= diff;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/// `drcDec_SelectionProcess_Process(hInstance, hUniDrcConfig, hLoudnessInfoSet, hSelProcOutput)`
pub fn drc_dec_selection_process_process(
    h_instance: &mut DrcSelectionProcess,
    h_uni_drc_config: &mut UniDrcConfig,
    h_loudness_info_set: &LoudnessInfoSet,
    h_sel_proc_output: &mut SelProcOutput,
) -> DrcDecSelectionProcessReturn {
    let codec_mode = h_instance.codec_mode;
    let input = &mut h_instance.sel_proc_input;
    let sel = &mut h_instance.selection_data;
    let mut p_candidates_selected: usize = 0;
    let mut p_candidates_potential: usize = 1;
    selection_set_number(&mut sel[p_candidates_selected], 0);
    selection_set_number(&mut sel[p_candidates_potential], 0);

    let mut ret_val = generate_virtual_drc_sets(input, h_uni_drc_config, codec_mode);
    if ret_val != 0 {
        return ret_val;
    }
    let h_uni_drc_config: &UniDrcConfig = h_uni_drc_config;

    if input.base_channel_count as i32 != h_uni_drc_config.channel_layout.base_channel_count as i32
    {
        input.base_channel_count = h_uni_drc_config.channel_layout.base_channel_count as i8;
    }

    if (input.target_config_request_type != 0)
        || (input.target_config_request_type == 0 && input.num_downmix_id_requests == 0)
    {
        ret_val = channel_layout_to_downmix_id_mapping(input, h_uni_drc_config);

        if is_error(ret_val) {
            return ret_val;
        }
    }

    ret_val = drc_set_pre_selection(
        input,
        h_uni_drc_config,
        h_loudness_info_set,
        sel,
        &mut p_candidates_potential,
        &mut p_candidates_selected,
        codec_mode,
    );
    if ret_val != 0 {
        return ret_val;
    }

    if input.album_mode != 0 {
        swap_selection_and_clear(sel, &mut p_candidates_potential, &mut p_candidates_selected);

        let (pot, sl) = pot_sel(sel, p_candidates_potential, p_candidates_selected);
        ret_val = select_album_loudness(h_loudness_info_set, h_uni_drc_config, pot, sl);
        if ret_val != 0 {
            return ret_val;
        }

        if selection_get_number(&sel[p_candidates_selected]) == 0 {
            swap_selection(&mut p_candidates_potential, &mut p_candidates_selected);
        }
    }

    swap_selection_and_clear(sel, &mut p_candidates_potential, &mut p_candidates_selected);

    ret_val = drc_set_request_selection(
        input,
        h_uni_drc_config,
        sel,
        &mut p_candidates_potential,
        &mut p_candidates_selected,
    );
    if ret_val != 0 {
        return ret_val;
    }

    ret_val = drc_set_final_selection(
        input,
        sel,
        &mut p_candidates_potential,
        &mut p_candidates_selected,
        codec_mode,
        h_uni_drc_config,
    );
    if ret_val != 0 {
        return ret_val;
    }

    let selection_data = sel[p_candidates_selected].data[0];
    ret_val = generate_output_info(
        input,
        h_sel_proc_output,
        h_uni_drc_config,
        h_loudness_info_set,
        &selection_data,
        codec_mode,
    );
    if is_error(ret_val) {
        return ret_val;
    }

    ret_val = select_downmix_matrix(h_sel_proc_output, h_uni_drc_config);
    if ret_val != 0 {
        return ret_val;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/*******************************************/
/* static functions                        */
/*******************************************/

fn init_default_params(p: &mut SelProcInput) -> DrcDecSelectionProcessReturn {
    /* system parameters */
    p.base_channel_count = -1;
    p.base_layout = -1;
    p.target_config_request_type = TCRT_DOWNMIX_ID;
    p.num_downmix_id_requests = 0;

    /* loudness normalization parameters */
    p.album_mode = 0;
    p.peak_limiter_present = 0;
    p.loudness_normalization_on = 1;
    p.target_loudness = fl2fxconst_dbl((-24.0f32 / 128.0f32) as f64);
    p.loudness_deviation_max = DEFAULT_LOUDNESS_DEVIATION_MAX;
    p.loudness_normalization_gain_db_max = MAXVAL_DBL; /* infinity as default */
    p.loudness_normalization_gain_modification_db = 0;
    p.output_peak_level_max = 0;
    if p.peak_limiter_present == 1 {
        p.output_peak_level_max = fl2fxconst_dbl((6.0f32 / 128.0f32) as f64);
    }

    /* dynamic range control parameters */
    p.dynamic_range_control_on = 1;

    p.num_drc_feature_requests = 0;

    /* other parameters */
    p.boost = fl2fxconst_sgl((1.0f32 / 2.0f32) as f64);
    p.compress = fl2fxconst_sgl((1.0f32 / 2.0f32) as f64);
    p.drc_characteristic_target = 0;

    /* MPEG-H parameters */
    p.mpegh_request.raw[0] = 0; /* numGroupIdsRequested */
    p.mpegh_request.raw[OFS_NUM_PRESET] = 0; /* numGroupPresetIdsRequested */

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn init_codec_mode_params(
    p: &mut SelProcInput,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    match codec_mode {
        SEL_PROC_MPEG_H_3DA => {
            p.loudness_deviation_max = 0;
            p.peak_limiter_present = 1; /* peak limiter is mandatory */
            /* The peak limiter also has to catch overshoots due to user interactivity, downmixing
            etc. Therefore the maximum output peak level is reduced to 0 dB. */
            p.output_peak_level_max = 0;
        }
        SEL_PROC_TEST_TIME_DOMAIN | SEL_PROC_TEST_QMF_DOMAIN | SEL_PROC_TEST_STFT_DOMAIN => {
            /* for testing, adapt to default settings in reference software */
            p.loudness_normalization_on = 0;
            p.dynamic_range_control_on = 0;
        }
        _ => {
            p.loudness_deviation_max = DEFAULT_LOUDNESS_DEVIATION_MAX;
            p.peak_limiter_present = 0;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn channel_layout_to_downmix_id_mapping(
    p: &mut SelProcInput,
    cfg: &UniDrcConfig,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val = DRCDEC_SELECTION_PROCESS_NO_ERROR;

    p.num_downmix_id_requests = 0;

    match p.target_config_request_type {
        TCRT_DOWNMIX_ID => {
            p.downmix_id_requested[0] = 0;
            p.num_downmix_id_requests = 1;
        }

        TCRT_TARGET_LAYOUT => {
            if p.target_layout_requested as i32 == p.base_layout as i32 {
                p.downmix_id_requested[0] = 0;
                p.num_downmix_id_requests = 1;
            }

            if p.num_downmix_id_requests == 0 {
                for i in 0..cfg.downmix_instructions_count as usize {
                    let p_down = &cfg.downmix_instructions[i];

                    if p.target_layout_requested == p_down.target_layout {
                        // C writes without range check (MAX_REQUESTS_DOWNMIX_ID entries)
                        if let Some(v) =
                            p.downmix_id_requested.get_mut(p.num_downmix_id_requests as usize)
                        {
                            *v = p_down.downmix_id;
                        } else {
                            mpeghdec_fdk::deviation::hit();
                        }
                        p.num_downmix_id_requests += 1;
                    }
                }
            }

            if p.base_layout == -1 {
                ret_val = DRCDEC_SELECTION_PROCESS_WARNING;
            }

            if p.num_downmix_id_requests == 0 {
                p.downmix_id_requested[0] = 0;
                p.num_downmix_id_requests = 1;
                ret_val = DRCDEC_SELECTION_PROCESS_WARNING;
            }
        }

        TCRT_TARGET_CHANNEL_COUNT => {
            if p.target_channel_count_requested as i32 == p.base_channel_count as i32 {
                p.downmix_id_requested[0] = 0;
                p.num_downmix_id_requests = 1;
            }

            if p.num_downmix_id_requests == 0 {
                for i in 0..cfg.downmix_instructions_count as usize {
                    let p_down = &cfg.downmix_instructions[i];

                    if p.target_channel_count_requested == p_down.target_channel_count {
                        // C writes without range check (MAX_REQUESTS_DOWNMIX_ID entries)
                        if let Some(v) =
                            p.downmix_id_requested.get_mut(p.num_downmix_id_requests as usize)
                        {
                            *v = p_down.downmix_id;
                        } else {
                            mpeghdec_fdk::deviation::hit();
                        }
                        p.num_downmix_id_requests += 1;
                    }
                }
            }

            if p.base_channel_count == -1 {
                ret_val = DRCDEC_SELECTION_PROCESS_WARNING;
            }

            if p.num_downmix_id_requests == 0 {
                ret_val = DRCDEC_SELECTION_PROCESS_WARNING;
                p.downmix_id_requested[0] = 0;
                p.num_downmix_id_requests = 1;
            }
        }

        _ => return DRCDEC_SELECTION_PROCESS_NOT_OK,
    }

    ret_val
}

/*******************************************/

/* Note: Numbering of DRC pre-selection steps according to MPEG-D Part-4 DRC Amd1 */

/* #1: DownmixId of DRC set matches the requested downmixId.
#2: Output channel layout of DRC set matches the requested layout.
#3: Channel count of DRC set matches the requested channel count. */
fn pre_selection_requirement123(
    n_requested_downmix_id: i32,
    p_drc_instruction_uni_drc: &DrcInstructionsUniDrc,
    p_match_found: &mut i32,
) -> DrcDecSelectionProcessReturn {
    *p_match_found = 0;

    for i in 0..p_drc_instruction_uni_drc.downmix_id_count as usize {
        let d = p_drc_instruction_uni_drc.downmix_id[i.min(7)] as i32;
        if (d == n_requested_downmix_id)
            || (d == DOWNMIX_ID_ANY_DOWNMIX)
            || ((d == DOWNMIX_ID_BASE_LAYOUT) && (p_drc_instruction_uni_drc.drc_set_id > 0))
        {
            *p_match_found = 1;
            break;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/* #4: The DRC set is not a "Fade-" or "Ducking-" only DRC set. */
fn pre_selection_requirement4(
    p_drc_instruction: &DrcInstructionsUniDrc,
    n_dynamic_range_control_on: i32,
    p_match_found: &mut i32,
) -> DrcDecSelectionProcessReturn {
    *p_match_found = 0;

    let effect = p_drc_instruction.drc_set_effect as i32;
    if n_dynamic_range_control_on == 1 {
        if (effect != EB_FADE)
            && (effect != EB_DUCK_OTHER)
            && (effect != EB_DUCK_SELF)
            && (effect != 0 || p_drc_instruction.drc_set_id < 0)
        {
            *p_match_found = 1;
        }
    } else {
        *p_match_found = 1;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/* #5: The number of DRC bands is supported. Moreover, gainSetIndex and gainSequenceIndex are
 * within the allowed range. */
fn pre_selection_requirement5(
    p_drc_instruction_uni_drc: &DrcInstructionsUniDrc,
    p_coef: Option<&DrcCoefficientsUniDrc>,
    p_match_found: &mut i32,
) -> DrcDecSelectionProcessReturn {
    *p_match_found = 1;

    if p_drc_instruction_uni_drc.drc_set_id < 0 {
        /* virtual DRC sets are okay */
        return DRCDEC_SELECTION_PROCESS_NO_ERROR;
    }

    let Some(p_coef) = p_coef else {
        /* check for parametricDRC */
        *p_match_found = 0; /* parametricDRC not supported */
        return DRCDEC_SELECTION_PROCESS_NO_ERROR;
    };

    if p_coef.drc_location != p_drc_instruction_uni_drc.drc_location {
        /* drcLocation must be LOCATION_SELECTED */
        *p_match_found = 0;
        return DRCDEC_SELECTION_PROCESS_NO_ERROR;
    }

    for i in 0..p_drc_instruction_uni_drc.n_drc_channel_groups as usize {
        let index_drc_coeff =
            p_drc_instruction_uni_drc.gain_set_index_for_channel_group[i.min(27)] as i32;

        if index_drc_coeff >= 48 {
            *p_match_found = 0;
            return DRCDEC_SELECTION_PROCESS_NO_ERROR;
        }

        if index_drc_coeff > p_coef.gain_set_count as i32 - 1 {
            /* check for parametricDRC */
            continue;
        }

        let gain_set = gain_set_at(p_coef, index_drc_coeff);
        let band_count = gain_set.band_count as usize;

        if band_count > 4 {
            *p_match_found = 0;
        }

        for b in 0..band_count {
            let gsi = if b < 4 { gain_set.gain_sequence_index[b] } else { 0 };
            if (gsi >= 48) || (gsi >= p_coef.gain_sequence_count) {
                *p_match_found = 0;
                return DRCDEC_SELECTION_PROCESS_NO_ERROR;
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/* #6: Independent use of DRC set is permitted.*/
fn pre_selection_requirement6(
    p_drc_instruction_uni_drc: &DrcInstructionsUniDrc,
    p_match_found: &mut i32,
) -> DrcDecSelectionProcessReturn {
    *p_match_found = 0;

    if ((p_drc_instruction_uni_drc.depends_on_drc_set_present == 0)
        && (p_drc_instruction_uni_drc.no_independent_use() == 0))
        || (p_drc_instruction_uni_drc.depends_on_drc_set_present == 1)
    {
        *p_match_found = 1;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/* #7: DRC sets that require EQ are only permitted if EQ is supported. */
fn pre_selection_requirement7(
    p_drc_instruction_uni_drc: &DrcInstructionsUniDrc,
    p_match_found: &mut i32,
) -> DrcDecSelectionProcessReturn {
    *p_match_found = 1;

    if p_drc_instruction_uni_drc.requires_eq != 0 {
        /* EQ is not supported */
        *p_match_found = 0;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn set_selection_data_info(
    p_data: &mut DrcDecSelectionData,
    mut loudness: FixpDbl,                           /* e = 7 */
    mut loudness_normalization_gain_db: FixpDbl,     /* e = 7 */
    mut loudness_normalization_gain_db_max: FixpDbl, /* e = 7 */
    mut loudness_deviation_max: FixpDbl,             /* e = 7 */
    mut signal_peak_level: FixpDbl,                  /* e = 7 */
    mut output_peak_level_max: FixpDbl,              /* e = 7 */
    apply_adjustment: i32,
) {
    let mut adjustment: FixpDbl = 0; /* e = 8 */

    /* use e = 8 for all function parameters to prevent overflow */
    loudness >>= 1;
    loudness_normalization_gain_db >>= 1;
    loudness_normalization_gain_db_max >>= 1;
    loudness_deviation_max >>= 1;
    signal_peak_level >>= 1;
    output_peak_level_max >>= 1;

    if apply_adjustment != 0 {
        adjustment = f_max(
            0,
            signal_peak_level
                .wrapping_add(loudness_normalization_gain_db)
                .wrapping_sub(output_peak_level_max),
        );
        adjustment = f_min(adjustment, f_max(0, loudness_deviation_max));
    }

    p_data.loudness_normalization_gain_db_adjusted = f_min(
        loudness_normalization_gain_db.wrapping_sub(adjustment),
        loudness_normalization_gain_db_max,
    );
    p_data.output_loudness = loudness.wrapping_add(p_data.loudness_normalization_gain_db_adjusted);
    p_data.output_peak_level =
        signal_peak_level.wrapping_add(p_data.loudness_normalization_gain_db_adjusted);

    /* shift back to e = 7 using saturation */
    p_data.loudness_normalization_gain_db_adjusted =
        saturate_left_shift(p_data.loudness_normalization_gain_db_adjusted, 1, DFRACT_BITS);
    p_data.output_loudness = saturate_left_shift(p_data.output_loudness, 1, DFRACT_BITS);
    p_data.output_peak_level = saturate_left_shift(p_data.output_peak_level, 1, DFRACT_BITS);
}

fn target_loudness_in_range(
    p_drc_instruction_uni_drc: &DrcInstructionsUniDrc,
    target_loudness: FixpDbl,
) -> bool {
    let upper: FixpDbl = (p_drc_instruction_uni_drc.drc_set_target_loudness_value_upper as FixpDbl)
        .wrapping_shl((DFRACT_BITS - 1 - 7) as u32);
    let lower: FixpDbl = (p_drc_instruction_uni_drc.drc_set_target_loudness_value_lower as FixpDbl)
        .wrapping_shl((DFRACT_BITS - 1 - 7) as u32);

    p_drc_instruction_uni_drc.drc_set_target_loudness_present != 0
        && upper >= target_loudness
        && lower < target_loudness
}

fn drc_set_is_usable(h_uni_drc_config: &UniDrcConfig, inst_idx: usize) -> bool {
    let mut usable = 0;
    let p_coef = select_drc_coefficients(h_uni_drc_config, LOCATION_SELECTED)
        .map(|c| &h_uni_drc_config.drc_coefficients_uni_drc[c]);
    let p_inst = &h_uni_drc_config.drc_instructions_uni_drc[inst_idx];

    /* check if ID is unique */
    if select_drc_instructions(h_uni_drc_config, p_inst.drc_set_id as i32) != Some(inst_idx) {
        return false;
    }
    /* sanity check on drcInstructions */
    pre_selection_requirement5(p_inst, p_coef, &mut usable);
    usable != 0
}

/* #8: The range of the target loudness specified for a DRC set has to include the requested
 * decoder target loudness. */
fn pre_selection_requirement8(
    p: &SelProcInput,
    downmix_id_index: usize,
    h_uni_drc_config: &UniDrcConfig,
    h_loudness_info_set: &LoudnessInfoSet,
    inst_idx: usize,
    p_candidates_potential: &mut DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    let p_drc_instruction_uni_drc = &h_uni_drc_config.drc_instructions_uni_drc[inst_idx];
    let mut explicit_peak_information_present = 0;
    let mut signal_peak_level: FixpDbl = 0;
    let mut add_to_candidate = false;

    let mut loudness_normalization_gain_db: FixpDbl = 0;
    let mut loudness: FixpDbl = 0;

    let loudness_deviation_max: FixpDbl =
        (p.loudness_deviation_max as FixpDbl) << (DFRACT_BITS - 1 - 7);

    {
        let mut group_id_requested = -1;
        let mut group_preset_id_requested = -1;

        if p_drc_instruction_uni_drc.drc_instructions_type == 3 {
            group_preset_id_requested = p_drc_instruction_uni_drc.mae_group_preset_id() as i32;
        } else if p_drc_instruction_uni_drc.drc_instructions_type == 2 {
            group_id_requested = p_drc_instruction_uni_drc.mae_group_id as i32;
        }

        let ret_val = get_loudness(
            h_loudness_info_set,
            p.album_mode as i32,
            MDR_ANCHOR_LOUDNESS,
            MSR_EXPERT_PANEL,
            p.target_loudness,
            p_drc_instruction_uni_drc.drc_set_id as i32,
            p.downmix_id_requested[downmix_id_index] as i32,
            codec_mode,
            group_id_requested,
            group_preset_id_requested,
            &p.mpegh_request,
            &mut loudness_normalization_gain_db,
            &mut loudness,
        );
        if ret_val != 0 {
            return ret_val;
        }
    }

    if p.loudness_normalization_on == 0 {
        loudness_normalization_gain_db = 0;
    }

    let ret_val = get_signal_peak_level(
        p,
        h_loudness_info_set,
        p_drc_instruction_uni_drc,
        p.downmix_id_requested[downmix_id_index] as i32,
        &mut explicit_peak_information_present,
        &mut signal_peak_level,
        codec_mode,
    );
    if ret_val != 0 {
        return ret_val;
    }

    if p.dynamic_range_control_on != 0 {
        if explicit_peak_information_present == 0 {
            if p_drc_instruction_uni_drc.drc_set_target_loudness_present != 0
                && ((p.loudness_normalization_on != 0
                    && target_loudness_in_range(p_drc_instruction_uni_drc, p.target_loudness))
                    || p.loudness_normalization_on == 0)
            {
                let Some(p_data) = selection_add_new(p_candidates_selected) else {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                };

                set_selection_data_info(
                    p_data,
                    loudness,
                    loudness_normalization_gain_db,
                    p.loudness_normalization_gain_db_max,
                    loudness_deviation_max,
                    signal_peak_level,
                    p.output_peak_level_max,
                    0,
                );
                p_data.downmix_id_request_index = downmix_id_index as u8;
                p_data.p_inst = inst_idx;
                p_data.selection_flag = 1; /* signal pre-selection step dealing with drcSetTargetLoudness */

                if p.loudness_normalization_on != 0 {
                    p_data.output_peak_level = p.target_loudness.wrapping_sub(
                        (p_drc_instruction_uni_drc.drc_set_target_loudness_value_upper as FixpDbl)
                            .wrapping_shl((DFRACT_BITS - 1 - 7) as u32),
                    );
                } else {
                    p_data.output_peak_level = 0;
                }
            } else {
                if (p.loudness_normalization_on == 0)
                    || (p_drc_instruction_uni_drc.drc_set_target_loudness_present == 0)
                    || (p.loudness_normalization_on != 0
                        && target_loudness_in_range(p_drc_instruction_uni_drc, p.target_loudness))
                {
                    add_to_candidate = true;
                }
            }
        } else {
            add_to_candidate = true;
        }

        if add_to_candidate {
            let Some(p_data) = selection_add_new(p_candidates_potential) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };

            set_selection_data_info(
                p_data,
                loudness,
                loudness_normalization_gain_db,
                p.loudness_normalization_gain_db_max,
                loudness_deviation_max,
                signal_peak_level,
                p.output_peak_level_max,
                0,
            );
            p_data.downmix_id_request_index = downmix_id_index as u8;
            p_data.p_inst = inst_idx;
            p_data.selection_flag = 0;
        }
    } else {
        if p_drc_instruction_uni_drc.drc_set_id < 0 {
            let Some(p_data) = selection_add_new(p_candidates_selected) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };

            set_selection_data_info(
                p_data,
                loudness,
                loudness_normalization_gain_db,
                p.loudness_normalization_gain_db_max,
                loudness_deviation_max,
                signal_peak_level,
                p.output_peak_level_max,
                1,
            );

            p_data.downmix_id_request_index = downmix_id_index as u8;
            p_data.p_inst = inst_idx;
            p_data.selection_flag = 0;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/* #9: Clipping is minimized. */
fn pre_selection_requirement9(
    p: &SelProcInput,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        if p_candidate.output_peak_level <= p.output_peak_level_max
            && !selection_add(p_candidates_selected, p_candidate)
        {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn pre_selection_requirement10(
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
    mpegh_request: &MpeghRequest,
    p_omit_pre_selection_based_on_requested_group_id: &mut i32,
) -> DrcDecSelectionProcessReturn {
    *p_omit_pre_selection_based_on_requested_group_id = 0;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };
        let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

        if p_inst.drc_instructions_type == 0 || p_inst.drc_instructions_type == 2 {
            if !selection_add(p_candidates_selected, p_candidate) {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        } else if p_inst.drc_instructions_type == 3 {
            for j in 0..mpegh_request.num_group_preset_ids_requested() as usize {
                if p_inst.mae_group_preset_id() == mpegh_request.group_preset_id_requested(j) {
                    if !selection_add(p_candidates_selected, p_candidate) {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    }

                    *p_omit_pre_selection_based_on_requested_group_id = 1;
                }
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn pre_selection_requirement11(
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
    mpegh_request: &MpeghRequest,
    omit_pre_selection_based_on_requested_group_id: i32,
) -> DrcDecSelectionProcessReturn {
    if omit_pre_selection_based_on_requested_group_id == 1 {
        for i in 0..selection_get_number(p_candidates_potential) {
            let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };
            let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

            if (p_inst.drc_instructions_type == 0 || p_inst.drc_instructions_type == 3)
                && !selection_add(p_candidates_selected, p_candidate)
            {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }
    } else {
        for i in 0..selection_get_number(p_candidates_potential) {
            let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };
            let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

            if p_inst.drc_instructions_type == 0 {
                if !selection_add(p_candidates_selected, p_candidate) {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            } else if p_inst.drc_instructions_type == 2 {
                for j in 0..mpegh_request.num_group_ids_requested() as usize {
                    if p_inst.mae_group_id == mpegh_request.group_id_requested(j)
                        && !selection_add(p_candidates_selected, p_candidate)
                    {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    }
                }
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_pre_selection_single_instruction(
    p: &SelProcInput,
    downmix_id_index: usize,
    h_uni_drc_config: &UniDrcConfig,
    h_loudness_info_set: &LoudnessInfoSet,
    inst_idx: usize,
    p_candidates_potential: &mut DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val;
    let mut match_found = 0;
    let p_coef = select_drc_coefficients(h_uni_drc_config, LOCATION_SELECTED)
        .map(|c| &h_uni_drc_config.drc_coefficients_uni_drc[c]);
    let p_drc_instruction_uni_drc = &h_uni_drc_config.drc_instructions_uni_drc[inst_idx];

    ret_val = pre_selection_requirement123(
        p.downmix_id_requested[downmix_id_index] as i32,
        p_drc_instruction_uni_drc,
        &mut match_found,
    );

    if ret_val == 0 && match_found != 0 {
        ret_val = pre_selection_requirement4(
            p_drc_instruction_uni_drc,
            p.dynamic_range_control_on as i32,
            &mut match_found,
        );
    }

    if ret_val == 0 && match_found != 0 {
        ret_val = pre_selection_requirement5(p_drc_instruction_uni_drc, p_coef, &mut match_found);
    }

    if ret_val == 0 && match_found != 0 {
        ret_val = pre_selection_requirement6(p_drc_instruction_uni_drc, &mut match_found);
    }

    if ret_val == 0 && match_found != 0 {
        ret_val = pre_selection_requirement7(p_drc_instruction_uni_drc, &mut match_found);
    }

    if ret_val == 0 && match_found != 0 {
        ret_val = pre_selection_requirement8(
            p,
            downmix_id_index,
            h_uni_drc_config,
            h_loudness_info_set,
            inst_idx,
            p_candidates_potential,
            p_candidates_selected,
            codec_mode,
        );
    }

    ret_val
}

fn drc_set_selection_add_candidates(
    p: &SelProcInput,
    cfg: &UniDrcConfig,
    p_candidates_potential: &mut DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut n_hit_count = 0;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        if target_loudness_in_range(
            &cfg.drc_instructions_uni_drc[p_candidate.p_inst],
            p.target_loudness,
        ) {
            n_hit_count += 1;
        }
    }

    if n_hit_count != 0 {
        for i in 0..selection_get_number(p_candidates_potential) {
            let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };

            if target_loudness_in_range(
                &cfg.drc_instructions_uni_drc[p_candidate.p_inst],
                p.target_loudness,
            ) && !selection_add(p_candidates_selected, p_candidate)
            {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }
    } else {
        /* Initialize with very high value. e = 7 */
        let mut lowest_peak_level: FixpDbl = fl2fxconst_dbl((126.9f32 / 128.0f32) as f64);

        for i in 0..selection_get_number(p_candidates_potential) {
            let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };

            let peak_level = p_candidate.output_peak_level;
            let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

            if peak_level < lowest_peak_level {
                if p_inst.drc_instructions_type == 3 {
                    for j in 0..p.mpegh_request.num_group_preset_ids_requested() as usize {
                        if p_inst.mae_group_preset_id()
                            == p.mpegh_request.group_preset_id_requested(j)
                        {
                            lowest_peak_level = peak_level;
                            break;
                        }
                    }
                } else if p_inst.drc_instructions_type == 2 {
                    for j in 0..p.mpegh_request.num_group_ids_requested() as usize {
                        if p_inst.mae_group_id == p.mpegh_request.group_id_requested(j) {
                            lowest_peak_level = peak_level;
                            break;
                        }
                    }
                } else {
                    lowest_peak_level = peak_level;
                }
            }
        }

        /* add all with lowest peak level or max 1dB above */
        for i in 0..selection_get_number(p_candidates_potential) {
            let loudness_deviation_max: FixpDbl =
                (p.loudness_deviation_max as FixpDbl) << (DFRACT_BITS - 1 - 7); /* e = 7 */

            if !(0..(32 + 1)).contains(&i) {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
            let p_candidate = &mut p_candidates_potential.data[i as usize];

            let peak_level = p_candidate.output_peak_level;

            if peak_level == lowest_peak_level
                || peak_level
                    <= lowest_peak_level.wrapping_add(fl2fxconst_dbl((1.0f32 / 128.0f32) as f64))
            {
                let mut adjustment = f_max(0, peak_level.wrapping_sub(p.output_peak_level_max));
                adjustment = f_min(adjustment, f_max(0, loudness_deviation_max));

                p_candidate.loudness_normalization_gain_db_adjusted =
                    p_candidate.loudness_normalization_gain_db_adjusted.wrapping_sub(adjustment);
                p_candidate.output_peak_level =
                    p_candidate.output_peak_level.wrapping_sub(adjustment);
                p_candidate.output_loudness = p_candidate.output_loudness.wrapping_sub(adjustment);
                let c = *p_candidate;
                if !selection_add(p_candidates_selected, &c) {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn dependent_drc_instruction(
    h_uni_drc_config: &UniDrcConfig,
    p_inst: &DrcInstructionsUniDrc,
    pp_drc_instructions_dependent: &mut usize,
) -> DrcDecSelectionProcessReturn {
    let count = h_uni_drc_config.drc_instructions_uni_drc_count as usize;
    let mut i = 0usize;
    while i < count {
        if h_uni_drc_config.drc_instructions_uni_drc[i].drc_set_id == p_inst.depends_on_drc_set {
            break;
        }
        i += 1;
    }

    if i == count {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    if h_uni_drc_config.drc_instructions_uni_drc[i].depends_on_drc_set_present == 1 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    *pp_drc_instructions_dependent = i;

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn select_drc_set_effect_none(
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        if (cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_effect & 0xff) == 0
            && !selection_add(p_candidates_selected, p_candidate)
        {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn select_single_effect_type(
    h_uni_drc_config: &UniDrcConfig,
    effect_type: DrcEffectTypeRequest,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val = DRCDEC_SELECTION_PROCESS_NO_ERROR;

    if effect_type == DETR_NONE {
        ret_val = select_drc_set_effect_none(
            h_uni_drc_config,
            p_candidates_potential,
            p_candidates_selected,
        );
        if ret_val != 0 {
            return ret_val;
        }
    } else {
        let effect_bit_position: i32 = 1i32.wrapping_shl((effect_type - 1) as u32);

        for i in 0..selection_get_number(p_candidates_potential) {
            let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };

            let p_inst = &h_uni_drc_config.drc_instructions_uni_drc[p_candidate.p_inst];

            if p_inst.depends_on_drc_set_present == 0 {
                if (p_inst.drc_set_effect as i32 & effect_bit_position) != 0
                    && !selection_add(p_candidates_selected, p_candidate)
                {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            } else {
                let mut dep = 0usize;
                ret_val = dependent_drc_instruction(h_uni_drc_config, p_inst, &mut dep);
                if ret_val != 0 {
                    return ret_val;
                }
                let p_drc_instructions_dependent = &h_uni_drc_config.drc_instructions_uni_drc[dep];

                if ((p_inst.drc_set_effect as i32 & effect_bit_position) != 0
                    || (p_drc_instructions_dependent.drc_set_effect as i32 & effect_bit_position)
                        != 0)
                    && !selection_add(p_candidates_selected, p_candidate)
                {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }
        }
    }

    ret_val
}

fn select_effect_type_feature(
    h_uni_drc_config: &UniDrcConfig,
    drc_feature_request: DrcFeatureRequest,
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
) -> DrcDecSelectionProcessReturn {
    let mut desired_effect_type_found = false;

    for i in 0..drc_feature_request.num_requests_desired as usize {
        let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
        let ret_val = select_single_effect_type(
            h_uni_drc_config,
            drc_feature_request.request[i.min(MAX_REQUESTS_DRC_EFFECT_TYPE - 1)],
            pot,
            sl,
        );
        if ret_val != 0 {
            return ret_val;
        }

        if selection_get_number(&sel[*pp_candidates_selected]) != 0 {
            desired_effect_type_found = true;
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
        }
    }

    if !desired_effect_type_found {
        for i in drc_feature_request.num_requests_desired as usize
            ..drc_feature_request.num_requests as usize
        {
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            let ret_val = select_single_effect_type(
                h_uni_drc_config,
                drc_feature_request.request[i.min(MAX_REQUESTS_DRC_EFFECT_TYPE - 1)],
                pot,
                sl,
            );
            if ret_val != 0 {
                return ret_val;
            }

            if selection_get_number(&sel[*pp_candidates_selected]) != 0 {
                swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
                break;
            }
        }
    }

    swap_selection(pp_candidates_potential, pp_candidates_selected);

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_final_selection_peak_value0(
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut lowest_peak_level: FixpDbl = MAXVAL_DBL;
    let mut index_lowest_peak_level = 0;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        if p_candidate.output_peak_level <= 0 && !selection_add(p_candidates_selected, p_candidate)
        {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }

        if p_candidate.output_peak_level < lowest_peak_level {
            index_lowest_peak_level = i;
            lowest_peak_level = p_candidate.output_peak_level;
        }
    }

    if selection_get_number(p_candidates_selected) == 0 {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, index_lowest_peak_level)
        else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };
        if !selection_add(p_candidates_selected, p_candidate) {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_final_selection_preset_id_group_id(
    p: &SelProcInput,
    cfg: &UniDrcConfig,
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
) -> DrcDecSelectionProcessReturn {
    let mut num_members_group_preset_id: i32 = 0;
    let mut max_num_members_group_preset_ids: i32 = 0;
    let mut mae_group_preset_id_lowest: i32 = 1000;
    let mut instructions_type3_found = false;

    /* final MPEG-H selection based on groupPresetId and groupId */

    for i in 0..selection_get_number(&sel[*pp_candidates_potential]) {
        let Some(p_candidate) = selection_get_at(&sel[*pp_candidates_potential], i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };
        if cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_instructions_type == 3 {
            instructions_type3_found = true;
        }
    }

    if instructions_type3_found {
        {
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            for i in 0..selection_get_number(pot) {
                let Some(p_candidate) = selection_get_at(pot, i) else {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                };
                let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

                if p_inst.drc_instructions_type == 3
                    && p.mpegh_request.group_preset_id_requested_preference() as i32
                        == p_inst.mae_group_preset_id() as i32
                {
                    if !selection_add(sl, p_candidate) {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    }
                    break;
                }
            }

            if selection_get_number(sl) == 0 {
                for i in 0..selection_get_number(pot) {
                    let Some(p_candidate) = selection_get_at(pot, i) else {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    };
                    let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

                    if p_inst.drc_instructions_type == 3 {
                        for j in 0..p.mpegh_request.num_group_preset_ids_requested() as usize {
                            if p.mpegh_request.group_preset_id_requested(j)
                                == p_inst.mae_group_preset_id()
                            {
                                num_members_group_preset_id =
                                    p.mpegh_request.num_members_group_preset_ids_requested(j)
                                        as i32;
                                break;
                            }
                        }

                        if max_num_members_group_preset_ids <= num_members_group_preset_id {
                            if max_num_members_group_preset_ids < num_members_group_preset_id {
                                max_num_members_group_preset_ids = num_members_group_preset_id;

                                selection_set_number(sl, 0);
                            }
                            if !selection_add(sl, p_candidate) {
                                return DRCDEC_SELECTION_PROCESS_NOT_OK;
                            }
                        }
                    }
                }
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);

            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            for i in 0..selection_get_number(pot) {
                let Some(p_candidate) = selection_get_at(pot, i) else {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                };
                let id =
                    cfg.drc_instructions_uni_drc[p_candidate.p_inst].mae_group_preset_id() as i32;

                if mae_group_preset_id_lowest >= id {
                    if mae_group_preset_id_lowest > id {
                        mae_group_preset_id_lowest = id;

                        selection_set_number(sl, 0);
                    }
                    if !selection_add(sl, p_candidate) {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    }
                }
            }
        }
    } else {
        {
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            for i in 0..selection_get_number(pot) {
                let Some(p_candidate) = selection_get_at(pot, i) else {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                };

                if cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_instructions_type == 2
                    && !selection_add(sl, p_candidate)
                {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) == 0 {
            swap_selection(pp_candidates_potential, pp_candidates_selected);
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_final_selection_downmix_id(
    p: &SelProcInput,
    cfg: &UniDrcConfig,
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
) -> DrcDecSelectionProcessReturn {
    {
        let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
        for i in 0..selection_get_number(pot) {
            let Some(p_candidate) = selection_get_at(pot, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };
            let p_inst = &cfg.drc_instructions_uni_drc[p_candidate.p_inst];

            for j in 0..p_inst.downmix_id_count as usize {
                let d = p_inst.downmix_id[j.min(7)] as i32;
                let req = p
                    .downmix_id_requested
                    .get(p_candidate.downmix_id_request_index as usize)
                    .copied()
                    .unwrap_or(0) as i32;
                if DOWNMIX_ID_BASE_LAYOUT != d
                    && DOWNMIX_ID_ANY_DOWNMIX != d
                    && req == d
                    && !selection_add(sl, p_candidate)
                {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }
        }
    }

    if selection_get_number(&sel[*pp_candidates_selected]) == 0 {
        swap_selection(pp_candidates_potential, pp_candidates_selected);
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn cross_sum(mut value: i32) -> i32 {
    let mut sum = 0;

    while value != 0 {
        if (value & 1) == 1 {
            sum += 1;
        }

        value >>= 1;
    }

    sum
}

fn drc_set_final_selection_effect_types(
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut min_num_effects = 1000;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        let mut effects = cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_effect as i32;
        effects &= 0xffff ^ (EB_GENERAL_COMPR);
        let num_effects = cross_sum(effects);

        if num_effects < min_num_effects {
            min_num_effects = num_effects;
        }
    }

    /* add all with minimum number of effects */
    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        let mut effects = cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_effect as i32;
        effects &= 0xffff ^ (EB_GENERAL_COMPR);
        let num_effects = cross_sum(effects);

        if num_effects == min_num_effects && !selection_add(p_candidates_selected, p_candidate) {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn select_smallest_target_loudness_value_upper(
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut min_val: i8 = 0x7F;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        let val =
            cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_target_loudness_value_upper;

        if val < min_val {
            min_val = val;
        }
    }

    /* add all with same smallest drcSetTargetLoudnessValueUpper */
    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        let val =
            cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_target_loudness_value_upper;

        if val == min_val && !selection_add(p_candidates_selected, p_candidate) {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/// Note: the C function swaps its local copies of the selection pointers; the caller does not
/// see these swaps. The indices are therefore passed by value.
fn drc_set_final_selection_target_loudness(
    cfg: &UniDrcConfig,
    target_loudness: FixpDbl,
    sel: &mut Sel,
    mut p_candidates_potential: usize,
    mut p_candidates_selected: usize,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val;

    {
        let (pot, sl) = pot_sel(sel, p_candidates_potential, p_candidates_selected);
        for i in 0..selection_get_number(pot) {
            let Some(p_candidate) = selection_get_at(pot, i) else {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            };

            if p_candidate.selection_flag == 0 && !selection_add(sl, p_candidate) {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }

        if selection_get_number(sl) == 0 {
            ret_val = select_smallest_target_loudness_value_upper(cfg, pot, sl);
            if ret_val != 0 {
                return ret_val;
            }
        }
    }

    ret_val = DRCDEC_SELECTION_PROCESS_NO_ERROR;
    if selection_get_number(&sel[p_candidates_selected]) > 1 {
        swap_selection_and_clear(sel, &mut p_candidates_potential, &mut p_candidates_selected);

        {
            let (pot, sl) = pot_sel(sel, p_candidates_potential, p_candidates_selected);
            for i in 0..selection_get_number(pot) {
                let Some(p_candidate) = selection_get_at(pot, i) else {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                };

                if target_loudness_in_range(
                    &cfg.drc_instructions_uni_drc[p_candidate.p_inst],
                    target_loudness,
                ) && !selection_add(sl, p_candidate)
                {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }
        }

        if selection_get_number(&sel[p_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, &mut p_candidates_potential, &mut p_candidates_selected);

            let (pot, sl) = pot_sel(sel, p_candidates_potential, p_candidates_selected);
            ret_val = select_smallest_target_loudness_value_upper(cfg, pot, sl);
            if ret_val != 0 {
                return ret_val;
            }
        }
    }

    ret_val
}

fn drc_set_final_selection_peak_value_largest(
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut largest_peak_level: FixpDbl = MINVAL_DBL;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        let peak_level = p_candidate.output_peak_level;

        if peak_level > largest_peak_level {
            largest_peak_level = peak_level;
        }
    }

    /* add all with same largest peak level */
    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        if p_candidate.output_peak_level == largest_peak_level
            && !selection_add(p_candidates_selected, p_candidate)
        {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_final_selection_drc_set_id(
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut largest_id = -1000;
    let mut p_candidate_selected: Option<&DrcDecSelectionData> = None;

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };

        let id = cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_id as i32;

        if id > largest_id {
            largest_id = id;
            p_candidate_selected = Some(p_candidate);
        }
    }

    if let Some(c) = p_candidate_selected {
        if !selection_add(p_candidates_selected, c) {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    } else {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_final_selection(
    p: &SelProcInput,
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
    codec_mode: SelProcCodecMode,
    cfg: &UniDrcConfig,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val = DRCDEC_SELECTION_PROCESS_NO_ERROR;

    if selection_get_number(&sel[*pp_candidates_potential]) == 0 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    } else if selection_get_number(&sel[*pp_candidates_potential]) == 1 {
        swap_selection(pp_candidates_potential, pp_candidates_selected);
        /* finished */
    } else {
        /* > 1 */
        {
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            ret_val = drc_set_final_selection_peak_value0(pot, sl);
            if ret_val != 0 {
                return ret_val;
            }
        }

        if codec_mode == SEL_PROC_MPEG_H_3DA
            && selection_get_number(&sel[*pp_candidates_selected]) > 1
        {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);

            ret_val = drc_set_final_selection_preset_id_group_id(
                p,
                cfg,
                sel,
                pp_candidates_potential,
                pp_candidates_selected,
            );
            if ret_val != 0 {
                return ret_val;
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
            ret_val = drc_set_final_selection_downmix_id(
                p,
                cfg,
                sel,
                pp_candidates_potential,
                pp_candidates_selected,
            );
            if ret_val != 0 {
                return ret_val;
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            ret_val = drc_set_final_selection_effect_types(cfg, pot, sl);
            if ret_val != 0 {
                return ret_val;
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
            ret_val = drc_set_final_selection_target_loudness(
                cfg,
                p.target_loudness,
                sel,
                *pp_candidates_potential,
                *pp_candidates_selected,
            );
            if ret_val != 0 {
                return ret_val;
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            ret_val = drc_set_final_selection_peak_value_largest(pot, sl);
            if ret_val != 0 {
                return ret_val;
            }
        }

        if selection_get_number(&sel[*pp_candidates_selected]) > 1 {
            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            ret_val = drc_set_final_selection_drc_set_id(cfg, pot, sl);
            if ret_val != 0 {
                return ret_val;
            }
        }
    }

    if selection_get_number(&sel[*pp_candidates_selected]) == 0 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    ret_val
}

fn generate_virtual_drc_sets(
    p: &SelProcInput,
    h_uni_drc_config: &mut UniDrcConfig,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    let mut n_mixes = h_uni_drc_config.downmix_instructions_count as usize + 1;
    let mut index = h_uni_drc_config.drc_instructions_uni_drc_count as usize;
    let mut index_virtual: i32 = -1;

    if codec_mode == SEL_PROC_MPEG_H_3DA {
        n_mixes = 1;
    }

    if (index + n_mixes) > (32 + 1) {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    let p_drc_instruction = &mut h_uni_drc_config.drc_instructions_uni_drc[index];
    *p_drc_instruction = DrcInstructionsUniDrc::ZERO;

    p_drc_instruction.drc_set_id = index_virtual as i8;
    index += 1;
    index_virtual -= 1;
    p_drc_instruction.downmix_id_count = 1;

    if (codec_mode == SEL_PROC_MPEG_H_3DA) && (p.num_downmix_id_requests != 0) {
        p_drc_instruction.downmix_id[0] = p.downmix_id_requested[0];
    } else {
        p_drc_instruction.downmix_id[0] = DOWNMIX_ID_BASE_LAYOUT as u8;
    }

    for i in 1..n_mixes {
        let downmix_id = h_uni_drc_config.downmix_instructions[i - 1].downmix_id;
        let p_drc_instruction = &mut h_uni_drc_config.drc_instructions_uni_drc[index];
        *p_drc_instruction = DrcInstructionsUniDrc::ZERO;
        p_drc_instruction.drc_set_id = index_virtual as i8;
        p_drc_instruction.downmix_id[0] = downmix_id;
        p_drc_instruction.downmix_id_count = 1;
        index += 1;
        index_virtual -= 1;
    }

    h_uni_drc_config.drc_instructions_count_incl_virtual =
        h_uni_drc_config.drc_instructions_uni_drc_count.wrapping_add(n_mixes as u8);

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn generate_output_info(
    p: &SelProcInput,
    out: &mut SelProcOutput,
    cfg: &UniDrcConfig,
    h_loudness_info_set: &LoudnessInfoSet,
    p_selection_data: &DrcDecSelectionData,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val = DRCDEC_SELECTION_PROCESS_NO_ERROR;

    let mut has_dependend: u8 = 0;
    let mut has_fading: u8 = 0;
    let mut has_ducking = false;
    let mut mixing_level: FixpDbl = 0;
    let album_mode = p.album_mode as i32;
    let p_downmix_id_requested = &p.downmix_id_requested;
    let boost = p.boost;
    let compress = p.compress;
    let sel_inst = &cfg.drc_instructions_uni_drc[p_selection_data.p_inst];

    out.num_selected_drc_sets = 1;
    out.selected_drc_set_ids[0] = sel_inst.drc_set_id;
    out.selected_downmix_ids[0] =
        if sel_inst.drc_apply_to_downmix == 1 { sel_inst.downmix_id[0] } else { 0 };
    out.loudness_normalization_gain_db = p_selection_data
        .loudness_normalization_gain_db_adjusted
        .wrapping_add(p.loudness_normalization_gain_modification_db);
    out.output_peak_level_db = p_selection_data.output_peak_level;
    out.output_loudness = p_selection_data.output_loudness;

    out.boost = boost;
    out.compress = compress;
    out.base_channel_count = cfg.channel_layout.base_channel_count;
    out.target_channel_count = cfg.channel_layout.base_channel_count;
    out.active_downmix_id = p_downmix_id_requested
        .get(p_selection_data.downmix_id_request_index as usize)
        .copied()
        .unwrap_or(0);

    get_mixing_level(
        h_loudness_info_set,
        p_downmix_id_requested[0] as i32,
        out.selected_drc_set_ids[0] as i32,
        album_mode,
        &mut mixing_level,
    );
    out.mixing_level = mixing_level;

    get_group_loudness(h_loudness_info_set, out);

    /*dependent*/
    if sel_inst.depends_on_drc_set_present != 0 {
        let depends_on_drc_set_id = sel_inst.depends_on_drc_set as i32;

        for i in 0..cfg.drc_instructions_count_incl_virtual as usize {
            if !drc_set_is_usable(cfg, i) {
                continue;
            }
            let p_inst = &cfg.drc_instructions_uni_drc[i];

            if p_inst.drc_set_id as i32 == depends_on_drc_set_id {
                let n = out.num_selected_drc_sets as usize;
                out.selected_drc_set_ids[n] = p_inst.drc_set_id;
                out.selected_downmix_ids[n] =
                    if p_inst.drc_apply_to_downmix == 1 { p_inst.downmix_id[0] } else { 0 };
                out.num_selected_drc_sets += 1;
                has_dependend = 1;
                break;
            }
        }
    }

    /* fading */
    if p.album_mode == 0 {
        for i in 0..cfg.drc_instructions_uni_drc_count as usize {
            if !drc_set_is_usable(cfg, i) {
                continue;
            }
            let p_inst = &cfg.drc_instructions_uni_drc[i];

            if p_inst.drc_set_effect as i32 & EB_FADE != 0 {
                if p_inst.downmix_id[0] as i32 == DOWNMIX_ID_ANY_DOWNMIX {
                    let mut add = false;
                    if p_inst.drc_instructions_type == 0 {
                        add = true;
                    } else if p_inst.drc_instructions_type == 2 {
                        for m in 0..p.mpegh_request.num_group_ids_requested() as usize {
                            if p.mpegh_request.group_id_requested(m) == p_inst.mae_group_id {
                                add = true;
                                break;
                            }
                        }
                    } else if p_inst.drc_instructions_type == 3 {
                        for m in 0..p.mpegh_request.num_group_preset_ids_requested() as usize {
                            if p.mpegh_request.group_preset_id_requested(m)
                                == p_inst.mae_group_preset_id()
                            {
                                add = true;
                                break;
                            }
                        }
                    } else {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    }
                    if add {
                        has_fading = 1;
                        out.num_selected_drc_sets = has_dependend + 1;
                        let n = out.num_selected_drc_sets as usize;
                        out.selected_drc_set_ids[n] = p_inst.drc_set_id;
                        out.selected_downmix_ids[n] =
                            if p_inst.drc_apply_to_downmix == 1 { p_inst.downmix_id[0] } else { 0 };
                        out.num_selected_drc_sets += 1;
                    }
                } else {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }
        }
    }

    /* ducking */
    for i in 0..cfg.drc_instructions_uni_drc_count as usize {
        if !drc_set_is_usable(cfg, i) {
            continue;
        }
        let p_inst = &cfg.drc_instructions_uni_drc[i];

        if p_inst.drc_set_effect as i32 & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0 {
            for j in 0..p_inst.downmix_id_count as usize {
                if p_inst.downmix_id[j.min(7)] == out.active_downmix_id {
                    out.num_selected_drc_sets = has_dependend + 1; /* ducking overrides fading */

                    let n = out.num_selected_drc_sets as usize;
                    out.selected_drc_set_ids[n] = p_inst.drc_set_id;
                    /* force ducking DRC set to be processed on base layout */
                    out.selected_downmix_ids[n] = 0;
                    out.num_selected_drc_sets += 1;
                    has_ducking = true;
                }
            }
        }
    }

    /* repeat for DOWNMIX_ID_BASE_LAYOUT if no ducking found*/

    if codec_mode == SEL_PROC_MPEG_H_3DA && has_ducking {
        /* remove ducking */
        has_ducking = false;
        out.num_selected_drc_sets = has_dependend + has_fading + 1;
    }

    if !has_ducking {
        for i in 0..cfg.drc_instructions_uni_drc_count as usize {
            if !drc_set_is_usable(cfg, i) {
                continue;
            }
            let p_inst = &cfg.drc_instructions_uni_drc[i];

            if p_inst.drc_set_effect as i32 & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0 {
                for j in 0..p_inst.downmix_id_count as usize {
                    if p_inst.downmix_id[j.min(7)] as i32 == DOWNMIX_ID_BASE_LAYOUT {
                        has_ducking = false;

                        if p_inst.drc_instructions_type == 0 {
                            has_ducking = true;
                        } else if p_inst.drc_instructions_type == 2 {
                            for m in 0..p.mpegh_request.num_group_ids_requested() as usize {
                                if p.mpegh_request.group_id_requested(m) == p_inst.mae_group_id {
                                    has_ducking = true;
                                    break;
                                }
                            }
                        } else if p_inst.drc_instructions_type == 3 {
                            for m in 0..p.mpegh_request.num_group_preset_ids_requested() as usize {
                                if p.mpegh_request.group_preset_id_requested(m)
                                    == p_inst.mae_group_preset_id()
                                {
                                    has_ducking = true;
                                    break;
                                }
                            }
                        } else {
                            ret_val = DRCDEC_SELECTION_PROCESS_WARNING;
                        }

                        if has_ducking {
                            out.num_selected_drc_sets = has_dependend + 1; /* ducking overrides fading */
                            let n = out.num_selected_drc_sets as usize;
                            out.selected_drc_set_ids[n] = p_inst.drc_set_id;
                            /* force ducking DRC set to be processed on base layout */
                            out.selected_downmix_ids[n] = 0;
                            out.num_selected_drc_sets += 1;
                        }
                    }
                }
            }
        }
    }

    if out.num_selected_drc_sets > 3 {
        /* maximum permitted number of applied DRC sets is 3, see section 6.3.5 of
         * ISO/IEC 23003-4 */
        out.num_selected_drc_sets = 0;
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    /* sorting: Ducking/Fading -> Dependent -> Selected */
    if out.num_selected_drc_sets == 3 {
        out.selected_drc_set_ids.swap(0, 2);
        out.selected_downmix_ids.swap(0, 2);
    } else if out.num_selected_drc_sets == 2 {
        out.selected_drc_set_ids.swap(0, 1);
        out.selected_downmix_ids.swap(0, 1);
    }

    ret_val
}

fn select_downmix_matrix(
    out: &mut SelProcOutput,
    cfg: &UniDrcConfig,
) -> DrcDecSelectionProcessReturn {
    out.base_channel_count = cfg.channel_layout.base_channel_count;
    out.target_channel_count = cfg.channel_layout.base_channel_count;
    out.target_layout = -1;
    out.downmix_matrix_present = 0;

    if out.active_downmix_id != 0 {
        for i in 0..cfg.downmix_instructions_count as usize {
            let p_down = &cfg.downmix_instructions[i.min(31)];
            if p_down.target_channel_count > 28 {
                continue;
            }

            if out.active_downmix_id == p_down.downmix_id {
                out.target_channel_count = p_down.target_channel_count;
                out.target_layout = p_down.target_layout as i16;

                break;
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

fn drc_set_pre_selection(
    p: &SelProcInput,
    h_uni_drc_config: &UniDrcConfig,
    h_loudness_info_set: &LoudnessInfoSet,
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    let mut ret_val;

    for i in 0..p.num_downmix_id_requests as usize {
        for j in 0..h_uni_drc_config.drc_instructions_count_incl_virtual as usize {
            /* check if ID is unique */
            if select_drc_instructions(
                h_uni_drc_config,
                h_uni_drc_config.drc_instructions_uni_drc[j].drc_set_id as i32,
            ) != Some(j)
            {
                continue;
            }

            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            ret_val = drc_set_pre_selection_single_instruction(
                p,
                i.min(MAX_REQUESTS_DOWNMIX_ID - 1),
                h_uni_drc_config,
                h_loudness_info_set,
                j,
                pot,
                sl,
                codec_mode,
            );
            if ret_val != 0 {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }
    }

    {
        let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
        ret_val = pre_selection_requirement9(p, pot, sl);
        if ret_val != 0 {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }

        if selection_get_number(sl) == 0 {
            ret_val = drc_set_selection_add_candidates(p, h_uni_drc_config, pot, sl);
            if ret_val != 0 {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }
    }

    if codec_mode == SEL_PROC_MPEG_H_3DA {
        let mut omit_pre_selection_based_on_requested_group_id = 0;
        swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
        {
            let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
            ret_val = pre_selection_requirement10(
                h_uni_drc_config,
                pot,
                sl,
                &p.mpegh_request,
                &mut omit_pre_selection_based_on_requested_group_id,
            );
            if ret_val != 0 {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }

        swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
        let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
        ret_val = pre_selection_requirement11(
            h_uni_drc_config,
            pot,
            sl,
            &p.mpegh_request,
            omit_pre_selection_based_on_requested_group_id,
        );
        if ret_val != 0 {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        }
    }

    ret_val
}

fn drc_set_request_selection(
    p: &SelProcInput,
    h_uni_drc_config: &UniDrcConfig,
    sel: &mut Sel,
    pp_candidates_potential: &mut usize,
    pp_candidates_selected: &mut usize,
) -> DrcDecSelectionProcessReturn {
    if selection_get_number(&sel[*pp_candidates_potential]) == 0 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    if p.dynamic_range_control_on != 0 {
        if p.num_drc_feature_requests == 0 {
            {
                let (pot, sl) = pot_sel(sel, *pp_candidates_potential, *pp_candidates_selected);
                let ret_val = select_drc_set_effect_none(h_uni_drc_config, pot, sl);
                if ret_val != 0 {
                    return ret_val;
                }
            }

            if selection_get_number(&sel[*pp_candidates_selected]) == 0 {
                let mut fallback_request = DrcFeatureRequest::ZERO;
                fallback_request.num_requests = 5;
                fallback_request.num_requests_desired = 5;
                fallback_request.request[0] = DETR_GENERAL_COMPR;
                fallback_request.request[1] = DETR_NIGHT;
                fallback_request.request[2] = DETR_NOISY;
                fallback_request.request[3] = DETR_LIMITED;
                fallback_request.request[4] = DETR_LOWLEVEL;

                let ret_val = select_effect_type_feature(
                    h_uni_drc_config,
                    fallback_request,
                    sel,
                    pp_candidates_potential,
                    pp_candidates_selected,
                );
                if ret_val != 0 {
                    return DRCDEC_SELECTION_PROCESS_NOT_OK;
                }
            }

            swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
        } else {
            for i in 0..p.num_drc_feature_requests as usize {
                if p.drc_feature_request_type[i.min(MAX_REQUESTS_DRC_FEATURE - 1)]
                    == DFRT_EFFECT_TYPE
                {
                    let ret_val = select_effect_type_feature(
                        h_uni_drc_config,
                        p.drc_feature_request[i.min(MAX_REQUESTS_DRC_FEATURE - 1)],
                        sel,
                        pp_candidates_potential,
                        pp_candidates_selected,
                    );

                    swap_selection_and_clear(sel, pp_candidates_potential, pp_candidates_selected);
                    if ret_val != 0 {
                        return DRCDEC_SELECTION_PROCESS_NOT_OK;
                    }
                }
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/*******************************************/

fn get_loudness_info_structure<'a>(
    h_loudness_info_set: &'a LoudnessInfoSet,
    drc_set_id: i32,
    downmix_id: i32,
    group_id_requested: i32,
    group_preset_id_requested: i32,
    mpegh_request: &MpeghRequest,
    codec_mode: SelProcCodecMode,
    album_mode: i32,
) -> Option<&'a LoudnessInfo> {
    let mut loudness_info_matching = [0usize; 32];
    let mut info_count: usize = 0;

    let (count, p_loudness_info): (usize, &[LoudnessInfo; 32]) = if album_mode != 0 {
        (
            h_loudness_info_set.loudness_info_album_count as usize,
            &h_loudness_info_set.loudness_info_album,
        )
    } else {
        (h_loudness_info_set.loudness_info_count as usize, &h_loudness_info_set.loudness_info)
    };

    for i in 0..count.min(32) {
        let li = &p_loudness_info[i];
        if (li.drc_set_id as i32 == drc_set_id) && (li.downmix_id as i32 == downmix_id) {
            for j in 0..li.measurement_count as usize {
                let md = li.loudness_measurement[j.min(15)].method_definition;
                if md == 1 || md == 2 {
                    loudness_info_matching[info_count] = i;
                    info_count += 1;
                    break;
                }
            }
        }
    }

    if codec_mode == SEL_PROC_MPEG_H_3DA {
        let mut k = 0usize;
        let mut l = 0usize;
        let mut m = 0usize;
        let mut loudness_info_type3_match = [0usize; 32];
        let mut loudness_info_type3_match_num_members = [0i32; 32];
        let mut loudness_info_type2_match = [0usize; 32];
        let mut loudness_info_type0_match = [0usize; 32];
        let mut max_num_members_group_preset_ids: i32 = 0;
        let mut max_group_id: i32 = 0;
        let mut largest_group_preset_id: i32 = 0;

        let info = |n: usize| -> &LoudnessInfo { &p_loudness_info[n] };

        /* categorize loudnessInfoMatching dependent on loudnessInfoType */
        for n in 0..info_count {
            let li = info(loudness_info_matching[n]);
            if li.loudness_info_type == 3 {
                for i in 0..mpegh_request.num_group_preset_ids_requested() as usize {
                    if mpegh_request.group_preset_id_requested(i) == li.mae_group_preset_id() {
                        loudness_info_type3_match[k] = n;
                        loudness_info_type3_match_num_members[k] =
                            mpegh_request.num_members_group_preset_ids_requested(i) as i32;
                        k += 1;
                        break;
                    }
                }
            } else if li.loudness_info_type == 2 {
                for i in 0..mpegh_request.num_group_ids_requested() as usize {
                    if mpegh_request.group_id_requested(i) == li.mae_group_id {
                        loudness_info_type2_match[l] = n;
                        l += 1;
                        break;
                    }
                }
            } else if li.loudness_info_type == 0 {
                loudness_info_type0_match[m] = n;
                m += 1;
            }
        }

        info_count = 0;

        /* groupPresetId match */
        if k != 0 {
            /* full match */
            if group_preset_id_requested != -1 {
                for n in 0..k {
                    let src = loudness_info_matching[loudness_info_type3_match[n]];
                    if group_preset_id_requested == info(src).mae_group_preset_id() as i32 {
                        loudness_info_matching[info_count] = src;
                        info_count += 1;
                    }
                }
            }

            /* preference match */
            if info_count == 0 && mpegh_request.group_preset_id_requested_preference() != -1 {
                for n in 0..k {
                    let src = loudness_info_matching[loudness_info_type3_match[n]];
                    if mpegh_request.group_preset_id_requested_preference() as i32
                        == info(src).mae_group_preset_id() as i32
                    {
                        loudness_info_matching[info_count] = src;
                        info_count += 1;
                    }
                }
            }

            /* other match */
            if info_count == 0 {
                for n in 0..k {
                    /* choose groupPreset with largest number of members */
                    if max_num_members_group_preset_ids <= loudness_info_type3_match_num_members[n]
                    {
                        if max_num_members_group_preset_ids
                            < loudness_info_type3_match_num_members[n]
                        {
                            max_num_members_group_preset_ids =
                                loudness_info_type3_match_num_members[n];
                            info_count = 0;
                        }

                        /* choose largest groupPresetId */
                        let src = loudness_info_matching[loudness_info_type3_match[n]];
                        let id = info(src).mae_group_preset_id() as i32;
                        if largest_group_preset_id <= id {
                            if largest_group_preset_id < id {
                                largest_group_preset_id = id;
                                info_count = 0;
                            }

                            loudness_info_matching[info_count] = src;
                            info_count += 1;
                        }
                    }
                }
            }
        }

        /* groupId match */
        if info_count == 0 && l != 0 {
            /* full match */
            if group_id_requested != -1 {
                for n in 0..l {
                    let src = loudness_info_matching[loudness_info_type2_match[n]];
                    if group_id_requested == info(src).mae_group_id as i32 {
                        loudness_info_matching[info_count] = src;
                        info_count += 1;
                    }
                }
            }

            /* other match */
            if info_count == 0 {
                for n in 0..l {
                    let src = loudness_info_matching[loudness_info_type2_match[n]];
                    let id = info(src).mae_group_id as i32;
                    if max_group_id <= id {
                        if max_group_id < id {
                            max_group_id = id;
                            info_count = 0;
                        }

                        loudness_info_matching[info_count] = src;
                        info_count += 1;
                    }
                }
            }
        }

        /* other match without groupId and groupPresetId */
        if info_count == 0 && m != 0 {
            for n in 0..m {
                loudness_info_matching[info_count] =
                    loudness_info_matching[loudness_info_type0_match[n]];
                info_count += 1;
            }
        }
    }

    if info_count > 0 {
        return Some(&p_loudness_info[loudness_info_matching[0]]);
    }

    None
}

fn get_applicable_loudness_info_structure<'a>(
    h_loudness_info_set: &'a LoudnessInfoSet,
    drc_set_id: i32,
    downmix_id_requested: i32,
    group_id_requested: i32,
    group_preset_id_requested: i32,
    p_mpegh_request: &MpeghRequest,
    codec_mode: SelProcCodecMode,
    album_mode: i32,
) -> Option<&'a LoudnessInfo> {
    let get = |d: i32, m: i32| {
        get_loudness_info_structure(
            h_loudness_info_set,
            d,
            m,
            group_id_requested,
            group_preset_id_requested,
            p_mpegh_request,
            codec_mode,
            album_mode,
        )
    };

    /* default value */
    get(drc_set_id, downmix_id_requested)
        /* fallback values */
        .or_else(|| get(drc_set_id, 0x7F))
        .or_else(|| get(0x3F, downmix_id_requested))
        .or_else(|| get(0, downmix_id_requested))
        .or_else(|| get(0x3F, 0x7F))
        .or_else(|| get(0, 0x7F))
        .or_else(|| get(drc_set_id, 0))
        .or_else(|| get(0x3F, 0))
        .or_else(|| get(0, 0))
}

/*******************************************/

/// `VALUE_ORDER`
struct ValueOrder {
    value: FixpDbl,
    order: i32,
}

fn init_value_order() -> ValueOrder {
    ValueOrder { value: 0, order: -1 }
}

fn get_method_value(
    p_value_order: &mut ValueOrder,
    value: FixpDbl,
    measurement_system: i32,
    measurement_system_requested: i32,
) -> DrcDecSelectionProcessReturn {
    const ROWS: i32 = 11;
    const COLUMNS: i32 = 12;
    const P_ORDERING: [[i32; 12]; 11] = [
        [0, 0, 8, 0, 1, 3, 0, 5, 6, 7, 4, 2], /* default = bonus1770 */
        [0, 0, 8, 0, 1, 3, 0, 5, 6, 7, 4, 2], /* bonus1770 */
        [0, 0, 1, 0, 8, 5, 0, 2, 3, 4, 6, 7], /* bonusUser */
        [0, 0, 3, 0, 1, 8, 0, 4, 5, 6, 7, 2], /* bonusExpert */
        [0, 0, 5, 0, 1, 3, 0, 8, 6, 7, 4, 2], /* ResA */
        [0, 0, 5, 0, 1, 3, 0, 6, 8, 7, 4, 2], /* ResB */
        [0, 0, 5, 0, 1, 3, 0, 6, 7, 8, 4, 2], /* ResC */
        [0, 0, 3, 0, 1, 7, 0, 4, 5, 6, 8, 2], /* ResD */
        [0, 0, 1, 0, 7, 5, 0, 2, 3, 4, 6, 8], /* ResE */
        [0, 0, 1, 0, 0, 0, 0, 2, 3, 4, 0, 0], /* ProgramLoudness */
        [0, 7, 0, 0, 0, 0, 6, 5, 4, 3, 2, 1], /* PeakLoudness */
    ];

    if measurement_system_requested < 0
        || measurement_system_requested >= ROWS
        || measurement_system < 0
        || measurement_system >= COLUMNS
    {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    let o = P_ORDERING[measurement_system_requested as usize][measurement_system as usize];
    if o > p_value_order.order {
        p_value_order.order = o;
        p_value_order.value = value;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/*******************************************/

fn get_loudness(
    h_loudness_info_set: &LoudnessInfoSet,
    album_mode: i32,
    measurement_method_requested: MethodDefinitionRequest,
    measurement_system_requested: MeasurementSystemRequest,
    target_loudness: FixpDbl, /* e = 7 */
    mut drc_set_id: i32,
    downmix_id_requested: i32,
    codec_mode: SelProcCodecMode,
    group_id_requested: i32,
    group_preset_id_requested: i32,
    p_mpegh_request: &MpeghRequest,
    p_loudness_normalization_gain: &mut FixpDbl, /* e = 7 */
    p_loudness: &mut FixpDbl,                    /* e = 7 */
) -> DrcDecSelectionProcessReturn {
    /* map MDR_DEFAULT to MDR_PROGRAM_LOUDNESS */
    let requested_method_definition = if measurement_method_requested < MDR_ANCHOR_LOUDNESS {
        MDR_PROGRAM_LOUDNESS
    } else {
        MDR_ANCHOR_LOUDNESS
    };

    if measurement_method_requested > MDR_ANCHOR_LOUDNESS {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    }

    let mut value_order = init_value_order();

    *p_loudness = UNDEFINED_LOUDNESS_VALUE;
    *p_loudness_normalization_gain = 0;

    if drc_set_id < 0 {
        drc_set_id = 0;
    }

    let mut p_loudness_info = get_applicable_loudness_info_structure(
        h_loudness_info_set,
        drc_set_id,
        downmix_id_requested,
        group_id_requested,
        group_preset_id_requested,
        p_mpegh_request,
        codec_mode,
        album_mode,
    );

    if album_mode != 0 && p_loudness_info.is_none() {
        p_loudness_info = get_applicable_loudness_info_structure(
            h_loudness_info_set,
            drc_set_id,
            downmix_id_requested,
            group_id_requested,
            group_preset_id_requested,
            p_mpegh_request,
            codec_mode,
            0,
        );
    }

    let Some(p_loudness_info) = p_loudness_info else {
        return DRCDEC_SELECTION_PROCESS_NO_ERROR;
    };

    let mut index: i32 = -1;

    loop {
        index = find_method_definition(p_loudness_info, requested_method_definition, index + 1);

        if index >= 0 {
            let lm = &p_loudness_info.loudness_measurement[index as usize];
            get_method_value(
                &mut value_order,
                lm.method_value,
                lm.measurement_system as i32,
                measurement_system_requested,
            );
        }
        if index < 0 {
            break;
        }
    }

    /* repeat with other method definition */
    if value_order.order == -1 {
        index = -1;

        loop {
            index = find_method_definition(
                p_loudness_info,
                if requested_method_definition == MDR_PROGRAM_LOUDNESS {
                    MDR_ANCHOR_LOUDNESS
                } else {
                    MDR_PROGRAM_LOUDNESS
                },
                index + 1,
            );

            if index >= 0 {
                let lm = &p_loudness_info.loudness_measurement[index as usize];
                get_method_value(
                    &mut value_order,
                    lm.method_value,
                    lm.measurement_system as i32,
                    measurement_system_requested,
                );
            }
            if index < 0 {
                break;
            }
        }
    }

    if value_order.order == -1 {
        return DRCDEC_SELECTION_PROCESS_NOT_OK;
    } else {
        *p_loudness_normalization_gain = target_loudness.wrapping_sub(value_order.value);
        *p_loudness = value_order.value;
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/*******************************************/

fn loudness_info_group_matches(
    li: &LoudnessInfo,
    codec_mode: SelProcCodecMode,
    drc_instructions_type: i32,
    group_preset_id: i32,
    group_id: i32,
) -> bool {
    (codec_mode != SEL_PROC_MPEG_H_3DA)
        || ((drc_instructions_type == 3
            && li.loudness_info_type == 3
            && group_preset_id == li.mae_group_preset_id() as i32)
            || (drc_instructions_type == 2
                && li.loudness_info_type == 2
                && group_id == li.mae_group_id as i32)
            || (drc_instructions_type == 0 && li.loudness_info_type == 0))
}

fn get_true_peak_level(
    h_loudness_info_set: &LoudnessInfoSet,
    drc_set_id: i32,
    downmix_id: i32,
    album_mode: i32,
    p_true_peak_level: &mut FixpDbl,
    codec_mode: SelProcCodecMode,
    drc_instructions_type: i32,
    group_preset_id: i32,
    group_id: i32,
) -> bool {
    let (count, p_loudness_info) = if album_mode != 0 {
        (
            h_loudness_info_set.loudness_info_album_count as usize,
            &h_loudness_info_set.loudness_info_album,
        )
    } else {
        (h_loudness_info_set.loudness_info_count as usize, &h_loudness_info_set.loudness_info)
    };

    for li in p_loudness_info.iter().take(count) {
        if (li.drc_set_id as i32 == drc_set_id)
            && (li.downmix_id as i32 == downmix_id)
            && loudness_info_group_matches(
                li,
                codec_mode,
                drc_instructions_type,
                group_preset_id,
                group_id,
            )
            && li.true_peak_level_present != 0
        {
            *p_true_peak_level = li.true_peak_level;
            return true; /* value found */
        }
    }

    false /* no value found */
}

fn get_sample_peak_level(
    h_loudness_info_set: &LoudnessInfoSet,
    drc_set_id: i32,
    downmix_id: i32,
    album_mode: i32,
    p_sample_peak_level: &mut FixpDbl, /* e = 7 */
    codec_mode: SelProcCodecMode,
    drc_instructions_type: i32,
    group_preset_id: i32,
    group_id: i32,
) -> bool {
    let (count, p_loudness_info) = if album_mode != 0 {
        (
            h_loudness_info_set.loudness_info_album_count as usize,
            &h_loudness_info_set.loudness_info_album,
        )
    } else {
        (h_loudness_info_set.loudness_info_count as usize, &h_loudness_info_set.loudness_info)
    };

    for li in p_loudness_info.iter().take(count) {
        if (li.drc_set_id as i32 == drc_set_id)
            && (li.downmix_id as i32 == downmix_id)
            && loudness_info_group_matches(
                li,
                codec_mode,
                drc_instructions_type,
                group_preset_id,
                group_id,
            )
            && li.sample_peak_level_present != 0
        {
            *p_sample_peak_level = li.sample_peak_level;
            return true; /* value found */
        }
    }

    false /* no value found */
}

fn get_limiter_peak_target(
    p_drc_instruction: &DrcInstructionsUniDrc,
    downmix_id: i32,
    p_limiter_peak_target: &mut FixpDbl,
) -> bool {
    if p_drc_instruction.limiter_peak_target_present != 0 {
        if (p_drc_instruction.downmix_id[0] as i32 == downmix_id)
            || (p_drc_instruction.downmix_id[0] == 0x7F)
        {
            *p_limiter_peak_target = fx_sgl2fx_dbl(p_drc_instruction.limiter_peak_target) >> 2;
            return true; /* value found */
        }

        for i in 0..p_drc_instruction.downmix_id_count as usize {
            if p_drc_instruction.downmix_id[i.min(7)] as i32 == downmix_id {
                *p_limiter_peak_target = fx_sgl2fx_dbl(p_drc_instruction.limiter_peak_target) >> 2;
                return true; /* value found */
            }
        }
    }

    false /* no value found */
}

fn get_peak_virtual_drc_from_loudness_info(
    loudness_info: &LoudnessInfo,
    p_mpegh_request: &MpeghRequest,
    signal_peak_level: &mut FixpDbl,
    explicit_peak_information_present: &mut i32,
) -> bool {
    let matches = |li: &LoudnessInfo| -> bool {
        if li.loudness_info_type == 3
            && p_mpegh_request.group_preset_id_requested_preference() as i32
                == li.mae_group_preset_id() as i32
        {
            return true;
        }

        if li.loudness_info_type == 3 {
            for n in 0..p_mpegh_request.num_group_preset_ids_requested() as usize {
                if p_mpegh_request.group_preset_id_requested(n) == li.mae_group_preset_id() {
                    return true;
                }
            }
        }

        if li.loudness_info_type == 2 {
            for n in 0..p_mpegh_request.num_group_ids_requested() as usize {
                if p_mpegh_request.group_id_requested(n) == li.mae_group_id {
                    return true;
                }
            }
        }
        false
    };

    if loudness_info.true_peak_level_present != 0 {
        *signal_peak_level = loudness_info.true_peak_level;

        *explicit_peak_information_present = 1;

        if matches(loudness_info) {
            return true;
        }
    }

    if loudness_info.sample_peak_level_present != 0 {
        *signal_peak_level = loudness_info.sample_peak_level;

        *explicit_peak_information_present = 1;

        if matches(loudness_info) {
            return true;
        }
    }

    false
}

fn get_peak_virtual_drc_from_loudness_info_set(
    h_loudness_info_set: &LoudnessInfoSet,
    album_mode: i32,
    drc_set_id_requested: i32,
    downmix_id_requested: i32,
    p_mpegh_request: &MpeghRequest,
    signal_peak_level: &mut FixpDbl,
    explicit_peak_information_present: &mut i32,
) -> bool {
    let mut match_found = false;

    let (count, p_loudness_info) = if album_mode != 0 {
        (
            h_loudness_info_set.loudness_info_album_count as usize,
            &h_loudness_info_set.loudness_info_album,
        )
    } else {
        (h_loudness_info_set.loudness_info_count as usize, &h_loudness_info_set.loudness_info)
    };

    for li in p_loudness_info.iter().take(count) {
        if drc_set_id_requested == li.drc_set_id as i32
            && downmix_id_requested == li.downmix_id as i32
        {
            match_found = get_peak_virtual_drc_from_loudness_info(
                li,
                p_mpegh_request,
                signal_peak_level,
                explicit_peak_information_present,
            );

            if match_found {
                break;
            }
        }
    }

    match_found
}

fn get_signal_peak_level(
    p: &SelProcInput,
    h_loudness_info_set: &LoudnessInfoSet,
    p_inst: &DrcInstructionsUniDrc,
    downmix_id_requested: i32,
    explicit_peak_information_present: &mut i32,
    signal_peak_level_out: &mut FixpDbl, /* e = 7 */
    codec_mode: SelProcCodecMode,
) -> DrcDecSelectionProcessReturn {
    let ret_val = DRCDEC_SELECTION_PROCESS_NO_ERROR;

    let album_mode = p.album_mode as i32;

    let mut signal_peak_level_tmp: FixpDbl;
    let mut signal_peak_level: FixpDbl = 0;

    let mut match_found = false;
    let dmx_id = downmix_id_requested;
    let mut drc_set_id = p_inst.drc_set_id as i32;
    let t = p_inst.drc_instructions_type as i32;
    let gp = p_inst.mae_group_preset_id() as i32;
    let gi = p_inst.mae_group_id as i32;

    if drc_set_id < 0 {
        drc_set_id = 0;
    }

    *explicit_peak_information_present = 1;

    if codec_mode == SEL_PROC_MPEG_H_3DA {
        if drc_set_id == 0 {
            match_found = get_peak_virtual_drc_from_loudness_info_set(
                h_loudness_info_set,
                album_mode,
                drc_set_id,
                dmx_id,
                &p.mpegh_request,
                signal_peak_level_out,
                explicit_peak_information_present,
            );
        }

        if match_found {
            return ret_val; /* finished */
        }
    }

    match_found = get_true_peak_level(
        h_loudness_info_set,
        drc_set_id,
        dmx_id,
        album_mode,
        &mut signal_peak_level,
        codec_mode,
        t,
        gp,
        gi,
    );
    if !match_found {
        match_found = get_sample_peak_level(
            h_loudness_info_set,
            drc_set_id,
            dmx_id,
            album_mode,
            &mut signal_peak_level,
            codec_mode,
            t,
            gp,
            gi,
        );
    }
    if !match_found {
        match_found = get_true_peak_level(
            h_loudness_info_set,
            0x3F,
            dmx_id,
            album_mode,
            &mut signal_peak_level,
            codec_mode,
            t,
            gp,
            gi,
        );
    }
    if !match_found {
        match_found = get_sample_peak_level(
            h_loudness_info_set,
            0x3F,
            dmx_id,
            album_mode,
            &mut signal_peak_level,
            codec_mode,
            t,
            gp,
            gi,
        );
    }
    if !match_found {
        match_found = get_limiter_peak_target(p_inst, dmx_id, &mut signal_peak_level);
    }
    if !match_found && (dmx_id != 0) {
        let downmix_peak_level_db: FixpDbl = 0;

        *explicit_peak_information_present = 0;

        signal_peak_level_tmp = 0;

        if codec_mode == SEL_PROC_MPEG_H_3DA {
            if drc_set_id == 0 {
                match_found = get_peak_virtual_drc_from_loudness_info_set(
                    h_loudness_info_set,
                    album_mode,
                    drc_set_id,
                    dmx_id,
                    &p.mpegh_request,
                    &mut signal_peak_level_tmp,
                    explicit_peak_information_present,
                );
            }

            if match_found {
                *signal_peak_level_out = signal_peak_level_tmp.wrapping_add(downmix_peak_level_db);
                return ret_val; /* finished */
            }
        }

        if !match_found {
            match_found = get_true_peak_level(
                h_loudness_info_set,
                drc_set_id,
                0,
                album_mode,
                &mut signal_peak_level_tmp,
                codec_mode,
                t,
                gp,
                gi,
            );
        }
        if !match_found {
            match_found = get_sample_peak_level(
                h_loudness_info_set,
                drc_set_id,
                0,
                album_mode,
                &mut signal_peak_level_tmp,
                codec_mode,
                t,
                gp,
                gi,
            );
        }
        if !match_found {
            match_found = get_true_peak_level(
                h_loudness_info_set,
                0x3F,
                0,
                album_mode,
                &mut signal_peak_level_tmp,
                codec_mode,
                t,
                gp,
                gi,
            );
        }
        if !match_found {
            match_found = get_sample_peak_level(
                h_loudness_info_set,
                0x3F,
                0,
                album_mode,
                &mut signal_peak_level_tmp,
                codec_mode,
                t,
                gp,
                gi,
            );
        }
        if !match_found {
            get_limiter_peak_target(p_inst, 0, &mut signal_peak_level_tmp);
        }

        signal_peak_level = signal_peak_level_tmp.wrapping_add(downmix_peak_level_db);
    } else if !match_found {
        signal_peak_level = 0; /* worst case estimate */
        *explicit_peak_information_present = 0;
    }

    *signal_peak_level_out = signal_peak_level;

    ret_val
}

/*******************************************/

fn select_album_loudness(
    h_loudness_info_set: &LoudnessInfoSet,
    cfg: &UniDrcConfig,
    p_candidates_potential: &DrcDecSelection,
    p_candidates_selected: &mut DrcDecSelection,
) -> DrcDecSelectionProcessReturn {
    let mut v = 0; /* number of selected virtual DRC sets */

    for i in 0..selection_get_number(p_candidates_potential) {
        let Some(p_candidate) = selection_get_at(p_candidates_potential, i) else {
            return DRCDEC_SELECTION_PROCESS_NOT_OK;
        };
        let drc_set_id = cfg.drc_instructions_uni_drc[p_candidate.p_inst].drc_set_id;

        if drc_set_id < 0 {
            /* keep virtual DRC sets */
            v += 1;
            if !selection_add(p_candidates_selected, p_candidate) {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
            continue;
        }
        for j in 0..h_loudness_info_set.loudness_info_album_count as usize {
            if drc_set_id == h_loudness_info_set.loudness_info_album[j.min(31)].drc_set_id
                && !selection_add(p_candidates_selected, p_candidate)
            {
                return DRCDEC_SELECTION_PROCESS_NOT_OK;
            }
        }
    }

    if v == selection_get_number(p_candidates_selected) {
        /* we have only virtual DRC sets. Dismiss the selection */
        selection_clear(p_candidates_selected);
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/*******************************************/

fn find_method_definition(
    p_loudness_info: &LoudnessInfo,
    method_definition: i32,
    start_index: i32,
) -> i32 {
    let mut index = -1;

    let mut i = start_index;
    while i < p_loudness_info.measurement_count as i32 {
        if p_loudness_info.loudness_measurement[(i as usize).min(15)].method_definition as i32
            == method_definition
        {
            index = i;
            break;
        }
        i += 1;
    }

    index
}

/*******************************************/

fn get_mixing_level(
    h_loudness_info_set: &LoudnessInfoSet,
    downmix_id_requested: i32,
    mut drc_set_id_requested: i32,
    album_mode: i32,
    p_mixing_level: &mut FixpDbl,
) -> DrcDecSelectionProcessReturn {
    let mixing_level_default: FixpDbl = fl2fxconst_dbl((85.0f32 / 128.0f32) as f64);

    *p_mixing_level = mixing_level_default;

    if drc_set_id_requested < 0 {
        drc_set_id_requested = 0;
    }

    let (count, p_loudness_info) = if album_mode != 0 {
        (
            h_loudness_info_set.loudness_info_album_count as usize,
            &h_loudness_info_set.loudness_info_album,
        )
    } else {
        (h_loudness_info_set.loudness_info_count as usize, &h_loudness_info_set.loudness_info)
    };

    for li in p_loudness_info.iter().take(count) {
        if (drc_set_id_requested == li.drc_set_id as i32)
            && ((downmix_id_requested == li.downmix_id as i32)
                || (DOWNMIX_ID_ANY_DOWNMIX == li.downmix_id as i32))
        {
            let index = find_method_definition(li, MD_MIXING_LEVEL, 0);

            if index >= 0 {
                *p_mixing_level = li.loudness_measurement[index as usize].method_value;
                break;
            }
        }
    }

    DRCDEC_SELECTION_PROCESS_NO_ERROR
}

/*******************************************/

fn get_group_loudness(
    h_loudness_info_set: &LoudnessInfoSet,
    out: &mut SelProcOutput,
) -> DrcDecSelectionProcessReturn {
    /* collect group loudness infos from loudnessInfoSet into selection process output */
    out.group_id_loudness_count = 0;
    for li in h_loudness_info_set
        .loudness_info
        .iter()
        .take(h_loudness_info_set.loudness_info_count as usize)
    {
        if li.loudness_info_type == 1 {
            /* group loudness */
            if li.downmix_id as i32 != DOWNMIX_ID_BASE_LAYOUT {
                continue;
            }
            if li.drc_set_id as i32 != DRC_SET_ID_NO_DRC {
                continue;
            }

            for m in 0..li.measurement_count as usize {
                /* assumption: program and anchor loudness are equivalent for groups */
                let lm = &li.loudness_measurement[m.min(15)];
                if (lm.method_definition as i32 == MD_PROGRAM_LOUDNESS)
                    || (lm.method_definition as i32 == MD_ANCHOR_LOUDNESS)
                {
                    let n = out.group_id_loudness_count as usize;
                    out.group_id[n] = li.mae_group_id;
                    out.group_id_loudness[n] = lm.method_value;
                    out.group_id_loudness_count += 1;
                    break;
                }
            }
        }
    }
    DRCDEC_SELECTION_PROCESS_NO_ERROR
}
