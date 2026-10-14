// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpegTPDec/include/tp_data.h.

//! MPEG transport data types: audio specific config, config change control, audio scene info
//! and the callback interface to the core decoder.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;

// Configuration

pub const TPDEC_MAX_PROGRAMS: usize = 1;
pub const TPDEC_MAX_LAYERS: usize = 4;
pub const TPDEC_MAX_TRACKS: usize = TPDEC_MAX_PROGRAMS * TPDEC_MAX_LAYERS;

pub const TP_MPEGH3DA_MAX_LEVEL: i32 = 5;

pub const TP_USAC_MAX_SPEAKERS: usize = 28;

pub const TP_MAX_CHANNELS_PER_SIGNAL_GROUP: usize = 28;

/// X OAM side-info + X extOAM + X/2 MCT + N preroll + N fill element + N DRC + N VR-EXT
pub const TP_USAC_MAX_EXT_ELEMENTS: usize = 2 * (2 * 28) + (2 * 28) / 2 + 4 * 1;

pub const TP_USAC_MAX_ELEMENTS: usize = (2 * 28) + TP_USAC_MAX_EXT_ELEMENTS;

pub const TP_MPEGH3DA_MAX_CONFIG_LEN: usize = 4096;
/// Next power of two of maximum of escapedValue(hBs, 4, 4, 8) in AudioPreRoll() (285).
pub const TP_USAC_MAX_CONFIG_LEN: usize = 512;

pub const TP_MPEGH_MAX_PROD_METADATA_CONFIG_LEN: i32 = 5;

pub const TP_MPEGH_MAX_SIGNAL_GROUPS: usize = 2 * 28;

/// `mha_pactyp_t`
pub type MhaPactyp = i32;
pub const MHA_PACTYP_NONE: MhaPactyp = -1;
pub const MHA_PACTYP_FILLDATA: MhaPactyp = 0;
pub const MHA_PACTYP_MPEGH3DACFG: MhaPactyp = 1;
pub const MHA_PACTYP_MPEGH3DAFRAME: MhaPactyp = 2;
pub const MHA_PACTYP_AUDIOSCENEINFO: MhaPactyp = 3;
// reserved for ISO use 4-5
pub const MHA_PACTYP_SYNC: MhaPactyp = 6;
pub const MHA_PACTYP_SYNCGAP: MhaPactyp = 7;
pub const MHA_PACTYP_MARKER: MhaPactyp = 8;
pub const MHA_PACTYP_CRC16: MhaPactyp = 9;
pub const MHA_PACTYP_CRC32: MhaPactyp = 10;
pub const MHA_PACTYP_DESCRIPTOR: MhaPactyp = 11;
pub const MHA_PACTYP_USERINTERACTION: MhaPactyp = 12;
pub const MHA_PACTYP_LOUDNESS_DRC: MhaPactyp = 13;
pub const MHA_PACTYP_BUFFERINFO: MhaPactyp = 14;
/// MPEG112 input
pub const MHA_PACTYP_GLOBAL_CRC16: MhaPactyp = 15;
/// MPEG112 input
pub const MHA_PACTYP_GLOBAL_CRC32: MhaPactyp = 16;
/// MPEG112 input
pub const MHA_PACTYP_AUDIOTRUNCATION: MhaPactyp = 17;
/// ISO/IEC 23008-3:2015/FDAM 5:201x Amd 1
pub const MHA_PACTYPE_EARCON: MhaPactyp = 19;
/// ISO/IEC 23008-3:2015/FDAM 5:201x Amd 1
pub const MHA_PACTYPE_PCMCONFIG: MhaPactyp = 20;
/// ISO/IEC 23008-3:2015/FDAM 5:201x Amd 1
pub const MHA_PACTYPE_PCMDATA: MhaPactyp = 21;
/// 23008-3:2019/Amd 1 (Amendment 1 to 2nd edition of MPEG-H)
pub const MHA_PACTYP_LOUDNESS: MhaPactyp = 22;
pub const MHA_PACTYP_FRAMELENGTH: MhaPactyp = 129;
/// max(escapedValue(3,8,8)) + 1
pub const MHA_PACTYP_UNKNOWN: MhaPactyp = 518;

/// `MHA_IS_MAIN_STREAM(x)`
#[inline]
pub fn mha_is_main_stream(x: u32) -> bool {
    0 < x && x <= 16
}

/// Number of frames for config change in ATSC
pub const TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES: i8 = 1;
/// Number of frames for config change in USAC
pub const TPDEC_USAC_NUM_CONFIG_CHANGE_FRAMES: i8 = 1;

pub const TPDEC_FLUSH_OFF: u8 = 0;
pub const TPDEC_MPEGH_CFG_CHANGE_ATSC_FLUSH_ON: u8 = 1;
pub const TPDEC_MPEGH_DASH_IPF_ATSC_FLUSH_ON: u8 = 2;
pub const TPDEC_USAC_DASH_IPF_FLUSH_ON: u8 = 3;

pub const TPDEC_BUILD_UP_OFF: u8 = 0;
pub const TPDEC_MPEGH_BUILD_UP_ON: u8 = 1;
pub const TPDEC_MPEGH_BUILD_UP_ON_IN_BAND: u8 = 2;
pub const TPDEC_USAC_BUILD_UP_ON: u8 = 3;
pub const TPDEC_MPEGH_BUILD_UP_IDLE: u8 = 4;
pub const TPDEC_MPEGH_BUILD_UP_IDLE_IN_BAND: u8 = 5;

// ProgramConfig: ISO/IEC 14496-3 4.4.1.1 Table 4.2 Program config element
/// Front/Side/Back channels
pub const PC_FSB_CHANNELS_MAX: usize = 16;
pub const PC_LFE_CHANNELS_MAX: usize = 4;
pub const PC_ASSOCDATA_MAX: usize = 8;
/// CC elements
pub const PC_CCEL_MAX: usize = 16;
pub const PC_COMMENTLENGTH: usize = 256;
pub const PC_NUM_HEIGHT_LAYER: usize = 3;

/// `CProgramConfig`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CProgramConfig {
    /// States whether PCE has been read successfully (0: invalid, 1: valid, 2: valid but
    /// implicit for trial).
    pub is_valid: u8,
    /// Amount of audio channels summing all channel elements including LFEs.
    pub num_channels: u8,
    /// Amount of audio channels summing only SCEs and CPEs.
    pub num_effective_channels: u8,
    pub el_counter: u8,
}

/// `TP_ASC_EXTENSION_ID`
pub type TpAscExtensionId = i32;
pub const ASCEXT_UNKOWN: TpAscExtensionId = -1;
pub const ASCEXT_SBR: TpAscExtensionId = 0x2b7;
pub const ASCEXT_PS: TpAscExtensionId = 0x548;
pub const ASCEXT_MPS: TpAscExtensionId = 0x76a;
pub const ASCEXT_SAOC: TpAscExtensionId = 0x7cb;
pub const ASCEXT_LDMPS: TpAscExtensionId = 0x7cc;

/// `struct OAMCONFIG`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OamConfig {
    pub oam_frame_length: u16,
    pub low_delay_metadata_coding: u8,
    pub has_dynamic_object_priority: u8,
    pub has_uniform_spread: u8,
    pub num_object_signals: u8,
}

/// `extConfig.prodMetadata`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ProdMetadataConfig {
    /// msb aligned bit field: one bit for each object group
    pub has_object_distance: u32,
    pub has_reference_distance: u8,
    pub bs_reference_distance: u8,
}

/// `extConfig.mct`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MctConfig {
    /// msb aligned bit field: one bit for each channel
    pub mct_chan_mask: u32,
}

/// `CSUsacExtElementConfig::extConfig`. In C this is a union; each member is written and read
/// only for the extension element type it belongs to (`usacExtElementType`), so the members are
/// stored separately here.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExtConfig {
    pub oam: OamConfig,
    pub prod_metadata: ProdMetadataConfig,
    pub mct: MctConfig,
}

/// `CSUsacExtElementConfig`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CSUsacExtElementConfig {
    pub usac_ext_element_type: UsacExtElementType,
    pub usac_ext_element_default_length: u16,
    pub usac_ext_element_payload_frag: u8,
    pub usac_ext_element_has_audio_pre_roll: u8,
    pub ext_config: ExtConfig,
}

/// `CSUsacElementConfig`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CSUsacElementConfig {
    pub usac_element_type: Mp4ElementId,
    pub fullband_lpd: u8,
    pub lpd_stereo_index: u8,
    pub m_noise_filling: u8,
    // enhanced noise filling / intelligent gap filling
    pub enhanced_noise_filling: u8,
    pub igf_use_enf: u8,
    pub igf_use_whitening: u8,
    pub igf_after_tns_synth: u8,
    pub igf_independent_tiling: u8,
    pub igf_start_index: u8,
    pub igf_use_high_res: u8,
    pub igf_stop_index: u8,
    pub igf_n_tiles: u8,
    pub shift_index1: u8,
    pub shift_channel1: u8,
    pub ext_element: CSUsacExtElementConfig,
}

/// `mpegh_speaker_t`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MpeghSpeaker {
    /// Flag indicating if the speaker is LFE.
    pub lfe: u8,
    /// Azimuth angle in degrees quantized in steps of 1 degree. Positive is counter clock wise.
    pub az: i16,
    /// Elevation angle in degrees quantized in steps of 1 degree. Positive is up.
    pub el: i16,
}

/// Target of the C pointer `CSSignalGroup::speakers`, which points into the enclosing
/// [`CSUsacConfig`]. Use [`CSUsacConfig::signal_group_speakers`] to resolve it.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SpeakerRef {
    /// `NULL` (signal group not parsed yet).
    #[default]
    None,
    /// `&usc->speakers[0]` (the reference layout).
    ReferenceLayout,
    /// `&usc->m_signalGroupTypeSpeakers[n]`.
    SignalGroup(u8),
}

/// Value of [`CSSignalGroup::speakers_owner`] for a pointer into the containing config.
pub const SPEAKERS_OWNER_SELF: u8 = 0xFF;

/// `CSSignalGroup`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CSSignalGroup {
    /// Type 0: Channel signals / 1: Object signals / 2: SAOC signals / 3: HOA signals /
    /// Other: USAC
    pub type_: u8,
    /// Number of signals related to this signal group
    pub count: u8,
    pub layout: u8,
    /// `speakers` pointer, see [`SpeakerRef`].
    pub speakers: SpeakerRef,
    /// Emulation of the absolute address of the C pointer `speakers`: the index into
    /// `TransportDec::asc` of the config the pointer points into, or [`SPEAKERS_OWNER_SELF`]
    /// (the config containing this signal group). A struct copy keeps the pointer target (in C,
    /// `hTp->asc[layer] = hTp->asc[TPDEC_MAX_TRACKS]` leaves the pointers pointing into
    /// `asc[TPDEC_MAX_TRACKS]`). Resolve with `TransportDec::signal_group_speakers()`.
    pub speakers_owner: u8,
    /// index of first signal in group
    pub first_sig_idx: u8,
    /// flag indicating that a downmix matrix delivered by the bit stream should be used.
    pub b_use_custom_downmix_matrix: u8,
}

/// `CSUsacConfig`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CSUsacConfig {
    pub m_frame_length_flag: u8,
    pub m_core_sbr_frame_length_index: u8,
    pub m_sbr_ratio_index: u8,
    /// number of audio channels signaled in UsacDecoderConfig() / mpegh3daDecoderConfig() via
    /// numElements and usacElementType
    pub m_n_usac_channels: u8,
    pub m_channel_configuration_index: u8,
    pub m_usac_num_elements: u32,
    pub element: [CSUsacElementConfig; TP_USAC_MAX_ELEMENTS],

    // general configuration data
    /// Signal group type.
    pub m_signal_group_type: [CSSignalGroup; TP_MPEGH_MAX_SIGNAL_GROUPS],
    pub m_signal_group_type_speakers: [MpeghSpeaker; 2 * 28],
    pub reference_layout: u8,
    pub speakers: [MpeghSpeaker; TP_USAC_MAX_SPEAKERS],
    pub bs_num_signal_groups: u8,
    pub mpegh3da_profile_level_indication: u8,
    pub num_audio_objects: u8,
    pub num_hoa_transport_channels: u8,
    /// count signal groups of type channels
    pub ch_num_signal_groups: u8,
    /// count signal groups of type object
    pub obj_num_signal_groups: u8,
    /// store indice of current signal group
    pub obj_signal_groups_indices: [u8; TP_MPEGH_MAX_SIGNAL_GROUPS],
    /// signal index to group index table
    pub sig_idx2_grp_idx: [u8; TP_MPEGH_MAX_SIGNAL_GROUPS],
    // downmixConfig() parameters
    pub passive_downmix_flag: u8,
    pub phase_align_strength: u8,
    pub immersive_downmix_flag: u8,
    pub downmix_config_type: u8,
    pub downmix_id_count: u8,
    pub sub_stream_index: u8,
    pub target_layout: i32,
    pub ui_manager_active: i8,
    pub num_audio_channels: u8,
    pub m_usac_config_extension_present: u8,
    pub element_length_present: u8,
}

impl Default for CSUsacConfig {
    fn default() -> Self {
        Self {
            m_frame_length_flag: 0,
            m_core_sbr_frame_length_index: 0,
            m_sbr_ratio_index: 0,
            m_n_usac_channels: 0,
            m_channel_configuration_index: 0,
            m_usac_num_elements: 0,
            element: [CSUsacElementConfig::default(); TP_USAC_MAX_ELEMENTS],
            m_signal_group_type: [CSSignalGroup::default(); TP_MPEGH_MAX_SIGNAL_GROUPS],
            m_signal_group_type_speakers: [MpeghSpeaker::default(); 2 * 28],
            reference_layout: 0,
            speakers: [MpeghSpeaker::default(); TP_USAC_MAX_SPEAKERS],
            bs_num_signal_groups: 0,
            mpegh3da_profile_level_indication: 0,
            num_audio_objects: 0,
            num_hoa_transport_channels: 0,
            ch_num_signal_groups: 0,
            obj_num_signal_groups: 0,
            obj_signal_groups_indices: [0; TP_MPEGH_MAX_SIGNAL_GROUPS],
            sig_idx2_grp_idx: [0; TP_MPEGH_MAX_SIGNAL_GROUPS],
            passive_downmix_flag: 0,
            phase_align_strength: 0,
            immersive_downmix_flag: 0,
            downmix_config_type: 0,
            downmix_id_count: 0,
            sub_stream_index: 0,
            target_layout: 0,
            ui_manager_active: 0,
            num_audio_channels: 0,
            m_usac_config_extension_present: 0,
            element_length_present: 0,
        }
    }
}

/// `ascs[layer].m_sc.m_usacConfig.m_signalGroupType[grp].speakers` for configs owned by the
/// transport decoder (`hTp->asc[]`): follows the pointer into the config it points to (see
/// [`CSSignalGroup::speakers_owner`]). Index the result with `0..count`.
pub fn asc_signal_group_speakers(
    ascs: &[CSAudioSpecificConfig],
    layer: usize,
    grp: usize,
) -> &[MpeghSpeaker] {
    let sg = &ascs[layer].m_sc.m_signal_group_type[grp];
    let owner =
        if sg.speakers_owner == SPEAKERS_OWNER_SELF { layer } else { sg.speakers_owner as usize };
    ascs[owner].m_sc.speakers_of(sg.speakers)
}

impl CSUsacConfig {
    /// Speakers of signal group `grp` (`m_signalGroupType[grp].speakers`), resolved within this
    /// config: a slice starting at the position the C pointer points to; index it with
    /// `0..count`. Empty for a `NULL` pointer. For configs owned by the transport decoder use
    /// `TransportDec::signal_group_speakers()`, which follows the pointer target like C does.
    pub fn signal_group_speakers(&self, grp: usize) -> &[MpeghSpeaker] {
        self.speakers_of(self.m_signal_group_type[grp].speakers)
    }

    /// Resolves a speaker reference within this config.
    pub fn speakers_of(&self, r: SpeakerRef) -> &[MpeghSpeaker] {
        match r {
            SpeakerRef::None => &[],
            SpeakerRef::ReferenceLayout => &self.speakers,
            SpeakerRef::SignalGroup(n) => &self.m_signal_group_type_speakers[n as usize..],
        }
    }
}

/// `CSAudioSpecificConfig`: audio configuration, suitable for encoder and decoder configuration.
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CSAudioSpecificConfig {
    /// `m_sc.m_usacConfig`: USAC specific configuration (the union has a single member).
    pub m_sc: CSUsacConfig,

    // Common ASC parameters
    /// Audio Object Type.
    pub m_aot: AudioObjectType,
    /// Samplerate.
    pub m_sampling_frequency: u32,
    /// Amount of samples per frame.
    pub m_samples_per_frame: u32,
    pub m_direct_mapping: u32,

    /// Audio object type
    pub m_extension_audio_object_type: AudioObjectType,
    /// Samplerate
    pub m_extension_sampling_frequency: u32,

    /// Channel configuration index
    pub m_channel_configuration: i8,

    /// Error protection index
    pub m_ep_config: i8,
    /// aacSectionDataResilienceFlag
    pub m_vcb11_flag: i8,
    /// aacScalefactorDataResilienceFlag
    pub m_rvlc_flag: i8,
    /// aacSpectralDataResilienceFlag
    pub m_hcr_flag: i8,

    /// Flag indicating the presence of SBR data in the bitstream
    pub m_sbr_present_flag: i8,
    /// Flag indicating the presence of parametric stereo data in the bitstream
    pub m_ps_present_flag: i8,
    /// Samplerate index
    pub m_sampling_frequency_index: u8,
    /// Samplerate index
    pub m_extension_sampling_frequency_index: u8,
    /// Channel configuration index
    pub m_extension_channel_configuration: i8,

    /// The flag indicates if the callback shall work in memory allocation mode or in config
    /// change detection mode
    pub config_mode: u8,
    /// Set if at least one aac config parameter has changed that requires a memory
    /// reconfiguration (`AacConfigChanged`).
    pub aac_config_changed: u8,
    /// Set if at least one sbr config parameter has changed that requires a memory
    /// reconfiguration (`SbrConfigChanged`).
    pub sbr_config_changed: u8,
    /// Set if at least one sac config parameter has changed that requires a memory
    /// reconfiguration (`SacConfigChanged`).
    pub sac_config_changed: u8,
}

impl Default for CSAudioSpecificConfig {
    /// All zero, like `FDKmemclear()` (use `audio_specific_config_init()` for the initial values).
    fn default() -> Self {
        Self {
            m_sc: CSUsacConfig::default(),
            m_aot: 0,
            m_sampling_frequency: 0,
            m_samples_per_frame: 0,
            m_direct_mapping: 0,
            m_extension_audio_object_type: 0,
            m_extension_sampling_frequency: 0,
            m_channel_configuration: 0,
            m_ep_config: 0,
            m_vcb11_flag: 0,
            m_rvlc_flag: 0,
            m_hcr_flag: 0,
            m_sbr_present_flag: 0,
            m_ps_present_flag: 0,
            m_sampling_frequency_index: 0,
            m_extension_sampling_frequency_index: 0,
            m_extension_channel_configuration: 0,
            config_mode: 0,
            aac_config_changed: 0,
            sbr_config_changed: 0,
            sac_config_changed: 0,
        }
    }
}

/// `CCtrlCFGChange`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CCtrlCFGChange {
    /// MHAS config
    pub mpegh3da_config: [u8; TP_MPEGH3DA_MAX_CONFIG_LEN],
    /// MHAS config length
    pub mpegh3da_config_len: u32,
    /// Flush frame counter
    pub flush_cnt: i8,
    /// Flag indicates flush mode: on|off
    pub flush_status: u8,
    /// Build up frame counter
    pub build_up_cnt: i8,
    /// Flag indicates build up mode: on|off
    pub build_up_status: u8,
    /// Flag indicates that the config changed and the decoder needs to be initialized again via
    /// callback. Make sure that memory is freed before initialization.
    pub cfg_changed: u8,
    /// Flag indicates that the content changed i.e. a right truncation occured before
    pub content_changed: u8,
    /// Flag indicates if config change has to be forced even if new config is the same
    pub force_cfg_change: u8,
    /// Flag indicates if truncation message was received
    pub truncation_present: u8,
    pub force_crossfade: u8,
}

impl Default for CCtrlCFGChange {
    fn default() -> Self {
        Self {
            mpegh3da_config: [0; TP_MPEGH3DA_MAX_CONFIG_LEN],
            mpegh3da_config_len: 0,
            flush_cnt: 0,
            flush_status: 0,
            build_up_cnt: 0,
            build_up_status: 0,
            cfg_changed: 0,
            content_changed: 0,
            force_cfg_change: 0,
            truncation_present: 0,
            force_crossfade: 0,
        }
    }
}

impl CCtrlCFGChange {
    /// Reads byte `idx` of `Mpegh3daConfig` like C does with an unchecked index: indices past the
    /// end of the array address the (little endian) bytes of the following `Mpegh3daConfigLen`.
    pub(crate) fn config_byte(&self, idx: usize) -> u8 {
        if idx < TP_MPEGH3DA_MAX_CONFIG_LEN {
            self.mpegh3da_config[idx]
        } else {
            self.mpegh3da_config_len.to_le_bytes()[idx - TP_MPEGH3DA_MAX_CONFIG_LEN]
        }
    }

    /// Writes byte `idx` of `Mpegh3daConfig`, see [`CCtrlCFGChange::config_byte`].
    pub(crate) fn set_config_byte(&mut self, idx: usize, v: u8) {
        if idx < TP_MPEGH3DA_MAX_CONFIG_LEN {
            self.mpegh3da_config[idx] = v;
        } else {
            let mut b = self.mpegh3da_config_len.to_le_bytes();
            b[idx - TP_MPEGH3DA_MAX_CONFIG_LEN] = v;
            self.mpegh3da_config_len = u32::from_le_bytes(b);
        }
    }
}

pub const ASI_MAX_LIMIT: usize = 28;
pub const ASI_MAX_GROUP_PRESETS: usize = 16;
pub const ASI_MAX_DESCRIPTION_LANGUAGES: usize = 8;

pub const ASI_MAX_GROUPS: usize = ASI_MAX_LIMIT;
pub const ASI_MAX_SWITCH_GROUPS: usize = ASI_MAX_LIMIT / 2;
pub const ASI_MAX_TARGET_LOUDNESS_CONDITIONS: usize = ASI_MAX_LIMIT / 2;
pub const ASI_MAX_GROUP_MEMBERS: usize = ASI_MAX_LIMIT;
pub const ASI_MAX_SWITCH_GROUP_MEMBERS: usize = ASI_MAX_LIMIT;
pub const ASI_MAX_GROUP_PRESET_CONDITIONS: usize = 16;
pub const ASI_MAX_COMPOSITE_PAIRS: usize = ASI_MAX_LIMIT;
pub const ASI_MAX_PREF_DESCR_LANGUAGES: usize = 10;
pub const ASI_MAX_DESCRIPTION_BLOCKS: usize =
    ASI_MAX_GROUPS + ASI_MAX_SWITCH_GROUPS + ASI_MAX_GROUP_PRESETS;
/// max(mae_bsDescriptionDataLength) + 1
pub const ASI_MAX_DESCRIPTION_LEN: usize = 256;
pub const ASI_MAX_STORED_DESCRIPTION_LEN: usize = 64;

pub const ASI_DIFF_DESCRIPTION: u32 = 1;
pub const ASI_DIFF_CONTENT: u32 = 2;
pub const ASI_DIFF_GROUP: u32 = 4;
pub const ASI_DIFF_SWITCHGRP: u32 = 8;
pub const ASI_DIFF_GRPPRESET: u32 = 16;
pub const ASI_DIFF_NEEDS_RESET: u32 = 32;
pub const ASI_DIFF_LCOMP: u32 = 64;
pub const ASI_DIFF_SCRNSIZE: u32 = 128;
pub const ASI_DIFF_DRC_INFO: u32 = 256;
pub const ASI_DIFF_AVAILABILITY: u32 = 512;

/// `ASI_DESCRIPTION` (variant with `ASI_MAX_DESCRIPTION_LANGUAGES` defined). `char` is signed on
/// the reference platform, hence `i8`.
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct AsiDescription {
    pub present: u8,
    pub num_languages: i16,
    pub pref_lang_idx: i16,
    pub language: [[i8; 3]; ASI_MAX_DESCRIPTION_LANGUAGES],
    pub data: [[i8; ASI_MAX_STORED_DESCRIPTION_LEN + 1]; ASI_MAX_DESCRIPTION_LANGUAGES],
}

impl Default for AsiDescription {
    fn default() -> Self {
        Self {
            present: 0,
            num_languages: 0,
            pref_lang_idx: 0,
            language: [[0; 3]; ASI_MAX_DESCRIPTION_LANGUAGES],
            data: [[0; ASI_MAX_STORED_DESCRIPTION_LEN + 1]; ASI_MAX_DESCRIPTION_LANGUAGES],
        }
    }
}

/// `ASI_CONTENT_DATA`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiContentData {
    pub content_kind: u8,
    pub content_language: [i8; 3],
}

/// `ASI_PRODUCTION_SCREEN_SIZE_DATA`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiProductionScreenSizeData {
    pub has_non_standard_screen_size: u8,
    pub is_centered_in_azimuth: u8,
    pub bs_screen_size_az: i16,
    pub bs_screen_size_left_az: i16,
    pub bs_screen_size_right_az: i16,
    pub bs_screen_size_top_el: i16,
    pub bs_screen_size_bottom_el: i16,
}

/// `ASI_EXCLUSION_SECTOR`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiExclusionSector {
    pub use_predefined_sector: u8,
    pub exclude_sector_index: u8,
    pub exclude_sector_min_azimuth: u8,
    pub exclude_sector_max_azimuth: u8,
    pub exclude_sector_min_elevation: u8,
    pub exclude_sector_max_elevation: u8,
}

/// `ASI_GROUP`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiGroup {
    pub group_id: u8,
    pub allow_on_off: u8,
    pub default_on_off: u8,

    pub allow_position_interactivity: u8,
    pub interactivity_min_az_offset: u8,
    pub interactivity_max_az_offset: u8,
    pub interactivity_min_el_offset: u8,
    pub interactivity_max_el_offset: u8,
    pub interactivity_min_dist_factor: u8,
    pub interactivity_max_dist_factor: u8,

    pub allow_gain_interactivity: u8,
    pub interactivity_min_gain: u8,
    pub interactivity_max_gain: u8,

    pub is_available: u8,
    pub priority: u8,
    pub closest_speaker_playout: u8,

    pub num_members: u8,
    pub has_conjunct_members: u8,

    pub start_id: u8,
    pub meta_data_element_id: [u8; ASI_MAX_GROUP_MEMBERS],

    // extension
    pub ext_present: u8,

    // content data
    pub cont_present: u8,
    pub content_data: AsiContentData,

    /// ID of switch group this group is member of (255 for none)
    pub switch_group_id: u8,
}

/// `ASI_SWITCH_GROUP`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiSwitchGroup {
    pub switch_group_id: u8,
    pub allow_on_off: u8,
    pub default_on_off: u8,

    pub num_members: u8,
    pub member_id: [u8; ASI_MAX_SWITCH_GROUP_MEMBERS],

    pub default_group_id: u8,
}

/// `ASI_GROUP_PRESET_CONDITION`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiGroupPresetCondition {
    pub reference_id: u8,
    pub condition_on_off: u8,

    pub disable_gain_interactivity: u8,
    pub gain_flag: u8,
    pub gain: u8,

    pub disable_position_interactivity: u8,
    pub position_flag: u8,
    pub az_offset: u8,
    pub el_offset: u8,
    pub dist_factor: u8,

    // extension
    pub is_switch_group_condition: u8,
}

/// `ASI_DMX_ID_GROUP_PRESET_EXT`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiDmxIdGroupPresetExt {
    pub downmix_id: u8,
    pub num_conditions: u8,
    pub conditions: [AsiGroupPresetCondition; ASI_MAX_GROUP_PRESET_CONDITIONS],
}

/// `ASI_GROUP_PRESET`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiGroupPreset {
    pub group_preset_id: u8,
    pub kind: u8,

    pub num_conditions: u8,
    pub conditions: [AsiGroupPresetCondition; ASI_MAX_GROUP_PRESET_CONDITIONS],

    // extension
    pub ext_present: u8,

    pub has_downmix_id_extension: u8,
    pub downmix_id_extension: AsiDmxIdGroupPresetExt,

    // production screen size
    pub production_screen_size: AsiProductionScreenSizeData,
}

/// `ASI_LOUDNESS_COMPENSATION_DATA`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiLoudnessCompensationData {
    pub group_loudness_present: u8,
    pub group_loudness: [u8; ASI_MAX_GROUPS],

    pub default_params_present: u8,
    pub default_include_group: [u8; ASI_MAX_GROUPS],

    pub default_min_max_gain_present: u8,
    pub default_min_gain: u8,
    pub default_max_gain: u8,

    pub preset_params_present: [u8; ASI_MAX_GROUP_PRESETS],
    pub preset_include_group: [[u8; ASI_MAX_GROUPS]; ASI_MAX_GROUP_PRESETS],

    pub preset_min_max_gain_present: [u8; ASI_MAX_GROUP_PRESETS],
    pub preset_min_gain: [u8; ASI_MAX_GROUP_PRESETS],
    pub preset_max_gain: [u8; ASI_MAX_GROUP_PRESETS],
}

/// `ASI_DRC_UI_INFO`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsiDrcUiInfo {
    pub num_target_loudness_conditions: u8,
    pub bs_target_loudness_value_upper: [u8; ASI_MAX_TARGET_LOUDNESS_CONDITIONS],
    pub drc_set_effect_available: [u16; ASI_MAX_TARGET_LOUDNESS_CONDITIONS],
}

/// `ASI_DESCRIPTIONS`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct AsiDescriptions {
    pub groups: [AsiDescription; ASI_MAX_GROUPS],
    pub switch_groups: [AsiDescription; ASI_MAX_SWITCH_GROUPS],
    pub group_presets: [AsiDescription; ASI_MAX_GROUP_PRESETS],
}

impl Default for AsiDescriptions {
    fn default() -> Self {
        Self {
            groups: [AsiDescription::default(); ASI_MAX_GROUPS],
            switch_groups: [AsiDescription::default(); ASI_MAX_SWITCH_GROUPS],
            group_presets: [AsiDescription::default(); ASI_MAX_GROUP_PRESETS],
        }
    }
}

/// `AUDIO_SCENE_INFO`
///
/// The C struct holds a pointer to separately allocated descriptions (`pDescriptions`, allocated
/// by the UI manager); here the descriptions are owned by the struct.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct AudioSceneInfo {
    pub is_main_stream: [u8; 16],
    pub audio_scene_info_id: u8,

    pub num_groups: u8,
    pub groups: [AsiGroup; ASI_MAX_GROUPS],

    pub num_switch_groups: u8,
    pub switch_groups: [AsiSwitchGroup; ASI_MAX_SWITCH_GROUPS],

    pub num_group_presets: u8,
    pub group_presets: [AsiGroupPreset; ASI_MAX_GROUP_PRESETS],

    /// `pDescriptions` (`None` for a `NULL` pointer)
    pub p_descriptions: Option<Box<AsiDescriptions>>,

    pub meta_data_element_id_offset: [u8; 16],
    /// the maximum available metaDataElementID in a Main Stream or Sub-Stream. -1 means no IDs.
    pub meta_data_element_id_max_avail: [i16; 16],

    pub comp_pairs_present: u8,
    pub num_composite_pairs: u8,
    pub composite_pairs: [[u8; 2]; ASI_MAX_COMPOSITE_PAIRS],

    pub scrn_size_present: u8,
    pub scrn_size_ext_present: u8,
    pub production_screen_size_data: AsiProductionScreenSizeData,

    pub loud_comp_present: u8,
    pub loudness_compensation_data: AsiLoudnessCompensationData,

    pub drc_ui_info_present: u8,
    pub drc_ui_info: AsiDrcUiInfo,

    pub scene_disp_present: u8,

    pub diff_flags: u32,

    pub pref_descr_languages: [[i8; 3]; ASI_MAX_PREF_DESCR_LANGUAGES],
    pub active_dmx_id: u8,

    pub crc_for_uid: u16,
    pub crc_for_compare: u16,
}

impl Default for AudioSceneInfo {
    /// All zero with `pDescriptions == NULL`.
    fn default() -> Self {
        Self {
            is_main_stream: [0; 16],
            audio_scene_info_id: 0,
            num_groups: 0,
            groups: [AsiGroup::default(); ASI_MAX_GROUPS],
            num_switch_groups: 0,
            switch_groups: [AsiSwitchGroup::default(); ASI_MAX_SWITCH_GROUPS],
            num_group_presets: 0,
            group_presets: [AsiGroupPreset::default(); ASI_MAX_GROUP_PRESETS],
            p_descriptions: None,
            meta_data_element_id_offset: [0; 16],
            meta_data_element_id_max_avail: [0; 16],
            comp_pairs_present: 0,
            num_composite_pairs: 0,
            composite_pairs: [[0; 2]; ASI_MAX_COMPOSITE_PAIRS],
            scrn_size_present: 0,
            scrn_size_ext_present: 0,
            production_screen_size_data: AsiProductionScreenSizeData::default(),
            loud_comp_present: 0,
            loudness_compensation_data: AsiLoudnessCompensationData::default(),
            drc_ui_info_present: 0,
            drc_ui_info: AsiDrcUiInfo::default(),
            scene_disp_present: 0,
            diff_flags: 0,
            pref_descr_languages: [[0; 3]; ASI_MAX_PREF_DESCR_LANGUAGES],
            active_dmx_id: 0,
            crc_for_uid: 0,
            crc_for_compare: 0,
        }
    }
}

pub const EARCON_MAX_NUM_SIGNALS: u32 = 2;
pub const EARCON_NUM_LANGUAGES: u32 = 1 << 4;
/// Buffer size = 2 (stereo Earcon) * (3 (maximum resampling ratio for MPEGH) * 1024 (MPEGH base
/// length)) + 775 (delayed samples before the limiter))
pub const EARCON_BUFFER_SIZE: usize = EARCON_MAX_NUM_SIGNALS as usize * (3 * 1024 + 775);

/// `EarconInfo`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EarconInfo {
    pub m_num_earcons: u32,
}

/// `EarconConfig`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EarconConfig {
    pub m_num_pcm_signals: u32,
    pub m_bs_pcm_frame_size_index: u32,
    pub m_bs_pcm_frame_size: u32,
    pub m_bs_pcm_loudness_value: u32,
    pub m_pcm_has_attenuation_gain: u32,
    pub m_bs_pcm_attenuation_gain: u32,
    pub m_bs_pcm_truncation_length: u32,
    pub m_bs_mhas_truncation_length: u32,
    pub sampling_frequency: u32,
    pub truncation_flag: u32,
    pub earcon_flag: u32,
    pub earcon_parse_error: u32,
}

/// Callback interface of the transport decoder to the core decoder: the Rust counterpart of
/// `CSTpCallBacks` together with the two pointers the transport decoder keeps into core decoder
/// memory (`pASI` from `transportDec_SetAsiParsing()` and `pLoudnessInfoSetPosition` from
/// `transportDec_RegisterUniDrcConfigCallback()`).
///
/// The C callbacks receive a `void*` user data pointer as first argument; here that is `self`.
/// Integer return values keep their C meaning (0 = success, otherwise a `TRANSPORTDEC_ERROR`
/// code). `cbDecodeFrame` is registered by the core decoder but never called by the transport
/// decoder, so it has no counterpart.
///
/// The C code checks some callback pointers for `NULL` before calling them; the `has_*` methods
/// answer these checks ("callback registered"). The core decoder registers all callbacks, so
/// the defaults return `true`. Callbacks that C calls without a check must be implemented.
pub trait TpCallbacks {
    /// `cbUpdateConfig(self, pAsc, configMode, configChanged)`: (re)configure the decoder for
    /// `pAsc = &ascs[index]`. `config_mode` is a combination of `AC_CM_*` flags;
    /// `config_changed` is in/out.
    ///
    /// `ascs` are all configs of the transport decoder (`hTp->asc[]`, including the temporary
    /// one at `TPDEC_MAX_TRACKS`): the core decoder keeps pointers to them (`pUsacConfig[]`) and
    /// reads the configs of the other substreams during the callback.
    ///
    /// In C, `configChanged` may point into `*pAsc` (`&asc->AacConfigChanged`); the Rust caller
    /// passes a copy of that field and stores the result back after the call.
    fn update_config(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        index: usize,
        config_mode: u8,
        config_changed: &mut u8,
    ) -> i32;

    /// `cbFreeMem(self, pAsc)`: free the config dependent memory of the (sub)stream described by
    /// `pAsc = &ascs[index]` (`sub_stream_index`, `config_mode`); see
    /// [`TpCallbacks::update_config`] for `ascs`.
    fn free_mem(&mut self, ascs: &[CSAudioSpecificConfig], index: usize) -> i32;

    /// Transport decoder parameters (`transportDec_SetParam()`) that the core decoder set while
    /// the transport decoder was busy calling [`TpCallbacks::update_config`] or
    /// [`TpCallbacks::free_mem`]. The transport decoder applies them right after these callbacks
    /// return (the C decoder calls `transportDec_SetParam()` from within the callback; the
    /// parameters are not read during the callback).
    fn take_pending_param(&mut self) -> Option<(i32, i32)> {
        None
    }

    /// `cbCtrlCFGChange(self, pCtrlCFGChangeStruct)`: config change (flush / build up) control.
    fn ctrl_cfg_change(&mut self, ctrl: &CCtrlCFGChange) -> i32;
    /// `cbCtrlCFGChange != NULL`
    fn has_ctrl_cfg_change(&self) -> bool {
        true
    }

    /// `cbTruncationMsg(self, nTruncSamples, truncType)`: truncation type 0 = right, 1 = left,
    /// 2 = implicit truncation for a config change without truncation message.
    fn truncation_msg(&mut self, n_trunc_samples: i32, trunc_type: i32) -> i32;
    /// `cbTruncationMsg != NULL`
    fn has_truncation_msg(&self) -> bool {
        true
    }

    /// `cbUniDrc(self, hBs, fullPayloadLength, payloadType, subStreamIndex, payloadStart, aot)`:
    /// parse a uniDrcConfig (`payload_type == 0`) or loudnessInfoSet (`payload_type == 1`) from
    /// `bs`. `bs == None` (`NULL`) clears the respective data (`payload_type == 2`: clear the
    /// downmix information). `payload_start` is the bit position (valid bits) of the
    /// configuration extension header of a loudnessInfoSet within mpegh3daConfig.
    fn uni_drc(
        &mut self,
        bs: Option<&mut FdkBitStream>,
        full_payload_length: i32,
        payload_type: i32,
        sub_stream_index: i32,
        payload_start: i32,
        aot: AudioObjectType,
    ) -> i32;
    /// `cbUniDrc != NULL`
    fn has_uni_drc(&self) -> bool {
        true
    }

    /// `cbUserInteract(self, hBs, type, MhasPacketLength)`: parse a user interaction, loudness
    /// DRC interface or ID marker packet.
    fn user_interact(
        &mut self,
        bs: &mut FdkBitStream,
        packet_type: MhaPactyp,
        mhas_packet_length: u32,
    ) -> i32;
    /// `cbUserInteract != NULL`
    fn has_user_interact(&self) -> bool {
        true
    }

    /// `cbParseDmxMatrix(self, hBs, usc)`: parse the downmix matrix set of `downmixConfig()`;
    /// may set `usc.m_signal_group_type[].b_use_custom_downmix_matrix`.
    fn parse_dmx_matrix(&mut self, bs: &mut FdkBitStream, usc: &mut CSUsacConfig) -> i32;

    /// `cbEarconBS(self, bs)`: earcon PCM data (or the frame bitstream for state handling).
    fn earcon_bs(&mut self, bs: &mut FdkBitStream) -> i32;
    /// `cbEarconBS != NULL` (also used for the `cbEarconBSData != NULL` check)
    fn has_earcon_bs(&self) -> bool {
        true
    }

    /// `cbEarconConfig(self, cf)`
    fn earcon_config(&mut self, config: &mut EarconConfig) -> i32;
    /// `cbEarconConfig != NULL` (also used for the `cbEarconConfigData != NULL` check)
    fn has_earcon_config(&self) -> bool {
        true
    }

    /// `cbEarconInfo(self, info)`
    fn earcon_info(&mut self, info: &mut EarconInfo) -> i32;
    /// `cbEarconInfo != NULL`
    fn has_earcon_info(&self) -> bool {
        true
    }

    /// `hTp->pASI`: the audio scene info to fill (`None` if no ASI parsing was requested with
    /// `transportDec_SetAsiParsing()`). Called whenever the ASI is accessed; the returned
    /// reference is not held across other callback calls.
    fn asi(&mut self) -> Option<&mut AudioSceneInfo>;

    /// `hTp->pLoudnessInfoSetPosition` (`INT[3]`): reference and start position (bits) and
    /// length (bytes) of the loudnessInfoSet within mpegh3daConfig. Written by the transport
    /// decoder (`[0]`, `[1]` and `[2]` at the start of each main stream config) and by the core in
    /// `uni_drc()` (`[1]` and `[2]`).
    fn loudness_info_set_position(&mut self) -> &mut [i32; 3];
}

/// `SamplingRateTable`
pub static SAMPLING_RATE_TABLE: [u32; 32] = [
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350, 0, 0,
    57600, 51200, 40000, 38400, 34150, 28800, 25600, 20000, 19200, 17075, 14400, 12800, 9600, 0, 0,
    0, 0,
];
