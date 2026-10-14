// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libSYS/include/FDK_audio.h.

//! Common audio codec type definitions.
//!
//! C enums are mapped to `i32` type aliases with associated constants, because the C code stores,
//! compares and computes with them as plain integers.

/// `TRANSPORT_TYPE`
pub type TransportType = i32;
pub const TT_UNKNOWN: TransportType = -1;
pub const TT_MP4_RAW: TransportType = 0;
pub const TT_MP4_ADIF: TransportType = 1;
pub const TT_MP4_ADTS: TransportType = 2;
pub const TT_MP4_LATM_MCP1: TransportType = 6;
pub const TT_MP4_LATM_MCP0: TransportType = 7;
pub const TT_MP4_LOAS: TransportType = 10;
pub const TT_DRM: TransportType = 12;
pub const TT_MHAS: TransportType = 20;
pub const TT_MHAS_PACKETIZED: TransportType = 24;
pub const TT_MHA_RAW: TransportType = 60;

/// `AUDIO_OBJECT_TYPE`
pub type AudioObjectType = i32;
pub const AOT_NONE: AudioObjectType = -1;
pub const AOT_NULL_OBJECT: AudioObjectType = 0;
pub const AOT_AAC_MAIN: AudioObjectType = 1;
pub const AOT_AAC_LC: AudioObjectType = 2;
pub const AOT_AAC_SSR: AudioObjectType = 3;
pub const AOT_AAC_LTP: AudioObjectType = 4;
pub const AOT_SBR: AudioObjectType = 5;
pub const AOT_AAC_SCAL: AudioObjectType = 6;
pub const AOT_TWIN_VQ: AudioObjectType = 7;
pub const AOT_CELP: AudioObjectType = 8;
pub const AOT_HVXC: AudioObjectType = 9;
pub const AOT_RSVD_10: AudioObjectType = 10;
pub const AOT_RSVD_11: AudioObjectType = 11;
pub const AOT_TTSI: AudioObjectType = 12;
pub const AOT_MAIN_SYNTH: AudioObjectType = 13;
pub const AOT_WAV_TAB_SYNTH: AudioObjectType = 14;
pub const AOT_GEN_MIDI: AudioObjectType = 15;
pub const AOT_ALG_SYNTH_AUD_FX: AudioObjectType = 16;
pub const AOT_ER_AAC_LC: AudioObjectType = 17;
pub const AOT_RSVD_18: AudioObjectType = 18;
pub const AOT_ER_AAC_LTP: AudioObjectType = 19;
pub const AOT_ER_AAC_SCAL: AudioObjectType = 20;
pub const AOT_ER_TWIN_VQ: AudioObjectType = 21;
pub const AOT_ER_BSAC: AudioObjectType = 22;
pub const AOT_ER_AAC_LD: AudioObjectType = 23;
pub const AOT_ER_CELP: AudioObjectType = 24;
pub const AOT_ER_HVXC: AudioObjectType = 25;
pub const AOT_ER_HILN: AudioObjectType = 26;
pub const AOT_ER_PARA: AudioObjectType = 27;
pub const AOT_RSVD_28: AudioObjectType = 28;
pub const AOT_PS: AudioObjectType = 29;
pub const AOT_MPEGS: AudioObjectType = 30;
pub const AOT_ESCAPE: AudioObjectType = 31;
pub const AOT_MP3ONMP4_L1: AudioObjectType = 32;
pub const AOT_MP3ONMP4_L2: AudioObjectType = 33;
pub const AOT_MP3ONMP4_L3: AudioObjectType = 34;
pub const AOT_RSVD_35: AudioObjectType = 35;
pub const AOT_RSVD_36: AudioObjectType = 36;
pub const AOT_AAC_SLS: AudioObjectType = 37;
pub const AOT_SLS: AudioObjectType = 38;
pub const AOT_ER_AAC_ELD: AudioObjectType = 39;
pub const AOT_USAC: AudioObjectType = 42;
pub const AOT_SAOC: AudioObjectType = 43;
pub const AOT_LD_MPEGS: AudioObjectType = 44;
pub const AOT_MP2_AAC_LC: AudioObjectType = 129;
pub const AOT_MP2_SBR: AudioObjectType = 132;
pub const AOT_DRM_AAC: AudioObjectType = 143;
pub const AOT_DRM_SBR: AudioObjectType = 144;
pub const AOT_DRM_MPEG_PS: AudioObjectType = 145;
pub const AOT_DRM_SURROUND: AudioObjectType = 146;
pub const AOT_DRM_USAC: AudioObjectType = 147;
pub const AOT_MPEGH3DA: AudioObjectType = 150;
pub const AOT_MPEGD_RESIDUALS: AudioObjectType = 256;

/// `CHANNEL_ORDER`
pub type ChannelOrder = i32;
pub const CH_ORDER_MPEG: ChannelOrder = 0;
pub const CH_ORDER_WAV: ChannelOrder = 1;
pub const CH_ORDER_CICP: ChannelOrder = 2;

/// `CHANNEL_MODE`
pub type ChannelMode = i32;
pub const MODE_INVALID: ChannelMode = -1;
pub const MODE_UNKNOWN: ChannelMode = 0;
pub const MODE_1: ChannelMode = 1;
pub const MODE_2: ChannelMode = 2;
pub const MODE_1_2: ChannelMode = 3;
pub const MODE_1_2_1: ChannelMode = 4;
pub const MODE_1_2_2: ChannelMode = 5;
pub const MODE_1_2_2_1: ChannelMode = 6;
pub const MODE_1_2_2_2_1: ChannelMode = 7;
pub const MODE_6_1: ChannelMode = 11;
pub const MODE_7_1_BACK: ChannelMode = 12;
pub const MODE_22_2: ChannelMode = 13;
pub const MODE_7_1_TOP_FRONT: ChannelMode = 14;
pub const MODE_1_1: ChannelMode = 16;
pub const MODE_1_1_1_1: ChannelMode = 17;
pub const MODE_1_1_1_1_1_1: ChannelMode = 18;
pub const MODE_1_1_1_1_1_1_1_1: ChannelMode = 19;
pub const MODE_1_1_1_1_1_1_1_1_1_1_1_1: ChannelMode = 20;
pub const MODE_2_2: ChannelMode = 21;
pub const MODE_2_2_2: ChannelMode = 22;
pub const MODE_2_2_2_2: ChannelMode = 23;
pub const MODE_2_2_2_2_2_2: ChannelMode = 24;
pub const MODE_2_1: ChannelMode = 30;
pub const MODE_7_1_REAR_SURROUND: ChannelMode = 33;
pub const MODE_7_1_FRONT_CENTER: ChannelMode = 34;
pub const MODE_212: ChannelMode = 128;

/// `AUDIO_CHANNEL_TYPE`
pub type AudioChannelType = i32;
pub const ACT_NONE: AudioChannelType = 0x00;
pub const ACT_FRONT: AudioChannelType = 0x01;
pub const ACT_SIDE: AudioChannelType = 0x02;
pub const ACT_BACK: AudioChannelType = 0x03;
pub const ACT_LFE: AudioChannelType = 0x04;
pub const ACT_TOP: AudioChannelType = 0x10;
pub const ACT_FRONT_TOP: AudioChannelType = 0x11;
pub const ACT_SIDE_TOP: AudioChannelType = 0x12;
pub const ACT_BACK_TOP: AudioChannelType = 0x13;
pub const ACT_BOTTOM: AudioChannelType = 0x20;
pub const ACT_FRONT_BOTTOM: AudioChannelType = 0x21;
pub const ACT_SIDE_BOTTOM: AudioChannelType = 0x22;
pub const ACT_BACK_BOTTOM: AudioChannelType = 0x23;

/// `SBR_PS_SIGNALING`
pub type SbrPsSignaling = i32;
pub const SIG_UNKNOWN: SbrPsSignaling = -1;
pub const SIG_IMPLICIT: SbrPsSignaling = 0;
pub const SIG_EXPLICIT_BW_COMPATIBLE: SbrPsSignaling = 1;
pub const SIG_EXPLICIT_HIERARCHICAL: SbrPsSignaling = 2;

/// `MP4_ELEMENT_ID`
pub type Mp4ElementId = i32;
pub const ID_NONE: Mp4ElementId = -1;
pub const ID_SCE: Mp4ElementId = 0;
pub const ID_CPE: Mp4ElementId = 1;
pub const ID_CCE: Mp4ElementId = 2;
pub const ID_LFE: Mp4ElementId = 3;
pub const ID_DSE: Mp4ElementId = 4;
pub const ID_PCE: Mp4ElementId = 5;
pub const ID_FIL: Mp4ElementId = 6;
pub const ID_END: Mp4ElementId = 7;
pub const ID_EXT: Mp4ElementId = 8;
pub const ID_SCAL: Mp4ElementId = 9;
pub const ID_USAC_SCE: Mp4ElementId = 0 + USAC_ID_BIT;
pub const ID_USAC_CPE: Mp4ElementId = 1 + USAC_ID_BIT;
pub const ID_USAC_LFE: Mp4ElementId = 2 + USAC_ID_BIT;
pub const ID_USAC_EXT: Mp4ElementId = 3 + USAC_ID_BIT;
pub const ID_USAC_END: Mp4ElementId = 4 + USAC_ID_BIT;
pub const ID_LAST: Mp4ElementId = 21;

/// `CONFIG_EXT_ID`
pub type ConfigExtId = i32;
pub const ID_CONFIG_EXT_FILL: ConfigExtId = 0;
pub const ID_CONFIG_EXT_DOWNMIX: ConfigExtId = 1;
pub const ID_CONFIG_EXT_LOUDNESS_INFO: ConfigExtId = 2;
pub const ID_CONFIG_EXT_AUDIOSCENE_INFO: ConfigExtId = 3;
pub const ID_CONFIG_EXT_HOA_MATRIX: ConfigExtId = 4;
pub const ID_CONFIG_EXT_SIG_GROUP_INFO: ConfigExtId = 6;
pub const ID_CONFIG_EXT_COMPATIBLE_PROFILELVL_SET: ConfigExtId = 7;

/// `EXT_PAYLOAD_TYPE`
pub type ExtPayloadType = i32;
pub const EXT_FIL: ExtPayloadType = 0x00;
pub const EXT_FILL_DATA: ExtPayloadType = 0x01;
pub const EXT_DATA_ELEMENT: ExtPayloadType = 0x02;
pub const EXT_DATA_LENGTH: ExtPayloadType = 0x03;
pub const EXT_UNI_DRC: ExtPayloadType = 0x04;
pub const EXT_LDSAC_DATA: ExtPayloadType = 0x09;
pub const EXT_SAOC_DATA: ExtPayloadType = 0x0a;
pub const EXT_DYNAMIC_RANGE: ExtPayloadType = 0x0b;
pub const EXT_SAC_DATA: ExtPayloadType = 0x0c;
pub const EXT_SBR_DATA: ExtPayloadType = 0x0d;
pub const EXT_SBR_DATA_CRC: ExtPayloadType = 0x0e;

/// `USAC_EXT_ELEMENT_TYPE`
pub type UsacExtElementType = i32;
pub const ID_EXT_ELE_FILL: UsacExtElementType = 0x00;
pub const ID_EXT_ELE_MPEGS: UsacExtElementType = 0x01;
pub const ID_EXT_ELE_SAOC: UsacExtElementType = 0x02;
pub const ID_EXT_ELE_AUDIOPREROLL: UsacExtElementType = 0x03;
pub const ID_EXT_ELE_UNI_DRC: UsacExtElementType = 0x04;
pub const ID_EXT_ELE_OBJ_METADATA: UsacExtElementType = 0x05;
pub const ID_EXT_ELE_SAOC_3D: UsacExtElementType = 0x06;
pub const ID_EXT_ELE_HOA: UsacExtElementType = 0x07;
pub const ID_EXT_ELE_FMT_CNVRTR: UsacExtElementType = 0x08;
pub const ID_EXT_ELE_MCT: UsacExtElementType = 0x09;
pub const ID_EXT_ELE_ENHANCED_OBJ_METADATA: UsacExtElementType = 0x0d;
pub const ID_EXT_ELE_PROD_METADATA: UsacExtElementType = 0x0e;
pub const ID_EXT_ELE_VR_METADATA: UsacExtElementType = 0x81;
pub const ID_EXT_ELE_UNKNOWN: UsacExtElementType = 0xFF;

// AC_* / CC_* flags
/// aacSectionDataResilienceFlag flag (from ASC): 1 means use virtual codebooks
pub const AC_ER_VCB11: u32 = 0x000001;
/// aacSpectralDataResilienceFlag flag (from ASC): 1 means use huffman codeword reordering
pub const AC_ER_RVLC: u32 = 0x000002;
/// aacSectionDataResilienceFlag flag (from ASC): 1 means use virtual codebooks
pub const AC_ER_HCR: u32 = 0x000004;
pub const AC_SCALABLE: u32 = 0x000008;
pub const AC_ELD: u32 = 0x000010;
pub const AC_LD: u32 = 0x000020;
pub const AC_ER: u32 = 0x000040;
pub const AC_BSAC: u32 = 0x000080;
pub const AC_USAC: u32 = 0x000100;
pub const AC_MPEGH3DA: u32 = 0x000200;
pub const AC_HDAAC: u32 = 0x000400;
pub const AC_RSVD50: u32 = 0x004000;
pub const AC_SBR_PRESENT: u32 = 0x008000;
pub const AC_SBRCRC: u32 = 0x010000;
pub const AC_PS_PRESENT: u32 = 0x020000;
pub const AC_MPS_PRESENT: u32 = 0x040000;
pub const AC_DRM: u32 = 0x080000;
pub const AC_INDEP: u32 = 0x100000;
pub const AC_MPEGD_RES: u32 = 0x200000;
pub const AC_SAOC_PRESENT: u32 = 0x400000;
pub const AC_DAB: u32 = 0x800000;
pub const AC_ELD_DOWNSCALE: u32 = 0x1000000;
pub const AC_LD_MPS: u32 = 0x2000000;
pub const AC_DRC_PRESENT: u32 = 0x4000000;
pub const AC_USAC_SCFGI3: u32 = 0x8000000;
// Audio Codec flags (reconfiguration)
/// Config mode signalizes the callback to work in config change detection mode
pub const AC_CM_DET_CFG_CHANGE: u8 = 0x000001;
/// Config mode signalizes the callback to work in memory allocation mode
pub const AC_CM_ALLOC_MEM: u8 = 0x000002;
/// Flag signalizes the callback that this is the last substream
pub const AC_CM_LAST_SUBSTREAM: u8 = 0x000004;
pub const AC_EL_GA_CCE: u32 = 0x000001;
pub const AC_EL_USAC_NOISE: u32 = 0x000002;
pub const AC_EL_USAC_ITES: u32 = 0x000004;
pub const AC_EL_USAC_PVC: u32 = 0x000008;
pub const AC_EL_USAC_MPS212: u32 = 0x000010;
pub const AC_EL_USAC_LFE: u32 = 0x000020;
/// USAC may use Complex Stereo Prediction in this channel element
pub const AC_EL_USAC_CP_POSSIBLE: u32 = 0x000040;
pub const AC_EL_ENHANCED_NOISE: u32 = 0x000080;
pub const AC_EL_IGF_AFTER_TNS: u32 = 0x000100;
pub const AC_EL_IGF_INDEP_TILING: u32 = 0x000200;
pub const AC_EL_IGF_USE_ENF: u32 = 0x000400;
pub const AC_EL_FULLBANDLPD: u32 = 0x000800;
pub const AC_EL_LPDSTEREOIDX: u32 = 0x001000;
pub const AC_EL_LFE: u32 = 0x002000;
pub const CC_MPEG_ID: u32 = 0x00100000;
pub const CC_IS_BASELAYER: u32 = 0x00200000;
pub const CC_PROTECTION: u32 = 0x00400000;
pub const CC_SBR: u32 = 0x00800000;
pub const CC_SBRCRC: u32 = 0x00010000;
pub const CC_SAC: u32 = 0x00020000;
pub const CC_RVLC: u32 = 0x01000000;
pub const CC_VCB11: u32 = 0x02000000;
pub const CC_HCR: u32 = 0x04000000;
pub const CC_PSEUDO_SURROUND: u32 = 0x08000000;
pub const CC_USAC_NOISE: u32 = 0x10000000;
pub const CC_USAC_TW: u32 = 0x20000000;
pub const CC_USAC_HBE: u32 = 0x40000000;

pub const USAC_ID_BIT: i32 = 16;
pub const EXT_ID_BITS: i32 = 4;

/// `TT_IS_PACKET(x)`
#[inline]
pub fn tt_is_packet(x: TransportType) -> bool {
    x == TT_MP4_RAW
        || x == TT_DRM
        || x == TT_MP4_LATM_MCP0
        || x == TT_MP4_LATM_MCP1
        || x == TT_MHA_RAW
        || x == TT_MHAS_PACKETIZED
}

/// `TT_CFG_IS_INBAND(x)`
#[inline]
pub fn tt_cfg_is_inband(x: TransportType) -> bool {
    x == TT_MP4_ADTS
        || x == TT_MP4_ADIF
        || x == TT_MP4_LATM_MCP1
        || x == TT_MHAS
        || x == TT_MHAS_PACKETIZED
}

/// `TT_NEEDS_BS_COPY(x)`
#[inline]
pub fn tt_needs_bs_copy(x: TransportType) -> bool {
    x == TT_MHA_RAW || x == TT_MHAS || x == TT_MHAS_PACKETIZED
}

/// `TT_IS_MPEGH(x)`
#[inline]
pub fn tt_is_mpegh(x: TransportType) -> bool {
    x == TT_MHAS || x == TT_MHAS_PACKETIZED || x == TT_MHA_RAW
}

/// `CAN_DO_PS(aot)`
#[inline]
pub fn can_do_ps(aot: AudioObjectType) -> bool {
    aot == AOT_AAC_LC || aot == AOT_SBR || aot == AOT_PS || aot == AOT_ER_BSAC || aot == AOT_DRM_AAC
}

/// `IS_USAC(aot)` (`AOT_RSVD50` does not exist in this code base).
#[inline]
pub fn is_usac(aot: AudioObjectType) -> bool {
    aot == AOT_USAC || aot == AOT_MPEGH3DA
}

/// `IS_LOWDELAY(aot)`
#[inline]
pub fn is_lowdelay(aot: AudioObjectType) -> bool {
    aot == AOT_ER_AAC_LD || aot == AOT_ER_AAC_ELD
}

/// `IS_CHANNEL_ELEMENT(elementId)`
#[inline]
pub fn is_channel_element(element_id: Mp4ElementId) -> bool {
    element_id == ID_SCE
        || element_id == ID_CPE
        || element_id == ID_LFE
        || element_id == ID_USAC_SCE
        || element_id == ID_USAC_CPE
        || element_id == ID_USAC_LFE
}

/// `IS_MP4_CHANNEL_ELEMENT(elementId)`
#[inline]
pub fn is_mp4_channel_element(element_id: Mp4ElementId) -> bool {
    element_id == ID_SCE || element_id == ID_CPE || element_id == ID_LFE
}

/// `IS_USAC_CHANNEL_ELEMENT(elementId)`
#[inline]
pub fn is_usac_channel_element(element_id: Mp4ElementId) -> bool {
    element_id == ID_USAC_SCE || element_id == ID_USAC_CPE || element_id == ID_USAC_LFE
}
