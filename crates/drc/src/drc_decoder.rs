// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDecoder.h.

//! Definitions used in all submodules of the DRC decoder.

use mpeghdec_fdk::{fl2fxconst_dbl, fl2fxconst_sgl, FixpDbl, FixpSgl};

/// `MAX_ACTIVE_DRCS`: maximum number of DRC sets applied simultaneously, see section 6.3.5 of
/// ISO/IEC 23003-4.
pub const MAX_ACTIVE_DRCS: usize = 3;
/// `ACTIVE_DRC_LOCATIONS`
pub const ACTIVE_DRC_LOCATIONS: usize = 2;

/// `DELAY_MODE`
pub type DelayMode = i32;
/// `DM_REGULAR_DELAY`
pub const DM_REGULAR_DELAY: DelayMode = 0;
/// `DM_LOW_DELAY`
pub const DM_LOW_DELAY: DelayMode = 1;

/// `DRC_ERROR`
pub type DrcError = i32;
/// `DE_OK`
pub const DE_OK: DrcError = 0;
/// `DE_NOT_OK`
pub const DE_NOT_OK: DrcError = -100;
/// `DE_PARAM_OUT_OF_RANGE`
pub const DE_PARAM_OUT_OF_RANGE: DrcError = -99;
/// `DE_PARAM_INVALID`
pub const DE_PARAM_INVALID: DrcError = -98;
/// `DE_MEMORY_ERROR`
pub const DE_MEMORY_ERROR: DrcError = -97;

/// `SUBBAND_DOMAIN_MODE`
pub type SubbandDomainMode = i32;
/// `SDM_OFF`
pub const SDM_OFF: SubbandDomainMode = 0;
/// `SDM_QMF64`
pub const SDM_QMF64: SubbandDomainMode = 1;
/// `SDM_QMF71`
pub const SDM_QMF71: SubbandDomainMode = 2;
/// `SDM_STFT256`
pub const SDM_STFT256: SubbandDomainMode = 3;

/// `DOWNMIX_ID_BASE_LAYOUT`
pub const DOWNMIX_ID_BASE_LAYOUT: i32 = 0x0;
/// `DOWNMIX_ID_ANY_DOWNMIX`
pub const DOWNMIX_ID_ANY_DOWNMIX: i32 = 0x7F;
/// `DRC_SET_ID_NO_DRC`
pub const DRC_SET_ID_NO_DRC: i32 = 0x0;
/// `DRC_SET_ID_ANY_DRC`
pub const DRC_SET_ID_ANY_DRC: i32 = 0x3F;

/// `LOCATION_MP4_INSTREAM_UNIDRC`
pub const LOCATION_MP4_INSTREAM_UNIDRC: i32 = 0x1;
/// `LOCATION_MP4_DYN_RANGE_INFO`
pub const LOCATION_MP4_DYN_RANGE_INFO: i32 = 0x2;
/// `LOCATION_MP4_COMPRESSION_VALUE`
pub const LOCATION_MP4_COMPRESSION_VALUE: i32 = 0x3;
/// `LOCATION_SELECTED`: set to location selected by system
pub const LOCATION_SELECTED: i32 = LOCATION_MP4_INSTREAM_UNIDRC;

/// `MAX_REQUESTS_DOWNMIX_ID`
pub const MAX_REQUESTS_DOWNMIX_ID: usize = 15;
/// `MAX_REQUESTS_DRC_FEATURE`
pub const MAX_REQUESTS_DRC_FEATURE: usize = 7;
/// `MAX_REQUESTS_DRC_EFFECT_TYPE`
pub const MAX_REQUESTS_DRC_EFFECT_TYPE: usize = 15;

/// `DEFAULT_LOUDNESS_DEVIATION_MAX`
pub const DEFAULT_LOUDNESS_DEVIATION_MAX: u8 = 63;

/// `DRC_INPUT_LOUDNESS_TARGET`
pub const DRC_INPUT_LOUDNESS_TARGET: FixpDbl = fl2fxconst_dbl((-31.0f32 / 128.0f32) as f64);
/// `DRC_INPUT_LOUDNESS_TARGET_SGL`
pub const DRC_INPUT_LOUDNESS_TARGET_SGL: FixpSgl = fl2fxconst_sgl((-31.0f32 / 128.0f32) as f64);
