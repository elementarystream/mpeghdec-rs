// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/channelinfo.h and libMpeghDec/src/channelinfo.cpp.

//! Individual channel stream info: the per channel data structures of the core decoder and the
//! `ics_info()` parser.
//!
//! See the [crate documentation](crate) for how the pointers of the C++ structures are resolved.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::{SFB_OFFSET_TABLES, TNS_MAX_BANDS_TBL};
use crate::aacdec_tns::CTnsData;
use crate::aacdecoder_lib::*;
use crate::conceal_types::CConcealmentInfo;
use crate::ltp_post::{LTP_MEM_IN_SIZE, LTP_MEM_OUT_SIZE};
use crate::stereo::{CCplxPredictionData, CJointStereoData, CJointStereoPersistentData};
use mpeghdec_arith::ac_arith_coder::CArcoData;
use mpeghdec_fdk::audio::{AC_BSAC, AC_ELD, AC_MPEGH3DA, AC_RSVD50, AC_SCALABLE, AC_USAC};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::{FixpDbl, FixpSgl};
use mpeghdec_fdk::mdct::{BlockType, MdctT, BLOCK_SHORT};
use mpeghdec_igf::igf_dec::{IgfPrivateData, IgfPrivateStaticData};

/// `WB_SECTION_SIZE`
pub const WB_SECTION_SIZE: usize = 1024 * 2;

/// `AACDEC_RENDER_MODE`: output rendering mode
pub type AacdecRenderMode = i32;
pub const AACDEC_RENDER_INVALID: AacdecRenderMode = 0;
pub const AACDEC_RENDER_IMDCT: AacdecRenderMode = 1;
pub const AACDEC_RENDER_ELDFB: AacdecRenderMode = 2;
pub const AACDEC_RENDER_LPD: AacdecRenderMode = 3;
pub const AACDEC_RENDER_INTIMDCT: AacdecRenderMode = 4;

/// `MAX_QUANTIZED_VALUE`
pub const MAX_QUANTIZED_VALUE: i32 = 8191;

/// `SamplingRateInfo`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SamplingRateInfo {
    /// `ScaleFactorBands_Long` (empty for `NULL`)
    pub scale_factor_bands_long: &'static [i16],
    /// `ScaleFactorBands_Short` (empty for `NULL`)
    pub scale_factor_bands_short: &'static [i16],
    /// `NumberOfScaleFactorBands_Long`
    pub number_of_scale_factor_bands_long: u8,
    /// `NumberOfScaleFactorBands_Short`
    pub number_of_scale_factor_bands_short: u8,
    /// `samplingRateIndex`
    pub sampling_rate_index: u32,
    /// `samplingRate`
    pub sampling_rate: u32,
}

/// `CRawDataInfo`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CRawDataInfo {
    /// `CommonWindow`
    pub common_window: u8,
    /// `GlobalGain`
    pub global_gain: u8,
}

/// `CIcsInfo`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CIcsInfo {
    /// `WindowGroupLength`
    pub window_group_length: [u8; 8],
    /// `WindowGroups`
    pub window_groups: u8,
    /// `Valid`
    pub valid: u8,
    /// `WindowShape`: 0: sine window, 1: KBD, 2: low overlap
    pub window_shape: u8,
    /// `WindowSequence`: 0: long, 1: start, 2: short, 3: stop
    pub window_sequence: BlockType,
    /// `MaxSfBands`
    pub max_sf_bands: u8,
    /// `max_sfb_ste`
    pub max_sfb_ste: u8,
    /// `ScaleFactorGrouping`
    pub scale_factor_grouping: u8,
    /// `TotalSfBands`
    pub total_sf_bands: u8,
}

/* Huffman codebook numbers */
pub const ZERO_HCB: u8 = 0;
pub const ESCBOOK: u8 = 11;
pub const NSPECBOOKS: u8 = ESCBOOK + 1;
pub const BOOKSCL: u8 = NSPECBOOKS;
pub const NOISE_HCB: u8 = 13;
pub const INTENSITY_HCB2: u8 = 14;
pub const INTENSITY_HCB: u8 = 15;
pub const LAST_HCB: u8 = 16;

/// `CpePersistentData`: the persistent data shared by both channels of a CPE. It needs to be
/// allocated for each CPE (`AAC_DECODER_INSTANCE::cpeStaticData[el]`, only for elements with
/// `AC_EL_USAC_CP_POSSIBLE`).
#[derive(StateDigest, Clone, Debug, Default, PartialEq, Eq)]
pub struct CpePersistentData {
    /// `jointStereoPersistentData`
    pub joint_stereo_persistent_data: CJointStereoPersistentData,
}

/// `CAacDecoderStaticChannelInfo`: persistent data of one channel.
///
/// Differences to the C++ structure (see the [crate documentation](crate)):
/// - `pOverlapBuffer` is owned by `IMdct` ([`MdctT::overlap`]).
/// - `hArCo` is `None` until allocated (`CArco_Create()` for USAC/MPEG-H).
/// - `pCpeStaticData` is not stored; the functions that need it take the
///   [`CpePersistentData`] of the element as an argument.
#[derive(StateDigest, Clone, Debug)]
pub struct CAacDecoderStaticChannelInfo {
    /// `IMdct` (owns `pOverlapBuffer`)
    pub i_mdct: MdctT,
    /// `hArCo`
    pub h_ar_co: Option<Box<CArcoData>>,
    /// `nfRandomSeed`: seed value for USAC noise filling random generator
    pub nf_random_seed: u32,
    /// `concealmentInfo`
    pub concealment_info: CConcealmentInfo,
    /// `IGF_StaticData`
    pub igf_static_data: IgfPrivateStaticData,

    /// `ltp_param`
    pub ltp_param: [i32; 3],
    /// `ltp_pitch_int_past`
    pub ltp_pitch_int_past: i32,
    /// `ltp_pitch_fr_past`
    pub ltp_pitch_fr_past: i32,
    /// `ltp_gain_past`
    pub ltp_gain_past: FixpSgl,
    /// `ltp_gainIdx_past`
    pub ltp_gain_idx_past: i32,
    /// `ltp_mem_in`
    pub ltp_mem_in: [FixpSgl; LTP_MEM_IN_SIZE],
    /// `ltp_mem_out`
    pub ltp_mem_out: [FixpSgl; LTP_MEM_OUT_SIZE],

    /* Frequency Domain Prediction (FDP) tool */
    /// `quantSpecPrev1`
    pub quant_spec_prev1: [i16; 160],
    /// `quantSpecPrev2`
    pub quant_spec_prev2: [i16; 160],
    /// `prevWindowShape`
    pub prev_window_shape: u8,
}

impl Default for CAacDecoderStaticChannelInfo {
    /// Zero-initialized like `GetAacDecoderStaticChannelInfo()` (`FDKcalloc`).
    fn default() -> Self {
        CAacDecoderStaticChannelInfo {
            i_mdct: MdctT::default(),
            h_ar_co: None,
            nf_random_seed: 0,
            concealment_info: CConcealmentInfo::default(),
            igf_static_data: IgfPrivateStaticData::default(),
            ltp_param: [0; 3],
            ltp_pitch_int_past: 0,
            ltp_pitch_fr_past: 0,
            ltp_gain_past: 0,
            ltp_gain_idx_past: 0,
            ltp_mem_in: [0; LTP_MEM_IN_SIZE],
            ltp_mem_out: [0; LTP_MEM_OUT_SIZE],
            quant_spec_prev1: [0; 160],
            quant_spec_prev2: [0; 160],
            prev_window_shape: 0,
        }
    }
}

/// `CAacDecoderDynamicData::specificTo.usac`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CUsacSpecificDynData {
    /// `fd_noise_level_and_offset`
    pub fd_noise_level_and_offset: u8,
    /// `tns_active`
    pub tns_active: u8,
}

/// `CAacDecoderDynamicData`: data of one channel that is required for one element only
/// (`CAacDecoderChannelInfo::pDynData`).
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CAacDecoderDynamicData {
    /* Common bit stream data */
    /// `aScaleFactor`: spectral scale factors for each sfb in each window.
    pub a_scale_factor: [i16; 8 * 16],
    /// `aSfbScale`: could be free after ApplyTools()
    pub a_sfb_scale: [i16; 8 * 16],
    /// `aCodeBook`: section data: codebook for each window and sfb.
    pub a_code_book: [u8; 8 * 16],
    /// `band_is_noise`
    pub band_is_noise: [u8; 8 * 16],
    /// `TnsData`
    pub tns_data: CTnsData,
    /// `RawDataInfo`
    pub raw_data_info: CRawDataInfo,
    /// `specificTo.usac` (`specificTo.aac` is empty)
    pub specific_to_usac: CUsacSpecificDynData,
}

impl Default for CAacDecoderDynamicData {
    /// Zero-initialized. Note: the C++ decoder allocates `pDynData` with `FDKmalloc()`, i.e. its
    /// initial content is undefined.
    fn default() -> Self {
        CAacDecoderDynamicData {
            a_scale_factor: [0; 8 * 16],
            a_sfb_scale: [0; 8 * 16],
            a_code_book: [0; 8 * 16],
            band_is_noise: [0; 8 * 16],
            tns_data: CTnsData::default(),
            raw_data_info: CRawDataInfo::default(),
            specific_to_usac: CUsacSpecificDynData::default(),
        }
    }
}

/// `CWorkBufferCore1`: `union { FIXP_DBL mdctOutTemp[1024]; FIXP_DBL workBuffer[2048]; }`, one
/// instance shared by all channels (`GetWorkBufferCore1()`).
///
/// The union is kept as one buffer because the C++ code accesses the same memory through
/// different views: `mdctOutTemp` is the IMDCT output buffer, the stereo filling downmix
/// (`dmx_prev_modified`) and holds the IGF TNF masks of `CChannelElement_Decode()` as bytes
/// (`(UCHAR*)mdctOutTemp + 512`, see [`CWorkBufferCore1::tnf_masks`]); `workBuffer` is the
/// complex prediction scratch buffer (`CJointStereoPersistentData::scratchBuffer`).
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CWorkBufferCore1 {
    /// `workBuffer` (`mdctOutTemp` is `buf[..1024]`)
    pub buf: Box<[FixpDbl; WB_SECTION_SIZE]>,
}

impl Default for CWorkBufferCore1 {
    fn default() -> Self {
        CWorkBufferCore1 { buf: Box::new([0; WB_SECTION_SIZE]) }
    }
}

impl CWorkBufferCore1 {
    /// `mdctOutTemp`
    pub fn mdct_out_temp(&mut self) -> &mut [FixpDbl] {
        &mut self.buf[..1024]
    }

    /// `workBuffer`
    pub fn work_buffer(&mut self) -> &mut [FixpDbl] {
        &mut self.buf[..]
    }

    /// The whole buffer as bytes (in memory order of a little-endian machine, like the reference).
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        // SAFETY: `buf` is a contiguous array of `i32`; every byte pattern is a valid `u8` and
        // `u8` has alignment 1. The returned slice borrows `self` mutably.
        unsafe {
            core::slice::from_raw_parts_mut(
                self.buf.as_mut_ptr() as *mut u8,
                WB_SECTION_SIZE * core::mem::size_of::<FixpDbl>(),
            )
        }
    }

    /// The IGF TNF masks of `CChannelElement_Decode()`:
    /// `TNF_maskL = (UCHAR*)mdctOutTemp + 512`, `TNF_maskR = (UCHAR*)mdctOutTemp + 512 + 1024`
    /// (1024 bytes each).
    pub fn tnf_masks(&mut self) -> (&mut [u8], &mut [u8]) {
        let bytes = &mut self.as_bytes_mut()[512..512 + 2048];
        bytes.split_at_mut(1024)
    }
}

/// `CAacDecoderCommonStaticData` without `pWorkBufferCore1` (which is shared by all channels and
/// passed separately): persistent data of one element, shared by both channels of a CPE
/// (`pComStaticData`).
#[derive(StateDigest, Clone, Debug, Default, PartialEq, Eq)]
pub struct CAacDecoderCommonStaticData {
    /// `cplxPredictionData` (allocated for CPEs with `AC_EL_USAC_CP_POSSIBLE`)
    pub cplx_prediction_data: Option<Box<CCplxPredictionData>>,
    /// `pJointStereoData` (allocated for CPEs; one for one element)
    ///
    /// Not part of the state digest: "joint stereo scratch memory (valid for this frame)" (C++
    /// comment of the structure), parsed with the element in every frame before it is used
    /// (checked with the buffer poisoning `POISON_DIGEST_SCRATCH`).
    #[digest(skip)]
    pub joint_stereo_data: Option<Box<CJointStereoData>>,
}

/// `CAacDecoderChannelInfo`: data of one channel of every element.
///
/// Differences to the C++ structure (see the [crate documentation](crate)):
/// - `pSpectralCoefficient` points into the decoder's `workBufferCore2` (`ch * 1024`); it is not
///   stored, the functions take the spectrum as a slice.
/// - `pDynData` is owned ([`CAacDecoderChannelInfo::p_dyn_data`]).
/// - `pComData` (an empty structure) and `pComStaticData` are not stored; the element functions
///   take the [`CAacDecoderCommonStaticData`] and the [`CWorkBufferCore1`] as arguments.
/// - `IGFdata.IGF_Common_channel_data_handle` is not stored (see `mpeghdec_igf::igf_dec`).
#[derive(StateDigest, Clone, Debug)]
pub struct CAacDecoderChannelInfo {
    /// `specScale`: scale shift values of each spectrum window
    pub spec_scale: [i16; 8],
    /// `icsInfo`
    pub ics_info: CIcsInfo,
    /// `granuleLength`: size of smallest spectrum piece
    pub granule_length: i32,
    /// `ElementInstanceTag`
    pub element_instance_tag: u8,
    /// `renderMode`: output signal rendering mode
    pub render_mode: AacdecRenderMode,
    /// `pDynData`: data required for one element and discarded after decoding
    pub p_dyn_data: Box<CAacDecoderDynamicData>,
    /// `currAliasingSymmetry`: required for MPEG-H MCT
    pub curr_aliasing_symmetry: i32,
    /// `transform_splitting_active`: flag signalling transform splitting
    pub transform_splitting_active: u8,
    /// `IGFdata`
    pub igf_data: IgfPrivateData,
    /* Frequency Domain Prediction (FDP) tool */
    /// `fdp_data_present`
    pub fdp_data_present: i32,
    /// `fdp_spacing_index`
    pub fdp_spacing_index: i32,
}

impl Default for CAacDecoderChannelInfo {
    /// Zero-initialized like `GetAacDecoderChannelInfo()` (`FDKaalloc`).
    fn default() -> Self {
        CAacDecoderChannelInfo {
            spec_scale: [0; 8],
            ics_info: CIcsInfo::default(),
            granule_length: 0,
            element_instance_tag: 0,
            render_mode: AACDEC_RENDER_INVALID,
            p_dyn_data: Box::default(),
            curr_aliasing_symmetry: 0,
            transform_splitting_active: 0,
            igf_data: IgfPrivateData::default(),
            fdp_data_present: 0,
            fdp_spacing_index: 0,
        }
    }
}

/// `getSamplingRateInfo(t, samplesPerFrame, samplingRateIndex, samplingRate)`
pub fn get_sampling_rate_info(
    t: &mut SamplingRateInfo,
    samples_per_frame: u32,
    mut sampling_rate_index: u32,
    sampling_rate: u32,
) -> AacDecoderError {
    /* Search closest samplerate according to ISO/IEC 13818-7:2005(E) 8.2.4 (Table 38): */
    if sampling_rate_index >= 15 {
        const BORDERS: [u32; 12] =
            [u32::MAX, 92017, 75132, 55426, 46009, 37566, 27713, 23004, 18783, 13856, 11502, 9391];
        let sampling_rate_search = sampling_rate;
        let mut i = 0u32;
        while i < 11 {
            if BORDERS[i as usize] > sampling_rate_search
                && sampling_rate_search >= BORDERS[i as usize + 1]
            {
                break;
            }
            i += 1;
        }
        sampling_rate_index = i;
    }

    t.sampling_rate_index = sampling_rate_index;
    t.sampling_rate = sampling_rate;

    let index = match samples_per_frame {
        1024 => 0,
        _ => return AAC_DEC_UNSUPPORTED_FORMAT,
    };

    let info = &SFB_OFFSET_TABLES[index][sampling_rate_index as usize];
    t.scale_factor_bands_long = info.sfb_offset_long;
    t.scale_factor_bands_short = info.sfb_offset_short;
    t.number_of_scale_factor_bands_long = info.number_of_sfb_long;
    t.number_of_scale_factor_bands_short = info.number_of_sfb_short;

    if t.scale_factor_bands_long.is_empty() || t.number_of_scale_factor_bands_long == 0 {
        t.sampling_rate = 0;
        return AAC_DEC_UNSUPPORTED_FORMAT;
    }

    debug_assert!(
        t.scale_factor_bands_long[t.number_of_scale_factor_bands_long as usize] as u32
            == samples_per_frame
    );

    AAC_DEC_OK
}

/// `IcsReadMaxSfb(bs, pIcsInfo, pSamplingRateInfo)`: read max SFB from bit stream and assign
/// TotalSfBands according to the window sequence and sample rate.
pub fn ics_read_max_sfb(
    bs: &mut FdkBitStream,
    p_ics_info: &mut CIcsInfo,
    p_sampling_rate_info: &SamplingRateInfo,
) -> AacDecoderError {
    let mut error_status = AAC_DEC_OK;
    let nbits;

    if is_long_block(p_ics_info) != 0 {
        nbits = 6;
        p_ics_info.total_sf_bands = p_sampling_rate_info.number_of_scale_factor_bands_long;
    } else {
        nbits = 4;
        p_ics_info.total_sf_bands = p_sampling_rate_info.number_of_scale_factor_bands_short;
    }
    p_ics_info.max_sf_bands = bs.read_bits(nbits) as u8;

    if p_ics_info.max_sf_bands > p_ics_info.total_sf_bands {
        error_status = AAC_DEC_PARSE_ERROR;
    }

    error_status
}

/// `IcsRead(bs, pIcsInfo, pSamplingRateInfo, flags)`
pub fn ics_read(
    bs: &mut FdkBitStream,
    p_ics_info: &mut CIcsInfo,
    p_sampling_rate_info: &SamplingRateInfo,
    flags: u32,
) -> AacDecoderError {
    let mut error_status;

    p_ics_info.valid = 0;

    if flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) == 0 {
        bs.read_bit();
    }
    p_ics_info.window_sequence = bs.read_2bits() as BlockType;
    p_ics_info.window_shape = bs.read_bit() as u8;

    error_status = ics_read_max_sfb(bs, p_ics_info, p_sampling_rate_info);
    if error_status == AAC_DEC_OK {
        if is_long_block(p_ics_info) != 0 {
            /* If not ELD nor Scalable nor BSAC nor USAC syntax then ... */
            if flags & (AC_ELD | AC_SCALABLE | AC_BSAC | AC_USAC | AC_RSVD50 | AC_MPEGH3DA) == 0
                && bs.read_bit() as u8 != 0
            /* UCHAR PredictorDataPresent */
            {
                error_status = AAC_DEC_UNSUPPORTED_PREDICTION;
            } else {
                p_ics_info.window_groups = 1;
                p_ics_info.window_group_length[0] = 1;
            }
        } else {
            p_ics_info.scale_factor_grouping = bs.read_bits(7) as u8;

            p_ics_info.window_groups = 0;

            for i in 0..(8 - 1) {
                let mask = 1u32 << (6 - i);
                p_ics_info.window_group_length[i] = 1;

                if p_ics_info.scale_factor_grouping as u32 & mask != 0 {
                    p_ics_info.window_group_length[p_ics_info.window_groups as usize] += 1;
                } else {
                    p_ics_info.window_groups += 1;
                }
            }

            /* loop runs to i < 7 only */
            p_ics_info.window_group_length[8 - 1] = 1;
            p_ics_info.window_groups += 1;
        }
    }

    if error_status == AAC_DEC_OK {
        p_ics_info.valid = 1;
    }

    error_status
}

/****************** inline functions ******************/

/// `IsValid(pIcsInfo)`
#[inline]
pub fn is_valid(p_ics_info: &CIcsInfo) -> u8 {
    p_ics_info.valid
}

/// `IsLongBlock(pIcsInfo)`
#[inline]
pub fn is_long_block(p_ics_info: &CIcsInfo) -> u8 {
    (p_ics_info.window_sequence != BLOCK_SHORT) as u8
}

/// `GetWindowShape(pIcsInfo)`
#[inline]
pub fn get_window_shape(p_ics_info: &CIcsInfo) -> u8 {
    p_ics_info.window_shape
}

/// `GetWindowSequence(pIcsInfo)`
#[inline]
pub fn get_window_sequence(p_ics_info: &CIcsInfo) -> BlockType {
    p_ics_info.window_sequence
}

/// `GetScaleFactorBandOffsets(pIcsInfo, samplingRateInfo)`
#[inline]
pub fn get_scale_factor_band_offsets(
    p_ics_info: &CIcsInfo,
    sampling_rate_info: &SamplingRateInfo,
) -> &'static [i16] {
    if is_long_block(p_ics_info) != 0 {
        sampling_rate_info.scale_factor_bands_long
    } else {
        sampling_rate_info.scale_factor_bands_short
    }
}

/// `GetNumberOfScaleFactorBands(pIcsInfo, samplingRateInfo)`
#[inline]
pub fn get_number_of_scale_factor_bands(
    p_ics_info: &CIcsInfo,
    sampling_rate_info: &SamplingRateInfo,
) -> u8 {
    if is_long_block(p_ics_info) != 0 {
        sampling_rate_info.number_of_scale_factor_bands_long
    } else {
        sampling_rate_info.number_of_scale_factor_bands_short
    }
}

/// `GetWindowsPerFrame(pIcsInfo)`
#[inline]
pub fn get_windows_per_frame(p_ics_info: &CIcsInfo) -> i32 {
    if p_ics_info.window_sequence == BLOCK_SHORT {
        8
    } else {
        1
    }
}

/// `GetWindowGroups(pIcsInfo)`
#[inline]
pub fn get_window_groups(p_ics_info: &CIcsInfo) -> u8 {
    p_ics_info.window_groups
}

/// `GetWindowGroupLength(pIcsInfo, index)`
#[inline]
pub fn get_window_group_length(p_ics_info: &CIcsInfo, index: i32) -> u8 {
    p_ics_info.window_group_length[index as usize]
}

/// `GetWindowGroupLengthTable(pIcsInfo)`
#[inline]
pub fn get_window_group_length_table(p_ics_info: &CIcsInfo) -> &[u8; 8] {
    &p_ics_info.window_group_length
}

/// `GetScaleFactorBandsTransmitted(pIcsInfo)`
#[inline]
pub fn get_scale_factor_bands_transmitted(p_ics_info: &CIcsInfo) -> u8 {
    p_ics_info.max_sf_bands
}

/// `GetScaleMaxFactorBandsTransmitted(pIcsInfo0, pIcsInfo1)`
#[inline]
pub fn get_scale_max_factor_bands_transmitted(
    p_ics_info0: &CIcsInfo,
    p_ics_info1: &CIcsInfo,
) -> u8 {
    p_ics_info0.max_sf_bands.max(p_ics_info1.max_sf_bands)
}

/// `GetScaleFactorBandsTotal(pIcsInfo)`
#[inline]
pub fn get_scale_factor_bands_total(p_ics_info: &CIcsInfo) -> u8 {
    p_ics_info.total_sf_bands
}

/// `GetMaximumTnsBands(pIcsInfo, samplingRateIndex)`. Note: This function applies to AAC-LC only!
#[inline]
pub fn get_maximum_tns_bands(p_ics_info: &CIcsInfo, sampling_rate_index: u32) -> u8 {
    TNS_MAX_BANDS_TBL[sampling_rate_index as usize][(is_long_block(p_ics_info) == 0) as usize]
}
