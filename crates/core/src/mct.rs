// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/mct.h, libMpeghDec/src/mct.cpp and libMpeghDec/src/arm/mct_arm.cpp
// (its NEON applyMctRotationIdx is not compiled in the reference configuration).

//! Multichannel coding tool (MCT) of MPEG-H 3D Audio, including MCT stereo filling.
//!
//! # Usage by the frame level (`aacdecoder.cpp`)
//!
//! - Configuration: [`cmct_initialize`] for every signal group with an MCT extension element
//!   (`pMCTdec[grp]`, an `Option<Box<CMct>>`); [`cmct_destroy`] (or dropping the box) frees it.
//! - Bitstream parsing (`ID_EXT_ELE_MCT`): [`cmct_inverse_mct_parse_bs`], then the caller sets
//!   [`CMct::mct_group_parsed`].
//! - After all elements of a frame are parsed (`CChannelElement_Read()`) and before they are
//!   decoded (`CChannelElement_Decode()`): [`cmct_mct_stereo_filling`] followed by
//!   [`cmct_stereo_filling_save_prev`], or [`cmct_stereo_filling_clear_prev`] if the frame is not
//!   OK. The channel arguments start at the first channel of the signal group
//!   (`self->pAacDecoderChannelInfo + channelOffset`), like the C++ pointer arguments; the
//!   spectrum of channel `i` of the group is `spec[i * 1024..(i + 1) * 1024]`
//!   (`pSpectralCoefficient`, see the [crate documentation](crate)).
//!
//! # Differences to the C++ code
//!
//! - `prevOutSpec` / `prevOutSpec_exp` are allocated with `FDKmalloc()` in C++, i.e. their initial
//!   content is undefined. Here they are zero-initialized.
//! - The C++ code has undefined behaviour for some corrupt bitstreams; the Rust code returns an
//!   error instead (see [`cmct_mct_stereo_filling`]).
//! - `CMct_MCT_StereoFilling()` may be called with the same channel as both channels of a pair
//!   (only for stale channel pairs that were never parsed successfully, see
//!   [`cmct_inverse_mct_parse_bs`]). The Rust code reproduces the sequential semantics of the C++
//!   code on the aliased memory.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::MANTISSA_TABLE;
use crate::aacdecoder_lib::AAC_DEC_OK;
use crate::channelinfo::*;
use mpeghdec_fdk::audio::{AC_EL_ENHANCED_NOISE, AC_EL_IGF_USE_ENF};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_div_norm, sqrt_fixp_lookup_e};
use mpeghdec_fdk::mdct::BLOCK_SHORT;
use mpeghdec_fdk::scale::{get_scalefactor, scale_values};
use mpeghdec_igf::igf_dec::{
    iis_igf_dec_lib_get_number_of_tiles, IgfPrivateData, IgfPrivateStaticData, WhiteningLevel,
    IGF_GRID_LONG_WINDOW, IGF_GRID_SHORT_WINDOW, IGF_TILE_LEN, WHITENING_STRONG,
};

/// `MAX_NUM_MCT_CHANNELS`
pub const MAX_NUM_MCT_CHANNELS: usize = 28;
/// `MAX_NUM_MCT_BOXES`: MPEG-H LC, see DAM3 Study on Table P9
pub const MAX_NUM_MCT_BOXES: usize = 28;
/// `MAX_NUM_MCT_BANDS`
pub const MAX_NUM_MCT_BANDS: usize = 64;

/// `CODE_BOOK_ALPHA_LAV`
pub const CODE_BOOK_ALPHA_LAV: usize = 121;
/// `CODE_BOOK_BETA_LAV`
pub const CODE_BOOK_BETA_LAV: usize = 65;
/// `DEFAULT_BETA`: equals 45 degrees
const DEFAULT_BETA: i32 = 48;
/// `DEFAULT_ALPHA`
const DEFAULT_ALPHA: i32 = 0;

/// `READ_MASK(a, b)`: bit `b` of the msb aligned mask `a` (`b` is in `0..64` wherever it is used).
#[inline(always)]
fn read_mask(a: u64, b: i32) -> u64 {
    a & 1u64.wrapping_shl((63 - b) as u32)
}

/* huffman tables */
/* Generated with scripts/codegen/dump_table.py from libMpeghDec/src/mct.cpp. */

/// `huff_ctabAngle`: code words of the angle code book (MCCSignalingType 1)
pub static HUFF_CTAB_ANGLE: [i32; CODE_BOOK_BETA_LAV] = [
    0x00000000, 0x0000000B, 0x00000012, 0x0000001B, 0x0000001F, 0x00000031, 0x0000003A, 0x00000043,
    0x00000065, 0x00000073, 0x00000082, 0x0000009A, 0x000000CE, 0x000000EE, 0x00000106, 0x0000013A,
    0x000001D9, 0x000001DE, 0x00000202, 0x00000261, 0x0000020F, 0x0000020E, 0x00000263, 0x00000266,
    0x00000272, 0x00000271, 0x00000277, 0x00000276, 0x00000334, 0x00000325, 0x00000326, 0x00000327,
    0x00000324, 0x00000323, 0x00000335, 0x00000322, 0x00000320, 0x00000321, 0x00000273, 0x00000270,
    0x00000267, 0x00000260, 0x000004C4, 0x000004C5, 0x00000203, 0x000001DF, 0x000001DA, 0x000001D8,
    0x0000019B, 0x000001DB, 0x00000132, 0x00000100, 0x000000CF, 0x000000CC, 0x0000009B, 0x00000081,
    0x00000072, 0x0000004F, 0x00000042, 0x00000038, 0x00000030, 0x0000001E, 0x0000001A, 0x00000011,
    0x0000000A,
];
/// `huff_ltabAngle`: code word lengths of the angle code book
pub static HUFF_LTAB_ANGLE: [i32; CODE_BOOK_BETA_LAV] = [
    0x00000001, 0x00000004, 0x00000005, 0x00000005, 0x00000005, 0x00000006, 0x00000006, 0x00000007,
    0x00000007, 0x00000007, 0x00000008, 0x00000008, 0x00000008, 0x00000008, 0x00000009, 0x00000009,
    0x00000009, 0x00000009, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A,
    0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A,
    0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A, 0x0000000A,
    0x0000000A, 0x0000000A, 0x0000000B, 0x0000000B, 0x0000000A, 0x00000009, 0x00000009, 0x00000009,
    0x00000009, 0x00000009, 0x00000009, 0x00000009, 0x00000008, 0x00000008, 0x00000008, 0x00000008,
    0x00000007, 0x00000007, 0x00000007, 0x00000006, 0x00000006, 0x00000005, 0x00000005, 0x00000005,
    0x00000004,
];
/// `huff_ctabscf`: code words of the prediction coefficient code book (MCCSignalingType 0)
pub static HUFF_CTABSCF: [i32; CODE_BOOK_ALPHA_LAV] = [
    0x0003FFE8, 0x0003FFE6, 0x0003FFE7, 0x0003FFE5, 0x0007FFF5, 0x0007FFF1, 0x0007FFED, 0x0007FFF6,
    0x0007FFEE, 0x0007FFEF, 0x0007FFF0, 0x0007FFFC, 0x0007FFFD, 0x0007FFFF, 0x0007FFFE, 0x0007FFF7,
    0x0007FFF8, 0x0007FFFB, 0x0007FFF9, 0x0003FFE4, 0x0007FFFA, 0x0003FFE3, 0x0001FFEF, 0x0001FFF0,
    0x0000FFF5, 0x0001FFEE, 0x0000FFF2, 0x0000FFF3, 0x0000FFF4, 0x0000FFF1, 0x00007FF6, 0x00007FF7,
    0x00003FF9, 0x00003FF5, 0x00003FF7, 0x00003FF3, 0x00003FF6, 0x00003FF2, 0x00001FF7, 0x00001FF5,
    0x00000FF9, 0x00000FF7, 0x00000FF6, 0x000007F9, 0x00000FF4, 0x000007F8, 0x000003F9, 0x000003F7,
    0x000003F5, 0x000001F8, 0x000001F7, 0x000000FA, 0x000000F8, 0x000000F6, 0x00000079, 0x0000003A,
    0x00000038, 0x0000001A, 0x0000000B, 0x00000004, 0x00000000, 0x0000000A, 0x0000000C, 0x0000001B,
    0x00000039, 0x0000003B, 0x00000078, 0x0000007A, 0x000000F7, 0x000000F9, 0x000001F6, 0x000001F9,
    0x000003F4, 0x000003F6, 0x000003F8, 0x000007F5, 0x000007F4, 0x000007F6, 0x000007F7, 0x00000FF5,
    0x00000FF8, 0x00001FF4, 0x00001FF6, 0x00001FF8, 0x00003FF8, 0x00003FF4, 0x0000FFF0, 0x00007FF4,
    0x0000FFF6, 0x00007FF5, 0x0003FFE2, 0x0007FFD9, 0x0007FFDA, 0x0007FFDB, 0x0007FFDC, 0x0007FFDD,
    0x0007FFDE, 0x0007FFD8, 0x0007FFD2, 0x0007FFD3, 0x0007FFD4, 0x0007FFD5, 0x0007FFD6, 0x0007FFF2,
    0x0007FFDF, 0x0007FFE7, 0x0007FFE8, 0x0007FFE9, 0x0007FFEA, 0x0007FFEB, 0x0007FFE6, 0x0007FFE0,
    0x0007FFE1, 0x0007FFE2, 0x0007FFE3, 0x0007FFE4, 0x0007FFE5, 0x0007FFD7, 0x0007FFEC, 0x0007FFF4,
    0x0007FFF3,
];
/// `huff_ltabscf`: code word lengths of the prediction coefficient code book
pub static HUFF_LTABSCF: [i32; CODE_BOOK_ALPHA_LAV] = [
    0x00000012, 0x00000012, 0x00000012, 0x00000012, 0x00000013, 0x00000013, 0x00000013, 0x00000013,
    0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013,
    0x00000013, 0x00000013, 0x00000013, 0x00000012, 0x00000013, 0x00000012, 0x00000011, 0x00000011,
    0x00000010, 0x00000011, 0x00000010, 0x00000010, 0x00000010, 0x00000010, 0x0000000F, 0x0000000F,
    0x0000000E, 0x0000000E, 0x0000000E, 0x0000000E, 0x0000000E, 0x0000000E, 0x0000000D, 0x0000000D,
    0x0000000C, 0x0000000C, 0x0000000C, 0x0000000B, 0x0000000C, 0x0000000B, 0x0000000A, 0x0000000A,
    0x0000000A, 0x00000009, 0x00000009, 0x00000008, 0x00000008, 0x00000008, 0x00000007, 0x00000006,
    0x00000006, 0x00000005, 0x00000004, 0x00000003, 0x00000001, 0x00000004, 0x00000004, 0x00000005,
    0x00000006, 0x00000006, 0x00000007, 0x00000007, 0x00000008, 0x00000008, 0x00000009, 0x00000009,
    0x0000000A, 0x0000000A, 0x0000000A, 0x0000000B, 0x0000000B, 0x0000000B, 0x0000000B, 0x0000000C,
    0x0000000C, 0x0000000D, 0x0000000D, 0x0000000D, 0x0000000E, 0x0000000E, 0x00000010, 0x0000000F,
    0x00000010, 0x0000000F, 0x00000012, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013,
    0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013,
    0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013,
    0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013, 0x00000013,
    0x00000013,
];
/// `HUFF_LTABANGLE_MAX_VALUE`
pub const HUFF_LTABANGLE_MAX_VALUE: i32 = 0xB;
/// `HUFF_LTABSCF_MAX_VALUE`
pub const HUFF_LTABSCF_MAX_VALUE: i32 = 0x13;

/* quantized sin/cos tables for index */
/// `tabIndexToCosSinAlpha`: quantized cos (`re`) / sin (`im`) of the angle index
pub static TAB_INDEX_TO_COS_SIN_ALPHA: [FixpSpk; CODE_BOOK_BETA_LAV] = [
    FixpSpk::new(0x0000, 0x8000u16 as i16),
    FixpSpk::new(0x0648, 0x8027u16 as i16),
    FixpSpk::new(0x0C8C, 0x809Eu16 as i16),
    FixpSpk::new(0x12C8, 0x8163u16 as i16),
    FixpSpk::new(0x18F9, 0x8276u16 as i16),
    FixpSpk::new(0x1F1A, 0x83D6u16 as i16),
    FixpSpk::new(0x2528, 0x8583u16 as i16),
    FixpSpk::new(0x2B1F, 0x877Bu16 as i16),
    FixpSpk::new(0x30FC, 0x89BEu16 as i16),
    FixpSpk::new(0x36BA, 0x8C4Au16 as i16),
    FixpSpk::new(0x3C57, 0x8F1Du16 as i16),
    FixpSpk::new(0x41CE, 0x9236u16 as i16),
    FixpSpk::new(0x471D, 0x9592u16 as i16),
    FixpSpk::new(0x4C40, 0x9930u16 as i16),
    FixpSpk::new(0x5134, 0x9D0Eu16 as i16),
    FixpSpk::new(0x55F6, 0xA129u16 as i16),
    FixpSpk::new(0x5A82, 0xA57Eu16 as i16),
    FixpSpk::new(0x5ED7, 0xAA0Au16 as i16),
    FixpSpk::new(0x62F2, 0xAECCu16 as i16),
    FixpSpk::new(0x66D0, 0xB3C0u16 as i16),
    FixpSpk::new(0x6A6E, 0xB8E3u16 as i16),
    FixpSpk::new(0x6DCA, 0xBE32u16 as i16),
    FixpSpk::new(0x70E3, 0xC3A9u16 as i16),
    FixpSpk::new(0x73B6, 0xC946u16 as i16),
    FixpSpk::new(0x7642, 0xCF04u16 as i16),
    FixpSpk::new(0x7885, 0xD4E1u16 as i16),
    FixpSpk::new(0x7A7D, 0xDAD8u16 as i16),
    FixpSpk::new(0x7C2A, 0xE0E6u16 as i16),
    FixpSpk::new(0x7D8A, 0xE707u16 as i16),
    FixpSpk::new(0x7E9D, 0xED38u16 as i16),
    FixpSpk::new(0x7F62, 0xF374u16 as i16),
    FixpSpk::new(0x7FD9, 0xF9B8u16 as i16),
    FixpSpk::new(0x7FFF, 0x0000),
    FixpSpk::new(0x7FD9, 0x0648),
    FixpSpk::new(0x7F62, 0x0C8C),
    FixpSpk::new(0x7E9D, 0x12C8),
    FixpSpk::new(0x7D8A, 0x18F9),
    FixpSpk::new(0x7C2A, 0x1F1A),
    FixpSpk::new(0x7A7D, 0x2528),
    FixpSpk::new(0x7885, 0x2B1F),
    FixpSpk::new(0x7642, 0x30FC),
    FixpSpk::new(0x73B6, 0x36BA),
    FixpSpk::new(0x70E3, 0x3C57),
    FixpSpk::new(0x6DCA, 0x41CE),
    FixpSpk::new(0x6A6E, 0x471D),
    FixpSpk::new(0x66D0, 0x4C40),
    FixpSpk::new(0x62F2, 0x5134),
    FixpSpk::new(0x5ED7, 0x55F6),
    FixpSpk::new(0x5A82, 0x5A82),
    FixpSpk::new(0x55F6, 0x5ED7),
    FixpSpk::new(0x5134, 0x62F2),
    FixpSpk::new(0x4C40, 0x66D0),
    FixpSpk::new(0x471D, 0x6A6E),
    FixpSpk::new(0x41CE, 0x6DCA),
    FixpSpk::new(0x3C57, 0x70E3),
    FixpSpk::new(0x36BA, 0x73B6),
    FixpSpk::new(0x30FC, 0x7642),
    FixpSpk::new(0x2B1F, 0x7885),
    FixpSpk::new(0x2528, 0x7A7D),
    FixpSpk::new(0x1F1A, 0x7C2A),
    FixpSpk::new(0x18F9, 0x7D8A),
    FixpSpk::new(0x12C8, 0x7E9D),
    FixpSpk::new(0x0C8C, 0x7F62),
    FixpSpk::new(0x0648, 0x7FD9),
    FixpSpk::new(0x0000, 0x7FFF),
];

/// `CMctWork`
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CMctWork {
    /// `keepTree`
    pub keep_tree: u8,
    /// `numPairs`
    pub num_pairs: u8,
    /// `codePairs`
    pub code_pairs: [[u8; 2]; MAX_NUM_MCT_BOXES],
    /// `bDeltaTime`
    pub b_delta_time: [u8; MAX_NUM_MCT_BOXES],
    /// `pairCoeffDeltaSfb` (entries of inactive bands keep their previous values)
    pub pair_coeff_delta_sfb: [[u8; MAX_NUM_MCT_BANDS]; MAX_NUM_MCT_BOXES],
    /// `pairCoeffDeltaFb`
    pub pair_coeff_delta_fb: [u8; MAX_NUM_MCT_BOXES],
    /// `numMctMaskBands`
    pub num_mct_mask_bands: [u8; MAX_NUM_MCT_BOXES],
    /// `mctMask`: msb aligned, one bit per band
    pub mct_mask: [u64; MAX_NUM_MCT_BOXES],
    /// `bHasMctMask`
    pub b_has_mct_mask: [u8; MAX_NUM_MCT_BOXES],
    /// `bHasBandwiseCoeffs`
    pub b_has_bandwise_coeffs: [u8; MAX_NUM_MCT_BOXES],
    /// `predDir`
    pub pred_dir: [u8; MAX_NUM_MCT_BOXES],
    /// `hasStereoFilling`
    pub has_stereo_filling: [u8; MAX_NUM_MCT_CHANNELS],
}

impl Default for CMctWork {
    /// Zero-initialized like `CMct_Initialize()` (`FDKcalloc`).
    fn default() -> Self {
        CMctWork {
            keep_tree: 0,
            num_pairs: 0,
            code_pairs: [[0; 2]; MAX_NUM_MCT_BOXES],
            b_delta_time: [0; MAX_NUM_MCT_BOXES],
            pair_coeff_delta_sfb: [[0; MAX_NUM_MCT_BANDS]; MAX_NUM_MCT_BOXES],
            pair_coeff_delta_fb: [0; MAX_NUM_MCT_BOXES],
            num_mct_mask_bands: [0; MAX_NUM_MCT_BOXES],
            mct_mask: [0; MAX_NUM_MCT_BOXES],
            b_has_mct_mask: [0; MAX_NUM_MCT_BOXES],
            b_has_bandwise_coeffs: [0; MAX_NUM_MCT_BOXES],
            pred_dir: [0; MAX_NUM_MCT_BOXES],
            has_stereo_filling: [0; MAX_NUM_MCT_CHANNELS],
        }
    }
}

/// `CMct`: state of the multichannel coding tool of one signal group.
///
/// Differences to the C++ structure: `mctWork`, `prevOutSpec` and `prevOutSpec_exp` are owned
/// (see the module documentation for the initial content).
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CMct {
    /// `mctWork`
    pub mct_work: Box<CMctWork>,
    /// `MCCSignalingType`
    pub mcc_signaling_type: u8,
    /// `MCTSignalingTypePrev`
    pub mct_signaling_type_prev: u8,
    /// `windowSequenceIsShortPrev`
    pub window_sequence_is_short_prev: [u8; MAX_NUM_MCT_BOXES],
    /// `channelMap`
    pub channel_map: [u8; MAX_NUM_MCT_CHANNELS],
    /// `numMctChannels`
    pub num_mct_channels: u8,
    /// `pairCoeffQSfbPrev`
    pub pair_coeff_q_sfb_prev: [[i16; MAX_NUM_MCT_BANDS]; MAX_NUM_MCT_BOXES],
    /// `pairCoeffQFbPrev`: fullband coeff
    pub pair_coeff_q_fb_prev: [i16; MAX_NUM_MCT_BOXES],
    /// `prevOutSpec`: `numMctChannels * 1024` values (empty for `NULL`)
    pub prev_out_spec: Vec<FixpDbl>,
    /// `prevOutSpec_exp`: `numMctChannels * 8 * 16` values (empty for `NULL`)
    pub prev_out_spec_exp: Vec<i16>,
    /// `Mct_group_parsed`: set by the frame level
    pub mct_group_parsed: u8,
}

impl Default for CMct {
    /// Zero-initialized like `CMct_Initialize()` (`FDKcalloc`).
    fn default() -> Self {
        CMct {
            mct_work: Box::default(),
            mcc_signaling_type: 0,
            mct_signaling_type_prev: 0,
            window_sequence_is_short_prev: [0; MAX_NUM_MCT_BOXES],
            channel_map: [0; MAX_NUM_MCT_CHANNELS],
            num_mct_channels: 0,
            pair_coeff_q_sfb_prev: [[0; MAX_NUM_MCT_BANDS]; MAX_NUM_MCT_BOXES],
            pair_coeff_q_fb_prev: [0; MAX_NUM_MCT_BOXES],
            prev_out_spec: Vec::new(),
            prev_out_spec_exp: Vec::new(),
            mct_group_parsed: 0,
        }
    }
}

/// `CMctPtr`
pub type CMctPtr = Option<Box<CMct>>;

/// The two channels of an MCT channel pair (or their data). The C++ code accesses them through
/// two pointers which can point to the same channel (only for stale channel pairs, see
/// [`cmct_inverse_mct_parse_bs`]); `One` reproduces the sequential semantics of such aliased
/// accesses.
enum Pair<'a, T: ?Sized> {
    Two(&'a mut T, &'a mut T),
    One(&'a mut T),
}

impl<'a, T: ?Sized> Pair<'a, T> {
    /// The first channel (`ch1`, `dmx`).
    #[inline(always)]
    fn a(&mut self) -> &mut T {
        match self {
            Pair::Two(a, _) => a,
            Pair::One(x) => x,
        }
    }

    /// The second channel (`ch2`, `res`).
    #[inline(always)]
    fn b(&mut self) -> &mut T {
        match self {
            Pair::Two(_, b) => b,
            Pair::One(x) => x,
        }
    }

    /// The same part of both channels.
    #[inline(always)]
    fn map<'b, U: ?Sized>(&'b mut self, f: impl Fn(&'b mut T) -> &'b mut U) -> Pair<'b, U> {
        match self {
            Pair::Two(a, b) => Pair::Two(f(a), f(b)),
            Pair::One(x) => Pair::One(f(x)),
        }
    }
}

/// Elements `i` and `j` of `s`.
fn pair_of<T>(s: &mut [T], i: usize, j: usize) -> Pair<'_, T> {
    if i == j {
        Pair::One(&mut s[i])
    } else if i < j {
        let (x, y) = s.split_at_mut(j);
        Pair::Two(&mut x[i], &mut y[0])
    } else {
        let (x, y) = s.split_at_mut(i);
        Pair::Two(&mut y[0], &mut x[j])
    }
}

/// Chunks `i` and `j` (of `len` elements) of `s`.
fn pair_of_chunks<T>(s: &mut [T], i: usize, j: usize, len: usize) -> Pair<'_, [T]> {
    if i == j {
        Pair::One(&mut s[i * len..(i + 1) * len])
    } else if i < j {
        let (x, y) = s.split_at_mut(j * len);
        Pair::Two(&mut x[i * len..(i + 1) * len], &mut y[..len])
    } else {
        let (x, y) = s.split_at_mut(i * len);
        Pair::Two(&mut y[..len], &mut x[j * len..(j + 1) * len])
    }
}

/// `GetTileWhiteningLevel(hPrivateData, TileNum)`
#[inline]
fn get_tile_whitening_level(h_private_data: &IgfPrivateData, tile_num: i32) -> WhiteningLevel {
    h_private_data.bitstream_data[0].igf_whitening_level[tile_num as usize]
}

/// `CMct_Initialize(pCMctPtr, mctChanMask, firstSigIdx, signalsInGroup)`: allocates and
/// initializes the MCT state. `mct_chan_mask` is an msb aligned bit field with one bit for each
/// channel of the signal group (`signals_in_group <= 28`, `TP_MAX_CHANNELS_PER_SIGNAL_GROUP`).
///
/// Returns -1 if `*p_cmct_ptr` is already allocated. The C++ allocation failures cannot happen.
pub fn cmct_initialize(
    p_cmct_ptr: &mut Option<Box<CMct>>,
    mct_chan_mask: u32,
    first_sig_idx: i32,
    signals_in_group: i32,
) -> i32 {
    if p_cmct_ptr.is_some() {
        return -1;
    }

    let mut mct = Box::<CMct>::default();

    /* get channelmap/mask for which channels the tool is active */
    mct.num_mct_channels = 0;
    for j in 0..signals_in_group {
        let channel_mask_j = mct_chan_mask & 1u32.wrapping_shl((31 - j) as u32);
        if channel_mask_j != 0 {
            mct.channel_map[mct.num_mct_channels as usize] = (j + first_sig_idx) as u8;
            mct.num_mct_channels += 1;
        }
    }

    if mct.num_mct_channels != 0 {
        /* FDKmalloc() in C++: the initial content is undefined; zero here */
        mct.prev_out_spec = vec![0; mct.num_mct_channels as usize * 1024];
        mct.prev_out_spec_exp = vec![0; mct.num_mct_channels as usize * 8 * 16];
    } else {
        mct.prev_out_spec = Vec::new();
        mct.prev_out_spec_exp = Vec::new();
    }

    mct.mct_signaling_type_prev = -1i8 as u8;

    *p_cmct_ptr = Some(mct);

    AAC_DEC_OK
}

/// `CMct_Destroy(self)`: frees the MCT state (the same as dropping it).
pub fn cmct_destroy(self_: Option<Box<CMct>>) {
    drop(self_);
}

/// `CMct_inverseMctParseChannelPair(hbitBuffer, pCodePairs, nChannels)`
fn cmct_inverse_mct_parse_channel_pair(
    hbit_buffer: &mut FdkBitStream,
    p_code_pairs: &mut [u8; 2],
    n_channels: u32,
) -> i32 {
    let mut err = 1;
    let mut pair_counter = 0;
    let mut max_num_pair_idx = (n_channels as i32) * (n_channels as i32 - 1) / 2 - 1;
    let mut num_bits = 1;

    if max_num_pair_idx < 0 {
        return 1;
    }

    loop {
        max_num_pair_idx >>= 1;
        if max_num_pair_idx == 0 {
            break;
        }
        num_bits += 1;
    }

    let pair_idx = hbit_buffer.read_bits(num_bits) as i32;

    'search: for chan1 in 1..n_channels {
        for chan0 in 0..chan1 {
            if pair_counter == pair_idx {
                p_code_pairs[0] = chan0 as u8;
                p_code_pairs[1] = chan1 as u8;
                err = 0; /* return from function */
                break 'search;
            } else {
                pair_counter += 1;
            }
        }
    }

    err
}

/// `CMct_decodeHuffman(hbitBuffer, ctab, ltab, maxl, limit)`
fn cmct_decode_huffman(
    hbit_buffer: &mut FdkBitStream,
    ctab: &[i32],
    ltab: &[i32],
    maxl: i32,
    limit: i32,
) -> i32 {
    let mut shift = 0;

    let bs_val = hbit_buffer.read_bits(maxl as u32) as i32;

    let mut tab_idx = 0;
    while tab_idx < limit {
        shift = maxl - ltab[tab_idx as usize]; /* shift in [0.. maxl] */
        if (bs_val >> shift) == ctab[tab_idx as usize] {
            /* found */
            break;
        }
        tab_idx += 1;
    }
    if tab_idx >= limit {
        /* End of angle code book reached */
        tab_idx = -1;
        shift = maxl;
    }
    hbit_buffer.push_back(shift as u32);

    tab_idx
}

/// `Read_MultichannelCodingBox(MCTSignalingType, usacIndepFlag, hbitBuffer, self, work, i,
/// huff_ctab, huff_ltab, huff_ltab_max_val, code_book_lav)`
#[allow(clippy::too_many_arguments)]
fn read_multichannel_coding_box(
    mct_signaling_type: i32,
    usac_indep_flag: i32,
    hbit_buffer: &mut FdkBitStream,
    self_: &mut CMct,
    i: usize,
    huff_ctab: &[i32],
    huff_ltab: &[i32],
    huff_ltab_max_val: i32,
    code_book_lav: i32,
) -> i32 {
    let num_mct_channels = self_.num_mct_channels;
    let work = &mut *self_.mct_work;

    /* read channel pair */
    if work.keep_tree == 0 {
        let ret_val = cmct_inverse_mct_parse_channel_pair(
            hbit_buffer,
            &mut work.code_pairs[i],
            num_mct_channels as u32,
        );
        if ret_val > 0 {
            return 1;
        }
    }

    /* read flags */
    work.b_has_mct_mask[i] = hbit_buffer.read_bit() as u8;
    work.b_has_bandwise_coeffs[i] = hbit_buffer.read_bit() as u8;

    /* read number of bands for non-fullband boxes */
    work.num_mct_mask_bands[i] = MAX_NUM_MCT_BANDS as u8;
    if work.b_has_mct_mask[i] != 0 || work.b_has_bandwise_coeffs[i] != 0 {
        let pair_is_short = hbit_buffer.read_bit();
        work.num_mct_mask_bands[i] = hbit_buffer.read_bits(5) as u8;
        if pair_is_short != 0 {
            work.num_mct_mask_bands[i] *= 8;
        }
        if work.num_mct_mask_bands[i] as usize > MAX_NUM_MCT_BANDS {
            return 1;
        }
    }

    /* evaluate mctMask flag */
    if work.b_has_mct_mask[i] != 0 && work.num_mct_mask_bands[i] != 0 {
        /* parse mct mask: read work->numMctMaskBands[i] (= 1...64) bits and left align them in UINT64
         */
        let n = work.num_mct_mask_bands[i] as i32;
        let bits_hi = (hbit_buffer.read_bits(f_min(32, n) as u32) as u64) << f_max(32, 64 - n);
        let bits_lo = (hbit_buffer.read_bits(f_max(0, n - 32) as u32) as u64) << (64 - n);
        work.mct_mask[i] = bits_hi | bits_lo;
    } else {
        /* all bands active */
        work.mct_mask[i] = 0xffffffffffffffff;
    }

    work.pred_dir[i] = if mct_signaling_type == 0 { hbit_buffer.read_bit() as u8 } else { 0 };

    if usac_indep_flag > 0 {
        work.b_delta_time[i] = 0;
    } else {
        work.b_delta_time[i] = hbit_buffer.read_bit() as u8;
    }

    if work.b_has_bandwise_coeffs[i] == 0 {
        /* use fullband angle */
        let val = cmct_decode_huffman(
            hbit_buffer,
            huff_ctab,
            huff_ltab,
            huff_ltab_max_val,
            code_book_lav,
        );
        /* -1 (no matching code word) is stored as 255 */
        work.pair_coeff_delta_fb[i] = val as u8;
    } else {
        /* use bandwise angles */
        for j in 0..work.num_mct_mask_bands[i] as i32 {
            if read_mask(work.mct_mask[i], j) > 0 {
                let val = cmct_decode_huffman(
                    hbit_buffer,
                    huff_ctab,
                    huff_ltab,
                    huff_ltab_max_val,
                    code_book_lav,
                );
                work.pair_coeff_delta_sfb[i][j as usize] = val as u8;
            }
        }
    }

    0
}

/// `CMct_inverseMctParseBS(self, hbitBuffer, usacIndepFlag, nChannels)`: parses
/// `MultichannelCodingFrame()`. Returns 0 on success, 1 on a parse error.
///
/// `n_channels` is not used (the number of channels of the tool is `numMctChannels`). The C++
/// `NULL` checks of `self` and `hbitBuffer` (returning -1) do not apply.
///
/// Note (C++ behaviour): for `MCCSignalingType == 1` the return value of
/// `Read_MultichannelCodingBox()` is ignored, i.e. a box with an invalid channel pair or too many
/// bands is skipped (keeping the stale data of the pair, possibly with `numMctMaskBands > 64`)
/// and parsing continues with the next box.
pub fn cmct_inverse_mct_parse_bs(
    self_: &mut CMct,
    hbit_buffer: &mut FdkBitStream,
    usac_indep_flag: i32,
    _n_channels: i32,
) -> i32 {
    let mut default_value = 0;

    let mct_stereo_filling = hbit_buffer.read_bit();
    self_.mcc_signaling_type = hbit_buffer.read_bit() as u8;
    self_.mct_work.keep_tree = 0;
    if usac_indep_flag == 0 {
        self_.mct_work.keep_tree = hbit_buffer.read_bit() as u8;
    }

    if self_.mcc_signaling_type == 0 {
        default_value = DEFAULT_ALPHA;
    } else if self_.mcc_signaling_type == 1 {
        default_value = DEFAULT_BETA;
    }

    if usac_indep_flag > 0 || self_.mct_signaling_type_prev != self_.mcc_signaling_type {
        /* reset coefficient memory on indepFlag */
        for i in 0..MAX_NUM_MCT_BOXES {
            self_.pair_coeff_q_fb_prev[i] = default_value as i16;
            self_.pair_coeff_q_sfb_prev[i] = [default_value as i16; MAX_NUM_MCT_BANDS];
        }
    }

    /* reset coefficient memory above the number of pairs */
    for i in self_.mct_work.num_pairs as usize..MAX_NUM_MCT_BOXES {
        self_.pair_coeff_q_fb_prev[i] = default_value as i16;
        self_.pair_coeff_q_sfb_prev[i] = [default_value as i16; MAX_NUM_MCT_BANDS];
    }

    if self_.mct_work.keep_tree == 0 {
        let tmp = hbit_buffer.escaped_value(5, 8, 16);
        if tmp > MAX_NUM_MCT_BOXES as u32 {
            return 1;
        }
        self_.mct_work.num_pairs = tmp as u8;
    }

    for i in 0..self_.mct_work.num_pairs as usize {
        if mct_stereo_filling != 0 {
            self_.mct_work.has_stereo_filling[i] = hbit_buffer.read_bit() as u8;
        } else {
            self_.mct_work.has_stereo_filling[i] = 0;
        }

        if self_.mcc_signaling_type == 0 {
            let ret_val = read_multichannel_coding_box(
                self_.mcc_signaling_type as i32,
                usac_indep_flag,
                hbit_buffer,
                self_,
                i,
                &HUFF_CTABSCF,
                &HUFF_LTABSCF,
                HUFF_LTABSCF_MAX_VALUE,
                CODE_BOOK_ALPHA_LAV as i32,
            );
            if ret_val != 0 {
                return ret_val;
            }
        } else if self_.mcc_signaling_type == 1 {
            /* The C++ code ignores the return value here. */
            read_multichannel_coding_box(
                self_.mcc_signaling_type as i32,
                usac_indep_flag,
                hbit_buffer,
                self_,
                i,
                &HUFF_CTAB_ANGLE,
                &HUFF_LTAB_ANGLE,
                HUFF_LTABANGLE_MAX_VALUE,
                CODE_BOOK_BETA_LAV as i32,
            );
        }
    }

    self_.mct_signaling_type_prev = self_.mcc_signaling_type;

    0
}

/// `clean_sfb_band_MCT_STEFI_DMX(outCoefficient, outCoefficient_exp, length)`
fn clean_sfb_band_mct_stefi_dmx(out_coefficient: &mut [FixpDbl], out_coefficient_exp: i32) {
    /*Thresholds for testing. Thresholds created with maximum possible precision */

    //"1e-8"
    const VALUE0_001: FixpDbl = 0x55e63b80;
    const VALUE0_001_EXP: i32 = -26;

    // (The C++ code lists further thresholds for "1e-7" ... "1e-2" as comments.)

    let mut delta = out_coefficient_exp - VALUE0_001_EXP;

    if delta >= 0 {
        delta = f_min(31, delta);
        let clean_threshold = VALUE0_001 >> delta;
        if clean_threshold != 0 {
            for c in out_coefficient.iter_mut() {
                if f_abs(*c) < clean_threshold {
                    *c = 0;
                }
            }
        }
    } else {
        out_coefficient.fill(0);
    }
}

/// `inverseDpcmAngleCoding(self, pairCoeffQSfb, pair, bIsShortBlock, windowsPerFrame)`: returns
/// the number of MCT bands per window or -1 on error.
///
/// Deviation: returns -1 if `numMctMaskBands[pair] > 64` (possible after an ignored parse error
/// with `MCCSignalingType == 1`), where the C++ code writes beyond the end of `pairCoeffQSfb` and
/// `pairCoeffQSfbPrev[pair]` (undefined behaviour).
fn inverse_dpcm_angle_coding(
    self_: &mut CMct,
    pair_coeff_q_sfb: &mut [i16; MAX_NUM_MCT_BANDS],
    pair: usize,
    b_is_short_block: i32,
    windows_per_frame: i32,
) -> i32 {
    let mut last_val: i32 = 0; /* always set before it is used */
    let mut coeff_q: i32;
    let work = &*self_.mct_work;
    let prev_sfb = &mut self_.pair_coeff_q_sfb_prev[pair];
    let prev_fb = &mut self_.pair_coeff_q_fb_prev[pair];
    let is_short_prev = &mut self_.window_sequence_is_short_prev[pair];

    if work.num_mct_mask_bands[pair] as usize > MAX_NUM_MCT_BANDS {
        mpeghdec_fdk::deviation::hit();
        return -1;
    }

    /* split transmitted mct bands into short blocks */
    if b_is_short_block == 0 {
        debug_assert!(windows_per_frame == 1);
    }
    let mct_bands_per_window =
        (work.num_mct_mask_bands[pair] as i32) >> (if windows_per_frame == 1 { 0 } else { 3 }); /* Short: divide by 8, Long: divide by 1 */
    let num_bands = work.num_mct_mask_bands[pair] as i32;
    let mct_mask = work.mct_mask[pair];

    if self_.mcc_signaling_type == 0 {
        if b_is_short_block != *is_short_prev as i32 {
            /* reset previous angles */
            *prev_fb = DEFAULT_ALPHA as i16;
            prev_sfb.fill(DEFAULT_ALPHA as i16);
        }
        *is_short_prev = b_is_short_block as u8;

        if work.b_has_bandwise_coeffs[pair] == 0 {
            /* use fullband coeff */
            if work.b_delta_time[pair] > 0 {
                last_val = *prev_fb as i32;
            } else {
                last_val = DEFAULT_ALPHA;
            }

            /* inverse dpcm */
            coeff_q = last_val - work.pair_coeff_delta_fb[pair] as i32 + 60;

            *prev_fb = coeff_q as i16;

            let mut band_mod_mct_bands_per_window = 0;
            for band in 0..num_bands {
                /* set all coeffs to fullband coeff */
                pair_coeff_q_sfb[band as usize] = coeff_q as i16;

                /* set previous coeffs according to mctMask */
                if read_mask(mct_mask, band) > 0 {
                    prev_sfb[band_mod_mct_bands_per_window as usize] = coeff_q as i16;
                } else {
                    prev_sfb[band_mod_mct_bands_per_window as usize] = DEFAULT_ALPHA as i16;
                }
                band_mod_mct_bands_per_window += 1;
                if band_mod_mct_bands_per_window == mct_bands_per_window {
                    band_mod_mct_bands_per_window = 0;
                }
            }
        } else {
            /* use bandwise coeffs */
            let mut band_mod_mct_bands_per_window = 0;
            for band in 0..num_bands {
                if work.b_delta_time[pair] > 0 {
                    /* modulo operation to use previous short block of same frame, no effect for long blocks
                     */
                    last_val = prev_sfb[band_mod_mct_bands_per_window as usize] as i32;
                } else {
                    /* reset freqency differential per shortblock to avoid predicting from highest to lowest
                     * freq */
                    if band_mod_mct_bands_per_window == 0 {
                        last_val = DEFAULT_ALPHA;
                    }
                }

                if read_mask(mct_mask, band) > 0 {
                    /* inverse dpcm */
                    coeff_q = last_val - work.pair_coeff_delta_sfb[pair][band as usize] as i32 + 60;

                    pair_coeff_q_sfb[band as usize] = coeff_q as i16;
                    last_val = coeff_q;
                    /* shortblock modulo operation only for previous values */
                    prev_sfb[band_mod_mct_bands_per_window as usize] = coeff_q as i16;
                } else {
                    /* reset coeffs for inactive bands */
                    prev_sfb[band_mod_mct_bands_per_window as usize] = DEFAULT_ALPHA as i16;
                }
                band_mod_mct_bands_per_window += 1;
                if band_mod_mct_bands_per_window == mct_bands_per_window {
                    band_mod_mct_bands_per_window = 0;
                }
            }

            /* reset fullband coeff */
            *prev_fb = DEFAULT_ALPHA as i16;
        }

        /* reset previous coeffs for bands above mct mask */
        for band in mct_bands_per_window as usize..MAX_NUM_MCT_BANDS {
            prev_sfb[band] = DEFAULT_ALPHA as i16;
        }
    } else if self_.mcc_signaling_type == 1 {
        if b_is_short_block != *is_short_prev as i32 {
            /* reset previous coeffs */
            *prev_fb = DEFAULT_BETA as i16;
            prev_sfb.fill(DEFAULT_BETA as i16);
        }
        *is_short_prev = b_is_short_block as u8;

        if work.b_has_bandwise_coeffs[pair] == 0 {
            /* use fullband coeff */
            if work.b_delta_time[pair] > 0 {
                last_val = *prev_fb as i32;
            } else {
                last_val = DEFAULT_BETA;
            }

            /* inverse dpcm */
            coeff_q = last_val + work.pair_coeff_delta_fb[pair] as i32;

            /* wrap around */
            if coeff_q >= CODE_BOOK_BETA_LAV as i32 {
                coeff_q -= CODE_BOOK_BETA_LAV as i32;
            }

            if coeff_q < 0 || coeff_q >= CODE_BOOK_BETA_LAV as i32 {
                return -1;
            }

            *prev_fb = coeff_q as i16;

            let mut band_mod_mct_bands_per_window = 0;
            for band in 0..num_bands {
                /* set all coeffs to fullband coeff */
                pair_coeff_q_sfb[band as usize] = coeff_q as i16;

                /* set previous coeffs according to mctMask */
                if read_mask(mct_mask, band) > 0 {
                    prev_sfb[band_mod_mct_bands_per_window as usize] = coeff_q as i16;
                } else {
                    prev_sfb[band_mod_mct_bands_per_window as usize] = DEFAULT_BETA as i16;
                }
                band_mod_mct_bands_per_window += 1;
                if band_mod_mct_bands_per_window == mct_bands_per_window {
                    band_mod_mct_bands_per_window = 0;
                }
            }
        } else {
            /* use bandwise coeffs */
            last_val = DEFAULT_BETA;

            let mut band_mod_mct_bands_per_window = 0;
            for band in 0..num_bands {
                if work.b_delta_time[pair] > 0 {
                    /* modulo operation to use previous short block of same frame, no effect for long blocks
                     */
                    last_val = prev_sfb[band_mod_mct_bands_per_window as usize] as i32;
                } else {
                    /* reset freqency differential per shortblock to avoid predicting from highest to lowest
                     * freq */
                    if band_mod_mct_bands_per_window == 0 {
                        last_val = DEFAULT_BETA;
                    }
                }

                if read_mask(mct_mask, band) > 0 {
                    /* inverse dpcm */
                    coeff_q = last_val + work.pair_coeff_delta_sfb[pair][band as usize] as i32;

                    /* wrap around */
                    if coeff_q >= CODE_BOOK_BETA_LAV as i32 {
                        coeff_q -= CODE_BOOK_BETA_LAV as i32;
                    }

                    if coeff_q < 0 || coeff_q >= CODE_BOOK_BETA_LAV as i32 {
                        return -1;
                    }

                    pair_coeff_q_sfb[band as usize] = coeff_q as i16;
                    last_val = coeff_q;
                    /* shortblock modulo operation only for previous values */
                    prev_sfb[band_mod_mct_bands_per_window as usize] = coeff_q as i16;
                } else {
                    /* reset coeffs for inactive bands */
                    prev_sfb[band_mod_mct_bands_per_window as usize] = DEFAULT_BETA as i16;
                }
                band_mod_mct_bands_per_window += 1;
                if band_mod_mct_bands_per_window == mct_bands_per_window {
                    band_mod_mct_bands_per_window = 0;
                }
            }

            /* reset fullband coeff */
            *prev_fb = DEFAULT_BETA as i16;
        }

        /* reset previous coeffs for bands above mct mask */
        for band in mct_bands_per_window as usize..MAX_NUM_MCT_BANDS {
            prev_sfb[band] = DEFAULT_BETA as i16;
        }
    }

    mct_bands_per_window
}

/// `applyMctInverseRotationFrame(left, lScale, right, rScale, prev_dmx, alphaQ, nSamples)`
fn apply_mct_inverse_rotation_frame(
    left: &[FixpDbl],
    l_scale: i32,
    right: &[FixpDbl],
    r_scale: i32,
    prev_dmx: &mut [FixpDbl],
    alpha_q: i16,
    n_samples: i32,
) {
    let cos_alpha: FixpSgl = TAB_INDEX_TO_COS_SIN_ALPHA[alpha_q as usize].re;
    let sin_alpha: FixpSgl = TAB_INDEX_TO_COS_SIN_ALPHA[alpha_q as usize].im;

    for n in 0..n_samples as usize {
        prev_dmx[n] = (f_mult_div2(left[n], cos_alpha) >> l_scale)
            .wrapping_add(f_mult_div2(right[n], sin_alpha) >> r_scale);
    }
}

/// `applyMctRotationIdx(dmx, dmxExp, res, resExp, alphaQ, nSamples)`: `s` holds the lines of the
/// band (`dmx`, `res`), `e[sfb]` the exponents (`dmxExp`, `resExp`).
fn apply_mct_rotation_idx(
    s: &mut Pair<[FixpDbl]>,
    e: &mut Pair<[i16]>,
    sfb: usize,
    alpha_q: i16,
    n_samples: i32,
) {
    let mut l_scale;
    let mut r_scale;
    let temp = e.a()[sfb] as i32 - e.b()[sfb] as i32;
    if temp >= 0 {
        l_scale = 0;
        r_scale = temp;
    } else {
        r_scale = 0;
        l_scale = -temp;
    }

    let out_exp = e.a()[sfb] as i32 + l_scale + 1;

    l_scale = f_min(31, l_scale);
    r_scale = f_min(31, r_scale);

    let cos_alpha: FixpSgl = TAB_INDEX_TO_COS_SIN_ALPHA[alpha_q as usize].re;
    let sin_alpha: FixpSgl = TAB_INDEX_TO_COS_SIN_ALPHA[alpha_q as usize].im;
    let n_samples = n_samples as usize;

    if cos_alpha == 0 && sin_alpha > 0 {
        for n in 0..n_samples {
            let temp_dmx = s.a()[n];
            s.a()[n] = s.b()[n].wrapping_neg();
            s.b()[n] = temp_dmx;
        }

        let temp_exp = e.a()[sfb];
        e.a()[sfb] = e.b()[sfb];
        e.b()[sfb] = temp_exp;
    } else if cos_alpha == 0 && sin_alpha < 0 {
        for n in 0..n_samples {
            let temp_dmx = s.a()[n];
            s.a()[n] = s.b()[n];
            s.b()[n] = temp_dmx.wrapping_neg();
        }

        let temp_exp = e.a()[sfb];
        e.a()[sfb] = e.b()[sfb];
        e.b()[sfb] = temp_exp;
    } else if sin_alpha == 0 && cos_alpha > 0 {
        /*Do nothing*/
    } else {
        for n in 0..n_samples {
            let temp_dmx = s.a()[n];
            let temp_res = s.b()[n];
            s.a()[n] = (f_mult_div2(temp_dmx, cos_alpha) >> l_scale)
                .wrapping_sub(f_mult_div2(temp_res, sin_alpha) >> r_scale);
            s.b()[n] = (f_mult_div2(temp_dmx, sin_alpha) >> l_scale)
                .wrapping_add(f_mult_div2(temp_res, cos_alpha) >> r_scale);
        }

        let headroom = get_scalefactor(&s.a()[..n_samples]) - 1;
        scale_values(&mut s.a()[..n_samples], headroom);
        e.a()[sfb] = (out_exp - headroom) as i16;

        let headroom = get_scalefactor(&s.b()[..n_samples]) - 1;
        scale_values(&mut s.b()[..n_samples], headroom);
        e.b()[sfb] = (out_exp - headroom) as i16;
    }
}

/// `applyMctPrediction(dmx, dmxExp, res, resExp, alphaQ, nSamples, predDir)`: `s` holds the lines
/// of the band (`dmx`, `res`), `e[sfb]` the exponents (`dmxExp`, `resExp`).
fn apply_mct_prediction(
    s: &mut Pair<[FixpDbl]>,
    e: &mut Pair<[i16]>,
    sfb: usize,
    alpha_q: i16,
    n_samples: i32,
    pred_dir: i32,
) {
    let mut l_scale;
    let mut r_scale;

    /* 0.1 in Q-3.34 */
    const POINT_ONE: FixpDbl = 0x66666666;

    let mut alpha_re_tmp: FixpDbl = 0;
    let mut alpha_exp = 0;

    if alpha_q != 0 {
        /* Find the minimum common headroom for alpha_re and alpha_im */
        let alpha_re_headroom = count_leading_bits((alpha_q as i32) << 16);

        /* Multiply alpha by 0.1 with maximum precision */
        alpha_re_tmp = f_mult(((alpha_q as i32) << alpha_re_headroom) as FixpSgl, POINT_ONE);

        /* Calculate shift exponent */
        /* (Q-3.34 *( Q15.0 - headroom)*/
        alpha_exp = 15 - alpha_re_headroom + (-3);
    }

    /* Upmix process */
    let igf_pred_dir: i32 = if pred_dir != 0 { -1 } else { 1 };

    let temp = e.a()[sfb] as i32 - e.b()[sfb] as i32;
    if temp >= 0 {
        l_scale = 2;
        r_scale = 2 + temp;
    } else {
        r_scale = 2;
        l_scale = -temp + 2;
    }

    if alpha_exp > 1 {
        l_scale += alpha_exp - 1;
        r_scale += alpha_exp - 1;
    }

    let out_exp = e.a()[sfb] as i32 + l_scale;

    let mut diff_scaling = l_scale - alpha_exp;

    diff_scaling = f_min(31, diff_scaling);
    l_scale = f_min(31, l_scale);
    r_scale = f_min(31, r_scale);

    let n_samples = n_samples as usize;
    for n in 0..n_samples {
        let mut temp_r = s.b()[n];
        let mut temp_l = s.a()[n];

        /* alpha_re[i] * specL[i] */
        let help1 = f_mult(alpha_re_tmp, temp_l) >> diff_scaling;

        temp_l >>= l_scale;
        temp_r >>= r_scale;

        /* side = specR[i] + alpha_re[i] * specL[i]*/
        let help2 = temp_r.wrapping_add(help1);

        /* specR[i] = -/+ (specL[i] + side; */
        s.b()[n] = temp_l.wrapping_sub(help2).wrapping_mul(igf_pred_dir);

        /* specL[i] = specL[i] + side; */
        s.a()[n] = temp_l.wrapping_add(help2);
    }

    let headroom = get_scalefactor(&s.a()[..n_samples]);
    scale_values(&mut s.a()[..n_samples], headroom);
    e.a()[sfb] = (out_exp - headroom) as i16;

    let headroom = get_scalefactor(&s.b()[..n_samples]);
    scale_values(&mut s.b()[..n_samples], headroom);
    e.b()[sfb] = (out_exp - headroom) as i16;
}

/// `applyMctRotationWrapper(self, dmx, dmxSfbExp, res, resSfbExp, alphaQSfb, mctMask,
/// numMctBands, totalSfb, pair, BandOffsets)`: `s` holds the spectra of the window (`dmx`,
/// `res`), `e` the exponents of the window (`dmxSfbExp`, `resSfbExp`).
#[allow(clippy::too_many_arguments)]
fn apply_mct_rotation_wrapper(
    self_: &CMct,
    s: &mut Pair<[FixpDbl]>,
    e: &mut Pair<[i16]>,
    alpha_q_sfb: &[i16],
    mct_mask: u64,
    num_mct_bands: i32,
    total_sfb: i32,
    pair: usize,
    band_offsets: &[i16],
) {
    let work = &*self_.mct_work;

    if self_.mcc_signaling_type == 0 {
        /* apply fullband box */
        if work.b_has_bandwise_coeffs[pair] == 0 && work.b_has_mct_mask[pair] == 0 {
            let alpha = alpha_q_sfb[0];
            let pred_dir = work.pred_dir[pair] as i32;
            for sfb in 0..total_sfb as usize {
                let start_line = band_offsets[sfb] as usize;
                let stop_line = band_offsets[sfb + 1] as usize;
                let n_samples = (stop_line - start_line) as i32;
                apply_mct_prediction(
                    &mut s.map(|x| &mut x[start_line..stop_line]),
                    e,
                    sfb,
                    alpha,
                    n_samples,
                    pred_dir,
                );
            }
        } else {
            /* apply bandwise processing */
            let sfb_max = f_min(total_sfb, num_mct_bands * 2);
            let pred_dir = work.pred_dir[pair] as i32;
            for sfb in 0..sfb_max {
                let i = sfb / 2;
                if read_mask(mct_mask, i) == 0 {
                    continue;
                }
                let sfb = sfb as usize;
                let start_line = band_offsets[sfb] as usize;
                let stop_line = band_offsets[sfb + 1] as usize;
                let n_samples = (stop_line - start_line) as i32;
                apply_mct_prediction(
                    &mut s.map(|x| &mut x[start_line..stop_line]),
                    e,
                    sfb,
                    alpha_q_sfb[i as usize],
                    n_samples,
                    pred_dir,
                );
            }
        }
    } else if self_.mcc_signaling_type == 1 {
        /* apply fullband box */
        if work.b_has_bandwise_coeffs[pair] == 0 && work.b_has_mct_mask[pair] == 0 {
            let alpha = alpha_q_sfb[0];
            for sfb in 0..total_sfb as usize {
                let start_line = band_offsets[sfb] as usize;
                let stop_line = band_offsets[sfb + 1] as usize;
                let n_samples = (stop_line - start_line) as i32;
                apply_mct_rotation_idx(
                    &mut s.map(|x| &mut x[start_line..stop_line]),
                    e,
                    sfb,
                    alpha,
                    n_samples,
                );
            }
        } else {
            /* apply bandwise processing */
            let sfb_max = f_min(total_sfb, num_mct_bands * 2);
            for sfb in 0..sfb_max {
                let i = sfb / 2;
                if read_mask(mct_mask, i) == 0 {
                    continue;
                }
                let sfb = sfb as usize;
                let start_line = band_offsets[sfb] as usize;
                let stop_line = band_offsets[sfb + 1] as usize;
                let n_samples = (stop_line - start_line) as i32;
                apply_mct_rotation_idx(
                    &mut s.map(|x| &mut x[start_line..stop_line]),
                    e,
                    sfb,
                    alpha_q_sfb[i as usize],
                    n_samples,
                );
            }
        }
    }
}

/// `CMct_StereoFilling_GetPreviousDmx(self, pairCoeffQSfb, pairCoeffQFb,
/// pAacDecoderStaticChannelInfo, pAacDecoderChannelInfo, prevSpec1, prevSpec1_exp, prevSpec2,
/// prevSpec2_exp, dmx_prev, dmx_prev_win_exp, pair, pScaleFactorBandOffsets, pWindowGroupLength,
/// windowGroups, max_noise_sfb, band_is_noise, applyPrediction, mctBandsPerWindow)`.
///
/// The unused parameters `pairCoeffQFb` and `pAacDecoderStaticChannelInfo` are omitted; of
/// `pAacDecoderChannelInfo` only `icsInfo` and `granuleLength` are used.
#[allow(clippy::too_many_arguments)]
fn cmct_stereo_filling_get_previous_dmx(
    self_: &CMct,
    pair_coeff_q_sfb: &[i16],
    ics_info: &CIcsInfo,
    granule_length: i32,
    prev_spec1: &[FixpDbl],
    prev_spec1_exp: &[i16],
    prev_spec2: &[FixpDbl],
    prev_spec2_exp: &[i16],
    dmx_prev: &mut [FixpDbl],
    dmx_prev_win_exp: &mut [i16],
    pair: usize,
    p_scale_factor_band_offsets: &[i16],
    p_window_group_length: &[u8; 8],
    window_groups: i32,
    max_noise_sfb: i32,
    band_is_noise: &mut [u8],
    apply_prediction: i32,
    mct_bands_per_window: i32,
) {
    let mut l_scale;
    let mut r_scale;
    let mut temp;
    let work = &*self_.mct_work;
    let offsets = p_scale_factor_band_offsets;

    let noise_filling_start_offset =
        if get_window_sequence(ics_info) == BLOCK_SHORT { 20 } else { 160 };

    /* determine sfb from where on noise filling is applied */
    let mut sfb = 0usize;
    while (offsets[sfb] as i32) < noise_filling_start_offset {
        sfb += 1;
    }
    let nf_start_offset_sfb = sfb;
    let max_noise_sfb = max_noise_sfb.max(0) as usize;

    if self_.mcc_signaling_type == 0 {
        if apply_prediction != 0 {
            let mut window = 0usize;
            for group in 0..window_groups as usize {
                for _groupwin in 0..p_window_group_length[group] {
                    let spec_off = window * granule_length as usize;

                    for sfb in nf_start_offset_sfb..max_noise_sfb {
                        /* Run algorithm only if the particular band has been fully noise filled, i.e. that it
                        has been empty before the noise-filling process */
                        if band_is_noise[group * 16 + sfb] == 0 {
                            continue;
                        }

                        /* Find if a difference between the scalings exists.*/
                        /* Add 1 so that we multiply by 0.5 */
                        let ei = window * 16 + sfb;
                        temp = prev_spec1_exp[ei] as i32 - prev_spec2_exp[ei] as i32;
                        if temp >= 0 {
                            l_scale = 1;
                            r_scale = temp + 1;
                            r_scale = f_min(r_scale, 31);
                            dmx_prev_win_exp[ei] = prev_spec1_exp[ei];
                        } else {
                            l_scale = -temp + 1;
                            l_scale = f_min(l_scale, 31);
                            r_scale = 1;
                            dmx_prev_win_exp[ei] = prev_spec2_exp[ei];
                        }

                        for index in offsets[sfb] as usize..offsets[sfb + 1] as usize {
                            /* MS output generation */
                            let temp_left = prev_spec1[spec_off + index] >> l_scale;
                            let temp_right = prev_spec2[spec_off + index] >> r_scale;

                            if work.pred_dir[pair] == 0 {
                                dmx_prev[spec_off + index] = temp_left.wrapping_add(temp_right);
                            } else {
                                dmx_prev[spec_off + index] = temp_left.wrapping_sub(temp_right);
                            }
                        }
                    } /*for (band=0; band<scaleFactorBandsTransmitted; band++)*/
                    window += 1;
                } /*  for (int groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++)*/
            } /* window */
        }
    }
    /*  if(self->MCCSignalingType == 0) */
    else if work.b_has_bandwise_coeffs[pair] == 0 && work.b_has_mct_mask[pair] == 0 {
        let alpha_q = pair_coeff_q_sfb[0];

        let mut window = 0usize;
        for group in 0..window_groups as usize {
            for _groupwin in 0..p_window_group_length[group] {
                let spec_off = window * granule_length as usize;

                for sfb in nf_start_offset_sfb..max_noise_sfb {
                    /* Run algorithm only if the particular band has been fully noise filled, i.e. that it has
                    been empty before the noise-filling process */
                    if band_is_noise[group * 16 + sfb] == 0 {
                        continue;
                    }

                    /* Find if a difference between the scalings exists. */
                    let start = spec_off + offsets[sfb] as usize;
                    let width = (offsets[sfb + 1] - offsets[sfb]) as i32;
                    let ei = window * 16 + sfb;

                    temp = prev_spec1_exp[ei] as i32 - prev_spec2_exp[ei] as i32;
                    if temp > 0 {
                        l_scale = 0;
                        r_scale = f_min(temp, 31);
                        dmx_prev_win_exp[ei] = prev_spec1_exp[ei].wrapping_add(1);
                    } else {
                        l_scale = f_min(-temp, 31);
                        r_scale = 0;
                        dmx_prev_win_exp[ei] = prev_spec2_exp[ei].wrapping_add(1);
                    }

                    apply_mct_inverse_rotation_frame(
                        &prev_spec1[start..],
                        l_scale,
                        &prev_spec2[start..],
                        r_scale,
                        &mut dmx_prev[start..],
                        alpha_q,
                        width,
                    );

                    clean_sfb_band_mct_stefi_dmx(
                        &mut dmx_prev[start..start + width as usize],
                        dmx_prev_win_exp[ei] as i32,
                    );
                } /*for (band=0; band<scaleFactorBandsTransmitted; band++)*/
                window += 1;
            } /*  for (int groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++)*/
        } /* window */
    } else {
        let mut mct_band_offset = 0;

        let mut window = 0usize;
        for group in 0..window_groups as usize {
            for _groupwin in 0..p_window_group_length[group] {
                let spec_off = window * granule_length as usize;

                for sfb in nf_start_offset_sfb..max_noise_sfb {
                    let mct_band = (sfb >> 1) as i32;

                    /* Run algorithm only if the particular band has been fully noise filled, i.e. that it has
                    been empty before the noise-filling process */
                    if band_is_noise[group * 16 + sfb] == 0 {
                        continue;
                    }

                    let start = spec_off + offsets[sfb] as usize;
                    let ei = window * 16 + sfb;

                    if mct_band < mct_bands_per_window
                        && read_mask(work.mct_mask[pair], mct_band + mct_band_offset) != 0
                    {
                        /* Find if a difference between the scalings exists. */
                        temp = prev_spec1_exp[ei] as i32 - prev_spec2_exp[ei] as i32;
                        if temp > 0 {
                            l_scale = 0;
                            r_scale = f_min(temp, 31);
                            dmx_prev_win_exp[ei] = prev_spec1_exp[ei].wrapping_add(1);
                        } else {
                            l_scale = f_min(-temp, 31);
                            r_scale = 0;
                            dmx_prev_win_exp[ei] = prev_spec2_exp[ei].wrapping_add(1);
                        }

                        let alpha_q = pair_coeff_q_sfb[(mct_band + mct_band_offset) as usize];

                        let width = (offsets[sfb + 1] - offsets[sfb]) as i32;
                        apply_mct_inverse_rotation_frame(
                            &prev_spec1[start..],
                            l_scale,
                            &prev_spec2[start..],
                            r_scale,
                            &mut dmx_prev[start..],
                            alpha_q,
                            width,
                        );

                        clean_sfb_band_mct_stefi_dmx(
                            &mut dmx_prev[start..start + width as usize],
                            dmx_prev_win_exp[ei] as i32,
                        );
                    } else {
                        let width = (offsets[sfb + 1] - offsets[sfb]) as usize;

                        dmx_prev[start..start + width]
                            .copy_from_slice(&prev_spec1[start..start + width]);

                        band_is_noise[group * 16 + sfb] = 0;

                        dmx_prev_win_exp[ei] = prev_spec1_exp[ei];
                    }
                } /*for (band=0; band<scaleFactorBandsTransmitted; band++)*/

                mct_band_offset += mct_bands_per_window;
                window += 1;
            } /*  for (int groupwin=0; groupwin<pWindowGroupLength[group]; groupwin++, window++)*/
        } /* window */
    }

    /* MS stereo */
}

/// `CMct_StereoFilling_Calc_Energy_SFBwise(chInfo, icsInfo, samplingRateInfo, Energy,
/// Energy_exp)`: `spec` is `chInfo->pSpectralCoefficient`, `a_sfb_scale` is
/// `chInfo->pDynData->aSfbScale`. Returns `(Energy, Energy_exp)`.
fn cmct_stereo_filling_calc_energy_sfbwise(
    spec: &[FixpDbl],
    a_sfb_scale: &[i16],
    ics_info: &CIcsInfo,
    sampling_rate_info: &SamplingRateInfo,
) -> (FixpDbl, i32) {
    let mut dmx_energy: FixpDbl = 0;

    let noise_filling_start_offset = 160;

    /* determine sfb from where on noise filling is applied */
    let p_scale_factor_band_offsets = get_scale_factor_band_offsets(ics_info, sampling_rate_info);
    let mut sfb = 0usize;
    while (p_scale_factor_band_offsets[sfb] as i32) < noise_filling_start_offset {
        sfb += 1;
    }
    let nf_start_offset_sfb = sfb;

    let end_offset = 1024 >> 1; /* !! */
    sfb = 0;
    while (p_scale_factor_band_offsets[sfb] as i32) < end_offset {
        sfb += 1;
    }
    let end_offset_sfb = sfb;

    /* Get the sfb scale data */
    let mut p2_exponent_dmx = nf_start_offset_sfb;

    /*Find a suitable headroom for subsequent additions */
    let sfb_width = p_scale_factor_band_offsets[nf_start_offset_sfb + 1] as i32
        - p_scale_factor_band_offsets[nf_start_offset_sfb] as i32;
    let mut sfb_shift = 0;
    loop {
        let temp = 1i32 << sfb_shift;
        if temp >= sfb_width {
            break;
        } else {
            sfb_shift += 1;
        }
    }

    let mut max_scale_factor_dmx = ((a_sfb_scale[p2_exponent_dmx] as i32) << 1) + sfb_shift + 1;

    for sfb in nf_start_offset_sfb..end_offset_sfb {
        let bin_start = p_scale_factor_band_offsets[sfb] as usize;
        let bin_stop = p_scale_factor_band_offsets[sfb + 1] as usize;
        let width = (bin_stop - bin_start) as i32;

        /*Find a suitable headroom for subsequent additions */
        let mut shift = 0;
        loop {
            let temp = 1i32 << shift;
            if temp >= width {
                break;
            } else {
                shift += 1;
            }
        }

        let mut temp_dmx_energy: FixpDbl = 0;
        for &v in &spec[bin_start..bin_stop] {
            temp_dmx_energy = temp_dmx_energy.wrapping_add(f_pow2_div2(v) >> shift);
        }

        if temp_dmx_energy != 0 {
            let energy_adjust_shift = ((a_sfb_scale[p2_exponent_dmx] as i32) << 1) + shift + 1;
            let diff;

            if energy_adjust_shift > max_scale_factor_dmx {
                diff = f_min(energy_adjust_shift - max_scale_factor_dmx, 31);
                dmx_energy >>= diff;
                dmx_energy = dmx_energy.wrapping_add(temp_dmx_energy);
                max_scale_factor_dmx = energy_adjust_shift;
            } else {
                diff = f_min(max_scale_factor_dmx - energy_adjust_shift, 31);
                temp_dmx_energy >>= diff;
                dmx_energy = dmx_energy.wrapping_add(temp_dmx_energy);
            }

            /* Adjust not to overflow */
            if cnt_leading_zeros(dmx_energy) == 1 {
                dmx_energy >>= 1;
                max_scale_factor_dmx += 1;
            }
        } /* if(temp_dmx_energy!=(FIXP_DBL)0) */

        p2_exponent_dmx += 1;
    } /* for (sfb= nfStartOffset_sfb; sfb < EndOffset_sfb; sfb++) */

    (dmx_energy, max_scale_factor_dmx)
}

/// `CMct_StereoFilling_SyncRNGs(stChInfo1, stChInfo2, chInfo1, chInfo2, icsInfo1, icsInfo2,
/// samplingRateInfo)`: `spec` holds the spectra of the two channels.
fn cmct_stereo_filling_sync_rngs(
    st_info: &mut Pair<Box<CAacDecoderStaticChannelInfo>>,
    ch_info: &mut Pair<Box<CAacDecoderChannelInfo>>,
    spec: &mut Pair<[FixpDbl]>,
    ics_info1: &CIcsInfo,
    ics_info2: &CIcsInfo,
    sampling_rate_info: &SamplingRateInfo,
) {
    /* Whitening is applied only to long frames */
    if is_long_block(ics_info1) != 0 && is_long_block(ics_info2) != 0 {
        /* Check if Strong whitening is used in some of the tiles */
        let num_of_tiles = iis_igf_dec_lib_get_number_of_tiles(&st_info.b().igf_static_data, 0);
        let mut sync_rng_flag = 0;
        for i in 0..num_of_tiles {
            let level = get_tile_whitening_level(&ch_info.b().igf_data, i);
            if level == WHITENING_STRONG {
                sync_rng_flag = 1;
                break;
            }
        }

        /* When Strong whitening is used syncronize the RNGs */
        if sync_rng_flag == 1 {
            let (mut energy_dmx, energy_dmx_exp) = cmct_stereo_filling_calc_energy_sfbwise(
                spec.a(),
                &ch_info.a().p_dyn_data.a_sfb_scale,
                ics_info1,
                sampling_rate_info,
            );
            let (mut energy_res, energy_res_exp) = cmct_stereo_filling_calc_energy_sfbwise(
                spec.b(),
                &ch_info.b().p_dyn_data.a_sfb_scale,
                ics_info2,
                sampling_rate_info,
            );

            /* Allign exponents of dmx and res */
            let mut diff = energy_dmx_exp - energy_res_exp;
            if diff >= 0 {
                diff = f_min(diff, 31);
                energy_res >>= diff;
            } else {
                diff = f_min(-diff, 31);
                energy_dmx >>= diff;
            }

            /* if (enDmx * 0.125f > enRes) */
            energy_dmx >>= 3;
            if energy_dmx > energy_res {
                let seed = st_info.a().nf_random_seed;
                st_info.b().nf_random_seed = seed;
            }
        } /* if(SyncRNGflag ==(INT)1) */
    } /* if(IsLongBlock(icsInfo1) && IsLongBlock(icsInfo2)) */
}

/// `MCT_StereoFilling(pSpec, pSpec_exp, pAacDecoderChannelInfo, pSamplingRateInfo, max_noise_sfb,
/// dmx_prev_modified, dmx_prev_modified_exp, band_is_noise)`: of `pAacDecoderChannelInfo` only
/// `icsInfo` and `pDynData->aScaleFactor` are used (`ics_info`, `a_scale_factor`).
#[allow(clippy::too_many_arguments)]
fn mct_stereo_filling(
    p_spec: &mut [FixpDbl],
    p_spec_exp: &mut [i16],
    ics_info: &CIcsInfo,
    a_scale_factor: &[i16],
    p_sampling_rate_info: &SamplingRateInfo,
    max_noise_sfb: i32,
    dmx_prev_modified: &mut [FixpDbl],
    dmx_prev_modified_exp: &mut [i16],
    band_is_noise: &[u8],
) {
    /* Comment:
     This algorithm does not work the same way as the Reference software. This is due to the
    datatypes being available as input. The RefSoft takes the scaled input, unscales it, works with
    un-scaled data and finally scales the data back with the orginal scaling factor. The present
    implemenation avoids this scaling/rescaling by working with scaled input data altogether. This
    leads to different computational values throughout the algorithm. In fact direct one to one
    intermediate value matching between RefSoft and the present one will be impossible. Just the
    output values will coincide. */

    let swb_offset = get_scale_factor_band_offsets(ics_info, p_sampling_rate_info);

    /* Only for long frames */
    let noise_filling_start_offset = 160;

    /* determine sfb from where on noise filling is applied */
    let mut sfb = 0usize;
    while (swb_offset[sfb] as i32) < noise_filling_start_offset {
        sfb += 1;
    }
    let nf_start_offset_sfb = sfb;

    for sfb in nf_start_offset_sfb..max_noise_sfb.max(0) as usize {
        /* Run algorithm only if the particular band has been fully noise filled, i.e. that it has been
        empty before the noise-filling process */
        if band_is_noise[sfb] == 0 {
            continue;
        }

        let bin_start: i16 = swb_offset[sfb];
        let bin_stop: i16 = swb_offset[sfb + 1];

        /* Calculate the widht of the particular sfb */
        let sfb_width: i16 = bin_stop - bin_start;

        let bins = bin_start as usize..bin_stop as usize;

        /* Find a suitable shift for dmx addition */
        let shift = DFRACT_BITS - f_normz(sfb_width as FixpDbl);

        let head_shift_sfb = get_scalefactor(&p_spec[bins.clone()]);

        /* Calculate the energy in the particular SFB */
        let mut en_res: FixpDbl = 0;
        let mut en_res_e: i32 = 0;
        if head_shift_sfb != 31 {
            for bin in bins.clone() {
                en_res = en_res.wrapping_add(f_pow2_div2(p_spec[bin] << head_shift_sfb) >> shift);
            } /* for (bin...) */

            en_res_e = ((p_spec_exp[sfb] as i32 - head_shift_sfb) << 1) + 1 + shift + 4;
        }

        /* Generate const "gain * 1" */
        let lsb = a_scale_factor[sfb] as i32 & 3;
        let msb = a_scale_factor[sfb] as i32 >> 2;
        let scaled_one: FixpDbl = MANTISSA_TABLE[lsb as usize][0];
        let scaled_one_exp: i16 = (msb + 1) as i16;

        /* Calculate the energy of 1 */
        let energy_of_one: FixpDbl = f_pow2(scaled_one);

        /* Multiply spectrum by 4 implicitly */
        p_spec_exp[sfb] = p_spec_exp[sfb].wrapping_add(2);

        /* Calculate enTarget. It is the lenght of the sfb mutiplied by the scaled energy */
        let length_bits = count_leading_bits((sfb_width as i32) << 16);
        let length_e = 15 - length_bits;
        let mut en_target: FixpDbl =
            f_mult(energy_of_one, ((sfb_width as i32) << length_bits) as FixpSgl);
        let mut en_target_e = ((scaled_one_exp as i32) << 1) + length_e;

        /* Alling the exponents of the two energy values*/
        let mut delta = en_target_e - en_res_e;

        let energy_difference_e;
        if delta > 0 {
            delta = f_min(31, delta);
            en_res >>= delta;
            energy_difference_e = en_target_e;
            en_res_e = en_target_e;
        } else {
            delta = -delta;
            delta = f_min(31, delta);
            en_target >>= delta;
            energy_difference_e = en_res_e;
            en_target_e = en_res_e;
        }

        /* Subtract the two energy values */
        let energy_difference: FixpDbl = en_target.wrapping_sub(en_res);

        if energy_difference > 0 {
            /* Get the previous downmix of the particular window */
            let dmx_prev = &mut *dmx_prev_modified;

            let mut energy_dmx: FixpDbl = 0;
            let mut energy_dmx_e: i32 = 0;

            /* Find the available headroom of the downmix signal and a suitable shift value*/
            let head_shift_dmx = get_scalefactor(&dmx_prev[bins.clone()]);

            /* Calculate the energy of the previous downmix. If it is non-zero, then the
            downmix will be scaled and added to the signal.*/
            if head_shift_dmx != 31 {
                /* "1e-8" */
                const VALUE0_001: FixpDbl = 0x55e63b80;
                const VALUE0_001_EXP: i32 = -26;

                /* Calculate "energy_dmx" exponent of the particular window */
                let dmx_exp =
                    ((dmx_prev_modified_exp[sfb] as i32 - head_shift_dmx) << 1) + 1 + shift;

                delta = dmx_exp - VALUE0_001_EXP;

                /*If the energy is comparable to the threshold, then calculate it.
                Otherwise, simply neglect it (set it to zero)*/
                if delta > 0 {
                    for bin in bins.clone() {
                        energy_dmx = energy_dmx
                            .wrapping_add(f_pow2_div2(dmx_prev[bin] << head_shift_dmx) >> shift);
                    } /* for (bin...) */

                    delta = f_min(31, delta);
                    energy_dmx = energy_dmx.wrapping_add(VALUE0_001 >> delta);
                    energy_dmx_e = dmx_exp;
                }
            }

            if energy_dmx > 0 {
                let mut temp_int = 0;

                /* Divide by energy_sfb. Calculate (enTarget - enRes) / energy_dmx */
                let temp_fixp_dbl = f_div_norm(energy_difference, energy_dmx, &mut temp_int);

                /* Correct the output exponent */
                temp_int += energy_difference_e - energy_dmx_e;

                /* Calculate sqrt((enTarget - enRes) / energy_dmx) */
                let mut tmp = sqrt_fixp_lookup_e(temp_fixp_dbl, &mut temp_int);

                /* We have to limit the gain we compute according to the RefSoft to 10. In RefSoft
                however they use unscaled data  while we use scaled data. Therefore we have to
                compare to a scaled threshold, i.e. 10* "Scaled_One" */
                const TEN: FixpSgl = 0x5000;
                const TEN_EXP: i32 = 4;
                let ten_scaled: FixpDbl = f_mult(TEN, scaled_one);
                let ten_scaled_exp = TEN_EXP + scaled_one_exp as i32;

                /* Temporary values for comparison purposes */
                let mut temp_ten_scaled = ten_scaled;
                let mut temp_tmp = tmp;

                /* We align the values to the same exponent */
                delta = temp_int - ten_scaled_exp;
                if delta >= 0 {
                    delta = f_min(31, delta);
                    temp_ten_scaled >>= delta;
                } else {
                    delta = -delta;
                    delta = f_min(31, delta);
                    temp_tmp >>= delta;
                }
                /* Compare and limit if necessary */
                if temp_tmp > temp_ten_scaled {
                    tmp = ten_scaled;
                    temp_int = ten_scaled_exp;
                }

                /* Modify the downmix so that it can be added to the spectrum later: "dmx_prev*tmp" */
                for bin in bins.clone() {
                    dmx_prev[bin] = f_mult_div2(dmx_prev[bin] << head_shift_dmx, tmp);
                } /* for (bin...) */

                /* "dmx_prev*tmp" exponent for every sfb and window */
                dmx_prev_modified_exp[sfb] =
                    ((dmx_prev_modified_exp[sfb] as i32 - head_shift_dmx + 1) + temp_int) as i16;

                /*Adjust exponents. Add and compute new energy*/
                let mut diff_1 = dmx_prev_modified_exp[sfb] as i32 - p_spec_exp[sfb] as i32;
                /* Adjust binary point and add. Calculate energy after addition of the dmx */
                en_res = 0;
                if diff_1 > 0 {
                    diff_1 = f_min(31, diff_1);
                    for bin in bins.clone() {
                        let temp = dmx_prev[bin].wrapping_add(p_spec[bin] >> diff_1);
                        p_spec[bin] = temp;
                        en_res = en_res.wrapping_add(f_pow2(temp) >> shift);
                    }
                    p_spec_exp[sfb] = dmx_prev_modified_exp[sfb];
                } else {
                    /* Shift right by 1 bit more to guarantee headroom for addition */
                    diff_1 = -diff_1 + 1;
                    diff_1 = f_min(31, diff_1);
                    for bin in bins.clone() {
                        let temp = (dmx_prev[bin] >> diff_1).wrapping_add(p_spec[bin] >> 1);
                        p_spec[bin] = temp;
                        en_res = en_res.wrapping_add(f_pow2(temp) >> shift);
                    }
                    p_spec_exp[sfb] = p_spec_exp[sfb].wrapping_add(1);
                }

                en_res_e = ((p_spec_exp[sfb] as i32) << 1) + shift;
            } /* if(energy_dmx != (FIXP_DBL)0) */

            if en_res > 0 {
                /*Calculate (enTarget / enRes) */
                let mut temp_int = 0;
                let temp_fixp_dbl = f_div_norm(en_target, en_res, &mut temp_int);
                temp_int += en_target_e - en_res_e;

                /*Calculate sqrt(enTarget / enRes) */
                let mut tmp = sqrt_fixp_lookup_e(temp_fixp_dbl, &mut temp_int);

                /* We have to limit the gain we compute according to the RefSoft to 10. */
                const TEN: FixpDbl = 0x50000000;
                const TEN_EXP: i32 = 4;

                /* Temporary values for comparison purposes */
                let mut temp_ten_scaled = TEN;
                let mut temp_tmp = tmp;

                /* We align the values to the same exponent */
                delta = temp_int - TEN_EXP;
                if delta >= 0 {
                    delta = f_min(31, delta);
                    temp_ten_scaled >>= delta;
                } else {
                    delta = -delta;
                    delta = f_min(31, delta);
                    temp_tmp >>= delta;
                }
                /* Compare and limit if necessary */
                if temp_tmp > temp_ten_scaled {
                    tmp = TEN;
                    temp_int = TEN_EXP;
                }

                /* Stereo filling changes the dynamic range of the signal,despite keeping the energy of
                the signal constant. */
                let head_shift = get_scalefactor(&p_spec[bins.clone()]);

                /* Modify spectrum */
                for bin in bins.clone() {
                    p_spec[bin] = f_mult_div2(p_spec[bin] << head_shift, tmp);
                } /* for (bin...) */

                /* Calculate output exponent */
                p_spec_exp[sfb] = ((p_spec_exp[sfb] as i32 - head_shift) + temp_int + 1) as i16;
            } /* if(enRes > 0) */
        } /* if(Energy_Difference > (FIXP_DBL)0) */
    } /* for (sfb...) */
}

/// `CMct_MCT_StereoFilling(self, streamInfo, pAacDecoderChannelInfo,
/// pAacDecoderStaticChannelInfo, samplingRateInfo, MCT_elFlags, usacIndepFrame)`: applies MCT
/// stereo filling and the inverse MCT rotations/predictions to the channels of the signal group.
/// Returns 0 on success, -1 on error.
///
/// - `aac_samples_per_frame`: `streamInfo->aacSamplesPerFrame` (the only field used).
/// - `p_aac_decoder_channel_info`, `p_aac_decoder_static_channel_info`, `mct_el_flags`: start at
///   the first channel of the signal group (`channelOffset`), indexed with `channelMap`.
/// - `spec`: the spectra of the channels (`pSpectralCoefficient` of channel `i` is
///   `spec[i * 1024..(i + 1) * 1024]`).
///
/// Deviations from the C++ code for corrupt bitstreams (undefined behaviour in C++): -1 is
/// returned if `numMctMaskBands[pair] > 64` (see `inverseDpcmAngleCoding`) and if stereo filling
/// needs the previous spectra of a group without MCT channels (`prevOutSpec == NULL`).
#[allow(clippy::too_many_arguments)]
pub fn cmct_mct_stereo_filling(
    self_: &mut CMct,
    aac_samples_per_frame: i32,
    p_aac_decoder_channel_info: &mut [Box<CAacDecoderChannelInfo>],
    p_aac_decoder_static_channel_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    spec: &mut [FixpDbl],
    sampling_rate_info: &SamplingRateInfo,
    mct_el_flags: &[u32],
    usac_indep_frame: u32,
) -> i32 {
    /* Sanity check */
    if self_.mct_work.num_pairs as usize > MAX_NUM_MCT_BOXES {
        return -1;
    }

    /* pairCoeffQSfb is a local array of the loop body in C++, i.e. its content is undefined for
    every pair. Only entries written by inverseDpcmAngleCoding() for the current pair are read,
    except for stale pairs with numMctMaskBands == 0 (never parsed, MCCSignalingType == 1),
    where alphaQSfb[0] is read without having been written; in the reference build this reads
    the stack slot of the array (the values of the previous pair). */
    let mut pair_coeff_q_sfb = [0i16; MAX_NUM_MCT_BANDS];

    for pair in 0..self_.mct_work.num_pairs as usize {
        let mut mct_band_offset: i32 = 0;
        let mut b_is_short_block = 0;

        let ch1 = self_.mct_work.code_pairs[pair][0] as usize;
        let ch2 = self_.mct_work.code_pairs[pair][1] as usize;
        let tag1 = self_.channel_map[ch1] as usize;
        let tag2 = self_.channel_map[ch2] as usize;

        let mut ch_info = pair_of(p_aac_decoder_channel_info, tag1, tag2);
        let mut st_info = pair_of(p_aac_decoder_static_channel_info, tag1, tag2);
        let mut spec_pair = pair_of_chunks(spec, tag1, tag2, 1024);

        let ics_info1 = ch_info.a().ics_info;
        let ics_info2 = ch_info.b().ics_info;

        let windows_per_frame = get_windows_per_frame(&ics_info1);

        /* sanity check */
        if windows_per_frame != get_windows_per_frame(&ics_info2) {
            return -1;
        }

        /* short block handling */
        if windows_per_frame > 1 {
            b_is_short_block = 1;
        }

        let mct_bands_per_window = inverse_dpcm_angle_coding(
            self_,
            &mut pair_coeff_q_sfb,
            pair,
            b_is_short_block,
            windows_per_frame,
        );

        if mct_bands_per_window < 0 {
            return -1;
        }

        if self_.mct_work.has_stereo_filling[pair] != 0 {
            /* Make sure the following code runs only for long blocks */
            if b_is_short_block == 0 {
                let zero_prev_out_spec1 = (ics_info1.window_sequence == BLOCK_SHORT)
                    != (st_info.a().concealment_info.window_sequence == BLOCK_SHORT);
                let zero_prev_out_spec2 = (ics_info2.window_sequence == BLOCK_SHORT)
                    != (st_info.b().concealment_info.window_sequence == BLOCK_SHORT);
                let b_no_frame_memory = usac_indep_frame != 0
                    || st_info.a().concealment_info.last_render_mode as i32 == AACDEC_RENDER_LPD
                    || st_info.b().concealment_info.last_render_mode as i32 == AACDEC_RENDER_LPD;

                /* Clear downmix for the case when it is not needed */
                let mut prev_dmx = [0 as FixpDbl; 1024];
                let mut prev_dmx_exp = [0i16; 8 * 16];
                /* uninitialized in C++; always written before it is read */
                let mut empty_buffer = [0 as FixpDbl; 1024];
                let mut empty_buffer_exp = [0i16; 8 * 16];

                let cp0 = self_.mct_work.code_pairs[pair][0] as usize;
                let cp1 = self_.mct_work.code_pairs[pair][1] as usize;

                /* If one can not compute previous dmx from both of the previous elements */
                if (zero_prev_out_spec1 && zero_prev_out_spec2) || b_no_frame_memory {
                    /* No previous DMX can be computed */
                }
                /* If one of the previous elements is available for computing previous dmx */
                else {
                    if (cp0.max(cp1) + 1) * 1024 > self_.prev_out_spec.len() {
                        /* numMctChannels == 0: prevOutSpec is NULL in C++ (undefined behaviour) */
                        mpeghdec_fdk::deviation::hit();
                        return -1;
                    }

                    let self_ref: &CMct = self_;
                    let mut prev_spec1: &[FixpDbl] = &self_ref.prev_out_spec[cp0 * 1024..][..1024];
                    let mut prev_spec2: &[FixpDbl] = &self_ref.prev_out_spec[cp1 * 1024..][..1024];
                    let mut prev_spec1_exp: &[i16] =
                        &self_ref.prev_out_spec_exp[cp0 * (8 * 16)..][..8 * 16];
                    let mut prev_spec2_exp: &[i16] =
                        &self_ref.prev_out_spec_exp[cp1 * (8 * 16)..][..8 * 16];

                    let apply_prediction = if zero_prev_out_spec1 || zero_prev_out_spec2 {
                        empty_buffer.fill(0);
                        empty_buffer_exp.fill(0);

                        if zero_prev_out_spec1 {
                            prev_spec1 = &empty_buffer;
                            prev_spec1_exp = &empty_buffer_exp;
                        } else {
                            prev_spec2 = &empty_buffer;
                            prev_spec2_exp = &empty_buffer_exp;
                        }

                        (zero_prev_out_spec1 && zero_prev_out_spec2) as i32
                    }
                    /* General case: both of the previous elements are available for computing previous dmx */
                    else {
                        1
                    }; /* else */

                    let c2 = ch_info.b();
                    cmct_stereo_filling_get_previous_dmx(
                        self_ref,
                        &pair_coeff_q_sfb,
                        &c2.ics_info,
                        c2.granule_length,
                        prev_spec1,
                        prev_spec1_exp,
                        prev_spec2,
                        prev_spec2_exp,
                        &mut prev_dmx,
                        &mut prev_dmx_exp,
                        pair,
                        get_scale_factor_band_offsets(&ics_info1, sampling_rate_info),
                        get_window_group_length_table(&ics_info1),
                        get_window_groups(&ics_info1) as i32,
                        ics_info2.max_sf_bands as i32,
                        &mut c2.p_dyn_data.band_is_noise,
                        apply_prediction,
                        mct_bands_per_window,
                    );
                }

                /* PrevDMX is changed/modified during Stero Filling so it must be preserved for the
                 * subsequent tile processing intact */
                let prev_dmx_temp = &mut empty_buffer; /*Reuse available buffer*/
                let prev_dmx_exp_temp = &mut empty_buffer_exp; /*Reuse available buffer*/
                prev_dmx_temp.copy_from_slice(&prev_dmx);
                prev_dmx_exp_temp.copy_from_slice(&prev_dmx_exp);

                /* Stereo Filling parameter calculation */
                {
                    let d2 = &mut *ch_info.b().p_dyn_data;
                    mct_stereo_filling(
                        spec_pair.b(),
                        &mut d2.a_sfb_scale,
                        &ics_info2,
                        &d2.a_scale_factor,
                        sampling_rate_info,
                        ics_info2.max_sf_bands as i32,
                        &mut prev_dmx,
                        &mut prev_dmx_exp,
                        &d2.band_is_noise,
                    );
                }

                if mct_el_flags[tag2] & AC_EL_ENHANCED_NOISE != 0 {
                    /* Run only if INF is active */
                    if mct_el_flags[tag2] & AC_EL_IGF_USE_ENF != 0 {
                        /* Apply MCT Stereo Filling to tiles */
                        let num_tiles = iis_igf_dec_lib_get_number_of_tiles(
                            &st_info.b().igf_static_data,
                            IGF_GRID_LONG_WINDOW,
                        );

                        for tile_idx in 0..num_tiles as usize {
                            let c2 = ch_info.b();
                            let s2 = st_info.b();

                            /* Get the spectrum of the particular tile
                            (iisIGFDecLibAccessSourceSpectrum(.., tileIdx, 0, 0), never NULL here) */
                            let p2_tile_spectrum = &mut c2.igf_data.spectrum_tab_array
                                [tile_idx * IGF_TILE_LEN..(tile_idx + 1) * IGF_TILE_LEN];

                            /* Get pointer to the exponent of the particular tile */
                            let p2_tile_spectrum_exp =
                                &mut s2.igf_static_data.f_spectrum_tab_sfb_exp[tile_idx];

                            /* Use the stored prevDmx for Stereo Filing */
                            prev_dmx.copy_from_slice(&prev_dmx_temp[..]);
                            prev_dmx_exp.copy_from_slice(&prev_dmx_exp_temp[..128]);

                            /* Apply MCT Steffi over the particular tile */
                            mct_stereo_filling(
                                p2_tile_spectrum,
                                p2_tile_spectrum_exp,
                                &ics_info2,
                                &c2.p_dyn_data.a_scale_factor,
                                sampling_rate_info,
                                ics_info2.max_sf_bands as i32,
                                &mut prev_dmx,
                                &mut prev_dmx_exp,
                                &c2.p_dyn_data.band_is_noise,
                            );
                        }
                    } /* if(MCT_elFlags[chTag[ch2]] & AC_EL_IGF_USE_ENF) */

                    cmct_stereo_filling_sync_rngs(
                        &mut st_info,
                        &mut ch_info,
                        &mut spec_pair,
                        &ics_info1,
                        &ics_info2,
                        sampling_rate_info,
                    );
                } /* if(MCT_elFlags[chTag[ch2]] & AC_EL_ENHANCED_NOISE) */
                ch_info.b().p_dyn_data.band_is_noise.fill(0);
            } /* if(bIsShortBlock == 0) */
        } /* if(work->hasStereoFilling[pair]) */

        let alpha_q_sfb = &pair_coeff_q_sfb;

        let max_sf_bands =
            f_max(ch_info.a().ics_info.max_sf_bands, ch_info.b().ics_info.max_sf_bands);
        ch_info.b().ics_info.max_sf_bands = max_sf_bands;
        ch_info.a().ics_info.max_sf_bands = max_sf_bands;

        let ics_info1 = ch_info.a().ics_info;
        let ics_info2 = ch_info.b().ics_info;
        let mct_mask = self_.mct_work.mct_mask[pair];
        let self_ref: &CMct = self_;

        /* inverse MCT rotation */
        let mut win = 0usize;
        for group in 0..ics_info1.window_groups as usize {
            for _groupwin in 0..ics_info1.window_group_length[group] {
                let n_samples = if windows_per_frame > 1 {
                    ch_info.a().granule_length
                } else {
                    aac_samples_per_frame
                };

                let spec_off = win * n_samples as usize;
                let total_sfb = ics_info1.total_sf_bands as i32;
                let band_offsets = get_scale_factor_band_offsets(&ics_info1, sampling_rate_info);

                apply_mct_rotation_wrapper(
                    self_ref,
                    &mut spec_pair.map(|s| &mut s[spec_off..]),
                    &mut ch_info.map(|c| &mut c.p_dyn_data.a_sfb_scale[win * 16..]),
                    &alpha_q_sfb[mct_band_offset as usize..],
                    mct_mask << mct_band_offset,
                    mct_bands_per_window,
                    total_sfb,
                    pair,
                    band_offsets,
                );

                if (mct_el_flags[tag1] & AC_EL_ENHANCED_NOISE != 0)
                    && (mct_el_flags[tag2] & AC_EL_ENHANCED_NOISE != 0)
                {
                    /* Run only if INF is active */
                    if (mct_el_flags[tag1] & AC_EL_IGF_USE_ENF != 0)
                        && (mct_el_flags[tag2] & AC_EL_IGF_USE_ENF != 0)
                    {
                        /* Apply MCT Stereo Filling to tiles */
                        let num_tiles = iis_igf_dec_lib_get_number_of_tiles(
                            &st_info.b().igf_static_data,
                            IGF_GRID_LONG_WINDOW,
                        );

                        let ws1 = if ics_info1.window_sequence == 2 {
                            IGF_GRID_SHORT_WINDOW
                        } else {
                            IGF_GRID_LONG_WINDOW
                        };
                        let ws2 = if ics_info2.window_sequence == 2 {
                            IGF_GRID_SHORT_WINDOW
                        } else {
                            IGF_GRID_LONG_WINDOW
                        };

                        for tile_idx in 0..num_tiles {
                            /* iisIGFDecLibAccessSourceSpectrum() returns NULL for tiles that do not
                            exist in the grid of the window sequence */
                            if !source_spectrum_exists(&st_info.a().igf_static_data, tile_idx, ws1)
                                || !source_spectrum_exists(
                                    &st_info.b().igf_static_data,
                                    tile_idx,
                                    ws2,
                                )
                            {
                                break; /* exit while loop, continue with next win */
                            }

                            let t = tile_idx as usize;
                            let tile_start = t * IGF_TILE_LEN + win * 128;
                            let tile_end = (t + 1) * IGF_TILE_LEN;

                            /* apply mct */
                            apply_mct_rotation_wrapper(
                                self_ref,
                                &mut ch_info.map(|c| {
                                    &mut c.igf_data.spectrum_tab_array[tile_start..tile_end]
                                }),
                                &mut st_info.map(|s| {
                                    &mut s.igf_static_data.f_spectrum_tab_sfb_exp[t][win * 16..]
                                }),
                                &alpha_q_sfb[mct_band_offset as usize..],
                                mct_mask << mct_band_offset,
                                mct_bands_per_window,
                                total_sfb,
                                pair,
                                band_offsets,
                            );
                        } /* for(int tileIdx=0;tileIdx<NumTiles;tileIdx++) */
                    } /* if(MCT_elFlags[chTag[ch2]] & AC_EL_IGF_USE_ENF) */
                } /* if(MCT_elFlags[chTag[ch2]] & AC_EL_ENHANCED_NOISE) */

                mct_band_offset += mct_bands_per_window;
                win += 1;
            } /* for win=... */
        } /* for group=... */
    }

    0
}

/// Whether `iisIGFDecLibAccessSourceSpectrum(hPrivateStaticData, hPrivateData, tileIdx,
/// window_sequence, win)` returns a non-`NULL` pointer. The spectrum of tile `t` from window
/// `win` on is `IgfPrivateData::spectrum_tab_array[t * 1024 + win * 128..(t + 1) * 1024]`.
#[inline]
fn source_spectrum_exists(
    h_private_static_data: &IgfPrivateStaticData,
    tile_idx: i32,
    window_sequence: i32,
) -> bool {
    tile_idx < iis_igf_dec_lib_get_number_of_tiles(h_private_static_data, window_sequence)
}

/// `CMct_StereoFilling_save_prev(self, pAacDecoderChannelInfo)`: saves the MCT output spectra
/// (`spec`, see [`cmct_mct_stereo_filling`]) and their exponents for the stereo filling of the next
/// frame.
pub fn cmct_stereo_filling_save_prev(
    self_: &mut CMct,
    p_aac_decoder_channel_info: &[Box<CAacDecoderChannelInfo>],
    spec: &[FixpDbl],
) {
    for i in 0..self_.num_mct_channels as usize {
        let tag = self_.channel_map[i] as usize;
        let ch_info = &p_aac_decoder_channel_info[tag];
        self_.prev_out_spec[i * 1024..(i + 1) * 1024]
            .copy_from_slice(&spec[tag * 1024..(tag + 1) * 1024]);
        self_.prev_out_spec_exp[i * (8 * 16)..(i + 1) * (8 * 16)]
            .copy_from_slice(&ch_info.p_dyn_data.a_sfb_scale);
    }
}

/// `CMct_StereoFilling_clear_prev(self, pAacDecoderChannelInfo)`: clears the saved spectra. The
/// unused parameter `pAacDecoderChannelInfo` is omitted.
pub fn cmct_stereo_filling_clear_prev(self_: &mut CMct) {
    self_.prev_out_spec.fill(0);
    self_.prev_out_spec_exp.fill(0);
}
