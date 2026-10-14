// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libIGFdec/include/FDK_igfDec.h, libIGFdec/src/FDK_igfDec.cpp and
// libIGFdec/src/arm/FDK_igfDec_arm.cpp (the ARM file only contains NEON code, which is inactive in
// the reference configuration: none of the `FUNCTION_*_func*` switches is defined).

//! Intelligent Gap Filling (IGF) decoder.
//!
//! # Data structures
//!
//! The C++ structures are kept, with these differences:
//!
//! - `IGF_MAP_INFO::fSpectrumTab[0]` always points to
//!   `hPrivateData->Spectrum_tab_array[tile * 1024]` of the `IGF_PRIVATE_DATA` that was passed
//!   together with the `IGF_PRIVATE_STATIC_DATA` to `iisIGFDecLibInit()`. The pointer is not
//!   stored: the source spectrum of tile `t` is `IgfPrivateData::spectrum_tab_array[t * 1024..]`
//!   ([`IGF_TILE_LEN`] values, for short blocks 8 windows of 128 lines).
//! - `IGF_PRIVATE_STATIC_DATA::sfbOffsetLB` / `sfbOffsetSB` point to ROM tables in C++; here the
//!   first `len + 1` values are copied into the structure by [`iis_igf_dec_lib_init`].
//! - `IGF_PRIVATE_DATA::IGF_Common_channel_data_handle` points to scratch memory shared between
//!   the channel elements (one instance per channel of a channel pair,
//!   `AAC_DECODER_INSTANCE::igf_private_data_common[2]`). It is not stored; the functions that use it take
//!   the [`IgfPrivateDataCommon`] as an argument.
//! - `IGF_PRIVATE_DATA_COMMON::virtualSpec` points into the time signal buffer of the decoder
//!   (`pTimeData2 + 1024 * ch`, 1024 values). It is not part of [`IgfPrivateDataCommon`]; the
//!   functions that use it take it as the `virtual_spec` argument. Note that the C++ code may read
//!   stale content from it (e.g. `iisIGFDecoderFillMono()` when no tile exists), so the core
//!   decoder must pass the same memory as the C++ code does.
//! - Enumerations used as integers (`WHITENING_LEVEL`, `DEC_IGF_GRID`, `IGF_FRAME_DIVISION`) are
//!   `i32` type aliases with constants.
//!
//! # Call pattern of the core decoder (libMpeghDec)
//!
//! Per channel, `CAacDecoderStaticChannelInfo` embeds an [`IgfPrivateStaticData`]
//! (`IGF_StaticData`) and `CAacDecoderChannelInfo` embeds an [`IgfPrivateData`] (`IGFdata`). The
//! decoder owns `IgfPrivateDataCommon igf_private_data_common[2]`; channel `ch2` (0 or 1) of an
//! element uses `igf_private_data_common[ch2]` and the virtual spectrum `pTimeData2[1024 * ch2..]`.
//!
//! 1. Configuration (`aacdecoder.cpp`): [`iis_igf_dec_lib_init`] for every channel of an element
//!    with `enhancedNoiseFilling`.
//! 2. Bitstream parsing (`channel.cpp`, `CChannelElement_Read` → IGF data of `usac_core_coder`):
//!    [`iis_igf_dec_lib_read_scf`] followed by [`iis_igf_dec_lib_read_igf`], both with
//!    [`IGF_FRAME_DIVISION_AAC_OR_TCX_LONG`], `window_sequence`/`igfWinMode` = 1 for short blocks.
//! 3. Spectral decoding (`channel.cpp`, `CChannelElement_Decode`): for channel pairs without
//!    independent tiling [`iis_igf_sync_data`]; with `igfUseEnf` (INF)
//!    [`iis_igf_dec_lib_inject_source_spectrum_new`], then the noise filling / stereo filling /
//!    FDP code of the core accesses the tiles with [`iis_igf_dec_lib_get_number_of_tiles`],
//!    [`iis_igf_dec_lib_access_source_spectrum`],
//!    [`iis_igf_dec_lib_access_source_spectrum_exponent`] and
//!    [`iis_igf_dec_lib_access_source_spectrum_scale`] (or directly through `spectrum_tab_array`,
//!    e.g. `chunks_mut(IGF_TILE_LEN)` when several tiles are written at once).
//! 4. IGF application: [`cigf_apply`] (`ApplyTools`, single channel or independent tiling) or
//!    [`cigf_apply_stereo`] (`CChannelElement_Decode`, common window), each followed for long
//!    blocks by [`cigf_tnf_apply`] with the TNF mask filled by the apply function.
//!    `cigf_tnf_apply` is called with the static data of the left channel for both channels in the
//!    stereo case.

use mpeghdec_fdk::digest::StateDigest;

use crate::igf_scf_decoder::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{
    f2_pow, f_div_norm, f_is_less_than, inv_fixp_e, sqrt_fixp_lookup_e,
};
use mpeghdec_fdk::lpc::clpc_auto_to_lpc_igf;
use mpeghdec_fdk::scale::{get_scalefactor, scale_value};

pub const IGF_CLEAN_THRESHOLD: i32 = 16;

pub const IGF_MAX_WIN: usize = 8;
pub const IGF_MAX_TILES: usize = 4;
pub const IGF_MAX_GRIDS: usize = 2;
pub const N_IGF_FRAME_DIVISION: usize = 2;
pub const IGF_MAX_SFB_BANDS: usize = 51;
pub const IGF_MAX_SFB_SHORT: usize = 16;
pub const IGF_MAX_TILE_WIDTH: i32 = 1024;

/// Length of the source spectrum of one tile in `IgfPrivateData::spectrum_tab_array`.
pub const IGF_TILE_LEN: usize = 1024;

/// Capacity of the copied scale factor band offset tables (`len + 1 <= 64` values).
pub const IGF_SFB_OFFSET_TAB_LEN: usize = 64;

const MAX_SPECTRUM_LEN: usize = 1024;

/// `IGF_FRAME_DIVISION`
pub type IgfFrameDivision = i32;
/// AAC short blocks are interleaved and AAC and TCX long can share memory
pub const IGF_FRAME_DIVISION_AAC_OR_TCX_LONG: IgfFrameDivision = 0;
pub const IGF_FRAME_DIVISION_TCX_SHORT_1: IgfFrameDivision = 1;
pub const IGF_FRAME_DIVISION_TCX_SHORT_2: IgfFrameDivision = 2;

/// `WHITENING_LEVEL`
pub type WhiteningLevel = i32;
pub const WHITENING_MID: WhiteningLevel = 0;
pub const WHITENING_OFF: WhiteningLevel = 1;
pub const WHITENING_STRONG: WhiteningLevel = 2;
pub const WHITENING_UNUSED: WhiteningLevel = 3;

/// `DEC_IGF_GRID`
pub type DecIgfGrid = i32;
pub const IGF_GRID_LONG_WINDOW: DecIgfGrid = 0;
pub const IGF_GRID_SHORT_WINDOW: DecIgfGrid = 1;
pub const IGF_GRID_TCX10_WINDOW: DecIgfGrid = 2;
pub const IGF_GRID_TCX20_WINDOW: DecIgfGrid = 3;
pub const IGF_GRID_UNDEF: DecIgfGrid = 4;

/// `IGF_MAP_INFO` (without `fSpectrumTab`, see the module documentation)
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct IgfMapInfo {
    /// `iWhiteningLevel`: whitening level of the tile
    pub i_whitening_level: WhiteningLevel,
    /// `iSfbWidthTab`: holds information for the sfb granulation in a tile
    pub i_sfb_width_tab: [i16; IGF_MAX_SFB_BANDS],
    /// `iSfbCnt`: count of sfbs in a tile
    pub i_sfb_cnt: i16,
    /// `iDesStart`: defines the first line in destination spectrum of the tile
    pub i_des_start: i16,
    /// `iWidth`: width (or length) of the tile
    pub i_width: i16,
    /// `iIdx`: tile index, transmitted in the bitstream
    pub i_idx: u8,
    /// `iSfbSplit`: When a paired SFB must be split between two tiles
    pub i_sfb_split: u8,
}

impl Default for IgfMapInfo {
    fn default() -> Self {
        IgfMapInfo {
            i_whitening_level: 0,
            i_sfb_width_tab: [0; IGF_MAX_SFB_BANDS],
            i_sfb_cnt: 0,
            i_des_start: 0,
            i_width: 0,
            i_idx: 0,
            i_sfb_split: 0,
        }
    }
}

/// `IGF_WORK_MEMORY`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct IgfWorkMemory {
    /// `fSfbGainTab`: gain to apply on IGF data
    pub f_sfb_gain_tab: [FixpDbl; IGF_MAX_SFB_BANDS],
    /// `fSfbDestinEnergyTab`: energy levels transmitted and decoded
    pub f_sfb_destin_energy_tab: [FixpDbl; IGF_MAX_SFB_BANDS],
    /// `fSfbTileEnergyTab`: energy levels calculated from source region
    pub f_sfb_tile_energy_tab: [FixpDbl; IGF_MAX_WIN * IGF_MAX_SFB_SHORT],
    /// `fSfbSurvivedEnergyTab`: energy levels calculated from source region
    pub f_sfb_survived_energy_tab: [FixpDbl; IGF_MAX_WIN * IGF_MAX_SFB_SHORT],
    /// `sE`
    pub s_e: [FixpDbl; IGF_MAX_SFB_BANDS],
    /// `tE`
    pub t_e: [FixpDbl; IGF_MAX_SFB_BANDS],

    /// `fSfbGainTab_exp`
    pub f_sfb_gain_tab_exp: [i8; IGF_MAX_SFB_BANDS],
    /// `fSfbDestinEnergyTab_exp`
    pub f_sfb_destin_energy_tab_exp: [i8; IGF_MAX_SFB_BANDS],
    /// `fSfbTileEnergyTab_exp`
    pub f_sfb_tile_energy_tab_exp: [i8; IGF_MAX_WIN * IGF_MAX_SFB_SHORT],
    /// `fSfbSurvivedEnergyTab_exp`
    pub f_sfb_survived_energy_tab_exp: [i8; IGF_MAX_WIN * IGF_MAX_SFB_SHORT],
    /// `sE_exp`
    pub s_e_exp: [i8; IGF_MAX_SFB_BANDS],
    /// `tE_exp`
    pub t_e_exp: [i8; IGF_MAX_SFB_BANDS],

    /// `intermediate_shift`
    pub intermediate_shift: [i8; IGF_MAX_WIN * IGF_MAX_SFB_SHORT],
    /// `Initial_exp`
    pub initial_exp: [i8; IGF_MAX_WIN * IGF_MAX_SFB_SHORT],
}

impl Default for IgfWorkMemory {
    fn default() -> Self {
        const NW: usize = IGF_MAX_WIN * IGF_MAX_SFB_SHORT;
        IgfWorkMemory {
            f_sfb_gain_tab: [0; IGF_MAX_SFB_BANDS],
            f_sfb_destin_energy_tab: [0; IGF_MAX_SFB_BANDS],
            f_sfb_tile_energy_tab: [0; NW],
            f_sfb_survived_energy_tab: [0; NW],
            s_e: [0; IGF_MAX_SFB_BANDS],
            t_e: [0; IGF_MAX_SFB_BANDS],
            f_sfb_gain_tab_exp: [0; IGF_MAX_SFB_BANDS],
            f_sfb_destin_energy_tab_exp: [0; IGF_MAX_SFB_BANDS],
            f_sfb_tile_energy_tab_exp: [0; NW],
            f_sfb_survived_energy_tab_exp: [0; NW],
            s_e_exp: [0; IGF_MAX_SFB_BANDS],
            t_e_exp: [0; IGF_MAX_SFB_BANDS],
            intermediate_shift: [0; NW],
            initial_exp: [0; NW],
        }
    }
}

/// `IGF_GRID_INFO`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[digest(extra = digest_tile_idx_tab)]
pub struct IgfGridInfo {
    /// `sIGFMapInfoTab`
    pub s_igf_map_info_tab: [IgfMapInfo; IGF_MAX_TILES],
    /// `iIGFStartLine`: start line in IGF range
    pub i_igf_start_line: i16,
    /// `iIGFStopLine`: stop line in IGF range
    pub i_igf_stop_line: i16,
    /// `iIGFMinLine`: minimal line where IGF start to copy from
    pub i_igf_min_line: i16,
    /// `iIGFTileIdxTab`: tile index table used with initialization of sIGFMapInfoTab
    ///
    /// State digest: only the first `iIGFNumTile` entries, the ones that are read
    /// (`iisIGFDecLibCheckTileTransition()`) and written (`iisIGFDecLibInitGridInfo()`).
    #[digest(skip)]
    pub i_igf_tile_idx_tab: [u8; IGF_MAX_TILES],
    /// `iIGFNumSfb`: IGF number of SCF bands in total
    pub i_igf_num_sfb: u8,
    /// `iIGFStartSfb`: start SBF in IGF range
    pub i_igf_start_sfb: u8,
    /// `iIGFStopSfb`: stop SFB in IGF range
    pub i_igf_stop_sfb: u8,
    /// `iIGFNumTile`: number of tiles used
    pub i_igf_num_tile: u8,
}

fn digest_tile_idx_tab(g: &IgfGridInfo, d: &mut mpeghdec_fdk::digest::Digester) {
    let n = (g.i_igf_num_tile as usize).min(IGF_MAX_TILES);
    d.field("i_igf_tile_idx_tab", &g.i_igf_tile_idx_tab[..n]);
}

fn digest_grids(s: &IgfPrivateStaticData, d: &mut mpeghdec_fdk::digest::Digester) {
    d.field_with("s_grid_info_tab", |d| {
        for (g, grid) in s.s_grid_info_tab.iter().enumerate() {
            d.element_with(g, |d| {
                let mut c = *grid;
                let canonical = match g as i32 {
                    IGF_GRID_LONG_WINDOW => {
                        iis_igf_dec_lib_init_grid_info(
                            &mut c,
                            IGF_GRID_LONG_WINDOW,
                            &s.sfb_offset_lb,
                            s.igf_start_sfb_lb as i32,
                            s.igf_stop_sfb_lb as i32,
                            s.igf_min_subband_lb,
                            s.use_high_res as i32,
                            s.igf_n_tiles_lb as i32,
                            &s.igf_tiles_lb,
                            &grid.i_igf_tile_idx_tab,
                        );
                        c == *grid
                    }
                    IGF_GRID_SHORT_WINDOW => {
                        iis_igf_dec_lib_init_grid_info(
                            &mut c,
                            IGF_GRID_SHORT_WINDOW,
                            &s.sfb_offset_sb,
                            s.igf_start_sfb_sb as i32,
                            s.igf_stop_sfb_sb as i32,
                            s.igf_min_subband_sb,
                            s.use_high_res as i32,
                            s.igf_n_tiles_sb as i32,
                            &s.igf_tiles_sb,
                            &grid.i_igf_tile_idx_tab,
                        );
                        c == *grid
                    }
                    _ => false,
                };
                d.bytes(&[canonical as u8]);
                if !canonical {
                    grid.digest(d);
                }
            });
        }
    });
}

/// `IGF_PRIVATE_STATIC_DATA`
///
/// State digest: the grids `sGridInfoTab` are a cache of the tile mapping for the tile indices
/// `iIGFTileIdxTab` of the last frame with the respective window sequence; before a grid is used,
/// `iisIGFDecLibCheckTileTransition()` initializes it again for the tile indices of the frame if
/// they differ. A grid that equals the result of `iisIGFDecLibInitGridInfo()` for its own tile
/// indices and the current configuration therefore behaves like any other such grid and is not
/// part of the digest (only this property is); other grids are fed completely.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
#[digest(extra = digest_grids)]
pub struct IgfPrivateStaticData {
    /// `hIGFSCFDecoder`
    pub h_igf_scf_decoder: IgfScfDecPrivateData,

    /// `sGridInfoTab` (state digest: see the structure)
    #[digest(skip)]
    pub s_grid_info_tab: [IgfGridInfo; IGF_MAX_GRIDS],

    /// `fSpectrumScale`: window scale values
    pub f_spectrum_scale: [[i16; IGF_MAX_WIN]; IGF_MAX_TILES],
    /// `fSpectrumTab_sfb_exp`: sfb scale values
    pub f_spectrum_tab_sfb_exp: [[i16; IGF_MAX_WIN * IGF_MAX_SFB_SHORT]; IGF_MAX_TILES],

    /// `igfWhiteningLevelPrev`
    pub igf_whitening_level_prev: [WhiteningLevel; IGF_MAX_TILES],

    /// `igfMinSubbandLB`
    pub igf_min_subband_lb: i32,
    /// `igfMinSubbandSB`
    pub igf_min_subband_sb: i32,
    /// `igfTilesLB`
    pub igf_tiles_lb: [i32; IGF_MAX_TILES],
    /// `igfTilesSB`
    pub igf_tiles_sb: [i32; IGF_MAX_TILES],
    /// `prevPatchNum`: indices of the best matching patch
    pub prev_patch_num: [u8; IGF_MAX_TILES],
    /// `numSfbLB`
    pub num_sfb_lb: u8,
    /// `sfbOffsetLB` (copy of the first `numSfbLB + 1` values)
    pub sfb_offset_lb: [i16; IGF_SFB_OFFSET_TAB_LEN],
    /// `numSfbSB`
    pub num_sfb_sb: u8,
    /// `sfbOffsetSB` (copy of the first `numSfbSB + 1` values)
    pub sfb_offset_sb: [i16; IGF_SFB_OFFSET_TAB_LEN],
    /// `enhancedNoiseFilling`
    pub enhanced_noise_filling: u8,
    /// `igfUseWhitening`
    pub igf_use_whitening: u8,
    /// `igfAfterTnsSynth`
    pub igf_after_tns_synth: u8,
    /// `igfIndependentTiling`
    pub igf_independent_tiling: u8,
    /// `useHighRes`
    pub use_high_res: u8,
    /// `igfStartSfbLB`
    pub igf_start_sfb_lb: u8,
    /// `igfStopSfbLB`
    pub igf_stop_sfb_lb: u8,
    /// `igfStartSfbSB`
    pub igf_start_sfb_sb: u8,
    /// `igfStopSfbSB`
    pub igf_stop_sfb_sb: u8,
    /// `igfNTilesLB`
    pub igf_n_tiles_lb: u8,
    /// `igfNTilesSB`
    pub igf_n_tiles_sb: u8,
}

impl Default for IgfPrivateStaticData {
    fn default() -> Self {
        IgfPrivateStaticData {
            h_igf_scf_decoder: IgfScfDecPrivateData::default(),
            s_grid_info_tab: [IgfGridInfo::default(); IGF_MAX_GRIDS],
            f_spectrum_scale: [[0; IGF_MAX_WIN]; IGF_MAX_TILES],
            f_spectrum_tab_sfb_exp: [[0; IGF_MAX_WIN * IGF_MAX_SFB_SHORT]; IGF_MAX_TILES],
            igf_whitening_level_prev: [0; IGF_MAX_TILES],
            igf_min_subband_lb: 0,
            igf_min_subband_sb: 0,
            igf_tiles_lb: [0; IGF_MAX_TILES],
            igf_tiles_sb: [0; IGF_MAX_TILES],
            prev_patch_num: [0; IGF_MAX_TILES],
            num_sfb_lb: 0,
            sfb_offset_lb: [0; IGF_SFB_OFFSET_TAB_LEN],
            num_sfb_sb: 0,
            sfb_offset_sb: [0; IGF_SFB_OFFSET_TAB_LEN],
            enhanced_noise_filling: 0,
            igf_use_whitening: 0,
            igf_after_tns_synth: 0,
            igf_independent_tiling: 0,
            use_high_res: 0,
            igf_start_sfb_lb: 0,
            igf_stop_sfb_lb: 0,
            igf_start_sfb_sb: 0,
            igf_stop_sfb_sb: 0,
            igf_n_tiles_lb: 0,
            igf_n_tiles_sb: 0,
        }
    }
}

/// `IGF_BITSTREAM_DATA`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct IgfBitstreamData {
    /// `sfe`: array which will contain the decoded quantized coefficients
    pub sfe: [i8; 256],
    /// `iCurrTileIdxTab`
    pub i_curr_tile_idx_tab: [u8; IGF_MAX_TILES],
    /// `igfWhiteningLevel`
    pub igf_whitening_level: [WhiteningLevel; IGF_MAX_TILES],
    /// `igf_allZero`
    pub igf_all_zero: u8,
    /// `igfUseEnfFlat`
    pub igf_use_enf_flat: u8,
}

impl Default for IgfBitstreamData {
    fn default() -> Self {
        IgfBitstreamData {
            sfe: [0; 256],
            i_curr_tile_idx_tab: [0; IGF_MAX_TILES],
            igf_whitening_level: [0; IGF_MAX_TILES],
            igf_all_zero: 0,
            igf_use_enf_flat: 0,
        }
    }
}

/// `IGF_PRIVATE_DATA_COMMON` (without `virtualSpec`, see the module documentation)
#[derive(StateDigest, Clone, Debug, Default, PartialEq, Eq)]
pub struct IgfPrivateDataCommon {
    /// `IGF_WorkingMem`
    pub igf_working_mem: [IgfWorkMemory; IGF_MAX_TILES],
}

/// `IGF_PRIVATE_DATA` (without `IGF_Common_channel_data_handle`, see the module documentation)
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct IgfPrivateData {
    /// `Spectrum_tab_array`: source spectra of the tiles, [`IGF_TILE_LEN`] values per tile
    pub spectrum_tab_array: [FixpDbl; IGF_MAX_TILES * IGF_TILE_LEN],
    /// `bitstreamData`
    pub bitstream_data: [IgfBitstreamData; N_IGF_FRAME_DIVISION],
    /// `tileNum`: indices of the best matching patch
    pub tile_num: [u8; IGF_MAX_TILES],
}

impl Default for IgfPrivateData {
    fn default() -> Self {
        IgfPrivateData {
            spectrum_tab_array: [0; IGF_MAX_TILES * IGF_TILE_LEN],
            bitstream_data: [IgfBitstreamData::default(); N_IGF_FRAME_DIVISION],
            tile_num: [0; IGF_MAX_TILES],
        }
    }
}

/// `randomSign(seed)`
fn random_sign(seed: &mut u32) -> i32 {
    *seed = ((*seed as u64) * 69069 + 5) as u32;
    if (*seed & 0x10000) > 0 {
        -1
    } else {
        1
    }
}

/// `get_IGF_tile()`
fn get_igf_tile(
    igf_min: i32,
    sfb_bgn: i32,
    sfb_end: i32,
    b_use_high_res: i32,
    swb_offset: &[i16],
    swb_shift: i32,
    tile: &mut [i32; IGF_MAX_TILES],
) -> i32 {
    let off = |j: i32| (swb_offset[j as usize] as i32) << swb_shift;
    let igf_bgn = off(sfb_bgn) as i16 as i32;
    let igf_end = f_min(1024u32, off(sfb_end) as u32) as i16 as i32;
    let mut mem = sfb_bgn;
    let igf_range = f_max(8, igf_end - igf_bgn);
    let mut sbs = igf_bgn;
    let mut n_tr = 0;

    for t in tile.iter_mut() {
        *t = igf_range >> 2;
    }

    if igf_bgn > igf_min && igf_bgn < igf_end {
        for i in 0..4 {
            let mut j = 0;
            loop {
                j += 1;
                if f_min(sbs + tile[i], igf_end) <= off(j) {
                    break;
                }
            }

            if b_use_high_res == 0 {
                if ((j - sfb_bgn) & 1) == 1 && (j - mem) > 1 && i < 3 {
                    j -= 1;
                } else if i == 3 {
                    j = sfb_end;
                }
                mem = j;
            }

            tile[i] = f_max(2, f_min(off(j) - sbs, igf_end - sbs));

            debug_assert!(tile[i] <= IGF_MAX_TILE_WIDTH);

            sbs += tile[i];
            n_tr += 1;
            if sbs == igf_end {
                break;
            }
        }
    }
    for t in tile.iter_mut().skip(n_tr as usize) {
        *t = 0;
    }
    n_tr
}

/// `iisIGFDecLibInitGridInfo()`
fn iis_igf_dec_lib_init_grid_info(
    h_grid_info: &mut IgfGridInfo,
    window_sequence: i32,
    sfb_offset: &[i16],
    sfb_start: i32,
    sfb_stop: i32,
    igf_min: i32,
    use_high_res: i32,
    n_t: i32,
    tile: &[i32; IGF_MAX_TILES],
    tile_idx: &[u8; IGF_MAX_TILES],
) {
    let swb_shift = 0;
    let off = |j: usize| (sfb_offset[j] as i32) << swb_shift;

    let igf_bgn = off(sfb_start as usize) as i16;

    h_grid_info.i_igf_num_sfb = (sfb_stop - sfb_start) as u8;
    h_grid_info.i_igf_start_sfb = sfb_start as u8;
    h_grid_info.i_igf_stop_sfb = sfb_stop as u8;
    h_grid_info.i_igf_start_line = igf_bgn;
    h_grid_info.i_igf_stop_line = f_min(1024u32, off(sfb_stop as usize) as u32) as i16;
    h_grid_info.i_igf_min_line = igf_min as i16;

    let n_t = n_t as usize;

    /* Compute tile begins */
    let mut tile_bgn: [i32; 5] = [igf_bgn as i32, 0, 0, 0, 0];

    /* Calculate offsets from tile begin to mapped field from base spectrum */
    for j in 0..n_t {
        tile_bgn[j + 1] = tile_bgn[j] + tile[j];

        /* Initialize tile mapping index tables */
        let h_map_info = &mut h_grid_info.s_igf_map_info_tab[j];
        h_map_info.i_des_start = tile_bgn[j] as i16;
        h_map_info.i_width = tile[j] as i16;
        h_map_info.i_idx = j as u8;
    }

    /* Aggregate tile and sfb info */
    let mut p = sfb_start as usize; /* p2_OffsetTable */
    let mut sfb_step = 1;
    if use_high_res == 0 && window_sequence == IGF_GRID_LONG_WINDOW {
        sfb_step = 2;
    }
    if sfb_step == 1 {
        /* No pairing case */
        for i in 0..n_t {
            let mut sfb_count = 0usize;
            let tile_upper_edge = tile_bgn[i + 1];
            let h_map_info = &mut h_grid_info.s_igf_map_info_tab[i];
            loop {
                let sfb_width = (off(p + 1) - off(p)) as i16;
                h_map_info.i_sfb_width_tab[sfb_count] = sfb_width;
                sfb_count += 1;
                p += 1;
                if tile_upper_edge <= off(p) {
                    break;
                }
            }

            h_map_info.i_sfb_cnt = sfb_count as i16;
            h_map_info.i_sfb_split = 0;
        }
    } else {
        /* Pairing case, i.e. sfbStep = 2. Must be checked if some tile boundary occurs in the
        middle of a sfb-pair. If yes, then sfb-pair splitting is signalled */
        let mut sfb_jump = sfb_step;
        let mut split_flag = 0;
        let stop_line = h_grid_info.i_igf_stop_line as i32;
        for i in 0..n_t {
            let mut sfb_count = 0usize;
            let mut sfb_width: i16;
            let tile_upper_edge = tile_bgn[i + 1];
            let h_map_info = &mut h_grid_info.s_igf_map_info_tab[i];
            loop {
                sfb_width = (off(p + sfb_jump) - off(p)) as i16;
                h_map_info.i_sfb_width_tab[sfb_count] = sfb_width;
                sfb_count += 1;
                p += sfb_jump;
                /*Introduced to prevent an illegal memory access when
                the end of the table is reached (p2_OffsetTable[1])*/
                if sfb_offset[p] as i32 >= stop_line {
                    break;
                }
                if sfb_offset[p + 1] as i32 == tile_bgn[n_t] {
                    sfb_jump = 1;
                } else {
                    sfb_jump = sfb_step;
                }
                if tile_upper_edge <= off(p) {
                    break;
                }
            }

            h_map_info.i_sfb_cnt = sfb_count as i16;
            if split_flag == 0 {
                h_map_info.i_sfb_split = 0;
            } else {
                h_map_info.i_sfb_split = 1;
                split_flag = 0;
            }
            /* If tile boundary is in the middle of the pair signal sfb-splitting */
            if off(p) > tile_upper_edge {
                sfb_count -= 1;
                h_map_info.i_sfb_width_tab[sfb_count] =
                    (sfb_width as i32 - (off(p) - off(p - 1))) as i16;
                split_flag = 1;
                p -= 1;
                sfb_jump = 1;
            }
        }
    }

    h_grid_info.i_igf_num_tile = n_t as u8;
    h_grid_info.i_igf_tile_idx_tab[..n_t].copy_from_slice(&tile_idx[..n_t]);
}

/// `iisIGFDecLibResetIGF()`
fn iis_igf_dec_lib_reset_igf(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    frame_type: i32,
) {
    let bd = &mut h_private_data.bitstream_data[frame_type as usize];
    for i in 0..IGF_MAX_TILES {
        bd.i_curr_tile_idx_tab[i] = 3;
        bd.igf_use_enf_flat = 0;
        h_private_static_data.prev_patch_num[i] = 3;
        bd.igf_whitening_level[i] = WHITENING_MID;
        h_private_static_data.igf_whitening_level_prev[i] = WHITENING_MID;
    }
}

/// Reads one whitening level (`WHITENING_MID`, `WHITENING_OFF` or `WHITENING_STRONG`).
fn read_whitening_level(bs: &mut FdkBitStream) -> WhiteningLevel {
    if bs.read_bit() != 0 {
        if bs.read_bit() != 0 {
            WHITENING_STRONG
        } else {
            WHITENING_OFF
        }
    } else {
        WHITENING_MID
    }
}

/// `iisIGFDecLibReadIGF(hPrivateStaticData, hPrivateData, hBStr, window_sequence,
/// bUsacIndependencyFlag, igfUseEnf, frameType)`: implements the decoding of IGF (tile numbers,
/// whitening levels and the temporal flattening trigger).
pub fn iis_igf_dec_lib_read_igf(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    bs: &mut FdkBitStream,
    window_sequence: i32,
    b_usac_independency_flag: i32,
    igf_use_enf: i32,
    frame_type: i32,
) -> i32 {
    let ret_code = 0;
    let ft = frame_type as usize;

    if h_private_data.bitstream_data[ft].igf_all_zero != 0 {
        iis_igf_dec_lib_reset_igf(h_private_static_data, h_private_data, frame_type);
        return ret_code;
    }

    let n_tiles =
        h_private_static_data.s_grid_info_tab[window_sequence as usize].i_igf_num_tile as usize;

    {
        /* read igfUsePrevTile */
        let igf_use_prev_tile = if b_usac_independency_flag != 0 { 0 } else { bs.read_bit() };
        /* read all igfTileNum */
        if igf_use_prev_tile != 0 {
            for i in 0..n_tiles {
                h_private_data.tile_num[i] = h_private_static_data.prev_patch_num[i];
            }
        } else {
            for i in 0..n_tiles {
                h_private_data.tile_num[i] = bs.read_bits(2) as u8;
            }
        }
    }

    let bd = &mut h_private_data.bitstream_data[ft];
    for i in 0..n_tiles {
        h_private_static_data.prev_patch_num[i] = h_private_data.tile_num[i];
        bd.igf_whitening_level[i] = WHITENING_MID;
    }

    for i in n_tiles..IGF_MAX_TILES {
        h_private_static_data.prev_patch_num[i] = 3;
        bd.igf_whitening_level[i] = WHITENING_MID;
    }

    if h_private_static_data.igf_use_whitening != 0 && window_sequence != IGF_GRID_SHORT_WINDOW {
        let igf_use_prev_whitening_level =
            if b_usac_independency_flag != 0 { 0 } else { bs.read_bit() };

        if igf_use_prev_whitening_level != 0 {
            for i in 0..n_tiles {
                bd.igf_whitening_level[i] = h_private_static_data.igf_whitening_level_prev[i];
            }
        } else {
            bd.igf_whitening_level[0] = read_whitening_level(bs);

            if bs.read_bit() != 0 {
                for i in 1..n_tiles {
                    bd.igf_whitening_level[i] = read_whitening_level(bs);
                }
            } else {
                for i in 1..n_tiles {
                    bd.igf_whitening_level[i] = bd.igf_whitening_level[0];
                }
            }
        }
    } /*  if(hPrivateStaticData->igfUseWhitening ... */

    for i in 0..IGF_MAX_TILES {
        h_private_static_data.igf_whitening_level_prev[i] = bd.igf_whitening_level[i];
    }

    for i in 0..n_tiles {
        /* map the tile indices to the current tile tab: */
        bd.i_curr_tile_idx_tab[i] = h_private_data.tile_num[i];
    }

    /* decode_temp_flattening_trigger */
    if igf_use_enf != 0 && window_sequence != IGF_GRID_SHORT_WINDOW {
        bd.igf_use_enf_flat = bs.read_bit() as u8;
    }

    ret_code
}

/// `iisIGFDecLibResetSCF()`: resets the internal decoder memory (context memory)
fn iis_igf_dec_lib_reset_scf(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    frame_type: i32,
) {
    h_private_data.bitstream_data[frame_type as usize].sfe.fill(0);

    iis_igf_scf_decoder_reset(&mut h_private_static_data.h_igf_scf_decoder);
}

/// `iisIGFDecLibReadSCF(hPrivateStaticData, hPrivateData, hBStr, indepFlag, igfWinMode, winCount,
/// frameType)`: implements the decoding of the IGF scale factors (`igf_allZero` flag and the
/// arithmetic coded energies).
pub fn iis_igf_dec_lib_read_scf(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    bs: &mut FdkBitStream,
    indep_flag: i32,
    igf_win_mode: i32,
    win_count: i32,
    frame_type: i32,
) {
    let sfe_reduce_precision_enabled = if igf_win_mode <= IGF_GRID_SHORT_WINDOW {
        (h_private_static_data.use_high_res == 0) as i32
    } else {
        0
    };

    let all_zero = bs.read_bit();

    h_private_data.bitstream_data[frame_type as usize].igf_all_zero = all_zero as u8;

    let win_flag = iis_igf_scf_decoder_get_last_frame_was_short_block(
        &h_private_static_data.h_igf_scf_decoder,
    ) != igf_win_mode;

    if all_zero != 0 || indep_flag != 0 || win_flag {
        iis_igf_dec_lib_reset_scf(h_private_static_data, h_private_data, frame_type);
    }

    if all_zero == 0 {
        iis_igf_scf_decoder_decode(
            &mut h_private_static_data.h_igf_scf_decoder,
            bs,
            &mut h_private_data.bitstream_data[frame_type as usize].sfe,
            indep_flag,
            igf_win_mode,
            win_count,
            sfe_reduce_precision_enabled,
        );
    }

    /* store actual window sequence */
    iis_igf_scf_decoder_set_last_frame_was_short_block(
        &mut h_private_static_data.h_igf_scf_decoder,
        igf_win_mode,
    );
}

/// `iisIGF_Sync_Data(hPrivateDataL, hPrivateDataR)`: copies the tiling, whitening and TNF
/// information of the channel with IGF data to the channel without (`igf_allZero`).
pub fn iis_igf_sync_data(
    h_private_data_l: &mut IgfPrivateData,
    h_private_data_r: &mut IgfPrivateData,
) {
    for i in 0..N_IGF_FRAME_DIVISION {
        let l = &mut h_private_data_l.bitstream_data[i];
        let r = &mut h_private_data_r.bitstream_data[i];

        if l.igf_all_zero == 1 && r.igf_all_zero == 0 {
            l.i_curr_tile_idx_tab = r.i_curr_tile_idx_tab;
            l.igf_whitening_level = r.igf_whitening_level;
            l.igf_use_enf_flat = r.igf_use_enf_flat;
        } else if l.igf_all_zero == 0 && r.igf_all_zero == 1 {
            r.i_curr_tile_idx_tab = l.i_curr_tile_idx_tab;
            r.igf_whitening_level = l.igf_whitening_level;
            r.igf_use_enf_flat = l.igf_use_enf_flat;
        }
    }
}

/// `getWhiteSpectralData(in, out, working_array, hWorkMem, start, stop)`.
///
/// `inp == None` means `in == out` (in-place operation on `out`).
fn get_white_spectral_data(
    inp: Option<&[FixpDbl]>,
    out: &mut [FixpDbl],
    working_array: &mut [i32],
    h_work_mem: &mut IgfWorkMemory,
    start: i32,
    stop: i32,
) {
    let s = start as usize;
    let e = stop as usize;

    /* Find available headroom in input signal */
    let shl1 = {
        let src: &[FixpDbl] = inp.unwrap_or(out);
        get_scalefactor(&src[s - 3..e]) /* scale factor to left-align input data */
    };

    /* Generate 1e-3 in Q-10.41 */
    let mut env: u32 = 0x83126E98; /* 0x4189374c<<1 */
    const VALUE0_001_EXP: i32 = -10; /* This exponent is even on purpose */

    /* Find the (even) exponent of the averaged env[i] variable */
    let mut shift_energy = 2; /* Minimum shift amount for energy accumulation: 2, max energy:
                              7*0x2000.0000 = 0xE000.0000 (for -1.0f) */
    let mut exp_env = ((h_work_mem.initial_exp[0] as i32 - shl1) << 1) + shift_energy;

    let delta = exp_env - VALUE0_001_EXP; /* delta is still even */
    if delta > 0 {
        /* Samples have higher exponent than 1e-3, when scaled left, we adapt the env initial value
         * accordingly */
        if delta <= 31 {
            env >>= delta;
        } else {
            env = 0;
        }
    } else {
        /* Sample energies have lower (or same) exponent than 1e-3, when shifted by 2, we add a
         * positive (or zero) shift-right delta */
        shift_energy -= delta;
        exp_env -= delta; /* exp_env = -10 (even, same as default env) */
    }

    match exp_env {
        -10 => {
            /* Normalization and shift in range 0 */
            if shift_energy == 2 {
                /* We need a shift-right by 4 */
                /* Normalization and shift in range 1..2 */
                shift_energy = 4;
                env >>= 2;
                exp_env += 2; /* exp_env: -8 */
            }
        }
        /* Normalization and shift in range 1..2 */
        -8 if shift_energy == 2 => {
            /* We need a shift-right by 4 */
            /* Normalization and shift in range 0..4 */
            shift_energy = 4;
            env >>= 2;
            exp_env += 2; /* exp_env: -6 */
        }
        _ => { /* [-6, -4, .. +??]: shift_energy = 2 already set */ }
    }

    /* shr1 (limited to 31) is used to scale right the energy values. With shr1=31, each energy
     * becomes 0x0000.0000 */
    /* Subtract 1 due to use of fPow2Div2 instead of fPow2 because of issues with fract class tests
     */
    let shr1 = f_min(31, shift_energy) - 1; /* scale factor to right-align energies */
    let mut n_min: i32;
    {
        let src: &[FixpDbl] = inp.unwrap_or(out);
        let energy = |v: FixpDbl| (f_pow2_div2(v.wrapping_shl(shl1 as u32)) as u32) >> shr1;

        /* accumulator for 7 sliding energy values plus initial value 0.001f */
        let mut acc = [0u32; 8];
        let mut idx = 0usize; /* index to refer to next data of array acc */

        /* Prolog: calculate energy average using first 7 samples */
        for i in (s - 3)..(s + 4) {
            let val_in = energy(src[i]);
            acc[idx] = val_in;
            idx += 1;
            env = env.wrapping_add(val_in);
        }
        /* acc[0..6] hold last 7 energy values, idx = 7 */

        let mut p1 = 0usize;

        n_min = 1000;
        /* In this loop, we evaluate the factors (shr2) to apply for the output, that is written in
         * a second run */
        let mut i = start;
        while i < stop - 4 {
            /* compute output sample: shr2 in range 0..64 */
            let shr2 = exp_env - f_normz(env as i32);
            if src[i as usize] != 0 {
                n_min = f_min(n_min, shr2);
            }
            working_array[p1] = shr2;
            p1 += 1;

            /* Update env and acc[],idx with next sample */
            let val_in = energy(src[(i + 4) as usize]);
            acc[idx] = val_in;
            idx += 1;
            idx &= 0x7; /* idx runs 0...7, 0...7 */
            env = env.wrapping_sub(acc[idx]).wrapping_add(val_in);
            i += 1;
        }

        /* Epilog: compute last 4 output samples using 4x latest env */
        while i < stop {
            /* compute output sample: shr2 in range 0..64 */
            let shr2 = exp_env - f_normz(env as i32);
            if src[i as usize] != 0 {
                n_min = f_min(n_min, shr2);
            }
            working_array[p1] = shr2;
            p1 += 1;
            i += 1;
        }
    }

    /* Check, if all in[i] are zero, ... */
    if n_min == 1000 {
        /* Obviously all in[i] are ZERO, let's quit after clearing the output */
        out[s..e].fill(0);
        h_work_mem.initial_exp[0] = 0;
    } else {
        /* make n_min even */
        n_min = (n_min >> 1) << 1;

        /* Calclulate the output data exponent */
        h_work_mem.initial_exp[0] =
            h_work_mem.initial_exp[0].wrapping_add((21 - shl1 - (n_min >> 1)) as i8);

        const CONST_INV_SQRT2: FixpDbl = 0x5A82799A; /* 1/sqrt(2)=0.70710678 => 0x5a82799A in Q0.31;*/
        /* compute output sample: shr2 in range 0..62 */
        let white = |v: FixpDbl, w: i32| {
            let shr2 = f_max(0, w - n_min);
            let val_in = v << shl1;
            let odd = f_mult(val_in, CONST_INV_SQRT2);
            let val_in = if shr2 & 1 != 0 { odd } else { val_in };
            val_in.wrapping_shr((shr2 >> 1) as u32)
        };
        let working = &working_array[..e - s];
        match inp {
            Some(src) => {
                for ((o, &v), &w) in out[s..e].iter_mut().zip(&src[s..e]).zip(working) {
                    *o = white(v, w);
                }
            }
            None => {
                for (o, &w) in out[s..e].iter_mut().zip(working) {
                    *o = white(*o, w);
                }
            }
        }
    }
}

/// `iisIGFDecLibInitFractScaleValues()`: initialization of energy values
fn iis_igf_dec_lib_init_fract_scale_values(
    h_private_static_data: &IgfPrivateStaticData,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    window_sequence: i32,
) {
    /* get the grid structure: */
    let h_grid = &h_private_static_data.s_grid_info_tab[window_sequence as usize];

    let max_win = if window_sequence == IGF_GRID_SHORT_WINDOW { 8 } else { 1 };

    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        let p_spec_scale_tile = &h_private_static_data.f_spectrum_scale[tile];
        for win in 0..max_win {
            work_mem[tile].initial_exp[win * IGF_MAX_SFB_SHORT] = p_spec_scale_tile[win] as i8;
        }
    }
}

/// `iisIGFDecoderApplyWhitening()`
fn iis_igf_decoder_apply_whitening(
    spectrum_tab_array: &mut [FixpDbl],
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    h_grid: &IgfGridInfo,
    p_spectral_data: &[FixpDbl],
    working_array: &mut [i32],
    random_seed: &mut u32,
    flag_inf_active: u8,
    mono_or_stereo_flag: u8,
) {
    let mut run_once = 0;
    let mut keep_tile_num = 0usize;
    let mut keep_initial_value: i32 = 0;
    let min_line = h_grid.i_igf_min_line as i32;
    let start_line = h_grid.i_igf_start_line as i32;

    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &mut work_mem[tile];
        let t0 = tile * IGF_TILE_LEN;

        if h_map.i_whitening_level == WHITENING_MID {
            /* If INF is active then all tiles are whitened independently. If, however, INF is not
             * active, then the first tile is whitened and copied to the rest */
            if flag_inf_active != 0 {
                get_white_spectral_data(
                    None,
                    &mut spectrum_tab_array[t0..t0 + IGF_TILE_LEN],
                    working_array,
                    h_work_mem,
                    min_line,
                    start_line,
                );
            } else if run_once == 0 {
                get_white_spectral_data(
                    Some(p_spectral_data),
                    &mut spectrum_tab_array[t0..t0 + IGF_TILE_LEN],
                    working_array,
                    h_work_mem,
                    min_line,
                    start_line,
                );
                keep_initial_value = h_work_mem.initial_exp[0] as i32;
                keep_tile_num = tile;
                run_once = 1;
            } else {
                /* when run once, then just copy  for the other tiles */
                let k0 = keep_tile_num * IGF_TILE_LEN;
                let n = h_grid.i_igf_start_line as usize;
                spectrum_tab_array.copy_within(k0..k0 + n, t0);
                h_work_mem.initial_exp[0] = keep_initial_value as i8;
            }
        }
        if h_map.i_whitening_level == WHITENING_STRONG {
            /* Define a constant */
            /*2097152 is 0x4000000 in Q22.9. Maximum left alligned */
            h_work_mem.initial_exp[0] = 22;

            /* first target line: */
            let mut tb = h_map.i_des_start as usize;
            /* width of the current tile: */
            let width = h_map.i_width as i32;

            let width_u = width.max(0) as usize;
            let p2_tile = &mut spectrum_tab_array[t0..t0 + width_u];
            p2_tile.fill(0);

            /* The RNG acts differently for mono and stereo */
            if mono_or_stereo_flag == 0 {
                for v in p2_tile.iter_mut() {
                    if p_spectral_data[tb] == 0 {
                        let rr = random_sign(random_seed);
                        *v = rr.wrapping_mul(0x40000000);
                    }
                    tb += 1;
                }
            } else {
                for v in p2_tile.iter_mut() {
                    let rr = random_sign(random_seed);
                    if p_spectral_data[tb] == 0 {
                        *v = rr.wrapping_mul(0x40000000);
                    }
                    tb += 1;
                }
            }
        } /* end else if( WHITENING_STRONG == hMap->iWhiteningLevel ) */
    } /* end for ( tile = 0.... */
}

/// Sums the energies of the windows `start_win..end_win` of one sfb after aligning them to the
/// maximum exponent plus `shift`. Returns the sum and the alignment exponent.
fn sum_group_energies(
    val: &[FixpDbl],
    exp: &[i8],
    sfb: usize,
    start_win: i32,
    end_win: i32,
    shift: i32,
) -> (FixpDbl, i32) {
    /* Find the maximum exponent to allign to */
    let mut max = -100;
    for win in start_win..end_win {
        let e = exp[win as usize * IGF_MAX_SFB_SHORT + sfb] as i32;
        if e > max {
            max = e;
        }
    }

    /* To guarantee that no overflow occurs */
    max += shift;

    /* Allign and Add energy values for all windows */
    let mut sum: FixpDbl = 0;
    for win in start_win..end_win {
        let k = win as usize * IGF_MAX_SFB_SHORT + sfb;
        let s_shift = exp[k] as i32;
        /* Limit the shift to 31 */
        let corrected_shift = f_max(-31, s_shift - max);
        sum = sum.wrapping_add(scale_value(val[k], corrected_shift));
    }
    (sum, max)
}

/// `iisIGFDecoderCollectEnergiesMonoNormalize()`
fn iis_igf_decoder_collect_energies_mono_normalize(
    h_grid: &IgfGridInfo,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    window_sequence: i32,
    group_length: u8,
    start_win: i32,
) {
    if window_sequence == IGF_GRID_SHORT_WINDOW {
        /*  Only for short windows */
        if group_length == 1 {
            /* IF "1" then copy */
            /* over all tiles: */
            for tile in 0..h_grid.i_igf_num_tile as usize {
                let h_map = &h_grid.s_igf_map_info_tab[tile];
                let w = &mut work_mem[tile];

                /* over all sfb of tile: */
                for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
                    let k = start_win as usize * IGF_MAX_SFB_SHORT + sfb;
                    w.s_e[sfb] = w.f_sfb_survived_energy_tab[k];
                    w.s_e_exp[sfb] = w.f_sfb_survived_energy_tab_exp[k];
                    w.t_e[sfb] = w.f_sfb_tile_energy_tab[k];
                    w.t_e_exp[sfb] = w.f_sfb_tile_energy_tab_exp[k];
                } /* for sfb */
            } /* for tile */
        } else if group_length == 2 || group_length == 4 || group_length == 8 {
            /* If "groupLength" is a power of two */
            let end_win = start_win + group_length as i32;

            let mut shift = (group_length >> 1) as i32; /* cases 2 and 4 */
            if group_length == 8 {
                shift = 3; /* case 8 */
            }

            /* over all tiles: */
            for tile in 0..h_grid.i_igf_num_tile as usize {
                let h_map = &h_grid.s_igf_map_info_tab[tile];
                let w = &mut work_mem[tile];

                /* over all sfb of tile: */
                for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
                    /* Survived energy */
                    let (s_e, max) = sum_group_energies(
                        &w.f_sfb_survived_energy_tab,
                        &w.f_sfb_survived_energy_tab_exp,
                        sfb,
                        start_win,
                        end_win,
                        shift,
                    );
                    w.s_e[sfb] = s_e;
                    w.s_e_exp[sfb] = (max - shift) as i8; /* Move binary point to the left to show division */

                    /* Tile energy */
                    let (t_e, max) = sum_group_energies(
                        &w.f_sfb_tile_energy_tab,
                        &w.f_sfb_tile_energy_tab_exp,
                        sfb,
                        start_win,
                        end_win,
                        shift,
                    );
                    w.t_e[sfb] = t_e;
                    w.t_e_exp[sfb] = (max - shift) as i8; /* Move binary point to the left to show division */
                } /* for sfb */
            } /* for tile */
        } else {
            /* The rest; the general case */
            let end_win = start_win + group_length as i32;

            let mut shift = 3; /* cases 5,6 and 7 */
            if group_length == 3 {
                shift = 2; /* case 3 */
            }

            /* over all tiles: */
            for tile in 0..h_grid.i_igf_num_tile as usize {
                let h_map = &h_grid.s_igf_map_info_tab[tile];
                let w = &mut work_mem[tile];

                /* over all sfb of tile: */
                for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
                    let mut temp_int = 0;

                    /* Survived energy */
                    let (s_e, max) = sum_group_energies(
                        &w.f_sfb_survived_energy_tab,
                        &w.f_sfb_survived_energy_tab_exp,
                        sfb,
                        start_win,
                        end_win,
                        shift,
                    );
                    /* Divide by the group length */
                    let s_e = f_div_norm(s_e, group_length as FixpDbl, &mut temp_int);
                    w.s_e[sfb] = s_e;
                    w.s_e_exp[sfb] = (temp_int + max - 31) as i8;

                    /* Tile energy */
                    let (t_e, max) = sum_group_energies(
                        &w.f_sfb_tile_energy_tab,
                        &w.f_sfb_tile_energy_tab_exp,
                        sfb,
                        start_win,
                        end_win,
                        shift,
                    );
                    /* Divide by the group length */
                    let t_e = f_div_norm(t_e, group_length as FixpDbl, &mut temp_int);
                    w.t_e[sfb] = t_e;
                    w.t_e_exp[sfb] = (temp_int + max - 31) as i8;
                } /* for sfb */
            } /* for tile */
        } /* end else */
    } else {
        /* for long windows */
        /* over all tiles: */
        for tile in 0..h_grid.i_igf_num_tile as usize {
            let h_map = &h_grid.s_igf_map_info_tab[tile];
            let w = &mut work_mem[tile];

            /* over all sfb of tile: */
            for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
                w.s_e[sfb] = w.f_sfb_survived_energy_tab[sfb];
                w.s_e_exp[sfb] = w.f_sfb_survived_energy_tab_exp[sfb];
                w.t_e[sfb] = w.f_sfb_tile_energy_tab[sfb];
                w.t_e_exp[sfb] = w.f_sfb_tile_energy_tab_exp[sfb];
            } /* for sfb */
        } /* for tile */
    }
}

/// `createIGF_array_inplace()`: copies `particular_tile` lines of the source spectrum
/// `p_spectral_data` to `temp_igf_band` according to the tile index.
fn create_igf_array_inplace(
    particular_tile: i32,
    particular_tile_idx: i32,
    particular_tile_bgn: i32,
    igf_bgn: i16,
    igf_min: i32,
    p_spectral_data: &[FixpDbl],
    temp_igf_band: &mut [FixpDbl],
) {
    let mut width = particular_tile;
    let igf_bgn = igf_bgn as i32;
    let src = igf_bgn - igf_min;
    let mut raw_offset_from_igf_bgn = 0;

    match particular_tile_idx {
        0 => {
            /* 2.5f */
            raw_offset_from_igf_bgn = (particular_tile << 1) + (particular_tile >> 1);
        }
        1 => {
            /* 2.0f */
            raw_offset_from_igf_bgn = particular_tile << 1;
        }
        2 => {
            /* 1.5f */
            raw_offset_from_igf_bgn = particular_tile + (particular_tile >> 1);
        }
        3 => {
            /* 1.0f */
            raw_offset_from_igf_bgn = particular_tile;
        }
        _ => {}
    }
    let mut offset = raw_offset_from_igf_bgn + particular_tile_bgn - igf_bgn;
    if (offset & 1) == 1 {
        offset += 1;
    }

    let mut sb = particular_tile_bgn - offset;
    let mut delta = igf_min - sb;
    let mut o = 0usize;

    /* If mapping field is below igfMin value */
    if delta > 0 {
        sb = igf_min + particular_tile_bgn % src;

        while width > 0 {
            width -= 1;
            temp_igf_band[o] = p_spectral_data[sb as usize];
            o += 1;
            sb += 1;
            delta -= 1;
            if delta == 0 || (sb % igf_bgn) == 0 {
                sb = igf_min;
            }
        }
    } else {
        while width > 0 {
            width -= 1;
            temp_igf_band[o] = p_spectral_data[sb as usize];
            o += 1;
            sb += 1;
        }
    }
}

/// `iisIGFDecoderCollectEnergiesMono()`: `temp_igf_band[*temp_pos..]` holds the mapped data of
/// the window, `*temp_pos` is advanced like the C++ pointer.
fn iis_igf_decoder_collect_energies_mono(
    h_private_static_data: &IgfPrivateStaticData,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    p_spectral_data: &[FixpDbl],
    spec_scale: i16,
    temp_igf_band: &[FixpDbl],
    temp_pos: &mut usize,
    window_sequence: i32,
    win: i32,
) {
    /* get the grid structure: */
    let h_grid = &h_private_static_data.s_grid_info_tab[window_sequence as usize];
    let wofs = win as usize * IGF_MAX_SFB_SHORT;

    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &mut work_mem[tile];

        /* first target line: */
        let mut tb = h_map.i_des_start as usize;

        for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
            /* width of the current sfb: */
            let width = h_map.i_sfb_width_tab[sfb] as i32;
            let wu = width.max(0) as usize;

            let p2 = *temp_pos;

            *temp_pos = (*temp_pos as i32 + width) as usize;

            let shift = get_scalefactor(&p_spectral_data[tb..tb + wu]);
            let shift1 = h_work_mem.intermediate_shift[wofs + sfb] as i32;

            /*Calculate the normalization necessary due to addition*/
            /* Check for power of two /special case */
            let mut width_shift = f_normz(width);
            /* Get the number of bits necessary minus one, because we need one sign bit only */
            width_shift = 31 - width_shift;

            let mut s_e: FixpDbl = 0;
            let mut t_e: FixpDbl = 0;

            /* over all lines in sfb of tile: */
            let src = &p_spectral_data[tb..tb + wu];
            let band = &temp_igf_band[p2..p2 + wu];
            tb += wu;
            for (&x, &y) in src.iter().zip(band) {
                /* sE += val * val;*/
                let val = x.wrapping_shl(shift as u32);
                let val_sgl = fx_dbl2fx_sgl(val);
                let s_e_temp = f_pow2_div2(val_sgl);

                /* tE += val * val;*/
                let val1 = y.wrapping_shl(shift1 as u32);
                let val1_sgl = fx_dbl2fx_sgl(val1);
                let mut t_e_temp = f_pow2_div2(val1_sgl);

                if val_sgl != 0 {
                    t_e_temp = 0;
                }

                /* summation of survived energy: */
                s_e = s_e.wrapping_add(s_e_temp >> width_shift);
                /* summation of tile energy: */
                t_e = t_e.wrapping_add(t_e_temp >> width_shift);
            } /* width */

            /* Store results */
            h_work_mem.f_sfb_survived_energy_tab[wofs + sfb] = s_e;
            h_work_mem.f_sfb_tile_energy_tab[wofs + sfb] = t_e;

            /* Calculating exponent of sE */
            let temp = 1 + ((spec_scale as i32 - shift) << 1) + width_shift;
            h_work_mem.f_sfb_survived_energy_tab_exp[wofs + sfb] = temp as i8;

            /* Calculating exponent of tE */
            let temp = 1
                + ((h_work_mem.initial_exp[wofs + sfb] as i32
                    - h_work_mem.intermediate_shift[wofs + sfb] as i32)
                    << 1)
                + width_shift;
            h_work_mem.f_sfb_tile_energy_tab_exp[wofs + sfb] = temp as i8;
        } /* for sfb */
    } /* for tile */

    /* Correction for the tile/sfb-pair split case */
    /* The ref soft finds the energy of the sfb-pair without splitting it. For this reason we mimic
    it here, by adding the two energy parts and dividing by two */
    if h_private_static_data.use_high_res == 0 && window_sequence == IGF_GRID_LONG_WINDOW {
        /* over all tiles: */
        for tile in 1..h_grid.i_igf_num_tile as usize {
            /* get the mapping structure: */
            let h_map = &h_grid.s_igf_map_info_tab[tile];

            if h_map.i_sfb_split != 0 {
                let w = &work_mem[tile];
                let s_e2 = w.f_sfb_survived_energy_tab[wofs];
                let s_e2_exp = w.f_sfb_survived_energy_tab_exp[wofs] as i32;
                let t_e2 = w.f_sfb_tile_energy_tab[wofs];
                let t_e2_exp = w.f_sfb_tile_energy_tab_exp[wofs] as i32;

                /* get the mapping structure of the previous tile */
                let h_map1 = &h_grid.s_igf_map_info_tab[tile - 1];
                let last_sfb = (h_map1.i_sfb_cnt as i32 - 1) as usize;
                let w1 = &work_mem[tile - 1];

                let s_e1 = w1.f_sfb_survived_energy_tab[wofs + last_sfb];
                let s_e1_exp = w1.f_sfb_survived_energy_tab_exp[wofs + last_sfb] as i32;
                let t_e1 = w1.f_sfb_tile_energy_tab[wofs + last_sfb];
                let t_e1_exp = w1.f_sfb_tile_energy_tab_exp[wofs + last_sfb] as i32;

                /* Find if a difference between the scalings exists.*/
                /* Add 1 so that we multiply by 0.5 */
                let (s_res, s_exp) = average_energies(s_e1, s_e1_exp, s_e2, s_e2_exp);
                let (t_res, t_exp) = average_energies(t_e1, t_e1_exp, t_e2, t_e2_exp);

                let w1 = &mut work_mem[tile - 1];
                w1.f_sfb_survived_energy_tab[wofs + last_sfb] = s_res;
                w1.f_sfb_survived_energy_tab_exp[wofs + last_sfb] = s_exp as i8;
                w1.f_sfb_tile_energy_tab[wofs + last_sfb] = t_res;
                w1.f_sfb_tile_energy_tab_exp[wofs + last_sfb] = t_exp as i8;

                let w = &mut work_mem[tile];
                w.f_sfb_survived_energy_tab[wofs] = s_res;
                w.f_sfb_survived_energy_tab_exp[wofs] = s_exp as i8;
                w.f_sfb_tile_energy_tab[wofs] = t_res;
                w.f_sfb_tile_energy_tab_exp[wofs] = t_exp as i8;
            } /* if(hMap->iSfbSplit) */
        } /* for tile */
    }
}

/// `(e1 + e2) / 2` of two energies with exponents (split sfb-pair correction).
fn average_energies(e1: FixpDbl, e1_exp: i32, e2: FixpDbl, e2_exp: i32) -> (FixpDbl, i32) {
    let tmp = e1_exp - e2_exp;
    let (l_scale, r_scale, out_exp);

    if tmp >= 0 {
        l_scale = 1;
        r_scale = f_min(tmp + 1, 31);
        out_exp = e1_exp;
    } else {
        l_scale = f_min(-tmp + 1, 31);
        r_scale = 1;
        out_exp = e2_exp;
    }

    ((e1 >> l_scale).wrapping_add(e2 >> r_scale), out_exp)
}

/// Maps the source spectrum of one tile to `temp_igf_band[pos..]` (whitened tiles are copied).
fn map_tile(
    h_map: &IgfMapInfo,
    h_grid: &IgfGridInfo,
    tile_spectrum: &[FixpDbl],
    tile_idx: u8,
    des_start: i16,
    temp_igf_band: &mut [FixpDbl],
) {
    let width = h_map.i_width as i32;

    if h_map.i_whitening_level != WHITENING_STRONG {
        create_igf_array_inplace(
            width,
            tile_idx as i32,
            des_start as i32,
            h_grid.i_igf_start_line,
            h_grid.i_igf_min_line as i32,
            tile_spectrum,
            temp_igf_band,
        );
    } else {
        let n = width.max(0) as usize;
        temp_igf_band[..n].copy_from_slice(&tile_spectrum[..n]);
    }
}

/// `iisIGFDecoderMappingMono()`
fn iis_igf_decoder_mapping_mono(
    h_grid: &IgfGridInfo,
    spectrum_tab_array: &[FixpDbl],
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    curr_tile_idx_tab: &[u8; IGF_MAX_TILES],
    temp_igf_band: &mut [FixpDbl],
    temp_pos: &mut usize,
    win: i32,
) {
    let wofs = win as usize * IGF_MAX_SFB_SHORT;

    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &mut work_mem[tile];
        let src = &spectrum_tab_array[tile * IGF_TILE_LEN + win as usize * 128..];

        map_tile(
            h_map,
            h_grid,
            src,
            curr_tile_idx_tab[tile],
            h_map.i_des_start,
            &mut temp_igf_band[*temp_pos..],
        );

        for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
            /* width of the current sfb: */
            let width = h_map.i_sfb_width_tab[sfb] as i32;
            let p = *temp_pos;

            h_work_mem.intermediate_shift[wofs + sfb] =
                get_scalefactor(&temp_igf_band[p..p + width.max(0) as usize]) as i8;

            h_work_mem.initial_exp[wofs + sfb] = h_work_mem.initial_exp[wofs];

            *temp_pos = (p as i32 + width) as usize;
        }
    }
}

/// `iisIGFDecoderMappingAndMSStereo()`: `tnf_mask` is the element TNF mask (1024 values).
fn iis_igf_decoder_mapping_and_ms_stereo(
    h_private_static_data_l: &IgfPrivateStaticData,
    h_private_static_data_r: &IgfPrivateStaticData,
    spectrum_tab_array_l: &[FixpDbl],
    spectrum_tab_array_r: &[FixpDbl],
    curr_tile_idx_tab_l: &[u8; IGF_MAX_TILES],
    curr_tile_idx_tab_r: &[u8; IGF_MAX_TILES],
    work_mem_l: &mut [IgfWorkMemory; IGF_MAX_TILES],
    work_mem_r: &mut [IgfWorkMemory; IGF_MAX_TILES],
    temp_igf_band_l: &mut [FixpDbl],
    temp_pos_l: &mut usize,
    temp_igf_band_r: &mut [FixpDbl],
    temp_pos_r: &mut usize,
    ms_used: &[u8],
    tnf_mask: &mut [u8],
    window_sequence: i32,
    win: i32,
    group: i32,
) {
    /* get the grid structure: */
    let h_grid_l = &h_private_static_data_l.s_grid_info_tab[window_sequence as usize];
    let h_grid_r = &h_private_static_data_r.s_grid_info_tab[window_sequence as usize];
    let wofs = win as usize * IGF_MAX_SFB_SHORT;

    /*make sure it is a common window*/
    debug_assert!(h_grid_l.i_igf_num_tile == h_grid_r.i_igf_num_tile);

    /* Adjust MsUsed to the IGF range and set pairing if active */
    let mut ms_idx = h_grid_l.i_igf_start_sfb as isize;
    let mut tnf_idx = h_grid_l.i_igf_start_line as usize;

    let mut sfb_step = 1;
    if h_private_static_data_l.use_high_res == 0 && window_sequence == IGF_GRID_LONG_WINDOW {
        sfb_step = 2;
    }

    /* over all tiles: */
    for tile in 0..h_grid_l.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map_l = &h_grid_l.s_igf_map_info_tab[tile];
        let h_map_r = &h_grid_r.s_igf_map_info_tab[tile];

        let h_work_mem_l = &mut work_mem_l[tile];
        let h_work_mem_r = &mut work_mem_r[tile];

        /* make sure it is common window: */
        debug_assert!(h_map_l.i_des_start == h_map_r.i_des_start);

        /*Just for MS/stereo case*/
        /* Calculate the decimal point difference for the particular window*/
        let delta = h_work_mem_l.initial_exp[wofs] as i32 - h_work_mem_r.initial_exp[wofs] as i32;
        /* Effectively Multiply by 0.5*/
        let mut shift_l = 1;
        let mut shift_r = 1;
        if delta > 0 {
            shift_r = f_min(31, shift_r + delta);
        } else {
            shift_l = f_min(31, shift_l - delta);
        }

        /* Map tile values according to mapping tables */
        let win_ofs = tile * IGF_TILE_LEN + win as usize * 128;
        map_tile(
            h_map_l,
            h_grid_l,
            &spectrum_tab_array_l[win_ofs..],
            curr_tile_idx_tab_l[tile],
            h_map_l.i_des_start,
            &mut temp_igf_band_l[*temp_pos_l..],
        );
        map_tile(
            h_map_r,
            h_grid_r,
            &spectrum_tab_array_r[win_ofs..],
            curr_tile_idx_tab_r[tile],
            h_map_l.i_des_start,
            &mut temp_igf_band_r[*temp_pos_r..],
        );

        /* When tile boundary is in the middle of a sfb pair */
        let mut sfb_jump = 0;
        if h_map_l.i_sfb_split != 0 {
            sfb_jump = 1;
            ms_idx -= 1;
        }

        for sfb in 0..h_map_l.i_sfb_cnt.max(0) as usize {
            /* width of the current sfb: */
            let width = h_map_l.i_sfb_width_tab[sfb] as i32;
            let wu = width.max(0) as usize;

            h_work_mem_l.initial_exp[wofs + sfb] = h_work_mem_l.initial_exp[wofs];
            h_work_mem_r.initial_exp[wofs + sfb] = h_work_mem_r.initial_exp[wofs];

            let pl = *temp_pos_l;
            let pr = *temp_pos_r;

            /*Just for MS/stereo case*/
            if (ms_used[ms_idx as usize] & (1u8 << group)) != 0 {
                for k in 0..wu {
                    /*Shift left to get maximum resolution of the output.*/
                    let val_l = temp_igf_band_l[pl + k] >> shift_l;
                    let val_r = temp_igf_band_r[pr + k] >> shift_r;

                    let val1_l = val_l.wrapping_add(val_r);
                    let val1_r = val_l.wrapping_sub(val_r);

                    temp_igf_band_l[pl + k] = val1_l;
                    temp_igf_band_r[pr + k] = val1_r;
                } /* while(width--) */

                tnf_mask[tnf_idx..tnf_idx + wu].fill(0x01);

                if delta >= 0 {
                    h_work_mem_r.initial_exp[wofs + sfb] =
                        h_work_mem_r.initial_exp[wofs + sfb].wrapping_add(delta as i8);
                } else {
                    h_work_mem_l.initial_exp[wofs + sfb] =
                        h_work_mem_l.initial_exp[wofs + sfb].wrapping_sub(delta as i8);
                }
            } /* if ( iUseMSTab[sfbOffset] ) */

            tnf_idx = (tnf_idx as i32 + width) as usize;

            /* Find leading zeroes */
            h_work_mem_l.intermediate_shift[wofs + sfb] =
                get_scalefactor(&temp_igf_band_l[pl..pl + wu]) as i8;
            h_work_mem_r.intermediate_shift[wofs + sfb] =
                get_scalefactor(&temp_igf_band_r[pr..pr + wu]) as i8;

            *temp_pos_l = (pl as i32 + width) as usize;
            *temp_pos_r = (pr as i32 + width) as usize;

            if sfb_jump != 0 {
                ms_idx += 1;
                sfb_jump = 0;
            } else {
                ms_idx += sfb_step;
            }
        }
    }
}

/// `iisIGFDecoderCalculateGainsNew()`
fn iis_igf_decoder_calculate_gains_new(
    h_grid: &IgfGridInfo,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
) {
    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &mut work_mem[tile];

        /* over all sfb of tile: */
        for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
            /* Allign the width of the current sfb to the left for maximum precision */
            let mut width: FixpSgl = h_map.i_sfb_width_tab[sfb];
            let headroom = f_normz(width) - 1;
            width = (width as i32).wrapping_shl(headroom as u32) as i16;

            /* destination energy: (is transmitted in bitstream) */
            let d_e = h_work_mem.f_sfb_destin_energy_tab[sfb];
            /*Get the exponent of the energy value*/
            let exp_d_e = h_work_mem.f_sfb_destin_energy_tab_exp[sfb] as i32;

            /* survived energy: (survived lines in destination region) */
            let mut s_e = h_work_mem.s_e[sfb];
            let exp_s_e = h_work_mem.s_e_exp[sfb] as i32;

            /* tile energy: (lines in source region) */
            let t_e = h_work_mem.t_e[sfb];
            let exp_t_e = h_work_mem.t_e_exp[sfb] as i32;

            /* missing energy: (energy to restore in destination region) */
            /* mE = dE * dE * width - sE;*/
            let mut temp_fixp_dbl = f_pow2(d_e);

            /*mE = dE * dE * width*/
            let mut m_e = f_mult_div2(temp_fixp_dbl, width);

            /*Calculate exp of mE*/
            let exp_m_e = (exp_d_e << 1) + (16 - headroom);

            /*mEtE_shift*/
            let m_e_t_e_shift = exp_m_e - exp_t_e;

            if s_e != 0 {
                /*sE shift calculation*/
                let mut shift_s_e = exp_s_e - exp_m_e;

                /* Limit shift_sE to 31*/
                shift_s_e = f_max(-31, shift_s_e);

                /*Check sE leading bits */
                let temp = f_norm(s_e);

                /* Check if sE and mE can be alligned */
                if shift_s_e > temp {
                    /* Obviously sE is greater than mE, so sE is set to max because of the
                     * substraction later */
                    s_e = MAXVAL_DBL;
                } else {
                    /*Shifting so that the output result of the next calcluation will be in the
                     * proper format */
                    s_e = scale_value(s_e, shift_s_e);
                }
            }

            /* mE = dE * dE * width - sE;*/
            m_e = m_e.wrapping_sub(s_e);

            /* calculate gain: */
            if m_e > 0 && t_e > 0 {
                /*Calculate
                hMap->fSfbGainTab[ sfb ] = MIN( 10.f, (float)sqrt( mE / tE) );*/

                /* result= mE / tE */
                let mut temp_int = 0;
                temp_fixp_dbl = f_div_norm(m_e, t_e, &mut temp_int);

                /* add to shift */
                temp_int += m_e_t_e_shift;

                /* sqrt(result) */
                let mut gain = sqrt_fixp_lookup_e(temp_fixp_dbl, &mut temp_int);

                if temp_int > 3 {
                    /* If value > 8 then generate 10.0 */
                    temp_fixp_dbl = scale_value(10, f_max(-31, 31 - temp_int));
                    /* MIN( 10.f, (float)sqrt( mE / tE) )
                    gain=fMin(gain,temp_FIXP_DBL);*/
                    if gain > temp_fixp_dbl {
                        /* 10.0 in Q4.27 */
                        gain = 0x50000000;
                        temp_int = 4;
                    }
                }

                /* Store the result */
                h_work_mem.f_sfb_gain_tab[sfb] = gain;
                h_work_mem.f_sfb_gain_tab_exp[sfb] = temp_int as i8;
            } else {
                h_work_mem.f_sfb_gain_tab[sfb] = 0;
                h_work_mem.f_sfb_gain_tab_exp[sfb] = 0;
            }
        } /* for sfb */
    } /* for tile */
}

/// `iisIGFDecLibCheckTileTransition()`: checks if a new tile index set was transmitted and
/// configures tilemapping
fn iis_igf_dec_lib_check_tile_transition(
    h_private_static_data: &mut IgfPrivateStaticData,
    bitstream_data: &IgfBitstreamData,
    window_sequence: i32,
) {
    let s = h_private_static_data;
    let h_grid = &mut s.s_grid_info_tab[window_sequence as usize];

    /*  check if the grid/mapping information is still valid: */
    let mut do_init = false;
    for tile in 0..h_grid.i_igf_num_tile as usize {
        if h_grid.i_igf_tile_idx_tab[tile] != bitstream_data.i_curr_tile_idx_tab[tile] {
            do_init = true;
        }
    }

    /* do initialization, if needed: */
    if do_init {
        match window_sequence {
            IGF_GRID_LONG_WINDOW => {
                /* AAC long blocks */
                iis_igf_dec_lib_init_grid_info(
                    h_grid,
                    IGF_GRID_LONG_WINDOW,
                    &s.sfb_offset_lb,
                    s.igf_start_sfb_lb as i32,
                    s.igf_stop_sfb_lb as i32,
                    s.igf_min_subband_lb,
                    s.use_high_res as i32,
                    s.igf_n_tiles_lb as i32,
                    &s.igf_tiles_lb,
                    &bitstream_data.i_curr_tile_idx_tab,
                );
            }
            IGF_GRID_SHORT_WINDOW => {
                /* AAC short blocks */
                iis_igf_dec_lib_init_grid_info(
                    h_grid,
                    IGF_GRID_SHORT_WINDOW,
                    &s.sfb_offset_sb,
                    s.igf_start_sfb_sb as i32,
                    s.igf_stop_sfb_sb as i32,
                    s.igf_min_subband_sb,
                    s.use_high_res as i32,
                    s.igf_n_tiles_sb as i32,
                    &s.igf_tiles_sb,
                    &bitstream_data.i_curr_tile_idx_tab,
                );
            }
            _ => {
                debug_assert!(false);
            }
        } /* switch window_sequence */
    } /* if doInit */

    /* map whitening levels: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        h_grid.s_igf_map_info_tab[tile].i_whitening_level =
            bitstream_data.igf_whitening_level[tile];
    }
}

/// `iisIGFDecoderApplyGainsMonoNew()`: mapping of source to destination and application of gain
/// values. `p_spectral_data` is the spectrum of the window, `temp_igf_band[*temp_pos..]` the mapped
/// data of the window.
fn iis_igf_decoder_apply_gains_mono_new(
    h_grid: &IgfGridInfo,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    virtual_spec: &mut [FixpDbl],
    p_spectral_data: &mut [FixpDbl],
    spec_scale: &mut i16,
    temp_igf_band: &[FixpDbl],
    temp_pos: &mut usize,
    window_sequence: i32,
    win: i32,
) {
    let wofs = win as usize * IGF_MAX_SFB_SHORT;
    let mut max_exponent = -1000;

    /* Find if an exponent adjustment  for the whole sequence is necessary */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &mut work_mem[tile];

        /* over all sfb of tile: */
        for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
            if h_work_mem.f_sfb_gain_tab[sfb] > 0 {
                /* Find headroom of the gain */
                let mut lead_zeros = f_normz(h_work_mem.f_sfb_gain_tab[sfb]) - 1;
                lead_zeros = f_max(0, lead_zeros);

                /* Left allign the gain */
                h_work_mem.f_sfb_gain_tab[sfb] <<= lead_zeros;

                /* Adjust the gain exponent */
                h_work_mem.f_sfb_gain_tab_exp[sfb] =
                    h_work_mem.f_sfb_gain_tab_exp[sfb].wrapping_sub(lead_zeros as i8);

                /* Headroom of data */
                let shift1 = h_work_mem.intermediate_shift[wofs + sfb] as i32;

                /* Calcuation of output exponent for the particular sfb */
                let mut shift = (h_work_mem.initial_exp[wofs + sfb] as i32 - shift1)
                    + h_work_mem.f_sfb_gain_tab_exp[sfb] as i32;

                /*  Limit shift_sE to 31 */
                shift = f_max(-31, shift);

                /* Find maximum exaponent */
                if shift > max_exponent {
                    max_exponent = shift;
                }
            } /* if(hWorkMem->fSfbGainTab[sfb] > 0) */
        } /* for(sfb = 0; sfb < hMap->iSfbCnt; sfb++) */
    } /* for (tile = 0; tile < hGrid->iIGFNumTile; tile++)*/

    /* Rescale the whole frame when the exponent in the IGF range is bigger than the specScale */
    let frame_length = if window_sequence != 0 { 128 } else { 1024 };

    if max_exponent > *spec_scale as i32 {
        /* Rescale main band */
        let diff = f_min(max_exponent - *spec_scale as i32, 31);
        for v in p_spectral_data[..frame_length].iter_mut() {
            *v >>= diff;
        }
        /* Change the exponent */
        *spec_scale = max_exponent as i16;
    } else {
        max_exponent = *spec_scale as i32;
    }

    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &work_mem[tile];

        /* first target line: */
        let mut tb = h_map.i_des_start as usize;

        /* over all sfb of tile: */
        for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
            /* width of the current sfb: */
            let width = h_map.i_sfb_width_tab[sfb] as i32;

            let mut shift1 = 0;
            let mut shift = 0;

            /* Make the gain a 16 bit value */
            let gain_sgl = fx_dbl2fx_sgl(h_work_mem.f_sfb_gain_tab[sfb]);

            if gain_sgl > 0 {
                shift1 = h_work_mem.intermediate_shift[wofs + sfb] as i32;

                /* Calcuation of shift value for output */
                shift = max_exponent
                    - ((h_work_mem.initial_exp[wofs + sfb] as i32 - shift1)
                        + h_work_mem.f_sfb_gain_tab_exp[sfb] as i32);

                shift = f_min(31, shift);
            }

            let p2 = *temp_pos;

            *temp_pos = (*temp_pos as i32 + width) as usize;

            debug_assert!((width & 3) == 0);
            /* over all lines in sfb of tile: */
            let wu = width.max(0) as usize;
            let src = &temp_igf_band[p2..p2 + wu];
            for (d, &t) in virtual_spec[tb..tb + wu].iter_mut().zip(src) {
                /* virtualSpec[tb] = hMap->fSfbGainTab[sfb] * hMap->fSpectrumTab[win][sb]; */
                /* When we flatten all values are filled from the main spectum, i.e. not only the
                zero ones. We create an additional array to the main spectral array and pass it
                to the TNF function */
                let temp = t.wrapping_shl(shift1 as u32);
                *d = f_mult(gain_sgl, temp).wrapping_shr(shift as u32);
            } /* width */
            tb += wu;
        } /* for sfb */
    } /* for tile */
}

/// `iisIGFDecoderFillMono()`
fn iis_igf_decoder_fill_mono(
    spectrum: &mut [FixpDbl],
    virtual_spec: &[FixpDbl],
    igf_start_line: i32,
    igf_stop_line: i32,
    tnf_mask: &mut [u8],
    flag_use: u8,
) {
    let (s, e) = (igf_start_line as usize, igf_stop_line.max(igf_start_line) as usize);
    let spectrum = &mut spectrum[s..e];
    let virtual_spec = &virtual_spec[s..e];
    if flag_use != 0 {
        /* When TNF in mono is used */
        /* Add data generated from IGF over the real data*/
        for ((x, &v), m) in spectrum.iter_mut().zip(virtual_spec).zip(&mut tnf_mask[s..e]) {
            let zero = *x == 0;
            if zero {
                *x = v;
            }
            *m = (!zero) as u8;
        }
    } else {
        /* When TNF in mono is not used */
        for (x, &v) in spectrum.iter_mut().zip(virtual_spec) {
            if *x == 0 {
                *x = v;
            }
        }
    }
}

/// `iisIGFDecoderFillStereo()`
fn iis_igf_decoder_fill_stereo(
    spectrum_l: &mut [FixpDbl],
    spectrum_r: &mut [FixpDbl],
    virtual_spec_l: &[FixpDbl],
    virtual_spec_r: &[FixpDbl],
    igf_start_line: i32,
    igf_stop_line: i32,
    tnf_mask: &[u8],
    tnf_mask_l: &mut [u8],
    tnf_mask_r: &mut [u8],
    flag_use: u8,
) {
    let (s, e) = (igf_start_line as usize, igf_stop_line.max(igf_start_line) as usize);
    if flag_use != 0 {
        /* When TNF in stereo is used */
        /* Add data generated from IGF over the real data*/
        for i in s..e {
            let mut temp_l = 1u8;
            let mut temp_r = 1u8;
            if spectrum_l[i] == 0 {
                spectrum_l[i] = virtual_spec_l[i];
                temp_l = 0;
            }
            if spectrum_r[i] == 0 {
                spectrum_r[i] = virtual_spec_r[i];
                temp_r = 0;
            }
            if tnf_mask[i] != 0 {
                temp_l = (temp_l != 0 || temp_r != 0) as u8;
                temp_r = temp_l;
            }
            tnf_mask_l[i] = temp_l;
            tnf_mask_r[i] = temp_r;
        }
    } else {
        /* When TNF in stereo is not used */
        for i in s..e {
            if spectrum_l[i] == 0 {
                spectrum_l[i] = virtual_spec_l[i];
            }
            if spectrum_r[i] == 0 {
                spectrum_r[i] = virtual_spec_r[i];
            }
        }
    }
}

/// `iisIGFDecoderDecodeDestinEnergies()`
fn iis_igf_decoder_decode_destin_energies(
    h_private_static_data: &IgfPrivateStaticData,
    bitstream_data: &IgfBitstreamData,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    window_sequence: i32,
    group: i32,
) {
    /* get the grid structure: */
    let h_grid = &h_private_static_data.s_grid_info_tab[window_sequence as usize];

    let mut p2sfe = group as isize * h_grid.i_igf_num_sfb as isize;

    let mut sfb_step = 1;
    if h_private_static_data.use_high_res == 0 && window_sequence == IGF_GRID_LONG_WINDOW {
        sfb_step = 2;
    }

    /* over all tiles: */
    for tile in 0..h_grid.i_igf_num_tile as usize {
        /* get the mapping structure: */
        let h_map = &h_grid.s_igf_map_info_tab[tile];
        let h_work_mem = &mut work_mem[tile];

        /* When tile boundary is in the middle of a sfb pair */
        let mut sfb_jump = 0;
        if h_map.i_sfb_split != 0 {
            sfb_jump = 1;
            p2sfe -= 1;
        }

        /* over all sfb of tile: */
        for sfb in 0..h_map.i_sfb_cnt.max(0) as usize {
            /*Data is in Q7.0, i.e from 0 to 127
            0.25 * hMap->fSfbDestinEnergyTab[ sfb ]
            Multiply implicitly by 0.25
            Assuming result in Q5.2*/
            let sfe = bitstream_data.sfe[p2sfe as usize] as i32;
            let mut exp_dbl = 0;

            /* pow( 2.0, 0.25 * hMap->fSfbDestinEnergyTab[ sfb ] ) */
            let temp_fixp_dbl = f2_pow(sfe, 29, &mut exp_dbl);

            if sfb_jump != 0 {
                p2sfe += 1;
                sfb_jump = 0;
            } else {
                p2sfe += sfb_step;
            }

            /* Store result and exponent of the result */
            h_work_mem.f_sfb_destin_energy_tab[sfb] = temp_fixp_dbl;

            h_work_mem.f_sfb_destin_energy_tab_exp[sfb] = exp_dbl as i8;
        } /* for sfb */
    } /* for tile */
}

/// Clean computational error in the IGF frequency range so that computed zeroes be recognized by
/// the algo. Non-MPEG/Out of standard procedure
fn clean_igf_range(
    p2_spectrum: &mut [FixpDbl],
    number_of_spectra: i32,
    num_mdct_lines: usize,
    igf_start_line: i32,
    igf_stop_line: i32,
) {
    if igf_start_line >= igf_stop_line {
        return;
    }
    let (start, stop) = (igf_start_line as usize, igf_stop_line as usize);
    for win in 0..number_of_spectra.max(0) as usize {
        let base = win * num_mdct_lines;
        for v in p2_spectrum[base + start..base + stop].iter_mut() {
            if *v > -IGF_CLEAN_THRESHOLD && *v < IGF_CLEAN_THRESHOLD {
                *v = 0;
            }
        }
    }
}

/// `CIgf_apply(hPrivateStaticData, hPrivateData, p2_spectrum, specScale, window_sequence,
/// numOfGroups, NumberOfSpectra, groupLength, randomSeed, TNF_mask, flag_INF_active, frameType)`:
/// applies IGF to the spectrum of a single channel.
///
/// `common` is `hPrivateData->IGF_Common_channel_data_handle`, `virtual_spec` its `virtualSpec`
/// (1024 values). `p2_spectrum` holds the spectrum (1024 lines, for short blocks 8 windows of 128
/// lines), `spec_scale` the exponents of the windows, `tnf_mask` 1024 values.
pub fn cigf_apply(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    common: &mut IgfPrivateDataCommon,
    virtual_spec: &mut [FixpDbl],
    p2_spectrum: &mut [FixpDbl],
    spec_scale: &mut [i16],
    window_sequence: i32,
    num_of_groups: i32,
    number_of_spectra: i32,
    group_length: &[u8],
    random_seed: &mut u32,
    tnf_mask: &mut [u8],
    flag_inf_active: u8,
    frame_type: i32,
) {
    let ft = frame_type as usize;
    let mut win_offset: i32 = 0;
    let mut temp_igf_band = [0 as FixpDbl; MAX_SPECTRUM_LEN];

    /* Skip IGF processing if no data is transmitted*/
    if h_private_data.bitstream_data[ft].igf_all_zero != 0 {
        return;
    }

    let ws = window_sequence as usize;

    /* Set parameters for long and short blocks */
    let num_mdct_lines = if window_sequence != IGF_GRID_SHORT_WINDOW { 1024 } else { 128 };

    /* Clean computational error in the IGF frequency range so that computed zeroes be recognized
     * by the algo. Non-MPEG/Out of standard procedure */
    let igf_start_line = h_private_static_data.s_grid_info_tab[ws].i_igf_start_line as i32;
    let igf_stop_line = h_private_static_data.s_grid_info_tab[ws].i_igf_stop_line as i32;

    clean_igf_range(p2_spectrum, number_of_spectra, num_mdct_lines, igf_start_line, igf_stop_line);

    if window_sequence == IGF_GRID_LONG_WINDOW || window_sequence == IGF_GRID_SHORT_WINDOW {
        /* If INF is active=> tiles have been initialized (injected) and have undergone processing
        earlier. When INF is not active tiles can be initialized just before the IGF processing.
        In this way we save a lot of processing power. */
        if flag_inf_active != 0 {
            /* Initialize scale factors for windows and tiles*/
            iis_igf_dec_lib_init_fract_scale_values(
                h_private_static_data,
                &mut common.igf_working_mem,
                window_sequence,
            );
        } else {
            /* INF not active */
            /* Copy spectrum and scale factors to tiles for further processing */
            iis_igf_dec_lib_inject_source_spectrum(
                h_private_static_data,
                h_private_data,
                &mut common.igf_working_mem,
                p2_spectrum,
                spec_scale,
                window_sequence,
                frame_type,
            );
        } /* (flag_INF_active)*/
    }

    let h_grid = &h_private_static_data.s_grid_info_tab[ws];
    let work_mem = &mut common.igf_working_mem;

    /* Applicable just for long windows; When whitening is used */
    if window_sequence != IGF_GRID_SHORT_WINDOW && h_private_static_data.igf_use_whitening != 0 {
        /* "temp_IGF_band" is passed as a working/intermediate result array */
        iis_igf_decoder_apply_whitening(
            &mut h_private_data.spectrum_tab_array,
            work_mem,
            h_grid,
            p2_spectrum,
            &mut temp_igf_band,
            random_seed,
            flag_inf_active,
            0,
        );
    }

    let bd = &h_private_data.bitstream_data[ft];

    /* Check if TNF is active in mono */
    let flag_tnf_active =
        (window_sequence != IGF_GRID_SHORT_WINDOW && bd.igf_use_enf_flat != 0) as u8;

    for group in 0..num_of_groups {
        /* memorize the offset of windows in a group: */
        let start_win = win_offset;
        let gl = group_length[group as usize];

        let mut temp_pos = 0usize;

        /* collect energies for each window in the group */
        for _win in 0..gl {
            let keep_temp_pos = temp_pos;

            /* Map data from source to IGF band */
            iis_igf_decoder_mapping_mono(
                h_grid,
                &h_private_data.spectrum_tab_array,
                work_mem,
                &bd.i_curr_tile_idx_tab,
                &mut temp_igf_band,
                &mut temp_pos,
                win_offset,
            );

            /* Calculate survived and tile energies */
            let mut p = keep_temp_pos;
            iis_igf_decoder_collect_energies_mono(
                h_private_static_data,
                work_mem,
                &p2_spectrum[win_offset as usize * 128..],
                spec_scale[win_offset as usize],
                &temp_igf_band,
                &mut p,
                window_sequence,
                win_offset,
            );
            win_offset += 1;
        }

        /* Normalize energy to the length of the group /Just for short windows */
        iis_igf_decoder_collect_energies_mono_normalize(
            h_grid,
            work_mem,
            window_sequence,
            gl,
            start_win,
        );

        /* decode the transmitted destination energies: */
        iis_igf_decoder_decode_destin_energies(
            h_private_static_data,
            bd,
            work_mem,
            window_sequence,
            group,
        );

        /* Calculate the gains for each sfb and tile */
        iis_igf_decoder_calculate_gains_new(h_grid, work_mem);

        /* apply IGF with computed energies: */
        win_offset = start_win;

        temp_pos = 0;

        for _win in 0..gl {
            let wo = win_offset as usize;
            /* Apply calculated gains to mapped data */
            iis_igf_decoder_apply_gains_mono_new(
                h_grid,
                work_mem,
                virtual_spec,
                &mut p2_spectrum[wo * 128..],
                &mut spec_scale[wo],
                &temp_igf_band,
                &mut temp_pos,
                window_sequence,
                win_offset,
            );

            iis_igf_decoder_fill_mono(
                &mut p2_spectrum[wo * 128..],
                virtual_spec,
                igf_start_line,
                igf_stop_line,
                tnf_mask,
                flag_tnf_active,
            );

            win_offset += 1;
        } /* for win */
    } /* for grp */
}

/// `CIgf_apply_stereo(...)`: applies IGF to the two channels of a channel pair with common window
/// (with M/S processing of the mapped tiles according to `ms_used`, `MsUsed[sfb]` with one bit per
/// window group).
///
/// See [`cigf_apply`] for the arguments; `_l`/`_r` are the left and right channel.
pub fn cigf_apply_stereo(
    h_private_static_data_l: &mut IgfPrivateStaticData,
    h_private_static_data_r: &mut IgfPrivateStaticData,
    h_private_data_l: &mut IgfPrivateData,
    h_private_data_r: &mut IgfPrivateData,
    common_l: &mut IgfPrivateDataCommon,
    common_r: &mut IgfPrivateDataCommon,
    virtual_spec_l: &mut [FixpDbl],
    virtual_spec_r: &mut [FixpDbl],
    p2_spectrum_l: &mut [FixpDbl],
    p2_spectrum_r: &mut [FixpDbl],
    spec_scale_l: &mut [i16],
    spec_scale_r: &mut [i16],
    window_sequence: i32,
    num_of_groups: i32,
    number_of_spectra: i32,
    group_length: &[u8],
    random_seed_l: &mut u32,
    random_seed_r: &mut u32,
    ms_used: &[u8],
    tnf_mask_l: &mut [u8],
    tnf_mask_r: &mut [u8],
    flag_inf_active: u8,
    frame_type: i32,
) {
    let ft = frame_type as usize;
    let mut win_offset: i32 = 0;
    let mut temp_igf_band_l = [0 as FixpDbl; MAX_SPECTRUM_LEN];
    let mut temp_igf_band_r = [0 as FixpDbl; MAX_SPECTRUM_LEN];

    /* Skip IGF processing if no data is transmitted*/
    if h_private_data_l.bitstream_data[ft].igf_all_zero != 0
        && h_private_data_r.bitstream_data[ft].igf_all_zero != 0
    {
        return;
    }

    let ws = window_sequence as usize;

    /*Set parameters for long and short blocks*/
    let num_mdct_lines = if window_sequence != IGF_GRID_SHORT_WINDOW { 1024 } else { 128 };

    /* Clean computational error in the IGF frequency range so that computed zeroes be recognized
     * by the algo. Non-MPEG/Out of standard procedure */
    let mut igf_start_line = h_private_static_data_l.s_grid_info_tab[ws].i_igf_start_line as i32;
    let mut igf_stop_line = h_private_static_data_l.s_grid_info_tab[ws].i_igf_stop_line as i32;

    clean_igf_range(
        p2_spectrum_l,
        number_of_spectra,
        num_mdct_lines,
        igf_start_line,
        igf_stop_line,
    );

    igf_start_line = h_private_static_data_r.s_grid_info_tab[ws].i_igf_start_line as i32;
    igf_stop_line = h_private_static_data_r.s_grid_info_tab[ws].i_igf_stop_line as i32;

    clean_igf_range(
        p2_spectrum_r,
        number_of_spectra,
        num_mdct_lines,
        igf_start_line,
        igf_stop_line,
    );

    /* If INF is active=> tiles have been initialized (injected) and have undergone processing
    earlier. When INF is not active tiles can be initialized just before the IGF processing. In
    this way we save a lot of processing power. */
    if flag_inf_active != 0 {
        /*Initialize scale factors for windows and tiles*/
        iis_igf_dec_lib_init_fract_scale_values(
            h_private_static_data_l,
            &mut common_l.igf_working_mem,
            window_sequence,
        );
        iis_igf_dec_lib_init_fract_scale_values(
            h_private_static_data_r,
            &mut common_r.igf_working_mem,
            window_sequence,
        );
    } else {
        /* INF not active */
        /* Copy spectrum and scale factors to tiles for further processing */
        iis_igf_dec_lib_inject_source_spectrum(
            h_private_static_data_l,
            h_private_data_l,
            &mut common_l.igf_working_mem,
            p2_spectrum_l,
            spec_scale_l,
            window_sequence,
            IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
        );
        iis_igf_dec_lib_inject_source_spectrum(
            h_private_static_data_r,
            h_private_data_r,
            &mut common_r.igf_working_mem,
            p2_spectrum_r,
            spec_scale_r,
            window_sequence,
            IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
        );
    }

    let static_l: &IgfPrivateStaticData = h_private_static_data_l;
    let static_r: &IgfPrivateStaticData = h_private_static_data_r;
    let h_grid_l = &static_l.s_grid_info_tab[ws];
    let h_grid_r = &static_r.s_grid_info_tab[ws];
    let work_mem_l = &mut common_l.igf_working_mem;
    let work_mem_r = &mut common_r.igf_working_mem;

    /*Applicable just for long windows; When whitening is used*/
    if window_sequence != IGF_GRID_SHORT_WINDOW && static_l.igf_use_whitening != 0 {
        /* "temp_IGF_bandL" is passed as a working/intermediate result array*/
        iis_igf_decoder_apply_whitening(
            &mut h_private_data_l.spectrum_tab_array,
            work_mem_l,
            h_grid_l,
            p2_spectrum_l,
            &mut temp_igf_band_l,
            random_seed_l,
            flag_inf_active,
            1,
        );
    }
    if window_sequence != IGF_GRID_SHORT_WINDOW && static_r.igf_use_whitening != 0 {
        /*"temp_IGF_bandR" is passed as a working/intermediate result array*/
        iis_igf_decoder_apply_whitening(
            &mut h_private_data_r.spectrum_tab_array,
            work_mem_r,
            h_grid_r,
            p2_spectrum_r,
            &mut temp_igf_band_r,
            random_seed_r,
            flag_inf_active,
            1,
        );
    }

    let bd_l = &h_private_data_l.bitstream_data[ft];
    let bd_r = &h_private_data_r.bitstream_data[ft];

    /* Check if TNF is active in stereo */
    let flag_tnf_active = (window_sequence != IGF_GRID_SHORT_WINDOW
        && (bd_l.igf_use_enf_flat != 0 || bd_r.igf_use_enf_flat != 0))
        as u8;

    /* Only TNF_mask[IGFstartLine..IGFstopLine] is cleared (and read) in C++. */
    let mut tnf_mask = [0u8; MAX_SPECTRUM_LEN];

    for group in 0..num_of_groups {
        /* memorize the offset of windows in a group: */
        let start_win = win_offset;
        let gl = group_length[group as usize];

        let mut temp_pos_l = 0usize;
        let mut temp_pos_r = 0usize;

        /* collect energies for each window in the group */
        for _win in 0..gl {
            let keep_temp_pos_l = temp_pos_l;
            let keep_temp_pos_r = temp_pos_r;

            /* Map data from source to IGF band and implement MS */
            iis_igf_decoder_mapping_and_ms_stereo(
                static_l,
                static_r,
                &h_private_data_l.spectrum_tab_array,
                &h_private_data_r.spectrum_tab_array,
                &bd_l.i_curr_tile_idx_tab,
                &bd_r.i_curr_tile_idx_tab,
                work_mem_l,
                work_mem_r,
                &mut temp_igf_band_l,
                &mut temp_pos_l,
                &mut temp_igf_band_r,
                &mut temp_pos_r,
                ms_used,
                &mut tnf_mask,
                window_sequence,
                win_offset,
                group,
            );

            let wo = win_offset as usize;

            /* Calculate survived and tile energies */
            let mut p = keep_temp_pos_l;
            iis_igf_decoder_collect_energies_mono(
                static_l,
                work_mem_l,
                &p2_spectrum_l[wo * 128..],
                spec_scale_l[wo],
                &temp_igf_band_l,
                &mut p,
                window_sequence,
                win_offset,
            );

            /* Calculate survived and tile energies */
            let mut p = keep_temp_pos_r;
            iis_igf_decoder_collect_energies_mono(
                static_r,
                work_mem_r,
                &p2_spectrum_r[wo * 128..],
                spec_scale_r[wo],
                &temp_igf_band_r,
                &mut p,
                window_sequence,
                win_offset,
            );

            win_offset += 1;
        }

        /*Normalize energy to the length of the group Just for short windows*/
        iis_igf_decoder_collect_energies_mono_normalize(
            h_grid_l,
            work_mem_l,
            window_sequence,
            gl,
            start_win,
        );
        iis_igf_decoder_collect_energies_mono_normalize(
            h_grid_r,
            work_mem_r,
            window_sequence,
            gl,
            start_win,
        );

        /*Decode the transmitted destination energies*/
        iis_igf_decoder_decode_destin_energies(static_l, bd_l, work_mem_l, window_sequence, group);

        /*Decode the transmitted destination energies*/
        iis_igf_decoder_decode_destin_energies(static_r, bd_r, work_mem_r, window_sequence, group);

        /* Calculate the gains for each sfb and tile */
        iis_igf_decoder_calculate_gains_new(h_grid_l, work_mem_l);

        /* Calculate the gains for each sfb and tile */
        iis_igf_decoder_calculate_gains_new(h_grid_r, work_mem_r);

        /* apply IGF with computed energies: */
        win_offset = start_win;

        temp_pos_l = 0;
        temp_pos_r = 0;

        for _win in 0..gl {
            let wo = win_offset as usize;
            /* Apply calculated gains to mapped data */
            iis_igf_decoder_apply_gains_mono_new(
                h_grid_l,
                work_mem_l,
                virtual_spec_l,
                &mut p2_spectrum_l[wo * 128..],
                &mut spec_scale_l[wo],
                &temp_igf_band_l,
                &mut temp_pos_l,
                window_sequence,
                win_offset,
            );
            /* Apply calculated gains to mapped data */
            iis_igf_decoder_apply_gains_mono_new(
                h_grid_r,
                work_mem_r,
                virtual_spec_r,
                &mut p2_spectrum_r[wo * 128..],
                &mut spec_scale_r[wo],
                &temp_igf_band_r,
                &mut temp_pos_r,
                window_sequence,
                win_offset,
            );

            iis_igf_decoder_fill_stereo(
                &mut p2_spectrum_l[wo * 128..],
                &mut p2_spectrum_r[wo * 128..],
                virtual_spec_l,
                virtual_spec_r,
                igf_start_line,
                igf_stop_line,
                &tnf_mask,
                tnf_mask_l,
                tnf_mask_r,
                flag_tnf_active,
            );

            win_offset += 1;
        } /* for win */
    } /* for grp */
}

/// `iisIGFDecLibInjectSourceSpectrum()`: copies spectral data to the internal IGF representation
fn iis_igf_dec_lib_inject_source_spectrum(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    work_mem: &mut [IgfWorkMemory; IGF_MAX_TILES],
    p_spectral_data: &[FixpDbl],
    spec_scale: &[i16],
    window_sequence: i32,
    frame_type: i32,
) {
    iis_igf_dec_lib_check_tile_transition(
        h_private_static_data,
        &h_private_data.bitstream_data[frame_type as usize],
        window_sequence,
    );

    let h_grid = &h_private_static_data.s_grid_info_tab[window_sequence as usize];

    /* Set parameters for long and short blocks */
    let (num_windows, copy_nr) = if window_sequence == IGF_GRID_SHORT_WINDOW {
        (8usize, 1024usize)
    } else {
        (1usize, h_grid.i_igf_start_line.max(0) as usize)
    };

    for t in 0..h_grid.i_igf_num_tile as usize {
        /* copy from pSpectralData to the internal representation: */
        let t0 = t * IGF_TILE_LEN;
        h_private_data.spectrum_tab_array[t0..t0 + copy_nr]
            .copy_from_slice(&p_spectral_data[..copy_nr]);
    }

    for t in 0..h_grid.i_igf_num_tile as usize {
        /* Copy the scale values for windows in an internal array */
        for win in 0..num_windows {
            work_mem[t].initial_exp[win * IGF_MAX_SFB_SHORT] = spec_scale[win] as i8;
        }
    }
}

/// `iisIGFDecLibInjectSourceSpectrumNew(hPrivateStaticData, hPrivateData, pSpectralData,
/// pSpectralData_exp, window_sequence, frameType)`: copies the spectrum (1024 values) and the sfb
/// exponents (`IGF_MAX_WIN * IGF_MAX_SFB_SHORT` values) into all tiles (INF).
pub fn iis_igf_dec_lib_inject_source_spectrum_new(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    p_spectral_data: &[FixpDbl],
    p_spectral_data_exp: &[i16],
    window_sequence: i32,
    frame_type: i32,
) {
    iis_igf_dec_lib_check_tile_transition(
        h_private_static_data,
        &h_private_data.bitstream_data[frame_type as usize],
        window_sequence,
    );

    let n_tiles =
        h_private_static_data.s_grid_info_tab[window_sequence as usize].i_igf_num_tile as usize;

    for t in 0..n_tiles {
        /* copy from pSpectralData scaling to the internal representation: */
        h_private_static_data.f_spectrum_tab_sfb_exp[t]
            .copy_from_slice(&p_spectral_data_exp[..IGF_MAX_WIN * IGF_MAX_SFB_SHORT]);
    }

    for t in 0..n_tiles {
        /* copy from pSpectralData to the internal representation: */
        let t0 = t * IGF_TILE_LEN;
        h_private_data.spectrum_tab_array[t0..t0 + 1024].copy_from_slice(&p_spectral_data[..1024]);
    }
}

/// `iisIGFGetStopSfbSB()`: pass `sfb_offset[len]` for shortblocks! returns IGF stop sfb for short
/// blocks
fn iis_igf_get_stop_sfb_sb(igf_stop_subband_lb: i32, sfb_offset: &[i16], len: i32) -> i32 {
    let mut igf_stop_sfb_sb = len;

    for sfb in 0..len {
        if sfb_offset[sfb as usize] as i32 >= igf_stop_subband_lb >> 3 {
            igf_stop_sfb_sb = sfb;
            break;
        }
    }

    igf_stop_sfb_sb
}

/// `iisIGFGetStopSfbLB()`: pass `sfb_offset[len]` for longblocks! returns IGF stop sfb
fn iis_igf_get_stop_sfb_lb(igf_start_sfb_lb: i32, igf_stop_idx: i32, len: i32) -> i32 {
    if igf_stop_idx != 15 {
        f_min(
            len,
            f_max(
                igf_start_sfb_lb + (((len - (igf_start_sfb_lb + 1)) * (igf_stop_idx + 2)) >> 4),
                igf_start_sfb_lb + 1,
            ),
        )
    } else {
        len
    }
}

/// `iisIGFGetStartSfbLB()`: returns IGF start sfb
fn iis_igf_get_start_sfb_lb(igf_start_idx: i32, len: i32) -> i32 {
    f_min(11 + igf_start_idx, len - 5)
}

/// `iisIGFGetStartSfbSB()`: pass `sfb_offset[len]` for shortblocks! returns IGF start sfb for short
/// blocks
fn iis_igf_get_start_sfb_sb(igf_start_subband_lb: i32, sfb_offset: &[i16], len: i32) -> i32 {
    let mut igf_start_sfb_sb = len;

    for sfb in 0..len {
        if sfb_offset[sfb as usize] as i32 >= igf_start_subband_lb >> 3 {
            igf_start_sfb_sb = sfb;
            break;
        }
    }

    igf_start_sfb_sb
}

/// Copies `len + 1` values of a scale factor band offset table.
fn copy_sfb_offsets(dst: &mut [i16; IGF_SFB_OFFSET_TAB_LEN], src: &[i16], len: i32) {
    let n = f_min((len + 1).max(0) as usize, src.len());
    dst.fill(0);
    dst[..n].copy_from_slice(&src[..n]);
}

/// `iisIGFDecLibInit(hPrivateStaticData, hPrivateData, hPrivateCommonData, igfStartIndex,
/// igfStopIndex, igfUseHighRes, igfUseWhitening, aacSampleRate, aacFrameLength, sfb_offset_LB,
/// len_LB, sfb_offset_SB, len_SB, enhancedNoiseFilling, igfAfterTnsSynth, igfIndependentTiling)`.
///
/// The C++ function only stores `hPrivateCommonData` in `hPrivateData`; that parameter is omitted
/// (see the module documentation). `sfb_offset_lb`/`sfb_offset_sb` hold at least `len_lb + 1` /
/// `len_sb + 1` values, which are copied.
pub fn iis_igf_dec_lib_init(
    h_private_static_data: &mut IgfPrivateStaticData,
    h_private_data: &mut IgfPrivateData,
    igf_start_index: u8,
    igf_stop_index: u8,
    igf_use_high_res: u8,
    igf_use_whitening: u8,
    aac_sample_rate: u32,
    aac_frame_length: i32,
    sfb_offset_lb: &[i16],
    len_lb: i32,
    sfb_offset_sb: &[i16],
    len_sb: i32,
    enhanced_noise_filling: u8,
    igf_after_tns_synth: u8,
    igf_independent_tiling: u8,
) {
    let s = h_private_static_data;
    let fixed_tile_idx: [u8; IGF_MAX_TILES] = [3, 3, 3, 3];

    s.use_high_res = igf_use_high_res;

    s.igf_start_sfb_lb = iis_igf_get_start_sfb_lb(igf_start_index as i32, len_lb) as u8;

    s.igf_stop_sfb_lb =
        iis_igf_get_stop_sfb_lb(s.igf_start_sfb_lb as i32, igf_stop_index as i32, len_lb) as u8;

    let start_line = sfb_offset_lb[s.igf_start_sfb_lb as usize];

    let stop_line = sfb_offset_lb[s.igf_stop_sfb_lb as usize];

    s.igf_start_sfb_sb = iis_igf_get_start_sfb_sb(start_line as i32, sfb_offset_sb, len_sb) as u8;

    s.igf_stop_sfb_sb = iis_igf_get_stop_sfb_sb(stop_line as i32, sfb_offset_sb, len_sb) as u8;

    s.num_sfb_lb = len_lb as u8;
    copy_sfb_offsets(&mut s.sfb_offset_lb, sfb_offset_lb, len_lb);

    s.num_sfb_sb = len_sb as u8;
    copy_sfb_offsets(&mut s.sfb_offset_sb, sfb_offset_sb, len_sb);

    let mut exp = 0;

    /* hPrivateStaticData->igfMinSubbandLB  = INT( (1125 * aacFrameLength ) / (aacSampleRate >> 1)
     * ); */
    let temp = f_div_norm(
        1125i32.wrapping_mul(aac_frame_length),
        (aac_sample_rate >> 1) as FixpDbl,
        &mut exp,
    );
    s.igf_min_subband_lb = temp.wrapping_shr((31 - exp) as u32);
    s.igf_min_subband_lb += s.igf_min_subband_lb % 2;

    /* hPrivateStaticData->igfMinSubbandSB  = INT( (1125 * aacFrameLength >> 3 ) / (aacSampleRate
     * >> 1) ); */
    let temp = f_div_norm(
        1125i32.wrapping_mul(aac_frame_length >> 3),
        (aac_sample_rate >> 1) as FixpDbl,
        &mut exp,
    );
    s.igf_min_subband_sb = temp.wrapping_shr((31 - exp) as u32);
    s.igf_min_subband_sb += s.igf_min_subband_sb % 2;

    s.igf_use_whitening = igf_use_whitening;

    s.enhanced_noise_filling = enhanced_noise_filling;
    s.igf_after_tns_synth = igf_after_tns_synth;
    s.igf_independent_tiling = igf_independent_tiling;

    s.igf_n_tiles_lb = get_igf_tile(
        s.igf_min_subband_lb,
        s.igf_start_sfb_lb as i32,
        s.igf_stop_sfb_lb as i32,
        s.use_high_res as i32,
        &s.sfb_offset_lb,
        0, /* swb shift factor */
        &mut s.igf_tiles_lb,
    ) as u8;

    s.igf_n_tiles_sb = get_igf_tile(
        s.igf_min_subband_sb,
        s.igf_start_sfb_sb as i32,
        s.igf_stop_sfb_sb as i32,
        s.use_high_res as i32,
        &s.sfb_offset_sb,
        0, /* swb shift factor */
        &mut s.igf_tiles_sb,
    ) as u8;

    iis_igf_scf_dec_lib_init(
        &mut s.h_igf_scf_decoder,
        s.igf_stop_sfb_lb as i32 - s.igf_start_sfb_lb as i32,
        s.igf_stop_sfb_sb as i32 - s.igf_start_sfb_sb as i32,
        s.igf_stop_sfb_sb as i32 - s.igf_start_sfb_sb as i32,
    );

    /* AAC long blocks */
    iis_igf_dec_lib_init_grid_info(
        &mut s.s_grid_info_tab[IGF_GRID_LONG_WINDOW as usize],
        IGF_GRID_LONG_WINDOW,
        &s.sfb_offset_lb,
        s.igf_start_sfb_lb as i32,
        s.igf_stop_sfb_lb as i32,
        s.igf_min_subband_lb,
        s.use_high_res as i32,
        s.igf_n_tiles_lb as i32,
        &s.igf_tiles_lb,
        &fixed_tile_idx,
    );

    /* AAC short blocks */
    iis_igf_dec_lib_init_grid_info(
        &mut s.s_grid_info_tab[IGF_GRID_SHORT_WINDOW as usize],
        IGF_GRID_SHORT_WINDOW,
        &s.sfb_offset_sb,
        s.igf_start_sfb_sb as i32,
        s.igf_stop_sfb_sb as i32,
        s.igf_min_subband_sb,
        s.use_high_res as i32,
        s.igf_n_tiles_sb as i32,
        &s.igf_tiles_sb,
        &fixed_tile_idx,
    );

    /* reset IGF */
    iis_igf_dec_lib_reset_igf(s, h_private_data, IGF_FRAME_DIVISION_AAC_OR_TCX_LONG);

    iis_igf_dec_lib_reset_igf(s, h_private_data, IGF_FRAME_DIVISION_TCX_SHORT_1);
}

/// `iisIGFDecLibAccessSourceSpectrum(hPrivateStaticData, hPrivateData, tileIdx, window_sequence,
/// win)`: the source spectrum of tile `tile_idx` from window `win` on (up to the end of the tile),
/// `None` if the tile does not exist.
pub fn iis_igf_dec_lib_access_source_spectrum<'a>(
    h_private_static_data: &IgfPrivateStaticData,
    h_private_data: &'a mut IgfPrivateData,
    tile_idx: i32,
    window_sequence: i32,
    win: i32,
) -> Option<&'a mut [FixpDbl]> {
    let h_grid = &h_private_static_data.s_grid_info_tab[window_sequence as usize];

    debug_assert!(win >= 0);
    if window_sequence == IGF_GRID_LONG_WINDOW {
        debug_assert!(win == 0);
    }
    if window_sequence == IGF_GRID_SHORT_WINDOW {
        debug_assert!(win <= 7);
    }

    if tile_idx >= h_grid.i_igf_num_tile as i32 {
        return None;
    }

    let t0 = tile_idx as usize * IGF_TILE_LEN;
    Some(&mut h_private_data.spectrum_tab_array[t0 + win as usize * 128..t0 + IGF_TILE_LEN])
}

/// `iisIGFDecLibAccessSourceSpectrum_exponent(hPrivateStaticData, tileIdx, window_sequence)`:
/// the sfb exponents of tile `tile_idx` (`fSpectrumTab_sfb_exp[tileIdx]`).
pub fn iis_igf_dec_lib_access_source_spectrum_exponent(
    h_private_static_data: &mut IgfPrivateStaticData,
    tile_idx: i32,
    window_sequence: i32,
) -> &mut [i16; IGF_MAX_WIN * IGF_MAX_SFB_SHORT] {
    debug_assert!(
        tile_idx
            <= h_private_static_data.s_grid_info_tab[window_sequence as usize].i_igf_num_tile
                as i32
    );

    &mut h_private_static_data.f_spectrum_tab_sfb_exp[tile_idx as usize]
}

/// `iisIGFDecLibAccessSourceSpectrum_Scale(hPrivateStaticData, hPrivateData, tileIdx,
/// window_sequence)`: the window exponents of tile `tile_idx` (`fSpectrumScale[tileIdx]`). The
/// unused `hPrivateData` parameter is omitted.
pub fn iis_igf_dec_lib_access_source_spectrum_scale(
    h_private_static_data: &mut IgfPrivateStaticData,
    tile_idx: i32,
    window_sequence: i32,
) -> &mut [i16; IGF_MAX_WIN] {
    debug_assert!(
        tile_idx
            < h_private_static_data.s_grid_info_tab[window_sequence as usize].i_igf_num_tile as i32
    );

    &mut h_private_static_data.f_spectrum_scale[tile_idx as usize]
}

/// `iisIGFDecLibGetNumberOfTiles(hPrivateStaticData, window_sequence)`
pub fn iis_igf_dec_lib_get_number_of_tiles(
    h_private_static_data: &IgfPrivateStaticData,
    window_sequence: i32,
) -> i32 {
    h_private_static_data.s_grid_info_tab[window_sequence as usize].i_igf_num_tile as i32
}

/********************************************************
   IGF TNF functions
*********************************************************/
const TNF_MAX_FILTER_ORDER: usize = 16;
const TNF_MAX_SUBDIVISIONS: usize = 3;
const TNF_MAX_SUBDIVISIONS_EXP: i32 = 2;

/// `DotProduct()`: energy of `input[..length]` with exponent.
fn dot_product(input: &[FixpDbl], input_exp: i32, length: i32) -> (FixpDbl, i32) {
    let n = length.max(0) as usize;
    let shift = get_scalefactor(&input[..n]);

    let loop_shift = DFRACT_BITS - f_normz(length);

    let mut acc: FixpDbl = 0;
    for &v in &input[..n] {
        let temp = f_pow2_div2(v << shift);
        acc = acc.wrapping_add(temp >> loop_shift);
    }
    (acc, ((input_exp - shift) << 1) + 1 + loop_shift)
}

/// `Same_Exponent_Correlation(X, Y, Exp, signal_length, correlation_length)` with `Y = X + lag`.
fn same_exponent_correlation(
    x: &[FixpDbl],
    lag: usize,
    exp: &mut i32,
    signal_length: i32,
    correlation_length: i32,
) -> FixpDbl {
    let shift = get_scalefactor(&x[..signal_length.max(0) as usize]);

    /* Find a suitable shift for dmx addition */
    let mut loop_shift = 31 - fixnorm_d(correlation_length);

    if loop_shift >= 1 {
        loop_shift -= 1;
    }

    let mut acc: FixpDbl = 0;
    for i in 0..correlation_length.max(0) as usize {
        let temp = f_mult_div2(x[i] << shift, x[lag + i] << shift);
        acc = acc.wrapping_add(temp >> loop_shift);
    }
    *exp = ((*exp - shift) << 1) + 1 + loop_shift;

    acc
}

/*************************************************************************
static data for this module
*************************************************************************/
/* 0.997803f,   0.991211f,  0.980225f,  0.964844f,  0.945068f,  0.920898f,  0.892334f, 0.859375f */
/* In Q0.31 */
/* Generated with scripts/codegen/dump_table.py from libIGFdec/src/FDK_igfDec.cpp. */
static TNS_ACF_WINDOW: [FixpDbl; 8] = [
    0x7FB80200, 0x7EE00080, 0x7D780380, 0x7B800200, 0x78F7FD00, 0x75DFFC80, 0x72380000, 0x6E000000,
];

/// `Autocorrelation(x, x_exp, n, lag)`: correlation of `x[..n - lag]` with `x[lag..n]`.
pub fn autocorrelation(x: &[FixpDbl], x_exp: &mut i32, n: i32, lag: i32) -> FixpDbl {
    same_exponent_correlation(x, lag as usize, x_exp, n, n - lag)
}

/// `iisIGF_TNFdetect()`: `aarray` is `Aarray + 1`.
fn iis_igf_tnf_detect(
    p2_spectrum: &[FixpDbl],
    spec_scale: &[i16],
    igf_bgn: i16,
    igf_end: i16,
    max_order: u8,
    aarray: &mut [FixpDbl],
    aarray_exp: &mut i32,
    curr_order: &mut u8,
) {
    let mut norms = [0 as FixpDbl; TNF_MAX_SUBDIVISIONS];
    let mut norms_exp = [0i32; TNF_MAX_SUBDIVISIONS];
    let n_subdivisions = TNF_MAX_SUBDIVISIONS;
    let mut lines = [0i32; 4];
    let mut line_diffs = [0i32; 3];

    /* Calculate line spacing */
    /* iStartLine = igfBgn + (igfEnd-igfBgn)*iSubdivisions/nSubdivisions;
    int const iEndLine = igfBgn + (igfEnd-igfBgn)*(iSubdivisions+1)/nSubdivisions; */

    /* 1/3 in Q0.31 = 0x2aaaaaab; 2/3 in Q0.31 = 0x55555555 */
    const ONE_THIRD: FixpDbl = 0x2aaaaaab;

    let span: FixpSgl = (igf_end as i32 - igf_bgn as i32) as i16;
    lines[0] = igf_bgn as i32; /* iSubdivisions =0 */
    let mut comp = f_mult(ONE_THIRD, span) >> 16;
    lines[1] = lines[0] + comp;
    comp = f_mult(ONE_THIRD, ((span as i32) << 1) as i16) >> 16;
    lines[2] = lines[0] + comp;
    lines[3] = igf_end as i32; /* iSubdivisions +1 = 3 */

    line_diffs[0] = lines[1] - lines[0];
    line_diffs[1] = lines[2] - lines[1];
    line_diffs[2] = lines[3] - lines[2];

    /* Calculate norms for each spectrum part */
    for i in 0..n_subdivisions {
        let (n, e) =
            dot_product(&p2_spectrum[lines[i] as usize..], spec_scale[0] as i32, line_diffs[i]);
        norms[i] = n;
        norms_exp[i] = e;
    }

    /* If a norm is lower than threshold, return */
    let mut exit_flag = false;
    for i in 0..n_subdivisions {
        /* Minimum value is 1 in Q3.28: 0.0000000037252902984619140625 */
        exit_flag = f_is_less_than(norms[i], norms_exp[i], 1, 3);
        if exit_flag {
            break;
        }
    }
    /* Return if the condition is met*/
    if exit_flag {
        return;
    }

    /* Calculate normalized autocorrelation for spectrum subdivision */
    let mut rxx = [[0 as FixpDbl; TNF_MAX_FILTER_ORDER + 1]; TNF_MAX_SUBDIVISIONS];
    let mut rxx_exp = [[0i32; TNF_MAX_FILTER_ORDER + 1]; TNF_MAX_SUBDIVISIONS];
    let spectrum_length = igf_end as i32 - igf_bgn as i32;
    let max_order = max_order as usize;

    for i in 0..n_subdivisions {
        let norm_value = norms[i];
        let mut fac_exp = norms_exp[i];

        let fac = inv_fixp_e(norm_value, &mut fac_exp);

        for lag in 1..=max_order {
            let temp = f_mult(fac, TNS_ACF_WINDOW[lag - 1]);

            let mut autocorr_exp = spec_scale[0] as i32;
            let autocorr = autocorrelation(
                &p2_spectrum[lines[i] as usize..],
                &mut autocorr_exp,
                line_diffs[i],
                lag as i32,
            );

            rxx[i][lag] = f_mult(temp, autocorr);
            rxx_exp[i][lag] = autocorr_exp + fac_exp;

            if rxx[i][lag] == 0 {
                rxx_exp[i][lag] = -1000;
            }
        } /* for(INT lag=1; */
    } /* for(iSubdivisions=0 ...*/

    /* Make the exponents for different lag values equal and add the results */
    let mut store_values = [0 as FixpDbl; 9];
    let mut max_exp = -1000;
    let mut min_rxx_headroom = 31;
    for lag in 1..=max_order {
        /* Find the maximum exponent */
        for i in 0..n_subdivisions {
            if max_exp < rxx_exp[i][lag] {
                max_exp = rxx_exp[i][lag];
            }
            let rxx_headroom = fixnorm_d(rxx[i][lag]);
            if rxx[i][lag] != 0 && rxx_headroom < min_rxx_headroom {
                min_rxx_headroom = rxx_headroom;
            }
        }
    }

    max_exp += 2;

    for lag in 1..=max_order {
        /* Shift and add values and store the exponent */
        store_values[lag] = 0;
        for i in 0..n_subdivisions {
            rxx[i][lag] <<= min_rxx_headroom;
            let shift = f_min(31, max_exp - rxx_exp[i][lag]);
            rxx[i][lag] = rxx[i][lag].wrapping_shr(shift as u32);
            store_values[lag] = store_values[lag].wrapping_add(rxx[i][lag]);
        }
    }

    let mut store_values_exp = max_exp - min_rxx_headroom;

    /* Allign maximum left for high precision */
    let shift_st_v = get_scalefactor(&store_values[1..=max_order]);
    for v in store_values[1..=max_order].iter_mut() {
        *v <<= shift_st_v;
    }
    store_values_exp -= shift_st_v;

    /* Adjust exponent of the rest of the values to fit the maximum (TNF_MAX_SUBDIVISIONS_EXP) */
    let mut diff = TNF_MAX_SUBDIVISIONS_EXP - store_values_exp;
    diff = f_min(31, diff);

    if diff > 0 {
        for v in store_values[1..=max_order].iter_mut() {
            *v >>= diff;
        }
        store_values_exp = TNF_MAX_SUBDIVISIONS_EXP;
    }

    /* Assign value and scale to fit the rest of the values */
    store_values[0] = scale_value(n_subdivisions as FixpDbl, 31 - store_values_exp);

    let filter_order = f_min(max_order as i32, spectrum_length >> 2);
    clpc_auto_to_lpc_igf(aarray, aarray_exp, &store_values, store_values_exp, filter_order, None);

    *curr_order = max_order as u8;
}

/// `iisIGF_TNFfilter()`
fn iis_igf_tnf_filter(
    p2_spectrum: &mut [FixpDbl],
    _spec_scale: i16,
    num_of_lines: i16,
    filter_order: u8,
    aarray: &[FixpDbl],
    aarray_exp: i32,
) {
    let num_of_lines = num_of_lines as i32;
    /* Find the available headroom of the downmix signal and a suitable shift value*/
    let mut head_shift = get_scalefactor(&p2_spectrum[..num_of_lines.max(0) as usize]);

    if head_shift == 31 {
        head_shift = 0;
    }

    let output_shift = (aarray_exp - head_shift + 2).clamp(-31, 31);

    debug_assert!(filter_order == 8);
    let filter_order = filter_order as i32;

    /* First part is running backwards with full filter length */
    let mut j = num_of_lines - 1;
    while j >= filter_order - 1 {
        let mut acc: FixpDbl = 0;
        for i in 0..filter_order {
            let tmp = f_mult(p2_spectrum[(j - i) as usize] << head_shift, aarray[i as usize]);
            acc = acc.wrapping_add(tmp >> 2);
        }
        p2_spectrum[j as usize] = scale_value(acc, output_shift);
        j -= 1;
    }

    /* second part is running with decreasing filter length */
    while j >= 0 {
        let mut acc: FixpDbl = 0;
        for i in 0..=j {
            let tmp = f_mult(p2_spectrum[(j - i) as usize] << head_shift, aarray[i as usize]);
            acc = acc.wrapping_add(tmp >> 2);
        }
        p2_spectrum[j as usize] = scale_value(acc, output_shift);
        j -= 1;
    }
}

/// `CIgf_TNF_apply(hPrivateStaticData, hPrivateData, p2_spectrum, specScale, TNF_mask,
/// window_sequence, frameType)`: temporal noise flattening of the IGF range, triggered by
/// `igfUseEnfFlat`.
///
/// `virtual_spec` is the `virtualSpec` of `hPrivateData->IGF_Common_channel_data_handle` (filled
/// by [`cigf_apply`] / [`cigf_apply_stereo`]); the lines with `tnf_mask[i] == 0` are copied from it
/// into `p2_spectrum` after filtering.
pub fn cigf_tnf_apply(
    h_private_static_data: &IgfPrivateStaticData,
    h_private_data: &IgfPrivateData,
    virtual_spec: &mut [FixpDbl],
    p2_spectrum: &mut [FixpDbl],
    spec_scale: &[i16],
    tnf_mask: &[u8],
    window_sequence: i32,
    frame_type: i32,
) {
    /* Start only if triggered */
    if h_private_data.bitstream_data[frame_type as usize].igf_use_enf_flat != 0 {
        let mut aarray = [0 as FixpDbl; TNF_MAX_FILTER_ORDER + 1];
        let mut aarray_exp = 0;
        let h_grid = &h_private_static_data.s_grid_info_tab[window_sequence as usize];
        let igf_bgn = h_grid.i_igf_start_line;
        let igf_end = h_grid.i_igf_stop_line;
        const MAX_ORDER: u8 = 8;
        let mut curr_order = 0u8;

        /* "virtualSpec" contains a modified IGF spectrum values.  Typically IGF fills the zero-gaps
        with values from the base spectrum and leaves the non-zero values intact. In this spectrum
        however all values are filled from the lower spectrum. Just later (at the end of the
        algorithm) the non-zero values are put back in the spectrum. */

        iis_igf_tnf_detect(
            virtual_spec,
            spec_scale,
            igf_bgn,
            igf_end,
            MAX_ORDER,
            &mut aarray[1..],
            &mut aarray_exp,
            &mut curr_order,
        );

        if curr_order > 0 {
            /* Assign 1 to the first value of the array */
            aarray[0] = 0x7fffffff;
            if aarray_exp > 0 {
                aarray[0] = 1i32.wrapping_shl((31 - aarray_exp) as u32);
            }

            iis_igf_tnf_filter(
                &mut virtual_spec[igf_bgn as usize..],
                spec_scale[0],
                (igf_end as i32 - igf_bgn as i32) as i16,
                curr_order,
                &aarray,
                aarray_exp,
            );
        }

        /* We make sure all values created by IGF (which were zero before IGF) are added to the
        non-zero values of the spectrum */
        if igf_bgn < igf_end {
            let r = igf_bgn as usize..igf_end as usize;
            for ((x, &v), &m) in
                p2_spectrum[r.clone()].iter_mut().zip(&virtual_spec[r.clone()]).zip(&tnf_mask[r])
            {
                if m == 0 {
                    *x = v;
                }
            }
        }
    } /* if(hPrivateData->igfUseEnfFlat) */
}
