// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_cicp2geometry.h and libFDK/src/FDK_cicp2geometry.cpp.
//
// Notice of the C++ source:
//
// This software module was originally developed by
//
// Fraunhofer IIS
//
// in the course of development of the ISO/IEC 23008-3 for reference purposes and its
// performance may not have been optimized. This software module is an implementation
// of one or more tools as specified by the ISO/IEC 23008-3 standard. ISO/IEC gives
// you a royalty-free, worldwide, non-exclusive, copyright license to copy, distribute,
// and make derivative works of this software module or modifications  thereof for use
// in implementations or products claiming conformance to the ISO/IEC 23008-3 standard
// and which satisfy any specified conformance criteria. Those intending to use this
// software module in products are advised that its use may infringe existing patents.
// ISO/IEC have no liability for use of this software module or modifications thereof.
// Copyright is not released for products that do not conform to the ISO/IEC 23008-3
// standard.
//
// Fraunhofer IIS retains full right to modify and use the code for its own purpose,
// assign or donate the code to a third party and to inhibit third parties from using
// the code for products that do not conform to MPEG-related ITU Recommendations and/or
// ISO/IEC International Standards.
//
// This copyright notice must be included in all copies or derivative works.
//
// Copyright (c) ISO/IEC 2013.

//! Conversion of CICP loudspeaker layout indices to a geometric description of the reproduction
//! setup and (tries) vice versa.

use crate::digest::StateDigest;

use crate::common_fix::{fl2fxconst_dbl, FixpDbl, MAXVAL_DBL};

/// `CICP2GEOMETRY_MAX_LOUDSPEAKERS`
pub const CICP2GEOMETRY_MAX_LOUDSPEAKERS: usize = 32;

/// `CICP2GEOMETRY_ERROR`
pub type Cicp2GeometryError = i32;
pub const CICP2GEOMETRY_OK: Cicp2GeometryError = 0;
pub const CICP2GEOMETRY_INIT_ERROR: Cicp2GeometryError = -999;
pub const CICP2GEOMETRY_CONFIG_ERROR: Cicp2GeometryError = -998;
pub const CICP2GEOMETRY_BAD_POINTER: Cicp2GeometryError = -997;
pub const CICP2GEOMETRY_ALLOC_ERROR: Cicp2GeometryError = -996;
pub const CICP2GEOMETRY_INVALID_AZIMITH: Cicp2GeometryError = -995;
pub const CICP2GEOMETRY_INVALID_ELEVATION: Cicp2GeometryError = -994;
pub const CICP2GEOMETRY_INVALID_LFE_FLAG: Cicp2GeometryError = -993;
pub const CICP2GEOMETRY_INVALID_SCREEN_RELATIVE_FLAG: Cicp2GeometryError = -992;
pub const CICP2GEOMETRY_UNKNOWN_SPEAKER_INDEX: Cicp2GeometryError = -991;
pub const CICP2GEOMETRY_INVALID_CICP_INDEX: Cicp2GeometryError = -990;
pub const CICP2GEOMETRY_UNSUPPORTED_CHANNEL_COUNT: Cicp2GeometryError = -989;
pub const CICP2GEOMETRY_INVALID_GEO_FILE_SYNTAX: Cicp2GeometryError = -988;
pub const CICP2GEOMETRY_INVALID_FILEHANDLE: Cicp2GeometryError = -987;

/// `CICP2GEOMETRY_CICP_INVALID`
pub const CICP2GEOMETRY_CICP_INVALID: i32 = -1000;
/// `CICP2GEOMETRY_CICP_VALID`
pub const CICP2GEOMETRY_CICP_VALID: i32 = 0;

/// `CICP2GEOMETRY_LOUDSPEAKER_TYPE`
pub type Cicp2GeometryLoudspeakerType = i32;
/// Known CICP loudspeaker
pub const CICP2GEOMETRY_LOUDSPEAKER_KNOWN: Cicp2GeometryLoudspeakerType = 1;
/// Unknown CICP loudspeaker
pub const CICP2GEOMETRY_LOUDSPEAKER_UNKNOWN: Cicp2GeometryLoudspeakerType = 0;
/// Invalid index for a CICP loudspeaker
pub const CICP2GEOMETRY_LOUDSPEAKER_INVALID: Cicp2GeometryLoudspeakerType = -1;

/// `CICP2GEOMETRAY_CICP_SPEAKER_INDEX_LABEL` (file-local enum of the C++ source)
pub const CH_M_L030: i32 = 0;
pub const CH_M_R030: i32 = 1;
pub const CH_M_000: i32 = 2;
pub const CH_LFE1: i32 = 3;
pub const CH_M_L110: i32 = 4;
pub const CH_M_R110: i32 = 5;
pub const CH_M_L022: i32 = 6;
pub const CH_M_R022: i32 = 7;
pub const CH_M_L135: i32 = 8;
pub const CH_M_R135: i32 = 9;
pub const CH_M_180: i32 = 10;
pub const CH_RES1: i32 = 11;
pub const CH_RES2: i32 = 12;
pub const CH_M_L090: i32 = 13;
pub const CH_M_R090: i32 = 14;
pub const CH_M_L060: i32 = 15;
pub const CH_M_R060: i32 = 16;
pub const CH_U_L030: i32 = 17;
pub const CH_U_R030: i32 = 18;
pub const CH_U_000: i32 = 19;
pub const CH_U_L135: i32 = 20;
pub const CH_U_R135: i32 = 21;
pub const CH_U_180: i32 = 22;
pub const CH_U_L090: i32 = 23;
pub const CH_U_R090: i32 = 24;
pub const CH_T_000: i32 = 25;
pub const CH_LFE2: i32 = 26;
pub const CH_L_L045: i32 = 27;
pub const CH_L_R045: i32 = 28;
pub const CH_L_000: i32 = 29;
pub const CH_U_L110: i32 = 30;
pub const CH_U_R110: i32 = 31;
pub const CH_U_L045: i32 = 32;
pub const CH_U_R045: i32 = 33;
pub const CH_M_L045: i32 = 34;
pub const CH_M_R045: i32 = 35;
pub const CH_LFE3: i32 = 36;
pub const CH_M_LSCR: i32 = 37;
pub const CH_M_RSCR: i32 = 38;
pub const CH_M_LSCH: i32 = 39;
pub const CH_M_RSCH: i32 = 40;
pub const CH_M_L150: i32 = 41;
pub const CH_M_R150: i32 = 42;
// Non standard positions
pub const CH_B_000: i32 = 128;
pub const CH_B_L030: i32 = 129;
pub const CH_B_R030: i32 = 130;
pub const CH_SU_000: i32 = 131;
pub const CH_SU_L030: i32 = 132;
pub const CH_SU_R030: i32 = 133;
pub const CH_SU_L110: i32 = 134;
pub const CH_SU_R110: i32 = 135;
pub const CH_SU_L078: i32 = 136;
pub const CH_SU_R078: i32 = 137;
pub const CH_M_L120: i32 = 138;
pub const CH_M_R120: i32 = 139;
pub const CH_B_L110: i32 = 140;
pub const CH_B_R110: i32 = 141;

/// `CICP2GEOMETRY_CHANNEL_GEOMETRY`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Cicp2GeometryChannelGeometry {
    /// Index specifying cicp loudspeaker. -1 if only geometry is given.
    pub cicp_loudspeaker_index: i32,
    /// Azimuth angle of the channel (`Az`).
    pub az: i32,
    /// Elevation angle of the channel (`El`).
    pub el: i32,
    /// Flag whether channel is an LFE (`LFE`).
    pub lfe: i32,
    /// Flag whether loudspeaker position is screen relative.
    pub screen_relative: i32,
    /// Distance info available for loudspeaker.
    pub has_distance: i32,
    /// Distance value in cm.
    pub distance: u32,
    /// Calibration info available for loudspeaker.
    pub has_loudspeaker_calibration_gain: i32,
    /// Linear calibration gain.
    pub loudspeaker_calibration_gain: i32,
    /// Enum describing the loudspeaker type.
    pub loudspeaker_type: Cicp2GeometryLoudspeakerType,
}

/// `CICP2GEOMETRY_CICP_LAYOUT` (file-local in the C++ source)
#[derive(Clone, Copy, Debug)]
struct Cicp2GeometryCicpLayout {
    /// CICP speaker layout index
    index: i16,
    /// amount of speakers
    number_of_loudspeakers: u8,
    /// list of speaker CICP indices
    speaker_id: [u8; CICP2GEOMETRY_MAX_LOUDSPEAKERS],
}

const fn g(
    idx: i32,
    az: i32,
    el: i32,
    lfe: i32,
    screen_relative: i32,
    has_distance: i32,
    distance: u32,
    has_gain: i32,
    gain: i32,
    loudspeaker_type: i32,
) -> Cicp2GeometryChannelGeometry {
    Cicp2GeometryChannelGeometry {
        cicp_loudspeaker_index: idx,
        az,
        el,
        lfe,
        screen_relative,
        has_distance,
        distance,
        has_loudspeaker_calibration_gain: has_gain,
        loudspeaker_calibration_gain: gain,
        loudspeaker_type,
    }
}

const fn l(index: i16, n: u8, ids: &[u8]) -> Cicp2GeometryCicpLayout {
    let mut speaker_id = [0u8; CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    let mut i = 0;
    while i < ids.len() {
        speaker_id[i] = ids[i];
        i += 1;
    }
    Cicp2GeometryCicpLayout { index, number_of_loudspeakers: n, speaker_id }
}

/// `CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE_NON_STD_GAP`
const CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE_NON_STD_GAP: i32 = 128 - 42 - 1;

fn match_loudspeakers(
    tmp_az_el_lfe: &Cicp2GeometryChannelGeometry,
    az_el_lfe: &Cicp2GeometryChannelGeometry,
) -> i32 {
    let mut error = (tmp_az_el_lfe.az != az_el_lfe.az) as i32;
    error |= (tmp_az_el_lfe.el != az_el_lfe.el) as i32;
    error |= (tmp_az_el_lfe.lfe != az_el_lfe.lfe) as i32;
    error |= (tmp_az_el_lfe.screen_relative != az_el_lfe.screen_relative) as i32;
    error
}

fn match_loudspeaker_setups(
    az_el_lfe: &[Cicp2GeometryChannelGeometry],
    tmp_az_el_lfe: &[Cicp2GeometryChannelGeometry; CICP2GEOMETRY_MAX_LOUDSPEAKERS],
    num_channels: i32,
    tmp_num_channels: i32,
    num_lfes: i32,
    tmp_num_lfes: i32,
    num_total_channels: i32,
) -> i32 {
    let mut error = 0;

    if num_channels != tmp_num_channels || num_lfes != tmp_num_lfes {
        return 1;
    }

    for ch in 0..num_total_channels as usize {
        error = match_loudspeakers(&tmp_az_el_lfe[ch], &az_el_lfe[ch]);
        if error != 0 {
            break;
        }
    }

    error
}

/// `cicp2geometry_get_cicpIndex_from_geometry(AzElLfe, numChannels, numLFEs, cicpIndex)`:
/// matches the given geometry against the known CICP loudspeaker layouts.
pub fn cicp2geometry_get_cicp_index_from_geometry(
    az_el_lfe: &[Cicp2GeometryChannelGeometry],
    num_channels: i32,
    num_lfes: i32,
    cicp_index: &mut i32,
) -> Cicp2GeometryError {
    let num_total_channels = num_channels + num_lfes;

    *cicp_index = CICP2GEOMETRY_CICP_INVALID;

    for layout in CICP_SPEAKER_LAYOUTS.iter() {
        if num_total_channels == layout.number_of_loudspeakers as i32 {
            let mut tmp_az_el_lfe =
                [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
            let mut tmp_num_channels = 0;
            let mut tmp_num_lfes = 0;

            cicp2geometry_get_geometry_from_cicp(
                layout.index as i32,
                Some(&mut tmp_az_el_lfe),
                &mut tmp_num_channels,
                &mut tmp_num_lfes,
            );
            let error = match_loudspeaker_setups(
                az_el_lfe,
                &tmp_az_el_lfe,
                num_channels,
                tmp_num_channels,
                num_lfes,
                tmp_num_lfes,
                num_total_channels,
            );
            if error == 0 {
                *cicp_index = layout.index as i32;
                return CICP2GEOMETRY_OK;
            }
        }
    }

    CICP2GEOMETRY_UNSUPPORTED_CHANNEL_COUNT
}

/// `cicp2geometry_get_geometry_from_cicp(cicpIndex, AzElLfe, numChannels, numLFEs)`: geometry of
/// each channel of the CICP layout `cicp_index` (`az_el_lfe = None` for a `NULL` pointer), the
/// number of channels without LFEs and the number of LFEs.
pub fn cicp2geometry_get_geometry_from_cicp(
    cicp_index: i32,
    mut az_el_lfe: Option<&mut [Cicp2GeometryChannelGeometry; CICP2GEOMETRY_MAX_LOUDSPEAKERS]>,
    num_channels: &mut i32,
    num_lfes: &mut i32,
) -> Cicp2GeometryError {
    for layout in CICP_SPEAKER_LAYOUTS.iter() {
        if layout.index as i32 == cicp_index {
            let mut tmp_lfe = 0;

            for ii in 0..layout.number_of_loudspeakers as usize {
                let speaker_index = layout.speaker_id[ii] as i32;
                let table_index = if speaker_index < 128 {
                    speaker_index
                } else {
                    speaker_index - CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE_NON_STD_GAP
                } as usize;
                if let Some(a) = az_el_lfe.as_deref_mut() {
                    a[ii] = CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE[table_index];
                }
                if CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE[table_index].lfe != 0 {
                    tmp_lfe += 1;
                }
            }
            *num_channels = layout.number_of_loudspeakers as i32 - tmp_lfe;
            *num_lfes = tmp_lfe;

            return CICP2GEOMETRY_OK;
        }
    }

    *num_channels = -1;
    *num_lfes = -1;

    CICP2GEOMETRY_INVALID_CICP_INDEX
}

/// `cicp2geometry_get_numChannels_from_cicp(cicpIndex, numTotalChannels)`: total number of
/// channels (including LFEs) of a CICP layout.
pub fn cicp2geometry_get_num_channels_from_cicp(
    cicp_index: i32,
    num_total_channels: &mut i32,
) -> Cicp2GeometryError {
    let mut num_channels = 0;
    let mut num_lfes = 0;

    let cicp_error =
        cicp2geometry_get_geometry_from_cicp(cicp_index, None, &mut num_channels, &mut num_lfes);

    *num_total_channels = num_channels + num_lfes;

    cicp_error
}

/// `cicp2geometry_get_numChannels_from_cicp(cicpIndex)` (inline overload without error code)
pub fn cicp2geometry_get_num_channels_from_cicp_noerr(cicp_index: i32) -> i32 {
    let mut n = 0;
    cicp2geometry_get_num_channels_from_cicp(cicp_index, &mut n);
    n
}

/// `cicp2geometry_get_geometry_from_cicp_loudspeaker_index(cicpLoudspeakerIndex, AzElLfe)`
/// (`az_el_lfe = None` for a `NULL` pointer).
pub fn cicp2geometry_get_geometry_from_cicp_loudspeaker_index(
    cicp_loudspeaker_index: i32,
    az_el_lfe: Option<&mut Cicp2GeometryChannelGeometry>,
) -> Cicp2GeometryError {
    let mut cicp_error = CICP2GEOMETRY_OK;

    let Some(az_el_lfe) = az_el_lfe else {
        return CICP2GEOMETRY_BAD_POINTER;
    };

    if cicp_loudspeaker_index >= 0
        && cicp_loudspeaker_index < CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE.len() as i32
    {
        *az_el_lfe = CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE[cicp_loudspeaker_index as usize];

        if az_el_lfe.loudspeaker_type == CICP2GEOMETRY_LOUDSPEAKER_INVALID {
            cicp_error = CICP2GEOMETRY_UNKNOWN_SPEAKER_INDEX;
        }
    } else {
        // save the cicp loudspeaker even if it's not known
        az_el_lfe.cicp_loudspeaker_index = cicp_loudspeaker_index;

        az_el_lfe.az = -1;
        az_el_lfe.el = -1;
        az_el_lfe.lfe = -1;
        az_el_lfe.screen_relative = -1;
        az_el_lfe.has_distance = 0;
        az_el_lfe.distance = 0;
        az_el_lfe.has_loudspeaker_calibration_gain = 0;
        az_el_lfe.loudspeaker_calibration_gain = 0;
        az_el_lfe.loudspeaker_type = CICP2GEOMETRY_LOUDSPEAKER_UNKNOWN;
        cicp_error = CICP2GEOMETRY_UNKNOWN_SPEAKER_INDEX;
    }

    cicp_error
}

/// `cicp2geometry_get_number_of_lfes(AzElLfe, numChannels, numLfes)`
pub fn cicp2geometry_get_number_of_lfes(
    az_el_lfe: &[Cicp2GeometryChannelGeometry],
    num_channels: u32,
    num_lfes: &mut u32,
) -> Cicp2GeometryError {
    let mut lfe_count = 0u32;

    for a in &az_el_lfe[..num_channels as usize] {
        if a.lfe != 0 {
            // FDK_ASSERT(AzElLfe[i].LFE != -1): avoid counting negative marked LFE's (no-op)
            lfe_count += 1;
        }
    }

    *num_lfes = lfe_count;
    CICP2GEOMETRY_OK
}

/// `cicp2geometry_get_front_speakers(targetLayout, numSpeakers, numSignalsMix, speakerPosIndices,
/// speakerGains)`: obtain indices and gains of the target layout speakers suitable to mix in
/// `num_speakers` (0..=2) front speaker signals.
pub fn cicp2geometry_get_front_speakers(
    target_layout: i32,
    num_speakers: i32,
    num_signals_mix: &mut i32,
    speaker_pos_indices: &mut [i32],
    speaker_gains: &mut [FixpDbl],
) -> Cicp2GeometryError {
    let speakers_to_look_for: [[i32; 2]; 2] = [[CH_M_000, CH_M_000], [CH_M_L030, CH_M_R030]];
    let mut geo_tl = [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    let mut num_tl_speakers = 0;
    let mut num_tl_lfe = 0;
    let mut found = 0;

    if !(0..=2).contains(&num_speakers) {
        return CICP2GEOMETRY_UNSUPPORTED_CHANNEL_COUNT;
    }

    let error = cicp2geometry_get_geometry_from_cicp(
        target_layout,
        Some(&mut geo_tl),
        &mut num_tl_speakers,
        &mut num_tl_lfe,
    );
    if error != CICP2GEOMETRY_OK {
        return error;
    }
    num_tl_speakers += num_tl_lfe;

    // Assume direct match
    *num_signals_mix = num_speakers;
    for g in speaker_gains.iter_mut().take(num_speakers as usize) {
        *g = MAXVAL_DBL;
    }

    for i in 0..num_speakers as usize {
        for t in 0..num_tl_speakers as usize {
            if geo_tl[t].cicp_loudspeaker_index
                == speakers_to_look_for[num_speakers as usize - 1][i]
            {
                speaker_pos_indices[i] = t as i32;
                found += 1;
                break;
            }
        }
    }

    // If direct match not possible, try switching center / stereo pair
    if found != num_speakers {
        found = 0;
        let num_speakers_match = if num_speakers == 1 { 2 } else { 1 };

        for i in 0..num_speakers_match as usize {
            speaker_gains[i] = fl2fxconst_dbl(0.707);

            for t in 0..num_tl_speakers as usize {
                if geo_tl[t].cicp_loudspeaker_index
                    == speakers_to_look_for[num_speakers_match as usize - 1][i]
                {
                    speaker_pos_indices[i] = t as i32;
                    found += 1;
                    break;
                }
            }
        }
        if num_speakers_match < num_speakers {
            speaker_pos_indices[1] = speaker_pos_indices[0];
        }

        if found == 0 {
            return CICP2GEOMETRY_CONFIG_ERROR;
        }

        *num_signals_mix = num_speakers_match;
    }

    CICP2GEOMETRY_OK
}

// Tables generated with scripts/codegen/dump_table.py from libFDK/src/FDK_cicp2geometry.cpp.

/// `CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE`: CICP loudspeaker indices 0..=42 followed by the non
/// standard positions 128..=141.
pub static CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLE: [Cicp2GeometryChannelGeometry; 57] = [
    g(0, 30, 0, 0, 0, 0, 0, 0, 0, 1),
    g(1, -30, 0, 0, 0, 0, 0, 0, 0, 1),
    g(2, 0, 0, 0, 0, 0, 0, 0, 0, 1),
    g(3, 0, -15, 1, 0, 0, 0, 0, 0, 1),
    g(4, 110, 0, 0, 0, 0, 0, 0, 0, 1),
    g(5, -110, 0, 0, 0, 0, 0, 0, 0, 1),
    g(6, 22, 0, 0, 0, 0, 0, 0, 0, 1),
    g(7, -22, 0, 0, 0, 0, 0, 0, 0, 1),
    g(8, 135, 0, 0, 0, 0, 0, 0, 0, 1),
    g(9, -135, 0, 0, 0, 0, 0, 0, 0, 1),
    g(10, 180, 0, 0, 0, 0, 0, 0, 0, 1),
    g(11, -1, -1, -1, -1, 0, 0, 0, 0, -1),
    g(12, -1, -1, -1, -1, 0, 0, 0, 0, -1),
    g(13, 90, 0, 0, 0, 0, 0, 0, 0, 1),
    g(14, -90, 0, 0, 0, 0, 0, 0, 0, 1),
    g(15, 60, 0, 0, 0, 0, 0, 0, 0, 1),
    g(16, -60, 0, 0, 0, 0, 0, 0, 0, 1),
    g(17, 30, 35, 0, 0, 0, 0, 0, 0, 1),
    g(18, -30, 35, 0, 0, 0, 0, 0, 0, 1),
    g(19, 0, 35, 0, 0, 0, 0, 0, 0, 1),
    g(20, 135, 35, 0, 0, 0, 0, 0, 0, 1),
    g(21, -135, 35, 0, 0, 0, 0, 0, 0, 1),
    g(22, 180, 35, 0, 0, 0, 0, 0, 0, 1),
    g(23, 90, 35, 0, 0, 0, 0, 0, 0, 1),
    g(24, -90, 35, 0, 0, 0, 0, 0, 0, 1),
    g(25, 0, 90, 0, 0, 0, 0, 0, 0, 1),
    g(26, 45, -15, 1, 0, 0, 0, 0, 0, 1),
    g(27, 45, -15, 0, 0, 0, 0, 0, 0, 1),
    g(28, -45, -15, 0, 0, 0, 0, 0, 0, 1),
    g(29, 0, -15, 0, 0, 0, 0, 0, 0, 1),
    g(30, 110, 35, 0, 0, 0, 0, 0, 0, 1),
    g(31, -110, 35, 0, 0, 0, 0, 0, 0, 1),
    g(32, 45, 35, 0, 0, 0, 0, 0, 0, 1),
    g(33, -45, 35, 0, 0, 0, 0, 0, 0, 1),
    g(34, 45, 0, 0, 0, 0, 0, 0, 0, 1),
    g(35, -45, 0, 0, 0, 0, 0, 0, 0, 1),
    g(36, -45, -15, 1, 0, 0, 0, 0, 0, 1),
    g(37, 60, 0, 0, 1, 0, 0, 0, 0, 1),
    g(38, -60, 0, 0, 1, 0, 0, 0, 0, 1),
    g(39, 30, 0, 0, 1, 0, 0, 0, 0, 1),
    g(40, -30, 0, 0, 1, 0, 0, 0, 0, 1),
    g(41, 150, 0, 0, 0, 0, 0, 0, 0, 1),
    g(42, -150, 0, 0, 0, 0, 0, 0, 0, 1),
    g(128, 0, -20, 0, 0, 0, 0, 0, 0, 1),
    g(129, 30, -20, 0, 0, 0, 0, 0, 0, 1),
    g(130, -30, -20, 0, 0, 0, 0, 0, 0, 1),
    g(131, 0, 30, 0, 0, 0, 0, 0, 0, 1),
    g(132, 30, 30, 0, 0, 0, 0, 0, 0, 1),
    g(133, -30, 30, 0, 0, 0, 0, 0, 0, 1),
    g(134, 110, 30, 0, 0, 0, 0, 0, 0, 1),
    g(135, -110, 30, 0, 0, 0, 0, 0, 0, 1),
    g(136, 78, 60, 0, 0, 0, 0, 0, 0, 1),
    g(137, -78, 60, 0, 0, 0, 0, 0, 0, 1),
    g(138, 120, 0, 0, 0, 0, 0, 0, 0, 1),
    g(139, -120, 0, 0, 0, 0, 0, 0, 0, 1),
    g(140, 110, -20, 0, 0, 0, 0, 0, 0, 1),
    g(141, -110, -20, 0, 0, 0, 0, 0, 0, 1),
];

/// `CICP_speakerLayouts`
static CICP_SPEAKER_LAYOUTS: [Cicp2GeometryCicpLayout; 296] = [
    l(1, 1, &[2]),
    l(2, 2, &[0, 1]),
    l(3, 3, &[0, 1, 2]),
    l(4, 4, &[0, 1, 2, 10]),
    l(5, 5, &[0, 1, 2, 4, 5]),
    l(6, 6, &[0, 1, 2, 3, 4, 5]),
    l(7, 8, &[0, 1, 2, 3, 4, 5, 15, 16]),
    l(9, 3, &[0, 1, 10]),
    l(10, 4, &[0, 1, 4, 5]),
    l(11, 7, &[0, 1, 2, 3, 4, 5, 10]),
    l(12, 8, &[0, 1, 2, 3, 4, 5, 8, 9]),
    l(
        13,
        24,
        &[
            15, 16, 2, 26, 8, 9, 0, 1, 10, 36, 13, 14, 32, 33, 19, 25, 20, 21, 23, 24, 22, 29, 27,
            28,
        ],
    ),
    l(14, 8, &[0, 1, 2, 3, 4, 5, 17, 18]),
    l(15, 12, &[0, 1, 2, 26, 8, 9, 36, 13, 14, 32, 33, 22]),
    l(16, 10, &[0, 1, 2, 3, 4, 5, 17, 18, 30, 31]),
    l(17, 12, &[0, 1, 2, 3, 4, 5, 17, 18, 19, 30, 31, 25]),
    l(18, 14, &[0, 1, 2, 3, 4, 5, 41, 42, 17, 18, 19, 30, 31, 25]),
    l(19, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 20, 21]),
    l(20, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 32, 33, 20, 21, 37, 38]),
    l(100, 10, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18]),
    l(101, 3, &[0, 1, 3]),
    l(102, 4, &[0, 1, 2, 3]),
    l(103, 5, &[0, 1, 3, 4, 5]),
    l(104, 6, &[0, 1, 2, 3, 17, 18]),
    l(105, 9, &[0, 1, 2, 8, 9, 13, 14, 17, 18]),
    l(106, 5, &[0, 1, 2, 17, 18]),
    l(107, 7, &[0, 1, 2, 4, 5, 15, 16]),
    l(108, 6, &[0, 1, 2, 4, 5, 10]),
    l(109, 7, &[0, 1, 2, 4, 5, 8, 9]),
    l(110, 7, &[0, 1, 2, 4, 5, 17, 18]),
    l(111, 9, &[0, 1, 2, 4, 5, 17, 18, 30, 31]),
    l(112, 11, &[0, 1, 2, 4, 5, 17, 18, 19, 25, 30, 31]),
    l(113, 13, &[0, 1, 2, 4, 5, 17, 18, 19, 25, 30, 31, 41, 42]),
    l(114, 11, &[0, 1, 2, 8, 9, 13, 14, 17, 18, 20, 21]),
    l(115, 13, &[0, 1, 2, 8, 9, 13, 14, 20, 21, 32, 33, 37, 38]),
    l(116, 5, &[0, 1, 3, 17, 18]),
    l(117, 5, &[0, 1, 3, 23, 24]),
    l(118, 5, &[0, 1, 3, 30, 31]),
    l(119, 6, &[0, 1, 2, 3, 23, 24]),
    l(120, 6, &[0, 1, 2, 3, 30, 31]),
    l(121, 7, &[0, 1, 3, 4, 5, 17, 18]),
    l(122, 7, &[0, 1, 3, 4, 5, 23, 24]),
    l(123, 7, &[0, 1, 3, 4, 5, 30, 31]),
    l(124, 9, &[0, 1, 3, 4, 5, 17, 18, 23, 24]),
    l(125, 9, &[0, 1, 3, 4, 5, 17, 18, 30, 31]),
    l(126, 9, &[0, 1, 3, 4, 5, 23, 24, 30, 31]),
    l(127, 10, &[0, 1, 3, 4, 5, 17, 18, 25, 30, 31]),
    l(128, 11, &[0, 1, 3, 4, 5, 17, 18, 23, 24, 30, 31]),
    l(129, 8, &[0, 1, 2, 3, 4, 5, 23, 24]),
    l(130, 8, &[0, 1, 2, 3, 4, 5, 30, 31]),
    l(131, 10, &[0, 1, 2, 3, 4, 5, 17, 18, 23, 24]),
    l(132, 10, &[0, 1, 2, 3, 4, 5, 23, 24, 30, 31]),
    l(133, 11, &[0, 1, 2, 3, 4, 5, 17, 18, 25, 30, 31]),
    l(134, 11, &[0, 1, 2, 3, 4, 5, 17, 18, 19, 30, 31]),
    l(135, 12, &[0, 1, 2, 3, 4, 5, 17, 18, 23, 24, 30, 31]),
    l(136, 5, &[0, 1, 3, 20, 21]),
    l(137, 6, &[0, 1, 2, 3, 20, 21]),
    l(138, 7, &[0, 1, 3, 4, 5, 20, 21]),
    l(139, 9, &[0, 1, 3, 4, 5, 17, 18, 20, 21]),
    l(140, 9, &[0, 1, 3, 4, 5, 20, 21, 23, 24]),
    l(141, 10, &[0, 1, 3, 4, 5, 17, 18, 20, 21, 25]),
    l(142, 11, &[0, 1, 3, 4, 5, 17, 18, 20, 21, 23, 24]),
    l(143, 8, &[0, 1, 2, 3, 4, 5, 20, 21]),
    l(144, 10, &[0, 1, 2, 3, 4, 5, 17, 18, 20, 21]),
    l(145, 10, &[0, 1, 2, 3, 4, 5, 20, 21, 23, 24]),
    l(146, 11, &[0, 1, 2, 3, 4, 5, 17, 18, 20, 21, 25]),
    l(147, 11, &[0, 1, 2, 3, 4, 5, 17, 18, 19, 20, 21]),
    l(148, 12, &[0, 1, 2, 3, 4, 5, 17, 18, 19, 20, 21, 25]),
    l(149, 12, &[0, 1, 2, 3, 4, 5, 17, 18, 20, 21, 23, 24]),
    l(150, 6, &[0, 1, 3, 4, 5, 10]),
    l(151, 8, &[0, 1, 3, 4, 5, 10, 17, 18]),
    l(152, 8, &[0, 1, 3, 4, 5, 10, 23, 24]),
    l(153, 8, &[0, 1, 3, 4, 5, 10, 30, 31]),
    l(154, 10, &[0, 1, 3, 4, 5, 10, 17, 18, 23, 24]),
    l(155, 10, &[0, 1, 3, 4, 5, 10, 17, 18, 30, 31]),
    l(156, 10, &[0, 1, 3, 4, 5, 10, 23, 24, 30, 31]),
    l(157, 11, &[0, 1, 3, 4, 5, 10, 17, 18, 25, 30, 31]),
    l(158, 12, &[0, 1, 3, 4, 5, 10, 17, 18, 23, 24, 30, 31]),
    l(159, 9, &[0, 1, 2, 3, 4, 5, 10, 17, 18]),
    l(160, 9, &[0, 1, 2, 3, 4, 5, 10, 23, 24]),
    l(161, 9, &[0, 1, 2, 3, 4, 5, 10, 30, 31]),
    l(162, 11, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 23, 24]),
    l(163, 11, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 30, 31]),
    l(164, 11, &[0, 1, 2, 3, 4, 5, 10, 23, 24, 30, 31]),
    l(165, 12, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 25, 30, 31]),
    l(166, 12, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 19, 30, 31]),
    l(167, 13, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 19, 25, 30, 31]),
    l(168, 13, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 23, 24, 30, 31]),
    l(169, 8, &[0, 1, 3, 4, 5, 10, 20, 21]),
    l(170, 10, &[0, 1, 3, 4, 5, 10, 17, 18, 20, 21]),
    l(171, 10, &[0, 1, 3, 4, 5, 10, 20, 21, 23, 24]),
    l(172, 11, &[0, 1, 3, 4, 5, 10, 17, 18, 20, 21, 25]),
    l(173, 12, &[0, 1, 3, 4, 5, 10, 17, 18, 20, 21, 23, 24]),
    l(174, 9, &[0, 1, 2, 3, 4, 5, 10, 20, 21]),
    l(175, 11, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 20, 21]),
    l(176, 11, &[0, 1, 2, 3, 4, 5, 10, 20, 21, 23, 24]),
    l(177, 12, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 20, 21, 25]),
    l(178, 12, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 19, 20, 21]),
    l(179, 13, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 19, 20, 21, 25]),
    l(180, 13, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 20, 21, 23, 24]),
    l(181, 7, &[0, 1, 3, 8, 9, 13, 14]),
    l(182, 9, &[0, 1, 3, 8, 9, 13, 14, 17, 18]),
    l(183, 9, &[0, 1, 3, 8, 9, 13, 14, 23, 24]),
    l(184, 9, &[0, 1, 3, 8, 9, 13, 14, 30, 31]),
    l(185, 11, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 23, 24]),
    l(186, 11, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 30, 31]),
    l(187, 11, &[0, 1, 3, 8, 9, 13, 14, 23, 24, 30, 31]),
    l(188, 12, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 25, 30, 31]),
    l(189, 13, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 23, 24, 30, 31]),
    l(190, 8, &[0, 1, 2, 3, 8, 9, 13, 14]),
    l(191, 10, &[0, 1, 2, 3, 8, 9, 13, 14, 23, 24]),
    l(192, 10, &[0, 1, 2, 3, 8, 9, 13, 14, 30, 31]),
    l(193, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 23, 24]),
    l(194, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 23, 24, 30, 31]),
    l(195, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 30, 31]),
    l(196, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 25, 30, 31]),
    l(197, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 19, 30, 31]),
    l(198, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 19, 25, 30, 31]),
    l(199, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 23, 24, 30, 31]),
    l(200, 9, &[0, 1, 3, 8, 9, 13, 14, 20, 21]),
    l(201, 11, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 20, 21]),
    l(202, 11, &[0, 1, 3, 8, 9, 13, 14, 20, 21, 23, 24]),
    l(203, 12, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 20, 21, 25]),
    l(204, 13, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 20, 21, 23, 24]),
    l(205, 10, &[0, 1, 2, 3, 8, 9, 13, 14, 20, 21]),
    l(206, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 20, 21, 23, 24]),
    l(207, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 20, 21, 25]),
    l(208, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 19, 20, 21]),
    l(209, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 19, 20, 21, 25]),
    l(210, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 20, 21, 23, 24]),
    l(211, 9, &[0, 1, 3, 8, 9, 13, 14, 32, 33]),
    l(212, 11, &[0, 1, 3, 8, 9, 13, 14, 23, 24, 32, 33]),
    l(213, 11, &[0, 1, 3, 8, 9, 13, 14, 30, 31, 32, 33]),
    l(214, 12, &[0, 1, 3, 8, 9, 13, 14, 25, 30, 31, 32, 33]),
    l(215, 13, &[0, 1, 3, 8, 9, 13, 14, 23, 24, 30, 31, 32, 33]),
    l(216, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 23, 24, 32, 33]),
    l(217, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 30, 31, 32, 33]),
    l(218, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 25, 30, 31, 32, 33]),
    l(219, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 19, 30, 31, 32, 33]),
    l(220, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 19, 25, 30, 31, 32, 33]),
    l(221, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 23, 24, 30, 31, 32, 33]),
    l(222, 11, &[0, 1, 3, 8, 9, 13, 14, 20, 21, 32, 33]),
    l(223, 12, &[0, 1, 3, 8, 9, 13, 14, 20, 21, 25, 32, 33]),
    l(224, 13, &[0, 1, 3, 8, 9, 13, 14, 20, 21, 23, 24, 32, 33]),
    l(225, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 20, 21, 32, 33]),
    l(226, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 20, 21, 25, 32, 33]),
    l(227, 13, &[0, 1, 2, 3, 8, 9, 13, 14, 19, 20, 21, 32, 33]),
    l(228, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 19, 20, 21, 25, 32, 33]),
    l(229, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 20, 21, 23, 24, 32, 33]),
    l(230, 7, &[0, 1, 3, 4, 5, 15, 16]),
    l(231, 9, &[0, 1, 3, 4, 5, 15, 16, 17, 18]),
    l(232, 9, &[0, 1, 3, 4, 5, 15, 16, 23, 24]),
    l(233, 9, &[0, 1, 3, 4, 5, 15, 16, 30, 31]),
    l(234, 11, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 23, 24]),
    l(235, 11, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 30, 31]),
    l(236, 11, &[0, 1, 3, 4, 5, 15, 16, 23, 24, 30, 31]),
    l(237, 13, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 23, 24, 30, 31]),
    l(238, 10, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18]),
    l(239, 10, &[0, 1, 2, 3, 4, 5, 15, 16, 23, 24]),
    l(240, 10, &[0, 1, 2, 3, 4, 5, 15, 16, 30, 31]),
    l(241, 12, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 23, 24]),
    l(242, 12, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 30, 31]),
    l(243, 12, &[0, 1, 2, 3, 4, 5, 15, 16, 23, 24, 30, 31]),
    l(244, 14, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 23, 24, 30, 31]),
    l(245, 9, &[0, 1, 3, 4, 5, 15, 16, 20, 21]),
    l(246, 11, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 20, 21]),
    l(247, 11, &[0, 1, 3, 4, 5, 15, 16, 20, 21, 23, 24]),
    l(248, 13, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 20, 21, 23, 24]),
    l(249, 10, &[0, 1, 2, 3, 4, 5, 15, 16, 20, 21]),
    l(250, 12, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 20, 21]),
    l(251, 12, &[0, 1, 2, 3, 4, 5, 15, 16, 20, 21, 23, 24]),
    l(252, 14, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 20, 21, 23, 24]),
    l(253, 8, &[0, 1, 3, 4, 5, 10, 15, 16]),
    l(254, 10, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18]),
    l(255, 10, &[0, 1, 3, 4, 5, 10, 15, 16, 23, 24]),
    l(256, 10, &[0, 1, 3, 4, 5, 10, 15, 16, 30, 31]),
    l(257, 12, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 23, 24]),
    l(258, 12, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 30, 31]),
    l(259, 12, &[0, 1, 3, 4, 5, 10, 15, 16, 23, 24, 30, 31]),
    l(260, 14, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 23, 24, 30, 31]),
    l(261, 9, &[0, 1, 2, 3, 4, 5, 10, 15, 16]),
    l(262, 11, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18]),
    l(263, 11, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 23, 24]),
    l(264, 11, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 30, 31]),
    l(265, 13, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 23, 24]),
    l(266, 13, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 30, 31]),
    l(267, 13, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 23, 24, 30, 31]),
    l(268, 15, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 23, 24, 30, 31]),
    l(269, 10, &[0, 1, 3, 4, 5, 10, 15, 16, 20, 21]),
    l(270, 12, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21]),
    l(271, 12, &[0, 1, 3, 4, 5, 10, 15, 16, 20, 21, 23, 24]),
    l(272, 14, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21, 23, 24]),
    l(273, 11, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 20, 21]),
    l(274, 13, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21]),
    l(275, 13, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 20, 21, 23, 24]),
    l(276, 15, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21, 23, 24]),
    l(277, 9, &[0, 1, 3, 8, 9, 13, 14, 15, 16]),
    l(278, 11, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18]),
    l(279, 11, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 23, 24]),
    l(280, 11, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 30, 31]),
    l(281, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 23, 24]),
    l(282, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 30, 31]),
    l(283, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 23, 24, 30, 31]),
    l(284, 15, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 23, 24, 30, 31]),
    l(285, 11, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 20, 21]),
    l(286, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21]),
    l(287, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 20, 21, 23, 24]),
    l(288, 15, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21, 23, 24]),
    l(289, 10, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16]),
    l(290, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18]),
    l(291, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 23, 24]),
    l(292, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 30, 31]),
    l(293, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 23, 24]),
    l(294, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 30, 31]),
    l(295, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 23, 24, 30, 31]),
    l(296, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 23, 24, 30, 31]),
    l(297, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 20, 21]),
    l(298, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21]),
    l(299, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 20, 21, 23, 24]),
    l(300, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21, 23, 24]),
    l(301, 11, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 32, 33]),
    l(302, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 23, 24, 32, 33]),
    l(303, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 30, 31, 32, 33]),
    l(304, 15, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 23, 24, 30, 31, 32, 33]),
    l(305, 13, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 20, 21, 32, 33]),
    l(306, 15, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 20, 21, 23, 24, 32, 33]),
    l(307, 12, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 32, 33]),
    l(308, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 23, 24, 32, 33]),
    l(309, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 30, 31, 32, 33]),
    l(310, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 23, 24, 30, 31, 32, 33]),
    l(311, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 20, 21, 32, 33]),
    l(312, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 20, 21, 23, 24, 32, 33]),
    l(313, 11, &[0, 1, 3, 4, 5, 17, 18, 20, 21, 30, 31]),
    l(314, 12, &[0, 1, 2, 3, 4, 5, 17, 18, 20, 21, 30, 31]),
    l(315, 12, &[0, 1, 3, 4, 5, 10, 17, 18, 20, 21, 30, 31]),
    l(316, 13, &[0, 1, 2, 3, 4, 5, 10, 17, 18, 20, 21, 30, 31]),
    l(317, 13, &[0, 1, 3, 8, 9, 13, 14, 17, 18, 20, 21, 30, 31]),
    l(318, 14, &[0, 1, 2, 3, 8, 9, 13, 14, 17, 18, 20, 21, 30, 31]),
    l(319, 12, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 20, 21, 25]),
    l(320, 12, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 25, 30, 31]),
    l(321, 13, &[0, 1, 3, 4, 5, 15, 16, 17, 18, 20, 21, 30, 31]),
    l(322, 13, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 20, 21, 25]),
    l(323, 13, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 19, 20, 21]),
    l(324, 14, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 19, 20, 21, 25]),
    l(325, 13, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 25, 30, 31]),
    l(326, 13, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 19, 30, 31]),
    l(327, 14, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 19, 25, 30, 31]),
    l(328, 14, &[0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 20, 21, 30, 31]),
    l(329, 13, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21, 25]),
    l(330, 13, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 25, 30, 31]),
    l(331, 14, &[0, 1, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21, 30, 31]),
    l(332, 14, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21, 25]),
    l(333, 14, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 19, 20, 21]),
    l(334, 15, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 19, 20, 21, 25]),
    l(335, 14, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 25, 30, 31]),
    l(336, 14, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 19, 30, 31]),
    l(337, 15, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 19, 25, 30, 31]),
    l(338, 15, &[0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 20, 21, 30, 31]),
    l(339, 14, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21, 25]),
    l(340, 14, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 25, 30, 31]),
    l(341, 15, &[0, 1, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21, 30, 31]),
    l(342, 15, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21, 25]),
    l(343, 15, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 19, 20, 21]),
    l(344, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 19, 20, 21, 25]),
    l(345, 15, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 25, 30, 31]),
    l(346, 15, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 19, 30, 31]),
    l(347, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 19, 25, 30, 31]),
    l(348, 16, &[0, 1, 2, 3, 8, 9, 13, 14, 15, 16, 17, 18, 20, 21, 30, 31]),
    l(349, 4, &[0, 1, 3, 10]),
    l(350, 5, &[0, 1, 2, 3, 10]),
    l(351, 2, &[2, 3]),
    l(400, 14, &[2, 0, 1, 4, 5, 131, 132, 133, 134, 135, 128, 129, 130, 3]),
    l(401, 9, &[0, 1, 8, 9, 13, 14, 17, 18, 3]),
    l(402, 10, &[2, 0, 1, 8, 9, 13, 14, 17, 18, 3]),
    l(403, 9, &[0, 1, 4, 5, 32, 33, 30, 31, 3]),
    l(404, 10, &[2, 0, 1, 4, 5, 32, 33, 30, 31, 3]),
    l(405, 9, &[0, 1, 4, 5, 17, 18, 20, 21, 3]),
    l(406, 10, &[2, 0, 1, 4, 5, 17, 18, 20, 21, 3]),
    l(407, 11, &[0, 1, 4, 5, 32, 33, 23, 24, 30, 31, 3]),
    l(408, 12, &[2, 0, 1, 4, 5, 32, 33, 23, 24, 30, 31, 3]),
    l(409, 9, &[0, 1, 4, 5, 15, 16, 32, 33, 3]),
    l(410, 10, &[2, 0, 1, 4, 5, 15, 16, 32, 33, 3]),
    l(411, 11, &[0, 1, 4, 5, 15, 16, 32, 33, 30, 31, 3]),
    l(412, 12, &[2, 0, 1, 4, 5, 15, 16, 32, 33, 30, 31, 3]),
    l(413, 13, &[2, 0, 1, 4, 5, 131, 132, 133, 134, 135, 128, 129, 130]),
    l(414, 13, &[2, 0, 1, 4, 5, 19, 17, 18, 30, 31, 128, 129, 130]),
    l(415, 11, &[2, 0, 1, 4, 5, 41, 42, 32, 33, 20, 21]),
    l(416, 9, &[2, 0, 1, 4, 5, 32, 33, 20, 21]),
    l(417, 7, &[2, 0, 1, 138, 139, 136, 137]),
    l(418, 14, &[2, 0, 1, 4, 5, 19, 17, 18, 30, 31, 128, 129, 130, 3]),
    l(419, 12, &[2, 0, 1, 4, 5, 41, 42, 32, 33, 20, 21, 3]),
    l(420, 10, &[2, 0, 1, 4, 5, 32, 33, 20, 21, 3]),
    l(421, 8, &[2, 0, 1, 138, 139, 136, 137, 3]),
    l(422, 10, &[2, 0, 1, 19, 22, 138, 139, 136, 137, 3]),
    l(423, 16, &[2, 0, 1, 4, 5, 19, 17, 18, 30, 31, 128, 129, 130, 140, 141, 3]),
    l(424, 16, &[2, 0, 1, 4, 5, 131, 132, 133, 134, 135, 128, 129, 130, 140, 141, 3]),
];
