// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_dmxMatrixParser.h and
// libFormatConverter/src/FDK_dmxMatrixParser.cpp.

//! Parser of the MPEG-H downmix matrix (`DownmixMatrixSet()`, `DownmixMatrix()`) and of the
//! equalizer configuration (`EqualizerConfig()`).

use mpeghdec_fdk::digest::StateDigest;

use crate::format_converter_intern::*;
use mpeghdec_drc::{DrcDecoder, DRC_DEC_DOWNMIX_ID};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::{
    cicp2geometry_get_num_channels_from_cicp, CICP2GEOMETRY_CICP_VALID,
};
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_add_norm, f_div_norm_signed, f_pow};
use mpeghdec_fdk::scale::scale_value;

include!("dmx_matrix_parser_tables.rs");

pub const DMX_MATRIX_SET_COUNT: usize = 32;
pub const DMX_MATRIX_MAX_SPEAKER_COUNT: usize = 32;
pub const DMX_MATRIX_CODER_STATE_COUNT_MAX: usize = 512;
pub const DMX_MATRIX_GAIN_TABLE_SIZE_MAX: usize = ((22 - (-47)) * (1 << 2) + 2) as usize;
pub const DMX_MATRIX_GAIN_ZERO: i32 = -256;
/// Q23
pub const FDK_DMX_MATRIX_GAIN_ZERO: FixpDbl = DMX_MATRIX_GAIN_ZERO * (1 << 23);

/// Max: ReadEscapedValue(hBitStream, 3, 5, 0) + 1; (dmx_matrix_dec.c)
pub const FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_EQ: usize = 39;
/// Max: ReadEscapedValue(hBitStream, 2, 4, 0) + 1; (dmx_matrix_dec.c)
pub const FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_PEAK_FILTER_PARAMS: usize = 19;
/// max at LC Level 5
pub const FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS: usize =
    FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS;

/// Mpeg-H actually allows up to 65536 loudspeakers in this context.
pub const FDK_MPEGHAUDIO_DEC_MAX_LOUDSPEAKERS: usize = 48;
pub const FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS: usize =
    FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNEL_GROUPS * 2;
pub const FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS: usize = FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS;

pub const FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID: usize = 9;
pub const FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE: usize = 256;

/// `pkFilterParamsStruct`: parameters struct for peak filter definition.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PkFilterParamsStruct {
    /// peak frequency \[Hz\]
    pub f: FixpDbl,
    /// peak Q factor
    pub q: FixpDbl,
    /// peak gain \[dB\]
    pub g: FixpDbl,
    /// peak frequency \[Hz\] exp
    pub f_e: i8,
    /// peak Q factor exp
    pub q_e: i8,
    /// peak gain \[dB\] exp
    pub g_e: i8,
}

/// `eqParamsStruct`: parameters struct for EQ definition.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EqParamsStruct {
    /// nPkFilter-element array of pkFilterParamsStructs
    pub pk_filter_params: [PkFilterParamsStruct; FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_PEAK_FILTER_PARAMS],
    /// global gain \[dB\]
    pub g: FixpDbl,
    /// global gain \[dB\] exp
    pub g_e: i8,
    /// number of cascaded peak filters
    pub n_pk_filter: u8,
}

/// `eqConfigStruct`: parameters struct for the entire EQ configuration.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct EqConfigStruct {
    /// array of EQs of size numEQs
    pub eq_params: [EqParamsStruct; FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_EQ],
    /// index map for the EQs of size inputCount
    pub eq_map: [u32; FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS],
    /// number of EQs
    pub num_eqs: u8,
}

impl Default for EqConfigStruct {
    fn default() -> Self {
        EqConfigStruct {
            eq_params: [EqParamsStruct::default(); FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_EQ],
            eq_map: [0; FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS],
            num_eqs: 0,
        }
    }
}

/// `FDK_DOWNMIX_GROUPS_MATRIX_SET`: the downmix matrices (raw `DownmixMatrix()` payloads) that
/// match the target layout, and the matrix index of each signal group.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct FdkDownmixGroupsMatrixSet {
    pub downmix_matrix_memory:
        [[u8; FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE]; FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID],
    /// payload size in bits
    pub downmix_matrix_size: [i32; FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID],
    pub downmix_matrix: [u8; FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS],
}

impl Default for FdkDownmixGroupsMatrixSet {
    /// All zero (the decoder clears the struct before `DownmixMatrixSet()`).
    fn default() -> Self {
        FdkDownmixGroupsMatrixSet {
            downmix_matrix_memory: [[0; FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE];
                FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID],
            downmix_matrix_size: [0; FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID],
            downmix_matrix: [0; FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS],
        }
    }
}

/// `SP_PAIR_TYPE`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum SpPairType {
    /// one center speaker
    #[default]
    Center = 0,
    /// a symmetric L/R speaker pair
    Symmetric = 1,
    /// an asymmetric single speaker
    Single = 2,
    /// the right speaker of a symmetric L/R speaker pair
    None = 3,
}

/// `SpeakerInformation`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SpeakerInformation {
    /// speaker elevation in degrees, positive angles upwards
    pub elevation: i16,
    /// speaker azimuth in degrees, positive angles to the left
    pub azimuth: i16,
    /// whether the speaker type is LFE
    pub is_lfe: i16,

    /// original speaker position in the channel list
    pub original_position: i16,
    /// channel is already used in the compact channel list
    pub is_already_used: i16,
    /// the right speaker of a symmetric L/R speaker pair (index into the speaker list, `None`
    /// for `NULL`)
    pub symmetric_pair: Option<usize>,
    /// the type of pair for compact speaker configurations
    pub pair_type: SpPairType,
}

/// `CoderState`
#[derive(StateDigest, Clone, Debug)]
pub struct CoderState {
    pub min_gain: i32,
    pub max_gain: i32,
    pub precision_level: i32,
    pub raw_coding_nonzeros: i32,
    pub gain_lgr_param: i32,
    pub history: [FixpDbl; DMX_MATRIX_CODER_STATE_COUNT_MAX],
    pub history_count: i32,
    pub gain_table: [FixpDbl; DMX_MATRIX_GAIN_TABLE_SIZE_MAX],
    pub gain_table_size: i32,
}

impl Default for CoderState {
    fn default() -> Self {
        CoderState {
            min_gain: 0,
            max_gain: 0,
            precision_level: 0,
            raw_coding_nonzeros: 0,
            gain_lgr_param: 0,
            history: [0; DMX_MATRIX_CODER_STATE_COUNT_MAX],
            history_count: 0,
            gain_table: [0; DMX_MATRIX_GAIN_TABLE_SIZE_MAX],
            gain_table_size: 0,
        }
    }
}

/// `FDK_DOWNMIX_BASIC_DATA`
#[derive(StateDigest, Clone, Copy, Debug, Default)]
struct FdkDownmixBasicData {
    downmix_id: [u8; DMX_MATRIX_SET_COUNT],
    cicp_speaker_layout_idx: [u8; DMX_MATRIX_SET_COUNT],
}

/// `ReadRange(hBs, alphabetSize)`
pub fn read_range(h_bs: &mut FdkBitStream, alphabet_size: u32) -> u32 {
    let n_bits = (32 - cnt_leading_zeros(alphabet_size as i32) - 1) as u32; /* floor(log2(alphabetSize)) */
    let n_unused = (1u32 << (n_bits + 1)).wrapping_sub(alphabet_size);
    let mut range = h_bs.read_bits(n_bits);

    if range >= n_unused {
        let range_extra = h_bs.read_bit();
        range = (range << 1).wrapping_sub(n_unused).wrapping_add(range_extra);
    }

    range
}

/// `EqualizerConfig(hBs, inputConfig, inputCount, eqConfig)`: parse the equalizer configuration.
/// Returns 0.
pub fn equalizer_config(
    h_bs: &mut FdkBitStream,
    _input_config: &[SpeakerInformation],
    input_count: u32,
    eq_config: &mut EqConfigStruct,
) -> i32 {
    let equalizer_present = h_bs.read_bit();
    if equalizer_present == 0 {
        eq_config.num_eqs = 0;
        eq_config.eq_map = [0; FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS];
        return 0;
    }

    let num_equalizers = h_bs.escaped_value(3, 5, 0) + 1;
    eq_config.num_eqs = num_equalizers as u8;
    let eq_precision_level = h_bs.read_bits(2);
    let sg_precision_level = f_min(eq_precision_level + 1, 3u32);
    let eq_extended_range = h_bs.read_bit();

    for i in 0..num_equalizers as usize {
        let num_sections = h_bs.escaped_value(2, 4, 0) + 1;
        eq_config.eq_params[i].n_pk_filter = num_sections as u8;
        let mut last_center_freq_p10 = 0u32;
        let mut last_center_freq_ld2 = 10u32;
        let mut max_center_freq_ld2 = 99u32;

        for j in 0..num_sections as usize {
            /* decode centerFreq */
            let center_freq_p10 = last_center_freq_p10 + read_range(h_bs, 4 - last_center_freq_p10);
            if center_freq_p10 > last_center_freq_p10 {
                last_center_freq_ld2 = 10;
            }
            if center_freq_p10 == 3 {
                max_center_freq_ld2 = 24;
            }
            let center_freq_ld2 = last_center_freq_ld2
                + read_range(h_bs, 1 + max_center_freq_ld2 - last_center_freq_ld2);
            let mut center_freq = center_freq_ld2;
            for _ in 0..center_freq_p10 {
                center_freq *= 10;
            }
            last_center_freq_p10 = center_freq_p10;
            last_center_freq_ld2 = center_freq_ld2;
            let mut headroom = (f_normz(center_freq as FixpDbl) - 1) as i8;
            let pk = &mut eq_config.eq_params[i].pk_filter_params[j];
            pk.f = (center_freq as FixpDbl).wrapping_shl(headroom as i32 as u32);
            pk.f_e = (32 - headroom as i32 - 1) as i8;
            /* decode qualityFactor */
            let mut tmp: FixpDbl;
            let quality_factor;
            let mut quality_factor_e = 0;
            let q_factor_index = h_bs.read_bits(5);
            if q_factor_index > 19 {
                let q_factor_extra = h_bs.read_bits(3);
                tmp = ((q_factor_index - 20) * 8 + q_factor_extra + 1) as FixpDbl;
                headroom = (f_normz(tmp) - 1) as i8;
                tmp <<= headroom as i32;
                quality_factor = f_add_norm(
                    0x4000_0000,
                    1,
                    f_mult(fl2fxconst_dbl(0.8), tmp),
                    (32 - headroom as i32) - 3 - 1,
                    &mut quality_factor_e,
                );
            } else {
                tmp = (q_factor_index + 1) as FixpDbl;
                headroom = (f_normz(tmp) - 1) as i8;
                tmp <<= headroom as i32;
                quality_factor = f_mult(fl2fxconst_dbl(0.8), tmp); /*  */
                quality_factor_e = (32 - headroom as i32 - 1) - 4;
            }
            pk.q = quality_factor;
            pk.q_e = quality_factor_e as i8;
            /* decode centerGain */
            let cg_bits = 4 + eq_extended_range + eq_precision_level;
            let center_gain_index = h_bs.read_bits(cg_bits);
            headroom = (f_normz(center_gain_index as FixpDbl) - 1) as i8;
            let mut center_gain =
                (center_gain_index as FixpDbl).wrapping_shl(headroom as i32 as u32);
            /* We have to apply a factor eqPrecisions[eqPrecisionLevel], that is defined as [1.0,
             * 0.5, 0.25, 0.1]. */
            /* We replace that by adding an offset (eqPrecisionLevel) to the exponent and replace
             * 0.1 by 0.8, if used */
            if eq_precision_level == 3 {
                center_gain = f_mult(fl2fxconst_dbl(0.8), center_gain);
            }
            let mut center_gain_e = 0;
            center_gain = f_add_norm(
                EQ_MIN_RANGES[eq_extended_range as usize][eq_precision_level as usize],
                5,
                center_gain,
                32 - headroom as i32 - 1 - eq_precision_level as i32,
                &mut center_gain_e,
            );
            pk.g = center_gain;
            pk.g_e = center_gain_e as i8;
        }
        /* decode scalingGain */
        let sg_bits = 4 + eq_extended_range + sg_precision_level;
        let scaling_gain_index = h_bs.read_bits(sg_bits);
        let headroom = (f_normz(scaling_gain_index as FixpDbl) - 1) as i8;
        let mut scaling_gain = (scaling_gain_index as FixpDbl).wrapping_shl(headroom as i32 as u32);
        /* We have to apply a factor eqPrecisions[sgPrecisionLevel], that is defined as [1.0, 0.5,
         * 0.25, 0.1]. */
        /* We replace that by adding an offset (sgPrecisionLevel) to the exponent and replace 0.1 by
         * 0.8, if used */
        if sg_precision_level == 3 {
            scaling_gain = f_mult(fl2fxconst_dbl(0.8), scaling_gain);
        }
        let mut scaling_gain_e = 0;
        scaling_gain = f_add_norm(
            EQ_MIN_RANGES[eq_extended_range as usize][sg_precision_level as usize],
            5,
            scaling_gain,
            32 - headroom as i32 - 1 - sg_precision_level as i32,
            &mut scaling_gain_e,
        );
        eq_config.eq_params[i].g = scaling_gain;
        eq_config.eq_params[i].g_e = scaling_gain_e as i8;
    }

    for i in 0..input_count as usize {
        let has_equalizer = h_bs.read_bit();
        if has_equalizer != 0 {
            eq_config.eq_map[i] = read_range(h_bs, num_equalizers) + 1;
        } else {
            eq_config.eq_map[i] = 0;
        }
    }

    0
}

/// `ConvertToCompactConfig(inputConfig, inputCount, compactInputCount, compactInputConfig)`:
/// group symmetric L/R speaker pairs. `compact_input_config` receives indices into
/// `input_config`, starting at `*compact_input_count`.
pub fn convert_to_compact_config(
    input_config: &mut [SpeakerInformation],
    input_count: i32,
    compact_input_count: &mut i32,
    compact_input_config: &mut [usize],
) {
    let n = input_count.max(0) as usize;

    for i in 0..n {
        input_config[i].original_position = i as i16;
        input_config[i].is_already_used = 0;
        input_config[i].symmetric_pair = None;
    }

    for i in 0..n {
        if input_config[i].is_already_used != 0 {
            continue;
        }

        if input_config[i].azimuth == 0 || input_config[i].azimuth.wrapping_abs() == 180 {
            /* the speaker is in the center, it has no pair */
            compact_input_config[*compact_input_count as usize] = i;
            *compact_input_count += 1;
            input_config[i].symmetric_pair = None;
            input_config[i].pair_type = SpPairType::Center;
            input_config[i].is_already_used = 1;
        } else {
            for j in i + 1..n {
                if input_config[j].is_already_used != 0 {
                    continue;
                }

                if input_config[i].is_lfe == input_config[j].is_lfe
                    && input_config[i].elevation == input_config[j].elevation
                    && input_config[i].azimuth as i32 == -(input_config[j].azimuth as i32)
                {
                    /* we found a symmetric L/R pair for speaker on position i */
                    if input_config[i].azimuth > 0 {
                        compact_input_config[*compact_input_count as usize] = i;
                        *compact_input_count += 1;
                        input_config[i].symmetric_pair = Some(j);
                        input_config[i].pair_type = SpPairType::Symmetric;
                        input_config[j].symmetric_pair = None;
                        input_config[j].pair_type = SpPairType::None;
                    } else {
                        compact_input_config[*compact_input_count as usize] = j;
                        *compact_input_count += 1;
                        input_config[j].symmetric_pair = Some(i);
                        input_config[j].pair_type = SpPairType::Symmetric;
                        input_config[i].symmetric_pair = None;
                        input_config[i].pair_type = SpPairType::None;
                    }

                    input_config[i].is_already_used = 1;
                    input_config[j].is_already_used = 1;
                    break;
                }
            }

            if input_config[i].is_already_used == 0 {
                /* we did not found a symmetric L/R pair for speaker on position i */
                compact_input_config[*compact_input_count as usize] = i;
                *compact_input_count += 1;
                input_config[i].symmetric_pair = None;
                input_config[i].pair_type = SpPairType::Single;
                input_config[i].is_already_used = 1;
            }
        }
    }
}

/// `DecodeFlatCompactMatrix(hBs, flatCompactMatrix, totalCount)`: run length decoding of the
/// flat compact matrix. Writes beyond the end of `flat_compact_matrix` (possible for invalid
/// bitstreams, in C they land in the scratch buffer behind it) are dropped.
pub fn decode_flat_compact_matrix(
    h_bs: &mut FdkBitStream,
    flat_compact_matrix: &mut [i8],
    total_count: i32,
) {
    let mut n_bits = 3;

    if total_count >= 256 {
        n_bits = 4;
    }
    let run_lgr_param = h_bs.read_bits(n_bits) as i32;
    let max_head = total_count >> run_lgr_param;
    let mut idx: i32 = 0;
    loop {
        let mut head = 0;

        while head < max_head {
            if h_bs.read_bit() != 0 {
                break;
            }
            head += 1;
        }
        let run = (head << run_lgr_param) + h_bs.read_bits(run_lgr_param as u32) as i32;
        for _ in 0..run {
            if let Some(v) = flat_compact_matrix.get_mut(idx as usize) {
                *v = 0;
            } else {
                mpeghdec_fdk::deviation::hit();
            }
            idx += 1;
        }
        if idx < total_count {
            flat_compact_matrix[idx as usize] = 1;
            idx += 1;
        }
        if idx >= total_count {
            break;
        }
    }
}

/// `DecodeGainValue(hBs, cs, gainValue)`: returns 0 on success, -1 for an invalid gain index.
pub fn decode_gain_value(
    h_bs: &mut FdkBitStream,
    cs: &CoderState,
    gain_value: &mut FixpDbl,
) -> i32 {
    *gain_value = FDK_DMX_MATRIX_GAIN_ZERO;

    if cs.raw_coding_nonzeros != 0 {
        let n_alphabet = ((cs.max_gain - cs.min_gain) << cs.precision_level) + 2;
        let gain_value_index = read_range(h_bs, n_alphabet as u32) as i32;
        *gain_value = cs.max_gain - (gain_value_index >> cs.precision_level);
    } else {
        let max_head = (cs.gain_table_size - 1) >> cs.gain_lgr_param;
        let mut head = 0;

        while head < max_head {
            if h_bs.read_bit() != 0 {
                break;
            }
            head += 1;
        }
        let gain_value_index =
            (head << cs.gain_lgr_param) + h_bs.read_bits(cs.gain_lgr_param as u32) as i32;
        if gain_value_index >= cs.gain_table_size {
            return -1;
        }
        *gain_value = cs.gain_table[gain_value_index as usize];
    }

    if *gain_value < (cs.min_gain << 23) {
        *gain_value = FDK_DMX_MATRIX_GAIN_ZERO;
    }

    0
}

/// `CoderStateGenerateGainTable(cs)`
pub fn coder_state_generate_gain_table(cs: &mut CoderState) {
    let mut index = 0usize;
    /* Word: Q24 (range -96 to 96 dB) */

    debug_assert!(cs.precision_level <= 2); /* 2 fractional bits give 0.25 dB resolution */

    /* add multiples of 3 to the table, first values <= 0 and then values > 0 */
    let mut i = 0;
    while i >= cs.min_gain {
        cs.gain_table[index] = i << 23;
        index += 1;
        i -= 3;
    }
    i = 3;
    while i <= cs.max_gain {
        cs.gain_table[index] = i << 23;
        index += 1;
        i += 3;
    }

    /* add the rest of the integers which are not multiples of 3 to the table */
    i = 0;
    while i >= cs.min_gain {
        if i % 3 != 0 {
            cs.gain_table[index] = i << 23;
            index += 1;
        }
        i -= 1;
    }

    i = 1;
    while i <= cs.max_gain {
        if i % 3 != 0 {
            cs.gain_table[index] = i << 23;
            index += 1;
        }
        i += 1;
    }

    /* add values which are multiples of 0.5, then multiples of 0.25,
    up to and finally multiples of 2 ^ (-precisionLevel) to the table,
    but do not add values which are already in the table */
    for p in 1..=cs.precision_level {
        let step: FixpDbl = 0x0040_0000 >> (p - 1);

        let mut f: FixpDbl = 0;
        while f >= (cs.min_gain << 23) {
            if (f & step) != 0 {
                cs.gain_table[index] = f;
                index += 1;
            }
            f -= step;
        }
        f = step;
        while f <= (cs.max_gain << 23) {
            if (f & step) != 0 {
                cs.gain_table[index] = f;
                index += 1;
            }
            f += step;
        }
    }

    cs.gain_table[index] = FDK_DMX_MATRIX_GAIN_ZERO;
    index += 1;
    cs.gain_table_size = index as i32;
    debug_assert!(cs.gain_table_size == ((cs.max_gain - cs.min_gain) << cs.precision_level) + 2);
}

static COMPACT_TEMPLATES_DATA: [&[i8]; 8] = [
    &COMPACT_TEMPLATE_CICP13_TO_CICP6,
    &COMPACT_TEMPLATE_CICP13_TO_CICP14,
    &COMPACT_TEMPLATE_CICP13_TO_CICP12,
    &COMPACT_TEMPLATE_CICP13_TO_CICP7,
    &COMPACT_TEMPLATE_CICP14_TO_CICP6,
    &COMPACT_TEMPLATE_CICP12_TO_CICP6,
    &COMPACT_TEMPLATE_CICP7_TO_CICP6,
    &COMPACT_TEMPLATE_CICP13_TO_CICP2,
];

/// `FindCompactTemplate(inputIndex, outputIndex)`: the compact template for the CICP layouts
/// (`None` if there is none).
pub fn find_compact_template(input_index: i32, output_index: i32) -> Option<&'static [i8]> {
    /* a compact template should be used only for CICP speaker layouts */
    debug_assert!(input_index != -1 && output_index != -1);

    for i in 0..COMPACT_TEMPLATES_DATA.len() {
        if COMPACT_TEMPLATES_INPUT_INDEX[i] as i32 != input_index {
            continue;
        }
        if COMPACT_TEMPLATES_OUTPUT_INDEX[i] as i32 != output_index {
            continue;
        }

        /* a matching template was found */
        return Some(COMPACT_TEMPLATES_DATA[i]);
    }

    /* a matching template was not found */
    None
}

/// `CoderStateInit(cs)`
pub fn coder_state_init(cs: &mut CoderState) {
    cs.min_gain = -1;
    cs.max_gain = 0;
    cs.precision_level = 0;
    cs.raw_coding_nonzeros = 1;
    cs.gain_lgr_param = -1;
    cs.history_count = 0;
    cs.gain_table_size = 0;
}

/// `DecodeDownmixMatrix(inputIndex, inputCount, inputConfig, outputIndex, outputCount,
/// outputConfig, hBs, downmixMatrix, eqConfig, p_buffer)`: decode a `DownmixMatrix()` payload
/// into the linear downmix matrix `downmix_matrix[i * outputCount + j]` (`FIXP_SGL`) and the
/// equalizer configuration. `inputIndex`/`outputIndex` are the CICP layout indices (-1 if not a
/// CICP layout). Returns 0 on success, -1 on error. The scratch buffer of C is allocated
/// internally.
///
/// The compact template is read with bounds checks: C reads beyond the template (adjacent
/// constant data) if the speaker lists do not match the CICP indices; the port uses 0 there.
pub fn decode_downmix_matrix(
    input_index: i32,
    input_count: i32,
    input_config: &mut [SpeakerInformation],
    output_index: i32,
    output_count: i32,
    output_config: &mut [SpeakerInformation],
    h_bs: &mut FdkBitStream,
    downmix_matrix: &mut [FixpSgl],
    eq_config: &mut EqConfigStruct,
) -> i32 {
    let mut downmix_matrix_tmp = vec![
        0 as FixpDbl;
        FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS
            * FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS
    ];
    let mut compact_input_config = [0usize; DMX_MATRIX_MAX_SPEAKER_COUNT];
    let mut compact_input_count = 0;
    let mut compact_output_config = [0usize; DMX_MATRIX_MAX_SPEAKER_COUNT];
    let mut compact_output_count = 0;
    let mut is_separable = [0i32; DMX_MATRIX_MAX_SPEAKER_COUNT];
    let mut is_symmetric = [0i32; DMX_MATRIX_MAX_SPEAKER_COUNT];
    let mut compact_downmix_matrix =
        [[0i32; DMX_MATRIX_MAX_SPEAKER_COUNT]; DMX_MATRIX_MAX_SPEAKER_COUNT];
    let mut compact_template: Option<&'static [i8]> = None; /* compactTemplate[compactInputCount][compactOutputCount]
                                                            stored in the C array layout */

    let mut cs = CoderState::default();

    equalizer_config(h_bs, input_config, input_count as u32, eq_config);

    let precision_level = h_bs.read_bits(2) as i32;
    if precision_level > 2 {
        return -1;
    }

    let max_gain = h_bs.escaped_value(3, 4, 0) as i32;
    let min_gain = -(h_bs.escaped_value(4, 5, 0) as i32 + 1);

    convert_to_compact_config(
        input_config,
        input_count,
        &mut compact_input_count,
        &mut compact_input_config,
    );
    convert_to_compact_config(
        output_config,
        output_count,
        &mut compact_output_count,
        &mut compact_output_config,
    );
    let cic = compact_input_count as usize;
    let coc = compact_output_count as usize;

    let is_all_separable = h_bs.read_bit();

    if is_all_separable == 0 {
        for i in 0..coc {
            if output_config[compact_output_config[i]].pair_type == SpPairType::Symmetric {
                is_separable[i] = h_bs.read_bit() as i32;
            }
        }
    } else {
        for i in 0..coc {
            if output_config[compact_output_config[i]].pair_type == SpPairType::Symmetric {
                is_separable[i] = 1;
            }
        }
    }

    let is_all_symmetric = h_bs.read_bit();
    if is_all_symmetric == 0 {
        for i in 0..coc {
            is_symmetric[i] = h_bs.read_bit() as i32;
        }
    } else {
        for i in 0..coc {
            is_symmetric[i] = 1;
        }
    }

    /* decode the compact template */
    let mix_lfe_only_to_lfe = h_bs.read_bit();
    let raw_coding_compact_matrix = h_bs.read_bit();

    if raw_coding_compact_matrix != 0 {
        for i in 0..cic {
            for j in 0..coc {
                if mix_lfe_only_to_lfe == 0
                    || input_config[compact_input_config[i]].is_lfe
                        == output_config[compact_output_config[j]].is_lfe
                {
                    compact_downmix_matrix[i][j] = h_bs.read_bit() as i32;
                } else {
                    compact_downmix_matrix[i][j] = 0;
                }
            }
        }
    } else {
        let mut flat_compact_matrix =
            [0i8; DMX_MATRIX_MAX_SPEAKER_COUNT * DMX_MATRIX_MAX_SPEAKER_COUNT];
        let use_compact_template = h_bs.read_bit();

        /* compute totalCount as described in the ComputeTotalCount() function */
        let total_count = if mix_lfe_only_to_lfe != 0 {
            let mut compact_input_lfe_count = 0;
            let mut compact_output_lfe_count = 0;

            for i in 0..cic {
                if input_config[compact_input_config[i]].is_lfe != 0 {
                    compact_input_lfe_count += 1;
                }
            }
            for i in 0..coc {
                if output_config[compact_output_config[i]].is_lfe != 0 {
                    compact_output_lfe_count += 1;
                }
            }
            (compact_input_count - compact_input_lfe_count)
                * (compact_output_count - compact_output_lfe_count)
        } else {
            compact_input_count * compact_output_count
        };

        if use_compact_template != 0 {
            /* a compact template should be used only for CICP speaker layouts */
            if input_index == -1 || output_index == -1 {
                /* when useCompactTemplate is enabled and one of the CICP indexes are -1, it means
                that the decoder version is too old and it does not have the required CICP tables */
                return -1;
            }

            compact_template = find_compact_template(input_index, output_index);
            if compact_template.is_none() {
                /* a template was not found */
                /* when useCompactTemplate is enabled and a template was not found, it means
                that the decoder version is too old and it does not have the required tables */
                return -1;
            }
        }

        decode_flat_compact_matrix(h_bs, &mut flat_compact_matrix, total_count);
        let mut count = 0usize;
        for i in 0..cic {
            for j in 0..coc {
                let in_lfe = input_config[compact_input_config[i]].is_lfe;
                let out_lfe = output_config[compact_output_config[j]].is_lfe;
                if mix_lfe_only_to_lfe != 0 && in_lfe != 0 && out_lfe != 0 {
                    compact_downmix_matrix[i][j] = h_bs.read_bit() as i32;
                } else if mix_lfe_only_to_lfe != 0 && (in_lfe ^ out_lfe) != 0 {
                    compact_downmix_matrix[i][j] = 0;
                } else {
                    /* !mixLFEOnlyToLFE || (!compactInputConfig[i]->isLFE &&
                    !compactOutputConfig[j]->isLFE) */
                    compact_downmix_matrix[i][j] = flat_compact_matrix[count] as i32;
                    count += 1;
                    if use_compact_template != 0 {
                        let t = compact_template.unwrap();
                        compact_downmix_matrix[i][j] ^=
                            t.get(i * coc + j).copied().unwrap_or_else(|| {
                                mpeghdec_fdk::deviation::hit();
                                0
                            }) as i32;
                    }
                }
            }
        }
        debug_assert!(count as i32 == total_count);
    }

    /* decode values for the nonzero entries in compactDownmixMatrix */
    let full_for_asymmetric_inputs = h_bs.read_bit() as i32;
    let raw_coding_nonzeros = h_bs.read_bit() as i32;

    coder_state_init(&mut cs);
    cs.history_count = 0;
    cs.raw_coding_nonzeros = raw_coding_nonzeros;
    cs.gain_lgr_param = -1;
    cs.min_gain = min_gain;
    cs.max_gain = max_gain;
    cs.precision_level = precision_level;
    cs.gain_table_size = 0;

    if raw_coding_nonzeros == 0 {
        cs.gain_lgr_param = h_bs.read_bits(3) as i32;
        coder_state_generate_gain_table(&mut cs);
    }

    let oc = output_count as usize;
    let tmp = &mut downmix_matrix_tmp;
    for i in 0..cic {
        let ic = &input_config[compact_input_config[i]];
        let i_type = ic.pair_type;
        debug_assert!(i_type != SpPairType::None);

        for j in 0..coc {
            let occ = &output_config[compact_output_config[j]];
            let o_type = occ.pair_type;
            let i1 = ic.original_position as usize;
            let o1 = occ.original_position as usize;
            debug_assert!(o_type != SpPairType::None);

            if i_type != SpPairType::Symmetric && o_type != SpPairType::Symmetric {
                tmp[i1 * oc + o1] = FDK_DMX_MATRIX_GAIN_ZERO;
                if compact_downmix_matrix[i][j] == 0 {
                    continue;
                }

                if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o1]) != 0 {
                    return -1;
                }
            } else if i_type != SpPairType::Symmetric {
                let o2 = output_config[occ.symmetric_pair.unwrap()].original_position as usize;
                let use_full = i_type == SpPairType::Single && full_for_asymmetric_inputs != 0;
                tmp[i1 * oc + o1] = FDK_DMX_MATRIX_GAIN_ZERO;
                tmp[i1 * oc + o2] = FDK_DMX_MATRIX_GAIN_ZERO;
                if compact_downmix_matrix[i][j] == 0 {
                    continue;
                }

                if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o1]) != 0 {
                    return -1;
                }
                if is_symmetric[j] != 0 && !use_full {
                    tmp[i1 * oc + o2] = tmp[i1 * oc + o1];
                } else if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o2]) != 0 {
                    return -1;
                }
            } else if o_type != SpPairType::Symmetric {
                let i2 = input_config[ic.symmetric_pair.unwrap()].original_position as usize;
                tmp[i1 * oc + o1] = FDK_DMX_MATRIX_GAIN_ZERO;
                tmp[i2 * oc + o1] = FDK_DMX_MATRIX_GAIN_ZERO;
                if compact_downmix_matrix[i][j] == 0 {
                    continue;
                }

                if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o1]) != 0 {
                    return -1;
                }
                if is_symmetric[j] != 0 {
                    tmp[i2 * oc + o1] = tmp[i1 * oc + o1];
                } else if decode_gain_value(h_bs, &cs, &mut tmp[i2 * oc + o1]) != 0 {
                    return -1;
                }
            } else {
                let i2 = input_config[ic.symmetric_pair.unwrap()].original_position as usize;
                let o2 = output_config[occ.symmetric_pair.unwrap()].original_position as usize;
                tmp[i1 * oc + o1] = FDK_DMX_MATRIX_GAIN_ZERO;
                tmp[i1 * oc + o2] = FDK_DMX_MATRIX_GAIN_ZERO;
                tmp[i2 * oc + o1] = FDK_DMX_MATRIX_GAIN_ZERO;
                tmp[i2 * oc + o2] = FDK_DMX_MATRIX_GAIN_ZERO;
                if compact_downmix_matrix[i][j] == 0 {
                    continue;
                }

                if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o1]) != 0 {
                    return -1;
                }
                if is_separable[j] != 0 && is_symmetric[j] != 0 {
                    tmp[i2 * oc + o2] = tmp[i1 * oc + o1];
                } else if is_separable[j] == 0 && is_symmetric[j] != 0 {
                    if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o2]) != 0 {
                        return -1;
                    }
                    tmp[i2 * oc + o1] = tmp[i1 * oc + o2];
                    tmp[i2 * oc + o2] = tmp[i1 * oc + o1];
                } else if is_separable[j] != 0 && is_symmetric[j] == 0 {
                    if decode_gain_value(h_bs, &cs, &mut tmp[i2 * oc + o2]) != 0 {
                        return -1;
                    };
                } else {
                    /* !isSeparable[j] && !isSymmetric[j] */
                    if decode_gain_value(h_bs, &cs, &mut tmp[i1 * oc + o2]) != 0 {
                        return -1;
                    }
                    if decode_gain_value(h_bs, &cs, &mut tmp[i2 * oc + o1]) != 0 {
                        return -1;
                    }
                    if decode_gain_value(h_bs, &cs, &mut tmp[i2 * oc + o2]) != 0 {
                        return -1;
                    }
                }
            }
        }
    }
    /* convert the downmix matrix to linear domain */
    for i in 0..input_count.max(0) as usize {
        for j in 0..oc {
            let value = tmp[i * oc + j];
            if (value >> 23) != DMX_MATRIX_GAIN_ZERO {
                let mut exp_e = 0;
                /* FL2FX_DBL(0.625) */
                let exp_m = f_div_norm_signed(value, 0x5000_0000, &mut exp_e);
                exp_e += 3; /* Value (Q23) / FL2FX_DBL(0.625) (Q26) -> exp_e = (8-5) */
                let mut pow_e = 0;
                let pow_m;
                if exp_m != 0 {
                    pow_m = f_pow(0x5000_0000, 4, exp_m, exp_e, &mut pow_e);
                } else {
                    pow_m = MAXVAL_DBL;
                    pow_e = 0;
                }
                downmix_matrix[i * oc + j] = fx_dbl2fx_sgl(scale_value(pow_m, pow_e));
            } else {
                downmix_matrix[i * oc + j] = 0;
            }
        }
    }
    0
}

/// `SetUniDrcDownmixInfo()`
fn set_uni_drc_downmix_info(
    h_uni_drc_dec: &mut DrcDecoder,
    downmix_id_count: i32,
    dmx_bd: &FdkDownmixBasicData,
    downmix_id: i32,
) -> i32 {
    let mut downmix_id_list = [0i32; DMX_MATRIX_SET_COUNT];
    let mut target_layout = [0i32; DMX_MATRIX_SET_COUNT];
    let mut target_channel_count = [0i32; DMX_MATRIX_SET_COUNT];

    if downmix_id_count as usize > DMX_MATRIX_SET_COUNT {
        return -1;
    }

    for i in 0..downmix_id_count.max(0) as usize {
        downmix_id_list[i] = dmx_bd.downmix_id[i] as i32;
        target_layout[i] = dmx_bd.cicp_speaker_layout_idx[i] as i32;
        /* get targetChannelCount */
        let err = cicp2geometry_get_num_channels_from_cicp(
            dmx_bd.cicp_speaker_layout_idx[i] as i32,
            &mut target_channel_count[i],
        );
        if err != 0 {
            return -1;
        }
    }
    let err = h_uni_drc_dec.set_downmix_instructions(
        downmix_id_count,
        &downmix_id_list,
        &target_layout,
        &target_channel_count,
    );
    if err != 0 {
        return -1;
    }

    let err = h_uni_drc_dec.set_param(DRC_DEC_DOWNMIX_ID, downmix_id);
    if err != 0 {
        return -1;
    }

    err
}

/// `DownmixMatrixSet(hBs, groupsDownmixMatrixSet, targetLayout, downmixConfigType, downmixId,
/// hUniDrcDec)`: parse `DownmixMatrixSet()`. The `DownmixMatrix()` payloads for the target
/// layout are stored in `groups_downmix_matrix_set` (which must be cleared by the caller), the
/// matching downmix id is returned in `downmix_id` (-1 if none) and forwarded to the DRC decoder
/// (if given). Returns 0 on success, -1 on error.
pub fn downmix_matrix_set(
    h_bs: &mut FdkBitStream,
    groups_downmix_matrix_set: &mut FdkDownmixGroupsMatrixSet,
    target_layout: i32,
    _downmix_config_type: i32,
    downmix_id: &mut i32,
    h_uni_drc_dec: Option<&mut DrcDecoder>,
) -> i32 {
    let mut error_status = 0;

    *downmix_id = -1;

    let mut dmx_bd = FdkDownmixBasicData::default();

    let downmix_id_count = h_bs.read_bits(5) as i32;

    let cicp_index = target_layout;

    for k in 0..downmix_id_count as usize {
        let downmix_id_local = h_bs.read_bits(7) as u8;
        let downmix_type = h_bs.read_bits(2) as u8;

        /* If downmixType > 1 exit */
        if downmix_type > 1 {
            error_status = -1;
            break;
        }
        dmx_bd.downmix_id[k] = downmix_id_local;

        let cicp_speaker_layout_idx = h_bs.read_bits(6) as u8;
        dmx_bd.cicp_speaker_layout_idx[k] = cicp_speaker_layout_idx;

        if cicp_speaker_layout_idx as i32 == cicp_index && cicp_index > CICP2GEOMETRY_CICP_VALID {
            *downmix_id = downmix_id_local as i32;
        }

        /*Do nothing for downmixType=0 */
        if downmix_type == 1 {
            if cicp_speaker_layout_idx as i32 == cicp_index {
                /* Parse just the needed data*/

                let bs_downmix_matrix_count = (h_bs.escaped_value(1, 3, 0) + 1) as u8;
                for l in 0..bs_downmix_matrix_count as usize {
                    let bs_num_assigned_group_ids = (h_bs.escaped_value(1, 4, 4) + 1) as u16;
                    for _ in 0..bs_num_assigned_group_ids {
                        let signal_group_id = h_bs.read_bits(5) as u8;
                        if signal_group_id as usize > FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS - 1 {
                            error_status = -1;
                            break;
                        }
                        groups_downmix_matrix_set.downmix_matrix[signal_group_id as usize] =
                            l as u8;
                    }

                    groups_downmix_matrix_set.downmix_matrix_size[l] =
                        h_bs.escaped_value(8, 8, 12) as i32;

                    let mut dmx_matrix_len_bytes =
                        groups_downmix_matrix_set.downmix_matrix_size[l] >> 3;
                    let dmx_matrix_bits_left = groups_downmix_matrix_set.downmix_matrix_size[l] % 8;

                    if ((groups_downmix_matrix_set.downmix_matrix_size[l] + 7) >> 3) as usize
                        > FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE
                    {
                        error_status = -1;
                        break;
                    }

                    let mut byte_cnt = 0usize;
                    while dmx_matrix_len_bytes > 0 {
                        dmx_matrix_len_bytes -= 1;
                        groups_downmix_matrix_set.downmix_matrix_memory[l][byte_cnt] =
                            h_bs.read_bits(8) as u8;
                        byte_cnt += 1;
                    }
                    if dmx_matrix_bits_left != 0 {
                        groups_downmix_matrix_set.downmix_matrix_memory[l][byte_cnt] =
                            (h_bs.read_bits(dmx_matrix_bits_left as u32)
                                << (8 - dmx_matrix_bits_left)) as u8;
                    }
                } /*for (UCHAR l = 0; l < bsDownmixMatrixCount; l++)*/

                if error_status != 0 {
                    break;
                }
            } else {
                /* Dummy parsing*/

                let bs_downmix_matrix_count = (h_bs.escaped_value(1, 3, 0) + 1) as u8;
                for l in 0..bs_downmix_matrix_count as usize {
                    let bs_num_assigned_group_ids = (h_bs.escaped_value(1, 4, 4) + 1) as u16;
                    for _ in 0..bs_num_assigned_group_ids {
                        let signal_group_id = h_bs.read_bits(5) as u8;
                        if signal_group_id as usize > FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS - 1 {
                            error_status = -1;
                            break;
                        }
                    }

                    let downmix_matrix_size = h_bs.escaped_value(8, 8, 12) as i32;

                    let mut dmx_matrix_len_bytes = downmix_matrix_size >> 3;
                    let dmx_matrix_bits_left = downmix_matrix_size % 8;

                    if ((groups_downmix_matrix_set.downmix_matrix_size[l] + 7) >> 3) as usize
                        > FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE
                    {
                        error_status = -1;
                        break;
                    }

                    while dmx_matrix_len_bytes > 0 {
                        dmx_matrix_len_bytes -= 1;
                        h_bs.read_bits(8);
                    }
                    if dmx_matrix_bits_left != 0 {
                        h_bs.read_bits(dmx_matrix_bits_left as u32);
                    }
                } /*for (UCHAR l = 0; l < bsDownmixMatrixCount; l++)*/

                if error_status != 0 {
                    break;
                }
            } /* if(CICPspeakerLayoutIdx == cicpIndex)*/
        } /* if ( downmixType == 1 )*/
    } /*for ( UCHAR k = 0; k < downmixIdCount; k++)*/

    /* Transfer of downmix information to DRC decoder */
    if error_status == 0 {
        if let Some(drc) = h_uni_drc_dec {
            error_status = set_uni_drc_downmix_info(drc, downmix_id_count, &dmx_bd, *downmix_id);
        }
    }

    error_status
}
