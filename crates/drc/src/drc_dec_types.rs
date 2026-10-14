// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDec_types.h.

//! Data structures corresponding to static and dynamic DRC/Loudness payload as defined in section
//! 7 of MPEG-D DRC standard, ISO/IEC 23003-4.
//!
//! All structures are plain data; `ZERO` corresponds to memory cleared with `FDKmemclear` or
//! allocated with `FDKcalloc`. Derived `PartialEq` implementations are equivalent to `FDKmemcmp`
//! of the C structures, because all union members and padding bytes are represented (padding is
//! always zero in C, the structures are cleared before they are filled).

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::{FixpDbl, FixpSgl};

/**************/
/* uniDrcGain */
/**************/

/// `GAIN_NODE`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GainNode {
    /// e = 7
    pub gain_db: FixpSgl,
    pub time: i16,
}

impl GainNode {
    pub const ZERO: Self = Self { gain_db: 0, time: 0 };
}

/// `UNI_DRC_GAIN_EXTENSION`: uniDrcGainExtension() (Table 56)
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UniDrcGainExtension {
    pub uni_drc_gain_ext_type: [u8; 8],
    pub ext_bit_size: [u32; 8 - 1],
}

impl UniDrcGainExtension {
    pub const ZERO: Self = Self { uni_drc_gain_ext_type: [0; 8], ext_bit_size: [0; 7] };
}

/// `UNI_DRC_GAIN`: uniDrcGain() (Table 55)
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct UniDrcGain {
    /// unsaturated value, i.e. as provided in bitstream
    pub n_nodes: [u8; 48],
    pub gain_node: [[GainNode; 32]; 48],

    pub uni_drc_gain_ext_present: u8,
    pub uni_drc_gain_extension: UniDrcGainExtension,

    /* derived data */
    pub n_decoded_sequences: [u8; 4],
    pub status: u8,
}

impl UniDrcGain {
    pub const ZERO: Self = Self {
        n_nodes: [0; 48],
        gain_node: [[GainNode::ZERO; 32]; 48],
        uni_drc_gain_ext_present: 0,
        uni_drc_gain_extension: UniDrcGainExtension::ZERO,
        n_decoded_sequences: [0; 4],
        status: 0,
    };
}

impl Default for UniDrcGain {
    fn default() -> Self {
        Self::ZERO
    }
}

/****************/
/* uniDrcConfig */
/****************/

/// `EFFECT_BIT`
pub type EffectBit = i32;
/// `EB_NIGHT`
pub const EB_NIGHT: EffectBit = 0x0001;
/// `EB_NOISY`
pub const EB_NOISY: EffectBit = 0x0002;
/// `EB_LIMITED`
pub const EB_LIMITED: EffectBit = 0x0004;
/// `EB_LOWLEVEL`
pub const EB_LOWLEVEL: EffectBit = 0x0008;
/// `EB_DIALOG`
pub const EB_DIALOG: EffectBit = 0x0010;
/// `EB_GENERAL_COMPR`
pub const EB_GENERAL_COMPR: EffectBit = 0x0020;
/// `EB_EXPAND`
pub const EB_EXPAND: EffectBit = 0x0040;
/// `EB_ARTISTIC`
pub const EB_ARTISTIC: EffectBit = 0x0080;
/// `EB_CLIPPING`
pub const EB_CLIPPING: EffectBit = 0x0100;
/// `EB_FADE`
pub const EB_FADE: EffectBit = 0x0200;
/// `EB_DUCK_OTHER`
pub const EB_DUCK_OTHER: EffectBit = 0x0400;
/// `EB_DUCK_SELF`
pub const EB_DUCK_SELF: EffectBit = 0x0800;

/// `GAIN_CODING_PROFILE`
pub type GainCodingProfile = i32;
/// `GCP_REGULAR`
pub const GCP_REGULAR: GainCodingProfile = 0;
/// `GCP_FADING`
pub const GCP_FADING: GainCodingProfile = 1;
/// `GCP_CLIPPING_DUCKING`
pub const GCP_CLIPPING_DUCKING: GainCodingProfile = 2;
/// `GCP_CONSTANT`
pub const GCP_CONSTANT: GainCodingProfile = 3;

/// `GAIN_INTERPOLATION_TYPE`
pub type GainInterpolationType = i32;
/// `GIT_SPLINE`
pub const GIT_SPLINE: GainInterpolationType = 0;
/// `GIT_LINEAR`
pub const GIT_LINEAR: GainInterpolationType = 1;

/// `CHARACTERISTIC_SIDE`
pub type CharacteristicSide = i32;
/// `CS_LEFT`
pub const CS_LEFT: CharacteristicSide = 0;
/// `CS_RIGHT`
pub const CS_RIGHT: CharacteristicSide = 1;

/// `CHARACTERISTIC_FORMAT`
pub type CharacteristicFormat = i32;
/// `CF_SIGMOID`
pub const CF_SIGMOID: CharacteristicFormat = 0;
/// `CF_NODES`
pub const CF_NODES: CharacteristicFormat = 1;

/// `EQ_SUBBAND_GAIN_FORMAT`
pub type EqSubbandGainFormat = i32;
/// `GF_QMF32`
pub const GF_QMF32: EqSubbandGainFormat = 0x1;
/// `GF_QMFHYBRID39`
pub const GF_QMFHYBRID39: EqSubbandGainFormat = 0x2;
/// `GF_QMF64`
pub const GF_QMF64: EqSubbandGainFormat = 0x3;
/// `GF_QMFHYBRID71`
pub const GF_QMFHYBRID71: EqSubbandGainFormat = 0x4;
/// `GF_QMF128`
pub const GF_QMF128: EqSubbandGainFormat = 0x5;
/// `GF_QMFHYBRID135`
pub const GF_QMFHYBRID135: EqSubbandGainFormat = 0x6;
/// `GF_UNIFORM`
pub const GF_UNIFORM: EqSubbandGainFormat = 0x7;

/// `DUCKING_MODIFICATION`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DuckingModification {
    /// e = 2
    pub ducking_scaling: FixpSgl,
}

impl DuckingModification {
    pub const ZERO: Self = Self { ducking_scaling: 0 };
}

/// `GAIN_MODIFICATION`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GainModification {
    /// e = 2
    pub attenuation_scaling: FixpSgl,
    /// e = 2
    pub amplification_scaling: FixpSgl,
    /// e = 4
    pub gain_offset: FixpSgl,
}

/// `BAND_BORDER`: union of `UCHAR crossoverFreqIndex` and `USHORT startSubBandIndex`.
///
/// The union is stored as its `USHORT` member; `crossoverFreqIndex` is its low byte (little
/// endian).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BandBorder {
    pub start_sub_band_index: u16,
}

impl BandBorder {
    pub const ZERO: Self = Self { start_sub_band_index: 0 };

    /// `crossoverFreqIndex`
    #[inline]
    pub fn crossover_freq_index(&self) -> u8 {
        self.start_sub_band_index as u8
    }

    /// Assignment to `crossoverFreqIndex` (writes the low byte only).
    #[inline]
    pub fn set_crossover_freq_index(&mut self, v: u8) {
        self.start_sub_band_index = (self.start_sub_band_index & 0xFF00) | v as u16;
    }
}

/// `CUSTOM_INDEX`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CustomIndex {
    pub left: u8,
    pub right: u8,
}

/// `DRC_CHARACTERISTIC`. The union of `UCHAR cicpIndex` and `CUSTOM_INDEX custom` is stored as
/// `cicp_index` (= `custom.left`) and `custom_right`.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DrcCharacteristic {
    pub present: u8,
    pub is_cicp: u8,
    /// `cicpIndex`, shares its byte with `custom.left`
    pub cicp_index: u8,
    /// `custom.right`
    pub custom_right: u8,
}

impl DrcCharacteristic {
    pub const ZERO: Self = Self { present: 0, is_cicp: 0, cicp_index: 0, custom_right: 0 };

    /// `custom`
    pub fn custom(&self) -> CustomIndex {
        CustomIndex { left: self.cicp_index, right: self.custom_right }
    }
}

/// `GAIN_SET`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GainSet {
    pub gain_coding_profile: u8,
    pub gain_interpolation_type: u8,
    pub full_frame: u8,
    pub time_alignment: u8,
    pub time_delta_min_present: u8,
    pub time_delta_min: u16,
    pub band_count: u8,
    pub drc_band_type: u8,
    pub gain_sequence_index: [u8; 4],
    pub drc_characteristic: [DrcCharacteristic; 4],
    pub band_border: [BandBorder; 4],
}

impl GainSet {
    pub const ZERO: Self = Self {
        gain_coding_profile: 0,
        gain_interpolation_type: 0,
        full_frame: 0,
        time_alignment: 0,
        time_delta_min_present: 0,
        time_delta_min: 0,
        band_count: 0,
        drc_band_type: 0,
        gain_sequence_index: [0; 4],
        drc_characteristic: [DrcCharacteristic::ZERO; 4],
        band_border: [BandBorder::ZERO; 4],
    };
}

/// `DRC_COEFFICIENTS_UNI_DRC`: drcCoefficientsUniDrc() (Table 67)
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct DrcCoefficientsUniDrc {
    pub drc_location: u8,
    pub drc_frame_size_present: u8,
    pub drc_frame_size: u16,
    /// unsaturated value, i.e. as provided in bitstream
    pub gain_sequence_count: u8,
    /// saturated to 48
    pub gain_set_count: u8,
    /// MPEG-H: gainSequence count for each substream
    pub stream_gain_sequence_count: [u8; 4],
    /// MPEG-H: gainSet count for each substream
    pub stream_gain_set_count: [u8; 4],
    pub gain_set: [GainSet; 48],
    /* derived data */
    pub gain_set_index_for_gain_sequence: [u8; 48],
}

impl DrcCoefficientsUniDrc {
    pub const ZERO: Self = Self {
        drc_location: 0,
        drc_frame_size_present: 0,
        drc_frame_size: 0,
        gain_sequence_count: 0,
        gain_set_count: 0,
        stream_gain_sequence_count: [0; 4],
        stream_gain_set_count: [0; 4],
        gain_set: [GainSet::ZERO; 48],
        gain_set_index_for_gain_sequence: [0; 48],
    };
}

impl Default for DrcCoefficientsUniDrc {
    fn default() -> Self {
        Self::ZERO
    }
}

/// Number of `FIXP_SGL` values in the union of `gainModificationForChannelGroup[28][4]` and
/// `duckingModificationForChannel[2 * 28]`.
pub const GAIN_DUCKING_MODIFICATION_LEN: usize = 28 * 4 * 3;

/// `DRC_INSTRUCTIONS_UNI_DRC`: drcInstructionsUniDrc() (Table 72)
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct DrcInstructionsUniDrc {
    pub drc_instructions_type: u8,
    /// `mae_groupID`, union with `mae_groupPresetID`
    pub mae_group_id: u8,
    pub drc_set_id: i8,
    pub drc_set_complexity_level: u8,
    pub drc_location: u8,
    pub drc_apply_to_downmix: u8,
    pub downmix_id_count: u8,
    pub downmix_id: [u8; 8],
    pub drc_set_effect: u16,
    pub limiter_peak_target_present: u8,
    /// e = 5
    pub limiter_peak_target: FixpSgl,
    pub drc_set_target_loudness_present: u8,
    pub drc_set_target_loudness_value_upper: i8,
    pub drc_set_target_loudness_value_lower: i8,
    pub depends_on_drc_set_present: u8,
    /// `dependsOnDrcSet`, union with `UCHAR noIndependentUse`
    pub depends_on_drc_set: i8,
    pub requires_eq: u8,
    /// Union of `GAIN_MODIFICATION gainModificationForChannelGroup[28][4]` and
    /// `DUCKING_MODIFICATION duckingModificationForChannel[2 * 28]` (all members are `FIXP_SGL`).
    /// Use the accessor methods.
    pub gain_ducking_modification: [FixpSgl; GAIN_DUCKING_MODIFICATION_LEN],
    pub gain_set_index: [i8; 2 * 28],

    /* derived data */
    pub drc_channel_count: u8,
    /// MPEG-H: nDrcChannelGroups for each substream
    pub stream_n_drc_channel_groups: [u8; 4],
    pub n_drc_channel_groups: u8,
    pub gain_set_index_for_channel_group: [i8; 28],
}

impl DrcInstructionsUniDrc {
    pub const ZERO: Self = Self {
        drc_instructions_type: 0,
        mae_group_id: 0,
        drc_set_id: 0,
        drc_set_complexity_level: 0,
        drc_location: 0,
        drc_apply_to_downmix: 0,
        downmix_id_count: 0,
        downmix_id: [0; 8],
        drc_set_effect: 0,
        limiter_peak_target_present: 0,
        limiter_peak_target: 0,
        drc_set_target_loudness_present: 0,
        drc_set_target_loudness_value_upper: 0,
        drc_set_target_loudness_value_lower: 0,
        depends_on_drc_set_present: 0,
        depends_on_drc_set: 0,
        requires_eq: 0,
        gain_ducking_modification: [0; GAIN_DUCKING_MODIFICATION_LEN],
        gain_set_index: [0; 2 * 28],
        drc_channel_count: 0,
        stream_n_drc_channel_groups: [0; 4],
        n_drc_channel_groups: 0,
        gain_set_index_for_channel_group: [0; 28],
    };

    /// `mae_groupPresetID` (union with `mae_groupID`)
    #[inline]
    pub fn mae_group_preset_id(&self) -> u8 {
        self.mae_group_id
    }

    /// `noIndependentUse` (union with `dependsOnDrcSet`)
    #[inline]
    pub fn no_independent_use(&self) -> u8 {
        self.depends_on_drc_set as u8
    }

    /// `gainModificationForChannelGroup[g][b]`
    #[inline]
    pub fn gain_modification(&self, g: usize, b: usize) -> GainModification {
        let i = (g * 4 + b) * 3;
        GainModification {
            attenuation_scaling: self.gain_ducking_modification[i],
            amplification_scaling: self.gain_ducking_modification[i + 1],
            gain_offset: self.gain_ducking_modification[i + 2],
        }
    }

    /// Assignment to `gainModificationForChannelGroup[g][b]`
    #[inline]
    pub fn set_gain_modification(&mut self, g: usize, b: usize, m: GainModification) {
        let i = (g * 4 + b) * 3;
        self.gain_ducking_modification[i] = m.attenuation_scaling;
        self.gain_ducking_modification[i + 1] = m.amplification_scaling;
        self.gain_ducking_modification[i + 2] = m.gain_offset;
    }

    /// `gainModificationForChannelGroup[g]` (all 4 bands)
    #[inline]
    pub fn gain_modification_group(&self, g: usize) -> [GainModification; 4] {
        [
            self.gain_modification(g, 0),
            self.gain_modification(g, 1),
            self.gain_modification(g, 2),
            self.gain_modification(g, 3),
        ]
    }

    /// `duckingModificationForChannel[c]`
    #[inline]
    pub fn ducking_modification(&self, c: usize) -> DuckingModification {
        DuckingModification { ducking_scaling: self.gain_ducking_modification[c] }
    }

    /// Assignment to `duckingModificationForChannel[c]`
    #[inline]
    pub fn set_ducking_modification(&mut self, c: usize, m: DuckingModification) {
        self.gain_ducking_modification[c] = m.ducking_scaling;
    }

    /// `duckingModificationForChannel` as an array
    pub fn ducking_modifications(&self) -> [DuckingModification; 2 * 28] {
        let mut d = [DuckingModification::ZERO; 2 * 28];
        for (c, v) in d.iter_mut().enumerate() {
            *v = self.ducking_modification(c);
        }
        d
    }
}

impl Default for DrcInstructionsUniDrc {
    fn default() -> Self {
        Self::ZERO
    }
}

/// `CHANNEL_LAYOUT`: channelLayout() (Table 62)
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelLayout {
    pub base_channel_count: u8,
    pub layout_signaling_present: u8,
}

/// `DOWNMIX_INSTRUCTIONS`: downmixInstructions() (Table 63)
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DownmixInstructions {
    pub downmix_id: u8,
    pub target_channel_count: u8,
    pub target_layout: u8,
    pub downmix_coefficients_present: u8,
}

impl DownmixInstructions {
    pub const ZERO: Self = Self {
        downmix_id: 0,
        target_channel_count: 0,
        target_layout: 0,
        downmix_coefficients_present: 0,
    };
}

/// `UNI_DRC_CONFIG_EXTENSION`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UniDrcConfigExtension {
    pub uni_drc_config_ext_type: [u8; 8],
    pub ext_bit_size: [u32; 8 - 1],
}

/// `UNI_DRC_CONFIG`: uniDrcConfig() (Table 57)
///
/// The scratch buffer `p_scratch` of the C structure is not needed (temporary structures are
/// local variables).
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct UniDrcConfig {
    pub downmix_instructions_count_v0: u8,
    pub downmix_instructions_count_v1: u8,
    /// saturated to 32
    pub downmix_instructions_count: u8,
    pub drc_coefficients_uni_drc_count_v0: [u8; 4],
    pub drc_coefficients_uni_drc_count_v1: u8,
    /// saturated to 2
    pub drc_coefficients_uni_drc_count: u8,
    pub drc_instructions_uni_drc_count_v0: [u8; 4],
    pub drc_instructions_uni_drc_count_v1: u8,
    /// saturated to (32 + 1)
    pub drc_instructions_uni_drc_count: u8,
    /// MPEG-H: base channel count for each substream
    pub stream_channel_count: [u8; 4],
    pub channel_layout: ChannelLayout,
    pub downmix_instructions: [DownmixInstructions; 32],
    pub drc_coefficients_uni_drc: [DrcCoefficientsUniDrc; 2],
    pub drc_instructions_uni_drc: [DrcInstructionsUniDrc; 32 + 1],
    pub loudness_info_set_present: u8,
    pub uni_drc_config_ext_present: u8,
    pub uni_drc_config_ext: UniDrcConfigExtension,

    /* derived data */
    pub drc_instructions_count_incl_virtual: u8,
    pub diff: u8,
}

impl UniDrcConfig {
    pub const ZERO: Self = Self {
        downmix_instructions_count_v0: 0,
        downmix_instructions_count_v1: 0,
        downmix_instructions_count: 0,
        drc_coefficients_uni_drc_count_v0: [0; 4],
        drc_coefficients_uni_drc_count_v1: 0,
        drc_coefficients_uni_drc_count: 0,
        drc_instructions_uni_drc_count_v0: [0; 4],
        drc_instructions_uni_drc_count_v1: 0,
        drc_instructions_uni_drc_count: 0,
        stream_channel_count: [0; 4],
        channel_layout: ChannelLayout { base_channel_count: 0, layout_signaling_present: 0 },
        downmix_instructions: [DownmixInstructions::ZERO; 32],
        drc_coefficients_uni_drc: [DrcCoefficientsUniDrc::ZERO; 2],
        drc_instructions_uni_drc: [DrcInstructionsUniDrc::ZERO; 32 + 1],
        loudness_info_set_present: 0,
        uni_drc_config_ext_present: 0,
        uni_drc_config_ext: UniDrcConfigExtension {
            uni_drc_config_ext_type: [0; 8],
            ext_bit_size: [0; 7],
        },
        drc_instructions_count_incl_virtual: 0,
        diff: 0,
    };
}

impl Default for UniDrcConfig {
    fn default() -> Self {
        Self::ZERO
    }
}

/*******************/
/* loudnessInfoSet */
/*******************/

/// `METHOD_DEFINITION`
pub type MethodDefinition = i32;
/// `MD_UNKNOWN_OTHER`
pub const MD_UNKNOWN_OTHER: MethodDefinition = 0;
/// `MD_PROGRAM_LOUDNESS`
pub const MD_PROGRAM_LOUDNESS: MethodDefinition = 1;
/// `MD_ANCHOR_LOUDNESS`
pub const MD_ANCHOR_LOUDNESS: MethodDefinition = 2;
/// `MD_MAX_OF_LOUDNESS_RANGE`
pub const MD_MAX_OF_LOUDNESS_RANGE: MethodDefinition = 3;
/// `MD_MOMENTARY_LOUDNESS_MAX`
pub const MD_MOMENTARY_LOUDNESS_MAX: MethodDefinition = 4;
/// `MD_SHORT_TERM_LOUDNESS_MAX`
pub const MD_SHORT_TERM_LOUDNESS_MAX: MethodDefinition = 5;
/// `MD_LOUDNESS_RANGE`
pub const MD_LOUDNESS_RANGE: MethodDefinition = 6;
/// `MD_MIXING_LEVEL`
pub const MD_MIXING_LEVEL: MethodDefinition = 7;
/// `MD_ROOM_TYPE`
pub const MD_ROOM_TYPE: MethodDefinition = 8;
/// `MD_SHORT_TERM_LOUDNESS`
pub const MD_SHORT_TERM_LOUDNESS: MethodDefinition = 9;

/// `MEASUREMENT_SYSTEM`
pub type MeasurementSystem = i32;
/// `MS_UNKNOWN_OTHER`
pub const MS_UNKNOWN_OTHER: MeasurementSystem = 0;
/// `MS_EBU_R_128`
pub const MS_EBU_R_128: MeasurementSystem = 1;
/// `MS_BS_1770_4`
pub const MS_BS_1770_4: MeasurementSystem = 2;
/// `MS_BS_1770_4_PRE_PROCESSING`
pub const MS_BS_1770_4_PRE_PROCESSING: MeasurementSystem = 3;
/// `MS_USER`
pub const MS_USER: MeasurementSystem = 4;
/// `MS_EXPERT_PANEL`
pub const MS_EXPERT_PANEL: MeasurementSystem = 5;
/// `MS_BS_1771_1`
pub const MS_BS_1771_1: MeasurementSystem = 6;
/// `MS_RESERVED_A`
pub const MS_RESERVED_A: MeasurementSystem = 7;
/// `MS_RESERVED_B`
pub const MS_RESERVED_B: MeasurementSystem = 8;
/// `MS_RESERVED_C`
pub const MS_RESERVED_C: MeasurementSystem = 9;
/// `MS_RESERVED_D`
pub const MS_RESERVED_D: MeasurementSystem = 10;
/// `MS_RESERVED_E`
pub const MS_RESERVED_E: MeasurementSystem = 11;

/// `RELIABILITY`
pub type Reliability = i32;
/// `R_UKNOWN`
pub const R_UKNOWN: Reliability = 0;
/// `R_UNVERIFIED`
pub const R_UNVERIFIED: Reliability = 1;
/// `R_CEILING`
pub const R_CEILING: Reliability = 2;
/// `R_ACCURATE`
pub const R_ACCURATE: Reliability = 3;

/// `LOUDNESS_MEASUREMENT`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LoudnessMeasurement {
    /// e = 7 for all methodDefinitions
    pub method_value: FixpDbl,
    pub method_definition: u8,
    pub measurement_system: u8,
    pub reliability: u8,
}

impl LoudnessMeasurement {
    pub const ZERO: Self =
        Self { method_value: 0, method_definition: 0, measurement_system: 0, reliability: 0 };
}

/// `LOUDNESS_INFO`: loudnessInfo() (Table 59)
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LoudnessInfo {
    pub loudness_info_type: u8,
    /// `mae_groupID`, union with `mae_groupPresetID`
    pub mae_group_id: u8,
    pub drc_set_id: i8,
    pub downmix_id: u8,
    pub sample_peak_level_present: u8,
    pub true_peak_level_present: u8,
    /// saturated to 16
    pub measurement_count: u8,
    /// e = 7
    pub sample_peak_level: FixpDbl,
    /// e = 7
    pub true_peak_level: FixpDbl,
    pub loudness_measurement: [LoudnessMeasurement; 16],
}

impl LoudnessInfo {
    pub const ZERO: Self = Self {
        loudness_info_type: 0,
        mae_group_id: 0,
        drc_set_id: 0,
        downmix_id: 0,
        sample_peak_level_present: 0,
        true_peak_level_present: 0,
        measurement_count: 0,
        sample_peak_level: 0,
        true_peak_level: 0,
        loudness_measurement: [LoudnessMeasurement::ZERO; 16],
    };

    /// `mae_groupPresetID` (union with `mae_groupID`)
    #[inline]
    pub fn mae_group_preset_id(&self) -> u8 {
        self.mae_group_id
    }
}

/// `LOUDNESS_INFO_SET_EXTENSION`: loudnessInfoSetExtension() (Table 61)
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LoudnessInfoSetExtension {
    pub loudness_info_set_ext_type: [u8; 8],
    pub ext_bit_size: [u32; 8 - 1],
}

/// `LOUDNESS_INFO_SET`: loudnessInfoSet() (Table 58)
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct LoudnessInfoSet {
    pub loudness_info_album_count_v0: [u8; 4],
    pub loudness_info_album_count_v1: u8,
    /// saturated to 32
    pub loudness_info_album_count: u8,
    pub loudness_info_count_v0: [u8; 4],
    pub loudness_info_count_v1: u8,
    /// saturated to 32
    pub loudness_info_count: u8,
    pub loudness_info_album: [LoudnessInfo; 32],
    pub loudness_info: [LoudnessInfo; 32],
    pub loudness_info_set_ext_present: u8,
    pub loudness_info_set_ext: LoudnessInfoSetExtension,
    /* derived data */
    pub diff: u8,
}

impl LoudnessInfoSet {
    pub const ZERO: Self = Self {
        loudness_info_album_count_v0: [0; 4],
        loudness_info_album_count_v1: 0,
        loudness_info_album_count: 0,
        loudness_info_count_v0: [0; 4],
        loudness_info_count_v1: 0,
        loudness_info_count: 0,
        loudness_info_album: [LoudnessInfo::ZERO; 32],
        loudness_info: [LoudnessInfo::ZERO; 32],
        loudness_info_set_ext_present: 0,
        loudness_info_set_ext: LoudnessInfoSetExtension {
            loudness_info_set_ext_type: [0; 8],
            ext_bit_size: [0; 7],
        },
        diff: 0,
    };
}

impl Default for LoudnessInfoSet {
    fn default() -> Self {
        Self::ZERO
    }
}
