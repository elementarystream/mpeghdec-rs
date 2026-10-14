// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_formatConverter_intern.h.

//! Internal data types of the format converter: parameters, state and the internal instance.

use mpeghdec_fdk::digest::StateDigest;

use crate::format_converter_active_dmx_stft::ActiveDownmixer;
use crate::format_converter_lib::*;
use mpeghdec_fdk::cicp2geometry::Cicp2GeometryChannelGeometry;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::stftfilterbank::{HandleStftFilterbank, StftFilterbankConfig};

pub const FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNEL_GROUPS: usize = 28;
pub const FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS: usize = 28;
pub const FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS: usize = 24;

/* Converter parameters */
pub const IN_OUT_N: usize = 100;
pub const MAXBANDS: usize = 58;
pub const NCHANOUT_MAX: usize = FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS;
pub const NCHANIN_MAX: usize = FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS;
pub const FDK_FORMAT_CONVERTER_MAX_LFE: usize = FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS;

/* Downmix Module data types and definitions */

/// `FIXP_DMX_H`: data type for downmix matrix coefficients.
pub type FixpDmxH = FixpSgl;
pub const MAXVAL_DMX_H: FixpDmxH = MAXVAL_SGL as FixpDmxH;
pub const MINVAL_DMX_H: FixpDmxH = MINVAL_SGL as FixpDmxH;

/// `FX_DBL2FX_DMX_H(x)`
#[inline(always)]
pub const fn fx_dbl2fx_dmx_h(x: FixpDbl) -> FixpDmxH {
    fx_dbl2fx_sgl(x)
}

/// `FX_DMX_H2FX_DBL(x)`
#[inline(always)]
pub const fn fx_dmx_h2fx_dbl(x: FixpDmxH) -> FixpDbl {
    fx_sgl2fx_dbl(x)
}

/* Data type for equalizer coefficients */

pub const FIXP_0_DOT_5: FixpDbl = 0x4000_0000;
pub const FIXP_0_DOT_25: FixpDbl = 0x2000_0000;
/// `FIXP_EQ_H`: data type for equalizer coefficients.
pub type FixpEqH = FixpDbl;
pub const MAXVAL_EQ_H: FixpEqH = MAXVAL_DBL;
pub const MINVAL_EQ_H: FixpEqH = MINVAL_DBL;

/// default EQs exponent
pub const EQ_H_EXP: i32 = 1;
pub const FIXP_EQ_H_FORMAT_1_DOT_0: FixpEqH = FIXP_0_DOT_5;
/// bitstream parsed EQs exponent
pub const EQ_BITSTREAM_H_EXP: i32 = 2;
pub const FIXP_EQ_BITSTREAM_H_FORMAT_1_DOT_0: FixpEqH = FIXP_0_DOT_25;

/// `DMXH_PCM`: data type for downmix input/output data.
pub type DmxhPcm = FixpDbl;
/// PCM output headroom, must have the same values as PCM_OUT_HEADROOM
pub const DMXH_HEADROOM: i32 = 8;

/// `FX_PI`
pub const FX_PI: FixpDbl = fl2fxconst_dbl(core::f64::consts::PI / 4.0);

/* downmix processing rules (converter_dmxrulesid_t) */

/* external rules (indices 0..10) */
/// no processing channel copy with gain
pub const RULE_NOPROC: i32 = 0;
/// eq for front up-median downmix
pub const RULE_EQ1: i32 = 1;
/// eq for surround up-median downmix
pub const RULE_EQ2: i32 = 2;
/// eq for top-up downmix
pub const RULE_EQ3: i32 = 3;
/// eq for top-median downmix
pub const RULE_EQ4: i32 = 4;
/// eq for horizontal channel that is displaced to height
pub const RULE_EQ5: i32 = 5;
/* New equalizers */
/// eq for CH_U_L030, CH_U_R030, CH_U_L045, CH_U_R045
pub const RULE_EQVF: i32 = 7;
/// eq for CH_U_L135, CH_U_R135
pub const RULE_EQVB: i32 = 8;
/// eq for CH_U_000
pub const RULE_EQVFC: i32 = 9;
/// eq for CH_U_180
pub const RULE_EQVBC: i32 = 10;
/// eq for CH_T_000
pub const RULE_EQVOG: i32 = 11;
/// eq for CH_U_L090, CH_U_R110
pub const RULE_EQVS: i32 = 12;
/// eq for CH_L_000, CH_L_L045, CH_L_R045
pub const RULE_EQBTM: i32 = 13;
/// eq for CH_U_L110, CH_U_R110
pub const RULE_EQVBA: i32 = 14;
/// eq for CH_M_L060 and CH_M_R060 to CH_M_L030 and CH_M_R030
pub const RULE_06_03: i32 = 15;
/// eq for CH_M_L090 and CH_M_R090 to CH_M_L030 and CH_M_R030
pub const RULE_09_03: i32 = 16;
/// eq for CH_M_L060 and CH_M_R060 to CH_M_L110 and CH_M_R110
pub const RULE_06_11: i32 = 17;
/// eq for CH_M_L090 and CH_M_R090 to CH_M_L110 and CH_M_R110
pub const RULE_09_11: i32 = 18;
/// eq for CH_M_L135 and CH_M_R135 to CH_M_L110 and CH_M_R110
pub const RULE_13_11: i32 = 19;
/// eq for CH_M_180 to CH_M_L110 and CH_M_R110
pub const RULE_18_11: i32 = 20;
/// first eq for random setups
pub const RULE_REQ: i32 = 21;
/// number of eqs, the value of N_EQ is 40 in case of randomization, otherwise 20
pub const N_EQ: usize = 40;
/* rules only used within converter only */
/// manual amplitude panning (specifying alpha0, alpha)
pub const RULE_PANNING: i32 = 100;
/// top channel to all upper channels
pub const RULE_TOP2ALLU: i32 = 101;
/// top channel to all horizontal channels
pub const RULE_TOP2ALLM: i32 = 102;
/// automatic amplitude panning (alpha0 defined by destination channels, alpha defined by source
/// channels)
pub const RULE_AUTOPAN: i32 = 103;
pub const RULE_VIRTUAL: i32 = 104;

/// `converter_pr_t`
#[derive(StateDigest, Clone, Debug)]
pub struct ConverterPr {
    /// equalizers [lin gains]
    pub eq: [[FixpEqH; MAXBANDS]; N_EQ],
}

impl Default for ConverterPr {
    fn default() -> Self {
        ConverterPr { eq: [[0; MAXBANDS]; N_EQ] }
    }
}

/// `converter_pr_tmp_t`
///
/// The arrays have two spare entries: `converter_init()` may write one entry past `IN_OUT_N` in
/// the panning rules before it detects the overflow and fails (in C this write lands in the
/// adjacent array; the result is discarded in both cases).
///
/// In C the struct lives in an uninitialized scratch buffer. `converter_init()` writes all
/// entries up to the end marker, except for an output setup without channels, where entries
/// before the end marker may remain stale. The port initializes the sources with the end marker
/// (-1), so such a matrix is empty.
#[derive(StateDigest, Clone, Debug)]
pub struct ConverterPrTmp {
    /// in-out conversion source
    pub in_out_src: [i8; IN_OUT_N + 2],
    /// in-out conversion destination
    pub in_out_dst: [i8; IN_OUT_N + 2],
    /// in-out conversion gain \[lin\]
    pub in_out_gain: [FixpDmxH; IN_OUT_N + 2],
    /// in-out conversion processing
    pub in_out_proc: [i8; IN_OUT_N + 2],
    /// in-out conversion gain, G_vL \[lin\]
    pub in_out_gain_l: [FixpDmxH; IN_OUT_N + 2],
    /* variables for the timbral elevation rendering */
    /// in-out conversion source
    pub in_out_src2: [i8; IN_OUT_N + 2],
    /// in-out conversion destination
    pub in_out_dst2: [i8; IN_OUT_N + 2],
    /// in-out conversion gain \[lin\]
    pub in_out_gain2: [FixpDmxH; IN_OUT_N + 2],
    /// in-out conversion processing
    pub in_out_proc2: [i8; IN_OUT_N + 2],
}

impl Default for ConverterPrTmp {
    fn default() -> Self {
        ConverterPrTmp {
            in_out_src: [-1; IN_OUT_N + 2],
            in_out_dst: [0; IN_OUT_N + 2],
            in_out_gain: [0; IN_OUT_N + 2],
            in_out_proc: [0; IN_OUT_N + 2],
            in_out_gain_l: [0; IN_OUT_N + 2],
            in_out_src2: [-1; IN_OUT_N + 2],
            in_out_dst2: [0; IN_OUT_N + 2],
            in_out_gain2: [0; IN_OUT_N + 2],
            in_out_proc2: [0; IN_OUT_N + 2],
        }
    }
}

/* INDEX : FOR THE CONVENIENCE FOR ONLY HEIGHT CHANNELS (enum H) */
pub const TFC: usize = 0;
pub const TFL: usize = 1;
pub const TFR: usize = 2;
pub const TFLA: usize = 3;
pub const TFRA: usize = 4;
pub const TSL: usize = 5;
pub const TSR: usize = 6;
pub const TBLA: usize = 7;
pub const TBRA: usize = 8;
pub const TBL: usize = 9;
pub const TBR: usize = 10;
pub const TBC: usize = 11;
pub const VOG: usize = 12;

/* INDEX : FOR THE 5.1 CHANNEL SYSTEM (enum C) */
pub const FL: usize = 0;
pub const FR: usize = 1;
pub const FC: usize = 2;
pub const SW: usize = 3;
pub const SL: usize = 4;
pub const SR: usize = 5;

/*
IMPORTANT/WARNING:
    -the converter implementation assumes that all horizontal channels are:
        CH_M_000 <= i <= CH_M_180
    -the converter implementation assumes that all height channels are:
        CH_U_000 <= i <= CH_U_180
    Should this be changed, then converter.c needs to be modified!
*/

/// `AFC_FORMAT_CONVERTER_CHANNEL_ID`
pub type AfcFormatConverterChannelId = i32;
/* horizontal channels */
pub const CH_M_000: i32 = 0;
pub const CH_M_L022: i32 = 1;
pub const CH_M_R022: i32 = 2;
pub const CH_M_L030: i32 = 3;
pub const CH_M_R030: i32 = 4;
pub const CH_M_L045: i32 = 5;
pub const CH_M_R045: i32 = 6;
pub const CH_M_L060: i32 = 7;
pub const CH_M_R060: i32 = 8;
pub const CH_M_L090: i32 = 9;
pub const CH_M_R090: i32 = 10;
pub const CH_M_L110: i32 = 11;
pub const CH_M_R110: i32 = 12;
pub const CH_M_L135: i32 = 13;
pub const CH_M_R135: i32 = 14;
pub const CH_M_L150: i32 = 15;
pub const CH_M_R150: i32 = 16;
pub const CH_M_180: i32 = 17;
/* height channels */
pub const CH_U_000: i32 = 18;
pub const CH_U_L045: i32 = 19;
pub const CH_U_R045: i32 = 20;
pub const CH_U_L030: i32 = 21;
pub const CH_U_R030: i32 = 22;
pub const CH_U_L090: i32 = 23;
pub const CH_U_R090: i32 = 24;
pub const CH_U_L110: i32 = 25;
pub const CH_U_R110: i32 = 26;
pub const CH_U_L135: i32 = 27;
pub const CH_U_R135: i32 = 28;
pub const CH_U_180: i32 = 29;
/* top channel */
pub const CH_T_000: i32 = 30;
/* low channels */
pub const CH_L_000: i32 = 31;
pub const CH_L_L045: i32 = 32;
pub const CH_L_R045: i32 = 33;
/* low frequency effects */
pub const CH_LFE1: i32 = 34;
pub const CH_LFE2: i32 = 35;
pub const CH_LFE3: i32 = 36;
/* SCR */
pub const CH_M_LSCR: i32 = 37;
pub const CH_M_RSCR: i32 = 38;
/* empty channel */
pub const CH_EMPTY: i32 = -1;

/// `FDK_FORMAT_CONVERTER_INPUT_FORMAT`
pub type FdkFormatConverterInputFormat = i32;
pub const FDK_FORMAT_CONVERTER_INPUT_FORMAT_INVALID: i32 = 0;
pub const FDK_FORMAT_CONVERTER_INPUT_FORMAT_LISTOFCHANNELS: i32 = 1;
pub const FDK_FORMAT_CONVERTER_INPUT_FORMAT_GENERIC: i32 = 2;

/// `FDK_FORMAT_CONVERTER_OUTPUT_FORMAT`
pub type FdkFormatConverterOutputFormat = i32;
pub const FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_INVALID: i32 = 0;
pub const FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_LISTOFCHANNELS: i32 = 1;
pub const FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC: i32 = 2;

/// `FDK_converter_formatid_t` (`FDK_tools_rom.h`), the values used by the format converter.
pub type FdkConverterFormatid = i32;
pub const FDK_FORMAT_IN_LISTOFCHANNELS: i32 = 22;
pub const FDK_FORMAT_OUT_LISTOFCHANNELS: i32 = 23;
pub const FDK_NFORMATS: i32 = 24;

/// Selects the downmix matrix a `FIXP_DMX_H*` of `FORMAT_CONVERTER_PARAMS` points to
/// (`dmxMatrixL_FDK`, `dmxMatrixH_FDK`).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DmxMtxSel {
    /// `NULL`
    #[default]
    None,
    /// `dmxMtx_sorted`
    Mtx,
    /// `dmxMtxL_sorted`
    MtxL,
    /// `dmxMtx2_sorted`
    Mtx2,
}

/// Selects the equalizer index vector `eqIndex_FDK` points to.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EqIndexSel {
    /// `NULL`
    #[default]
    None,
    /// `eqIndexVec_sorted`
    Vec,
    /// `eqIndexVec2_sorted`
    Vec2,
}

/// `FORMAT_CONVERTER_PARAMS` (`struct T_FORMAT_CONVERTER_PARAMS`)
#[derive(StateDigest, Clone, Debug, Default)]
pub struct FormatConverterParams {
    /// Sampling rate (never set).
    pub sampling_rate: i32,

    /// Needs to be set, if the IIS_FORMATCONVERTER_MODE ==
    /// IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN (never set).
    pub phase_align_strength: FixpDbl,
    pub adaptive_eq_strength: FixpDbl,

    /// format converter input format of internal init structure
    pub format_converter_input_format_internal: FdkConverterFormatid,
    /// format converter output format of internal init structure
    pub format_converter_output_format_internal: FdkConverterFormatid,

    /// internal init structure
    pub format_converter_params_internal: Option<Box<ConverterPr>>,

    /// flag signalling generic input,output formats
    pub generic_io_fmt: i32,

    /// dmx matrix \[in\]\[out\]
    pub dmx_mtx: Vec<Vec<FixpDmxH>>,
    pub dmx_mtx_sorted: Vec<FixpDmxH>,
    /// equalizer index vector \[in\]\[out\]
    pub eq_index_vec: Vec<Vec<i32>>,
    pub eq_index_vec_sorted: Vec<i32>,

    /// dmx matrix L
    pub dmx_mtx_l: Vec<Vec<FixpDmxH>>,
    pub dmx_mtx_l_sorted: Vec<FixpDmxH>,
    /// dmx matrix 2
    pub dmx_mtx2: Vec<Vec<FixpDmxH>>,
    pub dmx_mtx2_sorted: Vec<FixpDmxH>,
    /// equalizer index vector 2 \[in\]\[out\]
    pub eq_index_vec2: Vec<Vec<i32>>,
    pub eq_index_vec2_sorted: Vec<i32>,

    /* Immersive mode control */
    pub immersive_mode: i32,
    pub mode3_drendering: i32,
    pub mode2_drendering: i32,
    pub rendering3_d_type_flag_internal: i32,

    pub ch_out_count: [i32; NCHANOUT_MAX],
    pub ch_out_exp: [i32; NCHANOUT_MAX],
    pub dmx_iterations: i32,

    pub dmx_matrix_l_fdk: DmxMtxSel,
    pub dmx_matrix_h_fdk: DmxMtxSel,
    pub eq_index_fdk: EqIndexSel,

    /// flag signalling the DMX matrix coefficients have been set
    pub dmx_mtx_is_set: i32,

    /// formatConverterDelay
    pub format_converter_delay: i32,

    /// center frequencies
    pub center_frequencies_normalized: Option<&'static [FixpDbl]>,
}

impl FormatConverterParams {
    /// The sorted downmix matrix selected by `sel`.
    pub fn dmx_sorted(&self, sel: DmxMtxSel) -> &[FixpDmxH] {
        match sel {
            DmxMtxSel::None => &[],
            DmxMtxSel::Mtx => &self.dmx_mtx_sorted,
            DmxMtxSel::MtxL => &self.dmx_mtx_l_sorted,
            DmxMtxSel::Mtx2 => &self.dmx_mtx2_sorted,
        }
    }

    /// The sorted equalizer index vector selected by `sel`.
    pub fn eq_index_sorted(&self, sel: EqIndexSel) -> &[i32] {
        match sel {
            EqIndexSel::None => &[],
            EqIndexSel::Vec => &self.eq_index_vec_sorted,
            EqIndexSel::Vec2 => &self.eq_index_vec2_sorted,
        }
    }
}

/// `FORMAT_CONVERTER_STATE` (`struct T_FORMAT_CONVERTER_STATE`)
#[derive(StateDigest, Clone, Debug, Default)]
pub struct FormatConverterState {
    /// handle for params and states of active dmx in stft domain
    pub handle_active_dmx_stft: Option<Box<ActiveDownmixer>>,
}

/// `IIS_FORMATCONVERTER_INTERNAL`
#[derive(StateDigest, Clone, Debug)]
pub struct IisFormatConverterInternal {
    pub fc_num_freq_bands: u32,
    /// `fcCenterFrequencies` (`None` for `NULL`)
    pub fc_center_frequencies: Option<&'static [FixpDbl]>,

    pub input_channel_geo: [Cicp2GeometryChannelGeometry; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS],

    pub num_input_channels: [u32; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNEL_GROUPS],
    pub num_input_channel_groups: u32,

    pub amount_of_added_dmx_matrices_and_equalizers: u32,
    /// EqualizerConfig variables. Parsed and decoded bitstream EQs (`STFT_ERB_BANDS` values each,
    /// `None` for `NULL`).
    pub eq_gains: [Option<Vec<FixpEqH>>; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS],

    pub num_total_input_channels: u32,

    pub frame_size: u32,
    pub stft_headroom_prescaling: i32,
    pub num_time_slots: u32,
    pub sampling_rate: u32,

    pub immersive_downmix_flag: u32,
    pub rendering3_d_type_flag: u32,

    /* Candidates */
    pub mode: IisFormatconverterMode,
    pub fc_input_format: FdkFormatConverterInputFormat,
    pub fc_output_format: FdkFormatConverterOutputFormat,
    pub aes: i32,
    pub num_output_channels: u32,
    pub out_channel_geo: [Cicp2GeometryChannelGeometry; FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS],
    pub cicp_layout_index: i32,
    pub out_channel_vbap_flag: u32,
    pub in_channel_vbap_flag: u32,

    pub fc_params: Option<Box<FormatConverterParams>>,
    pub fc_state: Option<Box<FormatConverterState>>,

    /* Buffers and handles for STFT processing */
    /// real, imag interleaved, N/2+1 coefficient stored in imaginary part of first coefficient ->
    /// stft of length N results in N interleaved coefficients (empty for `NULL`)
    pub input_buffer_stft: Vec<Vec<FixpDbl>>,
    pub prev_input_buffer_stft: Vec<Vec<FixpDbl>>,
    pub output_buffer_stft: Vec<Vec<FixpDbl>>,
    /// Spectra of the time slots but the last one of a frame for the parallel processing (not in
    /// C, see `iis_format_converter_process_stft_par()`): `[slot][channel]`.
    pub slot_input_buffer_stft: Vec<Vec<Vec<FixpDbl>>>,
    pub slot_output_buffer_stft: Vec<Vec<Vec<FixpDbl>>>,
    pub stft_filterbank_analysis:
        [Option<HandleStftFilterbank>; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNEL_GROUPS],
    pub stft_filterbank_synthesis:
        [Option<HandleStftFilterbank>; FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS],

    pub stft_filterbank_config_analysis: StftFilterbankConfig,
    pub stft_filterbank_config_synthesis: StftFilterbankConfig,
    pub stft_frame_size: u32,
    pub stft_length: u32,
    pub stft_num_erb_bands: u32,
    pub format_in_list_of_channels:
        [AfcFormatConverterChannelId; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS],
    pub format_in_list_of_channels_nchan: i32,
    pub format_out_list_of_channels:
        [AfcFormatConverterChannelId; FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS],
    pub format_out_list_of_channels_nchan: i32,

    pub gvh: [[FixpDbl; 6]; 13],
    pub gvh_e: [[i32; 6]; 13],
    pub gvl: [[FixpDbl; 6]; 13],
    pub gvl_e: [[i32; 6]; 13],

    pub is4_gvh_stft_erb: [u32; 58],
    pub top_in: [i32; 13],
    pub mid_out: [u32; 6],

    pub erb_is4_gvh_l: u32,
    pub erb_is4_gvh_h: u32,

    pub open_success: i32,

    /// Fast mode (not in C, see [`crate::format_converter_fast`]): the channel mapping of the
    /// previous frame if it was processed by the time domain bypass.
    pub fast_bypass: Option<Box<crate::format_converter_fast::BypassMap>>,
    /// Fast mode (not in C): number of frames processed by the time domain bypass. Only
    /// reported, not part of the state digest.
    #[digest(skip)]
    pub fast_bypass_frames: u64,
}

impl Default for IisFormatConverterInternal {
    /// All fields zero, like `FDKcalloc()`.
    fn default() -> Self {
        IisFormatConverterInternal {
            fc_num_freq_bands: 0,
            fc_center_frequencies: None,
            input_channel_geo: [Cicp2GeometryChannelGeometry::default();
                FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS],
            num_input_channels: [0; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNEL_GROUPS],
            num_input_channel_groups: 0,
            amount_of_added_dmx_matrices_and_equalizers: 0,
            eq_gains: Default::default(),
            num_total_input_channels: 0,
            frame_size: 0,
            stft_headroom_prescaling: 0,
            num_time_slots: 0,
            sampling_rate: 0,
            immersive_downmix_flag: 0,
            rendering3_d_type_flag: 0,
            mode: IIS_FORMATCONVERTER_MODE_INVALID,
            fc_input_format: 0,
            fc_output_format: 0,
            aes: 0,
            num_output_channels: 0,
            out_channel_geo: [Cicp2GeometryChannelGeometry::default();
                FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS],
            cicp_layout_index: 0,
            out_channel_vbap_flag: 0,
            in_channel_vbap_flag: 0,
            fc_params: None,
            fc_state: None,
            input_buffer_stft: Vec::new(),
            prev_input_buffer_stft: Vec::new(),
            output_buffer_stft: Vec::new(),
            slot_input_buffer_stft: Vec::new(),
            slot_output_buffer_stft: Vec::new(),
            stft_filterbank_analysis: Default::default(),
            stft_filterbank_synthesis: Default::default(),
            stft_filterbank_config_analysis: StftFilterbankConfig::default(),
            stft_filterbank_config_synthesis: StftFilterbankConfig::default(),
            stft_frame_size: 0,
            stft_length: 0,
            stft_num_erb_bands: 0,
            format_in_list_of_channels: [0; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS],
            format_in_list_of_channels_nchan: 0,
            format_out_list_of_channels: [0; FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS],
            format_out_list_of_channels_nchan: 0,
            gvh: [[0; 6]; 13],
            gvh_e: [[0; 6]; 13],
            gvl: [[0; 6]; 13],
            gvl_e: [[0; 6]; 13],
            is4_gvh_stft_erb: [0; 58],
            top_in: [0; 13],
            mid_out: [0; 6],
            erb_is4_gvh_l: 0,
            erb_is4_gvh_h: 0,
            open_success: 0,
            fast_bypass: None,
            fast_bypass_frames: 0,
        }
    }
}

pub const MAX_CHANNELS: u32 = 33;
pub const HOPSIZE: u32 = 32;

/// For memory allocation of temporary vectors that are used in the processing
pub const NUMTEMPVECTORS: u32 = 4;

/// Used as denumerator minimum instead of division by zero
pub const AFC_H_EPSILON: FixpDbl = 1;
