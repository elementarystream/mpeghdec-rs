// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/aacdecoder.h and libMpeghDec/src/aacdecoder.cpp.

//! Frame level of the core decoder: decoder instance, configuration (`CAacDecoder_Init()`),
//! access unit decoding (`CAacDecoder_DecodeFrame()`), pre-roll parsing and the earcon (PCM data)
//! mixer.
//!
//! The functions that need the transport decoder (`hInput`) or the configs it holds (`ascs`, to
//! resolve [`AacDecoderInstance::p_usac_config`]) take them as parameters; see the
//! [crate documentation](crate), "Frame level".

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_ram::*;
use crate::aac_rom::PcmDec;
use crate::aacdecoder_lib::*;
use crate::aacdecoder_par::FreqToTimeOutcome;
use crate::block::cblock_frequency_to_time;
use crate::channel::{cchannel_element_decode_pool, cchannel_element_read, CChannelElementMem};
use crate::channelinfo::*;
use crate::conceal::*;
use crate::conceal_types::*;
use crate::delay::{fdk_delay_create, fdk_delay_destroy};
use crate::format_converter_glue::*;
use crate::ltp_post::ltp_post;
use crate::mct::*;
use crate::overlapadd::OVERLAP_BUFFER_SIZE;
use crate::pipeline::{Pending, RenderWorker};
use crate::render::*;
use crate::rs;
use mpeghdec_arith::ac_arith_coder::CArcoData;
use mpeghdec_drc::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::*;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::*;
use mpeghdec_fdk::mdct::{mdct_init, PCM_OUT_HEADROOM};
use mpeghdec_fdk::stftfilterbank::*;
use mpeghdec_fdk::td_upsampler::*;
use mpeghdec_fdk::thread_pool::ThreadPool;
use mpeghdec_formatconverter::{
    decode_downmix_matrix, format_converter_dmx_matrix_exponent, format_converter_set_eqs,
    EqConfigStruct, SpeakerInformation, FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS,
    FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS,
};
use mpeghdec_gvbap::*;
use mpeghdec_igf::igf_dec::*;
use mpeghdec_tpdec::*;
use mpeghdec_uimanager::*;
use std::collections::VecDeque;
use std::sync::Arc;

/// `TIME_DATA_FLUSH_SIZE`
pub const TIME_DATA_FLUSH_SIZE: i32 = 128;
/// `TIME_DATA_FLUSH_SIZE_SF`
pub const TIME_DATA_FLUSH_SIZE_SF: i32 = 7;
/// `AACDEC_MAX_NUM_PREROLL_AU_MPEGH`
pub const AACDEC_MAX_NUM_PREROLL_AU_MPEGH: u32 = 1;
/// `AACDEC_MAX_NUM_PREROLL_AU`
pub const AACDEC_MAX_NUM_PREROLL_AU: usize = AACDEC_MAX_NUM_PREROLL_AU_MPEGH as usize;

/// `TDL_MPEGH3DA_DEFAULT_ATTACK`: default attack time in ms for peak limiter
pub const TDL_MPEGH3DA_DEFAULT_ATTACK: u32 = 5;
/// `TDL_MPEGH3DA_DEFAULT_THRESHOLD`: default threshold (-1 dBFS) for peak limiter
pub const TDL_MPEGH3DA_DEFAULT_THRESHOLD: FixpDbl = fl2fxconst_dbl(0.89125094f32 as f64);

/// Size of the element tables (`(3 * ((28) * 2) + (((28) * 2)) / 2 + 4 * (1) + 1)`).
pub const AAC_MAX_ELEMENTS: usize = 3 * (28 * 2) + (28 * 2) / 2 + 4 + 1;
/// `((28) * 2)`: maximum number of transport channels (signals).
pub const AAC_MAX_SIGNALS: usize = 28 * 2;
/// `(24)`: maximum number of rendered output channels.
pub const AAC_MAX_OUTPUT_CHANNELS: usize = 24;

/* flushStatus */
pub const AACDEC_FLUSH_OFF: u8 = 0;
pub const AACDEC_MPEGH_CFG_CHANGE_ATSC_FLUSH_ON: u8 = 1;
pub const AACDEC_MPEGH_DASH_IPF_ATSC_FLUSH_ON: u8 = 2;
pub const AACDEC_USAC_DASH_IPF_FLUSH_ON: u8 = 3;

/* buildUpStatus */
pub const AACDEC_BUILD_UP_OFF: u8 = 0;
pub const AACDEC_MPEGH_BUILD_UP_ON: u8 = 1;
pub const AACDEC_MPEGH_BUILD_UP_ON_IN_BAND: u8 = 2;
pub const AACDEC_USAC_BUILD_UP_ON: u8 = 3;
pub const AACDEC_MPEGH_BUILD_UP_IDLE: u8 = 4;
pub const AACDEC_MPEGH_BUILD_UP_IDLE_IN_BAND: u8 = 5;

/// No cross-fade between frames shall be applied at next config change.
pub const AACDEC_CROSSFADE_BITMASK_OFF: u8 = 0;
/// Cross-fade is needed to switch channel at next config change when applying
/// mpegh3daElementInteraction from USERINTERACTION (UI) packet.
pub const AACDEC_CROSSFADE_BITMASK_UI: u8 = 1 << 0;
/// applyCrossfade is signaled in AudioPreRoll.
pub const AACDEC_CROSSFADE_BITMASK_PREROLL: u8 = 1 << 1;
/// Cross-fade at config change is demanded by `pCtrlCFGChangeStruct->forceCrossfade`.
pub const AACDEC_CROSSFADE_BITMASK_FORCE: u8 = 1 << 2;

/// `TRUNC_QUEUE_SIZE`
pub const TRUNC_QUEUE_SIZE: usize = 3;

/// `CAncData`
#[derive(StateDigest, Clone, Debug, Default)]
pub struct CAncData {
    /// `buffer`: the ancillary data buffer of the application (owned copy)
    pub buffer: Vec<u8>,
    /// `bufferSize`
    pub buffer_size: i32,
    /// `offset[8]`
    pub offset: [i32; 8],
    /// `nrElements`
    pub nr_elements: i32,
}

/// `EarconDecoder`
#[derive(StateDigest, Clone, Debug)]
pub struct EarconDecoder {
    pub earcon_config: EarconConfig,
    pub earcon_info: EarconInfo,
    pub first_frame: u8,
    pub target_loudness_old: i32,
    pub m_bs_pcm_loudness_value_old: u32,
    pub m_bs_pcm_attenuation_gain_old: u32,
    pub earcon_gain: FixpDbl,
    pub earcon_shift: i32,
    /// Exact attenuation value used if increment is 0.
    pub att_gain_store_high_precision: FixpDbl,
    pub att_gain_exp_store_high_precision: i32,
    /// current processing attenuation
    pub att_gain_intermediate: FixpDbl,
    pub att_gain_shift_intermediate: i32,
    pub att_gain_increment_intermediate: FixpDbl,
    /// start attenuation of current frame which ramps to AttGain_new
    pub att_gain_old: FixpDbl,
    pub att_gain_shift_old: i32,
    pub att_gain_increment_old: FixpDbl,
    /// end attenuation of current frame
    pub att_gain_new: FixpDbl,
    pub att_gain_shift_new: i32,
    pub earcon_data: Box<[FixpSgl; EARCON_BUFFER_SIZE]>,
    /// Amount of buffered earcon samples in EarconData. Stereo samples account as 2.
    pub accumulated_frame_size: i32,
    pub baseframe_size: i32,
    pub truncation_present: i32,
    /// Current amount of earcon signals
    pub num_pcm_signals_in_frame: i32,
    /// Past amount of earcon signals
    pub num_pcm_signals_old: i32,
    /// Amount of decoder output channels that receive earcon mix
    pub num_signals_mixed: i32,
    /// Amount of decoder output channels that are unrelated to earcons.
    pub number_of_rest_channels: i32,
    pub current_frame_has_earcon: i32,
    pub last_frame_had_earcon: i32,
    pub speaker_pos_indices: [i32; EARCON_MAX_NUM_SIGNALS as usize],
    pub speaker_pos_indices_rest: [i32; 28],
    pub speaker_gains: [i32; EARCON_MAX_NUM_SIGNALS as usize],
}

impl Default for EarconDecoder {
    /// All zero (the decoder instance is `calloc()`ed).
    fn default() -> Self {
        EarconDecoder {
            earcon_config: EarconConfig::default(),
            earcon_info: EarconInfo::default(),
            first_frame: 0,
            target_loudness_old: 0,
            m_bs_pcm_loudness_value_old: 0,
            m_bs_pcm_attenuation_gain_old: 0,
            earcon_gain: 0,
            earcon_shift: 0,
            att_gain_store_high_precision: 0,
            att_gain_exp_store_high_precision: 0,
            att_gain_intermediate: 0,
            att_gain_shift_intermediate: 0,
            att_gain_increment_intermediate: 0,
            att_gain_old: 0,
            att_gain_shift_old: 0,
            att_gain_increment_old: 0,
            att_gain_new: 0,
            att_gain_shift_new: 0,
            earcon_data: Box::new([0; EARCON_BUFFER_SIZE]),
            accumulated_frame_size: 0,
            baseframe_size: 0,
            truncation_present: 0,
            num_pcm_signals_in_frame: 0,
            num_pcm_signals_old: 0,
            num_signals_mixed: 0,
            number_of_rest_channels: 0,
            current_frame_has_earcon: 0,
            last_frame_had_earcon: 0,
            speaker_pos_indices: [0; EARCON_MAX_NUM_SIGNALS as usize],
            speaker_pos_indices_rest: [0; 28],
            speaker_gains: [0; EARCON_MAX_NUM_SIGNALS as usize],
        }
    }
}

/// `struct AAC_DECODER_INSTANCE` (`HANDLE_AACDECODER` is `Box<AacDecoderInstance>`).
///
/// See the [crate documentation](crate) for the memory owned by the instance. Differences to the
/// C++ structure:
/// - `pAacDecoderChannelInfo[]` / `pAacDecoderStaticChannelInfo[]` always hold 28 entries;
///   `NULL` entries are marked in [`AacDecoderInstance::ch_info_alloc`] /
///   [`AacDecoderInstance::st_info_alloc`] (allocating replaces the entry with zeroed memory).
/// - `pComStaticData` of the channels is [`AacDecoderInstance::com_static_data`] (owned by the
///   first channel of the element) and [`AacDecoderInstance::com_static_link`];
///   `pWorkBufferCore1` is [`AacDecoderInstance::work_buffer_core1`].
/// - `pCpeStaticData` of the static channel info is [`AacDecoderInstance::cpe_static_link`].
/// - `pUsacConfig[]` is [`AacDecoderInstance::p_usac_config`] (see the crate documentation).
/// - The groups downmix matrix set that `aacDecoder_ParseDmxMatrixCallback()` writes into
///   `pTimeData2` is stored there with the C++ memory layout ([`store_downmix_matrix_set`]).
///
/// State digest ([`mpeghdec_fdk::digest`]): all fields except the scratch memory and the
/// temporal channel memory, see the comments of the skipped fields.
#[derive(StateDigest)]
#[digest(extra = digest_conditional)]
pub struct AacDecoderInstance {
    /// Amount of AAC decoder channels allocated.
    pub aac_channels: i32,
    /// Amount of AAC decoder channels signalled in ASC.
    pub asc_channels: [i32; TPDEC_MAX_TRACKS],
    /// Amount of AAC decoder channels allocated per each track/substream.
    pub alloc_channels: [i32; TPDEC_MAX_TRACKS],
    /// frame counter (only incremented, never read: not part of the state digest)
    #[digest(skip)]
    pub block_number: i32,

    pub nr_of_layers: i32,

    /// PCM output format (interleaved/none interleaved).
    pub output_interleaved: i32,

    /// Headroom of the output time signal to prevent clipping
    pub aac_out_data_headroom: i32,

    /// Transport layer handle (`None` while the transport decoder is calling back).
    pub h_input: Option<Box<TransportDec>>,

    /// Sampling Rate information table
    pub sampling_rate_info: [SamplingRateInfo; TPDEC_MAX_TRACKS],

    /// Will be unset if a consistency check, e.g. CRC etc. fails
    pub frame_ok: u8,

    /// Flags for internal decoder use. DO NOT USE self::streaminfo::flags !
    pub flags: [u32; TPDEC_MAX_TRACKS],
    /// Flags for internal decoder use (element specific).
    pub el_flags: [u32; AAC_MAX_ELEMENTS],

    /// Table where the element Id's are listed
    pub elements: [Mp4ElementId; AAC_MAX_ELEMENTS],
    /// Table of MPEG canonical order to bitstream channel order mapping.
    pub ch_mapping: [u8; AAC_MAX_SIGNALS],

    /// Channel elements in bit stream order.
    pub channel_elements: [Mp4ElementId; AAC_MAX_ELEMENTS],

    /// Audio channel type of each output audio channel (from 0 upto numChannels).
    pub channel_type: [AudioChannelType; 28],
    /// Audio channel index for each output audio channel (from 0 upto numChannels).
    pub channel_indices: [u8; 28],

    pub pce: CProgramConfig,
    /// StreamInfo data (read from the bitstream)
    pub stream_info: CStreamInfo,
    /// Temporal channel memory
    ///
    /// Not part of the state digest: the per frame data of every channel (ICS info, section
    /// data, scale factors, TNS and IGF data, `pDynData`), which the element parsing writes before
    /// the element decoding reads it. Only the concealment of a frame (which signals a
    /// [hazard](mpeghdec_fdk::digest::hazard)) uses values left from an earlier frame.
    #[digest(skip)]
    pub p_aac_decoder_channel_info: Vec<Box<CAacDecoderChannelInfo>>,
    /// `pAacDecoderChannelInfo[ch] != NULL`
    pub ch_info_alloc: [bool; 28],
    /// Persistent channel memory
    pub p_aac_decoder_static_channel_info: Vec<Box<CAacDecoderStaticChannelInfo>>,
    /// `pAacDecoderStaticChannelInfo[ch] != NULL`
    pub st_info_alloc: [bool; 28],
    /// `pComStaticData` allocated for the element starting at channel `ch`.
    pub com_static_data: Vec<Option<Box<CAacDecoderCommonStaticData>>>,
    /// `pAacDecoderChannelInfo[ch]->pComStaticData`: the channel owning the structure.
    pub com_static_link: [Option<usize>; 28],
    /// `pComStaticData->pWorkBufferCore1` (one instance for all channels)
    ///
    /// Not part of the state digest: scratch memory of the element decoding (IMDCT output
    /// `mdctOutTemp`, IGF TNF masks and stereo filling downmix, which are cleared before use, and
    /// the complex prediction downmix), written before it is read within an access unit (checked
    /// with the buffer poisoning [`POISON_DIGEST_SCRATCH`]).
    #[digest(skip)]
    pub work_buffer_core1: CWorkBufferCore1,

    /// Not part of the state digest: the spectra of the current frame, written before they are
    /// read (see the buffer poisoning in [`crate::render`]).
    #[digest(skip)]
    pub work_buffer_core2: Vec<FixpDbl>,
    /// Part of the state digest only if the IGF may read stale content of it (see
    /// `digest_conditional()`).
    #[digest(skip)]
    pub p_time_data2: Vec<PcmDec>,
    /// Size of `pTimeData2` in bytes.
    pub time_data2_size: i32,

    /// Persistent data shared by both channels of a CPE. This structure is allocated once for
    /// each CPE.
    pub cpe_static_data: Vec<Option<Box<CpePersistentData>>>,
    /// `pAacDecoderStaticChannelInfo[ch]->pCpeStaticData`: index into `cpe_static_data`.
    pub cpe_static_link: [Option<usize>; 28],

    pub conceal_common_data: CConcealParams,
    pub conceal_method_user: CConcealmentMethod,

    pub num_usac_elements: [i32; TPDEC_MAX_TRACKS],
    /// `pUsacConfig[s]`: index of the transport decoder config (`hTp->asc[]`), `None` for
    /// `NULL`.
    pub p_usac_config: [Option<usize>; TPDEC_MAX_TRACKS],

    /// The amount of AAC core channels of the last successful decode call.
    pub aac_channels_prev: i32,
    /// Array holding the channelType values of the last successful decode call.
    pub channel_type_prev: [AudioChannelType; 28],
    /// Array holding the channelIndices values of the last successful decode call.
    pub channel_indices_prev: [u8; 28],

    /// Variable to store a supported ELD downscale factor of 1, 2, 3 or 4
    pub downscale_factor: u8,

    /// structure to handle ancillary data
    pub anc_data: CAncData,

    /// The limiter configuration requested by the library user
    pub limiter_enable_user: u8,
    /// The current limiter configuration.
    pub limiter_enable_curr: u8,

    pub num_obj_signal_groups: [u8; TPDEC_MAX_TRACKS + 1],

    pub multiband_drc_present: u8,
    pub num_time_slots: u8,
    pub stft_frame_size: u16,
    /// State digest: `[0]` and `[1]` are bit positions (remaining bits) in the access unit of the
    /// last configuration packet, read only as the difference `[0] - [1]` (the position of the
    /// loudness info set in the configuration) and only if `[2]` (its length) is positive
    /// (`disregardLoudnessChange()`); only these are fed.
    #[digest(with = digest_loudness_position)]
    pub loudness_info_set_position: [i32; 3],
    pub default_drc_set_effect: i8,
    pub default_target_loudness: i8,

    /// Requested Target layout index based on table 95 of ISO/IEC DIS 23008-3. Value 0 means no
    /// rendering process.
    pub target_layout: i32,
    /// Applied Target layout index which can be either equal targetLayout or referenceLayout (if
    /// targetLayout is 0).
    pub target_layout_config: i32,
    pub downmix_id: i32,

    /// IGF scratch memory shared between elements but requires two instances in case of CPE
    ///
    /// Not part of the state digest: the energies and gains of the current IGF tiles, computed
    /// before they are used in every frame (checked with [`POISON_DIGEST_SCRATCH`]).
    #[digest(skip)]
    pub igf_private_data_common: [IgfPrivateDataCommon; 2],
    pub p_mct_dec: Vec<Option<Box<CMct>>>,

    /// Indicates flush status: on|off
    pub flush_status: u8,
    /// Flush frame counter
    pub flush_cnt: i8,
    /// Indicates build up status: on|off
    pub build_up_status: u8,
    /// Build up frame counter
    pub build_up_cnt: i8,
    /// Indicates preRoll status: on|off
    pub has_audio_pre_roll: u8,
    /// Relative offset of the prerollAU end position to the AU start position in the bitstream
    pub preroll_au_length: [u32; AACDEC_MAX_NUM_PREROLL_AU + 1],
    /// Number of the actual processed preroll accessUnit
    pub access_unit: i32,
    /// Size of the resulting truncated current frame.
    pub truncate_frame_size: i16,

    pub h_ui_manager: Box<UiManager>,

    /// Currently active UI status
    pub ui_status: UserInteractivityStatus,
    /// Next scheduled UI status.
    pub ui_status_next: UserInteractivityStatus,
    pub drc_status: UiDrcLoudnessStatus,
    /// Flag indicating if the embedded UI manager is enabled. If zero, then UI events are managed
    /// through MHAS packets, if greater than zero UI events are managed through the XML message
    /// interface. If less than zero, the integrated UI manager is selected automatically
    /// according to MHAS UI input.
    pub ui_manager_enabled: i8,
    /// Flag indicating if the integrated UI manager is currently active.
    pub ui_manager_active: u8,
    /// Flags indicating if UI or DRC loudness UI Data for uiManagerEnabled=0 case are valid.
    pub ui_status_valid: i8,
    pub drc_status_valid: i8,
    /// Flag indicating that the next scheduled UI status uiStatusNext has a different signal
    /// setup and needs to be delayed until the next RAP so that it can be applied.
    pub ui_signal_changed: u8,
    /// Flag indicating if skipping of inactive elements is used
    pub use_element_skipping: u8,
    pub signal_skipped: [u8; TP_MPEGH_MAX_SIGNAL_GROUPS],

    /// The state of the rendering stage ([`crate::render`]); `None` while the rendering of an
    /// access unit runs on the render thread (use [`rs!`](crate::rs) to access it).
    pub render: Option<Box<RenderState>>,

    /// `transportDec_SetParam()` calls made while the transport decoder is taken out of the
    /// instance (applied when the transport decoder callback returns, or after the user
    /// interactivity processing of `aacDecoder_DecodeFrame()`).
    pub pending_tp_params: Vec<(TpdecParam, i32)>,

    /// Thread pool for the parallel parts of the frame decoding (not in C; `None`: everything
    /// runs sequentially on the calling thread). See [`AacDecoderInstance::set_thread_pool`].
    #[digest(skip)]
    pub thread_pool: Option<Arc<ThreadPool>>,
    /// IMDCT output buffers of the channels for the parallel inverse transform (not in C, see
    /// `AacDecoderInstance::frequency_to_time_parallel()`). Scratch memory, not part of the
    /// state digest.
    #[digest(skip)]
    pub imdct_scratch: Vec<FixpDbl>,
    /* Frame pipelining (not in C, see crate::render) */
    /// The render thread (`None`: sequential rendering).
    #[digest(skip)]
    pub render_worker: Option<Arc<RenderWorker>>,
    /// The rendering running on the render thread.
    #[digest(with = digest_no_pending_render)]
    pub(crate) pending_render: Option<Pending<(Box<RenderState>, FrameResult)>>,
    /// Frames rendered on the render thread, not yet taken by the caller.
    pub(crate) completed: VecDeque<FrameResult>,
    /// Reusable PCM buffers of the rendered frames (their content is overwritten by the next
    /// rendering: not part of the state digest).
    #[digest(skip)]
    pub(crate) pcm_buffers: Vec<Vec<IntPcm>>,
    /// Parses done on the parser copies while the rendering was running.
    pub(crate) parse_log: Vec<ParseRecord>,
    /// Copy of the DRC gain parser state (of the render state's DRC decoder).
    pub(crate) drc_gain_parser: Option<Box<UniDrcGainParser>>,
    /// Copies of the object metadata parser state of the renderers `hgVBAPRenderer[]`.
    pub(crate) oam_parsers: Vec<Option<Box<GVbapRenderer>>>,
    /// The earcon state `earconConfig.earconFlag`, `currentFrameHasEarcon`,
    /// `lastFrameHadEarcon` of the render state (only changed by the transport decoder
    /// callbacks, which the core stage runs): whether `aacDecoder_EarconSetBSCallback()` has
    /// anything to do.
    pub(crate) earcon_active: bool,
    /// Whether the IGF of the current configuration may read stale content of `pTimeData2`
    /// (`None`: not yet determined; see [`AacDecoderInstance::igf_reads_stale_data`]). A cache of
    /// the frame pipelining, not part of the state digest.
    #[digest(skip)]
    pub(crate) igf_stale_reads: Option<bool>,
    /// Number of parses repeated on the render state with a different result than on the parser
    /// copy (must be 0; a mismatch would mean that the output may differ from the sequential
    /// decoding).
    pub pipeline_mismatches: u32,

    /// Buffer poisoning of the frame pipeline analysis (not in C, see
    /// [`AacDecoderInstance::poison_buffers`]): `POISON_*` bits, from the environment variable
    /// `MPEGHDEC_POISON` when the decoder is opened.
    pub poison: u32,
    /// State of the pseudo random generator of the buffer poisoning. It only determines the
    /// garbage written into the buffers that are not part of the state digest (`pTimeData2`,
    /// `workBufferCore2`), so it is not part of it either.
    #[digest(skip)]
    pub poison_seed: u32,

    /// Fast mode options (not in C, see [`crate::fast`]).
    pub fast_options: crate::fast::FastOptions,
}

/// The state digest is only computed between access units, when no rendering runs on the render
/// thread.
fn digest_no_pending_render(
    p: &Option<Pending<(Box<RenderState>, FrameResult)>>,
    d: &mut mpeghdec_fdk::digest::Digester,
) {
    assert!(p.is_none(), "state digest while a rendering is pending");
    d.u32(0);
}

fn digest_loudness_position(pos: &[i32; 3], d: &mut mpeghdec_fdk::digest::Digester) {
    use mpeghdec_fdk::digest::StateDigest;
    pos[2].digest(d);
    if pos[2] > 0 {
        pos[0].wrapping_sub(pos[1]).digest(d);
    }
}

/// The parts of the state digest that depend on other fields:
/// - `pTimeData2` is part of it only if the IGF of the configuration may read content that an
///   earlier access unit left in it (`virtualSpec`, see
///   [`AacDecoderInstance::igf_reads_stale_data`]); otherwise it is written before it is read in
///   every access unit (see the buffer poisoning in [`crate::render`]).
/// - `alpha_q_im` of the complex prediction data of the elements: the IGF bands of the element
///   (see [`crate::stereo::CCplxPredictionData::alpha_q_im`]).
fn digest_conditional(s: &AacDecoderInstance, d: &mut mpeghdec_fdk::digest::Digester) {
    if s.igf_stale_reads_uncached() {
        d.field("p_time_data2", &s.p_time_data2);
    }
    /* `alpha_q_im` of the complex prediction data from the lowest IGF start band of the
    element on (see `CCplxPredictionData::alpha_q_im`) */
    d.field_with("alpha_q_im", |d| {
        for (ch, c) in s.com_static_data.iter().enumerate() {
            let Some(cpd) = c.as_deref().and_then(|c| c.cplx_prediction_data.as_deref()) else {
                continue;
            };
            let igf = &s.p_aac_decoder_static_channel_info[ch].igf_static_data;
            let m = crate::stereo::JOINT_STEREO_MAXIMUM_BANDS;
            let (mut from, mut to) = (m, 0);
            for (a, b) in [
                (igf.igf_start_sfb_lb, igf.igf_stop_sfb_lb),
                (igf.igf_start_sfb_sb, igf.igf_stop_sfb_sb),
            ] {
                if b > a {
                    from = from.min(a as usize);
                    to = to.max(b as usize);
                }
            }
            let (from, to) = (from.min(m), to.min(m));
            d.element_with(ch, |d| {
                d.u64(from as u64);
                d.u64(to as u64);
                if to > from {
                    for g in &cpd.alpha_q_im {
                        i16::digest_slice(&g[from..to], d);
                    }
                }
            });
        }
    });
}

/// [`AacDecoderInstance::poison`]: fill `pTimeData2` and `workBufferCore2` with garbage before the
/// core decoding of an access unit (i.e. after the rendering of the previous one).
pub const POISON_BEFORE_CORE: u32 = 1;
/// [`AacDecoderInstance::poison`]: fill `workBufferCore2` with garbage after the core decoding of
/// an access unit, before its rendering.
pub const POISON_BEFORE_RENDER: u32 = 2;
/// [`AacDecoderInstance::poison`]: also fill `pTimeData2` after the core decoding of an access
/// unit (negative control: destroys the decoded time signal, the output must change).
pub const POISON_TIME_SIGNAL: u32 = 4;
/// [`AacDecoderInstance::poison`]: fill the scratch memory that is not part of the state digest
/// (`workBufferCore1`, the IGF working memory, the joint stereo data of the elements and the
/// parallel IMDCT buffers) with garbage before the core decoding of an access unit: the output
/// must not change.
pub const POISON_DIGEST_SCRATCH: u32 = 8;

/// `CAacDecoder_GetELChannels(type)`: calculates the number of element channels
pub(crate) fn caac_decoder_get_el_channels(type_: Mp4ElementId) -> i32 {
    match type_ {
        ID_USAC_CPE | ID_CPE => 2,
        ID_USAC_SCE | ID_USAC_LFE | ID_SCE | ID_LFE => 1,
        _ => 0,
    }
}

/// `CAacDecoder_AncDataReset(ancData)`: reset ancillary data struct. Call before parsing a new
/// frame.
fn caac_decoder_anc_data_reset(anc_data: &mut CAncData) -> AacDecoderError {
    anc_data.offset = [0; 8];
    anc_data.nr_elements = 0;
    AAC_DEC_OK
}

/// `CAacDecoder_AncDataInit(ancData, buffer, size)`: initialize ancillary buffer
pub fn caac_decoder_anc_data_init(
    anc_data: &mut CAncData,
    buffer: Vec<u8>,
    size: i32,
) -> AacDecoderError {
    if size >= 0 {
        anc_data.buffer = buffer;
        anc_data.buffer_size = size;

        caac_decoder_anc_data_reset(anc_data);

        return AAC_DEC_OK;
    }

    AAC_DEC_ANC_DATA_ERROR
}

/// `CAacDecoder_AncDataGet(ancData, index, ptr, size)`: get one ancillary data element. Returns
/// the element (`None` for `*ptr = NULL`).
pub fn caac_decoder_anc_data_get(anc_data: &CAncData, index: i32) -> Option<&[u8]> {
    if index >= 0 && index < 8 - 1 && index < anc_data.nr_elements {
        let start = anc_data.offset[index as usize] as usize;
        let end = anc_data.offset[index as usize + 1] as usize;
        return anc_data.buffer.get(start..end);
    }
    None
}

/// `CStreamInfoInit(pStreamInfo)`
fn cstream_info_init(p_stream_info: &mut CStreamInfo) {
    p_stream_info.aac_sample_rate = 0;
    p_stream_info.profile = -1;
    p_stream_info.aot = AOT_NONE;

    p_stream_info.channel_config = -1;
    p_stream_info.bit_rate = 0;
    p_stream_info.aac_samples_per_frame = 0;

    p_stream_info.ext_aot = AOT_NONE;
    p_stream_info.ext_sampling_rate = 0;

    p_stream_info.flags = 0;

    p_stream_info.ep_config = -1; /* default: no ER */

    p_stream_info.num_channels = 0;
    p_stream_info.num_channels_before_mix = 0;
    p_stream_info.sample_rate = 0;
    p_stream_info.sample_rate_before_rs = 0;
    p_stream_info.frame_size = 0;

    p_stream_info.output_delay = 0;

    p_stream_info.pcm_ch_order = CH_ORDER_CICP; /* default: CICP */

    /* DRC */
    p_stream_info.drc_prog_ref_lev = -1; /* set program reference level to not indicated */
    p_stream_info.drc_pres_mode = -1; /* default: presentation mode not indicated */
    p_stream_info.pce_matrix_mixdown_idx = -1; /* default: matrix mixdown index from PCE not indicated */
    p_stream_info.pce_pseudo_surround_enable = -1; /* default: pseudo surround flag from PCE not indicated */

    p_stream_info.output_loudness = -1; /* default: no loudness metadata present */

    p_stream_info.f_bs_restart_ok = 0;
}

/// `EarconDecoder_Init(hEarconDecoderH)`
pub fn earcon_decoder_init(h: &mut EarconDecoder) {
    h.earcon_config.m_num_pcm_signals = 0;
    h.earcon_info.m_num_earcons = 0;
    h.first_frame = 1;
    h.accumulated_frame_size = 0;
    h.baseframe_size = 1024;
    h.num_pcm_signals_old = 0;
    h.current_frame_has_earcon = 0;
    h.last_frame_had_earcon = 0;
}

impl AacDecoderInstance {
    /// `GetAacDecoder()`: the zero-initialized instance.
    fn get_aac_decoder() -> Box<AacDecoderInstance> {
        Box::new(AacDecoderInstance {
            aac_channels: 0,
            asc_channels: [0; TPDEC_MAX_TRACKS],
            alloc_channels: [0; TPDEC_MAX_TRACKS],
            block_number: 0,
            nr_of_layers: 0,
            output_interleaved: 0,
            aac_out_data_headroom: 0,
            h_input: None,
            sampling_rate_info: [SamplingRateInfo::default(); TPDEC_MAX_TRACKS],
            frame_ok: 0,
            flags: [0; TPDEC_MAX_TRACKS],
            el_flags: [0; AAC_MAX_ELEMENTS],
            elements: [0; AAC_MAX_ELEMENTS],
            ch_mapping: [0; AAC_MAX_SIGNALS],
            channel_elements: [0; AAC_MAX_ELEMENTS],
            channel_type: [0; 28],
            channel_indices: [0; 28],
            pce: CProgramConfig::default(),
            stream_info: CStreamInfo::default(),
            p_aac_decoder_channel_info: (0..28).map(|_| Box::default()).collect(),
            ch_info_alloc: [false; 28],
            p_aac_decoder_static_channel_info: (0..28).map(|_| Box::default()).collect(),
            st_info_alloc: [false; 28],
            com_static_data: (0..28).map(|_| None).collect(),
            com_static_link: [None; 28],
            work_buffer_core1: CWorkBufferCore1::default(),
            work_buffer_core2: Vec::new(),
            p_time_data2: Vec::new(),
            time_data2_size: 0,
            cpe_static_data: (0..AAC_MAX_ELEMENTS).map(|_| None).collect(),
            cpe_static_link: [None; 28],
            conceal_common_data: CConcealParams::default(),
            conceal_method_user: 0,
            num_usac_elements: [0; TPDEC_MAX_TRACKS],
            p_usac_config: [None; TPDEC_MAX_TRACKS],
            aac_channels_prev: 0,
            channel_type_prev: [0; 28],
            channel_indices_prev: [0; 28],
            downscale_factor: 0,
            anc_data: CAncData::default(),
            limiter_enable_user: 0,
            limiter_enable_curr: 0,
            num_obj_signal_groups: [0; TPDEC_MAX_TRACKS + 1],
            multiband_drc_present: 0,
            num_time_slots: 0,
            stft_frame_size: 0,
            loudness_info_set_position: [0; 3],
            default_drc_set_effect: 0,
            default_target_loudness: 0,
            target_layout: 0,
            target_layout_config: 0,
            downmix_id: 0,
            igf_private_data_common: [IgfPrivateDataCommon::default(), Default::default()],
            p_mct_dec: (0..TP_MPEGH_MAX_SIGNAL_GROUPS).map(|_| None).collect(),
            flush_status: 0,
            flush_cnt: 0,
            build_up_status: 0,
            build_up_cnt: 0,
            has_audio_pre_roll: 0,
            preroll_au_length: [0; AACDEC_MAX_NUM_PREROLL_AU + 1],
            access_unit: 0,
            truncate_frame_size: 0,
            h_ui_manager: UiManager::create(0),
            ui_status: UserInteractivityStatus::default(),
            ui_status_next: UserInteractivityStatus::default(),
            drc_status: UiDrcLoudnessStatus::default(),
            ui_manager_enabled: 0,
            ui_manager_active: 0,
            ui_status_valid: 0,
            drc_status_valid: 0,
            ui_signal_changed: 0,
            use_element_skipping: 0,
            signal_skipped: [0; TP_MPEGH_MAX_SIGNAL_GROUPS],
            render: Some(RenderState::new()),
            pending_tp_params: Vec::new(),
            thread_pool: None,
            imdct_scratch: Vec::new(),
            render_worker: None,
            pending_render: None,
            completed: VecDeque::new(),
            pcm_buffers: Vec::new(),
            parse_log: Vec::new(),
            drc_gain_parser: None,
            oam_parsers: Vec::new(),
            earcon_active: false,
            igf_stale_reads: None,
            pipeline_mismatches: 0,
            poison: std::env::var("MPEGHDEC_POISON")
                .ok()
                .and_then(|v| v.trim().parse().ok())
                .unwrap_or(0),
            poison_seed: 0x1234_5678,
            fast_options: crate::fast::FastOptions::EXACT,
        })
    }

    /// Buffer poisoning (not in C): fills `pTimeData2` (`time_data2`) and / or `workBufferCore2`
    /// (`core2`) with pseudo random values. The sequential decoder calls this at the stage
    /// boundaries selected by [`AacDecoderInstance::poison`] to show that a stage does not read
    /// the content that the other stage left in these buffers: if the decoded output is unchanged,
    /// the core decoder and the rendering can work on separate buffers (frame pipelining).
    pub fn poison_buffers(&mut self, time_data2: bool, core2: bool) {
        let mut x = self.poison_seed;
        let mut fill = |buf: &mut [FixpDbl]| {
            for v in buf.iter_mut() {
                /* xorshift32 */
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                *v = x as FixpDbl;
            }
        };
        if time_data2 {
            fill(&mut self.p_time_data2);
        }
        if core2 {
            fill(&mut self.work_buffer_core2);
        }
        self.poison_seed = x;
    }

    /// Buffer poisoning of the scratch memory that is not part of the state digest
    /// ([`POISON_DIGEST_SCRATCH`]).
    pub fn poison_digest_scratch(&mut self) {
        let mut x = self.poison_seed;
        let mut next = || {
            /* xorshift32 */
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        for v in self.work_buffer_core1.buf.iter_mut() {
            *v = next() as FixpDbl;
        }
        for v in self.imdct_scratch.iter_mut() {
            *v = next() as FixpDbl;
        }
        for c in self.igf_private_data_common.iter_mut() {
            for w in c.igf_working_mem.iter_mut() {
                let IgfWorkMemory {
                    f_sfb_gain_tab,
                    f_sfb_destin_energy_tab,
                    f_sfb_tile_energy_tab,
                    f_sfb_survived_energy_tab,
                    s_e,
                    t_e,
                    f_sfb_gain_tab_exp,
                    f_sfb_destin_energy_tab_exp,
                    f_sfb_tile_energy_tab_exp,
                    f_sfb_survived_energy_tab_exp,
                    s_e_exp,
                    t_e_exp,
                    intermediate_shift,
                    initial_exp,
                } = w;
                for a in [
                    &mut f_sfb_gain_tab[..],
                    &mut f_sfb_destin_energy_tab[..],
                    &mut f_sfb_tile_energy_tab[..],
                    &mut f_sfb_survived_energy_tab[..],
                    &mut s_e[..],
                    &mut t_e[..],
                ] {
                    for v in a.iter_mut() {
                        *v = next() as FixpDbl;
                    }
                }
                for a in [
                    &mut f_sfb_gain_tab_exp[..],
                    &mut f_sfb_destin_energy_tab_exp[..],
                    &mut f_sfb_tile_energy_tab_exp[..],
                    &mut f_sfb_survived_energy_tab_exp[..],
                    &mut s_e_exp[..],
                    &mut t_e_exp[..],
                    &mut intermediate_shift[..],
                    &mut initial_exp[..],
                ] {
                    for v in a.iter_mut() {
                        *v = next() as i8;
                    }
                }
            }
        }
        for c in self.com_static_data.iter_mut().flatten() {
            if let Some(j) = c.joint_stereo_data.as_deref_mut() {
                j.ms_mask_present = next() as u8;
                for v in j.ms_used.iter_mut() {
                    *v = next() as u8;
                }
                j.cplx_pred_flag = next() as u8;
                j.igf_ms_mask_present = next() as u8;
                j.igf_cplx_pred_flag = next() as u8;
                for v in j.store_dmx_re_prev.iter_mut() {
                    *v = next() as FixpDbl;
                }
                j.store_dmx_re_prev_e = next() as i16;
            }
        }
        self.poison_seed = x;
    }

    /// Sets the thread pool used for the parallel parts of the frame decoding (not in C).
    ///
    /// The output does not depend on the pool (or its size): only work items that are
    /// independent of each other are distributed.
    pub fn set_thread_pool(&mut self, pool: Option<Arc<ThreadPool>>) {
        self.thread_pool = pool;
    }

    /// `pUsacConfig[s]` resolved with the transport decoder configs.
    #[inline]
    pub fn usc<'a>(&self, ascs: &'a [CSAudioSpecificConfig], s: usize) -> Option<&'a CSUsacConfig> {
        self.p_usac_config[s].map(|k| &ascs[k].m_sc)
    }

    /// `pUsacConfig[s]` for a `pUsacConfig` that the C++ code dereferences without check.
    #[inline]
    pub(crate) fn usc_ref<'a>(
        &self,
        ascs: &'a [CSAudioSpecificConfig],
        s: usize,
    ) -> &'a CSUsacConfig {
        self.usc(ascs, s).expect("pUsacConfig")
    }

    /// `transportDec_SetParam()` while the transport decoder is calling back (see
    /// [`AacDecoderInstance::pending_tp_params`]).
    pub(crate) fn tp_set_param(&mut self, param: TpdecParam, value: i32) {
        match self.h_input.as_deref_mut() {
            Some(tp) => {
                tp.set_param(param, value);
            }
            None => self.pending_tp_params.push((param, value)),
        }
    }

    /// `CAacDecoder_SignalInterruption(self)`: signal a bit stream interruption to the decoder
    pub fn caac_decoder_signal_interruption(&mut self) {
        if self.flags[0] & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 {
            for i in 0..self.aac_channels.clamp(0, 28) as usize {
                if self.st_info_alloc[i] {
                    /* number of active channels can be smaller */
                    if let Some(arco) =
                        self.p_aac_decoder_static_channel_info[i].h_ar_co.as_deref_mut()
                    {
                        arco.m_number_lines_prev = 0;
                    }
                }
            }
        }
    }

    /// `CAacDecoder_PreRollExtensionPayloadParse(self, numPrerollAU, prerollAUOffset)`: parse
    /// PreRoll Extension Payload
    pub fn caac_decoder_pre_roll_extension_payload_parse(
        &mut self,
        tp: &mut TransportDec,
        num_preroll_au: &mut u32,
        preroll_au_offset: &mut [u32],
    ) -> AacDecoderError {
        let mut error_status;
        let mut config_length: u32 = 0;
        let mut pre_roll_possible: u32 = 1;
        let mut min_num_preroll_au: i32 = -1;
        let mut config_changed: u8 = 0;
        let mut config = [0u8; TP_USAC_MAX_CONFIG_LEN];

        error_status = AAC_DEC_OK;

        'bail: {
            for sub_stream_index in 0..TPDEC_MAX_LAYERS {
                if self.num_usac_elements[sub_stream_index] <= 0 {
                    break;
                }

                /* Copy BS context */
                let mut bs = tp.get_bitstream(sub_stream_index as u32).clone();
                let h_bs = &mut bs;

                let au_start_anchor = h_bs.get_valid_bits();
                if au_start_anchor <= 0 {
                    error_status = AAC_DEC_NOT_ENOUGH_BITS;
                    break 'bail;
                }

                /* Independency flag */
                h_bs.read_bit();

                /* Payload present flag of extension ID_EXT_ELE_AUDIOPREROLL must be one */
                let ext_payload_present_flag = h_bs.read_bit();
                if ext_payload_present_flag == 0 {
                    pre_roll_possible = 0;
                }

                /* Default length flag of extension ID_EXT_ELE_AUDIOPREROLL must be zero */
                let use_default_length_flag = h_bs.read_bit();
                if use_default_length_flag != 0 {
                    pre_roll_possible = 0;
                }

                if pre_roll_possible != 0 {
                    /* extPayloadPresentFlag && !useDefaultLengthFlag */
                    /* Read overall ext payload length, useDefaultLengthFlag must be zero.  */
                    h_bs.escaped_value(8, 16, 0);

                    /* Read MPEG-H Config size */
                    config_length = h_bs.escaped_value(4, 4, 8);

                    /* Avoid decoding pre roll frames if there was no config change and no config
                     * is included in the pre roll ext payload. */
                    if (self.flags[0] & AC_MPEGH3DA != 0) && (config_length == 0) {
                        /* self->streamInfo.numChannels is checked additionally because for the
                         * first frame buildUpStatus is not set. */
                        if self.build_up_status == AACDEC_BUILD_UP_OFF
                            && self.stream_info.num_channels != 0
                        {
                            pre_roll_possible = 0;
                        }
                    }
                }

                /* If pre roll not possible then exit. */
                if pre_roll_possible == 0 {
                    /* Sanity check: if flushing is switched on, preRollPossible must be 1 */
                    if self.flush_status != AACDEC_FLUSH_OFF {
                        /* Mismatch of current payload and flushing status */
                        self.flush_status = AACDEC_FLUSH_OFF;
                        error_status = AAC_DEC_PARSE_ERROR;
                    } else {
                        continue;
                    }
                    break 'bail;
                }

                if self.flags[0] & AC_MPEGH3DA != 0 && config_length > 0 {
                    /* DASH IPF ATSC Config Change: Read new config and compare with current
                     * config. Apply reconfiguration if config's are different. */
                    for i in 0..config_length as usize {
                        let v = h_bs.read_bits(8) as u8;
                        if i < TP_USAC_MAX_CONFIG_LEN {
                            config[i] = v;
                        }
                    }
                    let terr = tp.in_band_config(
                        &config,
                        config_length,
                        self.build_up_status,
                        &mut config_changed,
                        0,
                        self,
                    );
                    if terr != TRANSPORTDEC_OK {
                        error_status = AAC_DEC_PARSE_ERROR;
                        break 'bail;
                    }
                }

                /* For the first frame buildUpStatus is not set and no flushing is performed but
                 * preroll AU's should processed. */
                /* buildUpStatus is checked additionally because if explicit cfg != implicit cfg ->
                 * idle state is forced. */
                if (self.stream_info.num_channels == 0)
                    && (self.flags[0] & AC_MPEGH3DA != 0)
                    && self.build_up_status == AACDEC_BUILD_UP_OFF
                {
                    self.build_up_status = AACDEC_MPEGH_BUILD_UP_ON;
                    /* sanity check: if buildUp status on -> flushing must be off */
                    if self.flush_status != AACDEC_FLUSH_OFF {
                        self.flush_status = AACDEC_FLUSH_OFF;
                        error_status = AAC_DEC_PARSE_ERROR;
                        break 'bail;
                    }
                }

                if self.flags[0] & AC_MPEGH3DA != 0 {
                    /* We are interested in preroll AUs if an explicit or an implicit config
                     * change is signalized in other words if the build up status is set. */
                    if (self.build_up_status == AACDEC_MPEGH_BUILD_UP_ON)
                        || (self.build_up_status == AACDEC_MPEGH_BUILD_UP_ON_IN_BAND)
                    {
                        let apply_crossfade = h_bs.read_bit();
                        if apply_crossfade != 0 {
                            rs!(self).apply_crossfade |= AACDEC_CROSSFADE_BITMASK_PREROLL;
                        } else {
                            rs!(self).apply_crossfade &= !AACDEC_CROSSFADE_BITMASK_PREROLL;
                        }
                        h_bs.read_bit(); /* reserved */
                        /* Read num preroll AU's */
                        *num_preroll_au = h_bs.escaped_value(2, 4, 0);
                        /* check limits for MPEG-H LC profile */
                        if *num_preroll_au > AACDEC_MAX_NUM_PREROLL_AU_MPEGH {
                            *num_preroll_au = 0;
                            error_status = AAC_DEC_PARSE_ERROR;
                            break 'bail;
                        }
                    }
                }

                for i in 0..*num_preroll_au as usize {
                    /* For every AU get length and offset in the bitstream */
                    let preroll_au_length = h_bs.escaped_value(16, 16, 0) as i32;
                    if preroll_au_length > 0 {
                        preroll_au_offset[i + sub_stream_index] =
                            (au_start_anchor - h_bs.get_valid_bits()) as u32;
                        let independency_flag = h_bs.read_bit();
                        if i == 0 && independency_flag == 0 {
                            *num_preroll_au = 0;
                            error_status = AAC_DEC_PARSE_ERROR;
                            break 'bail;
                        }
                        if sub_stream_index == 0 {
                            h_bs.push_for((preroll_au_length * 8 - 1) as u32);
                            self.preroll_au_length[i] =
                                ((preroll_au_length * 8) as u32).wrapping_add(preroll_au_offset[i]);
                        }
                    } else {
                        *num_preroll_au = 0;
                        error_status = AAC_DEC_PARSE_ERROR; /* Something is wrong */
                        break 'bail;
                    }
                }

                if min_num_preroll_au < 0 || (*num_preroll_au as i32) < min_num_preroll_au {
                    min_num_preroll_au = *num_preroll_au as i32;
                }
            }

            *num_preroll_au = if min_num_preroll_au < 0 { 0 } else { min_num_preroll_au as u32 };
        }

        error_status
    }

    /// `CAacDecoder_Open(bsFormat)`: initialization of the decoder instance.
    pub fn caac_decoder_open(_bs_format: TransportType) -> Option<Box<AacDecoderInstance>> {
        let mut self_ = Self::get_aac_decoder();

        self_.stream_info.num_elements = 0;
        self_.downscale_factor = 1;

        /* initialize anc data */
        caac_decoder_anc_data_init(&mut self_.anc_data, Vec::new(), 0);

        /* initialize stream info */
        cstream_info_init(&mut self_.stream_info);

        /* initialize progam config */
        cprogram_config_init(&mut self_.pce);

        /* initialize error concealment common data */
        cconcealment_init_common_data(&mut self_.conceal_common_data);
        self_.conceal_method_user = ConcealMethodNone; /* undefined -> auto mode */

        self_.work_buffer_core2 = get_work_buffer_core2();

        /* When MPEG-H is active use dedicated memory for core decoding */
        self_.p_time_data2 = get_work_buffer_core5();
        self_.time_data2_size = (WORK_BUFFER_CORE5_SIZE * core::mem::size_of::<PcmDec>()) as i32;

        Some(self_)
    }

    /// `CAacDecoder_DeInitRenderer(self, subStreamIndex)`: revert `CAacDecoder_InitRenderer()`
    fn caac_decoder_deinit_renderer(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        sub_stream_index: usize,
    ) {
        if sub_stream_index == 0 {
            let rs = rs!(self);
            if let Some(fc) = rs.p_format_converter.take() {
                rs.fast_bypass_frames += fc.member.fast_bypass_frames;
            }
        }

        let mut obj_sig_grp_offset = 0usize;
        let mut prev_signals = 0i32;

        if sub_stream_index == 0 {
            for i in 0..TP_MPEGH_MAX_SIGNAL_GROUPS {
                if let Some(r) = rs!(self).hg_vbap_renderer[i].take() {
                    gvbap_renderer_close(r);
                }
            }
        } else {
            for i in 0..sub_stream_index {
                let usc = self.usc_ref(ascs, i);
                for object_group_count in 0..usc.obj_num_signal_groups as usize {
                    let grp_idx = usc.obj_signal_groups_indices[object_group_count] as usize;

                    if self.get_on_off_flag(
                        usc.m_signal_group_type[grp_idx].first_sig_idx as i32 + prev_signals,
                    ) == 0
                    {
                        continue;
                    }
                    obj_sig_grp_offset += 1;
                }
                prev_signals = usc.m_n_usac_channels as i32;
            }

            for i in 0..self.num_obj_signal_groups[sub_stream_index] as usize {
                let grp_idx = self.usc_ref(ascs, i).obj_signal_groups_indices[i] as usize;

                if self.get_on_off_flag(
                    self.usc_ref(ascs, sub_stream_index).m_signal_group_type[grp_idx].first_sig_idx
                        as i32
                        + prev_signals,
                ) == 0
                {
                    continue;
                }
                if let Some(r) = rs!(self).hg_vbap_renderer[obj_sig_grp_offset].take() {
                    gvbap_renderer_close(r);
                }
                /* FDKmemmove(hgVBAPRenderer + off, hgVBAPRenderer + off + 1, ...) */
                rs!(self).hg_vbap_renderer[obj_sig_grp_offset..].rotate_left(1);
                rs!(self).hg_vbap_renderer[TP_MPEGH_MAX_SIGNAL_GROUPS - 1] = None;
            }
            /* Do not clear numObjSignalGroups, required for next substream close. */
        }

        if sub_stream_index == 0 {
            fdk_delay_destroy(&mut rs!(self).mpeg_h_rendered_delay);
        }
    }

    /// `applyDownmixMatrixPerSignalGroup(_p, self, pUsacConfig, p_buffer, aacChannelOffsetIdx)`:
    /// decodes the transmitted downmix matrices and EQs of the channel signal groups of
    /// `pUsacConfig[stream_index]` into the format converter (`pFormatConverter[0]`).
    ///
    /// The downmix matrix set is read from `pTimeData2` like in C++ (see
    /// [`store_downmix_matrix_set`]). The C++ code keeps `eqConfig` in `p_buffer`
    /// (`workBufferCore2`); here it is a local structure (reused for all signal groups like the
    /// C++ memory).
    fn apply_downmix_matrix_per_signal_group(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        stream_index: usize,
        aac_channel_offset_idx: i32,
    ) -> AacDecoderError {
        let usc = self.usc_ref(ascs, stream_index);
        let usc_index = self.p_usac_config[stream_index].expect("pUsacConfig");
        let target_layout = self.target_layout;
        let Some(fc) = rs!(self).p_format_converter.as_deref_mut() else {
            return AAC_DEC_OK;
        };
        let p = &mut *fc.member;

        if p.num_total_input_channels == 0 {
            return AAC_DEC_OK;
        }

        let mut eq_config = EqConfigStruct::default();
        let mut offset = 0usize; /* dmx_sorted / eqIndexVec_sorted */
        for grp in 0..usc.bs_num_signal_groups as usize {
            let sg = &usc.m_signal_group_type[grp];
            /* Skip signal groups that are not active. */
            /* (getOnOffFlag(); pFormatConverter[0] is borrowed) */
            if self
                .signal_skipped
                .get((sg.first_sig_idx as i32 + aac_channel_offset_idx) as usize)
                .is_some_and(|&f| f != 0)
            {
                continue;
            }

            if sg.b_use_custom_downmix_matrix != 0 {
                /* Decode downmix matrix per signal group */
                let mut input_config =
                    [SpeakerInformation::default(); FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS];
                let mut output_config_geo =
                    [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
                let mut num_output_ch = 0;
                let mut num_out_lfes = 0;
                let mut output_config =
                    [SpeakerInformation::default(); FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS];

                /* groupsDownmixMatrixSet was written by aacDecoder_ParseDmxMatrixCallback() into
                 * borrowed memory from self->pTimeData2 */
                let set = load_downmix_matrix_set(&self.p_time_data2);
                let dmx_index = set.downmix_matrix[grp] as usize;
                debug_assert!(set.downmix_matrix_size[dmx_index] > 0);
                let mem = set.downmix_matrix_memory[dmx_index];
                let mut bitbuf =
                    FdkBitStream::reader(&mem, set.downmix_matrix_size[dmx_index] as u32);

                let speakers = asc_signal_group_speakers(ascs, usc_index, grp);
                for n in 0..sg.count as usize {
                    input_config[n].azimuth = speakers[n].az;
                    input_config[n].elevation = speakers[n].el;
                    input_config[n].is_lfe = speakers[n].lfe as i16;
                }

                if cicp2geometry_get_geometry_from_cicp(
                    target_layout,
                    Some(&mut output_config_geo),
                    &mut num_output_ch,
                    &mut num_out_lfes,
                ) != 0
                {
                    return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                }

                for n in 0..(num_output_ch + num_out_lfes).max(0) as usize {
                    output_config[n].azimuth = output_config_geo[n].az as i16;
                    output_config[n].elevation = output_config_geo[n].el as i16;
                    output_config[n].is_lfe = output_config_geo[n].lfe as i16;
                }

                let params = p.fc_params.as_mut().expect("fcParams");
                if decode_downmix_matrix(
                    sg.layout as i32,
                    sg.count as i32,
                    &mut input_config,
                    target_layout,
                    num_output_ch + num_out_lfes,
                    &mut output_config,
                    &mut bitbuf,
                    &mut params.dmx_mtx_sorted[offset..],
                    &mut eq_config,
                ) != 0
                {
                    return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                }
                format_converter_dmx_matrix_exponent(p);
                if format_converter_set_eqs(
                    offset,
                    eq_config.num_eqs,
                    Some(&eq_config.eq_params),
                    &eq_config.eq_map,
                    grp as u32,
                    p,
                ) != 0
                {
                    return AAC_DEC_UNSUPPORTED_FORMAT;
                }
                p.amount_of_added_dmx_matrices_and_equalizers += p.num_input_channels[grp];
            }
            offset += sg.count as usize * p.num_output_channels as usize;
        }

        AAC_DEC_OK
    }

    /// `CAacDecoder_InitRenderer(self, pUsc, samplingFrequency, samplesPerFrame, elementOffset,
    /// p_buffer, aacChannelOffsetIdx, isLastSubstream)`: initialization of the renderer.
    /// `p_buffer` is `workBufferCore2`.
    fn caac_decoder_init_renderer(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        stream_index: usize,
        sampling_frequency: i32,
        samples_per_frame: i32,
        element_offset: usize,
        aac_channel_offset_idx: i32,
        is_last_substream: bool,
    ) -> AacDecoderError {
        let mut err = AAC_DEC_OK;
        let usc = self.usc_ref(ascs, stream_index);
        let usc_index = self.p_usac_config[stream_index].expect("pUsacConfig");

        'bail: {
            if self.target_layout == 0 {
                if stream_index == 0 {
                    self.target_layout_config = usc.reference_layout as i32;
                }
            } else {
                self.target_layout_config = self.target_layout;
            }
            /* Make reference layout available for post processing. */
            self.stream_info.reference_layout = self.usc_ref(ascs, 0).reference_layout;

            /* Check if the cicp and the number of rendered channels is supported. */
            if self.target_layout_config >= 0 {
                let mut num_rendered_channels = 0;
                if cicp2geometry_get_num_channels_from_cicp(
                    self.target_layout_config,
                    &mut num_rendered_channels,
                ) != 0
                {
                    err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                    break 'bail;
                }
                if num_rendered_channels > 24 {
                    err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                    break 'bail;
                }
            }

            if self.target_layout_config != -1 {
                {
                    let mode = IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT;
                    let mut out_geo = [Cicp2GeometryChannelGeometry::default(); 32];
                    let mut in_geo = [Cicp2GeometryChannelGeometry::default(); 32];

                    let mut chan = 0;
                    let mut lfe = 0;

                    /* Target layout */
                    let cicp_index_out = self.target_layout_config;
                    let no_out_channels =
                        cicp2geometry_get_num_channels_from_cicp_noerr(cicp_index_out);

                    /* Sanity check for the current implementation of the FormatConverter: Output
                     * channels */
                    if no_out_channels > FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS {
                        err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                        break 'bail;
                    }

                    {
                        let geo: &mut [Cicp2GeometryChannelGeometry;
                                 CICP2GEOMETRY_MAX_LOUDSPEAKERS] =
                            (&mut out_geo[..CICP2GEOMETRY_MAX_LOUDSPEAKERS]).try_into().unwrap();
                        let cicp_err = cicp2geometry_get_geometry_from_cicp(
                            cicp_index_out,
                            Some(geo),
                            &mut chan,
                            &mut lfe,
                        );
                        if cicp_err != 0 {
                            err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                            break 'bail;
                        }
                    }

                    /* create FormatConverter */
                    if stream_index == 0 {
                        debug_assert!(rs!(self).p_format_converter.is_none());
                        let (fc_err, fc) = fc_create(
                            mode,
                            &out_geo,
                            no_out_channels as u32,
                            sampling_frequency as u32,
                            samples_per_frame as u32,
                        );
                        rs!(self).p_format_converter = fc;
                        if fc_err != 0 {
                            err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                            break 'bail;
                        }
                    }

                    /* Set PAS and AES */
                    let (aes, pas) = if usc.passive_downmix_flag != 0 {
                        /* passive downmix config */
                        (0, 0)
                    } else {
                        /* active downmix config */
                        (7, usc.phase_align_strength as u32)
                    };

                    /* Set immersive flag */
                    if usc.immersive_downmix_flag != 0 && stream_index == 0 {
                        if let Some(fc) = rs!(self).p_format_converter.as_deref_mut() {
                            fc_set_immersive_downmix_flag(fc, 1);
                        }
                    }

                    for grp in 0..usc.bs_num_signal_groups as usize {
                        /* AudioChannel is type 0 */
                        if usc.m_signal_group_type[grp].type_ == 0 {
                            /* Skip signal groups that are not active. */
                            if self.get_on_off_flag(
                                usc.m_signal_group_type[grp].first_sig_idx as i32
                                    + aac_channel_offset_idx,
                            ) == 0
                            {
                                continue;
                            }

                            let Some(fc) = rs!(self).p_format_converter.as_deref_mut() else {
                                /* NULL pointer dereference in C++ */
                                err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                                break 'bail;
                            };
                            if fc_is_open(fc) {
                                err = AAC_DEC_PARSE_ERROR;
                                break 'bail;
                            }
                            let no_in_channels = usc.m_signal_group_type[grp].count as u32;
                            fc.num_signals_total += no_in_channels;

                            if fc.num_signals_total > FORMAT_CONVERTER_MAX_CHANNELS as u32 {
                                err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                                if self.h_ui_manager.asi.is_main_stream[stream_index] == 0 {
                                    /* Need to parse ASI first. Return OK to avoid skipping
                                     * possible ASI bit stream data. */
                                    return AAC_DEC_DECODE_FRAME_ERROR;
                                }
                                break 'bail;
                            }

                            /* cicp2geometry */
                            let speakers = asc_signal_group_speakers(ascs, usc_index, grp);
                            for i in 0..usc.m_signal_group_type[grp].count as usize {
                                /* Copy Geometry from speakers parser structure in case of unknown
                                 * channel. */
                                in_geo[i].cicp_loudspeaker_index = -1;
                                in_geo[i].az = speakers[i].az as i32;
                                in_geo[i].el = speakers[i].el as i32;
                                in_geo[i].lfe = speakers[i].lfe as i32;
                                in_geo[i].screen_relative = -1;
                                in_geo[i].has_distance = 0;
                                in_geo[i].distance = 0;
                                in_geo[i].has_loudspeaker_calibration_gain = 0;
                                in_geo[i].loudspeaker_calibration_gain = 0;
                                in_geo[i].loudspeaker_type = CICP2GEOMETRY_LOUDSPEAKER_UNKNOWN;
                            }

                            /* config input setup */
                            let fc_err = fc_add_input_setup(fc, &in_geo, no_in_channels);
                            if fc_err != 0 {
                                err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                                break 'bail;
                            }
                        }
                    }

                    let Some(fc) = rs!(self).p_format_converter.as_deref_mut() else {
                        err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                        break 'bail;
                    };
                    /* Sanity check for the current implementation of the FormatConverter: Input
                     * channels */
                    if fc.num_signals_total > FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS {
                        err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                        break 'bail;
                    }

                    if stream_index == 0 {
                        fc_set_aes(fc, aes);
                        fc_set_pas(fc, pas);
                    }

                    /* Open Format converter: This will generate the downmix matrix, EQs ... */
                    if is_last_substream {
                        let error = fc_open(fc);
                        if error != 0 {
                            err = AAC_DEC_OUT_OF_MEMORY;
                            break 'bail;
                        }

                        /* Applying the decoded downmix matrix and EQs for each signal group */
                        let error = self.apply_downmix_matrix_per_signal_group(
                            ascs,
                            stream_index,
                            aac_channel_offset_idx,
                        );
                        if error != 0 {
                            err = AAC_DEC_OUT_OF_MEMORY;
                            break 'bail;
                        }
                    }
                }

                if stream_index == 0 {
                    let mut delay_temp: i16 = 0;
                    /* delay compensation on rendered signal for different sampling rates delay
                    is (768*Fs/48000 - delayUpsampler) or (768*Fs/44100 - delayUpsampler)
                    sampling rates are from table "Allowed Sampling Rates and Resampling Ratios"
                    valid for Profile Level <= 4 */
                    let mut delay: i32 = 3 * 256 + TD_UPSAMPLER_MAX_DELAY; /* total delay */
                    let sample_rate_converter_fac_upsampling = match sampling_frequency {
                        14700 | 16000 => {
                            delay -= 3 * 256;
                            TD_FAC_UPSAMPLE_3_1
                        }
                        22050 | 24000 => {
                            delay -= 2 * 256;
                            TD_FAC_UPSAMPLE_2_1
                        }
                        29400 | 32000 => {
                            delay -= (3 * 256) / 2;
                            TD_FAC_UPSAMPLE_3_2
                        }
                        44100 | 48000 => {
                            delay -= 256;
                            TD_FAC_UPSAMPLE_1_1
                        }
                        _ => {
                            err = AAC_DEC_UNSUPPORTED_FORMAT;
                            break 'bail;
                        }
                    };
                    for ch in 0..24 {
                        delay_temp = td_upsampler_init(
                            sample_rate_converter_fac_upsampling,
                            &mut rs!(self).mpeg_h_sample_rate_converter_filter_states[ch],
                        );
                    }
                    delay -= delay_temp as i32;
                    debug_assert!(delay >= 0);
                    /* Get number of rendered channels */
                    let mut num_rendered_channels = 0;
                    if cicp2geometry_get_num_channels_from_cicp(
                        self.target_layout_config,
                        &mut num_rendered_channels,
                    ) != 0
                    {
                        err = AAC_DEC_UNSUPPORTED_FORMAT;
                        break 'bail;
                    }
                    let delay_err = fdk_delay_create(
                        &mut rs!(self).mpeg_h_rendered_delay,
                        delay as u16,
                        num_rendered_channels as u8,
                    ); /* (24) could be reduced to nOutputChannels */
                    if delay_err != 0 {
                        err = AAC_DEC_OUT_OF_MEMORY;
                        break 'bail;
                    }
                }

                /* Open Object Renderer instance */
                {
                    let mut num_channels = 0;
                    let mut num_lfe = 0;

                    let mut out_geometry_info =
                        [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
                    /* get geometry of loudspeakers by targetLayout (CICP index see also
                     * ISO-IEC_23001-8) */
                    {
                        let c_err = cicp2geometry_get_geometry_from_cicp(
                            self.target_layout_config,
                            Some(&mut out_geometry_info),
                            &mut num_channels,
                            &mut num_lfe,
                        );
                        if c_err != CICP2GEOMETRY_OK {
                            err = AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                            break 'bail;
                        }
                    }

                    /* check if production metadata config present */
                    let mut prod_metadata_present = 0;
                    for el in 0..usc.m_usac_num_elements as usize {
                        if usc.element[el].ext_element.usac_ext_element_type
                            == ID_EXT_ELE_PROD_METADATA
                        {
                            prod_metadata_present = 1;
                        }
                    }

                    /* open opject renderer instances */
                    let mut obj_grp_cnt = 0usize;
                    let mut signals_idx = 0i32;
                    let mut obj_grp_cnt_offset = 0usize;

                    /* Account signal groups of previous substreams */
                    for i in 0..stream_index {
                        let usc_i = self.usc_ref(ascs, i);
                        for object_group_count in 0..usc_i.obj_num_signal_groups as usize {
                            let grp_idx =
                                usc_i.obj_signal_groups_indices[object_group_count] as usize;

                            if self.get_on_off_flag(
                                usc_i.m_signal_group_type[grp_idx].first_sig_idx as i32
                                    + signals_idx,
                            ) == 0
                            {
                                continue;
                            }
                            obj_grp_cnt_offset += 1;
                        }
                        signals_idx += usc_i.m_n_usac_channels as i32;
                    }

                    for el_ in 0..usc.m_usac_num_elements as usize {
                        let el = element_offset + el_;
                        if usc.element[el_].ext_element.usac_ext_element_type
                            == ID_EXT_ELE_OBJ_METADATA
                        {
                            debug_assert!(rs!(self).hg_vbap_renderer
                                [obj_grp_cnt + obj_grp_cnt_offset]
                                .is_none());

                            /* Skip signal groups that are not active. */
                            if self.get_on_off_flag(signals_idx) == 0 {
                                if is_channel_element(self.elements[el]) {
                                    signals_idx +=
                                        if self.elements[el] == ID_USAC_CPE { 2 } else { 1 };
                                }
                                continue;
                            }

                            let oam = &usc.element[el_].ext_element.ext_config.oam;
                            if gvbap_renderer_open(
                                &mut rs!(self).hg_vbap_renderer[obj_grp_cnt + obj_grp_cnt_offset],
                                oam.num_object_signals as i32, /* number of objects for current signal group */
                                samples_per_frame,             /* core frame length */
                                oam.oam_frame_length as i32, /* oam frame length for current signal group */
                                &out_geometry_info,
                                num_channels + num_lfe,
                                self.target_layout_config,
                                oam.has_uniform_spread as i32,
                                if prod_metadata_present != 0 {
                                    GVBAP_ENHANCED
                                } else {
                                    GVBAP_LEGACY
                                },
                            ) != 0
                            {
                                err = AAC_DEC_DECODE_FRAME_ERROR;
                                /* save the amount of object signal groups for propper closing of
                                 * object renderer instances */
                                self.num_obj_signal_groups[stream_index] = (obj_grp_cnt + 1) as u8;
                                break 'bail;
                            }
                            obj_grp_cnt += 1;
                        }
                        if is_channel_element(self.elements[el]) {
                            signals_idx += if self.elements[el] == ID_USAC_CPE { 2 } else { 1 };
                        }
                    }
                    /* save the amount of object signal groups for later closing of object renderer
                     * instances */
                    self.num_obj_signal_groups[stream_index] = obj_grp_cnt as u8;
                    if (usc.obj_num_signal_groups != self.num_obj_signal_groups[stream_index])
                        && self.use_element_skipping == 0
                    {
                        err = AAC_DEC_DECODE_FRAME_ERROR;
                        break 'bail;
                    }
                }
            }

            return err;
        }

        self.caac_decoder_deinit_renderer(ascs, stream_index);

        err
    }

    /// `CAacDecoder_DeInit(self, subStreamIndex, configMode)`: revert `CAacDecoder_Init()`
    fn caac_decoder_deinit(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        sub_stream_index: usize,
        config_mode: u8,
    ) {
        /* the IGF configuration may change (frame pipelining) */
        self.igf_stale_reads = None;
        if sub_stream_index == 0 {
            for ch in 0..28 {
                if self.ch_info_alloc[ch] {
                    /* pDynData, pComStaticData (incl. pWorkBufferCore1, cplxPredictionData,
                     * pJointStereoData) and pComData are freed */
                    self.com_static_data[ch] = None;
                    self.com_static_link[ch] = None;
                }
                if self.st_info_alloc[ch] {
                    /* concealment spectrum, overlap buffer, hArCo */
                    self.st_info_alloc[ch] = false;
                    self.cpe_static_link[ch] = None;
                }
                if self.ch_info_alloc[ch] {
                    self.ch_info_alloc[ch] = false;
                }
            }

            for el in 0..AAC_MAX_ELEMENTS {
                self.cpe_static_data[el] = None;
            }

            if self.flags[sub_stream_index] & AC_MPEGH3DA != 0 {
                for grp in 0..TP_MPEGH_MAX_SIGNAL_GROUPS {
                    if let Some(m) = self.p_mct_dec[grp].take() {
                        cmct_destroy(Some(m));
                    }
                }
            }

            for ch in 0..AAC_MAX_SIGNALS {
                stft_filterbank_close(&mut rs!(self).stft_filterbank_analysis[ch]);
                stft_filterbank_close(&mut rs!(self).stft_filterbank_synthesis[ch]);
            }

            if self.flags[sub_stream_index] & AC_MPEGH3DA != 0 {
                self.caac_decoder_deinit_renderer(ascs, sub_stream_index);
            }
        }

        if sub_stream_index == 0 {
            self.aac_channels = 0;
            self.stream_info.aac_sample_rate = 0;
            self.stream_info.sample_rate = 0;
            self.stream_info.sample_rate_before_rs = 0;
            /* This samplerate value is checked for configuration change, not the others above. */
            self.sampling_rate_info[sub_stream_index].sampling_rate = 0;

            for i in 1..TPDEC_MAX_TRACKS {
                self.num_usac_elements[i] = 0;
                self.alloc_channels[i] = 0;
                self.flags[i] = 0;
                self.p_usac_config[i] = None;
            }
        } else {
            self.aac_channels -= self.alloc_channels[sub_stream_index];

            if config_mode & 0x08 != 0 {
                self.num_usac_elements[sub_stream_index] = 0;
                self.alloc_channels[sub_stream_index] = 0;
                self.flags[sub_stream_index] = 0;
                self.p_usac_config[sub_stream_index] = None;

                let mut element_offset = 0;
                for i in 0..sub_stream_index {
                    element_offset += self.num_usac_elements[i];
                }
                self.elements[element_offset as usize] = ID_END;
            }
        }
    }

    /// `CAacDecoder_AcceptFlags(self, asc, flags, elFlags, streamIndex, elementOffset)`: accept
    /// flags and element flags
    fn caac_decoder_accept_flags(
        &mut self,
        asc: &CSAudioSpecificConfig,
        flags: u32,
        el_flags: &[u32; AAC_MAX_ELEMENTS],
        stream_index: usize,
        element_offset: usize,
    ) {
        if flags & AC_MPEGH3DA != 0 {
            let els = element_offset..element_offset + asc.m_sc.m_usac_num_elements as usize;
            self.el_flags[els.clone()].copy_from_slice(&el_flags[els]);
        } else {
            self.el_flags = *el_flags;
        }

        self.flags[stream_index] = flags;
    }

    /// `CAacDecoder_CtrlCFGChange(self, flushStatus, flushCnt, buildUpStatus, buildUpCnt)`: set
    /// config change parameters.
    pub fn caac_decoder_ctrl_cfg_change(
        &mut self,
        flush_status: u8,
        flush_cnt: i8,
        build_up_status: u8,
        build_up_cnt: i8,
    ) -> AacDecoderError {
        self.flush_status = flush_status;
        self.flush_cnt = flush_cnt;
        self.build_up_status = build_up_status;
        self.build_up_cnt = build_up_cnt;

        AAC_DEC_OK
    }

    /// `CAacDecoder_FreeMem(self, subStreamIndex, configMode)`: free config dependent AAC memory.
    pub fn caac_decoder_free_mem(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        sub_stream_index: usize,
        config_mode: u8,
    ) -> AacDecoderError {
        self.caac_decoder_deinit(ascs, sub_stream_index, config_mode);
        AAC_DEC_OK
    }

    /// `CAacDecoder_Close(self)`: destroy aac decoder
    pub fn caac_decoder_close(&mut self, ascs: &[CSAudioSpecificConfig]) {
        /* Substream 0 needs to be the last. */
        self.caac_decoder_deinit(ascs, 0, 0);
    }

    /// `CAacDecoder_Init(self, asc, configMode, configChanged)`: initialization of the decoder
    /// instance for the config `ascs[asc_index]`.
    ///
    /// Returns 0 for success, <>0 for unsupported configurations.
    pub fn caac_decoder_init(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        asc_index: usize,
        config_mode: u8,
        config_changed: &mut u8,
    ) -> AacDecoderError {
        let asc = &ascs[asc_index];
        let mut err = AAC_DEC_OK;
        let asc_channels: i32;
        let mut asc_changed = 0;
        let mut element_offset: usize;
        let mut aac_channels_offset: i32;
        let mut aac_channels_offset_idx: i32;
        let mut stream_index: usize = 0;
        if asc.m_aot == AOT_MPEGH3DA {
            stream_index = asc.m_sc.sub_stream_index as usize;
        }

        let mut flags: u32;
        let mut el_flags = [0u32; AAC_MAX_ELEMENTS];

        let downscale_factor = self.downscale_factor;

        /* the IGF configuration may change (frame pipelining) */
        self.igf_stale_reads = None;

        self.aac_out_data_headroom = 3;

        if asc.m_aot == AOT_MPEGH3DA {
            self.aac_out_data_headroom = 8;
        }

        // set profile and check for supported aot
        let init_render_mode: AacdecRenderMode = match asc.m_aot {
            AOT_MPEGH3DA => AACDEC_RENDER_IMDCT,
            _ => return AAC_DEC_UNSUPPORTED_AOT,
        };

        cprogram_config_init(&mut self.pce);

        /* set channels */
        match asc.m_channel_configuration {
            0 => match asc.m_aot {
                AOT_MPEGH3DA => {
                    asc_channels = asc.m_sc.num_audio_channels as i32
                        + asc.m_sc.num_audio_objects as i32
                        + asc.m_sc.num_hoa_transport_channels as i32;
                }
                _ => return AAC_DEC_UNSUPPORTED_CHANNELCONFIG,
            },
            1..=6 => {
                asc_channels = asc.m_channel_configuration as i32;
            }
            11 => {
                asc_channels = 7;
            }
            7 | 12 | 14 => {
                asc_channels = 8;
            }
            13 => {
                /* 22.2 setup */
                asc_channels = 24;
            }
            _ => return AAC_DEC_UNSUPPORTED_CHANNELCONFIG,
        }

        'bail: {
            if (asc.m_aot == AOT_USAC) || (asc.m_aot == AOT_MPEGH3DA) {
                /* Check if Transport Decoder element list will fit into Audio decoder element
                 * list. */
                if asc.m_sc.m_usac_num_elements as usize > AAC_MAX_ELEMENTS {
                    break 'bail;
                }
            }

            aac_channels_offset = 0;
            aac_channels_offset_idx = 0;
            element_offset = 0;
            if asc.m_aot == AOT_MPEGH3DA {
                self.update_on_off_flags(ascs);
                for i in 0..stream_index {
                    if self.p_usac_config[i].is_none()
                        || (self.alloc_channels[i] == 0 && (config_mode & AC_CM_ALLOC_MEM != 0))
                    {
                        /* Its not possible to allocate sub streams before a main stream has been
                         * allocated. */
                        return AAC_DEC_UNSUPPORTED_FORMAT;
                    }
                    for j in 0..self.usc_ref(ascs, i).m_n_usac_channels as i32 {
                        if self.get_on_off_flag(aac_channels_offset_idx + j) != 0 {
                            aac_channels_offset += 1;
                        }
                    }
                    aac_channels_offset_idx += self.usc_ref(ascs, i).m_n_usac_channels as i32;
                }
                for i in 0..stream_index {
                    element_offset += self.num_usac_elements[i] as usize;
                }
            }
            if asc_channels <= 0 {
                return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
            }
            if (asc_channels + aac_channels_offset_idx) > (28 * 2) {
                return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
            }
            if config_mode & AC_CM_ALLOC_MEM != 0 && asc.m_aot == AOT_MPEGH3DA {
                /* Check max signal count for channels and objects. */
                let mut n_other = 0;
                let mut n_obj = 0;

                for sb in 0..TPDEC_MAX_TRACKS {
                    let Some(usc_sb) = self.usc(ascs, sb) else { continue };

                    for sg in 0..usc_sb.bs_num_signal_groups as usize {
                        let n_sig = usc_sb.m_signal_group_type[sg].count as i32;
                        if usc_sb.m_signal_group_type[sg].type_ == 1 {
                            n_obj += n_sig;
                        } else {
                            n_other += n_sig;
                        }
                    }
                }

                if (n_other == 0 && n_obj > 24) || ((n_other > 0) && ((n_obj + n_other) > 16)) {
                    if n_obj + n_other > (28 * 2) {
                        return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                    }

                    self.use_element_skipping = 1;
                    self.update_on_off_flags(ascs);
                } else {
                    self.use_element_skipping = 0;
                    self.update_on_off_flags(ascs);
                }
                self.tp_set_param(TPDEC_PARAM_USE_ELEM_SKIPPING, self.use_element_skipping as i32);
            }

            /* Set syntax flags */
            flags = 0;
            if asc.m_aot == AOT_MPEGH3DA {
                for el in element_offset..element_offset + asc.m_sc.m_usac_num_elements as usize {
                    el_flags[el] = 0;
                }
            } else {
                el_flags = [0; AAC_MAX_ELEMENTS];
            }

            if (asc.m_channel_configuration > 0)
                || (asc.m_aot == AOT_MPEGH3DA)
                || (asc.m_aot == AOT_USAC)
            {
                if (asc.m_aot == AOT_USAC) || (asc.m_aot == AOT_MPEGH3DA) {
                    /* copy pointer to usac config */
                    self.p_usac_config[stream_index] = Some(asc_index);
                    self.update_on_off_flags(ascs);
                    if self.num_usac_elements[stream_index] != asc.m_sc.m_usac_num_elements as i32 {
                        asc_changed = 1;
                    }

                    if config_mode & AC_CM_ALLOC_MEM != 0 {
                        self.num_usac_elements[stream_index] = asc.m_sc.m_usac_num_elements as i32;
                    }

                    let usc = &asc.m_sc;
                    for el_ in 0..usc.m_usac_num_elements as usize {
                        let el = el_ + element_offset;
                        if self.elements[el] != usc.element[el_].usac_element_type {
                            asc_changed = 1;
                        }
                        if config_mode & AC_CM_ALLOC_MEM != 0 {
                            self.elements[el] = usc.element[el_].usac_element_type;
                        }

                        let e = &usc.element[el_];
                        el_flags[el] |= if e.m_noise_filling != 0 { AC_EL_USAC_NOISE } else { 0 };
                        el_flags[el] |=
                            if e.usac_element_type == ID_USAC_LFE { AC_EL_USAC_LFE } else { 0 };
                        el_flags[el] |=
                            if e.enhanced_noise_filling != 0 { AC_EL_ENHANCED_NOISE } else { 0 };
                        el_flags[el] |= if e.igf_use_enf != 0 { AC_EL_IGF_USE_ENF } else { 0 };
                        el_flags[el] |=
                            if e.igf_after_tns_synth != 0 { AC_EL_IGF_AFTER_TNS } else { 0 };
                        el_flags[el] |=
                            if e.igf_independent_tiling != 0 { AC_EL_IGF_INDEP_TILING } else { 0 };
                        el_flags[el] |= if e.fullband_lpd != 0 { AC_EL_FULLBANDLPD } else { 0 };
                        el_flags[el] |=
                            if e.lpd_stereo_index != 0 { AC_EL_LPDSTEREOIDX } else { 0 };
                        el_flags[el] |=
                            if e.usac_element_type == ID_USAC_LFE { AC_EL_LFE } else { 0 };
                        if e.usac_element_type == ID_USAC_CPE {
                            el_flags[el] |= AC_EL_USAC_CP_POSSIBLE;
                        }
                    }

                    self.has_audio_pre_roll = 0;
                    if usc.m_usac_num_elements != 0 {
                        self.has_audio_pre_roll =
                            usc.element[0].ext_element.usac_ext_element_has_audio_pre_roll;
                    }
                    if config_mode & AC_CM_ALLOC_MEM != 0 {
                        let mut num_elements = 0;
                        for i in 0..TPDEC_MAX_TRACKS {
                            num_elements += self.num_usac_elements[i];
                        }
                        self.elements[num_elements as usize] = ID_END;
                    }
                }

                if asc.m_aot == AOT_MPEGH3DA {
                    let usc = &asc.m_sc;
                    let mut i = aac_channels_offset_idx;

                    for ch in 0..usc.m_n_usac_channels as i32 {
                        self.ch_mapping[(aac_channels_offset_idx + ch) as usize] = 255;
                    }

                    for el_ in 0..usc.m_usac_num_elements as usize {
                        let el_cfg = &usc.element[el_];

                        match el_cfg.usac_element_type {
                            ID_USAC_SCE | ID_USAC_LFE => {
                                /* assign to next free index */
                                for j in 0..AAC_MAX_SIGNALS {
                                    if self.ch_mapping[j] == 255 {
                                        self.ch_mapping[j] = i as u8;
                                        i += 1;
                                        break;
                                    }
                                }
                            }
                            ID_USAC_CPE => {
                                /* first CPE channel, assign to next free index + shift */
                                for j in 0..AAC_MAX_SIGNALS {
                                    if self.ch_mapping[j] == 255 {
                                        self.ch_mapping[j] = i as u8;
                                        i += 1;
                                        break;
                                    }
                                }

                                /* second CPE channel, assign to next free index + shift */
                                let mut shift = if el_cfg.shift_index1 != 0 {
                                    el_cfg.shift_channel1 as i32 + 1
                                } else {
                                    0
                                };
                                let mut j = 0usize;
                                while j < AAC_MAX_SIGNALS {
                                    if self.ch_mapping[j] == 255 {
                                        while shift > 0 {
                                            j += 1;
                                            if (j as i32 + shift) >= (28 * 2) {
                                                return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                                            }
                                            if self.ch_mapping[j] == 255 {
                                                shift -= 1;
                                            }
                                        }
                                        self.ch_mapping[j] = i as u8;
                                        i += 1;
                                        break;
                                    }
                                    j += 1;
                                }
                            }
                            _ => {}
                        }
                    }
                }
            }

            if self.stream_info.channel_config != asc.m_channel_configuration as i32 {
                if config_mode & AC_CM_ALLOC_MEM != 0 {
                    self.stream_info.channel_config = asc.m_channel_configuration as i32;
                }
                asc_changed = 1;
            }

            if self.stream_info.aot != asc.m_aot {
                if config_mode & AC_CM_ALLOC_MEM != 0 {
                    self.stream_info.aot = asc.m_aot;
                }
                asc_changed = 1;
            }

            if (asc.m_samples_per_frame as i32) % downscale_factor as i32 != 0 {
                return AAC_DEC_UNSUPPORTED_SAMPLINGRATE; /* frameSize/dsf must be an integer number */
            }

            self.stream_info.bit_rate = 0;

            /* --------- vcb11 ------------ */
            flags |= if asc.m_vcb11_flag != 0 { AC_ER_VCB11 } else { 0 };
            if asc.m_vcb11_flag != 0 {
                return AAC_DEC_UNSUPPORTED_ER_FORMAT;
            }

            /* ---------- rvlc ------------ */
            flags |= if asc.m_rvlc_flag != 0 { AC_ER_RVLC } else { 0 };
            if asc.m_rvlc_flag != 0 {
                return AAC_DEC_UNSUPPORTED_ER_FORMAT;
            }

            /* ----------- hcr ------------ */
            flags |= if asc.m_hcr_flag != 0 { AC_ER_HCR } else { 0 };
            if asc.m_hcr_flag != 0 {
                return AAC_DEC_UNSUPPORTED_ER_FORMAT;
            }

            flags |= if asc.m_ep_config >= 0 { AC_ER } else { 0 };

            if asc.m_aot == AOT_MPEGH3DA {
                flags |= AC_MPEGH3DA;
            }

            if (asc.m_ep_config >= 0) && (asc.m_channel_configuration <= 0) {
                /* we have to know the number of channels otherwise no decoding is possible */
                return AAC_DEC_UNSUPPORTED_ER_FORMAT;
            }

            self.stream_info.ep_config = asc.m_ep_config;

            if asc.m_ep_config > 1 {
                return AAC_DEC_UNSUPPORTED_ER_FORMAT;
            }

            /* Check if samplerate changed. */
            if (self.sampling_rate_info[stream_index].sampling_rate != asc.m_sampling_frequency)
                || (self.stream_info.aac_samples_per_frame
                    != asc.m_samples_per_frame as i32 / downscale_factor as i32)
            {
                asc_changed = 1;

                if config_mode & AC_CM_ALLOC_MEM != 0 {
                    /* Update samplerate info. */
                    let error = get_sampling_rate_info(
                        &mut self.sampling_rate_info[stream_index],
                        asc.m_samples_per_frame,
                        asc.m_sampling_frequency_index as u32,
                        asc.m_sampling_frequency,
                    );
                    if error != AAC_DEC_OK {
                        return error;
                    }
                    self.stream_info.aac_sample_rate = self.sampling_rate_info[0].sampling_rate
                        as i32
                        / self.downscale_factor as i32;
                    self.stream_info.aac_samples_per_frame =
                        asc.m_samples_per_frame as i32 / self.downscale_factor as i32;
                    if self.stream_info.aac_sample_rate <= 0 {
                        return AAC_DEC_UNSUPPORTED_SAMPLINGRATE;
                    }
                }
            }

            /* Check if amount of channels has changed. */
            if self.asc_channels[stream_index] != asc_channels {
                asc_changed = 1;
            }

            /* detect config change */
            if config_mode & AC_CM_DET_CFG_CHANGE != 0 {
                if asc_changed != 0 {
                    *config_changed = 1;
                }

                self.caac_decoder_accept_flags(asc, flags, &el_flags, stream_index, element_offset);

                return err;
            }

            if *config_changed != 0 {
                /* Allocate all memory structures for each channel */
                {
                    let mut ch = aac_channels_offset;
                    for ch_ in 0..asc_channels {
                        if asc.m_aot == AOT_MPEGH3DA {
                            let ch_idx = aac_channels_offset_idx + ch_;

                            if self.get_on_off_flag(ch_idx) == 0 {
                                continue;
                            }
                        }
                        if ch >= 28 {
                            break 'bail;
                        }
                        /* GetAacDecoderChannelInfo(ch) + pDynData */
                        self.p_aac_decoder_channel_info[ch as usize] =
                            get_aac_decoder_channel_info(ch as usize);
                        self.ch_info_alloc[ch as usize] = true;
                        self.com_static_link[ch as usize] = None;

                        ch += 1;
                    }

                    let mut ch_idx = aac_channels_offset_idx;
                    ch = aac_channels_offset;
                    let mut num_elements = 28 + 28;
                    if flags & (AC_MPEGH3DA | AC_USAC) != 0 {
                        num_elements = asc.m_sc.m_usac_num_elements as usize;
                    }
                    for el_ in 0..num_elements {
                        let el = element_offset + el_;

                        match self.elements[el] {
                            ID_SCE | ID_CPE | ID_LFE | ID_USAC_SCE | ID_USAC_CPE | ID_USAC_LFE => {
                                let el_channels = caac_decoder_get_el_channels(self.elements[el]);

                                if el_channels == 2 {
                                    /* The signal skip flag of both channels must be the same. */
                                    if self.get_on_off_flag(ch_idx)
                                        != self.get_on_off_flag(ch_idx + 1)
                                    {
                                        break 'bail;
                                    }
                                }

                                if self.get_on_off_flag(ch_idx) != 0 {
                                    if ch >= 28 {
                                        break 'bail;
                                    }
                                    let c = ch as usize;
                                    if !self.ch_info_alloc[c] {
                                        break 'bail;
                                    }
                                    let mut csd = Box::<CAacDecoderCommonStaticData>::default();
                                    if c == 0 {
                                        /* pComData = GetWorkBufferCore6(); pWorkBufferCore1 =
                                         * GetWorkBufferCore1() */
                                        self.work_buffer_core1 = get_work_buffer_core1();
                                    }
                                    if el_channels == 2 {
                                        csd.joint_stereo_data = Some(Box::default());
                                    }
                                    self.com_static_data[c] = Some(csd);
                                    self.com_static_link[c] = Some(c);

                                    if el_channels == 2 {
                                        if ch >= 28 - 1 {
                                            break 'bail;
                                        }
                                        if !self.ch_info_alloc[c + 1] {
                                            break 'bail;
                                        }
                                        self.com_static_link[c + 1] = Some(c);
                                    }

                                    ch += el_channels;
                                }
                                ch_idx += el_channels;
                            }
                            ID_USAC_EXT => {
                                let usc = self.usc_ref(ascs, stream_index);
                                if usc.element[el_].ext_element.usac_ext_element_type
                                    == ID_EXT_ELE_MCT
                                    && self.get_on_off_flag(ch_idx) != 0
                                {
                                    let grp = usc.sig_idx2_grp_idx
                                        [(ch_idx - aac_channels_offset_idx) as usize]
                                        as usize;
                                    let mut first_sig_idx = 0;
                                    let signals_in_group =
                                        usc.m_signal_group_type[grp].count as i32;
                                    let mut prev_signal_groups = 0usize;

                                    for i in 0..usc.m_signal_group_type[grp].first_sig_idx as i32 {
                                        if self.get_on_off_flag(i) != 0 {
                                            first_sig_idx += 1;
                                        }
                                    }

                                    for s in 0..stream_index {
                                        prev_signal_groups +=
                                            self.usc_ref(ascs, s).bs_num_signal_groups as usize;
                                    }

                                    let mct_err = cmct_initialize(
                                        &mut self.p_mct_dec[grp + prev_signal_groups],
                                        usc.element[el_].ext_element.ext_config.mct.mct_chan_mask,
                                        first_sig_idx,
                                        signals_in_group,
                                    );
                                    if mct_err != 0 {
                                        break 'bail;
                                    }
                                }
                            }
                            _ => {}
                        }

                        if self.elements[el] == ID_END {
                            break;
                        }
                    }

                    ch_idx = aac_channels_offset_idx;
                    ch = aac_channels_offset;
                    for _ in 0..asc_channels {
                        /* Allocate persistent channel memory */
                        if self.get_on_off_flag(ch_idx) != 0 {
                            let c = ch as usize;
                            if c >= 28 {
                                break 'bail;
                            }
                            self.p_aac_decoder_static_channel_info[c] =
                                get_aac_decoder_static_channel_info(c);
                            self.st_info_alloc[c] = true;
                            self.cpe_static_link[c] = None;
                            if flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 {
                                self.p_aac_decoder_static_channel_info[c].h_ar_co =
                                    Some(CArcoData::create());
                            }

                            ch += 1;
                        }
                        ch_idx += 1;
                    }

                    if flags & (AC_USAC | AC_MPEGH3DA) != 0 {
                        let mut complex_stereo_pred_possible = 0;
                        ch = aac_channels_offset;
                        ch_idx = aac_channels_offset_idx;
                        for el2_ in 0..asc.m_sc.m_usac_num_elements as usize {
                            let el2 = element_offset + el2_;
                            let mut el_ch = 0;

                            if self.elements[el2] == ID_USAC_CPE {
                                el_ch = 2;
                            } else if is_channel_element(self.elements[el2]) {
                                el_ch = 1;
                            }

                            if el_ch != 0 && self.get_on_off_flag(ch_idx) == 0 {
                                ch_idx += el_ch;
                                continue;
                            }

                            if el_flags[el2] & AC_EL_USAC_CP_POSSIBLE != 0 {
                                complex_stereo_pred_possible = 1;
                                if self.cpe_static_data[el2].is_none() {
                                    let mut cpe = get_cpe_persistent_data();
                                    cpe.joint_stereo_persistent_data.win_groups_prev = 1;
                                    self.cpe_static_data[el2] = Some(cpe);
                                }
                            }

                            for _ch2 in 0..el_ch as usize {
                                let c = ch as usize;
                                /* IGF decoder config */
                                /* virtualSpec = &pTimeData2[1024 * ch2] (see CChannelElementMem) */
                                let e = &asc.m_sc.element[el2_];
                                if e.enhanced_noise_filling != 0 {
                                    let sri = &self.sampling_rate_info[stream_index];
                                    let (st, chi) = (
                                        &mut self.p_aac_decoder_static_channel_info[c],
                                        &mut self.p_aac_decoder_channel_info[c],
                                    );
                                    iis_igf_dec_lib_init(
                                        &mut st.igf_static_data,
                                        &mut chi.igf_data,
                                        e.igf_start_index,
                                        e.igf_stop_index,
                                        e.igf_use_high_res,
                                        e.igf_use_whitening,
                                        sri.sampling_rate,
                                        self.stream_info.aac_samples_per_frame,
                                        sri.scale_factor_bands_long,
                                        sri.number_of_scale_factor_bands_long as i32,
                                        sri.scale_factor_bands_short,
                                        sri.number_of_scale_factor_bands_short as i32,
                                        e.enhanced_noise_filling,
                                        e.igf_after_tns_synth,
                                        e.igf_independent_tiling,
                                    );
                                }
                                /* Hook element specific cpeStaticData into channel specific
                                 * aacDecoderStaticChannelInfo (spectralCoeffs[ch2], specScale[ch2],
                                 * scratchBuffer and scratchBuffer2 are resolved by the element
                                 * functions) */
                                self.cpe_static_link[c] = if self.cpe_static_data[el2].is_some() {
                                    Some(el2)
                                } else {
                                    None
                                };
                                ch_idx += 1;
                                ch += 1;
                            } /* for each channel in current element */
                            if complex_stereo_pred_possible != 0 && el_ch == 2 {
                                /* needed once for all channels */
                                let owner = self.com_static_link[(ch - 1) as usize];
                                let Some(csd) =
                                    owner.and_then(|o| self.com_static_data[o].as_deref_mut())
                                else {
                                    break 'bail;
                                };
                                if csd.cplx_prediction_data.is_none() {
                                    csd.cplx_prediction_data = Some(get_cplx_prediction_data());
                                }
                            }
                            if el_ch > 0 {
                                self.p_aac_decoder_static_channel_info[(ch - el_ch) as usize]
                                    .nf_random_seed = 0x3039;
                                if self.elements[el2] == ID_USAC_CPE {
                                    self.p_aac_decoder_static_channel_info
                                        [(ch - el_ch + 1) as usize]
                                        .nf_random_seed = 0x10932;
                                }
                            }
                        } /* for each element */
                    }

                    /* Make allocated channel count persistent in decoder context. */
                    self.aac_channels = ch.max(self.aac_channels);
                    self.alloc_channels[stream_index] = ch - aac_channels_offset;
                }

                /* Make amount of signalled channels persistent in decoder context. */
                self.asc_channels[stream_index] = asc_channels;
                /* Init the previous channel count values. This is required to avoid a mismatch of
                memory accesses in the error concealment module and the allocated channel
                structures in this function. */
                self.aac_channels_prev = 0;
                /* Init channel element list with escape value. */
                self.channel_elements = [ID_END; AAC_MAX_ELEMENTS];
            }

            self.truncate_frame_size =
                ((1024 * 48000) / self.sampling_rate_info[0].sampling_rate as i32) as i16;

            /* Update structures */
            if *config_changed != 0 {
                /* Things to be done for each channel, which do not involve allocating memory.
                Doing these things only on the channels needed for the current configuration
                (ascChannels) could lead to memory access violation later (error concealment). */
                let mut ch = aac_channels_offset as usize;
                let mut ch_idx = aac_channels_offset_idx;
                for _ in 0..self.asc_channels[stream_index] {
                    if self.get_on_off_flag(ch_idx) == 0 {
                        ch_idx += 1;
                        continue;
                    }

                    self.p_aac_decoder_channel_info[ch].granule_length =
                        self.stream_info.aac_samples_per_frame / 8;
                    self.p_aac_decoder_channel_info[ch].render_mode = init_render_mode;

                    mdct_init(
                        &mut self.p_aac_decoder_static_channel_info[ch].i_mdct,
                        OVERLAP_BUFFER_SIZE,
                    );

                    /* Reset concealment only if ASC changed. Otherwise it will be done with any
                    config callback. E.g. every time the LATM SMC is present. */
                    cconcealment_init_channel_data(
                        &mut self.p_aac_decoder_static_channel_info[ch].concealment_info,
                        &self.conceal_common_data,
                        init_render_mode,
                        self.stream_info.aac_samples_per_frame,
                    );
                    ch += 1;
                    ch_idx += 1;
                }
            }

            if *config_changed != 0 {
                let (drc_dec_sample_rate, drc_dec_frame_size) =
                    if self.stream_info.ext_sampling_rate != 0 {
                        (
                            self.stream_info.ext_sampling_rate,
                            (self.stream_info.aac_samples_per_frame
                                * self.stream_info.ext_sampling_rate)
                                / self.stream_info.aac_sample_rate,
                        )
                    } else {
                        (self.stream_info.aac_sample_rate, self.stream_info.aac_samples_per_frame)
                    };

                if rs!(self).h_uni_drc_decoder.init(
                    drc_dec_frame_size,
                    drc_dec_sample_rate,
                    self.aac_channels,
                ) != 0
                {
                    break 'bail;
                }
            }
            self.multiband_drc_present = 0;
            if flags & AC_MPEGH3DA != 0 {
                self.multiband_drc_present =
                    rs!(self).h_uni_drc_decoder.get_param(DRC_DEC_IS_MULTIBAND_DRC_1) as u8;
            }

            if self.multiband_drc_present != 0 {
                /* Create filterbanks for Multiband DRC for object and/or HOA signal groups */
                self.stft_frame_size = 256;
                self.num_time_slots =
                    (self.stream_info.aac_samples_per_frame / self.stft_frame_size as i32) as u8;

                let stft_filterbank_config_analysis = StftFilterbankConfig {
                    frame_size: self.stft_frame_size as u32,
                    fft_size: 2 * self.stft_frame_size as u32,
                    stft_filterbank_mode: STFT_FILTERBANK_MODE_TIME_TO_FREQ,
                };
                let stft_filterbank_config_synthesis = StftFilterbankConfig {
                    frame_size: self.stft_frame_size as u32,
                    fft_size: 2 * self.stft_frame_size as u32,
                    stft_filterbank_mode: STFT_FILTERBANK_MODE_FREQ_TO_TIME,
                };
                let mut transport_prev_channel = 0usize;
                let mut grp_prev = 0usize;

                for s in 0..stream_index {
                    transport_prev_channel += self.usc_ref(ascs, s).m_n_usac_channels as usize;
                    grp_prev += self.usc_ref(ascs, s).bs_num_signal_groups as usize;
                }

                let usc = self.usc_ref(ascs, stream_index);
                for grp in 0..usc.bs_num_signal_groups as usize {
                    let transport_start_channel = transport_prev_channel
                        + usc.m_signal_group_type[grp].first_sig_idx as usize;
                    let transport_num_channels = usc.m_signal_group_type[grp].count as usize;

                    for ch in 0..transport_num_channels {
                        let stft_err = stft_filterbank_open(
                            &stft_filterbank_config_analysis,
                            &mut rs!(self).stft_filterbank_analysis[transport_start_channel + ch],
                        );
                        if stft_err != 0 {
                            break 'bail;
                        }

                        let stft_err = stft_filterbank_open(
                            &stft_filterbank_config_synthesis,
                            &mut rs!(self).stft_filterbank_synthesis[transport_start_channel + ch],
                        );
                        if stft_err != 0 {
                            break 'bail;
                        }
                    }
                    rs!(self).stft_headroom_prescaling[grp + grp_prev] = 0;
                }
            }

            if (asc.m_aot == AOT_MPEGH3DA) || (asc.m_aot == AOT_USAC) {
                if let Some(l) = rs!(self).h_limiter.as_mut() {
                    l.set_attack(TDL_MPEGH3DA_DEFAULT_ATTACK);
                    l.set_threshold(TDL_MPEGH3DA_DEFAULT_THRESHOLD);
                }
            }

            if *config_changed != 0 && asc.m_aot == AOT_MPEGH3DA {
                err = self.caac_decoder_init_renderer(
                    ascs,
                    stream_index,
                    self.sampling_rate_info[stream_index].sampling_rate as i32,
                    self.stream_info.aac_samples_per_frame,
                    element_offset,
                    aac_channels_offset_idx,
                    config_mode & AC_CM_LAST_SUBSTREAM != 0,
                );
                if err != AAC_DEC_OK {
                    break 'bail;
                }
            }

            self.caac_decoder_accept_flags(asc, flags, &el_flags, stream_index, element_offset);

            /* Update externally visible copy of flags */
            self.stream_info.flags = self.flags[0];

            return err;
        }

        self.caac_decoder_deinit(ascs, stream_index, 0);
        AAC_DEC_OUT_OF_MEMORY
    }

    /// `CAacDecoder_DecodeFrame(self, flags, pTimeData, timeDataSize, timeDataChannelOffset)`:
    /// decodes one frame. The time signal is written to `pTimeData2[time_data_offset..]`
    /// (`pTimeData`); `time_data_size` is its size in samples.
    ///
    /// The bitstream position of the transport decoder bitstream must be exactly the end of the
    /// access unit, including all byte alignment bits.
    pub fn caac_decoder_decode_frame(
        &mut self,
        tp: &mut TransportDec,
        flags: u32,
        time_data_offset: usize,
        time_data_size: i32,
        time_data_channel_offset: i32,
    ) -> AacDecoderError {
        let mut error_status = AAC_DEC_OK;

        let mut bsi: usize = 0; /* bs = transportDec_GetBitstream(self->hInput, 0) */

        let mut type_: Mp4ElementId = ID_NONE; /* Current element type */
        let mut aac_channels: i32 = 0; /* Channel counter for channels found in the bitstream */
        let mut stream_index: usize = 0; /* index of the current substream */

        let mut au_start_anchor: i32 = tp.bit_stream[bsi].get_valid_bits(); /* AU start bit buffer position for AU byte alignment */

        let check_sample_rate = self.stream_info.aac_sample_rate;

        let mut cconceal_td_fading_applied = [0i32; 28]; /* Initialize status of Time Domain fading */

        /* Init fBsRestartOk. For USAC/MPEG-H defaults to zero because AC_INDEP is required.
        For all other formats every frame is independent. */
        if self.flags[stream_index] & (AC_MPEGH3DA | AC_USAC) != 0 {
            self.stream_info.f_bs_restart_ok = 0;
        } else {
            self.stream_info.f_bs_restart_ok = 1;
        }

        if self.aac_channels <= 0 {
            return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
        }

        /* Any supported base layer valid AU will require more than 16 bits. */
        if (tp.get_au_bits_remaining(0) < 15) && (flags & (AACDEC_CONCEAL | AACDEC_FLUSH)) == 0 {
            self.frame_ok = 0;
            error_status = AAC_DEC_DECODE_FRAME_ERROR;
        }

        if self.flags[0] & AC_MPEGH3DA != 0 {
            self.update_on_off_flags(&tp.asc);
            if self.use_element_skipping != 0 {
                let mut cnt = 0;

                for ch in 0..self.asc_channels[0] {
                    cnt += self.get_on_off_flag(ch);
                }
                if cnt > 28 {
                    return AAC_DEC_UNSUPPORTED_CHANNELCONFIG;
                }
            }
        }

        /* Reset Program Config structure */
        cprogram_config_reset(&mut self.pce);

        caac_decoder_anc_data_reset(&mut self.anc_data);
        if (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0)
            && (self.flags[0] & (AC_USAC | AC_MPEGH3DA) == 0)
            && self.stream_info.channel_config == 0
        {
            /* Init Channel/Element mapping table */
            for ch in 0..28 {
                self.ch_mapping[ch] = 255;
            }
            if cprogram_config_is_valid(&self.pce) == 0 {
                for el in 0..AAC_MAX_ELEMENTS {
                    self.elements[el] = ID_NONE;
                }
            }
        }

        /* Check sampling frequency  */
        if self.stream_info.aac_sample_rate <= 0 {
            /* Instance maybe uninitialized! */
            return AAC_DEC_UNSUPPORTED_SAMPLINGRATE;
        }
        match check_sample_rate {
            96000 | 88200 | 64000 | 16000 | 12000 | 11025 | 8000 | 7350 | 48000 | 44100 | 32000
            | 24000 | 22050 => {}
            _ => {
                if self.flags[0] & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) == 0 {
                    return AAC_DEC_UNSUPPORTED_SAMPLINGRATE;
                }
            }
        }

        if flags & AACDEC_CLRHIST != 0 {
            /* Clear history */
            for ch in 0..self.aac_channels.max(0) as usize {
                /* Reset concealment */
                let render_mode = self.p_aac_decoder_channel_info[0].render_mode;
                cconcealment_init_channel_data(
                    &mut self.p_aac_decoder_static_channel_info[ch].concealment_info,
                    &self.conceal_common_data,
                    render_mode,
                    self.stream_info.aac_samples_per_frame,
                );
                /* Clear overlap-add buffers to avoid clicks. */
                let ov = &mut self.p_aac_decoder_static_channel_info[ch].i_mdct.overlap;
                let n = (OVERLAP_BUFFER_SIZE as usize).min(ov.len());
                ov[..n].fill(0);
            }
            if self.stream_info.channel_config > 0 {
                /* Declare the possibly adopted old PCE (with outdated metadata) invalid. */
                cprogram_config_init(&mut self.pce);
            }
        }

        let mut previous_element_index: u8 = 0; /* Canonical index of last element */
        let mut element_count: usize = 0; /* Element counter for elements found in the bitstream */
        let mut channel_element_count: usize = 0; /* Channel element counter */
        let mut mct_el_flags = [0u32; AAC_MAX_ELEMENTS];
        let mut p2_mct_el_flags: usize = 0;
        let mut el_cnt = [0i32; ID_LAST as usize]; /* element counter ( robustness ) */
        let mut element_count_prev_streams: usize = 0; /* Element count of all previous sub streams. */
        let mut aac_channels_idx: i32 = 0;

        while (type_ != ID_END)
            && (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0)
            && self.frame_ok != 0
        {
            let el_channels;
            let mut mpegh_element_length: i32 = -1;
            let mut el_start_bit_pos: i32 = 0;

            if element_count >= AAC_MAX_ELEMENTS {
                self.frame_ok = 0;
                error_status = AAC_DEC_PARSE_ERROR;
                break;
            }
            type_ = self.elements[element_count];

            /* Determine current sub stream */
            if self.flags[stream_index] & (AC_MPEGH3DA | AC_USAC | AC_RSVD50) != 0 {
                let mut num_elements = 0usize;

                for i in 0..=stream_index {
                    num_elements += self.num_usac_elements[i] as usize;
                }
                if element_count >= num_elements
                    && (stream_index < TPDEC_MAX_TRACKS - 1)
                    && self.asc_channels[stream_index + 1] > 0
                {
                    element_count_prev_streams = element_count;
                    stream_index += 1;
                    num_elements += self.num_usac_elements[stream_index] as usize;
                }
                /* Detect first element of sub stream to switch bit stream and parse indep flag. */
                if element_count as i64
                    == num_elements as i64 - self.num_usac_elements[stream_index] as i64
                {
                    /* Do byte alignment of current bit stream before switching to next. */
                    /* Only prerollAUs are byteAligned with respect to the first bit */
                    /* Byte alignment with respect to the first bit of the raw_data_block(). */
                    if (self.flags[stream_index] & (AC_RSVD50 | AC_USAC) == 0)
                        || (self.preroll_au_length[self.access_unit as usize] != 0)
                    /* indicates preroll */
                    {
                        tp.bit_stream[bsi].byte_align(au_start_anchor as u32);
                    }
                    /* Switch to next sub stream */
                    bsi = stream_index;
                    au_start_anchor = tp.bit_stream[bsi].get_valid_bits();

                    if (tp.get_au_bits_total(stream_index as u32) > 0)
                        || (tp.get_format() == TT_MP4_ADIF)
                    {
                        if (self.flags[stream_index] & (AC_USAC | AC_RSVD50) != 0
                            && element_count == 0)
                            || (self.flags[stream_index] & AC_MPEGH3DA != 0)
                        {
                            self.flags[stream_index] &= !AC_INDEP;

                            if tp.bit_stream[bsi].read_bit() != 0 {
                                self.flags[stream_index] |= AC_INDEP;
                                self.stream_info.f_bs_restart_ok = 1;
                            } else if self.aac_channels_prev == 0 {
                                self.frame_ok = 0; /* decoding cannot start on a frame which is no independent frame */
                            }

                            let mut ch = aac_channels;
                            for ch_idx in aac_channels_idx..self.asc_channels[stream_index] {
                                if self.get_on_off_flag(ch_idx) != 0 {
                                    /* Robustness check */
                                    if ch >= self.aac_channels {
                                        return AAC_DEC_UNKNOWN;
                                    }

                                    /* if last frame was broken and this frame is no independent
                                     * frame, correct decoding is impossible we need to trigger
                                     * concealment */
                                    if (cconcealment_get_last_frame_ok(
                                        Some(
                                            &self.p_aac_decoder_static_channel_info[ch as usize]
                                                .concealment_info,
                                        ),
                                        1,
                                    ) == 0)
                                        && (self.flags[stream_index] & AC_INDEP == 0)
                                    {
                                        self.frame_ok = 0;
                                    }
                                    ch += 1;
                                }
                            }
                        }
                    }
                }
                if (tp.get_au_bits_total(stream_index as u32) <= 0)
                    && (tp.get_format() != TT_MP4_ADIF)
                {
                    let el_channels = caac_decoder_get_el_channels(type_);

                    if el_channels > 0 {
                        if self.aac_channels_prev == 0 {
                            self.channel_elements[channel_element_count] = type_;
                        }
                        channel_element_count += 1;
                    }
                    aac_channels += el_channels;
                    aac_channels_idx += el_channels;
                    element_count += 1;
                    continue;
                }
            } else {
                stream_index = 0;
            }

            if tp.bit_stream[bsi].get_valid_bits() < 0 {
                self.frame_ok = 0;
            }

            'sw: {
                match type_ {
                    ID_SCE | ID_CPE | ID_LFE | ID_USAC_SCE | ID_USAC_CPE | ID_USAC_LFE => {
                        if element_count >= AAC_MAX_ELEMENTS {
                            self.frame_ok = 0;
                            error_status = AAC_DEC_PARSE_ERROR;
                            break 'sw;
                        }

                        if self.flags[stream_index] & AC_MPEGH3DA != 0
                            && self.usc_ref(&tp.asc, stream_index).element_length_present != 0
                        {
                            /* The value of the element length can be used to skip elements
                             * containing signals that are not to be rendered. */
                            mpegh_element_length = tp.bit_stream[bsi].read_bits(16) as i32;
                            el_start_bit_pos = tp.bit_stream[bsi].get_valid_bits();
                        }

                        el_channels = caac_decoder_get_el_channels(type_);

                        if self.flags[stream_index] & AC_MPEGH3DA != 0 {
                            /* Create an elFlag array for use in MCT */
                            mct_el_flags[p2_mct_el_flags] = self.el_flags[element_count];
                            p2_mct_el_flags += 1;
                            if el_channels == 2 {
                                mct_el_flags[p2_mct_el_flags] = self.el_flags[element_count];
                                p2_mct_el_flags += 1;
                            }

                            if self.get_on_off_flag(aac_channels_idx) == 0 {
                                if mpegh_element_length == -1 {
                                    self.frame_ok = 0;
                                    error_status = AAC_DEC_PARSE_ERROR;
                                    break 'sw;
                                }
                                /* Skip signals that are off. */
                                aac_channels_idx += el_channels;
                                tp.bit_stream[bsi].push_for(mpegh_element_length as u32);
                                break 'sw;
                            }
                        }
                        /* Consistency check */
                        {
                            let mut total_asc_channels = 0;

                            for i in 0..TPDEC_MAX_TRACKS {
                                total_asc_channels += self.asc_channels[i];
                            }
                            if (el_cnt[type_ as usize] >= (total_asc_channels >> (el_channels - 1)))
                                || (aac_channels > (total_asc_channels - el_channels))
                            {
                                error_status = AAC_DEC_DECODE_FRAME_ERROR;
                                self.frame_ok = 0;
                                break 'sw;
                            }
                        }

                        if (self.aac_channels_prev > 0)
                            && (type_ != self.channel_elements[channel_element_count])
                        {
                            error_status = AAC_DEC_PARSE_ERROR;
                            self.frame_ok = 0;
                            break 'sw;
                        }

                        if self.frame_ok != 0 {
                            let ac = aac_channels as usize;
                            error_status = match self.com_static_link[ac] {
                                Some(owner) if self.com_static_data[owner].is_some() => {
                                    let cpe_link = self.cpe_static_link[ac];
                                    let mut mem = CChannelElementMem {
                                        spec: &mut self.work_buffer_core2[ac * 1024..],
                                        com_static_data: self.com_static_data[owner]
                                            .as_deref_mut()
                                            .unwrap(),
                                        work_buffer_core1: &mut self.work_buffer_core1,
                                        cpe_static_data: match cpe_link {
                                            Some(e) => self.cpe_static_data[e].as_deref_mut(),
                                            None => None,
                                        },
                                        igf_private_data_common: &mut self.igf_private_data_common,
                                        time_data2: &mut self.p_time_data2,
                                    };
                                    cchannel_element_read(
                                        &mut tp.bit_stream[bsi],
                                        &mut self.p_aac_decoder_channel_info[ac..],
                                        &mut self.p_aac_decoder_static_channel_info[ac..],
                                        &mut mem,
                                        self.stream_info.aot,
                                        &self.sampling_rate_info[stream_index],
                                        self.flags[stream_index],
                                        self.el_flags[element_count],
                                        self.stream_info.aac_samples_per_frame as u32,
                                        el_channels as u8,
                                        self.stream_info.ep_config,
                                        None,
                                    )
                                }
                                /* NULL pointer in C++ */
                                _ => AAC_DEC_UNKNOWN,
                            };
                            if error_status != AAC_DEC_OK {
                                self.frame_ok = 0;
                            }
                        }

                        if self.frame_ok != 0 {
                            /* Lookup the element and decode it only if it belongs to the current
                             * program */
                            if cprogram_config_lookup_element(
                                &mut self.pce,
                                self.stream_info.channel_config as u32,
                                self.p_aac_decoder_channel_info[aac_channels as usize]
                                    .element_instance_tag as u32,
                                aac_channels as u32,
                                &mut self.ch_mapping,
                                &mut self.channel_type,
                                &mut self.channel_indices,
                                28,
                                &mut previous_element_index,
                                &mut self.elements,
                                type_,
                            ) != 0
                            {
                                if self.aac_channels_prev == 0 {
                                    self.channel_elements[channel_element_count] = type_;
                                }
                                channel_element_count += 1;
                                aac_channels += el_channels;
                                aac_channels_idx += el_channels;
                            } else {
                                self.frame_ok = 0;
                            }
                        }

                        el_cnt[type_ as usize] += 1;
                    }

                    ID_USAC_EXT => {
                        if (element_count - element_count_prev_streams) >= AAC_MAX_ELEMENTS {
                            self.frame_ok = 0;
                            error_status = AAC_DEC_PARSE_ERROR;
                            break 'sw;
                        }
                        /* parse extension element payload
                          q.v. mpegh3daExtElement() ISO/IEC DIS 23008-3  Table 30
                          or   UsacExElement() ISO/IEC FDIS 23003-3:2011(E)  Table 21
                        */
                        let mut usac_ext_element_payload_length: i32;
                        let ext_el = self.usc_ref(&tp.asc, stream_index).element
                            [element_count - element_count_prev_streams]
                            .ext_element;

                        if tp.bit_stream[bsi].read_bit() != 0 {
                            /* usacExtElementPresent */
                            if tp.bit_stream[bsi].read_bit() != 0 {
                                /* usacExtElementUseDefaultLength */
                                usac_ext_element_payload_length =
                                    ext_el.usac_ext_element_default_length as i32;
                            } else {
                                usac_ext_element_payload_length =
                                    tp.bit_stream[bsi].read_bits(8) as i32;
                                if usac_ext_element_payload_length == (1 << 8) - 1 {
                                    let value_add = tp.bit_stream[bsi].read_bits(16);
                                    usac_ext_element_payload_length += value_add as i32 - 2;
                                }
                            }
                            if usac_ext_element_payload_length > 0 {
                                if ext_el.usac_ext_element_payload_frag != 0 {
                                    /* usacExtElementStart = */
                                    tp.bit_stream[bsi].read_bit();
                                    /* usacExtElementStop = */
                                    tp.bit_stream[bsi].read_bit();
                                }

                                let mut usac_ext_bit_pos = tp.bit_stream[bsi].get_valid_bits();

                                let usac_ext_element_type = ext_el.usac_ext_element_type;

                                'ext: {
                                    match usac_ext_element_type {
                                        ID_EXT_ELE_OBJ_METADATA => {
                                            if self.target_layout_config > -1 {
                                                let mut gvbap_idx = 0usize;
                                                let mut object_on = 0;
                                                let mut sig_count = 0i32;

                                                'outer: for s in 0..=stream_index {
                                                    let usc_s = self.usc_ref(&tp.asc, s);
                                                    for grp_idx in
                                                        0..usc_s.bs_num_signal_groups as usize
                                                    {
                                                        if usc_s.m_signal_group_type[grp_idx].type_
                                                            == 1
                                                        {
                                                            let sig_idx = usc_s.m_signal_group_type
                                                                [grp_idx]
                                                                .first_sig_idx
                                                                as i32;
                                                            object_on =
                                                                self.get_on_off_flag(sig_idx);

                                                            if aac_channels_idx == sig_count {
                                                                break 'outer;
                                                            }

                                                            if object_on != 0 {
                                                                gvbap_idx += 1;
                                                            }
                                                        }

                                                        sig_count += usc_s.m_signal_group_type
                                                            [grp_idx]
                                                            .count
                                                            as i32;
                                                    }
                                                }

                                                if sig_count > aac_channels_idx {
                                                    /* Check amount of objectGroups counted */
                                                    error_status = AAC_DEC_PARSE_ERROR;
                                                    self.frame_ok = 0;
                                                    break 'ext;
                                                }

                                                if object_on != 0 {
                                                    /* objectMetadataFrame() of
                                                     * hgVBAPRenderer[gvbapIdx] (if it exists; see
                                                     * AacDecoderInstance::object_metadata_frame()
                                                     * for the frame pipelining) */
                                                    let oam = &ext_el.ext_config.oam;
                                                    if let Some(object_metadata_parse_error) = self
                                                        .object_metadata_frame(
                                                            gvbap_idx,
                                                            &mut tp.bit_stream[bsi],
                                                            usac_ext_element_payload_length,
                                                            oam.low_delay_metadata_coding,
                                                            oam.has_dynamic_object_priority,
                                                            oam.has_uniform_spread,
                                                        )
                                                    {
                                                        if object_metadata_parse_error != 0 {
                                                            error_status = AAC_DEC_PARSE_ERROR;
                                                            self.frame_ok = 0;
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        ID_EXT_ELE_PROD_METADATA => {
                                            if self.target_layout_config > -1 {
                                                let mut obj_grp = 0i32;
                                                let mut gvbap_idx = 0usize;
                                                let mut sig_count = 0i32;

                                                for s in 0..stream_index {
                                                    let usc_s = self.usc_ref(&tp.asc, s);
                                                    for grp_idx in
                                                        0..usc_s.bs_num_signal_groups as usize
                                                    {
                                                        sig_count += usc_s.m_signal_group_type
                                                            [grp_idx]
                                                            .count
                                                            as i32;

                                                        if usc_s.m_signal_group_type[grp_idx].type_
                                                            == 1
                                                        {
                                                            let sig_idx = usc_s.m_signal_group_type
                                                                [grp_idx]
                                                                .first_sig_idx
                                                                as i32;
                                                            if self.get_on_off_flag(sig_idx) != 0 {
                                                                gvbap_idx += 1;
                                                            }
                                                        }
                                                    }
                                                }
                                                if sig_count > aac_channels_idx {
                                                    /* Check amount of objectGroups counted */
                                                    error_status = AAC_DEC_PARSE_ERROR;
                                                    self.frame_ok = 0;
                                                    break 'ext;
                                                }

                                                let (tp_asc, tp_bs) =
                                                    (&tp.asc, &mut tp.bit_stream[bsi]);
                                                let usc_s = self.usc_ref(tp_asc, stream_index);
                                                for sig_grp in
                                                    0..usc_s.bs_num_signal_groups as usize
                                                {
                                                    if usc_s.m_signal_group_type[sig_grp].type_ == 1
                                                    {
                                                        let pm = &ext_el.ext_config.prod_metadata;
                                                        let bs_reference_distance =
                                                            pm.bs_reference_distance as i32;
                                                        let has_object_distance = (pm
                                                            .has_object_distance
                                                            & 1u32.wrapping_shl(
                                                                (31 - obj_grp) as u32,
                                                            ))
                                                            as i32;
                                                        let err;

                                                        if self.get_on_off_flag(
                                                            usc_s.m_signal_group_type[sig_grp]
                                                                .first_sig_idx
                                                                as i32
                                                                + aac_channels_idx,
                                                        ) != 0
                                                        {
                                                            if let Some(r) = rs!(self)
                                                                .hg_vbap_renderer[gvbap_idx]
                                                                .as_deref_mut()
                                                            {
                                                                let n = r.num_objects as i32 as u32;
                                                                err = prod_metadata_frame_group(
                                                                    Some(r),
                                                                    tp_bs,
                                                                    n,
                                                                    bs_reference_distance,
                                                                    has_object_distance,
                                                                );
                                                                gvbap_idx += 1;
                                                            } else {
                                                                err = 1;
                                                            }
                                                        } else {
                                                            err = prod_metadata_frame_group(
                                                                None,
                                                                tp_bs,
                                                                usc_s.m_signal_group_type[sig_grp]
                                                                    .count
                                                                    as u32,
                                                                bs_reference_distance,
                                                                has_object_distance,
                                                            );
                                                        }

                                                        if err != 0 {
                                                            error_status = AAC_DEC_PARSE_ERROR;
                                                            self.frame_ok = 0;
                                                            break;
                                                        }

                                                        obj_grp += 1;
                                                    }
                                                }
                                            }
                                        }
                                        ID_EXT_ELE_MCT => {
                                            let (tp_asc, tp_bs) =
                                                (&tp.asc, &mut tp.bit_stream[bsi]);
                                            let usc_s = self.usc_ref(tp_asc, stream_index);
                                            let grp = usc_s.sig_idx2_grp_idx
                                                [aac_channels_idx as usize]
                                                as usize;
                                            let mut prev_signal_groups = 0usize;

                                            for s in 0..stream_index {
                                                prev_signal_groups +=
                                                    self.usc_ref(tp_asc, s).bs_num_signal_groups
                                                        as usize;
                                            }

                                            if let Some(mct) = self.p_mct_dec
                                                [grp + prev_signal_groups]
                                                .as_deref_mut()
                                            {
                                                /* Store initial value for counting bits */
                                                let bits = tp_bs.get_valid_bits();

                                                let mut mct_err = cmct_inverse_mct_parse_bs(
                                                    mct,
                                                    tp_bs,
                                                    (self.flags[stream_index] & AC_INDEP != 0)
                                                        as i32,
                                                    usc_s.m_signal_group_type[grp].count as i32,
                                                );

                                                mct.mct_group_parsed = 1;

                                                /*Find bits consumed */
                                                let bits_left = bits - tp_bs.get_valid_bits();
                                                /* Check if number of bits read corresponds to the
                                                 * number of bytes transmitted. Max 7 bits can be
                                                 * wasted,i.e. the last byte can be partially used */
                                                let diff = (usac_ext_element_payload_length << 3)
                                                    - bits_left;
                                                if (diff > 7) || (diff < 0) {
                                                    mct_err = -1;
                                                }
                                                if mct_err != 0 {
                                                    self.frame_ok = 0;
                                                    error_status = AAC_DEC_PARSE_ERROR;
                                                }
                                            }
                                        }
                                        ID_EXT_ELE_FMT_CNVRTR => {
                                            /* FormatConverterFrame() */
                                            if stream_index == 0 {
                                                if let Some(fc) =
                                                    rs!(self).p_format_converter.as_deref_mut()
                                                {
                                                    fc_frame(fc, &mut tp.bit_stream[bsi]);
                                                }
                                            }
                                        }
                                        ID_EXT_ELE_UNI_DRC => {
                                            /* uniDrcGain() */
                                            /* FDK_drcDec_ReadUniDrcGain() (see
                                             * AacDecoderInstance::read_uni_drc_gain() for the
                                             * frame pipelining) */
                                            let drc_err = self.read_uni_drc_gain(
                                                &mut tp.bit_stream[bsi],
                                                stream_index as i32,
                                                usac_ext_element_payload_length,
                                            );
                                            if drc_err != DRC_DEC_OK {
                                                error_status = AAC_DEC_PARSE_ERROR;
                                            }
                                        }

                                        _ => {}
                                    }
                                }

                                /* Skip any remaining bits of extension payload */
                                usac_ext_bit_pos = (usac_ext_element_payload_length * 8)
                                    - (usac_ext_bit_pos - tp.bit_stream[bsi].get_valid_bits());
                                if usac_ext_bit_pos < 0 {
                                    self.frame_ok = 0;
                                    error_status = AAC_DEC_PARSE_ERROR;
                                }
                                tp.bit_stream[bsi].push_bi_directional(usac_ext_bit_pos);
                            }
                        }
                    }
                    ID_END | ID_USAC_END => {}

                    _ => {
                        error_status = AAC_DEC_DECODE_FRAME_ERROR;
                        self.frame_ok = 0;
                    }
                }
            }

            if self.flags[stream_index] & AC_MPEGH3DA != 0 && mpegh_element_length != -1 {
                let bits_offset =
                    mpegh_element_length - (el_start_bit_pos - tp.bit_stream[bsi].get_valid_bits());
                if bits_offset != 0 {
                    error_status = AAC_DEC_PARSE_ERROR;
                    self.frame_ok = 0;
                }
            }

            /* Support arbitrary amount of FIL elements for General Audio bitstreams by not
             * increasing element_count */
            if self.flags[0] & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA | AC_ELD | AC_SCALABLE | AC_ER)
                != 0
            {
                element_count += 1;
            }
        } /* while ( (type != ID_END) ... ) */

        if (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0)
            && (self.frame_ok == 1)
            && (self.flags[stream_index] & AC_MPEGH3DA == 0)
            && (self.aac_channels != aac_channels)
        {
            self.frame_ok = 0;

            if error_status == AAC_DEC_OK {
                error_status = AAC_DEC_PARSE_ERROR;
            }
        }

        if flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0 {
            /* float decoder checks if bitsLeft is in range 0-7; only prerollAUs are byteAligned
             * with respect to the first bit */
            /* Byte alignment with respect to the first bit of the raw_data_block(). */
            if (self.flags[stream_index] & (AC_RSVD50 | AC_USAC) == 0)
                || (self.preroll_au_length[self.access_unit as usize] != 0)
            /* indicates preroll */
            {
                tp.bit_stream[bsi].byte_align(au_start_anchor as u32);
            }

            /* Check if all bits of the raw_data_block() have been read. */
            if tp.get_au_bits_total(0) > 0 {
                let mut unread_bits = tp.get_au_bits_remaining(0);
                /* for pre-roll frames pre-roll length has to be used instead of total AU lenght */
                /* unreadBits regarding preroll bounds */
                if self.preroll_au_length[self.access_unit as usize] != 0 {
                    unread_bits = unread_bits - tp.get_au_bits_total(0)
                        + self.preroll_au_length[self.access_unit as usize] as i32;
                }

                if (unread_bits != 0
                    && (self.flags[stream_index] & (AC_RSVD50 | AC_USAC) == 0)
                    && stream_index == 0)
                    || (self.preroll_au_length[self.access_unit as usize] != 0
                        && 0 < unread_bits
                        && unread_bits <= 7)
                {
                    self.frame_ok = 0;
                    /* Do not overwrite current error */
                    if error_status == AAC_DEC_OK && self.frame_ok == 0 {
                        error_status = AAC_DEC_PARSE_ERROR;
                    }
                    /* Always put the bitbuffer at the right position after the current Access
                     * Unit. */
                    tp.bit_stream[bsi].push_bi_directional(unread_bits);
                }
            }

            /* Check the last element. The terminator (ID_END) has to be the last one (even if ER
             * syntax is used). */
            if self.frame_ok != 0 && type_ != ID_END {
                /* Do not overwrite current error */
                if error_status == AAC_DEC_OK {
                    error_status = AAC_DEC_PARSE_ERROR;
                }
                self.frame_ok = 0;
            }

            /* MCT */
            {
                let mut channel_offset = 0usize;
                let mut prev_signal_groups = 0usize;

                for si in 0..TPDEC_MAX_TRACKS {
                    if self.flags[si] & AC_MPEGH3DA != 0 {
                        let n_groups = self.usc_ref(&tp.asc, si).bs_num_signal_groups as usize;
                        for grp in prev_signal_groups..prev_signal_groups + n_groups {
                            if let Some(mct) = self.p_mct_dec[grp].as_deref_mut() {
                                if (self.frame_ok != 0) && (mct.mct_group_parsed != 0) {
                                    let err = cmct_mct_stereo_filling(
                                        mct,
                                        self.stream_info.aac_samples_per_frame,
                                        &mut self.p_aac_decoder_channel_info[channel_offset..],
                                        &mut self.p_aac_decoder_static_channel_info
                                            [channel_offset..],
                                        &mut self.work_buffer_core2[channel_offset * 1024..],
                                        &self.sampling_rate_info[si],
                                        &mct_el_flags[channel_offset..],
                                        (self.flags[si] & AC_INDEP != 0) as u32,
                                    );
                                    if err != 0 {
                                        self.frame_ok = 0;
                                        break;
                                    }

                                    /* Save MCT output spectra for usage in the next frame stereo
                                     * filling calculations */
                                    cmct_stereo_filling_save_prev(
                                        mct,
                                        &self.p_aac_decoder_channel_info[channel_offset..],
                                        &self.work_buffer_core2[channel_offset * 1024..],
                                    );
                                } else {
                                    /* Clear MCT output spectra if frame not ok */
                                    cmct_stereo_filling_clear_prev(mct);
                                }

                                mct.mct_group_parsed = 0;
                            } /* if (self->pMCTdec[grp]) */
                        }

                        prev_signal_groups += n_groups;
                    } /* if (self->flags & AC_MPEGH3DA)  */

                    channel_offset += self.alloc_channels[si].max(0) as usize;
                }

                stream_index = 0;
            }
            if self.frame_ok != 0 {
                if self.aac_channels_prev == 0 {
                    self.channel_elements[channel_element_count] = ID_END;
                } else if self.channel_elements[channel_element_count] != ID_END {
                    error_status = AAC_DEC_PARSE_ERROR;
                    self.frame_ok = 0;
                }
            }
        } /* !(AACDEC_CONCEAL|AACDEC_FLUSH) */

        element_count = 0;
        aac_channels = 0;
        aac_channels_idx = 0;
        type_ = ID_NONE;

        while type_ != ID_END && element_count < AAC_MAX_ELEMENTS {
            /* Determine current sub stream */
            if self.flags[stream_index] & (AC_MPEGH3DA | AC_USAC) != 0 {
                let mut num_elements = 0usize;

                for i in 0..=stream_index {
                    num_elements += self.num_usac_elements[i] as usize;
                }
                if element_count >= num_elements
                    && (stream_index < TPDEC_MAX_TRACKS - 1)
                    && self.p_usac_config[stream_index + 1].is_some()
                {
                    stream_index += 1;
                }
            } else {
                stream_index = 0;
            }

            if self.flags[stream_index] & (AC_USAC | AC_MPEGH3DA | AC_BSAC) != 0 {
                type_ = self.elements[element_count];
            } else {
                type_ = self.channel_elements[element_count];
            }

            if (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0) && self.frame_ok != 0 {
                /* Skip sub streams not being present. */
                if (tp.get_au_bits_total(stream_index as u32) <= 0)
                    && (tp.get_format() != TT_MP4_ADIF)
                {
                    let el_channels = caac_decoder_get_el_channels(type_);
                    aac_channels += el_channels;
                    aac_channels_idx += el_channels;
                    element_count += 1;
                    continue;
                }

                match type_ {
                    ID_SCE | ID_CPE | ID_LFE | ID_USAC_SCE | ID_USAC_CPE | ID_USAC_LFE => {
                        let el_channels = caac_decoder_get_el_channels(type_);

                        /* Skip signals that are not enabled */
                        if self.get_on_off_flag(aac_channels_idx) == 0 {
                            aac_channels_idx += el_channels;
                        } else {
                            let ac = aac_channels as usize;
                            if let Some(owner) = self.com_static_link[ac] {
                                if let Some(csd) = self.com_static_data[owner].as_deref_mut() {
                                    let cpe_link = self.cpe_static_link[ac];
                                    let mut mem = CChannelElementMem {
                                        spec: &mut self.work_buffer_core2[ac * 1024..],
                                        com_static_data: csd,
                                        work_buffer_core1: &mut self.work_buffer_core1,
                                        cpe_static_data: match cpe_link {
                                            Some(e) => self.cpe_static_data[e].as_deref_mut(),
                                            None => None,
                                        },
                                        igf_private_data_common: &mut self.igf_private_data_common,
                                        time_data2: &mut self.p_time_data2,
                                    };
                                    cchannel_element_decode_pool(
                                        &mut self.p_aac_decoder_channel_info[ac..],
                                        &mut self.p_aac_decoder_static_channel_info[ac..],
                                        &mut mem,
                                        &self.sampling_rate_info[stream_index],
                                        self.flags[stream_index],
                                        self.el_flags[element_count],
                                        el_channels,
                                        self.thread_pool.as_deref(),
                                    );
                                }
                            }
                            aac_channels += el_channels;
                            aac_channels_idx += el_channels;
                        }
                    }
                    ID_NONE => {
                        type_ = ID_END;
                    }
                    _ => {}
                }
            }
            element_count += 1;
        }

        /* More AAC channels than specified by the ASC not allowed. */
        if (aac_channels == 0 || aac_channels > self.aac_channels)
            && (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0)
        {
            /* Do not overwrite current error */
            if error_status == AAC_DEC_OK {
                error_status = AAC_DEC_DECODE_FRAME_ERROR;
            }
            self.frame_ok = 0;
            aac_channels = 0;
        }

        if (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0) && TRANSPORTDEC_OK != tp.crc_check() {
            error_status = AAC_DEC_CRC_ERROR;
            self.frame_ok = 0;
        }

        /* Ensure that in case of concealment a proper error status is set. */
        if (self.frame_ok == 0) && (error_status == AAC_DEC_OK) {
            error_status = AAC_DEC_DECODE_FRAME_ERROR;
        }

        if self.frame_ok != 0 && (flags & AACDEC_FLUSH != 0) {
            aac_channels = self.aac_channels_prev;
            /* Because the downmix could be active, its necessary to restore the channel type and
             * indices. */
            self.channel_type = self.channel_type_prev; /* restore */
            self.channel_indices = self.channel_indices_prev; /* restore */
        } else {
            /* store or restore the number of channels and the corresponding info */
            if self.frame_ok != 0 && (flags & AACDEC_CONCEAL == 0) {
                self.aac_channels_prev = aac_channels; /* store */
                self.channel_type_prev = self.channel_type; /* store */
                self.channel_indices_prev = self.channel_indices; /* store */
            } else if self.aac_channels > 0 {
                if (self.build_up_status == AACDEC_MPEGH_BUILD_UP_ON)
                    || (self.build_up_status == AACDEC_MPEGH_BUILD_UP_ON_IN_BAND)
                    || (self.build_up_status == AACDEC_USAC_BUILD_UP_ON)
                {
                    let mut i = 0usize;
                    let mut n = 0usize;
                    loop {
                        if is_channel_element(self.elements[i]) || (self.elements[i] == ID_END) {
                            self.channel_elements[n] = self.elements[i];
                            n += 1;
                        }
                        let e = self.elements[i];
                        i += 1;
                        if e == ID_END {
                            break;
                        }
                    }
                    aac_channels = self.aac_channels;
                    self.aac_channels_prev = aac_channels; /* store */
                } else {
                    aac_channels = self.aac_channels_prev; /* restore */
                }
                self.channel_type = self.channel_type_prev; /* restore */
                self.channel_indices = self.channel_indices_prev; /* restore */
            }
        }

        /* Update number of output channels */
        self.stream_info.aac_num_channels = aac_channels;

        /* Ensure consistency of IS_OUTPUT_VALID() macro. */
        if aac_channels == 0 {
            error_status = AAC_DEC_UNKNOWN;
        }

        /* If there is no valid data to transform into time domain, return. */
        if !is_output_valid(error_status) {
            return error_status;
        }

        /*
          Inverse transform
        */
        {
            /* Create a reverse mapping table */
            let mut reverse_ch_mapping = [0u8; AAC_MAX_SIGNALS];
            for c in 0..aac_channels.max(0) {
                let mut d = 0;
                while d < aac_channels - 1 {
                    if self.ch_mapping[d as usize] as i32 == c {
                        break;
                    }
                    d += 1;
                }
                reverse_ch_mapping[c as usize] = d as u8;
            }

            let spf = self.stream_info.aac_samples_per_frame;
            let mut parallel_done = false;
            if let Some(pool) = self.thread_pool.clone() {
                match self.frequency_to_time_parallel(
                    &pool,
                    tp,
                    flags,
                    element_count,
                    time_data_offset,
                    time_data_size,
                    time_data_channel_offset,
                    aac_channels,
                    &reverse_ch_mapping,
                    &mut error_status,
                ) {
                    FreqToTimeOutcome::Return(err) => return err,
                    FreqToTimeOutcome::Done(si) => {
                        stream_index = si;
                        parallel_done = true;
                    }
                    FreqToTimeOutcome::Fallback => {}
                }
            }
            stream_index = if parallel_done { stream_index } else { 0 };
            let mut c: i32 = 0;
            let mut c_idx: i32 = 0;
            let mut el_channels: i32 = 0;
            for el in 0..if parallel_done { 0 } else { element_count } {
                /* Determine current sub stream */
                if self.flags[stream_index] & (AC_MPEGH3DA | AC_USAC) != 0 {
                    let mut num_elements = 0usize;

                    for i in 0..=stream_index {
                        num_elements += self.num_usac_elements[i] as usize;
                    }
                    if el >= num_elements
                        && (stream_index < TPDEC_MAX_TRACKS - 1)
                        && self.p_usac_config[stream_index + 1].is_some()
                    {
                        stream_index += 1;
                    }
                } else {
                    stream_index = 0;
                }

                if self.flags[stream_index] & (AC_USAC | AC_MPEGH3DA | AC_BSAC) != 0 {
                    type_ = self.elements[el];
                } else {
                    type_ = self.channel_elements[el];
                }

                {
                    let n_element_channels = caac_decoder_get_el_channels(type_);

                    el_channels += n_element_channels;

                    if n_element_channels == 0 {
                        continue;
                    }
                }

                let mut el_ch = 0;
                /* "c" iterates in canonical MPEG channel order */
                'chl: while c_idx < el_channels {
                    'ch: {
                        if self.get_on_off_flag(c_idx) == 0 {
                            c -= 1;
                            break 'ch;
                        }
                        /* Robustness check */
                        if c >= aac_channels {
                            return AAC_DEC_UNKNOWN;
                        }
                        let cu = c as usize;

                        /* Setup offset for time buffer traversal. */
                        let offset = time_data_offset
                            + (reverse_ch_mapping[cu] as i32 * time_data_channel_offset) as usize;

                        if self.flags[stream_index] & AC_MPEGH3DA != 0 {
                            /* Clear audio data for sub streams which are currently not
                             * available. */
                            if tp.get_au_bits_total(stream_index as u32) <= 0
                                && (flags & (AACDEC_FLUSH | AACDEC_CONCEAL) == 0)
                            {
                                self.p_time_data2[offset..offset + spf as usize].fill(0);
                                break 'ch;
                            }
                        }

                        if flags & AACDEC_FLUSH != 0 {
                            /* Clear pAacDecoderChannelInfo->pSpectralCoefficient because with
                             * AACDEC_FLUSH set it contains undefined data. */
                            self.work_buffer_core2[cu * 1024..cu * 1024 + spf as usize].fill(0);
                        }

                        /* if The ics info is not valid and it will be stored and used in the
                         * following concealment method, mark the frame as erroneous */
                        {
                            let p_ics_info = &self.p_aac_decoder_channel_info[cu].ics_info;
                            let h_concealment_info =
                                &self.p_aac_decoder_static_channel_info[cu].concealment_info;
                            let mute_release_active = (self.frame_ok != 0
                                && (flags & AACDEC_CONCEAL == 0))
                                && ((h_concealment_info.conceal_state >= ConcealState_Mute && {
                                    /* concealment state, not part of the state digest */
                                    mpeghdec_fdk::digest::hazard();
                                    true
                                }) && (h_concealment_info.cnt_valid_frames
                                    < self.conceal_common_data.num_mute_release_frames));
                            let ics_is_invalid = get_scale_factor_bands_transmitted(p_ics_info)
                                > get_scale_factor_bands_total(p_ics_info);
                            let ics_info_usedin_fade_out = true;
                            if ics_info_usedin_fade_out && ics_is_invalid && !mute_release_active {
                                self.frame_ok = 0;
                            }
                        }

                        /*
                          Conceal defective spectral data
                        */
                        let conceal_apply_return_code = {
                            let st = &mut self.p_aac_decoder_static_channel_info[cu];
                            cconcealment_apply(
                                &mut st.concealment_info,
                                &self.conceal_common_data,
                                &mut self.p_aac_decoder_channel_info[cu],
                                &mut self.work_buffer_core2[cu * 1024..(cu + 1) * 1024],
                                &self.sampling_rate_info[stream_index],
                                spf,
                                0,
                                (self.frame_ok != 0 && (flags & AACDEC_CONCEAL == 0)) as i32,
                                self.flags[stream_index],
                            )
                        };
                        let frame_ok_but_conceal = conceal_apply_return_code == -1;

                        if time_data_size < time_data_channel_offset * self.aac_channels {
                            error_status = AAC_DEC_OUTPUT_BUFFER_TOO_SMALL;
                            /* leaves the channel loop only (without the increments) */
                            break 'chl;
                        }
                        if self.flush_status != 0
                            && (self.flush_cnt > 0)
                            && (flags & AACDEC_CONCEAL == 0)
                        {
                            self.p_time_data2[offset..offset + spf as usize].fill(0);
                        } else {
                            match self.p_aac_decoder_channel_info[cu].render_mode {
                                AACDEC_RENDER_IMDCT => {
                                    let frame_ok_arg = (self.frame_ok != 0
                                        && (flags & AACDEC_CONCEAL == 0)
                                        && !frame_ok_but_conceal)
                                        as i32;
                                    cblock_frequency_to_time(
                                        &mut self.p_aac_decoder_static_channel_info[cu],
                                        &mut self.p_aac_decoder_channel_info[cu],
                                        &mut self.work_buffer_core2[cu * 1024..(cu + 1) * 1024],
                                        &mut self.p_time_data2[offset..],
                                        spf as i16,
                                        frame_ok_arg,
                                        self.work_buffer_core1.mdct_out_temp(),
                                        self.aac_out_data_headroom,
                                        self.el_flags[el],
                                        el_ch,
                                    );
                                    if self.flags[stream_index] & AC_MPEGH3DA != 0 {
                                        let st = &mut self.p_aac_decoder_static_channel_info[cu];
                                        ltp_post(
                                            &mut self.p_time_data2[offset..],
                                            spf,
                                            self.stream_info.aac_sample_rate,
                                            &st.ltp_param,
                                            &mut st.ltp_pitch_int_past,
                                            &mut st.ltp_pitch_fr_past,
                                            &mut st.ltp_gain_past,
                                            &mut st.ltp_gain_idx_past,
                                            &mut st.ltp_mem_in,
                                            &mut st.ltp_mem_out,
                                        );
                                    }
                                }
                                _ => {
                                    error_status = AAC_DEC_UNKNOWN;
                                }
                            }
                        }
                        /* TimeDomainFading */
                        if cconceal_td_fading_applied[cu] == 0 {
                            cconceal_td_fading_applied[cu] = cconcealment_td_fading(
                                spf,
                                &mut self.p_aac_decoder_static_channel_info[cu].concealment_info,
                                &self.conceal_common_data,
                                self.aac_out_data_headroom,
                                &mut self.p_time_data2[offset..],
                            );
                            if c + 1 < 28 && c < aac_channels - 1 {
                                /* update next TDNoise Seed to avoid muting in case of Parametric
                                 * Stereo */
                                let seed = self.p_aac_decoder_static_channel_info[cu]
                                    .concealment_info
                                    .td_noise_seed;
                                self.p_aac_decoder_static_channel_info[cu + 1]
                                    .concealment_info
                                    .td_noise_seed = seed;
                            }
                        }
                    }
                    c += 1;
                    c_idx += 1;
                    el_ch += 1;
                }
            }
        }

        /* Add additional concealment delay */
        self.stream_info.output_delay = self.stream_info.output_delay.wrapping_add(
            cconcealment_get_delay(Some(&self.conceal_common_data))
                .wrapping_mul(self.stream_info.aac_samples_per_frame as u32),
        );

        if self.flags[stream_index] & (AC_MPEGH3DA | AC_USAC | AC_RSVD50) == 0 {
            let mut el = 0;
            while el < 28 + 28 {
                if self.elements[el] == ID_NONE {
                    break;
                }
                el += 1;
            }
            self.stream_info.num_elements = el as i32;
        }

        self.block_number += 1;

        error_status
    }
}

/// `PcmDataPayload(earconDecoder, TimeData, BaseframeSize, drcStatus_targetLoudness,
/// defaultTargetLoudness, targetLayout, LastFrameSamples, NewFrameSamples)`: add the PCM earcon
/// to the time signal (`pTimeData2`, channel `ch` at `time_data[BaseframeSize * ch..]`).
#[allow(clippy::too_many_arguments)]
pub fn pcm_data_payload(
    earcon_decoder: &mut EarconDecoder,
    time_data: &mut [FixpDbl],
    baseframe_size: u32,
    drc_status_target_loudness: i8,
    default_target_loudness: i8,
    target_layout: i32,
    mut last_frame_samples: i16,
    new_frame_samples: i16,
) -> TransportDecError {
    let mut earcon_gain: FixpDbl;
    let mut att_gain: FixpDbl;
    let mut att_gain_increment: FixpDbl;
    let mut earcon_shift: i32;
    let mut att_gain_shift: i32;

    if earcon_decoder.accumulated_frame_size <= 0 {
        /*Prepare for the next time*/
        earcon_decoder.accumulated_frame_size = 0;
        if last_frame_samples as i32 + new_frame_samples as i32 > 0 {
            earcon_decoder.first_frame = 1;
        }
        return TRANSPORTDEC_OK;
    }

    let config = &mut earcon_decoder.earcon_config;

    if (config.m_pcm_has_attenuation_gain != 0)
        && (config.m_bs_pcm_attenuation_gain != earcon_decoder.m_bs_pcm_attenuation_gain_old)
    {
        /*We calculate y=-0.25*m_bsPcmAttenuationGain */
        let mut headroom = f_norm(-(config.m_bs_pcm_attenuation_gain as i32));

        /*Left allign negated value */
        att_gain = (-(config.m_bs_pcm_attenuation_gain as i32)) << headroom;
        /*The exponent is 31 (i.e. int)  - headroom  -2 (from multiplication by 0.25)*/
        let mut att_gain_exp = 31 - headroom - 2;

        /* Calculate  y=10^(x/20), which can be represented as:
        y=2^(x*log2(10)*0.05), where:
        log2(10)*0.05 is 0x550a9685 in Q-2.33 format */
        let scaled_value: FixpDbl = 0x550a9685;
        att_gain = f_mult(att_gain, scaled_value);
        /*Adjust exponent*/
        att_gain_exp -= 2;
        /*Calculate pow2()*/
        let e = att_gain_exp;
        att_gain = f2_pow(att_gain, e, &mut att_gain_exp);

        /*AttGain is less or equal to 1 (0x40000000 with exp=1). Make sure the exponent is always
         * <=0 to simplify the calcuations later*/
        if (att_gain >= 0x40000000) && (att_gain_exp > 0) {
            att_gain = MAXVAL_DBL;
            att_gain_exp = 0;
        } else {
            headroom = f_norm(att_gain);
            att_gain <<= headroom;
            att_gain_exp -= headroom;
        }
        if config.m_bs_pcm_attenuation_gain == 255 {
            att_gain = 0; /* special case: map -63.75 dB => -inf dB */
        }

        /*Calculate ramp up/down. Compare present and past gains and calculate increment/decrement */

        /*Keep the old PRECISE values for calculation purposes*/
        let mut att_gain_old = earcon_decoder.att_gain_store_high_precision;
        let att_gain_exp_old = earcon_decoder.att_gain_exp_store_high_precision;
        earcon_decoder.att_gain_store_high_precision = att_gain;
        earcon_decoder.att_gain_exp_store_high_precision = att_gain_exp;

        /*Calculate increment/decrement and adjust exponent */
        let diff = att_gain_exp - att_gain_exp_old;
        if diff >= 0 {
            att_gain_old = att_gain_old.wrapping_shr(diff as u32);
            att_gain_shift = -att_gain_exp;
        } else {
            att_gain = att_gain.wrapping_shr((-diff) as u32);
            att_gain_shift = -att_gain_exp_old;
        }

        /*Calculate gain increment value*/
        if baseframe_size == 1024 {
            att_gain_increment = att_gain.wrapping_sub(att_gain_old) >> 10;
        } else if baseframe_size == 2048 {
            att_gain_increment = att_gain.wrapping_sub(att_gain_old) >> 11;
        } else {
            /*Calculate ramp-up/down multiplicative factor.
            Needed for truncations and for frame lengths that
            are not a power of two */
            let factor = f_div_norm_noexp(1, baseframe_size as FixpDbl);
            att_gain_increment = f_mult(att_gain.wrapping_sub(att_gain_old), factor);
        }

        /*Present-past adjustment*/
        earcon_decoder.att_gain_old = att_gain_old;
        earcon_decoder.att_gain_shift_old = att_gain_shift;
        earcon_decoder.att_gain_increment_old = att_gain_increment;

        /*Store the present Attenuation parameters*/
        earcon_decoder.att_gain_new = att_gain;
        earcon_decoder.att_gain_shift_new = att_gain_shift;

        /*Store the last attenuation gain*/
        earcon_decoder.m_bs_pcm_attenuation_gain_old = config.m_bs_pcm_attenuation_gain;
    }

    let config = earcon_decoder.earcon_config;

    /*Calculate the gain applied to the Earcon data*/
    earcon_gain = earcon_decoder.earcon_gain;
    earcon_shift = earcon_decoder.earcon_shift;

    let target_loudness: i32 =
        if drc_status_target_loudness == UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS {
            -(default_target_loudness as i32)
        } else {
            4 * drc_status_target_loudness as i32
        };

    if (earcon_decoder.target_loudness_old != target_loudness)
        || (earcon_decoder.m_bs_pcm_loudness_value_old != config.m_bs_pcm_loudness_value)
    {
        let mut earcon_gain_new: FixpDbl;
        let earcon_shift_new: i32;

        if target_loudness <= 0 {
            /*Set constant 57.75 in Q29.2 */
            let fifty_seven: i32 = 231;

            /* Loudness Normalization Gain = targetLoudness (LKFS) - PCM loudness (LKFS) */
            /* Calculate: "targetLoudness(LKFS) -(-57.75f + 0.25f *
             * hEarconDecoder->pcmConfig.bsPcmLoudnessValue)" in Q29.2 format */
            earcon_gain_new = target_loudness + fifty_seven - config.m_bs_pcm_loudness_value as i32;
            let mut earcon_gain_exp = 29;

            /*Adjust precision*/
            let mut headroom = f_norm(earcon_gain_new);
            earcon_gain_new <<= headroom;
            earcon_gain_exp -= headroom;

            /* Calculate  y=10^(x/20), which can be represented as:
            y=2^(x*log2(10)*0.05), where:
            log2(10)*0.05 is 0x550a9685 in Q-2.33 format */
            let scaled_value: FixpDbl = 0x550a9685;
            earcon_gain_new = f_mult(earcon_gain_new, scaled_value);
            /*Adjust exponent*/
            earcon_gain_exp -= 2;
            /*Calculate pow2()*/
            let e = earcon_gain_exp;
            earcon_gain_new = f2_pow(earcon_gain_new, e, &mut earcon_gain_exp);

            /*Adjust precision*/
            headroom = f_norm(earcon_gain_new);
            earcon_gain_new <<= headroom;
            earcon_gain_exp -= headroom;

            /*Shift to adjust to the output format*/
            earcon_shift_new = PCM_OUT_HEADROOM - earcon_gain_exp;
        } else {
            /* Loudness Normalization off. Use gain of 1.0 */
            earcon_gain_new = 0x40000000;
            earcon_shift_new = PCM_OUT_HEADROOM - 1;
        }

        /*Store for the future*/
        earcon_decoder.earcon_gain = earcon_gain_new;
        earcon_decoder.earcon_shift = earcon_shift_new;
        earcon_decoder.m_bs_pcm_loudness_value_old = config.m_bs_pcm_loudness_value;
        earcon_decoder.target_loudness_old = target_loudness;
    }

    /* Process delayed past signal LastFrameSamples and part of current NewFrameSamples */
    {
        /*Calculate old/new signal lengths and startPoint */
        let mut start_point: i32 = 0;

        if earcon_decoder.first_frame == 1 {
            start_point = last_frame_samples as i32;
            last_frame_samples = 0;
        }

        earcon_decoder.first_frame = 0;

        'bail: {
            let mut num_speakers: i32 = earcon_decoder.num_pcm_signals_old;
            let mut num_signals_mixed = earcon_decoder.num_signals_mixed;
            let mut number_of_rest_channels = earcon_decoder.number_of_rest_channels;

            let mut i: i32 = 0;
            let mut loop_counter_value: i32 = last_frame_samples as i32;
            let mut earcon_data_pointer: usize = 0;
            let bfs = baseframe_size as usize;

            /*Set initial gains*/
            att_gain = earcon_decoder.att_gain_intermediate;
            att_gain_shift = earcon_decoder.att_gain_shift_intermediate;
            att_gain_increment = earcon_decoder.att_gain_increment_intermediate;

            for j in 0..2 {
                /*Apply MPEGH attenuation gain on the non-Earcon channels*/
                {
                    if earcon_decoder.num_pcm_signals_old != num_speakers {
                        let mut num_total_channels = 0;

                        cicp2geometry_get_num_channels_from_cicp(
                            target_layout,
                            &mut num_total_channels,
                        );

                        if cicp2geometry_get_front_speakers(
                            target_layout,
                            num_speakers,
                            &mut num_signals_mixed,
                            &mut earcon_decoder.speaker_pos_indices,
                            &mut earcon_decoder.speaker_gains,
                        ) != CICP2GEOMETRY_OK
                        {
                            break 'bail;
                        }

                        /*Generate a signal matrix with non-Earcon signal positions*/
                        let mut k = 0usize;
                        for i2 in 0..num_total_channels {
                            let mut found = 0;
                            for j2 in 0..num_signals_mixed as usize {
                                if earcon_decoder.speaker_pos_indices[j2] == i2 {
                                    found = 1;
                                    break;
                                }
                            }
                            if found == 0 {
                                earcon_decoder.speaker_pos_indices_rest[k] = i2;
                                k += 1;
                            }
                        }

                        earcon_decoder.num_signals_mixed = num_signals_mixed;
                        number_of_rest_channels = num_total_channels - num_signals_mixed;
                        earcon_decoder.number_of_rest_channels = number_of_rest_channels;
                    }

                    /*If gain is always 1.0 skip processing.*/
                    if (att_gain == 0x7fffffff)
                        && (att_gain_shift == 0)
                        && (att_gain_increment == 0)
                    {
                        number_of_rest_channels = 0;
                    }

                    for k in 0..number_of_rest_channels.max(0) as usize {
                        /*Pointer to the respective channel*/
                        let base = bfs * earcon_decoder.speaker_pos_indices_rest[k] as usize;

                        let mut att_gain_temp = att_gain;
                        for ir in i..loop_counter_value {
                            let d = &mut time_data[base + ir as usize];
                            *d = f_mult(*d, att_gain_temp).wrapping_shr(att_gain_shift as u32);
                            att_gain_temp = att_gain_temp.wrapping_add(att_gain_increment);
                        }
                    }
                }
                let speaker_gains = earcon_decoder.speaker_gains;
                let speaker_pos_indices = earcon_decoder.speaker_pos_indices;
                /*Adjust for the changed speaker gain*/
                let mut earcon_gain2 = f_mult(earcon_gain, speaker_gains[0] as FixpDbl);

                /*Set target pointers*/
                let d1 = bfs * speaker_pos_indices[0] as usize;
                let d2 = bfs.wrapping_mul(speaker_pos_indices[1] as usize);

                /* Limit samples read */
                if loop_counter_value > 0 && num_speakers > 0 {
                    loop_counter_value = loop_counter_value
                        .min(start_point + earcon_decoder.accumulated_frame_size / num_speakers);
                }
                let data = &earcon_decoder.earcon_data;
                if (num_speakers == 1) && (num_signals_mixed == 1) {
                    while i < loop_counter_value {
                        let tco: FixpSgl = data[earcon_data_pointer];
                        earcon_data_pointer += 1;

                        let s = &mut time_data[d1 + i as usize];
                        *s = f_mult(*s, att_gain).wrapping_shr(att_gain_shift as u32).wrapping_add(
                            f_mult(tco, earcon_gain2).wrapping_shr(earcon_shift as u32),
                        );

                        att_gain = att_gain.wrapping_add(att_gain_increment);
                        i += 1;
                    }
                } else if (num_speakers == 2) && (num_signals_mixed == 2) {
                    while i < loop_counter_value {
                        let tco1: FixpSgl = data[earcon_data_pointer];
                        let tco2: FixpSgl = data[earcon_data_pointer + 1];
                        earcon_data_pointer += 2;

                        let s = &mut time_data[d1 + i as usize];
                        *s = f_mult(*s, att_gain).wrapping_shr(att_gain_shift as u32).wrapping_add(
                            f_mult(tco1, earcon_gain2).wrapping_shr(earcon_shift as u32),
                        );
                        let s = &mut time_data[d2 + i as usize];
                        *s = f_mult(*s, att_gain).wrapping_shr(att_gain_shift as u32).wrapping_add(
                            f_mult(tco2, earcon_gain2).wrapping_shr(earcon_shift as u32),
                        );

                        att_gain = att_gain.wrapping_add(att_gain_increment);
                        i += 1;
                    }
                } else if (num_speakers == 1) && (num_signals_mixed == 2) {
                    /*Adjust for the changed speaker gain. The gains are the same for the two
                     * channels */
                    earcon_gain2 = f_mult(earcon_gain2, speaker_gains[0] as FixpDbl);

                    while i < loop_counter_value {
                        let tco: FixpSgl = data[earcon_data_pointer];
                        earcon_data_pointer += 1;
                        let scaled_input =
                            f_mult(tco, earcon_gain2).wrapping_shr(earcon_shift as u32);

                        let s = &mut time_data[d1 + i as usize];
                        *s = f_mult(*s, att_gain)
                            .wrapping_shr(att_gain_shift as u32)
                            .wrapping_add(scaled_input);
                        let s = &mut time_data[d2 + i as usize];
                        *s = f_mult(*s, att_gain)
                            .wrapping_shr(att_gain_shift as u32)
                            .wrapping_add(scaled_input);

                        att_gain = att_gain.wrapping_add(att_gain_increment);
                        i += 1;
                    }
                } else if (num_speakers == 2) && (num_signals_mixed == 1) {
                    /*Adjust for the changed speaker gain*/
                    earcon_gain2 = f_mult(earcon_gain2, speaker_gains[0] as FixpDbl);

                    while i < loop_counter_value {
                        let tco: FixpSgl = data[earcon_data_pointer];
                        earcon_data_pointer += 1;
                        let s = &mut time_data[d1 + i as usize];
                        *s = f_mult(*s, att_gain).wrapping_shr(att_gain_shift as u32).wrapping_add(
                            f_mult(tco, earcon_gain2).wrapping_shr(earcon_shift as u32),
                        );
                        let tco: FixpSgl = data[earcon_data_pointer];
                        earcon_data_pointer += 1;
                        *s = s.wrapping_add(
                            f_mult(tco, earcon_gain2).wrapping_shr(earcon_shift as u32),
                        );
                        att_gain = att_gain.wrapping_add(att_gain_increment);
                        i += 1;
                    }
                }
                i = i.max(start_point);
                loop_counter_value = i + new_frame_samples as i32;

                if j == 0 {
                    /* Switch to gains of current frame */
                    att_gain = earcon_decoder.att_gain_old;
                    att_gain_shift = earcon_decoder.att_gain_shift_old;
                    att_gain_increment = earcon_decoder.att_gain_increment_old;

                    earcon_gain = earcon_decoder.earcon_gain;
                    earcon_shift = earcon_decoder.earcon_shift;
                }
                num_speakers = config.m_num_pcm_signals as i32;
            }

            /*Left align buffer*/
            let overall_samples_used = last_frame_samples as i32
                * earcon_decoder.num_pcm_signals_old
                + new_frame_samples as i32 * num_speakers;
            earcon_decoder.accumulated_frame_size -= overall_samples_used;
            if earcon_decoder.accumulated_frame_size < 0 {
                earcon_decoder.accumulated_frame_size = 0;
            }
            debug_assert!(
                (earcon_decoder.accumulated_frame_size + overall_samples_used) as usize
                    <= EARCON_BUFFER_SIZE
            );
            {
                let src = overall_samples_used.max(0) as usize;
                let n = (earcon_decoder.accumulated_frame_size as usize)
                    .min(EARCON_BUFFER_SIZE.saturating_sub(src));
                earcon_decoder.earcon_data.copy_within(src..src + n, 0);
            }
            earcon_decoder.num_pcm_signals_old = num_speakers;

            /* Save intermediate gain for next time to finish the current frame */
            earcon_decoder.att_gain_intermediate = att_gain;
            earcon_decoder.att_gain_shift_intermediate = att_gain_shift;
            earcon_decoder.att_gain_increment_intermediate = earcon_decoder.att_gain_increment_old;

            /* Pass new to old gains */
            earcon_decoder.att_gain_old = earcon_decoder.att_gain_new;
            earcon_decoder.att_gain_shift_old = earcon_decoder.att_gain_shift_new;
            earcon_decoder.att_gain_increment_old = 0;

            earcon_decoder.truncation_present = 0;
            if earcon_decoder.earcon_config.truncation_flag != 0 {
                earcon_decoder.truncation_present = 1;
                earcon_decoder.earcon_config.truncation_flag = 0;
            }
        }
    }

    TRANSPORTDEC_OK
}
