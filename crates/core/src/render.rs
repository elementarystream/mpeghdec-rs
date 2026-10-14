// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/aacdecoder_lib.cpp (the rendering part of aacDecoder_DecodeFrame()).

//! The rendering stage of `aacDecoder_DecodeFrame()` and the frame pipelining (not in C).
//!
//! `aacDecoder_DecodeFrame()` decodes an access unit in two stages:
//!
//! - **core** (stage A): transport decoder (`transportDec_ReadAccessUnit()` with its callbacks),
//!   pre-roll parsing and `CAacDecoder_DecodeFrame()` (parsing, element decoding, concealment,
//!   inverse transforms) into the time signal in `pTimeData2`;
//! - **render** (stage B): DRC, format conversion, object rendering, sample rate conversion,
//!   truncation, cross-fade, earcons, limiter and output interleaving.
//!
//! The user interactivity (`applyUserInteractivity()`) at the start of the rendering is split: its
//! control part (UI status, loudness compensation, DRC selection parameters, object position
//! offsets) runs at the end of the core stage, only the channel gains are applied by the render
//! stage.
//!
//! The state that only the rendering uses is [`RenderState`], owned by the decoder instance
//! ([`AacDecoderInstance::render`]). The render functions take it together with a
//! [`RenderCtx`] (the core decoder state the rendering reads), the stream info and the time
//! buffers as arguments, so they cannot touch the state of the core stage.
//!
//! # Frame pipelining
//!
//! With a render thread ([`AacDecoderInstance::set_render_worker`]), the render stage of access
//! unit N runs on that thread while the core stage decodes access unit N+1
//! ([`AacDecoderInstance::aac_decoder_decode_frame_pipelined`]). The render state and a snapshot
//! of the core state it reads are moved to the render thread; the core stage gets them back
//! when it needs them ([`AacDecoderInstance::join_render`], called by every access to the render
//! state from the core stage, see [`rs!`](crate::rs)), at the latest before the render stage of
//! the next access unit. The order of all operations on every piece of state is therefore the
//! sequential one, with these exceptions (see PORTING.md, "Frame pipelining"):
//!
//! - The time buffer `pTimeData2` and the work buffer `workBufferCore2` exist twice: the render
//!   stage works on the buffer the core stage decoded into and hands its previous buffer to the
//!   core stage. Neither stage reads the content the other stage left in these buffers
//!   (checked with the buffer poisoning of [`AacDecoderInstance::poison_buffers`]; the IGF
//!   `virtualSpec`, which may read stale content of `pTimeData2`, is written before it is read
//!   in the configurations that are pipelined, see `AacDecoderInstance::igf_reads_stale_data()`).
//! - The DRC gain and object metadata payloads that the core stage parses while the render
//!   stage runs are parsed twice: once on a copy of the parser state (for the bit position and
//!   the error status the core stage needs) and again on the real state before the next render
//!   stage (`ParseRecord`), checking that both parses agree.
//! - `CStreamInfo`: the render stage writes its fields into its own copy, which is returned
//!   with the rendered frame ([`FrameResult`]); the core stage sets the fields it reads itself
//!   (the number of output channels) to the values the render stage produces.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::PcmDec;
use crate::aacdecoder::*;
use crate::aacdecoder_lib::*;
use crate::delay::FdkSignalDelay;
use crate::format_converter_glue::fc_process;
use crate::format_converter_glue::FormatConverter;
use crate::pipeline::{Pending, RenderWorker};
use mpeghdec_drc::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::cicp2geometry_get_num_channels_from_cicp_noerr;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::mdct::PCM_OUT_HEADROOM;
use mpeghdec_fdk::scale::{get_scalefactor, scale_values_saturate_to};
use mpeghdec_fdk::stftfilterbank::*;
use mpeghdec_fdk::td_upsampler::*;
use mpeghdec_fdk::thread_pool::ThreadPool;
use mpeghdec_formatconverter::{
    IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT, IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN,
};
use mpeghdec_gvbap::*;
use mpeghdec_pcmutils::limiter::*;
use mpeghdec_pcmutils::pcm_utils::fdk_interleave;
use mpeghdec_tpdec::*;
use std::sync::Arc;

/// Accesses the render state from the core stage: waits for a running render stage first
/// ([`AacDecoderInstance::join_render`]). Borrows only the field `render`, so other fields of
/// the instance can be used in the same expression.
#[macro_export]
macro_rules! rs {
    ($s:expr) => {{
        $s.join_render();
        $s.render.as_deref_mut().expect("render state")
    }};
}

/// The state used by the rendering stage of `aacDecoder_DecodeFrame()` (fields of
/// `AAC_DECODER_INSTANCE`, see the module documentation).
#[derive(StateDigest)]
pub struct RenderState {
    pub h_uni_drc_decoder: Box<DrcDecoder>,
    /// Handle of time domain limiter.
    pub h_limiter: Option<TDLimiter>,
    /// handle to gVBAP data structure
    pub hg_vbap_renderer: Vec<Option<Box<GVbapRenderer>>>,
    pub stft_filterbank_analysis: Vec<Option<HandleStftFilterbank>>,
    pub stft_filterbank_synthesis: Vec<Option<HandleStftFilterbank>>,
    pub stft_headroom_prescaling: [i8; TP_MPEGH_MAX_SIGNAL_GROUPS],
    /// Format converter instance (`pFormatConverter[0]`; the others are never used).
    pub p_format_converter: Option<Box<FormatConverter>>,
    /// If any bit is set, cross-fade for seamless stream switching is applied
    pub apply_crossfade: u8,
    /* Truncation handler queue. Offsets are relative to the current frame, before the limiter, at
     * delay position 775. */
    /// Start offset from where samples removal starts
    pub truncate_start_offset: [i16; TRUNC_QUEUE_SIZE],
    /// Stop offset until where samples are removed
    pub truncate_stop_offset: [i16; TRUNC_QUEUE_SIZE],
    /// Amount of truncated (removed) samples of current frame
    pub truncate_sample_count: i16,
    /// Counter for discarding samples of delay at start
    pub discard_samples_at_start_cnt: i32,
    /// Memory for saving samples for MPEG-H crossfade
    pub crossfade_mem: Vec<PcmDec>,
    /// MPEG-H delay compensation for rendering chain (constant delay).
    pub mpeg_h_rendered_delay: FdkSignalDelay,
    pub delay_buffer: Vec<[PcmDec; 256]>,
    /// MPEG-H sample rate converter for upsampling to output sample rate
    pub mpeg_h_sample_rate_converter_filter_states: [[FixpDbl; TD_STATES_MEM_SIZE]; 24],
    pub earcon_decoder: EarconDecoder,

    /// Frame pipelining (not in C): the time buffer (`pTimeData2`) of the render stage, exchanged
    /// with the one of the core stage for every pipelined access unit (empty until the first
    /// one). Not part of the state digest (like `pTimeData2` of the core stage, whose digest
    /// covers the content that may be read stale).
    #[digest(skip)]
    pub time_data2: Vec<PcmDec>,
    /// Frame pipelining (not in C): the work buffer (`workBufferCore2`) of the render stage.
    /// Scratch memory, not part of the state digest.
    #[digest(skip)]
    pub work_buffer: Vec<FixpDbl>,
    /// Fast mode (not in C): frames bypassed by the format converter instances closed before.
    /// Only reported, not part of the state digest.
    #[digest(skip)]
    pub fast_bypass_frames: u64,
}

impl RenderState {
    /// The zero-initialized render state (the decoder instance is `calloc()`ed).
    pub(crate) fn new() -> Box<RenderState> {
        Box::new(RenderState {
            h_uni_drc_decoder: DrcDecoder::open(DRC_DEC_ALL),
            h_limiter: None,
            hg_vbap_renderer: (0..TP_MPEGH_MAX_SIGNAL_GROUPS).map(|_| None).collect(),
            stft_filterbank_analysis: (0..AAC_MAX_SIGNALS).map(|_| None).collect(),
            stft_filterbank_synthesis: (0..AAC_MAX_SIGNALS).map(|_| None).collect(),
            stft_headroom_prescaling: [0; TP_MPEGH_MAX_SIGNAL_GROUPS],
            p_format_converter: None,
            apply_crossfade: 0,
            truncate_start_offset: [0; TRUNC_QUEUE_SIZE],
            truncate_stop_offset: [0; TRUNC_QUEUE_SIZE],
            truncate_sample_count: 0,
            discard_samples_at_start_cnt: 0,
            crossfade_mem: vec![0; 128 * 24],
            mpeg_h_rendered_delay: FdkSignalDelay::default(),
            delay_buffer: vec![[0; 256]; 28],
            mpeg_h_sample_rate_converter_filter_states: [[0; TD_STATES_MEM_SIZE]; 24],
            earcon_decoder: EarconDecoder::default(),
            time_data2: Vec::new(),
            work_buffer: Vec::new(),
            fast_bypass_frames: 0,
        })
    }
}

/// A signal group of a configuration as read by the rendering (`CSSignalGroup`).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SigGroup {
    pub type_: u8,
    pub count: u8,
    pub first_sig_idx: u8,
}

/// The signal groups of the configuration of a (sub)stream (`pUsacConfig[s]`).
#[derive(StateDigest, Clone, Copy, Debug)]
pub struct StreamGroups {
    pub n_usac_channels: u8,
    pub num_groups: u8,
    pub groups: [SigGroup; TP_MPEGH_MAX_SIGNAL_GROUPS],
}

impl StreamGroups {
    const EMPTY: StreamGroups = StreamGroups {
        n_usac_channels: 0,
        num_groups: 0,
        groups: [SigGroup { type_: 0, count: 0, first_sig_idx: 0 }; TP_MPEGH_MAX_SIGNAL_GROUPS],
    };

    #[inline]
    fn groups(&self) -> &[SigGroup] {
        &self.groups[..self.num_groups as usize]
    }
}

/// The core decoder state read by the rendering stage (a snapshot taken at the end of the core
/// stage of the access unit).
#[derive(StateDigest, Clone)]
pub struct RenderCtx {
    /// `pUsacConfig[s]` of the (sub)streams up to the first `NULL`.
    pub streams: [StreamGroups; TPDEC_MAX_TRACKS],
    pub num_streams: usize,
    /// `signalSkipped[]` (`getOnOffFlag()`)
    pub signal_skipped: [u8; TP_MPEGH_MAX_SIGNAL_GROUPS],
    /// `flags[0]`
    pub flags0: u32,
    pub aac_channels: i32,
    pub multiband_drc_present: u8,
    pub num_time_slots: u8,
    pub stft_frame_size: u16,
    pub target_layout: i32,
    pub target_layout_config: i32,
    pub flush_status: u8,
    pub default_target_loudness: i8,
    /// `drcStatus.targetLoudness`
    pub drc_status_target_loudness: i8,
    pub limiter_enable_curr: u8,
    /// The channel gains of `applyUserInteractivity()` for `FDK_drcDec_SetChannelGains()`
    /// (`None` if `applyUserInteractivity()` returned before).
    pub channel_gain: Option<[FixpDbl; 28 * 2]>,
    /// Fast mode options (not in C, see [`crate::fast`]).
    pub fast: crate::fast::FastOptions,
}

impl RenderCtx {
    /// `getOnOffFlag(self, signal)`
    #[inline]
    pub fn get_on_off_flag(&self, signal: i32) -> i32 {
        self.signal_skipped.get(signal as usize).map_or(1, |&s| (s == 0) as i32)
    }

    #[inline]
    fn streams(&self) -> &[StreamGroups] {
        &self.streams[..self.num_streams]
    }

    /// Whether the channel count check of the DRC 1 loop of the rendering fails
    /// (`drcTotalChannels > aacChannels`, "configuration mismatch between signal groups and
    /// channel elements").
    pub fn drc_channels_mismatch(&self) -> bool {
        let mut signals_prev_streams: i32 = 0;
        let mut drc_total_channels: i32 = 0;
        for st in self.streams() {
            for sg in st.groups() {
                let transport_start_channel = sg.first_sig_idx as i32 + signals_prev_streams;
                let mut n = sg.count as i32;
                for sig in 0..sg.count as i32 {
                    if self.get_on_off_flag(sig + transport_start_channel) == 0 {
                        n -= 1;
                    }
                }
                drc_total_channels += n;
                if drc_total_channels > self.aac_channels {
                    return true;
                }
            }
            signals_prev_streams += st.n_usac_channels as i32;
        }
        false
    }

    /// Whether an active channel signal group exists (the format converter is applied).
    pub fn has_active_channel_group(&self) -> bool {
        let mut signals_prev_streams: i32 = 0;
        for st in self.streams() {
            for sg in st.groups() {
                if self.get_on_off_flag(sg.first_sig_idx as i32 + signals_prev_streams) != 0
                    && sg.type_ == 0
                {
                    return true;
                }
            }
            signals_prev_streams += st.n_usac_channels as i32;
        }
        false
    }
}

impl AacDecoderInstance {
    /// The [`RenderCtx`] of the current access unit.
    pub(crate) fn render_ctx(
        &self,
        ascs: &[CSAudioSpecificConfig],
        channel_gain: Option<[FixpDbl; 28 * 2]>,
    ) -> RenderCtx {
        let mut streams = [StreamGroups::EMPTY; TPDEC_MAX_TRACKS];
        let mut num_streams = 0;
        for (s, dst) in streams.iter_mut().enumerate() {
            let Some(usc) = self.usc(ascs, s) else { break };
            dst.n_usac_channels = usc.m_n_usac_channels;
            dst.num_groups = usc.bs_num_signal_groups;
            for (d, g) in dst
                .groups
                .iter_mut()
                .zip(&usc.m_signal_group_type[..usc.bs_num_signal_groups as usize])
            {
                *d = SigGroup { type_: g.type_, count: g.count, first_sig_idx: g.first_sig_idx };
            }
            num_streams += 1;
        }
        RenderCtx {
            streams,
            num_streams,
            signal_skipped: self.signal_skipped,
            flags0: self.flags[0],
            aac_channels: self.aac_channels,
            multiband_drc_present: self.multiband_drc_present,
            num_time_slots: self.num_time_slots,
            stft_frame_size: self.stft_frame_size,
            target_layout: self.target_layout,
            target_layout_config: self.target_layout_config,
            flush_status: self.flush_status,
            default_target_loudness: self.default_target_loudness,
            drc_status_target_loudness: self.drc_status.target_loudness,
            limiter_enable_curr: self.limiter_enable_curr,
            channel_gain,
            fast: self.fast_options,
        }
    }
}

/// The frame size, sample rate, upsampling factor, split frame size and hang over samples of the
/// sample rate conversion of the rendered signal (`frame_size`/`sample_rate` before the
/// conversion, `rd` the delay of the rendered signal).
pub(crate) fn upsampling_params(
    aac_sample_rate: i32,
    frame_size: i32,
    sample_rate: i32,
    rd: i32,
) -> (i32, i32, TdFacUpsample, i32, i32) {
    let mut ov_samples: i32 = 0;
    let mut split_frame_size: i32 = 0;
    let mut new_frame_size = frame_size;
    let mut new_sample_rate = sample_rate;
    let mut fac = TD_FAC_UPSAMPLE_1_1;

    match aac_sample_rate {
        14700 | 16000 => {
            new_frame_size = 3 * frame_size;
            new_sample_rate = 3 * sample_rate;
            fac = TD_FAC_UPSAMPLE_3_1;
            split_frame_size = (new_frame_size - rd) / 3;
            ov_samples = new_frame_size - rd - (3 * split_frame_size);
            if ov_samples != 0 {
                ov_samples = 3 - ov_samples;
                split_frame_size += 1;
            }
        }
        22050 | 24000 => {
            new_frame_size = 2 * frame_size;
            new_sample_rate = 2 * sample_rate;
            fac = TD_FAC_UPSAMPLE_2_1;
            split_frame_size = (new_frame_size - rd) >> 1;
            ov_samples = new_frame_size - rd - (2 * split_frame_size);
            if ov_samples != 0 {
                split_frame_size += 1;
            }
        }
        29400 | 32000 => {
            new_frame_size = (3 * frame_size) >> 1;
            new_sample_rate = (3 * sample_rate) >> 1;
            fac = TD_FAC_UPSAMPLE_3_2;
            split_frame_size = ((new_frame_size - rd) / 3) << 1;
            ov_samples = new_frame_size - rd - split_frame_size - (split_frame_size >> 1);
            if ov_samples != 0 {
                ov_samples = 3 - ov_samples;
                split_frame_size += 2;
            }
        }
        44100 | 48000 => {
            new_frame_size = frame_size;
            new_sample_rate = sample_rate;
            fac = TD_FAC_UPSAMPLE_1_1;
            split_frame_size = new_frame_size - rd;
            ov_samples = 0;
        }
        _ => {}
    }
    (new_frame_size, new_sample_rate, fac, split_frame_size, ov_samples)
}

impl RenderState {
    /// The MPEG-H rendering of `aacDecoder_DecodeFrame()` after the control part of
    /// `applyUserInteractivity()`: its channel gains, DRC 1, format conversion, object rendering,
    /// DRC 2/3 and sample rate conversion into `pTimeData2` (`time_data2`, using
    /// `workBufferCore2` (`core2`) as output buffer of the renderers).
    ///
    /// Returns `false` (with `error_status` set) if the rendering failed.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn render(
        &mut self,
        ctx: &RenderCtx,
        si: &mut CStreamInfo,
        time_data2: &mut [PcmDec],
        core2: &mut [FixpDbl],
        pool: Option<&ThreadPool>,
        error_status: &mut AacDecoderError,
    ) -> bool {
        /* Temporary downmix buffers */
        let mut p_time_data_in: usize = 0; /* pTimeData2 */

        /* set UI gains (in dB) which will be applied in DRC decoder (applyUserInteractivity()) */
        if let Some(channel_gain) = &ctx.channel_gain {
            self.h_uni_drc_decoder.set_channel_gains(
                1,
                si.num_channels,
                si.frame_size,
                channel_gain,
                &mut time_data2[p_time_data_in + 256..],
                si.frame_size + 256,
            );
        }

        let mut signals_prev_streams: i32 = 0;
        /* Apply DRC 1 (before downmix/rendering) */
        let mut drc_num_channels: i32;
        let mut drc_start_channel: i32 = 0;
        let mut drc_total_channels: i32 = 0;

        self.h_uni_drc_decoder.preprocess();

        let fs256 = si.frame_size + 256;
        for st in ctx.streams() {
            for (grp, sg) in st.groups().iter().enumerate() {
                let mut transport_start_channel = sg.first_sig_idx as i32 + signals_prev_streams;
                let mut transport_num_channels = sg.count as i32;
                let signal_group_type = sg.type_;
                let process_time_domain_drc = ctx.multiband_drc_present == 0;
                let mut process_stft_domain_drc_objects_or_hoa = false;

                /* get start channel index referenced in DRC configuration */
                match signal_group_type {
                    0 => {
                        /* channels */
                        drc_num_channels = transport_num_channels;
                    }
                    1 => {
                        /* objects */
                        process_stft_domain_drc_objects_or_hoa = ctx.multiband_drc_present != 0;
                        drc_num_channels = transport_num_channels;
                    }
                    _ => {
                        drc_num_channels = transport_num_channels;
                    }
                }

                /* substract inactive channels */
                let mut transport_num_channels2 = transport_num_channels;
                for sig in 0..transport_num_channels {
                    if ctx.get_on_off_flag(sig + transport_start_channel) == 0 {
                        transport_num_channels2 -= 1;
                    }
                }
                transport_num_channels = transport_num_channels2;
                let mut transport_start_channel2 = transport_start_channel;
                for sig in 0..transport_start_channel {
                    if ctx.get_on_off_flag(sig) == 0 {
                        transport_start_channel2 -= 1;
                    }
                }
                transport_start_channel = transport_start_channel2;
                drc_total_channels += transport_num_channels;

                if drc_total_channels > ctx.aac_channels {
                    /* configuration mismatch between signal groups and channel elements */
                    *error_status = AAC_DEC_UNKNOWN;
                    return false;
                }

                if process_time_domain_drc && transport_num_channels > 0 {
                    self.h_uni_drc_decoder.process_time(
                        0,
                        DRC_DEC_DRC1,
                        transport_start_channel,
                        drc_start_channel - transport_start_channel,
                        transport_num_channels,
                        &mut time_data2[256..],
                        fs256,
                    );
                }

                if process_stft_domain_drc_objects_or_hoa && transport_num_channels > 0 {
                    let mut stft_buffer = [0 as FixpDbl; 512]; /* 2*(stft frame size) */
                    let sfs = ctx.stft_frame_size as i32;
                    for i in 0..ctx.num_time_slots as i32 {
                        /* get headroom */
                        let mut stft_headroom = 31;
                        let mut stft_headroom_prescaling;
                        for ch in 0..transport_num_channels {
                            let o =
                                ((transport_start_channel + ch) * fs256 + i * sfs + 256) as usize;
                            stft_headroom = stft_headroom
                                .min(get_scalefactor(&time_data2[o..o + sfs as usize]));
                        }
                        /* Ensure PCM_OUT_HEADROOM bit headroom */
                        stft_headroom_prescaling = (stft_headroom - PCM_OUT_HEADROOM).max(0);
                        /* Don't prescale more than necessary */
                        stft_headroom_prescaling = stft_headroom_prescaling.min(PCM_OUT_HEADROOM);
                        /* Ensure at least one bit headroom for FFT */
                        if stft_headroom == 0 {
                            stft_headroom_prescaling = -1;
                        }

                        let stft_headroom_prescaling_min = (self.stft_headroom_prescaling[grp]
                            as i32)
                            .min(stft_headroom_prescaling);

                        /* Transform to frequency domain*/
                        for ch in 0..transport_num_channels {
                            let tsc = (transport_start_channel + ch) as usize;
                            let o =
                                ((transport_start_channel + ch) * fs256 + i * sfs + 256) as usize;
                            if let Some(fb) = self.stft_filterbank_analysis[tsc].as_deref_mut() {
                                stft_filterbank_process(
                                    &mut time_data2[o..o + sfs as usize],
                                    &mut stft_buffer,
                                    fb,
                                    stft_headroom_prescaling_min,
                                );
                            }

                            /* Delay of DRC gains: Delay of STFT Analysis (256) - 128 because
                             * MPEG-D DRC downsamples at the middle of each timeslot */
                            self.h_uni_drc_decoder.process_freq(
                                128,
                                DRC_DEC_DRC1,
                                transport_start_channel + ch,
                                drc_start_channel - transport_start_channel,
                                1,
                                i,
                                &mut [&mut stft_buffer[..]],
                                None,
                            );

                            /* clear output buffer first, as StftFilterbank_Process accumulates its
                             * output signal to the buffer */
                            time_data2[o..o + sfs as usize].fill(0);
                            /* Transform to time domain */
                            if let Some(fb) = self.stft_filterbank_synthesis[tsc].as_deref_mut() {
                                stft_filterbank_process(
                                    &mut stft_buffer,
                                    &mut time_data2[o..o + sfs as usize],
                                    fb,
                                    stft_headroom_prescaling_min,
                                );
                            }
                        }
                        /* Keep the current signal headroom correction for use in the next frame */
                        self.stft_headroom_prescaling[grp] = stft_headroom_prescaling as i8;
                    }
                }
                drc_start_channel += drc_num_channels;

                /* Compensate STFT delay in object and HOA signal path in case of time domain
                 * DRC. */
                if process_time_domain_drc {
                    let fsz = si.frame_size as usize;
                    for ch in 0..transport_num_channels2 {
                        let c = (transport_start_channel2 + ch) as usize;
                        let o = c * fs256 as usize;
                        let tmp = self.delay_buffer[c];
                        self.delay_buffer[c].copy_from_slice(&time_data2[o + fsz..o + fsz + 256]);
                        time_data2[o..o + 256].copy_from_slice(&tmp);
                    }
                }
            }
            signals_prev_streams += st.n_usac_channels as i32;
        }

        /* Amount of rendered channels. Does not change when we export the MPEGH
         * channels/objects/HOA, since the format converter is skipped*/
        si.num_channels = cicp2geometry_get_num_channels_from_cicp_noerr(ctx.target_layout_config);

        /* Clear output buffer, were each renderer will mix its output into. */
        if (ctx.flags0 & AC_MPEGH3DA != 0) && (ctx.target_layout_config > -1) {
            let n = (si.num_channels * si.frame_size).max(0) as usize;
            core2[..n].fill(0);
        }

        let mut num_obj_group = 0usize;
        let mut fc_done = 0; /* flag for running format converter only once */
        let mut p_format_conv_in: Vec<usize> = Vec::with_capacity(28);

        {
            let mut p = p_time_data_in + 256;
            let mut sig = 0i32;

            for st in ctx.streams() {
                for sg in st.groups() {
                    let on = ctx.get_on_off_flag(sig);
                    sig += sg.count as i32;
                    if on == 0 {
                        continue;
                    }

                    for _ in 0..sg.count {
                        if sg.type_ == 0 {
                            p_format_conv_in.push(p);
                        }
                        p += (si.aac_samples_per_frame + 256) as usize;
                    }
                }
            }
        }

        /* DMX processing IN: pTimeData_tmp (working buffer) OUT: pTimeData_tmp2 */
        signals_prev_streams = 0;
        for st in ctx.streams() {
            let mut signals_stream = 0i32;

            for sg in st.groups() {
                /* Skip inactive signal groups. */
                if ctx.get_on_off_flag(sg.first_sig_idx as i32 + signals_prev_streams) == 0 {
                    signals_stream += sg.count as i32;
                    continue;
                }

                /* Apply format converter if there is at least one channel signal group (always
                 * first). */
                if sg.type_ == 0 && fc_done == 0 {
                    fc_done = 1;

                    let drc = if ctx.multiband_drc_present != 0 {
                        Some(&mut *self.h_uni_drc_decoder)
                    } else {
                        None
                    };
                    let err = match self.p_format_converter.as_deref_mut() {
                        Some(fc) => fc_process(
                            fc,
                            pool,
                            drc,
                            &p_format_conv_in,
                            time_data2,
                            core2,
                            si.aac_samples_per_frame + 256,
                            ctx.fast.format_converter_bypass,
                        ),
                        None => -1,
                    };

                    if err != 0 {
                        *error_status = AAC_DEC_UNKNOWN;
                        return false;
                    }
                }

                let signal_offset = if ctx.multiband_drc_present != 0 { 256 } else { 0 };

                /* Apply object rendering on the object signal group */
                if sg.type_ == 1 {
                    if let Some(r) = self.hg_vbap_renderer[num_obj_group].as_deref_mut() {
                        gvbap_renderer_render_frame_time(
                            r,
                            &time_data2[p_time_data_in + signal_offset..],
                            core2,
                            256,
                            si.frame_size + 256,
                        );
                        num_obj_group += 1;
                    }
                }
                /* Increment for active signals */
                p_time_data_in += sg.count as usize * (si.frame_size + 256) as usize;

                /* Count all signals */
                signals_stream += sg.count as i32;
            }

            signals_prev_streams += signals_stream;
        }

        /* Apply DRC 2/3 (after downmix/rendering) */
        self.h_uni_drc_decoder.process_time(
            256,
            DRC_DEC_DRC2_DRC3,
            0,
            0,
            si.num_channels,
            core2,
            si.frame_size,
        );

        let rd = self.mpeg_h_rendered_delay.delay as i32;
        let (new_frame_size, new_sample_rate, sample_rate_converter_fac_upsampling, split, ov) =
            upsampling_params(si.aac_sample_rate, si.frame_size, si.sample_rate, rd);

        /* upsampling */
        {
            let rdu = rd as usize;
            let nfs = new_frame_size as usize;
            let fsz = si.frame_size as usize;
            let sfs = split as usize;
            let ovs = ov as usize;
            for ch in 0..si.num_channels.max(0) as usize {
                debug_assert!(ch < self.mpeg_h_rendered_delay.num_channels as usize);
                debug_assert!(nfs >= rdu);

                /* Copy delayed samples from delay buffer to time buffer */
                if rdu != 0 {
                    time_data2[nfs * ch..nfs * ch + rdu].copy_from_slice(
                        &self.mpeg_h_rendered_delay.delay_line[rdu * ch..rdu * ch + rdu],
                    );
                }

                /* Resample first part of the current time signal and store the result in the time
                 * buffer */
                td_upsampler(
                    sample_rate_converter_fac_upsampling,
                    &core2[fsz * ch..fsz * ch + sfs],
                    &mut time_data2[nfs * ch + rdu..],
                    &mut self.mpeg_h_sample_rate_converter_filter_states[ch],
                );

                /* Copy hang over samples from the time buffer to the delay buffer */
                if ovs != 0 {
                    self.mpeg_h_rendered_delay.delay_line[rdu * ch..rdu * ch + ovs]
                        .copy_from_slice(&time_data2[nfs * ch + nfs..nfs * ch + nfs + ovs]);
                }

                /* Resample second part of the current time signal and store the result in the
                 * delay buffer */
                td_upsampler(
                    sample_rate_converter_fac_upsampling,
                    &core2[fsz * ch + sfs..fsz * ch + fsz],
                    &mut self.mpeg_h_rendered_delay.delay_line[rdu * ch + ovs..],
                    &mut self.mpeg_h_sample_rate_converter_filter_states[ch],
                );
            }
        }

        /* change streamInfo parameters */
        si.frame_size = new_frame_size;
        si.sample_rate = new_sample_rate;

        true
    }

    /// The part of `aacDecoder_DecodeFrame()` after the rendering of an access unit: MPEG-H
    /// access unit size, truncation, cross-fade and earcon mixing. Returns the truncated frame
    /// size if the truncation was applied.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn render_tail(
        &mut self,
        ctx: &RenderCtx,
        si: &mut CStreamInfo,
        time_data2: &mut [PcmDec],
        flags: u32,
        access_unit: u32,
        num_preroll_au: u32,
        num_access_units: u32,
    ) -> Option<i32> {
        if (ctx.flush_status != 0 || flags & AACDEC_FLUSH != 0) && (flags & AACDEC_CONCEAL == 0) {
            si.mpegh_au_size = 0;
        } else if access_unit == num_access_units - 1 {
            si.mpegh_au_size = si.frame_size - self.truncate_sample_count as i32;
            self.truncate_sample_count = 0;
        }

        /* Truncation */
        if flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0 {
            Some(self.truncate(
                ctx,
                si,
                time_data2,
                access_unit,
                num_preroll_au,
                access_unit == num_access_units - 1,
            ))
        } else {
            /* Clear truncation queue */
            self.truncate_start_offset = [-128; TRUNC_QUEUE_SIZE];
            self.truncate_stop_offset = [-128; TRUNC_QUEUE_SIZE];

            if ctx.flags0 & AC_MPEGH3DA != 0 && access_unit == num_access_units - 1 {
                pcm_data_payload(
                    &mut self.earcon_decoder,
                    time_data2,
                    si.frame_size as u32,
                    ctx.drc_status_target_loudness,
                    ctx.default_target_loudness,
                    ctx.target_layout,
                    775,
                    (si.frame_size - 775) as i16,
                );
            }
            None
        }
    }

    /// The truncation and cross-fade handling of `aacDecoder_DecodeFrame()` (followed by the
    /// earcon mixing). Returns the truncated frame size.
    fn truncate(
        &mut self,
        ctx: &RenderCtx,
        si: &CStreamInfo,
        time_data2: &mut [PcmDec],
        access_unit: u32,
        num_preroll_au: u32,
        last_au: bool,
    ) -> i32 {
        let fsz = si.frame_size;
        let mut frame_size_remaining = fsz;
        let mut last_frame_samples: i32 = 775;
        let mut new_frame_samples = frame_size_remaining - last_frame_samples;

        /* extend/shift truncation range for flushed and preroll frames */
        for q in 0..TRUNC_QUEUE_SIZE {
            if ctx.flush_status != 0 && self.truncate_stop_offset[q] >= 775 {
                self.truncate_stop_offset[q] = (self.truncate_stop_offset[q] as i32 + fsz) as i16;
                /* If there is a trunc segment starting after position 0 (+ delay) it has to be
                shifted instead of extended because it is after the inserted flush/preroll */
                if self.truncate_start_offset[q] > 775 {
                    self.truncate_start_offset[q] =
                        (self.truncate_start_offset[q] as i32 + fsz) as i16;
                }
            }
            if access_unit < num_preroll_au && self.truncate_stop_offset[q] > 0 {
                self.truncate_stop_offset[q] = (self.truncate_stop_offset[q] as i32 + fsz) as i16;
                /* If there is a trunc segment starting after position 0 (+ delay) it has to be
                shifted instead of extended because it is after the inserted flush/preroll */
                if self.truncate_start_offset[q] > 775 {
                    self.truncate_start_offset[q] =
                        (self.truncate_start_offset[q] as i32 + fsz) as i16;
                }
            }
        }

        /* Go through truncation queue */
        for q in 0..TRUNC_QUEUE_SIZE {
            /* update offsets from previous iteration truncation segment */
            self.truncate_start_offset[q] = (-128)
                .max(self.truncate_start_offset[q] as i32 - (fsz - frame_size_remaining))
                as i16;
            self.truncate_stop_offset[q] = (-128)
                .max(self.truncate_stop_offset[q] as i32 - (fsz - frame_size_remaining))
                as i16;

            /* Skip inactive truncation segments. */
            if q > 0 && (self.truncate_start_offset[q] < 0 && self.truncate_stop_offset[q] < 0) {
                continue;
            }

            let tso = self.truncate_start_offset[q] as i32;
            let tsto = self.truncate_stop_offset[q] as i32;

            /* current frame start and stop sample index of truncated part */
            let trunc_start = 0.max(frame_size_remaining.min(tso));
            let trunc_stop = 0.max(frame_size_remaining.min(tsto));

            /* length of truncated and remaining part */
            let trunc_length = trunc_stop - trunc_start;
            let truncate_frame_size_current = frame_size_remaining - trunc_length;

            /* number of new frame and last frame samples for Earcon */
            new_frame_samples -=
                last_frame_samples.max(trunc_stop) - last_frame_samples.max(trunc_start);
            last_frame_samples -=
                last_frame_samples.min(trunc_stop) - last_frame_samples.min(trunc_start);

            /* save samples for crossfade */
            if (tso + 128 > 0) && (tso < frame_size_remaining) {
                let l = frame_size_remaining.min(tso + 128) - trunc_start;

                for ch in 0..si.num_channels.max(0) {
                    let dst = (128 * ch + trunc_start - tso) as usize;
                    let src = (fsz * ch + trunc_start) as usize;
                    self.crossfade_mem[dst..dst + l as usize]
                        .copy_from_slice(&time_data2[src..src + l as usize]);
                }
            }

            /* apply crossfade */
            if self.apply_crossfade != AACDEC_CROSSFADE_BITMASK_OFF
                && (tsto + 128 > 0)
                && (tsto < frame_size_remaining)
            {
                {
                    let step: FixpSgl = fl2fxconst_sgl(1.0 / 127.0);

                    let stop = frame_size_remaining.min(tsto + 128);
                    let mut alpha: FixpSgl = (((trunc_stop - tsto).wrapping_mul(step as i32))
                        as FixpSgl as i32
                        - step as i32) as FixpSgl;
                    for i in trunc_stop..stop {
                        /* increment alpha before the loop to avoid FIXP_SGL overflow */
                        alpha = (alpha as i32 + step as i32) as FixpSgl;
                        for ch in 0..si.num_channels.max(0) {
                            let idx = (fsz * ch + i) as usize;
                            let tmp_in = time_data2[idx];
                            let tmp_out = self.crossfade_mem[(128 * ch + i - tsto) as usize];
                            time_data2[idx] = f_mult(alpha, tmp_in).wrapping_add(f_mult(
                                (fl2fxconst_sgl(1.0) as i32 - alpha as i32) as FixpSgl,
                                tmp_out,
                            ));
                        }
                    }
                }

                if tsto + 128 <= frame_size_remaining {
                    /* disable cross-fade between frames at nect config change */
                    self.apply_crossfade = AACDEC_CROSSFADE_BITMASK_OFF;
                }
            }

            /* apply truncation */
            /* This version keeps the distance (self->streamInfo.frameSize) of the channel data.
            There are 2 exclusive cases:
            In case of truncStart !=0, we truncate all data [truncStart ...frameSize-1] => no copy
            required.
            In case of truncStop !=0, we truncate all data [0 ... truncStop-1] => copy
            [truncStop ... frameSize-1] to [0 ...truncateFrameSizeCurrent-1] */
            if (trunc_length > 0) && (truncate_frame_size_current > 0) {
                for ch in 0..si.num_channels.max(0) {
                    if trunc_stop < frame_size_remaining {
                        /* Truncate first samples [0..truncStop-1]: move remaining to start at 0
                         * for each channel */
                        let base = (fsz * ch) as usize;
                        time_data2.copy_within(
                            base + trunc_stop as usize
                                ..base + (trunc_stop + truncate_frame_size_current) as usize,
                            base,
                        );
                    }
                }
            }

            /* update offsets for next iteration preroll/flushing/decoding */
            self.truncate_start_offset[q] =
                (-128).max(self.truncate_start_offset[q] as i32 - frame_size_remaining) as i16;
            self.truncate_stop_offset[q] =
                (-128).max(self.truncate_stop_offset[q] as i32 - frame_size_remaining) as i16;

            frame_size_remaining -= trunc_length;
        } /* for (int q=0...) */

        if ctx.flags0 & AC_MPEGH3DA != 0 && last_au {
            pcm_data_payload(
                &mut self.earcon_decoder,
                time_data2,
                si.frame_size as u32,
                ctx.drc_status_target_loudness,
                ctx.default_target_loudness,
                ctx.target_layout,
                last_frame_samples.max(0) as i16,
                new_frame_samples.max(0) as i16,
            );
        }

        frame_size_remaining
    }

    /// The output loudness information of `aacDecoder_DecodeFrame()`.
    pub(crate) fn output_loudness(&self, si: &mut CStreamInfo, default_target_loudness: i8) {
        if self.h_uni_drc_decoder.get_param(DRC_DEC_IS_ACTIVE) != 0 {
            /* return output loudness information for MPEG-D DRC */
            let output_loudness = self.h_uni_drc_decoder.get_param(DRC_DEC_OUTPUT_LOUDNESS);
            if output_loudness == DRC_DEC_LOUDNESS_NOT_PRESENT {
                /* no valid MPEG-D DRC loudness value contained */
                si.output_loudness = -1;
            } else if output_loudness > 0 {
                /* positive output loudness values (very unusual) are limited to 0 LKFS */
                si.output_loudness = 0;
            } else {
                /* negate and scale from e = 7 to e = (31-2) */
                si.output_loudness = -output_loudness >> 22;
            }
        } else {
            /* return output loudness information for MPEG-4 DRC */
            if si.drc_prog_ref_lev < 0 {
                /* no MPEG-4 DRC loudness metadata contained */
                si.output_loudness = -1;
            } else if default_target_loudness < 0 {
                /* loudness normalization is off */
                si.output_loudness = si.drc_prog_ref_lev as i32;
            } else {
                si.output_loudness = default_target_loudness as i32;
            }
        }
    }

    /// The output stage of `aacDecoder_DecodeFrame()` after the access units (for
    /// `extAot != AOT_AAC_SLS`): interleaving (into `workBufferCore2`, `core2`), truncation of
    /// the frame size, limiter (with `pTimeData2` as work buffer) and discarding of the decoder
    /// delay at the start, into `p_time_data`.
    ///
    /// Returns `AAC_DEC_OUTPUT_BUFFER_TOO_SMALL` (bail out) or `AAC_DEC_SET_PARAM_FAIL` (return
    /// immediately) on errors, `AAC_DEC_OK` otherwise.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn limiter_output(
        &mut self,
        si: &mut CStreamInfo,
        time_data2: &mut [PcmDec],
        core2: &mut [FixpDbl],
        p_time_data: &mut [IntPcm],
        flags: u32,
        flags0: u32,
        limiter_enable_curr: u8,
        truncate_frame_size: i32,
        access_unit: u32,
        num_preroll_au: u32,
    ) -> AacDecoderError {
        let time_data_size = p_time_data.len() as i32;
        let mut pcm_limiter_scale: i32 = 0;
        pcm_limiter_scale += PCM_OUT_HEADROOM;

        if flags & AACDEC_CLRHIST != 0 {
            /* Delete the delayed signal. */
            if let Some(l) = self.h_limiter.as_mut() {
                l.reset();
            }
        }

        /* Check whether time data buffer is large enough. */
        if time_data_size < (si.num_channels * si.frame_size) {
            return AAC_DEC_OUTPUT_BUFFER_TOO_SMALL;
        }

        /* use workBufferCore2 buffer for interleaving */
        let block_length = si.frame_size;

        /* applyLimiter requests for interleaved data and doesn't support in-place
         * processing */
        /* Interleave output buffer */
        fdk_interleave(
            time_data2,
            core2,
            si.num_channels as u32,
            block_length as u32,
            si.frame_size as u32,
        );

        if truncate_frame_size != -1 {
            si.frame_size = truncate_frame_size;
        }

        if limiter_enable_curr != 0 {
            let limiter = self.h_limiter.as_mut().expect("hLimiter");
            /* Set actual signal parameters */
            limiter.set_n_channels(si.num_channels as u32);
            if limiter.set_sample_rate(si.sample_rate as u32) != TDLIMIT_OK {
                return AAC_DEC_SET_PARAM_FAIL;
            }

            /* limiter work buffer: pTimeData2 (workBufferCore5) */
            limiter.apply(
                core2,
                p_time_data,
                time_data2,
                None,
                pcm_limiter_scale,
                si.frame_size as u32,
            );

            if flags0 & AC_MPEGH3DA != 0 {
                if access_unit >= num_preroll_au {
                    /* after preroll frame decoding no valid samples to discard exist */
                    /* discard samples of constant decoder delay and pcmLimiter delay at
                     * start */
                    if self.discard_samples_at_start_cnt == -1 {
                        /* no left truncation has happend before thus the constant decoder
                         * delay is not truncated yet */
                        self.discard_samples_at_start_cnt =
                            limiter.get_delay() as i32 + 3 * 256 + TD_UPSAMPLER_MAX_DELAY;
                    } else if self.discard_samples_at_start_cnt == -2 {
                        /* one left truncation happend before and truncated the constant
                         * decoder delay */
                        self.discard_samples_at_start_cnt = limiter.get_delay() as i32;
                    }

                    if self.discard_samples_at_start_cnt > 0 && si.num_channels > 0 {
                        if si.frame_size <= self.discard_samples_at_start_cnt {
                            self.discard_samples_at_start_cnt -= si.frame_size;
                            si.frame_size = 0;
                        } else {
                            let nch = si.num_channels as usize;
                            let d = self.discard_samples_at_start_cnt as usize;
                            let n = (si.frame_size as usize - d) * nch;
                            p_time_data.copy_within(d * nch..d * nch + n, 0);
                            si.frame_size -= self.discard_samples_at_start_cnt;
                            self.discard_samples_at_start_cnt = 0;
                        }
                    }

                    if si.frame_size > 0 {
                        si.output_delay =
                            si.output_delay.wrapping_add(self.discard_samples_at_start_cnt as u32);
                    }
                }
            } else {
                /* Announce the additional limiter output delay */
                si.output_delay = si.output_delay.wrapping_add(limiter.get_delay());
            }
        } else {
            let n = (si.frame_size * si.num_channels).max(0) as usize;
            scale_values_saturate_to(&mut p_time_data[..n], &core2[..n], pcm_limiter_scale);
        }
        AAC_DEC_OK
    }
}

/// A frame whose rendering was completed by the render thread (frame pipelining).
#[derive(StateDigest)]
pub struct FrameResult {
    /// The return value of `aacDecoder_DecodeFrame()`.
    pub err: AacDecoderError,
    /// The stream info after `aacDecoder_DecodeFrame()` (the fields describing the rendered
    /// frame: `sampleRate`, `frameSize`, `numChannels`, `mpeghAUSize`, `outputLoudness`, ...).
    pub stream_info: CStreamInfo,
    /// The interleaved PCM output (`pTimeData`, `frameSize * numChannels` valid samples).
    pub pcm: Vec<IntPcm>,
}

/// The values of a pipelined frame the core stage knows when it hands the frame to the render
/// thread.
#[derive(Clone, Copy, Debug)]
pub struct DeferredFrame {
    /// The return value of `aacDecoder_DecodeFrame()` (the rendering of a pipelined frame does
    /// not change it).
    pub err: AacDecoderError,
    /// The output sample rate of the frame (`streamInfo.sampleRate`).
    pub sample_rate: i32,
    /// The number of output channels of the frame (`streamInfo.numChannels`).
    pub num_channels: i32,
}

/// The result of [`AacDecoderInstance::aac_decoder_decode_frame_pipelined`].
#[derive(Clone, Copy, Debug)]
pub enum DecodeStatus {
    /// The access unit was decoded completely: the return value of `aacDecoder_DecodeFrame()`;
    /// the output is in the PCM buffer and the stream info.
    Done(AacDecoderError),
    /// The rendering of the access unit runs on the render thread; its output will be returned
    /// by [`AacDecoderInstance::take_completed`] (after the frames rendered before).
    Deferred(DeferredFrame),
}

/// The rendering of a frame on the render thread.
pub(crate) struct RenderJob {
    rs: Box<RenderState>,
    ctx: RenderCtx,
    si: CStreamInfo,
    pcm: Vec<IntPcm>,
    err: AacDecoderError,
    pool: Option<Arc<ThreadPool>>,
}

impl RenderJob {
    /// The render stage of a pipelined access unit (single access unit, no flags, no flushing,
    /// rendering that cannot fail; see `AacDecoderInstance::can_defer_render()`): the same steps
    /// as the sequential `aacDecoder_DecodeFrame()`.
    fn run(mut self) -> (Box<RenderState>, FrameResult) {
        let rs = &mut *self.rs;
        let si = &mut self.si;
        let mut time_data2 = core::mem::take(&mut rs.time_data2);
        let mut work = core::mem::take(&mut rs.work_buffer);
        let mut err = self.err;
        let rendered =
            rs.render(&self.ctx, si, &mut time_data2, &mut work, self.pool.as_deref(), &mut err);
        debug_assert!(rendered, "pipelined rendering failed");
        let truncate_frame_size =
            rs.render_tail(&self.ctx, si, &mut time_data2, 0, 0, 0, 1).unwrap_or(-1);
        si.sample_rate_before_rs = si.sample_rate;
        rs.output_loudness(si, self.ctx.default_target_loudness);
        si.num_channels_before_mix = si.num_channels;
        si.flags = self.ctx.flags0;
        let e = rs.limiter_output(
            si,
            &mut time_data2,
            &mut work,
            &mut self.pcm,
            0,
            self.ctx.flags0,
            self.ctx.limiter_enable_curr,
            truncate_frame_size,
            0,
            0,
        );
        debug_assert_eq!(e, AAC_DEC_OK, "pipelined output failed");
        rs.time_data2 = time_data2;
        rs.work_buffer = work;
        (self.rs, FrameResult { err, stream_info: self.si, pcm: self.pcm })
    }
}

/// A parse of the core stage that was done on a copy of the parser state while the render
/// stage was running; it is repeated on the render state before the next render stage
/// ([`AacDecoderInstance::join_render`]).
#[derive(StateDigest)]
pub(crate) enum ParseRecord {
    /// `FDK_drcDec_ReadUniDrcGain()`
    DrcGain { payload: Vec<u8>, bits: u32, sub_stream_index: i32, consumed: i32, err: DrcDecError },
    /// `objectMetadataFrame()` of the renderer `hgVBAPRenderer[idx]`
    Oam {
        idx: usize,
        payload: Vec<u8>,
        bits: u32,
        payload_length: i32,
        low_delay_metadata_coding: u8,
        num_objects: u32,
        has_dynamic_object_priority: u8,
        has_uniform_spread: u8,
        consumed: i32,
    },
}

/// Copies `bits` bits of `bs` (from its current position, without moving it) into a buffer
/// whose size is a power of two.
pub(crate) fn copy_bits(bs: &FdkBitStream, bits: u32) -> Vec<u8> {
    let mut b = bs.clone();
    let n = (bits as usize).div_ceil(8);
    let mut out = vec![0u8; n.next_power_of_two().max(4)];
    let mut left = bits;
    for o in out.iter_mut() {
        if left == 0 {
            break;
        }
        let k = left.min(8);
        *o = (b.read_bits(k) << (8 - k)) as u8;
        left -= k;
    }
    out
}

/// Runs `f` on a bitstream reading the first `bits` bits of `payload`; returns its result and
/// the number of bits it consumed.
fn with_reader<R>(payload: &[u8], bits: u32, f: impl FnOnce(&mut FdkBitStream) -> R) -> (R, i32) {
    let mut bs = FdkBitStream::reader(payload, bits);
    let v0 = bs.get_valid_bits();
    let r = f(&mut bs);
    let used = v0 - bs.get_valid_bits();
    (r, used)
}

/// The parser state of `objectMetadataFrame()` of a renderer (a renderer holding only the
/// fields the parser reads and writes).
pub(crate) fn oam_parser_copy(r: &GVbapRenderer) -> Box<GVbapRenderer> {
    let mut c = Box::<GVbapRenderer>::default();
    c.num_objects = r.num_objects;
    c.frame_length = r.frame_length;
    c.oam_frame_length = r.oam_frame_length;
    c.num_oam_frames = r.num_oam_frames;
    c.render_mode = r.render_mode;
    c.oam_samples = r.oam_samples.clone();
    c.oam_samples_valid = r.oam_samples_valid.clone();
    c.oam_parsed_data = r.oam_parsed_data.clone();
    c.fixed_val = r.fixed_val;
    c
}

/// Whether `c` is a parser copy ([`oam_parser_copy`]) of a renderer with the configuration of
/// `r`.
fn oam_parser_matches(c: &GVbapRenderer, r: &GVbapRenderer) -> bool {
    c.num_objects == r.num_objects
        && c.frame_length == r.frame_length
        && c.oam_frame_length == r.oam_frame_length
        && c.num_oam_frames == r.num_oam_frames
        && c.render_mode == r.render_mode
        && c.oam_samples.len() == r.oam_samples.len()
        && c.oam_samples_valid.len() == r.oam_samples_valid.len()
        && c.oam_parsed_data.len() == r.oam_parsed_data.len()
}

impl AacDecoderInstance {
    /// Sets the render thread of the frame pipelining (not in C; `None`: the rendering runs
    /// sequentially). Waits for a running render stage.
    pub fn set_render_worker(&mut self, worker: Option<Arc<RenderWorker>>) {
        self.join_render();
        self.render_worker = worker;
    }

    /// Waits for the render stage running on the render thread (if any) and takes the render
    /// state back: the rendered frame is queued for [`AacDecoderInstance::take_completed`], the
    /// parses the core stage did on copies of the parser state since then are repeated on the
    /// render state.
    pub fn join_render(&mut self) {
        if let Some(p) = self.pending_render.take() {
            self.finish_render(p);
        }
    }

    /// Like [`AacDecoderInstance::join_render`], but only if the render stage has already
    /// finished. Returns `true` if no render stage is running afterwards.
    pub fn poll_render(&mut self) -> bool {
        if let Some(p) = &self.pending_render {
            if !p.is_done() {
                return false;
            }
            let p = self.pending_render.take().unwrap();
            self.finish_render(p);
        }
        true
    }

    /// Whether a render stage is running on the render thread.
    pub fn render_pending(&self) -> bool {
        self.pending_render.is_some()
    }

    /// The next frame whose rendering on the render thread was completed (in decoding order).
    pub fn take_completed(&mut self) -> Option<FrameResult> {
        self.completed.pop_front()
    }

    /// Returns the PCM buffer of a [`FrameResult`] for reuse.
    pub fn recycle_pcm(&mut self, pcm: Vec<IntPcm>) {
        if self.pcm_buffers.len() < 4 {
            self.pcm_buffers.push(pcm);
        }
    }

    fn finish_render(&mut self, p: Pending<(Box<RenderState>, FrameResult)>) {
        let (rs, result) = p.wait();
        self.render = Some(rs);
        self.completed.push_back(result);
        self.replay_parses();
    }

    /// Repeats the parses of the core stage on the render state (see [`ParseRecord`]).
    fn replay_parses(&mut self) {
        if self.parse_log.is_empty() {
            return;
        }
        let rs = self.render.as_deref_mut().expect("render state");
        for rec in self.parse_log.drain(..) {
            let ok = match rec {
                ParseRecord::DrcGain { payload, bits, sub_stream_index, consumed, err } => {
                    let drc = &mut rs.h_uni_drc_decoder;
                    let (e, used) = with_reader(&payload, bits, |bs| {
                        drc.read_uni_drc_gain(bs, sub_stream_index)
                    });
                    e == err && used == consumed
                }
                ParseRecord::Oam {
                    idx,
                    payload,
                    bits,
                    payload_length,
                    low_delay_metadata_coding,
                    num_objects,
                    has_dynamic_object_priority,
                    has_uniform_spread,
                    consumed,
                } => match rs.hg_vbap_renderer[idx].as_deref_mut() {
                    Some(r) => {
                        let (e, used) = with_reader(&payload, bits, |bs| {
                            object_metadata_frame(
                                r,
                                bs,
                                payload_length,
                                low_delay_metadata_coding,
                                num_objects,
                                has_dynamic_object_priority,
                                has_uniform_spread,
                            )
                        });
                        e == 0 && used == consumed
                    }
                    None => false,
                },
            };
            if !ok {
                self.pipeline_mismatches += 1;
            }
            debug_assert!(ok, "frame pipelining: repeated parse differs");
        }
    }

    /// Updates the parser copies of the core stage from the render state (before the render
    /// stage is moved to the render thread).
    fn refresh_parser_copies(&mut self) {
        let rs = self.render.as_deref().expect("render state");
        let drc = &rs.h_uni_drc_decoder;
        if !self.drc_gain_parser.as_ref().is_some_and(|p| drc.gain_parser_matches(p)) {
            self.drc_gain_parser = Some(Box::new(drc.gain_parser()));
        }
        self.oam_parsers.resize_with(rs.hg_vbap_renderer.len(), || None);
        for (c, r) in self.oam_parsers.iter_mut().zip(&rs.hg_vbap_renderer) {
            match (c.as_deref_mut(), r.as_deref()) {
                (_, None) => *c = None,
                (Some(cc), Some(rr)) if oam_parser_matches(cc, rr) => {
                    cc.oam_parsed_data.copy_from_slice(&rr.oam_parsed_data);
                    cc.fixed_val = rr.fixed_val;
                }
                (_, Some(rr)) => *c = Some(oam_parser_copy(rr)),
            }
        }
    }

    /// `FDK_drcDec_ReadUniDrcGain()` of the core stage: on the render state if it is available,
    /// otherwise on the parser copy (recorded for [`AacDecoderInstance::replay_parses`]); falls
    /// back to waiting for the render stage if the parse fails. Returns the result of
    /// `FDK_drcDec_ReadUniDrcGain()`.
    pub(crate) fn read_uni_drc_gain(
        &mut self,
        bs: &mut FdkBitStream,
        sub_stream_index: i32,
        payload_length: i32,
    ) -> DrcDecError {
        if self.render.is_none() {
            if let Some(p) = self.drc_gain_parser.as_deref_mut() {
                let start = bs.clone();
                let v0 = bs.get_valid_bits();
                let err = p.read_uni_drc_gain(bs, sub_stream_index);
                let consumed = v0 - bs.get_valid_bits();
                if err == DRC_DEC_OK && consumed <= payload_length * 8 {
                    let bits = (payload_length.max(0) * 8) as u32;
                    self.parse_log.push(ParseRecord::DrcGain {
                        payload: copy_bits(&start, bits),
                        bits,
                        sub_stream_index,
                        consumed,
                        err,
                    });
                    return err;
                }
                /* unusual: parse again on the render state */
                *bs = start;
            }
        }
        rs!(self).h_uni_drc_decoder.read_uni_drc_gain(bs, sub_stream_index)
    }

    /// `objectMetadataFrame()` of the core stage for the renderer `hgVBAPRenderer[idx]` (like
    /// [`AacDecoderInstance::read_uni_drc_gain`]). Returns `None` if the renderer does not
    /// exist.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn object_metadata_frame(
        &mut self,
        idx: usize,
        bs: &mut FdkBitStream,
        payload_length: i32,
        low_delay_metadata_coding: u8,
        has_dynamic_object_priority: u8,
        has_uniform_spread: u8,
    ) -> Option<i32> {
        if self.render.is_none() {
            if let Some(Some(p)) = self.oam_parsers.get_mut(idx) {
                let start = bs.clone();
                let v0 = bs.get_valid_bits();
                let num_objects = p.num_objects as i32 as u32;
                let err = object_metadata_frame(
                    p,
                    bs,
                    payload_length,
                    low_delay_metadata_coding,
                    num_objects,
                    has_dynamic_object_priority,
                    has_uniform_spread,
                );
                let consumed = v0 - bs.get_valid_bits();
                if err == 0 && consumed <= payload_length * 8 {
                    let bits = (payload_length.max(0) * 8) as u32;
                    self.parse_log.push(ParseRecord::Oam {
                        idx,
                        payload: copy_bits(&start, bits),
                        bits,
                        payload_length,
                        low_delay_metadata_coding,
                        num_objects,
                        has_dynamic_object_priority,
                        has_uniform_spread,
                        consumed,
                    });
                    return Some(err);
                }
                /* unusual: parse again on the render state */
                *bs = start;
            } else if self.oam_parsers.get(idx).is_some_and(|p| p.is_none()) {
                /* no renderer (the parser copies mirror the renderers) */
                return None;
            }
        }
        let r = rs!(self).hg_vbap_renderer[idx].as_deref_mut()?;
        let num_objects = r.num_objects as i32 as u32;
        Some(object_metadata_frame(
            r,
            bs,
            payload_length,
            low_delay_metadata_coding,
            num_objects,
            has_dynamic_object_priority,
            has_uniform_spread,
        ))
    }

    /// The number of output channels, frame size and sample rate the rendering of the current
    /// access unit produces (before truncation), see [`RenderState::render`].
    pub(crate) fn predict_render(&self, ctx: &RenderCtx) -> (i32, i32, i32) {
        let rs = self.render.as_deref().expect("render state");
        let num_channels = cicp2geometry_get_num_channels_from_cicp_noerr(ctx.target_layout_config);
        let rd = rs.mpeg_h_rendered_delay.delay as i32;
        let si = &self.stream_info;
        let (frame_size, sample_rate, ..) =
            upsampling_params(si.aac_sample_rate, si.frame_size, si.sample_rate, rd);
        (num_channels, frame_size, sample_rate)
    }

    /// Whether the rendering of the current access unit may run on the render thread (frame
    /// pipelining). Called after the control part of `applyUserInteractivity()`; the rendering
    /// is deferred only if
    ///
    /// - a render thread is set and the buffer poisoning is off,
    /// - the access unit is decoded without flags (concealment, flushing, interruption, clearing
    ///   of the history), is no pre-roll access unit and no pre-roll access units precede it,
    ///   and the decoder is neither flushing nor building up after a configuration change (the
    ///   rendering of these access units interacts with the core decoding; the caller also checks
    ///   the state at the start of the call, since it may end during the core decoding),
    /// - the rendering cannot fail (the error handling of a failed rendering changes the core
    ///   decoding of the following calls): consistent signal groups, a format converter in a
    ///   supported mode for channel signal groups, an output buffer large enough, a sample rate
    ///   supported by the limiter,
    /// - the IGF of the configuration cannot read stale content of `pTimeData2`
    ///   (`AacDecoderInstance::igf_reads_stale_data()`).
    pub(crate) fn can_defer_render(
        &mut self,
        ctx: &RenderCtx,
        flags: u32,
        access_unit: u32,
        num_access_units: u32,
        time_data_size: i32,
    ) -> bool {
        if self.render_worker.is_none() || self.poison != 0 {
            return false;
        }
        if flags != 0 || access_unit != 0 || num_access_units != 1 {
            return false;
        }
        if self.flush_status != AACDEC_FLUSH_OFF || self.build_up_status != AACDEC_BUILD_UP_OFF {
            return false;
        }
        if self.stream_info.ext_aot == AOT_AAC_SLS || ctx.drc_channels_mismatch() {
            return false;
        }
        let (num_channels, frame_size, sample_rate) = self.predict_render(ctx);
        let rs = self.render.as_deref().expect("render state");
        if ctx.has_active_channel_group() {
            let Some(fc) = rs.p_format_converter.as_deref() else { return false };
            let p = &*fc.member;
            if p.num_total_input_channels != 0
                && p.fc_params.is_some()
                && p.mode != IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
                && p.mode != IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN
            {
                return false;
            }
        }
        if time_data_size < num_channels * frame_size {
            return false;
        }
        if ctx.limiter_enable_curr != 0
            && !rs.h_limiter.as_ref().is_some_and(|l| l.sample_rate_supported(sample_rate as u32))
        {
            return false;
        }
        !self.igf_reads_stale_data()
    }

    /// Whether the IGF of the current configuration may read content of its `virtualSpec`
    /// (`pTimeData2[1024 * ch..]`) that an earlier access unit left there (e.g. the time signal
    /// of the previous rendering): `iisIGFDecoderFillMono()` / `iisIGFDecoderFillStereo()` and
    /// `CIgf_TNF_apply()` read `virtualSpec` in the IGF range of the grid, which
    /// `iisIGFDecoderApplyGainsMonoNew()` writes before only where the tiles of the grid lie.
    /// Returns `true` if the tiles of an IGF grid of a channel do not cover the IGF range of all
    /// channels with IGF (the stereo IGF uses the range of one channel for both).
    pub(crate) fn igf_reads_stale_data(&mut self) -> bool {
        if let Some(v) = self.igf_stale_reads {
            return v;
        }
        let stale = self.igf_stale_reads_uncached();
        self.igf_stale_reads = Some(stale);
        stale
    }

    /// [`AacDecoderInstance::igf_reads_stale_data`] without the cache.
    pub(crate) fn igf_stale_reads_uncached(&self) -> bool {
        let mut stale = false;
        for g in 0..mpeghdec_igf::igf_dec::IGF_MAX_GRIDS {
            let grids: Vec<&mpeghdec_igf::igf_dec::IgfGridInfo> = (0..28)
                .filter(|&ch| self.st_info_alloc[ch])
                .map(|ch| {
                    &self.p_aac_decoder_static_channel_info[ch].igf_static_data.s_grid_info_tab[g]
                })
                .filter(|gr| gr.i_igf_stop_line > gr.i_igf_start_line)
                .collect();
            let lo = grids.iter().map(|gr| gr.i_igf_start_line as i32).min().unwrap_or(0);
            let hi = grids.iter().map(|gr| gr.i_igf_stop_line as i32).max().unwrap_or(0);
            for gr in &grids {
                let mut covered = [false; 2048];
                for tile in &gr.s_igf_map_info_tab
                    [..(gr.i_igf_num_tile as usize).min(gr.s_igf_map_info_tab.len())]
                {
                    let mut tb = tile.i_des_start.max(0) as usize;
                    for &w in &tile.i_sfb_width_tab
                        [..(tile.i_sfb_cnt.max(0) as usize).min(tile.i_sfb_width_tab.len())]
                    {
                        let e = (tb + w.max(0) as usize).min(covered.len());
                        covered[tb.min(e)..e].fill(true);
                        tb = e;
                    }
                }
                if lo < 0 || !covered[lo as usize..hi.clamp(lo, 2048) as usize].iter().all(|&c| c) {
                    stale = true;
                }
            }
        }
        stale
    }

    /// Moves the render stage of the current access unit to the render thread: exchanges the
    /// time buffers, updates the parser copies and starts the job.
    pub(crate) fn launch_render(
        &mut self,
        ctx: RenderCtx,
        err: AacDecoderError,
        time_data_size: usize,
    ) {
        debug_assert!(self.pending_render.is_none());
        self.refresh_parser_copies();
        let mut rs = self.render.take().expect("render state");
        let ed = &rs.earcon_decoder;
        self.earcon_active = ed.earcon_config.earcon_flag != 0
            || ed.current_frame_has_earcon == 1
            || ed.last_frame_had_earcon == 1;
        if rs.time_data2.len() != self.p_time_data2.len() {
            rs.time_data2 = vec![0; self.p_time_data2.len()];
        }
        if rs.work_buffer.len() != self.work_buffer_core2.len() {
            rs.work_buffer = vec![0; self.work_buffer_core2.len()];
        }
        /* the render stage continues on the time signal of the core stage */
        core::mem::swap(&mut rs.time_data2, &mut self.p_time_data2);
        let mut pcm = self.pcm_buffers.pop().unwrap_or_default();
        pcm.resize(time_data_size, 0);
        let job =
            RenderJob { rs, ctx, si: self.stream_info, pcm, err, pool: self.thread_pool.clone() };
        let worker = self.render_worker.clone().expect("render worker");
        self.pending_render = Some(worker.run(move || job.run()));
    }
}

impl Drop for AacDecoderInstance {
    /// Waits for a rendering running on the render thread (its output is dropped).
    fn drop(&mut self) {
        if let Some(p) = self.pending_render.take() {
            /* a panic of the render stage was already reported by the render thread */
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| p.wait()));
        }
    }
}
