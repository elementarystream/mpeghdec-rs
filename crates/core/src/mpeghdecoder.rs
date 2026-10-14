// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from include/mpeghdecoder.h and libMpeghDec/src/mpeghdecoder.cpp.

//! MPEG-H decoder with timing handling (`mpeghdecoder_*()`).
//!
//! The decoder queues the presentation timestamps of the access units fed with
//! [`MpeghDecoderContext::process`] and the decoded PCM samples, adjusts the output to the timestamp
//! distances (inserting silence for gaps and dropping samples for overlaps, with fades) and
//! restarts the core decoder if two consecutive timestamps are more than 200 ms apart.
//!
//! Multi-core decoding (not in C): with more than one thread the decoder renders an access unit
//! on a render thread ([`crate::pipeline::RenderWorker`], see [`crate::render`])
//! while the next one is decoded, and a thread pool ([`mpeghdec_fdk::thread_pool::ThreadPool`])
//! distributes independent parts of a frame (per channel inverse transforms, format conversion)
//! over the remaining cores. With one thread neither is created. The output is identical for
//! every thread count.
//!
//! # Frame pipelining
//!
//! Because of the decoder delay, the output frame that [`MpeghDecoderContext::get_samples`] returns
//! after access unit N was fed contains samples rendered from access unit N, so rendering N
//! cannot overlap with decoding N+1 if every `get_samples()` call has to return what the
//! sequential decoder returns. The pipelined decoder returns the output frames one `process()`
//! call later instead: a `get_samples()` call that needs a frame still being rendered returns
//! `Ok(None)`, and the `get_samples()` calls the sequential decoder would have made at this point
//! (until `Ok(None)`) are executed on the decoder state of that point once the rendering has
//! finished, i.e. while the next access unit is decoded. The following `get_samples()` calls
//! return their output frames.
//!
//! The sequence of output frames (samples and [`MpeghDecoderOutputInfo`]) and the results of all
//! other calls are the same as with sequential decoding, provided that the caller calls
//! `get_samples()` until it returns `Ok(None)` after every `process()`, `flush()` and
//! `flush_and_get()` call. The rendering does not change the result of the decoding call, the
//! output sample rate or the number of channels, which the decoding thread knows beforehand; all
//! queue operations take place in the sequential order (operations that do not commute with the
//! pending ones wait for the rendering).

use mpeghdec_fdk::digest::StateDigest;
use mpeghdec_fdk::x87::F80;

use crate::aacdecoder::AacDecoderInstance;
use crate::aacdecoder_lib::*;
use crate::deque::*;
use crate::fast::FastOptions;
use crate::pipeline::RenderWorker;
use crate::render::{DecodeStatus, FrameResult};
use mpeghdec_fdk::audio::{TT_MHAS_PACKETIZED, TT_MHA_RAW};
use mpeghdec_fdk::cicp2geometry::cicp2geometry_get_num_channels_from_cicp_noerr;
use mpeghdec_fdk::digest::{digest_of, Digest, Digester};
use mpeghdec_fdk::thread_pool::ThreadPool;
use std::collections::VecDeque;
use std::sync::Arc;

/// `THRESHOLD`: the maximally allowed time difference (in nanoseconds) of two consecutively
/// provided MPEG-H frames. If this threshold is exceeded the decoding process will be restarted,
/// i.e. the decoding core will be reset to its initial start-up state (200 milliseconds ~ 9.375
/// frames of 1024 samples at 48kHz).
const THRESHOLD: u64 = 200000000;

const TIMESTAMP_ARRAY_SIZE: u32 = 10;
const FADE_ARRAY_SIZE: u32 = 10;
const NUM_FADE_SAMPLES_PER_CHANNEL: i32 = 128;

/// `MAX_NUM_FRAME_SAMPLES`: maximum number of samples per channel of an output frame.
pub const MAX_NUM_FRAME_SAMPLES: usize = 3072;
/// `MAX_NUM_OUTPUT_CHANNELS`
pub const MAX_NUM_OUTPUT_CHANNELS: usize = 24;

const TOLERANCE: i32 = 5;

const DEFAULT_DRC_SETTING_TARGET_REFERENCE_LEVEL: i32 = 96;
const DEFAULT_DRC_SETTING_EFFECT_TYPE: i32 = 0;
const DEFAULT_DRC_SETTING_BOOST_FACTOR: i32 = 127;
const DEFAULT_DRC_SETTING_ATTENUATION_FACTOR: i32 = 127;
const DEFAULT_DRC_SETTING_ALBUM_MODE: i32 = 0;

/// `MPEGH_DECODER_ERROR`
pub type MpeghDecoderError = i32;
/// No error occurred.
pub const MPEGH_DEC_OK: MpeghDecoderError = 0;
/// No more output samples are available. Feed more input data.
pub const MPEGH_DEC_FEED_DATA: MpeghDecoderError = 1;
/// A NULL pointer was provided as a function parameter.
pub const MPEGH_DEC_NULLPTR_ERROR: MpeghDecoderError = 2;
/// Heap returned NULL pointer.
pub const MPEGH_DEC_OUT_OF_MEMORY: MpeghDecoderError = 3;
/// The current input data could not be filled because the buffer is full or the size of a
/// provided output buffer is too small.
pub const MPEGH_DEC_BUFFER_ERROR: MpeghDecoderError = 4;
/// The provided input data could not be processed.
pub const MPEGH_DEC_PROCESS_ERROR: MpeghDecoderError = 5;
/// An unsupported parameter was provided as a function parameter.
pub const MPEGH_DEC_UNSUPPORTED_PARAM: MpeghDecoderError = 6;
/// A restart of the decoder is necessary due to a change of the output configuration (sample
/// rate, number of channels).
pub const MPEGH_DEC_NEEDS_RESTART: MpeghDecoderError = 7;

/// Result of the methods of [`MpeghDecoderContext`] (not in C): `Err` holds an error code other
/// than `MPEGH_DEC_OK` and `MPEGH_DEC_FEED_DATA`.
pub type MpeghResult<T> = Result<T, MpeghDecoderError>;

/// `MPEGH_DECODER_PARAMETER`
pub type MpeghDecoderParameter = i32;
/// MPEG-D DRC target reference level / decoder
/// target loudness in steps of 0.25 LU below full scale, 40 (-10 LKFS) to 127 (-31.75 LKFS).
/// Default: 96 (-24 LKFS).
pub const MPEGH_DEC_PARAM_TARGET_REFERENCE_LEVEL: MpeghDecoderParameter = 0;
/// MPEG-D DRC effect type request: -1 DRC off, 0 none
/// (default), 1 late night, 2 noisy environment, 3 limited playback range, 4 low playback
/// level, 5 dialog enhancement, 6 general compression.
pub const MPEGH_DEC_PARAM_EFFECT_TYPE: MpeghDecoderParameter = 1;
/// scaling factor for boosting gains, 0..127 (default 127).
pub const MPEGH_DEC_PARAM_BOOST_FACTOR: MpeghDecoderParameter = 2;
/// scaling factor for attenuating gains, 0..127
/// (default 127).
pub const MPEGH_DEC_PARAM_ATTENUATION_FACTOR: MpeghDecoderParameter = 3;
/// 0 disabled (default), 1 enabled.
pub const MPEGH_DEC_PARAM_ALBUM_MODE: MpeghDecoderParameter = 4;

/// `MPEGH_DECODER_OUTPUT_INFO`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MpeghDecoderOutputInfo {
    /// The number of samples per channel of the decoded PCM audio signal.
    pub num_samples_per_channel: i32,
    /// The number of audio channels of the decoded PCM audio signal.
    pub num_channels: i32,
    /// The sample rate in Hz of the decoded PCM audio signal.
    pub sample_rate: i32,
    /// The presentation timestamp in ticks of the sample rate of the decoded PCM audio signal.
    pub ticks: u64,
    /// The presentation timestamp in nano seconds of the decoded PCM audio signal.
    pub pts: u64,
    /// Audio output loudness in steps of -0.25 LU. Range: 0 (0 LKFS) to 231 (-57.75 LKFS). -1
    /// indicates that no loudness metadata is present.
    pub loudness: i32,
    /// Flag to signal if the decoded PCM audio signal is concealed.
    pub is_concealed: bool,
}

/// `AUInfo`
#[derive(StateDigest, Clone, Copy, Debug, Default)]
struct AuInfo {
    /// samples per channel
    au_size: i32,
    concealed: bool,
    output_loudness: i32,
}

/// A frame whose rendering runs on the render thread (frame pipelining, not in C).
#[derive(StateDigest, Clone, Copy, Debug)]
struct PendingFrame {
    /// The `concealed` flag of the decoding loop for the frame.
    concealed: bool,
    /// The output is not added to the queues (the sequential decoding returned before).
    discard: bool,
    /// The output sample rate and number of channels the rendering will produce.
    sample_rate: i32,
    num_channels: i32,
}

/// `OutputInfo`
#[derive(StateDigest, Clone, Copy, Debug, Default)]
struct OutputInfo {
    /// samples * channel
    size: i32,
    concealed: bool,
    output_loudness: i32,
}

/// `MPEGH_DECODER_CONTEXT` (`HANDLE_MPEGH_DECODER_CONTEXT`): the MPEG-H decoder. Dropping it is
/// `mpeghdecoder_destroy()`.
///
/// Feed access units with [`process`](MpeghDecoderContext::process) and call
/// [`get_samples`](MpeghDecoderContext::get_samples) until it returns `Ok(None)` after every
/// `process()`, [`flush`](MpeghDecoderContext::flush) and [`flush_and_get`](MpeghDecoderContext::flush_and_get)
/// call.
///
/// # Threads
///
/// The decoder runs on the calling thread unless more threads are requested (not in C:
/// [`with_threads`](MpeghDecoderContext::with_threads),
/// [`set_num_threads`](MpeghDecoderContext::set_num_threads)). With more than one thread it renders an
/// access unit while the next one is decoded (frame pipelining) and spreads independent parts of
/// a frame over the other threads. The output frames are then returned one `process()` call
/// later; the sequence of output frames and the results of all other calls are the same as with
/// one thread, provided that `get_samples()` is called as described above.
#[derive(StateDigest)]
pub struct MpeghDecoderContext {
    sample_rate: i32,
    number_of_channels: i32,
    cicp_index: i32,
    zero_signal: bool,

    mpeghdec: Option<Box<AacDecoderInstance>>,
    tmp_samples: Vec<i32>,

    mha_config: Vec<u8>,

    timestamp_in_queue: Deque<u64>,
    timestamp_out_queue: Deque<u64>,

    au_info_queue: Deque<AuInfo>,

    decoded_samples_queue: Deque<i32>,
    output_info_queue: Deque<OutputInfo>,
    output_samples_queue: Deque<i32>,

    fadeout_idx_queue: Deque<i32>,
    fadein_idx_queue: Deque<i32>,

    frame_number: u64,

    max_decoder_output_samples: u32,

    drc_update: bool,
    /* Desired DRC values (set by user). */
    desired_drc_target: i32,
    desired_drc_effect_type: i32,
    desired_drc_boost_factor: i32,
    desired_drc_att_factor: i32,
    desired_drc_album_mode: i32,

    /* DRC values from last time. */
    last_drc_target: i32,
    last_drc_effect_type: i32,
    last_drc_boost_factor: i32,
    last_drc_att_factor: i32,
    last_drc_album_mode: i32,

    /* Multi-core decoding (not in C). The output does not depend on the number of threads, so
    the threads are not part of the state digest. */
    #[digest(skip)]
    num_threads: usize,
    #[digest(skip)]
    thread_pool: Option<Arc<ThreadPool>>,
    /// The render thread of the frame pipelining.
    #[digest(skip)]
    render_worker: Option<Arc<RenderWorker>>,
    /// The frames whose rendering runs (or ran) on the render thread and whose output has not
    /// been added to the queues yet, in decoding order.
    pending_frames: VecDeque<PendingFrame>,
    /// A loop of `get_samples()` calls of the sequential decoder that is executed when the
    /// rendering has finished: the number of timestamps it can use (the timestamps queued after
    /// the loop are not visible to it).
    owed_get_samples: Option<u32>,
    /// Output frames of executed [`MpeghDecoderContext::owed_get_samples`] loops, returned by the next
    /// `get_samples()` calls.
    ready_frames: VecDeque<(MpeghDecoderOutputInfo, Vec<i32>)>,
    /// Reusable sample buffers of `ready_frames` (overwritten before use: not part of the state
    /// digest).
    #[digest(skip)]
    ready_buffers: Vec<Vec<i32>>,
    /// The sample buffer of `get_samples()` (`tmpSamples` in C; separate from the one of the
    /// decoding, which may hold the output of a frame while the pending `get_samples()` calls
    /// are executed). Not part of the state digest: `get_samples()` reads only the samples it has
    /// just copied into it from `decodedSamplesQueue`.
    #[digest(skip)]
    get_scratch: Vec<i32>,

    /* Fast mode (not in C) */
    /// The fast mode options applied to every core decoder instance.
    fast_options: FastOptions,
    /// Bypassed frames of the core decoder instances closed before (only reported, not part of
    /// the state digest).
    #[digest(skip)]
    fast_bypass_frames: u64,
}

/// `fade(sample, index, fadelen, fadein)`: fade in/out a signal
fn fade(sample: &mut i32, index: i32, fadelen: i32, fadein: bool) {
    if mpeghdec_fdk::ARCH_X86_32 {
        *sample = fade_x87(*sample, index, fadelen, fadein);
        return;
    }
    let factor = fade_factor(index, fadelen, fadein, f64::cos);
    *sample = (*sample as f32 * factor) as i32;
}

/// The factor of [`fade`] with the cosine function `cos`.
///
/// The factor uses the cosine of the C library (`cos()` in double precision), whose results may
/// differ in the last bit between platforms. They do not change the single precision factors:
/// see `fade_tests`.
#[inline(always)]
fn fade_factor(index: i32, fadelen: i32, fadein: bool, cos: impl Fn(f64) -> f64) -> f32 {
    if fadein {
        (-cos(std::f64::consts::PI * index as f32 as f64 / fadelen as f32 as f64) + 1.0f32 as f64)
            as f32
            / 2.0f32
    } else {
        (cos(std::f64::consts::PI * (index + 1) as f32 as f64 / fadelen as f32 as f64) + 1.0) as f32
            / 2.0f32
    }
}

/// [`fade`] as computed by the 32 bit x86 build (x87, see `mpeghdec_fdk::x87`): the argument of
/// `cos()` in extended precision (rounded to `double` for the call), `cos() +- 1` in extended
/// precision rounded to `float`, and the product of the factor and the sample without rounding.
fn fade_x87(sample: i32, index: i32, fadelen: i32, fadein: bool) -> i32 {
    let k = if fadein { index } else { index + 1 };
    let arg = F80::from_i32(k)
        .fmul(F80::from_f64(std::f64::consts::PI))
        .fdiv(F80::from_i32(fadelen))
        .to_f64();
    let c = F80::from_f64(arg.cos());
    let t = if fadein { F80::ONE.fsub(c) } else { c.fadd(F80::ONE) }.to_f32();
    F80::from_f32(t).fmul(F80::from_f32(0.5)).fmul(F80::from_i32(sample)).to_i32_trunc()
}

/// `(uint64_t)(x * mul / div + 0.5)` as computed by the 32 bit x86 build (x87: the operations in
/// extended precision, the conversions of `uint64_t` operands rounded to `double`).
fn x87_scaled_u64(x: F80, mul: F80, div: F80) -> u64 {
    x.fmul(mul).fdiv(div).fadd(F80::from_f32(0.5)).to_u64_trunc()
}

impl MpeghDecoderContext {
    /// `mpeghdecoder_init(cicpSetup)`: creates a decoder rendering to the CICP target layout
    /// `cicp_setup` (1..=20 without 8, 100..=351, 400..=422). Returns `None` for unsupported
    /// layouts or if the decoder could not be created.
    ///
    /// The decoder runs on the calling thread (see [`MpeghDecoderContext::with_threads`]).
    pub fn new(cicp_setup: i32) -> Option<MpeghDecoderContext> {
        MpeghDecoderContext::with_threads(cicp_setup, 1)
    }

    /// Like [`MpeghDecoderContext::new`], decoding with `num_threads` threads (not in C; 0 or 1:
    /// no worker threads, everything runs on the calling thread). If the worker threads cannot
    /// be created, the decoder runs single-threaded.
    pub fn with_threads(cicp_setup: i32, num_threads: usize) -> Option<MpeghDecoderContext> {
        let num_out_channels = cicp2geometry_get_num_channels_from_cicp_noerr(cicp_setup);

        /* Check for allowed target layouts */
        if (cicp_setup <= 0)
            || (cicp_setup == 8)
            || ((cicp_setup > 20) && (cicp_setup < 100))
            || ((cicp_setup > 351) && (cicp_setup < 400))
            || (cicp_setup > 422)
        {
            return None;
        }

        let max_decoder_output_samples =
            num_out_channels.max(0) as u32 * MAX_NUM_FRAME_SAMPLES as u32;

        let mut ctx = MpeghDecoderContext {
            sample_rate: -1,
            number_of_channels: -1,
            cicp_index: cicp_setup,
            zero_signal: false,
            mpeghdec: None,
            tmp_samples: vec![0; max_decoder_output_samples as usize],
            mha_config: Vec::new(),
            timestamp_in_queue: Deque::default(),
            timestamp_out_queue: Deque::default(),
            au_info_queue: Deque::default(),
            decoded_samples_queue: Deque::default(),
            output_info_queue: Deque::default(),
            output_samples_queue: Deque::default(),
            fadeout_idx_queue: Deque::default(),
            fadein_idx_queue: Deque::default(),
            frame_number: 0,
            max_decoder_output_samples,
            drc_update: true,
            /* Desired DRC values (set by user). Initialized to default values to be applied at
             * first processing step. */
            desired_drc_target: DEFAULT_DRC_SETTING_TARGET_REFERENCE_LEVEL,
            desired_drc_effect_type: DEFAULT_DRC_SETTING_EFFECT_TYPE,
            desired_drc_boost_factor: DEFAULT_DRC_SETTING_BOOST_FACTOR,
            desired_drc_att_factor: DEFAULT_DRC_SETTING_ATTENUATION_FACTOR,
            desired_drc_album_mode: DEFAULT_DRC_SETTING_ALBUM_MODE,
            /* DRC values from last time. Initialized to values out of range so that above default
             * values are used */
            last_drc_target: -2, /* not -1, as it is a valid parameter value */
            last_drc_effect_type: -2, /* not -1, as it is a valid parameter value */
            last_drc_boost_factor: -1,
            last_drc_att_factor: -1,
            last_drc_album_mode: -1,
            num_threads: 1,
            thread_pool: None,
            render_worker: None,
            pending_frames: VecDeque::new(),
            owed_get_samples: None,
            ready_frames: VecDeque::new(),
            ready_buffers: Vec::new(),
            get_scratch: vec![0; max_decoder_output_samples as usize],
            fast_options: FastOptions::EXACT,
            fast_bypass_frames: 0,
        };
        let _ = ctx.set_num_threads(num_threads);

        let mut dec = AacDecoderInstance::aac_decoder_open(TT_MHAS_PACKETIZED, 1)?;

        // set device target speaker setup according to CICP index
        if dec.aac_decoder_set_param(AAC_TARGET_LAYOUT_CICP, ctx.cicp_index) != AAC_DEC_OK {
            return None;
        }
        dec.set_thread_pool(ctx.thread_pool.clone());
        dec.set_render_worker(ctx.render_worker.clone());
        ctx.mpeghdec = Some(dec);

        // context variables initialization
        let m = ctx.max_decoder_output_samples;
        if deque_alloc(&mut ctx.timestamp_in_queue, TIMESTAMP_ARRAY_SIZE) < 0
            || deque_alloc(&mut ctx.timestamp_out_queue, TIMESTAMP_ARRAY_SIZE) < 0
            || deque_alloc(&mut ctx.au_info_queue, TIMESTAMP_ARRAY_SIZE) < 0
            || deque_alloc(&mut ctx.decoded_samples_queue, TIMESTAMP_ARRAY_SIZE * m) < 0
            || deque_alloc(&mut ctx.output_info_queue, TIMESTAMP_ARRAY_SIZE) < 0
            || deque_alloc(&mut ctx.output_samples_queue, TIMESTAMP_ARRAY_SIZE * m) < 0
            || deque_alloc(&mut ctx.fadeout_idx_queue, FADE_ARRAY_SIZE) < 0
            || deque_alloc(&mut ctx.fadein_idx_queue, FADE_ARRAY_SIZE) < 0
        {
            return None;
        }

        Some(ctx)
    }

    /// `mpeghdecoder_setMhaConfig(hCtx, config, configSize)`: sets an out-of-band
    /// `mpegh3daConfig()` (from the `mhaC` box of an `mha1` track) and restarts the decoder,
    /// which then decodes raw MPEG-H access units instead of MHAS.
    pub fn set_mha_config(&mut self, config: &[u8]) -> MpeghResult<()> {
        if config.is_empty() {
            return Err(MPEGH_DEC_UNSUPPORTED_PARAM);
        }
        self.mha_config = config.to_vec();

        self.restart_decoder()
    }

    /// `mpeghdecoder_process(hCtx, inData, inLength, timestamp)`: decodes one MHAS frame
    /// (access unit) with the presentation timestamp `timestamp` in nanoseconds.
    pub fn process(&mut self, in_data: &[u8], timestamp: u64) -> MpeghResult<()> {
        if in_data.is_empty() {
            return Err(MPEGH_DEC_UNSUPPORTED_PARAM);
        }
        if deque_full(&self.timestamp_in_queue) {
            /* the pending get_samples() calls pop timestamps */
            self.settle(Some(true));
            if deque_full(&self.timestamp_in_queue) {
                return Err(MPEGH_DEC_BUFFER_ERROR);
            }
        }
        if self.decoded_space_insufficient() {
            return Err(MPEGH_DEC_BUFFER_ERROR);
        }

        let mut valid_bytes = in_data.len() as u32;
        let mut decoding_successful = false;
        let mut do_conceal = false;

        // update the DRC settings if necessary
        self.update_drc_settings();

        // store the presentation timestamp associated with this MHAS frame; two consecutive
        // timestamps have to differ!
        if deque_empty(&self.timestamp_in_queue)
            || *deque_back(&self.timestamp_in_queue).unwrap() != timestamp
        {
            deque_push_back(&mut self.timestamp_in_queue, &timestamp);
        }

        let n = deque_size(&self.timestamp_in_queue);
        if n > 1 {
            let b = *deque_at(&self.timestamp_in_queue, n - 1).unwrap();
            let a = *deque_at(&self.timestamp_in_queue, n - 2).unwrap();
            let duration = b.wrapping_sub(a);

            if duration > THRESHOLD {
                self.restart_decoder()?;

                // store the presentation timestamp associated with this MHAS frame; two
                // consecutive timestamps have to differ! restartDecoder also cleared all queues!
                deque_push_back(&mut self.timestamp_in_queue, &timestamp);
            }
        }

        while valid_bytes != 0 {
            let mut concealed = false;
            let mut is_done = false;

            let Some(dec) = self.mpeghdec.as_deref_mut() else {
                /* aacDecoder_Fill(NULL) fails (the decoder could not be reopened by a restart) */
                self.settle(Some(true));
                deque_pop_back(&mut self.timestamp_in_queue);
                return Err(MPEGH_DEC_PROCESS_ERROR);
            };

            // pass the MHAS frame to the MPEG-H decoder
            let mut err = dec.aac_decoder_fill(&[in_data], core::slice::from_mut(&mut valid_bytes));

            if err != AAC_DEC_OK {
                // remove timestamp as it was not possible to fill in current AU
                self.settle(Some(true));
                deque_pop_back(&mut self.timestamp_in_queue);
                return Err(MPEGH_DEC_PROCESS_ERROR);
            }

            while !is_done {
                // run FDK decoding process
                let mut flags = 0;
                if do_conceal {
                    // force conceal of the decoder
                    concealed = true;
                    flags |= AACDEC_CONCEAL;
                }
                let m = self.max_decoder_output_samples as usize;
                let dec = self.mpeghdec.as_deref_mut().expect("decoder");
                let status =
                    dec.aac_decoder_decode_frame_pipelined(&mut self.tmp_samples[..m], flags);
                do_conceal = false; // do not conceal anymore

                let p_si = *dec.aac_decoder_get_stream_info();
                /* frame pipelining: the output of the frames rendered before comes first (only
                 * the frames finished during the call, the pending frame of this call is not
                 * registered yet), then the get_samples() calls after the previous process() */
                self.collect_rendered(None);
                if self.owed_get_samples.is_some() {
                    if matches!(status, DecodeStatus::Deferred(_)) {
                        /* the previous frame was rendered before this one was deferred */
                        self.settle(None);
                        debug_assert!(self.owed_get_samples.is_none());
                    } else {
                        self.settle(Some(true));
                    }
                }
                /* the output sample rate and number of channels of the frame */
                let (sample_rate, num_channels, deferred) = match status {
                    DecodeStatus::Done(e) => {
                        err = e;
                        (p_si.sample_rate, p_si.num_channels, false)
                    }
                    DecodeStatus::Deferred(d) => {
                        err = d.err;
                        (d.sample_rate, d.num_channels, true)
                    }
                };
                match err {
                    AAC_DEC_OK => {
                        // get information for the decoded PCM
                        decoding_successful = true;

                        if self.sample_rate == -1 && self.number_of_channels == -1 {
                            self.sample_rate = sample_rate;
                            self.number_of_channels = num_channels;
                        } else if self.sample_rate != sample_rate
                            || self.number_of_channels != num_channels
                        {
                            if deferred {
                                /* the output of the frame is dropped */
                                self.pending_frames.push_back(PendingFrame {
                                    concealed,
                                    discard: true,
                                    sample_rate,
                                    num_channels,
                                });
                            }
                            return Err(MPEGH_DEC_NEEDS_RESTART);
                        }
                    }

                    AAC_DEC_NOT_ENOUGH_BITS | AAC_DEC_TRANSPORT_SYNC_ERROR => {
                        if decoding_successful {
                            // This just indicates the end of a frame and new data has to be fed
                            // into the decoder.
                            is_done = true;
                        } else if self.sample_rate == -1 || self.number_of_channels == -1 {
                            // The decoder has no valid output configuration. Remove the queued
                            // presentation timestamp until first frame decoded successfully and
                            // sampling rate and number of channels is known.
                            deque_pop_front(&mut self.timestamp_in_queue);
                            is_done = true;
                        } else if err == AAC_DEC_TRANSPORT_SYNC_ERROR {
                            // We ran into a sync error, but the decoder has a valid output
                            // configuration. Therefore it is possible to obtain a force concealed
                            // frame in next call to decodeFrame.
                            do_conceal = true;
                        } else {
                            // There are not enough bits, but the decoder has a valid output
                            // configuration. Therefore we can end the loop and new data has to be
                            // fed into the decoder.
                            is_done = true;
                        }
                    }

                    AAC_DEC_INTERMEDIATE_OK => {
                        err = AAC_DEC_OK;
                    }

                    _ => {
                        // unhandled error codes: we ran into a decoding error
                        if is_decode_error(err) {
                            is_done = true;
                            concealed = true;
                        } else {
                            decoding_successful = false;
                            deque_pop_front(&mut self.timestamp_in_queue);
                            is_done = true;
                        }
                    }
                } // end of switch(err)

                // if the output is valid, add the PCM samples to the sample queue
                if is_output_valid(err) && deferred {
                    /* added when the rendering has finished (collect_rendered()) */
                    self.pending_frames.push_back(PendingFrame {
                        concealed,
                        discard: false,
                        sample_rate,
                        num_channels,
                    });
                } else if is_output_valid(err) {
                    // add decoded PCM samples to the sample queue
                    if p_si.frame_size > 0 {
                        let n = (p_si.frame_size * p_si.num_channels).max(0) as usize;
                        deque_bulk_push_back(
                            &mut self.decoded_samples_queue,
                            &self.tmp_samples[..n],
                        );
                    }
                    // add the MPEG-H AU size info received from decoder
                    if p_si.mpegh_au_size > 0 {
                        let au_info = AuInfo {
                            au_size: p_si.mpegh_au_size,
                            concealed,
                            output_loudness: p_si.output_loudness,
                        };

                        deque_push_back(&mut self.au_info_queue, &au_info);
                    }
                }
            } // end of while(!isDone)
        } // end of while(validBytes != 0)
        Ok(())
    }

    /// `mpeghdecoder_processTimescale(hCtx, inData, inLength, timestamp, timescale)`: like
    /// [`MpeghDecoderContext::process`] with the timestamp in units of `1 / timescale` seconds.
    pub fn process_timescale(
        &mut self,
        in_data: &[u8],
        timestamp: u64,
        timescale: u32,
    ) -> MpeghResult<()> {
        let timestamp_ns = if mpeghdec_fdk::ARCH_X86_32 {
            x87_scaled_u64(
                F80::from_f64(timestamp as f64),
                F80::from_f32(1e9),
                F80::from_i64(timescale as i64),
            )
        } else {
            (timestamp as f64 * 1e9 / timescale as f64 + 0.5) as u64
        };
        self.process(in_data, timestamp_ns)
    }

    /// `mpeghdecoder_getSamples(hCtx, outData, outLength, outInfo)`: obtains the next decoded
    /// frame (interleaved 32 bit samples, 24 significant bits in the upper bits: `INT_PCM` with 8
    /// bits of headroom removed by the limiter scaling). `out_data` must hold at least
    /// `MAX_NUM_FRAME_SAMPLES` samples for each output channel.
    ///
    /// Returns `Ok(None)` (`MPEGH_DEC_FEED_DATA`) if no complete frame is available; feed more
    /// input data.
    ///
    /// With more than one thread, the output frames are returned one `process()` call later (see
    /// [`MpeghDecoderContext`]).
    pub fn get_samples(
        &mut self,
        out_data: &mut [i32],
    ) -> MpeghResult<Option<MpeghDecoderOutputInfo>> {
        if (out_data.len() as u32) < self.max_decoder_output_samples {
            return Err(MPEGH_DEC_BUFFER_ERROR);
        }
        /* frame pipelining */
        self.settle(Some(false));
        if let Some((info, samples)) = self.ready_frames.pop_front() {
            out_data[..samples.len()].copy_from_slice(&samples);
            self.ready_buffers.push(samples);
            return Ok(Some(info));
        }
        if self.owed_get_samples.is_some() {
            /* these calls are already pending */
            return Ok(None);
        }
        if !self.pending_frames.is_empty() {
            /* the output of a frame is still being rendered: the get_samples() calls of the
             * sequential decoder are executed when it has finished */
            self.owed_get_samples = Some(deque_size(&self.timestamp_in_queue));
            return Ok(None);
        }
        self.get_samples_seq(out_data, None)
    }

    /// `mpeghdecoder_getSamples()` on the current state; `ts_limit` (frame pipelining) is the
    /// number of queued timestamps that are visible (the calls are executed later).
    fn get_samples_seq(
        &mut self,
        out_data: &mut [i32],
        ts_limit: Option<&mut u32>,
    ) -> MpeghResult<Option<MpeghDecoderOutputInfo>> {
        let mut out_info = MpeghDecoderOutputInfo {
            num_channels: self.number_of_channels,
            sample_rate: self.sample_rate,
            ..Default::default()
        };

        let ts_size = match &ts_limit {
            Some(l) => deque_size(&self.timestamp_in_queue).min(**l),
            None => deque_size(&self.timestamp_in_queue),
        };
        if ts_size < 2 {
            return Ok(None);
        }
        // calculate the duration if enough timestamps are available
        let b = *deque_at(&self.timestamp_in_queue, 1).unwrap();
        let a = *deque_at(&self.timestamp_in_queue, 0).unwrap();
        let duration = b.wrapping_sub(a);
        let duration_samples = if mpeghdec_fdk::ARCH_X86_32 {
            F80::from_f64(duration as f64)
                .fmul(F80::from_i32(self.sample_rate))
                .fdiv(F80::from_f32(1e9))
                .fadd(F80::from_f32(0.5))
                .to_i32_trunc()
        } else if mpeghdec_fdk::ARCH_X86 {
            /* cvttsd2si: INT_MIN for durations out of range (timestamp jumps); the Rust
             * conversion saturates like the AArch64 build */
            mpeghdec_fdk::fixpoint_math::x86_f64_to_i32(
                (duration as f64 * self.sample_rate as f64 / 1e9) + 0.5,
            )
        } else {
            ((duration as f64 * self.sample_rate as f64 / 1e9) + 0.5) as i32
        };

        let fadelen = NUM_FADE_SAMPLES_PER_CHANNEL * self.number_of_channels;

        // check whether a sample queue contains a complete PCM block to be sent to for
        // playback/postprocessing
        if !deque_empty(&self.au_info_queue) {
            let mut num_decoded_samples =
                deque_front(&self.au_info_queue).unwrap().au_size * self.number_of_channels;

            if num_decoded_samples > 0
                && deque_size(&self.decoded_samples_queue) >= num_decoded_samples as u32
            {
                // enough samples available
                let pts = *deque_pop_front(&mut self.timestamp_in_queue).unwrap();
                if let Some(l) = ts_limit {
                    *l -= 1;
                }
                let info = *deque_pop_front(&mut self.au_info_queue).unwrap();
                let au_size = info.au_size;
                let concealed = info.concealed;
                let output_loudness_tmp = info.output_loudness;

                deque_push_back(&mut self.timestamp_out_queue, &pts);

                let mut num_samples_to_remove = 0;
                let mut num_samples_to_add = 0;
                if au_size <= duration_samples + TOLERANCE
                    && au_size >= duration_samples - TOLERANCE
                {
                    // do nothing
                } else if au_size > duration_samples + TOLERANCE {
                    // remove samples from queue
                    num_samples_to_remove = (au_size - duration_samples) * self.number_of_channels;
                } else {
                    // add samples to queue
                    num_samples_to_add = (duration_samples - au_size) * self.number_of_channels;
                }

                if self.zero_signal && !concealed {
                    // the previous samples were zero and now there is an unconcealed signal
                    // available -> fade in set fadein start index
                    let fade_in_length = deque_size(&self.output_samples_queue) as i32;
                    deque_push_back(&mut self.fadein_idx_queue, &fade_in_length);
                    self.zero_signal = false;
                }

                if !self.zero_signal {
                    // the previous samples were non zero
                    if num_samples_to_add > 0 {
                        // we need to add zero samples -> fade out; set fadeout start index
                        let idx = deque_size(&self.output_samples_queue) as i32
                            + num_decoded_samples
                            - fadelen;
                        self.zero_signal = true;
                        if idx >= 0 {
                            deque_push_back(&mut self.fadeout_idx_queue, &idx);
                        }
                    }
                    if num_samples_to_remove > 0 {
                        // we need to remove samples -> fade out; set fadeout start index
                        let idx = deque_size(&self.output_samples_queue) as i32 - 1
                            + num_decoded_samples
                            - num_samples_to_remove
                            - fadelen;
                        if idx >= 0 {
                            deque_push_back(&mut self.fadeout_idx_queue, &idx);
                        }
                    }
                }

                // copy from sample queue to outputSamplesQueue
                if num_decoded_samples > 0 {
                    let samples_to_copy = (num_decoded_samples - num_samples_to_remove) as u32;
                    if concealed && self.zero_signal {
                        deque_bulk_pop_front_copy(
                            &mut self.decoded_samples_queue,
                            &mut self.get_scratch[..num_decoded_samples as usize],
                        );
                        // the previous samples were zero and the current signal is concealed
                        // -> replace samples with zero samples
                        deque_bulk_push_back_zeros(&mut self.output_samples_queue, samples_to_copy);
                    } else if !deque_bulk_move(
                        &mut self.decoded_samples_queue,
                        &mut self.output_samples_queue,
                        num_decoded_samples as u32,
                        samples_to_copy,
                    ) {
                        // (not reached: the queue holds the samples of the frame) the pop fails
                        // and the C++ code pushes the previous content of the buffer
                        deque_bulk_push_back(
                            &mut self.output_samples_queue,
                            &self.get_scratch[..samples_to_copy as usize],
                        );
                    }
                }

                // adjust the number of decoded samples after removing samples
                num_decoded_samples -= num_samples_to_remove;

                if num_samples_to_add > 0 {
                    // add zero samples
                    deque_bulk_push_back_zeros(
                        &mut self.output_samples_queue,
                        num_samples_to_add as u32,
                    );
                    // adjust the number of decoded samples after adding zero samples
                    num_decoded_samples += num_samples_to_add;
                }

                // add number of samples to outputSampleSizeQueue
                let dec_info = OutputInfo {
                    size: num_decoded_samples,
                    concealed: concealed || (num_samples_to_add > 0) || (num_samples_to_remove > 0),
                    output_loudness: output_loudness_tmp,
                };
                deque_push_back(&mut self.output_info_queue, &dec_info);
            } // end of if (deque_size(&hCtx->decodedSamplesQueue) >= numDecodedSamples)
        } // end of if (!deque_empty(&hCtx->auInfoQueue))

        // check if enough samples to output are available and apply fadein/fadeout if necessary
        if deque_empty(&self.output_info_queue) {
            return Ok(None);
        }

        let front = *deque_front(&self.output_info_queue).unwrap();
        if front.size > 0 && front.size > deque_size(&self.output_samples_queue) as i32 {
            // not enough samples available to output a frame
            return Ok(None);
        }

        let mut out_num_samples = front.size;

        // apply fadeout
        let mut num_indexes_to_remove = 0;
        for _ in 0..deque_size(&self.fadeout_idx_queue) {
            let index = *deque_front(&self.fadeout_idx_queue).unwrap() as u32;
            if index.wrapping_add(fadelen as u32) < deque_size(&self.output_samples_queue) {
                for i in 0..fadelen {
                    if let Some(s) = deque_at_mut(&mut self.output_samples_queue, index + i as u32)
                    {
                        fade(s, i, fadelen, false);
                    }
                }
                num_indexes_to_remove += 1;
            }
        }
        // remove used indexes from fadeoutIdxQueue
        deque_bulk_pop_front(&mut self.fadeout_idx_queue, num_indexes_to_remove);

        // apply fadein
        num_indexes_to_remove = 0;
        for _ in 0..deque_size(&self.fadein_idx_queue) {
            let index = *deque_front(&self.fadein_idx_queue).unwrap() as u32;
            if index.wrapping_add(fadelen as u32) < deque_size(&self.output_samples_queue) {
                for i in 0..fadelen {
                    if let Some(s) = deque_at_mut(&mut self.output_samples_queue, index + i as u32)
                    {
                        fade(s, i, fadelen, true);
                    }
                }
                num_indexes_to_remove += 1;
            }
        }
        // remove used indexes from fadeinIdxQueue
        deque_bulk_pop_front(&mut self.fadein_idx_queue, num_indexes_to_remove);

        let pts;
        let info;
        let tmp_frame_size = out_num_samples / self.number_of_channels;
        // check if the samples fit into the output buffer
        if tmp_frame_size > MAX_NUM_FRAME_SAMPLES as i32 {
            // not all samples can be written to output buffer; split it to consecutive calls
            let new_frame_size = MAX_NUM_FRAME_SAMPLES as i32;
            out_num_samples = new_frame_size * self.number_of_channels;
            let f = deque_front_mut(&mut self.output_info_queue).unwrap();
            f.size -= out_num_samples;
            info = *f;
            let tmp_pts = deque_front_mut(&mut self.timestamp_out_queue).unwrap();
            pts = *tmp_pts;
            let step = if mpeghdec_fdk::ARCH_X86_32 {
                x87_scaled_u64(
                    F80::from_i32(new_frame_size),
                    F80::from_f32(1e9),
                    F80::from_i32(self.sample_rate),
                )
            } else {
                (new_frame_size as f64 * 1e9 / self.sample_rate as f64 + 0.5) as u64
            };
            *tmp_pts = tmp_pts.wrapping_add(step);
        } else {
            // all samples fit into output buffer
            pts = *deque_pop_front(&mut self.timestamp_out_queue).unwrap();
            info = *deque_pop_front(&mut self.output_info_queue).unwrap();
        }

        // copy samples to outData
        deque_bulk_pop_front_copy(
            &mut self.output_samples_queue,
            &mut out_data[..out_num_samples.max(0) as usize],
        );

        // adjust fadein indexes
        let size = deque_size(&self.fadein_idx_queue);
        for _ in 0..size {
            let mut idx = *deque_pop_front(&mut self.fadein_idx_queue).unwrap();
            idx -= out_num_samples;
            if idx >= 0 {
                deque_push_back(&mut self.fadein_idx_queue, &idx);
            }
        }

        // adjust fadeout indexes
        let size = deque_size(&self.fadeout_idx_queue);
        for _ in 0..size {
            let mut idx = *deque_pop_front(&mut self.fadeout_idx_queue).unwrap();
            idx -= out_num_samples;
            if idx >= 0 {
                deque_push_back(&mut self.fadeout_idx_queue, &idx);
            }
        }

        out_info.num_samples_per_channel = out_num_samples / out_info.num_channels;
        out_info.pts = pts;
        out_info.ticks = if mpeghdec_fdk::ARCH_X86_32 {
            x87_scaled_u64(
                F80::from_f64(pts as f64),
                F80::from_i32(out_info.sample_rate),
                F80::from_f32(1e9),
            )
        } else {
            (pts as f64 * out_info.sample_rate as f64 / 1e9 + 0.5) as u64
        };
        out_info.loudness = info.output_loudness;
        out_info.is_concealed = info.concealed;

        Ok(Some(out_info))
    }

    /// `mpeghdecoder_flushAndGet(hCtx)`: flushes the decoder at the end of the stream; the
    /// remaining frames are obtained with [`MpeghDecoderContext::get_samples`].
    pub fn flush_and_get(&mut self) -> MpeghResult<()> {
        /* frame pipelining: all frames are rendered (and the pending get_samples() calls
         * executed) before the flushing */
        self.settle(Some(true));
        if deque_full(&self.timestamp_in_queue) {
            return Err(MPEGH_DEC_BUFFER_ERROR);
        }
        if deque_space(&self.decoded_samples_queue) < self.max_decoder_output_samples {
            return Err(MPEGH_DEC_BUFFER_ERROR);
        }
        let Some(dec) = self.mpeghdec.as_deref_mut() else {
            /* aacDecoder_DecodeFrame(NULL) returns an invalid output */
            return Ok(());
        };
        // flush the decoder
        let m = self.max_decoder_output_samples as usize;
        let err = dec.aac_decoder_decode_frame(&mut self.tmp_samples[..m], AACDEC_FLUSH);
        let p_si = *dec.aac_decoder_get_stream_info();
        if is_output_valid(err) {
            if self.sample_rate == -1 && self.number_of_channels == -1 {
                self.sample_rate = p_si.sample_rate;
                self.number_of_channels = p_si.num_channels;
            } else if self.sample_rate != p_si.sample_rate
                || self.number_of_channels != p_si.num_channels
            {
                return Err(MPEGH_DEC_NEEDS_RESTART);
            }
            // add decoded PCM samples to the sample queue
            if p_si.frame_size > 0 {
                let n = (p_si.frame_size * p_si.num_channels).max(0) as usize;
                deque_bulk_push_back(&mut self.decoded_samples_queue, &self.tmp_samples[..n]);
            }
            // add the MPEG-H AU size info received from decoder
            if p_si.mpegh_au_size > 0 {
                let au_info = AuInfo {
                    au_size: p_si.mpegh_au_size,
                    concealed: false,
                    output_loudness: p_si.output_loudness,
                };
                deque_push_back(&mut self.au_info_queue, &au_info);
            }

            // add the ending pts
            if !deque_empty(&self.timestamp_in_queue) {
                let mut pts = *deque_back(&self.timestamp_in_queue).unwrap();
                let mut last_frame_size = p_si.frame_size;
                if let Some(au_info) = deque_back(&self.au_info_queue) {
                    last_frame_size = au_info.au_size;
                }
                let step = if mpeghdec_fdk::ARCH_X86_32 {
                    x87_scaled_u64(
                        F80::from_i32(last_frame_size as i32),
                        F80::from_f32(1e9),
                        F80::from_i32(p_si.sample_rate),
                    )
                } else {
                    (last_frame_size as f64 * 1e9 / p_si.sample_rate as f64 + 0.5) as u64
                };
                pts = pts.wrapping_add(step);
                deque_push_back(&mut self.timestamp_in_queue, &pts);
            }
        }
        Ok(())
    }

    /// `mpeghdecoder_flush(hCtx)`: resets the decoder (e.g. for seeking); all queued data is
    /// discarded.
    pub fn flush(&mut self) -> MpeghResult<()> {
        self.restart_decoder()
    }

    /// `mpeghdecoder_setParam(hCtx, param, value)`: sets a DRC / loudness parameter; it is
    /// applied with the next [`MpeghDecoderContext::process`] call.
    pub fn set_param(&mut self, param: MpeghDecoderParameter, value: i32) -> MpeghResult<()> {
        let ok = match param {
            MPEGH_DEC_PARAM_TARGET_REFERENCE_LEVEL => {
                let ok = (40..=127).contains(&value);
                if ok {
                    self.desired_drc_target = value;
                }
                ok
            }
            MPEGH_DEC_PARAM_EFFECT_TYPE => {
                let ok = (-1..=6).contains(&value);
                if ok {
                    self.desired_drc_effect_type = value;
                }
                ok
            }
            MPEGH_DEC_PARAM_BOOST_FACTOR => {
                let ok = (0..=127).contains(&value);
                if ok {
                    self.desired_drc_boost_factor = value;
                }
                ok
            }
            MPEGH_DEC_PARAM_ATTENUATION_FACTOR => {
                let ok = (0..=127).contains(&value);
                if ok {
                    self.desired_drc_att_factor = value;
                }
                ok
            }
            MPEGH_DEC_PARAM_ALBUM_MODE => {
                let ok = value == 0 || value == 1;
                if ok {
                    self.desired_drc_album_mode = value;
                }
                ok
            }
            _ => false,
        };

        if !ok {
            return Err(MPEGH_DEC_UNSUPPORTED_PARAM);
        }
        self.drc_update = true;
        Ok(())
    }

    /// Sets the number of threads used for decoding (not in C): 0 or 1 decodes on the calling
    /// thread only (no worker threads). The
    /// decoded output does not depend on the number of threads. Fails with
    /// `MPEGH_DEC_OUT_OF_MEMORY` if the worker threads cannot be created (the decoder
    /// then runs single-threaded).
    pub fn set_num_threads(&mut self, num_threads: usize) -> MpeghResult<()> {
        let n = num_threads.max(1);
        let mut result = Ok(());
        if n != self.num_threads {
            /* finish the frames rendered by the old render thread */
            self.settle(Some(true));
            /* the old workers are joined when the core decoder releases them */
            self.thread_pool = None;
            self.render_worker = None;
            self.num_threads = 1;
            if n > 1 {
                /* one thread decodes, one renders (frame pipelining), the others are workers
                 * of the thread pool used by both */
                match RenderWorker::new() {
                    Ok(worker) => {
                        self.render_worker = Some(Arc::new(worker));
                        self.num_threads = n;
                        if n > 2 {
                            match ThreadPool::new(n - 1) {
                                Ok(pool) => self.thread_pool = Some(Arc::new(pool)),
                                Err(_) => {
                                    self.num_threads = 2;
                                    result = Err(MPEGH_DEC_OUT_OF_MEMORY);
                                }
                            }
                        }
                    }
                    Err(_) => result = Err(MPEGH_DEC_OUT_OF_MEMORY),
                }
            }
            if let Some(dec) = self.mpeghdec.as_deref_mut() {
                dec.set_thread_pool(self.thread_pool.clone());
                dec.set_render_worker(self.render_worker.clone());
            }
        }
        result
    }

    /// The number of threads used for decoding (see [`MpeghDecoderContext::set_num_threads`]).
    pub fn num_threads(&self) -> usize {
        self.num_threads
    }

    /// Enables or disables the fast mode (not in C): `true` enables all options of
    /// [`FastOptions`] ([`FastOptions::ALL`]), `false` restores the bit-exact decoding
    /// ([`FastOptions::EXACT`], the default). See [`MpeghDecoderContext::set_fast_options`].
    pub fn set_fast_mode(&mut self, enable: bool) {
        self.set_fast_options(if enable { FastOptions::ALL } else { FastOptions::EXACT });
    }

    /// Sets the fast mode options (not in C). **With any option enabled, the output is not
    /// bit-exact with the reference decoder**; see [`FastOptions`] for the options and the
    /// measured deviation. The number of output samples, their timing and the output info are
    /// the same as in the bit-exact mode. The options apply to the access units passed to
    /// [`MpeghDecoderContext::process`] afterwards and can be changed at any time; the output for a
    /// given sequence of calls does not depend on the number of threads.
    pub fn set_fast_options(&mut self, options: FastOptions) {
        self.fast_options = options;
        if let Some(dec) = self.mpeghdec.as_deref_mut() {
            dec.set_fast_options(options);
        }
    }

    /// The fast mode options (see [`MpeghDecoderContext::set_fast_options`]).
    pub fn fast_options(&self) -> FastOptions {
        self.fast_options
    }

    /// Number of frames whose format conversion used the time domain bypass of the fast mode
    /// ([`FastOptions::format_converter_bypass`]) since the decoder was created (not in C;
    /// for diagnostics). Waits for a running rendering.
    pub fn fast_bypass_frames(&mut self) -> u64 {
        self.fast_bypass_frames
            + self.mpeghdec.as_deref_mut().map_or(0, |dec| dec.fast_bypass_frames())
    }

    /// The core decoder (for inspection).
    pub fn core(&self) -> Option<&AacDecoderInstance> {
        self.mpeghdec.as_deref()
    }

    /// Digest of the complete decoder state (not in C, see [`mpeghdec_fdk::digest`]): two
    /// decoders with the same digest return the same output for the same future calls.
    ///
    /// With frame pipelining (more than one thread) a running rendering is waited for first;
    /// the digest of such a decoder differs from the one of a single-threaded decoder at the same
    /// point, since the pipelined decoder holds the output frames longer.
    pub fn state_digest(&mut self) -> Digest {
        self.join_for_digest();
        digest_of(&*self)
    }

    /// Like [`MpeghDecoderContext::state_digest`], also returning the digests of all fields of the
    /// state up to `max_depth` levels (for [`mpeghdec_fdk::digest::diff_traces`]). The digest
    /// differs from the one of [`MpeghDecoderContext::state_digest`].
    pub fn state_trace(&mut self, max_depth: usize) -> (Digest, Vec<(String, Digest)>) {
        self.join_for_digest();
        let mut d = Digester::traced(max_depth);
        StateDigest::digest(&*self, &mut d);
        d.finish_traced()
    }

    fn join_for_digest(&mut self) {
        if let Some(dec) = self.mpeghdec.as_deref_mut() {
            dec.join_render();
        }
    }

    /// Sets the content of the input buffer of the transport decoder (not in C, see
    /// [`mpeghdec_tpdec::TransportDec::set_bs_buffer`]); used by the chunked decoding.
    pub fn set_input_buffer_content(&mut self, content: &[u8]) {
        if let Some(tp) = self.mpeghdec.as_deref_mut().and_then(|d| d.h_input.as_deref_mut()) {
            tp.set_bs_buffer(content);
        }
    }

    /// Frame pipelining (not in C): adds the output of the frames whose rendering has finished
    /// to the queues, in decoding order, like the sequential decoding loop of
    /// [`MpeghDecoderContext::process`] does. `wait`: `Some(true)` waits for a running rendering,
    /// `Some(false)` takes it if it has finished, `None` takes only the frames the core decoder
    /// has already collected.
    fn collect_rendered(&mut self, wait: Option<bool>) {
        let Some(dec) = self.mpeghdec.as_deref_mut() else { return };
        match wait {
            Some(true) => dec.join_render(),
            Some(false) => {
                dec.poll_render();
            }
            None => {}
        }
        while let Some(r) = dec.take_completed() {
            let f = self.pending_frames.pop_front().expect("pending frame");
            let FrameResult { stream_info: p_si, pcm, .. } = r;
            debug_assert_eq!(
                (p_si.sample_rate, p_si.num_channels),
                (f.sample_rate, f.num_channels)
            );
            if !f.discard {
                // add decoded PCM samples to the sample queue
                if p_si.frame_size > 0 {
                    let n = (p_si.frame_size * p_si.num_channels).max(0) as usize;
                    deque_bulk_push_back(&mut self.decoded_samples_queue, &pcm[..n]);
                }
                // add the MPEG-H AU size info received from decoder
                if p_si.mpegh_au_size > 0 {
                    let au_info = AuInfo {
                        au_size: p_si.mpegh_au_size,
                        concealed: f.concealed,
                        output_loudness: p_si.output_loudness,
                    };
                    deque_push_back(&mut self.au_info_queue, &au_info);
                }
            }
            dec.recycle_pcm(pcm);
        }
    }

    /// Frame pipelining: adds the output of the rendered frames to the queues and executes the
    /// pending `get_samples()` calls if possible (`wait`: see
    /// [`MpeghDecoderContext::collect_rendered`]).
    fn settle(&mut self, wait: Option<bool>) {
        self.collect_rendered(wait);
        if self.pending_frames.is_empty() {
            if let Some(mut limit) = self.owed_get_samples.take() {
                loop {
                    let mut buf = self.ready_buffers.pop().unwrap_or_default();
                    buf.resize(self.max_decoder_output_samples as usize, 0);
                    match self.get_samples_seq(&mut buf, Some(&mut limit)) {
                        Ok(Some(info)) => {
                            buf.truncate(
                                (info.num_samples_per_channel * info.num_channels) as usize,
                            );
                            self.ready_frames.push_back((info, buf));
                        }
                        _ => {
                            self.ready_buffers.push(buf);
                            break;
                        }
                    }
                }
            }
        }
    }

    /// The check `deque_space(decodedSamplesQueue) < maxDecoderOutputSamples` of `process()`
    /// with the output of the frames still being rendered and the pending `get_samples()` calls
    /// (frame pipelining).
    fn decoded_space_insufficient(&mut self) -> bool {
        let m = self.max_decoder_output_samples;
        let pending = self.pending_frames.len() as u32;
        /* every frame adds at most maxDecoderOutputSamples, get_samples() only removes */
        if deque_space(&self.decoded_samples_queue) >= m.saturating_mul(pending + 1) {
            return false;
        }
        self.settle(Some(true));
        deque_space(&self.decoded_samples_queue) < m
    }

    /// `clearQueues(hCtx)`: clears all queues
    fn clear_queues(&mut self) {
        deque_clear(&mut self.timestamp_in_queue);
        deque_clear(&mut self.au_info_queue);
        deque_clear(&mut self.timestamp_out_queue);
        deque_clear(&mut self.decoded_samples_queue);
        deque_clear(&mut self.output_samples_queue);
        deque_clear(&mut self.output_info_queue);
        deque_clear(&mut self.fadein_idx_queue);
        deque_clear(&mut self.fadeout_idx_queue);
    }

    /// `restartDecoder(hCtx)`: restarts the decoder
    fn restart_decoder(&mut self) -> MpeghResult<()> {
        /* frame pipelining: the pending get_samples() calls come before the restart (their
         * output frames stay ready) */
        self.settle(Some(true));
        if let Some(dec) = self.mpeghdec.as_deref_mut() {
            self.fast_bypass_frames += dec.fast_bypass_frames();
        }
        self.mpeghdec = None;
        self.pending_frames.clear();
        self.owed_get_samples = None;

        let has_config = !self.mha_config.is_empty();
        let Some(mut dec) = AacDecoderInstance::aac_decoder_open(
            if has_config { TT_MHA_RAW } else { TT_MHAS_PACKETIZED },
            1,
        ) else {
            return Err(MPEGH_DEC_OUT_OF_MEMORY);
        };
        dec.set_thread_pool(self.thread_pool.clone());
        dec.set_render_worker(self.render_worker.clone());
        dec.set_fast_options(self.fast_options);

        // set device target speaker setup according to CICP index
        let error_status = dec.aac_decoder_set_param(AAC_TARGET_LAYOUT_CICP, self.cicp_index);
        if error_status != AAC_DEC_OK {
            self.mpeghdec = Some(dec);
            return Err(MPEGH_DEC_UNSUPPORTED_PARAM);
        }

        // set an out-of-band config if it was provided
        if has_config {
            let error_status = dec.aac_decoder_config_raw(&[&self.mha_config]);
            if error_status != AAC_DEC_OK {
                self.mpeghdec = Some(dec);
                return Err(MPEGH_DEC_UNSUPPORTED_PARAM);
            }
        }
        self.mpeghdec = Some(dec);

        self.clear_queues();
        self.zero_signal = false;

        self.sample_rate = -1;
        self.number_of_channels = -1;

        // force an update to the last desired DRC values after restarting the decoder
        self.drc_update = true;
        if self.last_drc_target == self.desired_drc_target {
            self.last_drc_target = -2;
        }
        if self.last_drc_effect_type == self.desired_drc_effect_type {
            self.last_drc_effect_type = -2;
        }
        if self.last_drc_boost_factor == self.desired_drc_boost_factor {
            self.last_drc_boost_factor = -1;
        }
        if self.last_drc_att_factor == self.desired_drc_att_factor {
            self.last_drc_att_factor = -1;
        }
        if self.last_drc_album_mode == self.desired_drc_album_mode {
            self.last_drc_album_mode = -1;
        }

        Ok(())
    }

    /// `updateDrcSettings(hCtx)`
    fn update_drc_settings(&mut self) {
        if !self.drc_update {
            return;
        }
        let Some(dec) = self.mpeghdec.as_deref_mut() else {
            /* aacDecoder_SetParam(NULL) fails for all parameters */
            self.drc_update = false;
            return;
        };
        if self.desired_drc_target != self.last_drc_target
            && dec.aac_decoder_set_param(AAC_DRC_REFERENCE_LEVEL, self.desired_drc_target)
                == AAC_DEC_OK
        {
            self.last_drc_target = self.desired_drc_target;
        }

        if self.desired_drc_effect_type != self.last_drc_effect_type
            && dec.aac_decoder_set_param(AAC_UNIDRC_SET_EFFECT, self.desired_drc_effect_type)
                == AAC_DEC_OK
        {
            self.last_drc_effect_type = self.desired_drc_effect_type;
        }

        if self.desired_drc_boost_factor != self.last_drc_boost_factor
            && dec.aac_decoder_set_param(AAC_DRC_BOOST_FACTOR, self.desired_drc_boost_factor)
                == AAC_DEC_OK
        {
            self.last_drc_boost_factor = self.desired_drc_boost_factor;
        }

        if self.desired_drc_att_factor != self.last_drc_att_factor
            && dec.aac_decoder_set_param(AAC_DRC_ATTENUATION_FACTOR, self.desired_drc_att_factor)
                == AAC_DEC_OK
        {
            self.last_drc_att_factor = self.desired_drc_att_factor;
        }

        if self.desired_drc_album_mode != self.last_drc_album_mode
            && dec.aac_decoder_set_param(AAC_UNIDRC_ALBUM_MODE, self.desired_drc_album_mode)
                == AAC_DEC_OK
        {
            self.last_drc_album_mode = self.desired_drc_album_mode;
        }

        self.drc_update = false;
    }
}

// ---------------------------------------------------------------------------------------------
// The C API of `mpeghdecoder.h`
// ---------------------------------------------------------------------------------------------

fn code(r: MpeghResult<()>) -> MpeghDecoderError {
    match r {
        Ok(()) => MPEGH_DEC_OK,
        Err(e) => e,
    }
}

/// `HANDLE_MPEGH_DECODER_CONTEXT mpeghdecoder_init(int32_t cicpSetup)`: `None` is `NULL`.
pub fn mpeghdecoder_init(cicp_setup: i32) -> Option<Box<MpeghDecoderContext>> {
    MpeghDecoderContext::new(cicp_setup).map(Box::new)
}

/// `mpeghdecoder_setMhaConfig(hCtx, config, configSize)`: `configSize` is `config.len()`.
pub fn mpeghdecoder_set_mha_config(
    h_ctx: &mut MpeghDecoderContext,
    config: &[u8],
) -> MpeghDecoderError {
    code(h_ctx.set_mha_config(config))
}

/// `void mpeghdecoder_destroy(HANDLE_MPEGH_DECODER_CONTEXT hCtx)`
pub fn mpeghdecoder_destroy(h_ctx: Option<Box<MpeghDecoderContext>>) {
    drop(h_ctx);
}

/// `mpeghdecoder_process(hCtx, inData, inLength, timestamp)`: `inLength` is `in_data.len()`,
/// `timestamp` in nanoseconds.
pub fn mpeghdecoder_process(
    h_ctx: &mut MpeghDecoderContext,
    in_data: &[u8],
    timestamp: u64,
) -> MpeghDecoderError {
    code(h_ctx.process(in_data, timestamp))
}

/// `mpeghdecoder_processTimescale(hCtx, inData, inLength, timestamp, timescale)`
pub fn mpeghdecoder_process_timescale(
    h_ctx: &mut MpeghDecoderContext,
    in_data: &[u8],
    timestamp: u64,
    timescale: u32,
) -> MpeghDecoderError {
    code(h_ctx.process_timescale(in_data, timestamp, timescale))
}

/// `mpeghdecoder_getSamples(hCtx, outData, outLength, outInfo)`: `outLength` is
/// `out_data.len()`. Like C, `MPEGH_DEC_FEED_DATA` sets the number of channels and the sample
/// rate of `out_info`, clears the other fields and leaves `ticks` unchanged.
pub fn mpeghdecoder_get_samples(
    h_ctx: &mut MpeghDecoderContext,
    out_data: &mut [i32],
    out_info: &mut MpeghDecoderOutputInfo,
) -> MpeghDecoderError {
    match h_ctx.get_samples(out_data) {
        Ok(Some(info)) => {
            *out_info = info;
            MPEGH_DEC_OK
        }
        Ok(None) => {
            out_info.num_channels = h_ctx.number_of_channels;
            out_info.sample_rate = h_ctx.sample_rate;
            out_info.num_samples_per_channel = 0;
            out_info.pts = 0;
            out_info.loudness = 0;
            out_info.is_concealed = false;
            MPEGH_DEC_FEED_DATA
        }
        Err(e) => e,
    }
}

/// `mpeghdecoder_flushAndGet(hCtx)`
pub fn mpeghdecoder_flush_and_get(h_ctx: &mut MpeghDecoderContext) -> MpeghDecoderError {
    code(h_ctx.flush_and_get())
}

/// `mpeghdecoder_flush(hCtx)`
pub fn mpeghdecoder_flush(h_ctx: &mut MpeghDecoderContext) -> MpeghDecoderError {
    code(h_ctx.flush())
}

/// `mpeghdecoder_setParam(hCtx, param, value)`
pub fn mpeghdecoder_set_param(
    h_ctx: &mut MpeghDecoderContext,
    param: MpeghDecoderParameter,
    value: i32,
) -> MpeghDecoderError {
    code(h_ctx.set_param(param, value))
}

#[cfg(test)]
mod fade_tests {
    use super::*;

    /// The neighbouring double precision values of `x` (`x` is neither 0 nor infinite).
    fn neighbours(x: f64) -> [f64; 2] {
        [f64::from_bits(x.to_bits() - 1), f64::from_bits(x.to_bits() + 1)]
    }

    /// The fade factors are the same on every platform: C libraries return `cos()` with an
    /// error of less than one unit in the last place, and for all fade lengths of the decoder a
    /// result that differs by one unit does not change the single precision factor. The only
    /// exceptions are the exact results at the ends of the fades (`cos(0) = 1`,
    /// `cos(PI) = -1`), which the C library of the platform has to return exactly.
    #[test]
    fn fade_factors_do_not_depend_on_the_cos_rounding() {
        assert_eq!(fade_factor(0, NUM_FADE_SAMPLES_PER_CHANNEL, true, f64::cos), 0.0);
        assert_eq!(
            fade_factor(
                NUM_FADE_SAMPLES_PER_CHANNEL - 1,
                NUM_FADE_SAMPLES_PER_CHANNEL,
                false,
                f64::cos
            ),
            0.0
        );
        for channels in 1..=MAX_NUM_OUTPUT_CHANNELS as i32 {
            let fadelen = NUM_FADE_SAMPLES_PER_CHANNEL * channels;
            for index in 0..fadelen {
                for fadein in [true, false] {
                    if (fadein && index == 0) || (!fadein && index == fadelen - 1) {
                        continue;
                    }
                    let factor = fade_factor(index, fadelen, fadein, f64::cos);
                    for n in 0..2 {
                        let other = fade_factor(index, fadelen, fadein, |x| neighbours(x.cos())[n]);
                        assert_eq!(
                            factor, other,
                            "{channels} channels, index {index}, fadein {fadein}"
                        );
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod state_digest_tests {
    use super::*;

    /// MHAS data and its access units (byte offset, size, 90 kHz timestamp).
    type Sample = (Vec<u8>, Vec<(usize, usize, u64)>);

    /// The access units and timestamps (90 kHz) of a test stream (`None` without the test
    /// material, see `MPEGHDEC_TEST_STREAMS`).
    fn load_sample(name: &str) -> Option<Sample> {
        let dir = std::env::var_os("MPEGHDEC_TEST_STREAMS")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| {
                std::env::var_os("MPEGHDEC_WORK_DIR")
                    .map(std::path::PathBuf::from)
                    .unwrap_or_else(|| {
                        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../work")
                    })
                    .join("streams")
            });
        let data = std::fs::read(dir.join(format!("{name}.mhas"))).ok()?;
        let pts = std::fs::read_to_string(dir.join(format!("{name}.pts.txt"))).ok()?;
        let idx = pts
            .lines()
            .filter_map(|l| {
                let v: Vec<u64> = l.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                (v.len() >= 3).then(|| (v[0] as usize, v[1] as usize, v[2]))
            })
            .collect();
        Some((data, idx))
    }

    fn feed(dec: &mut MpeghDecoderContext, data: &[u8], idx: &[(usize, usize, u64)]) {
        let mut out = vec![0i32; MAX_NUM_OUTPUT_CHANNELS * MAX_NUM_FRAME_SAMPLES];
        for &(o, n, p) in idx {
            dec.process_timescale(&data[o..o + n], p, 90000).unwrap();
            while dec.get_samples(&mut out).unwrap().is_some() {}
        }
    }

    /// Applies `toggle` (an involution) to the decoder and checks whether the digest changes,
    /// then applies it again and checks that the digest is restored.
    fn check(
        dec: &mut MpeghDecoderContext,
        what: &str,
        changes: bool,
        toggle: &dyn Fn(&mut MpeghDecoderContext),
    ) {
        let d0 = dec.state_digest();
        toggle(dec);
        let d1 = dec.state_digest();
        toggle(dec);
        assert_eq!(dec.state_digest(), d0, "{what}: not restored");
        if changes {
            assert_ne!(d1, d0, "{what}: the digest does not cover it");
        } else {
            assert_eq!(d1, d0, "{what}: the digest covers excluded state");
        }
    }

    fn core(d: &mut MpeghDecoderContext) -> &mut AacDecoderInstance {
        d.mpeghdec.as_deref_mut().unwrap()
    }

    fn render(d: &mut MpeghDecoderContext) -> &mut crate::render::RenderState {
        core(d).render.as_deref_mut().unwrap()
    }

    /// Two decoders fed with the same data have the same digest; changing any component of the
    /// state changes it; excluded scratch memory and the position of the elements in ring buffers
    /// do not.
    #[test]
    fn state_digest_covers_the_components() {
        let Some((data, idx)) = load_sample("s3_dvs_a") else { return };
        let idx = &idx[..300.min(idx.len())];
        let mut a = MpeghDecoderContext::with_threads(2, 1).unwrap();
        let mut b = MpeghDecoderContext::with_threads(2, 1).unwrap();
        feed(&mut a, &data, idx);
        feed(&mut b, &data, idx);
        assert_eq!(a.state_digest(), b.state_digest());
        assert_eq!(a.state_digest(), a.state_digest());
        let (t0, t1) = (a.state_trace(8), b.state_trace(8));
        assert_eq!(t0.1.len(), t1.1.len());
        assert!(mpeghdec_fdk::digest::diff_traces(&t0.1, &t1.1).is_empty());
        /* a decoder that decoded one access unit less differs */
        let mut c = MpeghDecoderContext::with_threads(2, 1).unwrap();
        feed(&mut c, &data, &idx[..idx.len() - 1]);
        assert_ne!(a.state_digest(), c.state_digest());

        let flip = |x: &mut i32| *x ^= 1;
        /* MpeghDecoderContext queues and flags */
        check(&mut a, "timestamp queue", true, &|d| {
            *deque_at_mut(&mut d.timestamp_in_queue, 0).unwrap() ^= 1
        });
        check(&mut a, "zero_signal", true, &|d| d.zero_signal = !d.zero_signal);
        check(&mut a, "drc settings", true, &|d| d.last_drc_target ^= 1);
        /* transport decoder */
        check(&mut a, "transport input buffer", true, &|d| {
            let tp = core(d).h_input.as_deref_mut().unwrap();
            let mut v = tp.bs_buffer();
            v[40000] ^= 0x10;
            tp.set_bs_buffer(&v);
        });
        check(&mut a, "transport configuration", true, &|d| {
            core(d).h_input.as_deref_mut().unwrap().asc[0].m_sampling_frequency ^= 1
        });
        /* core decoder */
        check(&mut a, "stream info", true, &|d| core(d).stream_info.sample_rate ^= 1);
        check(&mut a, "overlap buffer", true, &|d| {
            flip(&mut core(d).p_aac_decoder_static_channel_info[0].i_mdct.overlap[3])
        });
        check(&mut a, "noise filling seed", true, &|d| {
            core(d).p_aac_decoder_static_channel_info[1].nf_random_seed ^= 1
        });
        check(&mut a, "concealment spectrum", true, &|d| {
            core(d).p_aac_decoder_static_channel_info[0].concealment_info.spectral_coefficient[7] ^=
                1
        });
        check(&mut a, "arithmetic coder context", true, &|d| {
            let ar = core(d).p_aac_decoder_static_channel_info[0].h_ar_co.as_deref_mut().unwrap();
            ar.c_prev[5] ^= 1
        });
        check(&mut a, "UI manager", true, &|d| core(d).h_ui_manager.xml_state_changed ^= 1);
        check(&mut a, "UI status", true, &|d| core(d).ui_status_valid ^= 1);
        let toggle =
            |o: FastOptions| if o.is_exact() { FastOptions::ALL } else { FastOptions::EXACT };
        check(&mut a, "fast options", true, &|d| d.fast_options = toggle(d.fast_options));
        check(&mut a, "fast options of the core decoder", true, &|d| {
            let o = core(d).fast_options();
            core(d).set_fast_options(toggle(o))
        });
        /* render state */
        check(&mut a, "limiter", true, &|d| {
            flip(&mut render(d).h_limiter.as_mut().unwrap().smooth_state0)
        });
        check(&mut a, "limiter delay line", true, &|d| {
            flip(&mut render(d).h_limiter.as_mut().unwrap().delay_buf[1])
        });
        check(&mut a, "DRC gain decoder", true, &|d| {
            let g = render(d).h_uni_drc_decoder.h_gain_dec.as_deref_mut().unwrap();
            flip(&mut g.channel_gain_prev[0])
        });
        check(&mut a, "DRC dummy node buffer", true, &|d| {
            let g = render(d).h_uni_drc_decoder.h_gain_dec.as_deref_mut().unwrap();
            flip(&mut g.drc_gain_buffers.linear_node_buffer[48].linear_node_gain[2][0])
        });
        check(&mut a, "format converter energies", true, &|d| {
            let fc = render(d).p_format_converter.as_deref_mut().unwrap();
            let st = fc.member.fc_state.as_deref_mut().unwrap();
            let dmx = st.handle_active_dmx_stft.as_deref_mut().unwrap();
            flip(&mut dmx.target_ene_prev[0][3])
        });
        check(&mut a, "STFT filterbank", true, &|d| {
            let fc = render(d).p_format_converter.as_deref_mut().unwrap();
            let fb = fc.member.stft_filterbank_analysis[0].as_deref_mut().unwrap();
            flip(&mut fb.prev_audio_input[10])
        });
        check(&mut a, "rendered delay line", true, &|d| {
            flip(&mut render(d).mpeg_h_rendered_delay.delay_line[0])
        });
        check(&mut a, "earcon decoder", true, &|d| flip(&mut render(d).earcon_decoder.earcon_gain));

        /* not part of the digest */
        check(&mut a, "workBufferCore1", false, &|d| flip(&mut core(d).work_buffer_core1.buf[9]));
        check(&mut a, "workBufferCore2", false, &|d| flip(&mut core(d).work_buffer_core2[9]));
        check(&mut a, "temporal channel memory", false, &|d| {
            core(d).p_aac_decoder_channel_info[0].p_dyn_data.a_scale_factor[3] ^= 1
        });
        check(&mut a, "TD noise seed", false, &|d| {
            core(d).p_aac_decoder_static_channel_info[0].concealment_info.td_noise_seed ^= 1
        });
        check(&mut a, "statistics", false, &|d| core(d).stream_info.num_total_bytes ^= 1);
        check(&mut a, "get_samples scratch", false, &|d| d.get_scratch[0] ^= 1);
        check(&mut a, "bypass statistics", false, &|d| d.fast_bypass_frames ^= 1);

        /* ring buffers: the position of the elements is not part of the digest */
        let d0 = a.state_digest();
        {
            let l = render(&mut a).h_limiter.as_mut().unwrap();
            let (at, ch) = (l.attack as usize, l.channels as usize);
            l.delay_buf[..at * ch].rotate_right(ch);
            l.delay_buf_idx = (l.delay_buf_idx + 1) % at as u32;
            let n = at + 1;
            l.max_buf[..n].rotate_right(1);
            l.max_buf_idx = (l.max_buf_idx + 1) % n as u32;
        }
        assert_eq!(a.state_digest(), d0, "limiter ring buffers");
        {
            let q = &mut a.timestamp_in_queue;
            let v: Vec<u64> = (0..q.size).map(|i| *deque_at(q, i).unwrap()).collect();
            let first = (q.first + 3) % q.max;
            for (i, x) in v.iter().enumerate() {
                q.data[(first as usize + i) % q.max as usize] = *x;
            }
            q.first = first;
            q.last = (first + q.size) % q.max;
        }
        assert_eq!(a.state_digest(), d0, "deque");
        /* the rotated decoder decodes the same */
        feed(&mut a, &data, &idx[..40]);
        feed(&mut b, &data, &idx[..40]);
        assert_eq!(a.state_digest(), b.state_digest());
    }
}

#[cfg(test)]
mod x87_reference_tests {
    use super::*;
    use mpeghdec_fdk::common_fix::f_mult_div2;
    use mpeghdec_fdk::fixpoint_math::{inv_fixp, inv_fixp_e, inv_sqrt_norm2, sqrt_fixp};

    /// Compares the x87 model of the 32 bit x86 variant with the values of the reference
    /// compiler printed by `scripts/x87/x87_reference.cpp` (the file named by
    /// `MPEGHDEC_X87_REFERENCE`, see `scripts/x87/x87_check.py`). Skipped without it.
    #[test]
    fn x87_matches_the_reference() {
        let Some(path) = std::env::var_os("MPEGHDEC_X87_REFERENCE") else { return };
        if !mpeghdec_fdk::ARCH_X86_32 {
            panic!("build the 32 bit x86 variant (MPEGHDEC_ARCH=x86_32)");
        }
        let text = std::fs::read_to_string(path).unwrap();
        let (mut lines, mut errors) = (0, Vec::new());
        for line in text.lines() {
            let v: Vec<i64> = line[2..].split(' ').map(|x| x.parse().unwrap()).collect();
            let ok = if line.starts_with("P ") {
                let (op, q) = (v[0] as i32, v[1]);
                let (mut e1, mut e2) = (0, v[2] as i32);
                let r1 = inv_sqrt_norm2(op, &mut e1);
                let r2 = inv_fixp_e(op, &mut e2);
                [r1 as i64, e1 as i64, r2 as i64, e2 as i64] == v[3..7]
                    && (op < 0 || sqrt_fixp(op) as i64 == v[7])
                    && inv_fixp(op) as i64 == v[8]
                    && f_mult_div2(q, (op as i64).wrapping_mul(7919)) == v[9]
                    && f_mult_div2(q, op) == v[10]
            } else {
                fade_x87(v[0] as i32, v[1] as i32, v[2] as i32, v[3] != 0) as i64 == v[4]
            };
            lines += 1;
            if !ok {
                errors.push(line.to_string());
            }
        }
        assert!(lines > 0);
        assert!(errors.is_empty(), "{} of {lines} differ, first: {}", errors.len(), errors[0]);
    }
}
