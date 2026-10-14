// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: the safe Rust API of the decoder (`include/mpeghdecoder.h`).

//! [`Decoder`]: the MPEG-H decoder with timestamp handling.

use mpeghdec_core::mpeghdecoder as c;
use mpeghdec_core::mpeghdecoder::MpeghDecoderContext;

pub use mpeghdec_core::fast::FastOptions;

/// Maximum number of samples per channel of an output frame.
pub const MAX_NUM_FRAME_SAMPLES: usize = c::MAX_NUM_FRAME_SAMPLES;
/// Maximum number of output channels.
pub const MAX_NUM_OUTPUT_CHANNELS: usize = c::MAX_NUM_OUTPUT_CHANNELS;

/// The errors of the decoder (`MPEGH_DECODER_ERROR`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Error {
    /// The target layout is not supported, or the decoder could not be created.
    InvalidTargetLayout,
    /// Memory or threads could not be allocated (`MPEGH_DEC_OUT_OF_MEMORY`).
    OutOfMemory,
    /// The input could not be buffered because the buffer is full, or the output buffer is too
    /// small (`MPEGH_DEC_BUFFER_ERROR`).
    Buffer,
    /// The input could not be processed (`MPEGH_DEC_PROCESS_ERROR`).
    Process,
    /// A parameter or its value is not supported (`MPEGH_DEC_UNSUPPORTED_PARAM`).
    UnsupportedParam,
    /// The output configuration (sample rate, number of channels) changed; the decoder has to
    /// be created again (`MPEGH_DEC_NEEDS_RESTART`).
    NeedsRestart,
}

impl Error {
    pub(crate) fn from_code(code: c::MpeghDecoderError) -> Error {
        match code {
            c::MPEGH_DEC_OUT_OF_MEMORY => Error::OutOfMemory,
            c::MPEGH_DEC_BUFFER_ERROR => Error::Buffer,
            c::MPEGH_DEC_UNSUPPORTED_PARAM => Error::UnsupportedParam,
            c::MPEGH_DEC_NEEDS_RESTART => Error::NeedsRestart,
            _ => Error::Process,
        }
    }

    /// The `MPEGH_DECODER_ERROR` value of the C API (`MPEGH_DEC_UNSUPPORTED_PARAM` for
    /// [`Error::InvalidTargetLayout`], where C returns no decoder).
    pub fn code(self) -> i32 {
        match self {
            Error::InvalidTargetLayout | Error::UnsupportedParam => c::MPEGH_DEC_UNSUPPORTED_PARAM,
            Error::OutOfMemory => c::MPEGH_DEC_OUT_OF_MEMORY,
            Error::Buffer => c::MPEGH_DEC_BUFFER_ERROR,
            Error::Process => c::MPEGH_DEC_PROCESS_ERROR,
            Error::NeedsRestart => c::MPEGH_DEC_NEEDS_RESTART,
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::InvalidTargetLayout => "unsupported target layout",
            Error::OutOfMemory => "out of memory",
            Error::Buffer => "buffer full or too small",
            Error::Process => "input data could not be processed",
            Error::UnsupportedParam => "unsupported parameter",
            Error::NeedsRestart => "decoder restart required",
        })
    }
}

impl std::error::Error for Error {}

/// Result type of the decoder.
pub type Result<T> = core::result::Result<T, Error>;

fn result(code: c::MpeghDecoderError) -> Result<()> {
    if code == c::MPEGH_DEC_OK {
        Ok(())
    } else {
        Err(Error::from_code(code))
    }
}

/// The decoder parameters (`MPEGH_DECODER_PARAMETER`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Param {
    /// MPEG-D DRC target reference level / decoder target loudness in steps of 0.25 LU below full
    /// scale, 40 (-10 LKFS) to 127 (-31.75 LKFS). Default: 96 (-24 LKFS).
    TargetReferenceLevel,
    /// MPEG-D DRC effect type request: -1 DRC off, 0 none (default), 1 late night, 2 noisy
    /// environment, 3 limited playback range, 4 low playback level, 5 dialog enhancement, 6
    /// general compression.
    EffectType,
    /// Scaling factor for boosting gains, 0..=127 (default 127).
    BoostFactor,
    /// Scaling factor for attenuating gains, 0..=127 (default 127).
    AttenuationFactor,
    /// Album mode: 0 disabled (default), 1 enabled.
    AlbumMode,
}

impl Param {
    /// The `MPEGH_DECODER_PARAMETER` value of the C API.
    pub fn code(self) -> i32 {
        match self {
            Param::TargetReferenceLevel => c::MPEGH_DEC_PARAM_TARGET_REFERENCE_LEVEL,
            Param::EffectType => c::MPEGH_DEC_PARAM_EFFECT_TYPE,
            Param::BoostFactor => c::MPEGH_DEC_PARAM_BOOST_FACTOR,
            Param::AttenuationFactor => c::MPEGH_DEC_PARAM_ATTENUATION_FACTOR,
            Param::AlbumMode => c::MPEGH_DEC_PARAM_ALBUM_MODE,
        }
    }
}

/// The description of an output frame (`MPEGH_DECODER_OUTPUT_INFO`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct OutputInfo {
    /// Samples per channel.
    pub samples_per_channel: usize,
    /// Number of channels.
    pub channels: usize,
    /// Sample rate in Hz.
    pub sample_rate: u32,
    /// Presentation timestamp in ticks of the sample rate.
    pub ticks: u64,
    /// Presentation timestamp in nanoseconds.
    pub pts: u64,
    /// Output loudness in steps of -0.25 LU, 0 (0 LKFS) to 231 (-57.75 LKFS); `None` without
    /// loudness metadata.
    pub loudness: Option<i32>,
    /// The samples are concealed (the access unit was lost or corrupt).
    pub concealed: bool,
}

impl OutputInfo {
    /// The number of interleaved samples of the frame (`samples_per_channel * channels`).
    pub fn samples(&self) -> usize {
        self.samples_per_channel * self.channels
    }
}

impl From<c::MpeghDecoderOutputInfo> for OutputInfo {
    fn from(i: c::MpeghDecoderOutputInfo) -> OutputInfo {
        OutputInfo {
            samples_per_channel: i.num_samples_per_channel.max(0) as usize,
            channels: i.num_channels.max(0) as usize,
            sample_rate: i.sample_rate.max(0) as u32,
            ticks: i.ticks,
            pts: i.pts,
            loudness: (i.loudness >= 0).then_some(i.loudness),
            concealed: i.is_concealed,
        }
    }
}

/// The number of output channels of the CICP target layout `target_layout` (0 if unsupported).
pub fn num_output_channels(target_layout: i32) -> usize {
    mpeghdec_fdk::cicp2geometry::cicp2geometry_get_num_channels_from_cicp_noerr(target_layout)
        .max(0) as usize
}

/// An MPEG-H 3D Audio decoder with timestamp handling.
///
/// [`Decoder::process`] takes one access unit (MHAS packets, or a raw `mha1` access unit after
/// [`Decoder::set_mha_config`]) with its presentation timestamp; [`Decoder::get_samples`] returns
/// the decoded frames. The decoder adjusts the output to the timestamps: it inserts silence for
/// gaps, drops samples for overlaps (with fades) and restarts when two consecutive timestamps are
/// more than 200 ms apart.
///
/// ```no_run
/// # fn run(access_units: &[(Vec<u8>, u64)]) -> mpeghdec::Result<()> {
/// let mut dec = mpeghdec::Decoder::new(2)?; // stereo output
/// let mut out = vec![0i32; dec.output_buffer_len()];
/// for (au, pts) in access_units {
///     dec.process_timescale(au, *pts, 48000)?;
///     while let Some(info) = dec.get_samples(&mut out)? {
///         let samples = &out[..info.samples()]; // interleaved, 24 bit in the upper bits
///     }
/// }
/// dec.flush_and_get()?;
/// while let Some(info) = dec.get_samples(&mut out)? { /* ... */ }
/// # Ok(()) }
/// ```
///
/// # Threads
///
/// A decoder runs on the calling thread unless more threads are requested
/// ([`Decoder::with_threads`], [`Decoder::set_threads`]). With more than one thread it renders an
/// access unit while the next one is decoded (frame pipelining) and spreads independent parts of
/// a frame over the other threads. The output frames are then returned one `process()` call later;
/// the sequence of frames is the same as with one thread if [`Decoder::get_samples`] is called
/// until it returns `None` after every `process()`, `flush()` and `flush_and_get()` call, as above.
/// For whole files, [`crate::chunked::decode_chunked`] decodes in parallel chunks.
pub struct Decoder {
    pub(crate) ctx: Box<MpeghDecoderContext>,
    target_layout: i32,
}

impl core::fmt::Debug for Decoder {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Decoder")
            .field("target_layout", &self.target_layout)
            .field("threads", &self.threads())
            .finish_non_exhaustive()
    }
}

impl Decoder {
    /// A decoder rendering to the CICP target layout `target_layout` (1..=20 without 8,
    /// 100..=351, 400..=422; 2 is stereo, 6 is 5.1), running on the calling thread.
    pub fn new(target_layout: i32) -> Result<Decoder> {
        Decoder::with_threads(target_layout, 1)
    }

    /// Like [`Decoder::new`], decoding with `threads` threads (0 or 1: the calling thread only).
    /// If the worker threads cannot be created, the decoder runs on the calling thread.
    pub fn with_threads(target_layout: i32, threads: usize) -> Result<Decoder> {
        let ctx = MpeghDecoderContext::with_threads(target_layout, threads)
            .ok_or(Error::InvalidTargetLayout)?;
        Ok(Decoder { ctx: Box::new(ctx), target_layout })
    }

    /// The CICP target layout.
    #[must_use]
    pub fn target_layout(&self) -> i32 {
        self.target_layout
    }

    /// The number of output channels of the target layout.
    #[must_use]
    pub fn num_output_channels(&self) -> usize {
        num_output_channels(self.target_layout)
    }

    /// The length of the output buffer that [`Decoder::get_samples`] needs
    /// ([`MAX_NUM_FRAME_SAMPLES`] per output channel).
    #[must_use]
    pub fn output_buffer_len(&self) -> usize {
        self.num_output_channels() * MAX_NUM_FRAME_SAMPLES
    }

    /// Sets the configuration (`mpegh3daConfig()`) of a raw `mha1` track; the access units are
    /// then raw `mha1` access units instead of MHAS packets.
    pub fn set_mha_config(&mut self, config: &[u8]) -> Result<()> {
        result(c::mpeghdecoder_set_mha_config(&mut self.ctx, config))
    }

    /// Feeds one access unit with its presentation timestamp in nanoseconds.
    pub fn process(&mut self, access_unit: &[u8], timestamp_ns: u64) -> Result<()> {
        result(c::mpeghdecoder_process(&mut self.ctx, access_unit, timestamp_ns))
    }

    /// Feeds one access unit with its presentation timestamp in units of `1 / timescale` s.
    pub fn process_timescale(
        &mut self,
        access_unit: &[u8],
        timestamp: u64,
        timescale: u32,
    ) -> Result<()> {
        result(c::mpeghdecoder_process_timescale(&mut self.ctx, access_unit, timestamp, timescale))
    }

    /// Takes the next output frame: its interleaved samples (32 bit with 24 significant bits in
    /// the upper bits) are written to the start of `out`, which must hold at least
    /// [`Decoder::output_buffer_len`] samples. `None` if no complete frame is available: feed
    /// more input.
    pub fn get_samples(&mut self, out: &mut [i32]) -> Result<Option<OutputInfo>> {
        let mut info = c::MpeghDecoderOutputInfo::default();
        match c::mpeghdecoder_get_samples(&mut self.ctx, out, &mut info) {
            c::MPEGH_DEC_OK => Ok(Some(info.into())),
            c::MPEGH_DEC_FEED_DATA => Ok(None),
            e => Err(Error::from_code(e)),
        }
    }

    /// Ends the input: decodes the access units still buffered; take their output with
    /// [`Decoder::get_samples`] until it returns `None`.
    pub fn flush_and_get(&mut self) -> Result<()> {
        result(c::mpeghdecoder_flush_and_get(&mut self.ctx))
    }

    /// Discards the buffered input and output (e.g. before a seek).
    pub fn flush(&mut self) -> Result<()> {
        result(c::mpeghdecoder_flush(&mut self.ctx))
    }

    /// Sets a decoder parameter; it applies from the next access unit on.
    pub fn set_param(&mut self, param: Param, value: i32) -> Result<()> {
        result(c::mpeghdecoder_set_param(&mut self.ctx, param.code(), value))
    }

    /// Sets the number of threads (0 or 1: the calling thread only). The output does not depend
    /// on it. Fails with [`Error::OutOfMemory`] if the threads cannot be created (the decoder
    /// then runs on the calling thread).
    pub fn set_threads(&mut self, threads: usize) -> Result<()> {
        self.ctx.set_num_threads(threads).map_err(Error::from_code)
    }

    /// The number of threads.
    #[must_use]
    pub fn threads(&self) -> usize {
        self.ctx.num_threads()
    }

    /// Enables the fast mode (see [`FastOptions`]): **the output is then not bit-exact with the
    /// reference decoder**. Off by default.
    pub fn set_fast_mode(&mut self, enable: bool) {
        self.ctx.set_fast_mode(enable);
    }

    /// Sets the fast mode options (see [`Decoder::set_fast_mode`]).
    pub fn set_fast_options(&mut self, options: FastOptions) {
        self.ctx.set_fast_options(options);
    }

    /// The fast mode options.
    #[must_use]
    pub fn fast_options(&self) -> FastOptions {
        self.ctx.fast_options()
    }

    /// The number of output frames rendered with the fast mode bypass of the format converter
    /// so far.
    pub fn fast_bypass_frames(&mut self) -> u64 {
        self.ctx.fast_bypass_frames()
    }
}
