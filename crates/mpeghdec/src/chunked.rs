// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: chunked parallel decoding of whole files.

//! Chunked parallel decoding of a whole seekable input (not in C).
//!
//! [`decode_chunked`] returns the same sequence of output frames as the sequential decoding loop
//! of the demo decoder (for every access unit `process()`, after the last one `flush_and_get()`,
//! then `get_samples()` until it returns `None`), using several threads:
//!
//! - The access units are split into chunks. The *true run* decodes from the first access unit
//!   like the sequential decoder. Every further chunk is decoded speculatively by its own
//!   single-threaded decoder (a *speculative run*), which starts at a random access point some
//!   access units before the chunk (warm-up; its output is discarded).
//! - A chunk boundary is accepted only if the complete state of the decoder that decoded up to the
//!   boundary equals the state of the speculative run of the next chunk at the boundary, compared
//!   by their [digests](mpeghdec_fdk::digest). Since the decoder is deterministic, both decoders
//!   then return the same output from the boundary on. If the digests differ, the comparison is
//!   repeated at a few later access units of the chunk (more warm-up); if they never match, the
//!   decoder that decoded up to the boundary simply continues through the next chunk.
//! - Some state is not part of the digest (the random seed of the concealment noise) or only up
//!   to the position of the elements in ring buffers. Whenever such state could influence the
//!   decoding, the decoder signals a [hazard](mpeghdec_fdk::digest::hazard). The output of a
//!   speculative run is used only up to its first hazard; from there on, the true run (paused at
//!   the boundary where it handed over) decodes up to that point and continues.
//!
//! The output is streamed in order to the sink on the calling thread; speculative runs buffer
//! their output until it is their turn (bounded by [`ChunkedOptions::max_buffered_bytes`]).

use crate::{Error, OutputInfo, Param};
use mpeghdec_core::mpeghdecoder::{
    MpeghDecoderContext as MpeghDecoder, MpeghDecoderError, MpeghDecoderOutputInfo,
    MAX_NUM_FRAME_SAMPLES, MAX_NUM_OUTPUT_CHANNELS,
};
use mpeghdec_fdk::digest::{diff_traces, hazard_count, Digest};
use std::collections::{BTreeMap, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex};

/// `PACTYP_MPEGH3DACFG`
const PACTYP_MPEGH3DACFG: u32 = 1;
/// `PACTYP_MPEGH3DAFRAME`
const PACTYP_MPEGH3DAFRAME: u32 = 2;

/// The threshold of `MpeghDecoderContext::process()` for a restart of the decoder (200 ms in ns).
const RESTART_THRESHOLD_NS: u64 = 200_000_000;

/// Size of the input buffer of the transport decoder.
const INPUT_BUFFER_SIZE: usize = mpeghdec_tpdec::TRANSPORTDEC_BUFFER_SIZE;

/// Depth of the state traces of the debug output.
const TRACE_DEPTH: usize = 16;

/// One access unit of an input held in memory: the data passed to
/// [`Decoder::process_timescale`](crate::Decoder::process_timescale) and its presentation
/// timestamp in ticks of the timescale.
#[derive(Clone, Copy, Debug)]
pub struct AccessUnit<'a> {
    /// The access unit.
    pub data: &'a [u8],
    /// Its presentation timestamp in ticks of the timescale.
    pub pts: u64,
}

/// The input of [`decode_chunked`]: a sequence of access units (MHAS access units, or raw
/// `mha1` access units with [`DecoderSetup::mha_config`]) with random access to their data.
pub trait AccessUnitSource: Sync {
    /// The number of access units.
    fn len(&self) -> usize;
    /// Whether there are no access units.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// The presentation timestamp of access unit `index` in ticks of
    /// [`AccessUnitSource::timescale`].
    fn pts(&self, index: usize) -> u64;
    /// The size of access unit `index` in bytes.
    fn size(&self, index: usize) -> usize;
    /// The timescale of the timestamps.
    fn timescale(&self) -> u32;
    /// A reader of the access unit data (every thread opens its own).
    fn reader(&self) -> std::io::Result<Box<dyn AccessUnitReader + Send + '_>>;
}

/// Reads the data of access units (see [`AccessUnitSource::reader`]).
pub trait AccessUnitReader {
    /// The data of access unit `index`, valid until the next call.
    fn read(&mut self, index: usize) -> std::io::Result<&[u8]>;
}

/// An [`AccessUnitSource`] of access units in memory.
pub struct MemorySource<'a> {
    /// The access units in decoding order.
    pub access_units: &'a [AccessUnit<'a>],
    /// The timescale of the timestamps (ticks per second).
    pub timescale: u32,
}

impl AccessUnitSource for MemorySource<'_> {
    fn len(&self) -> usize {
        self.access_units.len()
    }
    fn pts(&self, index: usize) -> u64 {
        self.access_units[index].pts
    }
    fn size(&self, index: usize) -> usize {
        self.access_units[index].data.len()
    }
    fn timescale(&self) -> u32 {
        self.timescale
    }
    fn reader(&self) -> std::io::Result<Box<dyn AccessUnitReader + Send + '_>> {
        struct R<'a>(&'a [AccessUnit<'a>]);
        impl AccessUnitReader for R<'_> {
            fn read(&mut self, index: usize) -> std::io::Result<&[u8]> {
                Ok(self.0[index].data)
            }
        }
        Ok(Box::new(R(self.access_units)))
    }
}

/// The receiver of the output frames of [`decode_chunked`]: the frame info and the interleaved
/// samples (as returned by [`Decoder::get_samples`](crate::Decoder::get_samples)).
pub type Sink<'a, E> = dyn FnMut(&OutputInfo, &[i32]) -> Result<(), E> + 'a;

/// The configuration of the decoders (like the demo decoder: create, set the parameters, set the
/// `mha1` configuration).
#[derive(Clone, Debug, Default)]
pub struct DecoderSetup {
    /// CICP index of the target layout ([`Decoder::new`](crate::Decoder::new)).
    pub cicp_setup: i32,
    /// Parameters set with [`Decoder::set_param`](crate::Decoder::set_param) before decoding.
    pub params: Vec<(Param, i32)>,
    /// Out-of-band configuration of an `mha1` track
    /// ([`Decoder::set_mha_config`](crate::Decoder::set_mha_config)).
    pub mha_config: Option<Vec<u8>>,
    /// Fast mode options ([`Decoder::set_fast_options`](crate::Decoder::set_fast_options);
    /// default: bit-exact). With a fast
    /// option the output is that of a sequential decoding with the same options (the options are
    /// part of the state digest).
    pub fast: crate::FastOptions,
}

impl DecoderSetup {
    fn create(&self, threads: usize) -> Result<MpeghDecoder, String> {
        let mut dec = MpeghDecoder::with_threads(self.cicp_setup, threads)
            .ok_or_else(|| "unable to create the MPEG-H decoder".to_string())?;
        for &(p, v) in &self.params {
            let _ = dec.set_param(p.code(), v);
        }
        dec.set_fast_options(self.fast);
        if let Some(c) = &self.mha_config {
            dec.set_mha_config(c).map_err(|e| {
                format!("unable to set the MHA configuration: {}", Error::from_code(e))
            })?;
        }
        Ok(dec)
    }
}

/// Options of [`decode_chunked`].
#[derive(Clone, Debug)]
pub struct ChunkedOptions {
    /// Number of threads (0: the available parallelism): worker threads for the speculative runs;
    /// the calling thread decodes too (the true run and continuations) and passes the output on.
    pub threads: usize,
    /// Number of chunks (0: automatic, see [`ChunkedOptions::min_chunk_len`]). With one chunk the
    /// input is decoded sequentially.
    pub chunks: usize,
    /// Access units a speculative run decodes before its chunk (at least; it starts at the
    /// latest random access point before).
    pub warmup: usize,
    /// Access units after the start of a chunk at which its boundary is verified (in this
    /// order; the first one where the states are equal is taken).
    pub verify_offsets: Vec<usize>,
    /// Minimum number of access units per chunk of the automatic chunk count.
    pub min_chunk_len: usize,
    /// Minimum number of threads for the automatic chunk count to split the input at all. With
    /// two threads the frame pipeline of the sequential decoding is about as fast and needs far
    /// less memory.
    pub min_auto_threads: usize,
    /// Limit of the memory of the speculative runs that wait for their turn (bytes): their
    /// buffered output and their decoders. A speculative run waits while the limit is exceeded
    /// (unless the output is taken from it or its state digest is awaited). The decoders that
    /// are running (one per thread) come on top.
    pub max_buffered_bytes: usize,
    /// Start speculative runs also at independent frames (with the configuration packets of the
    /// latest access unit that has them prepended) when there is no access unit with a
    /// configuration packet before a chunk. Off by default: without configuration repetition the
    /// states rarely become equal (e.g. the noise filling seeds depend on all frames since the
    /// configuration). Raw `mha1` access units always start at independent frames.
    pub independent_starts: bool,
    /// After this many consecutive boundaries whose states differ (while the true run produces
    /// the output), the chunked decoding is given up: the true run decodes the rest with all
    /// threads (0: never).
    pub max_rejections: usize,
    /// Log the chunk decisions to stderr.
    pub log: bool,
    /// With [`ChunkedOptions::log`]: list the fields in which the states differ at every
    /// verification point where they differ.
    pub log_state_differences: bool,
}

impl Default for ChunkedOptions {
    fn default() -> Self {
        ChunkedOptions {
            threads: 0,
            chunks: 0,
            warmup: 216,
            verify_offsets: vec![0, 64, 192],
            min_chunk_len: 1024,
            min_auto_threads: 4,
            max_buffered_bytes: 256 << 20,
            independent_starts: false,
            max_rejections: 3,
            log: false,
            log_state_differences: false,
        }
    }
}

impl ChunkedOptions {
    /// [`ChunkedOptions::threads`] with 0 resolved.
    fn effective_threads(&self) -> usize {
        if self.threads == 0 {
            std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1)
        } else {
            self.threads
        }
    }

    /// The number of chunks for `n` access units decoded to `channels` output channels.
    ///
    /// Automatic: none (1) for less than two [`ChunkedOptions::min_chunk_len`] access units or
    /// fewer than [`ChunkedOptions::min_auto_threads`] threads, otherwise four per thread, but at
    /// least so many that the output of one chunk per worker thread fits into
    /// [`ChunkedOptions::max_buffered_bytes`] (the output of the chunks that finished before their
    /// turn is buffered), and chunks of at least three warm-up lengths.
    pub fn num_chunks(&self, n: usize, channels: usize) -> usize {
        if self.chunks > 0 {
            return self.chunks.min(n.max(1));
        }
        let threads = self.effective_threads();
        let base = (n / self.min_chunk_len.max(1)).min(4 * threads);
        if threads <= 1 || threads < self.min_auto_threads || base < 2 {
            return 1;
        }
        /* output of an access unit: 1024 samples of 32 bits per channel */
        let per_au = channels.max(1) * 1024 * 4;
        let mem_len = self.max_buffered_bytes / ((threads - 1).max(1) * per_au);
        let len = mem_len.max(3 * self.warmup).max(1);
        base.max(n / len)
    }
}

/// Statistics of [`decode_chunked`].
#[derive(Clone, Debug, Default)]
pub struct ChunkedStats {
    /// Number of chunks.
    pub chunks: usize,
    /// Boundaries at which the output was handed over to a speculative run.
    pub handoffs: usize,
    /// Speculative runs that were not used (states not equal at any verification point, errors,
    /// not started in time).
    pub rejected: usize,
    /// Hazards after which the true run took over.
    pub fallbacks: usize,
    /// Access units decoded in total (all runs, including warm-up and discarded output).
    pub decoded_access_units: usize,
    /// State digests computed.
    pub digests: usize,
    /// Output frames returned.
    pub frames: u64,
    /// Of the output: frames rendered with the fast mode bypass of the format converter (see
    /// [`Decoder::fast_bypass_frames`](crate::Decoder::fast_bypass_frames)).
    pub fast_bypass_frames: u64,
    /// The access unit from which on the input was decoded sequentially with all threads
    /// (`Some(0)`: no chunk has a random access point, or there is only one chunk).
    pub sequential_from: Option<usize>,
}

/// Errors of [`decode_chunked`]. They are the errors of the sequential decoding loop, reported
/// after all output frames before them.
#[derive(Debug)]
pub enum ChunkedError<E> {
    /// A decoder could not be created or configured.
    Setup(String),
    /// Reading access unit `index` failed.
    Input {
        /// The index of the access unit.
        index: usize,
        /// The error of the source.
        error: std::io::Error,
    },
    /// `process()` of access unit `index` failed.
    Process {
        /// The index of the access unit.
        index: usize,
        /// The error of the decoder.
        error: Error,
    },
    /// `flush_and_get()` after the last access unit failed.
    Flush(Error),
    /// `get_samples()` failed after `frames` output frames.
    Output {
        /// The number of output frames before the error.
        frames: u64,
        /// The error of the decoder.
        error: Error,
    },
    /// The sink returned an error.
    Sink(E),
}

impl<E: std::error::Error + 'static> std::error::Error for ChunkedError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ChunkedError::Setup(_) => None,
            ChunkedError::Input { error, .. } => Some(error),
            ChunkedError::Process { error, .. }
            | ChunkedError::Flush(error)
            | ChunkedError::Output { error, .. } => Some(error),
            ChunkedError::Sink(e) => Some(e),
        }
    }
}

impl<E: core::fmt::Display> core::fmt::Display for ChunkedError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ChunkedError::Setup(s) => write!(f, "{s}"),
            ChunkedError::Input { index, error } => write!(f, "[{index}] reading failed: {error}"),
            ChunkedError::Process { index, error } => write!(f, "[{index}] process: {error}"),
            ChunkedError::Flush(e) => write!(f, "flush: {e}"),
            ChunkedError::Output { frames, error } => write!(f, "[{frames}] get_samples: {error}"),
            ChunkedError::Sink(e) => write!(f, "{e}"),
        }
    }
}

/* ---------------------------------------------------------------------------------------- */
/* MHAS helpers                                                                             */
/* ---------------------------------------------------------------------------------------- */

/// The MHAS packets of `d`: `(type, header start, payload start, payload end)`.
fn mhas_packets(d: &[u8]) -> Vec<(u32, usize, usize, usize)> {
    fn escaped(d: &[u8], p: &mut usize, n1: u32, n2: u32, n3: u32) -> Option<u64> {
        let mut u = |n: u32| -> Option<u64> {
            let mut v = 0u64;
            for _ in 0..n {
                let byte = *d.get(*p >> 3)?;
                v = (v << 1) | ((byte >> (7 - (*p & 7))) & 1) as u64;
                *p += 1;
            }
            Some(v)
        };
        let mut v = u(n1)?;
        if v == (1 << n1) - 1 {
            let e = u(n2)?;
            v += e;
            if e == (1 << n2) - 1 {
                v += u(n3)?;
            }
        }
        Some(v)
    }
    let mut out = Vec::new();
    let mut off = 0;
    while off < d.len() {
        let mut p = 0;
        let h = &d[off..];
        let (Some(t), Some(_label), Some(len)) = (
            escaped(h, &mut p, 3, 8, 8),
            escaped(h, &mut p, 2, 8, 32),
            escaped(h, &mut p, 11, 24, 24),
        ) else {
            break;
        };
        let start = off + p.div_ceil(8);
        let end = start.saturating_add(len as usize).min(d.len());
        out.push((t as u32, off, start, end));
        off = end;
    }
    out
}

/// The access unit carries a configuration packet.
fn has_config(d: &[u8]) -> bool {
    mhas_packets(d).iter().any(|p| p.0 == PACTYP_MPEGH3DACFG)
}

/// The frame packet of the access unit has `usacIndependencyFlag` set (its first bit).
fn is_independent(d: &[u8]) -> bool {
    mhas_packets(d)
        .iter()
        .any(|&(t, _, p, e)| t == PACTYP_MPEGH3DAFRAME && e > p && d[p] & 0x80 != 0)
}

/// The packets before the first frame packet of `d` (configuration, audio scene info, ...).
fn config_prefix(d: &[u8]) -> &[u8] {
    let end = mhas_packets(d).iter().find(|p| p.0 == PACTYP_MPEGH3DAFRAME).map_or(0, |p| p.1);
    &d[..end]
}

/// The timestamp of access unit `i` in ns, as `process_timescale()` computes it.
fn pts_ns(src: &dyn AccessUnitSource, i: usize) -> u64 {
    (src.pts(i) as f64 * 1e9 / src.timescale() as f64 + 0.5) as u64
}

/// The decoder restarts at access unit `i` (timestamp distance to the previous access unit above
/// the threshold of `mpeghdecoder_process()`).
fn restarts_at(src: &dyn AccessUnitSource, i: usize) -> bool {
    i > 0 && pts_ns(src, i).wrapping_sub(pts_ns(src, i - 1)) > RESTART_THRESHOLD_NS
}

/// The content of the input buffer of the transport decoder of the sequential decoder after the
/// access unit `last` (see [`mpeghdec_tpdec::TransportDec::set_bs_buffer`]): every access unit is
/// written to the start of the buffer, the bytes after it are left over from the latest earlier
/// access unit that was longer, since the decoder was opened (at the start or at a restart).
fn input_buffer_content(
    src: &dyn AccessUnitSource,
    reader: &mut dyn AccessUnitReader,
    last: usize,
) -> std::io::Result<Vec<u8>> {
    let mut buf = vec![0u8; INPUT_BUFFER_SIZE];
    let mut covered = 0;
    let mut i = last;
    loop {
        let n = src.size(i).min(INPUT_BUFFER_SIZE);
        if n > covered {
            let d = reader.read(i)?;
            buf[covered..n].copy_from_slice(&d[covered..n]);
            covered = n;
        }
        if covered == INPUT_BUFFER_SIZE || i == 0 || restarts_at(src, i) {
            break;
        }
        i -= 1;
    }
    Ok(buf)
}

/// The start of the speculative run of a chunk starting at `target`: the latest random access
/// point at least `warmup` access units before (within a search window), with the configuration
/// packets to prepend. Random access points are access units with a configuration packet, with
/// `independent_starts` also independent frames (with the configuration packets of the latest
/// access unit that has them); for raw `mha1` access units independent frames. `None` if there
/// is none.
fn find_start(
    reader: &mut dyn AccessUnitReader,
    raw: bool,
    independent_starts: bool,
    target: usize,
    warmup: usize,
) -> std::io::Result<Option<(usize, Vec<u8>)>> {
    let Some(last) = target.checked_sub(warmup) else { return Ok(None) };
    let window = 4 * warmup.max(64);
    let lo = last.saturating_sub(window);
    let mut independent = None;
    let mut i = last;
    while i > lo && i > 0 {
        let d = reader.read(i)?;
        if raw {
            if d.first().is_some_and(|b| b & 0x80 != 0) {
                return Ok(Some((i, Vec::new())));
            }
        } else if has_config(d) {
            return Ok(Some((i, Vec::new())));
        } else if independent_starts && independent.is_none() && is_independent(d) {
            independent = Some(i);
        }
        i -= 1;
    }
    let Some(ind) = independent else { return Ok(None) };
    let mut k = ind;
    while k > 0 {
        k -= 1;
        let d = reader.read(k)?;
        if has_config(d) {
            return Ok(Some((ind, config_prefix(d).to_vec())));
        }
    }
    Ok(None)
}

/* ---------------------------------------------------------------------------------------- */
/* Runs                                                                                     */
/* ---------------------------------------------------------------------------------------- */

/// An output frame.
struct Frame {
    /// The access unit after whose `process()` call the frame was returned.
    tag: usize,
    info: MpeghDecoderOutputInfo,
    samples: Vec<i32>,
}

impl Frame {
    fn bytes(&self) -> usize {
        self.samples.len() * 4
    }
}

/// Why a run stopped with an error.
#[derive(Debug)]
enum RunError {
    Setup(String),
    Input(std::io::Error),
    Process(MpeghDecoderError),
    Flush(MpeghDecoderError),
    Output(MpeghDecoderError),
}

/// State digest of a run before an access unit (with the field digests in debug mode).
#[derive(Clone)]
struct DigestInfo {
    digest: Digest,
    trace: Option<Arc<Vec<(String, Digest)>>>,
}

/// A decoder decoding the input from some access unit on.
struct Run<'s> {
    dec: MpeghDecoder,
    reader: Box<dyn AccessUnitReader + Send + 's>,
    /// The next access unit to decode.
    pos: usize,
    /// The first access unit (with `prefix` prepended).
    start: usize,
    /// Configuration packets prepended to the first access unit (start at an independent frame).
    prefix: Vec<u8>,
    buf: Vec<i32>,
    decoded: usize,
    digests: usize,
    /// Count the frames of the fast mode bypass (the options are not bit-exact).
    fast: bool,
    /// `MpeghDecoderContext::fast_bypass_frames()` when last reported.
    bypass_seen: u64,
}

/// How a run segment ended.
enum RunEnd {
    /// At the stop position.
    Paused,
    /// After the last access unit (flushed).
    Finished,
    /// The event handler asked to stop.
    Stopped,
    /// An error at access unit `at`.
    Error(usize, RunError),
}

/// Events of a run.
enum Event<'s> {
    Frame(Frame),
    /// Frames rendered with the fast mode bypass of the format converter while decoding the
    /// access unit (sent before its frames).
    Bypassed(usize, u64),
    Digest(usize, DigestInfo),
    /// A hazard while decoding the access unit (sent before its frames).
    Hazard(usize),
    Error(usize, RunError),
    /// The run reached its stop position (the run is handed back).
    Paused(Box<Run<'s>>),
    Finished,
}

impl<'s> Run<'s> {
    fn new(
        setup: &DecoderSetup,
        src: &'s dyn AccessUnitSource,
        start: usize,
        prefix: Vec<u8>,
        threads: usize,
    ) -> Result<Run<'s>, RunError> {
        let dec = setup.create(threads).map_err(RunError::Setup)?;
        let reader = src.reader().map_err(RunError::Input)?;
        Ok(Run {
            dec,
            reader,
            pos: start,
            start,
            prefix,
            buf: vec![0; MAX_NUM_OUTPUT_CHANNELS * MAX_NUM_FRAME_SAMPLES],
            decoded: 0,
            digests: 0,
            fast: !setup.fast.is_exact(),
            bypass_seen: 0,
        })
    }

    fn digest(&mut self, debug: bool) -> DigestInfo {
        self.digests += 1;
        if debug {
            let (digest, trace) = self.dec.state_trace(TRACE_DEPTH);
            DigestInfo { digest, trace: Some(Arc::new(trace)) }
        } else {
            DigestInfo { digest: self.dec.state_digest(), trace: None }
        }
    }

    /// Decodes from `pos` up to `stop` (exclusive; at the end of the input: to the end, flushing
    /// after the last access unit). Before the access units in `digest_at` the state digest is
    /// passed to `on`. Frames of access units before `emit_from` are dropped. `on` returns
    /// `false` to stop.
    #[allow(clippy::too_many_arguments)]
    fn run(
        &mut self,
        src: &dyn AccessUnitSource,
        stop: usize,
        digest_at: &[usize],
        emit_from: usize,
        debug: bool,
        cancel: Option<&AtomicBool>,
        on: &mut dyn FnMut(Event<'s>) -> bool,
    ) -> RunEnd {
        let n = src.len();
        let mut frames: Vec<Frame> = Vec::new();
        /* a multi-threaded decoder (frame pipelining) reports the bypassed frames at the end only
        (querying them waits for the rendering) */
        let per_au = self.fast && self.dec.num_threads() == 1;
        if self.fast && !per_au {
            self.bypass_seen = self.dec.fast_bypass_frames();
        }
        loop {
            if cancel.is_some_and(|c| c.load(Ordering::Relaxed)) {
                return RunEnd::Stopped;
            }
            if digest_at.contains(&self.pos) {
                let d = self.digest(debug);
                if !on(Event::Digest(self.pos, d)) {
                    return RunEnd::Stopped;
                }
            }
            if self.pos >= stop && stop < n {
                return RunEnd::Paused;
            }
            let i = self.pos;
            let hazards = hazard_count();
            let pts = src.pts(i);
            let result = match self.reader.read(i) {
                Err(e) => return RunEnd::Error(i, RunError::Input(e)),
                Ok(data) if i == self.start && !self.prefix.is_empty() => {
                    let mut v = std::mem::take(&mut self.prefix);
                    v.extend_from_slice(data);
                    self.dec.process_timescale(&v, pts, src.timescale())
                }
                Ok(data) => self.dec.process_timescale(data, pts, src.timescale()),
            };
            self.decoded += 1;
            if let Err(e) = result {
                return RunEnd::Error(i, RunError::Process(e));
            }
            if i == self.start && i > 0 {
                /* the stale content of the input buffer of the sequential decoder */
                match input_buffer_content(src, &mut *self.reader, i) {
                    Ok(c) => self.dec.set_input_buffer_content(&c),
                    Err(e) => return RunEnd::Error(i, RunError::Input(e)),
                }
            }
            let last = i + 1 == n;
            if last {
                if let Err(e) = self.dec.flush_and_get() {
                    return RunEnd::Error(i, RunError::Flush(e));
                }
            }
            let mut error = None;
            loop {
                match self.dec.get_samples(&mut self.buf) {
                    Ok(Some(info)) => {
                        if i >= emit_from {
                            let len = (info.num_samples_per_channel * info.num_channels).max(0);
                            frames.push(Frame {
                                tag: i,
                                info,
                                samples: self.buf[..len as usize].to_vec(),
                            });
                        }
                    }
                    Ok(None) => break,
                    Err(e) => {
                        error = Some(e);
                        break;
                    }
                }
            }
            if hazard_count() != hazards && !on(Event::Hazard(i)) {
                return RunEnd::Stopped;
            }
            if per_au || (self.fast && last) {
                let b = self.dec.fast_bypass_frames();
                let d = b.saturating_sub(self.bypass_seen);
                self.bypass_seen = b;
                if d > 0 && !on(Event::Bypassed(i, d)) {
                    return RunEnd::Stopped;
                }
            }
            for f in frames.drain(..) {
                if !on(Event::Frame(f)) {
                    return RunEnd::Stopped;
                }
            }
            if let Some(e) = error {
                return RunEnd::Error(i, RunError::Output(e));
            }
            self.pos += 1;
            if last {
                return RunEnd::Finished;
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------- */
/* Speculative runs                                                                         */
/* ---------------------------------------------------------------------------------------- */

const NOT_STARTED: u8 = 0;
const STARTED: u8 = 1;
const ABSORBED: u8 = 2;

/// Shared state of a speculative chunk.
struct ChunkShared {
    state: AtomicU8,
    cancel: AtomicBool,
    /// Exempt from the buffer limit: the output of the run is consumed at once, or the
    /// coordinator waits for its state digest.
    exempt: AtomicBool,
}

/// The memory held by the speculative runs for the coordinator: the events sent to it that it has
/// not taken yet (per chunk; a discarded chunk releases all of it at once).
struct BudgetState {
    total: usize,
    per_chunk: Vec<usize>,
    closed: Vec<bool>,
}

/// Limit of the memory held by the speculative runs.
struct Budget {
    state: Mutex<BudgetState>,
    cv: Condvar,
    limit: usize,
}

impl Budget {
    fn new(chunks: usize, limit: usize) -> Budget {
        Budget {
            state: Mutex::new(BudgetState {
                total: 0,
                per_chunk: vec![0; chunks],
                closed: vec![false; chunks],
            }),
            cv: Condvar::new(),
            limit,
        }
    }

    /// Accounts `n` bytes that chunk `j` sends to the coordinator; waits before while the limit
    /// is exceeded and the chunk is not exempt. Returns `false` if the chunk was discarded.
    fn acquire(&self, j: usize, n: usize, c: &ChunkShared) -> bool {
        let mut st = self.state.lock().unwrap();
        while st.total > self.limit && !c.exempt.load(Ordering::Acquire) && !st.closed[j] {
            st = self.cv.wait_timeout(st, std::time::Duration::from_millis(50)).unwrap().0;
        }
        if st.closed[j] {
            return false;
        }
        st.total += n;
        st.per_chunk[j] += n;
        true
    }

    /// The coordinator took `n` bytes of chunk `j`.
    fn release(&self, j: usize, n: usize) {
        let mut st = self.state.lock().unwrap();
        let n = n.min(st.per_chunk[j]);
        st.per_chunk[j] -= n;
        st.total -= n;
        self.cv.notify_all();
    }

    /// Chunk `j` is discarded: releases everything it holds, later sends are refused.
    fn close(&self, j: usize) {
        let mut st = self.state.lock().unwrap();
        st.closed[j] = true;
        let n = std::mem::take(&mut st.per_chunk[j]);
        st.total -= n;
        self.cv.notify_all();
    }

    fn wake(&self) {
        let _g = self.state.lock().unwrap();
        self.cv.notify_all();
    }
}

/// The plan of the chunks.
struct Plan {
    /// Chunk starts, `starts[chunks] == n`.
    starts: Vec<usize>,
    /// Where the speculative run of chunk `j` starts (with the prefix of its first access unit);
    /// `None` for chunk 0 and for chunks without a random access point.
    run_starts: Vec<Option<(usize, Vec<u8>)>>,
    /// Memory of a decoder, counted in the buffer limit while a speculative run waits for its turn
    /// (estimate).
    run_cost: usize,
    verify_offsets: Vec<usize>,
}

impl Plan {
    /// Verification points of the boundary at the start of chunk `j`.
    fn candidates(&self, j: usize) -> Vec<usize> {
        let (s, e) = (self.starts[j], self.starts[j + 1]);
        let mut v: Vec<usize> =
            self.verify_offsets.iter().map(|&o| s + o).filter(|&p| p < e).collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// The speculative run of chunk `j` (on a worker thread). Returns the number of decoded access
/// units and digests.
#[allow(clippy::too_many_arguments)]
fn speculate<'s>(
    src: &'s dyn AccessUnitSource,
    setup: &DecoderSetup,
    plan: &Plan,
    j: usize,
    debug: bool,
    sh: &ChunkShared,
    budget: &Budget,
    tx: &mpsc::Sender<Event<'s>>,
) -> (usize, usize) {
    let n = src.len();
    let s = plan.starts[j];
    let end = plan.starts[j + 1];
    let Some((w, prefix)) = plan.run_starts[j].clone() else {
        let _ = tx.send(Event::Error(s, RunError::Setup("no random access point".into())));
        return (0, 0);
    };
    let mut run = match Run::new(setup, src, w, prefix, 1) {
        Ok(r) => r,
        Err(e) => {
            let _ = tx.send(Event::Error(w, e));
            return (0, 0);
        }
    };
    let mut digest_at = plan.candidates(j);
    if end < n {
        digest_at.push(end);
    }
    let mut on = |e: Event<'s>| -> bool {
        if let Event::Frame(f) = &e {
            if !budget.acquire(j, f.bytes(), sh) {
                return false;
            }
        }
        tx.send(e).is_ok() && !sh.cancel.load(Ordering::Relaxed)
    };
    let r = run.run(src, end, &digest_at, s, debug, Some(&sh.cancel), &mut on);
    let (d, g) = (run.decoded, run.digests);
    run.decoded = 0;
    run.digests = 0;
    match r {
        RunEnd::Paused => {
            /* the decoder waits for its turn */
            if budget.acquire(j, plan.run_cost, sh) {
                let _ = tx.send(Event::Paused(Box::new(run)));
            }
        }
        RunEnd::Finished => {
            let _ = tx.send(Event::Finished);
        }
        RunEnd::Stopped => {}
        RunEnd::Error(at, e) => {
            let _ = tx.send(Event::Error(at, e));
        }
    }
    (d, g)
}

/// A speculative chunk as seen by the coordinator.
struct ChunkHandle<'s> {
    index: usize,
    shared: Arc<ChunkShared>,
    rx: Option<mpsc::Receiver<Event<'s>>>,
    /// Events read ahead while waiting for a digest.
    stash: VecDeque<Event<'s>>,
    digests: BTreeMap<usize, DigestInfo>,
    /// The run ended (error, paused, finished) or its channel is closed.
    ended: bool,
}

impl<'s> ChunkHandle<'s> {
    /// The digest of the run before access unit `at` (waits for it); `None` if the run ended or
    /// passed `at` without one.
    fn wait_digest(&mut self, at: usize) -> Option<DigestInfo> {
        loop {
            if let Some(d) = self.digests.get(&at) {
                return Some(d.clone());
            }
            if self.ended || self.digests.range(at..).next().is_some() {
                return None;
            }
            let rx = self.rx.as_ref()?;
            match rx.recv() {
                Ok(Event::Digest(p, d)) => {
                    self.digests.insert(p, d);
                }
                Ok(e) => {
                    if matches!(e, Event::Error(..) | Event::Paused(_) | Event::Finished) {
                        self.ended = true;
                    }
                    self.stash.push_back(e);
                }
                Err(_) => self.ended = true,
            }
        }
    }

    /// Whether the run has no digest at `at` or later for sure.
    fn ended_before(&self, at: usize) -> bool {
        self.ended && self.digests.range(at..).next().is_none()
    }

    fn next_event(&mut self) -> Option<Event<'s>> {
        if let Some(e) = self.stash.pop_front() {
            return Some(e);
        }
        self.rx.as_ref()?.recv().ok()
    }

    /// Drops the chunk: cancels its run and releases the memory it holds (without waiting for
    /// the run to stop).
    fn discard(&mut self, budget: &Budget) {
        self.shared.cancel.store(true, Ordering::Release);
        budget.close(self.index);
        self.stash.clear();
        self.rx = None;
        self.ended = true;
    }
}

/* ---------------------------------------------------------------------------------------- */
/* Coordinator                                                                              */
/* ---------------------------------------------------------------------------------------- */

/// Decodes the whole input `src` with the decoder configuration `setup` and passes the output
/// frames (and their info) to `sink` in order (see the [module documentation](self)). The output
/// is the same as that of the sequential loop over all access units with a single
/// [`Decoder`](crate::Decoder): `process_timescale()` for every access unit, `flush_and_get()` after the
/// last one, and `get_samples()` until it returns `None` after each of them. Decoding stops at
/// the first error, which is returned after the frames before it.
pub fn decode_chunked<E>(
    src: &dyn AccessUnitSource,
    setup: &DecoderSetup,
    opts: &ChunkedOptions,
    sink: &mut Sink<E>,
) -> Result<ChunkedStats, ChunkedError<E>> {
    let n = src.len();
    let log = opts.log;
    let debug = opts.log && opts.log_state_differences;
    let raw = setup.mha_config.is_some();
    let chunks = opts.num_chunks(n, crate::num_output_channels(setup.cicp_setup).max(1));
    let mut stats = ChunkedStats { chunks, ..Default::default() };
    if n == 0 {
        return Ok(stats);
    }
    let starts: Vec<usize> = (0..=chunks).map(|j| j * n / chunks).collect();
    /* the random access points where the speculative runs start */
    let mut run_starts = vec![None; chunks];
    if chunks > 1 {
        let mut reader = src.reader().map_err(|error| ChunkedError::Input { index: 0, error })?;
        for j in 1..chunks {
            run_starts[j] =
                find_start(&mut *reader, raw, opts.independent_starts, starts[j], opts.warmup)
                    .map_err(|error| ChunkedError::Input { index: starts[j], error })?;
        }
    }
    let viable = run_starts.iter().filter(|s| s.is_some()).count();
    let threads = opts.effective_threads();
    /* chunked decoding only if at least half of the chunks have a random access point */
    let sequential = chunks < 2 || 2 * viable < chunks - 1;
    let channels = crate::num_output_channels(setup.cicp_setup).max(1);
    let plan = Plan {
        run_cost: (6 << 20) + channels * (512 << 10),
        starts: if sequential { vec![0, n] } else { starts },
        run_starts: if sequential { vec![None] } else { run_starts },
        verify_offsets: if opts.verify_offsets.is_empty() {
            vec![0]
        } else {
            opts.verify_offsets.clone()
        },
    };
    let chunks = plan.starts.len() - 1;
    let workers = if chunks > 1 { threads.max(1).min(chunks - 1) } else { 0 };
    if log {
        if sequential {
            eprintln!(
                "chunked: {n} access units, no chunks ({viable} random access points): sequential \
                 decoding with {threads} threads"
            );
        } else {
            eprintln!(
                "chunked: {n} access units, {chunks} chunks ({viable} random access points), \
                 warm-up {}, {workers} worker threads",
                opts.warmup
            );
        }
    }

    let budget = Budget::new(chunks, opts.max_buffered_bytes);
    let mut handles: Vec<ChunkHandle> = Vec::with_capacity(chunks);
    let mut tasks = VecDeque::new();
    for j in 0..chunks {
        let (tx, rx) = mpsc::channel();
        let shared = Arc::new(ChunkShared {
            state: AtomicU8::new(NOT_STARTED),
            cancel: AtomicBool::new(false),
            exempt: AtomicBool::new(false),
        });
        if j > 0 {
            tasks.push_back((j, shared.clone(), tx));
        }
        handles.push(ChunkHandle {
            index: j,
            shared,
            rx: Some(rx),
            stash: VecDeque::new(),
            digests: BTreeMap::new(),
            ended: false,
        });
    }
    let tasks = Mutex::new(tasks);
    let decoded_by_workers = AtomicUsize::new(0);
    let digests_by_workers = AtomicUsize::new(0);

    let result = std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| loop {
                let Some((j, sh, tx)) = tasks.lock().unwrap().pop_front() else { break };
                if sh
                    .state
                    .compare_exchange(NOT_STARTED, STARTED, Ordering::AcqRel, Ordering::Acquire)
                    .is_err()
                {
                    continue;
                }
                let (d, g) = speculate(src, setup, &plan, j, debug, &sh, &budget, &tx);
                decoded_by_workers.fetch_add(d, Ordering::Relaxed);
                digests_by_workers.fetch_add(g, Ordering::Relaxed);
            });
        }
        let r = coordinate(
            src,
            setup,
            &plan,
            &mut handles,
            &budget,
            Control { debug, log, threads, max_rejections: opts.max_rejections },
            &mut stats,
            sink,
        );
        /* stop the workers */
        tasks.lock().unwrap().clear();
        for h in handles.iter_mut() {
            h.discard(&budget);
        }
        r
    });
    stats.decoded_access_units += decoded_by_workers.into_inner();
    stats.digests += digests_by_workers.into_inner();
    if log {
        eprintln!("chunked: {stats:?}");
    }
    result.map(|()| stats)
}

/// The run that currently produces the output.
enum Auth<'s> {
    /// Driven by the coordinator.
    Inline(Box<Run<'s>>),
    /// A speculative run still running on a worker (its events are consumed from its handle).
    Remote(usize),
}

fn map_err<E>(at: usize, e: RunError, frames: u64) -> ChunkedError<E> {
    match e {
        RunError::Setup(s) => ChunkedError::Setup(s),
        RunError::Input(error) => ChunkedError::Input { index: at, error },
        RunError::Process(e) => ChunkedError::Process { index: at, error: Error::from_code(e) },
        RunError::Flush(e) => ChunkedError::Flush(Error::from_code(e)),
        RunError::Output(e) => ChunkedError::Output { frames, error: Error::from_code(e) },
    }
}

/// Settings of the coordinator.
#[derive(Clone, Copy)]
struct Control {
    debug: bool,
    log: bool,
    threads: usize,
    max_rejections: usize,
}

#[allow(clippy::too_many_arguments)]
fn coordinate<'s, E>(
    src: &'s dyn AccessUnitSource,
    setup: &DecoderSetup,
    plan: &Plan,
    handles: &mut [ChunkHandle<'s>],
    budget: &Budget,
    ctl: Control,
    stats: &mut ChunkedStats,
    sink: &mut Sink<E>,
) -> Result<(), ChunkedError<E>> {
    let Control { debug, log, .. } = ctl;
    let n = src.len();
    let chunks = plan.starts.len() - 1;
    /* without chunks, the true run decodes everything with all threads */
    let true_threads = if chunks == 1 { ctl.threads } else { 1 };
    if chunks == 1 {
        stats.sequential_from = Some(0);
    }
    let true_run =
        Run::new(setup, src, 0, Vec::new(), true_threads).map_err(|e| map_err(0, e, 0))?;
    /* consecutive boundaries whose states differ */
    let mut rejections = 0usize;
    /* the true run while a speculative run produces the output (paused where it handed over) */
    let mut paused_true: Option<Box<Run<'s>>> = None;
    let mut auth = Auth::Inline(Box::new(true_run));
    let mut auth_is_true = true;
    /* the output of access units before `valid_from` is not taken from the current run */
    let mut valid_from = 0usize;
    /* the chunk whose range the current run decodes */
    let mut k = 0usize;
    /* the digests of the current run */
    let mut auth_digests: BTreeMap<usize, DigestInfo> = BTreeMap::new();
    let mut sink_err: Option<E> = None;
    let mut frames: u64 = 0;
    /* where the current run stops next: the end of chunk k or a verification point */
    let mut stop = plan.starts[1];

    loop {
        /* 1. the current run decodes up to `stop`; its output is passed on */
        let mut hazard_at: Option<usize> = None;
        let end: RunEnd;
        let mut emit = |f: Frame, frames: &mut u64, sink_err: &mut Option<E>| -> bool {
            if f.tag < valid_from {
                return true;
            }
            *frames += 1;
            if let Err(e) = sink(&f.info.into(), &f.samples) {
                *sink_err = Some(e);
                return false;
            }
            true
        };
        let remote = match &auth {
            Auth::Remote(h) => Some(*h),
            Auth::Inline(_) => None,
        };
        if let Some(h) = remote {
            let mut got: Option<RunEnd> = None;
            let mut paused_run = None;
            while got.is_none() {
                let Some(e) = handles[h].next_event() else {
                    got = Some(RunEnd::Error(
                        valid_from,
                        RunError::Setup("speculative run ended unexpectedly".into()),
                    ));
                    break;
                };
                match e {
                    Event::Frame(f) => {
                        budget.release(h, f.bytes());
                        if !emit(f, &mut frames, &mut sink_err) {
                            got = Some(RunEnd::Stopped);
                        }
                    }
                    Event::Digest(p, d) => {
                        auth_digests.insert(p, d);
                    }
                    Event::Bypassed(at, b) => {
                        if at >= valid_from {
                            stats.fast_bypass_frames += b;
                        }
                    }
                    Event::Hazard(at) => {
                        if at >= valid_from {
                            hazard_at = Some(at);
                            got = Some(RunEnd::Stopped);
                        }
                    }
                    Event::Error(at, e) => got = Some(RunEnd::Error(at, e)),
                    Event::Paused(run) => {
                        budget.release(h, plan.run_cost);
                        paused_run = Some(run);
                        got = Some(RunEnd::Paused);
                    }
                    Event::Finished => got = Some(RunEnd::Finished),
                }
            }
            for (p, d) in std::mem::take(&mut handles[h].digests) {
                auth_digests.insert(p, d);
            }
            end = got.unwrap();
            if let Some(mut run) = paused_run {
                stats.decoded_access_units += std::mem::take(&mut run.decoded);
                stats.digests += std::mem::take(&mut run.digests);
                auth = Auth::Inline(run);
            }
        } else {
            let Auth::Inline(run) = &mut auth else { unreachable!() };
            let digest_at: Vec<usize> = if stop < n { vec![stop] } else { Vec::new() };
            let mut bypassed = 0u64;
            let mut on = |e: Event<'s>| -> bool {
                match e {
                    Event::Frame(f) => emit(f, &mut frames, &mut sink_err),
                    Event::Digest(p, d) => {
                        auth_digests.insert(p, d);
                        true
                    }
                    Event::Bypassed(at, b) => {
                        if at >= valid_from {
                            bypassed += b;
                        }
                        true
                    }
                    Event::Hazard(at) if !auth_is_true && at >= valid_from => {
                        hazard_at = Some(at);
                        false
                    }
                    _ => true,
                }
            };
            end = run.run(src, stop, &digest_at, valid_from, debug, None, &mut on);
            stats.fast_bypass_frames += bypassed;
            stats.decoded_access_units += std::mem::take(&mut run.decoded);
            stats.digests += std::mem::take(&mut run.digests);
        }
        stats.frames = frames;
        if let Some(e) = sink_err.take() {
            return Err(ChunkedError::Sink(e));
        }

        /* 2. a hazard in a speculative run: the true run takes over from there */
        if let Some(at) = hazard_at {
            if let Auth::Remote(h) = auth {
                handles[h].discard(budget);
            }
            let mut t = paused_true.take().expect("paused true run");
            if log {
                eprintln!(
                    "chunked: hazard at access unit {at} (chunk {k}); the true run continues from {}",
                    t.pos
                );
            }
            stats.fallbacks += 1;
            /* decode up to the hazard without output */
            let mut on = |_e: Event<'s>| true;
            match t.run(src, at, &[], n, false, None, &mut on) {
                RunEnd::Paused => {}
                RunEnd::Error(at, e) => return Err(map_err(at, e, frames)),
                _ => unreachable!("the true run ended before the hazard"),
            }
            stats.decoded_access_units += std::mem::take(&mut t.decoded);
            auth = Auth::Inline(t);
            auth_is_true = true;
            valid_from = at;
            auth_digests.clear();
            continue;
        }

        match end {
            RunEnd::Finished | RunEnd::Stopped => return Ok(()),
            RunEnd::Error(at, e) => return Err(map_err(at, e, frames)),
            RunEnd::Paused => {}
        }

        /* 3. at a verification point of the boundary to chunk j */
        let Auth::Inline(run) = &auth else { unreachable!() };
        let p = run.pos;
        let j = k + 1;
        debug_assert!(j < chunks);
        let hj = &mut handles[j];
        let started = match hj.shared.state.compare_exchange(
            NOT_STARTED,
            ABSORBED,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => false,
            Err(s) => s == STARTED,
        };
        let mut handoff = false;
        let mut reason = "not started";
        if started {
            reason = "no state digest";
            /* its state digest is awaited: exempt from the buffer limit */
            hj.shared.exempt.store(true, Ordering::Release);
            budget.wake();
            if let (Some(mine), Some(theirs)) = (auth_digests.get(&p), hj.wait_digest(p)) {
                reason = "the states differ";
                if mine.digest == theirs.digest {
                    handoff = true;
                } else if let (Some(a), Some(b)) = (&mine.trace, &theirs.trace) {
                    eprintln!("chunked: the states differ before access unit {p} (chunk {j}):");
                    for f in diff_traces(a, b) {
                        eprintln!("  {f}");
                    }
                }
            }
        }
        if handoff {
            rejections = 0;
            if log {
                eprintln!("chunked: chunk {j} takes over at access unit {p}");
            }
            stats.handoffs += 1;
            let Auth::Inline(run) = std::mem::replace(&mut auth, Auth::Remote(j)) else {
                unreachable!()
            };
            if auth_is_true {
                paused_true = Some(run);
            }
            auth_is_true = false;
            valid_from = p;
            k = j;
            auth_digests.clear();
            hj.shared.exempt.store(true, Ordering::Release);
            budget.wake();
            stop = plan.starts[k + 1];
            continue;
        }
        let next = plan.candidates(j).into_iter().find(|&c| c > p);
        if let Some(next) = next.filter(|&c| started && !hj.ended_before(c)) {
            /* verify again at the next verification point */
            stop = next;
            continue;
        }
        /* the current run continues through chunk j */
        if log {
            let why = match hj.stash.iter().find_map(|e| match e {
                Event::Error(at, e) => Some((at, e)),
                _ => None,
            }) {
                Some((at, e)) => format!("{reason}: {e:?} at access unit {at}"),
                None => reason.to_string(),
            };
            eprintln!(
                "chunked: chunk {j} rejected at access unit {p} ({why}); the current run continues"
            );
        }
        stats.rejected += 1;
        hj.discard(budget);
        k = j;
        stop = plan.starts[k + 1];
        if reason == "the states differ" {
            rejections += 1;
        }
        if auth_is_true && ctl.max_rejections > 0 && rejections >= ctl.max_rejections {
            /* the states do not become equal: the true run decodes the rest with all threads
            (it is never compared again, so it may use the frame pipelining) */
            if log {
                eprintln!(
                    "chunked: {rejections} boundaries rejected; sequential decoding from access \
                     unit {p} with {} threads",
                    ctl.threads
                );
            }
            for h in handles[j + 1..].iter_mut() {
                let _ = h.shared.state.compare_exchange(
                    NOT_STARTED,
                    ABSORBED,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                );
                h.discard(budget);
            }
            stats.rejected += chunks - 1 - j;
            let Auth::Inline(run) = &mut auth else { unreachable!() };
            let _ = run.dec.set_num_threads(ctl.threads);
            stats.sequential_from = Some(p);
            k = chunks - 1;
            stop = n;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_mhas(path: &str) -> (Vec<u8>, Vec<(usize, usize, u64)>) {
        let data = std::fs::read(path).unwrap();
        let pts = std::fs::read_to_string(path.replace(".mhas", ".pts.txt")).unwrap();
        let v = pts
            .lines()
            .filter_map(|l| {
                let v: Vec<u64> = l.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                (v.len() >= 3).then(|| (v[0] as usize, v[1] as usize, v[2]))
            })
            .collect();
        (data, v)
    }

    /// Diagnostic (ignored): the fields in which the state of a decoder that decoded `0..b` differs from
    /// the state of a decoder that decoded from a random access point `k` access units before
    /// (environment `PROBE=file.mhas,tl,b,k[,depth]`).
    #[test]
    #[ignore]
    fn probe_state_difference() {
        let Ok(spec) = std::env::var("PROBE") else { return };
        let a: Vec<&str> = spec.split(',').collect();
        let (data, idx) = read_mhas(a[0]);
        let tl: i32 = a[1].parse().unwrap();
        let b: usize = a[2].parse().unwrap();
        let k: usize = a[3].parse().unwrap();
        let depth: usize = a.get(4).map_or(12, |x| x.parse().unwrap());
        let aus: Vec<AccessUnit> =
            idx.iter().map(|&(o, n, p)| AccessUnit { data: &data[o..o + n], pts: p }).collect();
        let src = MemorySource { access_units: &aus, timescale: 90000 };
        let w = (0..=b.saturating_sub(k)).rev().find(|&i| has_config(aus[i].data)).unwrap_or(0);
        eprintln!("boundary {b}, warm-up start {w}");
        let setup = DecoderSetup { cicp_setup: tl, ..Default::default() };
        let run = |from: usize| {
            let mut r = Run::new(&setup, &src, from, Vec::new(), 1).ok().unwrap();
            let mut on = |_e: Event| true;
            let _ = r.run(&src, b, &[], usize::MAX, false, None, &mut on);
            r
        };
        let mut s = run(0);
        let mut c = run(w);
        let (ds, ts) = s.dec.state_trace(depth);
        let (dc, tc) = c.dec.state_trace(depth);
        eprintln!("digest seq {ds} chunk {dc} equal {}", ds == dc);
        for p in diff_traces(&ts, &tc) {
            eprintln!("  differs: {p}");
        }
    }
}
