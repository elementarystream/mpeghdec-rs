// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! The targets: one call program of the `mpeghdecoder_*()` API (and, in `ui_decode`, of the
//! `mpegh_UI_*()` API in front of it), run on the reference and on the port.
//!
//! # Input format
//!
//! ```text
//! byte 0    bits 0-1 threads of the port (0, 1: one thread, 2: two (frame pipelining), 3: four
//!           (pipelining and thread pool)), bit 2 processTimescale (else process with
//!           nanoseconds), bit 3 flushAndGet at the end, bits 4-7 target layout CICPS[n]
//! byte 1    number of op bytes L
//! byte 2    bits 0-3 timescale TIMESCALES[n], bits 4-7 frame duration FRAMES[n]
//! L bytes   ops (see below), executed in order; then one access unit per step until the input
//!           is used
//! decode_mha1 only: 16-bit little-endian length C, C bytes of mpegh3daConfig() for
//!           setMhaConfig (before the first access unit)
//! ui_decode only: 16 bytes, the UUID of the generated XML actions
//! rest      the access units, each a 16-bit little-endian length and the bytes
//! ```
//!
//! # Ops
//!
//! `op >> 5` selects the kind, `op & 31` is its argument:
//!
//! | kind | effect |
//! |---|---|
//! | 0 | process `1 + (arg & 15)` access units; `arg & 16`: without getSamples after them (one thread only) |
//! | 1 | timestamps: 0-7 jitter by `-4..4` 32nds of a frame, 8 the next access unit repeats the previous timestamp, 9 / 10 gap of 10 / 50 frames, 11 back 10 frames, 12 jump by a quarter of the range, 13 back to 0, 14 frame duration `FRAMES[next & 15]`, 15 duration 0, 16-31 `+ (arg - 16)` ticks |
//! | 2 | setParam(`(arg & 7) - 1`, `param_value(next)`) (parameters -1, 5 and 6 are invalid) |
//! | 3 | flush (seek): 0 only the flush, 1-15 and go back `arg` access units, 16-31 and skip `arg - 16` |
//! | 4 | flushAndGet; with `arg & 1` the next access units are processed after it as before |
//! | 5 | getSamples buffer size `OUT_SIZES[arg]` (one thread only; else always the full size) |
//! | 6 | 0 process 0 bytes; 1-15 process the access unit in two calls, split at `arg / 16` of its length; 16 process it twice; 17 skip it; 18 process it and the next one in one call; 19 process it without its last `next` bytes; 20 setMhaConfig(the next `next` op bytes); 21 setMhaConfig(the configuration of the input); 22 toggle getSamples after process calls (one thread only); 23 one getSamples call; 24 getSamples until FEED_DATA; else process one access unit |
//! | 7 | `ui_decode` only (else process one access unit): see [`Driver::ui_op`] |
//!
//! After every call the trace holds the return value; after getSamples also the output info and
//! (with one thread) the whole output buffer. With frame pipelining (more than one thread) the
//! port returns output frames one process() call later than the reference, so the frames
//! are compared as a sequence, getSamples is called until it returns FEED_DATA after every
//! process, flush and flushAndGet call (the condition under which the port returns the same
//! sequence, PORTING.md, "MpeghDecoderContext"), and the output buffer always has the full size. At the
//! end of a run the port may still hold the frames that the reference returned after its last
//! process, flush or flushAndGet call (it returns them with the next one); they are not compared.

use crate::guard::{Abort, RefBuf, Session};
use crate::{Blob, Outcome, Reader, Trace};
use mpeghdec_core::mpeghdecoder::{
    MpeghDecoderContext as MpeghDecoder, MpeghDecoderError, MAX_NUM_FRAME_SAMPLES,
    MAX_NUM_OUTPUT_CHANNELS,
};
use mpeghdec_cref::core_frame as c;
use mpeghdec_cref::uimanager as cu;
use mpeghdec_uimanager::mpegh_ui_manager::{MpeghUiManager, MPEGH_UI_INCOMPLETE_XML};

/// The targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// MHAS access units with in-band configuration (`mhm1`).
    Mhas,
    /// Raw access units with an out-of-band configuration (`mha1`, setMhaConfig).
    Mha1,
    /// The UI manager (feed, XML actions, persistence, update) in front of the decoder.
    Ui,
}

pub const TARGETS: [Target; 3] = [Target::Mhas, Target::Mha1, Target::Ui];

impl Target {
    pub fn name(self) -> &'static str {
        match self {
            Target::Mhas => "decode_mhas",
            Target::Mha1 => "decode_mha1",
            Target::Ui => "ui_decode",
        }
    }

    pub fn from_name(s: &str) -> Option<Target> {
        TARGETS.into_iter().find(|t| t.name() == s)
    }
}

pub const CICPS: [i32; 16] = [2, 6, 1, 12, 13, 19, 3, 5, 7, 9, 10, 14, 16, 17, 20, 15];
pub const TIMESCALES: [u32; 16] = [
    48000,
    90000,
    44100,
    1_000_000_000,
    32000,
    1000,
    1,
    24000,
    96000,
    22050,
    16000,
    8000,
    27_000_000,
    600,
    12345,
    0x7FFF_FFFF,
];
/// Frame durations (samples, sample rate) of the timestamps.
pub const FRAMES: [(u64, u64); 16] = [
    (1024, 48000),
    (1024, 48000),
    (1024, 44100),
    (1024, 32000),
    (2048, 48000),
    (512, 48000),
    (1024, 96000),
    (768, 48000),
    (4096, 48000),
    (1024, 24000),
    (1020, 48000),
    (1028, 48000),
    (1024, 47999),
    (1, 1),
    (1, 1000),
    (10, 1),
];
const FULL_OUT: usize = MAX_NUM_OUTPUT_CHANNELS * MAX_NUM_FRAME_SAMPLES;
pub const OUT_SIZES: [usize; 32] = [
    FULL_OUT,
    FULL_OUT,
    FULL_OUT + 1,
    FULL_OUT - 1,
    2 * 3072,
    2 * 3072 - 1,
    6 * 3072,
    6 * 3072 - 1,
    12 * 3072,
    24 * 3072 - 1,
    8 * 3072,
    10 * 3072,
    13 * 3072,
    16 * 3072,
    3072,
    3071,
    1,
    0,
    2 * 1024,
    6 * 1024,
    4 * 3072,
    5 * 3072,
    7 * 3072,
    9 * 3072,
    11 * 3072,
    14 * 3072,
    15 * 3072,
    17 * 3072,
    18 * 3072,
    20 * 3072,
    22 * 3072,
    23 * 3072,
];

/// A setParam value from one byte: the valid ranges and around, then edge values.
pub fn param_value(b: u8) -> i32 {
    const EDGE: [i32; 16] = [
        i32::MAX,
        i32::MIN,
        255,
        256,
        -128,
        -129,
        1000,
        65536,
        -65536,
        231,
        232,
        39,
        40,
        41,
        126,
        128,
    ];
    match b {
        0..=0xCF => b as i32 - 8,
        0xD0..=0xDF => EDGE[(b - 0xD0) as usize],
        _ => -((b - 0xDF) as i32) * 2,
    }
}

// ---- XML actions (ui_decode) ----

const ACTION_TYPES: [i32; 32] = [
    0, 10, 11, 12, 20, 21, 30, 31, 40, 41, 42, 43, 44, 60, 61, 62, 63, 64, 65, 70, 71, 90, 95, 100,
    30, 40, 60, 10, 1, 50, 255, -1,
];
const PARAM_INTS: [i32; 16] =
    [0, 1, 2, 3, 4, 5, -1, 255, 256, 1000, i32::MAX, i32::MIN, 0x0054_3261, 6, 7, 31];
const PARAM_FLOATS: [&str; 16] = [
    "0",
    "1",
    "-1",
    "0.5",
    "-12.75",
    "6",
    "-6",
    "3.5",
    "123456.78901",
    "-70",
    "-5.5",
    "0.25",
    "1e3",
    "abc",
    "",
    "99999999999",
];
const PARAM_BOOLS: [&str; 8] = ["true", "false", "1", "0", "tru", "", "False", "-1"];
const PARAM_TEXTS: [&str; 8] = ["eng", "kor", "deu", "", "e", "engl", "fre", "spa"];
const VERSIONS: [&str; 8] = ["11.0", "11.1", "11.1", "10.0", "9", "11.10", "abc", "+11.1"];

/// An XML action event from four op bytes and the UUID of the input.
pub fn gen_action(uuid: &[u8; 16], b: [u8; 4]) -> Vec<u8> {
    let action_type = ACTION_TYPES[(b[0] & 31) as usize];
    let int_v = if b[1] & 0x80 != 0 { PARAM_INTS[(b[1] & 15) as usize] } else { b[1] as i32 };
    let float_v = PARAM_FLOATS[(b[2] & 15) as usize];
    let mask = b[3];
    let mut s = String::new();
    if b[0] & 0x80 != 0 {
        s.push_str("<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n");
    }
    s.push_str("<ActionEvent");
    if mask & 1 == 0 {
        s.push_str(" uuid=\"");
        for (i, x) in uuid.iter().enumerate() {
            if i == 4 || i == 6 || i == 8 || i == 10 {
                s.push('-');
            }
            if b[0] & 0x40 != 0 {
                s.push_str(&format!("{x:02x}"));
            } else {
                s.push_str(&format!("{x:02X}"));
            }
        }
        s.push('"');
    }
    s.push_str(&format!(" version=\"{}\"", VERSIONS[((b[2] >> 4) & 7) as usize]));
    s.push_str(&format!(" actionType=\"{action_type}\""));
    if mask & 2 == 0 {
        s.push_str(&format!(" paramInt=\"{int_v}\""));
    }
    if mask & 4 != 0 {
        s.push_str(&format!(" paramFloat=\"{float_v}\""));
    }
    if mask & 8 != 0 {
        s.push_str(&format!(" paramBool=\"{}\"", PARAM_BOOLS[((mask >> 4) & 7) as usize]));
    }
    if mask & 0x80 != 0 {
        s.push_str(&format!(" paramText=\"{}\"", PARAM_TEXTS[((b[1] >> 4) & 7) as usize]));
    }
    s.push_str(if b[2] & 0x80 != 0 { " >" } else { " />" });
    s.into_bytes()
}

pub const XML_SIZES: [usize; 16] =
    [65536, 65536, 1024, 4096, 0, 1, 10, 100, 200, 400, 999, 1023, 2000, 3000, 16384, 64];
const XML_FLAGS: [u32; 4] = [0, 1, 4, 5];
/// Persistence memory sizes (2 to 9 bytes are undefined behavior in C).
pub const PERSIST_SIZES: [usize; 16] =
    [512, 1024, 2048, 700, 0, 1, 10, 11, 400, 439, 440, 441, 442, 480, 600, 65535];
pub const UPDATE_EXTRA: [usize; 8] = [4096, 4096, 0, 1, 16, 64, 256, 65536];

const MAX_PROCESS: usize = 192;
const MAX_STEPS: usize = 4096;
/// Upper bound of the getSamples calls in one loop (never reached by valid behavior: the
/// timestamp queue holds 10 entries).
const GET_LOOP_MAX: usize = 64;

/// The program of one input.
#[derive(Clone, Debug)]
pub struct Program {
    pub target: Target,
    pub threads: usize,
    pub timescale_mode: bool,
    pub flush_at_end: bool,
    pub cicp: i32,
    pub timescale: u32,
    pub frame: (u64, u64),
    pub ops: Vec<u8>,
    pub config: Option<Vec<u8>>,
    pub uuid: [u8; 16],
    pub aus: Vec<Vec<u8>>,
}

impl Program {
    pub fn parse(t: Target, data: &[u8]) -> Program {
        let mut r = Reader::new(data);
        let flags = r.u8();
        let nops = r.u8() as usize;
        let aux = r.u8();
        let ops = r.bytes(nops).to_vec();
        let config = if t == Target::Mha1 {
            let n = r.u16() as usize;
            Some(r.bytes(n).to_vec())
        } else {
            None
        };
        let mut uuid = [0u8; 16];
        if t == Target::Ui {
            let u = r.bytes(16);
            uuid[..u.len()].copy_from_slice(u);
        }
        let mut aus = Vec::new();
        while !r.at_end() {
            let n = r.u16() as usize;
            aus.push(r.bytes(n).to_vec());
        }
        Program {
            target: t,
            threads: [1, 1, 2, 4][(flags & 3) as usize],
            timescale_mode: flags & 4 != 0,
            flush_at_end: flags & 8 != 0,
            cicp: CICPS[(flags >> 4) as usize],
            timescale: TIMESCALES[(aux & 15) as usize],
            frame: FRAMES[(aux >> 4) as usize],
            ops,
            config,
            uuid,
            aus,
        }
    }

    /// Serializes a program (seed generation) with the header bytes `flags` and `aux`. `ops` is
    /// limited to 255 bytes, the configuration and the access units to 65535 bytes.
    pub fn to_bytes(&self, flags: u8, aux: u8) -> Vec<u8> {
        let mut v = vec![flags, self.ops.len().min(255) as u8, aux];
        v.extend_from_slice(&self.ops[..self.ops.len().min(255)]);
        if self.target == Target::Mha1 {
            let c = self.config.as_deref().unwrap_or(&[]);
            let n = c.len().min(65535);
            v.extend_from_slice(&(n as u16).to_le_bytes());
            v.extend_from_slice(&c[..n]);
        }
        if self.target == Target::Ui {
            v.extend_from_slice(&self.uuid);
        }
        for au in &self.aus {
            let n = au.len().min(65535);
            v.extend_from_slice(&(n as u16).to_le_bytes());
            v.extend_from_slice(&au[..n]);
        }
        v
    }

    /// Frames are compared as a sequence (frame pipelining of the port).
    pub fn pipelined(&self) -> bool {
        self.threads > 1
    }
}

/// One call and what it returned.
#[derive(Clone, Debug, PartialEq)]
pub enum Ev {
    Init {
        cicp: i32,
        ok: bool,
    },
    SetMhaConfig {
        len: usize,
        ret: i32,
    },
    SetParam {
        param: i32,
        value: i32,
        ret: i32,
    },
    Process {
        au: usize,
        len: usize,
        ts: u64,
        ret: i32,
    },
    /// One getSamples call (one thread): the info and timestamps if it returned a frame, the
    /// whole output buffer.
    GetSamples {
        size: usize,
        ret: i32,
        info: Option<([i32; 5], [u64; 2])>,
        out: Blob,
    },
    /// getSamples until FEED_DATA (or an error) with frame pipelining: the last return value.
    GetLoop {
        ret: i32,
    },
    Flush {
        ret: i32,
    },
    FlushAndGet {
        ret: i32,
    },
    UiFeed {
        len: usize,
        ret: i32,
    },
    UiUpdate {
        len: usize,
        cap: usize,
        ret: i32,
        out_len: u32,
        out: Blob,
    },
    UiApply {
        len: usize,
        ret: i32,
        flags: u32,
    },
    UiGetXml {
        size: usize,
        flags_in: u32,
        ret: i32,
        flags_out: u32,
        out: Blob,
    },
    UiSetPersist {
        size: Option<usize>,
        ret: i32,
    },
    UiGetPersist {
        ret: i32,
        size: u16,
        present: bool,
        data: Blob,
    },
}

/// An output frame (frame pipelining): numSamplesPerChannel, numChannels, sampleRate, loudness,
/// isConcealed; ticks, pts; the samples.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub info: [i32; 5],
    pub ts: [u64; 2],
    pub samples: Blob,
}

/// `MPEGH_DECODER_ERROR` of a port result.
fn code<T>(r: &Result<T, MpeghDecoderError>) -> i32 {
    match r {
        Ok(_) => 0,
        Err(e) => *e,
    }
}

/// The API as both sides offer it.
trait Api {
    fn init(&mut self, cicp: i32, threads: usize) -> Result<bool, Abort>;
    fn set_mha_config(&mut self, conf: &[u8]) -> Result<i32, Abort>;
    fn set_param(&mut self, param: i32, value: i32) -> Result<i32, Abort>;
    fn process(&mut self, data: &[u8], ts: u64, timescale: Option<u32>) -> Result<i32, Abort>;
    /// (ret, info, timestamps)
    fn get_samples(&mut self, out: &mut [i32]) -> Result<(i32, [i32; 5], [u64; 2]), Abort>;
    fn flush(&mut self) -> Result<i32, Abort>;
    fn flush_and_get(&mut self) -> Result<i32, Abort>;
    fn ui_open(&mut self) -> Result<(), Abort>;
    fn ui_feed(&mut self, au: &[u8]) -> Result<i32, Abort>;
    /// `buf` holds the access unit in its first `len` bytes; (ret, len).
    fn ui_update(&mut self, buf: &mut [u8], len: u32) -> Result<(i32, u32), Abort>;
    fn ui_apply(&mut self, xml: &[u8]) -> Result<(i32, u32), Abort>;
    fn ui_get_xml(&mut self, out: &mut [u8], flags_in: u32) -> Result<(i32, u32), Abort>;
    fn ui_set_persist(&mut self, mem: Option<&[u8]>) -> Result<i32, Abort>;
    /// (ret, size, present); the block's bytes go to `out` (resized to the size).
    fn ui_get_persist(&mut self, out: &mut Vec<u8>) -> Result<(i32, u16, bool), Abort>;
    fn close(&mut self);
    /// The number of deviations from the C++ code taken so far (`mpeghdec_fdk::deviation`; 0 for
    /// the reference).
    fn deviations(&self) -> u64 {
        0
    }
}

/// An API call of the driver: not made at the call where the port took a deviation from the C++
/// code (`stop_at`, from a run of the port), and on the port the run ends after a call that took
/// one (`hit_at`); both runs then end with [`Abort::Deviation`].
macro_rules! api {
    ($d:ident, $e:expr) => {{
        $d.pre_call()?;
        let r = $e?;
        $d.post_call()?;
        r
    }};
}

/// The reference, through the shims of `crates/cref/shim/core_frame.cpp` and `uimanager.cpp`.
/// Input buffers are copied into the reference arena inside each call ([`RefBuf`]).
struct RefSide {
    s: Session,
    h: *mut (),
    ui: *mut (),
    /// Persistence memory blocks given to the UI manager (alive until it is closed).
    persist: Vec<*mut u8>,
}

impl Api for RefSide {
    fn init(&mut self, cicp: i32, _threads: usize) -> Result<bool, Abort> {
        let mut h = core::ptr::null_mut();
        // SAFETY: shim call.
        self.s.call(|| h = unsafe { c::cref_mpegh_init(cicp) })?;
        self.h = h;
        Ok(!h.is_null())
    }

    fn set_mha_config(&mut self, conf: &[u8]) -> Result<i32, Abort> {
        let h = self.h;
        let mut r = 0;
        self.s.call(|| {
            let b = RefBuf::new(conf, 0);
            // SAFETY: valid handle, buffer of conf.len() bytes.
            r = unsafe { c::cref_mpegh_set_mha_config(h, b.ptr, conf.len() as u32) };
            b.free();
        })?;
        Ok(r)
    }

    fn set_param(&mut self, param: i32, value: i32) -> Result<i32, Abort> {
        let h = self.h;
        let mut r = 0;
        // SAFETY: valid handle.
        self.s.call(|| r = unsafe { c::cref_mpegh_set_param(h, param, value) })?;
        Ok(r)
    }

    fn process(&mut self, data: &[u8], ts: u64, timescale: Option<u32>) -> Result<i32, Abort> {
        let h = self.h;
        let mut r = 0;
        self.s.call(|| {
            let b = RefBuf::new(data, 0);
            let n = data.len() as u32;
            // SAFETY: valid handle, buffer of n bytes.
            r = unsafe {
                match timescale {
                    Some(t) => c::cref_mpegh_process_timescale(h, b.ptr, n, ts, t),
                    None => c::cref_mpegh_process(h, b.ptr, n, ts),
                }
            };
            b.free();
        })?;
        Ok(r)
    }

    fn get_samples(&mut self, out: &mut [i32]) -> Result<(i32, [i32; 5], [u64; 2]), Abort> {
        let (h, p, n) = (self.h, out.as_mut_ptr(), out.len() as u32);
        let mut info = [0i32; 5];
        let mut ts = [0u64; 2];
        let (ip, tp) = (info.as_mut_ptr(), ts.as_mut_ptr());
        let mut r = 0;
        // SAFETY: valid handle, buffer of n samples, 5 ints, 2 timestamps.
        self.s.call(|| r = unsafe { c::cref_mpegh_get_samples(h, p, n, ip, tp) })?;
        Ok((r, info, ts))
    }

    fn flush(&mut self) -> Result<i32, Abort> {
        let h = self.h;
        let mut r = 0;
        // SAFETY: valid handle.
        self.s.call(|| r = unsafe { c::cref_mpegh_flush(h) })?;
        Ok(r)
    }

    fn flush_and_get(&mut self) -> Result<i32, Abort> {
        let h = self.h;
        let mut r = 0;
        // SAFETY: valid handle.
        self.s.call(|| r = unsafe { c::cref_mpegh_flush_and_get(h) })?;
        Ok(r)
    }

    fn ui_open(&mut self) -> Result<(), Abort> {
        let mut h = core::ptr::null_mut();
        // SAFETY: shim call.
        self.s.call(|| h = unsafe { cu::cref_uim_open() })?;
        self.ui = h;
        Ok(())
    }

    fn ui_feed(&mut self, au: &[u8]) -> Result<i32, Abort> {
        let h = self.ui;
        let mut r = 0;
        self.s.call(|| {
            let b = RefBuf::new(au, 0);
            // SAFETY: valid handle, buffer of au.len() bytes.
            r = unsafe { cu::cref_uim_feed(h, b.ptr, au.len() as u32) };
            b.free();
        })?;
        Ok(r)
    }

    fn ui_update(&mut self, buf: &mut [u8], len: u32) -> Result<(i32, u32), Abort> {
        let h = self.ui;
        let (mut r, mut l) = (0, len);
        let lp = &mut l as *mut u32;
        let src = buf.to_vec();
        let (out, cap) = (buf.as_mut_ptr(), buf.len());
        self.s.call(|| {
            let b = RefBuf::new(&src, 0);
            // SAFETY: valid handle, buffer of cap bytes.
            r = unsafe { cu::cref_uim_update(h, b.ptr, cap as u32, lp) };
            // SAFETY: both buffers hold cap bytes.
            unsafe { core::ptr::copy_nonoverlapping(b.ptr, out, cap) };
            b.free();
        })?;
        Ok((r, l))
    }

    fn ui_apply(&mut self, xml: &[u8]) -> Result<(i32, u32), Abort> {
        let h = self.ui;
        let (mut r, mut fl) = (0, 0xdeadu32);
        let fp = &mut fl as *mut u32;
        self.s.call(|| {
            let b = RefBuf::new(xml, 0);
            // SAFETY: valid handle, buffer of xml.len() bytes.
            r = unsafe { cu::cref_uim_applyXml(h, b.ptr as *const i8, xml.len() as u32, fp) };
            b.free();
        })?;
        Ok((r, fl))
    }

    fn ui_get_xml(&mut self, out: &mut [u8], flags_in: u32) -> Result<(i32, u32), Abort> {
        let h = self.ui;
        let (mut r, mut fl) = (0, 0xdeadu32);
        let fp = &mut fl as *mut u32;
        let (op, size) = (out.as_mut_ptr(), out.len());
        let fill = out.first().copied().unwrap_or(0);
        self.s.call(|| {
            let b = RefBuf::new(&[], size.max(1));
            // SAFETY: b holds max(size, 1) bytes.
            unsafe { core::ptr::write_bytes(b.ptr, fill, size.max(1)) };
            // SAFETY: valid handle, buffer of `size` bytes.
            r = unsafe { cu::cref_uim_getXml(h, b.ptr as *mut i8, size as u32, flags_in, fp) };
            // SAFETY: both buffers hold `size` bytes.
            unsafe { core::ptr::copy_nonoverlapping(b.ptr, op, size) };
            b.free();
        })?;
        Ok((r, fl))
    }

    fn ui_set_persist(&mut self, mem: Option<&[u8]>) -> Result<i32, Abort> {
        let h = self.ui;
        let mut r = 0;
        let mut p: *mut u8 = core::ptr::null_mut();
        let pp = &mut p as *mut *mut u8;
        self.s.call(|| {
            // SAFETY: valid handle; the block stays alive until the UI manager is closed.
            r = unsafe {
                match mem {
                    None => cu::cref_uim_setPersist(h, core::ptr::null_mut(), 0),
                    Some(m) => {
                        let b = RefBuf::new(m, 0);
                        *pp = b.ptr;
                        cu::cref_uim_setPersist(h, b.ptr, m.len() as u16)
                    }
                }
            };
        })?;
        if !p.is_null() {
            self.persist.push(p);
        }
        Ok(r)
    }

    fn ui_get_persist(&mut self, out: &mut Vec<u8>) -> Result<(i32, u16, bool), Abort> {
        let h = self.ui;
        let mut r = 0;
        let mut p: *mut u8 = core::ptr::null_mut();
        let mut size = 0u16;
        let (pp, sp) = (&mut p as *mut *mut u8, &mut size as *mut u16);
        // SAFETY: valid handle and pointers.
        self.s.call(|| r = unsafe { cu::cref_uim_getPersist(h, pp, sp) })?;
        out.clear();
        if !p.is_null() {
            out.resize(size as usize, 0);
            let o = out.as_mut_ptr();
            // SAFETY: the reference's block holds `size` bytes.
            self.s.call(|| unsafe { core::ptr::copy_nonoverlapping(p, o, size as usize) })?;
        }
        Ok((r, size, !p.is_null()))
    }

    fn close(&mut self) {
        if !self.ui.is_null() {
            let h = self.ui;
            // SAFETY: valid handle, closed once.
            let _ = self.s.call_unbudgeted(|| unsafe { cu::cref_uim_close(h) });
            self.ui = core::ptr::null_mut();
        }
        for p in self.persist.drain(..) {
            let _ = self.s.call_unbudgeted(|| RefBuf { ptr: p }.free());
        }
        if !self.h.is_null() {
            let h = self.h;
            // SAFETY: valid handle, destroyed once.
            let _ = self.s.call_unbudgeted(|| unsafe { c::cref_mpegh_destroy(h) });
            self.h = core::ptr::null_mut();
        }
    }
}

/// The port (the straight ports of the C APIs: `MpeghDecoderContext`, `MpeghUiManager`).
#[derive(Default)]
struct PortSide {
    d: Option<MpeghDecoder>,
    ui: Option<Box<MpeghUiManager>>,
}

impl PortSide {
    fn d(&mut self) -> &mut MpeghDecoder {
        self.d.as_mut().expect("port handle")
    }

    fn ui(&mut self) -> &mut MpeghUiManager {
        self.ui.as_deref_mut().expect("port UI manager")
    }
}

impl Api for PortSide {
    fn init(&mut self, cicp: i32, threads: usize) -> Result<bool, Abort> {
        /* MPEGHDEC_FUZZ_PORT_THREADS (analysis of a finding): the port's number of threads,
         * without changing the call program */
        let threads = std::env::var("MPEGHDEC_FUZZ_PORT_THREADS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(threads);
        self.d = MpeghDecoder::with_threads(cicp, threads);
        Ok(self.d.is_some())
    }

    fn set_mha_config(&mut self, conf: &[u8]) -> Result<i32, Abort> {
        Ok(code(&self.d().set_mha_config(conf)))
    }

    fn set_param(&mut self, param: i32, value: i32) -> Result<i32, Abort> {
        Ok(code(&self.d().set_param(param, value)))
    }

    fn process(&mut self, data: &[u8], ts: u64, timescale: Option<u32>) -> Result<i32, Abort> {
        let r = match timescale {
            Some(t) => self.d().process_timescale(data, ts, t),
            None => self.d().process(data, ts),
        };
        Ok(code(&r))
    }

    fn get_samples(&mut self, out: &mut [i32]) -> Result<(i32, [i32; 5], [u64; 2]), Abort> {
        Ok(match self.d().get_samples(out) {
            Ok(Some(oi)) => (
                0,
                [
                    oi.num_samples_per_channel,
                    oi.num_channels,
                    oi.sample_rate,
                    oi.loudness,
                    oi.is_concealed as i32,
                ],
                [oi.ticks, oi.pts],
            ),
            /* MPEGH_DEC_FEED_DATA */
            Ok(None) => (1, [0; 5], [0; 2]),
            r => (code(&r), [0; 5], [0; 2]),
        })
    }

    fn flush(&mut self) -> Result<i32, Abort> {
        Ok(code(&self.d().flush()))
    }

    fn flush_and_get(&mut self) -> Result<i32, Abort> {
        Ok(code(&self.d().flush_and_get()))
    }

    fn ui_open(&mut self) -> Result<(), Abort> {
        self.ui = Some(MpeghUiManager::open());
        Ok(())
    }

    fn ui_feed(&mut self, au: &[u8]) -> Result<i32, Abort> {
        Ok(self.ui().feed_mhas(au))
    }

    fn ui_update(&mut self, buf: &mut [u8], len: u32) -> Result<(i32, u32), Abort> {
        let mut l = len;
        let r = self.ui().update_mhas(buf, &mut l);
        Ok((r, l))
    }

    fn ui_apply(&mut self, xml: &[u8]) -> Result<(i32, u32), Abort> {
        let mut fl = 0xdeadu32;
        let r = self.ui().apply_xml_action(xml, &mut fl);
        Ok((r, fl))
    }

    fn ui_get_xml(&mut self, out: &mut [u8], flags_in: u32) -> Result<(i32, u32), Abort> {
        let mut fl = 0xdeadu32;
        let r = self.ui().get_xml_scene_state(out, flags_in, &mut fl);
        Ok((r, fl))
    }

    fn ui_set_persist(&mut self, mem: Option<&[u8]>) -> Result<i32, Abort> {
        Ok(self.ui().set_persistence_memory(mem.map(|m| m.to_vec())))
    }

    fn ui_get_persist(&mut self, out: &mut Vec<u8>) -> Result<(i32, u16, bool), Abort> {
        let (r, mem, size) = self.ui().get_persistence_memory();
        out.clear();
        if let Some(m) = mem {
            out.extend_from_slice(&m[..(size as usize).min(m.len())]);
        }
        Ok((r, size, mem.is_some()))
    }

    fn close(&mut self) {
        self.ui = None;
        self.d = None;
    }

    fn deviations(&self) -> u64 {
        mpeghdec_fdk::deviation::count()
    }
}

/// The getSamples output buffer pattern before each call.
fn out_pattern() -> &'static [i32] {
    static P: std::sync::OnceLock<Vec<i32>> = std::sync::OnceLock::new();
    P.get_or_init(|| {
        (0..OUT_SIZES.iter().copied().max().unwrap() as u32)
            .map(|i| (i.wrapping_mul(2_654_435_761).wrapping_add(0x6A5D_1234) | 1) as i32)
            .collect()
    })
}

/// The op bytes and the position in them.
struct Ops<'b> {
    ops: &'b [u8],
    k: usize,
}

impl<'b> Ops<'b> {
    fn next(&mut self) -> u8 {
        let b = self.ops.get(self.k).copied().unwrap_or(0);
        self.k += 1;
        b
    }

    /// The next `n` op bytes (fewer at the end).
    fn take(&mut self, n: usize) -> &'b [u8] {
        let a = self.k.min(self.ops.len());
        let b = (self.k + n).min(self.ops.len());
        self.k += n;
        &self.ops[a..b]
    }
}

struct Driver<'a, A: Api> {
    api: &'a mut A,
    p: &'a Program,
    verbose: bool,
    trace: Trace<Ev, Frame>,
    out: Vec<i32>,
    out_size: usize,
    /// getSamples until FEED_DATA after process calls
    auto_get: bool,
    /// current access unit
    ai: usize,
    processed: usize,
    /// timestamp of the next access unit (in ticks of the timescale, or nanoseconds), its
    /// maximum, the frame duration
    ts: u64,
    ts_max: u64,
    step: u64,
    prev_ts: u64,
    repeat_ts: bool,
    // ui_decode
    ui_on: bool,
    dec_on: bool,
    apply_now: bool,
    pending_actions: Vec<Vec<u8>>,
    update_extra: usize,
    saved_persist: Vec<u8>,
    xml_buf: Vec<u8>,
    /// API calls made so far, the call not to make, the call at which the port took a deviation
    calls: usize,
    stop_at: Option<usize>,
    hit_at: Option<usize>,
    deviations: u64,
}

impl<A: Api> Driver<'_, A> {
    fn pre_call(&mut self) -> Result<(), Abort> {
        if self.stop_at == Some(self.calls) {
            return Err(Abort::Deviation);
        }
        self.deviations = self.api.deviations();
        Ok(())
    }

    fn post_call(&mut self) -> Result<(), Abort> {
        let k = self.calls;
        self.calls += 1;
        if self.api.deviations() != self.deviations {
            self.hit_at = Some(k);
            return Err(Abort::Deviation);
        }
        Ok(())
    }

    fn push(&mut self, e: Ev) {
        crate::echo(self.trace.events.len(), &e);
        self.trace.events.push(e);
    }

    fn timescale(&self) -> Option<u32> {
        self.p.timescale_mode.then_some(self.p.timescale)
    }

    fn ticks_per_second(&self) -> u64 {
        if self.p.timescale_mode {
            self.p.timescale as u64
        } else {
            1_000_000_000
        }
    }

    fn set_frame(&mut self, f: (u64, u64)) {
        self.step = (f.0 as f64 * self.ticks_per_second() as f64 / f.1 as f64).round() as u64;
    }

    fn move_ts(&mut self, delta: i64) {
        self.ts = (self.ts as i128 + delta as i128).clamp(0, self.ts_max as i128) as u64;
    }

    fn get_loop(&mut self) -> Result<(), Abort> {
        let size = if self.p.pipelined() { FULL_OUT } else { self.out_size };
        let mut ret = 1;
        for _ in 0..GET_LOOP_MAX {
            let (r, frame) = self.get_once(size)?;
            ret = r;
            if let Some(f) = frame {
                crate::echo(
                    self.trace.events.len(),
                    &format_args!("frame {}: {f:?}", self.trace.frames.len()),
                );
                self.trace.frames.push(f);
            }
            if r != 0 {
                break;
            }
        }
        if self.p.pipelined() {
            self.push(Ev::GetLoop { ret });
        }
        Ok(())
    }

    /// One getSamples call; with frame pipelining the frame (if any) is returned, else the call
    /// is recorded.
    fn get_once(&mut self, size: usize) -> Result<(i32, Option<Frame>), Abort> {
        if self.out.len() < size {
            self.out.resize(size, 0);
        }
        self.out[..size].copy_from_slice(&out_pattern()[..size]);
        let (ret, info, ts) = api!(self, self.api.get_samples(&mut self.out[..size]));
        if self.p.pipelined() {
            if ret != 0 {
                return Ok((ret, None));
            }
            let n = (info[0].max(0) as usize * info[1].max(0) as usize).min(size);
            let samples = Blob::from_i32(&self.out[..n], self.verbose);
            return Ok((ret, Some(Frame { info, ts, samples })));
        }
        let out = Blob::from_i32(&self.out[..size], self.verbose);
        self.push(Ev::GetSamples { size, ret, info: (ret == 0).then_some((info, ts)), out });
        Ok((ret, None))
    }

    fn after_call(&mut self) -> Result<(), Abort> {
        if self.auto_get || self.p.pipelined() {
            self.get_loop()?;
        }
        Ok(())
    }

    /// process() of `data` (the access unit `au`) with the current timestamp.
    fn process_data(&mut self, au: usize, data: &[u8], get: bool) -> Result<(), Abort> {
        let ts = if self.repeat_ts { self.prev_ts } else { self.ts };
        self.repeat_ts = false;
        self.trace.frames_floor = self.trace.frames.len();
        let ret = api!(self, self.api.process(data, ts, self.timescale()));
        self.push(Ev::Process { au, len: data.len(), ts, ret });
        self.processed += 1;
        if get || self.p.pipelined() {
            self.after_call()?;
        }
        Ok(())
    }

    fn advance_ts(&mut self) {
        self.prev_ts = self.ts;
        self.move_ts(self.step as i64);
    }

    /// Feeds the access unit through the UI manager (ui_decode), then decodes it.
    fn process_au(&mut self, data: &[u8], get: bool) -> Result<(), Abort> {
        let ai = self.ai;
        let mut data = data.to_vec();
        if self.p.target == Target::Ui && self.ui_on {
            let ret = api!(self, self.api.ui_feed(&data));
            self.push(Ev::UiFeed { len: data.len(), ret });
            for xml in std::mem::take(&mut self.pending_actions) {
                self.apply(&xml)?;
            }
            let cap = data.len() + self.update_extra;
            let mut buf = data.clone();
            buf.resize(cap, 0x5A);
            let (ret, out_len) = api!(self, self.api.ui_update(&mut buf, data.len() as u32));
            let out = Blob::new(&buf, self.verbose);
            self.push(Ev::UiUpdate { len: data.len(), cap, ret, out_len, out });
            if ret == 0 && (out_len as usize) <= cap {
                buf.truncate(out_len as usize);
                data = buf;
            }
        }
        if self.p.target != Target::Ui || self.dec_on {
            if data.is_empty() {
                /* process() rejects empty buffers; nothing to decode */
            } else {
                self.process_data(ai, &data, get)?;
            }
        }
        Ok(())
    }

    /// Processes the next access unit (if any).
    fn next_au(&mut self, get: bool) -> Result<(), Abort> {
        let Some(au) = self.p.aus.get(self.ai) else { return Ok(()) };
        let au = au.clone();
        self.process_au(&au, get)?;
        self.ai += 1;
        self.advance_ts();
        Ok(())
    }

    fn apply(&mut self, xml: &[u8]) -> Result<(), Abort> {
        let (ret, flags) = api!(self, self.api.ui_apply(xml));
        self.push(Ev::UiApply { len: xml.len(), ret, flags });
        Ok(())
    }

    fn action(&mut self, xml: Vec<u8>) -> Result<(), Abort> {
        if self.apply_now {
            self.apply(&xml)
        } else {
            self.pending_actions.push(xml);
            Ok(())
        }
    }

    fn get_xml(&mut self, size: usize, flags_in: u32) -> Result<(i32, u32), Abort> {
        self.xml_buf.clear();
        self.xml_buf.resize(size, 0xCC);
        let mut buf = std::mem::take(&mut self.xml_buf);
        let (ret, flags_out) = api!(self, self.api.ui_get_xml(&mut buf, flags_in));
        let out = Blob::new(&buf, self.verbose);
        self.xml_buf = buf;
        self.push(Ev::UiGetXml { size, flags_in, ret, flags_out, out });
        Ok((ret, flags_out))
    }

    fn set_mha_config(&mut self, conf: &[u8]) -> Result<(), Abort> {
        let ret = api!(self, self.api.set_mha_config(conf));
        self.push(Ev::SetMhaConfig { len: conf.len(), ret });
        self.after_call()
    }

    /// Kind 7 of `ui_decode`:
    ///
    /// | arg | effect |
    /// |---|---|
    /// | 0-7 | an XML action from the next 4 op bytes ([`gen_action`]): applied after the next feed (default) or now |
    /// | 8 | an XML action of the next `next` op bytes |
    /// | 9-12 | getXmlSceneState(size `XML_SIZES[next & 15]`, flags `0, 1, 4, 5`) |
    /// | 13 | getXmlSceneState in pieces of `XML_SIZES[next & 15]` bytes until the XML is complete |
    /// | 14 | setPersistenceMemory(`PERSIST_SIZES[next & 15]` bytes: `next` 0 zeros, 1 the last block of getPersistenceMemory, else filled with it) |
    /// | 15 | setPersistenceMemory(NULL) |
    /// | 16 | getPersistenceMemory |
    /// | 17 | toggle the UI manager processing of the access units (feed, update) |
    /// | 18 | update buffer capacity: the access unit and `UPDATE_EXTRA[next & 7]` bytes |
    /// | 19 | toggle the decoding of the access units |
    /// | 20 | toggle: XML actions now or after the next feed |
    /// | else | process one access unit |
    fn ui_op(&mut self, arg: u8, o: &mut Ops) -> Result<(), Abort> {
        match arg {
            0..=7 => {
                let b = [o.next(), o.next(), o.next(), o.next()];
                let xml = gen_action(&self.p.uuid, b);
                self.action(xml)?;
            }
            8 => {
                let n = o.next() as usize;
                let xml = o.take(n).to_vec();
                self.action(xml)?;
            }
            9..=12 => {
                let size = XML_SIZES[(o.next() & 15) as usize];
                self.get_xml(size, XML_FLAGS[(arg - 9) as usize])?;
            }
            13 => {
                let size = XML_SIZES[(o.next() & 15) as usize];
                let mut flags = 1;
                for _ in 0..64 {
                    let (ret, fl) = self.get_xml(size, flags)?;
                    if ret != 0 || fl & MPEGH_UI_INCOMPLETE_XML == 0 {
                        break;
                    }
                    flags = 0;
                }
            }
            14 => {
                let mut size = PERSIST_SIZES[(o.next() & 15) as usize];
                if (2..=9).contains(&size) {
                    size = 10;
                }
                let fill = o.next();
                let mem = match fill {
                    0 => vec![0u8; size],
                    1 => {
                        let mut m = self.saved_persist.clone();
                        m.resize(size, 0);
                        m
                    }
                    f => vec![f; size],
                };
                let ret = api!(self, self.api.ui_set_persist(Some(&mem)));
                self.push(Ev::UiSetPersist { size: Some(size), ret });
            }
            15 => {
                let ret = api!(self, self.api.ui_set_persist(None));
                self.push(Ev::UiSetPersist { size: None, ret });
            }
            16 => {
                let mut m = Vec::new();
                let (ret, size, present) = api!(self, self.api.ui_get_persist(&mut m));
                let data = Blob::new(&m, self.verbose);
                self.push(Ev::UiGetPersist { ret, size, present, data });
                if present {
                    self.saved_persist = m;
                }
            }
            17 => self.ui_on = !self.ui_on,
            18 => self.update_extra = UPDATE_EXTRA[(o.next() & 7) as usize],
            19 => self.dec_on = !self.dec_on,
            20 => self.apply_now = !self.apply_now,
            _ => self.next_au(true)?,
        }
        Ok(())
    }

    fn run(&mut self) -> Result<(), Abort> {
        let ok = api!(self, self.api.init(self.p.cicp, self.p.threads));
        self.push(Ev::Init { cicp: self.p.cicp, ok });
        if !ok {
            return Ok(());
        }
        if self.p.target == Target::Ui {
            api!(self, self.api.ui_open());
        }
        if let Some(c) = self.p.config.clone() {
            if !c.is_empty() {
                self.set_mha_config(&c)?;
            }
        }
        let p = self.p;
        let mut o = Ops { ops: &p.ops, k: 0 };
        let mut steps = 0usize;
        while steps < MAX_STEPS && self.processed < MAX_PROCESS {
            steps += 1;
            if o.k >= o.ops.len() {
                if self.ai >= p.aus.len() {
                    break;
                }
                self.next_au(true)?;
                continue;
            }
            let op = o.next();
            let arg = op & 31;
            match op >> 5 {
                0 => {
                    let get = arg & 16 == 0;
                    for _ in 0..=(arg & 15) {
                        self.next_au(get)?;
                    }
                }
                1 => {
                    let s = self.step as i64;
                    match arg {
                        0..=7 => self.move_ts((arg as i64 - 4 + (arg >= 4) as i64) * s / 32),
                        8 => self.repeat_ts = true,
                        9 => self.move_ts(10 * s),
                        10 => self.move_ts(50 * s),
                        11 => self.move_ts(-10 * s),
                        12 => self.move_ts((self.ts_max / 4) as i64),
                        13 => self.ts = 0,
                        14 => {
                            let f = FRAMES[(o.next() & 15) as usize];
                            self.set_frame(f);
                        }
                        15 => self.step = 0,
                        _ => self.move_ts(arg as i64 - 16),
                    }
                }
                2 => {
                    let param = (arg & 7) as i32 - 1;
                    let value = param_value(o.next());
                    let ret = api!(self, self.api.set_param(param, value));
                    self.push(Ev::SetParam { param, value, ret });
                }
                3 => {
                    self.trace.frames_floor = self.trace.frames.len();
                    let ret = api!(self, self.api.flush());
                    self.push(Ev::Flush { ret });
                    self.after_call()?;
                    if (1..16).contains(&arg) {
                        self.ai = self.ai.saturating_sub(arg as usize);
                    } else if arg >= 16 {
                        self.ai += (arg - 16) as usize;
                    }
                }
                4 => {
                    self.trace.frames_floor = self.trace.frames.len();
                    let ret = api!(self, self.api.flush_and_get());
                    self.push(Ev::FlushAndGet { ret });
                    self.after_call()?;
                }
                5 => {
                    if !self.p.pipelined() {
                        self.out_size = OUT_SIZES[arg as usize];
                    }
                }
                6 => match arg {
                    0 => self.process_data(self.ai, &[], true)?,
                    1..=15 => {
                        if let Some(au) = self.p.aus.get(self.ai) {
                            let au = au.clone();
                            let cut = au.len() * arg as usize / 16;
                            let (a, b) = au.split_at(cut);
                            self.process_au(a, false)?;
                            self.process_au(b, true)?;
                            self.ai += 1;
                            self.advance_ts();
                        }
                    }
                    16 => {
                        if let Some(au) = self.p.aus.get(self.ai) {
                            let au = au.clone();
                            self.process_au(&au, true)?;
                            self.advance_ts();
                        }
                        self.next_au(true)?;
                    }
                    17 => self.ai += 1,
                    18 => {
                        let mut au = self.p.aus.get(self.ai).cloned().unwrap_or_default();
                        au.extend_from_slice(self.p.aus.get(self.ai + 1).map_or(&[], |v| v));
                        self.process_au(&au, true)?;
                        self.ai += 2;
                        self.advance_ts();
                    }
                    19 => {
                        let n = o.next() as usize;
                        if let Some(au) = self.p.aus.get(self.ai) {
                            let au = au[..au.len().saturating_sub(n)].to_vec();
                            self.process_au(&au, true)?;
                            self.ai += 1;
                            self.advance_ts();
                        }
                    }
                    20 => {
                        let n = o.next() as usize;
                        let conf = o.take(n);
                        self.set_mha_config(conf)?;
                    }
                    21 => {
                        let c = self.p.config.clone().unwrap_or_default();
                        self.set_mha_config(&c)?;
                    }
                    22 => {
                        if !self.p.pipelined() {
                            self.auto_get = !self.auto_get;
                        }
                    }
                    23 => {
                        if !self.p.pipelined() {
                            self.get_once(self.out_size)?;
                        }
                    }
                    24 => self.get_loop()?,
                    _ => self.next_au(true)?,
                },
                _ => {
                    if self.p.target == Target::Ui {
                        self.ui_op(arg, &mut o)?;
                    } else {
                        self.next_au(true)?;
                    }
                }
            }
        }
        if self.p.flush_at_end {
            self.trace.frames_floor = self.trace.frames.len();
            let ret = api!(self, self.api.flush_and_get());
            self.push(Ev::FlushAndGet { ret });
            self.after_call()?;
        }
        Ok(())
    }
}

/// Runs the program; `stop_at`: the API call not to make (see [`api!`]). Returns the trace and
/// the call at which the port took a deviation.
fn run_api<A: Api>(
    api: &mut A,
    p: &Program,
    verbose: bool,
    stop_at: Option<usize>,
) -> (Trace<Ev, Frame>, Option<usize>) {
    let ts_max = if p.timescale_mode {
        /* the timestamp in nanoseconds must stay below 2^63 (the conversion to uint64_t) */
        (((1u64 << 62) as f64 * p.timescale as f64 / 1e9) as u64).min(1 << 62)
    } else {
        1u64 << 62
    };
    let mut d = Driver {
        api,
        p,
        verbose,
        trace: Trace::default(),
        out: Vec::new(),
        out_size: FULL_OUT,
        auto_get: true,
        ai: 0,
        processed: 0,
        ts: 0,
        ts_max,
        step: 0,
        prev_ts: 0,
        repeat_ts: false,
        ui_on: true,
        dec_on: true,
        apply_now: false,
        pending_actions: Vec::new(),
        update_extra: UPDATE_EXTRA[0],
        saved_persist: Vec::new(),
        xml_buf: Vec::new(),
        calls: 0,
        stop_at,
        hit_at: None,
        deviations: 0,
    };
    d.set_frame(p.frame);
    let r = d.run();
    let mut trace = std::mem::take(&mut d.trace);
    if let Err(a) = r {
        trace.abort = Some(a);
    }
    // Also after an aborted reference call: destroying the handle frees the memory (a crash
    // there is caught as well, and the handle is then leaked).
    d.api.close();
    (trace, d.hit_at)
}

/// Runs the program on the reference with the given memory pattern, up to the API call
/// `stop_at` (exclusive).
pub fn run_ref(
    p: &Program,
    pattern: i32,
    verbose: bool,
    stop_at: Option<usize>,
) -> Trace<Ev, Frame> {
    let mut api = RefSide {
        s: Session::begin(pattern),
        h: core::ptr::null_mut(),
        ui: core::ptr::null_mut(),
        persist: Vec::new(),
    };
    run_api(&mut api, p, verbose, stop_at).0
}

/// Runs the program on the port; also returns the API call at which it took a deviation from
/// the C++ code (the run ends after it).
pub fn run_port(p: &Program, verbose: bool) -> (Trace<Ev, Frame>, Option<usize>) {
    run_api(&mut PortSide::default(), p, verbose, None)
}

fn detail_ev(a: &Ev, b: &Ev) -> String {
    match (a, b) {
        (Ev::GetSamples { out: x, .. }, Ev::GetSamples { out: y, .. }) => {
            crate::diff_bytes("output samples", x, y, 4)
        }
        (Ev::UiUpdate { out: x, .. }, Ev::UiUpdate { out: y, .. }) => {
            crate::diff_bytes("updated MHAS bytes", x, y, 1)
        }
        (Ev::UiGetXml { out: x, .. }, Ev::UiGetXml { out: y, .. }) => {
            let mut s = crate::diff_bytes("XML bytes", x, y, 1);
            if let (Some(x), Some(y)) = (&x.bytes, &y.bytes) {
                if let Some(i) = x.iter().zip(y).position(|(a, b)| a != b) {
                    let a = i.saturating_sub(120);
                    s += &format!(
                        "\n  reference: {}\n  port:      {}",
                        String::from_utf8_lossy(&x[a..(i + 40).min(x.len())]),
                        String::from_utf8_lossy(&y[a..(i + 40).min(y.len())])
                    );
                }
            }
            s
        }
        (Ev::UiGetPersist { data: x, .. }, Ev::UiGetPersist { data: y, .. }) => {
            crate::diff_bytes("persistence bytes", x, y, 1)
        }
        _ => String::new(),
    }
}

fn detail_fr(a: &Frame, b: &Frame) -> String {
    crate::diff_bytes("output samples", &a.samples, &b.samples, 4)
}

/// Compares the reference and the port on one input.
pub fn check(t: Target, data: &[u8]) -> Outcome {
    let p = Program::parse(t, data);
    let describe = format!(
        "cicp {}, {} thread(s), {}, frame {:?}, {} access unit(s) / {} bytes, {} op bytes{}",
        p.cicp,
        p.threads,
        if p.timescale_mode {
            format!("processTimescale({})", p.timescale)
        } else {
            "process (ns)".to_string()
        },
        p.frame,
        p.aus.len(),
        p.aus.iter().map(|c| c.len()).sum::<usize>(),
        p.ops.len(),
        p.config.as_ref().map_or(String::new(), |c| format!(", config of {} bytes", c.len()))
    );
    /* the port runs first: the reference stops before the call where the port deviates */
    let stop = std::cell::Cell::new(None);
    crate::compare(
        t.name(),
        &describe,
        |pat, v| run_ref(&p, pat, v, stop.get()),
        |v| {
            let (tr, hit) = run_port(&p, v);
            stop.set(hit);
            tr
        },
        detail_ev,
        detail_fr,
    )
}

/// The fuzz target body.
pub fn fuzz(t: Target, data: &[u8]) {
    crate::begin_input(t.name(), data);
    crate::handle(t.name(), check(t, data));
}
