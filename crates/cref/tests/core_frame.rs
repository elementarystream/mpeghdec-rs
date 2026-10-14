// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of the libMpeghDec frame level: the `aacDecoder_*()` API (configuration,
//! access unit decoding, rendering, truncation, earcons, UI and DRC interfaces, limiter) and the
//! `mpeghdecoder_*()` API (timestamp handling, fades, restarts), decoding the test streams in C
//! and Rust with random decoder events (concealment, flushing, interruptions, corrupted and lost
//! access units, parameter changes, timestamp jitter and gaps, seeking, user interaction
//! packets inserted by the MPEG-H UI manager), and `PcmDataPayload()` with random earcon
//! configurations.
//!
//! The streams are taken from the test material in `work/` (see README.md, "Development");
//! tests without material are skipped. `MPEGHDEC_WORK_DIR` overrides the location of `work/` and
//! `MPEGHDEC_TEST_STREAMS` the one of `work/streams`.
#![cfg(not(cref_missing))]

#[allow(dead_code)]
#[path = "../../cli/src/mhas.rs"]
mod mhas;
#[allow(dead_code)]
#[path = "../../cli/src/mp4.rs"]
mod mp4;
#[allow(dead_code)]
#[path = "../../cli/src/reader.rs"]
mod reader;

use mpeghdec::FastOptions;
use mpeghdec_core::aacdecoder::*;
use mpeghdec_core::aacdecoder_lib::*;
use mpeghdec_core::mpeghdecoder::{MpeghDecoderContext as MpeghDecoder, MpeghDecoderError};
use mpeghdec_core::render::DecodeStatus;
use mpeghdec_cref::core_frame::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::{TT_MHAS_PACKETIZED, TT_MHA_RAW};
use mpeghdec_tpdec::EARCON_BUFFER_SIZE;
use mpeghdec_uimanager::mpegh_ui_manager::*;
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------------------------
// Test material
// ---------------------------------------------------------------------------------------------

fn work_dir() -> Option<PathBuf> {
    let dir = std::env::var_os("MPEGHDEC_WORK_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../work"));
    dir.is_dir().then_some(dir)
}

/// The access units of a stream with their presentation timestamps.
struct Stream {
    name: String,
    aus: Vec<Vec<u8>>,
    pts: Vec<u64>,
    timescale: u32,
    /// `mpegh3daConfig()` of an `mha1` track
    mha_config: Option<Vec<u8>>,
}

fn load_mp4(path: &Path) -> Option<Stream> {
    let f = mp4::Mp4File::open(path.to_str()?).ok()?;
    let t = f.tracks.iter().find(|t| t.codec != mp4::Codec::Other)?;
    Some(Stream {
        name: path.file_name()?.to_string_lossy().into_owned(),
        aus: t.samples.iter().map(|s| f.read_sample(s).unwrap()).collect(),
        pts: t.samples.iter().map(|s| s.pts()).collect(),
        timescale: t.timescale,
        mha_config: if t.codec == mp4::Codec::MpeghMha { t.mha_config.clone() } else { None },
    })
}

/// A raw MHAS stream with its `.pts.txt` side file (90 kHz timestamps).
fn load_mhas(base: &Path) -> Option<Stream> {
    let base = base.to_str()?;
    let data = std::fs::read(format!("{base}.mhas")).ok()?;
    let text = std::fs::read_to_string(format!("{base}.pts.txt")).ok()?;
    let mut aus = Vec::new();
    let mut pts = Vec::new();
    for line in text.lines() {
        let v: Vec<u64> = line.split_whitespace().filter_map(|x| x.parse().ok()).collect();
        if v.len() >= 3 {
            aus.push(data[v[0] as usize..(v[0] + v[1]) as usize].to_vec());
            pts.push(v[2]);
        }
    }
    Some(Stream {
        name: base.rsplit('/').next()?.to_string(),
        aus,
        pts,
        timescale: 90000,
        mha_config: None,
    })
}

fn vectors(names: &[&str]) -> Vec<Stream> {
    let Some(w) = work_dir() else { return Vec::new() };
    names
        .iter()
        .filter_map(|n| load_mp4(&w.join("vectors").join(n).join(format!("{n}.mp4"))))
        .collect()
}

fn test_streams(names: &[&str]) -> Vec<Stream> {
    let dir = match std::env::var_os("MPEGHDEC_TEST_STREAMS") {
        Some(d) => PathBuf::from(d),
        None => match work_dir() {
            Some(w) => w.join("streams"),
            None => return Vec::new(),
        },
    };
    names.iter().filter_map(|n| load_mhas(&dir.join(n))).collect()
}

// ---------------------------------------------------------------------------------------------
// aacDecoder_*() API
// ---------------------------------------------------------------------------------------------

thread_local! {
    /// Count of the `aacDecoder_DecodeFrame()` return values and of the frames with buffered earcon
    /// samples (key -1), with pending truncations or cross-fades (key -2) and of the frames whose
    /// rendering the pipelined decoder deferred (key -3) (coverage information).
    static STATS: std::cell::RefCell<std::collections::BTreeMap<i32, u32>> = Default::default();
}

fn print_stats(test: &str) {
    STATS.with(|st| {
        let v: Vec<String> = st.borrow().iter().map(|(k, n)| format!("{k:#x}: {n}")).collect();
        println!("{test}: decode results {}", v.join(", "));
    });
}

struct CAacDec(*mut ());

impl Drop for CAacDec {
    fn drop(&mut self) {
        unsafe { cref_aacdec_close(self.0) }
    }
}

const OUT_SIZE: usize = 24 * 3072;

/// `CStreamInfo` in the order of `cref_aacdec_stream_info()`.
fn stream_info_r(si: &CStreamInfo) -> [i64; 24] {
    [
        si.sample_rate as i64,
        si.sample_rate_before_rs as i64,
        si.frame_size as i64,
        si.mpegh_au_size as i64,
        si.num_channels as i64,
        si.num_channels_before_mix as i64,
        si.aac_sample_rate as i64,
        si.aot as i64,
        si.channel_config as i64,
        si.bit_rate as i64,
        si.aac_samples_per_frame as i64,
        si.aac_num_channels as i64,
        si.output_delay as i64,
        si.flags as i64,
        si.ep_config as i64,
        si.reference_layout as i64,
        si.num_lost_access_units as i64,
        si.num_total_bytes,
        si.num_bad_bytes,
        si.num_total_access_units,
        si.num_bad_access_units,
        si.drc_prog_ref_lev as i64,
        si.output_loudness as i64,
        si.f_bs_restart_ok as i64,
    ]
}

/// Internal state in the order of `cref_aacdec_state()`.
fn state_r(d: &AacDecoderInstance) -> [i32; 32] {
    let rs = d.render.as_deref().expect("render state");
    [
        d.aac_channels,
        d.asc_channels[0],
        d.frame_ok as i32,
        d.flags[0] as i32,
        d.flush_status as i32,
        d.flush_cnt as i32,
        d.build_up_status as i32,
        d.build_up_cnt as i32,
        d.has_audio_pre_roll as i32,
        rs.apply_crossfade as i32,
        rs.truncate_start_offset[0] as i32,
        rs.truncate_start_offset[1] as i32,
        rs.truncate_start_offset[2] as i32,
        rs.truncate_stop_offset[0] as i32,
        rs.truncate_stop_offset[1] as i32,
        rs.truncate_stop_offset[2] as i32,
        d.truncate_frame_size as i32,
        rs.truncate_sample_count as i32,
        rs.discard_samples_at_start_cnt,
        d.target_layout,
        d.target_layout_config,
        d.ui_manager_enabled as i32,
        d.ui_manager_active as i32,
        d.ui_status_valid as i32,
        d.drc_status_valid as i32,
        d.ui_signal_changed as i32,
        d.use_element_skipping as i32,
        d.limiter_enable_user as i32,
        d.limiter_enable_curr as i32,
        d.multiband_drc_present as i32,
        d.aac_channels_prev,
        d.downmix_id,
    ]
}

/// The output of an `aacDecoder_DecodeFrame()` call: error, `sampleRate`, `frameSize`,
/// `numChannels`, `mpeghAUSize`, `outputLoudness` (for valid output) and the samples.
type AacOutput = (i32, [i32; 5], Vec<i32>);

fn aac_output(err: i32, si: &CStreamInfo, pcm: &[i32]) -> AacOutput {
    if !is_output_valid(err) {
        return (err, [0; 5], Vec::new());
    }
    let n = (si.frame_size * si.num_channels).max(0) as usize;
    (
        err,
        [si.sample_rate, si.frame_size, si.num_channels, si.mpegh_au_size, si.output_loudness],
        pcm[..n].to_vec(),
    )
}

/// A Rust decoder with frame pipelining (render thread and thread pool) fed with the same data
/// as the pair; its output is compared with the one of the sequential Rust decoder.
struct Pipelined {
    d: Box<AacDecoderInstance>,
    out: Vec<i32>,
    /// Outputs of the sequential decoder not yet compared.
    expect: std::collections::VecDeque<AacOutput>,
    deferred: usize,
}

impl Pipelined {
    fn new(fmt: i32) -> Pipelined {
        let mut d = AacDecoderInstance::aac_decoder_open(fmt, 1).unwrap();
        d.set_render_worker(Some(std::sync::Arc::new(
            mpeghdec_core::pipeline::RenderWorker::new().unwrap(),
        )));
        d.set_thread_pool(Some(std::sync::Arc::new(
            mpeghdec_fdk::thread_pool::ThreadPool::new(3).unwrap(),
        )));
        Pipelined { d, out: vec![0; OUT_SIZE], expect: Default::default(), deferred: 0 }
    }

    fn compare(&self, ctx: &str, got: AacOutput, exp: AacOutput) {
        assert_eq!((got.0, got.1), (exp.0, exp.1), "{ctx}: pipelined decoder: result");
        if got.2 != exp.2 {
            let i = got.2.iter().zip(&exp.2).position(|(a, b)| a != b).unwrap_or(0);
            panic!("{ctx}: pipelined decoder: output differs at sample {i}");
        }
    }

    /// Compares the frames whose rendering has finished.
    fn take_completed(&mut self, ctx: &str) {
        while let Some(c) = self.d.take_completed() {
            let exp = self.expect.pop_front().expect("pipelined output");
            self.compare(ctx, aac_output(c.err, &c.stream_info, &c.pcm), exp);
            self.d.recycle_pcm(c.pcm);
        }
    }

    /// `aacDecoder_DecodeFrame()` with frame pipelining; `exp` is the output of the sequential
    /// decoder.
    fn decode(&mut self, ctx: &str, flags: u32, exp: AacOutput) {
        let st = self.d.aac_decoder_decode_frame_pipelined(&mut self.out, flags);
        self.take_completed(ctx);
        match st {
            DecodeStatus::Done(err) => {
                let si = *self.d.aac_decoder_get_stream_info();
                if is_output_valid(err) {
                    assert!(self.expect.is_empty(), "{ctx}: pipelined decoder: order");
                }
                self.compare(ctx, aac_output(err, &si, &self.out), exp);
            }
            DecodeStatus::Deferred(d) => {
                assert_eq!(
                    (d.err, d.sample_rate, d.num_channels),
                    (exp.0, exp.1[0], exp.1[2]),
                    "{ctx}: pipelined decoder: deferred frame"
                );
                self.expect.push_back(exp);
                self.deferred += 1;
            }
        }
    }

    fn finish(&mut self, ctx: &str) {
        self.d.join_render();
        self.take_completed(ctx);
        assert!(self.expect.is_empty(), "{ctx}: pipelined decoder: missing output");
        assert_eq!(self.d.pipeline_mismatches, 0, "{ctx}: pipelined decoder: parse mismatches");
    }
}

/// Fast mode of the Rust decoders of a test (not bit-exact, see [`FastCtl`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fast {
    /// Bit-exact decoding (the default).
    Off,
    /// Fast mode for the whole stream.
    On,
    /// The fast mode is switched on and off at random frames.
    Toggle,
}

/// Maximum deviation of the output of the fast mode from the reference: 1024 LSB of 24 bit
/// (the output samples have 24 significant bits in the upper bits).
const FAST_TOL: i64 = 1024 << 8;

/// Fast mode control of a test: the Rust decoders decode with [`FastOptions::ALL`] (switched
/// on and off at random frames with [`Fast::Toggle`]). Their output must be bit-exact until the
/// first frame the format converter bypass is applied, and within [`FAST_TOL`] of the
/// reference afterwards.
struct FastCtl {
    mode: Fast,
    g: Rng,
    on: bool,
    /// Largest deviation from the reference.
    max_diff: i64,
    /// Number of bypassed frames.
    bypassed: u64,
}

impl FastCtl {
    fn new(mode: Fast, seed: u64) -> Option<FastCtl> {
        (mode != Fast::Off).then(|| FastCtl {
            mode,
            g: Rng::new(seed ^ 0xfa57),
            on: true,
            max_diff: 0,
            bypassed: 0,
        })
    }

    fn options(&self) -> FastOptions {
        if self.on {
            FastOptions::ALL
        } else {
            FastOptions::EXACT
        }
    }

    /// Switches the fast mode at random (with [`Fast::Toggle`]); returns whether it changed.
    fn toggle(&mut self) -> bool {
        if self.mode == Fast::Toggle && self.g.range(0, 7) == 0 {
            self.on = !self.on;
            return true;
        }
        false
    }

    /// Compares the output of the Rust decoder with the one of the reference decoder.
    /// `bypassed`: frames bypassed so far by the Rust decoder.
    fn compare(&mut self, ctx: &str, r: &[i32], c: &[i32], bypassed: u64) {
        self.bypassed = bypassed;
        if r == c {
            return;
        }
        let (i, d) = r
            .iter()
            .zip(c)
            .map(|(&a, &b)| (a as i64 - b as i64).abs())
            .enumerate()
            .max_by_key(|x| x.1)
            .unwrap();
        assert!(bypassed > 0, "{ctx}: fast mode: output differs before the first bypass");
        assert!(d <= FAST_TOL, "{ctx}: fast mode: output differs by {d} at sample {i}");
        self.max_diff = self.max_diff.max(d);
    }
}

thread_local! {
    /// Fast mode tests: largest deviation from the reference, bypassed frames, all frames.
    static FAST_STATS: std::cell::RefCell<(i64, u64, u64)> = Default::default();
}

/// A pair of decoders (C and Rust) fed with the same data (and a pipelined Rust decoder).
struct AacPair {
    c: CAacDec,
    r: Box<AacDecoderInstance>,
    out_c: Vec<i32>,
    out_r: Vec<i32>,
    ctx: String,
    frames: usize,
    pl: Pipelined,
    fast: Option<FastCtl>,
}

impl AacPair {
    fn new(fmt: i32, ctx: &str) -> AacPair {
        let c = CAacDec(unsafe { cref_aacdec_open(fmt, 1) });
        assert!(!c.0.is_null());
        let r = AacDecoderInstance::aac_decoder_open(fmt, 1).unwrap();
        AacPair {
            c,
            r,
            out_c: vec![0; OUT_SIZE],
            out_r: vec![0; OUT_SIZE],
            ctx: ctx.to_string(),
            frames: 0,
            pl: Pipelined::new(fmt),
            fast: None,
        }
    }

    /// Enables the fast mode test (see [`FastCtl`]).
    fn set_fast(&mut self, mode: Fast, seed: u64) {
        self.fast = FastCtl::new(mode, seed);
        if let Some(f) = &self.fast {
            self.r.set_fast_options(f.options());
            self.pl.d.set_fast_options(f.options());
        }
    }

    fn check_state(&self, what: &str) {
        let mut sc = [0i64; 24];
        let mut stc = [0i32; 32];
        unsafe {
            cref_aacdec_stream_info(self.c.0, sc.as_mut_ptr());
            cref_aacdec_state(self.c.0, stc.as_mut_ptr());
        }
        assert_eq!(
            stream_info_r(self.r.aac_decoder_get_stream_info()),
            sc,
            "{} {what}: stream info",
            self.ctx
        );
        assert_eq!(state_r(&self.r), stc, "{} {what}: decoder state", self.ctx);
    }

    fn set_param(&mut self, param: i32, value: i32) {
        let ec = unsafe { cref_aacdec_set_param(self.c.0, param, value) };
        let er = self.r.aac_decoder_set_param(param, value);
        assert_eq!(er, ec, "{} set_param({param:#x}, {value})", self.ctx);
        let ep = self.pl.d.aac_decoder_set_param(param, value);
        assert_eq!(ep, er, "{} set_param({param:#x}, {value}): pipelined decoder", self.ctx);
    }

    fn config_raw(&mut self, conf: &[u8]) {
        let mut cc = conf.to_vec();
        let ec = unsafe { cref_aacdec_config_raw(self.c.0, cc.as_mut_ptr(), cc.len() as u32) };
        let er = self.r.aac_decoder_config_raw(&[conf]);
        assert_eq!(er, ec, "{} config_raw", self.ctx);
        assert_eq!(self.pl.d.aac_decoder_config_raw(&[conf]), er, "{} config_raw", self.ctx);
        self.check_state("config_raw");
    }

    /// `aacDecoder_Fill()` with `valid` bytes of `data` left; returns the bytes left.
    fn fill(&mut self, data: &[u8], valid: u32) -> u32 {
        let mut vc = valid;
        let mut vr = valid;
        let ec = unsafe { cref_aacdec_fill(self.c.0, data.as_ptr(), data.len() as u32, &mut vc) };
        let er = self.r.aac_decoder_fill(&[data], core::slice::from_mut(&mut vr));
        assert_eq!((er, vr), (ec, vc), "{} fill", self.ctx);
        let mut vp = valid;
        let ep = self.pl.d.aac_decoder_fill(&[data], core::slice::from_mut(&mut vp));
        assert_eq!((ep, vp), (er, vr), "{} fill: pipelined decoder", self.ctx);
        vr
    }

    fn decode(&mut self, flags: u32) -> AacDecoderError {
        if let Some(f) = self.fast.as_mut() {
            if f.toggle() {
                self.r.set_fast_options(f.options());
                self.pl.d.set_fast_options(f.options());
            }
        }
        let ec = unsafe {
            cref_aacdec_decode_frame(self.c.0, self.out_c.as_mut_ptr(), OUT_SIZE as i32, flags)
        };
        let er = self.r.aac_decoder_decode_frame(&mut self.out_r, flags);
        let what = format!("frame {} decode(flags {flags})", self.frames);
        assert_eq!(er, ec, "{} {what}: error", self.ctx);
        self.check_state(&what);
        if let Some(f) = self.fast.as_mut() {
            let bypassed = self.r.fast_bypass_frames();
            f.compare(&format!("{} {what}", self.ctx), &self.out_r, &self.out_c, bypassed);
        } else if self.out_r != self.out_c {
            let i = self.out_r.iter().zip(&self.out_c).position(|(a, b)| a != b).unwrap();
            let si = self.r.aac_decoder_get_stream_info();
            panic!(
                "{} {what}: output differs at sample {i} (channel {} of {}, rust {} c {})",
                self.ctx,
                i % si.num_channels.max(1) as usize,
                si.num_channels,
                self.out_r[i],
                self.out_c[i]
            );
        }
        let exp = aac_output(er, self.r.aac_decoder_get_stream_info(), &self.out_r);
        self.pl.decode(&format!("{} {what}", self.ctx), flags, exp);
        self.frames += 1;
        STATS.with(|st| *st.borrow_mut().entry(er).or_insert(0) += 1);
        let rs = self.r.render.as_deref().expect("render state");
        if rs.earcon_decoder.accumulated_frame_size > 0 {
            /* frames with earcon samples to mix (coverage information) */
            STATS.with(|st| *st.borrow_mut().entry(-1).or_insert(0) += 1);
        }
        if rs.truncate_stop_offset[0] != -128 || rs.apply_crossfade != 0 {
            /* frames with pending truncations or cross-fades (coverage information) */
            STATS.with(|st| *st.borrow_mut().entry(-2).or_insert(0) += 1);
        }
        er
    }

    /// Decodes all frames available after feeding an access unit.
    fn decode_all(&mut self, mut flags: u32) {
        for _ in 0..6 {
            let err = self.decode(flags);
            flags = 0;
            if err == AAC_DEC_NOT_ENOUGH_BITS
                || !(is_output_valid(err) || err == AAC_DEC_INTERMEDIATE_OK)
            {
                break;
            }
        }
    }
}

/// Random decoder parameter changes (valid and invalid values).
fn random_param(g: &mut Rng, p: &mut AacPair) {
    let (param, value) = match g.range(0, 9) {
        0 => (AAC_DRC_REFERENCE_LEVEL, g.range(30, 130)),
        1 => (AAC_UNIDRC_SET_EFFECT, g.range(-2, 7)),
        2 => (AAC_DRC_BOOST_FACTOR, g.range(-1, 128)),
        3 => (AAC_DRC_ATTENUATION_FACTOR, g.range(-1, 128)),
        4 => (AAC_UNIDRC_ALBUM_MODE, g.range(0, 1)),
        5 => (AAC_PCM_LIMITER_ENABLE, g.range(-1, 1)),
        6 => (AAC_CONCEAL_METHOD, g.range(0, 2)),
        7 => (AAC_TARGET_LAYOUT_CICP, [2, 6, -1][g.range(0, 2) as usize]),
        8 => (AAC_PCM_LIMITER_ATTACK_TIME, g.range(0, 20)),
        _ => (AAC_PCM_LIMITER_RELEAS_TIME, g.range(0, 200)),
    };
    p.set_param(param, value);
}

/// Decodes `max_aus` access units of `s` with C and Rust (target layout `tl`), with random
/// events if `events` is set.
fn run_aac(s: &Stream, tl: i32, seed: u64, max_aus: usize, events: bool, params: &[(i32, i32)]) {
    run_aac_fast(s, tl, seed, max_aus, events, params, Fast::Off);
}

/// [`run_aac`] with the fast mode `fast` of the Rust decoders.
fn run_aac_fast(
    s: &Stream,
    tl: i32,
    seed: u64,
    max_aus: usize,
    events: bool,
    params: &[(i32, i32)],
    fast: Fast,
) {
    let mut g = Rng::new(seed);
    let fmt = if s.mha_config.is_some() { TT_MHA_RAW } else { TT_MHAS_PACKETIZED };
    let mut p = AacPair::new(fmt, &format!("{} tl {tl} seed {seed} fast {fast:?}", s.name));
    p.set_fast(fast, seed);
    p.set_param(AAC_TARGET_LAYOUT_CICP, tl);
    for &(param, value) in params {
        p.set_param(param, value);
    }
    if let Some(c) = &s.mha_config {
        p.config_raw(c);
    }
    for (i, au) in s.aus.iter().take(max_aus).enumerate() {
        let mut flags = 0;
        let mut data = au.clone();
        if events {
            match g.range(0, 60) {
                0 => {
                    /* lost access unit: conceal instead */
                    p.decode_all(AACDEC_CONCEAL);
                    continue;
                }
                1 => flags = AACDEC_INTR,
                2 => flags = AACDEC_CLRHIST,
                3 => {
                    /* corrupt some bytes */
                    for _ in 0..g.range(1, 4) {
                        let k = g.range(0, data.len() as i32 - 1) as usize;
                        data[k] ^= 1 << g.range(0, 7);
                    }
                }
                4 => {
                    /* truncated access unit */
                    data.truncate(g.range(1, data.len() as i32) as usize);
                }
                5 => random_param(&mut g, &mut p),
                6 if i > 10 => {
                    p.decode_all(AACDEC_FLUSH);
                }
                _ => {}
            }
        }
        /* like mpeghdecoder_process(): feed until the access unit is consumed */
        let mut valid = data.len() as u32;
        for _ in 0..3 {
            valid = p.fill(&data, valid);
            p.decode_all(flags);
            flags = 0;
            if valid == 0 {
                break;
            }
        }
    }
    /* flush the decoder at the end */
    p.decode_all(AACDEC_FLUSH);
    p.decode_all(AACDEC_FLUSH);
    let ctx = p.ctx.clone();
    p.pl.finish(&ctx);
    STATS.with(|st| *st.borrow_mut().entry(-3).or_insert(0) += p.pl.deferred as u32);
    if let Some(f) = &p.fast {
        FAST_STATS.with(|st| {
            let mut st = st.borrow_mut();
            st.0 = st.0.max(f.max_diff);
            st.1 += f.bypassed;
            st.2 += p.frames as u64;
        });
    }
}

const VECTORS: [&str; 36] = [
    "c22_2_1024k",
    "c3_0_128k",
    "c5_0_192k",
    "c5_1_160k",
    "c5_1_256k",
    "c5_1_2_384k",
    "c5_1_4_448k",
    "c5_1_all_tools_128k",
    "c5_1_mct_160k",
    "c7_1_320k",
    "c7_1_4_512k",
    "c7_1_4_mct_320k",
    "c7_1_front_320k",
    "c9_1_4_640k",
    "mono_32k",
    "mono_64k",
    "mono_ltpf_fdp_48k",
    "obj4_256k",
    "stereo_128k",
    "stereo_128k_mha1",
    "stereo_256k",
    "stereo_64k",
    "stereo_96k_44k",
    "stereo_cplx_96k",
    "stereo_fdp_96k",
    "stereo_kernel_128k",
    "stereo_ltpf_64k",
    "stereo_nf_96k",
    "stereo_64k_32k",
    "stereo_48k_24k",
    "mono_32k_16k",
    "c5_1_128k_32k",
    "stereo_64k_22k",
    "stereo_64k_29k",
    "stereo_48k_14k",
    "obj4_128k_24k",
];

const STREAMS: [&str; 5] = ["s1_clip_a", "s3_dvs_b", "s4_dvs_a", "s5_short_a", "s2_short_a"];

/// All vectors without events, for all target layouts including -1 (no rendering) and 0
/// (reference layout).
#[test]
fn aacdec_vectors_all_layouts() {
    for s in vectors(&VECTORS) {
        for tl in [-1, 0, 1, 2, 6, 12, 13, 19] {
            run_aac(&s, tl, 1, 200, false, &[]);
        }
    }
    print_stats("aacdec_vectors_all_layouts");
}

/// All vectors with random decoder events and parameters.
#[test]
fn aacdec_vectors_events() {
    for (k, s) in vectors(&VECTORS).iter().enumerate() {
        for (j, tl) in [2, 6, -1].into_iter().enumerate() {
            run_aac(s, tl, 100 + (k * 3 + j) as u64, 400, true, &[]);
        }
    }
    print_stats("aacdec_vectors_events");
}

/// The test streams (in-band configs with SYNC packets, streams starting before the first
/// config, DRC and loudness metadata, audio scene information) with and without events.
#[test]
fn aacdec_streams() {
    for (k, s) in test_streams(&STREAMS).iter().enumerate() {
        run_aac(s, 2, 1, 600, false, &[]);
        run_aac(s, 6, 2, 300, false, &[(AAC_DRC_REFERENCE_LEVEL, 64), (AAC_UNIDRC_SET_EFFECT, 6)]);
        run_aac(s, 2, 3, 300, false, &[(AAC_PCM_LIMITER_ENABLE, 0), (AAC_DRC_BOOST_FACTOR, 20)]);
        for seed in 0..3 {
            run_aac(s, [2, 6, 12][seed], 200 + (k * 3 + seed) as u64, 600, true, &[]);
        }
    }
    print_stats("aacdec_streams");
}

/// Streams spliced from segments of different streams (config changes with flushing, build up,
/// implicit truncation and cross-fade).
fn spliced(streams: &[Stream], seed: u64, n: usize) -> Stream {
    let mut g = Rng::new(seed);
    let mut aus = Vec::new();
    let mut pts = Vec::new();
    let mut names = Vec::new();
    let mut t = 0u64;
    let cand: Vec<&Stream> = streams.iter().filter(|s| s.mha_config.is_none()).collect();
    for _ in 0..n {
        let s = cand[g.range(0, cand.len() as i32 - 1) as usize];
        /* start at a sync sample (the first AU of the stream or a config repetition) */
        let start = if g.range(0, 1) == 0 {
            0
        } else {
            g.range(0, (s.aus.len() / 48) as i32) as usize * 48
        };
        let len = g.range(5, 80) as usize;
        for i in start..(start + len).min(s.aus.len()) {
            aus.push(s.aus[i].clone());
            pts.push(t);
            t += 1024 * s.timescale as u64 / 48000;
        }
        names.push(format!("{}@{start}", s.name));
    }
    Stream {
        name: format!("splice {seed} [{}]", names.join(" ")),
        aus,
        pts,
        timescale: 48000,
        mha_config: None,
    }
}

#[test]
fn aacdec_config_changes() {
    let mut streams = test_streams(&STREAMS);
    streams.extend(vectors(&VECTORS));
    if streams.is_empty() {
        return;
    }
    for seed in 0..12u64 {
        let s = spliced(&streams, 500 + seed, 6);
        run_aac(&s, [2, 6, 12, -1][seed as usize % 4], seed, 2000, seed >= 6, &[]);
        run_mpegh(
            &s,
            [2, 6][seed as usize % 2],
            seed,
            2000,
            seed >= 6,
            [1, 2, 8][seed as usize % 3],
        );
    }
    print_stats("aacdec_config_changes");
}

// ---------------------------------------------------------------------------------------------
// User interaction (MHAS packets inserted by the MPEG-H UI manager)
// ---------------------------------------------------------------------------------------------

thread_local! {
    /// Count of rejected / accepted XML actions.
    static UI_STATS: std::cell::RefCell<[u32; 2]> = Default::default();
}

fn random_action(g: &mut Rng) -> String {
    let (t, attrs) = match g.range(0, 11) {
        0 => (10, format!("paramInt=\"{}\"", g.range(-1, 6))),
        1 => (11, format!("paramFloat=\"{}\"", g.range(0, 100) as f32 / 100.0)),
        2 => (12, format!("paramFloat=\"{}\"", g.range(0, 100) as f32 / 100.0)),
        3 => (20, format!("paramFloat=\"{}\"", -(g.range(40, 127) as f32) / 4.0)),
        4 => (21, format!("paramBool=\"{}\"", g.range(0, 1) == 1)),
        5 => (30, format!("paramInt=\"{}\"", g.range(0, 4))),
        6 => (40, format!("paramInt=\"{}\" paramBool=\"{}\"", g.range(0, 6), g.range(0, 1) == 1)),
        7 => (41, format!("paramInt=\"{}\" paramFloat=\"{}\"", g.range(0, 6), g.range(-10, 10))),
        8 => (42, format!("paramInt=\"{}\" paramFloat=\"{}\"", g.range(0, 6), g.range(-90, 90))),
        9 => (60, format!("paramInt=\"{}\" paramInt2=\"{}\"", g.range(0, 3), g.range(0, 6))),
        10 => (70, "paramText=\"kor\"".to_string()),
        _ => (100, String::new()),
    };
    format!("<ActionEvent uuid=\"00000000-0000-0000-0000-000000000000\" version=\"11.0\" actionType=\"{t}\" {attrs} />")
}

/// Applies random XML actions to a stream with the (Rust) MPEG-H UI manager, which inserts user
/// interaction and loudness/DRC packets into the MHAS stream.
fn with_ui_packets(s: &Stream, seed: u64) -> Stream {
    let mut g = Rng::new(seed);
    let mut ui = MpeghUiManager::open();
    let mut aus = Vec::with_capacity(s.aus.len());
    for au in &s.aus {
        let mut buf = au.clone();
        let len = buf.len() as u32;
        let feed = ui.feed_mhas(&buf);
        if g.range(0, 8) == 0 {
            let mut fl = 0;
            let e = ui.apply_xml_action(random_action(&mut g).as_bytes(), &mut fl);
            UI_STATS.with(|st| st.borrow_mut()[(e == MPEGH_UI_OK) as usize] += 1);
        }
        if feed == MPEGH_UI_OK {
            buf.resize(65536, 0);
            let mut new_len = len;
            if ui.update_mhas(&mut buf, &mut new_len) == MPEGH_UI_OK {
                buf.truncate(new_len as usize);
            } else {
                buf.truncate(len as usize);
            }
        }
        aus.push(buf);
    }
    Stream {
        name: format!("{} (ui {seed})", s.name),
        aus,
        pts: s.pts.clone(),
        timescale: s.timescale,
        mha_config: None,
    }
}

#[test]
fn aacdec_user_interaction() {
    let mut streams = test_streams(&STREAMS);
    streams.extend(vectors(&["obj4_256k", "c5_1_160k"]));
    for (k, s) in streams.iter().enumerate() {
        for seed in 0..2u64 {
            let u = with_ui_packets(s, 1000 + k as u64 * 10 + seed);
            run_aac(&u, [2, 6][seed as usize], seed, 800, false, &[]);
        }
    }
    print_stats("aacdec_user_interaction");
    UI_STATS.with(|st| println!("xml actions rejected/accepted: {:?}", st.borrow()));
}

// ---------------------------------------------------------------------------------------------
// Earcons (PCM data packets)
// ---------------------------------------------------------------------------------------------

/// MSB first bit writer.
#[derive(Default)]
struct BitWriter {
    v: Vec<u8>,
    n: usize,
}

impl BitWriter {
    fn put(&mut self, val: u32, bits: u32) {
        for i in (0..bits).rev() {
            if self.n % 8 == 0 {
                self.v.push(0);
            }
            if (val >> i) & 1 != 0 {
                *self.v.last_mut().unwrap() |= 0x80 >> (self.n % 8);
            }
            self.n += 1;
        }
    }

    /// `escapedValue(n1, n2, n3)`
    fn escaped(&mut self, v: u32, n1: u32, n2: u32, n3: u32) {
        let m1 = (1 << n1) - 1;
        if v < m1 {
            self.put(v, n1);
            return;
        }
        self.put(m1, n1);
        let v2 = v - m1;
        let m2 = (1 << n2) - 1;
        if v2 < m2 {
            self.put(v2, n2);
        } else {
            self.put(m2, n2);
            self.put(v2 - m2, n3);
        }
    }
}

/// One MHAS packet (`MHASPacketType`, `MHASPacketLabel`, `MHASPacketLength`, payload).
fn mhas_packet(packet_type: u32, label: u32, payload: &[u8]) -> Vec<u8> {
    let mut w = BitWriter::default();
    w.escaped(packet_type, 3, 8, 8);
    w.escaped(label, 2, 8, 32);
    w.escaped(payload.len() as u32, 11, 24, 24);
    let mut v = w.v;
    v.extend_from_slice(payload);
    v
}

/// Inserts `PACTYP_PCMCONFIG` / `PACTYP_PCMDATA` packets (earcons with random signals, frame
/// sizes, loudness and attenuation gains, switched on and off) before the frame packets.
fn with_earcons(s: &Stream, seed: u64) -> Stream {
    let mut g = Rng::new(seed);
    let mut aus = Vec::with_capacity(s.aus.len());
    let mut active = 0;
    let mut num_signals = 1u32;
    let mut has_att = 0u32;
    let mut loudness = 0u32;
    for au in &s.aus {
        if active == 0 && g.range(0, 15) == 0 {
            active = g.range(1, 40);
            num_signals = g.range(1, 2) as u32;
            has_att = g.range(0, 2) as u32;
            loudness = g.range(0, 255) as u32;
        }
        let mut out = Vec::with_capacity(au.len() + 4200);
        let mut off = 0;
        while off < au.len() {
            let Some(h) = mhas::parse_packet_header(&au[off..]) else { break };
            let end = (off + h.header_bytes + h.length as usize).min(au.len());
            if h.packet_type == mhas::PACTYP_MPEGH3DAFRAME && active > 0 {
                if g.range(0, 4) == 0 {
                    loudness = g.range(0, 255) as u32;
                }
                /* pcmDataConfig() */
                let mut w = BitWriter::default();
                w.put(num_signals - 1, 7); /* numPcmSignals */
                w.put(0, 1); /* pcmAlignAudioFlag */
                w.put(3, 5); /* pcmSamplingRateIndex (48 kHz) */
                w.put(1, 3); /* pcmBitsPerSampleIndex */
                w.put(6, 3); /* pcmFrameSizeIndex: variable frame size */
                for i in 0..num_signals {
                    w.put(i, 7); /* pcmSignal_ID */
                }
                w.put(loudness, 8); /* bsPcmLoudnessValue */
                w.put(has_att, 2); /* pcmHasAttenuationGain */
                if has_att == 1 {
                    w.put([0, 20, 100, 255][g.range(0, 3) as usize], 8);
                }
                out.extend_from_slice(&mhas_packet(20, 0, &w.v));
                /* pcmDataPayload() */
                let mut w = BitWriter::default();
                w.put(num_signals - 1, 7); /* numPcmSignalsInFrame */
                for i in 0..num_signals {
                    w.put(i, 7); /* pcmSignal_ID */
                }
                if has_att == 2 {
                    w.put(g.range(0, 255) as u32, 8); /* bsPcmAttenuationGain */
                }
                let frame_size = if g.range(0, 3) == 0 { g.range(0, 1024) } else { 1024 } as u32;
                w.put(frame_size, 16);
                for _ in 0..frame_size * num_signals {
                    w.put(g.i16() as u16 as u32, 16);
                }
                out.extend_from_slice(&mhas_packet(21, 0, &w.v));
                active -= 1;
            }
            out.extend_from_slice(&au[off..end]);
            off = end;
        }
        aus.push(out);
    }
    Stream {
        name: format!("{} (earcons {seed})", s.name),
        aus,
        pts: s.pts.clone(),
        timescale: s.timescale,
        mha_config: None,
    }
}

/// Inserts random `PACTYP_AUDIOTRUNCATION` packets (left and right truncations of the main
/// stream, earcon truncations) before the frame packets.
fn with_truncations(s: &Stream, seed: u64, earcons: bool) -> Stream {
    let base = if earcons {
        with_earcons(s, seed)
    } else {
        Stream {
            name: s.name.clone(),
            aus: s.aus.clone(),
            pts: s.pts.clone(),
            timescale: s.timescale,
            mha_config: None,
        }
    };
    let mut g = Rng::new(seed ^ 0x5a5a);
    let mut aus = Vec::with_capacity(base.aus.len());
    for au in &base.aus {
        let mut out = Vec::with_capacity(au.len() + 16);
        let mut off = 0;
        while off < au.len() {
            let Some(h) = mhas::parse_packet_header(&au[off..]) else { break };
            let end = (off + h.header_bytes + h.length as usize).min(au.len());
            if h.packet_type == mhas::PACTYP_MPEGH3DAFRAME && g.range(0, 25) == 0 {
                /* audioTruncationInfo() */
                let mut w = BitWriter::default();
                let from_begin = g.range(0, 1) as u32;
                w.put(1, 1); /* isActive */
                w.put(0, 1); /* reserved */
                w.put(from_begin, 1); /* truncFromBegin */
                w.put([0, 1, 100, 512, 775, 1000, 1023, 1024][g.range(0, 7) as usize], 13);
                let label = if g.range(0, 4) == 0 { 2049 } else { h.label };
                out.extend_from_slice(&mhas_packet(17, label, &w.v));
            }
            out.extend_from_slice(&au[off..end]);
            off = end;
        }
        aus.push(out);
    }
    Stream {
        name: format!("{} (truncations {seed})", base.name),
        aus,
        pts: base.pts.clone(),
        timescale: base.timescale,
        mha_config: None,
    }
}

#[test]
fn aacdec_truncations() {
    let mut streams = test_streams(&STREAMS[..3]);
    streams.extend(vectors(&["stereo_128k", "c5_1_160k", "obj4_256k", "stereo_48k_24k"]));
    for (k, s) in streams.iter().enumerate() {
        for seed in 0..3u64 {
            let t = with_truncations(s, 3000 + k as u64 * 10 + seed, seed == 1);
            run_aac(&t, [2, 6, 12][seed as usize], seed, 600, seed == 2, &[]);
            run_mpegh(&t, 2, seed, 600, false, [8, 1, 2][seed as usize]);
        }
    }
    print_stats("aacdec_truncations");
}

#[test]
fn aacdec_earcons() {
    let mut streams = test_streams(&STREAMS[..3]);
    streams.extend(vectors(&[
        "stereo_128k",
        "c5_1_160k",
        "obj4_256k",
        "stereo_48k_24k",
        "mono_64k",
    ]));
    for (k, s) in streams.iter().enumerate() {
        for seed in 0..3u64 {
            let e = with_earcons(s, 2000 + k as u64 * 10 + seed);
            run_aac(&e, [2, 6, 12][seed as usize], seed, 600, seed == 2, &[]);
            if seed == 0 {
                run_mpegh(&e, 2, seed, 600, false, 1);
                run_mpegh(&e, 2, seed, 600, false, 8);
            }
        }
    }
    print_stats("aacdec_earcons");
}

// ---------------------------------------------------------------------------------------------
// mpeghdecoder_*() API
// ---------------------------------------------------------------------------------------------

struct CMpegh(*mut ());

impl Drop for CMpegh {
    fn drop(&mut self) {
        unsafe { cref_mpegh_destroy(self.0) }
    }
}

fn code<T>(r: &Result<T, MpeghDecoderError>) -> i32 {
    match r {
        Ok(_) => 0,
        Err(e) => *e,
    }
}

/// An output frame of `get_samples()`: info, timestamps, samples.
type OutFrame = ([i32; 5], [u64; 2], Vec<i32>);

struct MpeghPair {
    c: CMpegh,
    r: MpeghDecoder,
    ctx: String,
    outputs: usize,
    out_c: Vec<i32>,
    out_r: Vec<i32>,
    /// Output frames not yet compared (the pipelined Rust decoder returns them later).
    q_c: std::collections::VecDeque<OutFrame>,
    q_r: std::collections::VecDeque<OutFrame>,
    /// Fast mode test (the deviations are checked at the end, see [`run_mpegh_fast`]).
    fast: Option<FastCtl>,
    /// Hash of the output frames of the Rust decoder.
    hash: u64,
}

impl MpeghPair {
    /// `get_samples()` until it returns `Ok(None)` (or an error) on both decoders and compares
    /// the output frames. With one thread, the calls return the same frames; with frame
    /// pipelining, the Rust decoder returns them up to one `process()` call later.
    fn get_all(&mut self) {
        let ec = loop {
            let mut info = [0i32; 5];
            let mut ts = [0u64; 2];
            let ec = unsafe {
                cref_mpegh_get_samples(
                    self.c.0,
                    self.out_c.as_mut_ptr(),
                    self.out_c.len() as u32,
                    info.as_mut_ptr(),
                    ts.as_mut_ptr(),
                )
            };
            if ec != 0 {
                break ec;
            }
            let n = (info[0] * info[1]) as usize;
            self.q_c.push_back((info, ts, self.out_c[..n].to_vec()));
        };
        let er = loop {
            let rr = self.r.get_samples(&mut self.out_r);
            let Ok(Some(oi)) = rr else {
                break match &rr {
                    Ok(None) => 1,
                    r => code(r),
                };
            };
            let n = (oi.num_samples_per_channel * oi.num_channels) as usize;
            self.q_r.push_back((
                [
                    oi.num_samples_per_channel,
                    oi.num_channels,
                    oi.sample_rate,
                    oi.loudness,
                    oi.is_concealed as i32,
                ],
                [oi.ticks, oi.pts],
                self.out_r[..n].to_vec(),
            ));
        };
        assert_eq!(er, ec, "{} output {}: get_samples", self.ctx, self.outputs);
        while !self.q_c.is_empty() && !self.q_r.is_empty() {
            let (info_c, ts_c, sc) = self.q_c.pop_front().unwrap();
            let (info_r, ts_r, sr) = self.q_r.pop_front().unwrap();
            assert_eq!(
                (info_r, ts_r),
                (info_c, ts_c),
                "{} output {}: info",
                self.ctx,
                self.outputs
            );
            for &v in &sr {
                self.hash = (self.hash ^ v as u32 as u64).wrapping_mul(0x100000001b3);
            }
            if let Some(f) = self.fast.as_mut() {
                /* bypassed frames are counted at the end (the pipelined decoder is not held) */
                f.compare(&format!("{} output {}", self.ctx, self.outputs), &sr, &sc, 1);
            } else if sr != sc {
                let i = (0..sr.len()).position(|i| sr[i] != sc[i]).unwrap();
                panic!(
                    "{} output {}: samples differ at {i} (rust {} c {})",
                    self.ctx, self.outputs, sr[i], sc[i]
                );
            }
            self.outputs += 1;
        }
        assert!(self.q_r.is_empty(), "{} output {}: extra output", self.ctx, self.outputs);
        if self.r.num_threads() == 1 {
            assert!(self.q_c.is_empty(), "{} output {}: missing output", self.ctx, self.outputs);
        } else {
            assert!(self.q_c.len() <= 3, "{} output {}: output lags", self.ctx, self.outputs);
        }
    }
}

/// Decodes `s` with the mpeghdecoder API in C and Rust with random timestamp jitter, gaps,
/// duplicate timestamps, seeks (flush), parameter changes and invalid calls.
fn run_mpegh(s: &Stream, cicp: i32, seed: u64, max_aus: usize, events: bool, threads: usize) {
    run_mpegh_fast(s, cicp, seed, max_aus, events, threads, Fast::Off);
}

/// [`run_mpegh`] with the fast mode `fast` of the Rust decoder. Returns a hash of the Rust
/// output.
fn run_mpegh_fast(
    s: &Stream,
    cicp: i32,
    seed: u64,
    max_aus: usize,
    events: bool,
    threads: usize,
    fast: Fast,
) -> u64 {
    let mut g = Rng::new(seed);
    let c = CMpegh(unsafe { cref_mpegh_init(cicp) });
    let r = MpeghDecoder::with_threads(cicp, threads);
    assert_eq!(c.0.is_null(), r.is_none(), "init({cicp})");
    let Some(r) = r else { return 0 };
    let mut p = MpeghPair {
        c,
        r,
        ctx: format!("{} cicp {cicp} seed {seed} threads {threads} fast {fast:?}", s.name),
        outputs: 0,
        out_c: vec![0; 24 * 3072],
        out_r: vec![0; 24 * 3072],
        q_c: Default::default(),
        q_r: Default::default(),
        fast: FastCtl::new(fast, seed),
        hash: 0,
    };
    if let Some(f) = &p.fast {
        p.r.set_fast_options(f.options());
    }
    if let Some(conf) = &s.mha_config {
        let ec = unsafe { cref_mpegh_set_mha_config(p.c.0, conf.as_ptr(), conf.len() as u32) };
        let er = p.r.set_mha_config(conf);
        assert_eq!(code(&er), ec, "{} set_mha_config", p.ctx);
    }
    let mut offset: i64 = 0;
    let mut i = 0usize;
    while i < s.aus.len().min(max_aus) {
        let mut pts = s.pts[i] as i64 + offset;
        if events {
            match g.range(0, 50) {
                0 => offset += g.range(-3000, 3000) as i64,  /* jitter */
                1 => offset += g.range(10000, 40000) as i64, /* gap (restart if > 200 ms) */
                2 if i > 0 => pts = s.pts[i - 1] as i64 + offset, /* duplicate timestamp */
                3 => {
                    /* seek */
                    let ec = unsafe { cref_mpegh_flush(p.c.0) };
                    let er = p.r.flush();
                    assert_eq!(code(&er), ec, "{} flush", p.ctx);
                    i = g.range(0, s.aus.len().min(max_aus) as i32 - 1) as usize;
                    continue;
                }
                4 => {
                    let (param, value) = (g.range(-1, 5), g.range(-2, 128));
                    let ec = unsafe { cref_mpegh_set_param(p.c.0, param, value) };
                    let er = code(&p.r.set_param(param, value));
                    assert_eq!(er, ec, "{} set_param({param}, {value})", p.ctx);
                }
                5 => {
                    /* skip an access unit */
                    i += 1;
                    continue;
                }
                _ => {}
            }
        }
        if let Some(f) = p.fast.as_mut() {
            if f.toggle() {
                p.r.set_fast_mode(f.on);
            }
        }
        let ts = pts.max(0) as u64;
        let au = &s.aus[i];
        let ec = unsafe {
            cref_mpegh_process_timescale(p.c.0, au.as_ptr(), au.len() as u32, ts, s.timescale)
        };
        let er = p.r.process_timescale(au, ts, s.timescale);
        assert_eq!(code(&er), ec, "{} AU {i}: process", p.ctx);
        p.get_all();
        i += 1;
    }
    let ec = unsafe { cref_mpegh_flush_and_get(p.c.0) };
    let er = p.r.flush_and_get();
    assert_eq!(code(&er), ec, "{} flush_and_get", p.ctx);
    p.get_all();
    assert!(p.q_c.is_empty(), "{}: missing output at the end", p.ctx);
    if let Some(f) = &p.fast {
        let bypassed = p.r.fast_bypass_frames();
        assert!(bypassed > 0 || f.max_diff == 0, "{}: fast mode: output differs", p.ctx);
        FAST_STATS.with(|st| {
            let mut st = st.borrow_mut();
            st.0 = st.0.max(f.max_diff);
            st.1 += bypassed;
            st.2 += p.outputs as u64;
        });
    }
    p.hash
}

#[test]
fn mpeghdecoder_api() {
    let mut streams = test_streams(&STREAMS);
    streams.extend(vectors(&[
        "stereo_128k_mha1",
        "obj4_256k",
        "c7_1_4_mct_320k",
        "stereo_96k_44k",
    ]));
    for (k, s) in streams.iter().enumerate() {
        /* sequential and pipelined (render thread, with and without thread pool) */
        for threads in [1, 2, 8] {
            run_mpegh(s, 2, 1, 1000, false, threads);
        }
        for seed in 0..3u64 {
            for threads in [1, 2, 8] {
                let cicp = [2, 6, 12][seed as usize];
                run_mpegh(s, cicp, 300 + k as u64 * 3 + seed, 800, true, threads);
            }
        }
    }
    /* invalid target layouts */
    for cicp in [-1, 0, 8, 21, 99, 352, 423, 500] {
        let c = unsafe { cref_mpegh_init(cicp) };
        assert_eq!(c.is_null(), MpeghDecoder::new(cicp).is_none(), "init({cicp})");
        if !c.is_null() {
            unsafe { cref_mpegh_destroy(c) };
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Fast mode (not in C, not bit-exact)
// ---------------------------------------------------------------------------------------------

fn print_fast_stats(test: &str) {
    FAST_STATS.with(|st| {
        let (d, b, n) = *st.borrow();
        println!(
            "{test}: fast mode: {b} bypassed frames of {n}, max deviation {d} ({} LSB of 24 bit)",
            d >> 8
        );
    });
}

/// All vectors in the fast mode at their own layout (target layout 0: the reference layout of
/// the stream, the bypass applies to all channel based vectors) and at other layouts (bit-exact
/// downmixes, or the bypass with silent output channels), without and with random events and
/// switching of the fast mode.
#[test]
fn fast_mode_vectors() {
    for (k, s) in vectors(&VECTORS).iter().enumerate() {
        for tl in [0, 2, 6, 13] {
            run_aac_fast(s, tl, 1, 200, false, &[], Fast::On);
        }
        run_aac_fast(s, 0, 600 + k as u64, 400, true, &[], Fast::Toggle);
    }
    print_fast_stats("fast_mode_vectors");
}

/// The test streams (DRC, loudness, configuration repetitions) in the fast mode.
#[test]
fn fast_mode_streams() {
    for (k, s) in test_streams(&STREAMS).iter().enumerate() {
        run_aac_fast(s, 2, 1, 600, false, &[], Fast::On);
        run_aac_fast(s, 6, 2, 300, false, &[(AAC_UNIDRC_SET_EFFECT, 6)], Fast::On);
        run_aac_fast(s, 2, 700 + k as u64, 600, true, &[], Fast::Toggle);
    }
    print_fast_stats("fast_mode_streams");
}

/// Configuration changes (flushing, build up, truncation, cross-fade) in the fast mode.
#[test]
fn fast_mode_config_changes() {
    let mut streams = test_streams(&STREAMS);
    streams.extend(vectors(&VECTORS));
    if streams.is_empty() {
        return;
    }
    for seed in 0..8u64 {
        let s = spliced(&streams, 800 + seed, 6);
        let fast = if seed % 2 == 0 { Fast::On } else { Fast::Toggle };
        run_aac_fast(&s, [2, 6, 0, 12][seed as usize % 4], seed, 2000, seed >= 4, &[], fast);
    }
    print_fast_stats("fast_mode_config_changes");
}

/// The mpeghdecoder API in the fast mode with timestamp gaps, seeks, parameter changes and
/// switching of the fast mode: within the tolerance of the reference, and the same output for
/// every number of threads.
#[test]
fn fast_mode_mpeghdecoder() {
    let mut streams = test_streams(&STREAMS[..3]);
    streams.extend(vectors(&["c5_1_mct_160k", "c7_1_4_512k", "obj4_256k", "stereo_96k_44k"]));
    let cicp = |s: &Stream| match s.name.split('_').next() {
        Some("c5") => 6,
        Some("c7") => 19,
        _ => 2,
    };
    for (k, s) in streams.iter().enumerate() {
        for (seed, events, fast) in [(1, false, Fast::On), (900 + k as u64, true, Fast::Toggle)] {
            let hashes: Vec<u64> = [1, 2, 8]
                .into_iter()
                .map(|threads| run_mpegh_fast(s, cicp(s), seed, 800, events, threads, fast))
                .collect();
            assert!(
                hashes.iter().all(|&h| h == hashes[0]),
                "{} seed {seed}: fast mode output depends on the number of threads",
                s.name
            );
        }
    }
    let spliced = spliced(&streams, 950, 6);
    let hashes: Vec<u64> = [1, 8]
        .into_iter()
        .map(|threads| run_mpegh_fast(&spliced, 2, 3, 2000, true, threads, Fast::Toggle))
        .collect();
    assert_eq!(hashes[0], hashes[1], "{}: fast mode output depends on the threads", spliced.name);
    print_fast_stats("fast_mode_mpeghdecoder");
}

// ---------------------------------------------------------------------------------------------
// PcmDataPayload()
// ---------------------------------------------------------------------------------------------

/// `EarconDecoder` state in the order of `cref_earcon_get()`.
fn earcon_r(e: &EarconDecoder) -> Vec<i32> {
    let c = &e.earcon_config;
    let mut v = vec![
        c.m_num_pcm_signals as i32,
        c.m_bs_pcm_frame_size_index as i32,
        c.m_bs_pcm_frame_size as i32,
        c.m_bs_pcm_loudness_value as i32,
        c.m_pcm_has_attenuation_gain as i32,
        c.m_bs_pcm_attenuation_gain as i32,
        c.m_bs_pcm_truncation_length as i32,
        c.m_bs_mhas_truncation_length as i32,
        c.sampling_frequency as i32,
        c.truncation_flag as i32,
        c.earcon_flag as i32,
        c.earcon_parse_error as i32,
        e.earcon_info.m_num_earcons as i32,
        e.first_frame as i32,
        e.target_loudness_old,
        e.m_bs_pcm_loudness_value_old as i32,
        e.m_bs_pcm_attenuation_gain_old as i32,
        e.earcon_gain,
        e.earcon_shift,
        e.att_gain_store_high_precision,
        e.att_gain_exp_store_high_precision,
        e.att_gain_intermediate,
        e.att_gain_shift_intermediate,
        e.att_gain_increment_intermediate,
        e.att_gain_old,
        e.att_gain_shift_old,
        e.att_gain_increment_old,
        e.att_gain_new,
        e.att_gain_shift_new,
        e.accumulated_frame_size,
        e.baseframe_size,
        e.truncation_present,
        e.num_pcm_signals_in_frame,
        e.num_pcm_signals_old,
        e.num_signals_mixed,
        e.number_of_rest_channels,
        e.current_frame_has_earcon,
        e.last_frame_had_earcon,
        e.speaker_pos_indices[0],
        e.speaker_pos_indices[1],
        e.speaker_gains[0],
        e.speaker_gains[1],
    ];
    v.extend_from_slice(&e.speaker_pos_indices_rest[..6]);
    v
}

#[test]
fn pcm_data_payload_random() {
    let mut g = Rng::new(7);
    for iter in 0..300 {
        let c = unsafe { cref_earcon_new() };
        let mut r = EarconDecoder::default();
        earcon_decoder_init(&mut r);
        let (tl, nch) =
            [(2, 2), (6, 6), (1, 1), (12, 8), (19, 12), (13, 24)][g.range(0, 5) as usize];
        let baseframe = [1024, 2048, 1536, 3072][g.range(0, 3) as usize];
        let mut time_c = vec![0i32; baseframe as usize * 24];
        for frame in 0..20 {
            let ctx = format!("iter {iter} frame {frame}");
            /* new earcon data / config (like aacDecoder_EarconSetBSCallback()) */
            if g.range(0, 2) > 0 {
                let e = &mut r;
                e.earcon_config.m_num_pcm_signals = g.range(1, 2) as u32;
                if g.range(0, 3) == 0 {
                    e.earcon_config.m_pcm_has_attenuation_gain = g.range(0, 2) as u32;
                    e.earcon_config.m_bs_pcm_attenuation_gain =
                        [0, 1, 40, 128, 254, 255][g.range(0, 5) as usize];
                }
                if g.range(0, 3) == 0 {
                    e.earcon_config.m_bs_pcm_loudness_value = g.range(0, 255) as u32;
                }
                e.earcon_config.truncation_flag = (g.range(0, 5) == 0) as u32;
                if e.first_frame == 1 {
                    e.earcon_gain = 0x7fffffff;
                    e.earcon_shift = 8;
                    e.target_loudness_old = 0;
                    e.m_bs_pcm_loudness_value_old = 0;
                    e.m_bs_pcm_attenuation_gain_old = 0;
                    e.att_gain_store_high_precision = 0x7fffffff;
                    e.att_gain_exp_store_high_precision = 0;
                    e.att_gain_intermediate = 0x7fffffff;
                    e.att_gain_shift_intermediate = 0;
                    e.att_gain_increment_intermediate = 0;
                    e.att_gain_old = 0x7fffffff;
                    e.att_gain_shift_old = 0;
                    e.att_gain_increment_old = 0;
                    e.att_gain_new = 0x7fffffff;
                    e.att_gain_shift_new = 0;
                    e.accumulated_frame_size = 0;
                }
                let n = (baseframe * e.earcon_config.m_num_pcm_signals as i32)
                    .min(3072 - e.accumulated_frame_size);
                for k in 0..n.max(0) as usize {
                    e.earcon_data[e.accumulated_frame_size as usize + k] = g.i16();
                }
                e.accumulated_frame_size += n.max(0);
                let v = earcon_r(e);
                unsafe {
                    cref_earcon_set(c, v.as_ptr());
                    let d = std::slice::from_raw_parts_mut(cref_earcon_data(c), EARCON_BUFFER_SIZE);
                    d.copy_from_slice(&e.earcon_data[..]);
                }
            }
            for x in time_c.iter_mut() {
                *x = g.i32() >> g.range(0, 12);
            }
            let mut time_r = time_c.clone();
            let frame_size = if g.range(0, 3) == 0 { g.range(0, baseframe) } else { baseframe };
            let last = if g.range(0, 3) == 0 { g.range(0, 775) } else { 775 }.min(frame_size);
            let newf = frame_size - last;
            let drc_tl = [-128, -24, -31, -16, 0, 5][g.range(0, 5) as usize];
            let def_tl = [96, 64, 124, -1, 40][g.range(0, 4) as usize];
            let ec = unsafe {
                cref_pcm_data_payload(
                    c,
                    time_c.as_mut_ptr(),
                    baseframe as u32,
                    drc_tl,
                    def_tl,
                    tl,
                    last,
                    newf,
                )
            };
            let er = pcm_data_payload(
                &mut r,
                &mut time_r,
                baseframe as u32,
                drc_tl as i8,
                def_tl as i8,
                tl,
                last as i16,
                newf as i16,
            );
            assert_eq!(er, ec, "{ctx}: error");
            let mut vc = vec![0i32; 48];
            unsafe { cref_earcon_get(c, vc.as_mut_ptr()) };
            assert_eq!(earcon_r(&r), vc, "{ctx}: state");
            let dc = unsafe { std::slice::from_raw_parts(cref_earcon_data(c), EARCON_BUFFER_SIZE) };
            assert!(dc == &r.earcon_data[..], "{ctx}: earcon data");
            assert!(time_r == time_c, "{ctx}: time data (nch {nch})");
        }
        unsafe { cref_earcon_free(c) };
    }
}

/// The fixed point constants of the frame level (float vs. double evaluation of the C++
/// expressions).
#[test]
fn frame_level_constants() {
    use mpeghdec_fdk::common_fix::{fl2fxconst_dbl, fl2fxconst_sgl};
    let mut c = [0i32; 9];
    unsafe { cref_ui_constants(c.as_mut_ptr()) };
    let r = [
        fl2fxconst_dbl((0.3321928f32 * (1 << 1) as f32) as f64),
        fl2fxconst_dbl(3.01029996f64 / (1 << 2) as f32 as f64),
        fl2fxconst_sgl((1.5f32 / 180.0f32 * 64.0f32) as f64) as i32,
        fl2fxconst_sgl((3.0f32 / 180.0f32 * 32.0f32) as f64) as i32,
        fl2fxconst_dbl((1.0f32 / (1 << 9) as f32) as f64),
        fl2fxconst_dbl((1.0f32 / (1 << 7) as f32) as f64),
        fl2fxconst_dbl((0.5f32 / 127.0f32) as f64),
        fl2fxconst_sgl(1.0 / 127.0) as i32,
        TDL_MPEGH3DA_DEFAULT_THRESHOLD,
    ];
    assert_eq!(r, c);
}
