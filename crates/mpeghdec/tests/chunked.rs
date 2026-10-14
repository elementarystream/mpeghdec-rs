// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: tests of the chunked parallel decoding.

//! The chunked decoding returns the output of the sequential decoding loop, also with chunk
//! boundaries at timestamp gaps (decoder restarts), configuration changes, corrupt (concealed)
//! and lost access units. The tests use the test streams (`MPEGHDEC_TEST_STREAMS`, default
//! `work/streams`) and are skipped without them.

use mpeghdec::chunked::*;
use mpeghdec::{
    Decoder as MpeghDecoder, FastOptions, OutputInfo as MpeghDecoderOutputInfo,
    MAX_NUM_FRAME_SAMPLES, MAX_NUM_OUTPUT_CHANNELS,
};

/// Access units (data, 90 kHz timestamp) of a test stream.
fn load(name: &str) -> Option<Vec<(Vec<u8>, u64)>> {
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
    Some(
        pts.lines()
            .filter_map(|l| {
                let v: Vec<u64> = l.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                (v.len() >= 3).then(|| (data[v[0] as usize..(v[0] + v[1]) as usize].to_vec(), v[2]))
            })
            .collect(),
    )
}

type Output = (Vec<(MpeghDecoderOutputInfo, Vec<i32>)>, Option<String>);

/// The sequential decoding loop of the demo decoder; the error is described like the chunked
/// decoding's.
fn sequential(aus: &[(Vec<u8>, u64)], tl: i32) -> Output {
    sequential_with(aus, tl, FastOptions::EXACT)
}

fn sequential_with(aus: &[(Vec<u8>, u64)], tl: i32, fast: FastOptions) -> Output {
    let mut dec = MpeghDecoder::with_threads(tl, 1).unwrap();
    dec.set_fast_options(fast);
    let mut out = vec![0i32; MAX_NUM_OUTPUT_CHANNELS * MAX_NUM_FRAME_SAMPLES];
    let mut frames = Vec::new();
    for (i, (d, pts)) in aus.iter().enumerate() {
        if let Err(e) = dec.process_timescale(d, *pts, 90000) {
            return (frames, Some(format!("process {i}: {e:?}")));
        }
        if i + 1 == aus.len() {
            if let Err(e) = dec.flush_and_get() {
                return (frames, Some(format!("flush: {e:?}")));
            }
        }
        loop {
            match dec.get_samples(&mut out) {
                Ok(Some(info)) => {
                    let n = info.samples();
                    frames.push((info, out[..n].to_vec()));
                }
                Ok(None) => break,
                Err(e) => return (frames, Some(format!("output: {e:?}"))),
            }
        }
    }
    (frames, None)
}

fn chunked(aus: &[(Vec<u8>, u64)], tl: i32, opts: &ChunkedOptions) -> (Output, ChunkedStats) {
    chunked_with(aus, tl, opts, FastOptions::EXACT)
}

fn chunked_with(
    aus: &[(Vec<u8>, u64)],
    tl: i32,
    opts: &ChunkedOptions,
    fast: FastOptions,
) -> (Output, ChunkedStats) {
    let units: Vec<AccessUnit> = aus.iter().map(|(d, p)| AccessUnit { data: d, pts: *p }).collect();
    let src = MemorySource { access_units: &units, timescale: 90000 };
    let setup = DecoderSetup { cicp_setup: tl, fast, ..Default::default() };
    let mut frames = Vec::new();
    let mut sink = |info: &MpeghDecoderOutputInfo, s: &[i32]| -> Result<(), ()> {
        frames.push((*info, s.to_vec()));
        Ok(())
    };
    let r = decode_chunked(&src, &setup, opts, &mut sink);
    let (err, stats) = match r {
        Ok(s) => (None, s),
        Err(ChunkedError::Process { index, error }) => {
            (Some(format!("process {index}: {error:?}")), ChunkedStats::default())
        }
        Err(ChunkedError::Flush(e)) => (Some(format!("flush: {e:?}")), ChunkedStats::default()),
        Err(ChunkedError::Output { error, .. }) => {
            (Some(format!("output: {error:?}")), ChunkedStats::default())
        }
        Err(e) => panic!("{e:?}"),
    };
    ((frames, err), stats)
}

fn opts(chunks: usize, warmup: usize) -> ChunkedOptions {
    ChunkedOptions { threads: 3, chunks, warmup, ..Default::default() }
}

/// Decodes chunked with `opts` and compares with the sequential decoding.
fn compare(what: &str, aus: &[(Vec<u8>, u64)], tl: i32, opts: &ChunkedOptions) -> ChunkedStats {
    compare_with(what, aus, tl, opts, FastOptions::EXACT)
}

fn compare_with(
    what: &str,
    aus: &[(Vec<u8>, u64)],
    tl: i32,
    opts: &ChunkedOptions,
    fast: FastOptions,
) -> ChunkedStats {
    let (seq_frames, seq_err) = sequential_with(aus, tl, fast);
    let ((frames, err), stats) = chunked_with(aus, tl, opts, fast);
    eprintln!("{what}: {} frames, error {err:?}, {stats:?}", frames.len());
    assert_eq!(err, seq_err, "{what}: error");
    assert_eq!(frames.len(), seq_frames.len(), "{what}: number of frames ({stats:?})");
    for (k, (a, b)) in frames.iter().zip(&seq_frames).enumerate() {
        assert!(a == b, "{what}: frame {k} differs ({stats:?})");
    }
    stats
}

#[test]
fn chunked_equals_sequential() {
    let Some(aus) = load("s3_dvs_a") else { return };
    let s = compare("6 chunks", &aus, 2, &opts(6, 216));
    assert!(s.handoffs > 0, "{s:?}");
    compare("one chunk", &aus, 2, &opts(1, 216));
    compare("22.2 target layout", &aus[..600], 13, &opts(3, 100));
    /* output buffer limit: the speculative runs wait */
    let o = ChunkedOptions { max_buffered_bytes: 1, ..opts(8, 64) };
    let s = compare("tiny buffer", &aus, 2, &o);
    assert!(s.handoffs > 0, "{s:?}");
    /* one worker thread */
    compare("1 thread", &aus, 2, &ChunkedOptions { threads: 1, ..opts(5, 100) });
}

/// A stream that starts before its first configuration packet.
#[test]
fn stream_starting_before_the_configuration() {
    let Some(aus) = load("s4_dvs_a") else { return };
    let s = compare("s4", &aus, 2, &opts(6, 216));
    assert!(s.handoffs > 0, "{s:?}");
}

/// Timestamp gaps of more than 200 ms restart the decoder (`Decoder::process()`), at a
/// chunk boundary, just before and after one and within the warm-up of a chunk.
#[test]
fn timestamp_gaps() {
    let Some(aus) = load("s1_clip_a") else { return };
    let n = aus.len();
    let chunks = 4;
    for gap_at in [n / chunks, n / chunks - 3, n / chunks + 2, 2 * n / chunks - 100, n - 5] {
        let mut v = aus.clone();
        for (_, p) in &mut v[gap_at..] {
            *p += 90000;
        }
        compare(&format!("gap at {gap_at}"), &v, 2, &opts(chunks, 216));
    }
    /* a timestamp that goes back */
    let mut v = aus.clone();
    for (_, p) in &mut v[n / 2..] {
        *p -= 45000;
    }
    compare("backwards", &v, 2, &opts(chunks, 216));
}

/// Access units that are missing (timestamp jumps below the restart threshold: the decoder
/// inserts silence) at and near chunk boundaries.
#[test]
fn lost_access_units() {
    let Some(aus) = load("s1_clip_a") else { return };
    let n = aus.len();
    let mut v = aus.clone();
    for k in [n / 4, n / 2 + 1, n / 2 - 2, 3 * n / 4 + 70] {
        v.remove(k);
        v.remove(k);
    }
    compare("lost", &v, 2, &opts(4, 216));
}

/// A configuration change: the access units of another stream follow (with continuing
/// timestamps), with the chunk boundaries around the change.
#[test]
fn configuration_change() {
    let (Some(a), Some(b)) = (load("s1_clip_a"), load("s3_dvs_a")) else {
        return;
    };
    let mut v: Vec<(Vec<u8>, u64)> = a[..700].to_vec();
    let base = v.last().unwrap().1 + 1920;
    let b0 = b[0].1;
    v.extend(b[..700].iter().map(|(d, p)| (d.clone(), base + (p - b0))));
    for chunks in [3, 4, 5] {
        compare(&format!("{chunks} chunks"), &v, 2, &opts(chunks, 216));
    }
    compare("small warm-up", &v, 2, &opts(7, 32));
}

/// Corrupt access units (decoding errors, concealment) near chunk boundaries: the output of a
/// speculative run is not used after a hazard.
#[test]
fn corrupt_access_units() {
    let Some(aus) = load("s3_dvs_a") else { return };
    let n = aus.len();
    let mut v = aus.clone();
    let chunks = 5;
    let mut total = ChunkedStats::default();
    for (k, at) in
        [n / chunks + 30, 2 * n / chunks + 1, 3 * n / chunks + 150].into_iter().enumerate()
    {
        for i in at..at + 4 {
            let d = &mut v[i].0;
            let len = d.len();
            /* damage the end of the frame payload */
            for j in len - len / 3..len {
                d[j] ^= (0x5a + k as u8).wrapping_mul(j as u8 | 1);
            }
        }
    }
    for warmup in [216, 16] {
        let s = compare(&format!("warm-up {warmup}"), &v, 2, &opts(chunks, warmup));
        total.fallbacks += s.fallbacks;
        total.rejected += s.rejected;
    }
    /* the concealment of the corrupt frames either stopped a speculative run (hazard) or its
    boundary was not accepted */
    assert!(total.fallbacks + total.rejected > 0, "{total:?}");
}

/// A process error of the sequential decoding (an access unit the decoder rejects) is returned
/// after the same frames.
#[test]
fn process_error() {
    let Some(aus) = load("s1_clip_a") else { return };
    let mut v = aus[..900].to_vec();
    /* an access unit larger than the input buffer of the transport decoder */
    v[700].0 = vec![0x55; 70000];
    compare("process error", &v, 2, &opts(4, 216));
}

/// Removes the MHAS configuration packets (type 1) from an access unit.
fn strip_config(d: &[u8]) -> Vec<u8> {
    fn escaped(d: &[u8], p: &mut usize, n1: u32, n2: u32, n3: u32) -> u64 {
        let mut u = |n: u32| {
            let mut v = 0u64;
            for _ in 0..n {
                v = (v << 1) | ((d[*p >> 3] >> (7 - (*p & 7))) & 1) as u64;
                *p += 1;
            }
            v
        };
        let mut v = u(n1);
        if v == (1 << n1) - 1 {
            let e = u(n2);
            v += e;
            if e == (1 << n2) - 1 {
                v += u(n3);
            }
        }
        v
    }
    let mut out = Vec::new();
    let mut off = 0;
    while off < d.len() {
        let mut p = 0;
        let h = &d[off..];
        let t = escaped(h, &mut p, 3, 8, 8);
        escaped(h, &mut p, 2, 8, 32);
        let len = escaped(h, &mut p, 11, 24, 24) as usize;
        let end = (off + p.div_ceil(8) + len).min(d.len());
        if t != 1 {
            out.extend_from_slice(&d[off..end]);
        }
        off = end;
    }
    out
}

/// Without configuration repetition the input is decoded sequentially (no random access points
/// for the chunks); with `independent_starts` the chunks start at independent frames.
#[test]
fn no_configuration_repetition() {
    let Some(aus) = load("s1_clip_a") else { return };
    let first = aus.iter().position(|(d, _)| strip_config(d).len() != d.len()).unwrap();
    let v: Vec<(Vec<u8>, u64)> = aus
        .iter()
        .enumerate()
        .map(|(i, (d, p))| (if i > first { strip_config(d) } else { d.clone() }, *p))
        .collect();
    let s = compare("config only once", &v, 2, &opts(4, 216));
    assert_eq!(s.sequential_from, Some(0), "{s:?}");
    let o = ChunkedOptions { independent_starts: true, ..opts(4, 216) };
    let s = compare("independent starts", &v, 2, &o);
    /* the speculative runs were started and compared (whether their states become equal
    depends on the stream) */
    assert!(s.digests > 0 && s.sequential_from != Some(0), "{s:?}");
}

/// When the states keep differing (a warm-up far too short for the format converter), the true
/// run gives up the chunked decoding and decodes the rest with all threads.
#[test]
fn giving_up() {
    let Some(aus) = load("s3_dvs_a") else { return };
    let o = ChunkedOptions { verify_offsets: vec![0], max_rejections: 2, ..opts(12, 1) };
    let s = compare("give up", &aus, 2, &o);
    assert!(s.sequential_from.is_some() || s.handoffs > 0, "{s:?}");
}

/// With the fast mode, the chunked decoding returns the output of the sequential decoding with
/// the fast mode (the options are part of the state digest). The bypassed format converter has
/// no slowly converging state; the warm-up only has to cover the DRC startup (2.5 s).
#[test]
fn fast_mode() {
    let Some(aus) = load("s1_clip_a") else { return };
    let s = compare_with("fast", &aus, 2, &opts(4, 216), FastOptions::ALL);
    assert_eq!(s.handoffs, 3, "{s:?}");
    /* the bypassed frames of the output are counted like in the sequential decoding */
    let mut dec = MpeghDecoder::with_threads(2, 1).unwrap();
    dec.set_fast_options(FastOptions::ALL);
    let mut out = vec![0i32; MAX_NUM_OUTPUT_CHANNELS * MAX_NUM_FRAME_SAMPLES];
    for (i, (d, pts)) in aus.iter().enumerate() {
        dec.process_timescale(d, *pts, 90000).unwrap();
        if i + 1 == aus.len() {
            dec.flush_and_get().unwrap();
        }
        while dec.get_samples(&mut out).unwrap().is_some() {}
    }
    assert!(s.fast_bypass_frames > 0);
    assert_eq!(s.fast_bypass_frames, dec.fast_bypass_frames(), "{s:?}");
    let s = compare_with("fast, short warm-up", &aus, 2, &opts(8, 120), FastOptions::ALL);
    assert!(s.handoffs >= 6, "{s:?}");
    compare_with("fast, 5.1 output", &aus, 6, &opts(4, 64), FastOptions::ALL);
    /* the chunked fast output differs from the bit-exact one (the bypass is used) */
    let exact = sequential(&aus, 2).0;
    let fast = chunked_with(&aus, 2, &opts(4, 216), FastOptions::ALL).0 .0;
    assert!(exact != fast);
}
