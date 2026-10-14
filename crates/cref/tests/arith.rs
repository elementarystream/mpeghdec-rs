// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libArithCoding (ac_arith_coder.cpp): random bitstreams are decoded frame
//! after frame with persistent context state by the C and the Rust arithmetic decoder.
#![cfg(not(cref_missing))]

use mpeghdec_arith::ac_arith_coder::*;
use mpeghdec_cref::arith::*;
use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};

fn bs_state(bs: &FdkBitStream) -> [u32; 6] {
    [
        bs.cache_word,
        bs.bits_in_cache,
        bs.h_bit_buf.bit_ndx(),
        bs.h_bit_buf.valid_bits as u32,
        bs.h_bit_buf.read_offset(),
        bs.h_bit_buf.write_offset(),
    ]
}

fn c_bs_state(h: *mut ()) -> [u32; 6] {
    let mut st = [0u32; 6];
    unsafe { cref_bs_state(h, st.as_mut_ptr()) };
    st
}

/// Random buffer content with different bit statistics.
fn random_bytes(g: &mut Rng, n: usize) -> Vec<u8> {
    let kind = g.range(0, 5);
    (0..n)
        .map(|_| match kind {
            0 => g.next_u32() as u8,
            1 => (g.next_u32() & g.next_u32() & g.next_u32()) as u8, // mostly zero bits
            2 => (g.next_u32() | g.next_u32() | g.next_u32()) as u8, // mostly one bits
            3 => {
                if g.range(0, 7) == 0 {
                    g.next_u32() as u8
                } else {
                    0
                }
            }
            _ => {
                if g.range(0, 7) == 0 {
                    g.next_u32() as u8
                } else {
                    0xFF
                }
            }
        })
        .collect()
}

/// A C and a Rust bitstream over identical copies of a random buffer.
struct BsPair {
    _buf_c: Vec<u8>,
    _buf_r: Vec<u8>,
    hc: *mut (),
    r: FdkBitStream,
}

impl BsPair {
    fn new(g: &mut Rng, log2_size: i32) -> Self {
        let size = 1u32 << log2_size;
        let mut buf_c = random_bytes(g, size as usize);
        let buf_r = buf_c.clone();
        let valid = match g.range(0, 5) {
            0 => g.range(0, 2000) as u32,
            _ => size * 8,
        };
        let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, valid, 0) };
        let mut r = FdkBitStream::default();
        r.init(&BitMem::from_slice(&buf_r), size, valid, FdkBsCfg::BsReader);
        BsPair { _buf_c: buf_c, _buf_r: buf_r, hc, r }
    }
}

impl Drop for BsPair {
    fn drop(&mut self) {
        unsafe { cref_bs_free(self.hc) };
    }
}

struct ArcoPair {
    hc: *mut (),
    r: Box<CArcoData>,
}

impl ArcoPair {
    fn new() -> Self {
        ArcoPair { hc: unsafe { cref_arco_create() }, r: CArcoData::create() }
    }

    fn check(&self, ctx: &str) {
        let mut nlp = 0i16;
        let mut c_prev = [0u8; C_PREV_LEN];
        unsafe { cref_arco_get_state(self.hc, &mut nlp, c_prev.as_mut_ptr()) };
        assert_eq!(self.r.m_number_lines_prev, nlp, "m_numberLinesPrev {ctx}");
        assert_eq!(self.r.c_prev[..], c_prev[..], "c_prev {ctx}");
    }

    fn set(&mut self, nlp: i16, c_prev: &[u8; C_PREV_LEN]) {
        self.r.m_number_lines_prev = nlp;
        self.r.c_prev = *c_prev;
        unsafe { cref_arco_set_state(self.hc, nlp, c_prev.as_ptr()) };
    }
}

impl Drop for ArcoPair {
    fn drop(&mut self) {
        unsafe { cref_arco_destroy(self.hc) };
    }
}

/// Decode one window with both implementations and compare everything.
fn decode_both(
    a: &mut ArcoPair,
    bs: &mut BsPair,
    lg: i32,
    lg_max: i32,
    reset: i32,
    ctx: &str,
    stats: &mut [u32; 2],
) -> i32 {
    // Output buffer pre-filled with garbage (the decoder clears lg_max entries).
    let mut spec_c = vec![0x5A5A_5A5Ai32; 1024 + 8];
    let mut spec_r = spec_c.clone();
    let ec = unsafe { cref_arco_decode(a.hc, bs.hc, spec_c.as_mut_ptr(), lg, lg_max, reset) };
    let er = carco_decode_arith_data(&mut a.r, &mut bs.r, &mut spec_r, lg, lg_max, reset);
    assert_eq!(er, ec, "error {ctx}");
    assert_eq!(spec_r, spec_c, "spectrum {ctx}");
    assert_eq!(bs_state(&bs.r), c_bs_state(bs.hc), "bitstream {ctx}");
    a.check(ctx);
    stats[(ec != ARITH_CODER_OK) as usize] += 1;
    ec
}

/// Window lengths used by the MPEG-H core coder for a frame length (long, short).
fn win_lens(frame_length: i32) -> [i32; 2] {
    [frame_length, frame_length / 8]
}

/// Transitions for which the C code (copyTableAmrwbArith2) reads outside of CArcoData.
fn undefined_transition(prev: i32, cur: i32) -> bool {
    (prev == 96 && cur == 1024) || (prev == 1024 && cur == 768)
}

fn pick_lg(g: &mut Rng, win_len: i32) -> i32 {
    match g.range(0, 9) {
        0 => 0,
        1 => win_len,
        2 => win_len + g.range(1, 8), // lg > lg_max: error
        3 => g.range(0, win_len),     // odd values allowed
        _ => g.range(0, win_len / 4) * 4,
    }
}

fn run_frames(seed: u64, stats: &mut [u32; 2]) {
    let mut g = Rng::new(seed);
    let mut a = ArcoPair::new();
    let mut frame_length = if g.range(0, 1) == 0 { 1024 } else { 768 };
    let mut prev_len = 0;
    let n_frames = g.range(5, 25);
    for frame in 0..n_frames {
        // Configuration change or interruption.
        match g.range(0, 30) {
            0 => frame_length = 1792 - frame_length,
            1 => {
                // CAacDecoder_SignalInterruption()
                let mut c_prev = [0u8; C_PREV_LEN];
                let mut nlp = 0i16;
                unsafe { cref_arco_get_state(a.hc, &mut nlp, c_prev.as_mut_ptr()) };
                a.set(0, &c_prev);
                prev_len = 0;
            }
            _ => {}
        }
        let short = g.range(0, 2) == 0;
        let num_win = if short { 8 } else { 1 };
        let win_len = win_lens(frame_length)[short as usize];
        let lg = pick_lg(&mut g, win_len);
        let mut reset = (g.range(0, 4) == 0) as i32;
        if undefined_transition(prev_len, win_len) {
            reset = 1;
        }
        let log2_size = if g.range(0, 3) == 0 { 8 } else { 12 };
        let mut bs = BsPair::new(&mut g, log2_size);
        for win in 0..num_win {
            let ctx = format!("seed {seed} frame {frame} win {win} lg {lg} lg_max {win_len}");
            let reset_win = (reset != 0 && win == 0) as i32;
            let err = decode_both(&mut a, &mut bs, lg, win_len, reset_win, &ctx, stats);
            if err != ARITH_CODER_OK {
                break;
            }
        }
        let mut nlp = 0i16;
        let mut c_prev = [0u8; C_PREV_LEN];
        unsafe { cref_arco_get_state(a.hc, &mut nlp, c_prev.as_mut_ptr()) };
        prev_len = nlp as i32;
    }
}

#[test]
fn decode_frames() {
    let mut stats = [0u32; 2];
    for seed in 0..600 {
        run_frames(seed, &mut stats);
    }
    // Both successful and failing calls must be covered.
    assert!(stats[0] > 1000 && stats[1] > 100, "stats {stats:?}");
}

/// All window length transitions with random (also out of range) context values.
#[test]
fn transitions_random_context() {
    let lens = [1024, 128, 768, 96];
    let mut stats = [0u32; 2];
    let mut g = Rng::new(4711);
    for iter in 0..3000 {
        let mut a = ArcoPair::new();
        let prev = lens[g.range(0, 3) as usize];
        let cur = lens[g.range(0, 3) as usize];
        if undefined_transition(prev, cur) {
            continue;
        }
        let mut c_prev = [0u8; C_PREV_LEN];
        let max = if g.range(0, 3) == 0 { 255 } else { 15 };
        for v in c_prev.iter_mut() {
            *v = g.range(0, max) as u8;
        }
        a.set(prev as i16, &c_prev);
        let mut bs = BsPair::new(&mut g, 12);
        let lg = pick_lg(&mut g, cur);
        let ctx = format!("iter {iter} {prev}->{cur} lg {lg}");
        decode_both(&mut a, &mut bs, lg, cur, 0, &ctx, &mut stats);
        // Continue with a second frame on the resulting context.
        let lg = pick_lg(&mut g, cur);
        decode_both(&mut a, &mut bs, lg, cur, 0, &ctx, &mut stats);
    }
}

/// `ari_decode_14bits` with the table sizes used by the IGF scale factor decoder and random
/// cumulative frequency tables and decoder states.
#[test]
fn ari_decode_14bits_random() {
    let mut g = Rng::new(99);
    for iter in 0..3000 {
        let cfl = [2, 4, 17, 27, 5][g.range(0, 4) as usize];
        let n = cfl.max(2) as usize;
        let mut c_freq: Vec<i16> = (0..n - 1).map(|_| g.range(0, 16384) as i16).collect();
        c_freq.sort_unstable_by(|a, b| b.cmp(a));
        c_freq.push(0);
        let mut st = if g.range(0, 1) == 0 {
            // consistent state
            let low = g.range(0, 0xFFFF);
            let high = g.range(low, 0xFFFF);
            let vobf = g.range(low, high);
            [low, high, vobf, 0]
        } else {
            [g.range(0, 0xFFFF), g.range(0, 0xFFFF), g.range(0, 0xFFFF), g.range(-5, 5)]
        };
        let mut bs = BsPair::new(&mut g, 8);
        let mut s = Tastat { low: st[0], high: st[1], vobf: st[2], bits_to_follow: st[3] };
        for k in 0..20 {
            let rc =
                unsafe { cref_ari_decode_14bits(bs.hc, st.as_mut_ptr(), c_freq.as_ptr(), cfl) };
            let rr = ari_decode_14bits(&mut bs.r, &mut s, &c_freq, cfl);
            let ctx = format!("iter {iter} step {k} cfl {cfl}");
            assert_eq!(rr, rc, "{ctx}");
            assert_eq!([s.low, s.high, s.vobf, s.bits_to_follow], st, "{ctx}");
            assert_eq!(bs_state(&bs.r), c_bs_state(bs.hc), "{ctx}");
        }
    }
}
