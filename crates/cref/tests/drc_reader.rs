// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of the libDRCdec bitstream readers (drcDec_reader.cpp): random, structured,
//! truncated and garbage mpegh3daUniDrcConfig(), mpegh3daLoudnessInfoSet() and uniDrcGain()
//! payloads are parsed into persistent C and Rust states, which are compared field by field after
//! every call, together with the error codes and the bitstream positions.
#![cfg(not(cref_missing))]

mod drc_common;

use drc_common::*;
use mpeghdec_cref::drc::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_drc::drc_dec_reader::*;
use mpeghdec_drc::drc_dec_tools::get_delta_tmin;
use mpeghdec_drc::drc_dec_types::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};

const BUF_SIZE: usize = 4096;

struct RustReader {
    cfg: Box<UniDrcConfig>,
    lis: Box<LoudnessInfoSet>,
    gain: Box<UniDrcGain>,
}

fn ser_rust(r: &RustReader) -> Vec<i32> {
    let mut o = Vec::new();
    ser_config(&r.cfg, &mut o);
    ser_loudness_set(&r.lis, &mut o);
    ser_gain(&r.gain, &mut o);
    o
}

fn ser_c(h: *mut ()) -> Vec<i32> {
    let mut v = vec![0i32; 200_000];
    let n = unsafe { cref_drc_reader_ser(h, v.as_mut_ptr()) };
    v.truncate(n as usize);
    v
}

/// Makes a bitstream buffer from a payload: either the payload (possibly truncated) or garbage.
fn make_buffer(g: &mut Rng, w: &BitWriter) -> (Vec<u8>, u32) {
    let mut buf = vec![0u8; BUF_SIZE];
    let mode = g.range(0, 9);
    if mode == 0 {
        /* garbage */
        for b in buf.iter_mut() {
            *b = g.next_u32() as u8;
        }
        let valid = g.range(0, (BUF_SIZE * 8) as i32) as u32;
        return (buf, valid);
    }
    let n = w.bytes.len().min(BUF_SIZE);
    buf[..n].copy_from_slice(&w.bytes[..n]);
    let bits = (w.bits as u32).min((BUF_SIZE * 8) as u32);
    if mode == 1 {
        /* truncated; garbage after the end */
        let valid = g.range(0, bits as i32) as u32;
        for b in buf[(valid as usize / 8 + 1).min(BUF_SIZE)..].iter_mut() {
            *b = g.next_u32() as u8;
        }
        return (buf, valid);
    }
    if mode == 2 {
        /* flip some bits */
        for _ in 0..g.range(1, 4) {
            let p = g.range(0, bits.max(1) as i32 - 1) as usize;
            buf[p / 8] ^= 0x80 >> (p % 8);
        }
    }
    (buf, bits)
}

fn run(seed: u64) {
    let mut g = Rng::new(seed);
    let hc = unsafe { cref_drc_reader_new() };
    let mut r = RustReader {
        cfg: Box::new(UniDrcConfig::ZERO),
        lis: Box::new(LoudnessInfoSet::ZERO),
        gain: Box::new(UniDrcGain::ZERO),
    };
    let nice_bias = g.range(0, 100);
    let frame_size = [1024, 1024, 512, 768, 2048, 256][g.range(0, 5) as usize];
    let sample_rate = [48000, 44100, 32000, 16000, 96000][g.range(0, 4) as usize];

    for step in 0..24 {
        let nice = g.range(0, 99) < nice_bias;
        let op = g.range(0, 9);
        let sub = if g.range(0, 9) < 7 { 0 } else { g.range(1, 4) };
        let mut w = BitWriter::new();
        let (res_c, res_r, left_c, left_r);
        match op {
            0..=3 => {
                let params = ConfigParams {
                    channels: if nice { g.range(1, 8) as u32 } else { g.range(0, 127) as u32 },
                    nice,
                    multiband: g.range(0, 1) == 1,
                    downmix: Vec::new(),
                    sub: sub.min(3) as usize,
                };
                write_uni_drc_config(&mut w, &mut g, &params);
                let (mut buf_c, valid) = make_buffer(&mut g, &w);
                let buf_r = buf_c.clone();
                let mut lc = 0;
                res_c = unsafe {
                    cref_drc_reader_config(
                        hc,
                        buf_c.as_mut_ptr(),
                        BUF_SIZE as u32,
                        valid,
                        sub,
                        &mut lc,
                    )
                };
                let mut bs = FdkBitStream::default();
                bs.init(&BitMem::from_slice(&buf_r), BUF_SIZE as u32, valid, FdkBsCfg::BsReader);
                res_r = drc_dec_read_mpegh3da_uni_drc_config(&mut bs, &mut r.cfg, &mut r.lis, sub);
                left_c = lc;
                left_r = bs.get_valid_bits();
            }
            4..=5 => {
                write_loudness_info_set(&mut w, &mut g, nice, &[1, 2, 3], sub.min(3) as usize);
                let (mut buf_c, valid) = make_buffer(&mut g, &w);
                let buf_r = buf_c.clone();
                let mut lc = 0;
                res_c = unsafe {
                    cref_drc_reader_loudness(
                        hc,
                        buf_c.as_mut_ptr(),
                        BUF_SIZE as u32,
                        valid,
                        sub,
                        &mut lc,
                    )
                };
                let mut bs = FdkBitStream::default();
                bs.init(&BitMem::from_slice(&buf_r), BUF_SIZE as u32, valid, FdkBsCfg::BsReader);
                res_r = drc_dec_read_mpegh3da_loudness_info_set(&mut bs, &mut r.lis, sub);
                left_c = lc;
                left_r = bs.get_valid_bits();
            }
            _ => {
                let coef_index = [0, 0, 0, 1, -1][g.range(0, 4) as usize];
                let delta_tmin_default = if g.range(0, 9) == 0 {
                    let d = g.range(-100, 64);
                    // A zero divisor in _readNodes() (frameSize / timeDeltaMin) is undefined:
                    // AArch64 gives 0 (reproduced by the port), x86 traps (SIGFPE).
                    if d == 0 && cfg!(any(target_arch = "x86", target_arch = "x86_64")) {
                        1
                    } else {
                        d
                    }
                } else {
                    get_delta_tmin(sample_rate)
                };
                if coef_index >= 0 {
                    let coef = r.cfg.drc_coefficients_uni_drc[coef_index as usize].clone();
                    write_uni_drc_gain(
                        &mut w,
                        &mut g,
                        &coef,
                        frame_size,
                        delta_tmin_default,
                        sub.min(3) as usize,
                        nice,
                    );
                }
                let (mut buf_c, valid) = make_buffer(&mut g, &w);
                let buf_r = buf_c.clone();
                let mut lc = 0;
                res_c = unsafe {
                    cref_drc_reader_gain(
                        hc,
                        buf_c.as_mut_ptr(),
                        BUF_SIZE as u32,
                        valid,
                        coef_index,
                        frame_size,
                        delta_tmin_default,
                        sub,
                        &mut lc,
                    )
                };
                let mut bs = FdkBitStream::default();
                bs.init(&BitMem::from_slice(&buf_r), BUF_SIZE as u32, valid, FdkBsCfg::BsReader);
                let coef = if coef_index >= 0 {
                    Some(r.cfg.drc_coefficients_uni_drc[coef_index as usize].clone())
                } else {
                    None
                };
                res_r = drc_dec_read_uni_drc_gain(
                    &mut bs,
                    coef.as_ref(),
                    frame_size,
                    delta_tmin_default,
                    sub,
                    &mut r.gain,
                );
                left_c = lc;
                left_r = bs.get_valid_bits();
            }
        }
        let ctx = format!("seed {seed} step {step} op {op} sub {sub}");
        /* coverage statistics: DRC_STATS=1 cargo test ... -- --nocapture */
        if std::env::var("DRC_STATS").is_ok() {
            let kind = if op <= 3 {
                0
            } else if op <= 5 {
                1
            } else {
                2
            };
            let nseq: u32 = r.gain.n_decoded_sequences.iter().map(|&v| v as u32).sum();
            eprintln!(
                "STAT {kind} {} {} {} {}",
                res_r,
                r.cfg.drc_instructions_uni_drc_count,
                r.lis.loudness_info_count,
                if kind == 2 { nseq } else { 0 }
            );
        }
        assert_eq!(res_c, res_r, "{ctx}: error code");
        assert_eq!(left_c, left_r, "{ctx}: bitstream position");
        assert_same(&ser_c(hc), &ser_rust(&r), &ctx);
    }
    unsafe { cref_drc_reader_free(hc) };
}

#[test]
fn drc_reader_random() {
    let n = if cfg!(debug_assertions) { 600 } else { 3000 };
    for seed in 0..n {
        run(seed);
    }
}

#[test]
fn drc_huffman_tables_are_complete() {
    use mpeghdec_drc::drc_dec_rom::*;
    assert_eq!(huffman_codes(&DELTA_GAIN_CODING_PROFILE_0_1_HUFFMAN).len(), 25);
    assert_eq!(huffman_codes(&DELTA_GAIN_CODING_PROFILE_2_HUFFMAN).len(), 49);
    assert_eq!(huffman_codes(&SLOPE_STEEPNESS_HUFFMAN).len(), 15);
}
