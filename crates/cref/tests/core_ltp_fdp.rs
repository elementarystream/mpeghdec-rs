// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libMpeghDec ltp_post.cpp (readLtpParam(), ltp_post() over long frame
//! sequences with persistent state) and fdp.cpp (FDP_DecodeBins() over frame sequences with
//! persistent quantSpecPrev1/2).
#![cfg(not(cref_missing))]

mod core_common;

use core_common::*;
use mpeghdec_core::aac_rom::SFB_OFFSET_TABLES;
use mpeghdec_core::fdp::fdp_decode_bins;
use mpeghdec_core::ltp_post::*;
use mpeghdec_cref::core_common::cref_core_ch_set_spec;
use mpeghdec_cref::core_ltp_fdp::*;
use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::{AC_INDEP, AC_MPEGH3DA};
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::mdct::{BLOCK_LONG, BLOCK_SHORT, BLOCK_START, BLOCK_STOP};

// ---------------------------------------------------------------------------------------------
// readLtpParam()
// ---------------------------------------------------------------------------------------------

#[test]
fn read_ltp_param_random_bits() {
    let mut rng = Rng::new(0x17f);
    for iter in 0..3000 {
        let buf: Vec<u8> = (0..16).map(|_| rng.next_u32() as u8).collect();
        let mut buf_c = buf.clone();
        let size = buf.len() as u32;
        let valid = size * 8;
        let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, valid, 0) };
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&buf), size, valid, FdkBsCfg::BsReader);

        // stale values survive if no data is present (or for LFE)
        let mut p_r = [rng.next_u32() as i32, rng.next_u32() as i32, rng.next_u32() as i32];
        let mut p_c = p_r;
        for call in 0..8 {
            let is_lfe = (rng.next_u32() % 3 == 0) as i32;
            let ret_c = unsafe { cref_core_ltp_read_ltp_param(hc, p_c.as_mut_ptr(), is_lfe) };
            let ret_r = read_ltp_param(&mut bs, &mut p_r, is_lfe);
            assert_eq!(ret_c, ret_r, "iter {iter} call {call}");
            assert_eq!(p_c, p_r, "iter {iter} call {call}");
            assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits(), "iter {iter}");
        }
        unsafe { cref_bs_free(hc) };
    }
}

// ---------------------------------------------------------------------------------------------
// ltp_post()
// ---------------------------------------------------------------------------------------------

/// Sampling rates: the MPEG-H core rates, all sfb table rates up to 48 kHz and some odd ones.
/// (Above 48 kHz the decoded pitch lag can exceed the output memory, the C++ code then reads
/// before its buffer; MPEG-H only allows core sampling rates up to 48 kHz.)
const LTP_RATES: [i32; 18] = [
    48000, 44100, 32000, 29400, 24000, 22050, 16000, 14700, 12000, 11025, 8000, 7350, 47999, 37800,
    25600, 12800, 9600, 6000,
];

/// LTP state of one channel (the `ltp_*` fields of `CAacDecoderStaticChannelInfo`).
#[derive(Clone, PartialEq, Debug)]
struct LtpState {
    param: [i32; 3],
    pitch_int_past: i32,
    pitch_fr_past: i32,
    gain_past: i16,
    gain_idx_past: i32,
    mem_in: Vec<i16>,
    mem_out: Vec<i16>,
}

impl LtpState {
    fn new() -> Self {
        LtpState {
            param: [0; 3],
            pitch_int_past: 0,
            pitch_fr_past: 0,
            gain_past: 0,
            gain_idx_past: 0,
            mem_in: vec![0; LTP_MEM_IN_SIZE],
            mem_out: vec![0; LTP_MEM_OUT_SIZE],
        }
    }
}

/// Decoded gain of `ltp_decode_params()` after `gain >>= 2` (to classify the transition case).
fn ltp_gain(p: &[i32; 3]) -> i16 {
    if p[0] == 0 {
        0
    } else if p[2] >= 3 {
        0x7fff >> 2
    } else {
        (((p[2] + 1) << 13) as i16) >> 2
    }
}

/// Random frame of `n` values of one of several signal kinds.
fn ltp_signal(rng: &mut Rng, n: usize) -> Vec<i32> {
    let kind = rng.next_u32() % 9;
    let amp = match rng.next_u32() % 4 {
        0 => 1 << 23,
        1 => 1 << 20,
        2 => 1 << 14,
        _ => 1 << 22,
    };
    let period = rng.range(20, 900) as f64;
    let phase = rng.range(0, 1000) as f64;
    (0..n)
        .map(|i| match kind {
            // pitched signal (sine + harmonic)
            0 | 1 => {
                let t = (i as f64 + phase) * 2.0 * std::f64::consts::PI / period;
                ((t.sin() * 0.7 + (2.0 * t).sin() * 0.25) * amp as f64) as i32
            }
            // pitched signal with noise
            2 => {
                let t = (i as f64 + phase) * 2.0 * std::f64::consts::PI / period;
                (t.sin() * amp as f64) as i32 + ((rng.next_u32() as i32) >> 12)
            }
            // noise of random magnitude
            3 => (rng.next_u32() as i32) >> rng.range(6, 20),
            // near saturation of the 16 bit input (+-2^23 after the headroom)
            4 => {
                let v = (1 << 23) - rng.range(0, 3000);
                if rng.next_u32() % 2 == 0 {
                    v
                } else {
                    -v
                }
            }
            // square wave at full scale
            5 => {
                if ((i as f64 + phase) / (period / 2.0)) as i64 % 2 == 0 {
                    (1 << 23) - 1
                } else {
                    -(1 << 23)
                }
            }
            // full range values (saturate in the conversion to 16 bit)
            6 => rng.i32(),
            // silence with sparse pulses
            7 if rng.next_u32() % 50 == 0 => rng.i32() >> 7,
            _ => 0,
        })
        .collect()
}

#[test]
fn ltp_post_frame_sequences() {
    let mut rng = Rng::new(0x1705);
    // pass-through, previous-only, fade-in, fade-out, same parameters, ZIR
    let mut cases = [0usize; 6];
    for seq in 0..120 {
        let fs = LTP_RATES[seq % LTP_RATES.len()];
        let l_frame: i32 = if seq % 4 == 3 { 768 } else { 1024 };
        let mut st_r = LtpState::new();
        let mut st_c = LtpState::new();
        let n_frames = 60;
        let p_on = rng.range(20, 90) as u32;
        let p_same = rng.range(0, 80) as u32;
        for frame in 0..n_frames {
            // New LTP parameters: keep, change pitch and/or gain or switch on/off. ltp_param[1]
            // and [2] keep their stale values if the data is not present.
            let mut p = st_r.param;
            p[0] = (rng.next_u32() % 100 < p_on) as i32;
            if rng.next_u32() % 100 >= p_same {
                match rng.next_u32() % 4 {
                    0 => p[1] = rng.range(0, 511),
                    1 => p[2] = rng.range(0, 3),
                    2 => {
                        p[1] = rng.range(0, 511);
                        p[2] = rng.range(0, 3);
                    }
                    _ => p[1] = [0, 1, 2, 3, 255, 256, 509, 510, 511][rng.range(0, 8) as usize],
                }
            }
            // L_frame 768: the pass-through case reads before sig (L_frame < LTP_MEM_OUT_SIZE),
            // so it is avoided.
            if l_frame < LTP_MEM_OUT_SIZE as i32 && st_r.gain_past == 0 {
                p[0] = 1;
            }
            st_r.param = p;
            st_c.param = p;

            let gain = ltp_gain(&p);
            let gain_past = st_r.gain_past;
            let (pi_past, pf_past) = (st_r.pitch_int_past, st_r.pitch_fr_past);

            let sig = ltp_signal(&mut rng, l_frame as usize);
            let mut sig_r = sig.clone();
            let mut sig_c = sig.clone();

            ltp_post(
                &mut sig_r,
                l_frame,
                fs,
                &st_r.param,
                &mut st_r.pitch_int_past,
                &mut st_r.pitch_fr_past,
                &mut st_r.gain_past,
                &mut st_r.gain_idx_past,
                &mut st_r.mem_in,
                &mut st_r.mem_out,
            );
            unsafe {
                cref_core_ltp_post(
                    sig_c.as_mut_ptr(),
                    l_frame,
                    fs,
                    st_c.param.as_mut_ptr(),
                    &mut st_c.pitch_int_past,
                    &mut st_c.pitch_fr_past,
                    &mut st_c.gain_past,
                    &mut st_c.gain_idx_past,
                    st_c.mem_in.as_mut_ptr(),
                    st_c.mem_out.as_mut_ptr(),
                )
            };
            let what = format!("seq {seq} fs {fs} L {l_frame} frame {frame} param {p:?}");
            assert_same(&format!("{what}: signal"), &sig_c, &sig_r);
            assert_eq!(st_c, st_r, "{what}: state");

            // classify the transition case
            let case = if gain_past == 0 && p[0] == 0 {
                0
            } else if gain == 0 && gain_past == 0 {
                1
            } else if gain_past == 0 {
                2
            } else if gain == 0 {
                3
            } else if gain == gain_past
                && st_r.pitch_int_past == pi_past
                && st_r.pitch_fr_past == pf_past
            {
                4
            } else {
                5
            };
            cases[case] += 1;
        }
    }
    // Every transition branch must have been hit (case 1 cannot occur: gain == 0 implies
    // ltp_param[0] == 0, which with gain_past == 0 is the pass-through case).
    for (i, &n) in cases.iter().enumerate() {
        if i != 1 {
            assert!(n > 50, "transition case {i} hit only {n} times: {cases:?}");
        }
    }
}

/// Starts from random (but consistent) states: random memories and past parameters decoded
/// from random ltp_param values.
#[test]
fn ltp_post_random_states() {
    let mut rng = Rng::new(0x1706);
    for iter in 0..400 {
        let fs = LTP_RATES[(rng.next_u32() as usize) % LTP_RATES.len()];
        let l_frame = 1024;
        let mut st_r = LtpState::new();
        for v in st_r.mem_in.iter_mut().chain(st_r.mem_out.iter_mut()) {
            *v = rng.i16();
        }
        // run one frame to get a consistent past state
        st_r.param = [1, rng.range(0, 511), rng.range(0, 3)];
        let mut sig = ltp_signal(&mut rng, l_frame as usize);
        let mut st_c = st_r.clone();
        let mut sig_c = sig.clone();
        for frame in 0..3 {
            if frame > 0 {
                st_r.param = [rng.range(0, 1), rng.range(0, 511), rng.range(0, 3)];
                st_c.param = st_r.param;
                sig = ltp_signal(&mut rng, l_frame as usize);
                sig_c = sig.clone();
            }
            ltp_post(
                &mut sig,
                l_frame,
                fs,
                &st_r.param,
                &mut st_r.pitch_int_past,
                &mut st_r.pitch_fr_past,
                &mut st_r.gain_past,
                &mut st_r.gain_idx_past,
                &mut st_r.mem_in,
                &mut st_r.mem_out,
            );
            unsafe {
                cref_core_ltp_post(
                    sig_c.as_mut_ptr(),
                    l_frame,
                    fs,
                    st_c.param.as_mut_ptr(),
                    &mut st_c.pitch_int_past,
                    &mut st_c.pitch_fr_past,
                    &mut st_c.gain_past,
                    &mut st_c.gain_idx_past,
                    st_c.mem_in.as_mut_ptr(),
                    st_c.mem_out.as_mut_ptr(),
                )
            };
            let what = format!("iter {iter} fs {fs} frame {frame}");
            assert_same(&format!("{what}: signal"), &sig_c, &sig);
            assert_eq!(st_c, st_r, "{what}: state");
        }
    }
}

// ---------------------------------------------------------------------------------------------
// FDP_DecodeBins()
// ---------------------------------------------------------------------------------------------

/// Sampling rates of the sfb offset tables (index = sampling rate index).
const SFB_RATES: [i32; 13] =
    [96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350];

fn quant_value(rng: &mut Rng) -> i32 {
    let v = match rng.next_u32() % 16 {
        0..=5 => 0,
        6..=9 => rng.range(1, 4),
        10..=11 => rng.range(1, 40),
        12 => rng.range(170, 190),
        13 => rng.range(1, 8191),
        14 => [181, 182, 8191, 8192, 100000, i32::MAX][rng.range(0, 5) as usize],
        _ => rng.range(1, 400),
    };
    if rng.next_u32() % 2 == 0 {
        v
    } else if v == i32::MAX {
        i32::MIN
    } else {
        -v
    }
}

fn spec_value(rng: &mut Rng, mag: u32) -> i32 {
    match rng.next_u32() % 8 {
        0 => 0,
        1 => rng.i32(),
        _ => (rng.next_u32() as i32) >> mag,
    }
}

#[test]
fn fdp_decode_bins_frame_sequences() {
    let mut rng = Rng::new(0xfd9);
    let mut active_frames = 0;
    let mut adjust_frames = 0;
    for seq in 0..150 {
        let sr_idx = seq % SFB_RATES.len();
        let fs = SFB_RATES[sr_idx];
        let tab = &SFB_OFFSET_TABLES[0][sr_idx];
        if tab.sfb_offset_long.is_empty() {
            continue;
        }
        let mut ch = ChPair::new();
        let mut st = StPair::new();
        let mut spec_c = vec![0i32; 1024];
        unsafe { cref_core_ch_set_spec(ch.c, spec_c.as_mut_ptr()) };
        let bw_mode = rng.next_u32() % 4;
        for frame in 0..40 {
            let short = rng.next_u32() % 8 == 0;
            let is_tcx = (rng.next_u32() % 10 == 0) as i32;
            let (offsets, nsfb) = if short {
                (tab.sfb_offset_short, tab.number_of_sfb_short as i32)
            } else {
                (tab.sfb_offset_long, tab.number_of_sfb_long as i32)
            };
            // predictionBandwidth as in CChannelElement_Decode() for frame lengths 1024 and 768
            let pred_bw = match bw_mode {
                0 | 1 => {
                    if fs >= 44100 {
                        132
                    } else {
                        160
                    }
                }
                2 => (160 * 768) / 1024,
                _ => [132, 160, 120, 100, 64][rng.range(0, 4) as usize],
            };
            let mut max_sfb = match rng.next_u32() % 6 {
                0 => 0,
                1 => nsfb,
                _ => rng.range(0, nsfb),
            };
            if is_tcx != 0 {
                // TCX uses all MaxSfBands; stay within the 160 lines of the FDP state
                while max_sfb > 0 && (offsets[max_sfb as usize] as i32) > pred_bw {
                    max_sfb -= 1;
                }
            }
            let flags = AC_MPEGH3DA | if rng.next_u32() % 10 == 0 { AC_INDEP } else { 0 };

            {
                let c = &mut ch.r;
                c.ics_info.window_sequence = if short {
                    BLOCK_SHORT
                } else {
                    [BLOCK_LONG, BLOCK_START, BLOCK_STOP][rng.range(0, 2) as usize]
                };
                c.ics_info.max_sf_bands = max_sfb as u8;
                c.fdp_data_present = (rng.next_u32() % 5 != 0) as i32;
                c.fdp_spacing_index = match rng.next_u32() % 8 {
                    0 => [-1, 0, 1, 254, 255, 256, 297, 1000, i32::MIN][rng.range(0, 8) as usize],
                    _ => rng.range(0, 255),
                };
                let d = &mut c.p_dyn_data;
                for v in d.a_scale_factor.iter_mut() {
                    *v = match rng.next_u32() % 6 {
                        0 => rng.range(-30, 20),
                        1 => rng.range(84, 200),
                        2 => [0, 20, 21, 22, 83, 84, 85, i16::MIN as i32, i16::MAX as i32]
                            [rng.range(0, 8) as usize],
                        _ => rng.range(21, 83),
                    } as i16;
                }
                for v in d.a_sfb_scale.iter_mut() {
                    *v = match rng.next_u32() % 6 {
                        0 => rng.range(-100, 100),
                        1 => [i16::MIN as i32, i16::MAX as i32, -32, 8, 9, 40, 72, 73]
                            [rng.range(0, 7) as usize],
                        _ => rng.range(0, 30),
                    } as i16;
                }
            }
            ch.sync_to_c();
            if frame == 0 {
                st.sync_to_c();
            }

            let mag = rng.range(0, 31) as u32;
            let spec: Vec<i32> = (0..1024).map(|_| spec_value(&mut rng, mag)).collect();
            spec_c.copy_from_slice(&spec);
            let mut spec_r = spec.clone();
            let quant: Vec<i32> = (0..1024).map(|_| quant_value(&mut rng)).collect();
            let sfb_scale_before = ch.r.p_dyn_data.a_sfb_scale;

            let gain_m = rng.i32();
            let gain_e = rng.range(-10, 10);
            fdp_decode_bins(
                &mut ch.r,
                &mut spec_r,
                &mut st.r,
                flags,
                &quant,
                offsets,
                pred_bw,
                is_tcx,
                gain_m,
                gain_e,
                0,
                0,
            );
            unsafe {
                cref_core_fdp_decode_bins(
                    ch.c,
                    st.c,
                    flags,
                    quant.as_ptr(),
                    offsets.as_ptr(),
                    pred_bw,
                    is_tcx,
                    gain_m,
                    gain_e,
                    0,
                    0,
                )
            };
            let what = format!(
                "seq {seq} fs {fs} frame {frame} short {short} tcx {is_tcx} bw {pred_bw} \
                 max_sfb {max_sfb} flags {flags:#x}"
            );
            assert_same(&format!("{what}: spectrum"), &spec_c, &spec_r);
            ch.check(&what);
            st.check(&what);
            if spec_r != spec {
                active_frames += 1;
            }
            if ch.r.p_dyn_data.a_sfb_scale != sfb_scale_before {
                adjust_frames += 1;
            }
        }
    }
    assert!(active_frames > 300, "FDP active in only {active_frames} frames");
    assert!(adjust_frames > 100, "aSfbScale adjusted in only {adjust_frames} frames");
}
