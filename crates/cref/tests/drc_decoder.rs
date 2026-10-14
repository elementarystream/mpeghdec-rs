// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of the libDRCdec decoder API (FDK_drcDecLib.cpp): both decoders are driven
//! through the same random API sequence (parameters, selection process requests, config /
//! loudness / gain payloads, channel gains, preprocessing and processing in the time and STFT
//! domain). Return values, output audio and the complete internal state (bitstream structures,
//! selection process output and gain decoder) are compared after every call.
#![cfg(not(cref_missing))]

mod drc_common;

use drc_common::*;
use mpeghdec_cref::drc::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_drc::drc_dec_tools::get_delta_tmin;
use mpeghdec_drc::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::MINVAL_DBL;

const BUF_SIZE: usize = 4096;

struct Pair {
    c: *mut (),
    r: Box<DrcDecoder>,
    seed: u64,
    step: usize,
}

impl Pair {
    fn ctx(&self, what: &str) -> String {
        format!("seed {} step {} {what}", self.seed, self.step)
    }

    fn check_state(&mut self, what: i32, label: &str) {
        let mut v = vec![0i32; 400_000];
        let n = unsafe { cref_drcdec_ser(self.c, what, v.as_mut_ptr()) };
        v.truncate(n as usize);
        let r = ser_decoder(&self.r, what);
        assert_same(&v, &r, &self.ctx(label));
    }

    fn check_params(&mut self) {
        for t in 0..=20 {
            let c = unsafe { cref_drcdec_get_param(self.c, t) };
            let r = self.r.get_param(t);
            assert_eq!(c, r, "{}", self.ctx(&format!("get_param {t}")));
        }
        for gid in [0, 1, 2, 3, 5] {
            let mut ac = 0;
            let c = unsafe { cref_drcdec_get_group_loudness(self.c, gid, &mut ac) };
            let mut ar = 0;
            let r = get_group_loudness(Some(self.r.get_selection_process_output()), gid, &mut ar);
            assert_eq!((c, ac), (r, ar), "{}", self.ctx("get_group_loudness"));
        }
    }

    /// Feeds the same payload to both decoders with one of the read functions.
    fn read(&mut self, kind: i32, g: &mut Rng, w: &BitWriter, sub: i32) {
        let mut buf_c = vec![0u8; BUF_SIZE];
        let n = w.bytes.len().min(BUF_SIZE);
        buf_c[..n].copy_from_slice(&w.bytes[..n]);
        let mut valid = (w.bits as u32).min((BUF_SIZE * 8) as u32);
        if g.range(0, 19) == 0 {
            /* truncated payload */
            valid = g.range(0, valid as i32) as u32;
        }
        let buf_r = buf_c.clone();
        let mut left_c = 0;
        let res_c = unsafe {
            match kind {
                0 => cref_drcdec_read_config(
                    self.c,
                    buf_c.as_mut_ptr(),
                    BUF_SIZE as u32,
                    valid,
                    sub,
                    &mut left_c,
                ),
                1 => cref_drcdec_read_loudness(
                    self.c,
                    buf_c.as_mut_ptr(),
                    BUF_SIZE as u32,
                    valid,
                    sub,
                    &mut left_c,
                ),
                _ => cref_drcdec_read_gain(
                    self.c,
                    buf_c.as_mut_ptr(),
                    BUF_SIZE as u32,
                    valid,
                    sub,
                    &mut left_c,
                ),
            }
        };
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&buf_r), BUF_SIZE as u32, valid, FdkBsCfg::BsReader);
        let res_r = match kind {
            0 => self.r.read_uni_drc_config(&mut bs, sub),
            1 => self.r.read_loudness_info_set(&mut bs, sub),
            _ => self.r.read_uni_drc_gain(&mut bs, sub),
        };
        let ctx = self.ctx(&format!("read {kind}"));
        assert_eq!(res_c, res_r, "{ctx}: error code");
        assert_eq!(left_c, bs.get_valid_bits(), "{ctx}: bitstream position");
        self.check_state(7, &format!("read {kind} state"));
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe { cref_drcdec_close(self.c) };
    }
}

fn random_pcm(g: &mut Rng, n: usize) -> Vec<i32> {
    let shift = g.range(0, 8) as u32;
    (0..n)
        .map(|_| match g.range(0, 63) {
            0 => [i32::MAX, i32::MIN, 0, 1, -1][g.range(0, 4) as usize],
            _ => (g.next_u32() as i32) >> (shift + g.range(0, 3) as u32),
        })
        .collect()
}

fn set_random_param(p: &mut Pair, g: &mut Rng, frame_size: i32) {
    let t = g.range(0, 20);
    let v: i32 = match t {
        DRC_DEC_BOOST | DRC_DEC_COMPRESS => match g.range(0, 5) {
            0 => g.i32(),
            1 => 0,
            2 => 0x4000_0000,
            _ => g.range(0, 0x4000_0000),
        },
        DRC_DEC_LOUDNESS_NORMALIZATION_ON | DRC_DEC_ALBUM_MODE => g.range(-1, 2),
        DRC_DEC_TARGET_LOUDNESS => match g.range(0, 4) {
            0 => g.i32(),
            _ => -(g.range(0, 70) << 24) - g.range(0, 1 << 24),
        },
        DRC_DEC_EFFECT_TYPE => g.range(-2, 10),
        DRC_DEC_EFFECT_TYPE_FALLBACK_CODE => match g.range(0, 5) {
            0 => -1,
            1 => 0,
            2 => g.i32(),
            _ => {
                let mut v = 0u32;
                for i in 0..g.range(1, 8) {
                    v |= (g.range(1, 8) as u32) << (4 * i);
                }
                v as i32
            }
        },
        DRC_DEC_DOWNMIX_ID => g.range(-1, 8),
        DRC_DEC_SAMPLE_RATE => [48000, 44100, 32000, -1, 0][g.range(0, 4) as usize],
        /* (other frame sizes would make the C code access the test buffers out of bounds) */
        DRC_DEC_FRAME_SIZE => [frame_size, -1][g.range(0, 1) as usize],
        _ => g.range(-5, 100),
    };
    let rc = unsafe { cref_drcdec_set_param(p.c, t, v) };
    let rr = p.r.set_param(t, v);
    assert_eq!(rc, rr, "{}", p.ctx(&format!("set_param {t} {v}")));
    p.check_state(7, "set_param");
}

fn set_mpegh_params(p: &mut Pair, g: &mut Rng) {
    if g.range(0, 1) == 0 {
        let preset = g.range(-1, 4);
        let n = g.range(0, 4);
        let ids: Vec<i32> = (0..n.max(1)).map(|_| g.range(0, 4)).collect();
        let rc = unsafe { cref_drcdec_set_mpegh_params_simple(p.c, preset, n, ids.as_ptr()) };
        let rr = p.r.set_selection_process_mpegh_parameters_simple(preset, n, &ids);
        assert_eq!(rc, rr, "{}", p.ctx("mpegh_simple"));
    } else {
        let ng = if g.range(0, 9) == 0 { 30 } else { g.range(0, 5) };
        let np = if g.range(0, 9) == 0 { 17 } else { g.range(0, 4) };
        let gids: Vec<i32> = (0..28).map(|_| g.range(0, 5)).collect();
        let pids: Vec<i32> = (0..16).map(|_| g.range(0, 5)).collect();
        let nm: Vec<i32> = (0..16).map(|_| g.range(0, 4)).collect();
        let pref = g.range(-1, 4);
        let rc = unsafe {
            cref_drcdec_set_mpegh_params(
                p.c,
                ng,
                gids.as_ptr(),
                np,
                pids.as_ptr(),
                nm.as_ptr(),
                pref,
            )
        };
        let rr = p.r.set_selection_process_mpegh_parameters(ng, &gids, np, &pids, &nm, pref);
        assert_eq!(rc, rr, "{}", p.ctx("mpegh"));
    }
    p.check_state(7, "mpegh params");
}

fn run(seed: u64) {
    let mut g = Rng::new(seed);
    let range =
        [DRC_DEC_ALL, DRC_DEC_ALL, DRC_DEC_ALL, DRC_DEC_ALL, DRC_DEC_SELECTION, DRC_DEC_GAIN]
            [g.range(0, 5) as usize];
    let c = unsafe { cref_drcdec_open(range) };
    assert!(!c.is_null());
    let mut p = Pair { c, r: DrcDecoder::open(range), seed, step: 0 };
    p.check_state(15, "open");

    let codec = [
        DRC_DEC_MPEG_H_3DA,
        DRC_DEC_MPEG_H_3DA,
        DRC_DEC_MPEG_H_3DA,
        DRC_DEC_MPEG_H_3DA,
        DRC_DEC_TEST_TIME_DOMAIN,
        DRC_DEC_TEST_STFT_DOMAIN,
        DRC_DEC_TEST_QMF_DOMAIN,
        DRC_DEC_MPEG_4_AAC,
    ][g.range(0, 7) as usize];
    for _ in 0..g.range(1, 2) {
        let rc = unsafe { cref_drcdec_set_codec_mode(p.c, codec) };
        let rr = p.r.set_codec_mode(codec);
        assert_eq!(rc, rr, "{}", p.ctx("set_codec_mode"));
    }
    if g.range(0, 9) == 0 {
        let rc = unsafe { cref_drcdec_set_codec_mode(p.c, DRC_DEC_MPEG_D_USAC) };
        let rr = p.r.set_codec_mode(DRC_DEC_MPEG_D_USAC);
        assert_eq!(rc, rr, "{}", p.ctx("set_codec_mode 2"));
    }

    let frame_size = [1024, 1024, 1024, 1024, 512, 768][g.range(0, 5) as usize];
    let sample_rate = [48000, 48000, 44100, 32000, 16000][g.range(0, 4) as usize];
    let channels = g.range(1, 8);
    let rc = unsafe { cref_drcdec_init(p.c, frame_size, sample_rate, channels) };
    let rr = p.r.init(frame_size, sample_rate, channels);
    assert_eq!(rc, rr, "{}", p.ctx("init"));
    p.check_state(15, "init");

    /* request a DRC effect type (otherwise mostly the virtual "no DRC" set is selected) */
    if g.range(0, 9) != 0 {
        let (t, v) = if g.range(0, 3) == 0 {
            (
                DRC_DEC_EFFECT_TYPE_FALLBACK_CODE,
                g.range(1, 6) | (g.range(0, 6) << 4) | (g.range(0, 6) << 8),
            )
        } else {
            (DRC_DEC_EFFECT_TYPE, g.range(1, 6))
        };
        let rc = unsafe { cref_drcdec_set_param(p.c, t, v) };
        let rr = p.r.set_param(t, v);
        assert_eq!(rc, rr, "{}", p.ctx("effect type"));
    }

    let nice_bias = g.range(60, 100);
    let multiband = g.range(0, 2) == 0;
    let mut downmix: Vec<(u8, u8)> = Vec::new();
    let n_frames = g.range(8, 24);
    let stride = frame_size as usize + 256;
    let mut max_sub = 1;

    for frame in 0..n_frames {
        p.step = frame as usize;
        let nice = g.range(0, 99) < nice_bias;

        /* configuration changes */
        if frame == 0 || g.range(0, 7) == 0 {
            if g.range(0, 5) == 0 {
                let n = g.range(0, 3);
                downmix = (0..n).map(|i| ((i + 1) as u8, g.range(1, 6) as u8)).collect();
                let ids: Vec<i32> = downmix.iter().map(|d| d.0 as i32).collect();
                let layouts: Vec<i32> = (0..n).map(|_| g.range(1, 20)).collect();
                let counts: Vec<i32> = downmix.iter().map(|d| d.1 as i32).collect();
                let rc = unsafe {
                    cref_drcdec_set_downmix_instructions(
                        p.c,
                        n,
                        ids.as_ptr(),
                        layouts.as_ptr(),
                        counts.as_ptr(),
                    )
                };
                let rr = p.r.set_downmix_instructions(n, &ids, &layouts, &counts);
                assert_eq!(rc, rr, "{}", p.ctx("downmix instructions"));
                p.check_state(7, "downmix instructions");
            }
            for _ in 0..g.range(0, 3) {
                set_random_param(&mut p, &mut g, frame_size);
            }
            if g.range(0, 2) == 0 {
                set_mpegh_params(&mut p, &mut g);
            }
            /* main stream and (sometimes) side streams */
            let n_sub = if g.range(0, 4) == 0 { g.range(2, 4) } else { 1 };
            for sub in 0..n_sub {
                let params = ConfigParams {
                    channels: if !nice {
                        g.range(0, 30) as u32
                    } else if sub == 0 {
                        channels as u32
                    } else {
                        g.range(1, 3) as u32
                    },
                    nice,
                    multiband,
                    downmix: downmix.clone(),
                    sub: sub as usize,
                };
                let mut w = BitWriter::new();
                let set_ids = write_uni_drc_config(&mut w, &mut g, &params);
                p.read(0, &mut g, &w, sub);
                if g.range(0, 2) == 0 {
                    let mut w = BitWriter::new();
                    write_loudness_info_set(&mut w, &mut g, nice, &set_ids, sub as usize);
                    p.read(1, &mut g, &w, sub);
                }
            }
            max_sub = n_sub;
            p.check_state(15, "config");
            p.check_params();
        }

        /* gain payloads (one per substream, sometimes missing) */
        for sub in 0..max_sub {
            if g.range(0, 9) == 0 {
                continue;
            }
            let mut w = BitWriter::new();
            if let Some(h) = p.r.h_gain_dec.as_deref() {
                let coef = h.drc_coef.clone();
                let dtd = get_delta_tmin(sample_rate);
                let gain_nice = nice || g.range(0, 1) == 0;
                write_uni_drc_gain(&mut w, &mut g, &coef, frame_size, dtd, sub as usize, gain_nice);
            } else {
                w.put(g.next_u32(), 32);
            }
            p.read(2, &mut g, &w, sub);
        }

        /* channel gains (applied to the time signal if no DRC set is active) */
        let nch = channels.min(28);
        let gains: Vec<i32> = (0..nch)
            .map(|_| match g.range(0, 9) {
                0 => MINVAL_DBL,
                1 => 0,
                _ => g.range(-30 << 24, 10 << 24) >> 7 << 7 >> g.range(0, 1),
            })
            .collect();
        let mut audio_c = random_pcm(&mut g, stride * nch as usize + 16);
        let mut audio_r = audio_c.clone();
        let upd = g.range(0, 1);
        if range & DRC_DEC_GAIN != 0 {
            /* (the C code crashes for decoders without gain decoder) */
            unsafe {
                cref_drcdec_set_channel_gains(
                    p.c,
                    upd,
                    nch,
                    frame_size,
                    gains.as_ptr() as *mut i32,
                    audio_c.as_mut_ptr(),
                    stride as i32,
                )
            };
            p.r.set_channel_gains(upd, nch, frame_size, &gains, &mut audio_r, stride as i32);
        }
        assert_same(&audio_c, &audio_r, &p.ctx("set_channel_gains audio"));
        p.check_state(7, "set_channel_gains");

        if g.range(0, 19) == 0 {
            /* process without preprocess */
            let rc = unsafe {
                cref_drcdec_process_time(
                    p.c,
                    0,
                    DRC_DEC_DRC1,
                    0,
                    0,
                    1,
                    audio_c.as_mut_ptr(),
                    stride as i32,
                )
            };
            let rr = p.r.process_time(0, DRC_DEC_DRC1, 0, 0, 1, &mut audio_r, stride as i32);
            assert_eq!(rc, rr, "{}", p.ctx("process before preprocess"));
        }

        let rc = unsafe { cref_drcdec_preprocess(p.c) };
        let rr = p.r.preprocess();
        assert_eq!(rc, rr, "{}", p.ctx("preprocess"));
        p.check_state(7, "preprocess");

        /* DRC 1 in the time domain, split into signal groups */
        let mut ch = 0;
        while ch < nch {
            let n = g.range(1, nch - ch);
            let drc_off = if g.range(0, 4) == 0 { g.range(-2, 3) } else { 0 };
            let delay = if g.range(0, 4) == 0 { g.range(0, 2 * frame_size + 8) } else { 0 };
            let rc = unsafe {
                cref_drcdec_process_time(
                    p.c,
                    delay,
                    DRC_DEC_DRC1,
                    ch,
                    drc_off,
                    n,
                    audio_c.as_mut_ptr(),
                    stride as i32,
                )
            };
            let audio_before = audio_r.clone();
            let rr =
                p.r.process_time(delay, DRC_DEC_DRC1, ch, drc_off, n, &mut audio_r, stride as i32);
            assert_eq!(rc, rr, "{}", p.ctx("process_time DRC1"));
            assert_same(&audio_c, &audio_r, &p.ctx("process_time DRC1 audio"));
            /* coverage statistics: DRC_STATS=1 cargo test ... -- --nocapture */
            if std::env::var("DRC_STATS").is_ok() {
                let o = p.r.get_selection_process_output();
                let mb = p.r.h_gain_dec.as_deref().map_or(-2, |h| h.multi_band_active_drc_index);
                eprintln!(
                    "STAT rc={rr} ninst={} eff={} nsel={} ids={:?} mb={mb} ln={} changed={}",
                    p.r.uni_drc_config.drc_instructions_uni_drc_count,
                    p.r.h_selection_proc.as_deref().map_or(-1, |s| s
                        .sel_proc_input
                        .drc_feature_request[0]
                        .request[0]
                        * 10
                        + s.sel_proc_input.num_drc_feature_requests as i32),
                    o.num_selected_drc_sets,
                    &o.selected_drc_set_ids[..o.num_selected_drc_sets.min(3) as usize],
                    o.loudness_normalization_gain_db,
                    audio_before != audio_r
                );
            }
            ch += n;
        }

        /* DRC 1 in the STFT domain */
        if g.range(0, 1) == 0 {
            let slots = (frame_size / 256).max(1);
            let n = g.range(1, nch);
            let ch_off = g.range(0, nch - n);
            let separate_imag = g.range(0, 2) == 0;
            let all_slots = g.range(0, 4) == 0;
            let len =
                if separate_imag { 256 } else { 512 } * if all_slots { slots as usize } else { 1 };
            let slot_list: Vec<i32> = if all_slots { vec![-1] } else { (0..slots).collect() };
            let delay = if g.range(0, 4) == 0 { g.range(0, 2 * frame_size + 8) } else { 128 };
            let loc = if g.range(0, 5) == 0 { DRC_DEC_DRC2_DRC3 } else { DRC_DEC_DRC1 };
            for &slot in &slot_list {
                let mut re_c = random_pcm(&mut g, len * n as usize);
                let mut im_c =
                    if separate_imag { random_pcm(&mut g, len * n as usize) } else { Vec::new() };
                let mut re_r: Vec<Vec<i32>> = re_c.chunks(len).map(|c| c.to_vec()).collect();
                let mut im_r: Vec<Vec<i32>> = im_c.chunks(len).map(|c| c.to_vec()).collect();
                let re_before = re_c.clone();
                let rc = unsafe {
                    cref_drcdec_process_freq(
                        p.c,
                        delay,
                        loc,
                        ch_off,
                        0,
                        n,
                        slot,
                        re_c.as_mut_ptr(),
                        if separate_imag { im_c.as_mut_ptr() } else { std::ptr::null_mut() },
                        len as i32,
                    )
                };
                let rr = p.r.process_freq(
                    delay,
                    loc,
                    ch_off,
                    0,
                    n,
                    slot,
                    &mut re_r,
                    if separate_imag { Some(&mut im_r[..]) } else { None },
                );
                assert_eq!(rc, rr, "{}", p.ctx("process_freq"));
                assert_same(&re_c, &re_r.concat(), &p.ctx("process_freq real"));
                if std::env::var("DRC_STATS").is_ok() {
                    let mb =
                        p.r.h_gain_dec.as_deref().map_or(-2, |h| h.multi_band_active_drc_index);
                    eprintln!("FSTAT rc={rr} mb={mb} changed={}", re_before != re_r.concat());
                }
                if separate_imag {
                    assert_same(&im_c, &im_r.concat(), &p.ctx("process_freq imag"));
                }
            }
        }

        /* DRC 2/3 after downmix */
        let nout = g.range(1, nch);
        let mut out_c = random_pcm(&mut g, frame_size as usize * nout as usize);
        let mut out_r = out_c.clone();
        let delay = if g.range(0, 4) == 0 { g.range(0, 2 * frame_size + 8) } else { 256 };
        let rc = unsafe {
            cref_drcdec_process_time(
                p.c,
                delay,
                DRC_DEC_DRC2_DRC3,
                0,
                0,
                nout,
                out_c.as_mut_ptr(),
                frame_size,
            )
        };
        let rr = p.r.process_time(delay, DRC_DEC_DRC2_DRC3, 0, 0, nout, &mut out_r, frame_size);
        assert_eq!(rc, rr, "{}", p.ctx("process_time DRC2/3"));
        assert_same(&out_c, &out_r, &p.ctx("process_time DRC2/3 audio"));

        p.check_state(if frame % 4 == 0 { 15 } else { 7 }, "end of frame");
        p.check_params();
    }
}

#[test]
fn drc_decoder_random() {
    let n = if cfg!(debug_assertions) { 300 } else { 2000 };
    for seed in 0..n {
        run(seed);
    }
}
