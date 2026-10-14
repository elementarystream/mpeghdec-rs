// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libMpeghDec conceal.cpp: parameter handling (InitCommonData, SetParams,
//! SetAttenuation, the getters), CConcealment_Store() and the complete concealment state machine
//! (CConcealment_Apply() followed by CConcealment_TDFading() like CAacDecoder_DecodeFrame()) over
//! long random frame sequences, comparing the channel info, the static channel info (concealment
//! state), the spectrum and the PCM output after every call.
#![cfg(not(cref_missing))]

mod core_common;

use core_common::*;
use mpeghdec_core::aacdecoder_lib::AAC_DEC_SET_PARAM_FAIL;
use mpeghdec_core::channelinfo::*;
use mpeghdec_core::conceal::*;
use mpeghdec_core::conceal_types::*;
use mpeghdec_cref::core_common::*;
use mpeghdec_cref::core_conceal::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::mdct::{BLOCK_LONG, BLOCK_SHORT, BLOCK_START, BLOCK_STOP};

/// A C and a Rust `CConcealParams`.
struct ParamsPair {
    c: *mut (),
    r: CConcealParams,
}

impl ParamsPair {
    fn new() -> Self {
        ParamsPair { c: unsafe { cref_core_conceal_params_new() }, r: CConcealParams::default() }
    }
    fn sync_to_c(&mut self) {
        let d = dump_conceal_params(&self.r);
        unsafe { cref_core_conceal_params_load(self.c, d.as_ptr()) };
    }
    #[track_caller]
    fn check(&self, what: &str) {
        let c = c_dump(|o| unsafe { cref_core_conceal_params_dump(self.c, o) });
        assert_same(&format!("{what}: conceal params"), &c, &dump_conceal_params(&self.r));
    }
}

impl Drop for ParamsPair {
    fn drop(&mut self) {
        unsafe { cref_core_conceal_params_free(self.c) };
    }
}

fn random_params(rng: &mut Rng, p: &mut CConcealParams) {
    for v in p.fade_out_factor.iter_mut().chain(p.fade_in_factor.iter_mut()) {
        *v = rng.i16();
    }
    p.method = rng.range(-2, 4);
    p.num_fade_out_frames = rng.range(-2, 40);
    p.num_fade_in_frames = rng.range(-2, 40);
    p.num_mute_release_frames = rng.range(-2, 70);
    p.comfort_noise_level = rng.i32();
}

/// A parameter value for `CConcealment_SetParams()`: not specified, valid, or out of range.
fn param_value(rng: &mut Rng, valid_max: i32) -> i32 {
    match rng.range(0, 9) {
        0..=1 => AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
        2..=5 => rng.range(0, valid_max),
        6 => [0, valid_max, valid_max + 1, -1][rng.range(0, 3) as usize],
        7 => rng.range(-100, 100),
        _ => rng.i32(),
    }
}

#[test]
fn init_common_data() {
    let mut p = ParamsPair::new();
    let mut rng = Rng::new(1);
    for _ in 0..4 {
        random_params(&mut rng, &mut p.r);
        p.sync_to_c();
        unsafe { cref_core_conceal_init_common_data(p.c) };
        cconcealment_init_common_data(&mut p.r);
        p.check("InitCommonData");
    }
}

#[test]
fn set_params() {
    let mut rng = Rng::new(2);
    let mut p = ParamsPair::new();
    for it in 0..200000 {
        random_params(&mut rng, &mut p.r);
        p.sync_to_c();
        let method = match rng.range(0, 5) {
            0 => AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
            1..=3 => rng.range(-2, 4),
            4 => rng.range(0, 1),
            _ => rng.i32(),
        };
        let fade_out = param_value(&mut rng, 31);
        let fade_in = param_value(&mut rng, 31);
        let mute_release = param_value(&mut rng, 63);
        let comf = match rng.range(0, 4) {
            0 => AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
            1 => rng.range(-2, 2),
            _ => rng.i32(),
        };
        let null = rng.range(0, 7) == 0;
        let c_err = unsafe {
            cref_core_conceal_set_params(
                if null { core::ptr::null_mut() } else { p.c },
                method,
                fade_out,
                fade_in,
                mute_release,
                comf,
            )
        };
        let r_err = cconcealment_set_params(
            if null { None } else { Some(&mut p.r) },
            method,
            fade_out,
            fade_in,
            mute_release,
            comf,
        );
        assert_eq!(c_err, r_err, "SetParams {it}: error code");
        p.check(&format!("SetParams {it}"));
    }
}

#[test]
fn set_attenuation() {
    let mut rng = Rng::new(3);
    let mut p = ParamsPair::new();
    for it in 0..50000 {
        random_params(&mut rng, &mut p.r);
        p.sync_to_c();
        let mut vecs = [[0i16; CONCEAL_MAX_NUM_FADE_FACTORS]; 2];
        for v in &mut vecs {
            let mode = rng.range(0, 3);
            for x in v.iter_mut() {
                *x = rng.range(0, 255) as i16;
            }
            match mode {
                0 => {}
                1 => {
                    /* one out-of-range value */
                    let i = rng.range(0, CONCEAL_MAX_NUM_FADE_FACTORS as i32 - 1) as usize;
                    v[i] = [-1, 256, i16::MIN, i16::MAX, rng.i16()][rng.range(0, 4) as usize];
                }
                2 => {
                    /* monotonic like a real attenuation vector (0.25 dB steps) */
                    let mut a = 0;
                    for x in v.iter_mut() {
                        a = (a + rng.range(0, 24)).min(255);
                        *x = a as i16;
                    }
                }
                _ => {
                    for x in v.iter_mut() {
                        *x = [0, 255, rng.range(0, 255)][rng.range(0, 2) as usize] as i16;
                    }
                }
            }
        }
        let use_out = rng.range(0, 3) != 0;
        let use_in = rng.range(0, 3) != 0;
        let null = rng.range(0, 7) == 0;
        let c_err = unsafe {
            cref_core_conceal_set_attenuation(
                if null { core::ptr::null_mut() } else { p.c },
                if use_out { vecs[0].as_ptr() } else { core::ptr::null() },
                if use_in { vecs[1].as_ptr() } else { core::ptr::null() },
            )
        };
        let r_err = cconcealment_set_attenuation(
            if null { None } else { Some(&mut p.r) },
            if use_out { Some(&vecs[0]) } else { None },
            if use_in { Some(&vecs[1]) } else { None },
        );
        assert_eq!(c_err, r_err, "SetAttenuation {it}: error code");
        p.check(&format!("SetAttenuation {it}"));
    }
    /* all quantized values in every position */
    for q in 0..=255i16 {
        let v = [q; CONCEAL_MAX_NUM_FADE_FACTORS];
        let c_err = unsafe { cref_core_conceal_set_attenuation(p.c, v.as_ptr(), v.as_ptr()) };
        let r_err = cconcealment_set_attenuation(Some(&mut p.r), Some(&v), Some(&v));
        assert_eq!(c_err, r_err);
        p.check(&format!("SetAttenuation q={q}"));
    }
}

#[test]
fn getters() {
    let mut rng = Rng::new(4);
    let mut p = ParamsPair::new();
    let mut st = StPair::new();
    for it in 0..20000 {
        random_params(&mut rng, &mut p.r);
        if rng.range(0, 3) == 0 {
            p.r.method = rng.i32();
        }
        p.sync_to_c();
        let null = rng.range(0, 5) == 0;
        let pc = if null { core::ptr::null_mut() } else { p.c };
        let pr = if null { None } else { Some(&p.r) };
        assert_eq!(unsafe { cref_core_conceal_get_method(pc) }, cconcealment_get_method(pr));
        assert_eq!(unsafe { cref_core_conceal_get_delay(pc) }, cconcealment_get_delay(pr));

        let ci = &mut st.r.concealment_info;
        ci.conceal_state = rng.range(0, 4);
        ci.prev_frame_ok = [rng.range(0, 1), rng.range(0, 1)];
        if rng.range(0, 3) == 0 {
            ci.prev_frame_ok = [rng.i32(), rng.i32()];
        }
        st.sync_to_c();
        let sc = if null { core::ptr::null_mut() } else { st.c };
        let sr = if null { None } else { Some(&st.r.concealment_info) };
        assert_eq!(unsafe { cref_core_conceal_get_state(sc) }, cconcealment_get_state(sr));
        let f = rng.i32();
        assert_eq!(
            unsafe { cref_core_conceal_get_last_frame_ok(sc, f) },
            cconcealment_get_last_frame_ok(sr, f),
            "GetLastFrameOk {it}"
        );
    }
}

/// Random `CIcsInfo` of a parsed frame: window sequence, shape and grouping. With
/// `allow_zero_groups`, `WindowGroups` is sometimes 0 (reads outside `WindowGroupLength[]` in
/// `CConcealment_Store()`).
fn random_ics(rng: &mut Rng, ics: &mut CIcsInfo, allow_zero_groups: bool) {
    ics.window_sequence =
        [BLOCK_LONG, BLOCK_START, BLOCK_SHORT, BLOCK_STOP][rng.range(0, 3) as usize];
    ics.window_shape = if rng.range(0, 9) == 0 { 2 } else { rng.range(0, 1) as u8 };
    ics.valid = rng.range(0, 1) as u8;
    ics.max_sf_bands = rng.range(0, 49) as u8;
    ics.total_sf_bands = rng.range(0, 49) as u8;
    ics.max_sfb_ste = rng.range(0, 49) as u8;
    if ics.window_sequence == BLOCK_SHORT {
        /* like IcsRead() */
        let grouping = rng.range(0, 127) as u8;
        ics.scale_factor_grouping = grouping;
        ics.window_groups = 0;
        for i in 0..7 {
            let mask = 1u32 << (6 - i);
            ics.window_group_length[i] = 1;
            if grouping as u32 & mask != 0 {
                ics.window_group_length[ics.window_groups as usize] += 1;
            } else {
                ics.window_groups += 1;
            }
        }
        ics.window_group_length[7] = 1;
        ics.window_groups += 1;
    } else {
        ics.window_groups = 1;
        ics.window_group_length[0] = 1;
        for g in &mut ics.window_group_length[1..] {
            if rng.range(0, 1) == 0 {
                *g = rng.range(0, 8) as u8;
            }
        }
    }
    if allow_zero_groups && rng.range(0, 49) == 0 {
        ics.window_groups = 0;
    }
}

fn random_spectrum(rng: &mut Rng, spec: &mut [i32]) {
    match rng.range(0, 5) {
        0 => spec.fill(0),
        1 => {
            for v in spec.iter_mut() {
                *v = rng.i32();
            }
        }
        2 => {
            for v in spec.iter_mut() {
                *v = [i32::MIN, i32::MIN + 1, i32::MAX, 0, -1, 1][rng.range(0, 5) as usize];
            }
        }
        _ => {
            let sh = rng.range(0, 31);
            for v in spec.iter_mut() {
                *v = (rng.next_u32() as i32) >> sh;
            }
        }
    }
}

fn random_pcm(rng: &mut Rng, pcm: &mut [i32]) {
    let mode = rng.range(0, 5);
    for v in pcm.iter_mut() {
        *v = match mode {
            0 => rng.i32(),
            1 => (rng.next_u32() as i32) >> rng.range(8, 31),
            2 => i32::MAX - rng.range(0, 1 << 20),
            3 => i32::MIN + rng.range(0, 1 << 20),
            4 => match rng.range(0, 3) {
                0 => i32::MAX - rng.range(0, 1 << 12),
                1 => i32::MIN + rng.range(0, 1 << 12),
                _ => rng.i32(),
            },
            _ => 0,
        };
    }
}

/// Random valid concealment parameters (as set through the decoder API).
fn random_valid_params(rng: &mut Rng, p: &mut ParamsPair) {
    let method = if rng.range(0, 3) == 0 { ConcealMethodMute } else { ConcealMethodNoise };
    let fade_out = match rng.range(0, 4) {
        0 => 0,
        1 => 31,
        _ => rng.range(0, 31),
    };
    let fade_in = match rng.range(0, 4) {
        0 => 0,
        1 => 31,
        _ => rng.range(0, 31),
    };
    let mute_release = match rng.range(0, 3) {
        0 => 0,
        1 => rng.range(0, 63),
        _ => rng.range(0, 5),
    };
    let comf = match rng.range(0, 5) {
        0 => 0,
        1 => 0x100000,
        2 => i32::MAX,
        _ => (rng.next_u32() >> 1) as i32 >> rng.range(0, 30),
    };
    let c_err =
        unsafe { cref_core_conceal_set_params(p.c, method, fade_out, fade_in, mute_release, comf) };
    let r_err =
        cconcealment_set_params(Some(&mut p.r), method, fade_out, fade_in, mute_release, comf);
    assert_eq!(c_err, 0);
    assert_eq!(r_err, 0);
    if rng.range(0, 2) == 0 {
        /* random attenuation vectors */
        let mut vecs = [[0i16; CONCEAL_MAX_NUM_FADE_FACTORS]; 2];
        for v in &mut vecs {
            let mut a = 0;
            for x in v.iter_mut() {
                a = (a + rng.range(0, 30)).min(255);
                *x = a as i16;
            }
        }
        let c_err =
            unsafe { cref_core_conceal_set_attenuation(p.c, vecs[0].as_ptr(), vecs[1].as_ptr()) };
        let r_err = cconcealment_set_attenuation(Some(&mut p.r), Some(&vecs[0]), Some(&vecs[1]));
        assert_eq!(c_err, r_err);
    }
    p.check("random_valid_params");
}

/// Generator of frameOk patterns: good runs and loss bursts of random lengths.
struct FramePattern {
    remaining: i32,
    bad: bool,
    /// Loss bursts that end the fade-out of short blocks with `numFadeOutFrames == 31` just
    /// before muting (`cntFadeFrames == 32`).
    max_fade: bool,
}

impl FramePattern {
    fn next(&mut self, rng: &mut Rng) -> i32 {
        while self.remaining == 0 {
            self.bad = !self.bad;
            self.remaining = if self.bad && self.max_fade {
                rng.range(13, 17)
            } else if self.bad {
                match rng.range(0, 9) {
                    0..=3 => 1,
                    4..=6 => rng.range(2, 5),
                    7 => rng.range(6, 40),
                    _ => rng.range(0, 3),
                }
            } else {
                match rng.range(0, 9) {
                    0..=3 => rng.range(1, 3),
                    4..=6 => rng.range(4, 12),
                    _ => rng.range(0, 60),
                }
            };
        }
        self.remaining -= 1;
        (!self.bad) as i32
    }
}

/// Events seen by [`run_sequence`] (to make sure that the random sequences cover the state
/// machine).
#[derive(Default)]
struct Coverage {
    /// `(concealState before, concealState after)` of `CConcealment_Apply()`
    transitions: [[u32; 5]; 5],
    /// `CConcealment_Apply()` returned -1 (mute release active)
    mute_release: u32,
    /// LPD restore of the concealment buffer / LPD clear in state mute
    lpd_restore: u32,
    lpd_clear: u32,
    /// `findEquiFadeFrame()` reading `fadeOutFactor[32]`
    fade_index_32: u32,
    /// `lastFadingType` after `CConcealment_TDFading()`
    fading_types: [u32; 4],
    /// comfort noise added in the vicinity of the clipping limits
    noise_near_clip: u32,
}

fn run_sequence(seed: u64, frames: usize, cov: &mut Coverage, max_fade: bool) {
    let mut rng = Rng::new(seed);
    let mut p = ParamsPair::new();
    let mut st = StPair::new();
    let mut ch = ChPair::new();
    let mut c_spec = vec![0i32; 1024];
    let mut r_spec = vec![0i32; 1024];
    unsafe { cref_core_ch_set_spec(ch.c, c_spec.as_mut_ptr()) };
    unsafe { cref_core_st_set_conceal_params(st.c, p.c) };

    let samples_per_frame = match rng.range(0, 7) {
        0 => 768,
        1 => 512,
        _ => 1024,
    };

    unsafe { cref_core_conceal_init_common_data(p.c) };
    cconcealment_init_common_data(&mut p.r);
    if rng.range(0, 3) != 0 {
        random_valid_params(&mut rng, &mut p);
    }
    let init_render_mode =
        [AACDEC_RENDER_INVALID, AACDEC_RENDER_IMDCT, AACDEC_RENDER_LPD][rng.range(0, 2) as usize];
    unsafe { cref_core_conceal_init_channel_data(st.c, p.c, init_render_mode, samples_per_frame) };
    cconcealment_init_channel_data(
        &mut st.r.concealment_info,
        &p.r,
        init_render_mode,
        samples_per_frame,
    );
    st.check("InitChannelData");

    let mut pattern = FramePattern { remaining: rng.range(0, 3), bad: false, max_fade };
    let lpd_rate = rng.range(0, 3); /* 0: never LPD */
    let sri = SamplingRateInfo::default();
    let mut c_pcm = vec![0i32; samples_per_frame as usize];
    let mut r_pcm = vec![0i32; samples_per_frame as usize];

    for f in 0..frames {
        let what = format!("seed {seed} frame {f}");
        if rng.range(0, 99) == 0 || (max_fade && f == 0) {
            random_valid_params(&mut rng, &mut p);
            if max_fade {
                let fade_in = rng.range(1, 31);
                /* comfort noise level -1 fails after the other parameters have been set */
                let c_err = unsafe { cref_core_conceal_set_params(p.c, 1, 31, fade_in, 0, -1) };
                let r_err = cconcealment_set_params(Some(&mut p.r), 1, 31, fade_in, 0, -1);
                assert_eq!((c_err, r_err), (AAC_DEC_SET_PARAM_FAIL, AAC_DEC_SET_PARAM_FAIL));
                p.check("max_fade params");
            }
        }

        let frame_ok = pattern.next(&mut rng);

        /* the channel data of the current (parsed or not) frame */
        {
            let c = &mut ch.r;
            random_ics(&mut rng, &mut c.ics_info, false);
            if max_fade && rng.range(0, 3) != 0 {
                while c.ics_info.window_sequence != BLOCK_SHORT {
                    random_ics(&mut rng, &mut c.ics_info, false);
                }
            }
            for s in &mut c.spec_scale {
                *s = if rng.range(0, 9) == 0 { rng.i16() } else { rng.range(-8, 40) as i16 };
            }
            c.render_mode = match rng.range(0, 19) {
                0 if lpd_rate > 0 => AACDEC_RENDER_LPD,
                1 if lpd_rate > 1 => AACDEC_RENDER_LPD,
                2..=3 if lpd_rate > 2 => AACDEC_RENDER_LPD,
                4 => AACDEC_RENDER_INVALID,
                5 => [AACDEC_RENDER_ELDFB, AACDEC_RENDER_INTIMDCT][rng.range(0, 1) as usize],
                _ => AACDEC_RENDER_IMDCT,
            };
            random_spectrum(&mut rng, &mut r_spec);
            c_spec.copy_from_slice(&r_spec);
            ch.sync_to_c();
        }

        let before = rng.range(0, 1);
        assert_eq!(
            unsafe { cref_core_conceal_get_last_frame_ok(st.c, before) },
            cconcealment_get_last_frame_ok(Some(&st.r.concealment_info), before),
            "{what}: GetLastFrameOk"
        );

        let state_before = st.r.concealment_info.conceal_state;
        if state_before == ConcealState_FadeOut
            && frame_ok != 0
            && st.r.concealment_info.cnt_fade_frames == CONCEAL_MAX_NUM_FADE_FACTORS as i32
            && st.r.concealment_info.cnt_valid_frames + 1 > p.r.num_mute_release_frames
            && p.r.num_fade_in_frames > 0
            && p.r.method == ConcealMethodNoise
        {
            cov.fade_index_32 += 1;
        }
        if ch.r.render_mode == AACDEC_RENDER_LPD && frame_ok == 0 {
            if state_before == ConcealState_Mute {
                cov.lpd_clear += 1;
            } else {
                cov.lpd_restore += 1;
            }
        }

        let flags = rng.next_u32();
        let last_lpd_mode = rng.range(0, 255);
        let c_ret = unsafe {
            cref_core_conceal_apply(st.c, ch.c, samples_per_frame, last_lpd_mode, frame_ok, flags)
        };
        let r_ret = cconcealment_apply(
            &mut st.r.concealment_info,
            &p.r,
            &mut ch.r,
            &mut r_spec,
            &sri,
            samples_per_frame,
            last_lpd_mode as u8,
            frame_ok,
            flags,
        );
        assert_eq!(c_ret, r_ret, "{what}: Apply return value");
        cov.transitions[state_before as usize][st.r.concealment_info.conceal_state as usize] += 1;
        cov.mute_release += (r_ret == -1) as u32;
        ch.check(&format!("{what}: Apply"));
        st.check(&format!("{what}: Apply"));
        assert_same(&format!("{what}: Apply spectrum"), &c_spec, &r_spec);
        assert_eq!(
            unsafe { cref_core_conceal_get_state(st.c) },
            cconcealment_get_state(Some(&st.r.concealment_info))
        );

        /* the time signal rendered from the spectrum */
        random_pcm(&mut rng, &mut r_pcm);
        c_pcm.copy_from_slice(&r_pcm);
        let headroom = match rng.range(0, 3) {
            0 => 0,
            1 => 3,
            _ => rng.range(0, 31),
        };
        let c_ret = unsafe {
            cref_core_conceal_td_fading(samples_per_frame, st.c, headroom, c_pcm.as_mut_ptr())
        };
        let r_ret = cconcealment_td_fading(
            samples_per_frame,
            &mut st.r.concealment_info,
            &p.r,
            headroom,
            &mut r_pcm,
        );
        assert_eq!(c_ret, r_ret, "{what}: TDFading return value");
        cov.fading_types[st.r.concealment_info.last_fading_type as usize] += 1;
        let ci = &st.r.concealment_info;
        if (ci.conceal_state != ConcealState_Ok || ci.conceal_state_old != ConcealState_Ok)
            && p.r.comfort_noise_level != 0
            && c_pcm.iter().any(|&v| v > i32::MAX - (1 << 20) || v < i32::MIN + (1 << 20))
        {
            cov.noise_near_clip += 1;
        }
        st.check(&format!("{what}: TDFading"));
        assert_same(&format!("{what}: TDFading pcm"), &c_pcm, &r_pcm);
    }
}

#[track_caller]
fn check_coverage(cov: &Coverage) {
    let t = &cov.transitions;
    let pairs = [
        (ConcealState_Ok, ConcealState_Ok),
        (ConcealState_Ok, ConcealState_Single),
        (ConcealState_Ok, ConcealState_Mute),
        (ConcealState_Single, ConcealState_Ok),
        (ConcealState_Single, ConcealState_FadeOut),
        (ConcealState_Single, ConcealState_Mute),
        (ConcealState_FadeOut, ConcealState_FadeOut),
        (ConcealState_FadeOut, ConcealState_FadeIn),
        (ConcealState_FadeOut, ConcealState_Mute),
        (ConcealState_FadeOut, ConcealState_Ok),
        (ConcealState_Mute, ConcealState_Mute),
        (ConcealState_Mute, ConcealState_FadeIn),
        (ConcealState_Mute, ConcealState_Ok),
        (ConcealState_FadeIn, ConcealState_FadeIn),
        (ConcealState_FadeIn, ConcealState_Ok),
        (ConcealState_FadeIn, ConcealState_FadeOut),
        (ConcealState_FadeIn, ConcealState_Mute),
    ];
    for (a, b) in pairs {
        assert!(t[a as usize][b as usize] > 0, "transition {a} -> {b} not covered");
    }
    assert!(cov.mute_release > 0, "mute release not covered");
    assert!(cov.lpd_restore > 0 && cov.lpd_clear > 0, "LPD restore/clear not covered");
    assert!(cov.fade_index_32 > 0, "findEquiFadeFrame(32) not covered");
    for ty in [FADE_TIMEDOMAIN_TOSPECTRALMUTE, FADE_TIMEDOMAIN_FROMSPECTRALMUTE, FADE_TIMEDOMAIN] {
        assert!(cov.fading_types[ty as usize] > 0, "fading type {ty} not covered");
    }
    assert!(cov.noise_near_clip > 0, "comfort noise clipping not covered");
}

#[test]
fn state_machine() {
    let mut cov = Coverage::default();
    for seed in 0..300 {
        run_sequence(1000 + seed, 400, &mut cov, false);
    }
    for seed in 0..50 {
        run_sequence(3000 + seed, 400, &mut cov, true);
    }
    check_coverage(&cov);
}

#[test]
fn state_machine_long() {
    let mut cov = Coverage::default();
    for seed in 0..8 {
        run_sequence(5000 + seed, 5000, &mut cov, false);
    }
    for seed in 0..2 {
        run_sequence(6000 + seed, 5000, &mut cov, true);
    }
    check_coverage(&cov);
}

#[test]
fn store() {
    let mut rng = Rng::new(6);
    let mut p = ParamsPair::new();
    let mut st = StPair::new();
    let mut ch = ChPair::new();
    let mut c_spec = vec![0i32; 1024];
    let mut r_spec = vec![0i32; 1024];
    unsafe { cref_core_ch_set_spec(ch.c, c_spec.as_mut_ptr()) };
    unsafe { cref_core_st_set_conceal_params(st.c, p.c) };
    for it in 0..5000 {
        random_params(&mut rng, &mut p.r);
        p.sync_to_c();
        random_ics(&mut rng, &mut ch.r.ics_info, true);
        if rng.range(0, 3) == 0 {
            ch.r.ics_info.window_groups = 0;
        }
        for s in &mut ch.r.spec_scale {
            *s = rng.i16();
        }
        ch.r.render_mode = rng.range(0, 4);
        random_spectrum(&mut rng, &mut r_spec);
        c_spec.copy_from_slice(&r_spec);
        ch.sync_to_c();
        unsafe { cref_core_conceal_store(st.c, ch.c) };
        cconcealment_store(&mut st.r.concealment_info, &p.r, &ch.r, &r_spec);
        st.check(&format!("Store {it}"));
        ch.check(&format!("Store {it}"));
    }
}
