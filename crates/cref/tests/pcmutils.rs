// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libPCMutils: the time domain limiter (limiter.cpp) on random
//! multichannel PCM with varying parameters, and FDK_interleave (pcm_utils.cpp).
#![cfg(not(cref_missing))]

use mpeghdec_cref::pcmutils::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::common_fix::{fl2fxconst_dbl, MAXVAL_DBL};
use mpeghdec_pcmutils::limiter::*;
use mpeghdec_pcmutils::pcm_utils::fdk_interleave;

/// A C and a Rust limiter with identical parameters.
struct LimPair {
    hc: *mut (),
    r: TDLimiter,
    max_buf_len: usize,
    delay_buf_len: usize,
    /// Number of apply() calls in simplified mode, default mode and with gain reduction.
    stats: [u32; 3],
}

impl LimPair {
    fn new(
        max_attack_ms: u32,
        release_ms: u32,
        threshold: i32,
        max_channels: u32,
        max_sample_rate: u32,
    ) -> Option<Self> {
        let hc = unsafe {
            cref_lim_create(max_attack_ms, release_ms, threshold, max_channels, max_sample_rate)
        };
        let r =
            TDLimiter::create(max_attack_ms, release_ms, threshold, max_channels, max_sample_rate);
        assert_eq!(hc.is_null(), r.is_none(), "create {max_attack_ms} {max_sample_rate}");
        let r = r?;
        let attack = (max_attack_ms * max_sample_rate / 1000) as usize;
        Some(LimPair {
            hc,
            r,
            max_buf_len: attack + 1,
            delay_buf_len: attack * max_channels as usize,
            stats: [0; 3],
        })
    }

    fn check(&self, ctx: &str) {
        let mut st = [0i32; 21];
        let mut max_buf = vec![0i32; self.max_buf_len];
        let mut delay_buf = vec![0i32; self.delay_buf_len];
        unsafe {
            cref_lim_state(
                self.hc,
                st.as_mut_ptr(),
                max_buf.as_mut_ptr(),
                max_buf.len() as u32,
                delay_buf.as_mut_ptr(),
                delay_buf.len() as u32,
            )
        };
        let r = &self.r;
        let rs = [
            r.attack as i32,
            r.attack_const,
            r.release_const,
            r.attack_ms as i32,
            r.release_ms as i32,
            r.max_attack_ms as i32,
            r.threshold,
            r.channels as i32,
            r.max_channels as i32,
            r.sample_rate as i32,
            r.max_sample_rate as i32,
            r.cor,
            r.max,
            r.max_buf_idx as i32,
            r.delay_buf_idx as i32,
            r.smooth_state0,
            r.min_gain,
            r.clean_samples as i32,
            r.release as i32,
            r.previous_mode as i32,
            r.scaling,
        ];
        assert_eq!(rs, st, "state {ctx}");
        assert_eq!(r.max_buf, max_buf, "maxBuf {ctx}");
        assert_eq!(r.delay_buf, delay_buf, "delayBuf {ctx}");
        assert_eq!(
            r.get_max_gain_reduction(),
            unsafe { cref_lim_getMaxGainReduction(self.hc) },
            "gain reduction {ctx}"
        );
        assert_eq!(r.get_delay(), unsafe { cref_lim_getDelay(self.hc) }, "delay {ctx}");
    }

    fn apply(&mut self, input: &[i32], gains: Option<&[i32]>, scaling: i32, n: u32, ctx: &str) {
        let len = n as usize * self.r.channels as usize;
        assert_eq!(input.len(), len);
        let mut in_c = input.to_vec();
        let mut in_r = input.to_vec();
        let mut out_c = vec![0x1234_5678i32; len];
        let mut out_r = out_c.clone();
        let mut work_c = vec![0x0765_4321i32; n as usize];
        let mut work_r = work_c.clone();
        let mut gains_c = gains.map(|g| g.to_vec());
        let ec = unsafe {
            cref_lim_apply(
                self.hc,
                in_c.as_mut_ptr(),
                out_c.as_mut_ptr(),
                work_c.as_mut_ptr(),
                gains_c.as_mut().map_or(std::ptr::null_mut(), |g| g.as_mut_ptr()),
                scaling,
                n,
            )
        };
        let er = self.r.apply(&mut in_r, &mut out_r, &mut work_r, gains, scaling, n);
        assert_eq!(er, ec, "apply result {ctx}");
        assert_eq!(in_r, in_c, "samplesIn {ctx}");
        assert_eq!(out_r, out_c, "samplesOut {ctx}");
        assert_eq!(work_r, work_c, "workBuf {ctx}");
        self.check(ctx);
        self.stats[self.r.previous_mode as usize] += 1;
        if self.r.previous_mode == 1 && self.r.get_max_gain_reduction() > 0 {
            self.stats[2] += 1;
        }
    }
}

impl Drop for LimPair {
    fn drop(&mut self) {
        unsafe { cref_lim_destroy(self.hc) };
    }
}

/// Random interleaved PCM of `n` samples and `channels` channels. The amplitude class changes
/// per call so that the limiter switches between the default and the simplified mode.
fn random_pcm(g: &mut Rng, n: u32, channels: u32, threshold: i32, scaling: i32) -> Vec<i32> {
    let thr = (threshold >> scaling).max(1) as i64;
    let peak: i64 = match g.range(0, 7) {
        0 => 0,
        1 => 1 << g.range(0, 16),
        2 => thr / 2,
        3 => thr,
        4 => thr * 2,
        5 => thr * g.range(2, 16) as i64,
        _ => i32::MAX as i64,
    };
    let peak = peak.min(i32::MAX as i64);
    (0..n * channels)
        .map(|_| {
            if peak == 0 {
                0
            } else if g.range(0, 50) == 0 {
                [i32::MIN, i32::MAX, -(peak as i32), peak as i32][g.range(0, 3) as usize]
            } else {
                ((g.next_u64() % (2 * peak as u64 + 1)) as i64 - peak) as i32
            }
        })
        .collect()
}

/// The limiter as used by the MPEG-H decoder (aacdecoder_lib.cpp).
#[test]
fn decoder_usage() {
    let layouts = [1u32, 2, 6, 8, 10, 12, 14, 24];
    let rates = [48000u32, 44100, 32000, 24000, 16000];
    let mut total = [0u32; 3];
    for seed in 0..40u64 {
        let mut g = Rng::new(seed);
        // TDL_MPEGH3DA_DEFAULT_ATTACK, TDL_RELEASE_DEFAULT_MS, MAXVAL_DBL, 24 channels, 48 kHz
        let mut l = LimPair::new(5, TDL_RELEASE_DEFAULT_MS, MAXVAL_DBL, 24, 48000).unwrap();
        l.check("create");
        let thr = fl2fxconst_dbl(0.89125094f32 as f64);
        assert_eq!(l.r.set_attack(5), unsafe { cref_lim_setAttack(l.hc, 5) });
        assert_eq!(l.r.set_threshold(thr), unsafe { cref_lim_setThreshold(l.hc, thr) });
        l.check("init");
        let mut channels = layouts[g.range(0, layouts.len() as i32 - 1) as usize];
        let mut rate = rates[g.range(0, rates.len() as i32 - 1) as usize];
        let frame_size = if g.range(0, 1) == 0 { 1024 } else { 768 };
        let scaling = g.range(6, 9);
        let mut amp_frames = 0;
        let mut class = 0;
        for frame in 0..120 {
            if g.range(0, 40) == 0 {
                channels = layouts[g.range(0, layouts.len() as i32 - 1) as usize];
            }
            if g.range(0, 60) == 0 {
                rate = rates[g.range(0, rates.len() as i32 - 1) as usize];
            }
            if g.range(0, 80) == 0 {
                // AACDEC_CLRHIST
                assert_eq!(l.r.reset(), unsafe { cref_lim_reset(l.hc) });
            }
            let ctx = format!("seed {seed} frame {frame} ch {channels} fs {rate}");
            assert_eq!(l.r.set_n_channels(channels), unsafe {
                cref_lim_setNChannels(l.hc, channels)
            });
            assert_eq!(l.r.set_sample_rate(rate), unsafe { cref_lim_setSampleRate(l.hc, rate) });
            // Keep the amplitude class for a while (quiet periods longer than the release time
            // switch to the simplified mode).
            if amp_frames == 0 {
                amp_frames = g.range(1, 30);
                class = g.range(0, 6);
            }
            amp_frames -= 1;
            let n =
                if g.range(0, 10) == 0 { g.range(0, frame_size) as u32 } else { frame_size as u32 };
            let input = random_pcm_class(&mut g, class, n, channels, thr, scaling);
            l.apply(&input, None, scaling, n, &ctx);
        }
        (0..3).for_each(|k| total[k] += l.stats[k]);
    }
    // Simplified mode, default mode and actual limiting must all be covered.
    assert!(total.iter().all(|&t| t > 200), "stats {total:?}");
}

fn random_pcm_class(g: &mut Rng, class: i32, n: u32, channels: u32, thr: i32, sc: i32) -> Vec<i32> {
    // Amplitude class: silence, quiet, below, at and above the threshold, full scale.
    let mut v = Vec::with_capacity((n * channels) as usize);
    let t = (thr >> sc).max(1) as i64;
    let peak: i64 = match class {
        0 => 0,
        1 => 1 << 12,
        2 => t / 2,
        3 => t,
        4 => t * 2,
        5 => t * 8,
        _ => i32::MAX as i64,
    };
    for _ in 0..n * channels {
        v.push(if peak == 0 {
            0
        } else {
            ((g.next_u64() % (2 * peak as u64 + 1)) as i64 - peak) as i32
        });
    }
    v
}

/// Random parameters and random operation sequences, with and without per sample gains.
#[test]
fn random_operations() {
    let mut total = [0u32; 3];
    for seed in 100..400u64 {
        let mut g = Rng::new(seed);
        let max_attack_ms = g.range(0, 20) as u32;
        let release_ms = g.range(0, 300) as u32;
        let threshold = match g.range(0, 3) {
            0 => MAXVAL_DBL,
            1 => fl2fxconst_dbl(0.89125094f32 as f64),
            2 => 0,
            _ => g.range(1, MAXVAL_DBL),
        };
        let max_channels = g.range(1, 24) as u32;
        let max_rate = [8000u32, 16000, 22050, 32000, 44100, 48000, 96000][g.range(0, 6) as usize];
        let Some(mut l) =
            LimPair::new(max_attack_ms, release_ms, threshold, max_channels, max_rate)
        else {
            continue;
        };
        l.check("create");
        let mut scaling = g.range(0, 12);
        for step in 0..150 {
            let ctx = format!("seed {seed} step {step}");
            let op = g.range(0, 20);
            let (a, c) = unsafe {
                match op {
                    0 => (l.r.reset(), cref_lim_reset(l.hc)),
                    1 => {
                        let n = g.range(0, max_channels as i32 + 2) as u32;
                        (l.r.set_n_channels(n), cref_lim_setNChannels(l.hc, n))
                    }
                    2 => {
                        let fs = match g.range(0, 3) {
                            0 => max_rate,
                            1 => g.range(0, max_rate as i32 + 1000) as u32,
                            _ => [8000u32, 16000, 44100, 48000][g.range(0, 3) as usize],
                        };
                        (l.r.set_sample_rate(fs), cref_lim_setSampleRate(l.hc, fs))
                    }
                    3 => {
                        let ms = g.range(0, max_attack_ms as i32 + 2) as u32;
                        (l.r.set_attack(ms), cref_lim_setAttack(l.hc, ms))
                    }
                    4 => {
                        let ms = g.range(0, 400) as u32;
                        (l.r.set_release(ms), cref_lim_setRelease(l.hc, ms))
                    }
                    5 => {
                        let t = g.range(0, MAXVAL_DBL);
                        (l.r.set_threshold(t), cref_lim_setThreshold(l.hc, t))
                    }
                    6 => {
                        scaling = g.range(0, 12);
                        (0, 0)
                    }
                    _ => {
                        let n = match g.range(0, 3) {
                            0 => g.range(0, 64) as u32,
                            1 => g.range(0, 2048) as u32,
                            _ => 1024,
                        };
                        let input = random_pcm(&mut g, n, l.r.channels, l.r.threshold, scaling);
                        let gains: Option<Vec<i32>> = if g.range(0, 2) == 0 {
                            Some(
                                (0..n)
                                    .map(|_| match g.range(0, 3) {
                                        0 => MAXVAL_DBL,
                                        1 => g.range(0, MAXVAL_DBL),
                                        _ => g.range(0x2000_0000, MAXVAL_DBL),
                                    })
                                    .collect(),
                            )
                        } else {
                            None
                        };
                        l.apply(&input, gains.as_deref(), scaling, n, &ctx);
                        (0, 0)
                    }
                }
            };
            assert_eq!(a, c, "op {op} {ctx}");
            l.check(&format!("op {op} {ctx}"));
        }
        (0..3).for_each(|k| total[k] += l.stats[k]);
    }
    assert!(total.iter().all(|&t| t > 500), "stats {total:?}");
}

#[test]
fn interleave() {
    let mut g = Rng::new(5);
    for _ in 0..500 {
        let channels = g.range(0, 24) as u32;
        let length = g.range(0, 300) as u32;
        let frame_size = length + g.range(0, 20) as u32;
        let input: Vec<i32> = (0..channels * frame_size).map(|_| g.i32()).collect();
        let mut out_c = vec![0x55i32; (channels * length) as usize + 4];
        let mut out_r = out_c.clone();
        unsafe {
            cref_FDK_interleave(input.as_ptr(), out_c.as_mut_ptr(), channels, frame_size, length)
        };
        fdk_interleave(&input, &mut out_r, channels, frame_size, length);
        assert_eq!(out_r, out_c);
    }
}
