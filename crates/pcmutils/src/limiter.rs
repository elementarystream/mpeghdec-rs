// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libPCMutils/include/limiter.h, libPCMutils/src/limiter.cpp and
// libPCMutils/src/version.h. libPCMutils/src/arm/limiter_arm.cpp only contains NEON code
// (applyLimiter_func1), which is not compiled in the reference configuration.

//! Hard limiter for clipping prevention.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_div_norm_noexp, f_log2, f_pow, fixp_round_to_int, inv_fixp};
use mpeghdec_fdk::scale::{
    saturate_left_shift, scale_value, scale_value_saturate, scale_values_saturate,
};

/* library info (version.h) */
pub const PCMUTIL_LIB_VL0: i32 = 4;
pub const PCMUTIL_LIB_VL1: i32 = 0;
pub const PCMUTIL_LIB_VL2: i32 = 0;
pub const PCMUTIL_LIB_TITLE: &str = "PCM Utility Lib";
/* library title */
pub const TDLIMIT_LIB_TITLE: &str = "TD Limiter Lib";

/// default attack time in ms
pub const TDL_ATTACK_DEFAULT_MS: u32 = 15;
/// default release time in ms
pub const TDL_RELEASE_DEFAULT_MS: u32 = 50;
/// factor on release time to let the gain fall back to 1.0 in float precision
pub const TDLIMIT_FALLBACK_FACTOR: u32 = 8;

/// `TDLIMITER_ERROR`
pub type TdLimiterError = i32;
pub const TDLIMIT_OK: TdLimiterError = 0;
pub const TDLIMIT_UNKNOWN: TdLimiterError = -1;
pub const __error_codes_start: TdLimiterError = -100;
pub const TDLIMIT_INVALID_HANDLE: TdLimiterError = -99;
pub const TDLIMIT_INVALID_PARAMETER: TdLimiterError = -98;
pub const __error_codes_end: TdLimiterError = -97;

/// `PCM_LIM`
pub type PcmLim = i32;
pub const PCM_LIM_BITS: i32 = 32;
pub const SAMPLE_BITS_LIM: i32 = DFRACT_BITS;

/// `FL2FXCONST_DBL(1.0f / (1 << 1))`
const HALF: FixpDbl = fl2fxconst_dbl(1.0 / (1 << 1) as f64);
/// `FL2FXCONST_SGL(0.1f * (1 << 1))`
const C_0_1_X2: FixpSgl = fl2fxconst_sgl((0.1f32 * (1 << 1) as f32) as f64);
/// `FL2FXCONST_SGL(1.11111111f / (1 << 1))`
const C_1_11_DIV2: FixpSgl = fl2fxconst_sgl((1.11111111f32 / (1 << 1) as f32) as f64);
/// `FL2FXCONST_DBL(-6.0206f / (1 << 3))`
const C_M6_0206_DIV8: FixpDbl = fl2fxconst_dbl((-6.0206f32 / (1 << 3) as f32) as f64);

/// `pow(0.1, 1.0 / (n + 1))` as computed by the limiter: `fPow(0.1, 0, invFixp(n + 1), 0)`.
fn pow_0_1(n: u32) -> FixpDbl {
    let mut e_ans = 0;
    let exponent = inv_fixp(n.wrapping_add(1) as FixpDbl);
    let c = f_pow(fl2fxconst_dbl(0.1f32 as f64), 0, exponent, 0, &mut e_ans);
    scale_value(c, e_ans)
}

/// `struct TDLimiter` (`TDLimiterPtr` handle). `pcmLimiter_Destroy()` is `drop`.
///
/// State digest: the delay line (`attack` samples of `channels` channels) and the running maximum
/// buffer (`attack + 1` values) are ring buffers; they are fed in the order of their read/write
/// positions (starting at `delayBufIdx` / `maxBufIdx`), without the positions. Every access is
/// relative to these positions (the maximum search over `maxBuf` is order independent and
/// `pcmLimiter_SetNChannels()` moves every sample within its slot), and the memory beyond the used
/// part of the buffers is never read.
#[derive(Clone, Debug)]
pub struct TDLimiter {
    pub attack: u32,
    pub attack_const: FixpDbl,
    pub release_const: FixpDbl,
    pub attack_ms: u32,
    pub release_ms: u32,
    pub max_attack_ms: u32,
    pub threshold: FixpDbl,
    pub channels: u32,
    pub max_channels: u32,
    pub sample_rate: u32,
    pub max_sample_rate: u32,
    pub cor: FixpDbl,
    pub max: FixpDbl,
    /// `maxBuf[attack + 1]` (allocated for the initial attack time)
    pub max_buf: Vec<FixpDbl>,
    /// `delayBuf[attack * maxChannels]` (allocated for the initial attack time)
    pub delay_buf: Vec<FixpDbl>,
    pub max_buf_idx: u32,
    pub delay_buf_idx: u32,
    pub smooth_state0: FixpDbl,
    pub min_gain: FixpDbl,
    pub clean_samples: u32,
    pub release: u32,
    pub previous_mode: u8,
    pub scaling: i32,
}

impl StateDigest for TDLimiter {
    fn digest(&self, d: &mut mpeghdec_fdk::digest::Digester) {
        let TDLimiter {
            attack,
            attack_const,
            release_const,
            attack_ms,
            release_ms,
            max_attack_ms,
            threshold,
            channels,
            max_channels,
            sample_rate,
            max_sample_rate,
            cor,
            max,
            max_buf,
            delay_buf,
            max_buf_idx,
            delay_buf_idx,
            smooth_state0,
            min_gain,
            clean_samples,
            release,
            previous_mode,
            scaling,
        } = self;
        d.field("attack", attack);
        d.field("attack_const", attack_const);
        d.field("release_const", release_const);
        d.field("attack_ms", attack_ms);
        d.field("release_ms", release_ms);
        d.field("max_attack_ms", max_attack_ms);
        d.field("threshold", threshold);
        d.field("channels", channels);
        d.field("max_channels", max_channels);
        d.field("sample_rate", sample_rate);
        d.field("max_sample_rate", max_sample_rate);
        d.field("cor", cor);
        d.field("max", max);
        let a = *attack as usize;
        let ch = *channels as usize;
        d.field_with("max_buf", |d| {
            let n = (a + 1).min(max_buf.len());
            let i = (*max_buf_idx as usize).min(n);
            FixpDbl::digest_slice(&max_buf[i..n], d);
            FixpDbl::digest_slice(&max_buf[..i], d);
        });
        d.field_with("delay_buf", |d| {
            let n = (a * ch).min(delay_buf.len());
            let i = (*delay_buf_idx as usize * ch).min(n);
            FixpDbl::digest_slice(&delay_buf[i..n], d);
            FixpDbl::digest_slice(&delay_buf[..i], d);
        });
        d.field("smooth_state0", smooth_state0);
        d.field("min_gain", min_gain);
        d.field("clean_samples", clean_samples);
        d.field("release", release);
        d.field("previous_mode", previous_mode);
        d.field("scaling", scaling);
    }
}

mpeghdec_fdk::avx2_clone! {
/// The maximum of the absolute values of the `channels` interleaved channels of every sample.
fn local_maxima(loc_max_array: &mut [PcmLim], samples_in: &[PcmLim], channels: usize) {
    if channels == 0 {
        loc_max_array.fill(0);
        return;
    }
    for (m, x) in loc_max_array.iter_mut().zip(samples_in.chunks_exact(channels)) {
        *m = x.iter().fold(0, |a, &v| f_max(a, f_abs(v)));
    }
}
}

impl TDLimiter {
    /// `pcmLimiter_Create(maxAttackMs, releaseMs, threshold, maxChannels, maxSampleRate)`
    ///
    /// * `max_attack_ms` – maximum and initial attack/lookahead time in milliseconds
    /// * `release_ms` – release time in milliseconds (90% time constant)
    /// * `threshold` – limiting threshold
    /// * `max_channels` – maximum and initial number of channels
    /// * `max_sample_rate` – maximum and initial sampling rate in Hz
    ///
    /// Returns `None` (C: `NULL`) if the attack time is 0 samples.
    pub fn create(
        max_attack_ms: u32,
        release_ms: u32,
        threshold: FixpDbl,
        max_channels: u32,
        max_sample_rate: u32,
    ) -> Option<TDLimiter> {
        /* calc attack and release time in samples */
        let attack = max_attack_ms.wrapping_mul(max_sample_rate) / 1000;
        if attack == 0 {
            return None;
        }

        let release = release_ms.wrapping_mul(max_sample_rate) / 1000;

        /* attackConst = pow(0.1, 1.0 / (attack + 1)) */
        let attack_const = pow_0_1(attack);

        /* releaseConst  = (float)pow(0.1, 1.0 / (release + 1)) */
        let release_const = pow_0_1(release);

        let mut limiter = TDLimiter {
            attack,
            attack_const,
            release_const,
            attack_ms: max_attack_ms,
            release_ms,
            max_attack_ms,
            threshold,
            channels: max_channels,
            max_channels,
            sample_rate: max_sample_rate,
            max_sample_rate,
            cor: 0,
            max: 0,
            max_buf: vec![0; attack as usize + 1],
            delay_buf: vec![0; attack as usize * max_channels as usize],
            max_buf_idx: 0,
            delay_buf_idx: 0,
            smooth_state0: 0,
            min_gain: 0,
            clean_samples: (TDLIMIT_FALLBACK_FACTOR + 1).wrapping_mul(release),
            release,
            previous_mode: 0,
            scaling: 0,
        };

        limiter.reset();

        Some(limiter)
    }

    /// `pcmLimiter_Apply(limiter, samplesIn, samplesOut, workBuf, pGainPerSample, scaling,
    /// nSamples)`
    ///
    /// * `samples_in` – interleaved input samples (`n_samples * channels`); modified in place if
    ///   `p_gain_per_sample` is given
    /// * `samples_out` – interleaved output samples (`n_samples * channels`)
    /// * `work_buf` – work buffer of `n_samples` entries
    /// * `p_gain_per_sample` – gains for each sample, or `None` (C: `NULL`) if not needed
    /// * `scaling` – scaling of output samples
    pub fn apply(
        &mut self,
        samples_in: &mut [PcmLim],
        samples_out: &mut [IntPcm],
        work_buf: &mut [PcmLim],
        p_gain_per_sample: Option<&[FixpDbl]>,
        scaling: i32,
        n_samples: u32,
    ) -> TdLimiterError {
        let channels = self.channels as usize;
        let attack = self.attack;
        let mut delay_buf_idx = self.delay_buf_idx;
        let threshold = self.threshold.wrapping_shr(scaling as u32);
        let mut max = self.max;
        let n = n_samples as usize;

        if self.scaling != scaling {
            scale_values_saturate(
                &mut self.delay_buf[..attack as usize * channels],
                self.scaling - scaling,
            );
            scale_values_saturate(&mut self.max_buf[..attack as usize + 1], self.scaling - scaling);
            max = scale_value_saturate(max, self.scaling - scaling);
            self.scaling = scaling;
        }

        /* Preparatory step. Find local maxima and store them in an array. Find the global maximum. */
        let mut global_max: PcmLim = 0;
        let loc_max_array = work_buf;
        let mut p_in = 0usize;

        if let Some(p_gain_per_sample) = p_gain_per_sample {
            /* apply additionalGain on input signal while searching for the maximum */
            for i in 0..n {
                let mut loc_max: PcmLim = 0;
                let additional_gain = p_gain_per_sample[i];
                for _ in 0..channels {
                    let tmp_in: FixpDbl = samples_in[p_in];
                    samples_in[p_in] = f_mult(tmp_in, additional_gain);
                    loc_max = f_max(loc_max, f_abs(samples_in[p_in]));
                    p_in += 1;
                }
                loc_max_array[i] = loc_max;
                global_max = f_max(global_max, loc_max);
            }
        } else {
            /* the maximum of the absolute values of the channels (order independent) */
            let loc_max_array = &mut loc_max_array[..n];
            match channels {
                1 => {
                    for (m, &x) in loc_max_array.iter_mut().zip(&samples_in[..n]) {
                        *m = f_max(0, f_abs(x));
                    }
                }
                2 => {
                    for (m, x) in loc_max_array.iter_mut().zip(samples_in[..2 * n].chunks_exact(2))
                    {
                        *m = f_max(f_max(0, f_abs(x[0])), f_abs(x[1]));
                    }
                }
                _ => local_maxima(loc_max_array, &samples_in[..n * channels], channels),
            }
            global_max = loc_max_array.iter().fold(global_max, |g, &m| f_max(g, m));
        }

        /* Check and control which mode (default/simplified) must be started */
        if global_max > threshold {
            self.clean_samples = 0;
        } else {
            self.clean_samples = self.clean_samples.wrapping_add(n_samples);
        }

        /* Simplified mode.The limiter is not active. Just handle Circular Buffer Input/Output. */
        if self.clean_samples > TDLIMIT_FALLBACK_FACTOR.wrapping_mul(self.release) {
            let delay_buf = &mut self.delay_buf;

            /* Copy delayed signal from delay line to output */
            let mut delay_head_length = attack - delay_buf_idx; /* delayHead: part of buffer after delayBufIdx */
            let mut delay_tail_length = delay_buf_idx; /* delayTail: part of buffer from beginning to delayBufIdx */

            let mut delay_head_copy_length = f_min(delay_head_length, n_samples);
            let mut delay_tail_copy_length =
                f_min(delay_tail_length, n_samples - delay_head_copy_length);

            let sat = |d: FixpDbl| saturate_left_shift(d, scaling, DFRACT_BITS);
            let head = delay_head_copy_length as usize * channels;
            let tail = delay_tail_copy_length as usize * channels;
            let p2_data = &delay_buf[delay_buf_idx as usize * channels..];
            for (o, &d) in samples_out[..head].iter_mut().zip(&p2_data[..head]) {
                *o = sat(d);
            }

            for (o, &d) in samples_out[head..head + tail].iter_mut().zip(&delay_buf[..tail]) {
                *o = sat(d);
            }

            /* Copy delayed signal that was not contained in delay line directly from input to output */
            let delay_direct_length = n_samples as i32 - attack as i32;
            let delay_direct_copy_length = f_max(delay_direct_length, 0) as u32;

            let p2_output = attack as usize * channels;
            let direct = delay_direct_copy_length as usize * channels;
            if direct > 0 {
                for (o, &x) in
                    samples_out[p2_output..p2_output + direct].iter_mut().zip(&samples_in[..direct])
                {
                    *o = sat(x);
                }
            }

            /* Increment delayBufIdx to the position where delay line is to be filled */
            delay_buf_idx = (delay_buf_idx + delay_direct_copy_length) % attack;

            /* Fill delay line from input */
            delay_head_length = attack - delay_buf_idx;
            delay_tail_length = delay_buf_idx;

            delay_head_copy_length = f_min(delay_head_length, n_samples);
            delay_tail_copy_length = f_min(delay_tail_length, n_samples - delay_head_copy_length);

            let dst = delay_buf_idx as usize * channels;
            let src = delay_direct_copy_length as usize * channels;
            let len = delay_head_copy_length as usize * channels;
            delay_buf[dst..dst + len].copy_from_slice(&samples_in[src..src + len]);

            let src = (delay_direct_copy_length + delay_head_copy_length) as usize * channels;
            let len = delay_tail_copy_length as usize * channels;
            delay_buf[..len].copy_from_slice(&samples_in[src..src + len]);

            /* Increment delayBufIdx after filling delay line */
            delay_buf_idx =
                (delay_buf_idx + delay_head_copy_length + delay_tail_copy_length) % attack;

            /* If we come from the default mode, then reset state */
            if self.previous_mode == 1 {
                self.max_buf_idx = 0;
                self.max = 0;
                self.cor = HALF;
                self.smooth_state0 = HALF;
                self.min_gain = HALF;
                self.previous_mode = 0; /* Set to simplified mode */
                self.max_buf[..self.attack as usize + 1].fill(0);
            }

            /* Store the circular buffer pointer */
            self.delay_buf_idx = delay_buf_idx;

            /* Limit cleanSamples growth */
            self.clean_samples = (TDLIMIT_FALLBACK_FACTOR + 1).wrapping_mul(self.release);
        } else {
            let mut max_buf_idx = self.max_buf_idx as usize;
            let mut gain: FixpDbl;
            let mut min_gain: FixpDbl = HALF;
            let attack_const = self.attack_const;
            let release_const = self.release_const;
            let mut cor = self.cor;
            let mut smooth_state0 = self.smooth_state0;
            let max_buf = &mut self.max_buf;
            let delay_buf = &mut self.delay_buf;
            let mut in_ofs = 0usize;
            let mut out_ofs = 0usize;

            for i in 0..n {
                let mut tmp: FixpDbl = loc_max_array[i];

                /* set threshold as lower border to save calculations in running maximum algorithm */
                tmp = f_max(tmp, threshold);

                /* running maximum */
                let old = max_buf[max_buf_idx];
                max_buf[max_buf_idx] = tmp;

                if tmp >= max {
                    /* new sample is greater than old maximum, so it is the new maximum */
                    max = tmp;
                } else if old < max {
                    /* maximum does not change, as the sample, which has left the window was
                    not the maximum */
                } else {
                    /* the old maximum has left the window, we have to search the complete
                    buffer for the new max */
                    max = max_buf[0];
                    for &m in &max_buf[1..=attack as usize] {
                        max = f_max(max, m);
                    }
                }
                max_buf_idx += 1;
                if max_buf_idx > attack as usize {
                    max_buf_idx = 0;
                }

                /* calc gain */
                /* gain is downscaled by one, so that gain = 1.0 can be represented */
                if max > threshold {
                    gain = f_div_norm_noexp(threshold, max) >> 1;
                } else {
                    gain = HALF;
                }

                /* gain smoothing, method: TDL_EXPONENTIAL */
                /* first order IIR filter with attack correction to avoid overshoots */

                /* correct the 'aiming' value of the exponential attack to avoid the remaining overshoot */
                if gain < smooth_state0 {
                    cor = f_min(
                        cor,
                        f_mult_div2(gain - f_mult_div2(C_0_1_X2, smooth_state0), C_1_11_DIV2) << 2,
                    );
                } else {
                    cor = gain;
                }

                /* smoothing filter */
                if cor < smooth_state0 {
                    smooth_state0 = f_mult(attack_const, smooth_state0 - cor) + cor; /* attack */
                    smooth_state0 = f_max(smooth_state0, gain); /* avoid overshooting target */
                } else {
                    /* sign inversion twice to round towards +infinity,
                    so that gain can converge to 1.0 again,
                    for bit-identical output when limiter is not active */
                    smooth_state0 = -f_mult(release_const, -(smooth_state0 - cor)) + cor;
                    /* release */
                }

                gain = smooth_state0;

                let d0 = delay_buf_idx as usize * channels;
                let p_delay_buf = &mut delay_buf[d0..d0 + channels];
                let out = &mut samples_out[out_ofs..out_ofs + channels];
                let inp = &samples_in[in_ofs..in_ofs + channels];
                for ((d, o), &x) in p_delay_buf.iter_mut().zip(out.iter_mut()).zip(inp) {
                    /* Apply gain to delayed signal */
                    let tmp = f_mult(*d, gain);
                    *o = saturate_left_shift(tmp, scaling + 1, DFRACT_BITS);
                    /* feed delay line */
                    *d = x;
                }

                delay_buf_idx += 1;
                if delay_buf_idx >= attack {
                    delay_buf_idx = 0;
                }

                /* save minimum gain factor */
                if gain < min_gain {
                    min_gain = gain;
                }

                /* advance sample pointer by <channel> samples */
                in_ofs += channels;
                out_ofs += channels;
            }

            self.max = max;
            self.max_buf_idx = max_buf_idx as u32;
            self.cor = cor;
            self.delay_buf_idx = delay_buf_idx;
            self.smooth_state0 = smooth_state0;
            self.min_gain = min_gain;
            self.previous_mode = 1; /*Set to default mode*/
        }

        TDLIMIT_OK
    }

    /// `pcmLimiter_SetThreshold(limiter, threshold)`
    pub fn set_threshold(&mut self, threshold: FixpDbl) -> TdLimiterError {
        self.threshold = threshold;

        TDLIMIT_OK
    }

    /// `pcmLimiter_Reset(limiter)`
    pub fn reset(&mut self) -> TdLimiterError {
        self.max_buf_idx = 0;
        self.delay_buf_idx = 0;
        self.max = 0;
        self.cor = HALF;
        self.smooth_state0 = HALF;
        self.min_gain = HALF;
        self.scaling = 0;

        self.max_buf[..self.attack as usize + 1].fill(0);
        self.delay_buf[..self.attack as usize * self.channels as usize].fill(0);

        TDLIMIT_OK
    }

    /// `pcmLimiter_GetDelay(limiter)`: exact delay caused by the limiter in samples per channel.
    pub fn get_delay(&self) -> u32 {
        self.attack
    }

    /// `pcmLimiter_GetMaxGainReduction(limiter)`: maximum gain reduction in last processed block
    /// in dB.
    pub fn get_max_gain_reduction(&self) -> i32 {
        /* maximum gain reduction in dB = -20 * log10(limiter->minGain)
        = -20 * log2(limiter->minGain)/log2(10) = -6.0206*log2(limiter->minGain) */
        let mut e_ans = 0;

        let loggain = f_log2(self.min_gain, 1, &mut e_ans);

        let max_gain_reduction = f_mult(loggain, C_M6_0206_DIV8);

        fixp_round_to_int(max_gain_reduction, e_ans + 3)
    }

    /// `pcmLimiter_SetNChannels(limiter, nChannels)`: number of channels (<= maxChannels specified
    /// on create).
    pub fn set_n_channels(&mut self, n_channels: u32) -> TdLimiterError {
        if self.channels == n_channels {
            return TDLIMIT_OK;
        }

        if n_channels > self.max_channels {
            return TDLIMIT_INVALID_PARAMETER;
        }

        let attack = self.attack as usize;
        let channels = self.channels as usize;
        let n_ch = n_channels as usize;
        let delay_buf = &mut self.delay_buf;
        if channels > n_ch {
            for i in 1..attack {
                for c in 0..n_ch {
                    delay_buf[i * n_ch + c] = delay_buf[i * channels + c];
                }
            }
        } else if channels < n_ch {
            for i in (0..attack).rev() {
                for c in (0..channels).rev() {
                    delay_buf[i * n_ch + c] = delay_buf[i * channels + c];
                }
                for c in (channels..n_ch).rev() {
                    delay_buf[i * n_ch + c] = 0;
                }
            }
        }
        self.channels = n_channels;

        TDLIMIT_OK
    }

    /// Whether [`TDLimiter::set_sample_rate`] accepts `sample_rate` (not in C).
    pub fn sample_rate_supported(&self, sample_rate: u32) -> bool {
        sample_rate <= self.max_sample_rate && self.attack_ms.wrapping_mul(sample_rate) / 1000 != 0
    }

    /// `pcmLimiter_SetSampleRate(limiter, sampleRate)`: sampling rate in Hz (<= maxSampleRate
    /// specified on create).
    pub fn set_sample_rate(&mut self, sample_rate: u32) -> TdLimiterError {
        if sample_rate > self.max_sample_rate {
            return TDLIMIT_INVALID_PARAMETER;
        }

        /* update attack and release time in samples */
        let attack = self.attack_ms.wrapping_mul(sample_rate) / 1000;
        if attack == 0 {
            return TDLIMIT_INVALID_PARAMETER;
        }

        let release = self.release_ms.wrapping_mul(sample_rate) / 1000;

        /* attackConst = pow(0.1, 1.0 / (attack + 1)) */
        let attack_const = pow_0_1(attack);

        /* releaseConst  = (float)pow(0.1, 1.0 / (release + 1)) */
        let release_const = pow_0_1(release);

        self.attack = attack;
        self.attack_const = attack_const;
        self.release_const = release_const;

        /* In case that the sampling rate changes, seamless switching is not supported yet */
        if self.sample_rate != sample_rate {
            self.reset();
        }

        self.sample_rate = sample_rate;

        TDLIMIT_OK
    }

    /// `pcmLimiter_SetAttack(limiter, attackMs)`: attack time in ms (<= maxAttackMs specified on
    /// create).
    pub fn set_attack(&mut self, attack_ms: u32) -> TdLimiterError {
        if attack_ms > self.max_attack_ms {
            return TDLIMIT_INVALID_PARAMETER;
        }

        /* calculate attack time in samples */
        let attack = attack_ms.wrapping_mul(self.sample_rate) / 1000;
        if attack == 0 {
            return TDLIMIT_INVALID_PARAMETER;
        }

        /* attackConst = pow(0.1, 1.0 / (attack + 1)) */
        let attack_const = pow_0_1(attack);

        /* Reset in case that the attack time changes. Seamless switching is not supported yet */
        if self.attack != attack {
            self.reset();
        }

        self.attack = attack;
        self.attack_const = attack_const;
        self.attack_ms = attack_ms;

        TDLIMIT_OK
    }

    /// `pcmLimiter_SetRelease(limiter, releaseMs)`: release time in ms.
    pub fn set_release(&mut self, release_ms: u32) -> TdLimiterError {
        /* calculate  release time in samples */
        let release = release_ms.wrapping_mul(self.sample_rate) / 1000;

        /* releaseConst  = (float)pow(0.1, 1.0 / (release + 1)) */
        let release_const = pow_0_1(release);

        self.release_const = release_const;
        self.release_ms = release_ms;

        TDLIMIT_OK
    }
}
