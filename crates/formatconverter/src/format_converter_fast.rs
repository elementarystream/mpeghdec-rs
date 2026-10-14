// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: the time domain bypass of the STFT format converter (fast mode).

//! Fast mode (not in C, **not bit-exact**): time domain bypass of the STFT format converter.
//!
//! When every output channel is fed by at most one input channel and every input channel feeds
//! at most one output channel, with the unity downmix coefficient and a flat equalizer, the
//! active downmix of `IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT` is mathematically a
//! delayed copy of the input: the target energy of every ERB band equals the realized energy, so
//! the equalizer gain `sqrt(target / realized)` is 1, and the sine windowed STFT analysis and
//! synthesis (50 % overlap, `sin² + cos² = 1`) reconstruct the input delayed by one STFT frame
//! (`stftFrameSize` = 256 samples). The reference decoder still runs the STFT and the active
//! downmix in that case (e.g. stereo to stereo). Its fixed-point equalizer gains fluctuate
//! around 1 by about 1e-5, so the bypass output differs from the reference output only by the
//! reference's own rounding noise (see `docs/research.md#relaxed-exactness` and
//! `scripts/fast_accuracy.py`).
//!
//! [`format_converter_bypass_map`] derives the conditions from the converter state every frame;
//! [`format_converter_bypass`] replaces the STFT processing of a frame by
//! `out[o][t] += fMult(in[i][t - stftFrameSize], dmx[i][o])`. The delay line of an input
//! channel is the `prevAudioInput` buffer of its STFT analysis filterbank, which holds the last
//! `stftFrameSize` input samples in both paths, so the switch from the STFT processing to the
//! bypass is seamless. [`format_converter_leave_bypass`] prepares the STFT state for the switch
//! back (the overlap-add buffers of the synthesis filterbanks, the headroom prescaling, the
//! active downmix state).

use crate::format_converter_active_dmx_stft::ActiveDownmixer;
use crate::format_converter_constants_stft::STFT_ERB_BANDS;
use crate::format_converter_intern::*;
use crate::format_converter_lib::IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::digest::StateDigest;
use mpeghdec_fdk::scale::get_scalefactor;
use mpeghdec_fdk::stftfilterbank::StftFilterbank;

/// The downmix coefficient of a 1:1 mapping that the bypass reproduces: 1.0 in `FIXP_DMX_H`
/// (`FL2FXCONST_SGL(1.0)` saturates to `MAXVAL_SGL`).
pub const BYPASS_UNITY_COEFFICIENT: FixpDmxH = MAXVAL_DMX_H;

/// The channel mapping of a format converter whose STFT processing is equivalent to a delayed
/// copy (see [`format_converter_bypass_map`]).
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct BypassMap {
    /// Per output channel: the input channel feeding it and the downmix coefficient, `None` for
    /// a silent output channel.
    pub src: [Option<(u8, FixpDmxH)>; NCHANOUT_MAX],
}

/// The equalizer `activeDmxProcess_STFT()` uses for the equalizer index `idx` and the value of
/// a gain of 1.0 in its format (`None` if the index selects no equalizer; the active downmix then
/// uses the neutral equalizer of the default format, see there).
fn equalizer(p: &IisFormatConverterInternal, idx: i32) -> Option<(&[FixpEqH], FixpEqH)> {
    let params = p.fc_params.as_deref()?;
    let idx = usize::try_from(idx).ok().filter(|&i| i < N_EQ)?;
    if p.amount_of_added_dmx_matrices_and_equalizers != 0 {
        /* equalizers of the bitstream (EQ_BITSTREAM_H_EXP) */
        if idx >= p.amount_of_added_dmx_matrices_and_equalizers as usize {
            return None;
        }
        let eq = p.eq_gains.get(idx)?.as_deref()?;
        Some((eq, FIXP_EQ_BITSTREAM_H_FORMAT_1_DOT_0))
    } else if idx == 0 {
        Some((&[FIXP_EQ_H_FORMAT_1_DOT_0; STFT_ERB_BANDS], FIXP_EQ_H_FORMAT_1_DOT_0))
    } else if (RULE_EQ1 as usize..=RULE_EQ5 as usize).contains(&idx) {
        let pr = params.format_converter_params_internal.as_deref()?;
        Some((&pr.eq[idx - 1], FIXP_EQ_H_FORMAT_1_DOT_0))
    } else {
        /* no equalizer: the neutral one */
        Some((&[FIXP_EQ_H_FORMAT_1_DOT_0; STFT_ERB_BANDS], FIXP_EQ_H_FORMAT_1_DOT_0))
    }
}

fn stft_config_ok(fb: Option<&StftFilterbank>, frame_size: u32) -> bool {
    fb.is_some_and(|fb| {
        let c = &fb.stft_filterbank_config;
        c.frame_size == frame_size
            && c.fft_size == 2 * frame_size
            && fb.prev_audio_input.len() >= frame_size as usize
            && fb.window_fall.len() >= frame_size as usize
    })
}

/// The channel mapping if the STFT processing of the format converter `p` is equivalent to a
/// delayed copy of the input channels, `None` otherwise. All of the following must hold:
///
/// - the converter is open in the mode `IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT` (the
///   one the decoder uses) with the STFT filterbanks of all input and output channels, a 50 %
///   overlap without zero padding (`fftSize == 2 * stftFrameSize`) and a frame of whole STFT
///   frames;
/// - no immersive downmix (no 3D / 2D rendering with separate low / high frequency matrices and
///   no delayed top channels): one downmix matrix for all ERB bands, the same delay for all
///   channels;
/// - the downmix matrix used by the active downmix (`dmxMatrixL_FDK`) has at most one non-zero
///   coefficient per input channel (row) and per output channel (column), and every non-zero
///   coefficient is [`BYPASS_UNITY_COEFFICIENT`];
/// - the equalizer of every non-zero coefficient is flat (all 58 ERB band gains are exactly 1.0).
///
/// The caller must additionally make sure that no DRC is applied in the STFT domain (the
/// multiband DRC of `IIS_FormatConverter_Process()`), which the bypass does not reproduce.
pub fn format_converter_bypass_map(p: &IisFormatConverterInternal) -> Option<BypassMap> {
    if p.mode != IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT {
        return None;
    }
    let params = p.fc_params.as_deref()?;
    let h = p.fc_state.as_deref()?.handle_active_dmx_stft.as_deref()?;
    let n_in = p.num_total_input_channels as usize;
    let n_out = p.num_output_channels as usize;
    let sfs = p.stft_frame_size;
    if n_in == 0
        || n_in > NCHANIN_MAX
        || n_out > NCHANOUT_MAX
        || h.num_in_chans as usize != n_in
        || h.num_out_chans as usize != n_out
        || sfs == 0
        || p.num_time_slots == 0
        || p.num_time_slots * sfs != p.frame_size
    {
        return None;
    }
    /* no immersive mode */
    if p.immersive_downmix_flag != 0
        || params.immersive_mode != 0
        || params.mode3_drendering != 0
        || params.mode2_drendering != 0
        || params.dmx_iterations != 1
    {
        return None;
    }
    if !(0..n_in).all(|i| stft_config_ok(p.stft_filterbank_analysis[i].as_deref(), sfs))
        || !(0..n_out).all(|o| stft_config_ok(p.stft_filterbank_synthesis[o].as_deref(), sfs))
    {
        return None;
    }
    let mtx = params.dmx_sorted(params.dmx_matrix_l_fdk);
    let eq_idx = params.eq_index_sorted(params.eq_index_fdk);
    if mtx.len() < n_in * n_out || eq_idx.len() < n_in * n_out {
        return None;
    }
    let mut map = BypassMap { src: [None; NCHANOUT_MAX] };
    for i in 0..n_in {
        let mut used = false;
        for o in 0..n_out {
            let c = mtx[i * n_out + o];
            if c == 0 {
                continue;
            }
            if used || map.src[o].is_some() || c != BYPASS_UNITY_COEFFICIENT {
                return None;
            }
            let (eq, one) = equalizer(p, eq_idx[i * n_out + o])?;
            if eq.len() < STFT_ERB_BANDS || eq[..STFT_ERB_BANDS].iter().any(|&g| g != one) {
                return None;
            }
            used = true;
            map.src[o] = Some((i as u8, c));
        }
    }
    Some(map)
}

/// `out[t] += fMult(x[t], c)`
#[inline]
fn mix(out: &mut [FixpDbl], x: &[FixpDbl], c: FixpDmxH) {
    for (y, &v) in out.iter_mut().zip(x) {
        *y = y.wrapping_add(f_mult(v, c));
    }
}

/// Processes one frame with the time domain bypass if [`format_converter_bypass_map`] allows it
/// (returns `false` without changing anything otherwise). Same buffers as
/// `IIS_FormatConverter_Process()`: the delayed input channels are added to `out_buffer`
/// (`numOutputChannels` blocks of `frameSize` samples), the input is not modified.
pub fn format_converter_bypass(
    p: &mut IisFormatConverterInternal,
    dein_buffer: &[Option<&mut [FixpDbl]>],
    out_buffer: &mut [FixpDbl],
) -> bool {
    let Some(map) = format_converter_bypass_map(p) else { return false };
    let fs = p.frame_size as usize;
    let d = p.stft_frame_size as usize;
    let n_in = p.num_total_input_channels as usize;
    let n_out = p.num_output_channels as usize;
    if out_buffer.len() < n_out * fs
        || dein_buffer.len() < n_in
        || dein_buffer[..n_in].iter().any(|x| x.as_ref().is_some_and(|x| x.len() < fs))
    {
        return false;
    }

    for (o, out) in out_buffer.chunks_exact_mut(fs).take(n_out).enumerate() {
        let Some((i, c)) = map.src[o] else { continue };
        let i = i as usize;
        let prev = &p.stft_filterbank_analysis[i].as_deref().unwrap().prev_audio_input;
        let (head, tail) = out.split_at_mut(d);
        /* the last stftFrameSize samples of the previous frame */
        mix(head, &prev[..d], c);
        /* a NULL input channel is silent */
        if let Some(inp) = dein_buffer[i].as_deref() {
            mix(tail, &inp[..fs - d], c);
        }
    }
    /* update the delay lines */
    for (i, inp) in dein_buffer[..n_in].iter().enumerate() {
        let prev = &mut p.stft_filterbank_analysis[i].as_deref_mut().unwrap().prev_audio_input;
        match inp.as_deref() {
            Some(inp) => prev[..d].copy_from_slice(&inp[fs - d..fs]),
            None => prev[..d].fill(0),
        }
    }

    p.fast_bypass = Some(Box::new(map));
    p.fast_bypass_frames += 1;
    true
}

/// Prepares the STFT processing after frames processed by [`format_converter_bypass`] (does
/// nothing if the previous frame was not bypassed), so that its output continues the delayed
/// copy:
///
/// - the overlap-add buffer of every synthesis filterbank gets the second half of the
///   (windowed) output of the last time slot, i.e. `fMult(fMult(fMult(x, c), w), w)` of the last
///   `stftFrameSize` input samples `x` of the mapped input channel with the falling half `w` of
///   the sine window (the analysis filterbanks already hold these samples);
/// - `stftHeadroomPrescaling` is the prescaling of the last time slot that the STFT processing
///   would have kept (from the headroom of the samples in the analysis filterbanks);
/// - the active downmix state (smoothed energies and their headroom) starts again like after
///   `activeDmxStftInit()`, as at the start of a stream.
pub fn format_converter_leave_bypass(p: &mut IisFormatConverterInternal) {
    let Some(map) = p.fast_bypass.take() else { return };
    let d = p.stft_frame_size as usize;
    let n_in = p.num_total_input_channels as usize;
    let n_out = p.num_output_channels as usize;

    for o in 0..n_out {
        let Some(syn) = p.stft_filterbank_synthesis[o].as_deref_mut() else { continue };
        let prev = &mut syn.prev_audio_input[..d];
        match map.src[o] {
            Some((i, c)) => {
                let x =
                    &p.stft_filterbank_analysis[i as usize].as_deref().unwrap().prev_audio_input;
                for ((y, &v), &w) in prev.iter_mut().zip(&x[..d]).zip(&syn.window_fall[..d]) {
                    *y = f_mult(f_mult(f_mult(v, c), w), w);
                }
            }
            None => prev.fill(0),
        }
    }

    let mut stft_headroom = 31;
    for fb in p.stft_filterbank_analysis[..n_in].iter().flatten() {
        stft_headroom = f_min(stft_headroom, get_scalefactor(&fb.prev_audio_input[..d]));
    }
    /* like IIS_FormatConverter_Process_STFT() */
    let mut stft_headroom_prescaling = f_min(f_max(stft_headroom - 8, 0), 8);
    if stft_headroom == 0 {
        stft_headroom_prescaling = -1;
    }
    p.stft_headroom_prescaling = stft_headroom_prescaling;

    if let Some(h) = p.fc_state.as_deref_mut().and_then(|s| s.handle_active_dmx_stft.as_deref_mut())
    {
        reset_active_downmix(h);
    }
}

/// The state of the active downmix after `activeDmxStftInit()` (the parameters are kept).
fn reset_active_downmix(h: &mut ActiveDownmixer) {
    for v in h.target_ene_prev.iter_mut().chain(h.realized_ene_prev.iter_mut()) {
        v.fill(0);
    }
    h.target_ene_prev_exp.fill(0);
    h.in_buf_stft_headroom_prev = [31; STFT_ERB_BANDS];
    h.realized_sig_headroom_prev = [31; STFT_ERB_BANDS];
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format_converter_lib::*;
    use mpeghdec_fdk::cicp2geometry::*;

    const FS: usize = 1024;
    const DELAY: usize = 256;

    /// A format converter set up like `CAacDecoder_InitRenderer()` does for a channel signal
    /// group of layout `cicp_in` and the target layout `cicp_out`.
    fn converter(cicp_in: i32, cicp_out: i32, aes: u32) -> (Box<IisFormatConverter>, usize, usize) {
        let geo = |cicp: i32| {
            let mut g = [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
            let (mut ch, mut lfe) = (0, 0);
            assert_eq!(
                cicp2geometry_get_geometry_from_cicp(cicp, Some(&mut g), &mut ch, &mut lfe),
                0
            );
            (g, (ch + lfe) as usize)
        };
        let (out_geo, n_out) = geo(cicp_out);
        let (mut in_geo, n_in) = geo(cicp_in);
        for g in in_geo.iter_mut() {
            g.cicp_loudspeaker_index = -1;
        }
        let mut fc = None;
        let mode = IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT;
        assert_eq!(
            iis_format_converter_create(&mut fc, mode, &out_geo, n_out as u32, 48000, 1024),
            0
        );
        let mut fc = fc.unwrap();
        assert_eq!(
            iis_format_converter_config_add_input_setup(&mut fc, &in_geo, n_in as u32, 0, None),
            0
        );
        fc.num_signals_total += n_in as u32;
        iis_format_converter_config_set_aes(&mut fc, aes);
        iis_format_converter_config_set_pas(&mut fc, 0);
        assert_eq!(iis_format_converter_open(&mut fc, 4 * 24 * 3072), 0);
        (fc, n_in, n_out)
    }

    /// Full scale of the time signal of the decoder (`PCM_OUT_HEADROOM` = 8 bits of headroom:
    /// one unit is one LSB of the 24 bit output).
    const FULL_SCALE: f64 = (1 << 23) as f64;

    /// Input frame `k` of `n` channels: sines with changing amplitudes (up to 90 % of the full
    /// scale).
    fn input(k: usize, n: usize) -> Vec<i32> {
        let mut v = vec![0i32; n * FS];
        for ch in 0..n {
            let amp = [0.45, 0.02, 0.3, 0.0005, 0.9][(k / 3 + ch) % 5] * FULL_SCALE;
            for t in 0..FS {
                let x = (k * FS + t) as f64;
                let s =
                    (x * 0.013 * (ch + 1) as f64).sin() * 0.7 + (x * 0.29 + ch as f64).sin() * 0.3;
                v[ch * FS + t] = (amp * s) as i32;
            }
        }
        v
    }

    /// Processes `frames` frames; `bypass(k)` selects the fast mode of frame `k`. Returns the
    /// output (`n_out` channels of `frames * FS` samples) and the number of bypassed frames.
    fn run(
        cicp_in: i32,
        cicp_out: i32,
        aes: u32,
        frames: usize,
        bypass: impl Fn(usize) -> bool,
    ) -> (Vec<Vec<i32>>, u64) {
        let (mut fc, n_in, n_out) = converter(cicp_in, cicp_out, aes);
        let mut out = vec![Vec::new(); n_out];
        for k in 0..frames {
            let mut inp = input(k, n_in);
            let mut dein: Vec<Option<&mut [i32]>> = inp.chunks_mut(FS).map(Some).collect();
            let mut o = vec![0i32; n_out * FS];
            let err = iis_format_converter_process_fast(
                &mut fc,
                None,
                None,
                &mut dein,
                &mut o,
                FS as i32,
                bypass(k),
            );
            assert_eq!(err, 0);
            for (ch, out) in out.iter_mut().enumerate() {
                out.extend_from_slice(&o[ch * FS..(ch + 1) * FS]);
            }
        }
        (out, fc.member.fast_bypass_frames)
    }

    fn max_diff(a: &[Vec<i32>], b: &[Vec<i32>]) -> i64 {
        a.iter()
            .flatten()
            .zip(b.iter().flatten())
            .map(|(&x, &y)| (x as i64 - y as i64).abs())
            .max()
            .unwrap()
    }

    /// Maximum deviation of the bypass from the STFT processing in LSB of 24 bit (the tolerance
    /// of `scripts/fast_accuracy.py`). The rounding noise of the STFT processing of the x86 variant
    /// is slightly larger (its `fMult()` truncates to even results).
    const TOL: i64 = if mpeghdec_fdk::ARCH_X86 { 1280 } else { 1024 };

    #[test]
    fn identity_is_a_delayed_copy() {
        for (cicp, aes) in [(6, 7), (2, 7), (13, 7), (19, 7), (6, 0)] {
            let (fast, n) = run(cicp, cicp, aes, 12, |_| true);
            assert_eq!(n, 12);
            let n_in = fast.len();
            let inputs: Vec<Vec<i32>> = (0..12).map(|k| input(k, n_in)).collect();
            /* exactly the input delayed by 256 samples with the unity coefficient */
            for (ch, out) in fast.iter().enumerate() {
                for (t, &y) in out.iter().enumerate() {
                    let x = match t.checked_sub(DELAY) {
                        None => 0,
                        Some(s) => inputs[s / FS][ch * FS + s % FS],
                    };
                    assert_eq!(y, f_mult(x, BYPASS_UNITY_COEFFICIENT), "cicp {cicp} ch {ch} t {t}");
                }
            }
            let (exact, n) = run(cicp, cicp, aes, 12, |_| false);
            assert_eq!(n, 0);
            let d = max_diff(&exact, &fast);
            println!("cicp {cicp} aes {aes}: bypass differs from the STFT by {d}");
            assert!(d <= TOL, "cicp {cicp} aes {aes}: bypass differs from the STFT by {d}");
        }
    }

    #[test]
    fn transitions() {
        /* switching on and off after every run length of 1 to 4 frames */
        let pattern = [0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1];
        for (cicp, aes) in [(6, 7), (2, 7), (19, 0)] {
            let (exact, _) = run(cicp, cicp, aes, pattern.len(), |_| false);
            let (mixed, n) = run(cicp, cicp, aes, pattern.len(), |k| pattern[k] != 0);
            assert_eq!(n, pattern.iter().sum::<u64>());
            let d = max_diff(&exact, &mixed);
            println!("cicp {cicp} aes {aes}: switching differs from the STFT by {d}");
            assert!(d <= TOL, "cicp {cicp}: switching differs from the STFT by {d}");
        }
    }

    #[test]
    fn mapping_conditions() {
        let map = |i, o| {
            let (fc, _, _) = converter(i, o, 7);
            format_converter_bypass_map(&fc.member)
        };
        /* same layout: identity */
        let m = map(6, 6).unwrap();
        assert!((0..6).all(|o| m.src[o] == Some((o as u8, BYPASS_UNITY_COEFFICIENT))));
        /* stereo into 5.1: L and R are copied, the other channels stay silent */
        let m = map(2, 6).unwrap();
        assert_eq!(m.src.iter().filter(|s| s.is_some()).count(), 2);
        /* real downmixes */
        assert!(map(6, 2).is_none());
        assert!(map(13, 2).is_none());
        assert!(map(19, 6).is_none());
        assert!(map(2, 1).is_none());
    }

    #[test]
    fn downmix_is_not_bypassed() {
        let (exact, _) = run(6, 2, 7, 6, |_| false);
        let (fast, n) = run(6, 2, 7, 6, |_| true);
        assert_eq!(n, 0);
        assert_eq!(exact, fast);
    }
}
