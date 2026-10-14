// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/mdct.h, libFDK/src/mdct.cpp and libFDK/src/arm/mdct_arm.cpp (the
// ARM file only contains NEON code, which is inactive in the reference configuration).

//! MDCT/MDST routines: forward MDCT and inverse modulated lapped transform with overlap-add.
//!
//! # Memory model
//!
//! In C++, `mdct_t` points to an overlap buffer owned by the decoder
//! (`CAacDecoderStaticChannelInfo::pOverlapBuffer`, passed to `mdct_init()`). The memory is used
//! in two ways (`union overlap { freq; time; }`): the spectral overlap of the last transform is
//! stored at its end (`overlap.freq + ov_size - tl/2 .. ov_size`), output samples that did not
//! fit into the output buffer at its start (`overlap.time[0 .. ov_offset]`).
//!
//! Here **[`MdctT`] owns the overlap buffer** ([`MdctT::overlap`]). [`mdct_init`] (re)sizes it
//! without clearing existing contents, like the C++ code (the decoder's `FDKmemclear` of
//! `pOverlapBuffer` becomes `mdct.overlap.fill(0)`). The window slopes (`prev_wrs`) are
//! `&'static [FixpWtp]` into the tables of [`tools_rom`](crate::tools_rom). The asymmetric
//! overlap (`pAsymOvlp`) is an `Option<Vec<FixpDbl>>` that is taken (reset to `None`) when it is
//! used, like the C++ code resets the pointer to `NULL`.
//!
//! [`imlt_block`] writes output samples either into `output` or, once `no_out_samples` samples
//! have been produced, into the overlap buffer, and it reads the previous spectral overlap either
//! from the overlap buffer (first window) or from the previous window of `spectrum`. This pointer
//! juggling is reproduced with (buffer, index) pairs.

use crate::digest::StateDigest;

use crate::common_fix::*;
use crate::dct::{dct_iii, dct_iv, dst_iii, dst_iv};
use crate::scale::{saturate_right_shift, scale_values_saturate};
use crate::tools_rom::FixpWtp;

/// Output additional headroom
pub const MDCT_OUT_HEADROOM: i32 = 2;
pub const PCM_OUT_BITS: i32 = DFRACT_BITS;
/// Must have the same values as DMXH_HEADROOM
pub const PCM_OUT_HEADROOM: i32 = 8;

pub const MDCT_OUTPUT_SCALE: i32 = -MDCT_OUT_HEADROOM + (DFRACT_BITS - PCM_OUT_BITS);
/// Refer to "Output word length" in ISO/IEC 14496-3:2008(E) 23.2.3.6
pub const MDCT_OUTPUT_GAIN: i32 = 16;

/// `IMDCT_SCALE(x, s)`
#[inline(always)]
pub const fn imdct_scale(x: FixpDbl, s: i32) -> FixpDbl {
    saturate_right_shift(x, s + MDCT_OUTPUT_SCALE, PCM_OUT_BITS)
}

/// `IMDCT_SCALE_DBL(x)`
#[inline(always)]
pub const fn imdct_scale_dbl(x: FixpDbl) -> FixpDbl {
    x
}

/// `IMDCT_SCALE_DBL_LSH1(x)`: `SATURATE_LEFT_SHIFT_ALT(x, 1, DFRACT_BITS)`, i.e. `2 * x`
/// saturated to `MINVAL_DBL + 1 ..= MAXVAL_DBL` (written as a saturating addition, which
/// vectorizes).
#[inline(always)]
pub const fn imdct_scale_dbl_lsh1(x: FixpDbl) -> FixpDbl {
    let r = x.saturating_add(x);
    if r < MINVAL_DBL + 1 {
        MINVAL_DBL + 1
    } else {
        r
    }
}

#[test]
fn imdct_scale_dbl_lsh1_is_saturate_left_shift_alt() {
    use crate::scale::saturate_left_shift_alt;
    let mut x = 0x2545_f491_4f6c_dd1du64;
    let edge = [0, 1, -1, 0x3fff_ffff, 0x4000_0000, -0x4000_0000, -0x4000_0001, -0x3fff_ffff];
    for v in edge.into_iter().chain([i32::MIN, i32::MIN + 1, i32::MAX, i32::MAX - 1]) {
        assert_eq!(imdct_scale_dbl_lsh1(v), saturate_left_shift_alt(v, 1, DFRACT_BITS), "{v}");
    }
    for _ in 0..1_000_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let v = (x >> 16) as i32 >> (x % 8);
        assert_eq!(imdct_scale_dbl_lsh1(v), saturate_left_shift_alt(v, 1, DFRACT_BITS), "{v}");
    }
}

pub const MLT_FLAG_CURR_ALIAS_SYMMETRY: i32 = 1;

/// `BLOCK_TYPE`
pub type BlockType = i32;
/// normal long block
pub const BLOCK_LONG: BlockType = 0;
/// long start block
pub const BLOCK_START: BlockType = 1;
/// 8 short blocks sequence
pub const BLOCK_SHORT: BlockType = 2;
/// long stop block
pub const BLOCK_STOP: BlockType = 3;

/// `WINDOW_SHAPE`
pub type WindowShape = i32;
pub const SHAPE_SINE: WindowShape = 0;
pub const SHAPE_KBD: WindowShape = 1;
pub const SHAPE_LOL: WindowShape = 2;

/// `mdct_t`: MDCT persistent data. See the [module documentation](self) for the memory model.
#[derive(StateDigest, Clone, Debug, Default)]
pub struct MdctT {
    /// Overlap memory (`overlap.freq` / `overlap.time`), owned by the MDCT.
    pub overlap: Vec<FixpDbl>,
    /// previous right window slope
    pub prev_wrs: Option<&'static [FixpWtp]>,
    /// previous transform length
    pub prev_tl: i32,
    /// previous right window offset
    pub prev_nr: i32,
    /// previous right window slope length
    pub prev_fr: i32,
    /// overlap time data fill level
    pub ov_offset: i32,
    /// Overlap buffer size in words
    pub ov_size: i32,
    pub prev_alias_symmetry: i32,
    pub prev_prev_alias_symmetry: i32,
    /// asymmetric overlap (used for stereo LPD transition), `pAsymOvlp`
    pub p_asym_ovlp: Option<Vec<FixpDbl>>,
}

/// `mdct_init(hMdct, overlap, overlapBufferSize)`: initialize a valid MDCT handle.
///
/// The overlap buffer owned by `h_mdct` is resized to `overlap_buffer_size` values; existing
/// contents are kept (new values are zero), like the C++ code which does not clear the buffer.
pub fn mdct_init(h_mdct: &mut MdctT, overlap_buffer_size: i32) {
    h_mdct.overlap.resize(overlap_buffer_size as usize, 0);
    h_mdct.prev_fr = 0;
    h_mdct.prev_nr = 0;
    h_mdct.prev_tl = 0;
    h_mdct.ov_size = overlap_buffer_size;
    h_mdct.prev_alias_symmetry = 0;
    h_mdct.prev_prev_alias_symmetry = 0;
    h_mdct.p_asym_ovlp = None;
}

/// `mdct_block(hMdct, pTimeData, noInSamples, mdctData, nSpec, tl, pRightWindowPart, fr,
/// pMdctData_e)`: forward MDCT of `n_spec` blocks of length `tl`.
///
/// This implements the forward MDCT transform on an input block of data. The input block is in a
/// form (A,B,C,D) where A,B,C and D are the respective 1/4th segments of the block. The input
/// block is folded in the form (-D-Cr,A-Br), where the 'r' suffix denotes flipping of the
/// sequence. Windowing and folding are done together; the folded block is passed to the DCT IV.
///
/// Returns the number of input samples processed.
pub fn mdct_block(
    h_mdct: &mut MdctT,
    time_data: &[IntPcm],
    no_in_samples: i32,
    mdct_data: &mut [FixpDbl],
    n_spec: i32,
    tl: i32,
    right_window_part: &'static [FixpWtp],
    fr: i32,
    p_mdct_data_e: &mut [i16],
) -> i32 {
    // tl: transform length, fl: left window slope length, nl: left window slope offset,
    // fr: right window slope length, nr: right window slope offset.
    let wrs = right_window_part;

    // Detect FRprevious / FL mismatches and override parameters accordingly
    if h_mdct.prev_fr == 0 {
        // At start just initialize and pass parameters as they are
        h_mdct.prev_fr = fr;
        h_mdct.prev_wrs = Some(wrs);
        h_mdct.prev_tl = tl;
    }

    // Derive NR
    let nr = (tl - fr) >> 1;

    // Skip input samples if tl is smaller than block size
    let mut t = ((no_in_samples - tl) >> 1) as isize;
    let tlu = tl as usize;

    for n in 0..n_spec as usize {
        // MDCT scale:
        // + 1: fMultDiv2() in windowing.
        // + 1: Because of factor 1/2 in Princen-Bradley compliant windowed TDAC.
        let mut mdct_data_e: i32 = 1 + 1;

        let td = |i: i32| time_data[(t + i as isize) as usize];
        let md = &mut mdct_data[n * tlu..(n + 1) * tlu];

        // Derive left parameters
        let wls = h_mdct.prev_wrs;
        let fl = h_mdct.prev_fr;
        let nl = (tl - fl) >> 1;

        // Left window slope offset
        for i in 0..nl {
            md[((tl / 2) + i) as usize] = -(td(tl - i - 1) >> 1); // 0(A)-Br
        }

        // Implements the folding and windowing of the left part of the sequence, that is
        // segments A and B.
        if fl / 2 > 0 {
            let wls = wls.expect("mdct_block: missing left window slope");
            for i in 0..fl / 2 {
                let w = wls[i as usize];
                let tmp0 = f_mult_div2(td(i + nl), w.im); // a*window
                md[((tl / 2) + i + nl) as usize] = f_mult_sub_div2(tmp0, td(tl - nl - i - 1), w.re);
                // A*window-Br*window
            }
        }

        // Right window slope offset
        for i in 0..nr {
            md[((tl / 2) - 1 - i) as usize] = -(td(tl + i) >> 1); // -C flipped at placing
        }

        // Implements the folding and windowing of the right part of the sequence, that is,
        // segments C and D.
        for i in 0..fr / 2 {
            let w = wrs[i as usize];
            let tmp1 = f_mult_div2(td(tl + nr + i), w.re); // C*window
                                                           // -(C*window+Dr*window) and flip before placing -> -Cr - D
            md[((tl / 2) - nr - i - 1) as usize] =
                f_mult_add_div2(tmp1, td((tl * 2) - nr - i - 1), w.im).wrapping_neg();
        }

        // We pass the shortened folded data (-D-Cr,A-Br) to the MDCT function
        dct_iv(md, &mut mdct_data_e);

        p_mdct_data_e[n] = mdct_data_e as i16;

        t += tl as isize;

        h_mdct.prev_wrs = Some(wrs);
        h_mdct.prev_fr = fr;
        h_mdct.prev_tl = tl;
    }

    n_spec * tl
}

/// `imdct_gain(&gain_m, &gain_e, tl)`: add/multiply the 2/N transform gain and the MPEG-4 part 3
/// defined output gain (see [`MDCT_OUTPUT_GAIN`]) to the given mantissa and exponent.
pub fn imdct_gain(p_gain_m: &mut FixpDbl, p_gain_e: &mut i32, tl: i32) {
    let mut gain_m = *p_gain_m;
    let mut gain_e = *p_gain_e;

    gain_e += -MDCT_OUTPUT_GAIN - MDCT_OUT_HEADROOM + 1;
    if tl == 0 {
        // Dont regard the 2/N factor from the IDCT. It is compensated for somewhere else.
        *p_gain_e = gain_e;
        return;
    }

    let log2_tl = DFRACT_BITS - 1 - f_normz(tl);
    gain_e += -log2_tl;

    // Detect non-radix 2 transform length and add amplitude compensation factor which cannot be
    // included into the exponent above
    let factor = match tl >> (log2_tl - 2) {
        // 10 ms, 1/tl = 1.0/(FDKpow(2.0, -log2_tl) * 0.53333333333333333333)
        0x7 => Some(fl2fxconst_dbl(0.53333333333333333333_f32 as f64)),
        // 3/4 of radix 2, 1/tl = 1.0/(FDKpow(2.0, -log2_tl) * 2.0/3.0)
        0x6 => Some(fl2fxconst_dbl(2.0 / 3.0)),
        // 0.8 of radix 2 (e.g. tl 160), 1/tl = 1.0/(FDKpow(2.0, -log2_tl) * 0.8/1.5)
        0x5 => Some(fl2fxconst_dbl(0.53333333333333333333_f32 as f64)),
        // radix 2, nothing to do.
        0x4 => None,
        // unsupported
        _ => {
            debug_assert!(false, "imdct_gain: unsupported transform length {tl}");
            None
        }
    };
    if let Some(factor) = factor {
        if gain_m == 0 {
            gain_m = factor;
        } else {
            gain_m = f_mult(gain_m, factor);
        }
    }

    *p_gain_m = gain_m;
    *p_gain_e = gain_e;
}

/// `imdct_drain(hMdct, output, nrSamplesRoom)`: drain buffered output samples into `output`.
/// Changes the MDCT state. Returns the number of drained samples.
pub fn imdct_drain(h_mdct: &mut MdctT, output: &mut [FixpDbl], nr_samples_room: i32) -> i32 {
    let mut buffered_samples = 0;

    if nr_samples_room > 0 {
        buffered_samples = h_mdct.ov_offset;

        debug_assert!(buffered_samples <= nr_samples_room);

        if buffered_samples > 0 {
            let n = buffered_samples as usize;
            output[..n].copy_from_slice(&h_mdct.overlap[..n]);
            h_mdct.ov_offset = 0;
        }
    }
    buffered_samples
}

/// `imdct_copy_ov_and_nr(hMdct, pTimeData, nrSamples)`: copy overlap time domain data to
/// `time_data`. Does not change the MDCT state. Returns the number of copied samples (ov + nr).
pub fn imdct_copy_ov_and_nr(h_mdct: &MdctT, time_data: &mut [FixpDbl], nr_samples: i32) -> i32 {
    let mut nr_samples = nr_samples;
    let nt = f_min(h_mdct.ov_offset, nr_samples);
    nr_samples -= nt;
    let nf = f_min(h_mdct.prev_nr, nr_samples);
    let ntu = nt.max(0) as usize;
    time_data[..ntu].copy_from_slice(&h_mdct.overlap[..ntu]);
    let mut t = ntu;

    let mut p_ovl = (h_mdct.ov_size - 1) as usize;
    if h_mdct.prev_prev_alias_symmetry == 0 {
        for _ in 0..nf {
            let x = h_mdct.overlap[p_ovl].wrapping_neg();
            p_ovl = p_ovl.wrapping_sub(1);
            time_data[t] = imdct_scale_dbl(x);
            t += 1;
        }
    } else {
        for _ in 0..nf {
            let x = h_mdct.overlap[p_ovl];
            p_ovl = p_ovl.wrapping_sub(1);
            time_data[t] = imdct_scale_dbl(x);
            t += 1;
        }
    }

    nt + nf
}

/// `imdct_copy_full(hMdct, pTimeData, nrSamples)`: copy the overlap time domain data and the full
/// previous transform to `time_data`. Does not change the MDCT state.
pub fn imdct_copy_full(h_mdct: &MdctT, time_data: &mut [FixpDbl], nr_samples: i32) -> i32 {
    let mut nr_samples = nr_samples;
    let nt = f_min(h_mdct.ov_offset, nr_samples);
    nr_samples -= nt;
    let nf = f_min(h_mdct.prev_tl >> 1, nr_samples);
    nr_samples -= nf;
    let nf2 = f_min(h_mdct.prev_tl >> 1, nr_samples);

    let ntu = nt.max(0) as usize;
    time_data[..ntu].copy_from_slice(&h_mdct.overlap[..ntu]);
    let mut t = ntu;

    let mut p_ovl = (h_mdct.ov_size - 1) as isize;
    for _ in 0..nf {
        let x = h_mdct.overlap[p_ovl as usize].wrapping_neg();
        p_ovl -= 1;
        time_data[t] = imdct_scale_dbl(x);
        t += 1;
    }
    for _ in 0..nf2 {
        p_ovl += 1;
        let x = h_mdct.overlap[p_ovl as usize].wrapping_neg();
        time_data[t] = imdct_scale_dbl(x);
        t += 1;
    }

    nt + nf + nf2
}

/// `imdct_adapt_parameters(hMdct, &fl, &nl, tl, wls, noOutSamples)`: adapt MDCT parameters for
/// non-matching window slopes.
///
/// `pfl`: left overlap window side length, `pnl`: length of the left n part of the window,
/// `wls`: left side overlap window coefficients, `no_out_samples`: desired number of output
/// samples.
pub fn imdct_adapt_parameters(
    h_mdct: &mut MdctT,
    pfl: &mut i32,
    pnl: &mut i32,
    _tl: i32,
    wls: &'static [FixpWtp],
    no_out_samples: i32,
) {
    let mut fl = *pfl;
    let mut nl = *pnl;
    let mut use_current = false;
    let mut use_previous = false;
    if h_mdct.prev_tl == 0 {
        h_mdct.prev_wrs = Some(wls);
        h_mdct.prev_fr = fl;
        h_mdct.prev_nr = (no_out_samples - fl) >> 1;
        h_mdct.prev_tl = no_out_samples;
        h_mdct.ov_offset = 0;
        use_current = true;
    }

    let window_diff = (h_mdct.prev_fr - fl) >> 1;

    // check if the previous window slope can be adjusted to match the current window slope
    if h_mdct.prev_nr + window_diff > 0 {
        use_current = true;
    }
    // check if the current window slope can be adjusted to match the previous window slope
    if nl - window_diff > 0 {
        use_previous = true;
    }

    // if both is possible choose the larger of both window slope lengths
    if use_current && use_previous && fl < h_mdct.prev_fr {
        use_current = false;
    }
    // If the previous transform block is big enough, enlarge previous window overlap, if not,
    // then shrink current window overlap.
    if use_current {
        h_mdct.prev_nr += window_diff;
        h_mdct.prev_fr = fl;
        h_mdct.prev_wrs = Some(wls);
    } else {
        nl -= window_diff;
        fl = h_mdct.prev_fr;
    }

    *pfl = fl;
    *pnl = nl;
}

/// Buffers a pointer of [`imlt_block`] can point into.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Buf {
    /// `output`
    Out,
    /// `hMdct->overlap`
    Ovl,
    /// `spectrum`
    Spec,
}

/// A C++ pointer into one of the [`Buf`] buffers.
#[derive(Clone, Copy)]
struct Ptr {
    buf: Buf,
    idx: isize,
}

impl Ptr {
    #[inline(always)]
    fn new(buf: Buf, idx: isize) -> Self {
        Ptr { buf, idx }
    }
}

struct Bufs<'a> {
    out: &'a mut [FixpDbl],
    ovl: &'a mut [FixpDbl],
    spec: &'a mut [FixpDbl],
}

impl Bufs<'_> {
    #[inline(always)]
    fn get(&self, p: Ptr) -> FixpDbl {
        let i = p.idx as usize;
        match p.buf {
            Buf::Out => self.out[i],
            Buf::Ovl => self.ovl[i],
            Buf::Spec => self.spec[i],
        }
    }

    #[inline(always)]
    fn set(&mut self, p: Ptr, v: FixpDbl) {
        let i = p.idx as usize;
        match p.buf {
            Buf::Out => self.out[i] = v,
            Buf::Ovl => self.ovl[i] = v,
            Buf::Spec => self.spec[i] = v,
        }
    }
}

crate::avx2_clone! {
/// The window crossing loop of [`imlt_block`] for `fl2` samples on slices: with
/// `(x1, x0) = cplxMultDiv2(curr[i], ±ovl[-i], w[i].re, w[i].im)` it writes `out0[i] =
/// IMDCT_SCALE_DBL_LSH1(x0)` and `out1[-i] = IMDCT_SCALE_DBL_LSH1(±x1)` (`ovl` and `out1` run
/// backwards). Handles the buffer combinations where the written ranges cannot overlap the read
/// ones (output in `output`, or output in the overlap buffer while the overlap data comes from the
/// spectrum); returns `false` for the others (the caller runs the generic code).
#[allow(clippy::too_many_arguments)]
fn window_crossing_fast(
    b: &mut Bufs,
    p_curr: Ptr,
    p_ovl: Ptr,
    p_out0: Ptr,
    p_out1: Ptr,
    fl2: usize,
    win: &[FixpWtp],
    neg_ovl: bool,
    neg_x1: bool,
) -> bool {
    if fl2 == 0
        || p_curr.buf != Buf::Spec
        || p_out0.buf != p_out1.buf
        || (p_out0.buf == Buf::Ovl && p_ovl.buf != Buf::Spec)
        || p_out0.buf == Buf::Spec
        || p_ovl.idx + 1 < fl2 as isize
        || p_out1.idx + 1 < fl2 as isize
        || p_out0.idx < 0
        || p_out0.idx + fl2 as isize > p_out1.idx + 1 - fl2 as isize
    {
        return false;
    }
    let c = p_curr.idx as usize;
    let o = (p_ovl.idx + 1) as usize - fl2;
    let a0 = p_out0.idx as usize;
    let a1 = (p_out1.idx + 1) as usize - fl2;
    let (outs, ovl, spec): (&mut [FixpDbl], &[FixpDbl], &[FixpDbl]) = match (p_out0.buf, p_ovl.buf)
    {
        (Buf::Out, Buf::Ovl) => (&mut *b.out, &*b.ovl, &*b.spec),
        (Buf::Out, Buf::Spec) => (&mut *b.out, &*b.spec, &*b.spec),
        (Buf::Ovl, Buf::Spec) => (&mut *b.ovl, &*b.spec, &*b.spec),
        _ => return false,
    };
    let (lo, hi) = outs.split_at_mut(a1);
    let out0 = &mut lo[a0..a0 + fl2];
    let out1 = &mut hi[..fl2];
    let curr = &spec[c..c + fl2];
    let ovl = &ovl[o..o + fl2];
    match (neg_ovl, neg_x1) {
        (true, true) => window_crossing::<true, true>(curr, ovl, win, out0, out1),
        (true, false) => window_crossing::<true, false>(curr, ovl, win, out0, out1),
        (false, true) => window_crossing::<false, true>(curr, ovl, win, out0, out1),
        (false, false) => window_crossing::<false, false>(curr, ovl, win, out0, out1),
    }
    true
}
}

#[inline(always)]
fn window_crossing<const NEG_OVL: bool, const NEG_X1: bool>(
    curr: &[FixpDbl],
    ovl: &[FixpDbl],
    win: &[FixpWtp],
    out0: &mut [FixpDbl],
    out1: &mut [FixpDbl],
) {
    let n = curr.len();
    let (ovl, win, out0, out1) = (&ovl[..n], &win[..n], &mut out0[..n], &mut out1[..n]);
    for i in 0..n {
        let o = ovl[n - 1 - i];
        let o = if NEG_OVL { o.wrapping_neg() } else { o };
        let (x1, x0) = cplx_mult_div2(curr[i], o, win[i].re, win[i].im);
        out0[i] = imdct_scale_dbl_lsh1(x0);
        out1[n - 1 - i] = imdct_scale_dbl_lsh1(if NEG_X1 { x1.wrapping_neg() } else { x1 });
    }
}

/// `imlt_block(hMdct, output, spectrum, scalefactor, nSpec, noOutSamples, tl, wls, fl, wrs, fr,
/// gain, flags)`: perform several inverse MLT transforms (frequency domain to time domain).
///
/// This implements the inverse modulated lapped transform, a generalized version of the inverse
/// MDCT. Setting none of the MLT_*_ALIAS_FLAG flags computes the IMDCT, setting all of them
/// computes the IMDST. Other combinations compute type III transforms used by the MPEG-H
/// multichannel tool for transitions between MDCT/MDST.
///
/// * `output`: output time domain signal (at least `no_out_samples` values).
/// * `spectrum`: the `n_spec` input spectra of length `tl` (modified in place).
/// * `scalefactor`: exponents of the input spectra.
/// * `wls`/`fl`: left side overlap window coefficients and length.
/// * `wrs`/`fr`: right side overlap window coefficients and length of all individual IMLTs.
/// * `gain`: factor to apply to the output samples (if != 0).
/// * `flags`: [`MLT_FLAG_CURR_ALIAS_SYMMETRY`].
///
/// Returns the number of output samples written to `output`.
pub fn imlt_block(
    h_mdct: &mut MdctT,
    output: &mut [FixpDbl],
    spectrum: &mut [FixpDbl],
    scalefactor: &[i16],
    n_spec: i32,
    no_out_samples: i32,
    tl: i32,
    wls: &'static [FixpWtp],
    fl: i32,
    wrs: &'static [FixpWtp],
    fr: i32,
    gain: FixpDbl,
    flags: i32,
) -> i32 {
    let mut fl = fl;
    let mut gain = gain;
    let mut nr_samples = 0;
    let mut transform_gain_e = 0;
    let curr_alias_symmetry = flags & MLT_FLAG_CURR_ALIAS_SYMMETRY;

    // Derive NR and NL
    let nr = (tl - fr) >> 1;
    let mut nl = (tl - fl) >> 1;

    // Include 2/N IMDCT gain into gain factor and exponent.
    imdct_gain(&mut gain, &mut transform_gain_e, tl);

    // Detect FRprevious / FL mismatches and override parameters accordingly
    if h_mdct.prev_fr != fl {
        imdct_adapt_parameters(h_mdct, &mut fl, &mut nl, tl, wls, no_out_samples);
    }

    // The overlap buffer is taken out of the handle while the other fields are updated.
    let mut ovl_buf = core::mem::take(&mut h_mdct.overlap);
    let ov_size = h_mdct.ov_size as isize;
    let mut b = Bufs { out: output, ovl: &mut ovl_buf, spec: spectrum };

    let mut p_ovl = Ptr::new(Buf::Ovl, ov_size - 1);
    let mut p_out0 = Ptr::new(Buf::Out, 0);

    if no_out_samples > nr_samples {
        // Purge buffered output.
        for i in 0..h_mdct.ov_offset as usize {
            let v = b.ovl[i];
            b.set(p_out0, v);
            p_out0.idx += 1;
        }
        nr_samples = h_mdct.ov_offset;
        h_mdct.ov_offset = 0;
    }

    let tlu = tl as usize;
    for w in 0..n_spec {
        // Detect FRprevious / FL mismatches and override parameters accordingly
        if h_mdct.prev_fr != fl {
            imdct_adapt_parameters(h_mdct, &mut fl, &mut nl, tl, wls, no_out_samples);
        }

        let mut spec_shift_scale = transform_gain_e;

        // Setup window pointers
        let p_window = h_mdct.prev_wrs;

        // Current spectrum
        let spec_off = w as usize * tlu;
        {
            let p_spec = &mut b.spec[spec_off..spec_off + tlu];

            // DCT IV of current spectrum.
            if curr_alias_symmetry == 0 {
                if h_mdct.prev_alias_symmetry == 0 {
                    dct_iv(p_spec, &mut spec_shift_scale);
                } else {
                    let mut tmp = [0 as FixpDbl; 1024];
                    dct_iii(p_spec, &mut tmp, &mut spec_shift_scale);
                }
            } else {
                if h_mdct.prev_alias_symmetry == 0 {
                    let mut tmp = [0 as FixpDbl; 1024];
                    dst_iii(p_spec, &mut tmp, &mut spec_shift_scale);
                } else {
                    dst_iv(p_spec, &mut spec_shift_scale);
                }
            }

            // Optional scaling of time domain - no yet windowed - of current spectrum
            // and de-scale current spectrum signal (time domain, no yet windowed)
            if gain != 0 {
                for v in p_spec.iter_mut() {
                    *v = f_mult(*v, gain);
                }
            }
            let loc_scale =
                f_min(scalefactor[w as usize] as i32 + spec_shift_scale, DFRACT_BITS - 1);
            scale_values_saturate(p_spec, loc_scale);
        }

        if no_out_samples <= nr_samples {
            // Divert output first half to overlap buffer if we already got enough output samples.
            p_out0 = Ptr::new(Buf::Ovl, h_mdct.ov_offset as isize);
            h_mdct.ov_offset += h_mdct.prev_nr + fl / 2;
        } else {
            // Account output samples
            nr_samples += h_mdct.prev_nr + fl / 2;
        }

        // NR output samples 0 .. NR. -overlap[TL/2..TL/2-NR]
        if h_mdct.prev_prev_alias_symmetry == 0 {
            for _ in 0..h_mdct.prev_nr {
                let x = b.get(p_ovl).wrapping_neg();
                p_ovl.idx -= 1;
                b.set(p_out0, imdct_scale_dbl(x));
                p_out0.idx += 1;
            }
        } else {
            for _ in 0..h_mdct.prev_nr {
                let x = b.get(p_ovl);
                p_ovl.idx -= 1;
                b.set(p_out0, imdct_scale_dbl(x));
                p_out0.idx += 1;
            }
        }

        let mut p_out1;
        if no_out_samples <= nr_samples {
            // Divert output second half to overlap buffer if we already got enough output
            // samples.
            p_out1 = Ptr::new(Buf::Ovl, (h_mdct.ov_offset + fl / 2 - 1) as isize);
            h_mdct.ov_offset += fl / 2 + nl;
        } else {
            p_out1 = Ptr::new(p_out0.buf, p_out0.idx + (fl - 1) as isize);
            nr_samples += fl / 2 + nl;
        }

        // output samples before window crossing point NR .. TL/2.
        //   -overlap[TL/2-NR..TL/2-NR-FL/2] + current[NR..TL/2]
        // output samples after window crossing point TL/2 .. TL/2+FL/2.
        //   -overlap[0..FL/2] - current[TL/2..FL/2]
        let mut p_curr = Ptr::new(Buf::Spec, (spec_off + tlu) as isize - (fl / 2) as isize);
        let fl2 = (fl / 2) as usize;
        let win: &[FixpWtp] =
            if fl2 > 0 { p_window.expect("imlt_block: missing window slope") } else { &[] };
        let neg_ovl = h_mdct.prev_prev_alias_symmetry == 0;
        let neg_x1 = h_mdct.prev_alias_symmetry == 0;
        if (h_mdct.p_asym_ovlp.is_none() || !(neg_ovl && neg_x1))
            && window_crossing_fast(
                &mut b, p_curr, p_ovl, p_out0, p_out1, fl2, win, neg_ovl, neg_x1,
            )
        {
            p_out0.idx += fl2 as isize;
            p_out1.idx -= fl2 as isize;
        } else if h_mdct.prev_prev_alias_symmetry == 0 {
            if h_mdct.prev_alias_symmetry == 0 {
                if h_mdct.p_asym_ovlp.is_none() {
                    for i in 0..fl2 {
                        let (x1, x0) = cplx_mult_div2(
                            b.get(p_curr),
                            b.get(p_ovl).wrapping_neg(),
                            win[i].re,
                            win[i].im,
                        );
                        p_curr.idx += 1;
                        p_ovl.idx -= 1;
                        b.set(p_out0, imdct_scale_dbl_lsh1(x0));
                        b.set(p_out1, imdct_scale_dbl_lsh1(x1.wrapping_neg()));
                        p_out0.idx += 1;
                        p_out1.idx -= 1;
                    }
                } else {
                    let asym = h_mdct.p_asym_ovlp.take().unwrap_or_default();
                    let mut p_asym_ovl = fl / 2 - 1;
                    for i in 0..fl2 {
                        let curr = b.get(p_curr);
                        let x1 = f_mult_div2(curr, win[i].re).wrapping_neg()
                            + f_mult_div2(asym[p_asym_ovl as usize], win[i].im);
                        let x0 =
                            f_mult_div2(curr, win[i].im) - f_mult_div2(b.get(p_ovl), win[i].re);
                        p_curr.idx += 1;
                        p_ovl.idx -= 1;
                        p_asym_ovl -= 1;
                        b.set(p_out0, imdct_scale_dbl_lsh1(x0));
                        p_out0.idx += 1;
                        b.set(p_out1, imdct_scale_dbl_lsh1(x1));
                        p_out1.idx -= 1;
                    }
                    // Note: pAsymOvlp is reset to NULL (taken above)
                }
            } else {
                // prevAliasingSymmetry == 1
                for i in 0..fl2 {
                    let (x1, x0) = cplx_mult_div2(
                        b.get(p_curr),
                        b.get(p_ovl).wrapping_neg(),
                        win[i].re,
                        win[i].im,
                    );
                    p_curr.idx += 1;
                    p_ovl.idx -= 1;
                    b.set(p_out0, imdct_scale_dbl_lsh1(x0));
                    b.set(p_out1, imdct_scale_dbl_lsh1(x1));
                    p_out0.idx += 1;
                    p_out1.idx -= 1;
                }
            }
        } else {
            // prevPrevAliasingSymmetry == 1
            if h_mdct.prev_alias_symmetry == 0 {
                for i in 0..fl2 {
                    let (x1, x0) =
                        cplx_mult_div2(b.get(p_curr), b.get(p_ovl), win[i].re, win[i].im);
                    p_curr.idx += 1;
                    p_ovl.idx -= 1;
                    b.set(p_out0, imdct_scale_dbl_lsh1(x0));
                    b.set(p_out1, imdct_scale_dbl_lsh1(x1.wrapping_neg()));
                    p_out0.idx += 1;
                    p_out1.idx -= 1;
                }
            } else {
                // prevAliasingSymmetry == 1
                for i in 0..fl2 {
                    let (x1, x0) =
                        cplx_mult_div2(b.get(p_curr), b.get(p_ovl), win[i].re, win[i].im);
                    p_curr.idx += 1;
                    p_ovl.idx -= 1;
                    b.set(p_out0, imdct_scale_dbl_lsh1(x0));
                    b.set(p_out1, imdct_scale_dbl_lsh1(x1));
                    p_out0.idx += 1;
                    p_out1.idx -= 1;
                }
            }
        }

        // NL output samples TL/2+FL/2..TL. - current[FL/2..0]
        p_out0.idx += ((fl / 2) + nl) as isize;
        p_out1.idx += ((fl / 2) + 1) as isize;
        p_curr = Ptr::new(Buf::Spec, (spec_off + tlu) as isize - (fl / 2) as isize - 1);
        if h_mdct.prev_alias_symmetry == 0 {
            for _ in 0..nl {
                let x = b.get(p_curr).wrapping_neg();
                p_curr.idx -= 1;
                b.set(p_out1, imdct_scale_dbl(x));
                p_out1.idx += 1;
            }
        } else {
            for _ in 0..nl {
                let x = b.get(p_curr);
                p_curr.idx -= 1;
                b.set(p_out1, imdct_scale_dbl(x));
                p_out1.idx += 1;
            }
        }

        // Set overlap source pointer for next window pOvl = pSpec + tl/2 - 1;
        p_ovl = Ptr::new(Buf::Spec, (spec_off + tlu / 2) as isize - 1);

        // Previous window values.
        h_mdct.prev_nr = nr;
        h_mdct.prev_fr = fr;
        h_mdct.prev_tl = tl;
        h_mdct.prev_wrs = Some(wrs);

        // Previous aliasing symmetry
        h_mdct.prev_prev_alias_symmetry = h_mdct.prev_alias_symmetry;
        h_mdct.prev_alias_symmetry = curr_alias_symmetry;
    }

    // Save overlap
    let half = tlu / 2;
    let dst = (ov_size as usize) - half;
    let src = (n_spec as usize - 1) * tlu;
    b.ovl[dst..dst + half].copy_from_slice(&b.spec[src..src + half]);

    h_mdct.overlap = ovl_buf;

    nr_samples
}

/// `imdct_block(...)`: perform several inverse MDCT transforms, see [`imlt_block`].
#[inline]
pub fn imdct_block(
    h_mdct: &mut MdctT,
    output: &mut [FixpDbl],
    spectrum: &mut [FixpDbl],
    scalefactor: &[i16],
    n_spec: i32,
    no_out_samples: i32,
    tl: i32,
    wls: &'static [FixpWtp],
    fl: i32,
    wrs: &'static [FixpWtp],
    fr: i32,
    gain: FixpDbl,
) -> i32 {
    h_mdct.prev_alias_symmetry = 0;
    h_mdct.prev_prev_alias_symmetry = 0;
    imlt_block(
        h_mdct,
        output,
        spectrum,
        scalefactor,
        n_spec,
        no_out_samples,
        tl,
        wls,
        fl,
        wrs,
        fr,
        gain,
        0,
    )
}

/// `imdst_block(...)`: perform several inverse MDST transforms, see [`imlt_block`].
#[inline]
pub fn imdst_block(
    h_mdct: &mut MdctT,
    output: &mut [FixpDbl],
    spectrum: &mut [FixpDbl],
    scalefactor: &[i16],
    n_spec: i32,
    no_out_samples: i32,
    tl: i32,
    wls: &'static [FixpWtp],
    fl: i32,
    wrs: &'static [FixpWtp],
    fr: i32,
    gain: FixpDbl,
) -> i32 {
    h_mdct.prev_alias_symmetry = 1;
    h_mdct.prev_prev_alias_symmetry = 1;
    imlt_block(
        h_mdct,
        output,
        spectrum,
        scalefactor,
        n_spec,
        no_out_samples,
        tl,
        wls,
        fl,
        wrs,
        fr,
        gain,
        MLT_FLAG_CURR_ALIAS_SYMMETRY,
    )
}
