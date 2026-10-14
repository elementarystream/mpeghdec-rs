// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/rfft.h, libFDK/src/rfft.cpp and libFDK/src/arm/rfft_arm.cpp (the
// AArch64 build compiles the generic C variants of fft_postsort()/fft_presort()).

//! Real valued FFT using a complex FFT of half length.

use crate::common_fix::*;
use crate::fft::fft;
use crate::tools_rom::{FixpStp, SINE_TABLE_512};

/// `rfft(fftLength, pBuffer, bufferSize, scalefactor)`: in-place real FFT using a complex FFT of
/// half length.
///
/// An FFT of a real input signal with N samples results in N/2+1 complex output samples. If the
/// buffer declared by `buffer_len` holds at least N+2 values, the real part of the (N/2+1)th value
/// is stored at `buffer[N]` (and the imaginary parts of the first and last value are zero);
/// otherwise it is stored into the imaginary part of the first complex value.
///
/// `buffer_len` corresponds to the C++ `bufferSize / sizeof(FIXP_DBL)`, i.e. the buffer size in
/// `FIXP_DBL` values the caller declares (it may be smaller than `buffer.len()`). The input data
/// must have at least 1 bit headroom. Supported lengths: 128, 256 and 512.
///
/// The pre/post processing uses wrapping arithmetic like the C++ code, which may overflow for
/// full scale input.
pub fn rfft(
    fft_length: u32,
    buffer: &mut [FixpDbl],
    buffer_len: u32,
    scalefactor: &mut i32,
) -> i32 {
    fft((fft_length / 2) as i32, buffer, scalefactor);

    fft_postsort(buffer, fft_length as i32, scalefactor);

    if buffer_len >= fft_length + 2 {
        let n = fft_length as usize;
        buffer[n] = buffer[1];
        buffer[n + 1] = 0;
        buffer[1] = 0;
    }

    0
}

/// `irfft(fftLength, pBuffer, bufferSize, scalefactor)`: in-place real IFFT using a complex FFT of
/// half length.
///
/// The buffer holds N/2+1 complex spectral values; the imaginary part of the first and the last
/// value must be zero. Alternatively the buffer holds N/2 complex values and the real part of the
/// (N/2+1)th value is placed into the imaginary part of the first value (if `buffer_len` < N+1).
/// See [`rfft`] for `buffer_len`.
///
/// Note: like the C++ code, the scale of the result is not added to `scalefactor`.
pub fn irfft(
    fft_length: u32,
    buffer: &mut [FixpDbl],
    buffer_len: u32,
    _scalefactor: &mut i32,
) -> i32 {
    let mut fft_e = 0;

    if buffer_len > fft_length {
        buffer[1] = buffer[fft_length as usize];
    }

    fft_presort(buffer, fft_length as i32, &mut fft_e);

    fft((fft_length / 2) as i32, buffer, &mut fft_e);

    0
}

/// `getSineTab(length, &pSineTab, &step)`: returns the sine table and step used by the pre/post
/// processing of a real FFT of the given length (`(None, 0)` for unsupported lengths).
pub fn get_sine_tab(length: u32) -> (Option<&'static [FixpStp]>, u32) {
    match length {
        64 => (Some(&SINE_TABLE_512), 1 << (9 - 4)),
        128 => (Some(&SINE_TABLE_512), 1 << (9 - 5)),
        256 => (Some(&SINE_TABLE_512), 1 << (9 - 6)),
        512 => (Some(&SINE_TABLE_512), 1 << (9 - 7)),
        _ => (None, 0),
    }
}

/// `fft_postsort()`
fn fft_postsort(buf: &mut [FixpDbl], n: i32, scalefactor: &mut i32) {
    *scalefactor += 1;

    let (tab, step) = get_sine_tab(n as u32);
    let tab = tab.expect("rfft: unsupported FFT length");
    let step = step as usize;
    let n = n as usize;

    let tmp1 = buf[0];
    buf[0] = (tmp1 >> 1).wrapping_add(buf[1] >> 1);
    buf[1] = (tmp1 >> 1).wrapping_sub(buf[1] >> 1);

    // The iterations are independent of each other (each one reads and writes its own four
    // complex values); the vectorized variant computes the first ones.
    let start = post_presort_fast(buf, n, true);
    let mut p0 = 2 + 2 * start;
    let mut p1 = n - 2 - 2 * start;
    let mut p2 = n / 2 - 2 - 2 * start;
    let mut p3 = n / 2 + 2 + 2 * start;
    let mut w = step * (1 + start);

    for _ in start..((n >> 3) - 1) {
        let sre = (buf[p0] >> 1).wrapping_add(buf[p1] >> 1);
        let sim = (buf[p0 + 1] >> 1).wrapping_add(buf[p1 + 1] >> 1);
        let dre = (buf[p0] >> 1).wrapping_sub(buf[p1] >> 1);
        let dim = (buf[p0 + 1] >> 1).wrapping_sub(buf[p1 + 1] >> 1);

        let (tmp1, tmp2) = cplx_mult_div2(sim, dre, tab[w].re, tab[w].im);

        buf[p0] = (sre >> 1).wrapping_add(tmp1);
        buf[p0 + 1] = (dim >> 1).wrapping_sub(tmp2);
        buf[p1] = (sre >> 1).wrapping_sub(tmp1);
        buf[p1 + 1] = (dim >> 1).wrapping_add(tmp2).wrapping_neg();

        let sre = (buf[p2] >> 1).wrapping_add(buf[p3] >> 1);
        let sim = (buf[p2 + 1] >> 1).wrapping_add(buf[p3 + 1] >> 1);
        let dre = (buf[p2] >> 1).wrapping_sub(buf[p3] >> 1);
        let dim = (buf[p2 + 1] >> 1).wrapping_sub(buf[p3 + 1] >> 1);

        let (tmp1, tmp2) = cplx_mult_div2(dre, sim, tab[w].re, tab[w].im);

        buf[p2] = (sre >> 1).wrapping_sub(tmp1);
        buf[p2 + 1] = (dim >> 1).wrapping_sub(tmp2);
        buf[p3] = (sre >> 1).wrapping_add(tmp1);
        buf[p3 + 1] = (dim >> 1).wrapping_add(tmp2).wrapping_neg();

        p0 += 2;
        p1 -= 2;
        p2 -= 2;
        p3 += 2;
        w += step;
    }
    // Last iteration: here p0 == p2 and p1 == p3.
    {
        let sre = (buf[p2] >> 1).wrapping_add(buf[p3] >> 1);
        let sim = (buf[p2 + 1] >> 1).wrapping_add(buf[p3 + 1] >> 1);
        let dre = (buf[p2] >> 1).wrapping_sub(buf[p3] >> 1);
        let dim = (buf[p2 + 1] >> 1).wrapping_sub(buf[p3 + 1] >> 1);

        let (tmp1, tmp2) = cplx_mult_div2(sim, dre, tab[w].re, tab[w].im);

        buf[p0] = (sre >> 1).wrapping_add(tmp1);
        buf[p0 + 1] = (dim >> 1).wrapping_sub(tmp2);
        buf[p1] = (sre >> 1).wrapping_sub(tmp1);
        buf[p1 + 1] = (dim >> 1).wrapping_add(tmp2).wrapping_neg();
    }

    buf[n >> 1] >>= 1;
    buf[(n >> 1) + 1] = buf[(n >> 1) + 1].wrapping_neg() >> 1;
}

/// `fft_presort()`
fn fft_presort(buf: &mut [FixpDbl], n: i32, scalefactor: &mut i32) {
    *scalefactor += 1;

    let (tab, step) = get_sine_tab(n as u32);
    let tab = tab.expect("irfft: unsupported FFT length");
    let step = step as usize;
    let n = n as usize;

    let tmp1 = buf[0];
    buf[0] = ((tmp1 >> 1).wrapping_add(buf[1] >> 1)) >> 1;
    buf[1] = ((tmp1 >> 1).wrapping_sub(buf[1] >> 1)) >> 1;

    let start = post_presort_fast(buf, n, false);
    let mut p0 = 2 + 2 * start;
    let mut p1 = n - 2 - 2 * start;
    let mut p2 = n / 2 - 2 - 2 * start;
    let mut p3 = n / 2 + 2 + 2 * start;
    let mut w = step * (1 + start);

    for _ in start..((n >> 3) - 1) {
        let sre = (buf[p0] >> 1).wrapping_add(buf[p1] >> 1);
        let sim = (buf[p0 + 1] >> 1).wrapping_add(buf[p1 + 1] >> 1);
        let dre = (buf[p0] >> 1).wrapping_sub(buf[p1] >> 1);
        let dim = (buf[p0 + 1] >> 1).wrapping_sub(buf[p1 + 1] >> 1);

        let (tmp2, tmp1) = cplx_mult_div2(dre, sim, tab[w].re, tab[w].im);

        buf[p0] = (sre >> 1).wrapping_add(tmp1);
        buf[p0 + 1] = tmp2.wrapping_sub(dim >> 1);
        buf[p1] = (sre >> 1).wrapping_sub(tmp1);
        buf[p1 + 1] = tmp2.wrapping_add(dim >> 1);

        let sre = (buf[p2] >> 1).wrapping_add(buf[p3] >> 1);
        let sim = (buf[p2 + 1] >> 1).wrapping_add(buf[p3 + 1] >> 1);
        let dre = (buf[p2] >> 1).wrapping_sub(buf[p3] >> 1);
        let dim = (buf[p2 + 1] >> 1).wrapping_sub(buf[p3 + 1] >> 1);

        let (tmp2, tmp1) = cplx_mult_div2(sim, dre, tab[w].re, tab[w].im);

        buf[p2] = (sre >> 1).wrapping_add(tmp1);
        buf[p2 + 1] = (dim >> 1).wrapping_add(tmp2).wrapping_neg();
        buf[p3] = (sre >> 1).wrapping_sub(tmp1);
        buf[p3 + 1] = (dim >> 1).wrapping_sub(tmp2);

        p0 += 2;
        p1 -= 2;
        p2 -= 2;
        p3 += 2;
        w += step;
    }
    // Last iteration: here p0 == p2 and p1 == p3.
    {
        let sre = (buf[p2] >> 1).wrapping_add(buf[p3] >> 1);
        let sim = (buf[p2 + 1] >> 1).wrapping_add(buf[p3 + 1] >> 1);
        let dre = (buf[p2] >> 1).wrapping_sub(buf[p3] >> 1);
        let dim = (buf[p2 + 1] >> 1).wrapping_sub(buf[p3 + 1] >> 1);

        let (tmp2, tmp1) = cplx_mult_div2(sim, dre, tab[w].re, tab[w].im);

        buf[p2] = (sre >> 1).wrapping_add(tmp1);
        buf[p2 + 1] = (dim >> 1).wrapping_add(tmp2).wrapping_neg();
        buf[p3] = (sre >> 1).wrapping_sub(tmp1);
        buf[p3 + 1] = (dim >> 1).wrapping_sub(tmp2);
    }

    buf[(n >> 1) + 1] = buf[(n >> 1) + 1].wrapping_neg() >> 1;
    buf[n >> 1] >>= 1;
}

/// Computes the first iterations of the main loop of `fft_postsort()` (`post`) or
/// `fft_presort()` with NEON and returns the number of iterations done (0 without NEON).
#[inline(always)]
fn post_presort_fast(buf: &mut [FixpDbl], n: usize, post: bool) -> usize {
    // The NEON code reproduces the AArch64 arithmetic of the reference.
    #[cfg(target_arch = "aarch64")]
    if let Some(tw) = fast::twiddles(n).filter(|_| !crate::ARCH_X86) {
        assert!(buf.len() >= n);
        let chunks = ((n >> 3) - 1) / 4;
        // SAFETY: `buf` holds `n` values; `run` accesses the complex values 1 .. n/2 - 1.
        unsafe { fast::run(buf.as_mut_ptr(), n, tw, chunks, post) };
        return 4 * chunks;
    }
    // The AVX2 code reproduces the x86 arithmetic of the reference.
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    if crate::ARCH_X86 && crate::simd::avx2() {
        if let Some(tw) = fast_x86::twiddles(n) {
            assert!(buf.len() >= n);
            let chunks = ((n >> 3) - 1) / 8;
            // SAFETY: AVX2 is available; `buf` holds `n` values; `run` accesses the complex
            // values 1 .. n/2 - 1.
            unsafe { fast_x86::run(buf.as_mut_ptr(), n, tw, chunks, post) };
            return 8 * chunks;
        }
    }
    let _ = (buf, n, post);
    0
}

/// AVX2 variants of the main loops of `fft_postsort()` and `fft_presort()` with the arithmetic
/// of the x86 build: eight consecutive iterations in parallel like the NEON variant (see
/// `fast`). In the x86 build, `cplxMultDiv2()` subtracts or adds the individually truncated
/// products `M(x, w) = (x * w) >> 16`, which are computed exactly in 32 bit lanes as
/// `(x >> 16) * w + (((x & 0xFFFF) * w) >> 16)`.
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
mod fast_x86 {
    use super::*;
    use crate::simd::x86::*;
    use std::sync::OnceLock;

    /// Twiddle factors `tab[(i + 1) * step]` of the iterations `i`.
    pub struct Twiddles {
        re: Box<[i32]>,
        im: Box<[i32]>,
    }

    pub fn twiddles(n: usize) -> Option<&'static Twiddles> {
        static TW: [OnceLock<Twiddles>; 4] = [const { OnceLock::new() }; 4];
        let idx = match n {
            64 => 0,
            128 => 1,
            256 => 2,
            512 => 3,
            _ => return None,
        };
        Some(TW[idx].get_or_init(|| {
            let (tab, step) = get_sine_tab(n as u32);
            let (tab, step) = (tab.unwrap(), step as usize);
            let t = |i: usize| tab[(i + 1) * step];
            let cnt = (n >> 3) - 1;
            Twiddles {
                re: (0..cnt).map(|i| t(i).re as i32).collect(),
                im: (0..cnt).map(|i| t(i).im as i32).collect(),
            }
        }))
    }

    /// Sum and difference terms `(sre >> 1, sim, dre, dim >> 1)` of two complex values.
    #[inline(always)]
    unsafe fn sums(
        x: (__m256i, __m256i),
        y: (__m256i, __m256i),
    ) -> (__m256i, __m256i, __m256i, __m256i) {
        let (xr, xi) = (_mm256_srai_epi32::<1>(x.0), _mm256_srai_epi32::<1>(x.1));
        let (yr, yi) = (_mm256_srai_epi32::<1>(y.0), _mm256_srai_epi32::<1>(y.1));
        (
            _mm256_srai_epi32::<1>(_mm256_add_epi32(xr, yr)),
            _mm256_add_epi32(xi, yi),
            _mm256_sub_epi32(xr, yr),
            _mm256_srai_epi32::<1>(_mm256_sub_epi32(xi, yi)),
        )
    }

    /// Iterations `0 .. 8 * chunks` of the main loop of `fft_postsort()` (`post`) or
    /// `fft_presort()`; `buf` holds `n` values.
    #[target_feature(enable = "avx2")]
    pub unsafe fn run(buf: *mut i32, n: usize, tw: &Twiddles, chunks: usize, post: bool) {
        for ch in 0..chunks {
            let i = 8 * ch;
            let (a, b, c, d) = (1 + i, n / 2 - 1 - i, n / 4 - 1 - i, n / 4 + 1 + i);
            let tr = _mm256_loadu_si256(tw.re.as_ptr().add(i) as *const __m256i);
            let ti = _mm256_loadu_si256(tw.im.as_ptr().add(i) as *const __m256i);

            let (sre, sim, dre, dim) = sums(load_up(buf, a), load_down(buf, b));
            if post {
                // (tmp1, tmp2) = cplxMultDiv2(sim, dre, re, im)
                let tmp1 = sub(mul_div2(sim, tr), mul_div2(dre, ti));
                let tmp2 = add(mul_div2(sim, ti), mul_div2(dre, tr));
                store_up(buf, a, add(sre, tmp1), sub(dim, tmp2));
                store_down(buf, b, sub(sre, tmp1), neg(add(dim, tmp2)));
            } else {
                // (tmp2, tmp1) = cplxMultDiv2(dre, sim, re, im)
                let tmp2 = sub(mul_div2(dre, tr), mul_div2(sim, ti));
                let tmp1 = add(mul_div2(dre, ti), mul_div2(sim, tr));
                store_up(buf, a, add(sre, tmp1), sub(tmp2, dim));
                store_down(buf, b, sub(sre, tmp1), add(tmp2, dim));
            }

            let (sre, sim, dre, dim) = sums(load_down(buf, c), load_up(buf, d));
            if post {
                // (tmp1, tmp2) = cplxMultDiv2(dre, sim, re, im)
                let tmp1 = sub(mul_div2(dre, tr), mul_div2(sim, ti));
                let tmp2 = add(mul_div2(dre, ti), mul_div2(sim, tr));
                store_down(buf, c, sub(sre, tmp1), sub(dim, tmp2));
                store_up(buf, d, add(sre, tmp1), neg(add(dim, tmp2)));
            } else {
                // (tmp2, tmp1) = cplxMultDiv2(sim, dre, re, im)
                let tmp2 = sub(mul_div2(sim, tr), mul_div2(dre, ti));
                let tmp1 = add(mul_div2(sim, ti), mul_div2(dre, tr));
                store_down(buf, c, add(sre, tmp1), neg(add(dim, tmp2)));
                store_up(buf, d, sub(sre, tmp1), sub(dim, tmp2));
            }
        }
    }
}

/// NEON variants of the main loops of `fft_postsort()` and `fft_presort()`.
///
/// Iteration `i` of the C++ loops works on the complex values `a = 1 + i` and `b = n/2 - 1 - i`
/// (first butterfly) and `c = n/4 - 1 - i` and `d = n/4 + 1 + i` (second butterfly) with the
/// twiddle factor `tab[(i + 1) * step]`; four consecutive iterations are computed in parallel
/// (the descending indices with reversed lanes). The complex multiplications
/// `cplxMultDiv2()` compute `(x * re - y * im) >> 16` and `(x * im + y * re) >> 16` in 64 bits,
/// all other operations are the wrapping 32 bit operations of the C++ code.
#[cfg(target_arch = "aarch64")]
mod fast {
    use super::*;
    use core::arch::aarch64::*;
    use std::sync::OnceLock;

    /// Twiddle factors `tab[(i + 1) * step]` of the iterations `i` (and the negated
    /// imaginary parts).
    pub struct Twiddles {
        re: Box<[i32]>,
        im: Box<[i32]>,
        neg_im: Box<[i32]>,
    }

    pub fn twiddles(n: usize) -> Option<&'static Twiddles> {
        static TW: [OnceLock<Twiddles>; 4] = [const { OnceLock::new() }; 4];
        let idx = match n {
            64 => 0,
            128 => 1,
            256 => 2,
            512 => 3,
            _ => return None,
        };
        Some(TW[idx].get_or_init(|| {
            let (tab, step) = get_sine_tab(n as u32);
            let (tab, step) = (tab.unwrap(), step as usize);
            let t = |i: usize| tab[(i + 1) * step];
            let cnt = (n >> 3) - 1;
            Twiddles {
                re: (0..cnt).map(|i| t(i).re as i32).collect(),
                im: (0..cnt).map(|i| t(i).im as i32).collect(),
                neg_im: (0..cnt).map(|i| -(t(i).im as i32)).collect(),
            }
        }))
    }

    /// Reverses the order of the 4 lanes.
    #[inline(always)]
    unsafe fn rev(x: int32x4_t) -> int32x4_t {
        let r = vrev64q_s32(x);
        vextq_s32::<2>(r, r)
    }

    /// Loads the complex values `k, k + 1, k + 2, k + 3` as (re, im).
    #[inline(always)]
    unsafe fn load_up(p: *const i32, k: usize) -> (int32x4_t, int32x4_t) {
        let v = vld2q_s32(p.add(2 * k));
        (v.0, v.1)
    }

    /// Loads the complex values `k, k - 1, k - 2, k - 3` as (re, im).
    #[inline(always)]
    unsafe fn load_down(p: *const i32, k: usize) -> (int32x4_t, int32x4_t) {
        let v = vld2q_s32(p.add(2 * (k - 3)));
        (rev(v.0), rev(v.1))
    }

    #[inline(always)]
    unsafe fn store_up(p: *mut i32, k: usize, re: int32x4_t, im: int32x4_t) {
        vst2q_s32(p.add(2 * k), int32x4x2_t(re, im));
    }

    #[inline(always)]
    unsafe fn store_down(p: *mut i32, k: usize, re: int32x4_t, im: int32x4_t) {
        vst2q_s32(p.add(2 * (k - 3)), int32x4x2_t(rev(re), rev(im)));
    }

    /// `(a * p + b * q) >> 16` per lane in 64 bits, truncated to 32 bits.
    #[inline(always)]
    unsafe fn mac2(a: int32x4_t, p: int32x4_t, b: int32x4_t, q: int32x4_t) -> int32x4_t {
        let lo = vmlal_s32(
            vmull_s32(vget_low_s32(a), vget_low_s32(p)),
            vget_low_s32(b),
            vget_low_s32(q),
        );
        let hi = vmlal_high_s32(vmull_high_s32(a, p), b, q);
        vshrn_high_n_s64::<16>(vshrn_n_s64::<16>(lo), hi)
    }

    /// Sum and difference terms `(sre, sim, dre, dim)` of two complex values.
    #[inline(always)]
    unsafe fn sums(
        x: (int32x4_t, int32x4_t),
        y: (int32x4_t, int32x4_t),
    ) -> (int32x4_t, int32x4_t, int32x4_t, int32x4_t) {
        let (xr, xi) = (vshrq_n_s32::<1>(x.0), vshrq_n_s32::<1>(x.1));
        let (yr, yi) = (vshrq_n_s32::<1>(y.0), vshrq_n_s32::<1>(y.1));
        (vaddq_s32(xr, yr), vaddq_s32(xi, yi), vsubq_s32(xr, yr), vsubq_s32(xi, yi))
    }

    /// Iterations `0 .. 4 * chunks` of the main loop of `fft_postsort()` (`post`) or
    /// `fft_presort()`; `buf` holds `n` values.
    pub unsafe fn run(buf: *mut i32, n: usize, tw: &Twiddles, chunks: usize, post: bool) {
        for ch in 0..chunks {
            let i = 4 * ch;
            let (a, b, c, d) = (1 + i, n / 2 - 1 - i, n / 4 - 1 - i, n / 4 + 1 + i);
            let tr = vld1q_s32(tw.re.as_ptr().add(i));
            let ti = vld1q_s32(tw.im.as_ptr().add(i));
            let nti = vld1q_s32(tw.neg_im.as_ptr().add(i));

            let (sre, sim, dre, dim) = sums(load_up(buf, a), load_down(buf, b));
            let sre = vshrq_n_s32::<1>(sre);
            let dim = vshrq_n_s32::<1>(dim);
            if post {
                // (tmp1, tmp2) = cplxMultDiv2(sim, dre, re, im)
                let tmp1 = mac2(sim, tr, dre, nti);
                let tmp2 = mac2(sim, ti, dre, tr);
                store_up(buf, a, vaddq_s32(sre, tmp1), vsubq_s32(dim, tmp2));
                store_down(buf, b, vsubq_s32(sre, tmp1), vnegq_s32(vaddq_s32(dim, tmp2)));
            } else {
                // (tmp2, tmp1) = cplxMultDiv2(dre, sim, re, im)
                let tmp2 = mac2(dre, tr, sim, nti);
                let tmp1 = mac2(dre, ti, sim, tr);
                store_up(buf, a, vaddq_s32(sre, tmp1), vsubq_s32(tmp2, dim));
                store_down(buf, b, vsubq_s32(sre, tmp1), vaddq_s32(tmp2, dim));
            }

            let (sre, sim, dre, dim) = sums(load_down(buf, c), load_up(buf, d));
            let sre = vshrq_n_s32::<1>(sre);
            let dim = vshrq_n_s32::<1>(dim);
            if post {
                // (tmp1, tmp2) = cplxMultDiv2(dre, sim, re, im)
                let tmp1 = mac2(dre, tr, sim, nti);
                let tmp2 = mac2(dre, ti, sim, tr);
                store_down(buf, c, vsubq_s32(sre, tmp1), vsubq_s32(dim, tmp2));
                store_up(buf, d, vaddq_s32(sre, tmp1), vnegq_s32(vaddq_s32(dim, tmp2)));
            } else {
                // (tmp2, tmp1) = cplxMultDiv2(sim, dre, re, im)
                let tmp2 = mac2(sim, tr, dre, nti);
                let tmp1 = mac2(sim, ti, dre, tr);
                store_down(buf, c, vaddq_s32(sre, tmp1), vnegq_s32(vaddq_s32(dim, tmp2)));
                store_up(buf, d, vsubq_s32(sre, tmp1), vsubq_s32(dim, tmp2));
            }
        }
    }
}
