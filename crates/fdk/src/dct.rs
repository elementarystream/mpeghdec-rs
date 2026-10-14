// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/dct.h, libFDK/src/dct.cpp and libFDK/src/arm/dct_arm.cpp (the
// AArch64 build without NEON compiles the generic C variants).

//! Discrete cosine and sine transforms (DCT-II, DCT-III, DST-III, DCT-IV, DST-IV).
//!
//! All transforms work in place on `p_dat`; the transform length `L` is `p_dat.len()`. Only the
//! radix-2 lengths whose half length is supported by [`fft`](crate::fft) work, i.e. 128, 256,
//! 512 and 1024. The exponent `p_dat_e` is updated according to the transform scale.

use crate::common_fix::*;
use crate::fft::fft;
use crate::tools_rom::{stc, FixpStp, FixpWtp, SINE_TABLE_1024, WINDOW_SLOPES};

/// `dct_getTables(&twiddle, &sin_twiddle, &sin_step, length)`: returns
/// `(twiddle, sin_twiddle, sin_step)` for the given transform length (`None`/0 for unsupported
/// lengths).
pub fn dct_get_tables(
    length: i32,
) -> (Option<&'static [FixpWtp]>, Option<&'static [FixpStp]>, i32) {
    let ld2_length = DFRACT_BITS - 1 - f_normz(length) - 1;

    // Extract sort of "eigenvalue" (the 3 left most bits) of length.
    match length >> (ld2_length - 1) {
        0x4 => {
            // radix 2
            let twiddle = WINDOW_SLOPES[0][0][(ld2_length - 1) as usize];
            (twiddle, Some(&SINE_TABLE_1024), 1 << (10 - ld2_length))
        }
        _ => (None, None, 0),
    }
}

/// `dct_III(pDat, tmp, L, pDat_e)`: DCT-III of length `L = p_dat.len()`.
///
/// `tmp` is a work buffer of at least `L` values.
pub fn dct_iii(p_dat: &mut [FixpDbl], tmp: &mut [FixpDbl], p_dat_e: &mut i32) {
    let l = p_dat.len();
    let m = l >> 1;

    let (_, sin_twiddle, inc) = dct_get_tables(l as i32);
    let sin_twiddle = sin_twiddle.expect("dct_III: unsupported length");
    let inc = (inc >> 1) as usize;

    let mut p_tmp_0 = 2usize;
    let mut p_tmp_1 = (m - 1) * 2;
    let mut index = 4 * inc;

    // This loop performs multiplication for index i (i*inc)
    for i in 1..(m >> 1) {
        let (accu2, accu1) = cplx_mult_div2(
            p_dat[l - i],
            p_dat[i],
            sin_twiddle[i * inc].re,
            sin_twiddle[i * inc].im,
        );
        let w = sin_twiddle[(m - i) * inc];
        let (mut accu4, mut accu3) = cplx_mult_div2(p_dat[m + i], p_dat[m - i], w.re, w.im);
        accu3 >>= 1;
        accu4 >>= 1;

        let w = sin_twiddle[index];
        let (accu6, accu5) = if i < (m / 4) {
            cplx_mult_div2(accu3 - (accu1 >> 1), (accu2 >> 1) + accu4, w.re, w.im)
        } else {
            let (accu6, accu5) =
                cplx_mult_div2((accu2 >> 1) + accu4, accu3 - (accu1 >> 1), w.re, w.im);
            (accu6.wrapping_neg(), accu5)
        };
        let xr = (accu1 >> 1) + accu3;
        tmp[p_tmp_0] = (xr >> 1) - accu5;
        tmp[p_tmp_1] = (xr >> 1) + accu5;

        let xr = (accu2 >> 1) - accu4;
        tmp[p_tmp_0 + 1] = (xr >> 1) - accu6;
        tmp[p_tmp_1 + 1] = ((xr >> 1) + accu6).wrapping_neg();

        if i < (m / 4) {
            index += 4 * inc;
        } else {
            index -= 4 * inc;
        }
        p_tmp_0 += 2;
        p_tmp_1 -= 2;
    }

    let xr = f_mult_div2(p_dat[m], sin_twiddle[m * inc].re); // cos((PI/(2*L))*M)
    tmp[0] = ((p_dat[0] >> 1) + xr) >> 1;
    tmp[1] = ((p_dat[0] >> 1) - xr) >> 1;

    let w = sin_twiddle[m * inc / 2];
    let (accu2, accu1) = cplx_mult_div2(p_dat[l - (m / 2)], p_dat[m / 2], w.re, w.im);
    tmp[m] = accu1 >> 1;
    tmp[m + 1] = accu2 >> 1;

    // dit_fft expects 1 bit scaled input values
    fft(m as i32, tmp, p_dat_e);

    let mut p_tmp_1 = l;
    let mut t = 0usize;
    let mut d = 0usize;
    for _ in 0..(m >> 1) {
        let tmp1 = tmp[t];
        let tmp2 = tmp[t + 1];
        t += 2;
        let tmp3 = tmp[p_tmp_1 - 1];
        let tmp4 = tmp[p_tmp_1 - 2];
        p_tmp_1 -= 2;
        p_dat[d] = tmp1;
        p_dat[d + 1] = tmp3;
        p_dat[d + 2] = tmp2;
        p_dat[d + 3] = tmp4;
        d += 4;
    }

    *p_dat_e += 2;
}

/// `dst_III(pDat, tmp, L, pDat_e)`: DST-III of length `L = p_dat.len()`.
///
/// `tmp` is a work buffer of at least `L` values.
pub fn dst_iii(p_dat: &mut [FixpDbl], tmp: &mut [FixpDbl], p_dat_e: &mut i32) {
    let l = p_dat.len();

    // note: DCT III is reused here, direct DST III implementation might be more efficient

    // mirror input
    p_dat.reverse();

    // DCT-III
    dct_iii(p_dat, tmp, p_dat_e);

    // flip signs at odd indices
    for i in (1..l).step_by(2) {
        p_dat[i] = p_dat[i].wrapping_neg();
    }
}

/// `dct_II(pDat, tmp, L, pDat_e)`: DCT-II of length `L = p_dat.len()`.
///
/// `tmp` is a work buffer of at least `L` values.
pub fn dct_ii(p_dat: &mut [FixpDbl], tmp: &mut [FixpDbl], p_dat_e: &mut i32) {
    let l = p_dat.len();
    let m = l >> 1;

    let (_, sin_twiddle, inc) = dct_get_tables(l as i32);
    let sin_twiddle = sin_twiddle.expect("dct_II: unsupported length");
    let inc = (inc >> 1) as usize;

    for i in 0..m {
        tmp[i] = p_dat[2 * i] >> 2;
        tmp[l - 1 - i] = p_dat[2 * i + 1] >> 2;
    }

    fft(m as i32, tmp, p_dat_e);
    *p_dat_e += 2;

    let w = sin_twiddle[(m / 2) * inc];
    let (accu1, accu2) = cplx_mult(tmp[m], tmp[m + 1], w.re, w.im);
    p_dat[l - (m / 2)] = accu2;
    p_dat[m / 2] = accu1;

    p_dat[0] = tmp[0] + tmp[1];
    p_dat[m] = f_mult(tmp[0] - tmp[1], sin_twiddle[m * inc].re);

    let mut index = 4 * inc;
    let mut p_tmp_0 = 2usize;
    let mut p_tmp_1 = (m - 1) * 2;
    for i in 1..(m >> 1) {
        let a1 = (tmp[p_tmp_0 + 1] >> 1) + (tmp[p_tmp_1 + 1] >> 1);
        let a2 = (tmp[p_tmp_1] >> 1) - (tmp[p_tmp_0] >> 1);

        let w = sin_twiddle[index];
        let (accu1, accu2) = if i < (m / 4) {
            cplx_mult(a2, a1, w.re, w.im)
        } else {
            let (accu1, accu2) = cplx_mult(a1, a2, w.re, w.im);
            (accu1.wrapping_neg(), accu2)
        };

        let a1 = (tmp[p_tmp_0] >> 1) + (tmp[p_tmp_1] >> 1);
        let a2 = (tmp[p_tmp_0 + 1] >> 1) - (tmp[p_tmp_1 + 1] >> 1);

        let w = sin_twiddle[i * inc];
        let (accu3, accu4) = cplx_mult(accu1 + a2, a1 + accu2, w.re, w.im);
        p_dat[l - i] = accu3.wrapping_neg();
        p_dat[i] = accu4;

        let w = sin_twiddle[(m - i) * inc];
        let (accu3, accu4) = cplx_mult(accu1 - a2, a1 - accu2, w.re, w.im);
        p_dat[m + i] = accu3.wrapping_neg();
        p_dat[m - i] = accu4;

        if i < (m / 4) {
            index += 4 * inc;
        } else {
            index -= 4 * inc;
        }
        p_tmp_0 += 2;
        p_tmp_1 -= 2;
    }
}

/// `dct_IV(pDat, L, pDat_e)`: DCT-IV of length `L = p_dat.len()`.
pub fn dct_iv(p_dat: &mut [FixpDbl], p_dat_e: &mut i32) {
    let l = p_dat.len();
    let m = l >> 1;

    let (twiddle, sin_twiddle, sin_step) = dct_get_tables(l as i32);
    let twiddle = twiddle.expect("dct_IV: unsupported length");
    let sin_twiddle = sin_twiddle.expect("dct_IV: unsupported length");
    let sin_step = sin_step as usize;

    // The NEON code reproduces the AArch64 arithmetic of the reference.
    #[cfg(target_arch = "aarch64")]
    if !crate::ARCH_X86 && m >= 16 && m.is_power_of_two() {
        fast::dct_iv_pre(p_dat, twiddle);
        fft(m as i32, p_dat, p_dat_e);
        fast::dct_iv_post(p_dat, sin_twiddle, sin_step);
        // Add twiddeling scale.
        *p_dat_e += 2;
        return;
    }
    // The AVX2 code reproduces the x86 arithmetic of the reference.
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    if crate::ARCH_X86 && m >= 16 && m.is_power_of_two() && crate::simd::avx2() {
        // SAFETY: AVX2 is available.
        unsafe { fast_x86::dct_iv_pre(p_dat, twiddle) };
        fft(m as i32, p_dat, p_dat_e);
        // SAFETY: AVX2 is available.
        unsafe { fast_x86::dct_iv_post(p_dat, sin_twiddle, sin_step) };
        // Add twiddeling scale.
        *p_dat_e += 2;
        return;
    }

    {
        let mut p_dat_0 = 0usize;
        let mut p_dat_1 = l - 2;
        let mut i = 0usize;

        while i + 1 < m {
            let accu1 = p_dat[p_dat_1 + 1];
            let accu2 = p_dat[p_dat_0];
            let accu3 = p_dat[p_dat_0 + 1];
            let accu4 = p_dat[p_dat_1];

            let (accu1, accu2) = cplx_mult_div2(accu1, accu2, twiddle[i].re, twiddle[i].im);
            let (accu3, accu4) = cplx_mult_div2(accu4, accu3, twiddle[i + 1].re, twiddle[i + 1].im);

            p_dat[p_dat_0] = accu2 >> 1;
            p_dat[p_dat_0 + 1] = accu1 >> 1;
            p_dat[p_dat_1] = accu4 >> 1;
            p_dat[p_dat_1 + 1] = -(accu3 >> 1);

            i += 2;
            p_dat_0 += 2;
            p_dat_1 -= 2;
        }
        if m & 1 != 0 {
            let accu1 = p_dat[p_dat_1 + 1];
            let accu2 = p_dat[p_dat_0];

            let (accu1, accu2) = cplx_mult_div2(accu1, accu2, twiddle[i].re, twiddle[i].im);

            p_dat[p_dat_0] = accu2 >> 1;
            p_dat[p_dat_0 + 1] = accu1 >> 1;
        }
    }

    fft(m as i32, p_dat, p_dat_e);

    {
        let mut p_dat_0 = 0usize;
        let mut p_dat_1 = l - 2;

        // Sin and Cos values are 0.0f and 1.0f
        let mut accu1 = p_dat[p_dat_1];
        let mut accu2 = p_dat[p_dat_1 + 1];

        p_dat[p_dat_1 + 1] = p_dat[p_dat_0 + 1].wrapping_neg();

        let mut idx = sin_step;
        for _ in 1..((m + 1) >> 1) {
            let twd = sin_twiddle[idx];

            let (accu3, accu4) = cplx_mult(accu1, accu2, twd.re, twd.im);
            p_dat[p_dat_0 + 1] = accu3;
            p_dat[p_dat_1] = accu4;

            p_dat_0 += 2;
            p_dat_1 -= 2;

            let (accu3, accu4) = cplx_mult(p_dat[p_dat_0 + 1], p_dat[p_dat_0], twd.re, twd.im);

            accu1 = p_dat[p_dat_1];
            accu2 = p_dat[p_dat_1 + 1];

            p_dat[p_dat_1 + 1] = accu3.wrapping_neg();
            p_dat[p_dat_0] = accu4;
            idx += sin_step;
        }

        if (m & 1) == 0 {
            // Last Sin and Cos value pair are the same
            accu1 = f_mult(accu1, stc(0x5a82799a));
            accu2 = f_mult(accu2, stc(0x5a82799a));

            p_dat[p_dat_1] = accu1 + accu2;
            p_dat[p_dat_0 + 1] = accu1 - accu2;
        }
    }

    // Add twiddeling scale.
    *p_dat_e += 2;
}

/// `dst_IV(pDat, L, pDat_e)`: DST-IV of length `L = p_dat.len()`.
pub fn dst_iv(p_dat: &mut [FixpDbl], p_dat_e: &mut i32) {
    let l = p_dat.len();
    let m = l >> 1;

    let (twiddle, sin_twiddle, sin_step) = dct_get_tables(l as i32);
    let twiddle = twiddle.expect("dst_IV: unsupported length");
    let sin_twiddle = sin_twiddle.expect("dst_IV: unsupported length");
    let sin_step = sin_step as usize;

    {
        let mut p_dat_0 = 0usize;
        let mut p_dat_1 = l - 2;
        let mut i = 0usize;

        while i + 1 < m {
            let accu1 = p_dat[p_dat_1 + 1] >> 1;
            let accu2 = -(p_dat[p_dat_0] >> 1);
            let accu3 = p_dat[p_dat_0 + 1] >> 1;
            let accu4 = -(p_dat[p_dat_1] >> 1);

            let (accu1, accu2) = cplx_mult_div2(accu1, accu2, twiddle[i].re, twiddle[i].im);
            let (accu3, accu4) = cplx_mult_div2(accu4, accu3, twiddle[i + 1].re, twiddle[i + 1].im);

            p_dat[p_dat_0] = accu2;
            p_dat[p_dat_0 + 1] = accu1;
            p_dat[p_dat_1] = accu4;
            p_dat[p_dat_1 + 1] = accu3.wrapping_neg();

            i += 2;
            p_dat_0 += 2;
            p_dat_1 -= 2;
        }
        if m & 1 != 0 {
            let accu1 = p_dat[p_dat_1 + 1];
            let accu2 = p_dat[p_dat_0].wrapping_neg();

            let (accu1, accu2) = cplx_mult_div2(accu1, accu2, twiddle[i].re, twiddle[i].im);

            p_dat[p_dat_0] = accu2 >> 1;
            p_dat[p_dat_0 + 1] = accu1 >> 1;
        }
    }

    fft(m as i32, p_dat, p_dat_e);

    {
        let mut p_dat_0 = 0usize;
        let mut p_dat_1 = l - 2;

        // Sin and Cos values are 0.0f and 1.0f
        let mut accu1 = p_dat[p_dat_1];
        let mut accu2 = p_dat[p_dat_1 + 1];

        p_dat[p_dat_1 + 1] = p_dat[p_dat_0].wrapping_neg();
        p_dat[p_dat_0] = p_dat[p_dat_0 + 1];

        let mut idx = sin_step;
        for _ in 1..((m + 1) >> 1) {
            let twd = sin_twiddle[idx];

            let (accu3, accu4) = cplx_mult(accu1, accu2, twd.re, twd.im);
            p_dat[p_dat_1] = accu3.wrapping_neg();
            p_dat[p_dat_0 + 1] = accu4.wrapping_neg();

            p_dat_0 += 2;
            p_dat_1 -= 2;

            let (accu3, accu4) = cplx_mult(p_dat[p_dat_0 + 1], p_dat[p_dat_0], twd.re, twd.im);

            accu1 = p_dat[p_dat_1];
            accu2 = p_dat[p_dat_1 + 1];

            p_dat[p_dat_0] = accu3;
            p_dat[p_dat_1 + 1] = accu4.wrapping_neg();
            idx += sin_step;
        }

        if (m & 1) == 0 {
            // Last Sin and Cos value pair are the same
            accu1 = f_mult(accu1, stc(0x5a82799a));
            accu2 = f_mult(accu2, stc(0x5a82799a));

            p_dat[p_dat_0 + 1] = accu1.wrapping_neg() - accu2;
            p_dat[p_dat_1] = accu2 - accu1;
        }
    }

    *p_dat_e += 2;
}

/// NEON variants of the pre and post twiddle loops of [`dct_iv`] (lengths `L = 2m` with
/// `m >= 16` a power of two).
///
/// Every iteration of the pre twiddle loop reads and writes only the complex values `t` and
/// `m - 1 - t`. The post twiddle loop is written per complex pair `j` (forward value `j`,
/// backward value `m - 1 - j`): the C++ loop computes the real part of the forward value and the
/// imaginary part of the backward value in iteration `t = j` (twiddle `w_j`) and the other two
/// parts in iteration `t = j + 1` (twiddle `w_{j+1}`), always from the values of the FFT output
/// (each value is read before it is overwritten). So the pairs are independent; 4 of them are
/// computed in parallel with exactly the arithmetic of `cplxMultDiv2()`/`cplxMult()` (64-bit
/// products, truncating shifts) and wrapping negations.
#[cfg(target_arch = "aarch64")]
mod fast {
    use super::*;
    use core::arch::aarch64::*;
    use std::sync::OnceLock;

    /// `(a * p + b * q) >> N` per lane in 64 bits, truncated to 32 bits.
    #[inline(always)]
    unsafe fn mac2<const N: i32>(
        a: int32x4_t,
        p: int32x4_t,
        b: int32x4_t,
        q: int32x4_t,
    ) -> int32x4_t {
        let lo = vmlal_s32(
            vmull_s32(vget_low_s32(a), vget_low_s32(p)),
            vget_low_s32(b),
            vget_low_s32(q),
        );
        let hi = vmlal_high_s32(vmull_high_s32(a, p), b, q);
        vshrn_high_n_s64::<N>(vshrn_n_s64::<N>(lo), hi)
    }

    /// Reverses the order of the 4 lanes.
    #[inline(always)]
    unsafe fn rev(x: int32x4_t) -> int32x4_t {
        let r = vrev64q_s32(x);
        vextq_s32::<2>(r, r)
    }

    /// Loads the complex values `k, k - 1, k - 2, k - 3` as (re, im).
    #[inline(always)]
    unsafe fn load_down(p: *const i32, k: usize) -> (int32x4_t, int32x4_t) {
        let v = vld2q_s32(p.add(2 * (k - 3)));
        (rev(v.0), rev(v.1))
    }

    #[inline(always)]
    unsafe fn store_down(p: *mut i32, k: usize, re: int32x4_t, im: int32x4_t) {
        vst2q_s32(p.add(2 * (k - 3)), int32x4x2_t(rev(re), rev(im)));
    }

    /// Pre twiddling: iteration `t` of the C++ loop for 4 consecutive `t`.
    pub fn dct_iv_pre(d: &mut [FixpDbl], twiddle: &[FixpWtp]) {
        let l = d.len();
        let m = l / 2;
        assert!(m >= 16 && m % 8 == 0 && twiddle.len() >= m);
        let dp = d.as_mut_ptr();
        let tp = twiddle.as_ptr() as *const i16;
        // SAFETY: t + 3 < m / 2, so the forward values 2t..2t+7 and the backward complex values
        // m - 1 - t - 3 ..= m - 1 - t are within the l values of d; the twiddles 2t..2t+7
        // (FixpWtp is repr(C) (re, im) of i16) are within the m entries.
        unsafe {
            for c in 0..m / 8 {
                let t = 4 * c;
                let f = vld2q_s32(dp.add(2 * t));
                let (br, bi) = load_down(dp, m - 1 - t);
                let w = vld4_s16(tp.add(4 * t));
                let (tr0, ti0) = (vmovl_s16(w.0), vmovl_s16(w.1));
                let (tr1, ti1) = (vmovl_s16(w.2), vmovl_s16(w.3));
                // (accu1, accu2) = cplxMultDiv2(bi, fr, tw[2t]);
                let accu1 = mac2::<16>(bi, tr0, f.0, vnegq_s32(ti0));
                let accu2 = mac2::<16>(bi, ti0, f.0, tr0);
                // (accu3, accu4) = cplxMultDiv2(br, fi, tw[2t + 1]);
                let accu3 = mac2::<16>(br, tr1, f.1, vnegq_s32(ti1));
                let accu4 = mac2::<16>(br, ti1, f.1, tr1);
                vst2q_s32(
                    dp.add(2 * t),
                    int32x4x2_t(vshrq_n_s32::<1>(accu2), vshrq_n_s32::<1>(accu1)),
                );
                store_down(
                    dp,
                    m - 1 - t,
                    vshrq_n_s32::<1>(accu4),
                    vnegq_s32(vshrq_n_s32::<1>(accu3)),
                );
            }
        }
    }

    /// Post twiddle factors `w_j = sin_twiddle[j * sin_step]` (`j < m / 2`) as (re, im, -im).
    struct PostTwiddles {
        re: Box<[i32]>,
        im: Box<[i32]>,
        neg_im: Box<[i32]>,
    }

    fn post_twiddles(
        m: usize,
        sin_twiddle: &'static [FixpStp],
        sin_step: usize,
    ) -> &'static PostTwiddles {
        static TW: [OnceLock<PostTwiddles>; 16] = [const { OnceLock::new() }; 16];
        let t = TW[m.trailing_zeros() as usize].get_or_init(|| {
            let w = |j: usize| sin_twiddle[j * sin_step];
            PostTwiddles {
                re: (0..m / 2).map(|j| w(j).re as i32).collect(),
                im: (0..m / 2).map(|j| w(j).im as i32).collect(),
                neg_im: (0..m / 2).map(|j| -(w(j).im as i32)).collect(),
            }
        });
        // The table depends on the length only (dct_getTables()).
        debug_assert!(t.re[1] == sin_twiddle[sin_step].re as i32);
        t
    }

    pub fn dct_iv_post(d: &mut [FixpDbl], sin_twiddle: &'static [FixpStp], sin_step: usize) {
        let l = d.len();
        let m = l / 2;
        assert!(m >= 16 && m.is_power_of_two());
        let tw = post_twiddles(m, sin_twiddle, sin_step);

        // j = 1 .. m/2 - 2
        let last = m / 2 - 1;
        let chunks = (last - 1) / 4;
        let dp = d.as_mut_ptr();
        // SAFETY: for j + 3 <= m/2 - 2 the forward values 2j .. 2j + 7 and the backward complex
        // values m - 1 - j - 3 ..= m - 1 - j are within d; the twiddle tables hold m/2 values and
        // j + 4 <= m/2 - 1.
        unsafe {
            for c in 0..chunks {
                let j = 1 + 4 * c;
                let f = vld2q_s32(dp.add(2 * j));
                let (p, q) = load_down(dp, m - 1 - j);
                let (wr, wi, nwi) = (
                    vld1q_s32(tw.re.as_ptr().add(j)),
                    vld1q_s32(tw.im.as_ptr().add(j)),
                    vld1q_s32(tw.neg_im.as_ptr().add(j)),
                );
                let (wr1, wi1, nwi1) = (
                    vld1q_s32(tw.re.as_ptr().add(j + 1)),
                    vld1q_s32(tw.im.as_ptr().add(j + 1)),
                    vld1q_s32(tw.neg_im.as_ptr().add(j + 1)),
                );
                // (b3, b4) = cplxMult(fi, fr, w_j)
                let b3 = mac2::<15>(f.1, wr, f.0, nwi);
                let b4 = mac2::<15>(f.1, wi, f.0, wr);
                // (a3, a4) = cplxMult(p, q, w_{j+1})
                let a3 = mac2::<15>(p, wr1, q, nwi1);
                let a4 = mac2::<15>(p, wi1, q, wr1);
                vst2q_s32(dp.add(2 * j), int32x4x2_t(b4, a3));
                store_down(dp, m - 1 - j, a4, vnegq_s32(b3));
            }
        }
        dct_iv_post_rest(d, sin_twiddle, sin_step, 1 + 4 * chunks);
    }
}

/// The post twiddling of [`dct_iv`] (lengths `L = 2m` with `m >= 16` a power of two) per
/// complex pair `j` (see the NEON variant `fast`) except for the pairs `1 .. first`, which the
/// vectorized variants compute.
#[cfg(any(target_arch = "aarch64", target_arch = "x86", target_arch = "x86_64"))]
fn dct_iv_post_rest(d: &mut [FixpDbl], sin_twiddle: &[FixpStp], sin_step: usize, first: usize) {
    let l = d.len();
    let m = l / 2;
    let w = |j: usize| sin_twiddle[j * sin_step];

    // j = 0: the real part of the first value is not changed; Sin and Cos values are 0.0f and
    // 1.0f for the imaginary part of the last value.
    {
        let d1 = d[1];
        let (a3, a4) = cplx_mult(d[l - 2], d[l - 1], w(1).re, w(1).im);
        d[1] = a3;
        d[l - 2] = a4;
        d[l - 1] = d1.wrapping_neg();
    }

    // j = first .. m/2 - 2
    let last = m / 2 - 1;
    for j in first..last {
        let (wj, wj1) = (w(j), w(j + 1));
        let (fr, fi) = (d[2 * j], d[2 * j + 1]);
        let (p, q) = (d[l - 2 - 2 * j], d[l - 1 - 2 * j]);
        let (b3, b4) = cplx_mult(fi, fr, wj.re, wj.im);
        let (a3, a4) = cplx_mult(p, q, wj1.re, wj1.im);
        d[2 * j] = b4;
        d[2 * j + 1] = a3;
        d[l - 2 - 2 * j] = a4;
        d[l - 1 - 2 * j] = b3.wrapping_neg();
    }

    // j = m/2 - 1: Last Sin and Cos value pair are the same
    {
        let j = last;
        let (fr, fi) = (d[2 * j], d[2 * j + 1]);
        let (b3, b4) = cplx_mult(fi, fr, w(j).re, w(j).im);
        let accu1 = f_mult(d[m], stc(0x5a82799a));
        let accu2 = f_mult(d[m + 1], stc(0x5a82799a));
        d[2 * j] = b4;
        d[m + 1] = b3.wrapping_neg();
        d[m] = accu1 + accu2;
        d[2 * j + 1] = accu1 - accu2;
    }
}

/// AVX2 variants of the pre and post twiddle loops of [`dct_iv`] with the arithmetic of the x86
/// build (lengths `L = 2m` with `m >= 16` a power of two): 8 iterations of the pre twiddle loop
/// and 8 pairs of the post twiddle loop (see `fast`) at a time. In the x86 build,
/// `cplxMultDiv2()` and `cplxMult()` subtract or add the individually truncated products
/// `(x * w) >> 16` (`cplxMult()`: shifted left by one).
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
mod fast_x86 {
    use super::*;
    use crate::simd::x86::*;
    use std::sync::OnceLock;

    /// Twiddle factors per complex value: pre twiddling `twiddle[2t]` (`re0`, `im0`) and
    /// `twiddle[2t + 1]` (`re1`, `im1`) for `t < m / 2`, post twiddling
    /// `sin_twiddle[j * sin_step]` (`post_re`, `post_im`) for `j < m / 2`.
    struct Twiddles {
        re0: Box<[i32]>,
        im0: Box<[i32]>,
        re1: Box<[i32]>,
        im1: Box<[i32]>,
        post_re: Box<[i32]>,
        post_im: Box<[i32]>,
    }

    /// The tables of the length `2m` (`dct_getTables()` depends on the length only).
    fn twiddles(m: usize) -> &'static Twiddles {
        static TW: [OnceLock<Twiddles>; 16] = [const { OnceLock::new() }; 16];
        TW[m.trailing_zeros() as usize].get_or_init(|| {
            let (twiddle, sin_twiddle, sin_step) = dct_get_tables(2 * m as i32);
            let (twiddle, sin_twiddle) = (twiddle.unwrap(), sin_twiddle.unwrap());
            let w = |j: usize| sin_twiddle[j * sin_step as usize];
            Twiddles {
                re0: (0..m / 2).map(|t| twiddle[2 * t].re as i32).collect(),
                im0: (0..m / 2).map(|t| twiddle[2 * t].im as i32).collect(),
                re1: (0..m / 2).map(|t| twiddle[2 * t + 1].re as i32).collect(),
                im1: (0..m / 2).map(|t| twiddle[2 * t + 1].im as i32).collect(),
                post_re: (0..m / 2).map(|j| w(j).re as i32).collect(),
                post_im: (0..m / 2).map(|j| w(j).im as i32).collect(),
            }
        })
    }

    /// `(x * w) >> 15` truncated like `fixmul_DS()` of the x86 build: `((x * w) >> 16) << 1`.
    #[inline(always)]
    unsafe fn mul(x: __m256i, w: __m256i) -> __m256i {
        _mm256_slli_epi32::<1>(mul_div2(x, w))
    }

    /// Pre twiddling: iteration `t` of the C++ loop for 8 consecutive `t`.
    ///
    /// # Safety
    ///
    /// AVX2 must be available.
    #[target_feature(enable = "avx2")]
    pub unsafe fn dct_iv_pre(d: &mut [FixpDbl], twiddle: &[FixpWtp]) {
        let l = d.len();
        let m = l / 2;
        assert!(m >= 16 && m.is_power_of_two());
        let tw = twiddles(m);
        debug_assert!(tw.re1[1] == twiddle[3].re as i32);
        let dp = d.as_mut_ptr();
        // SAFETY: t + 7 < m / 2, so the forward values t .. t + 7 and the backward values
        // m - 1 - t - 7 ..= m - 1 - t are within the m complex values of d; the tables hold
        // m / 2 values.
        unsafe {
            for t in (0..m / 2).step_by(8) {
                let (fr, fi) = load_up(dp, t);
                let (br, bi) = load_down(dp, m - 1 - t);
                let (tr0, ti0) = (load(tw.re0.as_ptr().add(t)), load(tw.im0.as_ptr().add(t)));
                let (tr1, ti1) = (load(tw.re1.as_ptr().add(t)), load(tw.im1.as_ptr().add(t)));
                // (accu1, accu2) = cplxMultDiv2(bi, fr, tw[2t]);
                let accu1 = sub(mul_div2(bi, tr0), mul_div2(fr, ti0));
                let accu2 = add(mul_div2(bi, ti0), mul_div2(fr, tr0));
                // (accu3, accu4) = cplxMultDiv2(br, fi, tw[2t + 1]);
                let accu3 = sub(mul_div2(br, tr1), mul_div2(fi, ti1));
                let accu4 = add(mul_div2(br, ti1), mul_div2(fi, tr1));
                store_up(dp, t, _mm256_srai_epi32::<1>(accu2), _mm256_srai_epi32::<1>(accu1));
                store_down(
                    dp,
                    m - 1 - t,
                    _mm256_srai_epi32::<1>(accu4),
                    neg(_mm256_srai_epi32::<1>(accu3)),
                );
            }
        }
    }

    /// Post twiddling: the pairs `1 .. m/2 - 2` 8 at a time, the others with
    /// `dct_iv_post_rest()`.
    ///
    /// # Safety
    ///
    /// AVX2 must be available.
    #[target_feature(enable = "avx2")]
    pub unsafe fn dct_iv_post(d: &mut [FixpDbl], sin_twiddle: &[FixpStp], sin_step: usize) {
        let l = d.len();
        let m = l / 2;
        assert!(m >= 16 && m.is_power_of_two());
        let tw = twiddles(m);
        debug_assert!(tw.post_re[1] == sin_twiddle[sin_step].re as i32);
        let last = m / 2 - 1;
        let chunks = (last - 1) / 8;
        let dp = d.as_mut_ptr();
        // SAFETY: for j + 7 <= m/2 - 2 the forward values j .. j + 7 and the backward values
        // m - 1 - j - 7 ..= m - 1 - j are within d; the tables hold m/2 values and
        // j + 8 <= m/2 - 1.
        unsafe {
            for c in 0..chunks {
                let j = 1 + 8 * c;
                let (fr, fi) = load_up(dp, j);
                let (p, q) = load_down(dp, m - 1 - j);
                let (wr, wi) = (load(tw.post_re.as_ptr().add(j)), load(tw.post_im.as_ptr().add(j)));
                let (wr1, wi1) =
                    (load(tw.post_re.as_ptr().add(j + 1)), load(tw.post_im.as_ptr().add(j + 1)));
                // (b3, b4) = cplxMult(fi, fr, w_j)
                let b3 = sub(mul(fi, wr), mul(fr, wi));
                let b4 = add(mul(fi, wi), mul(fr, wr));
                // (a3, a4) = cplxMult(p, q, w_{j+1})
                let a3 = sub(mul(p, wr1), mul(q, wi1));
                let a4 = add(mul(p, wi1), mul(q, wr1));
                store_up(dp, j, b4, a3);
                store_down(dp, m - 1 - j, a4, neg(b3));
            }
        }
        dct_iv_post_rest(d, sin_twiddle, sin_step, 1 + 8 * chunks);
    }
}
