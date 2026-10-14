// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/fft_rad2.h, libFDK/src/fft_rad2.cpp and
// libFDK/src/arm/fft_rad2_arm.cpp (the AArch64 build compiles the generic C variant of the ARM
// file: the first two stages are combined into one radix-4 pass).

//! Radix-2 decimation in time FFT.

use crate::common_fix::*;
use crate::scramble::scramble;
#[cfg(any(target_arch = "aarch64", target_arch = "x86", target_arch = "x86_64"))]
use crate::tools_rom::SINE_TABLE_512;
use crate::tools_rom::{stc, FixpStp};

/// `dit_fft(x, ldn, trigdata, trigDataSize)`: in-place complex valued FFT of length `2^ldn`.
///
/// `x` holds the interleaved real/imaginary pairs; the input data must have at least 1 bit
/// headroom. `trigdata` is a sine table with at least `2^ldn / 2` values, `trig_data_size` its
/// length. The output is scaled by `2^-ldn`.
///
/// The butterflies use wrapping arithmetic: without sufficient headroom the C++ code overflows
/// (e.g. in the STFT synthesis of full scale spectra) and wraps around.
///
/// On AArch64 (NEON) and on x86 with AVX2 (x86 variant, see `crate::simd`), the transforms with
/// `SineTable512` (all lengths used by the decoder) run a vectorized implementation that computes
/// every butterfly with exactly the same arithmetic.
pub fn dit_fft(x: &mut [FixpDbl], ldn: i32, trigdata: &[FixpStp], trig_data_size: i32) {
    // The NEON code reproduces the AArch64 arithmetic of the reference.
    #[cfg(target_arch = "aarch64")]
    if !crate::ARCH_X86
        && (4..=fast::MAX_LDN).contains(&ldn)
        && trig_data_size == 512
        && core::ptr::eq(trigdata.as_ptr(), SINE_TABLE_512.as_ptr())
    {
        fast::dit_fft(x, ldn as u32);
        return;
    }
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    if crate::ARCH_X86
        && (3..=fast_x86::MAX_LDN).contains(&ldn)
        && trig_data_size == 512
        && core::ptr::eq(trigdata.as_ptr(), SINE_TABLE_512.as_ptr())
        && crate::simd::avx2()
    {
        // SAFETY: AVX2 is available.
        unsafe { fast_x86::dit_fft(x, ldn as u32) };
        return;
    }
    dit_fft_generic(x, ldn, trigdata, trig_data_size);
}

/// Bit reversal permutation and the first two stages (combined radix-4 butterflies) of
/// [`dit_fft`] for `n` complex values.
#[inline]
fn radix4_first_stages(x: &mut [FixpDbl], n: usize) {
    scramble(x, n as i32);

    // Combined radix-4 butterflies of the first two stages.
    let mut i = 0usize;
    while i < 2 * n {
        let mut a00 = x[i].wrapping_add(x[i + 2]) >> 1;
        let mut a10 = x[i + 4].wrapping_add(x[i + 6]) >> 1;
        let mut a20 = x[i + 1].wrapping_add(x[i + 3]) >> 1;
        let mut a30 = x[i + 5].wrapping_add(x[i + 7]) >> 1;

        x[i] = a00.wrapping_add(a10);
        x[i + 4] = a00.wrapping_sub(a10);
        x[i + 1] = a20.wrapping_add(a30);
        x[i + 5] = a20.wrapping_sub(a30);

        a00 = a00.wrapping_sub(x[i + 2]);
        a10 = a10.wrapping_sub(x[i + 6]);
        a20 = a20.wrapping_sub(x[i + 3]);
        a30 = a30.wrapping_sub(x[i + 7]);

        x[i + 2] = a00.wrapping_add(a30);
        x[i + 6] = a00.wrapping_sub(a30);
        x[i + 3] = a20.wrapping_sub(a10);
        x[i + 7] = a20.wrapping_add(a10);
        i += 8;
    }
}

/// Portable implementation of [`dit_fft`] (a straight port of the C++ code).
pub fn dit_fft_generic(x: &mut [FixpDbl], ldn: i32, trigdata: &[FixpStp], trig_data_size: i32) {
    let n: i32 = 1 << ldn;

    radix4_first_stages(x, n as usize);

    let w_pi_fourth: FixpSgl = stc(0x5a82799a);
    let mut mh: usize = 1 << 1;
    let mut ldm = ldn - 2;
    let mut trigstep = trig_data_size as usize;
    let n = n as usize;

    loop {
        let mut p_trig_data = 0usize;
        mh <<= 1;
        trigstep >>= 1;

        // j = 0: trivial twiddle factor
        {
            let mut xt1 = 0usize;
            let mut r = n;
            loop {
                let mut xt2 = xt1 + (mh << 1);

                let vi = x[xt2 + 1] >> 1;
                let vr = x[xt2] >> 1;
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_add(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_sub(vi);

                xt1 += mh;
                xt2 += mh;

                let vr = x[xt2 + 1] >> 1;
                let vi = x[xt2] >> 1;
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_sub(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_add(vi);

                xt1 = xt2 + mh;
                r -= mh << 1;
                if r == 0 {
                    break;
                }
            }
        }

        let mut j = 4usize;
        while j < mh {
            let mut xt1 = j >> 1;
            let mut r = n;
            p_trig_data += trigstep;
            let cs = trigdata[p_trig_data];

            loop {
                let mut xt2 = xt1 + (mh << 1);

                let (vi, vr) = cplx_mult_div2(x[xt2 + 1], x[xt2], cs.re, cs.im);
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_add(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_sub(vi);

                xt1 += mh;
                xt2 += mh;

                let (vr, vi) = cplx_mult_div2(x[xt2 + 1], x[xt2], cs.re, cs.im);
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_sub(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_add(vi);

                // Same twiddle factor
                xt1 -= j;
                xt2 = xt1 + (mh << 1);

                let (vi, vr) = cplx_mult_div2(x[xt2], x[xt2 + 1], cs.re, cs.im);
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_sub(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_add(vi);

                xt1 += mh;
                xt2 += mh;

                let (vr, vi) = cplx_mult_div2(x[xt2], x[xt2 + 1], cs.re, cs.im);
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_sub(vr);
                x[xt1 + 1] = ui.wrapping_sub(vi);
                x[xt2] = ur.wrapping_add(vr);
                x[xt2 + 1] = ui.wrapping_add(vi);

                xt1 = xt2 + j;
                r -= mh << 1;
                if r == 0 {
                    break;
                }
            }
            j += 4;
        }

        // j = mh / 2: twiddle factor pi/4
        {
            let mut xt1 = mh >> 1;
            let mut r = n;
            loop {
                let mut xt2 = xt1 + (mh << 1);

                let (vi, vr) = cplx_mult_div2(x[xt2 + 1], x[xt2], w_pi_fourth, w_pi_fourth);
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_add(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_sub(vi);

                xt1 += mh;
                xt2 += mh;

                let (vr, vi) = cplx_mult_div2(x[xt2 + 1], x[xt2], w_pi_fourth, w_pi_fourth);
                let ur = x[xt1] >> 1;
                let ui = x[xt1 + 1] >> 1;
                x[xt1] = ur.wrapping_add(vr);
                x[xt1 + 1] = ui.wrapping_sub(vi);
                x[xt2] = ur.wrapping_sub(vr);
                x[xt2 + 1] = ui.wrapping_add(vi);

                xt1 = xt2 + mh;
                r -= mh << 1;
                if r == 0 {
                    break;
                }
            }
        }

        ldm -= 1;
        if ldm == 0 {
            break;
        }
    }
}

/// Vectorized [`dit_fft`] for `SineTable512` (AArch64 NEON).
///
/// The butterflies of one stage are independent of each other, so they may be computed in any
/// order; every butterfly uses exactly the arithmetic of the C++ code:
///
/// - The bit reversal permutation and the first two (radix-4) stages are fused into one
///   out-of-place pass: the radix-4 butterfly of output group `g` reads the complex inputs
///   `r`, `r + n/2`, `r + n/4` and `r + 3n/4` with `r = bitrev(g)`, so four consecutive `r` are
///   processed in parallel and the results are scattered to the groups `bitrev(r)`.
/// - Each radix-2 butterfly with the span `mh` (complex values) and index `k < mh` computes
///   `x1 = u + v`, `x2 = u - v` with `u = x1 >> 1` and
///   `v = (sr * ((x2.re * p + x2.im * q) >> 16), si * ((x2.re * r + x2.im * t) >> 16))`.
///   The C++ code handles the indices `k`, `mh/2 - k`, `mh/2 + k`, `mh - k` with four variants of
///   the complex multiplication (swapped operands, negated sine) and different signs in the
///   final additions; each variant maps to one set of coefficients `(p, q, r, t)` and signs
///   `(sr, si)` that reproduces it exactly (see `fast::build_coefs`). The trivial twiddle
///   factors `k = 0` and `k = mh/2` shift by 1 (`x >> 1 == (x * 32768) >> 16`). The products
///   are computed in 64 bits and truncated like `cplxMultDiv2()`; a negative sign negates the
///   truncated product (`u - v` instead of `u + v`), like the C++ code.
/// - The negation of a truncated product is folded into the multiplication: for a 64-bit sum
///   of products `A`, `-(A >> 16) == (-A + 0xFFFF) >> 16`, so a lane with a negative sign uses
///   the negated coefficients and the bias `0xFFFF` in the accumulator. With the signs folded,
///   `(r, t) == (-q, p)` holds for all lanes, i.e. `v = x2 * (p - i q)` (checked when the table
///   is built).
#[cfg(target_arch = "aarch64")]
mod fast {
    use super::*;
    use core::arch::aarch64::*;
    use core::mem::MaybeUninit;
    use std::sync::OnceLock;

    /// Largest supported `ldn`.
    pub const MAX_LDN: i32 = 10;
    const MAX_N: usize = 1 << MAX_LDN;

    /// Coefficients of 4 butterflies with the signs folded in: `v = ((x2.re * p + x2.im * q +
    /// br) >> 16, (x2.im * p - x2.re * q + bi) >> 16)`.
    #[repr(C, align(16))]
    #[derive(Clone, Copy)]
    struct BlockCoef {
        p: [i32; 4],
        q: [i32; 4],
        nq: [i32; 4],
        br: [i64; 4],
        bi: [i64; 4],
    }

    /// Coefficients of the radix-2 stages `mh = 4, 8, ..., MAX_N / 2`; the stage `mh` starts at
    /// block `(mh - 4) / 4`.
    fn coefs() -> &'static [BlockCoef] {
        static COEFS: OnceLock<Box<[BlockCoef]>> = OnceLock::new();
        COEFS.get_or_init(|| {
            let lanes = build_coefs(&SINE_TABLE_512, 512);
            lanes
                .chunks_exact(24)
                .map(|b| {
                    let f = |field: usize, l: usize| b[4 * field + l];
                    let mut c =
                        BlockCoef { p: [0; 4], q: [0; 4], nq: [0; 4], br: [0; 4], bi: [0; 4] };
                    for l in 0..4 {
                        let (p, q, r, t) = (f(0, l), f(1, l), f(2, l), f(3, l));
                        let (sr, si) = (f(4, l) != 0, f(5, l) != 0);
                        let (p, q) = if sr { (-p, -q) } else { (p, q) };
                        let (r, t) = if si { (-r, -t) } else { (r, t) };
                        assert!(r == -q && t == p, "FFT coefficient table");
                        c.p[l] = p;
                        c.q[l] = q;
                        c.nq[l] = -q;
                        c.br[l] = if sr { 0xFFFF } else { 0 };
                        c.bi[l] = if si { 0xFFFF } else { 0 };
                    }
                    c
                })
                .collect()
        })
    }

    /// Coefficient table of the radix-2 stages for the sine table `trigdata` of size
    /// `trig_data_size` as blocks of 4 lanes of `p`, `q`, `r`, `t`, `sr`, `si` (sign masks 0
    /// or -1), see the module documentation.
    fn build_coefs(trigdata: &[FixpStp], trig_data_size: usize) -> Box<[i32]> {
        let w = stc(0x5a82799a) as i32; // twiddle factor pi/4
        let mut out = Vec::with_capacity(6 * MAX_N);
        let mut mh = 4usize;
        // The C++ code halves trigstep (starting with trigDataSize) in every stage.
        let mut trigstep = trig_data_size >> 1;
        while mh < MAX_N {
            let q = mh / 4;
            let cs = |i: usize| {
                let t = trigdata[i * trigstep];
                (t.re as i32, t.im as i32)
            };
            let lane = |k: usize| -> [i32; 6] {
                if k == 0 {
                    // ur + (vr >> 1), ui + (vi >> 1)
                    [32768, 0, 0, 32768, 0, 0]
                } else if k < q {
                    // (vi, vr) = cplxMultDiv2(x2.im, x2.re, c, s); x1 = u + v
                    let (c, s) = cs(k);
                    [c, s, -s, c, 0, 0]
                } else if k == q {
                    [w, w, -w, w, 0, 0]
                } else if k < 2 * q {
                    // index mh/2 - k': (vi, vr) = cplxMultDiv2(x2.re, x2.im, c, s);
                    // x1 = (ur + vr, ui - vi)
                    let (c, s) = cs(2 * q - k);
                    [s, c, c, -s, 0, -1]
                } else if k == 2 * q {
                    // vr = x2.im >> 1, vi = x2.re >> 1; x1 = (ur + vr, ui - vi)
                    [0, 32768, 32768, 0, 0, -1]
                } else if k < 3 * q {
                    // index mh/2 + k': (vr, vi) = cplxMultDiv2(x2.im, x2.re, c, s);
                    // x1 = (ur + vr, ui - vi)
                    let (c, s) = cs(k - 2 * q);
                    [-s, c, c, s, 0, -1]
                } else if k == 3 * q {
                    [-w, w, w, w, 0, -1]
                } else {
                    // index mh - k': (vr, vi) = cplxMultDiv2(x2.re, x2.im, c, s);
                    // x1 = (ur - vr, ui - vi)
                    let (c, s) = cs(4 * q - k);
                    [c, -s, s, c, -1, -1]
                }
            };
            for blk in 0..mh / 4 {
                let lanes =
                    [lane(4 * blk), lane(4 * blk + 1), lane(4 * blk + 2), lane(4 * blk + 3)];
                for f in 0..6 {
                    for l in &lanes {
                        out.push(l[f]);
                    }
                }
            }
            mh <<= 1;
            trigstep >>= 1;
        }
        out.into_boxed_slice()
    }

    pub fn dit_fft(x: &mut [FixpDbl], ldn: u32) {
        let n = 1usize << ldn;
        assert!((4..=MAX_LDN as u32).contains(&ldn) && x.len() >= 2 * n);
        let coefs = coefs();
        let mut tmp = [MaybeUninit::<i32>::uninit(); 2 * MAX_N];
        // SAFETY: `x` holds `2 * n` values and `tmp` at least `2 * n`; the first pass reads
        // `x` and writes all `2 * n` values of `tmp` before the first stage reads them. The
        // coefficients of stage `mh` are `mh / 4` blocks from block `(mh - 4) / 4`, within the
        // table for `mh <= MAX_N / 2`.
        unsafe {
            let xp = x.as_mut_ptr();
            let tp = tmp.as_mut_ptr() as *mut i32;
            first_pass(xp, tp, ldn);
            let mut src = tp as *const i32;
            let mut mh = 4usize;
            while 2 * mh < n {
                stage2(src, xp, n, mh, coefs.as_ptr());
                src = xp;
                mh <<= 2;
            }
            if mh < n {
                stage(src, xp, n, mh, coefs.as_ptr().add((mh - 4) / 4));
            }
        }
    }

    /// Bit reversal permutation and radix-4 butterflies (the first two stages) from `x` into
    /// `y` (`2 * 2^ldn` values each, `ldn >= 4`).
    #[inline(always)]
    unsafe fn first_pass(x: *const i32, y: *mut i32, ldn: u32) {
        let q = 1usize << (ldn - 2);
        let shift = 32 - (ldn - 2);
        let mut r = 0usize;
        while r < q {
            let c0 = vld2q_s32(x.add(2 * r));
            let c1 = vld2q_s32(x.add(2 * (r + 2 * q)));
            let c2 = vld2q_s32(x.add(2 * (r + q)));
            let c3 = vld2q_s32(x.add(2 * (r + 3 * q)));

            let a00 = vshrq_n_s32::<1>(vaddq_s32(c0.0, c1.0));
            let a10 = vshrq_n_s32::<1>(vaddq_s32(c2.0, c3.0));
            let a20 = vshrq_n_s32::<1>(vaddq_s32(c0.1, c1.1));
            let a30 = vshrq_n_s32::<1>(vaddq_s32(c2.1, c3.1));

            let y0r = vaddq_s32(a00, a10);
            let y2r = vsubq_s32(a00, a10);
            let y0i = vaddq_s32(a20, a30);
            let y2i = vsubq_s32(a20, a30);

            let a00 = vsubq_s32(a00, c1.0);
            let a10 = vsubq_s32(a10, c3.0);
            let a20 = vsubq_s32(a20, c1.1);
            let a30 = vsubq_s32(a30, c3.1);

            let y1r = vaddq_s32(a00, a30);
            let y3r = vsubq_s32(a00, a30);
            let y1i = vsubq_s32(a20, a10);
            let y3i = vaddq_s32(a20, a10);

            let lo = int32x4x4_t(y0r, y0i, y1r, y1i);
            let hi = int32x4x4_t(y2r, y2i, y3r, y3i);
            let dst = |l: usize| y.add(8 * (((r + l) as u32).reverse_bits() >> shift) as usize);
            let d = dst(0);
            vst4q_lane_s32::<0>(d, lo);
            vst4q_lane_s32::<0>(d.add(4), hi);
            let d = dst(1);
            vst4q_lane_s32::<1>(d, lo);
            vst4q_lane_s32::<1>(d.add(4), hi);
            let d = dst(2);
            vst4q_lane_s32::<2>(d, lo);
            vst4q_lane_s32::<2>(d.add(4), hi);
            let d = dst(3);
            vst4q_lane_s32::<3>(d, lo);
            vst4q_lane_s32::<3>(d.add(4), hi);
            r += 4;
        }
    }

    /// `(bias + a * p + b * q) >> 16` per lane in 64 bits, truncated to 32 bits.
    #[inline(always)]
    unsafe fn mac2b(
        bias: (int64x2_t, int64x2_t),
        a: int32x4_t,
        p: int32x4_t,
        b: int32x4_t,
        q: int32x4_t,
    ) -> int32x4_t {
        let lo = vmlal_s32(
            vmlal_s32(bias.0, vget_low_s32(a), vget_low_s32(p)),
            vget_low_s32(b),
            vget_low_s32(q),
        );
        let hi = vmlal_high_s32(vmlal_high_s32(bias.1, a, p), b, q);
        vshrn_high_n_s64::<16>(vshrn_n_s64::<16>(lo), hi)
    }

    /// Coefficients of 4 butterflies in registers.
    #[derive(Clone, Copy)]
    struct Coef {
        p: int32x4_t,
        q: int32x4_t,
        nq: int32x4_t,
        br: (int64x2_t, int64x2_t),
        bi: (int64x2_t, int64x2_t),
    }

    #[inline(always)]
    unsafe fn load_coef(c: *const BlockCoef) -> Coef {
        let c = &*c;
        Coef {
            p: vld1q_s32(c.p.as_ptr()),
            q: vld1q_s32(c.q.as_ptr()),
            nq: vld1q_s32(c.nq.as_ptr()),
            br: (vld1q_s64(c.br.as_ptr()), vld1q_s64(c.br.as_ptr().add(2))),
            bi: (vld1q_s64(c.bi.as_ptr()), vld1q_s64(c.bi.as_ptr().add(2))),
        }
    }

    /// 4 radix-2 butterflies: returns `(x1 >> 1) + v` and `(x1 >> 1) - v` (see the module
    /// documentation).
    #[inline(always)]
    unsafe fn bfly(x1: int32x4x2_t, x2: int32x4x2_t, c: &Coef) -> (int32x4x2_t, int32x4x2_t) {
        let ur = vshrq_n_s32::<1>(x1.0);
        let ui = vshrq_n_s32::<1>(x1.1);
        let vr = mac2b(c.br, x2.0, c.p, x2.1, c.q);
        let vi = mac2b(c.bi, x2.1, c.p, x2.0, c.nq);
        (
            int32x4x2_t(vaddq_s32(ur, vr), vaddq_s32(ui, vi)),
            int32x4x2_t(vsubq_s32(ur, vr), vsubq_s32(ui, vi)),
        )
    }

    /// One radix-2 stage with the span `mh` from `src` to `dst` (may be the same buffer). The
    /// coefficients of a block of 4 butterflies are loaded once for all groups.
    #[inline(always)]
    unsafe fn stage(src: *const i32, dst: *mut i32, n: usize, mh: usize, coef: *const BlockCoef) {
        let mut k = 0usize;
        while k < mh {
            let c = load_coef(coef.add(k / 4));
            let mut g = k;
            while g < n {
                let a = vld2q_s32(src.add(2 * g));
                let b = vld2q_s32(src.add(2 * (g + mh)));
                let (a, b) = bfly(a, b, &c);
                vst2q_s32(dst.add(2 * g), a);
                vst2q_s32(dst.add(2 * (g + mh)), b);
                g += 2 * mh;
            }
            k += 4;
        }
    }

    /// The radix-2 stages with the spans `mh` and `2 * mh` in one pass: the complex values
    /// `k`, `k + mh`, `k + 2 mh`, `k + 3 mh` of a group of `4 mh` values only depend on each other
    /// in these two stages (stage `mh`: pairs `(k, k + mh)`, `(k + 2 mh, k + 3 mh)`; stage `2 mh`:
    /// pairs `(k, k + 2 mh)`, `(k + mh, k + 3 mh)`).
    #[inline(always)]
    unsafe fn stage2(src: *const i32, dst: *mut i32, n: usize, mh: usize, coefs: *const BlockCoef) {
        let c1_base = coefs.add((mh - 4) / 4);
        let c2_base = coefs.add((2 * mh - 4) / 4);
        let mut k = 0usize;
        while k < mh {
            let c1 = load_coef(c1_base.add(k / 4));
            let c2a = load_coef(c2_base.add(k / 4));
            let c2b = load_coef(c2_base.add((k + mh) / 4));
            let mut g = k;
            while g < n {
                let x0 = vld2q_s32(src.add(2 * g));
                let x1 = vld2q_s32(src.add(2 * (g + mh)));
                let x2 = vld2q_s32(src.add(2 * (g + 2 * mh)));
                let x3 = vld2q_s32(src.add(2 * (g + 3 * mh)));
                let (x0, x1) = bfly(x0, x1, &c1);
                let (x2, x3) = bfly(x2, x3, &c1);
                let (x0, x2) = bfly(x0, x2, &c2a);
                let (x1, x3) = bfly(x1, x3, &c2b);
                vst2q_s32(dst.add(2 * g), x0);
                vst2q_s32(dst.add(2 * (g + mh)), x1);
                vst2q_s32(dst.add(2 * (g + 2 * mh)), x2);
                vst2q_s32(dst.add(2 * (g + 3 * mh)), x3);
                g += 4 * mh;
            }
            k += 4;
        }
    }
}

/// Vectorized [`dit_fft`] for `SineTable512` with the arithmetic of the x86 build (AVX2).
///
/// The bit reversal is fused into an out-of-place pass of the first two (radix-4) stages into a
/// buffer, which the first radix-2 stage reads. The radix-2 stages compute 4 butterflies (8
/// values of the interleaved real/imaginary data) at a time.
/// In the x86 build, `cplxMultDiv2()` subtracts or adds the individually truncated products
/// `M(x, w) = (x * w) >> 16`, so every output value of a butterfly is
/// `(x1 >> 1) +- (sa * M(x2, a) + sb * M(swap(x2), b))`, where `swap(x2)` exchanges the real and
/// imaginary parts and the coefficients `a`, `b` and the signs `sa`, `sb` (+-1) of every value
/// reproduce the four variants of the butterfly in the C++ code (indices `k`, `mh/2 - k`,
/// `mh/2 + k`, `mh - k` of the span `mh`, see `build`). The trivial twiddle factors `k = 0` and
/// `k = mh/2` use `x >> 1 == M(x, 32768)` and `M(x, 0) == 0`.
///
/// `M(x, w)` is computed exactly in 32 bit lanes as `(x >> 16) * w + (((x & 0xFFFF) * w) >> 16)`
/// (no intermediate result exceeds 31 bits for `|w| <= 32768`); a negative sign negates the
/// truncated product, and all sums wrap around like the C++ code.
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
mod fast_x86 {
    use super::*;
    use crate::simd::x86::*;
    use std::sync::OnceLock;

    /// Largest supported `ldn`.
    pub const MAX_LDN: i32 = 10;
    const MAX_N: usize = 1 << MAX_LDN;

    /// Coefficients of the radix-2 stages `mh = 4, 8, ..., MAX_N / 2` for every value of the
    /// interleaved data: stage `mh` uses the `2 * mh` entries from `2 * (mh - 4)`.
    struct Tables {
        a: Box<[i32]>,
        b: Box<[i32]>,
        /// signs (1 or -1)
        sa: Box<[i32]>,
        sb: Box<[i32]>,
    }

    fn tables() -> &'static Tables {
        static TABLES: OnceLock<Tables> = OnceLock::new();
        TABLES.get_or_init(|| build(&SINE_TABLE_512, 512))
    }

    /// The coefficients for the sine table `trigdata` of size `trig_data_size`.
    fn build(trigdata: &[FixpStp], trig_data_size: usize) -> Tables {
        let w = stc(0x5a82799a) as i32; // twiddle factor pi/4
        let len = 2 * (MAX_N - 4);
        let (mut a, mut b, mut sa, mut sb) =
            (Vec::with_capacity(len), Vec::with_capacity(len), Vec::new(), Vec::new());
        let mut mh = 4usize;
        // The C++ code halves trigstep (starting with trigDataSize) in every stage.
        let mut trigstep = trig_data_size >> 1;
        while mh < MAX_N {
            let q = mh / 4;
            let cs = |i: usize| {
                let t = trigdata[i * trigstep];
                (t.re as i32, t.im as i32)
            };
            for k in 0..mh {
                // vr = s1 M(re, a1) + s2 M(im, b1), vi = s3 M(re, a2) + s4 M(im, b2),
                // x1 = (ur + tr vr, ui + ti vi), x2 = (ur - tr vr, ui - ti vi); signs as +-1
                let [a1, s1, b1, s2, a2, s3, b2, s4, tr, ti] = if k == 0 {
                    [32768, 1, 0, 1, 0, 1, 32768, 1, 1, 1]
                } else if k <= q {
                    // (vi, vr) = cplxMultDiv2(x2.im, x2.re, c, s): vr = M(re, c) + M(im, s),
                    // vi = M(im, c) - M(re, s)
                    let (c, s) = if k == q { (w, w) } else { cs(k) };
                    [c, 1, s, 1, s, -1, c, 1, 1, 1]
                } else if k < 2 * q {
                    // index mh/2 - k': (vi, vr) = cplxMultDiv2(x2.re, x2.im, c, s):
                    // vr = M(re, s) + M(im, c), vi = M(re, c) - M(im, s); x1 = (ur + vr, ui - vi)
                    let (c, s) = cs(2 * q - k);
                    [s, 1, c, 1, c, 1, s, -1, 1, -1]
                } else if k == 2 * q {
                    // vr = x2.im >> 1, vi = x2.re >> 1; x1 = (ur + vr, ui - vi)
                    [0, 1, 32768, 1, 32768, 1, 0, 1, 1, -1]
                } else if k <= 3 * q {
                    // index mh/2 + k': (vr, vi) = cplxMultDiv2(x2.im, x2.re, c, s):
                    // vr = M(im, c) - M(re, s), vi = M(im, s) + M(re, c); x1 = (ur + vr, ui - vi)
                    let (c, s) = if k == 3 * q { (w, w) } else { cs(k - 2 * q) };
                    [s, -1, c, 1, c, 1, s, 1, 1, -1]
                } else {
                    // index mh - k': (vr, vi) = cplxMultDiv2(x2.re, x2.im, c, s):
                    // vr = M(re, c) - M(im, s), vi = M(re, s) + M(im, c); x1 = (ur - vr, ui - vi)
                    let (c, s) = cs(4 * q - k);
                    [c, 1, s, -1, s, 1, c, 1, -1, -1]
                };
                // real part: x2 = re, swap(x2) = im
                a.push(a1);
                sa.push(tr * s1);
                b.push(b1);
                sb.push(tr * s2);
                // imaginary part: x2 = im, swap(x2) = re
                a.push(b2);
                sa.push(ti * s4);
                b.push(a2);
                sb.push(ti * s3);
            }
            mh <<= 1;
            trigstep >>= 1;
        }
        Tables {
            a: a.into_boxed_slice(),
            b: b.into_boxed_slice(),
            sa: sa.into_boxed_slice(),
            sb: sb.into_boxed_slice(),
        }
    }

    /// [`dit_fft`] for `SineTable512` and `3 <= ldn <= MAX_LDN`.
    ///
    /// # Safety
    ///
    /// AVX2 must be available.
    #[target_feature(enable = "avx2")]
    pub unsafe fn dit_fft(x: &mut [FixpDbl], ldn: u32) {
        let n = 1usize << ldn;
        assert!((3..=MAX_LDN as u32).contains(&ldn) && x.len() >= 2 * n);
        let t = tables();
        let p = x.as_mut_ptr();
        let mut tmp = [core::mem::MaybeUninit::<i32>::uninit(); 2 * MAX_N];
        let tp = tmp.as_mut_ptr() as *mut i32;
        let mut mh = 4usize;
        let mut src = p as *const i32;
        if ldn >= 4 {
            // SAFETY: x holds 2 * n values and tmp at least 2 * n; the pass writes all of them.
            unsafe { first_pass(p, tp, ldn) };
            src = tp;
        } else {
            scramble(x, n as i32);
            // SAFETY: x holds 2 * n values.
            unsafe { radix4(p, n) };
        }
        while mh < n {
            let base = 2 * (mh - 4);
            // SAFETY: the stage reads and writes the values 2 * (g + k) .. 2 * (g + k + mh) + 8
            // with g + k + mh + 4 <= n, and the coefficients base + 2 * k + 8 <= 2 * (MAX_N - 4).
            unsafe {
                stage(
                    src,
                    p,
                    n,
                    mh,
                    t.a.as_ptr().add(base),
                    t.b.as_ptr().add(base),
                    t.sa.as_ptr().add(base),
                    t.sb.as_ptr().add(base),
                )
            };
            src = p;
            mh <<= 1;
        }
    }

    /// Bit reversal of 8 bit values.
    static BITREV8: [u8; 256] = {
        let mut t = [0u8; 256];
        let mut i = 0;
        while i < 256 {
            t[i] = (i as u8).reverse_bits();
            i += 1;
        }
        t
    };

    /// Bit reversal permutation and the radix-4 butterflies of the first two stages (see
    /// `radix4_first_stages()`) from `x` into `y` (`2 * 2^ldn` values each, `ldn >= 4`): the
    /// output group `g` (4 complex values) reads the inputs `r`, `r + n/2`, `r + n/4`,
    /// `r + 3n/4` with `r = bitrev(g)`, so 4 consecutive `r` are loaded as 4 vectors, the
    /// butterflies computed on the interleaved values, and the results transposed into the 4
    /// output groups `bitrev(r + l) = bitrev(r / 4) + rev2(l) << (ldn - 4)`.
    #[inline]
    #[target_feature(enable = "avx2")]
    unsafe fn first_pass(x: *const i32, y: *mut i32, ldn: u32) {
        let q = 1usize << (ldn - 2);
        let kb = ldn - 4;
        let sign = _mm256_setr_epi32(1, -1, 1, -1, 1, -1, 1, -1);
        for u in 0..q / 4 {
            let r = 4 * u;
            // SAFETY: r + 3q + 4 <= n; the groups are below n / 4.
            unsafe {
                let ld = |k: usize| _mm256_loadu_si256(x.add(2 * k) as *const __m256i);
                let (c0, c1, c2, c3) = (ld(r), ld(r + 2 * q), ld(r + q), ld(r + 3 * q));
                let s = _mm256_srai_epi32::<1>(_mm256_add_epi32(c0, c1));
                let t = _mm256_srai_epi32::<1>(_mm256_add_epi32(c2, c3));
                let y0 = _mm256_add_epi32(s, t);
                let y2 = _mm256_sub_epi32(s, t);
                let s = _mm256_sub_epi32(s, c1);
                let t = _mm256_sub_epi32(t, c3);
                // (t.im, -t.re)
                let ts = _mm256_sign_epi32(_mm256_shuffle_epi32::<0b10_11_00_01>(t), sign);
                let y1 = _mm256_add_epi32(s, ts);
                let y3 = _mm256_sub_epi32(s, ts);
                // transpose the 4 x 4 complex values
                let t0 = _mm256_unpacklo_epi64(y0, y1);
                let t1 = _mm256_unpackhi_epi64(y0, y1);
                let t2 = _mm256_unpacklo_epi64(y2, y3);
                let t3 = _mm256_unpackhi_epi64(y2, y3);
                let g = (BITREV8[u] as usize) >> (8 - kb);
                let st =
                    |g: usize, v: __m256i| _mm256_storeu_si256(y.add(8 * g) as *mut __m256i, v);
                st(g, _mm256_permute2x128_si256::<0x20>(t0, t2));
                st(g + (2 << kb), _mm256_permute2x128_si256::<0x20>(t1, t3));
                st(g + (1 << kb), _mm256_permute2x128_si256::<0x31>(t0, t2));
                st(g + (3 << kb), _mm256_permute2x128_si256::<0x31>(t1, t3));
            }
        }
    }

    /// The combined radix-4 butterflies of the first two stages (see `radix4_first_stages()`),
    /// one per register `(r0 i0 r1 i1 r2 i2 r3 i3)`: the sums `a = (x0 + x1) >> 1` and the
    /// differences `b = a - x1` of the complex pairs `(x0, x1)` give `t = (a0 b0 | a1 b1)`
    /// (complex values `a00 a20 b00 b20 | a10 a30 b10 b30` in the C++ code), the output is
    /// `(lo + h | lo - h)` with `lo = (a00 a20 b00 b20)` and `h = (a10 a30 b30 -b10)`.
    #[inline]
    #[target_feature(enable = "avx2")]
    unsafe fn radix4(x: *mut i32, n: usize) {
        let sign = _mm256_setr_epi32(1, 1, 1, -1, -1, -1, -1, 1);
        let idx_h = _mm256_setr_epi32(4, 5, 7, 6, 4, 5, 7, 6);
        for k in 0..n / 4 {
            // SAFETY: the values 8k .. 8k + 8 are within the 2n values.
            unsafe {
                let p = x.add(8 * k) as *mut __m256i;
                let v = _mm256_loadu_si256(p);
                let vs = _mm256_shuffle_epi32::<0b01_00_11_10>(v);
                let a = _mm256_srai_epi32::<1>(_mm256_add_epi32(v, vs));
                let t = _mm256_blend_epi32::<0b1100_1100>(a, _mm256_sub_epi32(a, v));
                let lo = _mm256_permute4x64_epi64::<0b01_00_01_00>(t);
                let h = _mm256_sign_epi32(_mm256_permutevar8x32_epi32(t, idx_h), sign);
                _mm256_storeu_si256(p, _mm256_add_epi32(lo, h));
            }
        }
    }

    /// `v` negated where `sign` is -1.
    #[inline]
    #[target_feature(enable = "avx2")]
    unsafe fn cneg(v: __m256i, sign: __m256i) -> __m256i {
        _mm256_sign_epi32(v, sign)
    }

    /// One radix-2 stage with the span `mh >= 4` (complex values) from `src` to `dst` (the
    /// same buffer, or different ones).
    #[inline]
    #[target_feature(enable = "avx2")]
    unsafe fn stage(
        src: *const i32,
        dst: *mut i32,
        n: usize,
        mh: usize,
        a: *const i32,
        b: *const i32,
        sa: *const i32,
        sb: *const i32,
    ) {
        let mut k = 0usize;
        while k < mh {
            // SAFETY: see `dit_fft`.
            unsafe {
                let ca = _mm256_loadu_si256(a.add(2 * k) as *const __m256i);
                let cb = _mm256_loadu_si256(b.add(2 * k) as *const __m256i);
                let ma = _mm256_loadu_si256(sa.add(2 * k) as *const __m256i);
                let mb = _mm256_loadu_si256(sb.add(2 * k) as *const __m256i);
                let mut g = k;
                while g < n {
                    let p1 = dst.add(2 * g) as *mut __m256i;
                    let p2 = dst.add(2 * (g + mh)) as *mut __m256i;
                    let x1 = _mm256_loadu_si256(src.add(2 * g) as *const __m256i);
                    let x2 = _mm256_loadu_si256(src.add(2 * (g + mh)) as *const __m256i);
                    let x2s = _mm256_shuffle_epi32::<0b10_11_00_01>(x2);
                    let v =
                        _mm256_add_epi32(cneg(mul_div2(x2, ca), ma), cneg(mul_div2(x2s, cb), mb));
                    let u = _mm256_srai_epi32::<1>(x1);
                    _mm256_storeu_si256(p1, _mm256_add_epi32(u, v));
                    _mm256_storeu_si256(p2, _mm256_sub_epi32(u, v));
                    g += 2 * mh;
                }
            }
            k += 4;
        }
    }
}
