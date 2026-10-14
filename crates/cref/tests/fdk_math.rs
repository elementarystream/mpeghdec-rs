// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of the fixed-point primitives (common_fix.h, fixpoint_math.h, scale.h).
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_math::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::fixpoint_math::*;
use mpeghdec_fdk::scale::*;
use mpeghdec_fdk::*;

const N: usize = 200_000;

macro_rules! check {
    ($name:expr, $rust:expr, $c:expr, $($arg:expr),*) => {{
        let r = $rust;
        let c = unsafe { $c };
        assert_eq!(r, c, "{} mismatch for args {:?}", $name, ($($arg,)*));
    }};
}

#[test]
fn fmult_family() {
    let mut g = Rng::new(1);
    for _ in 0..N {
        let (a, b, x) = (g.i32(), g.i32(), g.i32());
        let (s, t) = (g.i16(), g.i16());
        let (q, r) = (g.i64(), g.i64());
        check!("fMult_DD", f_mult(a, b), cref_fMult_DD(a, b), a, b);
        check!("fMult_DS", f_mult(a, s), cref_fMult_DS(a, s), a, s);
        check!("fMult_SD", f_mult(s, a), cref_fMult_SD(s, a), s, a);
        check!("fMult_SS", f_mult(s, t), cref_fMult_SS(s, t), s, t);
        check!("fMultDiv2_DD", f_mult_div2(a, b), cref_fMultDiv2_DD(a, b), a, b);
        check!("fMultDiv2_DS", f_mult_div2(a, s), cref_fMultDiv2_DS(a, s), a, s);
        check!("fMultDiv2_SD", f_mult_div2(s, a), cref_fMultDiv2_SD(s, a), s, a);
        check!("fMultDiv2_SS", f_mult_div2(s, t), cref_fMultDiv2_SS(s, t), s, t);
        check!("fMult_QQ", f_mult(q, r), cref_fMult_QQ(q, r), q, r);
        check!("fMult_QD", f_mult(q, a), cref_fMult_QD(q, a), q, a);
        check!("fMultDiv2_QQ", f_mult_div2(q, r), cref_fMultDiv2_QQ(q, r), q, r);
        check!("fMultDiv2_QD", f_mult_div2(q, a), cref_fMultDiv2_QD(q, a), q, a);
        check!("fMultBitExact_DD", f_mult_bit_exact(a, b), cref_fMultBitExact_DD(a, b), a, b);
        check!("fMultBitExact_DS", f_mult_bit_exact(a, s), cref_fMultBitExact_DS(a, s), a, s);
        check!("fMultBitExact_SD", f_mult_bit_exact(s, a), cref_fMultBitExact_SD(s, a), s, a);
        check!(
            "fMultDiv2BitExact_DD",
            f_mult_div2_bit_exact(a, b),
            cref_fMultDiv2BitExact_DD(a, b),
            a,
            b
        );
        check!(
            "fMultDiv2BitExact_DS",
            f_mult_div2_bit_exact(a, s),
            cref_fMultDiv2BitExact_DS(a, s),
            a,
            s
        );
        check!(
            "fMultDiv2BitExact_SD",
            f_mult_div2_bit_exact(s, a),
            cref_fMultDiv2BitExact_SD(s, a),
            s,
            a
        );
        check!("fPow2_D", f_pow2(a), cref_fPow2_D(a), a);
        check!("fPow2_S", f_pow2(s), cref_fPow2_S(s), s);
        check!("fPow2Div2_D", f_pow2_div2(a), cref_fPow2Div2_D(a), a);
        check!("fPow2Div2_S", f_pow2_div2(s), cref_fPow2Div2_S(s), s);

        check!("fMultAdd_DD", f_mult_add(x, a, b), cref_fMultAdd_DD(x, a, b), x, a, b);
        check!("fMultAdd_DS", f_mult_add(x, a, s), cref_fMultAdd_DS(x, a, s), x, a, s);
        check!("fMultAdd_SD", f_mult_add(x, s, a), cref_fMultAdd_SD(x, s, a), x, s, a);
        check!("fMultAdd_SS", f_mult_add(x, s, t), cref_fMultAdd_SS(x, s, t), x, s, t);
        check!("fMultAddDiv2_DD", f_mult_add_div2(x, a, b), cref_fMultAddDiv2_DD(x, a, b), x, a, b);
        check!("fMultAddDiv2_DS", f_mult_add_div2(x, a, s), cref_fMultAddDiv2_DS(x, a, s), x, a, s);
        check!("fMultAddDiv2_SD", f_mult_add_div2(x, s, a), cref_fMultAddDiv2_SD(x, s, a), x, s, a);
        check!("fMultAddDiv2_SS", f_mult_add_div2(x, s, t), cref_fMultAddDiv2_SS(x, s, t), x, s, t);
        check!("fMultSub_DD", f_mult_sub(x, a, b), cref_fMultSub_DD(x, a, b), x, a, b);
        check!("fMultSub_DS", f_mult_sub(x, a, s), cref_fMultSub_DS(x, a, s), x, a, s);
        check!("fMultSub_SD", f_mult_sub(x, s, a), cref_fMultSub_SD(x, s, a), x, s, a);
        check!("fMultSub_SS", f_mult_sub(x, s, t), cref_fMultSub_SS(x, s, t), x, s, t);
        check!("fMultSubDiv2_DD", f_mult_sub_div2(x, a, b), cref_fMultSubDiv2_DD(x, a, b), x, a, b);
        check!("fMultSubDiv2_DS", f_mult_sub_div2(x, a, s), cref_fMultSubDiv2_DS(x, a, s), x, a, s);
        check!("fMultSubDiv2_SD", f_mult_sub_div2(x, s, a), cref_fMultSubDiv2_SD(x, s, a), x, s, a);
        check!("fMultSubDiv2_SS", f_mult_sub_div2(x, s, t), cref_fMultSubDiv2_SS(x, s, t), x, s, t);
        check!(
            "fMultAddDiv2BitExact_DD",
            f_mult_add_div2_bit_exact(x, a, b),
            cref_fMultAddDiv2BitExact_DD(x, a, b),
            x,
            a,
            b
        );
        check!(
            "fMultAddDiv2BitExact_DS",
            f_mult_add_div2_bit_exact(x, a, s),
            cref_fMultAddDiv2BitExact_DS(x, a, s),
            x,
            a,
            s
        );
        check!(
            "fMultAddDiv2BitExact_SD",
            f_mult_add_div2_bit_exact(x, s, a),
            cref_fMultAddDiv2BitExact_SD(x, s, a),
            x,
            s,
            a
        );
        check!(
            "fMultSubDiv2BitExact_DD",
            f_mult_sub_div2_bit_exact(x, a, b),
            cref_fMultSubDiv2BitExact_DD(x, a, b),
            x,
            a,
            b
        );
        check!(
            "fMultSubDiv2BitExact_DS",
            f_mult_sub_div2_bit_exact(x, a, s),
            cref_fMultSubDiv2BitExact_DS(x, a, s),
            x,
            a,
            s
        );
        check!(
            "fMultSubDiv2BitExact_SD",
            f_mult_sub_div2_bit_exact(x, s, a),
            cref_fMultSubDiv2BitExact_SD(x, s, a),
            x,
            s,
            a
        );
        check!("fPow2AddDiv2_D", f_pow2_add_div2(x, a), cref_fPow2AddDiv2_D(x, a), x, a);
        check!("fPow2AddDiv2_S", f_pow2_add_div2_sgl(x, s), cref_fPow2AddDiv2_S(x, s), x, s);
        check!("fPow2Add_D", f_pow2_add(x, a), cref_fPow2Add_D(x, a), x, a);
        check!("fPow2Add_S", f_pow2_add_sgl(x, s), cref_fPow2Add_S(x, s), x, s);
    }
}

#[test]
fn cplx_mult_family() {
    let mut g = Rng::new(2);
    let mut o = [0i32; 2];
    let mut os = [0i16; 2];
    for _ in 0..N {
        let (ar, ai, br, bi, cr, ci) = (g.i32(), g.i32(), g.i32(), g.i32(), g.i32(), g.i32());
        let (sr, si, tr, ti) = (g.i16(), g.i16(), g.i16(), g.i16());
        let args = (ar, ai, br, bi, cr, ci, sr, si, tr, ti);
        unsafe { cref_cplxMultDiv2_DD(o.as_mut_ptr(), ar, ai, br, bi) };
        assert_eq!(cplx_mult_div2(ar, ai, br, bi), (o[0], o[1]), "cplxMultDiv2_DD {args:?}");
        unsafe { cref_cplxMultDiv2_DS(o.as_mut_ptr(), ar, ai, sr, si) };
        assert_eq!(cplx_mult_div2(ar, ai, sr, si), (o[0], o[1]), "cplxMultDiv2_DS {args:?}");
        unsafe { cref_cplxMultDiv2_SPK(o.as_mut_ptr(), ar, ai, sr, si) };
        assert_eq!(cplx_mult_div2(ar, ai, sr, si), (o[0], o[1]), "cplxMultDiv2_SPK {args:?}");
        unsafe { cref_cplxMult_DD(o.as_mut_ptr(), ar, ai, br, bi) };
        assert_eq!(cplx_mult(ar, ai, br, bi), (o[0], o[1]), "cplxMult_DD {args:?}");
        unsafe { cref_cplxMult_DS(o.as_mut_ptr(), ar, ai, sr, si) };
        assert_eq!(cplx_mult(ar, ai, sr, si), (o[0], o[1]), "cplxMult_DS {args:?}");
        o = [cr, ci];
        unsafe { cref_cplxMultAddDiv2_DD(o.as_mut_ptr(), ar, ai, br, bi) };
        assert_eq!(
            cplx_mult_add_div2(cr, ci, ar, ai, br, bi),
            (o[0], o[1]),
            "cplxMultAddDiv2_DD {args:?}"
        );
        o = [cr, ci];
        unsafe { cref_cplxMultAddDiv2_DS(o.as_mut_ptr(), ar, ai, sr, si) };
        assert_eq!(
            cplx_mult_add_div2(cr, ci, ar, ai, sr, si),
            (o[0], o[1]),
            "cplxMultAddDiv2_DS {args:?}"
        );
        o = [cr, ci];
        unsafe { cref_cplxMultSubDiv2_DD(o.as_mut_ptr(), ar, ai, br, bi) };
        assert_eq!(
            cplx_mult_sub_div2(cr, ci, ar, ai, br, bi),
            (o[0], o[1]),
            "cplxMultSubDiv2_DD {args:?}"
        );
        o = [cr, ci];
        unsafe { cref_cplxMultSubDiv2_DS(o.as_mut_ptr(), ar, ai, sr, si) };
        assert_eq!(
            cplx_mult_sub_div2(cr, ci, ar, ai, sr, si),
            (o[0], o[1]),
            "cplxMultSubDiv2_DS {args:?}"
        );
        unsafe { cref_cplxMultDiv2_SS(o.as_mut_ptr(), sr, si, tr, ti) };
        assert_eq!(cplx_mult_div2_16x16(sr, si, tr, ti), (o[0], o[1]), "cplxMultDiv2_SS {args:?}");
        unsafe { cref_cplxMultDiv2_SS_sgl(os.as_mut_ptr(), sr, si, tr, ti) };
        assert_eq!(
            cplx_mult_div2_16x16_sgl(sr, si, tr, ti),
            (os[0], os[1]),
            "cplxMultDiv2_SS_sgl {args:?}"
        );
    }
}

#[test]
fn abs_norm_scale() {
    let mut g = Rng::new(3);
    for _ in 0..N {
        let (a, s, q) = (g.i32(), g.i16(), g.i64());
        check!("fAbs_D", f_abs(a), cref_fAbs_D(a), a);
        check!("fAbs_S", f_abs(s), cref_fAbs_S(s), s);
        check!("fAbs_Q", f_abs(q), cref_fAbs_Q(q), q);
        check!("fNorm_D", f_norm(a), cref_fNorm_D(a), a);
        check!("fNorm_S", f_norm(s), cref_fNorm_S(s), s);
        check!("fNormz_D", f_normz(a), cref_fNormz_D(a), a);
        check!("fNormz_S", f_normz(s), cref_fNormz_S(s), s);
        check!("fNormz_Q", f_normz(q), cref_fNormz_Q(q), q);
        let sh = g.range(-31, 31);
        check!("scaleValue_D", scale_value(a, sh), cref_scaleValue_D(a, sh), a, sh);
        let shs = g.range(-15, 15);
        check!("scaleValue_S", scale_value_sgl(s, shs), cref_scaleValue_S(s, shs), s, shs);
        let shw = g.range(-128, 127);
        check!(
            "scaleValueSaturate",
            scale_value_saturate(a, shw),
            cref_scaleValueSaturate(a, shw),
            a,
            shw
        );
        let mut v = a;
        scale_value_in_place(&mut v, sh);
        check!("scaleValueInPlace", v, cref_scaleValueInPlace(a, sh), a, sh);
        for dbits in [16, 24, 32] {
            let sc = g.range(0, 31);
            check!(
                "SATURATE_RIGHT_SHIFT",
                saturate_right_shift(a, sc, dbits),
                cref_SATURATE_RIGHT_SHIFT(a, sc, dbits),
                a,
                sc,
                dbits
            );
            check!(
                "SATURATE_LEFT_SHIFT",
                saturate_left_shift(a, sc, dbits),
                cref_SATURATE_LEFT_SHIFT(a, sc, dbits),
                a,
                sc,
                dbits
            );
            check!(
                "SATURATE_SHIFT",
                saturate_shift(a, sh, dbits),
                cref_SATURATE_SHIFT(a, sh, dbits),
                a,
                sh,
                dbits
            );
            let scr = g.range(0, 30);
            check!(
                "SATURATE_RIGHT_SHIFT_RND",
                saturate_right_shift_rnd(a >> 1, scr, dbits),
                cref_SATURATE_RIGHT_SHIFT_RND(a >> 1, scr, dbits),
                a,
                scr,
                dbits
            );
            check!(
                "SATURATE_LEFT_SHIFT_ALT",
                saturate_left_shift_alt(a, sc, dbits),
                cref_SATURATE_LEFT_SHIFT_ALT(a, sc, dbits),
                a,
                sc,
                dbits
            );
            check!(
                "SATURATE_RIGHT_SHIFT_ALT",
                saturate_right_shift_alt(a, sc, dbits),
                cref_SATURATE_RIGHT_SHIFT_ALT(a, sc, dbits),
                a,
                sc,
                dbits
            );
        }
    }
}

#[test]
fn scale_vectors() {
    let mut g = Rng::new(4);
    for iter in 0..3000 {
        let len = (g.next_u32() % 67) as usize;
        let shift = if iter % 2 == 0 { g.range(-40, 40) } else { g.range(-3, 3) };
        let src: Vec<i32> = (0..len).map(|_| g.i32() >> g.range(0, 31)).collect();
        let src_s: Vec<i16> = (0..len).map(|_| g.i16() >> g.range(0, 15)).collect();
        let factor = g.i32();

        let (mut r, mut c) = (src.clone(), src.clone());
        scale_values_with_factor(&mut r, factor, shift);
        unsafe { cref_scaleValuesWithFactor(c.as_mut_ptr(), factor, len as i32, shift) };
        assert_eq!(r, c, "scaleValuesWithFactor {shift} {factor}");

        let (mut r, mut c) = (src.clone(), src.clone());
        scale_values(&mut r, shift);
        unsafe { cref_scaleValues_D(c.as_mut_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValues_D {shift}");

        let (mut r, mut c) = (src_s.clone(), src_s.clone());
        scale_values_sgl(&mut r, shift);
        unsafe { cref_scaleValues_S(c.as_mut_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValues_S {shift}");

        let (mut r, mut c) = (vec![0; len], vec![0; len]);
        scale_values_to(&mut r, &src, shift);
        unsafe { cref_scaleValues_DD(c.as_mut_ptr(), src.as_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValues_DD {shift}");

        let (mut r, mut c) = (src.clone(), src.clone());
        scale_values_saturate(&mut r, shift);
        unsafe { cref_scaleValuesSaturate_D(c.as_mut_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValuesSaturate_D {shift}");

        let (mut r, mut c) = (vec![0; len], vec![0; len]);
        scale_values_saturate_to(&mut r, &src, shift);
        unsafe { cref_scaleValuesSaturate_DD(c.as_mut_ptr(), src.as_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValuesSaturate_DD {shift}");

        let (mut r, mut c) = (vec![0i16; len], vec![0i16; len]);
        scale_values_saturate_dbl_to_sgl(&mut r, &src, shift);
        unsafe { cref_scaleValuesSaturate_SD(c.as_mut_ptr(), src.as_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValuesSaturate_SD {shift}");

        let (mut r, mut c) = (src_s.clone(), src_s.clone());
        scale_values_saturate_sgl(&mut r, shift);
        unsafe { cref_scaleValuesSaturate_S(c.as_mut_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValuesSaturate_S {shift}");

        let (mut r, mut c) = (vec![0i16; len], vec![0i16; len]);
        scale_values_saturate_sgl_to(&mut r, &src_s, shift);
        unsafe { cref_scaleValuesSaturate_SS(c.as_mut_ptr(), src_s.as_ptr(), len as i32, shift) };
        assert_eq!(r, c, "scaleValuesSaturate_SS {shift}");

        check!(
            "getScalefactorShort",
            get_scalefactor_short(&src_s),
            cref_getScalefactorShort(src_s.as_ptr(), len as i32),
            len
        );
        check!(
            "getScalefactor_D",
            get_scalefactor(&src),
            cref_getScalefactor_D(src.as_ptr(), len as i32),
            len
        );
        check!(
            "getScalefactor_S",
            get_scalefactor_sgl(&src_s),
            cref_getScalefactor_S(src_s.as_ptr(), len as i32),
            len
        );
        if len > 0 {
            let stride = 1 + (g.next_u32() % 3) as usize;
            let n = (len - 1) / stride + 1;
            check!(
                "getScalefactorPCM",
                get_scalefactor_pcm(&src, n, stride),
                cref_getScalefactorPCM(src.as_ptr(), n as i32, stride as i32),
                len,
                stride
            );
        }
    }
}

#[test]
fn fixpoint_math_functions() {
    let mut g = Rng::new(5);
    for _ in 0..N {
        let (a, b) = (g.i32(), g.i32());
        let (ae, be) = (g.range(-40, 40), g.range(-40, 40));
        let (s, t) = (g.i16(), g.i16());
        check!(
            "fIsLessThan_D",
            f_is_less_than(a, ae, b, be) as i32,
            cref_fIsLessThan_D(a, ae, b, be),
            a,
            ae,
            b,
            be
        );
        check!(
            "fIsLessThan_S",
            f_is_less_than_sgl(s, ae, t, be) as i32,
            cref_fIsLessThan_S(s, ae, t, be),
            s,
            ae,
            t,
            be
        );
        check!("CalcInvLdData", calc_inv_ld_data(a), cref_CalcInvLdData(a), a);
        let i = g.range(-5, 200);
        check!("CalcLdInt", calc_ld_int(i), cref_CalcLdInt(i), i);
        let p = a & 0x7FFF_FFFF;
        check!("sqrtFixp_lookup", sqrt_fixp_lookup(p), cref_sqrtFixp_lookup(p), p);
        let (mut e1, mut e2) = (ae, ae);
        check!(
            "sqrtFixp_lookup_e",
            sqrt_fixp_lookup_e(p, &mut e1),
            cref_sqrtFixp_lookup_e(p, &mut e2),
            p,
            ae
        );
        assert_eq!(e1, e2, "sqrtFixp_lookup_e exponent {p} {ae}");
        let (mut e1, mut e2) = (0, 0);
        check!("invSqrtNorm2", inv_sqrt_norm2(p, &mut e1), cref_invSqrtNorm2(p, &mut e2), p);
        assert_eq!(e1, e2, "invSqrtNorm2 exponent {p}");
        check!("sqrtFixp", sqrt_fixp(p), cref_sqrtFixp(p), p);
        check!("invFixp", inv_fixp(a), cref_invFixp(a), a);
        let (mut e1, mut e2) = (ae, ae);
        check!("invFixp_e", inv_fixp_e(a, &mut e1), cref_invFixp_e(a, &mut e2), a, ae);
        assert_eq!(e1, e2, "invFixp_e exponent {a} {ae}");
        let (num, den) = (f_min(p, b & 0x7FFF_FFFF), f_max(p, b & 0x7FFF_FFFF) | 1);
        check!("schur_div", schur_div(num, den, 16), cref_schur_div(num, den, 16), num, den);
        let (mut e1, mut e2) = (0, 0);
        check!("fMultNorm_e", f_mult_norm(a, b, &mut e1), cref_fMultNorm_e(a, b, &mut e2), a, b);
        assert_eq!(e1, e2, "fMultNorm_e exponent {a} {b}");
        check!("fMultNorm", f_mult_norm_noexp(a, b), cref_fMultNorm(a, b), a, b);
        let re = g.range(-40, 40);
        check!(
            "fMultNorm_to",
            f_mult_norm_to(a, ae, b, be, re),
            cref_fMultNorm_to(a, ae, b, be, re),
            a,
            ae,
            b,
            be,
            re
        );
        let n = g.i32() >> g.range(0, 31);
        check!("fMultI", f_mult_i(a, n), cref_fMultI(a, n), a, n);
        check!("fMultIfloor", f_mult_i_floor(a, n), cref_fMultIfloor(a, n), a, n);
        check!("fMultIceil", f_mult_i_ceil(a, n), cref_fMultIceil(a, n), a, n);
        let (pn, pd) = (p, (b & 0x7FFF_FFFF) | 1);
        let (mut e1, mut e2) = (0, 0);
        check!("fDivNorm_e", f_div_norm(pn, pd, &mut e1), cref_fDivNorm_e(pn, pd, &mut e2), pn, pd);
        assert_eq!(e1, e2, "fDivNorm_e exponent {pn} {pd}");
        check!("fDivNorm", f_div_norm_noexp(num, den), cref_fDivNorm(num, den), num, den);
        let (mut e1, mut e2) = (0, 0);
        check!(
            "fDivNormSigned_e",
            f_div_norm_signed(a, b, &mut e1),
            cref_fDivNormSigned_e(a, b, &mut e2),
            a,
            b
        );
        assert_eq!(e1, e2, "fDivNormSigned_e exponent {a} {b}");
        check!("fDivNormSigned", f_div_norm_signed_noexp(a, b), cref_fDivNormSigned(a, b), a, b);
        let (mut e1, mut e2) = (0, 0);
        check!(
            "fDivNormHighPrec",
            f_div_norm_high_prec(pn, pd, &mut e1),
            cref_fDivNormHighPrec(pn, pd, &mut e2),
            pn,
            pd
        );
        assert_eq!(e1, e2, "fDivNormHighPrec exponent {pn} {pd}");
        let (mut e1, mut e2) = (ae, ae);
        check!("fAdjust", f_adjust(a, &mut e1), cref_fAdjust(a, &mut e2), a, ae);
        assert_eq!(e1, e2);
        let (am, bm) = (a >> 1, b >> 1);
        let (mut e1, mut e2) = (0, 0);
        check!(
            "fAddNorm_e",
            f_add_norm(am, ae, bm, be, &mut e1),
            cref_fAddNorm_e(am, ae, bm, be, &mut e2),
            am,
            ae,
            bm,
            be
        );
        assert_eq!(e1, e2);
        let rr = f_max(ae, be) + g.range(0, 3);
        check!(
            "fAddNorm_to",
            f_add_norm_to(
                am >> 2,
                ae - 31 + (ae.abs() % 31),
                bm >> 2,
                be - 31 + (be.abs() % 31),
                rr
            ),
            cref_fAddNorm_to(
                am >> 2,
                ae - 31 + (ae.abs() % 31),
                bm >> 2,
                be - 31 + (be.abs() % 31),
                rr
            ),
            am,
            ae,
            bm,
            be,
            rr
        );
        let ee = g.range(-31, 7);
        let (mut e1, mut e2) = (0, 0);
        check!("f2Pow_e", f2_pow(a, ee, &mut e1), cref_f2Pow_e(a, ee, &mut e2), a, ee);
        assert_eq!(e1, e2, "f2Pow_e exponent");
        let ee2 = g.range(-31, 4);
        check!("f2Pow", f2_pow_noexp(a >> 6, ee2), cref_f2Pow(a >> 6, ee2), a, ee2);
        let (be2, ee3) = (g.range(-10, 10), g.range(-10, 3));
        let (mut e1, mut e2) = (0, 0);
        check!(
            "fPow",
            f_pow(a, be2, b >> 8, ee3, &mut e1),
            cref_fPow(a, be2, b >> 8, ee3, &mut e2),
            a,
            be2,
            b,
            ee3
        );
        assert_eq!(e1, e2, "fPow exponent");
        let (mut e1, mut e2) = (0, 0);
        check!(
            "fLdPow_e",
            f_ld_pow(a >> 4, 3, b >> 8, ee3, &mut e1),
            cref_fLdPow_e(a >> 4, 3, b >> 8, ee3, &mut e2),
            a,
            b,
            ee3
        );
        assert_eq!(e1, e2, "fLdPow exponent");
        check!(
            "fLdPow",
            f_ld_pow_noexp(a >> 4, 3, b >> 8, ee3),
            cref_fLdPow(a >> 4, 3, b >> 8, ee3),
            a,
            b,
            ee3
        );
        let nn = g.range(-6, 6);
        let (mut e1, mut e2) = (0, 0);
        check!(
            "fPowInt",
            f_pow_int(p, be2, nn, &mut e1),
            cref_fPowInt(p, be2, nn, &mut e2),
            p,
            be2,
            nn
        );
        assert_eq!(e1, e2, "fPowInt exponent");
        let (mut e1, mut e2) = (0, 0);
        check!("fLog2_e", f_log2(a, ae, &mut e1), cref_fLog2_e(a, ae, &mut e2), a, ae);
        assert_eq!(e1, e2, "fLog2 exponent");
        let le = g.range(-20, 20);
        check!("fLog2", f_log2_noexp(a, le), cref_fLog2(a, le), a, le);
        check!("fAddSaturate_S", f_add_saturate_sgl(s, t), cref_fAddSaturate_S(s, t), s, t);
        check!("fAddSaturate_D", f_add_saturate(a, b), cref_fAddSaturate_D(a, b), a, b);
        check!("fNegateSaturate_D", f_negate_saturate(a), cref_fNegateSaturate_D(a), a);
        check!("fNegateSaturate_S", f_negate_saturate_sgl(s), cref_fNegateSaturate_S(s), s);
        let sf = g.range(0, 31);
        check!("fixp_floorToInt", fixp_floor_to_int(a, sf), cref_fixp_floorToInt(a, sf), a, sf);
        check!("fixp_floor", fixp_floor(a, sf), cref_fixp_floor(a, sf), a, sf);
        let sfc = g.range(1, 31);
        check!("fixp_ceilToInt", fixp_ceil_to_int(a, sfc), cref_fixp_ceilToInt(a, sfc), a, sfc);
        check!("fixp_ceil", fixp_ceil(a, sfc), cref_fixp_ceil(a, sfc), a, sfc);
        check!(
            "fixp_truncateToInt",
            fixp_truncate_to_int(a, sfc),
            cref_fixp_truncateToInt(a, sfc),
            a,
            sfc
        );
        check!("fixp_truncate", fixp_truncate(a, sfc), cref_fixp_truncate(a, sfc), a, sfc);
        check!("fixp_roundToInt", fixp_round_to_int(a, sf), cref_fixp_roundToInt(a, sf), a, sf);
        check!("fixp_round", fixp_round(a, sfc), cref_fixp_round(a, sfc), a, sfc);
        let ii = g.range(-3, 90);
        check!("GetInvInt", get_inv_int(ii), cref_GetInvInt(ii), ii);
    }
}

#[test]
fn add_saturate_edge_cases() {
    let vals = [
        i32::MIN,
        i32::MIN + 1,
        i32::MIN / 2,
        -2,
        -1,
        0,
        1,
        2,
        i32::MAX / 2,
        i32::MAX / 2 + 1,
        i32::MAX - 1,
        i32::MAX,
    ];
    for &a in &vals {
        for &b in &vals {
            assert_eq!(f_add_saturate(a, b), unsafe { cref_fAddSaturate_D(a, b) }, "{a} {b}");
        }
    }
}
