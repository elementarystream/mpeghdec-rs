// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of FDK_lpc.h for all LPC orders.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_lpc::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::lpc::*;

const ORDERS: core::ops::RangeInclusive<i32> = 1..=LPC_MAX_ORDER as i32;

fn vec_i32(g: &mut Rng, n: usize, shift: u32) -> Vec<i32> {
    (0..n).map(|_| g.i32() >> shift).collect()
}

/// Random reflection coefficient with |k| < 1.
fn refl_sgl(g: &mut Rng) -> i16 {
    ((g.i16() as i32 * 15) / 16) as i16
}

fn refl_dbl(g: &mut Rng) -> i32 {
    ((g.i32() as i64 * 15) / 16) as i32
}

#[test]
fn synthesis_lattice() {
    let mut g = Rng::new(31);
    for order in ORDERS {
        for iter in 0..200 {
            let coeff_s: Vec<i16> = (0..order).map(|_| refl_sgl(&mut g)).collect();
            let coeff_d: Vec<i32> = (0..order).map(|_| refl_dbl(&mut g)).collect();
            let mut st_sr = vec![0i32; order as usize];
            let mut st_sc = st_sr.clone();
            let mut st_dr = vec_i32(&mut g, order as usize, 8);
            let mut st_dc = st_dr.clone();
            let signal_e = g.range(-3, 3);
            let signal_e_out = g.range(-3, 3);
            for _block in 0..3 {
                // also longer than the 256 values history window of the fast path
                let size = if g.range(0, 3) == 0 { g.range(200, 1100) } else { g.range(0, 200) };
                let size = size as usize;
                let inc = if g.range(0, 1) == 0 { 1 } else { -1 };
                let shift = g.range(3, 10) as u32;
                let sig = vec_i32(&mut g, size, shift);
                let what = format!(
                    "order {order} iter {iter} size {size} inc {inc} e {signal_e}/{signal_e_out}"
                );

                let mut r = sig.clone();
                let mut c = sig.clone();
                clpc_synthesis_lattice_sgl(
                    &mut r,
                    signal_e,
                    signal_e_out,
                    inc,
                    &coeff_s,
                    order,
                    &mut st_sr,
                );
                unsafe {
                    cref_CLpc_SynthesisLattice_SGL(
                        c.as_mut_ptr(),
                        size as i32,
                        signal_e,
                        signal_e_out,
                        inc,
                        coeff_s.as_ptr(),
                        order,
                        st_sc.as_mut_ptr(),
                    )
                };
                assert_eq!(r, c, "lattice SGL {what}");
                assert_eq!(st_sr, st_sc, "lattice SGL state {what}");

                let mut r = sig.clone();
                let mut c = sig.clone();
                clpc_synthesis_lattice(
                    &mut r,
                    signal_e,
                    signal_e_out,
                    inc,
                    &coeff_d,
                    order,
                    &mut st_dr,
                );
                unsafe {
                    cref_CLpc_SynthesisLattice_DBL(
                        c.as_mut_ptr(),
                        size as i32,
                        signal_e,
                        signal_e_out,
                        inc,
                        coeff_d.as_ptr(),
                        order,
                        st_dc.as_mut_ptr(),
                    )
                };
                assert_eq!(r, c, "lattice DBL {what}");
                assert_eq!(st_dr, st_dc, "lattice DBL state {what}");
            }
        }
    }
}

#[test]
fn parcor_to_lpc_and_synthesis() {
    let mut g = Rng::new(32);
    for order in ORDERS {
        for iter in 0..300 {
            let n = order as usize;
            // ParCor -> LPC (DBL and SGL)
            let refl_d: Vec<i32> = (0..n).map(|_| refl_dbl(&mut g)).collect();
            let refl_s: Vec<i16> = (0..n).map(|_| refl_sgl(&mut g)).collect();
            let mut lpc_dr = vec![0i32; n];
            let mut lpc_dc = vec![0i32; n];
            let mut lpc_sr = vec![0i16; n];
            let mut lpc_sc = vec![0i16; n];
            let mut wr = vec![0i64; n];
            let mut wc = vec![0i64; n];
            let er = clpc_parcor_to_lpc(&refl_d, &mut lpc_dr, order, &mut wr);
            let ec = unsafe {
                cref_CLpc_ParcorToLpc_DBL(
                    refl_d.as_ptr(),
                    lpc_dc.as_mut_ptr(),
                    order,
                    wc.as_mut_ptr(),
                )
            };
            assert_eq!((er, &lpc_dr, &wr), (ec, &lpc_dc, &wc), "ParcorToLpc DBL order {order}");
            let es = clpc_parcor_to_lpc_sgl(&refl_s, &mut lpc_sr, order, &mut wr);
            let esc = unsafe {
                cref_CLpc_ParcorToLpc_SGL(
                    refl_s.as_ptr(),
                    lpc_sc.as_mut_ptr(),
                    order,
                    wc.as_mut_ptr(),
                )
            };
            assert_eq!((es, &lpc_sr, &wr), (esc, &lpc_sc, &wc), "ParcorToLpc SGL order {order}");

            // IIR synthesis with these coefficients
            let mut st_dr = if iter % 2 == 0 { vec![0i32; n] } else { vec_i32(&mut g, n, 12) };
            let mut st_dc = st_dr.clone();
            let mut st_sr = vec_i32(&mut g, n, 12);
            let mut st_sc = st_sr.clone();
            let mut idx_dr = g.range(0, order - 1);
            let mut idx_dc = idx_dr;
            let mut idx_sr = g.range(0, order - 1);
            let mut idx_sc = idx_sr;
            for _block in 0..3 {
                // also longer than the 256 values history window of the fast path
                let size = if g.range(0, 3) == 0 { g.range(200, 1100) } else { g.range(0, 200) };
                let size = size as usize;
                let inc = if g.range(0, 1) == 0 { 1 } else { -1 };
                let shift = g.range(4, 12) as u32;
                let sig = vec_i32(&mut g, size, shift);
                let what = format!("order {order} iter {iter} size {size} inc {inc}");

                let mut r = sig.clone();
                let mut c = sig.clone();
                clpc_synthesis(&mut r, inc, &lpc_dr, er, order, &mut st_dr, &mut idx_dr);
                unsafe {
                    cref_CLpc_Synthesis_DBL(
                        c.as_mut_ptr(),
                        size as i32,
                        inc,
                        lpc_dc.as_ptr(),
                        ec,
                        order,
                        st_dc.as_mut_ptr(),
                        &mut idx_dc,
                    )
                };
                assert_eq!((&r, &st_dr, idx_dr), (&c, &st_dc, idx_dc), "Synthesis DBL {what}");

                let mut r = sig.clone();
                let mut c = sig.clone();
                clpc_synthesis_sgl(&mut r, inc, &lpc_sr, es, order, &mut st_sr, &mut idx_sr);
                unsafe {
                    cref_CLpc_Synthesis_SGL(
                        c.as_mut_ptr(),
                        size as i32,
                        inc,
                        lpc_sc.as_ptr(),
                        esc,
                        order,
                        st_sc.as_mut_ptr(),
                        &mut idx_sc,
                    )
                };
                assert_eq!((&r, &st_sr, idx_sr), (&c, &st_sc, idx_sc), "Synthesis SGL {what}");
            }
        }
    }
}

#[test]
fn autocorr_and_levinson() {
    let mut g = Rng::new(33);
    for order in ORDERS {
        for iter in 0..300 {
            let m = order as usize;
            let size = g.range(1, 512) as usize;
            let shift = g.range(0, 20) as u32;
            let mut sig = vec_i32(&mut g, size, shift);
            if iter % 50 == 0 {
                sig.fill(0);
            }
            let signal_e = g.range(-4, 4);
            let what = format!("order {order} iter {iter} size {size}");

            // Autocorrelation with m + 1 lags
            let mut ar = vec![0i32; m + 1];
            let mut ac = ar.clone();
            let (mut aer, mut aec) = (0, 0);
            let nr = clpc_auto_corr(&sig, signal_e, &mut ar, &mut aer, order + 1);
            let nc = unsafe {
                cref_CLpc_AutoCorr(
                    sig.as_ptr(),
                    signal_e,
                    ac.as_mut_ptr(),
                    &mut aec,
                    size as i32,
                    order + 1,
                )
            };
            assert_eq!((nr, &ar, aer), (nc, &ac, aec), "AutoCorr {what}");

            // Schur
            for use_gain in 0..2 {
                let mut acr = ar.clone();
                let mut acc = ar.clone();
                let mut rr = vec![0x55i16; m];
                let mut rc = rr.clone();
                let (mut gm, mut ge) = (0x1234, 77);
                let mut gc = [gm, ge];
                let gain = if use_gain == 1 { Some((&mut gm, &mut ge)) } else { None };
                clpc_auto_to_parcor(&mut acr, aer, &mut rr, order, gain);
                unsafe {
                    cref_CLpc_AutoToParcor(
                        acc.as_mut_ptr(),
                        aec,
                        rc.as_mut_ptr(),
                        order,
                        gc.as_mut_ptr(),
                        use_gain,
                    )
                };
                assert_eq!(
                    (&acr, &rr, [gm, ge]),
                    (&acc, &rc, gc),
                    "AutoToParcor {what} gain {use_gain}"
                );
            }

            // Levinson-Durbin
            let r_e = g.range(-4, 4);
            let mut lr = vec![0x5555i32; m];
            let mut lc = lr.clone();
            let (mut er, mut ec) = (99, 99);
            clpc_auto_to_lpc_igf(&mut lr, &mut er, &ar, r_e, order, None);
            unsafe { cref_CLpc_AutoToLpcIGF(lc.as_mut_ptr(), &mut ec, ar.as_ptr(), r_e, order) };
            assert_eq!((&lr, er), (&lc, ec), "AutoToLpcIGF {what}");

            for is_ltpf in 0..2 {
                let mut lr = vec![0x555i16; m];
                let mut lc = lr.clone();
                let (mut er, mut ec) = (99, 99);
                clpc_auto_to_lpc(&mut lr, &mut er, &ar, r_e, order, None, is_ltpf);
                unsafe {
                    cref_CLpc_AutoToLpc(lc.as_mut_ptr(), &mut ec, ar.as_ptr(), r_e, order, is_ltpf)
                };
                assert_eq!((&lr, er), (&lc, ec), "AutoToLpc {what} ltpf {is_ltpf}");
            }
        }
    }
}

#[test]
fn levinson_random_autocorrelation() {
    let mut g = Rng::new(34);
    for order in ORDERS {
        for iter in 0..500 {
            let m = order as usize;
            let shift = g.range(0, 8) as u32;
            let mut r: Vec<i32> = vec_i32(&mut g, m + 1, shift);
            if iter % 2 == 0 {
                r[0] = r[0].wrapping_abs();
            }
            let r_e = g.range(-4, 4);
            let what = format!("order {order} iter {iter}");
            let mut lr = vec![0x5555i32; m];
            let mut lc = lr.clone();
            let (mut er, mut ec) = (99, 99);
            clpc_auto_to_lpc_igf(&mut lr, &mut er, &r, r_e, order, None);
            unsafe { cref_CLpc_AutoToLpcIGF(lc.as_mut_ptr(), &mut ec, r.as_ptr(), r_e, order) };
            assert_eq!((&lr, er), (&lc, ec), "AutoToLpcIGF {what}");
            for is_ltpf in 0..2 {
                let mut lr = vec![0x555i16; m];
                let mut lc = lr.clone();
                let (mut er, mut ec) = (99, 99);
                clpc_auto_to_lpc(&mut lr, &mut er, &r, r_e, order, None, is_ltpf);
                unsafe {
                    cref_CLpc_AutoToLpc(lc.as_mut_ptr(), &mut ec, r.as_ptr(), r_e, order, is_ltpf)
                };
                assert_eq!((&lr, er), (&lc, ec), "AutoToLpc {what} ltpf {is_ltpf}");
            }
        }
    }
}
