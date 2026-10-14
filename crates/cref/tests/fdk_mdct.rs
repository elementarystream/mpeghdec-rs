// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of mdct.h: decoder-like frame sequences with all window sequences, window
//! shapes, aliasing symmetries and transform splitting are run through the C and the Rust IMLT,
//! comparing output, spectrum, overlap buffer and state after every call.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_mdct::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::mdct::*;
use mpeghdec_fdk::tools_rom::*;
use mpeghdec_fdk::FixpSpk;

const OV_SIZE: i32 = 512;
const FRAME_LEN: i32 = 1024;

fn rust_tables() -> [&'static [FixpSpk]; 11] {
    [
        &SINE_TABLE_512,
        &SINE_TABLE_1024,
        &SINE_WINDOW_128,
        &SINE_WINDOW_256,
        &SINE_WINDOW_512,
        &SINE_WINDOW_1024,
        &KBD_WINDOW_64,
        &KBD_WINDOW_128,
        &KBD_WINDOW_256,
        &KBD_WINDOW_1024,
        &SIN_TWIDDLE_L64,
    ]
}

fn table_id(t: Option<&[FixpSpk]>) -> i32 {
    match t {
        None => -1,
        Some(t) => rust_tables()
            .iter()
            .position(|r| core::ptr::eq(r.as_ptr(), t.as_ptr()))
            .map_or(-2, |i| i as i32),
    }
}

struct Pair {
    c: *mut (),
    r: MdctT,
}

impl Pair {
    fn new() -> Self {
        let c = unsafe { cref_mdct_new() };
        unsafe { cref_mdct_init(c, OV_SIZE) };
        let mut r = MdctT::default();
        mdct_init(&mut r, OV_SIZE);
        Pair { c, r }
    }

    fn check(&self, what: &str) {
        let mut st = [0i32; 9];
        unsafe { cref_mdct_state(self.c, st.as_mut_ptr()) };
        let r = &self.r;
        let rs = [
            r.prev_tl,
            r.prev_nr,
            r.prev_fr,
            r.ov_offset,
            r.ov_size,
            r.prev_alias_symmetry,
            r.prev_prev_alias_symmetry,
            table_id(r.prev_wrs),
            r.p_asym_ovlp.is_some() as i32,
        ];
        assert_eq!(
            rs, st,
            "{what}: state [prev_tl, prev_nr, prev_fr, ov_offset, ov_size, pas, ppas, wrs, asym]"
        );
        let ovl =
            unsafe { core::slice::from_raw_parts(cref_mdct_overlap(self.c), OV_SIZE as usize) };
        assert_eq!(&r.overlap[..], ovl, "{what}: overlap");
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe { cref_mdct_free(self.c) };
    }
}

#[test]
fn imdct_gain_all_lengths() {
    let mut g = Rng::new(21);
    let mut lengths = vec![0];
    for k in 2..=11 {
        lengths.push(1 << k);
    }
    lengths.extend_from_slice(&[80, 96, 120, 160, 192, 240, 320, 384, 480, 640, 768, 960, 1920]);
    for &tl in &lengths {
        for i in 0..1000 {
            let m0 = if i % 4 == 0 { 0 } else { g.i32() };
            let e0 = g.range(-20, 20);
            let (mut mr, mut er) = (m0, e0);
            let (mut mc, mut ec) = (m0, e0);
            imdct_gain(&mut mr, &mut er, tl);
            unsafe { cref_imdct_gain(&mut mc, &mut ec, tl) };
            assert_eq!((mr, er), (mc, ec), "imdct_gain({m0}, {e0}, {tl})");
        }
    }
}

/// Window parameters of a frame as determined by CBlock_FrequencyToTime().
struct FrameCfg {
    tl: i32,
    n_spec: i32,
    fl: i32,
    fr: i32,
    shape: i32,
}

fn frame_cfg(g: &mut Rng, prev_tl: i32, prev_fr: i32, seq: i32, shape: i32) -> FrameCfg {
    let frame_len = FRAME_LEN;
    let mut tl = frame_len;
    let mut n_spec = 1;
    let (fl, fr);
    match seq {
        BLOCK_START => {
            // Transform splitting after a low overlap window makes the second window adapt the
            // slope length without a matching window table (out of bounds read in C++).
            if g.range(0, 3) == 0 && prev_fr != 256 {
                // transform splitting
                tl >>= 1;
                n_spec = 2;
                fl = frame_len >> 3;
                fr = frame_len >> 3;
            } else {
                fl = frame_len;
                fr = frame_len >> 3;
            }
        }
        BLOCK_SHORT => {
            fl = frame_len >> 3;
            fr = frame_len >> 3;
            tl >>= 3;
            n_spec = 8;
        }
        BLOCK_STOP => {
            fl = frame_len >> 3;
            fr = frame_len;
        }
        _ => {
            let nr2 = if shape == 2 { (frame_len * 3) >> 2 } else { 0 };
            fr = frame_len - nr2;
            fl = if prev_tl == 0 { fr } else { frame_len };
        }
    }
    FrameCfg { tl, n_spec, fl, fr, shape }
}

fn run_sequence(seed: u64, frames: usize) {
    let mut g = Rng::new(seed);
    let mut p = Pair::new();
    p.check("init");

    let mut seq = BLOCK_LONG;
    for frame in 0..frames {
        // Mostly valid window sequence transitions, sometimes arbitrary ones.
        seq = if g.range(0, 5) == 0 {
            g.range(0, 3)
        } else {
            match seq {
                BLOCK_LONG | BLOCK_STOP => [BLOCK_LONG, BLOCK_START][g.range(0, 1) as usize],
                _ => [BLOCK_SHORT, BLOCK_STOP][g.range(0, 1) as usize],
            }
        };
        let shape = g.range(0, 2);
        let cfg = frame_cfg(&mut g, p.r.prev_tl, p.r.prev_fr, seq, shape);
        let what = format!(
            "seed {seed} frame {frame} seq {seq} shape {shape} tl {} fl {} fr {}",
            cfg.tl, cfg.fl, cfg.fr
        );

        // Aliasing symmetry (MPEG-H multichannel coding tool)
        let mut flags = 0;
        if g.range(0, 3) == 0 {
            flags = g.range(0, 1);
            if g.range(0, 3) == 0 {
                // prev_aliasing_symmetry read from the bitstream (independent frame)
                let pas = g.range(0, 1);
                let ov = p.r.ov_offset;
                let ppas = p.r.prev_prev_alias_symmetry;
                p.r.prev_alias_symmetry = pas;
                unsafe { cref_mdct_set(p.c, pas, ppas, ov) };
            }
        }

        // Asymmetric overlap
        let mut asym_c: Vec<i32> = Vec::new();
        if g.range(0, 7) == 0 {
            asym_c = (0..FRAME_LEN as usize / 2).map(|_| g.i32() >> g.range(0, 8)).collect();
            p.r.p_asym_ovlp = Some(asym_c.clone());
            unsafe { cref_mdct_set_asym(p.c, asym_c.as_mut_ptr()) };
        }

        let n = (cfg.tl * cfg.n_spec) as usize;
        let spec_shift = g.range(0, 6) as u32;
        let mut spec_r: Vec<i32> = (0..n).map(|_| g.i32() >> spec_shift).collect();
        if g.range(0, 10) == 0 {
            spec_r.fill(0);
        }
        let mut spec_c = spec_r.clone();
        let sf: Vec<i16> = (0..cfg.n_spec).map(|_| g.range(-4, 12) as i16).collect();
        let gain = if g.range(0, 4) == 0 { g.i32() } else { 0 };
        let mut out_r = vec![0x5555_5555i32; 2 * FRAME_LEN as usize];
        let mut out_c = out_r.clone();

        let wls_r = fdk_get_window_slope(cfg.fl, cfg.shape);
        let wrs_r = fdk_get_window_slope(cfg.fr, cfg.shape);
        let variant = g.range(0, 5);
        if std::env::var("MDCT_DEBUG").is_ok() {
            eprintln!(
                "{what} variant {variant} flags {flags} state {:?}",
                (
                    p.r.prev_tl,
                    p.r.prev_nr,
                    p.r.prev_fr,
                    p.r.ov_offset,
                    p.r.prev_alias_symmetry,
                    p.r.prev_prev_alias_symmetry,
                    p.r.prev_wrs.map(|w| w.len())
                )
            );
        }
        let (nr, nc) = unsafe {
            match variant {
                0 => (
                    imdct_block(
                        &mut p.r,
                        &mut out_r,
                        &mut spec_r,
                        &sf,
                        cfg.n_spec,
                        FRAME_LEN,
                        cfg.tl,
                        wls_r,
                        cfg.fl,
                        wrs_r,
                        cfg.fr,
                        gain,
                    ),
                    cref_imdct_block(
                        p.c,
                        out_c.as_mut_ptr(),
                        spec_c.as_mut_ptr(),
                        sf.as_ptr(),
                        cfg.n_spec,
                        FRAME_LEN,
                        cfg.tl,
                        cfg.fl,
                        cfg.shape,
                        cfg.fl,
                        cfg.fr,
                        cfg.shape,
                        cfg.fr,
                        gain,
                    ),
                ),
                1 => (
                    imdst_block(
                        &mut p.r,
                        &mut out_r,
                        &mut spec_r,
                        &sf,
                        cfg.n_spec,
                        FRAME_LEN,
                        cfg.tl,
                        wls_r,
                        cfg.fl,
                        wrs_r,
                        cfg.fr,
                        gain,
                    ),
                    cref_imdst_block(
                        p.c,
                        out_c.as_mut_ptr(),
                        spec_c.as_mut_ptr(),
                        sf.as_ptr(),
                        cfg.n_spec,
                        FRAME_LEN,
                        cfg.tl,
                        cfg.fl,
                        cfg.shape,
                        cfg.fl,
                        cfg.fr,
                        cfg.shape,
                        cfg.fr,
                        gain,
                    ),
                ),
                _ => (
                    imlt_block(
                        &mut p.r,
                        &mut out_r,
                        &mut spec_r,
                        &sf,
                        cfg.n_spec,
                        FRAME_LEN,
                        cfg.tl,
                        wls_r,
                        cfg.fl,
                        wrs_r,
                        cfg.fr,
                        gain,
                        flags,
                    ),
                    cref_imlt_block(
                        p.c,
                        out_c.as_mut_ptr(),
                        spec_c.as_mut_ptr(),
                        sf.as_ptr(),
                        cfg.n_spec,
                        FRAME_LEN,
                        cfg.tl,
                        cfg.fl,
                        cfg.shape,
                        cfg.fl,
                        cfg.fr,
                        cfg.shape,
                        cfg.fr,
                        gain,
                        flags,
                    ),
                ),
            }
        };
        assert_eq!(nr, nc, "{what}: returned samples");
        assert_eq!(out_r, out_c, "{what}: output");
        assert_eq!(spec_r, spec_c, "{what}: spectrum");

        // Unused asymmetric overlap: reset on both sides before the buffer goes away.
        let mut st = [0i32; 9];
        unsafe { cref_mdct_state(p.c, st.as_mut_ptr()) };
        assert_eq!(p.r.p_asym_ovlp.is_some() as i32, st[8], "{what}: asym");
        p.r.p_asym_ovlp = None;
        unsafe { cref_mdct_set_asym(p.c, core::ptr::null_mut()) };
        drop(asym_c);
        p.check(&what);

        // Read-only access to the overlap
        let nreq = g.range(0, 2 * FRAME_LEN);
        let mut buf_r = vec![0x1234_5678i32; 3 * FRAME_LEN as usize];
        let mut buf_c = buf_r.clone();
        let (a, b) = unsafe {
            (
                imdct_copy_ov_and_nr(&p.r, &mut buf_r, nreq),
                cref_imdct_copy_ov_and_nr(p.c, buf_c.as_mut_ptr(), nreq),
            )
        };
        assert_eq!((a, &buf_r), (b, &buf_c), "{what}: imdct_copy_ov_and_nr({nreq})");
        if p.r.prev_tl <= 2 * OV_SIZE {
            let (a, b) = unsafe {
                (
                    imdct_copy_full(&p.r, &mut buf_r, nreq),
                    cref_imdct_copy_full(p.c, buf_c.as_mut_ptr(), nreq),
                )
            };
            assert_eq!((a, &buf_r), (b, &buf_c), "{what}: imdct_copy_full({nreq})");
        }
        if g.range(0, 10) == 0 {
            let room = g.range(-1, 2 * FRAME_LEN);
            if room <= 0 || p.r.ov_offset <= room {
                let (a, b) = unsafe {
                    (
                        imdct_drain(&mut p.r, &mut buf_r, room),
                        cref_imdct_drain(p.c, buf_c.as_mut_ptr(), room),
                    )
                };
                assert_eq!((a, &buf_r), (b, &buf_c), "{what}: imdct_drain({room})");
                p.check(&what);
            }
        }
        if g.range(0, 40) == 0 {
            // Re-initialization (keeps the overlap contents)
            mdct_init(&mut p.r, OV_SIZE);
            unsafe { cref_mdct_init(p.c, OV_SIZE) };
            p.check(&what);
        }
    }
}

#[test]
fn imlt_sequences() {
    for seed in 0..200 {
        run_sequence(seed, 60);
    }
}

#[test]
fn adapt_parameters() {
    let mut g = Rng::new(22);
    for i in 0..20_000 {
        let p = Pair::new();
        let mut p = p;
        // Establish a previous state with a random frame.
        let shape = g.range(0, 1);
        if i % 5 != 0 {
            let fr_prev = [128, 256, 512, 1024][g.range(0, 3) as usize];
            let prev_nr = g.range(-64, 512);
            // Use adapt_parameters itself to set up prev_tl/prev_fr/prev_nr.
            let (mut fl, mut nl) = (fr_prev, 0);
            let w = fdk_get_window_slope(fr_prev, shape);
            imdct_adapt_parameters(&mut p.r, &mut fl, &mut nl, 1024, w, 2 * prev_nr + fr_prev);
            let (mut flc, mut nlc) = (fr_prev, 0);
            unsafe {
                cref_imdct_adapt_parameters(
                    p.c,
                    &mut flc,
                    &mut nlc,
                    1024,
                    fr_prev,
                    shape,
                    2 * prev_nr + fr_prev,
                )
            };
            assert_eq!((fl, nl), (flc, nlc));
            p.check("setup");
        }
        let fl0 = [128, 256, 512, 1024][g.range(0, 3) as usize];
        let nl0 = g.range(-64, 512);
        let no_out = [1024, 512, 2048][g.range(0, 2) as usize];
        let w = fdk_get_window_slope(fl0, shape);
        let (mut fl, mut nl) = (fl0, nl0);
        imdct_adapt_parameters(&mut p.r, &mut fl, &mut nl, 1024, w, no_out);
        let (mut flc, mut nlc) = (fl0, nl0);
        unsafe { cref_imdct_adapt_parameters(p.c, &mut flc, &mut nlc, 1024, fl0, shape, no_out) };
        assert_eq!((fl, nl), (flc, nlc), "adapt({fl0}, {nl0}, {no_out})");
        p.check("adapt");
    }
}

#[test]
fn mdct_forward() {
    let mut g = Rng::new(23);
    for iter in 0..300 {
        let mut p = Pair::new();
        let mut seq = BLOCK_LONG;
        for frame in 0..8 {
            // mdct_block requires matching window slopes (valid window sequence transitions).
            seq = match seq {
                BLOCK_LONG | BLOCK_STOP => [BLOCK_LONG, BLOCK_START][g.range(0, 1) as usize],
                _ => [BLOCK_SHORT, BLOCK_STOP][g.range(0, 1) as usize],
            };
            let shape = g.range(0, 1);
            let (tl, n_spec, fr) = match seq {
                BLOCK_SHORT => (128, 8, 128),
                BLOCK_START => (1024, 1, 128),
                _ => (1024, 1, 1024),
            };
            let no_in = if g.range(0, 1) == 0 { tl } else { 2 * tl };
            let len = ((no_in - tl) >> 1) + (n_spec + 1) * tl;
            let shift = g.range(0, 8) as u32;
            let time: Vec<i32> = (0..len).map(|_| g.i32() >> shift).collect();
            let mut out_r = vec![0i32; (n_spec * tl) as usize];
            let mut out_c = out_r.clone();
            let mut e_r = vec![0i16; n_spec as usize];
            let mut e_c = e_r.clone();
            let wrs = fdk_get_window_slope(fr, shape);
            let a = mdct_block(&mut p.r, &time, no_in, &mut out_r, n_spec, tl, wrs, fr, &mut e_r);
            let b = unsafe {
                cref_mdct_block(
                    p.c,
                    time.as_ptr(),
                    no_in,
                    out_c.as_mut_ptr(),
                    n_spec,
                    tl,
                    fr,
                    shape,
                    fr,
                    e_c.as_mut_ptr(),
                )
            };
            let what = format!("iter {iter} frame {frame} seq {seq}");
            assert_eq!(a, b, "{what}");
            assert_eq!(e_r, e_c, "{what}: exponents");
            assert_eq!(out_r, out_c, "{what}: spectrum");
            p.check(&what);
        }
    }
}
