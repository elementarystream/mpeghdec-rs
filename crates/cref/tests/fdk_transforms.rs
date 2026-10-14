// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of FDK_tools_rom, FDK_trigFcts, scramble, fft_rad2, fft, rfft and dct.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_transforms::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::dct::*;
use mpeghdec_fdk::fft::*;
use mpeghdec_fdk::fft_rad2::{dit_fft, dit_fft_generic};
use mpeghdec_fdk::rfft::*;
use mpeghdec_fdk::scramble::scramble;
use mpeghdec_fdk::tools_rom::*;
use mpeghdec_fdk::trig_fcts::*;
use mpeghdec_fdk::FixpSpk;

/// The tables in the order of `cref_rom_table()`.
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

#[test]
fn rom_tables() {
    for (id, t) in rust_tables().iter().enumerate() {
        let p = unsafe { cref_rom_table(id as i32) };
        assert!(!p.is_null());
        let c = unsafe { core::slice::from_raw_parts(p, 2 * t.len()) };
        for (i, w) in t.iter().enumerate() {
            assert_eq!((w.re, w.im), (c[2 * i], c[2 * i + 1]), "table {id} entry {i}");
        }
    }
}

#[test]
fn window_slopes() {
    for shape in 0..2 {
        for raster in 0..4 {
            for idx in 0..10 {
                let c = unsafe { cref_windowSlopes(shape, raster, idx) };
                let r = table_id(WINDOW_SLOPES[shape as usize][raster as usize][idx as usize]);
                assert_eq!(r, c, "windowSlopes[{shape}][{raster}][{idx}]");
            }
        }
    }
    let mut lengths = vec![];
    for k in 3..=11 {
        lengths.push(1 << k);
    }
    lengths.extend_from_slice(&[96, 120, 160, 192, 240, 320, 384, 480, 640, 768, 960, 1536, 1920]);
    for shape in 0..4 {
        for &len in &lengths {
            if shape & 1 == 1 && len < 64 {
                continue; // out of bounds table access in C++
            }
            let c = unsafe { cref_FDKgetWindowSlope(len, shape) };
            let r = table_id(fdk_get_window_slope_opt(len, shape));
            assert_eq!(r, c, "FDKgetWindowSlope({len}, {shape})");
            if r >= 0 {
                assert_eq!(table_id(Some(fdk_get_window_slope(len, shape))), c);
            }
        }
    }
}

fn compare_element_list(r: Option<&'static ElementList>, c: *mut (), path: &str) {
    match r {
        None => assert!(c.is_null(), "{path}: Rust None, C non-NULL"),
        Some(r) => {
            assert!(!c.is_null(), "{path}: Rust Some, C NULL");
            let mut i = 0;
            loop {
                let cid = unsafe { cref_el_id(c, i as i32) };
                let rid = r.id[i] as i32;
                assert_eq!(rid, cid, "{path}: id[{i}]");
                if r.id[i] == RbdId::end_of_sequence || r.id[i] == RbdId::link_sequence {
                    assert_eq!(i + 1, r.id.len(), "{path}: list length");
                    break;
                }
                i += 1;
            }
            for k in 0..2 {
                let cn = unsafe { cref_el_next(c, k as i32) };
                compare_element_list(r.next[k], cn, &format!("{path}.next[{k}]"));
            }
        }
    }
}

#[test]
fn element_lists() {
    for aot in [AOT_AAC_LC, AOT_USAC, AOT_MPEGH3DA, AOT_ER_AAC_ELD] {
        for n_channels in 1..=2u8 {
            for el_flags in [0u32, AC_EL_USAC_LFE, AC_EL_USAC_NOISE] {
                if el_flags == AC_EL_USAC_LFE && n_channels != 1 {
                    continue; // FDK_ASSERT
                }
                let r = get_bitstream_element_list(aot, 0, n_channels, 0, el_flags);
                let c =
                    unsafe { cref_getBitstreamElementList(aot, 0, n_channels as i32, 0, el_flags) };
                compare_element_list(r, c, &format!("aot {aot} ch {n_channels} flags {el_flags}"));
            }
        }
    }
}

#[test]
fn trig() {
    let mut g = Rng::new(7);
    for scale in 0..=20 {
        for _ in 0..20_000 {
            let x = g.i32();
            let x2 = g.i32();
            assert_eq!(
                fixp_cos(x, scale),
                unsafe { cref_fixp_cos(x, scale) },
                "fixp_cos({x}, {scale})"
            );
            assert_eq!(
                fixp_sin(x, scale),
                unsafe { cref_fixp_sin(x, scale) },
                "fixp_sin({x}, {scale})"
            );
            let mut c = [0i32; 4];
            unsafe { cref_fixp_cos_sin(c.as_mut_ptr(), x, scale) };
            let (rc, rs) = fixp_cos_sin(x, scale);
            assert_eq!([rc, rs], [c[0], c[1]], "fixp_cos_sin({x}, {scale})");
            unsafe { cref_inline_fixp_cos_sin(c.as_mut_ptr(), x, x2, scale) };
            assert_eq!(
                inline_fixp_cos_sin(x, x2, scale),
                c,
                "inline_fixp_cos_sin({x}, {x2}, {scale})"
            );
        }
    }
}

fn random_vec(g: &mut Rng, n: usize, shift: u32) -> Vec<i32> {
    (0..n).map(|_| g.i32() >> shift).collect()
}

#[test]
fn scramble_all_lengths() {
    let mut g = Rng::new(11);
    for k in 1..=11 {
        let n = 1i32 << k;
        for _ in 0..4 {
            let mut r = random_vec(&mut g, 2 * n as usize, 0);
            let mut c = r.clone();
            scramble(&mut r, n);
            unsafe { cref_scramble(c.as_mut_ptr(), n) };
            assert_eq!(r, c, "scramble({n})");
        }
        // It must be the bit reversal permutation.
        let mut x: Vec<i32> = (0..2 * n).collect();
        scramble(&mut x, n);
        for m in 0..n as u32 {
            let rev = m.reverse_bits() >> (32 - k);
            assert_eq!(x[2 * rev as usize], 2 * m as i32, "scramble({n}) index {m}");
        }
    }
}

#[test]
fn dit_fft_all_lengths() {
    let mut g = Rng::new(12);
    for (table, tab, size) in [(0, &SINE_TABLE_512[..], 512), (1, &SINE_TABLE_1024[..], 1024)] {
        for ldn in 3..=10 {
            for iter in 0..50 {
                let n = 1usize << ldn;
                let mut r = random_vec(&mut g, 2 * n, [0, 1, 4][iter % 3]);
                let mut c = r.clone();
                dit_fft(&mut r, ldn, tab, size);
                unsafe { cref_dit_fft(c.as_mut_ptr(), ldn, table) };
                assert_eq!(r, c, "dit_fft(ldn {ldn}, table {table})");
            }
            // Constant and alternating extreme inputs (overflow and wrap-around of every stage).
            let n = 1usize << ldn;
            for pattern in 0..6 {
                let mut r: Vec<i32> = (0..2 * n)
                    .map(|i| match pattern {
                        0 => i32::MIN,
                        1 => i32::MAX,
                        2 => [i32::MIN, i32::MAX][i % 2],
                        3 => [i32::MIN, i32::MAX][(i / 2) % 2],
                        4 => [i32::MIN, i32::MIN, i32::MAX, i32::MAX, -1, 0][i % 6],
                        _ => [i32::MIN, -1, 1, i32::MAX][(i * 7 + i / 3) % 4],
                    })
                    .collect();
                let mut c = r.clone();
                dit_fft(&mut r, ldn, tab, size);
                unsafe { cref_dit_fft(c.as_mut_ptr(), ldn, table) };
                assert_eq!(r, c, "dit_fft(ldn {ldn}, table {table}) pattern {pattern}");
            }
        }
    }
    // The vectorized and the generic implementation on many random inputs.
    for ldn in 3..=10 {
        let n = 1usize << ldn;
        for iter in 0..2000 {
            let mut r = random_vec(&mut g, 2 * n, [0, 1, 4][iter % 3]);
            let mut c = r.clone();
            dit_fft(&mut r, ldn, &SINE_TABLE_512, 512);
            dit_fft_generic(&mut c, ldn, &SINE_TABLE_512, 512);
            assert_eq!(r, c, "dit_fft(ldn {ldn}) vs generic");
        }
    }
}

#[test]
fn fft_all_lengths() {
    let mut g = Rng::new(13);
    for length in [16, 32, 64, 128, 256, 512, 1024] {
        let supported = (64..=512).contains(&length);
        if !supported && cfg!(debug_assertions) {
            continue; // FDK_ASSERT
        }
        for iter in 0..100 {
            let mut r = random_vec(&mut g, 2 * length as usize, iter % 3);
            let mut c = r.clone();
            let mut sr = g.range(-10, 10);
            let mut sc = sr;
            let sc0 = sc;
            fft(length, &mut r, &mut sr);
            unsafe { cref_fft(length, c.as_mut_ptr(), &mut sc) };
            assert_eq!(r, c, "fft({length})");
            assert_eq!(sr, sc, "fft({length}) scale");
            if supported {
                assert_ne!(sr, sc0, "fft({length}) did nothing");
            }
        }
    }
}

#[test]
fn rfft_all_lengths() {
    let mut g = Rng::new(14);
    for length in [128u32, 256, 512] {
        for &buffer_len in &[length, length + 1, length + 2] {
            for iter in 0..100 {
                let shift = iter % 3;
                let mut r = random_vec(&mut g, length as usize + 2, shift);
                let mut c = r.clone();
                let mut sr = g.range(-10, 10);
                let mut sc = sr;
                let rr = rfft(length, &mut r, buffer_len, &mut sr);
                let rc = unsafe { cref_rfft(length, c.as_mut_ptr(), buffer_len, &mut sc) };
                assert_eq!((rr, sr), (rc, sc), "rfft({length}, {buffer_len})");
                assert_eq!(r, c, "rfft({length}, {buffer_len})");

                let mut r = random_vec(&mut g, length as usize + 2, shift);
                let mut c = r.clone();
                let mut sr = g.range(-10, 10);
                let mut sc = sr;
                let rr = irfft(length, &mut r, buffer_len, &mut sr);
                let rc = unsafe { cref_irfft(length, c.as_mut_ptr(), buffer_len, &mut sc) };
                assert_eq!((rr, sr), (rc, sc), "irfft({length}, {buffer_len})");
                assert_eq!(r, c, "irfft({length}, {buffer_len})");
            }
        }
    }
    for length in [16u32, 32, 64, 96, 128, 256, 512, 1024, 2048] {
        let mut step_c = 0u32;
        let id_c = unsafe { cref_getSineTab(length, &mut step_c) };
        let (tab, step) = get_sine_tab(length);
        assert_eq!((table_id(tab), step), (id_c, step_c), "getSineTab({length})");
    }
}

#[test]
fn dct_tables() {
    let mut lengths: Vec<i32> = (3..=11).map(|k| 1 << k).collect();
    lengths.extend_from_slice(&[96, 120, 160, 192, 240, 480, 768, 960]);
    for &length in &lengths {
        let mut c = [0i32; 3];
        unsafe { cref_dct_getTables(c.as_mut_ptr(), length) };
        let (tw, sin_tw, step) = dct_get_tables(length);
        assert_eq!([table_id(tw), table_id(sin_tw), step], c, "dct_getTables({length})");
    }
}

#[test]
fn dct_all_lengths() {
    let mut g = Rng::new(15);
    for l in [128usize, 256, 512, 1024] {
        for iter in 0..200 {
            let shift = [0, 0, 1, 4, 12][iter % 5];
            let input = random_vec(&mut g, l, shift);
            let e0 = g.range(-8, 8);
            for kind in 0..5 {
                let mut r = input.clone();
                let mut c = input.clone();
                let mut tr = vec![0i32; l];
                let mut tc = vec![0i32; l];
                let (mut er, mut ec) = (e0, e0);
                let li = l as i32;
                unsafe {
                    match kind {
                        0 => {
                            dct_ii(&mut r, &mut tr, &mut er);
                            cref_dct_II(c.as_mut_ptr(), tc.as_mut_ptr(), li, &mut ec);
                        }
                        1 => {
                            dct_iii(&mut r, &mut tr, &mut er);
                            cref_dct_III(c.as_mut_ptr(), tc.as_mut_ptr(), li, &mut ec);
                        }
                        2 => {
                            dst_iii(&mut r, &mut tr, &mut er);
                            cref_dst_III(c.as_mut_ptr(), tc.as_mut_ptr(), li, &mut ec);
                        }
                        3 => {
                            dct_iv(&mut r, &mut er);
                            cref_dct_IV(c.as_mut_ptr(), li, &mut ec);
                        }
                        _ => {
                            dst_iv(&mut r, &mut er);
                            cref_dst_IV(c.as_mut_ptr(), li, &mut ec);
                        }
                    }
                }
                assert_eq!(er, ec, "kind {kind} L {l} exponent");
                assert_eq!(r, c, "kind {kind} L {l}");
                assert!(r != input || input.iter().all(|&x| x == 0), "kind {kind} L {l}: no-op");
                if kind < 3 {
                    assert_eq!(tr, tc, "kind {kind} L {l} tmp");
                }
            }
        }
    }
}
