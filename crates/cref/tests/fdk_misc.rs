// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of TD_upsampler.h and the STFT filterbank, plus FDK_matrixCalloc.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_misc::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::matrix_calloc::*;
use mpeghdec_fdk::stftfilterbank::*;
use mpeghdec_fdk::td_upsampler::*;

fn vec_i32(g: &mut Rng, n: usize, shift: u32) -> Vec<i32> {
    (0..n).map(|_| g.i32() >> shift).collect()
}

#[test]
fn td_upsampler_all_factors() {
    let mut g = Rng::new(41);
    for fac in [TD_FAC_UPSAMPLE_1_1, TD_FAC_UPSAMPLE_3_2, TD_FAC_UPSAMPLE_2_1, TD_FAC_UPSAMPLE_3_1]
    {
        for iter in 0..100 {
            let mut st_r = vec_i32(&mut g, TD_STATES_MEM_SIZE, 0);
            let mut st_c = st_r.clone();
            let dr = td_upsampler_init(fac, &mut st_r);
            let dc = unsafe { cref_TD_upsampler_init(fac, st_c.as_mut_ptr()) };
            assert_eq!((dr, &st_r), (dc, &st_c), "TD_upsampler_init({fac})");
            for block in 0..5 {
                let len = 2 * g.range(0, 300) as usize;
                let shift = g.range(0, 6) as u32;
                let input = vec_i32(&mut g, len, shift);
                let mut out_r = vec![0x3333_3333i32; 3 * len + 8];
                let mut out_c = out_r.clone();
                let nr = td_upsampler(fac, &input, &mut out_r, &mut st_r);
                let nc = unsafe {
                    cref_TD_upsampler(
                        fac,
                        input.as_ptr(),
                        len as i16,
                        out_c.as_mut_ptr(),
                        st_c.as_mut_ptr(),
                    )
                };
                let what = format!("fac {fac} iter {iter} block {block} len {len}");
                assert_eq!(nr, nc, "{what}");
                assert_eq!(out_r, out_c, "{what}: output");
                assert_eq!(st_r, st_c, "{what}: states");
            }
        }
    }
}

#[test]
fn stft_filterbank() {
    let mut g = Rng::new(42);
    for fft_size in [256u32, 512] {
        let frame_size = fft_size / 2;
        for mode in [STFT_FILTERBANK_MODE_TIME_TO_FREQ, STFT_FILTERBANK_MODE_FREQ_TO_TIME] {
            for iter in 0..30 {
                let cfg = StftFilterbankConfig { stft_filterbank_mode: mode, frame_size, fft_size };
                let mut h_r = None;
                let err_r = stft_filterbank_open(&cfg, &mut h_r);
                let mut err_c = 0;
                let h_c = unsafe { cref_stft_open(mode, frame_size, fft_size, &mut err_c) };
                assert_eq!(err_r, err_c);
                let h = h_r.as_mut().unwrap();
                assert_eq!(h.num_zeros_padded_two_sided, unsafe { cref_stft_zeros(h_c) });

                for frame in 0..20 {
                    let headroom = g.range(0, 4);
                    let what = format!(
                        "fft {fft_size} mode {mode} iter {iter} frame {frame} headroom {headroom}"
                    );
                    let (mut in_r, mut out_r);
                    if mode == STFT_FILTERBANK_MODE_TIME_TO_FREQ {
                        let shift = g.range(headroom as i32, 8) as u32 + 1;
                        in_r = vec_i32(&mut g, frame_size as usize, shift);
                        out_r = vec![0x7777_7777i32; fft_size as usize];
                    } else {
                        let shift = g.range(0, 12) as u32;
                        in_r = vec_i32(&mut g, fft_size as usize, shift);
                        if g.range(0, 3) == 0 {
                            in_r.iter_mut().for_each(|x| *x >>= 1);
                        }
                        let shift = g.range(4, 10) as u32;
                        out_r = vec_i32(&mut g, frame_size as usize, shift);
                    }
                    let mut in_c = in_r.clone();
                    let mut out_c = out_r.clone();
                    stft_filterbank_process(&mut in_r, &mut out_r, h, headroom);
                    unsafe {
                        cref_stft_process(in_c.as_mut_ptr(), out_c.as_mut_ptr(), h_c, headroom)
                    };
                    assert_eq!(out_r, out_c, "{what}: output");
                    assert_eq!(in_r, in_c, "{what}: input");
                    let prev_c = unsafe {
                        core::slice::from_raw_parts(cref_stft_prev(h_c), frame_size as usize)
                    };
                    assert_eq!(&h.prev_audio_input[..], prev_c, "{what}: prevAudioInput");
                }
                assert_eq!(stft_filterbank_close(&mut h_r), unsafe { cref_stft_close(h_c) });
                assert!(h_r.is_none());
            }
        }
    }
    // Invalid mode
    let cfg = StftFilterbankConfig {
        stft_filterbank_mode: STFT_FILTERBANK_MODE_INVALID,
        frame_size: 256,
        fft_size: 512,
    };
    let mut h_r = None;
    let mut err_c = 0;
    let h_c = unsafe { cref_stft_open(STFT_FILTERBANK_MODE_INVALID, 256, 512, &mut err_c) };
    assert!(h_c.is_null());
    assert_eq!(stft_filterbank_open(&cfg, &mut h_r), err_c);
    assert!(h_r.is_none());
}

#[test]
fn matrix_calloc() {
    let mut m: Matrix2D<i32> = fdk_calloc_matrix_2d(3, 5).unwrap();
    assert_eq!((m.dim1(), m.dim2()), (3, 5));
    assert!(m.as_flat().iter().all(|&x| x == 0));
    m[1][2] = 7;
    m[2][4] = 9;
    assert_eq!(m.as_flat()[5 + 2], 7);
    assert_eq!(m.as_flat()[14], 9);
    {
        let mut rows = m.rows_mut();
        let r0 = rows.next().unwrap();
        let r1 = rows.next().unwrap();
        r0[0] = r1[2];
    }
    assert_eq!(m[0][0], 7);
    assert_eq!(m.rows().count(), 3);
    assert!(fdk_calloc_matrix_2d::<i32>(0, 5).is_none());
    assert!(fdk_calloc_matrix_2d::<i32>(5, 0).is_none());
    let v: Vec<i16> = fdk_calloc_matrix_1d(4);
    assert_eq!(v, [0; 4]);
}
