// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of the libMpeghDec block, TNS and joint stereo functions (block.cpp,
//! aacdec_tns.cpp, stereo.cpp, CChannel_CodebookTableInit) on random element states: every
//! function is called with the same random state in C and Rust and the complete element state is
//! compared afterwards.
#![cfg(not(cref_missing))]

mod core_common;

use core_common::element::*;
use core_common::*;
use mpeghdec_core::aac_rom::HUFFMAN_CODE_BOOK_SCL;
use mpeghdec_core::aacdec_tns::*;
use mpeghdec_core::block::*;
use mpeghdec_core::channel::*;
use mpeghdec_core::channelinfo::*;
use mpeghdec_core::stereo::*;
use mpeghdec_cref::core_channel::*;
use mpeghdec_cref::core_common::*;
use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::igf::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::mdct::{BLOCK_LONG, BLOCK_SHORT, BLOCK_START};
use mpeghdec_igf::igf_dec::*;

// ---------------------------------------------------------------------------------------------
// Random states
// ---------------------------------------------------------------------------------------------

/// A random ics info (like IcsRead() produces it).
fn random_ics(g: &mut Rng, sri: &SamplingRateInfo, ws: Option<i32>) -> CIcsInfo {
    let mut ics = CIcsInfo {
        window_sequence: ws.unwrap_or_else(|| g.range(0, 3)),
        window_shape: g.range(0, 1) as u8,
        ..Default::default()
    };
    let short = ics.window_sequence == BLOCK_SHORT;
    ics.total_sf_bands = if short {
        sri.number_of_scale_factor_bands_short
    } else {
        sri.number_of_scale_factor_bands_long
    };
    ics.max_sf_bands = match g.range(0, 4) {
        0 => 0,
        1 => ics.total_sf_bands,
        _ => g.range(0, ics.total_sf_bands as i32) as u8,
    };
    ics.max_sfb_ste = g.range(0, ics.max_sf_bands as i32) as u8;
    ics.valid = 1;
    if short {
        ics.scale_factor_grouping = g.range(0, 127) as u8;
        ics.window_groups = 0;
        for i in 0..7 {
            let mask = 1 << (6 - i);
            ics.window_group_length[i] = 1;
            if ics.scale_factor_grouping & mask != 0 {
                ics.window_group_length[ics.window_groups as usize] += 1;
            } else {
                ics.window_groups += 1;
            }
        }
        ics.window_group_length[7] = 1;
        ics.window_groups += 1;
    } else {
        ics.window_groups = 1;
        ics.window_group_length[0] = 1;
    }
    ics
}

/// A random spectrum value of random magnitude.
fn rnd_spec(g: &mut Rng) -> i32 {
    match g.range(0, 7) {
        0 => 0,
        1 => g.range(-3, 3),
        _ => g.i32() >> g.range(0, 31),
    }
}

/// Random TNS data (valid filter bands and orders) for an ics info.
fn random_tns(g: &mut Rng, ics: &CIcsInfo) -> CTnsData {
    let mut t = CTnsData::default();
    if g.range(0, 2) == 0 {
        return t;
    }
    let short = ics.window_sequence == BLOCK_SHORT;
    let nwin = if short { 8 } else { 1 };
    t.active = 1;
    t.gain_ld = if g.range(0, 1) != 0 { 4 } else { 0 };
    for w in 0..nwin {
        let n = g.range(0, if short { 1 } else { 3 }) as usize;
        t.number_of_filters[w] = n as u8;
        let mut next = ics.total_sf_bands as i32;
        for f in 0..n {
            let fi = &mut t.filter[w][f];
            let len = g.range(0, next);
            fi.start_band = (next - len) as u8;
            fi.stop_band = next as u8;
            next -= len;
            fi.order = g.range(0, if short { 7 } else { 15 }) as u8;
            fi.direction = if g.range(0, 1) != 0 { -1 } else { 1 };
            fi.resolution = g.range(3, 4) as i8;
            for c in 0..fi.order as usize {
                fi.coeff[c] =
                    if fi.resolution == 3 { g.range(-4, 3) } else { g.range(-8, 7) } as i8;
            }
        }
    }
    t
}

/// Randomizes the channel state of the element (ics info, scale factors, codebooks, TNS, noise,
/// spectra, concealment data) and the element data.
fn randomize(g: &mut Rng, el: &mut El, common_window: bool) {
    let sri = el.sri;
    let ws0 = g.range(0, 3);
    let ics0 = random_ics(g, &sri, Some(ws0));
    for c in 0..el.cfg.nch {
        let ics = if common_window && c == 1 {
            let mut i = ics0;
            i.max_sf_bands = if g.range(0, 2) == 0 {
                i.max_sf_bands
            } else {
                g.range(0, i.total_sf_bands as i32) as u8
            };
            i
        } else if c == 0 {
            ics0
        } else {
            random_ics(g, &sri, None)
        };
        let ch = &mut el.ch[c];
        ch.ics_info = ics;
        let d = &mut ch.p_dyn_data;
        for i in 0..128 {
            d.a_scale_factor[i] = g.range(-100, 155) as i16;
            d.a_sfb_scale[i] = match g.range(0, 5) {
                0 => g.range(-5, 60),
                _ => g.range(0, 30),
            } as i16;
            d.a_code_book[i] = match g.range(0, 9) {
                0 => ZERO_HCB,
                1 => INTENSITY_HCB,
                2 => INTENSITY_HCB2,
                3 => NOISE_HCB,
                _ => ESCBOOK,
            };
            d.band_is_noise[i] = g.range(0, 1) as u8;
        }
        /* Like after CBlock_InverseQuantizeSpectralData(): the scale of bands that are not
        transmitted is 0. (With nonzero values the complex prediction can shift by negative
        amounts, which the C++ loops execute vectorized or scalar with different results.) */
        if ics.window_sequence == BLOCK_SHORT {
            for w in 0..8 {
                for b in ics.max_sf_bands as usize..16 {
                    d.a_sfb_scale[w * 16 + b] = 0;
                }
            }
        } else {
            for b in ics.max_sf_bands as usize..128 {
                d.a_sfb_scale[b] = 0;
            }
        }
        d.tns_data = random_tns(g, &ics);
        d.raw_data_info.common_window = common_window as u8;
        d.raw_data_info.global_gain = g.range(0, 255) as u8;
        d.specific_to_usac.fd_noise_level_and_offset = g.range(0, 255) as u8;
        d.specific_to_usac.tns_active = g.range(0, 1) as u8;
        ch.spec_scale = [0; 8];
        for s in ch.spec_scale.iter_mut() {
            *s = g.range(0, 30) as i16;
        }
        ch.transform_splitting_active =
            (ics.window_sequence == BLOCK_START && g.range(0, 1) != 0) as u8;
        ch.curr_aliasing_symmetry = g.range(0, 1);
        let st = &mut el.st[c];
        for v in st.concealment_info.spectral_coefficient.iter_mut() {
            *v = rnd_spec(g);
        }
        for s in st.concealment_info.spec_scale.iter_mut() {
            *s = g.range(0, 30) as i16;
        }
        st.nf_random_seed = g.next_u32();
    }
    for v in el.spec.iter_mut() {
        *v = rnd_spec(g);
    }
    for v in el.wb.buf.iter_mut() {
        *v = rnd_spec(g);
    }
    for v in el.td2.iter_mut() {
        *v = rnd_spec(g);
    }
    if let Some(j) = &mut el.com.joint_stereo_data {
        j.ms_mask_present = g.range(0, 3) as u8;
        for m in j.ms_used.iter_mut() {
            *m = g.next_u32() as u8;
        }
        j.cplx_pred_flag = g.range(0, 1) as u8;
        j.igf_ms_mask_present = g.range(0, 3) as u8;
        j.igf_cplx_pred_flag = g.range(0, 1) as u8;
        for v in j.store_dmx_re_prev.iter_mut() {
            *v = rnd_spec(g);
        }
        j.store_dmx_re_prev_e = g.range(0, 30) as i16;
    }
    if let Some(p) = &mut el.com.cplx_prediction_data {
        p.pred_dir = g.range(0, 1) as u8;
        p.igf_pred_dir = g.range(0, 1) as u8;
        p.complex_coef = g.range(0, 1) as u8;
        p.use_prev_frame = p.complex_coef & g.range(0, 1) as u8;
        for grp in 0..8 {
            for b in 0..64 {
                p.alpha_q_re[grp][b] = g.range(-30, 30) as i16;
                p.alpha_q_im[grp][b] = g.range(-30, 30) as i16;
            }
        }
    }
    if let Some(p) = &mut el.cpe {
        let j = &mut p.joint_stereo_persistent_data;
        j.clear_spectral_coeffs = (g.range(0, 5) == 0) as u8;
        j.win_seq_prev = g.range(0, 3) as u8;
        j.win_shape_prev = g.range(0, 1) as u8;
        j.win_groups_prev = g.range(1, 8) as u8;
        for grp in 0..8 {
            for b in 0..64 {
                j.alpha_q_re_prev[grp][b] = g.range(-30, 30) as i16;
                j.alpha_q_im_prev[grp][b] = g.range(-30, 30) as i16;
            }
        }
    }
    el.sync_to_c();
}

/// Random IGF bitstream data and tile content of all channels (read with the IGF parser from
/// random bits and injected spectra in C and Rust).
fn randomize_igf(g: &mut Rng, el: &mut El) {
    if !el.cfg.enf {
        return;
    }
    for c in 0..el.cfg.nch {
        let short = (el.ch[c].ics_info.window_sequence == BLOCK_SHORT) as i32;
        let groups = if short != 0 { el.ch[c].ics_info.window_groups as i32 } else { 1 };
        let mut w = BitWriter::new();
        if g.range(0, 3) == 0 {
            w.bit(1);
        }
        w.random(g, 4000);
        let mut buf_c = w.buf.clone();
        let hc =
            unsafe { cref_bs_new(buf_c.as_mut_ptr(), BUF_SIZE as u32, (BUF_SIZE * 8) as u32, 0) };
        let (_b, mut bs) = w.reader_at(0);
        let (cc, cs) = (el.c_ch(c), el.c_st(c));
        let (ci, si) = unsafe { (cref_core_ch_igf(cc), cref_core_st_igf(cs)) };
        let indep = g.range(0, 1);
        let use_enf = el.cfg.use_enf as i32;
        unsafe {
            cref_igf_read_scf(si, ci, hc, indep, short, groups, 0);
            cref_igf_read_igf(si, ci, hc, short, indep, use_enf, 0);
            cref_bs_free(hc);
        }
        let (st, ch) = (&mut el.st[c], &mut el.ch[c]);
        iis_igf_dec_lib_read_scf(
            &mut st.igf_static_data,
            &mut ch.igf_data,
            &mut bs,
            indep,
            short,
            groups,
            0,
        );
        iis_igf_dec_lib_read_igf(
            &mut st.igf_static_data,
            &mut ch.igf_data,
            &mut bs,
            short,
            indep,
            use_enf,
            0,
        );
        /* tile spectra and exponents */
        let mut spec: Vec<i32> = (0..1024).map(|_| rnd_spec(g)).collect();
        /* like iisIGFDecLibInjectSourceSpectrumNew() in CChannelElement_Read(): the sfb scales */
        let mut exp: Vec<i16> = ch.p_dyn_data.a_sfb_scale.to_vec();
        unsafe { cref_igf_inject_new(si, ci, spec.as_mut_ptr(), exp.as_mut_ptr(), short, 0) };
        iis_igf_dec_lib_inject_source_spectrum_new(
            &mut st.igf_static_data,
            &mut ch.igf_data,
            &spec,
            &exp,
            short,
            0,
        );
        let mut scale = [0i16; 32];
        for s in scale.iter_mut() {
            *s = g.range(0, 30) as i16;
        }
        unsafe { cref_igf_set_spectrum_scale(si, scale.as_ptr()) };
        for t in 0..4 {
            st.igf_static_data.f_spectrum_scale[t].copy_from_slice(&scale[t * 8..t * 8 + 8]);
        }
    }
}

fn random_cfg(g: &mut Rng, nch: usize, enf: bool) -> Cfg {
    let mut c = Cfg::random(g);
    c.nch = nch;
    c.lfe = false;
    c.enf = enf;
    c
}

fn bs_pair(buf: &mut [u8], buf_c: &mut [u8]) -> (*mut (), FdkBitStream) {
    let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), BUF_SIZE as u32, (BUF_SIZE * 8) as u32, 0) };
    let mut bs = FdkBitStream::default();
    bs.init(&BitMem::from_slice(buf), BUF_SIZE as u32, (BUF_SIZE * 8) as u32, FdkBsCfg::BsReader);
    (hc, bs)
}

fn random_buf(g: &mut Rng, bias: i32) -> Vec<u8> {
    let mut w = BitWriter::new();
    for _ in 0..BUF_SIZE * 8 {
        let b = match bias {
            0 => g.next_u32() & 1,
            _ => (g.range(0, bias) == 0) as u32,
        };
        w.bit(b);
    }
    w.buf
}

// ---------------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------------

#[test]
fn huffman_words_and_inline_functions() {
    let mut g = Rng::new(1);
    for iter in 0..3000 {
        let mut buf = random_buf(&mut g, [0, 2, 3, 8][iter % 4]);
        let mut buf_c = buf.clone();
        let (hc, mut bs) = bs_pair(&mut buf, &mut buf_c);
        for k in 0..50 {
            let cb = g.range(0, 1);
            let vc = unsafe { cref_core_decode_huffman_word(hc, cb) };
            let vr = if cb != 0 {
                cblock_decode_huffman_word_cb(&mut bs, &HUFFMAN_CODE_BOOK_SCL)
            } else {
                cblock_decode_huffman_word(
                    &mut bs,
                    &mpeghdec_core::aac_rom::AAC_CODE_BOOK_DESCRIPTION_SCL,
                )
            };
            assert_eq!(vc, vr, "iter {iter} word {k}");
            assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits());
        }
        unsafe { cref_bs_free(hc) };
    }
    for q in 0..8192 {
        for lsb in 0..4u32 {
            let (mut c, mut r) = (q, q);
            let ec = unsafe { cref_core_evaluate_power43(&mut c, lsb) };
            let er = evaluate_power43(&mut r, lsb);
            assert_eq!((ec, c), (er, r), "EvaluatePower43({q}, {lsb})");
            for scale in [-40, -20, -3, 0, 3, 10, 20] {
                for v in [q, -q] {
                    let (mut c, mut r) = (v, v);
                    unsafe { cref_core_inverse_quantize(&mut c, lsb, scale) };
                    inverse_quantize(&mut r, lsb, scale);
                    assert_eq!(c, r, "InverseQuantize({v}, {lsb}, {scale})");
                }
            }
        }
    }
    for idx in 0..256 {
        let (mut c, mut r) = ([0i32; 4], [0i32; 4]);
        let dc = unsafe { cref_core_unpack_index(idx, c.as_mut_ptr()) };
        let dr = cblock_unpack_index(
            idx,
            &mut r,
            &mpeghdec_core::aac_rom::AAC_CODE_BOOK_DESCRIPTION_SCL,
        );
        assert_eq!((dc, c), (dr, r));
    }
}

#[test]
fn scale_factor_data() {
    let mut g = Rng::new(2);
    for iter in 0..4000 {
        let mut el = El::new(random_cfg(&mut g, 1, false));
        randomize(&mut g, &mut el, false);
        if g.range(0, 1) != 0 {
            unsafe { cref_core_codebook_table_init(el.c, 0) };
            cchannel_codebook_table_init(&mut el.ch[0]);
            el.check(&format!("iter {iter}: codebook table init"));
        }
        let flags = [0, AC_USAC, AC_MPEGH3DA, AC_MPEGH3DA | AC_INDEP][g.range(0, 3) as usize];
        let r_ = g.range(0, 3);
        let mut buf = random_buf(&mut g, [0, 2, 3, 6][r_ as usize]);
        let mut buf_c = buf.clone();
        let (hc, mut bs) = bs_pair(&mut buf, &mut buf_c);
        let ec = unsafe { cref_core_el_read_sf(el.c, 0, hc, flags) };
        let er = cblock_read_scale_factor_data(&mut el.ch[0], &mut bs, flags);
        assert_eq!(ec, er, "iter {iter}");
        assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits(), "iter {iter}");
        unsafe { cref_bs_free(hc) };
        el.check(&format!("iter {iter}: scale factors"));
    }
}

#[test]
fn inverse_quantization_and_scaling() {
    let mut g = Rng::new(3);
    for iter in 0..3000 {
        let r_ = g.range(0, 1);
        let mut el = El::new(random_cfg(&mut g, 1, r_ != 0));
        randomize(&mut g, &mut el, false);
        /* quantized values */
        let big = g.range(0, 5) == 0;
        for v in el.spec.iter_mut() {
            *v = match g.range(0, 6) {
                0 => g.range(-8191, 8191),
                1 if big => g.range(-9000, 9000),
                _ => g.range(-20, 20),
            };
        }
        el.sync_buffers_to_c();
        let active = g.range(0, 1);
        let ec = unsafe { cref_core_el_inverse_quantize(el.c, 0, active) };
        let er = {
            let El { ch, spec, sri, .. } = &mut el;
            cblock_inverse_quantize_spectral_data(&mut ch[0], &mut spec[..1024], sri, active as u8)
        };
        assert_eq!(ec, er, "iter {iter}");
        el.check(&format!("iter {iter}: inverse quantization"));

        let max_sfbs = g.range(0, el.ch[0].ics_info.max_sf_bands as i32) as u8;
        unsafe { cref_core_el_scale_spectral_data(el.c, 0, max_sfbs as i32) };
        {
            let El { ch, spec, sri, .. } = &mut el;
            cblock_scale_spectral_data(&mut ch[0], &mut spec[..1024], max_sfbs, sri);
        }
        el.check(&format!("iter {iter}: scale spectral data"));

        if el.cfg.enf {
            randomize_igf(&mut g, &mut el);
            el.check(&format!("iter {iter}: random igf"));
            unsafe { cref_core_el_scale_tile_data(el.c, 0, max_sfbs as i32) };
            {
                let El { ch, st, sri, .. } = &mut el;
                cblock_scale_tile_data(&mut st[0], &mut ch[0], max_sfbs, sri);
            }
            el.check(&format!("iter {iter}: scale tile data"));
        }
    }
}

#[test]
fn noise_filling_and_stereo_filling() {
    let mut g = Rng::new(4);
    for iter in 0..3000 {
        let nch = g.range(1, 2) as usize;
        let r_ = g.range(0, 1);
        let mut el = El::new(random_cfg(&mut g, nch, r_ != 0));
        randomize(&mut g, &mut el, nch == 2);
        randomize_igf(&mut g, &mut el);
        el.check(&format!("iter {iter}: init"));
        let c = g.range(0, nch as i32 - 1) as usize;
        let inf = (el.cfg.enf && g.range(0, 1) != 0) as i32;
        let sf = g.range(0, 1);
        let max_noise_sfb = g.range(0, el.ch[c].ics_info.max_sf_bands as i32);
        unsafe { cref_core_el_apply_noise(el.c, c as i32, inf, sf, max_noise_sfb) };
        {
            let El { ch, st, spec, sri, .. } = &mut el;
            cblock_apply_noise(
                &mut st[c],
                &mut ch[c],
                &mut spec[c * 1024..(c + 1) * 1024],
                sri,
                inf,
                sf,
                max_noise_sfb,
            );
        }
        el.check(&format!("iter {iter}: noise filling (inf {inf}, sf {sf}, max {max_noise_sfb})"));

        if nch == 2 && el.ch[1].ics_info.window_sequence != BLOCK_SHORT {
            /* previous downmix and stereo filling of the right channel */
            let mut dmx_c = vec![0i32; 1024];
            let mut exp_c = vec![0i16; 64];
            unsafe {
                cref_core_el_get_previous_dmx(
                    el.c,
                    dmx_c.as_mut_ptr(),
                    exp_c.as_mut_ptr(),
                    max_noise_sfb,
                )
            };
            let mut dmx_r = vec![0i32; 1024];
            let mut exp_r = vec![0i16; 64];
            {
                let (l, r) = (&el.st[0], &el.st[1]);
                let ics = &el.ch[0].ics_info;
                igf_stereo_filling_get_previous_dmx(
                    &el.cpe.as_ref().unwrap().joint_stereo_persistent_data,
                    el.com.cplx_prediction_data.as_ref().unwrap(),
                    &l.concealment_info.spectral_coefficient,
                    &r.concealment_info.spectral_coefficient,
                    &l.concealment_info.spec_scale,
                    &r.concealment_info.spec_scale,
                    ics,
                    el.ch[0].granule_length,
                    el.ch[1].granule_length,
                    &mut dmx_r,
                    &mut exp_r,
                    get_scale_factor_band_offsets(ics, &el.sri),
                    &ics.window_group_length,
                    ics.window_groups as i32,
                    max_noise_sfb,
                    &el.ch[1].p_dyn_data.band_is_noise,
                );
            }
            assert_same(&format!("iter {iter}: previous dmx"), &dmx_c, &dmx_r);
            assert_same(
                &format!("iter {iter}: previous dmx exp"),
                &exp_c.iter().map(|&v| v as i32).collect::<Vec<_>>(),
                &exp_r.iter().map(|&v| v as i32).collect::<Vec<_>>(),
            );
            /* random downmix for the stereo filling */
            for v in dmx_r.iter_mut() {
                *v = rnd_spec(&mut g);
            }
            let e0 = g.range(-5, 30) as i16;
            for e in exp_r.iter_mut() {
                *e = e0;
            }
            let (mut dmx_c, mut exp_c) = (dmx_r.clone(), exp_r.clone());
            let tiles = (el.cfg.enf && g.range(0, 1) != 0) as i32;
            unsafe {
                cref_core_el_stereo_filling(
                    el.c,
                    max_noise_sfb,
                    dmx_c.as_mut_ptr(),
                    exp_c.as_mut_ptr(),
                    tiles,
                )
            };
            {
                let El { ch, st, spec, sri, .. } = &mut el;
                let chr = &mut ch[1];
                igf_stereo_filling_prepare(
                    chr,
                    &spec[1024..],
                    sri,
                    max_noise_sfb,
                    &mut dmx_r,
                    &mut exp_r,
                    &chr.p_dyn_data.band_is_noise,
                );
                if tiles != 0 {
                    let str_ = &mut st[1];
                    let n = iis_igf_dec_lib_get_number_of_tiles(&str_.igf_static_data, 0);
                    for t in 0..n {
                        let ts = iis_igf_dec_lib_access_source_spectrum(
                            &str_.igf_static_data,
                            &mut chr.igf_data,
                            t,
                            0,
                            0,
                        )
                        .unwrap();
                        let te = iis_igf_dec_lib_access_source_spectrum_exponent(
                            &mut str_.igf_static_data,
                            t,
                            0,
                        );
                        igf_stereo_filling_apply(
                            &chr.ics_info,
                            &chr.p_dyn_data.a_scale_factor,
                            ts,
                            te,
                            sri,
                            max_noise_sfb,
                            &dmx_r,
                            &exp_r,
                            &chr.p_dyn_data.band_is_noise,
                        );
                    }
                }
                let d = &mut *chr.p_dyn_data;
                igf_stereo_filling_apply(
                    &chr.ics_info,
                    &d.a_scale_factor,
                    &mut spec[1024..2048],
                    &mut d.a_sfb_scale,
                    sri,
                    max_noise_sfb,
                    &dmx_r,
                    &exp_r,
                    &d.band_is_noise,
                );
            }
            assert_same(&format!("iter {iter}: stereo filling dmx"), &dmx_c, &dmx_r);
            el.check(&format!("iter {iter}: stereo filling"));
        }
    }
}

#[test]
fn arithmetic_spectral_data() {
    let mut g = Rng::new(5);
    for iter in 0..400 {
        let mut el = El::new(random_cfg(&mut g, 1, false));
        for frame in 0..10 {
            randomize(&mut g, &mut el, false);
            let flags = AC_MPEGH3DA | if frame == 0 || g.range(0, 4) == 0 { AC_INDEP } else { 0 };
            let mut buf = random_buf(&mut g, 0);
            let mut buf_c = buf.clone();
            let (hc, mut bs) = bs_pair(&mut buf, &mut buf_c);
            let ec = unsafe { cref_core_el_read_ac(el.c, 0, hc, 1024, flags) };
            let er = {
                let El { ch, st, spec, sri, .. } = &mut el;
                cblock_read_ac_spectral_data(
                    &mut bs,
                    &ch[0],
                    &mut spec[..1024],
                    &mut st[0],
                    sri,
                    1024,
                    flags,
                )
            };
            assert_eq!(ec, er, "iter {iter} frame {frame}");
            assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits());
            unsafe { cref_bs_free(hc) };
            el.check(&format!("iter {iter} frame {frame}: ac spectral data"));
        }
    }
}

#[test]
fn tns_read_and_apply() {
    let mut g = Rng::new(6);
    for iter in 0..4000 {
        let mut el = El::new(random_cfg(&mut g, 2, false));
        let r_ = g.range(0, 1);
        randomize(&mut g, &mut el, r_ != 0);
        let flags = [0, AC_USAC, AC_MPEGH3DA][g.range(0, 2) as usize];
        let r_ = g.range(0, 2);
        let mut buf = random_buf(&mut g, [0, 2, 4][r_ as usize]);
        let mut buf_c = buf.clone();
        let (hc, mut bs) = bs_pair(&mut buf, &mut buf_c);
        if g.range(0, 1) != 0 {
            let el_flags = [0, AC_EL_ENHANCED_NOISE, AC_EL_ENHANCED_NOISE | AC_EL_IGF_AFTER_TNS]
                [g.range(0, 2) as usize];
            let cw = g.range(0, 1);
            let rc = unsafe { cref_core_el_tns_read_usac(el.c, hc, flags, el_flags, cw) };
            let mut on_lr = 0xAAu8;
            {
                let (a, b) = el.ch.split_at_mut(1);
                let c0 = &mut a[0];
                ctns_read_data_present_usac(
                    &mut bs,
                    &mut c0.p_dyn_data.tns_data,
                    &mut b[0].p_dyn_data.tns_data,
                    &mut on_lr,
                    &c0.ics_info,
                    flags,
                    el_flags,
                    cw,
                );
            }
            assert_eq!(rc, on_lr as i32, "iter {iter}");
        }
        for c in 0..2 {
            let dp = g.range(0, 1) as u8;
            el.ch[c].p_dyn_data.tns_data.data_present = dp;
            let d = dump_ch(&el.ch[c]);
            unsafe { cref_core_ch_load(el.c_ch(c), d.as_ptr()) };
            let ec = unsafe { cref_core_el_tns_read(el.c, c as i32, hc, flags) };
            let er = {
                let ch = &mut el.ch[c];
                ctns_read(&mut bs, &mut ch.p_dyn_data.tns_data, &ch.ics_info, flags)
            };
            assert_eq!(ec, er, "iter {iter}");
        }
        assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits(), "iter {iter}");
        unsafe { cref_bs_free(hc) };
        el.check(&format!("iter {iter}: tns read"));
        for c in 0..2 {
            let t = random_tns(&mut g, &el.ch[c].ics_info);
            el.ch[c].p_dyn_data.tns_data = t;
            let d = dump_ch(&el.ch[c]);
            unsafe { cref_core_ch_load(el.c_ch(c), d.as_ptr()) };
            let nbands = g.range(0, el.ch[c].ics_info.total_sf_bands as i32);
            let igf = g.range(0, 1);
            let flags = [0, AC_USAC, AC_MPEGH3DA][g.range(0, 2) as usize];
            unsafe { cref_core_el_tns_apply(el.c, c as i32, nbands, igf, flags) };
            {
                let El { ch, spec, sri, .. } = &mut el;
                let chc = &ch[c];
                ctns_apply(
                    &chc.p_dyn_data.tns_data,
                    &chc.ics_info,
                    &mut spec[c * 1024..(c + 1) * 1024],
                    sri,
                    chc.granule_length,
                    nbands as u8,
                    igf as u8,
                    flags,
                );
            }
            el.check(&format!("iter {iter}: tns apply {c}"));
        }
    }
}

#[test]
fn joint_stereo_read() {
    let mut g = Rng::new(7);
    for iter in 0..4000 {
        let r_ = g.range(0, 1);
        let mut el = El::new(random_cfg(&mut g, 2, r_ != 0));
        randomize(&mut g, &mut el, true);
        let flags = [AC_USAC, AC_MPEGH3DA, AC_MPEGH3DA | AC_INDEP][g.range(0, 2) as usize];
        let r_ = g.range(0, 2);
        let mut buf = random_buf(&mut g, [0, 2, 4][r_ as usize]);
        let mut buf_c = buf.clone();
        let (hc, mut bs) = bs_pair(&mut buf, &mut buf_c);
        let ics = el.ch[0].ics_info;
        let max_sfb_ste = g.range(0, ics.max_sf_bands as i32);
        let clear = g.range(max_sfb_ste, 64);
        let cplx = g.range(0, 3).min(1);
        let igf = (el.cfg.enf && g.range(0, 1) != 0) as i32;
        let short = ics.window_sequence == BLOCK_SHORT;
        let s = &el.st[0].igf_static_data;
        let (start, stop) = if short {
            (s.igf_start_sfb_sb as i32, s.igf_stop_sfb_sb as i32)
        } else {
            (s.igf_start_sfb_lb as i32, s.igf_stop_sfb_lb as i32)
        };
        let hr = s.use_high_res as i32;
        let rc = unsafe {
            cref_core_el_read_ms(el.c, hc, max_sfb_ste, clear, cplx, flags, igf, start, stop, hr)
        };
        let rr = {
            let El { com, cpe, .. } = &mut el;
            let jsd = com.joint_stereo_data.as_deref_mut().unwrap();
            let jspd = if cplx != 0 {
                Some(&mut cpe.as_deref_mut().unwrap().joint_stereo_persistent_data)
            } else {
                None
            };
            let cpd = if cplx != 0 { com.cplx_prediction_data.as_deref_mut() } else { None };
            let mut r = cjoint_stereo_read(
                &mut bs,
                jsd,
                ics.window_groups as i32,
                max_sfb_ste,
                clear,
                jspd,
                cpd,
                cplx,
                ics.total_sf_bands as i32,
                ics.window_sequence,
                flags,
            );
            if igf != 0 {
                r = r * 16
                    + cjoint_stereo_read_igf(
                        &mut bs,
                        com.joint_stereo_data.as_deref_mut().unwrap(),
                        ics.window_groups as i32,
                        start,
                        stop,
                        hr,
                        &mut cpe.as_deref_mut().unwrap().joint_stereo_persistent_data,
                        com.cplx_prediction_data.as_deref_mut().unwrap(),
                        ics.window_sequence,
                        flags,
                    );
            }
            r
        };
        assert_eq!(rc, rr, "iter {iter}");
        assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits(), "iter {iter}");
        unsafe { cref_bs_free(hc) };
        el.check(&format!("iter {iter}: joint stereo read"));
    }
}

#[test]
fn joint_stereo_apply() {
    let mut g = Rng::new(8);
    for iter in 0..6000 {
        let r_ = g.range(0, 1);
        let mut el = El::new(random_cfg(&mut g, 2, r_ != 0));
        randomize(&mut g, &mut el, true);
        randomize_igf(&mut g, &mut el);
        el.check(&format!("iter {iter}: init"));
        let (sfb_l, sfb_r) =
            (el.ch[0].ics_info.max_sf_bands as i32, el.ch[1].ics_info.max_sf_bands as i32);
        let max_sfb_ste = g.range(0, sfb_l.max(sfb_r));
        el.ch[0].ics_info.max_sfb_ste = max_sfb_ste as u8;
        el.sync_channels_to_c();
        let kind = g.range(0, 2);
        match kind {
            0 => {
                unsafe { cref_core_el_apply_ms(el.c, max_sfb_ste, sfb_l, sfb_r) };
                let El { ch, st, spec, sri, com, cpe, wb, td2, .. } = &mut el;
                let (a, b) = ch.split_at_mut(1);
                let (chl, chr) = (&mut a[0], &mut b[0]);
                let (sl, sr) = (&st[0], &st[1]);
                let jsd = com.joint_stereo_data.as_deref_mut().unwrap();
                let cplx = if jsd.cplx_pred_flag != 0 {
                    Some(CJointStereoCplxCtx {
                        cplx_prediction_data: com.cplx_prediction_data.as_deref().unwrap(),
                        joint_stereo_persistent_data: &cpe
                            .as_deref()
                            .unwrap()
                            .joint_stereo_persistent_data,
                        static_spectral_coeffs_l: &sl.concealment_info.spectral_coefficient,
                        static_spectral_coeffs_r: &sr.concealment_info.spectral_coefficient,
                        static_spec_scale_l: &sl.concealment_info.spec_scale,
                        static_spec_scale_r: &sr.concealment_info.spec_scale,
                        scratch_buffer: wb.work_buffer(),
                        scratch_buffer2: &mut td2[2048..3072],
                    })
                } else {
                    None
                };
                let (spl, spr) = spec.split_at_mut(1024);
                let offs = get_scale_factor_band_offsets(&chl.ics_info, sri);
                let wgl = chl.ics_info.window_group_length;
                cjoint_stereo_apply_ms(
                    jsd,
                    cplx,
                    &chl.ics_info,
                    chl.granule_length,
                    chr.granule_length,
                    chl.transform_splitting_active,
                    spl,
                    spr,
                    &mut chl.p_dyn_data.a_sfb_scale,
                    &mut chr.p_dyn_data.a_sfb_scale,
                    &mut chl.spec_scale,
                    &mut chr.spec_scale,
                    offs,
                    &wgl,
                    chl.ics_info.window_groups as i32,
                    max_sfb_ste,
                    sfb_l,
                    sfb_r,
                    1,
                );
            }
            1 if el.cfg.enf => {
                unsafe { cref_core_el_apply_ms_igf_core(el.c, max_sfb_ste, sfb_l, sfb_r) };
                let El { ch, st, sri, com, cpe, wb, td2, .. } = &mut el;
                let (a, b) = ch.split_at_mut(1);
                let (sa, sb) = st.split_at_mut(1);
                let offs = get_scale_factor_band_offsets(&a[0].ics_info, sri);
                let wgl = a[0].ics_info.window_group_length;
                let groups = a[0].ics_info.window_groups as i32;
                cjoint_stereo_apply_ms_igf_core(
                    &mut a[0],
                    &mut b[0],
                    &mut sa[0],
                    &mut sb[0],
                    com.joint_stereo_data.as_deref_mut().unwrap(),
                    com.cplx_prediction_data.as_deref(),
                    cpe.as_deref(),
                    wb,
                    td2,
                    offs,
                    &wgl,
                    groups,
                    max_sfb_ste,
                    sfb_l,
                    sfb_r,
                );
            }
            _ => {
                if !el.cfg.enf {
                    continue;
                }
                let virt = g.range(0, 1);
                unsafe { cref_core_el_apply_ms_igf(el.c, virt) };
                let El { ch, st, spec, sri, com, td2, .. } = &mut el;
                let (spl, spr) = if virt != 0 {
                    td2[..2048].split_at_mut(1024)
                } else {
                    spec.split_at_mut(1024)
                };
                let (chl, chr) = (&ch[0], &ch[1]);
                cjoint_stereo_apply_ms_igf(
                    com.joint_stereo_data.as_deref().unwrap(),
                    com.cplx_prediction_data.as_deref(),
                    &st[0].igf_static_data,
                    &chl.ics_info,
                    chl.granule_length,
                    chr.granule_length,
                    &chl.spec_scale,
                    &chr.spec_scale,
                    spl,
                    spr,
                    get_scale_factor_band_offsets(&chl.ics_info, sri),
                    &chl.ics_info.window_group_length,
                    chl.ics_info.window_groups as i32,
                );
            }
        }
        el.check(&format!("iter {iter}: joint stereo apply {kind}"));
    }
}

#[test]
fn apply_tools_random() {
    let mut g = Rng::new(9);
    for iter in 0..3000 {
        let nch = g.range(1, 2) as usize;
        let r_ = g.range(0, 2);
        let mut el = El::new(random_cfg(&mut g, nch, r_ != 0));
        let r_ = g.range(0, 1);
        randomize(&mut g, &mut el, nch == 2 && r_ != 0);
        randomize_igf(&mut g, &mut el);
        el.check(&format!("iter {iter}: init"));
        let flags = [AC_MPEGH3DA, AC_USAC, AC_MPEGH3DA | AC_INDEP][g.range(0, 2) as usize];
        let el_flags = el.cfg.el_flags();
        let c = g.range(0, nch as i32 - 1);
        let cw = g.range(0, 1);
        unsafe { cref_core_el_apply_tools(el.c, flags, el_flags, c, cw) };
        {
            let (ch, st, mem, sri) = el.split();
            apply_tools(
                st,
                ch,
                mem.spec,
                sri,
                flags,
                el_flags,
                c,
                cw,
                mem.igf_private_data_common,
                mem.time_data2,
            );
        }
        el.check(&format!("iter {iter}: apply tools"));
    }
}

#[test]
fn frequency_to_time_sequences() {
    let mut g = Rng::new(10);
    for iter in 0..300 {
        let mut el = El::new(random_cfg(&mut g, 1, false));
        let headroom = g.range(0, 8);
        let mut prev = BLOCK_LONG;
        for frame in 0..40 {
            randomize(&mut g, &mut el, false);
            let ws = if g.range(0, 3) == 0 {
                g.range(0, 3)
            } else {
                [BLOCK_START, 2, 3, 0][prev as usize]
            };
            prev = ws;
            el.ch[0].ics_info.window_sequence = ws;
            el.ch[0].transform_splitting_active = (ws == BLOCK_START && g.range(0, 1) != 0) as u8;
            el.sync_to_c();
            let mut out_c = vec![0i32; 1024];
            let mut out_r = vec![0i32; 1024];
            unsafe { cref_core_el_f2t(el.c, 0, out_c.as_mut_ptr(), 1024, 1, headroom, 0) };
            {
                let El { st, ch, spec, wb, .. } = &mut el;
                cblock_frequency_to_time(
                    &mut st[0],
                    &mut ch[0],
                    &mut spec[..1024],
                    &mut out_r,
                    1024,
                    1,
                    wb.mdct_out_temp(),
                    headroom,
                    0,
                    0,
                );
            }
            assert_same(&format!("iter {iter} frame {frame}: output"), &out_c, &out_r);
            el.check(&format!("iter {iter} frame {frame}: f2t"));
        }
    }
}
