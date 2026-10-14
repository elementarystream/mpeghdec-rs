// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libMpeghDec aac_rom.cpp and channelinfo.cpp: all ROM tables,
//! getSamplingRateInfo() for all indices and sampling rates, IcsRead()/IcsReadMaxSfb() on random
//! bitstreams.
#![cfg(not(cref_missing))]

mod core_common;

use core_common::*;
use mpeghdec_core::aac_rom::*;
use mpeghdec_core::channelinfo::*;
use mpeghdec_cref::core_rom::*;
use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};

fn rust_rom(id: i32) -> Vec<i32> {
    let mut o = Vec::new();
    match id {
        0 => MANTISSA_TABLE.iter().for_each(|r| o.extend_from_slice(r)),
        1 => o.extend_from_slice(&INVERSE_QUANT_TABLE_MANT),
        2 => o.extend(INVERSE_QUANT_TABLE_EXP.iter().map(|&v| v as i32)),
        3 => o.extend_from_slice(&INVERSE_QUANT_INTERPOLATION_TABLE),
        4 => {
            o.extend_from_slice(&A2_TO_POW0P25_MANT);
            o.push(A2_TO_POW0P25_EXP);
            o.push(INVERSE_QUANT_INTERPOLATION_EXP);
        }
        5 => {
            for row in &SFB_OFFSET_TABLES {
                for s in row {
                    o.push(s.number_of_sfb_long as i32);
                    o.push(s.number_of_sfb_short as i32);
                    o.push(!s.sfb_offset_long.is_empty() as i32);
                    o.push(!s.sfb_offset_short.is_empty() as i32);
                    if !s.sfb_offset_long.is_empty() {
                        assert_eq!(s.sfb_offset_long.len(), s.number_of_sfb_long as usize + 1);
                        o.extend(s.sfb_offset_long.iter().map(|&v| v as i32));
                    }
                    if !s.sfb_offset_short.is_empty() {
                        assert_eq!(s.sfb_offset_short.len(), s.number_of_sfb_short as usize + 1);
                        o.extend(s.sfb_offset_short.iter().map(|&v| v as i32));
                    }
                }
            }
        }
        6 => {
            let d = &AAC_CODE_BOOK_DESCRIPTION_SCL;
            o.extend([d.dimension, d.num_bits, d.offset].map(|v| v as i32));
            assert_eq!(d.code_book.len(), 65);
            d.code_book.iter().for_each(|r| o.extend(r.iter().map(|&v| v as i32)));
        }
        7 => TNS_MAX_BANDS_TBL.iter().for_each(|r| o.extend(r.iter().map(|&v| v as i32))),
        8 => o.extend_from_slice(&FDKAACDEC_TNS_COEFF3),
        9 => o.extend_from_slice(&FDKAACDEC_TNS_COEFF4),
        10 => o.extend(AACDEC_RANDOM_SIGN.iter().map(|&v| v as i32)),
        11 => MDST_FILT_COEF_CURR.iter().for_each(|r| o.extend_from_slice(r)),
        12 => MDST_FILT_COEF_PREV.iter().for_each(|r| o.extend_from_slice(r)),
        _ => unreachable!(),
    }
    o
}

#[test]
fn rom_tables() {
    for id in 0..=12 {
        let c = c_dump(|o| unsafe { cref_core_rom_dump(id, o) });
        assert!(!c.is_empty());
        assert_same(&format!("table {id}"), &c, &rust_rom(id));
    }
}

fn rust_sri(samples_per_frame: u32, sr_idx: u32, sr: u32) -> Vec<i32> {
    let mut t = SamplingRateInfo::default();
    let err = get_sampling_rate_info(&mut t, samples_per_frame, sr_idx, sr);
    let mut o = vec![err, t.sampling_rate_index as i32, t.sampling_rate as i32];
    o.push(t.number_of_scale_factor_bands_long as i32);
    o.push(t.number_of_scale_factor_bands_short as i32);
    o.push(!t.scale_factor_bands_long.is_empty() as i32);
    o.push(!t.scale_factor_bands_short.is_empty() as i32);
    if !t.scale_factor_bands_long.is_empty() {
        o.extend(
            t.scale_factor_bands_long[..=t.number_of_scale_factor_bands_long as usize]
                .iter()
                .map(|&v| v as i32),
        );
    }
    if !t.scale_factor_bands_short.is_empty() {
        o.extend(
            t.scale_factor_bands_short[..=t.number_of_scale_factor_bands_short as usize]
                .iter()
                .map(|&v| v as i32),
        );
    }
    o
}

#[test]
fn sampling_rate_info() {
    let mut rng = Rng::new(7);
    let mut cases = Vec::new();
    for spf in [1024u32, 768, 960, 2048, 0] {
        for idx in 0..40u32 {
            for sr in
                [0u32, 7350, 8000, 9391, 9390, 11502, 44100, 48000, 92017, 92016, 96000, 200000]
            {
                cases.push((spf, idx, sr));
            }
        }
    }
    for _ in 0..20000 {
        let spf = if rng.next_u32() % 4 == 0 { rng.next_u32() % 2100 } else { 1024 };
        let idx = rng.next_u32() % 40;
        let sr = match rng.next_u32() % 3 {
            0 => rng.next_u32(),
            _ => rng.next_u32() % 130000,
        };
        cases.push((spf, idx, sr));
    }
    for (spf, idx, sr) in cases {
        let c = c_dump(|o| unsafe { cref_core_get_sampling_rate_info(spf, idx, sr, o) });
        assert_same(
            &format!("getSamplingRateInfo({spf}, {idx}, {sr})"),
            &c,
            &rust_sri(spf, idx, sr),
        );
    }
}

fn ics_to_arr(ics: &CIcsInfo) -> [i32; 16] {
    let mut o = Vec::new();
    dump_ics(&mut o, ics);
    o.try_into().unwrap()
}

#[test]
fn ics_read_random() {
    let mut rng = Rng::new(11);
    for iter in 0..30000 {
        let sr_idx = (rng.next_u32() % 13) as i32;
        let flags = match rng.next_u32() % 4 {
            0 => 0,
            1 => AC_USAC,
            2 => AC_ELD,
            _ => AC_MPEGH3DA | (rng.next_u32() & AC_INDEP),
        };
        let max_sfb_only = (rng.next_u32() % 4 == 0) as i32;
        let mut ics = CIcsInfo::default();
        {
            let mut a = [0i32; 16];
            for v in a.iter_mut() {
                *v = (rng.next_u32() % 256) as i32;
            }
            a[11] = (rng.next_u32() % 4) as i32;
            load_ics(&mut Rd::new(&a), &mut ics);
        }
        let buf: Vec<u8> = (0..8).map(|_| rng.next_u32() as u8).collect();
        let mut buf_c = buf.clone();
        let size = buf.len() as u32;
        let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, size * 8, 0) };
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&buf), size, size * 8, FdkBsCfg::BsReader);

        let mut sri = SamplingRateInfo::default();
        get_sampling_rate_info(&mut sri, 1024, sr_idx as u32, 48000);
        let mut arr = ics_to_arr(&ics);
        let err_c =
            unsafe { cref_core_ics_read(hc, arr.as_mut_ptr(), sr_idx, flags, max_sfb_only) };
        let err_r = if max_sfb_only != 0 {
            ics_read_max_sfb(&mut bs, &mut ics, &sri)
        } else {
            ics_read(&mut bs, &mut ics, &sri, flags)
        };
        assert_eq!(err_c, err_r, "iter {iter}");
        assert_same(&format!("iter {iter}"), &arr, &ics_to_arr(&ics));
        assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits(), "iter {iter}");
        unsafe { cref_bs_free(hc) };
    }
}

/// Random values of the state layouts of shim/core_common.cpp survive a load into the C
/// structures and a dump (checks that the C and Rust layouts match field by field).
#[test]
fn state_layouts_round_trip() {
    use mpeghdec_core::conceal_types::CConcealParams;
    use mpeghdec_core::stereo::*;
    use mpeghdec_cref::core_common::*;
    let mut rng = Rng::new(3);
    let mut rnd = |a: &mut [i32], m: u32| {
        for v in a.iter_mut() {
            *v = (rng.next_u32() % m) as i32;
        }
    };
    for _ in 0..20 {
        let mut ch = ChPair::new();
        let mut a = dump_ch(&ch.r);
        rnd(&mut a, 100);
        load_ch(&mut ch.r, &a);
        ch.sync_to_c();
        ch.check("ch");

        let mut st = StPair::new();
        let mut a = dump_st(&st.r);
        rnd(&mut a, 100);
        a[7] = -1; /* prev_wrs is not loaded */
        load_st(&mut st.r, &a);
        st.sync_to_c();
        st.check("st");

        let mut p = CConcealParams::default();
        let mut a = dump_conceal_params(&p);
        rnd(&mut a, 30000);
        load_conceal_params(&mut p, &a);
        let cp = unsafe { cref_core_conceal_params_new() };
        unsafe { cref_core_conceal_params_load(cp, a.as_ptr()) };
        let c = c_dump(|o| unsafe { cref_core_conceal_params_dump(cp, o) });
        assert_same("conceal params", &c, &dump_conceal_params(&p));
        unsafe { cref_core_conceal_params_free(cp) };

        let mut wb = vec![0i32; 2048];
        let com = unsafe { cref_core_com_new(1, 1, wb.as_mut_ptr()) };
        let mut j = CJointStereoData::default();
        let mut a = dump_jsd(&j);
        rnd(&mut a, 200);
        load_jsd(&mut j, &a);
        unsafe { cref_core_jsd_load(com, a.as_ptr()) };
        assert_same("jsd", &c_dump(|o| unsafe { cref_core_jsd_dump(com, o) }), &dump_jsd(&j));
        let mut cp = CCplxPredictionData::default();
        let mut a = dump_cpd(&cp);
        rnd(&mut a, 200);
        load_cpd(&mut cp, &a);
        unsafe { cref_core_cpd_load(com, a.as_ptr()) };
        assert_same("cpd", &c_dump(|o| unsafe { cref_core_cpd_dump(com, o) }), &dump_cpd(&cp));
        unsafe { cref_core_com_free(com) };

        let mut cpe = CpePersistentData::default();
        let mut a = dump_cpe(&cpe);
        rnd(&mut a, 200);
        load_cpe(&mut cpe, &a);
        let c = unsafe { cref_core_cpe_new() };
        unsafe { cref_core_cpe_load(c, a.as_ptr()) };
        assert_same("cpe", &c_dump(|o| unsafe { cref_core_cpe_dump(c, o) }), &dump_cpe(&cpe));
        unsafe { cref_core_cpe_free(c) };
    }
}
