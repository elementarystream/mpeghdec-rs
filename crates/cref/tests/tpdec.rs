// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libMpegTPDec: the C++ and the Rust transport decoder are fed the same
//! data (the MHAS test streams, corrupted and truncated variants, synthetic MHAS streams with
//! generated configs and audio scene information, random data) and their complete internal
//! state, return values and callback sequences are compared after every call.
//!
//! The test streams are read from `$MPEGHDEC_TEST_STREAMS` (default `work/streams`); tests
//! using them are skipped if the directory does not exist. `MPEGHDEC_TPDEC_FULL=1` processes the
//! complete files.
#![cfg(not(cref_missing))]

mod tpdec_common;

use mpeghdec_cref::testutil::Rng;
use mpeghdec_cref::tpdec::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_tpdec::*;
use tpdec_common::*;

fn reader(data: &[u8]) -> (Vec<u8>, u32) {
    let size = (data.len() + 4).next_power_of_two().max(16);
    let mut buf = vec![0u8; size];
    buf[..data.len()].copy_from_slice(data);
    (buf, (data.len() * 8) as u32)
}

/// Parses `data` as mpegh3daConfig() with both implementations and compares the results.
fn compare_config_parse(p: &mut Pair, data: &[u8], sub_stream_index: i32) -> i32 {
    let (mut buf_c, valid) = reader(data);
    let buf_r = buf_c.clone();
    let mut out_c = vec![0i32; 20000];
    let n = unsafe {
        cref_tp_parseConfig(
            p.c,
            buf_c.as_mut_ptr(),
            buf_c.len() as u32,
            valid,
            sub_stream_index,
            out_c.as_mut_ptr(),
        )
    } as usize;
    let mut bs = FdkBitStream::default();
    bs.init(&BitMem::from_slice(&buf_r), buf_r.len() as u32, valid, FdkBsCfg::BsReader);
    let mut asc = CSAudioSpecificConfig::default();
    let target_layout = p.r.as_ref().unwrap().target_layout;
    let err = mpegh3da_config_parse(
        &mut asc,
        &mut bs,
        0,
        &mut p.rec,
        AC_CM_DET_CFG_CHANGE,
        0,
        target_layout,
        sub_stream_index,
    );
    let mut out_r = vec![err, bs.get_valid_bits()];
    put_asc(&mut out_r, &asc);
    assert_eq!(out_r.as_slice(), &out_c[..n], "config parse of {data:02x?}");
    p.check("config parse");
    err
}

#[test]
fn config_parser_generated() {
    let mut g = Rng::new(0x5eed_c0f1);
    let mut stats = [0u32; 3];
    let iters = if full() { 60_000 } else { 12_000 };
    for it in 0..iters {
        let reg = if g.range(0, 3) == 0 { g.next_u32() & REG_ALL } else { REG_ALL };
        let fail =
            if g.range(0, 4) == 0 { (g.next_u32(), g.range(2, 30) as u32, 0) } else { (0, 0, 0) };
        let mut p = Pair::new(TT_MHAS, 1, reg, fail, &format!("config {it}"));
        if g.range(0, 3) == 0 {
            p.set_param(TPDEC_PARAM_TARGETLAYOUT, g.range(0, 20));
        }
        for _ in 0..4 {
            let mut c = gen_config(&mut g);
            if g.range(0, 3) == 0 {
                c = mutate(&mut g, &c);
            }
            let ssi = if g.range(0, 4) == 0 { g.range(1, 3) } else { 0 };
            let err = compare_config_parse(&mut p, &c, ssi);
            stats[match err {
                TRANSPORTDEC_OK => 0,
                TRANSPORTDEC_PARSE_ERROR => 1,
                _ => 2,
            }] += 1;
        }
    }
    println!("config parse results (ok, parse error, other): {stats:?}");
    assert!(stats.iter().all(|&s| s > 100), "poor coverage: {stats:?}");
}

#[test]
fn config_parser_random_bytes() {
    let mut g = Rng::new(77);
    let mut p = Pair::new(TT_MHAS, 1, REG_ALL, (0, 0, 0), "random config");
    for _ in 0..20_000 {
        let n = g.range(0, 64) as usize;
        let d: Vec<u8> = (0..n).map(|_| g.next_u32() as u8).collect();
        compare_config_parse(&mut p, &d, 0);
    }
}

#[test]
fn config_parser_real_configs() {
    let files = mhas_files();
    if files.is_empty() {
        eprintln!("test streams not found, skipping");
        return;
    }
    let mut g = Rng::new(3);
    let mut p = Pair::new(TT_MHAS, 1, REG_ALL, (0, 0, 0), "real config");
    for f in &files {
        let data = read_prefix(f, 300_000);
        let pk = parse_mhas(&data);
        let cfg = pk.iter().find(|x| x.ptype == 1).expect("config packet");
        assert_eq!(compare_config_parse(&mut p, &cfg.payload, 0), TRANSPORTDEC_OK);
        for _ in 0..300 {
            let m = mutate(&mut g, &cfg.payload);
            compare_config_parse(&mut p, &m, g.range(0, 3));
        }
    }
}

/// Parses `data` as mae_AudioSceneInfo() into the ASI of both callback contexts.
fn compare_asi_parse(p: &mut Pair, data: &[u8], num_max_ids: i32, stream_index: i32) -> i32 {
    let (mut buf_c, valid) = reader(data);
    let buf_r = buf_c.clone();
    let mut vc = 0;
    let ec = unsafe {
        cref_tp_parseAsi(
            p.c,
            buf_c.as_mut_ptr(),
            buf_c.len() as u32,
            valid,
            num_max_ids,
            stream_index,
            &mut vc,
        )
    };
    let mut bs = FdkBitStream::default();
    bs.init(&BitMem::from_slice(&buf_r), buf_r.len() as u32, valid, FdkBsCfg::BsReader);
    let er = mae_audio_scene_info(&mut p.rec.asi, &mut bs, num_max_ids, stream_index);
    assert_eq!((er, bs.get_valid_bits()), (ec, vc), "asi parse of {data:02x?}");
    p.check("asi parse");
    er
}

#[test]
fn asi_parser() {
    let mut g = Rng::new(0xa51);
    let mut ok = 0;
    let iters = if full() { 20_000 } else { 5_000 };
    for it in 0..iters {
        let mut p = Pair::new(TT_MHA_RAW, 1, REG_ALL, (0, 0, 0), &format!("asi {it}"));
        // a config for checkASI()
        let cfg = gen_config(&mut g);
        p.oob_config(&cfg, 0);
        for i in 0..ASI_MAX_PREF_DESCR_LANGUAGES as i32 {
            if g.range(0, 2) == 0 {
                let l: [u8; 3] = if g.range(0, 1) == 0 { *b"eng" } else { *b"ger" };
                unsafe { cref_tp_asiSetPrefLang(p.c, i, l.as_ptr()) };
                p.rec.asi.pref_descr_languages[i as usize] = l.map(|x| x as i8);
            }
        }
        let dmx = g.range(0, 3);
        unsafe { cref_tp_asiSetActiveDmxId(p.c, dmx) };
        p.rec.asi.active_dmx_id = dmx as u8;
        for _ in 0..6 {
            let main = g.range(0, 4) != 0;
            let mut a = gen_asi(&mut g, main, 56).bytes;
            if g.range(0, 4) == 0 {
                a = mutate(&mut g, &a);
            }
            let si = if main { 0 } else { g.range(0, 3) };
            let max_ids = if g.range(0, 3) == 0 { g.range(0, 60) } else { 56 };
            if compare_asi_parse(&mut p, &a, max_ids, si) == TRANSPORTDEC_OK {
                ok += 1;
            }
            let si = g.range(0, 3);
            let ec = unsafe { cref_tp_checkAsi(p.c, 0, si) };
            let r = p.r.as_ref().unwrap();
            let er = check_asi(
                &p.rec.asi,
                r.asc[0].m_sc.bs_num_signal_groups as i32,
                &r.asc[0].m_sc.m_signal_group_type,
                si,
            );
            assert_eq!(er, ec, "checkASI");
            let (gi, si2, pi) = (g.range(0, 30), g.range(0, 31), g.range(0, 31));
            let ids_c = unsafe { cref_tp_asiIds(p.c, gi, si2, pi) };
            let a = &p.rec.asi;
            let ids_r = ((asi_group_id2idx(a, gi) + 1) << 16)
                | ((asi_switch_group_id2idx(a, si2) + 1) << 8)
                | (asi_group_preset_id2idx(a, pi) + 1);
            assert_eq!(ids_r, ids_c);
            match g.range(0, 10) {
                0 => {
                    unsafe { cref_tp_asiReset(p.c) };
                    asi_reset(&mut p.rec.asi);
                }
                1 => {
                    unsafe { cref_tp_asiResetAvailability(p.c) };
                    asi_reset_availability(&mut p.rec.asi);
                }
                _ => {}
            }
            p.check("asi ops");
        }
    }
    println!("asi parsed ok: {ok}");
    assert!(ok > 1000);
}

#[test]
fn iso639_codes() {
    let mut g = Rng::new(9);
    let chars: Vec<u8> = b"aAzZkoenghdsqiEN\0 1@\x7f\x80\xff".to_vec();
    let check = |b: [u8; 3]| {
        let mut c = b;
        let rc = unsafe { cref_tp_isoMap(c.as_mut_ptr()) };
        let mut r = b.map(|x| x as i8);
        asi_map_iso639_2_t2b_and_tolower(&mut r);
        let rr = asi_check_iso639_2(&mut r);
        assert_eq!((rr, r.map(|x| x as u8)), (rc, c), "{b:?}");
    };
    for &a in &chars {
        for &b in &chars {
            for &c in &chars {
                check([a, b, c]);
            }
        }
    }
    for code in [
        b"sqi", b"hye", b"eus", b"bod", b"mya", b"ces", b"zho", b"cym", b"deu", b"nld", b"ell",
        b"fas", b"fra", b"kat", b"isl", b"mkd", b"mri", b"msa", b"ron", b"slk", b"ko\0", b"en\0",
        b"DEU", b"Fra",
    ] {
        check(*code);
    }
    for _ in 0..100_000 {
        check([g.next_u32() as u8, g.next_u32() as u8, g.next_u32() as u8]);
    }
}

#[test]
fn earcon_and_pcm_config() {
    let mut g = Rng::new(10);
    for _ in 0..50_000 {
        let n = g.range(0, 40) as usize;
        let d: Vec<u8> = (0..n).map(|_| g.next_u32() as u8).collect();
        let (mut buf_c, valid) = reader(&d);
        let buf_r = buf_c.clone();
        let mut out = [0i32; 8];
        let ec = unsafe {
            cref_tp_earconInfo(buf_c.as_mut_ptr(), buf_c.len() as u32, valid, out.as_mut_ptr())
        };
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&buf_r), buf_r.len() as u32, valid, FdkBsCfg::BsReader);
        let mut info = EarconInfo::default();
        let er = earcon_info(&mut bs, &mut info);
        assert_eq!((er, info.m_num_earcons as i32, bs.get_valid_bits()), (ec, out[0], out[1]));

        let ec = unsafe {
            cref_tp_pcmDataConfig(buf_c.as_mut_ptr(), buf_c.len() as u32, valid, out.as_mut_ptr())
        };
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&buf_r), buf_r.len() as u32, valid, FdkBsCfg::BsReader);
        let mut cf = EarconConfig::default();
        let er = pcm_data_config(&mut bs, &mut cf);
        let r = [
            cf.m_num_pcm_signals as i32,
            cf.m_bs_pcm_frame_size_index as i32,
            cf.m_bs_pcm_frame_size as i32,
            cf.m_bs_pcm_loudness_value as i32,
            cf.m_pcm_has_attenuation_gain as i32,
            cf.m_bs_pcm_attenuation_gain as i32,
            bs.get_valid_bits(),
        ];
        assert_eq!((er, r), (ec, [out[0], out[1], out[2], out[3], out[4], out[5], out[6]]));
    }
}

#[test]
fn program_config() {
    let mut g = Rng::new(11);
    for _ in 0..2000 {
        let n = g.range(0, 40);
        let mut types: Vec<i32> = (0..n).map(|_| g.range(-1, 21)).collect();
        let mut out = vec![0i32; 256];
        let m = unsafe { cref_tp_programConfig(types.as_mut_ptr(), n, out.as_mut_ptr()) } as usize;
        let mut pce = CProgramConfig::default();
        cprogram_config_init(&mut pce);
        let mut el_list = [ID_NONE; 64];
        let (mut chm, mut chi) = ([0u8; 8], [0u8; 8]);
        let mut cht = [0i32; 8];
        let mut r = Vec::new();
        for i in 0..n as usize {
            let mut elm = 0xAAu8;
            r.push(cprogram_config_lookup_element(
                &mut pce,
                0,
                0,
                0,
                &mut chm,
                &mut cht,
                &mut chi,
                8,
                &mut elm,
                &mut el_list,
                types[i],
            ));
            r.push(elm as i32);
            r.push(pce.el_counter as i32);
            if i == n as usize / 2 {
                cprogram_config_reset(&mut pce);
            }
        }
        r.push(cprogram_config_is_valid(&pce));
        r.extend_from_slice(&el_list);
        assert_eq!(r.as_slice(), &out[..m]);
    }
}

#[test]
fn open_formats() {
    for fmt in [-1, 0, 1, 2, 6, 7, 10, 12, 20, 24, 60, 99] {
        for layers in 0..6 {
            let p = Pair::new(fmt, layers, REG_ALL, (0, 0, 0), "open");
            drop(p);
        }
    }
}

/// Bytes processed per stream: `$MPEGHDEC_TPDEC_LIMIT`, everything with `MPEGHDEC_TPDEC_FULL=1`.
fn stream_limit() -> usize {
    if full() {
        return usize::MAX;
    }
    std::env::var("MPEGHDEC_TPDEC_LIMIT").ok().and_then(|v| v.parse().ok()).unwrap_or(160_000)
}

/// One track per program unless the full test was requested.
fn selected_files() -> Vec<std::path::PathBuf> {
    mhas_files().into_iter().filter(|f| full() || f.to_string_lossy().contains("_a.")).collect()
}

#[test]
fn real_streams_mhas() {
    let files = selected_files();
    if files.is_empty() {
        eprintln!("test streams not found, skipping");
        return;
    }
    for (i, f) in files.iter().enumerate() {
        let data = read_prefix(f, stream_limit());
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        let prm = Params { clean: true, layers: 1, reg: REG_ALL, fail: (0, 0, 0), params: vec![] };
        let (ok, steps) = run_mhas_stream(&data, i as u64, &prm, &name);
        println!("{name}: TT_MHAS {ok} AUs, {steps} checks");
        // TT_MHAS needs MHA_PACTYP_SYNC packets for synchronization (not sent by all streams).
        let has_sync = parse_mhas(&data).iter().any(|x| x.ptype == 6);
        assert!(ok > 100 || !has_sync, "{name}: only {ok} access units");
        let mut g = Rng::new(1000 + i as u64);
        let prm = random_params(&mut g);
        run_mhas_stream(&data[..data.len().min(60_000)], 100 + i as u64, &prm, &name);
    }
}

#[test]
fn real_streams_packetized() {
    let files = selected_files();
    if files.is_empty() {
        eprintln!("test streams not found, skipping");
        return;
    }
    for (i, f) in files.iter().enumerate() {
        let data = read_prefix(f, stream_limit());
        let aus = au_list(f, data.len());
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        let prm = Params { clean: true, layers: 1, reg: REG_ALL, fail: (0, 0, 0), params: vec![] };
        let (ok, steps) = run_mhas_packetized(&data, &aus, i as u64, &prm, &name);
        println!("{name}: TT_MHAS_PACKETIZED {ok} of {} AUs, {steps} checks", aus.len());
        assert!(ok > 100, "{name}: only {ok} access units");
        let mut g = Rng::new(3000 + i as u64);
        let prm = random_params(&mut g);
        run_mhas_packetized(&data, &aus[..aus.len().min(200)], 300 + i as u64, &prm, &name);
    }
}

#[test]
fn real_streams_raw() {
    let files = selected_files();
    if files.is_empty() {
        eprintln!("test streams not found, skipping");
        return;
    }
    for (i, f) in files.iter().enumerate() {
        let data = read_prefix(f, stream_limit());
        let pk = parse_mhas(&data);
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        let prm = Params { clean: true, layers: 1, reg: REG_ALL, fail: (0, 0, 0), params: vec![] };
        let (ok, steps) = run_mha_raw(&pk, i as u64, &prm, &name);
        println!("{name}: TT_MHA_RAW {ok} AUs, {steps} checks");
        assert!(ok > 100);
        let mut g = Rng::new(2000 + i as u64);
        let prm = random_params(&mut g);
        run_mha_raw(&pk[..pk.len().min(400)], 200 + i as u64, &prm, &name);
    }
}

#[test]
fn corrupted_streams() {
    let files = selected_files();
    if files.is_empty() {
        eprintln!("test streams not found, skipping");
        return;
    }
    let mut g = Rng::new(0xbad);
    let n_variants = if full() { 40 } else { 8 };
    for (fi, f) in files.iter().enumerate() {
        let data = read_prefix(f, 50_000);
        let aus = au_list(f, data.len());
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        for v in 0..n_variants {
            let mut d = data.clone();
            let mut aus_v = aus.clone();
            match v % 4 {
                0 => {
                    // sparse bit errors
                    for _ in 0..g.range(1, 40) {
                        let i = g.range(0, d.len() as i32 * 8 - 1) as usize;
                        d[i / 8] ^= 0x80 >> (i % 8);
                    }
                }
                1 => {
                    // byte deletions / insertions (AU boundaries become wrong)
                    for _ in 0..g.range(1, 10) {
                        let i = g.range(0, d.len() as i32 - 1) as usize;
                        if g.range(0, 1) == 0 {
                            d.remove(i);
                        } else {
                            d.insert(i, g.next_u32() as u8);
                        }
                    }
                    aus_v.retain(|&(o, n)| o + n <= d.len());
                }
                2 => {
                    // truncated stream and truncated access units
                    let n = g.range(1000, d.len() as i32) as usize;
                    d.truncate(n);
                    aus_v = aus
                        .iter()
                        .filter(|&&(o, _)| o < n)
                        .map(|&(o, s)| (o, if g.range(0, 5) == 0 { s / 2 } else { s.min(n - o) }))
                        .collect();
                }
                _ => {
                    // burst of garbage
                    let i = g.range(0, d.len() as i32 - 600) as usize;
                    for k in 0..g.range(1, 500) as usize {
                        d[i + k] = g.next_u32() as u8;
                    }
                }
            }
            let prm = random_params(&mut g);
            let nm = format!("{name} variant {v}");
            run_mhas_stream(&d, g.next_u64(), &prm, &nm);
            run_mhas_packetized(&d, &aus_v, g.next_u64(), &prm, &nm);
            if v % 2 == 0 {
                run_mha_raw(&parse_mhas(&d), g.next_u64(), &prm, &nm);
            }
        }
        if !full() && fi >= 5 {
            break;
        }
    }
}

#[test]
fn random_data() {
    let mut g = Rng::new(0xdead);
    let iters = if full() { 3000 } else { 600 };
    for it in 0..iters {
        let n = g.range(0, 5000) as usize;
        let mut d: Vec<u8> = (0..n).map(|_| g.next_u32() as u8).collect();
        if g.range(0, 2) == 0 {
            // MHAS-like: start with a sync packet
            let mut s = Vec::new();
            put_packet(&mut s, 6, 0, &[0xa5]);
            s.extend_from_slice(&d);
            d = s;
        }
        let prm = random_params(&mut g);
        let name = format!("random {it}");
        run_mhas_stream(&d, g.next_u64(), &prm, &name);
        let mut aus = Vec::new();
        let mut o = 0;
        while o < d.len() {
            let s = (g.range(1, 800) as usize).min(d.len() - o);
            aus.push((o, s));
            o += s;
        }
        run_mhas_packetized(&d, &aus, g.next_u64(), &prm, &name);
        let pk = vec![
            MhasPacket {
                ptype: 1,
                label: 1,
                payload: d[..d.len().min(g.range(0, 100) as usize)].to_vec(),
            },
            MhasPacket { ptype: 2, label: 1, payload: d.clone() },
        ];
        run_mha_raw(&pk, g.next_u64(), &prm, &name);
    }
}

#[test]
fn synthetic_streams() {
    let base = mhas_files().first().map(|f| {
        let pk = parse_mhas(&read_prefix(f, 100_000));
        pk.iter().find(|x| x.ptype == 1).map(|x| x.payload.clone()).unwrap()
    });
    let mut g = Rng::new(0x5717);
    let iters = if full() { 4000 } else { 700 };
    let mut total_ok = [0u64; 3];
    for it in 0..iters {
        let n_aus = g.range(1, 40) as usize;
        let (d, aus) = gen_mhas(&mut g, n_aus, base.as_deref());
        let prm = random_params(&mut g);
        let name = format!("synthetic {it}");
        let seeds = [g.next_u64(), g.next_u64(), if it % 4 == 0 { g.next_u64() } else { 0 }];
        if std::env::var("TPDEC_ONLY").is_ok_and(|v| v != it.to_string()) {
            continue;
        }
        total_ok[0] += run_mhas_stream(&d, seeds[0], &prm, &name).0;
        total_ok[1] += run_mhas_packetized(&d, &aus, seeds[1], &prm, &name).0;
        if it % 4 == 0 {
            total_ok[2] += run_mha_raw(&parse_mhas(&d), seeds[2], &prm, &name).0;
        }
    }
    println!("synthetic streams: AUs decoded (mhas, packetized, raw): {total_ok:?}");
    assert!(total_ok.iter().all(|&n| n > 100));
    let st = STATS.lock().unwrap();
    println!("callback calls by type (process wide): {:?}", st.0);
    let states: Vec<(u64, u64)> =
        (0..64).filter(|b| st.1 >> b & 1 != 0).map(|b| (b / 8, b % 8)).collect();
    println!("(flushStatus, buildUpStatus) seen in cbCtrlCFGChange: {states:?}");
    if std::env::var("TPDEC_ONLY").is_err() {
        assert!(st.0[1..=10].iter().all(|&n| n > 0), "callback not covered: {:?}", st.0);
    }
}

/// Writes an explicit speaker layout (speakerLayoutType 2) of `n` speakers at non-center
/// azimuths; the last speaker adds a symmetric pair, which makes SpeakerConfig3d() write one
/// speaker beyond the signalled number.
fn explicit_layout(w: &mut BitWriter, n: u32, az0: u32, last_pair: bool) {
    w.put(2, 2);
    w.put_escaped(n - 1, 5, 8, 16);
    w.put(1, 1); // angularPrecision
    for i in 0..n {
        w.put(0, 1); // isCICPspeakerIdx
        w.put(0, 2); // ElevationClass
        w.put(az0 + i, 8); // AzimuthAngleIdx
        w.put(1, 1); // AzimuthDirection
        w.put(0, 1); // isLFE
        w.put((last_pair && i == n - 1) as u32, 1); // alsoAddSymmetricPair
    }
}

fn config_header(w: &mut BitWriter) {
    w.put(0x0b, 8);
    w.put(3, 5);
    w.put(1, 3);
    w.put(0, 2);
}

fn cpe_elements(w: &mut BitWriter, n: u32) {
    w.put_escaped(n - 1, 4, 8, 16);
    w.put(0, 1); // elementLengthPresent
    for _ in 0..n {
        w.put(1, 2); // ID_USAC_CPE
        w.put(0, 4); // tw_mdct, fullbandLpd, noiseFilling, enhancedNoiseFilling
        w.put(0, 2); // qceIndex
        w.put(0, 1); // shiftIndex1
        w.put(0, 1); // lpdStereoIndex
    }
    w.put(0, 1); // usacConfigExtensionPresent
}

#[test]
fn speaker_layout_overflow() {
    let mut p = Pair::new(TT_MHAS, 1, REG_ALL, (0, 0, 0), "speaker overflow");
    // Reference layout: 28 speakers plus a symmetric pair -> writes speakers[28], i.e. into
    // bsNumSignalGroups .. objNumSignalGroups.
    for az0 in [1u32, 20, 100, 200] {
        let mut w = BitWriter::default();
        config_header(&mut w);
        explicit_layout(&mut w, 28, az0, true);
        w.put(0, 5); // one signal group
        w.put(0, 3); // channels
        w.put_escaped(1, 5, 8, 16); // 2 signals
        w.put(1, 1); // differsFromReferenceLayout
        w.put(0, 2); // CICP layout
        w.put(2, 6); // stereo
        cpe_elements(&mut w, 1);
        compare_config_parse(&mut p, &w.bytes, 0);
        // the overflow wrote the azimuth of the extra speaker into numAudioObjects and
        // numHOATransportChannels
        let asc = rust_parse(&mut p, &w.bytes);
        let az = (az0 + 27) as i16; // -(-(az0 + 27))
        assert_eq!(asc.m_sc.num_audio_objects, az as u16 as u8);
        assert_eq!(asc.m_sc.num_hoa_transport_channels, (az as u16 >> 8) as u8);
    }
    // Two signal groups of 28 channels; the second one's explicit layout with a trailing pair
    // writes m_signalGroupTypeSpeakers[56], i.e. into referenceLayout and speakers[0].
    for (az0, pair1) in [(1u32, false), (40, true), (150, true)] {
        let mut w = BitWriter::default();
        config_header(&mut w);
        w.put(0, 2);
        w.put(2, 6); // reference layout: CICP 2
        w.put(1, 5); // two signal groups
        for pair in [pair1, true] {
            w.put(0, 3);
            w.put_escaped(27, 5, 8, 16);
            w.put(1, 1);
            explicit_layout(&mut w, 28, az0, pair);
        }
        cpe_elements(&mut w, 28);
        let err = compare_config_parse(&mut p, &w.bytes, 0);
        assert_eq!(err, TRANSPORTDEC_OK);
        // the overflow wrote the Lfe flag of the extra speaker into referenceLayout
        assert_eq!(rust_parse(&mut p, &w.bytes).m_sc.reference_layout, 0);
    }
}

fn rust_parse(_p: &mut Pair, data: &[u8]) -> CSAudioSpecificConfig {
    let (buf, valid) = reader(data);
    let mut bs = FdkBitStream::default();
    bs.init(&BitMem::from_slice(&buf), buf.len() as u32, valid, FdkBsCfg::BsReader);
    let mut asc = CSAudioSpecificConfig::default();
    let mut rec = Rec::new(REG_ALL, 0, 0, 0);
    mpegh3da_config_parse(&mut asc, &mut bs, 0, &mut rec, AC_CM_DET_CFG_CHANGE, 0, 0, 1);
    asc
}
