// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libUIManager and libMpeghUIMan: the C++ and the Rust UI manager are
//! driven through the public `mpegh_UI_*` API (the MHAS test streams, synthetic streams with rich
//! audio scene information, corrupted variants) and through the `UI_Manager_*` API used by the
//! core decoder, with random sequences of XML action events, scene state requests (with all
//! buffer sizes) and persistence memory handling. Return codes, rewritten MHAS bytes, complete
//! XML output buffers, persistence memory blocks and the complete internal states are compared
//! after every call. The XML action parser and the persistence manager are also compared on their
//! own with random data.
//!
//! The test streams are read from `$MPEGHDEC_TEST_STREAMS` (default `work/streams`); tests
//! using them are skipped if the directory does not exist. `MPEGHDEC_TPDEC_FULL=1` runs longer
//! variants.
#![cfg(not(cref_missing))]

mod tpdec_common;

use mpeghdec_cref::testutil::Rng;
use mpeghdec_cref::uimanager::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_tpdec::{mae_audio_scene_info, AudioSceneInfo};
use mpeghdec_uimanager::ui_persistence::UiPersistenceManager;
use mpeghdec_uimanager::ui_xml::ui_manager_parse_xml_action;
use mpeghdec_uimanager::*;
use tpdec_common::{au_list, full, gen_mhas, mhas_files, mutate, put_asi, put_packet, BitWriter};

// ---- state dumps (layout of crates/cref/shim/uimanager.cpp) ----

fn fnv(p: &[u8]) -> i32 {
    let mut h: u32 = 2166136261;
    for &b in p {
        h = (h ^ b as u32).wrapping_mul(16777619);
    }
    h as i32
}

fn put_ui_state(o: &mut Vec<i32>, s: &UiState) {
    o.extend(s.uuid.iter().map(|&v| v as i32));
    for p in s.groups.iter() {
        o.extend_from_slice(&[
            p.on_off as i32,
            p.allow_on_off as i32,
            p.gain as i32,
            p.default_gain as i32,
            p.allow_gain_interactivity as i32,
            p.az_offset as i32,
            p.el_offset as i32,
            p.dist_factor as i32,
            p.default_az_offset as i32,
            p.default_el_offset as i32,
            p.default_dist_factor as i32,
            p.allow_position_interactivity as i32,
            p.is_available as i32,
        ]);
    }
    for p in s.switch_groups.iter() {
        o.extend_from_slice(&[
            p.on_off as i32,
            p.active_member_index as i32,
            p.allow_switch as i32,
            p.is_available as i32,
        ]);
    }
    o.extend(s.group_presets.iter().map(|p| p.is_available as i32));
    o.push(s.active_preset_index as i32);
    for l in s.pref_audio_languages.iter() {
        o.extend(l.iter().map(|&v| v as i32));
    }
    o.push(s.accessibility_preference as i32);
    o.push(s.target_loudness as i32);
    o.push(s.drc_selected as i32);
    o.push(s.album_mode as i32);
    o.push(s.boost as i32);
    o.push(s.compress as i32);
}

fn put_persistence(o: &mut Vec<i32>, p: &UiPersistenceManager) {
    o.push(p.memory.is_some() as i32);
    o.push(p.mem_size as i32);
    o.push(p.max_keys as i32);
    o.push(p.max_commands as i32);
    o.push(if p.memory.is_some() { p.commands_offset as i32 } else { -1 });
    o.push(p.first_key_idx as i32);
    o.push(p.next_get_cmd_idx as i32);
    o.push(p.next_get_cmd_flags as i32);
    o.push(p.accessibility_setting as i32);
    o.extend(p.preferred_language.iter().map(|&v| v as i32));
    o.push(match &p.memory {
        Some(m) => fnv(&m[..p.mem_size as usize]),
        None => fnv(&[]),
    });
}

fn put_ui_manager(o: &mut Vec<i32>, h: &UiManager) {
    put_asi(o, &h.asi);
    put_ui_state(o, &h.ui_state);
    o.push(h.xml_state_changed as i32);
    o.push(h.ui_state_changed as i32);
    o.push(h.drc_state_changed as i32);
    o.push(h.config_changed as i32);
    o.push(h.is_active as i32);
    o.push(h.xml_writer.n_left as i32);
    o.push(h.xml_writer.next_preset_idx as i32);
    o.push(h.xml_writer.next_group_idx as i32);
    o.push(h.xml_writer.next_switch_group_member_idx as i32);
    put_persistence(o, &h.h_persistence);
}

fn mpegh_state(m: &MpeghUiManager) -> Vec<i32> {
    let mut o = vec![m.is_active as i32, m.config_found as i32];
    o.extend(m.substream_label.iter().map(|&v| v as i32));
    o.push(m.insert_offset as i32);
    put_ui_manager(&mut o, &m.h_ui_manager);
    o
}

fn put_action(o: &mut Vec<i32>, a: &UiManagerAction) {
    o.extend(a.uuid.iter().map(|&v| v as i32));
    o.push(a.action_type as i32);
    o.push(a.param_int);
    o.push(a.param_float);
    o.push(a.param_bool as i32);
    o.extend(a.param_text.iter().map(|&v| v as i32));
    o.push(a.present_flags as i32);
}

fn get_action(v: &[i32]) -> UiManagerAction {
    let mut a = UiManagerAction::default();
    for i in 0..16 {
        a.uuid[i] = v[i] as u8;
    }
    a.action_type = v[16] as u8;
    a.param_int = v[17];
    a.param_float = v[18];
    a.param_bool = v[19] as u8;
    for i in 0..41 {
        a.param_text[i] = v[20 + i] as i8;
    }
    a.present_flags = v[61] as u8;
    a
}

fn first_diff(a: &[i32], b: &[i32]) -> String {
    if a.len() != b.len() {
        return format!("length {} vs {}", a.len(), b.len());
    }
    let i = a.iter().zip(b).position(|(x, y)| x != y).unwrap();
    format!("index {i}: rust {} vs c {}", a[i], b[i])
}

// ---- generators ----

fn pick<T: Copy>(g: &mut Rng, v: &[T]) -> T {
    v[(g.next_u32() as usize) % v.len()]
}

fn chance(g: &mut Rng, n: i32) -> bool {
    g.range(0, n - 1) == 0
}

const LANGS: [&[u8; 3]; 14] = [
    b"eng", b"deu", b"ger", b"kor", b"fre", b"fra", b"spa", b"ENG", b"x1!", b"zho", b"chi",
    b"ko\0", b"en\0", b"und",
];

fn put_lang(w: &mut BitWriter, g: &mut Rng) {
    let l: &[u8; 3] = pick(g, &LANGS);
    w.put_bytes(l);
}

fn put_description_text(w: &mut BitWriter, g: &mut Rng) {
    let n = if chance(g, 5) { g.range(50, 90) } else { g.range(1, 20) } as u32;
    w.put(n - 1, 8);
    for _ in 0..n {
        let c = match g.range(0, 12) {
            0 => pick(g, b"<>&\"'"),
            1 => 0xC0 | (g.next_u32() & 0x3f) as u8,
            2 => 0x80 | (g.next_u32() & 0x3f) as u8,
            3 => b' ',
            _ => b'A' + g.range(0, 25) as u8,
        };
        w.put(c as u32, 8);
    }
}

fn put_condition(w: &mut BitWriter, g: &mut Rng, ref_id: u32, ref_bits: u32) {
    w.put(ref_id, ref_bits);
    let on = !chance(g, 4) as u32;
    w.put(on, 1);
    if on != 0 {
        w.put(chance(g, 3) as u32, 1); // disableGainInteractivity
        let gf = chance(g, 2) as u32;
        w.put(gf, 1);
        if gf != 0 {
            w.put(g.next_u32() & 0xff, 8);
        }
        w.put(chance(g, 3) as u32, 1); // disablePositionInteractivity
        let pf = chance(g, 3) as u32;
        w.put(pf, 1);
        if pf != 0 {
            w.put(g.next_u32() & 0xff, 8);
            w.put(g.next_u32() & 0x3f, 6);
            w.put(g.next_u32() & 0xf, 4);
        }
    }
}

/// Generates a (mostly valid) main stream mae_AudioSceneInfo() with groups, switch groups,
/// presets, descriptions, content data, DRC UI info and preset extensions.
fn gen_rich_asi(g: &mut Rng) -> Vec<u8> {
    let mut w = BitWriter::default();
    w.put(1, 1); // isMainStream
    let idp = g.range(0, 1) as u32;
    w.put(idp, 1);
    if idp != 0 {
        w.put(g.next_u32() & 0xff, 8);
    }

    // groups
    let ng = if chance(g, 10) { g.range(0, 16) } else { g.range(1, 7) } as usize;
    let mut gids: Vec<u32> = Vec::new();
    while gids.len() < ng {
        let id = if chance(g, 4) { g.range(0, 127) } else { g.range(0, 15) } as u32;
        if !gids.contains(&id) || chance(g, 50) {
            gids.push(id);
        }
    }
    w.put(ng as u32, 7);
    let mut next_id = 0u32;
    let mut group_start = Vec::new();
    for &gid in &gids {
        w.put(gid, 7);
        w.put(!chance(g, 3) as u32, 1); // allowOnOff
        w.put(!chance(g, 3) as u32, 1); // defaultOnOff
        let ap = chance(g, 2) as u32;
        w.put(ap, 1);
        if ap != 0 {
            for n in [7, 7, 5, 5, 4, 4] {
                let v = if chance(g, 4) { 0 } else { g.next_u32() & ((1 << n) - 1) };
                w.put(v, n);
            }
        }
        let agi = chance(g, 2) as u32;
        w.put(agi, 1);
        if agi != 0 {
            w.put(if chance(g, 4) { 0 } else { g.range(0, 63) as u32 }, 6);
            w.put(g.range(0, 31) as u32, 5);
        }
        let nm = g.range(1, 3) as u32;
        w.put(nm - 1, 7);
        w.put(1, 1); // hasConjunctMembers
        group_start.push(next_id);
        w.put(next_id.min(127), 7);
        next_id += nm;
    }

    // switch groups
    let nsg = if ng >= 2 { g.range(0, 3) } else { 0 } as usize;
    let mut free: Vec<u32> = gids.clone();
    let mut sgids: Vec<u32> = Vec::new();
    w.put(nsg as u32, 5);
    for _ in 0..nsg {
        let mut id = g.range(0, 31) as u32;
        while sgids.contains(&id) && !chance(g, 30) {
            id = g.range(0, 31) as u32;
        }
        sgids.push(id);
        w.put(id, 5);
        let a = chance(g, 2) as u32;
        w.put(a, 1);
        if a != 0 {
            w.put(g.range(0, 1) as u32, 1);
        }
        let nm = (g.range(1, 4) as usize).min(free.len().max(1));
        w.put(nm as u32 - 1, 5);
        let mut members = Vec::new();
        for _ in 0..nm {
            let m = if !free.is_empty() && !chance(g, 30) {
                free.remove((g.next_u32() as usize) % free.len())
            } else if chance(g, 2) {
                pick(g, &gids)
            } else {
                g.range(0, 127) as u32
            };
            members.push(m);
            w.put(m, 7);
        }
        let d = if chance(g, 8) { g.range(0, 127) as u32 } else { pick(g, &members) };
        w.put(d, 7);
    }

    // presets
    let ngp = if chance(g, 6) { 0 } else { g.range(1, 5) } as usize;
    let mut gpids: Vec<u32> = Vec::new();
    let mut sw_flags: Vec<Vec<u32>> = Vec::new();
    w.put(ngp as u32, 5);
    for _ in 0..ngp {
        let mut id = g.range(0, 31) as u32;
        while gpids.contains(&id) && !chance(g, 30) {
            id = g.range(0, 31) as u32;
        }
        gpids.push(id);
        w.put(id, 5);
        w.put(pick(g, &[0u32, 0, 1, 5, 6, 7, 7, 3, 31]), 5); // kind
        let nc = g.range(1, 4) as u32;
        w.put(nc - 1, 4);
        let mut flags = Vec::new();
        for _ in 0..nc {
            let sw = !sgids.is_empty() && chance(g, 3);
            flags.push(sw as u32);
            let r = if sw {
                pick(g, &sgids)
            } else if !gids.is_empty() && !chance(g, 12) {
                pick(g, &gids)
            } else {
                g.range(0, 127) as u32
            };
            put_condition(&mut w, g, r, 7);
        }
        sw_flags.push(flags);
    }

    // mae_Data()
    let mut sets: Vec<(u32, BitWriter)> = Vec::new();
    for dt in [0u32, 1, 5] {
        let ids: &Vec<u32> = match dt {
            0 => &gids,
            1 => &sgids,
            _ => &gpids,
        };
        if ids.is_empty() || chance(g, 3) {
            continue;
        }
        let mut p = BitWriter::default();
        let nb = g.range(1, ids.len().min(4) as i32) as u32;
        p.put(nb - 1, 7);
        for _ in 0..nb {
            p.put(pick(g, ids), if dt == 0 { 7 } else { 5 });
            let nl = if chance(g, 3) { g.range(2, 8) } else { 1 } as u32;
            p.put(nl - 1, 4);
            for _ in 0..nl {
                put_lang(&mut p, g);
                put_description_text(&mut p, g);
            }
        }
        sets.push((dt, p));
    }
    if !gids.is_empty() && !chance(g, 4) {
        let mut p = BitWriter::default();
        let nb = g.range(1, gids.len().min(5) as i32) as u32;
        p.put(nb - 1, 7);
        for _ in 0..nb {
            p.put(pick(g, &gids), 7);
            p.put(pick(g, &[0u32, 1, 2, 2, 3, 5, 9, 15]), 4);
            let hl = chance(g, 2) as u32;
            p.put(hl, 1);
            if hl != 0 {
                put_lang(&mut p, g);
            }
        }
        sets.push((2, p));
    }
    if chance(g, 2) {
        let mut p = BitWriter::default();
        let v = if chance(g, 8) { 1 } else { 0 };
        p.put(v, 2);
        if v == 0 {
            let n = g.range(1, 4) as u32;
            p.put(n - 1, 3);
            for _ in 0..n {
                p.put(g.next_u32() & 0x3f, 6);
                p.put(g.next_u32() & 0xffff, 16);
            }
        }
        sets.push((6, p));
    }
    if ngp > 0 && (sw_flags.iter().flatten().any(|&f| f != 0) || chance(g, 3)) {
        let mut p = BitWriter::default();
        for flags in &sw_flags {
            let hs = (flags.iter().any(|&f| f != 0) || chance(g, 4)) as u32;
            p.put(hs, 1);
            if hs != 0 {
                for &f in flags {
                    p.put(f, 1);
                }
            }
            let hd = chance(g, 4) as u32;
            p.put(hd, 1);
            if hd != 0 {
                let n = g.range(1, 2) as u32;
                p.put(n, 5);
                for _ in 0..n {
                    p.put(g.range(0, 3) as u32, 7); // downmixId
                    let nc = g.range(1, 3) as u32;
                    p.put(nc - 1, 4);
                    for _ in 0..nc {
                        let sw = !sgids.is_empty() && chance(g, 3);
                        p.put(sw as u32, 1);
                        if sw {
                            let r = pick(g, &sgids);
                            put_condition(&mut p, g, r, 5);
                        } else {
                            let r = if gids.is_empty() { 0 } else { pick(g, &gids) };
                            put_condition(&mut p, g, r, 7);
                        }
                    }
                }
            }
        }
        sets.push((8, p));
    }
    if chance(g, 6) {
        let mut p = BitWriter::default();
        p.put(0, 1);
        p.put(0, 1);
        for _ in 0..ngp {
            p.put(0, 1);
        }
        sets.push((9, p));
    }
    // shuffle
    for i in (1..sets.len()).rev() {
        let j = (g.next_u32() as usize) % (i + 1);
        sets.swap(i, j);
    }
    w.put(sets.len() as u32, 4);
    for (dt, p) in &sets {
        w.put(*dt, 4);
        w.put(p.byte_len() as u32, 16);
        w.put_writer(p);
        for _ in p.nbits..p.byte_len() * 8 {
            w.put(0, 1);
        }
    }

    // metaDataElementIDmaxAvail
    let max_avail = if next_id == 0 {
        0
    } else if chance(g, 4) && ng > 1 {
        // only the first groups available
        let k = g.range(1, ng as i32 - 1) as usize;
        group_start[k].max(1) - 1
    } else if chance(g, 20) {
        g.range(0, 127) as u32
    } else {
        next_id - 1
    };
    w.put(max_avail.min(127), 7);
    w.bytes
}

/// Sub-stream mae_AudioSceneInfo()
fn gen_sub_asi(g: &mut Rng) -> Vec<u8> {
    let mut w = BitWriter::default();
    w.put(0, 1);
    w.put(g.range(0, 30) as u32, 7);
    w.put(g.range(0, 60) as u32, 7);
    w.bytes
}

fn gen_uuid_str(g: &mut Rng, uuid: &[u8; 16]) -> String {
    let mut s = String::new();
    let lower = chance(g, 4);
    for (i, b) in uuid.iter().enumerate() {
        if i == 4 || i == 6 || i == 8 || i == 10 {
            s.push('-');
        }
        if lower {
            s.push_str(&format!("{b:02x}"));
        } else {
            s.push_str(&format!("{b:02X}"));
        }
    }
    match g.range(0, 30) {
        0 => s.truncate(g.range(0, s.len() as i32) as usize),
        1 => s = s.replace('-', ""),
        2 => s.insert(g.range(0, s.len() as i32) as usize, 'x'),
        _ => {}
    }
    s
}

fn gen_float_str(g: &mut Rng) -> String {
    match g.range(0, 12) {
        0 => pick(
            g,
            &[
                "0",
                "1",
                "-1",
                "0.5",
                "-0.5",
                "1.0",
                "-63",
                "-10",
                "-128",
                "-64",
                "-9.5",
                "abc",
                "",
                "+3",
                "1.23456",
                "-180",
                "180",
                "90",
                "-90",
                "3.",
                ".5",
                "-",
                "100000",
                "-0.00001",
                "65536",
                "-40000.25",
                "2147483647",
                "1e3",
                "0.9999",
            ],
        )
        .to_string(),
        1..=4 => {
            let v = g.range(-200, 200);
            let f = g.range(0, 9999);
            let d = g.range(0, 5) as usize;
            if d == 0 {
                format!("{v}")
            } else {
                let fs = format!("{f:04}");
                format!("{v}.{}", &fs[..d.min(4)])
            }
        }
        5 => format!("{}", g.range(-70, -5)),
        6 => format!("{}.{}", g.range(-70, -5), g.range(0, 9)),
        7 => format!("{}.{}", g.range(0, 1), g.range(0, 99)),
        _ => format!("{}.{}", g.range(-20, 20), pick(g, &["0", "5", "25", "75", "1", "333"])),
    }
}

/// Generates an XML action event (valid and invalid) for the scene of `h`.
fn gen_action(g: &mut Rng, h: &UiManager) -> Vec<u8> {
    let asi: &AudioSceneInfo = &h.asi;
    let uuid =
        if !chance(g, 8) { h.ui_state.uuid } else { core::array::from_fn(|_| g.next_u32() as u8) };
    let version = if chance(g, 10) {
        pick(g, &["10.0", "9.0", "9", "8.99", "11.2", "11.10", "abc", "+11.1", "11.1999", "11.09"])
    } else {
        pick(g, &["11.0", "11.1", "11.1"])
    };
    let action_type: i32 = if chance(g, 25) {
        g.range(-5, 300)
    } else {
        pick(
            g,
            &[
                0, 10, 10, 11, 12, 20, 20, 21, 30, 30, 30, 31, 31, 40, 40, 40, 41, 41, 42, 42, 43,
                43, 44, 60, 60, 60, 61, 61, 62, 63, 64, 65, 70, 70, 71, 71, 90, 95, 95, 100,
            ],
        )
    };
    let gids: Vec<i32> =
        (0..asi.num_groups as usize).map(|i| asi.groups[i].group_id as i32).collect();
    let sgids: Vec<i32> = (0..asi.num_switch_groups as usize)
        .map(|i| asi.switch_groups[i].switch_group_id as i32)
        .collect();
    let gpids: Vec<i32> = (0..asi.num_group_presets as usize)
        .map(|i| asi.group_presets[i].group_preset_id as i32)
        .collect();
    let param_int: i32 = if chance(g, 10) {
        pick(g, &[0, -1, 255, 256, 1000, i32::MAX, -2, 9, 5])
    } else {
        match action_type {
            10 => pick(
                g,
                &[
                    0,
                    1,
                    2,
                    3,
                    4,
                    5,
                    6,
                    7,
                    15,
                    16,
                    0x0054_3261,
                    0x0000_0019,
                    0x1234_5678,
                    -1,
                    -2,
                    0x8765_4321u32 as i32,
                ],
            ),
            30 | 95 if !gpids.is_empty() => pick(g, &gpids),
            31 => g.range(-1, 5),
            40..=44 if !gids.is_empty() => pick(g, &gids),
            60..=65 if !sgids.is_empty() => pick(g, &sgids),
            70 | 71 => g.range(-1, 10),
            _ => g.range(0, 31),
        }
    };
    let param_float = if action_type == 60 && !chance(g, 4) {
        // switch group member ID
        let id = if !gids.is_empty() { pick(g, &gids) } else { g.range(0, 30) };
        format!("{id}")
    } else if matches!(action_type, 11 | 12) && !chance(g, 3) {
        // DRC boost/compress in 0..1
        format!("0.{}", g.range(0, 9999))
    } else {
        gen_float_str(g)
    };
    let param_bool = pick(g, &["true", "false", "1", "0", "tru", "yes", "2", "", "False", "-1"]);
    let param_text = if chance(g, 6) {
        let n = g.range(0, 45) as usize;
        (0..n).map(|_| (b'a' + g.range(0, 25) as u8) as char).collect::<String>()
    } else {
        let l: &[u8; 3] = pick(g, &LANGS);
        String::from_utf8_lossy(l).replace('\0', "")
    };

    let mut attrs: Vec<String> = vec![
        format!("uuid=\"{}\"", gen_uuid_str(g, &uuid)),
        format!("version=\"{version}\""),
        format!("actionType=\"{action_type}\""),
    ];
    let need_int = !matches!(action_type, 0 | 11 | 12 | 20 | 21 | 90 | 100);
    let need_float = matches!(action_type, 11 | 12 | 20 | 41..=44 | 60 | 62..=65);
    let need_bool = matches!(action_type, 21 | 40 | 61);
    let need_text = matches!(action_type, 70 | 71);
    if if need_int { !chance(g, 12) } else { chance(g, 4) } {
        attrs.push(format!("paramInt=\"{param_int}\""));
    }
    if if need_float { !chance(g, 12) } else { chance(g, 4) } {
        attrs.push(format!("paramFloat=\"{param_float}\""));
    }
    if if need_bool { !chance(g, 12) } else { chance(g, 4) } {
        attrs.push(format!("paramBool=\"{param_bool}\""));
    }
    if if need_text { !chance(g, 12) } else { chance(g, 4) } {
        attrs.push(format!("paramText=\"{param_text}\""));
    }
    if chance(g, 3) {
        for i in (1..attrs.len()).rev() {
            let j = (g.next_u32() as usize) % (i + 1);
            attrs.swap(i, j);
        }
    }
    if chance(g, 15) && !attrs.is_empty() {
        let i = (g.next_u32() as usize) % attrs.len();
        attrs.remove(i);
    }
    let mut s = String::new();
    if chance(g, 3) {
        s.push_str("<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n");
    }
    s.push_str("<ActionEvent");
    for a in &attrs {
        s.push(if chance(g, 10) { '\n' } else { ' ' });
        s.push_str(a);
    }
    s.push_str(pick(g, &[" />", "/>", "/>\n", " >", ""]));
    let mut v = s.into_bytes();
    if chance(g, 12) {
        v = mutate(g, &v);
    }
    v
}

fn gen_raw_action_text(g: &mut Rng) -> Vec<u8> {
    let parts: [&[u8]; 23] = [
        b"99999999999",
        b"-2147483648",
        b"123456.78901",
        b"<ActionEvent",
        b"/>",
        b" uuid=\"",
        b"\"",
        b"=",
        b" version=\"11.1\"",
        b" version=\"9.5\"",
        b" actionType=\"",
        b"30",
        b"100",
        b" paramInt=\"",
        b" paramFloat=\"",
        b"-12.75",
        b" paramBool=\"",
        b"true",
        b" paramText=\"",
        b"eng",
        b"12345678-9abc-DEF0-1234-56789abcdef0",
        b" ",
        b"\0",
    ];
    let mut v = Vec::new();
    for _ in 0..g.range(0, 30) {
        if chance(g, 6) {
            v.push(g.next_u32() as u8);
        } else {
            v.extend_from_slice(pick(g, &parts));
        }
    }
    v
}

// ---- MHAS streams ----

/// Generates access units with rich audio scene information and random extra packets.
fn gen_rich_stream(g: &mut Rng, n_aus: usize) -> Vec<Vec<u8>> {
    let mut aus = Vec::new();
    let main_label = pick(g, &[1u32, 1, 1, 2, 16]);
    let mut sub_labels: Vec<u32> = match g.range(0, 5) {
        0 => vec![17],
        1 => vec![17, 33],
        _ => vec![],
    };
    let mut asi_seed = g.next_u64();
    let rap_period = g.range(1, 6) as usize;
    let marker_uuid: Vec<u8> = (0..g.range(0, 18)).map(|_| g.next_u32() as u8).collect();
    let config: Vec<u8> = (0..g.range(4, 20)).map(|_| g.next_u32() as u8).collect();
    for au in 0..n_aus {
        let mut out = Vec::new();
        if au % rap_period == 0 {
            if chance(g, 2) {
                put_packet(&mut out, 6, 0, &[0xa5]);
            }
            match g.range(0, 15) {
                0 => asi_seed = g.next_u64(),
                1 if !sub_labels.is_empty() => {
                    // substream label change (same or different label group)
                    let i = (g.next_u32() as usize) % sub_labels.len();
                    sub_labels[i] = pick(g, &[17u32, 18, 33, 34, 49]);
                }
                _ => {}
            }
            put_packet(
                &mut out,
                1,
                if chance(g, 40) { g.range(0, 40) as u32 } else { main_label },
                &config,
            );
            for &l in &sub_labels {
                if !chance(g, 6) {
                    put_packet(&mut out, 1, l, &config);
                }
            }
            if !chance(g, 8) {
                let a = gen_rich_asi(&mut Rng::new(asi_seed));
                put_packet(&mut out, 3, main_label, &a);
            }
            for &l in &sub_labels {
                if chance(g, 2) {
                    put_packet(&mut out, 3, l, &gen_sub_asi(g));
                }
            }
            if chance(g, 2) {
                put_packet(&mut out, 14, main_label, &[0x00]);
            }
        }
        for _ in 0..g.range(0, 2) {
            let (t, l, p): (u32, u32, Vec<u8>) = match g.range(0, 12) {
                0 => {
                    let mut p = vec![0xE0, 0x00];
                    p.extend_from_slice(&marker_uuid);
                    if chance(g, 5) {
                        p[0] = 0x12;
                    }
                    (8, pick(g, &[main_label, 3, 17]), p)
                }
                1 if chance(g, 3) => (12, pick(g, &[main_label, 20]), vec![0; 3]),
                2 if chance(g, 3) => (13, pick(g, &[main_label, 20]), vec![0; 3]),
                3 => (0, main_label, vec![0; g.range(0, 5) as usize]),
                4 => (17, main_label, vec![0x80, 0x10]),
                5 => (9, 0, vec![1, 2]),
                6 => {
                    (g.range(0, 40) as u32, g.range(0, 20) as u32, vec![0; g.range(0, 4) as usize])
                }
                _ => (6, 0, vec![0xa5]),
            };
            put_packet(&mut out, t, l, &p);
        }
        let frame: Vec<u8> = (0..g.range(4, 60)).map(|_| g.next_u32() as u8).collect();
        put_packet(&mut out, 2, main_label, &frame);
        for &l in &sub_labels {
            if !chance(g, 6) {
                put_packet(&mut out, 2, l, &frame[..4]);
            }
        }
        if chance(g, 40) {
            out = mutate(g, &out);
        }
        aus.push(out);
    }
    aus
}

// ---- driver of the mpegh_UI_* API ----

struct Api {
    c: *mut (),
    r: Box<MpeghUiManager>,
    c_mem: Option<Vec<u16>>,
    name: String,
    stats: [u64; 8],
}

impl Drop for Api {
    fn drop(&mut self) {
        unsafe { cref_uim_close(self.c) };
    }
}

impl Api {
    fn new(name: &str) -> Self {
        let c = unsafe { cref_uim_open() };
        assert!(!c.is_null());
        let a = Api {
            c,
            r: MpeghUiManager::open(),
            c_mem: None,
            name: name.to_string(),
            stats: [0; 8],
        };
        a.check("open");
        a
    }

    fn check(&self, what: &str) {
        let mut out_c = vec![0i32; 200_000];
        let n = unsafe { cref_uim_state(self.c, out_c.as_mut_ptr()) } as usize;
        let out_r = mpegh_state(&self.r);
        if out_r.as_slice() != &out_c[..n] {
            panic!(
                "{}: state differs after {what}: {}",
                self.name,
                first_diff(&out_r, &out_c[..n])
            );
        }
    }

    fn feed(&mut self, au: &[u8]) -> i32 {
        let mut buf = au.to_vec();
        let ec = unsafe { cref_uim_feed(self.c, buf.as_mut_ptr(), buf.len() as u32) };
        let er = self.r.feed_mhas(au);
        assert_eq!(er, ec, "{}: feed", self.name);
        self.check("feed");
        er
    }

    fn update(&mut self, au: &[u8], extra: usize) -> i32 {
        let mut buf_c = au.to_vec();
        buf_c.resize(au.len() + extra, 0x5a);
        let mut buf_r = buf_c.clone();
        let mut len_c = au.len() as u32;
        let mut len_r = len_c;
        let ec =
            unsafe { cref_uim_update(self.c, buf_c.as_mut_ptr(), buf_c.len() as u32, &mut len_c) };
        let er = self.r.update_mhas(&mut buf_r, &mut len_r);
        assert_eq!(er, ec, "{}: update", self.name);
        assert_eq!(len_r, len_c, "{}: update length", self.name);
        assert_eq!(buf_r, buf_c, "{}: update output", self.name);
        if er == MPEGH_UI_OK && len_r as usize > au.len() {
            self.stats[2] += 1;
        }
        self.check("update");
        er
    }

    fn get_xml(&mut self, size: usize, flags_in: u32, fill: u8) -> (i32, Vec<u8>, u32) {
        let mut buf_c = vec![fill as i8; size.max(1)];
        let mut buf_r = vec![fill; size];
        let mut fl_c = 0xdead;
        let mut fl_r = 0xbeef;
        let ec = unsafe {
            cref_uim_getXml(self.c, buf_c.as_mut_ptr(), size as u32, flags_in, &mut fl_c)
        };
        let er = self.r.get_xml_scene_state(&mut buf_r, flags_in, &mut fl_r);
        assert_eq!(er, ec, "{}: getXml size {size}", self.name);
        assert_eq!(fl_r, fl_c, "{}: getXml flags size {size}", self.name);
        let bc: Vec<u8> = buf_c[..size].iter().map(|&v| v as u8).collect();
        if buf_r != bc {
            let i = buf_r.iter().zip(&bc).position(|(a, b)| a != b).unwrap();
            panic!(
                "{}: xml output differs at {i} (size {size}):\nrust: {}\nc:    {}",
                self.name,
                String::from_utf8_lossy(&buf_r[i.saturating_sub(200)..(i + 50).min(size)]),
                String::from_utf8_lossy(&bc[i.saturating_sub(200)..(i + 50).min(size)])
            );
        }
        if er == 0 && fl_r & MPEGH_UI_NO_CHANGE == 0 {
            self.stats[3] += 1;
        }
        self.check("getXml");
        (er, buf_r, fl_r)
    }

    fn apply(&mut self, xml: &[u8]) -> i32 {
        let mut fl_c = 0xdead;
        let mut fl_r = 0xbeef;
        let ec = unsafe {
            cref_uim_applyXml(self.c, xml.as_ptr() as *const i8, xml.len() as u32, &mut fl_c)
        };
        let er = self.r.apply_xml_action(xml, &mut fl_r);
        assert_eq!(er, ec, "{}: apply {}", self.name, String::from_utf8_lossy(xml));
        assert_eq!(fl_r, fl_c);
        self.stats[if er == 0 { 0 } else { 1 }] += 1;
        self.check(&format!("apply {}", String::from_utf8_lossy(xml)));
        er
    }

    /// `mem`: block content (sizes 2..=9 are undefined behaviour in C++ and are not used).
    fn set_persist(&mut self, mem: Option<Vec<u8>>) -> i32 {
        let (ec, new_c) = match &mem {
            None => (unsafe { cref_uim_setPersist(self.c, core::ptr::null_mut(), 0) }, None),
            Some(m) => {
                let mut v = vec![0u16; m.len().div_ceil(2).max(1)];
                unsafe {
                    core::ptr::copy_nonoverlapping(m.as_ptr(), v.as_mut_ptr() as *mut u8, m.len());
                }
                let e = unsafe {
                    cref_uim_setPersist(self.c, v.as_mut_ptr() as *mut u8, m.len() as u16)
                };
                (e, Some(v))
            }
        };
        // the old block may be finalized by the call above; replace it only now
        self.c_mem = new_c;
        let er = self.r.set_persistence_memory(mem);
        assert_eq!(er, ec, "{}: setPersist", self.name);
        self.check("setPersist");
        er
    }

    fn get_persist(&mut self) -> Option<Vec<u8>> {
        let mut p: *mut u8 = core::ptr::null_mut();
        let mut size_c = 0u16;
        let ec = unsafe { cref_uim_getPersist(self.c, &mut p, &mut size_c) };
        let (er, mem, size_r) = self.r.get_persistence_memory();
        assert_eq!(er, ec);
        assert_eq!(size_r, size_c, "{}: getPersist size", self.name);
        assert_eq!(mem.is_some(), !p.is_null(), "{}: getPersist block", self.name);
        let res = mem.map(|m| {
            let c = unsafe { core::slice::from_raw_parts(p, size_c as usize) };
            assert_eq!(&m[..size_r as usize], c, "{}: persistence memory", self.name);
            m.to_vec()
        });
        self.check("getPersist");
        res
    }
}

fn random_persist_size(g: &mut Rng) -> usize {
    match g.range(0, 10) {
        0 => pick(g, &[0usize, 1, 10, 11, 400, 439, 440, 441, 442, 443, 480]),
        1 => g.range(10, 600) as usize,
        2 => 65535,
        _ => pick(g, &[512usize, 1024, 2048, 700]),
    }
}

/// Random operations between two access units.
fn random_ops(a: &mut Api, g: &mut Rng, saved: &mut Vec<Vec<u8>>, n: i32) {
    for _ in 0..n {
        match g.range(0, 20) {
            0..=8 => {
                let x = gen_action(g, &a.r.h_ui_manager);
                a.apply(&x);
            }
            9 => {
                let x = gen_raw_action_text(g);
                a.apply(&x);
            }
            10..=13 => {
                let size = match g.range(0, 6) {
                    0 => g.range(0, 200) as usize,
                    1 => g.range(200, 3000) as usize,
                    _ => 65536,
                };
                let flags = pick(g, &[0u32, 0, 1, 4, 5, 2, 0xff]);
                let fill = pick(g, &[0u8, 0xcc, b'x']);
                a.get_xml(size, flags, fill);
            }
            14 => {
                // complete XML in chunks
                let size = g.range(1, 400) as usize;
                let mut flags = pick(g, &[1u32, 5]);
                for _ in 0..200 {
                    let (e, _, fl) = a.get_xml(size, flags, 0xcc);
                    if e != 0 || fl & MPEGH_UI_INCOMPLETE_XML == 0 {
                        break;
                    }
                    flags = 0;
                }
            }
            15 => {
                if let Some(m) = a.get_persist() {
                    saved.push(m);
                }
            }
            16 => {
                // load a saved block (possibly corrupted) or a new one
                let mem = if !saved.is_empty() && chance(g, 2) {
                    let mut m = saved[(g.next_u32() as usize) % saved.len()].clone();
                    if chance(g, 4) && !m.is_empty() {
                        let i = (g.next_u32() as usize) % m.len();
                        m[i] ^= 1 << g.range(0, 7);
                    }
                    Some(m)
                } else if chance(g, 8) {
                    None
                } else {
                    let n = random_persist_size(g);
                    let fill = if chance(g, 2) { 0 } else { g.next_u32() as u8 };
                    Some(vec![fill; n])
                };
                a.set_persist(mem);
            }
            _ => {
                let x = gen_action(g, &a.r.h_ui_manager);
                a.apply(&x);
                a.get_xml(65536, 0, 0);
            }
        }
    }
}

fn run_stream(aus: &[Vec<u8>], seed: u64, name: &str, ops: i32) -> [u64; 8] {
    let mut g = Rng::new(seed);
    let mut a = Api::new(name);
    let mut saved = Vec::new();
    if !chance(&mut g, 3) {
        let n = pick(&mut g, &[512usize, 1024, 4096]);
        a.set_persist(Some(vec![0; n]));
    }
    for au in aus {
        if chance(&mut g, 4) {
            let k = g.range(1, ops);
            random_ops(&mut a, &mut g, &mut saved, k);
        }
        let e = a.feed(au);
        a.stats[4 + (e != 0) as usize] += 1;
        if chance(&mut g, 3) {
            let k = g.range(1, ops);
            random_ops(&mut a, &mut g, &mut saved, k);
        }
        if !chance(&mut g, 6) {
            let extra = if chance(&mut g, 10) { g.range(0, 20) } else { 256 } as usize;
            a.update(au, extra);
            if chance(&mut g, 20) {
                a.update(au, 600);
            }
        }
        if chance(&mut g, 2) {
            a.get_xml(65536, 0, 0);
        }
    }
    a.stats
}

fn add(s: &mut [u64; 8], t: [u64; 8]) {
    for i in 0..8 {
        s[i] += t[i];
    }
}

#[test]
fn api_test_streams() {
    let files = mhas_files();
    if files.is_empty() {
        eprintln!("test streams not found, skipping");
        return;
    }
    let limit = if full() { 3_000_000 } else { 400_000 };
    let mut g = Rng::new(0x51);
    let mut stats = [0u64; 8];
    for (k, f) in files.iter().enumerate() {
        if !full() && k % 3 != 0 {
            continue;
        }
        let data = tpdec_common::read_prefix(f, limit);
        let aus: Vec<Vec<u8>> =
            au_list(f, data.len()).iter().map(|&(o, n)| data[o..o + n].to_vec()).collect();
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        add(&mut stats, run_stream(&aus, g.next_u64(), &name, 4));
        // corrupted variant
        let mut aus2 = aus.clone();
        for au in aus2.iter_mut().take(300) {
            if chance(&mut g, 10) {
                *au = mutate(&mut g, au);
            }
        }
        aus2.truncate(300);
        add(&mut stats, run_stream(&aus2, g.next_u64(), &format!("{name} (mutated)"), 3));
    }
    println!("actions ok/err, updates, xml, feed ok/err: {stats:?}");
    assert!(stats[0] > 100 && stats[2] > 100 && stats[3] > 100, "poor coverage: {stats:?}");
}

#[test]
fn api_generated_streams() {
    let mut g = Rng::new(0x5eed_0011);
    let mut stats = [0u64; 8];
    let n = if full() { 5000 } else { 800 };
    for it in 0..n {
        let k = g.range(3, 30) as usize;
        let aus = gen_rich_stream(&mut g, k);
        add(&mut stats, run_stream(&aus, g.next_u64(), &format!("generated {it}"), 6));
    }
    println!("actions ok/err, updates, xml, feed ok/err: {stats:?}");
    assert!(
        stats[0] > 1000 && stats[1] > 1000 && stats[2] > 500 && stats[3] > 500,
        "poor coverage: {stats:?}"
    );
}

#[test]
fn api_tpdec_generated_streams() {
    // streams of the transport decoder tests (random configs and ASIs, many packet types)
    let mut g = Rng::new(0x7707);
    let mut stats = [0u64; 8];
    let n = if full() { 1000 } else { 200 };
    for it in 0..n {
        let k = g.range(3, 20) as usize;
        let (data, aus) = gen_mhas(&mut g, k, None);
        let aus: Vec<Vec<u8>> = aus.iter().map(|&(o, n)| data[o..o + n].to_vec()).collect();
        add(&mut stats, run_stream(&aus, g.next_u64(), &format!("tpdec generated {it}"), 4));
    }
    println!("actions ok/err, updates, xml, feed ok/err: {stats:?}");
}

#[test]
fn api_random_bytes() {
    let mut g = Rng::new(99);
    let mut a = Api::new("random bytes");
    let mut saved = Vec::new();
    for _ in 0..3000 {
        let n = g.range(0, 100) as usize;
        let mut d: Vec<u8> = (0..n).map(|_| g.next_u32() as u8).collect();
        if chance(&mut g, 3) {
            // valid packet structure with random types
            d.clear();
            for _ in 0..g.range(1, 5) {
                let p: Vec<u8> = (0..g.range(0, 10)).map(|_| g.next_u32() as u8).collect();
                put_packet(&mut d, g.range(0, 20) as u32, g.range(0, 40) as u32, &p);
            }
        }
        a.feed(&d);
        if chance(&mut g, 2) {
            a.update(&d, 300);
        }
        if chance(&mut g, 4) {
            random_ops(&mut a, &mut g, &mut saved, 2);
        }
    }
    // oversized input
    let big = vec![0u8; 64 * 1024 + 1];
    a.feed(&big);
}

// ---- UI_Manager_* API (as used by the core decoder) ----

struct Mgr {
    c: *mut (),
    r: Box<UiManager>,
    c_mem: Option<Vec<u16>>,
    store: bool,
    status_c: Vec<i32>,
    status_r: UserInteractivityStatus,
    drc_c: Vec<i32>,
    drc_r: UiDrcLoudnessStatus,
}

impl Drop for Mgr {
    fn drop(&mut self) {
        unsafe { cref_uimgr_delete(self.c) };
    }
}

fn put_status(s: &UserInteractivityStatus) -> Vec<i32> {
    let mut o = vec![s.interaction_mode as i32, s.num_groups as i32, s.group_preset_id as i32];
    for d in s.group_data.iter() {
        o.extend_from_slice(&[
            d.group_id as i32,
            d.on_off as i32,
            d.route_to_wire as i32,
            d.route_to_wire_id as i32,
            d.change_position as i32,
            d.az_offset as i32,
            d.el_offset as i32,
            d.dist_fact as i32,
            d.change_gain as i32,
            d.gain as i32,
        ]);
    }
    o
}

impl Mgr {
    fn new(store: bool) -> Self {
        let c = unsafe { cref_uimgr_create(store as i32) };
        let m = Mgr {
            c,
            r: UiManager::create(store as i32),
            c_mem: None,
            store,
            status_c: vec![0; 3 + 28 * 10],
            status_r: UserInteractivityStatus::default(),
            drc_c: vec![0; 5],
            drc_r: UiDrcLoudnessStatus::default(),
        };
        m.check("create");
        m
    }

    fn check(&self, what: &str) {
        let mut out_c = vec![0i32; 200_000];
        let n = unsafe { cref_uimgr_state(self.c, out_c.as_mut_ptr()) } as usize;
        let mut out_r = Vec::new();
        put_ui_manager(&mut out_r, &self.r);
        if out_r.as_slice() != &out_c[..n] {
            panic!("state differs after {what}: {}", first_diff(&out_r, &out_c[..n]));
        }
    }

    fn parse_asi(&mut self, asi: &[u8], stream_index: i32) -> i32 {
        let size = (asi.len() + 8).next_power_of_two();
        let mut buf_c = vec![0u8; size];
        buf_c[..asi.len()].copy_from_slice(asi);
        let buf_r = buf_c.clone();
        let valid = (asi.len() * 8) as u32;
        let ec = unsafe {
            cref_uimgr_parseAsi(self.c, buf_c.as_mut_ptr(), size as u32, valid, stream_index)
        };
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&buf_r), size as u32, valid, FdkBsCfg::BsReader);
        let er = mae_audio_scene_info(self.r.get_asi_pointer(), &mut bs, 2 * 28, stream_index);
        assert_eq!(er, ec, "parse asi");
        self.check("parse asi");
        er
    }

    fn op(&mut self, g: &mut Rng) {
        match g.range(0, 16) {
            0 => {
                let v = g.range(0, 1);
                let ec = unsafe { cref_uimgr_setIsActive(self.c, v) };
                assert_eq!(self.r.set_is_active(v as u8), ec);
                self.check("setIsActive");
            }
            1 => {
                let ec = unsafe { cref_uimgr_getXmlChanged(self.c) };
                assert_eq!(self.r.get_xml_changed(), ec);
                if chance(g, 2) {
                    unsafe { cref_uimgr_resetXmlChanged(self.c) };
                    self.r.reset_xml_changed();
                }
                self.check("xmlChanged");
            }
            2 if self.store => {
                let size = if chance(g, 3) { g.range(0, 2000) as usize } else { 65536 };
                let flags = pick(g, &[0u32, 1, 4, 5]);
                let mut buf_c = vec![0x33i8; size.max(1)];
                let mut buf_r = vec![0x33u8; size];
                let (mut fc, mut fr) = (0, 0);
                let ec = unsafe {
                    cref_uimgr_getXml(self.c, buf_c.as_mut_ptr(), size as u32, flags, &mut fc)
                };
                let er = self.r.get_xml_scene_state(&mut buf_r, flags, &mut fr);
                assert_eq!(er, ec);
                assert_eq!(fr, fc);
                let bc: Vec<u8> = buf_c[..size].iter().map(|&v| v as u8).collect();
                assert!(buf_r == bc, "xml differs");
                self.check("getXml");
            }
            3..=6 => {
                let x = gen_action(g, &self.r);
                let (mut fc, mut fr) = (1, 2);
                let ec = unsafe {
                    cref_uimgr_applyXml(self.c, x.as_ptr() as *const i8, x.len() as u32, &mut fc)
                };
                let er = self.r.apply_xml_action(&x, &mut fr);
                assert_eq!(er, ec, "apply {}", String::from_utf8_lossy(&x));
                assert_eq!(fr, fc);
                self.check("apply");
            }
            7 => {
                let mut uuid: [u8; 16] = if chance(g, 2) {
                    self.r.ui_state.uuid
                } else {
                    core::array::from_fn(|_| g.next_u32() as u8)
                };
                let crc = g.range(0, 1);
                let mut uc = uuid;
                let ec = unsafe { cref_uimgr_setUUID(self.c, uc.as_mut_ptr(), crc) };
                assert_eq!(self.r.set_uuid(&uuid, crc as u8), ec);
                let ec = unsafe { cref_uimgr_getUUID(self.c, uc.as_mut_ptr()) };
                assert_eq!(self.r.get_uuid(&mut uuid), ec);
                assert_eq!(uuid, uc);
                self.check("uuid");
            }
            8 | 9 => {
                let use_sig = g.range(0, 1) != 0;
                let mut sig_c = 0u8;
                let mut sig_r = 0u8;
                let ec = unsafe {
                    cref_uimgr_getInteractivityStatus(
                        self.c,
                        self.status_c.as_mut_ptr(),
                        if use_sig { &mut sig_c } else { core::ptr::null_mut() },
                    )
                };
                let er = self.r.get_interactivity_status(
                    &mut self.status_r,
                    if use_sig { Some(&mut sig_r) } else { None },
                );
                assert_eq!(er, ec);
                assert_eq!(sig_r, sig_c);
                assert_eq!(put_status(&self.status_r), self.status_c);
                self.check("interactivity status");
            }
            10 => {
                let ec = unsafe { cref_uimgr_getDrcStatus(self.c, self.drc_c.as_mut_ptr()) };
                let er = self.r.get_drc_loudness_status(&mut self.drc_r);
                assert_eq!(er, ec);
                let d = &self.drc_r;
                assert_eq!(
                    vec![
                        d.target_loudness as i32,
                        d.drc_selected as i32,
                        d.album_mode as i32,
                        d.boost as i32,
                        d.compress as i32
                    ],
                    self.drc_c
                );
                self.check("drc status");
            }
            11 => {
                let (mut a, mut b) = (7u8, 7u8);
                let (mut c, mut d) = (7u8, 7u8);
                let ec = unsafe { cref_uimgr_getStatusChanged(self.c, &mut a, &mut b) };
                assert_eq!(self.r.get_status_changed(Some(&mut c), Some(&mut d)), ec);
                assert_eq!((c, d), (a, b));
                self.check("status changed");
            }
            12 => {
                let n = random_persist_size(g);
                let fill = if chance(g, 2) { 0 } else { 0xff };
                let mut v = vec![0u16; n.div_ceil(2).max(1)];
                unsafe { core::ptr::write_bytes(v.as_mut_ptr() as *mut u8, fill, n) };
                let ec =
                    unsafe { cref_uimgr_setPersist(self.c, v.as_mut_ptr() as *mut u8, n as u16) };
                self.c_mem = Some(v);
                assert_eq!(self.r.set_persistence_memory(Some(vec![fill; n])), ec);
                self.check("set persistence");
            }
            13 => {
                let mut p: *mut u8 = core::ptr::null_mut();
                let mut size_c = 0u16;
                let ec = unsafe { cref_uimgr_getPersist(self.c, &mut p, &mut size_c) };
                let (mem, size_r) = self.r.get_persistence_memory();
                assert_eq!(ec, 0);
                assert_eq!(size_r, size_c);
                assert_eq!(mem.is_some(), !p.is_null());
                if let Some(m) = mem {
                    let c = unsafe { core::slice::from_raw_parts(p, size_c as usize) };
                    assert_eq!(&m[..size_r as usize], c);
                }
                self.check("get persistence");
            }
            14 => {
                if chance(g, 2) {
                    unsafe { cref_uimgr_asiReset(self.c) };
                    mpeghdec_tpdec::asi_reset(self.r.get_asi_pointer());
                } else {
                    unsafe { cref_uimgr_asiResetAvailability(self.c) };
                    mpeghdec_tpdec::asi_reset_availability(self.r.get_asi_pointer());
                }
                self.check("asi reset");
            }
            _ => {
                let id = pick(g, &[0, 1, 2, 3, 255]);
                unsafe { cref_uimgr_setActiveDmxId(self.c, id) };
                self.r.get_asi_pointer().active_dmx_id = id as u8;
                self.check("active dmx id");
            }
        }
    }
}

#[test]
fn ui_manager_core_api() {
    let mut g = Rng::new(0xc0de);
    let n = if full() { 10000 } else { 1500 };
    let mut ok = 0;
    for _ in 0..n {
        let mut m = Mgr::new(!chance(&mut g, 5));
        let mut seed = g.next_u64();
        for _ in 0..g.range(1, 40) {
            if chance(&mut g, 3) {
                if chance(&mut g, 4) {
                    seed = g.next_u64();
                }
                let mut asi = gen_rich_asi(&mut Rng::new(seed));
                if chance(&mut g, 15) {
                    asi = mutate(&mut g, &asi);
                }
                if m.parse_asi(&asi, 0) == 0 {
                    ok += 1;
                }
                if chance(&mut g, 5) {
                    m.parse_asi(&gen_sub_asi(&mut g), 1);
                }
            }
            for _ in 0..g.range(1, 6) {
                m.op(&mut g);
            }
        }
    }
    assert!(ok > 1000, "poor coverage: {ok}");
}

// ---- XML action parser ----

#[test]
fn xml_action_parser() {
    let mut g = Rng::new(4242);
    let dummy = UiManager::create(1);
    let mut accepted = 0;
    let n = if full() { 400_000 } else { 100_000 };
    for _ in 0..n {
        let x = match g.range(0, 3) {
            0 => gen_raw_action_text(&mut g),
            _ => gen_action(&mut g, &dummy),
        };
        let mut out_c = vec![0i32; 62];
        let rc = unsafe {
            cref_uimgr_parseXmlAction(x.as_ptr() as *const i8, x.len() as u32, out_c.as_mut_ptr())
        };
        let mut a = UiManagerAction::default();
        let rr = ui_manager_parse_xml_action(&x, &mut a);
        assert_eq!(rr, rc, "{}", String::from_utf8_lossy(&x));
        let mut out_r = Vec::new();
        put_action(&mut out_r, &a);
        // paramText behind the terminating NUL is not written by the parser
        let t = 20 + a.param_text.iter().position(|&c| c == 0).unwrap_or(40) + 1;
        assert_eq!(&out_r[..t], &out_c[..t], "{}", String::from_utf8_lossy(&x));
        assert_eq!(out_r[61], out_c[61]);
        if rr != 0 {
            accepted += 1;
        }
    }
    assert!(accepted > n / 5, "poor coverage: {accepted}");
}

// ---- persistence manager ----

struct Pers {
    c: *mut (),
    r: Box<UiPersistenceManager>,
    c_mem: Option<Vec<u16>>,
}

impl Drop for Pers {
    fn drop(&mut self) {
        unsafe { cref_pers_delete(self.c) };
    }
}

impl Pers {
    fn check(&self, what: &str) {
        let mut out_c = vec![0i32; 64];
        let n = unsafe { cref_pers_state(self.c, out_c.as_mut_ptr()) } as usize;
        let mut out_r = Vec::new();
        put_persistence(&mut out_r, &self.r);
        assert_eq!(out_r.as_slice(), &out_c[..n], "persistence state after {what}");
    }

    fn set(&mut self, mem: Option<Vec<u8>>) -> i32 {
        let (ec, v) = match &mem {
            None => (unsafe { cref_pers_setMemory(self.c, core::ptr::null_mut(), 0) }, None),
            Some(m) => {
                let mut v = vec![0u16; m.len().div_ceil(2).max(1)];
                unsafe {
                    core::ptr::copy_nonoverlapping(m.as_ptr(), v.as_mut_ptr() as *mut u8, m.len())
                };
                (
                    unsafe {
                        cref_pers_setMemory(self.c, v.as_mut_ptr() as *mut u8, m.len() as u16)
                    },
                    Some(v),
                )
            }
        };
        self.c_mem = v;
        let er = self.r.set_memory(mem);
        assert_eq!(er, ec, "setMemory");
        self.check("setMemory");
        er
    }

    fn get(&mut self) -> Option<Vec<u8>> {
        let mut p: *mut u8 = core::ptr::null_mut();
        let mut size_c = 0u16;
        unsafe { cref_pers_getMemory(self.c, &mut p, &mut size_c) };
        let (mem, size_r) = self.r.get_memory();
        assert_eq!(size_r, size_c);
        assert_eq!(mem.is_some(), !p.is_null());
        let res = mem.map(|m| {
            let c = unsafe { core::slice::from_raw_parts(p, size_c as usize) };
            assert_eq!(&m[..size_r as usize], c, "persistence memory");
            m[..size_r as usize].to_vec()
        });
        self.check("getMemory");
        res
    }
}

#[test]
fn persistence_manager() {
    let mut g = Rng::new(0x9e75);
    let n = if full() { 10000 } else { 1500 };
    let mut restored = 0;
    let mut commands = 0;
    for _ in 0..n {
        let mut p = Pers {
            c: unsafe { cref_pers_create() },
            r: UiPersistenceManager::create(),
            c_mem: None,
        };
        p.check("create");
        let uuids: Vec<[u8; 16]> = (0..g.range(1, 20))
            .map(|_| core::array::from_fn(|_| (g.next_u32() % 3) as u8))
            .collect();
        let mut saved: Vec<Vec<u8>> = Vec::new();
        let size = pick(&mut g, &[440usize, 441, 512, 600, 1024, 4096]);
        p.set(Some(vec![0; size]));
        for _ in 0..g.range(10, 300) {
            match g.range(0, 12) {
                0..=6 => {
                    let mut v: Vec<i32> = pick(&mut g, &uuids).iter().map(|&b| b as i32).collect();
                    v.push(pick(
                        &mut g,
                        &[0, 30, 30, 31, 40, 41, 42, 43, 60, 61, 62, 63, 64, 70, 70, 10, 95, 255],
                    ));
                    v.push(if chance(&mut g, 4) { g.i32() } else { g.range(0, 12) });
                    v.push(if chance(&mut g, 3) { g.i32() } else { g.range(-40, 40) << 15 });
                    v.push(g.range(0, 2));
                    for k in 0..41 {
                        v.push(if k < 3 { pick(&mut g, b"engdeuk") as i32 } else { 0 });
                    }
                    v.push(g.next_u32() as u8 as i32);
                    let a = get_action(&v);
                    unsafe { cref_pers_save(p.c, v.as_ptr()) };
                    p.r.save_command(&a);
                    p.check("save");
                }
                7 | 8 => {
                    // restore sequence
                    let mut a = UiManagerAction::default();
                    let mut v = Vec::new();
                    put_action(&mut v, &a);
                    let uuid = pick(&mut g, &uuids);
                    let mut first = true;
                    for _ in 0..1000 {
                        let rc = unsafe {
                            cref_pers_get(
                                p.c,
                                if first { uuid.as_ptr() } else { core::ptr::null() },
                                v.as_mut_ptr(),
                            )
                        };
                        let rr = p.r.get_command(if first { Some(&uuid) } else { None }, &mut a);
                        assert_eq!(rr as i32, rc);
                        let mut w = Vec::new();
                        put_action(&mut w, &a);
                        assert_eq!(w, v);
                        p.check("get");
                        first = false;
                        if rr == 0 {
                            break;
                        }
                        commands += 1;
                    }
                }
                9 => {
                    if let Some(m) = p.get() {
                        saved.push(m);
                    }
                }
                10 => {
                    let mem = if !saved.is_empty() && !chance(&mut g, 4) {
                        let mut m = saved[(g.next_u32() as usize) % saved.len()].clone();
                        if chance(&mut g, 5) {
                            let i = (g.next_u32() as usize) % m.len();
                            m[i] ^= 0x10;
                        }
                        Some(m)
                    } else if chance(&mut g, 3) {
                        None
                    } else {
                        Some(vec![g.next_u32() as u8; random_persist_size(&mut g)])
                    };
                    if p.set(mem) == 1 {
                        restored += 1;
                    }
                }
                _ => {}
            }
        }
    }
    assert!(restored > 100 && commands > 1000, "poor coverage: {restored} {commands}");
}
