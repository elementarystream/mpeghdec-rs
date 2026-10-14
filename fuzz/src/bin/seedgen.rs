// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Seed corpora of the fuzz targets from the test streams (fuzz/README.md, `fuzz/seeds.sh`).
//!
//! ```text
//! seedgen OUT_DIR [--cases FILE] FILE.mp4|FILE.mhas...
//! ```
//!
//! For every stream (the first MPEG-H track of an MP4 file, or a raw MHAS stream) a few inputs
//! are written to `OUT_DIR/<target>/`: the first access units (up to about 24 KB) with one and two
//! threads at two target layouts, a program with seeks, timestamp gaps and parameter changes, and
//! the access units from a later random access point. `mhm1` tracks and MHAS streams seed
//! `decode_mhas` and `ui_decode` (with XML actions for the groups and presets of the scene, and
//! the UUID of the stream), `mha1` tracks seed `decode_mha1`; MHAS streams also seed
//! `decode_mha1` with their configuration out of band.
//!
//! `--cases FILE` adds the decoder settings of the conformance test cases
//! (docs/research.md#conformance-bitstreams, `work/conformance/cases.json`; `fuzz/seeds.sh`
//! writes FILE): one line per case, `NAME STREAM TL RL DSE PRESET` (`-` for an unused setting),
//! for which the stream is decoded at target layout TL with setParam(TargetReferenceLevel, RL)
//! and setParam(EffectType, DSE) (`decode_mhas`), and with the preset selection
//! (`actionType="95"`) through the UI manager (`ui_decode`).

#[path = "../../../crates/cli/src/mp4.rs"]
#[allow(dead_code)]
mod mp4;

use mpeghdec_fuzz::dec::{Program, Target, CICPS};
use mpeghdec_uimanager::mpegh_ui_manager::MpeghUiManager;
use std::path::Path;

const MAX_BYTES: usize = 24 * 1024;
const MAX_AUS: usize = 48;

/// The access units and the out-of-band configuration of a stream.
type Stream = (Vec<Vec<u8>>, Option<Vec<u8>>);

/// Reads a [`Stream`] from an MHAS or MP4 file.
fn read_stream(path: &str) -> Option<Stream> {
    if path.ends_with(".mhas") {
        let d = std::fs::read(path).ok()?;
        return Some((split_mhas(&d), None));
    }
    let f = mp4::Mp4File::open(path).ok()?;
    let t =
        f.tracks.iter().find(|t| matches!(t.codec, mp4::Codec::MpeghMhm | mp4::Codec::MpeghMha))?;
    let mut aus = Vec::new();
    let mut bytes = 0;
    /* the beginning and, for the later random access point, the following samples */
    for s in t.samples.iter().take(4000) {
        let d = f.read_sample(s).ok()?;
        bytes += d.len();
        aus.push(d);
        if bytes > 8 * MAX_BYTES {
            break;
        }
    }
    let conf = if t.codec == mp4::Codec::MpeghMha { t.mha_config.clone() } else { None };
    Some((aus, conf))
}

/// The MHAS packet header at the start of `d`: (type, length, header bytes).
fn packet_header(d: &[u8]) -> Option<(u32, usize, usize)> {
    let mut p = 0usize;
    let mut bits = |n: usize| -> Option<u32> {
        let mut v = 0u32;
        for _ in 0..n {
            let b = d.get(p / 8)?;
            v = (v << 1) | ((b >> (7 - p % 8)) & 1) as u32;
            p += 1;
        }
        Some(v)
    };
    let mut escaped = |a: usize, b: usize, c: usize| -> Option<u32> {
        let mut v = bits(a)?;
        if v == (1 << a) - 1 {
            let w = bits(b)?;
            v += w;
            if w == (1 << b) - 1 {
                v += bits(c)?;
            }
        }
        Some(v)
    };
    let ty = escaped(3, 8, 8)?;
    let _label = escaped(2, 8, 32)?;
    let len = escaped(11, 24, 24)?;
    Some((ty, len as usize, p.div_ceil(8)))
}

/// Splits an MHAS stream into access units (each ends with an MPEGH3DAFRAME packet).
fn split_mhas(d: &[u8]) -> Vec<Vec<u8>> {
    const PACTYP_MPEGH3DAFRAME: u32 = 2;
    let (mut aus, mut start, mut off) = (Vec::new(), 0, 0);
    while off < d.len() && aus.len() < 4000 {
        let Some((ty, len, hb)) = packet_header(&d[off..]) else { break };
        off = (off + hb + len).min(d.len());
        if ty == PACTYP_MPEGH3DAFRAME {
            aus.push(d[start..off].to_vec());
            start = off;
            if aus.iter().map(Vec::len).sum::<usize>() > 8 * MAX_BYTES {
                break;
            }
        }
    }
    aus
}

/// Whether the access unit holds an MPEGH3DACFG packet.
fn has_config(au: &[u8]) -> bool {
    const PACTYP_MPEGH3DACFG: u32 = 1;
    let mut off = 0;
    while off < au.len() {
        let Some((ty, len, hb)) = packet_header(&au[off..]) else { return false };
        if ty == PACTYP_MPEGH3DACFG {
            return true;
        }
        off += hb + len;
    }
    false
}

/// The payloads of the packets of type `ty` in `au`.
fn payloads(au: &[u8], ty: u32) -> Vec<Vec<u8>> {
    let mut v = Vec::new();
    let mut off = 0;
    while off < au.len() {
        let Some((t, len, hb)) = packet_header(&au[off..]) else { break };
        let end = (off + hb + len).min(au.len());
        if t == ty {
            v.push(au[(off + hb).min(end)..end].to_vec());
        }
        off = end;
    }
    v
}

/// An `mha1` version of an MHAS stream: the first MPEGH3DACFG payload as the configuration, the
/// MPEGH3DAFRAME payloads as the access units.
fn to_mha1(aus: &[Vec<u8>]) -> Option<(Vec<Vec<u8>>, Vec<u8>)> {
    let conf = aus.iter().find_map(|au| payloads(au, 1).into_iter().next())?;
    let raw = aus.iter().flat_map(|au| payloads(au, 2)).collect();
    Some((raw, conf))
}

/// The access units from `from` up to the size limits.
fn take(aus: &[Vec<u8>], from: usize) -> Vec<Vec<u8>> {
    let mut v = Vec::new();
    let mut n = 0;
    for au in aus.iter().skip(from).take(MAX_AUS) {
        if n + au.len() > MAX_BYTES && !v.is_empty() {
            break;
        }
        n += au.len();
        v.push(au.clone());
    }
    v
}

fn cicp_flag(cicp: i32) -> u8 {
    (CICPS.iter().position(|&c| c == cicp).unwrap() as u8) << 4
}

/// The UUID and the group and preset IDs of the scene, from the port's UI manager.
fn scene(aus: &[Vec<u8>]) -> ([u8; 16], Vec<u8>, Vec<u8>) {
    let mut ui = MpeghUiManager::open();
    for au in aus {
        ui.feed_mhas(au);
        let mut buf = au.clone();
        buf.resize(au.len() + 4096, 0);
        let mut len = au.len() as u32;
        ui.update_mhas(&mut buf, &mut len);
    }
    let h = &ui.h_ui_manager;
    let gids = (0..h.asi.num_groups as usize).map(|i| h.asi.groups[i].group_id).take(8).collect();
    let pids = (0..h.asi.num_group_presets as usize)
        .map(|i| h.asi.group_presets[i].group_preset_id)
        .take(8)
        .collect();
    (h.ui_state.uuid, gids, pids)
}

struct Out {
    dir: String,
    n: usize,
}

impl Out {
    fn write(&mut self, t: Target, name: &str, tag: &str, p: &Program, flags: u8, aux: u8) {
        let dir = Path::new(&self.dir).join(t.name());
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(format!("{name}.{tag}")), p.to_bytes(flags, aux)).unwrap();
        self.n += 1;
    }
}

fn seeds(out: &mut Out, path: &str) {
    let Some((aus, conf)) = read_stream(path) else {
        eprintln!("{path}: no MPEG-H stream");
        return;
    };
    if aus.is_empty() {
        return;
    }
    let name = Path::new(path).file_stem().unwrap().to_string_lossy().replace(['.', ' '], "_");
    let t = if conf.is_some() { Target::Mha1 } else { Target::Mhas };
    let first = take(&aus, 0);
    let base = Program {
        target: t,
        threads: 1,
        timescale_mode: false,
        flush_at_end: true,
        cicp: 2,
        timescale: 48000,
        frame: (1024, 48000),
        ops: Vec::new(),
        config: conf.clone(),
        uuid: [0; 16],
        aus: first.clone(),
    };
    /* flags: bits 0-1 threads, bit 2 processTimescale, bit 3 flushAndGet, bits 4-7 cicp */
    out.write(t, &name, "tl2", &base, 8 | cicp_flag(2), 0x00);
    out.write(t, &name, "tl6.j2.ts", &base, 8 | 4 | 2 | cicp_flag(6), 0x00);
    out.write(t, &name, "tl13.j4", &base, 8 | 3 | cicp_flag(13), 0x01);
    /* seeks, gaps, duplicate timestamps, parameter changes, flushAndGet in between */
    let mut p = base.clone();
    p.ops = vec![
        0x03, // process 4
        0x29, // gap of 10 frames
        0x01, // process 2
        0x42, 0x0E, // setParam(EffectType, 6)
        0x41, 0x50, // setParam(TargetReferenceLevel, 72)
        0x02, // process 3
        0x63, // flush, back 3
        0x02, // process 3
        0x28, // duplicate timestamp
        0x00, // process 1
        0x24, 0x24, // jitter
        0x01, // process 2
        0x80, // flushAndGet
        0x03, // process 4
    ];
    out.write(t, &name, "events", &p, 8 | cicp_flag(6), 0x00);
    let mut p1 = p.clone();
    p1.threads = 2;
    out.write(t, &name, "events.j2", &p1, 8 | 2 | cicp_flag(2), 0x00);
    /* a later random access point */
    if t == Target::Mhas {
        if let Some(k) = (8..aus.len()).find(|&k| has_config(&aus[k])) {
            let mut p = base.clone();
            p.aus = take(&aus, k);
            out.write(t, &name, "rap", &p, 8 | cicp_flag(2), 0x00);
        }
    }
    if t != Target::Mhas {
        return;
    }
    /* decode_mha1: the stream with the configuration out of band */
    if let Some((raw, c)) = to_mha1(&aus) {
        let mut p = base.clone();
        p.target = Target::Mha1;
        p.config = Some(c);
        p.aus = take(&raw, 0);
        out.write(Target::Mha1, &name, "mhas.tl2", &p, 8 | cicp_flag(2), 0x00);
        let mut p2 = p.clone();
        p2.ops = vec![0x03, 0x29, 0x01, 0x63, 0x02, 0xD5, 0x03, 0x80, 0x03];
        out.write(Target::Mha1, &name, "mhas.events", &p2, 8 | 2 | cicp_flag(6), 0x00);
    }
    /* ui_decode: the stream with XML actions for its scene */
    let (uuid, gids, pids) = scene(&first);
    let mut p = base.clone();
    p.target = Target::Ui;
    p.uuid = uuid;
    out.write(Target::Ui, &name, "plain", &p, 8 | cicp_flag(2), 0x00);
    let mut ops = vec![
        0x03, // process 4
        0xE9, 0x02, // getXmlSceneState(1024, 0)
        0xEE, 0x00, 0x00, // setPersistenceMemory(512 zero bytes)
    ];
    /* actionType 30 (preset), 40 (group on/off), 41 (gain), 60 (switch group) */
    for &pid in &pids {
        ops.extend_from_slice(&[0xE0, 6, pid, 0, 0]);
        ops.push(0x01);
    }
    for &gid in &gids {
        ops.extend_from_slice(&[0xE0, 8, gid, 0, 0x08]); // paramBool
        ops.push(0x00);
        ops.extend_from_slice(&[0xE0, 9, gid, 4, 0x04]); // paramFloat -12.75
        ops.push(0x00);
    }
    ops.extend_from_slice(&[0xE0, 1, 3, 0, 0]); // actionType 10 (target layout)
    ops.extend_from_slice(&[0x01, 0xF0, 0xED, 0x08, 0x03]);
    let mut p2 = p.clone();
    p2.ops = ops;
    out.write(Target::Ui, &name, "actions", &p2, 8 | cicp_flag(6), 0x00);
    p2.threads = 2;
    out.write(Target::Ui, &name, "actions.j2", &p2, 8 | 2 | cicp_flag(2), 0x00);
}

/// The seeds of one conformance test case (see the top of the file).
fn case_seeds(out: &mut Out, line: &str) {
    let f: Vec<&str> = line.split_whitespace().collect();
    let [name, path, tl, rl, dse, preset] = f[..] else { return };
    let num = |s: &str| s.parse::<i32>().ok();
    let Some(tl) = num(tl).filter(|t| CICPS.contains(t)) else {
        eprintln!("{name}: target layout {tl} not in CICPS");
        return;
    };
    let Some((aus, None)) = read_stream(path) else { return };
    let mut ops = Vec::new();
    if let Some(rl) = num(rl) {
        ops.extend_from_slice(&[0x41, (rl + 8) as u8]); // setParam(TargetReferenceLevel)
    }
    if let Some(dse) = num(dse) {
        ops.extend_from_slice(&[0x42, (dse + 8) as u8]); // setParam(EffectType)
    }
    let name = name.replace(['.', ' '], "_");
    let mut p = Program {
        target: Target::Mhas,
        threads: 1,
        timescale_mode: false,
        flush_at_end: true,
        cicp: tl,
        timescale: 48000,
        frame: (1024, 48000),
        ops: ops.clone(),
        config: None,
        uuid: [0; 16],
        aus: take(&aus, 0),
    };
    out.write(Target::Mhas, &name, "case", &p, 8 | cicp_flag(tl), 0x00);
    if let Some(preset) = num(preset) {
        p.target = Target::Ui;
        p.uuid = scene(&p.aus).0;
        /* actionType 95 (preset selection without the UUID check), applied after the first feed */
        p.ops.extend_from_slice(&[0xE0, 22, preset as u8, 0, 0]);
        out.write(Target::Ui, &name, "case", &p, 8 | cicp_flag(tl), 0x00);
    }
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        eprintln!("usage: seedgen OUT_DIR [--cases FILE] FILE.mp4|FILE.mhas...");
        std::process::exit(2);
    }
    let mut out = Out { dir: args.remove(0), n: 0 };
    if args.first().map(String::as_str) == Some("--cases") {
        let cases = std::fs::read_to_string(&args[1]).expect("read the cases");
        for line in cases.lines() {
            case_seeds(&mut out, line);
        }
        args.drain(..2);
    }
    for f in &args {
        seeds(&mut out, f);
    }
    eprintln!("{} seeds written to {}", out.n, out.dir);
}
