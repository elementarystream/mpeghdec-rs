// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Shared helpers of the libDRCdec differential tests: a bit writer, random payload generators
//! for mpegh3daLoudnessInfoSet(), mpegh3daUniDrcConfig() and uniDrcGain(), and serializers of the
//! Rust structures in the same order as the C serializers in `shim/drc.cpp`.
#![allow(dead_code)]

use mpeghdec_cref::testutil::Rng;
use mpeghdec_drc::drc_dec_gain_decoder::*;
use mpeghdec_drc::drc_dec_rom::*;
use mpeghdec_drc::drc_dec_selection_process::SelProcOutput;
use mpeghdec_drc::drc_dec_types::*;
use mpeghdec_drc::drc_gain_dec_multiband::MAX_NUM_SUBBANDS;
use mpeghdec_drc::DrcDecoder;

/* ------------------------------------------------------------------------------------------ */
/* bit writer                                                                                 */
/* ------------------------------------------------------------------------------------------ */

pub struct BitWriter {
    pub bytes: Vec<u8>,
    pub bits: usize,
}

impl BitWriter {
    pub fn new() -> Self {
        BitWriter { bytes: Vec::new(), bits: 0 }
    }

    pub fn put(&mut self, v: u32, n: u32) {
        for i in (0..n).rev() {
            let bit = (v >> i) & 1;
            if self.bits % 8 == 0 {
                self.bytes.push(0);
            }
            if bit != 0 {
                let last = self.bytes.len() - 1;
                self.bytes[last] |= 0x80 >> (self.bits % 8);
            }
            self.bits += 1;
        }
    }

    pub fn bit(&mut self, b: bool) {
        self.put(b as u32, 1);
    }
}

/// Codewords of a Huffman table in the format of `drcDec_rom.cpp`: `(value, code, length)`.
pub fn huffman_codes(h: &[[i8; 2]]) -> Vec<(i32, u32, u32)> {
    let mut out = Vec::new();
    fn walk(h: &[[i8; 2]], node: usize, code: u32, len: u32, out: &mut Vec<(i32, u32, u32)>) {
        for bit in 0..2 {
            let next = h[node][bit];
            let c = (code << 1) | bit as u32;
            if next >= 0 {
                walk(h, next as usize, c, len + 1, out);
            } else {
                out.push((next as i32 + 64, c, len + 1));
            }
        }
    }
    walk(h, 0, 0, 0, &mut out);
    out
}

fn put_huffman(w: &mut BitWriter, g: &mut Rng, h: &[[i8; 2]]) {
    let codes = huffman_codes(h);
    let (_, code, len) = codes[g.range(0, codes.len() as i32 - 1) as usize];
    w.put(code, len);
}

fn chance(g: &mut Rng, percent: i32) -> bool {
    g.range(0, 99) < percent
}

fn pick<T: Copy>(g: &mut Rng, v: &[T]) -> T {
    v[g.range(0, v.len() as i32 - 1) as usize]
}

/* ------------------------------------------------------------------------------------------ */
/* extension payloads                                                                         */
/* ------------------------------------------------------------------------------------------ */

fn write_extension(w: &mut BitWriter, g: &mut Rng, size_len_bits: u32) {
    let n = g.range(0, 2);
    for _ in 0..n {
        w.put(g.range(1, 15) as u32, 4); /* ext type */
        let bit_size_len = g.range(0, 2) as u32;
        w.put(bit_size_len, size_len_bits);
        let bit_size = g.range(0, (1 << (bit_size_len + 4)) - 1) as u32;
        w.put(bit_size, bit_size_len + 4);
        for _ in 0..bit_size + 1 {
            w.put(g.next_u32() & 1, 1);
        }
    }
    w.put(0, 4); /* UNIDRC..EXT_TERM */
}

/* ------------------------------------------------------------------------------------------ */
/* loudnessInfoSet                                                                            */
/* ------------------------------------------------------------------------------------------ */

fn write_loudness_info(w: &mut BitWriter, g: &mut Rng, nice: bool, set_ids: &[u8]) {
    let drc_set_id = if nice {
        if chance(g, 60) || set_ids.is_empty() {
            pick(g, &[0u8, 0, 0x3F])
        } else {
            pick(g, set_ids)
        }
    } else {
        g.range(0, 63) as u8
    };
    w.put(drc_set_id as u32, 6);
    let downmix_id = if nice { pick(g, &[0u32, 0, 0, 0x7F, 1]) } else { g.range(0, 127) as u32 };
    w.put(downmix_id, 7);
    let spl = chance(g, 40);
    w.bit(spl);
    if spl {
        w.put(if chance(g, 10) { 0 } else { g.range(1, 4095) as u32 }, 12);
    }
    let tpl = chance(g, 40);
    w.bit(tpl);
    if tpl {
        w.put(if chance(g, 10) { 0 } else { g.range(1, 4095) as u32 }, 12);
        w.put(g.range(0, 15) as u32, 4);
        w.put(g.range(0, 3) as u32, 2);
    }
    let count = if nice { g.range(0, 3) } else { g.range(0, 15) };
    w.put(count as u32, 4);
    for _ in 0..count {
        let md =
            if nice { pick(g, &[1u32, 1, 2, 2, 7, 6, 3, 9, 0, 8]) } else { g.range(0, 15) as u32 };
        w.put(md, 4);
        let bits = match md {
            0..=6 | 9 => 8,
            7 => 5,
            8 => 2,
            _ => 0, /* invalid: parser returns an error */
        };
        let v = if nice && (md == 1 || md == 2) { g.range(100, 180) as u32 } else { g.next_u32() };
        w.put(v & ((1u32 << bits) - 1), bits);
        if bits == 0 && !nice {
            w.put(g.next_u32(), 16);
        }
        w.put(if nice { g.range(0, 11) as u32 } else { g.range(0, 15) as u32 }, 4);
        w.put(g.range(0, 3) as u32, 2);
    }
}

/// mpegh3daLoudnessInfoSet()
pub fn write_loudness_info_set(
    w: &mut BitWriter,
    g: &mut Rng,
    nice: bool,
    set_ids: &[u8],
    sub: usize,
) {
    let count = if nice { g.range(0, 5) } else { g.range(0, 63) };
    w.put(count as u32, 6);
    for _ in 0..count {
        let t = if nice { pick(g, &[0u32, 0, 0, 1, 2, 3]) } else { g.range(0, 3) as u32 };
        w.put(t, 2);
        if t == 1 || t == 2 {
            w.put(if nice { g.range(0, 4) as u32 } else { g.range(0, 127) as u32 }, 7);
        } else if t == 3 {
            w.put(if nice { g.range(0, 4) as u32 } else { g.range(0, 31) as u32 }, 5);
        }
        write_loudness_info(w, g, nice, set_ids);
    }
    let album = chance(g, 30);
    w.bit(album);
    if album {
        let count = if nice { g.range(0, 3) } else { g.range(0, 63) };
        w.put(count as u32, 6);
        for _ in 0..count {
            write_loudness_info(w, g, nice, set_ids);
        }
    }
    let ext = if sub == 0 || !nice { chance(g, 15) } else { false };
    w.bit(ext);
    if ext {
        write_extension(w, g, 4);
    }
}

/* ------------------------------------------------------------------------------------------ */
/* uniDrcConfig                                                                               */
/* ------------------------------------------------------------------------------------------ */

/// Parameters of a generated mpegh3daUniDrcConfig().
pub struct ConfigParams {
    /// stream channel count of this substream
    pub channels: u32,
    /// generate a mostly well-formed payload
    pub nice: bool,
    /// allow multiband gain sets
    pub multiband: bool,
    /// downmix instructions known to the decoder (`downmixId`, `targetChannelCount`)
    pub downmix: Vec<(u8, u8)>,
    pub sub: usize,
}

fn write_gain_set(w: &mut BitWriter, g: &mut Rng, p: &ConfigParams) -> u32 {
    let profile = if p.nice { pick(g, &[0u32, 0, 0, 1, 2, 3]) } else { g.range(0, 3) as u32 };
    w.put(profile, 2);
    w.put(g.range(0, 1) as u32, 1); /* gainInterpolationType */
    w.put(g.range(0, 1) as u32, 1); /* fullFrame */
    w.put(g.range(0, 1) as u32, 1); /* timeAlignment */
    let tdm = chance(g, 30);
    w.bit(tdm);
    if tdm {
        w.put(
            if p.nice { pick(g, &[0u32, 1, 3, 7, 15, 31, 63]) } else { g.range(0, 2047) as u32 },
            11,
        );
    }
    if profile != 3 {
        let band_count = if p.nice {
            if p.multiband && chance(g, 40) {
                g.range(2, 4) as u32
            } else {
                1
            }
        } else {
            g.range(0, 6) as u32
        };
        w.put(band_count, 4);
        let mut band_type = 0;
        if band_count > 1 {
            band_type = g.range(0, 1) as u32;
            w.put(band_type, 1);
        }
        for _ in 0..band_count {
            w.put(if chance(g, 50) { 0 } else { g.range(1, 127) as u32 }, 7); /* drcCharacteristic */
        }
        let mut border: u32 = 0;
        for _ in 1..band_count {
            if band_type == 1 {
                border = if p.nice {
                    (border + g.range(1, 4) as u32).min(15)
                } else {
                    g.range(0, 15) as u32
                };
                w.put(border, 4);
            } else {
                border = if p.nice {
                    (border + g.range(1, 60) as u32).min(255)
                } else {
                    g.range(0, 1023) as u32
                };
                w.put(border, 10);
            }
        }
        band_count
    } else {
        1
    }
}

fn write_gain_modification(w: &mut BitWriter, g: &mut Rng) {
    let scaling = chance(g, 30);
    w.bit(scaling);
    if scaling {
        w.put(g.range(0, 15) as u32, 4);
        w.put(g.range(0, 15) as u32, 4);
    }
    let offset = chance(g, 30);
    w.bit(offset);
    if offset {
        w.put(g.range(0, 1) as u32, 1);
        w.put(g.range(0, 31) as u32, 5);
    }
}

/// Writes the channel gainSetIndex list with repeats; returns the gainSetIndex values per channel.
fn write_channel_indices(
    w: &mut BitWriter,
    g: &mut Rng,
    n: u32,
    n_gain_sets: u32,
    nice: bool,
    ducking: bool,
) -> Vec<i32> {
    let mut v = Vec::new();
    let mut c = 0;
    while c < n {
        let bs = if nice {
            if chance(g, 10) {
                0
            } else if chance(g, 5) {
                n_gain_sets + 1
            } else {
                g.range(1, n_gain_sets.max(1) as i32) as u32
            }
        } else {
            g.range(0, 63) as u32
        };
        w.put(bs, 6);
        if ducking {
            let present = chance(g, 50);
            w.bit(present);
            if present {
                w.put(g.range(0, 15) as u32, 4);
            }
        }
        v.push(bs as i32 - 1);
        c += 1;
        let remaining = n - c;
        let repeat = remaining > 0 && chance(g, if nice { 40 } else { 50 });
        w.bit(repeat);
        if repeat {
            let cnt = if nice {
                g.range(1, remaining.min(32) as i32) as u32
            } else {
                g.range(1, 32) as u32
            };
            w.put(cnt - 1, 5);
            for _ in 0..cnt {
                v.push(bs as i32 - 1);
            }
            c += cnt;
        }
    }
    v
}

fn unique_groups(idx: &[i32]) -> usize {
    let mut u: Vec<i32> = Vec::new();
    for &i in idx {
        if i >= 0 && !u.contains(&i) {
            u.push(i);
        }
    }
    u.len()
}

/// mpegh3daUniDrcConfig(). Returns the written drcSetIds.
pub fn write_uni_drc_config(w: &mut BitWriter, g: &mut Rng, p: &ConfigParams) -> Vec<u8> {
    let n_coef = if p.nice { pick(g, &[1u32, 1, 1, 1, 2, 0]) } else { g.range(0, 7) as u32 };
    let n_inst = if p.nice { g.range(0, 6) as u32 } else { g.range(0, 40) as u32 };
    w.put(n_coef, 3);
    w.put(n_inst, 6);
    w.put(p.channels, 7);

    let mut n_gain_sets_first = 0;
    for c in 0..n_coef {
        let location = if p.nice {
            if c == 0 {
                1
            } else {
                pick(g, &[1u32, 2])
            }
        } else {
            g.range(0, 15) as u32
        };
        w.put(location, 4);
        let fs_present = !p.nice && chance(g, 10);
        w.bit(fs_present);
        if fs_present {
            let r = g.range(0, 32767) as u32;
            w.put(pick(g, &[1023u32, 511, 767, r]), 15);
        }
        let n_sets = if p.nice { g.range(1, 5) as u32 } else { g.range(0, 63) as u32 };
        w.put(n_sets, 6);
        for _ in 0..n_sets {
            write_gain_set(w, g, p);
        }
        if c == 0 {
            n_gain_sets_first = n_sets;
        }
    }

    let mut set_ids = Vec::new();
    for i in 0..n_inst {
        let t = if p.nice { pick(g, &[0u32, 0, 0, 0, 2, 3]) } else { pick(g, &[0u32, 2, 3]) };
        if t == 0 {
            w.put(0, 1);
        } else {
            w.put(1, 1);
            w.put(t - 2, 1);
            if t == 2 {
                w.put(if p.nice { g.range(0, 4) as u32 } else { g.range(0, 127) as u32 }, 7);
            } else {
                w.put(if p.nice { g.range(0, 4) as u32 } else { g.range(0, 31) as u32 }, 5);
            }
        }
        let set_id = if p.nice {
            if chance(g, 5) {
                g.range(0, 63) as u32
            } else {
                i + 1
            }
        } else {
            g.range(0, 63) as u32
        };
        set_ids.push(set_id as u8);
        w.put(set_id, 6);
        w.put(
            if p.nice {
                if chance(g, 95) {
                    1
                } else {
                    2
                }
            } else {
                g.range(0, 15) as u32
            },
            4,
        );

        let dmx0: u32 = if p.nice {
            if chance(g, 85) {
                0
            } else if chance(g, 50) {
                0x7F
            } else if !p.downmix.is_empty() && chance(g, 80) {
                pick(g, &p.downmix).0 as u32
            } else {
                g.range(1, 5) as u32
            }
        } else {
            g.range(0, 127) as u32
        };
        w.put(dmx0, 7);
        let additional = chance(g, if p.nice { 10 } else { 30 });
        w.bit(additional);
        let mut dmx_count = 1;
        if additional {
            let n = g.range(0, 7) as u32;
            w.put(n, 3);
            for _ in 0..n {
                w.put(g.range(0, 127) as u32, 7);
            }
            dmx_count = 1 + n;
        }

        let effect: u32 = if p.nice {
            let r = g.range(0, 99);
            if r < 60 {
                1 << g.range(0, 5)
            } else if r < 70 {
                (1 << g.range(0, 5)) | (1 << g.range(0, 5))
            } else if r < 75 {
                0x100 /* clipping */
            } else if r < 80 {
                0x200 /* fade */
            } else if r < 85 {
                pick(g, &[0x400u32, 0x800])
            } else if r < 90 {
                0
            } else {
                g.next_u32() & 0xFFFF
            }
        } else {
            g.next_u32() & 0xFFFF
        };
        w.put(effect, 16);
        let ducking = effect & 0xC00 != 0;
        if !ducking {
            let lpt = chance(g, 30);
            w.bit(lpt);
            if lpt {
                w.put(g.range(0, 255) as u32, 8);
            }
        }
        let tl = chance(g, if p.nice { 15 } else { 30 });
        w.bit(tl);
        if tl {
            /* value = bs - 63 dB; nice ranges mostly include the default target loudness */
            w.put(if p.nice && chance(g, 70) { g.range(39, 63) } else { g.range(0, 63) } as u32, 6);
            let lower = chance(g, 50);
            w.bit(lower);
            if lower {
                w.put(
                    if p.nice && chance(g, 70) { g.range(0, 38) } else { g.range(0, 63) } as u32,
                    6,
                );
            }
        }
        let depends = chance(g, if p.nice { 10 } else { 30 });
        w.bit(depends);
        if depends {
            w.put(
                if p.nice {
                    g.range(1, n_inst.max(1) as i32) as u32
                } else {
                    g.range(0, 63) as u32
                },
                6,
            );
        } else {
            w.put(if chance(g, if p.nice { 5 } else { 30 }) { 1 } else { 0 }, 1);
            /* noIndependentUse */
        }

        if ducking {
            write_channel_indices(w, g, p.channels, n_gain_sets_first, p.nice, true);
        } else {
            /* channel count as derived by the parser */
            let mut derive = false;
            let n_ch = if dmx0 != 0 && dmx0 != 0x7F && dmx_count == 1 {
                if !p.downmix.is_empty() {
                    match p.downmix.iter().find(|d| d.0 as u32 == dmx0) {
                        Some(d) => d.1 as u32,
                        None => 0, /* parser fails */
                    }
                } else {
                    derive = true;
                    1
                }
            } else if dmx0 == 0x7F || dmx_count > 1 {
                1
            } else {
                p.channels
            };
            let idx = if derive {
                /* one index, the repeat count defines the channel count */
                let bs = g.range(0, n_gain_sets_first.max(1) as i32) as u32;
                w.put(bs, 6);
                let repeat = chance(g, 50);
                w.bit(repeat);
                let mut v = vec![bs as i32 - 1];
                if repeat {
                    let cnt = g.range(1, 7) as u32;
                    w.put(cnt - 1, 5);
                    for _ in 0..cnt {
                        v.push(bs as i32 - 1);
                    }
                }
                v
            } else {
                write_channel_indices(w, g, n_ch, n_gain_sets_first, p.nice, false)
            };
            let groups = if dmx0 == 0x7F || dmx_count > 1 {
                (idx[0] >= 0) as usize
            } else {
                unique_groups(&idx)
            };
            for _ in 0..groups {
                write_gain_modification(w, g);
            }
        }
    }

    let ext = if p.sub == 0 || !p.nice { chance(g, 15) } else { false };
    w.bit(ext);
    if ext {
        write_extension(w, g, 4);
    }
    let lis = chance(g, 40);
    w.bit(lis);
    if lis {
        write_loudness_info_set(w, g, p.nice, &set_ids, p.sub);
    }
    set_ids
}

/* ------------------------------------------------------------------------------------------ */
/* uniDrcGain                                                                                 */
/* ------------------------------------------------------------------------------------------ */

fn get_z(n_nodes_max: i32) -> u32 {
    let mut z = 1u32;
    while (1i64 << z) < 2 * n_nodes_max as i64 {
        z += 1;
    }
    z
}

fn write_gain_initial(w: &mut BitWriter, g: &mut Rng, profile: u32) {
    match profile {
        0 => {
            w.put(g.range(0, 1) as u32, 1);
            w.put(g.range(0, 255) as u32, 8);
        }
        1 => {
            let s = chance(g, 70);
            w.bit(s);
            if s {
                w.put(g.range(0, 1023) as u32, 10);
            }
        }
        2 => {
            let s = chance(g, 70);
            w.bit(s);
            if s {
                w.put(g.range(0, 255) as u32, 8);
            }
        }
        _ => {}
    }
}

fn write_time_delta(w: &mut BitWriter, d: u32, z: u32) {
    match d {
        1 => w.put(0, 2),
        2..=5 => {
            w.put(1, 2);
            w.put(d - 2, 2);
        }
        6..=13 => {
            w.put(2, 2);
            w.put(d - 6, 3);
        }
        _ => {
            w.put(3, 2);
            w.put((d - 14) & ((1u32 << z.min(31)) - 1), z);
        }
    }
}

/// uniDrcGain() for the gain sequences of substream `sub` of `coef`.
pub fn write_uni_drc_gain(
    w: &mut BitWriter,
    g: &mut Rng,
    coef: &DrcCoefficientsUniDrc,
    frame_size: i32,
    delta_tmin_default: i32,
    sub: usize,
    nice: bool,
) {
    let start: usize = coef.stream_gain_sequence_count[..sub].iter().map(|&v| v as usize).sum();
    let count = coef.stream_gain_sequence_count[sub] as usize;
    for seq in 0..count {
        if start + seq >= 48 {
            return;
        }
        let idx = coef.gain_set_index_for_gain_sequence[start + seq];
        if idx >= coef.gain_set_count || idx >= 48 {
            return;
        }
        let gs = &coef.gain_set[idx as usize];
        let profile = gs.gain_coding_profile as u32;
        if profile == 3 {
            continue;
        }
        let tdm = if gs.time_delta_min_present != 0 {
            gs.time_delta_min as i32
        } else {
            delta_tmin_default
        };
        let slots = if tdm > 0 { (frame_size / tdm).max(1) } else { 1 };
        let z = get_z(if tdm != 0 { frame_size / tdm } else { 0 });
        let mode = chance(g, 70);
        w.bit(mode);
        if !mode {
            write_gain_initial(w, g, profile);
            continue;
        }
        let n_nodes = if nice { g.range(1, 8) } else { g.range(1, 40) } as u32;
        for _ in 0..n_nodes - 1 {
            w.put(0, 1);
        }
        w.put(1, 1);
        if gs.gain_interpolation_type == 0 {
            for _ in 0..n_nodes {
                put_huffman(w, g, &SLOPE_STEEPNESS_HUFFMAN);
            }
        }
        let frame_end = if gs.full_frame == 0 {
            let f = chance(g, 50);
            w.bit(f);
            f
        } else {
            true
        };
        let n_deltas = if frame_end { n_nodes - 1 } else { n_nodes };
        let avg = (slots as u32 / n_nodes.max(1)).max(1);
        for _ in 0..n_deltas {
            let d = if nice && chance(g, 90) {
                g.range(1, (avg as i32).max(1)) as u32
            } else {
                g.range(1, ((1u32 << z.min(20)) + 13).min(4000) as i32) as u32
            };
            write_time_delta(w, d, z);
        }
        write_gain_initial(w, g, profile);
        let table: &[[i8; 2]] = if profile == 2 {
            &DELTA_GAIN_CODING_PROFILE_2_HUFFMAN
        } else {
            &DELTA_GAIN_CODING_PROFILE_0_1_HUFFMAN
        };
        for _ in 1..n_nodes {
            put_huffman(w, g, table);
        }
    }
    let ext = chance(g, 10);
    w.bit(ext);
    if ext {
        write_extension(w, g, 3);
    }
}

/* ------------------------------------------------------------------------------------------ */
/* serializers (same order as shim/drc.cpp)                                                   */
/* ------------------------------------------------------------------------------------------ */

pub fn ser_gain_set(s: &GainSet, o: &mut Vec<i32>) {
    o.push(s.gain_coding_profile as i32);
    o.push(s.gain_interpolation_type as i32);
    o.push(s.full_frame as i32);
    o.push(s.time_alignment as i32);
    o.push(s.time_delta_min_present as i32);
    o.push(s.time_delta_min as i32);
    o.push(s.band_count as i32);
    o.push(s.drc_band_type as i32);
    o.extend(s.gain_sequence_index.iter().map(|&v| v as i32));
    for c in &s.drc_characteristic {
        o.push(c.present as i32);
        o.push(c.is_cicp as i32);
        o.push(c.cicp_index as i32);
        o.push(c.custom_right as i32);
    }
    o.extend(s.band_border.iter().map(|b| b.start_sub_band_index as i32));
}

pub fn ser_coef(c: &DrcCoefficientsUniDrc, o: &mut Vec<i32>) {
    o.push(c.drc_location as i32);
    o.push(c.drc_frame_size_present as i32);
    o.push(c.drc_frame_size as i32);
    o.push(c.gain_sequence_count as i32);
    o.push(c.gain_set_count as i32);
    o.extend(c.stream_gain_sequence_count.iter().map(|&v| v as i32));
    o.extend(c.stream_gain_set_count.iter().map(|&v| v as i32));
    for s in &c.gain_set {
        ser_gain_set(s, o);
    }
    o.extend(c.gain_set_index_for_gain_sequence.iter().map(|&v| v as i32));
}

pub fn ser_inst(p: &DrcInstructionsUniDrc, o: &mut Vec<i32>) {
    o.push(p.drc_instructions_type as i32);
    o.push(p.mae_group_id as i32);
    o.push(p.drc_set_id as i32);
    o.push(p.drc_set_complexity_level as i32);
    o.push(p.drc_location as i32);
    o.push(p.drc_apply_to_downmix as i32);
    o.push(p.downmix_id_count as i32);
    o.extend(p.downmix_id.iter().map(|&v| v as i32));
    o.push(p.drc_set_effect as i32);
    o.push(p.limiter_peak_target_present as i32);
    o.push(p.limiter_peak_target as i32);
    o.push(p.drc_set_target_loudness_present as i32);
    o.push(p.drc_set_target_loudness_value_upper as i32);
    o.push(p.drc_set_target_loudness_value_lower as i32);
    o.push(p.depends_on_drc_set_present as i32);
    o.push(p.depends_on_drc_set as i32);
    o.push(p.requires_eq as i32);
    o.extend(p.gain_ducking_modification.iter().map(|&v| v as i32));
    o.extend(p.gain_set_index.iter().map(|&v| v as i32));
    o.push(p.drc_channel_count as i32);
    o.extend(p.stream_n_drc_channel_groups.iter().map(|&v| v as i32));
    o.push(p.n_drc_channel_groups as i32);
    o.extend(p.gain_set_index_for_channel_group.iter().map(|&v| v as i32));
}

pub fn ser_config(c: &UniDrcConfig, o: &mut Vec<i32>) {
    o.push(c.downmix_instructions_count_v0 as i32);
    o.push(c.downmix_instructions_count_v1 as i32);
    o.push(c.downmix_instructions_count as i32);
    o.extend(c.drc_coefficients_uni_drc_count_v0.iter().map(|&v| v as i32));
    o.push(c.drc_coefficients_uni_drc_count_v1 as i32);
    o.push(c.drc_coefficients_uni_drc_count as i32);
    o.extend(c.drc_instructions_uni_drc_count_v0.iter().map(|&v| v as i32));
    o.push(c.drc_instructions_uni_drc_count_v1 as i32);
    o.push(c.drc_instructions_uni_drc_count as i32);
    o.extend(c.stream_channel_count.iter().map(|&v| v as i32));
    o.push(c.channel_layout.base_channel_count as i32);
    o.push(c.channel_layout.layout_signaling_present as i32);
    for d in &c.downmix_instructions {
        o.push(d.downmix_id as i32);
        o.push(d.target_channel_count as i32);
        o.push(d.target_layout as i32);
        o.push(d.downmix_coefficients_present as i32);
    }
    for k in &c.drc_coefficients_uni_drc {
        ser_coef(k, o);
    }
    for p in &c.drc_instructions_uni_drc {
        ser_inst(p, o);
    }
    o.push(c.loudness_info_set_present as i32);
    o.push(c.uni_drc_config_ext_present as i32);
    o.extend(c.uni_drc_config_ext.uni_drc_config_ext_type.iter().map(|&v| v as i32));
    o.extend(c.uni_drc_config_ext.ext_bit_size.iter().map(|&v| v as i32));
    o.push(c.drc_instructions_count_incl_virtual as i32);
    o.push(c.diff as i32);
}

pub fn ser_loudness_info(l: &LoudnessInfo, o: &mut Vec<i32>) {
    o.push(l.loudness_info_type as i32);
    o.push(l.mae_group_id as i32);
    o.push(l.drc_set_id as i32);
    o.push(l.downmix_id as i32);
    o.push(l.sample_peak_level_present as i32);
    o.push(l.true_peak_level_present as i32);
    o.push(l.measurement_count as i32);
    o.push(l.sample_peak_level);
    o.push(l.true_peak_level);
    for m in &l.loudness_measurement {
        o.push(m.method_value);
        o.push(m.method_definition as i32);
        o.push(m.measurement_system as i32);
        o.push(m.reliability as i32);
    }
}

pub fn ser_loudness_set(s: &LoudnessInfoSet, o: &mut Vec<i32>) {
    o.extend(s.loudness_info_album_count_v0.iter().map(|&v| v as i32));
    o.push(s.loudness_info_album_count_v1 as i32);
    o.push(s.loudness_info_album_count as i32);
    o.extend(s.loudness_info_count_v0.iter().map(|&v| v as i32));
    o.push(s.loudness_info_count_v1 as i32);
    o.push(s.loudness_info_count as i32);
    for l in &s.loudness_info_album {
        ser_loudness_info(l, o);
    }
    for l in &s.loudness_info {
        ser_loudness_info(l, o);
    }
    o.push(s.loudness_info_set_ext_present as i32);
    o.extend(s.loudness_info_set_ext.loudness_info_set_ext_type.iter().map(|&v| v as i32));
    o.extend(s.loudness_info_set_ext.ext_bit_size.iter().map(|&v| v as i32));
    o.push(s.diff as i32);
}

pub fn ser_gain(s: &UniDrcGain, o: &mut Vec<i32>) {
    o.extend(s.n_nodes.iter().map(|&v| v as i32));
    for seq in &s.gain_node {
        for n in seq {
            o.push(n.gain_db as i32);
            o.push(n.time as i32);
        }
    }
    o.push(s.uni_drc_gain_ext_present as i32);
    o.extend(s.uni_drc_gain_extension.uni_drc_gain_ext_type.iter().map(|&v| v as i32));
    o.extend(s.uni_drc_gain_extension.ext_bit_size.iter().map(|&v| v as i32));
    o.extend(s.n_decoded_sequences.iter().map(|&v| v as i32));
    o.push(s.status as i32);
}

pub fn ser_sel_proc_output(s: &SelProcOutput, o: &mut Vec<i32>) {
    o.push(s.output_peak_level_db);
    o.push(s.loudness_normalization_gain_db);
    o.push(s.output_loudness);
    o.push(s.num_selected_drc_sets as i32);
    o.extend(s.selected_drc_set_ids.iter().map(|&v| v as i32));
    o.extend(s.selected_downmix_ids.iter().map(|&v| v as i32));
    o.push(s.active_downmix_id as i32);
    o.push(s.base_channel_count as i32);
    o.push(s.target_channel_count as i32);
    o.push(s.target_layout as i32);
    o.push(s.downmix_matrix_present as i32);
    o.push(s.boost as i32);
    o.push(s.compress as i32);
    o.push(s.group_id_loudness_count as i32);
    o.extend(s.group_id.iter().map(|&v| v as i32));
    o.extend(s.group_id_loudness.iter().copied());
    o.push(s.mixing_level);
}

fn ser_lnb(l: &LinearNodeBuffer, o: &mut Vec<i32>) {
    o.push(l.gain_interpolation_type);
    o.extend(l.n_nodes.iter().copied());
    for f in &l.linear_node_gain {
        o.extend(f.iter().copied());
    }
    for f in &l.linear_node_time {
        o.extend(f.iter().map(|&v| v as i32));
    }
}

pub fn ser_gain_dec(h: &DrcGainDecoder, o: &mut Vec<i32>, with_overlap: bool) {
    o.push(h.delta_tmin_default);
    o.push(h.frame_size);
    o.push(h.sample_rate);
    o.push(h.ms_per_frame);
    o.push(h.loudness_normalisation_gain_db);
    o.push(h.delay_mode);
    ser_coef(&h.drc_coef, o);
    o.extend(h.n_active_drcs.iter().copied());
    for loc in &h.active_drc {
        for a in loc {
            o.push(a.active_drc_offset);
            ser_inst(&a.drc_inst, o);
            o.extend(
                a.ducking_modification_for_channel_group.iter().map(|d| d.ducking_scaling as i32),
            );
            o.extend(a.channel_group_for_channel.iter().map(|&v| v as i32));
            o.extend(a.band_count_for_channel_group.iter().map(|&v| v as i32));
            o.extend(a.gain_element_for_group.iter().map(|&v| v as i32));
            o.extend(a.channel_group_is_parametric_drc.iter().map(|&v| v as i32));
            o.push(a.gain_element_count as i32);
            for c in &a.lnb_index_for_channel {
                o.extend(c.iter().copied());
            }
            o.push(a.subband_gains_ready);
        }
    }
    o.push(h.multi_band_active_drc_index);
    o.push(h.channel_gain_active_drc_index);
    o.extend(h.channel_gain.iter().copied());
    o.extend(h.channel_gain_prev.iter().copied());
    o.push(h.drc_gain_buffers.lnb_pointer);
    for l in &h.drc_gain_buffers.linear_node_buffer {
        ser_lnb(l, o); /* 48 buffers followed by dummyLnb */
    }
    if with_overlap {
        for g in &h.overlap {
            for b in g {
                o.push(b.start_index as i32);
                o.push(b.stop_index as i32);
                o.extend(b.overlap_weight[..MAX_NUM_SUBBANDS].iter().map(|&v| v as i32));
            }
        }
    }
    o.extend(h.subband_gains.iter().copied()); /* subbandGains followed by dummySubbandGains */
    o.push(h.status);
    o.push(h.time_domain_supported);
    o.push(h.startup_ms);
    o.push(h.subband_domain_supported);
}

/// Serialization of the decoder state, see `cref_drcdec_ser`.
pub fn ser_decoder(d: &DrcDecoder, what: i32) -> Vec<i32> {
    let mut o = vec![d.codec_mode, d.functional_range, d.status, d.sel_proc_input_diff];
    if what & 1 != 0 {
        ser_config(&d.uni_drc_config, &mut o);
        ser_loudness_set(&d.loudness_info_set, &mut o);
        ser_gain(&d.uni_drc_gain, &mut o);
    }
    if what & 2 != 0 {
        ser_sel_proc_output(&d.sel_proc_output, &mut o);
    }
    if what & 4 != 0 {
        if let Some(h) = d.h_gain_dec.as_deref() {
            ser_gain_dec(h, &mut o, what & 8 != 0);
        }
    }
    o
}

/// Panics with the position of the first difference.
pub fn assert_same(c: &[i32], r: &[i32], ctx: &str) {
    if c != r {
        let n = c.len().min(r.len());
        let i = (0..n).find(|&i| c[i] != r[i]).unwrap_or(n);
        let lo = i.saturating_sub(4);
        panic!(
            "{ctx}: mismatch at index {i} (len C {} / Rust {}): C {:?} Rust {:?}",
            c.len(),
            r.len(),
            &c[lo..(i + 5).min(c.len())],
            &r[lo..(i + 5).min(r.len())]
        );
    }
}
