// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libMpeghDec mct.cpp (multichannel coding tool): CMct_Initialize() with
//! random masks, CMct_inverseMctParseBS() on random and crafted bitstreams, and
//! CMct_MCT_StereoFilling() with CMct_StereoFilling_save_prev()/clear_prev() over sequences of
//! frames with persistent state. The complete MCT state, all channel structures (including the
//! IGF data), the spectra and the bitstream positions are compared after every call.
#![cfg(not(cref_missing))]

mod core_common;

use core_common::*;
use mpeghdec_core::aac_rom::SFB_OFFSET_TABLES;
use mpeghdec_core::channelinfo::*;
use mpeghdec_core::mct::*;
use mpeghdec_cref::core_common::*;
use mpeghdec_cref::core_mct::*;
use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::igf::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::{AC_EL_ENHANCED_NOISE, AC_EL_IGF_USE_ENF, AC_INDEP};
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::mdct::BLOCK_SHORT;
use mpeghdec_igf::igf_dec::*;

/// Nominal sampling rates of the entries of `sfbOffsetTables[0]`.
const SAMPLE_RATES: [u32; 12] =
    [96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000];

// ---------------------------------------------------------------------------------------------
// MCT state dump (mirrors cref_core_mct_dump() / cref_core_mct_load())
// ---------------------------------------------------------------------------------------------

fn push_u8(o: &mut Vec<i32>, a: &[u8]) {
    o.extend(a.iter().map(|&v| v as i32));
}

fn push_i16(o: &mut Vec<i32>, a: &[i16]) {
    o.extend(a.iter().map(|&v| v as i32));
}

fn dump_mct(m: &CMct) -> Vec<i32> {
    let w = &m.mct_work;
    let mut o = vec![w.keep_tree as i32, w.num_pairs as i32];
    for p in &w.code_pairs {
        push_u8(&mut o, p);
    }
    push_u8(&mut o, &w.b_delta_time);
    for r in &w.pair_coeff_delta_sfb {
        push_u8(&mut o, r);
    }
    push_u8(&mut o, &w.pair_coeff_delta_fb);
    push_u8(&mut o, &w.num_mct_mask_bands);
    for &mk in &w.mct_mask {
        o.push((mk >> 32) as u32 as i32);
        o.push(mk as u32 as i32);
    }
    push_u8(&mut o, &w.b_has_mct_mask);
    push_u8(&mut o, &w.b_has_bandwise_coeffs);
    push_u8(&mut o, &w.pred_dir);
    push_u8(&mut o, &w.has_stereo_filling);
    o.push(m.mcc_signaling_type as i32);
    o.push(m.mct_signaling_type_prev as i32);
    push_u8(&mut o, &m.window_sequence_is_short_prev);
    push_u8(&mut o, &m.channel_map);
    o.push(m.num_mct_channels as i32);
    for r in &m.pair_coeff_q_sfb_prev {
        push_i16(&mut o, r);
    }
    push_i16(&mut o, &m.pair_coeff_q_fb_prev);
    o.push(!m.prev_out_spec.is_empty() as i32);
    o.extend_from_slice(&m.prev_out_spec);
    o.push(!m.prev_out_spec_exp.is_empty() as i32);
    push_i16(&mut o, &m.prev_out_spec_exp);
    o.push(m.mct_group_parsed as i32);
    o
}

fn load_mct(m: &mut CMct, a: &[i32]) {
    let r = &mut Rd::new(a);
    let w = &mut m.mct_work;
    w.keep_tree = r.u8();
    w.num_pairs = r.u8();
    for p in &mut w.code_pairs {
        r.fill_u8(p);
    }
    r.fill_u8(&mut w.b_delta_time);
    for row in &mut w.pair_coeff_delta_sfb {
        r.fill_u8(row);
    }
    r.fill_u8(&mut w.pair_coeff_delta_fb);
    r.fill_u8(&mut w.num_mct_mask_bands);
    for mk in &mut w.mct_mask {
        let hi = r.u32() as u64;
        let lo = r.u32() as u64;
        *mk = (hi << 32) | lo;
    }
    r.fill_u8(&mut w.b_has_mct_mask);
    r.fill_u8(&mut w.b_has_bandwise_coeffs);
    r.fill_u8(&mut w.pred_dir);
    r.fill_u8(&mut w.has_stereo_filling);
    m.mcc_signaling_type = r.u8();
    m.mct_signaling_type_prev = r.u8();
    r.fill_u8(&mut m.window_sequence_is_short_prev);
    r.fill_u8(&mut m.channel_map);
    r.i32(); /* numMctChannels */
    for row in &mut m.pair_coeff_q_sfb_prev {
        r.fill_i16(row);
    }
    r.fill_i16(&mut m.pair_coeff_q_fb_prev);
    r.i32();
    r.fill_i32(&mut m.prev_out_spec);
    r.i32();
    r.fill_i16(&mut m.prev_out_spec_exp);
    m.mct_group_parsed = r.u8();
    assert_eq!(r.n, a.len());
}

/// A C and a Rust `CMct`.
struct MctPair {
    c: *mut (),
    r: Option<Box<CMct>>,
}

impl MctPair {
    fn new() -> Self {
        MctPair { c: std::ptr::null_mut(), r: None }
    }

    /// `CMct_Initialize()` in both implementations; returns the (equal) return value.
    fn initialize(&mut self, mask: u32, first: i32, signals: i32) -> i32 {
        let rc = unsafe { cref_core_mct_initialize(&mut self.c, mask, first, signals) };
        let rr = cmct_initialize(&mut self.r, mask, first, signals);
        assert_eq!(rr, rc, "CMct_Initialize({mask:#x}, {first}, {signals})");
        assert_eq!(self.c.is_null(), self.r.is_none());
        rr
    }

    fn rm(&mut self) -> &mut CMct {
        self.r.as_mut().unwrap()
    }

    #[track_caller]
    fn check(&self, what: &str) {
        let c = c_dump(|o| unsafe { cref_core_mct_dump(self.c, o) });
        assert_same(&format!("{what}: CMct"), &c, &dump_mct(self.r.as_ref().unwrap()));
    }

    /// Loads a state (in the dump layout) into both implementations.
    fn load(&mut self, a: &[i32]) {
        unsafe { cref_core_mct_load(self.c, a.as_ptr()) };
        load_mct(self.rm(), a);
    }
}

impl Drop for MctPair {
    fn drop(&mut self) {
        if !self.c.is_null() {
            unsafe { cref_core_mct_destroy(self.c) };
        }
        cmct_destroy(self.r.take());
    }
}

// ---------------------------------------------------------------------------------------------
// Bitstreams
// ---------------------------------------------------------------------------------------------

const BS_SIZE: usize = 8192;

struct BsPair {
    _buf_c: Vec<u8>,
    _buf_r: Vec<u8>,
    hc: *mut (),
    r: FdkBitStream,
}

impl BsPair {
    /// A reader over `bytes`, padded with `pad` to `BS_SIZE` bytes.
    fn new(bytes: &[u8], pad: &[u8]) -> Self {
        let mut buf_c = bytes.to_vec();
        buf_c.extend_from_slice(pad);
        buf_c.resize(BS_SIZE, 0);
        let buf_r = buf_c.clone();
        let size = BS_SIZE as u32;
        let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, size * 8, 0) };
        let mut r = FdkBitStream::default();
        r.init(&BitMem::from_slice(&buf_r), size, size * 8, FdkBsCfg::BsReader);
        BsPair { _buf_c: buf_c, _buf_r: buf_r, hc, r }
    }

    #[track_caller]
    fn check(&self, what: &str) {
        let mut st = [0u32; 6];
        unsafe { cref_bs_state(self.hc, st.as_mut_ptr()) };
        let r = &self.r;
        let rs = [
            r.cache_word,
            r.bits_in_cache,
            r.h_bit_buf.bit_ndx(),
            r.h_bit_buf.valid_bits as u32,
            r.h_bit_buf.read_offset(),
            r.h_bit_buf.write_offset(),
        ];
        assert_eq!(rs, st, "{what}: bitstream");
    }
}

impl Drop for BsPair {
    fn drop(&mut self) {
        unsafe { cref_bs_free(self.hc) };
    }
}

fn random_bytes(g: &mut Rng, n: usize) -> Vec<u8> {
    let kind = g.range(0, 4);
    (0..n)
        .map(|_| match kind {
            0 => g.next_u32() as u8,
            1 => (g.next_u32() & g.next_u32() & g.next_u32()) as u8,
            2 => (g.next_u32() | g.next_u32() | g.next_u32()) as u8,
            3 => {
                if g.range(0, 7) == 0 {
                    g.next_u32() as u8
                } else {
                    0
                }
            }
            _ => {
                if g.range(0, 7) == 0 {
                    g.next_u32() as u8
                } else {
                    0xFF
                }
            }
        })
        .collect()
}

/// Parses with both implementations and compares return value, bitstream and state.
#[track_caller]
fn parse(m: &mut MctPair, bs: &mut BsPair, indep: i32, n_channels: i32, what: &str) -> i32 {
    let rc = unsafe { cref_core_mct_parse(m.c, bs.hc, indep, n_channels) };
    let rr = cmct_inverse_mct_parse_bs(m.rm(), &mut bs.r, indep, n_channels);
    assert_eq!(rr, rc, "{what}: return value");
    bs.check(what);
    m.check(what);
    rr
}

/// MSB first bit writer.
#[derive(Default)]
struct BitWriter {
    bytes: Vec<u8>,
    nbits: usize,
}

impl BitWriter {
    fn put(&mut self, v: u32, n: u32) {
        for i in (0..n).rev() {
            if self.nbits % 8 == 0 {
                self.bytes.push(0);
            }
            if (v >> i) & 1 != 0 {
                *self.bytes.last_mut().unwrap() |= 0x80 >> (self.nbits % 8);
            }
            self.nbits += 1;
        }
    }

    /// Inverse of `escapedValue(hBs, 5, 8, 16)`.
    fn escaped(&mut self, v: u32) {
        if v < 31 {
            self.put(v, 5);
        } else {
            self.put(31, 5);
            let v = v - 31;
            if v < 255 {
                self.put(v, 8);
            } else {
                self.put(255, 8);
                self.put(v - 255, 16);
            }
        }
    }

    fn huffman(&mut self, g: &mut Rng, ctab: &[i32], ltab: &[i32]) {
        let k = g.range(0, ctab.len() as i32 - 1) as usize;
        self.put(ctab[k] as u32, ltab[k] as u32);
    }
}

/// Kinds of errors injected into crafted MCT frames.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Errors {
    /// No errors.
    None,
    /// All kinds of errors.
    All,
    /// Only errors after which CMct_MCT_StereoFilling() has defined behaviour in C++: errors that
    /// make the parser fail (MCCSignalingType 0, too many pairs) and invalid channel pairs of
    /// MCCSignalingType 1 boxes whose stale data is valid.
    StereoSafe,
}

/// Writes a random `MultichannelCodingFrame()` for the current state `m`.
fn write_mct_frame(g: &mut Rng, m: &CMct, indep: bool, errors: Errors) -> BitWriter {
    let mut w = BitWriter::default();
    let err = |g: &mut Rng| errors != Errors::None && g.range(0, 15) == 0;

    let stereo_filling = g.range(0, 3) != 0;
    let typ = if g.range(0, 4) == 0 {
        1 - m.mcc_signaling_type.min(1) as u32
    } else {
        m.mcc_signaling_type.min(1) as u32
    };
    w.put(stereo_filling as u32, 1);
    w.put(typ, 1);
    let mut keep_tree = false;
    if !indep {
        keep_tree = g.range(0, 2) == 0;
        w.put(keep_tree as u32, 1);
    }

    let n_ch = m.num_mct_channels as u32;
    let n_pairs_possible = n_ch * n_ch.saturating_sub(1) / 2;
    let num_pairs = if keep_tree {
        m.mct_work.num_pairs as u32
    } else {
        let np = if errors != Errors::None && g.range(0, 30) == 0 {
            g.range(29, 400) as u32
        } else if n_pairs_possible == 0 {
            /* MCCSignalingType 0 fails at the first box, 1 skips the boxes */
            if typ == 0 && errors == Errors::None {
                0
            } else {
                g.range(0, 3) as u32
            }
        } else {
            g.range(0, (n_pairs_possible + 2).min(28) as i32) as u32
        };
        w.escaped(np);
        if np > 28 {
            return w;
        }
        np
    };

    for i in 0..num_pairs as usize {
        if stereo_filling {
            w.put(g.range(0, 1) as u32, 1);
        }

        /* channel pair */
        if !keep_tree {
            if n_pairs_possible == 0 {
                /* CMct_inverseMctParseChannelPair() fails without reading */
                if typ == 0 {
                    return w;
                }
                continue;
            }
            let mut max_idx = n_pairs_possible as i32 - 1;
            let mut num_bits = 1;
            loop {
                max_idx >>= 1;
                if max_idx == 0 {
                    break;
                }
                num_bits += 1;
            }
            let stale_ok = {
                let cp = m.mct_work.code_pairs[i];
                cp[0] < cp[1] && m.mct_work.num_mct_mask_bands[i] <= 64
            };
            let invalid_possible = n_pairs_possible < (1 << num_bits);
            let inject =
                invalid_possible && err(g) && (errors == Errors::All || typ == 0 || stale_ok);
            if inject {
                w.put(g.range(n_pairs_possible as i32, (1 << num_bits) - 1) as u32, num_bits);
                if typ == 0 {
                    return w;
                }
                continue;
            }
            w.put(g.range(0, n_pairs_possible as i32 - 1) as u32, num_bits);
        }

        let has_mask = g.range(0, 2) == 0;
        let bandwise = g.range(0, 2) == 0;
        w.put(has_mask as u32, 1);
        w.put(bandwise as u32, 1);
        let mut n_bands = 64;
        if has_mask || bandwise {
            let short = g.range(0, 2) == 0;
            let bands5 = if short {
                if err(g) && (errors == Errors::All || typ == 0) {
                    g.range(9, 31)
                } else {
                    g.range(0, 8)
                }
            } else {
                g.range(0, 31)
            };
            w.put(short as u32, 1);
            w.put(bands5 as u32, 5);
            n_bands = bands5 * if short { 8 } else { 1 };
            if n_bands > 64 {
                if typ == 0 {
                    return w;
                }
                continue;
            }
        }
        let mut mask = u64::MAX;
        if has_mask && n_bands > 0 {
            let p = g.range(0, 4);
            mask = 0;
            for j in 0..n_bands {
                let bit = g.range(0, 3) < p;
                w.put(bit as u32, 1);
                if bit {
                    mask |= 1u64 << (63 - j);
                }
            }
        }
        if typ == 0 {
            w.put(g.range(0, 1) as u32, 1);
        }
        if !indep {
            w.put(g.range(0, 1) as u32, 1);
        }
        let (ctab, ltab): (&[i32], &[i32]) = if typ == 0 {
            (&HUFF_CTABSCF, &HUFF_LTABSCF)
        } else {
            (&HUFF_CTAB_ANGLE, &HUFF_LTAB_ANGLE)
        };
        if !bandwise {
            w.huffman(g, ctab, ltab);
        } else {
            for j in 0..n_bands {
                if mask & (1u64 << (63 - j)) != 0 {
                    w.huffman(g, ctab, ltab);
                }
            }
        }
    }
    w
}

fn random_indep(g: &mut Rng) -> i32 {
    match g.range(0, 5) {
        0 => 1,
        1 => AC_INDEP as i32,
        _ => 0,
    }
}

// ---------------------------------------------------------------------------------------------
// IGF state dumps (mirror cref_igf_dump_static() / cref_igf_dump_data())
// ---------------------------------------------------------------------------------------------

fn dump_igf_static(s: &IgfPrivateStaticData) -> Vec<i32> {
    let mut o = Vec::new();
    let h = &s.h_igf_scf_decoder;
    o.extend([h.st.low, h.st.high, h.st.vobf, h.st.bits_to_follow]);
    o.extend(h.prev_buffer.iter().map(|&v| v as i32));
    o.extend([h.prev_sb, h.prev_lb, h.prev_tcx_medium, h.prev_tcx_long].map(|v| v as i32));
    o.extend([h.prev_dlb, h.prev_dsb, h.prev_dtcx_medium, h.prev_dtcx_long].map(|v| v as i32));
    o.extend([h.scf_count_long_block, h.scf_count_short_block, h.scf_count_tcx_block]);
    o.extend([h.t, h.last_frame_block_type, h.arith_decode_first_symbol]);
    for gi in &s.s_grid_info_tab {
        for (t, m) in gi.s_igf_map_info_tab.iter().enumerate() {
            o.push(if t < gi.i_igf_num_tile as usize { (t * IGF_TILE_LEN) as i32 } else { -1 });
            o.push(m.i_whitening_level);
            o.extend(m.i_sfb_width_tab.iter().map(|&v| v as i32));
            o.extend([m.i_sfb_cnt as i32, m.i_des_start as i32, m.i_width as i32]);
            o.extend([m.i_idx as i32, m.i_sfb_split as i32]);
        }
        o.extend([gi.i_igf_start_line, gi.i_igf_stop_line, gi.i_igf_min_line].map(|v| v as i32));
        o.extend(gi.i_igf_tile_idx_tab.iter().map(|&v| v as i32));
        o.extend(
            [gi.i_igf_num_sfb, gi.i_igf_start_sfb, gi.i_igf_stop_sfb, gi.i_igf_num_tile]
                .map(|v| v as i32),
        );
    }
    for t in &s.f_spectrum_scale {
        push_i16(&mut o, t);
    }
    for t in &s.f_spectrum_tab_sfb_exp {
        push_i16(&mut o, t);
    }
    o.extend(s.igf_whitening_level_prev);
    o.extend([s.igf_min_subband_lb, s.igf_min_subband_sb]);
    o.extend(s.igf_tiles_lb);
    o.extend(s.igf_tiles_sb);
    push_u8(&mut o, &s.prev_patch_num);
    /* the C++ offset table pointers are NULL before iisIGFDecLibInit() */
    o.push(s.num_sfb_lb as i32);
    if s.num_sfb_lb != 0 {
        push_i16(&mut o, &s.sfb_offset_lb[..=s.num_sfb_lb as usize]);
    }
    o.push(s.num_sfb_sb as i32);
    if s.num_sfb_sb != 0 {
        push_i16(&mut o, &s.sfb_offset_sb[..=s.num_sfb_sb as usize]);
    }
    o.extend(
        [
            s.enhanced_noise_filling,
            s.igf_use_whitening,
            s.igf_after_tns_synth,
            s.igf_independent_tiling,
            s.use_high_res,
            s.igf_start_sfb_lb,
            s.igf_stop_sfb_lb,
            s.igf_start_sfb_sb,
            s.igf_stop_sfb_sb,
            s.igf_n_tiles_lb,
            s.igf_n_tiles_sb,
        ]
        .map(|v| v as i32),
    );
    o
}

fn dump_igf_data(d: &IgfPrivateData) -> Vec<i32> {
    let mut o = Vec::new();
    o.extend_from_slice(&d.spectrum_tab_array);
    for b in &d.bitstream_data {
        o.extend(b.sfe.iter().map(|&v| v as i32));
        push_u8(&mut o, &b.i_curr_tile_idx_tab);
        o.extend(b.igf_whitening_level);
        o.extend([b.igf_all_zero as i32, b.igf_use_enf_flat as i32]);
    }
    push_u8(&mut o, &d.tile_num);
    o
}

// ---------------------------------------------------------------------------------------------
// The channels of a signal group in C and Rust
// ---------------------------------------------------------------------------------------------

struct Group {
    ch: Vec<ChPair>,
    st: Vec<StPair>,
    c_spec: Vec<i32>,
    r_spec: Vec<i32>,
    igf_common: *mut (),
    /// Whether iisIGFDecLibInit() was called for the channel.
    igf_init: Vec<bool>,
    sr_idx: u32,
    sample_rate: u32,
    sri: SamplingRateInfo,
}

impl Group {
    fn new(n: usize, sr_idx: u32) -> Self {
        let sample_rate = SAMPLE_RATES[sr_idx as usize];
        let mut sri = SamplingRateInfo::default();
        assert_eq!(get_sampling_rate_info(&mut sri, 1024, sr_idx, sample_rate), 0);
        let mut grp = Group {
            ch: (0..n).map(|_| ChPair::new()).collect(),
            st: (0..n).map(|_| StPair::new()).collect(),
            c_spec: vec![0; n * 1024],
            r_spec: vec![0; n * 1024],
            igf_common: unsafe { cref_igf_common_new() },
            igf_init: vec![false; n],
            sr_idx,
            sample_rate,
            sri,
        };
        for i in 0..n {
            let p = unsafe { grp.c_spec.as_mut_ptr().add(i * 1024) };
            unsafe { cref_core_ch_set_spec(grp.ch[i].c, p) };
        }
        grp
    }

    fn n(&self) -> usize {
        self.ch.len()
    }

    /// `iisIGFDecLibInit()` of channel `i` with random parameters.
    fn init_igf(&mut self, g: &mut Rng, i: usize) {
        let info = &SFB_OFFSET_TABLES[0][self.sr_idx as usize];
        let (start, stop) = (g.range(0, 15) as u8, g.range(0, 15) as u8);
        let (high_res, whitening) = (g.range(0, 1) as u8, g.range(0, 1) as u8);
        let (after_tns, indep_tiling) = (g.range(0, 1) as u8, g.range(0, 1) as u8);
        let (mut len_lb, mut len_sb) = (0, 0);
        let plb = unsafe { cref_igf_sfb_table(self.sr_idx as i32, 0, &mut len_lb) };
        let psb = unsafe { cref_igf_sfb_table(self.sr_idx as i32, 1, &mut len_sb) };
        assert_eq!(len_lb, info.number_of_sfb_long as i32);
        assert_eq!(len_sb, info.number_of_sfb_short as i32);
        unsafe {
            cref_igf_init(
                cref_core_st_igf(self.st[i].c),
                cref_core_ch_igf(self.ch[i].c),
                self.igf_common,
                start,
                stop,
                high_res,
                whitening,
                self.sample_rate,
                1024,
                plb,
                len_lb,
                psb,
                len_sb,
                1,
                after_tns,
                indep_tiling,
            )
        };
        iis_igf_dec_lib_init(
            &mut self.st[i].r.igf_static_data,
            &mut self.ch[i].r.igf_data,
            start,
            stop,
            high_res,
            whitening,
            self.sample_rate,
            1024,
            info.sfb_offset_long,
            len_lb,
            info.sfb_offset_short,
            len_sb,
            1,
            after_tns,
            indep_tiling,
        );
        self.igf_init[i] = true;
    }

    /// Random channel data of one frame (Rust), then copied to C.
    fn randomize(&mut self, g: &mut Rng, frame_short: bool) {
        let n = self.n();
        for i in 0..n {
            let ch = &mut self.ch[i].r;
            let short = if g.range(0, 15) == 0 { g.range(0, 1) == 1 } else { frame_short };
            let ics = &mut ch.ics_info;
            if short {
                ics.window_sequence = BLOCK_SHORT;
                ics.scale_factor_grouping = g.range(0, 127) as u8;
                ics.window_groups = 0;
                for k in 0..7 {
                    ics.window_group_length[k] = 1;
                    if ics.scale_factor_grouping as u32 & (1 << (6 - k)) != 0 {
                        ics.window_group_length[ics.window_groups as usize] += 1;
                    } else {
                        ics.window_groups += 1;
                    }
                }
                ics.window_group_length[7] = 1;
                ics.window_groups += 1;
                ics.total_sf_bands = self.sri.number_of_scale_factor_bands_short;
            } else {
                ics.window_sequence = [0, 1, 3][g.range(0, 2) as usize];
                ics.window_groups = 1;
                ics.window_group_length[0] = 1;
                ics.total_sf_bands = self.sri.number_of_scale_factor_bands_long;
            }
            ics.max_sf_bands = if g.range(0, 2) == 0 {
                ics.total_sf_bands
            } else {
                g.range(0, ics.total_sf_bands as i32) as u8
            };
            ics.window_shape = g.range(0, 1) as u8;
            ics.valid = 1;
            ch.granule_length = 128;

            let offsets = get_scale_factor_band_offsets(&ch.ics_info, &self.sri);
            let nsfb = ch.ics_info.total_sf_bands as usize;
            let nwin = if short { 8 } else { 1 };
            let spec = &mut self.r_spec[i * 1024..(i + 1) * 1024];
            for w in 0..nwin {
                for sfb in 0..nsfb {
                    let kind = g.range(0, 5);
                    for k in offsets[sfb] as usize..offsets[sfb + 1] as usize {
                        spec[w * 128 * (short as usize) + k] = match kind {
                            0 => 0,
                            1 => g.i32() >> g.range(8, 31),
                            2 => g.i32() >> 4,
                            _ => g.i32(),
                        };
                    }
                }
            }
            let d = &mut ch.p_dyn_data;
            let exp_kind = g.range(0, 7);
            for v in d.a_sfb_scale.iter_mut() {
                *v = match exp_kind {
                    0 => g.i16(),
                    1 => g.range(-40, 60) as i16,
                    _ => g.range(-6, 20) as i16,
                };
            }
            for v in d.a_scale_factor.iter_mut() {
                *v = if g.range(0, 15) == 0 { g.i16() } else { g.range(-40, 200) as i16 };
            }
            let p_noise = g.range(0, 4);
            for v in d.band_is_noise.iter_mut() {
                *v = (g.range(0, 3) < p_noise) as u8;
            }

            let st = &mut self.st[i].r;
            st.concealment_info.window_sequence =
                if g.range(0, 3) == 0 { g.range(0, 3) } else { ch.ics_info.window_sequence };
            st.concealment_info.last_render_mode = [0, 1, 3, 4][g.range(0, 3) as usize];
            if g.range(0, 5) == 0 {
                st.concealment_info.last_render_mode = 3;
            }
            st.nf_random_seed = g.next_u32();

            /* IGF tiles */
            for v in ch.igf_data.spectrum_tab_array.iter_mut() {
                *v = if g.range(0, 3) == 0 { g.i32() >> g.range(0, 20) } else { g.i32() >> 8 };
            }
            for t in st.igf_static_data.f_spectrum_tab_sfb_exp.iter_mut() {
                for v in t.iter_mut() {
                    *v = if g.range(0, 20) == 0 { g.i16() } else { g.range(-6, 20) as i16 };
                }
            }
            for v in ch.igf_data.bitstream_data[0].igf_whitening_level.iter_mut() {
                *v = g.range(0, 3);
            }
        }
        self.sync_to_c();
    }

    fn sync_to_c(&mut self) {
        for i in 0..self.n() {
            self.ch[i].sync_to_c();
            self.st[i].sync_to_c();
            let sd = &self.st[i].r.igf_static_data;
            let d = &self.ch[i].r.igf_data;
            let exps: Vec<i16> = sd.f_spectrum_tab_sfb_exp.iter().flatten().copied().collect();
            let wl = d.bitstream_data[0].igf_whitening_level;
            unsafe {
                let cs = cref_core_st_igf(self.st[i].c);
                let cd = cref_core_ch_igf(self.ch[i].c);
                cref_igf_set_tile_spectrum(cd, d.spectrum_tab_array.as_ptr());
                cref_core_mct_igf_set(cs, cd, exps.as_ptr(), wl.as_ptr());
            }
        }
        self.c_spec.copy_from_slice(&self.r_spec);
    }

    #[track_caller]
    fn check(&self, what: &str) {
        for i in 0..self.n() {
            let w = format!("{what}: channel {i}");
            self.ch[i].check(&w);
            self.st[i].check(&w);
            let mut cs = c_dump(|o| unsafe {
                cref_igf_dump_static(
                    cref_core_st_igf(self.st[i].c),
                    cref_core_ch_igf(self.ch[i].c),
                    o,
                )
            });
            let mut rs = dump_igf_static(&self.st[i].r.igf_static_data);
            if !self.igf_init[i] {
                /* prevSB, prevLB, prevTCXmedium, prevTCXlong are dumped as pointer differences,
                which are meaningless before iisIGFDecLibInit() (NULL pointers) */
                cs[116..120].fill(0);
                rs[116..120].fill(0);
            }
            assert_same(&format!("{w}: IGF static"), &cs, &rs);
            let cd = c_dump(|o| unsafe { cref_igf_dump_data(cref_core_ch_igf(self.ch[i].c), o) });
            assert_same(&format!("{w}: IGF data"), &cd, &dump_igf_data(&self.ch[i].r.igf_data));
        }
        assert_same(&format!("{what}: spectra"), &self.c_spec, &self.r_spec);
    }

    fn c_ch(&self) -> Vec<*mut ()> {
        self.ch.iter().map(|p| p.c).collect()
    }

    fn c_st(&self) -> Vec<*mut ()> {
        self.st.iter().map(|p| p.c).collect()
    }

    /// Runs `f` on the Rust channel structures (moved out of the pairs temporarily).
    fn with_rust<R>(
        &mut self,
        f: impl FnOnce(
            &mut [Box<CAacDecoderChannelInfo>],
            &mut [Box<CAacDecoderStaticChannelInfo>],
            &mut [i32],
            &SamplingRateInfo,
        ) -> R,
    ) -> R {
        let mut chs: Vec<_> = self.ch.iter_mut().map(|p| std::mem::take(&mut p.r)).collect();
        let mut sts: Vec<_> = self.st.iter_mut().map(|p| std::mem::take(&mut p.r)).collect();
        let r = f(&mut chs, &mut sts, &mut self.r_spec, &self.sri);
        for (p, b) in self.ch.iter_mut().zip(chs) {
            p.r = b;
        }
        for (p, b) in self.st.iter_mut().zip(sts) {
            p.r = b;
        }
        r
    }

    /// `CMct_MCT_StereoFilling()` in both implementations; compares everything.
    #[track_caller]
    fn stereo_filling(&mut self, m: &mut MctPair, flags: &[u32], indep: u32, what: &str) -> i32 {
        let mut cch = self.c_ch();
        let mut cst = self.c_st();
        let rc = unsafe {
            cref_core_mct_stereo_filling(
                m.c,
                1024,
                cch.as_mut_ptr(),
                cst.as_mut_ptr(),
                self.n() as i32,
                self.sr_idx,
                self.sample_rate,
                flags.as_ptr(),
                indep,
            )
        };
        let mct = m.r.as_mut().unwrap();
        let rr = self.with_rust(|ch, st, spec, sri| {
            cmct_mct_stereo_filling(mct, 1024, ch, st, spec, sri, flags, indep)
        });
        assert_eq!(rr, rc, "{what}: CMct_MCT_StereoFilling return value");
        self.check(what);
        m.check(what);
        rr
    }

    #[track_caller]
    fn save_prev(&mut self, m: &mut MctPair, what: &str) {
        let mut cch = self.c_ch();
        unsafe { cref_core_mct_save_prev(m.c, cch.as_mut_ptr(), self.n() as i32) };
        let mct = m.r.as_mut().unwrap();
        self.with_rust(|ch, _, spec, _| cmct_stereo_filling_save_prev(mct, ch, spec));
        m.check(what);
    }

    #[track_caller]
    fn clear_prev(&mut self, m: &mut MctPair, what: &str) {
        let mut cch = self.c_ch();
        unsafe { cref_core_mct_clear_prev(m.c, cch.as_mut_ptr(), self.n() as i32) };
        cmct_stereo_filling_clear_prev(m.rm());
        m.check(what);
    }
}

impl Drop for Group {
    fn drop(&mut self) {
        unsafe { cref_igf_common_free(self.igf_common) };
    }
}

// ---------------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------------

#[test]
fn initialize() {
    let mut g = Rng::new(0x3C7_0001);
    for it in 0..3000 {
        let signals = g.range(0, 28);
        let mask = match g.range(0, 5) {
            0 => 0,
            1 => u32::MAX,
            2 => g.next_u32() & g.next_u32(),
            _ => g.next_u32(),
        };
        let first = g.range(0, 30);
        let mut m = MctPair::new();
        assert_eq!(m.initialize(mask, first, signals), 0);
        m.check(&format!("init {it}"));
        let n = m.r.as_ref().unwrap().num_mct_channels;
        assert_eq!(n as u32, (mask >> (32 - signals.max(1))).count_ones() * (signals > 0) as u32);
        /* an allocated handle is not initialized again */
        assert_eq!(m.initialize(g.next_u32(), 0, 5), -1);
        m.check(&format!("init again {it}"));
    }
}

#[test]
fn parse_random() {
    let mut g = Rng::new(0x3C7_0002);
    for it in 0..3000 {
        let signals = g.range(0, 28);
        let mask = if g.range(0, 3) == 0 { g.next_u32() } else { u32::MAX };
        let mut m = MctPair::new();
        m.initialize(mask, g.range(0, 3), signals);
        let bytes = random_bytes(&mut g, BS_SIZE);
        let mut bs = BsPair::new(&bytes, &[]);
        for f in 0..8 {
            let indep = random_indep(&mut g);
            parse(&mut m, &mut bs, indep, signals, &format!("random {it}/{f}"));
        }
    }
}

#[test]
fn parse_crafted() {
    let mut g = Rng::new(0x3C7_0003);
    for it in 0..1500 {
        let signals = g.range(0, 28);
        let mask = match g.range(0, 3) {
            0 => g.next_u32(),
            1 => u32::MAX,
            _ => g.next_u32() | g.next_u32(),
        };
        let mut m = MctPair::new();
        m.initialize(mask, 0, signals);
        for f in 0..12 {
            let indep = random_indep(&mut g);
            let errors = if g.range(0, 1) == 0 { Errors::All } else { Errors::None };
            let w = write_mct_frame(&mut g, m.r.as_ref().unwrap(), indep != 0, errors);
            let pad = random_bytes(&mut g, 64);
            let mut bs = BsPair::new(&w.bytes, &pad);
            let what = format!("crafted {it}/{f}");
            let ret = parse(&mut m, &mut bs, indep, signals, &what);
            if errors == Errors::None {
                assert_eq!(ret, 0, "{what}");
                /* the parser consumed exactly the written bits */
                assert_eq!(bs.r.get_valid_bits(), (BS_SIZE * 8 - w.nbits) as i32, "{what}");
            }
        }
    }
}

/// Random MCT element flags of the channels.
fn random_flags(g: &mut Rng, n: usize) -> Vec<u32> {
    let p = g.range(0, 4);
    (0..n)
        .map(|_| {
            let mut f = g.next_u32() & !(AC_EL_ENHANCED_NOISE | AC_EL_IGF_USE_ENF);
            if g.range(0, 3) < p {
                f |= AC_EL_ENHANCED_NOISE;
            }
            if g.range(0, 3) < p {
                f |= AC_EL_IGF_USE_ENF;
            }
            f
        })
        .collect()
}

/// A signal group with random channels, channel map and IGF configuration.
fn random_group(g: &mut Rng) -> (Group, MctPair, i32) {
    let n = g.range(2, 8) as usize;
    let sr_idx = g.range(0, 11) as u32;
    let mut grp = Group::new(n, sr_idx);
    for i in 0..n {
        if g.range(0, 2) != 0 {
            grp.init_igf(g, i);
        }
    }
    let first = g.range(0, (n as i32 - 2).min(2));
    let signals = n as i32 - first;
    /* at least two MCT channels */
    let mask = loop {
        let mk = if g.range(0, 2) == 0 { u32::MAX } else { g.next_u32() };
        if (mk >> (32 - signals)).count_ones() >= 2 {
            break mk;
        }
    };
    let mut m = MctPair::new();
    assert_eq!(m.initialize(mask, first, signals), 0);
    (grp, m, signals)
}

#[test]
fn stereo_filling() {
    let mut g = Rng::new(0x3C7_0004);
    for it in 0..1500 {
        let (mut grp, mut m, signals) = random_group(&mut g);
        let short_prob = g.range(0, 3);
        for f in 0..16 {
            let what = format!("stereo filling {it}/{f}");
            let frame_short = g.range(0, 5) < short_prob;
            grp.randomize(&mut g, frame_short);
            grp.check(&format!("{what}: randomize"));

            let indep = random_indep(&mut g);
            let errors = if g.range(0, 3) == 0 { Errors::StereoSafe } else { Errors::None };
            let w = write_mct_frame(&mut g, m.r.as_ref().unwrap(), indep != 0, errors);
            let pad = random_bytes(&mut g, 64);
            let mut bs = BsPair::new(&w.bytes, &pad);
            let ret = parse(&mut m, &mut bs, indep, signals, &what);

            if ret == 0 && g.range(0, 9) != 0 {
                let flags = random_flags(&mut g, grp.n());
                let err = grp.stereo_filling(&mut m, &flags, (indep != 0) as u32, &what);
                if err == 0 {
                    grp.save_prev(&mut m, &format!("{what}: save_prev"));
                }
            } else {
                grp.clear_prev(&mut m, &format!("{what}: clear_prev"));
            }
        }
    }
}

/// Channel pairs that refer to the same channel twice (possible for stale channel pairs of
/// MCCSignalingType 1): the aliased accesses must follow the sequential C++ semantics.
#[test]
fn stereo_filling_aliased_pairs() {
    let mut g = Rng::new(0x3C7_0005);
    for it in 0..600 {
        let (mut grp, mut m, signals) = random_group(&mut g);
        for f in 0..8 {
            let what = format!("aliased {it}/{f}");
            let frame_short = g.range(0, 2) == 0;
            grp.randomize(&mut g, frame_short);
            let indep = random_indep(&mut g);
            let w = write_mct_frame(&mut g, m.r.as_ref().unwrap(), indep != 0, Errors::None);
            let mut bs = BsPair::new(&w.bytes, &[]);
            assert_eq!(parse(&mut m, &mut bs, indep, signals, &what), 0);

            /* let some pairs refer to one channel */
            let st = {
                let mct = m.r.as_ref().unwrap();
                let mut state = (**mct).clone();
                for p in 0..mct.mct_work.num_pairs as usize {
                    if g.range(0, 1) == 0 {
                        let k = g.range(0, mct.num_mct_channels as i32 - 1) as u8;
                        state.mct_work.code_pairs[p] = [k, k];
                    }
                }
                dump_mct(&state)
            };
            m.load(&st);
            m.check(&format!("{what}: load"));

            let flags = random_flags(&mut g, grp.n());
            if grp.stereo_filling(&mut m, &flags, (indep != 0) as u32, &what) == 0 {
                grp.save_prev(&mut m, &format!("{what}: save_prev"));
            }
        }
    }
}
