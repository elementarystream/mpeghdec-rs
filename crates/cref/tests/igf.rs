// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libIGFdec (FDK_igfDec.cpp, FDK_igfSCFDecoder.cpp): initialization for
//! all configurations, bitstream parsing and IGF processing over many frames with persistent state
//! are run with the C and the Rust implementation, and all state is compared after every call.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::igf::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_igf::igf_dec::*;

/// Nominal sampling rates of the entries of `sfbOffsetTables[0]`.
const SAMPLE_RATES: [u32; 13] =
    [96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350];
/// Sampling rate ranges `SR_BORDERS[i + 1]..SR_BORDERS[i]` mapped to the table entries
/// (`getSamplingRateInfo`).
const SR_BORDERS: [u32; 13] =
    [120000, 92017, 75132, 55426, 46009, 37566, 27713, 23004, 18783, 13856, 11502, 9391, 7000];

fn sfb_table(sr_idx: i32, short: bool) -> (*const i16, Vec<i16>, i32) {
    let mut len = 0;
    let p = unsafe { cref_igf_sfb_table(sr_idx, short as i32, &mut len) };
    let v = unsafe { std::slice::from_raw_parts(p, len as usize + 1) }.to_vec();
    (p, v, len)
}

// ---------------------------------------------------------------------------------------------
// State dumps in the same order as the C shim (cref_igf_dump_*)
// ---------------------------------------------------------------------------------------------

fn dump_static(s: &IgfPrivateStaticData) -> Vec<i32> {
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
        o.extend(t.iter().map(|&v| v as i32));
    }
    for t in &s.f_spectrum_tab_sfb_exp {
        o.extend(t.iter().map(|&v| v as i32));
    }
    o.extend(s.igf_whitening_level_prev);
    o.extend([s.igf_min_subband_lb, s.igf_min_subband_sb]);
    o.extend(s.igf_tiles_lb);
    o.extend(s.igf_tiles_sb);
    o.extend(s.prev_patch_num.iter().map(|&v| v as i32));
    o.push(s.num_sfb_lb as i32);
    o.extend(s.sfb_offset_lb[..=s.num_sfb_lb as usize].iter().map(|&v| v as i32));
    o.push(s.num_sfb_sb as i32);
    o.extend(s.sfb_offset_sb[..=s.num_sfb_sb as usize].iter().map(|&v| v as i32));
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

fn dump_data(d: &IgfPrivateData) -> Vec<i32> {
    let mut o = Vec::new();
    o.extend_from_slice(&d.spectrum_tab_array);
    for b in &d.bitstream_data {
        o.extend(b.sfe.iter().map(|&v| v as i32));
        o.extend(b.i_curr_tile_idx_tab.iter().map(|&v| v as i32));
        o.extend(b.igf_whitening_level);
        o.extend([b.igf_all_zero as i32, b.igf_use_enf_flat as i32]);
    }
    o.extend(d.tile_num.iter().map(|&v| v as i32));
    o
}

fn dump_common(c: &IgfPrivateDataCommon) -> Vec<i32> {
    let mut o = Vec::new();
    for w in &c.igf_working_mem {
        o.extend_from_slice(&w.f_sfb_gain_tab);
        o.extend_from_slice(&w.f_sfb_destin_energy_tab);
        o.extend_from_slice(&w.f_sfb_tile_energy_tab);
        o.extend_from_slice(&w.f_sfb_survived_energy_tab);
        o.extend_from_slice(&w.s_e);
        o.extend_from_slice(&w.t_e);
        for a in [
            &w.f_sfb_gain_tab_exp[..],
            &w.f_sfb_destin_energy_tab_exp[..],
            &w.f_sfb_tile_energy_tab_exp[..],
            &w.f_sfb_survived_energy_tab_exp[..],
            &w.s_e_exp[..],
            &w.t_e_exp[..],
            &w.intermediate_shift[..],
            &w.initial_exp[..],
        ] {
            o.extend(a.iter().map(|&v| v as i32));
        }
    }
    o
}

fn c_dump(f: impl FnOnce(*mut i32) -> i32) -> Vec<i32> {
    let mut buf = vec![0i32; 1 << 15];
    let n = f(buf.as_mut_ptr());
    buf.truncate(n as usize);
    buf
}

/// Index and values of the first difference (for readable assertion messages).
fn first_diff(a: &[i32], b: &[i32]) -> Option<(usize, i32, i32)> {
    if a.len() != b.len() {
        return Some((usize::MAX, a.len() as i32, b.len() as i32));
    }
    a.iter().zip(b).enumerate().find(|(_, (x, y))| x != y).map(|(i, (x, y))| (i, *x, *y))
}

// ---------------------------------------------------------------------------------------------
// One IGF channel in C and Rust
// ---------------------------------------------------------------------------------------------

struct Chan {
    cs: *mut (),
    cd: *mut (),
    cc: *mut (),
    rs: Box<IgfPrivateStaticData>,
    rd: Box<IgfPrivateData>,
    rc: Box<IgfPrivateDataCommon>,
    rvs: Vec<i32>,
}

impl Chan {
    fn new() -> Self {
        unsafe {
            Chan {
                cs: cref_igf_static_new(),
                cd: cref_igf_data_new(),
                cc: cref_igf_common_new(),
                rs: Box::default(),
                rd: Box::default(),
                rc: Box::default(),
                rvs: vec![0; 1024],
            }
        }
    }

    fn c_virtual_spec(&self) -> &[i32] {
        unsafe { std::slice::from_raw_parts(cref_igf_common_virtual_spec(self.cc), 1024) }
    }

    fn c_virtual_spec_mut(&mut self) -> &mut [i32] {
        unsafe { std::slice::from_raw_parts_mut(cref_igf_common_virtual_spec(self.cc), 1024) }
    }

    fn init(&mut self, cfg: &Cfg) {
        let (plb, lb, len_lb) = sfb_table(cfg.sr_idx, false);
        let (psb, sb, len_sb) = sfb_table(cfg.sr_idx, true);
        unsafe {
            cref_igf_init(
                self.cs,
                self.cd,
                self.cc,
                cfg.start,
                cfg.stop,
                cfg.high_res,
                cfg.whitening,
                cfg.sample_rate,
                1024,
                plb,
                len_lb,
                psb,
                len_sb,
                1,
                cfg.after_tns,
                cfg.indep_tiling,
            )
        };
        iis_igf_dec_lib_init(
            &mut self.rs,
            &mut self.rd,
            cfg.start,
            cfg.stop,
            cfg.high_res,
            cfg.whitening,
            cfg.sample_rate,
            1024,
            &lb,
            len_lb,
            &sb,
            len_sb,
            1,
            cfg.after_tns,
            cfg.indep_tiling,
        );
    }

    fn check(&self, ctx: &str) {
        let cs = c_dump(|o| unsafe { cref_igf_dump_static(self.cs, self.cd, o) });
        let rs = dump_static(&self.rs);
        assert!(first_diff(&rs, &cs).is_none(), "static {:?} {ctx}", first_diff(&rs, &cs));
        let cd = c_dump(|o| unsafe { cref_igf_dump_data(self.cd, o) });
        let rd = dump_data(&self.rd);
        assert!(first_diff(&rd, &cd).is_none(), "data {:?} {ctx}", first_diff(&rd, &cd));
        let cc = c_dump(|o| unsafe { cref_igf_dump_common(self.cc, o) });
        let rc = dump_common(&self.rc);
        assert!(first_diff(&rc, &cc).is_none(), "common {:?} {ctx}", first_diff(&rc, &cc));
        assert!(
            first_diff(&self.rvs, self.c_virtual_spec()).is_none(),
            "virtualSpec {:?} {ctx}",
            first_diff(&self.rvs, self.c_virtual_spec())
        );
    }

    fn num_tiles(&self, ws: i32) -> i32 {
        let n = iis_igf_dec_lib_get_number_of_tiles(&self.rs, ws);
        assert_eq!(n, unsafe { cref_igf_num_tiles(self.cs, ws) });
        n
    }
}

impl Drop for Chan {
    fn drop(&mut self) {
        unsafe {
            cref_igf_free(self.cs);
            cref_igf_free(self.cd);
            cref_igf_common_free(self.cc);
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Cfg {
    sr_idx: i32,
    sample_rate: u32,
    start: u8,
    stop: u8,
    high_res: u8,
    whitening: u8,
    after_tns: u8,
    indep_tiling: u8,
}

/// The nominal sampling rate of a table entry or a random rate mapped to it.
fn random_rate(g: &mut Rng, sr_idx: i32) -> u32 {
    if sr_idx >= 12 || g.range(0, 1) == 0 {
        SAMPLE_RATES[sr_idx as usize]
    } else {
        let hi = SR_BORDERS[sr_idx as usize] - 1;
        let lo = SR_BORDERS[sr_idx as usize + 1];
        g.range(lo as i32, hi as i32) as u32
    }
}

fn random_cfg(g: &mut Rng) -> Cfg {
    let sr_idx = g.range(0, 12);
    let sample_rate = random_rate(g, sr_idx);
    Cfg {
        sr_idx,
        sample_rate,
        start: g.range(0, 15) as u8,
        stop: g.range(0, 15) as u8,
        high_res: g.range(0, 1) as u8,
        whitening: g.range(0, 1) as u8,
        after_tns: g.range(0, 1) as u8,
        indep_tiling: g.range(0, 1) as u8,
    }
}

// ---------------------------------------------------------------------------------------------
// Bitstreams
// ---------------------------------------------------------------------------------------------

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

struct BsPair {
    _buf_c: Vec<u8>,
    _buf_r: Vec<u8>,
    hc: *mut (),
    r: FdkBitStream,
}

impl BsPair {
    fn new(g: &mut Rng) -> Self {
        let size = 256u32;
        let mut buf_c = random_bytes(g, size as usize);
        let buf_r = buf_c.clone();
        let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, size * 8, 0) };
        let mut r = FdkBitStream::default();
        r.init(&BitMem::from_slice(&buf_r), size, size * 8, FdkBsCfg::BsReader);
        BsPair { _buf_c: buf_c, _buf_r: buf_r, hc, r }
    }

    fn check(&self, ctx: &str) {
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
        assert_eq!(rs, st, "bitstream {ctx}");
    }
}

impl Drop for BsPair {
    fn drop(&mut self) {
        unsafe { cref_bs_free(self.hc) };
    }
}

/// Reads the IGF side info of one channel (`CChannelElement_Read`: ReadSCF + ReadIGF).
fn read_channel(
    ch: &mut Chan,
    bs: &mut BsPair,
    indep: i32,
    short: bool,
    groups: i32,
    use_enf: i32,
    frame_type: i32,
    ctx: &str,
) {
    let ws = short as i32;
    unsafe { cref_igf_read_scf(ch.cs, ch.cd, bs.hc, indep, ws, groups, frame_type) };
    iis_igf_dec_lib_read_scf(&mut ch.rs, &mut ch.rd, &mut bs.r, indep, ws, groups, frame_type);
    bs.check(&format!("read_scf {ctx}"));
    ch.check(&format!("read_scf {ctx}"));
    let rc = unsafe { cref_igf_read_igf(ch.cs, ch.cd, bs.hc, ws, indep, use_enf, frame_type) };
    let rr =
        iis_igf_dec_lib_read_igf(&mut ch.rs, &mut ch.rd, &mut bs.r, ws, indep, use_enf, frame_type);
    assert_eq!(rr, rc, "read_igf {ctx}");
    bs.check(&format!("read_igf {ctx}"));
    ch.check(&format!("read_igf {ctx}"));
}

// ---------------------------------------------------------------------------------------------
// Spectra
// ---------------------------------------------------------------------------------------------

/// Random spectrum of 1024 lines with varying structure; `igf_range` gives the lines where most
/// lines are zero (to be filled by IGF).
fn random_spectrum(g: &mut Rng, igf_range: (usize, usize), short: bool) -> Vec<i32> {
    let kind = g.range(0, 9);
    let mut v = vec![0i32; 1024];
    let win_len = if short { 128 } else { 1024 };
    let mut shift = g.range(0, 31);
    for (i, x) in v.iter_mut().enumerate() {
        if i % 8 == 0 && g.range(0, 3) == 0 {
            shift = g.range(if kind == 8 { 0 } else { 2 }, 31);
        }
        let l = i % win_len;
        let in_igf = l >= igf_range.0 && l < igf_range.1;
        *x = match kind {
            0 => 0,
            1 => g.i32(),
            2 if in_igf => 0,
            3 if in_igf => g.range(-20, 20),
            _ => {
                let p_zero = if in_igf { 6 } else { 2 };
                if g.range(0, 9) < p_zero {
                    0
                } else {
                    (g.next_u32() as i32) >> shift
                }
            }
        };
    }
    v
}

fn random_group_lengths(g: &mut Rng, short: bool) -> Vec<u8> {
    if !short {
        return vec![1];
    }
    let mut v = Vec::new();
    let mut left = 8;
    while left > 0 {
        let l = if g.range(0, 2) == 0 { g.range(1, left) } else { 1.max(left.min(g.range(1, 3))) };
        v.push(l as u8);
        left -= l;
    }
    v
}

fn random_scale(g: &mut Rng) -> i16 {
    match g.range(0, 9) {
        0 => g.range(-40, 80) as i16,
        1 => g.i16(),
        _ => g.range(0, 30) as i16,
    }
}

/// Emulates the INF path of the core decoder: `iisIGFDecLibInjectSourceSpectrumNew` followed by
/// noise filling of the tiles and setting of the window scales of the tiles.
fn inject_inf(g: &mut Rng, ch: &mut Chan, spec: &[i32], short: bool, frame_type: i32, ctx: &str) {
    let ws = short as i32;
    let mut exp: Vec<i16> = (0..128).map(|_| g.range(-5, 40) as i16).collect();
    let mut sc = spec.to_vec();
    unsafe { cref_igf_inject_new(ch.cs, ch.cd, sc.as_mut_ptr(), exp.as_mut_ptr(), ws, frame_type) };
    iis_igf_dec_lib_inject_source_spectrum_new(&mut ch.rs, &mut ch.rd, spec, &exp, ws, frame_type);
    ch.check(&format!("inject {ctx}"));

    // Noise filling of the tiles (random modification of the zero lines).
    if g.range(0, 2) != 0 {
        let noise = (g.next_u32() as i32) >> g.range(4, 31);
        for v in ch.rd.spectrum_tab_array.iter_mut() {
            if *v == 0 && g.range(0, 1) == 0 {
                *v = if g.range(0, 1) == 0 { noise } else { -noise };
            }
        }
        unsafe { cref_igf_set_tile_spectrum(ch.cd, ch.rd.spectrum_tab_array.as_ptr()) };
    }
    let n = ch.num_tiles(ws);
    let mut scale = [[0i16; 8]; 4];
    for (t, s) in scale.iter_mut().enumerate() {
        for v in s.iter_mut() {
            *v = random_scale(g);
        }
        if (t as i32) < n {
            *iis_igf_dec_lib_access_source_spectrum_scale(&mut ch.rs, t as i32, ws) = *s;
        }
    }
    // The C shim sets all tiles at once; mirror the other tiles in Rust.
    ch.rs.f_spectrum_scale = scale;
    unsafe { cref_igf_set_spectrum_scale(ch.cs, scale.as_ptr() as *const i16) };
}

/// Random content of the virtual spectrum (it lives in the time signal buffer of the decoder).
fn randomize_virtual_spec(g: &mut Rng, ch: &mut Chan) {
    if g.range(0, 3) == 0 {
        for v in ch.rvs.iter_mut() {
            *v = g.i32();
        }
        let rvs = ch.rvs.clone();
        ch.c_virtual_spec_mut().copy_from_slice(&rvs);
    }
}

fn igf_range(ch: &Chan, short: bool) -> (usize, usize) {
    let gi = &ch.rs.s_grid_info_tab[short as usize];
    (gi.i_igf_start_line.max(0) as usize, gi.i_igf_stop_line.max(0) as usize)
}

// ---------------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------------

/// Initialization for all sampling rates and all IGF start/stop indices.
#[test]
fn init_all_configurations() {
    let mut g = Rng::new(1);
    let mut n_tiles_hist = [0u32; 5];
    for sr_idx in 0..13 {
        for start in 0..16u8 {
            for stop in 0..16u8 {
                for high_res in 0..2u8 {
                    let mut cfg = random_cfg(&mut g);
                    cfg.sr_idx = sr_idx;
                    cfg.sample_rate = random_rate(&mut g, sr_idx);
                    cfg.start = start;
                    cfg.stop = stop;
                    cfg.high_res = high_res;
                    let mut ch = Chan::new();
                    ch.init(&cfg);
                    let ctx = format!("{cfg:?}");
                    ch.check(&ctx);
                    for ws in 0..2 {
                        let n = ch.num_tiles(ws);
                        n_tiles_hist[n as usize] += 1;
                        for t in 0..5 {
                            for win in 0..if ws == 1 { 8 } else { 1 } {
                                let c =
                                    unsafe { cref_igf_access_spectrum(ch.cs, ch.cd, t, ws, win) };
                                let base = ch.rd.spectrum_tab_array.as_ptr() as usize;
                                let r = iis_igf_dec_lib_access_source_spectrum(
                                    &ch.rs, &mut ch.rd, t, ws, win,
                                )
                                .map(|s| ((s.as_ptr() as usize - base) / 4) as i32)
                                .unwrap_or(-1);
                                assert_eq!(r, c, "access t {t} ws {ws} win {win} {ctx}");
                            }
                            if t < n {
                                let c = unsafe { cref_igf_access_exponent(ch.cs, t, ws) };
                                let base = ch.rs.f_spectrum_tab_sfb_exp.as_ptr() as usize;
                                let r = iis_igf_dec_lib_access_source_spectrum_exponent(
                                    &mut ch.rs, t, ws,
                                )
                                .as_ptr() as usize;
                                assert_eq!(((r - base) / 2) as i32, c, "{ctx}");
                                let c = unsafe { cref_igf_access_scale(ch.cs, ch.cd, t, ws) };
                                let base = ch.rs.f_spectrum_scale.as_ptr() as usize;
                                let r =
                                    iis_igf_dec_lib_access_source_spectrum_scale(&mut ch.rs, t, ws)
                                        .as_ptr() as usize;
                                assert_eq!(((r - base) / 2) as i32, c, "{ctx}");
                            }
                        }
                    }
                }
            }
        }
    }
    // Configurations without tiles (IGF start below the minimum source line) and with 1..4 tiles
    // must be covered.
    assert!(n_tiles_hist.iter().all(|&n| n > 0), "{n_tiles_hist:?}");
}

/// Frame parameters shared by the channels of an element.
struct Frame {
    short: bool,
    groups: Vec<u8>,
    indep: i32,
    frame_type: i32,
    inf: u8,
}

fn random_frame(g: &mut Rng, frame: i32, prev_short: bool) -> Frame {
    let short = if g.range(0, 4) == 0 { !prev_short } else { prev_short };
    Frame {
        short,
        groups: random_group_lengths(g, short),
        indep: (frame == 0 || g.range(0, 5) == 0) as i32,
        frame_type: if g.range(0, 15) == 0 { 1 } else { 0 },
        inf: g.range(0, 1) as u8,
    }
}

/// Coverage counters of the randomized tests.
#[derive(Default)]
struct Cov(std::collections::BTreeMap<&'static str, u32>);

impl Cov {
    fn hit(&mut self, name: &'static str, cond: bool) {
        *self.0.entry(name).or_default() += cond as u32;
    }

    fn assert_min(&self, min: u32) {
        eprintln!("coverage: {:?}", self.0);
        for (k, &v) in &self.0 {
            assert!(v >= min, "coverage of {k}: {v} < {min} ({:?})", self.0);
        }
    }
}

/// Records which code paths the processing of a channel exercised.
fn cover_apply(cov: &mut Cov, ch: &Chan, f: &Frame, before: &[i32], after: &[i32], inf: u8) {
    let ws = f.short as usize;
    let gi = &ch.rs.s_grid_info_tab[ws];
    let bd = &ch.rd.bitstream_data[f.frame_type as usize];
    let active = bd.igf_all_zero == 0 && gi.i_igf_num_tile > 0;
    let tiles = &gi.s_igf_map_info_tab[..gi.i_igf_num_tile as usize];
    cov.hit(if f.short { "active short" } else { "active long" }, active);
    cov.hit("all zero", bd.igf_all_zero != 0);
    cov.hit("no tiles", gi.i_igf_num_tile == 0);
    cov.hit("4 tiles", active && gi.i_igf_num_tile == 4);
    cov.hit("spectrum changed", active && before != after);
    cov.hit("inf", active && inf != 0);
    cov.hit("sfb split", active && tiles.iter().any(|m| m.i_sfb_split != 0));
    cov.hit(
        "tile idx != 3",
        active && gi.i_igf_tile_idx_tab[..tiles.len()].iter().any(|&t| t != 3),
    );
    let wh = active && !f.short && ch.rs.igf_use_whitening != 0;
    for (name, level) in [
        ("whitening mid", WHITENING_MID),
        ("whitening off", WHITENING_OFF),
        ("whitening strong", WHITENING_STRONG),
    ] {
        cov.hit(name, wh && tiles.iter().any(|m| m.i_whitening_level == level));
    }
    cov.hit(
        "group len 2/4/8",
        active && f.short && f.groups.iter().any(|&l| l == 2 || l == 4 || l == 8),
    );
    cov.hit(
        "group len 3/5/6/7",
        active && f.short && f.groups.iter().any(|&l| [3, 5, 6, 7].contains(&l)),
    );
    cov.hit("enf flat", active && !f.short && bd.igf_use_enf_flat != 0);
}

/// Mono: parsing and processing of many frames with persistent state (`ApplyTools`).
fn run_mono(seed: u64, cov: &mut Cov, cfg: Option<Cfg>, n_frames: Option<i32>) {
    let mut g = Rng::new(seed);
    let cfg = cfg.unwrap_or_else(|| random_cfg(&mut g));
    let mut ch = Chan::new();
    ch.init(&cfg);
    let use_enf = g.range(0, 3) != 0;
    let mut seed_c: u32 = g.next_u32();
    let mut seed_r = seed_c;
    let mut short = false;
    for frame in 0..n_frames.unwrap_or_else(|| g.range(5, 30)) {
        let f = random_frame(&mut g, frame, short);
        short = f.short;
        let inf = if use_enf { f.inf } else { 0 };
        let ws = short as i32;
        let ctx = format!("seed {seed} frame {frame} short {short} inf {inf} {cfg:?}");

        let mut bs = BsPair::new(&mut g);
        let ng = f.groups.len() as i32;
        read_channel(&mut ch, &mut bs, f.indep, short, ng, use_enf as i32, f.frame_type, &ctx);
        let mut spec = random_spectrum(&mut g, igf_range(&ch, short), short);
        let spec_in = spec.clone();
        let mut scale: Vec<i16> = (0..8).map(|_| random_scale(&mut g)).collect();
        if inf != 0 {
            inject_inf(&mut g, &mut ch, &spec, short, f.frame_type, &ctx);
        }
        randomize_virtual_spec(&mut g, &mut ch);

        let mut spec_c = spec.clone();
        let mut scale_c = scale.clone();
        let mut mask_c = vec![0u8; 1024];
        let mut mask_r = vec![0u8; 1024];
        let nspec = if short { 8 } else { 1 };
        unsafe {
            cref_igf_apply(
                ch.cs,
                ch.cd,
                spec_c.as_mut_ptr(),
                scale_c.as_mut_ptr(),
                ws,
                ng,
                nspec,
                f.groups.as_ptr(),
                &mut seed_c,
                mask_c.as_mut_ptr(),
                inf,
                f.frame_type,
            )
        };
        cigf_apply(
            &mut ch.rs,
            &mut ch.rd,
            &mut ch.rc,
            &mut ch.rvs,
            &mut spec,
            &mut scale,
            ws,
            ng,
            nspec,
            &f.groups,
            &mut seed_r,
            &mut mask_r,
            inf,
            f.frame_type,
        );
        assert_eq!(seed_r, seed_c, "seed {ctx}");
        assert!(
            first_diff(&spec, &spec_c).is_none(),
            "spec {:?} {ctx}",
            first_diff(&spec, &spec_c)
        );
        assert_eq!(scale, scale_c, "scale {ctx}");
        assert_eq!(mask_r, mask_c, "mask {ctx}");
        ch.check(&format!("apply {ctx}"));
        cover_apply(cov, &ch, &f, &spec_in, &spec, inf);

        if !short {
            let spec_in = spec.clone();
            unsafe {
                cref_igf_tnf_apply(
                    ch.cs,
                    ch.cd,
                    spec_c.as_mut_ptr(),
                    scale_c.as_mut_ptr(),
                    mask_c.as_ptr(),
                    0,
                    f.frame_type,
                )
            };
            cigf_tnf_apply(
                &ch.rs,
                &ch.rd,
                &mut ch.rvs,
                &mut spec,
                &scale,
                &mask_r,
                0,
                f.frame_type,
            );
            assert!(
                first_diff(&spec, &spec_c).is_none(),
                "tnf {:?} {ctx}",
                first_diff(&spec, &spec_c)
            );
            ch.check(&format!("tnf {ctx}"));
            cov.hit("tnf changed", spec_in != spec);
        }
    }
}

#[test]
fn mono_frames() {
    let mut cov = Cov::default();
    for seed in 0..3000 {
        run_mono(seed, &mut cov, None, None);
    }
    cov.assert_min(50);
}

/// Stereo: two channels with common window (`CChannelElement_Decode`: sync, INF, CIgf_apply_stereo
/// and TNF with the static data of the left channel).
fn run_stereo(seed: u64, cov: &mut Cov, cfg: Option<Cfg>, n_frames: Option<i32>) {
    let mut g = Rng::new(seed);
    let cfg = cfg.unwrap_or_else(|| random_cfg(&mut g));
    let mut l = Chan::new();
    let mut r = Chan::new();
    l.init(&cfg);
    r.init(&cfg);
    let use_enf = g.range(0, 3) != 0;
    let mut seeds_c: [u32; 2] = [g.next_u32(), g.next_u32()];
    let mut seeds_r = seeds_c;
    let mut short = false;
    for frame in 0..n_frames.unwrap_or_else(|| g.range(5, 25)) {
        let f = random_frame(&mut g, frame, short);
        short = f.short;
        let inf = if use_enf { f.inf } else { 0 };
        let ws = short as i32;
        let ctx = format!("seed {seed} frame {frame} short {short} inf {inf} {cfg:?}");
        let ng = f.groups.len() as i32;
        // CIgf_apply_stereo injects with IGF_FRAME_DIVISION_AAC_OR_TCX_LONG.
        let ft = 0;

        let mut bs = BsPair::new(&mut g);
        read_channel(&mut l, &mut bs, f.indep, short, ng, use_enf as i32, ft, &ctx);
        read_channel(&mut r, &mut bs, f.indep, short, ng, use_enf as i32, ft, &ctx);

        unsafe { cref_igf_sync(l.cd, r.cd) };
        iis_igf_sync_data(&mut l.rd, &mut r.rd);
        l.check(&format!("sync {ctx}"));
        r.check(&format!("sync {ctx}"));
        let all_zero = [&l, &r].map(|c| c.rd.bitstream_data[0].igf_all_zero);
        cov.hit("sync", all_zero[0] != all_zero[1]);

        let mut spec_l = random_spectrum(&mut g, igf_range(&l, short), short);
        let mut spec_r = random_spectrum(&mut g, igf_range(&r, short), short);
        let mut scale_l: Vec<i16> = (0..8).map(|_| random_scale(&mut g)).collect();
        let mut scale_r: Vec<i16> = (0..8).map(|_| random_scale(&mut g)).collect();
        if inf != 0 {
            inject_inf(&mut g, &mut l, &spec_l, short, ft, &ctx);
            inject_inf(&mut g, &mut r, &spec_r, short, ft, &ctx);
        }
        randomize_virtual_spec(&mut g, &mut l);
        randomize_virtual_spec(&mut g, &mut r);
        let ms_used: Vec<u8> = match g.range(0, 3) {
            0 => vec![0; 64],
            1 => vec![0xFF; 64],
            _ => (0..64).map(|_| g.next_u32() as u8).collect(),
        };

        let (spec_l_in, spec_r_in) = (spec_l.clone(), spec_r.clone());
        let mut spec_lc = spec_l.clone();
        let mut spec_rc = spec_r.clone();
        let mut scale_lc = scale_l.clone();
        let mut scale_rc = scale_r.clone();
        let mut mask_lc = vec![0u8; 1024];
        let mut mask_rc = vec![0u8; 1024];
        let mut mask_lr = vec![0u8; 1024];
        let mut mask_rr = vec![0u8; 1024];
        let nspec = if short { 8 } else { 1 };
        unsafe {
            cref_igf_apply_stereo(
                l.cs,
                r.cs,
                l.cd,
                r.cd,
                spec_lc.as_mut_ptr(),
                spec_rc.as_mut_ptr(),
                scale_lc.as_mut_ptr(),
                scale_rc.as_mut_ptr(),
                ws,
                ng,
                nspec,
                f.groups.as_ptr(),
                &mut seeds_c[0],
                &mut seeds_c[1],
                ms_used.as_ptr(),
                mask_lc.as_mut_ptr(),
                mask_rc.as_mut_ptr(),
                inf,
                ft,
            )
        };
        {
            let (sl, sr) = seeds_r.split_at_mut(1);
            cigf_apply_stereo(
                &mut l.rs,
                &mut r.rs,
                &mut l.rd,
                &mut r.rd,
                &mut l.rc,
                &mut r.rc,
                &mut l.rvs,
                &mut r.rvs,
                &mut spec_l,
                &mut spec_r,
                &mut scale_l,
                &mut scale_r,
                ws,
                ng,
                nspec,
                &f.groups,
                &mut sl[0],
                &mut sr[0],
                &ms_used,
                &mut mask_lr,
                &mut mask_rr,
                inf,
                ft,
            );
        }
        assert_eq!(seeds_r, seeds_c, "seeds {ctx}");
        assert!(
            first_diff(&spec_l, &spec_lc).is_none(),
            "specL {:?} {ctx}",
            first_diff(&spec_l, &spec_lc)
        );
        assert!(
            first_diff(&spec_r, &spec_rc).is_none(),
            "specR {:?} {ctx}",
            first_diff(&spec_r, &spec_rc)
        );
        assert_eq!(scale_l, scale_lc, "scaleL {ctx}");
        assert_eq!(scale_r, scale_rc, "scaleR {ctx}");
        assert_eq!(mask_lr, mask_lc, "maskL {ctx}");
        assert_eq!(mask_rr, mask_rc, "maskR {ctx}");
        l.check(&format!("apply L {ctx}"));
        r.check(&format!("apply R {ctx}"));
        cover_apply(cov, &l, &f, &spec_l_in, &spec_l, inf);
        cover_apply(cov, &r, &f, &spec_r_in, &spec_r, inf);
        cov.hit("ms", ms_used.iter().any(|&m| m != 0) && l.rd.bitstream_data[0].igf_all_zero == 0);

        if !short {
            // CIgf_TNF_apply is called with the static data of the left channel for both channels.
            let (tnf_l_in, tnf_r_in) = (spec_l.clone(), spec_r.clone());
            unsafe {
                cref_igf_tnf_apply(
                    l.cs,
                    l.cd,
                    spec_lc.as_mut_ptr(),
                    scale_lc.as_mut_ptr(),
                    mask_lc.as_ptr(),
                    0,
                    0,
                );
                cref_igf_tnf_apply(
                    l.cs,
                    r.cd,
                    spec_rc.as_mut_ptr(),
                    scale_rc.as_mut_ptr(),
                    mask_rc.as_ptr(),
                    0,
                    0,
                );
            }
            cigf_tnf_apply(&l.rs, &l.rd, &mut l.rvs, &mut spec_l, &scale_l, &mask_lr, 0, 0);
            cigf_tnf_apply(&l.rs, &r.rd, &mut r.rvs, &mut spec_r, &scale_r, &mask_rr, 0, 0);
            cov.hit("tnf changed", spec_l != tnf_l_in || spec_r != tnf_r_in);
            assert!(first_diff(&spec_l, &spec_lc).is_none(), "tnf L {ctx}");
            assert!(first_diff(&spec_r, &spec_rc).is_none(), "tnf R {ctx}");
            l.check(&format!("tnf L {ctx}"));
            r.check(&format!("tnf R {ctx}"));
        }
    }
}

#[test]
fn stereo_frames() {
    let mut cov = Cov::default();
    for seed in 0..2000 {
        run_stereo(seed, &mut cov, None, None);
    }
    cov.assert_min(50);
}

/// Processing of a few mono and stereo frames for every configuration (sampling rate, IGF
/// start/stop index, high resolution).
#[test]
fn all_configurations_frames() {
    let mut g = Rng::new(3);
    let mut cov = Cov::default();
    let mut seed = 100_000;
    for sr_idx in 0..13 {
        for start in 0..16u8 {
            for stop in 0..16u8 {
                for high_res in 0..2u8 {
                    let mut cfg = random_cfg(&mut g);
                    cfg.sr_idx = sr_idx;
                    cfg.sample_rate = random_rate(&mut g, sr_idx);
                    cfg.start = start;
                    cfg.stop = stop;
                    cfg.high_res = high_res;
                    cfg.whitening = 1;
                    seed += 1;
                    run_mono(seed, &mut cov, Some(cfg), Some(4));
                    run_stereo(seed, &mut cov, Some(cfg), Some(4));
                }
            }
        }
    }
    cov.assert_min(50);
}

/// `Autocorrelation()` with random data.
#[test]
fn autocorrelation_random() {
    let mut g = Rng::new(7);
    for iter in 0..20000 {
        let n = g.range(0, 400);
        let lag = g.range(0, 10);
        let shift = g.range(0, 31);
        let x: Vec<i32> = (0..(n.max(lag) + 16)).map(|_| g.i32() >> shift).collect();
        let e0 = g.range(-40, 40);
        let (mut ec, mut er) = (e0, e0);
        let mut xc = x.clone();
        let c = unsafe { cref_igf_autocorrelation(xc.as_mut_ptr(), &mut ec, n, lag) };
        let r = autocorrelation(&x, &mut er, n, lag);
        assert_eq!((r, er), (c, ec), "iter {iter} n {n} lag {lag}");
    }
}
