// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of the libMpeghDec channel element functions: sequences of frames of SCE,
//! CPE and LFE elements are generated from the element syntax (with random tool configurations,
//! window sequences, TNS, noise filling, M/S, complex prediction, IGF, LTPF and FDP parameters and
//! random arithmetic coded payloads), parsed with `CChannelElement_Read()`, decoded with
//! `CChannelElement_Decode()`, concealed with `CConcealment_Apply()` and transformed with
//! `CBlock_FrequencyToTime()` in C and Rust, comparing the complete state after every call.
#![cfg(not(cref_missing))]

mod core_common;

use core_common::element::*;
use core_common::*;
use mpeghdec_arith::ac_arith_coder::{carco_decode_arith_data, CArcoData};
use mpeghdec_core::aacdecoder_lib::*;
use mpeghdec_core::block::*;
use mpeghdec_core::channel::*;
use mpeghdec_core::conceal::*;
use mpeghdec_cref::core_channel::*;
use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::mdct::{BLOCK_LONG, BLOCK_SHORT, BLOCK_START, BLOCK_STOP};
use mpeghdec_fdk::tools_rom::{get_bitstream_element_list, RbdId};
use mpeghdec_igf::igf_dec::*;

// ---------------------------------------------------------------------------------------------
// Frame generator
// ---------------------------------------------------------------------------------------------

/// Per channel syntax state of the generator.
#[derive(Clone, Copy, Default)]
struct GenCh {
    ws: i32,
    shape: u32,
    max_sfb: u32,
    groups: u32,
    tns_present: bool,
}

/// Shadow decoder state used to find the length of the random payloads (arithmetic coded
/// spectral data, IGF data).
struct Shadow {
    arco: Vec<CArcoData>,
    igf_st: Vec<IgfPrivateStaticData>,
    igf_d: Vec<IgfPrivateData>,
}

pub struct Gen {
    pub g: Rng,
    codes: Vec<(u32, u32)>,
    prev_ws: [i32; 2],
    /// noise bytes (`fd_noise_level_and_offset`) and common window of the last frame
    pub last_noise: [u32; 2],
    pub last_cw: bool,
    /// probability (1/n) of an invalid value / corruption
    pub err_rate: i32,
}

impl Gen {
    pub fn new(seed: u64) -> Self {
        Gen {
            g: Rng::new(seed),
            codes: scl_codes(),
            prev_ws: [BLOCK_LONG; 2],
            last_noise: [0; 2],
            last_cw: false,
            err_rate: 40,
        }
    }

    fn err(&mut self) -> bool {
        self.err_rate > 0 && self.g.range(0, self.err_rate) == 0
    }

    fn sf_code(&mut self, w: &mut BitWriter, delta: i32) {
        let (c, l) = self.codes[(delta + 60) as usize];
        w.bits(c, l);
    }

    fn next_ws(&mut self, ch: usize, lfe: bool) -> i32 {
        if lfe {
            return if self.err() { BLOCK_SHORT } else { BLOCK_LONG };
        }
        let p = self.prev_ws[ch];

        if self.g.range(0, 5) == 0 {
            self.g.range(0, 3)
        } else {
            match p {
                BLOCK_LONG => {
                    if self.g.range(0, 3) == 0 {
                        BLOCK_START
                    } else {
                        BLOCK_LONG
                    }
                }
                BLOCK_START => {
                    if self.g.range(0, 2) == 0 {
                        BLOCK_SHORT
                    } else {
                        BLOCK_START
                    }
                }
                BLOCK_SHORT => {
                    if self.g.range(0, 2) == 0 {
                        BLOCK_STOP
                    } else {
                        BLOCK_SHORT
                    }
                }
                _ => BLOCK_LONG,
            }
        }
    }

    fn ics_info(&mut self, w: &mut BitWriter, gc: &mut GenCh, ch: usize, cfg: &Cfg, el: &El) {
        let ws = self.next_ws(ch, cfg.lfe);
        gc.ws = ws;
        gc.shape = self.g.range(0, 1) as u32;
        w.bits(ws as u32, 2);
        w.bit(gc.shape);
        let short = ws == BLOCK_SHORT;
        let total = if short {
            el.sri.number_of_scale_factor_bands_short
        } else {
            el.sri.number_of_scale_factor_bands_long
        } as i32;
        let mut max_sfb = match self.g.range(0, 5) {
            0 => 0,
            1 => total,
            _ => self.g.range(0, total),
        };
        if cfg.enf && !short && self.g.range(0, 3) != 0 {
            let igf_start = el.st[0].igf_static_data.igf_start_sfb_lb as i32;
            max_sfb = max_sfb.min(igf_start);
        }
        if self.err() {
            max_sfb = self.g.range(0, if short { 15 } else { 63 });
        }
        gc.max_sfb = max_sfb as u32;
        w.bits(max_sfb as u32, if short { 4 } else { 6 });
        gc.groups = 1;
        if short {
            let grouping = self.g.range(0, 127) as u32;
            w.bits(grouping, 7);
            gc.groups = 8 - grouping.count_ones();
        }
        self.prev_ws[ch] = ws;
    }

    fn ltp_param(&mut self, w: &mut BitWriter, lfe: bool) {
        let present = if lfe { self.err() } else { self.g.range(0, 2) == 0 };
        w.bit(present as u32);
        if present {
            w.bits(self.g.range(0, 511) as u32, 9);
            w.bits(self.g.range(0, 3) as u32, 2);
        }
    }

    fn tns_data(&mut self, w: &mut BitWriter, gc: &GenCh, total: u32) {
        let short = gc.ws == BLOCK_SHORT;
        let nwin = if short { 8 } else { 1 };
        for _ in 0..nwin {
            let n_filt = self.g.range(0, if short { 1 } else { 3 }) as u32;
            w.bits(n_filt, if short { 1 } else { 2 });
            if n_filt != 0 {
                let coef_res = self.g.range(0, 1) as u32;
                w.bit(coef_res);
                for _ in 0..n_filt {
                    let length =
                        self.g.range(0, if short { 15 } else { total.min(63) as i32 }) as u32;
                    w.bits(length, if short { 4 } else { 6 });
                    let order = self.g.range(0, if short { 7 } else { 15 }) as u32;
                    w.bits(order, if short { 3 } else { 4 });
                    if order != 0 {
                        w.bit(self.g.range(0, 1) as u32);
                        let coef_compress = self.g.range(0, 1) as u32;
                        w.bit(coef_compress);
                        for _ in 0..order {
                            w.bits(self.g.next_u32(), coef_res + 3 - coef_compress);
                        }
                    }
                }
            }
        }
    }

    fn scale_factors(&mut self, w: &mut BitWriter, gc: &GenCh, global_gain: i32) {
        let mut factor = global_gain;
        for group in 0..gc.groups {
            for band in 0..gc.max_sfb {
                if group == 0 && band == 0 {
                    continue;
                }
                let mut delta = match self.g.range(0, 7) {
                    0..=2 => 0,
                    3..=5 => self.g.range(-4, 4),
                    _ => self.g.range(-20, 20),
                };
                if factor + delta < 20 || factor + delta > 230 {
                    delta = -delta;
                }
                if self.err() {
                    delta = self.g.range(-60, 60);
                }
                factor += delta;
                self.sf_code(w, delta);
            }
        }
    }

    /// Random arithmetic coded spectral data; the shadow decoder determines its length.
    fn ac_spectral_data(
        &mut self,
        w: &mut BitWriter,
        gc: &GenCh,
        ch: usize,
        flags: u32,
        el: &El,
        sh: &mut Shadow,
    ) {
        let reset = if flags & AC_INDEP != 0 {
            1
        } else {
            let r = (self.g.range(0, 8) == 0) as u32;
            w.bit(r);
            r
        };
        let short = gc.ws == BLOCK_SHORT;
        let offs =
            if short { el.sri.scale_factor_bands_short } else { el.sri.scale_factor_bands_long };
        let max_sfb = gc.max_sfb.min(if short {
            el.sri.number_of_scale_factor_bands_short
        } else {
            el.sri.number_of_scale_factor_bands_long
        } as u32);
        let lg = offs[max_sfb as usize] as i32;
        let (num_win, win_len) = if short { (8, 128) } else { (1, 1024) };
        let start = w.pos;
        w.random(&mut self.g, 12000);
        let (_b, mut bs) = w.reader_at(start);
        let v0 = bs.get_valid_bits();
        let mut spec = vec![0i32; 1024];
        for win in 0..num_win {
            let e = carco_decode_arith_data(
                &mut sh.arco[ch],
                &mut bs,
                &mut spec[win * 128..],
                lg,
                win_len,
                (reset != 0 && win == 0) as i32,
            );
            if e != 0 {
                break;
            }
        }
        let used = (v0 - bs.get_valid_bits()).max(0) as usize;
        w.pos = start + used;
    }

    /// Random IGF data; the shadow IGF decoder determines its length.
    fn igf_data(
        &mut self,
        w: &mut BitWriter,
        gc: &GenCh,
        ch: usize,
        flags: u32,
        el_flags: u32,
        sh: &mut Shadow,
    ) {
        let start = w.pos;
        w.random(&mut self.g, 3000);
        if self.g.range(0, 3) == 0 {
            /* igf_allZero */
            w.pos = start;
            w.bit(1);
            w.random(&mut self.g, 3000);
        }
        let (_b, mut bs) = w.reader_at(start);
        let v0 = bs.get_valid_bits();
        let short = (gc.ws == BLOCK_SHORT) as i32;
        let groups = if short != 0 { gc.groups as i32 } else { 1 };
        let indep = (flags & AC_INDEP != 0) as i32;
        iis_igf_dec_lib_read_scf(
            &mut sh.igf_st[ch],
            &mut sh.igf_d[ch],
            &mut bs,
            indep,
            short,
            groups,
            0,
        );
        iis_igf_dec_lib_read_igf(
            &mut sh.igf_st[ch],
            &mut sh.igf_d[ch],
            &mut bs,
            short,
            indep,
            (el_flags & AC_EL_IGF_USE_ENF != 0) as i32,
            0,
        );
        let used = (v0 - bs.get_valid_bits()).max(0) as usize;
        w.pos = start + used;
    }

    fn ms_data(&mut self, w: &mut BitWriter, gcs: &[GenCh; 2], cfg: &Cfg, flags: u32, el: &El) {
        let gc = &gcs[0];
        let short = gc.ws == BLOCK_SHORT;
        let groups = gc.groups;
        let igf = cfg.enf && cfg.indep_tiling == 0;
        let s = &el.st[0].igf_static_data;
        let (igf_start, igf_stop) = if short {
            (s.igf_start_sfb_sb as u32, s.igf_stop_sfb_sb as u32)
        } else {
            (s.igf_start_sfb_lb as u32, s.igf_stop_sfb_lb as u32)
        };
        let mut max_sfb_ste = gcs[0].max_sfb.max(gcs[1].max_sfb);
        if igf {
            max_sfb_ste = max_sfb_ste.min(igf_start);
        }
        let total = if short {
            el.sri.number_of_scale_factor_bands_short
        } else {
            el.sri.number_of_scale_factor_bands_long
        } as u32;
        let msp = self.g.range(0, 3) as u32;
        w.bits(msp, 2);
        let mut ms_used = [0u8; 64];
        match msp {
            1 => {
                for group in 0..groups {
                    for band in 0..max_sfb_ste {
                        let b = self.g.range(0, 1) as u32;
                        w.bit(b);
                        ms_used[band as usize] |= (b << group) as u8;
                    }
                }
            }
            3 => {
                let all = self.g.range(0, 2) == 0;
                w.bit(all as u32);
                if all {
                    for group in 0..groups {
                        for band in 0..max_sfb_ste {
                            ms_used[band as usize] |= 1 << group;
                        }
                    }
                } else {
                    for group in 0..groups {
                        let mut band = 0;
                        while band < max_sfb_ste {
                            let b = self.g.range(0, 1) as u32;
                            w.bit(b);
                            ms_used[band as usize] |= (b << group) as u8;
                            if band + 1 < total {
                                ms_used[band as usize + 1] |= ms_used[band as usize] & (1 << group);
                            }
                            band += 2;
                        }
                    }
                }
                w.bit(self.g.range(0, 1) as u32); /* pred_dir */
                let complex = self.g.range(0, 1) as u32;
                w.bit(complex);
                if complex != 0 && flags & AC_INDEP == 0 {
                    w.bit(self.g.range(0, 1) as u32); /* use_prev_frame */
                }
                if flags & AC_INDEP == 0 {
                    w.bit(self.g.range(0, 1) as u32); /* delta_code_time */
                }
                for group in 0..groups {
                    let mut band = 0;
                    while band < max_sfb_ste {
                        if ms_used[band as usize] & (1 << group) != 0 {
                            let d = self.g.range(-4, 4);
                            self.sf_code(w, d);
                            if complex != 0 {
                                let d = self.g.range(-4, 4);
                                self.sf_code(w, d);
                            }
                        }
                        band += 2;
                    }
                }
            }
            _ => {}
        }
        if igf {
            let imsp = self.g.range(0, 3) as u32;
            w.bits(imsp, 2);
            let mut ms_used = [0u8; 64];
            match imsp {
                1 => {
                    for group in 0..groups {
                        let mut sfb = igf_start;
                        while sfb < igf_stop {
                            let b = self.g.range(0, 1) as u32;
                            w.bit(b);
                            ms_used[sfb as usize] |= (b << group) as u8;
                            sfb += 2 - cfg.high_res as u32;
                        }
                    }
                }
                3 => {
                    let all = self.g.range(0, 2) == 0;
                    w.bit(all as u32);
                    if all {
                        for group in 0..groups {
                            for sfb in igf_start..igf_stop {
                                ms_used[sfb as usize] |= 1 << group;
                            }
                        }
                    } else {
                        for group in 0..groups {
                            let mut sfb = igf_start;
                            while sfb < igf_stop {
                                let b = self.g.range(0, 1) as u32;
                                w.bit(b);
                                ms_used[sfb as usize] |= (b << group) as u8;
                                if sfb + 1 < igf_stop {
                                    ms_used[sfb as usize + 1] = ms_used[sfb as usize];
                                }
                                sfb += 2;
                            }
                        }
                    }
                    w.bit(self.g.range(0, 1) as u32); /* igf_pred_dir */
                    if flags & AC_INDEP == 0 {
                        w.bit(self.g.range(0, 1) as u32); /* igf_delta_code_time */
                    }
                    for group in 0..groups {
                        let mut sfb = igf_start;
                        while sfb < igf_stop {
                            if ms_used[sfb as usize] & (1 << group) != 0 {
                                let d = self.g.range(-4, 4);
                                self.sf_code(w, d);
                            }
                            sfb += 2;
                        }
                    }
                }
                _ => {}
            }
        }
    }

    /// Generates one frame of the element by walking the element list like
    /// `CChannelElement_Read()`.
    pub fn frame(&mut self, el: &El, flags: u32) -> Vec<u8> {
        let cfg = el.cfg;
        let el_flags = cfg.el_flags();
        let mut sh = Shadow {
            arco: el.st.iter().map(|s| (**s.h_ar_co.as_ref().unwrap()).clone()).collect(),
            igf_st: el.st.iter().map(|s| s.igf_static_data.clone()).collect(),
            igf_d: el.ch.iter().map(|c| c.igf_data.clone()).collect(),
        };
        let mut w = BitWriter::new();
        let mut list =
            get_bitstream_element_list(AOT_MPEGH3DA, 0, cfg.nch as u8, 0, el_flags).unwrap();
        let mut gcs = [GenCh::default(); 2];
        let mut i = 0i32;
        let mut ch = 0usize;
        let mut decision_bit = 0u32;
        let mut common_window = false;
        let mut tns_active = false;
        let mut common_ltpf = false;
        let mut global_gain = [0i32; 2];
        loop {
            match list.id[i as usize] {
                RbdId::enhancedNoiseFilling => {
                    if el_flags & AC_EL_ENHANCED_NOISE != 0 {
                        let gc = gcs[ch];
                        self.igf_data(&mut w, &gc, ch, flags, el_flags, &mut sh);
                    }
                }
                RbdId::common_window => {
                    common_window = self.g.range(0, 3) != 0;
                    self.last_cw = common_window;
                    w.bit(common_window as u32);
                    decision_bit = common_window as u32;
                }
                RbdId::ics_info => {
                    let mut gc = gcs[ch];
                    self.ics_info(&mut w, &mut gc, ch, &cfg, el);
                    gcs[ch] = gc;
                    if cfg.nch == 2 && common_window {
                        gcs[1] = gcs[0];
                        self.prev_ws[1] = self.prev_ws[0];
                    }
                }
                RbdId::common_max_sfb => {
                    w.bit(!self.err() as u32);
                }
                RbdId::ms => {
                    self.ms_data(&mut w, &gcs, &cfg, flags, el);
                }
                RbdId::global_gain => {
                    global_gain[ch] = self.g.range(60, 200);
                    w.bits(global_gain[ch] as u32, 8);
                }
                RbdId::scale_factor_data_usac => {
                    if !common_ltpf {
                        self.ltp_param(&mut w, cfg.lfe);
                    }
                    let gc = gcs[ch];
                    if flags & AC_INDEP == 0 && gc.ws != BLOCK_SHORT {
                        let fdp = if cfg.lfe { self.err() } else { self.g.range(0, 1) != 0 };
                        w.bit(fdp as u32);
                        if fdp {
                            w.bits(self.g.range(0, 255) as u32, 8);
                        }
                    }
                    let alias = |s: &mut Self| {
                        if cfg.lfe {
                            s.err() as u32
                        } else {
                            s.g.range(0, 1) as u32
                        }
                    };
                    if flags & AC_INDEP != 0 {
                        let a = alias(self);
                        w.bit(a);
                    }
                    let a = alias(self);
                    w.bit(a);
                    self.scale_factors(&mut w, &gc, global_gain[ch]);
                }
                RbdId::tns_data_present => {
                    let p = self.g.range(0, 2) == 0 && (!cfg.lfe || self.err());
                    gcs[ch].tns_present = p;
                    w.bit(p as u32);
                }
                RbdId::tns_data => {
                    if gcs[ch].tns_present {
                        let gc = gcs[ch];
                        let total = if gc.ws == BLOCK_SHORT {
                            el.sri.number_of_scale_factor_bands_short
                        } else {
                            el.sri.number_of_scale_factor_bands_long
                        } as u32;
                        self.tns_data(&mut w, &gc, total);
                    }
                }
                RbdId::tw_data => {}
                RbdId::common_tw => {
                    common_ltpf = self.g.range(0, 1) != 0;
                    w.bit(common_ltpf as u32);
                    if common_ltpf {
                        self.ltp_param(&mut w, cfg.lfe);
                    }
                }
                RbdId::tns_data_present_usac => {
                    if tns_active {
                        let mut common_tns = false;
                        if common_window {
                            common_tns = self.g.range(0, 1) != 0;
                            w.bit(common_tns as u32);
                        }
                        if (el_flags & (AC_EL_ENHANCED_NOISE | AC_EL_IGF_AFTER_TNS))
                            != AC_EL_ENHANCED_NOISE
                        {
                            w.bit(!self.err() as u32); /* tns_on_lr */
                        }
                        if common_tns {
                            let gc = gcs[0];
                            let total = if gc.ws == BLOCK_SHORT {
                                el.sri.number_of_scale_factor_bands_short
                            } else {
                                el.sri.number_of_scale_factor_bands_long
                            } as u32;
                            self.tns_data(&mut w, &gc, total);
                            gcs[0].tns_present = false;
                            gcs[1].tns_present = false;
                        } else {
                            let both = self.g.range(0, 1) != 0;
                            w.bit(both as u32);
                            if both {
                                gcs[0].tns_present = true;
                                gcs[1].tns_present = true;
                            } else {
                                let p1 = self.g.range(0, 1) != 0;
                                w.bit(p1 as u32);
                                gcs[1].tns_present = p1;
                                gcs[0].tns_present = !p1;
                            }
                        }
                    } else {
                        gcs[0].tns_present = false;
                        gcs[1].tns_present = false;
                    }
                }
                RbdId::core_mode => {
                    decision_bit = self.err() as u32;
                    w.bit(decision_bit);
                }
                RbdId::tns_active => {
                    tns_active = self.g.range(0, 2) != 0;
                    w.bit(tns_active as u32);
                }
                RbdId::noise => {
                    if el_flags & AC_EL_USAC_NOISE != 0 {
                        let v = if self.g.range(0, 3) == 0 {
                            self.g.range(1, 31) as u32
                        } else {
                            self.g.range(0, 255) as u32
                        };
                        self.last_noise[ch] = v;
                        w.bits(v, 8);
                    }
                }
                RbdId::fac_data => {
                    w.bit(self.err() as u32);
                }
                RbdId::ac_spectral_data => {
                    let gc = gcs[ch];
                    self.ac_spectral_data(&mut w, &gc, ch, flags, el, &mut sh);
                }
                RbdId::next_channel => {
                    ch = (ch + 1) % cfg.nch;
                }
                RbdId::link_sequence => {
                    match list.next[decision_bit as usize] {
                        Some(l) => list = l,
                        None => break,
                    }
                    i = -1;
                }
                RbdId::end_of_sequence => break,
                _ => break,
            }
            i += 1;
            if list.id[i as usize] == RbdId::end_of_sequence {
                break;
            }
        }
        /* random corruption */
        if self.err() {
            let n = self.g.range(1, 3);
            for _ in 0..n {
                let p = self.g.range(0, w.pos.max(1) as i32 - 1) as usize;
                w.buf[p >> 3] ^= 1 << (7 - (p & 7));
            }
        }
        w.buf
    }
}

// ---------------------------------------------------------------------------------------------
// Frame loop
// ---------------------------------------------------------------------------------------------

#[derive(Default, Debug)]
pub struct Cov {
    pub frames: u32,
    pub ok: u32,
    pub errors: std::collections::BTreeMap<i32, u32>,
    pub cplx: u32,
    pub ms: u32,
    pub igf_ms: u32,
    pub transform_splitting: u32,
    pub stereo_filling: u32,
    pub use_prev_frame: u32,
    pub igf_active: u32,
    pub tns: u32,
    pub fdp: u32,
    pub ltp: u32,
    pub short: u32,
    pub start_ts_cp: u32,
}

/// Runs `n_frames` frames of one element configuration through C and Rust.
pub fn run(seed: u64, cfg: Cfg, n_frames: usize, err_rate: i32, cov: &mut Cov) {
    let mut gen = Gen::new(seed);
    gen.err_rate = err_rate;
    let mut el = El::new(cfg);
    el.check("init");
    let el_flags = cfg.el_flags();
    let headroom = gen.g.range(0, 6);
    for frame in 0..n_frames {
        let what = format!("seed {seed} cfg {cfg:?} frame {frame}");
        let indep = frame == 0 || gen.g.range(0, 10) == 0;
        let flags = AC_MPEGH3DA | if indep { AC_INDEP } else { 0 };

        /* stale content of the shared buffers */
        if gen.g.range(0, 4) == 0 {
            for v in el.wb.buf.iter_mut() {
                *v = gen.g.i32() >> gen.g.range(0, 31);
            }
            el.sync_buffers_to_c();
        }

        let buf = gen.frame(&el, flags);
        let mut buf_c = buf.clone();
        let hc =
            unsafe { cref_bs_new(buf_c.as_mut_ptr(), BUF_SIZE as u32, (BUF_SIZE * 8) as u32, 0) };
        let mut bs = FdkBitStream::default();
        bs.init(
            &BitMem::from_slice(&buf),
            BUF_SIZE as u32,
            (BUF_SIZE * 8) as u32,
            FdkBsCfg::BsReader,
        );

        let err_c = unsafe {
            cref_core_el_read(el.c, hc, AOT_MPEGH3DA, flags, el_flags, 1024, cfg.nch as i32)
        };
        let err_r = {
            let (ch, st, mut mem, sri) = el.split();
            cchannel_element_read(
                &mut bs,
                ch,
                st,
                &mut mem,
                AOT_MPEGH3DA,
                sri,
                flags,
                el_flags,
                1024,
                cfg.nch as u8,
                0,
                None,
            )
        };
        assert_eq!(err_c, err_r, "{what}: CChannelElement_Read error");
        assert_eq!(unsafe { cref_bs_getValidBits(hc) }, bs.get_valid_bits(), "{what}: bits");
        unsafe { cref_bs_free(hc) };
        el.check(&format!("{what}: after read"));
        cov.frames += 1;
        *cov.errors.entry(err_r).or_default() += 1;

        let frame_ok = (err_r == AAC_DEC_OK) as i32;
        if frame_ok != 0 {
            cov.ok += 1;
            if let Some(j) = &el.com.joint_stereo_data {
                if el.ch[0].p_dyn_data.raw_data_info.common_window != 0 {
                    if j.cplx_pred_flag != 0 {
                        cov.cplx += 1;
                    } else if j.ms_mask_present != 0 {
                        cov.ms += 1;
                    }
                    if cfg.enf && j.igf_ms_mask_present != 0 {
                        cov.igf_ms += 1;
                    }
                }
            }
            if el.ch[0].transform_splitting_active != 0 {
                cov.transform_splitting += 1;
            }
            let long = el.ch[0].ics_info.window_sequence != BLOCK_SHORT;
            if !long {
                cov.short += 1;
            }
            if cfg.noise && cfg.nch == 2 && gen.last_cw && (1..32).contains(&gen.last_noise[1]) {
                if el.ch[1].ics_info.window_sequence != BLOCK_SHORT {
                    cov.stereo_filling += 1;
                }
            }
            if let (Some(j), Some(p)) = (&el.com.joint_stereo_data, &el.com.cplx_prediction_data) {
                if el.ch[0].p_dyn_data.raw_data_info.common_window != 0 && j.cplx_pred_flag != 0 {
                    if p.use_prev_frame != 0 {
                        cov.use_prev_frame += 1;
                    }
                    if el.ch[0].transform_splitting_active != 0 && p.complex_coef != 0 {
                        cov.start_ts_cp += 1;
                    }
                }
            }
            for c in 0..cfg.nch {
                if cfg.enf && el.ch[c].igf_data.bitstream_data[0].igf_all_zero == 0 {
                    cov.igf_active += 1;
                }
                if el.ch[c].p_dyn_data.tns_data.active != 0 {
                    cov.tns += 1;
                }
                if el.ch[c].fdp_spacing_index >= 0 {
                    cov.fdp += 1;
                }
                if el.st[c].ltp_param[0] != 0 {
                    cov.ltp += 1;
                }
            }
            unsafe { cref_core_el_decode(el.c, flags, el_flags, cfg.nch as i32) };
            {
                let (ch, st, mut mem, sri) = el.split();
                /* every other element decode runs the parallel variant */
                static CALLS: std::sync::atomic::AtomicUsize =
                    std::sync::atomic::AtomicUsize::new(0);
                let pool = (CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed) % 2 == 1)
                    .then(test_pool);
                cchannel_element_decode_pool(
                    ch,
                    st,
                    &mut mem,
                    sri,
                    flags,
                    el_flags,
                    cfg.nch as i32,
                    pool,
                );
            }
            el.check(&format!("{what}: after decode"));
        }

        for c in 0..cfg.nch {
            let rc = unsafe { cref_core_el_conceal_apply(el.c, c as i32, frame_ok, flags) };
            let rr = {
                let (st, ch) = (&mut el.st[c], &mut el.ch[c]);
                cconcealment_apply(
                    &mut st.concealment_info,
                    &el.conceal,
                    ch,
                    &mut el.spec[c * 1024..(c + 1) * 1024],
                    &el.sri,
                    1024,
                    0,
                    frame_ok,
                    flags,
                )
            };
            assert_eq!(rc, rr, "{what}: CConcealment_Apply");
            el.check(&format!("{what}: after conceal {c}"));

            let mut out_c = vec![0i32; 1024];
            let mut out_r = vec![0i32; 1024];
            unsafe {
                cref_core_el_f2t(
                    el.c,
                    c as i32,
                    out_c.as_mut_ptr(),
                    1024,
                    frame_ok,
                    headroom,
                    el_flags,
                )
            };
            {
                let El { st, ch, spec, wb, .. } = &mut el;
                cblock_frequency_to_time(
                    &mut st[c],
                    &mut ch[c],
                    &mut spec[c * 1024..(c + 1) * 1024],
                    &mut out_r,
                    1024,
                    frame_ok,
                    wb.mdct_out_temp(),
                    headroom,
                    el_flags,
                    c as i32,
                );
            }
            assert_same(&format!("{what}: time output {c}"), &out_c, &out_r);
            el.check(&format!("{what}: after f2t {c}"));
            /* the time signal lands in pTimeData2 (read as stale virtualSpec by IGF) */
            el.td2[c * 1024..(c + 1) * 1024].copy_from_slice(&out_r);
            el.sync_buffers_to_c();
        }
        el.sync_buffers_to_c();
    }
}

#[test]
fn scl_codebook_encoder() {
    let codes = scl_codes();
    assert_eq!(codes[60], (0, 1));
}

#[test]
fn element_sequences() {
    let mut cov = Cov::default();
    let mut g = Rng::new(0x5eed);
    for s in 0..800u64 {
        let cfg = Cfg::random(&mut g);
        run(s, cfg, 40, 40, &mut cov);
    }
    eprintln!("{cov:?}");
    assert!(cov.ok > cov.frames / 4, "{cov:?}");
    assert!(
        cov.cplx > 0
            && cov.ms > 0
            && cov.igf_ms > 0
            && cov.transform_splitting > 0
            && cov.stereo_filling > 0
            && cov.use_prev_frame > 0
            && cov.start_ts_cp > 0,
        "{cov:?}"
    );
}

/// Long sequences without injected errors (persistent state over many frames).
#[test]
fn long_element_sequences() {
    let mut cov = Cov::default();
    let mut g = Rng::new(0x10ee);
    for s in 0..40u64 {
        let cfg = Cfg::random(&mut g);
        run(1000 + s, cfg, 300, 0, &mut cov);
    }
    eprintln!("{cov:?}");
    assert!(cov.ok > cov.frames / 2, "{cov:?}");
}

/// Thread pool for the parallel variants.
fn test_pool() -> &'static mpeghdec_fdk::thread_pool::ThreadPool {
    static POOL: std::sync::OnceLock<mpeghdec_fdk::thread_pool::ThreadPool> =
        std::sync::OnceLock::new();
    POOL.get_or_init(|| mpeghdec_fdk::thread_pool::ThreadPool::new(3).unwrap())
}
