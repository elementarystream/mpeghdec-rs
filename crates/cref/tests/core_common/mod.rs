// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Shared helpers of the libMpeghDec differential tests: state dumps and loads mirroring
//! shim/core_common.cpp, C/Rust pairs of the per channel structures and comparison helpers.
#![allow(dead_code)]

pub mod element;

use mpeghdec_arith::ac_arith_coder::CArcoData;
use mpeghdec_core::aac_ram::*;
use mpeghdec_core::aacdec_tns::*;
use mpeghdec_core::channelinfo::*;
use mpeghdec_core::conceal_types::*;
use mpeghdec_core::overlapadd::OVERLAP_BUFFER_SIZE;
use mpeghdec_core::stereo::*;
use mpeghdec_cref::core_common::*;
use mpeghdec_fdk::mdct::mdct_init;
use mpeghdec_fdk::tools_rom::*;
use mpeghdec_fdk::FixpSpk;
use mpeghdec_igf::igf_dec::*;

// ---------------------------------------------------------------------------------------------
// Generic helpers
// ---------------------------------------------------------------------------------------------

/// Calls a C dump function with a large enough buffer.
pub fn c_dump(f: impl FnOnce(*mut i32) -> i32) -> Vec<i32> {
    let mut buf = vec![0i32; 1 << 16];
    let n = f(buf.as_mut_ptr());
    buf.truncate(n as usize);
    buf
}

/// Index and values of the first difference (for readable assertion messages).
pub fn first_diff(a: &[i32], b: &[i32]) -> Option<(usize, i32, i32)> {
    if a.len() != b.len() {
        return Some((usize::MAX, a.len() as i32, b.len() as i32));
    }
    a.iter().zip(b).enumerate().find(|(_, (x, y))| x != y).map(|(i, (x, y))| (i, *x, *y))
}

/// Asserts that two dumps are equal.
#[track_caller]
pub fn assert_same(what: &str, c: &[i32], r: &[i32]) {
    if let Some((i, x, y)) = first_diff(c, r) {
        panic!("{what}: first difference at index {i}: C {x} Rust {y}");
    }
}

fn push_i16(o: &mut Vec<i32>, a: &[i16]) {
    o.extend(a.iter().map(|&v| v as i32));
}

fn push_u8(o: &mut Vec<i32>, a: &[u8]) {
    o.extend(a.iter().map(|&v| v as i32));
}

fn push_i8(o: &mut Vec<i32>, a: &[i8]) {
    o.extend(a.iter().map(|&v| v as i32));
}

/// Reader of a flat state array (the inverse of the dump functions).
pub struct Rd<'a> {
    pub a: &'a [i32],
    pub n: usize,
}

impl<'a> Rd<'a> {
    pub fn new(a: &'a [i32]) -> Self {
        Rd { a, n: 0 }
    }
    pub fn i32(&mut self) -> i32 {
        let v = self.a[self.n];
        self.n += 1;
        v
    }
    pub fn u8(&mut self) -> u8 {
        self.i32() as u8
    }
    pub fn i8(&mut self) -> i8 {
        self.i32() as i8
    }
    pub fn i16(&mut self) -> i16 {
        self.i32() as i16
    }
    pub fn u32(&mut self) -> u32 {
        self.i32() as u32
    }
    pub fn fill_i32(&mut self, d: &mut [i32]) {
        for v in d {
            *v = self.i32();
        }
    }
    pub fn fill_i16(&mut self, d: &mut [i16]) {
        for v in d {
            *v = self.i16();
        }
    }
    pub fn fill_u8(&mut self, d: &mut [u8]) {
        for v in d {
            *v = self.u8();
        }
    }
    pub fn fill_i8(&mut self, d: &mut [i8]) {
        for v in d {
            *v = self.i8();
        }
    }
}

// ---------------------------------------------------------------------------------------------
// ROM table ids (see cref_rom_table_id() in shim/fdk_transforms.cpp)
// ---------------------------------------------------------------------------------------------

fn rust_tables() -> [&'static [FixpSpk]; 11] {
    [
        &SINE_TABLE_512,
        &SINE_TABLE_1024,
        &SINE_WINDOW_128,
        &SINE_WINDOW_256,
        &SINE_WINDOW_512,
        &SINE_WINDOW_1024,
        &KBD_WINDOW_64,
        &KBD_WINDOW_128,
        &KBD_WINDOW_256,
        &KBD_WINDOW_1024,
        &SIN_TWIDDLE_L64,
    ]
}

pub fn table_id(t: Option<&[FixpSpk]>) -> i32 {
    match t {
        None => -1,
        Some(t) => rust_tables()
            .iter()
            .position(|r| core::ptr::eq(r.as_ptr(), t.as_ptr()))
            .map_or(-2, |i| i as i32),
    }
}

// ---------------------------------------------------------------------------------------------
// Dumps mirroring shim/core_common.cpp
// ---------------------------------------------------------------------------------------------

pub fn dump_ics(o: &mut Vec<i32>, ics: &CIcsInfo) {
    push_u8(o, &ics.window_group_length);
    o.extend([ics.window_groups, ics.valid, ics.window_shape].map(|v| v as i32));
    o.push(ics.window_sequence);
    o.extend(
        [ics.max_sf_bands, ics.max_sfb_ste, ics.scale_factor_grouping, ics.total_sf_bands]
            .map(|v| v as i32),
    );
}

pub fn load_ics(r: &mut Rd, ics: &mut CIcsInfo) {
    r.fill_u8(&mut ics.window_group_length);
    ics.window_groups = r.u8();
    ics.valid = r.u8();
    ics.window_shape = r.u8();
    ics.window_sequence = r.i32();
    ics.max_sf_bands = r.u8();
    ics.max_sfb_ste = r.u8();
    ics.scale_factor_grouping = r.u8();
    ics.total_sf_bands = r.u8();
}

pub fn dump_tns(o: &mut Vec<i32>, t: &CTnsData) {
    for w in &t.filter {
        for f in w {
            push_i8(o, &f.coeff);
            o.extend([f.start_band as i32, f.stop_band as i32]);
            o.extend([f.direction as i32, f.resolution as i32, f.order as i32]);
        }
    }
    push_u8(o, &t.number_of_filters);
    o.extend([t.data_present, t.active, t.gain_ld].map(|v| v as i32));
}

pub fn load_tns(r: &mut Rd, t: &mut CTnsData) {
    for w in &mut t.filter {
        for f in w {
            r.fill_i8(&mut f.coeff);
            f.start_band = r.u8();
            f.stop_band = r.u8();
            f.direction = r.i8();
            f.resolution = r.i8();
            f.order = r.u8();
        }
    }
    r.fill_u8(&mut t.number_of_filters);
    t.data_present = r.u8();
    t.active = r.u8();
    t.gain_ld = r.u8();
}

/// Dump of `CAacDecoderChannelInfo` without IGF data (see `cref_core_ch_dump()`).
pub fn dump_ch(c: &CAacDecoderChannelInfo) -> Vec<i32> {
    let mut o = Vec::new();
    push_i16(&mut o, &c.spec_scale);
    dump_ics(&mut o, &c.ics_info);
    o.push(c.granule_length);
    o.push(c.element_instance_tag as i32);
    o.push(c.render_mode);
    o.push(c.curr_aliasing_symmetry);
    o.push(c.transform_splitting_active as i32);
    o.push(c.fdp_data_present);
    o.push(c.fdp_spacing_index);
    let d = &c.p_dyn_data;
    push_i16(&mut o, &d.a_scale_factor);
    push_i16(&mut o, &d.a_sfb_scale);
    push_u8(&mut o, &d.a_code_book);
    push_u8(&mut o, &d.band_is_noise);
    dump_tns(&mut o, &d.tns_data);
    o.push(d.raw_data_info.common_window as i32);
    o.push(d.raw_data_info.global_gain as i32);
    o.push(d.specific_to_usac.fd_noise_level_and_offset as i32);
    o.push(d.specific_to_usac.tns_active as i32);
    o
}

/// Loads the layout of [`dump_ch`].
pub fn load_ch(c: &mut CAacDecoderChannelInfo, a: &[i32]) {
    let r = &mut Rd::new(a);
    r.fill_i16(&mut c.spec_scale);
    load_ics(r, &mut c.ics_info);
    c.granule_length = r.i32();
    c.element_instance_tag = r.u8();
    c.render_mode = r.i32();
    c.curr_aliasing_symmetry = r.i32();
    c.transform_splitting_active = r.u8();
    c.fdp_data_present = r.i32();
    c.fdp_spacing_index = r.i32();
    let d = &mut c.p_dyn_data;
    r.fill_i16(&mut d.a_scale_factor);
    r.fill_i16(&mut d.a_sfb_scale);
    r.fill_u8(&mut d.a_code_book);
    r.fill_u8(&mut d.band_is_noise);
    load_tns(r, &mut d.tns_data);
    d.raw_data_info.common_window = r.u8();
    d.raw_data_info.global_gain = r.u8();
    d.specific_to_usac.fd_noise_level_and_offset = r.u8();
    d.specific_to_usac.tns_active = r.u8();
    assert_eq!(r.n, a.len());
}

pub fn dump_conceal(o: &mut Vec<i32>, c: &CConcealmentInfo) {
    o.extend_from_slice(&c.spectral_coefficient[..1024]);
    push_i16(o, &c.spec_scale);
    o.push(c.i_random_phase);
    o.extend(c.prev_frame_ok);
    o.push(c.cnt_valid_frames);
    o.push(c.cnt_fade_frames);
    o.extend(c.win_grp_offset);
    o.extend(c.att_grp_offset);
    o.push(c.last_render_mode as i32);
    o.push(c.window_shape as i32);
    o.push(c.window_sequence);
    o.push(c.last_win_grp_len as i32);
    o.push(c.conceal_state);
    o.push(c.conceal_state_old);
    o.push(c.fade_old);
    o.push(c.last_fading_type);
    o.push(c.last_window_groups as i32);
    push_u8(o, &c.last_window_group_length);
    o.push(c.td_noise_seed as i32);
    o.extend(c.td_noise_states);
    push_i16(o, &c.td_noise_coef);
    o.push(c.td_noise_att as i32);
}

pub fn load_conceal(r: &mut Rd, c: &mut CConcealmentInfo) {
    r.fill_i32(&mut c.spectral_coefficient[..1024]);
    r.fill_i16(&mut c.spec_scale);
    c.i_random_phase = r.i32();
    r.fill_i32(&mut c.prev_frame_ok);
    c.cnt_valid_frames = r.i32();
    c.cnt_fade_frames = r.i32();
    r.fill_i32(&mut c.win_grp_offset);
    r.fill_i32(&mut c.att_grp_offset);
    c.last_render_mode = r.i8();
    c.window_shape = r.u8();
    c.window_sequence = r.i32();
    c.last_win_grp_len = r.u8();
    c.conceal_state = r.i32();
    c.conceal_state_old = r.i32();
    c.fade_old = r.i32();
    c.last_fading_type = r.i32();
    c.last_window_groups = r.u8();
    r.fill_u8(&mut c.last_window_group_length);
    c.td_noise_seed = r.u32();
    r.fill_i32(&mut c.td_noise_states);
    r.fill_i16(&mut c.td_noise_coef);
    c.td_noise_att = r.i16();
}

/// Dump of `CAacDecoderStaticChannelInfo` without IGF data (see `cref_core_st_dump()`).
pub fn dump_st(s: &CAacDecoderStaticChannelInfo) -> Vec<i32> {
    let mut o = Vec::new();
    let m = &s.i_mdct;
    o.extend([m.prev_tl, m.prev_nr, m.prev_fr, m.ov_offset, m.ov_size]);
    o.extend([m.prev_alias_symmetry, m.prev_prev_alias_symmetry]);
    o.push(table_id(m.prev_wrs));
    o.extend_from_slice(&m.overlap[..OVERLAP_BUFFER_SIZE as usize]);
    let a = s.h_ar_co.as_ref().expect("hArCo");
    o.push(a.m_number_lines_prev as i32);
    push_u8(&mut o, &a.c_prev);
    o.push(s.nf_random_seed as i32);
    dump_conceal(&mut o, &s.concealment_info);
    o.extend(s.ltp_param);
    o.extend([s.ltp_pitch_int_past, s.ltp_pitch_fr_past]);
    o.push(s.ltp_gain_past as i32);
    o.push(s.ltp_gain_idx_past);
    push_i16(&mut o, &s.ltp_mem_in);
    push_i16(&mut o, &s.ltp_mem_out);
    push_i16(&mut o, &s.quant_spec_prev1);
    push_i16(&mut o, &s.quant_spec_prev2);
    o.push(s.prev_window_shape as i32);
    o
}

/// Loads the layout of [`dump_st`] (the window slope `prev_wrs` is not changed).
pub fn load_st(s: &mut CAacDecoderStaticChannelInfo, a: &[i32]) {
    let r = &mut Rd::new(a);
    let m = &mut s.i_mdct;
    m.prev_tl = r.i32();
    m.prev_nr = r.i32();
    m.prev_fr = r.i32();
    m.ov_offset = r.i32();
    m.ov_size = r.i32();
    m.prev_alias_symmetry = r.i32();
    m.prev_prev_alias_symmetry = r.i32();
    r.i32();
    r.fill_i32(&mut m.overlap[..OVERLAP_BUFFER_SIZE as usize]);
    let ac = s.h_ar_co.as_mut().expect("hArCo");
    ac.m_number_lines_prev = r.i16();
    r.fill_u8(&mut ac.c_prev);
    s.nf_random_seed = r.u32();
    load_conceal(r, &mut s.concealment_info);
    r.fill_i32(&mut s.ltp_param);
    s.ltp_pitch_int_past = r.i32();
    s.ltp_pitch_fr_past = r.i32();
    s.ltp_gain_past = r.i16();
    s.ltp_gain_idx_past = r.i32();
    r.fill_i16(&mut s.ltp_mem_in);
    r.fill_i16(&mut s.ltp_mem_out);
    r.fill_i16(&mut s.quant_spec_prev1);
    r.fill_i16(&mut s.quant_spec_prev2);
    s.prev_window_shape = r.u8();
    assert_eq!(r.n, a.len());
}

pub fn dump_conceal_params(c: &CConcealParams) -> Vec<i32> {
    let mut o = Vec::new();
    push_i16(&mut o, &c.fade_out_factor);
    push_i16(&mut o, &c.fade_in_factor);
    o.extend([c.method, c.num_fade_out_frames, c.num_fade_in_frames, c.num_mute_release_frames]);
    o.push(c.comfort_noise_level);
    o
}

pub fn load_conceal_params(c: &mut CConcealParams, a: &[i32]) {
    let r = &mut Rd::new(a);
    r.fill_i16(&mut c.fade_out_factor);
    r.fill_i16(&mut c.fade_in_factor);
    c.method = r.i32();
    c.num_fade_out_frames = r.i32();
    c.num_fade_in_frames = r.i32();
    c.num_mute_release_frames = r.i32();
    c.comfort_noise_level = r.i32();
    assert_eq!(r.n, a.len());
}

pub fn dump_jsd(j: &CJointStereoData) -> Vec<i32> {
    let mut o = vec![j.ms_mask_present as i32];
    push_u8(&mut o, &j.ms_used);
    o.extend([j.cplx_pred_flag, j.igf_ms_mask_present, j.igf_cplx_pred_flag].map(|v| v as i32));
    o.extend_from_slice(&j.store_dmx_re_prev);
    o.push(j.store_dmx_re_prev_e as i32);
    o
}

pub fn load_jsd(j: &mut CJointStereoData, a: &[i32]) {
    let r = &mut Rd::new(a);
    j.ms_mask_present = r.u8();
    r.fill_u8(&mut j.ms_used);
    j.cplx_pred_flag = r.u8();
    j.igf_ms_mask_present = r.u8();
    j.igf_cplx_pred_flag = r.u8();
    r.fill_i32(&mut j.store_dmx_re_prev);
    j.store_dmx_re_prev_e = r.i16();
    assert_eq!(r.n, a.len());
}

pub fn dump_cpd(c: &CCplxPredictionData) -> Vec<i32> {
    let mut o =
        [c.pred_dir, c.igf_pred_dir, c.complex_coef, c.use_prev_frame].map(|v| v as i32).to_vec();
    for g in &c.alpha_q_re {
        push_i16(&mut o, g);
    }
    for g in &c.alpha_q_im {
        push_i16(&mut o, g);
    }
    o
}

pub fn load_cpd(c: &mut CCplxPredictionData, a: &[i32]) {
    let r = &mut Rd::new(a);
    c.pred_dir = r.u8();
    c.igf_pred_dir = r.u8();
    c.complex_coef = r.u8();
    c.use_prev_frame = r.u8();
    for g in &mut c.alpha_q_re {
        r.fill_i16(g);
    }
    for g in &mut c.alpha_q_im {
        r.fill_i16(g);
    }
    assert_eq!(r.n, a.len());
}

pub fn dump_cpe(p: &CpePersistentData) -> Vec<i32> {
    let j = &p.joint_stereo_persistent_data;
    let mut o = vec![j.clear_spectral_coeffs as i32];
    for g in &j.alpha_q_re_prev {
        push_i16(&mut o, g);
    }
    for g in &j.alpha_q_im_prev {
        push_i16(&mut o, g);
    }
    o.extend([j.win_seq_prev, j.win_shape_prev, j.win_groups_prev].map(|v| v as i32));
    o
}

pub fn load_cpe(p: &mut CpePersistentData, a: &[i32]) {
    let j = &mut p.joint_stereo_persistent_data;
    let r = &mut Rd::new(a);
    j.clear_spectral_coeffs = r.u8();
    for g in &mut j.alpha_q_re_prev {
        r.fill_i16(g);
    }
    for g in &mut j.alpha_q_im_prev {
        r.fill_i16(g);
    }
    j.win_seq_prev = r.u8();
    j.win_shape_prev = r.u8();
    j.win_groups_prev = r.u8();
    assert_eq!(r.n, a.len());
}

// ---------------------------------------------------------------------------------------------
// C/Rust pairs
// ---------------------------------------------------------------------------------------------

/// A C and a Rust `CAacDecoderChannelInfo` (the C one with its `pDynData`).
pub struct ChPair {
    pub c: *mut (),
    pub r: Box<CAacDecoderChannelInfo>,
}

impl ChPair {
    pub fn new() -> Self {
        ChPair { c: unsafe { cref_core_ch_new() }, r: get_aac_decoder_channel_info(0) }
    }
    /// Loads the state of the Rust structure into the C structure.
    pub fn sync_to_c(&mut self) {
        let d = dump_ch(&self.r);
        unsafe { cref_core_ch_load(self.c, d.as_ptr()) };
    }
    #[track_caller]
    pub fn check(&self, what: &str) {
        let c = c_dump(|o| unsafe { cref_core_ch_dump(self.c, o) });
        assert_same(&format!("{what}: channel info"), &c, &dump_ch(&self.r));
    }
}

impl Drop for ChPair {
    fn drop(&mut self) {
        unsafe { cref_core_ch_free(self.c) };
    }
}

/// A C and a Rust `CAacDecoderStaticChannelInfo`, allocated like `CAacDecoder_Init()` does for
/// USAC/MPEG-H (concealment buffer, overlap buffer with `mdct_init()`, `hArCo`).
pub struct StPair {
    pub c: *mut (),
    pub r: Box<CAacDecoderStaticChannelInfo>,
}

impl StPair {
    pub fn new() -> Self {
        let mut r = get_aac_decoder_static_channel_info(0);
        r.h_ar_co = Some(CArcoData::create());
        mdct_init(&mut r.i_mdct, OVERLAP_BUFFER_SIZE);
        StPair { c: unsafe { cref_core_st_new() }, r }
    }
    pub fn sync_to_c(&mut self) {
        let d = dump_st(&self.r);
        unsafe { cref_core_st_load(self.c, d.as_ptr()) };
    }
    #[track_caller]
    pub fn check(&self, what: &str) {
        let c = c_dump(|o| unsafe { cref_core_st_dump(self.c, o) });
        assert_same(&format!("{what}: static channel info"), &c, &dump_st(&self.r));
    }
}

impl Drop for StPair {
    fn drop(&mut self) {
        unsafe { cref_core_st_free(self.c) };
    }
}

// ---------------------------------------------------------------------------------------------
// IGF state dumps (mirror cref_igf_dump_* of shim/igf.cpp, copied from tests/igf.rs)
// ---------------------------------------------------------------------------------------------

pub fn dump_igf_static(s: &IgfPrivateStaticData) -> Vec<i32> {
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

pub fn dump_igf_data(d: &IgfPrivateData) -> Vec<i32> {
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

pub fn dump_igf_common(c: &IgfPrivateDataCommon) -> Vec<i32> {
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
