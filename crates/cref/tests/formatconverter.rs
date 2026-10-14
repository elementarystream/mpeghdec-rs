// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libFormatConverter: initialization for all combinations of CICP input
//! layouts and target layouts (passive time domain, active and custom STFT modes, immersive
//! downmix), random input / output geometries (VBAP fallbacks), processing of random and
//! full-scale signals over many frames (output, modified input and complete state after every
//! frame), the downmix matrix parser (`DownmixMatrixSet()`, `DownmixMatrix()` with equalizers) on
//! crafted and random bitstreams, the application of transmitted downmix matrices and EQs, and
//! internal helpers (`peak_filter()`, channel matching, matrix post-processing).
#![cfg(not(cref_missing))]

#[path = "drc_common/mod.rs"]
mod drc_common;

use drc_common::{assert_same, ser_decoder, BitWriter};
use mpeghdec_cref::drc::*;
use mpeghdec_cref::formatconverter::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_drc::drc_dec_tools::get_delta_tmin;
use mpeghdec_drc::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::*;
use mpeghdec_fdk::stftfilterbank::HandleStftFilterbank;
use mpeghdec_formatconverter::format_converter_active_dmx_stft::ActiveDownmixer;
use mpeghdec_formatconverter::format_converter_init::peak_filter;
use mpeghdec_formatconverter::*;
use mpeghdec_gvbap::quick_hull::*;

const OPEN_BUF_SIZE: u32 = 4 * 24 * 1024 * 3;
const DMX_BUF_SIZE: usize = 256;

/* ---------------------------------------------------------------------------------------- */
/* state serializer (same order as cref_fc_ser)                                             */
/* ---------------------------------------------------------------------------------------- */

fn ser_geo(g: &Cicp2GeometryChannelGeometry, o: &mut Vec<i32>) {
    o.extend_from_slice(&[
        g.cicp_loudspeaker_index,
        g.az,
        g.el,
        g.lfe,
        g.screen_relative,
        g.has_distance,
        g.distance as i32,
        g.has_loudspeaker_calibration_gain,
        g.loudspeaker_calibration_gain,
        g.loudspeaker_type,
    ]);
}

fn ser_fb(f: &Option<HandleStftFilterbank>, o: &mut Vec<i32>) {
    o.push(f.is_some() as i32);
    if let Some(f) = f {
        let c = &f.stft_filterbank_config;
        o.extend_from_slice(&[c.stft_filterbank_mode, c.frame_size as i32, c.fft_size as i32]);
        o.push(f.num_zeros_padded_two_sided);
        o.extend_from_slice(&f.prev_audio_input[..c.frame_size as usize]);
    }
}

fn dmx_sel(s: DmxMtxSel) -> i32 {
    match s {
        DmxMtxSel::None => 0,
        DmxMtxSel::Mtx => 1,
        DmxMtxSel::MtxL => 2,
        DmxMtxSel::Mtx2 => 3,
    }
}

fn eq_sel(s: EqIndexSel) -> i32 {
    match s {
        EqIndexSel::None => 0,
        EqIndexSel::Vec => 1,
        EqIndexSel::Vec2 => 2,
    }
}

fn ser_active(h: &ActiveDownmixer, o: &mut Vec<i32>) {
    o.extend_from_slice(&[h.num_in_chans as i32, h.num_out_chans as i32, h.fixp_aes]);
    for i in 0..h.num_out_chans as usize {
        for k in 0..58 {
            o.push(h.target_ene_prev[i][k]);
            o.push(h.realized_ene_prev[i][k]);
        }
    }
    for k in 0..58 {
        o.push(h.target_ene_prev_exp[k]);
        o.push(h.in_buf_stft_headroom_prev[k] as i32);
        o.push(h.realized_sig_headroom_prev[k] as i32);
    }
}

fn ser(f: &IisFormatConverter) -> Vec<i32> {
    ser_sections(f).0
}

/// Serialized state and the start offsets of its sections (for error messages).
fn ser_sections(f: &IisFormatConverter) -> (Vec<i32>, Vec<(usize, &'static str)>) {
    let mut sec = Vec::new();
    let p = &*f.member;
    let mut o = vec![
        f.num_local_speaker as i32,
        f.pas,
        f.aes,
        f.num_signals_total as i32,
        f.fc_mode,
        p.fc_num_freq_bands as i32,
        p.fc_center_frequencies.is_some() as i32,
    ];
    for g in p.input_channel_geo.iter() {
        ser_geo(g, &mut o);
    }
    o.extend(p.num_input_channels.iter().map(|&v| v as i32));
    o.push(p.num_input_channel_groups as i32);
    o.push(p.amount_of_added_dmx_matrices_and_equalizers as i32);
    for g in p.eq_gains.iter() {
        o.push(g.is_some() as i32);
        if let Some(g) = g {
            o.extend_from_slice(&g[..58]);
        }
    }
    o.extend_from_slice(&[
        p.num_total_input_channels as i32,
        p.frame_size as i32,
        p.stft_headroom_prescaling,
        p.num_time_slots as i32,
        p.sampling_rate as i32,
        p.immersive_downmix_flag as i32,
        p.rendering3_d_type_flag as i32,
        p.mode,
        p.fc_input_format,
        p.fc_output_format,
        p.aes,
        p.num_output_channels as i32,
    ]);
    for g in p.out_channel_geo.iter() {
        ser_geo(g, &mut o);
    }
    o.extend_from_slice(&[
        p.cicp_layout_index,
        p.out_channel_vbap_flag as i32,
        p.in_channel_vbap_flag as i32,
    ]);

    sec.push((o.len(), "params"));
    o.push(p.fc_params.is_some() as i32);
    if let Some(pp) = p.fc_params.as_deref() {
        let (nin, nout) = (p.num_total_input_channels as usize, p.num_output_channels as usize);
        o.extend_from_slice(&[
            pp.sampling_rate,
            pp.phase_align_strength,
            pp.adaptive_eq_strength,
            pp.format_converter_input_format_internal,
            pp.format_converter_output_format_internal,
            pp.format_converter_params_internal.is_some() as i32,
        ]);
        if let Some(pr) = pp.format_converter_params_internal.as_deref() {
            for e in pr.eq.iter() {
                o.extend_from_slice(e);
            }
        }
        o.push(pp.generic_io_fmt);
        for i in 0..nin {
            for j in 0..nout {
                o.push(pp.dmx_mtx[i][j] as i32);
                o.push(pp.dmx_mtx_l[i][j] as i32);
                o.push(pp.dmx_mtx2[i][j] as i32);
                o.push(pp.eq_index_vec[i][j]);
                o.push(pp.eq_index_vec2[i][j]);
            }
        }
        for i in 0..nin * nout {
            o.push(pp.dmx_mtx_sorted[i] as i32);
            o.push(pp.dmx_mtx_l_sorted[i] as i32);
            o.push(pp.dmx_mtx2_sorted[i] as i32);
            o.push(pp.eq_index_vec_sorted[i]);
            o.push(pp.eq_index_vec2_sorted[i]);
        }
        o.extend_from_slice(&[
            pp.immersive_mode,
            pp.mode3_drendering,
            pp.mode2_drendering,
            pp.rendering3_d_type_flag_internal,
        ]);
        o.extend_from_slice(&pp.ch_out_count);
        o.extend_from_slice(&pp.ch_out_exp);
        o.extend_from_slice(&[
            pp.dmx_iterations,
            dmx_sel(pp.dmx_matrix_l_fdk),
            dmx_sel(pp.dmx_matrix_h_fdk),
            eq_sel(pp.eq_index_fdk),
            pp.dmx_mtx_is_set,
            pp.format_converter_delay,
            pp.center_frequencies_normalized.is_some() as i32,
        ]);
    }

    sec.push((o.len(), "active downmix state"));
    o.push(p.fc_state.is_some() as i32);
    if let Some(s) = p.fc_state.as_deref() {
        o.push(s.handle_active_dmx_stft.is_some() as i32);
        if let Some(h) = s.handle_active_dmx_stft.as_deref() {
            ser_active(h, &mut o);
        }
    }

    sec.push((o.len(), "stft buffers"));
    let len = p.stft_length as usize;
    o.push(!p.input_buffer_stft.is_empty() as i32);
    if !p.input_buffer_stft.is_empty() {
        for i in 0..p.num_total_input_channels as usize {
            o.extend_from_slice(&p.input_buffer_stft[i][..len]);
        }
    }
    o.push(!p.prev_input_buffer_stft.is_empty() as i32);
    for b in p.prev_input_buffer_stft.iter() {
        o.extend_from_slice(&b[..len]);
    }
    o.push(!p.output_buffer_stft.is_empty() as i32);
    if !p.output_buffer_stft.is_empty() {
        for i in 0..p.num_output_channels as usize {
            o.extend_from_slice(&p.output_buffer_stft[i][..len]);
        }
    }
    sec.push((o.len(), "filterbanks"));
    for f in p.stft_filterbank_analysis.iter() {
        ser_fb(f, &mut o);
    }
    for f in p.stft_filterbank_synthesis.iter() {
        ser_fb(f, &mut o);
    }
    for c in [&p.stft_filterbank_config_analysis, &p.stft_filterbank_config_synthesis] {
        o.extend_from_slice(&[c.stft_filterbank_mode, c.frame_size as i32, c.fft_size as i32]);
    }
    o.extend_from_slice(&[
        p.stft_frame_size as i32,
        p.stft_length as i32,
        p.stft_num_erb_bands as i32,
    ]);
    sec.push((o.len(), "channel lists and immersive parameters"));
    o.extend_from_slice(&p.format_in_list_of_channels);
    o.push(p.format_in_list_of_channels_nchan);
    o.extend_from_slice(&p.format_out_list_of_channels);
    o.push(p.format_out_list_of_channels_nchan);
    for i in 0..13 {
        for j in 0..6 {
            o.extend_from_slice(&[p.gvh[i][j], p.gvh_e[i][j], p.gvl[i][j], p.gvl_e[i][j]]);
        }
    }
    o.extend(p.is4_gvh_stft_erb.iter().map(|&v| v as i32));
    o.extend_from_slice(&p.top_in);
    o.extend(p.mid_out.iter().map(|&v| v as i32));
    o.extend_from_slice(&[p.erb_is4_gvh_l as i32, p.erb_is4_gvh_h as i32, p.open_success]);
    (o, sec)
}

/* ---------------------------------------------------------------------------------------- */
/* helpers                                                                                  */
/* ---------------------------------------------------------------------------------------- */

fn geo_ints(g: &[Cicp2GeometryChannelGeometry]) -> Vec<i32> {
    let mut o = Vec::new();
    for x in g {
        ser_geo(x, &mut o);
    }
    o.push(0); /* never empty */
    o
}

fn cicp_geo(idx: i32) -> Option<Vec<Cicp2GeometryChannelGeometry>> {
    let mut g = [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    let (mut ch, mut lfe) = (0, 0);
    if cicp2geometry_get_geometry_from_cicp(idx, Some(&mut g), &mut ch, &mut lfe) != 0 {
        return None;
    }
    Some(g[..(ch + lfe) as usize].to_vec())
}

/// All CICP layouts known to cicp2geometry.
fn cicp_layouts() -> Vec<(i32, Vec<Cicp2GeometryChannelGeometry>)> {
    (1..256).filter_map(|i| cicp_geo(i).map(|g| (i, g))).collect()
}

/// Geometry of a signal group as the decoder hands it over (only Az, El, LFE are known).
fn group_geo(g: &[Cicp2GeometryChannelGeometry]) -> Vec<Cicp2GeometryChannelGeometry> {
    g.iter()
        .map(|x| Cicp2GeometryChannelGeometry {
            cicp_loudspeaker_index: -1,
            az: x.az,
            el: x.el,
            lfe: x.lfe,
            screen_relative: -1,
            has_distance: 0,
            distance: 0,
            has_loudspeaker_calibration_gain: 0,
            loudspeaker_calibration_gain: 0,
            loudspeaker_type: CICP2GEOMETRY_LOUDSPEAKER_UNKNOWN,
        })
        .collect()
}

fn random_geo(g: &mut Rng, n: usize, lfe_chance: i32) -> Vec<Cicp2GeometryChannelGeometry> {
    (0..n)
        .map(|_| {
            let (az, el) = match g.range(0, 5) {
                0 => (g.range(-180, 180), g.range(-90, 90)),
                1 => (
                    [
                        0, 30, -30, 110, -110, 45, -45, 90, -90, 135, -135, 180, 22, -22, 60, -60,
                        150, -150,
                    ][g.range(0, 17) as usize],
                    [0, 0, 35, -15, 90][g.range(0, 4) as usize],
                ),
                2 => (g.range(-200, 200), g.range(-100, 100)),
                _ => (
                    g.range(-180, 180) / 5 * 5,
                    [0, 0, 0, 15, 30, 35, 45, 60, -15, -30][g.range(0, 9) as usize],
                ),
            };
            Cicp2GeometryChannelGeometry {
                cicp_loudspeaker_index: -1,
                az,
                el,
                lfe: (g.range(0, 99) < lfe_chance) as i32,
                screen_relative: -1,
                loudspeaker_type: CICP2GEOMETRY_LOUDSPEAKER_UNKNOWN,
                ..Default::default()
            }
        })
        .collect()
}

/* The following two functions are copied from tests/gvbap.rs: output layouts for which the
 * VBAP fallback of the C++ code reads uninitialized memory or overflows fixed size lists are
 * excluded. */

/// Replicates the speaker search of `addGhostToSubset()`: true if the C++ code would read
/// uninitialized subset indices (exactly 4 speakers found, but not one per range).
fn subset_ub(pos: &[(i32, i32)]) -> bool {
    for r in [&RANGE_OF_SUBSET_A, &RANGE_OF_SUBSET_B, &RANGE_OF_SUBSET_C, &RANGE_OF_SUBSET_D] {
        let mut found = 0;
        let mut set = [false; 4];
        for i in 0..4 {
            for &(a, e) in pos {
                if a >= r[i * 4] && a <= r[i * 4 + 1] && e >= r[i * 4 + 2] && e <= r[i * 4 + 3] {
                    set[i] = true;
                    found += 1;
                    if found == 4 {
                        break;
                    }
                }
            }
        }
        if found == 4 && set.iter().any(|&s| !s) {
            return true;
        }
    }
    false
}

/// Replicates `qh_sphere_triangulation()` without imaginary speaker tables: true if the C++ code
/// would overflow one of its fixed size lists (at most 30 vertices, 51 triangles and 10 visible
/// triangles per added vertex for the 61 entry `edgeList` of `add_vertex()`) or if the
/// triangulation is empty.
fn c_lists_overflow(pos: &[(i32, i32)]) -> bool {
    let az: Vec<i32> = pos.iter().map(|p| p.0).collect();
    let el: Vec<i32> = pos.iter().map(|p| p.1).collect();
    let mut vl = VertexList::default();
    reset_vertex_list(&mut vl);
    qh_gen_vertex_list(pos.len() as i32, &az, &el, &mut vl);
    let mut pentagon = [0i32; 5];
    if add_imaginary_speakers(&mut vl, &mut pentagon) != 0 || vl.size > 30 {
        return true;
    }
    let mut tl = TriangleList::default();
    reset_triangle_list(&mut tl);
    let p = pentagon;
    for t in [
        [p[0], p[2], p[3]],
        [p[0], p[3], p[4]],
        [p[0], p[4], p[2]],
        [p[1], p[3], p[2]],
        [p[1], p[4], p[3]],
        [p[1], p[2], p[4]],
    ] {
        add_triangle_to_list(&mut tl, Triangle { index: t });
    }
    let mut order: Vec<SurroundList> = (0..vl.size as usize)
        .map(|i| SurroundList { index: i as i32, azi: vl.element[i].index })
        .collect();
    mpeghdec_gvbap::qsort::qsort(&mut order, azi_cmp);
    let v = &vl.element;
    for o in order {
        let vis = tl.element[..tl.size as usize]
            .iter()
            .filter(|t| {
                let [a, b, c] = t.index;
                dist_point_plane(
                    v[a as usize].xyz,
                    v[b as usize].xyz,
                    v[c as usize].xyz,
                    v[o.index as usize].xyz,
                ) > -214748
            })
            .count();
        if vis > 10 {
            return true;
        }
        add_vertex(&vl, &mut tl, o.index);
        if tl.size > 51 {
            return true;
        }
    }
    /* an empty triangulation (degenerate layout) makes the renderer read uninitialized memory */
    tl.size == 0
}

/// Random output geometry that is safe for the gVBAP fallback of the C++ code (at least one
/// speaker below 45 degrees elevation, no ambiguous quadrilateral subsets, no duplicate
/// positions, see tests/gvbap.rs).
fn random_out_geo(g: &mut Rng, n: usize) -> Vec<Cicp2GeometryChannelGeometry> {
    loop {
        let geo = random_geo(g, n, 8);
        let real: Vec<(i32, i32)> = geo
            .iter()
            .filter(|s| s.lfe == 0)
            .map(|s| (s.az.wrapping_mul(11930464), s.el.wrapping_mul(11930464)))
            .collect();
        if !real.iter().any(|&(_, e)| e.wrapping_abs() < 45 * 11930464 + 5965232) {
            continue;
        }
        if subset_ub(&real) || c_lists_overflow(&real) {
            continue;
        }
        let mut dup = false;
        for i in 0..real.len() {
            for j in 0..i {
                if (real[i].0 as i64 - real[j].0 as i64).abs() < 3 * 11930464
                    && (real[i].1 as i64 - real[j].1 as i64).abs() < 3 * 11930464
                {
                    dup = true;
                }
            }
        }
        if !dup {
            return geo;
        }
    }
}

const SAMPLE_RATES: [u32; 12] =
    [48000, 48000, 44100, 32000, 24000, 22050, 16000, 14700, 29400, 64000, 88200, 96000];

/// Thread pool for the parallel variants.
fn test_pool() -> &'static mpeghdec_fdk::thread_pool::ThreadPool {
    static POOL: std::sync::OnceLock<mpeghdec_fdk::thread_pool::ThreadPool> =
        std::sync::OnceLock::new();
    POOL.get_or_init(|| mpeghdec_fdk::thread_pool::ThreadPool::new(4).unwrap())
}

/// Differential pair of format converter instances.
struct Fc {
    c: *mut (),
    r: Option<Box<IisFormatConverter>>,
    ctx: String,
}

impl Drop for Fc {
    fn drop(&mut self) {
        if !self.c.is_null() {
            unsafe { cref_fc_close(self.c) };
        }
        iis_format_converter_close(&mut self.r);
    }
}

impl Fc {
    fn create(
        mode: i32,
        out: &[Cicp2GeometryChannelGeometry],
        fs: u32,
        frame: u32,
        ctx: &str,
    ) -> Option<Fc> {
        let gi = geo_ints(out);
        let mut err = 0;
        let c = unsafe { cref_fc_create(mode, gi.as_ptr(), out.len() as u32, fs, frame, &mut err) };
        let mut r = None;
        let er = iis_format_converter_create(&mut r, mode, out, out.len() as u32, fs, frame);
        assert_eq!(err, er, "{ctx}: create");
        if err != 0 {
            assert!(c.is_null() && r.is_none());
            return None;
        }
        let f = Fc { c, r, ctx: ctx.to_string() };
        f.check("create");
        Some(f)
    }

    fn fc(&mut self) -> &mut IisFormatConverter {
        self.r.as_mut().unwrap()
    }

    fn check(&self, label: &str) {
        let mut v = vec![0i32; 400_000];
        let n = unsafe { cref_fc_ser(self.c, v.as_mut_ptr()) };
        v.truncate(n as usize);
        let (r, sec) = ser_sections(self.r.as_ref().unwrap());
        if v != r {
            let i = (0..v.len().min(r.len())).find(|&i| v[i] != r[i]).unwrap_or(0);
            let s = sec.iter().rev().find(|x| x.0 <= i).map(|x| x.1).unwrap_or("instance");
            assert_same(&v, &r, &format!("{}: {label}: section {s}", self.ctx));
        }
    }

    fn add_input(&mut self, geo: &[Cicp2GeometryChannelGeometry]) -> i32 {
        let gi = geo_ints(geo);
        let c = unsafe { cref_fc_add_input(self.c, gi.as_ptr(), geo.len() as u32) };
        let r =
            iis_format_converter_config_add_input_setup(self.fc(), geo, geo.len() as u32, 0, None);
        assert_eq!(c, r, "{}: add_input", self.ctx);
        unsafe { cref_fc_add_signals(self.c, geo.len() as u32) };
        self.fc().num_signals_total += geo.len() as u32;
        c
    }

    fn config(&mut self, aes: u32, pas: u32, immersive: u32) {
        unsafe {
            cref_fc_set_aes(self.c, aes);
            cref_fc_set_pas(self.c, pas);
        }
        iis_format_converter_config_set_aes(self.fc(), aes);
        iis_format_converter_config_set_pas(self.fc(), pas);
        let c = unsafe { cref_fc_set_immersive(self.c, immersive) };
        let r = iis_format_converter_config_set_immersive_downmix_flag(self.fc(), immersive);
        assert_eq!(c, r, "{}: immersive", self.ctx);
    }

    fn open(&mut self) -> i32 {
        let c = unsafe { cref_fc_open(self.c, OPEN_BUF_SIZE) };
        let r = iis_format_converter_open(self.fc(), OPEN_BUF_SIZE);
        assert_eq!(c, r, "{}: open", self.ctx);
        assert_eq!(unsafe { cref_fc_is_open(self.c) }, iis_format_converter_is_open(self.fc()));
        let mut d = 0;
        iis_format_converter_get_delay(self.fc(), &mut d);
        assert_eq!(unsafe { cref_fc_get_delay(self.c) }, d as i32);
        self.check("open");
        c
    }

    /// Parses a FormatConverterFrame() with the given bit.
    fn frame_bit(&mut self, bit: u32) {
        let mut buf = [(bit << 7) as u8, 0, 0, 0];
        let mut left = 0;
        let rc = unsafe { cref_fc_frame(self.c, buf.as_mut_ptr(), 4, 1, &mut left) };
        let mut bs = FdkBitStream::reader(&buf, 1);
        let rr = format_converter_frame(self.fc(), &mut bs);
        assert_eq!((rc, left), (rr, bs.get_valid_bits()), "{}: frame", self.ctx);
    }

    /// Processes one frame of `input` (`num_in` blocks of `stride` samples) into `out`.
    /// Returns true if the output of the frame is not all zero.
    fn process(
        &mut self,
        input: &[i32],
        stride: usize,
        is_null: &[i32],
        out: &[i32],
        label: &str,
        drc: Option<(*mut (), &mut DrcDecoder)>,
    ) -> bool {
        let (drc_c, drc_r) = match drc {
            Some((c, r)) => (c, Some(r)),
            None => (core::ptr::null_mut(), None),
        };
        let mut in_c = input.to_vec();
        let mut out_c = out.to_vec();
        let rc = unsafe {
            cref_fc_process(
                self.c,
                in_c.as_mut_ptr(),
                stride as i32,
                is_null.as_ptr(),
                out_c.as_mut_ptr(),
                drc_c,
            )
        };
        /* the parallel variant runs on a copy of the state before the frame */
        let mut par = self.r.clone();
        let mut par_drc = drc_r.as_deref().cloned();
        let mut in_r = input.to_vec();
        let mut out_r = out.to_vec();
        let rr = {
            let mut chans: Vec<Option<&mut [i32]>> = in_r
                .chunks_mut(stride)
                .zip(is_null.iter())
                .map(|(c, &n)| if n != 0 { None } else { Some(c) })
                .collect();
            iis_format_converter_process(
                self.r.as_mut().unwrap(),
                drc_r,
                &mut chans,
                &mut out_r,
                stride as i32,
            )
        };
        assert_eq!(rc, rr, "{}: {label}: process return", self.ctx);
        self.check(label);
        assert_same(&out_c, &out_r, &format!("{}: {label}: output", self.ctx));
        assert_same(&in_c, &in_r, &format!("{}: {label}: input", self.ctx));

        /* parallel variant: same output, input and state */
        let mut in_p = input.to_vec();
        let mut out_p = out.to_vec();
        let rp = {
            let mut chans: Vec<Option<&mut [i32]>> = in_p
                .chunks_mut(stride)
                .zip(is_null.iter())
                .map(|(c, &n)| if n != 0 { None } else { Some(c) })
                .collect();
            iis_format_converter_process_pool(
                par.as_mut().unwrap(),
                Some(test_pool()),
                par_drc.as_mut(),
                &mut chans,
                &mut out_p,
                stride as i32,
            )
        };
        assert_eq!(rr, rp, "{}: {label}: parallel process return", self.ctx);
        assert_same(&out_r, &out_p, &format!("{}: {label}: parallel output", self.ctx));
        assert_same(&in_r, &in_p, &format!("{}: {label}: parallel input", self.ctx));
        assert_same(
            &ser_sections(self.r.as_ref().unwrap()).0,
            &ser_sections(par.as_ref().unwrap()).0,
            &format!("{}: {label}: parallel state", self.ctx),
        );
        out_c.iter().any(|&v| v != 0)
    }
}

/// Random time signal of one channel.
fn gen_signal(g: &mut Rng, dst: &mut [i32]) {
    match g.range(0, 9) {
        0 => dst.fill(0),
        1 => {
            /* full scale noise */
            for v in dst.iter_mut() {
                *v = if g.range(0, 1) == 0 { i32::MAX } else { i32::MIN };
            }
        }
        2 => {
            /* full scale square wave */
            let p = g.range(2, 200) as usize;
            for (i, v) in dst.iter_mut().enumerate() {
                *v = if (i / p) % 2 == 0 { i32::MAX } else { -i32::MAX };
            }
        }
        3 => {
            /* sine-like (triangle) with random amplitude */
            let sh = g.range(0, 12) as u32;
            let step = g.range(1, 1 << 24);
            let mut x: i32 = g.next_u32() as i32;
            for v in dst.iter_mut() {
                x = x.wrapping_add(step);
                *v = ((x ^ (x >> 31)).wrapping_sub(0x4000_0000) << 1) >> sh;
            }
        }
        4 => {
            /* impulses */
            dst.fill(0);
            for _ in 0..g.range(1, 8) {
                let i = g.range(0, dst.len() as i32 - 1) as usize;
                dst[i] = g.i32();
            }
        }
        _ => {
            /* noise with random headroom */
            let sh = g.range(0, 24) as u32;
            for v in dst.iter_mut() {
                *v = (g.next_u32() as i32) >> sh;
            }
        }
    }
}

fn gen_frame(
    g: &mut Rng,
    nin: usize,
    nout: usize,
    frame: usize,
    stride: usize,
    null_chance: i32,
) -> (Vec<i32>, Vec<i32>, Vec<i32>) {
    let mut input = vec![0i32; nin * stride];
    for ch in 0..nin {
        gen_signal(g, &mut input[ch * stride..ch * stride + frame]);
        for v in input[ch * stride + frame..(ch + 1) * stride].iter_mut() {
            *v = g.next_u32() as i32;
        }
    }
    let is_null: Vec<i32> = (0..nin).map(|_| (g.range(0, 99) < null_chance) as i32).collect();
    let mut out = vec![0i32; nout * frame];
    if g.range(0, 9) == 0 {
        let sh = g.range(4, 30) as u32;
        for v in out.iter_mut() {
            *v = (g.next_u32() as i32) >> sh;
        }
    }
    (input, is_null, out)
}

/// Runs `frames` frames through a pair of opened converters. Returns the number of frames with
/// non-zero output.
fn run_frames(f: &mut Fc, g: &mut Rng, frames: usize, null_chance: i32, toggle_3d: bool) -> usize {
    let mut nonzero = 0;
    let nin = f.r.as_ref().unwrap().member.num_total_input_channels as usize;
    let nout = f.r.as_ref().unwrap().member.num_output_channels as usize;
    let frame = f.r.as_ref().unwrap().member.frame_size as usize;
    let stride = frame + 256;
    for k in 0..frames {
        if toggle_3d && g.range(0, 3) == 0 {
            let b = g.range(0, 1) as u32;
            f.frame_bit(b);
        }
        let (input, is_null, out) = gen_frame(g, nin, nout, frame, stride, null_chance);
        nonzero += f.process(&input, stride, &is_null, &out, &format!("frame {k}"), None) as usize;
    }
    nonzero
}

/* ---------------------------------------------------------------------------------------- */
/* initialization                                                                           */
/* ---------------------------------------------------------------------------------------- */

fn init_combination(
    mode: i32,
    in_layouts: &[(i32, &[Cicp2GeometryChannelGeometry])],
    out: &[Cicp2GeometryChannelGeometry],
    fs: u32,
    immersive: u32,
    aes: u32,
    pas: u32,
    rendering_3d: Option<u32>,
    g: &mut Rng,
    frames: usize,
    ctx: &str,
) {
    let Some(mut f) = Fc::create(mode, out, fs, 1024, ctx) else { return };
    if immersive != 0 {
        f.config(0, 0, 1);
    }
    for (_, geo) in in_layouts {
        let geo = group_geo(geo);
        if f.add_input(&geo) != 0 {
            break;
        }
    }
    f.config(aes, pas, immersive);
    if let Some(b) = rendering_3d {
        f.frame_bit(b);
    }
    f.check("config");
    let err = f.open();
    if err == 0 && frames > 0 {
        run_frames(&mut f, g, frames, 0, immersive != 0);
    }
}

#[test]
fn init_all_cicp_combinations() {
    let layouts = cicp_layouts();
    let mut g = Rng::new(1);
    let mut n = 0;
    for (ii, gi) in layouts.iter() {
        if gi.len() > 28 {
            continue;
        }
        for (oi, go) in layouts.iter() {
            if go.len() > 24 {
                continue;
            }
            for mode in [
                IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
                IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT,
                IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN,
            ] {
                let immersive_values: &[u32] =
                    if mode == IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN {
                        &[0]
                    } else {
                        &[0, 1]
                    };
                for &immersive in immersive_values {
                    let fs = SAMPLE_RATES[(n % SAMPLE_RATES.len() as u64) as usize];
                    let (aes, pas) = if n % 3 == 0 { (0, 0) } else { (7, (n % 8) as u32) };
                    let r3d = if immersive != 0 { Some((n % 2) as u32) } else { None };
                    n += 1;
                    init_combination(
                        mode,
                        &[(*ii, gi)],
                        go,
                        fs,
                        immersive,
                        aes,
                        pas,
                        r3d,
                        &mut g,
                        (n % 13 == 0) as usize,
                        &format!("in {ii} out {oi} mode {mode} imm {immersive} fs {fs}"),
                    );
                }
            }
        }
    }
    assert!(n > 5000);
}

#[test]
fn init_multiple_groups() {
    let layouts: Vec<_> = cicp_layouts().into_iter().filter(|(_, g)| g.len() <= 24).collect();
    let mut g = Rng::new(2);
    for it in 0..1500 {
        let (oi, go) = &layouts[g.range(0, layouts.len() as i32 - 1) as usize];
        let ngroups = g.range(1, 4);
        let mut groups: Vec<(i32, Vec<Cicp2GeometryChannelGeometry>)> = Vec::new();
        for _ in 0..ngroups {
            if g.range(0, 2) == 0 {
                let n = g.range(1, 12) as usize;
                groups.push((-1, random_geo(&mut g, n, 10)));
            } else {
                let (ii, gi) = &layouts[g.range(0, layouts.len() as i32 - 1) as usize];
                groups.push((*ii, gi.clone()));
            }
        }
        let groups_ref: Vec<(i32, &[Cicp2GeometryChannelGeometry])> =
            groups.iter().map(|(i, v)| (*i, v.as_slice())).collect();
        let mode = [
            IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN,
        ][g.range(0, 3) as usize];
        let immersive = g.range(0, 1) as u32;
        let fs = SAMPLE_RATES[g.range(0, SAMPLE_RATES.len() as i32 - 1) as usize];
        let aes = [0, 7, 7, 3][g.range(0, 3) as usize];
        let pas = g.range(0, 7) as u32;
        let r3d = if g.range(0, 1) == 0 { Some(g.range(0, 1) as u32) } else { None };
        let frames = g.range(0, 2) as usize;
        init_combination(
            mode,
            &groups_ref,
            go,
            fs,
            immersive,
            aes,
            pas,
            r3d,
            &mut g,
            frames,
            &format!(
                "it {it} out {oi} groups {:?} mode {mode}",
                groups.iter().map(|x| x.0).collect::<Vec<_>>()
            ),
        );
    }
}

#[test]
fn init_random_geometries() {
    let mut g = Rng::new(3);
    let layouts: Vec<_> = cicp_layouts().into_iter().filter(|(_, g)| g.len() <= 24).collect();
    for it in 0..2000 {
        let out: Vec<Cicp2GeometryChannelGeometry> = if g.range(0, 2) == 0 {
            let n = g.range(1, 24) as usize;
            random_out_geo(&mut g, n)
        } else {
            layouts[g.range(0, layouts.len() as i32 - 1) as usize].1.clone()
        };
        let n = g.range(0, 28) as usize;
        let inp = random_geo(&mut g, n, 10);
        /* The other modes are not supported: formatConverterInit_internal() reads uninitialized
         * scratch memory for them (C crashes). */
        let mode = [
            IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN,
        ][g.range(0, 2) as usize];
        let fs = if g.range(0, 20) == 0 {
            [7999, 8000, 384000, 384001, 0][g.range(0, 4) as usize]
        } else {
            SAMPLE_RATES[g.range(0, SAMPLE_RATES.len() as i32 - 1) as usize]
        };
        let frame =
            if g.range(0, 10) == 0 { [512, 2048, 768][g.range(0, 2) as usize] } else { 1024 };
        let ctx = format!("it {it} mode {mode} fs {fs} frame {frame}");
        let Some(mut f) = Fc::create(mode, &out, fs, frame, &ctx) else { continue };
        let immersive = g.range(0, 1) as u32;
        f.config([0, 7][g.range(0, 1) as usize], g.range(0, 7) as u32, immersive);
        f.add_input(&inp);
        let err = f.open();
        if err == 0 && g.range(0, 3) == 0 {
            run_frames(&mut f, &mut g, 2, 10, true);
        }
    }
}

/* ---------------------------------------------------------------------------------------- */
/* processing                                                                               */
/* ---------------------------------------------------------------------------------------- */

#[test]
fn process_many_frames() {
    let layouts: Vec<_> = cicp_layouts().into_iter().filter(|(_, g)| g.len() <= 24).collect();
    let common_out = [2, 6, 6, 6, 6, 5, 5, 12, 19, 1, 3, 7, 13, 14, 16, 17];
    let common_in = [6, 12, 13, 16, 17, 19, 20, 14, 2, 1, 11, 15];
    let mut g = Rng::new(4);
    let (mut opened, mut nonzero, mut immersive_3d, mut stft) = (0, 0, 0, 0);
    for it in 0..400 {
        let oi = if g.range(0, 3) == 0 {
            layouts[g.range(0, layouts.len() as i32 - 1) as usize].0
        } else {
            common_out[g.range(0, common_out.len() as i32 - 1) as usize]
        };
        let out = cicp_geo(oi).unwrap();
        let mode = [
            IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
            IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN,
        ][g.range(0, 3) as usize];
        let fs = SAMPLE_RATES[g.range(0, SAMPLE_RATES.len() as i32 - 1) as usize];
        let frame = if mode == IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN {
            [1024, 1024, 768, 2048, 256][g.range(0, 4) as usize]
        } else {
            1024
        };
        let ctx = format!("it {it} out {oi} mode {mode} fs {fs}");
        let mut f = Fc::create(mode, &out, fs, frame, &ctx).unwrap();
        let immersive = g.range(0, 1) as u32;
        let (aes, pas) = if g.range(0, 3) == 0 { (0, 0) } else { (7, g.range(0, 7) as u32) };
        if g.range(0, 4) == 0 {
            let n = g.range(1, 16) as usize;
            f.add_input(&random_geo(&mut g, n, 10));
        } else {
            for _ in 0..g.range(1, 2) {
                let ii = common_in[g.range(0, common_in.len() as i32 - 1) as usize];
                let geo = group_geo(&cicp_geo(ii).unwrap());
                if f.add_input(&geo) != 0 {
                    break;
                }
            }
        }
        f.config(aes, pas, immersive);
        if g.range(0, 1) == 0 {
            f.frame_bit(1);
        }
        if f.open() != 0 {
            continue;
        }
        let frames = g.range(10, 40) as usize;
        let null_chance = [0, 0, 10][g.range(0, 2) as usize];
        opened += 1;
        let m = &f.r.as_ref().unwrap().member;
        immersive_3d += m.fc_params.as_ref().unwrap().mode3_drendering as usize;
        stft += (m.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT) as usize;
        nonzero += run_frames(&mut f, &mut g, frames, null_chance, true);
    }
    eprintln!("process_many_frames: {opened} opened ({stft} STFT, {immersive_3d} 3D), {nonzero} non-zero frames");
    assert!(opened > 300 && stft > 200 && immersive_3d > 12 && nonzero > 6000);
}

/* ---------------------------------------------------------------------------------------- */
/* downmix matrix bitstreams                                                                */
/* ---------------------------------------------------------------------------------------- */

fn put_escaped(w: &mut BitWriter, v: u32, n1: u32, n2: u32, n3: u32) {
    let m1 = (1u32 << n1) - 1;
    if v < m1 {
        w.put(v, n1);
        return;
    }
    w.put(m1, n1);
    let v = v - m1;
    let m2 = (1u32 << n2) - 1;
    if v < m2 {
        w.put(v, n2);
        return;
    }
    w.put(m2, n2);
    w.put(v - m2, n3);
}

fn put_range(w: &mut BitWriter, v: u32, alphabet: u32) {
    let n_bits = 31 - alphabet.leading_zeros();
    let n_unused = (1u32 << (n_bits + 1)) - alphabet;
    if v < n_unused {
        w.put(v, n_bits);
    } else {
        let x = v + n_unused;
        w.put(x >> 1, n_bits);
        w.put(x & 1, 1);
    }
}

/// Exp-Golomb like run/index code of DecodeFlatCompactMatrix() / DecodeGainValue().
fn put_lgr(w: &mut BitWriter, v: u32, p: u32, max_head: u32) {
    let head = v >> p;
    for _ in 0..head {
        w.put(0, 1);
    }
    if head < max_head {
        w.put(1, 1);
    }
    w.put(v & ((1 << p) - 1), p);
}

fn write_eq_config(w: &mut BitWriter, g: &mut Rng, input_count: usize) {
    if g.range(0, 2) == 0 {
        w.put(0, 1);
        return;
    }
    w.put(1, 1);
    let num_eq = if g.range(0, 10) == 0 { g.range(1, 39) } else { g.range(1, 4) } as u32;
    put_escaped(w, num_eq - 1, 3, 5, 0);
    let prec = g.range(0, 3) as u32;
    w.put(prec, 2);
    let ext = g.range(0, 1) as u32;
    w.put(ext, 1);
    for _ in 0..num_eq {
        let nsec = if g.range(0, 8) == 0 { g.range(1, 19) } else { g.range(1, 3) } as u32;
        put_escaped(w, nsec - 1, 2, 4, 0);
        let (mut last_p10, mut last_ld2, mut max_ld2) = (0u32, 10u32, 99u32);
        for _ in 0..nsec {
            let dp = g.range(0, (4 - last_p10 - 1) as i32) as u32;
            put_range(w, dp, 4 - last_p10);
            let p10 = last_p10 + dp;
            if p10 > last_p10 {
                last_ld2 = 10;
            }
            if p10 == 3 {
                max_ld2 = 24;
            }
            let a = 1 + max_ld2 - last_ld2;
            let dl = g.range(0, a as i32 - 1) as u32;
            put_range(w, dl, a);
            last_p10 = p10;
            last_ld2 += dl;
            let q = g.range(0, 31) as u32;
            w.put(q, 5);
            if q > 19 {
                w.put(g.range(0, 7) as u32, 3);
            }
            w.put(g.next_u32(), 4 + ext + prec);
        }
        w.put(g.next_u32(), 4 + ext + (prec + 1).min(3));
    }
    for _ in 0..input_count {
        if g.range(0, 1) == 1 {
            w.put(1, 1);
            put_range(w, g.range(0, num_eq as i32 - 1) as u32, num_eq);
        } else {
            w.put(0, 1);
        }
    }
}

fn speakers(geo: &[(i32, i32, i32)]) -> Vec<SpeakerInformation> {
    geo.iter()
        .map(|&(az, el, lfe)| SpeakerInformation {
            azimuth: az as i16,
            elevation: el as i16,
            is_lfe: lfe as i16,
            ..Default::default()
        })
        .collect()
}

/// Writes a (mostly valid) DownmixMatrix() payload for the given speaker lists.
fn write_downmix_matrix(
    w: &mut BitWriter,
    g: &mut Rng,
    in_idx: i32,
    inp: &[(i32, i32, i32)],
    out_idx: i32,
    out: &[(i32, i32, i32)],
) {
    write_eq_config(w, g, inp.len());
    let prec = if g.range(0, 30) == 0 { 3 } else { g.range(0, 2) } as u32;
    w.put(prec, 2);
    let max_gain = if g.range(0, 5) == 0 { g.range(0, 22) } else { g.range(0, 6) } as u32;
    put_escaped(w, max_gain, 3, 4, 0);
    let min_neg = if g.range(0, 5) == 0 { g.range(1, 47) } else { g.range(1, 20) } as u32;
    put_escaped(w, min_neg - 1, 4, 5, 0);

    let mut ic = speakers(inp);
    let mut oc = speakers(out);
    let (mut cic, mut coc) = (0, 0);
    let mut cin = [0usize; 32];
    let mut cout = [0usize; 32];
    convert_to_compact_config(&mut ic, inp.len() as i32, &mut cic, &mut cin);
    convert_to_compact_config(&mut oc, out.len() as i32, &mut coc, &mut cout);
    let (cic, coc) = (cic as usize, coc as usize);

    let all_sep = g.range(0, 1);
    w.put(all_sep as u32, 1);
    let mut is_sep = [1u32; 32];
    if all_sep == 0 {
        for j in 0..coc {
            if oc[cout[j]].pair_type == SpPairType::Symmetric {
                is_sep[j] = g.range(0, 1) as u32;
                w.put(is_sep[j], 1);
            }
        }
    }
    let all_sym = g.range(0, 1);
    w.put(all_sym as u32, 1);
    let mut is_sym = [1u32; 32];
    if all_sym == 0 {
        for j in 0..coc {
            is_sym[j] = g.range(0, 1) as u32;
            w.put(is_sym[j], 1);
        }
    }
    let mix_lfe = g.range(0, 3) != 0;
    w.put(mix_lfe as u32, 1);
    let raw = g.range(0, 1) == 1;
    w.put(raw as u32, 1);
    let density = g.range(10, 90);
    let mut compact = [[0u32; 32]; 32];
    if raw {
        for i in 0..cic {
            for j in 0..coc {
                if !mix_lfe || ic[cin[i]].is_lfe == oc[cout[j]].is_lfe {
                    compact[i][j] = (g.range(0, 99) < density) as u32;
                    w.put(compact[i][j], 1);
                }
            }
        }
    } else {
        let has_template =
            in_idx != -1 && out_idx != -1 && find_compact_template(in_idx, out_idx).is_some();
        let use_template = if has_template { g.range(0, 3) != 0 } else { g.range(0, 30) == 0 };
        w.put(use_template as u32, 1);
        let total = if mix_lfe {
            let il = (0..cic).filter(|&i| ic[cin[i]].is_lfe != 0).count();
            let ol = (0..coc).filter(|&j| oc[cout[j]].is_lfe != 0).count();
            (cic - il) * (coc - ol)
        } else {
            cic * coc
        };
        let n_bits = if total >= 256 { 4 } else { 3 };
        let p =
            if g.range(0, 10) == 0 { g.range(0, (1 << n_bits) - 1) } else { g.range(0, 3) } as u32;
        w.put(p, n_bits);
        let flat: Vec<u32> = (0..total).map(|_| (g.range(0, 99) < density) as u32).collect();
        let max_head = (total as u32) >> p;
        let mut run = 0u32;
        for &b in &flat {
            if b == 1 {
                put_lgr(w, run, p, max_head);
                run = 0;
            } else {
                run += 1;
            }
        }
        if run > 0 || total == 0 {
            put_lgr(w, run, p, max_head);
        }
        let template = if use_template && has_template {
            find_compact_template(in_idx, out_idx)
        } else {
            None
        };
        let mut count = 0;
        for i in 0..cic {
            for j in 0..coc {
                let (il, ol) = (ic[cin[i]].is_lfe, oc[cout[j]].is_lfe);
                if mix_lfe && il != 0 && ol != 0 {
                    compact[i][j] = g.range(0, 1) as u32;
                    w.put(compact[i][j], 1);
                } else if mix_lfe && (il ^ ol) != 0 {
                    compact[i][j] = 0;
                } else {
                    compact[i][j] = flat[count];
                    count += 1;
                    if let Some(t) = template {
                        compact[i][j] ^= t.get(i * coc + j).copied().unwrap_or(0) as u32;
                    }
                }
            }
        }
    }
    let full_asym = g.range(0, 1) == 1;
    w.put(full_asym as u32, 1);
    let raw_nz = g.range(0, 1) == 1;
    w.put(raw_nz as u32, 1);
    let prec = prec.min(2);
    let alphabet = ((max_gain + min_neg) << prec) + 2;
    let lgr = g.range(0, 7) as u32;
    if !raw_nz {
        w.put(lgr, 3);
    }
    let put_gain = |w: &mut BitWriter, g: &mut Rng| {
        if raw_nz {
            put_range(w, g.range(0, alphabet as i32 - 1) as u32, alphabet);
        } else {
            let v = g.range(0, alphabet as i32 - 1) as u32;
            put_lgr(w, v, lgr, (alphabet - 1) >> lgr);
        }
    };
    for i in 0..cic {
        let it = ic[cin[i]].pair_type;
        for j in 0..coc {
            let ot = oc[cout[j]].pair_type;
            if compact[i][j] == 0 {
                continue;
            }
            let n = if it != SpPairType::Symmetric && ot != SpPairType::Symmetric {
                1
            } else if it != SpPairType::Symmetric {
                let use_full = it == SpPairType::Single && full_asym;
                if is_sym[j] != 0 && !use_full {
                    1
                } else {
                    2
                }
            } else if ot != SpPairType::Symmetric {
                if is_sym[j] != 0 {
                    1
                } else {
                    2
                }
            } else if is_sep[j] != 0 && is_sym[j] != 0 {
                1
            } else if is_sym[j] != 0 || is_sep[j] != 0 {
                2
            } else {
                4
            };
            for _ in 0..n {
                put_gain(w, g);
            }
        }
    }
    /* trailing garbage */
    for _ in 0..g.range(0, 3) {
        w.put(g.next_u32(), 8);
    }
}

fn spk_triplets(geo: &[Cicp2GeometryChannelGeometry]) -> Vec<(i32, i32, i32)> {
    geo.iter().map(|x| (x.az, x.el, x.lfe)).collect()
}

fn ser_eq_config(e: &EqConfigStruct, o: &mut Vec<i32>) {
    o.push(e.num_eqs as i32);
    o.extend(e.eq_map.iter().map(|&v| v as i32));
    for q in e.eq_params.iter() {
        o.extend_from_slice(&[q.n_pk_filter as i32, q.g, q.g_e as i32]);
        for p in q.pk_filter_params.iter() {
            o.extend_from_slice(&[p.f, p.q, p.g, p.f_e as i32, p.q_e as i32, p.g_e as i32]);
        }
    }
}

fn ser_speakers(s: &[SpeakerInformation], o: &mut Vec<i32>) {
    for x in s {
        o.extend_from_slice(&[
            x.elevation as i32,
            x.azimuth as i32,
            x.is_lfe as i32,
            x.original_position as i32,
            x.is_already_used as i32,
            x.symmetric_pair.map(|v| v as i32).unwrap_or(-1),
            x.pair_type as i32,
        ]);
    }
}

/// Returns (return code, number of EQs).
fn decode_dmx_pair(
    in_idx: i32,
    inp: &[(i32, i32, i32)],
    out_idx: i32,
    out: &[(i32, i32, i32)],
    w: &BitWriter,
    valid: u32,
    ctx: &str,
) -> (i32, u8) {
    let mut buf = vec![0u8; DMX_BUF_SIZE];
    let n = w.bytes.len().min(DMX_BUF_SIZE);
    buf[..n].copy_from_slice(&w.bytes[..n]);
    let in_i: Vec<i32> = inp.iter().flat_map(|&(a, b, c)| [a, b, c]).chain([0]).collect();
    let out_i: Vec<i32> = out.iter().flat_map(|&(a, b, c)| [a, b, c]).chain([0]).collect();
    let mut dmx_c = vec![0i32; inp.len() * out.len() + 1];
    let mut ser_c = vec![0i32; 100_000];
    let (mut ser_len, mut left_c) = (0, 0);
    let mut buf_c = buf.clone();
    let rc = unsafe {
        cref_decode_dmx_matrix(
            in_idx,
            inp.len() as i32,
            in_i.as_ptr(),
            out_idx,
            out.len() as i32,
            out_i.as_ptr(),
            buf_c.as_mut_ptr(),
            DMX_BUF_SIZE as u32,
            valid,
            dmx_c.as_mut_ptr(),
            ser_c.as_mut_ptr(),
            &mut ser_len,
            &mut left_c,
        )
    };
    ser_c.truncate(ser_len as usize);

    let mut ic = speakers(inp);
    let mut oc = speakers(out);
    let mut eq = EqConfigStruct::default();
    let mut dmx = vec![0x1234i16; 32 * 32];
    let buf_r = buf.clone();
    let mut bs = FdkBitStream::reader(&buf_r, valid);
    let rr = decode_downmix_matrix(
        in_idx,
        inp.len() as i32,
        &mut ic,
        out_idx,
        out.len() as i32,
        &mut oc,
        &mut bs,
        &mut dmx,
        &mut eq,
    );
    assert_eq!(rc, rr, "{ctx}: return");
    assert_eq!(left_c, bs.get_valid_bits(), "{ctx}: bits left");
    let dmx_r: Vec<i32> = dmx[..inp.len() * out.len()].iter().map(|&v| v as i32).collect();
    assert_same(&dmx_c[..inp.len() * out.len()], &dmx_r, &format!("{ctx}: matrix"));
    let mut ser_r = Vec::new();
    ser_eq_config(&eq, &mut ser_r);
    ser_speakers(&ic, &mut ser_r);
    ser_speakers(&oc, &mut ser_r);
    assert_same(&ser_c, &ser_r, &format!("{ctx}: eq config / speakers"));
    (rr, eq.num_eqs)
}

#[test]
fn decode_downmix_matrix_crafted_and_random() {
    let layouts: Vec<_> = cicp_layouts();
    let mut g = Rng::new(5);
    let template_pairs = [(13, 6), (13, 14), (13, 12), (13, 7), (14, 6), (12, 6), (7, 6), (13, 2)];
    let (mut ok, mut with_eq, mut n) = (0, 0, 0);
    for it in 0..20000 {
        let (in_idx, inp) = match g.range(0, 5) {
            0 => {
                let n = g.range(1, 28) as usize;
                (-1, spk_triplets(&random_geo(&mut g, n, 10)))
            }
            1 => {
                let (a, _) = template_pairs[g.range(0, 7) as usize];
                (a, spk_triplets(&cicp_geo(a).unwrap()))
            }
            _ => {
                let (i, geo) = &layouts[g.range(0, layouts.len() as i32 - 1) as usize];
                if geo.len() > 28 {
                    continue;
                }
                (*i, spk_triplets(geo))
            }
        };
        let (out_idx, out) = if g.range(0, 2) == 0 {
            let (_, b) = template_pairs[g.range(0, 7) as usize];
            (b, spk_triplets(&cicp_geo(b).unwrap()))
        } else {
            let (i, geo) = &layouts[g.range(0, layouts.len() as i32 - 1) as usize];
            if geo.len() > 24 {
                continue;
            }
            (*i, spk_triplets(geo))
        };
        let mut w = BitWriter::new();
        if g.range(0, 5) == 0 {
            /* random bits */
            for _ in 0..g.range(0, 256) {
                w.put(g.next_u32(), 8);
            }
        } else {
            write_downmix_matrix(&mut w, &mut g, in_idx, &inp, out_idx, &out);
        }
        let mut valid = (w.bits as u32).min(DMX_BUF_SIZE as u32 * 8);
        if g.range(0, 20) == 0 {
            valid = g.range(0, valid as i32) as u32;
        }
        let (rc, neq) = decode_dmx_pair(
            in_idx,
            &inp,
            out_idx,
            &out,
            &w,
            valid,
            &format!("it {it} in {in_idx} out {out_idx}"),
        );
        n += 1;
        ok += (rc == 0) as usize;
        with_eq += (rc == 0 && neq > 0) as usize;
    }
    eprintln!("decode_downmix_matrix: {n} payloads, {ok} decoded, {with_eq} with EQs");
    assert!(ok > n / 3 && with_eq > n / 8);
}

/// Writes a DownmixMatrixSet(); matrices for `target` get real DownmixMatrix() payloads.
fn write_downmix_matrix_set(w: &mut BitWriter, g: &mut Rng, target: i32) {
    let count = if g.range(0, 5) == 0 { g.range(0, 31) } else { g.range(0, 4) } as u32;
    w.put(count, 5);
    for _ in 0..count {
        w.put(g.range(0, 127) as u32, 7);
        let typ = if g.range(0, 20) == 0 { g.range(2, 3) } else { g.range(0, 1) } as u32;
        w.put(typ, 2);
        let cicp = if g.range(0, 1) == 0 { target.clamp(0, 63) } else { g.range(0, 63) } as u32;
        w.put(cicp, 6);
        if typ == 1 {
            let m = if g.range(0, 5) == 0 { g.range(1, 9) } else { g.range(1, 2) } as u32;
            put_escaped(w, m - 1, 1, 3, 0);
            for _ in 0..m {
                let nids = if g.range(0, 5) == 0 { g.range(1, 32) } else { g.range(1, 3) } as u32;
                put_escaped(w, nids - 1, 1, 4, 4);
                for _ in 0..nids {
                    w.put(g.range(0, 31) as u32, 5);
                }
                let mut payload = BitWriter::new();
                if let (Some(out), true) = (cicp_geo(cicp as i32), g.range(0, 1) == 0) {
                    let n = g.range(1, 12) as usize;
                    let inp = spk_triplets(&random_geo(g, n, 10));
                    write_downmix_matrix(
                        &mut payload,
                        g,
                        -1,
                        &inp,
                        cicp as i32,
                        &spk_triplets(&out),
                    );
                } else {
                    for _ in 0..g.range(0, 40) {
                        payload.put(g.next_u32(), 8);
                    }
                }
                let mut size = payload.bits as u32;
                if g.range(0, 30) == 0 {
                    size = g.range(2000, 2500) as u32;
                }
                put_escaped(w, size, 8, 8, 12);
                for i in 0..size as usize {
                    let b = payload.bytes.get(i / 8).copied().unwrap_or(0);
                    w.put(((b >> (7 - i % 8)) & 1) as u32, 1);
                }
            }
        }
    }
}

#[test]
fn downmix_matrix_set_bitstreams() {
    let mut g = Rng::new(6);
    const BS_SIZE: usize = 8192;
    let (mut ok, mut stored) = (0, 0);
    for it in 0..6000 {
        let target = [6, 2, 12, 13, 19, 0, -1, 5, 1, 20][g.range(0, 9) as usize];
        let mut w = BitWriter::new();
        if g.range(0, 6) == 0 {
            for _ in 0..g.range(0, 64) {
                w.put(g.next_u32(), 8);
            }
        } else {
            write_downmix_matrix_set(&mut w, &mut g, target);
        }
        let mut buf = vec![0u8; BS_SIZE];
        let n = w.bytes.len().min(BS_SIZE);
        buf[..n].copy_from_slice(&w.bytes[..n]);
        let mut valid = (w.bits as u32).min(BS_SIZE as u32 * 8);
        if g.range(0, 20) == 0 {
            valid = g.range(0, valid as i32) as u32;
        }
        let with_drc = g.range(0, 2) == 0;
        let (drc_c, mut drc_r) = if with_drc {
            let c = unsafe { cref_drcdec_open(DRC_DEC_ALL) };
            let mut r = DrcDecoder::open(DRC_DEC_ALL);
            unsafe {
                cref_drcdec_set_codec_mode(c, DRC_DEC_MPEG_H_3DA);
                cref_drcdec_init(c, 1024, 48000, 6);
            }
            r.set_codec_mode(DRC_DEC_MPEG_H_3DA);
            r.init(1024, 48000, 6);
            (c, Some(r))
        } else {
            (core::ptr::null_mut(), None)
        };
        let dct = g.range(0, 3);
        let mut id_c = 0;
        let mut left_c = 0;
        let mut set_c = vec![0i32; 9 * 256 + 9 + 56];
        let mut buf_c = buf.clone();
        let rc = unsafe {
            cref_dmx_matrix_set(
                buf_c.as_mut_ptr(),
                BS_SIZE as u32,
                valid,
                target,
                dct,
                &mut id_c,
                set_c.as_mut_ptr(),
                &mut left_c,
                drc_c,
            )
        };
        let mut set = FdkDownmixGroupsMatrixSet::default();
        let mut id_r = 0;
        let buf_r = buf.clone();
        let mut bs = FdkBitStream::reader(&buf_r, valid);
        let rr =
            downmix_matrix_set(&mut bs, &mut set, target, dct, &mut id_r, drc_r.as_deref_mut());
        let ctx = format!("it {it} target {target}");
        assert_eq!((rc, id_c, left_c), (rr, id_r, bs.get_valid_bits()), "{ctx}");
        let mut set_r = Vec::new();
        for l in 0..9 {
            set_r.extend(set.downmix_matrix_memory[l].iter().map(|&v| v as i32));
        }
        set_r.extend_from_slice(&set.downmix_matrix_size);
        set_r.extend(set.downmix_matrix.iter().map(|&v| v as i32));
        assert_same(&set_c, &set_r, &format!("{ctx}: set"));
        ok += (rc == 0) as usize;
        stored += (rc == 0 && set.downmix_matrix_size.iter().any(|&v| v > 0)) as usize;
        if with_drc {
            let mut v = vec![0i32; 400_000];
            let n = unsafe { cref_drcdec_ser(drc_c, 3, v.as_mut_ptr()) };
            v.truncate(n as usize);
            let r = ser_decoder(drc_r.as_ref().unwrap(), 3);
            assert_same(&v, &r, &format!("{ctx}: drc state"));
            for t in 0..=20 {
                assert_eq!(
                    unsafe { cref_drcdec_get_param(drc_c, t) },
                    drc_r.as_ref().unwrap().get_param(t),
                    "{ctx}: drc param {t}"
                );
            }
            unsafe { cref_drcdec_close(drc_c) };
        }
    }
    eprintln!("downmix_matrix_set: {ok} parsed, {stored} with matrices for the target layout");
    assert!(ok > 3000 && stored > 1000);
}

/* ---------------------------------------------------------------------------------------- */
/* transmitted downmix matrices and EQs (applyDownmixMatrixPerSignalGroup())                */
/* ---------------------------------------------------------------------------------------- */

/// Rust version of cref_fc_apply_dmx().
fn apply_dmx_rust(
    fc: &mut IisFormatConverter,
    grp: i32,
    layout: i32,
    spk: &[(i32, i32, i32)],
    target: i32,
    mem: &mut [u8],
    size_bits: i32,
    offset: usize,
) -> i32 {
    let p = &mut *fc.member;
    let mut input_config =
        [SpeakerInformation::default(); FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS];
    let mut output_config = [SpeakerInformation::default(); FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS];
    for (n, &(az, el, lfe)) in spk.iter().enumerate() {
        input_config[n].azimuth = az as i16;
        input_config[n].elevation = el as i16;
        input_config[n].is_lfe = lfe as i16;
    }
    let mut og = [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    let (mut nch, mut nlfe) = (0, 0);
    if cicp2geometry_get_geometry_from_cicp(target, Some(&mut og), &mut nch, &mut nlfe) != 0 {
        return 3;
    }
    for n in 0..(nch + nlfe) as usize {
        output_config[n].azimuth = og[n].az as i16;
        output_config[n].elevation = og[n].el as i16;
        output_config[n].is_lfe = og[n].lfe as i16;
    }
    let mut bs = FdkBitStream::reader(mem, size_bits as u32);
    let mut eq = EqConfigStruct::default();
    let err = decode_downmix_matrix(
        layout,
        spk.len() as i32,
        &mut input_config,
        target,
        nch + nlfe,
        &mut output_config,
        &mut bs,
        &mut p.fc_params.as_mut().unwrap().dmx_mtx_sorted[offset..],
        &mut eq,
    );
    if err != 0 {
        return 1;
    }
    format_converter_dmx_matrix_exponent(p);
    let err = format_converter_set_eqs(
        offset,
        eq.num_eqs,
        Some(&eq.eq_params),
        &eq.eq_map,
        grp as u32,
        p,
    );
    if err != 0 {
        return 2;
    }
    p.amount_of_added_dmx_matrices_and_equalizers += p.num_input_channels[grp as usize];
    0
}

/// A signal group: its CICP index and the geometry of its channels.
type Group = (i32, Vec<(i32, i32, i32)>);

#[test]
fn transmitted_downmix_matrices() {
    let layouts: Vec<_> = cicp_layouts().into_iter().filter(|(_, g)| g.len() <= 24).collect();
    let common_out = [2, 6, 6, 12, 7, 14, 19, 1, 5];
    let common_in = [13, 13, 14, 12, 7, 6, 19, 16, 2];
    let mut g = Rng::new(7);
    let (mut applied, mut with_eq, mut nonzero) = (0, 0, 0);
    for it in 0..600 {
        let target = if g.range(0, 3) == 0 {
            layouts[g.range(0, layouts.len() as i32 - 1) as usize].0
        } else {
            common_out[g.range(0, common_out.len() as i32 - 1) as usize]
        };
        let out = cicp_geo(target).unwrap();
        let fs = SAMPLE_RATES[g.range(0, SAMPLE_RATES.len() as i32 - 1) as usize];
        let ctx = format!("it {it} target {target} fs {fs}");
        let mut f =
            Fc::create(IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT, &out, fs, 1024, &ctx)
                .unwrap();
        let immersive = (g.range(0, 3) == 0) as u32;
        let (aes, pas) = if g.range(0, 3) == 0 { (0, 0) } else { (7, g.range(0, 7) as u32) };
        let mut groups: Vec<Group> = Vec::new();
        let mut total = 0;
        for _ in 0..g.range(1, 3) {
            let (idx, geo) = if g.range(0, 3) == 0 {
                let n = g.range(1, 10) as usize;
                (-1, random_geo(&mut g, n, 10))
            } else {
                let i = common_in[g.range(0, common_in.len() as i32 - 1) as usize];
                (i, cicp_geo(i).unwrap())
            };
            if total + geo.len() > 28 {
                break;
            }
            total += geo.len();
            f.add_input(&group_geo(&geo));
            groups.push((idx, spk_triplets(&geo)));
        }
        f.config(aes, pas, immersive);
        /* In the immersive 2D rendering the active downmix uses the equalizer indices of the rule
         * based downmix (eqIndexVec2_sorted) with the bitstream equalizers: C reads uninitialized
         * equalizer pointers for indices beyond the number of bitstream equalizers. Only the 3D
         * rendering (which uses the bitstream equalizer indices) is compared. */
        let fixed_3d = immersive != 0 && (out.len() == 5 || out.len() == 6);
        if fixed_3d {
            f.frame_bit(1);
        }
        if f.open() != 0 {
            continue;
        }
        let nout = out.len();
        let mut offset = 0usize;
        let mut ok = true;
        for (grp, (idx, spk)) in groups.iter().enumerate() {
            let mut w = BitWriter::new();
            write_downmix_matrix(&mut w, &mut g, *idx, spk, target, &spk_triplets(&out));
            let mut mem = vec![0u8; DMX_BUF_SIZE];
            let n = w.bytes.len().min(DMX_BUF_SIZE);
            mem[..n].copy_from_slice(&w.bytes[..n]);
            let bits = (w.bits as i32).min(DMX_BUF_SIZE as i32 * 8);
            let spk_i: Vec<i32> = spk.iter().flat_map(|&(a, b, c)| [a, b, c]).chain([0]).collect();
            let mut mem_c = mem.clone();
            let rc = unsafe {
                cref_fc_apply_dmx(
                    f.c,
                    grp as i32,
                    *idx,
                    spk.len() as i32,
                    spk_i.as_ptr(),
                    target,
                    mem_c.as_mut_ptr(),
                    bits,
                    offset as i32,
                )
            };
            let rr = apply_dmx_rust(
                f.r.as_mut().unwrap(),
                grp as i32,
                *idx,
                spk,
                target,
                &mut mem,
                bits,
                offset,
            );
            assert_eq!(rc, rr, "{ctx}: apply_dmx group {grp}");
            f.check(&format!("apply_dmx group {grp}"));
            applied += (rc == 0) as usize;
            with_eq += (rc == 0
                && f.r
                    .as_ref()
                    .unwrap()
                    .member
                    .eq_gains
                    .iter()
                    .flatten()
                    .flatten()
                    .any(|&v| v != FIXP_EQ_BITSTREAM_H_FORMAT_1_DOT_0))
                as usize;
            if rc != 0 {
                ok = false;
                break;
            }
            offset += spk.len() * nout;
        }
        if ok {
            let frames = g.range(3, 12) as usize;
            nonzero += run_frames(&mut f, &mut g, frames, 0, !fixed_3d);
        }
    }
    eprintln!("transmitted_downmix_matrices: {applied} matrices applied ({with_eq} with EQ gains), {nonzero} non-zero frames");
    assert!(applied > 300 && with_eq > 150 && nonzero > 1000);
}

/* ---------------------------------------------------------------------------------------- */
/* internal helpers                                                                         */
/* ---------------------------------------------------------------------------------------- */

#[test]
fn peak_filter_random() {
    let mut g = Rng::new(8);
    for it in 0..200000 {
        let (f, f_e, q, q_e, gg, g_e, big_g, big_g_e, b, b_e) = if g.range(0, 3) == 0 {
            (
                g.i32(),
                g.range(-40, 40),
                g.i32(),
                g.range(-40, 40),
                g.i32(),
                g.range(-40, 40),
                g.i32(),
                g.range(-40, 40),
                g.i32(),
                g.range(-40, 40),
            )
        } else {
            /* ranges of the bitstream EQs */
            (
                g.range(0x4000_0000, i32::MAX),
                g.range(4, 17),
                g.range(0x4000_0000, i32::MAX),
                g.range(-4, 7),
                g.i32() >> g.range(0, 8),
                g.range(0, 6),
                g.i32() >> g.range(0, 8),
                g.range(0, 6),
                g.range(0, i32::MAX),
                g.range(10, 16),
            )
        };
        let r0 = g.range(-5, 5);
        let mut ec = r0;
        let c =
            unsafe { cref_peak_filter(f, f_e, q, q_e, gg, g_e, big_g, big_g_e, b, b_e, &mut ec) };
        let mut er = r0;
        let r = peak_filter(f, f_e, q, q_e, gg, g_e, big_g, big_g_e, b, b_e, &mut er);
        assert_eq!(
            (c, ec),
            (r, er),
            "it {it}: {:?}",
            (f, f_e, q, q_e, gg, g_e, big_g, big_g_e, b, b_e)
        );
    }
}

#[test]
fn match_channels_and_postprocess() {
    let mut g = Rng::new(9);
    for it in 0..30000 {
        let n = g.range(0, 24) as usize;
        let geo = random_geo(&mut g, n, 15);
        let inout = g.range(0, 1);
        let gi = geo_ints(&geo);
        let mut vc = vec![0i32; 33];
        let mut uc = vec![0i32; 33];
        let mut nuc = 0u32;
        let rc = unsafe {
            cref_fc_match_channels(
                gi.as_ptr(),
                inout,
                n as i32,
                vc.as_mut_ptr(),
                &mut nuc,
                uc.as_mut_ptr(),
            )
        };
        let mut vr = vec![0i32; 33];
        let mut ur = vec![0i32; 33];
        let mut nur = 0u32;
        let rr = format_converter_match_ch_config2_channels(
            &geo, inout, n as i32, &mut vr, &mut nur, &mut ur,
        );
        assert_eq!((rc, nuc), (rr, nur), "it {it}");
        if rc == 0 {
            assert_eq!(&vc[..n], &vr[..n], "it {it}");
            let nu = (nuc as usize).min(n);
            assert_eq!(&uc[..nu], &ur[..nu], "it {it}");
        }

        let (nin, nout) = (g.range(1, 28) as usize, g.range(1, 24) as usize);
        let mut m: Vec<i32> = (0..nin * nout)
            .map(|_| match g.range(0, 3) {
                0 => 0,
                1 => g.range(0, 0x7FFF),
                2 => g.i16() as i32,
                _ => g.range(0, 0x3000),
            })
            .collect();
        let mut rows: Vec<Vec<i16>> =
            m.chunks(nout).map(|r| r.iter().map(|&v| v as i16).collect()).collect();
        let rc = unsafe { cref_fc_postprocess(m.as_mut_ptr(), nin as u32, nout as u32) };
        let rr = format_converter_postprocess_dmx_mtx(Some(&mut rows[..]), nin as u32, nout as u32);
        assert_eq!(rc, rr);
        let flat: Vec<i32> = rows.iter().flatten().map(|&v| v as i32).collect();
        assert_same(&m, &flat, &format!("postprocess it {it}"));
    }
}

/* ---------------------------------------------------------------------------------------- */
/* processing with DRC gains applied in the STFT domain                                     */
/* ---------------------------------------------------------------------------------------- */

const DRC_BUF_SIZE: usize = 4096;

/// Differential pair of DRC decoders.
struct Drc {
    c: *mut (),
    r: Box<DrcDecoder>,
}

impl Drop for Drc {
    fn drop(&mut self) {
        unsafe { cref_drcdec_close(self.c) };
    }
}

impl Drc {
    fn check(&self, ctx: &str) {
        let mut v = vec![0i32; 400_000];
        let n = unsafe { cref_drcdec_ser(self.c, 7, v.as_mut_ptr()) };
        v.truncate(n as usize);
        assert_same(&v, &ser_decoder(&self.r, 7), &format!("{ctx}: drc state"));
    }

    /// Feeds a config (0), loudness (1) or gain (2) payload to both decoders.
    fn read(&mut self, kind: i32, w: &BitWriter, ctx: &str) {
        let mut buf_c = vec![0u8; DRC_BUF_SIZE];
        let n = w.bytes.len().min(DRC_BUF_SIZE);
        buf_c[..n].copy_from_slice(&w.bytes[..n]);
        let valid = (w.bits as u32).min((DRC_BUF_SIZE * 8) as u32);
        let buf_r = buf_c.clone();
        let mut left_c = 0;
        let res_c = unsafe {
            match kind {
                0 => cref_drcdec_read_config(
                    self.c,
                    buf_c.as_mut_ptr(),
                    DRC_BUF_SIZE as u32,
                    valid,
                    0,
                    &mut left_c,
                ),
                1 => cref_drcdec_read_loudness(
                    self.c,
                    buf_c.as_mut_ptr(),
                    DRC_BUF_SIZE as u32,
                    valid,
                    0,
                    &mut left_c,
                ),
                _ => cref_drcdec_read_gain(
                    self.c,
                    buf_c.as_mut_ptr(),
                    DRC_BUF_SIZE as u32,
                    valid,
                    0,
                    &mut left_c,
                ),
            }
        };
        let mut bs = FdkBitStream::reader(&buf_r, valid);
        let res_r = match kind {
            0 => self.r.read_uni_drc_config(&mut bs, 0),
            1 => self.r.read_loudness_info_set(&mut bs, 0),
            _ => self.r.read_uni_drc_gain(&mut bs, 0),
        };
        assert_eq!((res_c, left_c), (res_r, bs.get_valid_bits()), "{ctx}: drc read {kind}");
    }
}

#[test]
fn process_with_drc() {
    let mut g = Rng::new(10);
    let common_out = [2, 6, 6, 5, 12, 1];
    let common_in = [6, 12, 13, 16, 19, 2];
    let mut active = 0;
    for it in 0..60 {
        let oi = common_out[g.range(0, common_out.len() as i32 - 1) as usize];
        let ii = common_in[g.range(0, common_in.len() as i32 - 1) as usize];
        let out = cicp_geo(oi).unwrap();
        let inp = group_geo(&cicp_geo(ii).unwrap());
        let fs = [48000, 44100, 32000, 24000][g.range(0, 3) as usize];
        let ctx = format!("it {it} in {ii} out {oi} fs {fs}");
        let mut f =
            Fc::create(IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT, &out, fs, 1024, &ctx)
                .unwrap();
        f.add_input(&inp);
        f.config(7, g.range(0, 7) as u32, g.range(0, 1) as u32);
        if f.open() != 0 {
            continue;
        }

        let c = unsafe { cref_drcdec_open(DRC_DEC_ALL) };
        let mut d = Drc { c, r: DrcDecoder::open(DRC_DEC_ALL) };
        let nin = inp.len() as i32;
        unsafe {
            cref_drcdec_set_codec_mode(d.c, DRC_DEC_MPEG_H_3DA);
            cref_drcdec_init(d.c, 1024, fs as i32, nin);
        }
        d.r.set_codec_mode(DRC_DEC_MPEG_H_3DA);
        d.r.init(1024, fs as i32, nin);
        let effect = g.range(1, 6);
        let rc = unsafe { cref_drcdec_set_param(d.c, DRC_DEC_EFFECT_TYPE, effect) };
        assert_eq!(rc, d.r.set_param(DRC_DEC_EFFECT_TYPE, effect), "{ctx}: effect type");
        let params = drc_common::ConfigParams {
            channels: nin as u32,
            nice: true,
            multiband: g.range(0, 3) != 0,
            downmix: Vec::new(),
            sub: 0,
        };
        let mut w = BitWriter::new();
        let set_ids = drc_common::write_uni_drc_config(&mut w, &mut g, &params);
        d.read(0, &w, &ctx);
        let mut w = BitWriter::new();
        drc_common::write_loudness_info_set(&mut w, &mut g, true, &set_ids, 0);
        d.read(1, &w, &ctx);
        d.check(&format!("{ctx}: config"));

        let stride = 1024 + 256;
        for k in 0..g.range(6, 16) {
            let mut w = BitWriter::new();
            if let Some(h) = d.r.h_gain_dec.as_deref() {
                let coef = h.drc_coef.clone();
                drc_common::write_uni_drc_gain(
                    &mut w,
                    &mut g,
                    &coef,
                    1024,
                    get_delta_tmin(fs as i32),
                    0,
                    true,
                );
            }
            d.read(2, &w, &ctx);
            let rc = unsafe { cref_drcdec_preprocess(d.c) };
            assert_eq!(rc, d.r.preprocess(), "{ctx}: preprocess");
            let (input, is_null, outb) = gen_frame(&mut g, inp.len(), out.len(), 1024, stride, 0);
            let before = d.r.h_gain_dec.as_deref().map_or(-1, |h| h.multi_band_active_drc_index);
            f.process(
                &input,
                stride,
                &is_null,
                &outb,
                &format!("frame {k}"),
                Some((d.c, &mut d.r)),
            );
            d.check(&format!("{ctx}: frame {k}"));
            active += (before >= 0) as usize;
        }
    }
    eprintln!("process_with_drc: {active} frames with an active multiband DRC");
    assert!(active > 20);
}
