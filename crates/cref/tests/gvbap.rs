// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libgVBAPRenderer: Cartesian math, the libc `qsort()` behaviour,
//! quickhull triangulation (random layouts and all CICP layouts, with and without the enhanced
//! imaginary speaker / superset tables), renderer initialization (complete state), VBAP gains and
//! rendering of random object signals with random, moving object metadata over many frames
//! (output and complete state after every frame), OAM / production metadata parsing and garbage
//! parameters.
#![cfg(not(cref_missing))]

use mpeghdec_cref::gvbap::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::*;
use mpeghdec_gvbap::quick_hull::*;
use mpeghdec_gvbap::*;

fn assert_same(c: &[i32], r: &[i32], ctx: &str) {
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

/* ---------------------------------------------------------------------------------------- */
/* state serializer (same order as cref_gvbap_ser)                                          */
/* ---------------------------------------------------------------------------------------- */

fn ser_point(p: &PointCartesian, out: &mut Vec<i32>) {
    out.extend_from_slice(&[p.x, p.y, p.z]);
}

fn ser_oam(s: &OamSample, out: &mut Vec<i32>) {
    out.extend_from_slice(&[
        s.sph.azi,
        s.sph.ele,
        s.sph.rad,
        s.gain,
        s.spread_angle,
        s.spread_height,
        s.spread_depth,
        s.goa_bs_object_distance as i32,
    ]);
}

fn ser(h: &GVbapRenderer) -> Vec<i32> {
    ser_sections(h).0
}

/// Serialized state and the start offsets of its sections (for error messages).
fn ser_sections(h: &GVbapRenderer) -> (Vec<i32>, Vec<(usize, &'static str)>) {
    let mut sec = Vec::new();
    let mut o = Vec::new();
    let (nch, nobj, nof, gcl) = (
        h.num_channels as usize,
        h.num_objects as usize,
        h.num_oam_frames as usize,
        h.gain_cache_length as usize,
    );
    o.extend_from_slice(&[
        h.num_channels as i32,
        h.num_lfe as i32,
        h.num_ghosts as i32,
        h.num_objects as i32,
        h.frame_length as i32,
        h.oam_frame_length as i32,
        h.num_oam_frames as i32,
        h.gain_cache_length as i32,
        h.start_gains_filled as i32,
        h.render_mode as i32,
        h.has_uniform_spread as i32,
        h.oam_data_valid as i32,
        h.ghost_voice_of_god_speaker_index as i32,
        h.ghost_voice_of_hell_speaker_index as i32,
    ]);
    o.extend(h.metadata_present.iter().map(|&v| v as i32));
    o.extend(h.metadata_present_valid.iter().map(|&v| v as i32));
    o.extend(h.fixed_val.iter().map(|&v| v as i32));
    sec.push((o.len(), "speaker setup"));
    let ss = &h.speaker_setup;
    o.extend_from_slice(&[
        ss.speaker_list_size,
        ss.speaker_triplet_size,
        ss.greatest_inverse_matrix_exponent,
        ss.covers_upper_half_sphere as i32,
        ss.covers_lower_half_sphere as i32,
        ss.has_height_speakers as i32,
    ]);
    for i in 0..ss.speaker_list_size as usize {
        let s = &ss.speaker_list[i];
        o.extend_from_slice(&[
            s.sph.azi,
            s.sph.ele,
            s.sph.rad,
            s.is_lfe as i32,
            s.is_ghost as i32,
            ss.mapping[i],
        ]);
    }
    sec.push((o.len(), "speaker triplets"));
    for t in ss.speaker_triplet[..ss.speaker_triplet_size as usize].iter() {
        o.extend_from_slice(&t.triangle);
        for p in t.matrix.iter() {
            ser_point(p, &mut o);
        }
        for p in t.inverse_matrix.iter() {
            ser_point(p, &mut o);
        }
        o.push(t.exponent_inverse_matrix);
    }
    sec.push((o.len(), "downmix matrix"));
    o.push(h.downmix_matrix.is_some() as i32);
    o.push(h.downmix_matrix_num_rows);
    o.push(h.downmix_matrix_num_cols);
    if let Some(d) = h.downmix_matrix.as_ref() {
        for i in 0..h.downmix_matrix_num_rows as usize {
            o.extend_from_slice(&d[i][..h.downmix_matrix_num_cols as usize]);
        }
    }
    sec.push((o.len(), "base vectors"));
    for p in h.base_vectors.iter() {
        ser_point(p, &mut o);
    }
    sec.push((o.len(), "OAM_parsed_data"));
    o.extend_from_slice(&h.oam_parsed_data[..nobj * 8]);
    sec.push((o.len(), "oamSamples"));
    for k in 0..nof {
        for i in 0..nobj {
            ser_oam(&h.oam_samples[k][i], &mut o);
        }
    }
    sec.push((o.len(), "oamSamplesValid"));
    for k in 0..nof {
        for i in 0..nobj {
            ser_oam(&h.oam_samples_valid[k][i], &mut o);
        }
    }
    sec.push((o.len(), "start/end gains, step/scale state"));
    for i in 0..nobj {
        for j in 0..nch {
            o.extend_from_slice(&[
                h.start_gains[i][j],
                h.end_gains[i][j],
                h.step_state[i][j],
                h.scale_state[i][j],
            ]);
        }
    }
    sec.push((o.len(), "startGainsMax/prevGainsMax"));
    for j in 0..nch {
        o.extend_from_slice(&[h.start_gains_max[j], h.prev_gains_max[j]]);
    }
    sec.push((o.len(), "gainCache"));
    o.extend_from_slice(&h.gain_cache[..gcl]);
    sec.push((o.len(), "spread_gainsSpread"));
    o.extend_from_slice(&h.spread_gains_spread);
    sec.push((o.len(), "spread_gainArray/spread_gainsVSO"));
    o.push(!h.spread_gain_array.is_empty() as i32);
    if !h.spread_gain_array.is_empty() {
        o.extend_from_slice(&h.spread_gain_array[..gcl]);
        for v in h.spread_gains_vso.iter() {
            o.extend_from_slice(&v[..gcl]);
        }
    }
    (o, sec)
}

fn ser_c(c: *mut ()) -> Vec<i32> {
    let mut v = vec![0i32; 200_000];
    let n = unsafe { cref_gvbap_ser(c, v.as_mut_ptr()) };
    v.truncate(n as usize);
    v
}

/* ---------------------------------------------------------------------------------------- */
/* layouts                                                                                   */
/* ---------------------------------------------------------------------------------------- */

#[derive(Clone, Debug)]
struct Layout {
    /// (Az, El, LFE)
    spk: Vec<(i32, i32, i32)>,
    cicp: i32,
}

impl Layout {
    fn geo_flat(&self) -> Vec<i32> {
        self.spk.iter().flat_map(|&(a, e, l)| [a, e, l]).collect()
    }
    fn geo(&self) -> Vec<Cicp2GeometryChannelGeometry> {
        self.spk
            .iter()
            .map(|&(a, e, l)| Cicp2GeometryChannelGeometry {
                cicp_loudspeaker_index: -1,
                az: a,
                el: e,
                lfe: l,
                ..Default::default()
            })
            .collect()
    }
}

/// All CICP layouts known by cicp2geometry with at most 24 channels.
fn cicp_layouts() -> Vec<Layout> {
    let mut v = Vec::new();
    for cicp in 0..1024 {
        let mut g = [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
        let (mut nch, mut nlfe) = (0, 0);
        if cicp2geometry_get_geometry_from_cicp(cicp, Some(&mut g), &mut nch, &mut nlfe)
            != CICP2GEOMETRY_OK
        {
            continue;
        }
        let n = (nch + nlfe) as usize;
        if n == 0 || n > 24 {
            continue;
        }
        v.push(Layout { spk: g[..n].iter().map(|s| (s.az, s.el, s.lfe)).collect(), cicp });
    }
    v
}

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

fn rand_angle(g: &mut Rng, max: i32) -> i32 {
    match g.range(0, 9) {
        0 => [0, max, -max, 30, -30, 110, -110, 135, -135, 180, -180, 45, -45, 90, -90]
            [g.range(0, 14) as usize]
            .clamp(-max, max),
        1..=3 => g.range(-max / 6, max / 6),
        _ => g.range(-max, max),
    }
}

/// Random loudspeaker layout (C++-safe: at least one speaker below 45 degrees elevation, no
/// ambiguous quadrilateral subsets, no duplicate positions).
fn random_layout(g: &mut Rng, max_spk: i32) -> Layout {
    loop {
        let n = g.range(1, max_spk);
        let mut spk = Vec::new();
        for _ in 0..n {
            let az = rand_angle(g, 180);
            let el = match g.range(0, 5) {
                0 => rand_angle(g, 90),
                1 => g.range(-44, 44),
                _ => [0, 0, 0, 30, 35, 45, -15, -30, 60, 90, -90][g.range(0, 10) as usize],
            };
            spk.push((az, el, 0));
        }
        /* LFEs */
        for _ in 0..g.range(0, 2) {
            let pos = g.range(0, spk.len() as i32) as usize;
            spk.insert(pos, (rand_angle(g, 180), g.range(-30, 0), 1));
        }
        let real: Vec<(i32, i32)> = spk
            .iter()
            .filter(|s| s.2 == 0)
            .map(|&(a, e, _)| (a.wrapping_mul(11930464), e.wrapping_mul(11930464)))
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
        if dup {
            continue;
        }
        return Layout { spk, cicp: if g.range(0, 1) == 0 { -1 } else { 0 } };
    }
}

/* ---------------------------------------------------------------------------------------- */
/* renderer pair                                                                              */
/* ---------------------------------------------------------------------------------------- */

struct Pair {
    c: *mut (),
    r: Box<GVbapRenderer>,
    ctx: String,
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe { cref_gvbap_close(self.c) };
    }
}

impl Pair {
    fn open(
        lay: &Layout,
        num_objects: i32,
        frame_length: i32,
        oam_frame_length: i32,
        uniform: i32,
        mode: i32,
        ctx: &str,
    ) -> Option<Pair> {
        let geo = lay.geo_flat();
        let n = lay.spk.len() as i32;
        let mut err_c = 0;
        let c = unsafe {
            cref_gvbap_open(
                num_objects,
                frame_length,
                oam_frame_length,
                geo.as_ptr(),
                n,
                lay.cicp,
                uniform,
                mode,
                &mut err_c,
            )
        };
        let mut r = None;
        let err_r = gvbap_renderer_open(
            &mut r,
            num_objects,
            frame_length,
            oam_frame_length,
            &lay.geo(),
            n,
            lay.cicp,
            uniform,
            mode,
        );
        assert_eq!(err_c, err_r, "{ctx}: open error");
        if err_c != 0 {
            assert!(c.is_null());
            return None;
        }
        let p = Pair { c, r: r.unwrap(), ctx: ctx.to_string() };
        p.check("open");
        Some(p)
    }

    fn check(&self, what: &str) {
        let c = ser_c(self.c);
        let (r, sec) = ser_sections(&self.r);
        if c != r {
            let i = (0..c.len().min(r.len())).find(|&i| c[i] != r[i]).unwrap_or(0);
            let s = sec.iter().rev().find(|s| s.0 <= i).map_or(("", 0), |s| (s.1, i - s.0));
            assert_same(&c, &r, &format!("{} {what} (section {} + {})", self.ctx, s.0, s.1));
        }
    }
}

/* ---------------------------------------------------------------------------------------- */
/* tests                                                                                     */
/* ---------------------------------------------------------------------------------------- */

#[test]
fn math_primitives() {
    let mut g = Rng::new(1);
    for it in 0..200_000 {
        let (a, e, rad) = (g.i32(), g.i32(), g.i32());
        let mut c = [0i32; 3];
        unsafe { cref_gvbap_sph2cart(a, e, rad, c.as_mut_ptr()) };
        let r = spherical_to_cartesian(PointSpherical { azi: a, ele: e, rad });
        assert_eq!(c, [r.x, r.y, r.z], "sph2cart {it}");

        let pa = [g.i32(), g.i32(), g.i32()];
        let pb = [g.i32(), g.i32(), g.i32()];
        let mut c = [0i32; 4];
        unsafe { cref_gvbap_cross_dot(pa.as_ptr(), pb.as_ptr(), c.as_mut_ptr()) };
        let (qa, qb) = (
            PointCartesian { x: pa[0], y: pa[1], z: pa[2] },
            PointCartesian { x: pb[0], y: pb[1], z: pb[2] },
        );
        let x = cross_product(&qa, &qb);
        assert_eq!(c, [x.x, x.y, x.z, dot_product(&qa, &qb)], "cross/dot {it}");

        let mut c = pa;
        let rc = unsafe { cref_gvbap_normalize(c.as_mut_ptr()) };
        let mut q = qa;
        let rr = normalize(&mut q);
        assert_eq!((rc, c), (rr, [q.x, q.y, q.z]), "normalize {it}");

        /* points on or near the unit sphere and random points */
        let mut pts = [0i32; 12];
        for k in 0..4 {
            let p = if g.range(0, 3) != 0 {
                spherical_to_cartesian(PointSpherical {
                    azi: g.i32(),
                    ele: g.i32() >> 1,
                    rad: MAXVAL_DBL_,
                })
            } else {
                PointCartesian { x: g.i32(), y: g.i32(), z: g.i32() }
            };
            pts[3 * k..3 * k + 3].copy_from_slice(&[p.x, p.y, p.z]);
        }
        let c = unsafe { cref_gvbap_dist_point_plane(pts.as_ptr()) };
        let p = |k: usize| PointCartesian { x: pts[3 * k], y: pts[3 * k + 1], z: pts[3 * k + 2] };
        assert_eq!(c, dist_point_plane(p(0), p(1), p(2), p(3)), "distPointPlane {it}");

        let (a, e) = if g.range(0, 1) == 0 {
            (g.range(-180, 180) * 11930464, g.range(-180, 180) * 11930464)
        } else {
            (g.i32(), g.i32())
        };
        let mut c = [0i32; 7];
        unsafe { cref_gvbap_init_vertex(a, e, c.as_mut_ptr()) };
        let v = init_vertex(a, e);
        assert_eq!(
            c,
            [v.xyz.x, v.xyz.y, v.xyz.z, v.index, v.sph.azi, v.sph.ele, v.sph.rad],
            "init_vertex {it}"
        );
    }
}

const MAXVAL_DBL_: i32 = 0x7FFF_FFFF;

#[test]
fn qsort_matches_libc() {
    let mut g = Rng::new(2);
    for it in 0..200_000 {
        let n = match g.range(0, 3) {
            0 => g.range(0, 8),
            1 => g.range(8, 41),
            _ => g.range(41, 120),
        } as usize;
        let distinct = [1, 2, 3, 5, 8, 16, 1000, 1 << 30][g.range(0, 7) as usize];
        let pattern = g.range(0, 6);
        let mut pairs = Vec::with_capacity(2 * n);
        for i in 0..n {
            let key = match pattern {
                0 => g.range(0, distinct - 1),
                1 => i as i32 % distinct,
                2 => (n - i) as i32 % distinct,
                3 => (i.min(n - i)) as i32 % distinct,
                4 => g.i32() >> g.range(0, 31),
                _ => {
                    if g.range(0, 3) == 0 {
                        g.range(0, distinct - 1)
                    } else {
                        (i as i32) * 2
                    }
                }
            };
            pairs.push(key.wrapping_mul(2)); /* aziCmp compares azi >> 1 */
            pairs.push(i as i32);
        }
        let mut c = pairs.clone();
        unsafe { cref_gvbap_qsort(c.as_mut_ptr(), n as i32) };
        let mut r: Vec<SurroundList> =
            (0..n).map(|i| SurroundList { azi: pairs[2 * i], index: pairs[2 * i + 1] }).collect();
        mpeghdec_gvbap::qsort::qsort(&mut r, azi_cmp);
        let r: Vec<i32> = r.iter().flat_map(|s| [s.azi, s.index]).collect();
        assert_same(&c, &r, &format!("qsort {it} n {n} pattern {pattern}"));
    }
}

fn triangulate_pair(cicp: i32, azel: &[i32], ctx: &str) {
    let n = azel.len() / 2;
    let mut vc = vec![0i32; 7 * 64];
    let mut tc = vec![0i32; 3 * 128];
    let mut dc = vec![0i32; 64 * 64];
    let (mut nv, mut nt, mut rows, mut cols) = (0, 0, 0, 0);
    let err_c = unsafe {
        cref_gvbap_triangulate(
            cicp,
            azel.as_ptr(),
            n as i32,
            vc.as_mut_ptr(),
            &mut nv,
            tc.as_mut_ptr(),
            &mut nt,
            dc.as_mut_ptr(),
            &mut rows,
            &mut cols,
        )
    };
    let az: Vec<i32> = (0..n).map(|i| azel[2 * i]).collect();
    let el: Vec<i32> = (0..n).map(|i| azel[2 * i + 1]).collect();
    let mut vl = VertexList::default();
    let mut tl = TriangleList::default();
    reset_vertex_list(&mut vl);
    qh_gen_vertex_list(n as i32, &az, &el, &mut vl);
    reset_triangle_list(&mut tl);
    let mut dm = None;
    let (mut rr, mut cr) = (0, 0);
    let err_r = qh_sphere_triangulation(cicp, &mut vl, &mut tl, &mut dm, &mut rr, &mut cr);
    assert_eq!(err_c, err_r, "{ctx}: error");
    let vr: Vec<i32> = vl.element[..vl.size as usize]
        .iter()
        .flat_map(|v| [v.xyz.x, v.xyz.y, v.xyz.z, v.index, v.sph.azi, v.sph.ele, v.sph.rad])
        .collect();
    assert_same(&vc[..7 * nv as usize], &vr, &format!("{ctx}: vertices"));
    let tr: Vec<i32> = tl.element[..tl.size as usize].iter().flat_map(|t| t.index).collect();
    assert_same(&tc[..3 * nt as usize], &tr, &format!("{ctx}: triangles"));
    match dm {
        None => assert_eq!((rows, cols), (-1, -1), "{ctx}: downmix matrix"),
        Some(d) => {
            assert_eq!((rows, cols), (rr, cr), "{ctx}: downmix size");
            assert_same(
                &dc[..(rows * cols) as usize],
                &d.as_flat()[..(rows * cols) as usize],
                &format!("{ctx}: downmix"),
            );
        }
    }
}

#[test]
fn triangulation_random_layouts() {
    let mut g = Rng::new(3);
    for it in 0..50_000 {
        let lay = random_layout(&mut g, 20);
        let azel: Vec<i32> = lay
            .spk
            .iter()
            .filter(|s| s.2 == 0)
            .flat_map(|&(a, e, _)| [a * 11930464, e * 11930464])
            .collect();
        triangulate_pair(lay.cicp, &azel, &format!("triangulation {it} {:?}", lay.spk));
    }
}

#[test]
fn triangulation_cicp_layouts() {
    for lay in cicp_layouts() {
        let azel: Vec<i32> = lay
            .spk
            .iter()
            .filter(|s| s.2 == 0)
            .flat_map(|&(a, e, _)| [a * 11930464, e * 11930464])
            .collect();
        for cicp in [0, lay.cicp] {
            triangulate_pair(
                cicp,
                &azel,
                &format!("triangulation CICP {} (table {cicp})", lay.cicp),
            );
        }
    }
}

/// Random OAM value (azi, ele, rad, gain, spreadAngle, spreadHeight, spreadDepth), including
/// extreme values.
fn rand_oam(g: &mut Rng, prev: Option<[i32; 7]>) -> [i32; 7] {
    let mut v = [0i32; 7];
    let quant = |g: &mut Rng, lo: i32, hi: i32, step: i32| g.range(lo, hi).wrapping_mul(step);
    let gains = [0x0010624E, 0x10000000, 0x5F4FDF3B, 0x7FFF_FFFF, 0x0400_0000, 0];
    match (prev, g.range(0, 9)) {
        (Some(p), 0..=5) => {
            /* moving object */
            v = p;
            v[0] = v[0].wrapping_add(g.range(-3, 3) * 17895697);
            v[1] = v[1].clamp(-30 * 35791394, 30 * 35791394) + g.range(-1, 1) * 35791394;
            if g.range(0, 4) == 0 {
                v[4] = quant(g, 0, 120, 17895697);
            }
        }
        (_, 6) => {
            /* extreme values */
            for x in v.iter_mut() {
                *x = g.i32();
            }
        }
        _ => {
            v[0] = quant(g, -120, 120, 17895697);
            v[1] = quant(g, -30, 30, 35791394);
            v[2] = match g.range(0, 5) {
                0 => g.range(0, 0x0800_0000),
                1 => 0x0800_0000,
                2 => g.i32(),
                _ => 0x0800_0000 >> g.range(0, 8),
            };
            v[3] = if g.range(0, 3) == 0 {
                gains[g.range(0, 5) as usize]
            } else {
                g.range(0, 0x5F4FDF3B)
            };
            v[4] = if g.range(0, 2) == 0 { 0 } else { quant(g, 0, 120, 17895697) };
            v[5] = if g.range(0, 2) == 0 { 0 } else { quant(g, 0, 30, 35791394) };
            v[6] = if g.range(0, 2) == 0 { 0 } else { g.range(0, 0x7C00_0000) };
        }
    }
    v
}

fn rand_signal(g: &mut Rng, n: usize) -> Vec<i32> {
    let kind = g.range(0, 4);
    (0..n)
        .map(|i| match kind {
            0 => 0,
            1 => g.i32(),
            2 => ((i as i32).wrapping_mul(0x0123_4567)) >> g.range(0, 4),
            _ => (g.next_u32() as i32) >> g.range(0, 12),
        })
        .collect()
}

/// Renders `frames` frames with random OAM and signals, comparing output and state.
fn render_frames(p: &mut Pair, g: &mut Rng, frames: usize, oam_mode: i32) {
    let nobj = p.r.num_objects as usize;
    let nch = p.r.num_channels as usize;
    let nof = p.r.num_oam_frames as usize;
    let fl = p.r.frame_length as usize;
    let in_off = fl + 256;
    let mut prev: Vec<Option<[i32; 7]>> = vec![None; nobj];
    for f in 0..frames {
        /* object metadata */
        let valid = if oam_mode == 1 { (g.range(0, 7) != 0) as i32 } else { 1 };
        let uniform =
            if g.range(0, 9) == 0 { g.range(0, 1) } else { p.r.has_uniform_spread as i32 };
        /* at least one OAM frame with metadata (otherwise the C++ code reads oamSamples[-1]) */
        let first_present = g.range(0, nof as i32 - 1) as usize;
        for k in 0..nof {
            let present = k == first_present || g.range(0, 1) == 1;
            unsafe { cref_gvbap_set_flags(p.c, k as i32, present as i32, valid, uniform) };
            p.r.metadata_present[k] = present as u8;
            p.r.oam_data_valid = valid as u8;
            p.r.has_uniform_spread = uniform as u8;
            for o in 0..nobj {
                let v = rand_oam(g, prev[o]);
                prev[o] = Some(v);
                unsafe { cref_gvbap_set_oam(p.c, k as i32, o as i32, v.as_ptr()) };
                let s = &mut p.r.oam_samples[k][o];
                s.sph.azi = v[0];
                s.sph.ele = v[1];
                s.sph.rad = v[2];
                s.gain = v[3];
                s.spread_angle = v[4];
                s.spread_height = v[5];
                s.spread_depth = v[6];
            }
        }
        /* signals */
        let input = rand_signal(g, nobj * in_off);
        let mut out_c = rand_signal(g, nch * fl);
        let mut out_r = out_c.clone();
        let ssp = [256, 256, 0, 64, 128, 8 * g.range(0, (fl / 8).min(128) as i32)]
            [g.range(0, 5) as usize]
            .min(fl as i32)
            .min(256);
        let mut in_c = input.clone();
        let rc = unsafe {
            cref_gvbap_render(p.c, in_c.as_mut_ptr(), out_c.as_mut_ptr(), ssp, in_off as i32)
        };
        let rr = gvbap_renderer_render_frame_time(&mut p.r, &input, &mut out_r, ssp, in_off as i32);
        assert_eq!(rc, rr);
        assert_same(&out_c, &out_r, &format!("{} frame {f} output", p.ctx));
        p.check(&format!("frame {f}"));
    }
}

#[test]
fn open_and_render_cicp_layouts() {
    let mut g = Rng::new(4);
    let layouts = cicp_layouts();
    for (li, lay) in layouts.iter().enumerate() {
        for mode in [GVBAP_LEGACY, GVBAP_ENHANCED] {
            for uniform in [0, 1] {
                let nobj = [1, 2, 5, 16, 32][(li + uniform as usize) % 5];
                let (fl, div) = [(1024, 1), (1024, 2), (1024, 4), (768, 4), (512, 2), (256, 4)]
                    [(li * 2 + mode as usize) % 6];
                let ctx = format!("CICP {} mode {mode} uniform {uniform} objects {nobj}", lay.cicp);
                let Some(mut p) = Pair::open(lay, nobj, fl, fl / div, uniform, mode, &ctx) else {
                    panic!("{ctx}: open failed");
                };
                render_frames(&mut p, &mut g, 12, 1);
            }
        }
    }
}

#[test]
fn open_and_render_random_layouts() {
    let mut g = Rng::new(5);
    for it in 0..3000 {
        let lay = random_layout(&mut g, 20);
        let mode = g.range(0, 1);
        let uniform = g.range(0, 1);
        let nobj = g.range(1, 32);
        let (fl, div) = [(1024, 1), (1024, 2), (1024, 4), (768, 4), (512, 2), (256, 4), (256, 1)]
            [g.range(0, 6) as usize];
        let ctx = format!("random {it} {lay:?} mode {mode} uniform {uniform} objects {nobj}");
        let Some(mut p) = Pair::open(&lay, nobj, fl, fl / div, uniform, mode, &ctx) else {
            panic!("{ctx}: open failed");
        };
        render_frames(&mut p, &mut g, 4, 1);
    }
}

#[test]
fn long_render_sequences() {
    let mut g = Rng::new(6);
    let layouts = cicp_layouts();
    for it in 0..48 {
        let lay = if it % 2 == 0 {
            layouts[g.range(0, layouts.len() as i32 - 1) as usize].clone()
        } else {
            random_layout(&mut g, 16)
        };
        let mode = it % 3 % 2;
        let nobj = g.range(1, 8);
        let ctx = format!("long {it} CICP {} mode {mode}", lay.cicp);
        let Some(mut p) = Pair::open(
            &lay,
            nobj,
            1024,
            [1024, 512, 256][it as usize % 3],
            g.range(0, 1),
            mode,
            &ctx,
        ) else {
            panic!("{ctx}: open failed");
        };
        render_frames(&mut p, &mut g, 150, 1);
    }
}

#[test]
fn calculate_vbap_static_gains() {
    /* format converter usage: frame length 1, CICP index -1, enhanced mode */
    let mut g = Rng::new(7);
    let layouts = cicp_layouts();
    for it in 0..800 {
        let mut lay = if it % 2 == 0 {
            layouts[g.range(0, layouts.len() as i32 - 1) as usize].clone()
        } else {
            random_layout(&mut g, 22)
        };
        if it % 4 < 2 {
            lay.cicp = -1;
        }
        let mode = if it % 3 == 0 { GVBAP_LEGACY } else { GVBAP_ENHANCED };
        let ctx = format!("static gains {it} CICP {} mode {mode}", lay.cicp);
        let Some(mut p) = Pair::open(&lay, 24, 1, 1, 1, mode, &ctx) else {
            panic!("{ctx}: open failed");
        };
        let nch = lay.spk.len();
        for k in 0..40 {
            let mut v = rand_oam(&mut g, None);
            if g.range(0, 1) == 0 {
                v = [
                    g.range(-180, 180) * 11930464,
                    g.range(-90, 90) * 11930464,
                    0x0800_0000,
                    0x7FFF_FFFF,
                    0,
                    0,
                    0,
                ];
            }
            let no_downmix = g.range(0, 1);
            let n = if no_downmix != 0 { p.r.gain_cache_length as usize } else { nch };
            let mut gc = vec![0x5555_5555i32; n.max(nch) + 8];
            let mut gr = gc.clone();
            unsafe { cref_gvbap_calculate_vbap(p.c, v.as_ptr(), gc.as_mut_ptr(), no_downmix) };
            let s = OamSample {
                sph: PointSpherical { azi: v[0], ele: v[1], rad: v[2] },
                cart: spherical_to_cartesian(PointSpherical { azi: v[0], ele: v[1], rad: v[2] }),
                gain: v[3],
                spread_angle: v[4],
                spread_height: v[5],
                spread_depth: v[6],
                goa_bs_object_distance: 0,
            };
            calculate_vbap(&mut p.r, &s, &mut gr, no_downmix);
            assert_same(&gc, &gr, &format!("{ctx} vbap {k}"));
            p.check(&format!("vbap {k}"));
        }
    }
}

/* ---------------------------------------------------------------------------------------- */
/* metadata parsing                                                                           */
/* ---------------------------------------------------------------------------------------- */

const BUF_SIZE: usize = 1024;

fn parse_oam(p: &mut Pair, g: &mut Rng, nobj: u32) {
    let mut buf = vec![0u8; BUF_SIZE];
    let len = g.range(1, 200) as usize;
    let density = g.range(0, 3);
    for b in buf[..len].iter_mut() {
        *b = match density {
            0 => 0,
            1 => g.next_u32() as u8 & g.next_u32() as u8,
            _ => g.next_u32() as u8,
        };
    }
    let valid = (8 * len as u32).min(
        g.range(0, 8 * len as i32 + 8) as u32 + if g.range(0, 3) == 0 { 0 } else { 8 * len as u32 },
    );
    let valid = valid.min((BUF_SIZE * 8) as u32);
    let payload = g.range(0, len as i32 + 4);
    let low_delay = if g.range(0, 7) == 0 { 0 } else { 1 };
    let dyn_prio = g.range(0, 1);
    let uniform = g.range(0, 1);
    let mut buf_c = buf.clone();
    let mut left_c = 0;
    let rc = unsafe {
        cref_gvbap_oam_frame(
            p.c,
            buf_c.as_mut_ptr(),
            BUF_SIZE as u32,
            valid,
            payload,
            low_delay,
            nobj,
            dyn_prio,
            uniform,
            &mut left_c,
        )
    };
    let buf_r = buf.clone();
    let mut bs = FdkBitStream::reader(&buf_r, valid);
    let rr = object_metadata_frame(
        &mut p.r,
        &mut bs,
        payload,
        low_delay as u8,
        nobj,
        dyn_prio as u8,
        uniform as u8,
    );
    let left_r = bs.get_valid_bits();
    assert_eq!((rc, left_c), (rr, left_r), "{}: objectMetadataFrame", p.ctx);
    p.check("objectMetadataFrame");
}

fn parse_prod(p: &mut Pair, g: &mut Rng, nobj: u32, with_handle: bool) {
    let mut buf = vec![0u8; BUF_SIZE];
    for b in buf[..64].iter_mut() {
        *b = g.next_u32() as u8;
    }
    let valid = g.range(0, 64 * 8) as u32;
    let ref_dist = if g.range(0, 5) == 0 { g.i32() >> g.range(8, 31) } else { g.range(0, 255) };
    let has_dist = g.range(0, 2) as i32;
    let mut buf_c = buf.clone();
    let mut left_c = 0;
    let rc = unsafe {
        cref_gvbap_prod_frame(
            if with_handle { p.c } else { core::ptr::null_mut() },
            buf_c.as_mut_ptr(),
            BUF_SIZE as u32,
            valid,
            nobj,
            ref_dist,
            has_dist,
            &mut left_c,
        )
    };
    let buf_r = buf.clone();
    let mut bs = FdkBitStream::reader(&buf_r, valid);
    let rr = prod_metadata_frame_group(
        if with_handle { Some(&mut p.r) } else { None },
        &mut bs,
        nobj,
        ref_dist,
        has_dist,
    );
    let left_r = bs.get_valid_bits();
    assert_eq!((rc, left_c), (rr, left_r), "{}: prodMetadataFrameGroup", p.ctx);
    p.check("prodMetadataFrameGroup");
}

#[test]
fn metadata_parsing_and_rendering() {
    let mut g = Rng::new(8);
    let layouts = cicp_layouts();
    for it in 0..1500 {
        let lay = if it % 3 == 0 {
            random_layout(&mut g, 16)
        } else {
            layouts[g.range(0, layouts.len() as i32 - 1) as usize].clone()
        };
        /* 32 objects: the C++ I-frame parser does not terminate (UCHAR loop counter) */
        let nobj = g.range(1, 31);
        let mode = g.range(0, 1);
        let (fl, div) =
            [(1024, 1), (1024, 2), (1024, 4), (768, 4), (256, 4)][g.range(0, 4) as usize];
        let ctx = format!("oam {it} CICP {} mode {mode} objects {nobj}", lay.cicp);
        let Some(mut p) = Pair::open(&lay, nobj, fl, fl / div, g.range(0, 1), mode, &ctx) else {
            panic!("{ctx}: open failed");
        };
        let in_off = fl as usize + 256;
        let nch = lay.spk.len();
        for f in 0..8 {
            if g.range(0, 5) != 0 {
                parse_oam(&mut p, &mut g, nobj as u32);
            }
            if g.range(0, 1) == 0 {
                let with = g.range(0, 4) != 0;
                let n = if with { nobj as u32 } else { g.range(1, 40) as u32 };
                parse_prod(&mut p, &mut g, n, with);
            }
            let input = rand_signal(&mut g, nobj as usize * in_off);
            let mut out_c = vec![0i32; nch * fl as usize];
            let mut out_r = out_c.clone();
            let mut in_c = input.clone();
            unsafe {
                cref_gvbap_render(p.c, in_c.as_mut_ptr(), out_c.as_mut_ptr(), 256, in_off as i32)
            };
            gvbap_renderer_render_frame_time(&mut p.r, &input, &mut out_r, 256, in_off as i32);
            assert_same(&out_c, &out_r, &format!("{ctx} frame {f} output"));
            p.check(&format!("frame {f}"));
        }
    }
}

#[test]
fn oam_iframe_32_objects_terminates() {
    /* The C++ loop over the objects uses a UCHAR counter and never terminates for 32 objects;
     * the port stops at the wrap-around. */
    let lay = &cicp_layouts()[1];
    let mut r = None;
    assert_eq!(
        gvbap_renderer_open(
            &mut r,
            32,
            1024,
            1024,
            &lay.geo(),
            lay.spk.len() as i32,
            lay.cicp,
            1,
            1
        ),
        0
    );
    let mut r = r.unwrap();
    let buf = vec![0xFFu8; BUF_SIZE];
    let mut bs = FdkBitStream::reader(&buf, (BUF_SIZE * 8) as u32);
    object_metadata_frame(&mut r, &mut bs, 1000, 1, 32, 1, 0);
    let buf = vec![0x5Au8; BUF_SIZE];
    let mut bs = FdkBitStream::reader(&buf, (BUF_SIZE * 8) as u32);
    object_metadata_frame(&mut r, &mut bs, 1000, 1, 32, 0, 1);
}

#[test]
fn garbage_parameters() {
    let lay = cicp_layouts().into_iter().find(|l| l.cicp == 6).unwrap();
    let many: Layout = Layout { spk: vec![(0, 0, 0); 25], cicp: 0 };
    let cases: [(i32, i32, i32, &Layout, i32); 7] = [
        (33, 1024, 1024, &lay, 0),
        (100, 1024, 1024, &lay, 1),
        (0, 1024, 1024, &lay, 0),
        (0, 1024, 256, &lay, 1),
        (4, 2048, 1024, &lay, 0),
        (4, 1025, 1025, &lay, 1),
        (4, 1024, 1024, &many, 0),
    ];
    for (i, &(nobj, fl, ofl, l, mode)) in cases.iter().enumerate() {
        let r = Pair::open(l, nobj, fl, ofl, 1, mode, &format!("garbage {i}"));
        assert!(r.is_none(), "garbage {i} should fail");
    }
    /* Rust only: parameters for which the C++ code has undefined behaviour or tries to allocate
     * gigabytes must not panic */
    for nobj in [-1, -100, i32::MIN] {
        let mut r = None;
        assert_eq!(gvbap_renderer_open(&mut r, nobj, 1024, 1024, &lay.geo(), 6, 6, 1, 1), -2);
    }
    for nch in [-3, 0] {
        let mut r = None;
        assert_eq!(gvbap_renderer_open(&mut r, 4, 1024, 1024, &lay.geo(), nch, 6, 1, 0), -2);
    }
    for (fl, ofl) in [(1024, 0), (0, 1024), (-1024, 1024), (1024, 32), (1024, -1)] {
        let mut r = None;
        assert_eq!(gvbap_renderer_open(&mut r, 4, fl, ofl, &lay.geo(), 6, 6, 1, 1), -1);
    }
    let mut r = None;
    let up: Vec<_> = lay
        .geo()
        .into_iter()
        .map(|mut s| {
            s.el = 60;
            s
        })
        .collect();
    assert_eq!(gvbap_renderer_open(&mut r, 4, 1024, 1024, &up, 6, 0, 1, 0), -2);
}
