// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of FDK_cicp2geometry.cpp: all CICP layout indices, all loudspeaker
//! indices, geometry to CICP index matching and front speaker selection.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_cicp2geometry::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::cicp2geometry::*;

type Geo = Cicp2GeometryChannelGeometry;

fn to_ints(g: &Geo) -> [i32; 10] {
    [
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
    ]
}

fn from_ints(o: &[i32]) -> Geo {
    Geo {
        cicp_loudspeaker_index: o[0],
        az: o[1],
        el: o[2],
        lfe: o[3],
        screen_relative: o[4],
        has_distance: o[5],
        distance: o[6] as u32,
        has_loudspeaker_calibration_gain: o[7],
        loudspeaker_calibration_gain: o[8],
        loudspeaker_type: o[9],
    }
}

fn random_geo(g: &mut Rng) -> [i32; 10] {
    let mut v = [0i32; 10];
    for x in v.iter_mut() {
        *x = g.range(-3, 3);
    }
    v
}

/// Indices to probe: every value that appears in the layout table plus a range around it.
fn cicp_indices() -> Vec<i32> {
    (-5..1100).collect()
}

fn geometry_from_cicp(idx: i32, g: &mut Rng, use_null: bool) -> (i32, i32, i32, Vec<i32>) {
    let init: Vec<i32> = (0..32).flat_map(|_| random_geo(g)).collect();
    let mut out_c = init.clone();
    let (mut nc_c, mut nl_c) = (7, 9);
    let err_c = unsafe {
        cref_cicp_geometryFromCicp(idx, out_c.as_mut_ptr(), &mut nc_c, &mut nl_c, use_null as i32)
    };
    let mut geo = [Geo::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    for i in 0..32 {
        geo[i] = from_ints(&init[10 * i..10 * i + 10]);
    }
    let (mut nc, mut nl) = (7, 9);
    let err = cicp2geometry_get_geometry_from_cicp(
        idx,
        if use_null { None } else { Some(&mut geo) },
        &mut nc,
        &mut nl,
    );
    let out_r: Vec<i32> = geo.iter().flat_map(to_ints).collect();
    assert_eq!((err, nc, nl), (err_c, nc_c, nl_c), "cicp {idx}");
    assert_eq!(out_r, out_c, "cicp {idx}");
    (err, nc, nl, out_r)
}

#[test]
fn geometry_from_cicp_all() {
    let mut g = Rng::new(3);
    let mut valid = 0;
    for idx in cicp_indices() {
        let (err, ..) = geometry_from_cicp(idx, &mut g, false);
        geometry_from_cicp(idx, &mut g, true);
        if err == CICP2GEOMETRY_OK {
            valid += 1;
        }
        let mut nt_c = 5;
        let err_c = unsafe { cref_cicp_numChannelsFromCicp(idx, &mut nt_c) };
        let mut nt = 5;
        let err_r = cicp2geometry_get_num_channels_from_cicp(idx, &mut nt);
        assert_eq!((err_r, nt), (err_c, nt_c), "numChannels {idx}");
        assert_eq!(cicp2geometry_get_num_channels_from_cicp_noerr(idx), unsafe {
            cref_cicp_numChannelsFromCicpNoErr(idx)
        });
    }
    assert_eq!(valid, 296);
}

#[test]
fn geometry_from_loudspeaker_index_all() {
    let mut g = Rng::new(4);
    for idx in -300..300 {
        for use_null in [false, true] {
            let init = random_geo(&mut g);
            let mut out_c = init;
            let err_c =
                unsafe { cref_cicp_geometryFromLsIndex(idx, out_c.as_mut_ptr(), use_null as i32) };
            let mut geo = from_ints(&init);
            let err = cicp2geometry_get_geometry_from_cicp_loudspeaker_index(
                idx,
                if use_null { None } else { Some(&mut geo) },
            );
            assert_eq!(err, err_c, "ls idx {idx}");
            assert_eq!(to_ints(&geo), out_c, "ls idx {idx}");
        }
    }
}

fn index_from_geometry(geo: &[i32], nch: i32, nlfe: i32) {
    let n = (geo.len() / 10) as i32;
    let mut idx_c = 12345;
    let err_c = unsafe { cref_cicp_cicpIndexFromGeometry(geo.as_ptr(), n, nch, nlfe, &mut idx_c) };
    let mut arr = [Geo::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    for i in 0..n as usize {
        arr[i] = from_ints(&geo[10 * i..10 * i + 10]);
    }
    let mut idx = 12345;
    let err = cicp2geometry_get_cicp_index_from_geometry(&arr, nch, nlfe, &mut idx);
    assert_eq!((err, idx), (err_c, idx_c), "nch {nch} nlfe {nlfe} geo {geo:?}");
}

#[test]
fn cicp_index_from_geometry() {
    let mut g = Rng::new(5);
    for idx in cicp_indices() {
        let (err, nc, nl, out) = geometry_from_cicp(idx, &mut g, false);
        if err != CICP2GEOMETRY_OK {
            continue;
        }
        let total = (nc + nl) as usize;
        let geo = out[..10 * total].to_vec();
        // Exact geometry, wrong counts, and perturbed geometries.
        index_from_geometry(&geo, nc, nl);
        index_from_geometry(&geo, nc + 1, nl - 1);
        index_from_geometry(&geo, nc - 1, nl + 1);
        for _ in 0..20 {
            let mut p = geo.clone();
            let k = g.range(0, (10 * total) as i32 - 1) as usize;
            p[k] += g.range(-2, 2);
            index_from_geometry(&p, nc, nl);
            // Swap two speakers.
            let (a, b) =
                (g.range(0, total as i32 - 1) as usize, g.range(0, total as i32 - 1) as usize);
            let mut q = geo.clone();
            for j in 0..10 {
                q.swap(10 * a + j, 10 * b + j);
            }
            index_from_geometry(&q, nc, nl);
        }
    }
    for _ in 0..2000 {
        let n = g.range(0, 32);
        let geo: Vec<i32> = (0..n).flat_map(|_| random_geo(&mut g)).collect();
        let nl = g.range(0, n.min(4));
        index_from_geometry(&geo, n - nl, nl);
    }
}

#[test]
fn number_of_lfes() {
    let mut g = Rng::new(6);
    for _ in 0..5000 {
        let n = g.range(0, 32) as u32;
        let geo: Vec<i32> = (0..32).flat_map(|_| random_geo(&mut g)).collect();
        let mut nl_c = 77;
        let err_c = unsafe { cref_cicp_numberOfLfes(geo.as_ptr(), n, &mut nl_c) };
        let arr: Vec<Geo> = (0..32).map(|i| from_ints(&geo[10 * i..10 * i + 10])).collect();
        let mut nl = 77;
        let err = cicp2geometry_get_number_of_lfes(&arr, n, &mut nl);
        assert_eq!((err, nl), (err_c, nl_c));
    }
}

#[test]
fn front_speakers() {
    let mut g = Rng::new(7);
    for tl in cicp_indices() {
        for ns in -1..=3 {
            let pos0 = [g.i32(), g.i32()];
            let gains0 = [g.i32(), g.i32()];
            let (mut pos_c, mut gains_c, mut mix_c) = (pos0, gains0, 99);
            let err_c = unsafe {
                cref_cicp_frontSpeakers(
                    tl,
                    ns,
                    &mut mix_c,
                    pos_c.as_mut_ptr(),
                    gains_c.as_mut_ptr(),
                )
            };
            let (mut pos, mut gains, mut mix) = (pos0, gains0, 99);
            let err = cicp2geometry_get_front_speakers(tl, ns, &mut mix, &mut pos, &mut gains);
            assert_eq!((err, mix, pos, gains), (err_c, mix_c, pos_c, gains_c), "tl {tl} ns {ns}");
        }
    }
}
