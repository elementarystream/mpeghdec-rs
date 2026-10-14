// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libgVBAPRenderer/include/quickHull.h and libgVBAPRenderer/src/quickHull.cpp.

//! Quickhull algorithm to calculate triangulation meshes of the convex hull around the given
//! loudspeaker setup, including the imaginary (ghost) loudspeakers and the ghost downmix matrix.
//!
//! Deviations from the C++ code, which only matter where the C++ code has undefined behaviour:
//! - The vertex, triangle and edge lists have more capacity than the C++ arrays
//!   (`GVBAPRENDERER_VERTEX_LIST_MAX_SIZE`, `GVBAPRENDERER_TRIANGLE_LIST_MAX_SIZE` and the
//!   `edgeList` of `add_vertex()`), whose overflow is not checked in the release build.
//! - [`add_ghost_to_subset`] initializes `subsetIndex` with 0; the C++ code reads uninitialized
//!   entries if several speakers fall into one range of a subset while another range is empty.
//! - [`add_imaginary_speakers`] requires at least one speaker with an elevation below 45 degrees
//!   (the C++ code writes out of bounds otherwise); it returns 1 (error) in that case.

use mpeghdec_fdk::digest::StateDigest;

use crate::cartesian_math::*;
use crate::qsort::qsort;
use crate::tables::{get_imag_speaker_table, get_speaker_super_set_table};
use mpeghdec_fdk::cicp2geometry::*;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{get_inv_int, sqrt_fixp};
use mpeghdec_fdk::matrix_calloc::Matrix2D;
use mpeghdec_fdk::scale::saturate_left_shift;

/// 11930464,7 corresponds to 1 degree in FIXP_DBL domain. Bring down because of rounding reasons
/// in init_vertex.
pub const FX_DBL_DEGREE2INT: i32 = 11930464;

/// `GVBAPRENDERER_VERTEX_LIST_MAX_SIZE` (28 + 2 safety)
pub const GVBAPRENDERER_VERTEX_LIST_MAX_SIZE: usize = 28 + 2;
/// `GVBAPRENDERER_TRIANGLE_LIST_MAX_SIZE` (49 + 2 safety)
pub const GVBAPRENDERER_TRIANGLE_LIST_MAX_SIZE: usize = 49 + 2;

/// Capacity of the Rust vertex list (see the module documentation).
const VERTEX_LIST_CAPACITY: usize = 64;
/// Capacity of the Rust triangle lists (see the module documentation).
const TRIANGLE_LIST_CAPACITY: usize = 2 * VERTEX_LIST_CAPACITY;

/// `D2F(val)`: convert a degree value to fixpoint.
pub const fn d2f(val: f64) -> FixpDbl {
    fl2fxconst_dbl(val / 180.0)
}

/* quadrilateral front and rear subsets */
/// `rangeOfSubsetA`: CH_M_L030, CH_M_R030, CH_U_R030, CH_U_L030
pub static RANGE_OF_SUBSET_A: [i32; 16] = [
    274400688, 441427194, -107374182, 238609294, -441427194, -274400688, -107374182, 238609294,
    -441427194, -131235112, 250539759, 715827883, 131235112, 441427194, 250539759, 715827883,
];

/// `rangeOfSubsetB`: CH_M_L045, CH_M_R045, CH_U_R045, CH_U_L045
pub static RANGE_OF_SUBSET_B: [i32; 16] = [
    453357659, 620384165, -107374182, 238609294, -620384165, -453357659, -107374182, 238609294,
    -787410671, -453357659, 250539759, 715827883, 453357659, 787410671, 250539759, 715827883,
];

/// `rangeOfSubsetC`: CH_M_L110, CH_U_L110, CH_U_R110, CH_M_R110
pub static RANGE_OF_SUBSET_C: [i32; 16] = [
    1204976936,
    1479377624,
    -536870912,
    238609294,
    1204976936,
    1479377624,
    250539759,
    715827883,
    -1479377624,
    -1204976936,
    250539759,
    715827883,
    -1479377624,
    -1204976936,
    -536870912,
    238609294,
];

/// `rangeOfSubsetD`: CH_M_L135, CH_U_L135, CH_U_R135, CH_M_R135
pub static RANGE_OF_SUBSET_D: [i32; 16] = [
    1491308089,
    1694125989,
    -536870912,
    238609294,
    1491308089,
    1873082960,
    250539759,
    715827883,
    -1873082960,
    -1491308089,
    250539759,
    715827883,
    -1694125989,
    -1491308089,
    -536870912,
    238609294,
];

/// `vertex`: vertex with Cartesian coordinates, sorting index and spherical coordinates.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Vertex {
    /// `xyz`
    pub xyz: PointCartesian,
    /// `index`: sorting index
    pub index: i32,
    /// `sph`
    pub sph: PointSpherical,
}

/// `vertexList`
#[derive(StateDigest, Clone, Debug)]
pub struct VertexList {
    /// `size`
    pub size: i32,
    /// `limit`
    pub limit: i32,
    /// `element`
    pub element: [Vertex; VERTEX_LIST_CAPACITY],
}

impl Default for VertexList {
    fn default() -> Self {
        VertexList { size: 0, limit: 0, element: [Vertex::default(); VERTEX_LIST_CAPACITY] }
    }
}

/// `triangle`: three vertex indices.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Triangle {
    /// `index`
    pub index: [i32; 3],
}

/// `triangleList`
#[derive(StateDigest, Clone, Debug)]
pub struct TriangleList {
    /// `size`
    pub size: i32,
    /// `limit`
    pub limit: i32,
    /// `element`
    pub element: [Triangle; TRIANGLE_LIST_CAPACITY],
}

impl Default for TriangleList {
    fn default() -> Self {
        TriangleList { size: 0, limit: 0, element: [Triangle::default(); TRIANGLE_LIST_CAPACITY] }
    }
}

/// `surroundList`: speaker azimuth and index (also used as a list of index pairs for sorting).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SurroundList {
    /// `azi`
    pub azi: FixpDbl,
    /// `index`
    pub index: i32,
}

/// `imagSpeakerTable`: pre-calculated data for imaginary loudspeakers.
#[derive(StateDigest, Debug)]
pub struct ImagSpeakerTable {
    /// `numGhost`: number of imaginary/ghost loudspeakers
    pub num_ghost: i32,
    /// `aziGhost`: azimuth angles of the imaginary loudspeakers
    pub azi_ghost: &'static [i32],
    /// `eleGhost`: elevation angles of the imaginary loudspeakers
    pub ele_ghost: &'static [i32],
    /// `initialPoly`: vertex indices of the initial polyhedron
    pub initial_poly: &'static [i32],
    /// `numRow`: number of rows of the downmix matrix
    pub num_row: i32,
    /// `numCol`: number of columns of the downmix matrix
    pub num_col: i32,
    /// `downMixData`: sequential list of the coefficients of the downmix matrix
    pub down_mix_data: &'static [FixpDbl],
}

/// `speakerSuperSetTable`
#[derive(StateDigest, Debug)]
pub struct SpeakerSuperSetTable {
    /// `supersetCicpIndex`: index of loudspeaker superset
    pub superset_cicp_index: i32,
    /// `nSpeakersSuperSet`: number of speakers in the superset (not counting LFEs)
    pub n_speakers_super_set: i32,
    /// `nSpeakersSubSet`: number of speakers in the subset (not counting LFEs)
    pub n_speakers_sub_set: i32,
    /// `downMixData`: sequential list of the coefficients of the superset-to-subset downmix matrix
    pub down_mix_data: &'static [FixpDbl],
}

/******************************************
 *   Public API Functions                 *
 ******************************************/

/// `qh_gen_VertexList(numSpk, az, el, vL)`: fills `vl` with the given speaker positions
/// (azimuth and elevation mapped from -180 ... 180 degrees to -1 ... 1).
pub fn qh_gen_vertex_list(num_spk: i32, az: &[FixpDbl], el: &[FixpDbl], vl: &mut VertexList) {
    for i in 0..num_spk.max(0) as usize {
        add_vertex_to_list(vl, init_vertex(az[i], el[i]));
    }
}

/// `aziCmp(a, b)`: `surroundList` comparison function.
pub fn azi_cmp(a: &SurroundList, b: &SurroundList) -> i32 {
    (a.azi >> 1) - (b.azi >> 1) /* shift one bit to prevent overflow */
}

/// `qh_sphere_triangulation(cicpIndex, vL, tL, downmixMatrix, downmixMatrixNumRows,
/// downmixMatrixNumCols)`: fills `tl` with speaker triangles for the vertex list `vl` (adding
/// imaginary speakers to `vl`) and generates a downmix matrix if ghost speakers were added.
/// Returns an error code.
pub fn qh_sphere_triangulation(
    mut cicp_index: i32,
    vl: &mut VertexList,
    tl: &mut TriangleList,
    downmix_matrix: &mut Option<Matrix2D<FixpDbl>>,
    downmix_matrix_num_rows: &mut i32,
    downmix_matrix_num_cols: &mut i32,
) -> i32 {
    let mut real_speaker = vl.size; /* store number of real speakers */
    let subset_speaker = real_speaker; /* number of subset speakers */
    let mut pentagon = [0i32; 5];
    let mut downmix_matrix_superset: Option<Matrix2D<FixpDbl>> = None;

    if cicp_index != 0 {
        if augment_setup_to_superset(&mut cicp_index, vl, &mut downmix_matrix_superset) != 0 {
            /*memory allocation error*/
            return 1;
        }
        real_speaker = vl.size;
    }

    /* Add imaginary speakers and get initial pentagon as convex hull */
    if cicp_index != 0 {
        if add_imaginary_speakers_cicp(cicp_index, vl, &mut pentagon) != 0 {
            return 1;
        }
    } else if add_imaginary_speakers(vl, &mut pentagon) != 0 {
        /*memory allocation error*/
        return 1;
    }

    /* Add first initial polyhedron to triangle list*/
    for i in 0..2 {
        add_triangle_to_list(
            tl,
            Triangle { index: [pentagon[0], pentagon[i + 2], pentagon[i + 3]] },
        );
    }
    add_triangle_to_list(tl, Triangle { index: [pentagon[0], pentagon[4], pentagon[2]] });

    for i in 0..2 {
        add_triangle_to_list(
            tl,
            Triangle { index: [pentagon[1], pentagon[i + 3], pentagon[i + 2]] },
        );
    }
    add_triangle_to_list(tl, Triangle { index: [pentagon[1], pentagon[2], pentagon[4]] });

    let vsize = vl.size as usize;

    /* Sorting the order of vertices by index and save the order in positionIndexList
     * Where index is the position in the vertexList vL and azi is the calculated sorting index */
    let mut position_index_list: Vec<SurroundList> =
        (0..vsize).map(|i| SurroundList { index: i as i32, azi: vl.element[i].index }).collect();

    qsort(&mut position_index_list, azi_cmp);

    for p in position_index_list.iter() {
        add_vertex(vl, tl, p.index);
    }

    /* if some ghost speaker was added then calculate downmix matrix */
    if vl.size > real_speaker {
        let vsize = vl.size as usize;
        let rs = real_speaker as usize;
        if cicp_index != 0 {
            if let Some(h_table) = get_imag_speaker_table(cicp_index) {
                /* copy downmix matrix from imaginary speaker table */
                let (rows, cols) = (h_table.num_row, h_table.num_col);
                if rows <= 0 || cols <= 0 {
                    /*memory allocation error*/
                    return 1;
                }
                let mut m = Matrix2D::new(rows as usize, cols as usize);
                for row in 0..rows as usize {
                    for col in 0..cols as usize {
                        m[row][col] = h_table.down_mix_data[col + row * cols as usize];
                    }
                }
                *downmix_matrix = Some(m);
                *downmix_matrix_num_rows = rows;
                *downmix_matrix_num_cols = cols;
            }
        }

        if downmix_matrix.is_none() {
            if rs == 0 {
                /* fdkCallocMatrix2D() returns NULL for a zero dimension */
                return 1;
            }
            /* Dim: realSpeaker x (realSpeaker + ghost) */
            let mut dmx = Matrix2D::<FixpDbl>::new(rs, vsize);
            /* Dim: (realSpeaker + ghost) x (realSpeaker + ghost) */
            let mut adjacency_matrix = Matrix2D::<FixpDbl>::new(vsize, vsize);
            let mut m_square = Matrix2D::<FixpDbl>::new(vsize, vsize);
            *downmix_matrix_num_rows = real_speaker;
            *downmix_matrix_num_cols = vl.size;

            /* create adjacencyMatrix */
            for i in 0..vsize {
                /* go through the whole adjacencyMatrix */
                if i < rs {
                    /* for real speakers set main diagonal to 1.0 */
                    adjacency_matrix[i][i] = MAXVAL_DBL; /* Set to 1.0 */
                } else {
                    /* for ghost speakers search neighbours and mark them in the adjacencyMatrix
                     * with 1.0 */
                    /* go through all Triangles */
                    let ii = i as i32;
                    for k in 0..tl.size as usize {
                        let t = tl.element[k].index;
                        if t[0] == ii || t[1] == ii || t[2] == ii {
                            /* if a neighbour to ghostspeaker i is found */
                            /* indicate neighbourhood only if the element is not itself */
                            for &e in t.iter() {
                                if e != ii {
                                    adjacency_matrix[e as usize][i] = MAXVAL_DBL;
                                    /* Set to 1.0 */
                                }
                            }
                        }
                    }
                }

                /* normalize ghost columns */
                let mut cnt = 0;
                for k in 0..vsize {
                    /* counting to how many speakers the ghost energy have to distributed */
                    if adjacency_matrix[k][i] > 0 {
                        cnt += 1;
                    }
                }
                if cnt > 0 {
                    let inverse_cnt = get_inv_int(cnt); /* Only values for cnt below 80 are allowed! */
                    for k in 0..vsize {
                        /* normalize the distributed energy */
                        if adjacency_matrix[k][i] > 0 {
                            adjacency_matrix[k][i] = f_mult(adjacency_matrix[k][i], inverse_cnt);
                        }
                    }
                }
            }

            /* distribute the energy of ghost speakers to real speakers */
            /* the lower rows belong to the ghost speaker have to be as near as possible to 0 */
            for _cnt in 0..20 {
                /* The maximum number of iterations we do is 20 */
                /* Multiply adjacencyMatrix by itself and sort the result in mSquare */
                for i in 0..vsize {
                    for k in 0..vsize {
                        let mut acc: FixpDbl = 0;
                        for j in 0..vsize {
                            acc = acc.wrapping_add(f_mult(
                                adjacency_matrix[i][j],
                                adjacency_matrix[j][k],
                            ));
                        }
                        m_square[i][k] = acc;
                    }
                }

                /* copy mSquare to adjacencyMatrix */
                adjacency_matrix.as_flat_mut().copy_from_slice(m_square.as_flat());

                /* Search the highest value in the submatrix which holds the values for the
                 * ghostspeakers. It is the part which maps ghosts to ghosts. It starts at row
                 * realspeaker and at column realspeaker and goes to the end */
                let mut max: FixpDbl = 0; /* stores the greatest value */
                for i in rs..vsize {
                    for j in rs..vsize {
                        if adjacency_matrix[i][j] > max {
                            max = adjacency_matrix[i][j];
                        }
                    }
                }

                /* if the highest value is lower than 0.0001f we stop iteration */
                if max <= fl2fxconst_dbl(0.0001) {
                    break;
                }
            }

            /* copy adjacencyMatrix to downmixMatrix but cut the ghost rows off */
            for i in 0..rs {
                dmx[i].copy_from_slice(&adjacency_matrix[i]);
            }

            /* Power Normalize */
            for i in 0..rs {
                for j in rs..vsize {
                    /* start at realSpeaker position should work, because before this position we
                     * have a identity submatrix */
                    if dmx[i][j] != 0 && dmx[i][j] != MAXVAL_DBL {
                        /* calculate square root only for non trivial values */
                        dmx[i][j] = sqrt_fixp(dmx[i][j]);
                    }
                }
            }
            *downmix_matrix = Some(dmx);
        }

        if let Some(superset) = downmix_matrix_superset.as_ref() {
            /* multiply superset downmix matrix with imaginary speaker downmix matrix */
            let imaginaries = downmix_matrix.take().unwrap();
            if subset_speaker <= 0 {
                /*memory allocation error*/
                return 1;
            }
            let ss = subset_speaker as usize;
            let mut dmx = Matrix2D::<FixpDbl>::new(ss, vsize);
            *downmix_matrix_num_rows = subset_speaker;
            *downmix_matrix_num_cols = vl.size;

            for i in 0..ss {
                for j in 0..vsize {
                    let mut tmp: FixpDbl = 0;
                    for k in 0..rs {
                        tmp = tmp.wrapping_add(f_mult_div2(superset[i][k], imaginaries[k][j]));
                    }
                    dmx[i][j] = tmp;
                }
            }
            *downmix_matrix = Some(dmx);
        }
    }

    0
}

/******************************************
 *   Private Functions                     *
 ******************************************/

/// `resetVertexList(vL)`
pub fn reset_vertex_list(vl: &mut VertexList) {
    vl.size = 0;
    vl.limit = GVBAPRENDERER_VERTEX_LIST_MAX_SIZE as i32;
}

/// `addVertexToList(vL, v)`
pub fn add_vertex_to_list(vl: &mut VertexList, v: Vertex) {
    if vl.size as usize >= GVBAPRENDERER_VERTEX_LIST_MAX_SIZE {
        /* overflow of the C++ array */
        mpeghdec_fdk::deviation::hit();
    }
    vl.element[vl.size as usize] = v;
    vl.size += 1;
}

/// `resetTriangleList(tL)`
pub fn reset_triangle_list(tl: &mut TriangleList) {
    tl.size = 0;
    tl.limit = GVBAPRENDERER_TRIANGLE_LIST_MAX_SIZE as i32;
}

/// `addTriangleToList(tL, t)`
pub fn add_triangle_to_list(tl: &mut TriangleList, t: Triangle) {
    if tl.size as usize >= GVBAPRENDERER_TRIANGLE_LIST_MAX_SIZE {
        /* overflow of the C++ array */
        mpeghdec_fdk::deviation::hit();
    }
    tl.element[tl.size as usize] = t;
    tl.size += 1;
}

/// `add_vertex(vL, tL, vtx)`: adds the vertex `vtx` to the convex hull given by the triangles
/// `tl`. The plane orientation has to be counter clockwise and the result is guaranteed to be
/// counter clockwise as well. The given convex hull has to cover at least a hemisphere.
pub fn add_vertex(vl: &VertexList, tl: &mut TriangleList, vtx: i32) {
    let mut tl_new = [Triangle::default(); TRIANGLE_LIST_CAPACITY];
    let mut tl_new_size = 0usize;
    let mut tl_vis = [Triangle::default(); TRIANGLE_LIST_CAPACITY];
    let mut tl_vis_size = 0usize;

    let v = &vl.element;
    for i in 0..tl.size as usize {
        let [a, b, c] = tl.element[i].index;
        let distance = dist_point_plane(
            v[a as usize].xyz,
            v[b as usize].xyz,
            v[c as usize].xyz,
            v[vtx as usize].xyz,
        ); /* exponent is 1 */
        if distance > fl2fx_dbl_f64(-0.0001) {
            /* ref code value is -0.00001 */
            tl_vis[tl_vis_size] = tl.element[i];
            tl_vis_size += 1;
        } else {
            tl_new[tl_new_size] = tl.element[i];
            tl_new_size += 1;
        }
    }

    /* Extracts visible edges for a given set of triangles.
     * Essentially, all edges which occurs exactly once in the set of triangles are visible. */
    let mut edge_list = [0i32; TRIANGLE_LIST_CAPACITY * 3 * 2];

    /* create all possible edges */
    for i in 0..tl_vis_size {
        let t = tl_vis[i].index;
        edge_list[6 * i] = t[0];
        edge_list[6 * i + 1] = t[1];
        sort_edge_list(&mut edge_list[6 * i..]); /* Sort for easier comparability */
        edge_list[6 * i + 2] = t[0];
        edge_list[6 * i + 3] = t[2];
        sort_edge_list(&mut edge_list[6 * i + 2..]); /* Sort for easier comparability */
        edge_list[6 * i + 4] = t[1];
        edge_list[6 * i + 5] = t[2];
        sort_edge_list(&mut edge_list[6 * i + 4..]); /* Sort for easier comparability */
    }

    /* delete those which are doubles */
    let mut a = 0usize; /* counter for visible edges */
    for i in 0..tl_vis_size * 3 {
        if edge_list[2 * i] == -1 {
            /* old duplicate */
            continue;
        }

        let mut duplicate = false;
        for k in i + 1..tl_vis_size * 3 {
            if edge_list[2 * i] == edge_list[2 * k] && edge_list[2 * i + 1] == edge_list[2 * k + 1]
            {
                edge_list[2 * k] = -1; /* do not insert break. We have to mark all duplicates pairs */
                duplicate = true; /* duplicated element found. => no visible edge */
            }
        }
        if !duplicate {
            /* copy visible edges. Store it in existing list */
            edge_list[2 * a] = edge_list[2 * i];
            edge_list[2 * a + 1] = edge_list[2 * i + 1];
            a += 1; /* Count visible edges. This value is at same time the length of the new list */
        }
    }

    /* Delete entries of the old triangleList tL by setting size to new size and copy new entries */
    tl.size = tl_new_size as i32;
    /* Copy tLnew to tL */
    tl.element[..tl_new_size].copy_from_slice(&tl_new[..tl_new_size]);

    /* Add new values to triangleList tL */
    let origin = PointCartesian::default();
    for i in 0..a {
        /* check if flipping the orientation of the plane is necessary
         * It will ensure counter clockwise order of the vertices */
        let ea = edge_list[2 * i];
        let eb = edge_list[2 * i + 1];
        let distance =
            dist_point_plane(v[ea as usize].xyz, v[eb as usize].xyz, v[vtx as usize].xyz, origin); /* exponent don't care */
        let tmp = if distance > 0 {
            Triangle { index: [vtx, eb, ea] }
        } else {
            Triangle { index: [ea, eb, vtx] }
        };

        add_triangle_to_list(tl, tmp);
    }
}

/// `FL2FX_DBL(val)` evaluated for a `double` argument.
const fn fl2fx_dbl_f64(val: f64) -> FixpDbl {
    if val > 0.0 {
        (val * DFRACT_FIX_SCALE as f32 as f64 + 0.5) as i32
    } else {
        (val * DFRACT_FIX_SCALE as f32 as f64 - 0.5) as i32
    }
}

/// `sortEdgeList(edgeList)`: sorts the two indices of an edge in ascending order.
pub fn sort_edge_list(edge_list: &mut [i32]) {
    if edge_list[0] > edge_list[1] {
        edge_list.swap(0, 1);
    }
}

/// `distPointPlane(x1, x2, x3, p)`: distance between the point `p` and the plane spanned by
/// `x1`, `x2`, `x3` with sign as orientation. The result has an exponent of 1.
pub fn dist_point_plane(
    mut x1: PointCartesian,
    mut x2: PointCartesian,
    mut x3: PointCartesian,
    mut p: PointCartesian,
) -> FixpDbl {
    /* Prescale to avoid overflow of subtraction
     * If direction of the vectors x1x2, x1x3 are the same then the resulting normal
     * vector will be the same as without scaling.
     * x1 isn't a point on plane anymore. But we scaled p also,
     * from this it follows that (x-p)n is now 0.5(x-p)n (scaled with an exponent of 1) */
    x1.shr_assign(1);
    x2.shr_assign(1);
    x3.shr_assign(1);
    p.shr_assign(1);

    let op_a = x3.sub(&x1);
    let op_b = x2.sub(&x1);
    let mut n = cross_product(&op_a, &op_b); /* exponent doesen't really matter, because the direction is what we want */
    normalize(&mut n);
    let op_a = x1.sub(&p);
    let dist = dot_product(&op_a, &n); /* returns an exponent of 2. All together we have now a exponent of
                                       3 because of scaled x1 and p */

    /* Maximal possible dist in this context could be 2 (because we are in a sphere with radius 1.
     * Thus we normalize to a exponent of 1 with saturation */
    saturate_left_shift(dist, 2, DFRACT_BITS)
}

/// `init_vertex(azimuth, elevation)`: initializes a vertex including the sorting index.
pub fn init_vertex(azimuth: FixpDbl, elevation: FixpDbl) -> Vertex {
    let mut v = Vertex::default();
    v.sph.azi = azimuth;
    v.sph.ele = f_max(d2f(-90.0), f_min(d2f(90.0), elevation));
    v.sph.rad = fl2fxconst_dbl(1.0);

    /* Calculate Cartesian points from spherical points */
    v.xyz = spherical_to_cartesian(v.sph);

    let mut tmp = f_abs(1073741824 - f_abs(v.sph.azi)); // D2F(90.0)
    tmp += 5965232; /* Preparing for rounding */
    // D2F(0.5)
    let idx_azi = tmp / FX_DBL_DEGREE2INT; /* Calculate integer from fixpoint value
                                           11930465 corresponds to 1 degree */

    tmp = f_abs(v.sph.ele) + 5965232; /* Preparing for rounding */
    // D2F(0.5)
    let idx_ele = tmp / FX_DBL_DEGREE2INT; /* Calculate integer from fixpoint value
                                           11930465 corresponds to 1 degree */

    v.index = idx_azi + (181 * idx_ele); /* vertices on the median plane have lowest index */
    v
}

/// `add_imaginary_speakers(vL, pentagon)`: adds imaginary speakers by the 5 rules in the
/// definition and returns the indices of the initial pentagon. Returns an error code.
pub fn add_imaginary_speakers(vl: &mut VertexList, pentagon: &mut [i32; 5]) -> i32 {
    let mut highest_speaker = 0usize;
    let mut lowest_speaker = 0usize;

    let thresh = d2f(45.0);
    let max_angle = d2f(160.0);

    let number_of_real_speaker = vl.size.max(0) as usize;

    /* Rule 1 and 2 */

    /* Search for highest and lowest speaker in vertexList */
    for i in 0..number_of_real_speaker {
        if vl.element[i].sph.ele > vl.element[highest_speaker].sph.ele {
            highest_speaker = i;
        }
        if vl.element[i].sph.ele < vl.element[lowest_speaker].sph.ele {
            lowest_speaker = i;
        }
    }

    /* Adding Voice of God Ghost speaker if necessary */
    if vl.element[highest_speaker].sph.ele < thresh {
        highest_speaker = vl.size as usize;
        add_vertex_to_list(vl, init_vertex(0, d2f(90.0)));
    }
    /* Adding Voice of Hell Ghost speaker if necessary */
    if vl.element[lowest_speaker].sph.ele > -thresh {
        lowest_speaker = vl.size as usize;
        add_vertex_to_list(vl, init_vertex(0, d2f(-90.0)));
    }

    /* Rule 3 */
    if add_ghost_to_subset(vl, &RANGE_OF_SUBSET_A, true) == 0 {
        /* If no ghostspeaker was added */
        add_ghost_to_subset(vl, &RANGE_OF_SUBSET_B, true);
    }

    /* Rule 4 */
    if add_ghost_to_subset(vl, &RANGE_OF_SUBSET_C, false) == 0 {
        /* If no ghostspeaker was added */
        add_ghost_to_subset(vl, &RANGE_OF_SUBSET_D, false);
    }

    /* Rule 5 */
    /* copy real surround speaker into surroundList */
    let mut azi_surround_list: Vec<SurroundList> = Vec::with_capacity(number_of_real_speaker + 2);
    for i in 0..number_of_real_speaker {
        if f_abs(vl.element[i].sph.ele) < thresh {
            azi_surround_list.push(SurroundList { azi: vl.element[i].sph.azi, index: i as i32 });
        }
    }
    let cnt = azi_surround_list.len();
    if cnt == 0 {
        /* the C++ code writes out of bounds in this case */
        mpeghdec_fdk::deviation::hit();
        return 1;
    }
    /* sort surroundList by Azimuth */
    qsort(&mut azi_surround_list, azi_cmp);

    /* calculate angle differences between real surround speakers */
    let mut angle_diff_list = vec![0 as FixpDbl; cnt];
    for i in 1..cnt {
        /* Scale because angle between two speaker could be greater than 180 degrees */
        angle_diff_list[i - 1] =
            (azi_surround_list[i].azi >> 1) - (azi_surround_list[i - 1].azi >> 1);
    }

    angle_diff_list[cnt - 1] = (azi_surround_list[0].azi >> 1)
        .wrapping_sub(azi_surround_list[cnt - 1].azi >> 1)
        .wrapping_add(MAXVAL_DBL); // + D2F(180.0)

    /* if there are gaps greater 160 degrees fill them with ghost speakers */
    for i in 0..cnt {
        let mut sector = angle_diff_list[i];
        if sector >= max_angle {
            /* remember angle scaling: if angle difference is greater than 2 times max_angle
             * (320 degrees) */
            sector = f_mult(sector, fl2fxconst_dbl(2.0 / 3.0));
            for k in 0..2 {
                let azi = azi_surround_list[i].azi.wrapping_add(sector.wrapping_shl(k));
                azi_surround_list.push(SurroundList { index: vl.size, azi });
                add_vertex_to_list(vl, init_vertex(azi, 0));
            }
        } else if sector >= (max_angle >> 1) {
            /* if angle difference is greater than max_angle (160 degrees)*/
            let azi = azi_surround_list[i].azi.wrapping_add(sector);
            azi_surround_list.push(SurroundList { index: vl.size, azi });
            add_vertex_to_list(vl, init_vertex(azi, 0));
        }
    }

    /* sort surroundList by Azimuth again. Because the new added values are not sorted */
    qsort(&mut azi_surround_list, azi_cmp);

    /******************************************
     *   Find indices of initial pentagon     *
     ******************************************/

    pentagon[0] = highest_speaker as i32;
    pentagon[1] = lowest_speaker as i32;

    let mut i = 0usize;
    let mut k = 0usize;

    for cnt in 1..azi_surround_list.len() {
        /* if (aziSurroundList[i].azi <= aziSurroundList[0].azi + 160) */
        if (azi_surround_list[cnt].azi >> 1) <= ((azi_surround_list[0].azi >> 1) + d2f(80.0)) {
            i = cnt;
        }
        /* if (aziSurroundList[i].azi >= aziSurroundList[0].azi + 200) */
        if (azi_surround_list[cnt].azi >> 2) >= ((azi_surround_list[0].azi >> 2) + d2f(50.0)) {
            k = cnt;
            break;
        }
    }

    pentagon[2] = azi_surround_list[0].index;
    pentagon[3] = azi_surround_list[i].index;
    pentagon[4] = azi_surround_list[k].index;

    0
}

/// `add_imaginary_speakers_cicp(cicpIndex, vL, idx)`: adds the imaginary speakers of the table
/// of `cicp_index` (or falls back to [`add_imaginary_speakers`]). Returns an error code (the C++
/// function returns `void` and ignores the error of `add_imaginary_speakers()`).
pub fn add_imaginary_speakers_cicp(
    cicp_index: i32,
    vl: &mut VertexList,
    idx: &mut [i32; 5],
) -> i32 {
    let Some(h_table) = get_imag_speaker_table(cicp_index) else {
        return add_imaginary_speakers(vl, idx);
    };

    /* extend vertex list */
    for n in 0..h_table.num_ghost as usize {
        add_vertex_to_list(
            vl,
            init_vertex(
                h_table.azi_ghost[n].wrapping_mul(11930464),
                h_table.ele_ghost[n].wrapping_mul(11930464),
            ),
        );
    }

    /* set initial vertex list */
    idx.copy_from_slice(&h_table.initial_poly[..5]);
    0
}

/// `augment_setup_to_superset(cicpIdx, vL, downmix_mat)` (static): replaces the speakers of
/// `vl` by the loudspeaker superset of `cicp_idx` (if there is one) and returns the
/// superset-to-subset downmix matrix.
fn augment_setup_to_superset(
    cicp_idx: &mut i32,
    vl: &mut VertexList,
    downmix_mat: &mut Option<Matrix2D<FixpDbl>>,
) -> Cicp2GeometryError {
    let mut err_cicp = CICP2GEOMETRY_OK;
    let mut cicp_geometry =
        [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
    let mut num_channels = 0;
    let mut num_lfes = 0;

    if let Some(h_table) = get_speaker_super_set_table(*cicp_idx) {
        err_cicp = cicp2geometry_get_geometry_from_cicp(
            h_table.superset_cicp_index,
            Some(&mut cicp_geometry),
            &mut num_channels,
            &mut num_lfes,
        );
        if err_cicp == CICP2GEOMETRY_OK {
            *cicp_idx = h_table.superset_cicp_index;
            reset_vertex_list(vl);
            for g in cicp_geometry[..(num_channels + num_lfes) as usize].iter() {
                if g.lfe == 0 {
                    add_vertex_to_list(
                        vl,
                        init_vertex(g.az.wrapping_mul(11930464), g.el.wrapping_mul(11930464)),
                    );
                }
            }

            let (rows, cols) = (h_table.n_speakers_sub_set, h_table.n_speakers_super_set);
            if rows <= 0 || cols <= 0 {
                return CICP2GEOMETRY_ALLOC_ERROR;
            }
            let mut m = Matrix2D::new(rows as usize, cols as usize);
            for row in 0..rows as usize {
                for col in 0..cols as usize {
                    m[row][col] = h_table.down_mix_data[row * cols as usize + col];
                }
            }
            *downmix_mat = Some(m);
        }
    }

    err_cicp
}

/// `addGhostToSubset(vL, subsetRange, frontSide)`: tests whether a subset exists and whether it
/// is empty. If an empty subset was found, a ghost speaker is added. Returns 1 if a ghost speaker
/// was added, 0 otherwise.
pub fn add_ghost_to_subset(
    vl: &mut VertexList,
    subset_range: &[FixpDbl; 16],
    front_side: bool,
) -> i32 {
    let mut subset_index = [0usize; 4];
    let mut subset_found = [false; 4];
    let mut found_speaker = 0;
    let mut rel_corner_points = [0 as FixpDbl; 8];
    let mut found_point_in_polygon = false;
    let size = vl.size.max(0) as usize;

    /* Search speaker subset in vertexList regarding to given subsetRange */
    for i in 0..4 {
        for k in 0..size {
            let s = &vl.element[k].sph;
            if s.azi >= subset_range[i * 4]
                && s.azi <= subset_range[i * 4 + 1]
                && s.ele >= subset_range[i * 4 + 2]
                && s.ele <= subset_range[i * 4 + 3]
            {
                subset_index[i] = k;
                subset_found[i] = true;
                found_speaker += 1;
                if found_speaker == 4 {
                    break; /* make sure to find never more than 4 speaker */
                }
            }
        }
    }

    if found_speaker == 4 && subset_found.contains(&false) {
        /* the C++ code reads uninitialized entries of subsetIndex */
        mpeghdec_fdk::deviation::hit();
    }
    if found_speaker == 4 {
        /* complete subset was found */
        /* check whether subset is empty or not */
        for i in 0..size {
            /* go through all vertices */
            if subset_index.contains(&i) {
                /* don't check the corner points of the polygon itself */
                continue;
            }

            /* shift the polygon by the testing point. relCornerPoints are the relative
             * coordinates. */
            let t = vl.element[i].sph;
            for k in 0..4 {
                let c = vl.element[subset_index[k]].sph;
                if front_side {
                    /* shift one bit right to prevent overflow */
                    rel_corner_points[2 * k] = (c.azi >> 1) - (t.azi >> 1);
                } else {
                    /* testing speaker polygon is on back side. To cope with the azimuth wrap
                     * around at -180/180 we revert the orientation to front side */
                    let reverted_corner_point = c.azi ^ MINVAL_DBL;
                    let reverted_testing_point = t.azi ^ MINVAL_DBL;
                    /* shift one bit right to prevent overflow */
                    rel_corner_points[2 * k] =
                        (reverted_corner_point >> 1) - (reverted_testing_point >> 1);
                }
                /* shift one bit right to prevent overflow */
                rel_corner_points[2 * k + 1] = (c.ele >> 1) - (t.ele >> 1);
            }

            found_point_in_polygon = true;
            /* indicates the 'orientation' of the line defined by a polygonal boundary. If all
             * orientations are the same, then the point is inside the polygon. */
            for k in 0..4 {
                let next = (k + 1) % 4;
                /* shift multiplication results one bit right to prevent overflow */
                found_point_in_polygon = found_point_in_polygon
                    && (f_mult_div2(rel_corner_points[2 * next], rel_corner_points[2 * k + 1])
                        - f_mult_div2(rel_corner_points[2 * next + 1], rel_corner_points[2 * k]))
                        >= 0;
            }
            if found_point_in_polygon {
                break;
            }
        }

        if !found_point_in_polygon {
            /* Subset is empty. We have to add a Ghost speaker */
            let mut mean_azi: FixpDbl = 0;
            let mut mean_ele: FixpDbl = 0;

            for &si in subset_index.iter() {
                mean_azi += vl.element[si].sph.azi >> 2; /* Divide by 4 */
                mean_ele += vl.element[si].sph.ele >> 2; /* Divide by 4 */
            }
            if !front_side {
                mean_azi = mean_azi.wrapping_sub(MAXVAL_DBL); //- D2F(180.0)
            }

            add_vertex_to_list(vl, init_vertex(mean_azi, mean_ele));

            return 1;
        }
    }

    0
}
