// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libgVBAPRenderer/include/vbap_core.h and libgVBAPRenderer/src/vbap_core.cpp.
// FUNCTION_calculateOneSourcePosition_func1 and FUNCTION_dotProductXYZ are not defined in the
// reference configuration, so the generic C code is ported.

//! Vector Base Amplitude Panning (VBAP) core: renderer state, inverse speaker matrices and the
//! VBAP gain calculation (including the legacy MDAP spread with 19 base vectors).

use mpeghdec_fdk::digest::StateDigest;

use crate::cartesian_math::*;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_div_norm, inv_fixp_e, inv_sqrt_norm2};
use mpeghdec_fdk::matrix_calloc::Matrix2D;
use mpeghdec_fdk::scale::scale_value_saturate;
use mpeghdec_fdk::trig_fcts::fixp_cos_sin;

/// `VBAP_PCM`
pub type VbapPcm = FixpDbl;

pub const GVBAPRENDERER_MAX_OBJECTS: i32 = 32;
pub const GVBAPRENDERER_MAX_CHANNEL_OUT: i32 = 24;
pub const GVBAPRENDERER_MAX_GHOST_SPEAKER: i32 = 6;
pub const GVBAPRENDERER_MAX_OAM_FRAMES_PER_CORE_FRAME: usize = 16;
pub const GVBAPRENDERER_MAX_FRAMELENGTH: i32 = 1024;

pub const GVBAP_LEGACY: i32 = 0;
pub const GVBAP_ENHANCED: i32 = 1;

pub const GVBAP_SPREAD_NUM_VSO_AZI: usize = 12; /* 360 / 30 */
pub const GVBAP_SPREAD_NUM_VSO_ELE: usize = 7; /* 180 / 30 + 1 */

pub const GVBAP_SPREAD_NUM_VSO: usize =
    GVBAP_SPREAD_NUM_VSO_AZI * (GVBAP_SPREAD_NUM_VSO_ELE - 2) + 2;

/* OAM definitions */
pub const OAM_NUMBER_COMPONENTS: u8 = 5; /* number of OAM components */

pub const OAM_BITS_AZI: i32 = 8; /* number of quantization bits for the azimuth angle */
pub const OAM_BITS_ELE: i32 = 6; /* number of quantization bits for the elevation angle */
pub const OAM_BITS_RAD: i32 = 4; /* number of quantization bits for the radius (logarithmic) */
pub const OAM_BITS_GAIN: i32 = 7; /* number of quantization bits for the object gain */
pub const OAM_BITS_SPREAD: i32 = 7; /* number of quantization bits for the spread angle */
pub const OAM_BITS_SPREAD_HEIGHT: i32 = 5; /* number of quantization bits for the spread angle */
pub const OAM_BITS_SPREAD_DEPTH: i32 = 4; /* number of quantization bits for the spread angle */
pub const OAM_BITS_OBJ_PRIORITY: i32 = 3; /* number of quantization bits for the object priority */

/// `OAM_COMPONENT`
pub type OamComponent = i32;
pub const AZIMUTH: OamComponent = 0;
pub const ELEVATION: OamComponent = 1;
pub const RADIUS: OamComponent = 2;
pub const GAIN: OamComponent = 3;
pub const SPREAD: OamComponent = 4;
pub const DYNAMIC_OBJECT_PRIORITY: OamComponent = 5;
pub const SPREAD_HEIGHT: OamComponent = 6;
pub const SPREAD_DEPTH: OamComponent = 7;
/// maximal number of OAM components
pub const OAM_NUMBER_MAX_COMPONENTS: OamComponent = 8;

pub const MIN_GAIN: i32 = -42949672; /* -0.01f */

/// `OAM_SAMPLE`: a single object metadata sample.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OamSample {
    /// `sph`
    pub sph: PointSpherical,
    /// `cart`
    pub cart: PointCartesian,
    /// `gain`: linear gain value. Range 0.004 ... 5.957, exponent = 3.
    pub gain: FixpDbl,
    /// `spreadAngle`: spread angle or spread width in radian. Range 0 ... 180 mapped to 0 ... 1.0.
    pub spread_angle: FixpDbl,
    /// `spreadHeight`: spread height in radian. Range 0 ... 90 mapped to 0 ... 0.5.
    pub spread_height: FixpDbl,
    /// `spreadDepth`: spread depth. Range 0 ... 15.5, exponent = 4.
    pub spread_depth: FixpDbl,
    /// `goa_bsObjectDistance`
    pub goa_bs_object_distance: u16,
}

/// `SPEAKER`: all relevant information for one speaker.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Speaker {
    /// `sph`: spherical speaker coordinates
    pub sph: PointSpherical,
    /// `isLFE`
    pub is_lfe: bool,
    /// `isGhost`
    pub is_ghost: bool,
}

/// `SPEAKERTRIPLET`: speaker triangle with the matrix of the Cartesian speaker coordinates (one
/// speaker per row) and its inverse.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SpeakerTriplet {
    /// `triangle`
    pub triangle: [i32; 3],
    /// `matrix`
    pub matrix: [PointCartesian; 3],
    /// `inverseMatrix`
    pub inverse_matrix: [PointCartesian; 3],
    /// `exponentInverseMatrix`
    pub exponent_inverse_matrix: i32,
}

/// `SPEAKERSETUP`: all required information about the speaker setup.
#[derive(StateDigest, Clone, Debug, Default)]
pub struct SpeakerSetup {
    /// `speakerList`: all real and LFE speakers (non-LFE speakers first)
    pub speaker_list: Vec<Speaker>,
    /// `speakerTriplet`
    pub speaker_triplet: Vec<SpeakerTriplet>,
    /// `greatestInverseMatrixExponent`
    pub greatest_inverse_matrix_exponent: i32,
    /// `speakerListSize`
    pub speaker_list_size: i32,
    /// `speakerTripletSize`
    pub speaker_triplet_size: i32,
    /// `mapping`: output channel index of each entry of `speaker_list`
    pub mapping: Vec<i32>,
    /// `coversUpperHalfSphere`: speakers between 5 and 21 degrees elevation
    pub covers_upper_half_sphere: bool,
    /// `coversLowerHalfSphere`: speakers below -5 degrees elevation
    pub covers_lower_half_sphere: bool,
    /// `hasHeightSpeakers`: speakers above 21 degrees elevation
    pub has_height_speakers: bool,
}

/// `GVBAPRENDERER` (`HANDLE_GVBAPRENDERER` is `&mut GVbapRenderer`): speaker setup, OAM data,
/// start/end gains and interpolation state of one object renderer instance.
#[derive(StateDigest, Clone, Debug)]
pub struct GVbapRenderer {
    /// `baseVectors` (legacy spread, 19 + 1 entries)
    pub base_vectors: [PointCartesian; 19 + 1],

    /// `speakerSetup`
    pub speaker_setup: SpeakerSetup,

    /// `oamSamples`: Dim: `[OAM frames][objects]`
    pub oam_samples: Vec<Vec<OamSample>>,

    /// `metadataPresent`: Dim: [OAM frames]
    pub metadata_present: [u8; GVBAPRENDERER_MAX_OAM_FRAMES_PER_CORE_FRAME],

    /// `hasUniformSpread`
    pub has_uniform_spread: u8,

    /// `oamDataValid`
    pub oam_data_valid: u8,
    /// `metadataPresentValid`: Dim: [OAM frames]. Used for concealment.
    pub metadata_present_valid: [u8; GVBAPRENDERER_MAX_OAM_FRAMES_PER_CORE_FRAME],
    /// `oamSamplesValid`: Dim: `[OAM frames][objects]`. Used for concealment.
    pub oam_samples_valid: Vec<Vec<OamSample>>,

    /// `fixed_val`
    pub fixed_val: [u8; OAM_NUMBER_MAX_COMPONENTS as usize],
    /// `OAM_parsed_data`: Dim: [objects * OAM_NUMBER_MAX_COMPONENTS]
    pub oam_parsed_data: Vec<i32>,

    /// `startGainsMax`
    pub start_gains_max: Vec<FixpDbl>,
    /// `prevGainsMax`
    pub prev_gains_max: Vec<FixpDbl>,

    /// `startGains`: Dim: [objects][output channels]
    pub start_gains: Matrix2D<FixpDbl>,
    /// `endGains`: Dim: [objects][output channels]
    pub end_gains: Matrix2D<FixpDbl>,

    /// `stepState`: Dim: [objects][output channels]
    pub step_state: Matrix2D<FixpDbl>,
    /// `scaleState`: Dim: [objects][output channels]
    pub scale_state: Matrix2D<FixpDbl>,

    /// `gainCache`: Dim: [real channels without LFE but with ghost speakers]
    pub gain_cache: Vec<FixpDbl>,

    /// `downmixMatrix`: Dim: [real speakers without LFE][real speakers without LFE + ghosts],
    /// `None` for a NULL pointer
    pub downmix_matrix: Option<Matrix2D<FixpDbl>>,
    /// `downmixMatrixNumRows`
    pub downmix_matrix_num_rows: i32,
    /// `downmixMatrixNumCols`
    pub downmix_matrix_num_cols: i32,

    /// `numChannels`: number of real speakers including LFE
    pub num_channels: i8,
    /// `numLFE`
    pub num_lfe: i8,
    /// `numGhosts`
    pub num_ghosts: i8,
    /// `numObjects`
    pub num_objects: i8,
    /// `frameLength`
    pub frame_length: i16,
    /// `oamFrameLength`
    pub oam_frame_length: i16,
    /// `numOamFrames`
    pub num_oam_frames: i8,
    /// `gainCacheLength`
    pub gain_cache_length: i8,

    /// `startGainsFilled`
    pub start_gains_filled: u8,

    /// `spread_gainsVSO`: vbap-panned gains of virtual spread objects (empty vectors for
    /// NULL pointers)
    pub spread_gains_vso: Vec<Vec<FixpDbl>>,
    /// `spread_gainsSpread`: spread gains for virtual spread objects
    pub spread_gains_spread: [FixpDbl; GVBAP_SPREAD_NUM_VSO],
    /// `spread_gainArray`
    pub spread_gain_array: Vec<FixpDbl>,
    /// `ghostVoiceOfGodSpeakerIndex`
    pub ghost_voice_of_god_speaker_index: i8,
    /// `ghostVoiceOfHellSpeakerIndex`
    pub ghost_voice_of_hell_speaker_index: i8,
    /// `renderMode`: 0: original rendering (MPEG standardized), 1: alternative (proprietary)
    /// rendering (new spread rendering, new imaginary speaker initialization)
    pub render_mode: u8,
}

impl Default for GVbapRenderer {
    /// All fields zero like the `FDKcalloc()`ed handle.
    fn default() -> Self {
        GVbapRenderer {
            base_vectors: Default::default(),
            speaker_setup: Default::default(),
            oam_samples: Vec::new(),
            metadata_present: [0; GVBAPRENDERER_MAX_OAM_FRAMES_PER_CORE_FRAME],
            has_uniform_spread: 0,
            oam_data_valid: 0,
            metadata_present_valid: [0; GVBAPRENDERER_MAX_OAM_FRAMES_PER_CORE_FRAME],
            oam_samples_valid: Vec::new(),
            fixed_val: [0; OAM_NUMBER_MAX_COMPONENTS as usize],
            oam_parsed_data: Vec::new(),
            start_gains_max: Vec::new(),
            prev_gains_max: Vec::new(),
            start_gains: Default::default(),
            end_gains: Default::default(),
            step_state: Default::default(),
            scale_state: Default::default(),
            gain_cache: Vec::new(),
            downmix_matrix: None,
            downmix_matrix_num_rows: 0,
            downmix_matrix_num_cols: 0,
            num_channels: 0,
            num_lfe: 0,
            num_ghosts: 0,
            num_objects: 0,
            frame_length: 0,
            oam_frame_length: 0,
            num_oam_frames: 0,
            gain_cache_length: 0,
            start_gains_filled: 0,
            spread_gains_vso: Vec::new(),
            spread_gains_spread: [0; GVBAP_SPREAD_NUM_VSO],
            spread_gain_array: Vec::new(),
            ghost_voice_of_god_speaker_index: 0,
            ghost_voice_of_hell_speaker_index: 0,
            render_mode: 0,
        }
    }
}

/// `generateInverseMatrices(HANDLE_GVBAPRENDERER)`: uses the Cramer rule to calculate the
/// inverse of the speaker matrices and stores the results in transposed order.
pub fn generate_inverse_matrices(h: &mut GVbapRenderer) -> i32 {
    let size = h.speaker_setup.speaker_triplet_size.max(0) as usize;
    for speaker_triplet in h.speaker_setup.speaker_triplet[..size].iter_mut() {
        let matrix = speaker_triplet.matrix;

        /* First calculate det(M) */
        /* additive part */
        let mut tmp = f_mult_div2(matrix[0].x, matrix[1].y); /* tmp  has an exponent of 1 */
        let mut det_m = f_mult_div2(tmp, matrix[2].z); /* detM has an exponent of 2 */
        tmp = f_mult_div2(matrix[0].y, matrix[1].z);
        det_m = f_mult_add_div2(det_m, tmp, matrix[2].x); /* detM could be 2 at most -> exponent of 2 prevents overflow */
        tmp = f_mult_div2(matrix[0].z, matrix[1].x);
        det_m = f_mult_add_div2(det_m, tmp, matrix[2].y); /* detM could be 3 at most -> exponent of 2 prevents overflow */

        /* subtractive part, add first, subtract later from detM */
        tmp = f_mult_div2(matrix[0].z, matrix[1].y);
        let mut det_s = f_mult_div2(tmp, matrix[2].x);
        tmp = f_mult_div2(matrix[0].y, matrix[1].x);
        det_s = f_mult_add_div2(det_s, tmp, matrix[2].z);
        tmp = f_mult_div2(matrix[0].x, matrix[1].z);
        det_s = f_mult_add_div2(det_s, tmp, matrix[2].y);
        det_m = (det_m >> 1) - (det_s >> 1); /* detM could be 6 at most -> exponent of 3 prevents overflow */

        /* Calculate inverse of detM */
        let mut det_m_inv_exponent = 3; /* 3 because detM has an exponent of 3 */
        let det_m_inv = inv_fixp_e(det_m, &mut det_m_inv_exponent);

        /* Calculate Det(A_ij) */
        let sub = |a: FixpDbl, b: FixpDbl, c: FixpDbl, d: FixpDbl| {
            f_mult(f_mult_sub_div2(f_mult_div2(a, b), c, d), det_m_inv)
        };
        let inv = &mut speaker_triplet.inverse_matrix;
        inv[0].x = sub(matrix[1].y, matrix[2].z, matrix[1].z, matrix[2].y);
        inv[1].x = sub(matrix[0].z, matrix[2].y, matrix[0].y, matrix[2].z);
        inv[2].x = sub(matrix[0].y, matrix[1].z, matrix[0].z, matrix[1].y);
        inv[0].y = sub(matrix[1].z, matrix[2].x, matrix[1].x, matrix[2].z);
        inv[1].y = sub(matrix[0].x, matrix[2].z, matrix[0].z, matrix[2].x);
        inv[2].y = sub(matrix[0].z, matrix[1].x, matrix[0].x, matrix[1].z);
        inv[0].z = sub(matrix[1].x, matrix[2].y, matrix[1].y, matrix[2].x);
        inv[1].z = sub(matrix[0].y, matrix[2].x, matrix[0].x, matrix[2].y);
        inv[2].z = sub(matrix[0].x, matrix[1].y, matrix[0].y, matrix[1].x);

        det_m_inv_exponent += 1; /* Add 1 because all sub determinants has exponents of 1 */
        /* Matrix exponent mustn't be negative */
        if det_m_inv_exponent < 0 {
            /* normalize exponent to 0 */
            let sh = det_m_inv_exponent.wrapping_neg() as u32;
            for p in inv.iter_mut() {
                p.x = p.x.wrapping_shr(sh);
                p.y = p.y.wrapping_shr(sh);
                p.z = p.z.wrapping_shr(sh);
            }
        }
        speaker_triplet.exponent_inverse_matrix = det_m_inv_exponent;
    }

    /* search for the biggest exponent over all matrices */
    /* (an empty triangulation of a degenerate layout reads uninitialized memory in C++) */
    let mut greatest_exponent =
        h.speaker_setup.speaker_triplet.first().map_or(0, |t| t.exponent_inverse_matrix);
    for t in h.speaker_setup.speaker_triplet[..size].iter().skip(1) {
        if t.exponent_inverse_matrix > greatest_exponent {
            greatest_exponent = t.exponent_inverse_matrix;
        }
    }
    h.speaker_setup.greatest_inverse_matrix_exponent = greatest_exponent;

    0
}

/// `calculateVbap(hgVBAPRenderer, source, final_gains, noDownmix)`: calculates the gains for
/// all speakers including ghosts and maps the ghost speakers to real ones if a downmix matrix
/// exists (and `no_downmix == 0`). The gains are power normalized and include `source.gain`.
///
/// `final_gains` receives `numChannels - numLFE` gains (downmix) or
/// `numChannels - numLFE + numGhosts` gains (no downmix).
pub fn calculate_vbap(
    h: &mut GVbapRenderer,
    source: &OamSample,
    final_gains: &mut [FixpDbl],
    no_downmix: i32,
) {
    let mut gain_norm: FixpDbl = 0;
    let mut gain_norm_inv_exp = 0;

    let mut num_speaker = h.num_channels as i32 - h.num_lfe as i32;

    /* first calculate gains for all speakers including ghosts. Store the result to gain cache */
    calculate_vbap_gain(h, source);

    /* Check whether a downmix matrix for ghost speaker exists or not */
    if let Some(dmx) = h.downmix_matrix.as_ref().filter(|_| no_downmix == 0) {
        let n = num_speaker.max(0) as usize;
        final_gains[..n].fill(0);

        /* Downmix ghost speakers */
        let col_runs = (h.num_channels as i32 - h.num_lfe as i32 + h.num_ghosts as i32).max(0);
        for row in 0..n {
            let mut tmp_gains: FixpDbl = 0;
            let dmx_row = &dmx[row];
            for col in 0..col_runs as usize {
                tmp_gains = f_mult_add_div2(tmp_gains, dmx_row[col], h.gain_cache[col]);
            }
            final_gains[row] = tmp_gains;
        }
    } else {
        num_speaker = h.num_channels as i32 - h.num_lfe as i32 + h.num_ghosts as i32;
        let n = num_speaker.max(0) as usize;
        final_gains[..n].copy_from_slice(&h.gain_cache[..n]);
    }

    let n = num_speaker.max(0) as usize;
    /* gain correction is necessary because adding power normalized gains through downmix */
    for &g in final_gains[..n].iter() {
        gain_norm = f_pow2_add(gain_norm, g >> 2); /* this will limit the number of speakers to 64 */
    }

    let mut gain_norm_inv = inv_sqrt_norm2(gain_norm, &mut gain_norm_inv_exp);
    gain_norm_inv = f_mult(gain_norm_inv, source.gain); /* Incorporate gainFactor */

    /* gain_norm should always be greater than each final_gains[i]
     * but if there is only one gain value in final_gains vector different from 0
     * it could be happen that gain_norm is a little smaller than final_gain[i] because of
     * arithmetic inaccuracy
     */
    for g in final_gains[..n].iter_mut() {
        let tmp = f_mult(*g >> 2, gain_norm_inv);
        *g = scale_value_saturate(tmp, gain_norm_inv_exp);
    }
}

/// `calculateVbapGain(hgVBAPRenderer, source)`: calculates the gains for the given object
/// position into the gain cache, including the OAM spread angle (legacy mode).
pub fn calculate_vbap_gain(h: &mut GVbapRenderer, source: &OamSample) {
    /* clear gain cache in handler */
    let n = (h.num_channels as i32 + h.num_ghosts as i32 - h.num_lfe as i32).max(0) as usize;
    h.gain_cache[..n].fill(0);

    if h.render_mode as i32 == GVBAP_LEGACY {
        /* calculate spread vectors (0 - 90 degrees) */
        calc_spread_vectors(h, source);

        /* calculate gains for all spread vectors */
        calculate_one_source_position(h, None);

        /* calculate second part spreading (90 - 180 degrees), include gain normalization */
        /* resulting gains has an exponent of 2 */
        calc_spread_gains(h, source.spread_angle);
    } else {
        calculate_one_source_position(h, Some(&source.cart));

        normalize_power(h);
    }
}

/// `calcSpreadVectors(hgVBAPRenderer, source)`: calculates the 19 spread base vectors.
pub fn calc_spread_vectors(h: &mut GVbapRenderer, source: &OamSample) {
    let zero = PointCartesian::default();

    let mut exp = 0;

    /* Limit spread Angle to 0.001 <= alpha <= 90 degrees */
    /* "source.spreadAngle" is in percent. We calculate: round((0.001 deg/180 deg)*2^31) = 11931*/
    let mut spread_angle = f_max(source.spread_angle, 11931);
    /* "spreadAngle" is in percent. We calculate: round((90deg/180deg)*2^31) = 1073741824*/
    spread_angle = f_min(spread_angle, 1073741824);

    let mut p0 = source.cart;
    /* Rotate p0 vector by 90 degrees in elevation direction */
    let direction = PointSpherical {
        ele: if source.sph.ele < 0 {
            source.sph.ele.wrapping_add(1073741824) /* Add 90 degrees */
        } else {
            source.sph.ele.wrapping_sub(1073741824) /* Subtract 90 degrees */
        },
        azi: source.sph.azi,
        rad: fl2fxconst_dbl(1.0),
    };

    let mut v = spherical_to_cartesian(direction);
    let mut u = cross_product(&v, &source.cart);

    /* u is already scaled with an exponent of 1. This scale should not be undone
     * because baseVectors 7, 10, 13, 16 could overflow */

    /*Store p0 in full precision with a relative exponent (to p0) for subsequent calculations */
    let p1 = p0;
    let mut p1_relative_exp = 1;
    if h.has_uniform_spread != 0 {
        /* also scale v and p0 to the half */
        v.shr_assign(1);
        p0.shr_assign(1);
    } else {
        /* With spreadAngle = spreadWidth determine the ratio
        alpha_r = az_el_spread_ratio * 2 ^ az_el_spread_ratio_e = spreadHeight / spreadAngle. The
        base vector v is multiplied with alpha_r. Furthermore, scale u and p0 as needed: The exponent
        of u, v and p0 can be set to any equal value so that it cancels out in the division later.*/

        /* Limit spreadHeight to 0.001 <= alpha <= 90 degrees */
        let mut spread_height = f_max(source.spread_height, 11931);
        spread_height = f_min(spread_height, 1073741824);

        if spread_height != spread_angle && h.speaker_setup.has_height_speakers {
            let mut az_el_spread_ratio_e = 0;
            let az_el_spread_ratio =
                f_div_norm(spread_height, spread_angle, &mut az_el_spread_ratio_e);
            // we assert the divison returns an exponent in the range -(DFRACT_BITS - 1) <=
            // az_el_spread_ratio_e <= (DFRACT_BITS - 1)
            v = v.scale(az_el_spread_ratio);
            if az_el_spread_ratio_e > 0 {
                u.shr_assign(az_el_spread_ratio_e - 1);
                p0.shr_assign(az_el_spread_ratio_e);
                p1_relative_exp = az_el_spread_ratio_e;
            } else {
                v.shr_assign(f_min(DFRACT_BITS - 1, 1 - az_el_spread_ratio_e));
                p0.shr_assign(1);
            }
        } else {
            v.shr_assign(1);
            p0.shr_assign(1);
        }
    }

    if !h.speaker_setup.has_height_speakers {
        v = PointCartesian::default();
    }

    let bv = &mut h.base_vectors;
    bv[0] = p0;
    bv[1] = u;
    bv[4] = zero.sub(&u);

    /* 0.75f * u + 0.25f * p0 */
    bv[2] = u.scale(1610612736).add(&p0.scale(536870912));
    /* -0.75f * u + 0.25f * p0 */
    bv[5] = u.scale(-1610612736).add(&p0.scale(536870912));

    /* 0.375f * u + 0.625f * p0 */
    bv[3] = u.scale(805306368).add(&p0.scale(1342177280));
    /* -0.375f * u + 0.625f * p0 */
    bv[6] = u.scale(-805306368).add(&p0.scale(1342177280));

    /*  0.5f * u + 0.866f * v + 0.333f * p0 */
    bv[7] = u.scale(1073741824).add(&v.scale(1859720839)).add(&p0.scale(715112054));
    /* -0.5f * u + 0.866f * v + 0.333f * p0 */
    bv[10] = u.scale(-1073741824).add(&v.scale(1859720839)).add(&p0.scale(715112054));
    /* -0.5f * u - 0.866f * v + 0.333f * p0 */
    bv[13] = u.scale(-1073741824).sub(&v.scale(1859720839)).add(&p0.scale(715112054));
    /*  0.5f * u - 0.866f * v + 0.333f * p0 */
    bv[16] = u.scale(1073741824).sub(&v.scale(1859720839)).add(&p0.scale(715112054));

    /*  0.5f * p7  + 0.5f * p0 */
    bv[8] = bv[7].scale(1073741824).add(&p0.scale(1073741824));
    bv[11] = bv[10].scale(1073741824).add(&p0.scale(1073741824));
    bv[14] = bv[13].scale(1073741824).add(&p0.scale(1073741824));
    bv[17] = bv[16].scale(1073741824).add(&p0.scale(1073741824));

    /*  0.25f * p7  + 0.75f * p0 */
    bv[9] = bv[7].scale(536870912).add(&p0.scale(1610612736));
    bv[12] = bv[10].scale(536870912).add(&p0.scale(1610612736));
    bv[15] = bv[13].scale(536870912).add(&p0.scale(1610612736));
    bv[18] = bv[16].scale(536870912).add(&p0.scale(1610612736));

    /* Calculate tan(spreadangle) use cos sin for calculation */
    /* Scale spreadAngle from -pi ... pi to a power of 2 */
    /* New exponent will be 2 */
    spread_angle = f_mult(spread_angle, 1686629713i32); /* Multiply by pi/4 */

    let (cos_spread, sin_spread) = fixp_cos_sin(spread_angle, 2);
    let inv_sin_spread = inv_fixp_e(sin_spread, &mut exp);

    let mut summand = p1.scale(f_mult_div2(cos_spread, inv_sin_spread));
    let mut result_exponent_max = exp + 1 - p1_relative_exp;

    if result_exponent_max >= 0 {
        result_exponent_max = f_min(result_exponent_max, 31);
    } else {
        result_exponent_max = f_min(-result_exponent_max, 31);

        summand.x >>= result_exponent_max;
        summand.y >>= result_exponent_max;
        summand.z >>= result_exponent_max;

        result_exponent_max = 0;
    }

    for b in bv[..19].iter_mut() {
        b.x = (b.x >> result_exponent_max).wrapping_add(summand.x);
        b.y = (b.y >> result_exponent_max).wrapping_add(summand.y);
        b.z = (b.z >> result_exponent_max).wrapping_add(summand.z);
    }
}

/// `calculateOneSourcePosition(hgVBAPRenderer, baseVector)`: finds the triangle into which each
/// base vector points (legacy: the 19 spread base vectors, `base_vector == None`; enhanced: only
/// `base_vector`) and adds the three speaker gains to the gain cache.
pub fn calculate_one_source_position(h: &mut GVbapRenderer, base_vector: Option<&PointCartesian>) {
    let inv_mat_max_exp = h.speaker_setup.greatest_inverse_matrix_exponent;
    let legacy = h.render_mode as i32 == GVBAP_LEGACY;

    /* iterate over all spread (base) vectors */
    for i in 0..19 {
        let mut winner_set = 0usize;
        let a = if legacy { h.base_vectors[i] } else { *base_vector.unwrap() };

        let mut gain = PointCartesian::default();
        let mut greatest_smallest_gain = MINVAL_DBL; /* set to lowest possible value */

        /* iterate over all triplets */
        let size = h.speaker_setup.speaker_triplet_size.max(0) as usize;
        for (j, t) in h.speaker_setup.speaker_triplet[..size].iter().enumerate() {
            let bx = &t.inverse_matrix[0];
            let by = &t.inverse_matrix[1];
            let bz = &t.inverse_matrix[2];
            let gain_tmp = PointCartesian {
                x: (f_mult(a.x, bx.x) >> 2) + (f_mult(a.y, bx.y) >> 2) + (f_mult(a.z, bx.z) >> 2),
                y: (f_mult(a.x, by.x) >> 2) + (f_mult(a.y, by.y) >> 2) + (f_mult(a.z, by.z) >> 2),
                z: (f_mult(a.x, bz.x) >> 2) + (f_mult(a.y, bz.y) >> 2) + (f_mult(a.z, bz.z) >> 2),
            };
            /* search the smallest gain value of the three gains */
            let mut smallest_gain_of_set = f_min(f_min(gain_tmp.x, gain_tmp.y), gain_tmp.z);

            smallest_gain_of_set = smallest_gain_of_set
                .wrapping_shr((inv_mat_max_exp - t.exponent_inverse_matrix) as u32);

            /* find the best possible set */
            if smallest_gain_of_set > greatest_smallest_gain {
                winner_set = j;
                gain = gain_tmp;
                greatest_smallest_gain = smallest_gain_of_set;
            }
        } /* j */

        /* If there are negative gains then set them to 0 */
        gain.x = f_max(gain.x, 0);
        gain.y = f_max(gain.y, 0);
        gain.z = f_max(gain.z, 0);
        normalize(&mut gain);

        /* Add gains to cache */
        /* In worst case one speaker gets the whole energy. That means the value could be 19 at
         * most, because of the 19 base vectors */
        /* To prevent overflow scale wit an exponent of 5 (divide by 32) */
        /* An empty triangulation of a degenerate layout reads uninitialized memory in C++ (and
         * adds zero gains there). */
        let Some(t) = h.speaker_setup.speaker_triplet.get(winner_set) else {
            mpeghdec_fdk::deviation::hit();
            if !legacy {
                break;
            }
            continue;
        };
        let tri = t.triangle;
        let gc = &mut h.gain_cache;
        gc[tri[0] as usize] = gc[tri[0] as usize].wrapping_add(gain.x >> 5);
        gc[tri[1] as usize] = gc[tri[1] as usize].wrapping_add(gain.y >> 5);
        gc[tri[2] as usize] = gc[tri[2] as usize].wrapping_add(gain.z >> 5);

        if !legacy {
            break;
        }
    } /* i */
}

/// `calcSpreadGains(hgVBAPRenderer, spreadAngle)`: second part of the legacy spreading (90 - 180
/// degrees) including gain normalization.
pub fn calc_spread_gains(h: &mut GVbapRenderer, spread_angle: FixpDbl) {
    let w: FixpDbl;
    let w_1: FixpDbl;
    let mut gain_norm: FixpDbl = 0;
    let mut gain_norm_inv: FixpDbl;
    let mut gain_norm_inv_exp = 0;
    let mut unit_gain: FixpDbl;
    let len = h.gain_cache_length.max(0) as usize;

    if spread_angle > 0x40000000 {
        /* if greater than 90 degrees */
        let w0 = spread_angle - 0x40000000; /* w = spreadAngle - 90 degrees */
        w = w0 << 1;
        w_1 = MAXVAL_DBL - w;

        /* Calculate one element of a virtual unit gain vector (all elements are equal) */
        unit_gain = MAXVAL_DBL >> 8; /* The value of 8 means, that are not more than 256
                                     speakers including ghosts are allowed */
        unit_gain = unit_gain.wrapping_mul(h.gain_cache_length as i32);
        gain_norm_inv = inv_sqrt_norm2(unit_gain, &mut gain_norm_inv_exp);
        unit_gain = f_mult(w >> 4, gain_norm_inv).wrapping_shl(gain_norm_inv_exp as u32);
    /* results exponent is 4-4 = 0 */
    } else {
        w = 0;
        w_1 = MAXVAL_DBL;
        unit_gain = 0;
    }

    /* Adding all gains in gaincache array never results in a higher value than 19/32 see code line
     * 418 */
    for &g in h.gain_cache[..len].iter() {
        gain_norm = f_pow2_add(gain_norm, g);
    }

    /* Normalize gains */
    gain_norm_inv = inv_sqrt_norm2(gain_norm, &mut gain_norm_inv_exp);

    /* gain_norm should always be greater than each final_gains[i]
     * but if there is only one gain value in final_gains vector different from 0
     * it could be happen that gain_norm is a little smaller than final_gain[i] because of
     * arithmetic inaccuracy
     */
    let mut gain_norm_inv_w_1 = gain_norm_inv;
    if w != 0 {
        gain_norm_inv_w_1 = f_mult(gain_norm_inv, w_1); /* include factor for g_MDAP' */
    }
    for g in h.gain_cache[..len].iter_mut() {
        let mut tmp = f_mult(*g, gain_norm_inv_w_1);
        tmp = scale_value_saturate(tmp, gain_norm_inv_exp - 1);
        *g = tmp.wrapping_add(unit_gain >> 1);
    }

    /* If spread angle was over 90 degrees then power normalize again */
    if w != 0 {
        gain_norm = f_pow2(unit_gain) >> 8;
        let width_shift = f_normz(h.gain_cache_length as i16) - 1;
        let temp = ((h.gain_cache_length as i16 as i32) << width_shift) as i16;
        gain_norm = gain_norm.wrapping_shl((15 - width_shift) as u32);
        gain_norm = f_mult(gain_norm, temp);
        gain_norm = gain_norm.wrapping_add(w_1 >> 8);
        gain_norm_inv = inv_sqrt_norm2(gain_norm, &mut gain_norm_inv_exp);
        for g in h.gain_cache[..len].iter_mut() {
            let tmp = *g >> 4;
            *g = f_mult(tmp, gain_norm_inv).wrapping_shl(gain_norm_inv_exp as u32);
        }
    }
}

/// `normalizePower(hgVBAPRenderer)` (static): power normalization of the gain cache.
fn normalize_power(h: &mut GVbapRenderer) {
    let mut gain_norm: FixpDbl = 0;
    let mut gain_norm_inv_exp = 0;
    let len = h.gain_cache_length.max(0) as usize;

    /* Adding all gains in gaincache array never results in a higher value than 19/32 see code line
     * 418 */
    for &g in h.gain_cache[..len].iter() {
        gain_norm = f_pow2_add(gain_norm, g);
    }

    /* Normalize gains */
    let gain_norm_inv = inv_sqrt_norm2(gain_norm, &mut gain_norm_inv_exp);

    for g in h.gain_cache[..len].iter_mut() {
        let tmp = f_mult(*g, gain_norm_inv);
        *g = scale_value_saturate(tmp, gain_norm_inv_exp - 1);
    }
}
