// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/fdp.h, libMpeghDec/src/fdp.cpp and libMpeghDec/src/arm/fdp_arm.cpp
// (the ARM file only contains ARMv7 NEON code, which is inactive in the reference configuration).

//! TCX/AAC frequency domain prediction (FDP).

use crate::channelinfo::{CAacDecoderChannelInfo, CAacDecoderStaticChannelInfo};
use mpeghdec_fdk::audio::AC_INDEP;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::mdct::BLOCK_SHORT;
use mpeghdec_fdk::scale::{get_scalefactor, scale_value};

/// `HLM`
const HLM: i32 = 128;

/// `fdp_exp`: `64 * (0:181) .^ (4/3)`
static FDP_EXP: [u16; 182] = [
    0, 64, 161, 277, 406, 547, 698, 857, 1024, 1198, 1379, 1566, 1758, 1956, 2159, 2368, 2580,
    2798, 3019, 3245, 3474, 3708, 3945, 4186, 4431, 4678, 4930, 5184, 5442, 5702, 5966, 6232, 6502,
    6774, 7049, 7327, 7608, 7891, 8176, 8464, 8755, 9048, 9344, 9641, 9941, 10244, 10548, 10855,
    11164, 11476, 11789, 12104, 12422, 12741, 13063, 13386, 13712, 14039, 14369, 14700, 15033,
    15368, 15705, 16044, 16384, 16726, 17070, 17416, 17763, 18113, 18463, 18816, 19170, 19526,
    19883, 20242, 20603, 20965, 21329, 21694, 22061, 22430, 22800, 23171, 23544, 23919, 24295,
    24672, 25051, 25431, 25813, 26196, 26581, 26966, 27354, 27742, 28132, 28524, 28917, 29311,
    29706, 30103, 30501, 30900, 31301, 31703, 32106, 32511, 32916, 33323, 33732, 34141, 34552,
    34964, 35377, 35791, 36207, 36624, 37042, 37461, 37881, 38303, 38725, 39149, 39574, 40000,
    40427, 40856, 41285, 41716, 42147, 42580, 43014, 43449, 43885, 44323, 44761, 45200, 45641,
    46082, 46525, 46968, 47413, 47859, 48306, 48753, 49202, 49652, 50103, 50555, 51008, 51462,
    51916, 52372, 52829, 53287, 53746, 54206, 54667, 55128, 55591, 56055, 56520, 56985, 57452,
    57920, 58388, 58858, 59328, 59799, 60271, 60745, 61219, 61694, 62170, 62647, 63124, 63603,
    64083, 64563, 65044, 65527,
];

/// `fdp_scf`: `2 .^ ((0:62 + 1) / 4)`
static FDP_SCF: [u16; 63] = [
    1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 19, 23, 27, 32, 38, 45, 54, 64, 76, 91,
    108, 128, 152, 181, 215, 256, 304, 362, 431, 512, 609, 724, 861, 1024, 1218, 1448, 1722, 2048,
    2435, 2896, 3444, 4096, 4871, 5793, 6889, 8192, 9742, 11585, 13777, 16384, 19484, 23170, 27554,
    32768, 38968, 46341, 55109,
]; /* 65536 */

/// `fdp_s1`: `49152 * fdp_scl .* fdp_sin`
static FDP_S1: [u16; 129] = [
    0, 726, 1451, 2176, 2901, 3625, 4348, 5071, 5792, 6512, 7230, 7947, 8662, 9376, 10087, 10796,
    11503, 12207, 12908, 13607, 14303, 14995, 15685, 16371, 17054, 17732, 18408, 19079, 19746,
    20409, 21068, 21722, 22372, 23017, 23658, 24293, 24924, 25549, 26169, 26784, 27394, 27998,
    28596, 29189, 29776, 30357, 30932, 31501, 32064, 32621, 33171, 33715, 34253, 34784, 35309,
    35827, 36339, 36844, 37282, 37698, 38105, 38503, 38892, 39272, 39643, 40005, 40359, 40703,
    41039, 41366, 41684, 41994, 42296, 42589, 42874, 43151, 43419, 43680, 43933, 44179, 44416,
    44646, 44869, 45085, 45293, 45494, 45689, 45876, 46057, 46232, 46400, 46562, 46717, 46867,
    47011, 47149, 47281, 47408, 47529, 47645, 47756, 47862, 47963, 48059, 48150, 48237, 48319,
    48397, 48471, 48540, 48605, 48666, 48724, 48777, 48826, 48872, 48914, 48953, 48988, 49019,
    49047, 49072, 49093, 49111, 49126, 49137, 49146, 49150, 49152,
];

/// `fdp_s2`: `-18432 * fdp_scl .* fdp_scl`
static FDP_S2: [i16; 129] = [
    -26681, -26680, -26678, -26675, -26670, -26665, -26658, -26650, -26641, -26630, -26618, -26605,
    -26591, -26576, -26559, -26541, -26522, -26502, -26481, -26459, -26436, -26411, -26386, -26359,
    -26331, -26303, -26273, -26242, -26211, -26178, -26145, -26110, -26075, -26039, -26002, -25964,
    -25926, -25887, -25847, -25806, -25765, -25723, -25680, -25637, -25593, -25549, -25504, -25458,
    -25413, -25366, -25320, -25273, -25225, -25178, -25130, -25082, -25033, -24985, -24856, -24710,
    -24564, -24417, -24272, -24126, -23981, -23836, -23691, -23547, -23404, -23262, -23121, -22980,
    -22841, -22702, -22565, -22429, -22295, -22162, -22030, -21900, -21771, -21644, -21519, -21396,
    -21274, -21155, -21037, -20921, -20808, -20697, -20587, -20481, -20376, -20274, -20174, -20077,
    -19982, -19889, -19799, -19712, -19627, -19546, -19466, -19390, -19316, -19245, -19177, -19112,
    -19049, -18990, -18933, -18879, -18829, -18781, -18736, -18695, -18656, -18620, -18588, -18558,
    -18532, -18508, -18488, -18471, -18457, -18446, -18438, -18434, -18432,
];

// The reference build (clang, AArch64) auto-vectorizes the loops that add the scaled FDP estimate
// to the spectrum. `scaleValue(fdp_int[i], fdp_shift)` may shift by 32 or more; the scalar
// instructions mask the amount modulo 32 (like `scale_value()`), the vector instructions
// (`ushl`/`sshl`) use the signed low byte of the amount and give 0 (left and logical right
// shifts) or the sign (arithmetic right shifts) for amounts of 32 or more. The vector loop
// processes the first `width & !15` values of a band of `width >= 16` values, the scalar loop
// the rest. `scale_fdp()` reproduces this split (and that of the x86-64 build).

/// NEON `ushl` of one 32 bit lane by `shift` (the signed low byte is used).
#[inline]
fn neon_ushl(v: FixpDbl, shift: i32) -> FixpDbl {
    let s = shift as i8 as i32;
    if s >= 0 {
        if s >= 32 {
            0
        } else {
            v << s
        }
    } else if -s >= 32 {
        0
    } else {
        ((v as u32) >> -s) as FixpDbl
    }
}

/// NEON `sshl` of one 32 bit lane by `shift` (the signed low byte is used).
#[inline]
fn neon_sshl(v: FixpDbl, shift: i32) -> FixpDbl {
    let s = shift as i8 as i32;
    if s >= 0 {
        if s >= 32 {
            0
        } else {
            v << s
        }
    } else if -s >= 32 {
        v >> 31
    } else {
        v >> -s
    }
}

/// SSE2 `pslld` / `psrad` of one 32 bit lane by the non-negative 32 bit amount `s` (zero
/// extended to the 64 bit count): 0 (left shifts) or the sign (arithmetic right shifts) for
/// amounts of 32 or more.
#[inline]
fn sse_shift(v: FixpDbl, fdp_shift: i32) -> FixpDbl {
    if fdp_shift > 0 {
        if fdp_shift >= 32 {
            0
        } else {
            v << fdp_shift
        }
    } else {
        let s = fdp_shift.unsigned_abs();
        if s >= 32 {
            v >> 31
        } else {
            v >> s
        }
    }
}

/// `scaleValue(fdp, fdp_shift)` as computed by the reference build for value `idx` of a band of
/// `width` values (see above).
///
/// The x86-64 build (clang, SSE2) vectorizes the same loops with 8 values per iteration
/// (`pslld` / `psrad`, see [`sse_shift`]) for bands of `width >= 8` values: the vector loop
/// processes the first `width & !7` values, the scalar loop (amounts modulo 32) the rest.
#[inline]
fn scale_fdp(v: FixpDbl, fdp_shift: i32, idx: usize, width: usize) -> FixpDbl {
    if mpeghdec_fdk::ARCH_X86_32 {
        // no SSE: the 32 bit x86 build does not vectorize the loops
        return scale_value(v, fdp_shift);
    }
    if mpeghdec_fdk::ARCH_X86 {
        return if width >= 8 && idx < (width & !7) {
            sse_shift(v, fdp_shift)
        } else {
            scale_value(v, fdp_shift)
        };
    }
    let vectorized = width >= 16 && idx < (width & !15);
    if !vectorized {
        scale_value(v, fdp_shift)
    } else if fdp_shift > 0 {
        neon_ushl(v, fdp_shift)
    } else {
        neon_sshl(v, fdp_shift)
    }
}

/// `FDP_DecodeBins(pAacDecoderChannelInfo, pAacDecoderStaticChannelInfo, flags, quantSpecCurr,
/// sfbOffsets, predictionBandwidth, isTcx, i_gain_m, i_gain_e, g_div64, lg)`
///
/// Frequency domain prediction: computes the prediction estimate from the quantized spectra of
/// the two previous frames (`quantSpecPrev1/2` of the static channel info), updates them with
/// the current quantized spectrum `quant_spec_curr` and adds the estimate to the spectrum
/// `output_spec_curr` (`pAacDecoderChannelInfo->pSpectralCoefficient`), adapting
/// `pDynData->aSfbScale` where necessary.
///
/// `i_gain_m`, `i_gain_e`, `g_div64` and `lg` are not used (like in C++).
#[allow(clippy::too_many_arguments)]
pub fn fdp_decode_bins(
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    output_spec_curr: &mut [FixpDbl],
    p_aac_decoder_static_channel_info: &mut CAacDecoderStaticChannelInfo,
    flags: u32,
    quant_spec_curr: &[FixpDbl],
    sfb_offsets: &[i16],
    prediction_bandwidth: i32,
    is_tcx: i32,
    _i_gain_m: FixpDbl,
    _i_gain_e: i32,
    _g_div64: i32,
    _lg: i32,
) {
    let st = p_aac_decoder_static_channel_info;
    let ch = p_aac_decoder_channel_info;
    let quant_scf_curr = &ch.p_dyn_data.a_scale_factor;
    let fdp_spacing_index = ch.fdp_spacing_index;
    let b_eight_short_sequence = ch.ics_info.window_sequence == BLOCK_SHORT;
    let max_lines = prediction_bandwidth;
    let mut sfb_num_active = ch.ics_info.max_sf_bands as i32;

    let fdp_spacing_value = (894 / 3i32).wrapping_sub(fdp_spacing_index); /* only used for 0..=255 */
    let mut i: i32 = 0;
    let mut harmonic_spacing = 0;
    let mut harm_index = -HLM;
    let mut comp_index = 2 * HLM; /* harmonic/compare index */
    let mut fdp_int = [0 as FixpDbl; 160];

    let sfb_off = |sfb: i32| sfb_offsets[sfb as usize] as i32;

    /* if indepFlag==1 -> reset FDP history */
    if flags & AC_INDEP != 0 {
        st.quant_spec_prev1.fill(0);
        st.quant_spec_prev2.fill(0);
    }

    if is_tcx == 0 && (b_eight_short_sequence || sfb_num_active <= 0) {
        /* no FD predictor */
        sfb_num_active = 0;
    } else if is_tcx != 0 || max_lines < sfb_off(sfb_num_active) {
        /* ISO 23003-3, Table 109 */
        if is_tcx == 0 {
            sfb_num_active = 0;
            while sfb_off(sfb_num_active) < max_lines {
                sfb_num_active += 1;
            }
            if sfb_off(sfb_num_active) > max_lines {
                sfb_num_active -= 1; /* sanity */
            }
        }
        if ch.fdp_data_present != 0
            && flags & AC_INDEP == 0
            && fdp_spacing_index >= 0
            && fdp_spacing_index < 256
        {
            harmonic_spacing = (894 * 512 + fdp_spacing_value) / (2 * fdp_spacing_value);
        }
    }

    /* reset coefficients, adapt both for each harmonic */
    let mut s1: i32 = 0;
    let mut s2: i32 = 0;

    let quant_spec_prev1 = &mut st.quant_spec_prev1;
    let quant_spec_prev2 = &mut st.quant_spec_prev2;

    /* start decoding: for each quantized bin compute predictor estimate */
    if harmonic_spacing != 0 {
        /* FDP active and allowed, compute estimate */
        i = sfb_off(0);
        while i < sfb_off(sfb_num_active) {
            let iu = i as usize;
            if f_abs(i * 2 * HLM - harm_index) >= 3 * HLM {
                /* bin not harmonic */
                fdp_int[iu] = 0;
            } else {
                /* bin is part of the currently active harmonic line */
                let reg32 = s1
                    .wrapping_mul(quant_spec_prev1[iu] as i32)
                    .wrapping_add(s2.wrapping_mul(quant_spec_prev2[iu] as i32));
                fdp_int[iu] = ((f_abs(reg32) as u32).wrapping_add(16384) >> 15) as FixpDbl;
                if reg32 < 0 {
                    fdp_int[iu] = -fdp_int[iu];
                }
            }
            if i * 2 * HLM == comp_index {
                /* update indices and LPC coeffs */
                harm_index += harmonic_spacing;
                comp_index = harm_index & (2 * HLM - 1);
                if comp_index > HLM {
                    comp_index = 2 * HLM - comp_index; /* exploit trigon. symmetry */
                }
                s1 = FDP_S1[comp_index as usize] as i32;
                s2 = FDP_S2[comp_index as usize] as i32;
                comp_index = harm_index >> 8; /* integer unscaled harm. index */
                if (comp_index & 1) == 0 {
                    s1 *= -1; /* negate first LPC coeff for even harm. indices */
                }
                comp_index = 2 * HLM + ((harm_index + HLM) >> 8) * 2 * HLM; /* index */
            }
            i += 1;
        }
    }
    for sfb in 0..sfb_num_active {
        let mut scf16 = quant_scf_curr[sfb as usize] as i32 - 21; /* shifted sf[sfb] */

        if scf16 < 0 {
            scf16 = 0;
        } else {
            scf16 = if scf16 > 62 { 65536 } else { FDP_SCF[scf16 as usize] as i32 };
            /* scf look-up */
        }

        /* start update: compute integer sum of each line and its estimate */
        i = sfb_off(sfb);
        while i < sfb_off(sfb + 1) {
            let iu = i as usize;
            let q = quant_spec_curr[iu];
            let mut x_int = FDP_EXP[f_min(f_abs(q), 181) as usize] as i32; /* look-up */

            x_int = ((512u32).wrapping_add((x_int as u32).wrapping_mul(scf16 as u32)) >> 10) as i32;
            if q < 0 {
                x_int *= -1;
            }
            if harmonic_spacing != 0 {
                x_int += fdp_int[iu]; /* add previously computed FDP estimate */
            }
            quant_spec_prev2[iu] = quant_spec_prev1[iu];
            quant_spec_prev1[iu] = f_min(f_max(x_int, -31775), 31775) as i16;
            i += 1;
        }
    }
    /* finalize update: reset states of currently uncoded spectral lines */
    while i < max_lines {
        quant_spec_prev1[i as usize] = 0;
        quant_spec_prev2[i as usize] = 0;
        i += 1;
    }

    if harmonic_spacing != 0 {
        /* FDP active and allowed, compute estimate */
        let a_sfb_scale = &mut ch.p_dyn_data.a_sfb_scale;
        let mut p2output = 0usize;
        let mut p2fdp = 0usize;

        for sfb in 0..sfb_num_active {
            let width = (sfb_off(sfb + 1) - sfb_off(sfb)) as usize;

            /* We calculate the number of missing (non-available) headroom which would happen in
             * fdp_int after being scaled */
            let mut fdp_scale = get_scalefactor(&fdp_int[p2fdp..p2fdp + width]);
            let mut fdp_shift = 40 - a_sfb_scale[sfb as usize] as i32;
            fdp_scale -= fdp_shift;

            /* Find the headroom of the signal */
            let output_scale = get_scalefactor(&output_spec_curr[p2output..p2output + width]);

            /* Find the minimum of the headroom of the signal and the missing fdp headroom */
            let diff_min = f_min(fdp_scale, output_scale);

            /* When binary point adjustment is necessary. We make sure that fdp data does not
            overflow after shifting and that there is at least one bit headroom to accomodate the
            increase due to addition */
            if diff_min < 1 {
                let mut shift = diff_min - 1;

                /* Correct scale factor */
                a_sfb_scale[sfb as usize] = (a_sfb_scale[sfb as usize] as i32 - shift) as i16;

                fdp_shift = 40 - a_sfb_scale[sfb as usize] as i32;

                shift = f_max(-31, shift);

                for k in 0..width {
                    /* shift is in [-31, -1]: identical scalar and vector results */
                    let val1 = scale_value(output_spec_curr[p2output], shift);
                    let val2 = scale_fdp(fdp_int[p2fdp], fdp_shift, k, width);
                    p2fdp += 1;
                    output_spec_curr[p2output] = val1.wrapping_add(val2);
                    p2output += 1;
                }
            } else {
                /* When binary point adjustment is not necessary */
                for k in 0..width {
                    let val = scale_fdp(fdp_int[p2fdp], fdp_shift, k, width);
                    p2fdp += 1;
                    output_spec_curr[p2output] = output_spec_curr[p2output].wrapping_add(val);
                    p2output += 1;
                }
            }
        } /* for (sfb = 0; sfb < sfbNumActive; sfb++) */
    } /* if (harmonicSpacing) */
}
