// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcGainDec_preprocess.cpp, drcGainDec_preprocess.h.

//! Conversion of the decoded gain nodes to linear gains, including gain modifications.

use mpeghdec_fdk::fixpoint_math::f_add_norm;
use mpeghdec_fdk::scale::scale_value_saturate;
use mpeghdec_fdk::{
    f_max, f_min, f_mult, f_mult_div2, fl2fxconst_dbl, fl2fxconst_sgl, fx_sgl2fx_dbl, FixpDbl,
    FixpSgl,
};

use crate::drc_dec_gain_decoder::*;
use crate::drc_dec_reader::gain_set_at;
use crate::drc_dec_tools::approx_db2lin;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;

/// `SLOPE_FACTOR_DB_TO_LINEAR`: ln(10) / 20
const SLOPE_FACTOR_DB_TO_LINEAR: FixpDbl = fl2fxconst_dbl((0.1151f32 * 8.0f32) as f64);

/// `NODE_MODIFICATION`
struct NodeModification {
    drc_set_effect: i32,
    p_d_mod: Option<DuckingModification>,
    p_g_mod: Option<[GainModification; 4]>,
    limiter_peak_target_present: i32,
    limiter_peak_target: FixpSgl,
    loudness_normalization_gain_db: FixpDbl,
    compress: FixpSgl,
    boost: FixpSgl,
}

fn to_linear(
    node_mod: &NodeModification,
    drc_band: usize,
    gain_db: FixpSgl,        /* in: gain value in dB, e = 7 */
    slope_db: FixpSgl,       /* in: slope value in dB/deltaTmin, e = 2 */
    gain_lin: &mut FixpDbl,  /* out: linear gain value, e = 7 */
    slope_lin: &mut FixpDbl, /* out: linear slope value, e = 7 */
) -> DrcError {
    const ONE_E2: FixpSgl = fl2fxconst_sgl((1.0f32 / 4.0f32) as f64);
    let mut gain_ratio_m: FixpDbl = fl2fxconst_dbl((1.0f32 / 2.0f32) as f64);
    let p_g_mod: Option<GainModification> = node_mod.p_g_mod.map(|m| m[drc_band]);
    let p_d_mod = node_mod.p_d_mod;
    let mut gain_lin_e = 0;
    let mut gain_ratio_e = 1;
    let mut gain_db_out_e = 0;

    if ((node_mod.drc_set_effect & (EB_DUCK_OTHER | EB_DUCK_SELF)) == 0)
        && (node_mod.drc_set_effect != EB_FADE)
        && (node_mod.drc_set_effect != EB_CLIPPING)
    {
        if gain_db < 0 {
            gain_ratio_m = f_mult_div2(gain_ratio_m, node_mod.compress);
        } else {
            gain_ratio_m = f_mult_div2(gain_ratio_m, node_mod.boost);
        }
        gain_ratio_e += 2;
    }
    if let Some(g_mod) = p_g_mod {
        if (g_mod.attenuation_scaling != ONE_E2) || (g_mod.amplification_scaling != ONE_E2) {
            if gain_db < 0 {
                gain_ratio_m = f_mult_div2(gain_ratio_m, g_mod.attenuation_scaling);
            } else {
                gain_ratio_m = f_mult_div2(gain_ratio_m, g_mod.amplification_scaling);
            }
            gain_ratio_e += 3;
        }
    }
    if let Some(d_mod) = p_d_mod {
        if (node_mod.drc_set_effect & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0)
            && (d_mod.ducking_scaling != ONE_E2)
        {
            gain_ratio_m = f_mult_div2(gain_ratio_m, d_mod.ducking_scaling);
            gain_ratio_e += 3;
        }
    }

    let gain_db_modified: FixpDbl = f_mult_div2(gain_db, gain_ratio_m); /* resulting e: 7 + gainRatio_e + 1*/
    let mut gain_db_offset: FixpDbl = 0;

    if let Some(g_mod) = p_g_mod {
        if g_mod.gain_offset != 0 {
            /* *gainLin *= (float)pow(2.0, (double)(pGMod->gainOffset/6.0f)); */
            gain_db_offset = gain_db_offset.wrapping_add(fx_sgl2fx_dbl(g_mod.gain_offset) >> 4);
            /* resulting e: 8 */
        }
    }
    if (node_mod.limiter_peak_target_present == 1) && (node_mod.drc_set_effect == EB_CLIPPING) {
        /* The only drcSetEffect is "clipping prevention" */
        /* loudnessNormalizationGainModificationDb is included in loudnessNormalizationGainDb */
        /* *gainLin *= (float)pow(2.0, max(0.0, -nodeModification->limiterPeakTarget -
         * nodeModification->loudnessNormalizationGainDb)/6.0); */
        let neg_lpt = (-(node_mod.limiter_peak_target as i32)).wrapping_shl(16);
        gain_db_offset = gain_db_offset.wrapping_add(f_max(
            0,
            (neg_lpt >> 3).wrapping_sub(node_mod.loudness_normalization_gain_db >> 1),
        )); /* resulting e: 8 */
    }
    let gain_db_out: FixpDbl;
    if gain_db_offset != 0 {
        gain_db_out = f_add_norm(
            gain_db_modified,
            7 + gain_ratio_e + 1,
            gain_db_offset,
            8,
            &mut gain_db_out_e,
        );
    } else {
        gain_db_out = gain_db_modified;
        gain_db_out_e = 7 + gain_ratio_e + 1;
    }

    /* *gainLin = (float)pow(2.0, (double)(gainDb_modified[1] / 6.0f)); */
    let mut gain_lin_m = approx_db2lin(gain_db_out, gain_db_out_e, &mut gain_lin_e);
    *gain_lin = scale_value_saturate(gain_lin_m, gain_lin_e - 7);

    /* *slopeLin = SLOPE_FACTOR_DB_TO_LINEAR * gainRatio * *gainLin * slopeDb; */
    if slope_db == 0 {
        *slope_lin = 0;
    } else {
        let mut tmp_dbl: FixpDbl = f_mult(slope_db, SLOPE_FACTOR_DB_TO_LINEAR); /* resulting e: 2 - 3 = -1 */
        tmp_dbl = f_mult(tmp_dbl, gain_ratio_m); /* resulting e: -1 + gainRatio_e */
        if gain_db_offset != 0 {
            /* recalculate gainLin from gainDb that wasn't modified by gainOffset and
             * limiterPeakTarget */
            gain_lin_m = approx_db2lin(gain_db_modified, 7 + gain_ratio_e, &mut gain_lin_e);
        }
        let slope_lin_m = f_mult(tmp_dbl, gain_lin_m);
        *slope_lin = scale_value_saturate(slope_lin_m, -1 + gain_ratio_e + gain_lin_e - 7);
    }

    if (node_mod.limiter_peak_target_present == 1) && (node_mod.drc_set_effect == EB_CLIPPING) {
        if *gain_lin >= fl2fxconst_dbl((1.0f32 / 128.0f32) as f64) {
            *gain_lin = fl2fxconst_dbl((1.0f32 / 128.0f32) as f64);
            *slope_lin = 0;
        }
    }

    DE_OK
}

/// `_getNumSequences(hUniDrcGain)`: get overall number of sequences contained in uniDrcGain
pub fn get_num_sequences(h_uni_drc_gain: &UniDrcGain) -> i32 {
    let mut num_sequences = h_uni_drc_gain.n_decoded_sequences[0] as i32;
    for s in 1..4 {
        num_sequences += h_uni_drc_gain.n_decoded_sequences[s] as i32;
    }
    num_sequences
}

/// `prepareDrcGain(hGainDec, hUniDrcGain, compress, boost, loudnessNormalizationGainDb,
/// activeDrcIndex, activeDrcLocation)`: prepare buffers containing linear nodes for each gain
/// sequence
pub fn prepare_drc_gain(
    h_gain_dec: &mut DrcGainDecoder,
    h_uni_drc_gain: &UniDrcGain,
    compress: FixpSgl,
    boost: FixpSgl,
    loudness_normalization_gain_db: FixpDbl,
    active_drc_index: usize,
    active_drc_location: usize,
) -> DrcError {
    let lnbp = h_gain_dec.drc_gain_buffers.lnb_pointer as usize;
    let p_active_drc = &h_gain_dec.active_drc[active_drc_location][active_drc_index];
    let p_inst = &p_active_drc.drc_inst;
    let p_coef = &h_gain_dec.drc_coef;
    let drc_gain_buffers = &mut h_gain_dec.drc_gain_buffers;

    let mut node_mod = NodeModification {
        drc_set_effect: p_inst.drc_set_effect as i32,
        p_d_mod: None,
        p_g_mod: None,
        limiter_peak_target_present: p_inst.limiter_peak_target_present as i32,
        limiter_peak_target: p_inst.limiter_peak_target,
        loudness_normalization_gain_db,
        compress,
        boost,
    };

    let mut gain_element_index: i32 = 0;
    for g in 0..p_inst.n_drc_channel_groups as usize {
        if p_active_drc.channel_group_is_parametric_drc[g] == 0 {
            let gain_set_index = p_inst.gain_set_index_for_channel_group[g] as i32;

            if node_mod.drc_set_effect & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0 {
                node_mod.p_d_mod = Some(p_active_drc.ducking_modification_for_channel_group[g]);
                node_mod.p_g_mod = None;
            } else {
                node_mod.p_g_mod = Some(p_inst.gain_modification_group(g));
                node_mod.p_d_mod = None;
            }

            let n_drc_bands = p_active_drc.band_count_for_channel_group[g] as usize;
            for b in 0..n_drc_bands {
                let p_gain_set = gain_set_at(p_coef, gain_set_index);
                let seq = p_gain_set.gain_sequence_index[b.min(3)] as i32;

                /* linearNodeBuffer contains a copy of the gain sequences (consisting of nodes) that
                are relevant for decoding. It also contains gain sequences of previous frames. */
                let p_lnb =
                    drc_gain_buffers.lnb_mut(p_active_drc.active_drc_offset + gain_element_index);
                p_lnb.gain_interpolation_type =
                    p_gain_set.gain_interpolation_type as GainInterpolationType;

                /* sanity check on sequence index */
                if seq >= get_num_sequences(h_uni_drc_gain) {
                    return DE_NOT_OK;
                }
                if seq >= 48 {
                    /* C reads out of bounds here (undefined behaviour, not reachable for DRC sets
                     * that passed the selection process) */
                    mpeghdec_fdk::deviation::hit();
                    return DE_NOT_OK;
                }
                let seq = seq as usize;

                /* copy a node buffer and convert from dB to linear */
                p_lnb.written[lnbp] = false;
                p_lnb.n_nodes[lnbp] = f_min(h_uni_drc_gain.n_nodes[seq] as i32, 32);
                for i in 0..p_lnb.n_nodes[lnbp] as usize {
                    let mut gain_lin = 0;
                    let mut slope_lin = 0;
                    let err = to_linear(
                        &node_mod,
                        b,
                        h_uni_drc_gain.gain_node[seq][i].gain_db,
                        0,
                        &mut gain_lin,
                        &mut slope_lin,
                    );
                    if err != 0 {
                        return err;
                    }
                    p_lnb.linear_node_gain[lnbp][i] = gain_lin;
                    p_lnb.linear_node_time[lnbp][i] = h_uni_drc_gain.gain_node[seq][i].time;
                }
                p_lnb.written[lnbp] = true;
                gain_element_index += 1;
            }
        } else {
            /* parametric DRC not supported */
            gain_element_index += 1;
        }
    }
    DE_OK
}
