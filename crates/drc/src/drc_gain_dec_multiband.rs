// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcGainDec_multiband.cpp, drcGainDec_multiband.h.

//! Overlap weights for multiband DRC in the subband domain.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::fixpoint_math::{f_div_norm, f_pow, inv_fixp};
use mpeghdec_fdk::scale::{scale_value, scale_value_saturate};
use mpeghdec_fdk::{fl2fxconst_dbl, fx_dbl2fx_sgl, fx_sgl2fx_dbl, FixpDbl, FixpSgl, MAXVAL_SGL};

use crate::drc_dec_gain_decoder::{
    SUBBAND_NUM_BANDS_QMF64, SUBBAND_NUM_BANDS_QMF71, SUBBAND_NUM_BANDS_STFT256,
};
use crate::drc_dec_reader::gain_set_at;
use crate::drc_dec_rom::FILTER_BANK_PARAMETERS;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;

/// `MAX_NUM_SUBBANDS`
pub const MAX_NUM_SUBBANDS: usize = 256;

/// `OVERLAP_PARAMETERS`
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct OverlapParameters {
    pub start_index: u16,
    pub stop_index: u16,
    pub overlap_weight: [FixpSgl; MAX_NUM_SUBBANDS],
}

impl OverlapParameters {
    pub const ZERO: Self =
        Self { start_index: 0, stop_index: 0, overlap_weight: [0; MAX_NUM_SUBBANDS] };
}

#[allow(clippy::approx_constant, clippy::excessive_precision)]
const LOG10_2_INV: f32 = 3.32192809f32; /* 1.0 / log10(2.0) */
const LR_FILTER_SLOPE: f32 = -24.0f32; /* slope of Linkwitz/Riley filters in dB per octave */
const NORM_CONST: FixpDbl =
    fl2fxconst_dbl((LR_FILTER_SLOPE * LOG10_2_INV / 20.0f32 / 4.0f32) as f64);

fn init_fcenter_norm_sb(
    n_subbands: i32,
    subband_domain_mode: SubbandDomainMode,
    f_center_norm_sb: &mut [FixpDbl],
) -> DrcError {
    let inv_n_subbands: i32 = inv_fixp(n_subbands);
    match subband_domain_mode {
        SDM_STFT256 => {
            for s in 0..n_subbands {
                /* fCenterNormSb[s] = s / (2.0f * nSubbands); */
                f_center_norm_sb[s as usize] = s.wrapping_mul(inv_n_subbands >> 1);
            }
        }
        _ => return DE_NOT_OK, /* not supported */
    }
    DE_OK
}

fn generate_slope(
    audio_decoder_subband_count: usize,
    f_center_norm_sb: &[FixpDbl],
    f_cross_norm_lo: FixpDbl,
    f_cross_norm_hi: FixpDbl,
    response: &mut [FixpSgl],
) {
    for s in 0..audio_decoder_subband_count {
        let mut r_e = 0;
        if f_center_norm_sb[s] < f_cross_norm_lo {
            /* response[s] = pow (10.0, NORM_CONST * log10(fCrossNormLo / fCenterNormSb[s]))
            = pow (fCrossNormLo / fCenterNormSb[s], NORM_CONST) */
            if f_center_norm_sb[s] > 0 {
                let ratio = f_div_norm(f_cross_norm_lo, f_center_norm_sb[s], &mut r_e);
                let e_in = r_e;
                let power = f_pow(ratio, e_in, NORM_CONST, 2, &mut r_e);
                response[s] = fx_dbl2fx_sgl(scale_value(power, r_e));
            } else {
                response[s] = 0;
            }
        } else if f_center_norm_sb[s] < f_cross_norm_hi {
            response[s] = MAXVAL_SGL as FixpSgl; /* nearly 1.0f */
        } else {
            /* response[s] = pow (10.0, norm * log10(fCenterNormSb[s] / fCrossNormHi))
            = pow (fCenterNormSb[s] / fCrossNormHi, NORM_CONST) */
            let ratio = f_div_norm(f_center_norm_sb[s], f_cross_norm_hi, &mut r_e);
            let e_in = r_e;
            let power = f_pow(ratio, e_in, NORM_CONST, 2, &mut r_e);
            response[s] = fx_dbl2fx_sgl(scale_value(power, r_e));
        }
    }
}

fn generate_overlap_weights(
    n_drc_bands: usize,
    drc_band_type: i32,
    p_band_border: &[BandBorder; 4],
    subband_domain_mode: SubbandDomainMode,
    overlap_for_group: &mut [OverlapParameters; 4],
) -> DrcError {
    let mut f_center_norm_sb = [0 as FixpDbl; MAX_NUM_SUBBANDS];
    let audio_decoder_subband_count: usize = match subband_domain_mode {
        SDM_QMF64 => SUBBAND_NUM_BANDS_QMF64,
        SDM_QMF71 => SUBBAND_NUM_BANDS_QMF71,
        SDM_STFT256 => SUBBAND_NUM_BANDS_STFT256,
        _ => return DE_NOT_OK,
    };

    if drc_band_type == 1 {
        if init_fcenter_norm_sb(
            audio_decoder_subband_count as i32,
            subband_domain_mode,
            &mut f_center_norm_sb,
        ) != 0
        {
            return DE_NOT_OK;
        }

        let mut f_cross_norm_lo: FixpDbl = 0;
        for b in 0..n_drc_bands {
            let f_cross_norm_hi = if b < n_drc_bands - 1 {
                // C indexes the table without range check; crossoverFreqIndex is a 4 bit value.
                FILTER_BANK_PARAMETERS[p_band_border[b + 1].crossover_freq_index() as usize & 15]
                    .f_cross_norm
            } else {
                fl2fxconst_dbl(0.5)
            };
            generate_slope(
                audio_decoder_subband_count,
                &f_center_norm_sb,
                f_cross_norm_lo,
                f_cross_norm_hi,
                &mut overlap_for_group[b].overlap_weight,
            );

            f_cross_norm_lo = f_cross_norm_hi;
        }
        for s in 0..audio_decoder_subband_count {
            let mut w_norm: FixpDbl = 0;
            for b in 0..n_drc_bands {
                /* e_wNorm = ld(4) = 3 */
                w_norm =
                    w_norm.wrapping_add(fx_sgl2fx_dbl(overlap_for_group[b].overlap_weight[s]) >> 3);
            }
            for b in 0..n_drc_bands {
                /* overlapForGroup[b].overlapWeight[s] /= wNorm; */
                let mut e_tmp = 0;
                let tmp_overlap_weight = f_div_norm(
                    fx_sgl2fx_dbl(overlap_for_group[b].overlap_weight[s]),
                    w_norm,
                    &mut e_tmp,
                );
                overlap_for_group[b].overlap_weight[s] =
                    fx_dbl2fx_sgl(scale_value_saturate(tmp_overlap_weight, e_tmp - 3));
            }
        }
        for b in 0..n_drc_bands {
            /* get start and stop indices of nonzero entries of overlapWeight */
            let mut s = 0usize;
            while s < audio_decoder_subband_count {
                if overlap_for_group[b].overlap_weight[s] > 0 {
                    overlap_for_group[b].start_index = s as u16;
                    break;
                }
                s += 1;
            }
            while s < audio_decoder_subband_count {
                if overlap_for_group[b].overlap_weight[s] > 0 {
                    overlap_for_group[b].stop_index = s as u16;
                } else {
                    break;
                }
                s += 1;
            }
            /* stop_index is excluding, i.e. it is the first index outside of the nonzero entry
             * section */
            overlap_for_group[b].stop_index = overlap_for_group[b].stop_index.wrapping_add(1);
        }
    } else {
        let mut start_sub_band_index: i32 = 0;
        for b in 0..n_drc_bands {
            let stop_sub_band_index: i32;
            if b < n_drc_bands - 1 {
                /* stopSubBandIndex is including, i.e. it is the last index within the band */
                stop_sub_band_index = p_band_border[b + 1].start_sub_band_index as i32 - 1;
                if stop_sub_band_index < start_sub_band_index {
                    return DE_NOT_OK;
                }
                if stop_sub_band_index > (audio_decoder_subband_count as i32 - 1) {
                    return DE_NOT_OK;
                }
            } else {
                stop_sub_band_index = audio_decoder_subband_count as i32 - 1;
            }
            overlap_for_group[b].start_index = start_sub_band_index as u16;
            overlap_for_group[b].stop_index = (stop_sub_band_index + 1) as u16;
            for s in 0..audio_decoder_subband_count as i32 {
                if s >= start_sub_band_index && s <= stop_sub_band_index {
                    overlap_for_group[b].overlap_weight[s as usize] = MAXVAL_SGL as FixpSgl;
                /* nearly 1.0f */
                } else {
                    overlap_for_group[b].overlap_weight[s as usize] = 0;
                }
            }
            start_sub_band_index = stop_sub_band_index + 1;
        }
    }

    DE_OK
}

/// `initOverlapWeight(pCoef, pInst, bandCountForChannelGroup, subbandDomainMode, pOverlap)`
pub fn init_overlap_weight(
    p_coef: &DrcCoefficientsUniDrc,
    p_inst: &DrcInstructionsUniDrc,
    band_count_for_channel_group: &[u8; 28],
    subband_domain_mode: SubbandDomainMode,
    p_overlap: &mut [[OverlapParameters; 4]; 28],
) -> DrcError {
    for g in 0..p_inst.n_drc_channel_groups as usize {
        if band_count_for_channel_group[g] > 1 {
            let gain_set = gain_set_at(p_coef, p_inst.gain_set_index_for_channel_group[g] as i32);
            let err = generate_overlap_weights(
                (band_count_for_channel_group[g] as usize).min(4),
                gain_set.drc_band_type as i32,
                &gain_set.band_border,
                subband_domain_mode,
                &mut p_overlap[g],
            );
            if err != 0 {
                return err;
            }
        }
    }

    DE_OK
}
