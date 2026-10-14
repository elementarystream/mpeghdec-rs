// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDec_tools.cpp, drcDec_tools.h.

//! Helper functions shared by the DRC decoder modules.

use mpeghdec_fdk::fixpoint_math::f2_pow;
use mpeghdec_fdk::{f_mult, fl2fxconst_dbl, FixpDbl, FixpSgl};

use crate::drc_dec_types::*;
use crate::drc_decoder::*;

/// `getDeltaTmin(sampleRate)`
pub fn get_delta_tmin(sample_rate: i32) -> i32 {
    /* half_ms = round (0.0005 * sampleRate); */
    let half_ms = (sample_rate.wrapping_add(1000)) / 2000;
    let mut delta_tmin = 1i32;
    if sample_rate < 1000 {
        return DE_NOT_OK;
    }
    while delta_tmin <= half_ms {
        delta_tmin <<= 1;
    }
    delta_tmin
}

/// `selectDrcCoefficients(hUniDrcConfig, location)`: index of the matching
/// `drcCoefficientsUniDrc` element (`NULL` → `None`).
pub fn select_drc_coefficients(h_uni_drc_config: &UniDrcConfig, location: i32) -> Option<usize> {
    let mut c: i32 = -1;
    for n in 0..h_uni_drc_config.drc_coefficients_uni_drc_count as usize {
        if h_uni_drc_config.drc_coefficients_uni_drc[n].drc_location as i32 == location {
            c = n as i32;
        }
    }
    if c >= 0 {
        return Some(c as usize);
    }
    None /* possible during bitstream parsing */
}

/// `selectDrcInstructions(hUniDrcConfig, drcSetId)`: index of the matching
/// `drcInstructionsUniDrc` element.
pub fn select_drc_instructions(h_uni_drc_config: &UniDrcConfig, drc_set_id: i32) -> Option<usize> {
    (0..h_uni_drc_config.drc_instructions_count_incl_virtual as usize)
        .find(|&i| h_uni_drc_config.drc_instructions_uni_drc[i].drc_set_id as i32 == drc_set_id)
}

/// `selectDownmixInstructions(hUniDrcConfig, downmixId)`: index of the matching
/// `downmixInstructions` element.
pub fn select_downmix_instructions(
    h_uni_drc_config: &UniDrcConfig,
    downmix_id: i32,
) -> Option<usize> {
    (0..h_uni_drc_config.downmix_instructions_count as usize)
        .find(|&i| h_uni_drc_config.downmix_instructions[i].downmix_id as i32 == downmix_id)
}

/// `deriveDrcChannelGroups(...)`
///
/// `ducking_modification_for_channel` and `ducking_modification_for_channel_group` correspond to
/// the pointer arguments that may be `NULL`.
pub fn derive_drc_channel_groups(
    drc_set_effect: i32,                                              /* in */
    channel_count: i32,                                               /* in */
    gain_set_index: &[i8],                                            /* in */
    ducking_modification_for_channel: Option<&[DuckingModification]>, /* in */
    n_drc_channel_groups: &mut u8,                                    /* out */
    unique_index: &mut [i8],      /* out (gainSetIndexForChannelGroup) */
    group_for_channel: &mut [i8], /* out */
    ducking_modification_for_channel_group: Option<&mut [DuckingModification]>, /* out */
) -> DrcError {
    let mut ducking_sequence: i32 = -1;
    let mut g: usize;
    let mut unique_scaling: [FixpSgl; 28] = [0; 28];

    for g in 0..28 {
        unique_index[g] = -10;
        unique_scaling[g] = -1; /* FIXP_SGL(-1.0f) */
    }

    g = 0;

    if drc_set_effect & EB_DUCK_OTHER != 0 {
        for c in 0..channel_count.max(0) as usize {
            let mut matched = false;
            if c >= 2 * 28 {
                return DE_MEMORY_ERROR;
            }
            let idx = gain_set_index[c] as i32;
            // C dereferences the pointer unchecked; it is only NULL for sets without ducking.
            let factor = ducking_modification_for_channel.map_or(0, |d| d[c].ducking_scaling);
            if idx < 0 {
                for n in 0..g {
                    if unique_scaling[n] == factor {
                        matched = true;
                        group_for_channel[c] = n as i8;
                        break;
                    }
                }
                if !matched {
                    if g >= 28 {
                        return DE_MEMORY_ERROR;
                    }
                    unique_index[g] = idx as i8;
                    unique_scaling[g] = factor;
                    group_for_channel[c] = g as i8;
                    g += 1;
                }
            } else {
                if (ducking_sequence > 0) && (ducking_sequence != idx) {
                    return DE_NOT_OK;
                }
                ducking_sequence = idx;
                group_for_channel[c] = -1;
            }
        }
        if ducking_sequence == -1 {
            return DE_NOT_OK;
        }
    } else if drc_set_effect & EB_DUCK_SELF != 0 {
        for c in 0..channel_count.max(0) as usize {
            let mut matched = false;
            if c >= 2 * 28 {
                return DE_MEMORY_ERROR;
            }
            let idx = gain_set_index[c] as i32;
            let factor = ducking_modification_for_channel.map_or(0, |d| d[c].ducking_scaling);
            if idx >= 0 {
                for n in 0..g {
                    if (unique_index[n] as i32 == idx) && (unique_scaling[n] == factor) {
                        matched = true;
                        group_for_channel[c] = n as i8;
                        break;
                    }
                }
                if !matched {
                    if g >= 28 {
                        return DE_MEMORY_ERROR;
                    }
                    unique_index[g] = idx as i8;
                    unique_scaling[g] = factor;
                    group_for_channel[c] = g as i8;
                    g += 1;
                }
            } else {
                group_for_channel[c] = -1;
            }
        }
    } else {
        /* no ducking */
        for c in 0..channel_count.max(0) as usize {
            if c >= 2 * 28 {
                return DE_MEMORY_ERROR;
            }
            let idx = gain_set_index[c] as i32;
            let mut matched = false;
            if idx >= 0 {
                for n in 0..g {
                    if unique_index[n] as i32 == idx {
                        matched = true;
                        group_for_channel[c] = n as i8;
                        break;
                    }
                }
                if !matched {
                    if g >= 28 {
                        return DE_MEMORY_ERROR;
                    }
                    unique_index[g] = idx as i8;
                    group_for_channel[c] = g as i8;
                    g += 1;
                }
            } else {
                group_for_channel[c] = -1;
            }
        }
    }
    *n_drc_channel_groups = g as u8;

    if drc_set_effect & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0 {
        let mut out = ducking_modification_for_channel_group;
        for g in 0..*n_drc_channel_groups as usize {
            if drc_set_effect & EB_DUCK_OTHER != 0 {
                unique_index[g] = ducking_sequence as i8;
            }
            if let Some(out) = out.as_deref_mut() {
                out[g].ducking_scaling = unique_scaling[g];
            }
        }
    }

    /* sanity check */
    for g in 0..*n_drc_channel_groups as usize {
        if unique_index[g] < 0 {
            return DE_NOT_OK;
        }
    }

    DE_OK
}

/// `dB2lin(dB_m, dB_e, pLin_e)`: get linear value from dB.
///
/// return lin_val = 10^(dB_val/20) = 2^(log2(10)/20*dB_val)
/// with dB_val = dB_m *2^dB_e and lin_val = lin_m * 2^lin_e
pub fn db2lin(db_m: FixpDbl, db_e: i32, p_lin_e: &mut i32) -> FixpDbl {
    const C: FixpDbl = fl2fxconst_dbl((0.1660964f32 * 4.0f32) as f64);
    f2_pow(f_mult(db_m, C), db_e - 2, p_lin_e)
}

/// `approxDb2lin(dB_m, dB_e, pLin_e)`: get linear value from approximate dB.
///
/// return lin_val = 2^(dB_val/6)
/// with dB_val = dB_m *2^dB_e and lin_val = lin_m * 2^lin_e
pub fn approx_db2lin(db_m: FixpDbl, db_e: i32, p_lin_e: &mut i32) -> FixpDbl {
    const C: FixpDbl = fl2fxconst_dbl((0.1666667f32 * 4.0f32) as f64);
    f2_pow(f_mult(db_m, C), db_e - 2, p_lin_e)
}

/// `bitstreamContainsMultibandDrc(hUniDrcConfig, downmixId)`
pub fn bitstream_contains_multiband_drc(h_uni_drc_config: &UniDrcConfig, downmix_id: i32) -> i32 {
    let mut is_multiband = 0;

    let Some(c) = select_drc_coefficients(h_uni_drc_config, LOCATION_SELECTED) else {
        return 0;
    };
    let p_coef = &h_uni_drc_config.drc_coefficients_uni_drc[c];

    for i in 0..h_uni_drc_config.drc_instructions_uni_drc_count as usize {
        let p_inst = &h_uni_drc_config.drc_instructions_uni_drc[i];
        for d in 0..p_inst.downmix_id_count as usize {
            if downmix_id == p_inst.downmix_id[d] as i32 {
                for g in 0..p_inst.n_drc_channel_groups as usize {
                    let seq = p_inst.gain_set_index_for_channel_group[g] as i32;
                    if (seq < p_coef.gain_set_count as i32)
                        && (p_coef.gain_set[seq as usize].band_count > 1)
                    {
                        is_multiband = 1;
                    }
                }
            }
        }
    }

    is_multiband
}
