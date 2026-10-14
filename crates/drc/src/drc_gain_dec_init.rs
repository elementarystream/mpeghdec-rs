// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcGainDec_init.cpp, drcGainDec_init.h.

//! Initialization of the DRC gain decoder for the selected DRC sets.

use mpeghdec_fdk::fl2fxconst_dbl;

use crate::drc_dec_gain_decoder::*;
use crate::drc_dec_reader::gain_set_at;
use crate::drc_dec_tools::*;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;
use crate::drc_gain_dec_multiband::init_overlap_weight;

fn generate_drc_instructions_derived_data(
    h_uni_drc_config: &UniDrcConfig,
    p_inst: &DrcInstructionsUniDrc,
    p_coef: &DrcCoefficientsUniDrc,
    p_active_drc: &mut ActiveDrc,
) -> DrcError {
    let mut gain_element_count: i32 = 0;
    let mut n_drc_channel_groups: u8 = 0;
    let mut gain_set_index_for_channel_group = [0i8; 28];

    let is_ducking = p_inst.drc_set_effect as i32 & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0;
    let d_mod_for_channel = p_inst.ducking_modifications();
    let err = derive_drc_channel_groups(
        p_inst.drc_set_effect as i32,
        p_inst.drc_channel_count as i32,
        &p_inst.gain_set_index,
        if is_ducking { Some(&d_mod_for_channel) } else { None },
        &mut n_drc_channel_groups,
        &mut gain_set_index_for_channel_group,
        &mut p_active_drc.channel_group_for_channel,
        if is_ducking {
            Some(&mut p_active_drc.ducking_modification_for_channel_group)
        } else {
            None
        },
    );
    if err != 0 {
        return err;
    }

    /* sanity check */
    if n_drc_channel_groups != p_inst.n_drc_channel_groups {
        return DE_NOT_OK;
    }
    for g in 0..p_inst.n_drc_channel_groups as usize {
        if gain_set_index_for_channel_group[g] != p_inst.gain_set_index_for_channel_group[g] {
            return DE_NOT_OK;
        }
    }

    for g in 0..p_inst.n_drc_channel_groups as usize {
        let seq = p_inst.gain_set_index_for_channel_group[g] as i32;
        if seq != -1
            && (h_uni_drc_config.drc_coefficients_uni_drc_count == 0
                || seq >= p_coef.gain_set_count as i32)
        {
            p_active_drc.channel_group_is_parametric_drc[g] = 1;
        } else {
            p_active_drc.channel_group_is_parametric_drc[g] = 0;
            if seq >= p_coef.gain_set_count as i32 {
                return DE_NOT_OK;
            }
        }
    }

    /* gainElementCount */
    if is_ducking {
        for g in 0..p_inst.n_drc_channel_groups as usize {
            p_active_drc.band_count_for_channel_group[g] = 1;
        }
        p_active_drc.gain_element_count = p_inst.n_drc_channel_groups; /* one gain element per channel group */
    } else {
        for g in 0..p_inst.n_drc_channel_groups as usize {
            if p_active_drc.channel_group_is_parametric_drc[g] != 0 {
                gain_element_count += 1;
                p_active_drc.band_count_for_channel_group[g] = 1;
            } else {
                let seq = p_inst.gain_set_index_for_channel_group[g] as i32;
                let band_count = gain_set_at(p_coef, seq).band_count;
                p_active_drc.band_count_for_channel_group[g] = band_count;
                gain_element_count += band_count as i32;
            }
        }
        p_active_drc.gain_element_count = gain_element_count as u8;
    }

    /* prepare gainElementForGroup (cumulated sum of bandCountForChannelGroup) */
    p_active_drc.gain_element_for_group[0] = 0;
    for g in 1..p_inst.n_drc_channel_groups as usize {
        /* index of first gain sequence in channel group */
        p_active_drc.gain_element_for_group[g] = p_active_drc.gain_element_for_group[g - 1]
            .wrapping_add(p_active_drc.band_count_for_channel_group[g - 1]);
    }

    DE_OK
}

/// `initGainDec(hGainDec)`
pub fn init_gain_dec(h_gain_dec: &mut DrcGainDecoder) -> DrcError {
    /* sanity check */
    if h_gain_dec.delta_tmin_default > h_gain_dec.frame_size {
        return DE_NOT_OK;
    }

    for h in 0..ACTIVE_DRC_LOCATIONS {
        for i in 0..MAX_ACTIVE_DRCS {
            for j in 0..28 {
                /* use startup node at the beginning */
                h_gain_dec.active_drc[h][i].lnb_index_for_channel[j][0] = 0;
                for k in 1..NUM_LNB_FRAMES {
                    h_gain_dec.active_drc[h][i].lnb_index_for_channel[j][k] = -1;
                }
            }
        }
    }

    for j in 0..28 {
        h_gain_dec.channel_gain[j] = fl2fxconst_dbl((1.0f32 / 256.0f32) as f64);
    }

    for i in 0..SUBBAND_GAINS_ROW {
        h_gain_dec.subband_gains[DUMMY_SUBBAND_GAINS_ROW * SUBBAND_GAINS_ROW + i] =
            fl2fxconst_dbl((1.0f32 / 128.0f32) as f64);
    }

    h_gain_dec.status = 0; /* startup */
    h_gain_dec.startup_ms = 2500;

    DE_OK
}

/// `initDrcGainBuffers(frameSize, drcGainBuffers)`
pub fn init_drc_gain_buffers(frame_size: i32, drc_gain_buffers: &mut DrcGainBuffers) {
    /* prepare 48 instances of node buffers */
    for i in 0..48 {
        drc_gain_buffers.linear_node_buffer[i].written = [false; NUM_LNB_FRAMES];
        for j in 0..NUM_LNB_FRAMES {
            let lnb = &mut drc_gain_buffers.linear_node_buffer[i];
            lnb.n_nodes[j] = 1;
            lnb.linear_node_gain[j][0] = fl2fxconst_dbl((1.0f32 / 128.0f32) as f64);
            if j == 0 {
                lnb.linear_node_time[j][0] = 0; /* initialize last node with startup node */
            } else {
                lnb.linear_node_time[j][0] = (frame_size - 1) as i16;
            }
        }
    }

    /* prepare dummyLnb, a linearNodeBuffer containing a constant gain of 0 dB, for the "no DRC
     * processing" case */
    let dummy = &mut drc_gain_buffers.linear_node_buffer[DUMMY_LNB];
    dummy.gain_interpolation_type = GIT_LINEAR;
    for i in 0..NUM_LNB_FRAMES {
        dummy.n_nodes[i] = 1;
        dummy.linear_node_gain[i][0] = fl2fxconst_dbl((1.0f32 / 128.0f32) as f64);
        dummy.linear_node_time[i][0] = (frame_size - 1) as i16;
    }

    drc_gain_buffers.lnb_pointer = 0;
}

/// `initActiveDrc(hGainDec, hUniDrcConfig, drcSetIdSelected, downmixIdSelected)`
pub fn init_active_drc(
    h_gain_dec: &mut DrcGainDecoder,
    h_uni_drc_config: &UniDrcConfig,
    drc_set_id_selected: i32,
    _downmix_id_selected: i32,
) -> DrcError {
    let mut is_multiband = false;
    let mut loc: usize = 0; /* before or after downmix */

    let Some(inst_idx) = select_drc_instructions(h_uni_drc_config, drc_set_id_selected) else {
        return DE_NOT_OK;
    };
    let p_inst = &h_uni_drc_config.drc_instructions_uni_drc[inst_idx];

    if p_inst.drc_apply_to_downmix != 0 {
        loc = 1;
    }

    let n = h_gain_dec.n_active_drcs[loc] as usize;
    if p_inst.drc_set_id >= 0 {
        let err = generate_drc_instructions_derived_data(
            h_uni_drc_config,
            p_inst,
            &h_gain_dec.drc_coef,
            &mut h_gain_dec.active_drc[loc][n],
        );
        if err != 0 {
            return err;
        }
    }

    h_gain_dec.active_drc[loc][n].drc_inst = p_inst.clone(); /* keep deep copy of drcInstructions */

    if loc == 0 {
        for g in 0..p_inst.n_drc_channel_groups as usize {
            if h_gain_dec.active_drc[loc][n].band_count_for_channel_group[g] > 1 {
                if h_gain_dec.multi_band_active_drc_index != -1 {
                    return DE_NOT_OK;
                }
                is_multiband = true;
            }
        }

        if is_multiband {
            /* Keep activeDrc index of multiband DRC set */
            h_gain_dec.multi_band_active_drc_index = n as i32;
            if h_gain_dec.subband_domain_supported != 0 {
                /* initialize subband domain multiband processing */
                let err = init_overlap_weight(
                    &h_gain_dec.drc_coef,
                    p_inst,
                    &h_gain_dec.active_drc[loc][n].band_count_for_channel_group,
                    h_gain_dec.subband_domain_supported,
                    &mut h_gain_dec.overlap,
                );
                if err != 0 {
                    return err;
                }
            }
        }
    }

    h_gain_dec.n_active_drcs[loc] += 1;
    if h_gain_dec.n_active_drcs[loc] > MAX_ACTIVE_DRCS as i32 {
        return DE_NOT_OK;
    }

    DE_OK
}

/// `addVirtualToActiveDrc(hGainDec)`
pub fn add_virtual_to_active_drc(h_gain_dec: &mut DrcGainDecoder) {
    /* add a virtual DRC set to the activeDrcs, if there is none present yet.
    This removes clicks if nActiveDrcs changes during runtime due to a user interaction. */

    for l in 0..ACTIVE_DRC_LOCATIONS {
        /* cross-check if adding a virtual DRC set is possible */
        if h_gain_dec.n_active_drcs[l] >= MAX_ACTIVE_DRCS as i32 {
            continue;
        }

        /* Don't add virtual DRC for subband processing, because clicks don't occur there */
        if l == 0 && h_gain_dec.multi_band_active_drc_index >= 0 {
            continue;
        }

        let mut virtual_drc_active = false;
        /* cross-check if there is already a virtual DRC set active */
        for a in 0..h_gain_dec.n_active_drcs[l].max(0) as usize {
            if h_gain_dec.active_drc[l][a].drc_inst.drc_set_id < 0 {
                virtual_drc_active = true;
            }
        }

        if !virtual_drc_active {
            /* add a virtual DRC set */
            let n = h_gain_dec.n_active_drcs[l].max(0) as usize;
            let p_inst = &mut h_gain_dec.active_drc[l][n].drc_inst;
            *p_inst = DrcInstructionsUniDrc::ZERO;
            p_inst.drc_set_id = -1;

            if l == 0 {
                p_inst.downmix_id_count = 1;
                p_inst.downmix_id[0] = DOWNMIX_ID_BASE_LAYOUT as u8;
            } else {
                p_inst.downmix_id_count = 1;
                p_inst.downmix_id[0] = DOWNMIX_ID_ANY_DOWNMIX as u8;
                p_inst.drc_apply_to_downmix = 1;
            }

            h_gain_dec.n_active_drcs[l] += 1;
        }
    }

    /* There should now always be a virtual DRC set on location 0 (before downmix).
    Pick the first one to apply channelGains */
    if h_gain_dec.n_active_drcs[0] > 0 && h_gain_dec.channel_gain_active_drc_index == -1 {
        h_gain_dec.channel_gain_active_drc_index = 0;
    }
}

/// `initActiveDrcOffset(hGainDec)`
pub fn init_active_drc_offset(h_gain_dec: &mut DrcGainDecoder) -> DrcError {
    let mut acc_gain_element_count: i32 = 0;
    for l in 0..ACTIVE_DRC_LOCATIONS {
        for a in 0..h_gain_dec.n_active_drcs[l].max(0) as usize {
            h_gain_dec.active_drc[l][a].active_drc_offset = acc_gain_element_count;
            acc_gain_element_count += h_gain_dec.active_drc[l][a].gain_element_count as i32;
            if acc_gain_element_count > 48 {
                h_gain_dec.n_active_drcs[l] = a as i32;
                return DE_NOT_OK;
            }
        }
    }

    DE_OK
}
