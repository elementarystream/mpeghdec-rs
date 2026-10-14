// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/ui.h and libMpeghDec/src/ui.cpp.

//! MPEG-H user interactivity: parsing of the user interaction, loudness/DRC interface and marker
//! MHAS packets, selection of the active UI status (embedded UI manager or MHAS packets), element
//! skipping and the application of the interactivity (group gains, loudness compensation,
//! object position offsets, DRC/loudness parameters).

use crate::aacdecoder::*;
use crate::aacdecoder_lib::*;
use mpeghdec_drc::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::*;
use mpeghdec_fdk::scale::scale_value;
use mpeghdec_tpdec::*;
use mpeghdec_uimanager::*;

/// `getDrcEffectTypeFallback(drcEffectTypeRequested)`: apply default fallbacks for requested DRC
/// effect types
fn get_drc_effect_type_fallback(drc_effect_type_requested: i32) -> i32 {
    match drc_effect_type_requested {
        -1 => UI_MANAGER_DRC_OFF as i32, /* DRC OFF*/
        0 => 0x00000000,                 /* NONE */
        1 => 0x00543261,                 /* NIGHT */
        2 => 0x00543162,                 /* NOISY */
        3 => 0x00542163,                 /* LIMITED */
        4 => 0x00531264,                 /* LOWLEVEL */
        5 => 0x00432165,                 /* DIALOG */
        6 => 0x00543216,                 /* GENERAL */
        _ => 0x00000000,
    }
}

/// `(UCHAR)` conversion of an `int` expression.
#[inline]
fn uchar(v: i32) -> u8 {
    v as u8
}

impl AacDecoderInstance {
    /// `mpegh3daElementInteraction(self, hBs)`: parse user interaction packet
    /// (mpegh3daElementInteraction)
    fn mpegh3da_element_interaction(&mut self, h_bs: &mut FdkBitStream) -> i32 {
        let p_asi = &self.h_ui_manager.asi;
        let mut ui_status = self.ui_status;

        let interaction_signature_data_length = h_bs.read_bits(8);
        if interaction_signature_data_length > 0 {
            /* skip signature for now */
            h_bs.push_for(8 + interaction_signature_data_length * 8);
        }

        /* ElementInteractionData */
        ui_status.interaction_mode = h_bs.read_bit() as u8;
        let num_groups = h_bs.read_bits(7) as i32;
        if num_groups > ASI_MAX_GROUPS as i32 {
            return AAC_DEC_PARSE_ERROR;
        }

        if ui_status.interaction_mode != 0 {
            ui_status.group_preset_id = h_bs.read_bits(5) as u8;
            if asi_group_preset_id2idx(p_asi, ui_status.group_preset_id as i32) < 0 {
                return AAC_DEC_PARSE_ERROR;
            }
        }

        /* init on/off states to ASI defaults */
        ui_status.num_groups = p_asi.num_groups;

        for grp_idx in 0..p_asi.num_groups as usize {
            ui_status.group_data[grp_idx].group_id = p_asi.groups[grp_idx].group_id;

            if p_asi.groups[grp_idx].switch_group_id == 255 {
                ui_status.group_data[grp_idx].on_off = p_asi.groups[grp_idx].default_on_off;
            }
        }

        for sw_grp_idx in 0..p_asi.num_switch_groups as usize {
            if p_asi.switch_groups[sw_grp_idx].default_on_off != 0 {
                let grp_idx = asi_group_id2idx(
                    p_asi,
                    p_asi.switch_groups[sw_grp_idx].default_group_id as i32,
                );
                if grp_idx >= 0 {
                    ui_status.group_data[grp_idx as usize].on_off = 1;
                }
            }
        }

        /* read group data */
        for _ in 0..num_groups {
            /* read group ID */
            let group_id = h_bs.read_bits(7) as u8;

            /* get group index and pointer */
            let grp_idx = asi_group_id2idx(p_asi, group_id as i32);
            if grp_idx < 0 {
                return AAC_DEC_PARSE_ERROR;
            }
            let grp_idx = grp_idx as usize;

            let p_asi_group = &p_asi.groups[grp_idx];

            /* on/off state */
            let on_off = h_bs.read_bit() as u8;
            if p_asi_group.allow_on_off != 0 || (p_asi_group.switch_group_id != 255) {
                let sw_grp_idx = asi_switch_group_id2idx(p_asi, p_asi_group.switch_group_id as i32);
                /* (a negative index is undefined behaviour in C++; the members are not reset) */
                if on_off != 0 && (p_asi_group.switch_group_id != 255) && sw_grp_idx < 0 {
                    mpeghdec_fdk::deviation::hit();
                }
                if on_off != 0 && (p_asi_group.switch_group_id != 255) && sw_grp_idx >= 0 {
                    let sw_grp_idx = sw_grp_idx as usize;
                    for j in 0..p_asi.switch_groups[sw_grp_idx].num_members as usize {
                        let mem_grp_idx = asi_group_id2idx(
                            p_asi,
                            p_asi.switch_groups[sw_grp_idx].member_id[j] as i32,
                        );
                        if mem_grp_idx >= 0 {
                            ui_status.group_data[mem_grp_idx as usize].on_off = 0;
                        }
                    }
                }

                ui_status.group_data[grp_idx].on_off = on_off;
            }

            let p_group = &mut ui_status.group_data[grp_idx];

            /* route to WIRE */
            p_group.route_to_wire = h_bs.read_bit() as u8;
            if p_group.route_to_wire != 0 {
                p_group.route_to_wire_id = h_bs.read_bits(16) as u8;
            }

            /* interactivity */
            if on_off != 0 {
                /* position */
                p_group.change_position = h_bs.read_bit() as u8;
                if p_group.change_position != 0 {
                    p_group.az_offset = h_bs.read_bits(8) as u8;
                    p_group.el_offset = h_bs.read_bits(6) as u8;
                    p_group.dist_fact = h_bs.read_bits(4) as u8;
                }

                if p_asi_group.allow_position_interactivity == 0 {
                    p_group.change_position = 0;
                }

                if p_group.change_position != 0 {
                    p_group.az_offset = p_group
                        .az_offset
                        .max(uchar(-(p_asi_group.interactivity_min_az_offset as i32) + 128));
                    p_group.az_offset = p_group
                        .az_offset
                        .min(uchar(p_asi_group.interactivity_max_az_offset as i32 + 128));

                    p_group.el_offset = p_group
                        .el_offset
                        .max(uchar(-(p_asi_group.interactivity_min_el_offset as i32) + 32));
                    p_group.el_offset = p_group
                        .el_offset
                        .min(uchar(p_asi_group.interactivity_max_el_offset as i32 + 32));
                }

                /* gain */
                p_group.change_gain = h_bs.read_bit() as u8;
                if p_group.change_gain != 0 {
                    p_group.gain = h_bs.read_bits(7) as u8;
                }

                if p_asi_group.allow_gain_interactivity == 0 {
                    p_group.change_gain = 0;
                }

                if p_group.change_gain != 0 {
                    if p_asi_group.interactivity_min_gain != 0 {
                        p_group.gain =
                            p_group.gain.max(uchar(p_asi_group.interactivity_min_gain as i32 + 1));
                        /* minGain - 63 + 64 */
                    }

                    p_group.gain =
                        p_group.gain.min(uchar(p_asi_group.interactivity_max_gain as i32 + 64));
                }
            }
        }

        if h_bs.read_bit() != 0 {
            /* skip local zoom area for now */
            h_bs.push_for(10 + 10 + 9 + 10);
        }

        /* apply preset conditions */
        if ui_status.interaction_mode != 0 {
            let preset_idx = asi_group_preset_id2idx(p_asi, ui_status.group_preset_id as i32);
            let conditions: &[AsiGroupPresetCondition] = if preset_idx < 0 {
                &[]
            } else {
                let gp = &p_asi.group_presets[preset_idx as usize];
                if gp.has_downmix_id_extension != 0 {
                    &gp.downmix_id_extension.conditions
                        [..gp.downmix_id_extension.num_conditions as usize]
                } else {
                    &gp.conditions[..gp.num_conditions as usize]
                }
            };

            /* apply conditions */
            for cond in conditions {
                if cond.is_switch_group_condition == 0 {
                    /* group condition */
                    let grp_idx = asi_group_id2idx(p_asi, cond.reference_id as i32);
                    if grp_idx < 0 {
                        continue;
                    }
                    let grp_idx = grp_idx as usize;

                    /* on/off flag */
                    if cond.condition_on_off != 0 && (p_asi.groups[grp_idx].switch_group_id != 255)
                    {
                        let sw_grp_idx = asi_switch_group_id2idx(
                            p_asi,
                            p_asi.groups[grp_idx].switch_group_id as i32,
                        );
                        /* (a negative index is undefined behaviour in C++) */
                        let n = if sw_grp_idx >= 0 {
                            p_asi.switch_groups[sw_grp_idx as usize].num_members as usize
                        } else {
                            mpeghdec_fdk::deviation::hit();
                            0
                        };
                        let sw_grp_idx = sw_grp_idx.max(0) as usize;

                        for j in 0..n {
                            let mem_grp_idx = asi_group_id2idx(
                                p_asi,
                                p_asi.switch_groups[sw_grp_idx].member_id[j] as i32,
                            );
                            if mem_grp_idx >= 0 {
                                ui_status.group_data[mem_grp_idx as usize].on_off = 0;
                            }
                        }
                    }

                    ui_status.group_data[grp_idx].on_off = cond.condition_on_off;

                    /* disable interactivity flags */
                    if cond.disable_gain_interactivity != 0 {
                        ui_status.group_data[grp_idx].change_gain = 0;
                    }
                    if cond.disable_position_interactivity != 0 {
                        ui_status.group_data[grp_idx].change_position = 0;
                    }
                } else {
                    /* switch group condition */
                    let sw_grp_idx = asi_switch_group_id2idx(p_asi, cond.reference_id as i32);
                    if sw_grp_idx < 0 {
                        continue;
                    }
                    let sw_grp_idx = sw_grp_idx as usize;
                    let mut num_on = 0;

                    for j in 0..p_asi.switch_groups[sw_grp_idx].num_members as usize {
                        let grp_idx = asi_group_id2idx(
                            p_asi,
                            p_asi.switch_groups[sw_grp_idx].member_id[j] as i32,
                        );
                        if grp_idx < 0 {
                            continue;
                        }
                        let g = &mut ui_status.group_data[grp_idx as usize];

                        /* if switch group turned off by condition, set all members to off */
                        if cond.condition_on_off == 0 {
                            g.on_off = 0;
                        }

                        /* count active members */
                        if g.on_off != 0 {
                            num_on += 1;
                        }

                        /* disable interactivity flags */
                        if cond.disable_gain_interactivity != 0 {
                            g.change_gain = 0;
                        }
                        if cond.disable_position_interactivity != 0 {
                            g.change_position = 0;
                        }
                    }

                    /* if switch group turned on by condition and no member active, activate
                     * default member */
                    if cond.condition_on_off != 0 && (num_on == 0) {
                        let grp_idx = asi_group_id2idx(
                            p_asi,
                            p_asi.switch_groups[sw_grp_idx].default_group_id as i32,
                        );
                        if grp_idx >= 0 {
                            ui_status.group_data[grp_idx as usize].on_off = 1;
                        }
                    }
                }
            }
        }

        /* validate switch group states */
        for sw_grp_idx in 0..p_asi.num_switch_groups as usize {
            let sg = &p_asi.switch_groups[sw_grp_idx];
            let mut num_on = 0;

            /* count active members */
            for i in 0..sg.num_members as usize {
                let grp_idx = asi_group_id2idx(p_asi, sg.member_id[i] as i32);
                if (grp_idx >= 0) && ui_status.group_data[grp_idx as usize].on_off != 0 {
                    num_on += 1;
                }
            }

            /* if more than one active member set to default state */
            if num_on > 1 {
                for i in 0..sg.num_members as usize {
                    let grp_idx = asi_group_id2idx(p_asi, sg.member_id[i] as i32);
                    if grp_idx >= 0 {
                        ui_status.group_data[grp_idx as usize].on_off = 0;
                    }
                }

                if sg.default_on_off != 0 {
                    let grp_idx = asi_group_id2idx(p_asi, sg.default_group_id as i32);
                    if grp_idx >= 0 {
                        ui_status.group_data[grp_idx as usize].on_off = 1;
                    }
                }
            }
            /* if no member active and turning switch group off not allowed, activate default
             * member */
            else if num_on == 0 && sg.allow_on_off == 0 && sg.default_on_off != 0 {
                let grp_idx = asi_group_id2idx(p_asi, sg.default_group_id as i32);
                if grp_idx >= 0 {
                    ui_status.group_data[grp_idx as usize].on_off = 1;
                }
            }
        }

        for grp_idx in 0..p_asi.num_groups as usize {
            let next = self.ui_status_next.group_data[grp_idx];
            let p_group = &mut ui_status.group_data[grp_idx];

            /* for turned off groups copy previous interactivity state to avoid transition to
             * default during fade-out frame */
            if p_group.on_off == 0 && next.on_off != 0 {
                p_group.change_position = next.change_position;
                p_group.az_offset = next.az_offset;
                p_group.el_offset = next.el_offset;
                p_group.dist_fact = next.dist_fact;

                p_group.change_gain = next.change_gain;
                p_group.gain = next.gain;
            }
            let group = *p_group;

            /* Copy parameters to all other switch group members which are inactive */
            if group.on_off != 0 {
                let mut found = false;
                for sw_grp_idx in 0..p_asi.num_switch_groups as usize {
                    let sg = &p_asi.switch_groups[sw_grp_idx];
                    /* find switch group */
                    for i in 0..sg.num_members as usize {
                        if group.group_id == sg.member_id[i] {
                            found = true;
                            break;
                        }
                    }
                    if found {
                        for i in 0..sg.num_members as usize {
                            if group.group_id != sg.member_id[i] {
                                let swgrp = asi_group_id2idx(p_asi, sg.member_id[i] as i32);
                                if swgrp < 0 {
                                    /* groupData[-1] in C++ (undefined behaviour) */
                                    mpeghdec_fdk::deviation::hit();
                                    continue;
                                }
                                let p_g = &mut ui_status.group_data[swgrp as usize];

                                p_g.change_position = group.change_position;
                                p_g.az_offset = group.az_offset;
                                p_g.el_offset = group.el_offset;
                                p_g.dist_fact = group.dist_fact;

                                p_g.change_gain = group.change_gain;
                                p_g.gain = group.gain;
                            }
                        }
                        break;
                    }
                }
            }

            /* check if group on/off states changed (for element skipping) */
            if ui_status.group_data[grp_idx].on_off != next.on_off {
                /* signals have changed, requires forced config change and delaying UI change
                 * until next RAP */
                self.ui_signal_changed = 1;
            }
        }

        self.ui_status_next = ui_status;
        self.ui_status_valid = 1;

        0
    }

    /// `mpegh3daLoudnessDrcInterface(self, hBs)`
    fn mpegh3da_loudness_drc_interface(&mut self, h_bs: &mut FdkBitStream) -> i32 {
        let mut target_loudness: i32 = -1;
        let mut drc_effect_type: u32 = 0x80000000; /* not set */
        let mut boost: i32 = -1; /* Q8 representation. 256 .. 0 --> 1.0 .. 0.0  */
        let mut compress: i32 = -1; /* Q8 representation. 256 .. 0 --> 1.0 .. 0.0  */
        let mut album_mode: i32 = -1;

        /* uniDrcInterfaceSignature */
        if h_bs.read_bit() != 0 {
            h_bs.read_bits(8); /* uniDrcInterfaceSignatureType */
            let uni_drc_interface_signature_data_length = h_bs.read_bits(8); /* uniDrcInterfaceSignatureDataLength */
            h_bs.push_for(uni_drc_interface_signature_data_length * 8); /* uniDrcInterfaceSignatureData */
        }

        /* systemInterface */
        if h_bs.read_bit() != 0 {
            let target_config_request_type = h_bs.read_bits(2);

            match target_config_request_type {
                0 => {
                    let num_downmix_id_requests = h_bs.read_bits(4);

                    for _ in 0..num_downmix_id_requests {
                        h_bs.read_bits(7); /* downmixIdRequested[i] */
                    }
                }
                1 => {
                    h_bs.read_bits(8); /* targetLayoutRequested */
                }
                2 => {
                    /* targetChannelCountRequested  = */
                    h_bs.read_bits(7) /* + 1 */;
                }
                _ => return 1,
            }
        }

        /* loudnessNormalizationControlInterface */
        if h_bs.read_bit() != 0 {
            let loudness_normalization_on = h_bs.read_bit();

            if loudness_normalization_on == 1 {
                /* targetLoudness = -  FDKreadBits(hBs, 12) * 0.03125f; */
                target_loudness = h_bs.read_bits(12) as i32;
            }
        }

        /* loudnessNormalizationParameterInterface */
        if h_bs.read_bit() != 0 {
            album_mode = h_bs.read_bit() as i32; /* albumMode */
            h_bs.read_bit(); /* peakLimiterPresent */

            if h_bs.read_bit() != 0 {
                /* changeLoudnessDeviationMax */
                h_bs.read_bits(6); /* loudnessDeviationMax */
            }

            if h_bs.read_bit() != 0 {
                /* changeLoudnessMeasurementMethod */
                h_bs.read_bits(3); /* loudnessMeasurementMethod */
            }

            if h_bs.read_bit() != 0 {
                /* changeLoudnessMeasurementSystem */
                h_bs.read_bits(4); /*  loudnessMeasurementSystem*/
            }

            if h_bs.read_bit() != 0 {
                /* changeLoudnessMeasurementPreProc */
                h_bs.read_bits(2); /* loudnessMeasurementPreProc */
            }

            if h_bs.read_bit() != 0 {
                /* changeDeviceCutOffFrequency */
                h_bs.read_bits(6); /* deviceCutOffFrequency */
            }

            if h_bs.read_bit() != 0 {
                /* changeLoudnessNormalizationGainDbMax */
                h_bs.read_bits(6); /* loudnessNormalizationGainDbMax */
            }

            if h_bs.read_bit() != 0 {
                /* changeLoudnessNormalizationGainModificationDb */
                h_bs.read_bits(10); /* loudnessNormalizationGainModificationDb */
            }

            if h_bs.read_bit() != 0 {
                /* changeOutputPeakLevelMax */
                h_bs.read_bits(6); /* outputPeakLevelMax */
            }
        }

        /* dynamicRangeControlInterface */
        if h_bs.read_bit() != 0 {
            if h_bs.read_bit() != 0 {
                /* dynamicRangeControlOn */
                let num_drc_feature_requests = h_bs.read_bits(3); /* numDrcFeatureRequests */

                for _ in 0..num_drc_feature_requests {
                    let drc_feature_request_type = h_bs.read_bits(2); /* drcFeatureRequestType */

                    match drc_feature_request_type {
                        0 => {
                            let num_drc_effect_type_requests = h_bs.read_bits(4); /* numDrcEffectTypeRequests */
                            h_bs.read_bits(4); /* numDrcEffectTypeRequestsDesired, forced to 1 */
                            if num_drc_effect_type_requests > 0 {
                                for j in 0..num_drc_effect_type_requests {
                                    let drc_effect_type_request = h_bs.read_bits(4); /* drcEffectTypeRequest */
                                    if j == 0 {
                                        drc_effect_type = drc_effect_type_request;
                                    } else if j < 8 {
                                        drc_effect_type |= drc_effect_type_request << (4 * j);
                                    }
                                    /* further drcEffectTypeRequest entries are ignored */
                                }
                            } else {
                                /* numDrcEffectTypeRequests == 0 means: use DRC selection of
                                 * decoder API */
                                drc_effect_type = UI_MANAGER_USE_DEFAULT_DRC_SELECTED;
                            }
                        }
                        1 => {
                            h_bs.read_bits(2); /* dynRangeMeasurementRequestType */

                            if h_bs.read_bit() != 0 {
                                /* dynRangeRequestedIsRange */
                                h_bs.read_bits(8);
                            } else {
                                h_bs.read_bits(8);
                                h_bs.read_bits(8);
                            }
                        }
                        2 => {
                            h_bs.read_bits(7); /* drcCharacteristicRequest */
                        }
                        _ => return 1,
                    }
                }
            } else {
                /* dynamicRangeControlOn = 0 */
                drc_effect_type = UI_MANAGER_DRC_OFF; /* signal "DRC off" */
            }
        }

        /* dynamicRangeControlParameterInterface */
        if h_bs.read_bit() != 0 {
            let change_compress = h_bs.read_bit();
            let change_boost = h_bs.read_bit();

            if change_compress != 0 {
                let tmp = h_bs.read_bits(8) as i32;
                if tmp < 255 {
                    compress = 256 - tmp;
                } else {
                    compress = 0;
                }
            }

            if change_boost != 0 {
                let tmp = h_bs.read_bits(8) as i32;
                if tmp < 255 {
                    boost = 256 - tmp;
                } else {
                    boost = 0;
                }
            }

            if h_bs.read_bit() != 0 {
                /* changeDrcCharacteristicTarget */
                h_bs.read_bits(8); /* drcCharacteristicTarget */
            }
        }

        /* uniDrcInterfaceExtension */
        if h_bs.read_bit() != 0 {
            let mut i = 0;

            let mut uni_drc_interface_ext_type = h_bs.read_bits(4); /* uniDrcInterfaceExtType[i] */

            while uni_drc_interface_ext_type != 0x0 {
                /* UNIDRCINTERFACEEXT_TERM = 0x0 */
                let tmp = h_bs.read_bits(4) + 4;
                let ext_bit_size = h_bs.read_bits(tmp) + 1; /* extSizeBits */

                /* add future extensions here */
                h_bs.push_for(ext_bit_size);

                i += 1;
                if i >= 8 {
                    /* 8 = 8 */
                    return 1;
                }
                uni_drc_interface_ext_type = h_bs.read_bits(4); /* uniDrcInterfaceExtType[i] */
            }
        }

        /* transfer DRC parameters */
        if target_loudness != -1 {
            self.drc_status.target_loudness = (-target_loudness >> 5) as i8;
        } else {
            /* use target loudness selection of decoder API. Loudness normalization is mandatory */
            self.drc_status.target_loudness = UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS;
        }

        if drc_effect_type != 0x80000000 {
            self.drc_status.drc_selected = drc_effect_type;
        } else {
            /* drcEffectType not set */
            self.drc_status.drc_selected = UI_MANAGER_USE_DEFAULT_DRC_SELECTED;
        }

        if album_mode != -1 {
            self.drc_status.album_mode = album_mode as i8;
        } else {
            self.drc_status.album_mode = -1;
        }
        if boost != -1 {
            self.drc_status.boost = (boost << 6) as i16;
        } else {
            self.drc_status.boost = -1;
        }
        if compress != -1 {
            self.drc_status.compress = (compress << 6) as i16;
        } else {
            self.drc_status.compress = -1;
        }

        self.drc_status_valid = 1;

        0
    }

    /// `mpegh3daSetContentIdentifier(handle, hBs, mhasPacketLength)`
    fn mpegh3da_set_content_identifier(
        &mut self,
        h_bs: &mut FdkBitStream,
        mhas_packet_length: u32,
    ) -> i32 {
        let mut id_bytes = mhas_packet_length.wrapping_sub(2);
        let mut uuid = [0u8; 16];

        if id_bytes > 16 {
            id_bytes = 16;
        }

        for i in 0..16u32 {
            if i >= (16 - id_bytes) {
                uuid[i as usize] = h_bs.read_bits(8) as u8;
            } else {
                uuid[i as usize] = 0;
            }
        }

        if self.ui_manager_active != 0 {
            /* ignore error return here (UI error should not trigger a sync error) */
            self.h_ui_manager.set_uuid(&uuid, 1);
        }

        0
    }

    /// `mpegh3daInteractivity(user_data, hBs, pac_type, mhasPacketLength)`: the user interaction
    /// callback of the transport decoder.
    pub(crate) fn mpegh3da_interactivity(
        &mut self,
        h_bs: &mut FdkBitStream,
        pac_type: MhaPactyp,
        mhas_packet_length: u32,
    ) -> i32 {
        match pac_type {
            MHA_PACTYP_USERINTERACTION => self.mpegh3da_element_interaction(h_bs),
            MHA_PACTYP_LOUDNESS_DRC => self.mpegh3da_loudness_drc_interface(h_bs),
            MHA_PACTYP_MARKER => self.mpegh3da_set_content_identifier(h_bs, mhas_packet_length),
            _ => 1,
        }
    }

    /// `FDK_mpeghUiInitialize(self)`. The user interaction callback is always registered (the
    /// transport decoder calls `mpegh3daInteractivity()` through
    /// [`mpeghdec_tpdec::TpCallbacks::user_interact`]).
    pub fn fdk_mpegh_ui_initialize(&mut self, ascs: &[CSAudioSpecificConfig]) -> AacDecoderError {
        /* Enabled automatic UI manager mode by default */
        self.ui_manager_enabled = -1;
        self.ui_manager_active = 1;
        self.ui_status_valid = 0;
        self.drc_status_valid = 0;
        self.ui_signal_changed = 0;
        self.ui_status = UserInteractivityStatus::default();
        self.ui_status_next = UserInteractivityStatus::default();
        self.update_on_off_flags(ascs);

        AAC_DEC_OK
    }

    /// `updateUiStatus(self)`
    pub fn update_ui_status(&mut self, ascs: &[CSAudioSpecificConfig]) -> i32 {
        let mut valid = 0;

        if self.ui_manager_enabled != 0 {
            if self.ui_manager_enabled > 0 {
                self.ui_manager_active = 1;
            } else if let Some(usc0) = self.usc(ascs, 0) {
                if usc0.ui_manager_active >= 0 {
                    self.ui_manager_active = usc0.ui_manager_active as u8;
                }
            }
        } else {
            self.ui_manager_active = 0;
        }

        if self.ui_manager_active == 0 && self.ui_status_valid != 0 {
            /* check availability */
            let p_asi = &self.h_ui_manager.asi;
            for grp_idx in 0..self.ui_status_next.num_groups as usize {
                if self.ui_status_next.group_data[grp_idx].on_off != 0
                    && p_asi.groups[grp_idx].is_available == 0
                {
                    self.ui_status_valid = -1;
                    if self.ui_manager_enabled != 0 {
                        self.ui_manager_active = 1;
                    }
                    valid = -1;
                    break;
                }
            }
        }

        self.h_ui_manager.set_is_active(self.ui_manager_active);

        if self.ui_manager_active != 0 {
            if self.h_ui_manager.get_interactivity_status(
                &mut self.ui_status_next,
                Some(&mut self.ui_signal_changed),
            ) == UI_MANAGER_OK
            {
                valid = 1;
            } else {
                valid = -1;
            }
        }
        if self.ui_status_valid > 0 {
            valid = 1;
        }

        if valid < 0 {
            return -1;
        }

        if valid == 0 {
            return 1;
        }

        if self.ui_signal_changed == 0 || self.use_element_skipping == 0 {
            self.ui_status = self.ui_status_next;
            self.update_on_off_flags(ascs);
        }

        0
    }

    /// `updateOnOffFlags(self)`
    pub fn update_on_off_flags(&mut self, ascs: &[CSAudioSpecificConfig]) {
        let Some(usc0) = self.usc(ascs, 0) else {
            return;
        };

        /* clear skipped signals flags */
        self.signal_skipped = [0; TP_MPEGH_MAX_SIGNAL_GROUPS];

        /* check if element skipping active */
        if self.use_element_skipping == 0
            || (self.flags[0] & AC_MPEGH3DA == 0)
            || usc0.element_length_present == 0
        {
            return;
        }

        /* get ASI */
        let p_asi = &self.h_ui_manager.asi;
        if p_asi.num_groups == 0 {
            return;
        }

        /* loop over groups */
        for grp in 0..self.ui_status.num_groups as usize {
            /* if group is off ... */
            if self.ui_status.group_data[grp].on_off != 0 {
                continue;
            }
            let asi_grp_idx =
                asi_group_id2idx(p_asi, self.ui_status.group_data[grp].group_id as i32);
            if asi_grp_idx < 0 {
                continue;
            }
            let g = &p_asi.groups[asi_grp_idx as usize];

            /* ... and group is switch group member ... */
            if g.switch_group_id != 255 {
                /* ... set skipped flag for signals in group */
                for grp_mem in 0..g.num_members as usize {
                    /* determine signal index from MAE ID */
                    let mut mae_id = g.meta_data_element_id[grp_mem] as i32;
                    let mut sig_idx = 0i32;

                    for stream_index in 0..TPDEC_MAX_TRACKS {
                        let Some(usc_s) = self.usc(ascs, stream_index) else { break };
                        for sig_grp_idx in 0..usc_s.bs_num_signal_groups as usize {
                            if usc0.m_signal_group_type[sig_grp_idx].type_ != 3 {
                                /* not HOA */
                                let count = usc_s.m_signal_group_type[sig_grp_idx].count as i32;
                                if mae_id >= count {
                                    mae_id -= count;
                                    sig_idx += count;
                                } else {
                                    sig_idx += mae_id;
                                    mae_id = 0;
                                    break;
                                }
                            }
                        }
                    }

                    /* set skipped flag */
                    if mae_id == 0 {
                        if let Some(f) = self.signal_skipped.get_mut(sig_idx as usize) {
                            *f = 1;
                        }
                    }
                }
            }
        }
    }

    /// `getOnOffFlag(self, signal)`
    #[inline]
    ///
    /// Signals beyond the array (undefined behaviour in C++, malformed configs only) are on.
    pub fn get_on_off_flag(&self, signal: i32) -> i32 {
        match self.signal_skipped.get(signal as usize) {
            Some(&s) => (s == 0) as i32,
            None => {
                mpeghdec_fdk::deviation::hit();
                1
            }
        }
    }

    /// `applyUserInteractivity(self, pTimeData)`: apply user interactivity, except for the final
    /// `FDK_drcDec_SetChannelGains()` call, which the rendering does
    /// (`RenderState::render()`) with the returned channel gains (`None` if the
    /// function returned before). The render state must be available
    /// ([`AacDecoderInstance::join_render`]).
    pub fn apply_user_interactivity(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
    ) -> (AacDecoderError, Option<[FixpDbl; 28 * 2]>) {
        let mut drc_status = UiDrcLoudnessStatus::default();
        let mut channel_gain = [0 as FixpDbl; 28 * 2]; /* e = 8 */
        let mut metadata_element_id_last: i32 = 0;
        let mut mae_id_start_ch = [0u8; 28 * 2];
        let mut mae_id_stop_ch = [0u8; 28 * 2];
        let mut mae_id_obj_grp = [255u8; 28 * 2];
        let mut mae_id_obj_idx = [0u8; 28 * 2];
        let mut valid;

        /* generate lookup tables */
        let mut obj_grp: u8 = 0;
        let mut signals_skipped: i32 = 0;
        let mut signals_prev_streams: i32 = 0;
        for stream_index in 0..TPDEC_MAX_TRACKS {
            let Some(usc) = self.usc(ascs, stream_index) else { break };
            let mut metadata_element_id =
                self.h_ui_manager.asi.meta_data_element_id_offset[stream_index] as i32;
            for grp in 0..usc.bs_num_signal_groups as usize {
                let sg = &usc.m_signal_group_type[grp];
                match sg.type_ {
                    0 => {
                        let mut id = 0i32;
                        for id_idx in 0..sg.count as i32 {
                            if self.get_on_off_flag(
                                sg.first_sig_idx as i32 + signals_prev_streams + id_idx,
                            ) == 0
                            {
                                signals_skipped += 1;
                                metadata_element_id += 1;
                                continue;
                            }
                            if metadata_element_id >= (28 * 2) {
                                return (AAC_DEC_UNSUPPORTED_CHANNELCONFIG, None);
                            }
                            mae_id_start_ch[metadata_element_id as usize] =
                                (sg.first_sig_idx as i32 + signals_prev_streams + id
                                    - signals_skipped) as u8;
                            mae_id_stop_ch[metadata_element_id as usize] =
                                (sg.first_sig_idx as i32 + signals_prev_streams + id + 1
                                    - signals_skipped) as u8;
                            metadata_element_id += 1;
                            id += 1;
                        }
                    }
                    1 => {
                        let mut id = 0i32;
                        let mut obj_grp_skipped = 1;
                        for id_idx in 0..sg.count as i32 {
                            if self.get_on_off_flag(
                                sg.first_sig_idx as i32 + signals_prev_streams + id_idx,
                            ) == 0
                            {
                                signals_skipped += 1;
                                metadata_element_id += 1;
                                continue;
                            }
                            match self
                                .render
                                .as_deref()
                                .expect("render state")
                                .hg_vbap_renderer
                                .get(obj_grp as usize)
                                .and_then(|r| r.as_deref())
                            {
                                Some(r) if id < r.num_objects as i32 => {}
                                _ => return (AAC_DEC_UNSUPPORTED_CHANNELCONFIG, None),
                            }
                            if metadata_element_id >= (28 * 2) {
                                return (AAC_DEC_UNSUPPORTED_CHANNELCONFIG, None);
                            }
                            mae_id_start_ch[metadata_element_id as usize] =
                                (sg.first_sig_idx as i32 + signals_prev_streams + id
                                    - signals_skipped) as u8;
                            mae_id_stop_ch[metadata_element_id as usize] =
                                (sg.first_sig_idx as i32 + signals_prev_streams + id + 1
                                    - signals_skipped) as u8;
                            mae_id_obj_grp[metadata_element_id as usize] = obj_grp;
                            mae_id_obj_idx[metadata_element_id as usize] = id as u8;
                            metadata_element_id += 1;
                            id += 1;
                            obj_grp_skipped = 0;
                        }
                        if obj_grp_skipped == 0 {
                            obj_grp = obj_grp.wrapping_add(1);
                        }
                    }
                    2 => return (AAC_DEC_UNSUPPORTED_FORMAT, None), /* SAOC not supported */
                    3 => return (AAC_DEC_UNSUPPORTED_FORMAT, None),
                    _ => {}
                }
            }
            signals_prev_streams += self.asc_channels[stream_index];
            metadata_element_id_last = metadata_element_id_last.max(metadata_element_id);
        }

        if metadata_element_id_last > (28 * 2) {
            return (AAC_DEC_UNSUPPORTED_CHANNELCONFIG, None);
        }

        let result = self.update_ui_status(ascs);

        valid = 0;
        if result == 0 {
            valid = 1;
        } else if result < 0 && self.h_ui_manager.asi.num_groups > 0 {
            return (AAC_DEC_DECODE_FRAME_ERROR, None);
        }

        if self.use_element_skipping != 0
            && self.ui_signal_changed != 0
            && (self.flush_status == AACDEC_FLUSH_OFF)
            && (self.build_up_status == AACDEC_BUILD_UP_OFF)
            && (self.render.as_deref().expect("render state").truncate_stop_offset[0] == -128)
        {
            /* make sure no config change is in progress before forcing one */
            self.tp_set_param(TPDEC_PARAM_FORCE_CONFIG_CHANGE, 0);
            self.render.as_deref_mut().expect("render state").apply_crossfade |=
                AACDEC_CROSSFADE_BITMASK_UI; /* cross-fade shall be applied */
        }

        if valid != 0 {
            let p_asi = &self.h_ui_manager.asi;

            let loudness_compensation_gain_db = get_loudness_compensation_gain_db(
                p_asi,
                &self.ui_status,
                self.render
                    .as_deref()
                    .expect("render state")
                    .h_uni_drc_decoder
                    .get_selection_process_output(),
            );

            for ch in 0..self.stream_info.num_channels.max(0) as usize {
                channel_gain[ch] = loudness_compensation_gain_db;
            }

            for i in 0..self.ui_status.num_groups as usize {
                let on_off;
                let mut gain_db: FixpDbl = 0;
                let mut az_offset: i32 = 0;
                let mut el_offset: i32 = 0;

                /* get group index */
                let grp_idx = asi_group_id2idx(p_asi, self.ui_status.group_data[i].group_id as i32);
                if grp_idx < 0 {
                    continue;
                }
                let grp_idx = grp_idx as usize;

                /* group preset gain and position offset */
                if self.ui_status.interaction_mode != 0 {
                    let preset_idx =
                        asi_group_preset_id2idx(p_asi, self.ui_status.group_preset_id as i32);

                    if preset_idx >= 0 {
                        let gp = &p_asi.group_presets[preset_idx as usize];
                        let conditions: &[AsiGroupPresetCondition] =
                            if gp.has_downmix_id_extension != 0 {
                                &gp.downmix_id_extension.conditions
                                    [..gp.downmix_id_extension.num_conditions as usize]
                            } else {
                                &gp.conditions[..gp.num_conditions as usize]
                            };

                        for cond in conditions {
                            if (cond.is_switch_group_condition == 0
                                && (cond.reference_id == self.ui_status.group_data[i].group_id))
                                || (cond.is_switch_group_condition != 0
                                    && (cond.reference_id == p_asi.groups[grp_idx].switch_group_id))
                            {
                                if cond.gain_flag != 0 {
                                    gain_db = ((cond.gain as FixpDbl - 255) << (31 - 8 - 1))
                                        + (32 << (31 - 8));
                                }

                                if cond.position_flag != 0 {
                                    az_offset = cond.az_offset as i32 - 128;
                                    el_offset = cond.el_offset as i32 - 32;
                                }
                            }
                        }
                    }
                }

                /* get on/off and gain */
                let gd = &self.ui_status.group_data[i];
                if gd.on_off == 0 || (gd.change_gain != 0 && (gd.gain == 0)) {
                    on_off = 0;
                } else {
                    on_off = 1;

                    if gd.change_gain != 0 {
                        gain_db = (gd.gain as i32 - 64).clamp(-63, 31) << (31 - 8);
                    }
                }

                if gd.change_position != 0 {
                    az_offset = gd.az_offset as i32 - 128;
                    el_offset = gd.el_offset as i32 - 32;
                }

                /* convert position offset */
                az_offset = (-120).max(120.min(az_offset));
                let az_offset_conv: FixpDbl = f_mult(
                    (az_offset << 8) as FixpSgl,
                    fl2fxconst_sgl((1.5f32 / 180.0f32 * 64.0f32) as f64),
                ) << 1;

                el_offset = (-30).max(30.min(el_offset));
                let el_offset_conv: FixpDbl = f_mult(
                    (el_offset << 10) as FixpSgl,
                    fl2fxconst_sgl((3.0f32 / 180.0f32 * 32.0f32) as f64),
                );

                /* apply to group members */
                for j in 0..p_asi.groups[grp_idx].num_members as usize {
                    let mae_id = p_asi.groups[grp_idx].meta_data_element_id[j] as i32;

                    if mae_id >= metadata_element_id_last {
                        continue;
                    }
                    let mae_id = mae_id as usize;

                    /* gain */
                    for ch in mae_id_start_ch[mae_id] as usize..mae_id_stop_ch[mae_id] as usize {
                        if channel_gain[ch] == MINVAL_DBL {
                            /* sanity check to prevent underflow */
                            continue;
                        }
                        if on_off != 0 {
                            channel_gain[ch] = f_add_saturate(channel_gain[ch], gain_db);
                        } else {
                            channel_gain[ch] = MINVAL_DBL; /* -inf dB */
                        }
                    }

                    /* position offset */
                    if mae_id_obj_grp[mae_id] != 255 {
                        if let Some(r) =
                            self.render.as_deref_mut().expect("render state").hg_vbap_renderer
                                [mae_id_obj_grp[mae_id] as usize]
                                .as_deref_mut()
                        {
                            let oi = mae_id_obj_idx[mae_id] as usize;
                            for k in 0..r.num_oam_frames.max(0) as usize {
                                let s = &mut r.oam_samples[k][oi].sph;
                                s.azi = (s.azi as i64 + az_offset_conv as i64) as i32;
                                s.ele = (s.ele as i64 + el_offset_conv as i64) as i32;
                            }
                        }
                    }
                }
            }
            {
                /* transfer groupPreset and active groups to uniDrc decoder */
                let mut num_group_ids_requested = 0usize;
                let mut group_preset_id_requested = -1;
                let mut group_ids_requested = [0i32; ASI_MAX_GROUPS];

                if self.ui_status.interaction_mode != 0 {
                    group_preset_id_requested = self.ui_status.group_preset_id as i32;
                }
                for i in 0..self.ui_status.num_groups as usize {
                    if self.ui_status.group_data[i].on_off != 0 {
                        group_ids_requested[num_group_ids_requested] =
                            self.ui_status.group_data[i].group_id as i32;
                        num_group_ids_requested += 1;
                    }
                }
                self.render
                    .as_deref_mut()
                    .expect("render state")
                    .h_uni_drc_decoder
                    .set_selection_process_mpegh_parameters_simple(
                        group_preset_id_requested,
                        num_group_ids_requested as i32,
                        &group_ids_requested,
                    );
            }
        }

        /* transfer selected DRC effect and target loudness to uniDrc decoder */
        valid = 0;
        if self.ui_manager_active != 0
            && self.h_ui_manager.get_drc_loudness_status(&mut drc_status) == UI_MANAGER_OK
        {
            valid = 1;
        }
        if self.drc_status_valid != 0 {
            drc_status = self.drc_status;
            valid = 1;
        }
        if valid != 0 {
            let drc = &mut self.render.as_deref_mut().expect("render state").h_uni_drc_decoder;
            if drc_status.target_loudness == UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS {
                /* use target loudness that was set with the API */
                if self.default_target_loudness >= 0 {
                    if drc.set_param(DRC_DEC_LOUDNESS_NORMALIZATION_ON, 1) != 0 {
                        return (AAC_DEC_SET_PARAM_FAIL, None);
                    }
                    if drc.set_param(
                        DRC_DEC_TARGET_LOUDNESS,
                        (-(self.default_target_loudness as i32))
                            .wrapping_mul(fl2fxconst_dbl((1.0f32 / (1 << 9) as f32) as f64)),
                    ) != 0
                    {
                        return (AAC_DEC_SET_PARAM_FAIL, None);
                    }
                } else {
                    /* Loudness Normalization off*/
                    if drc.set_param(DRC_DEC_LOUDNESS_NORMALIZATION_ON, 0) != 0 {
                        return (AAC_DEC_SET_PARAM_FAIL, None);
                    }
                }
            } else {
                /* use target loudness from UI manager */
                if drc.set_param(DRC_DEC_LOUDNESS_NORMALIZATION_ON, 1) != 0 {
                    return (AAC_DEC_SET_PARAM_FAIL, None);
                }
                if drc.set_param(
                    DRC_DEC_TARGET_LOUDNESS,
                    (drc_status.target_loudness as i32)
                        .wrapping_mul(fl2fxconst_dbl((1.0f32 / (1 << 7) as f32) as f64)),
                ) != 0
                {
                    return (AAC_DEC_SET_PARAM_FAIL, None);
                }
            }

            if drc_status.drc_selected == UI_MANAGER_USE_DEFAULT_DRC_SELECTED {
                /* use DRC effect type request that was set with the API */
                if drc.set_param(
                    DRC_DEC_EFFECT_TYPE_FALLBACK_CODE,
                    get_drc_effect_type_fallback(self.default_drc_set_effect as i32),
                ) != 0
                {
                    return (AAC_DEC_SET_PARAM_FAIL, None);
                }
            } else {
                /* use DRC effect type request from UI manager */
                if drc.set_param(DRC_DEC_EFFECT_TYPE_FALLBACK_CODE, drc_status.drc_selected as i32)
                    != 0
                {
                    return (AAC_DEC_SET_PARAM_FAIL, None);
                }
            }

            if drc_status.album_mode != -1
                && drc.set_param(DRC_DEC_ALBUM_MODE, drc_status.album_mode as FixpDbl) != 0
            {
                return (AAC_DEC_SET_PARAM_FAIL, None);
            }

            if drc_status.boost != -1
                && drc.set_param(DRC_DEC_BOOST, fx_sgl2fx_dbl(drc_status.boost)) != 0
            {
                return (AAC_DEC_SET_PARAM_FAIL, None);
            }

            if drc_status.compress != -1
                && drc.set_param(DRC_DEC_COMPRESS, fx_sgl2fx_dbl(drc_status.compress)) != 0
            {
                return (AAC_DEC_SET_PARAM_FAIL, None);
            }

            self.drc_status = drc_status;
        }

        /* set UI gains (in dB) which will be applied in DRC decoder: done by the rendering */
        (AAC_DEC_OK, Some(channel_gain))
    }
}

/// `computeLoudnessCompensationGain(numGroups, includeGroup, groupLoudnessValueMissingFlag,
/// groupLoudness, groupGainDefaultDb, groupGainInteractivityDb, groupStateDefault,
/// groupStateInteractivity, minGain, maxGain)`: Table AMD11.2 of Amendmend 3 to MPEG-H standard
#[allow(clippy::too_many_arguments)]
pub fn compute_loudness_compensation_gain(
    num_groups: i32,
    include_group: &[u8; ASI_MAX_GROUPS],
    group_loudness_value_missing_flag: i32, /* group loudness value missing for one or more groups */
    group_loudness: &[FixpDbl; ASI_MAX_GROUPS],
    group_gain_default_db: &[u8; ASI_MAX_GROUPS], /* groupPresetGain in dB = 0.5 * (mae_groupPresetGain - 255) + 32 */
    group_gain_interactivity_db: &[u8; ASI_MAX_GROUPS], /* interactivity gain in dB = uiStatus.groupData[i].gain - 64; */
    group_state_default: &[u8; ASI_MAX_GROUPS],
    group_state_interactivity: &[u8; ASI_MAX_GROUPS],
    min_gain: u8, /* loudnessCompPresetMinGain in dB = -3 * mae_bsLoudnessCompPresetMinGain */
    max_gain: u8, /* loudnessCompPresetMaxGain in dB = 3 * mae_bsLoudnessCompPresetMaxGain */
) -> FixpDbl {
    let mut loudness_reference: FixpDbl = 0;
    let mut loudness_after_interact: FixpDbl = 0;
    let mut lr_e = 0;
    let mut lai_e = 0;
    let mut tmp1_e = 0;
    let mut tmp2_e = 0;
    let mut lcgp2_e = 0;
    let mut lcgdb_e = 0;

    /* compute components of loudness compensation gain */
    for n in 0..num_groups.max(0) as usize {
        let mut tmp1_db: FixpDbl =
            ((group_gain_default_db[n] as i32 - 255) << (31 - 8 - 1)).wrapping_add(32 << (31 - 8)); /* e = 8 */
        let mut tmp2_db: FixpDbl =
            (group_gain_interactivity_db[n] as i32 - 64).clamp(-63, 31) << (31 - 8); /* e = 8 */
        if group_loudness_value_missing_flag == 0 {
            tmp1_db = tmp1_db.wrapping_add(group_loudness[n]);
            tmp2_db = tmp2_db.wrapping_add(group_loudness[n]);
        }
        /* 10^(dB_val/10) = 2^(log2(10)/10*dB_val) */
        let c = fl2fxconst_dbl((0.3321928f32 * (1 << 1) as f32) as f64);
        let tmp1 = f2_pow(f_mult(tmp1_db, c), 8 - 1, &mut tmp1_e);
        let tmp2 = f2_pow(f_mult(tmp2_db, c), 8 - 1, &mut tmp2_e);

        if include_group[n] != 0 {
            if group_state_default[n] != 0 {
                let e = lr_e;
                loudness_reference = f_add_norm(loudness_reference, e, tmp1, tmp1_e, &mut lr_e);
            }
            if group_state_interactivity[n] != 0 {
                let e = lai_e;
                loudness_after_interact =
                    f_add_norm(loudness_after_interact, e, tmp2, tmp2_e, &mut lai_e);
            }
        }
    }

    if (loudness_reference <= 0) || (loudness_after_interact <= 0) {
        return 0;
    }

    if (lr_e == lai_e) && (loudness_after_interact == loudness_reference) {
        return 0;
    }

    let loudness_compensation_gain_pow2 =
        f_div_norm(loudness_reference, loudness_after_interact, &mut lcgp2_e);
    lcgp2_e += lr_e - lai_e;

    /* loudness compensation gain in dB */
    /* 10*log10(lin_val) = 20/log2(10)*log2(lin_val) */
    let mut loudness_compensation_gain_db = f_mult_div2(
        fl2fxconst_dbl(3.01029996f64 / (1 << 2) as f32 as f64),
        f_log2(loudness_compensation_gain_pow2, lcgp2_e, &mut lcgdb_e),
    );
    lcgdb_e += 2 + 1;
    loudness_compensation_gain_db = scale_value(loudness_compensation_gain_db, lcgdb_e - 8); /* e = 8 */

    let min_gain_db: FixpDbl =
        if min_gain == 15 { MINVAL_DBL } else { (-3 * min_gain as i32) << (31 - 8) };
    let max_gain_db: FixpDbl = (3 * max_gain as i32) << (31 - 8);

    /* clip loudness compensation gain to min/max gain */
    loudness_compensation_gain_db.min(max_gain_db).max(min_gain_db)
}

/// `getLoudnessCompensationGainDb(pASI, pUiStatus, pUniDrcSelProcOutput)`: Loudness Compensation
/// after Gain Interactivity according to Amendmend 3 to MPEG-H standard: Keep output loudness
/// after user interaction on groups. Returns the gain in dB (e = 8).
pub fn get_loudness_compensation_gain_db(
    p_asi: &AudioSceneInfo,
    p_ui_status: &UserInteractivityStatus,
    p_uni_drc_sel_proc_output: &SelProcOutput,
) -> FixpDbl {
    let mut group_loudness_value_missing_flag = 0;
    let mut include_group = [0u8; ASI_MAX_GROUPS];
    let mut group_state_default = [0u8; ASI_MAX_GROUPS];
    let mut group_state_interactivity = [0u8; ASI_MAX_GROUPS];
    let mut group_loudness = [0 as FixpDbl; ASI_MAX_GROUPS]; /* e = 8 */
    let mut min_gain: u8;
    let mut max_gain: u8;
    let mut group_gain_default_db = [0u8; ASI_MAX_GROUPS];
    let mut group_gain_interactivity_db = [0u8; ASI_MAX_GROUPS];
    let p_lcd = &p_asi.loudness_compensation_data;
    let num_groups = p_asi.num_groups as usize;

    if p_asi.loud_comp_present == 0 {
        return 0;
    }

    if p_lcd.group_loudness_present != 0 {
        for g in 0..num_groups {
            /* loudnessCompGroupLoudness in dB = 0.25 * mae_bsLoudnessCompGroupLoudness - 57.75*/
            group_loudness[g] = (p_lcd.group_loudness[g] as FixpDbl - 231) << (31 - 8 - 2);
            /* e = 8 */
        }
    } else {
        let mut group_loudness_available = 0;
        for g in 0..num_groups {
            group_loudness[g] = get_group_loudness(
                Some(p_uni_drc_sel_proc_output),
                p_asi.groups[g].group_id as i32,
                &mut group_loudness_available,
            ) >> 1; /* e = 8 */
            if group_loudness_available == 0 {
                /* loudness for this group not found */
                group_loudness_value_missing_flag = 1;
                break;
            }
        }
    }

    /* fallbacks for non-present cases */
    for g in 0..num_groups {
        include_group[g] = 1;
        group_gain_default_db[g] = 191; /* 0 dB */
    }
    min_gain = 15; /* -inf dB */
    max_gain = 7; /* 21 dB */

    /* get default state and current interactivity gain and state */
    for g in 0..num_groups {
        group_state_default[g] = p_asi.groups[g].default_on_off;
        if p_ui_status.group_data[g].change_gain != 0 {
            group_gain_interactivity_db[g] = p_ui_status.group_data[g].gain;
        } else {
            group_gain_interactivity_db[g] = 64; /* 0 dB */
        }
        if (p_ui_status.group_data[g].change_gain != 0) && (p_ui_status.group_data[g].gain == 0) {
            group_state_interactivity[g] = 0;
        } else {
            group_state_interactivity[g] = p_ui_status.group_data[g].on_off;
        }
    }
    for i in 0..p_asi.num_switch_groups as usize {
        for j in 0..p_asi.switch_groups[i].num_members as usize {
            let grp_idx = asi_group_id2idx(p_asi, p_asi.switch_groups[i].member_id[j] as i32);
            if grp_idx >= 0 {
                /* index to group found */
                group_state_default[grp_idx as usize] = 0;
            }
        }
        let grp_idx = asi_group_id2idx(p_asi, p_asi.switch_groups[i].default_group_id as i32);
        if grp_idx >= 0 {
            /* index to group found */
            group_state_default[grp_idx as usize] = p_asi.switch_groups[i].default_on_off;
        }
    }

    if p_ui_status.interaction_mode != 0 {
        /* preset selected */
        let preset_idx = asi_group_preset_id2idx(p_asi, p_ui_status.group_preset_id as i32);
        if preset_idx >= 0 {
            /* index to groupPreset found */
            let preset_idx = preset_idx as usize;

            if p_lcd.preset_params_present[preset_idx] != 0 {
                include_group[..num_groups]
                    .copy_from_slice(&p_lcd.preset_include_group[preset_idx][..num_groups]);
                if p_lcd.preset_min_max_gain_present[preset_idx] != 0 {
                    min_gain = p_lcd.preset_min_gain[preset_idx];
                    max_gain = p_lcd.preset_max_gain[preset_idx];
                }
            }

            let gp = &p_asi.group_presets[preset_idx];
            let conditions: &[AsiGroupPresetCondition] = if gp.has_downmix_id_extension != 0 {
                &gp.downmix_id_extension.conditions
                    [..gp.downmix_id_extension.num_conditions as usize]
            } else {
                &gp.conditions[..gp.num_conditions as usize]
            };

            for cond in conditions {
                /* find condition matching to group */
                if cond.is_switch_group_condition != 0 {
                    let switch_grp_idx = asi_switch_group_id2idx(p_asi, cond.reference_id as i32);
                    if switch_grp_idx >= 0 {
                        /* index to switchGroup found */
                        let sg = &p_asi.switch_groups[switch_grp_idx as usize];
                        for i in 0..sg.num_members as usize {
                            let grp_idx = asi_group_id2idx(p_asi, sg.member_id[i] as i32);
                            if grp_idx >= 0 {
                                /* index to group found */
                                group_state_default[grp_idx as usize] = 0;
                                if cond.gain_flag != 0 {
                                    group_gain_default_db[grp_idx as usize] = cond.gain;
                                }
                            }
                        }
                        let grp_idx = asi_group_id2idx(p_asi, sg.default_group_id as i32);
                        if grp_idx >= 0 {
                            /* index to group found */
                            group_state_default[grp_idx as usize] = cond.condition_on_off;
                        }
                    }
                } else {
                    /* get group index */
                    let grp_idx = asi_group_id2idx(p_asi, cond.reference_id as i32);
                    if grp_idx >= 0 {
                        /* index to group found */
                        /* if group belongs to a switch group, deactivate all other members of
                         * this switch group */
                        for i in 0..p_asi.num_switch_groups as usize {
                            let sg = &p_asi.switch_groups[i];
                            let mut part_of_switch_group = false;
                            for j in 0..sg.num_members as usize {
                                if cond.reference_id == sg.member_id[j] {
                                    part_of_switch_group = true;
                                }
                            }
                            if part_of_switch_group {
                                for j in 0..sg.num_members as usize {
                                    let grp_idx2 = asi_group_id2idx(p_asi, sg.member_id[j] as i32);
                                    if grp_idx2 >= 0 {
                                        /* index to group found */
                                        group_state_default[grp_idx2 as usize] = 0;
                                        if cond.gain_flag != 0 {
                                            group_gain_default_db[grp_idx2 as usize] = cond.gain;
                                        }
                                    }
                                }
                            }
                        }
                        if cond.gain_flag != 0 {
                            group_gain_default_db[grp_idx as usize] = cond.gain;
                        }
                        group_state_default[grp_idx as usize] = cond.condition_on_off;
                    }
                }
            }
        }
    } else {
        /* default scene (no preset selected) */
        if p_lcd.default_params_present != 0 {
            include_group[..num_groups].copy_from_slice(&p_lcd.default_include_group[..num_groups]);
            if p_lcd.default_min_max_gain_present != 0 {
                min_gain = p_lcd.default_min_gain;
                max_gain = p_lcd.default_max_gain;
            }
        }
    }

    /* set group gain to default if changeGain is false */
    for g in 0..num_groups {
        if p_ui_status.group_data[g].change_gain == 0 {
            let tmp = (((group_gain_default_db[g] as i32 - 255) >> 1) + 32 + 64).max(0);
            group_gain_interactivity_db[g] = tmp as u8;
        }
    }

    compute_loudness_compensation_gain(
        p_asi.num_groups as i32,
        &include_group,
        group_loudness_value_missing_flag,
        &group_loudness,
        &group_gain_default_db,
        &group_gain_interactivity_db,
        &group_state_default,
        &group_state_interactivity,
        min_gain,
        max_gain,
    ) /* e = 8 */
}
