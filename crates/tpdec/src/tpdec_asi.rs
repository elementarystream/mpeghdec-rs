// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpegTPDec/src/tpdec_asi.cpp.

//! MPEG-H audio scene information (`mae_AudioSceneInfo()`, ISO/IEC 23008-3 15.2) parser.

use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::crc::FdkCrcInfo;

use crate::tp_data::*;
use crate::tpdec_lib::*;

const ID_MAE_GROUP_DESCRIPTION: i32 = 0;
const ID_MAE_SWITCHGROUP_DESCRIPTION: i32 = 1;
const ID_MAE_GROUP_CONTENT: i32 = 2;
const ID_MAE_GROUP_COMPOSITE: i32 = 3;
const ID_MAE_SCREEN_SIZE: i32 = 4;
const ID_MAE_GROUPPRESET_DESCRIPTION: i32 = 5;
const ID_MAE_DRC_UI_INFO: i32 = 6;
const ID_MAE_SCREEN_SIZE_EXTENSION: i32 = 7;
const ID_MAE_GROUP_PRESET_EXTENSION: i32 = 8;
const ID_MAE_LOUDNESS_COMPENSATION: i32 = 9;

const INVALID_ID: u8 = 255;

/// `compAssign(dest, src)`: compare and assign; returns true if the value changed.
#[inline]
fn comp_assign<T: PartialEq + Copy>(dest: &mut T, src: T) -> bool {
    let diff = *dest != src;
    *dest = src;
    diff
}

/// `FDKstrncpy(dest, src, 3)` (`strncpy()`: copy up to the first NUL, pad with NULs)
fn strncpy3(dest: &mut [i8; 3], src: &[i8; 3]) {
    let mut i = 0;
    while i < 3 && src[i] != 0 {
        dest[i] = src[i];
        i += 1;
    }
    while i < 3 {
        dest[i] = 0;
        i += 1;
    }
}

/// `FDKstrncmp(s1, s2, 3) == 0`
fn strncmp3_eq(s1: &[i8; 3], s2: &[i8; 3]) -> bool {
    for k in 0..3 {
        if s1[k] != s2[k] {
            return false;
        }
        if s1[k] == 0 {
            return true;
        }
    }
    true
}

/// `FDKtolower(c)` in the "C" locale
fn fdk_tolower(c: i32) -> i32 {
    if c < 0 || c > 255 {
        0
    } else if (b'A' as i32..=b'Z' as i32).contains(&c) {
        c + 32
    } else {
        c
    }
}

/// `FDKisalpha(c)` in the "C" locale
fn fdk_isalpha(c: i32) -> bool {
    if c < 0 || c > 255 {
        false
    } else {
        (c as u8).is_ascii_alphabetic()
    }
}

/// `asiGroupID2idx(asi, groupID)`: group index from group ID, -1 if not found.
pub fn asi_group_id2idx(asi: &AudioSceneInfo, group_id: i32) -> i32 {
    for i in 0..asi.num_groups as usize {
        if asi.groups[i].group_id as i32 == group_id {
            return i as i32;
        }
    }
    -1
}

/// `asiSwitchGroupID2idx(asi, switchGroupID)`: switch group index from switch group ID, -1 if not
/// found.
pub fn asi_switch_group_id2idx(asi: &AudioSceneInfo, switch_group_id: i32) -> i32 {
    for i in 0..asi.num_switch_groups as usize {
        if asi.switch_groups[i].switch_group_id as i32 == switch_group_id {
            return i as i32;
        }
    }
    -1
}

/// `asiGroupPresetID2idx(asi, groupPresetID)`: preset index from preset ID, -1 if not found.
pub fn asi_group_preset_id2idx(asi: &AudioSceneInfo, group_preset_id: i32) -> i32 {
    for i in 0..asi.num_group_presets as usize {
        if asi.group_presets[i].group_preset_id as i32 == group_preset_id {
            return i as i32;
        }
    }
    -1
}

/// `asiReset(asi)`: reset the ASI (in case of a parse error). Keeps the description storage
/// (cleared), `activeDmxId` and the preferred description languages.
pub fn asi_reset(asi: &mut AudioSceneInfo) {
    let mut pref_descr_languages_old = [[0i8; 3]; ASI_MAX_PREF_DESCR_LANGUAGES];
    let active_dmx_id_old = asi.active_dmx_id;
    let p_descriptions_old = asi.p_descriptions.take();

    for i in 0..ASI_MAX_PREF_DESCR_LANGUAGES {
        strncpy3(&mut pref_descr_languages_old[i], &asi.pref_descr_languages[i]);
    }

    *asi = AudioSceneInfo::default();

    if let Some(mut d) = p_descriptions_old {
        *d = AsiDescriptions::default();
        asi.p_descriptions = Some(d);
    }

    asi.active_dmx_id = active_dmx_id_old;
    for i in 0..ASI_MAX_PREF_DESCR_LANGUAGES {
        strncpy3(&mut asi.pref_descr_languages[i], &pref_descr_languages_old[i]);
    }
    for v in asi.meta_data_element_id_max_avail.iter_mut() {
        *v = -1;
    }
    asi.diff_flags = ASI_DIFF_NEEDS_RESET;
}

fn mae_group_definition(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
    num_max_element_ids: i32,
) -> TransportDecError {
    for grp in 0..asi.num_groups as usize {
        let mut diff = 0u32;
        let group = &mut asi.groups[grp];

        if comp_assign(&mut group.group_id, bs.read_bits(7) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if comp_assign(&mut group.allow_on_off, bs.read_bit() as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if comp_assign(&mut group.default_on_off, bs.read_bit() as u8) {
            diff |= ASI_DIFF_GROUP;
        }

        if comp_assign(&mut group.allow_position_interactivity, bs.read_bit() as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if group.allow_position_interactivity != 0 {
            if comp_assign(&mut group.interactivity_min_az_offset, bs.read_bits(7) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
            if comp_assign(&mut group.interactivity_max_az_offset, bs.read_bits(7) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
            if comp_assign(&mut group.interactivity_min_el_offset, bs.read_bits(5) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
            if comp_assign(&mut group.interactivity_max_el_offset, bs.read_bits(5) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
            if comp_assign(&mut group.interactivity_min_dist_factor, bs.read_bits(4) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
            if comp_assign(&mut group.interactivity_max_dist_factor, bs.read_bits(4) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
        }

        if comp_assign(&mut group.allow_gain_interactivity, bs.read_bit() as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if group.allow_gain_interactivity != 0 {
            if comp_assign(&mut group.interactivity_min_gain, bs.read_bits(6) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
            if comp_assign(&mut group.interactivity_max_gain, bs.read_bits(5) as u8) {
                diff |= ASI_DIFF_GROUP;
            }
        }

        if comp_assign(&mut group.num_members, (bs.read_bits(7) + 1) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if group.num_members as usize > ASI_MAX_GROUP_MEMBERS {
            asi.diff_flags |= diff;
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        group.has_conjunct_members = bs.read_bit() as u8;

        if group.has_conjunct_members != 0 {
            group.start_id = bs.read_bits(7) as u8;
            if group.start_id as i32 >= num_max_element_ids {
                asi.diff_flags |= diff;
                return TRANSPORTDEC_UNSUPPORTED_FORMAT;
            }

            for obj in 0..group.num_members as usize {
                if comp_assign(
                    &mut group.meta_data_element_id[obj],
                    group.start_id.wrapping_add(obj as u8),
                ) {
                    diff |= ASI_DIFF_NEEDS_RESET;
                }
                if group.meta_data_element_id[obj] as i32 >= num_max_element_ids {
                    asi.diff_flags |= diff;
                    return TRANSPORTDEC_UNSUPPORTED_FORMAT;
                }
            }
        } else {
            for obj in 0..group.num_members as usize {
                if comp_assign(&mut group.meta_data_element_id[obj], bs.read_bits(7) as u8) {
                    diff |= ASI_DIFF_NEEDS_RESET;
                }
                if group.meta_data_element_id[obj] as i32 >= num_max_element_ids {
                    asi.diff_flags |= diff;
                    return TRANSPORTDEC_UNSUPPORTED_FORMAT;
                }
            }
        }

        group.ext_present = 0;
        group.cont_present = 0;

        group.switch_group_id = INVALID_ID;
        // Presume that no groups are available initially.
        group.is_available = 0;

        asi.diff_flags |= diff;
        if asi.diff_flags & ASI_DIFF_NEEDS_RESET != 0 {
            if let Some(d) = asi.p_descriptions.as_deref_mut() {
                d.groups[grp].present = 0;
            }
        }
    }

    TRANSPORTDEC_OK
}

fn mae_group_availability_check(asi: &mut AudioSceneInfo) -> TransportDecError {
    for grp in 0..asi.num_groups as usize {
        let group = &asi.groups[grp];

        let mut group_detected = 0;
        for stream_index in 0..16 {
            if asi.meta_data_element_id_max_avail[stream_index] as i32
                >= asi.meta_data_element_id_offset[stream_index] as i32
            {
                for obj in 0..group.num_members as usize {
                    // Pass over previous groups
                    if (group.meta_data_element_id[obj] as i32)
                        < asi.meta_data_element_id_offset[stream_index] as i32
                        || (group.meta_data_element_id[obj] as i32)
                            > asi.meta_data_element_id_max_avail[stream_index] as i32
                    {
                        break;
                    }
                    group_detected += 1;
                }
            }
        }

        // A check for the case, when metaDataElementIDs are not consecutive
        if group_detected != 0 && group_detected != group.num_members as i32 {
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }

        if group_detected != 0 {
            asi.groups[grp].is_available = 1;
        }
    }

    TRANSPORTDEC_OK
}

/// `asiResetAvailability(asi)`
pub fn asi_reset_availability(asi: &mut AudioSceneInfo) {
    for v in asi.meta_data_element_id_max_avail.iter_mut() {
        *v = -1;
    }
    for i in 0..asi.num_groups as usize {
        asi.groups[i].is_available = 0;
    }
}

fn mae_drc_user_interface_info(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    let version = bs.read_2bits();

    if version == 0 {
        if comp_assign(
            &mut asi.drc_ui_info.num_target_loudness_conditions,
            (bs.read_bits(3) + 1) as u8,
        ) {
            asi.diff_flags |= ASI_DIFF_DRC_INFO;
        }
        if asi.drc_ui_info.num_target_loudness_conditions as usize
            > ASI_MAX_TARGET_LOUDNESS_CONDITIONS
        {
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }

        for c in 0..asi.drc_ui_info.num_target_loudness_conditions as usize {
            if comp_assign(
                &mut asi.drc_ui_info.bs_target_loudness_value_upper[c],
                bs.read_bits(6) as u8,
            ) {
                asi.diff_flags |= ASI_DIFF_DRC_INFO;
            }
            if comp_assign(
                &mut asi.drc_ui_info.drc_set_effect_available[c],
                bs.read_bits(16) as u16,
            ) {
                asi.diff_flags |= ASI_DIFF_DRC_INFO;
            }
        }
    } else {
        if comp_assign(&mut asi.drc_ui_info.num_target_loudness_conditions, 0) {
            asi.diff_flags |= ASI_DIFF_DRC_INFO;
        }
        // discard remaining bits signaled by mae_dataLength
    }

    TRANSPORTDEC_OK
}

fn mae_switch_group_definition(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    for swgrp in 0..asi.num_switch_groups as usize {
        {
            let sg = &mut asi.switch_groups[swgrp];
            let mut diff = 0;
            if comp_assign(&mut sg.switch_group_id, bs.read_bits(5) as u8) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }

            if comp_assign(&mut sg.allow_on_off, bs.read_bit() as u8) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }
            if sg.allow_on_off != 0 {
                if comp_assign(&mut sg.default_on_off, bs.read_bit() as u8) {
                    diff |= ASI_DIFF_SWITCHGRP;
                }
            } else {
                sg.default_on_off = 1;
            }

            if comp_assign(&mut sg.num_members, (bs.read_bits(5) + 1) as u8) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }
            asi.diff_flags |= diff;
        }
        if asi.switch_groups[swgrp].num_members as usize > ASI_MAX_SWITCH_GROUP_MEMBERS {
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        for grp in 0..asi.switch_groups[swgrp].num_members as usize {
            if comp_assign(&mut asi.switch_groups[swgrp].member_id[grp], bs.read_bits(7) as u8) {
                asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
            }

            let grp_idx = asi_group_id2idx(asi, asi.switch_groups[swgrp].member_id[grp] as i32);
            if grp_idx < 0 {
                return TRANSPORTDEC_PARSE_ERROR;
            }
            asi.groups[grp_idx as usize].switch_group_id = asi.switch_groups[swgrp].switch_group_id;
        }
        if comp_assign(&mut asi.switch_groups[swgrp].default_group_id, bs.read_bits(7) as u8) {
            asi.diff_flags |= ASI_DIFF_SWITCHGRP;
        }

        if asi.diff_flags & ASI_DIFF_NEEDS_RESET != 0 {
            if let Some(d) = asi.p_descriptions.as_deref_mut() {
                d.switch_groups[swgrp].present = 0;
            }
        }
    }

    TRANSPORTDEC_OK
}

/// Reads the gain/position part of a group preset condition (shared by the group preset
/// definition and its downmix ID extension). Returns the diff flags.
fn read_condition_on_off_part(c: &mut AsiGroupPresetCondition, bs: &mut FdkBitStream) -> u32 {
    let mut diff = 0;
    if comp_assign(&mut c.disable_gain_interactivity, bs.read_bit() as u8) {
        diff |= ASI_DIFF_NEEDS_RESET;
    }
    if comp_assign(&mut c.gain_flag, bs.read_bit() as u8) {
        diff |= ASI_DIFF_NEEDS_RESET;
    }
    if c.gain_flag != 0 && comp_assign(&mut c.gain, bs.read_bits(8) as u8) {
        diff |= ASI_DIFF_NEEDS_RESET;
    }

    if comp_assign(&mut c.disable_position_interactivity, bs.read_bit() as u8) {
        diff |= ASI_DIFF_NEEDS_RESET;
    }
    if comp_assign(&mut c.position_flag, bs.read_bit() as u8) {
        diff |= ASI_DIFF_NEEDS_RESET;
    }
    if c.position_flag != 0 {
        if comp_assign(&mut c.az_offset, bs.read_bits(8) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if comp_assign(&mut c.el_offset, bs.read_bits(6) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if comp_assign(&mut c.dist_factor, bs.read_bits(4) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
    }
    diff
}

fn mae_group_preset_definition(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    for gp in 0..asi.num_group_presets as usize {
        let mut diff = 0;
        let preset = &mut asi.group_presets[gp];

        if comp_assign(&mut preset.group_preset_id, bs.read_bits(5) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if comp_assign(&mut preset.kind, bs.read_bits(5) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }

        if comp_assign(&mut preset.num_conditions, (bs.read_bits(4) + 1) as u8) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        if preset.num_conditions as usize > ASI_MAX_GROUP_PRESET_CONDITIONS {
            asi.diff_flags |= diff;
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        for cnd in 0..preset.num_conditions as usize {
            let c = &mut preset.conditions[cnd];
            if comp_assign(&mut c.reference_id, bs.read_bits(7) as u8) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }
            if comp_assign(&mut c.condition_on_off, bs.read_bit() as u8) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }

            if c.condition_on_off != 0 {
                diff |= read_condition_on_off_part(c, bs);
            }
        }

        preset.ext_present = 0;

        preset.production_screen_size.has_non_standard_screen_size = 0;

        asi.diff_flags |= diff;
        if asi.diff_flags & ASI_DIFF_NEEDS_RESET != 0 {
            if let Some(d) = asi.p_descriptions.as_deref_mut() {
                d.group_presets[gp].present = 0;
            }
        }
    }

    TRANSPORTDEC_OK
}

fn mae_group_preset_definition_extension(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    let active_dmx_id = asi.active_dmx_id;
    for gp in 0..asi.num_group_presets as usize {
        let mut diff = 0;
        let preset = &mut asi.group_presets[gp];

        preset.ext_present = 1;

        let has_switch_group_conditions = bs.read_bit();
        if has_switch_group_conditions != 0 {
            for cnd in 0..preset.num_conditions as usize {
                if comp_assign(
                    &mut preset.conditions[cnd].is_switch_group_condition,
                    bs.read_bit() as u8,
                ) {
                    diff |= ASI_DIFF_NEEDS_RESET;
                }
            }
        } else {
            for cnd in 0..preset.num_conditions as usize {
                if comp_assign(&mut preset.conditions[cnd].is_switch_group_condition, 0) {
                    diff |= ASI_DIFF_NEEDS_RESET;
                }
            }
        }

        let has_downmix_id_group_preset_extensions = bs.read_bit();
        if has_downmix_id_group_preset_extensions != 0 {
            let mut found_dmx_id = 0u8;

            let num_downmix_id_group_preset_extensions = bs.read_bits(5);
            for _egp in 0..num_downmix_id_group_preset_extensions {
                let tmp_dmx_id = bs.read_bits(7) as u8;
                if tmp_dmx_id == active_dmx_id {
                    found_dmx_id = 1;

                    let ext = &mut preset.downmix_id_extension;
                    if comp_assign(&mut ext.downmix_id, tmp_dmx_id) {
                        diff |= ASI_DIFF_NEEDS_RESET;
                    }
                    if comp_assign(&mut ext.num_conditions, (bs.read_bits(4) + 1) as u8) {
                        diff |= ASI_DIFF_NEEDS_RESET;
                    }
                    if ext.num_conditions as usize > ASI_MAX_GROUP_PRESET_CONDITIONS {
                        asi.diff_flags |= diff;
                        return TRANSPORTDEC_UNSUPPORTED_FORMAT;
                    }

                    for cnd in 0..ext.num_conditions as usize {
                        let condition = &mut ext.conditions[cnd];

                        if comp_assign(
                            &mut condition.is_switch_group_condition,
                            bs.read_bit() as u8,
                        ) {
                            diff |= ASI_DIFF_NEEDS_RESET;
                        }
                        let bits = if condition.is_switch_group_condition != 0 { 5 } else { 7 };
                        if comp_assign(&mut condition.reference_id, bs.read_bits(bits) as u8) {
                            diff |= ASI_DIFF_NEEDS_RESET;
                        }

                        if comp_assign(&mut condition.condition_on_off, bs.read_bit() as u8) {
                            diff |= ASI_DIFF_NEEDS_RESET;
                        }
                        if condition.condition_on_off != 0 {
                            diff |= read_condition_on_off_part(condition, bs);
                        }
                    }
                } else {
                    let num_conditions = bs.read_bits(4) + 1;

                    for _cnd in 0..num_conditions {
                        let is_switch_group_condition = bs.read_bit();
                        if is_switch_group_condition != 0 {
                            bs.push_for(5);
                        } else {
                            bs.push_for(7);
                        }

                        let condition_on_off = bs.read_bit();
                        if condition_on_off != 0 {
                            bs.push_for(1);
                            let gain_flag = bs.read_bit();
                            if gain_flag != 0 {
                                bs.push_for(8);
                            }

                            bs.push_for(1);
                            let position_flag = bs.read_bit();
                            if position_flag != 0 {
                                bs.push_for(8 + 6 + 4);
                            }
                        }
                    }
                }
            }
            if comp_assign(&mut preset.has_downmix_id_extension, found_dmx_id) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }
        } else if comp_assign(&mut preset.has_downmix_id_extension, 0) {
            diff |= ASI_DIFF_NEEDS_RESET;
        }
        asi.diff_flags |= diff;
    }

    TRANSPORTDEC_OK
}

const NUM_SPECIAL_LANG_CODES: usize = 20 + 2;

/// `asiMapISO639_2_T2B_and_tolower(buf)`: the three letter language code (ISO 639-2) in `buf` is
/// replaced with an unambiguous representation: the T (terminology) version is replaced with the
/// B (bibliographic) version and all letters are converted to lower case.
pub fn asi_map_iso639_2_t2b_and_tolower(buf: &mut [i8; 3]) {
    // double possibilities according to ISO 639-2 T/B: { B version }, { T version }
    const LANG_CODES: [[&[u8; 3]; 2]; NUM_SPECIAL_LANG_CODES] = [
        [b"alb", b"sqi"],
        [b"arm", b"hye"],
        [b"baq", b"eus"],
        [b"tib", b"bod"],
        [b"bur", b"mya"],
        [b"cze", b"ces"],
        [b"chi", b"zho"],
        [b"wel", b"cym"],
        [b"ger", b"deu"],
        [b"dut", b"nld"],
        [b"gre", b"ell"],
        [b"per", b"fas"],
        [b"fre", b"fra"],
        [b"geo", b"kat"],
        [b"ice", b"isl"],
        [b"mac", b"mkd"],
        [b"mao", b"mri"],
        [b"may", b"msa"],
        [b"rum", b"ron"],
        [b"slo", b"slk"],
        // invalid input but map to valid ISO 639-2 language code
        [b"kor", b"ko\0"],
        // invalid input but map to valid ISO 639-2 language code
        [b"eng", b"en\0"],
    ];

    for c in buf.iter_mut() {
        *c = fdk_tolower(*c as i32) as i8;
    }

    for codes in LANG_CODES.iter() {
        let t = codes[1];
        if buf[0] == t[0] as i8 && buf[1] == t[1] as i8 && buf[2] == t[2] as i8 {
            for k in 0..3 {
                buf[k] = codes[0][k] as i8;
            }
            return;
        }
    }
}

/// `asiCheckISO639_2(buf)`: if `buf` contains an invalid ISO 639-2 language code replace it with
/// "und" (undetermined) and return 1, else return 0. Only a simple check is done (`isalpha()`).
pub fn asi_check_iso639_2(buf: &mut [i8; 3]) -> i32 {
    if !fdk_isalpha(buf[0] as i32) || !fdk_isalpha(buf[1] as i32) || !fdk_isalpha(buf[2] as i32) {
        buf[0] = b'u' as i8;
        buf[1] = b'n' as i8;
        buf[2] = b'd' as i8;
        return 1;
    }
    0
}

fn read_language(bs: &mut FdkBitStream) -> [i8; 3] {
    let mut language = [0i8; 3];
    language[0] = bs.read_bits(8) as u8 as i8;
    language[1] = bs.read_bits(8) as u8 as i8;
    language[2] = bs.read_bits(8) as u8 as i8;
    asi_map_iso639_2_t2b_and_tolower(&mut language);
    asi_check_iso639_2(&mut language); // ignore error to allow playback of the stream
    language
}

fn mae_description(
    type_: i32,
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    let num_description_blocks = bs.read_bits(7) as i32 + 1;
    if num_description_blocks > ASI_MAX_DESCRIPTION_BLOCKS as i32 {
        return TRANSPORTDEC_PARSE_ERROR;
    }

    for _n in 0..num_description_blocks {
        let mut curr_prio = ASI_MAX_PREF_DESCR_LANGUAGES as i32 + 1;

        let idx = if type_ == ID_MAE_GROUP_DESCRIPTION {
            let group_id = bs.read_bits(7) as i32;
            asi_group_id2idx(asi, group_id)
        } else if type_ == ID_MAE_SWITCHGROUP_DESCRIPTION {
            let switch_group_id = bs.read_bits(5) as i32;
            asi_switch_group_id2idx(asi, switch_group_id)
        } else {
            // ID_MAE_GROUPPRESET_DESCRIPTION
            let group_preset_id = bs.read_bits(5) as i32;
            asi_group_preset_id2idx(asi, group_preset_id)
        };
        if idx < 0 {
            return TRANSPORTDEC_PARSE_ERROR;
        }
        let idx = idx as usize;

        let pref = asi.pref_descr_languages;
        let mut diff = 0u32;
        let mut descr: Option<&mut AsiDescription> =
            asi.p_descriptions.as_deref_mut().map(|d| match type_ {
                ID_MAE_GROUP_DESCRIPTION => &mut d.groups[idx],
                ID_MAE_SWITCHGROUP_DESCRIPTION => &mut d.switch_groups[idx],
                _ => &mut d.group_presets[idx],
            });

        // get priority of current language
        if let Some(d) = descr.as_deref() {
            if d.present != 0 {
                curr_prio = 0;
                while curr_prio < ASI_MAX_PREF_DESCR_LANGUAGES as i32 {
                    let lang = &d.language[d.pref_lang_idx as usize];
                    if lang[0] != 0 && strncmp3_eq(lang, &pref[curr_prio as usize]) {
                        break;
                    }
                    curr_prio += 1;
                }
            }
        }

        let num_desc_languages = bs.read_bits(4) as i32 + 1;
        if num_desc_languages > ASI_MAX_DESCRIPTION_LANGUAGES as i32 {
            asi.diff_flags |= diff;
            return TRANSPORTDEC_PARSE_ERROR;
        }
        if let Some(d) = descr.as_deref_mut() {
            d.num_languages = num_desc_languages as i16;
        }
        for i in 0..num_desc_languages as usize {
            let language = read_language(bs);
            let data_length = bs.read_bits(8) as i32 + 1;

            if let Some(d) = descr.as_deref_mut() {
                // get priority of language
                let mut prio = 0;
                while prio < ASI_MAX_PREF_DESCR_LANGUAGES as i32 {
                    if language[0] != 0 && strncmp3_eq(&language, &pref[prio as usize]) {
                        break;
                    }
                    prio += 1;
                }
                if prio == ASI_MAX_PREF_DESCR_LANGUAGES as i32 {
                    prio += i as i32; // if not in preferred list select first transmitted
                }
                if prio == ASI_MAX_PREF_DESCR_LANGUAGES as i32
                    && d.present != 0
                    && !strncmp3_eq(&language, &d.language[d.pref_lang_idx as usize])
                {
                    prio += 1;
                }

                if prio <= curr_prio {
                    d.pref_lang_idx = i as i16;
                }

                d.present = 1;

                for k in 0..3 {
                    if comp_assign(&mut d.language[i][k], language[k]) {
                        diff |= ASI_DIFF_DESCRIPTION;
                    }
                }

                let mut j = 0usize;
                while (j as i32) < data_length {
                    if j >= ASI_MAX_STORED_DESCRIPTION_LEN {
                        break;
                    }

                    let mut c = bs.read_bits(8) as u8 as i8;

                    // prevent truncation within UTF-8 character
                    if (j == ASI_MAX_STORED_DESCRIPTION_LEN - 1 && (c as i32 & 0xC0) == 0xC0)
                        || (j == ASI_MAX_STORED_DESCRIPTION_LEN - 2 && (c as i32 & 0xE0) == 0xE0)
                        || (j == ASI_MAX_STORED_DESCRIPTION_LEN - 3 && (c as i32 & 0xF0) == 0xF0)
                    {
                        c = 0;
                    }

                    if comp_assign(&mut d.data[i][j], c) {
                        diff |= ASI_DIFF_DESCRIPTION;
                    }
                    j += 1;
                }
                if comp_assign(&mut d.data[i][j], 0) {
                    diff |= ASI_DIFF_DESCRIPTION; // terminate string
                }
                bs.push_for((8 * (data_length - j as i32)) as u32);

                curr_prio = prio;
            } else {
                // skip string
                bs.push_for((8 * data_length) as u32);
            }
        }
        asi.diff_flags |= diff;
    }

    TRANSPORTDEC_OK
}

fn mae_content_data(asi: &mut AudioSceneInfo, bs: &mut FdkBitStream) -> TransportDecError {
    let num_content_data_blocks = bs.read_bits(7) + 1;

    for _n in 0..num_content_data_blocks {
        let group_id = bs.read_bits(7) as i32;
        let i = asi_group_id2idx(asi, group_id);
        if i < 0 {
            return TRANSPORTDEC_PARSE_ERROR;
        }
        let group = &mut asi.groups[i as usize];
        let mut diff = 0;

        group.cont_present = 1;
        let content_data = &mut group.content_data;

        if comp_assign(&mut content_data.content_kind, bs.read_bits(4) as u8) {
            diff |= ASI_DIFF_CONTENT;
        }

        let has_content_language = bs.read_bit();
        let language = if has_content_language != 0 { read_language(bs) } else { [0i8; 3] };
        for k in 0..3 {
            if comp_assign(&mut content_data.content_language[k], language[k]) {
                diff |= ASI_DIFF_CONTENT;
            }
        }
        asi.diff_flags |= diff;
    }

    TRANSPORTDEC_OK
}

fn mae_composite_pair(asi: &mut AudioSceneInfo, bs: &mut FdkBitStream) -> TransportDecError {
    if comp_assign(&mut asi.num_composite_pairs, (bs.read_bits(7) + 1) as u8) {
        asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
    }
    if asi.num_composite_pairs as usize >= ASI_MAX_COMPOSITE_PAIRS {
        return TRANSPORTDEC_UNSUPPORTED_FORMAT;
    }
    for sa in 0..asi.num_composite_pairs as usize {
        if comp_assign(&mut asi.composite_pairs[sa][0], bs.read_bits(7) as u8) {
            asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
        }
        if comp_assign(&mut asi.composite_pairs[sa][1], bs.read_bits(7) as u8) {
            asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
        }
    }

    TRANSPORTDEC_OK
}

fn mae_production_screen_size_data(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    let mut diff = 0;
    let d = &mut asi.production_screen_size_data;
    if comp_assign(&mut d.has_non_standard_screen_size, bs.read_bit() as u8) {
        diff |= ASI_DIFF_SCRNSIZE;
    }
    if d.has_non_standard_screen_size != 0 {
        if comp_assign(&mut d.bs_screen_size_az, bs.read_bits(9) as i16) {
            diff |= ASI_DIFF_SCRNSIZE;
        }
        if comp_assign(&mut d.bs_screen_size_top_el, bs.read_bits(9) as i16) {
            diff |= ASI_DIFF_SCRNSIZE;
        }
        if comp_assign(&mut d.bs_screen_size_bottom_el, bs.read_bits(9) as i16) {
            diff |= ASI_DIFF_SCRNSIZE;
        }
    }
    asi.diff_flags |= diff;

    TRANSPORTDEC_OK
}

fn mae_production_screen_size_data_extension(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    let overwrite_production_screen_size_data = bs.read_bit();
    if overwrite_production_screen_size_data != 0 {
        if comp_assign(
            &mut asi.production_screen_size_data.bs_screen_size_left_az,
            bs.read_bits(10) as i16,
        ) {
            asi.diff_flags |= ASI_DIFF_SCRNSIZE;
        }
        if comp_assign(
            &mut asi.production_screen_size_data.bs_screen_size_right_az,
            bs.read_bits(10) as i16,
        ) {
            asi.diff_flags |= ASI_DIFF_SCRNSIZE;
        }
    }

    let num_preset_production_screens = bs.read_bits(5);
    for _n in 0..num_preset_production_screens {
        let group_preset_id = bs.read_bits(5) as i32;
        let i = asi_group_preset_id2idx(asi, group_preset_id);
        if i < 0 {
            return TRANSPORTDEC_PARSE_ERROR;
        }
        let mut diff = 0;
        let ps = &mut asi.group_presets[i as usize].production_screen_size;

        if comp_assign(&mut ps.has_non_standard_screen_size, bs.read_bit() as u8) {
            diff |= ASI_DIFF_SCRNSIZE;
        }
        if ps.has_non_standard_screen_size != 0 {
            if comp_assign(&mut ps.is_centered_in_azimuth, bs.read_bit() as u8) {
                diff |= ASI_DIFF_SCRNSIZE;
            }
            if ps.is_centered_in_azimuth != 0 {
                if comp_assign(&mut ps.bs_screen_size_az, bs.read_bits(9) as i16) {
                    diff |= ASI_DIFF_SCRNSIZE;
                }
            } else {
                if comp_assign(&mut ps.bs_screen_size_left_az, bs.read_bits(10) as i16) {
                    diff |= ASI_DIFF_SCRNSIZE;
                }
                if comp_assign(&mut ps.bs_screen_size_right_az, bs.read_bits(10) as i16) {
                    diff |= ASI_DIFF_SCRNSIZE;
                }
            }
            if comp_assign(&mut ps.bs_screen_size_top_el, bs.read_bits(9) as i16) {
                diff |= ASI_DIFF_SCRNSIZE;
            }
            if comp_assign(&mut ps.bs_screen_size_bottom_el, bs.read_bits(9) as i16) {
                diff |= ASI_DIFF_SCRNSIZE;
            }
        }
        asi.diff_flags |= diff;
    }

    TRANSPORTDEC_OK
}

fn mae_loudness_compensation_data(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
) -> TransportDecError {
    let num_groups = asi.num_groups as usize;
    let num_group_presets = asi.num_group_presets as usize;
    let lc = &mut asi.loudness_compensation_data;
    let mut diff = 0;

    if comp_assign(&mut lc.group_loudness_present, bs.read_bit() as u8) {
        diff |= ASI_DIFF_LCOMP;
    }
    if lc.group_loudness_present != 0 {
        for grp in 0..num_groups {
            if comp_assign(&mut lc.group_loudness[grp], bs.read_bits(8) as u8) {
                diff |= ASI_DIFF_LCOMP;
            }
        }
    }

    if comp_assign(&mut lc.default_params_present, bs.read_bit() as u8) {
        diff |= ASI_DIFF_LCOMP;
    }
    if lc.default_params_present != 0 {
        for grp in 0..num_groups {
            if comp_assign(&mut lc.default_include_group[grp], bs.read_bit() as u8) {
                diff |= ASI_DIFF_LCOMP;
            }
        }

        if comp_assign(&mut lc.default_min_max_gain_present, bs.read_bit() as u8) {
            diff |= ASI_DIFF_LCOMP;
        }
        if lc.default_min_max_gain_present != 0 {
            if comp_assign(&mut lc.default_min_gain, bs.read_bits(4) as u8) {
                diff |= ASI_DIFF_LCOMP;
            }
            if comp_assign(&mut lc.default_max_gain, bs.read_bits(4) as u8) {
                diff |= ASI_DIFF_LCOMP;
            }
        }
    }

    for gp in 0..num_group_presets {
        if comp_assign(&mut lc.preset_params_present[gp], bs.read_bit() as u8) {
            diff |= ASI_DIFF_LCOMP;
        }
        if lc.preset_params_present[gp] != 0 {
            for grp in 0..num_groups {
                if comp_assign(&mut lc.preset_include_group[gp][grp], bs.read_bit() as u8) {
                    diff |= ASI_DIFF_LCOMP;
                }
            }

            if comp_assign(&mut lc.preset_min_max_gain_present[gp], bs.read_bit() as u8) {
                diff |= ASI_DIFF_LCOMP;
            }
            if lc.preset_min_max_gain_present[gp] != 0 {
                if comp_assign(&mut lc.preset_min_gain[gp], bs.read_bits(4) as u8) {
                    diff |= ASI_DIFF_LCOMP;
                }
                if comp_assign(&mut lc.preset_max_gain[gp], bs.read_bits(4) as u8) {
                    diff |= ASI_DIFF_LCOMP;
                }
            }
        }
    }
    asi.diff_flags |= diff;

    TRANSPORTDEC_OK
}

/// Clears a description that is not present in the current ASI.
fn clear_description(description: &mut AsiDescription) -> u32 {
    let mut diff = 0;
    if comp_assign(&mut description.num_languages, 0) {
        diff |= ASI_DIFF_DESCRIPTION;
    }
    for i in 0..ASI_MAX_DESCRIPTION_LANGUAGES {
        if comp_assign(&mut description.language[i][0], 0) {
            diff |= ASI_DIFF_DESCRIPTION;
        }
        if comp_assign(&mut description.data[i][0], 0) {
            diff |= ASI_DIFF_DESCRIPTION;
        }
    }
    description.pref_lang_idx = 0;
    diff
}

fn uses_crc1(data_type: i32) -> bool {
    data_type == ID_MAE_GROUP_DESCRIPTION
        || data_type == ID_MAE_SWITCHGROUP_DESCRIPTION
        || data_type == ID_MAE_GROUPPRESET_DESCRIPTION
        || data_type == ID_MAE_GROUP_CONTENT
        || data_type == ID_MAE_DRC_UI_INFO
        || data_type == ID_MAE_GROUP_PRESET_EXTENSION
}

fn mae_data(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
    crc_info: &mut [FdkCrcInfo; 2],
) -> TransportDecError {
    let mut err = TRANSPORTDEC_OK;

    asi.comp_pairs_present = 0;
    asi.scrn_size_present = 0;
    asi.scrn_size_ext_present = 0;
    asi.loud_comp_present = 0;
    asi.scene_disp_present = 0;
    asi.drc_ui_info_present = 0;

    let num_data_sets = bs.read_bits(4);
    for _dscr in 0..num_data_sets {
        let mut crc_reg = [0i32; 2];

        let data_type = bs.read_bits(4) as i32;
        let data_length = bs.read_bits(16) as i32;

        let mut i = bs.get_valid_bits();

        if data_type == ID_MAE_GROUP_CONTENT {
            crc_reg[0] = crc_info[0].start_reg(bs, 0);
        }
        if uses_crc1(data_type) {
            crc_reg[1] = crc_info[1].start_reg(bs, 0);
        }

        match data_type {
            ID_MAE_GROUP_DESCRIPTION
            | ID_MAE_SWITCHGROUP_DESCRIPTION
            | ID_MAE_GROUPPRESET_DESCRIPTION => {
                err = mae_description(data_type, asi, bs);
            }
            ID_MAE_GROUP_CONTENT => {
                err = mae_content_data(asi, bs);
            }
            ID_MAE_GROUP_COMPOSITE => {
                err = mae_composite_pair(asi, bs);
                asi.comp_pairs_present = 1;
            }
            ID_MAE_SCREEN_SIZE => {
                err = mae_production_screen_size_data(asi, bs);
                asi.scrn_size_present = 1;
            }
            ID_MAE_DRC_UI_INFO => {
                err = mae_drc_user_interface_info(asi, bs);
                asi.drc_ui_info_present = 1;
            }
            ID_MAE_SCREEN_SIZE_EXTENSION => {
                err = mae_production_screen_size_data_extension(asi, bs);
                asi.scrn_size_ext_present = 1;
            }
            ID_MAE_GROUP_PRESET_EXTENSION => {
                err = mae_group_preset_definition_extension(asi, bs);
            }
            ID_MAE_LOUDNESS_COMPENSATION => {
                err = mae_loudness_compensation_data(asi, bs);
                asi.loud_comp_present = 1;
            }
            _ => {}
        }

        i = data_length * 8 - (i - bs.get_valid_bits());
        if i >= 0 {
            bs.push_for(i as u32);
        } else {
            return TRANSPORTDEC_PARSE_ERROR;
        }

        if data_type == ID_MAE_GROUP_CONTENT {
            crc_info[0].end_reg(bs, crc_reg[0]);
        }
        if uses_crc1(data_type) {
            crc_info[1].end_reg(bs, crc_reg[1]);
        }

        if err != TRANSPORTDEC_OK {
            return err;
        }
    }

    // clear data of elements not present
    for grp in 0..asi.num_groups as usize {
        if asi.drc_ui_info_present == 0 {
            asi.drc_ui_info.num_target_loudness_conditions = 0;
        }

        let mut diff = 0;
        if let Some(d) = asi.p_descriptions.as_deref_mut() {
            let description = &mut d.groups[grp];
            if description.present == 0 {
                diff |= clear_description(description);
            }
        }

        let group = &mut asi.groups[grp];
        if group.cont_present == 0 {
            if comp_assign(&mut group.content_data.content_kind, 0) {
                diff |= ASI_DIFF_CONTENT;
            }
            if comp_assign(&mut group.content_data.content_language[0], 0) {
                diff |= ASI_DIFF_CONTENT;
            }
        }
        asi.diff_flags |= diff;
    }

    for swgrp in 0..asi.num_switch_groups as usize {
        if let Some(d) = asi.p_descriptions.as_deref_mut() {
            let description = &mut d.switch_groups[swgrp];
            if description.present == 0 {
                asi.diff_flags |= clear_description(description);
            }
        }
    }

    for gp in 0..asi.num_group_presets as usize {
        let mut diff = 0;
        let preset = &mut asi.group_presets[gp];
        if preset.ext_present == 0 {
            if comp_assign(&mut preset.has_downmix_id_extension, 0) {
                diff |= ASI_DIFF_NEEDS_RESET;
            }

            for cnd in 0..preset.num_conditions as usize {
                if comp_assign(&mut preset.conditions[cnd].is_switch_group_condition, 0) {
                    diff |= ASI_DIFF_NEEDS_RESET;
                }
            }
        }

        if let Some(d) = asi.p_descriptions.as_deref_mut() {
            let description = &mut d.group_presets[gp];
            if description.present == 0 {
                diff |= clear_description(description);
            }
        }
        asi.diff_flags |= diff;
    }

    if asi.comp_pairs_present == 0 && comp_assign(&mut asi.num_composite_pairs, 0) {
        asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
    }

    if asi.scrn_size_present == 0
        && comp_assign(&mut asi.production_screen_size_data.has_non_standard_screen_size, 0)
    {
        asi.diff_flags |= ASI_DIFF_SCRNSIZE;
    }

    if asi.scrn_size_ext_present == 0 {
        if comp_assign(&mut asi.production_screen_size_data.is_centered_in_azimuth, 1) {
            asi.diff_flags |= ASI_DIFF_SCRNSIZE;
        }

        for gp in 0..asi.num_group_presets as usize {
            if comp_assign(
                &mut asi.group_presets[gp].production_screen_size.has_non_standard_screen_size,
                0,
            ) {
                asi.diff_flags |= ASI_DIFF_SCRNSIZE;
            }
        }
    }

    if asi.loud_comp_present == 0 {
        if comp_assign(&mut asi.loudness_compensation_data.group_loudness_present, 0) {
            asi.diff_flags |= ASI_DIFF_LCOMP;
        }
        if comp_assign(&mut asi.loudness_compensation_data.default_params_present, 0) {
            asi.diff_flags |= ASI_DIFF_LCOMP;
        }
        for gp in 0..asi.num_group_presets as usize {
            if comp_assign(&mut asi.loudness_compensation_data.preset_params_present[gp], 0) {
                asi.diff_flags |= ASI_DIFF_LCOMP;
            }
        }
    }
    err
}

/// `mae_AudioSceneInfo(asi, bs, numMaxElementIDs, streamIndex)`: parse MPEG-H audio scene info.
pub fn mae_audio_scene_info(
    asi: &mut AudioSceneInfo,
    bs: &mut FdkBitStream,
    num_max_element_ids: i32,
    stream_index: i32,
) -> TransportDecError {
    let mut crc_info = [FdkCrcInfo::default(); 2];
    let mut crc_reg = [0i32; 2];
    let si = stream_index as usize;

    crc_info[0].init(0x8021, 0, 16);
    crc_info[1].init(0x8021, 0, 16);

    crc_reg[0] = crc_info[0].start_reg(bs, 0);
    crc_reg[1] = crc_info[1].start_reg(bs, 0);

    let is_main_stream = bs.read_bit() as u8;
    if is_main_stream != 0 {
        asi.diff_flags = 0;
    }
    if comp_assign(&mut asi.is_main_stream[si], is_main_stream) {
        asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
    }

    if asi.is_main_stream[si] != 0 {
        let audio_scene_info_id_present = bs.read_bit();
        let id = if audio_scene_info_id_present != 0 { bs.read_bits(8) as u8 } else { 0 };
        if comp_assign(&mut asi.audio_scene_info_id, id) {
            asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
        }

        if comp_assign(&mut asi.num_groups, bs.read_bits(7) as u8) {
            asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
        }
        if asi.num_groups as usize > ASI_MAX_GROUPS {
            asi_reset(asi);
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        let err = mae_group_definition(asi, bs, num_max_element_ids);
        if err != TRANSPORTDEC_OK {
            asi_reset(asi);
            return err;
        }
        if comp_assign(&mut asi.num_switch_groups, bs.read_bits(5) as u8) {
            asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
        }
        if asi.num_switch_groups as usize > ASI_MAX_SWITCH_GROUPS {
            asi_reset(asi);
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        let err = mae_switch_group_definition(asi, bs);
        if err != TRANSPORTDEC_OK {
            asi_reset(asi);
            return err;
        }
        if comp_assign(&mut asi.num_group_presets, bs.read_bits(5) as u8) {
            asi.diff_flags |= ASI_DIFF_NEEDS_RESET;
        }
        if asi.num_group_presets as usize > ASI_MAX_GROUP_PRESETS {
            asi_reset(asi);
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        let err = mae_group_preset_definition(asi, bs);
        if err != TRANSPORTDEC_OK {
            asi_reset(asi);
            return err;
        }

        crc_info[0].end_reg(bs, crc_reg[0]);
        crc_info[1].end_reg(bs, crc_reg[1]);

        let err = mae_data(asi, bs, &mut crc_info);
        if err != TRANSPORTDEC_OK {
            asi_reset(asi);
            return err;
        }
        asi.meta_data_element_id_offset[si] = 0;
    } else if comp_assign(&mut asi.meta_data_element_id_offset[si], (bs.read_bits(7) + 1) as u8) {
        asi.diff_flags |= ASI_DIFF_AVAILABILITY;
    }

    if comp_assign(&mut asi.meta_data_element_id_max_avail[si], bs.read_bits(7) as i16) {
        asi.diff_flags |= ASI_DIFF_AVAILABILITY;
    }

    let err = mae_group_availability_check(asi);
    if err != TRANSPORTDEC_OK {
        asi_reset(asi);
        return err;
    }

    if asi.is_main_stream[si] != 0 {
        asi.crc_for_uid = crc_info[0].get_crc();
        asi.crc_for_compare = crc_info[1].get_crc();
    }

    TRANSPORTDEC_OK
}

/// `checkASI(asi, numSignalGroups, signalGroups, streamIndex)`: check that the MAE IDs of the ASI
/// groups are unique and that switch group members are always complete signal groups.
pub fn check_asi(
    asi: &AudioSceneInfo,
    num_signal_groups: i32,
    signal_groups: &[CSSignalGroup],
    stream_index: i32,
) -> TransportDecError {
    let mut mae_id2grp_idx = [INVALID_ID; 2 * 28];

    // create mapping table for MAE ID to ASI group index
    for i in 0..asi.num_groups as usize {
        for j in 0..asi.groups[i].num_members as usize {
            let mae_id = asi.groups[i].meta_data_element_id[j] as usize;

            if mae_id >= 2 * 28 {
                return TRANSPORTDEC_PARSE_ERROR;
            }

            // return error if same MAE ID occurs multiple times
            if mae_id2grp_idx[mae_id] != INVALID_ID {
                return TRANSPORTDEC_PARSE_ERROR;
            }

            mae_id2grp_idx[mae_id] = i as u8;
        }
    }

    // check if switch group members are always complete signal groups
    let mut mae_id: u8 = asi.meta_data_element_id_offset[stream_index as usize];
    for i in 0..num_signal_groups.max(0) as usize {
        if signal_groups[i].type_ == 3 {
            // HOA signal group (has single MAE ID for whole signal group)
            mae_id = mae_id.wrapping_add(1);
        } else {
            // other signal group types (have MAE IDs for individual signals)
            for j in 1..signal_groups[i].count as usize {
                let m = mae_id as usize;
                if m + j >= 2 * 28 {
                    return TRANSPORTDEC_PARSE_ERROR;
                }

                // check if all signal group members are in same ASI group
                if mae_id2grp_idx[m + j] != mae_id2grp_idx[m] {
                    // if not, check if one of them is in a switch group
                    if mae_id2grp_idx[m] != INVALID_ID
                        && asi.groups[mae_id2grp_idx[m] as usize].switch_group_id != INVALID_ID
                    {
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                    if mae_id2grp_idx[m + j] != INVALID_ID
                        && asi.groups[mae_id2grp_idx[m + j] as usize].switch_group_id != INVALID_ID
                    {
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                }
            }

            mae_id = mae_id.wrapping_add(signal_groups[i].count);
        }
    }

    TRANSPORTDEC_OK
}
