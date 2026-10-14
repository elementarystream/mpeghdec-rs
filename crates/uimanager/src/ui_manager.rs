// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libUIManager/include/uiManager.h, libUIManager/src/uiManagerInternal.h and
// libUIManager/src/uiManager.cpp.

//! MPEG-H user interactivity manager: keeps the audio scene information (ASI), the user
//! interactivity state (presets, groups, switch groups, DRC/loudness settings) and applies XML
//! action events.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_tpdec::*;

use crate::ui_persistence::UiPersistenceManager;
use crate::ui_xml::{ui_manager_parse_xml_action, ui_manager_write_xml};

// ---- uiManager.h ----

/// `UI_MANAGER_FORCE_UPDATE`: force output of XML, even if nothing changed
pub const UI_MANAGER_FORCE_UPDATE: u32 = 1;
/// `UI_MANAGER_FORCE_RESTART_XML`: force restart of partial XML output
pub const UI_MANAGER_FORCE_RESTART_XML: u32 = 4;

/// `UI_MANAGER_NO_CHANGE`: nothing changed, no XML written
pub const UI_MANAGER_NO_CHANGE: u32 = 1;
/// `UI_MANAGER_CONTINUES_XML`: partial XML output, continues output of previous call
pub const UI_MANAGER_CONTINUES_XML: u32 = 2;
/// `UI_MANAGER_INCOMPLETE_XML`: XML output is not complete, further call required
pub const UI_MANAGER_INCOMPLETE_XML: u32 = 4;
/// `UI_MANAGER_SHORT_OUTPUT`: DEPRECATED, no longer used
pub const UI_MANAGER_SHORT_OUTPUT: u32 = 8;

/// `UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS`
pub const UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS: i8 = -128;
/// `UI_MANAGER_DRC_OFF` (-1)
pub const UI_MANAGER_DRC_OFF: u32 = 0xFFFF_FFFF;
/// `UI_MANAGER_USE_DEFAULT_DRC_SELECTED` (-2)
pub const UI_MANAGER_USE_DEFAULT_DRC_SELECTED: u32 = 0xFFFF_FFFE;

/// `UI_MANAGER_ERROR`
pub type UiManagerError = i32;
pub const UI_MANAGER_OK: UiManagerError = 0;
pub const UI_MANAGER_OUT_OF_MEMORY: UiManagerError = 1;
pub const UI_MANAGER_BUFFER_TOO_SMALL: UiManagerError = 2;
pub const UI_MANAGER_INVALID_PARAM: UiManagerError = 3;
pub const UI_MANAGER_NOT_ALLOWED: UiManagerError = 4;
pub const UI_MANAGER_INVALID_STATE: UiManagerError = 5;
pub const UI_MANAGER_OK_BUT_NO_VALID_DATA: UiManagerError = 6;
pub const UI_MANAGER_NOT_AVAILABLE: UiManagerError = 7;

/// `USER_INTERACTIVITY_GROUP_STATUS`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UserInteractivityGroupStatus {
    pub group_id: u8,
    pub on_off: u8,
    pub route_to_wire: u8,
    pub route_to_wire_id: u8,
    pub change_position: u8,
    pub az_offset: u8,
    pub el_offset: u8,
    pub dist_fact: u8,
    pub change_gain: u8,
    pub gain: u8,
}

/// `USER_INTERACTIVITY_STATUS`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UserInteractivityStatus {
    pub interaction_mode: u8,
    pub num_groups: u8,
    pub group_preset_id: u8,
    pub group_data: [UserInteractivityGroupStatus; ASI_MAX_GROUPS],
}

/// `UI_DRC_LOUDNESS_STATUS`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UiDrcLoudnessStatus {
    pub target_loudness: i8,
    pub drc_selected: u32,
    pub album_mode: i8,
    pub boost: i16,
    pub compress: i16,
}

// ---- uiManagerInternal.h ----

pub const PRESET_ID_AUTO: u8 = 255;
pub const INVALID_ID: u8 = 255;
pub const INVALID_IDX: u8 = 255;
pub const NUM_PREF_LANGUAGES: usize = 10;
pub const NUM_ACCESSIBILITY_MODES: usize = 5;

pub const XML_START_INDEX: i8 = -1;
pub const XML_END_INDEX: i8 = 127;

pub const MAX_XML_TEXT_LENGTH: usize = 40;
pub const FLAG_XML_PARAM_INT: u8 = 1;
pub const FLAG_XML_PARAM_FLOAT: u8 = 2;
pub const FLAG_XML_PARAM_BOOL: u8 = 4;
pub const FLAG_XML_PARAM_TEXT: u8 = 8;

// UI commands (`UI_MANAGER_COMMAND`)
pub const UI_MANAGER_COMMAND_RESET: u8 = 0;

pub const UI_MANAGER_COMMAND_DRC_SELECTED: u8 = 10;
pub const UI_MANAGER_COMMAND_DRC_BOOST: u8 = 11;
pub const UI_MANAGER_COMMAND_DRC_COMPRESS: u8 = 12;
pub const UI_MANAGER_COMMAND_TARGET_LOUDNESS: u8 = 20;
pub const UI_MANAGER_COMMAND_ALBUM_MODE: u8 = 21;

pub const UI_MANAGER_COMMAND_PRESET_SELECTED: u8 = 30;
pub const UI_MANAGER_COMMAND_ACCESSIBILITY_PREFERENCE: u8 = 31;

pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_MUTING_CHANGED: u8 = 40;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_BALANCE_CHANGED: u8 = 41;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_AZIMUTH_CHANGED: u8 = 42;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_ELEVATION_CHANGED: u8 = 43;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_DISTANCE_CHANGED: u8 = 44;

pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_SELECTED: u8 = 60;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_MUTING_CHANGED: u8 = 61;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_BALANCE_CHANGED: u8 = 62;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_AZIMUTH_CHANGED: u8 = 63;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_ELEVATION_CHANGED: u8 = 64;
pub const UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_DISTANCE_CHANGED: u8 = 65;

pub const UI_MANAGER_COMMAND_AUDIO_LANGUAGE_SELECTED: u8 = 70;
pub const UI_MANAGER_COMMAND_INTERFACE_LANGUAGE_SELECTED: u8 = 71;

pub const UI_MANAGER_COMMAND_SET_GUID: u8 = 90;

pub const UI_MANAGER_COMMAND_PRESET_SELECTED_NO_UUID: u8 = 95;

pub const UI_MANAGER_COMMAND_FORCE_SCENESTATE_UPDATE: u8 = 100;

pub const UI_MANAGER_COMMAND_INVALID: u8 = 255;

/// `UI_STATE_GROUP`: group state
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UiStateGroup {
    pub on_off: u8,
    pub allow_on_off: u8,

    pub gain: i16,
    pub default_gain: i16,
    pub allow_gain_interactivity: u8,

    pub az_offset: u8,
    pub el_offset: u8,
    pub dist_factor: u8,
    pub default_az_offset: u8,
    pub default_el_offset: u8,
    pub default_dist_factor: u8,
    pub allow_position_interactivity: u8,

    pub is_available: u8,
}

/// `UI_STATE_SWITCH_GROUP`: switch group state
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UiStateSwitchGroup {
    pub on_off: u8,
    pub active_member_index: u8,
    pub allow_switch: u8,
    pub is_available: u8,
}

/// `UI_STATE_GROUP_PRESET`: group preset state
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UiStateGroupPreset {
    pub is_available: u8,
}

/// `UI_STATE`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UiState {
    pub uuid: [u8; 16],

    pub groups: [UiStateGroup; ASI_MAX_GROUPS],
    pub switch_groups: [UiStateSwitchGroup; ASI_MAX_SWITCH_GROUPS],
    pub group_presets: [UiStateGroupPreset; ASI_MAX_GROUP_PRESETS],
    pub active_preset_index: u8,
    pub pref_audio_languages: [[i8; 3]; NUM_PREF_LANGUAGES],
    pub accessibility_preference: u8,

    pub target_loudness: i8,
    pub drc_selected: u32,
    pub album_mode: i8,
    pub boost: i16,
    pub compress: i16,
}

/// `UI_MANAGER_XML_WRITER`: XML writer state. `pOut` and `pLastValidPos` are byte offsets into
/// the output buffer of the current `uiManagerWriteXML()` call.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UiManagerXmlWriter {
    pub p_out: usize,
    pub n_left: u32,
    pub p_last_valid_pos: Option<usize>,
    pub next_preset_idx: i8,
    pub next_group_idx: i8,
    pub next_switch_group_member_idx: i8,
}

/// `UI_MANAGER_ACTION`: UI action params
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct UiManagerAction {
    pub uuid: [u8; 16],
    pub action_type: u8,
    pub param_int: i32,
    pub param_float: i32,
    pub param_bool: u8,
    pub param_text: [i8; MAX_XML_TEXT_LENGTH + 1],
    pub present_flags: u8,
}

impl Default for UiManagerAction {
    fn default() -> Self {
        Self {
            uuid: [0; 16],
            action_type: 0,
            param_int: 0,
            param_float: 0,
            param_bool: 0,
            param_text: [0; MAX_XML_TEXT_LENGTH + 1],
            present_flags: 0,
        }
    }
}

/// `UI_MANAGER` (`HANDLE_UI_MANAGER` is `Box<UiManager>` or `&mut UiManager`)
#[derive(StateDigest, Clone, Debug)]
pub struct UiManager {
    pub asi: AudioSceneInfo,
    pub ui_state: UiState,
    pub xml_state_changed: u8,
    pub ui_state_changed: u8,
    pub drc_state_changed: u8,
    pub config_changed: u8,
    pub is_active: u8,
    pub xml_writer: UiManagerXmlWriter,
    pub h_persistence: Box<UiPersistenceManager>,
}

/// Side effects on the `UI_MANAGER` flags of the functions that may operate either on the
/// manager's own state or on a temporary state (`pUiState` argument in C).
#[derive(Default)]
pub(crate) struct Fx {
    xml: bool,
    ui: bool,
}

#[inline]
fn gid2idx(asi: &AudioSceneInfo, id: u8) -> i32 {
    asi_group_id2idx(asi, id as i32)
}

/// `FDKstrncmp(s1, s2, 3) == 0`
pub(crate) fn strncmp3_eq(s1: &[i8], s2: &[i8]) -> bool {
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

/// get first available switch group member (always operates on the manager's own state)
fn select_first_available_switch_group_member(
    asi: &AudioSceneInfo,
    st: &mut UiState,
    switch_group_idx: usize,
) -> UiManagerError {
    for i in 0..asi.switch_groups[switch_group_idx].num_members as usize {
        let grp_idx = gid2idx(asi, asi.switch_groups[switch_group_idx].member_id[i]);
        if grp_idx < 0 {
            continue;
        }

        if st.groups[grp_idx as usize].is_available != 0 {
            st.switch_groups[switch_group_idx].active_member_index = i as u8;
            return UI_MANAGER_OK;
        }
    }

    UI_MANAGER_NOT_AVAILABLE
}

/// `selectAudioLang()`: select audio language; `main` is `pUiState == &hUiManager->uiState`.
fn select_audio_lang(asi: &AudioSceneInfo, st: &mut UiState, main: bool, fx: &mut Fx) {
    for swgrp_idx in 0..asi.num_switch_groups as usize {
        let switch_group = &asi.switch_groups[swgrp_idx];
        let mut curr_prio = NUM_PREF_LANGUAGES as i32;
        let mut sel_grp_id = switch_group.default_group_id as i32;
        let mut has_lang = false;

        let grp_id =
            switch_group.member_id[st.switch_groups[swgrp_idx].active_member_index as usize];
        if gid2idx(asi, grp_id) < 0 {
            continue;
        }

        for member_idx in 0..switch_group.num_members as usize {
            let grp_id = switch_group.member_id[member_idx];
            let grp_idx = gid2idx(asi, grp_id);
            if grp_idx < 0 {
                continue;
            }
            let group = &asi.groups[grp_idx as usize];
            if group.is_available == 0 && main {
                continue;
            }

            if group.cont_present != 0 && group.content_data.content_language[0] != 0 {
                has_lang = true;

                // get language priority of member
                let mut prio = 0;
                while prio < NUM_PREF_LANGUAGES {
                    if strncmp3_eq(
                        &group.content_data.content_language,
                        &st.pref_audio_languages[prio],
                    ) {
                        break;
                    }
                    prio += 1;
                }

                // select member if it has higher priority
                if (prio as i32) < curr_prio {
                    sel_grp_id = grp_id as i32;
                    curr_prio = prio as i32;
                }
            }
        }

        if has_lang && sel_grp_id != INVALID_ID as i32 {
            select_switch_group(asi, st, main, fx, swgrp_idx, sel_grp_id as u8);

            if main {
                fx.xml = true;
            }
        }
    }
}

fn set_accessibility_index(
    asi: &AudioSceneInfo,
    ui: &UiState,
    accessibility_index: u8,
    group_preset_id: &mut u8,
) -> i32 {
    let mut found = 0;
    let mut min_id = 256;
    // map 0..4 to corresponding mae_groupPresetKind for none, visual_impaired,
    // light_hearing_impaired, heavy_hearing_impaired, visual_impaired
    const MAP: [i32; NUM_ACCESSIBILITY_MODES] = [0, 7, 5, 6, 7];

    for i in 0..asi.num_group_presets as usize {
        if asi.group_presets[i].kind as i32 == MAP[accessibility_index as usize]
            && ui.group_presets[i].is_available != 0
            && (asi.group_presets[i].group_preset_id as i32) < min_id
        {
            min_id = asi.group_presets[i].group_preset_id as i32;
            found = 1;
        }
    }

    if found != 0 {
        *group_preset_id = min_id as u8;
    }

    found
}

/// get presetId according to accessibility preference index
fn get_accessibility_preset(
    asi: &AudioSceneInfo,
    ui: &UiState,
    accessibility_index: u8,
    group_preset_id: &mut u8,
) -> i32 {
    // initialize with default preset, which is the preset with the lowest presetId
    *group_preset_id = get_min_preset_id(asi, ui, 1) as u8;

    if accessibility_index == 0 {
        return (asi_group_preset_id2idx(asi, *group_preset_id as i32) >= 0) as i32;
    }

    let mut ret_val = set_accessibility_index(asi, ui, accessibility_index, group_preset_id);

    // try again with heavy hearing impaired setting
    if ret_val == 0 && (accessibility_index == 2 || accessibility_index == 4) {
        ret_val = set_accessibility_index(asi, ui, 3, group_preset_id);
    }

    // try again with light hearing impaired setting
    if ret_val == 0 && (accessibility_index == 3 || accessibility_index == 4) {
        ret_val = set_accessibility_index(asi, ui, 2, group_preset_id);
    }

    ret_val
}

/// conditions of a group preset (replaced by the downmixIdGroupPresetExtension if present)
fn preset_conditions(preset: &AsiGroupPreset) -> &[AsiGroupPresetCondition] {
    if preset.has_downmix_id_extension != 0 {
        let n = preset.downmix_id_extension.num_conditions as usize;
        &preset.downmix_id_extension.conditions[..n]
    } else {
        &preset.conditions[..preset.num_conditions as usize]
    }
}

/// preset availability from the group and switch group availability of `st`
fn update_preset_availability(asi: &AudioSceneInfo, st: &mut UiState) {
    for p in 0..asi.num_group_presets as usize {
        st.group_presets[p].is_available = 1;

        // check conditions
        for c in preset_conditions(&asi.group_presets[p]) {
            if c.is_switch_group_condition == 0 {
                // group condition
                let grp_idx = gid2idx(asi, c.reference_id);
                if grp_idx < 0 {
                    continue;
                }

                if st.groups[grp_idx as usize].is_available == 0 && c.condition_on_off != 0 {
                    st.group_presets[p].is_available = 0;
                    break;
                }
            } else {
                // switch group condition
                let sw_grp_idx = asi_switch_group_id2idx(asi, c.reference_id as i32);
                if sw_grp_idx < 0 {
                    continue;
                }

                if st.switch_groups[sw_grp_idx as usize].is_available == 0
                    && c.condition_on_off != 0
                {
                    st.group_presets[p].is_available = 0;
                    break;
                }
            }
        }
    }
}

/// `resetAvailability()`
fn reset_availability(asi: &AudioSceneInfo, st: &mut UiState) {
    // groups
    for i in 0..asi.num_groups as usize {
        st.groups[i].is_available = asi.groups[i].is_available;
    }

    // switch groups
    for i in 0..asi.num_switch_groups as usize {
        st.switch_groups[i].is_available = 0;

        for member_idx in 0..asi.switch_groups[i].num_members as usize {
            let grp_idx = gid2idx(asi, asi.switch_groups[i].member_id[member_idx]);
            if grp_idx < 0 {
                continue;
            }

            // Check if at least one member is available.
            if asi.groups[grp_idx as usize].is_available != 0 {
                st.switch_groups[i].is_available = 1;
            }
        }
    }

    // presets
    update_preset_availability(asi, st);
}

/// `reset()`: reset state to default
fn reset(asi: &AudioSceneInfo, st: &mut UiState, main: bool, fx: &mut Fx, keep_switch_groups: i32) {
    // groups
    for i in 0..asi.num_groups as usize {
        let g = &asi.groups[i];
        let gs = &mut st.groups[i];

        // on/off
        gs.on_off = g.default_on_off;
        gs.allow_on_off = g.allow_on_off;

        // gain
        gs.gain = 0;
        gs.default_gain = 0;
        gs.allow_gain_interactivity = g.allow_gain_interactivity;

        // position
        gs.az_offset = 128;
        gs.el_offset = 32;
        gs.dist_factor = 12;
        gs.default_az_offset = 128;
        gs.default_el_offset = 32;
        gs.default_dist_factor = 12;
        gs.allow_position_interactivity = g.allow_position_interactivity;

        gs.is_available = g.is_available;
    }

    // switch groups
    for i in 0..asi.num_switch_groups as usize {
        let sg = &asi.switch_groups[i];

        st.switch_groups[i].on_off = sg.default_on_off;
        st.switch_groups[i].allow_switch = 1;

        st.switch_groups[i].is_available = 0;

        for member_idx in 0..sg.num_members as usize {
            let grp_idx = gid2idx(asi, sg.member_id[member_idx]);
            if grp_idx < 0 {
                continue;
            }
            let grp_idx = grp_idx as usize;

            st.groups[grp_idx].on_off = 0;
            st.groups[grp_idx].allow_on_off = sg.allow_on_off;

            // Check if at least one member is available.
            if asi.groups[grp_idx].is_available != 0 {
                st.switch_groups[i].is_available = 1;
            }
        }

        let err = if keep_switch_groups == 0 {
            st.switch_groups[i].active_member_index = 0;
            select_switch_group(asi, st, main, fx, i, sg.default_group_id)
        } else {
            let id = sg.member_id[st.switch_groups[i].active_member_index as usize];
            select_switch_group(asi, st, main, fx, i, id)
        };

        if err != UI_MANAGER_OK {
            st.switch_groups[i].on_off = 0;
        }
    }

    // presets
    update_preset_availability(asi, st);

    st.active_preset_index = INVALID_IDX;

    if keep_switch_groups == 0 {
        // select audio language
        select_audio_lang(asi, st, main, fx);
    }

    if main {
        fx.xml = true;
        fx.ui = true;
    }
}

/// `cmpGroupProperties()`: compare asi group properties: 0 = no diff, 1 = different
fn cmp_group_properties(asi: &AudioSceneInfo, group_index1: i32, group_index2: i32) -> i32 {
    if group_index1 == group_index2
        || group_index1 >= asi.num_groups as i32
        || group_index2 >= asi.num_groups as i32
    {
        return 0;
    }

    let grp1 = &asi.groups[group_index1 as usize];
    let grp2 = &asi.groups[group_index2 as usize];

    if grp1.allow_position_interactivity != grp2.allow_position_interactivity {
        return 1;
    }
    if grp1.allow_gain_interactivity != grp2.allow_gain_interactivity {
        return 1;
    }

    // ignore allowOnOff and defaultOnOff because this is not relevant for switchgroup members

    if grp1.interactivity_min_az_offset != grp2.interactivity_min_az_offset
        || grp1.interactivity_max_az_offset != grp2.interactivity_max_az_offset
        || grp1.interactivity_min_el_offset != grp2.interactivity_min_el_offset
        || grp1.interactivity_max_el_offset != grp2.interactivity_max_el_offset
        || grp1.interactivity_min_dist_factor != grp2.interactivity_min_dist_factor
        || grp1.interactivity_max_dist_factor != grp2.interactivity_max_dist_factor
        || grp1.interactivity_min_gain != grp2.interactivity_min_gain
        || grp1.interactivity_max_gain != grp2.interactivity_max_gain
    {
        return 1;
    }

    0
}

/// `selectSwitchGroup()`: select active switch group member
fn select_switch_group(
    asi: &AudioSceneInfo,
    st: &mut UiState,
    main: bool,
    fx: &mut Fx,
    switch_grp_idx: usize,
    group_id: u8,
) -> UiManagerError {
    let switch_group = &asi.switch_groups[switch_grp_idx];
    let mut found = false;

    // check if allowed
    if st.switch_groups[switch_grp_idx].allow_switch == 0 {
        return UI_MANAGER_NOT_ALLOWED;
    }

    // check if available
    if main {
        if st.switch_groups[switch_grp_idx].is_available == 0 {
            return UI_MANAGER_NOT_AVAILABLE;
        }

        let grp_idx = gid2idx(asi, group_id);
        if grp_idx < 0 {
            return UI_MANAGER_INVALID_PARAM;
        }
        if st.groups[grp_idx as usize].is_available == 0 {
            return UI_MANAGER_NOT_AVAILABLE;
        }
    }

    // get currently active group id
    let prev_grp_idx = gid2idx(
        asi,
        switch_group.member_id[st.switch_groups[switch_grp_idx].active_member_index as usize],
    );
    if prev_grp_idx < 0 {
        return UI_MANAGER_INVALID_STATE;
    }

    // find member to be activated
    for i in 0..switch_group.num_members as usize {
        // get group index
        let grp_idx = gid2idx(asi, switch_group.member_id[i]);
        if grp_idx < 0 {
            continue;
        }

        // turn selected member on, all others off
        if switch_group.member_id[i] == group_id {
            st.switch_groups[switch_grp_idx].active_member_index = i as u8;
            st.groups[grp_idx as usize].on_off = st.switch_groups[switch_grp_idx].on_off;
            found = true;
        } else {
            st.groups[grp_idx as usize].on_off = 0;
        }
    }

    // if ID not found
    if !found {
        if switch_group.allow_on_off != 0 {
            // switch group turned off
            st.switch_groups[switch_grp_idx].on_off = 0;
        } else {
            // turn previous active member back on (the active member index is unchanged, so
            // this is prevGrpIdx)
            let grp_id = switch_group.member_id
                [st.switch_groups[switch_grp_idx].active_member_index as usize];
            let grp_idx = gid2idx(asi, grp_id);
            st.groups[grp_idx as usize].on_off = 1;
        }
    }

    // get active group id
    let grp_idx = gid2idx(
        asi,
        switch_group.member_id[st.switch_groups[switch_grp_idx].active_member_index as usize],
    );
    if grp_idx < 0 {
        return UI_MANAGER_INVALID_STATE;
    }

    if main {
        if cmp_group_properties(asi, prev_grp_idx, grp_idx) != 0 {
            fx.xml = true;
        }

        fx.ui = true;
    }

    if !found {
        return UI_MANAGER_INVALID_PARAM;
    }

    UI_MANAGER_OK
}

/// `applyConditionToGroup()`: apply preset condition to group
fn apply_condition_to_group(
    asi: &AudioSceneInfo,
    st: &mut UiState,
    condition: &AsiGroupPresetCondition,
    grp_idx: usize,
) {
    let gs = &mut st.groups[grp_idx];

    // on/off (do not set for switch group members)
    if asi.groups[grp_idx].switch_group_id == INVALID_ID {
        gs.on_off = condition.condition_on_off;
    }

    // disable on/off
    gs.allow_on_off = 0;

    if condition.condition_on_off != 0 {
        // gain interactivity
        if condition.disable_gain_interactivity != 0 {
            gs.allow_gain_interactivity = 0;
        }

        // default gain
        if condition.gain_flag != 0 {
            gs.default_gain = (condition.gain as i16 - 255) + 64;
            gs.gain = gs.default_gain;
        }

        // position interactivity
        if condition.disable_position_interactivity != 0 {
            gs.allow_position_interactivity = 0;
        }

        // default position
        if condition.position_flag != 0 {
            gs.default_az_offset = condition.az_offset;
            gs.az_offset = condition.az_offset;
            gs.default_el_offset = condition.el_offset;
            gs.el_offset = condition.el_offset;
            gs.default_dist_factor = condition.dist_factor;
            gs.dist_factor = condition.dist_factor;
        }
    }
}

/// `applyPreset()`: apply preset. `main_ui` is `None` if `st` is the manager's own state,
/// otherwise the manager's own state (read by the automatic preset selection).
fn apply_preset(
    asi: &AudioSceneInfo,
    st: &mut UiState,
    main_ui: Option<&UiState>,
    fx: &mut Fx,
    mut preset_id: u8,
) -> UiManagerError {
    let main = main_ui.is_none();

    if preset_id == PRESET_ID_AUTO {
        // find appropriate preset (default preset is returned for accessibility preference 0)
        let ui = main_ui.unwrap_or(&*st);
        get_accessibility_preset(asi, ui, ui.accessibility_preference, &mut preset_id);
        if asi_group_preset_id2idx(asi, preset_id as i32) < 0 {
            return UI_MANAGER_NOT_AVAILABLE;
        }
    }

    // get preset index and pointer
    let preset_idx = asi_group_preset_id2idx(asi, preset_id as i32);
    if preset_idx < 0 {
        return UI_MANAGER_INVALID_PARAM;
    }
    let preset_idx = preset_idx as usize;

    // check if available
    if main && st.group_presets[preset_idx].is_available == 0 {
        return UI_MANAGER_NOT_AVAILABLE;
    }

    let preset = &asi.group_presets[preset_idx];

    // set active preset index
    st.active_preset_index = preset_idx as u8;

    // replace conditions if downmixIdGroupPresetExtension applies
    let conditions = preset_conditions(preset);

    // apply preset conditions
    for cond in conditions {
        if cond.is_switch_group_condition == 0 {
            // apply group condition
            // get group index
            let grp_idx = gid2idx(asi, cond.reference_id);
            if grp_idx < 0 {
                continue;
            }
            let grp_idx = grp_idx as usize;

            // check if group is member of a switch group
            if asi.groups[grp_idx].switch_group_id == INVALID_ID {
                apply_condition_to_group(asi, st, cond, grp_idx);
            } else {
                // get switch group index and pointer
                let swgrp_idx =
                    asi_switch_group_id2idx(asi, asi.groups[grp_idx].switch_group_id as i32);
                if swgrp_idx < 0 {
                    continue;
                }
                let swgrp_idx = swgrp_idx as usize;
                let switch_group = &asi.switch_groups[swgrp_idx];

                // apply condition to switch group
                if cond.condition_on_off != 0 {
                    // on/off
                    st.switch_groups[swgrp_idx].on_off = 1;

                    // disable switch
                    st.switch_groups[swgrp_idx].allow_switch = 0;

                    // apply condition to all switch group members
                    for member_idx in 0..switch_group.num_members as usize {
                        let grp_id = switch_group.member_id[member_idx];

                        // get group index
                        let grp_idx = gid2idx(asi, grp_id);
                        if grp_idx < 0 {
                            continue;
                        }
                        let grp_idx = grp_idx as usize;

                        // on/off
                        if grp_id == cond.reference_id {
                            st.groups[grp_idx].on_off = 1;
                            st.switch_groups[swgrp_idx].active_member_index = member_idx as u8;
                        } else {
                            st.groups[grp_idx].on_off = 0;
                        }

                        apply_condition_to_group(asi, st, cond, grp_idx);
                    }
                }
            }
        } else {
            // switch group condition
            // get switch group index and pointer
            let swgrp_idx = asi_switch_group_id2idx(asi, cond.reference_id as i32);
            if swgrp_idx < 0 {
                continue;
            }
            let swgrp_idx = swgrp_idx as usize;
            let switch_group = &asi.switch_groups[swgrp_idx];

            // on/off
            st.switch_groups[swgrp_idx].on_off = cond.condition_on_off;

            // set active member on/off
            let grp_id =
                switch_group.member_id[st.switch_groups[swgrp_idx].active_member_index as usize];
            let mut grp_idx = gid2idx(asi, grp_id);
            if grp_idx < 0 {
                continue;
            }

            // if active member not available, select first available member
            if main && cond.condition_on_off != 0 && st.groups[grp_idx as usize].is_available == 0 {
                st.groups[grp_idx as usize].on_off = 0;

                select_first_available_switch_group_member(asi, st, swgrp_idx);

                let grp_id = switch_group.member_id
                    [st.switch_groups[swgrp_idx].active_member_index as usize];
                grp_idx = gid2idx(asi, grp_id);
            }

            st.groups[grp_idx as usize].on_off = cond.condition_on_off;

            // apply condition to members
            for member_idx in 0..switch_group.num_members as usize {
                // get group index
                let grp_idx = gid2idx(asi, switch_group.member_id[member_idx]);
                if grp_idx < 0 {
                    continue;
                }

                apply_condition_to_group(asi, st, cond, grp_idx as usize);
            }
        }
    }

    if main {
        fx.xml = true;
        fx.ui = true;
    }

    UI_MANAGER_OK
}

/// `simulatePreset(hUiManager, presetID, pUiState)`: `ui` is the manager's own state, `st` a
/// temporary state.
pub fn simulate_preset(asi: &AudioSceneInfo, ui: &UiState, preset_id: u8, st: &mut UiState) {
    let mut fx = Fx::default();
    reset(asi, st, false, &mut fx, 0);
    apply_preset(asi, st, Some(ui), &mut fx, preset_id);
}

/// `getMinPresetID(hUiManager, checkAvailability)`: get lowest groupPresetID
pub fn get_min_preset_id(asi: &AudioSceneInfo, ui: &UiState, check_availability: i32) -> i32 {
    let mut min_id = 255;

    if asi.num_group_presets == 0 {
        return 0;
    }

    for i in 0..asi.num_group_presets as usize {
        if check_availability != 0 && ui.group_presets[i].is_available == 0 {
            continue;
        }

        if (asi.group_presets[i].group_preset_id as i32) < min_id {
            min_id = asi.group_presets[i].group_preset_id as i32;
        }
    }
    if min_id == 255 {
        min_id = 0;
    }

    min_id
}

/// compare UUID
fn comp_uuid(uuid1: &[u8; 16], uuid2: &[u8; 16]) -> bool {
    uuid1 == uuid2
}

/// apply default fallbacks for requested DRC effect types
fn get_drc_effect_type_fallback(drc_effect_type_requested: i32) -> u32 {
    let mut ret_val = drc_effect_type_requested as u32;
    if ret_val <= 0xF {
        ret_val = match drc_effect_type_requested {
            0x0000_0000 => 0x0000_0000, // NONE
            0x0000_0001 => 0x0054_3261, // NIGHT
            0x0000_0002 => 0x0054_3162, // NOISY
            0x0000_0003 => 0x0054_2163, // LIMITED
            0x0000_0004 => 0x0053_1264, // LOWLEVEL
            0x0000_0005 => 0x0043_2165, // DIALOG
            0x0000_0006 => 0x0054_3216, // GENERAL
            _ => 0x0000_0000,
        };
    }

    ret_val
}

/// check validity of DRC effect type fallback code
fn check_drc_effect_type_fallback(drc_effect_type_requested: i32) -> i32 {
    let ul = drc_effect_type_requested as u32;
    if ul == UI_MANAGER_DRC_OFF {
        return 0;
    }
    if ul == UI_MANAGER_USE_DEFAULT_DRC_SELECTED {
        return 0;
    }
    for i in 0..8 {
        // Each byte, representing a fallback DRC effect type, may not be >=
        // EFFECT_TYPE_REQUESTED_COUNT (9)
        if ((ul >> (4 * i)) & 0xF) >= 9 {
            return -1;
        }
    }
    0
}

/// `(SHORT)(((LONG)paramFloat + 0x8000) >> 15) & ~1`: Q16 gain in dB to 0.5 dB steps
#[inline]
fn gain_from_float(param_float: i32) -> i16 {
    ((((param_float.wrapping_add(0x8000)) >> 15) as i16) as i32 & !1) as i16
}

/// `(UCHAR)((((paramFloat * 2 / 3) + 0x8000) >> 16) + 128)`: Q16 degrees to azimuth offset
#[inline]
fn az_from_float(param_float: i32) -> u8 {
    (((param_float.wrapping_mul(2) / 3).wrapping_add(0x8000) >> 16) + 128) as u8
}

/// `(UCHAR)(((((LONG)paramFloat / 3) + 0x8000) >> 16) + 32)`: Q16 degrees to elevation offset
#[inline]
fn el_from_float(param_float: i32) -> u8 {
    (((param_float / 3 + 0x8000) >> 16) + 32) as u8
}

impl UiManager {
    /// `UI_Manager_Create(phUiManager, storeAsiDescriptions)`: create instance
    pub fn create(store_asi_descriptions: i32) -> Box<UiManager> {
        let mut asi = AudioSceneInfo::default();
        if store_asi_descriptions != 0 {
            asi.p_descriptions = Some(Box::default());
        }
        asi.active_dmx_id = INVALID_ID;
        let ui_state = UiState {
            target_loudness: UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS,
            drc_selected: UI_MANAGER_USE_DEFAULT_DRC_SELECTED,
            album_mode: -1,
            boost: -1,
            compress: -1,
            ..Default::default()
        };

        Box::new(UiManager {
            asi,
            ui_state,
            xml_state_changed: 1,
            ui_state_changed: 1,
            drc_state_changed: 1,
            config_changed: 1,
            is_active: 1,
            xml_writer: UiManagerXmlWriter::default(),
            h_persistence: UiPersistenceManager::create(),
        })
    }

    fn apply_fx(&mut self, fx: Fx) {
        if fx.xml {
            self.xml_state_changed = 1;
        }
        if fx.ui {
            self.ui_state_changed = 1;
        }
    }

    fn reset_main(&mut self, keep_switch_groups: i32) {
        let mut fx = Fx::default();
        reset(&self.asi, &mut self.ui_state, true, &mut fx, keep_switch_groups);
        self.apply_fx(fx);
    }

    fn apply_preset_main(&mut self, preset_id: u8) -> UiManagerError {
        let mut fx = Fx::default();
        let err = apply_preset(&self.asi, &mut self.ui_state, None, &mut fx, preset_id);
        self.apply_fx(fx);
        err
    }

    fn select_switch_group_main(&mut self, switch_grp_idx: usize, group_id: u8) -> UiManagerError {
        let mut fx = Fx::default();
        let err = select_switch_group(
            &self.asi,
            &mut self.ui_state,
            true,
            &mut fx,
            switch_grp_idx,
            group_id,
        );
        self.apply_fx(fx);
        err
    }

    fn select_audio_lang_main(&mut self) {
        let mut fx = Fx::default();
        select_audio_lang(&self.asi, &mut self.ui_state, true, &mut fx);
        self.apply_fx(fx);
    }

    /// set group on/off
    fn set_group_on_off(&mut self, group_id: u8, on_off: u8) -> UiManagerError {
        // get group index
        let grp_idx = gid2idx(&self.asi, group_id);
        if grp_idx < 0 {
            return UI_MANAGER_INVALID_PARAM;
        }
        let grp_idx = grp_idx as usize;

        // check if allowed
        if self.ui_state.groups[grp_idx].allow_on_off == 0 {
            return UI_MANAGER_NOT_ALLOWED;
        }
        if self.asi.groups[grp_idx].switch_group_id != INVALID_ID {
            return UI_MANAGER_NOT_ALLOWED;
        }

        // check if available
        if on_off != 0 && self.ui_state.groups[grp_idx].is_available == 0 {
            return UI_MANAGER_NOT_AVAILABLE;
        }

        // set on/off
        self.ui_state.groups[grp_idx].on_off = on_off;

        self.ui_state_changed = 1;

        UI_MANAGER_OK
    }

    /// Index of the active member group of a switch group (`UI_MANAGER_INVALID_PARAM` /
    /// `UI_MANAGER_INVALID_STATE` on errors).
    fn switch_group_active(&self, switch_group_id: u8) -> Result<(usize, usize), UiManagerError> {
        // get switch group index
        let swgrp_idx = asi_switch_group_id2idx(&self.asi, switch_group_id as i32);
        if swgrp_idx < 0 {
            return Err(UI_MANAGER_INVALID_PARAM);
        }
        let swgrp_idx = swgrp_idx as usize;

        // get active group index
        let member_idx = self.ui_state.switch_groups[swgrp_idx].active_member_index as usize;
        let grp_idx = gid2idx(&self.asi, self.asi.switch_groups[swgrp_idx].member_id[member_idx]);
        if grp_idx < 0 {
            return Err(UI_MANAGER_INVALID_STATE);
        }
        Ok((swgrp_idx, grp_idx as usize))
    }

    /// set switch group on/off
    fn set_switch_group_on_off(&mut self, switch_group_id: u8, on_off: u8) -> UiManagerError {
        let (swgrp_idx, mut grp_idx) = match self.switch_group_active(switch_group_id) {
            Ok(v) => v,
            Err(e) => return e,
        };

        // check if allowed
        if self.ui_state.groups[grp_idx].allow_on_off == 0 {
            return UI_MANAGER_NOT_ALLOWED;
        }

        // check if available
        if on_off != 0 && self.ui_state.switch_groups[swgrp_idx].is_available == 0 {
            return UI_MANAGER_NOT_AVAILABLE;
        }

        // if active member not available, select first available member
        if on_off != 0 && self.ui_state.groups[grp_idx].is_available == 0 {
            self.ui_state.groups[grp_idx].on_off = 0;

            select_first_available_switch_group_member(&self.asi, &mut self.ui_state, swgrp_idx);

            let member_idx = self.ui_state.switch_groups[swgrp_idx].active_member_index as usize;
            grp_idx = gid2idx(&self.asi, self.asi.switch_groups[swgrp_idx].member_id[member_idx])
                as usize;
        }

        // set on/off
        self.ui_state.switch_groups[swgrp_idx].on_off = on_off;
        self.ui_state.groups[grp_idx].on_off = on_off;

        self.ui_state_changed = 1;

        UI_MANAGER_OK
    }

    /// limit a gain to the allowed range of a group
    fn limit_gain(&self, grp_idx: usize, mut gain: i16) -> i16 {
        let g = &self.asi.groups[grp_idx];
        if g.interactivity_min_gain != 0 {
            gain = gain.max(((g.interactivity_min_gain as i16 - 63) << 1) as i16);
        }
        gain.min(((g.interactivity_max_gain as i32) << 1) as i16)
    }

    /// set group gain
    fn set_group_gain(&mut self, group_id: u8, gain: i16) -> UiManagerError {
        // get group index
        let grp_idx = gid2idx(&self.asi, group_id);
        if grp_idx < 0 {
            return UI_MANAGER_INVALID_PARAM;
        }
        let grp_idx = grp_idx as usize;

        // check if allowed
        if self.ui_state.groups[grp_idx].allow_gain_interactivity == 0 {
            return UI_MANAGER_NOT_ALLOWED;
        }
        if self.asi.groups[grp_idx].switch_group_id != INVALID_ID {
            return UI_MANAGER_NOT_ALLOWED;
        }

        // limit to allowed range, set gain
        self.ui_state.groups[grp_idx].gain = self.limit_gain(grp_idx, gain);

        self.ui_state_changed = 1;

        UI_MANAGER_OK
    }

    /// set switch group gain
    fn set_switch_group_gain(&mut self, switch_group_id: u8, gain: i16) -> UiManagerError {
        let (swgrp_idx, grp_idx) = match self.switch_group_active(switch_group_id) {
            Ok(v) => v,
            Err(e) => return e,
        };

        // check if allowed
        if self.ui_state.groups[grp_idx].allow_gain_interactivity == 0 {
            return UI_MANAGER_NOT_ALLOWED;
        }

        // set gain for all members
        for member_idx in 0..self.asi.switch_groups[swgrp_idx].num_members as usize {
            let grp_idx =
                gid2idx(&self.asi, self.asi.switch_groups[swgrp_idx].member_id[member_idx]);
            if grp_idx < 0 {
                continue;
            }
            let grp_idx = grp_idx as usize;

            // check if allowed
            if self.asi.groups[grp_idx].allow_gain_interactivity == 0 {
                continue;
            }

            // limit to allowed range
            self.ui_state.groups[grp_idx].gain = self.limit_gain(grp_idx, gain);
        }

        self.ui_state_changed = 1;

        UI_MANAGER_OK
    }

    /// limit an azimuth offset to the allowed range of a group
    fn limit_az(&self, grp_idx: usize, az_offset: u8) -> u8 {
        let g = &self.asi.groups[grp_idx];
        az_offset
            .max(128u8.wrapping_sub(g.interactivity_min_az_offset))
            .min(128u8.wrapping_add(g.interactivity_max_az_offset))
    }

    /// limit an elevation offset to the allowed range of a group
    fn limit_el(&self, grp_idx: usize, el_offset: u8) -> u8 {
        let g = &self.asi.groups[grp_idx];
        el_offset
            .max(32u8.wrapping_sub(g.interactivity_min_el_offset))
            .min(32u8.wrapping_add(g.interactivity_max_el_offset))
    }

    /// set group azimuth (`el == false`) or elevation (`el == true`) offset
    fn set_group_pos_offset(&mut self, group_id: u8, offset: u8, el: bool) -> UiManagerError {
        // get group index
        let grp_idx = gid2idx(&self.asi, group_id);
        if grp_idx < 0 {
            return UI_MANAGER_INVALID_PARAM;
        }
        let grp_idx = grp_idx as usize;

        // check if allowed
        if self.ui_state.groups[grp_idx].allow_position_interactivity == 0 {
            return UI_MANAGER_NOT_ALLOWED;
        }
        if self.asi.groups[grp_idx].switch_group_id != INVALID_ID {
            return UI_MANAGER_NOT_ALLOWED;
        }

        // limit to allowed range, set offset
        if el {
            self.ui_state.groups[grp_idx].el_offset = self.limit_el(grp_idx, offset);
        } else {
            self.ui_state.groups[grp_idx].az_offset = self.limit_az(grp_idx, offset);
        }

        self.ui_state_changed = 1;

        UI_MANAGER_OK
    }

    /// set switch group azimuth (`el == false`) or elevation (`el == true`) offset
    fn set_switch_group_pos_offset(
        &mut self,
        switch_group_id: u8,
        offset: u8,
        el: bool,
    ) -> UiManagerError {
        let (swgrp_idx, grp_idx) = match self.switch_group_active(switch_group_id) {
            Ok(v) => v,
            Err(e) => return e,
        };

        // check if allowed
        if self.ui_state.groups[grp_idx].allow_position_interactivity == 0 {
            return UI_MANAGER_NOT_ALLOWED;
        }

        // set offset for all members
        for member_idx in 0..self.asi.switch_groups[swgrp_idx].num_members as usize {
            let grp_idx =
                gid2idx(&self.asi, self.asi.switch_groups[swgrp_idx].member_id[member_idx]);
            if grp_idx < 0 {
                continue;
            }
            let grp_idx = grp_idx as usize;

            // check if allowed
            if self.asi.groups[grp_idx].allow_position_interactivity == 0 {
                continue;
            }

            // limit to allowed range
            if el {
                self.ui_state.groups[grp_idx].el_offset = self.limit_el(grp_idx, offset);
            } else {
                self.ui_state.groups[grp_idx].az_offset = self.limit_az(grp_idx, offset);
            }
        }

        self.ui_state_changed = 1;

        UI_MANAGER_OK
    }

    /// `update()`: check/update state
    fn update(&mut self) {
        if self.asi.diff_flags & ASI_DIFF_AVAILABILITY != 0 {
            reset_availability(&self.asi, &mut self.ui_state);
            self.xml_state_changed = 1;
        }

        if self.asi.diff_flags & !(ASI_DIFF_AVAILABILITY | ASI_DIFF_DESCRIPTION) != 0 {
            for i in 0..15 {
                self.ui_state.uuid[i] = 0;
            }

            // set last byte of UUID to ASI ID (0..255)
            self.ui_state.uuid[15] = self.asi.audio_scene_info_id;

            // xor upper 16 bits of UUID with partial ASI CRC
            self.ui_state.uuid[0] ^= (self.asi.crc_for_uid >> 8) as u8;
            self.ui_state.uuid[1] ^= (self.asi.crc_for_uid & 0xFF) as u8;

            // reset
            self.reset_main(0);
            self.apply_preset_main(PRESET_ID_AUTO);

            // restore state
            self.persistence_restore();

            self.config_changed = 1;
        }

        if self.asi.diff_flags & ASI_DIFF_DESCRIPTION != 0 {
            self.xml_state_changed = 1;
        }

        self.asi.diff_flags = 0;
    }

    /// Common check of the interactive commands: UUID matches, required parameters present.
    fn check_action(&self, action: &UiManagerAction, flags: u8) -> bool {
        comp_uuid(&self.ui_state.uuid, &action.uuid) && (action.present_flags & flags) == flags
    }

    /// `performAction()`: perform UI action
    fn perform_action(&mut self, action: &UiManagerAction) -> UiManagerError {
        let mut err = UI_MANAGER_OK;
        const INT_BOOL: u8 = FLAG_XML_PARAM_INT | FLAG_XML_PARAM_BOOL;
        const INT_FLOAT: u8 = FLAG_XML_PARAM_INT | FLAG_XML_PARAM_FLOAT;

        match action.action_type {
            UI_MANAGER_COMMAND_RESET => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                self.reset_main(0);
                self.apply_preset_main(PRESET_ID_AUTO);
            }

            UI_MANAGER_COMMAND_DRC_SELECTED => {
                if action.present_flags & FLAG_XML_PARAM_INT == 0
                    || check_drc_effect_type_fallback(action.param_int) != 0
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    self.ui_state.drc_selected = get_drc_effect_type_fallback(action.param_int);
                    self.drc_state_changed = 1;
                }
            }

            UI_MANAGER_COMMAND_DRC_BOOST | UI_MANAGER_COMMAND_DRC_COMPRESS => {
                if action.present_flags & FLAG_XML_PARAM_FLOAT == 0
                    || action.param_float < 0
                    || action.param_float > (1 << 16)
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    // Q16 to Q14
                    let v = (action.param_float >> 2) as i16;
                    if action.action_type == UI_MANAGER_COMMAND_DRC_BOOST {
                        self.ui_state.boost = v;
                    } else {
                        self.ui_state.compress = v;
                    }
                    self.drc_state_changed = 1;
                }
            }

            UI_MANAGER_COMMAND_TARGET_LOUDNESS => {
                if action.present_flags & FLAG_XML_PARAM_FLOAT == 0
                    || ((action.param_float < -63 * (1 << 16)
                        || action.param_float > -10 * (1 << 16))
                        && action.param_float
                            != UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS as i32 * (1 << 16))
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    self.ui_state.target_loudness = (action.param_float >> 16) as i8;
                    self.drc_state_changed = 1;
                }
            }

            UI_MANAGER_COMMAND_ALBUM_MODE => {
                if action.present_flags & FLAG_XML_PARAM_BOOL == 0 {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    self.ui_state.album_mode = action.param_bool as i8;
                    self.drc_state_changed = 1;
                }
            }

            UI_MANAGER_COMMAND_PRESET_SELECTED | UI_MANAGER_COMMAND_PRESET_SELECTED_NO_UUID => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                let uuid_ok = action.action_type == UI_MANAGER_COMMAND_PRESET_SELECTED_NO_UUID
                    || comp_uuid(&self.ui_state.uuid, &action.uuid);
                if !uuid_ok
                    || action.present_flags & FLAG_XML_PARAM_INT == 0
                    || asi_group_preset_id2idx(&self.asi, action.param_int) < 0
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    self.reset_main(0);
                    err = self.apply_preset_main(action.param_int as u8);
                }
            }

            UI_MANAGER_COMMAND_ACCESSIBILITY_PREFERENCE => {
                if action.present_flags & FLAG_XML_PARAM_INT == 0
                    || action.param_int < 0
                    || action.param_int >= NUM_ACCESSIBILITY_MODES as i32
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    let mut group_preset_id = 0u8;
                    self.ui_state.accessibility_preference = action.param_int as u8;
                    if self.is_active != 0 {
                        get_accessibility_preset(
                            &self.asi,
                            &self.ui_state,
                            self.ui_state.accessibility_preference,
                            &mut group_preset_id,
                        );
                        self.reset_main(0);
                        self.apply_preset_main(group_preset_id);
                    }
                    self.drc_state_changed = 1;
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_MUTING_CHANGED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_BOOL) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    err = self
                        .set_group_on_off(action.param_int as u8, (action.param_bool == 0) as u8);
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_BALANCE_CHANGED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_FLOAT) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    err = self.set_group_gain(
                        action.param_int as u8,
                        gain_from_float(action.param_float),
                    );
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_AZIMUTH_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_ELEVATION_CHANGED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_FLOAT) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else if action.action_type == UI_MANAGER_COMMAND_AUDIO_ELEMENT_AZIMUTH_CHANGED {
                    err = self.set_group_pos_offset(
                        action.param_int as u8,
                        az_from_float(action.param_float),
                        false,
                    );
                } else {
                    err = self.set_group_pos_offset(
                        action.param_int as u8,
                        el_from_float(action.param_float),
                        true,
                    );
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_DISTANCE_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_DISTANCE_CHANGED => {
                // currently not supported
                return UI_MANAGER_INVALID_PARAM;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_SELECTED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_FLOAT) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    let swgrp_idx = asi_switch_group_id2idx(&self.asi, action.param_int);
                    if swgrp_idx < 0 {
                        return UI_MANAGER_INVALID_PARAM;
                    }

                    err = self.select_switch_group_main(
                        swgrp_idx as usize,
                        (action.param_float >> 16) as u8,
                    );
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_MUTING_CHANGED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_BOOL) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    err = self.set_switch_group_on_off(
                        action.param_int as u8,
                        (action.param_bool == 0) as u8,
                    );
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_BALANCE_CHANGED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_FLOAT) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    err = self.set_switch_group_gain(
                        action.param_int as u8,
                        gain_from_float(action.param_float),
                    );
                }
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_AZIMUTH_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_ELEVATION_CHANGED => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }
                if !self.check_action(action, INT_FLOAT) {
                    err = UI_MANAGER_INVALID_PARAM;
                } else if action.action_type
                    == UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_AZIMUTH_CHANGED
                {
                    err = self.set_switch_group_pos_offset(
                        action.param_int as u8,
                        az_from_float(action.param_float),
                        false,
                    );
                } else {
                    err = self.set_switch_group_pos_offset(
                        action.param_int as u8,
                        el_from_float(action.param_float),
                        true,
                    );
                }
            }

            UI_MANAGER_COMMAND_AUDIO_LANGUAGE_SELECTED => {
                if action.param_int < 0
                    || action.param_int >= NUM_PREF_LANGUAGES as i32
                    || action.present_flags & FLAG_XML_PARAM_TEXT == 0
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    let mut language =
                        [action.param_text[0], action.param_text[1], action.param_text[2]];
                    if asi_check_iso639_2(&mut language) != 0 {
                        err = UI_MANAGER_INVALID_PARAM;
                    } else {
                        asi_map_iso639_2_t2b_and_tolower(&mut language);
                        self.ui_state.pref_audio_languages[action.param_int as usize] = language;

                        if self.is_active != 0 {
                            self.select_audio_lang_main();
                        }
                    }
                }
            }

            UI_MANAGER_COMMAND_INTERFACE_LANGUAGE_SELECTED => {
                if action.param_int < 0
                    || action.param_int >= ASI_MAX_PREF_DESCR_LANGUAGES as i32
                    || action.present_flags & FLAG_XML_PARAM_TEXT == 0
                {
                    err = UI_MANAGER_INVALID_PARAM;
                } else {
                    let mut language =
                        [action.param_text[0], action.param_text[1], action.param_text[2]];
                    if asi_check_iso639_2(&mut language) != 0 {
                        err = UI_MANAGER_INVALID_PARAM;
                    } else {
                        asi_map_iso639_2_t2b_and_tolower(&mut language);
                        let pi = action.param_int as usize;

                        if self.asi.pref_descr_languages[pi] != language {
                            self.xml_state_changed = 1;
                        }

                        self.asi.pref_descr_languages[pi] = language;

                        if self.is_active != 0 {
                            self.update_pref_description_languages();
                        }
                    }
                }
            }

            UI_MANAGER_COMMAND_SET_GUID => {
                if self.is_active == 0 {
                    return UI_MANAGER_NOT_ALLOWED;
                }

                let chg = self.ui_state.uuid != action.uuid;
                self.ui_state.uuid = action.uuid;

                // restore state after change
                if chg {
                    self.persistence_restore();
                }
            }

            UI_MANAGER_COMMAND_FORCE_SCENESTATE_UPDATE => {
                self.xml_state_changed = 1;
            }

            _ => return UI_MANAGER_INVALID_PARAM,
        }

        err
    }

    /// Selects the preferred description language of all descriptions (part of the
    /// `UI_MANAGER_COMMAND_INTERFACE_LANGUAGE_SELECTED` handling).
    fn update_pref_description_languages(&mut self) {
        let pref = self.asi.pref_descr_languages;
        let (ng, nsg, ngp) = (
            self.asi.num_groups as usize,
            self.asi.num_switch_groups as usize,
            self.asi.num_group_presets as usize,
        );
        let Some(d) = self.asi.p_descriptions.as_deref_mut() else { return };

        // loop over groups, switch groups, presets
        for type_ in 0..3 {
            let descriptions: &mut [AsiDescription] = match type_ {
                0 => &mut d.groups[..ng],
                1 => &mut d.switch_groups[..nsg],
                _ => &mut d.group_presets[..ngp],
            };

            // loop over descriptions
            for descr in descriptions.iter_mut() {
                if descr.present == 0 {
                    continue;
                }
                let mut curr_prio = 1000;

                // loop over languages
                for lng in
                    0..(descr.num_languages.max(0) as usize).min(ASI_MAX_DESCRIPTION_LANGUAGES)
                {
                    let mut prio = 0;
                    while prio < ASI_MAX_PREF_DESCR_LANGUAGES {
                        if descr.language[lng][0] != 0
                            && strncmp3_eq(&descr.language[lng], &pref[prio])
                        {
                            break;
                        }
                        prio += 1;
                    }
                    if prio == ASI_MAX_PREF_DESCR_LANGUAGES {
                        prio += lng; // if not in preferred list select first transmitted
                    }

                    if (prio as i32) < curr_prio {
                        descr.pref_lang_idx = lng as i16;
                        curr_prio = prio as i32;
                    }
                }
            }
        }
    }

    /// `persistenceRestore()`: persistence manager restore
    fn persistence_restore(&mut self) {
        // The C code uses an uninitialized local; only the fields set by
        // persistenceManagerGetCommand() are used.
        let mut action = UiManagerAction::default();

        // get first action
        let uuid = self.ui_state.uuid;
        let mut result = self.h_persistence.get_command(Some(&uuid), &mut action);

        // loop over remaining actions
        while result != 0 {
            // perform action
            if action.action_type == UI_MANAGER_COMMAND_ACCESSIBILITY_PREFERENCE {
                self.reset_main(0);
                self.apply_preset_main(PRESET_ID_AUTO);
            } else if action.action_type == UI_MANAGER_COMMAND_AUDIO_LANGUAGE_SELECTED {
                self.select_audio_lang_main();
            } else {
                self.perform_action(&action);
            }

            // get next action
            result = self.h_persistence.get_command(None, &mut action);
        }
    }

    /// `UI_Manager_GetAsiPointer(hUiManager)`: get ASI pointer
    pub fn get_asi_pointer(&mut self) -> &mut AudioSceneInfo {
        &mut self.asi
    }

    /// `UI_Manager_SetIsActive(hUiManager, isActive)`
    pub fn set_is_active(&mut self, is_active: u8) -> UiManagerError {
        if self.is_active != is_active {
            self.is_active = is_active;
            self.xml_state_changed = 1;
            self.ui_state_changed = 1;
            self.drc_state_changed = 1;
            if is_active != 0 {
                self.config_changed = 1;
                self.reset_main(0);
                self.apply_preset_main(PRESET_ID_AUTO);

                self.persistence_restore();
            } else {
                self.config_changed = 0;
            }
        }
        UI_MANAGER_OK
    }

    /// `UI_Manager_GetXmlChanged(hUiManager)`: get XML state changed flag
    pub fn get_xml_changed(&mut self) -> i32 {
        self.update();

        self.xml_state_changed as i32
    }

    /// `UI_Manager_ResetXmlChanged(hUiManager)`: reset XML state changed flag
    pub fn reset_xml_changed(&mut self) {
        self.xml_state_changed = 0;
    }

    /// `UI_Manager_GetXmlSceneState(hUiManager, xmlOut, xmlOutSize, flagsIn, flagsOut)`: get XML
    /// scene data. The output buffer size (`xmlOutSize`) is `xml_out.len()`; for an empty
    /// buffer nothing is written (C writes `xmlOut[0]`).
    pub fn get_xml_scene_state(
        &mut self,
        xml_out: &mut [u8],
        mut flags_in: u32,
        flags_out: &mut u32,
    ) -> UiManagerError {
        if let Some(c) = xml_out.first_mut() {
            *c = 0;
        }

        *flags_out = 0;

        // check/update state
        self.update();

        // check if we have to output XML
        if self.xml_state_changed == 0
            && flags_in & UI_MANAGER_FORCE_UPDATE == 0
            && self.xml_writer.next_preset_idx == XML_START_INDEX
        {
            *flags_out |= UI_MANAGER_NO_CHANGE;
            return UI_MANAGER_OK;
        }

        // if something has changed we have to restart XML output
        if self.xml_state_changed != 0 {
            flags_in |= UI_MANAGER_FORCE_RESTART_XML;
        }

        // write XML string
        let err = ui_manager_write_xml(self, xml_out, flags_in, flags_out);

        if err == UI_MANAGER_OK {
            // reset changed flag
            self.xml_state_changed = 0;

            // reset config change flag
            self.config_changed = 0;
        }

        err
    }

    /// `UI_Manager_ApplyXmlAction(hUiManager, xmlIn, xmlInSize, flagsOut)`: apply XML action
    /// (`xmlInSize` is `xml_in.len()`).
    pub fn apply_xml_action(&mut self, xml_in: &[u8], flags_out: &mut u32) -> UiManagerError {
        let mut action = UiManagerAction::default();

        *flags_out = 0;

        self.update();

        if ui_manager_parse_xml_action(xml_in, &mut action) == 0 {
            return UI_MANAGER_INVALID_PARAM;
        }

        let err = self.perform_action(&action);

        if err == UI_MANAGER_OK {
            self.h_persistence.save_command(&action);
        }

        if err != UI_MANAGER_OK {
            self.xml_state_changed = 1;
        }

        err
    }

    /// `UI_Manager_GetUUID(hUiManager, uuid)`: get UUID
    pub fn get_uuid(&self, uuid: &mut [u8; 16]) -> UiManagerError {
        if self.is_active == 0 {
            return UI_MANAGER_INVALID_STATE;
        }

        *uuid = self.ui_state.uuid;

        UI_MANAGER_OK
    }

    /// `UI_Manager_SetUUID(hUiManager, uuid, applyAsiCrc)`: set UUID
    pub fn set_uuid(&mut self, uuid: &[u8; 16], apply_asi_crc: u8) -> UiManagerError {
        if self.is_active == 0 {
            return UI_MANAGER_INVALID_STATE;
        }

        // save previous UUID
        let prev_uuid = self.ui_state.uuid;

        self.update();

        // copy new UUID
        self.ui_state.uuid = *uuid;

        if apply_asi_crc != 0 {
            // xor upper 16 bits of UUID with partial ASI CRC
            self.ui_state.uuid[0] ^= (self.asi.crc_for_uid >> 8) as u8;
            self.ui_state.uuid[1] ^= (self.asi.crc_for_uid & 0xFF) as u8;
        }

        // check if UUID changed, restore state
        if !comp_uuid(&prev_uuid, &self.ui_state.uuid) {
            self.persistence_restore();
        }

        UI_MANAGER_OK
    }

    /// `UI_Manager_GetInteractivityStatus(hUiManager, pUiStatus, pSignalChanged)`: get
    /// interactivity status
    pub fn get_interactivity_status(
        &mut self,
        ui_status: &mut UserInteractivityStatus,
        mut signals_changed: Option<&mut u8>,
    ) -> UiManagerError {
        if self.is_active == 0 {
            return UI_MANAGER_INVALID_STATE;
        }

        self.update();

        // verify current status is valid
        {
            let mut valid = true;

            // check if an active group is not available
            for grp in 0..self.asi.num_groups as usize {
                if self.ui_state.groups[grp].on_off != 0
                    && self.ui_state.groups[grp].is_available == 0
                {
                    valid = false;
                    break;
                }
            }

            // check if no preset is active
            if self.asi.num_group_presets > 0 && self.ui_state.active_preset_index == INVALID_IDX {
                valid = false;
            }

            // reset if not valid
            if !valid {
                self.reset_main(0);

                if self.asi.num_group_presets > 0 {
                    let err = self.apply_preset_main(PRESET_ID_AUTO);
                    if err != UI_MANAGER_OK {
                        return err;
                    }
                }
            }
        }

        if self.ui_state.active_preset_index == INVALID_IDX {
            ui_status.interaction_mode = 0;
            ui_status.group_preset_id = INVALID_ID;
        } else {
            ui_status.interaction_mode = 1;
            ui_status.group_preset_id =
                self.asi.group_presets[self.ui_state.active_preset_index as usize].group_preset_id;
        }

        ui_status.num_groups = self.asi.num_groups;

        for grp in 0..ui_status.num_groups as usize {
            let gs = &self.ui_state.groups[grp];
            let gd = &mut ui_status.group_data[grp];
            gd.group_id = self.asi.groups[grp].group_id;
            if let Some(sc) = signals_changed.as_deref_mut() {
                if gd.on_off != gs.on_off {
                    *sc = 1;
                }
            }
            gd.on_off = gs.on_off;
            gd.route_to_wire = 0;

            gd.change_position = 0;
            if gs.az_offset != gs.default_az_offset
                || gs.el_offset != gs.default_el_offset
                || gs.dist_factor != gs.default_dist_factor
            {
                gd.change_position = 1;
            }

            gd.az_offset = gs.az_offset;
            gd.el_offset = gs.el_offset;
            gd.dist_fact = gs.dist_factor;

            gd.change_gain = 0;
            if gs.gain != gs.default_gain {
                gd.change_gain = 1;
            }

            gd.gain = 0.max((gs.gain as i32 >> 1) + 64) as u8;
        }

        self.ui_state_changed = 0;

        UI_MANAGER_OK
    }

    /// `UI_Manager_GetDrcLoudnessStatus(hUiManager, pDrcLoudnessStatus)`: get DRC/loudness status
    pub fn get_drc_loudness_status(
        &mut self,
        drc_loudness_status: &mut UiDrcLoudnessStatus,
    ) -> UiManagerError {
        if self.is_active == 0 {
            return UI_MANAGER_INVALID_STATE;
        }

        drc_loudness_status.target_loudness = self.ui_state.target_loudness;

        // if accessibility preference is set to hearing impaired, force dialog DRC mode,
        // otherwise return selected mode
        if self.ui_state.accessibility_preference >= 2 {
            drc_loudness_status.drc_selected = get_drc_effect_type_fallback(5);
        } else {
            drc_loudness_status.drc_selected = self.ui_state.drc_selected;
        }

        drc_loudness_status.album_mode = self.ui_state.album_mode;
        drc_loudness_status.boost = self.ui_state.boost;
        drc_loudness_status.compress = self.ui_state.compress;

        self.drc_state_changed = 0;

        UI_MANAGER_OK
    }

    /// `UI_Manager_GetStatusChanged(hUiManager, pInteractivityStatusChanged,
    /// pDrcLoudnessStatusChanged)`: check status change
    pub fn get_status_changed(
        &mut self,
        interactivity_status_changed: Option<&mut u8>,
        drc_loudness_status_changed: Option<&mut u8>,
    ) -> UiManagerError {
        self.update();

        if let Some(p) = interactivity_status_changed {
            *p = self.ui_state_changed;
        }
        if let Some(p) = drc_loudness_status_changed {
            *p = self.drc_state_changed;
        }

        UI_MANAGER_OK
    }

    /// `UI_Manager_SetPersistenceMemory(hUiManager, persistenceMemoryBlock,
    /// persistenceMemorySize)`: set persistence memory. The block is owned by the UI manager
    /// until it is replaced; its size is `min(len, 65535)`.
    pub fn set_persistence_memory(
        &mut self,
        persistence_memory_block: Option<Vec<u8>>,
    ) -> UiManagerError {
        let r = self.h_persistence.set_memory(persistence_memory_block);

        match r {
            -1 => UI_MANAGER_BUFFER_TOO_SMALL,
            -2 => UI_MANAGER_INVALID_PARAM,
            0 => UI_MANAGER_OK_BUT_NO_VALID_DATA,
            _ => UI_MANAGER_OK,
        }
    }

    /// `UI_Manager_GetPersistenceMemory(hUiManager, persistenceMemoryBlock,
    /// persistenceMemorySize)`: get persistence memory (updates its content). Returns the
    /// current block (`None` if persistence handling is inactive) and the size.
    pub fn get_persistence_memory(&mut self) -> (Option<&[u8]>, u16) {
        self.h_persistence.get_memory()
    }
}
