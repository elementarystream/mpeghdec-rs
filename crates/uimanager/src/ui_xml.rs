// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libUIManager/src/uiXml.h and libUIManager/src/uiXml.cpp.

//! MPEG-H UI manager XML formatting (scene state) and parsing (action events).
//!
//! The writer reproduces the output buffer of the C++ implementation byte by byte, including
//! the bytes behind the terminating NUL (`strncpy()` zero padding, partially written elements
//! of incomplete output).

use mpeghdec_tpdec::*;

use crate::ui_manager::*;

/// `sort()`: bubble sort
fn sort(data: &mut [u16]) {
    let mut n = data.len();

    while n > 1 {
        for i in 1..n {
            if data[i - 1] > data[i] {
                data.swap(i - 1, i);
            }
        }
        n -= 1;
    }
}

/// `FDKstrlen()` of a NUL terminated `char` array
fn strlen(s: &[i8]) -> usize {
    s.iter().position(|&c| c == 0).unwrap_or(s.len())
}

/// `appendIDtoStr()`: append ID to string
fn append_id_to_str(s: &mut [i8], id: u8) {
    let mut p = strlen(s);
    let mut tmp = id;

    if id >= 100 {
        let digit = tmp / 100;
        s[p] = (digit + b'0') as i8;
        p += 1;
        tmp -= digit * 100;
    }

    if id >= 10 {
        let digit = tmp / 10;
        s[p] = (digit + b'0') as i8;
        p += 1;
        tmp -= digit * 10;
    }

    s[p] = (tmp + b'0') as i8;
    p += 1;

    s[p] = 0;
}

/// `resetXmlWriter()`: reset XML writer state
fn reset_xml_writer(w: &mut UiManagerXmlWriter) {
    w.next_preset_idx = XML_START_INDEX;
    w.next_group_idx = XML_START_INDEX;
    w.next_switch_group_member_idx = XML_START_INDEX;
}

/// XML writer of one `uiManagerWriteXML()` call: the persistent writer state and the output
/// buffer.
struct Xw<'a> {
    s: &'a mut UiManagerXmlWriter,
    out: &'a mut [u8],
    /// the buffer behind the write position is known to be zero (`strncpy()` padding done)
    tail_zeroed: bool,
}

impl Xw<'_> {
    /// `writeChar()`: write character
    #[inline]
    fn char(&mut self, c: u8) {
        if self.s.n_left == 0 {
            return;
        }
        self.out[self.s.p_out] = c;
        self.s.p_out += 1;
        self.s.n_left -= 1;
    }

    /// `writeString()`: write string
    fn string(&mut self, s: &str) {
        let s = s.as_bytes();

        if self.s.n_left == 0 {
            return;
        }

        let l = s.len();
        if l as u32 > self.s.n_left {
            self.s.n_left = 0;
            return;
        }

        // FDKstrncpy(pOut, str, nLeft): copy and pad the rest of the buffer with zeros
        let p = self.s.p_out;
        self.out[p..p + l].copy_from_slice(s);
        if !self.tail_zeroed {
            self.out[p + l..p + self.s.n_left as usize].fill(0);
            self.tail_zeroed = true;
        }
        self.s.p_out += l;
        self.s.n_left -= l as u32;
    }

    /// `writeEscapedString()`: write escaped string
    fn escaped_string(&mut self, s: &[i8]) {
        let mut l_max_out = ASI_MAX_DESCRIPTION_LEN as i32;
        // Original Character   XML entity replacement   XML numeric replacement
        // < (0x3c)             &lt;                     &#60;
        // > (0x3e)             &gt;                     &#62;
        // " (0x22)             &quot;                   &#34;
        // & (0x26)             &amp;                    &#38;
        // ' (0x27)             &apos;                   &#39;

        if self.s.n_left == 0 {
            return;
        }

        let l = strlen(s);
        for &c in &s[..l] {
            let c = c as u8;
            let one = [c];
            let chars: &[u8] = match c {
                0x3c => b"&#60;",
                0x3e => b"&#62;",
                0x22 => b"&#34;",
                0x26 => b"&#38;",
                0x27 => b"&#39;",
                _ => &one,
            };
            let chars_len = chars.len() as u32;
            l_max_out -= chars_len as i32;
            if l_max_out < 0 {
                // Truncate written output to the maximum length of the corresponding unescaped
                // string to avoid memory issues.
                return;
            }
            if chars_len > self.s.n_left {
                self.s.n_left = 0;
                return;
            }
            let p = self.s.p_out;
            self.out[p..p + chars.len()].copy_from_slice(chars);
            self.s.p_out += chars.len();
            self.s.n_left -= chars_len;
        }
    }

    /// `writeBool()`: write boolean
    fn bool(&mut self, val: bool) {
        if val {
            self.string("\"true\"");
        } else {
            self.string("\"false\"");
        }
    }

    /// `writeID()`: write ID
    fn id(&mut self, id: u8) {
        if self.s.n_left == 0 {
            return;
        }

        self.char(b'"');

        let mut tmp = id;

        if id >= 100 {
            let digit = tmp / 100;
            self.char(digit + b'0');
            tmp -= digit * 100;
        }

        if id >= 10 {
            let digit = tmp / 10;
            self.char(digit + b'0');
            tmp -= digit * 10;
        }

        self.char(tmp + b'0');

        self.char(b'"');
    }

    /// `writeUUID()`: write UUID
    fn uuid(&mut self, uuid: &[u8; 16]) {
        if self.s.n_left == 0 {
            return;
        }

        self.char(b'"');

        for (i, &b) in uuid.iter().enumerate() {
            if i == 4 || i == 6 || i == 8 || i == 10 {
                self.char(b'-');
            }

            for c in [b >> 4, b & 0xF] {
                self.char(if c < 10 { c + b'0' } else { c + b'A' - 10 });
            }
        }

        self.char(b'"');
    }

    /// `writeValue()`: write gain (value in 0.5 steps)
    fn value(&mut self, val_x2: i16) {
        if self.s.n_left == 0 {
            return;
        }

        self.char(b'"');

        let val_x2 = val_x2.clamp(-1999, 1999);

        if val_x2 < 0 {
            self.char(b'-');
        }

        let i = (val_x2.abs() >> 1) as i32;
        let mut tmp = i;

        if i >= 100 {
            let digit = (tmp / 100) as u8;
            self.char(digit + b'0');
            tmp -= digit as i32 * 100;
        }

        if i >= 10 {
            let digit = (tmp / 10) as u8;
            self.char(digit + b'0');
            tmp -= digit as i32 * 10;
        }

        self.char(tmp as u8 + b'0');

        self.char(b'.');
        self.char(if val_x2 & 1 != 0 { b'5' } else { b'0' });

        self.char(b'"');
    }

    /// Records the current position as end of the last complete element.
    #[inline]
    fn mark_valid(&mut self) {
        self.s.p_last_valid_pos = Some(self.s.p_out);
    }

    /// `writeDescription()`: write description
    fn description(
        &mut self,
        descr: Option<&AsiDescription>,
        content: Option<&AsiContentData>,
        preset_kind: Option<u8>,
    ) {
        if let Some(c) = content {
            if c.content_kind != 0 {
                self.string("<kind table=\"ContentKindTable\" code=");
                self.id(c.content_kind);
                self.string("/>\n");
            }
        }

        if let Some(k) = preset_kind {
            if k != 0 {
                self.string("<kind table=\"PresetTable\" code=");
                self.id(k);
                self.string("/>\n");
            }
        }

        let has_lang = content.is_some_and(|c| c.content_language[0] != 0);
        let descr = descr.filter(|d| d.present != 0);
        if !has_lang && descr.is_none() {
            return;
        }

        self.string("<customKind");

        if let Some(c) = content.filter(|_| has_lang) {
            self.string(" langCode=\"");
            self.char(c.content_language[0] as u8);
            self.char(c.content_language[1] as u8);
            self.char(c.content_language[2] as u8);
            self.char(b'"');
        }

        self.string(">\n");

        if let Some(d) = descr {
            for i in 0..(d.num_languages.max(0) as usize).min(ASI_MAX_DESCRIPTION_LANGUAGES) {
                self.string("<description");
                // `if (pDescr->language[0])` tests an array (always true)
                self.string(" langCode=\"");
                self.char(d.language[i][0] as u8);
                self.char(d.language[i][1] as u8);
                self.char(d.language[i][2] as u8);
                self.char(b'"');

                if i as i32 == d.pref_lang_idx as i32 {
                    self.string(" isPreferred=\"true\"");
                }
                self.string(">");

                self.escaped_string(&d.data[i]);

                self.string("</description>\n");
            }
        }

        self.string("</customKind>\n");
    }

    /// `writeInteractParams()`: write interaction params
    fn interact_params(&mut self, group: &AsiGroup, gs: &UiStateGroup) {
        if gs.allow_gain_interactivity != 0 {
            self.string("<prominenceLevelProp min=");
            self.value(((group.interactivity_min_gain as i32 - 63) << 1) as i16);

            self.string(" max=");
            self.value(((group.interactivity_max_gain as i32) << 1) as i16);

            self.string(" def=");
            self.value(gs.default_gain);

            self.string(" val=");
            self.value(gs.gain);

            self.string(" isActionAllowed=");
            self.bool(gs.allow_gain_interactivity != 0);

            self.string("/>\n");
        }

        if gs.allow_on_off != 0 {
            self.string("<mutingProp def=");
            self.bool(group.default_on_off == 0);

            self.string(" val=");
            self.bool(gs.on_off == 0);

            self.string(" isActionAllowed=");
            self.bool(gs.allow_on_off != 0);

            self.string("/>\n");
        }

        if gs.allow_position_interactivity != 0 {
            if group.interactivity_max_az_offset as i32
                > -(group.interactivity_min_az_offset as i32)
            {
                self.string("<azimuthProp min=");
                self.value((-3 * group.interactivity_min_az_offset as i32) as i16);

                self.string(" max=");
                self.value((3 * group.interactivity_max_az_offset as i32) as i16);

                self.string(" def=");
                self.value((3 * (gs.default_az_offset as i32 - 128)) as i16);

                self.string(" val=");
                self.value((3 * (gs.az_offset as i32 - 128)) as i16);

                self.string(" isActionAllowed=");
                self.bool(gs.allow_position_interactivity != 0);

                self.string("/>\n");
            }

            if group.interactivity_max_el_offset as i32
                > -(group.interactivity_min_el_offset as i32)
            {
                self.string("<elevationProp min=");
                self.value((-6 * group.interactivity_min_el_offset as i32) as i16);

                self.string(" max=");
                self.value((6 * group.interactivity_max_el_offset as i32) as i16);

                self.string(" def=");
                self.value((6 * (gs.default_el_offset as i32 - 32)) as i16);

                self.string(" val=");
                self.value((6 * (gs.el_offset as i32 - 32)) as i16);

                self.string(" isActionAllowed=");
                self.bool(gs.allow_position_interactivity != 0);

                self.string("/>\n");
            }

            // distance factor currently not supported
        }
    }
}

/// Read-only parts of the UI manager used by the XML writer.
struct Ctx<'a> {
    asi: &'a AudioSceneInfo,
    ui: &'a UiState,
    config_changed: u8,
}

impl Ctx<'_> {
    fn descriptions(&self) -> Option<&AsiDescriptions> {
        self.asi.p_descriptions.as_deref()
    }
}

/// Default description with language "eng" and the given text.
fn default_description(text: &[u8], id: Option<u8>) -> AsiDescription {
    let mut d =
        AsiDescription { present: 1, num_languages: 1, pref_lang_idx: 0, ..Default::default() };
    d.language[0] = [b'e' as i8, b'n' as i8, b'g' as i8];
    for (k, &c) in text.iter().enumerate() {
        d.data[0][k] = c as i8;
    }
    if let Some(id) = id {
        append_id_to_str(&mut d.data[0], id);
    }
    d
}

/// `writePreset()`: write preset
fn write_preset(w: &mut Xw, c: &Ctx, preset_idx: i32, sorted_grp_ids: &[u16]) {
    let preset =
        if preset_idx >= 0 { Some(&c.asi.group_presets[preset_idx as usize]) } else { None };
    let description = match c.descriptions() {
        Some(d) if preset_idx >= 0 => Some(&d.group_presets[preset_idx as usize]),
        _ => None,
    };

    if w.s.next_group_idx == XML_START_INDEX {
        w.string("<preset id=");
        w.id(preset.map_or(0, |p| p.group_preset_id));

        w.string(" isActive=");
        w.bool(preset.is_none_or(|_| c.ui.active_preset_index as i32 == preset_idx));

        w.string(" isDefault=");
        w.bool(
            preset.is_none_or(|p| p.group_preset_id as i32 == get_min_preset_id(c.asi, c.ui, 0)),
        );

        w.string(" isAvailable=");
        w.bool(preset.is_none_or(|_| c.ui.group_presets[preset_idx as usize].is_available != 0));

        w.string(">\n");

        if let (Some(d), Some(p)) = (description.filter(|d| d.present != 0), preset) {
            w.description(Some(d), None, Some(p.kind));
        } else if let Some(p) = preset {
            let descr = default_description(b"Preset ", Some(p.group_preset_id));
            w.description(Some(&descr), None, Some(p.kind));
        } else {
            let descr = default_description(b"Default", None);
            w.description(Some(&descr), None, None);
        }
        if w.s.n_left != 0 {
            w.mark_valid();
            w.s.next_group_idx = 0;
        } else {
            return;
        }
    }

    {
        let mut tmp_ui_state;
        let ui_state: &UiState = if c.ui.active_preset_index as i32 == preset_idx || preset_idx < 0
        {
            c.ui
        } else {
            tmp_ui_state = *c.ui;
            simulate_preset(c.asi, c.ui, preset.unwrap().group_preset_id, &mut tmp_ui_state);
            &tmp_ui_state
        };

        // write groups and switch groups
        let mut i = w.s.next_group_idx as usize;
        while i < sorted_grp_ids.len() {
            let id = sorted_grp_ids[i];
            let idx = (id & 0xFF) as usize;
            if id & 0x100 != 0 {
                let sw_state = &ui_state.switch_groups[idx];
                let sg = &c.asi.switch_groups[idx];
                let grp_idx = asi_group_id2idx(
                    c.asi,
                    sg.member_id[sw_state.active_member_index as usize] as i32,
                );
                let allow_switch = sw_state.allow_switch != 0 && sg.num_members > 1;

                if grp_idx >= 0 {
                    let gs = &ui_state.groups[grp_idx as usize];

                    // show switch group only if any interaction allowed
                    let mut allow_any_action = allow_switch
                        || gs.allow_on_off != 0
                        || gs.allow_gain_interactivity != 0
                        || gs.allow_position_interactivity != 0;
                    if gs.on_off == 0 && gs.allow_on_off == 0 {
                        allow_any_action = false;
                    }

                    let mut show = allow_any_action;
                    // show switch group if it is always on and has content info or description
                    if !show && gs.on_off != 0 {
                        let g = &c.asi.groups[grp_idx as usize];
                        if g.cont_present != 0 {
                            if g.content_data.content_kind != 0 {
                                show = true;
                            }
                            if g.content_data.content_language[0] != 0 {
                                show = true;
                            }
                        }
                        // (C++ dereferences pDescriptions without a NULL check)
                        if c.descriptions().is_some_and(|d| d.switch_groups[idx].present != 0) {
                            show = true;
                        }
                    }

                    if show {
                        // write switch group
                        write_switch_group(w, c, idx, ui_state, !allow_any_action);
                    }
                } else {
                    i += 1;
                    continue;
                }
            } else {
                let gs = &ui_state.groups[idx];

                // show group only if any interaction allowed
                let mut allow_any_action = gs.allow_on_off != 0
                    || gs.allow_gain_interactivity != 0
                    || gs.allow_position_interactivity != 0;
                if gs.on_off == 0 && gs.allow_on_off == 0 {
                    allow_any_action = false;
                }

                let mut show = allow_any_action;
                // show group if it is always on and has content info or description
                if !show && gs.on_off != 0 {
                    let g = &c.asi.groups[idx];
                    if g.cont_present != 0 {
                        if g.content_data.content_kind != 0 {
                            show = true;
                        }
                        if g.content_data.content_language[0] != 0 {
                            show = true;
                        }
                    }
                    // (C++ dereferences pDescriptions without a NULL check)
                    if c.descriptions().is_some_and(|d| d.groups[idx].present != 0) {
                        show = true;
                    }
                }

                if show {
                    // write group
                    write_group(w, c, idx, ui_state, if allow_any_action { 0 } else { 1 });
                }
            }

            if w.s.n_left != 0 {
                w.mark_valid();
                w.s.next_group_idx = (i + 1) as i8;
            } else {
                return;
            }
            i += 1;
        }
    }

    w.string("</preset>\n");

    if w.s.n_left != 0 {
        w.mark_valid();
        w.s.next_group_idx = XML_START_INDEX;
        w.s.next_switch_group_member_idx = XML_START_INDEX;
    }
}

/// `writeGroup()`: write group; `non_interact` 0: interactive, 1: non-interactive,
/// 2: `audioElement` without interaction params.
fn write_group(w: &mut Xw, c: &Ctx, group_idx: usize, ui_state: &UiState, non_interact: i32) {
    let group = &c.asi.groups[group_idx];
    let description = c.descriptions().map(|d| &d.groups[group_idx]);
    let gs = &ui_state.groups[group_idx];

    if non_interact == 1 {
        w.string("<nonInteractiveAudioElement");
    } else {
        w.string("<audioElement");
    }

    w.string(" id=");
    w.id(group.group_id);

    w.string(" isAvailable=");
    w.bool(gs.is_available != 0);

    w.string(">\n");

    if non_interact == 0 {
        w.interact_params(group, gs);
    }

    w.description(
        description,
        if group.cont_present != 0 { Some(&group.content_data) } else { None },
        None,
    );

    if non_interact == 1 {
        w.string("</nonInteractiveAudioElement>\n");
    } else {
        w.string("</audioElement>\n");
    }
}

/// `writeSwitchGroup()`: write switch group
fn write_switch_group(
    w: &mut Xw,
    c: &Ctx,
    switch_group_idx: usize,
    ui_state: &UiState,
    non_interact: bool,
) {
    let sg = &c.asi.switch_groups[switch_group_idx];
    let sg_description = c.descriptions().map(|d| &d.switch_groups[switch_group_idx]);
    let sw_state = &ui_state.switch_groups[switch_group_idx];
    let mut grp_idx: i32 = 0;

    if w.s.next_switch_group_member_idx == XML_START_INDEX {
        if non_interact {
            w.string("<nonInteractiveAudioElementSwitch");
        } else {
            w.string("<audioElementSwitch");
        }

        w.string(" id=");
        w.id(sg.switch_group_id);

        w.string(" isAvailable=");
        w.bool(sw_state.is_available != 0);

        if !non_interact {
            w.string(" isActionAllowed=");
            w.bool(sw_state.allow_switch != 0 && sg.num_members > 1);
        }

        w.string(">\n");

        grp_idx =
            asi_group_id2idx(c.asi, sg.member_id[sw_state.active_member_index as usize] as i32);

        if grp_idx < 0 {
            return;
        }

        if !non_interact {
            let mut tmp_group = c.asi.groups[grp_idx as usize];
            tmp_group.allow_on_off = sg.allow_on_off;
            tmp_group.default_on_off = sg.default_on_off;
            w.interact_params(&tmp_group, &ui_state.groups[grp_idx as usize]);

            w.string("<audioElements>\n");

            if w.s.n_left != 0 {
                w.mark_valid();
                w.s.next_switch_group_member_idx = 0;
            } else {
                return;
            }
        }
    }

    if non_interact {
        write_group(w, c, grp_idx as usize, ui_state, 2);
    } else {
        // sort switch group members by ID
        let n = sg.num_members as usize;
        let mut sorted_ids = [0u16; ASI_MAX_SWITCH_GROUP_MEMBERS];
        for i in 0..n {
            sorted_ids[i] = ((sg.member_id[i] as u16) << 8) | i as u16;
        }
        sort(&mut sorted_ids[..n]);

        for i in (w.s.next_switch_group_member_idx.max(0) as usize)..n {
            let grp_idx = asi_group_id2idx(c.asi, (sorted_ids[i] >> 8) as i32);

            if grp_idx < 0 {
                continue;
            }
            let grp_idx = grp_idx as usize;

            let group = &c.asi.groups[grp_idx];
            let gs = &ui_state.groups[grp_idx];
            let group_description = c.descriptions().map(|d| &d.groups[grp_idx]);
            let member = (sorted_ids[i] & 0xFF) as u8;

            if sw_state.allow_switch == 0 && sw_state.active_member_index != member {
                continue;
            }

            w.string("<audioElement id=");
            w.id(group.group_id);

            w.string(" isActive=");
            w.bool(sw_state.active_member_index == member);

            w.string(" isDefault=");
            let is_default = if sw_state.allow_switch != 0 {
                sg.default_group_id == group.group_id
            } else {
                sw_state.active_member_index == member
            };
            w.bool(is_default);

            w.string(" isAvailable=");
            w.bool(gs.is_available != 0);

            w.string(">\n");

            let content = if group.cont_present != 0 { Some(&group.content_data) } else { None };
            if let Some(d) = group_description.filter(|d| d.present != 0) {
                w.description(Some(d), content, None);
            } else {
                let descr = default_description(b"", Some(group.group_id));
                w.description(Some(&descr), content, None);
            }

            w.string("</audioElement>\n");

            if w.s.n_left != 0 {
                w.mark_valid();
                w.s.next_switch_group_member_idx = (i + 1) as i8;
            } else {
                return;
            }
        }

        w.string("</audioElements>\n");
    }

    w.description(sg_description, None, None);

    if non_interact {
        w.string("</nonInteractiveAudioElementSwitch>\n");
    } else {
        w.string("</audioElementSwitch>\n");
    }

    if w.s.n_left != 0 {
        w.mark_valid();
        w.s.next_switch_group_member_idx = XML_START_INDEX;
    }
}

/// `writeDrcInfo()`: write DRC info
fn write_drc_info(w: &mut Xw, c: &Ctx) {
    let drc_info = &c.asi.drc_ui_info;
    let mut drc_set_effect_available: u16 = 0;

    if c.asi.drc_ui_info_present != 0 {
        let n = drc_info.num_target_loudness_conditions as usize;
        if c.ui.target_loudness == UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS {
            // target loudness is not known here. Use superset of all drcSetEffectAvailable
            // entries.
            for i in 0..n {
                drc_set_effect_available |= drc_info.drc_set_effect_available[i];
            }
        } else {
            // choose drcSetEffectAvailable bitmask for given target loudness
            let mut target_loudness_value_lower = -63;
            for i in 0..n {
                let target_loudness_value_upper =
                    drc_info.bs_target_loudness_value_upper[i] as i32 - 63;
                if (c.ui.target_loudness as i32) > target_loudness_value_lower
                    && (c.ui.target_loudness as i32) <= target_loudness_value_upper
                {
                    drc_set_effect_available = drc_info.drc_set_effect_available[i];
                    break;
                }
                target_loudness_value_lower = target_loudness_value_upper;
            }
        }
    } else {
        // if no information is present, all bits shall be set to one
        drc_set_effect_available = 0x7F;
    }

    w.string("<DRCInfo>\n");

    for i in 0..7 {
        if drc_set_effect_available & (1 << i) != 0 {
            w.string("<drcSetEffectAvailable index=");
            w.id(i as u8);
            w.string(" />\n");
        }
    }

    w.string("</DRCInfo>\n");
}

/// `writeScene()`: write scene
fn write_scene(w: &mut Xw, c: &Ctx, short_info: u8) {
    let asi = c.asi;
    let mut sorted_preset_ids = [0u16; ASI_MAX_GROUP_PRESETS];
    let mut sorted_grp_ids = [0u16; ASI_MAX_GROUPS + ASI_MAX_SWITCH_GROUPS];
    let mut main_dlg: u16 = 0xFFFF;

    if w.s.next_preset_idx == XML_START_INDEX {
        // start XML
        w.string("<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n");

        // start AudioSceneConfig
        if short_info == 2 {
            // 2 indicates no UI available
            w.string("<AudioSceneConfig uuid=\"00000000-0000-0000-0000-000000000000\"");
        } else {
            w.string("<AudioSceneConfig uuid=");
            w.uuid(&c.ui.uuid);
        }
        w.string(" version=\"11.1\"");
        w.string(" configChanged=");
        if short_info == 2 {
            // 2 indicates no UI available
            w.bool(false);
        } else {
            w.bool(c.config_changed != 0);
        }

        if short_info != 0 {
            w.string("/>\n");
            return;
        }

        w.string(">\n");

        // write DRC info
        write_drc_info(w, c);

        // start presets
        w.string("<presets>\n");

        if w.s.n_left != 0 {
            w.mark_valid();
            w.s.next_preset_idx = 0;
        } else {
            return;
        }
    }

    // sort presets by ID
    let n_presets = asi.num_group_presets as usize;
    let mut i = 0usize;
    while i < n_presets {
        sorted_preset_ids[i] = ((asi.group_presets[i].group_preset_id as u16) << 8) | i as u16;
        i += 1;
    }
    sort(&mut sorted_preset_ids[..n_presets]);

    // sort groups and switch groups by ID, but main dialog element first
    let mut n_sorted_grp_ids = 0usize;
    i = 0;
    while i < asi.num_groups as usize {
        let tmp = ((asi.groups[i].group_id as u16) << 9) | i as u16;

        // find main dialog ID
        if asi.groups[i].cont_present != 0
            && asi.groups[i].content_data.content_kind == 2
            && tmp < main_dlg
        {
            main_dlg = tmp;
        }

        // check if group is switch group member
        if asi.groups[i].switch_group_id == INVALID_ID {
            sorted_grp_ids[n_sorted_grp_ids] = tmp;
            n_sorted_grp_ids += 1;
        }
        i += 1;
    }

    i = 0;
    while i < asi.num_switch_groups as usize {
        sorted_grp_ids[n_sorted_grp_ids] =
            ((asi.switch_groups[i].switch_group_id as u16) << 9) | 0x100 | i as u16;
        n_sorted_grp_ids += 1;
        i += 1;
    }

    sort(&mut sorted_grp_ids[..n_sorted_grp_ids]);

    if main_dlg != 0xFFFF {
        let sw_grp_id = asi.groups[(main_dlg & 0xFF) as usize].switch_group_id;

        if sw_grp_id != INVALID_ID {
            let sw_grp_idx = asi_switch_group_id2idx(asi, sw_grp_id as i32);
            if sw_grp_idx >= 0 {
                main_dlg = ((sw_grp_id as u16) << 9) | 0x100 | sw_grp_idx as u16;
            }
        }

        i = 1;
        while i < n_sorted_grp_ids {
            if sorted_grp_ids[i] == main_dlg {
                sorted_grp_ids.copy_within(0..i, 1);
                sorted_grp_ids[0] = main_dlg;
                break;
            }
            i += 1;
        }
    }
    let sorted_grp_ids = &sorted_grp_ids[..n_sorted_grp_ids];

    // write presets
    if asi.num_group_presets == 0 && w.s.next_preset_idx == 0 {
        write_preset(w, c, -1, sorted_grp_ids);

        if w.s.n_left != 0 {
            w.mark_valid();
            // (the C++ code uses the value of the loop counter of the previous loops here)
            w.s.next_preset_idx = (i + 1) as i8;
        } else {
            return;
        }
    }
    let mut i = w.s.next_preset_idx.max(0) as usize;
    while i < n_presets {
        write_preset(w, c, (sorted_preset_ids[i] & 0xFF) as i32, sorted_grp_ids);

        if w.s.n_left != 0 {
            w.mark_valid();
            w.s.next_preset_idx = (i + 1) as i8;
        } else {
            return;
        }
        i += 1;
    }

    // end presets
    if w.s.next_preset_idx != XML_END_INDEX {
        w.string("</presets>\n");
        if w.s.n_left != 0 {
            w.mark_valid();
            w.s.next_preset_idx = XML_END_INDEX;
        } else {
            return;
        }
    }

    // end AudioSceneConfig
    w.string("</AudioSceneConfig>\n");

    if w.s.n_left != 0 {
        reset_xml_writer(w.s);
    }
}

/// `uiManagerWriteXML(hUiManager, xmlOut, xmlOutSize, flagsIn, flagsOut)`: write complete XML
/// string (`xmlOutSize` is `xml_out.len()`).
pub fn ui_manager_write_xml(
    h: &mut UiManager,
    xml_out: &mut [u8],
    flags_in: u32,
    flags_out: &mut u32,
) -> UiManagerError {
    h.xml_writer.p_out = 0;
    h.xml_writer.n_left = xml_out.len() as u32;
    h.xml_writer.p_last_valid_pos = None;

    if flags_in & UI_MANAGER_FORCE_RESTART_XML != 0 {
        reset_xml_writer(&mut h.xml_writer);
    }

    if h.xml_writer.next_preset_idx != XML_START_INDEX {
        *flags_out |= UI_MANAGER_CONTINUES_XML;
    }

    let ctx = Ctx { asi: &h.asi, ui: &h.ui_state, config_changed: h.config_changed };
    let mut w = Xw { s: &mut h.xml_writer, out: xml_out, tail_zeroed: false };

    {
        let mut short_info = 0;

        if h.is_active == 0 {
            short_info = 2; // 2 indicates no UI available
        }
        write_scene(&mut w, &ctx, short_info);
    }

    if w.s.n_left == 0 {
        let Some(p) = w.s.p_last_valid_pos else {
            // buffer too small to write anything
            if let Some(c) = w.out.first_mut() {
                *c = 0;
            }
            return UI_MANAGER_BUFFER_TOO_SMALL;
        };

        // terminate string after last complete element
        if p < w.out.len() {
            w.out[p] = 0;
        }
        *flags_out |= UI_MANAGER_INCOMPLETE_XML;
    }

    // terminate string
    w.char(0);

    UI_MANAGER_OK
}

/// `findChar()`: find char in string; returns the index behind it (0 if not found)
fn find_char(char_to_find: u8, s: &[u8], len: i32) -> i32 {
    for i in 0..len.max(0) as usize {
        if at(s, i) == char_to_find {
            return i as i32 + 1;
        }
    }

    0
}

/// `findString()`: find string; returns the index behind it (0 if not found)
fn find_string(str_to_find: &[u8], s: &[u8], len: i32) -> i32 {
    let l = str_to_find.len() as u32;
    let len = len as u32;

    if len < l {
        return 0;
    }

    for i in 0..=(len - l) as usize {
        if (0..l as usize).all(|j| at(s, i + j) == str_to_find[j]) {
            return (i + l as usize) as i32;
        }
    }

    0
}

/// Byte of the input (the parser never reads behind a closing `"` found inside the input;
/// out of range reads return NUL).
#[inline]
fn at(s: &[u8], i: usize) -> u8 {
    s.get(i).copied().unwrap_or(0)
}

/// `findAttrib()`: find XML attribute; returns the offset of the value (behind the opening `"`)
fn find_attrib(attrib: &[u8], s: &[u8], len: i32) -> Option<usize> {
    let mut p = 0usize;
    let mut l = len;

    // find attribute name
    let i = find_string(attrib, &s[p..], l);
    if i == 0 {
        return None;
    }
    p += i as usize;
    l -= i;

    // find =
    let i = find_char(b'=', &s[p..], l);
    if i == 0 {
        return None;
    }
    p += i as usize;
    l -= i;

    // find opening "
    let i = find_char(b'"', &s[p..], l);
    if i == 0 {
        return None;
    }
    p += i as usize;
    l -= i;

    // find closing "
    let i = find_char(b'"', &s[p..], l);
    if i == 0 {
        return None;
    }

    Some(p)
}

/// `readInt()`: read int
fn read_int(s: &[u8]) -> i32 {
    let mut i: i32 = 0;
    let mut p = 0;
    let mut sgn = 1;

    // sign
    if at(s, p) == b'-' {
        sgn = -1;
        p += 1;
    }

    while at(s, p) != b'"' {
        let c = at(s, p);
        if !c.is_ascii_digit() {
            return 0;
        }
        i = i.wrapping_mul(10).wrapping_add((c - b'0') as i32);
        p += 1;
    }

    if sgn < 0 {
        i = i.wrapping_neg();
    }

    i
}

/// `readFloat()`: read float (result in Q16)
fn read_float(s: &[u8]) -> i32 {
    // parse float value
    let mut i: i32 = 0;
    let mut f: i32 = 0;
    let mut nf: i32 = 1;
    let mut sgn = 0;
    let mut p = 0;

    // sign
    if at(s, p) == b'-' {
        sgn = -1;
        p += 1;
    } else if at(s, p) == b'+' {
        sgn = 1;
        p += 1;
    }

    // integer part
    while at(s, p) != b'"' && at(s, p) != b'.' {
        let c = at(s, p);
        if !c.is_ascii_digit() {
            return 0;
        }
        i = i.wrapping_mul(10).wrapping_add((c - b'0') as i32);
        p += 1;
    }

    // fractional part
    if at(s, p) == b'.' {
        p += 1;
    }
    while at(s, p) != b'"' {
        let c = at(s, p);
        if !c.is_ascii_digit() {
            return 0;
        }
        f = 10 * f + (c - b'0') as i32;
        nf *= 10;
        if nf >= 10000 {
            break;
        }
        p += 1;
    }

    // store result in Q16 format
    let mut res = (i.wrapping_shl(16)).wrapping_add((f << 16) / nf);
    if sgn < 0 {
        res = res.wrapping_neg();
    }

    res
}

/// `readText()`: read text
fn read_text(s: &[u8], text: &mut [i8; MAX_XML_TEXT_LENGTH + 1]) {
    let mut i = 0;

    while i < MAX_XML_TEXT_LENGTH {
        if at(s, i) == b'"' {
            break;
        }

        text[i] = at(s, i) as i8;
        i += 1;
    }

    text[i] = 0;
}

/// `FDKstrncmp(str, lit, n) == 0`
fn strncmp_eq(s: &[u8], lit: &[u8], n: usize) -> bool {
    for k in 0..n {
        let a = at(s, k);
        let b = lit.get(k).copied().unwrap_or(0);
        if a != b {
            return false;
        }
        if a == 0 {
            return true;
        }
    }
    true
}

/// `readBool()`: read boolean
fn read_bool(s: &[u8]) -> u8 {
    if strncmp_eq(s, b"true", 4) {
        return 1;
    }
    if strncmp_eq(s, b"false", 4) {
        return 0;
    }
    (read_int(s) != 0) as u8
}

/// `readUUID()`: read UUID
fn read_uuid(s: &[u8], uuid: &mut [u8; 16]) {
    let hex = |c: u8| -> Option<u8> {
        match c {
            b'0'..=b'9' => Some(c - b'0'),
            b'A'..=b'F' => Some(c - b'A' + 10),
            b'a'..=b'f' => Some(c - b'a' + 10),
            _ => None,
        }
    };
    let mut p = 0;

    for i in 0..16 {
        if i == 4 || i == 6 || i == 8 || i == 10 {
            if at(s, p) != b'-' {
                break;
            }
            p += 1;
        }

        let Some(hi) = hex(at(s, p)) else { break };
        p += 1;

        let Some(lo) = hex(at(s, p)) else { break };
        p += 1;
        uuid[i] = (hi << 4).wrapping_add(lo);
    }
}

/// `uiManagerParseXmlAction(xmlIn, xmlInSize, pAction)`: parse XML action (`xmlInSize` is
/// `xml_in.len()`). Returns the number of parsed bytes, 0 on failure.
pub fn ui_manager_parse_xml_action(xml_in: &[u8], action: &mut UiManagerAction) -> u32 {
    let mut l = xml_in.len() as i32;
    let mut p = 0usize;

    action.uuid = [0; 16];
    action.action_type = UI_MANAGER_COMMAND_INVALID;
    action.param_int = 0;
    action.param_float = 0;
    action.param_bool = 0;
    action.param_text[0] = 0;
    action.present_flags = 0;

    // find ActionEvent tag
    let i = find_string(b"<ActionEvent", &xml_in[p..], l);
    if i == 0 {
        return 0;
    }
    p += i as usize;
    l -= i;

    // find end of ActionEvent
    let i = find_string(b"/>", &xml_in[p..], l);
    if i == 0 {
        return 0;
    }
    l = i - 2;

    let s = &xml_in[p..];

    // parse uuid attribute
    let Some(a) = find_attrib(b"uuid", s, l) else { return 0 };
    read_uuid(&s[a..], &mut action.uuid);

    // parse version attribute
    let Some(a) = find_attrib(b"version", s, l) else { return 0 };
    let version = read_float(&s[a..]);
    if !(0x90000..=0xB1999).contains(&version) {
        return 0;
    }

    // parse actionType attribute
    let Some(a) = find_attrib(b"actionType", s, l) else { return 0 };
    action.action_type = read_int(&s[a..]) as u8;

    if action.action_type == UI_MANAGER_COMMAND_FORCE_SCENESTATE_UPDATE && version < 0xB1999 {
        return 0;
    }

    // parse paramInt attribute
    if let Some(a) = find_attrib(b"paramInt", s, l) {
        action.param_int = read_int(&s[a..]);
        action.present_flags |= FLAG_XML_PARAM_INT;
    }

    // parse paramFloat attribute
    if let Some(a) = find_attrib(b"paramFloat", s, l) {
        action.param_float = read_float(&s[a..]);
        action.present_flags |= FLAG_XML_PARAM_FLOAT;
    }

    // parse paramBool attribute
    if let Some(a) = find_attrib(b"paramBool", s, l) {
        action.param_bool = read_bool(&s[a..]);
        action.present_flags |= FLAG_XML_PARAM_BOOL;
    }

    // parse paramText attribute
    if let Some(a) = find_attrib(b"paramText", s, l) {
        read_text(&s[a..], &mut action.param_text);
        action.present_flags |= FLAG_XML_PARAM_TEXT;
    }

    // Note: p is always positive
    (p as u32).wrapping_add(l as u32).wrapping_add(2)
}
