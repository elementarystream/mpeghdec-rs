// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Shared helpers of the transport decoder differential tests: state dumps mirroring
//! shim/tpdec.cpp, recording callbacks, a C/Rust decoder pair and bitstream generators.
#![allow(dead_code)]

use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_cref::tpdec::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::cicp2geometry::*;
use mpeghdec_fdk::common_fix::fixnormz_s;
use mpeghdec_fdk::crc::FdkCrcInfo;
use mpeghdec_tpdec::*;

// ---- state dump (mirrors shim/tpdec.cpp) ----

pub fn hash_bytes(p: &[u8]) -> u32 {
    p.iter().fold(0u32, |h, &b| h.wrapping_mul(31).wrapping_add(b as u32))
}

fn put_speaker(o: &mut Vec<i32>, s: &MpeghSpeaker) {
    o.push(s.lfe as i32);
    o.push(s.az as i32);
    o.push(s.el as i32);
}

pub fn put_asc(o: &mut Vec<i32>, a: &CSAudioSpecificConfig) {
    put_asc_with(o, a, &|g| a.m_sc.signal_group_speakers(g));
}

pub fn put_asc_with<'a>(
    o: &mut Vec<i32>,
    a: &CSAudioSpecificConfig,
    spk_of: &dyn Fn(usize) -> &'a [MpeghSpeaker],
) {
    let u = &a.m_sc;
    o.push(u.m_frame_length_flag as i32);
    o.push(u.m_core_sbr_frame_length_index as i32);
    o.push(u.m_sbr_ratio_index as i32);
    o.push(u.m_n_usac_channels as i32);
    o.push(u.m_channel_configuration_index as i32);
    o.push(u.m_usac_num_elements as i32);
    for e in u.element.iter() {
        o.extend_from_slice(&[
            e.usac_element_type,
            e.fullband_lpd as i32,
            e.lpd_stereo_index as i32,
            e.m_noise_filling as i32,
            e.enhanced_noise_filling as i32,
            e.igf_use_enf as i32,
            e.igf_use_whitening as i32,
            e.igf_after_tns_synth as i32,
            e.igf_independent_tiling as i32,
            e.igf_start_index as i32,
            e.igf_use_high_res as i32,
            e.igf_stop_index as i32,
            e.igf_n_tiles as i32,
            e.shift_index1 as i32,
            e.shift_channel1 as i32,
        ]);
        let x = &e.ext_element;
        o.push(x.usac_ext_element_type);
        o.push(x.usac_ext_element_default_length as i32);
        o.push(x.usac_ext_element_payload_frag as i32);
        o.push(x.usac_ext_element_has_audio_pre_roll as i32);
        match x.usac_ext_element_type {
            ID_EXT_ELE_OBJ_METADATA => {
                let m = &x.ext_config.oam;
                o.extend_from_slice(&[
                    m.oam_frame_length as i32,
                    m.low_delay_metadata_coding as i32,
                    m.has_dynamic_object_priority as i32,
                    m.has_uniform_spread as i32,
                    m.num_object_signals as i32,
                ]);
            }
            ID_EXT_ELE_MCT => o.push(x.ext_config.mct.mct_chan_mask as i32),
            ID_EXT_ELE_PROD_METADATA => {
                let p = &x.ext_config.prod_metadata;
                o.extend_from_slice(&[
                    p.has_object_distance as i32,
                    p.has_reference_distance as i32,
                    p.bs_reference_distance as i32,
                ]);
            }
            _ => {}
        }
    }
    for g in 0..TP_MPEGH_MAX_SIGNAL_GROUPS {
        let s = &u.m_signal_group_type[g];
        o.push(s.type_ as i32);
        o.push(s.count as i32);
        o.push(s.layout as i32);
        let (kind, idx, avail) = match s.speakers {
            SpeakerRef::None => (0, 0, 0),
            SpeakerRef::ReferenceLayout => (1, 0, TP_USAC_MAX_SPEAKERS),
            SpeakerRef::SignalGroup(n) => (2, n as i32, 2 * 28 - n as usize),
        };
        o.push(kind);
        o.push(idx);
        o.push(s.first_sig_idx as i32);
        o.push(s.b_use_custom_downmix_matrix as i32);
        let n = (s.count as usize).min(avail);
        let spk = spk_of(g);
        for i in 0..n {
            put_speaker(o, &spk[i]);
        }
    }
    for s in u.m_signal_group_type_speakers.iter() {
        put_speaker(o, s);
    }
    o.push(u.reference_layout as i32);
    for s in u.speakers.iter() {
        put_speaker(o, s);
    }
    o.extend_from_slice(&[
        u.bs_num_signal_groups as i32,
        u.mpegh3da_profile_level_indication as i32,
        u.num_audio_objects as i32,
        u.num_hoa_transport_channels as i32,
        u.ch_num_signal_groups as i32,
        u.obj_num_signal_groups as i32,
    ]);
    o.extend(u.obj_signal_groups_indices.iter().map(|&v| v as i32));
    o.extend(u.sig_idx2_grp_idx.iter().map(|&v| v as i32));
    o.extend_from_slice(&[
        u.passive_downmix_flag as i32,
        u.phase_align_strength as i32,
        u.immersive_downmix_flag as i32,
        u.downmix_config_type as i32,
        u.downmix_id_count as i32,
        u.sub_stream_index as i32,
        u.target_layout,
        u.ui_manager_active as i32,
        u.num_audio_channels as i32,
        u.m_usac_config_extension_present as i32,
        u.element_length_present as i32,
    ]);
    o.extend_from_slice(&[
        a.m_aot,
        a.m_sampling_frequency as i32,
        a.m_samples_per_frame as i32,
        a.m_direct_mapping as i32,
        a.m_extension_audio_object_type,
        a.m_extension_sampling_frequency as i32,
        a.m_channel_configuration as i32,
        a.m_ep_config as i32,
        a.m_vcb11_flag as i32,
        a.m_rvlc_flag as i32,
        a.m_hcr_flag as i32,
        a.m_sbr_present_flag as i32,
        a.m_ps_present_flag as i32,
        a.m_sampling_frequency_index as i32,
        a.m_extension_sampling_frequency_index as i32,
        a.m_extension_channel_configuration as i32,
        a.config_mode as i32,
        a.aac_config_changed as i32,
        a.sbr_config_changed as i32,
        a.sac_config_changed as i32,
    ]);
}

fn put_description(o: &mut Vec<i32>, d: &AsiDescription) {
    o.push(d.present as i32);
    o.push(d.num_languages as i32);
    o.push(d.pref_lang_idx as i32);
    for i in 0..ASI_MAX_DESCRIPTION_LANGUAGES {
        o.extend(d.language[i].iter().map(|&v| v as i32));
        o.extend(d.data[i].iter().map(|&v| v as i32));
    }
}

fn put_screen(o: &mut Vec<i32>, s: &AsiProductionScreenSizeData) {
    o.extend_from_slice(&[
        s.has_non_standard_screen_size as i32,
        s.is_centered_in_azimuth as i32,
        s.bs_screen_size_az as i32,
        s.bs_screen_size_left_az as i32,
        s.bs_screen_size_right_az as i32,
        s.bs_screen_size_top_el as i32,
        s.bs_screen_size_bottom_el as i32,
    ]);
}

fn put_condition(o: &mut Vec<i32>, c: &AsiGroupPresetCondition) {
    o.extend_from_slice(&[
        c.reference_id as i32,
        c.condition_on_off as i32,
        c.disable_gain_interactivity as i32,
        c.gain_flag as i32,
        c.gain as i32,
        c.disable_position_interactivity as i32,
        c.position_flag as i32,
        c.az_offset as i32,
        c.el_offset as i32,
        c.dist_factor as i32,
        c.is_switch_group_condition as i32,
    ]);
}

pub fn put_asi(o: &mut Vec<i32>, a: &AudioSceneInfo) {
    o.extend(a.is_main_stream.iter().map(|&v| v as i32));
    o.push(a.audio_scene_info_id as i32);
    o.push(a.num_groups as i32);
    for p in a.groups.iter() {
        o.extend_from_slice(&[
            p.group_id as i32,
            p.allow_on_off as i32,
            p.default_on_off as i32,
            p.allow_position_interactivity as i32,
            p.interactivity_min_az_offset as i32,
            p.interactivity_max_az_offset as i32,
            p.interactivity_min_el_offset as i32,
            p.interactivity_max_el_offset as i32,
            p.interactivity_min_dist_factor as i32,
            p.interactivity_max_dist_factor as i32,
            p.allow_gain_interactivity as i32,
            p.interactivity_min_gain as i32,
            p.interactivity_max_gain as i32,
            p.is_available as i32,
            p.priority as i32,
            p.closest_speaker_playout as i32,
            p.num_members as i32,
            p.has_conjunct_members as i32,
            p.start_id as i32,
        ]);
        o.extend(p.meta_data_element_id.iter().map(|&v| v as i32));
        o.push(p.ext_present as i32);
        o.push(p.cont_present as i32);
        o.push(p.content_data.content_kind as i32);
        o.extend(p.content_data.content_language.iter().map(|&v| v as i32));
        o.push(p.switch_group_id as i32);
    }
    o.push(a.num_switch_groups as i32);
    for p in a.switch_groups.iter() {
        o.extend_from_slice(&[
            p.switch_group_id as i32,
            p.allow_on_off as i32,
            p.default_on_off as i32,
            p.num_members as i32,
        ]);
        o.extend(p.member_id.iter().map(|&v| v as i32));
        o.push(p.default_group_id as i32);
    }
    o.push(a.num_group_presets as i32);
    for p in a.group_presets.iter() {
        o.push(p.group_preset_id as i32);
        o.push(p.kind as i32);
        o.push(p.num_conditions as i32);
        for c in p.conditions.iter() {
            put_condition(o, c);
        }
        o.push(p.ext_present as i32);
        o.push(p.has_downmix_id_extension as i32);
        o.push(p.downmix_id_extension.downmix_id as i32);
        o.push(p.downmix_id_extension.num_conditions as i32);
        for c in p.downmix_id_extension.conditions.iter() {
            put_condition(o, c);
        }
        put_screen(o, &p.production_screen_size);
    }
    o.push(a.p_descriptions.is_some() as i32);
    if let Some(d) = a.p_descriptions.as_deref() {
        for x in d.groups.iter() {
            put_description(o, x);
        }
        for x in d.switch_groups.iter() {
            put_description(o, x);
        }
        for x in d.group_presets.iter() {
            put_description(o, x);
        }
    }
    o.extend(a.meta_data_element_id_offset.iter().map(|&v| v as i32));
    o.extend(a.meta_data_element_id_max_avail.iter().map(|&v| v as i32));
    o.push(a.comp_pairs_present as i32);
    o.push(a.num_composite_pairs as i32);
    for p in a.composite_pairs.iter() {
        o.push(p[0] as i32);
        o.push(p[1] as i32);
    }
    o.push(a.scrn_size_present as i32);
    o.push(a.scrn_size_ext_present as i32);
    put_screen(o, &a.production_screen_size_data);
    o.push(a.loud_comp_present as i32);
    let l = &a.loudness_compensation_data;
    o.push(l.group_loudness_present as i32);
    o.extend(l.group_loudness.iter().map(|&v| v as i32));
    o.push(l.default_params_present as i32);
    o.extend(l.default_include_group.iter().map(|&v| v as i32));
    o.push(l.default_min_max_gain_present as i32);
    o.push(l.default_min_gain as i32);
    o.push(l.default_max_gain as i32);
    for p in 0..ASI_MAX_GROUP_PRESETS {
        o.push(l.preset_params_present[p] as i32);
        o.extend(l.preset_include_group[p].iter().map(|&v| v as i32));
        o.push(l.preset_min_max_gain_present[p] as i32);
        o.push(l.preset_min_gain[p] as i32);
        o.push(l.preset_max_gain[p] as i32);
    }
    o.push(a.drc_ui_info_present as i32);
    o.push(a.drc_ui_info.num_target_loudness_conditions as i32);
    for i in 0..ASI_MAX_TARGET_LOUDNESS_CONDITIONS {
        o.push(a.drc_ui_info.bs_target_loudness_value_upper[i] as i32);
        o.push(a.drc_ui_info.drc_set_effect_available[i] as i32);
    }
    o.push(a.scene_disp_present as i32);
    o.push(a.diff_flags as i32);
    for l in a.pref_descr_languages.iter() {
        o.extend(l.iter().map(|&v| v as i32));
    }
    o.push(a.active_dmx_id as i32);
    o.push(a.crc_for_uid as i32);
    o.push(a.crc_for_compare as i32);
}

pub fn put_ctrl(o: &mut Vec<i32>, c: &CCtrlCFGChange) {
    o.extend_from_slice(&[
        hash_bytes(&c.mpegh3da_config) as i32,
        c.mpegh3da_config_len as i32,
        c.flush_cnt as i32,
        c.flush_status as i32,
        c.build_up_cnt as i32,
        c.build_up_status as i32,
        c.cfg_changed as i32,
        c.content_changed as i32,
        c.force_cfg_change as i32,
        c.truncation_present as i32,
        c.force_crossfade as i32,
    ]);
}

fn put_bs(o: &mut Vec<i32>, b: &FdkBitStream) {
    o.extend_from_slice(&[
        b.cache_word as i32,
        b.bits_in_cache as i32,
        b.h_bit_buf.bit_ndx() as i32,
        b.h_bit_buf.valid_bits,
        b.h_bit_buf.read_offset() as i32,
        b.h_bit_buf.write_offset() as i32,
        b.h_bit_buf.buf_size() as i32,
        b.config_cache as i32,
    ]);
}

pub fn asc_checksum(a: &CSAudioSpecificConfig) -> i32 {
    let mut v = Vec::with_capacity(12000);
    put_asc(&mut v, a);
    v.iter().fold(0u32, |h, &x| h.wrapping_mul(1_000_003).wrapping_add(x as u32)) as i32
}

pub fn rust_state(t: &TransportDec, rec: &Rec) -> Vec<i32> {
    let mut o = Vec::with_capacity(100_000);
    o.push(t.transport_fmt);
    for b in t.bit_stream.iter() {
        put_bs(&mut o, b);
    }
    o.push(t.parser.buffer_fullness);
    o.push(t.parser.end_of_frame as i32);
    o.extend(t.parser.mhas_labels.iter().map(|&v| v as i32));
    o.extend(t.parser.substream_state.iter().map(|&v| v as i32));
    o.extend(t.parser.flags.iter().map(|&v| v as i32));
    for (k, a) in t.asc.iter().enumerate() {
        put_asc_with(&mut o, a, &|g| t.signal_group_speakers(k, g));
    }
    for c in t.ctrl_cfg_change.iter() {
        put_ctrl(&mut o, c);
    }
    o.push(t.global_frame_pos as i32);
    o.extend(t.access_unit_anchor.iter().map(|&v| v as i32));
    o.extend_from_slice(&t.au_length);
    o.extend_from_slice(&[
        t.number_of_raw_data_blocks,
        t.avg_bit_rate as i32,
        t.last_valid_buffer_fullness as i32,
        t.remainder,
        t.missing_access_units,
        t.burst_period as i32,
        t.hold_off_frames as i32,
        t.flags as i32,
        t.target_layout,
        hash_bytes(&t.bs_buffer()) as i32,
    ]);
    put_asi(&mut o, &rec.asi);
    o.extend_from_slice(&rec.pos);
    o.push(rec.counter as i32);
    o
}

// ---- recording callbacks (mirror shim/tpdec.cpp) ----

pub const REG_TRUNC: u32 = 1;
pub const REG_UNIDRC: u32 = 2;
pub const REG_UI: u32 = 4;
pub const REG_EARCON_BS: u32 = 8;
pub const REG_EARCON_CFG: u32 = 16;
pub const REG_EARCON_INFO: u32 = 32;
pub const REG_ASI: u32 = 64;
pub const REG_ALL: u32 = 127;

pub struct Rec {
    pub log: Vec<i32>,
    pub counter: u32,
    pub fail_mask: u32,
    pub fail_period: u32,
    pub fail_phase: u32,
    pub reg: u32,
    pub asi: AudioSceneInfo,
    pub pos: [i32; 3],
    pub full_asc_log: bool,
    /// Coverage statistics: calls per callback type, (flushStatus, buildUpStatus) pairs seen.
    pub stats: [u64; 12],
    pub ctrl_states: u64,
}

/// Coverage statistics accumulated over all pairs of a test process.
pub static STATS: std::sync::Mutex<([u64; 12], u64)> = std::sync::Mutex::new(([0; 12], 0));

impl Rec {
    pub fn new(reg: u32, fail_mask: u32, fail_period: u32, fail_phase: u32) -> Self {
        let asi = AudioSceneInfo { p_descriptions: Some(Box::default()), ..Default::default() };
        Rec {
            log: Vec::new(),
            counter: 0,
            fail_mask,
            fail_period,
            fail_phase,
            reg,
            asi,
            pos: [0; 3],
            full_asc_log: false,
            stats: [0; 12],
            ctrl_states: 0,
        }
    }

    fn fail_ret(&mut self, t: u32) -> i32 {
        self.stats[t as usize] += 1;
        let n = self.counter;
        self.counter = self.counter.wrapping_add(1);
        if (self.fail_mask >> t) & 1 != 0
            && self.fail_period != 0
            && n % self.fail_period == self.fail_phase
        {
            return TRANSPORTDEC_UNKOWN_ERROR;
        }
        0
    }
}

impl TpCallbacks for Rec {
    fn update_config(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        index: usize,
        config_mode: u8,
        config_changed: &mut u8,
    ) -> i32 {
        let asc = &ascs[index];
        self.log.extend_from_slice(&[1, config_mode as i32, *config_changed as i32]);
        if self.full_asc_log {
            put_asc(&mut self.log, asc);
        } else {
            self.log.push(asc_checksum(asc));
        }
        if config_mode & AC_CM_DET_CFG_CHANGE != 0 && self.counter % 7 == 3 {
            *config_changed = 1;
        }
        self.fail_ret(1)
    }

    fn free_mem(&mut self, ascs: &[CSAudioSpecificConfig], index: usize) -> i32 {
        let asc = &ascs[index];
        self.log.extend_from_slice(&[
            2,
            asc.config_mode as i32,
            asc.m_sc.sub_stream_index as i32,
            asc.m_aot,
        ]);
        self.fail_ret(2)
    }

    fn ctrl_cfg_change(&mut self, ctrl: &CCtrlCFGChange) -> i32 {
        self.log.push(3);
        put_ctrl(&mut self.log, ctrl);
        self.ctrl_states |=
            1u64 << ((ctrl.flush_status as u32 & 7) * 8 + (ctrl.build_up_status as u32 & 7));
        self.fail_ret(3)
    }

    fn truncation_msg(&mut self, n_trunc_samples: i32, trunc_type: i32) -> i32 {
        self.log.extend_from_slice(&[4, n_trunc_samples, trunc_type]);
        self.fail_ret(4)
    }
    fn has_truncation_msg(&self) -> bool {
        self.reg & REG_TRUNC != 0
    }

    fn uni_drc(
        &mut self,
        bs: Option<&mut FdkBitStream>,
        full_payload_length: i32,
        payload_type: i32,
        sub_stream_index: i32,
        payload_start: i32,
        aot: AudioObjectType,
    ) -> i32 {
        self.log.push(5);
        self.log.push(bs.is_some() as i32);
        match bs {
            Some(bs) => {
                self.log.push(bs.get_valid_bits());
                self.log.extend_from_slice(&[
                    full_payload_length,
                    payload_type,
                    sub_stream_index,
                    payload_start,
                    aot,
                ]);
                if payload_type == 1 {
                    self.pos[1] = payload_start;
                    self.pos[2] = full_payload_length;
                }
                let mut n =
                    (self.counter.wrapping_mul(13) % (full_payload_length as u32 * 8 + 1)) as i32;
                while n > 0 {
                    let k = n.min(32);
                    self.log.push(bs.read_bits(k as u32) as i32);
                    n -= k;
                }
            }
            None => {
                self.log.push(0);
                self.log.extend_from_slice(&[
                    full_payload_length,
                    payload_type,
                    sub_stream_index,
                    payload_start,
                    aot,
                ]);
            }
        }
        self.fail_ret(5)
    }
    fn has_uni_drc(&self) -> bool {
        self.reg & REG_UNIDRC != 0
    }

    fn user_interact(
        &mut self,
        bs: &mut FdkBitStream,
        packet_type: MhaPactyp,
        mhas_packet_length: u32,
    ) -> i32 {
        self.log.extend_from_slice(&[
            6,
            packet_type,
            mhas_packet_length as i32,
            bs.get_valid_bits(),
        ]);
        let n = (self.counter % 3) as i32 * 8;
        if n <= (mhas_packet_length as i32 - 2) * 8 {
            self.log.push(bs.read_bits(n as u32) as i32);
        }
        self.fail_ret(6)
    }
    fn has_user_interact(&self) -> bool {
        self.reg & REG_UI != 0
    }

    fn parse_dmx_matrix(&mut self, bs: &mut FdkBitStream, usc: &mut CSUsacConfig) -> i32 {
        self.log.extend_from_slice(&[
            7,
            bs.get_valid_bits(),
            usc.target_layout,
            usc.bs_num_signal_groups as i32,
            usc.downmix_config_type as i32,
        ]);
        self.log.push(bs.read_bits(8) as i32);
        usc.m_signal_group_type[0].b_use_custom_downmix_matrix = (self.counter & 1) as u8;
        self.fail_ret(7)
    }

    fn earcon_bs(&mut self, bs: &mut FdkBitStream) -> i32 {
        self.stats[8] += 1;
        self.log.extend_from_slice(&[8, bs.get_valid_bits()]);
        self.counter = self.counter.wrapping_add(1);
        0
    }
    fn has_earcon_bs(&self) -> bool {
        self.reg & REG_EARCON_BS != 0
    }

    fn earcon_config(&mut self, cf: &mut EarconConfig) -> i32 {
        self.stats[9] += 1;
        self.log.extend_from_slice(&[
            9,
            cf.m_num_pcm_signals as i32,
            cf.m_bs_pcm_frame_size_index as i32,
            cf.m_bs_pcm_frame_size as i32,
            cf.m_bs_pcm_loudness_value as i32,
            cf.m_pcm_has_attenuation_gain as i32,
            cf.m_bs_pcm_attenuation_gain as i32,
            cf.m_bs_pcm_truncation_length as i32,
            cf.m_bs_mhas_truncation_length as i32,
            cf.sampling_frequency as i32,
            cf.truncation_flag as i32,
            cf.earcon_flag as i32,
            cf.earcon_parse_error as i32,
        ]);
        self.counter = self.counter.wrapping_add(1);
        0
    }
    fn has_earcon_config(&self) -> bool {
        self.reg & REG_EARCON_CFG != 0
    }

    fn earcon_info(&mut self, info: &mut EarconInfo) -> i32 {
        self.stats[10] += 1;
        self.log.extend_from_slice(&[10, info.m_num_earcons as i32]);
        self.counter = self.counter.wrapping_add(1);
        0
    }
    fn has_earcon_info(&self) -> bool {
        self.reg & REG_EARCON_INFO != 0
    }

    fn asi(&mut self) -> Option<&mut AudioSceneInfo> {
        if self.reg & REG_ASI != 0 {
            Some(&mut self.asi)
        } else {
            None
        }
    }

    fn loudness_info_set_position(&mut self) -> &mut [i32; 3] {
        &mut self.pos
    }
}

// ---- C / Rust decoder pair ----

pub struct Pair {
    pub c: *mut (),
    pub r: Option<Box<TransportDec>>,
    pub rec: Rec,
    pub name: String,
    pub steps: u64,
    state_buf: Vec<i32>,
    log_buf: Vec<i32>,
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe { cref_tp_close(self.c) };
        let mut st = STATS.lock().unwrap();
        for i in 0..12 {
            st.0[i] += self.rec.stats[i];
        }
        st.1 |= self.rec.ctrl_states;
    }
}

impl Pair {
    pub fn new(
        fmt: TransportType,
        layers: u32,
        reg: u32,
        fail: (u32, u32, u32),
        name: &str,
    ) -> Pair {
        let c = unsafe { cref_tp_open(fmt, layers, reg, fail.0, fail.1, fail.2) };
        let r = TransportDec::open(fmt, 0, layers);
        assert_eq!(unsafe { cref_tp_isOpen(c) } != 0, r.is_some(), "open {fmt} {layers}");
        let mut p = Pair {
            c,
            r,
            rec: Rec::new(reg, fail.0, fail.1, fail.2),
            name: name.to_string(),
            steps: 0,
            state_buf: vec![0; 200_000],
            log_buf: vec![0; 1 << 22],
        };
        if p.r.is_some() {
            if std::env::var("TPDEC_FULL_ASC_LOG").is_ok() {
                p.rec.full_asc_log = true;
                unsafe { cref_tp_setFullAscLog(p.c, 1) };
            }
            p.check("open");
        }
        p
    }

    pub fn is_open(&self) -> bool {
        self.r.is_some()
    }

    fn tp(&mut self) -> &mut TransportDec {
        self.r.as_mut().unwrap()
    }

    /// Compares the complete state and the callback logs.
    pub fn check(&mut self, what: &str) {
        self.steps += 1;
        let n = unsafe { cref_tp_state(self.c, self.state_buf.as_mut_ptr()) } as usize;
        let rs = rust_state(self.r.as_ref().unwrap(), &self.rec);
        let cs = &self.state_buf[..n];
        if rs.as_slice() != cs {
            let first = rs.iter().zip(cs.iter()).position(|(a, b)| a != b);
            panic!(
                "{}: state mismatch after {what} (step {}): len {} vs {}, first difference at {:?}: rust {:?} c {:?}",
                self.name,
                self.steps,
                rs.len(),
                cs.len(),
                first,
                first.map(|i| &rs[i.saturating_sub(3)..(i + 4).min(rs.len())]),
                first.map(|i| &cs[i.saturating_sub(3)..(i + 4).min(cs.len())]),
            );
        }
        let ln = unsafe {
            cref_tp_takeLog(self.c, self.log_buf.as_mut_ptr(), self.log_buf.len() as i32)
        } as usize;
        let cl = &self.log_buf[..ln.min(self.log_buf.len())];
        if self.rec.log.len() != ln || self.rec.log[..cl.len()] != *cl {
            let first = self.rec.log.iter().zip(cl.iter()).position(|(a, b)| a != b);
            panic!(
                "{}: callback log mismatch after {what} (step {}): len {} vs {}, first difference at {:?}\nrust {:?}\nc    {:?}",
                self.name,
                self.steps,
                self.rec.log.len(),
                ln,
                first,
                first.map(|i| &self.rec.log[i.saturating_sub(10)..(i + 10).min(self.rec.log.len())]),
                first.map(|i| &cl[i.saturating_sub(10)..(i + 10).min(cl.len())])
            );
        }
        self.rec.log.clear();
    }

    pub fn fill(&mut self, data: &[u8], layer: i32) -> (i32, u32) {
        let mut bv_c = data.len() as u32;
        let mut bv_r = bv_c;
        let ec =
            unsafe { cref_tp_fillData(self.c, data.as_ptr(), data.len() as u32, &mut bv_c, layer) };
        let er = self.tp().fill_data(data, &mut bv_r, layer);
        assert_eq!((er, bv_r), (ec, bv_c), "{}: fill_data", self.name);
        self.check("fill_data");
        (er, bv_r)
    }

    pub fn read_au(&mut self, layer: u32) -> i32 {
        let ec = unsafe { cref_tp_readAccessUnit(self.c, layer) };
        let r = self.r.as_mut().unwrap();
        let er = r.read_access_unit(layer, &mut self.rec);
        assert_eq!(er, ec, "{}: read_access_unit (step {})", self.name, self.steps);
        self.check("read_access_unit");
        er
    }

    pub fn end_au(&mut self) -> i32 {
        let ec = unsafe { cref_tp_endAccessUnit(self.c) };
        let er = self.tp().end_access_unit();
        assert_eq!(er, ec, "{}: end_access_unit", self.name);
        self.check("end_access_unit");
        er
    }

    pub fn au_bits(&mut self, layer: u32) -> (i32, i32) {
        let rc = unsafe { cref_tp_getAuBitsRemaining(self.c, layer) };
        let tc = unsafe { cref_tp_getAuBitsTotal(self.c, layer) };
        let mc = unsafe { cref_tp_getMissingAccessUnitCount(self.c) };
        let fc = unsafe { cref_tp_getFormat(self.c) };
        let crc_c = unsafe { (cref_tp_crcStartReg(self.c, 0), cref_tp_crcCheck(self.c)) };
        let t = self.tp();
        let rr = t.get_au_bits_remaining(layer);
        let tr = t.get_au_bits_total(layer);
        let mut mr = 0;
        t.get_missing_access_unit_count(&mut mr);
        let fr = t.get_format();
        assert_eq!((rr, tr, mr, fr), (rc, tc, mc, fc), "{}: au bits", self.name);
        assert_eq!((t.crc_start_reg(0), t.crc_check()), crc_c);
        t.crc_end_reg(-1);
        self.check("au_bits");
        (rr, tr)
    }

    /// Reads `n` bits (in chunks of at most 32) from the access unit bitstream of `layer`, like
    /// the core decoder does.
    pub fn consume(&mut self, layer: u32, mut n: i32) {
        let bc = unsafe { cref_tp_getBitstream(self.c, layer) };
        while n > 0 {
            let k = n.min(32) as u32;
            let vc = unsafe { cref_bs_readBits(bc, k) };
            let vr = self.tp().get_bitstream(layer).read_bits(k);
            assert_eq!(vr, vc, "{}: consume", self.name);
            n -= k as i32;
        }
        self.check("consume");
    }

    pub fn push(&mut self, layer: u32, n: i32) {
        let bc = unsafe { cref_tp_getBitstream(self.c, layer) };
        unsafe { cref_bs_pushBiDirectional(bc, n) };
        self.tp().get_bitstream(layer).push_bi_directional(n);
        self.check("push");
    }

    pub fn set_param(&mut self, param: i32, value: i32) {
        let ec = unsafe { cref_tp_setParam(self.c, param, value) };
        let er = self.tp().set_param(param, value);
        assert_eq!(er, ec);
        self.check("set_param");
    }

    pub fn oob_config(&mut self, conf: &[u8], layer: u32) -> i32 {
        let mut cc = conf.to_vec();
        let ec =
            unsafe { cref_tp_outOfBandConfig(self.c, cc.as_mut_ptr(), conf.len() as u32, layer) };
        let r = self.r.as_mut().unwrap();
        let er = r.out_of_band_config(conf, layer, &mut self.rec);
        assert_eq!(er, ec, "{}: out_of_band_config", self.name);
        self.check("out_of_band_config");
        er
    }

    pub fn in_band_config(&mut self, conf: &[u8], build_up_status: u8, layer: u32) -> (i32, u8) {
        let mut buf = [0u8; TP_USAC_MAX_CONFIG_LEN];
        let len = conf.len().min(TP_USAC_MAX_CONFIG_LEN);
        buf[..len].copy_from_slice(&conf[..len]);
        let mut cc_c: u32 = 7;
        let ec = unsafe {
            cref_tp_inBandConfig(
                self.c,
                buf.as_mut_ptr(),
                len as u32,
                build_up_status as u32,
                &mut cc_c,
                layer,
            )
        };
        let mut cc_r: u8 = 7;
        let r = self.r.as_mut().unwrap();
        let er =
            r.in_band_config(&buf, len as u32, build_up_status, &mut cc_r, layer, &mut self.rec);
        assert_eq!((er, cc_r as u32), (ec, cc_c), "{}: in_band_config", self.name);
        self.check("in_band_config");
        (er, cc_r)
    }
}

// ---- stream helpers ----

pub fn streams_dir() -> Option<std::path::PathBuf> {
    let d = std::env::var_os("MPEGHDEC_TEST_STREAMS").map(std::path::PathBuf::from).unwrap_or_else(
        || {
            std::env::var_os("MPEGHDEC_WORK_DIR")
                .map(std::path::PathBuf::from)
                .unwrap_or_else(|| {
                    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../work")
                })
                .join("streams")
        },
    );
    if d.is_dir() {
        Some(d)
    } else {
        None
    }
}

/// All `.mhas` files (sorted).
pub fn mhas_files() -> Vec<std::path::PathBuf> {
    let Some(d) = streams_dir() else { return Vec::new() };
    let mut v: Vec<_> = std::fs::read_dir(d)
        .unwrap()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "mhas"))
        .collect();
    v.sort();
    v
}

/// Whether the long variant of the tests was requested (`MPEGHDEC_TPDEC_FULL=1`).
pub fn full() -> bool {
    std::env::var("MPEGHDEC_TPDEC_FULL").is_ok_and(|v| v == "1")
}

/// Reads up to `limit` bytes of a file.
pub fn read_prefix(p: &std::path::Path, limit: usize) -> Vec<u8> {
    use std::io::Read;
    let mut f = std::fs::File::open(p).unwrap();
    let mut v = Vec::new();
    f.by_ref().take(limit as u64).read_to_end(&mut v).unwrap();
    v
}

/// Access unit boundaries from the `.pts.txt` file next to an `.mhas` file.
pub fn au_list(p: &std::path::Path, limit: usize) -> Vec<(usize, usize)> {
    let t = p.with_extension("pts.txt");
    let s = std::fs::read_to_string(t).unwrap_or_default();
    s.lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
        })
        .filter(|&(o, n): &(usize, usize)| o + n <= limit)
        .collect()
}

#[derive(Clone, Debug)]
pub struct MhasPacket {
    pub ptype: u32,
    pub label: u32,
    pub payload: Vec<u8>,
}

/// Splits an MHAS byte stream into packets (stops at the first incomplete packet).
pub fn parse_mhas(data: &[u8]) -> Vec<MhasPacket> {
    let mut v = Vec::new();
    let size = data.len().next_power_of_two().max(1024);
    let mut buf = vec![0u8; size];
    buf[..data.len()].copy_from_slice(data);
    let mut bs = FdkBitStream::default();
    bs.init(&BitMem::from_slice(&buf), size as u32, (data.len() * 8) as u32, FdkBsCfg::BsReader);
    loop {
        if bs.get_valid_bits() < 24 {
            break;
        }
        let ptype = bs.escaped_value(3, 8, 8);
        let label = bs.escaped_value(2, 8, 32);
        let len = bs.escaped_value(11, 24, 24) as usize;
        let vb = bs.get_valid_bits();
        if vb < 0 || (vb as usize) < len * 8 || vb % 8 != 0 {
            break;
        }
        let pos = data.len() - vb as usize / 8;
        v.push(MhasPacket { ptype, label, payload: data[pos..pos + len].to_vec() });
        bs.push_for((len * 8) as u32);
    }
    v
}

// ---- bit writer and generators ----

#[derive(Default, Clone)]
pub struct BitWriter {
    pub bytes: Vec<u8>,
    pub nbits: usize,
}

impl BitWriter {
    pub fn put(&mut self, v: u32, n: u32) {
        for i in (0..n).rev() {
            let bit = (v >> i) & 1;
            if self.nbits % 8 == 0 {
                self.bytes.push(0);
            }
            if bit != 0 {
                *self.bytes.last_mut().unwrap() |= 0x80 >> (self.nbits % 8);
            }
            self.nbits += 1;
        }
    }
    pub fn put_escaped(&mut self, mut v: u32, n1: u32, n2: u32, n3: u32) {
        let m1 = (1u32 << n1) - 1;
        if v < m1 {
            self.put(v, n1);
            return;
        }
        self.put(m1, n1);
        v -= m1;
        let m2 = (1u32 << n2) - 1;
        if v < m2 {
            self.put(v, n2);
            return;
        }
        self.put(m2, n2);
        v -= m2;
        self.put(v, n3);
    }
    pub fn put_bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.put(x as u32, 8);
        }
    }
    pub fn put_writer(&mut self, w: &BitWriter) {
        for i in 0..w.nbits {
            self.put(((w.bytes[i / 8] >> (7 - i % 8)) & 1) as u32, 1);
        }
    }
    pub fn byte_len(&self) -> usize {
        self.nbits.div_ceil(8)
    }
}

fn pick<T: Copy>(g: &mut Rng, v: &[T]) -> T {
    v[(g.next_u32() as usize) % v.len()]
}

/// Writes a payload as `escaped length` + bytes, sometimes with a wrong length.
fn put_sized_payload(w: &mut BitWriter, g: &mut Rng, p: &BitWriter, n1: u32, n2: u32, n3: u32) {
    let mut len = p.byte_len();
    match g.range(0, 30) {
        0 => len += g.range(1, 3) as usize,
        1 => len = len.saturating_sub(1),
        _ => {}
    }
    w.put_escaped(len as u32, n1, n2, n3);
    let mut q = p.clone();
    while q.nbits < len * 8 {
        q.put(g.next_u32() & 0xff, 8.min((len * 8 - q.nbits) as u32));
    }
    for i in 0..(len * 8).min(q.nbits) {
        w.put(((q.bytes[i / 8] >> (7 - i % 8)) & 1) as u32, 1);
    }
}

/// SpeakerConfig3d(); returns the number of speakers (or None for invalid layouts).
fn gen_speaker_config(w: &mut BitWriter, g: &mut Rng, want: Option<usize>) -> Option<usize> {
    let t = match g.range(0, 19) {
        0..=7 => 0,
        8..=10 => 1,
        11..=17 => 2,
        _ => 3,
    };
    w.put(t, 2);
    if t == 0 {
        let cicp = if g.range(0, 15) == 0 { g.range(0, 63) } else { g.range(1, 20) };
        w.put(cicp as u32, 6);
        let n = cicp2geometry_get_num_channels_from_cicp_noerr(cicp);
        return if cicp < 21 && n > 0 { Some(n as usize) } else { None };
    }
    let n = match want {
        Some(n) if g.range(0, 10) != 0 => n,
        _ => {
            ({
                let hi = if g.range(0, 10) == 0 { 40 } else { 28 };
                g.range(1, hi)
            }) as usize
        }
    };
    w.put_escaped(n as u32 - 1, 5, 8, 16);
    if t == 3 {
        return Some(n);
    }
    let valid_ls: Vec<u32> = (0..43u32).filter(|&i| i != 11 && i != 12).collect();
    let ls_az = |idx: u32| -> i32 {
        let mut geo = Cicp2GeometryChannelGeometry::default();
        cicp2geometry_get_geometry_from_cicp_loudspeaker_index(idx as i32, Some(&mut geo));
        geo.az as i16 as i32
    };
    if t == 1 {
        for _ in 0..n {
            let idx = if g.range(0, 30) == 0 { g.range(0, 127) as u32 } else { pick(g, &valid_ls) };
            w.put(idx, 7);
        }
        return Some(n);
    }
    let prec = g.range(0, 1) as u32;
    w.put(prec, 1);
    let mut i = 0;
    while i < n {
        let az;
        if g.range(0, 3) == 0 {
            w.put(1, 1);
            let idx = pick(g, &valid_ls);
            w.put(idx, 7);
            az = ls_az(idx);
        } else {
            w.put(0, 1);
            let ec = g.range(0, 3) as u32;
            w.put(ec, 2);
            if ec == 3 {
                let a = g.range(0, if prec != 0 { 127 } else { 31 }) as u32;
                w.put(a, if prec != 0 { 7 } else { 5 });
                if a != 0 {
                    w.put(g.range(0, 1) as u32, 1);
                }
            }
            let a = match g.range(0, 4) {
                0 => 0,
                1 => {
                    if prec != 0 {
                        180
                    } else {
                        36
                    }
                }
                _ => g.range(0, if prec != 0 { 255 } else { 63 }),
            };
            w.put(a as u32, if prec != 0 { 8 } else { 6 });
            let mut az0 = a * if prec != 0 { 1 } else { 5 };
            if az0 != 0 && az0 != 180 {
                let d = g.range(0, 1);
                w.put(d as u32, 1);
                if d != 0 {
                    az0 = -az0;
                }
            }
            az = az0;
            w.put((g.range(0, 7) == 0) as u32, 1); // isLFE
        }
        if az != 0 && az != 180 {
            // pair: sometimes also at the very end (writes one speaker too many)
            let pair = if i + 1 < n { g.range(0, 2) == 0 } else { g.range(0, 6) == 0 };
            w.put(pair as u32, 1);
            if pair {
                i += 1;
            }
        }
        i += 1;
    }
    Some(n)
}

fn gen_ext_element(
    w: &mut BitWriter,
    g: &mut Rng,
    ty: u32,
    num_signals: usize,
    num_obj_groups: usize,
    num_ch_groups: usize,
) {
    w.put_escaped(ty, 4, 8, 16);
    let mut p = BitWriter::default();
    match ty as i32 {
        ID_EXT_ELE_OBJ_METADATA => {
            p.put((g.range(0, 15) != 0) as u32, 1); // lowDelayMetadataCoding
            let has_core_length = g.range(0, 2) != 0;
            p.put(has_core_length as u32, 1);
            if !has_core_length {
                p.put(pick(g, &[15u32, 7, 3, 20]), 6);
            }
            let scr = g.range(0, 3) == 0;
            p.put(scr as u32, 1);
            if scr {
                p.put(
                    (g.next_u32() as u64 & ((1u64 << num_signals) - 1)) as u32,
                    num_signals as u32,
                );
            }
            p.put(g.range(0, 1) as u32, 1);
            p.put(g.range(0, 1) as u32, 1);
        }
        ID_EXT_ELE_MCT => {
            p.put((g.next_u32() as u64 & ((1u64 << num_signals) - 1)) as u32, num_signals as u32);
        }
        ID_EXT_ELE_PROD_METADATA => {
            let hrd = g.range(0, 1) as u32;
            p.put(hrd, 1);
            if hrd != 0 {
                p.put(g.range(0, 127) as u32, 7);
            }
            if num_obj_groups > 0 {
                p.put(g.next_u32() >> (32 - num_obj_groups), num_obj_groups as u32);
            }
            for _ in 0..num_ch_groups {
                p.put(g.range(0, 1) as u32, 1);
            }
        }
        _ => {
            for _ in 0..g.range(0, 6) {
                p.put(g.next_u32() & 0xff, 8);
            }
        }
    }
    let mut len = p.byte_len();
    match g.range(0, 25) {
        0 => len += 1,
        1 => len = len.saturating_sub(1),
        _ => {}
    }
    w.put_escaped(len as u32, 4, 8, 16);
    let dl = g.range(0, 3) == 0;
    w.put(dl as u32, 1);
    if dl {
        w.put_escaped(g.range(0, 300) as u32, 8, 16, 0);
    }
    w.put(g.range(0, 1) as u32, 1); // payloadFrag
    for i in 0..len * 8 {
        let b = if i < p.nbits {
            (p.bytes[i / 8] >> (7 - i % 8)) & 1
        } else {
            (g.next_u32() & 1) as u8
        };
        w.put(b as u32, 1);
    }
}

fn gen_core_config(w: &mut BitWriter, g: &mut Rng, cpe: bool, nbits: i32) {
    w.put((g.range(0, 40) == 0) as u32, 1); // tw_mdct
    w.put(g.range(0, 1) as u32, 1); // fullbandLpd
    w.put(g.range(0, 1) as u32, 1); // noiseFilling
    let enf = g.range(0, 2) == 0;
    w.put(enf as u32, 1);
    if enf {
        w.put(g.next_u32() & 0xf, 4);
        w.put(g.range(0, 31) as u32, 5);
        w.put(g.range(0, 15) as u32, 4);
    }
    if cpe {
        if enf {
            w.put(g.range(0, 1) as u32, 1);
        }
        w.put(if g.range(0, 30) == 0 { g.range(1, 3) as u32 } else { 0 }, 2); // qceIndex
        let s = g.range(0, 3) == 0;
        w.put(s as u32, 1);
        if s {
            w.put(g.next_u32() & ((1u32 << nbits) - 1), nbits as u32);
        }
        w.put(g.range(0, 1) as u32, 1); // lpdStereoIndex
    }
}

/// Generates an ASI payload (mae_AudioSceneInfo()).
pub fn gen_asi(g: &mut Rng, main: bool, max_ids: usize) -> BitWriter {
    let mut w = BitWriter::default();
    w.put(main as u32, 1);
    if !main {
        w.put(g.range(0, 30) as u32, 7);
        w.put(g.range(0, 60) as u32, 7);
        return w;
    }
    let id_present = g.range(0, 1) as u32;
    w.put(id_present, 1);
    if id_present != 0 {
        w.put(g.next_u32() & 0xff, 8);
    }
    let ng = if g.range(0, 20) == 0 { g.range(0, 40) } else { g.range(0, 5) } as usize;
    w.put(ng as u32, 7);
    let mut group_ids = Vec::new();
    let mut next_id = 0u32;
    for _ in 0..ng.min(40) {
        let gid = if g.range(0, 10) == 0 { g.range(0, 127) as u32 } else { g.range(0, 20) as u32 };
        group_ids.push(gid);
        w.put(gid, 7);
        w.put(g.range(0, 1) as u32, 1);
        w.put(g.range(0, 1) as u32, 1);
        let ap = g.range(0, 1) as u32;
        w.put(ap, 1);
        if ap != 0 {
            for n in [7, 7, 5, 5, 4, 4] {
                w.put(g.next_u32() & ((1 << n) - 1), n);
            }
        }
        let agi = g.range(0, 1) as u32;
        w.put(agi, 1);
        if agi != 0 {
            w.put(g.range(0, 63) as u32, 6);
            w.put(g.range(0, 31) as u32, 5);
        }
        let nm = if g.range(0, 20) == 0 { g.range(1, 40) } else { g.range(1, 4) } as u32;
        w.put(nm - 1, 7);
        let conj = g.range(0, 1) as u32;
        w.put(conj, 1);
        if conj != 0 {
            let s = if g.range(0, 10) == 0 { g.range(0, 127) as u32 } else { next_id };
            w.put(s, 7);
            next_id = s + nm;
        } else {
            for _ in 0..nm {
                let id = if g.range(0, 10) == 0 {
                    g.range(0, (max_ids as i32 + 3).min(127)) as u32
                } else {
                    next_id
                };
                w.put(id, 7);
                next_id += 1;
            }
        }
    }
    let nsg = if g.range(0, 20) == 0 { g.range(0, 31) } else { g.range(0, 3) } as u32;
    w.put(nsg, 5);
    let mut sg_ids = Vec::new();
    for _ in 0..nsg {
        let id = g.range(0, 31) as u32;
        sg_ids.push(id);
        w.put(id, 5);
        let a = g.range(0, 1) as u32;
        w.put(a, 1);
        if a != 0 {
            w.put(g.range(0, 1) as u32, 1);
        }
        let nm = ({
            let hi = if g.range(0, 20) == 0 { 32 } else { 3 };
            g.range(1, hi)
        }) as u32;
        w.put(nm - 1, 5);
        for _ in 0..nm {
            let id = if !group_ids.is_empty() && g.range(0, 15) != 0 {
                pick(g, &group_ids)
            } else {
                g.range(0, 127) as u32
            };
            w.put(id, 7);
        }
        let d = if !group_ids.is_empty() { pick(g, &group_ids) } else { 0 };
        w.put(d, 7);
    }
    let ngp = if g.range(0, 20) == 0 { g.range(0, 31) } else { g.range(0, 3) } as u32;
    w.put(ngp, 5);
    let mut gp_ids = Vec::new();
    let mut gp_conds = Vec::new();
    let cond = |w: &mut BitWriter, g: &mut Rng, ref_bits: u32| {
        w.put(g.next_u32() & ((1 << ref_bits) - 1), ref_bits);
        let on = g.range(0, 1) as u32;
        w.put(on, 1);
        if on != 0 {
            w.put(g.range(0, 1) as u32, 1);
            let gf = g.range(0, 1) as u32;
            w.put(gf, 1);
            if gf != 0 {
                w.put(g.next_u32() & 0xff, 8);
            }
            w.put(g.range(0, 1) as u32, 1);
            let pf = g.range(0, 1) as u32;
            w.put(pf, 1);
            if pf != 0 {
                w.put(g.next_u32() & 0xff, 8);
                w.put(g.next_u32() & 0x3f, 6);
                w.put(g.next_u32() & 0xf, 4);
            }
        }
    };
    for _ in 0..ngp {
        let id = g.range(0, 31) as u32;
        gp_ids.push(id);
        w.put(id, 5);
        w.put(g.next_u32() & 0x1f, 5);
        let nc = g.range(1, 16) as u32;
        gp_conds.push(nc);
        w.put(nc - 1, 4);
        for _ in 0..nc {
            cond(&mut w, g, 7);
        }
    }
    // mae_Data()
    let nds = g.range(0, 15) as u32;
    w.put(nds, 4);
    for _ in 0..nds {
        let dt = if g.range(0, 15) == 0 { g.range(0, 15) } else { g.range(0, 9) } as u32;
        let mut p = BitWriter::default();
        match dt {
            0 | 1 | 5 => {
                let nb = g.range(1, 4) as u32;
                p.put(nb - 1, 7);
                for _ in 0..nb {
                    let (ids, bits) = match dt {
                        0 => (&group_ids, 7),
                        1 => (&sg_ids, 5),
                        _ => (&gp_ids, 5),
                    };
                    let id = if !ids.is_empty() && g.range(0, 10) != 0 {
                        pick(g, ids)
                    } else {
                        g.range(0, 31) as u32
                    };
                    p.put(id, bits);
                    let nl = ({
                        let hi = if g.range(0, 10) == 0 { 16 } else { 3 };
                        g.range(1, hi)
                    }) as u32;
                    p.put(nl - 1, 4);
                    for _ in 0..nl {
                        let lang: &[u8; 3] = pick(
                            g,
                            &[
                                b"eng",
                                b"ENG",
                                b"deu",
                                b"ger",
                                b"kor",
                                b"ko\0",
                                b"fra",
                                b"x1!",
                                b"zho",
                                b"\xc3\xa9a",
                            ],
                        );
                        p.put_bytes(lang);
                        let dl = if g.range(0, 4) == 0 { g.range(50, 255) } else { g.range(0, 20) }
                            as u32;
                        p.put(dl, 8);
                        for _ in 0..=dl {
                            let c = if g.range(0, 5) == 0 {
                                0xC0 | (g.next_u32() & 0x3f)
                            } else {
                                0x41 + g.range(0, 25) as u32
                            };
                            p.put(c, 8);
                        }
                    }
                }
            }
            2 => {
                let nb = g.range(1, 4) as u32;
                p.put(nb - 1, 7);
                for _ in 0..nb {
                    let id = if !group_ids.is_empty() && g.range(0, 10) != 0 {
                        pick(g, &group_ids)
                    } else {
                        g.range(0, 127) as u32
                    };
                    p.put(id, 7);
                    p.put(g.next_u32() & 0xf, 4);
                    let hl = g.range(0, 1) as u32;
                    p.put(hl, 1);
                    if hl != 0 {
                        p.put_bytes(pick(g, &[b"spa", b"deu", b"ABC", b"1ab"]));
                    }
                }
            }
            3 => {
                let n = ({
                    let hi = if g.range(0, 5) == 0 { 40 } else { 5 };
                    g.range(1, hi)
                }) as u32;
                p.put(n - 1, 7);
                for _ in 0..n {
                    p.put(g.next_u32() & 0x7f, 7);
                    p.put(g.next_u32() & 0x7f, 7);
                }
            }
            4 => {
                let h = g.range(0, 1) as u32;
                p.put(h, 1);
                if h != 0 {
                    for _ in 0..3 {
                        p.put(g.next_u32() & 0x1ff, 9);
                    }
                }
            }
            6 => {
                let v = if g.range(0, 5) == 0 { g.range(1, 3) } else { 0 } as u32;
                p.put(v, 2);
                if v == 0 {
                    let n = g.range(1, 8) as u32;
                    p.put(n - 1, 3);
                    for _ in 0..n {
                        p.put(g.next_u32() & 0x3f, 6);
                        p.put(g.next_u32() & 0xffff, 16);
                    }
                }
            }
            7 => {
                let o = g.range(0, 1) as u32;
                p.put(o, 1);
                if o != 0 {
                    p.put(g.next_u32() & 0x3ff, 10);
                    p.put(g.next_u32() & 0x3ff, 10);
                }
                let n = g.range(0, 3) as u32;
                p.put(n, 5);
                for _ in 0..n {
                    let id = if !gp_ids.is_empty() && g.range(0, 10) != 0 {
                        pick(g, &gp_ids)
                    } else {
                        g.range(0, 31) as u32
                    };
                    p.put(id, 5);
                    let hn = g.range(0, 1) as u32;
                    p.put(hn, 1);
                    if hn != 0 {
                        let c = g.range(0, 1) as u32;
                        p.put(c, 1);
                        if c != 0 {
                            p.put(g.next_u32() & 0x1ff, 9);
                        } else {
                            p.put(g.next_u32() & 0x3ff, 10);
                            p.put(g.next_u32() & 0x3ff, 10);
                        }
                        p.put(g.next_u32() & 0x1ff, 9);
                        p.put(g.next_u32() & 0x1ff, 9);
                    }
                }
            }
            8 => {
                for gpi in 0..ngp as usize {
                    let hs = g.range(0, 1) as u32;
                    p.put(hs, 1);
                    if hs != 0 {
                        for _ in 0..gp_conds[gpi] {
                            p.put(g.range(0, 1) as u32, 1);
                        }
                    }
                    let hd = g.range(0, 1) as u32;
                    p.put(hd, 1);
                    if hd != 0 {
                        let n = g.range(0, 3) as u32;
                        p.put(n, 5);
                        for _ in 0..n {
                            p.put(g.range(0, 3) as u32, 7); // downmixId (activeDmxId is 0..3)
                            let nc = g.range(1, 16) as u32;
                            p.put(nc - 1, 4);
                            for _ in 0..nc {
                                let sw = g.range(0, 1) as u32;
                                p.put(sw, 1);
                                cond(&mut p, g, if sw != 0 { 5 } else { 7 });
                            }
                        }
                    }
                }
            }
            9 => {
                for _ in 0..3 {
                    let pr = g.range(0, 1) as u32;
                    p.put(pr, 1);
                    if pr != 0 {
                        for _ in 0..ng {
                            p.put(g.next_u32() & 0xff, 8);
                        }
                    }
                }
                for _ in 0..(ngp as usize + 2) {
                    p.put(g.next_u32() & 0xff, 8);
                }
            }
            _ => {
                for _ in 0..g.range(0, 5) {
                    p.put(g.next_u32() & 0xff, 8);
                }
            }
        }
        w.put(dt, 4);
        let mut len = p.byte_len() as u32;
        if g.range(0, 20) == 0 {
            len = len.saturating_sub(1);
        }
        w.put(len, 16);
        for i in 0..(len as usize * 8) {
            let b = if i < p.nbits { (p.bytes[i / 8] >> (7 - i % 8)) & 1 } else { 0 };
            w.put(b as u32, 1);
        }
    }
    w.put(0, 7); // metaDataElementIDmaxAvail
    let n = w.nbits;
    // overwrite the last 7 bits with a plausible maxAvail
    let m = if g.range(0, 5) == 0 { g.range(0, 127) } else { next_id.max(1) as i32 - 1 } as u32;
    let mut w2 = BitWriter::default();
    for i in 0..n - 7 {
        w2.put(((w.bytes[i / 8] >> (7 - i % 8)) & 1) as u32, 1);
    }
    w2.put(m & 0x7f, 7);
    w2
}

/// Generates an mpegh3daConfig().
pub fn gen_config(g: &mut Rng) -> Vec<u8> {
    gen_config_with(g, None)
}

/// Generates an mpegh3daConfig(); `loudness` overrides the bytes of a loudnessInfoSet config
/// extension (the random generator is advanced identically).
pub fn gen_config_with(g: &mut Rng, loudness: Option<u8>) -> Vec<u8> {
    let mut w = BitWriter::default();
    w.put(pick(g, &[0x0bu32, 0x0c, 0x0d, 0x03]), 8);
    let sf =
        if g.range(0, 20) == 0 { g.range(0, 31) as u32 } else { pick(g, &[3u32, 4, 5, 6, 7, 8]) };
    w.put(sf, 5);
    if sf == 31 {
        w.put(pick(g, &[29400u32, 14700, 48000, 12345]), 24);
    }
    w.put(if g.range(0, 30) == 0 { g.range(0, 7) as u32 } else { 1 }, 3);
    w.put(g.range(0, 1) as u32, 1);
    w.put(g.range(0, 1) as u32, 1);
    let ref_n = gen_speaker_config(&mut w, g, None).unwrap_or(2);

    // Signals3d()
    let ngroups = if g.range(0, 15) == 0 { g.range(1, 32) } else { g.range(1, 3) } as usize;
    w.put(ngroups as u32 - 1, 5);
    let mut groups = Vec::new(); // (type, count)
    let mut ty_prev = 0;
    let mut total = 0usize;
    for _ in 0..ngroups {
        let ty = if g.range(0, 20) == 0 {
            g.range(0, 7) as u32
        } else {
            (ty_prev + (g.range(0, 3) == 0) as u32).min(1)
        };
        let ty = if g.range(0, 30) == 0 { ty_prev.saturating_sub(1) } else { ty.max(ty_prev) };
        ty_prev = ty;
        w.put(ty, 3);
        let mut count = if g.range(0, 15) == 0 { g.range(1, 30) } else { g.range(1, 6) } as usize;
        let mut differs = true;
        if ty == 0 && g.range(0, 2) != 0 && ref_n <= 28 {
            count = ref_n;
            differs = false;
        }
        w.put_escaped(count as u32 - 1, 5, 8, 16);
        if ty == 0 {
            w.put(differs as u32, 1);
            if differs {
                gen_speaker_config(&mut w, g, Some(count));
            }
        }
        groups.push((ty, count));
        total += count;
    }

    // mpegh3daDecoderConfig()
    let num_obj_groups = groups.iter().filter(|x| x.0 == 1).count();
    let num_ch_groups = groups.iter().filter(|x| x.0 == 0).count();
    let sum_minus_1 = (total as i32 - 1).max(1);
    let nbits = 15 - fixnormz_s(sum_minus_1 as i16) + 1;
    let mut el = BitWriter::default();
    let mut num_el = 0u32;
    if g.range(0, 6) == 0 {
        el.put(3, 2);
        gen_ext_element(
            &mut el,
            g,
            ID_EXT_ELE_PROD_METADATA as u32,
            1,
            num_obj_groups,
            num_ch_groups,
        );
        num_el += 1;
    }
    for &(ty, count) in &groups {
        // extension elements first
        let mut exts = Vec::new();
        if ty == 1 {
            exts.push(ID_EXT_ELE_OBJ_METADATA as u32);
        }
        if g.range(0, 4) == 0 {
            exts.push(ID_EXT_ELE_MCT as u32);
        }
        for _ in 0..g.range(0, 2) {
            exts.push(pick(g, &[0u32, 3, 4, 4, 8, 7, 1, 0x81, 200]));
        }
        for e in exts {
            el.put(3, 2);
            gen_ext_element(&mut el, g, e, count.min(28), num_obj_groups, num_ch_groups);
            num_el += 1;
        }
        let mut left = count as i32;
        while left > 0 {
            let t = if left >= 2 && g.range(0, 2) != 0 {
                1
            } else if ty == 0 && g.range(0, 5) == 0 {
                2
            } else {
                0
            };
            el.put(t, 2);
            match t {
                0 => gen_core_config(&mut el, g, false, nbits),
                1 => gen_core_config(&mut el, g, true, nbits),
                _ => {}
            }
            left -= if t == 1 { 2 } else { 1 };
            num_el += 1;
        }
    }
    if g.range(0, 5) == 0 {
        el.put(3, 2);
        let t = pick(g, &[0u32, 4, 3]);
        gen_ext_element(&mut el, g, t, 1, num_obj_groups, num_ch_groups);
        num_el += 1;
    }
    w.put_escaped(num_el - 1, 4, 8, 16);
    w.put(g.range(0, 1) as u32, 1); // elementLengthPresent
    w.put_writer(&el);

    // mpegh3daConfigExtension()
    let ext = g.range(0, 3) != 0;
    w.put(ext as u32, 1);
    if ext {
        let n = g.range(1, 4) as u32;
        w.put_escaped(n - 1, 2, 4, 8);
        for _ in 0..n {
            let t = pick(g, &[0u32, 1, 2, 2, 3, 3, 4, 6, 7, 9]);
            w.put_escaped(t, 4, 8, 16);
            let mut p = BitWriter::default();
            match t {
                0 => {
                    for _ in 0..g.range(0, 4) {
                        p.put(if g.range(0, 20) == 0 { 0x5a } else { 0xa5 }, 8);
                    }
                }
                1 => {
                    let dt = g.range(0, 3) as u32;
                    p.put(dt, 2);
                    if dt == 0 || dt == 2 {
                        let pd = g.range(0, 1) as u32;
                        p.put(pd, 1);
                        if pd == 0 {
                            p.put(g.range(0, 7) as u32, 3);
                        }
                        p.put(g.range(0, 1) as u32, 1);
                    }
                    for _ in 0..g.range(0, 3) {
                        p.put(g.next_u32() & 0xff, 8);
                    }
                }
                2 => {
                    for _ in 0..g.range(1, 12) {
                        let v = g.next_u32() & 0xff;
                        p.put(loudness.map_or(v, |x| x as u32), 8);
                    }
                }
                3 => {
                    let main = g.range(0, 5) != 0;
                    let a = gen_asi(g, main, total + 3);
                    p.put_writer(&a);
                }
                6 => {
                    for _ in 0..ngroups {
                        p.put(g.next_u32() & 0xf, 4);
                    }
                }
                _ => {
                    for _ in 0..g.range(0, 5) {
                        p.put(g.next_u32() & 0xff, 8);
                    }
                }
            }
            put_sized_payload(&mut w, g, &p, 4, 8, 16);
        }
    }
    w.bytes
}

/// Bit error / truncation mutations of a byte string.
pub fn mutate(g: &mut Rng, d: &[u8]) -> Vec<u8> {
    let mut v = d.to_vec();
    if v.is_empty() {
        return v;
    }
    match g.range(0, 5) {
        0 | 1 => {
            for _ in 0..g.range(1, 4) {
                let i = g.range(0, v.len() as i32 * 8 - 1) as usize;
                v[i / 8] ^= 0x80 >> (i % 8);
            }
        }
        2 => {
            let n = g.range(0, v.len() as i32 - 1) as usize;
            v.truncate(n);
        }
        3 => {
            let i = g.range(0, v.len() as i32 - 1) as usize;
            v[i] = g.next_u32() as u8;
        }
        _ => {
            let i = g.range(0, v.len() as i32) as usize;
            v.insert(i, g.next_u32() as u8);
        }
    }
    v
}

pub fn put_packet(out: &mut Vec<u8>, ptype: u32, label: u32, payload: &[u8]) {
    let mut w = BitWriter::default();
    w.put_escaped(ptype, 3, 8, 8);
    w.put_escaped(label, 2, 8, 32);
    w.put_escaped(payload.len() as u32, 11, 24, 24);
    while w.nbits % 8 != 0 {
        w.put(0, 1);
    }
    out.extend_from_slice(&w.bytes);
    out.extend_from_slice(payload);
}

/// CRC16 of a payload as computed by the MHAS parser (poly 0x8021, start 0xffff).
pub fn crc16(payload: &[u8]) -> u32 {
    let size = (payload.len() + 8).next_power_of_two();
    let mut buf = vec![0u8; size];
    buf[..payload.len()].copy_from_slice(payload);
    let mut bs = FdkBitStream::default();
    bs.init(&BitMem::from_slice(&buf), size as u32, (payload.len() * 8) as u32, FdkBsCfg::BsReader);
    let mut crc = FdkCrcInfo::default();
    crc.init(0x8021, 0xffff, 16);
    let reg = crc.start_reg(&mut bs, (payload.len() * 8) as i32);
    bs.push_for((payload.len() * 8) as u32);
    crc.end_reg(&mut bs, reg);
    crc.get_crc() as u32
}

fn gen_earcon_info(g: &mut Rng) -> Vec<u8> {
    let mut w = BitWriter::default();
    let n = g.range(1, 3) as u32;
    w.put(n - 1, 7);
    for _ in 0..n {
        w.put(g.next_u32() & 0x1fff, 13);
        let pt = g.range(0, 3) as u32;
        w.put(pt, 2);
        if pt == 0 {
            w.put(3, 7);
        } else if pt == 1 {
            w.put(g.next_u32() & 0x7fffff, 23);
        }
        let hg = g.range(0, 1) as u32;
        w.put(hg, 1);
        if hg != 0 {
            w.put(9, 7);
        }
        let ht = g.range(0, 1) as u32;
        w.put(ht, 1);
        if ht != 0 {
            let nl = g.range(0, 2) as u32;
            w.put(nl, 4);
            for _ in 0..nl {
                w.put(0x656e67, 24);
                let l = g.range(0, 5) as u32;
                w.put(l, 8);
                for _ in 0..l {
                    w.put(0x41, 8);
                }
            }
        }
    }
    w.bytes
}

fn gen_pcm_config(g: &mut Rng) -> Vec<u8> {
    let mut w = BitWriter::default();
    let n = g.range(1, 3) as u32;
    w.put(n - 1, 7);
    w.put(g.range(0, 1) as u32, 1);
    let sr = if g.range(0, 3) == 0 { 31 } else { 3 };
    w.put(sr, 5);
    if sr == 31 {
        w.put(48000, 24);
    }
    w.put(g.range(0, 7) as u32, 3);
    let fs = g.range(0, 7) as u32;
    w.put(fs, 3);
    if fs == 5 {
        w.put(g.range(0, 2048) as u32, 16);
    }
    for _ in 0..n {
        w.put(g.range(0, 127) as u32, 7);
    }
    w.put(g.next_u32() & 0xff, 8);
    let ag = g.range(0, 3) as u32;
    w.put(ag, 2);
    if ag == 1 {
        w.put(g.next_u32() & 0xff, 8);
    }
    w.bytes
}

/// Generates a synthetic MHAS stream; returns the bytes and the access unit boundaries.
pub fn gen_mhas(
    g: &mut Rng,
    n_aus: usize,
    base_config: Option<&[u8]>,
) -> (Vec<u8>, Vec<(usize, usize)>) {
    let mut out = Vec::new();
    let mut aus = Vec::new();
    let main_label = pick(g, &[1u32, 1, 1, 2, 16]);
    let sub_labels: Vec<u32> = match g.range(0, 3) {
        0 => vec![17],
        1 => vec![17, 33],
        _ => vec![],
    };
    let mut config_seed = g.next_u64();
    let mut config: Vec<u8> = match base_config {
        Some(c) if g.range(0, 1) == 0 => c.to_vec(),
        _ => gen_config(&mut Rng::new(config_seed)),
    };
    let mut sub_configs: Vec<Vec<u8>> = sub_labels.iter().map(|_| gen_config(g)).collect();
    let rap_period = g.range(1, 8) as usize;
    for au in 0..n_aus {
        let start = out.len();
        let rap = au % rap_period == 0;
        if rap {
            if g.range(0, 2) == 0 {
                put_packet(&mut out, 6, 0, &[if g.range(0, 30) == 0 { 0x5a } else { 0xa5 }]);
            }
            match g.range(0, 12) {
                0 => {
                    config_seed = g.next_u64();
                    config = gen_config(&mut Rng::new(config_seed));
                }
                1 => config = mutate(g, &config),
                2..=4 => {
                    // loudness only change (handled by disregardLoudnessChange())
                    let v = g.next_u32() as u8;
                    config = gen_config_with(&mut Rng::new(config_seed), Some(v));
                }
                _ => {}
            }
            if g.range(0, 10) == 0 {
                let c = crc16(&config);
                put_packet(&mut out, 9, 0, &[(c >> 8) as u8, c as u8 ^ (g.range(0, 5) == 0) as u8]);
            }
            put_packet(
                &mut out,
                1,
                if g.range(0, 40) == 0 { g.range(0, 40) as u32 } else { main_label },
                &config,
            );
            for (i, &l) in sub_labels.iter().enumerate() {
                if g.range(0, 8) == 0 {
                    sub_configs[i] = gen_config(g);
                }
                if g.range(0, 6) != 0 {
                    put_packet(&mut out, 1, l, &sub_configs[i]);
                }
            }
            if g.range(0, 3) != 0 {
                let a = gen_asi(g, true, 56);
                put_packet(&mut out, 3, main_label, &a.bytes);
            }
            for &l in &sub_labels {
                if g.range(0, 3) == 0 {
                    let a = gen_asi(g, false, 56);
                    put_packet(&mut out, 3, l, &a.bytes);
                }
            }
            if g.range(0, 2) == 0 {
                let mut w = BitWriter::default();
                let f = g.range(0, 1) as u32;
                w.put(f, 1);
                if f != 0 {
                    w.put_escaped(g.range(0, 100000) as u32, 15, 24, 32);
                }
                put_packet(&mut out, 14, main_label, &w.bytes);
            }
        }
        for _ in 0..g.range(0, 3) {
            let (t, l, p): (u32, u32, Vec<u8>) = match g.range(0, 15) {
                0 => (22, main_label, (0..g.range(1, 20)).map(|_| g.next_u32() as u8).collect()),
                1 => (12, main_label, (0..g.range(1, 10)).map(|_| g.next_u32() as u8).collect()),
                2 => (
                    13,
                    pick(g, &[main_label, 5, 0]),
                    (0..g.range(1, 10)).map(|_| g.next_u32() as u8).collect(),
                ),
                3 => {
                    let mut p = vec![0xE0, 0x00];
                    p.extend((0..g.range(0, 8)).map(|_| g.next_u32() as u8));
                    if g.range(0, 5) == 0 {
                        p[0] = 0x12;
                    }
                    (8, pick(g, &[main_label, 3]), p)
                }
                4 => {
                    let mut w = BitWriter::default();
                    w.put((g.range(0, 5) != 0) as u32, 1);
                    w.put(0, 1);
                    w.put(g.range(0, 1) as u32, 1);
                    w.put(g.range(0, 1200) as u32, 13);
                    (17, pick(g, &[main_label, 0, 2049]), w.bytes)
                }
                5 => (19, pick(g, &[2049, 0]), gen_earcon_info(g)),
                6 => (20, pick(g, &[2049, 0]), gen_pcm_config(g)),
                7 => (
                    21,
                    pick(g, &[2049, 0]),
                    (0..g.range(1, 30)).map(|_| g.next_u32() as u8).collect(),
                ),
                8 => (0, main_label, vec![0; g.range(0, 5) as usize]),
                9 => {
                    let t = g.range(0, 40) as u32;
                    (
                        t,
                        g.range(0, 20) as u32,
                        (0..g.range(0, 5)).map(|_| g.next_u32() as u8).collect(),
                    )
                }
                10 => {
                    let p: Vec<u8> = (0..g.range(1, 10)).map(|_| g.next_u32() as u8).collect();
                    let c = crc16(&p);
                    put_packet(&mut out, 9, 0, &[(c >> 8) as u8, c as u8]);
                    (22, main_label, p)
                }
                11 => (10, 0, vec![1, 2, 3, 4]),
                _ => (6, 0, vec![0xa5]),
            };
            put_packet(&mut out, t, l, &p);
        }
        let fl = g.range(4, 300) as usize;
        let frame: Vec<u8> = (0..fl).map(|_| g.next_u32() as u8).collect();
        put_packet(&mut out, 2, main_label, &frame);
        for &l in &sub_labels {
            if g.range(0, 6) != 0 {
                let f: Vec<u8> = (0..g.range(4, 100)).map(|_| g.next_u32() as u8).collect();
                put_packet(&mut out, 2, l, &f);
            }
        }
        aus.push((start, out.len() - start));
    }
    (out, aus)
}

// ---- drivers ----

pub struct Params {
    /// Decode like the core decoder: consume complete access units, no random operations.
    pub clean: bool,
    pub layers: u32,
    pub reg: u32,
    pub fail: (u32, u32, u32),
    pub params: Vec<(i32, i32)>,
}

pub fn random_params(g: &mut Rng) -> Params {
    let reg = if g.range(0, 3) == 0 { g.next_u32() & REG_ALL } else { REG_ALL };
    let fail =
        if g.range(0, 3) == 0 { (g.next_u32(), g.range(2, 40) as u32, 1) } else { (0, 0, 0) };
    let mut params = Vec::new();
    for _ in 0..g.range(0, 3) {
        params.push(match g.range(0, 7) {
            0 => (TPDEC_PARAM_MINIMIZE_DELAY, 1),
            1 => (TPDEC_PARAM_EARLY_CONFIG, 1),
            2 => (TPDEC_PARAM_IGNORE_BUFFERFULLNESS, 1),
            3 => (TPDEC_PARAM_CHECK_TWO_SYNCS, 1),
            4 => (TPDEC_PARAM_USE_ELEM_SKIPPING, 1),
            5 => (TPDEC_PARAM_TARGETLAYOUT, g.range(-1, 20)),
            6 => (TPDEC_PARAM_SET_BITRATE, g.range(0, 1_000_000)),
            _ => (TPDEC_PARAM_BURST_PERIOD, g.range(0, 1000)),
        });
    }
    Params { clean: false, layers: pick(g, &[1u32, 1, 1, 2, 4]), reg, fail, params }
}

fn consume_au(p: &mut Pair, g: &mut Rng, clean: bool) {
    let (rem, _total) = p.au_bits(0);
    match if clean { 9 } else { g.range(0, 9) } {
        0 => {}
        1 => p.consume(0, g.range(0, 64)),
        2 => p.push(0, g.range(-64, 64)),
        3 => p.consume(0, rem.clamp(0, 2000) + g.range(0, 40)),
        _ => {
            if rem > 0 {
                if rem <= 4096 {
                    p.consume(0, rem);
                } else {
                    p.push(0, rem);
                }
            }
        }
    }
}

fn random_op(p: &mut Pair, g: &mut Rng) {
    match g.range(0, 200) {
        0 => p.set_param(TPDEC_PARAM_RESET, 1),
        1 => p.set_param(TPDEC_PARAM_FORCE_CONFIG_CHANGE, 0),
        2 => p.set_param(TPDEC_PARAM_EARLY_CONFIG, g.range(0, 1)),
        3 => p.set_param(TPDEC_PARAM_MINIMIZE_DELAY, g.range(0, 1)),
        4 => p.set_param(TPDEC_PARAM_USE_ELEM_SKIPPING, g.range(0, 1)),
        5 => p.set_param(TPDEC_PARAM_CHECK_TWO_SYNCS, g.range(0, 1)),
        6 => p.set_param(TPDEC_PARAM_TARGETLAYOUT, g.range(0, 20)),
        7 => {
            let v = g.range(0, 5) as u8;
            p.rec.asi.active_dmx_id = v;
            unsafe { cref_tp_asiSetActiveDmxId(p.c, v as i32) };
        }
        _ => {}
    }
}

/// Streaming MHAS (TT_MHAS): feed random chunks, read and consume access units.
pub fn run_mhas_stream(data: &[u8], seed: u64, prm: &Params, name: &str) -> (u64, u64) {
    let mut g = Rng::new(seed);
    let mut p = Pair::new(TT_MHAS, prm.layers, prm.reg, prm.fail, name);
    for &(k, v) in &prm.params {
        p.set_param(k, v);
    }
    let mut pos = 0usize;
    let mut ok = 0u64;
    let mut idle = 0;
    let max_chunk = pick(&mut g, &[200, 3000, 30000, 70000]);
    loop {
        if pos < data.len() && (prm.clean || g.range(0, 2) != 0) {
            let n = (data.len() - pos).min(g.range(1, max_chunk) as usize);
            let (_, left) = p.fill(&data[pos..pos + n], 0);
            pos += n - left as usize;
        }
        let err = p.read_au(0);
        if err == TRANSPORTDEC_OK {
            ok += 1;
            idle = 0;
            consume_au(&mut p, &mut g, prm.clean);
            if prm.clean || g.range(0, 15) != 0 {
                p.end_au();
            }
        } else if pos >= data.len() {
            idle += 1;
            if idle > 50 {
                break;
            }
        }
        if !prm.clean {
            random_op(&mut p, &mut g);
        }
    }
    (ok, p.steps)
}

/// Packetized MHAS (TT_MHAS_PACKETIZED): one access unit per fill.
pub fn run_mhas_packetized(
    data: &[u8],
    aus: &[(usize, usize)],
    seed: u64,
    prm: &Params,
    name: &str,
) -> (u64, u64) {
    let mut g = Rng::new(seed);
    let mut p = Pair::new(TT_MHAS_PACKETIZED, prm.layers, prm.reg, prm.fail, name);
    for &(k, v) in &prm.params {
        p.set_param(k, v);
    }
    let mut ok = 0u64;
    let mut i = 0;
    let mut tries = 0;
    while i < aus.len() {
        let (o, n) = aus[i];
        let (_, left) = p.fill(&data[o..(o + n).min(data.len())], 0);
        if left == 0 || tries > 3 {
            i += 1;
            tries = 0;
        } else {
            tries += 1;
        }
        let err = p.read_au(0);
        if err == TRANSPORTDEC_OK {
            ok += 1;
            consume_au(&mut p, &mut g, prm.clean);
            if prm.clean || g.range(0, 15) != 0 {
                p.end_au();
            }
        }
        if !prm.clean {
            random_op(&mut p, &mut g);
        }
    }
    (ok, p.steps)
}

/// Raw MPEG-H (TT_MHA_RAW): out-of-band config, raw frames and in-band configs.
pub fn run_mha_raw(packets: &[MhasPacket], seed: u64, prm: &Params, name: &str) -> (u64, u64) {
    let mut g = Rng::new(seed);
    let mut p = Pair::new(TT_MHA_RAW, prm.layers, prm.reg, prm.fail, name);
    for &(k, v) in &prm.params {
        p.set_param(k, v);
    }
    let configs: Vec<&MhasPacket> = packets.iter().filter(|x| x.ptype == 1).collect();
    let mut ok = 0u64;
    let mut configured = false;
    for pk in packets {
        if pk.ptype == 1 {
            if !configured || (!prm.clean && g.range(0, 8) == 0) {
                let c = if !prm.clean && g.range(0, 4) == 0 {
                    mutate(&mut g, &pk.payload)
                } else {
                    pk.payload.clone()
                };
                p.oob_config(&c, 0);
                configured = true;
            } else if g.range(0, 2) == 0 {
                let c = match if prm.clean { 4 } else { g.range(0, 5) } {
                    0 => mutate(&mut g, &pk.payload),
                    1 if !configs.is_empty() => gen_config(&mut g),
                    _ => pk.payload.clone(),
                };
                let bus = if prm.clean { 0 } else { pick(&mut g, &[0u8, 0, 1, 4]) };
                p.in_band_config(&c, bus, 0);
            }
            continue;
        }
        if pk.ptype != 2 {
            continue;
        }
        let f = if !prm.clean && g.range(0, 20) == 0 {
            mutate(&mut g, &pk.payload)
        } else {
            pk.payload.clone()
        };
        p.fill(&f, 0);
        let err = p.read_au(0);
        if err == TRANSPORTDEC_OK {
            ok += 1;
            consume_au(&mut p, &mut g, prm.clean);
            if prm.clean || g.range(0, 10) != 0 {
                p.end_au();
            }
        }
        if !prm.clean {
            random_op(&mut p, &mut g);
        }
    }
    (ok, p.steps)
}
