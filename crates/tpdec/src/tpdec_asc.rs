// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpegTPDec/src/tpdec_asc.cpp.

//! MPEG-H 3D audio config parser (`mpegh3daConfig()`, ISO/IEC 23008-3 5.2.2), program config
//! helpers and earcon config parsers.

use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::*;
use mpeghdec_fdk::common_fix::fixnormz_s;

use crate::tp_data::*;
use crate::tpdec_asi::{asi_reset, mae_audio_scene_info};
use crate::tpdec_lib::*;

const MAX_CICP_VALUE: i32 = 21;

/// `CProgramConfig_Reset(pPce)`
pub fn cprogram_config_reset(pce: &mut CProgramConfig) {
    pce.el_counter = 0;
}

/// `CProgramConfig_Init(pPce)`
pub fn cprogram_config_init(pce: &mut CProgramConfig) {
    *pce = CProgramConfig::default();
}

/// `CProgramConfig_IsValid(pPce)`
pub fn cprogram_config_is_valid(pce: &CProgramConfig) -> i32 {
    (pce.is_valid != 0) as i32
}

/// `CProgramConfig_LookupElement(pPce, chConfig, tag, channelIdx, chMapping, chType, chIndex,
/// chDescrLen, elMapping, elList, elType)`: implicit channel mapping; stores all channel element
/// IDs. Always returns 1.
pub fn cprogram_config_lookup_element(
    pce: &mut CProgramConfig,
    _channel_config: u32,
    _tag: u32,
    _channel_idx: u32,
    _ch_mapping: &mut [u8],
    _ch_type: &mut [AudioChannelType],
    _ch_index: &mut [u8],
    _ch_descr_len: u32,
    el_mapping: &mut u8,
    el_list: &mut [Mp4ElementId],
    el_type: Mp4ElementId,
) -> i32 {
    // Implicit channel mapping.
    if is_mp4_channel_element(el_type) {
        // Store all channel element IDs
        el_list[pce.el_counter as usize] = el_type;
        *el_mapping = pce.el_counter;
        pce.el_counter = pce.el_counter.wrapping_add(1);
    }

    1
}

fn get_sample_rate(bs: &mut FdkBitStream, index: &mut u8, n_bits: u32) -> i32 {
    let idx = bs.read_bits(n_bits) as i32;
    let sample_rate = if idx == (1 << n_bits) - 1 {
        if bs.get_valid_bits() < 24 {
            return 0;
        }
        bs.read_bits(24) as i32
    } else {
        SAMPLING_RATE_TABLE[idx as usize] as i32
    };

    *index = idx as u8;

    sample_rate
}

/// Subroutine for parsing extension element configuration:
/// UsacExtElementConfig() q.v. ISO/IEC FDIS 23003-3:2011(E) Table 14,
/// mpegh3daExtElementConfig() q.v. ISO/IEC DIS 23008-3 Table 13.
fn ext_element_config(
    ext_element: &mut CSUsacExtElementConfig,
    hbs: &mut FdkBitStream,
    cb: &mut dyn TpCallbacks,
    num_signals_in_group: u8,
    num_object_groups: u8,
    num_channel_groups: u8,
    core_frame_length: u32,
    sub_stream_index: i32,
    aot: AudioObjectType,
) -> TransportDecError {
    let mut error_status = TRANSPORTDEC_OK;

    let mut usac_ext_element_type = hbs.escaped_value(4, 8, 16);

    let usac_ext_element_config_length = hbs.escaped_value(4, 8, 16) as i32;

    if hbs.read_bit() != 0 {
        // usacExtElementDefaultLengthPresent
        ext_element.usac_ext_element_default_length =
            hbs.escaped_value(8, 16, 0).wrapping_add(1) as u16;
    } else {
        ext_element.usac_ext_element_default_length = 0;
    }

    ext_element.usac_ext_element_payload_frag = hbs.read_bit() as u8;

    let bs_anchor = hbs.get_valid_bits();

    // Return an error in case the bitbuffer fill level is too low.
    if bs_anchor < usac_ext_element_config_length * 8 {
        return TRANSPORTDEC_NOT_ENOUGH_BITS;
    }

    match usac_ext_element_type as i32 {
        ID_EXT_ELE_UNKNOWN | ID_EXT_ELE_FILL => {}
        ID_EXT_ELE_AUDIOPREROLL => {
            // No configuration element
            ext_element.usac_ext_element_has_audio_pre_roll = 1;
        }
        ID_EXT_ELE_FMT_CNVRTR => {
            // No configuration element
        }
        ID_EXT_ELE_OBJ_METADATA => {
            // parsing of ObjectMetadataConfig()
            let oam = &mut ext_element.ext_config.oam;

            // save number of objects
            oam.num_object_signals = num_signals_in_group;
            oam.low_delay_metadata_coding = hbs.read_bit() as u8;
            if oam.low_delay_metadata_coding != 1 {
                return TRANSPORTDEC_UNSUPPORTED_FORMAT; // only low delay supported
            }
            if hbs.read_bit() == 0 {
                // does not have the frame length of the corecoder (!hasCoreLength)
                oam.oam_frame_length = (hbs.read_bits(6) + 1) as u16; // framelength = blocksize/64
                oam.oam_frame_length <<= 6; // blocksize
                if oam.oam_frame_length as u32 != core_frame_length
                    && oam.oam_frame_length as u32 != (core_frame_length >> 1)
                    && oam.oam_frame_length as u32 != (core_frame_length >> 2)
                {
                    return TRANSPORTDEC_PARSE_ERROR;
                }
            } else {
                oam.oam_frame_length = core_frame_length as u16;
            }
            if oam.oam_frame_length as u32 > core_frame_length {
                return TRANSPORTDEC_UNSUPPORTED_FORMAT;
            }
            if hbs.read_bit() != 0 {
                // hasScreenRelativeObjects
                let _is_screen_relative_object = hbs.read_bits(num_signals_in_group as u32);
            }
            oam.has_dynamic_object_priority = hbs.read_bit() as u8;
            oam.has_uniform_spread = hbs.read_bit() as u8;
        }
        ID_EXT_ELE_MCT => {
            // MCTConfig()
            if num_signals_in_group > 0 {
                ext_element.ext_config.mct.mct_chan_mask = hbs
                    .read_bits(num_signals_in_group as u32)
                    .wrapping_shl(32 - num_signals_in_group as u32);
            } else {
                ext_element.ext_config.mct.mct_chan_mask = 0;
            }
        }
        ID_EXT_ELE_HOA => {
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        ID_EXT_ELE_UNI_DRC => {
            if cb.has_uni_drc() {
                error_status = cb.uni_drc(
                    Some(hbs),
                    usac_ext_element_config_length,
                    0, // uniDrcConfig
                    sub_stream_index,
                    0,
                    aot,
                );
                if error_status != TRANSPORTDEC_OK {
                    return error_status;
                }
            }
        }
        ID_EXT_ELE_PROD_METADATA => {
            if usac_ext_element_config_length > TP_MPEGH_MAX_PROD_METADATA_CONFIG_LEN {
                error_status = TRANSPORTDEC_PARSE_ERROR;
            } else {
                let pm = &mut ext_element.ext_config.prod_metadata;

                pm.has_reference_distance = hbs.read_bit() as u8;

                if pm.has_reference_distance != 0 {
                    pm.bs_reference_distance = hbs.read_bits(7) as u8;
                } else {
                    pm.bs_reference_distance = 57;
                }

                if num_object_groups > 0 {
                    pm.has_object_distance = hbs
                        .read_bits(num_object_groups as u32)
                        .wrapping_shl(32 - num_object_groups as u32);
                } else {
                    pm.has_object_distance = 0;
                }

                for _gp in 0..num_channel_groups {
                    let _direct_headphone = hbs.read_bit();
                }
            }
        }
        _ => {
            usac_ext_element_type = ID_EXT_ELE_UNKNOWN as u32;
        }
    }
    ext_element.usac_ext_element_type = usac_ext_element_type as UsacExtElementType;

    // Adjust bit stream position. This is required because of byte alignment and unhandled
    // extensions.
    let left_bits = (usac_ext_element_config_length << 3) - (bs_anchor - hbs.get_valid_bits());
    if left_bits >= 0 {
        hbs.push_for(left_bits as u32);
    } else {
        // parsed too many bits
        error_status = TRANSPORTDEC_PARSE_ERROR;
    }

    error_status
}

/// downmixConfig() subroutine for parsing downmix configuration and downmix matrix parsing.
fn downmix_config(
    usc: &mut CSUsacConfig,
    hbs: &mut FdkBitStream,
    cb: &mut dyn TpCallbacks,
) -> TransportDecError {
    let mut error_status = TRANSPORTDEC_OK;
    usc.downmix_config_type = 0;
    usc.passive_downmix_flag = 0;
    usc.immersive_downmix_flag = 0;

    usc.downmix_config_type = hbs.read_bits(2) as u8;
    if usc.downmix_config_type == 0 || usc.downmix_config_type == 2 {
        usc.passive_downmix_flag = hbs.read_bits(1) as u8;
        if usc.passive_downmix_flag == 0 {
            usc.phase_align_strength = hbs.read_bits(3) as u8;
        }
        usc.immersive_downmix_flag = hbs.read_bits(1) as u8;
        if usc.immersive_downmix_flag != 0 {
            usc.downmix_config_type = 0;
        }
    }
    if (usc.downmix_config_type == 1 || usc.downmix_config_type == 2)
        && cb.parse_dmx_matrix(hbs, usc) != 0
    {
        error_status = TRANSPORTDEC_UNKOWN_ERROR;
    }

    error_status
}

/// Subroutine for parsing the USAC / MPEG-H configuration extension:
/// UsacConfigExtension() q.v. ISO/IEC FDIS 23003-3:2011(E) Table 15,
/// mpegh3daConfigExtension() q.v. ISO/IEC DIS 23008-3 Table 14.
fn config_extension(
    usc: &mut CSUsacConfig,
    aot: AudioObjectType,
    hbs: &mut FdkBitStream,
    cb: &mut dyn TpCallbacks,
) -> TransportDecError {
    let mut error_status = TRANSPORTDEC_OK;

    let mut ext_is_present = 0; // Bit field to detect unallowed multiple extension configs
                                // index of loudnessInfoSet config extension. -1 if not contained.
    let mut loudness_info_set_index = -1;
    let tmp_sub_stream_index = usc.sub_stream_index as i32;
    let tmp_aot = aot;

    let num_config_extensions = hbs.escaped_value(2, 4, 8) as i32 + 1;
    for conf_ext_idx in 0..num_config_extensions {
        let loudness_info_set_config_extension_position = hbs.get_valid_bits();
        let usac_config_ext_type = hbs.escaped_value(4, 8, 16);
        let mut usac_config_ext_length = hbs.escaped_value(4, 8, 16) as i32;

        // Start bit position of config extension
        let nbits = hbs.get_valid_bits();

        // Return an error in case the bitbuffer fill level is too low.
        if nbits < usac_config_ext_length * 8 {
            return TRANSPORTDEC_NOT_ENOUGH_BITS;
        }

        match usac_config_ext_type as i32 {
            ID_CONFIG_EXT_FILL => {
                for _i in 0..usac_config_ext_length {
                    if hbs.read_bits(8) != 0xa5 {
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                }
            }
            ID_CONFIG_EXT_DOWNMIX => {
                if ext_is_present & (1 << ID_CONFIG_EXT_DOWNMIX) != 0 {
                    return TRANSPORTDEC_PARSE_ERROR;
                }
                ext_is_present |= 1 << ID_CONFIG_EXT_DOWNMIX;
                error_status = downmix_config(usc, hbs, cb);
                if error_status != TRANSPORTDEC_OK {
                    return error_status;
                }
            }
            ID_CONFIG_EXT_LOUDNESS_INFO => {
                // mpegh3daLoudnessInfoSet();
                if ext_is_present & (1 << ID_CONFIG_EXT_LOUDNESS_INFO) != 0 {
                    return TRANSPORTDEC_PARSE_ERROR;
                }
                ext_is_present |= 1 << ID_CONFIG_EXT_LOUDNESS_INFO;
                if cb.has_uni_drc() {
                    error_status = cb.uni_drc(
                        Some(hbs),
                        usac_config_ext_length,
                        1, // loudnessInfoSet
                        tmp_sub_stream_index,
                        loudness_info_set_config_extension_position,
                        tmp_aot,
                    );
                    if error_status != TRANSPORTDEC_OK {
                        return error_status;
                    }
                    loudness_info_set_index = conf_ext_idx;
                }
            }
            ID_CONFIG_EXT_AUDIOSCENE_INFO => {
                if ext_is_present & (1 << ID_CONFIG_EXT_AUDIOSCENE_INFO) != 0 {
                    return TRANSPORTDEC_PARSE_ERROR;
                }
                ext_is_present |= 1 << ID_CONFIG_EXT_AUDIOSCENE_INFO;
                if usac_config_ext_length > 0 {
                    if let Some(asi) = cb.asi() {
                        let mut num_max_element_ids =
                            usc.num_audio_channels as i32 + usc.num_audio_objects as i32;
                        for i in 0..usc.bs_num_signal_groups as usize {
                            if usc.m_signal_group_type[i].type_ == 3 {
                                // for HOA one ID per signal group
                                num_max_element_ids += 1;
                            }
                        }

                        asi_reset(asi);

                        if mae_audio_scene_info(
                            asi,
                            hbs,
                            num_max_element_ids,
                            usc.sub_stream_index as i32,
                        ) != 0
                        {
                            return TRANSPORTDEC_PARSE_ERROR;
                        }
                    }
                }
            }
            ID_CONFIG_EXT_HOA_MATRIX => {
                // HoaRenderingMatrixSet();
            }
            ID_CONFIG_EXT_SIG_GROUP_INFO => {
                for _grp in 0..usc.bs_num_signal_groups {
                    hbs.read_bits(3); // groupPriority[grp]
                    hbs.read_bits(1); // fixedPosition[grp]
                }
            }
            _ => {}
        }

        // Skip remaining bits. If too many bits were parsed, assume error.
        usac_config_ext_length = 8 * usac_config_ext_length - (nbits - hbs.get_valid_bits());
        if usac_config_ext_length < 0 {
            return TRANSPORTDEC_PARSE_ERROR;
        }
        hbs.push_for(usac_config_ext_length as u32);
    }

    if loudness_info_set_index == -1 && cb.has_uni_drc() {
        // no loudnessInfoSet contained. Clear the loudnessInfoSet struct by feeding an empty
        // config extension
        error_status = cb.uni_drc(None, 0, 1, tmp_sub_stream_index, 0, tmp_aot);
        if error_status != TRANSPORTDEC_OK {
            return error_status;
        }
    }

    error_status
}

/// This function unifies decoder config parsing of USAC and MPEGH:
/// mpegh3daDecoderConfig() ISO/IEC DIS 23008-3 Table 8,
/// UsacDecoderConfig() ISO/IEC FDIS 23003-3 Table 6.
fn usac_mpegh_decoder_config_parse(
    asc: &mut CSAudioSpecificConfig,
    hbs: &mut FdkBitStream,
    cb: &mut dyn TpCallbacks,
) -> TransportDecError {
    let mut error_status = TRANSPORTDEC_OK;
    let aot = asc.m_aot;
    let samples_per_frame = asc.m_samples_per_frame;
    let channel_configuration = asc.m_channel_configuration;
    let usc = &mut asc.m_sc;
    let mut grp = 0usize; // index of current signal group
    let mut cnt_signals: i32 = 0; // count number of signals according to one group
    let mut mpegh_mct_element: i32 = -1;
    // Last extension element before signal element.
    let mut last_ext_element: UsacExtElementType = ID_EXT_ELE_UNKNOWN;
    let mut uni_drc_element: i32 = -1; // index of uniDrc extension element. -1 if not contained.

    let number_of_elements = hbs.escaped_value(4, 8, 16) as i32 + 1;
    if aot == AOT_MPEGH3DA {
        usc.element_length_present = hbs.read_bit() as u8;
    }
    usc.m_usac_num_elements = number_of_elements as u32;
    if number_of_elements > TP_USAC_MAX_ELEMENTS as i32 {
        return TRANSPORTDEC_UNSUPPORTED_FORMAT;
    }
    usc.m_n_usac_channels = 0;
    usc.m_channel_configuration_index = channel_configuration as u8;

    for i in 0..number_of_elements as usize {
        // set USAC_ID_BIT to map usacElementType to MP4_ELEMENT_ID enum
        let usac_element_type = (hbs.read_bits(2) as i32) | USAC_ID_BIT;
        usc.element[i].usac_element_type = usac_element_type;

        if aot == AOT_MPEGH3DA {
            // calculate the current group
            if is_channel_element(usac_element_type) {
                cnt_signals += 1;
                if usac_element_type == ID_USAC_CPE {
                    cnt_signals += 1;
                }
                // check whether we are in the next signal group PART I
                // This is for the case we have no extension element in one group
                if cnt_signals > usc.m_signal_group_type[grp].count as i32
                    && usc.bs_num_signal_groups > 1
                {
                    cnt_signals -= usc.m_signal_group_type[grp].count as i32;
                    grp += 1;
                    if grp >= usc.bs_num_signal_groups as usize
                        || cnt_signals > usc.m_signal_group_type[grp].count as i32
                    {
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                    mpegh_mct_element = -1;
                }
                last_ext_element = ID_EXT_ELE_UNKNOWN;
            }
        }

        match usac_element_type {
            ID_USAC_SCE | ID_USAC_CPE => {
                // UsacCoreConfig() ISO/IEC FDIS 23003-3  Table 10
                if hbs.read_bit() != 0 {
                    // tw_mdct
                    return TRANSPORTDEC_UNSUPPORTED_FORMAT;
                }
                let el = &mut usc.element[i];
                if aot == AOT_MPEGH3DA {
                    el.fullband_lpd = hbs.read_bits(1) as u8;
                }
                el.m_noise_filling = hbs.read_bits(1) as u8;
                // end of UsacCoreConfig()
                if aot == AOT_MPEGH3DA {
                    // mpegh3daCoreConfig() ISO/IEC FDIS 23008-3  Table 11
                    el.enhanced_noise_filling = hbs.read_bits(1) as u8;
                    if el.enhanced_noise_filling != 0 {
                        el.igf_use_enf = hbs.read_bit() as u8;
                        el.igf_use_high_res = hbs.read_bit() as u8;
                        el.igf_use_whitening = hbs.read_bit() as u8;
                        el.igf_after_tns_synth = hbs.read_bit() as u8;
                        el.igf_start_index = hbs.read_bits(5) as u8;
                        el.igf_stop_index = hbs.read_bits(4) as u8;
                    }
                    // end of mpegh3daCoreConfig()
                }
                if usac_element_type == ID_USAC_SCE {
                    usc.m_n_usac_channels = usc.m_n_usac_channels.wrapping_add(1);
                    continue;
                }

                if aot == AOT_MPEGH3DA {
                    // IGF IndependentTiling
                    if el.enhanced_noise_filling != 0 {
                        el.igf_independent_tiling = hbs.read_bits(1) as u8;
                    }
                }
                usc.m_n_usac_channels = usc.m_n_usac_channels.wrapping_add(2);

                if aot == AOT_MPEGH3DA {
                    // parse quad channel element config

                    // formula for calculating nBits q.v.: ISO/IEC DIS 23008-3  Table 10
                    // nbits = floor(log2(numAudioChannels + numAudioObjects +
                    // numHOATransportChannels + numSAOCTransportChannels - 1)) + 1
                    let sum_tmp = (usc.num_audio_channels as u32
                        + usc.num_audio_objects as u32
                        + usc.num_hoa_transport_channels as u32)
                        .wrapping_sub(1);
                    if sum_tmp == 0 {
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                    let nbits = 15 - fixnormz_s(sum_tmp as i16) + 1;

                    let qce_index = hbs.read_bits(2);
                    if qce_index != 0 {
                        // QCE not supported by MPEG-H LC Profile
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                    let el = &mut usc.element[i];
                    el.shift_index1 = hbs.read_bits(1) as u8;
                    if el.shift_index1 > 0 {
                        el.shift_channel1 = hbs.read_bits(nbits as u32) as u8;
                    }
                    if usc.m_sbr_ratio_index == 0 {
                        usc.element[i].lpd_stereo_index = hbs.read_bit() as u8;
                    }
                }
            }

            ID_USAC_LFE => {
                // Check if MCT is active and if this LFE is part of the MCT signals, because that
                // is not valid.
                if mpegh_mct_element != -1
                    && (cnt_signals > TP_MAX_CHANNELS_PER_SIGNAL_GROUP as i32
                        || usc.element[mpegh_mct_element as usize]
                            .ext_element
                            .ext_config
                            .mct
                            .mct_chan_mask
                            & (1u32 << (31 - (cnt_signals - 1)))
                            != 0)
                {
                    return TRANSPORTDEC_PARSE_ERROR;
                }
                usc.element[i].fullband_lpd = 0;
                usc.element[i].enhanced_noise_filling = 0;
                usc.element[i].m_noise_filling = 0;
                usc.m_n_usac_channels = usc.m_n_usac_channels.wrapping_add(1);
            }

            ID_USAC_EXT => {
                // check whether we are in the next signal group PART II
                // works only if extension elements comes first
                if cnt_signals >= usc.m_signal_group_type[grp].count as i32
                    && usc.bs_num_signal_groups > 1
                {
                    cnt_signals -= usc.m_signal_group_type[grp].count as i32;

                    grp += 1;
                    if grp as i32 > usc.bs_num_signal_groups as i32 - 1 {
                        grp = usc.bs_num_signal_groups as usize - 1;
                    }
                }

                error_status = ext_element_config(
                    &mut usc.element[i].ext_element,
                    hbs,
                    cb,
                    usc.m_signal_group_type[grp].count,
                    usc.obj_num_signal_groups,
                    usc.ch_num_signal_groups,
                    samples_per_frame,
                    usc.sub_stream_index as i32,
                    aot,
                );
                let ext_type = usc.element[i].ext_element.usac_ext_element_type;
                if ext_type == ID_EXT_ELE_MCT {
                    mpegh_mct_element = i as i32;
                }
                if ext_type != ID_EXT_ELE_FILL {
                    last_ext_element = ext_type;
                }
                if ext_type == ID_EXT_ELE_PROD_METADATA && i > 0 {
                    for j in (0..i).rev() {
                        if is_channel_element(usc.element[j].usac_element_type) {
                            // ID_EXT_ELE_PROD_METADATA must precede channel elements
                            return TRANSPORTDEC_PARSE_ERROR;
                        } else if usc.element[j].ext_element.usac_ext_element_type
                            == ID_EXT_ELE_PROD_METADATA
                        {
                            // ID_EXT_ELE_PROD_METADATA can only occur once
                            return TRANSPORTDEC_PARSE_ERROR;
                        }
                    }
                }
                if ext_type == ID_EXT_ELE_UNI_DRC {
                    uni_drc_element = i as i32;
                }

                if error_status != 0 {
                    return error_status;
                }
            }

            _ => {
                // non USAC-element encountered
                return TRANSPORTDEC_PARSE_ERROR;
            }
        }
    }

    // sanity check
    if aot == AOT_MPEGH3DA
        && (usc.m_n_usac_channels as i32
            != usc.num_audio_channels as i32
                + usc.num_audio_objects as i32
                + usc.num_hoa_transport_channels as i32
            || (last_ext_element != ID_EXT_ELE_UNKNOWN && last_ext_element != ID_EXT_ELE_FILL))
    {
        return TRANSPORTDEC_PARSE_ERROR;
    }

    if uni_drc_element == -1 && cb.has_uni_drc() {
        // no uniDrcConfig contained. Clear the uniDrcConfig struct by feeding an empty extension
        // element
        let sub_stream_index = usc.sub_stream_index as i32;
        error_status = cb.uni_drc(None, 0, 0, sub_stream_index, 0, aot);
        if error_status != TRANSPORTDEC_OK {
            return error_status;
        }
    }

    error_status
}

fn ch_is_explicit_center(azimuth_angle: i32) -> bool {
    azimuth_angle == 0 || azimuth_angle == 180
}

/// Emulation of the memory of `CSUsacConfig` from `m_signalGroupTypeSpeakers` up to
/// `objNumSignalGroups` in the C layout of the reference build.
///
/// `SpeakerConfig3d()` writes speaker descriptions through a pointer without bounds check: a
/// symmetric speaker pair at the end of a full layout (or a layout that is larger than its signal
/// group) writes beyond the end of the speaker array into the following members of
/// `CSUsacConfig`. The parser continues with the overwritten values, so this is reproduced
/// byte-exactly: the speakers are written into this image, which is then stored back into the
/// struct.
struct SpeakerMem {
    b: [u8; SPK_MEM_SIZE],
}

const SPK_SIZE: usize = 6; // sizeof(mpegh_speaker_t): Lfe @0, Az @2, El @4
const OFF_REFERENCE_LAYOUT: usize = 2 * 28 * SPK_SIZE;
const OFF_SPEAKERS: usize = OFF_REFERENCE_LAYOUT + 2;
const OFF_BS_NUM_SIGNAL_GROUPS: usize = OFF_SPEAKERS + TP_USAC_MAX_SPEAKERS * SPK_SIZE;
const SPK_MEM_SIZE: usize = OFF_BS_NUM_SIGNAL_GROUPS + 6;

impl SpeakerMem {
    fn load(usc: &CSUsacConfig) -> Self {
        let mut m = SpeakerMem { b: [0; SPK_MEM_SIZE] };
        for (i, s) in usc.m_signal_group_type_speakers.iter().enumerate() {
            m.put_speaker(i * SPK_SIZE, s);
        }
        m.b[OFF_REFERENCE_LAYOUT] = usc.reference_layout;
        for (i, s) in usc.speakers.iter().enumerate() {
            m.put_speaker(OFF_SPEAKERS + i * SPK_SIZE, s);
        }
        let o = OFF_BS_NUM_SIGNAL_GROUPS;
        m.b[o] = usc.bs_num_signal_groups;
        m.b[o + 1] = usc.mpegh3da_profile_level_indication;
        m.b[o + 2] = usc.num_audio_objects;
        m.b[o + 3] = usc.num_hoa_transport_channels;
        m.b[o + 4] = usc.ch_num_signal_groups;
        m.b[o + 5] = usc.obj_num_signal_groups;
        m
    }

    fn store(&self, usc: &mut CSUsacConfig) {
        for i in 0..usc.m_signal_group_type_speakers.len() {
            usc.m_signal_group_type_speakers[i] = self.get_speaker(i * SPK_SIZE);
        }
        usc.reference_layout = self.b[OFF_REFERENCE_LAYOUT];
        for i in 0..usc.speakers.len() {
            usc.speakers[i] = self.get_speaker(OFF_SPEAKERS + i * SPK_SIZE);
        }
        let o = OFF_BS_NUM_SIGNAL_GROUPS;
        usc.bs_num_signal_groups = self.b[o];
        usc.mpegh3da_profile_level_indication = self.b[o + 1];
        usc.num_audio_objects = self.b[o + 2];
        usc.num_hoa_transport_channels = self.b[o + 3];
        usc.ch_num_signal_groups = self.b[o + 4];
        usc.obj_num_signal_groups = self.b[o + 5];
    }

    fn put_speaker(&mut self, off: usize, s: &MpeghSpeaker) {
        self.b[off] = s.lfe;
        self.b[off + 2..off + 4].copy_from_slice(&s.az.to_le_bytes());
        self.b[off + 4..off + 6].copy_from_slice(&s.el.to_le_bytes());
    }

    fn get_speaker(&self, off: usize) -> MpeghSpeaker {
        MpeghSpeaker {
            lfe: self.b[off],
            az: i16::from_le_bytes([self.b[off + 2], self.b[off + 3]]),
            el: i16::from_le_bytes([self.b[off + 4], self.b[off + 5]]),
        }
    }

    fn set_lfe(&mut self, base: usize, i: usize, v: u8) {
        self.b[base + i * SPK_SIZE] = v;
    }
    fn set_az(&mut self, base: usize, i: usize, v: i16) {
        let o = base + i * SPK_SIZE + 2;
        self.b[o..o + 2].copy_from_slice(&v.to_le_bytes());
    }
    fn set_el(&mut self, base: usize, i: usize, v: i16) {
        let o = base + i * SPK_SIZE + 4;
        self.b[o..o + 2].copy_from_slice(&v.to_le_bytes());
    }
    fn az(&self, base: usize, i: usize) -> i16 {
        let o = base + i * SPK_SIZE + 2;
        i16::from_le_bytes([self.b[o], self.b[o + 1]])
    }
    fn el(&self, base: usize, i: usize) -> i16 {
        let o = base + i * SPK_SIZE + 4;
        i16::from_le_bytes([self.b[o], self.b[o + 1]])
    }
}

/// Where `SpeakerConfig3d()` writes the speakers to.
#[derive(Clone, Copy)]
enum SpeakerTarget {
    /// `usc->speakers`
    ReferenceLayout,
    /// `&usc->m_signalGroupTypeSpeakers[n]`
    SignalGroup(usize),
}

/// `SpeakerConfig3d(hBs, speakers, pNumSpeakers)`: returns the CICP layout index, -1 on error or
/// -2 for contribution mode.
fn speaker_config_3d(
    hbs: &mut FdkBitStream,
    usc: &mut CSUsacConfig,
    target: SpeakerTarget,
    p_num_speakers: &mut i32,
) -> i32 {
    let mut mem = SpeakerMem::load(usc);
    let base = match target {
        SpeakerTarget::ReferenceLayout => OFF_SPEAKERS,
        SpeakerTarget::SignalGroup(n) => n * SPK_SIZE,
    };
    let r = speaker_config_3d_mem(hbs, &mut mem, base, p_num_speakers);
    mem.store(usc);
    r
}

fn speaker_config_3d_mem(
    hbs: &mut FdkBitStream,
    mem: &mut SpeakerMem,
    base: usize,
    p_num_speakers: &mut i32,
) -> i32 {
    let mut cicp = 0;

    let speaker_layout_type = hbs.read_bits(2);
    if speaker_layout_type == 0 {
        let mut az_el_lfe =
            [Cicp2GeometryChannelGeometry::default(); CICP2GEOMETRY_MAX_LOUDSPEAKERS];
        let mut n_lfe = 0;

        cicp = hbs.read_bits(6) as i32; // CICPspeakerLayoutIdx
        if cicp >= MAX_CICP_VALUE {
            return -1;
        }

        let error = cicp2geometry_get_geometry_from_cicp(
            cicp,
            Some(&mut az_el_lfe),
            p_num_speakers,
            &mut n_lfe,
        );
        if error != CICP2GEOMETRY_OK {
            return -1;
        }
        *p_num_speakers += n_lfe;

        for i in 0..(*p_num_speakers).max(0) as usize {
            mem.set_az(base, i, az_el_lfe[i].az as i16);
            mem.set_el(base, i, az_el_lfe[i].el as i16);
            mem.set_lfe(base, i, az_el_lfe[i].lfe as u8);
        }
    } else {
        let num_speakers = hbs.escaped_value(5, 8, 16) as i32 + 1;
        if num_speakers > TP_USAC_MAX_SPEAKERS as i32 {
            return -1;
        }
        *p_num_speakers = num_speakers;
        if speaker_layout_type == 1 {
            for i in 0..num_speakers as usize {
                let mut az_el_lfe = Cicp2GeometryChannelGeometry::default();

                let cicp_idx = hbs.read_bits(7) as i32; // CICPspeakerIdx
                let error = cicp2geometry_get_geometry_from_cicp_loudspeaker_index(
                    cicp_idx,
                    Some(&mut az_el_lfe),
                );
                if error != CICP2GEOMETRY_OK {
                    return -1;
                }
                mem.set_az(base, i, az_el_lfe.az as i16);
                mem.set_el(base, i, az_el_lfe.el as i16);
                mem.set_lfe(base, i, az_el_lfe.lfe as u8);
            }
        } else if speaker_layout_type == 2 {
            let angular_precision = hbs.read_bits(1);
            let mut i = 0usize;
            while (i as i32) < num_speakers {
                let is_cicp_speaker_idx = hbs.read_bit();

                if is_cicp_speaker_idx != 0 {
                    let mut az_el_lfe = Cicp2GeometryChannelGeometry::default();

                    let cicp_idx = hbs.read_bits(7) as i32; // CICPspeakerIdx
                    let error = cicp2geometry_get_geometry_from_cicp_loudspeaker_index(
                        cicp_idx,
                        Some(&mut az_el_lfe),
                    );
                    if error != CICP2GEOMETRY_OK {
                        return -1;
                    }
                    mem.set_az(base, i, az_el_lfe.az as i16);
                    mem.set_el(base, i, az_el_lfe.el as i16);
                    mem.set_lfe(base, i, az_el_lfe.lfe as u8);
                } else {
                    let elevation_class = hbs.read_bits(2);

                    match elevation_class {
                        0 => mem.set_el(base, i, 0),
                        1 => mem.set_el(base, i, 35),
                        2 => mem.set_el(base, i, -15),
                        _ => {
                            let mut elevation_angle_idx =
                                hbs.read_bits(if angular_precision != 0 { 7 } else { 5 }) as i32;

                            if elevation_angle_idx != 0 {
                                // Table 44
                                let elevation_direction = hbs.read_bit();
                                if elevation_direction != 0 {
                                    elevation_angle_idx = -elevation_angle_idx;
                                }
                            }
                            mem.set_el(
                                base,
                                i,
                                (elevation_angle_idx * if angular_precision != 0 { 1 } else { 5 })
                                    as i16,
                            );
                        }
                    }
                    {
                        let azimuth_angle_idx =
                            hbs.read_bits(if angular_precision != 0 { 8 } else { 6 }) as i32;

                        mem.set_az(
                            base,
                            i,
                            (azimuth_angle_idx * if angular_precision != 0 { 1 } else { 5 }) as i16,
                        );
                        if !ch_is_explicit_center(mem.az(base, i) as i32) {
                            let azimuth_direction = hbs.read_bit();
                            if azimuth_direction != 0 {
                                mem.set_az(base, i, (-(mem.az(base, i) as i32)) as i16);
                            }
                        }
                    }
                    mem.set_lfe(base, i, hbs.read_bit() as u8); // isLFE
                }

                if !ch_is_explicit_center(mem.az(base, i) as i32) {
                    let also_add_symmetric_pair = hbs.read_bit();
                    if also_add_symmetric_pair != 0 {
                        // Also add the speaker with the opposite AzimuthDirection
                        i += 1;
                        mem.set_az(base, i, (-(mem.az(base, i - 1) as i32)) as i16);
                        mem.set_el(base, i, mem.el(base, i - 1));
                        mem.set_lfe(base, i, 0);
                    }
                }
                i += 1;
            }
        } else {
            return -2; // speakerLayoutType == 3 is Contribution Mode
        }
    }
    cicp
}

fn mpegh_config_parse(
    asc: &mut CSAudioSpecificConfig,
    hbs: &mut FdkBitStream,
    cb: &mut dyn TpCallbacks,
) -> TransportDecError {
    let mut num_speakers_ref_layout = 0;

    asc.m_sc.mpegh3da_profile_level_indication = hbs.read_bits(8) as u8;

    let usac_sampling_frequency = get_sample_rate(hbs, &mut asc.m_sampling_frequency_index, 5);
    match usac_sampling_frequency {
        48000 | 44100 | 32000 | 29400 | 24000 | 22050 | 16000 | 14700 => {}
        _ => {
            // unsupported sampling frequency
            return TRANSPORTDEC_PARSE_ERROR;
        }
    }

    asc.m_sampling_frequency = usac_sampling_frequency as u32;

    // coreSbrFrameLengthIndex
    if hbs.read_bits(3) != 1 {
        return TRANSPORTDEC_PARSE_ERROR;
    }
    asc.m_sc.m_core_sbr_frame_length_index = 1;
    asc.m_samples_per_frame = 1024;
    asc.m_sc.m_sbr_ratio_index = 0;

    hbs.read_bit(); // coreQmfDelayCompensation

    hbs.read_bit(); // receiverDelayCompensation

    asc.m_channel_configuration = 0;

    let usc = &mut asc.m_sc;
    let mut reference_layout =
        speaker_config_3d(hbs, usc, SpeakerTarget::ReferenceLayout, &mut num_speakers_ref_layout);
    if reference_layout < 0 {
        if reference_layout == -2 {
            // contribution mode
            reference_layout = 0;
        } else {
            return TRANSPORTDEC_PARSE_ERROR;
        }
    }
    usc.reference_layout = reference_layout as u8;

    // FrameworkConfig3d()
    {
        // Signals3d()
        let mut signal_group_type_grp_prev = 0;
        let mut sig_idx = 0usize;
        usc.ch_num_signal_groups = 0;
        usc.obj_num_signal_groups = 0;
        usc.bs_num_signal_groups = (hbs.read_bits(5) + 1) as u8;
        if usc.bs_num_signal_groups as usize > TP_MPEGH_MAX_SIGNAL_GROUPS {
            return TRANSPORTDEC_PARSE_ERROR;
        }
        let mut grp = 0usize;
        while grp < usc.bs_num_signal_groups as usize {
            let signal_group_type_grp = hbs.read_bits(3) as i32;

            // bsNumberOfSignals_grp + 1
            let number_of_signals_grp = hbs.escaped_value(5, 8, 16) as i32 + 1;
            if sig_idx as i32 + number_of_signals_grp > TP_MPEGH_MAX_SIGNAL_GROUPS as i32 {
                return TRANSPORTDEC_UNSUPPORTED_FORMAT;
            }

            if number_of_signals_grp > TP_MAX_CHANNELS_PER_SIGNAL_GROUP as i32 {
                return TRANSPORTDEC_UNSUPPORTED_FORMAT;
            }

            // Check signal group type order
            if signal_group_type_grp < signal_group_type_grp_prev {
                return TRANSPORTDEC_PARSE_ERROR;
            }
            signal_group_type_grp_prev = signal_group_type_grp;

            let first_sig_idx = sig_idx;
            {
                let sg = &mut usc.m_signal_group_type[grp];
                sg.layout = usc.reference_layout;

                sg.count = number_of_signals_grp as u8;
                sg.type_ = signal_group_type_grp as u8;
                sg.first_sig_idx = sig_idx as u8;

                // Assign speaker description memory for signal group.
                sg.speakers = SpeakerRef::SignalGroup(sig_idx as u8);
                sg.speakers_owner = SPEAKERS_OWNER_SELF;
            }

            for _i in 0..number_of_signals_grp {
                usc.sig_idx2_grp_idx[sig_idx] = grp as u8;
                sig_idx += 1;
            }

            match signal_group_type_grp {
                0 => {
                    // SignalGroupTypeChannels
                    usc.ch_num_signal_groups = usc.ch_num_signal_groups.wrapping_add(1);
                    usc.num_audio_channels =
                        usc.num_audio_channels.wrapping_add(number_of_signals_grp as u8);
                    let mut num_speakers = 0;

                    if hbs.read_bit() != 0 {
                        // differsFromReferenceLayout
                        let layout = speaker_config_3d(
                            hbs,
                            usc,
                            SpeakerTarget::SignalGroup(first_sig_idx),
                            &mut num_speakers,
                        );
                        if layout < 0 {
                            return TRANSPORTDEC_PARSE_ERROR;
                        }
                        usc.m_signal_group_type[grp].layout = layout as u8;
                    } else {
                        usc.m_signal_group_type[grp].layout = usc.reference_layout;
                        usc.m_signal_group_type[grp].speakers = SpeakerRef::ReferenceLayout;
                        num_speakers = num_speakers_ref_layout;
                    }
                    if number_of_signals_grp != num_speakers {
                        return TRANSPORTDEC_PARSE_ERROR;
                    }
                }
                1 => {
                    // SignalGroupTypeObject
                    // store indice of current signal group
                    usc.obj_signal_groups_indices[usc.obj_num_signal_groups as usize] = grp as u8;
                    // count signal groups of type object
                    usc.obj_num_signal_groups = usc.obj_num_signal_groups.wrapping_add(1);
                    usc.num_audio_objects =
                        usc.num_audio_objects.wrapping_add(number_of_signals_grp as u8);
                }
                _ => {
                    // 3: SignalGroupTypeHOA, others
                    return TRANSPORTDEC_UNSUPPORTED_FORMAT;
                }
            }
            grp += 1;
        }
    }

    let mut err = usac_mpegh_decoder_config_parse(asc, hbs, cb);

    if err != TRANSPORTDEC_OK {
        return err;
    }

    if hbs.read_bits(1) != 0 {
        // usacConfigExtensionPresent
        let aot = asc.m_aot;
        err = config_extension(&mut asc.m_sc, aot, hbs, cb);
    } else if cb.has_uni_drc() {
        // no loudnessInfoSet contained. Clear the loudnessInfoSet struct by feeding an empty
        // config extension
        let sub_stream_index = asc.m_sc.sub_stream_index as i32;
        err = cb.uni_drc(None, 0, 1, sub_stream_index, 0, asc.m_aot);
        if err != TRANSPORTDEC_OK {
            return err;
        }
    }

    err
}

// API Functions

/// `AudioSpecificConfig_Init(pAsc)`: clear the struct and init all values that should not be
/// zero.
pub fn audio_specific_config_init(asc: &mut CSAudioSpecificConfig) {
    *asc = CSAudioSpecificConfig::default();

    // Init all values that should not be zero.
    asc.m_aot = AOT_NONE;
    asc.m_sampling_frequency_index = 0xf;
    asc.m_ep_config = -1;
    asc.m_extension_audio_object_type = AOT_NULL_OBJECT;
}

/// `Mpegh3daConfig_Parse(pAsc, pASI, hBs, fExplicitBackwardCompatible, cb, configMode,
/// configChanged, targetLayout, subStreamIndex, pLoudnessInfoSetPosition)`: parse an
/// mpegh3daConfig() (ISO/IEC DIS 23008-3) into `asc`.
///
/// The ASI (`pASI`) and `pLoudnessInfoSetPosition` are obtained from `cb`
/// ([`TpCallbacks::asi`], [`TpCallbacks::loudness_info_set_position`]).
pub fn mpegh3da_config_parse(
    asc: &mut CSAudioSpecificConfig,
    bs: &mut FdkBitStream,
    _f_explicit_backward_compatible: i32,
    cb: &mut dyn TpCallbacks,
    config_mode: u8,
    config_changed: u8,
    target_layout: i32,
    sub_stream_index: i32,
) -> TransportDecError {
    // Temporal code in order to compare and signal a change in one of the signal groups
    audio_specific_config_init(asc);

    asc.m_sc.sub_stream_index = sub_stream_index as u8;
    asc.m_sc.target_layout = target_layout;

    asc.config_mode = config_mode;
    asc.aac_config_changed = config_changed;
    asc.sbr_config_changed = config_changed;
    asc.sac_config_changed = config_changed;

    asc.m_aot = AOT_MPEGH3DA;

    // => gets overwritten with usacSamplingFrequencyIndex in MpegHConfig_Parse()
    asc.m_sampling_frequency = 0;
    // => gets overwritten with channelConfigurationIndex in MpegHConfig_Parse()
    asc.m_channel_configuration = 0;

    // SBR extension ( explicit non-backwards compatible mode )
    asc.m_sbr_present_flag = 0;
    asc.m_ps_present_flag = 0;
    asc.m_extension_audio_object_type = AOT_NULL_OBJECT;

    if sub_stream_index == 0 {
        let valid_bits = bs.get_valid_bits();
        let pos = cb.loudness_info_set_position();
        pos[0] = valid_bits;
        pos[1] = 0;
        pos[2] = 0;
    }

    // parse MPEGH 3D audio config, q.v. ISO/IEC DIS 23008-3  mpegh3daConfig()
    mpegh_config_parse(asc, bs, cb)
}

/// `earconInfo(bs, info)`: syntax of earconInfo().
pub fn earcon_info(bs: &mut FdkBitStream, info: &mut EarconInfo) -> TransportDecError {
    info.m_num_earcons = bs.read_bits(7) + 1;
    if info.m_num_earcons > EARCON_MAX_NUM_SIGNALS {
        return TRANSPORTDEC_PARSE_ERROR;
    }
    for _i in 0..info.m_num_earcons {
        bs.read_bits(1); // earconIsIndependent
        bs.read_bits(7); // earconID
        bs.read_bits(4); // earconType
        bs.read_bits(1); // earconActive
        let earcon_position_type = bs.read_bits(2); // earconPositionType
        if earcon_position_type == 0 {
            bs.read_bits(7);
        } else if earcon_position_type == 1 {
            bs.read_bits(8); // earcon_azimuth
            bs.read_bits(6); // earcon_elevation
            bs.read_bits(9); // earcon_distance
        }

        if bs.read_bits(1) != 0 {
            // earconHasGain
            bs.read_bits(7); // earcon_gain
        }

        if bs.read_bits(1) != 0 {
            // earconHasTextLabel
            let earcon_num_languages = bs.read_bits(4);
            for _n in 0..earcon_num_languages {
                bs.read_bits(24); // earconLanguage
                let earcon_text_data_length = bs.read_bits(8); // earconTextDataLength
                for _c in 0..earcon_text_data_length {
                    bs.read_bits(8); // earconTextData
                }
            }
        }
    }
    TRANSPORTDEC_OK
}

/// `pcmDataConfig(bs, config)`
pub fn pcm_data_config(bs: &mut FdkBitStream, config: &mut EarconConfig) -> TransportDecError {
    config.m_num_pcm_signals = bs.read_bits(7) + 1; // numPcmSignals
    if config.m_num_pcm_signals > EARCON_MAX_NUM_SIGNALS {
        return TRANSPORTDEC_PARSE_ERROR;
    }
    bs.read_bits(1); // pcmAlignAudioFlag
    if bs.read_bits(5) == 0x1f {
        // pcmSamplingRateIndex
        bs.read_bits(24); // pcmSamplingRate
    }

    bs.read_bits(3); // pcmBitsPerSampleIndex

    config.m_bs_pcm_frame_size_index = bs.read_bits(3);
    match config.m_bs_pcm_frame_size_index {
        2 | 3 => config.m_bs_pcm_frame_size = 1024,
        5 => config.m_bs_pcm_frame_size = bs.read_bits(16), // pcmFixFrameSize
        _ => {}
    }

    for _i in 0..config.m_num_pcm_signals {
        bs.read_bits(7); // pcmSignal_ID[i]
    }

    config.m_bs_pcm_loudness_value = bs.read_bits(8); // bsPcmLoudnessValue
    config.m_pcm_has_attenuation_gain = bs.read_bits(2); // pcmHasAttenuationGain

    if config.m_pcm_has_attenuation_gain == 1 {
        config.m_bs_pcm_attenuation_gain = bs.read_bits(8); // bsPcmAttenuationGain
    }

    TRANSPORTDEC_OK
}
