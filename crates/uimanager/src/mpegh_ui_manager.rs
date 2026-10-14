// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from include/mpeghUIManager.h and libMpeghUIMan/src/mpeghUIManager.cpp.

//! MPEG-H UI manager library interface (`mpegh_UI_*`): parses MHAS streams, applies user
//! interactivity and inserts user interaction and loudness/DRC packets into the MHAS stream.

use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_tpdec::*;

use crate::ui_manager::*;

/// `MAX_NUMBER_SUBSTREAMS`
pub const MAX_NUMBER_SUBSTREAMS: usize = 16;

/// `MPEGH_UI_ERROR`
pub type MpeghUiError = i32;
pub const MPEGH_UI_OK: MpeghUiError = 0;
pub const MPEGH_UI_OUT_OF_MEMORY: MpeghUiError = 1;
/// This error is not recoverable until a new RAP frame is processed, i.e. UI events will be
/// delayed.
pub const MPEGH_UI_BUFFER_TOO_SMALL: MpeghUiError = 2;
pub const MPEGH_UI_INVALID_PARAM: MpeghUiError = 3;
pub const MPEGH_UI_NOT_ALLOWED: MpeghUiError = 4;
pub const MPEGH_UI_INVALID_STATE: MpeghUiError = 5;
pub const MPEGH_UI_PARSE_ERROR: MpeghUiError = 6;
pub const MPEGH_UI_OK_BUT_NO_VALID_DATA: MpeghUiError = 7;

/// `MPEGH_UI_FORCE_UPDATE`: input flag for [`MpeghUiManager::get_xml_scene_state`]: force output
/// of XML description, even if nothing has changed since the last call.
pub const MPEGH_UI_FORCE_UPDATE: u32 = 1;
/// `MPEGH_UI_FORCE_RESTART_XML`: input flag for [`MpeghUiManager::get_xml_scene_state`]: force
/// restart of XML output if incomplete output was returned by previous call.
pub const MPEGH_UI_FORCE_RESTART_XML: u32 = 4;

/// `MPEGH_UI_NO_CHANGE`: nothing has changed since the last call, no XML output was generated.
pub const MPEGH_UI_NO_CHANGE: u32 = 1;
/// `MPEGH_UI_CONTINUES_XML`: XML output is a continuation of incomplete output of the previous
/// call.
pub const MPEGH_UI_CONTINUES_XML: u32 = 2;
/// `MPEGH_UI_INCOMPLETE_XML`: XML output is not complete, at least one further call of the
/// function is required to get the complete XML output.
pub const MPEGH_UI_INCOMPLETE_XML: u32 = 4;
/// `MPEGH_UI_SHORT_OUTPUT`: DEPRECATED, no longer used.
pub const MPEGH_UI_SHORT_OUTPUT: u32 = 8;

/// Size of the internal input copy (`SAFE_BUFFER_READ`): 64 KiB for Level 4 and 32 KiB for
/// Level 3.
const BS_BUFFER_SIZE: usize = 64 * 1024;

/// `MPEGH_UI_MANAGER` (`HANDLE_MPEGH_UI_MANAGER` is `Box<MpeghUiManager>`, created by
/// [`MpeghUiManager::open`]; `mpegh_UI_Manager_Close()` is dropping it)
pub struct MpeghUiManager {
    pub h_ui_manager: Box<UiManager>,
    pub is_active: u8,
    pub config_found: u8,
    pub substream_label: [u32; MAX_NUMBER_SUBSTREAMS],
    pub insert_offset: u32,
    /// `SAFE_BUFFER_READ` input copy (keeps its content between calls like in C++)
    bs_buffer: BitMem,
}

/// A copy with its own input buffer (C's struct copy copies the array).
impl Clone for MpeghUiManager {
    fn clone(&self) -> MpeghUiManager {
        MpeghUiManager {
            h_ui_manager: self.h_ui_manager.clone(),
            is_active: self.is_active,
            config_found: self.config_found,
            substream_label: self.substream_label,
            insert_offset: self.insert_offset,
            bs_buffer: self.bs_buffer.deep_clone(),
        }
    }
}

fn next_pow2(x: u32) -> u32 {
    let mut y: u32 = 1;
    while y < x {
        y <<= 1;
    }
    y
}

/// Bit writer that only counts bits if no bitstream is given (`FDKwriteBits(NULL, ...)`).
struct Bw<'a>(Option<&'a mut FdkBitStream>);

impl Bw<'_> {
    fn bits(&mut self, value: u32, n: u32) -> u32 {
        if let Some(bs) = self.0.as_deref_mut() {
            bs.write_bits(value, n);
        }
        n
    }

    /// `FDKwriteEscapedValue()`
    fn escaped(&mut self, mut value: u32, n1: u32, n2: u32, n3: u32) -> u32 {
        let mut nbits: u8 = 0;
        let mut tmp = (1u32 << n1) - 1;

        if value < tmp {
            nbits = nbits.wrapping_add(self.bits(value, n1) as u8);
        } else {
            nbits = nbits.wrapping_add(self.bits(tmp, n1) as u8);
            value -= tmp;
            tmp = (1u32 << n2) - 1;

            if value < tmp {
                nbits = nbits.wrapping_add(self.bits(value, n2) as u8);
            } else {
                nbits = nbits.wrapping_add(self.bits(tmp, n2) as u8);
                value -= tmp;

                nbits = nbits.wrapping_add(self.bits(value, n3) as u8);
            }
        }

        nbits as u32
    }
}

/// write MHAS packet header
fn write_packet_header(w: &mut Bw, packet_type: u32, packet_label: u32, packet_length: u32) -> u32 {
    let mut n_bits = 0;

    n_bits += w.escaped(packet_type, 3, 8, 8);
    n_bits += w.escaped(packet_label, 2, 8, 32);
    n_bits += w.escaped(packet_length, 11, 24, 24);

    n_bits >> 3
}

/// write UI packet
fn write_ui_packet(w: &mut Bw, ui_status: &UserInteractivityStatus) -> u32 {
    let mut n_bits = 0;

    // mpegh3daElementInteraction
    n_bits += w.bits(0, 8); // ei_InteractionSignatureDataLength

    n_bits += w.bits(ui_status.interaction_mode as u32, 1);
    n_bits += w.bits(ui_status.num_groups as u32, 7);
    if ui_status.interaction_mode != 0 {
        n_bits += w.bits(ui_status.group_preset_id as u32, 5);
    }

    for grp in 0..ui_status.num_groups as usize {
        let gd = &ui_status.group_data[grp];
        n_bits += w.bits(gd.group_id as u32, 7);
        n_bits += w.bits(gd.on_off as u32, 1);
        n_bits += w.bits(gd.route_to_wire as u32, 1);
        if gd.route_to_wire != 0 {
            n_bits += w.bits(gd.route_to_wire_id as u32, 16);
        }

        if gd.on_off != 0 {
            // always set change flags to avoid problems with downmix presets
            let change_position = 1;
            let change_gain = 1;

            n_bits += w.bits(change_position, 1);
            if change_position != 0 {
                n_bits += w.bits(gd.az_offset as u32, 8);
                n_bits += w.bits(gd.el_offset as u32, 6);
                n_bits += w.bits(gd.dist_fact as u32, 4);
            }

            n_bits += w.bits(change_gain, 1);
            if change_gain != 0 {
                n_bits += w.bits(gd.gain as u32, 7);
            }
        }
    }

    n_bits += w.bits(0, 1); // hasLocalZoomAreaSize

    if n_bits & 7 != 0 {
        n_bits += w.bits(0, 8 - (n_bits & 7));
    }

    n_bits >> 3
}

/// write DRC/loudness packet
fn write_drc_packet(w: &mut Bw, drc_status: &UiDrcLoudnessStatus) -> u32 {
    let mut n_bits = 0;

    n_bits += w.bits(0, 1); // uniDrcInterfaceSignaturePresent

    n_bits += w.bits(0, 1); // systemInterfacePresent

    if drc_status.target_loudness == UI_MANAGER_USE_DEFAULT_TARGET_LOUDNESS {
        n_bits += w.bits(0, 1); // loudnessNormalizationControlInterfacePresent
    } else {
        n_bits += w.bits(1, 1); // loudnessNormalizationControlInterfacePresent
        n_bits += w.bits(1, 1); // loudnessNormalizationOn
        n_bits += w.bits((-(drc_status.target_loudness as i32) << 5) as u32, 12);
        // targetLoudness
    }

    if drc_status.album_mode < 0 {
        n_bits += w.bits(0, 1); // loudnessNormalizationParameterInterfacePresent
    } else {
        n_bits += w.bits(1, 1); // loudnessNormalizationParameterInterfacePresent
        n_bits += w.bits(drc_status.album_mode as u32, 1); // albumMode
        n_bits += w.bits(1, 1); // peakLimiterPresent = 1, as peak limiter is mandatory in MPEG-H
        n_bits += w.bits(0, 1); // changeLoudnessDeviationMax
        n_bits += w.bits(0, 1); // changeLoudnessMeasurementMethod
        n_bits += w.bits(0, 1); // changeLoudnessMeasurementSystem
        n_bits += w.bits(0, 1); // changeLoudnessMeasurementPreProc
        n_bits += w.bits(0, 1); // changeDeviceCutOffFrequency
        n_bits += w.bits(0, 1); // changeLoudnessNormalizationGainDbMax
        n_bits += w.bits(0, 1); // changeLoudnessNormalizationGainModificationDb
        n_bits += w.bits(0, 1); // changeOutputPeakLevelMax
    }

    n_bits += w.bits(1, 1); // dynamicRangeControlInterfacePresent

    if drc_status.drc_selected == UI_MANAGER_DRC_OFF {
        n_bits += w.bits(0, 1); // dynamicRangeControlOn
    } else if drc_status.drc_selected == UI_MANAGER_USE_DEFAULT_DRC_SELECTED {
        n_bits += w.bits(1, 1); // dynamicRangeControlOn
        n_bits += w.bits(1, 3); // numDrcFeatureRequests
        n_bits += w.bits(0, 2); // drcFeatureRequestType

        n_bits += w.bits(0, 4); // numDrcEffectTypeRequests
                                // numDrcEffectTypeRequestsDesired: value 1 to align to legacy bitstreams. It will be
                                // ignored.
        n_bits += w.bits(1, 4);
    } else {
        n_bits += w.bits(1, 1); // dynamicRangeControlOn

        let mut n = 0;
        let mut tmp = drc_status.drc_selected as i32;
        for i in 0..8 {
            if i == 0 || tmp & 15 != 0 {
                n += 1;
            }
            tmp >>= 4;
        }

        n_bits += w.bits(1, 3); // numDrcFeatureRequests
        n_bits += w.bits(0, 2); // drcFeatureRequestType

        n_bits += w.bits(n, 4); // numDrcEffectTypeRequests
        n_bits += w.bits(1, 4); // numDrcEffectTypeRequestsDesired

        tmp = drc_status.drc_selected as i32;
        for i in 0..8 {
            if i == 0 || tmp & 15 != 0 {
                n_bits += w.bits((tmp & 15) as u32, 4);
            }
            tmp >>= 4;
        }
    }

    if drc_status.boost < 0 && drc_status.compress < 0 {
        n_bits += w.bits(0, 1); // dynamicRangeControlParameterInterfacePresent
    } else {
        let change_compress = (drc_status.compress >= 0) as u32;
        let change_boost = (drc_status.boost >= 0) as u32;

        n_bits += w.bits(1, 1); // dynamicRangeControlParameterInterfacePresent

        n_bits += w.bits(change_compress, 1); // changeCompress
        n_bits += w.bits(change_boost, 1); // changeBoost

        if change_compress != 0 {
            let compress = (0x100 - (drc_status.compress as i32 >> 6)).min(0xFF);
            n_bits += w.bits(compress as u32, 8); // compress
        }
        if change_boost != 0 {
            let boost = (0x100 - (drc_status.boost as i32 >> 6)).min(0xFF);
            n_bits += w.bits(boost as u32, 8); // boost
        }

        n_bits += w.bits(0, 1); // changeDrcCharacteristicTarget
    }

    n_bits += w.bits(0, 1); // uniDrcInterfaceExtensionPresent

    if n_bits & 7 != 0 {
        n_bits += w.bits(0, 8 - (n_bits & 7));
    }

    n_bits >> 3
}

impl MpeghUiManager {
    /// `mpegh_UI_Manager_Open()`: open a mpegh UI manager instance
    pub fn open() -> Box<MpeghUiManager> {
        let mut s = Box::new(MpeghUiManager {
            h_ui_manager: UiManager::create(1),
            is_active: 0,
            config_found: 0,
            substream_label: [0; MAX_NUMBER_SUBSTREAMS],
            insert_offset: u32::MAX,
            bs_buffer: BitMem::new(BS_BUFFER_SIZE),
        });

        s.h_ui_manager.set_is_active(s.is_active);

        s
    }

    /// `mpegh_UI_GetXmlSceneState(self, xmlOut, xmlOutSize, flagsIn, flagsOut)`: get XML
    /// description of audio scene and available user interactivity parameters. `xmlOutSize` is
    /// `xml_out.len()` (has to be at least 1024 to ensure success of the function).
    pub fn get_xml_scene_state(
        &mut self,
        xml_out: &mut [u8],
        flags_in: u32,
        flags_out: &mut u32,
    ) -> MpeghUiError {
        self.h_ui_manager.get_xml_scene_state(xml_out, flags_in, flags_out)
    }

    /// `mpegh_UI_ApplyXmlAction(self, xmlIn, xmlInSize, flagsOut)`: send XML description of UI
    /// command to the UI manager (`xmlInSize` is `xml_in.len()`).
    pub fn apply_xml_action(&mut self, xml_in: &[u8], flags_out: &mut u32) -> MpeghUiError {
        self.h_ui_manager.apply_xml_action(xml_in, flags_out)
    }

    /// `mpegh_UI_FeedMHAS(self, mhasBuffer, mhasLength)`: feed MHAS input (all MHAS packets
    /// for exactly one audio frame) into UI manager.
    pub fn feed_mhas(&mut self, mhas_buffer: &[u8]) -> MpeghUiError {
        let mhas_length = mhas_buffer.len();
        let mut prev_packet_type = MHA_PACTYP_NONE;
        let mut frame_found = false;
        let mut ui_packet_found = false;
        let mut marker_found = false;
        let mut labels = [0u32; MAX_NUMBER_SUBSTREAMS];
        let mut prev_xml_changed = 0;
        let mut uuid = [0u8; 16];
        let mut prev_uuid = [0u8; 16];
        let mut prev_asi_crc: u16 = 0;
        let mut substr_switch = false;

        if mhas_length > BS_BUFFER_SIZE {
            return MPEGH_UI_PARSE_ERROR;
        }
        self.bs_buffer.write(0, mhas_buffer);
        let mut bs = FdkBitStream::default();
        bs.init(
            &self.bs_buffer,
            BS_BUFFER_SIZE as u32,
            (mhas_length * 8) as u32,
            FdkBsCfg::BsReader,
        );
        let h_bs = &mut bs;

        let mut n_bits_ins = h_bs.get_valid_bits();

        self.config_found = 0;
        self.insert_offset = u32::MAX;

        while h_bs.get_valid_bits() != 0 {
            let mut is_main_stream = false;
            let mut substream_index: i32 = -1;

            // parse MHAS packet header
            let packet_type = h_bs.escaped_value(3, 8, 8) as MhaPactyp;
            let packet_label = h_bs.escaped_value(2, 8, 32);
            let packet_length = h_bs.escaped_value(11, 24, 24);

            let mut n_bits = h_bs.get_valid_bits();
            if n_bits < (8 * packet_length) as i32 {
                return MPEGH_UI_PARSE_ERROR;
            }

            if prev_packet_type == MHA_PACTYP_MPEGH3DACFG
                && packet_type != MHA_PACTYP_MPEGH3DACFG
                && packet_type != MHA_PACTYP_CRC16
                && packet_type != MHA_PACTYP_CRC32
            {
                let mut config_change = false;

                if labels[0] == 0 || labels[0] > 16 {
                    return MPEGH_UI_PARSE_ERROR;
                }

                // check config change and substream switching
                for i in 0..MAX_NUMBER_SUBSTREAMS {
                    if labels[i] != self.substream_label[i] {
                        if i == 0
                            || (labels[i].wrapping_sub(1) >> 4)
                                == (self.substream_label[i].wrapping_sub(1) >> 4)
                        {
                            config_change = true;
                        } else {
                            substr_switch = true;
                        }
                    }
                    self.substream_label[i] = labels[i];
                }

                // save previous XML changed flag and UUID
                prev_xml_changed = self.h_ui_manager.get_xml_changed();
                self.h_ui_manager.get_uuid(&mut prev_uuid);
                prev_asi_crc = self.h_ui_manager.get_asi_pointer().crc_for_compare;

                if config_change {
                    // clear ASI (to prevent keeping old ASI or parts of old ASI)
                    asi_reset(self.h_ui_manager.get_asi_pointer());
                } else if substr_switch {
                    // reset availability states
                    asi_reset_availability(self.h_ui_manager.get_asi_pointer());
                }
            }

            if packet_label > 0 {
                for i in 0..MAX_NUMBER_SUBSTREAMS {
                    if packet_label == self.substream_label[i] {
                        substream_index = i as i32;
                        break;
                    }
                }
            }

            // check if main stream packet
            if packet_label <= 0x10 {
                is_main_stream = true;
            }

            match packet_type {
                MHA_PACTYP_MPEGH3DACFG => 'cfg: {
                    if packet_label == 0 {
                        return MPEGH_UI_PARSE_ERROR;
                    }

                    if is_main_stream {
                        if self.config_found != 0 {
                            return MPEGH_UI_PARSE_ERROR;
                        }

                        self.config_found = 1;
                        substream_index = 0;
                    } else {
                        if self.config_found == 0 {
                            return MPEGH_UI_PARSE_ERROR;
                        }

                        substream_index = 1;
                        while (substream_index as usize) < MAX_NUMBER_SUBSTREAMS {
                            let l = labels[substream_index as usize];
                            if (packet_label.wrapping_sub(1) >> 4) == (l.wrapping_sub(1) >> 4) {
                                return MPEGH_UI_PARSE_ERROR;
                            }
                            if l == 0 {
                                break;
                            }
                            substream_index += 1;
                        }
                        if substream_index as usize == MAX_NUMBER_SUBSTREAMS {
                            break 'cfg;
                        }
                    }

                    labels[substream_index as usize] = packet_label;
                }

                MHA_PACTYP_AUDIOSCENEINFO => {
                    if packet_label == 0 || substream_index < 0 {
                        return MPEGH_UI_PARSE_ERROR;
                    }

                    // parse ASI
                    let asi = self.h_ui_manager.get_asi_pointer();
                    if mae_audio_scene_info(asi, h_bs, 2 * 28, substream_index) != TRANSPORTDEC_OK {
                        return MPEGH_UI_PARSE_ERROR;
                    }
                }

                MHA_PACTYP_MARKER => {
                    if packet_length > 2
                        && h_bs.read_bits(8) == 0xE0
                        && h_bs.read_bits(8) == 0x0
                        && is_main_stream
                    {
                        let id_bytes = (packet_length - 2).min(16);

                        for i in 0..16u32 {
                            if i >= 16 - id_bytes {
                                uuid[i as usize] = h_bs.read_bits(8) as u8;
                            } else {
                                uuid[i as usize] = 0;
                            }
                        }

                        marker_found = true;
                    }
                }

                MHA_PACTYP_USERINTERACTION | MHA_PACTYP_LOUDNESS_DRC => {
                    if packet_label == 0 {
                        return MPEGH_UI_PARSE_ERROR;
                    }

                    if is_main_stream {
                        ui_packet_found = true;
                    }
                }

                MHA_PACTYP_MPEGH3DAFRAME => {
                    if packet_label == 0 || substream_index < 0 {
                        return MPEGH_UI_PARSE_ERROR;
                    }

                    frame_found = true;
                }

                _ => {}
            }

            // skip remaining bits
            n_bits = (8 * packet_length as i32).wrapping_sub(n_bits - h_bs.get_valid_bits());
            if n_bits >= 0 {
                h_bs.push_for(n_bits as u32);
            } else {
                return MPEGH_UI_PARSE_ERROR;
            }

            if frame_found {
                break;
            }

            if packet_type != MHA_PACTYP_CRC16 && packet_type != MHA_PACTYP_CRC32 {
                prev_packet_type = packet_type;
            }

            // update position to insert UI/DRC packets
            if packet_type != MHA_PACTYP_MPEGH3DAFRAME
                && packet_type != MHA_PACTYP_AUDIOTRUNCATION
                && packet_type != MHA_PACTYP_CRC16
                && packet_type != MHA_PACTYP_CRC32
                && packet_type != MHA_PACTYPE_EARCON
                && packet_type != MHA_PACTYPE_PCMCONFIG
                && packet_type != MHA_PACTYPE_PCMDATA
            {
                n_bits_ins = h_bs.get_valid_bits();
            }
        }

        // if UUID did not change, reset XML state changed flag to previous state (to prevent new
        // XML output when nothing changed)
        if self.h_ui_manager.get_xml_changed() != 0 && prev_xml_changed == 0 && !substr_switch {
            let mut cur_uuid = [0u8; 16];

            self.h_ui_manager.get_uuid(&mut cur_uuid);

            let uuid_chg = cur_uuid != prev_uuid;
            let crc_chg = self.h_ui_manager.get_asi_pointer().crc_for_compare != prev_asi_crc;

            if !uuid_chg && !crc_chg {
                self.h_ui_manager.reset_xml_changed();
            }
        }

        // check if a frame was found
        if !frame_found {
            return MPEGH_UI_PARSE_ERROR;
        }

        // check if UI packet found and set in/active
        if self.is_active != 0 && ui_packet_found {
            self.is_active = 0;
            self.h_ui_manager.set_is_active(0);
        } else if self.is_active == 0 && !ui_packet_found && self.config_found != 0 {
            self.is_active = 1;
            self.h_ui_manager.set_is_active(1);
        }

        if marker_found && self.is_active != 0 {
            self.h_ui_manager.set_uuid(&uuid, 1);
        }

        // compute byte offset to insert UI/DRC packets
        self.insert_offset = (mhas_length as u32).wrapping_sub((n_bits_ins >> 3) as u32);

        MPEGH_UI_OK
    }

    /// `mpegh_UI_UpdateMHAS(self, mhasBuffer, mhasBufferLength, mhasLength)`: update MHAS
    /// buffer, inserting UI and DRC packets from UI manager. `mhasBufferLength` is
    /// `mhas_buffer.len()`; `mhas_length` is the length of the MHAS data in the buffer (the same
    /// data as passed to the previous [`feed_mhas`](Self::feed_mhas) call) and receives the
    /// updated length.
    ///
    /// Inconsistent lengths (`*mhas_length` larger than the buffer or smaller than the insert
    /// position), which are undefined behaviour in C++, return `MPEGH_UI_INVALID_PARAM`.
    pub fn update_mhas(&mut self, mhas_buffer: &mut [u8], mhas_length: &mut u32) -> MpeghUiError {
        let mut ui_status = UserInteractivityStatus::default();
        let mut drc_status = UiDrcLoudnessStatus::default();
        let mut ui_packet_length = 0;
        let mut drc_packet_length = 0;
        let mut n_bytes: u32 = 0;
        let mut ui_status_changed = 0u8;
        let mut drc_status_changed = 0u8;

        if self.is_active == 0 {
            return MPEGH_UI_NOT_ALLOWED;
        }

        if self.insert_offset == u32::MAX {
            return MPEGH_UI_INVALID_STATE;
        }

        // check for status change
        self.h_ui_manager
            .get_status_changed(Some(&mut ui_status_changed), Some(&mut drc_status_changed));

        let mut insert_ui_packet = self.config_found != 0 || ui_status_changed != 0;
        let insert_drc_packet = self.config_found != 0 || drc_status_changed != 0;

        // without ASI do not insert UI packets
        if self.h_ui_manager.get_asi_pointer().num_groups == 0 {
            insert_ui_packet = false;
        }

        let mut count = Bw(None);

        if insert_ui_packet {
            // get UI status
            let err = self.h_ui_manager.get_interactivity_status(&mut ui_status, None);
            if err != UI_MANAGER_OK {
                return err;
            }

            // get UI packet length
            ui_packet_length = write_ui_packet(&mut count, &ui_status);
            n_bytes += ui_packet_length;

            // get header length
            n_bytes += write_packet_header(
                &mut count,
                MHA_PACTYP_USERINTERACTION as u32,
                self.substream_label[0],
                ui_packet_length,
            );
        }

        if insert_drc_packet {
            // get DRC/loudness status
            let err = self.h_ui_manager.get_drc_loudness_status(&mut drc_status);
            if err != UI_MANAGER_OK {
                return err;
            }

            // get DRC packet length
            drc_packet_length = write_drc_packet(&mut count, &drc_status);
            n_bytes += drc_packet_length;

            // get header length
            n_bytes += write_packet_header(
                &mut count,
                MHA_PACTYP_LOUDNESS_DRC as u32,
                self.substream_label[0],
                drc_packet_length,
            );
        }

        if n_bytes == 0 {
            return MPEGH_UI_OK;
        }

        // check buffer size
        let mhas_buffer_length = mhas_buffer.len() as u64;
        if *mhas_length as u64 + n_bytes as u64 > mhas_buffer_length {
            return MPEGH_UI_BUFFER_TOO_SMALL;
        }
        if self.insert_offset > *mhas_length {
            /* undefined behaviour in C++ */
            mpeghdec_fdk::deviation::hit();
            return MPEGH_UI_INVALID_PARAM;
        }

        // prepare writing of packets
        let off = self.insert_offset as usize;
        mhas_buffer.copy_within(off..*mhas_length as usize, off + n_bytes as usize);
        let size = next_pow2(n_bytes) as usize;
        let mut buf = vec![0u8; size];
        buf[..n_bytes as usize].copy_from_slice(&mhas_buffer[off..off + n_bytes as usize]);
        let mem = BitMem::from_slice(&buf);
        let mut bs = FdkBitStream::default();
        bs.init(&mem, size as u32, 0, FdkBsCfg::BsWriter);
        let mut w = Bw(Some(&mut bs));

        if insert_ui_packet {
            // insert UI packet
            write_packet_header(
                &mut w,
                MHA_PACTYP_USERINTERACTION as u32,
                self.substream_label[0],
                ui_packet_length,
            );
            write_ui_packet(&mut w, &ui_status);
        }

        if insert_drc_packet {
            // insert DRC packet
            write_packet_header(
                &mut w,
                MHA_PACTYP_LOUDNESS_DRC as u32,
                self.substream_label[0],
                drc_packet_length,
            );
            write_drc_packet(&mut w, &drc_status);
        }

        bs.sync_cache();
        mem.read(0, &mut buf);
        mhas_buffer[off..off + n_bytes as usize].copy_from_slice(&buf[..n_bytes as usize]);

        *mhas_length += n_bytes;

        MPEGH_UI_OK
    }

    /// `mpegh_UI_SetPersistenceMemory(self, persistenceMemoryBlock, persistenceMemorySize)`: set
    /// memory block for storing UI persistency data (activates internal persistence handling,
    /// `None` deactivates it). The block is owned by the UI manager until it is replaced; the
    /// size used is `min(len, 65535)` (the minimum recommended size is 512 bytes). The block
    /// can be empty or filled with saved content of a previous session.
    pub fn set_persistence_memory(
        &mut self,
        persistence_memory_block: Option<Vec<u8>>,
    ) -> MpeghUiError {
        let err = self.h_ui_manager.set_persistence_memory(persistence_memory_block);

        match err {
            UI_MANAGER_OK => MPEGH_UI_OK,
            UI_MANAGER_BUFFER_TOO_SMALL => MPEGH_UI_BUFFER_TOO_SMALL,
            UI_MANAGER_OK_BUT_NO_VALID_DATA => MPEGH_UI_OK_BUT_NO_VALID_DATA,
            _ => MPEGH_UI_INVALID_PARAM,
        }
    }

    /// `mpegh_UI_GetPersistenceMemory(self, persistenceMemoryBlock, persistenceMemorySize)`: get
    /// current UI persistency memory block (updates content, call is required before storing
    /// the persistence memory for later reuse). Returns the block (`None` if internal persistency
    /// handling has not been activated) and its size in bytes.
    pub fn get_persistence_memory(&mut self) -> (MpeghUiError, Option<&[u8]>, u16) {
        let (mem, size) = self.h_ui_manager.get_persistence_memory();

        (MPEGH_UI_OK, mem, size)
    }
}
