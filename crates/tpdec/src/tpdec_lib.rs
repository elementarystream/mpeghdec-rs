// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpegTPDec/include/tpdec_lib.h and libMpegTPDec/src/tpdec_lib.cpp.

//! MPEG transport decoder: MHAS parser (ISO/IEC 23008-3 clause 14) and raw MPEG-H access units.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::crc::FdkCrcInfo;

use crate::tp_data::*;
use crate::tpdec_asc::{earcon_info, mpegh3da_config_parse, pcm_data_config};
use crate::tpdec_asi::{asi_reset, check_asi, mae_audio_scene_info};

/// `TRANSPORTDEC_ERROR`
pub type TransportDecError = i32;
/// All fine.
pub const TRANSPORTDEC_OK: TransportDecError = 0;

// Synchronization errors. Wait for new input data and try again.
pub const TPDEC_SYNC_ERROR_START: TransportDecError = 0x100;
/// Out of bits. Provide more bits and try again.
pub const TRANSPORTDEC_NOT_ENOUGH_BITS: TransportDecError = 0x101;
/// No sync was found or sync got lost. Keep trying.
pub const TRANSPORTDEC_SYNC_ERROR: TransportDecError = 0x102;
pub const TPDEC_SYNC_ERROR_END: TransportDecError = 0x103;

// Decode errors. Mostly caused due to bit errors.
pub const TPDEC_DECODE_ERROR_START: TransportDecError = 0x400;
/// Bitstream data showed inconsistencies (wrong syntax).
pub const TRANSPORTDEC_PARSE_ERROR: TransportDecError = 0x401;
/// Unsupported format or feature found in the bitstream data.
pub const TRANSPORTDEC_UNSUPPORTED_FORMAT: TransportDecError = 0x402;
/// CRC error encountered in bitstream data.
pub const TRANSPORTDEC_CRC_ERROR: TransportDecError = 0x403;
pub const TPDEC_DECODE_ERROR_END: TransportDecError = 0x404;

// Fatal errors. Stop immediately on one of these errors!
pub const TPDEC_FATAL_ERROR_START: TransportDecError = 0x200;
/// An unknown error occured.
pub const TRANSPORTDEC_UNKOWN_ERROR: TransportDecError = 0x201;
/// An invalid parameter was passed to a function.
pub const TRANSPORTDEC_INVALID_PARAMETER: TransportDecError = 0x202;
/// The decoder needs to be restarted, since the requiered configuration change cannot be
/// performed.
pub const TRANSPORTDEC_NEED_TO_RESTART: TransportDecError = 0x203;
/// In case of packet based formats: Supplied number of bits exceed the size of the internal bit
/// buffer.
pub const TRANSPORTDEC_TOO_MANY_BITS: TransportDecError = 0x204;
pub const TPDEC_FATAL_ERROR_END: TransportDecError = 0x205;

/// `TPDEC_IS_DECODE_ERROR(err)`
#[inline]
pub fn tpdec_is_decode_error(err: TransportDecError) -> bool {
    (TPDEC_DECODE_ERROR_START..=TPDEC_DECODE_ERROR_END).contains(&err)
}

/// `TPDEC_IS_FATAL_ERROR(err)`
#[inline]
pub fn tpdec_is_fatal_error(err: TransportDecError) -> bool {
    (TPDEC_FATAL_ERROR_START..=TPDEC_FATAL_ERROR_END).contains(&err)
}

/// `TPDEC_PARAM`: parameter identifiers for `transportDec_SetParam()`.
pub type TpdecParam = i32;
/// Delay minimization strategy. 0: none, 1: discard as many frames as possible.
pub const TPDEC_PARAM_MINIMIZE_DELAY: TpdecParam = 1;
/// Enable early config discovery.
pub const TPDEC_PARAM_EARLY_CONFIG: TpdecParam = 2;
/// Ignore buffer fullness.
pub const TPDEC_PARAM_IGNORE_BUFFERFULLNESS: TpdecParam = 3;
/// Set average bit rate for bit stream interruption frame misses estimation.
pub const TPDEC_PARAM_SET_BITRATE: TpdecParam = 4;
/// Reset transport decoder instance status.
pub const TPDEC_PARAM_RESET: TpdecParam = 5;
/// Set data reception burst period in mili seconds.
pub const TPDEC_PARAM_BURST_PERIOD: TpdecParam = 6;
/// Set CICP target layout
pub const TPDEC_PARAM_TARGETLAYOUT: TpdecParam = 7;
/// Force config change for next received config
pub const TPDEC_PARAM_FORCE_CONFIG_CHANGE: TpdecParam = 8;
pub const TPDEC_PARAM_USE_ELEM_SKIPPING: TpdecParam = 9;
/// Enhance synchronization robustness by requiring two valid sync words in expected positions.
pub const TPDEC_PARAM_CHECK_TWO_SYNCS: TpdecParam = 10;

// CELP stuff
pub const MPE: i32 = 0;
pub const RPE: i32 = 1;
pub const FS8KHZ: i32 = 0;
pub const FS16KHZ: i32 = 1;

/// Defintion of flags that can be passed to `transportDecOpen()`
pub const TP_FLAG_MPEG4: u32 = 1;

pub const CSAUDIOSPECIFICCONFIG_SIZE: usize = TPDEC_MAX_TRACKS + 1;

/// Size of the internal bitstream buffer (`65536 * 1`).
pub const TRANSPORTDEC_BUFFER_SIZE: usize = 65536;

pub const MHAS_CONFIG_PRESENT: u32 = 0x001;
pub const MHAS_UI_PRESENT: u32 = 0x002;

pub const SUBSTREAM_STATE_NOT_PRESENT: u32 = 0;
pub const SUBSTREAM_STATE_PRESENT: u32 = 1;
pub const SUBSTREAM_STATE_CFG_CHG: u32 = 2;
pub const SUBSTREAM_STATE_REPLACE: u32 = 3;
pub const SUBSTREAM_STATE_ADD: u32 = 4;
pub const SUBSTREAM_STATE_REMOVE: u32 = 5;

// Flag bitmasks for "flags" member of struct TRANSPORTDEC
pub const TPDEC_SYNCOK: u32 = 1;
pub const TPDEC_MINIMIZE_DELAY: u32 = 2;
pub const TPDEC_IGNORE_BUFFERFULLNESS: u32 = 4;
pub const TPDEC_EARLY_CONFIG: u32 = 8;
pub const TPDEC_LOST_FRAMES_PENDING: u32 = 16;
pub const TPDEC_CONFIG_FOUND: u32 = 32;
pub const TPDEC_USE_ELEM_SKIPPING: u32 = 64;
pub const TPDEC_CHECK_TWO_SYNCS: u32 = 128;

// force config/content change
pub const TPDEC_FORCE_CONFIG_CHANGE: u8 = 1;
pub const TPDEC_FORCE_CONTENT_CHANGE: u8 = 2;

/// How many bits to advance for synchronization search.
const TPDEC_SYNCSKIP: i32 = 8;

/// `transportdec_parser_t` (a union with the single member `mhas`).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TransportDecParser {
    pub buffer_fullness: i32,
    pub end_of_frame: u32,
    pub mhas_labels: [u32; TPDEC_MAX_TRACKS],
    pub substream_state: [u32; TPDEC_MAX_TRACKS],
    pub flags: [u32; TPDEC_MAX_TRACKS],
}

/// `struct TRANSPORTDEC` (`HANDLE_TRANSPORTDEC` is `&mut TransportDec`).
///
/// The callbacks (`CSTpCallBacks`), `pASI` and `pLoudnessInfoSetPosition` are not stored: the
/// functions that use them take a [`TpCallbacks`] argument.
#[derive(StateDigest)]
pub struct TransportDec {
    /// MPEG4 transportDec type.
    pub transport_fmt: TransportType,

    /// Bitstream readers; all of them read from the internal buffer.
    pub bit_stream: [FdkBitStream; TPDEC_MAX_LAYERS],
    /// Internal bitstream data buffer (`bsBuffer`), shared by the bitstreams.
    bs_buffer: BitMem,

    /// Format specific parser structs.
    pub parser: TransportDecParser,

    /// Audio specific config from the last config found. One additional CSAudioSpecificConfig is
    /// used temporarily for parsing.
    pub asc: [CSAudioSpecificConfig; CSAUDIOSPECIFICCONFIG_SIZE],
    /// Controls config change
    pub ctrl_cfg_change: [CCtrlCFGChange; TPDEC_MAX_TRACKS],

    /// Global transport frame reference bit position.
    pub global_frame_pos: u32,
    /// Current access unit start bit position.
    pub access_unit_anchor: [u32; TPDEC_MAX_LAYERS],
    /// Length of current access unit.
    pub au_length: [i32; TPDEC_MAX_LAYERS],
    /// Current number of raw data blocks contained remaining from the current transport frame.
    pub number_of_raw_data_blocks: i32,
    /// Average bit rate used for frame loss estimation.
    pub avg_bit_rate: u32,
    /// Last valid buffer fullness value for frame loss estimation
    pub last_valid_buffer_fullness: u32,
    /// Reminder in division during lost access unit estimation.
    pub remainder: i32,
    /// Estimated missing access units.
    pub missing_access_units: i32,
    /// Data burst period in mili seconds.
    pub burst_period: u32,
    /// Amount of frames that were already hold off due to buffer fullness condition not being
    /// met.
    pub hold_off_frames: u32,
    /// Flags.
    pub flags: u32,
    /// CICP target layout.
    pub target_layout: i32,
}

/// `HANDLE_TRANSPORTDEC`
pub type HandleTransportDec<'a> = &'a mut TransportDec;

/// Arguments of `mpeghPrepareDecode()` that are locals of `transportDec_readHeader()`.
struct PrepareDecodeState<'a> {
    bs_main_stream: &'a FdkBitStream,
    main_stream_present: i32,
    sync_layer_frame_bits: &'a mut i32,
    raw_data_block_length: &'a mut i32,
    au_bit_pos_build_up: i32,
}

impl TransportDec {
    /// `transportDec_Open(transportDecFmt, flags, nrOfLayer)`: open a transport decoder.
    /// Returns `None` for unsupported formats or too many layers.
    pub fn open(transport_fmt: TransportType, _flags: u32, nr_of_layers: u32) -> Option<Box<Self>> {
        match transport_fmt {
            TT_MP4_RAW | TT_MHA_RAW | TT_MHAS | TT_MHAS_PACKETIZED => {}
            _ => return None,
        }

        if nr_of_layers > TPDEC_MAX_LAYERS as u32 {
            return None;
        }

        // All members zero like the calloc()ed C struct.
        let mut h = Box::new(TransportDec {
            transport_fmt,
            bit_stream: core::array::from_fn(|_| FdkBitStream::default()),
            bs_buffer: BitMem::new(TRANSPORTDEC_BUFFER_SIZE),
            parser: TransportDecParser::default(),
            asc: [CSAudioSpecificConfig::default(); CSAUDIOSPECIFICCONFIG_SIZE],
            ctrl_cfg_change: [CCtrlCFGChange::default(); TPDEC_MAX_TRACKS],
            global_frame_pos: 0,
            access_unit_anchor: [0; TPDEC_MAX_LAYERS],
            au_length: [0; TPDEC_MAX_LAYERS],
            number_of_raw_data_blocks: 0,
            avg_bit_rate: 0,
            last_valid_buffer_fullness: 0,
            remainder: 0,
            missing_access_units: 0,
            burst_period: 0,
            hold_off_frames: 0,
            flags: 0,
            target_layout: 0,
        });

        // Create bitstream
        for i in 0..nr_of_layers as usize {
            h.bit_stream[i].init(
                &h.bs_buffer,
                TRANSPORTDEC_BUFFER_SIZE as u32,
                0,
                FdkBsCfg::BsReader,
            );
        }
        h.burst_period = 0;

        Some(h)
    }

    /// A copy of the content of the internal bitstream buffer (`bsBuffer`).
    pub fn bs_buffer(&self) -> Vec<u8> {
        self.bs_buffer.to_vec()
    }

    /// Sets the content of the internal bitstream buffer (not in C): `content` at the start of
    /// the buffer, zeros after it. The positions of the bitstream readers are not changed.
    ///
    /// For the chunked decoding (`mpeghdec::chunked`): with packet based formats every access
    /// unit is written to the start of the buffer, so the bytes after the current access unit
    /// are left over from earlier (longer) access units. A decoder that starts in the middle of a
    /// stream sets them to what a decoder that decoded the whole stream would have, so that the
    /// buffers (which are part of the state digest) are identical.
    pub fn set_bs_buffer(&mut self, content: &[u8]) {
        let n = content.len().min(self.bs_buffer.len());
        self.bs_buffer.write(0, &content[..n]);
        self.bs_buffer.fill_from(n, 0);
    }

    /// `transportDec_OutOfBandConfig(hTp, conf, length, layer)`: configure the transport decoder
    /// with a binary coded mpegh3daConfig (`conf`, `length = conf.len()`). The configuration
    /// callbacks are called as well.
    pub fn out_of_band_config(
        &mut self,
        conf: &[u8],
        layer: u32,
        cb: &mut dyn TpCallbacks,
    ) -> TransportDecError {
        let layer = layer as usize;
        let length = conf.len() as u32;
        let mut err = TRANSPORTDEC_OK;

        let mut f_config_found = 0;

        let mut config_changed: u8 = 0;
        let mut config_mode: u8 = AC_CM_DET_CFG_CHANGE;

        let mut tmp_conf = [0u8; 1024];
        if length > 1024 {
            return TRANSPORTDEC_UNSUPPORTED_FORMAT;
        }
        tmp_conf[..conf.len()].copy_from_slice(conf);
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(&tmp_conf), 1024, length << 3, FdkBsCfg::BsReader);

        for i in 0..2 {
            if i > 0 {
                let vb = bs.get_valid_bits();
                bs.push_back(((length * 8) as i32).wrapping_sub(vb) as u32);
                config_mode = AC_CM_ALLOC_MEM;
            }

            // config transport decoder
            match self.transport_fmt {
                TT_MHA_RAW | TT_MHAS | TT_MHAS_PACKETIZED => {
                    f_config_found = 1;
                    err = mpegh3da_config_parse(
                        &mut self.asc[TPDEC_MAX_TRACKS],
                        &mut bs,
                        1,
                        cb,
                        config_mode,
                        config_changed,
                        self.target_layout,
                        0,
                    );
                    self.stamp_speakers_owner(TPDEC_MAX_TRACKS);
                    if err == TRANSPORTDEC_OK {
                        self.asc[layer] = self.asc[TPDEC_MAX_TRACKS];
                        let err_c = self.update_config_asc_flag(layer, cb, None);
                        if err_c != 0 {
                            err = TRANSPORTDEC_PARSE_ERROR;
                        }
                    }

                    if config_mode & AC_CM_DET_CFG_CHANGE != 0
                        && self.ctrl_cfg_change[layer].mpegh3da_config[..conf.len()] != *conf
                    {
                        self.asc[layer].aac_config_changed |= 1;
                    }

                    self.ctrl_cfg_change[layer].mpegh3da_config_len = length;
                    self.ctrl_cfg_change[layer].mpegh3da_config[..conf.len()].copy_from_slice(conf);
                }
                _ => {
                    err = TRANSPORTDEC_UNSUPPORTED_FORMAT;
                }
            }

            if err == TRANSPORTDEC_OK
                && i == 0
                && (self.asc[layer].aac_config_changed != 0
                    || self.asc[layer].sbr_config_changed != 0
                    || self.asc[layer].sac_config_changed != 0)
            {
                config_changed = 1;
                let err_c = self.call_free_mem(cb, layer);
                if err_c != 0 {
                    err = TRANSPORTDEC_PARSE_ERROR;
                }
            }

            // if an error is detected terminate config parsing to avoid that an invalid config is
            // accepted in the second pass
            if err != TRANSPORTDEC_OK {
                break;
            }
        }

        if err == TRANSPORTDEC_OK && f_config_found != 0 {
            self.flags |= TPDEC_CONFIG_FOUND;
        }

        err
    }

    /// Calls `cbUpdateConfig(data, &asc[k], mode, configChanged)` and applies the transport
    /// decoder parameters the core decoder set during the callback
    /// ([`TpCallbacks::take_pending_param`]).
    fn call_update_config(
        &mut self,
        cb: &mut dyn TpCallbacks,
        k: usize,
        mode: u8,
        changed: &mut u8,
    ) -> i32 {
        let r = cb.update_config(&self.asc, k, mode, changed);
        while let Some((param, value)) = cb.take_pending_param() {
            self.set_param(param, value);
        }
        r
    }

    /// Calls `cbFreeMem(data, &asc[k])`.
    fn call_free_mem(&mut self, cb: &mut dyn TpCallbacks, k: usize) -> i32 {
        let r = cb.free_mem(&self.asc, k);
        while let Some((param, value)) = cb.take_pending_param() {
            self.set_param(param, value);
        }
        r
    }

    /// Calls `cbUpdateConfig(data, &asc[layer], mode, &asc[layer].AacConfigChanged)`, with mode
    /// `asc[layer].configMode | extra_mode`.
    fn update_config_asc_flag(
        &mut self,
        layer: usize,
        cb: &mut dyn TpCallbacks,
        extra_mode: Option<u8>,
    ) -> i32 {
        let mode = self.asc[layer].config_mode | extra_mode.unwrap_or(0);
        let mut changed = self.asc[layer].aac_config_changed;
        let r = self.call_update_config(cb, layer, mode, &mut changed);
        self.asc[layer].aac_config_changed = changed;
        r
    }

    /// `transportDec_InBandConfig(hTp, newConfig, newConfigLength, buildUpStatus, configChanged,
    /// layer)`: configure the transport decoder via a binary coded mpegh3daConfig from an
    /// AudioPreRoll. `configChanged` is set to 1 if the new config differs from the previous one.
    ///
    /// `new_config` is the caller's config buffer (`UCHAR config[TP_USAC_MAX_CONFIG_LEN]` in the
    /// core decoder); like in C, a bitstream with a buffer size of `TP_USAC_MAX_CONFIG_LEN` bytes
    /// reads it, so its first `TP_USAC_MAX_CONFIG_LEN` bytes are used (missing bytes read as 0).
    /// `new_config_length` must not exceed `TP_USAC_MAX_CONFIG_LEN`.
    pub fn in_band_config(
        &mut self,
        new_config: &[u8],
        new_config_length: u32,
        build_up_status: u8,
        config_changed: &mut u8,
        layer: u32,
        cb: &mut dyn TpCallbacks,
    ) -> TransportDecError {
        let layer = layer as usize;
        let mut err = TRANSPORTDEC_OK;
        let mut f_config_found = 0;
        let config_mode: u8 = AC_CM_ALLOC_MEM;
        let len = new_config_length as usize;

        let mut cfg = [0u8; TP_USAC_MAX_CONFIG_LEN];
        let n = new_config.len().min(TP_USAC_MAX_CONFIG_LEN);
        cfg[..n].copy_from_slice(&new_config[..n]);
        let mut bs = FdkBitStream::default();
        bs.init(
            &BitMem::from_slice(&cfg),
            TP_USAC_MAX_CONFIG_LEN as u32,
            new_config_length << 3,
            FdkBsCfg::BsReader,
        );

        if self.ctrl_cfg_change[layer].flush_status == TPDEC_FLUSH_OFF
            && self.ctrl_cfg_change[layer].build_up_status != TPDEC_MPEGH_BUILD_UP_IDLE_IN_BAND
            && self.asc[0].m_aot == AOT_MPEGH3DA
        {
            if layer == 0 {
                let l = self.ctrl_cfg_change[layer].mpegh3da_config_len as i32;
                self.disregard_loudness_change(&mut bs, l, cb);
            }
            if self.ctrl_cfg_change[layer].mpegh3da_config_len == new_config_length
                && cfg[..len] == self.ctrl_cfg_change[layer].mpegh3da_config[..len]
            {
                *config_changed = 0;
                return err;
            }
        }

        {
            let ctrl = &mut self.ctrl_cfg_change[layer];
            if ctrl.flush_status == TPDEC_FLUSH_OFF
                && ctrl.build_up_status != TPDEC_MPEGH_BUILD_UP_IDLE_IN_BAND
            {
                ctrl.flush_cnt = 0;
                ctrl.build_up_cnt = 0;
                ctrl.build_up_status = TPDEC_BUILD_UP_OFF;
                if self.asc[0].m_aot == AOT_MPEGH3DA {
                    ctrl.flush_status = TPDEC_MPEGH_DASH_IPF_ATSC_FLUSH_ON;

                    if self.number_of_raw_data_blocks != 0 {
                        err = TRANSPORTDEC_PARSE_ERROR;
                    }
                    self.number_of_raw_data_blocks = 1;

                    // Skip truncation callback if explicit and implicit config are different.
                    if build_up_status != TPDEC_MPEGH_BUILD_UP_ON {
                        if ctrl.truncation_present == 0 {
                            if cb.has_truncation_msg() {
                                cb.truncation_msg(0, 2);
                            }
                            ctrl.truncation_present = 1;
                        }
                    } else {
                        // Skip flushing if explicit and implicit config are different. Continue
                        // with idle status
                        ctrl.flush_cnt = TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES;
                    }
                }
            }

            if ctrl.flush_status == TPDEC_MPEGH_DASH_IPF_ATSC_FLUSH_ON
                || ctrl.flush_status == TPDEC_USAC_DASH_IPF_FLUSH_ON
            {
                let mut counter: i8 = 0;
                if self.asc[0].m_aot == AOT_MPEGH3DA {
                    counter = TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES;
                }
                if ctrl.flush_cnt >= counter {
                    ctrl.flush_cnt = 0;
                    ctrl.flush_status = TPDEC_FLUSH_OFF;
                    ctrl.force_cfg_change = 0;
                    if self.asc[0].m_aot == AOT_MPEGH3DA {
                        ctrl.build_up_cnt = TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES;
                        ctrl.build_up_status = TPDEC_MPEGH_BUILD_UP_IDLE_IN_BAND;
                        ctrl.truncation_present = 0;
                        if self.number_of_raw_data_blocks != 1 {
                            err = TRANSPORTDEC_PARSE_ERROR;
                        }
                    }
                }

                // Activate flush mode. After that continue with build up mode in core
                if cb.has_ctrl_cfg_change() && cb.ctrl_cfg_change(ctrl) != 0 {
                    err = TRANSPORTDEC_PARSE_ERROR;
                }

                if ctrl.flush_status == TPDEC_MPEGH_DASH_IPF_ATSC_FLUSH_ON
                    || ctrl.flush_status == TPDEC_USAC_DASH_IPF_FLUSH_ON
                {
                    ctrl.flush_cnt = ctrl.flush_cnt.wrapping_add(1);
                    return err;
                }
            } else if ctrl.build_up_status == TPDEC_MPEGH_BUILD_UP_IDLE_IN_BAND {
                ctrl.build_up_status = TPDEC_MPEGH_BUILD_UP_ON_IN_BAND;
                if cb.has_ctrl_cfg_change() && cb.ctrl_cfg_change(ctrl) != 0 {
                    err = TRANSPORTDEC_PARSE_ERROR;
                }

                self.number_of_raw_data_blocks = 0;
                *config_changed = 1;
                return err;
            }
        }

        if self.asc[0].m_aot == AOT_MPEGH3DA {
            *config_changed = 1;
            // free memory for config change
            let err_c = self.call_free_mem(cb, layer);
            if err_c != 0 {
                err = TRANSPORTDEC_PARSE_ERROR;
            }

            // config transport decoder
            if err == TRANSPORTDEC_OK {
                match self.transport_fmt {
                    TT_MHA_RAW | TT_MHAS | TT_MHAS_PACKETIZED => {
                        f_config_found = 1;
                        err = mpegh3da_config_parse(
                            &mut self.asc[TPDEC_MAX_TRACKS],
                            &mut bs,
                            1,
                            cb,
                            config_mode,
                            *config_changed,
                            self.target_layout,
                            0,
                        );
                        self.stamp_speakers_owner(TPDEC_MAX_TRACKS);
                        if err == TRANSPORTDEC_OK {
                            self.asc[layer] = self.asc[TPDEC_MAX_TRACKS];
                            let err_c =
                                self.update_config_asc_flag(layer, cb, Some(AC_CM_LAST_SUBSTREAM));
                            if err_c != 0 {
                                err = TRANSPORTDEC_PARSE_ERROR;
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        // save new config
        if err == TRANSPORTDEC_OK {
            if self.asc[0].m_aot == AOT_MPEGH3DA {
                self.ctrl_cfg_change[layer].mpegh3da_config_len = new_config_length;
                self.ctrl_cfg_change[layer].mpegh3da_config[..len].copy_from_slice(&cfg[..len]);
            }
        } else {
            self.number_of_raw_data_blocks = 0;

            // If parsing error while config found, clear ctrlCFGChange-struct
            let ctrl = &mut self.ctrl_cfg_change[layer];
            ctrl.mpegh3da_config_len = 1;
            ctrl.flush_cnt = 0;
            ctrl.flush_status = TPDEC_FLUSH_OFF;
            ctrl.build_up_cnt = 0;
            ctrl.build_up_status = TPDEC_BUILD_UP_OFF;
            ctrl.cfg_changed = 0;
            ctrl.content_changed = 0;
            ctrl.force_cfg_change = 0;
            ctrl.truncation_present = 0;

            if cb.has_ctrl_cfg_change() {
                cb.ctrl_cfg_change(ctrl);
            }
        }

        if err == TRANSPORTDEC_OK && f_config_found != 0 {
            self.flags |= TPDEC_CONFIG_FOUND;
        }

        err
    }

    /// `transportDec_FillData(hTp, pBuffer, bufferSize, pBytesValid, layer)`: copy as much data as
    /// possible from the end of `buffer` (`bytes_valid` bytes are still unread) into the internal
    /// input buffer.
    pub fn fill_data(
        &mut self,
        buffer: &[u8],
        bytes_valid: &mut u32,
        layer: i32,
    ) -> TransportDecError {
        if layer < 0 || layer >= TPDEC_MAX_LAYERS as i32 {
            return TRANSPORTDEC_INVALID_PARAMETER;
        }
        let layer = layer as usize;

        // set bitbuffer shortcut
        let hbs = &mut self.bit_stream[layer];

        if tt_is_packet(self.transport_fmt) {
            if self.number_of_raw_data_blocks == 0 {
                hbs.reset_bitbuffer(FdkBsCfg::BsReader);
                hbs.feed_buffer(buffer, bytes_valid);
                if *bytes_valid != 0 {
                    return TRANSPORTDEC_TOO_MANY_BITS;
                }
            }
        } else {
            // ... else feed bitbuffer with new stream data (append).

            if *bytes_valid == 0 {
                // nothing to do
                return TRANSPORTDEC_OK;
            } else {
                let bv = *bytes_valid;
                hbs.feed_buffer(buffer, bytes_valid);

                if self.number_of_raw_data_blocks > 0 {
                    let fed = bv.wrapping_sub(*bytes_valid).wrapping_mul(8);
                    self.global_frame_pos = self.global_frame_pos.wrapping_add(fed);
                    if self.transport_fmt == TT_MHAS {
                        self.parser.end_of_frame = self.parser.end_of_frame.wrapping_add(fed);
                    }
                    self.access_unit_anchor[layer] = hbs.get_valid_bits() as u32;
                }
            }
        }

        TRANSPORTDEC_OK
    }

    /// `transportDec_GetBitstream(hTp, layer)`: the bitstream of the current access unit of
    /// `layer`.
    pub fn get_bitstream(&mut self, layer: u32) -> &mut FdkBitStream {
        &mut self.bit_stream[layer as usize]
    }

    /// `transportDec_GetFormat(hTp)`
    pub fn get_format(&self) -> TransportType {
        self.transport_fmt
    }

    /// The audio specific config of `layer` (`hTp->asc[layer]`), as passed to the callbacks.
    pub fn asc(&self, layer: usize) -> &CSAudioSpecificConfig {
        &self.asc[layer]
    }

    /// `hTp->asc[layer].m_sc.m_usacConfig.m_signalGroupType[grp].speakers`: follows the pointer
    /// into the config it points to, which after `asc[layer] = asc[TPDEC_MAX_TRACKS]` is
    /// `asc[TPDEC_MAX_TRACKS]` (see [`CSSignalGroup::speakers_owner`]). Index the result with
    /// `0..count`.
    pub fn signal_group_speakers(&self, layer: usize, grp: usize) -> &[MpeghSpeaker] {
        asc_signal_group_speakers(&self.asc, layer, grp)
    }

    /// Records that the speaker pointers set by a config parse into `asc[k]` point into
    /// `asc[k]`.
    fn stamp_speakers_owner(&mut self, k: usize) {
        for sg in self.asc[k].m_sc.m_signal_group_type.iter_mut() {
            if sg.speakers != SpeakerRef::None {
                sg.speakers_owner = k as u8;
            }
        }
    }

    /// `transportDec_AdjustEndOfAccessUnit()`: adjust bit stream position at the end of an
    /// access unit.
    fn adjust_end_of_access_unit(&mut self) -> TransportDecError {
        let err = TRANSPORTDEC_OK;
        let end_of_frame = self.parser.end_of_frame;
        let hbs = &mut self.bit_stream[0];

        match self.transport_fmt {
            TT_MHAS | TT_MHAS_PACKETIZED => {
                let offset = hbs.get_valid_bits().wrapping_sub(end_of_frame as i32);
                if offset != 0 {
                    hbs.push_bi_directional(offset);
                }
            }
            _ => {}
        }

        err
    }

    /// `disregardLoudnessChange()`: enable loudness change on the fly (without config change) if
    /// only the values in loudnessInfoSet change, by copying the new loudnessInfoSet into the
    /// reference config before comparison.
    fn disregard_loudness_change(
        &mut self,
        hbs: &mut FdkBitStream,
        mhas_packet_length: i32,
        cb: &mut dyn TpCallbacks,
    ) {
        let pos = *cb.loudness_info_set_position();
        let loudness_start_bit = pos[0].wrapping_sub(pos[1]);
        let loudness_start_byte = loudness_start_bit >> 3;
        let loudness_payload_length = pos[2];
        let mhas_substream = 0;

        if loudness_start_bit <= 0
            || loudness_payload_length <= 0
            || loudness_start_byte.wrapping_add(loudness_payload_length) > mhas_packet_length
        {
            return;
        }

        let start_pos = hbs.get_valid_bits();
        hbs.push_for(loudness_start_bit as u32);
        let usac_config_ext_type = hbs.escaped_value(4, 8, 16);
        let usac_config_ext_length = hbs.escaped_value(4, 8, 16) as i32;
        if usac_config_ext_type == ID_CONFIG_EXT_LOUDNESS_INFO as u32
            && usac_config_ext_length == pos[2]
        {
            let ctrl = &mut self.ctrl_cfg_change[mhas_substream];
            let sb = loudness_start_byte as usize;
            // disregard loudnessInfoSet in comparison by replacing it in the reference in advance
            let vb = hbs.get_valid_bits();
            hbs.push_back(start_pos.wrapping_sub(vb) as u32);
            hbs.push_for((loudness_start_byte << 3) as u32);

            let this_byte = hbs.read_bits(8) as u8;
            let mut bitmask: u8 = ((1u32 << (loudness_start_bit & 0x7)) - 1) as u8;
            ctrl.set_config_byte(sb, ctrl.config_byte(sb) & !bitmask);
            ctrl.set_config_byte(sb, ctrl.config_byte(sb) | (this_byte & bitmask));
            let mut i = 1usize;
            while (i as i32) < usac_config_ext_length + 1 {
                ctrl.set_config_byte(sb + i, hbs.read_bits(8) as u8);
                i += 1;
            }
            let this_byte = hbs.read_bits(8) as u8;
            bitmask = !bitmask;
            ctrl.set_config_byte(sb + i, ctrl.config_byte(sb + i) & !bitmask);
            ctrl.set_config_byte(sb + i, ctrl.config_byte(sb + i) | (this_byte & bitmask));
        }
        let vb = hbs.get_valid_bits();
        hbs.push_back(start_pos.wrapping_sub(vb) as u32);
    }

    /// `mpeghPrepareDecode()`: returns -1 to bail out, 1 to stop MHAS packet parsing, 0 else.
    fn mpegh_prepare_decode(
        &mut self,
        hbs: &mut FdkBitStream,
        tt: TransportType,
        st: PrepareDecodeState,
        mhas_packet_length: i32,
        nbits: i32,
    ) -> i32 {
        if tt == TT_MHAS {
            // The following code is present twice.
            if self.ctrl_cfg_change[0].build_up_status == TPDEC_MPEGH_BUILD_UP_ON {
                if st.au_bit_pos_build_up != -1 {
                    let vb = hbs.get_valid_bits();
                    hbs.push_back(st.au_bit_pos_build_up.wrapping_sub(vb) as u32);
                    return -1;
                } else {
                    // Something went wrong during build up, reset.
                    for c in self.ctrl_cfg_change.iter_mut() {
                        c.build_up_status = TPDEC_MPEGH_BUILD_UP_IDLE;
                    }
                }
            }
            if st.main_stream_present != 0 {
                // Restore bit stream state to main stream
                *hbs = st.bs_main_stream.clone();
                *st.raw_data_block_length = self.au_length[0];
                *st.sync_layer_frame_bits = self.au_length[0];
            }
            // The preceding code is present twice.
        } else {
            // The following code is present twice.
            if self.ctrl_cfg_change[0].build_up_status == TPDEC_MPEGH_BUILD_UP_ON {
                if st.au_bit_pos_build_up != -1 {
                    let vb = hbs.get_valid_bits();
                    hbs.push_back(st.au_bit_pos_build_up.wrapping_sub(vb) as u32);
                    return -1;
                } else {
                    // Something went wrong during build up, reset.
                    for c in self.ctrl_cfg_change.iter_mut() {
                        c.build_up_status = TPDEC_MPEGH_BUILD_UP_IDLE;
                    }
                }
            } else {
                // Restore bit stream state to main stream
                if st.main_stream_present != 0 {
                    *hbs = st.bs_main_stream.clone();
                    self.parser.end_of_frame =
                        nbits.wrapping_sub(mhas_packet_length.wrapping_mul(8)) as u32;
                    *st.raw_data_block_length = self.au_length[0];
                    *st.sync_layer_frame_bits = self.au_length[0];
                    return 1;
                }
            }
            // The preceding code is present twice.
        }
        0
    }

    /// `transportDec_readHeader()` on `bitStream[bs_idx]`. `raw_data_block_length` is in/out;
    /// the other `Option` arguments correspond to the nullable C pointers.
    fn read_header(
        &mut self,
        bs_idx: usize,
        _sync_length: i32,
        _ignore_buffer_fullness: i32,
        p_raw_data_block_length: &mut i32,
        pf_traverse_more_frames: Option<&mut i32>,
        p_sync_layer_frame_bits: Option<&mut i32>,
        pf_config_found: Option<&mut i32>,
        p_header_bits: Option<&mut i32>,
        cb: &mut dyn TpCallbacks,
    ) -> TransportDecError {
        let mut err = TRANSPORTDEC_OK;
        let mut raw_data_block_length = *p_raw_data_block_length;
        let f_traverse_more_frames = pf_traverse_more_frames.as_deref().copied().unwrap_or(0);
        let mut sync_layer_frame_bits = p_sync_layer_frame_bits.as_deref().copied().unwrap_or(0);
        let mut f_config_found = pf_config_found.as_deref().copied().unwrap_or(0);

        // `hBs` points to bitStream[bs_idx]; it is worked on as a copy that is stored back at the
        // end. Writes to bitStream[bs_idx] in between are mirrored into the copy.
        let mut hbs = self.bit_stream[bs_idx].clone();

        let start_pos = hbs.get_valid_bits();

        'bail: {
            match self.transport_fmt {
                TT_MHAS | TT_MHAS_PACKETIZED => {
                    let mut mhas_packet_type: MhaPactyp;
                    let mut crc: u32 = 0;
                    let mut crc_type: i32 = -1; // -1: none, 0: 16 bit, 1: 32 bits
                    let mut nbits: i32;
                    let mut main_stream_present = 0;
                    let mut crc_data = FdkCrcInfo::default();
                    let mut bs_main_stream = FdkBitStream::default();
                    let mut earcon_config = EarconConfig::default();
                    let mut nbits3da_cfg: i32 = 0;
                    let mut nbits_mhas_main: i32 = 0;
                    let mut au_bit_pos_build_up: i32 = -1;
                    let mut mhas_label_present = [0u8; TPDEC_MAX_TRACKS];
                    let mut start_up = 0;
                    let mut force_cfg_change = 0;
                    let mut last_substream_index: i32 = 0;
                    let mut goto_bail = 0;

                    for i in 0..TPDEC_MAX_TRACKS {
                        // Invalidate all streams
                        self.au_length[i] = 0;
                        mhas_label_present[i] = 0;
                    }
                    // Reset secondary streams
                    for i in 1..TPDEC_MAX_TRACKS {
                        self.bit_stream[i].reset_bitbuffer(FdkBsCfg::BsReader);
                        if i == bs_idx {
                            hbs.reset_bitbuffer(FdkBsCfg::BsReader);
                        }
                    }

                    // force config change if a side-stream starts or ends
                    {
                        nbits = hbs.get_valid_bits();
                        let mut config_found = 0;
                        let mut frame_found = 0;
                        let mut labels = [0u32; TPDEC_MAX_TRACKS];
                        let mut frame_present = [0i32; TPDEC_MAX_TRACKS];
                        let mut i: usize = 0;

                        while hbs.get_valid_bits() > 0 {
                            mhas_packet_type = hbs.escaped_value(3, 8, 8) as i32;
                            let mhas_packet_label = hbs.escaped_value(2, 8, 32);
                            let mhas_packet_length = hbs.escaped_value(11, 24, 24) as i32;

                            if frame_found != 0 && mhas_packet_label <= 16 {
                                break;
                            }

                            if mhas_packet_type == MHA_PACTYP_MPEGH3DACFG {
                                // check packet label is valid and main stream is first
                                if mhas_packet_label == 0 || (i == 0 && mhas_packet_label > 16) {
                                    err = TRANSPORTDEC_SYNC_ERROR;
                                    break 'bail;
                                }

                                // check for multiple labels within same sub-stream
                                for j in 0..i {
                                    if (mhas_packet_label.wrapping_sub(1) >> 4)
                                        == (labels[j].wrapping_sub(1) >> 4)
                                    {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                        break 'bail;
                                    }
                                }

                                if i < TPDEC_MAX_TRACKS {
                                    config_found = 1;
                                    labels[i] = mhas_packet_label;
                                    i += 1;
                                }
                            } else if mhas_packet_type == MHA_PACTYP_MPEGH3DAFRAME {
                                frame_found = 1;
                                for j in 0..TPDEC_MAX_TRACKS {
                                    let l = if config_found != 0 {
                                        labels[j]
                                    } else {
                                        self.parser.mhas_labels[j]
                                    };
                                    if mhas_packet_label == l {
                                        frame_present[j] = 1;
                                        last_substream_index = j as i32;
                                        break;
                                    }
                                }
                            }

                            hbs.push_for(mhas_packet_length.wrapping_mul(8) as u32);
                        }

                        let p = &mut self.parser;
                        if config_found != 0 {
                            // check for substream changes
                            for i in (0..TPDEC_MAX_TRACKS).rev() {
                                if p.mhas_labels[i] == 0 && labels[i] != 0 {
                                    p.substream_state[i] = SUBSTREAM_STATE_ADD;
                                } else if p.mhas_labels[i] != 0 && labels[i] == 0 {
                                    p.substream_state[i] = SUBSTREAM_STATE_REMOVE;
                                } else if p.mhas_labels[i] != labels[i] {
                                    if (p.mhas_labels[i].wrapping_sub(1) >> 4)
                                        == (labels[i].wrapping_sub(1) >> 4)
                                    {
                                        p.substream_state[i] = SUBSTREAM_STATE_CFG_CHG;
                                    } else {
                                        p.substream_state[i] = SUBSTREAM_STATE_REPLACE;
                                    }
                                }

                                p.mhas_labels[i] = labels[i];
                            }

                            // force config change in case of new/lost substreams
                            if self.ctrl_cfg_change[0].flush_status == TPDEC_FLUSH_OFF
                                && self.ctrl_cfg_change[0].build_up_status == TPDEC_BUILD_UP_OFF
                            {
                                for i in 0..TPDEC_MAX_TRACKS {
                                    if p.substream_state[i] > SUBSTREAM_STATE_PRESENT {
                                        force_cfg_change = 1;
                                        if p.substream_state[i] != SUBSTREAM_STATE_REMOVE {
                                            p.substream_state[i] = SUBSTREAM_STATE_PRESENT;
                                        }
                                    }
                                }
                            }
                            // remove lost substreams
                            else if self.ctrl_cfg_change[0].build_up_status
                                == TPDEC_MPEGH_BUILD_UP_IDLE
                            {
                                for i in (0..TPDEC_MAX_TRACKS).rev() {
                                    if p.substream_state[i] == SUBSTREAM_STATE_REMOVE {
                                        p.substream_state[i] = SUBSTREAM_STATE_NOT_PRESENT;
                                    }
                                }
                            }
                        }

                        // check for side-streams lost between RAPs
                        for i in 1..TPDEC_MAX_TRACKS {
                            if self.parser.mhas_labels[i] != 0 && frame_present[i] == 0 {
                                // remove lost side-stream; all subsequent side-streams are also
                                // removed (added again at next RAP; keeping them would need defrag
                                // code with lots of memmoves to close the gap)
                                for j in (i..TPDEC_MAX_TRACKS).rev() {
                                    if self.parser.substream_state[j] == SUBSTREAM_STATE_PRESENT {
                                        self.asc[j].config_mode |= 0x08;
                                        let err_c = self.call_free_mem(cb, j);
                                        self.asc[j].config_mode &= !0x08;

                                        if err_c != 0 {
                                            err = TRANSPORTDEC_UNKOWN_ERROR;
                                        }

                                        self.parser.mhas_labels[j] = 0;
                                        self.parser.substream_state[j] =
                                            SUBSTREAM_STATE_NOT_PRESENT;
                                    }
                                }
                            }
                        }

                        if force_cfg_change != 0 {
                            for i in 0..TPDEC_MAX_TRACKS {
                                if self.parser.mhas_labels[i] > 0 {
                                    self.ctrl_cfg_change[i].force_cfg_change =
                                        TPDEC_FORCE_CONTENT_CHANGE;
                                }
                            }
                        } else if self.ctrl_cfg_change[0].force_cfg_change != 0 {
                            for i in 1..TPDEC_MAX_TRACKS {
                                if self.parser.mhas_labels[i] > 0 {
                                    self.ctrl_cfg_change[i].force_cfg_change =
                                        self.ctrl_cfg_change[0].force_cfg_change;
                                }
                            }
                        }

                        let vb = hbs.get_valid_bits();
                        hbs.push_back(nbits.wrapping_sub(vb) as u32);
                        mhas_packet_type = MHA_PACTYP_NONE;
                    }

                    loop {
                        let mut mhas_substream: i32;

                        let nbits_mhas_current = hbs.get_valid_bits();
                        if nbits_mhas_current <= 0 {
                            self.parser.end_of_frame = 0;
                            err = TRANSPORTDEC_NOT_ENOUGH_BITS;
                            break;
                        }
                        self.parser.end_of_frame = nbits_mhas_current as u32;

                        let mhas_packet_type_prev = mhas_packet_type;
                        mhas_packet_type = hbs.escaped_value(3, 8, 8) as i32;
                        let mut mhas_packet_label = hbs.escaped_value(2, 8, 32);
                        let mhas_packet_length = hbs.escaped_value(11, 24, 24) as i32;

                        nbits = hbs.get_valid_bits();

                        if goto_bail != 0
                            && mhas_packet_type != MHA_PACTYP_MPEGH3DACFG
                            && mhas_packet_type != MHA_PACTYP_CRC16
                            && mhas_packet_type != MHA_PACTYP_CRC32
                        {
                            let vb = hbs.get_valid_bits();
                            hbs.push_back(nbits_mhas_main.wrapping_sub(vb) as u32);
                            break 'bail;
                        }

                        // sanity check if read too many bits
                        if nbits < 0 {
                            err = TRANSPORTDEC_NOT_ENOUGH_BITS;
                            break 'bail;
                        }

                        // Packet length sanity checks
                        if mhas_packet_length > TRANSPORTDEC_BUFFER_SIZE as i32 {
                            err = TRANSPORTDEC_SYNC_ERROR;
                            break 'bail;
                        }
                        if mhas_packet_length * 8 > nbits {
                            err = TRANSPORTDEC_NOT_ENOUGH_BITS;
                            break 'bail;
                        }

                        // Verify CRC if present
                        if crc_type >= 0 {
                            const POLY: [u32; 2] = [0x8021, 0x04c1_1db7];
                            const LENGTH: [u32; 2] = [16, 32];
                            const STARTV: [u32; 2] = [0xffff, 0xffff_ffff];
                            let t = crc_type as usize;
                            crc_data.init(POLY[t], STARTV[t], LENGTH[t]);
                            let reg = crc_data.start_reg(&mut hbs, mhas_packet_length * 8);
                            hbs.push_for((mhas_packet_length * 8) as u32);
                            crc_data.end_reg(&mut hbs, reg);
                            if crc_data.get_crc() as u32 != crc {
                                err = TRANSPORTDEC_SYNC_ERROR;
                                break 'bail;
                            }
                            hbs.push_back((mhas_packet_length * 8) as u32);
                            crc_type = -1; // Reset CRC type
                        }

                        // Redirect EARCON packets
                        if mhas_packet_label == 2049
                            && (mhas_packet_type == MHA_PACTYPE_EARCON
                                || mhas_packet_type == MHA_PACTYPE_PCMCONFIG
                                || mhas_packet_type == MHA_PACTYPE_PCMDATA
                                || mhas_packet_type == MHA_PACTYP_AUDIOTRUNCATION)
                        {
                            mhas_packet_label = 0;
                        }

                        // Count MHA_PACTYP_MPEGH3DACFG of secondary streams
                        if mhas_packet_label > 16 && mhas_packet_label != 0 {
                            // Lookup mhas sub stream index or find empty sub stream entry
                            mhas_substream = -1;
                            for i in (1..TPDEC_MAX_TRACKS).rev() {
                                if self.parser.mhas_labels[i] == mhas_packet_label {
                                    mhas_substream = i as i32;
                                    break;
                                }
                                if self.parser.mhas_labels[i] == 0 {
                                    mhas_substream = i as i32;
                                }
                            }

                            if mhas_substream >= TPDEC_MAX_TRACKS as i32 || mhas_substream < 0 {
                                mhas_packet_type = MHA_PACTYP_NONE;
                                mhas_packet_label = 0;
                                mhas_substream = 0;
                            }
                        } else {
                            mhas_substream = 0;
                            if mhas_packet_type != MHA_PACTYP_CRC16
                                && mhas_packet_type != MHA_PACTYP_CRC32
                            {
                                nbits_mhas_main = nbits_mhas_current;
                            }
                        }
                        let ms = mhas_substream as usize;

                        // At the end of a series of MHA_PACTYP_MPEGH3DAFRAME break to start
                        // decoding
                        if self.transport_fmt == TT_MHAS
                            && mhas_packet_type_prev == MHA_PACTYP_MPEGH3DAFRAME
                            && mhas_packet_label <= 16
                        {
                            for _i in 0..TPDEC_MAX_TRACKS {
                                // skip all MHAS packets belonging to a MHA_PACTYP_MPEGH3DACFG
                                // except itself after an AU that is right truncated
                                if self.ctrl_cfg_change[ms].truncation_present != 0 {
                                    self.parser.mhas_labels[ms] = 0;
                                }
                            }

                            let tt = self.transport_fmt;
                            let result = self.mpegh_prepare_decode(
                                &mut hbs,
                                tt,
                                PrepareDecodeState {
                                    bs_main_stream: &bs_main_stream,
                                    main_stream_present,
                                    sync_layer_frame_bits: &mut sync_layer_frame_bits,
                                    raw_data_block_length: &mut raw_data_block_length,
                                    au_bit_pos_build_up,
                                },
                                mhas_packet_length,
                                nbits,
                            );
                            if result == -1 {
                                break 'bail;
                            }
                            break;
                        }

                        // Process MHAS packet according to its type
                        'sw: {
                            match mhas_packet_type {
                                MHA_PACTYP_SYNC => {
                                    let sync_byte = hbs.read_bits(8) as i32;
                                    if sync_byte != 0xA5 || mhas_packet_label != 0 {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                        break 'bail;
                                    }
                                }
                                MHA_PACTYP_MPEGH3DACFG => {
                                    if mhas_packet_label == 0 {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                        break 'sw;
                                    }
                                    nbits3da_cfg = nbits_mhas_main;
                                    if self.ctrl_cfg_change[0].flush_status == TPDEC_FLUSH_OFF {
                                        let mut mpeg3da_cfg_changed: u8;

                                        if ms == 0 {
                                            start_up =
                                                (self.ctrl_cfg_change[ms].mpegh3da_config_len == 0)
                                                    as i32;
                                        }

                                        // Check if config buffer is large enough
                                        if mhas_packet_length > TP_MPEGH3DA_MAX_CONFIG_LEN as i32 {
                                            err = TRANSPORTDEC_SYNC_ERROR;
                                            break 'bail;
                                        }

                                        // Check if config has changed. Save new config.
                                        if mhas_packet_length
                                            != self.ctrl_cfg_change[ms].mpegh3da_config_len as i32
                                        {
                                            mpeg3da_cfg_changed = 1;
                                            let ctrl = &mut self.ctrl_cfg_change[ms];
                                            ctrl.cfg_changed = 1;
                                            for i in 0..mhas_packet_length as usize {
                                                ctrl.mpegh3da_config[i] = hbs.read_bits(8) as u8;
                                            }
                                            ctrl.mpegh3da_config_len = mhas_packet_length as u32;
                                        } else {
                                            mpeg3da_cfg_changed = 0;
                                            if ms == 0 {
                                                self.disregard_loudness_change(
                                                    &mut hbs,
                                                    mhas_packet_length,
                                                    cb,
                                                );
                                            }
                                            let ctrl = &mut self.ctrl_cfg_change[ms];
                                            let mut i = 0usize;
                                            while (i as i32) < mhas_packet_length {
                                                if ctrl.mpegh3da_config[i] != hbs.read_bits(8) as u8
                                                {
                                                    mpeg3da_cfg_changed = 1;
                                                    ctrl.cfg_changed = 1;
                                                    hbs.push_back(8);
                                                    break;
                                                }
                                                i += 1;
                                            }
                                            while (i as i32) < mhas_packet_length {
                                                ctrl.mpegh3da_config[i] = hbs.read_bits(8) as u8;
                                                i += 1;
                                            }
                                        }

                                        if self.parser.mhas_labels[ms] != mhas_packet_label {
                                            mpeg3da_cfg_changed = 1;
                                            self.ctrl_cfg_change[ms].cfg_changed = 1;
                                            self.parser.mhas_labels[ms] = mhas_packet_label;
                                        }

                                        if mpeg3da_cfg_changed != 0 {
                                            self.ctrl_cfg_change[ms].content_changed = 1;
                                        }

                                        hbs.push_back((mhas_packet_length * 8) as u32);

                                        {
                                            let ctrl = &mut self.ctrl_cfg_change[ms];
                                            if ctrl.force_cfg_change != 0 {
                                                mpeg3da_cfg_changed = 1;
                                                ctrl.cfg_changed = 1;
                                                if ctrl.force_cfg_change
                                                    == TPDEC_FORCE_CONTENT_CHANGE
                                                {
                                                    ctrl.content_changed = 1;
                                                }
                                                ctrl.force_cfg_change = 0;
                                            }
                                        }

                                        // Clear ASI and downmix information in uniDrcConfig either
                                        // at startUp or in case of a config change after flushing.
                                        // ASI and downmix information is still needed during
                                        // flushing.
                                        if self.ctrl_cfg_change[ms].content_changed != 0
                                            && (mpeg3da_cfg_changed == 0 || start_up == 1)
                                        {
                                            // if content change clear ASI (to avoid keeping old ASI
                                            // if new content does not have ASI)
                                            if let Some(asi) = cb.asi() {
                                                asi_reset(asi);
                                            }
                                            // also clear downmix information in uniDrcConfig
                                            if cb.has_uni_drc() {
                                                cb.uni_drc(
                                                    None,
                                                    0,
                                                    2,
                                                    mhas_substream,
                                                    0,
                                                    AOT_MPEGH3DA,
                                                );
                                            }
                                        }

                                        if mpeg3da_cfg_changed == 1 && start_up == 0 {
                                            let ctrl = &mut self.ctrl_cfg_change[ms];
                                            if ms == 0 {
                                                ctrl.flush_cnt = 0;
                                                ctrl.flush_status =
                                                    TPDEC_MPEGH_CFG_CHANGE_ATSC_FLUSH_ON;
                                            }

                                            if ctrl.truncation_present == 0 {
                                                if cb.has_truncation_msg() && ms == 0 {
                                                    cb.truncation_msg(0, 2);
                                                }
                                                ctrl.truncation_present = 1;
                                            }
                                        } else {
                                            let mut config_changed: u8 = 0;
                                            let config_mode: u8 = AC_CM_DET_CFG_CHANGE;

                                            // save current ASC struct
                                            self.asc[TPDEC_MAX_TRACKS] = self.asc[ms];
                                            err = mpegh3da_config_parse(
                                                &mut self.asc[ms],
                                                &mut hbs,
                                                0,
                                                cb,
                                                config_mode,
                                                config_changed,
                                                self.target_layout,
                                                mhas_substream,
                                            );
                                            self.stamp_speakers_owner(ms);
                                            earcon_config.sampling_frequency =
                                                self.asc[ms].m_sampling_frequency;
                                            if err == TRANSPORTDEC_OK {
                                                if self.asc[ms].m_sampling_frequency
                                                    != self.asc[0].m_sampling_frequency
                                                {
                                                    self.parser.mhas_labels[ms] = 0;
                                                    self.ctrl_cfg_change[ms].mpegh3da_config_len =
                                                        0;
                                                    if mhas_substream == last_substream_index {
                                                        last_substream_index = mhas_substream - 1;
                                                        while last_substream_index > 0 {
                                                            if self.parser.mhas_labels
                                                                [last_substream_index as usize]
                                                                > 0
                                                            {
                                                                break;
                                                            }
                                                            last_substream_index -= 1;
                                                        }
                                                    }
                                                    break 'sw;
                                                }
                                                self.parser.mhas_labels[ms] = mhas_packet_label;
                                                let err_c = self.call_update_config(
                                                    cb,
                                                    ms,
                                                    config_mode,
                                                    &mut config_changed,
                                                );
                                                if err_c != 0 {
                                                    err = TRANSPORTDEC_SYNC_ERROR;
                                                    break 'bail;
                                                } else {
                                                    f_config_found = 1;
                                                }
                                            } else {
                                                // restore previous ASC struct
                                                self.asc[ms] = self.asc[TPDEC_MAX_TRACKS];
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                                break 'bail;
                                            }

                                            if err == TRANSPORTDEC_OK {
                                                if self.ctrl_cfg_change[ms].cfg_changed != 0
                                                    || config_changed != 0
                                                    || self.asc[ms].aac_config_changed != 0
                                                    || self.asc[ms].sbr_config_changed != 0
                                                    || self.asc[ms].sac_config_changed != 0
                                                {
                                                    self.ctrl_cfg_change[ms].cfg_changed = 1;
                                                    let err_c = self.call_free_mem(cb, ms);
                                                    if err_c != 0 {
                                                        err = TRANSPORTDEC_SYNC_ERROR;
                                                    }
                                                    if ms == 0 && start_up != 0 {
                                                        let ctrl = &mut self.ctrl_cfg_change[ms];
                                                        ctrl.flush_status = TPDEC_FLUSH_OFF;
                                                        ctrl.flush_cnt = 0;
                                                        ctrl.build_up_status =
                                                            TPDEC_MPEGH_BUILD_UP_IDLE;
                                                        ctrl.build_up_cnt =
                                                            TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES
                                                                - 1;
                                                    } else if ms > 0
                                                        && self.ctrl_cfg_change[0].cfg_changed == 0
                                                    {
                                                        err = TRANSPORTDEC_SYNC_ERROR;
                                                        break 'bail;
                                                    }
                                                } else {
                                                    self.parser.flags[ms] |= MHAS_CONFIG_PRESENT;
                                                    self.parser.flags[ms] &= !MHAS_UI_PRESENT;
                                                    break 'sw;
                                                }
                                            }
                                        }
                                    }

                                    if ms == 0
                                        && self.ctrl_cfg_change[ms].flush_status
                                            == TPDEC_MPEGH_CFG_CHANGE_ATSC_FLUSH_ON
                                    {
                                        let vb = hbs.get_valid_bits();
                                        hbs.push_back(nbits_mhas_main.wrapping_sub(vb) as u32);
                                        // Activate flush mode in core
                                        if cb.ctrl_cfg_change(&self.ctrl_cfg_change[ms]) != 0 {
                                            err = TRANSPORTDEC_SYNC_ERROR;
                                        }

                                        self.ctrl_cfg_change[ms].flush_cnt =
                                            self.ctrl_cfg_change[ms].flush_cnt.wrapping_add(1);

                                        if err == TRANSPORTDEC_OK
                                            && self.transport_fmt == TT_MHAS_PACKETIZED
                                        {
                                            // don't reset buffer after flushing
                                            self.number_of_raw_data_blocks = 1;
                                        }

                                        // Deactivate flush mode and prepare for build up mode
                                        let ctrl = &mut self.ctrl_cfg_change[ms];
                                        if ctrl.flush_cnt
                                            >= TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES - 1
                                        {
                                            ctrl.flush_status = TPDEC_FLUSH_OFF;
                                            ctrl.flush_cnt = 0;
                                            ctrl.build_up_status = TPDEC_MPEGH_BUILD_UP_IDLE;
                                            ctrl.build_up_cnt =
                                                TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES - 1;
                                            ctrl.force_cfg_change = 0;
                                            ctrl.truncation_present = 0;
                                        }
                                        goto_bail = 1;
                                    }
                                    self.parser.flags[ms] |= MHAS_CONFIG_PRESENT;
                                    self.parser.flags[ms] &= !MHAS_UI_PRESENT;
                                }
                                MHA_PACTYP_MPEGH3DAFRAME => {
                                    if mhas_packet_label != 0
                                        && self.parser.mhas_labels[ms] == mhas_packet_label
                                    {
                                        let flags = self.parser.flags[ms];
                                        self.asc[ms].m_sc.ui_manager_active =
                                            if flags & MHAS_CONFIG_PRESENT != 0 {
                                                (flags & MHAS_UI_PRESENT == 0) as i8
                                            } else if flags & MHAS_UI_PRESENT != 0 {
                                                0
                                            } else {
                                                -1
                                            };
                                        self.parser.flags[ms] &=
                                            !(MHAS_CONFIG_PRESENT | MHAS_UI_PRESENT);

                                        if self.ctrl_cfg_change[ms].content_changed != 0 {
                                            self.ctrl_cfg_change[ms].content_changed = 0;
                                        }
                                        if self.ctrl_cfg_change[ms].cfg_changed != 0 {
                                            self.ctrl_cfg_change[ms].cfg_changed = 0;
                                            let mut config_changed: u8 = 1;
                                            let mut config_mode: u8 = AC_CM_ALLOC_MEM;
                                            if mhas_substream == last_substream_index {
                                                config_mode |= AC_CM_LAST_SUBSTREAM;
                                            }
                                            let err_c = self.call_update_config(
                                                cb,
                                                ms,
                                                config_mode,
                                                &mut config_changed,
                                            );
                                            if err_c != 0 {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                                break 'bail;
                                            } else {
                                                f_config_found = 1;
                                            }
                                        }

                                        // Flag MHAS label to be present.
                                        mhas_label_present[ms] = 1;

                                        if ms == 0
                                            && self.ctrl_cfg_change[ms].build_up_status
                                                == TPDEC_MPEGH_BUILD_UP_IDLE
                                        {
                                            // Activate build up mode in core
                                            if cb.ctrl_cfg_change(&self.ctrl_cfg_change[ms]) != 0 {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }
                                            self.ctrl_cfg_change[ms].build_up_status =
                                                TPDEC_MPEGH_BUILD_UP_ON;
                                            if au_bit_pos_build_up == -1 {
                                                au_bit_pos_build_up = nbits_mhas_main;
                                            }
                                            if err == TRANSPORTDEC_OK
                                                && self.transport_fmt == TT_MHAS_PACKETIZED
                                            {
                                                // don't reset buffer after idle process
                                                self.number_of_raw_data_blocks = 1;
                                            }
                                            break 'sw;
                                        }

                                        if ms == 0
                                            && self.ctrl_cfg_change[ms].build_up_status
                                                == TPDEC_MPEGH_BUILD_UP_ON
                                        {
                                            // Activate build up mode in core
                                            if cb.ctrl_cfg_change(&self.ctrl_cfg_change[ms]) != 0 {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }
                                            self.ctrl_cfg_change[ms].build_up_status =
                                                TPDEC_BUILD_UP_OFF;
                                        }

                                        if self.parser.mhas_labels[ms] != 0 {
                                            self.au_length[ms] = mhas_packet_length * 8;
                                        }
                                        if ms > 0 {
                                            self.bit_stream[ms] = hbs.clone();
                                        } else {
                                            main_stream_present = 1;
                                            bs_main_stream = hbs.clone();
                                        }
                                    } else if mhas_packet_label <= 16 {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                    } else {
                                        hbs.push_for((mhas_packet_length * 8) as u32);
                                    }

                                    if self.ctrl_cfg_change[0].build_up_status == TPDEC_BUILD_UP_OFF
                                        && cb.has_earcon_bs()
                                        && ms == 0
                                    {
                                        cb.earcon_bs(&mut hbs);
                                    }
                                }
                                MHA_PACTYP_AUDIOSCENEINFO => {
                                    if mhas_packet_label != 0
                                        && self.parser.mhas_labels[ms] == mhas_packet_label
                                    {
                                        let needs_reset;
                                        if let Some(asi) = cb.asi() {
                                            if mae_audio_scene_info(
                                                asi,
                                                &mut hbs,
                                                2 * 28,
                                                mhas_substream,
                                            ) != 0
                                            {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            } else if check_asi(
                                                asi,
                                                self.asc[ms].m_sc.bs_num_signal_groups as i32,
                                                &self.asc[ms].m_sc.m_signal_group_type,
                                                mhas_substream,
                                            ) != 0
                                            {
                                                asi_reset(asi);
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }
                                            needs_reset =
                                                asi.diff_flags & ASI_DIFF_NEEDS_RESET != 0;
                                        } else {
                                            break 'sw;
                                        }

                                        if needs_reset
                                            && (self.flags & TPDEC_USE_ELEM_SKIPPING) != 0
                                            && nbits3da_cfg != 0
                                            && self.ctrl_cfg_change[ms].build_up_status
                                                == TPDEC_BUILD_UP_OFF
                                            && ms == 0
                                            && self.ctrl_cfg_change[ms].mpegh3da_config_len != 0
                                        {
                                            {
                                                let ctrl = &mut self.ctrl_cfg_change[ms];
                                                ctrl.cfg_changed = 1;
                                                ctrl.force_cfg_change = 0;
                                                ctrl.force_crossfade = 1;

                                                ctrl.flush_cnt = 0;
                                                ctrl.flush_status =
                                                    TPDEC_MPEGH_CFG_CHANGE_ATSC_FLUSH_ON;

                                                if ctrl.truncation_present == 0 {
                                                    if cb.has_truncation_msg() && ms == 0 {
                                                        cb.truncation_msg(0, 2);
                                                    }
                                                    ctrl.truncation_present = 1;
                                                }
                                            }
                                            let vb = hbs.get_valid_bits();
                                            hbs.push_back(nbits3da_cfg.wrapping_sub(vb) as u32);
                                            // Activate flush mode in core
                                            if cb.ctrl_cfg_change(&self.ctrl_cfg_change[ms]) != 0 {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }

                                            self.ctrl_cfg_change[ms].flush_cnt =
                                                self.ctrl_cfg_change[ms].flush_cnt.wrapping_add(1);

                                            if err == TRANSPORTDEC_OK
                                                && self.transport_fmt == TT_MHAS_PACKETIZED
                                            {
                                                // don't reset buffer after flushing
                                                self.number_of_raw_data_blocks = 1;
                                            }

                                            // Deactivate flush mode and prepare for build up mode
                                            let ctrl = &mut self.ctrl_cfg_change[ms];
                                            if ctrl.flush_cnt
                                                >= TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES - 1
                                            {
                                                ctrl.flush_status = TPDEC_FLUSH_OFF;
                                                ctrl.flush_cnt = 0;
                                                ctrl.build_up_status = TPDEC_MPEGH_BUILD_UP_IDLE;
                                                ctrl.build_up_cnt =
                                                    TPDEC_MPEGH_NUM_CONFIG_CHANGE_FRAMES - 1;
                                                ctrl.truncation_present = 0;
                                                ctrl.force_crossfade = 0;
                                            }
                                            break 'bail;
                                        }
                                    } else if mhas_packet_label == 0
                                        || self.parser.mhas_labels[ms] != 0
                                    {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                    }
                                }
                                MHA_PACTYP_MARKER => {
                                    if cb.has_user_interact()
                                        && mhas_packet_length > 2
                                        && hbs.read_bits(8) == 0xE0
                                        && hbs.read_bits(8) == 0x0
                                    {
                                        // Value 0xE0 indicates an ID_Marker. This packet contains
                                        // an unique content identifier.
                                        if self.parser.mhas_labels[ms] == mhas_packet_label
                                            && mha_is_main_stream(mhas_packet_label)
                                        {
                                            if cb.user_interact(
                                                &mut hbs,
                                                mhas_packet_type,
                                                mhas_packet_length as u32,
                                            ) != 0
                                            {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }
                                        } else {
                                            err = TRANSPORTDEC_SYNC_ERROR;
                                        }
                                    }
                                }
                                MHA_PACTYP_LOUDNESS_DRC | MHA_PACTYP_USERINTERACTION => {
                                    if self.parser.mhas_labels[ms] == mhas_packet_label
                                        && mha_is_main_stream(mhas_packet_label)
                                    {
                                        if cb.has_user_interact() {
                                            if cb.user_interact(
                                                &mut hbs,
                                                mhas_packet_type,
                                                mhas_packet_length as u32,
                                            ) != 0
                                            {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            } else {
                                                self.parser.flags[ms] |= MHAS_UI_PRESENT;
                                            }
                                        }
                                    } else {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                    }
                                }
                                MHA_PACTYP_BUFFERINFO => {
                                    if hbs.read_bit() != 0 {
                                        self.parser.buffer_fullness =
                                            hbs.escaped_value(15, 24, 32) as i32;
                                    }
                                }
                                // Audio TruncationInfo
                                MHA_PACTYP_AUDIOTRUNCATION => {
                                    // audioTruncationInfo();
                                    if mha_is_main_stream(mhas_packet_label)
                                        && self.parser.mhas_labels[ms] == mhas_packet_label
                                    {
                                        let is_active = hbs.read_bits(1) as i32;
                                        hbs.read_bits(1); // reserved
                                        let trunc_from_begin = hbs.read_bits(1) as i32;
                                        let n_trunc_samples = hbs.read_bits(13) as i32;
                                        earcon_config.m_bs_mhas_truncation_length =
                                            n_trunc_samples as u32;

                                        // Check that truncation length does not exceed max
                                        // nominal frame length
                                        if is_active != 0
                                            && 48000u32 * 1024
                                                < (n_trunc_samples as u32)
                                                    .wrapping_mul(self.asc[0].m_sampling_frequency)
                                        {
                                            err = TRANSPORTDEC_SYNC_ERROR;
                                        }

                                        if is_active != 0 && cb.has_truncation_msg() {
                                            // truncation callback should not be executed if a
                                            // left truncation without content change or without
                                            // config change occurred
                                            let ctrl = &self.ctrl_cfg_change[ms];
                                            if !(ctrl.content_changed == 0
                                                && ctrl.cfg_changed == 0
                                                && trunc_from_begin != 0)
                                                && cb.truncation_msg(
                                                    n_trunc_samples,
                                                    trunc_from_begin,
                                                ) != 0
                                            {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }
                                        }
                                        if is_active != 0 && trunc_from_begin == 0 {
                                            self.ctrl_cfg_change[ms].truncation_present = 1;
                                            if self.ctrl_cfg_change[0].flush_status
                                                == TPDEC_FLUSH_OFF
                                                && self.ctrl_cfg_change[0].build_up_status
                                                    == TPDEC_BUILD_UP_OFF
                                            {
                                                self.ctrl_cfg_change[ms].force_cfg_change =
                                                    TPDEC_FORCE_CONTENT_CHANGE;
                                            }
                                        }
                                    }
                                    if mhas_packet_label == 0 {
                                        let is_active = hbs.read_bits(1);
                                        if is_active == 0 {
                                            err = TRANSPORTDEC_SYNC_ERROR;
                                        }
                                        hbs.read_bits(1); // reserved
                                        let trunc_from_begin = hbs.read_bits(1);
                                        if trunc_from_begin != 0 {
                                            err = TRANSPORTDEC_SYNC_ERROR;
                                        }
                                        let n_trunc_samples = hbs.read_bits(13);
                                        // Store for further processing
                                        if cb.has_earcon_config() {
                                            earcon_config.truncation_flag = 1;
                                            earcon_config.m_bs_pcm_truncation_length =
                                                n_trunc_samples;
                                            cb.earcon_config(&mut earcon_config);
                                        }
                                    }
                                }
                                MHA_PACTYP_CRC32 => {
                                    crc = hbs.read_bits(32);
                                    // crcType = 1; Not yet supported
                                }
                                MHA_PACTYP_CRC16 => {
                                    crc = hbs.read_bits(16);
                                    crc_type = 0; // Signal CRC for next item in MHAS
                                }
                                MHA_PACTYPE_EARCON => {
                                    if cb.has_earcon_info() {
                                        let mut info = EarconInfo::default();
                                        if earcon_info(&mut hbs, &mut info) != 0 {
                                            earcon_config.earcon_parse_error = 1;
                                            break 'sw;
                                        }
                                        cb.earcon_info(&mut info);
                                    }
                                }
                                MHA_PACTYPE_PCMCONFIG => {
                                    if cb.has_earcon_config() {
                                        if earcon_config.earcon_parse_error != 0 {
                                            break 'sw;
                                        }
                                        // Parse the Config data
                                        if pcm_data_config(&mut hbs, &mut earcon_config) != 0 {
                                            earcon_config.earcon_parse_error = 1;
                                            break 'sw;
                                        }
                                        earcon_config.earcon_flag = 1;
                                        cb.earcon_config(&mut earcon_config);
                                    }
                                }
                                MHA_PACTYPE_PCMDATA => {
                                    if cb.has_earcon_bs() {
                                        if earcon_config.earcon_parse_error != 0 {
                                            break 'sw;
                                        }
                                        cb.earcon_bs(&mut hbs);
                                    }
                                }
                                MHA_PACTYP_LOUDNESS => {
                                    // mpegh3daLoudnessInfoSet();
                                    if mhas_packet_label != 0
                                        && self.parser.mhas_labels[ms] == mhas_packet_label
                                    {
                                        if cb.has_uni_drc() {
                                            let err_c = cb.uni_drc(
                                                Some(&mut hbs),
                                                mhas_packet_length,
                                                1, // loudnessInfoSet
                                                0,
                                                0,
                                                AOT_MPEGH3DA,
                                            );
                                            if err_c != 0 {
                                                err = TRANSPORTDEC_SYNC_ERROR;
                                            }
                                        }
                                    } else {
                                        err = TRANSPORTDEC_SYNC_ERROR;
                                    }
                                }
                                _ => {}
                            }
                        }

                        if self.transport_fmt == TT_MHAS_PACKETIZED
                            && (mhas_packet_type == MHA_PACTYP_MPEGH3DAFRAME
                                || (mhas_packet_type == MHA_PACTYP_NONE && mhas_packet_label == 0))
                            && nbits <= mhas_packet_length * 8
                        {
                            let tt = self.transport_fmt;
                            let result = self.mpegh_prepare_decode(
                                &mut hbs,
                                tt,
                                PrepareDecodeState {
                                    bs_main_stream: &bs_main_stream,
                                    main_stream_present,
                                    sync_layer_frame_bits: &mut sync_layer_frame_bits,
                                    raw_data_block_length: &mut raw_data_block_length,
                                    au_bit_pos_build_up,
                                },
                                mhas_packet_length,
                                nbits,
                            );
                            if result == -1 {
                                break 'bail;
                            }
                            if result == 1 {
                                break;
                            }
                        }

                        // Check amount of parsed bits. If too many bits were read assume parse
                        // error
                        {
                            nbits = 8 * mhas_packet_length - (nbits - hbs.get_valid_bits());
                            if nbits >= 0 {
                                hbs.push_for(nbits as u32);
                            } else {
                                err = TRANSPORTDEC_SYNC_ERROR;
                            }
                        }
                        if err != TRANSPORTDEC_OK {
                            break;
                        }
                    } // loop over all MHAS packets

                    if self.transport_fmt == TT_MHAS_PACKETIZED {
                        // packet finished
                        self.number_of_raw_data_blocks = 0;
                    }
                    if err != TRANSPORTDEC_OK {
                        for f in self.parser.flags.iter_mut() {
                            *f &= !MHAS_CONFIG_PRESENT;
                        }
                    }
                }
                _ => {
                    sync_layer_frame_bits = 0;
                }
            }
        }

        // bail:
        *p_raw_data_block_length = raw_data_block_length;

        if let Some(h) = p_header_bits {
            *h += start_pos - hbs.get_valid_bits();
        }

        self.bit_stream[bs_idx] = hbs;

        for i in 0..TPDEC_MAX_TRACKS {
            // If parsing error while config found, clear ctrlCFGChange-struct
            if self.ctrl_cfg_change[i].cfg_changed != 0 && err != TRANSPORTDEC_OK {
                self.number_of_raw_data_blocks = 0;
                let ctrl = &mut self.ctrl_cfg_change[i];
                ctrl.mpegh3da_config_len = 1;
                ctrl.flush_cnt = 0;
                ctrl.flush_status = TPDEC_FLUSH_OFF;
                ctrl.build_up_cnt = 0;
                ctrl.build_up_status = TPDEC_BUILD_UP_OFF;
                ctrl.cfg_changed = 0;
                ctrl.content_changed = 0;
                ctrl.force_cfg_change = 0;
                ctrl.truncation_present = 0;

                cb.ctrl_cfg_change(ctrl);
            }
        }

        if let Some(p) = pf_config_found {
            *p = f_config_found;
        }

        if let Some(p) = pf_traverse_more_frames {
            *p = f_traverse_more_frames;
        }
        if let Some(p) = p_sync_layer_frame_bits {
            *p = sync_layer_frame_bits;
        }

        if err == TRANSPORTDEC_NOT_ENOUGH_BITS && tt_is_packet(self.transport_fmt) {
            err = TRANSPORTDEC_SYNC_ERROR;
        }

        err
    }

    /// `synchronization()`
    #[allow(clippy::nonminimal_bool)]
    fn synchronization(
        &mut self,
        p_header_bits: Option<&mut i32>,
        cb: &mut dyn TpCallbacks,
    ) -> TransportDecError {
        let mut err = TRANSPORTDEC_OK;
        let mut err_first_frame = TRANSPORTDEC_OK;

        let mut sync_layer_frame_bits = 0; // Length of sync layer frame (i.e. LOAS)
        let mut raw_data_block_length = 0;
        let mut raw_data_block_length_previous;
        let mut header_bits = 0;
        let mut header_bits_first_frame = 0;
        let mut header_bits_previous;
        let mut num_frames_traversed = 0;
        let mut f_traverse_more_frames: i32;
        let mut f_config_found = (self.flags & TPDEC_CONFIG_FOUND) as i32;
        let mut start_pos_first_frame = -1;
        let mut num_raw_data_blocks_first_frame = 0;
        let mut num_raw_data_blocks_previous;
        let mut global_frame_pos_first_frame: i32 = 0;
        let mut raw_data_block_length_first_frame = 0;
        let mut ignore_buffer_fullness = (self.flags
            & (TPDEC_LOST_FRAMES_PENDING | TPDEC_IGNORE_BUFFERFULLNESS | TPDEC_SYNCOK))
            as i32;
        let mut end_tp_frame_bits_previous: u32 = 0;

        let mut context_first_frame = TransportDecParser::default();

        let total_bits = self.bit_stream[0].get_valid_bits();

        'bail: {
            if total_bits <= 0 {
                err = TRANSPORTDEC_NOT_ENOUGH_BITS;
                break 'bail;
            }

            f_traverse_more_frames = ((self.flags & (TPDEC_MINIMIZE_DELAY | TPDEC_EARLY_CONFIG))
                != 0
                && (self.flags & TPDEC_SYNCOK) == 0) as i32;

            // Set transport specific sync parameters
            let (sync_word, sync_length): (u32, i32) = match self.transport_fmt {
                TT_MHAS => ((6 << (2 + 11 + 8)) + (1 << 8) + 0xa5, 3 + 2 + 11 + 8),
                _ => (0, 0),
            };

            let sync_mask: u32 = (1u32 << sync_length).wrapping_sub(1);

            loop {
                // Bits available in bitstream buffer
                let mut bits_avail: i32;

                f_config_found = (self.flags & TPDEC_CONFIG_FOUND) as i32;
                header_bits_previous = header_bits;

                bits_avail = self.bit_stream[0].get_valid_bits();

                if self.number_of_raw_data_blocks == 0
                    // In case of MHAS skip sync parsing once we are in SYNCOK state because there
                    // might be no MHA_PACTYP_SYNC here.
                    && ((self.transport_fmt == TT_MHAS && (self.flags & TPDEC_SYNCOK) == 0)
                        || self.transport_fmt != TT_MHAS)
                {
                    // search synchword

                    if bits_avail - sync_length < TPDEC_SYNCSKIP {
                        err = TRANSPORTDEC_NOT_ENOUGH_BITS;
                        header_bits = 0;
                    } else {
                        let hbs = &mut self.bit_stream[0];
                        let mut synch = hbs.read_bits(sync_length as u32);

                        if (self.flags & TPDEC_SYNCOK) == 0 {
                            while bits_avail - sync_length >= TPDEC_SYNCSKIP {
                                if synch == sync_word {
                                    break;
                                }
                                synch = ((synch << TPDEC_SYNCSKIP) & sync_mask)
                                    | hbs.read_bits(TPDEC_SYNCSKIP as u32);
                                bits_avail -= TPDEC_SYNCSKIP;
                            }
                        }
                        if synch != sync_word {
                            // No correct syncword found.
                            err = TRANSPORTDEC_SYNC_ERROR;
                        } else {
                            err = TRANSPORTDEC_OK;
                        }
                        header_bits = sync_length;
                    }
                } else {
                    header_bits = 0;
                }

                // Save previous raw data block data
                raw_data_block_length_previous = raw_data_block_length;
                num_raw_data_blocks_previous = self.number_of_raw_data_blocks;

                // Parse transport header (raw data block granularity)

                if err == TRANSPORTDEC_OK {
                    err = self.read_header(
                        0,
                        sync_length,
                        ignore_buffer_fullness,
                        &mut raw_data_block_length,
                        Some(&mut f_traverse_more_frames),
                        Some(&mut sync_layer_frame_bits),
                        Some(&mut f_config_found),
                        Some(&mut header_bits),
                        cb,
                    );
                    if header_bits > bits_avail {
                        err = if header_bits < self.bit_stream[0].h_bit_buf.buf_bits() as i32 {
                            TRANSPORTDEC_NOT_ENOUGH_BITS
                        } else {
                            TRANSPORTDEC_SYNC_ERROR
                        };
                    }
                    if tpdec_is_fatal_error(err) {
                        // Rewind - TPDEC_SYNCSKIP, in order to look for a synch one bit ahead next
                        // time. Ensure that the bit amount lands at a multiple of TPDEC_SYNCSKIP.
                        self.bit_stream[0].push_bi_directional(
                            -header_bits + TPDEC_SYNCSKIP + (bits_avail % TPDEC_SYNCSKIP),
                        );

                        break 'bail;
                    }
                }

                bits_avail -= header_bits;

                let mut check_length_bits = sync_layer_frame_bits;
                if self.flags & TPDEC_CHECK_TWO_SYNCS != 0 {
                    check_length_bits += sync_length;
                }

                // Check if the whole frame would fit the bitstream buffer
                if err == TRANSPORTDEC_OK {
                    if check_length_bits + header_bits
                        > ((TRANSPORTDEC_BUFFER_SIZE as i32) << 3) - 7
                    {
                        // We assume that the size of the transport bit buffer has been chosen to
                        // meet all system requirements, thus this condition is considered a
                        // synchronisation error.
                        err = TRANSPORTDEC_SYNC_ERROR;
                    } else if bits_avail < check_length_bits {
                        err = TRANSPORTDEC_NOT_ENOUGH_BITS;
                    }
                }

                if err == TRANSPORTDEC_NOT_ENOUGH_BITS {
                    break;
                }

                if self.flags & TPDEC_CHECK_TWO_SYNCS != 0
                    && err == TRANSPORTDEC_OK
                    && self.number_of_raw_data_blocks == 0
                    && sync_layer_frame_bits > 0
                {
                    let hbs = &mut self.bit_stream[0];

                    // Forward to next sync word
                    hbs.push_for(sync_layer_frame_bits as u32);

                    // Read next synchword
                    let next_synch = hbs.read_bits(sync_length as u32);

                    if next_synch != sync_word {
                        err = TRANSPORTDEC_SYNC_ERROR;
                    }

                    // Rewind to the payload position
                    hbs.push_back((sync_layer_frame_bits + sync_length) as u32);
                }

                if err == TRANSPORTDEC_SYNC_ERROR {
                    // Enforce re-sync of transport headers.
                    self.number_of_raw_data_blocks = 0;

                    // Ensure that the bit amount lands at a multiple of TPDEC_SYNCSKIP
                    let bits = (bits_avail + header_bits) % TPDEC_SYNCSKIP;
                    // Rewind - TPDEC_SYNCSKIP, in order to look for a synch one bit ahead next
                    // time.
                    self.bit_stream[0].push_bi_directional(-(header_bits - TPDEC_SYNCSKIP) + bits);
                    header_bits = 0;
                }

                // Frame traversal
                if f_traverse_more_frames != 0 {
                    // Save parser context for early config discovery "rewind all frames"
                    if (self.flags & TPDEC_EARLY_CONFIG) != 0
                        && (self.flags & TPDEC_MINIMIZE_DELAY) == 0
                    {
                        // ignore buffer fullness if just traversing additional frames for ECD
                        ignore_buffer_fullness = 1;

                        // Save context in order to return later
                        if err == TRANSPORTDEC_OK && start_pos_first_frame == -1 {
                            start_pos_first_frame = self.bit_stream[0].get_valid_bits();
                            num_raw_data_blocks_first_frame = self.number_of_raw_data_blocks;
                            global_frame_pos_first_frame = self.global_frame_pos as i32;
                            raw_data_block_length_first_frame = raw_data_block_length;
                            header_bits_first_frame = header_bits;
                            err_first_frame = err;
                            context_first_frame = self.parser;
                        }

                        // Break when config was found or it is not possible anymore to find a
                        // config
                        if start_pos_first_frame != -1
                            && (f_config_found != 0 || err != TRANSPORTDEC_OK)
                        {
                            // In case of ECD and sync error, do not rewind anywhere.
                            if err == TRANSPORTDEC_SYNC_ERROR {
                                start_pos_first_frame = -1;
                                f_config_found = 0;
                                num_frames_traversed = 0;
                            }
                            break;
                        }
                    }

                    if err == TRANSPORTDEC_OK {
                        self.bit_stream[0].push_for(raw_data_block_length as u32);
                        num_frames_traversed += 1;
                        end_tp_frame_bits_previous = self.bit_stream[0].get_valid_bits() as u32;
                        // Ignore error here itentionally.
                        self.adjust_end_of_access_unit();
                        end_tp_frame_bits_previous = end_tp_frame_bits_previous
                            .wrapping_sub(self.bit_stream[0].get_valid_bits() as u32);
                    }
                }

                if !(f_traverse_more_frames != 0
                    || (err == TRANSPORTDEC_SYNC_ERROR && (self.flags & TPDEC_SYNCOK) == 0))
                {
                    break;
                }
            }

            // Restore context in case of ECD frame traversal
            if start_pos_first_frame != -1 && (f_config_found != 0 || err != TRANSPORTDEC_OK) {
                let vb = self.bit_stream[0].get_valid_bits();
                self.bit_stream[0].push_bi_directional(vb - start_pos_first_frame);
                self.parser = context_first_frame;
                self.number_of_raw_data_blocks = num_raw_data_blocks_first_frame;
                self.global_frame_pos = global_frame_pos_first_frame as u32;
                raw_data_block_length = raw_data_block_length_first_frame;
                header_bits = header_bits_first_frame;
                err = err_first_frame;
                num_frames_traversed = 0;
            }

            // Rewind for retry because of not enough bits
            if err == TRANSPORTDEC_NOT_ENOUGH_BITS {
                self.bit_stream[0].push_back(header_bits as u32);
                self.number_of_raw_data_blocks = num_raw_data_blocks_previous;
                header_bits = 0;
                raw_data_block_length = raw_data_block_length_previous;
            } else {
                // reset hold off frame counter
                self.hold_off_frames = 0;
            }

            // Return to last good frame in case of frame traversal but not ECD.
            if num_frames_traversed > 0 {
                self.bit_stream[0].push_back(
                    (raw_data_block_length_previous as u32)
                        .wrapping_add(end_tp_frame_bits_previous),
                );
                if err != TRANSPORTDEC_OK {
                    self.number_of_raw_data_blocks = num_raw_data_blocks_previous;
                    header_bits = header_bits_previous;
                    raw_data_block_length = raw_data_block_length_previous;
                }
                err = TRANSPORTDEC_OK;
            }
        }

        // bail:
        self.au_length[0] = raw_data_block_length;

        // Detect pointless TRANSPORTDEC_NOT_ENOUGH_BITS error case, where the bit buffer is already
        // full, or no new burst packet fits. Recover by advancing the bit buffer.
        if total_bits > 0 && err == TRANSPORTDEC_NOT_ENOUGH_BITS {
            let limit = ((TRANSPORTDEC_BUFFER_SIZE as u32 * 8)
                .wrapping_sub(self.avg_bit_rate.wrapping_mul(self.burst_period) / 1000))
            .wrapping_sub(7) as i32;
            if self.bit_stream[0].get_valid_bits() >= limit {
                self.bit_stream[0].push_for(TPDEC_SYNCSKIP as u32);
                err = TRANSPORTDEC_SYNC_ERROR;
            }
        }

        if err == TRANSPORTDEC_OK {
            self.flags |= TPDEC_SYNCOK;
        }

        if f_config_found != 0 {
            self.flags |= TPDEC_CONFIG_FOUND;
        }

        if let Some(h) = p_header_bits {
            *h = header_bits;
        }

        if err == TRANSPORTDEC_SYNC_ERROR {
            self.flags &= !TPDEC_SYNCOK;
        }

        err
    }

    /// `transportDec_readStream()`: synchronize to the stream (the missing access unit estimation
    /// is not implemented for MHAS).
    fn read_stream(&mut self, _layer: u32, cb: &mut dyn TpCallbacks) -> TransportDecError {
        let mut header_bits = 0;
        let error = self.synchronization(Some(&mut header_bits), cb);

        self.missing_access_units = -1;

        error
    }

    /// `transportDec_ReadAccessUnit(hTp, layer)`: read one access unit from the transport
    /// medium.
    pub fn read_access_unit(&mut self, layer: u32, cb: &mut dyn TpCallbacks) -> TransportDecError {
        let mut err = TRANSPORTDEC_OK;
        let l = layer as usize;

        if self.bit_stream[l].get_valid_bits() <= 0 {
            // This is only relevant for RAW and ADIF cases.
            // For streaming formats err will get overwritten.
            err = TRANSPORTDEC_NOT_ENOUGH_BITS;
            self.number_of_raw_data_blocks = 0;
        }

        match self.transport_fmt {
            TT_MP4_RAW | TT_MHA_RAW => {
                // One Access Unit was filled into buffer. So get the length out of the buffer.
                self.au_length[l] = self.bit_stream[l].get_valid_bits();
                self.flags |= TPDEC_SYNCOK;
            }

            TT_MHAS_PACKETIZED => {
                if err == TRANSPORTDEC_OK {
                    let mut f_config_found = (self.flags & TPDEC_CONFIG_FOUND) as i32;
                    let mut au_length = self.au_length[l];
                    err = self.read_header(
                        l,
                        0,
                        1,
                        &mut au_length,
                        None,
                        None,
                        Some(&mut f_config_found),
                        None,
                        cb,
                    );
                    self.au_length[l] = au_length;
                    if f_config_found != 0 {
                        self.flags |= TPDEC_CONFIG_FOUND;
                    }
                }
            }

            TT_MHAS => {
                err = self.read_stream(layer, cb);
            }

            _ => {
                err = TRANSPORTDEC_UNSUPPORTED_FORMAT;
            }
        }

        if err == TRANSPORTDEC_OK {
            self.access_unit_anchor[l] = self.bit_stream[l].get_valid_bits() as u32;
        } else {
            self.access_unit_anchor[l] = 0;
        }

        err
    }

    /// `transportDec_GetAuBitsRemaining(hTp, layer)`: remaining amount of bits of the current
    /// access unit; can be below zero if too many bits have been read.
    pub fn get_au_bits_remaining(&mut self, layer: u32) -> i32 {
        let l = layer as usize;
        let mut bits;

        if self.access_unit_anchor[l] > 0 && self.au_length[l] > 0 {
            bits = self.bit_stream[l].get_valid_bits();
            if bits >= 0 {
                bits = self.au_length[l] - (self.access_unit_anchor[l] as i32 - bits);
            }
        } else {
            bits = self.bit_stream[l].get_valid_bits();
        }

        bits
    }

    /// `transportDec_GetAuBitsTotal(hTp, layer)`: total amount of bits of the current access unit.
    pub fn get_au_bits_total(&self, layer: u32) -> i32 {
        self.au_length[layer as usize]
    }

    /// `transportDec_GetMissingAccessUnitCount(pNAccessUnits, hTp)`
    pub fn get_missing_access_unit_count(&self, p_n_access_units: &mut i32) -> TransportDecError {
        *p_n_access_units = self.missing_access_units;
        TRANSPORTDEC_OK
    }

    /// `transportDec_EndAccessUnit(hTp)`: to be called when the decoder has finished parsing one
    /// access unit, for bitstream housekeeping.
    pub fn end_access_unit(&mut self) -> TransportDecError {
        self.adjust_end_of_access_unit()
    }

    /// `transportDec_SetParam(hTp, param, value)`. `TPDEC_PARAM_FORCE_CONFIG_CHANGE` requires
    /// `0 <= value < TPDEC_MAX_TRACKS` (unchecked array index in C).
    pub fn set_param(&mut self, param: TpdecParam, value: i32) -> TransportDecError {
        let set = |flags: &mut u32, bit: u32| {
            if value != 0 {
                *flags |= bit;
            } else {
                *flags &= !bit;
            }
        };
        match param {
            TPDEC_PARAM_MINIMIZE_DELAY => set(&mut self.flags, TPDEC_MINIMIZE_DELAY),
            TPDEC_PARAM_EARLY_CONFIG => set(&mut self.flags, TPDEC_EARLY_CONFIG),
            TPDEC_PARAM_IGNORE_BUFFERFULLNESS => set(&mut self.flags, TPDEC_IGNORE_BUFFERFULLNESS),
            TPDEC_PARAM_SET_BITRATE => self.avg_bit_rate = value as u32,
            TPDEC_PARAM_BURST_PERIOD => self.burst_period = value as u32,
            TPDEC_PARAM_RESET => {
                for i in 0..TPDEC_MAX_TRACKS {
                    self.bit_stream[i].reset_bitbuffer(FdkBsCfg::BsReader);
                    self.au_length[i] = 0;
                    self.access_unit_anchor[i] = 0;
                    self.ctrl_cfg_change[i].mpegh3da_config_len = 0;
                }
                self.flags &= !(TPDEC_SYNCOK | TPDEC_CONFIG_FOUND | TPDEC_LOST_FRAMES_PENDING);
                self.remainder = 0;
                self.avg_bit_rate = 0;
                self.missing_access_units = 0;
                self.number_of_raw_data_blocks = 0;
                self.global_frame_pos = 0;
                self.hold_off_frames = 0;
            }
            TPDEC_PARAM_TARGETLAYOUT => self.target_layout = value,
            TPDEC_PARAM_FORCE_CONFIG_CHANGE => {
                self.ctrl_cfg_change[value as usize].force_cfg_change = TPDEC_FORCE_CONFIG_CHANGE;
            }
            TPDEC_PARAM_USE_ELEM_SKIPPING => set(&mut self.flags, TPDEC_USE_ELEM_SKIPPING),
            TPDEC_PARAM_CHECK_TWO_SYNCS => set(&mut self.flags, TPDEC_CHECK_TWO_SYNCS),
            _ => {}
        }

        TRANSPORTDEC_OK
    }

    /// `transportDec_CrcStartReg(hTp, mBits)`: no CRC regions for the supported formats.
    pub fn crc_start_reg(&mut self, _m_bits: i32) -> i32 {
        -1
    }

    /// `transportDec_CrcEndReg(hTp, reg)`: no CRC regions for the supported formats.
    pub fn crc_end_reg(&mut self, _reg: i32) {}

    /// `transportDec_CrcCheck(hTp)`: no CRC for the supported formats.
    pub fn crc_check(&mut self) -> TransportDecError {
        TRANSPORTDEC_OK
    }
}
