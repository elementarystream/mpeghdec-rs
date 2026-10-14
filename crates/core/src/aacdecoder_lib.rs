// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/include/aacdecoder_lib.h and libMpeghDec/src/aacdecoder_lib.cpp.

//! Public interface of the AAC/MPEG-H core decoder library (`aacDecoder_*()`): error codes,
//! parameters, `CStreamInfo`, the decoder API and the transport decoder callbacks.

use mpeghdec_fdk::digest::StateDigest;

/// `AAC_DECODER_ERROR`: AAC decoder error codes.
pub type AacDecoderError = i32;

/// No error occurred. Output buffer is valid and error free.
pub const AAC_DEC_OK: AacDecoderError = 0x0000;
/// No error occurred. Output samples are not available yet.
pub const AAC_DEC_INTERMEDIATE_OK: AacDecoderError = 0x0001;
/// Heap returned NULL pointer. Output buffer is invalid.
pub const AAC_DEC_OUT_OF_MEMORY: AacDecoderError = 0x0002;
/// Error condition is of unknown reason, or from a another module. Output buffer is invalid.
pub const AAC_DEC_UNKNOWN: AacDecoderError = 0x0005;

/* Synchronization errors. Output buffer is invalid. */
pub const aac_dec_sync_error_start: AacDecoderError = 0x1000;
/// The transport decoder had synchronization problems.
pub const AAC_DEC_TRANSPORT_SYNC_ERROR: AacDecoderError = 0x1001;
/// The input buffer ran out of bits.
pub const AAC_DEC_NOT_ENOUGH_BITS: AacDecoderError = 0x1002;
pub const aac_dec_sync_error_end: AacDecoderError = 0x1FFF;

/* Initialization errors. Output buffer is invalid. */
pub const aac_dec_init_error_start: AacDecoderError = 0x2000;
/// The handle passed to the function call was invalid (NULL).
pub const AAC_DEC_INVALID_HANDLE: AacDecoderError = 0x2001;
/// The AOT found in the configuration is not supported.
pub const AAC_DEC_UNSUPPORTED_AOT: AacDecoderError = 0x2002;
/// The bitstream format is not supported.
pub const AAC_DEC_UNSUPPORTED_FORMAT: AacDecoderError = 0x2003;
/// The error resilience tool format is not supported.
pub const AAC_DEC_UNSUPPORTED_ER_FORMAT: AacDecoderError = 0x2004;
/// The error protection format is not supported.
pub const AAC_DEC_UNSUPPORTED_EPCONFIG: AacDecoderError = 0x2005;
/// More than one layer for AAC scalable is not supported.
pub const AAC_DEC_UNSUPPORTED_MULTILAYER: AacDecoderError = 0x2006;
/// The channel configuration (either number or arrangement) is not supported.
pub const AAC_DEC_UNSUPPORTED_CHANNELCONFIG: AacDecoderError = 0x2007;
/// The sample rate specified in the configuration is not supported.
pub const AAC_DEC_UNSUPPORTED_SAMPLINGRATE: AacDecoderError = 0x2008;
/// The SBR configuration is not supported.
pub const AAC_DEC_INVALID_SBR_CONFIG: AacDecoderError = 0x2009;
/// The parameter could not be set.
pub const AAC_DEC_SET_PARAM_FAIL: AacDecoderError = 0x200A;
/// The decoder needs to be restarted.
pub const AAC_DEC_NEED_TO_RESTART: AacDecoderError = 0x200B;
/// The provided output buffer is too small.
pub const AAC_DEC_OUTPUT_BUFFER_TOO_SMALL: AacDecoderError = 0x200C;
/// The function parameter type is not supported.
pub const AAC_DEC_INVALID_PARAM: AacDecoderError = 0x200D;
pub const aac_dec_init_error_end: AacDecoderError = 0x2FFF;

/* Decode errors. Output buffer is valid but concealed. */
pub const aac_dec_decode_error_start: AacDecoderError = 0x4000;
/// The transport decoder encountered an unexpected error.
pub const AAC_DEC_TRANSPORT_ERROR: AacDecoderError = 0x4001;
/// Error while parsing the bitstream.
pub const AAC_DEC_PARSE_ERROR: AacDecoderError = 0x4002;
/// The extension payload type found is not supported.
pub const AAC_DEC_UNSUPPORTED_EXTENSION_PAYLOAD: AacDecoderError = 0x4003;
/// The parsed bitstream value is out of range.
pub const AAC_DEC_DECODE_FRAME_ERROR: AacDecoderError = 0x4004;
/// The embedded CRC did not match.
pub const AAC_DEC_CRC_ERROR: AacDecoderError = 0x4005;
/// An invalid codebook was signaled.
pub const AAC_DEC_INVALID_CODE_BOOK: AacDecoderError = 0x4006;
/// Predictor found, but not supported in the AAC Low Complexity profile.
pub const AAC_DEC_UNSUPPORTED_PREDICTION: AacDecoderError = 0x4007;
/// A CCE element was found which is not supported.
pub const AAC_DEC_UNSUPPORTED_CCE: AacDecoderError = 0x4008;
/// A LFE element was found which is not supported.
pub const AAC_DEC_UNSUPPORTED_LFE: AacDecoderError = 0x4009;
/// Gain control data found but not supported.
pub const AAC_DEC_UNSUPPORTED_GAIN_CONTROL_DATA: AacDecoderError = 0x400A;
/// SBA found, but currently not supported in the BSAC profile.
pub const AAC_DEC_UNSUPPORTED_SBA: AacDecoderError = 0x400B;
/// Error while reading TNS data.
pub const AAC_DEC_TNS_READ_ERROR: AacDecoderError = 0x400C;
/// Error while decoding error resilient data.
pub const AAC_DEC_RVLC_ERROR: AacDecoderError = 0x400D;
pub const aac_dec_decode_error_end: AacDecoderError = 0x4FFF;

/* Ancillary data errors. Output buffer is valid. */
pub const aac_dec_anc_data_error_start: AacDecoderError = 0x8000;
/// Non severe error concerning the ancillary data handling.
pub const AAC_DEC_ANC_DATA_ERROR: AacDecoderError = 0x8001;
/// The registered ancillary data buffer is too small to receive the parsed data.
pub const AAC_DEC_TOO_SMALL_ANC_BUFFER: AacDecoderError = 0x8002;
/// More than the allowed number of ancillary data elements should be written to buffer.
pub const AAC_DEC_TOO_MANY_ANC_ELEMENTS: AacDecoderError = 0x8003;
pub const aac_dec_anc_data_error_end: AacDecoderError = 0x8FFF;

/// `IS_INIT_ERROR(err)`: identifies initialization errors. Output buffer is invalid.
#[inline]
pub fn is_init_error(err: AacDecoderError) -> bool {
    (aac_dec_init_error_start..=aac_dec_init_error_end).contains(&err)
}

/// `IS_DECODE_ERROR(err)`: identifies decode errors. Output buffer is valid but concealed.
#[inline]
pub fn is_decode_error(err: AacDecoderError) -> bool {
    (aac_dec_decode_error_start..=aac_dec_decode_error_end).contains(&err)
}

/// `IS_OUTPUT_VALID(err)`: the audio output buffer contains valid (possibly concealed) samples.
#[inline]
pub fn is_output_valid(err: AacDecoderError) -> bool {
    err == AAC_DEC_OK || is_decode_error(err)
}

/// `AACDEC_PARAM`: decoder parameters for [`AacDecoderInstance::aac_decoder_set_param`].
pub type AacdecParam = i32;
pub const AAC_PCM_DUAL_CHANNEL_OUTPUT_MODE: AacdecParam = 0x0002;
pub const AAC_PCM_OUTPUT_CHANNEL_MAPPING: AacdecParam = 0x0003;
pub const AAC_PCM_PSEUDO_SURROUND_DOWNMIX_MODE: AacdecParam = 0x0004;
pub const AAC_PCM_LIMITER_ENABLE: AacdecParam = 0x0010;
pub const AAC_PCM_LIMITER_ATTACK_TIME: AacdecParam = 0x0011;
pub const AAC_PCM_LIMITER_RELEAS_TIME: AacdecParam = 0x0012;
pub const AAC_METADATA_PROFILE: AacdecParam = 0x0020;
pub const AAC_METADATA_EXPIRY_TIME: AacdecParam = 0x0021;
pub const AAC_CONCEAL_METHOD: AacdecParam = 0x0100;
pub const AAC_CONCEAL_FADEOUT_SLOPE: AacdecParam = 0x0101;
pub const AAC_CONCEAL_FADEIN_SLOPE: AacdecParam = 0x0102;
pub const AAC_CONCEAL_MUTE_RELEASE: AacdecParam = 0x0103;
pub const AAC_CONCEAL_COMFORT_NOISE_LEVEL: AacdecParam = 0x0104;
pub const AAC_CONCEAL_FADEOUT_ATTENUATION_VECTOR: AacdecParam = 0x0105;
pub const AAC_CONCEAL_FADEIN_ATTENUATION_VECTOR: AacdecParam = 0x0106;
pub const AAC_DRC_BOOST_FACTOR: AacdecParam = 0x0200;
pub const AAC_DRC_ATTENUATION_FACTOR: AacdecParam = 0x0201;
pub const AAC_DRC_REFERENCE_LEVEL: AacdecParam = 0x0202;
pub const AAC_DRC_HEAVY_COMPRESSION: AacdecParam = 0x0203;
pub const AAC_DRC_DEFAULT_PRESENTATION_MODE: AacdecParam = 0x0204;
pub const AAC_DRC_ENC_TARGET_LEVEL: AacdecParam = 0x0205;
pub const AAC_UNIDRC_SET_EFFECT: AacdecParam = 0x0206;
pub const AAC_UNIDRC_ALBUM_MODE: AacdecParam = 0x0207;
pub const AAC_QMF_LOWPOWER: AacdecParam = 0x0300;
pub const AAC_QMF_RESAMPLER: AacdecParam = 0x0301;
pub const AAC_SAOC_ENABLE: AacdecParam = 0x0400;
pub const AAC_MPEGS_ENABLE: AacdecParam = 0x0500;
pub const AAC_MPEGS_BINAURAL_ENABLE: AacdecParam = 0x0501;
pub const AAC_MPEGS_BINAURAL_QUALITY: AacdecParam = 0x0504;
pub const AAC_MPEGS_BINAURAL_DISTANCE: AacdecParam = 0x0505;
pub const AAC_MPEGS_BINAURAL_DIALOG_CLARITY: AacdecParam = 0x0506;
pub const AAC_MPEGS_BINAURAL_FRONT_ANGLE: AacdecParam = 0x0507;
pub const AAC_MPEGS_BINAURAL_BACK_ANGLE: AacdecParam = 0x0508;
pub const AAC_MPEGS_BINAURAL_PRESET: AacdecParam = 0x0509;
pub const AAC_MPEGS_BLIND_MODE_ENABLE: AacdecParam = 0x0510;
pub const AAC_TPDEC_PARAM_MINIMIZE_DELAY: AacdecParam = 0x0600;
pub const AAC_TPDEC_PARAM_IGNORE_BUFFERFULLNESS: AacdecParam = 0x0601;
pub const AAC_TPDEC_PARAM_EARLY_CONFIG: AacdecParam = 0x0602;
pub const AAC_TPDEC_CLEAR_BUFFER: AacdecParam = 0x0603;
pub const AAC_TPDEC_PARAM_SET_BITRATE: AacdecParam = 0x0604;
pub const AAC_TPDEC_PARAM_SET_BURSTPERIOD: AacdecParam = 0x0605;
pub const AAC_TPDEC_PARAM_CHECK_TWO_SYNCS: AacdecParam = 0x0606;
pub const AAC_USAC_LP_FLAGS: AacdecParam = 0x0800;
pub const AAC_TARGET_LAYOUT_CICP: AacdecParam = 0x0900;
pub const AAC_ADDITIONAL_STEREO_DMX: AacdecParam = 0x0908;
pub const AAC_MPEGH_GOA_ENABLE: AacdecParam = 0x0909;
pub const AAC_EQ_FILTER_ATTENUATION_VECTOR: AacdecParam = 0x0A00;

/// Flag for `aacDecoder_DecodeFrame()`: Trigger the built-in error concealment module to generate
/// a substitute signal for one lost frame. New input data will not be considered.
pub const AACDEC_CONCEAL: u32 = 1;
/// Flag for `aacDecoder_DecodeFrame()`: Flush all filterbanks to get all delayed audio without
/// having new input data. Thus new input data will not be considered.
pub const AACDEC_FLUSH: u32 = 2;
/// Flag for `aacDecoder_DecodeFrame()`: Signal an input bit stream data discontinuity. Resync any
/// internals as necessary.
pub const AACDEC_INTR: u32 = 4;
/// Flag for `aacDecoder_DecodeFrame()`: Clear all signal delay lines and history buffers.
pub const AACDEC_CLRHIST: u32 = 8;

/// `CStreamInfo`: information about the currently decoded audio data. All fields are read-only.
///
/// `pChannelType`, `pChannelIndices` and `pElements` point into the decoder instance; see
/// [`AacDecoderInstance::channel_type`], [`AacDecoderInstance::channel_indices`] and
/// [`AacDecoderInstance::elements`].
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CStreamInfo {
    /* These members are the only really relevant ones for the user. */
    /// The sample rate in Hz of the decoded PCM audio signal (incl. potential resampling).
    pub sample_rate: i32,
    /// The sample rate in Hz of the output PCM audio signal.
    pub sample_rate_before_rs: i32,
    /// The frame size of the decoded PCM audio signal.
    pub frame_size: i32,
    /// The full frame size in samples of the last decoded MPEG-H access unit. -1 means undefined
    /// (e.g. if the decoder returned with AAC_DEC_NOT_ENOUGH_BITS).
    pub mpegh_au_size: i32,
    /// The number of output audio channels.
    pub num_channels: i32,
    /// The number of output audio channels before the up-/downmix.
    pub num_channels_before_mix: i32,
    /// The number of bistream elements.
    pub num_elements: i32,
    /// Audio channel order (MPEG, WAV, CICP) provided at the decoder output.
    pub pcm_ch_order: ChannelOrder,

    /* Decoder internal members. */
    /// Sampling rate in Hz without SBR (from configuration info).
    pub aac_sample_rate: i32,
    /// MPEG-2 profile (from file header) (-1: not applicable (e. g. MPEG-4)).
    pub profile: i32,
    /// Audio Object Type (from ASC).
    pub aot: AudioObjectType,
    /// Channel configuration (0: PCE defined, 1: mono, 2: stereo, ...
    pub channel_config: i32,
    /// Instantaneous bit rate.
    pub bit_rate: i32,
    /// Samples per frame for the AAC core (from ASC).
    pub aac_samples_per_frame: i32,
    /// The number of audio channels after AAC core processing.
    pub aac_num_channels: i32,
    /// Extension Audio Object Type (from ASC)
    pub ext_aot: AudioObjectType,
    /// Extension sampling rate in Hz (from ASC).
    pub ext_sampling_rate: i32,

    /// The number of samples the output is additionally delayed by the decoder.
    pub output_delay: u32,
    /// Copy of internal flags.
    pub flags: u32,

    /// epConfig level (from ASC): only level 0 supported, -1 means no ER.
    pub ep_config: i8,
    /// MPEG-H reference layout found in the last MPEG-H config.
    pub reference_layout: u8,
    /* Statistics */
    /// Estimated amount of lost access units in case of AAC_DEC_TRANSPORT_SYNC_ERROR.
    pub num_lost_access_units: i32,

    /* The four counters are statistics for the application: the decoder only resets and
    increments them (aacDecoder_UpdateBitStreamCounters()) and never reads them, so they are not
    part of the state digest (they count everything since the decoder was opened). */
    /// Number of total bytes that have passed through the decoder.
    #[digest(skip)]
    pub num_total_bytes: i64,
    /// Number of total bytes that were considered with errors from numTotalBytes.
    #[digest(skip)]
    pub num_bad_bytes: i64,
    /// Number of total access units that have passed through the decoder.
    #[digest(skip)]
    pub num_total_access_units: i64,
    /// Number of total access units that were considered with errors from numTotalBytes.
    #[digest(skip)]
    pub num_bad_access_units: i64,

    /* Metadata */
    /// DRC program reference level.
    pub drc_prog_ref_lev: i8,
    /// DRC presentation mode.
    pub drc_pres_mode: i8,
    /// The 2 bit matrix mixdown index extracted from PCE.
    pub pce_matrix_mixdown_idx: i8,
    /// Pseudo Surround flag extracted from PCE
    pub pce_pseudo_surround_enable: i8,
    /// Audio output loudness in steps of -0.25 LU. Range: 0 (0 LKFS) to 231 (-57.75 LKFS). -1
    /// indicates that no loudness metadata is present.
    pub output_loudness: i32,

    /* Others */
    /// Flag indicating decoder context was synchronized. Set at error free independent frames.
    pub f_bs_restart_ok: u8,
}

use crate::aac_rom::PcmDec;
use crate::aacdecoder::*;
use crate::conceal::*;
use crate::conceal_types::*;
use crate::format_converter_glue::{
    downmix_matrix_set, store_downmix_matrix_set, DownmixGroupsMatrixSet,
    DOWNMIX_GROUPS_MATRIX_SET_WORDS,
};
use crate::render::*;
use crate::rs;
use mpeghdec_drc::*;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::cicp2geometry_get_num_channels_from_cicp_noerr;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::*;
use mpeghdec_fdk::mdct::PCM_OUT_HEADROOM;
use mpeghdec_fdk::scale::scale_values;
use mpeghdec_fdk::td_upsampler::*;
use mpeghdec_pcmutils::limiter::*;
use mpeghdec_tpdec::*;

impl AacDecoderInstance {
    /// `aacDecoder_GetFreeBytes(self, pFreeBytes)`
    pub fn aac_decoder_get_free_bytes(&mut self, p_free_bytes: &mut u32) -> AacDecoderError {
        /* reset free bytes */
        *p_free_bytes = 0;

        /* return nr of free bytes */
        let Some(tp) = self.h_input.as_deref_mut() else {
            return AAC_DEC_INVALID_HANDLE;
        };
        *p_free_bytes = (tp.get_bitstream(0).get_free_bits() >> 3) as u32;

        /* success */
        AAC_DEC_OK
    }

    /// `aacDecoder_Config(self, pAscStruct, configMode, configChanged)`: config decoder using a
    /// CSAudioSpecificConfig struct.
    fn aac_decoder_config(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        index: usize,
        config_mode: u8,
        config_changed: &mut u8,
    ) -> AacDecoderError {
        if self.flags[0] & AC_MPEGH3DA != 0 {
            /* Assimilate new UI status with changed signals */
            if (config_mode & AC_CM_ALLOC_MEM != 0)
                && (self.update_ui_status(ascs) == 0)
                && self.ui_signal_changed != 0
            {
                self.ui_status = self.ui_status_next;
                self.ui_signal_changed = 0;
                self.update_on_off_flags(ascs);
            }
        }

        /* Initialize AAC core decoder, and update self->streaminfo */
        self.caac_decoder_init(ascs, index, config_mode, config_changed)
    }

    /// `aacDecoder_ConfigRaw(self, conf, length)`: explicitly configure the decoder by passing a
    /// raw mpegh3daConfig (one buffer per layer).
    pub fn aac_decoder_config_raw(&mut self, conf: &[&[u8]]) -> AacDecoderError {
        let mut err = AAC_DEC_OK;

        let Some(mut tp) = self.h_input.take() else {
            return AAC_DEC_INVALID_HANDLE;
        };
        let nr_of_layers = self.nr_of_layers as usize;
        if tt_is_mpegh(tp.get_format()) && tt_cfg_is_inband(tp.get_format()) {
            self.h_input = Some(tp);
            return AAC_DEC_OK; /* in case of mhm1/mhm2 ignore optional out of band config (mhaC) */
        }
        for layer in 0..nr_of_layers {
            let Some(c) = conf.get(layer) else { break };
            if !c.is_empty() {
                let err_tp = tp.out_of_band_config(c, layer as u32, self);
                if err_tp != TRANSPORTDEC_OK {
                    err = match err_tp {
                        TRANSPORTDEC_NEED_TO_RESTART => AAC_DEC_NEED_TO_RESTART,
                        TRANSPORTDEC_UNSUPPORTED_FORMAT => AAC_DEC_UNSUPPORTED_FORMAT,
                        _ => AAC_DEC_UNKNOWN,
                    };
                    /* if baselayer is OK we continue decoding */
                    if layer >= 1 {
                        self.nr_of_layers = layer as i32;
                        err = AAC_DEC_OK;
                    }
                    break;
                }
            }
        }
        self.h_input = Some(tp);

        err
    }

    /// `aacDecoder_AncDataInit(self, buffer, size)`
    pub fn aac_decoder_anc_data_init(&mut self, buffer: Vec<u8>) -> AacDecoderError {
        let size = buffer.len() as i32;
        caac_decoder_anc_data_init(&mut self.anc_data, buffer, size)
    }

    /// `aacDecoder_AncDataGet(self, index, ptr, size)`
    pub fn aac_decoder_anc_data_get(&self, index: i32) -> Option<&[u8]> {
        caac_decoder_anc_data_get(&self.anc_data, index)
    }

    /// `setConcealMethod(self, method)`
    fn set_conceal_method(&mut self, method: i32) -> AacDecoderError {
        let error_status;
        let mut method_revert = 0;
        let mut backup_method = ConcealMethodNone;

        'bail: {
            if self.flags[0] & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 && method >= 2 {
                /* Interpolation concealment is not implemented for USAC/RSVD50 */
                method_revert = 1;
            }
            if self.flags[0] & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 && method >= 2 {
                /* Interpolation concealment is not implemented for USAC/RSVD50 */
                error_status = AAC_DEC_SET_PARAM_FAIL;
                break 'bail;
            }

            /* Get current method/delay */
            backup_method = cconcealment_get_method(Some(&self.conceal_common_data));

            /* Be sure to set AAC and SBR concealment method simultaneously! */
            error_status = cconcealment_set_params(
                Some(&mut self.conceal_common_data),
                if method_revert == 0 { method } else { 1 }, // concealMethod
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,          // concealFadeOutSlope
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,          // concealFadeInSlope
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,          // concealMuteRelease
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,          // concealComfNoiseLevel
            );
        }

        if (error_status != AAC_DEC_OK) && (error_status != AAC_DEC_INVALID_HANDLE) {
            /* Revert to the initial state */
            cconcealment_set_params(
                Some(&mut self.conceal_common_data),
                backup_method,
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
                AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
            );
        }

        error_status
    }

    /// `aacDecoder_SetParam(self, param, value)`: set one single decoder parameter.
    pub fn aac_decoder_set_param(&mut self, param: AacdecParam, value: i32) -> AacDecoderError {
        let mut error_status = AAC_DEC_OK;
        let mut err_tp = TRANSPORTDEC_OK;
        let mut uni_drc_err = DRC_DEC_OK;

        /* configure the subsystems */
        match param {
            AAC_PCM_LIMITER_ENABLE => {
                if !(-1..=1).contains(&value) {
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                self.limiter_enable_user = (value & 0xFF) as u8; /* Truncation is verified */
            }

            AAC_PCM_LIMITER_ATTACK_TIME => {
                if value <= 0 {
                    /* module function converts value to unsigned */
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                match rs!(self).h_limiter.as_mut().map(|l| l.set_attack(value as u32)) {
                    Some(TDLIMIT_OK) => {}
                    None | Some(TDLIMIT_INVALID_HANDLE) => return AAC_DEC_INVALID_HANDLE,
                    _ => return AAC_DEC_SET_PARAM_FAIL,
                }
            }

            AAC_PCM_LIMITER_RELEAS_TIME => {
                if value <= 0 {
                    /* module function converts value to unsigned */
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                match rs!(self).h_limiter.as_mut().map(|l| l.set_release(value as u32)) {
                    Some(TDLIMIT_OK) => {}
                    None | Some(TDLIMIT_INVALID_HANDLE) => return AAC_DEC_INVALID_HANDLE,
                    _ => return AAC_DEC_SET_PARAM_FAIL,
                }
            }

            AAC_PCM_OUTPUT_CHANNEL_MAPPING => {
                if !(0..=2).contains(&value) {
                    return AAC_DEC_SET_PARAM_FAIL;
                }
            }

            /* Target layout switch */
            AAC_TARGET_LAYOUT_CICP => {
                if value != -1
                    && value != 0
                    && cicp2geometry_get_num_channels_from_cicp_noerr(value) <= 0
                {
                    return AAC_DEC_SET_PARAM_FAIL;
                } else if self.asc_channels[0] > 0 {
                    /* Once the decoder has configured it is not possible to change the reference
                     * layout. */
                    return AAC_DEC_SET_PARAM_FAIL;
                } else {
                    let mut num_channels = 0;

                    if value > 0 {
                        num_channels = cicp2geometry_get_num_channels_from_cicp_noerr(value);
                    }

                    self.target_layout = value;
                    let Some(tp) = self.h_input.as_deref_mut() else {
                        return AAC_DEC_SET_PARAM_FAIL;
                    };
                    if tp.set_param(TPDEC_PARAM_TARGETLAYOUT, self.target_layout) != TRANSPORTDEC_OK
                    {
                        return AAC_DEC_SET_PARAM_FAIL;
                    }
                    let fmt = tp.get_format();
                    if self.target_layout == -1 {
                        uni_drc_err = rs!(self)
                            .h_uni_drc_decoder
                            .set_param(DRC_DEC_LOUDNESS_NORMALIZATION_ON, 0);
                        self.limiter_enable_user = 0;
                    }

                    if fmt != TT_MHAS
                        && fmt != TT_MHAS_PACKETIZED
                        && fmt != TT_MHA_RAW
                        && num_channels > 0
                    {
                        uni_drc_err = rs!(self)
                            .h_uni_drc_decoder
                            .set_param(DRC_DEC_TARGET_CHANNEL_COUNT_REQUESTED, num_channels);
                    }
                }
            }

            AAC_DRC_ATTENUATION_FACTOR => {
                /* DRC compression factor (where 0 is no and 127 is max compression) */
                if !(0..=127).contains(&value) {
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                uni_drc_err = rs!(self).h_uni_drc_decoder.set_param(
                    DRC_DEC_COMPRESS,
                    value * fl2fxconst_dbl((0.5f32 / 127.0f32) as f64),
                );
            }

            AAC_DRC_BOOST_FACTOR => {
                /* DRC boost factor (where 0 is no and 127 is max boost) */
                if !(0..=127).contains(&value) {
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                uni_drc_err = rs!(self)
                    .h_uni_drc_decoder
                    .set_param(DRC_DEC_BOOST, value * fl2fxconst_dbl((0.5f32 / 127.0f32) as f64));
            }

            AAC_DRC_REFERENCE_LEVEL => {
                let fmt = self.h_input.as_deref().map(|t| t.get_format()).unwrap_or(TT_UNKNOWN);
                if tt_is_mpegh(fmt) && (value < 0) {
                    /* Disabling loudness normalisation is not supported for MPEG-H */
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                if (value >= 0) && ((value < 40) || (value > 127)) {
                    /* allowed range: -10 to -31.75 LKFS */
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                /* DRC target reference level quantized in 0.25 LU steps using values [40..127].
                Negative values switch off loudness normalisation. */

                /* set target loudness also for MPEG-D DRC */
                uni_drc_err = rs!(self)
                    .h_uni_drc_decoder
                    .set_param(DRC_DEC_LOUDNESS_NORMALIZATION_ON, (value >= 0) as FixpDbl);
                self.default_target_loudness = value as i8;
            }

            AAC_UNIDRC_SET_EFFECT => {
                if !(-1..=6).contains(&value) {
                    return AAC_DEC_SET_PARAM_FAIL;
                }
                uni_drc_err = rs!(self).h_uni_drc_decoder.set_param(DRC_DEC_EFFECT_TYPE, value);
                self.default_drc_set_effect = value as i8;
            }
            AAC_UNIDRC_ALBUM_MODE => {
                uni_drc_err = rs!(self).h_uni_drc_decoder.set_param(DRC_DEC_ALBUM_MODE, value);
            }

            AAC_TPDEC_PARAM_MINIMIZE_DELAY
            | AAC_TPDEC_PARAM_IGNORE_BUFFERFULLNESS
            | AAC_TPDEC_PARAM_EARLY_CONFIG
            | AAC_TPDEC_PARAM_SET_BITRATE
            | AAC_TPDEC_PARAM_SET_BURSTPERIOD
            | AAC_TPDEC_PARAM_CHECK_TWO_SYNCS => {
                let tp_param = match param {
                    AAC_TPDEC_PARAM_MINIMIZE_DELAY => TPDEC_PARAM_MINIMIZE_DELAY,
                    AAC_TPDEC_PARAM_IGNORE_BUFFERFULLNESS => TPDEC_PARAM_IGNORE_BUFFERFULLNESS,
                    AAC_TPDEC_PARAM_EARLY_CONFIG => TPDEC_PARAM_EARLY_CONFIG,
                    AAC_TPDEC_PARAM_SET_BITRATE => TPDEC_PARAM_SET_BITRATE,
                    AAC_TPDEC_PARAM_SET_BURSTPERIOD => TPDEC_PARAM_BURST_PERIOD,
                    _ => TPDEC_PARAM_CHECK_TWO_SYNCS,
                };
                if let Some(tp) = self.h_input.as_deref_mut() {
                    err_tp = tp.set_param(tp_param, value);
                }
            }
            AAC_TPDEC_CLEAR_BUFFER => {
                if let Some(tp) = self.h_input.as_deref_mut() {
                    err_tp = tp.set_param(TPDEC_PARAM_RESET, 1);
                }
                self.stream_info.num_lost_access_units = 0;
                self.stream_info.num_bad_bytes = 0;
                self.stream_info.num_total_bytes = 0;
            }
            AAC_CONCEAL_METHOD => {
                /* Changing the concealment method can introduce additional bitstream delay. And
                that in turn affects sub libraries and modules which makes the whole thing quite
                complex.  So the complete changing routine is packed into a helper function which
                keeps all modules and libs in a consistent state even in the case an error
                occures. */
                error_status = self.set_conceal_method(value);
                if error_status == AAC_DEC_OK {
                    self.conceal_method_user = value;
                }
            }

            AAC_CONCEAL_FADEOUT_SLOPE
            | AAC_CONCEAL_FADEIN_SLOPE
            | AAC_CONCEAL_MUTE_RELEASE
            | AAC_CONCEAL_COMFORT_NOISE_LEVEL => {
                let ns = AACDEC_CONCEAL_PARAM_NOT_SPECIFIED;
                let sel = |p| if p == param { value } else { ns };
                error_status = cconcealment_set_params(
                    Some(&mut self.conceal_common_data),
                    ns,
                    sel(AAC_CONCEAL_FADEOUT_SLOPE),
                    sel(AAC_CONCEAL_FADEIN_SLOPE),
                    sel(AAC_CONCEAL_MUTE_RELEASE),
                    sel(AAC_CONCEAL_COMFORT_NOISE_LEVEL),
                );
            }

            _ => return AAC_DEC_SET_PARAM_FAIL,
        } /* switch(param) */

        if err_tp != TRANSPORTDEC_OK && error_status == AAC_DEC_OK {
            error_status = AAC_DEC_SET_PARAM_FAIL;
        }

        if error_status == AAC_DEC_OK {
            /* Check error code returned by MPEG-D DRC decoder library: */
            match uni_drc_err {
                0 => {}
                -9998 => error_status = AAC_DEC_INVALID_HANDLE,
                _ => error_status = AAC_DEC_SET_PARAM_FAIL,
            }
        }

        error_status
    }

    /// `aacDecoder_SetParamVector(self, param, pValue)`: set one vector of decoder parameters.
    pub fn aac_decoder_set_param_vector(
        &mut self,
        param: AacdecParam,
        p_value: &[i16],
    ) -> AacDecoderError {
        match param {
            AAC_CONCEAL_FADEOUT_ATTENUATION_VECTOR => cconcealment_set_attenuation(
                Some(&mut self.conceal_common_data),
                Some(p_value),
                None,
            ),
            AAC_CONCEAL_FADEIN_ATTENUATION_VECTOR => cconcealment_set_attenuation(
                Some(&mut self.conceal_common_data),
                None,
                Some(p_value),
            ),
            _ => AAC_DEC_SET_PARAM_FAIL,
        }
    }

    /// `aacDecoder_Open(transportFmt, nrOfLayers)`: open an AAC decoder instance.
    pub fn aac_decoder_open(
        transport_fmt: TransportType,
        nr_of_layers: u32,
    ) -> Option<Box<AacDecoderInstance>> {
        let nr_of_layers_min = nr_of_layers.min(TPDEC_MAX_LAYERS as u32);

        /* Allocate transport layer struct. */
        let mut p_in = TransportDec::open(transport_fmt, 0, nr_of_layers_min)?;

        /* Allocate AAC decoder core struct. */
        let mut aac_dec = AacDecoderInstance::caac_decoder_open(transport_fmt)?;

        aac_dec.nr_of_layers = nr_of_layers_min as i32;

        aac_dec.target_layout = 6; /* default channel layout is 5.1 */
        p_in.set_param(TPDEC_PARAM_TARGETLAYOUT, aac_dec.target_layout);

        /* The callbacks (config update, free memory, config switch control, truncation, uniDrc,
         * downmix matrix, earcons, user interaction) are the TpCallbacks implementation of the
         * instance. */
        aac_dec.fdk_mpegh_ui_initialize(&p_in.asc);
        aac_dec.h_input = Some(p_in);

        aac_dec.default_target_loudness = 96;

        earcon_decoder_init(&mut rs!(aac_dec).earcon_decoder);

        rs!(aac_dec).h_limiter = TDLimiter::create(
            TDL_MPEGH3DA_DEFAULT_ATTACK,
            TDL_RELEASE_DEFAULT_MS,
            MAXVAL_DBL,
            24,
            48000,
        );
        rs!(aac_dec).h_limiter.as_ref()?;
        aac_dec.limiter_enable_user = 255; /* (UCHAR)-1 */
        aac_dec.limiter_enable_curr = 0;
        rs!(aac_dec).discard_samples_at_start_cnt = -1; /* initialized for startup detection */

        rs!(aac_dec).truncate_start_offset = [-128; TRUNC_QUEUE_SIZE];
        rs!(aac_dec).truncate_stop_offset = [-128; TRUNC_QUEUE_SIZE];

        /* Assure that all modules have same delay */
        let method = cconcealment_get_method(Some(&aac_dec.conceal_common_data));
        if aac_dec.set_conceal_method(method) != 0 {
            return None;
        }

        Some(aac_dec)
    }

    /// `aacDecoder_Fill(self, pBuffer, bufferSize, pBytesValid)`: fill the decoder's internal
    /// input buffer with bitstream data from the external input buffers (one per layer; the
    /// buffer sizes are the slice lengths).
    pub fn aac_decoder_fill(
        &mut self,
        p_buffer: &[&[u8]],
        p_bytes_valid: &mut [u32],
    ) -> AacDecoderError {
        let nr_of_layers = self.nr_of_layers as usize;
        let Some(tp) = self.h_input.as_deref_mut() else {
            return AAC_DEC_INVALID_HANDLE;
        };
        if p_buffer.len() < nr_of_layers || p_bytes_valid.len() < nr_of_layers {
            return AAC_DEC_INVALID_PARAM;
        }

        for layer in 0..nr_of_layers {
            let tp_err = tp.fill_data(p_buffer[layer], &mut p_bytes_valid[layer], layer as i32);
            if tp_err != TRANSPORTDEC_OK {
                return AAC_DEC_UNKNOWN; /* Must be an internal error */
            }
        }

        AAC_DEC_OK
    }

    /// `aacDecoder_SignalInterruption(self)`
    fn aac_decoder_signal_interruption(&mut self) {
        self.caac_decoder_signal_interruption();

        self.ui_status_valid = 0;
        self.drc_status_valid = 0;
    }

    /// `aacDecoder_DecodeFrame(self, pTimeData, timeDataSize, flags)`: decode one audio frame
    /// into `p_time_data` (interleaved; `timeDataSize = p_time_data.len()`).
    pub fn aac_decoder_decode_frame(
        &mut self,
        p_time_data: &mut [IntPcm],
        flags: u32,
    ) -> AacDecoderError {
        let Some(mut tp) = self.h_input.take() else {
            return AAC_DEC_INVALID_HANDLE;
        };
        let status = self.aac_decoder_decode_frame_tp(&mut tp, p_time_data, flags, false);
        self.h_input = Some(tp);
        match status {
            DecodeStatus::Done(err) => err,
            DecodeStatus::Deferred(_) => unreachable!("rendering deferred"),
        }
    }

    /// `aacDecoder_DecodeFrame()` with frame pipelining (not in C): like
    /// [`AacDecoderInstance::aac_decoder_decode_frame`], but if a render thread is set
    /// ([`AacDecoderInstance::set_render_worker`]) and the access unit allows it, only the core
    /// decoding is done: the rendering runs on the render thread
    /// ([`DecodeStatus::Deferred`]) and its output is returned later by
    /// [`AacDecoderInstance::take_completed`] (after the output of the frames rendered before;
    /// the caller has to handle these results before the output of later calls). `p_time_data`
    /// only receives the output of [`DecodeStatus::Done`] frames; its length is also the output
    /// buffer size of the deferred frames.
    ///
    /// The results are the same as with the sequential calls, except for the bit rate and the
    /// byte and access unit counters of the stream info, which the core stage maintains with
    /// the frame size before truncation.
    pub fn aac_decoder_decode_frame_pipelined(
        &mut self,
        p_time_data: &mut [IntPcm],
        flags: u32,
    ) -> DecodeStatus {
        let Some(mut tp) = self.h_input.take() else {
            return DecodeStatus::Done(AAC_DEC_INVALID_HANDLE);
        };
        let allow = self.render_worker.is_some();
        let status = self.aac_decoder_decode_frame_tp(&mut tp, p_time_data, flags, allow);
        self.h_input = Some(tp);
        status
    }

    #[allow(unused_assignments)]
    fn aac_decoder_decode_frame_tp(
        &mut self,
        tp: &mut TransportDec,
        p_time_data: &mut [IntPcm],
        flags: u32,
        allow_defer: bool,
    ) -> DecodeStatus {
        let time_data_size = p_time_data.len() as i32;
        let mut deferred: Option<DeferredFrame> = None;
        let mut error_status: AacDecoderError = AAC_DEC_OK;
        let mut f_tp_interruption = 0; /* Transport originated interruption detection. */
        let mut f_tp_conceal = 0; /* Transport originated concealment. */
        let stream_index: usize = 0;
        let mut truncate_frame_size: i32 = -1; /* not set */
        let mut access_unit: u32 = 0;
        let mut num_access_units: u32 = 1;
        let mut num_preroll_au: u32 = 0;
        let mut f_end_au_not_adjusted = 0; /* The end of the access unit was not adjusted */

        if flags & AACDEC_INTR != 0 {
            self.stream_info.num_lost_access_units = 0;
        }
        /* frame pipelining: a flushing or build-up may end during the core decoding of this
         * access unit; its rendering still interacts with the core decoding of the next one */
        let flushing_at_start =
            self.flush_status != AACDEC_FLUSH_OFF || self.build_up_status != AACDEC_BUILD_UP_OFF;
        self.stream_info.mpegh_au_size = 0;

        if self.poison & POISON_BEFORE_CORE != 0 {
            self.poison_buffers(true, true);
        }
        if self.poison & POISON_DIGEST_SCRATCH != 0 {
            self.poison_digest_scratch();
        }

        /* Get current bits position for bitrate calculation. */
        let n_bits = tp.bit_stream[0].get_valid_bits();

        'bail: {
            if !((flags & (AACDEC_CONCEAL | AACDEC_FLUSH) != 0)
                || (self.flush_status == AACDEC_MPEGH_DASH_IPF_ATSC_FLUSH_ON)
                || (self.flush_status == AACDEC_USAC_DASH_IPF_FLUSH_ON)
                || (self.build_up_status == AACDEC_MPEGH_BUILD_UP_IDLE_IN_BAND))
            {
                for layer in 0..self.nr_of_layers as u32 {
                    let err = tp.read_access_unit(layer, self);
                    if err != TRANSPORTDEC_OK {
                        match err {
                            TRANSPORTDEC_NOT_ENOUGH_BITS => {
                                error_status = AAC_DEC_NOT_ENOUGH_BITS;
                                break 'bail;
                            }
                            TRANSPORTDEC_SYNC_ERROR => {
                                if tt_is_packet(tp.get_format()) {
                                    error_status = AAC_DEC_PARSE_ERROR;
                                    f_tp_conceal = 1;
                                } else {
                                    let mut n = 0;
                                    tp.get_missing_access_unit_count(&mut n);
                                    self.stream_info.num_lost_access_units = n;
                                    f_tp_interruption = 1;
                                }
                            }
                            TRANSPORTDEC_NEED_TO_RESTART => {
                                error_status = AAC_DEC_NEED_TO_RESTART;
                                break 'bail;
                            }
                            TRANSPORTDEC_CRC_ERROR => {
                                f_tp_conceal = 1;
                            }
                            TRANSPORTDEC_UNSUPPORTED_FORMAT => {
                                error_status = AAC_DEC_UNSUPPORTED_FORMAT;
                                break 'bail;
                            }
                            _ => {
                                error_status = AAC_DEC_UNKNOWN;
                                break 'bail;
                            }
                        }
                    }
                }
            } else if self.stream_info.num_lost_access_units > 0 {
                self.stream_info.num_lost_access_units -= 1;
            }

            self.frame_ok = 1;

            if self.build_up_status == AACDEC_MPEGH_BUILD_UP_IDLE {
                /* return without decoding audio to allow UI persistency module to restore
                 * settings first */
                self.stream_info.frame_size = 0;
                error_status = AAC_DEC_INTERMEDIATE_OK;
                break 'bail;
            }

            let mut preroll_au_offset = [0u32; AACDEC_MAX_NUM_PREROLL_AU * TPDEC_MAX_LAYERS];
            self.preroll_au_length = [0; AACDEC_MAX_NUM_PREROLL_AU + 1];

            let mut au_start_anchor = [0i32; TPDEC_MAX_LAYERS];

            /* Process preroll frames and current frame */
            loop {
                if (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) == 0)
                    && (self.flush_status != AACDEC_MPEGH_CFG_CHANGE_ATSC_FLUSH_ON)
                    && (access_unit == 0)
                    && (self.has_audio_pre_roll != 0
                        || (self.build_up_status == AACDEC_MPEGH_BUILD_UP_IDLE_IN_BAND))
                    && f_tp_interruption == 0
                    && f_tp_conceal == 0
                /* Bit stream pointer needs to be at the beginning of a (valid) AU. */
                {
                    error_status = self.caac_decoder_pre_roll_extension_payload_parse(
                        tp,
                        &mut num_preroll_au,
                        &mut preroll_au_offset,
                    );

                    if error_status != AAC_DEC_OK {
                        match error_status {
                            AAC_DEC_NOT_ENOUGH_BITS => break 'bail,
                            AAC_DEC_PARSE_ERROR => self.frame_ok = 0,
                            _ => {}
                        }
                    }

                    num_access_units += num_preroll_au;
                }

                if self.build_up_status == AACDEC_MPEGH_BUILD_UP_IDLE_IN_BAND {
                    /* return without decoding audio to allow UI persistency module to restore
                     * settings first */
                    self.stream_info.frame_size = 0;
                    error_status = AAC_DEC_INTERMEDIATE_OK;
                    break 'bail;
                }

                if access_unit > 0 && self.poison & POISON_BEFORE_CORE != 0 {
                    self.poison_buffers(true, true);
                }

                for i in (0..TPDEC_MAX_LAYERS).rev() {
                    let h_bs_au = &mut tp.bit_stream[i];
                    au_start_anchor[i] = h_bs_au.get_valid_bits();

                    if au_start_anchor[i] > 0 && access_unit < num_preroll_au {
                        h_bs_au.push_for(preroll_au_offset[access_unit as usize + i]);
                    }
                }
                self.access_unit = access_unit as i32;

                /* Signal bit stream interruption to other modules if required. */
                if f_tp_interruption != 0 || ((flags & AACDEC_INTR != 0) && (access_unit == 0)) {
                    self.aac_decoder_signal_interruption();
                    if !((flags & AACDEC_INTR != 0) && (access_unit == 0)) {
                        error_status = AAC_DEC_TRANSPORT_SYNC_ERROR;
                        break 'bail;
                    }
                }

                /* Empty bit buffer in case of flush request. */
                if (flags & AACDEC_FLUSH != 0)
                    && (flags & AACDEC_CONCEAL == 0)
                    && self.flush_status == 0
                {
                    tp.set_param(TPDEC_PARAM_RESET, 1);
                    self.stream_info.num_lost_access_units = 0;
                    self.stream_info.num_bad_bytes = 0;
                    self.stream_info.num_total_bytes = 0;
                }
                /* Reset the output delay field. The modules will add their figures one after
                 * another. */
                self.stream_info.output_delay = 0;

                if self.limiter_enable_user == 255 {
                    /* Enable limiter for all non-lowdelay and non-HD AOT's. */
                    self.limiter_enable_curr =
                        if self.flags[0] & (AC_LD | AC_ELD | AC_HDAAC) != 0 { 0 } else { 1 };
                } else {
                    /* Use limiter configuration as requested. */
                    self.limiter_enable_curr = self.limiter_enable_user;
                }

                let time_data2_size = self.time_data2_size / core::mem::size_of::<PcmDec>() as i32;

                /* conceal if UI status invalid */
                if self.update_ui_status(&tp.asc) < 0 {
                    self.frame_ok = 0;
                }

                error_status = self.caac_decoder_decode_frame(
                    tp,
                    flags
                        | if f_tp_conceal != 0 { AACDEC_CONCEAL } else { 0 }
                        | if self.flush_status != 0 && (flags & AACDEC_CONCEAL == 0) {
                            AACDEC_FLUSH
                        } else {
                            0
                        },
                    256,
                    time_data2_size - 256,
                    self.stream_info.aac_samples_per_frame + 256,
                );

                if is_output_valid(error_status) && self.poison & POISON_BEFORE_RENDER != 0 {
                    self.poison_buffers(self.poison & POISON_TIME_SIGNAL != 0, true);
                }

                let mut time_data_headroom = self.aac_out_data_headroom;

                if !((flags & (AACDEC_CONCEAL | AACDEC_FLUSH) != 0)
                    || f_tp_conceal != 0
                    || self.flush_status != 0)
                    && (!is_output_valid(error_status) || access_unit >= num_preroll_au)
                {
                    let tp_err = tp.end_access_unit();
                    if tp_err != TRANSPORTDEC_OK {
                        self.frame_ok = 0;
                    }
                } else if is_output_valid(error_status) {
                    /* while preroll processing later possibly an error in the renderer part
                     * occurrs */
                    f_end_au_not_adjusted = 1;
                }

                /* If the current pTimeData2 does not contain a valid signal, there nothing else we
                 * can do, so bail. */
                if !is_output_valid(error_status) {
                    break 'bail;
                }

                self.stream_info.sample_rate = self.stream_info.aac_sample_rate;
                self.stream_info.frame_size = self.stream_info.aac_samples_per_frame;

                self.stream_info.num_channels = self.stream_info.aac_num_channels;

                /* MPEG-H Rendering */

                if (self.flags[0] & AC_MPEGH3DA != 0)
                    && (self.target_layout_config >= 0)
                    && (self.stream_info.num_channels > 0)
                {
                    self.join_render();

                    /* apply user interactivity (its channel gains are applied by the rendering) */
                    let (error_ui, channel_gain) = self.apply_user_interactivity(&tp.asc);
                    /* transportDec_SetParam() calls of applyUserInteractivity() */
                    for (param, value) in core::mem::take(&mut self.pending_tp_params) {
                        tp.set_param(param, value);
                    }
                    if error_ui != AAC_DEC_OK {
                        error_status = error_ui;
                        if !is_output_valid(error_status) {
                            break 'bail;
                        }
                    }
                    let ctx = self.render_ctx(&tp.asc, channel_gain);

                    if allow_defer
                        && !flushing_at_start
                        && self.can_defer_render(
                            &ctx,
                            flags,
                            access_unit,
                            num_access_units,
                            time_data_size,
                        )
                    {
                        /* The rendering (the rest of this function for this access unit) runs
                         * on the render thread; this thread continues with the core decoding
                         * of the next access unit. */
                        let (num_channels, frame_size, sample_rate) = self.predict_render(&ctx);
                        self.launch_render(ctx, error_status, p_time_data.len());
                        deferred =
                            Some(DeferredFrame { err: error_status, sample_rate, num_channels });

                        /* the stream info of the core stage (the render stage returns its own) */
                        self.stream_info.num_channels = num_channels;
                        self.stream_info.frame_size = frame_size;
                        self.stream_info.sample_rate = sample_rate;
                        self.stream_info.sample_rate_before_rs = sample_rate;
                        self.stream_info.num_channels_before_mix = num_channels;
                        /* Update externally visible copy of flags */
                        self.stream_info.flags = self.flags[0];
                        /* no flushing, no pre-roll, no build-up: a single access unit */
                        access_unit += 1;
                        break;
                    }

                    let rs = self.render.as_deref_mut().expect("render state");
                    let rendered = rs.render(
                        &ctx,
                        &mut self.stream_info,
                        &mut self.p_time_data2,
                        &mut self.work_buffer_core2,
                        self.thread_pool.as_deref(),
                        &mut error_status,
                    );
                    if !rendered {
                        break 'bail;
                    }

                    /* MPEG-H AU size, truncation, cross-fade and earcons */
                    if let Some(t) = rs.render_tail(
                        &ctx,
                        &mut self.stream_info,
                        &mut self.p_time_data2,
                        flags,
                        access_unit,
                        num_preroll_au,
                        num_access_units,
                    ) {
                        truncate_frame_size = t;
                    }

                    /* Target Layout dependency */
                    debug_assert!(
                        cicp2geometry_get_num_channels_from_cicp_noerr(self.target_layout_config)
                            != 0
                    );

                    time_data_headroom = PCM_OUT_HEADROOM;
                } else {
                    for ii in 0..self.stream_info.aac_num_channels.max(0) as usize {
                        let fsz = self.stream_info.frame_size as usize;
                        let src =
                            ii * (self.stream_info.aac_samples_per_frame + 256) as usize + 256;
                        self.p_time_data2.copy_within(src..src + fsz, ii * fsz);
                    }
                }

                self.stream_info.sample_rate_before_rs = self.stream_info.sample_rate;

                if !((self.flags[0] & AC_MPEGH3DA != 0)
                    && (self.target_layout_config >= 0)
                    && (self.stream_info.num_channels > 0))
                    && PCM_OUT_HEADROOM != time_data_headroom
                {
                    let n = (self.stream_info.frame_size * self.stream_info.num_channels).max(0)
                        as usize;
                    scale_values(
                        &mut self.p_time_data2[..n],
                        -(PCM_OUT_HEADROOM - time_data_headroom),
                    );
                }

                rs!(self).output_loudness(&mut self.stream_info, self.default_target_loudness);

                self.stream_info.num_channels_before_mix = self.stream_info.num_channels;

                /* Signal interruption to take effect in next frame. */
                if (flags & AACDEC_FLUSH != 0 || self.flush_status != 0)
                    && (flags & AACDEC_CONCEAL == 0)
                {
                    self.aac_decoder_signal_interruption();
                }

                /* Update externally visible copy of flags */
                self.stream_info.flags = self.flags[0];

                if access_unit < num_preroll_au {
                    for i in (0..TPDEC_MAX_LAYERS).rev() {
                        let h_bs_au = &mut tp.bit_stream[i];
                        if au_start_anchor[i] > 0 {
                            let vb = h_bs_au.get_valid_bits();
                            h_bs_au.push_back((au_start_anchor[i] - vb) as u32);
                        }
                    }
                } else if (self.build_up_status == AACDEC_MPEGH_BUILD_UP_ON)
                    || (self.build_up_status == AACDEC_MPEGH_BUILD_UP_ON_IN_BAND)
                    || (self.build_up_status == AACDEC_USAC_BUILD_UP_ON)
                {
                    self.build_up_cnt = self.build_up_cnt.wrapping_sub(1);

                    if self.build_up_cnt < 0 {
                        self.build_up_status = 0;
                    }
                }

                if self.flush_status != AACDEC_USAC_DASH_IPF_FLUSH_ON {
                    access_unit += 1;
                }

                if !((access_unit < num_access_units)
                    || ((self.flush_status == AACDEC_USAC_DASH_IPF_FLUSH_ON)
                        && (flags & AACDEC_CONCEAL == 0)))
                {
                    break;
                }
            }

            if deferred.is_none() && self.stream_info.ext_aot != AOT_AAC_SLS {
                let rs = rs!(self);
                match rs.limiter_output(
                    &mut self.stream_info,
                    &mut self.p_time_data2,
                    &mut self.work_buffer_core2,
                    p_time_data,
                    flags,
                    self.flags[stream_index],
                    self.limiter_enable_curr,
                    truncate_frame_size,
                    access_unit,
                    num_preroll_au,
                ) {
                    AAC_DEC_OK => {}
                    AAC_DEC_SET_PARAM_FAIL => return DecodeStatus::Done(AAC_DEC_SET_PARAM_FAIL),
                    e => {
                        error_status = e;
                        break 'bail;
                    }
                }
            } /* if (self->streamInfo.extAot != AOT_AAC_SLS)*/
        }

        /* error in renderer part occurred, ErrorStatus was set to invalid output */
        if f_end_au_not_adjusted != 0
            && !is_output_valid(error_status)
            && (access_unit < num_preroll_au)
        {
            tp.end_access_unit();
        }

        /* Update Statistics */
        aac_decoder_update_bit_stream_counters(
            &mut self.stream_info,
            &mut tp.bit_stream[0],
            n_bits,
            error_status,
        );

        if !(is_output_valid(error_status) || (error_status == AAC_DEC_INTERMEDIATE_OK)) {
            self.stream_info.mpegh_au_size = -1;
            self.stream_info.frame_size = 0;
        }

        /*Logical AND of the independent frame flag and "frame decoded correctly" flag*/
        self.stream_info.f_bs_restart_ok &= if self.frame_ok != 0 { 1 } else { 0 };

        match deferred {
            Some(d) => DecodeStatus::Deferred(d),
            None => DecodeStatus::Done(error_status),
        }
    }

    /// `aacDecoder_GetStreamInfo(self)`
    pub fn aac_decoder_get_stream_info(&self) -> &CStreamInfo {
        &self.stream_info
    }
}

/// `aacDecoder_UpdateBitStreamCounters(pSi, hBs, nBits, ErrorStatus)`
fn aac_decoder_update_bit_stream_counters(
    p_si: &mut CStreamInfo,
    h_bs: &mut FdkBitStream,
    n_bits: i32,
    error_status: AacDecoderError,
) {
    /* calculate bit difference (amount of bits moved forward) */
    let n_bits = n_bits - h_bs.get_valid_bits();

    /* Calc bitrate. */
    if p_si.frame_size > 0 {
        /* bitRate = nBits * sampleRate / frameSize */
        let mut ratio_e = 0;
        let ratio_m = f_div_norm(p_si.sample_rate, p_si.frame_size, &mut ratio_e);
        p_si.bit_rate = f_mult_norm_to(n_bits, DFRACT_BITS - 1, ratio_m, ratio_e, DFRACT_BITS - 1);
    }

    /* bit/byte counters */
    {
        let n_bytes = n_bits >> 3;
        p_si.num_total_bytes += n_bytes as i64;
        if is_output_valid(error_status) {
            p_si.num_total_access_units += 1;
        }
        if is_decode_error(error_status) {
            p_si.num_bad_bytes += n_bytes as i64;
            p_si.num_bad_access_units += 1;
        }
    }
}

impl TpCallbacks for AacDecoderInstance {
    /// `aacDecoder_ConfigCallback(handle, pAscStruct, configMode, configChanged)`
    fn update_config(
        &mut self,
        ascs: &[CSAudioSpecificConfig],
        index: usize,
        config_mode: u8,
        config_changed: &mut u8,
    ) -> i32 {
        let err = self.aac_decoder_config(ascs, index, config_mode, config_changed);

        let err_tp = if err == AAC_DEC_OK {
            TRANSPORTDEC_OK
        } else if err == AAC_DEC_NEED_TO_RESTART {
            TRANSPORTDEC_NEED_TO_RESTART
        } else if is_init_error(err) {
            TRANSPORTDEC_UNSUPPORTED_FORMAT
        } else {
            /* Fatal errors */
            TRANSPORTDEC_UNKOWN_ERROR
        };

        if *config_changed != 0 && (config_mode & AC_CM_DET_CFG_CHANGE != 0) {
            self.h_ui_manager.asi.active_dmx_id = self.downmix_id as u8;
        }

        err_tp
    }

    /// `aacDecoder_FreeMemCallback(handle, pAscStruct)`
    fn free_mem(&mut self, ascs: &[CSAudioSpecificConfig], index: usize) -> i32 {
        let asc = &ascs[index];
        let mut sub_stream_index = 0;

        if asc.m_aot == AOT_MPEGH3DA {
            sub_stream_index = asc.m_sc.sub_stream_index as usize;
        }

        if self.caac_decoder_free_mem(ascs, sub_stream_index, asc.config_mode) != AAC_DEC_OK {
            TRANSPORTDEC_UNKOWN_ERROR
        } else {
            TRANSPORTDEC_OK
        }
    }

    fn take_pending_param(&mut self) -> Option<(i32, i32)> {
        if self.pending_tp_params.is_empty() {
            None
        } else {
            Some(self.pending_tp_params.remove(0))
        }
    }

    /// `aacDecoder_CtrlCFGChangeCallback(handle, pCtrlCFGChangeStruct)`
    fn ctrl_cfg_change(&mut self, ctrl: &CCtrlCFGChange) -> i32 {
        self.caac_decoder_ctrl_cfg_change(
            ctrl.flush_status,
            ctrl.flush_cnt,
            ctrl.build_up_status,
            ctrl.build_up_cnt,
        );
        if ctrl.force_crossfade != 0 {
            /* demand cross-fade between frames at next config change */
            rs!(self).apply_crossfade |= AACDEC_CROSSFADE_BITMASK_FORCE;
        }

        TRANSPORTDEC_OK
    }

    /// `aacDecoder_TruncationMsgCallback(handle, nTruncSamples, truncType)`
    fn truncation_msg(&mut self, n_trunc_samples: i32, trunc_type: i32) -> i32 {
        let delay = 3 * 256 + TD_UPSAMPLER_MAX_DELAY;

        /* Determine truncation boundaries */
        let (start_offset, stop_offset) = if trunc_type == 0 {
            /* right truncation */
            if self.truncate_frame_size <= 0 {
                return TRANSPORTDEC_PARSE_ERROR;
            }
            (
                delay + self.truncate_frame_size as i32 - n_trunc_samples,
                delay + self.truncate_frame_size as i32,
            )
        } else if trunc_type == 1 {
            /* left truncation */
            (delay, delay + n_trunc_samples)
        } else {
            /* generate truncation for config change without truncation message */
            (delay, delay)
        };

        let mut q = 0;
        while q < TRUNC_QUEUE_SIZE {
            if (rs!(self).truncate_start_offset[q] > 0 || rs!(self).truncate_stop_offset[q] > 0)
                && !(rs!(self).truncate_start_offset[q] as i32 == stop_offset
                    || rs!(self).truncate_stop_offset[q] as i32 >= start_offset)
            {
                q += 1;
                continue;
            }

            /* The truncation segment list is required to be ordered. Reorder in case required. */
            if q < (TRUNC_QUEUE_SIZE - 1)
                && (rs!(self).truncate_start_offset[q + 1] > 0
                    || rs!(self).truncate_stop_offset[q + 1] > 0)
            {
                rs!(self).truncate_start_offset.copy_within(q + 1..TRUNC_QUEUE_SIZE, q);
                rs!(self).truncate_stop_offset.copy_within(q + 1..TRUNC_QUEUE_SIZE, q);
                rs!(self).truncate_start_offset[TRUNC_QUEUE_SIZE - 1] = -128;
                rs!(self).truncate_stop_offset[TRUNC_QUEUE_SIZE - 1] = -128;
                q += 1;
                continue;
            }

            rs!(self).truncate_sample_count =
                (rs!(self).truncate_sample_count as i32 + n_trunc_samples) as i16;
            if rs!(self).truncate_stop_offset[q] > 0 {
                rs!(self).truncate_start_offset[q] =
                    (rs!(self).truncate_start_offset[q] as i32).min(start_offset) as i16;
                rs!(self).truncate_stop_offset[q] =
                    (rs!(self).truncate_stop_offset[q] as i32).max(stop_offset) as i16;
            } else {
                rs!(self).truncate_start_offset[q] = start_offset as i16;
                rs!(self).truncate_stop_offset[q] = stop_offset as i16;
            }
            if trunc_type == 1 {
                /* left truncation */
                if q == 0 && rs!(self).discard_samples_at_start_cnt == -1 {
                    rs!(self).discard_samples_at_start_cnt = -2;
                }
            } else {
                /* right truncation or generated truncation for config change without truncation
                 * message */
                if q == 0 && rs!(self).discard_samples_at_start_cnt == -2 {
                    rs!(self).discard_samples_at_start_cnt = -1;
                }
            }

            break;
        }

        TRANSPORTDEC_OK
    }

    /// `aacDecoder_UniDrcCallback(handle, hBs, fullPayloadLength, payloadType, subStreamIndex,
    /// payloadStart, aot)`
    fn uni_drc(
        &mut self,
        bs: Option<&mut FdkBitStream>,
        full_payload_length: i32,
        payload_type: i32,
        sub_stream_index: i32,
        payload_start: i32,
        aot: AudioObjectType,
    ) -> i32 {
        let dummy_buffer = [0u8; 4];
        let mut dummy_bs;
        let h_read_bs: &mut FdkBitStream = match bs {
            None => {
                if payload_type == 2 {
                    /* clear downmix information in uniDrcConfig */
                    rs!(self).h_uni_drc_decoder.set_downmix_instructions(0, &[], &[], &[]);
                    return TRANSPORTDEC_OK;
                }
                /* use dummy zero payload to clear memory */
                dummy_bs = FdkBitStream::reader(&dummy_buffer, 24);
                &mut dummy_bs
            }
            Some(b) => b,
        };

        let drc_dec_codec_mode = if aot == AOT_MPEGH3DA {
            DRC_DEC_MPEG_H_3DA
        } else if aot == AOT_USAC {
            DRC_DEC_MPEG_D_USAC
        } else {
            DRC_DEC_CODEC_MODE_UNDEFINED
        };

        let err = rs!(self).h_uni_drc_decoder.set_codec_mode(drc_dec_codec_mode);
        if err != 0 {
            return TRANSPORTDEC_UNKOWN_ERROR;
        }

        let err = if payload_type == 0 {
            /* uniDrcConfig */
            rs!(self).h_uni_drc_decoder.read_uni_drc_config(h_read_bs, sub_stream_index)
        } else {
            /* loudnessInfoSet */
            let e = rs!(self).h_uni_drc_decoder.read_loudness_info_set(h_read_bs, sub_stream_index);
            self.loudness_info_set_position[1] = payload_start;
            self.loudness_info_set_position[2] = full_payload_length;
            e
        };

        if err == DRC_DEC_OK {
            TRANSPORTDEC_OK
        } else {
            TRANSPORTDEC_UNKOWN_ERROR
        }
    }

    /// `mpegh3daInteractivity(user_data, hBs, pac_type, mhasPacketLength)`
    fn user_interact(
        &mut self,
        bs: &mut FdkBitStream,
        packet_type: MhaPactyp,
        mhas_packet_length: u32,
    ) -> i32 {
        self.mpegh3da_interactivity(bs, packet_type, mhas_packet_length)
    }

    /// `aacDecoder_ParseDmxMatrixCallback(handle, hBs, usc)`
    fn parse_dmx_matrix(&mut self, bs: &mut FdkBitStream, usc: &mut CSUsacConfig) -> i32 {
        let mut matching_transmitted_dmx_matrix_found = 0;

        let target_layout = usc.target_layout;
        let num_signal_groups = usc.bs_num_signal_groups as usize;
        let downmix_config_type = usc.downmix_config_type as i32;

        self.join_render();
        /* borrow memory from pTimeData2: the data will be read by DecodeDownmixMatrix() during
         * CAacDecoder_InitRenderer() */
        self.p_time_data2[..DOWNMIX_GROUPS_MATRIX_SET_WORDS].fill(0);
        let mut groups_downmix_matrix_set = DownmixGroupsMatrixSet::default();

        let mut err = downmix_matrix_set(
            bs,
            &mut groups_downmix_matrix_set,
            target_layout,
            downmix_config_type,
            &mut self.downmix_id,
            &mut self.render.as_deref_mut().expect("render state").h_uni_drc_decoder,
        );
        store_downmix_matrix_set(&groups_downmix_matrix_set, &mut self.p_time_data2);

        if err == 0 {
            self.h_ui_manager.asi.active_dmx_id = (self.downmix_id & 0xFF) as u8; /* Truncation is verified */

            let set = &groups_downmix_matrix_set;
            for i in 0..num_signal_groups {
                if usc.m_signal_group_type[i].type_ == 0 {
                    let dmx_index = set.downmix_matrix.get(i).copied().unwrap_or(0) as usize;
                    if set.downmix_matrix_size.get(dmx_index).copied().unwrap_or(0) > 0 {
                        debug_assert!(usc.m_signal_group_type[i].b_use_custom_downmix_matrix == 0);
                        /* downmixMatrix parsed for the signal group i. */
                        usc.m_signal_group_type[i].b_use_custom_downmix_matrix = 1;
                        /* signal transmitted dmx matrix. */
                        matching_transmitted_dmx_matrix_found = 1;
                    }
                }
            }

            /* Sanity check for transmitted downmix matrix parsing. Every bUseCustomDownmixMatrix
             * must be 1, if matchingTransmittedDmxMatrixFound = 1 */
            for i in 0..num_signal_groups {
                if (usc.m_signal_group_type[i].b_use_custom_downmix_matrix == 0)
                    && (usc.m_signal_group_type[i].type_ == 0)
                    && matching_transmitted_dmx_matrix_found == 1
                {
                    /* No custom dmx matrix for this channels signal group i but a transmitted
                     * dmx matrix was found for one signal group -> invalid bitstream */
                    err = TRANSPORTDEC_PARSE_ERROR;
                }
            }
        }

        if err == 0 {
            TRANSPORTDEC_OK
        } else {
            TRANSPORTDEC_UNKOWN_ERROR
        }
    }

    /// `aacDecoder_EarconSetBSCallback(handle, bs)`
    fn earcon_bs(&mut self, bs: &mut FdkBitStream) -> i32 {
        if self.render.is_none() && !self.earcon_active {
            /* nothing to do (earconFlag, currentFrameHasEarcon and lastFrameHadEarcon are 0; they
             * are only changed by the transport decoder callbacks), no need to wait for the
             * rendering */
            return 0;
        }
        let ed = &mut rs!(self).earcon_decoder;

        if ed.earcon_config.earcon_flag != 0 {
            ed.current_frame_has_earcon = 1;

            ed.earcon_config.earcon_flag = 0;

            let num_pcm_signals_in_frame = bs.read_bits(7) + 1; // numPcmSignalsInFrame

            if num_pcm_signals_in_frame > EARCON_MAX_NUM_SIGNALS {
                return 0;
            }
            ed.num_pcm_signals_in_frame = num_pcm_signals_in_frame as i32;

            /*Set initial state*/
            if ed.first_frame == 1 {
                /*Set attenuation gain to 1.0f*/
                ed.earcon_gain = 0x7fffffff;
                ed.earcon_shift = PCM_OUT_HEADROOM;
                ed.target_loudness_old = 0;
                ed.m_bs_pcm_loudness_value_old = 0;
                ed.m_bs_pcm_attenuation_gain_old = 0;
                ed.att_gain_store_high_precision = 0x7fffffff;
                ed.att_gain_exp_store_high_precision = 0;
                ed.att_gain_intermediate = 0x7fffffff;
                ed.att_gain_shift_intermediate = 0;
                ed.att_gain_increment_intermediate = 0;
                ed.att_gain_old = 0x7fffffff;
                ed.att_gain_shift_old = 0;
                ed.att_gain_increment_old = 0;
                ed.att_gain_new = 0x7fffffff;
                ed.att_gain_shift_new = 0;
                ed.accumulated_frame_size = 0;
            }

            for _ in 0..num_pcm_signals_in_frame {
                bs.read_bits(7); // pcmSignal_ID[i]
            }

            if ed.earcon_config.m_pcm_has_attenuation_gain == 2 {
                ed.earcon_config.m_bs_pcm_attenuation_gain = bs.read_bits(8); // bsPcmAttenuationGain
            }

            /*If the attenuation does not change any more, keep it as it was (but with high
             * precision)*/
            if ed.earcon_config.m_pcm_has_attenuation_gain == 0
                || ed.earcon_config.m_bs_pcm_attenuation_gain == ed.m_bs_pcm_attenuation_gain_old
            {
                ed.att_gain_new = ed.att_gain_store_high_precision;
                ed.att_gain_shift_new = -ed.att_gain_exp_store_high_precision;
            }

            let mut frame_size: i32 = 0;
            if ed.earcon_config.m_bs_pcm_frame_size_index == 6 {
                frame_size = bs.read_bits(16) as i32; // variable frame size
            }

            if ed.earcon_config.sampling_frequency != 0 {
                match ed.earcon_config.sampling_frequency {
                    48000 => ed.baseframe_size = 1024,
                    32000 => ed.baseframe_size = 1536,
                    24000 => ed.baseframe_size = 2048,
                    16000 => ed.baseframe_size = 3072,
                    _ => {
                        /* unsupported sampling frequency */
                        return 0;
                    }
                }
            }

            let nsig = num_pcm_signals_in_frame as i32;

            /*Sanity check for frameSize*/
            if frame_size > ed.baseframe_size {
                return 0;
            }
            /*Sanity check for buffer fullness*/
            if (ed.accumulated_frame_size + ed.baseframe_size * nsig) > EARCON_BUFFER_SIZE as i32 {
                return 0;
            }

            let write_index = ed.accumulated_frame_size as usize;
            let n = (frame_size * nsig) as usize;
            for i in 0..n {
                ed.earcon_data[write_index + i] = bs.read_bits(16) as i16; // pcmSample in Q0.15
            }

            /*Fill with zeros to the end of the MPEGH frame*/
            let mut mpegh_frame_size = ed.baseframe_size;
            mpegh_frame_size -= ed.earcon_config.m_bs_mhas_truncation_length as i32;
            let diff = 0.max(mpegh_frame_size - frame_size);

            /*Sanity check for buffer fullness*/
            if (ed.accumulated_frame_size + (frame_size + diff) * nsig) > EARCON_BUFFER_SIZE as i32
            {
                return 0;
            }
            if diff != 0 {
                let s = write_index + n;
                ed.earcon_data[s..s + (diff * nsig) as usize].fill(0);
            }

            /*Increase the accumulated frame size*/
            ed.accumulated_frame_size += (frame_size + diff) * nsig;
            ed.earcon_config.m_bs_pcm_frame_size = (frame_size + diff) as u32;
        } else if ed.current_frame_has_earcon == 1 {
            ed.last_frame_had_earcon = 1;
            ed.current_frame_has_earcon = 0;
            return 0;
        } else if ed.last_frame_had_earcon == 1 {
            ed.last_frame_had_earcon = 0;

            let write_index = ed.accumulated_frame_size;
            /*Fill with zeros to the end of the MPEGH frame*/
            let fill_size = ed.baseframe_size * ed.num_pcm_signals_in_frame;
            /*Sanity check for buffer fullness*/
            if (write_index + fill_size) >= EARCON_BUFFER_SIZE as i32 {
                return 0;
            }
            ed.earcon_data[write_index as usize..(write_index + fill_size) as usize].fill(0);

            /*Increase the accumulated frame size*/
            ed.accumulated_frame_size += fill_size;

            ed.earcon_config.m_bs_pcm_frame_size = ed.baseframe_size as u32;

            ed.earcon_config.m_pcm_has_attenuation_gain = 2;
            ed.earcon_config.m_bs_pcm_attenuation_gain = 0;

            return 0;
        }

        0
    }

    /// `aacDecoder_EarconSetConfigCallback(handle, earconConfig)`
    fn earcon_config(&mut self, config: &mut EarconConfig) -> i32 {
        rs!(self).earcon_decoder.earcon_config = *config;
        0
    }

    /// `aacDecoder_EarconSetInfoCallback(handle, earconInfo)`
    fn earcon_info(&mut self, info: &mut EarconInfo) -> i32 {
        rs!(self).earcon_decoder.earcon_info = *info;
        0
    }

    /// `hTp->pASI` (`transportDec_SetAsiParsing(pIn, UI_Manager_GetAsiPointer(hUiManager))`)
    fn asi(&mut self) -> Option<&mut AudioSceneInfo> {
        Some(&mut self.h_ui_manager.asi)
    }

    /// `hTp->pLoudnessInfoSetPosition` (`loudnessInfoSetPosition`)
    fn loudness_info_set_position(&mut self) -> &mut [i32; 3] {
        &mut self.loudness_info_set_position
    }
}
