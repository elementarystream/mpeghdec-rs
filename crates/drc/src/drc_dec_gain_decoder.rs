// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDec_gainDecoder.cpp, drcDec_gainDecoder.h and
// arm/drcDec_gainDecoder_arm.cpp (only contains NEON code, which is not compiled in the reference
// configuration).

//! DRC gain decoder: decoding of the gain sequences and application to the audio signal.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::scale::{saturate_left_shift, scale_value_saturate};
use mpeghdec_fdk::{
    cnt_leading_zeros, f_min, f_mult, f_mult_div2, fl2fxconst_dbl, fl2fxconst_sgl, fx_dbl2fx_sgl,
    FixpDbl, FixpSgl, DFRACT_BITS, MINVAL_DBL,
};

use crate::drc_dec_reader::c_div;
use crate::drc_dec_tools::*;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;
use crate::drc_gain_dec_init::*;
use crate::drc_gain_dec_multiband::OverlapParameters;
use crate::drc_gain_dec_preprocess::prepare_drc_gain;
use crate::drc_gain_dec_process::{process_drc_subband, process_drc_time};

/* Definitions common to gainDecoder submodule */

/// `NUM_LNB_FRAMES`: previous frame + this frame + one frame for DM_REGULAR_DELAY + (maximum
/// delaySamples)/frameSize
pub const NUM_LNB_FRAMES: usize = 5;

/// `SUBBAND_NUM_BANDS_QMF64`
pub const SUBBAND_NUM_BANDS_QMF64: usize = 64;
/// `SUBBAND_ANALYSIS_DELAY_QMF64`
pub const SUBBAND_ANALYSIS_DELAY_QMF64: i32 = 320;
/// `SUBBAND_NUM_BANDS_QMF71` (according to ISO/IEC 23003-1:2007)
pub const SUBBAND_NUM_BANDS_QMF71: usize = 71;
/// `SUBBAND_ANALYSIS_DELAY_QMF71`
pub const SUBBAND_ANALYSIS_DELAY_QMF71: i32 = 320 + 384;
/// `SUBBAND_NUM_BANDS_STFT256` (according to ISO/IEC 23008-3:2015/AMD3)
pub const SUBBAND_NUM_BANDS_STFT256: usize = 256;
/// `SUBBAND_ANALYSIS_DELAY_STFT256`
pub const SUBBAND_ANALYSIS_DELAY_STFT256: i32 = 256;

/// `GAIN_DEC_LOCATION`
pub type GainDecLocation = i32;
/// `GAIN_DEC_DRC1`
pub const GAIN_DEC_DRC1: GainDecLocation = 0;
/// `GAIN_DEC_DRC2_DRC3`
pub const GAIN_DEC_DRC2_DRC3: GainDecLocation = 1;

/// `GAIN_DEC_PARAM`
pub type GainDecParam = i32;
/// `GAIN_DEC_FRAME_SIZE`
pub const GAIN_DEC_FRAME_SIZE: GainDecParam = 0;
/// `GAIN_DEC_SAMPLE_RATE`
pub const GAIN_DEC_SAMPLE_RATE: GainDecParam = 1;

/// Number of entries per row of `subbandGains`: `4 * (1024 / 256)`.
pub const SUBBAND_GAINS_ROW: usize = 4 * (1024 / 256);

/// `LINEAR_NODE_BUFFER`
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct LinearNodeBuffer {
    pub gain_interpolation_type: GainInterpolationType,
    /// number of nodes, saturated to 32
    pub n_nodes: [i32; NUM_LNB_FRAMES],
    /// e = 7
    pub linear_node_gain: [[FixpDbl; 32]; NUM_LNB_FRAMES],
    pub linear_node_time: [[i16; 32]; NUM_LNB_FRAMES],
    /// The node buffer of the frame was written since the gain buffers were initialized (not in
    /// C, for the state digest: see [`DrcGainDecoder`]).
    pub written: [bool; NUM_LNB_FRAMES],
}

impl LinearNodeBuffer {
    pub const ZERO: Self = Self {
        gain_interpolation_type: 0,
        n_nodes: [0; NUM_LNB_FRAMES],
        linear_node_gain: [[0; 32]; NUM_LNB_FRAMES],
        linear_node_time: [[0; 32]; NUM_LNB_FRAMES],
        written: [false; NUM_LNB_FRAMES],
    };

    /// Signals a [hazard](mpeghdec_fdk::digest::hazard) if the node buffer of the frame `ix` is
    /// read but was not written since the gain buffers were initialized (its content is not part
    /// of the state digest). `idx` is the index of the node buffer; the `dummyLnb` is always part
    /// of the digest.
    #[inline]
    pub fn check_read(&self, idx: i32, ix: usize) {
        if (0..DUMMY_LNB as i32).contains(&idx) && !self.written[ix] {
            mpeghdec_fdk::digest::hazard();
        }
    }
}

/// Index of `dummyLnb` in [`DrcGainBuffers::linear_node_buffer`].
pub const DUMMY_LNB: usize = 48;

/// `DRC_GAIN_BUFFERS`
///
/// `linearNodeBuffer[48]` and `dummyLnb` are stored in one array of 49 elements (`dummyLnb` at
/// index [`DUMMY_LNB`]), which is the memory layout of the C structure: C code indexing
/// `linearNodeBuffer[48]` with corrupt configurations accesses `dummyLnb`.
///
/// State digest: see [`DrcGainDecoder`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DrcGainBuffers {
    pub lnb_pointer: i32,
    pub linear_node_buffer: [LinearNodeBuffer; 48 + 1],
}

impl DrcGainBuffers {
    pub const ZERO: Self =
        Self { lnb_pointer: 0, linear_node_buffer: [LinearNodeBuffer::ZERO; 48 + 1] };

    /// `dummyLnb`
    pub fn dummy_lnb(&self) -> &LinearNodeBuffer {
        &self.linear_node_buffer[DUMMY_LNB]
    }

    /// `&linearNodeBuffer[idx]`. Indices beyond `dummyLnb` would be out of bounds in C (undefined
    /// behaviour); they are mapped to `dummyLnb`.
    #[inline]
    pub fn lnb(&self, idx: i32) -> &LinearNodeBuffer {
        if idx as usize > DUMMY_LNB {
            mpeghdec_fdk::deviation::hit();
        }
        &self.linear_node_buffer[(idx as usize).min(DUMMY_LNB)]
    }

    /// Mutable variant of [`DrcGainBuffers::lnb`].
    #[inline]
    pub fn lnb_mut(&mut self, idx: i32) -> &mut LinearNodeBuffer {
        if idx as usize > DUMMY_LNB {
            mpeghdec_fdk::deviation::hit();
        }
        &mut self.linear_node_buffer[(idx as usize).min(DUMMY_LNB)]
    }
}

/// `ACTIVE_DRC` (state digest: see [`DrcGainDecoder`])
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActiveDrc {
    pub active_drc_offset: i32,
    pub drc_inst: DrcInstructionsUniDrc,

    pub ducking_modification_for_channel_group: [DuckingModification; 28],
    pub channel_group_for_channel: [i8; 2 * 28],

    pub band_count_for_channel_group: [u8; 28],
    pub gain_element_for_group: [u8; 28],
    pub channel_group_is_parametric_drc: [u8; 28],
    /// number of different DRC gains including all DRC bands
    pub gain_element_count: u8,
    pub lnb_index_for_channel: [[i32; NUM_LNB_FRAMES]; 28],
    pub subband_gains_ready: i32,
}

impl ActiveDrc {
    pub const ZERO: Self = Self {
        active_drc_offset: 0,
        drc_inst: DrcInstructionsUniDrc::ZERO,
        ducking_modification_for_channel_group: [DuckingModification::ZERO; 28],
        channel_group_for_channel: [0; 2 * 28],
        band_count_for_channel_group: [0; 28],
        gain_element_for_group: [0; 28],
        channel_group_is_parametric_drc: [0; 28],
        gain_element_count: 0,
        lnb_index_for_channel: [[0; NUM_LNB_FRAMES]; 28],
        subband_gains_ready: 0,
    };
}

/// `DRC_GAIN_DECODER`
///
/// `subbandGains[48][4 * (1024 / 256)]` and `dummySubbandGains[4 * (1024 / 256)]` are stored in
/// one flat array (`dummySubbandGains` is the last row), which is the memory layout of the C
/// structure; C code writing beyond a row (frame sizes larger than 1024) continues in the next row.
///
/// State digest: the node buffers of the last [`NUM_LNB_FRAMES`] frames (`nNodes`,
/// `linearNodeGain`, `linearNodeTime` of every `LINEAR_NODE_BUFFER` and `lnbIndexForChannel` of
/// every `ACTIVE_DRC`) form a ring buffer whose newest entry is `lnbPointer`; they are fed from the
/// oldest to the newest frame, without `lnbPointer`. The gain decoding accesses them relative to
/// `lnbPointer`, except for the out-of-bounds read of C for a node buffer without nodes, which
/// signals a [hazard](mpeghdec_fdk::digest::hazard).
///
/// The node buffers of the 48 gain elements are initialized with a layout that depends on the
/// physical position (`initDrcGainBuffers()`: the node of the frame 0 has the time 0) and are not
/// initialized again when other DRC sets become active. Only the node buffers written since the
/// initialization ([`LinearNodeBuffer::written`], not in C) are part of the digest (the nodes up
/// to `nNodes`, which are the ones read); reading another one signals a hazard
/// ([`LinearNodeBuffer::check_read`]). The `dummyLnb` is fed completely.
///
/// Of the `ACTIVE_DRC` entries beyond `nActiveDrcs`, `drcInst` is not part of the digest: it is
/// assigned completely when the entry becomes active (`initActiveDrc()`,
/// `addVirtualToActiveDrc()`) and not read before. Their other fields may be reused by a virtual
/// DRC set and are part of the digest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DrcGainDecoder {
    pub delta_tmin_default: i32,
    pub frame_size: i32,
    pub sample_rate: i32,
    pub ms_per_frame: i32,
    pub loudness_normalisation_gain_db: FixpDbl,
    pub delay_mode: DelayMode,

    pub drc_coef: DrcCoefficientsUniDrc,
    pub n_active_drcs: [i32; ACTIVE_DRC_LOCATIONS],
    pub active_drc: [[ActiveDrc; MAX_ACTIVE_DRCS]; ACTIVE_DRC_LOCATIONS],
    /// relates to location 0: before downmix
    pub multi_band_active_drc_index: i32,
    /// relates to location 0: before downmix
    pub channel_gain_active_drc_index: i32,
    /// e = 8
    pub channel_gain: [FixpDbl; 28],
    /// e = 8
    pub channel_gain_prev: [FixpDbl; 28],

    pub drc_gain_buffers: DrcGainBuffers,
    pub overlap: [[OverlapParameters; 4]; 28],
    /// `subbandGains` followed by `dummySubbandGains`
    pub subband_gains: [FixpDbl; (48 + 1) * SUBBAND_GAINS_ROW],

    pub status: i32,
    pub time_domain_supported: i32,
    /// reverse counter active for time segment 2.5 seconds after startup
    pub startup_ms: i32,
    pub subband_domain_supported: SubbandDomainMode,
}

/// The physical index of the node buffer ring entry `m` (0: oldest) for `lnbPointer`.
fn lnb_ring(lnb_pointer: i32, m: usize) -> usize {
    (lnb_pointer.rem_euclid(NUM_LNB_FRAMES as i32) as usize + 1 + m) % NUM_LNB_FRAMES
}

fn digest_active_drc(
    a: &ActiveDrc,
    active: bool,
    lnb_pointer: i32,
    d: &mut mpeghdec_fdk::digest::Digester,
) {
    let ActiveDrc {
        active_drc_offset,
        drc_inst,
        ducking_modification_for_channel_group,
        channel_group_for_channel,
        band_count_for_channel_group,
        gain_element_for_group,
        channel_group_is_parametric_drc,
        gain_element_count,
        lnb_index_for_channel,
        subband_gains_ready,
    } = a;
    d.field("active_drc_offset", active_drc_offset);
    if active {
        d.field("drc_inst", drc_inst);
    }
    d.field("ducking_modification_for_channel_group", ducking_modification_for_channel_group);
    d.field("channel_group_for_channel", channel_group_for_channel);
    d.field("band_count_for_channel_group", band_count_for_channel_group);
    d.field("gain_element_for_group", gain_element_for_group);
    d.field("channel_group_is_parametric_drc", channel_group_is_parametric_drc);
    d.field("gain_element_count", gain_element_count);
    d.field_with("lnb_index_for_channel", |d| {
        for c in lnb_index_for_channel {
            for m in 0..NUM_LNB_FRAMES {
                c[lnb_ring(lnb_pointer, m)].digest(d);
            }
        }
    });
    d.field("subband_gains_ready", subband_gains_ready);
}

impl StateDigest for DrcGainDecoder {
    fn digest(&self, d: &mut mpeghdec_fdk::digest::Digester) {
        let DrcGainDecoder {
            delta_tmin_default,
            frame_size,
            sample_rate,
            ms_per_frame,
            loudness_normalisation_gain_db,
            delay_mode,
            drc_coef,
            n_active_drcs,
            active_drc,
            multi_band_active_drc_index,
            channel_gain_active_drc_index,
            channel_gain,
            channel_gain_prev,
            drc_gain_buffers,
            overlap,
            subband_gains,
            status,
            time_domain_supported,
            startup_ms,
            subband_domain_supported,
        } = self;
        let DrcGainBuffers { lnb_pointer, linear_node_buffer } = drc_gain_buffers;
        let p = *lnb_pointer;
        d.field("delta_tmin_default", delta_tmin_default);
        d.field("frame_size", frame_size);
        d.field("sample_rate", sample_rate);
        d.field("ms_per_frame", ms_per_frame);
        d.field("loudness_normalisation_gain_db", loudness_normalisation_gain_db);
        d.field("delay_mode", delay_mode);
        d.field("drc_coef", drc_coef);
        d.field("n_active_drcs", n_active_drcs);
        d.field_with("active_drc", |d| {
            for (l, loc) in active_drc.iter().enumerate() {
                for (a, x) in loc.iter().enumerate() {
                    let active = (a as i32) < n_active_drcs[l];
                    d.element_with(l * MAX_ACTIVE_DRCS + a, |d| digest_active_drc(x, active, p, d));
                }
            }
        });
        d.field("multi_band_active_drc_index", multi_band_active_drc_index);
        d.field("channel_gain_active_drc_index", channel_gain_active_drc_index);
        d.field("channel_gain", channel_gain);
        d.field("channel_gain_prev", channel_gain_prev);
        d.field_with("linear_node_buffer", |d| {
            for (i, lnb) in linear_node_buffer.iter().enumerate() {
                d.element_with(i, |d| {
                    let LinearNodeBuffer {
                        gain_interpolation_type,
                        n_nodes,
                        linear_node_gain,
                        linear_node_time,
                        written,
                    } = lnb;
                    let dummy = i == DUMMY_LNB;
                    if dummy || written.iter().any(|&w| w) {
                        gain_interpolation_type.digest(d);
                    }
                    for m in 0..NUM_LNB_FRAMES {
                        let k = lnb_ring(p, m);
                        written[k].digest(d);
                        if dummy {
                            n_nodes[k].digest(d);
                            linear_node_gain[k].digest(d);
                            linear_node_time[k].digest(d);
                        } else if written[k] {
                            let n = n_nodes[k].clamp(0, 32) as usize;
                            n_nodes[k].digest(d);
                            FixpDbl::digest_slice(&linear_node_gain[k][..n], d);
                            i16::digest_slice(&linear_node_time[k][..n], d);
                        }
                    }
                });
            }
        });
        d.field("overlap", overlap);
        d.field("subband_gains", subband_gains);
        d.field("status", status);
        d.field("time_domain_supported", time_domain_supported);
        d.field("startup_ms", startup_ms);
        d.field("subband_domain_supported", subband_domain_supported);
    }
}

const OVERLAP_ZERO_GROUP: [OverlapParameters; 4] = [OverlapParameters::ZERO; 4];

/// Row index of `dummySubbandGains` in [`DrcGainDecoder::subband_gains`].
pub const DUMMY_SUBBAND_GAINS_ROW: usize = 48;

impl DrcGainDecoder {
    pub const ZERO: Self = Self {
        delta_tmin_default: 0,
        frame_size: 0,
        sample_rate: 0,
        ms_per_frame: 0,
        loudness_normalisation_gain_db: 0,
        delay_mode: 0,
        drc_coef: DrcCoefficientsUniDrc::ZERO,
        n_active_drcs: [0; ACTIVE_DRC_LOCATIONS],
        active_drc: [
            [ActiveDrc::ZERO, ActiveDrc::ZERO, ActiveDrc::ZERO],
            [ActiveDrc::ZERO, ActiveDrc::ZERO, ActiveDrc::ZERO],
        ],
        multi_band_active_drc_index: 0,
        channel_gain_active_drc_index: 0,
        channel_gain: [0; 28],
        channel_gain_prev: [0; 28],
        drc_gain_buffers: DrcGainBuffers::ZERO,
        overlap: [OVERLAP_ZERO_GROUP; 28],
        subband_gains: [0; (48 + 1) * SUBBAND_GAINS_ROW],
        status: 0,
        time_domain_supported: 0,
        startup_ms: 0,
        subband_domain_supported: 0,
    };
}

/*******************************************/
/* static functions                        */
/*******************************************/

fn set_channel_gains_internal(
    h_gain_dec: &mut DrcGainDecoder,
    num_channel_gains: i32,
    channel_gain_db: &[FixpDbl],
) {
    for i in 0..num_channel_gains.max(0) as usize {
        if channel_gain_db[i] == MINVAL_DBL {
            h_gain_dec.channel_gain[i] = 0;
        } else {
            /* add loudness normalisation gain (dB) to channel gain (dB) */
            let mut tmp_channel_gain_db = (channel_gain_db[i] >> 1)
                .wrapping_add(h_gain_dec.loudness_normalisation_gain_db >> 2);
            tmp_channel_gain_db = saturate_left_shift(tmp_channel_gain_db, 1, DFRACT_BITS);
            let mut channel_gain_e = 0;
            let channel_gain = db2lin(tmp_channel_gain_db, 8, &mut channel_gain_e);
            h_gain_dec.channel_gain[i] = scale_value_saturate(channel_gain, channel_gain_e - 8);
        }
    }
}

/*******************************************/
/* public functions                        */
/*******************************************/

/// `drcDec_GainDecoder_Open(phGainDec)`
pub fn drc_dec_gain_decoder_open() -> Box<DrcGainDecoder> {
    let mut h_gain_dec = Box::new(DrcGainDecoder::ZERO);

    h_gain_dec.multi_band_active_drc_index = -1;
    h_gain_dec.channel_gain_active_drc_index = -1;
    h_gain_dec.time_domain_supported = 1;

    h_gain_dec
}

/// `drcDec_GainDecoder_Init(hGainDec)`
pub fn drc_dec_gain_decoder_init(h_gain_dec: &mut DrcGainDecoder) -> DrcError {
    let err = init_gain_dec(h_gain_dec);
    if err != 0 {
        return err;
    }

    let frame_size = h_gain_dec.frame_size;
    init_drc_gain_buffers(frame_size, &mut h_gain_dec.drc_gain_buffers);

    err
}

/// `drcDec_GainDecoder_SetParam(hGainDec, paramType, paramValue)`
pub fn drc_dec_gain_decoder_set_param(
    h_gain_dec: &mut DrcGainDecoder,
    param_type: GainDecParam,
    param_value: i32,
) -> DrcError {
    match param_type {
        GAIN_DEC_FRAME_SIZE => {
            if param_value < 0 {
                return DE_PARAM_OUT_OF_RANGE;
            }
            h_gain_dec.frame_size = param_value;
            if h_gain_dec.sample_rate != 0 {
                h_gain_dec.ms_per_frame =
                    c_div(h_gain_dec.frame_size.wrapping_mul(1000), h_gain_dec.sample_rate);
            }
        }
        GAIN_DEC_SAMPLE_RATE => {
            if param_value < 0 {
                return DE_PARAM_OUT_OF_RANGE;
            }
            h_gain_dec.sample_rate = param_value;
            if h_gain_dec.sample_rate != 0 {
                h_gain_dec.ms_per_frame =
                    c_div(h_gain_dec.frame_size.wrapping_mul(1000), h_gain_dec.sample_rate);
            }
            h_gain_dec.delta_tmin_default = get_delta_tmin(h_gain_dec.sample_rate);
        }
        _ => return DE_PARAM_INVALID,
    }
    DE_OK
}

/// `drcDec_GainDecoder_SetCodecDependentParameters(hGainDec, delayMode, timeDomainSupported,
/// subbandDomainSupported)`
pub fn drc_dec_gain_decoder_set_codec_dependent_parameters(
    h_gain_dec: &mut DrcGainDecoder,
    delay_mode: DelayMode,
    time_domain_supported: i32,
    subband_domain_supported: SubbandDomainMode,
) -> DrcError {
    if (delay_mode != DM_REGULAR_DELAY) && (delay_mode != DM_LOW_DELAY) {
        return DE_NOT_OK;
    }

    h_gain_dec.delay_mode = delay_mode;
    h_gain_dec.time_domain_supported = time_domain_supported;
    h_gain_dec.subband_domain_supported = subband_domain_supported;

    DE_OK
}

/// `drcDec_GainDecoder_Config(hGainDec, hUniDrcConfig, numSelectedDrcSets, selectedDrcSetIds,
/// selectedDownmixIds)`
pub fn drc_dec_gain_decoder_config(
    h_gain_dec: &mut DrcGainDecoder,
    h_uni_drc_config: &UniDrcConfig,
    num_selected_drc_sets: u8,
    selected_drc_set_ids: &[i8],
    selected_downmix_ids: &[u8],
) -> DrcError {
    let p_coef = select_drc_coefficients(h_uni_drc_config, LOCATION_SELECTED);

    h_gain_dec.n_active_drcs[0] = 0;
    h_gain_dec.n_active_drcs[1] = 0;
    h_gain_dec.multi_band_active_drc_index = -1;
    h_gain_dec.channel_gain_active_drc_index = -1;

    if let Some(c) = p_coef {
        h_gain_dec.drc_coef = h_uni_drc_config.drc_coefficients_uni_drc[c].clone();
    /* keep deep copy of drcCoefficients */
    } else {
        h_gain_dec.drc_coef = DrcCoefficientsUniDrc::ZERO;
    }

    if h_gain_dec.drc_coef.drc_frame_size_present != 0
        && (h_gain_dec.drc_coef.drc_frame_size as i32 != h_gain_dec.frame_size)
    {
        return DE_NOT_OK;
    }

    for a in 0..num_selected_drc_sets as usize {
        let err = init_active_drc(
            h_gain_dec,
            h_uni_drc_config,
            selected_drc_set_ids[a] as i32,
            selected_downmix_ids[a] as i32,
        );
        if err != 0 {
            return err;
        }
    }

    add_virtual_to_active_drc(h_gain_dec);

    init_active_drc_offset(h_gain_dec)
}

/// `drcDec_GainDecoder_Preprocess(hGainDec, hUniDrcGain, loudnessNormalizationGainDb, boost,
/// compress)`
pub fn drc_dec_gain_decoder_preprocess(
    h_gain_dec: &mut DrcGainDecoder,
    h_uni_drc_gain: &mut UniDrcGain,
    loudness_normalization_gain_db: FixpDbl,
    boost: FixpSgl,
    compress: FixpSgl,
) -> DrcError {
    /* lnbPointer is the index on the most recent node buffer */
    h_gain_dec.drc_gain_buffers.lnb_pointer += 1;
    if h_gain_dec.drc_gain_buffers.lnb_pointer >= NUM_LNB_FRAMES as i32 {
        h_gain_dec.drc_gain_buffers.lnb_pointer = 0;
    }

    for l in 0..ACTIVE_DRC_LOCATIONS {
        for a in 0..h_gain_dec.n_active_drcs[l].max(0) as usize {
            /* prepare gain interpolation of sequences used by copying and modifying nodes in node
             * buffers */
            let err = prepare_drc_gain(
                h_gain_dec,
                h_uni_drc_gain,
                compress,
                boost,
                loudness_normalization_gain_db,
                a,
                l,
            );
            if err != 0 {
                return err;
            }
        }
    }

    let lnbp = h_gain_dec.drc_gain_buffers.lnb_pointer as usize;
    for l in 0..ACTIVE_DRC_LOCATIONS {
        for a in 0..MAX_ACTIVE_DRCS {
            for c in 0..28 {
                h_gain_dec.active_drc[l][a].lnb_index_for_channel[c][lnbp] = -1;
                /* "no DRC processing" */
            }
            h_gain_dec.active_drc[l][a].subband_gains_ready = 0;
        }
    }

    for s in 0..4 {
        h_uni_drc_gain.n_decoded_sequences[s] = 0;
    }

    if h_gain_dec.startup_ms > 0 {
        h_gain_dec.startup_ms -= h_gain_dec.ms_per_frame;
    } else {
        h_gain_dec.startup_ms = 0;
    }

    DE_OK
}

/// `drcDec_GainDecoder_Conceal(hGainDec, hUniDrcConfig, hUniDrcGain)`: create gain sequence out of
/// gain sequences of last frame for concealment and flushing
pub fn drc_dec_gain_decoder_conceal(
    h_gain_dec: &mut DrcGainDecoder,
    _h_uni_drc_config: &UniDrcConfig,
    h_uni_drc_gain: &mut UniDrcGain,
) -> DrcError {
    let p_coef = &h_gain_dec.drc_coef;
    let gain_sequence_count: usize = if p_coef.gain_sequence_count != 0 {
        f_min(p_coef.gain_sequence_count, 48u8) as usize
    } else {
        1
    };

    for seq in 0..gain_sequence_count {
        let mut last_gain_db: FixpSgl = 0;

        let last_node_index = h_uni_drc_gain.n_nodes[seq] as i32 - 1;
        if (last_node_index >= 0) && (last_node_index < 32) {
            last_gain_db = h_uni_drc_gain.gain_node[seq][last_node_index as usize].gain_db;
        }

        h_uni_drc_gain.n_nodes[seq] = 1;
        if last_gain_db > 0 {
            h_uni_drc_gain.gain_node[seq][0].gain_db =
                fx_dbl2fx_sgl(f_mult(fl2fxconst_sgl(0.9f32 as f64), last_gain_db));
        } else {
            h_uni_drc_gain.gain_node[seq][0].gain_db =
                fx_dbl2fx_sgl(f_mult(fl2fxconst_sgl(0.98f32 as f64), last_gain_db));
        }
        h_uni_drc_gain.gain_node[seq][0].time = (h_gain_dec.frame_size - 1) as i16;
    }

    /* update nDecodedSequences for sanity check */
    h_uni_drc_gain.n_decoded_sequences[0] = gain_sequence_count as u8;
    for s in 1..4 {
        h_uni_drc_gain.n_decoded_sequences[s] = 0;
    }
    DE_OK
}

/// `drcDec_GainDecoder_SetChannelGains(hGainDec, numChannels, frameSize, channelGainDb,
/// audioBufferChannelOffset, audioBuffer)`
///
/// `audio_buffer` starts at the first sample of the first channel; channel `c` starts at
/// `c * audio_buffer_channel_offset`. Note that (like in C) channels with a constant gain of 1.0
/// do not advance the channel pointer.
pub fn drc_dec_gain_decoder_set_channel_gains(
    h_gain_dec: &mut DrcGainDecoder,
    num_channels: i32,
    frame_size: i32,
    channel_gain_db: &[FixpDbl],
    audio_buffer_channel_offset: i32,
    audio_buffer: &mut [FixpDbl],
) {
    let num_channels = num_channels.max(0) as usize;
    if h_gain_dec.channel_gain_active_drc_index >= 0 {
        /* channel gains will be applied in drcDec_GainDecoder_ProcessTimeDomain or
         * drcDec_GainDecoder_ProcessSubbandDomain, respectively. */
        set_channel_gains_internal(h_gain_dec, num_channels as i32, channel_gain_db);

        if h_gain_dec.status == 0 {
            /* overwrite previous channel gains at startup */
            for c in 0..num_channels {
                h_gain_dec.channel_gain_prev[c] = h_gain_dec.channel_gain[c];
            }
            h_gain_dec.status = 1;
        }
    } else {
        /* smooth and apply channel gains */
        let mut prev_channel_gain = [0 as FixpDbl; 28];
        prev_channel_gain[..num_channels].copy_from_slice(&h_gain_dec.channel_gain[..num_channels]);

        set_channel_gains_internal(h_gain_dec, num_channels as i32, channel_gain_db);

        if h_gain_dec.status == 0 {
            /* overwrite previous channel gains at startup */
            prev_channel_gain[..num_channels]
                .copy_from_slice(&h_gain_dec.channel_gain[..num_channels]);
            h_gain_dec.status = 1;
        }

        let mut pos: isize = 0;
        for c in 0..num_channels {
            let mut n_min = f_min(
                f_min(
                    cnt_leading_zeros(prev_channel_gain[c]),
                    cnt_leading_zeros(h_gain_dec.channel_gain[c]),
                ) - 1,
                9,
            );
            let mut gain = prev_channel_gain[c].wrapping_shl(n_min as u32);
            let mut stepsize =
                h_gain_dec.channel_gain[c].wrapping_shl(n_min as u32).wrapping_sub(gain);

            const ONE: FixpDbl = fl2fxconst_dbl((1.0f32 / 256.0f32) as f64);
            if (prev_channel_gain[c] == ONE) && (h_gain_dec.channel_gain[c] == ONE) {
                /* gain of 1.0 throughout the section. Skip processing to keep bit-identical
                 * signal. */
                continue;
            }

            if stepsize != 0 {
                if frame_size == 1024 {
                    stepsize >>= 10;
                } else {
                    stepsize = c_div(stepsize, frame_size);
                }
            }
            n_min = 9 - n_min;
            let base = pos as usize;
            for i in 0..frame_size.max(0) as usize {
                let x = &mut audio_buffer[base + i];
                *x = f_mult_div2(*x, gain).wrapping_shl(n_min as u32);
                gain = gain.wrapping_add(stepsize);
            }
            pos += audio_buffer_channel_offset as isize;
        }
    }
}

/// `drcDec_GainDecoder_ProcessTimeDomain(hGainDec, delaySamples, drcLocation, channelOffset,
/// drcChannelOffset, numChannelsProcessed, timeDataChannelOffset, audioIOBuffer)`
pub fn drc_dec_gain_decoder_process_time_domain(
    h_gain_dec: &mut DrcGainDecoder,
    delay_samples: i32,
    drc_location: GainDecLocation,
    channel_offset: i32,
    drc_channel_offset: i32,
    num_channels_processed: i32,
    time_data_channel_offset: i32,
    audio_io_buffer: &mut [FixpDbl],
) -> DrcError {
    if h_gain_dec.time_domain_supported == 0 {
        return DE_NOT_OK;
    }

    if !(drc_location == GAIN_DEC_DRC1 || drc_location == GAIN_DEC_DRC2_DRC3) {
        return DE_NOT_OK;
    }

    for a in 0..h_gain_dec.n_active_drcs[drc_location as usize].max(0) as usize {
        /* Apply DRC */
        let err = process_drc_time(
            h_gain_dec,
            a,
            drc_location as usize,
            delay_samples,
            channel_offset,
            drc_channel_offset,
            num_channels_processed,
            time_data_channel_offset,
            audio_io_buffer,
        );
        if err != 0 {
            return err;
        }
    }

    DE_OK
}

/// `drcDec_GainDecoder_ProcessSubbandDomain(hGainDec, delaySamples, drcLocation, channelOffset,
/// drcChannelOffset, numChannelsProcessed, processSingleTimeslot, audioIOBufferReal[],
/// audioIOBufferImag[])`
pub fn drc_dec_gain_decoder_process_subband_domain<R: AsMut<[FixpDbl]>>(
    h_gain_dec: &mut DrcGainDecoder,
    delay_samples: i32,
    drc_location: GainDecLocation,
    channel_offset: i32,
    drc_channel_offset: i32,
    num_channels_processed: i32,
    process_single_timeslot: i32,
    audio_io_buffer_real: &mut [R],
    mut audio_io_buffer_imag: Option<&mut [R]>,
) -> DrcError {
    if h_gain_dec.subband_domain_supported == SDM_OFF {
        return DE_NOT_OK;
    }

    if !(drc_location == GAIN_DEC_DRC1 || drc_location == GAIN_DEC_DRC2_DRC3) {
        return DE_NOT_OK;
    }

    for a in 0..h_gain_dec.n_active_drcs[drc_location as usize].max(0) as usize {
        /* Apply DRC */
        let err = process_drc_subband(
            h_gain_dec,
            a,
            drc_location as usize,
            delay_samples,
            channel_offset,
            drc_channel_offset,
            num_channels_processed,
            process_single_timeslot,
            audio_io_buffer_real,
            audio_io_buffer_imag.as_deref_mut(),
        );
        if err != 0 {
            return err;
        }
    }

    DE_OK
}

/// `drcDec_GainDecoder_SetLoudnessNormalizationGainDb(hGainDec, loudnessNormalizationGainDb)`
pub fn drc_dec_gain_decoder_set_loudness_normalization_gain_db(
    h_gain_dec: &mut DrcGainDecoder,
    loudness_normalization_gain_db: FixpDbl,
) -> DrcError {
    h_gain_dec.loudness_normalisation_gain_db = loudness_normalization_gain_db;
    DE_OK
}

/// `drcDec_GainDecoder_GetFrameSize(hGainDec)`
pub fn drc_dec_gain_decoder_get_frame_size(h_gain_dec: Option<&DrcGainDecoder>) -> i32 {
    h_gain_dec.map_or(-1, |h| h.frame_size)
}

/// `drcDec_GainDecoder_GetDeltaTminDefault(hGainDec)`
pub fn drc_dec_gain_decoder_get_delta_tmin_default(h_gain_dec: Option<&DrcGainDecoder>) -> i32 {
    h_gain_dec.map_or(-1, |h| h.delta_tmin_default)
}

/// `drcDec_GainDecoder_GetStartupPhase(hGainDec)`
pub fn drc_dec_gain_decoder_get_startup_phase(h_gain_dec: Option<&DrcGainDecoder>) -> i32 {
    let Some(h) = h_gain_dec else {
        return -1;
    };

    if h.startup_ms > h.ms_per_frame {
        return 2; /* decoder is in 2.5 seconds startup phase */
    }
    if h.startup_ms > 0 {
        return 1; /* decoder is in last frame of startup phase */
    }
    0 /* startup phase is over */
}

/// `drcDec_GainDecoder_GetDrcCoefficients(hGainDec)`
pub fn drc_dec_gain_decoder_get_drc_coefficients(
    h_gain_dec: Option<&DrcGainDecoder>,
) -> Option<&DrcCoefficientsUniDrc> {
    h_gain_dec.map(|h| &h.drc_coef)
}
