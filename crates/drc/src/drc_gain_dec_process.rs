// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcGainDec_process.cpp, drcGainDec_process.h.

//! Gain interpolation and application of the DRC gains in the time and subband domains.

use mpeghdec_fdk::fixpoint_math::f_mult_norm_to;
use mpeghdec_fdk::{
    cnt_leading_zeros, count_leading_bits, f_max, f_min, f_mult, f_mult_div2, fixnormz_s,
    fl2fxconst_dbl, FixpDbl,
};

use crate::drc_dec_gain_decoder::*;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;
use crate::drc_gain_dec_multiband::MAX_NUM_SUBBANDS;

#[allow(dead_code)]
const E_TGAINSTEP: i32 = 12;

const GAIN_ONE_E7: FixpDbl = fl2fxconst_dbl((1.0f32 / 128.0f32) as f64);
const GAIN_ONE_E8: FixpDbl = fl2fxconst_dbl((1.0f32 / 256.0f32) as f64);

/// `pLnb->linearNodeGain[ix][pLnb->nNodes[ix] - 1]`, including the out-of-bounds read of C for
/// `nNodes == 0` (reads the preceding memory of the C structure).
#[inline]
fn last_node_gain(lnb: &LinearNodeBuffer, ix: usize) -> FixpDbl {
    let n = lnb.n_nodes[ix];
    if n >= 1 {
        lnb.linear_node_gain[ix][(n - 1).min(31) as usize]
    } else {
        /* reads at a physical position of the node ring buffer, which is part of the state
        digest only relative to lnbPointer */
        mpeghdec_fdk::digest::hazard();
        if ix > 0 {
            lnb.linear_node_gain[ix - 1][31]
        } else {
            lnb.n_nodes[NUM_LNB_FRAMES - 1]
        }
    }
}

/// `pLnb->linearNodeTime[ix][pLnb->nNodes[ix] - 1]`, see [`last_node_gain`].
#[inline]
fn last_node_time(lnb: &LinearNodeBuffer, ix: usize) -> i16 {
    let n = lnb.n_nodes[ix];
    if n >= 1 {
        lnb.linear_node_time[ix][(n - 1).min(31) as usize]
    } else {
        mpeghdec_fdk::digest::hazard();
        if ix > 0 {
            lnb.linear_node_time[ix - 1][31]
        } else {
            /* high half word of linearNodeGain[NUM_LNB_FRAMES - 1][31] (little endian) */
            (lnb.linear_node_gain[NUM_LNB_FRAMES - 1][31] >> 16) as i16
        }
    }
}

fn prepare_lnb_index(
    p_active_drc: &mut ActiveDrc,
    channel_offset: i32,
    drc_channel_offset: i32,
    num_channels_processed: i32,
    lnb_pointer: usize,
) -> DrcError {
    /* channelOffset: start index of physical channels
    numChannelsProcessed: number of processed channels, physical channels and DRC channels
    channelOffset + drcChannelOffset: start index of DRC channels,
       i.e. the channel order referenced in pInst.sequenceIndex */

    /* sanity checks */
    if (channel_offset + num_channels_processed) > 28 {
        return DE_NOT_OK;
    }

    if (channel_offset + drc_channel_offset + num_channels_processed) > 2 * 28 {
        return DE_NOT_OK;
    }

    if (channel_offset + drc_channel_offset) < 0 {
        return DE_NOT_OK;
    }

    /* prepare lnbIndexForChannel, a map of indices from each channel to its corresponding
     * linearNodeBuffer instance */
    for c in channel_offset..channel_offset + num_channels_processed {
        if p_active_drc.drc_inst.drc_set_id > 0 {
            let mut drc_channel = c + drc_channel_offset;
            /* fallback for configuration with more physical channels than DRC channels: reuse DRC
            gain of first channel. This is necessary for HE-AAC mono with stereo output */
            if drc_channel >= p_active_drc.drc_inst.drc_channel_count as i32 {
                drc_channel = 0;
            }
            let g = p_active_drc.channel_group_for_channel[drc_channel as usize];
            if (g >= 0) && p_active_drc.channel_group_is_parametric_drc[g as usize] == 0 {
                p_active_drc.lnb_index_for_channel[c as usize][lnb_pointer] = p_active_drc
                    .active_drc_offset
                    + p_active_drc.gain_element_for_group[g as usize] as i32;
            }
        }
    }

    DE_OK
}

fn interpolate_drc_gain(
    _gain_interpolation_type: GainInterpolationType,
    time_prev: i16,       /* time0 */
    t_gain_step: i16,     /* time1 - time0 */
    start: i16,           /* start sample of processing */
    stop: i16,            /* stop sample of processing */
    stepsize: i16,        /* processing hop size */
    gain_left: FixpDbl,   /* gain at time0, e = 7 */
    gain_right: FixpDbl,  /* gain at time1, e = 7 */
    slope_left: FixpDbl,  /* slope at time0, e = 7 */
    slope_right: FixpDbl, /* slope at time1, e = 7 */
    buffer: &mut [FixpDbl],
) -> DrcError {
    if t_gain_step < 0 {
        return DE_NOT_OK;
    }
    if t_gain_step == 0 {
        return DE_OK;
    }

    if (gain_left == GAIN_ONE_E7)
        && (gain_right == GAIN_ONE_E7)
        && (slope_left == 0)
        && (slope_right == 0)
    {
        /* gain of 1.0 throughout the section. Skip processing to keep bit-identical signal. */
        return DE_OK;
    }

    let (start, time_prev, stop, stepsize) =
        (start as i32, time_prev as i32, stop as i32, stepsize as i32);

    /* get start index offset and buffer index for downsampled interpolation */
    /* start_modulo = (start+timePrev)%stepsize; */
    /* stepsize is a power of 2 */
    let start_modulo = (start + time_prev) & (stepsize - 1);
    let start_offset = if start_modulo != 0 { stepsize - start_modulo } else { 0 };
    /* n_buf = (start + timePrev + start_offset)/stepsize; */
    let n_buf = (start + time_prev + start_offset) >> (15 - fixnormz_s(stepsize as i16));

    {
        /* gainInterpolationType == GIT_LINEAR */
        /* runs: Number of gain applications on buffer.
        runs = ceil((stop - start - start_offset)/stepsize). This works for stepsize = 2^N only. */
        let runs: i32 =
            (stop - start - start_offset + stepsize - 1) >> (30 - count_leading_bits(stepsize));
        if runs <= 0 {
            return DE_OK;
        }
        /* n_min: common headroom of gainRight and gainLeft, limited by 8 */
        let mut n_min: i32 =
            f_min(f_min(cnt_leading_zeros(gain_right), cnt_leading_zeros(gain_left)) - 1, 8);
        /* a: increment per sample for linear interpolation between gainLeft and gainRight */
        let mut a: i32 = gain_right
            .wrapping_shl(n_min as u32)
            .wrapping_sub(gain_left.wrapping_shl(n_min as u32))
            .wrapping_div(t_gain_step as i32);
        let mut a_step: i32 = 0;
        if runs > 1 {
            /* a_step: increment per step for linear interpolation */
            a_step = a.wrapping_mul(stepsize);
        }
        /* n: starting position of first gain application with respect to time0 */
        let n = start + start_offset;
        /* a: linear interpolation gain of first gain application */
        a = a.wrapping_mul(n).wrapping_add(gain_left.wrapping_shl(n_min as u32));
        let buffer = &mut buffer[n_buf as usize..];
        /* n_min: scaling value to compensate for e = 7 of the gain, and for fMultDiv2 */
        n_min = 8 - n_min;
        let runs = runs as usize;
        for x in buffer[..runs - 1].iter_mut() {
            *x = f_mult_div2(*x, a).wrapping_shl(n_min as u32);
            a = a.wrapping_add(a_step);
        }
        let x = &mut buffer[runs - 1];
        *x = f_mult_div2(*x, a).wrapping_shl(n_min as u32);
    }
    DE_OK
}

fn process_node_segments(
    frame_size: i32,
    gain_interpolation_type: GainInterpolationType,
    n_nodes: i32,
    p_node_lin_gain: &[FixpDbl; 32],
    p_node_lin_time: &[i16; 32],
    offset: i32,
    stepsize: i16,
    node_lin_gain_previous: FixpDbl, /* the last node gain of the previous frame */
    node_lin_time_previous: i16,     /* the last node time of the previous frame */
    channel_gain: FixpDbl,           /* e = 8 */
    p_channel_gain_previous: &mut FixpDbl, /* e = 8 */
    buffer: &mut [FixpDbl],
) -> DrcError {
    /* The last sample of the last interpolation segment. */
    let mut time_prev: i16;
    /* e = 7 */
    let mut gain_lin_prev: FixpDbl;
    let slope_lin: FixpDbl = 0; /* e = 7 */
    let slope_lin_prev: FixpDbl = 0; /* e = 7 */

    time_prev = (node_lin_time_previous as i32).wrapping_add(offset) as i16;
    gain_lin_prev = node_lin_gain_previous;
    for n in 0..n_nodes.clamp(0, 32) as usize {
        /* The last sample of the current interpolation segment. */
        let time: i16 = (p_node_lin_time[n] as i32).wrapping_add(offset) as i16;
        let duration: i16 = (time as i32 - time_prev as i32) as i16;
        let gain_lin: FixpDbl = p_node_lin_gain[n];

        /* skip over invalid sections with negative duration */
        if duration < 0 {
            continue;
        }

        if (time_prev as i32 >= (frame_size - 1)) || (time < 0) {
            /* This segment (between previous and current node) lies outside of this audio frame */
            time_prev = time;
            gain_lin_prev = gain_lin;
            continue;
        }

        /* start and stop are the boundaries of the region of this segment that lie within this
        audio frame. Their values are relative to the beginning of this segment. stop is the first
        sample that isn't processed any more. */
        let start: i16 = f_max(-(time_prev as i32), 1) as i16;
        let stop: i16 = (f_min(time, (frame_size - 1) as i16) as i32 - time_prev as i32 + 1) as i16;

        let gain_lin_chan_prev = f_mult_norm_to(gain_lin_prev, 7, *p_channel_gain_previous, 8, 7);

        let gain_lin_chan = if start == 1 {
            /* This is a new segment. Apply new channelGain. */
            let g = f_mult_norm_to(gain_lin, 7, channel_gain, 8, 7);
            *p_channel_gain_previous = channel_gain;
            g
        } else {
            /* This is a segment already processed last frame. Needs to be completed with old
            channelGain. */
            f_mult_norm_to(gain_lin, 7, *p_channel_gain_previous, 8, 7)
        };

        let err = interpolate_drc_gain(
            gain_interpolation_type,
            time_prev,
            duration,
            start,
            stop,
            stepsize,
            gain_lin_chan_prev,
            gain_lin_chan,
            slope_lin_prev,
            slope_lin,
            buffer,
        );
        if err != 0 {
            return err;
        }

        time_prev = time;
        gain_lin_prev = gain_lin;
    }
    DE_OK
}

/// `processDrcTime(hGainDec, activeDrcIndex, activeDrcLocation, delaySamples, channelOffset,
/// drcChannelOffset, numChannelsProcessed, timeDataChannelOffset, deinterleavedAudio)`: process
/// DRC on time-domain signal
pub fn process_drc_time(
    h_gain_dec: &mut DrcGainDecoder,
    active_drc_index: usize,
    active_drc_location: usize,
    delay_samples: i32,
    channel_offset: i32,
    drc_channel_offset: i32,
    num_channels_processed: i32,
    time_data_channel_offset: i32,
    deinterleaved_audio: &mut [FixpDbl],
) -> DrcError {
    let lnb_pointer = h_gain_dec.drc_gain_buffers.lnb_pointer as usize;
    let mut offset = 0;

    if h_gain_dec.delay_mode == DM_REGULAR_DELAY {
        offset = h_gain_dec.frame_size;
    }

    if (delay_samples + offset) > (NUM_LNB_FRAMES as i32 - 2) * h_gain_dec.frame_size {
        /* if delaySamples is too big, NUM_LNB_FRAMES should be increased */
        return DE_NOT_OK;
    }

    let err = prepare_lnb_index(
        &mut h_gain_dec.active_drc[active_drc_location][active_drc_index],
        channel_offset,
        drc_channel_offset,
        num_channels_processed,
        lnb_pointer,
    );
    if err != 0 {
        return err;
    }

    let frame_size = h_gain_dec.frame_size;
    let channel_gain_active_drc_index = h_gain_dec.channel_gain_active_drc_index;
    let p_active_drc = &h_gain_dec.active_drc[active_drc_location][active_drc_index];
    let p_drc_gain_buffers = &h_gain_dec.drc_gain_buffers;

    /* apply channelOffset */
    let mut audio_pos: isize = channel_offset as isize * time_data_channel_offset as isize;

    /* signal processing loop */
    for c in channel_offset..channel_offset + num_channels_processed {
        let c = c as usize;
        let mut gain_one: FixpDbl = GAIN_ONE_E8;
        let channel_gain: FixpDbl;
        let p_channel_gain_prev: &mut FixpDbl;
        if active_drc_location == 0 && active_drc_index as i32 == channel_gain_active_drc_index {
            channel_gain = h_gain_dec.channel_gain[c];
            p_channel_gain_prev = &mut h_gain_dec.channel_gain_prev[c];
        } else {
            channel_gain = gain_one;
            p_channel_gain_prev = &mut gain_one;
        }

        let b: i32 = 0;
        {
            /* get pointer to oldest linearNodes */
            let mut lnb_ix: i32 = lnb_pointer as i32 + 1 - NUM_LNB_FRAMES as i32;
            while lnb_ix < 0 {
                lnb_ix += NUM_LNB_FRAMES as i32;
            }

            /* Loop over all node buffers in linearNodeBuffer.
            All nodes which are not relevant for the current frame are sorted out inside
            _processNodeSegments. */
            for i in 0..NUM_LNB_FRAMES as i32 - 1 {
                /* Prepare previous node */
                let idx_prev = p_active_drc.lnb_index_for_channel[c][lnb_ix as usize];
                let p_lnb_previous = if idx_prev >= 0 {
                    p_drc_gain_buffers.lnb(idx_prev + b)
                } else {
                    p_drc_gain_buffers.dummy_lnb()
                };
                p_lnb_previous.check_read(idx_prev + b, lnb_ix as usize);
                let node_gain_previous = last_node_gain(p_lnb_previous, lnb_ix as usize);
                let mut node_time_previous = last_node_time(p_lnb_previous, lnb_ix as usize);
                node_time_previous = (node_time_previous as i32).wrapping_sub(frame_size) as i16;

                /* Prepare current linearNodeBuffer instance */
                lnb_ix += 1;
                if lnb_ix >= NUM_LNB_FRAMES as i32 {
                    lnb_ix = 0;
                }

                /* if lnbIndexForChannel changes over time, use the old indices for smooth
                 * transitions */
                let idx = p_active_drc.lnb_index_for_channel[c][lnb_ix as usize];
                let p_lnb = if idx >= 0 {
                    p_drc_gain_buffers.lnb(idx + b)
                } else {
                    /* lnbIndexForChannel = -1 means "no DRC processing", due to
                    drcInstructionsIndex < 0, drcSetId < 0 or channel group < 0 */
                    p_drc_gain_buffers.dummy_lnb()
                };
                p_lnb.check_read(idx + b, lnb_ix as usize);

                /* number of frames of offset with respect to lnbPointer */
                let lnb_pointer_diff = i - (NUM_LNB_FRAMES as i32 - 2);

                let err = process_node_segments(
                    frame_size,
                    p_lnb.gain_interpolation_type,
                    p_lnb.n_nodes[lnb_ix as usize],
                    &p_lnb.linear_node_gain[lnb_ix as usize],
                    &p_lnb.linear_node_time[lnb_ix as usize],
                    lnb_pointer_diff * frame_size + delay_samples + offset,
                    1,
                    node_gain_previous,
                    node_time_previous,
                    channel_gain,
                    p_channel_gain_prev,
                    &mut deinterleaved_audio[audio_pos as usize..],
                );
                if err != 0 {
                    return err;
                }
            }
            audio_pos += time_data_channel_offset as isize; /* proceed to next channel */
        }
    }
    DE_OK
}

/// `processDrcSubband(hGainDec, activeDrcIndex, activeDrcLocation, delaySamples, channelOffset,
/// drcChannelOffset, numChannelsProcessed, processSingleTimeslot, deinterleavedAudioReal[],
/// deinterleavedAudioImag[])`: process DRC on subband-domain signal
///
/// If `deinterleaved_audio_imag` is `None`, the signal in `deinterleaved_audio_real` is
/// interleaved (real, imag, real, imag, ...).
pub fn process_drc_subband<R: AsMut<[FixpDbl]>>(
    h_gain_dec: &mut DrcGainDecoder,
    active_drc_index: usize,
    active_drc_location: usize,
    delay_samples: i32,
    channel_offset: i32,
    drc_channel_offset: i32,
    num_channels_processed: i32,
    process_single_timeslot: i32,
    deinterleaved_audio_real: &mut [R],
    mut deinterleaved_audio_imag: Option<&mut [R]>,
) -> DrcError {
    let lnb_pointer = h_gain_dec.drc_gain_buffers.lnb_pointer as usize;
    let subband_domain_mode = h_gain_dec.subband_domain_supported;
    let mut signal_index = 0usize;
    let n_decoder_subbands: usize;
    let l: i16; /* L: downsampling factor */
    let mut offset = 0;

    if h_gain_dec.delay_mode == DM_REGULAR_DELAY {
        offset = h_gain_dec.frame_size;
    }

    if (delay_samples + offset) > (NUM_LNB_FRAMES as i32 - 2) * h_gain_dec.frame_size {
        /* if delaySamples is too big, NUM_LNB_FRAMES should be increased */
        return DE_NOT_OK;
    }

    match subband_domain_mode {
        SDM_QMF64 | SDM_QMF71 => {
            /* QMF domain processing is not supported. */
            return DE_NOT_OK;
        }
        SDM_STFT256 => {
            n_decoder_subbands = SUBBAND_NUM_BANDS_STFT256;
            l = 256;
            /* analysisDelay = SUBBAND_ANALYSIS_DELAY_STFT256; */
        }
        _ => return DE_NOT_OK,
    }

    /* frameSizeSb = hGainDec->frameSize/L; */
    /* L is a power of 2 */
    let frame_size_sb: i32 = h_gain_dec.frame_size >> (15 - fixnormz_s(l)); /* timeslots per frame */

    let (m_start, m_stop) =
        if (process_single_timeslot < 0) || (process_single_timeslot >= frame_size_sb) {
            (0, frame_size_sb)
        } else {
            (process_single_timeslot, process_single_timeslot + 1)
        };

    let err = prepare_lnb_index(
        &mut h_gain_dec.active_drc[active_drc_location][active_drc_index],
        channel_offset,
        drc_channel_offset,
        num_channels_processed,
        lnb_pointer,
    );
    if err != 0 {
        return err;
    }

    let frame_size = h_gain_dec.frame_size;
    let active_drc_offset =
        h_gain_dec.active_drc[active_drc_location][active_drc_index].active_drc_offset;

    if h_gain_dec.active_drc[active_drc_location][active_drc_index].subband_gains_ready == 0 {
        /* only for the first time per frame that processDrcSubband is called */
        let mut gain_one: FixpDbl = GAIN_ONE_E8;
        let p_active_drc = &h_gain_dec.active_drc[active_drc_location][active_drc_index];
        let p_inst = &p_active_drc.drc_inst;
        let p_linear_node_buffer = &h_gain_dec.drc_gain_buffers;
        let subband_gains = &mut h_gain_dec.subband_gains;
        /* write subbandGains */
        for g in 0..p_inst.n_drc_channel_groups as usize {
            for b in 0..p_active_drc.band_count_for_channel_group[g] as i32 {
                let lnb_idx = active_drc_offset + p_active_drc.gain_element_for_group[g] as i32 + b;
                let p_lnb = p_linear_node_buffer.lnb(lnb_idx);

                let base = (active_drc_offset as usize + g) * SUBBAND_GAINS_ROW
                    + (b * frame_size_sb) as usize;
                if base >= subband_gains.len() {
                    /* out of bounds in C (undefined behaviour) */
                    mpeghdec_fdk::deviation::hit();
                    continue;
                }
                for m in 0..frame_size_sb.max(0) as usize {
                    if let Some(v) = subband_gains.get_mut(base + m) {
                        *v = GAIN_ONE_E7;
                    } else {
                        mpeghdec_fdk::deviation::hit();
                    }
                }

                let mut lnb_ix: i32 = lnb_pointer as i32 - (NUM_LNB_FRAMES as i32 - 1);
                while lnb_ix < 0 {
                    lnb_ix += NUM_LNB_FRAMES as i32;
                }

                /* Loop over all node buffers in linearNodeBuffer.
                All nodes which are not relevant for the current frame are sorted out inside
                _processNodeSegments. */
                for i in 0..NUM_LNB_FRAMES as i32 - 1 {
                    /* Prepare previous node */
                    p_lnb.check_read(lnb_idx, lnb_ix as usize);
                    let node_gain_previous = last_node_gain(p_lnb, lnb_ix as usize);
                    let mut node_time_previous = last_node_time(p_lnb, lnb_ix as usize);
                    node_time_previous =
                        (node_time_previous as i32).wrapping_sub(frame_size) as i16;

                    lnb_ix += 1;
                    if lnb_ix >= NUM_LNB_FRAMES as i32 {
                        lnb_ix = 0;
                    }

                    /* number of frames of offset with respect to lnbPointer */
                    let lnb_pointer_diff = i - (NUM_LNB_FRAMES as i32 - 2);

                    p_lnb.check_read(lnb_idx, lnb_ix as usize);
                    let gain_one_value = gain_one;
                    let err = process_node_segments(
                        frame_size,
                        p_lnb.gain_interpolation_type,
                        p_lnb.n_nodes[lnb_ix as usize],
                        &p_lnb.linear_node_gain[lnb_ix as usize],
                        &p_lnb.linear_node_time[lnb_ix as usize],
                        lnb_pointer_diff * frame_size + delay_samples + offset - (l as i32 - 1) / 2,
                        l,
                        node_gain_previous,
                        node_time_previous,
                        gain_one_value,
                        &mut gain_one,
                        &mut subband_gains[base..],
                    );
                    if err != 0 {
                        return err;
                    }
                }
            }
        }
        h_gain_dec.active_drc[active_drc_location][active_drc_index].subband_gains_ready = 1;
    }

    let channel_gain_active_drc_index = h_gain_dec.channel_gain_active_drc_index;
    let p_active_drc = &h_gain_dec.active_drc[active_drc_location][active_drc_index];
    let p_inst = &p_active_drc.drc_inst;
    let p_overlap = &h_gain_dec.overlap;
    let subband_gains = &h_gain_dec.subband_gains;

    for c in channel_offset..channel_offset + num_channels_processed {
        let mut band_count = 1;
        let g: i32 = if p_inst.drc_set_id > 0 {
            p_active_drc.channel_group_for_channel[(c + drc_channel_offset) as usize] as i32
        } else {
            -1
        };

        let audio_real: &mut [FixpDbl] = deinterleaved_audio_real[signal_index].as_mut();
        let mut audio_imag: Option<&mut [FixpDbl]> =
            deinterleaved_audio_imag.as_deref_mut().map(|im| im[signal_index].as_mut());
        let mut ri = 0usize;
        let mut ii = 0usize;

        let this_base: usize;
        if (g >= 0) && p_active_drc.channel_group_is_parametric_drc[g as usize] == 0 {
            this_base = (active_drc_offset + g) as usize * SUBBAND_GAINS_ROW;
            band_count = p_active_drc.band_count_for_channel_group[g as usize] as i32;
        } else {
            this_base = DUMMY_SUBBAND_GAINS_ROW * SUBBAND_GAINS_ROW;
        }
        let this_subband_gains_buffer = |idx: i32| -> FixpDbl {
            /* reads beyond the array are undefined behaviour in C */
            subband_gains.get(this_base + idx as usize).copied().unwrap_or_else(|| {
                mpeghdec_fdk::deviation::hit();
                0
            })
        };

        let apply_channel_gain =
            active_drc_location == 0 && active_drc_index as i32 == channel_gain_active_drc_index;

        for m in m_start..m_stop {
            let mut n_min: i32 = 8;
            if band_count > 1 {
                /* multiband DRC */
                let mut gain_sbs = [0 as FixpDbl; MAX_NUM_SUBBANDS];
                let g = g as usize;
                /* normalize gainLr for keeping signal precision */
                for b in 0..band_count {
                    let mut gain_lr = this_subband_gains_buffer(b * frame_size_sb + m);
                    if apply_channel_gain {
                        gain_lr =
                            f_mult_norm_to(gain_lr, 7, h_gain_dec.channel_gain[c as usize], 8, 7);
                    }
                    n_min = f_min(cnt_leading_zeros(gain_lr) - 1, n_min);
                }
                let mut b = band_count;
                while b > 0 {
                    /* count reverse as the last band contains the most nonzero overlapWeights */
                    b -= 1;
                    let mut gain_lr = this_subband_gains_buffer(b * frame_size_sb + m);
                    if apply_channel_gain {
                        gain_lr =
                            f_mult_norm_to(gain_lr, 7, h_gain_dec.channel_gain[c as usize], 8, 7);
                    }
                    gain_lr = gain_lr.wrapping_shl(n_min as u32);
                    let ov = &p_overlap[g][b as usize];
                    if b == band_count - 1 {
                        for s in 0..n_decoder_subbands {
                            gain_sbs[s] = f_mult(ov.overlap_weight[s], gain_lr);
                        }
                        for v in gain_sbs[n_decoder_subbands..].iter_mut() {
                            *v = 0;
                        }
                    } else {
                        let stop = (ov.stop_index as usize).min(MAX_NUM_SUBBANDS);
                        for s in ov.start_index as usize..stop {
                            gain_sbs[s] =
                                gain_sbs[s].wrapping_add(f_mult(ov.overlap_weight[s], gain_lr));
                        }
                    }
                }
                n_min = 8 - n_min;
                let sh = n_min as u32;
                if let Some(audio_imag) = audio_imag.as_deref_mut() {
                    audio_real[ri] = f_mult_div2(audio_real[ri], gain_sbs[0]).wrapping_shl(sh);
                    ri += 1;
                    if subband_domain_mode == SDM_STFT256 {
                        /* For STFT filterbank, the real value of the nyquist band is stored at the
                        second array entry */
                        audio_imag[ii] =
                            f_mult_div2(audio_imag[ii], gain_sbs[n_decoder_subbands - 1])
                                .wrapping_shl(sh);
                    } else {
                        audio_imag[ii] = f_mult_div2(audio_imag[ii], gain_sbs[0]).wrapping_shl(sh);
                    }
                    ii += 1;
                    for s in 1..n_decoder_subbands {
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sbs[s]).wrapping_shl(sh);
                        ri += 1;
                        audio_imag[ii] = f_mult_div2(audio_imag[ii], gain_sbs[s]).wrapping_shl(sh);
                        ii += 1;
                    }
                } else {
                    /* real and imaginary parts are interleaved */
                    audio_real[ri] = f_mult_div2(audio_real[ri], gain_sbs[0]).wrapping_shl(sh);
                    ri += 1;
                    if subband_domain_mode == SDM_STFT256 {
                        /* For STFT filterbank, the real value of the nyquist band is stored at the
                        second array entry */
                        audio_real[ri] =
                            f_mult_div2(audio_real[ri], gain_sbs[n_decoder_subbands - 1])
                                .wrapping_shl(sh);
                    } else {
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sbs[0]).wrapping_shl(sh);
                    }
                    ri += 1;
                    for s in 1..n_decoder_subbands {
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sbs[s]).wrapping_shl(sh);
                        ri += 1;
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sbs[s]).wrapping_shl(sh);
                        ri += 1;
                    }
                }
            } else {
                /* single-band DRC */
                let mut gain_sb = this_subband_gains_buffer(m);
                if apply_channel_gain {
                    gain_sb = f_mult_norm_to(gain_sb, 7, h_gain_dec.channel_gain[c as usize], 8, 7);
                }
                /* normalize gainSb for keeping signal precision */
                n_min = f_min(cnt_leading_zeros(gain_sb) - 1, n_min);
                gain_sb = gain_sb.wrapping_shl(n_min as u32);
                n_min = 8 - n_min;
                let sh = n_min as u32;
                if let Some(audio_imag) = audio_imag.as_deref_mut() {
                    for _ in 0..n_decoder_subbands {
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sb).wrapping_shl(sh);
                        ri += 1;
                        audio_imag[ii] = f_mult_div2(audio_imag[ii], gain_sb).wrapping_shl(sh);
                        ii += 1;
                    }
                } else {
                    /* real and imaginary parts are interleaved */
                    for _ in 0..n_decoder_subbands {
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sb).wrapping_shl(sh);
                        ri += 1;
                        audio_real[ri] = f_mult_div2(audio_real[ri], gain_sb).wrapping_shl(sh);
                        ri += 1;
                    }
                }
            }
        }
        signal_index += 1;
    }
    DE_OK
}
