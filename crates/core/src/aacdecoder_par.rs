// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not a port: parallel variants of parts of CAacDecoder_DecodeFrame() (the reference decoder is
// single-threaded).

//! Multi-core variants of parts of the frame decoding.
//!
//! These functions are used instead of the sequential code when the decoder has a thread pool
//! ([`AacDecoderInstance::thread_pool`]). They produce exactly the same results and leave exactly
//! the same state behind as the sequential code: only work that is independent is distributed,
//! state that is shared between the work items is handled as described at each function.

use crate::aac_rom::PcmDec;
use crate::aacdecoder::{caac_decoder_get_el_channels, AacDecoderInstance};
use crate::aacdecoder_lib::*;
use crate::block::cblock_frequency_to_time;
use crate::channelinfo::*;
use crate::conceal::*;
use crate::conceal_types::*;
use crate::ltp_post::ltp_post;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::common_fix::FixpDbl;
use mpeghdec_fdk::thread_pool::ThreadPool;
use mpeghdec_tpdec::*;

/// Result of [`AacDecoderInstance::frequency_to_time_parallel`].
pub(crate) enum FreqToTimeOutcome {
    /// Not handled (a configuration the parallel code does not cover); run the sequential code.
    Fallback,
    /// Done; the final `stream_index` of the channel loop.
    Done(usize),
    /// `CAacDecoder_DecodeFrame()` returns with this error (after the channels before the
    /// failing one were processed, like the sequential loop).
    Return(AacDecoderError),
}

/// One channel of the inverse transform loop.
enum ChannelJob {
    /// The sub stream is not available: the time signal is cleared.
    Clear { offset: usize },
    /// Concealment, inverse transform, LTP post filter and time domain fading.
    Decode { cu: usize, offset: usize, stream_index: usize, el: usize, el_ch: i32, frame_ok: u8 },
}

/// The memory of one channel for [`decode_channel`].
struct ChannelTask<'a> {
    st: &'a mut CAacDecoderStaticChannelInfo,
    ch: &'a mut CAacDecoderChannelInfo,
    spec: &'a mut [FixpDbl],
    time: &'a mut [PcmDec],
    /// IMDCT output buffer (instead of the shared `mdctOutTemp`).
    scratch: &'a mut [FixpDbl],
    stream_index: usize,
    el: usize,
    el_ch: i32,
    frame_ok: u8,
    /// Set if the inverse transform wrote `scratch`.
    imdct_done: bool,
    /// Set for an unsupported render mode (`AAC_DEC_UNKNOWN`).
    render_error: bool,
}

/// The read-only frame parameters of [`decode_channel`].
struct FrameParams<'a> {
    conceal_common_data: &'a CConcealParams,
    sampling_rate_info: &'a [SamplingRateInfo],
    stream_flags: &'a [u32],
    el_flags: &'a [u32],
    flags: u32,
    spf: i32,
    flush_clear: bool,
    aac_out_data_headroom: i32,
    aac_sample_rate: i32,
}

impl AacDecoderInstance {
    /// The per channel loop at the end of `CAacDecoder_DecodeFrame()` (concealment, inverse
    /// transform, LTP post filter, time domain fading) with the channels distributed over the
    /// thread pool.
    ///
    /// Why the result equals the sequential loop:
    ///
    /// - A channel `c` only accesses its own channel info, static channel info, spectrum
    ///   (`workBufferCore2[c * 1024..]`) and time signal (`pTimeData2[offset(c)..]`, `spf`
    ///   samples; the offsets of all channels must be different, otherwise the sequential code is
    ///   used), plus read-only frame data (concealment parameters, sampling rate info, flags).
    /// - `frame_ok` may be cleared by the ICS check of a channel and is passed on to the
    ///   following channels. The check only reads the state of its own channel that no earlier
    ///   channel modifies, so the value for every channel is computed first, in channel order.
    /// - `CBlock_FrequencyToTime()` uses the shared `mdctOutTemp` as IMDCT output buffer, which
    ///   the IMDCT writes completely (`spf` samples) before reading it (`imlt_block()` reads only
    ///   the spectrum and the overlap buffer). Every task uses its own buffer; afterwards the
    ///   output of the last channel in channel order is copied to `mdctOutTemp`, leaving the
    ///   buffer as the sequential loop does (later users write it before reading anyway).
    /// - `CConcealment_TDFading()` of channel `c` passes its TD noise seed on to channel `c + 1`.
    ///   The fading only touches the concealment info and time signal of its channel and the seed
    ///   of the next channel, which the concealment, IMDCT and LTP of the other channels do not
    ///   use; so the fading runs after the parallel part, sequentially in channel order.
    /// - Conditions that leave the loop early are evaluated in the same order: the robustness
    ///   check returns after the preceding channels were processed; the (frame constant) output
    ///   buffer size check makes the caller use the sequential code.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn frequency_to_time_parallel(
        &mut self,
        pool: &ThreadPool,
        tp: &TransportDec,
        flags: u32,
        element_count: usize,
        time_data_offset: usize,
        time_data_size: i32,
        time_data_channel_offset: i32,
        aac_channels: i32,
        reverse_ch_mapping: &[u8],
        error_status: &mut AacDecoderError,
    ) -> FreqToTimeOutcome {
        let spf = self.stream_info.aac_samples_per_frame;
        if time_data_size < time_data_channel_offset * self.aac_channels
            || spf <= 0
            || spf > 1024
            || time_data_channel_offset < spf
        {
            return FreqToTimeOutcome::Fallback;
        }

        /* 1. The channel loop without the processing: channel order, offsets, frame_ok. */
        let mut jobs: Vec<ChannelJob> = Vec::with_capacity(28);
        let mut frame_ok = self.frame_ok;
        let mut early_return = None;
        let mut stream_index = 0usize;
        let mut c: i32 = 0;
        let mut c_idx: i32 = 0;
        let mut el_channels: i32 = 0;
        'els: for el in 0..element_count {
            /* Determine current sub stream */
            if self.flags[stream_index] & (AC_MPEGH3DA | AC_USAC) != 0 {
                let mut num_elements = 0usize;
                for i in 0..=stream_index {
                    num_elements += self.num_usac_elements[i] as usize;
                }
                if el >= num_elements
                    && (stream_index < TPDEC_MAX_TRACKS - 1)
                    && self.p_usac_config[stream_index + 1].is_some()
                {
                    stream_index += 1;
                }
            } else {
                stream_index = 0;
            }

            let type_ = if self.flags[stream_index] & (AC_USAC | AC_MPEGH3DA | AC_BSAC) != 0 {
                self.elements[el]
            } else {
                self.channel_elements[el]
            };

            let n_element_channels = caac_decoder_get_el_channels(type_);
            el_channels += n_element_channels;
            if n_element_channels == 0 {
                continue;
            }

            let mut el_ch = 0;
            while c_idx < el_channels {
                'ch: {
                    if self.get_on_off_flag(c_idx) == 0 {
                        c -= 1;
                        break 'ch;
                    }
                    /* Robustness check */
                    if c >= aac_channels {
                        early_return = Some(AAC_DEC_UNKNOWN);
                        break 'els;
                    }
                    let cu = c as usize;
                    let offset = time_data_offset
                        + (reverse_ch_mapping[cu] as i32 * time_data_channel_offset) as usize;

                    if self.flags[stream_index] & AC_MPEGH3DA != 0
                        && tp.get_au_bits_total(stream_index as u32) <= 0
                        && (flags & (AACDEC_FLUSH | AACDEC_CONCEAL) == 0)
                    {
                        jobs.push(ChannelJob::Clear { offset });
                        break 'ch;
                    }

                    {
                        let p_ics_info = &self.p_aac_decoder_channel_info[cu].ics_info;
                        let h_concealment_info =
                            &self.p_aac_decoder_static_channel_info[cu].concealment_info;
                        let mute_release_active = (frame_ok != 0 && (flags & AACDEC_CONCEAL == 0))
                            && ((h_concealment_info.conceal_state >= ConcealState_Mute && {
                                /* concealment state, not part of the state digest */
                                mpeghdec_fdk::digest::hazard();
                                true
                            }) && (h_concealment_info.cnt_valid_frames
                                < self.conceal_common_data.num_mute_release_frames));
                        let ics_is_invalid = get_scale_factor_bands_transmitted(p_ics_info)
                            > get_scale_factor_bands_total(p_ics_info);
                        if ics_is_invalid && !mute_release_active {
                            frame_ok = 0;
                        }
                    }
                    jobs.push(ChannelJob::Decode { cu, offset, stream_index, el, el_ch, frame_ok });
                }
                c += 1;
                c_idx += 1;
                el_ch += 1;
            }
        }

        /* The time signals of the channels must not overlap. */
        let n_slots = (self.p_time_data2.len().saturating_sub(time_data_offset)
            / time_data_channel_offset as usize)
            .min(28);
        let mut slot_used = [false; 28];
        for job in &jobs {
            let offset = match *job {
                ChannelJob::Clear { offset } | ChannelJob::Decode { offset, .. } => offset,
            };
            let slot = (offset - time_data_offset) / time_data_channel_offset as usize;
            if slot >= n_slots || slot_used[slot] {
                return FreqToTimeOutcome::Fallback;
            }
            slot_used[slot] = true;
        }
        self.frame_ok = frame_ok;

        /* 2. Concealment, IMDCT and LTP of all channels in parallel. */
        let spfu = spf as usize;
        let n_decode = jobs.iter().filter(|j| matches!(j, ChannelJob::Decode { .. })).count();
        if self.imdct_scratch.len() < n_decode * 1024 {
            self.imdct_scratch.resize(n_decode * 1024, 0);
        }
        let mut last_imdct: Option<usize> = None; /* task index */
        let mut render_error = false;
        {
            let mut ch_infos: [Option<&mut CAacDecoderChannelInfo>; 28] =
                core::array::from_fn(|_| None);
            for (slot, ch) in ch_infos.iter_mut().zip(self.p_aac_decoder_channel_info.iter_mut()) {
                *slot = Some(&mut **ch);
            }
            let mut st_infos: [Option<&mut CAacDecoderStaticChannelInfo>; 28] =
                core::array::from_fn(|_| None);
            for (slot, st) in
                st_infos.iter_mut().zip(self.p_aac_decoder_static_channel_info.iter_mut())
            {
                *slot = Some(&mut **st);
            }
            let mut specs: [Option<&mut [FixpDbl]>; 28] = core::array::from_fn(|_| None);
            for (slot, spec) in specs.iter_mut().zip(self.work_buffer_core2.chunks_mut(1024)) {
                *slot = Some(spec);
            }
            let mut times: [Option<&mut [PcmDec]>; 28] = core::array::from_fn(|_| None);
            for (slot, t) in times
                .iter_mut()
                .zip(
                    self.p_time_data2[time_data_offset..]
                        .chunks_mut(time_data_channel_offset as usize),
                )
                .take(n_slots)
            {
                *slot = Some(t);
            }
            let mut scratch = self.imdct_scratch.chunks_mut(1024);

            let mut tasks: Vec<ChannelTask> = Vec::with_capacity(n_decode);
            for job in &jobs {
                match *job {
                    ChannelJob::Clear { offset } => {
                        let slot = (offset - time_data_offset) / time_data_channel_offset as usize;
                        times[slot].take().unwrap()[..spfu].fill(0);
                    }
                    ChannelJob::Decode { cu, offset, stream_index, el, el_ch, frame_ok } => {
                        let slot = (offset - time_data_offset) / time_data_channel_offset as usize;
                        let (Some(st), Some(ch), Some(spec), Some(time)) = (
                            st_infos[cu].take(),
                            ch_infos[cu].take(),
                            specs[cu].take(),
                            times[slot].take(),
                        ) else {
                            unreachable!("channel {cu} used twice");
                        };
                        tasks.push(ChannelTask {
                            st,
                            ch,
                            spec,
                            time: &mut time[..spfu],
                            scratch: scratch.next().unwrap(),
                            stream_index,
                            el,
                            el_ch,
                            frame_ok,
                            imdct_done: false,
                            render_error: false,
                        });
                    }
                }
            }

            let params = FrameParams {
                conceal_common_data: &self.conceal_common_data,
                sampling_rate_info: &self.sampling_rate_info,
                stream_flags: &self.flags,
                el_flags: &self.el_flags,
                flags,
                spf,
                flush_clear: self.flush_status != 0
                    && (self.flush_cnt > 0)
                    && (flags & AACDEC_CONCEAL == 0),
                aac_out_data_headroom: self.aac_out_data_headroom,
                aac_sample_rate: self.stream_info.aac_sample_rate,
            };
            pool.for_each_mut(&mut tasks, |_, task| decode_channel(task, &params));

            for (i, task) in tasks.iter().enumerate() {
                if task.imdct_done {
                    last_imdct = Some(i);
                }
                render_error |= task.render_error;
            }
        }
        if render_error {
            *error_status = AAC_DEC_UNKNOWN;
        }
        if let Some(i) = last_imdct {
            self.work_buffer_core1.mdct_out_temp()[..spfu]
                .copy_from_slice(&self.imdct_scratch[i * 1024..i * 1024 + spfu]);
        }

        /* 3. TimeDomainFading in channel order (passes the TD noise seed to the next channel). */
        for job in &jobs {
            if let ChannelJob::Decode { cu, offset, .. } = *job {
                cconcealment_td_fading(
                    spf,
                    &mut self.p_aac_decoder_static_channel_info[cu].concealment_info,
                    &self.conceal_common_data,
                    self.aac_out_data_headroom,
                    &mut self.p_time_data2[offset..offset + spfu],
                );
                let c = cu as i32;
                if c + 1 < 28 && c < aac_channels - 1 {
                    /* update next TDNoise Seed to avoid muting in case of Parametric Stereo */
                    let seed =
                        self.p_aac_decoder_static_channel_info[cu].concealment_info.td_noise_seed;
                    self.p_aac_decoder_static_channel_info[cu + 1].concealment_info.td_noise_seed =
                        seed;
                }
            }
        }

        match early_return {
            Some(err) => FreqToTimeOutcome::Return(err),
            None => FreqToTimeOutcome::Done(stream_index),
        }
    }
}

/// Concealment, inverse transform and LTP post filter of one channel (the body of the channel
/// loop of `CAacDecoder_DecodeFrame()` without the time domain fading).
fn decode_channel(t: &mut ChannelTask, p: &FrameParams) {
    let si = t.stream_index;
    let spf = p.spf;
    if p.flags & AACDEC_FLUSH != 0 {
        /* Clear pAacDecoderChannelInfo->pSpectralCoefficient because with AACDEC_FLUSH set it
         * contains undefined data. */
        t.spec[..spf as usize].fill(0);
    }

    /*
      Conceal defective spectral data
    */
    let conceal_apply_return_code = cconcealment_apply(
        &mut t.st.concealment_info,
        p.conceal_common_data,
        t.ch,
        &mut t.spec[..1024],
        &p.sampling_rate_info[si],
        spf,
        0,
        (t.frame_ok != 0 && (p.flags & AACDEC_CONCEAL == 0)) as i32,
        p.stream_flags[si],
    );
    let frame_ok_but_conceal = conceal_apply_return_code == -1;

    if p.flush_clear {
        t.time.fill(0);
    } else {
        match t.ch.render_mode {
            AACDEC_RENDER_IMDCT => {
                let frame_ok_arg = (t.frame_ok != 0
                    && (p.flags & AACDEC_CONCEAL == 0)
                    && !frame_ok_but_conceal) as i32;
                cblock_frequency_to_time(
                    t.st,
                    t.ch,
                    &mut t.spec[..1024],
                    t.time,
                    spf as i16,
                    frame_ok_arg,
                    t.scratch,
                    p.aac_out_data_headroom,
                    p.el_flags[t.el],
                    t.el_ch,
                );
                t.imdct_done = true;
                if p.stream_flags[si] & AC_MPEGH3DA != 0 {
                    let st = &mut *t.st;
                    ltp_post(
                        t.time,
                        spf,
                        p.aac_sample_rate,
                        &st.ltp_param,
                        &mut st.ltp_pitch_int_past,
                        &mut st.ltp_pitch_fr_past,
                        &mut st.ltp_gain_past,
                        &mut st.ltp_gain_idx_past,
                        &mut st.ltp_mem_in,
                        &mut st.ltp_mem_out,
                    );
                }
            }
            _ => {
                t.render_error = true;
            }
        }
    }
}
