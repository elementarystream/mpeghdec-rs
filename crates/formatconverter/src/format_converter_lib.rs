// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_formatConverterLib.h and
// libFormatConverter/src/FDK_formatConverterLib.cpp.

//! Public interface of the format converter library.

use mpeghdec_fdk::digest::StateDigest;

use crate::format_converter::*;
use crate::format_converter_constants::*;
use crate::format_converter_constants_stft::*;
use crate::format_converter_data::*;
use crate::format_converter_intern::*;
use crate::format_converter_process::*;
use mpeghdec_drc::{DrcDecoder, DRC_DEC_DRC1};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::{
    cicp2geometry_get_cicp_index_from_geometry, cicp2geometry_get_number_of_lfes,
    Cicp2GeometryChannelGeometry,
};
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::scale::get_scalefactor;
use mpeghdec_fdk::stftfilterbank::*;
use mpeghdec_fdk::thread_pool::ThreadPool;
use mpeghdec_gvbap::*;

pub const AFC_MAX_IN_CHANNELS: u32 = 32;
pub const AFC_MAX_OUT_CHANNELS: u32 = 32;
pub const MAX_NUM_DMX_RULES: usize = 200;

/// `IIS_FORMATCONVERTER_MODE`
pub type IisFormatconverterMode = i32;
pub const IIS_FORMATCONVERTER_MODE_INVALID: IisFormatconverterMode = 0;
pub const IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN: IisFormatconverterMode = 1;
pub const IIS_FORMATCONVERTER_MODE_PASSIVE_FREQ_DOMAIN: IisFormatconverterMode = 2;
pub const IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_PHASE_ALIGN: IisFormatconverterMode = 3;
pub const IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_PHASE_ALIGN: IisFormatconverterMode = 4;
pub const IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT: IisFormatconverterMode = 5;
pub const IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT: IisFormatconverterMode = 6;

/// `IIS_FORMATCONVERTER`: format converter instance (`IIS_FORMATCONVERTER_HANDLE` is
/// `Box<IisFormatConverter>`).
#[derive(StateDigest, Clone, Debug)]
pub struct IisFormatConverter {
    pub num_local_speaker: u32,
    pub pas: i32,
    pub aes: i32,
    /// `member`: the internal instance.
    pub member: Box<IisFormatConverterInternal>,
    pub num_signals_total: u32,
    pub fc_mode: IisFormatconverterMode,
}

/**********************************************/
/*                                            */
/*           Initialization  Methods          */
/*                                            */
/**********************************************/

/// `IIS_FormatConverter_Create(self, mode, outGeo, numOutChannels, sampleRate, frameSize)`:
/// create a format converter for the output setup `out_geo` (`num_out_channels` entries).
/// Returns 0 on success, 1 on error (in which case `*self_` is left untouched).
///
/// `num_out_channels` must not exceed `FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS` (the caller
/// checks this; C would write past the geometry array, the port returns 1).
pub fn iis_format_converter_create(
    self_: &mut Option<Box<IisFormatConverter>>,
    mode: IisFormatconverterMode,
    out_geo: &[Cicp2GeometryChannelGeometry],
    num_out_channels: u32,
    sample_rate: u32,
    frame_size: u32,
) -> i32 {
    let err = 0;

    /* Other frame sizes may need a special treatment.
    see processing in phase align. */
    match mode {
        IIS_FORMATCONVERTER_MODE_PASSIVE_FREQ_DOMAIN
        | IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_PHASE_ALIGN
        | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_PHASE_ALIGN
        | IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
            if frame_size != 1024 =>
        {
            return 1;
        }
        _ => {}
    }
    if num_out_channels as usize > FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS {
        mpeghdec_fdk::deviation::hit();
        return 1;
    }

    let mut p = Box::<IisFormatConverterInternal>::default();

    if mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        || mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
    {
        p.stft_num_erb_bands = STFT_ERB_BANDS as u32;
        p.stft_frame_size = STFT_FRAME_SIZE;
        p.stft_length = STFT_LENGTH;
        p.fc_num_freq_bands = p.stft_length / 2 + 1;
        p.fc_center_frequencies = Some(&F_BANDS_NRM_STFT_256_ERB_58);
    }
    p.gvh = GVH;
    p.gvl = GVL;
    p.gvh_e = [[0; 6]; 13];
    p.gvl_e = [[0; 6]; 13];

    p.frame_size = frame_size;

    p.out_channel_vbap_flag = 0;
    p.in_channel_vbap_flag = 0;
    p.fc_params = None;
    p.fc_state = None;

    p.fc_input_format = FDK_FORMAT_CONVERTER_INPUT_FORMAT_LISTOFCHANNELS;
    p.fc_output_format = FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_LISTOFCHANNELS;

    p.num_total_input_channels = 0;
    p.num_output_channels = num_out_channels;
    p.sampling_rate = sample_rate;

    p.stft_headroom_prescaling = 0;

    if mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        || mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
    {
        p.num_time_slots = frame_size / p.stft_frame_size;
    }

    p.immersive_downmix_flag = 0;

    p.cicp_layout_index = -1;
    p.amount_of_added_dmx_matrices_and_equalizers = 0;

    /* FDK_ASSERT(mode is valid): not checked in the reference build. Only the modes
     * PASSIVE_TIME_DOMAIN, ACTIVE_FREQ_DOMAIN_STFT and CUSTOM_FREQ_DOMAIN_STFT are functional; for
     * the other modes formatConverterInit_internal() reads uninitialized scratch memory in C (the
     * port creates an empty downmix matrix instead). */

    p.mode = mode;

    let n = num_out_channels as usize;
    p.out_channel_geo[..n].copy_from_slice(&out_geo[..n]);

    *self_ = Some(Box::new(IisFormatConverter {
        num_local_speaker: num_out_channels,
        pas: 0,
        aes: 0,
        member: p,
        /* Initialization: Total number of signals and format converter mode. */
        num_signals_total: 0,
        fc_mode: mode,
    }));

    err
}

/// `IIS_FormatConverter_Config_AddInputSetup(self, geo, numChannels, channelOffset, channelId)`:
/// add the geometric information of another channel signal group with `num_channels` channels.
/// Returns 0 for no error, -1 or 1 in case of an error. `channel_offset` and `channel_id` are
/// unused.
pub fn iis_format_converter_config_add_input_setup(
    self_: &mut IisFormatConverter,
    geo: &[Cicp2GeometryChannelGeometry],
    num_channels: u32,
    _channel_offset: u32,
    _channel_id: Option<&[i32]>,
) -> i32 {
    let mut num_lfes = 0usize;
    let p = &mut *self_.member;

    /* sanity check: check number of input channels */
    if (p.num_total_input_channels + num_channels) as usize
        > FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS
    {
        return -1;
    }

    /* sanity check: check number of LFE channels */
    for k in 0..p.num_total_input_channels as usize {
        if p.input_channel_geo[k].lfe == 1 {
            num_lfes += 1;
        }
    }
    for g in &geo[..num_channels as usize] {
        if g.lfe == 1 {
            num_lfes += 1;
        }
    }

    if num_lfes > FDK_FORMAT_CONVERTER_MAX_LFE {
        return 1;
    }

    let start = p.num_total_input_channels as usize;
    p.input_channel_geo[start..start + num_channels as usize]
        .copy_from_slice(&geo[..num_channels as usize]);

    p.num_total_input_channels += num_channels;

    // `numInputChannelGroups` cannot exceed the array size: every group has at least one channel
    // or the total channel check above keeps working (groups of zero channels would overflow in
    // C; they are ignored here).
    if (p.num_input_channel_groups as usize) < FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNEL_GROUPS {
        p.num_input_channels[p.num_input_channel_groups as usize] = num_channels;
        p.num_input_channel_groups += 1;
    }

    0
}

/// `IIS_FormatConverter_Config_SetAES(self, aes)`
pub fn iis_format_converter_config_set_aes(self_: &mut IisFormatConverter, aes: u32) {
    debug_assert!(aes <= 7);
    self_.aes = aes as i32;
}

/// `IIS_FormatConverter_Config_SetPAS(self, pas)`
pub fn iis_format_converter_config_set_pas(self_: &mut IisFormatConverter, pas: u32) {
    debug_assert!(pas <= 7);
    self_.pas = pas as i32;
}

/// `IIS_FormatConverter_Config_SetImmersiveDownmixFlag(self, immersiveFlag)`
pub fn iis_format_converter_config_set_immersive_downmix_flag(
    self_: &mut IisFormatConverter,
    immersive_flag: u32,
) -> i32 {
    if immersive_flag != 0 && immersive_flag != 1 {
        return -1;
    }
    self_.member.immersive_downmix_flag = immersive_flag;
    0
}

/// `IIS_FormatConverter_Config_SetRendering3DTypeFlag(self, rendering3DtypeFlag)`
pub fn iis_format_converter_config_set_rendering3_d_type_flag(
    self_: &mut IisFormatConverter,
    rendering3_dtype_flag: u32,
) -> i32 {
    if rendering3_dtype_flag != 0 && rendering3_dtype_flag != 1 {
        return -1;
    }
    self_.member.rendering3_d_type_flag = rendering3_dtype_flag;
    0
}

/// `IIS_FormatConverter_GetDelay(self, delay)`
pub fn iis_format_converter_get_delay(self_: &IisFormatConverter, delay: &mut u32) -> i32 {
    /* stftFrameSize equals the delay caused by the FormatConverter.
    In case of IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN stftFrameSize and delay are zero.
    */
    *delay = self_.member.stft_frame_size;
    0
}

/// `IIS_FormatConverter_IsOpen(self)`
pub fn iis_format_converter_is_open(self_: &IisFormatConverter) -> i32 {
    self_.member.open_success
}

/**********************************************/
/*                                            */
/*      Start of data processing methods      */
/*                                            */
/**********************************************/

/// Minimum size in bytes of the work buffer handed over to `IIS_FormatConverter_Open()`.
pub const FORMAT_CONVERTER_OPEN_BUF_SIZE_MIN: u32 = (core::mem::size_of::<i32>()
    * (6 * IN_OUT_N
        + 4 * (IN_OUT_N * core::mem::size_of::<FixpDmxH>()) / core::mem::size_of::<i32>()
        + MAX_NUM_DMX_RULES * 8)) as u32;

/// `IIS_FormatConverter_Open(self, p_buffer, buf_size)`: call this after all `_Config_` methods
/// have been called. The format converter then initializes itself according to those settings
/// (downmix matrix, EQs, STFT filterbanks, ...).
///
/// The C++ function takes a scratch buffer of `buf_size` bytes; the port allocates its temporary
/// data itself and only keeps the size check (returns -1 if `buf_size` is too small).
pub fn iis_format_converter_open(self_: &mut IisFormatConverter, buf_size: u32) -> i32 {
    let mut err;

    let num_channels = self_.num_local_speaker as i32;
    let mut num_lfes = 0u32;
    let mut channel_in_vec = [0i32; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS];
    let mut channel_out_vec = [0i32; FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS];
    let mut num_unknown_out_ch = 0u32;
    let mut num_unknown_in_ch = 0u32;

    let mut unknown_out_ch_vec = [0i32; FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS];
    let mut unknown_in_ch_vec = [0i32; FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS];

    if buf_size < FORMAT_CONVERTER_OPEN_BUF_SIZE_MIN {
        return -1;
    }

    'cleanup: {
        let p = &mut *self_.member;
        p.aes = self_.aes;

        /* match known channels in channel config */
        err = format_converter_match_ch_config2_channels(
            &p.input_channel_geo,
            0,
            p.num_total_input_channels as i32,
            &mut channel_in_vec,
            &mut num_unknown_in_ch,
            &mut unknown_in_ch_vec,
        );

        if err != 0 {
            break 'cleanup;
        }

        if num_unknown_in_ch > 0 && p.fc_output_format != FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC
        {
            p.in_channel_vbap_flag = 1;
        }

        p.num_total_input_channels = 0;

        for i in 0..p.num_input_channel_groups as usize {
            p.num_total_input_channels += p.num_input_channels[i];
        }

        if p.num_total_input_channels == 0 {
            /* nothing to be done: avoid allocation of the full FormatConverter */
            break 'cleanup;
        }

        cicp2geometry_get_number_of_lfes(
            &p.out_channel_geo,
            self_.num_local_speaker,
            &mut num_lfes,
        );

        /* Error can be ignored, because _p->cicpLayoutIndex is only required to detect target
         * layout 5 and 6 for immersive rendering. */
        cicp2geometry_get_cicp_index_from_geometry(
            &p.out_channel_geo,
            num_channels - num_lfes as i32,
            num_lfes as i32,
            &mut p.cicp_layout_index,
        );

        /* OUTPUT: match known channels in channel config */
        err = format_converter_match_ch_config2_channels(
            &p.out_channel_geo,
            1,
            self_.num_local_speaker as i32,
            &mut channel_out_vec,
            &mut num_unknown_out_ch,
            &mut unknown_out_ch_vec,
        );

        if err != 0 {
            break 'cleanup;
        }

        if num_unknown_out_ch > 0 {
            p.fc_input_format = FDK_FORMAT_CONVERTER_INPUT_FORMAT_LISTOFCHANNELS;
            p.fc_output_format = FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_LISTOFCHANNELS;
            p.out_channel_vbap_flag = 1;
        }

        /* apply channel config as output setup in format converter and clean up */
        if p.fc_output_format != FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC {
            format_converter_set_in_out_format(p, 1, num_channels, &channel_out_vec);
        }

        /* apply channel config as input setup in format converter and clean up */
        let n_in = p.num_total_input_channels as i32;
        format_converter_set_in_out_format(p, 0, n_in, &channel_in_vec);

        err = format_converter_open(p.mode, p);
        if err != 0 {
            break 'cleanup;
        }

        if p.mode == IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
            || p.mode == IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
        {
            err = init_stft(p);
            if err != 0 {
                break 'cleanup;
            }
        }

        let cf = p.fc_center_frequencies;
        err = format_converter_init(p, cf);

        match err {
            0 => {
                /* Everything's fine */
                /* Generic/Immersive control code */
                format_converter_dmx_matrix_control(p);
                /* Exponent calculation */
                format_converter_dmx_matrix_exponent(p);
            }
            -2 | -8 => {
                p.fc_input_format = FDK_FORMAT_CONVERTER_INPUT_FORMAT_GENERIC;
                p.fc_output_format = FDK_FORMAT_CONVERTER_OUTPUT_FORMAT_GENERIC;
                p.out_channel_vbap_flag = 1;

                /* close, open, initialize for generic input/output setups: */
                format_converter_close(p);
                match p.mode {
                    IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_PHASE_ALIGN
                    | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_PHASE_ALIGN => {
                        p.mode = IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_PHASE_ALIGN;
                    }
                    IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
                    | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT => {
                        p.mode = IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT;
                    }
                    _ => {}
                }

                err = format_converter_open(p.mode, p);
                if err != 0 {
                    break 'cleanup;
                }
                p.fc_params.as_mut().unwrap().generic_io_fmt = 1;
                let cf = p.fc_center_frequencies;
                err = format_converter_init(p, cf);
                if err != 0 {
                    break 'cleanup;
                }
            }
            _ => {
                break 'cleanup;
            }
        }

        set_custom_downmix_parameters(p, self_.aes, self_.pas);

        {
            if p.out_channel_vbap_flag != 0 {
                err = apply_full_vbap_fallback(p);
                if err != 0 {
                    break 'cleanup;
                }
                /* Generic/Immersive control code */
                format_converter_dmx_matrix_control(p);
                /* Exponent calculation */
                format_converter_dmx_matrix_exponent(p);
            }

            /*if( numUnknownCh > 0 )*/
            if p.in_channel_vbap_flag != 0 && p.out_channel_vbap_flag == 0 {
                err = apply_partial_vbap_input_fallback(p, num_unknown_in_ch, &unknown_in_ch_vec);
                if err != 0 {
                    break 'cleanup;
                }
                /* Generic/Immersive control code */
                format_converter_dmx_matrix_control(p);
                /* Exponent calculation */
                format_converter_dmx_matrix_exponent(p);
            }
        }

        p.open_success = 1;
    }

    /* FC_OPEN_CLEANUP_AND_RETURN: */
    if err == 0 {
        check_configuration(self_)
    } else {
        err
    }
}

/***************************************
 * FormatConverter with STFT Processing *
 ****************************************/
fn iis_format_converter_process_stft(
    self_: &mut IisFormatConverter,
    mut h_drc_dec: Option<&mut DrcDecoder>,
    dein_buffer: &mut [Option<&mut [FixpDbl]>],
    deout_buffer: &mut [FixpDbl],
) -> i32 {
    let p = &mut *self_.member;
    let sfs = p.stft_frame_size as usize;
    let fs = p.frame_size as usize;

    for i in 0..p.num_time_slots as usize {
        let mut stft_headroom = 31;
        let mut stft_headroom_prescaling;
        for ch in 0..p.num_total_input_channels as usize {
            let Some(inp) = dein_buffer[ch].as_deref() else { continue };
            stft_headroom = f_min(stft_headroom, get_scalefactor(&inp[i * sfs..(i + 1) * sfs]));
        }
        for ch in 0..p.num_output_channels as usize {
            let o = ch * fs + i * sfs;
            stft_headroom = f_min(stft_headroom, get_scalefactor(&deout_buffer[o..o + sfs]));
        }

        /* Ensure 8 bit headroom */
        stft_headroom_prescaling = f_max(stft_headroom - 8, 0);
        /* Don't prescale more than necessary */
        stft_headroom_prescaling = f_min(stft_headroom_prescaling, 8);
        /* Ensure at least one bit headroom for FFT */
        if stft_headroom == 0 {
            stft_headroom_prescaling = -1;
        }

        /* Store for use in  the format converter. The minimum is used, because the previous
        signal may be with less headroom than the current one and saturation may occur
        (the previous signal is overlapped with the current one) */
        p.stft_headroom_prescaling = f_min(p.stft_headroom_prescaling, stft_headroom_prescaling);

        /* Transform into frequency domain*/
        for ch in 0..p.num_total_input_channels as usize {
            let Some(inp) = dein_buffer[ch].as_deref_mut() else {
                p.input_buffer_stft[ch][..p.stft_length as usize].fill(0);
                continue;
            };
            stft_filterbank_process(
                &mut inp[i * sfs..],
                &mut p.input_buffer_stft[ch],
                p.stft_filterbank_analysis[ch].as_mut().unwrap(),
                p.stft_headroom_prescaling,
            );
        }

        if let Some(drc) = h_drc_dec.as_deref_mut() {
            /* Delay of DRC gains: Delay of STFT Analysis (256) - 128 because MPEG-D DRC downsamples
             * at the middle of each timeslot*/
            drc.process_freq(
                128,
                DRC_DEC_DRC1,
                0,
                0,
                p.num_total_input_channels as i32,
                i as i32,
                &mut p.input_buffer_stft[..],
                None,
            );
        }

        format_converter_process_stft(p);

        /* Transform into time domain */
        for ch in 0..p.num_output_channels as usize {
            let o = ch * fs + i * sfs;
            stft_filterbank_process(
                &mut p.output_buffer_stft[ch],
                &mut deout_buffer[o..],
                p.stft_filterbank_synthesis[ch].as_mut().unwrap(),
                p.stft_headroom_prescaling,
            );
        }

        /* Keep the current signal headroom correction for use in the next frame */
        p.stft_headroom_prescaling = stft_headroom_prescaling;
    }
    0
}

/// Maximum number of STFT time slots per frame of [`iis_format_converter_process_stft_par`].
const MAX_PAR_TIME_SLOTS: usize = 8;

/// STFT analysis of all time slots of one input channel (a task of
/// [`iis_format_converter_process_stft_par`]).
struct AnalysisTask<'a> {
    input: Option<&'a mut [FixpDbl]>,
    filterbank: Option<&'a mut StftFilterbank>,
    spectra: [Option<&'a mut [FixpDbl]>; MAX_PAR_TIME_SLOTS],
}

/// STFT synthesis of all time slots of one output channel.
struct SynthesisTask<'a> {
    output: &'a mut [FixpDbl],
    filterbank: &'a mut StftFilterbank,
    spectra: [Option<&'a mut [FixpDbl]>; MAX_PAR_TIME_SLOTS],
}

/// `IIS_FormatConverter_Process_STFT()` with the STFT analysis (per input channel) and synthesis
/// (per output channel) of all time slots of the frame distributed over `pool`.
///
/// The sequential code processes the time slots one after the other: headroom, analysis of all
/// input channels, DRC, active downmix, synthesis of all output channels. Here the headroom and
/// the prescaling of every time slot are determined first, then the analysis of all time slots
/// runs per input channel in parallel, then DRC and active downmix run per time slot
/// sequentially, and finally the synthesis of all time slots per output channel in parallel.
/// This is exact because
/// - the headroom of time slot `i` depends only on the input and output samples of time slot `i`,
///   which the processing of the earlier time slots does not modify (the analysis scales and the
///   synthesis writes only the samples of its own time slot);
/// - the analysis (synthesis) filterbank of a channel is only used by that channel, in time slot
///   order, with the prescaling of the time slot;
/// - every time slot gets its own input and output spectra; the DRC and the active downmix of
///   time slot `i` see exactly the spectra of time slot `i` and the prescaling of time slot `i` in
///   `stftHeadroomPrescaling`. The spectra of the last time slot are the instance buffers
///   (`inputBufferStft` / `outputBufferStft`), so they hold the same content after the frame.
fn iis_format_converter_process_stft_par(
    self_: &mut IisFormatConverter,
    pool: &ThreadPool,
    mut h_drc_dec: Option<&mut DrcDecoder>,
    dein_buffer: &mut [Option<&mut [FixpDbl]>],
    deout_buffer: &mut [FixpDbl],
) -> i32 {
    let p = &mut *self_.member;
    let sfs = p.stft_frame_size as usize;
    let fs = p.frame_size as usize;
    let n_slots = p.num_time_slots as usize;
    let n_in = p.num_total_input_channels as usize;
    let n_out = p.num_output_channels as usize;
    let len = p.stft_length as usize;

    /* headroom and prescaling of every time slot */
    let mut prescaling = [0i32; MAX_PAR_TIME_SLOTS]; /* stftHeadroomPrescaling of the slot */
    let mut next_prescaling = [0i32; MAX_PAR_TIME_SLOTS]; /* the value kept for the next slot */
    let mut state = p.stft_headroom_prescaling;
    for i in 0..n_slots {
        let mut stft_headroom = 31;
        let mut stft_headroom_prescaling;
        for ch in 0..n_in {
            let Some(inp) = dein_buffer[ch].as_deref() else { continue };
            stft_headroom = f_min(stft_headroom, get_scalefactor(&inp[i * sfs..(i + 1) * sfs]));
        }
        for ch in 0..n_out {
            let o = ch * fs + i * sfs;
            stft_headroom = f_min(stft_headroom, get_scalefactor(&deout_buffer[o..o + sfs]));
        }
        /* Ensure 8 bit headroom */
        stft_headroom_prescaling = f_max(stft_headroom - 8, 0);
        /* Don't prescale more than necessary */
        stft_headroom_prescaling = f_min(stft_headroom_prescaling, 8);
        /* Ensure at least one bit headroom for FFT */
        if stft_headroom == 0 {
            stft_headroom_prescaling = -1;
        }
        state = f_min(state, stft_headroom_prescaling);
        prescaling[i] = state;
        next_prescaling[i] = stft_headroom_prescaling;
        state = stft_headroom_prescaling;
    }

    /* spectra of the time slots 0..n_slots - 1 (the last one uses the instance buffers) */
    if p.slot_input_buffer_stft.len() != n_slots - 1
        || p.slot_input_buffer_stft.iter().any(|s| s.len() != n_in)
        || p.slot_output_buffer_stft.iter().any(|s| s.len() != n_out)
    {
        p.slot_input_buffer_stft = vec![vec![vec![0; len]; n_in]; n_slots - 1];
        p.slot_output_buffer_stft = vec![vec![vec![0; len]; n_out]; n_slots - 1];
    }

    /* Transform into frequency domain */
    {
        let mut tasks: Vec<AnalysisTask> = dein_buffer[..n_in]
            .iter_mut()
            .zip(p.stft_filterbank_analysis.iter_mut())
            .map(|(inp, fb)| AnalysisTask {
                input: inp.as_deref_mut(),
                filterbank: fb.as_deref_mut(),
                spectra: Default::default(),
            })
            .collect();
        for (i, slot) in
            p.slot_input_buffer_stft.iter_mut().chain([&mut p.input_buffer_stft]).enumerate()
        {
            for (task, spec) in tasks.iter_mut().zip(slot.iter_mut()) {
                task.spectra[i] = Some(&mut spec[..]);
            }
        }
        pool.for_each_mut(&mut tasks, |_, t| {
            for i in 0..n_slots {
                let spec = t.spectra[i].as_deref_mut().unwrap();
                match t.input.as_deref_mut() {
                    Some(inp) => stft_filterbank_process(
                        &mut inp[i * sfs..],
                        spec,
                        t.filterbank.as_deref_mut().unwrap(),
                        prescaling[i],
                    ),
                    None => spec[..len].fill(0),
                }
            }
        });
    }

    for i in 0..n_slots {
        let last = i == n_slots - 1;
        if !last {
            core::mem::swap(&mut p.input_buffer_stft, &mut p.slot_input_buffer_stft[i]);
            core::mem::swap(&mut p.output_buffer_stft, &mut p.slot_output_buffer_stft[i]);
        }
        p.stft_headroom_prescaling = prescaling[i];

        if let Some(drc) = h_drc_dec.as_deref_mut() {
            /* Delay of DRC gains: Delay of STFT Analysis (256) - 128 because MPEG-D DRC downsamples
             * at the middle of each timeslot*/
            drc.process_freq(
                128,
                DRC_DEC_DRC1,
                0,
                0,
                n_in as i32,
                i as i32,
                &mut p.input_buffer_stft[..],
                None,
            );
        }

        format_converter_process_stft(p);

        if !last {
            core::mem::swap(&mut p.input_buffer_stft, &mut p.slot_input_buffer_stft[i]);
            core::mem::swap(&mut p.output_buffer_stft, &mut p.slot_output_buffer_stft[i]);
        }
        /* Keep the current signal headroom correction for use in the next frame */
        p.stft_headroom_prescaling = next_prescaling[i];
    }

    /* Transform into time domain */
    {
        let mut tasks: Vec<SynthesisTask> = deout_buffer
            .chunks_mut(fs)
            .zip(p.stft_filterbank_synthesis.iter_mut())
            .take(n_out)
            .map(|(output, fb)| SynthesisTask {
                output,
                filterbank: fb.as_deref_mut().unwrap(),
                spectra: Default::default(),
            })
            .collect();
        assert_eq!(tasks.len(), n_out, "format converter: output buffer too small");
        for (i, slot) in
            p.slot_output_buffer_stft.iter_mut().chain([&mut p.output_buffer_stft]).enumerate()
        {
            for (task, spec) in tasks.iter_mut().zip(slot.iter_mut()) {
                task.spectra[i] = Some(&mut spec[..]);
            }
        }
        pool.for_each_mut(&mut tasks, |_, t| {
            for i in 0..n_slots {
                stft_filterbank_process(
                    t.spectra[i].as_deref_mut().unwrap(),
                    &mut t.output[i * sfs..],
                    t.filterbank,
                    prescaling[i],
                );
            }
        });
    }
    0
}

/// `IIS_FormatConverter_Process(self, hDrcDec, deinBuffer, outBuffer, inputBufferChannelOffset)`:
/// convert one frame.
///
/// `dein_buffer[ch]` is the input signal of input channel `ch` (at least `frameSize` samples,
/// `None` for a `NULL` pointer, i.e. a silent channel). In the STFT modes the input samples are
/// scaled in place like in C (`StftFilterbank_Process()` applies the headroom to its input), so
/// the input is modified. `out_buffer` holds `numOutputChannels` blocks of `frameSize` samples;
/// the converted signal is added to its content (STFT synthesis overlap-add / passive
/// accumulation), so the caller has to provide cleared (or pre-mixed) output. If `h_drc_dec` is
/// given, the DRC gains are applied in the STFT domain (`FDK_drcDec_ProcessFreq()`).
/// `input_buffer_channel_offset` is unused (like in C). Returns 0 on success, -1 for
/// unsupported modes.
pub fn iis_format_converter_process(
    self_: &mut IisFormatConverter,
    h_drc_dec: Option<&mut DrcDecoder>,
    dein_buffer: &mut [Option<&mut [FixpDbl]>],
    out_buffer: &mut [FixpDbl],
    input_buffer_channel_offset: i32,
) -> i32 {
    iis_format_converter_process_pool(
        self_,
        None,
        h_drc_dec,
        dein_buffer,
        out_buffer,
        input_buffer_channel_offset,
    )
}

/// [`iis_format_converter_process`] with the STFT analysis and synthesis distributed over `pool`
/// (not in C; the result is identical, see `iis_format_converter_process_stft_par()`).
pub fn iis_format_converter_process_pool(
    self_: &mut IisFormatConverter,
    pool: Option<&ThreadPool>,
    h_drc_dec: Option<&mut DrcDecoder>,
    dein_buffer: &mut [Option<&mut [FixpDbl]>],
    out_buffer: &mut [FixpDbl],
    input_buffer_channel_offset: i32,
) -> i32 {
    iis_format_converter_process_fast(
        self_,
        pool,
        h_drc_dec,
        dein_buffer,
        out_buffer,
        input_buffer_channel_offset,
        false,
    )
}

/// [`iis_format_converter_process_pool`] with the fast mode (not in C, **not bit-exact**): if
/// `bypass` is set, no DRC is applied in the STFT domain (`h_drc_dec` is `None`) and the
/// conditions of [`format_converter_bypass_map`](crate::format_converter_bypass_map) hold, the
/// STFT processing is replaced by the equivalent delayed copy of the input channels (see
/// [`crate::format_converter_fast`]). With `bypass == false` the result is the one of
/// `IIS_FormatConverter_Process()`, except after bypassed frames (the STFT processing then
/// continues from the state of the bypass).
#[allow(clippy::too_many_arguments)]
pub fn iis_format_converter_process_fast(
    self_: &mut IisFormatConverter,
    pool: Option<&ThreadPool>,
    h_drc_dec: Option<&mut DrcDecoder>,
    dein_buffer: &mut [Option<&mut [FixpDbl]>],
    out_buffer: &mut [FixpDbl],
    _input_buffer_channel_offset: i32,
    bypass: bool,
) -> i32 {
    let mut error = 0;

    {
        let p = &*self_.member;
        if p.num_total_input_channels == 0 {
            return 0;
        }
        if p.fc_params.is_none() {
            /* fails if IIS_FormatConverter_Open() was not called before */
            return 0;
        }
    }

    /* fast mode (not in C) */
    if bypass || self_.member.fast_bypass.is_some() {
        if bypass
            && h_drc_dec.is_none()
            && crate::format_converter_fast::format_converter_bypass(
                &mut self_.member,
                dein_buffer,
                out_buffer,
            )
        {
            return 0;
        }
        crate::format_converter_fast::format_converter_leave_bypass(&mut self_.member);
    }

    /* deoutBuffer[i] = &outBuffer[i * frameSize] */

    match self_.member.mode {
        IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT => {
            let slots = self_.member.num_time_slots as usize;
            error = match pool {
                Some(pool) if (1..=MAX_PAR_TIME_SLOTS).contains(&slots) => {
                    iis_format_converter_process_stft_par(
                        self_,
                        pool,
                        h_drc_dec,
                        dein_buffer,
                        out_buffer,
                    )
                }
                _ => iis_format_converter_process_stft(self_, h_drc_dec, dein_buffer, out_buffer),
            };
        }
        IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN => {
            format_converter_process_passive_time_domain_frame_length(
                dein_buffer,
                out_buffer,
                &self_.member,
            );
        }
        _ => {
            error = -1;
        }
    }

    error
}

/// `IIS_FormatConverter_Close(self)`: free the instance (`*self_` becomes `None`).
pub fn iis_format_converter_close(self_: &mut Option<Box<IisFormatConverter>>) -> i32 {
    let Some(s) = self_.as_mut() else { return 0 };
    let p = &mut *s.member;

    /*********  STFT **************/
    for i in 0..p.num_total_input_channels as usize {
        stft_filterbank_close(&mut p.stft_filterbank_analysis[i]);
    }
    for i in 0..p.num_output_channels as usize {
        stft_filterbank_close(&mut p.stft_filterbank_synthesis[i]);
    }
    p.prev_input_buffer_stft = Vec::new();
    p.input_buffer_stft = Vec::new();
    p.output_buffer_stft = Vec::new();
    format_converter_close(p);

    *self_ = None;
    0
}

/**********************************************/
/*                                            */
/*              Private Methods               */
/*                                            */
/**********************************************/

/// `_checkConfiguration()`
fn check_configuration(self_: &mut IisFormatConverter) -> i32 {
    match self_.member.mode {
        IIS_FORMATCONVERTER_MODE_PASSIVE_TIME_DOMAIN
        | IIS_FORMATCONVERTER_MODE_PASSIVE_FREQ_DOMAIN => {
            self_.aes = 0;
            self_.pas = 0;
        }
        IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_PHASE_ALIGN => {
            self_.aes = 7;
            self_.pas = 3;
        }
        IIS_FORMATCONVERTER_MODE_ACTIVE_FREQ_DOMAIN_STFT
        | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT
        | IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_PHASE_ALIGN => {
            if self_.aes < 0 || self_.aes > 7 {
                return -1;
            }
            if self_.pas < 0 || self_.pas > 7 {
                return -1;
            }
        }
        _ => {
            return -1;
        }
    }
    0
}

/// `_applyLfeRuleSet()`
fn apply_lfe_rule_set(
    input_channel_geo: &[Cicp2GeometryChannelGeometry],
    num_input_channels: u32,
    output_channel_geo: &[Cicp2GeometryChannelGeometry],
    num_output_channels: u32,
    dmx_mtx: &mut [Vec<FixpDmxH>],
) -> i32 {
    let mut num_in_lfes = 0u32;
    let mut num_out_lfes = 0u32;

    cicp2geometry_get_number_of_lfes(input_channel_geo, num_input_channels, &mut num_in_lfes);
    cicp2geometry_get_number_of_lfes(output_channel_geo, num_output_channels, &mut num_out_lfes);

    if num_out_lfes == 0 || num_in_lfes == 0 {
        return 0;
    }

    if num_out_lfes == 1 {
        let mut lfe_index: i32 = -1;
        for inp in 0..num_input_channels as usize {
            for out in 0..num_output_channels as usize {
                if input_channel_geo[inp].lfe != 0 || output_channel_geo[out].lfe != 0 {
                    dmx_mtx[inp][out] = 0;
                }
            }
        }

        for out in 0..num_output_channels as usize {
            if output_channel_geo[out].lfe != 0 {
                lfe_index = out as i32;
            }
        }

        for inp in 0..num_input_channels as usize {
            if input_channel_geo[inp].lfe != 0 {
                dmx_mtx[inp][lfe_index as usize] = MAXVAL_DMX_H;
            }
        }
        return 0;
    } /* End: ( numOutLfes == 1 ) */

    0
}

/// `gVBAPRenderer_GetStaticGains()`: creates the gVBAP DMX gains of `num_objects` input
/// positions for the output setup.
fn gvbap_renderer_get_static_gains(
    in_geometry_info: &[Cicp2GeometryChannelGeometry],
    num_objects: i32,
    out_cicp_index: i32,
    out_geometry_info: &[Cicp2GeometryChannelGeometry],
    out_channels: i32,
    static_gains_matrix: &mut [Vec<FixpDmxH>],
) -> i32 {
    /* Initialization of the renderization parameters in order to create a new downmix matrix. */
    let mut hg_vbap_renderer: Option<Box<GVbapRenderer>> = None;
    if gvbap_renderer_open(
        &mut hg_vbap_renderer,
        num_objects,
        1,
        1,
        out_geometry_info,
        out_channels,
        out_cicp_index,
        1,
        1,
    ) != 0
    {
        return 1; /* memory allocation error */
    }
    let h = hg_vbap_renderer.as_mut().unwrap();

    let mut gains = vec![0 as FixpDbl; out_channels.max(0) as usize];
    for i in 0..num_objects.max(0) as usize {
        let mut sample = OamSample::default();
        sample.sph.azi = (in_geometry_info[i].az as i64 * 11930464i32 as i64) as i32;
        sample.sph.ele = in_geometry_info[i].el.wrapping_mul(11930464);
        sample.sph.rad = fl2fxconst_dbl(1.0 / 16.0);
        sample.gain = MAXVAL_DBL;

        /* calculate cartesian stop position from spherical to cartesian */
        sample.cart = spherical_to_cartesian(sample.sph); /* This does the conversion */
        /* calculate gains */
        calculate_vbap(h, &sample, &mut gains, 0); /* This will do the calculation */

        for j in 0..out_channels as usize {
            let mapped_channel = if !h.speaker_setup.mapping.is_empty() {
                h.speaker_setup.mapping[j] as usize
            } else {
                j
            };
            static_gains_matrix[i][mapped_channel] = fx_dbl2fx_dmx_h(gains[j]);
        }
    }

    0
}

/// `_applyPartialVbapInputFallback()`
fn apply_partial_vbap_input_fallback(
    p: &mut IisFormatConverterInternal,
    num_unknown_in_ch: u32,
    unknown_in_ch_vec: &[i32],
) -> i32 {
    let nin = p.num_total_input_channels as usize;
    let nout = p.num_output_channels as usize;
    let nun = num_unknown_in_ch as usize;
    let mut dmx_mtx = vec![vec![0 as FixpDmxH; nout]; nin];

    /* allocate temporary memory */
    let mut unknown_channels_geo = vec![Cicp2GeometryChannelGeometry::default(); nun];
    let mut tmp_dmx_mtx = vec![vec![0 as FixpDmxH; nout]; nun];
    for ch in 0..nun {
        let g = &p.input_channel_geo[unknown_in_ch_vec[ch] as usize];
        unknown_channels_geo[ch].az = g.az;
        unknown_channels_geo[ch].el = g.el;
        unknown_channels_geo[ch].lfe = g.lfe;
    }

    /* obtain gVBAP DMX gains: */
    let err = gvbap_renderer_get_static_gains(
        &unknown_channels_geo,
        nun as i32,
        -1,
        &p.out_channel_geo,
        nout as i32,
        &mut tmp_dmx_mtx,
    );
    if err != 0 {
        return 1;
    }

    /* copy gains for unknown channels into full-size DMX matrix */
    for ch in 0..nun {
        for j in 0..nout {
            dmx_mtx[unknown_in_ch_vec[ch] as usize][j] = tmp_dmx_mtx[ch][j];
        }
    }

    apply_lfe_rule_set(
        &p.input_channel_geo,
        p.num_total_input_channels,
        &p.out_channel_geo,
        p.num_output_channels,
        &mut dmx_mtx,
    );

    /* post-process gains to reduce phantom sources */
    format_converter_postprocess_dmx_mtx(Some(&mut dmx_mtx), nin as u32, nout as u32);

    if format_converter_add_dmx_mtx(Some(&dmx_mtx), p) != 0 {
        return -1;
    }

    0
}

/// `_applyFullVbapFallback()`
fn apply_full_vbap_fallback(p: &mut IisFormatConverterInternal) -> i32 {
    let nin = p.num_total_input_channels;
    let nout = p.num_output_channels;

    /* obtain gVBAP DMX gains: */
    let err = {
        let params = p.fc_params.as_mut().unwrap();
        gvbap_renderer_get_static_gains(
            &p.input_channel_geo,
            nin as i32,
            -1,
            &p.out_channel_geo,
            nout as i32,
            &mut params.dmx_mtx,
        )
    };
    if err != 0 {
        return err;
    }

    /* post-process gains to reduce phantom sources */
    format_converter_postprocess_dmx_mtx(
        Some(&mut p.fc_params.as_mut().unwrap().dmx_mtx),
        nin,
        nout,
    );

    /* The LFE rule set is applied to a temporary (zero) matrix which is discarded. */
    let mut tmp_dmx_mtx = vec![vec![0 as FixpDmxH; nout as usize]; nin as usize];
    apply_lfe_rule_set(&p.input_channel_geo, nin, &p.out_channel_geo, nout, &mut tmp_dmx_mtx);

    format_converter_set_dmx_mtx(p)
}

/// `FormatConverterFrame(FormatConverter, hBitStr)`: parse the format converter extension
/// element (`ID_EXT_ELE_FMT_CNVRTR`). Always returns 1.
pub fn format_converter_frame(
    format_converter: &mut IisFormatConverter,
    h_bit_str: &mut FdkBitStream,
) -> i32 {
    let rendering3_d_type = h_bit_str.read_bit();

    iis_format_converter_config_set_rendering3_d_type_flag(format_converter, rendering3_d_type);

    1
}

/// `_initSTFT()`
fn init_stft(p: &mut IisFormatConverterInternal) -> i32 {
    let mut status = 0;
    p.stft_filterbank_config_analysis.frame_size = p.stft_frame_size;
    p.stft_filterbank_config_analysis.fft_size = p.stft_length;
    p.stft_filterbank_config_analysis.stft_filterbank_mode = STFT_FILTERBANK_MODE_TIME_TO_FREQ;

    p.stft_filterbank_config_synthesis = p.stft_filterbank_config_analysis;
    p.stft_filterbank_config_synthesis.stft_filterbank_mode = STFT_FILTERBANK_MODE_FREQ_TO_TIME;

    let len = p.stft_length as usize;

    /*Initialize Analysis filterbank*/
    p.input_buffer_stft = vec![vec![0; len]; p.num_total_input_channels as usize];

    if p.immersive_downmix_flag != 0 {
        /* Initialize prevInputBufferStft */
        p.prev_input_buffer_stft = vec![vec![0; len]; TFRA + 1];
    } else {
        p.prev_input_buffer_stft = Vec::new();
    }

    for ch in 0..p.num_total_input_channels as usize {
        let err = stft_filterbank_open(
            &p.stft_filterbank_config_analysis,
            &mut p.stft_filterbank_analysis[ch],
        );
        if err != 0 {
            return -1;
        }
    }

    /*Initialize Synthesis filterbank*/
    p.output_buffer_stft = vec![vec![0; len]; p.num_output_channels as usize];

    for ch in 0..p.num_output_channels as usize {
        let err = stft_filterbank_open(
            &p.stft_filterbank_config_synthesis,
            &mut p.stft_filterbank_synthesis[ch],
        );
        if err != 0 {
            status = -1;
            return status;
        }
    }
    status
}
