// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/include/FDK_formatConverter_process.h and
// libFormatConverter/src/FDK_formatConverter_process.cpp.
// formatConverterProcess_passive_timeDomain() is declared but not implemented in C++.
//
// Notice of the C++ source:
//
// This software module was originally developed by
//
// Fraunhofer IIS
//
// in the course of development of the ISO/IEC 23008-3 for reference purposes and its
// performance may not have been optimized. This software module is an implementation
// of one or more tools as specified by the ISO/IEC 23008-3 standard. ISO/IEC gives
// you a royalty-free, worldwide, non-exclusive, copyright license to copy, distribute,
// and make derivative works of this software module or modifications  thereof for use
// in implementations or products claiming conformance to the ISO/IEC 23008-3 standard
// and which satisfy any specified conformance criteria. Those intending to use this
// software module in products are advised that its use may infringe existing patents.
// ISO/IEC have no liability for use of this software module or modifications thereof.
// Copyright is not released for products that do not conform to the ISO/IEC 23008-3
// standard.
//
// Fraunhofer IIS retains full right to modify and use the code for its own purpose,
// assign or donate the code to a third party and to inhibit third parties from using
// the code for products that do not conform to MPEG-related ITU Recommendations and/or
// ISO/IEC International Standards.
//
// This copyright notice must be included in all copies or derivative works.
//
// Copyright (c) ISO/IEC 2013.

//! Format converter processing: passive time domain downmix and the STFT domain downmix.

use crate::format_converter_active_dmx_stft::active_dmx_process_stft;
use crate::format_converter_intern::*;
use mpeghdec_fdk::common_fix::*;

/// `formatConverterProcess_passive_timeDomain_frameLength(pIn, pOut, fcInt)`: passive downmix
/// in the time domain. `p_in[ch]` holds `frameSize` samples of input channel `ch` (`None` for a
/// `NULL` pointer), `p_out` holds `numOutputChannels` blocks of `frameSize` samples to which the
/// downmix is added.
///
/// Like in C, the downmix coefficient pointer is not advanced for skipped (`NULL`) input
/// channels.
pub fn format_converter_process_passive_time_domain_frame_length(
    p_in: &[Option<&mut [DmxhPcm]>],
    p_out: &mut [DmxhPcm],
    fc_int: &IisFormatConverterInternal,
) {
    /* Buffer definitions:

    Input buffer (pIn):
      - Blockwise:
          Blocks of frameSize samples from channel 0 to numInChannels-1.

    Accumulation output buffer:
      - Blockwise:
          Blockwise accumulated frames with samples from 0 to frameSize-1 in pOut
    */

    let num_out_channels = fc_int.num_output_channels as usize;
    let num_in_channels = fc_int.num_total_input_channels as usize;
    let frame_length = fc_int.frame_size as usize;
    let dmx_mtx = &fc_int.fc_params.as_ref().unwrap().dmx_mtx_sorted;
    let mut m = 0usize;

    for in_ch in 0..num_in_channels {
        let Some(inp) = p_in[in_ch].as_deref() else { continue };

        /* Reset output buffer pointer */
        for out_ch in 0..num_out_channels {
            let dmx_coef = dmx_mtx[m];
            m += 1;
            if dmx_coef != 0 {
                let out = &mut p_out[out_ch * frame_length..(out_ch + 1) * frame_length];
                for (o, &x) in out.iter_mut().zip(inp[..frame_length].iter()) {
                    *o = o.wrapping_add(f_mult(x, dmx_coef));
                }
            }
        }
    }
}

/// `formatConverter_process_STFT(fcInt)`: STFT domain downmix of one time slot.
pub fn format_converter_process_stft(fc_int: &mut IisFormatConverterInternal) {
    active_dmx_process_stft(fc_int); // fcInt->fcState->handleActiveDmxStft
}
