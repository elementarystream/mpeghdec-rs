// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFormatConverter/src/FDK_formatConverter_constants_stft.h and
// libFormatConverter/src/FDK_formatConverter_constants_stft.cpp.

//! Constants of the STFT domain (active) downmix: ERB band borders of the 512 point STFT.

use crate::format_converter_intern::FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS;
use mpeghdec_fdk::common_fix::*;

pub const STFT_FRAME_SIZE: u32 = 256;
pub const STFT_LENGTH: u32 = 2 * STFT_FRAME_SIZE;
pub const STFT_ERB_BANDS: usize = 58;
pub const IIS_FORMATCONVERTER_MAX_OUT_CH: usize = FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS;

/// `ALPHA_AEQ`
pub const ALPHA_AEQ: FixpDbl = fl2fxconst_dbl(0.0435f32 as f64);

/* constants for fft transform size of 512 coefficients */

/// `erb_freq_idx_256_58`: upper STFT band border (exclusive) of each ERB band.
pub static ERB_FREQ_IDX_256_58: [u32; 58] = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32, 33, 36, 39, 43, 47, 51, 55, 60, 65, 70, 77, 84, 91, 98, 106, 115, 125,
    136, 148, 160, 174, 189, 205, 222, 241, 256,
];

/// `erb_freq_idx_256_58_exp`: energy accumulation headroom of each ERB band.
/// Note: last value is a dummy value for easier loop structure.
pub static ERB_FREQ_IDX_256_58_EXP: [i32; 59] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 4, 0,
];
