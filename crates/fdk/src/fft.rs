// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/fft.h and libFDK/src/fft.cpp.
//
// The reference build only supports the radix-2 lengths 64, 128, 256 and 512 (all other lengths,
// including every IFFT length, hit FDK_ASSERT(0) in C++).

//! Complex FFT.

use crate::common_fix::FixpDbl;
use crate::fft_rad2::dit_fft;
use crate::tools_rom::SINE_TABLE_512;

pub const SCALEFACTOR2048: i32 = 10;
pub const SCALEFACTOR1024: i32 = 9;
pub const SCALEFACTOR512: i32 = 8;
pub const SCALEFACTOR256: i32 = 7;
pub const SCALEFACTOR128: i32 = 6;
pub const SCALEFACTOR64: i32 = 5;
pub const SCALEFACTOR32: i32 = 4;
pub const SCALEFACTOR16: i32 = 3;
pub const SCALEFACTOR8: i32 = 2;
pub const SCALEFACTOR4: i32 = 1;
pub const SCALEFACTOR2: i32 = 1;

/// `fft(length, pInput, pScalefactor)`: in-place complex valued FFT of length `length`.
///
/// `input` holds `length` interleaved real/imaginary pairs; the input data must have at least 1
/// bit headroom. `scalefactor` is updated according to the FFT scale. Supported lengths: 64, 128,
/// 256 and 512; other lengths leave the data unchanged like the release build of the reference
/// (`FDK_ASSERT`, a `debug_assert!` here).
pub fn fft(length: i32, input: &mut [FixpDbl], scalefactor: &mut i32) {
    match length {
        64 => {
            dit_fft(input, 6, &SINE_TABLE_512, 512);
            *scalefactor += SCALEFACTOR64;
        }
        128 => {
            dit_fft(input, 7, &SINE_TABLE_512, 512);
            *scalefactor += SCALEFACTOR128;
        }
        256 => {
            dit_fft(input, 8, &SINE_TABLE_512, 512);
            *scalefactor += SCALEFACTOR256;
        }
        512 => {
            dit_fft(input, 9, &SINE_TABLE_512, 512);
            *scalefactor += SCALEFACTOR512;
        }
        _ => debug_assert!(false, "fft: FFT length {length} not supported"),
    }
}

/// `ifft(length, pInput, scalefactor)`: no IFFT length is supported by the reference build, the
/// data is left unchanged (`FDK_ASSERT`, a `debug_assert!` here).
pub fn ifft(length: i32, _input: &mut [FixpDbl], _scalefactor: &mut i32) {
    debug_assert!(false, "ifft: IFFT length {length} not supported");
}
