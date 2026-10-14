// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/overlapadd.h.

//! Spectrum layout definitions.
//!
//! `SPECTRAL_PTR` (`FIXP_DBL*`) becomes a slice; the macros become index helpers:
//! `SPEC(ptr, w, gl)` is `&mut ptr[spec(w, gl)..]`.

/// `OverlapBufferSize`: default overlap is half of the frame size.
pub const OVERLAP_BUFFER_SIZE: i32 = 1024 / 2;

/// Length of `SPECTRUM` (`FIXP_DBL[1024]`).
pub const SPECTRUM_LEN: usize = 1024;

/// `SPEC(ptr, w, gl)`: offset of window `w` (granule length `gl`) in a spectrum.
#[inline(always)]
pub const fn spec(w: i32, gl: i32) -> usize {
    (w * gl) as usize
}

/// `SPEC_TCX(ptr, f, gl, fb)`: offset of TCX frame `f`.
#[inline(always)]
pub const fn spec_tcx(f: i32, gl: i32, fb: i32) -> usize {
    (f * (gl * 2) * (if fb == 0 { 1 } else { 2 })) as usize
}
