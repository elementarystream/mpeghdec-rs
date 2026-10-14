// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from include/mpeghdecoder.h, include/mpeghUIManager.h and mpeghdec.pc.in (the C API of
// the library).

//! C ABI of mpeghdec: a drop-in replacement for the `mpeghdec` library of the reference
//! implementation.
//!
//! The library exports the 17 functions of `mpeghdecoder.h` and `mpeghUIManager.h` (the headers
//! are in `include/mpeghdec/`, unchanged) with the behaviour of the reference implementation,
//! including its error codes. The functions pass their arguments to the straight ports of the C
//! APIs ([`mpeghdec_core::mpeghdecoder`], [`mpeghdec_uimanager::mpegh_ui_manager`]); see
//! [`dec`] and [`ui`] for threads, panics and the few deviations for invalid pointers.
//!
//! `cargo build --release -p mpeghdec-capi` builds `libmpeghdec_capi.{a,so,dylib}`; install it as
//! `libmpeghdec.*` with the headers and `mpeghdec.pc` (see `scripts/capi-test.sh`).

#![allow(non_snake_case, non_camel_case_types)]

pub mod dec;
pub mod ui;
