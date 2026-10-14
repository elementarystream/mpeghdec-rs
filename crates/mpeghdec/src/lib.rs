// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! MPEG-H 3D Audio decoder (ISO/IEC 23008-3, Baseline profile): a bit-exact Rust port of the
//! Fraunhofer FDK MPEG-H decoder.
//!
//! - [`Decoder`] decodes MHAS access units (or raw `mha1` access units with an out-of-band
//!   configuration) with their presentation timestamps and returns the rendered PCM frames for
//!   a CICP target loudspeaker layout.
//! - [`UiManager`] applies user interactivity (XML actions) to an MHAS stream before it is
//!   decoded and describes the audio scene.
//! - [`chunked`] decodes a whole seekable input in parallel chunks.
//!
//! The output is bit-exact with the reference decoder built for the same architecture (AArch64,
//! x86-64 or 32 bit x86, selected by the target; the environment variable `MPEGHDEC_ARCH=arm`,
//! `x86` or `x86_32` at build time selects another one). The opt-in fast mode
//! ([`Decoder::set_fast_mode`]) trades bit-exactness for speed.
//!
//! Environment variables read at run time: `MPEGHDEC_NO_SIMD` (any value) disables the AVX2 code
//! paths on x86 (same output); `MPEGHDEC_POISON` fills scratch buffers with garbage for testing
//! (same output, slower).
//!
//! The C API of the reference decoder (`mpeghdecoder_*()`, `mpegh_UI_*()`) is available as a
//! straight port in `mpeghdec_core::mpeghdecoder` and `mpeghdec_uimanager::mpegh_ui_manager`;
//! the other `mpeghdec-*` crates are internals of this crate without stability guarantees.

#![warn(missing_docs)]

pub mod chunked;
mod decoder;
mod ui;

pub use decoder::*;
pub use ui::*;
