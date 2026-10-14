// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

#![no_main]
//! The UI manager API in front of the decoder API (src/dec.rs).
use libfuzzer_sys::fuzz_target;
use mpeghdec_fuzz::dec::{fuzz, Target};

fuzz_target!(|data: &[u8]| fuzz(Target::Ui, data));
