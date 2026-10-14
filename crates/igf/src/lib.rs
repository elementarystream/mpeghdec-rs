// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libIGFdec`.
//!
//! - [`igf_dec`]: IGF decoder (`FDK_igfDec.h`), bitstream parsing, tile mapping, whitening,
//!   gap filling and temporal noise flattening.
//! - [`igf_scf_decoder`]: arithmetic decoder of the IGF scale factor energies
//!   (`FDK_igfSCFDecoder.h`).

pub mod igf_dec;
pub mod igf_scf_decoder;
