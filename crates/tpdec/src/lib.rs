// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libMpegTPDec`: MHAS transport decoder, MPEG-H 3D audio config (`mpegh3daConfig()`)
//! parser and audio scene information parser.
//!
//! The transport decoder ([`tpdec_lib::TransportDec`]) owns the input bit buffer. It calls back
//! into the core decoder through the [`tp_data::TpCallbacks`] trait, which the functions that may
//! trigger callbacks take as an argument.

pub mod tp_data;
pub mod tp_version;
pub mod tpdec_asc;
pub mod tpdec_asi;
pub mod tpdec_lib;

pub use tp_data::*;
pub use tpdec_asc::*;
pub use tpdec_asi::*;
pub use tpdec_lib::*;
