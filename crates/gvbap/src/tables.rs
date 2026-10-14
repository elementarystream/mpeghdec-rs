// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libgVBAPRenderer/include/tables.h and libgVBAPRenderer/src/tables.cpp.

//! Imaginary speaker and loudspeaker superset tables of the enhanced rendering mode.
//!
//! The table data in `tables/imag.rs` and `tables/superset.rs` is generated from the C++ source
//! by `scripts/codegen/gen_gvbap_tables.py`.

mod imag;
mod superset;

pub use imag::get_imag_speaker_table;
pub use superset::get_speaker_super_set_table;
