// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libDRCdec`: MPEG-D DRC decoder (ISO/IEC 23003-4) as used by MPEG-H 3D Audio.
//!
//! The public interface (`FDK_drcDecLib.h`) is [`DrcDecoder`] in [`drc_dec_lib`].

pub mod drc_dec_gain_decoder;
pub mod drc_dec_lib;
pub mod drc_dec_reader;
pub mod drc_dec_rom;
pub mod drc_dec_selection_process;
pub mod drc_dec_tools;
pub mod drc_dec_types;
pub mod drc_decoder;
pub mod drc_gain_dec_init;
pub mod drc_gain_dec_multiband;
pub mod drc_gain_dec_preprocess;
pub mod drc_gain_dec_process;

pub use drc_dec_lib::*;
pub use drc_dec_selection_process::SelProcOutput;
