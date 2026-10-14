// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_core.h and libFDK/src/FDK_core.cpp.

//! FDK tools versioning support.
//!
//! In mpeghdec r4.0.1 `FDK_core.h` declares nothing and `FDK_core.cpp` only defines the library
//! version macros; the `LIB_INFO`/`FDK_MODULE_ID` machinery of other FDK code bases does not
//! exist here (the transport decoder version is in `mpeghdec_tpdec::tp_version`).

/// `FDK_TOOLS_LIB_VL0`
pub const FDK_TOOLS_LIB_VL0: i32 = 4;
/// `FDK_TOOLS_LIB_VL1`
pub const FDK_TOOLS_LIB_VL1: i32 = 0;
/// `FDK_TOOLS_LIB_VL2`
pub const FDK_TOOLS_LIB_VL2: i32 = 0;
/// `FDK_TOOLS_LIB_TITLE`
pub const FDK_TOOLS_LIB_TITLE: &str = "FDK Tools";
