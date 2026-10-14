// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Fixed-point DSP and bitstream primitives: port of `libFDK` and `libSYS`.
//!
//! The arithmetic reproduces the code paths of the reference implementation bit-exactly: those
//! of the x86 build on x86-64 and 32 bit x86 targets, those of the AArch64 build (Apple arm64,
//! NEON disabled) on all other targets (see [`ARCH_X86`] and [`ARCH_X86_32`]).

/* the derived `StateDigest` implementations refer to `::mpeghdec_fdk` */
extern crate self as mpeghdec_fdk;

/// `true` if the port reproduces the x86 build of the reference implementation (`__x86__`),
/// `false` for the AArch64 build (`__arm__`, `__aarch64__`, Apple without NEON).
///
/// The builds differ in the rounding of some primitives (e.g. `fMult()`, `cplxMult()`,
/// `schur_div()`), so their outputs differ in the least significant bits. The default is the
/// build of the target architecture (x86 for `x86` and `x86_64`, AArch64 for all others); the
/// environment variable `MPEGHDEC_ARCH=x86|x86_32|arm` at build time overrides it.
pub const ARCH_X86: bool = cfg!(mpeghdec_x86);

/// `true` if the port reproduces the 32 bit x86 build of the reference implementation (i686,
/// clang's default target without SSE; implies [`ARCH_X86`]).
///
/// It differs from the x86-64 build in the 64 bit multiplications (`fixmuldiv2_QQ()` and
/// `fixmuldiv2_QD()` without a 128 bit product), in the floating point computations, which use
/// the x87 unit with extended precision (see [`x87`]), and in loops that the x86-64 build
/// vectorizes. The default for `x86` targets; `MPEGHDEC_ARCH=x86_32` selects it elsewhere.
pub const ARCH_X86_32: bool = cfg!(mpeghdec_x86_32);

pub mod common_fix;
pub mod fixpoint_math;
pub mod scale;
pub mod tools_rom;

pub use common_fix::*;
pub mod audio;
pub mod bitbuffer;
pub mod bitstream;

pub mod cicp2geometry;
pub mod core;
pub mod crc;
pub mod dct;
pub mod deviation;
pub mod digest;
pub mod fft;
pub mod fft_rad2;
pub mod lpc;
pub mod matrix_calloc;
pub mod mdct;
pub mod rfft;
pub mod scramble;
pub mod simd;
pub mod stftfilterbank;
pub mod td_upsampler;
pub mod thread_pool;
pub mod trig_fcts;
pub mod x87;
