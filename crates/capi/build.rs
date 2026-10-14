// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from src/CMakeLists.txt (the name of the shared library).

//! The shared library gets the name of the reference build (`add_library(mpeghdec)` without a
//! version): install name `@rpath/libmpeghdec.dylib` on Apple targets, soname `libmpeghdec.so`
//! on other ELF targets. Cargo names the file `libmpeghdec_capi.*`; installing it as
//! `libmpeghdec.*` is the installer's task (`scripts/capi-test.sh` does it).

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    let vendor = std::env::var("CARGO_CFG_TARGET_VENDOR").unwrap_or_default();
    let family = std::env::var("CARGO_CFG_TARGET_FAMILY").unwrap_or_default();
    let os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if vendor == "apple" {
        println!("cargo:rustc-cdylib-link-arg=-Wl,-install_name,@rpath/libmpeghdec.dylib");
    } else if family == "unix" && os != "windows" {
        println!("cargo:rustc-cdylib-link-arg=-Wl,-soname,libmpeghdec.so");
    }
}
