// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Compiles the crash/hang guard of the reference calls (src/guard.c). The reference itself and
// its shims come from mpeghdec-cref (crates/cref/build.rs).
fn main() {
    println!("cargo::rerun-if-changed=src/guard.c");
    cc::Build::new().file("src/guard.c").opt_level(2).warnings(true).compile("fzguard");
}
