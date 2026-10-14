// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Selects the code paths of the reference implementation that the port reproduces. The
// reference computes some primitives differently depending on the architecture it is built for
// (e.g. `fMult()` is `(a*b)>>31` on AArch64 and `((a*b)>>32)<<1` on x86), so its output differs
// between architectures in the least significant bits. The port follows the x86-64 build on
// x86-64 targets, the 32 bit x86 build (i686: no SSE, x87 floating point) on 32 bit x86 targets
// and the AArch64 build (Apple arm64, NEON disabled) on all other targets.
//
// MPEGHDEC_ARCH=x86 (x86-64), MPEGHDEC_ARCH=x86_32 or MPEGHDEC_ARCH=arm overrides the choice,
// e.g. to test the AArch64 variant on an x86-64 host against the output of the AArch64
// reference.

fn main() {
    println!("cargo::rustc-check-cfg=cfg(mpeghdec_x86)");
    println!("cargo::rustc-check-cfg=cfg(mpeghdec_x86_32)");
    println!("cargo::rerun-if-env-changed=MPEGHDEC_ARCH");
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let arch = match std::env::var("MPEGHDEC_ARCH").as_deref() {
        Ok(a @ ("x86" | "x86_32" | "arm")) => a.to_string(),
        Ok(other) => {
            panic!("MPEGHDEC_ARCH must be \"x86\", \"x86_32\" or \"arm\", not {other:?}")
        }
        Err(_) => match target_arch.as_str() {
            "x86_64" => "x86".to_string(),
            "x86" => "x86_32".to_string(),
            _ => "arm".to_string(),
        },
    };
    if arch != "arm" {
        println!("cargo::rustc-cfg=mpeghdec_x86");
    }
    if arch == "x86_32" {
        println!("cargo::rustc-cfg=mpeghdec_x86_32");
    }
}
