// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Builds the Fraunhofer C++ reference implementation together with the C shims in `shim/` into a
// static library, so that tests can compare the Rust port against the original code.
//
// The C++ sources are taken from `$MPEGHDEC_C_DIR` (default: `reference/src` of
// `$MPEGHDEC_WORK_DIR` or of `<workspace>/work`). If they are missing, the `cref_missing` cfg is
// set and all differential tests are skipped.

use std::env;
use std::path::{Path, PathBuf};

fn collect(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    let mut entries: Vec<_> = entries.flatten().map(|e| e.path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            // Architecture specific files are #included by the generic sources.
            if p.file_name().is_some_and(|n| n == "arm" || n == "x86") {
                continue;
            }
            collect(&p, out);
        } else if p.extension().is_some_and(|e| e == "cpp") {
            out.push(p);
        }
    }
}

fn main() {
    println!("cargo::rustc-check-cfg=cfg(cref_missing)");
    println!("cargo::rerun-if-env-changed=MPEGHDEC_C_DIR");
    println!("cargo::rerun-if-env-changed=MPEGHDEC_WORK_DIR");
    println!("cargo::rerun-if-env-changed=CXX");
    println!("cargo::rerun-if-changed=shim");

    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let c_dir = env::var_os("MPEGHDEC_C_DIR").map(PathBuf::from).unwrap_or_else(|| {
        env::var_os("MPEGHDEC_WORK_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| manifest.join("../../work"))
            .join("reference/src")
    });
    let src = c_dir.join("src");
    if !src.join("libFDK").is_dir() {
        println!(
            "cargo::warning=C++ reference sources not found at {}; differential tests are skipped",
            c_dir.display()
        );
        println!("cargo::rustc-cfg=cref_missing");
        return;
    }

    let mut files = Vec::new();
    collect(&src, &mut files);
    collect(&manifest.join("shim"), &mut files);

    let mut build = cc::Build::new();
    // The reference decoder is built with clang (scripts/setup-reference.sh; Apple clang on macOS).
    // Undefined shifts behave differently in scalar and auto-vectorized code, so the results of
    // some functions depend on the compiler: use clang unless CXX selects another compiler.
    let target = env::var("TARGET").unwrap_or_default();
    let cxx_set =
        ["CXX".to_string(), format!("CXX_{}", target.replace('-', "_")), format!("CXX_{target}")]
            .iter()
            .any(|v| env::var_os(v).is_some());
    if !cxx_set && !target.contains("apple") {
        build.compiler("clang++");
    }
    build
        .cpp(true)
        .std("c++11")
        .opt_level(3)
        .debug(false)
        .warnings(false)
        .define("NDEBUG", None)
        .define("MPEGHDEC_INTERNAL", "1")
        .define("MPEGHDEC_STATIC", "1")
        .include(c_dir.join("include"))
        .include(c_dir.join("include/sys"));
    let mut libs: Vec<_> = std::fs::read_dir(&src).unwrap().flatten().map(|e| e.path()).collect();
    libs.sort();
    for lib in libs {
        if lib.join("include").is_dir() {
            build.include(lib.join("include"));
        }
        if lib.join("src").is_dir() {
            build.include(lib.join("src"));
        }
    }
    for f in &files {
        build.file(f);
    }
    build.compile("mpeghdec_cref");
}
