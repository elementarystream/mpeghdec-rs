// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Replays fuzz inputs outside libFuzzer and reports what each one does.
//!
//! ```text
//! replay [--trace] [--hang-check] TARGET FILE|DIR...
//! ```
//!
//! TARGET is a target name (`decode_mhas`, `decode_mha1`, `ui_decode`), or `auto` to take it
//! from the path (`fuzz/artifacts/<target>/...`, `fuzz/corpus/<target>/...`). `--trace` also
//! prints both call traces. `--hang-check` (implied for `timeout-*` files) first runs each side
//! alone in a child process with a wall-clock limit (`MPEGHDEC_FUZZ_HANG_SECONDS`, default 120)
//! and reports which side hangs. Exit code 1 if any input shows a difference, a port panic or a
//! port hang.

use mpeghdec_fuzz::dec::{self, Program, Target};
use mpeghdec_fuzz::Outcome;
use std::path::{Path, PathBuf};

fn infer(path: &Path) -> Option<Target> {
    /* fuzz/artifacts/<target>[-<tag>]/... */
    path.ancestors().filter_map(|a| a.file_name()).find_map(|n| {
        let n = n.to_string_lossy();
        Target::from_name(&n).or_else(|| Target::from_name(n.split('-').next()?))
    })
}

fn print_trace(side: &str, t: &mpeghdec_fuzz::Trace<dec::Ev, dec::Frame>) {
    println!(
        "  {side} trace ({} calls, {} frames{}):",
        t.events.len(),
        t.frames.len(),
        t.abort.map_or(String::new(), |a| format!(", then aborted: {a}"))
    );
    for (i, e) in t.events.iter().enumerate() {
        println!("    #{i} {e:?}");
    }
    for (i, f) in t.frames.iter().enumerate() {
        println!("    frame {i} {f:?}");
    }
}

/// Runs one side alone (`--side ref|port TARGET FILE`, used by the hang check).
fn run_side(side: &str, t: Target, data: &[u8]) {
    let p = Program::parse(t, data);
    if side == "ref" {
        drop(dec::run_ref(&p, 0, false, None));
    } else {
        drop(dec::run_port(&p, false));
    }
}

/// Whether `side` finishes within the limit, run in a child process.
fn finishes(side: &str, t: &str, f: &Path) -> Option<bool> {
    let limit: u64 = std::env::var("MPEGHDEC_FUZZ_HANG_SECONDS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(120);
    let mut child = std::process::Command::new(std::env::current_exe().ok()?)
        .args(["--side", side, t])
        .arg(f)
        .env("MPEGHDEC_FUZZ_REF_BUDGET_MS", "0")
        .stdout(std::process::Stdio::null())
        .spawn()
        .ok()?;
    let start = std::time::Instant::now();
    loop {
        if let Ok(Some(st)) = child.try_wait() {
            return Some(st.success());
        }
        if start.elapsed().as_secs() >= limit {
            let _ = child.kill();
            let _ = child.wait();
            return Some(false);
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}

fn files(args: &[String]) -> Vec<PathBuf> {
    let mut v = Vec::new();
    for a in args {
        let p = PathBuf::from(a);
        if p.is_dir() {
            let mut d: Vec<PathBuf> = std::fs::read_dir(&p)
                .map(|r| r.flatten().map(|e| e.path()).filter(|p| p.is_file()).collect())
                .unwrap_or_default();
            d.sort();
            v.extend(d.into_iter().filter(|p| {
                let n = p.file_name().unwrap().to_string_lossy();
                !n.ends_with(".log") && !n.starts_with('.')
            }));
        } else {
            v.push(p);
        }
    }
    v
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--side") {
        let t = Target::from_name(&args[2]).expect("target");
        let data = std::fs::read(&args[3]).expect("read input");
        run_side(&args[1], t, &data);
        return;
    }
    let mut trace = false;
    let mut hang_check = false;
    args.retain(|a| match a.as_str() {
        "--trace" => {
            trace = true;
            false
        }
        "--hang-check" => {
            hang_check = true;
            false
        }
        _ => true,
    });
    if args.len() < 2 {
        eprintln!("usage: replay [--trace] [--hang-check] TARGET|auto FILE|DIR...");
        std::process::exit(2);
    }
    let fixed = (args[0] != "auto").then(|| Target::from_name(&args[0]).expect("unknown target"));
    let mut bad = 0;
    let mut counts = [0usize; 4];
    // A port panic is caught per input (the default hook prints its message and location).
    for f in files(&args[1..]) {
        let Some(t) = fixed.or_else(|| infer(&f)) else {
            println!("{}: unknown target (pass it explicitly)", f.display());
            continue;
        };
        let data = std::fs::read(&f).expect("read input");
        let name = f.display().to_string();
        let is_timeout = f.file_name().is_some_and(|n| n.to_string_lossy().starts_with("timeout-"));
        if hang_check || is_timeout {
            let r = finishes("ref", t.name(), &f);
            let p = finishes("port", t.name(), &f);
            println!("{name}: hang check: reference {r:?}, port {p:?} (Some(true): finished)");
            if p != Some(true) {
                bad += 1;
                println!("{name}: PORT HANG or crash");
                continue;
            }
        }
        let r = std::panic::catch_unwind(|| dec::check(t, &data));
        match r {
            Ok(Outcome::Match { events }) => {
                counts[0] += 1;
                println!("{name}: match ({events} calls)");
            }
            Ok(Outcome::RefAbort { at, why }) => {
                counts[1] += 1;
                println!("{name}: reference aborted at call {at}: {why} (matched up to it)");
            }
            Ok(Outcome::RefNondeterministic { at }) => {
                counts[2] += 1;
                println!("{name}: reference reads uninitialized memory (first difference at {at})");
            }
            Ok(Outcome::Mismatch(report)) => {
                counts[3] += 1;
                bad += 1;
                println!("{name}:\n{report}");
            }
            Err(e) => {
                bad += 1;
                let msg = e
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_default();
                println!("{name}: PORT PANIC: {msg}");
            }
        }
        if trace {
            let p = Program::parse(t, &data);
            let (port, hit) = dec::run_port(&p, false);
            print_trace("reference", &dec::run_ref(&p, 0, false, hit));
            print_trace("port", &port);
        }
    }
    println!(
        "summary: {} match, {} reference aborts, {} uninitialized reference reads, {} differences, {} bad",
        counts[0], counts[1], counts[2], counts[3], bad
    );
    std::process::exit(if bad > 0 { 1 } else { 0 });
}
