// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential fuzzing of the Rust port against the C++ reference mpeghdec (fuzz/README.md).
//!
//! Every target decodes its input into a call program, runs the program on the reference
//! (`mpeghdec_cref`: the reference library and its shims, every call guarded by src/guard.c) and
//! on the port's public API, records a trace of every call and its results, and compares the
//! traces:
//!
//! - equal traces: nothing to report;
//! - the reference crashed or ran out of its CPU budget: the traces are compared up to the
//!   aborted call;
//! - a difference: the reference is run again with other stack and heap contents; if its own
//!   trace changes, the input depends on uninitialized reference memory and is skipped;
//!   otherwise the target panics with a report, and libFuzzer stores the input.
//!
//! The port runs unguarded: a panic or a hang of the port is a finding as well.

pub mod dec;
pub mod guard;

use guard::Abort;
use std::fmt::{self, Write as _};

/// Reads the fuzz input; reads past the end give 0.
pub struct Reader<'a> {
    data: &'a [u8],
    pub pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Reader { data, pos: 0 }
    }

    pub fn u8(&mut self) -> u8 {
        let b = self.data.get(self.pos).copied().unwrap_or(0);
        self.pos += 1;
        b
    }

    pub fn u16(&mut self) -> u16 {
        u16::from_le_bytes([self.u8(), self.u8()])
    }

    /// The next `n` bytes (fewer at the end of the input).
    pub fn bytes(&mut self, n: usize) -> &'a [u8] {
        let start = self.pos.min(self.data.len());
        let end = (start + n).min(self.data.len());
        self.pos = start + n;
        &self.data[start..end]
    }

    pub fn rest(&mut self) -> &'a [u8] {
        let start = self.pos.min(self.data.len());
        self.pos = self.data.len();
        &self.data[start..]
    }

    pub fn at_end(&self) -> bool {
        self.pos >= self.data.len()
    }
}

/// 64-bit hash of a byte string (not cryptographic; it only has to separate outputs).
pub fn hash(data: &[u8]) -> u64 {
    let mut h = 0x9E37_79B9_7F4A_7C15u64 ^ data.len() as u64;
    let (chunks, rest) = data.as_chunks::<8>();
    for c in chunks {
        let w = u64::from_le_bytes(*c);
        h = (h ^ w).wrapping_mul(0xFF51_AFD7_ED55_8CCD).rotate_left(31);
    }
    for (i, &b) in rest.iter().enumerate() {
        h = (h ^ ((b as u64) << (8 * i))).wrapping_mul(0xC4CE_B9FE_1A85_EC53);
    }
    h ^= h >> 33;
    h = h.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
    h ^ (h >> 33)
}

/// A byte string compared by hash and length; the bytes are kept only in verbose runs (for the
/// report of a difference).
#[derive(Clone)]
pub struct Blob {
    pub hash: u64,
    pub len: usize,
    pub bytes: Option<Vec<u8>>,
}

impl Blob {
    pub fn new(data: &[u8], verbose: bool) -> Blob {
        Blob { hash: hash(data), len: data.len(), bytes: verbose.then(|| data.to_vec()) }
    }

    /// The samples as little-endian bytes (the reference platforms are little endian).
    pub fn from_i32(data: &[i32], verbose: bool) -> Blob {
        // SAFETY: i32 has no padding; the byte view covers the same memory.
        let b = unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 4) };
        Blob::new(b, verbose)
    }

    pub fn empty() -> Blob {
        Blob::new(&[], false)
    }
}

impl PartialEq for Blob {
    fn eq(&self, o: &Blob) -> bool {
        self.hash == o.hash && self.len == o.len
    }
}

impl fmt::Debug for Blob {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "<{} bytes, hash {:016x}>", self.len, self.hash)
    }
}

/// With `MPEGHDEC_FUZZ_ECHO=1`, every call is printed as it is recorded (shows the calls up to a
/// port panic).
pub fn echo<E: fmt::Debug>(i: usize, e: &E) {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *ON.get_or_init(|| std::env::var_os("MPEGHDEC_FUZZ_ECHO").is_some_and(|v| v == "1")) {
        eprintln!("    #{i} {e:?}");
    }
}

/// The calls of one side and how the run ended.
pub struct Trace<E, F> {
    pub events: Vec<E>,
    /// Output frames compared as a sequence, not call by call (the frame pipelining of the
    /// port returns them later than the reference; empty when the frames are part of `events`).
    pub frames: Vec<F>,
    /// The number of frames before the last process, flush or flushAndGet call (frame
    /// pipelining: the port returns the frames of that call with the next one).
    pub frames_floor: usize,
    /// The reference aborted the call after `events.len()` events.
    pub abort: Option<Abort>,
}

impl<E, F> Default for Trace<E, F> {
    fn default() -> Self {
        Trace { events: Vec::new(), frames: Vec::new(), frames_floor: 0, abort: None }
    }
}

/// Result of one input.
#[derive(Debug)]
pub enum Outcome {
    /// Both sides gave the same trace.
    Match { events: usize },
    /// The reference aborted; the traces matched up to the aborted call.
    RefAbort { at: usize, why: Abort },
    /// The traces differ, but so do two runs of the reference with different memory contents:
    /// the reference reads uninitialized memory there.
    RefNondeterministic { at: usize },
    /// A difference (the report).
    Mismatch(String),
}

/// Where two traces first differ.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Diff {
    Event(usize),
    Frame(usize),
}

/// The first difference, if any. If the reference aborted, only the events before the aborted
/// call and the frames it returned before are compared. The port may lack the frames that the
/// reference returned after its last process, flush or flushAndGet call (frame pipelining returns
/// them with the next one, and the run ended).
pub fn first_difference<E: PartialEq, F: PartialEq>(
    r: &Trace<E, F>,
    p: &Trace<E, F>,
) -> Option<Diff> {
    let n = if r.abort.is_some() { r.events.len() } else { r.events.len().max(p.events.len()) };
    if let Some(i) = (0..n).find(|&i| r.events.get(i) != p.events.get(i)) {
        return Some(Diff::Event(i));
    }
    let lagging = p.frames.len() < r.frames.len() && p.frames.len() >= r.frames_floor;
    let n = if r.abort.is_some() || lagging {
        r.frames.len().min(p.frames.len())
    } else {
        r.frames.len().max(p.frames.len())
    };
    (0..n).find(|&i| r.frames.get(i) != p.frames.get(i)).map(Diff::Frame)
}

/// Whether two reference traces are the same run.
pub fn same_run<E: PartialEq, F: PartialEq>(a: &Trace<E, F>, b: &Trace<E, F>) -> bool {
    a.abort == b.abort && a.events == b.events && a.frames == b.frames
}

/// Memory patterns of the repeated reference runs (the first run uses 0).
pub const RECHECK_PATTERNS: [i32; 3] = [0xA5, 0x5A, 0xFF];

/// The comparison of one input: `run_port(verbose)` (always before `run_ref`, which may depend on
/// it), `run_ref(pattern, verbose)`; `detail(ref, port)` adds detail about two differing events or
/// frames to a report.
pub fn compare<E: PartialEq + fmt::Debug, F: PartialEq + fmt::Debug>(
    target: &str,
    describe: &str,
    mut run_ref: impl FnMut(i32, bool) -> Trace<E, F>,
    mut run_port: impl FnMut(bool) -> Trace<E, F>,
    detail_ev: impl Fn(&E, &E) -> String,
    detail_fr: impl Fn(&F, &F) -> String,
) -> Outcome {
    let p = run_port(false);
    let r = run_ref(0, false);
    let Some(d) = first_difference(&r, &p) else {
        return match r.abort {
            Some(why) => Outcome::RefAbort { at: r.events.len(), why },
            None => Outcome::Match { events: r.events.len() },
        };
    };
    let at = match d {
        Diff::Event(i) | Diff::Frame(i) => i,
    };
    for pat in RECHECK_PATTERNS {
        if !same_run(&r, &run_ref(pat, false)) {
            return Outcome::RefNondeterministic { at };
        }
    }
    // Verbose runs for the report (they keep the output bytes).
    let pv = run_port(true);
    let rv = run_ref(0, true);
    let d = first_difference(&rv, &pv).unwrap_or(d);
    let mut s = String::new();
    writeln!(s, "mpeghdec-rs differential fuzzing: DIFFERENCE in {target}").unwrap();
    writeln!(s, "input: {describe}").unwrap();
    let ev = |t: &Trace<E, F>, k: usize| match t.events.get(k) {
        Some(e) => format!("{e:?}"),
        None => match t.abort {
            Some(a) if k == t.events.len() => format!("(reference call aborted: {a})"),
            _ => "(no call: the run ended)".to_string(),
        },
    };
    let fr = |t: &Trace<E, F>, k: usize| match t.frames.get(k) {
        Some(f) => format!("{f:?}"),
        None => "(no frame)".to_string(),
    };
    match d {
        Diff::Event(i) => {
            writeln!(s, "first differing call: #{i}").unwrap();
            writeln!(s, "  reference: {}", ev(&rv, i)).unwrap();
            writeln!(s, "  port:      {}", ev(&pv, i)).unwrap();
            if let (Some(a), Some(b)) = (rv.events.get(i), pv.events.get(i)) {
                let x = detail_ev(a, b);
                if !x.is_empty() {
                    writeln!(s, "  {x}").unwrap();
                }
            }
            let from = i.saturating_sub(10);
            if from < i {
                writeln!(s, "preceding calls (equal on both sides):").unwrap();
                for k in from..i {
                    writeln!(s, "  #{k} {}", ev(&rv, k)).unwrap();
                }
            }
        }
        Diff::Frame(i) => {
            writeln!(s, "first differing output frame: #{i} (frames compared as a sequence)")
                .unwrap();
            writeln!(s, "  reference: {}", fr(&rv, i)).unwrap();
            writeln!(s, "  port:      {}", fr(&pv, i)).unwrap();
            if let (Some(a), Some(b)) = (rv.frames.get(i), pv.frames.get(i)) {
                let x = detail_fr(a, b);
                if !x.is_empty() {
                    writeln!(s, "  {x}").unwrap();
                }
            }
        }
    }
    writeln!(
        s,
        "calls: reference {} ({}), port {}; frames: reference {}, port {}",
        rv.events.len(),
        rv.abort.map_or("completed".to_string(), |a| format!("aborted: {a}")),
        pv.events.len(),
        rv.frames.len(),
        pv.frames.len()
    )
    .unwrap();
    Outcome::Mismatch(s)
}

/// The first differing element of two byte strings of a verbose run, as text (`elem`: 1 for
/// bytes, 4 for 32-bit samples).
pub fn diff_bytes(what: &str, a: &Blob, b: &Blob, elem: usize) -> String {
    let (Some(x), Some(y)) = (&a.bytes, &b.bytes) else { return String::new() };
    let n = x.len().min(y.len()) / elem;
    let get = |v: &[u8], k: usize| -> i64 {
        match elem {
            4 => i32::from_le_bytes(v[4 * k..4 * k + 4].try_into().unwrap()) as i64,
            _ => v[k] as i64,
        }
    };
    let diffs: Vec<usize> = (0..n).filter(|&k| get(x, k) != get(y, k)).collect();
    match diffs.first() {
        Some(&k) => format!(
            "{what}: {} of {n} elements differ; first at {k}: reference {} port {}",
            diffs.len(),
            get(x, k),
            get(y, k)
        ),
        None if x.len() != y.len() => format!("{what}: lengths {} and {}", x.len(), y.len()),
        None => String::new(),
    }
}

/// The input being run (for the report of a panic).
static CURRENT: std::sync::Mutex<(&str, Vec<u8>)> = std::sync::Mutex::new(("", Vec::new()));

/// The directory of the panic reports: `MPEGHDEC_FUZZ_REPORT_DIR`, default
/// `fuzz/artifacts/reports`.
fn report_dir() -> std::path::PathBuf {
    std::env::var_os("MPEGHDEC_FUZZ_REPORT_DIR")
        .map(Into::into)
        .unwrap_or_else(|| concat!(env!("CARGO_MANIFEST_DIR"), "/artifacts/reports").into())
}

/// Records the input of the next run. On the first call, installs a panic hook that writes the
/// panic message (a DIFFERENCE report or a port panic, also in a worker thread of the port) and
/// the input to the report directory, as `<target>-<hash>.txt` and `<target>-<hash>.input`: in
/// libFuzzer's fork mode the output of the fuzzing processes is not kept, and a finding that
/// depends on timing may not replay.
pub fn begin_input(target: &'static str, data: &[u8]) {
    static HOOK: std::sync::Once = std::sync::Once::new();
    HOOK.call_once(|| {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            if let Ok(cur) = CURRENT.try_lock() {
                let dir = report_dir();
                let _ = std::fs::create_dir_all(&dir);
                let name = format!("{}-{:016x}", cur.0, hash(&cur.1));
                let msg = info
                    .payload()
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| info.payload().downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_default();
                let thread = std::thread::current().name().unwrap_or("?").to_string();
                let loc = info.location().map_or(String::new(), |l| l.to_string());
                let text = format!("panic in thread {thread} at {loc}:\n{msg}\n");
                let _ = std::fs::write(dir.join(format!("{name}.txt")), text);
                let _ = std::fs::write(dir.join(format!("{name}.input")), &cur.1);
            }
            prev(info)
        }));
    });
    if let Ok(mut cur) = CURRENT.lock() {
        cur.0 = target;
        cur.1.clear();
        cur.1.extend_from_slice(data);
    }
}

/// What the fuzz targets do with an outcome: report a difference by panicking (libFuzzer stores
/// the input); count the skipped inputs (printed every 2^k skips).
pub fn handle(target: &str, o: Outcome) {
    use std::sync::atomic::{AtomicU64, Ordering};
    static ABORTS: AtomicU64 = AtomicU64::new(0);
    static NONDET: AtomicU64 = AtomicU64::new(0);
    match o {
        Outcome::Match { .. } => {}
        Outcome::RefAbort { why, .. } => {
            let n = ABORTS.fetch_add(1, Ordering::Relaxed) + 1;
            if n.is_power_of_two() {
                eprintln!("[{target}] {n} inputs with an aborted reference call (latest: {why})");
            }
        }
        Outcome::RefNondeterministic { .. } => {
            let n = NONDET.fetch_add(1, Ordering::Relaxed) + 1;
            if n.is_power_of_two() {
                eprintln!(
                    "[{target}] {n} inputs skipped: the reference reads uninitialized memory"
                );
            }
        }
        Outcome::Mismatch(report) => panic!("{report}"),
    }
}
