// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: state digest of the chunked decoding.

//! Digest of the complete decoder state (not in C).
//!
//! The decoder is deterministic: its future output depends only on its state and its future
//! input. The chunked decoding (`mpeghdec::chunked`) decodes ranges of a file in parallel, each
//! range with its own decoder that starts some access units earlier (warm-up); a range boundary is
//! accepted only if the state of the decoder of the next range equals the state of the decoder
//! that decoded up to the boundary, which is checked by comparing their digests.
//!
//! [`StateDigest`] feeds a value to a [`Digester`]. It is implemented for the primitive types,
//! arrays, slices, `Vec`, `Box`, `Option`, `VecDeque`, tuples and references (by address: a
//! reference in the decoder state points to a constant table), and derived for the decoder
//! structures with `#[derive(StateDigest)]` (crate `mpeghdec-digest-derive`), which feeds every
//! field. Fields that cannot influence the future output are excluded with `#[digest(skip)]` and
//! a comment explaining why (scratch memory written before it is read, reported counters, thread
//! handles). Types holding raw pointers implement the trait by hand.
//!
//! The digest is a 128 bit hash (two independent chains of the `wyhash` multiply-fold mixing over
//! 16 byte blocks), not a cryptographic hash: two different states have the same digest with a
//! probability of about 2^-128 for unrelated inputs (collisions would have to be constructed).
//!
//! [`Digester::traced`] additionally records the digest of every field (up to a depth), so that the
//! fields in which two decoder states differ can be listed ([`diff_traces`]).

use std::collections::VecDeque;

pub use mpeghdec_digest_derive::StateDigest;

/// Secrets of the mixing function (the `wyhash` default secret).
const S: [u64; 4] =
    [0xa076_1d64_78bd_642f, 0xe703_7ed1_a0b4_28db, 0x8ebc_6af0_9c88_c6e3, 0x5899_65cc_7537_4cc3];

#[inline(always)]
fn mum(a: u64, b: u64) -> u64 {
    let r = (a as u128).wrapping_mul(b as u128);
    (r as u64) ^ ((r >> 64) as u64)
}

/// Hash chain state.
#[derive(Clone, Copy)]
struct Lanes {
    h0: u64,
    h1: u64,
    pending: [u8; 16],
    npending: usize,
    total: u64,
}

impl Lanes {
    const fn new() -> Lanes {
        Lanes { h0: S[0], h1: S[1], pending: [0; 16], npending: 0, total: 0 }
    }

    #[inline(always)]
    fn block(&mut self, x0: u64, x1: u64) {
        self.h0 = mum(x0 ^ S[1], x1 ^ self.h0);
        self.h1 = mum(x0 ^ S[2], x1 ^ self.h1 ^ S[3]);
    }

    #[inline]
    fn write(&mut self, mut bytes: &[u8]) {
        self.total = self.total.wrapping_add(bytes.len() as u64);
        if self.npending > 0 {
            let n = (16 - self.npending).min(bytes.len());
            self.pending[self.npending..self.npending + n].copy_from_slice(&bytes[..n]);
            self.npending += n;
            bytes = &bytes[n..];
            if self.npending < 16 {
                return;
            }
            let p = self.pending;
            self.block(
                u64::from_le_bytes(p[..8].try_into().unwrap()),
                u64::from_le_bytes(p[8..].try_into().unwrap()),
            );
            self.npending = 0;
        }
        let mut chunks = bytes.chunks_exact(16);
        for c in &mut chunks {
            self.block(
                u64::from_le_bytes(c[..8].try_into().unwrap()),
                u64::from_le_bytes(c[8..].try_into().unwrap()),
            );
        }
        let r = chunks.remainder();
        self.pending[..r.len()].copy_from_slice(r);
        self.npending = r.len();
    }

    fn finish(mut self) -> [u64; 2] {
        let mut p = [0u8; 16];
        p[..self.npending].copy_from_slice(&self.pending[..self.npending]);
        self.block(
            u64::from_le_bytes(p[..8].try_into().unwrap()),
            u64::from_le_bytes(p[8..].try_into().unwrap()),
        );
        self.block(self.total, self.npending as u64);
        [mum(self.h0 ^ S[0], self.h1 ^ S[3]), mum(self.h1 ^ S[1], self.h0 ^ S[2])]
    }
}

/// The digest of a decoder state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct Digest(pub [u64; 2]);

impl core::fmt::Display for Digest {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:016x}{:016x}", self.0[0], self.0[1])
    }
}

struct Trace {
    max_depth: usize,
    path: String,
    /// Parent states and path lengths of the entered fields.
    stack: Vec<(Lanes, usize)>,
    entries: Vec<(String, Digest)>,
}

/// Collects the digest of a state (see the [module documentation](self)).
pub struct Digester {
    lanes: Lanes,
    trace: Option<Box<Trace>>,
}

impl Default for Digester {
    fn default() -> Self {
        Digester::new()
    }
}

impl Digester {
    /// A digester computing the digest only.
    pub const fn new() -> Digester {
        Digester { lanes: Lanes::new(), trace: None }
    }

    /// A digester that also records the digest of every field up to `max_depth` levels (see
    /// [`Digester::finish_traced`]). The digest differs from the one of [`Digester::new`].
    pub fn traced(max_depth: usize) -> Digester {
        Digester {
            lanes: Lanes::new(),
            trace: Some(Box::new(Trace {
                max_depth,
                path: String::new(),
                stack: Vec::new(),
                entries: Vec::new(),
            })),
        }
    }

    /// Number of bytes fed so far (at the current level of a traced digester).
    pub fn bytes_fed(&self) -> u64 {
        self.lanes.total
    }

    /// The digest of everything fed so far.
    pub fn finish(self) -> Digest {
        Digest(self.lanes.finish())
    }

    /// The digest and the recorded digests of the fields: `(path, digest)` in depth-first
    /// order, each field after its children.
    pub fn finish_traced(self) -> (Digest, Vec<(String, Digest)>) {
        let entries = self.trace.map(|t| t.entries).unwrap_or_default();
        (Digest(self.lanes.finish()), entries)
    }

    /// Feeds raw bytes.
    #[inline]
    pub fn bytes(&mut self, b: &[u8]) {
        self.lanes.write(b);
    }

    /// Feeds a `u32` (e.g. an enum variant index).
    #[inline]
    pub fn u32(&mut self, v: u32) {
        self.lanes.write(&v.to_le_bytes());
    }

    /// Feeds a `u64` (e.g. a length).
    #[inline]
    pub fn u64(&mut self, v: u64) {
        self.lanes.write(&v.to_le_bytes());
    }

    #[inline]
    fn enter(&mut self, name: &dyn core::fmt::Display) -> bool {
        let Some(t) = self.trace.as_deref_mut() else { return false };
        if t.stack.len() >= t.max_depth {
            return false;
        }
        let len = t.path.len();
        if len > 0 {
            t.path.push('.');
        }
        use core::fmt::Write;
        let _ = write!(t.path, "{name}");
        t.stack.push((self.lanes, len));
        self.lanes = Lanes::new();
        true
    }

    #[inline]
    fn leave(&mut self) {
        let t = self.trace.as_deref_mut().expect("trace");
        let (parent, len) = t.stack.pop().expect("entered field");
        let d = Digest(core::mem::replace(&mut self.lanes, parent).finish());
        t.entries.push((t.path.clone(), d));
        t.path.truncate(len);
        self.lanes.write(&d.0[0].to_le_bytes());
        self.lanes.write(&d.0[1].to_le_bytes());
    }

    /// Feeds the field `name` of a structure.
    #[inline]
    pub fn field<T: StateDigest + ?Sized>(&mut self, name: &'static str, v: &T) {
        if self.trace.is_none() {
            v.digest(self);
        } else {
            let e = self.enter(&name);
            v.digest(self);
            if e {
                self.leave();
            }
        }
    }

    /// Feeds the field `name` with the function `f`.
    #[inline]
    pub fn field_with(&mut self, name: &'static str, f: impl FnOnce(&mut Digester)) {
        if self.trace.is_none() {
            f(self);
        } else {
            let e = self.enter(&name);
            f(self);
            if e {
                self.leave();
            }
        }
    }

    /// Feeds the element `index` of a sequence with the function `f`.
    #[inline]
    pub fn element_with(&mut self, index: usize, f: impl FnOnce(&mut Digester)) {
        if self.trace.is_none() {
            f(self);
        } else {
            let e = self.enter(&format_args!("[{index}]"));
            f(self);
            if e {
                self.leave();
            }
        }
    }

    /// Feeds the element `index` of a sequence.
    #[inline]
    pub fn element<T: StateDigest + ?Sized>(&mut self, index: usize, v: &T) {
        if self.trace.is_none() {
            v.digest(self);
        } else {
            let e = self.enter(&format_args!("[{index}]"));
            v.digest(self);
            if e {
                self.leave();
            }
        }
    }
}

thread_local! {
    static HAZARDS: core::cell::Cell<u64> = const { core::cell::Cell::new(0) };
}

/// Signals a hazard of the state digest on the current thread: state that is not part of the
/// digest (or only up to a symmetry, like the position of the elements in a ring buffer)
/// influences the decoding. Two decoders with the same digest decode identically only up to the
/// first hazard.
///
/// The hazards are: the time domain noise of the concealment (its random seed is not part of the
/// digest), the out-of-bounds read of the DRC gain interpolation for a node buffer without nodes
/// (reads memory at a physical position of the node ring buffer) and `deque_pop_back()` with
/// `last == 0` (computes the physical position from `size`).
#[inline]
pub fn hazard() {
    HAZARDS.with(|h| h.set(h.get() + 1));
}

/// The number of hazards signalled on the current thread so far (see [`hazard`]).
pub fn hazard_count() -> u64 {
    HAZARDS.with(|h| h.get())
}

/// The paths of the fields whose digests differ in two traces of [`Digester::finish_traced`],
/// only the innermost ones (a differing field is not listed if one of its fields differs), with
/// the fields that exist in only one of the traces.
pub fn diff_traces(a: &[(String, Digest)], b: &[(String, Digest)]) -> Vec<String> {
    let mb: std::collections::HashMap<&str, Digest> =
        b.iter().map(|(p, d)| (p.as_str(), *d)).collect();
    let ma: std::collections::HashSet<&str> = a.iter().map(|(p, _)| p.as_str()).collect();
    let mut diff: Vec<&str> = Vec::new();
    for (p, d) in a {
        match mb.get(p.as_str()) {
            Some(x) if x == d => {}
            _ => diff.push(p),
        }
    }
    for (p, _) in b {
        if !ma.contains(p.as_str()) {
            diff.push(p);
        }
    }
    let is_parent = |p: &str, q: &str| {
        q.len() > p.len() && q.starts_with(p) && matches!(q.as_bytes()[p.len()], b'.')
    };
    let set: Vec<&str> = diff.clone();
    diff.retain(|p| !set.iter().any(|q| is_parent(p, q)));
    diff.sort_unstable();
    diff.dedup();
    diff.into_iter().map(String::from).collect()
}

/// A value that is part of the decoder state (see the [module documentation](self)).
pub trait StateDigest {
    /// Feeds the value to `d`.
    fn digest(&self, d: &mut Digester);

    /// Feeds the elements of a slice (primitive types feed the memory at once).
    #[inline]
    fn digest_slice(s: &[Self], d: &mut Digester)
    where
        Self: Sized,
    {
        for (i, x) in s.iter().enumerate() {
            d.element(i, x);
        }
    }
}

/// The digest of `v`.
pub fn digest_of<T: StateDigest + ?Sized>(v: &T) -> Digest {
    let mut d = Digester::new();
    v.digest(&mut d);
    d.finish()
}

macro_rules! primitive {
    ($($t:ty),*) => {$(
        impl StateDigest for $t {
            #[inline]
            fn digest(&self, d: &mut Digester) {
                d.bytes(&self.to_le_bytes());
            }

            #[inline]
            #[cfg(target_endian = "little")]
            fn digest_slice(s: &[Self], d: &mut Digester) {
                // SAFETY: the primitive integer and float types have no padding and every bit
                // pattern is initialized, so their memory can be read as bytes (little endian,
                // the bytes of `to_le_bytes()`).
                let b = unsafe {
                    core::slice::from_raw_parts(s.as_ptr().cast::<u8>(), core::mem::size_of_val(s))
                };
                d.bytes(b);
            }
        }
    )*};
}

primitive!(u8, i8, u16, i16, u32, i32, u64, i64, u128, i128, f32, f64);

impl StateDigest for usize {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        d.u64(*self as u64);
    }
}

impl StateDigest for isize {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        d.u64(*self as u64);
    }
}

impl StateDigest for bool {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        d.bytes(&[*self as u8]);
    }

    #[inline]
    fn digest_slice(s: &[Self], d: &mut Digester) {
        // SAFETY: `bool` is one byte with the value 0 or 1.
        let b = unsafe { core::slice::from_raw_parts(s.as_ptr().cast::<u8>(), s.len()) };
        d.bytes(b);
    }
}

impl StateDigest for char {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        d.u32(*self as u32);
    }
}

impl StateDigest for () {
    #[inline]
    fn digest(&self, _d: &mut Digester) {}
}

impl StateDigest for str {
    fn digest(&self, d: &mut Digester) {
        d.u64(self.len() as u64);
        d.bytes(self.as_bytes());
    }
}

impl StateDigest for String {
    fn digest(&self, d: &mut Digester) {
        self.as_str().digest(d);
    }
}

impl<T: StateDigest, const N: usize> StateDigest for [T; N] {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        T::digest_slice(self, d);
    }
}

impl<T: StateDigest> StateDigest for [T] {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        d.u64(self.len() as u64);
        T::digest_slice(self, d);
    }
}

impl<T: StateDigest> StateDigest for Vec<T> {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        self.as_slice().digest(d);
    }
}

impl<T: StateDigest> StateDigest for VecDeque<T> {
    fn digest(&self, d: &mut Digester) {
        d.u64(self.len() as u64);
        let (a, b) = self.as_slices();
        T::digest_slice(a, d);
        T::digest_slice(b, d);
    }
}

impl<T: StateDigest + ?Sized> StateDigest for Box<T> {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        (**self).digest(d);
    }
}

impl<T: StateDigest> StateDigest for Option<T> {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        match self {
            None => d.bytes(&[0]),
            Some(v) => {
                d.bytes(&[1]);
                v.digest(d);
            }
        }
    }
}

/// References in the decoder state point to constant tables: fed by address and size (the same
/// table gives the same digest; the digests are only compared within one process).
impl<T: ?Sized> StateDigest for &T {
    #[inline]
    fn digest(&self, d: &mut Digester) {
        d.u64((*self as *const T).cast::<u8>() as usize as u64);
        d.u64(core::mem::size_of_val(*self) as u64);
    }
}

impl<T> StateDigest for core::marker::PhantomData<T> {
    #[inline]
    fn digest(&self, _d: &mut Digester) {}
}

macro_rules! tuple {
    ($($n:tt $t:ident),*) => {
        impl<$($t: StateDigest),*> StateDigest for ($($t,)*) {
            #[inline]
            fn digest(&self, d: &mut Digester) {
                $(d.element($n, &self.$n);)*
            }
        }
    };
}

tuple!(0 A);
tuple!(0 A, 1 B);
tuple!(0 A, 1 B, 2 C);
tuple!(0 A, 1 B, 2 C, 3 D);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digest_distinguishes_values_and_layout() {
        assert_eq!(digest_of(&[1i32, 2, 3]), digest_of(&[1i32, 2, 3]));
        assert_ne!(digest_of(&[1i32, 2, 3]), digest_of(&[1i32, 2, 4]));
        /* lengths are part of the digest of slices */
        assert_ne!(digest_of(&vec![0u8; 3]), digest_of(&vec![0u8; 4]));
        assert_ne!(digest_of(&(vec![1u8], vec![2u8, 3])), digest_of(&(vec![1u8, 2], vec![3u8])));
        assert_ne!(digest_of(&Some(0u8)), digest_of(&None::<u8>));
        /* the digest does not depend on how the bytes are split */
        let mut a = Digester::new();
        a.bytes(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]);
        let mut b = Digester::new();
        b.bytes(&[1, 2, 3]);
        b.bytes(&[4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]);
        assert_eq!(a.finish(), b.finish());
        /* every bit of a large buffer matters */
        let mut v = vec![0u32; 4096];
        let d0 = digest_of(&v);
        for i in [0, 1, 17, 4095] {
            for bit in [0, 13, 31] {
                v[i] ^= 1 << bit;
                assert_ne!(digest_of(&v), d0);
                v[i] ^= 1 << bit;
            }
        }
        assert_eq!(digest_of(&v), d0);
    }

    #[test]
    fn traces_locate_differences() {
        let t = |v: &(Vec<u8>, [i16; 2], Option<u32>)| {
            let mut d = Digester::traced(8);
            v.digest(&mut d);
            d.finish_traced().1
        };
        let a = (vec![1u8, 2], [3i16, 4], Some(5u32));
        let mut b = a.clone();
        b.1[1] = 9;
        assert_eq!(diff_traces(&t(&a), &t(&b)), vec!["[1]".to_string()]);
        assert!(diff_traces(&t(&a), &t(&a)).is_empty());
    }
}
