// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Count of the places reached where the port deviates from the C++ code because the C++ code
//! has undefined behaviour there (corrupt bitstreams: out-of-bounds writes and the like), not in
//! C. The differential fuzz targets (`fuzz/`) compare the port with the reference only up to the
//! first API call that reaches one of them.

use core::sync::atomic::{AtomicU64, Ordering};

static COUNT: AtomicU64 = AtomicU64::new(0);

/// Records that a deviation was taken.
#[cold]
pub fn hit() {
    COUNT.fetch_add(1, Ordering::Relaxed);
}

/// The number of deviations taken so far in this process (all decoders and threads).
pub fn count() -> u64 {
    COUNT.load(Ordering::Relaxed)
}
