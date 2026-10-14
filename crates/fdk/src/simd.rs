// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Runtime selection of the x86 SIMD code paths.
//!
//! The x86 and x86-64 targets only guarantee SSE2 (or nothing, for i686), so the AVX2 code paths
//! are selected at run time. They compute exactly the same values as the portable code. The
//! environment variable `MPEGHDEC_NO_SIMD` (any value) disables them, e.g. for comparisons.

/// `true` if the AVX2 code paths are used (x86 and x86-64 with AVX2, unless `MPEGHDEC_NO_SIMD`
/// is set).
#[inline]
pub fn avx2() -> bool {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    {
        use std::sync::atomic::{AtomicU8, Ordering};
        // 0: not yet determined, 1: no, 2: yes
        static AVX2: AtomicU8 = AtomicU8::new(0);
        match AVX2.load(Ordering::Relaxed) {
            2 => true,
            1 => false,
            _ => {
                let on = std::is_x86_feature_detected!("avx2")
                    && std::env::var_os("MPEGHDEC_NO_SIMD").is_none();
                AVX2.store(if on { 2 } else { 1 }, Ordering::Relaxed);
                on
            }
        }
    }
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
    {
        false
    }
}

/// `true` if the x86-64-v3 code paths are used: [`avx2()`] and BMI1, BMI2 and LZCNT.
#[inline]
pub fn x86_v3() -> bool {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    {
        use std::sync::atomic::{AtomicU8, Ordering};
        // 0: not yet determined, 1: no, 2: yes
        static V3: AtomicU8 = AtomicU8::new(0);
        match V3.load(Ordering::Relaxed) {
            2 => true,
            1 => false,
            _ => {
                let on = avx2()
                    && std::is_x86_feature_detected!("bmi1")
                    && std::is_x86_feature_detected!("bmi2")
                    && std::is_x86_feature_detected!("lzcnt")
                    && std::is_x86_feature_detected!("popcnt");
                V3.store(if on { 2 } else { 1 }, Ordering::Relaxed);
                on
            }
        }
    }
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
    {
        false
    }
}

/// Defines a function whose body is compiled twice: for the baseline target and, on x86 and
/// x86-64, for AVX2 (selected at run time with [`avx2()`]). The compiler vectorizes the loops of
/// the AVX2 clone with 256 bit registers and the AVX2 instructions (e.g. `vpmuldq` for the 64 bit
/// products of `fMult()`), which compute the same values.
///
/// The parameters must be plain identifiers (no patterns, no `self`).
#[macro_export]
macro_rules! avx2_clone {
    (
        $(#[$attr:meta])*
        $vis:vis fn $name:ident($($arg:ident: $ty:ty),* $(,)?) $(-> $ret:ty)? $body:block
    ) => {
        $(#[$attr])*
        $vis fn $name($($arg: $ty),*) $(-> $ret)? {
            #[inline(always)]
            #[allow(clippy::too_many_arguments)]
            fn imp($($arg: $ty),*) $(-> $ret)? $body
            #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
            if $crate::simd::avx2() {
                #[target_feature(enable = "avx2")]
                #[allow(clippy::too_many_arguments)]
                unsafe fn avx2($($arg: $ty),*) $(-> $ret)? {
                    imp($($arg),*)
                }
                // SAFETY: AVX2 is available.
                return unsafe { avx2($($arg),*) };
            }
            imp($($arg),*)
        }
    };
}

/// AVX2 building blocks of the vectorized code (to be inlined into `avx2` functions).
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
pub(crate) mod x86 {
    #[cfg(target_arch = "x86")]
    pub use core::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    pub use core::arch::x86_64::*;

    /// `(x * w) >> 16` per lane for 16 bit values `w`, the `fixmuldiv2_DS()` of the x86 build.
    /// Computed exactly in 32 bit lanes as `(x >> 16) * w + (((x & 0xFFFF) * w) >> 16)` (no
    /// intermediate value exceeds 31 bits).
    #[inline(always)]
    pub unsafe fn mul_div2(x: __m256i, w: __m256i) -> __m256i {
        let hi = _mm256_mullo_epi32(_mm256_srai_epi32::<16>(x), w);
        let lo = _mm256_and_si256(x, _mm256_set1_epi32(0xFFFF));
        _mm256_add_epi32(hi, _mm256_srai_epi32::<16>(_mm256_mullo_epi32(lo, w)))
    }

    /// Reverses the order of the 8 lanes.
    #[inline(always)]
    pub unsafe fn rev(x: __m256i) -> __m256i {
        _mm256_permutevar8x32_epi32(x, _mm256_setr_epi32(7, 6, 5, 4, 3, 2, 1, 0))
    }

    /// Loads the interleaved complex values `k .. k + 8` as (re, im).
    #[inline(always)]
    pub unsafe fn load_up(p: *const i32, k: usize) -> (__m256i, __m256i) {
        let lo = _mm256_loadu_si256(p.add(2 * k) as *const __m256i);
        let hi = _mm256_loadu_si256(p.add(2 * k + 8) as *const __m256i);
        // (r0 r1 i0 i1 | r2 r3 i2 i3), (r4 r5 i4 i5 | r6 r7 i6 i7)
        let lo = _mm256_shuffle_epi32::<0b11_01_10_00>(lo);
        let hi = _mm256_shuffle_epi32::<0b11_01_10_00>(hi);
        // (r0 r1 r4 r5 | r2 r3 r6 r7)
        let re = _mm256_unpacklo_epi64(lo, hi);
        let im = _mm256_unpackhi_epi64(lo, hi);
        (
            _mm256_permute4x64_epi64::<0b11_01_10_00>(re),
            _mm256_permute4x64_epi64::<0b11_01_10_00>(im),
        )
    }

    /// Loads the interleaved complex values `k, k - 1, ..., k - 7` as (re, im).
    #[inline(always)]
    pub unsafe fn load_down(p: *const i32, k: usize) -> (__m256i, __m256i) {
        let (re, im) = load_up(p, k - 7);
        (rev(re), rev(im))
    }

    /// Stores the interleaved complex values `k .. k + 8`.
    #[inline(always)]
    pub unsafe fn store_up(p: *mut i32, k: usize, re: __m256i, im: __m256i) {
        // (r0 r1 r4 r5 | r2 r3 r6 r7)
        let re = _mm256_permute4x64_epi64::<0b11_01_10_00>(re);
        let im = _mm256_permute4x64_epi64::<0b11_01_10_00>(im);
        _mm256_storeu_si256(p.add(2 * k) as *mut __m256i, _mm256_unpacklo_epi32(re, im));
        _mm256_storeu_si256(p.add(2 * k + 8) as *mut __m256i, _mm256_unpackhi_epi32(re, im));
    }

    /// Stores the interleaved complex values `k, k - 1, ..., k - 7`.
    #[inline(always)]
    pub unsafe fn store_down(p: *mut i32, k: usize, re: __m256i, im: __m256i) {
        store_up(p, k - 7, rev(re), rev(im));
    }

    /// Loads 8 values.
    #[inline(always)]
    pub unsafe fn load(p: *const i32) -> __m256i {
        _mm256_loadu_si256(p as *const __m256i)
    }

    #[inline(always)]
    pub unsafe fn add(x: __m256i, y: __m256i) -> __m256i {
        _mm256_add_epi32(x, y)
    }

    #[inline(always)]
    pub unsafe fn sub(x: __m256i, y: __m256i) -> __m256i {
        _mm256_sub_epi32(x, y)
    }

    #[inline(always)]
    pub unsafe fn neg(x: __m256i) -> __m256i {
        _mm256_sub_epi32(_mm256_setzero_si256(), x)
    }
}
