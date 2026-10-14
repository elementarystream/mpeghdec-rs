// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Software model of the x87 floating point arithmetic of the 32 bit x86 build.
//!
//! clang's default 32 bit x86 target (i686) has no SSE, so the reference computes `float` and
//! `double` expressions with the x87 unit: the operations round to the 64 bit significand of the
//! extended precision format (precision control "extended", the Linux default; round to nearest
//! even), and values are only rounded to `float` or `double` where the compiled code stores them
//! to memory, e.g. for explicit conversions to `float` (`fstps`). Conversions from `int` are
//! exact (`fild`) and are not rounded to the type of the C expression.
//!
//! [`F80`] reproduces the operations used by the reference (multiplication, division, square
//! root, rounding to `float`, truncation to `int` like `fistp` with the rounding mode "toward
//! zero") for finite values and NaN; the values in the decoder never get near the limits of the
//! exponent range (subnormal `float` results are not modelled).

/// A value of the x87 extended precision format: `(-1)^neg * mant * 2^(exp - 63)` with a
/// normalized significand (`mant >= 2^63`), or zero (`mant == 0`), or NaN.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct F80 {
    neg: bool,
    exp: i32,
    mant: u64,
    nan: bool,
}

impl F80 {
    pub const ZERO: F80 = F80 { neg: false, exp: 0, mant: 0, nan: false };
    pub const ONE: F80 = F80 { neg: false, exp: 0, mant: 1 << 63, nan: false };
    pub const NAN: F80 = F80 { neg: false, exp: 0, mant: 0, nan: true };

    /// `true` for NaN.
    pub fn is_nan(self) -> bool {
        self.nan
    }

    /// The value `wide * 2^wide_exp` (plus less than one unit of `wide` if `sticky`), rounded to
    /// a 64 bit significand (round to nearest, ties to even).
    fn round(neg: bool, wide: u128, wide_exp: i32, sticky: bool) -> F80 {
        if wide == 0 {
            return F80 { neg, ..F80::ZERO };
        }
        let bits = 128 - wide.leading_zeros() as i32;
        let exp = wide_exp + bits - 1;
        if bits <= 64 {
            // exact, or the sticky rest is below half a unit
            return F80 { neg, exp, mant: (wide << (64 - bits)) as u64, nan: false };
        }
        let shift = (bits - 64) as u32;
        let mut mant = (wide >> shift) as u64;
        let rest = wide & ((1u128 << shift) - 1);
        let half = 1u128 << (shift - 1);
        let up = rest > half || (rest == half && (sticky || mant & 1 != 0));
        let mut exp = exp;
        if up {
            mant = mant.wrapping_add(1);
            if mant == 0 {
                mant = 1 << 63;
                exp += 1;
            }
        }
        F80 { neg, exp, mant, nan: false }
    }

    /// `fild`: exact conversion of an integer.
    pub fn from_i32(v: i32) -> F80 {
        F80::round(v < 0, v.unsigned_abs() as u128, 0, false)
    }

    /// `fild` of a 64 bit integer (exact, the significand has 64 bits).
    pub fn from_i64(v: i64) -> F80 {
        F80::round(v < 0, v.unsigned_abs() as u128, 0, false)
    }

    /// `fld` of a `float` (exact).
    pub fn from_f32(v: f32) -> F80 {
        F80::from_f64(v as f64)
    }

    /// `fld` of a `double` (exact).
    pub fn from_f64(v: f64) -> F80 {
        if v.is_nan() {
            return F80::NAN;
        }
        let bits = v.to_bits();
        let neg = bits >> 63 != 0;
        let biased = ((bits >> 52) & 0x7ff) as i32;
        let frac = bits & ((1u64 << 52) - 1);
        assert!(biased != 0x7ff, "x87 model: infinities are not supported");
        if biased == 0 {
            // zero or subnormal
            return F80::round(neg, frac as u128, -1074, false);
        }
        F80::round(neg, (frac | (1u64 << 52)) as u128, biased - 1075, false)
    }

    /// `fmul`
    pub fn fmul(self, b: F80) -> F80 {
        if self.nan || b.nan {
            return F80::NAN;
        }
        let neg = self.neg != b.neg;
        F80::round(neg, self.mant as u128 * b.mant as u128, self.exp + b.exp - 126, false)
    }

    /// `fadd`
    pub fn fadd(self, b: F80) -> F80 {
        if self.nan || b.nan {
            return F80::NAN;
        }
        if self.mant == 0 {
            return if b.mant == 0 { F80 { neg: self.neg && b.neg, ..F80::ZERO } } else { b };
        }
        if b.mant == 0 {
            return self;
        }
        let (big, small) =
            if (self.exp, self.mant) >= (b.exp, b.mant) { (self, b) } else { (b, self) };
        // 62 guard bits below the significand of the larger operand
        let a = (big.mant as u128) << 62;
        let d = (big.exp - small.exp) as u32;
        let full = (small.mant as u128) << 62;
        let (c, sticky) =
            if d >= 127 { (0, true) } else { (full >> d, full & ((1u128 << d) - 1) != 0) };
        let wide_exp = big.exp - 63 - 62;
        if big.neg == small.neg {
            F80::round(big.neg, a + c, wide_exp, sticky)
        } else if a == c && !sticky {
            F80::ZERO
        } else {
            // the sticky rest of the subtrahend lowers the difference below a - c
            F80::round(big.neg, a - c - sticky as u128, wide_exp, sticky)
        }
    }

    /// `fsub`
    pub fn fsub(self, b: F80) -> F80 {
        self.fadd(F80 { neg: !b.neg, ..b })
    }

    /// `fdiv`: `self / b` (`b` must not be zero).
    pub fn fdiv(self, b: F80) -> F80 {
        if self.nan || b.nan {
            return F80::NAN;
        }
        assert!(b.mant != 0, "x87 model: division by zero");
        let neg = self.neg != b.neg;
        if self.mant == 0 {
            return F80 { neg, ..F80::ZERO };
        }
        let d = b.mant as u128;
        let num = (self.mant as u128) << 64;
        let (q, r) = (num / d, num % d);
        // 62 more quotient bits and the sticky bit of the rest
        let r2 = r << 62;
        let wide = (q << 62) | (r2 / d);
        F80::round(neg, wide, self.exp - b.exp - 126, r2 % d != 0)
    }

    /// `fsqrt` (NaN for negative values).
    pub fn sqrt(self) -> F80 {
        if self.nan || (self.neg && self.mant != 0) {
            return F80::NAN;
        }
        if self.mant == 0 {
            return self;
        }
        // value = mant * 2^e with e = exp - 63; n = mant * 2^k in [2^126, 2^128) with e - k even
        let e = self.exp - 63;
        let k = if (e - 63) % 2 == 0 { 63 } else { 64 };
        let n = (self.mant as u128) << k;
        let s = n.isqrt();
        let rest = n - s * s;
        // s has 64 bits; the exact root lies in [s, s + 1) and is never s + 1/2
        let half_exp = (e - k) / 2;
        F80::round(false, (s << 1) | (rest > s) as u128, half_exp - 1, rest != 0)
    }

    /// Rounding to `float` (`fstps`), round to nearest even.
    pub fn to_f32(self) -> f32 {
        if self.nan {
            return f32::NAN;
        }
        self.to_bits(24) as f32
    }

    /// Rounding to `double` (`fstpl`), round to nearest even.
    pub fn to_f64(self) -> f64 {
        if self.nan {
            return f64::NAN;
        }
        self.to_bits(53)
    }

    /// The value rounded to a significand of `p <= 53` bits, as an `f64` (exact).
    fn to_bits(self, p: u32) -> f64 {
        if self.mant == 0 {
            return if self.neg { -0.0 } else { 0.0 };
        }
        let shift = 64 - p;
        let mut m = self.mant >> shift;
        let rest = self.mant & ((1u64 << shift) - 1);
        let half = 1u64 << (shift - 1);
        let mut exp = self.exp;
        if rest > half || (rest == half && m & 1 != 0) {
            m += 1;
            if m == 1 << p {
                m >>= 1;
                exp += 1;
            }
        }
        // m * 2^(exp - p + 1), exact in f64 for the exponent range used here
        let v = m as f64 * pow2(exp - p as i32 + 1);
        if self.neg {
            -v
        } else {
            v
        }
    }

    /// `fistp` with the rounding mode "toward zero" (as the compiled C conversion to `int`):
    /// `INT_MIN` ("integer indefinite") for NaN and values out of range.
    pub fn to_i32_trunc(self) -> i32 {
        match self.to_i64_trunc_checked(31) {
            Some(v) => v as i32,
            None => i32::MIN,
        }
    }

    /// `fistpll` with the rounding mode "toward zero": `INT64_MIN` for NaN and values out of
    /// range.
    pub fn to_i64_trunc(self) -> i64 {
        self.to_i64_trunc_checked(63).unwrap_or(i64::MIN)
    }

    /// The conversion to `uint64_t` of the compiled C code (`fistpll` of the value minus 2^63
    /// for values from 2^63, the sign bit of the result toggled): truncation toward zero for
    /// values in `0 .. 2^64`.
    pub fn to_u64_trunc(self) -> u64 {
        let two63 = F80 { neg: false, exp: 63, mant: 1 << 63, nan: false };
        if !self.nan && !self.neg && self.exp >= 63 {
            (self.fsub(two63).to_i64_trunc() as u64) ^ (1 << 63)
        } else {
            self.to_i64_trunc() as u64
        }
    }

    /// The value truncated toward zero if it fits into a signed integer of `bits + 1` bits.
    fn to_i64_trunc_checked(self, bits: i32) -> Option<i64> {
        if self.nan {
            return None;
        }
        if self.mant == 0 || self.exp < 0 {
            return Some(0);
        }
        if self.exp > bits {
            return None;
        }
        let mag = (self.mant >> (63 - self.exp)) as i128;
        let v = if self.neg { -mag } else { mag };
        let lim = 1i128 << bits;
        if v >= lim || v < -lim {
            None
        } else {
            Some(v as i64)
        }
    }
}

/// `2^e` as `f64` (normal range).
fn pow2(e: i32) -> f64 {
    let mut v = 1.0f64;
    let mut e = e;
    while e > 1000 {
        v *= f64::from_bits(((1023 + 1000) as u64) << 52);
        e -= 1000;
    }
    while e < -1000 {
        v *= f64::from_bits(((1023 - 1000) as u64) << 52);
        e += 1000;
    }
    v * f64::from_bits(((1023 + e) as u64) << 52)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_conversions() {
        for v in [0, 1, -1, 3, 0x7fff_ffff, i32::MIN, 12345678] {
            assert_eq!(F80::from_i32(v).to_i32_trunc(), v);
            assert_eq!(F80::from_i32(v).to_f64(), v as f64);
        }
        for v in [0.5f32, -1.25, 46340.9492, 1e-20, 3.0e30] {
            assert_eq!(F80::from_f32(v).to_f32(), v);
        }
        assert_eq!(F80::from_f64(2147483648.0).to_i32_trunc(), i32::MIN);
        assert_eq!(F80::from_f64(-2147483648.0).to_i32_trunc(), i32::MIN);
        assert_eq!(F80::from_f64(-2147483647.9).to_i32_trunc(), -2147483647);
        assert_eq!(F80::NAN.to_i32_trunc(), i32::MIN);
    }

    #[test]
    fn rounding_to_f32_and_f64() {
        // 1 + 2^-24 is a tie in float (to even: 1), 1 + 3 * 2^-25 rounds up
        let a = F80::ONE.fmul(F80::from_f64(1.0 + 2f64.powi(-24)));
        assert_eq!(a.to_f32(), 1.0);
        let b = F80::from_f64(1.0 + 3.0 * 2f64.powi(-25));
        assert_eq!(b.to_f32(), 1.0 + 2f32.powi(-23));
        assert_eq!(F80::from_i32(16777217).to_f32(), 16777216.0);
        assert_eq!(F80::from_i32(16777219).to_f32(), 16777220.0);
        assert_eq!(F80::from_i64((1 << 53) + 1).to_f64(), (1u64 << 53) as f64);
    }

    #[test]
    fn addition() {
        let d = |v: f64| F80::from_f64(v);
        assert_eq!(d(1.5).fadd(d(2.25)).to_f64(), 3.75);
        assert_eq!(d(1.5).fsub(d(2.25)).to_f64(), -0.75);
        assert_eq!(d(1.0).fsub(d(1.0)).to_f64(), 0.0);
        // 2^64 + 1 is rounded to 2^64 (ties to even), 2^64 + 3 to 2^64 + 4 (65 bits)
        let two64 = d(2f64.powi(64));
        assert_eq!(two64.fadd(d(1.0)), two64);
        assert_eq!(two64.fadd(d(3.0)).fsub(two64).to_f64(), 4.0);
        // 1 - 2^-70 rounds to 1, 1 - 2^-64 is exact
        assert_eq!(d(1.0).fsub(d(2f64.powi(-70))), d(1.0));
        assert_eq!(d(1.0).fsub(d(2f64.powi(-64))).fsub(d(1.0)).to_f64(), -(2f64.powi(-64)));
        assert_eq!(d(1e19).to_u64_trunc(), 10_000_000_000_000_000_000);
        assert_eq!(d(12345.99).to_u64_trunc(), 12345);
    }

    #[test]
    fn operations_match_f64_where_the_result_is_exact() {
        let d = |v: f64| F80::from_f64(v);
        assert_eq!(d(3.0).fmul(d(-0.5)).to_f64(), -1.5);
        assert_eq!(d(1.0).fdiv(d(4.0)).to_f64(), 0.25);
        assert_eq!(d(2.25).sqrt().to_f64(), 1.5);
        assert_eq!(d(1e-6).fmul(d(1e6)).fdiv(d(1e-6)).to_f64(), 1e6);
        assert!(d(-2.0).sqrt().is_nan());
    }

    /// The extended precision results rounded to `double` agree with the correctly rounded
    /// `f64` results except for rare double rounding cases.
    #[test]
    fn operations_round_like_ieee_extended() {
        let mut x = 0x1234_5678_9abc_def1u64;
        let mut same = 0;
        for _ in 0..10000 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let a = (x >> 11) as f64 / (1u64 << 40) as f64 + 0.5;
            let b = (x & 0xfff_ffff) as f64 + 1.0;
            same += (F80::from_f64(a).fdiv(F80::from_f64(b)).to_f64() == a / b) as u32;
            same += (F80::from_f64(a).fmul(F80::from_f64(b)).to_f64() == a * b) as u32;
            same += (F80::from_f64(a).sqrt().to_f64() == a.sqrt()) as u32;
        }
        assert!(same >= 29990, "{same}");
    }
}
