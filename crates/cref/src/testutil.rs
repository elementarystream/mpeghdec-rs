// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Helpers for randomized differential tests.

/// Small deterministic xorshift64* generator.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    pub fn next_u32(&mut self) -> u32 {
        (self.next_u64() >> 32) as u32
    }

    /// Random `i32`, biased towards edge cases and values of varying magnitude.
    pub fn i32(&mut self) -> i32 {
        match self.next_u32() % 16 {
            0 => [
                0,
                1,
                -1,
                i32::MAX,
                i32::MIN,
                i32::MIN + 1,
                i32::MAX - 1,
                0x4000_0000,
                -0x4000_0000,
            ][(self.next_u32() % 9) as usize],
            1..=5 => (self.next_u32() as i32) >> (self.next_u32() % 32),
            _ => self.next_u32() as i32,
        }
    }

    /// Random `i16`, biased towards edge cases.
    pub fn i16(&mut self) -> i16 {
        match self.next_u32() % 16 {
            0 => [0, 1, -1, i16::MAX, i16::MIN, i16::MIN + 1, 0x4000, -0x4000]
                [(self.next_u32() % 8) as usize],
            1..=4 => (self.next_u32() as i16) >> (self.next_u32() % 16),
            _ => self.next_u32() as i16,
        }
    }

    /// Random `i64`, biased towards edge cases.
    pub fn i64(&mut self) -> i64 {
        match self.next_u32() % 16 {
            0 => [0, 1, -1, i64::MAX, i64::MIN, i64::MIN + 1][(self.next_u32() % 6) as usize],
            1..=5 => (self.next_u64() as i64) >> (self.next_u32() % 64),
            _ => self.next_u64() as i64,
        }
    }

    /// Random integer in `lo..=hi`.
    pub fn range(&mut self, lo: i32, hi: i32) -> i32 {
        lo + (self.next_u64() % ((hi as i64 - lo as i64 + 1) as u64)) as i32
    }
}
