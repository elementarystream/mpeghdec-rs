// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/scramble.h and libFDK/include/arm/scramble_arm.h (the AArch64 build
// compiles the table driven variants for the lengths 64, 128, 256 and 512).

//! Bit reversal permutation of interleaved complex data.

use crate::common_fix::FixpDbl;

/// `SCRAMBLE(x, a, b)`: exchange the complex values `a` and `b`.
#[inline(always)]
fn swap_cplx(x: &mut [FixpDbl], a: usize, b: usize) {
    x.swap(2 * a, 2 * b);
    x.swap(2 * a + 1, 2 * b + 1);
}

static BITREVERSE: [u8; 16] =
    [0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE, 0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF];

/// `scramble(x, length)`: bit reversal of the `length` interleaved complex values in `x`.
pub fn scramble(x: &mut [FixpDbl], length: i32) {
    match length {
        64 => {
            // This table indicates, which entries of the vector should be exchanged.
            static BR_OFFSETS64: [[u8; 2]; 28] = [
                [1, 32],
                [2, 16],
                [3, 48],
                [4, 8],
                [5, 40],
                [6, 24],
                [7, 56],
                [9, 36],
                [10, 20],
                [11, 52],
                [13, 44],
                [14, 28],
                [15, 60],
                [17, 34],
                [19, 50],
                [21, 42],
                [22, 26],
                [23, 58],
                [25, 38],
                [27, 54],
                [29, 46],
                [31, 62],
                [35, 49],
                [37, 41],
                [39, 57],
                [43, 53],
                [47, 61],
                [55, 59],
            ];
            for i in (0..28).rev() {
                let src = BR_OFFSETS64[i][0] as usize;
                let dst = BR_OFFSETS64[i][1] as usize;
                swap_cplx(x, src, dst);
            }
        }
        128 => {
            // In order to bit-reverse a 6-bit value, the lookup table must be used twice:
            // rev(n) = bitreverse[n&0xF]<<2 + bitreverse[n>>4]>>2
            // These bits indicate, if the entry x[n] has to be exchanged or not. The last words
            // are skipped and replaced by hard-coded exchanges at the procedure end.
            static BITS: [u32; 3] = [0xEEEEFEFE, 0xAAAAEAEA, 0x8888A8A8];
            let mut n: usize = 0;
            let mut i = 0;
            while i < 2 {
                let mut bits16 = BITS[i];
                for _ in 0..16 {
                    let rev_n = ((BITREVERSE[n & 0xF] as usize) << 2)
                        + ((BITREVERSE[n >> 4] as usize) >> 2);
                    swap_cplx(x, 2 * n + 1, rev_n + 64);
                    if bits16 & 1 != 0 {
                        swap_cplx(x, 2 * n, rev_n);
                    }
                    bits16 >>= 2;
                    n += 1;
                }
                i += 1;
            }
            let mut bits16 = BITS[i];
            for _ in 0..8 {
                if bits16 & 2 != 0 {
                    let rev_n = ((BITREVERSE[n & 0xF] as usize) << 2)
                        + ((BITREVERSE[n >> 4] as usize) >> 2);
                    swap_cplx(x, 2 * n + 1, rev_n + 64);
                }
                n += 1;
                if bits16 & 8 != 0 {
                    let rev_n = ((BITREVERSE[n & 0xF] as usize) << 2)
                        + ((BITREVERSE[n >> 4] as usize) >> 2);
                    swap_cplx(x, 2 * n + 1, rev_n + 64);
                }
                bits16 >>= 4;
                n += 1;
            }
            swap_cplx(x, 103, 115);
            swap_cplx(x, 111, 123);
        }
        256 => {
            static BITS: [u32; 7] = [
                0xFEFEFFFE, 0xEEEEFEEE, 0xEAEAEEEA, 0xAAAAEAAA, 0xA8A8AAA8, 0x8888A888, 0x80808880,
            ];
            let mut n: usize = 0;
            let mut i = 0;
            while i < 4 {
                let mut bits16 = BITS[i];
                let bitreverse_lsb = (BITREVERSE[i] >> 1) as usize;
                for j in 0..16 {
                    let rev_n = bitreverse_lsb + ((BITREVERSE[j] as usize) << 3);
                    swap_cplx(x, 2 * n + 1, rev_n + 128);
                    if bits16 & 1 != 0 {
                        swap_cplx(x, 2 * n, rev_n);
                    }
                    bits16 >>= 2;
                    n += 1;
                }
                i += 1;
            }
            while i < 7 {
                let mut bits16 = BITS[i];
                let bitreverse_lsb = (BITREVERSE[i] >> 1) as usize + 128;
                for j in 0..16 {
                    if bits16 & 2 != 0 {
                        let rev_n = bitreverse_lsb + ((BITREVERSE[j] as usize) << 3);
                        swap_cplx(x, 2 * n + 1, rev_n);
                    }
                    bits16 >>= 2;
                    n += 1;
                }
                i += 1;
            }
            swap_cplx(x, 239, 247);
        }
        512 => {
            static BITS: [u16; 14] = [
                0xFFFE, 0xFEFE, 0xFEEE, 0xEEEE, 0xEEEA, 0xEAEA, 0xEAAA, 0xAAAA, 0xAAA8, 0xA8A8,
                0xA888, 0x8888, 0x8880, 0x8080,
            ];
            let mut n: usize = 0;
            let mut i = 0;
            while i < 8 {
                let mut bits16 = BITS[i] as u32;
                bits16 += bits16 << 16;
                let bitreverse_lsb = BITREVERSE[i] as usize;
                for j in 0..16 {
                    let rev_n = bitreverse_lsb + ((BITREVERSE[j] as usize) << 4);
                    if bits16 & 1 != 0 {
                        swap_cplx(x, 2 * n, rev_n);
                    }
                    swap_cplx(x, 2 * n + 1, rev_n + 256);
                    bits16 >>= 2;
                    n += 1;
                }
                i += 1;
            }
            while i < 14 {
                let mut bits16 = BITS[i] as u32;
                bits16 += bits16 << 16;
                let bitreverse_lsb = BITREVERSE[i] as usize + 256;
                for j in 0..16 {
                    if bits16 & 2 != 0 {
                        let rev_n = bitreverse_lsb + ((BITREVERSE[j] as usize) << 4);
                        swap_cplx(x, 2 * n + 1, rev_n);
                    }
                    bits16 >>= 2;
                    n += 1;
                }
                i += 1;
            }
            swap_cplx(x, 463, 487);
            swap_cplx(x, 479, 503);
        }
        _ => {
            let mut j: i32 = 0;
            let mut m: i32 = 1;
            while m < length - 1 {
                let mut k = length >> 1;
                loop {
                    j ^= k;
                    if j & k != 0 {
                        break;
                    }
                    k >>= 1;
                }
                if j > m {
                    swap_cplx(x, m as usize, j as usize);
                }
                m += 1;
            }
        }
    }
}
