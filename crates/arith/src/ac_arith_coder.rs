// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libArithCoding/include/ac_arith_coder.h and libArithCoding/src/ac_arith_coder.cpp.

//! Spectral noiseless coding scheme based on an arithmetic coder in conjunction with an adaptive
//! context (USAC / MPEG-H 3D Audio).

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::FixpDbl;

/// `ARITH_CODING_ERROR`
pub type ArithCodingError = i32;
pub const ARITH_CODER_OK: ArithCodingError = 0;
pub const ARITH_CODER_ERROR: ArithCodingError = 5;

pub const cbitsnew: i32 = 16;
pub const stat_bitsnew: i32 = 14;
/// `0xFFFF`
pub const ari_q4new: i32 = (1 << cbitsnew) - 1;
/// `0x4000`
pub const ari_q1new: i32 = ari_q4new / 4 + 1;
/// `0x8000`
pub const ari_q2new: i32 = 2 * ari_q1new;
/// `0xC000`
pub const ari_q3new: i32 = 3 * ari_q1new;

pub const VAL_ESC: i32 = 16;

/* Arithmetic coder library info */
pub const AC_LIB_VL0: i32 = 3;
pub const AC_LIB_VL1: i32 = 0;
pub const AC_LIB_VL2: i32 = 0;
pub const AC_LIB_TITLE: &str = "Arithmetic Coder Lib";

/// Number of entries of `CArcoData::c_prev`: `(1024 / 2) + 4`.
pub const C_PREV_LEN: usize = (1024 / 2) + 4;

/// `CArcoData`: arithmetic decoder state kept per channel across frames.
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CArcoData {
    /// `m_numberLinesPrev`
    pub m_number_lines_prev: i16,
    /// `c_prev`: 2-tuple context of previous frame, 4 bit.
    pub c_prev: [u8; C_PREV_LEN],
}

impl Default for CArcoData {
    fn default() -> Self {
        CArcoData { m_number_lines_prev: 0, c_prev: [0; C_PREV_LEN] }
    }
}

impl CArcoData {
    /// `CArco_Create()`: a zero-initialized state. `CArco_Destroy()` is `drop`.
    pub fn create() -> Box<CArcoData> {
        Box::default()
    }
}

/// `Tastat`: arithmetic decoder interval state.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tastat {
    pub low: i32,
    pub high: i32,
    pub vobf: i32,
    pub bits_to_follow: i32,
}

#[inline(always)]
fn mul_sbc_14bits(r: i32, c: i32) -> i32 {
    (r * c) >> stat_bitsnew
}

/// `ari_decode_14bits(hBs, s, c_freq, cfl)`: decode one symbol with the cumulative frequency
/// table `c_freq` of `cfl` symbols (`cfl` is one of 2, 4, 17 or 27; other values decode symbol 0).
#[inline]
pub fn ari_decode_14bits(hbs: &mut FdkBitStream, s: &mut Tastat, c_freq: &[i16], cfl: i32) -> i32 {
    if cfl == VAL_ESC + 1 {
        ari_decode_sym::<17, 17>(hbs, s, c_freq[..17].try_into().unwrap())
    } else if cfl == 4 {
        ari_decode_sym::<4, 4>(hbs, s, c_freq[..4].try_into().unwrap())
    } else if cfl == 2 {
        ari_decode_sym::<2, 2>(hbs, s, c_freq[..2].try_into().unwrap())
    } else if cfl == 27 {
        ari_decode_sym::<27, 27>(hbs, s, c_freq[..27].try_into().unwrap())
    } else {
        ari_decode_sym::<0, 1>(hbs, s, c_freq[..1].try_into().unwrap())
    }
}

/// [`ari_decode_14bits`] for a table of `CFL` symbols (`N == CFL`; `CFL == 0`: any other size,
/// decodes symbol 0 with a table of which only the first entry is used, `N == 1`).
#[inline(always)]
fn ari_decode_sym<const CFL: usize, const N: usize>(
    hbs: &mut FdkBitStream,
    s: &mut Tastat,
    c_freq: &[i16; N],
) -> i32 {
    let mut low = s.low;
    let mut high = s.high;
    let value = s.vobf;

    let range = high - low + 1;
    let c = ((value - low + 1) << stat_bitsnew) - 1;
    // `p` is the index relative to `c_freq - 1`, i.e. `p[k]` is `c_freq[p + k - 1]`.
    let f = |i: usize| c_freq[i] as i32 * range;
    let mut p: usize = 0;

    if CFL == (VAL_ESC + 1) as usize {
        /* In 50% of all cases, the first entry is the right one, so we check it prior to all others */
        if f(p) > c {
            p += 1;
            if f(p + 7) > c {
                p += 8;
            }
            if f(p + 3) > c {
                p += 4;
            }
            if f(p + 1) > c {
                p += 2;
            }
            if f(p) > c {
                p += 1;
            }
        }
    } else if CFL == 4 {
        if f(p + 1) > c {
            p += 2;
        }
        if f(p) > c {
            p += 1;
        }
    } else if CFL == 2 {
        if f(p) > c {
            p += 1;
        }
    } else if CFL == 27 {
        let p_24 = 24;

        if f(p + 15) > c {
            p += 16;
        }
        if f(p + 7) > c {
            p += 8;
        }
        if p != p_24 {
            if f(p + 3) > c {
                p += 4;
            }
        }
        if f(p + 1) > c {
            p += 2;
        }

        if p != p_24 + 2 {
            if f(p) > c {
                p += 1;
            }
        }
    }

    let symbol = p;

    if symbol != 0 {
        high = low + mul_sbc_14bits(range, c_freq[symbol - 1] as i32) - 1;
    }

    low += mul_sbc_14bits(range, c_freq[symbol] as i32);

    ari_renormalize(hbs, s, low as u16 as u32, high as u16 as u32, value as u32);

    symbol as i32
}

/// The renormalization loop of `ari_decode_14bits()`, with all iterations done at once:
///
/// ```text
/// for (;;) {
///   if (us_high & 0x8000) {
///     if (!(us_low & 0x8000)) {
///       if (us_low & 0x4000 && !(us_high & 0x4000)) {
///         us_low -= 0x4000; us_high -= 0x4000; value -= 0x4000;
///       } else break;
///     }
///   }
///   us_low = us_low << 1; us_high = (us_high << 1) | 1; value = (value << 1) | FDKreadBit(hBs);
/// }
/// ```
///
/// Every iteration shifts bit 14 of the (16 bit) state into bit 15. As long as bit 15 of `high`
/// is clear or bit 15 of `low` is set, a plain shift is done; these are the `n1` leading one bits
/// of `~high | low`. Afterwards bit 15 of `high` is set and bit 15 of `low` is clear; this does
/// not change anymore since the following iterations (if any) subtract `0x4000`: they are the
/// `n3` leading one bits of `low & ~high` below bit 15. Subtracting `0x4000` from a value whose
/// bit 14 is set clears it (`low`), from a value whose bit 14 is clear it sets it and borrows from
/// bit 15 (`high`, `value`); as bit 15 and above are shifted out and only the 16 LSBs of the
/// result are used (`value` is masked when stored), it is the same as toggling bit 14. So the
/// `n3` subtractions toggle bits `14 .. 15 - n3` of the state after the first `n1` shifts. The
/// shifted-in bits (0 for `low`, 1 for `high`) terminate both runs, so `n1 + n3 <= 16` and the
/// `n1 + n3` bits read one by one are read at once.
#[inline(always)]
fn ari_renormalize(hbs: &mut FdkBitStream, s: &mut Tastat, low: u32, high: u32, value: u32) {
    let n1 = (!((!high | low) << 16)).leading_zeros(); /* 0..=16 */
    let low1 = (low << n1) & 0xFFFF;
    let high1 = ((high << n1) | ((1 << n1) - 1)) & 0xFFFF;
    let n3 = (!((low1 & !high1 & 0x7FFF) << 17)).leading_zeros(); /* 0..=15 */
    let toggle = ((1u32 << n3) - 1) << (15 - n3);
    let n = n1 + n3;
    let bits = hbs.read_bits(n);
    s.low = (((low1 ^ toggle) << n3) & 0xFFFF) as i32;
    s.high = ((((high1 ^ toggle) << n3) | ((1 << n3) - 1)) & 0xFFFF) as i32;
    s.vobf = (((((value << n1) ^ toggle) << n3) | bits) & 0xFFFF) as i32;
}

/// `copyTableAmrwbArith2(tab, sizeIn, sizeOut)` applied to `pArcoData->c_prev`: up/downmix of the
/// previous context on a short-to-long or long-to-short block transition.
///
/// For unusual size combinations the C code accesses memory in front of `c_prev` (the bytes of
/// `m_numberLinesPrev`) or outside of the structure. Accesses to `m_numberLinesPrev` are
/// reproduced (it is overwritten by the caller afterwards); accesses outside of the structure are
/// undefined in C, here they read 0 and are otherwise ignored.
fn copy_table_amrwb_arith2(arco: &mut CArcoData, size_in: i32, size_out: i32) {
    // Memory image of the structure: m_numberLinesPrev (little endian), then c_prev.
    const OFS: i32 = 2 + 2; // position of `tab = c_prev + 2` in `mem`
    let mut mem = [0u8; 2 + C_PREV_LEN];
    mem[..2].copy_from_slice(&arco.m_number_lines_prev.to_le_bytes());
    mem[2..].copy_from_slice(&arco.c_prev);
    let get = |mem: &[u8], i: i32| -> u8 {
        let a = i + OFS;
        if a >= 0 && (a as usize) < mem.len() {
            mem[a as usize]
        } else {
            0
        }
    };
    let set = |mem: &mut [u8], i: i32, v: u8| {
        let a = i + OFS;
        if a >= 0 && (a as usize) < mem.len() {
            mem[a as usize] = v;
        }
    };

    let mut k = 2;

    if size_in < size_out {
        let v = get(&mem, size_in);
        set(&mut mem, size_out, v);
        let v = get(&mem, size_in + 1);
        set(&mut mem, size_out + 1, v);
        if size_in < (size_out >> 2) {
            k = 8;
        } else if size_in == (size_out >> 2) {
            k = 4;
        }

        let mut i = size_out - 1;
        let mut j = size_in - 1;

        while i >= 0 {
            let tq_data0 = get(&mem, j);

            for _ in 0..(k >> 1) {
                set(&mut mem, i, tq_data0);
                i -= 1;
                set(&mut mem, i, tq_data0);
                i -= 1;
            }
            j -= 1;
        }
    } else {
        if size_out < (size_in >> 2) {
            k = 8;
        } else if size_out == (size_in >> 2) {
            k = 4;
        }

        let mut i = 0;
        let mut j = 0;
        while i < size_out {
            let tq_data0 = get(&mem, j);

            set(&mut mem, i, tq_data0);
            i += 1;
            j += k;
        }
        let v = get(&mem, size_in);
        set(&mut mem, size_out, v);
        let v = get(&mem, size_in + 1);
        set(&mut mem, size_out + 1, v);
    }

    arco.m_number_lines_prev = i16::from_le_bytes([mem[0], mem[1]]);
    arco.c_prev.copy_from_slice(&mem[2..]);
}

/// `get_pk_v2(s)`: map the context state to the index of the cumulative frequency table.
#[inline]
fn get_pk_v2(s: u32) -> u32 {
    /* Bypass complex table search for s in range 0..0xF */
    if s <= 0xF {
        return ARI_MERGED_HASH_PS_EXT[s as usize] as u32;
    }

    let p = &ARI_MERGED_HASH_PS;
    let mut o = 0usize;
    let s12 = (s << 12).wrapping_sub(1);
    if s12 > p[485] {
        o += 486; /* 742 - 256 = 486 */
    } else {
        if s12 > p[255] {
            o += 256;
        }
    }

    if s12 > p[o + 127] {
        o += 128;
    }
    if s12 > p[o + 63] {
        o += 64;
    }
    if s12 > p[o + 31] {
        o += 32;
    }
    if s12 > p[o + 15] {
        o += 16;
    }
    if s12 > p[o + 7] {
        o += 8;
    }
    if s12 > p[o + 3] {
        o += 4;
    }
    if s12 > p[o + 1] {
        o += 2;
    }
    let mut j = p[o];
    if s12 > j {
        j = p[o + 1];
    }
    if s != (j >> 12) {
        j >>= 6;
    }
    j & 0x3F
}

/// Number of entries of the [`get_pk_v2`] lookup table: all context states `s` of `decode2()`
/// without escape offset (16 bit context and the "cumulative amplitude below 2" flag in bit 16).
const PK_TABLE_LEN: usize = 1 << 17;

/// The lookup table `get_pk_v2(s)` for `s < PK_TABLE_LEN`, computed on first use.
fn pk_table() -> &'static [u8; PK_TABLE_LEN] {
    static TABLE: std::sync::OnceLock<Box<[u8; PK_TABLE_LEN]>> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        let mut t = Box::new([0u8; PK_TABLE_LEN]);
        for (s, v) in t.iter_mut().enumerate() {
            *v = get_pk_v2(s as u32) as u8;
        }
        t
    })
}

/// `decode2(bbuf, c_prev, pSpectralCoefficient, n, nt)`: `c_prev` starts at
/// `pArcoData->c_prev + 2`.
fn decode2(
    bbuf: &mut FdkBitStream,
    c_prev: &mut [u8],
    p_spectral_coefficient: &mut [FixpDbl],
    n: i32,
    nt: i32,
) -> ArithCodingError {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64"))]
    return fast::decode2(bbuf, c_prev, p_spectral_coefficient, n, nt);
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
    decode2_generic(bbuf, c_prev, p_spectral_coefficient, n, nt)
}

/// The straight port of `decode2()` (the decoder on targets other than x86 and AArch64).
#[inline(always)]
#[cfg_attr(
    any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64"),
    allow(dead_code)
)]
fn decode2_generic(
    bbuf: &mut FdkBitStream,
    c_prev: &mut [u8],
    p_spectral_coefficient: &mut [FixpDbl],
    n: i32,
    nt: i32,
) -> ArithCodingError {
    let error_status = ARITH_CODER_OK;
    let mut r: i32;

    let mut c_3: i32 = 0; /* context of current frame 3 time steps ago */
    let mut c_2: i32 = 0; /* context of current frame 2 time steps ago */
    let mut c_1: i32 = 0; /* context of current frame 1 time steps ago */
    let mut c_0: i32; /* context of current frame to be calculated */

    /* ari_start_decoding_14bits */
    let mut as_ = Tastat {
        low: 0,
        high: ari_q4new,
        vobf: bbuf.read_bits(cbitsnew as u32) as i32,
        bits_to_follow: 0,
    };

    /* arith_map_context */
    let mut state_inc: u16 = ((c_prev[0] as i32) << 12) as u16;
    let pk_table = pk_table();

    // The loop below accesses c_prev[..n + 1] and p_spectral_coefficient[..2 * n].
    let nu = n.max(0) as usize;
    let (c_prev_n, spec_n) = (&mut c_prev[..nu + 1], &mut p_spectral_coefficient[..2 * nu]);

    let mut i: i32 = 0;
    while i < n {
        let iu = i as usize;
        /* arith_get_context */
        let mut s: u32 = (state_inc >> 8) as u32;
        s = s.wrapping_add((c_prev_n[iu + 1] as u32) << 8);
        s = (s << 4).wrapping_add(c_1 as u32);

        state_inc = s as u16;

        if i > 3 {
            /* Cumulative amplitude below 2 */
            if (c_1 + c_2 + c_3) < 5 {
                s = s.wrapping_add(0x10000);
            }
        }

        /* MSBs decoding */
        let mut lev: i32 = 0;
        let mut esc_nb: i32 = 0;
        loop {
            let s_esc = s.wrapping_add((esc_nb << (VAL_ESC + 1)) as u32);
            let pki = match pk_table.get(s_esc as usize) {
                Some(&pki) => pki as u32,
                None => get_pk_v2(s_esc),
            };
            r = ari_decode_sym::<17, 17>(bbuf, &mut as_, &ARI_PK[pki as usize & 0x3F]);
            if r < VAL_ESC {
                break;
            }

            lev += 1;

            if lev > 23 {
                return ARITH_CODER_ERROR;
            }

            if esc_nb < 7 {
                esc_nb += 1;
            }
        }

        /* Stop symbol */
        if r == 0 {
            if esc_nb > 0 {
                break; /* Stop symbol */
            }
            c_0 = 1;
        } else {
            let mut b = r >> 2;
            let mut a = r & 0x3;

            /* LSBs decoding */
            for _ in 0..lev {
                let pidx = if a == 0 {
                    1
                } else if b == 0 {
                    0
                } else {
                    2
                };
                r = ari_decode_sym::<4, 4>(bbuf, &mut as_, &ARI_LSB2[pidx]);
                a = (a << 1) | (r & 1);
                b = (b << 1) | (r >> 1);
            }

            spec_n[2 * iu] = a;
            spec_n[2 * iu + 1] = b;

            c_0 = a + b + 1;
            if c_0 > 0xF {
                c_0 = 0xF;
            }
        }

        /* arith_update_context */
        c_3 = c_2;
        c_2 = c_1;
        c_1 = c_0;
        c_prev_n[iu] = c_0 as u8;

        i += 1;
    }

    bbuf.push_back((cbitsnew - 2) as u32);

    /* We need to run only from 0 to i-1 since all other q[i][1].a,b will be cleared later */
    let j = i as usize;
    // Branchless form of: bits = (a != 0) + (b != 0); if bits != 0 { r = FDKreadBits(bits);
    // if a != 0 && (r >> (bits - 1)) == 0 { a = -a } if b != 0 && (r & 1) == 0 { b = -b } }.
    // Reading 0 bits does not change the bitstream state and negating 0 has no effect.
    for q in spec_n[..2 * j].chunks_exact_mut(2) {
        let bits = (q[0] != 0) as u32 + (q[1] != 0) as u32;
        let r = bbuf.read_bits(bits);
        if (r >> (bits.wrapping_sub(1) & 31)) & 1 == 0 {
            q[0] = -q[0];
        }
        if r & 1 == 0 {
            q[1] = -q[1];
        }
    }

    c_prev[j..nt as usize].fill(1);

    error_status
}

/// `CArco_DecodeArithData(pArcoData, hBs, mdctSpectrum, lg, lg_max, arith_reset_flag)`: decode a
/// spectral data element by using an adaptive context dependent arithmetic coding scheme.
///
/// * `mdct_spectrum` – quantized data output (at least `lg_max` entries, cleared first).
/// * `lg` – number of quantized spectral coefficients (output by the arithmetic decoder).
/// * `lg_max` – max number of quantized spectral coefficients (at most 1024).
/// * `arith_reset_flag` – flag which indicates if the spectral noiseless context must be reset.
pub fn carco_decode_arith_data(
    p_arco_data: &mut CArcoData,
    hbs: &mut FdkBitStream,
    mdct_spectrum: &mut [FixpDbl],
    lg: i32,
    lg_max: i32,
    arith_reset_flag: i32,
) -> ArithCodingError {
    let mut error_status = ARITH_CODER_OK;

    /* Check lg and lg_max consistency. */
    if lg_max < lg {
        return ARITH_CODER_ERROR;
    }

    mdct_spectrum[..lg_max as usize].fill(0);

    /* arith_map_context */
    if arith_reset_flag != 0 {
        p_arco_data.c_prev[..((lg_max / 2) + 4) as usize].fill(0);
    } else {
        if lg_max != p_arco_data.m_number_lines_prev as i32 {
            if p_arco_data.m_number_lines_prev == 0 {
                /* Cannot decode without a valid AC context */
                return ARITH_CODER_ERROR;
            }

            /* short-to-long or long-to-short block transition */
            /* Current length differs compared to previous - perform up/downmix of m_qbuf */
            let size_in = (p_arco_data.m_number_lines_prev >> 1) as i32;
            copy_table_amrwb_arith2(p_arco_data, size_in, lg_max >> 1);
        }
    }

    p_arco_data.m_number_lines_prev = lg_max as i16;

    if lg > 0 {
        error_status =
            decode2(hbs, &mut p_arco_data.c_prev[2..], mdct_spectrum, lg >> 1, lg_max >> 1);
    } else {
        p_arco_data.c_prev[2..2 + (lg_max >> 1) as usize].fill(1);
    }

    if hbs.get_valid_bits() < 0 {
        return ARITH_CODER_ERROR;
    }

    error_status
}

/* 16 bytes quick lookup for ari_merged_hash_ps (the bytes of the ULONG[4] table in memory) */
static ARI_MERGED_HASH_PS_EXT: [u8; 16] = [
    0x01, 0x04, 0x34, 0x0A, 0x0D, 0x10, 0x13, 0x16, 0x12, 0x12, 0x1F, 0x25, 0x25, 0x25, 0x25, 0x2E,
];

static ARI_MERGED_HASH_PS: [u32; 742] = [
    0x00001044, 0x00003D0A, 0x00005350, 0x000074D6, 0x0000A49F, 0x0000F96E, 0x00111000, 0x01111E83,
    0x01113146, 0x01114036, 0x01116863, 0x011194E9, 0x0111F7EE, 0x0112269B, 0x01124775, 0x01126DA1,
    0x0112D912, 0x01131AF0, 0x011336DD, 0x01135CF5, 0x01139DF8, 0x01141A5B, 0x01144773, 0x01146CF5,
    0x0114FDE9, 0x01166CF5, 0x0116FDE4, 0x01174CF3, 0x011FFDCF, 0x01211CC2, 0x01213B2D, 0x01214036,
    0x01216863, 0x012194D2, 0x0122197F, 0x01223AAD, 0x01224036, 0x01226878, 0x0122A929, 0x0122F4AB,
    0x01232B2D, 0x012347B6, 0x01237DF8, 0x0123B929, 0x012417DD, 0x01245D76, 0x01249DF8, 0x0124F912,
    0x01255D75, 0x0125FDE9, 0x01265D75, 0x012B8DF7, 0x01311E2A, 0x01313B5E, 0x0131687B, 0x01321A6D,
    0x013237BC, 0x01326863, 0x0132F4EE, 0x01332B5E, 0x01335DA1, 0x01338E24, 0x01341A5E, 0x01343DB6,
    0x01348DF8, 0x01351935, 0x01355DB7, 0x0135FE12, 0x01376DF7, 0x013FFE29, 0x01400024, 0x01423821,
    0x014318F6, 0x01433821, 0x0143F8E5, 0x01443DA1, 0x01486E38, 0x014FF929, 0x01543EE3, 0x015FF912,
    0x016F298C, 0x018A5A69, 0x021007F1, 0x02112C2C, 0x02114B48, 0x02117353, 0x0211F4AE, 0x02122FEA,
    0x02124B48, 0x02127850, 0x0212F72E, 0x02133A9E, 0x02134036, 0x02138864, 0x021414AD, 0x0214379E,
    0x02145DB6, 0x0214FE1F, 0x02166DB7, 0x02200DC4, 0x02212FEA, 0x022147A0, 0x02218369, 0x0221F7EE,
    0x02222AAD, 0x02224788, 0x02226863, 0x02229929, 0x0222F4AB, 0x02232A9E, 0x02234F08, 0x02237864,
    0x0223A929, 0x022417DE, 0x02244F36, 0x02248863, 0x02251A74, 0x02256DA1, 0x0225FE12, 0x02263DB6,
    0x02276DF7, 0x022FFE29, 0x0231186D, 0x023137BC, 0x02314020, 0x02319ED6, 0x0232196D, 0x023237BC,
    0x02325809, 0x02329429, 0x023317ED, 0x02333F08, 0x02335809, 0x023378E4, 0x02341A7C, 0x02344221,
    0x0234A8D3, 0x023514BC, 0x02354221, 0x0235F8DF, 0x02364861, 0x023FFE29, 0x02400024, 0x0241583B,
    0x024214C8, 0x02424809, 0x02427EE6, 0x02431708, 0x02434809, 0x02436ED0, 0x02441A76, 0x02443821,
    0x024458E3, 0x0244F91F, 0x02454863, 0x0246190A, 0x02464863, 0x024FF929, 0x02525ED0, 0x025314E1,
    0x025348FB, 0x025419A1, 0x025458D0, 0x0254F4E5, 0x02552861, 0x025FF912, 0x02665993, 0x027F5A69,
    0x029F1481, 0x02CF28A4, 0x03100AF0, 0x031120AA, 0x031147A0, 0x03118356, 0x031217EC, 0x03123B5E,
    0x03124008, 0x03127350, 0x031314AA, 0x0313201E, 0x03134F08, 0x03136863, 0x03141A5E, 0x03143F3C,
    0x03200847, 0x03212AAD, 0x03214F20, 0x03218ED6, 0x032218AD, 0x032237BC, 0x03225809, 0x03229416,
    0x032317ED, 0x03234F20, 0x03237350, 0x0323FA6B, 0x03243F08, 0x03246863, 0x0324F925, 0x03254221,
    0x0325F8DF, 0x03264821, 0x032FFE29, 0x03311E47, 0x03313F08, 0x0331580D, 0x033214DE, 0x03323F08,
    0x03324020, 0x03326350, 0x033294E9, 0x033317DE, 0x03333F08, 0x0333627B, 0x0333A9A9, 0x033417FC,
    0x03343220, 0x0334627B, 0x0334A9A9, 0x0335148A, 0x03353220, 0x033588E4, 0x03361A4A, 0x03363821,
    0x0336F8D2, 0x03376863, 0x03411939, 0x0341583B, 0x034214C8, 0x03424809, 0x03426ED0, 0x03431588,
    0x03434809, 0x03436ED0, 0x03441A48, 0x0344480D, 0x03446ED0, 0x03451A4A, 0x03453809, 0x03455EFB,
    0x034614CA, 0x03463849, 0x034F8924, 0x03500A69, 0x035252D0, 0x035314E0, 0x0353324D, 0x03535ED0,
    0x035414E0, 0x0354324D, 0x03545ED0, 0x0354F4E8, 0x0355384D, 0x03555ED0, 0x0355F4DF, 0x03564350,
    0x035969A6, 0x035FFA52, 0x036649A6, 0x036FFA52, 0x037F4F66, 0x039D7492, 0x03BF6892, 0x03DF8A1F,
    0x04100B84, 0x04112107, 0x0411520D, 0x041214EA, 0x04124F20, 0x04131EDE, 0x04133F08, 0x04135809,
    0x0413F42B, 0x04142F08, 0x04200847, 0x042121FC, 0x04214209, 0x04221407, 0x0422203C, 0x04224209,
    0x04226350, 0x04231A7C, 0x04234209, 0x0423637B, 0x04241A7C, 0x04243220, 0x0424627B, 0x042514C8,
    0x04254809, 0x042FF8E9, 0x04311E47, 0x04313220, 0x0431527B, 0x043214FC, 0x04323220, 0x04326250,
    0x043315BC, 0x04333220, 0x0433527B, 0x04338413, 0x04341488, 0x04344809, 0x04346ED0, 0x04351F48,
    0x0435527B, 0x0435F9A5, 0x04363809, 0x04375EFB, 0x043FF929, 0x04412E79, 0x0441427B, 0x044219B9,
    0x04423809, 0x0442537B, 0x044314C8, 0x04432020, 0x0443527B, 0x044414CA, 0x04443809, 0x0444537B,
    0x04448993, 0x0445148A, 0x04453809, 0x04455ED0, 0x0445F4E5, 0x0446384D, 0x045009A6, 0x045272D3,
    0x045314A0, 0x0453324D, 0x04535ED0, 0x045415A0, 0x0454324D, 0x04545ED0, 0x04551F60, 0x0455324D,
    0x04562989, 0x04564350, 0x045FF4D2, 0x04665993, 0x047FFF62, 0x048FF725, 0x049F44BD, 0x04BFB7E5,
    0x04EF8A25, 0x04FFFB98, 0x051131F9, 0x051212C7, 0x05134209, 0x05200247, 0x05211007, 0x05213E60,
    0x052212C7, 0x05224209, 0x052319BC, 0x05233220, 0x0523527B, 0x052414C8, 0x05243820, 0x053112F9,
    0x05313E49, 0x05321439, 0x05323E49, 0x0532537B, 0x053314C8, 0x0533480D, 0x05337413, 0x05341488,
    0x0534527B, 0x0534F4EB, 0x05353809, 0x05356ED0, 0x0535F4E5, 0x0536427B, 0x054119B9, 0x054212F9,
    0x05423249, 0x05426ED3, 0x05431739, 0x05433249, 0x05435ED0, 0x0543F4EB, 0x05443809, 0x05445ED0,
    0x0544F4E8, 0x0545324D, 0x054FF992, 0x055362D3, 0x0553F5AB, 0x05544350, 0x055514CA, 0x0555427B,
    0x0555F4E5, 0x0556327B, 0x055FF4D2, 0x05665993, 0x05774F53, 0x059FF728, 0x05CC37FD, 0x05EFBA28,
    0x05FFFB98, 0x061131F9, 0x06121407, 0x06133E60, 0x061A72E4, 0x06211E47, 0x06214E4B, 0x062214C7,
    0x06223E60, 0x062312F9, 0x06233E60, 0x063112F9, 0x06313E4C, 0x063219B9, 0x06323E49, 0x06331439,
    0x06333809, 0x06336EE6, 0x0633F5AB, 0x06343809, 0x0634F42B, 0x0635427B, 0x063FF992, 0x064342FB,
    0x0643F4EB, 0x0644427B, 0x064524C9, 0x06655993, 0x0666170A, 0x066652E6, 0x067A6F56, 0x0698473D,
    0x06CF67D2, 0x06EF3A26, 0x06FFFAD8, 0x071131CC, 0x07211307, 0x07222E79, 0x072292DC, 0x07234E4B,
    0x073112F9, 0x07322339, 0x073632CB, 0x073FF992, 0x074432CB, 0x075549A6, 0x0776FF68, 0x07774350,
    0x0788473D, 0x07CF4516, 0x07EF3A26, 0x07FFFAD8, 0x08222E79, 0x083112F9, 0x0834330B, 0x0845338B,
    0x08756F5C, 0x0887F725, 0x08884366, 0x08AF649C, 0x08F00898, 0x08FFFAD8, 0x091111C7, 0x0932330B,
    0x0945338B, 0x09774F7D, 0x0998C725, 0x09996416, 0x09EF87E5, 0x09FFFAD8, 0x0A34330B, 0x0A45338B,
    0x0A77467D, 0x0AA9F52B, 0x0AAA6416, 0x0ABD67DF, 0x0AFFFA18, 0x0B33330B, 0x0B4443A6, 0x0B76467D,
    0x0BB9751F, 0x0BBB59BD, 0x0BEF5892, 0x0BFFFAD8, 0x0C221339, 0x0C53338E, 0x0C76367D, 0x0CCAF52E,
    0x0CCC6996, 0x0CFFFA18, 0x0D44438E, 0x0D64264E, 0x0DDCF52E, 0x0DDD5996, 0x0DFFFA18, 0x0E43338E,
    0x0E68465C, 0x0EEE651C, 0x0EFFFA18, 0x0F33238E, 0x0F553659, 0x0F8F451C, 0x0FAFF8AE, 0x0FF00A2E,
    0x0FFF1ACC, 0x0FFF33BD, 0x0FFF7522, 0x0FFFFAD8, 0x10002C72, 0x1111103E, 0x11121E83, 0x11131E9A,
    0x1121115A, 0x11221170, 0x112316F0, 0x1124175D, 0x11311CC2, 0x11321182, 0x11331D42, 0x11411D48,
    0x11421836, 0x11431876, 0x11441DF5, 0x1152287B, 0x12111903, 0x1212115A, 0x121316F0, 0x12211B30,
    0x12221B30, 0x12231B02, 0x12311184, 0x12321D04, 0x12331784, 0x12411D39, 0x12412020, 0x12422220,
    0x12511D89, 0x1252227B, 0x1258184A, 0x12832992, 0x1311171A, 0x13121B30, 0x1312202C, 0x131320AA,
    0x132120AA, 0x132220AD, 0x13232FED, 0x13312107, 0x13322134, 0x13332134, 0x13411D39, 0x13431E74,
    0x13441834, 0x134812B4, 0x1352230B, 0x13611E4B, 0x136522E4, 0x141113C2, 0x141211C4, 0x143121F9,
    0x143221F9, 0x143321CA, 0x14351D34, 0x14431E47, 0x14441E74, 0x144612B4, 0x1452230E, 0x14551E74,
    0x1471130E, 0x151113C2, 0x152121F9, 0x153121F9, 0x153221F9, 0x15331007, 0x15522E4E, 0x15551E74,
    0x1571130E, 0x161113C7, 0x162121F9, 0x163121F9, 0x16611E79, 0x16661334, 0x171113C7, 0x172121F9,
    0x17451E47, 0x1771130C, 0x181113C7, 0x18211E47, 0x18511E4C, 0x1882130C, 0x191113C7, 0x19331E79,
    0x1A111307, 0x1A311E79, 0x1F52230E, 0x200003C1, 0x20001027, 0x20004467, 0x200079E7, 0x2000E5EF,
    0x21100BC0, 0x211129C0, 0x21114011, 0x211189E7, 0x2111F5EF, 0x21124011, 0x21127455, 0x211325C0,
    0x21134011, 0x21137455, 0x211425C0, 0x21212440, 0x21213001, 0x2121F9EF, 0x21222540, 0x21226455,
    0x2122F5EF, 0x21233051, 0x2123F56F, 0x21244451, 0x21312551, 0x21323451, 0x21332551, 0x21844555,
    0x221125C0, 0x22113011, 0x2211F9EF, 0x22123051, 0x2212F9EF, 0x221329D1, 0x22212541, 0x22213011,
    0x2221F9EF, 0x22223451, 0x2222F9EF, 0x22232551, 0x2223F56F, 0x22312551, 0x223229D1, 0x2232F56F,
    0x22332551, 0x2233F56F, 0x22875555, 0x22DAB5D7, 0x23112BD1, 0x23115467, 0x231225D1, 0x232129D1,
    0x232229D1, 0x2322F9EF, 0x23233451, 0x2323F9EF, 0x23312551, 0x233229D1, 0x2332F9EF, 0x2333F56F,
    0x237FF557, 0x238569D5, 0x23D955D7, 0x24100BE7, 0x248789E7, 0x24E315D7, 0x24FFFBEF, 0x259869E7,
    0x25DFF5EF, 0x25FFFBEF, 0x268789E7, 0x26DFA5D7, 0x26FFFBEF, 0x279649E7, 0x27E425D7, 0x27FFFBEF,
    0x288879E7, 0x28EFF5EF, 0x28FFFBEF, 0x298439E7, 0x29F115EF, 0x29FFFBEF, 0x2A7659E7, 0x2AEF75D7,
    0x2AFFFBEF, 0x2B7C89E7, 0x2BEF95D7, 0x2BFFFBEF, 0x2C6659E7, 0x2CD555D7, 0x2CFFFBEF, 0x2D6329E7,
    0x2DDD55E7, 0x2DFFFBEB, 0x2E8479D7, 0x2EEE35E7, 0x2EFFFBEF, 0x2F5459E7, 0x2FCF85D7, 0x2FFEFBEB,
    0x2FFFA5EF, 0x2FFFEBEF, 0x30001AE7, 0x30002001, 0x311129C0, 0x31221015, 0x31232000, 0x31332451,
    0x32112540, 0x32131027, 0x32212440, 0x33452455, 0x4000F9D7, 0x4122F9D7, 0x43F65555, 0x43FFF5D7,
    0x44F55567, 0x44FFF5D7, 0x45F00557, 0x45FFF5D7, 0x46F659D7, 0x471005E7, 0x47F449E7, 0x481005E7,
    0x48EFA9D5, 0x48FFF5EF, 0x49F449E7, 0x49FFF5EF, 0x4AEA79E7, 0x4AFFF5EF, 0x4BE9C9D5, 0x4BFFF5EF,
    0x4CE549E7, 0x4CFFF5EF, 0x4DE359E7, 0x4DFFF5D7, 0x4EE469E7, 0x4EFFF5D7, 0x4FEF39E7, 0x4FFFF5EF,
    0x6000F9E7, 0x69FFF557, 0x6FFFF9D7, 0x811009D7, 0x8EFFF555, 0xFFFFF9E7,
];

static ARI_PK: [[i16; 17]; 64] = [
    [708, 706, 579, 569, 568, 567, 479, 469, 297, 138, 97, 91, 72, 52, 38, 34, 0],
    [
        7619, 6917, 6519, 6412, 5514, 5003, 4683, 4563, 3907, 3297, 3125, 3060, 2904, 2718, 2631,
        2590, 0,
    ],
    [7263, 4888, 4810, 4803, 1889, 415, 335, 327, 195, 72, 52, 49, 36, 20, 15, 14, 0],
    [3626, 2197, 2188, 2187, 582, 57, 47, 46, 30, 12, 9, 8, 6, 4, 3, 2, 0],
    [7806, 5541, 5451, 5441, 2720, 834, 691, 674, 487, 243, 179, 167, 139, 98, 77, 70, 0],
    [6684, 4101, 4058, 4055, 1748, 426, 368, 364, 322, 257, 235, 232, 228, 222, 217, 215, 0],
    [9162, 5964, 5831, 5819, 3269, 866, 658, 638, 535, 348, 258, 244, 234, 214, 195, 186, 0],
    [10638, 8491, 8365, 8351, 4418, 2067, 1859, 1834, 1190, 601, 495, 478, 356, 217, 174, 164, 0],
    [
        13389, 10514, 10032, 9961, 7166, 3488, 2655, 2524, 2015, 1140, 760, 672, 585, 426, 325,
        283, 0,
    ],
    [
        14861, 12788, 12115, 11952, 9987, 6657, 5323, 4984, 4324, 3001, 2205, 1943, 1764, 1394,
        1115, 978, 0,
    ],
    [
        12876, 10004, 9661, 9610, 7107, 3435, 2711, 2595, 2257, 1508, 1059, 952, 893, 753, 609,
        538, 0,
    ],
    [
        15125, 13591, 13049, 12874, 11192, 8543, 7406, 7023, 6291, 4922, 4104, 3769, 3465, 2890,
        2486, 2275, 0,
    ],
    [
        14574, 13106, 12731, 12638, 10453, 7947, 7233, 7037, 6031, 4618, 4081, 3906, 3465, 2802,
        2476, 2349, 0,
    ],
    [
        15070, 13179, 12517, 12351, 10742, 7657, 6200, 5825, 5264, 3998, 3014, 2662, 2510, 2153,
        1799, 1564, 0,
    ],
    [
        15542, 14466, 14007, 13844, 12489, 10409, 9481, 9132, 8305, 6940, 6193, 5867, 5458, 4743,
        4291, 4047, 0,
    ],
    [
        15165, 14384, 14084, 13934, 12911, 11485, 10844, 10513, 10002, 8993, 8380, 8051, 7711,
        7036, 6514, 6233, 0,
    ],
    [
        15642, 14279, 13625, 13393, 12348, 9971, 8405, 7858, 7335, 6119, 4918, 4376, 4185, 3719,
        3231, 2860, 0,
    ],
    [
        13408, 13407, 11471, 11218, 11217, 11216, 9473, 9216, 6480, 3689, 2857, 2690, 2256, 1732,
        1405, 1302, 0,
    ],
    [
        16098, 15584, 15191, 14931, 14514, 13578, 12703, 12103, 11830, 11172, 10475, 9867, 9695,
        9281, 8825, 8389, 0,
    ],
    [
        15844, 14873, 14277, 13996, 13230, 11535, 10205, 9543, 9107, 8086, 7085, 6419, 6214, 5713,
        5195, 4731, 0,
    ],
    [
        16131, 15720, 15443, 15276, 14848, 13971, 13314, 12910, 12591, 11874, 11225, 10788, 10573,
        10077, 9585, 9209, 0,
    ],
    [
        16331, 16330, 12283, 11435, 11434, 11433, 8725, 8049, 6065, 4138, 3187, 2842, 2529, 2171,
        1907, 1745, 0,
    ],
    [
        16011, 15292, 14782, 14528, 14008, 12767, 11556, 10921, 10591, 9759, 8813, 8043, 7855,
        7383, 6863, 6282, 0,
    ],
    [
        16380, 16379, 15159, 14610, 14609, 14608, 12859, 12111, 11046, 9536, 8348, 7713, 7216,
        6533, 5964, 5546, 0,
    ],
    [
        16367, 16333, 16294, 16253, 16222, 16143, 16048, 15947, 15915, 15832, 15731, 15619, 15589,
        15512, 15416, 15310, 0,
    ],
    [
        15967, 15319, 14937, 14753, 14010, 12638, 11787, 11360, 10805, 9706, 8934, 8515, 8166,
        7456, 6911, 6575, 0,
    ],
    [4906, 3005, 2985, 2984, 875, 102, 83, 81, 47, 17, 12, 11, 8, 5, 4, 3, 0],
    [7217, 4346, 4269, 4264, 1924, 428, 340, 332, 280, 203, 179, 175, 171, 164, 159, 157, 0],
    [
        16010, 15415, 15032, 14805, 14228, 13043, 12168, 11634, 11265, 10419, 9645, 9110, 8892,
        8378, 7850, 7437, 0,
    ],
    [8573, 5218, 5046, 5032, 2787, 771, 555, 533, 443, 286, 218, 205, 197, 181, 168, 162, 0],
    [11474, 8095, 7822, 7796, 4632, 1443, 1046, 1004, 748, 351, 218, 194, 167, 121, 93, 83, 0],
    [
        16152, 15764, 15463, 15264, 14925, 14189, 13536, 13070, 12846, 12314, 11763, 11277, 11131,
        10777, 10383, 10011, 0,
    ],
    [
        14187, 11654, 11043, 10919, 8498, 4885, 3778, 3552, 2947, 1835, 1283, 1134, 998, 749, 585,
        514, 0,
    ],
    [
        14162, 11527, 10759, 10557, 8601, 5417, 4105, 3753, 3286, 2353, 1708, 1473, 1370, 1148,
        959, 840, 0,
    ],
    [
        16205, 15902, 15669, 15498, 15213, 14601, 14068, 13674, 13463, 12970, 12471, 12061, 11916,
        11564, 11183, 10841, 0,
    ],
    [
        15043, 12972, 12092, 11792, 10265, 7446, 5934, 5379, 4883, 3825, 3036, 2647, 2507, 2185,
        1901, 1699, 0,
    ],
    [
        15320, 13694, 12782, 12352, 11191, 8936, 7433, 6671, 6255, 5366, 4622, 4158, 4020, 3712,
        3420, 3198, 0,
    ],
    [
        16255, 16020, 15768, 15600, 15416, 14963, 14440, 14006, 13875, 13534, 13137, 12697, 12602,
        12364, 12084, 11781, 0,
    ],
    [
        15627, 14503, 13906, 13622, 12557, 10527, 9269, 8661, 8117, 6933, 5994, 5474, 5222, 4664,
        4166, 3841, 0,
    ],
    [
        16366, 16365, 14547, 14160, 14159, 14158, 11969, 11473, 8735, 6147, 4911, 4530, 3865, 3180,
        2710, 2473, 0,
    ],
    [
        16257, 16038, 15871, 15754, 15536, 15071, 14673, 14390, 14230, 13842, 13452, 13136, 13021,
        12745, 12434, 12154, 0,
    ],
    [
        15855, 14971, 14338, 13939, 13239, 11782, 10585, 9805, 9444, 8623, 7846, 7254, 7079, 6673,
        6262, 5923, 0,
    ],
    [9492, 6318, 6197, 6189, 3004, 652, 489, 477, 333, 143, 96, 90, 78, 60, 50, 47, 0],
    [
        16313, 16191, 16063, 15968, 15851, 15590, 15303, 15082, 14968, 14704, 14427, 14177, 14095,
        13899, 13674, 13457, 0,
    ],
    [8485, 5473, 5389, 5383, 2411, 494, 386, 377, 278, 150, 117, 112, 103, 89, 81, 78, 0],
    [10497, 7154, 6959, 6943, 3788, 1004, 734, 709, 517, 238, 152, 138, 120, 90, 72, 66, 0],
    [
        16317, 16226, 16127, 16040, 15955, 15762, 15547, 15345, 15277, 15111, 14922, 14723, 14671,
        14546, 14396, 14239, 0,
    ],
    [
        16382, 16381, 15858, 15540, 15539, 15538, 14704, 14168, 13768, 13092, 12452, 11925, 11683,
        11268, 10841, 10460, 0,
    ],
    [5974, 3798, 3758, 3755, 1275, 205, 166, 162, 95, 35, 26, 24, 18, 11, 8, 7, 0],
    [3532, 2258, 2246, 2244, 731, 135, 118, 115, 87, 45, 36, 34, 29, 21, 17, 16, 0],
    [7466, 4882, 4821, 4811, 2476, 886, 788, 771, 688, 531, 469, 457, 437, 400, 369, 361, 0],
    [9580, 5772, 5291, 5216, 3444, 1496, 1025, 928, 806, 578, 433, 384, 366, 331, 296, 273, 0],
    [10692, 7730, 7543, 7521, 4679, 1746, 1391, 1346, 1128, 692, 495, 458, 424, 353, 291, 268, 0],
    [11040, 7132, 6549, 6452, 4377, 1875, 1253, 1130, 958, 631, 431, 370, 346, 296, 253, 227, 0],
    [12687, 9332, 8701, 8585, 6266, 3093, 2182, 2004, 1683, 1072, 712, 608, 559, 458, 373, 323, 0],
    [
        13429, 9853, 8860, 8584, 6806, 4039, 2862, 2478, 2239, 1764, 1409, 1224, 1178, 1077, 979,
        903, 0,
    ],
    [
        14685, 12163, 11061, 10668, 9101, 6345, 4871, 4263, 3908, 3200, 2668, 2368, 2285, 2106,
        1942, 1819, 0,
    ],
    [
        13295, 11302, 10999, 10945, 7947, 5036, 4490, 4385, 3391, 2185, 1836, 1757, 1424, 998, 833,
        785, 0,
    ],
    [4992, 2993, 2972, 2970, 1269, 575, 552, 549, 530, 505, 497, 495, 493, 489, 486, 485, 0],
    [
        15419, 13862, 13104, 12819, 11429, 8753, 7220, 6651, 6020, 4667, 3663, 3220, 2995, 2511,
        2107, 1871, 0,
    ],
    [12468, 9263, 8912, 8873, 5758, 2193, 1625, 1556, 1187, 589, 371, 330, 283, 200, 149, 131, 0],
    [
        15870, 15076, 14615, 14369, 13586, 12034, 10990, 10423, 9953, 8908, 8031, 7488, 7233, 6648,
        6101, 5712, 0,
    ],
    [1693, 978, 976, 975, 194, 18, 16, 15, 11, 7, 6, 5, 4, 3, 2, 1, 0],
    [7992, 5218, 5147, 5143, 2152, 366, 282, 276, 173, 59, 38, 35, 27, 16, 11, 10, 0],
];
static ARI_LSB2: [[i16; 4]; 3] =
    [[12571, 10569, 3696, 0], [12661, 5700, 3751, 0], [10827, 6884, 2929, 0]];

/// The arithmetic decoder of [`decode2`] for x86, x86-64 and AArch64 (the other targets use the
/// straight port, `decode2_generic`). It computes exactly the same symbols, states and bitstream
/// positions, with a shorter dependency chain per symbol:
/// - The symbol search of `ari_decode_14bits()` keeps the test of the first table entry (symbol 0
///   is the most frequent one) and replaces the remaining binary search, which the compiler turns
///   into a serial chain of conditional moves, by a vectorized count of the entries above the
///   threshold (SSE2 or NEON, exact 16 bit products).
/// - The renormalization is simplified algebraically (see `ari_renormalize`).
/// - The entries of the context table `pk_table` that the next tuple can use are loaded one tuple
///   ahead.
/// - With BMI1/BMI2/LZCNT (x86-64-v3, selected at run time), the bit operations of the
///   renormalization and of the bit reader are single instructions.
#[cfg(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64"))]
mod fast {
    use super::*;
    #[cfg(target_arch = "aarch64")]
    use core::arch::aarch64::*;
    #[cfg(target_arch = "x86")]
    use core::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    use core::arch::x86_64::*;

    /// The cumulative frequency row `row` with the entry 16384 prepended: `high` is then updated
    /// without a branch for symbol 0 (`(range * 16384) >> 14 == range` leaves it unchanged).
    const fn with_full_range<const N: usize, const M: usize>(row: [i16; N]) -> [i16; M] {
        let mut out = [0i16; M];
        out[0] = 1 << stat_bitsnew;
        let mut i = 0;
        while i < N {
            out[i + 1] = row[i];
            i += 1;
        }
        out
    }

    /// The symbol index of a strictly decreasing cumulative frequency row for
    /// `1 <= range <= 0xFFFF`: the number of its first 16 entries `cf` with `cf * range > c` (the
    /// comparison results form a prefix of ones, as the row is monotone). The products are below
    /// `2^30` and computed exactly from the 16 bit halves (`pmullw`/`pmulhuw`).
    #[cfg(any(target_arch = "x86_64", all(target_arch = "x86", target_feature = "sse2")))]
    #[inline(always)]
    fn count_gt16(row: &[i16; 17], c: i32, range: i32) -> usize {
        debug_assert!((1..=0xFFFF).contains(&range));
        // SAFETY: the row has 17 entries (16 are loaded); SSE2 is part of the x86-64 baseline
        // and enabled for this 32 bit x86 target.
        unsafe {
            let r = _mm_set1_epi16(range as i16);
            let cv = _mm_set1_epi32(c);
            let lo = _mm_loadu_si128(row.as_ptr() as *const __m128i);
            let hi = _mm_loadu_si128(row.as_ptr().add(8) as *const __m128i);
            let (l0, l1) = (_mm_mullo_epi16(lo, r), _mm_mulhi_epu16(lo, r));
            let (h0, h1) = (_mm_mullo_epi16(hi, r), _mm_mulhi_epu16(hi, r));
            let p0 = _mm_unpacklo_epi16(l0, l1);
            let p1 = _mm_unpackhi_epi16(l0, l1);
            let p2 = _mm_unpacklo_epi16(h0, h1);
            let p3 = _mm_unpackhi_epi16(h0, h1);
            let m = _mm_packs_epi16(
                _mm_packs_epi32(_mm_cmpgt_epi32(p0, cv), _mm_cmpgt_epi32(p1, cv)),
                _mm_packs_epi32(_mm_cmpgt_epi32(p2, cv), _mm_cmpgt_epi32(p3, cv)),
            );
            (!(_mm_movemask_epi8(m) as u32)).trailing_zeros() as usize
        }
    }

    /// [`count_gt16`] for 32 bit x86 targets without SSE2.
    #[cfg(all(target_arch = "x86", not(target_feature = "sse2")))]
    #[inline(always)]
    fn count_gt16(row: &[i16; 17], c: i32, range: i32) -> usize {
        row[..16].iter().filter(|&&cf| cf as i32 * range > c).count()
    }

    /// [`count_gt16`] for AArch64: the products are computed exactly by `umull` (the entries and
    /// the range are below `2^16`, the products below `2^30`), and the comparison results (-1
    /// or 0 per lane) are added up.
    #[cfg(target_arch = "aarch64")]
    #[inline(always)]
    fn count_gt16(row: &[i16; 17], c: i32, range: i32) -> usize {
        debug_assert!((1..=0xFFFF).contains(&range));
        // SAFETY: the row has 17 entries (16 are loaded); NEON is part of the AArch64 baseline.
        unsafe {
            let r = vdupq_n_u16(range as u16);
            let cv = vdupq_n_s32(c);
            let lo = vld1q_u16(row.as_ptr() as *const u16);
            let hi = vld1q_u16(row.as_ptr().add(8) as *const u16);
            let gt = |p: uint32x4_t| vcgtq_s32(vreinterpretq_s32_u32(p), cv);
            let m0 = gt(vmull_u16(vget_low_u16(lo), vget_low_u16(r)));
            let m1 = gt(vmull_high_u16(lo, r));
            let m2 = gt(vmull_u16(vget_low_u16(hi), vget_low_u16(r)));
            let m3 = gt(vmull_high_u16(hi, r));
            vaddvq_u32(vaddq_u32(vaddq_u32(m0, m1), vaddq_u32(m2, m3))).wrapping_neg() as usize
        }
    }

    /// `ari_decode_14bits()` for a 17 symbol table: the search of the C++ code compares the entries
    /// 0..16 (the result is at most 16). The first entry is checked first (the symbol is 0 in about
    /// half of the cases); the remaining binary search is replaced by a vectorized count.
    #[inline(always)]
    fn decode_msb(hbs: &mut FdkBitStream, s: &mut Tastat, row: &[i16; 17]) -> i32 {
        let low = s.low;
        let mut high = s.high;
        let value = s.vobf;
        let range = high - low + 1;
        let c = ((value - low + 1) << stat_bitsnew) - 1;
        let f = |i: usize| row[i] as i32 * range;
        let mut symbol = 0;
        if f(0) > c {
            if (1..=0xFFFF).contains(&range) {
                symbol = count_gt16(row, c, range);
            } else {
                // The range is 0x10000 for the first symbol of a block (and anything else only
                // with an invalid state): the search of the C++ code.
                symbol = 1;
                if f(symbol + 7) > c {
                    symbol += 8;
                }
                if f(symbol + 3) > c {
                    symbol += 4;
                }
                if f(symbol + 1) > c {
                    symbol += 2;
                }
                if f(symbol) > c {
                    symbol += 1;
                }
            }
            high = low + mul_sbc_14bits(range, row[symbol - 1] as i32) - 1;
        }
        let low = low + mul_sbc_14bits(range, row[symbol] as i32);
        ari_renormalize(hbs, s, low as u16 as u32, high as u16 as u32, value as u32);
        symbol as i32
    }

    /// `ari_decode_14bits()` for a 4 symbol table (`row` with 16384 prepended): the branchless form
    /// of the search of the C++ code, which compares the entries 0..3 (the result is at most 3).
    #[inline(always)]
    fn decode_lsb(hbs: &mut FdkBitStream, s: &mut Tastat, row: &[i16; 5]) -> i32 {
        let low = s.low;
        let high = s.high;
        let value = s.vobf;
        let range = high - low + 1;
        let c = ((value - low + 1) << stat_bitsnew) - 1;
        let f = |i: usize| row[i + 1] as i32 * range;
        // The row is non-increasing, so the comparisons form a prefix of ones.
        let symbol = (f(0) > c) as usize + (f(1) > c) as usize + (f(2) > c) as usize;
        let high = low + mul_sbc_14bits(range, row[symbol] as i32) - 1;
        let low = low + mul_sbc_14bits(range, row[symbol + 1] as i32);
        ari_renormalize(hbs, s, low as u16 as u32, high as u16 as u32, value as u32);
        symbol as i32
    }

    /// [`super::ari_renormalize`] with the toggles applied after the shift: toggling bits
    /// `14 .. 15 - n3` and shifting by `n3` leaves only the toggle of bit 15 within the 16 LSBs.
    #[inline(always)]
    pub(super) fn ari_renormalize(
        hbs: &mut FdkBitStream,
        s: &mut Tastat,
        low: u32,
        high: u32,
        value: u32,
    ) {
        let n1 = (!((!high | low) << 16)).leading_zeros(); /* 0..=16 */
        // After the first n1 shifts the bits 14 .. 15 - n3 of `low & ~high` are set:
        // `low1 & ~high1` of the derivation above is `(low & ~high) << n1` (the shifted-in bits of
        // `low` are 0).
        let n3 = (!(((low & !high) << n1) << 17)).leading_zeros(); /* 0..=15 */
        let n = n1 + n3;
        // The toggle of bit 15 applies if and only if n3 > 0.
        let t15 = ((n3 != 0) as u32) << 15;
        let bits = hbs.read_bits_short(n);
        s.low = (((low << n) & 0xFFFF) ^ t15) as i32;
        s.high = ((((high << n) | ((1 << n) - 1)) & 0xFFFF) ^ t15) as i32;
        s.vobf = ((((value << n) | bits) & 0xFFFF) ^ t15) as i32;
    }

    pub fn decode2(
        bbuf: &mut FdkBitStream,
        c_prev: &mut [u8],
        p_spectral_coefficient: &mut [FixpDbl],
        n: i32,
        nt: i32,
    ) -> ArithCodingError {
        #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
        if mpeghdec_fdk::simd::x86_v3() {
            // The same code with the BMI1/BMI2/LZCNT instructions (`bzhi`, `shlx`, `lzcnt`), which
            // shorten the dependency chain of the renormalization.
            #[target_feature(enable = "avx2,bmi1,bmi2,lzcnt,popcnt")]
            unsafe fn v3(
                bbuf: &mut FdkBitStream,
                c_prev: &mut [u8],
                p_spectral_coefficient: &mut [FixpDbl],
                n: i32,
                nt: i32,
            ) -> ArithCodingError {
                decode2_impl(bbuf, c_prev, p_spectral_coefficient, n, nt)
            }
            // SAFETY: the features are available.
            return unsafe { v3(bbuf, c_prev, p_spectral_coefficient, n, nt) };
        }
        decode2_impl(bbuf, c_prev, p_spectral_coefficient, n, nt)
    }

    #[inline(always)]
    pub(super) fn decode2_impl(
        bbuf: &mut FdkBitStream,
        c_prev: &mut [u8],
        p_spectral_coefficient: &mut [FixpDbl],
        n: i32,
        nt: i32,
    ) -> ArithCodingError {
        let error_status = ARITH_CODER_OK;
        let mut r: i32;

        let mut c_3: i32 = 0; /* context of current frame 3 time steps ago */
        let mut c_2: i32 = 0; /* context of current frame 2 time steps ago */
        let mut c_1: i32 = 0; /* context of current frame 1 time steps ago */
        let mut c_0: i32; /* context of current frame to be calculated */

        /* ari_start_decoding_14bits */
        let mut as_ = Tastat {
            low: 0,
            high: ari_q4new,
            vobf: bbuf.read_bits(cbitsnew as u32) as i32,
            bits_to_follow: 0,
        };

        /* arith_map_context */
        let mut state_inc: u16 = ((c_prev[0] as i32) << 12) as u16;
        let pk_table = pk_table();

        // The loop below accesses c_prev[..n + 1] and p_spectral_coefficient[..2 * n].
        let nu = n.max(0) as usize;
        let (c_prev_n, spec_n) = (&mut c_prev[..nu + 1], &mut p_spectral_coefficient[..2 * nu]);

        // The context `s` of a tuple is `base + c_1` (plus 0x10000 for small amplitudes), where
        // `base` only depends on the previous tuple's context and the previous frame: the 16
        // possible table entries (and the 16 for the small amplitude flag) are loaded as soon as
        // `base` is known, one tuple ahead, and the entry is selected from registers (the loads of
        // the 128 KiB table would otherwise wait for the decoded amplitude).
        // (Contexts beyond the table, which only invalid previous contexts produce, use the hash
        // function like the C++ code.)
        let pk_chunks = |base: usize| -> ([u64; 4], bool) {
            if base + 0x10010 > PK_TABLE_LEN {
                return ([0; 4], false);
            }
            let w = |p: usize| u64::from_le_bytes(pk_table[p..p + 8].try_into().unwrap());
            ([w(base), w(base + 8), w(base + 0x10000), w(base + 0x10008)], true)
        };
        let (mut chunks, mut chunks_ok) =
            pk_chunks(((state_inc >> 8) as usize + ((c_prev_n[1.min(nu)] as usize) << 8)) << 4);

        let mut i: i32 = 0;
        while i < n {
            let iu = i as usize;
            /* arith_get_context */
            let mut s: u32 = (state_inc >> 8) as u32;
            s = s.wrapping_add((c_prev_n[iu + 1] as u32) << 8);
            s = (s << 4).wrapping_add(c_1 as u32);

            state_inc = s as u16;

            /* Cumulative amplitude below 2 */
            let small = i > 3 && (c_1 + c_2 + c_3) < 5;
            if small {
                s = s.wrapping_add(0x10000);
            }

            // pk_table[s] from the chunks loaded for this tuple, then the loads for the next one.
            let (lo, hi) = if small { (chunks[2], chunks[3]) } else { (chunks[0], chunks[1]) };
            let word = if c_1 < 8 { lo } else { hi };
            let mut pki = (word >> (8 * (c_1 & 7))) as u32 & 0xFF;
            if !chunks_ok {
                pki = match pk_table.get(s as usize) {
                    Some(&pki) => pki as u32,
                    None => get_pk_v2(s),
                };
            }
            debug_assert_eq!(
                pki as u8,
                pk_table.get(s as usize).copied().unwrap_or(get_pk_v2(s) as u8)
            );
            (chunks, chunks_ok) = pk_chunks(
                ((state_inc >> 8) as usize + ((c_prev_n[(iu + 2).min(nu)] as usize) << 8)) << 4,
            );

            /* MSBs decoding */
            let mut lev: i32 = 0;
            let mut esc_nb: i32 = 0;
            loop {
                r = decode_msb(bbuf, &mut as_, &ARI_PK[pki as usize & 0x3F]);
                if r < VAL_ESC {
                    break;
                }

                lev += 1;

                if lev > 23 {
                    return ARITH_CODER_ERROR;
                }

                if esc_nb < 7 {
                    esc_nb += 1;
                }
                let s_esc = s.wrapping_add((esc_nb << (VAL_ESC + 1)) as u32);
                pki = match pk_table.get(s_esc as usize) {
                    Some(&pki) => pki as u32,
                    None => get_pk_v2(s_esc),
                };
            }

            /* Stop symbol */
            if r == 0 {
                if esc_nb > 0 {
                    break; /* Stop symbol */
                }
                c_0 = 1;
            } else {
                let mut b = r >> 2;
                let mut a = r & 0x3;

                /* LSBs decoding */
                for _ in 0..lev {
                    let pidx = if a == 0 {
                        1
                    } else if b == 0 {
                        0
                    } else {
                        2
                    };
                    r = decode_lsb(bbuf, &mut as_, &ARI_LSB2_X[pidx]);
                    a = (a << 1) | (r & 1);
                    b = (b << 1) | (r >> 1);
                }

                spec_n[2 * iu] = a;
                spec_n[2 * iu + 1] = b;

                c_0 = a + b + 1;
                if c_0 > 0xF {
                    c_0 = 0xF;
                }
            }

            /* arith_update_context */
            c_3 = c_2;
            c_2 = c_1;
            c_1 = c_0;
            c_prev_n[iu] = c_0 as u8;

            i += 1;
        }

        bbuf.push_back((cbitsnew - 2) as u32);

        /* We need to run only from 0 to i-1 since all other q[i][1].a,b will be cleared later */
        let j = i as usize;
        // Branchless sign decoding, see `decode2_generic`.
        for q in spec_n[..2 * j].chunks_exact_mut(2) {
            let bits = (q[0] != 0) as u32 + (q[1] != 0) as u32;
            let r = bbuf.read_bits(bits);
            if (r >> (bits.wrapping_sub(1) & 31)) & 1 == 0 {
                q[0] = -q[0];
            }
            if r & 1 == 0 {
                q[1] = -q[1];
            }
        }

        c_prev[j..nt as usize].fill(1);

        error_status
    }

    static ARI_LSB2_X: [[i16; 5]; 3] = [
        with_full_range::<4, 5>(ARI_LSB2[0]),
        with_full_range::<4, 5>(ARI_LSB2[1]),
        with_full_range::<4, 5>(ARI_LSB2[2]),
    ];
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The renormalization loop of the C code (one bit per iteration).
    fn renormalize_ref(hbs: &mut FdkBitStream, s: &mut Tastat, low: u16, high: u16, value: i32) {
        let (mut us_low, mut us_high, mut value) = (low, high, value);
        loop {
            if us_high & 0x8000 != 0 {
                if us_low & 0x8000 == 0 {
                    if us_low & 0x4000 != 0 && us_high & 0x4000 == 0 {
                        us_low -= 0x4000;
                        us_high -= 0x4000;
                        value = value.wrapping_sub(0x4000);
                    } else {
                        break;
                    }
                }
            }
            us_low <<= 1;
            us_high = (us_high << 1) | 1;
            value = (value << 1) | hbs.read_bit() as i32;
        }
        s.low = us_low as i32;
        s.high = us_high as i32;
        s.vobf = value & 0xFFFF;
    }

    #[test]
    fn renormalize_equals_loop() {
        let mut buf = [0u8; 256];
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        for b in buf.iter_mut() {
            *b = next() as u8;
        }
        let mut bs_ref = FdkBitStream::reader(&buf, 256 * 8);
        let mut bs = bs_ref.clone();
        let edges =
            [0u32, 1, 0x3FFF, 0x4000, 0x4001, 0x7FFF, 0x8000, 0x8001, 0xBFFF, 0xC000, 0xFFFF];
        for k in 0..2_000_000u32 {
            let r = next();
            let (low, high) = if (k as usize) < edges.len() * edges.len() {
                (edges[k as usize % edges.len()], edges[k as usize / edges.len()])
            } else {
                // Random states, with a bias towards long runs of renormalization steps.
                let m = (1u32 << (r >> 60)) - 1;
                ((r as u32 & 0xFFFF) & !m, ((r >> 16) as u32 & 0xFFFF) | m)
            };
            let value = (r >> 32) as u32;
            let (mut s_ref, mut s) = (Tastat::default(), Tastat::default());
            renormalize_ref(&mut bs_ref, &mut s_ref, low as u16, high as u16, value as i32);
            #[cfg(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64"))]
            {
                let (mut bs_fast, mut s_fast) = (bs.clone(), Tastat::default());
                fast::ari_renormalize(&mut bs_fast, &mut s_fast, low, high, value);
                ari_renormalize(&mut bs, &mut s, low, high, value);
                assert_eq!(s_fast, s, "fast: low {low:#x} high {high:#x} value {value:#x}");
                assert_eq!(
                    (bs_fast.cache_word, bs_fast.bits_in_cache, bs_fast.h_bit_buf.bit_ndx()),
                    (bs.cache_word, bs.bits_in_cache, bs.h_bit_buf.bit_ndx())
                );
            }
            #[cfg(not(any(
                target_arch = "x86",
                target_arch = "x86_64",
                target_arch = "aarch64"
            )))]
            ari_renormalize(&mut bs, &mut s, low, high, value);
            assert_eq!(s, s_ref, "low {low:#x} high {high:#x} value {value:#x}");
            assert_eq!(
                (bs.cache_word, bs.bits_in_cache, bs.h_bit_buf.bit_ndx()),
                (bs_ref.cache_word, bs_ref.bits_in_cache, bs_ref.h_bit_buf.bit_ndx())
            );
        }
    }

    #[cfg(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64"))]
    type DecodeFn<'a> =
        dyn Fn(&mut FdkBitStream, &mut [u8], &mut [FixpDbl]) -> ArithCodingError + 'a;

    /// The x86 / AArch64 decoder (on x86 with and without the x86-64-v3 instructions) against the
    /// straight port, on random bitstreams and contexts (including invalid previous contexts).
    #[cfg(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64"))]
    #[test]
    fn fast_decode2_equals_generic() {
        let mut x = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = move || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        let iters = if cfg!(debug_assertions) { 2_000 } else { 20_000 };
        for iter in 0..iters {
            let mut buf = vec![0u8; 4096];
            // Sparse bitstreams decode long runs of small values; random ones escapes and errors.
            let density = next() % 4;
            for b in buf.iter_mut() {
                let r = next();
                *b = if density == 0 { (r & (r >> 8) & (r >> 16)) as u8 } else { r as u8 };
            }
            let nt = (next() % 513) as usize;
            let n = (next() % (nt as u64 + 1)) as i32;
            let mut c_prev = [0u8; C_PREV_LEN - 2];
            let invalid = next() % 8 == 0;
            for c in c_prev.iter_mut() {
                let r = next();
                *c = if invalid && r % 16 == 0 { (r >> 8) as u8 } else { (r >> 8) as u8 & 0xF };
            }
            let spec = vec![0 as FixpDbl; 1024];
            let run = |f: &DecodeFn<'_>| {
                let b = buf.clone();
                let mut bs = FdkBitStream::reader(&b, 4096 * 8);
                let (mut cp, mut sp) = (c_prev, spec.clone());
                let r = f(&mut bs, &mut cp, &mut sp);
                (r, cp, sp, bs.cache_word, bs.bits_in_cache, bs.h_bit_buf.bit_ndx())
            };
            let nt_i = nt as i32;
            let generic = run(&|bs, cp, sp| decode2_generic(bs, cp, sp, n, nt_i));
            let fast = run(&|bs, cp, sp| fast::decode2(bs, cp, sp, n, nt_i));
            let base = run(&|bs, cp, sp| fast::decode2_impl(bs, cp, sp, n, nt_i));
            assert!(fast == generic, "iteration {iter}: fast decoder differs");
            assert!(base == generic, "iteration {iter}: fast baseline decoder differs");
        }
    }
}
