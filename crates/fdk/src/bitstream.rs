// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_bitstream.h.

//! Cached bitstream reader/writer on top of [`FdkBitBuf`].

use crate::digest::StateDigest;

use crate::bitbuffer::{BitMem, FdkBitBuf, BIT_MASK, MAX_BUFSIZE_BYTES};

pub const CACHE_BITS: u32 = 32;
pub const BUFSIZE_DUMMY_VALUE: u32 = MAX_BUFSIZE_BYTES;

/// `FDK_BS_CFG`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u32)]
pub enum FdkBsCfg {
    #[default]
    BsReader = 0,
    BsWriter = 1,
}

/// `FDK_BITSTREAM`: a bit reader or writer over a [`BitMem`] (see [`FdkBitBuf`]). Clones share
/// the memory, like the struct assignments of C that save and restore a bitstream position.
#[derive(StateDigest, Clone, Debug, Default)]
pub struct FdkBitStream {
    pub cache_word: u32,
    pub bits_in_cache: u32,
    pub h_bit_buf: FdkBitBuf,
    pub config_cache: FdkBsCfg,
}

/// `HANDLE_FDK_BITSTREAM`
pub type HandleFdkBitStream<'a> = &'a mut FdkBitStream;

impl FdkBitStream {
    /// `FDKcreateBitStream(pBuffer, bufSize, config)`: a bitstream over the first `buf_size` bytes
    /// (a power of two) of `mem`.
    pub fn create(mem: &BitMem, buf_size: u32, config: FdkBsCfg) -> Self {
        let mut bs = FdkBitStream::default();
        bs.h_bit_buf.init(mem, buf_size, 0);
        bs.cache_word = 0;
        bs.bits_in_cache = 0;
        bs.config_cache = config;
        bs
    }

    /// `FDKinitBitStream(hBitStream, pBuffer, bufSize, validBits, config)`
    pub fn init(&mut self, mem: &BitMem, buf_size: u32, valid_bits: u32, config: FdkBsCfg) {
        self.h_bit_buf.init(mem, buf_size, valid_bits);
        self.cache_word = 0;
        self.bits_in_cache = 0;
        self.config_cache = config;
    }

    /// Convenience constructor: a reader over a copy of `data` (whose length must be a power of
    /// two) with `valid_bits` readable bits.
    pub fn reader(data: &[u8], valid_bits: u32) -> Self {
        let mut bs = FdkBitStream::default();
        bs.init(&BitMem::from_slice(data), data.len() as u32, valid_bits, FdkBsCfg::BsReader);
        bs
    }

    /// `FDKresetBitbuffer(hBitStream, config)`
    pub fn reset_bitbuffer(&mut self, config: FdkBsCfg) {
        self.h_bit_buf.reset();
        self.cache_word = 0;
        self.bits_in_cache = 0;
        self.config_cache = config;
    }

    /// `FDKdeleteBitStream()`: nothing to free.
    pub fn delete(&mut self) {
        self.h_bit_buf.delete();
    }

    /// `FDKreadBits(hBitStream, numberOfBits)`: read `number_of_bits` (0..=32) bits.
    #[inline(always)]
    pub fn read_bits(&mut self, number_of_bits: u32) -> u32 {
        let mut bits: u32 = 0;
        let missing_bits = number_of_bits as i32 - self.bits_in_cache as i32;
        debug_assert!(number_of_bits <= 32);
        if missing_bits > 0 {
            if missing_bits != 32 {
                bits = self.cache_word << missing_bits;
            }
            self.cache_word = self.h_bit_buf.get32();
            self.bits_in_cache += CACHE_BITS;
        }
        self.bits_in_cache -= number_of_bits;
        (bits | self.cache_word.wrapping_shr(self.bits_in_cache))
            & BIT_MASK[number_of_bits as usize]
    }

    /// [`read_bits`](Self::read_bits) for `number_of_bits < 32` (the mask is computed, not
    /// loaded).
    #[inline(always)]
    pub fn read_bits_short(&mut self, number_of_bits: u32) -> u32 {
        let mut bits: u32 = 0;
        let missing_bits = number_of_bits as i32 - self.bits_in_cache as i32;
        debug_assert!(number_of_bits < 32);
        if missing_bits > 0 {
            bits = self.cache_word << missing_bits;
            self.cache_word = self.h_bit_buf.get32();
            self.bits_in_cache += CACHE_BITS;
        }
        self.bits_in_cache -= number_of_bits;
        (bits | self.cache_word.wrapping_shr(self.bits_in_cache)) & ((1u32 << number_of_bits) - 1)
    }

    /// `FDKreadBit(hBitStream)`
    #[inline(always)]
    pub fn read_bit(&mut self) -> u32 {
        if self.bits_in_cache == 0 {
            self.cache_word = self.h_bit_buf.get32();
            self.bits_in_cache = CACHE_BITS - 1;
            return self.cache_word >> 31;
        }
        self.bits_in_cache -= 1;
        (self.cache_word >> self.bits_in_cache) & 1
    }

    /// `FDKread2Bits(hBitStream)`
    #[inline(always)]
    pub fn read_2bits(&mut self) -> u32 {
        let mut bits: u32 = 0;
        let missing_bits = 2 - self.bits_in_cache as i32;
        if missing_bits > 0 {
            bits = self.cache_word << missing_bits;
            self.cache_word = self.h_bit_buf.get32();
            self.bits_in_cache += CACHE_BITS;
        }
        self.bits_in_cache -= 2;
        (bits | self.cache_word.wrapping_shr(self.bits_in_cache)) & 0x3
    }

    /// `FDKreadBitsBwd(hBitStream, numberOfBits)`
    #[inline]
    pub fn read_bits_bwd(&mut self, number_of_bits: u32) -> u32 {
        let valid_mask = BIT_MASK[number_of_bits as usize];
        if self.bits_in_cache <= number_of_bits {
            let free_bits = (CACHE_BITS - 1) - self.bits_in_cache;
            self.cache_word =
                self.cache_word.wrapping_shl(free_bits) | self.h_bit_buf.get_bwd(free_bits);
            self.bits_in_cache += free_bits;
        }
        self.bits_in_cache -= number_of_bits;
        (self.cache_word >> self.bits_in_cache) & valid_mask
    }

    /// `escapedValue(hBitStream, nBits1, nBits2, nBits3)`
    #[inline]
    pub fn escaped_value(&mut self, n_bits1: i32, n_bits2: i32, n_bits3: i32) -> u32 {
        let mut value = self.read_bits(n_bits1 as u32);
        if value == (1u32 << n_bits1) - 1 {
            let value_add = self.read_bits(n_bits2 as u32);
            value = value.wrapping_add(value_add);
            if value_add == (1u32 << n_bits2) - 1 {
                let temp_diff = 0xFFFF_FFFFu32 - value;
                let temp_value = self.read_bits(n_bits3 as u32);
                if temp_value > temp_diff {
                    return 0xFFFF_FFFF;
                }
                value += temp_value;
            }
        }
        value
    }

    /// `FDKgetBits(hBitStream, numBits)`: read directly from the bit buffer, bypassing the cache.
    #[inline]
    pub fn get_bits(&mut self, num_bits: u32) -> u32 {
        self.h_bit_buf.get(num_bits)
    }

    /// `FDKwriteBits(hBitStream, value, numberOfBits)`
    #[inline]
    pub fn write_bits(&mut self, mut value: u32, number_of_bits: u32) -> u8 {
        let valid_mask = BIT_MASK[number_of_bits as usize];
        if self.bits_in_cache + number_of_bits < CACHE_BITS {
            self.bits_in_cache += number_of_bits;
            self.cache_word = self.cache_word.wrapping_shl(number_of_bits) | (value & valid_mask);
        } else {
            // Put always 32 bits into memory
            // - fill cache's LSBits with MSBits of value
            // - store 32 bits in memory using subroutine
            // - fill remaining bits into cache's LSBits
            // - upper bits in cache are don't care
            // Compute number of bits to be filled into cache
            let missing_bits = CACHE_BITS - self.bits_in_cache;
            let remaining_bits = number_of_bits - missing_bits;
            value &= valid_mask;
            // Avoid shift left by 32 positions
            let mut cache_word =
                if missing_bits == 32 { 0 } else { self.cache_word << missing_bits };
            cache_word |= value >> remaining_bits;
            self.h_bit_buf.put(cache_word, 32);
            self.cache_word = value;
            self.bits_in_cache = remaining_bits;
        }
        number_of_bits as u8
    }

    /// `FDKwriteBitsBwd(hBitStream, value, numberOfBits)`
    #[inline]
    pub fn write_bits_bwd(&mut self, value: u32, number_of_bits: u32) -> u8 {
        let valid_mask = BIT_MASK[number_of_bits as usize];
        if self.bits_in_cache + number_of_bits <= CACHE_BITS {
            self.bits_in_cache += number_of_bits;
            self.cache_word = self.cache_word.wrapping_shl(number_of_bits) | (value & valid_mask);
        } else {
            self.h_bit_buf.put_bwd(self.cache_word, self.bits_in_cache);
            self.bits_in_cache = number_of_bits;
            self.cache_word = value & valid_mask;
        }
        number_of_bits as u8
    }

    /// `FDKwriteEscapedValue(hBitStream, value, nBits1, nBits2, nBits3)`
    pub fn write_escaped_value(
        &mut self,
        mut value: u32,
        n_bits1: u32,
        n_bits2: u32,
        n_bits3: u32,
    ) -> u8 {
        let mut nbits: u8 = 0;
        let mut tmp = (1u32 << n_bits1) - 1;
        if value < tmp {
            nbits = nbits.wrapping_add(self.write_bits(value, n_bits1));
        } else {
            nbits = nbits.wrapping_add(self.write_bits(tmp, n_bits1));
            value -= tmp;
            tmp = (1u32 << n_bits2) - 1;
            if value < tmp {
                nbits = nbits.wrapping_add(self.write_bits(value, n_bits2));
            } else {
                nbits = nbits.wrapping_add(self.write_bits(tmp, n_bits2));
                value -= tmp;
                nbits = nbits.wrapping_add(self.write_bits(value, n_bits3));
            }
        }
        nbits
    }

    /// `FDKsyncCache(hBitStream)`
    #[inline]
    pub fn sync_cache(&mut self) {
        if self.config_cache == FdkBsCfg::BsReader {
            self.h_bit_buf.push_back(self.bits_in_cache, self.config_cache as u8);
        } else if self.bits_in_cache != 0 {
            // BS_WRITER
            self.h_bit_buf.put(self.cache_word, self.bits_in_cache);
        }
        self.bits_in_cache = 0;
        self.cache_word = 0;
    }

    /// `FDKsyncCacheBwd(hBitStream)`
    #[inline]
    pub fn sync_cache_bwd(&mut self) {
        if self.config_cache == FdkBsCfg::BsReader {
            self.h_bit_buf.push_forward(self.bits_in_cache, self.config_cache as u8);
        } else {
            // BS_WRITER
            self.h_bit_buf.put_bwd(self.cache_word, self.bits_in_cache);
        }
        self.bits_in_cache = 0;
        self.cache_word = 0;
    }

    /// `FDKbyteAlign(hBitStream, alignmentAnchor)`
    #[inline]
    pub fn byte_align(&mut self, alignment_anchor: u32) {
        self.sync_cache();
        if self.config_cache == FdkBsCfg::BsReader {
            let n = (8
                - ((alignment_anchor as i32).wrapping_sub(self.h_bit_buf.get_valid_bits()) & 0x07))
                & 0x07;
            self.h_bit_buf.push_forward(n as u32, self.config_cache as u8);
        } else {
            let n = (8
                - (self.h_bit_buf.get_valid_bits().wrapping_sub(alignment_anchor as i32) & 0x07))
                & 0x07;
            self.h_bit_buf.put(0, n as u32);
        }
    }

    /// `FDKpushBackCache(hBitStream, numberOfBits)`
    #[inline]
    pub fn push_back_cache(&mut self, number_of_bits: u32) {
        debug_assert!(self.bits_in_cache + number_of_bits <= CACHE_BITS);
        self.bits_in_cache += number_of_bits;
    }

    /// `FDKpushBack(hBitStream, numberOfBits)`
    #[inline]
    pub fn push_back(&mut self, number_of_bits: u32) {
        if self.bits_in_cache.wrapping_add(number_of_bits) < CACHE_BITS
            && self.config_cache == FdkBsCfg::BsReader
        {
            self.bits_in_cache += number_of_bits;
            self.sync_cache(); // sync cache to avoid invalid cache
        } else {
            self.sync_cache();
            self.h_bit_buf.push_back(number_of_bits, self.config_cache as u8);
        }
    }

    /// `FDKpushFor(hBitStream, numberOfBits)`
    #[inline]
    pub fn push_for(&mut self, number_of_bits: u32) {
        if self.bits_in_cache > number_of_bits && self.config_cache == FdkBsCfg::BsReader {
            self.bits_in_cache -= number_of_bits;
        } else {
            self.sync_cache();
            self.h_bit_buf.push_forward(number_of_bits, self.config_cache as u8);
        }
    }

    /// `FDKpushBiDirectional(hBitStream, numberOfBits)`
    #[inline]
    pub fn push_bi_directional(&mut self, number_of_bits: i32) {
        if number_of_bits >= 0 {
            self.push_for(number_of_bits as u32);
        } else {
            self.push_back(number_of_bits.wrapping_neg() as u32);
        }
    }

    /// `FDKgetValidBits(hBitStream)` (syncs the cache)
    #[inline]
    pub fn get_valid_bits(&mut self) -> i32 {
        self.sync_cache();
        self.h_bit_buf.get_valid_bits()
    }

    /// `FDKgetFreeBits(hBitStream)`
    #[inline]
    pub fn get_free_bits(&self) -> i32 {
        self.h_bit_buf.get_free_bits()
    }

    /// `FDKfeedBuffer(hBitStream, inputBuffer, bufferSize, bytesValid)`
    #[inline]
    pub fn feed_buffer(&mut self, input: &[u8], bytes_valid: &mut u32) {
        self.sync_cache();
        self.h_bit_buf.feed(input, bytes_valid);
    }

    /// `FDKcopyBuffer(hBSDst, hBSSrc, bytesValid)`: copies from `src` into `self`.
    #[inline]
    pub fn copy_buffer(&mut self, src: &mut FdkBitStream, bytes_valid: &mut u32) {
        src.sync_cache();
        self.h_bit_buf.copy_from(&mut src.h_bit_buf, bytes_valid);
    }

    /// `FDKfetchBuffer(hBitStream, outputBuffer, writeBytes)`
    #[inline]
    pub fn fetch_buffer(&mut self, out: &mut [u8], write_bytes: &mut u32) {
        self.sync_cache();
        self.h_bit_buf.fetch(out, write_bytes);
    }
}
