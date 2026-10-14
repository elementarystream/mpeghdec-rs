// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_bitbuffer.h and libFDK/src/FDK_bitbuffer.cpp.

//! Circular bit buffer read/write routines.
//!
//! In C a bit buffer holds a pointer to a ring buffer whose size is a power of two, and the code
//! saves and restores bitstream states by struct assignment while several such copies share the
//! memory. Here the memory is a [`BitMem`], a reference-counted array of bytes: clones of a bit
//! buffer (the struct assignments of C) read and write the same bytes, and the memory lives as
//! long as any bit buffer refers to it, so no use of a bit buffer can reach freed memory. The
//! bytes are atomics accessed with relaxed ordering (plain loads and stores), so bit buffers can
//! be sent between threads.
//!
//! All indices are masked with the buffer size, as in C. Every access is also checked against
//! the buffer size: this never triggers for an initialized bit buffer, and makes a default bit
//! buffer (without memory) harmless: it reads zeros and drops writes.

use std::sync::atomic::{AtomicU8, Ordering::Relaxed};
use std::sync::Arc;

/// Leave 3 bits headroom so MAX_BUFSIZE can be represented in bits as well.
pub const MAX_BUFSIZE_BYTES: u32 = 0x0800_0000;

/// `BitMask[n] = (1 << n) - 1` for `n = 0..=32`.
pub static BIT_MASK: [u32; 33] = [
    0x0, 0x1, 0x3, 0x7, 0xf, 0x1f, 0x3f, 0x7f, 0xff, 0x1ff, 0x3ff, 0x7ff, 0xfff, 0x1fff, 0x3fff,
    0x7fff, 0xffff, 0x1ffff, 0x3ffff, 0x7ffff, 0xfffff, 0x1fffff, 0x3fffff, 0x7fffff, 0xffffff,
    0x1ffffff, 0x3ffffff, 0x7ffffff, 0xfffffff, 0x1fffffff, 0x3fffffff, 0x7fffffff, 0xffffffff,
];

/// The memory of bit buffers: a fixed-size array of bytes shared by all clones (see the module
/// documentation). Accesses outside of it read 0 and drop the write.
#[derive(Clone, Default)]
pub struct BitMem(Arc<[AtomicU8]>);

impl BitMem {
    /// `len` zero bytes.
    pub fn new(len: usize) -> BitMem {
        BitMem((0..len).map(|_| AtomicU8::new(0)).collect())
    }

    /// A copy of `data`.
    pub fn from_slice(data: &[u8]) -> BitMem {
        BitMem(data.iter().map(|&b| AtomicU8::new(b)).collect())
    }

    /// The size in bytes.
    #[inline]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether the memory is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The byte at `idx` (0 outside of the memory).
    #[inline(always)]
    pub fn get(&self, idx: usize) -> u8 {
        self.0.get(idx).map_or(0, |b| b.load(Relaxed))
    }

    /// Writes the byte at `idx` (dropped outside of the memory).
    #[inline(always)]
    pub fn set(&self, idx: usize, v: u8) {
        if let Some(b) = self.0.get(idx) {
            b.store(v, Relaxed);
        }
    }

    /// Copies the bytes from `idx` on into `out` (0 outside of the memory).
    pub fn read(&self, idx: usize, out: &mut [u8]) {
        for (k, o) in out.iter_mut().enumerate() {
            *o = idx.checked_add(k).map_or(0, |j| self.get(j));
        }
    }

    /// Writes `data` from `idx` on (dropped outside of the memory).
    pub fn write(&self, idx: usize, data: &[u8]) {
        for (k, &v) in data.iter().enumerate() {
            if let Some(j) = idx.checked_add(k) {
                self.set(j, v);
            }
        }
    }

    /// Sets all bytes from `idx` on to `v`.
    pub fn fill_from(&self, idx: usize, v: u8) {
        for b in self.0.iter().skip(idx) {
            b.store(v, Relaxed);
        }
    }

    /// A new memory with a copy of the content (not shared).
    pub fn deep_clone(&self) -> BitMem {
        BitMem(self.0.iter().map(|b| AtomicU8::new(b.load(Relaxed))).collect())
    }

    /// A copy of the content.
    pub fn to_vec(&self) -> Vec<u8> {
        self.0.iter().map(|b| b.load(Relaxed)).collect()
    }

    /// Whether `self` and `other` are the same memory.
    #[inline]
    pub fn ptr_eq(&self, other: &BitMem) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl core::fmt::Debug for BitMem {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "BitMem({} bytes)", self.len())
    }
}

/// The content (the memory belongs to the state of its owner, e.g. the transport decoder).
impl crate::digest::StateDigest for BitMem {
    fn digest(&self, d: &mut crate::digest::Digester) {
        // the digest of the content as a `[u8]`
        d.u64(self.len() as u64);
        let mut chunk = [0u8; 4096];
        for start in (0..self.len()).step_by(chunk.len()) {
            let n = chunk.len().min(self.len() - start);
            self.read(start, &mut chunk[..n]);
            d.bytes(&chunk[..n]);
        }
    }
}

/// `FDK_BITBUF`: a ring buffer of bits over a [`BitMem`] shared with the clones of the bit
/// buffer. A default bit buffer has no memory (`buf_size` 0) and reads zeros.
#[derive(Clone, Debug, Default)]
pub struct FdkBitBuf {
    pub valid_bits: i32,
    pub(crate) read_offset: u32,
    pub(crate) write_offset: u32,
    pub(crate) bit_ndx: u32,
    mem: BitMem,
    pub(crate) buf_size: u32,
    pub(crate) buf_bits: u32,
}

/// All fields except the memory: its content is part of the state of its owner (the transport
/// decoder); two bit buffers of different decoders use different memory.
impl crate::digest::StateDigest for FdkBitBuf {
    fn digest(&self, d: &mut crate::digest::Digester) {
        let FdkBitBuf {
            valid_bits,
            read_offset,
            write_offset,
            bit_ndx,
            mem: _,
            buf_size,
            buf_bits,
        } = self;
        d.field("valid_bits", valid_bits);
        d.field("read_offset", read_offset);
        d.field("write_offset", write_offset);
        d.field("bit_ndx", bit_ndx);
        d.field("buf_size", buf_size);
        d.field("buf_bits", buf_bits);
    }
}

impl FdkBitBuf {
    /// Byte `idx` of the memory; 0 outside of it. The callers mask or bound the index to the
    /// buffer size, which is at most the size of the memory (0 without memory).
    #[inline(always)]
    fn byte(&self, idx: u32) -> u32 {
        self.mem.get(idx as usize) as u32
    }

    /// Sets byte `idx` of the memory; ignored outside of it (see [`FdkBitBuf::byte`]).
    #[inline(always)]
    fn set_byte(&mut self, idx: u32, v: u8) {
        self.mem.set(idx as usize, v);
    }

    /// `hBitBuf->bitNdx`: the bit position in the buffer.
    #[inline]
    pub fn bit_ndx(&self) -> u32 {
        self.bit_ndx
    }

    /// `hBitBuf->ReadOffset`
    #[inline]
    pub fn read_offset(&self) -> u32 {
        self.read_offset
    }

    /// `hBitBuf->WriteOffset`
    #[inline]
    pub fn write_offset(&self) -> u32 {
        self.write_offset
    }

    /// `hBitBuf->bufSize`: the size of the buffer in bytes.
    #[inline]
    pub fn buf_size(&self) -> u32 {
        self.buf_size
    }

    /// `hBitBuf->bufBits`: the size of the buffer in bits.
    #[inline]
    pub fn buf_bits(&self) -> u32 {
        self.buf_bits
    }

    /// The memory of the buffer (`hBitBuf->Buffer`).
    #[inline]
    pub fn mem(&self) -> &BitMem {
        &self.mem
    }

    /// `FDK_CreateBitBuffer()`: initializes the bit buffer and clears the memory.
    pub fn create(&mut self, mem: &BitMem, buf_size: u32) {
        self.init(mem, buf_size, 0);
        for i in 0..buf_size as usize {
            self.mem.set(i, 0);
        }
    }

    /// `FDK_DeleteBitBuffer()`: nothing to do.
    pub fn delete(&mut self) {}

    /// `FDK_InitBitBuffer()`: a bit buffer over the first `buf_size` bytes of `mem` (a power of
    /// two, at most the size of `mem`; a larger size is reduced to the size of `mem`).
    pub fn init(&mut self, mem: &BitMem, buf_size: u32, valid_bits: u32) {
        debug_assert!(buf_size as usize <= mem.len());
        let buf_size = buf_size.min(mem.len() as u32);
        self.valid_bits = valid_bits as i32;
        self.read_offset = 0;
        self.write_offset = 0;
        self.bit_ndx = 0;
        self.mem = mem.clone();
        self.buf_size = buf_size;
        self.buf_bits = buf_size << 3;
        debug_assert!(self.valid_bits <= self.buf_bits as i32);
        debug_assert!(buf_size > 0 && buf_size <= MAX_BUFSIZE_BYTES);
        debug_assert!(buf_size.is_power_of_two());
    }

    /// `FDK_ResetBitBuffer()`
    pub fn reset(&mut self) {
        self.valid_bits = 0;
        self.read_offset = 0;
        self.write_offset = 0;
        self.bit_ndx = 0;
    }

    /// `FDK_get()`: read `number_of_bits` (1..=32) bits.
    #[inline]
    pub fn get(&mut self, number_of_bits: u32) -> u32 {
        let byte_offset = self.bit_ndx >> 3;
        let bit_offset = self.bit_ndx & 0x07;
        self.bit_ndx = self.bit_ndx.wrapping_add(number_of_bits) & (self.buf_bits.wrapping_sub(1));
        self.valid_bits = self.valid_bits.wrapping_sub(number_of_bits as i32);
        let byte_mask = self.buf_size.wrapping_sub(1);
        let mut tx = (self.byte(byte_offset & byte_mask) << 24)
            | (self.byte((byte_offset + 1) & byte_mask) << 16)
            | (self.byte((byte_offset + 2) & byte_mask) << 8)
            | self.byte((byte_offset + 3) & byte_mask);
        if bit_offset != 0 {
            tx <<= bit_offset;
            tx |= self.byte((byte_offset + 4) & byte_mask) >> (8 - bit_offset);
        }
        tx.wrapping_shr(32 - number_of_bits)
    }

    /// `FDK_get32()`: read 32 bits.
    #[inline]
    pub fn get32(&mut self) -> u32 {
        let mut bit_ndx = self.bit_ndx.wrapping_add(32);
        self.bit_ndx = bit_ndx & (self.buf_bits.wrapping_sub(1));
        self.valid_bits = self.valid_bits.wrapping_sub(32);
        let byte_offset = bit_ndx.wrapping_sub(1) >> 3;
        if bit_ndx <= self.buf_bits {
            let mut cache = (self.byte(byte_offset - 3) << 24)
                | (self.byte(byte_offset - 2) << 16)
                | (self.byte(byte_offset - 1) << 8)
                | self.byte(byte_offset);
            bit_ndx &= 7;
            if bit_ndx != 0 {
                cache = (cache >> (8 - bit_ndx)) | (self.byte(byte_offset - 4) << (24 + bit_ndx));
            }
            cache
        } else {
            let byte_mask = self.buf_size.wrapping_sub(1);
            let mut cache = (self.byte(byte_offset.wrapping_sub(3) & byte_mask) << 24)
                | (self.byte(byte_offset.wrapping_sub(2) & byte_mask) << 16)
                | (self.byte(byte_offset.wrapping_sub(1) & byte_mask) << 8)
                | self.byte(byte_offset & byte_mask);
            bit_ndx &= 7;
            if bit_ndx != 0 {
                cache = (cache >> (8 - bit_ndx))
                    | (self.byte(byte_offset.wrapping_sub(4) & byte_mask) << (24 + bit_ndx));
            }
            cache
        }
    }

    /// `FDK_getBwd()`: read `number_of_bits` bits backwards.
    pub fn get_bwd(&mut self, number_of_bits: u32) -> u32 {
        let byte_offset = self.bit_ndx >> 3;
        let bit_offset = self.bit_ndx & 0x07;
        let byte_mask = self.buf_size.wrapping_sub(1);
        self.bit_ndx = self.bit_ndx.wrapping_sub(number_of_bits) & (self.buf_bits.wrapping_sub(1));
        self.valid_bits = self.valid_bits.wrapping_add(number_of_bits as i32);
        let mut tx = (self.byte(byte_offset.wrapping_sub(3) & byte_mask) << 24)
            | (self.byte(byte_offset.wrapping_sub(2) & byte_mask) << 16)
            | (self.byte(byte_offset.wrapping_sub(1) & byte_mask) << 8)
            | self.byte(byte_offset & byte_mask);
        let mut txa: u32 = 0;
        tx >>= 8 - bit_offset;
        if bit_offset != 0 && number_of_bits > 24 {
            tx |= self.byte(byte_offset.wrapping_sub(4) & byte_mask) << (24 + bit_offset);
        }
        // in place turn around
        for i in 0..16u32 {
            let bit_mask_r = 1u32 << i;
            let bit_mask_l = 0x8000_0000u32 >> i;
            txa |= (tx & bit_mask_r) << (31 - (i << 1));
            txa |= (tx & bit_mask_l) >> (31 - (i << 1));
        }
        txa.wrapping_shr(32 - number_of_bits)
    }

    /// `FDK_put()`: write `number_of_bits` bits of `value`.
    pub fn put(&mut self, value: u32, number_of_bits: u32) {
        if number_of_bits != 0 {
            let byte_offset0 = self.bit_ndx >> 3;
            let bit_offset = self.bit_ndx & 0x7;
            self.bit_ndx =
                self.bit_ndx.wrapping_add(number_of_bits) & (self.buf_bits.wrapping_sub(1));
            self.valid_bits = self.valid_bits.wrapping_add(number_of_bits as i32);
            let byte_mask = self.buf_size.wrapping_sub(1);
            let byte_offset1 = (byte_offset0 + 1) & byte_mask;
            let byte_offset2 = (byte_offset0 + 2) & byte_mask;
            let byte_offset3 = (byte_offset0 + 3) & byte_mask;
            // Create tmp containing free bits at the left border followed by bits to write, LSB's
            // are cleared, if available. Create mask to apply upon all buffer bytes.
            let tmp = (value << (32 - number_of_bits)) >> bit_offset;
            let mask =
                !((BIT_MASK[number_of_bits as usize] << (32 - number_of_bits)) >> bit_offset);
            // read all 4 bytes from buffer and create a 32-bit cache
            let mut cache = (self.byte(byte_offset0) << 24)
                | (self.byte(byte_offset1) << 16)
                | (self.byte(byte_offset2) << 8)
                | self.byte(byte_offset3);
            cache = (cache & mask) | tmp;
            self.set_byte(byte_offset0, (cache >> 24) as u8);
            self.set_byte(byte_offset1, (cache >> 16) as u8);
            self.set_byte(byte_offset2, (cache >> 8) as u8);
            self.set_byte(byte_offset3, cache as u8);
            if bit_offset + number_of_bits > 32 {
                let byte_offset4 = (byte_offset0 + 4) & byte_mask;
                // remaining bits: in range 1..7
                // replace MSBits of next byte in buffer by LSBits of "value"
                let bits = (bit_offset + number_of_bits) & 7;
                let mut cache = self.byte(byte_offset4) & !(BIT_MASK[bits as usize] << (8 - bits));
                cache |= value << (8 - bits);
                self.set_byte(byte_offset4, cache as u8);
            }
        }
    }

    /// `FDK_putBwd()`: write `number_of_bits` bits of `value` backwards.
    pub fn put_bwd(&mut self, mut value: u32, number_of_bits: u32) {
        let byte_offset = self.bit_ndx >> 3;
        let bit_offset = 7 - (self.bit_ndx & 0x07);
        let byte_mask = self.buf_size.wrapping_sub(1);
        let mask = !(BIT_MASK[number_of_bits as usize] << bit_offset);
        let mut tmp: u32 = 0;
        self.bit_ndx = self.bit_ndx.wrapping_sub(number_of_bits) & (self.buf_bits.wrapping_sub(1));
        self.valid_bits = self.valid_bits.wrapping_sub(number_of_bits as i32);
        // in place turn around
        for i in 0..16u32 {
            let bit_mask_r = 1u32 << i;
            let bit_mask_l = 0x8000_0000u32 >> i;
            tmp |= (value & bit_mask_r) << (31 - (i << 1));
            tmp |= (value & bit_mask_l) >> (31 - (i << 1));
        }
        value = tmp;
        tmp = value.wrapping_shr(32 - number_of_bits) << bit_offset;
        let i0 = byte_offset & byte_mask;
        let i1 = byte_offset.wrapping_sub(1) & byte_mask;
        let i2 = byte_offset.wrapping_sub(2) & byte_mask;
        let i3 = byte_offset.wrapping_sub(3) & byte_mask;
        self.set_byte(i0, ((self.byte(i0) & mask) | (tmp as u8 as u32)) as u8);
        self.set_byte(i1, ((self.byte(i1) & (mask >> 8)) | ((tmp >> 8) as u8 as u32)) as u8);
        self.set_byte(i2, ((self.byte(i2) & (mask >> 16)) | ((tmp >> 16) as u8 as u32)) as u8);
        self.set_byte(i3, ((self.byte(i3) & (mask >> 24)) | ((tmp >> 24) as u8 as u32)) as u8);
        if bit_offset + number_of_bits > 32 {
            let i4 = byte_offset.wrapping_sub(4) & byte_mask;
            let v = (value.wrapping_shr(64 - number_of_bits - bit_offset) as u8 as u32)
                | (self.byte(i4)
                    & !(BIT_MASK[bit_offset as usize].wrapping_shr(32 - number_of_bits)));
            self.set_byte(i4, v as u8);
        }
    }

    /// `FDK_pushBack()`: move the read/write position back by `number_of_bits`.
    #[inline]
    pub fn push_back(&mut self, number_of_bits: u32, config: u8) {
        self.valid_bits = if config == 0 {
            self.valid_bits.wrapping_add(number_of_bits as i32)
        } else {
            self.valid_bits.wrapping_sub(number_of_bits as i32)
        };
        self.bit_ndx = ((self.bit_ndx as i32).wrapping_sub(number_of_bits as i32) as u32)
            & (self.buf_bits.wrapping_sub(1));
    }

    /// `FDK_pushForward()`: move the read/write position forward by `number_of_bits`.
    #[inline]
    pub fn push_forward(&mut self, number_of_bits: u32, config: u8) {
        self.valid_bits = if config == 0 {
            self.valid_bits.wrapping_sub(number_of_bits as i32)
        } else {
            self.valid_bits.wrapping_add(number_of_bits as i32)
        };
        self.bit_ndx = ((self.bit_ndx as i32).wrapping_add(number_of_bits as i32) as u32)
            & (self.buf_bits.wrapping_sub(1));
    }

    /// `FDK_getValidBits()`
    #[inline]
    pub fn get_valid_bits(&self) -> i32 {
        self.valid_bits
    }

    /// `FDK_getFreeBits()`
    #[inline]
    pub fn get_free_bits(&self) -> i32 {
        (self.buf_bits as i32).wrapping_sub(self.valid_bits)
    }

    /// `FDK_Feed(hBitBuf, inputBuffer, bufferSize, bytesValid)`: copies the last `*bytes_valid`
    /// bytes of `input` into the buffer (as much as fits) and decrements `*bytes_valid`.
    ///
    /// # Panics
    /// If `*bytes_valid > input.len()`.
    pub fn feed(&mut self, input: &[u8], bytes_valid: &mut u32) {
        assert!(*bytes_valid as usize <= input.len(), "FDK_Feed: bytesValid > bufferSize");
        let buffer_size = input.len() as u32;
        let mut pos = (buffer_size - *bytes_valid) as usize;
        let mut b_total: u32 = 0;
        let mut b_to_read = f_min_u(
            self.buf_bits,
            (self.buf_bits as i32).wrapping_sub(self.valid_bits).max(0) as u32,
        ) >> 3;
        let mut no_of_bytes = f_min_u(b_to_read, *bytes_valid);
        while no_of_bytes > 0 {
            // split read to buffer size
            b_to_read = self.buf_size - self.read_offset;
            b_to_read = f_min_u(b_to_read, no_of_bytes);
            if b_to_read == 0 {
                /* a bit buffer without memory */
                break;
            }
            // copy 'bToRead' bytes from 'ptr' to inputbuffer
            self.mem.write(self.read_offset as usize, &input[pos..pos + b_to_read as usize]);
            // add noOfBits to number of valid bits in buffer
            self.valid_bits = self.valid_bits.wrapping_add((b_to_read << 3) as i32);
            b_total += b_to_read;
            pos += b_to_read as usize;
            self.read_offset = (self.read_offset + b_to_read) & (self.buf_size.wrapping_sub(1));
            no_of_bytes -= b_to_read;
        }
        *bytes_valid -= b_total;
    }

    fn copy_aligned_block(&mut self, dst: &mut FdkBitBuf, dst_offset: u32, b_to_read: u32) {
        let byte_offset = self.bit_ndx >> 3;
        let byte_mask = self.buf_size.wrapping_sub(1);
        for i in 0..b_to_read {
            let v = self.byte((byte_offset + i) & byte_mask);
            dst.set_byte(dst_offset + i, v as u8);
        }
        let bits = b_to_read << 3;
        self.bit_ndx = self.bit_ndx.wrapping_add(bits) & (self.buf_bits.wrapping_sub(1));
        self.valid_bits = self.valid_bits.wrapping_sub(bits as i32);
    }

    /// `FDK_Copy(hBitBufDst, hBitBufSrc, bytesValid)`: copies bytes from `src` into `self`.
    pub fn copy_from(&mut self, src: &mut FdkBitBuf, bytes_valid: &mut u32) {
        let mut b_total: i32 = 0;
        // limit noOfBytes to valid bytes in src buffer and available bytes in dst buffer
        let mut b_to_read = (src.valid_bits >> 3) as u32;
        let mut no_of_bytes = f_min_u(b_to_read, *bytes_valid);
        b_to_read = self.get_free_bits() as u32;
        no_of_bytes = f_min_u(b_to_read, no_of_bytes);
        while no_of_bytes > 0 {
            // Split Read to buffer size
            b_to_read = self.buf_size - self.read_offset;
            b_to_read = f_min_u(no_of_bytes, b_to_read);
            if b_to_read == 0 {
                /* a bit buffer without memory */
                break;
            }
            // copy 'bToRead' bytes from buffer to buffer
            if src.bit_ndx & 0x07 == 0 {
                src.copy_aligned_block(self, self.read_offset, b_to_read);
            } else {
                for i in 0..b_to_read {
                    let v = src.get(8) as u8;
                    self.set_byte(self.read_offset + i, v);
                }
            }
            // add noOfBits to number of valid bits in buffer
            self.valid_bits = self.valid_bits.wrapping_add((b_to_read << 3) as i32);
            b_total += b_to_read as i32;
            self.read_offset = (self.read_offset + b_to_read) & (self.buf_size.wrapping_sub(1));
            no_of_bytes -= b_to_read;
        }
        *bytes_valid = bytes_valid.wrapping_sub(b_total as u32);
    }

    /// `FDK_Fetch(hBitBuf, outBuf, writeBytes)`: at most `out.len()` bytes (C writes past
    /// `outBuf`).
    pub fn fetch(&mut self, out: &mut [u8], write_bytes: &mut u32) {
        let mut pos = 0usize;
        let mut b_total: u32 = 0;
        let mut b_to_write = (self.valid_bits >> 3) as u32;
        let mut no_of_bytes =
            f_min_u(f_min_u(b_to_write, *write_bytes), out.len().min(u32::MAX as usize) as u32);
        while no_of_bytes > 0 {
            // split write to buffer size
            b_to_write = self.buf_size - self.write_offset;
            b_to_write = f_min_u(b_to_write, no_of_bytes);
            if b_to_write == 0 {
                /* a bit buffer without memory */
                break;
            }
            // copy 'bToWrite' bytes from bitbuffer to outputbuffer
            for i in 0..b_to_write {
                out[pos + i as usize] = self.byte(self.write_offset + i) as u8;
            }
            // sub noOfBits from number of valid bits in buffer
            self.valid_bits = self.valid_bits.wrapping_sub((b_to_write << 3) as i32);
            b_total += b_to_write;
            pos += b_to_write as usize;
            self.write_offset = (self.write_offset + b_to_write) & (self.buf_size.wrapping_sub(1));
            no_of_bytes -= b_to_write;
        }
        *write_bytes = b_total;
    }
}

#[inline(always)]
fn f_min_u(a: u32, b: u32) -> u32 {
    if a < b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitstream::FdkBitStream;

    /// A bitstream without memory reads zeros and ignores writes.
    #[test]
    fn default_bitstream_has_no_memory() {
        let mut bs = FdkBitStream::default();
        assert_eq!(bs.read_bits(8), 0);
        assert_eq!(bs.read_bits(32), 0);
        let mut b = FdkBitBuf::default();
        b.put(0xFF, 8);
        assert_eq!(b.get(8), 0);
        // the position wraps like the unsigned arithmetic of C
        bs.push_back(64);
        assert_eq!(bs.read_bits(32), 0);
        assert_eq!(bs.read_bits(32), 0);
        bs.write_bits(1, 32);
        b.push_forward(0xFFFF_FFF0, 1);
        b.put(1, 32);
        b.push_forward(0x8000_0000, 0);
        let mut valid = 1;
        b.feed(&[0], &mut valid);
    }

    /// Fetching stops at the end of the output.
    #[test]
    fn fetch_is_bounded_by_the_output() {
        let mut b = FdkBitBuf::default();
        b.init(&BitMem::from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]), 8, 64);
        let mut out = [0u8; 3];
        let mut n = 8;
        b.fetch(&mut out, &mut n);
        assert_eq!((n, out), (3, [1, 2, 3]));
    }

    /// Fetching from a bit buffer without memory ends (no bytes can be fetched).
    #[test]
    fn fetch_without_memory_ends() {
        let mut bs = FdkBitStream::default();
        bs.push_back(8);
        let mut out = [0u8; 4];
        let mut n = 1;
        bs.fetch_buffer(&mut out, &mut n);
        assert_eq!(n, 0);
    }

    #[test]
    #[should_panic(expected = "bytesValid > bufferSize")]
    fn feed_rejects_more_valid_bytes_than_input() {
        let mut b = FdkBitBuf::default();
        b.init(&BitMem::new(64), 64, 0);
        let mut valid = 100;
        b.feed(&[0x11; 4], &mut valid);
    }

    /// Clones share the memory, which outlives the bit buffer it was created for.
    #[test]
    fn clones_share_the_memory() {
        let mut w = FdkBitBuf::default();
        w.init(&BitMem::new(4), 4, 0);
        let mut r = w.clone();
        drop(w.clone());
        w.put(0xA5, 8);
        drop(w);
        r.valid_bits = 8;
        assert_eq!(r.get(8), 0xA5);
    }
}
