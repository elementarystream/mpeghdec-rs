// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libSYS/src/wav_file.cpp (output part).

//! WAV file writer producing byte-identical files to `WAV_OutputOpen/Write/Close()`.
//!
//! The header always reserves a 28 byte `JUNK` chunk (80 bytes header in total) that is turned
//! into a `ds64` chunk (RF64) if the file exceeds 4 GiB.

use std::fs::File;
use std::io::{self, BufWriter, Seek, SeekFrom, Write};

pub const WAV_HEADER_SIZE: u32 = 80;

pub struct WavWriter {
    out: BufWriter<File>,
    num_channels: u16,
    sample_rate: u32,
    bits_per_sample: u16,
    data_size64: u64,
    /// Conversion buffer of `write_i32()` (reused).
    buf: Vec<u8>,
}

impl WavWriter {
    /// `WAV_OutputOpen()`: `bits_per_sample` is 16, 24 or 32 (integer PCM).
    pub fn create(
        path: &str,
        sample_rate: u32,
        num_channels: u16,
        bits_per_sample: u16,
    ) -> io::Result<Self> {
        if ![16, 24, 32].contains(&bits_per_sample) {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "invalid bits per sample"));
        }
        let mut w = WavWriter {
            out: BufWriter::with_capacity(1 << 20, File::create(path)?),
            num_channels,
            sample_rate,
            bits_per_sample,
            data_size64: 0,
            buf: Vec::new(),
        };
        // Preliminary header in case the file is not closed properly.
        let data_size = 0x7fff_ffff - WAV_HEADER_SIZE - 8;
        let header = w.header(b"RIFF", 0x7fff_ffff, b"JUNK", 0, 0, 0, data_size);
        w.out.write_all(&header)?;
        Ok(w)
    }

    #[allow(clippy::too_many_arguments)]
    fn header(
        &self,
        riff_type: &[u8; 4],
        riff_size: u32,
        chunk_id: &[u8; 4],
        riff_size64: u64,
        data_size64: u64,
        samples_per_ch64: u64,
        data_size: u32,
    ) -> Vec<u8> {
        let block_align = self.num_channels * (self.bits_per_sample >> 3);
        let mut h = Vec::with_capacity(WAV_HEADER_SIZE as usize);
        h.extend_from_slice(riff_type);
        h.extend_from_slice(&riff_size.to_le_bytes());
        h.extend_from_slice(b"WAVE");
        h.extend_from_slice(chunk_id);
        h.extend_from_slice(&28u32.to_le_bytes());
        h.extend_from_slice(&riff_size64.to_le_bytes());
        h.extend_from_slice(&data_size64.to_le_bytes());
        h.extend_from_slice(&samples_per_ch64.to_le_bytes());
        h.extend_from_slice(&0u32.to_le_bytes()); // tableLength
        h.extend_from_slice(b"fmt ");
        h.extend_from_slice(&16u32.to_le_bytes());
        h.extend_from_slice(&1u16.to_le_bytes()); // PCM
        h.extend_from_slice(&self.num_channels.to_le_bytes());
        h.extend_from_slice(&self.sample_rate.to_le_bytes());
        h.extend_from_slice(&self.sample_rate.wrapping_mul(block_align as u32).to_le_bytes());
        h.extend_from_slice(&block_align.to_le_bytes());
        h.extend_from_slice(&self.bits_per_sample.to_le_bytes());
        h.extend_from_slice(b"data");
        h.extend_from_slice(&data_size.to_le_bytes());
        debug_assert_eq!(h.len(), WAV_HEADER_SIZE as usize);
        h
    }

    /// `WAV_OutputWrite(wav, samples, n, 32, 32)`: writes 32-bit samples (MSB aligned) with the
    /// configured bit depth by an arithmetic right shift.
    pub fn write_i32(&mut self, samples: &[i32]) -> io::Result<()> {
        let bytes = (self.bits_per_sample >> 3) as usize;
        let shift = 32 - self.bits_per_sample as u32;
        self.buf.clear();
        self.buf.resize(samples.len() * bytes, 0);
        match bytes {
            3 => {
                let done = pack24_fast(&mut self.buf, samples, shift);
                let (buf, samples) = (&mut self.buf[3 * done..], &samples[done..]);
                // 4 samples -> 12 bytes, packed into 3 little endian words
                let mut chunks = buf.chunks_exact_mut(12);
                let mut src = samples.chunks_exact(4);
                for (d, s) in (&mut chunks).zip(&mut src) {
                    let v: [u32; 4] = core::array::from_fn(|k| (s[k] >> shift) as u32 & 0xff_ffff);
                    d[0..4].copy_from_slice(&(v[0] | v[1] << 24).to_le_bytes());
                    d[4..8].copy_from_slice(&(v[1] >> 8 | v[2] << 16).to_le_bytes());
                    d[8..12].copy_from_slice(&(v[2] >> 16 | v[3] << 8).to_le_bytes());
                }
                for (d, &s) in chunks.into_remainder().chunks_exact_mut(3).zip(src.remainder()) {
                    d.copy_from_slice(&(s >> shift).to_le_bytes()[..3]);
                }
            }
            _ => {
                for (d, &s) in self.buf.chunks_exact_mut(bytes).zip(samples) {
                    d.copy_from_slice(&(s >> shift).to_le_bytes()[..bytes]);
                }
            }
        }
        self.out.write_all(&self.buf)?;
        self.data_size64 += (samples.len() * bytes) as u64;
        Ok(())
    }

    /// `WAV_OutputClose()`: rewrites the header with the final sizes.
    pub fn close(mut self) -> io::Result<()> {
        let header = if self.data_size64 + WAV_HEADER_SIZE as u64 >= 1u64 << 32 {
            let riff_size64 = self.data_size64 + WAV_HEADER_SIZE as u64 - 8;
            let samples =
                self.data_size64 / (self.num_channels as u64 * (self.bits_per_sample as u64 >> 3));
            self.header(
                b"RF64",
                0xFFFF_FFFF,
                b"ds64",
                riff_size64,
                self.data_size64,
                samples,
                0xFFFF_FFFF,
            )
        } else {
            let data_size = self.data_size64 as u32;
            self.header(b"RIFF", data_size + WAV_HEADER_SIZE - 8, b"JUNK", 0, 0, 0, data_size)
        };
        self.out.flush()?;
        let mut f = self.out.into_inner().map_err(|e| e.into_error())?;
        f.seek(SeekFrom::Start(0))?;
        f.write_all(&header)?;
        f.flush()
    }
}

/// Packs the first samples of `samples` (`>> shift`, the 3 low bytes, little endian) into `buf`
/// with AVX2 (8 samples into 24 bytes at a time) and returns their number (0 without AVX2).
fn pack24_fast(buf: &mut [u8], samples: &[i32], shift: u32) -> usize {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    if mpeghdec_fdk::simd::avx2() {
        // Each 32 byte store writes 8 bytes beyond the 24 packed ones, which the next store (or
        // the scalar code) overwrites; the last store must stay within the 3 bytes per sample.
        let n = samples.len().saturating_sub(3) / 8 * 8;
        assert!(buf.len() >= 3 * samples.len());
        // SAFETY: AVX2 is available; the last store ends at 3 * (n - 8) + 32 = 3 * n + 8 bytes,
        // within the 3 * samples.len() bytes of buf as n <= samples.len() - 3.
        unsafe { pack24_avx2(buf.as_mut_ptr(), samples.as_ptr(), n, shift) };
        return n;
    }
    let _ = (buf, samples, shift);
    0
}

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn pack24_avx2(dst: *mut u8, src: *const i32, n: usize, shift: u32) {
    #[cfg(target_arch = "x86")]
    use core::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    use core::arch::x86_64::*;
    // the bytes 0..3 of each sample within each 128 bit lane, then the two 12 byte halves
    // joined
    let bytes = _mm256_setr_epi8(
        0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, -1, -1, -1, -1, //
        0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, -1, -1, -1, -1,
    );
    let words = _mm256_setr_epi32(0, 1, 2, 4, 5, 6, 3, 7);
    let count = _mm_cvtsi32_si128(shift as i32);
    let mut i = 0;
    while i < n {
        // SAFETY: see pack24_fast().
        unsafe {
            let v = _mm256_sra_epi32(_mm256_loadu_si256(src.add(i) as *const __m256i), count);
            let p = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(v, bytes), words);
            _mm256_storeu_si256(dst.add(3 * i) as *mut __m256i, p);
        }
        i += 8;
    }
}

/// Number of samples collected before they are handed over to the writer thread.
const HANDOVER_SAMPLES: usize = 1 << 16;

/// A [`WavWriter`] that optionally runs in a separate thread (not in C): the sample conversion
/// and the file writes then overlap with the decoding of the next frames. The file content is
/// the same; write errors of the thread are reported by a later [`WavSink::write_i32`] or by
/// [`WavSink::close`].
pub enum WavSink {
    Direct(WavWriter),
    Threaded {
        /// samples collected for the next hand-over to the writer thread
        pending: Vec<i32>,
        tx: std::sync::mpsc::SyncSender<Vec<i32>>,
        /// buffers returned by the writer thread for reuse
        recycle: std::sync::mpsc::Receiver<Vec<i32>>,
        handle: std::thread::JoinHandle<io::Result<WavWriter>>,
    },
}

impl WavSink {
    /// Writes with `w` directly, or in a writer thread if `threaded`.
    pub fn new(w: WavWriter, threaded: bool) -> WavSink {
        if !threaded {
            return WavSink::Direct(w);
        }
        let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<i32>>(8);
        let (recycle_tx, recycle) = std::sync::mpsc::channel::<Vec<i32>>();
        let handle = std::thread::spawn(move || {
            let mut w = w;
            for buf in rx {
                w.write_i32(&buf)?;
                let _ = recycle_tx.send(buf);
            }
            Ok(w)
        });
        WavSink::Threaded { pending: Vec::new(), tx, recycle, handle }
    }

    /// See [`WavWriter::write_i32`].
    pub fn write_i32(&mut self, samples: &[i32]) -> io::Result<()> {
        match self {
            WavSink::Direct(w) => w.write_i32(samples),
            WavSink::Threaded { pending, tx, recycle, .. } => {
                pending.extend_from_slice(samples);
                if pending.len() < HANDOVER_SAMPLES {
                    return Ok(());
                }
                /* hand over batches of frames (a hand-over may wake the writer thread) */
                let mut next = recycle.try_recv().unwrap_or_default();
                next.clear();
                let buf = core::mem::replace(pending, next);
                tx.send(buf).map_err(|_| io::Error::other("the WAV writer thread failed"))
            }
        }
    }

    /// See [`WavWriter::close`].
    pub fn close(self) -> io::Result<()> {
        match self {
            WavSink::Direct(w) => w.close(),
            WavSink::Threaded { pending, tx, recycle, handle } => {
                let sent = pending.is_empty() || tx.send(pending).is_ok();
                drop(tx);
                drop(recycle);
                let w =
                    handle.join().map_err(|_| io::Error::other("WAV writer thread panicked"))??;
                if !sent {
                    return Err(io::Error::other("the WAV writer thread failed"));
                }
                w.close()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The vectorized 24 bit packing against the bytes of the scalar conversion, for all
    /// lengths up to 100 (and that it writes nothing beyond the packed bytes).
    #[test]
    fn pack24_matches_scalar() {
        let mut x = 0x9E37_79B9_7F4A_7C15u64;
        for len in 0..100 {
            let samples: Vec<i32> = (0..len)
                .map(|_| {
                    x ^= x << 13;
                    x ^= x >> 7;
                    x ^= x << 17;
                    x as i32
                })
                .collect();
            let mut buf = vec![0xA5u8; 3 * len + 64];
            let done = pack24_fast(&mut buf[..3 * len], &samples, 8);
            assert!(done <= len);
            for (i, &s) in samples[..done].iter().enumerate() {
                assert_eq!(buf[3 * i..3 * i + 3], (s >> 8).to_le_bytes()[..3], "len {len}");
            }
            assert!(buf[3 * len..].iter().all(|&b| b == 0xA5), "len {len}: written beyond");
        }
    }
}
