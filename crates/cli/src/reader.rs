// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! On-demand reading of the access units of the input file (not in C: the demo reads the samples
//! through mmtisobmff). Only the sample tables are kept in memory.

/// Reads sample payloads (or any byte ranges) of a file on demand into a reused buffer.
///
/// Consecutive reads of adjacent ranges (the samples of a track in the `mdat` box usually are)
/// are served by a read buffer without seeking.
pub struct SampleReader {
    file: std::io::BufReader<std::fs::File>,
    /// Current position of `file`.
    pos: u64,
    buf: Vec<u8>,
}

impl SampleReader {
    /// Opens `path` for reading.
    pub fn open(path: &str) -> std::io::Result<SampleReader> {
        let file = std::fs::File::open(path)?;
        Ok(SampleReader {
            file: std::io::BufReader::with_capacity(1 << 16, file),
            pos: 0,
            buf: Vec::new(),
        })
    }

    /// Reads `size` bytes at `offset`; the returned slice is valid until the next read.
    pub fn read(&mut self, offset: u64, size: usize) -> std::io::Result<&[u8]> {
        use std::io::Read;
        if offset != self.pos {
            let delta = offset as i128 - self.pos as i128;
            if let Ok(d) = i64::try_from(delta) {
                self.file.seek_relative(d)?;
            } else {
                std::io::Seek::seek(&mut self.file, std::io::SeekFrom::Start(offset))?;
            }
            self.pos = offset;
        }
        self.buf.resize(size, 0);
        let r = self.file.read_exact(&mut self.buf);
        if r.is_err() {
            /* the position is unknown after a failed read */
            self.pos = std::io::Seek::stream_position(&mut self.file).unwrap_or(u64::MAX);
            r?;
        }
        self.pos += size as u64;
        Ok(&self.buf)
    }
}
