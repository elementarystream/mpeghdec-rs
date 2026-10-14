// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Minimal ISOBMFF/MP4 reader for MPEG-H audio tracks (`mhm1`, `mha1`).
//!
//! The reference decoder demo uses the Fraunhofer mmtisobmff library. This reader implements the
//! subset needed by the demo: the sample tables of plain (non-fragmented) files, and the seek
//! logic of `CSampleReader::sampleIndexForTimestamp` in `nearestSyncSample` mode.

use std::fmt;

#[derive(Debug)]
pub struct Mp4Error(pub String);

impl fmt::Display for Mp4Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Mp4Error {}

type Result<T> = std::result::Result<T, Mp4Error>;

fn err<T>(msg: impl Into<String>) -> Result<T> {
    Err(Mp4Error(msg.into()))
}

/// Codec of a track, derived from its sample entry type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Codec {
    /// `mhm1`: MHAS packets with in-band configuration.
    MpeghMhm,
    /// `mha1`: raw MPEG-H access units, configuration in the `mhaC` box.
    MpeghMha,
    Other,
}

/// Per-sample metadata.
#[derive(Clone, Copy, Debug)]
pub struct SampleInfo {
    pub offset: u64,
    pub size: u32,
    pub duration: u32,
    pub dts: i64,
    pub cts_offset: i64,
    pub is_sync: bool,
}

impl SampleInfo {
    /// Presentation timestamp (`dts + cts_offset`) in ticks of the track timescale.
    pub fn pts(&self) -> u64 {
        (self.dts + self.cts_offset).max(0) as u64
    }
}

#[derive(Debug)]
pub struct Track {
    pub track_id: u32,
    pub handler: [u8; 4],
    pub coding_name: [u8; 4],
    pub codec: Codec,
    pub timescale: u32,
    pub duration: u64,
    /// Sample rate from the audio sample entry.
    pub sample_rate: u32,
    /// `mpegh3daConfig()` from the `mhaC` box (for `mha1` tracks).
    pub mha_config: Option<Vec<u8>>,
    pub samples: Vec<SampleInfo>,
}

impl Track {
    pub fn max_sample_size(&self) -> u32 {
        self.samples.iter().map(|s| s.size).max().unwrap_or(0)
    }

    /// Index of the sample to start with when seeking to `seek_time` (in ticks of `timescale`)
    /// in `nearestSyncSample` mode, exactly like mmtisobmff's `sampleIndexForTimestamp`.
    pub fn nearest_sync_sample(&self, seek_pts: u64, seek_timescale: u32) -> usize {
        let user_seek_time = seek_pts as f64 / seek_timescale as f64;
        let mut acc_duration: u64 = 0;
        let mut frame_index = 0usize;
        let mut sync_sample_index = 0usize;
        let mut sync_sample_index_n_minus_one = 0usize;
        let mut found_user_seek_position = false;
        let mut user_seek_position_index = 0usize;
        for s in &self.samples {
            if s.is_sync {
                sync_sample_index_n_minus_one = sync_sample_index;
                sync_sample_index = frame_index;
                if found_user_seek_position {
                    break;
                }
            }
            let current_time = acc_duration as f64 / self.timescale as f64;
            if current_time >= user_seek_time && !found_user_seek_position {
                user_seek_position_index = frame_index;
                found_user_seek_position = true;
            }
            acc_duration += s.duration as u64;
            frame_index += 1;
        }
        if !found_user_seek_position {
            user_seek_position_index = frame_index;
        }
        let d_sync = (sync_sample_index as i64 - user_seek_position_index as i64).abs();
        let d_prev = (user_seek_position_index as i64 - sync_sample_index_n_minus_one as i64).abs();
        if d_sync <= d_prev {
            sync_sample_index
        } else {
            sync_sample_index_n_minus_one
        }
    }
}

/// An MP4 file: the sample tables of its tracks (the `moov` box). The sample payloads stay in the
/// file and are read on demand ([`Mp4File::read_sample`], [`SampleReader`](crate::reader::SampleReader)).
pub struct Mp4File {
    file: std::fs::File,
    /// Length of the file in bytes.
    pub len: u64,
    pub tracks: Vec<Track>,
}

/// Reads exactly `buf.len()` bytes at `offset` of `file`.
fn read_at(file: &std::fs::File, offset: u64, buf: &mut [u8]) -> std::io::Result<()> {
    use std::io::{Read, Seek};
    let mut f = file;
    f.seek(std::io::SeekFrom::Start(offset))?;
    f.read_exact(buf)
}

struct Reader<'a> {
    d: &'a [u8],
    p: usize,
}

impl<'a> Reader<'a> {
    fn new(d: &'a [u8]) -> Self {
        Reader { d, p: 0 }
    }
    fn need(&self, n: usize) -> Result<()> {
        if self.p + n > self.d.len() {
            return err("truncated box");
        }
        Ok(())
    }
    fn u8(&mut self) -> Result<u8> {
        self.need(1)?;
        self.p += 1;
        Ok(self.d[self.p - 1])
    }
    fn u16(&mut self) -> Result<u16> {
        self.need(2)?;
        self.p += 2;
        Ok(u16::from_be_bytes([self.d[self.p - 2], self.d[self.p - 1]]))
    }
    fn u32(&mut self) -> Result<u32> {
        self.need(4)?;
        self.p += 4;
        Ok(u32::from_be_bytes(self.d[self.p - 4..self.p].try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64> {
        self.need(8)?;
        self.p += 8;
        Ok(u64::from_be_bytes(self.d[self.p - 8..self.p].try_into().unwrap()))
    }
    fn bytes(&mut self, n: usize) -> Result<&'a [u8]> {
        self.need(n)?;
        self.p += n;
        Ok(&self.d[self.p - n..self.p])
    }
    fn fourcc(&mut self) -> Result<[u8; 4]> {
        Ok(self.bytes(4)?.try_into().unwrap())
    }
}

/// Iterates over the boxes in `d`, yielding `(type, payload)`.
fn boxes(d: &[u8]) -> Result<Vec<([u8; 4], &[u8])>> {
    let mut out = Vec::new();
    let mut r = Reader::new(d);
    while r.p + 8 <= d.len() {
        let start = r.p;
        let mut size = r.u32()? as u64;
        let typ = r.fourcc()?;
        if size == 1 {
            size = r.u64()?;
        } else if size == 0 {
            size = (d.len() - start) as u64;
        }
        let header = r.p - start;
        let end = (start as u64).checked_add(size).filter(|&e| e <= d.len() as u64);
        let Some(end) = end.filter(|_| size >= header as u64) else {
            return err(format!("invalid size of box '{}'", String::from_utf8_lossy(&typ)));
        };
        out.push((typ, &d[r.p..end as usize]));
        r.p = end as usize;
    }
    Ok(out)
}

/// The values of the run-length table `runs` (count, value) for `n` samples: runs beyond `n` are
/// ignored, missing values are the default.
fn expand_runs<T: Copy + Default>(runs: &[(u32, T)], n: usize) -> Vec<T> {
    let mut v = Vec::with_capacity(n);
    for &(count, value) in runs {
        let k = (count as usize).min(n - v.len());
        v.extend(std::iter::repeat_n(value, k));
    }
    v.resize(n, T::default());
    v
}

fn find<'a>(list: &[([u8; 4], &'a [u8])], typ: &[u8; 4]) -> Option<&'a [u8]> {
    list.iter().find(|(t, _)| t == typ).map(|(_, p)| *p)
}

#[derive(Default)]
struct SampleTables {
    stts: Vec<(u32, u32)>,
    ctts: Vec<(u32, i64)>,
    stss: Option<Vec<u32>>,
    sizes: Vec<u32>,
    stsc: Vec<(u32, u32)>,
    chunk_offsets: Vec<u64>,
}

impl Mp4File {
    /// Opens an MP4 file and parses its sample tables. Only the box headers of the top level
    /// and the `moov` box are read; the sample payloads are read on demand.
    pub fn open(path: &str) -> Result<Mp4File> {
        let io = |e: std::io::Error| Mp4Error(format!("{path}: {e}"));
        let file = std::fs::File::open(path).map_err(io)?;
        let len = file.metadata().map_err(io)?.len();
        /* walk the top level boxes (like `boxes()`, validating every box header) */
        let mut moov = None;
        let mut p: u64 = 0;
        while p + 8 <= len {
            let mut h = [0u8; 16];
            read_at(&file, p, &mut h[..8]).map_err(io)?;
            let mut size = u32::from_be_bytes(h[0..4].try_into().unwrap()) as u64;
            let typ: [u8; 4] = h[4..8].try_into().unwrap();
            let mut header = 8;
            if size == 1 {
                if p + 16 > len {
                    return err("truncated box");
                }
                read_at(&file, p + 8, &mut h[8..16]).map_err(io)?;
                size = u64::from_be_bytes(h[8..16].try_into().unwrap());
                header = 16;
            } else if size == 0 {
                size = len - p;
            }
            if size < header || p.checked_add(size).is_none_or(|e| e > len) {
                return err(format!("invalid size of box '{}'", String::from_utf8_lossy(&typ)));
            }
            if &typ == b"moov" && moov.is_none() {
                moov = Some((p + header, size - header));
            }
            p += size;
        }
        let Some((moov_pos, moov_len)) = moov else {
            return err("no moov box found");
        };
        let mut data = vec![0u8; moov_len as usize];
        read_at(&file, moov_pos, &mut data).map_err(io)?;
        let tracks = Self::parse_moov(&data, len)?;
        Ok(Mp4File { file, len, tracks })
    }

    /// Parses the payload of the `moov` box.
    fn parse_moov(moov: &[u8], file_len: u64) -> Result<Vec<Track>> {
        let mut tracks = Vec::new();
        for (t, trak) in boxes(moov)? {
            if &t == b"trak" {
                tracks.push(Self::parse_trak(trak, file_len)?);
            }
        }
        Ok(tracks)
    }

    /// `file_len`: the size of the file, which bounds the media data of the track.
    fn parse_trak(trak: &[u8], file_len: u64) -> Result<Track> {
        let trak = boxes(trak)?;
        let tkhd = find(&trak, b"tkhd").ok_or(Mp4Error("missing tkhd".into()))?;
        let mut r = Reader::new(tkhd);
        let version = r.u8()?;
        r.bytes(3)?;
        let track_id = if version == 1 {
            r.u64()?;
            r.u64()?;
            r.u32()?
        } else {
            r.u32()?;
            r.u32()?;
            r.u32()?
        };
        let mdia = boxes(find(&trak, b"mdia").ok_or(Mp4Error("missing mdia".into()))?)?;
        let mdhd = find(&mdia, b"mdhd").ok_or(Mp4Error("missing mdhd".into()))?;
        let mut r = Reader::new(mdhd);
        let version = r.u8()?;
        r.bytes(3)?;
        let (timescale, duration) = if version == 1 {
            r.u64()?;
            r.u64()?;
            (r.u32()?, r.u64()?)
        } else {
            r.u32()?;
            r.u32()?;
            (r.u32()?, r.u32()? as u64)
        };
        let mut handler = [0u8; 4];
        if let Some(hdlr) = find(&mdia, b"hdlr") {
            let mut r = Reader::new(hdlr);
            r.bytes(8)?;
            handler = r.fourcc()?;
        }
        let minf = boxes(find(&mdia, b"minf").ok_or(Mp4Error("missing minf".into()))?)?;
        let stbl = boxes(find(&minf, b"stbl").ok_or(Mp4Error("missing stbl".into()))?)?;

        // Sample description
        let mut coding_name = [0u8; 4];
        let mut sample_rate = 0;
        let mut mha_config = None;
        if let Some(stsd) = find(&stbl, b"stsd") {
            let mut r = Reader::new(stsd);
            r.bytes(4)?;
            let count = r.u32()?;
            if count > 0 {
                let entries = boxes(&stsd[8..])?;
                if let Some((typ, entry)) = entries.first() {
                    coding_name = *typ;
                    // AudioSampleEntry: 6 reserved, data_reference_index, 8 reserved,
                    // channelcount, samplesize, pre_defined, reserved, samplerate (16.16)
                    let mut r = Reader::new(entry);
                    r.bytes(6 + 2 + 8 + 2 + 2 + 2 + 2)?;
                    sample_rate = r.u32()? >> 16;
                    for (t, payload) in boxes(&entry[r.p..])? {
                        if &t == b"mhaC" {
                            let mut r = Reader::new(payload);
                            r.u8()?; // configurationVersion
                            r.u8()?; // mpegh3daProfileLevelIndication
                            r.u8()?; // referenceChannelLayout
                            let len = r.u16()? as usize;
                            mha_config = Some(r.bytes(len)?.to_vec());
                        }
                    }
                }
            }
        }
        let codec = match &coding_name {
            b"mhm1" | b"mhm2" => Codec::MpeghMhm,
            b"mha1" | b"mha2" => Codec::MpeghMha,
            _ => Codec::Other,
        };

        let mut t = SampleTables::default();
        if let Some(b) = find(&stbl, b"stts") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            for _ in 0..r.u32()? {
                t.stts.push((r.u32()?, r.u32()?));
            }
        }
        if let Some(b) = find(&stbl, b"ctts") {
            let mut r = Reader::new(b);
            let version = r.u8()?;
            r.bytes(3)?;
            for _ in 0..r.u32()? {
                let count = r.u32()?;
                let off = r.u32()?;
                let off = if version == 1 { off as i32 as i64 } else { off as i64 };
                t.ctts.push((count, off));
            }
        }
        if let Some(b) = find(&stbl, b"stss") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            let mut v = Vec::new();
            for _ in 0..r.u32()? {
                v.push(r.u32()?);
            }
            t.stss = Some(v);
        }
        if let Some(b) = find(&stbl, b"stsz") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            let size = r.u32()?;
            let count = r.u32()?;
            /* a constant sample size does not need an entry per sample: the samples have to fit
             * into the file, and an access unit has at least 2 bytes */
            if size != 0 && size.max(2) as u64 * count as u64 > file_len {
                return err("sample sizes exceed the file");
            }
            for _ in 0..count {
                t.sizes.push(if size != 0 { size } else { r.u32()? });
            }
        } else if let Some(b) = find(&stbl, b"stz2") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            r.bytes(3)?;
            let field = r.u8()?;
            let count = r.u32()?;
            let mut i = 0;
            while i < count {
                match field {
                    4 => {
                        let v = r.u8()?;
                        t.sizes.push((v >> 4) as u32);
                        if i + 1 < count {
                            t.sizes.push((v & 15) as u32);
                        }
                        i += 2;
                        continue;
                    }
                    8 => t.sizes.push(r.u8()? as u32),
                    16 => t.sizes.push(r.u16()? as u32),
                    _ => return err("invalid stz2 field size"),
                }
                i += 1;
            }
        }
        if let Some(b) = find(&stbl, b"stsc") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            for _ in 0..r.u32()? {
                let first_chunk = r.u32()?;
                let samples_per_chunk = r.u32()?;
                r.u32()?; // sample_description_index
                t.stsc.push((first_chunk, samples_per_chunk));
            }
        }
        if let Some(b) = find(&stbl, b"stco") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            for _ in 0..r.u32()? {
                t.chunk_offsets.push(r.u32()? as u64);
            }
        } else if let Some(b) = find(&stbl, b"co64") {
            let mut r = Reader::new(b);
            r.bytes(4)?;
            for _ in 0..r.u32()? {
                t.chunk_offsets.push(r.u64()?);
            }
        }

        let samples = Self::build_samples(&t)?;
        Ok(Track {
            track_id,
            handler,
            coding_name,
            codec,
            timescale,
            duration,
            sample_rate,
            mha_config,
            samples,
        })
    }

    fn build_samples(t: &SampleTables) -> Result<Vec<SampleInfo>> {
        let n = t.sizes.len();
        let mut samples = Vec::with_capacity(n);
        // Offsets from chunk tables
        let mut offsets = Vec::with_capacity(n);
        for (ci, &chunk_offset) in t.chunk_offsets.iter().enumerate() {
            let chunk = ci as u32 + 1;
            let per_chunk = t
                .stsc
                .iter()
                .rev()
                .find(|(first, _)| *first <= chunk)
                .map(|(_, s)| *s)
                .unwrap_or(0);
            let mut off = chunk_offset;
            for _ in 0..per_chunk {
                if offsets.len() == n {
                    break;
                }
                offsets.push(off);
                off = off.saturating_add(t.sizes[offsets.len() - 1] as u64);
            }
        }
        if offsets.len() != n {
            return err("inconsistent sample tables");
        }
        let durations = expand_runs(&t.stts, n);
        let cts = expand_runs(&t.ctts, n);
        let mut dts: i64 = 0;
        for i in 0..n {
            let is_sync = match &t.stss {
                Some(list) => list.binary_search(&(i as u32 + 1)).is_ok(),
                None => true,
            };
            samples.push(SampleInfo {
                offset: offsets[i],
                size: t.sizes[i],
                duration: durations[i],
                dts,
                cts_offset: cts[i],
                is_sync,
            });
            dts += durations[i] as i64;
        }
        Ok(samples)
    }

    /// Whether the payload of sample `s` lies within the file.
    pub fn sample_in_file(&self, s: &SampleInfo) -> bool {
        s.offset.checked_add(s.size as u64).is_some_and(|e| e <= self.len)
    }

    /// Reads the payload of sample `s`.
    pub fn read_sample(&self, s: &SampleInfo) -> Result<Vec<u8>> {
        if !self.sample_in_file(s) {
            return err("sample data out of file bounds");
        }
        let mut v = vec![0u8; s.size as usize];
        read_at(&self.file, s.offset, &mut v).map_err(|e| Mp4Error(e.to_string()))?;
        Ok(v)
    }
}

#[cfg(test)]
mod robustness_tests {
    use super::*;

    /// A 64 bit box size whose end wraps around is rejected.
    #[test]
    fn box_size_overflow_is_rejected() {
        let mut d = Vec::new();
        d.extend_from_slice(&8u32.to_be_bytes());
        d.extend_from_slice(b"free");
        d.extend_from_slice(&1u32.to_be_bytes());
        d.extend_from_slice(b"abcd");
        d.extend_from_slice(&(u64::MAX - 7).to_be_bytes());
        assert!(boxes(&d).is_err());
    }

    /// Run lengths are expanded only up to the number of samples.
    #[test]
    fn run_lengths_are_bounded_by_the_sample_count() {
        let v = expand_runs(&[(u32::MAX, 1024u32), (5, 7)], 3);
        assert_eq!(v, [1024, 1024, 1024]);
        let v = expand_runs::<i64>(&[(1, -2)], 3);
        assert_eq!(v, [-2, 0, 0]);
    }
}
