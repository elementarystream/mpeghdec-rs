// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Splitting of raw MHAS streams (ISO/IEC 23008-3, clause 14) into access units.
//!
//! This is not part of the reference decoder demo, which only reads MP4 files. An access unit is
//! the sequence of MHAS packets up to and including the next `PACTYP_MPEGH3DAFRAME` packet.

pub const PACTYP_MPEGH3DACFG: u32 = 1;
pub const PACTYP_MPEGH3DAFRAME: u32 = 2;

struct Bits<'a> {
    d: &'a [u8],
    p: usize,
}

impl Bits<'_> {
    fn u(&mut self, n: u32) -> Option<u32> {
        let mut v: u64 = 0;
        for _ in 0..n {
            let byte = *self.d.get(self.p >> 3)?;
            v = (v << 1) | ((byte >> (7 - (self.p & 7))) & 1) as u64;
            self.p += 1;
        }
        Some(v as u32)
    }

    fn escaped(&mut self, n1: u32, n2: u32, n3: u32) -> Option<u32> {
        let mut v = self.u(n1)?;
        if v == (1u32 << n1) - 1 {
            let e = self.u(n2)?;
            v = v.checked_add(e)?;
            if e == ((1u64 << n2) - 1) as u32 {
                v = v.checked_add(self.u(n3)?)?;
            }
        }
        Some(v)
    }
}

/// Header of one MHAS packet.
#[derive(Clone, Copy, Debug)]
pub struct PacketHeader {
    pub packet_type: u32,
    pub label: u32,
    pub length: u32,
    pub header_bytes: usize,
}

/// Parses the MHAS packet header at the start of `d`.
pub fn parse_packet_header(d: &[u8]) -> Option<PacketHeader> {
    let mut b = Bits { d, p: 0 };
    let packet_type = b.escaped(3, 8, 8)?;
    let label = b.escaped(2, 8, 32)?;
    let length = b.escaped(11, 24, 24)?;
    Some(PacketHeader { packet_type, label, length, header_bytes: b.p.div_ceil(8) })
}

/// Splits an MHAS stream into access units, returned as `(offset, size)` pairs. A trailing
/// incomplete access unit is returned as well.
pub fn split_access_units(d: &[u8]) -> Vec<(usize, usize)> {
    let mut aus = Vec::new();
    let mut start = 0;
    let mut off = 0;
    while off < d.len() {
        let Some(h) = parse_packet_header(&d[off..]) else { break };
        let end = off + h.header_bytes + h.length as usize;
        if end > d.len() {
            off = d.len();
            break;
        }
        off = end;
        if h.packet_type == PACTYP_MPEGH3DAFRAME {
            aus.push((start, off - start));
            start = off;
        }
    }
    if start < off.min(d.len()) {
        aus.push((start, d.len() - start));
    }
    aus
}

/// Like [`split_access_units`] for the stream in the file of `r` (`len` bytes): only the packet
/// headers are read, the payloads are skipped.
pub fn split_access_units_reader(
    r: &mut crate::reader::SampleReader,
    len: u64,
) -> std::io::Result<Vec<(u64, usize)>> {
    /* the longest packet header: 3 + 8 + 8, 2 + 8 + 32 and 11 + 24 + 24 bits */
    const MAX_HEADER_BYTES: u64 = 15;
    let mut aus = Vec::new();
    let mut start: u64 = 0;
    let mut off: u64 = 0;
    while off < len {
        let n = (len - off).min(MAX_HEADER_BYTES) as usize;
        let Some(h) = parse_packet_header(r.read(off, n)?) else { break };
        let end = off + h.header_bytes as u64 + h.length as u64;
        if end > len {
            off = len;
            break;
        }
        off = end;
        if h.packet_type == PACTYP_MPEGH3DAFRAME {
            aus.push((start, (off - start) as usize));
            start = off;
        }
    }
    if start < off.min(len) {
        aus.push((start, (len - start) as usize));
    }
    Ok(aus)
}

/// Whether the access unit `d` contains a config packet (a random access point).
pub fn has_config_packet(d: &[u8]) -> bool {
    let mut off = 0;
    while off < d.len() {
        let Some(h) = parse_packet_header(&d[off..]) else { break };
        if h.packet_type == PACTYP_MPEGH3DACFG {
            return true;
        }
        off += h.header_bytes + h.length as usize;
    }
    false
}

/// Reads a `.pts.txt` side file (`byte_offset size pts` per line, 90 kHz PTS).
pub fn read_pts_file(path: &str) -> std::io::Result<Vec<(usize, usize, u64)>> {
    let text = std::fs::read_to_string(path)?;
    let mut out = Vec::new();
    for line in text.lines() {
        let v: Vec<u64> = line.split_whitespace().filter_map(|x| x.parse().ok()).collect();
        if v.len() >= 3 {
            out.push((v[0] as usize, v[1] as usize, v[2]));
        }
    }
    Ok(out)
}
