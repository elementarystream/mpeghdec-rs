// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of FDK_crc.cpp: random crc configurations and nested crc regions over
//! random reader and writer bitstreams.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::fdk_crc::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::crc::{FdkCrcInfo, MAX_CRC_REGS};

fn state(c: &FdkCrcInfo) -> Vec<i32> {
    let mut v = vec![
        c.crc_value as i32,
        c.crc_poly as i32,
        c.crc_mask as i32,
        c.start_value as i32,
        c.crc_len as i32,
        c.reg_start as i32,
        c.reg_stop as i32,
        c.p_crc_lookup.map_or(-1, |t| t[1] as i32),
    ];
    for r in &c.crc_reg_data {
        v.extend_from_slice(&[r.is_active as i32, r.max_bits, r.bit_buf_cnt_bits, r.valid_bits]);
    }
    v
}

fn c_state(h: *mut ()) -> Vec<i32> {
    let mut v = vec![0i32; 8 + 4 * MAX_CRC_REGS];
    unsafe { cref_crc_state(h, v.as_mut_ptr()) };
    v
}

fn bs_state(bs: &FdkBitStream) -> [u32; 6] {
    [
        bs.cache_word,
        bs.bits_in_cache,
        bs.h_bit_buf.bit_ndx(),
        bs.h_bit_buf.valid_bits as u32,
        bs.h_bit_buf.read_offset(),
        bs.h_bit_buf.write_offset(),
    ]
}

fn random_config(g: &mut Rng) -> (u32, u32, u32) {
    let poly = match g.range(0, 5) {
        0 => 0x8021,
        1 => 0x8005,
        2 => 0x1021,
        3 => 0x001d,
        4 => 0x04c1_1db7,
        _ => g.next_u32(),
    };
    let len = match g.range(0, 3) {
        0 | 1 => 16,
        2 => 8,
        _ => g.range(0, 32) as u32,
    };
    let start = match g.range(0, 2) {
        0 => 0,
        1 => 0xffff,
        _ => g.next_u32(),
    };
    (poly, start, len)
}

fn run(seed: u64, writer: bool) {
    let mut g = Rng::new(seed);
    let size = 1u32 << g.range(6, 10);
    let mut buf_c: Vec<u8> = (0..size).map(|_| g.next_u32() as u8).collect();
    let mem_r = BitMem::from_slice(&buf_c);
    let cfg = if writer { FdkBsCfg::BsWriter } else { FdkBsCfg::BsReader };
    let valid = if writer { 0 } else { size * 8 };
    let bc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, valid, cfg as i32) };
    let mut br = FdkBitStream::default();
    br.init(&mem_r, size, valid, cfg);

    let hc = unsafe { cref_crc_new() };
    let mut r = FdkCrcInfo::default();
    let (poly, start, len) = random_config(&mut g);
    r.init(poly, start, len);
    unsafe { cref_crc_init(hc, poly, start, len) };

    // Remaining bit budget so that readers never run beyond the buffer end.
    let mut budget = (size * 8) as i32 - 64;
    let mut active: Vec<i32> = Vec::new();
    for step in 0..300 {
        let op = g.range(0, 9);
        let (a, c) = match op {
            0 | 1 if active.len() < MAX_CRC_REGS => {
                let m_bits = match g.range(0, 3) {
                    0 | 1 => 0,
                    2 => g.range(1, 300),
                    _ => -g.range(1, 300),
                };
                let a = r.start_reg(&mut br, m_bits);
                let c = unsafe { cref_crc_startReg(hc, bc, m_bits) };
                active.push(a);
                (a as u32, c as u32)
            }
            2 | 3 if !active.is_empty() => {
                let reg = active.remove(0);
                let a = r.end_reg(&mut br, reg);
                let c = unsafe { cref_crc_endReg(hc, bc, reg) };
                (a as u32, c as u32)
            }
            4 => (r.get_crc() as u32, unsafe { cref_crc_getCRC(hc) }),
            5 if active.is_empty() && g.range(0, 5) == 0 => {
                let (poly, start, len) = random_config(&mut g);
                r.init(poly, start, len);
                unsafe { cref_crc_init(hc, poly, start, len) };
                (0, 0)
            }
            _ => {
                let n = g.range(0, 32).min(budget.max(0)) as u32;
                budget -= n as i32;
                if writer {
                    let v = g.next_u32();
                    (br.write_bits(v, n) as u32, unsafe { cref_bs_writeBits(bc, v, n) })
                } else {
                    (br.read_bits(n), unsafe { cref_bs_readBits(bc, n) })
                }
            }
        };
        assert_eq!(a, c, "seed {seed} step {step} op {op}");
        assert_eq!(state(&r), c_state(hc), "crc state seed {seed} step {step} op {op}");
        let mut st = [0u32; 6];
        unsafe { cref_bs_state(bc, st.as_mut_ptr()) };
        assert_eq!(bs_state(&br), st, "bs state seed {seed} step {step} op {op}");
    }
    // Close all open regions.
    while !active.is_empty() {
        let reg = active.remove(0);
        r.end_reg(&mut br, reg);
        unsafe { cref_crc_endReg(hc, bc, reg) };
        assert_eq!(state(&r), c_state(hc), "final seed {seed}");
    }
    assert_eq!(r.get_crc() as u32, unsafe { cref_crc_getCRC(hc) });
    r.reset();
    unsafe { cref_crc_reset(hc) };
    assert_eq!(state(&r), c_state(hc));
    unsafe {
        cref_crc_free(hc);
        cref_bs_free(bc);
    }
}

#[test]
fn crc_reader() {
    for seed in 0..1500 {
        run(seed, false);
    }
}

#[test]
fn crc_writer() {
    for seed in 5000..6500 {
        run(seed, true);
    }
}

#[test]
fn crc_all_lengths_and_polys() {
    // Single region over a byte-aligned and unaligned range for every crc length.
    let mut g = Rng::new(99);
    for len in 0..=32u32 {
        for _ in 0..50 {
            let size = 256u32;
            let mut buf_c: Vec<u8> = (0..size).map(|_| g.next_u32() as u8).collect();
            let buf_r = buf_c.clone();
            let bc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, size * 8, 0) };
            let mut br = FdkBitStream::default();
            br.init(&BitMem::from_slice(&buf_r), size, size * 8, FdkBsCfg::BsReader);
            let hc = unsafe { cref_crc_new() };
            let mut r = FdkCrcInfo::default();
            let poly = if g.range(0, 1) == 0 { 0x8021 } else { g.next_u32() };
            let start = g.next_u32();
            r.init(poly, start, len);
            unsafe { cref_crc_init(hc, poly, start, len) };
            let skip = g.range(0, 32) as u32;
            br.read_bits(skip);
            unsafe { cref_bs_readBits(bc, skip) };
            let m_bits = g.range(-600, 600);
            let reg = r.start_reg(&mut br, m_bits);
            unsafe { cref_crc_startReg(hc, bc, m_bits) };
            let n = g.range(0, 1000);
            br.push_for(n as u32);
            unsafe { cref_bs_pushFor(bc, n as u32) };
            r.end_reg(&mut br, reg);
            unsafe { cref_crc_endReg(hc, bc, reg) };
            assert_eq!(state(&r), c_state(hc), "len {len}");
            assert_eq!(r.get_crc() as u32, unsafe { cref_crc_getCRC(hc) });
            unsafe {
                cref_crc_free(hc);
                cref_bs_free(bc);
            }
        }
    }
}
