// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of FDK_bitstream.h / FDK_bitbuffer.cpp: random operation sequences are
//! applied to a C and a Rust bitstream over identical buffers.
#![cfg(not(cref_missing))]

use mpeghdec_cref::fdk_bitstream::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};

fn state(bs: &FdkBitStream) -> [u32; 6] {
    [
        bs.cache_word,
        bs.bits_in_cache,
        bs.h_bit_buf.bit_ndx(),
        bs.h_bit_buf.valid_bits as u32,
        bs.h_bit_buf.read_offset(),
        bs.h_bit_buf.write_offset(),
    ]
}

fn run(seed: u64, writer: bool) {
    let mut g = Rng::new(seed);
    let size = 1u32 << g.range(4, 9);
    let mut buf_c: Vec<u8> = (0..size).map(|_| g.next_u32() as u8).collect();
    let mem_r = BitMem::from_slice(&buf_c);
    let valid = if writer { 0 } else { g.range(0, (size * 8) as i32) as u32 };
    let cfg = if writer { FdkBsCfg::BsWriter } else { FdkBsCfg::BsReader };
    let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, valid, cfg as i32) };
    let mut r = FdkBitStream::default();
    r.init(&mem_r, size, valid, cfg);
    let mut st = [0u32; 6];
    for step in 0..400 {
        let op = if writer { g.range(0, 7) } else { g.range(0, 11) };
        let n = g.range(0, 32) as u32;
        let (a, c): (u32, u32) = unsafe {
            match (writer, op) {
                (false, 0) => (r.read_bits(n), cref_bs_readBits(hc, n)),
                (false, 1) => (r.read_bit(), cref_bs_readBit(hc)),
                (false, 2) => (r.read_2bits(), cref_bs_read2Bits(hc)),
                (false, 3) => {
                    let n = g.range(1, 31) as u32;
                    (r.read_bits_bwd(n), cref_bs_readBitsBwd(hc, n))
                }
                (false, 4) => {
                    let (x, y, z) = (g.range(1, 8), g.range(1, 8), g.range(1, 16));
                    (r.escaped_value(x, y, z), cref_bs_escapedValue(hc, x, y, z))
                }
                (false, 5) => {
                    r.sync_cache();
                    cref_bs_syncCache(hc);
                    let n = g.range(1, 32) as u32;
                    (r.get_bits(n), cref_bs_getBits(hc, n))
                }
                (false, 6) => {
                    let n = g.range(0, 31) as u32;
                    r.push_back(n);
                    cref_bs_pushBack(hc, n);
                    (0, 0)
                }
                (false, 7) => {
                    r.push_for(n);
                    cref_bs_pushFor(hc, n);
                    (0, 0)
                }
                (false, 8) => {
                    let anchor = g.range(0, 4096) as u32;
                    r.byte_align(anchor);
                    cref_bs_byteAlign(hc, anchor);
                    (0, 0)
                }
                (false, 9) => (r.get_valid_bits() as u32, cref_bs_getValidBits(hc) as u32),
                (false, _) => {
                    let n = g.range(-40, 40);
                    r.push_bi_directional(n);
                    cref_bs_pushBiDirectional(hc, n);
                    (0, 0)
                }
                (true, 0..=2) => {
                    let v = g.next_u32();
                    (r.write_bits(v, n) as u32, cref_bs_writeBits(hc, v, n))
                }
                (true, 3) => {
                    let (x, y, z) =
                        (g.range(1, 8) as u32, g.range(1, 8) as u32, g.range(1, 16) as u32);
                    let v = g.next_u32() % 3000;
                    (
                        r.write_escaped_value(v, x, y, z) as u32,
                        cref_bs_writeEscapedValue(hc, v, x, y, z),
                    )
                }
                (true, 4) => {
                    r.sync_cache();
                    cref_bs_syncCache(hc);
                    (0, 0)
                }
                (true, 5) => {
                    let anchor = g.range(0, 4096) as u32;
                    r.byte_align(anchor);
                    cref_bs_byteAlign(hc, anchor);
                    (0, 0)
                }
                (true, _) => (r.get_free_bits() as u32, cref_bs_getFreeBits(hc) as u32),
            }
        };
        unsafe { cref_bs_state(hc, st.as_mut_ptr()) };
        assert_eq!(a, c, "seed {seed} step {step} op {op} n {n}");
        assert_eq!(state(&r), st, "state seed {seed} step {step} op {op} n {n}");
        assert_eq!(mem_r.to_vec(), buf_c, "buffer seed {seed} step {step} op {op}");
    }
    unsafe { cref_bs_free(hc) };
}

#[test]
fn reader_ops() {
    for seed in 0..500 {
        run(seed, false);
    }
}

#[test]
fn writer_ops() {
    for seed in 1000..1500 {
        run(seed, true);
    }
}

#[test]
fn feed_fetch_copy() {
    let mut g = Rng::new(77);
    for _ in 0..200 {
        let size = 1u32 << g.range(4, 8);
        let mut buf_c = vec![0u8; size as usize];
        let mem_r = BitMem::from_slice(&buf_c);
        let hc = unsafe { cref_bs_new(buf_c.as_mut_ptr(), size, 0, 0) };
        let mut r = FdkBitStream::create(&mem_r, size, FdkBsCfg::BsReader);
        for _ in 0..20 {
            let input: Vec<u8> = (0..g.range(0, 300)).map(|_| g.next_u32() as u8).collect();
            let mut vc = g.range(0, input.len() as i32) as u32;
            let mut vr = vc;
            r.feed_buffer(&input, &mut vr);
            unsafe { cref_bs_feedBuffer(hc, input.as_ptr(), input.len() as u32, &mut vc) };
            assert_eq!(vr, vc);
            let n = g.range(0, 40) as u32;
            let a = r.read_bits(n.min(32));
            let b = unsafe { cref_bs_readBits(hc, n.min(32)) };
            assert_eq!(a, b);
            let mut out_r = vec![0u8; 64];
            let mut out_c = vec![0u8; 64];
            let mut wr = g.range(0, 64) as u32;
            let mut wc = wr;
            r.fetch_buffer(&mut out_r, &mut wr);
            unsafe { cref_bs_fetchBuffer(hc, out_c.as_mut_ptr(), &mut wc) };
            assert_eq!((wr, &out_r), (wc, &out_c));
            assert_eq!(mem_r.to_vec(), buf_c);
        }
        // copy between two bitstreams
        let mut dst_c = vec![0u8; 64];
        let dst_r = BitMem::from_slice(&dst_c);
        let hdc = unsafe { cref_bs_new(dst_c.as_mut_ptr(), 64, 0, 0) };
        let mut dr = FdkBitStream::create(&dst_r, 64, FdkBsCfg::BsReader);
        let mut vc = g.range(0, 100) as u32;
        let mut vr = vc;
        dr.copy_buffer(&mut r, &mut vr);
        unsafe { cref_bs_copyBuffer(hdc, hc, &mut vc) };
        assert_eq!(vr, vc);
        assert_eq!(dst_r.to_vec(), dst_c);
        unsafe {
            cref_bs_free(hc);
            cref_bs_free(hdc);
        }
    }
}
