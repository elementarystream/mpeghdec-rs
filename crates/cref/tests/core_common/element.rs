// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! C/Rust channel element pair (the memory `CAacDecoder_Init()` assigns to the channels of one
//! element), bitstream writer and scale factor Huffman encoder for the element level tests.

use super::*;
use mpeghdec_arith::ac_arith_coder::CArcoData;
use mpeghdec_core::aac_ram::*;
use mpeghdec_core::aacdecoder_lib::*;
use mpeghdec_core::block::*;
use mpeghdec_core::channel::*;
use mpeghdec_core::channelinfo::*;
use mpeghdec_core::conceal::*;
use mpeghdec_core::conceal_types::*;
use mpeghdec_core::overlapadd::OVERLAP_BUFFER_SIZE;
use mpeghdec_cref::core_channel::*;
use mpeghdec_cref::core_common::*;
use mpeghdec_cref::igf::*;
use mpeghdec_cref::testutil::Rng;
use mpeghdec_fdk::audio::*;
use mpeghdec_fdk::bitbuffer::BitMem;
use mpeghdec_fdk::bitstream::{FdkBitStream, FdkBsCfg};
use mpeghdec_fdk::mdct::mdct_init;
use mpeghdec_igf::igf_dec::*;

/// Nominal sampling rates of the entries of `sfbOffsetTables[0]`.
pub const SAMPLE_RATES: [u32; 13] =
    [96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350];

// ---------------------------------------------------------------------------------------------
// Element configuration and C/Rust element pair
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Cfg {
    pub nch: usize,
    pub lfe: bool,
    pub sr_idx: i32,
    pub sample_rate: u32,
    pub noise: bool,
    pub enf: bool,
    pub igf_start: u8,
    pub igf_stop: u8,
    pub high_res: u8,
    pub whitening: u8,
    pub after_tns: u8,
    pub indep_tiling: u8,
    pub use_enf: bool,
}

impl Cfg {
    pub fn el_flags(&self) -> u32 {
        let mut f = 0;
        if self.noise {
            f |= AC_EL_USAC_NOISE;
        }
        if self.lfe {
            f |= AC_EL_USAC_LFE | AC_EL_LFE;
        }
        if self.enf {
            f |= AC_EL_ENHANCED_NOISE;
            if self.use_enf {
                f |= AC_EL_IGF_USE_ENF;
            }
            if self.after_tns != 0 {
                f |= AC_EL_IGF_AFTER_TNS;
            }
            if self.indep_tiling != 0 {
                f |= AC_EL_IGF_INDEP_TILING;
            }
        }
        if self.nch == 2 {
            f |= AC_EL_USAC_CP_POSSIBLE;
        }
        f
    }

    pub fn random(g: &mut Rng) -> Cfg {
        let nch = if g.range(0, 2) == 0 { 1 } else { 2 };
        let lfe = nch == 1 && g.range(0, 5) == 0;
        let sr_idx = g.range(0, 12);
        let enf = !lfe && g.range(0, 2) != 0;
        Cfg {
            nch,
            lfe,
            sr_idx,
            sample_rate: SAMPLE_RATES[sr_idx as usize],
            noise: !lfe && g.range(0, 3) != 0,
            enf,
            igf_start: g.range(0, 15) as u8,
            igf_stop: g.range(0, 15) as u8,
            high_res: g.range(0, 1) as u8,
            whitening: g.range(0, 1) as u8,
            after_tns: g.range(0, 1) as u8,
            indep_tiling: g.range(0, 1) as u8,
            use_enf: g.range(0, 1) != 0,
        }
    }
}

/// A C and a Rust channel element, allocated and wired like `CAacDecoder_Init()`.
pub struct El {
    pub c: *mut (),
    pub cfg: Cfg,
    pub ch: Vec<Box<CAacDecoderChannelInfo>>,
    pub st: Vec<Box<CAacDecoderStaticChannelInfo>>,
    pub com: CAacDecoderCommonStaticData,
    pub cpe: Option<Box<CpePersistentData>>,
    pub spec: Vec<i32>,
    pub wb: CWorkBufferCore1,
    pub td2: Vec<i32>,
    pub igf_common: Box<[IgfPrivateDataCommon; 2]>,
    pub sri: SamplingRateInfo,
    pub conceal: CConcealParams,
}

impl El {
    pub fn new(cfg: Cfg) -> El {
        let cp = cfg.nch == 2;
        let c = unsafe {
            cref_core_el_new(
                cfg.nch as i32,
                cfg.sr_idx,
                cfg.sample_rate,
                cp as i32,
                cfg.enf as i32,
                cfg.igf_start as i32,
                cfg.igf_stop as i32,
                cfg.high_res as i32,
                cfg.whitening as i32,
                cfg.after_tns as i32,
                cfg.indep_tiling as i32,
            )
        };
        assert!(!c.is_null());
        let mut sri = SamplingRateInfo::default();
        assert_eq!(
            get_sampling_rate_info(&mut sri, 1024, cfg.sr_idx as u32, cfg.sample_rate),
            AAC_DEC_OK
        );
        let mut conceal = CConcealParams::default();
        cconcealment_init_common_data(&mut conceal);
        let mut com = CAacDecoderCommonStaticData::default();
        if cfg.nch == 2 {
            com.joint_stereo_data = Some(Box::default());
            if cp {
                com.cplx_prediction_data = Some(get_cplx_prediction_data());
            }
        }
        let mut cpe = None;
        if cp {
            let mut p = get_cpe_persistent_data();
            p.joint_stereo_persistent_data.win_groups_prev = 1;
            cpe = Some(p);
        }
        let mut ch = Vec::new();
        let mut st = Vec::new();
        for i in 0..cfg.nch {
            let mut c = get_aac_decoder_channel_info(i);
            let mut s = get_aac_decoder_static_channel_info(i);
            s.h_ar_co = Some(CArcoData::create());
            if cfg.enf {
                iis_igf_dec_lib_init(
                    &mut s.igf_static_data,
                    &mut c.igf_data,
                    cfg.igf_start,
                    cfg.igf_stop,
                    cfg.high_res,
                    cfg.whitening,
                    sri.sampling_rate,
                    1024,
                    sri.scale_factor_bands_long,
                    sri.number_of_scale_factor_bands_long as i32,
                    sri.scale_factor_bands_short,
                    sri.number_of_scale_factor_bands_short as i32,
                    1,
                    cfg.after_tns,
                    cfg.indep_tiling,
                );
            }
            ch.push(c);
            st.push(s);
        }
        st[0].nf_random_seed = 0x3039;
        if cfg.nch == 2 {
            st[1].nf_random_seed = 0x10932;
        }
        for i in 0..cfg.nch {
            ch[i].granule_length = 1024 / 8;
            ch[i].render_mode = AACDEC_RENDER_IMDCT;
            mdct_init(&mut st[i].i_mdct, OVERLAP_BUFFER_SIZE);
            cconcealment_init_channel_data(
                &mut st[i].concealment_info,
                &conceal,
                AACDEC_RENDER_IMDCT,
                1024,
            );
        }
        El {
            c,
            cfg,
            ch,
            st,
            com,
            cpe,
            spec: vec![0; 2048],
            wb: get_work_buffer_core1(),
            td2: vec![0; 3072],
            igf_common: Box::default(),
            sri,
            conceal,
        }
    }

    /// The channel arrays, the element memory and the sampling rate info (split borrows).
    #[allow(clippy::type_complexity)]
    pub fn split(
        &mut self,
    ) -> (
        &mut [Box<CAacDecoderChannelInfo>],
        &mut [Box<CAacDecoderStaticChannelInfo>],
        CChannelElementMem<'_>,
        &SamplingRateInfo,
    ) {
        let El { ch, st, com, cpe, spec, wb, td2, igf_common, sri, .. } = self;
        (
            ch,
            st,
            CChannelElementMem {
                spec,
                com_static_data: com,
                work_buffer_core1: wb,
                cpe_static_data: cpe.as_deref_mut(),
                igf_private_data_common: igf_common,
                time_data2: td2,
            },
            sri,
        )
    }

    pub fn c_ch(&self, i: usize) -> *mut () {
        unsafe { cref_core_el_ch(self.c, i as i32) }
    }

    pub fn c_st(&self, i: usize) -> *mut () {
        unsafe { cref_core_el_st(self.c, i as i32) }
    }

    /// Compares the complete state of the element.
    #[track_caller]
    pub fn check(&self, what: &str) {
        for i in 0..self.cfg.nch {
            let (cc, cs) = (self.c_ch(i), self.c_st(i));
            let c = c_dump(|o| unsafe { cref_core_ch_dump(cc, o) });
            assert_same(&format!("{what}: ch{i}"), &c, &dump_ch(&self.ch[i]));
            let c = c_dump(|o| unsafe { cref_core_st_dump(cs, o) });
            assert_same(&format!("{what}: st{i}"), &c, &dump_st(&self.st[i]));
            let (ci, si) = unsafe { (cref_core_ch_igf(cc), cref_core_st_igf(cs)) };
            let c = c_dump(|o| unsafe { cref_igf_dump_data(ci, o) });
            assert_same(&format!("{what}: igf data {i}"), &c, &dump_igf_data(&self.ch[i].igf_data));
            if self.cfg.enf {
                let c = c_dump(|o| unsafe { cref_igf_dump_static(si, ci, o) });
                assert_same(
                    &format!("{what}: igf static {i}"),
                    &c,
                    &dump_igf_static(&self.st[i].igf_static_data),
                );
            }
            let cm = unsafe { cref_core_el_igf_common(self.c, i as i32) };
            let c = c_dump(|o| unsafe { cref_igf_dump_common(cm, o) });
            assert_same(
                &format!("{what}: igf common {i}"),
                &c,
                &dump_igf_common(&self.igf_common[i]),
            );
        }
        let com = unsafe { cref_core_el_com(self.c) };
        if let Some(j) = &self.com.joint_stereo_data {
            let c = c_dump(|o| unsafe { cref_core_jsd_dump(com, o) });
            assert_same(&format!("{what}: jsd"), &c, &dump_jsd(j));
        }
        if let Some(p) = &self.com.cplx_prediction_data {
            let c = c_dump(|o| unsafe { cref_core_cpd_dump(com, o) });
            assert_same(&format!("{what}: cpd"), &c, &dump_cpd(p));
        }
        if let Some(p) = &self.cpe {
            let cp = unsafe { cref_core_el_cpe(self.c) };
            let c = c_dump(|o| unsafe { cref_core_cpe_dump(cp, o) });
            assert_same(&format!("{what}: cpe"), &c, &dump_cpe(p));
        }
        let n = 1024 * self.cfg.nch;
        let c = unsafe { std::slice::from_raw_parts(cref_core_el_spec(self.c), n) };
        assert_same(&format!("{what}: spectrum"), c, &self.spec[..n]);
        let c = unsafe { std::slice::from_raw_parts(cref_core_el_wb(self.c), 2048) };
        assert_same(&format!("{what}: work buffer core1"), c, &self.wb.buf[..]);
        let c = unsafe { std::slice::from_raw_parts(cref_core_el_time_data2(self.c), 3072) };
        assert_same(&format!("{what}: time data 2"), c, &self.td2);
    }

    /// Copies Rust memory buffers (spectrum, work buffer, time data) to C.
    pub fn sync_buffers_to_c(&self) {
        unsafe {
            std::slice::from_raw_parts_mut(cref_core_el_spec(self.c), 2048)
                .copy_from_slice(&self.spec);
            std::slice::from_raw_parts_mut(cref_core_el_wb(self.c), 2048)
                .copy_from_slice(&self.wb.buf[..]);
            std::slice::from_raw_parts_mut(cref_core_el_time_data2(self.c), 3072)
                .copy_from_slice(&self.td2);
        }
    }

    /// Copies the Rust channel structures (without IGF data) to C.
    pub fn sync_channels_to_c(&self) {
        for i in 0..self.cfg.nch {
            let d = dump_ch(&self.ch[i]);
            unsafe { cref_core_ch_load(self.c_ch(i), d.as_ptr()) };
            let d = dump_st(&self.st[i]);
            unsafe { cref_core_st_load(self.c_st(i), d.as_ptr()) };
        }
    }

    /// Copies the Rust element data (joint stereo data, complex prediction data, CPE persistent
    /// data) to C.
    pub fn sync_element_data_to_c(&self) {
        let com = unsafe { cref_core_el_com(self.c) };
        if let Some(j) = &self.com.joint_stereo_data {
            let d = dump_jsd(j);
            unsafe { cref_core_jsd_load(com, d.as_ptr()) };
        }
        if let Some(p) = &self.com.cplx_prediction_data {
            let d = dump_cpd(p);
            unsafe { cref_core_cpd_load(com, d.as_ptr()) };
        }
        if let Some(p) = &self.cpe {
            let d = dump_cpe(p);
            unsafe { cref_core_cpe_load(cref_core_el_cpe(self.c), d.as_ptr()) };
        }
    }

    /// Copies all Rust state except the IGF data to C.
    pub fn sync_to_c(&self) {
        self.sync_buffers_to_c();
        self.sync_channels_to_c();
        self.sync_element_data_to_c();
    }
}

impl Drop for El {
    fn drop(&mut self) {
        unsafe { cref_core_el_free(self.c) };
    }
}

// ---------------------------------------------------------------------------------------------
// Bitstream writer and scale factor Huffman encoder
// ---------------------------------------------------------------------------------------------

pub const BUF_SIZE: usize = 8192;

pub struct BitWriter {
    pub buf: Vec<u8>,
    pub pos: usize,
}

impl BitWriter {
    pub fn new() -> Self {
        BitWriter { buf: vec![0; BUF_SIZE], pos: 0 }
    }
    pub fn bit(&mut self, b: u32) {
        let byte = self.pos >> 3;
        let sh = 7 - (self.pos & 7);
        if b & 1 != 0 {
            self.buf[byte] |= 1 << sh;
        } else {
            self.buf[byte] &= !(1 << sh);
        }
        self.pos += 1;
    }
    pub fn bits(&mut self, v: u32, n: u32) {
        for i in (0..n).rev() {
            self.bit((v >> i) & 1);
        }
    }
    pub fn random(&mut self, g: &mut Rng, n: usize) {
        for _ in 0..n {
            self.bit(g.next_u32() & 1);
        }
    }
    /// A bitstream reader over a copy of the buffer, positioned at bit `pos`.
    pub fn reader_at(&self, pos: usize) -> (Vec<u8>, FdkBitStream) {
        let b = self.buf.clone();
        let mut bs = FdkBitStream::default();
        bs.init(
            &BitMem::from_slice(&b),
            BUF_SIZE as u32,
            (BUF_SIZE * 8) as u32,
            FdkBsCfg::BsReader,
        );
        bs.push_for(pos as u32);
        (b, bs)
    }
}

/// Codewords `(code, length)` of the scale factor Huffman codebook, by decoded index.
pub fn scl_codes() -> Vec<(u32, u32)> {
    let mut codes = vec![(0u32, 0u32); 121];
    for pat in 0u32..(1 << 19) {
        let mut buf = [0u8; 8];
        buf[..4].copy_from_slice(&(pat << 13).to_be_bytes());
        let mut bs = FdkBitStream::reader(&buf, 64);
        let v =
            cblock_decode_huffman_word_cb(&mut bs, &mpeghdec_core::aac_rom::HUFFMAN_CODE_BOOK_SCL);
        let len = 64 - bs.get_valid_bits() as u32;
        let code = pat >> (19 - len);
        let e = &mut codes[v as usize];
        if e.1 == 0 {
            *e = (code, len);
        } else {
            assert_eq!(*e, (code, len));
        }
    }
    assert!(codes.iter().all(|c| c.1 > 0));
    codes
}
