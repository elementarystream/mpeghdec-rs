// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libMpeghDec`, the MPEG-H 3D Audio core decoder.
//!
//! - Frame level: [`aacdecoder_lib`] is the decoder API (`aacDecoder_*()`, `CStreamInfo`, the
//!   transport decoder callbacks); [`aacdecoder`] holds the decoder instance
//!   ([`aacdecoder::AacDecoderInstance`]), the configuration (`CAacDecoder_Init()`), the access
//!   unit decoding (`CAacDecoder_DecodeFrame()`) and the earcon mixer; [`ui`] the user
//!   interactivity, loudness and DRC interface. The rendering chain (user interactivity, DRC,
//!   format conversion ([`format_converter_glue`]), object rendering, sample rate conversion,
//!   truncation, cross-fade, earcons, limiter and delay compensation) is part of
//!   `aacDecoder_DecodeFrame()`; it is ported in [`render`], with the state it uses in
//!   [`render::RenderState`], which also implements the frame pipelining (not in C: the
//!   rendering of an access unit runs on a render thread, [`pipeline`], while the next access
//!   unit is decoded). `mpeghdecoder.cpp` is ported in [`mpeghdecoder`]; the safe Rust API over
//!   it is the `mpeghdec` crate.
//! - Below the frame level: the channel element parser and decoder and the per channel tools
//!   (described below).
//!
//! # Frame level
//!
//! The transport decoder (`hInput`) is owned by the instance but taken out of it
//! ([`aacdecoder::AacDecoderInstance::h_input`] is `None` then) while the decoder calls into it:
//! the transport decoder calls back into the instance through [`mpeghdec_tpdec::TpCallbacks`].
//! `pUsacConfig[s]` points into the configs of the transport decoder (`hTp->asc[]`); it is stored
//! as the index of the config and resolved with the config array, which the callbacks receive
//! and the other functions take from the transport decoder. `transportDec_SetParam()` calls from
//! within the configuration callback are queued and applied by the transport decoder when the
//! callback returns.
//!
//! # Ownership model
//!
//! The C++ decoder shares memory through pointers stored in the channel structures. The Rust
//! structures do not store these pointers; the memory is owned by the decoder (or by the
//! structures themselves) and passed to the functions explicitly, reproducing exactly which
//! memory the C++ code reads and writes (including stale content of reused buffers).
//!
//! ## Decoder owned memory (`AAC_DECODER_INSTANCE`)
//!
//! | C++ | Rust | Notes |
//! | --- | --- | --- |
//! | `pAacDecoderChannelInfo[28]` | `Vec<Box<`[`channelinfo::CAacDecoderChannelInfo`]`>>` | functions take `&mut [Box<CAacDecoderChannelInfo>]` starting at the first channel of the element (`&self->pAacDecoderChannelInfo[aacChannels]`) |
//! | `pAacDecoderStaticChannelInfo[28]` | `Vec<Box<`[`channelinfo::CAacDecoderStaticChannelInfo`]`>>` | same indexing |
//! | `workBufferCore2` ([`aac_ram::get_work_buffer_core2`]) | `Vec<FixpDbl>` | `pSpectralCoefficient` of channel `ch` is `workBufferCore2[ch * 1024..]`; functions take `spec: &mut [FixpDbl]` starting at the first channel of the element, channel `i` of the element at `spec[i * 1024..]` |
//! | `pTimeData2` ([`aac_ram::get_work_buffer_core5`]) | `Vec<PcmDec>` | also the IGF `virtualSpec` of channel `ch2` of an element (`pTimeData2[1024 * ch2..]`) and the complex prediction scratch buffer `scratchBuffer2` (`pTimeData2[2048..3072]`); functions take `time_data2: &mut [FixpDbl]` (the start of `pTimeData2`, at least 3072 values); it must be the buffer the frame level uses for the time output (see below) |
//! | `pComStaticData->pWorkBufferCore1` ([`aac_ram::get_work_buffer_core1`]) | [`channelinfo::CWorkBufferCore1`] | one instance shared by all channels: `mdctOutTemp` (IMDCT output, stereo filling downmix), IGF TNF masks (bytes), complex prediction `scratchBuffer` |
//! | `pComStaticData` (per element, shared by both channels of a CPE) | [`channelinfo::CAacDecoderCommonStaticData`] | holds `pJointStereoData` and `cplxPredictionData` |
//! | `cpeStaticData[el]` (`pAacDecoderStaticChannelInfo[ch]->pCpeStaticData`) | `Option<Box<`[`channelinfo::CpePersistentData`]`>>` per element | only for elements with `AC_EL_USAC_CP_POSSIBLE` |
//! | `igf_private_data_common[2]` | `[IgfPrivateDataCommon; 2]` | channel `ch2` of an element uses `[ch2]` |
//! | `concealCommonData` | [`conceal_types::CConcealParams`] | `concealmentInfo.pConcealParams` of every channel; passed to the concealment functions |
//! | `pMCTdec[grp]` | `Option<Box<mct::CMct>>` | |
//!
//! ## Pointers of the per channel structures
//!
//! - `CAacDecoderChannelInfo::pSpectralCoefficient`: see `workBufferCore2` above.
//! - `CAacDecoderChannelInfo::pDynData`: owned (`p_dyn_data`); the C++ memory is allocated once
//!   per channel and never shared.
//! - `CAacDecoderChannelInfo::pComData`: an empty structure, omitted.
//! - `CAacDecoderStaticChannelInfo::pOverlapBuffer`: owned by `IMdct` (`MdctT::overlap`).
//! - `CAacDecoderStaticChannelInfo::concealmentInfo.spectralCoefficient`: owned by the
//!   [`conceal_types::CConcealmentInfo`]; it is also
//!   `CJointStereoPersistentData::spectralCoeffs[ch2]` (and `specScale[ch2]` is
//!   `concealmentInfo.specScale`), read by complex prediction and IGF stereo filling through the
//!   static channel info of the element's channels.
//! - `CJointStereoPersistentData::scratchBuffer` / `scratchBuffer2`: see the table above.
//! - `IGF_PRIVATE_DATA::IGF_Common_channel_data_handle`: see `mpeghdec_igf::igf_dec`.
//! - `CConcealmentInfo::pConcealParams`: always `&concealCommonData`, passed explicitly.
//! - `CAacDecoderStaticChannelInfo::hArCo`: `Option<Box<CArcoData>>`, allocated by the frame
//!   level for USAC/MPEG-H (`CArco_Create()`).
//!
//! Where a C++ function receives the same memory twice (e.g. `CBlock_ApplyNoise(...,
//! &pAacDecoderStaticChannelInfo->nfRandomSeed, pAacDecoderChannelInfo->pDynData->band_is_noise,
//! ...)`), the redundant parameter is omitted; the function documentation says so.
//!
//! ## Memory that is read with stale content
//!
//! These reads must be reproduced by passing the same memory as the C++ decoder:
//!
//! - The IGF `virtualSpec` (`pTimeData2[1024 * ch2..]`) is read before it is written when no tile
//!   exists (see `mpeghdec_igf::igf_dec`): it contains the time signal (and other data) the frame
//!   level left in `pTimeData2`.
//! - [`channelinfo::CWorkBufferCore1`] is one union used by all elements for different purposes
//!   (IMDCT output of `CBlock_FrequencyToTime()`, TNF masks, stereo filling downmix, complex
//!   prediction downmix). All uses in this crate write before they read, but the buffer must
//!   nevertheless be the one decoder wide instance.
//! - The concealment spectrum and scale of the channels are the "previous frame" of complex
//!   prediction (`use_prev_frame`) and IGF stereo filling; they are updated by
//!   `CConcealment_Apply()` (i.e. by the frame level after `CChannelElement_Decode()`).
//! - `pDynData` persists across frames per channel; some fields are only written by some element
//!   types (e.g. `fd_noise_level_and_offset` only with noise filling).
//! - Uninitialized C++ memory (`FDKmalloc()`: `pDynData`, `pJointStereoData`, MCT `prevOutSpec`)
//!   is zero-initialized here. No read of such memory before it is written was found for valid
//!   configurations.
//!
//! # Use by the frame level (`CAacDecoder_Init()` / `CAacDecoder_DecodeFrame()`)
//!
//! Configuration (per channel `ch` and element `el`, see `CAacDecoder_Init()`):
//!
//! 1. [`channelinfo::get_sampling_rate_info`] (`getSamplingRateInfo`).
//! 2. [`aac_ram::get_aac_decoder_channel_info`] / [`aac_ram::get_aac_decoder_static_channel_info`]
//!    (includes the concealment and overlap buffers), `h_ar_co = Some(CArcoData::create())`.
//! 3. Per element [`channelinfo::CAacDecoderCommonStaticData`] with `joint_stereo_data` (CPE)
//!    and `cplx_prediction_data` (`AC_EL_USAC_CP_POSSIBLE`, [`aac_ram::get_cplx_prediction_data`]),
//!    [`aac_ram::get_cpe_persistent_data`] with `win_groups_prev = 1`.
//! 4. `iis_igf_dec_lib_init()` for elements with enhanced noise filling, `nf_random_seed`
//!    (`0x3039` / `0x10932`), `granule_length`, `render_mode`, `mdct_init(&mut i_mdct,
//!    OVERLAP_BUFFER_SIZE)`, [`conceal::cconcealment_init_channel_data`]; [`mct::cmct_initialize`]
//!    per MCT signal group; [`conceal::cconcealment_init_common_data`] once.
//!
//! Decoding of a frame (see `CAacDecoder_DecodeFrame()`):
//!
//! 1. Per channel element: [`channel::cchannel_element_read`] (`CChannelElement_Read`) with the
//!    channel arrays starting at the element's first channel and a [`channel::CChannelElementMem`]
//!    (`spec = &mut workBufferCore2[aacChannels * 1024..]`, the element's
//!    `CAacDecoderCommonStaticData` and `CpePersistentData`, the decoder's `CWorkBufferCore1`,
//!    `igf_private_data_common` and `pTimeData2`).
//! 2. MCT: [`mct::cmct_mct_stereo_filling`] and [`mct::cmct_stereo_filling_save_prev`] /
//!    [`mct::cmct_stereo_filling_clear_prev`].
//! 3. Per channel element: [`channel::cchannel_element_decode`] (`CChannelElement_Decode`);
//!    with a thread pool [`channel::cchannel_element_decode_pool`] runs the per channel tools
//!    (IGF, TNS) of the two channels of a CPE in parallel.
//! 4. Per channel: [`conceal::cconcealment_apply`] (`CConcealment_Apply`),
//!    [`block::cblock_frequency_to_time`] (`CBlock_FrequencyToTime`, with `pWorkBuffer1 =
//!    mdctOutTemp` of the decoder's `CWorkBufferCore1`), [`ltp_post::ltp_post`] and
//!    [`conceal::cconcealment_td_fading`] (`CConcealment_TDFading`).
//!    With a thread pool ([`aacdecoder::AacDecoderInstance::set_thread_pool`]) the channels are
//!    processed in parallel ([`aacdecoder_par`]; each channel gets its own IMDCT output buffer
//!    and the time domain fading runs afterwards in channel order).
//!
//! The frame level also uses [`delay`] (`FDK_Delay_*`), [`deque`] and the concealment parameter
//! functions of [`conceal`].
//!
//! # Deviations from the C++ code
//!
//! Only for states that are not expected in valid streams (C++ undefined behaviour):
//!
//! - The complex prediction (`CJointStereo_ApplyMS()`) rescales the spectra by
//!   `specScale - aSfbScale`, which is negative if a channel has a larger `aSfbScale` than the
//!   frame maximum in a band at or above its `MaxSfBands`. `CBlock_InverseQuantizeSpectralData()`
//!   never produces this; the MCT rewrites the exponents of all bands but sets `MaxSfBands` of
//!   both channels of its pairs to the maximum, so only a CPE whose two channels belong to
//!   different MCT pairs can get there. The reference build runs these loops partly vectorized
//!   (NEON shifts by negative amounts shift left) and partly scalar (amount modulo 32); the Rust
//!   code always behaves like the scalar code.
//! - See the documentation of [`mct`], [`conceal`] and [`ltp_post`] for their cases.

pub mod aac_ram;
pub mod aac_rom;
pub mod aacdec_tns;
pub mod aacdecoder;
pub mod aacdecoder_lib;
pub mod aacdecoder_par;
pub mod block;
pub mod channel;
pub mod channelinfo;
pub mod conceal;
pub mod conceal_types;
pub mod delay;
pub mod deque;
pub mod fast;
pub mod fdp;
pub mod format_converter_glue;
pub mod ltp_post;
pub mod mct;
pub mod mpeghdecoder;
pub mod overlapadd;
pub mod pipeline;
pub mod render;
pub mod stereo;
pub mod ui;
pub mod usacdec_const;
