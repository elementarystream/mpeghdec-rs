// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/aac_ram.h and libMpeghDec/src/aac_ram.cpp.

//! Memory of the core decoder.
//!
//! The C++ static memory pools (`C_ALLOC_MEM*`) become ordinary owned allocations. The `Get*()`
//! functions return zero-initialized memory like the C++ allocators (`FDKcalloc`); `Free*()` is
//! `drop`. The overlay work buffers are allocated with the sizes of the C++ pools because their
//! layout matters (see the [crate documentation](crate)).

use crate::aac_rom::PcmDec;
use crate::channelinfo::{
    CAacDecoderChannelInfo, CAacDecoderStaticChannelInfo, CWorkBufferCore1, CpePersistentData,
};
use crate::conceal_types::FixpCncl;
use crate::overlapadd::OVERLAP_BUFFER_SIZE;
use crate::stereo::CCplxPredictionData;
use mpeghdec_fdk::common_fix::FixpDbl;

/// Maximum number of channels (`(28)` in the C++ sources).
pub const AAC_DECODER_CHANNELS: usize = 28;

/// Size of `WorkBufferCore2` in `FIXP_DBL`: `(24) * (1024 * 3)`.
pub const WORK_BUFFER_CORE2_SIZE: usize = 24 * (1024 * 3);

/// Size of `WorkBufferCore5` (`pTimeData2`) in `PCM_DEC`: `(24) * ((1024 * 3) + 256)`.
pub const WORK_BUFFER_CORE5_SIZE: usize = 24 * ((1024 * 3) + 256);

/// `GetAacDecoderChannelInfo(ch)` together with the `pDynData` allocation of
/// `CAacDecoder_Init()`.
pub fn get_aac_decoder_channel_info(_ch: usize) -> Box<CAacDecoderChannelInfo> {
    Box::default()
}

/// `GetAacDecoderStaticChannelInfo(ch)` together with the buffers `CAacDecoder_Init()` assigns to
/// it: the concealment spectrum (`GetConcealmentSpecBuffer(ch)`, 1024 values) and the overlap
/// buffer (`GetOverlapBuffer(ch)`, [`OVERLAP_BUFFER_SIZE`] values, owned by `IMdct`). `hArCo`
/// (`CArco_Create()`) is allocated by the caller for USAC/MPEG-H.
pub fn get_aac_decoder_static_channel_info(_ch: usize) -> Box<CAacDecoderStaticChannelInfo> {
    let mut s: Box<CAacDecoderStaticChannelInfo> = Box::default();
    s.concealment_info.spectral_coefficient = get_concealment_spec_buffer(_ch);
    s.i_mdct.overlap = get_overlap_buffer(_ch);
    s
}

/// `GetConcealmentSpecBuffer(ch)`: 1024 values.
pub fn get_concealment_spec_buffer(_ch: usize) -> Vec<FixpCncl> {
    vec![0; 1024]
}

/// `GetOverlapBuffer(ch)`: [`OVERLAP_BUFFER_SIZE`] values.
pub fn get_overlap_buffer(_ch: usize) -> Vec<FixpDbl> {
    vec![0; OVERLAP_BUFFER_SIZE as usize]
}

/// `GetCpePersistentData()`
pub fn get_cpe_persistent_data() -> Box<CpePersistentData> {
    Box::default()
}

/// `GetCplxPredictionData()`
pub fn get_cplx_prediction_data() -> Box<CCplxPredictionData> {
    Box::default()
}

/// `GetWorkBufferCore1()`: the `CWorkBufferCore1` union shared by all channels.
pub fn get_work_buffer_core1() -> CWorkBufferCore1 {
    CWorkBufferCore1::default()
}

/// `GetWorkBufferCore2()`: `workBufferCore2`, [`WORK_BUFFER_CORE2_SIZE`] values. Holds the
/// spectral coefficients of all channels (`pSpectralCoefficient` of channel `ch` is
/// `workBufferCore2[ch * 1024..]`).
pub fn get_work_buffer_core2() -> Vec<FixpDbl> {
    vec![0; WORK_BUFFER_CORE2_SIZE]
}

/// `GetWorkBufferCore5()`: `pTimeData2`, [`WORK_BUFFER_CORE5_SIZE`] values.
pub fn get_work_buffer_core5() -> Vec<PcmDec> {
    vec![0; WORK_BUFFER_CORE5_SIZE]
}
