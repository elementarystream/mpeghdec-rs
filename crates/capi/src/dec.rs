// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from include/mpeghdecoder.h (the C API).

//! Decoder part of the C ABI: `mpeghdecoder_*()` with the signatures of `mpeghdecoder.h`.
//!
//! Each function checks its pointers like C and passes the arguments to the straight port
//! ([`mpeghdec_core::mpeghdecoder`]), which also reproduces what C writes to `outInfo` when
//! `mpeghdecoder_getSamples()` returns `MPEGH_DEC_FEED_DATA`. `HANDLE_MPEGH_DECODER_CONTEXT`
//! points to a [`MPEGH_DECODER_CONTEXT`] (a `Box` released by `mpeghdecoder_destroy()`).
//!
//! **Threads (not in C).** `mpeghdecoder_init()` reads the environment variable
//! `MPEGHDEC_THREADS` (a positive integer; unset, invalid or 0: 1, the default). With more than
//! one thread the decoder pipelines frames: the output frames come one `mpeghdecoder_process()`
//! call later, so the sequence of frames is the same as with one thread only if the caller calls
//! `mpeghdecoder_getSamples()` until it returns `MPEGH_DEC_FEED_DATA` after every `process()`,
//! `flush()` and `flushAndGet()` call (as the demo decoder does).
//!
//! **Panics.** Every function runs the port inside `catch_unwind`. No input is known to cause a
//! panic; after one, the handle is marked failed and its functions return
//! `MPEGH_DEC_PROCESS_ERROR`.

use core::ffi::c_int;
use mpeghdec_core::mpeghdecoder as d;
use std::panic::{catch_unwind, AssertUnwindSafe};

/// `MPEGH_DECODER_ERROR`
pub type MPEGH_DECODER_ERROR = c_int;
/// `MPEGH_DECODER_PARAMETER`
pub type MPEGH_DECODER_PARAMETER = c_int;

/// `MPEGH_DECODER_OUTPUT_INFO`
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MPEGH_DECODER_OUTPUT_INFO {
    pub numSamplesPerChannel: c_int,
    pub numChannels: c_int,
    pub sampleRate: c_int,
    pub ticks: u64,
    pub pts: u64,
    pub loudness: c_int,
    pub isConcealed: bool,
}

/// `struct MPEGH_DECODER_CONTEXT`: the decoder behind a handle.
pub struct MPEGH_DECODER_CONTEXT {
    ctx: d::MpeghDecoderContext,
    /// A call on this handle panicked.
    failed: bool,
}

/// `HANDLE_MPEGH_DECODER_CONTEXT`
pub type HANDLE_MPEGH_DECODER_CONTEXT = *mut MPEGH_DECODER_CONTEXT;

/// The number of decoder threads from `MPEGHDEC_THREADS` (1 if unset or invalid).
fn threads_from_env() -> usize {
    std::env::var("MPEGHDEC_THREADS").ok().and_then(|v| v.trim().parse().ok()).unwrap_or(1).max(1)
}

/// Runs `f` on the decoder of `h`: `MPEGH_DEC_NULLPTR_ERROR` for a NULL handle,
/// `MPEGH_DEC_PROCESS_ERROR` after a panic.
///
/// # Safety
/// `h` NULL or an open handle.
unsafe fn with_ctx(
    h: HANDLE_MPEGH_DECODER_CONTEXT,
    f: impl FnOnce(&mut d::MpeghDecoderContext) -> MPEGH_DECODER_ERROR,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    let Some(h) = (unsafe { h.as_mut() }) else { return d::MPEGH_DEC_NULLPTR_ERROR };
    if h.failed {
        return d::MPEGH_DEC_PROCESS_ERROR;
    }
    match catch_unwind(AssertUnwindSafe(|| f(&mut h.ctx))) {
        Ok(r) => r,
        Err(_) => {
            h.failed = true;
            d::MPEGH_DEC_PROCESS_ERROR
        }
    }
}

/// The bytes at `p` (`len` of them), or `None` for NULL.
///
/// # Safety
/// `p` NULL or valid for reads of `len` bytes.
unsafe fn bytes<'a>(p: *const u8, len: u32) -> Option<&'a [u8]> {
    if p.is_null() {
        None
    } else if len == 0 {
        Some(&[])
    } else {
        // SAFETY: the caller's guarantee.
        Some(unsafe { core::slice::from_raw_parts(p, len as usize) })
    }
}

/// `HANDLE_MPEGH_DECODER_CONTEXT mpeghdecoder_init(int32_t cicpSetup)`
#[no_mangle]
pub extern "C" fn mpeghdecoder_init(cicpSetup: i32) -> HANDLE_MPEGH_DECODER_CONTEXT {
    let threads = threads_from_env();
    let ctx = catch_unwind(|| d::MpeghDecoderContext::with_threads(cicpSetup, threads));
    match ctx {
        Ok(Some(ctx)) => Box::into_raw(Box::new(MPEGH_DECODER_CONTEXT { ctx, failed: false })),
        _ => core::ptr::null_mut(),
    }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_setMhaConfig(HANDLE_MPEGH_DECODER_CONTEXT hCtx, const uint8_t*
/// config, uint32_t configSize)`
///
/// # Safety
/// `hCtx` NULL or an open handle; `config` NULL or valid for reads of `configSize` bytes.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_setMhaConfig(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
    config: *const u8,
    configSize: u32,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    let Some(config) = (unsafe { bytes(config, configSize) }) else {
        return d::MPEGH_DEC_NULLPTR_ERROR;
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ctx(hCtx, |c| d::mpeghdecoder_set_mha_config(c, config)) }
}

/// `void mpeghdecoder_destroy(HANDLE_MPEGH_DECODER_CONTEXT hCtx)`
///
/// # Safety
/// `hCtx` NULL or an open handle, which is invalid afterwards.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_destroy(hCtx: HANDLE_MPEGH_DECODER_CONTEXT) {
    if !hCtx.is_null() {
        // SAFETY: an open handle is a `Box` of `mpeghdecoder_init()`.
        let h = unsafe { Box::from_raw(hCtx) };
        let _ = catch_unwind(AssertUnwindSafe(|| drop(h)));
    }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_process(HANDLE_MPEGH_DECODER_CONTEXT hCtx, const uint8_t*
/// inData, uint32_t inLength, uint64_t timestamp)`
///
/// # Safety
/// `hCtx` NULL or an open handle; `inData` NULL or valid for reads of `inLength` bytes.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_process(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
    inData: *const u8,
    inLength: u32,
    timestamp: u64,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    let Some(data) = (unsafe { bytes(inData, inLength) }) else {
        return d::MPEGH_DEC_NULLPTR_ERROR;
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ctx(hCtx, |c| d::mpeghdecoder_process(c, data, timestamp)) }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_processTimescale(HANDLE_MPEGH_DECODER_CONTEXT hCtx, const
/// uint8_t* inData, uint32_t inLength, uint64_t timestamp, uint32_t timescale)`
///
/// # Safety
/// See [`mpeghdecoder_process`].
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_processTimescale(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
    inData: *const u8,
    inLength: u32,
    timestamp: u64,
    timescale: u32,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    let Some(data) = (unsafe { bytes(inData, inLength) }) else {
        return d::MPEGH_DEC_NULLPTR_ERROR;
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ctx(hCtx, |c| d::mpeghdecoder_process_timescale(c, data, timestamp, timescale)) }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_getSamples(HANDLE_MPEGH_DECODER_CONTEXT hCtx, int32_t*
/// outData, uint32_t outLength, MPEGH_DECODER_OUTPUT_INFO* outInfo)`
///
/// # Safety
/// `hCtx` NULL or an open handle; `outData` NULL or valid for writes of `outLength` samples;
/// `outInfo` NULL or valid for reads and writes.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_getSamples(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
    outData: *mut i32,
    outLength: u32,
    outInfo: *mut MPEGH_DECODER_OUTPUT_INFO,
) -> MPEGH_DECODER_ERROR {
    if outData.is_null() || outInfo.is_null() {
        return d::MPEGH_DEC_NULLPTR_ERROR;
    }
    // SAFETY: the caller's guarantee.
    let (out, out_info) =
        unsafe { (core::slice::from_raw_parts_mut(outData, outLength as usize), &mut *outInfo) };
    // SAFETY: the caller's guarantee.
    unsafe {
        with_ctx(hCtx, |c| {
            let mut info = d::MpeghDecoderOutputInfo {
                num_samples_per_channel: out_info.numSamplesPerChannel,
                num_channels: out_info.numChannels,
                sample_rate: out_info.sampleRate,
                ticks: out_info.ticks,
                pts: out_info.pts,
                loudness: out_info.loudness,
                is_concealed: out_info.isConcealed,
            };
            let err = d::mpeghdecoder_get_samples(c, out, &mut info);
            *out_info = MPEGH_DECODER_OUTPUT_INFO {
                numSamplesPerChannel: info.num_samples_per_channel,
                numChannels: info.num_channels,
                sampleRate: info.sample_rate,
                ticks: info.ticks,
                pts: info.pts,
                loudness: info.loudness,
                isConcealed: info.is_concealed,
            };
            err
        })
    }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_flushAndGet(HANDLE_MPEGH_DECODER_CONTEXT hCtx)`
///
/// # Safety
/// `hCtx` NULL or an open handle.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_flushAndGet(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    unsafe { with_ctx(hCtx, d::mpeghdecoder_flush_and_get) }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_flush(HANDLE_MPEGH_DECODER_CONTEXT hCtx)`
///
/// # Safety
/// `hCtx` NULL or an open handle.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_flush(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    unsafe { with_ctx(hCtx, d::mpeghdecoder_flush) }
}

/// `MPEGH_DECODER_ERROR mpeghdecoder_setParam(HANDLE_MPEGH_DECODER_CONTEXT hCtx,
/// MPEGH_DECODER_PARAMETER param, int value)`
///
/// # Safety
/// `hCtx` NULL or an open handle.
#[no_mangle]
pub unsafe extern "C" fn mpeghdecoder_setParam(
    hCtx: HANDLE_MPEGH_DECODER_CONTEXT,
    param: MPEGH_DECODER_PARAMETER,
    value: c_int,
) -> MPEGH_DECODER_ERROR {
    // SAFETY: the caller's guarantee.
    unsafe { with_ctx(hCtx, |c| d::mpeghdecoder_set_param(c, param, value)) }
}
