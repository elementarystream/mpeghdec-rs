// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/FDK_delay.h and libMpeghDec/src/FDK_delay.cpp.

//! Delay element for multiple channels.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::PcmDec;

/// `FDK_SignalDelay`: one delay element for multiple channels.
#[derive(StateDigest, Clone, Debug, Default, PartialEq, Eq)]
pub struct FdkSignalDelay {
    /// `delay_line`: the allocated delay line (`delay * num_channels` values; empty for `NULL`).
    pub delay_line: Vec<PcmDec>,
    /// `delay`: delay required in samples (per channel).
    pub delay: u16,
    /// `num_channels`: number of channels to delay.
    pub num_channels: u8,
}

/// `FDK_Delay_Create(data, delay, num_channels)`: create a delay element for multiple channels
/// with fixed delay value.
///
/// Returns -1 on out of memory, else 0.
pub fn fdk_delay_create(data: &mut FdkSignalDelay, delay: u16, num_channels: u8) -> i32 {
    debug_assert!(num_channels > 0);

    if delay > 0 {
        let len = num_channels as usize * delay as usize;
        let mut v: Vec<PcmDec> = Vec::new();
        if v.try_reserve_exact(len).is_err() {
            return -1;
        }
        v.resize(len, 0);
        data.delay_line = v;
    } else {
        data.delay_line = Vec::new();
    }
    data.num_channels = num_channels;
    data.delay = delay;

    0
}

/// `FDK_Delay_Destroy(data)`: destroy a delay element.
pub fn fdk_delay_destroy(data: &mut FdkSignalDelay) {
    data.delay_line = Vec::new();
    data.delay = 0;
    data.num_channels = 0;
}
