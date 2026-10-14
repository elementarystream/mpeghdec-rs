// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libPCMutils/include/pcm_utils.h and libPCMutils/src/pcm_utils.cpp.
// libPCMutils/src/arm/pcm_utils_arm.cpp only contains NEON code, which is not compiled in the
// reference configuration.

//! Functions that perform (de)interleaving combined with format change.

use mpeghdec_fdk::common_fix::FixpDbl;

/// `FDK_interleave(pIn, pOut, channels, frameSize, length)`: interleave `length` samples of each
/// of the `channels` channels stored consecutively in `p_in` (`frame_size` samples apart).
pub fn fdk_interleave(
    p_in: &[FixpDbl],
    p_out: &mut [i32],
    channels: u32,
    frame_size: u32,
    length: u32,
) {
    let channels = channels as usize;
    let length = length as usize;
    let frame_size = frame_size as usize;
    if channels == 0 || length == 0 {
        return;
    }
    let p_out = &mut p_out[..channels * length];
    if channels == 2 {
        let (l, r) = (&p_in[..length], &p_in[frame_size..frame_size + length]);
        for ((o, &a), &b) in p_out.chunks_exact_mut(2).zip(l).zip(r) {
            o[0] = a;
            o[1] = b;
        }
        return;
    }
    // In tiles of samples: the strided stores of all channels stay in the L1 cache.
    const TILE: usize = 64;
    for s in (0..length).step_by(TILE) {
        let len = TILE.min(length - s);
        let out = &mut p_out[s * channels..(s + len) * channels];
        for ch in 0..channels {
            let input = &p_in[ch * frame_size + s..][..len];
            for (o, &v) in out[ch..].iter_mut().step_by(channels).zip(input) {
                *o = v;
            }
        }
    }
}
