// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/usacdec_const.h.

//! USAC related constants.

/* scale factors */
/// exponent of code[], fixed codebook vector
pub const SF_CODE: i32 = 6;
/// exponent of gain code and smoothed gain code
pub const SF_GAIN_C: i32 = 16;
/// exponent of exc[] and exc2[], excitation buffer
pub const SF_EXC: i32 = 16;
/// exponent of gain_pit
pub const SF_GAIN_P: i32 = 1;
/// exponent of period/voicing factor
pub const SF_PFAC: i32 = 0;
/// exponent of synthesis buffer
pub const SF_SYNTH: i32 = SF_EXC;
/// exponent of LP domain synthesis filter coefficient
pub const SF_A_COEFFS: i32 = 3;
/// exponent of stability factor
pub const SF_STAB: i32 = 1;

/* definitions which are independent of coreCoderFrameLength */
/// LP filter order
pub const M_LP_FILTER_ORDER: i32 = 16;
/// LP filter scale
pub const LP_FILTER_SCALE: i32 = 4;

/// Minimum pitch lag with resolution 1/4
pub const PIT_MIN_12k8: i32 = 34;
/// Maximum pitch lag for fs=12.8kHz
pub const PIT_MAX_12k8: i32 = 231;
/// Frequency scale denominator
pub const FSCALE_DENOM: i32 = 12800;
/// Minimum allowed frequency scale for acelp decoder
pub const FAC_FSCALE_MIN: i32 = 6000;

/// Default value from ref soft
pub const LPD_MAX_CORE_SR: i32 = 24000;
/// Maximum allowed frequency scale for acelp decoder
pub const FAC_FSCALE_MAX: i32 = LPD_MAX_CORE_SR;

/// Maximum pitch lag (= 411 for fs_max = 24000)
pub const PIT_MAX_TMP: i32 = PIT_MAX_12k8
    + (6 * ((((FAC_FSCALE_MAX * PIT_MIN_12k8) + (FSCALE_DENOM / 2)) / FSCALE_DENOM)
        - PIT_MIN_12k8));
/// `PIT_MAX_MAX` (cannot be smaller than 256 because of tcx time domain concealment)
pub const PIT_MAX_MAX: i32 = if PIT_MAX_TMP < 256 { 256 } else { PIT_MAX_TMP };

/// number of division (20ms) per 80ms frame
pub const NB_DIV: i32 = 4;
/// subframe size (5ms)
pub const L_SUBFR: i32 = 64;
/// bass postfilter delay (subframe)
pub const BPF_SFD: i32 = 1;
/// bass postfilter delay (samples)
pub const BPF_DELAY: i32 = BPF_SFD * L_SUBFR;

/// Delay of up-sampling filter (bass post-filter)
pub const L_FILT: i32 = 12;
/// for bass post-filter
pub const L_EXTRA: i32 = 96;
/// Length of filter for interpolation (acelp decoder)
pub const L_INTERPOL: i32 = 16 + 1;

/* definitions for coreCoderFrameLength = 1024 */
/// length of one 80ms superframe
pub const L_FRAME_PLUS_1024: i32 = 1024;
/// length of one acelp or tcx20 frame
pub const L_DIV_1024: i32 = L_FRAME_PLUS_1024 / NB_DIV;
/// number of 5ms subframe per division
pub const NB_SUBFR_1024: i32 = L_DIV_1024 / L_SUBFR;
/// number of 5ms subframe per 80ms frame
pub const NB_SUBFR_SUPERFR_1024: i32 = L_FRAME_PLUS_1024 / L_SUBFR;
/// AAC delay (subframe)
pub const AAC_SFD_1024: i32 = NB_SUBFR_SUPERFR_1024 / 2;
/// AAC delay (samples)
pub const AAC_DELAY_1024: i32 = AAC_SFD_1024 * L_SUBFR;
/// synthesis delay (subframe)
pub const SYN_SFD_1024: i32 = AAC_SFD_1024 - BPF_SFD;
/// synthesis delay (samples)
pub const SYN_DELAY_1024: i32 = SYN_SFD_1024 * L_SUBFR;
/// FAC frame length
pub const LFAC_1024: i32 = L_DIV_1024 / 2;
/// for transitions EIGHT_SHORT FD->LPD and vv.
pub const LFAC_SHORT_1024: i32 = L_DIV_1024 / 4;
/// FD noise shaping resolution (64=100Hz/point)
pub const FDNS_NPTS_1024: i32 = 64;

/* definitions for coreCoderFrameLength = 768 */
pub const L_FRAME_PLUS_768: i32 = 768;
/// length of one acelp or tcx20 frame
pub const L_DIV_768: i32 = L_FRAME_PLUS_768 / NB_DIV;
/// number of 5ms subframe per division
pub const NB_SUBFR_768: i32 = L_DIV_768 / L_SUBFR;
/// number of 5ms subframe per 80ms frame
pub const NB_SUBFR_SUPERFR_768: i32 = L_FRAME_PLUS_768 / L_SUBFR;
/// AAC delay (subframe)
pub const AAC_SFD_768: i32 = NB_SUBFR_SUPERFR_768 / 2;
/// AAC delay (samples)
pub const AAC_DELAY_768: i32 = AAC_SFD_768 * L_SUBFR;
/// synthesis delay (subframe)
pub const SYN_SFD_768: i32 = AAC_SFD_768 - BPF_SFD;
/// synthesis delay (samples)
pub const SYN_DELAY_768: i32 = SYN_SFD_768 * L_SUBFR;
/// FAC frame length
pub const LFAC_768: i32 = L_DIV_768 / 2;
/// for transitions EIGHT_SHORT FD->LPD and vv.
pub const LFAC_SHORT_768: i32 = L_DIV_768 / 4;

/* maximum (used for memory allocation) */
pub const L_FRAME_PLUS: i32 = L_FRAME_PLUS_1024;
pub const L_DIV: i32 = L_DIV_1024;
pub const NB_SUBFR: i32 = NB_SUBFR_1024;
pub const NB_SUBFR_SUPERFR: i32 = NB_SUBFR_SUPERFR_1024;
pub const AAC_SFD: i32 = AAC_SFD_1024;
pub const AAC_DELAY: i32 = AAC_DELAY_1024;
pub const SYN_SFD: i32 = SYN_SFD_1024;
pub const SYN_DELAY: i32 = SYN_DELAY_1024;
pub const LFAC: i32 = LFAC_1024;
pub const LFAC_SHORT: i32 = LFAC_SHORT_1024;
pub const FDNS_NPTS: i32 = FDNS_NPTS_1024;
