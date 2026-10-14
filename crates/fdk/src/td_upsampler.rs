// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/TD_upsampler.h, libFDK/src/TD_upsampler.cpp and
// libFDK/src/arm/TD_upsampler_arm.cpp (the ARM file only contains NEON code, which is inactive in
// the reference configuration). The filter coefficients were dumped from the C++ sources with
// scripts/codegen/dump_table.py.

//! Fixed-point implementation of a factor 1, 1.5, 2 and 3 upsampling LP-interpolator utilizing
//! biquad sections.

use crate::digest::StateDigest;

use crate::common_fix::*;
use crate::scale::scale_value_saturate;

pub const TD_UPSAMPLER_MAX_DELAY: i32 = 7;

/// size of the states memory
pub const TD_STATES_MEM_SIZE: usize = 7 * 2;

/// `RESAMPLING`
pub type Resampling = i32;
pub const DOWNSAMPLE: Resampling = 0;
pub const JUSTFILTER: Resampling = 1;

/// `TD_FAC_UPSAMPLE`: upsampling factor types
pub type TdFacUpsample = i32;
/// no upsampling
pub const TD_FAC_UPSAMPLE_1_1: TdFacUpsample = 0;
/// factor 3/2 upsampling
pub const TD_FAC_UPSAMPLE_3_2: TdFacUpsample = 1;
/// factor 2/1 upsampling
pub const TD_FAC_UPSAMPLE_2_1: TdFacUpsample = 2;
/// factor 3/1 upsampling
pub const TD_FAC_UPSAMPLE_3_1: TdFacUpsample = 3;

/// Number of biquad sections.
const NUM_SOS: usize = 7;

/// `struct filter`: cascade of biquad sections.
#[derive(StateDigest)]
struct Filter {
    coeff: [[FixpSgl; 4]; NUM_SOS],
    sos_gain: [FixpSgl; NUM_SOS],
    scale_fac_mantissa: FixpDbl,
    delay: [i8; 2],
    scale_fac_exp: i8,
}

const fn rows(c: [i16; 28]) -> [[FixpSgl; 4]; NUM_SOS] {
    let mut out = [[0; 4]; NUM_SOS];
    let mut i = 0;
    while i < NUM_SOS {
        out[i] = [c[4 * i], c[4 * i + 1], c[4 * i + 2], c[4 * i + 3]];
        i += 1;
    }
    out
}

/// `sos_2`: 2:1 interpolation filter
static SOS_2: Filter = Filter {
    coeff: rows([
        11196, 16384, 597, 7533, //
        32211, 16384, 6172, 700, //
        22133, 16384, 4040, 2951, //
        6339, 16384, -1953, 14208, //
        16060, 16384, 2339, 5000, //
        28215, 16384, 5407, 1478, //
        7945, 16384, -915, 10551,
    ]),
    sos_gain: [18271, 11728, 13952, 23996, 15920, 12503, 20943],
    scale_fac_mantissa: 0x7FDFFFFE,
    delay: [4, 0],
    scale_fac_exp: 1,
};

/// `sos_3`: 3:1 (and 3:2) interpolation filter
static SOS_3: Filter = Filter {
    coeff: rows([
        -6035, 16384, -11757, 8334, //
        31146, 16384, -2700, 237, //
        8894, 16384, -6241, 3198, //
        -10916, 16384, -16740, 14531, //
        -200, 16384, -8964, 5620, //
        20913, 16384, -3994, 1300, //
        -9393, 16384, -14377, 11267,
    ]),
    sos_gain: [15824, 7112, 10462, 21204, 13094, 8344, 18589],
    scale_fac_mantissa: 0x61998D08,
    delay: [3, 7],
    scale_fac_exp: 2,
};

/// `TD_applyFilter(sampleIn, sosData, states)`: filter one sample through all biquad sections.
///
/// The recursion uses wrapping arithmetic like the C++ code (which relies on the filter design
/// to avoid overflows).
fn td_apply_filter(sample_in: FixpDbl, sos_data: &Filter, states: &mut [FixpDbl]) -> FixpDbl {
    let mut input = sample_in;
    let mut output = 0;

    for s in 0..NUM_SOS {
        let state0 = states[s * 2];
        let mut state1 = states[s * 2 + 1];

        input = f_mult(input, sos_data.sos_gain[s]);
        output = input.wrapping_add(state0 << 1);
        state1 = state1
            .wrapping_add(f_mult(input, sos_data.coeff[s][0]))
            .wrapping_sub(f_mult(output, sos_data.coeff[s][2]));
        let state2 =
            f_mult(input, sos_data.coeff[s][1]).wrapping_sub(f_mult(output, sos_data.coeff[s][3]));

        states[s * 2] = state1;
        states[s * 2 + 1] = state2;
        input = output;
    }

    scale_value_saturate(f_mult(output, sos_data.scale_fac_mantissa), sos_data.scale_fac_exp as i32)
}

/// `TD_upsampler_init(facUpsample, states)`: zero-initialize the interpolation filter state
/// memory (at least [`TD_STATES_MEM_SIZE`] values). Returns the delay of the upsampler at the
/// output sample rate.
pub fn td_upsampler_init(fac_upsample: TdFacUpsample, states: &mut [FixpDbl]) -> i16 {
    states[..TD_STATES_MEM_SIZE].fill(0);

    let delay = match fac_upsample {
        TD_FAC_UPSAMPLE_1_1 => 0,
        TD_FAC_UPSAMPLE_3_2 => SOS_3.delay[0],
        TD_FAC_UPSAMPLE_2_1 => SOS_2.delay[0],
        TD_FAC_UPSAMPLE_3_1 => SOS_3.delay[1],
        _ => 0,
    };
    delay as i16
}

/// `TD_upsampler(facUpsample, sigIn, lenIn, sigOut, states)`: time-domain upsampling of the
/// `sig_in.len()` input samples (must be even for 3/2) into `sig_out`.
///
/// Returns the length of the upsampled signal. Unlike in C++, the input and output buffers cannot
/// be the same (the C++ decoder never calls it in place).
pub fn td_upsampler(
    fac_upsample: TdFacUpsample,
    sig_in: &[FixpDbl],
    sig_out: &mut [FixpDbl],
    states: &mut [FixpDbl],
) -> i16 {
    let len_in = sig_in.len() as i16;
    let mut len_out: i32 = 0;

    if len_in <= 0 {
        return 0;
    }
    let len_in = len_in as i32;
    let mut o = 0usize;

    match fac_upsample {
        TD_FAC_UPSAMPLE_1_1 => {
            sig_out[..len_in as usize].copy_from_slice(&sig_in[..len_in as usize]);
            len_out = len_in;
        }
        TD_FAC_UPSAMPLE_3_2 => {
            let mut temp = [0 as FixpDbl; 6];
            let mut ii = 0usize;
            for _ in 0..len_in / 2 {
                for k in 0..2 {
                    let input = sig_in[ii];
                    ii += 1;
                    temp[3 * k] = td_apply_filter(input, &SOS_3, states);
                    temp[3 * k + 1] = td_apply_filter(0, &SOS_3, states);
                    temp[3 * k + 2] = td_apply_filter(0, &SOS_3, states);
                }
                sig_out[o] = temp[0];
                sig_out[o + 1] = temp[2];
                sig_out[o + 2] = temp[4];
                o += 3;
            }
            len_out = (3 * len_in) / 2;
        }
        TD_FAC_UPSAMPLE_2_1 => {
            for &x in &sig_in[..len_in as usize] {
                let input = x >> 1;
                sig_out[o] = td_apply_filter(input, &SOS_2, states) << 1;
                sig_out[o + 1] = td_apply_filter(0, &SOS_2, states) << 1;
                o += 2;
            }
            len_out = 2 * len_in;
        }
        TD_FAC_UPSAMPLE_3_1 => {
            for &input in &sig_in[..len_in as usize] {
                sig_out[o] = td_apply_filter(input, &SOS_3, states);
                sig_out[o + 1] = td_apply_filter(0, &SOS_3, states);
                sig_out[o + 2] = td_apply_filter(0, &SOS_3, states);
                o += 3;
            }
            len_out = 3 * len_in;
        }
        _ => {}
    }

    len_out as i16
}
