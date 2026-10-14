// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libgVBAPRenderer/include/gVBAPRenderer.h, libgVBAPRenderer/src/gVBAPRenderer.cpp
// and libgVBAPRenderer/src/arm/gVBAPRenderer_arm.cpp. The latter only contains assembler
// versions of the rendering loops (FUNCTION_gVBAPRenderer_RenderFrame_Time_func1/2) that are not
// compiled on AArch64/Apple, so the generic C loops are ported.

//! Object renderer API: object metadata (OAM) parsing, renderer setup and rendering of object
//! signals to the output channels with gain interpolation over the frame.

use crate::cartesian_math::*;
use crate::quick_hull::*;
use crate::vbap_core::*;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::Cicp2GeometryChannelGeometry;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{
    f2_pow, f_add_saturate, f_div_norm_high_prec, inv_fixp_e, inv_sqrt_norm2,
};
use mpeghdec_fdk::matrix_calloc::{fdk_calloc_matrix_2d, Matrix2D};
use mpeghdec_fdk::scale::{get_scalefactor, saturate_left_shift, scale_value};

/* FDK VBAP library info */
pub const FDK_VBAP_LIB_VL0: i32 = 1;
pub const FDK_VBAP_LIB_VL1: i32 = 0;
pub const FDK_VBAP_LIB_VL2: i32 = 0;
pub const FDK_VBAP_LIB_TITLE: &str = "FDK VBAP";

/* GVBAPRENDERER_MAX_FRAMELENGTH <= 1024 */
const VBAP_SCALE_GAINS: i32 = 3;
const VBAP_MAX_SCALE_OBJECTS: i32 = 5;
const VBAP_MAX_SCALE_SAMPLES: i32 = VBAP_SCALE_GAINS + VBAP_MAX_SCALE_OBJECTS;

/// `spread_depth_table[s] = (2^(s/3)/2) - 0.5` with an exponent of 4
static SPREAD_DEPTH_TABLE: [i32; 16] = [
    0x00000000, 0x010A28BE, 0x02597FA9, 0x04000000, 0x0614517D, 0x08B2FF53, 0x0C000000, 0x1028A2FA,
    0x1565FEA5, 0x1C000000, 0x245145F3, 0x2ECBFD4A, 0x3C000000, 0x4CA28BE6, 0x6197FA95, 0x7C000000,
];

/// `gain_dec_tab`: gain decoding table, `gain[o] = pow(10.0, (gain[o] - 32.0) / 40.0)` limited to
/// 0.004 ... 5.957.
static GAIN_DEC_TAB: [i32; 129] = [
    0x0010624E, 0x001145D0, 0x00124BD0, 0x0013615A, 0x00148754, 0x0015BEBB, 0x00170894, 0x001865F9,
    0x0019D812, 0x001B6018, 0x001CFF59, 0x001EB736, 0x00208922, 0x002276A2, 0x0024816A, 0x0026AB3B,
    0x0028F5C3, 0x002B631B, 0x002DF531, 0x0030AE5E, 0x003390D1, 0x00369EF8, 0x0039DB83, 0x003D4921,
    0x0040EAD7, 0x0044C394, 0x0048D69E, 0x004D277B, 0x0051B9DC, 0x00569187, 0x005BB2AE, 0x006121AE,
    0x0066E30E, 0x006CFBC1, 0x007370E6, 0x007A4805, 0x008186E9, 0x008933B0, 0x009154E7, 0x0099F16E,
    0x00A31091, 0x00ACBA1D, 0x00B6F621, 0x00C1CD80, 0x00CD4949, 0x00D9734E, 0x00E655CA, 0x00F3FBBB,
    0x010270B3, 0x0111C0F3, 0x0121F97A, 0x01332821, 0x01455B54, 0x0158A259, 0x016D0E20, 0x0182B049,
    0x0199999A, 0x01B1DF0B, 0x01CB93E8, 0x01E6CFAD, 0x0203A82B, 0x022235B5, 0x02429323, 0x0264DB4C,
    0x02892C64, 0x02AFA3C7, 0x02D8622C, 0x03038ACF, 0x03314298, 0x0361AF48, 0x0394FACE, 0x03CB50CC,
    0x0404DE8C, 0x0441D58D, 0x048268FB, 0x04C6D034, 0x050F4517, 0x055C04E4, 0x05AD5107, 0x06036E4A,
    0x065EA5A8, 0x06BF4525, 0x07259D48, 0x079206FE, 0x0804DCDF, 0x087E810F, 0x08FF59E8, 0x0987D54C,
    0x0A1866FF, 0x0AB1897A, 0x0B53BEC4, 0x0BFF9145, 0x0CB59147, 0x0D765775, 0x0E428D43, 0x0F1AE2DA,
    0x10000000, 0x10F2B66B, 0x11F3C70D, 0x13041CC5, 0x142491B0, 0x15561911, 0x1699BF59, 0x17F090F7,
    0x195BBBE8, 0x1ADC65C7, 0x1C73D5BB, 0x1E236C16, 0x1FEC99F2, 0x21D0D8CB, 0x23D1CC10, 0x25F127F6,
    0x2830B179, 0x2A925786, 0x2D1819D2, 0x2FC42203, 0x3298B2EA, 0x359830E4, 0x38C52A41, 0x3C224EDF,
    0x3FB2788E, 0x4378B370, 0x477824D5, 0x4BB445ED, 0x5030A0B2, 0x54F10A9A, 0x59F9830E, 0x5F4E54F8,
    0x5F4FDF3B,
];

/* intracoded / dynamic object metadata word sizes and signedness */
const WORDSIZE: [i32; 6] = [
    OAM_BITS_AZI,
    OAM_BITS_ELE,
    OAM_BITS_RAD,
    OAM_BITS_GAIN,
    OAM_BITS_SPREAD,
    OAM_BITS_OBJ_PRIORITY,
];
const SIGN: [i32; 6] = [1, 1, 0, 1, 0, 0]; /* 1 signed; 0 = unsigned */

const SPREAD_I: usize = SPREAD as usize;
const SPREAD_HEIGHT_I: usize = SPREAD_HEIGHT as usize;
const SPREAD_DEPTH_I: usize = SPREAD_DEPTH as usize;
const NMAX: usize = OAM_NUMBER_MAX_COMPONENTS as usize;

/// Sign extension of a `bits` wide value (`(x << (32 - bits)) >> (32 - bits)`).
#[inline]
fn sign_extend(x: i32, bits: i32) -> i32 {
    x.wrapping_shl((32 - bits) as u32).wrapping_shr((32 - bits) as u32)
}

/// Loop over the object offsets `n0 = 8, 16, ... < 8 * num_objects` with the `UCHAR` counter of
/// the C++ code. For 32 objects, the C++ loop never terminates (the counter wraps around to 0);
/// here the loop ends at the wrap-around.
fn for_each_object_offset(num_objects: u32, mut f: impl FnMut(usize)) {
    let mut n0: u8 = NMAX as u8;
    while (n0 as u32) < NMAX as u32 * num_objects {
        f(n0 as usize);
        n0 = n0.wrapping_add(NMAX as u8);
        if n0 == 0 {
            break;
        }
    }
}

/// `get_iframe()` (static): parses a COAM I-frame (`intracoded_object_metadata_low_delay()`).
fn get_iframe(
    bs: &mut FdkBitStream,
    pos: &mut [i32],
    fixed_val: &mut [u8; NMAX],
    num_objects: u32,
    has_dynamic_object_priority: u8,
    has_uniform_spread: u8,
) {
    let num_comp = OAM_NUMBER_COMPONENTS as usize + has_dynamic_object_priority as usize;

    /* Table 76 */
    /* intracoded_object_metadata_low_delay() */
    if num_objects > 1 {
        /* Number of objects in one single OAM frame */
        for n in 0..num_comp {
            /* fixed {azimuth; elevation; radius; gain; spread; dynamic_object_priority} */
            fixed_val[n] = bs.read_bit() as u8;
            if has_uniform_spread == 0 && n == SPREAD_I {
                fixed_val[SPREAD_HEIGHT_I] = fixed_val[n];
                fixed_val[SPREAD_DEPTH_I] = fixed_val[n];
            }
            let common_val = if fixed_val[n] != 0 {
                true
            } else {
                /* common {azimuth; elevation; radius; gain; spread; dynamic_object_priority} */
                bs.read_bit() != 0
            };
            if common_val {
                /* default {azimuth; elevation; radius; gain; spread; dynamic_object_priority} */
                pos[n] = bs.read_bits(WORDSIZE[n] as u32) as i32;
                /* shift signed bit to right position */
                if SIGN[n] != 0 {
                    pos[n] = sign_extend(pos[n], WORDSIZE[n]);
                }
                /* copy fixed/common value for all objects */
                let v = pos[n];
                for_each_object_offset(num_objects, |n0| pos[n + n0] = v);
                if has_uniform_spread == 0 && n == SPREAD_I {
                    /* position spread_height */
                    pos[SPREAD_HEIGHT_I] = bs.read_bits(OAM_BITS_SPREAD_HEIGHT as u32) as u8 as i32;
                    /* position spread_depth */
                    pos[SPREAD_DEPTH_I] = bs.read_bits(OAM_BITS_SPREAD_DEPTH as u32) as u8 as i32;
                    let (sh, sd) = (pos[SPREAD_HEIGHT_I], pos[SPREAD_DEPTH_I]);
                    for_each_object_offset(num_objects, |n0| {
                        pos[SPREAD_HEIGHT_I + n0] = sh; /* copy fixed value for all objects */
                        pos[SPREAD_DEPTH_I + n0] = sd; /* copy fixed value for all objects */
                    });
                }
            } else {
                let mut n0: u8 = 0;
                while (n0 as u32) < NMAX as u32 * num_objects {
                    let o = n0 as usize;
                    /* position {azimuth; elevation; radius; gain; spread;
                     * dynamic_object_priority} */
                    pos[o + n] = bs.read_bits(WORDSIZE[n] as u32) as i32;
                    /* shift signed bit to right position */
                    if SIGN[n] != 0 {
                        pos[o + n] = sign_extend(pos[o + n], WORDSIZE[n]);
                    }
                    if has_uniform_spread == 0 && n == SPREAD_I {
                        /* position spread_height */
                        pos[SPREAD_HEIGHT_I + o] =
                            bs.read_bits(OAM_BITS_SPREAD_HEIGHT as u32) as u8 as i32;
                        /* position spread_depth */
                        pos[SPREAD_DEPTH_I + o] =
                            bs.read_bits(OAM_BITS_SPREAD_DEPTH as u32) as u8 as i32;
                    }
                    n0 = n0.wrapping_add(NMAX as u8);
                    if n0 == 0 {
                        break;
                    }
                }
            }
        } /* for loop over n */
    } else {
        for n in 0..num_comp {
            /* position {azimuth; elevation; radius; gain; spread; dynamic_object_priority} */
            pos[n] = bs.read_bits(WORDSIZE[n] as u32) as u8 as i32;
            /* shift signed bit to right position */
            if SIGN[n] != 0 {
                pos[n] = sign_extend(pos[n], WORDSIZE[n]);
            }
            if has_uniform_spread == 0 && n == SPREAD_I {
                /* position spread_height */
                pos[SPREAD_HEIGHT_I] = bs.read_bits(OAM_BITS_SPREAD_HEIGHT as u32) as u8 as i32;
                /* position spread_depth */
                pos[SPREAD_DEPTH_I] = bs.read_bits(OAM_BITS_SPREAD_DEPTH as u32) as u8 as i32;
            }
        }
    }
}

/// `get_dframe()` (static): parses a COAM D-frame (`dynamic_object_metadata()`).
fn get_dframe(
    bs: &mut FdkBitStream,
    pos: &mut [i32],
    fixed_val: &[u8; NMAX],
    num_objects: u32,
    has_dynamic_object_priority: u8,
    has_uniform_spread: u8,
) {
    let num_comp = OAM_NUMBER_COMPONENTS as usize + has_dynamic_object_priority as usize;
    let mut n0: u8 = 0;

    /* Table 77 */
    /* dynamic_object_metadata() */
    let flag_absolute = bs.read_bit() != 0;
    for _obj in 0..num_objects {
        let has_object_data = bs.read_bit() != 0;
        let o = n0 as usize;
        if has_object_data {
            /* Table 78 */
            /* single_dynamic_object_metadata() */
            if flag_absolute {
                for n in 0..num_comp {
                    pos[o + n] = 0;
                    if fixed_val[n] == 0 {
                        pos[o + n] = bs.read_bits(WORDSIZE[n] as u32) as i32;
                        if SIGN[n] != 0 {
                            /* recover sign bit */
                            pos[o + n] = sign_extend(pos[o + n], WORDSIZE[n]);
                        }
                        if has_uniform_spread == 0 && n == SPREAD_I {
                            /* position spread_height */
                            pos[SPREAD_HEIGHT_I + o] =
                                bs.read_bits(OAM_BITS_SPREAD_HEIGHT as u32) as u8 as i32;
                            /* position spread_depth */
                            pos[SPREAD_DEPTH_I + o] =
                                bs.read_bits(OAM_BITS_SPREAD_DEPTH as u32) as u8 as i32;
                        }
                    } else {
                        pos[o + n] = 0;
                    }
                }
            } else {
                /* DPCM */
                let num_bits = (bs.read_bits(3) as u8).wrapping_add(2);
                for n in 0..num_comp {
                    if fixed_val[n] == 0 {
                        let flag_val = bs.read_bit() != 0;
                        if flag_val {
                            let bits: u8 = if WORDSIZE[n] == OAM_BITS_AZI {
                                num_bits
                            } else {
                                f_min(num_bits as i32, WORDSIZE[n] + 1) as u8
                            };
                            let relative_part =
                                sign_extend(bs.read_bits(bits as u32) as i32, bits as i32); /* every value is signed */

                            pos[o + n] = pos[o + n].wrapping_add(relative_part);
                        }
                        if has_uniform_spread == 0 && n == SPREAD_I {
                            if bs.read_bit() != 0 {
                                let bits = f_min(num_bits as i32, OAM_BITS_SPREAD_HEIGHT + 1);
                                let relative_part =
                                    sign_extend(bs.read_bits(bits as u32) as i32, bits);
                                pos[SPREAD_HEIGHT_I + o] =
                                    pos[SPREAD_HEIGHT_I + o].wrapping_add(relative_part);
                            }

                            if bs.read_bit() != 0 {
                                let bits = f_min(num_bits as i32, OAM_BITS_SPREAD_DEPTH + 1);
                                let relative_part =
                                    sign_extend(bs.read_bits(bits as u32) as i32, bits);
                                pos[SPREAD_DEPTH_I + o] =
                                    pos[SPREAD_DEPTH_I + o].wrapping_add(relative_part);
                            }
                        }
                    }
                } /* for (n = 0; n < (OAM_NUMBER_COMPONENTS + hasDynamicObjectPriority); n++) */
            } /* if (flag_absolute) */
        } /* if (has_object_data) */
        n0 = n0.wrapping_add(NMAX as u8);
    } /* for (obj = 0; obj < num_objects; obj++) */
}

/// `range(value, minval, maxval)` (static)
#[inline]
fn range(value: FixpDbl, minval: FixpDbl, maxval: FixpDbl) -> FixpDbl {
    f_min(maxval, f_max(value, minval))
}

/// Copies the OAM sample fields (all but `cart`), like the concealment code does.
#[inline]
fn copy_oam_sample(dst: &mut OamSample, src: &OamSample) {
    dst.gain = src.gain;
    dst.sph = src.sph;
    dst.goa_bs_object_distance = src.goa_bs_object_distance;
    dst.spread_angle = src.spread_angle;
    dst.spread_depth = src.spread_depth;
    dst.spread_height = src.spread_height;
}

/// `oam_concealment()` (static): fills the current samples with the last valid samples. If there
/// were no valid samples yet then everything will be rendered to center with gain 1.0.
fn oam_concealment(h: &mut GVbapRenderer) {
    for k in 0..h.num_oam_frames.max(0) as usize {
        h.metadata_present[k] = h.metadata_present_valid[k];

        if h.metadata_present[k] != 0 {
            for i in 0..h.num_objects.max(0) as usize {
                let src = h.oam_samples_valid[k][i];
                copy_oam_sample(&mut h.oam_samples[k][i], &src);
            }
        }
    }
}

/// `object_metadata()` (static): parses one `object_metadata()` element and converts it into
/// OAM samples. Returns 1 for unsupported (non low delay) metadata.
fn object_metadata(
    bs: &mut FdkBitStream,
    low_delay_metadata_coding: u8,
    num_objects: u32,
    has_dynamic_object_priority: u8,
    has_uniform_spread: u8,
    pos: &mut [i32],
    fixed_val: &mut [u8; NMAX],
    oam_sample: &mut [OamSample],
    render_mode: i32,
) -> i32 {
    /* Table 70 */
    if low_delay_metadata_coding == 0 {
        /* object_metadata_efficient */
        if bs.read_bit() != 0 {
            /* has_differential_metadata */
            return 1;
        }
    } else {
        /* object_metadata_low_delay */
        /* Table 75 */
        let has_intracoded_object_metadata = bs.read_bit();
        if has_intracoded_object_metadata != 0 {
            /* intracoded_object_metadata_low_delay() */
            get_iframe(
                bs,
                pos,
                fixed_val,
                num_objects,
                has_dynamic_object_priority,
                has_uniform_spread,
            );
        } else {
            /* dynamic_object_metadata */
            get_dframe(
                bs,
                pos,
                fixed_val,
                num_objects,
                has_dynamic_object_priority,
                has_uniform_spread,
            );
        }
    }

    /**************************************/
    /* Post Processing of Object Metadata */
    /**************************************/
    for i in 0..num_objects as usize {
        let pi = i * NMAX;
        let s = &mut oam_sample[i];

        /* Azimuth limit Range -180 ... 180 Degree */
        let temp_azi = range(pos[pi + AZIMUTH as usize], -120, 120); /* 180 = 1.5 * 120 */
        /* revert encoders scaling */
        s.sph.azi = temp_azi * 17895697; /* (2^31 - 1) / 120 */

        /* Elevation limit Range -90 ... 90 Degree */
        let temp_ele = range(pos[pi + ELEVATION as usize], -30, 30); /* 90 = 3.0 * 30 */
        /* revert encoders scaling */
        s.sph.ele = temp_ele * 35791394; /* (2^31 - 1) / 60 */

        if render_mode == GVBAP_LEGACY {
            s.sph.rad = fl2fxconst_dbl(1.0 / 16.0); /* set radius to 1.0 */
        }

        /* Gain revert encoders scaling */
        let tab_index = f_min(64, f_max(-64, pos[pi + GAIN as usize])) + 64;
        s.gain = GAIN_DEC_TAB[tab_index as usize];

        /* Spread limit Range 0 ... 180 Degree */
        let temp_sp = range(pos[pi + SPREAD_I], 0, 120); /* 180 = 1.5 * 120 */
        /* revert encoders scaling */
        s.spread_angle = temp_sp * 17895697; /* (2^31 - 1) / 120 */

        if has_uniform_spread == 0 {
            /* SpreadHeigh limit Range 0 ... 90 Degree */
            let temp_sh = range(pos[pi + SPREAD_HEIGHT_I], 0, 30); /* 90 = 3.0 * 30 */
            /* SpreadDepth limit Range 0 ... 15.5 */
            let temp_sd = range(pos[pi + SPREAD_DEPTH_I], 0, 15); /* 0 ... 15.5 */
            let mut spread_depth = SPREAD_DEPTH_TABLE[temp_sd as usize];
            spread_depth = f_mult(spread_depth, fl2fxconst_dbl(1.0 / 15.5));
            /* revert encoders scaling */
            s.spread_height = temp_sh * 35791394; /* (2^31 - 1) / (2*30) */
            s.spread_depth = spread_depth; /* exponent = 4 */
        } else {
            /* prevent uninitialized values */
            s.spread_height = 0;
            s.spread_depth = 0;
        }
    }

    0
}

/// `objectMetadataFrame(hgVBAPRenderer, bs, usacExtElementPayloadLength, lowDelayMetadataCoding,
/// num_objects, hasDynamicObjectPriority, hasUniformSpread)`: parses the object metadata of one
/// frame (`ID_EXT_ELE_OBJ_METADATA` payload) into the OAM samples of the renderer and moves the
/// bitstream to the end of the payload. Returns 0 on success.
///
/// `num_objects` must not exceed the number of objects of the renderer.
pub fn object_metadata_frame(
    h: &mut GVbapRenderer,
    bs: &mut FdkBitStream,
    usac_ext_element_payload_length: i32,
    low_delay_metadata_coding: u8,
    num_objects: u32,
    has_dynamic_object_priority: u8,
    has_uniform_spread: u8,
) -> i32 {
    let mut err;

    h.oam_data_valid = 0;

    let num_oam_frames = h.num_oam_frames as i32 as u32;

    h.has_uniform_spread = has_uniform_spread; /* store uniformSpread information */

    let render_mode = h.render_mode as i32;

    let mut cnt_read_bits = bs.get_valid_bits();

    if h.oam_frame_length < h.frame_length {
        let mut present_frames = 0;

        for i in 0..num_oam_frames as usize {
            h.metadata_present[i] = bs.read_bit() as u8;

            if h.metadata_present[i] != 0 {
                err = object_metadata(
                    bs,
                    low_delay_metadata_coding,
                    num_objects,
                    has_dynamic_object_priority,
                    has_uniform_spread,
                    &mut h.oam_parsed_data,
                    &mut h.fixed_val,
                    &mut h.oam_samples[i],
                    render_mode,
                );
                if err != 0 {
                    return err;
                }
                present_frames += 1;
            }
        }
        if present_frames == 0 {
            return 1;
        }
    } else {
        h.metadata_present[0] = 1;

        err = object_metadata(
            bs,
            low_delay_metadata_coding,
            num_objects,
            has_dynamic_object_priority,
            has_uniform_spread,
            &mut h.oam_parsed_data,
            &mut h.fixed_val,
            &mut h.oam_samples[0],
            render_mode,
        );
        if err != 0 {
            return err;
        }
    }

    /* point bitstream buffer to right position */
    let mut left_bits = bs.get_valid_bits();
    if left_bits < 0 {
        return -1;
    }
    cnt_read_bits = cnt_read_bits.wrapping_sub(left_bits);
    left_bits = (usac_ext_element_payload_length.wrapping_shl(3)).wrapping_sub(cnt_read_bits);
    if left_bits < 0 {
        return -1;
    }
    bs.push_bi_directional(left_bits); /* set bitstream to right position */

    /* Valid object meta data available => save it for concealment */
    h.oam_data_valid = 1;

    /* Copy new valid object meta data to oamSamplesValid for concealment */
    for k in 0..h.num_oam_frames.max(0) as usize {
        h.metadata_present_valid[k] = h.metadata_present[k];

        if h.metadata_present_valid[k] != 0 {
            for l in 0..h.num_objects.max(0) as usize {
                let src = h.oam_samples[k][l];
                copy_oam_sample(&mut h.oam_samples_valid[k][l], &src);
            }
        }
    }

    0
}

/// `decodeDistance()` (static)
fn decode_distance(position_distance: i32, bs_reference_distance: i32) -> FixpDbl {
    let mut distance: FixpDbl = 0;

    if position_distance > 0 {
        let mut e = 0;
        distance = f_mult(
            fl2fxconst_dbl(0.0472188798661443 * 16.0),
            (position_distance.wrapping_sub(bs_reference_distance).wrapping_sub(120))
                .wrapping_shl(31 - 9),
        ); /* exp 5 */
        distance = f2_pow(distance, 5, &mut e);

        if e > 4 && distance > (MAXVAL_DBL.wrapping_shr((e - 4) as u32)) {
            distance = MAXVAL_DBL;
        } else {
            distance = scale_value(distance, e - 4);
        }
    }

    distance
}

/// `prodMetadataFrameGroup(hgVBAPRenderer, bs, numObjects, bsReferenceDistance,
/// hasObjectDistance)`: parses the production metadata (object distances) of one object group.
/// With `h == None` the data is only skipped. Returns 0 on success.
pub fn prod_metadata_frame_group(
    mut h: Option<&mut GVbapRenderer>,
    bs: &mut FdkBitStream,
    num_objects: u32,
    bs_reference_distance: i32,
    has_object_distance: i32,
) -> i32 {
    /* sets the distance of object o for all OAM frames */
    fn set(h: &mut GVbapRenderer, o: usize, distance: FixpDbl, d: i32) {
        for i in 0..h.num_oam_frames.max(0) as usize {
            h.oam_samples[i][o].sph.rad = distance;
            h.oam_samples[i][o].goa_bs_object_distance = d as u16;
        }
    }

    if has_object_distance != 0 {
        let has_intra_coded_data = bs.read_bit();

        if has_intra_coded_data != 0 {
            if num_objects > 1 {
                let fixed_distance = bs.read_bit();

                if fixed_distance != 0 {
                    let default_distance = bs.read_bits(9) as i32;
                    if let Some(h) = h.as_deref_mut() {
                        let distance = decode_distance(default_distance, bs_reference_distance);
                        for o in 0..num_objects as usize {
                            set(h, o, distance, default_distance);
                        }
                    }
                } else {
                    let common_distance = bs.read_bit();

                    if common_distance != 0 {
                        let default_distance = bs.read_bits(9) as i32;
                        if let Some(h) = h.as_deref_mut() {
                            let distance = decode_distance(default_distance, bs_reference_distance);
                            for o in 0..num_objects as usize {
                                set(h, o, distance, default_distance);
                            }
                        }
                    } else {
                        for o in 0..num_objects as usize {
                            let position_distance = bs.read_bits(9) as i32;
                            if let Some(h) = h.as_deref_mut() {
                                let distance =
                                    decode_distance(position_distance, bs_reference_distance);
                                set(h, o, distance, position_distance);
                            }
                        }
                    }
                }
            } else {
                let position_distance = bs.read_bits(9) as i32;
                if let Some(h) = h.as_deref_mut() {
                    let distance = decode_distance(position_distance, bs_reference_distance);
                    set(h, 0, distance, position_distance);
                }
            }
        } else {
            return 1;
        }
    } else if let Some(h) = h {
        for i in 0..h.num_oam_frames.max(0) as usize {
            for o in 0..num_objects as usize {
                h.oam_samples[i][o].sph.rad = fl2fxconst_dbl(1.0 / 16.0);
            }
        }
    }

    0
}

/* ------------------ SPREAD -------------------- */

const GVBAP_SPREAD_OPEN_ANGLE_VSO: FixpDbl = fl2fxconst_dbl(30.0 / 180.0); /* 30 degrees */
const GVBAP_SPREAD_PARABLE_EXP: i32 = 6;

/// `FL2FXCONST_DBL(GVBAP_SPREAD_80DB * GVBAP_SPREAD_80DB * (1 << 26))` (float expression)
const SPREAD_80DB_SQR: FixpDbl = fl2fxconst_dbl((0.0001f32 * 0.0001f32 * (1 << 26) as f32) as f64);
/// `-FL2FXCONST_DBL(GVBAP_SPREAD_PARABLE_SCALING_FACTOR / 2)` with
/// `GVBAP_SPREAD_PARABLE_SCALING_FACTOR = -1.1682f` (-4*(1-db2mag(-3)), comes from
/// -3dB = -scalFact*(spd/2)^2+1)
const SPREAD_PARABLE_SCALING: FixpDbl = -fl2fxconst_dbl((-1.1682f32 / 2.0f32) as f64);

static RING_INDICES30_1: [usize; 7] = [10, 11, 0, 1, 2, 9, 3];
static RING_INDICES30_2: [usize; 7] = [8, 7, 6, 5, 4, 9, 3];

/// `spread_indexRing`: ring containing VSO indices
static SPREAD_INDEX_RING: [u8; 3 * GVBAP_SPREAD_NUM_VSO_AZI] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
];

/// `gVBAPRenderer_Spread_internal_spread_mappingToH()`: exp 1
fn spread_mapping_to_h(spread_angle: FixpDbl) -> FixpDbl {
    let mut m = f_mult_div2(fl2fxconst_dbl(1.25 / 2.0), spread_angle);
    m = m.wrapping_add(f_mult_div2(
        fl2fxconst_dbl(0.0069 / 2.0 * 180.0),
        f_mult(spread_angle, spread_angle),
    ));
    m.wrapping_add(GVBAP_SPREAD_OPEN_ANGLE_VSO >> 1) /* exp 1 */
}

/// `gVBAPRenderer_Spread_internal_spread_Parable()`: output scaled by
/// 180^2 / 2^GVBAP_SPREAD_PARABLE_EXP compared to float implementation.
fn spread_parable(spread_angle: FixpDbl /* exp 1 */) -> FixpDbl {
    let mut tmp = f_mult(spread_angle, spread_angle); /* exp 2 */
    let mut e = 0;

    tmp = f_div_norm_high_prec(SPREAD_PARABLE_SCALING, tmp, &mut e).wrapping_neg(); /* exp e-1 */
    scale_value(tmp, e - 1 - GVBAP_SPREAD_PARABLE_EXP)
}

/// `gVBAPRenderer_Spread_internal_indexRingLimiter()`
fn index_ring_limiter(spread_angle: FixpDbl /* exp 1 */) -> i32 {
    f_min(
        (GVBAP_SPREAD_NUM_VSO_AZI >> 1) as i32,
        -((spread_angle.wrapping_neg().wrapping_add(spread_angle >> 2)) >> 27),
    )
}

/// `fMax(FL2FXCONST_DBL(0), t + ((FIXP_DBL)MAXVAL_DBL >> GVBAP_SPREAD_PARABLE_EXP)) <<
/// GVBAP_SPREAD_PARABLE_EXP` as compiled by clang: the addition may overflow for extreme
/// parameters (undefined behaviour), and clang evaluates the comparison as
/// `t > -(MAXVAL_DBL >> 6)` and the result as `(t << 6) + (MAXVAL_DBL >> 6 << 6)` (wrapping).
#[inline]
fn parable_clamp(t: FixpDbl) -> FixpDbl {
    const C: FixpDbl = MAXVAL_DBL >> GVBAP_SPREAD_PARABLE_EXP;
    if t > -C {
        t.wrapping_add(C).wrapping_shl(GVBAP_SPREAD_PARABLE_EXP as u32)
    } else {
        0
    }
}

/// Parable gain of an angle difference (common part of the layer gain calculation).
#[inline]
fn parable_gain(parable: FixpDbl, diff: FixpDbl) -> FixpDbl {
    let mut tmp = f_mult(diff, diff);
    if tmp < 0 {
        tmp = MAXVAL_DBL;
    }
    parable_clamp(f_mult(parable, tmp))
}

/// `gVBAPRenderer_Spread_internal_calculateLayerGains()`
fn spread_calculate_layer_gains(
    angle_object: FixpDbl,
    loop_limit: i32,
    parable: FixpDbl,
    gains: &mut [FixpDbl],
) {
    /* sign of object angle */
    let plumin: i32 = if angle_object < 0 { -1 } else { 1 };

    /* intermediate spread parameters for indexRing */
    let tmp = f_abs(angle_object);
    let mut multiple = (tmp.wrapping_add(tmp.wrapping_neg() >> 2)) >> 28;

    let diff_clk_dir = f_min(
        GVBAP_SPREAD_OPEN_ANGLE_VSO,
        tmp.wrapping_sub(GVBAP_SPREAD_OPEN_ANGLE_VSO.wrapping_mul(multiple)),
    );
    let diff_anti_clk_dir = GVBAP_SPREAD_OPEN_ANGLE_VSO.wrapping_sub(diff_clk_dir);

    if angle_object < 0 {
        multiple = -multiple;
    }

    /* calculate gains for the given layer */
    for n in 0..loop_limit {
        let idx1_arg = multiple + GVBAP_SPREAD_NUM_VSO_AZI as i32 - plumin * n;
        let idx2_arg = multiple + GVBAP_SPREAD_NUM_VSO_AZI as i32 + plumin * (n + 1);
        let idx1 = SPREAD_INDEX_RING[idx1_arg as usize] as usize;
        let idx2 = SPREAD_INDEX_RING[idx2_arg as usize] as usize;

        let off = GVBAP_SPREAD_OPEN_ANGLE_VSO.wrapping_mul(n);
        gains[idx1] = parable_gain(parable, diff_clk_dir.wrapping_add(off));
        gains[idx2] = parable_gain(parable, diff_anti_clk_dir.wrapping_add(off));
    }
}

/// `gVBAPRenderer_Spread_internal_calculateVSOGains()`
fn spread_calculate_vso_gains(
    h: &mut GVbapRenderer,
    obj_angle_ele: FixpDbl,
    tmp_gain_azi1: &[FixpDbl],
    tmp_gain_azi2: &[FixpDbl],
    tmp_gain_ele1: &[FixpDbl],
    tmp_gain_ele2: &[FixpDbl],
    parable_azi: FixpDbl,
    parable_ele: FixpDbl,
) {
    let mut obj_no = 0;

    /* calculate VSO spread gains */
    for ne in 0..GVBAP_SPREAD_NUM_VSO_ELE - 2 {
        for na in 0..GVBAP_SPREAD_NUM_VSO_AZI {
            h.spread_gains_spread[obj_no] = (f_mult(tmp_gain_azi1[na], tmp_gain_ele1[ne]) >> 1)
                + (f_mult(tmp_gain_azi2[na], tmp_gain_ele2[ne]) >> 1); /* exp 1 */
            obj_no += 1;
        }
    }

    /* spread gains for VSO with elevation +/-90 degrees */
    /* Separate calculation as azi angle for VSO with elevation +/-90 degrees is the object azimuth
     * angle itself, but defined once -> diffazi = 0 or 180 */
    let tmp = parable_clamp(parable_azi);

    let mut idx = GVBAP_SPREAD_NUM_VSO_ELE - 2;
    h.spread_gains_spread[obj_no] =
        (tmp_gain_ele1[idx] >> 1) + (f_mult(tmp, tmp_gain_ele2[idx]) >> 1); /* exp 1 */
    obj_no += 1;
    idx = GVBAP_SPREAD_NUM_VSO_ELE - 1;
    h.spread_gains_spread[obj_no] =
        (tmp_gain_ele1[idx] >> 1) + (f_mult(tmp, tmp_gain_ele2[idx]) >> 1); /* exp 1 */

    /* normalization due to extension of elevation from +/-90 degrees to +/-180 degrees */
    let diff_ele_norm = if obj_angle_ele >= 0 {
        let ele_norm = MAXVAL_DBL - obj_angle_ele;
        ele_norm.wrapping_sub(obj_angle_ele)
    } else {
        let ele_norm = MINVAL_DBL.wrapping_sub(obj_angle_ele);
        ele_norm.wrapping_sub(obj_angle_ele)
    };

    let tmp2 = parable_clamp(f_mult(parable_ele, f_mult(diff_ele_norm, diff_ele_norm)));
    let mut norm_term = (MAXVAL_DBL >> 1) + (f_mult(tmp, tmp2) >> 1); /* exp 1 */
    let mut e = 1;
    norm_term = inv_fixp_e(norm_term, &mut e);
    norm_term = scale_value(norm_term, e - 1); /* exp 1 */
    for g in h.spread_gains_spread.iter_mut() {
        let mut t = f_mult(norm_term, *g).wrapping_shl(1); /* exp 1 */
        if t < 0 {
            t = MAXVAL_DBL;
        }
        *g = t; /* exp 1 */
    }
}

/// `gVBAPRenderer_Spread_internal_normalizeVector()`
fn spread_normalize_vector(vector: &mut [FixpDbl]) {
    let vector_length = vector.len() as i32;
    let mut nrg: FixpDbl = 0;

    let mut shift = 32 - f_normz(vector_length - 1);
    let headroom = get_scalefactor(vector);

    for &v in vector.iter() {
        let tmp = v << headroom;
        nrg = nrg.wrapping_add(f_mult(tmp, tmp) >> shift);
    }

    if shift & 1 != 0 {
        nrg >>= 1;
        shift += 1;
    }

    let mut e = shift - (headroom << 1);

    let tmp = f_max(1, scale_value(SPREAD_80DB_SQR, f_max(-31, -e - 26)));
    nrg = f_max(nrg, tmp);

    let denom = inv_sqrt_norm2(nrg, &mut e);
    e -= (shift >> 1) - headroom;
    e -= 3;

    if e >= 0 {
        for v in vector.iter_mut() {
            *v = f_mult(*v, denom).wrapping_shl(e as u32);
        }
    } else {
        let e = -e;
        for v in vector.iter_mut() {
            *v = f_mult(*v, denom).wrapping_shr(e as u32);
        }
    }
}

/// `gVBAPRenderer_Spread_internal_calculateGains()`
fn spread_calculate_gains(
    h: &mut GVbapRenderer,
    obj_angle_azi: FixpDbl,
    obj_angle_ele: FixpDbl,
    spread: &[FixpDbl; 3],
    gain_array: &mut [FixpDbl],
) {
    let mut tmp_gain_azi1 = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_AZI];
    let mut tmp_gain_azi2 = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_AZI];
    let mut tmp_gain_ele1 = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_ELE];
    let mut tmp_gain_ele2 = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_ELE];
    let mut tmp_gain_ele = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_AZI];
    let spread_angle_azi;
    let mut spread_angle_ele;
    let mut mute_vo_gh = false;

    if h.has_uniform_spread != 0 {
        spread_angle_azi = spread[0];
        spread_angle_ele = spread[0] >> 1;
    } else {
        spread_angle_azi = spread[0];
        spread_angle_ele = spread[1];
        if spread_angle_ele < 0 {
            spread_angle_ele = MAXVAL_DBL;
        }
    }

    /* smoothing gain for transition between spread=0 and spread>0 w/o jumps */
    let mut smooth_gain_temp = f_mult(
        fl2fxconst_dbl(0.89),
        f_min(MAXVAL_DBL / 6, f_max(spread_angle_azi, spread_angle_ele)),
    );
    let mut smooth_gain = (smooth_gain_temp.wrapping_shl(2)).wrapping_add(smooth_gain_temp << 1); /* "*6" */
    smooth_gain_temp = f_mult(
        fl2fxconst_dbl(0.11),
        f_min(MAXVAL_DBL / 6, f_min(spread_angle_azi, spread_angle_ele)),
    );
    smooth_gain = smooth_gain
        .wrapping_add((smooth_gain_temp.wrapping_shl(2)).wrapping_add(smooth_gain_temp << 1)); /* "*6" */

    /* mapping to MPEGH effect strength behavior - perceptually tuned */
    let mut spread_angle_azi_mapped = spread_mapping_to_h(spread_angle_azi); /* exp 1 */
    let mut spread_angle_ele_mapped = spread_mapping_to_h(spread_angle_ele); /* exp 1 */

    /* preprocessing steps for non-uniform spread */
    if spread_angle_azi_mapped > spread_angle_ele_mapped {
        /* ensure minimum vertical spread */
        spread_angle_ele_mapped = f_max(spread_angle_ele_mapped, GVBAP_SPREAD_OPEN_ANGLE_VSO >> 1);
    /* exp 1 */
    } else if spread_angle_azi_mapped < spread_angle_ele_mapped {
        /* muting spread on ghost loudspeakers at +/-90 degrees elevation in case of vertical
         * spread */
        mute_vo_gh = true;

        /* ensure minimum horizontal spread */
        spread_angle_azi_mapped = f_max(spread_angle_azi_mapped, GVBAP_SPREAD_OPEN_ANGLE_VSO >> 1);
        /* exp 1 */
    }

    /* calculate parable values for spread gain */
    let parable_azi = spread_parable(spread_angle_azi_mapped);
    let parable_ele = spread_parable(spread_angle_ele_mapped);

    /* calculate minimum number of loop itarations required for calculating the largest angle
     * difference that does not lead to zero */
    let loop_limit_azi = index_ring_limiter(spread_angle_azi_mapped);
    let loop_limit_ele = index_ring_limiter(spread_angle_ele_mapped); /* also here spread_numVSO_azi is required */

    /* calculate azimuth layer gains */
    spread_calculate_layer_gains(obj_angle_azi, loop_limit_azi, parable_azi, &mut tmp_gain_azi1);
    for n in 0..GVBAP_SPREAD_NUM_VSO_AZI / 2 {
        let idx = GVBAP_SPREAD_NUM_VSO_AZI / 2 + n;
        tmp_gain_azi2[n] = tmp_gain_azi1[idx];
        tmp_gain_azi2[idx] = tmp_gain_azi1[n];
    }

    /* calculate elevation layer gains */
    spread_calculate_layer_gains(obj_angle_ele, loop_limit_ele, parable_ele, &mut tmp_gain_ele);
    for m in 0..GVBAP_SPREAD_NUM_VSO_ELE {
        tmp_gain_ele1[m] = tmp_gain_ele[RING_INDICES30_1[m]];
        tmp_gain_ele2[m] = tmp_gain_ele[RING_INDICES30_2[m]];
    }

    /* calculate VSO gains */
    spread_calculate_vso_gains(
        h,
        obj_angle_ele,
        &tmp_gain_azi1,
        &tmp_gain_azi2,
        &tmp_gain_ele1,
        &tmp_gain_ele2,
        parable_azi,
        parable_ele,
    );

    /* applying VSO spread gains to VSO loudspeaker gains (panning gains of VSO) */
    let len = h.gain_cache_length.max(0) as usize;
    for g_no in 0..len {
        let mut tmp = gain_array[g_no] >> 2; /* exp 5 */

        if !mute_vo_gh
            || (g_no as i32 != h.ghost_voice_of_god_speaker_index as i32
                && g_no as i32 != h.ghost_voice_of_hell_speaker_index as i32)
        {
            for obj_no in 0..GVBAP_SPREAD_NUM_VSO {
                tmp = tmp.wrapping_add(
                    f_mult(
                        f_mult(h.spread_gains_vso[obj_no][g_no], h.spread_gains_spread[obj_no]),
                        smooth_gain,
                    ) >> 4,
                ); /* exp 5 */
            }
        }

        gain_array[g_no] = tmp; /* exp 5 */
    }

    /* gain normalization */
    spread_normalize_vector(&mut gain_array[..len]);
}

/// `gVBAPRenderer_Spread_renderSpread()`
fn spread_render_spread(
    h: &mut GVbapRenderer,
    output_gain_array: &mut [FixpDbl],
    mut oam_sample: OamSample,
) {
    let spread = [oam_sample.spread_angle, oam_sample.spread_height, oam_sample.spread_depth];
    let gain = oam_sample.gain;
    let num_channels = h.num_channels.max(0) as usize;

    oam_sample.spread_angle = 0;
    oam_sample.spread_height = 0;
    oam_sample.spread_depth = 0;
    oam_sample.gain = fl2fxconst_dbl(1.0 / 8.0);
    output_gain_array[..num_channels].fill(0);

    /* calculate VBAP gains (assuming spread = 0 for implemented MDAP spread, no downmixing of ghost
     * speakers, gain factor set to 1.0 because gain factor will be applied after spread gains have
     * also been factored in) */
    let mut gain_array = core::mem::take(&mut h.spread_gain_array);
    calculate_vbap(h, &oam_sample, &mut gain_array, 1);

    /* calculate spread gains */
    spread_calculate_gains(h, oam_sample.sph.azi, oam_sample.sph.ele, &spread, &mut gain_array);

    /* downmix ghost loudspeakers to real loudspeaker array */
    if let Some(dmx) = h.downmix_matrix.as_ref() {
        for row in 0..h.downmix_matrix_num_rows.max(0) as usize {
            let dmx_row = &dmx[row];
            for col in 0..h.downmix_matrix_num_cols.max(0) as usize {
                output_gain_array[row] =
                    output_gain_array[row].wrapping_add(f_mult(gain_array[col], dmx_row[col]) >> 5);
            }
        }
    }
    h.spread_gain_array = gain_array;

    /* gain normalization */
    spread_normalize_vector(&mut output_gain_array[..num_channels]);

    /* Apply gain factor */
    for g in output_gain_array[..num_channels].iter_mut() {
        *g = f_mult(*g, gain).wrapping_shl(3);
    }
}

/// `sNorm = (((float)atan(0.1f / d)) * 0.31830988f * 2.0f - 0.0636f) * 1.0679f;`
/// `sTmp = 0.5f * ((1.0f - d) + sNorm);`
static DISTANCE_TO_SPREAD_TABLE: [i16; 257] = [
    32767, 32269, 31771, 31276, 30785, 30299, 29819, 29347, 28882, 28426, 27980, 27543, 27117,
    26702, 26297, 25904, 25522, 25150, 24790, 24440, 24101, 23771, 23452, 23143, 22843, 22552,
    22269, 21995, 21729, 21471, 21220, 20976, 20739, 20508, 20284, 20065, 19852, 19645, 19443,
    19245, 19053, 18864, 18680, 18501, 18325, 18152, 17984, 17819, 17657, 17498, 17342, 17189,
    17039, 16891, 16746, 16604, 16463, 16325, 16189, 16055, 15923, 15793, 15665, 15538, 15414,
    15290, 15169, 15049, 14930, 14812, 14696, 14582, 14468, 14356, 14245, 14135, 14026, 13918,
    13812, 13706, 13601, 13497, 13394, 13292, 13190, 13090, 12990, 12891, 12793, 12695, 12598,
    12502, 12406, 12311, 12217, 12123, 12030, 11937, 11845, 11754, 11663, 11572, 11482, 11393,
    11304, 11215, 11127, 11039, 10952, 10865, 10778, 10692, 10606, 10521, 10436, 10351, 10267,
    10183, 10099, 10015, 9932, 9850, 9767, 9685, 9603, 9521, 9440, 9359, 9278, 9197, 9117, 9037,
    8957, 8877, 8798, 8719, 8640, 8561, 8482, 8404, 8326, 8248, 8170, 8092, 8015, 7938, 7861, 7784,
    7707, 7630, 7554, 7478, 7402, 7326, 7250, 7174, 7099, 7024, 6948, 6873, 6798, 6724, 6649, 6574,
    6500, 6426, 6352, 6277, 6204, 6130, 6056, 5982, 5909, 5836, 5762, 5689, 5616, 5543, 5470, 5398,
    5325, 5252, 5180, 5107, 5035, 4963, 4891, 4819, 4747, 4675, 4603, 4531, 4460, 4388, 4317, 4245,
    4174, 4103, 4031, 3960, 3889, 3818, 3747, 3677, 3606, 3535, 3464, 3394, 3323, 3253, 3182, 3112,
    3042, 2972, 2901, 2831, 2761, 2691, 2621, 2551, 2482, 2412, 2342, 2272, 2203, 2133, 2064, 1994,
    1925, 1855, 1786, 1717, 1647, 1578, 1509, 1440, 1371, 1302, 1233, 1164, 1095, 1026, 957, 888,
    820, 751, 682, 614, 545, 476, 408, 339, 271, 202, 134, 66, -3,
];

/// `gVBAPRenderer_Spread_distanceToSpreadIndividual()`
fn spread_distance_to_spread_individual(mut d: FixpDbl, s_in: FixpDbl) -> FixpDbl {
    /* limit(d, 0.0001f, 1.0f); */
    d = f_max(d, fl2fxconst_dbl(0.0001));

    /* table lookup with linear interpolation */
    let i = (d >> (DFRACT_BITS - 1 - 8)) as usize;
    let f = (d & (MAXVAL_DBL >> 8)) << 8;

    let t = &DISTANCE_TO_SPREAD_TABLE;
    let mut s_tmp =
        fx_sgl2fx_dbl(t[i]).wrapping_add(f_mult(f, (t[i + 1] as i32 - t[i] as i32) as i16));

    /* limit(sTmp, 0.0f, 1.0f); */
    if s_tmp < fl2fxconst_dbl(0.01f32 as f64) {
        s_tmp = 0;
    }

    /* return (1.0f - sIn) * sTmp + sIn; */
    f_mult(MAXVAL_DBL.wrapping_sub(s_in), s_tmp).wrapping_add(s_in)
}

/// `gVBAPRenderer_Spread_distanceToSpread()`
fn spread_distance_to_spread(oam_sample: &mut OamSample, has_uniform_spread: i32) {
    let mut length = MAXVAL_DBL;
    if oam_sample.sph.rad < fl2fxconst_dbl(1.0 / 16.0) {
        length = oam_sample.sph.rad.wrapping_shl(4);
    }

    if length < fl2fxconst_dbl(0.999) {
        if has_uniform_spread != 0 {
            oam_sample.spread_angle =
                spread_distance_to_spread_individual(length, oam_sample.spread_angle);
        } else {
            oam_sample.spread_angle =
                spread_distance_to_spread_individual(length, oam_sample.spread_angle);

            let tmp = f_min(MAXVAL_DBL >> 1, oam_sample.spread_height).wrapping_shl(1);
            oam_sample.spread_height = spread_distance_to_spread_individual(length, tmp) >> 1;

            let tmp = f_min(MAXVAL_DBL >> 4, oam_sample.spread_depth).wrapping_shl(4);
            oam_sample.spread_depth = spread_distance_to_spread_individual(length, tmp) >> 4;
        }
    }
}

/// `gVBAPRenderer_Spread_initVSOPositions()`
fn spread_init_vso_positions(
    azi_vso_angles: &mut [FixpDbl; GVBAP_SPREAD_NUM_VSO_AZI],
    ele_vso_angles: &mut [FixpDbl; GVBAP_SPREAD_NUM_VSO_ELE],
) {
    /* define azimuth angles */
    azi_vso_angles[0] = 0;
    for na in 1..GVBAP_SPREAD_NUM_VSO_AZI {
        azi_vso_angles[na] = azi_vso_angles[na - 1].wrapping_add(GVBAP_SPREAD_OPEN_ANGLE_VSO);
    }

    /* define elevation angles */
    ele_vso_angles[0] = MINVAL_DBL >> 1; /* -90 degrees */
    for ne in 1..GVBAP_SPREAD_NUM_VSO_ELE {
        ele_vso_angles[ne] = ele_vso_angles[ne - 1] + GVBAP_SPREAD_OPEN_ANGLE_VSO;
    }
}

/* ------ END OF SPREAD ------*/

/// `gVBAPRenderer_Open(phgVBAPRenderer, numObjects, frameLength, oamFrameLength,
/// outGeometryInfo, outChannels, outCICPIndex, hasUniformSpread, renderMode)`: creates the
/// renderer for `num_objects` objects and the loudspeaker setup `out_geometry_info`
/// (`out_channels` speakers including LFEs): triangulation, inverse matrices, ghost speaker
/// downmix and (enhanced mode) the panning gains of the virtual spread objects.
///
/// `out_cicp_index` selects the precomputed imaginary speaker / superset tables in the enhanced
/// mode (`render_mode == GVBAP_ENHANCED`). Returns 0 on success, -1 for unsupported parameters
/// and -2 for failed "allocations". Like in C++, `*ph` is set as soon as the handle exists, also
/// if an error is returned afterwards.
pub fn gvbap_renderer_open(
    ph: &mut Option<Box<GVbapRenderer>>,
    num_objects: i32,
    frame_length: i32,
    oam_frame_length: i32,
    out_geometry_info: &[Cicp2GeometryChannelGeometry],
    out_channels: i32,
    out_cicp_index: i32,
    has_uniform_spread: i32,
    render_mode: i32,
) -> i32 {
    /* some value range checks */
    if num_objects > GVBAPRENDERER_MAX_OBJECTS {
        /* too many objects */
        return -1;
    }
    if out_channels > GVBAPRENDERER_MAX_CHANNEL_OUT {
        /* too many speakers */
        return -1;
    }
    if frame_length > GVBAPRENDERER_MAX_FRAMELENGTH {
        return -1;
    }
    /* Not checked in C++ (undefined behaviour there): number of OAM frames out of range. */
    if oam_frame_length <= 0 {
        mpeghdec_fdk::deviation::hit();
        return -1;
    }
    let num_oam_frames = frame_length / oam_frame_length;
    if num_oam_frames <= 0 || num_oam_frames > GVBAPRENDERER_MAX_OAM_FRAMES_PER_CORE_FRAME as i32 {
        mpeghdec_fdk::deviation::hit();
        return -1;
    }

    *ph = Some(Box::default());
    let h: &mut GVbapRenderer = ph.as_deref_mut().unwrap();

    /* allocate memory for speakerlist and fill it with values and save the mapping order */
    if out_channels < 0 {
        return -2; /* could not allocate memory */
    }
    let out_ch = out_channels as usize;
    h.speaker_setup.speaker_list = vec![Speaker::default(); out_ch];
    h.speaker_setup.mapping = vec![0; out_ch];

    /* Save azimuth an elevation temporarily for vertex, triangle-List and downmixMatrix
     * generation */
    /* In this way the same order as speakerList is guaranteed for the vertex list and the saved
     * mapping will fit to it */
    let mut azimuth = [0 as FixpDbl; GVBAPRENDERER_MAX_CHANNEL_OUT as usize];
    let mut elevation = [0 as FixpDbl; GVBAPRENDERER_MAX_CHANNEL_OUT as usize];

    let mut cnt = 0usize;
    let ss = &mut h.speaker_setup;
    ss.has_height_speakers = false;
    ss.covers_upper_half_sphere = false;
    ss.covers_lower_half_sphere = false;
    for (i, g) in out_geometry_info[..out_ch].iter().enumerate() {
        if g.lfe <= 0 {
            /* first copy all non LFE channels */
            let s = &mut ss.speaker_list[cnt];
            s.sph.azi = g.az.wrapping_mul(11930464); /* real value is 11930464.71 -> round down will prevent overflow */
            s.sph.ele = g.el.wrapping_mul(11930464); /* max error is 0.3*180*2^-31 */
            s.sph.rad = fl2fxconst_dbl(1.0);
            s.is_lfe = false;
            s.is_ghost = false;
            /* Copy non LFE channels for creating triangles and downmix matrix */
            azimuth[cnt] = s.sph.azi;
            elevation[cnt] = s.sph.ele;

            /* get some speaker setup covering information */
            if g.el > 21 {
                ss.has_height_speakers = true;
            } else if g.el >= 5 {
                ss.covers_upper_half_sphere = true;
            } else if g.el <= -5 {
                ss.covers_lower_half_sphere = true;
            }

            ss.mapping[cnt] = i as i32;
            cnt += 1;
        }
    }
    let num_lfe = out_channels - cnt as i32;
    for (i, g) in out_geometry_info[..out_ch].iter().enumerate() {
        if g.lfe > 0 {
            /* then copy all LFE channels */
            let s = &mut ss.speaker_list[cnt];
            s.sph.azi = g.az.wrapping_mul(11930464); /* real value is 11930464.71 -> round down will prevent overflow */
            s.sph.ele = g.el.wrapping_mul(11930464); /* max error is 0.3*180*2^-31 */
            s.sph.rad = fl2fxconst_dbl(1.0);
            s.is_lfe = true;
            s.is_ghost = true;
            ss.mapping[cnt] = i as i32;
            cnt += 1;
        }
    }
    ss.speaker_list_size = cnt as i32;

    /* Generate vertexList and add Ghost speakers if necessary */
    let mut v_list = Box::<VertexList>::default();
    let vl = &mut *v_list;
    reset_vertex_list(vl);

    qh_gen_vertex_list(out_channels - num_lfe, &azimuth, &elevation, vl);

    let mut t_list = Box::<TriangleList>::default();
    let tl = &mut *t_list;
    reset_triangle_list(tl);

    /* Generate triangle List for speaker triangles and a downmixMatrix if Ghostspeaker where
     * added */
    if qh_sphere_triangulation(
        if render_mode == GVBAP_ENHANCED { out_cicp_index } else { 0 },
        vl,
        tl,
        &mut h.downmix_matrix,
        &mut h.downmix_matrix_num_rows,
        &mut h.downmix_matrix_num_cols,
    ) != 0
    {
        return -2; /* could not allocate memory */
    }

    if vl.size == out_channels - num_lfe {
        /* if no downmixMatrix exists then free memory and set pointer to NULL */
        h.downmix_matrix = None;
    }

    /* allocate memory for speakerTriplet and fill it with values */
    /* copy triangle list and fill matrices */
    h.speaker_setup.speaker_triplet = tl.element[..tl.size as usize]
        .iter()
        .map(|t| SpeakerTriplet {
            triangle: t.index,
            matrix: [
                vl.element[t.index[0] as usize].xyz,
                vl.element[t.index[1] as usize].xyz,
                vl.element[t.index[2] as usize].xyz,
            ],
            ..Default::default()
        })
        .collect();
    h.speaker_setup.speaker_triplet_size = tl.size;

    let num_ghosts = vl.size - (out_channels - num_lfe); /* Number of Ghostspeakers */

    /* Calculate inverse matrices */
    generate_inverse_matrices(h);

    /* Set parameters */
    h.num_lfe = num_lfe as i8; /* Number of LFE speakers */
    h.num_channels = out_channels as i8; /* Number of real speakers including LFE */
    h.num_objects = num_objects as i8; /* Number of objects which will be rendered to the given loudspeaker setup */
    h.frame_length = frame_length as i16; /* Number of samples for one frame */
    h.oam_frame_length = oam_frame_length as i16; /* Number of samples for one OAM frame */
    h.num_oam_frames = num_oam_frames as i8; /* number of OAM frames per core frame*/
    h.num_ghosts = num_ghosts as i8; /* Number of ghost speakers */
    h.start_gains_filled = 0; /* indicates that no endGains copied to startGains yet */

    let gain_cache_length = out_channels + num_ghosts - num_lfe;
    h.gain_cache_length = gain_cache_length as i8;

    /* Alocate memory for gain cache. Length is number of real speakers including LFE + number of
     * ghosts */
    h.gain_cache = vec![0; gain_cache_length.max(0) as usize];

    /* Alocate memory for start and end gains. Length is number of real speakers including LFE */
    /* Dim: objects x (number of output channels) */
    let n_ch = h.num_channels as i32;
    let alloc = || -> Option<Matrix2D<FixpDbl>> {
        if num_objects < 0 || n_ch < 0 {
            return None;
        }
        fdk_calloc_matrix_2d(num_objects as u32, n_ch as u32)
    };
    let (Some(sg), Some(eg), Some(st), Some(sc)) = (alloc(), alloc(), alloc(), alloc()) else {
        return -2; /* could not allocate memory */
    };
    h.start_gains = sg;
    h.end_gains = eg;
    h.step_state = st;
    h.scale_state = sc;

    h.start_gains_max = vec![0; n_ch as usize];
    h.prev_gains_max = vec![0; n_ch as usize];
    h.oam_parsed_data = vec![0; h.num_objects as usize * NMAX];

    h.render_mode = render_mode as u8;

    if render_mode == GVBAP_ENHANCED {
        /* initialization of spread rendering */
        let spread_num_involved_ls = gain_cache_length.max(0) as usize;

        h.ghost_voice_of_god_speaker_index = -1;
        h.ghost_voice_of_hell_speaker_index = -1;
        for i in 0..spread_num_involved_ls {
            /* Assumption: A speaker is a ghost speaker if it is not downmixed to exactly one output
             * speaker */
            let mut n_output_targets = 0;
            if let Some(dmx) = h.downmix_matrix.as_ref() {
                for j in 0..h.downmix_matrix_num_rows.max(0) as usize {
                    if dmx[j][i] != 0 {
                        n_output_targets += 1;
                    }
                }
            }

            if n_output_targets != 1 {
                let el = vl.element[i].sph.ele;
                if el > 85 * 11930464 {
                    h.ghost_voice_of_god_speaker_index = i as i8;
                }
                if el < -85 * 11930464 {
                    h.ghost_voice_of_hell_speaker_index = i as i8;
                }
            }
        }

        h.spread_gain_array = vec![0; spread_num_involved_ls];

        h.spread_gains_vso = vec![vec![0; spread_num_involved_ls]; GVBAP_SPREAD_NUM_VSO];

        let mut azi_vso_angles = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_AZI];
        let mut ele_vso_angles = [0 as FixpDbl; GVBAP_SPREAD_NUM_VSO_ELE];
        let mut obj_no = 0;
        let mut oam = OamSample::default();

        /* virtual spread object positions */
        spread_init_vso_positions(&mut azi_vso_angles, &mut ele_vso_angles);

        oam.gain = fl2fxconst_dbl(1.0);
        oam.spread_angle = 0;
        oam.spread_height = 0;
        oam.spread_depth = 0;

        let mut vso = core::mem::take(&mut h.spread_gains_vso);

        /* panning of VSOs with VBAP */
        for ne in 1..GVBAP_SPREAD_NUM_VSO_ELE - 1 {
            for na in 0..GVBAP_SPREAD_NUM_VSO_AZI {
                oam.sph.azi = azi_vso_angles[na];
                oam.sph.ele = ele_vso_angles[ne];
                oam.sph.rad = fl2fxconst_dbl(1.0);
                oam.goa_bs_object_distance = 0;
                oam.cart = spherical_to_cartesian(oam.sph);

                /* "non-downmix" gain output including ghost speakers */
                calculate_vbap(h, &oam, &mut vso[obj_no], 1);
                obj_no += 1;
            }
        }

        /* vbap-panning of virtual spread objects at elevation -90 degrees */
        oam.sph.azi = 0;
        oam.sph.ele = ele_vso_angles[0];
        oam.sph.rad = fl2fxconst_dbl(1.0);
        oam.cart = spherical_to_cartesian(oam.sph);
        calculate_vbap(h, &oam, &mut vso[obj_no], 1);
        obj_no += 1;

        /* vbap-panning of virtual spread objects at elevation +90 degrees */
        oam.sph.azi = 0;
        oam.sph.ele = ele_vso_angles[GVBAP_SPREAD_NUM_VSO_ELE - 1];
        oam.sph.rad = fl2fxconst_dbl(1.0);
        oam.cart = spherical_to_cartesian(oam.sph);
        calculate_vbap(h, &oam, &mut vso[obj_no], 1);

        h.spread_gains_vso = vso;
    }

    /* Alocate memory for OAM samples */
    let nof = num_oam_frames as usize;
    let nobj = num_objects as usize;
    h.oam_samples = vec![vec![OamSample::default(); nobj]; nof];

    /* set initial values for the case that the first frame is a bad one */
    for i in 0..nobj {
        let s = &mut h.oam_samples[nof - 1][i];
        s.gain = fl2fx_dbl(1.00f32 / 8.0f32);
        s.goa_bs_object_distance = 0;
        s.spread_angle = 0;
        s.sph.azi = 0;
        s.sph.ele = 0;
        s.sph.rad = fl2fxconst_dbl(1.0 / 16.0);
        for j in 0..nof {
            h.oam_samples[j][i].sph.rad = fl2fxconst_dbl(1.0 / 16.0);
        }
        let s = &mut h.oam_samples[nof - 1][i];
        s.spread_depth = 0;
        s.spread_height = 0;
    }
    h.metadata_present[nof - 1] = 1;

    /* CONCEALMENT */
    h.oam_samples_valid = vec![vec![OamSample::default(); nobj]; nof];

    /* if no good frame is stored yet, then everything will rendered to center with gain 1.0 */
    for s in h.oam_samples_valid[nof - 1].iter_mut() {
        s.gain = fl2fx_dbl(1.00f32 / 8.0f32);
        s.goa_bs_object_distance = 0;
        s.sph.azi = 0;
        s.sph.ele = 0;
        s.sph.rad = fl2fxconst_dbl(1.0 / 16.0);
        s.spread_angle = 0;
        s.spread_depth = 0;
        s.spread_height = 0;
    }
    h.metadata_present_valid[nof - 1] = 1;

    if render_mode == GVBAP_ENHANCED {
        h.has_uniform_spread = has_uniform_spread as u8;
    } else {
        h.has_uniform_spread = 1; /* set uniform Spread as true */
    }

    0
}

/// `inv_length_table` of `gVBAPRenderer_RenderFrame_Time()`: `1 / (length / 64)` scaled.
static INV_LENGTH_TABLE: [i32; 16] = [
    0x04000000, 0x02000000, 0x01555555, 0x01000000, 0x00CCCCCD, 0x00AAAAAB, 0x00924925, 0x00800000,
    0x0071C71C, 0x00666666, 0x005D1746, 0x00555555, 0x004EC4EC, 0x00492492, 0x00444444, 0x00400000,
];

/// `gVBAPRenderer_RenderFrame_Time(hgVBAPRenderer, inputBuffer, outputBuffer,
/// startSamplePosition, inputBufferChannelOffset)`: renders one frame of all objects and adds
/// the result (saturated) to `output_buffer`.
///
/// Object `o` is read from `input_buffer[o * input_buffer_channel_offset ..]`, output channel `c`
/// is `output_buffer[c * frameLength .. (c + 1) * frameLength]`. The gains are interpolated
/// linearly from the gains of the previous OAM sample to the gains of the current one over each
/// OAM frame; the first `start_sample_position` samples of an OAM frame continue the ramp of the
/// previous frame.
pub fn gvbap_renderer_render_frame_time(
    h: &mut GVbapRenderer,
    input_buffer: &[VbapPcm],
    output_buffer: &mut [VbapPcm],
    start_sample_position: i32,
    input_buffer_channel_offset: i32,
) -> i32 {
    if h.oam_data_valid == 0 {
        oam_concealment(h);
    }

    let num_oam_frames = h.num_oam_frames as i32;
    let mut oam_frame = 0;
    let mut start_sample = 0i32;
    let mut oam_frame_with_meta_data: i32 = -1;
    let num_channels = h.num_channels.max(0) as usize;
    let num_objects = h.num_objects.max(0) as usize;

    /* number of objects in fractional representation scaled by VBAP_MAX_SCALE_OBJECTS bits (maximum
     * of 31 Objects) */
    let num_objects_fract =
        (h.num_objects as i32).wrapping_shl((DFRACT_BITS - 1 - VBAP_MAX_SCALE_OBJECTS) as u32);

    let mut end_gains_max = [0 as FixpDbl; GVBAPRENDERER_MAX_CHANNEL_OUT as usize];
    let mut output_cache = [0 as FixpDbl; GVBAPRENDERER_MAX_FRAMELENGTH as usize];

    while oam_frame < num_oam_frames {
        if h.metadata_present[oam_frame as usize] == 0 && oam_frame != num_oam_frames - 1 {
            oam_frame += 1;
            continue;
        }

        let stop_sample = (oam_frame + 1) * h.oam_frame_length as i32;
        let stop_frame = if h.metadata_present[oam_frame as usize] != 0 {
            oam_frame_with_meta_data = oam_frame;
            oam_frame as usize
        } else if oam_frame_with_meta_data >= 0 {
            oam_frame_with_meta_data as usize
        } else {
            /* no OAM frame with metadata (only possible if the handle was modified directly;
             * undefined behaviour in C++): use the current frame */
            mpeghdec_fdk::deviation::hit();
            oam_frame as usize
        };

        let length = stop_sample - start_sample;
        /* index out of range is undefined behaviour in C++ */
        let inv_length =
            INV_LENGTH_TABLE.get(((length >> 6) - 1) as usize).copied().unwrap_or_else(|| {
                mpeghdec_fdk::deviation::hit();
                0
            });

        /* Do rendering for all objects */
        end_gains_max[..num_channels].fill(0);

        /* First calculate gains for all objects */
        let mut end_gains = core::mem::take(&mut h.end_gains);
        for object in 0..num_objects {
            /* create local copy of sample, as radius is modified */
            let mut oam_stop_sample = h.oam_samples[stop_frame][object];

            if h.render_mode as i32 == GVBAP_LEGACY {
                oam_stop_sample.sph.rad = fl2fxconst_dbl(1.0 / 16.0);
                oam_stop_sample.cart = spherical_to_cartesian(oam_stop_sample.sph);
                calculate_vbap(h, &oam_stop_sample, &mut end_gains[object], 0);
            } else {
                /* distance to spread mapping */
                spread_distance_to_spread(&mut oam_stop_sample, h.has_uniform_spread as i32);

                oam_stop_sample.sph.rad = fl2fxconst_dbl(1.0 / 16.0);
                oam_stop_sample.cart = spherical_to_cartesian(oam_stop_sample.sph);

                if oam_stop_sample.spread_angle != 0
                    || (h.has_uniform_spread == 0 && oam_stop_sample.spread_height != 0)
                {
                    spread_render_spread(h, &mut end_gains[object], oam_stop_sample);
                } else {
                    calculate_vbap(h, &oam_stop_sample, &mut end_gains[object], 0);
                }
            }

            for channel in 0..num_channels {
                end_gains_max[channel] = f_max(end_gains_max[channel], end_gains[object][channel]);
            }

            /* is it the first run? If yes copy endGains to startGains */
            if h.start_gains_filled == 0 {
                h.start_gains[object].copy_from_slice(&end_gains[object]);
            }
        }
        h.end_gains = end_gains;
        if h.start_gains_filled == 0 {
            for channel in 0..num_channels {
                h.prev_gains_max[channel] = 0;
                h.start_gains_max[channel] = end_gains_max[channel];
            }
        }
        h.start_gains_filled = 1;

        let length_u = length.max(0) as usize;
        let ssp = start_sample_position.max(0) as usize;

        /* Add object channels multiplied by gain to output buffer */
        for channel in 0..num_channels {
            let mapped_channel = h.speaker_setup.mapping[channel];
            output_cache[..length_u].fill(0);

            let mut tmp = f_max(h.start_gains_max[channel], end_gains_max[channel]);
            let max_gain = f_max(tmp, h.prev_gains_max[channel]);

            h.prev_gains_max[channel] = tmp;
            h.start_gains_max[channel] = end_gains_max[channel];

            tmp = f_mult(max_gain, num_objects_fract);

            /* calculate headroom of accumulated gains */
            let mut s = f_max(cnt_leading_zeros(tmp) - 1, 0);

            /* scalefactor s can be in the range of -VBAP_SCALE_GAINS to VBAP_MAX_SCALE_OBJECTS */
            s = f_max(-VBAP_SCALE_GAINS, VBAP_MAX_SCALE_OBJECTS - s);

            /* scalefactor s2 can be in the range of 1 to VBAP_MAX_SCALE_SAMPLES+1 */
            let s2 = VBAP_MAX_SCALE_SAMPLES - VBAP_MAX_SCALE_OBJECTS + s + 1;

            /* consider fMultDiv2() */
            s -= 1;

            let s1 = if s < 0 { -s } else { s };

            for object in 0..num_objects {
                /* load States */
                #[cfg_attr(any(target_arch = "x86", target_arch = "x86_64"), allow(unused_mut))]
                let mut scale_state = h.scale_state[object][channel];
                let step_state = h.step_state[object][channel];

                /* linear interpolation */
                let step = f_mult_div2(
                    h.end_gains[object][channel].wrapping_sub(h.start_gains[object][channel]),
                    inv_length,
                );
                let mut scale = h.start_gains[object][channel];

                /* Save last endGains to next startGains */
                h.start_gains[object][channel] = h.end_gains[object][channel];
                /* Skip cases which are not exactly zero due to calculation differences (improves
                 * accuracy and workload) */
                const MIN_GAIN_SKIP: FixpDbl = 0xfff;
                if scale < MIN_GAIN_SKIP
                    && step == 0
                    && scale_state < MIN_GAIN_SKIP
                    && step_state == 0
                {
                    continue;
                }
                let p_in = &input_buffer
                    [(object as i32 * input_buffer_channel_offset + start_sample) as usize..];

                let first = ssp;
                // x86: the same loops in an AVX2 clone (the 64 bit products of `fMult()` and
                // the ramp are vectorized).
                #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
                {
                    let (sh, left) =
                        if s < 0 { ((s1 - 1) as u32, true) } else { ((s1 + 1) as u32, false) };
                    /* process first startSamplePosition samples */
                    let _ = mix_ramp(
                        &mut output_cache[..first],
                        &p_in[..first],
                        scale_state,
                        step_state,
                        sh,
                        left,
                    );
                    /* process remaining samples (none if first >= length) */
                    let rest = first.min(length_u)..length_u;
                    scale = mix_ramp(
                        &mut output_cache[rest.clone()],
                        &p_in[rest],
                        scale,
                        step,
                        sh,
                        left,
                    );
                }
                #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
                {
                    if s < 0 {
                        let sh = (s1 - 1) as u32;
                        for sample in 0..first {
                            /* process first startSamplePosition samples */
                            scale_state = scale_state.wrapping_add(step_state);
                            output_cache[sample] = output_cache[sample]
                                .wrapping_add(f_mult(p_in[sample], scale_state).wrapping_shl(sh));
                        }
                        for sample in first..length_u {
                            /* process remaining samples */
                            scale = scale.wrapping_add(step);
                            output_cache[sample] = output_cache[sample]
                                .wrapping_add(f_mult(p_in[sample], scale).wrapping_shl(sh));
                        }
                    } else {
                        let sh = (s1 + 1) as u32;
                        for sample in 0..first {
                            /* process first startSamplePosition samples */
                            scale_state = scale_state.wrapping_add(step_state);
                            output_cache[sample] = output_cache[sample]
                                .wrapping_add(f_mult(p_in[sample], scale_state).wrapping_shr(sh));
                        }
                        for sample in first..length_u {
                            /* process remaining samples */
                            scale = scale.wrapping_add(step);
                            output_cache[sample] = output_cache[sample]
                                .wrapping_add(f_mult(p_in[sample], scale).wrapping_shr(sh));
                        }
                    }
                }
                /* save states */
                h.scale_state[object][channel] = scale;
                h.step_state[object][channel] = step;
            }

            let out_off = (mapped_channel * h.frame_length as i32 + start_sample) as usize;
            let p_out = &mut output_buffer[out_off..out_off + length_u];

            /* copy left shifted and saturated samples to output buffer */
            if s2 > 1 {
                for (o, &c) in p_out.iter_mut().zip(output_cache[..length_u].iter()) {
                    *o = f_add_saturate(*o, saturate_left_shift(c, s2 - 1, DFRACT_BITS));
                }
            } else {
                for (o, &c) in p_out.iter_mut().zip(output_cache[..length_u].iter()) {
                    *o = f_add_saturate(*o, c);
                }
            }
        }

        oam_frame += 1;
        start_sample = stop_sample;
    } /* while loop over all oamSamples (in case of high rate OAM) */

    0
}

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
mpeghdec_fdk::avx2_clone! {
    /// The gain ramp of the object rendering: `out[i] += fMult(inp[i], scale + (i + 1) * step)`
    /// shifted left (`left`) or right by `sh` (the amount modulo 32); returns the final gain.
    fn mix_ramp(
        out: &mut [FixpDbl],
        inp: &[FixpDbl],
        scale: FixpDbl,
        step: FixpDbl,
        sh: u32,
        left: bool,
    ) -> FixpDbl {
        let n = out.len().min(inp.len());
        let (out, inp) = (&mut out[..n], &inp[..n]);
        let mut scale = scale;
        if left {
            for (o, &x) in out.iter_mut().zip(inp) {
                scale = scale.wrapping_add(step);
                *o = o.wrapping_add(f_mult(x, scale).wrapping_shl(sh));
            }
        } else {
            for (o, &x) in out.iter_mut().zip(inp) {
                scale = scale.wrapping_add(step);
                *o = o.wrapping_add(f_mult(x, scale).wrapping_shr(sh));
            }
        }
        scale
    }
}

/// `gVBAPRenderer_Close(hgVBAPRenderer)`: frees the renderer.
pub fn gvbap_renderer_close(h: Box<GVbapRenderer>) -> i32 {
    drop(h);
    0
}
