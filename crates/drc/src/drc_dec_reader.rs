// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDec_reader.cpp, drcDec_reader.h.

//! Bitstream parser for uniDrcGain(), mpegh3daUniDrcConfig() and mpegh3daLoudnessInfoSet().

use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::scale::saturate_left_shift;
use mpeghdec_fdk::{
    f_min, fl2fxconst_dbl, fl2fxconst_sgl, FixpDbl, FixpSgl, DFRACT_BITS, FRACT_BITS,
};

use crate::drc_dec_rom::*;
use crate::drc_dec_tools::*;
use crate::drc_dec_types::*;
use crate::drc_decoder::*;

/* MPEG-D DRC AMD 1 */

const UNIDRCGAINEXT_TERM: u8 = 0x0;
const UNIDRCLOUDEXT_TERM: u8 = 0x0;
const UNIDRCCONFEXT_TERM: u8 = 0x0;

fn get_z(n_nodes_max: i32) -> i32 {
    /* Z is the minimum codeword length that is needed to encode all possible timeDelta values */
    /* Z = ceil(log2(2*nNodesMax)) */
    let mut z = 1;
    while (1i32.wrapping_shl(z as u32)) < (n_nodes_max.wrapping_mul(2)) {
        z += 1;
    }
    z
}

fn get_time_delta_min(p_gset: &GainSet, delta_tmin_default: i32) -> i32 {
    if p_gset.time_delta_min_present != 0 {
        p_gset.time_delta_min as i32
    } else {
        delta_tmin_default
    }
}

/// `_compAssign`: compare and assign
#[inline]
pub(crate) fn comp_assign<T: PartialEq + Copy>(dest: &mut T, src: T) -> i32 {
    let diff = (*dest != src) as i32;
    *dest = src;
    diff
}

type Huffman = [[i8; 2]];

fn decode_huffman_cw(h: &Huffman, h_bs: &mut FdkBitStream) -> i32 {
    let mut index: i8 = 0;
    while index >= 0 {
        let bit = h_bs.read_bits(1) as usize;
        index = h[index as usize][bit];
    }
    index as i32 + 64 /* Add offset */
}

/**************/
/* uniDrcGain */
/**************/

fn decode_gain_initial(h_bs: &mut FdkBitStream, gain_coding_profile: i32) -> FixpSgl {
    let mut gain_initial: FixpSgl = 0;
    match gain_coding_profile {
        GCP_REGULAR => {
            let sign = h_bs.read_bits(1);
            let magn = h_bs.read_bits(8) as i32;
            gain_initial = (magn << (FRACT_BITS - 1 - 3 - 7)) as FixpSgl; /* magn * 0.125; */
            if sign != 0 {
                gain_initial = gain_initial.wrapping_neg();
            }
        }
        GCP_FADING => {
            let sign = h_bs.read_bits(1);
            if sign == 0 {
                gain_initial = 0;
            } else {
                let magn = h_bs.read_bits(10) as i32;
                /* - (magn + 1) * 0.125; */
                gain_initial = ((-(magn + 1)) << (FRACT_BITS - 1 - 3 - 7)) as FixpSgl;
            }
        }
        GCP_CLIPPING_DUCKING => {
            let sign = h_bs.read_bits(1);
            if sign == 0 {
                gain_initial = 0;
            } else {
                let magn = h_bs.read_bits(8) as i32;
                /* - (magn + 1) * 0.125; */
                gain_initial = ((-(magn + 1)) << (FRACT_BITS - 1 - 3 - 7)) as FixpSgl;
            }
        }
        _ => {} /* GCP_CONSTANT */
    }
    gain_initial
}

fn decode_n_nodes(h_bs: &mut FdkBitStream) -> i32 {
    let mut n_nodes = 0;
    let mut end_marker = 0;

    /* decode number of nodes */
    while end_marker != 1 {
        n_nodes += 1;
        if n_nodes >= 128 {
            break;
        }
        end_marker = h_bs.read_bits(1);
    }
    n_nodes
}

fn decode_gains(
    h_bs: &mut FdkBitStream,
    gain_coding_profile: i32,
    n_nodes: i32,
    p_nodes: &mut [GainNode; 32],
) {
    p_nodes[0].gain_db = decode_gain_initial(h_bs, gain_coding_profile);

    let delta_gain_codebook: &Huffman = if gain_coding_profile == GCP_CLIPPING_DUCKING {
        &DELTA_GAIN_CODING_PROFILE_2_HUFFMAN
    } else {
        &DELTA_GAIN_CODING_PROFILE_0_1_HUFFMAN
    };

    for k in 1..n_nodes.max(0) as usize {
        let delta_gain = decode_huffman_cw(delta_gain_codebook, h_bs);
        if k >= 32 {
            continue;
        }
        /* gain_dB_e = 7 */
        /* pNodes[k-1].gainDb + 0.125*deltaGain */
        let v = ((p_nodes[k - 1].gain_db >> 1) as i32)
            + ((delta_gain << (FRACT_BITS - 1 - 7 - 3 - 1)) as FixpSgl) as i32;
        p_nodes[k].gain_db = v as FixpSgl;
        p_nodes[k].gain_db =
            saturate_left_shift(p_nodes[k].gain_db as i32, 1, FRACT_BITS) as FixpSgl;
    }
}

fn decode_slopes(h_bs: &mut FdkBitStream, gain_interpolation_type: i32, n_nodes: i32) {
    if gain_interpolation_type == GIT_SPLINE {
        /* decode slope steepness */
        for _ in 0..n_nodes {
            decode_huffman_cw(&SLOPE_STEEPNESS_HUFFMAN, h_bs);
        }
    }
}

fn decode_time_delta(h_bs: &mut FdkBitStream, z: i32) -> i32 {
    let prefix = h_bs.read_bits(2);
    match prefix {
        0x0 => 1,
        0x1 => {
            let mu = h_bs.read_bits(2) as i32;
            mu + 2
        }
        0x2 => {
            let mu = h_bs.read_bits(3) as i32;
            mu + 6
        }
        _ => {
            let mu = h_bs.read_bits(z as u32) as i32;
            mu.wrapping_add(14)
        }
    }
}

fn decode_times(
    h_bs: &mut FdkBitStream,
    delta_tmin: i32,
    frame_size: i32,
    full_frame: i32,
    time_offset: i32,
    z: i32,
    n_nodes: i32,
    p_nodes: &mut [GainNode; 32],
) {
    let mut time_offs = time_offset;

    let frame_end_flag = if full_frame == 0 { h_bs.read_bits(1) as i32 } else { 1 };

    if frame_end_flag == 1 {
        /* frameEndFlag == 1 signals that the last node is at the end of the DRC frame */
        let mut node_res_flag = 0;
        let mut k: i32 = 0;
        while k < n_nodes - 1 {
            /* decode a delta time value */
            let time_delta = decode_time_delta(h_bs, z);
            if k >= (32 - 1) {
                k += 1;
                continue;
            }
            /* frameEndFlag == 1 needs special handling for last node with node reservoir */
            let node_time_tmp = time_offs.wrapping_add(time_delta.wrapping_mul(delta_tmin));
            if node_time_tmp > frame_size.wrapping_add(time_offset) {
                if node_res_flag == 0 {
                    p_nodes[k as usize].time = frame_size.wrapping_add(time_offset) as i16;
                    node_res_flag = 1;
                }
                p_nodes[k as usize + 1].time = node_time_tmp as i16;
            } else {
                p_nodes[k as usize].time = node_time_tmp as i16;
            }
            time_offs = node_time_tmp;
            k += 1;
        }
        if node_res_flag == 0 {
            k = f_min(k, 32 - 1);
            p_nodes[k as usize].time = frame_size.wrapping_add(time_offset) as i16;
        }
    } else {
        for k in 0..n_nodes.max(0) as usize {
            /* decode a delta time value */
            let time_delta = decode_time_delta(h_bs, z);
            if k >= 32 {
                continue;
            }
            p_nodes[k].time = time_offs.wrapping_add(time_delta.wrapping_mul(delta_tmin)) as i16;
            time_offs = p_nodes[k].time as i32;
        }
    }
}

fn read_nodes(
    h_bs: &mut FdkBitStream,
    gain_set: &GainSet,
    frame_size: i32,
    time_delta_min: i32,
    p_n_nodes: &mut u8,
    p_nodes: &mut [GainNode; 32],
) {
    let n_nodes: i32;
    let z = get_z(c_div(frame_size, time_delta_min));
    let time_offset = if gain_set.time_alignment == 0 {
        -1
    } else {
        /* timeOffset = - deltaTmin + floor((deltaTmin-1)/2); */
        -time_delta_min + c_div(time_delta_min - 1, 2)
    };

    let drc_gain_coding_mode = h_bs.read_bits(1);
    if drc_gain_coding_mode == 0 {
        /* "simple" mode: only one node at the end of the frame with slope = 0 */
        n_nodes = 1;
        p_nodes[0].gain_db = decode_gain_initial(h_bs, gain_set.gain_coding_profile as i32);
        p_nodes[0].time = frame_size.wrapping_add(time_offset) as i16;
    } else {
        n_nodes = decode_n_nodes(h_bs);

        decode_slopes(h_bs, gain_set.gain_interpolation_type as i32, n_nodes);

        decode_times(
            h_bs,
            time_delta_min,
            frame_size,
            gain_set.full_frame as i32,
            time_offset,
            z,
            n_nodes,
            p_nodes,
        );

        decode_gains(h_bs, gain_set.gain_coding_profile as i32, n_nodes, p_nodes);
    }
    *p_n_nodes = n_nodes as u8;
}

/// Integer division with the AArch64 semantics (`sdiv` returns 0 for a zero divisor and wraps for
/// `INT_MIN / -1`), where C would be undefined.
#[inline]
pub(crate) fn c_div(a: i32, b: i32) -> i32 {
    if b == 0 {
        0
    } else {
        a.wrapping_div(b)
    }
}

fn read_drc_gain_sequence(
    h_bs: &mut FdkBitStream,
    gain_set: &GainSet,
    frame_size: i32,
    time_delta_min: i32,
    p_n_nodes: &mut u8,
    p_nodes: &mut [GainNode; 32],
) {
    let mut time_buf_prev_frame = [0i16; 32];
    let mut time_buf_cur_frame = [0i16; 32];

    if gain_set.gain_coding_profile as i32 == GCP_CONSTANT {
        *p_n_nodes = 1;
        p_nodes[0].time = (frame_size - 1) as i16;
        p_nodes[0].gain_db = 0;
    } else {
        read_nodes(h_bs, gain_set, frame_size, time_delta_min, p_n_nodes, p_nodes);

        /* count number of nodes in node reservoir */
        let mut n_nodes_node_res = 0usize;
        let mut n_nodes_cur = 0usize;
        /* count and buffer nodes from node reservoir */
        let mut k = 0usize;
        while k < *p_n_nodes as usize {
            if k >= 32 {
                break;
            }
            let t = p_nodes[k].time as i32;
            if t >= frame_size.wrapping_mul(2) {
                break; /* time is out of range */
            }
            if t >= frame_size {
                /* write node reservoir times into buffer */
                time_buf_prev_frame[n_nodes_node_res] = p_nodes[k].time;
                n_nodes_node_res += 1;
            } else {
                /* times from current frame */
                time_buf_cur_frame[n_nodes_cur] = p_nodes[k].time;
                n_nodes_cur += 1;
            }
            k += 1;
        }
        /* restrict nNodes to valid ones if break occured */
        *p_n_nodes = k as u8;

        /* insert dummy node if no node is available */
        if *p_n_nodes == 0 {
            *p_n_nodes = 1;
            p_nodes[0].time = (frame_size - 1) as i16;
            p_nodes[0].gain_db = 0;
        }

        /* compose right time order (bit reservoir first) */
        let mut k = 0usize;
        while k < n_nodes_node_res {
            /* subtract two time frameSize: one to remove node reservoir offset and one to get the
             * negative index relative to the current frame
             */
            p_nodes[k].time =
                (time_buf_prev_frame[k] as i32).wrapping_sub(frame_size.wrapping_mul(2)) as i16;
            k += 1;
        }
        /* ...and times from current frame */
        for m in 0..n_nodes_cur {
            p_nodes[k].time = time_buf_cur_frame[m];
            k += 1;
        }
    }
}

fn read_uni_drc_gain_extension(
    h_bs: &mut FdkBitStream,
    p_ext: &mut UniDrcGainExtension,
) -> DrcError {
    let mut k = 0usize;
    p_ext.uni_drc_gain_ext_type[k] = h_bs.read_bits(4) as u8;
    while p_ext.uni_drc_gain_ext_type[k] != UNIDRCGAINEXT_TERM {
        if k >= (8 - 1) {
            return DE_MEMORY_ERROR;
        }
        let bit_size_len = h_bs.read_bits(3);
        let ext_size_bits = bit_size_len + 4;

        let bit_size = h_bs.read_bits(ext_size_bits);
        p_ext.ext_bit_size[k] = bit_size + 1;

        /* add future extensions here */
        h_bs.push_for(p_ext.ext_bit_size[k]);
        k += 1;
        p_ext.uni_drc_gain_ext_type[k] = h_bs.read_bits(4) as u8;
    }

    DE_OK
}

fn get_uni_drc_gain_status(p_coef: &DrcCoefficientsUniDrc, h_uni_drc_gain: &UniDrcGain) -> u8 {
    /* The status of uniDrcGain is set to 1 if all expected gain sequences have been read.
    If the status stays 0, concealment should be applied. */
    for s in 0..4 {
        if h_uni_drc_gain.n_decoded_sequences[s] != p_coef.stream_gain_sequence_count[s] {
            return 0;
        }
    }
    1
}

/// `drcDec_readUniDrcGain(hBs, pCoef, frameSize, deltaTminDefault, subStreamIndex, hUniDrcGain)`
pub fn drc_dec_read_uni_drc_gain(
    h_bs: &mut FdkBitStream,
    p_coef: Option<&DrcCoefficientsUniDrc>,
    frame_size: i32,
    delta_tmin_default: i32,
    sub_stream_index: i32,
    h_uni_drc_gain: &mut UniDrcGain,
) -> DrcError {
    let mut err = DE_OK;
    let mut start_index_gain_sequence: i32 = 0;
    let mut sequence_skipped = false;
    if sub_stream_index >= 4 {
        return DE_OK;
    }
    let sub = sub_stream_index as usize;

    let Some(p_coef) = p_coef else {
        h_uni_drc_gain.status = 0;
        return DE_OK;
    };
    /* accumulate gainSequenceCount from lower substreams */
    for s in 0..sub {
        start_index_gain_sequence += p_coef.stream_gain_sequence_count[s] as i32;
    }
    let gain_sequence_count = p_coef.stream_gain_sequence_count[sub] as i32;

    for seq in 0..gain_sequence_count {
        let mut tmp_n_nodes: u8 = 0;
        let mut tmp_nodes = [GainNode::ZERO; 32];

        if (start_index_gain_sequence + seq) >= 48 {
            sequence_skipped = true;
            continue;
        }
        let gs = (start_index_gain_sequence + seq) as usize;

        /* insert the gainSequences of this substream, starting from startIndexGainSequence */
        let index = p_coef.gain_set_index_for_gain_sequence[gs];
        if (index >= p_coef.gain_set_count) || (index >= 48) {
            return DE_NOT_OK;
        }
        let gain_set = &p_coef.gain_set[index as usize];

        let time_delta_min = get_time_delta_min(gain_set, delta_tmin_default);

        read_drc_gain_sequence(
            h_bs,
            gain_set,
            frame_size,
            time_delta_min,
            &mut tmp_n_nodes,
            &mut tmp_nodes,
        );

        h_uni_drc_gain.n_nodes[gs] = tmp_n_nodes;
        let n = f_min(tmp_n_nodes, 32u8) as usize;
        h_uni_drc_gain.gain_node[gs][..n].copy_from_slice(&tmp_nodes[..n]);
    }

    if !sequence_skipped {
        /* all sequences have been read */
        h_uni_drc_gain.uni_drc_gain_ext_present = h_bs.read_bits(1) as u8;
        if h_uni_drc_gain.uni_drc_gain_ext_present == 1 {
            if sub_stream_index != 0 {
                return DE_NOT_OK; /* uniDrcGainExtension is not allowed for side-stream */
            }

            err = read_uni_drc_gain_extension(h_bs, &mut h_uni_drc_gain.uni_drc_gain_extension);
            if err != 0 {
                return err;
            }
        }
    }

    if err == DE_OK {
        /* keep number of read (or skipped) gain sequences for this substream. */
        h_uni_drc_gain.n_decoded_sequences[sub] = gain_sequence_count as u8;
    }

    h_uni_drc_gain.status = get_uni_drc_gain_status(p_coef, h_uni_drc_gain);

    err
}

/****************/
/* uniDrcConfig */
/****************/

fn decode_ducking_modification(h_bs: &mut FdkBitStream) -> DuckingModification {
    let ducking_scaling_present = h_bs.read_bits(1);

    let ducking_scaling = if ducking_scaling_present != 0 {
        let bs_ducking_scaling = h_bs.read_bits(4) as i32;
        let sigma = bs_ducking_scaling >> 3;
        let mu = bs_ducking_scaling & 0x7;

        if sigma != 0 {
            ((7 - mu) << (FRACT_BITS - 1 - 3 - 2)) as FixpSgl /* 1.0 - 0.125 * (1 + mu); */
        } else {
            ((9 + mu) << (FRACT_BITS - 1 - 3 - 2)) as FixpSgl /* 1.0 + 0.125 * (1 + mu); */
        }
    } else {
        (1 << (FRACT_BITS - 1 - 2)) as FixpSgl /* 1.0 */
    };
    DuckingModification { ducking_scaling }
}

fn decode_gain_modification(
    h_bs: &mut FdkBitStream,
    _version: i32,
    _band_count: i32,
    p_inst: &mut DrcInstructionsUniDrc,
    g: usize,
) {
    let mut attenuation_scaling: FixpSgl = fl2fxconst_sgl((1.0f32 / 4.0f32) as f64);
    let mut amplification_scaling: FixpSgl = fl2fxconst_sgl((1.0f32 / 4.0f32) as f64);
    let mut gain_offset: FixpSgl = 0;
    let gain_scaling_present = h_bs.read_bits(1);
    if gain_scaling_present != 0 {
        let bs_attenuation_scaling = h_bs.read_bits(4) as i32;
        /* bsAttenuationScaling * 0.125; */
        attenuation_scaling = (bs_attenuation_scaling << (FRACT_BITS - 1 - 3 - 2)) as FixpSgl;
        let bs_amplification_scaling = h_bs.read_bits(4) as i32;
        /* bsAmplificationScaling * 0.125; */
        amplification_scaling = (bs_amplification_scaling << (FRACT_BITS - 1 - 3 - 2)) as FixpSgl;
    }
    let gain_offset_present = h_bs.read_bits(1);
    if gain_offset_present != 0 {
        let sign = h_bs.read_bits(1);
        let bs_gain_offset = h_bs.read_bits(5) as i32;
        /* (1+bsGainOffset) * 0.25; */
        gain_offset = ((1 + bs_gain_offset) << (FRACT_BITS - 1 - 2 - 4)) as FixpSgl;
        if sign != 0 {
            gain_offset = -gain_offset;
        }
    }
    for b in 0..4 {
        p_inst.set_gain_modification(
            g,
            b,
            GainModification { attenuation_scaling, amplification_scaling, gain_offset },
        );
    }
}

fn read_drc_characteristic(
    h_bs: &mut FdkBitStream,
    version: i32,
    p_d_char: &mut DrcCharacteristic,
) {
    if version == 0 {
        p_d_char.cicp_index = h_bs.read_bits(7) as u8;
        if p_d_char.cicp_index > 0 {
            p_d_char.present = 1;
            p_d_char.is_cicp = 1;
        } else {
            p_d_char.present = 0;
        }
    }
}

fn read_band_border(h_bs: &mut FdkBitStream, p_b_bord: &mut BandBorder, drc_band_type: i32) {
    if drc_band_type != 0 {
        p_b_bord.set_crossover_freq_index(h_bs.read_bits(4) as u8);
    } else {
        p_b_bord.start_sub_band_index = h_bs.read_bits(10) as u16;
    }
}

fn read_gain_set(
    h_bs: &mut FdkBitStream,
    version: i32,
    gain_sequence_index: &mut i32,
    p_g_set: &mut GainSet,
) -> DrcError {
    p_g_set.gain_coding_profile = h_bs.read_bits(2) as u8;
    p_g_set.gain_interpolation_type = h_bs.read_bits(1) as u8;
    p_g_set.full_frame = h_bs.read_bits(1) as u8;
    p_g_set.time_alignment = h_bs.read_bits(1) as u8;
    p_g_set.time_delta_min_present = h_bs.read_bits(1) as u8;

    if p_g_set.time_delta_min_present != 0 {
        let bs_time_delta_min = h_bs.read_bits(11);
        p_g_set.time_delta_min = (bs_time_delta_min + 1) as u16;
    }

    if p_g_set.gain_coding_profile as i32 != GCP_CONSTANT {
        p_g_set.band_count = h_bs.read_bits(4) as u8;
        if p_g_set.band_count > 4 {
            return DE_MEMORY_ERROR;
        }

        if p_g_set.band_count > 1 {
            p_g_set.drc_band_type = h_bs.read_bits(1) as u8;
        }

        for i in 0..p_g_set.band_count as usize {
            if version == 0 {
                *gain_sequence_index += 1;
            }
            p_g_set.gain_sequence_index[i] = *gain_sequence_index as u8;
            read_drc_characteristic(h_bs, version, &mut p_g_set.drc_characteristic[i]);
        }
        for i in 1..p_g_set.band_count as usize {
            let drc_band_type = p_g_set.drc_band_type as i32;
            read_band_border(h_bs, &mut p_g_set.band_border[i], drc_band_type);
        }
    } else {
        p_g_set.band_count = 1;
        *gain_sequence_index += 1;
        p_g_set.gain_sequence_index[0] = *gain_sequence_index as u8;
    }

    DE_OK
}

fn derive_gain_set_index_for_gain_sequence(p_coef: &mut DrcCoefficientsUniDrc) {
    for gs in 0..48 {
        p_coef.gain_set_index_for_gain_sequence[gs] = 255;
    }
    for gs in 0..p_coef.gain_set_count as usize {
        for b in 0..p_coef.gain_set[gs].band_count as usize {
            let seq = p_coef.gain_set[gs].gain_sequence_index[b];
            if seq >= 48 {
                continue;
            }
            p_coef.gain_set_index_for_gain_sequence[seq as usize] = gs as u8;
        }
    }
}

fn read_drc_coefficients_uni_drc(
    h_bs: &mut FdkBitStream,
    version: i32,
    p_coef: &mut DrcCoefficientsUniDrc,
) -> DrcError {
    let err = DE_OK;
    let mut gain_sequence_index: i32 = -1;

    p_coef.drc_location = h_bs.read_bits(4) as u8;
    p_coef.drc_frame_size_present = h_bs.read_bits(1) as u8;

    if p_coef.drc_frame_size_present == 1 {
        let bs_drc_frame_size = h_bs.read_bits(15);
        p_coef.drc_frame_size = (bs_drc_frame_size + 1) as u16;
    }
    if version == 0 {
        let mut gain_sequence_count: i32 = 0;
        let gain_set_count = h_bs.read_bits(6) as i32;
        p_coef.gain_set_count = f_min(gain_set_count, 48) as u8;
        for i in 0..gain_set_count as usize {
            let mut tmp_gset = GainSet::ZERO;
            let err = read_gain_set(h_bs, version, &mut gain_sequence_index, &mut tmp_gset);
            if err != 0 {
                return err;
            }
            gain_sequence_count += tmp_gset.band_count as i32;

            if i >= 48 {
                continue;
            }
            p_coef.gain_set[i] = tmp_gset;
        }
        p_coef.gain_sequence_count = gain_sequence_count as u8;
    }

    derive_gain_set_index_for_gain_sequence(p_coef);

    err
}

fn merge_substream_drc_coefficients(
    p_coef_substream: &mut DrcCoefficientsUniDrc,
    p_coef: &mut DrcCoefficientsUniDrc,
    sub_stream_index: usize,
    p_diff: &mut i32,
) -> DrcError {
    let mut tmp_diff = 0;
    tmp_diff |= comp_assign(&mut p_coef.drc_location, p_coef_substream.drc_location);
    tmp_diff |=
        comp_assign(&mut p_coef.drc_frame_size_present, p_coef_substream.drc_frame_size_present);
    tmp_diff |= comp_assign(&mut p_coef.drc_frame_size, p_coef_substream.drc_frame_size);

    if sub_stream_index == 0 {
        *p_diff |= tmp_diff;
    } else {
        /* check if side stream fits to main stream */
        if tmp_diff != 0 {
            return DE_NOT_OK;
        }
    }

    /* merge together substream element to full element */
    *p_diff |= comp_assign(
        &mut p_coef.stream_gain_set_count[sub_stream_index],
        p_coef_substream.gain_set_count,
    );
    *p_diff |= comp_assign(
        &mut p_coef.stream_gain_sequence_count[sub_stream_index],
        p_coef_substream.gain_sequence_count,
    );

    /* accumulate gainSetCount and gainSequenceCount */
    p_coef.gain_set_count = 0;
    p_coef.gain_sequence_count = 0;
    for s in 0..4 {
        p_coef.gain_set_count = p_coef.gain_set_count.wrapping_add(p_coef.stream_gain_set_count[s]);
        p_coef.gain_sequence_count =
            p_coef.gain_sequence_count.wrapping_add(p_coef.stream_gain_sequence_count[s]);
    }
    p_coef.gain_set_count = f_min(p_coef.gain_set_count as i32, 48) as u8;

    /* accumulate gainSetCount and gainSequenceCount from lower substreams */
    let mut start_index_gain_set = 0usize;
    let mut start_index_gain_sequence = 0i32;
    for s in 0..sub_stream_index {
        start_index_gain_set += p_coef.stream_gain_set_count[s] as usize;
        start_index_gain_sequence += p_coef.stream_gain_sequence_count[s] as i32;
    }

    /* insert the gainSet elements of this substream, starting from startIndexGainSet */
    for gs in 0..p_coef.stream_gain_set_count[sub_stream_index] as usize {
        if (start_index_gain_set + gs) >= 48 {
            continue;
        }
        /* adapt gainSequenceIndex */
        for b in 0..p_coef_substream.gain_set[gs].band_count as usize {
            let v = &mut p_coef_substream.gain_set[gs].gain_sequence_index[b];
            *v = (*v as i32).wrapping_add(start_index_gain_sequence) as u8;
        }
        if *p_diff == 0 {
            *p_diff |= (p_coef_substream.gain_set[gs] != p_coef.gain_set[start_index_gain_set + gs])
                as i32;
        }
        p_coef.gain_set[start_index_gain_set + gs] = p_coef_substream.gain_set[gs];
    }

    derive_gain_set_index_for_gain_sequence(p_coef);

    DE_OK
}

fn read_drc_instructions_uni_drc(
    h_bs: &mut FdkBitStream,
    version: i32,
    h_uni_drc_config: &UniDrcConfig,
    base_channel_count: u8,
    p_inst: &mut DrcInstructionsUniDrc,
) -> DrcError {
    let err;
    let mut channel_count: i32;
    let mut channel_group_for_channel = [0i8; 2 * 28];
    let mut ducking_modification_for_channel_group = [DuckingModification::ZERO; 28];

    p_inst.drc_set_id = h_bs.read_bits(6) as i8;
    if version == 0 {
        /* Assume all v0 DRC sets to be manageable in terms of complexity */
        p_inst.drc_set_complexity_level = 2;
    } else {
        p_inst.drc_set_complexity_level = h_bs.read_bits(4) as u8;
    }
    p_inst.drc_location = h_bs.read_bits(4) as u8;
    let downmix_id_present = if version == 0 { 1 } else { h_bs.read_bits(1) };
    if downmix_id_present != 0 {
        p_inst.downmix_id[0] = h_bs.read_bits(7) as u8;
        if version == 0 {
            if p_inst.downmix_id[0] == 0 {
                p_inst.drc_apply_to_downmix = 0;
            } else {
                p_inst.drc_apply_to_downmix = 1;
            }
        } else {
            p_inst.drc_apply_to_downmix = h_bs.read_bits(1) as u8;
        }

        let additional_downmix_id_present = h_bs.read_bits(1);
        if additional_downmix_id_present != 0 {
            let additional_downmix_id_count = h_bs.read_bits(3) as usize;
            if (1 + additional_downmix_id_count) > 8 {
                return DE_MEMORY_ERROR;
            }
            for i in 0..additional_downmix_id_count {
                p_inst.downmix_id[i + 1] = h_bs.read_bits(7) as u8;
            }
            p_inst.downmix_id_count = (1 + additional_downmix_id_count) as u8;
        } else {
            p_inst.downmix_id_count = 1;
        }
    } else {
        p_inst.downmix_id[0] = 0;
        p_inst.downmix_id_count = 1;
    }

    p_inst.drc_set_effect = h_bs.read_bits(16) as u16;

    if (p_inst.drc_set_effect as i32 & (EB_DUCK_OTHER | EB_DUCK_SELF)) == 0 {
        p_inst.limiter_peak_target_present = h_bs.read_bits(1) as u8;
        if p_inst.limiter_peak_target_present != 0 {
            let bs_limiter_peak_target = h_bs.read_bits(8) as i32;
            /* - bsLimiterPeakTarget * 0.125; */
            p_inst.limiter_peak_target = (-(((bs_limiter_peak_target << (FRACT_BITS - 1 - 3 - 5))
                as FixpSgl) as i32)) as FixpSgl;
        }
    }

    p_inst.drc_set_target_loudness_present = h_bs.read_bits(1) as u8;

    /* set default values */
    p_inst.drc_set_target_loudness_value_upper = 0;
    p_inst.drc_set_target_loudness_value_lower = -63;

    if p_inst.drc_set_target_loudness_present == 1 {
        let bs_drc_set_target_loudness_value_upper = h_bs.read_bits(6) as i32;
        p_inst.drc_set_target_loudness_value_upper =
            (bs_drc_set_target_loudness_value_upper - 63) as i8;
        let drc_set_target_loudness_value_lower_present = h_bs.read_bits(1);
        if drc_set_target_loudness_value_lower_present == 1 {
            let bs_drc_set_target_loudness_value_lower = h_bs.read_bits(6) as i32;
            p_inst.drc_set_target_loudness_value_lower =
                (bs_drc_set_target_loudness_value_lower - 63) as i8;
        }
    }

    p_inst.depends_on_drc_set_present = h_bs.read_bits(1) as u8;

    p_inst.depends_on_drc_set = 0; /* noIndependentUse = 0 */
    if p_inst.depends_on_drc_set_present != 0 {
        p_inst.depends_on_drc_set = h_bs.read_bits(6) as i8;
    } else {
        p_inst.depends_on_drc_set = h_bs.read_bits(1) as i8; /* noIndependentUse */
    }

    if version == 0 {
        p_inst.requires_eq = 0;
    } else {
        p_inst.requires_eq = h_bs.read_bits(1) as u8;
    }

    let p_coef = select_drc_coefficients(h_uni_drc_config, p_inst.drc_location as i32)
        .map(|c| &h_uni_drc_config.drc_coefficients_uni_drc[c]);

    channel_count = base_channel_count as i32;
    p_inst.drc_channel_count = base_channel_count;

    if p_inst.drc_set_effect as i32 & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0 {
        let mut c: usize = 0;
        while (c as i32) < channel_count {
            let bs_gain_set_index = h_bs.read_bits(6) as i32;
            if c >= 2 * 28 {
                return DE_MEMORY_ERROR;
            }
            p_inst.gain_set_index[c] = (bs_gain_set_index - 1) as i8;
            let d = decode_ducking_modification(h_bs);
            p_inst.set_ducking_modification(c, d);

            c += 1;
            let repeat_parameters = h_bs.read_bits(1);
            if repeat_parameters == 1 {
                let mut bs_repeat_parameters_count = h_bs.read_bits(5) as i32;
                bs_repeat_parameters_count += 1;
                for _ in 0..bs_repeat_parameters_count {
                    if c >= 2 * 28 {
                        return DE_MEMORY_ERROR;
                    }
                    p_inst.gain_set_index[c] = p_inst.gain_set_index[c - 1];
                    let d = p_inst.ducking_modification(c - 1);
                    p_inst.set_ducking_modification(c, d);
                    c += 1;
                }
            }
        }
        if c as i32 > channel_count {
            return DE_NOT_OK;
        }

        let d_mod_for_channel = p_inst.ducking_modifications();
        let gain_set_index = p_inst.gain_set_index;
        err = derive_drc_channel_groups(
            p_inst.drc_set_effect as i32,
            p_inst.drc_channel_count as i32,
            &gain_set_index,
            Some(&d_mod_for_channel),
            &mut p_inst.n_drc_channel_groups,
            &mut p_inst.gain_set_index_for_channel_group,
            &mut channel_group_for_channel,
            Some(&mut ducking_modification_for_channel_group),
        );
        if err != 0 {
            return err;
        }
    } else {
        let mut derive_channel_count = false;
        if ((version == 0) || (p_inst.drc_apply_to_downmix != 0))
            && (p_inst.downmix_id[0] as i32 != DOWNMIX_ID_BASE_LAYOUT)
            && (p_inst.downmix_id[0] as i32 != DOWNMIX_ID_ANY_DOWNMIX)
            && (p_inst.downmix_id_count == 1)
        {
            if h_uni_drc_config.downmix_instructions_count != 0 {
                let Some(d) =
                    select_downmix_instructions(h_uni_drc_config, p_inst.downmix_id[0] as i32)
                else {
                    return DE_NOT_OK;
                };
                /* targetChannelCountFromDownmixId*/
                p_inst.drc_channel_count =
                    h_uni_drc_config.downmix_instructions[d].target_channel_count;
                channel_count = p_inst.drc_channel_count as i32;
            } else {
                derive_channel_count = true;
                channel_count = 1;
            }
        } else if ((version == 0) || (p_inst.drc_apply_to_downmix != 0))
            && ((p_inst.downmix_id[0] as i32 == DOWNMIX_ID_ANY_DOWNMIX)
                || (p_inst.downmix_id_count > 1))
        {
            /* Set maximum channel count as upper border. The effective channel count is set at the
             * process function. */
            p_inst.drc_channel_count = 2 * 28;
            channel_count = 1;
        }

        let mut c: usize = 0;
        while (c as i32) < channel_count {
            let bs_gain_set_index = h_bs.read_bits(6) as i32;
            if c >= 2 * 28 {
                return DE_MEMORY_ERROR;
            }
            p_inst.gain_set_index[c] = (bs_gain_set_index - 1) as i8;
            c += 1;
            let repeat_sequence_index = h_bs.read_bits(1);

            if repeat_sequence_index == 1 {
                let mut bs_repeat_sequence_count = h_bs.read_bits(5) as i32;
                bs_repeat_sequence_count += 1;
                if derive_channel_count {
                    channel_count = 1 + bs_repeat_sequence_count;
                }
                for _ in 0..bs_repeat_sequence_count {
                    if c >= 2 * 28 {
                        return DE_MEMORY_ERROR;
                    }
                    p_inst.gain_set_index[c] = (bs_gain_set_index - 1) as i8;
                    c += 1;
                }
            }
        }
        if c as i32 > channel_count {
            return DE_NOT_OK;
        }
        if derive_channel_count {
            p_inst.drc_channel_count = channel_count as u8;
        }

        /* DOWNMIX_ID_ANY_DOWNMIX: channelCount is 1. Distribute gainSetIndex to all channels. */
        if (p_inst.downmix_id[0] as i32 == DOWNMIX_ID_ANY_DOWNMIX) || (p_inst.downmix_id_count > 1)
        {
            for c in 1..p_inst.drc_channel_count as usize {
                p_inst.gain_set_index[c] = p_inst.gain_set_index[0];
            }
        }

        let gain_set_index = p_inst.gain_set_index;
        err = derive_drc_channel_groups(
            p_inst.drc_set_effect as i32,
            p_inst.drc_channel_count as i32,
            &gain_set_index,
            None,
            &mut p_inst.n_drc_channel_groups,
            &mut p_inst.gain_set_index_for_channel_group,
            &mut channel_group_for_channel,
            None,
        );
        if err != 0 {
            return err;
        }

        for g in 0..p_inst.n_drc_channel_groups as usize {
            let set = p_inst.gain_set_index_for_channel_group[g] as i32;

            /* get bandCount */
            let band_count = match p_coef {
                Some(p_coef) if set < p_coef.gain_set_count as i32 => {
                    gain_set_at(p_coef, set).band_count as i32
                }
                _ => 1,
            };

            decode_gain_modification(h_bs, version, band_count, p_inst, g);
        }
    }

    err
}

/// `&pCoef->gainSet[idx]`. For indices outside the array (possible with corrupt configurations
/// where `SCHAR` gain set indices wrap around), the C code reads out of bounds (undefined
/// behaviour); here a zeroed gain set is returned instead.
pub(crate) fn gain_set_at(p_coef: &DrcCoefficientsUniDrc, idx: i32) -> &GainSet {
    static ZERO: GainSet = GainSet::ZERO;
    if (0..48).contains(&idx) {
        &p_coef.gain_set[idx as usize]
    } else {
        mpeghdec_fdk::deviation::hit();
        &ZERO
    }
}

fn merge_substream_drc_instructions(
    p_inst_substream: &DrcInstructionsUniDrc,
    p_inst: &mut DrcInstructionsUniDrc,
    stream_channel_count: [u8; 4],
    base_channel_count: u8,
    coef_stream_gain_set_count: Option<[u8; 4]>,
    sub_stream_index: usize,
    p_diff: &mut i32,
) -> DrcError {
    let mut tmp_diff = 0;

    tmp_diff |=
        comp_assign(&mut p_inst.drc_instructions_type, p_inst_substream.drc_instructions_type);
    if p_inst.drc_instructions_type != 0 {
        if p_inst.drc_instructions_type == 2 || p_inst.drc_instructions_type == 3 {
            tmp_diff |= comp_assign(&mut p_inst.mae_group_id, p_inst_substream.mae_group_id);
        }
    }
    tmp_diff |= comp_assign(&mut p_inst.drc_set_id, p_inst_substream.drc_set_id);
    tmp_diff |= comp_assign(
        &mut p_inst.drc_set_complexity_level,
        p_inst_substream.drc_set_complexity_level,
    );

    tmp_diff |= comp_assign(&mut p_inst.drc_location, p_inst_substream.drc_location);
    tmp_diff |=
        comp_assign(&mut p_inst.drc_apply_to_downmix, p_inst_substream.drc_apply_to_downmix);
    tmp_diff |= comp_assign(&mut p_inst.downmix_id_count, p_inst_substream.downmix_id_count);
    for d in 0..p_inst.downmix_id_count as usize {
        tmp_diff |= comp_assign(&mut p_inst.downmix_id[d], p_inst_substream.downmix_id[d]);
    }
    tmp_diff |= comp_assign(&mut p_inst.drc_set_effect, p_inst_substream.drc_set_effect);
    tmp_diff |= comp_assign(
        &mut p_inst.limiter_peak_target_present,
        p_inst_substream.limiter_peak_target_present,
    );
    if p_inst.limiter_peak_target_present != 0 {
        tmp_diff |=
            comp_assign(&mut p_inst.limiter_peak_target, p_inst_substream.limiter_peak_target);
    }
    tmp_diff |= comp_assign(
        &mut p_inst.drc_set_target_loudness_present,
        p_inst_substream.drc_set_target_loudness_present,
    );
    if p_inst.drc_set_target_loudness_present != 0 {
        tmp_diff |= comp_assign(
            &mut p_inst.drc_set_target_loudness_value_upper,
            p_inst_substream.drc_set_target_loudness_value_upper,
        );
        tmp_diff |= comp_assign(
            &mut p_inst.drc_set_target_loudness_value_lower,
            p_inst_substream.drc_set_target_loudness_value_lower,
        );
    }
    tmp_diff |= comp_assign(
        &mut p_inst.depends_on_drc_set_present,
        p_inst_substream.depends_on_drc_set_present,
    );
    /* dependsOnDrcSet or noIndependentUse (union) */
    tmp_diff |= comp_assign(&mut p_inst.depends_on_drc_set, p_inst_substream.depends_on_drc_set);

    if sub_stream_index == 0 {
        *p_diff |= tmp_diff;
    } else {
        /* check if side stream fits to main stream */
        if tmp_diff != 0 {
            return DE_NOT_OK;
        }
        /* side streams are valid only for DRC-1. */
        if (p_inst.downmix_id_count != 1) && (p_inst.downmix_id[0] as i32 != DOWNMIX_ID_BASE_LAYOUT)
        {
            return DE_NOT_OK;
        }
        if p_inst_substream.drc_channel_count != stream_channel_count[sub_stream_index] {
            return DE_NOT_OK;
        }
    }

    if (p_inst.downmix_id_count == 1) && (p_inst.downmix_id[0] as i32 == DOWNMIX_ID_BASE_LAYOUT) {
        /* DRC-1: accumulated baseChannelCount of all streams */
        p_inst.drc_channel_count = base_channel_count;
    } else {
        /* DRC-2/3: use existing drcChannelCount*/
        p_inst.drc_channel_count = p_inst_substream.drc_channel_count;
    }

    *p_diff |= comp_assign(
        &mut p_inst.stream_n_drc_channel_groups[sub_stream_index],
        p_inst_substream.n_drc_channel_groups,
    );

    /* accumulate nDrcChannelGroups */
    p_inst.n_drc_channel_groups = 0;
    for s in 0..4 {
        p_inst.n_drc_channel_groups =
            p_inst.n_drc_channel_groups.wrapping_add(p_inst.stream_n_drc_channel_groups[s]);
    }
    if p_inst.n_drc_channel_groups > 28 {
        return DE_NOT_OK;
    }

    /* accumulate streamChannelCount, nDrcChannelGroups and gainSetCount from lower substreams */
    let mut start_index_channel = 0usize;
    let mut start_index_channel_group = 0usize;
    let mut start_index_gain_set = 0i32;
    for s in 0..sub_stream_index {
        start_index_channel += stream_channel_count[s] as usize;
        start_index_channel_group += p_inst.stream_n_drc_channel_groups[s] as usize;
        if let Some(cnt) = coef_stream_gain_set_count {
            start_index_gain_set += cnt[s] as i32;
        }
    }

    if p_inst.drc_set_effect as i32 & (EB_DUCK_OTHER | EB_DUCK_SELF) != 0 {
        for c in 0..p_inst_substream.drc_channel_count as usize {
            if (start_index_channel + c) >= 2 * 28 {
                continue;
            }

            /* insert the duckingModificationForChannel elements of this substream, starting from
             * startIndexChannel */
            let d = p_inst_substream.ducking_modification(c);
            if *p_diff == 0 {
                *p_diff |= (d != p_inst.ducking_modification(start_index_channel + c)) as i32;
            }
            p_inst.set_ducking_modification(start_index_channel + c, d);
        }
    } else {
        for cg in 0..p_inst.stream_n_drc_channel_groups[sub_stream_index] as usize {
            if (start_index_channel_group + cg) >= 28 {
                continue;
            }

            /* insert the gainModificationForChannelGroup elements of this substream, starting from
             * startIndexChannelGroup */
            for b in 0..4 {
                if *p_diff == 0 {
                    *p_diff |= (p_inst_substream.gain_modification_group(cg)
                        != p_inst.gain_modification_group(start_index_channel_group + cg))
                        as i32;
                }
                let m = p_inst_substream.gain_modification(cg, b);
                p_inst.set_gain_modification(start_index_channel_group + cg, b, m);
            }
        }
    }

    /* insert the gainSetIndex elements of this substream, starting from startIndexChannel */
    for c in 0..p_inst_substream.drc_channel_count as usize {
        let mut gsi = p_inst_substream.gain_set_index[c] as i32;
        if (start_index_channel + c) >= 2 * 28 {
            continue;
        }
        if gsi >= 0 {
            /* keep gainSetIndex -1 untouched */
            gsi += start_index_gain_set;
        }
        *p_diff |= comp_assign(&mut p_inst.gain_set_index[start_index_channel + c], gsi as i8);
    }

    /* insert the gainSetIndexForChannelGroup elements of this substream, starting from
     * startIndexChannelGroup */
    for cg in 0..p_inst.stream_n_drc_channel_groups[sub_stream_index] as usize {
        if (start_index_channel_group + cg) >= 28 {
            continue;
        }
        *p_diff |= comp_assign(
            &mut p_inst.gain_set_index_for_channel_group[start_index_channel_group + cg],
            (p_inst_substream.gain_set_index_for_channel_group[cg] as i32 + start_index_gain_set)
                as i8,
        );
    }

    DE_OK
}

fn read_mpegh3da_uni_drc_channel_layout(h_bs: &mut FdkBitStream, p_chan: &mut ChannelLayout) {
    p_chan.base_channel_count = h_bs.read_bits(7) as u8;
    p_chan.layout_signaling_present = 0;
}

fn read_uni_drc_config_extension(
    h_bs: &mut FdkBitStream,
    h_uni_drc_config: &mut UniDrcConfig,
) -> DrcError {
    let p_ext = &mut h_uni_drc_config.uni_drc_config_ext;

    let mut k = 0usize;
    p_ext.uni_drc_config_ext_type[k] = h_bs.read_bits(4) as u8;
    while p_ext.uni_drc_config_ext_type[k] != UNIDRCCONFEXT_TERM {
        if k >= (8 - 1) {
            return DE_MEMORY_ERROR;
        }
        let bit_size_len = h_bs.read_bits(4);
        let ext_size_bits = bit_size_len + 4;

        let bit_size = h_bs.read_bits(ext_size_bits);
        p_ext.ext_bit_size[k] = bit_size + 1;

        /* add future extensions here */
        h_bs.push_for(p_ext.ext_bit_size[k]);
        k += 1;
        p_ext.uni_drc_config_ext_type[k] = h_bs.read_bits(4) as u8;
    }

    DE_OK
}

/// `drcDec_readMpegh3daUniDrcConfig(hBs, hUniDrcConfig, hLoudnessInfoSet, subStreamIndex)`
pub fn drc_dec_read_mpegh3da_uni_drc_config(
    h_bs: &mut FdkBitStream,
    h_uni_drc_config: &mut UniDrcConfig,
    h_loudness_info_set: &mut LoudnessInfoSet,
    sub_stream_index: i32,
) -> DrcError {
    let mut err = DE_OK;
    let mut diff = 0;
    let mut tmp_chan = ChannelLayout::default();
    if sub_stream_index >= 4 {
        return DE_OK;
    }
    let sub = sub_stream_index as usize;

    diff |= comp_assign(
        &mut h_uni_drc_config.drc_coefficients_uni_drc_count_v0[sub],
        h_bs.read_bits(3) as u8,
    );
    diff |= comp_assign(
        &mut h_uni_drc_config.drc_instructions_uni_drc_count_v0[sub],
        h_bs.read_bits(6) as u8,
    );

    if sub > 0 {
        /* side streams may not have more elements than the main stream */
        if h_uni_drc_config.drc_coefficients_uni_drc_count_v0[sub]
            > h_uni_drc_config.drc_coefficients_uni_drc_count_v0[0]
        {
            return DE_NOT_OK;
        }
        if h_uni_drc_config.drc_instructions_uni_drc_count_v0[sub]
            > h_uni_drc_config.drc_instructions_uni_drc_count_v0[0]
        {
            return DE_NOT_OK;
        }
    }

    read_mpegh3da_uni_drc_channel_layout(h_bs, &mut tmp_chan);

    diff |=
        comp_assign(&mut h_uni_drc_config.stream_channel_count[sub], tmp_chan.base_channel_count);

    /* accumulate streamChannelCount to get baseChannelCount */
    tmp_chan.base_channel_count = 0;
    for i in 0..4 {
        tmp_chan.base_channel_count =
            tmp_chan.base_channel_count.wrapping_add(h_uni_drc_config.stream_channel_count[i]);
    }

    if diff == 0 {
        diff |= (tmp_chan != h_uni_drc_config.channel_layout) as i32;
    }
    h_uni_drc_config.channel_layout = tmp_chan;

    h_uni_drc_config.drc_coefficients_uni_drc_count =
        f_min(h_uni_drc_config.drc_coefficients_uni_drc_count_v0[0], 2u8);
    for i in 0..h_uni_drc_config.drc_coefficients_uni_drc_count_v0[sub] as usize {
        let mut tmp_coef = Box::new(DrcCoefficientsUniDrc::ZERO);

        err = read_drc_coefficients_uni_drc(h_bs, 0, &mut tmp_coef);
        if err != 0 {
            return err;
        }

        let j = if sub == 0 {
            /* main stream defines the order of drcCoefficients */
            i
        } else {
            /* for side stream, find existing drcCoefficients with the same drcLocation */
            let mut j = 0usize;
            while j < h_uni_drc_config.drc_coefficients_uni_drc_count as usize {
                if h_uni_drc_config.drc_instructions_uni_drc[j].drc_location
                    == tmp_coef.drc_location
                {
                    break;
                }
                j += 1;
            }
            j
        };

        if j >= 2 {
            continue;
        }

        err = merge_substream_drc_coefficients(
            &mut tmp_coef,
            &mut h_uni_drc_config.drc_coefficients_uni_drc[j],
            sub,
            &mut diff,
        );
        if err != 0 {
            return err;
        }
    }

    h_uni_drc_config.drc_instructions_uni_drc_count =
        f_min(h_uni_drc_config.drc_instructions_uni_drc_count_v0[0], 32u8);
    for i in 0..h_uni_drc_config.drc_instructions_uni_drc_count_v0[sub] as usize {
        let mut tmp_inst = Box::new(DrcInstructionsUniDrc::ZERO);

        tmp_inst.drc_instructions_type = h_bs.read_bits(1) as u8;
        if tmp_inst.drc_instructions_type != 0 {
            tmp_inst.drc_instructions_type = (h_bs.read_bits(1) + 2) as u8;
            if tmp_inst.drc_instructions_type == 2 {
                tmp_inst.mae_group_id = h_bs.read_bits(7) as u8;
            } else if tmp_inst.drc_instructions_type == 3 {
                tmp_inst.mae_group_id = h_bs.read_bits(5) as u8; /* mae_groupPresetID */
            }
        }
        let base_channel_count = h_uni_drc_config.stream_channel_count[sub];
        err = read_drc_instructions_uni_drc(
            h_bs,
            0,
            h_uni_drc_config,
            base_channel_count,
            &mut tmp_inst,
        );
        if err != 0 {
            return err;
        }

        let j = if sub == 0 {
            /* main stream defines the order of drcInstructions */
            i
        } else {
            /* for side stream, find existing drcInstructions with the same drcSetId */
            let mut j = 0usize;
            while j < h_uni_drc_config.drc_instructions_uni_drc_count as usize {
                if h_uni_drc_config.drc_instructions_uni_drc[j].drc_set_id == tmp_inst.drc_set_id {
                    break;
                }
                j += 1;
            }
            j
        };

        if j >= 32 {
            continue;
        }

        let coef_stream_gain_set_count = select_drc_coefficients(
            h_uni_drc_config,
            h_uni_drc_config.drc_instructions_uni_drc[j].drc_location as i32,
        )
        .map(|c| h_uni_drc_config.drc_coefficients_uni_drc[c].stream_gain_set_count);
        let stream_channel_count = h_uni_drc_config.stream_channel_count;
        let base_channel_count = h_uni_drc_config.channel_layout.base_channel_count;
        err = merge_substream_drc_instructions(
            &tmp_inst,
            &mut h_uni_drc_config.drc_instructions_uni_drc[j],
            stream_channel_count,
            base_channel_count,
            coef_stream_gain_set_count,
            sub,
            &mut diff,
        );
        if err != 0 {
            return err;
        }
    }

    let uni_drc_config_ext_present = h_bs.read_bits(1) as u8;
    if sub == 0 {
        diff |= comp_assign(
            &mut h_uni_drc_config.uni_drc_config_ext_present,
            uni_drc_config_ext_present,
        );
    } else {
        /* uniDrcConfigExtension is not allowed for side-stream */
        if uni_drc_config_ext_present != 0 {
            return DE_NOT_OK;
        }
    }

    h_uni_drc_config.diff |= diff as u8;

    if uni_drc_config_ext_present != 0 {
        err = read_uni_drc_config_extension(h_bs, h_uni_drc_config);
        if err != 0 {
            return err;
        }
    }

    h_uni_drc_config.loudness_info_set_present = h_bs.read_bits(1) as u8;
    if h_uni_drc_config.loudness_info_set_present == 1 {
        err = drc_dec_read_mpegh3da_loudness_info_set(h_bs, h_loudness_info_set, sub_stream_index);
        if err != 0 {
            return err;
        }
    }

    err
}

/*******************/
/* loudnessInfoSet */
/*******************/

fn decode_method_value(
    h_bs: &mut FdkBitStream,
    method_definition: u8,
    method_value: &mut FixpDbl,
) -> DrcError {
    let val: FixpDbl;
    match method_definition as i32 {
        MD_UNKNOWN_OTHER
        | MD_PROGRAM_LOUDNESS
        | MD_ANCHOR_LOUDNESS
        | MD_MAX_OF_LOUDNESS_RANGE
        | MD_MOMENTARY_LOUDNESS_MAX
        | MD_SHORT_TERM_LOUDNESS_MAX => {
            let tmp = h_bs.read_bits(8) as i32;
            /* -57.75 + tmp * 0.25; */
            val =
                fl2fxconst_dbl((-57.75f32 / 128.0f32) as f64) + (tmp << (DFRACT_BITS - 1 - 2 - 7));
        }
        MD_LOUDNESS_RANGE => {
            let tmp = h_bs.read_bits(8) as i32;
            if tmp == 0 {
                val = 0;
            } else if tmp <= 128 {
                val = tmp << (DFRACT_BITS - 1 - 2 - 7); /* tmp * 0.25; */
            } else if tmp <= 204 {
                /* 0.5 * tmp - 32.0f; */
                val = (tmp << (DFRACT_BITS - 1 - 1 - 7))
                    - fl2fxconst_dbl((32.0f32 / 128.0f32) as f64);
            } else {
                /* downscale by 1 more bit to prevent overflow at intermediate result */
                let v =
                    (tmp << (DFRACT_BITS - 1 - 8)) - fl2fxconst_dbl((134.0f32 / 256.0f32) as f64);
                val = v << 1; /* tmp - 134.0; */
            }
        }
        MD_MIXING_LEVEL => {
            let tmp = h_bs.read_bits(5) as i32;
            /* tmp + 80.0; */
            val = (tmp << (DFRACT_BITS - 1 - 7)) + fl2fxconst_dbl((80.0f32 / 128.0f32) as f64);
        }
        MD_ROOM_TYPE => {
            let tmp = h_bs.read_bits(2) as i32;
            val = tmp << (DFRACT_BITS - 1 - 7); /* tmp; */
        }
        MD_SHORT_TERM_LOUDNESS => {
            let tmp = h_bs.read_bits(8) as i32;
            /* -116.0 + tmp * 0.5; */
            val =
                fl2fxconst_dbl((-116.0f32 / 128.0f32) as f64) + (tmp << (DFRACT_BITS - 1 - 1 - 7));
        }
        _ => return DE_NOT_OK, /* invalid methodDefinition value */
    }
    *method_value = val;
    DE_OK
}

fn read_loudness_measurement(
    h_bs: &mut FdkBitStream,
    p_meas: &mut LoudnessMeasurement,
) -> DrcError {
    p_meas.method_definition = h_bs.read_bits(4) as u8;
    let err = decode_method_value(h_bs, p_meas.method_definition, &mut p_meas.method_value);
    if err != 0 {
        return err;
    }
    p_meas.measurement_system = h_bs.read_bits(4) as u8;
    p_meas.reliability = h_bs.read_bits(2) as u8;

    err
}

fn read_loudness_info(
    h_bs: &mut FdkBitStream,
    _version: i32,
    loudness_info: &mut LoudnessInfo,
) -> DrcError {
    loudness_info.drc_set_id = h_bs.read_bits(6) as i8;
    loudness_info.downmix_id = h_bs.read_bits(7) as u8;

    loudness_info.sample_peak_level_present = h_bs.read_bits(1) as u8;
    if loudness_info.sample_peak_level_present != 0 {
        let bs_sample_peak_level = h_bs.read_bits(12) as i32;
        if bs_sample_peak_level == 0 {
            loudness_info.sample_peak_level_present = 0;
            loudness_info.sample_peak_level = 0;
        } else {
            /* 20.0 - bsSamplePeakLevel * 0.03125; */
            loudness_info.sample_peak_level = fl2fxconst_dbl((20.0f32 / 128.0f32) as f64)
                - (bs_sample_peak_level << (DFRACT_BITS - 1 - 5 - 7));
        }
    }

    loudness_info.true_peak_level_present = h_bs.read_bits(1) as u8;
    if loudness_info.true_peak_level_present != 0 {
        let bs_true_peak_level = h_bs.read_bits(12) as i32;
        if bs_true_peak_level == 0 {
            loudness_info.true_peak_level_present = 0;
            loudness_info.true_peak_level = 0;
        } else {
            loudness_info.true_peak_level = fl2fxconst_dbl((20.0f32 / 128.0f32) as f64)
                - (bs_true_peak_level << (DFRACT_BITS - 1 - 5 - 7));
        }
        h_bs.push_for(4); /* truePeakLevelMeasurementSystem */
        h_bs.push_for(2); /* truePeakLevelReliability */
    }

    let measurement_count = h_bs.read_bits(4) as usize;
    loudness_info.measurement_count = f_min(measurement_count, 16) as u8;
    for i in 0..measurement_count {
        let mut tmp_meas = LoudnessMeasurement::ZERO;
        let err = read_loudness_measurement(h_bs, &mut tmp_meas);
        if err != 0 {
            return err;
        }
        if i >= 16 {
            continue;
        }
        loudness_info.loudness_measurement[i] = tmp_meas;
    }

    DE_OK
}

fn read_loudness_info_set_extension(
    h_bs: &mut FdkBitStream,
    h_loudness_info_set: &mut LoudnessInfoSet,
) -> DrcError {
    let p_ext = &mut h_loudness_info_set.loudness_info_set_ext;

    let mut k = 0usize;
    p_ext.loudness_info_set_ext_type[k] = h_bs.read_bits(4) as u8;
    while p_ext.loudness_info_set_ext_type[k] != UNIDRCLOUDEXT_TERM {
        if k >= (8 - 1) {
            return DE_MEMORY_ERROR;
        }
        let bit_size_len = h_bs.read_bits(4);
        let ext_size_bits = bit_size_len + 4;

        let bit_size = h_bs.read_bits(ext_size_bits);
        p_ext.ext_bit_size[k] = bit_size + 1;

        /* add future extensions here */
        h_bs.push_for(p_ext.ext_bit_size[k]);
        k += 1;
        p_ext.loudness_info_set_ext_type[k] = h_bs.read_bits(4) as u8;
    }

    DE_OK
}

/// `drcDec_readMpegh3daLoudnessInfoSet(hBs, hLoudnessInfoSet, subStreamIndex)`: parser for
/// mpegh3daLoudnessInfoSet()
pub fn drc_dec_read_mpegh3da_loudness_info_set(
    h_bs: &mut FdkBitStream,
    h_loudness_info_set: &mut LoudnessInfoSet,
    sub_stream_index: i32,
) -> DrcError {
    let mut diff = 0;
    if sub_stream_index >= 4 {
        return DE_OK;
    }
    let sub = sub_stream_index as usize;

    diff |=
        comp_assign(&mut h_loudness_info_set.loudness_info_count_v0[sub], h_bs.read_bits(6) as u8);

    /* accumulate loudnessInfoCountV0 from lower substreams */
    let mut start_index = 0usize;
    for i in 0..sub {
        start_index += h_loudness_info_set.loudness_info_count_v0[i] as usize;
    }

    h_loudness_info_set.loudness_info_count =
        f_min(start_index + h_loudness_info_set.loudness_info_count_v0[sub] as usize, 32) as u8;
    for i in 0..h_loudness_info_set.loudness_info_count_v0[sub] as usize {
        let mut tmp_loud = LoudnessInfo::ZERO;
        tmp_loud.loudness_info_type = h_bs.read_bits(2) as u8;
        if (tmp_loud.loudness_info_type == 1) || (tmp_loud.loudness_info_type == 2) {
            tmp_loud.mae_group_id = h_bs.read_bits(7) as u8;
        } else if tmp_loud.loudness_info_type == 3 {
            tmp_loud.mae_group_id = h_bs.read_bits(5) as u8; /* mae_groupPresetID */
        }

        let err = read_loudness_info(h_bs, 0, &mut tmp_loud);
        if err != 0 {
            return err;
        }
        if start_index + i >= 32 {
            continue;
        }

        /* insert the loudnessInfo element of this substream, starting from startIndex */
        if diff == 0 {
            diff |= (tmp_loud != h_loudness_info_set.loudness_info[start_index + i]) as i32;
        }
        h_loudness_info_set.loudness_info[start_index + i] = tmp_loud;
    }

    let loudness_info_album_present = h_bs.read_bits(1);
    if loudness_info_album_present != 0 {
        diff |= comp_assign(
            &mut h_loudness_info_set.loudness_info_album_count_v0[sub],
            h_bs.read_bits(6) as u8,
        );
    } else {
        diff |= comp_assign(&mut h_loudness_info_set.loudness_info_album_count_v0[sub], 0);
    }

    /* accumulate loudnessInfoAlbumCountV0 from lower substreams */
    let mut start_index = 0usize;
    for i in 0..sub {
        start_index += h_loudness_info_set.loudness_info_album_count_v0[i] as usize;
    }

    h_loudness_info_set.loudness_info_album_count =
        f_min(start_index + h_loudness_info_set.loudness_info_album_count_v0[sub] as usize, 32)
            as u8;
    for i in 0..h_loudness_info_set.loudness_info_album_count_v0[sub] as usize {
        let mut tmp_loud = LoudnessInfo::ZERO;
        let err = read_loudness_info(h_bs, 0, &mut tmp_loud);
        if err != 0 {
            return err;
        }
        if start_index + i >= 32 {
            continue;
        }

        /* insert the loudnessInfoAlbum element of this substream, starting from startIndex */
        if diff == 0 {
            diff |= (tmp_loud != h_loudness_info_set.loudness_info_album[start_index + i]) as i32;
        }
        h_loudness_info_set.loudness_info_album[start_index + i] = tmp_loud;
    }

    let loudness_info_set_ext_present = h_bs.read_bits(1) as u8;
    if sub == 0 {
        diff |= comp_assign(
            &mut h_loudness_info_set.loudness_info_set_ext_present,
            loudness_info_set_ext_present,
        );
    } else {
        /* loudnessInfoSetExtension is not allowed for side-stream */
        if loudness_info_set_ext_present != 0 {
            return DE_NOT_OK;
        }
    }

    h_loudness_info_set.diff |= diff as u8;

    if loudness_info_set_ext_present != 0 {
        let err = read_loudness_info_set_extension(h_bs, h_loudness_info_set);
        if err != 0 {
            return err;
        }
    }

    DE_OK
}
