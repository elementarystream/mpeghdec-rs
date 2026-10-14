// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libIGFdec/include/FDK_igfSCFDecoder.h and libIGFdec/src/FDK_igfSCFDecoder.cpp.

//! Arithmetic decoder for the IGF scale factor energies (`sfe`).
//!
//! Differences to the C++ structure `IGFSCFDEC_PRIVATE_DATA`:
//! - The bitstream handle (`hBs`) and the output pointer (`sfe`), which C++ stores in the
//!   structure for the duration of one [`iis_igf_scf_decoder_decode`] call, are function
//!   arguments.
//! - The history pointers `prevLB`, `prevSB`, `prevTCXmedium` and `prevTCXlong` point into
//!   `prevBuffer`; they are stored as offsets into [`IgfScfDecPrivateData::prev_buffer`].
//! - The fields `high`, `low`, `bits_to_follow` and `value` are never used by the C++ code and
//!   are omitted.

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_arith::ac_arith_coder::{ari_decode_14bits, ari_q4new, cbitsnew, Tastat};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::f_abs;

/// `IGF_SCF_WIN_MODE`
pub type IgfScfWinMode = i32;
pub const IGF_SCF_LONG: IgfScfWinMode = 0;
pub const IGF_SCF_SHORT: IgfScfWinMode = 1;
pub const IGF_SCF_TCX_MEDIUM: IgfScfWinMode = 2;
pub const IGF_SCF_TCX_LONG: IgfScfWinMode = 3;
pub const IGF_SCF_CODEC_TRANSITION: IgfScfWinMode = 4;

/// Size of `prevBuffer`: 64 (LB) + 3*16 (SB and 2 TCX modes).
pub const IGF_SCF_PREV_BUFFER_SIZE: usize = 112;

/// `IGFSCFDEC_PRIVATE_DATA`
#[derive(StateDigest, Clone, Copy, Debug, PartialEq, Eq)]
pub struct IgfScfDecPrivateData {
    /// `st`
    ///
    /// Not part of the state digest: the arithmetic decoder is started (`st` initialized) before
    /// the first symbol of every `iisIGFSCFDecoderDecode()` call.
    #[digest(skip)]
    pub st: Tastat,
    /// `prevBuffer`
    pub prev_buffer: [i8; IGF_SCF_PREV_BUFFER_SIZE],
    /// `prevSB` (offset into `prev_buffer`)
    pub prev_sb: usize,
    /// `prevLB` (offset into `prev_buffer`)
    pub prev_lb: usize,
    /// `prevTCXmedium` (offset into `prev_buffer`)
    pub prev_tcx_medium: usize,
    /// `prevTCXlong` (offset into `prev_buffer`)
    pub prev_tcx_long: usize,
    /// `prevDLB`
    pub prev_dlb: i8,
    /// `prevDSB`
    pub prev_dsb: i8,
    /// `prevDTCXmedium`
    pub prev_dtcx_medium: i8,
    /// `prevDTCXlong`
    pub prev_dtcx_long: i8,
    /// `scfCountLongBlock`
    pub scf_count_long_block: i32,
    /// `scfCountShortBlock`
    pub scf_count_short_block: i32,
    /// `scfCountTCXBlock`
    pub scf_count_tcx_block: i32,
    /// `t`
    pub t: i32,
    /// `lastFrameBlockType`
    pub last_frame_block_type: i32,
    /// `arith_decode_first_symbol`
    pub arith_decode_first_symbol: i32,
}

impl Default for IgfScfDecPrivateData {
    fn default() -> Self {
        IgfScfDecPrivateData {
            st: Tastat::default(),
            prev_buffer: [0; IGF_SCF_PREV_BUFFER_SIZE],
            prev_sb: 0,
            prev_lb: 0,
            prev_tcx_medium: 0,
            prev_tcx_long: 0,
            prev_dlb: 0,
            prev_dsb: 0,
            prev_dtcx_medium: 0,
            prev_dtcx_long: 0,
            scf_count_long_block: 0,
            scf_count_short_block: 0,
            scf_count_tcx_block: 0,
            t: 0,
            last_frame_block_type: 0,
            arith_decode_first_symbol: 0,
        }
    }
}

/// `iisIGFSCFDecLibInit(hPrivateData, scfCountLongBlock, scfCountShortBlock, scfCountTCXBlock)`
pub fn iis_igf_scf_dec_lib_init(
    h: &mut IgfScfDecPrivateData,
    scf_count_long_block: i32,
    scf_count_short_block: i32,
    scf_count_tcx_block: i32,
) {
    let mut scf_count_tcx = scf_count_tcx_block;

    if scf_count_tcx <= 0 {
        scf_count_tcx = scf_count_short_block;
    }

    *h = IgfScfDecPrivateData::default();

    /* init ptrs to qBuffers, this avoids allocating each buffer seperate: */
    h.prev_lb = 0;
    h.prev_sb = (h.prev_lb as i32 + scf_count_long_block) as usize;
    h.prev_tcx_medium = (h.prev_sb as i32 + scf_count_short_block) as usize;
    h.prev_tcx_long = (h.prev_tcx_medium as i32 + scf_count_tcx) as usize;

    h.prev_dlb = 0;
    h.prev_dsb = 0;
    h.prev_dtcx_medium = 0;
    h.prev_dtcx_long = 0;

    h.scf_count_long_block = scf_count_long_block;
    h.scf_count_short_block = scf_count_short_block;
    h.scf_count_tcx_block = scf_count_tcx;
}

/// `arith_first_symbol()`: return TRUE if it is the first symbol of the sequence, FALSE otherwise.
fn arith_first_symbol(h: &mut IgfScfDecPrivateData) -> bool {
    if h.arith_decode_first_symbol != 0 {
        h.arith_decode_first_symbol = 0;
        true
    } else {
        false
    }
}

/// `ari_start_decoding_14bits`, done before the first symbol of a sequence.
fn arith_start_if_first(h: &mut IgfScfDecPrivateData, bs: &mut FdkBitStream) {
    if arith_first_symbol(h) {
        h.st.low = 0;
        h.st.high = ari_q4new;
        h.st.vobf = bs.read_bits(cbitsnew as u32) as i32;
    }
}

/// number of scale factor bands to group for Scale Factor Energies
const SFE_GROUP_SIZE: usize = 2;
/// factor to decrease quantization precision for Scale Factor Energies
const SFE_QUANT_EXTRA: i32 = 2;

const MIN_ENC_SEPARATE: i32 = -12;
const MAX_ENC_SEPARATE: i32 = 12;
/* two extra symbols are used for escape coding of large positive and negative values */
const SYMBOLS_IN_TABLE: i32 = 1 + (MAX_ENC_SEPARATE - MIN_ENC_SEPARATE + 1) + 1;

const CTX_OFFSET: i32 = 3;

fn quant_ctx(ctx: i32) -> i32 {
    /*
      ctx ... -5 -4 -3 -2 -1 0 1 2 3 4 5 ...
    Q(ctx)... -3 -3 -3 -2 -1 0 1 2 3 3 3 ...
    */
    if f_abs(ctx) <= 3 {
        ctx
    } else if ctx > 3 {
        3
    } else {
        /* ctx < -3 */
        -3
    }
}

fn arith_decode_bits(h: &mut IgfScfDecPrivateData, bs: &mut FdkBitStream, n_bits: i32) -> i32 {
    const CF_FOR_BIT: [i16; 2] = [8192, 0];
    let mut x = 0;

    arith_start_if_first(h, bs);

    let mut i = n_bits - 1;
    while i >= 0 {
        let bit = ari_decode_14bits(bs, &mut h.st, &CF_FOR_BIT, 2);
        x += bit << i;
        i -= 1;
    }

    x
}

fn arith_decode_residual(
    h: &mut IgfScfDecPrivateData,
    bs: &mut FdkBitStream,
    cumulative_frequency_table: &[i16],
    table_offset: i32,
) -> i32 {
    arith_start_if_first(h, bs);

    let val = ari_decode_14bits(bs, &mut h.st, cumulative_frequency_table, SYMBOLS_IN_TABLE);
    /* meaning of the values of val: */
    /* esc_{0} MIN_ENC_SEPARATE (MIN_ENC_SEPARATE + 1) ... (MAX_ENC_SEPARATE - 1) MAX_ENC_SEPARATE
     * esc_{SYMBOLS_IN_TABLE - 1} */

    let mut x;
    if val != 0 && val != SYMBOLS_IN_TABLE - 1 {
        let mut tmp = (val - 1) + MIN_ENC_SEPARATE;

        tmp -= table_offset;
        return tmp;
    } else if val == 0 {
        /* escape code 0 to indicate x <= MIN_ENC_SEPARATE - 1 */
        /* decode the tail of the distribution */

        /* decode extra with 4 bits */
        let mut extra = arith_decode_bits(h, bs, 4);
        if extra == 15 {
            /* escape code 15 to indicate extra >= 15 */
            /* decode addtional extra with 7 bits */
            extra = 15 + arith_decode_bits(h, bs, 7);
        }

        x = (MIN_ENC_SEPARATE - 1) - extra;
    } else {
        /* escape code (SYMBOLS_IN_TABLE - 1) to indicate x >= MAX_ENC_SEPARATE + 1 */
        /* decode the tail of the distribution */

        /* decode extra with 4 bits */
        let mut extra = arith_decode_bits(h, bs, 4);
        if extra == 15 {
            /* escape code 15 to indicate extra >= 15 */
            /* decode addtional extra with 7 bits */
            extra = 15 + arith_decode_bits(h, bs, 7);
        }

        x = (MAX_ENC_SEPARATE + 1) + extra;
    }

    x -= table_offset;
    x
}

fn arith_decode_flush(bs: &mut FdkBitStream) {
    bs.push_back(14);
}

fn arith_decode_flush_no_data(bs: &mut FdkBitStream) {
    bs.push_for(2);
}

/// `decode_sfe_vector()`: `prev_x` is the history of `length` values in `h.prev_buffer` starting
/// at `prev_ofs`, `x` receives the decoded vector.
fn decode_sfe_vector(
    h: &mut IgfScfDecPrivateData,
    bs: &mut FdkBitStream,
    t: i32,                            /* frame counter reset to 0 at each independent frame */
    prev_prev_x0: i8,                  /* value d = prev_prev_x[0], when t >= 2 */
    prev_ofs: usize,                   /* previous vector */
    x: &mut [i8],                      /* current vector to decode */
    length: i32,                       /* number of elements to decode */
    igf_win_mode: i32,                 /* whether a short block is processed */
    sfe_reduce_precision_enabled: i32, /* 0 = no pairing, 1 = pairing */
) {
    /*
       f
       ^
       |  d a x
       |    c b
       |      e  --> t
    */
    let sfe_group_enabled = sfe_reduce_precision_enabled != 0 && igf_win_mode == 0;
    let increment = if sfe_group_enabled { SFE_GROUP_SIZE } else { 1 }; /* size of each group */
    let length = if length > 0 { length as usize } else { 0 };

    let mut f = 0usize;
    while f < length {
        let prev_x = |h: &IgfScfDecPrivateData, i: usize| h.prev_buffer[prev_ofs + i] as i32;

        if t == 0 {
            if f == 0 {
                x[f] = arith_decode_bits(h, bs, 7) as i8; /* 7 bit raw */
            } else if f == increment {
                let pred = x[f - increment] as i32; /* pred = b */

                x[f] = (pred + arith_decode_residual(h, bs, &CF_SE01, CF_OFF_SE01 as i32)) as i8;
            } else {
                /* f >= 2 * increment */
                let pred = x[f - increment] as i32; /* pred = b */
                let ctx = quant_ctx(x[f - increment] as i32 - x[f - 2 * increment] as i32); /* Q(b - e) */
                let c = (CTX_OFFSET + ctx) as usize;

                x[f] =
                    (pred + arith_decode_residual(h, bs, &CF_SE02[c], CF_OFF_SE02[c] as i32)) as i8;
            }
        } else if f == 0 {
            if t == 1 {
                let pred = prev_x(h, f); /* pred = a */

                x[f] = (pred + arith_decode_residual(h, bs, &CF_SE10, CF_OFF_SE10 as i32)) as i8;
            } else {
                /* t >= 2 */
                let pred = prev_x(h, f); /* pred = a */
                let ctx = quant_ctx(prev_x(h, f) - prev_prev_x0 as i32); /* Q(a - d) */
                let c = (CTX_OFFSET + ctx) as usize;

                x[f] =
                    (pred + arith_decode_residual(h, bs, &CF_SE20[c], CF_OFF_SE20[c] as i32)) as i8;
            }
        } else {
            /* (t >= 1) && (f >= increment) */
            let pred = prev_x(h, f) + x[f - increment] as i32 - prev_x(h, f - increment); /* pred = a + b - c */
            let ctx_f = quant_ctx(prev_x(h, f) - prev_x(h, f - increment)); /* Q(a - c) */
            let ctx_t = quant_ctx(x[f - increment] as i32 - prev_x(h, f - increment)); /* Q(b - c) */
            let ct = (CTX_OFFSET + ctx_t) as usize;
            let cf = (CTX_OFFSET + ctx_f) as usize;

            x[f] = (pred
                + arith_decode_residual(h, bs, &CF_SE11[ct][cf], CF_OFF_SE11[ct][cf] as i32))
                as i8;
        }

        /* all the values in the group must be identical with the first one */
        for z in 1..increment {
            if f + z < length {
                x[f + z] = x[f];
            }
        }
        f += increment;
    }
}

/// `iisIGFSCFDecoderGetLastFrameWasShortBlock(hPrivateData)`
pub fn iis_igf_scf_decoder_get_last_frame_was_short_block(h: &IgfScfDecPrivateData) -> i32 {
    h.last_frame_block_type
}

/// `iisIGFSCFDecoderSetLastFrameWasShortBlock(hPrivateData, igfBlockType)`
pub fn iis_igf_scf_decoder_set_last_frame_was_short_block(
    h: &mut IgfScfDecPrivateData,
    igf_block_type: i32,
) {
    h.last_frame_block_type = igf_block_type;
}

fn clear_prev(h: &mut IgfScfDecPrivateData, ofs: usize, count: i32) {
    if count > 0 {
        h.prev_buffer[ofs..ofs + count as usize].fill(0);
    }
}

/// `iisIGFSCFDecoderReset(hPrivateData)`: resets the internal decoder memory (context memory).
pub fn iis_igf_scf_decoder_reset(h: &mut IgfScfDecPrivateData) {
    /* reset of coder */
    clear_prev(h, h.prev_lb, h.scf_count_long_block);
    clear_prev(h, h.prev_sb, h.scf_count_short_block);
    clear_prev(h, h.prev_tcx_medium, h.scf_count_tcx_block);
    clear_prev(h, h.prev_tcx_long, h.scf_count_tcx_block);
    h.t = 0;
    h.prev_dlb = 0;
    h.prev_dsb = 0;
    h.prev_dtcx_medium = 0;
    h.prev_dtcx_long = 0;
}

/// Advances the history after decoding a vector: `prevD = prev[0]; prev = x`.
fn advance_history(h: &mut IgfScfDecPrivateData, prev_ofs: usize, x: &[i8], count: i32) -> i8 {
    let prev_d = h.prev_buffer[prev_ofs];
    if count > 0 {
        let n = count as usize;
        h.prev_buffer[prev_ofs..prev_ofs + n].copy_from_slice(&x[..n]);
    }
    prev_d
}

/// Rescale the sfe output values to align to the default 1.5 dB quantization step.
fn rescale_sfe(sfe: &mut [i8], count: i32) {
    /* according to the formula in the calculateScfEnergiesForIGF function: gain = (int)(0.5 + 4
     * * log_2(energy)) */
    /* note that the entire decoding process and history buffer management uses the original
     * unscaled values */
    for tt in 0..count.max(0) as usize {
        sfe[tt] = (sfe[tt] as i32 * SFE_QUANT_EXTRA) as i8;
    }
}

/// `iisIGFSCFDecoderDecode(hPrivateData, hBStr, sfe, indepFlag, igfWinMode, noShortBlock,
/// sfeReducePrecisionEnabled)`: main decoder function.
///
/// `sfe` receives the decoded quantized coefficients (`noShortBlock * scfCountShortBlock` values
/// for short blocks).
pub fn iis_igf_scf_decoder_decode(
    h: &mut IgfScfDecPrivateData,
    bs: &mut FdkBitStream,
    sfe: &mut [i8],
    indep_flag: i32,
    igf_win_mode: i32,
    no_short_block: i32,
    sfe_reduce_precision_enabled: i32,
) {
    h.arith_decode_first_symbol = 1;

    /* check if coder needs a reset and do it if neccessary */
    let is_window_transition = h.last_frame_block_type != igf_win_mode;
    if indep_flag != 0 || is_window_transition {
        /* reset of coder */
        iis_igf_scf_decoder_reset(h);
    }

    match igf_win_mode {
        IGF_SCF_LONG => {
            let count = h.scf_count_long_block;
            decode_sfe_vector(
                h,
                bs,
                h.t,
                h.prev_dlb,
                h.prev_lb,
                sfe,
                count,
                igf_win_mode,
                sfe_reduce_precision_enabled,
            );
            /* advancing history: */
            h.prev_dlb = advance_history(h, h.prev_lb, sfe, count);

            if sfe_reduce_precision_enabled != 0 {
                rescale_sfe(sfe, count);
            }

            h.t += 1;
            h.last_frame_block_type = igf_win_mode;

            if count != 0 {
                arith_decode_flush(bs); /* finish decoding */
            } else {
                arith_decode_flush_no_data(bs); /* finish decoding if nothing is read*/
            }
        }
        IGF_SCF_SHORT => {
            /* sfe data with short blocks now e.g. noShortBlock=3, scfCountShortBlock = 4:
             * e11 e12 e13 e14 | e21 e22 e23 e24 | e31 e32 e33 e34 */
            let count = h.scf_count_short_block;
            let mut ofs = 0usize;
            for _w in 0..no_short_block {
                let x = &mut sfe[ofs..];
                decode_sfe_vector(
                    h,
                    bs,
                    h.t,
                    h.prev_dsb,
                    h.prev_sb,
                    x,
                    count,
                    igf_win_mode,
                    sfe_reduce_precision_enabled,
                );
                /* advancing history: */
                h.prev_dsb = advance_history(h, h.prev_sb, x, count);

                if sfe_reduce_precision_enabled != 0 {
                    rescale_sfe(x, count);
                }

                ofs = (ofs as i32 + count) as usize;
                h.t += 1;
            }
            h.last_frame_block_type = igf_win_mode;

            if count != 0 {
                arith_decode_flush(bs); /* finish decoding */
            } else {
                arith_decode_flush_no_data(bs); /* finish decoding if nothing is read*/
            }
        }
        _ => {}
    }
}

/***************** begin generated tables from sfe_tables.c *****************/
/* Generated with scripts/codegen/dump_table.py from libIGFdec/src/FDK_igfSCFDecoder.cpp. */

// static short cf_off_se00 = -55;

const CF_OFF_SE01: i16 = 2;

const CF_OFF_SE10: i16 = -4;

static CF_SE01: [i16; 27] = [
    16370, 16360, 16350, 16336, 16326, 16283, 16215, 16065, 15799, 15417, 14875, 13795, 12038,
    9704, 6736, 3918, 2054, 1066, 563, 311, 180, 98, 64, 20, 15, 5, 0,
];

static CF_SE10: [i16; 27] = [
    16218, 16145, 16013, 15754, 15426, 14663, 13563, 11627, 8894, 6220, 4333, 3223, 2680, 2347,
    2058, 1887, 1638, 1472, 1306, 1154, 1012, 895, 758, 655, 562, 489, 0,
];

static CF_SE02: [[i16; 27]; 7] = [
    [
        16332, 16306, 16278, 16242, 16180, 16086, 15936, 15689, 15289, 14657, 13632, 12095, 9926,
        6975, 4213, 2285, 1163, 637, 349, 196, 125, 82, 52, 28, 11, 2, 0,
    ],
    [
        16370, 16367, 16364, 16358, 16350, 16330, 16284, 16170, 16030, 15647, 14840, 13094, 10364,
        6833, 3742, 1639, 643, 282, 159, 85, 42, 22, 16, 15, 4, 1, 0,
    ],
    [
        16373, 16371, 16367, 16363, 16354, 16336, 16290, 16204, 16047, 15735, 14940, 13159, 10171,
        6377, 3044, 1212, 474, 208, 115, 60, 27, 14, 7, 6, 5, 1, 0,
    ],
    [
        16382, 16377, 16367, 16357, 16334, 16281, 16213, 16035, 15613, 14694, 12898, 9720, 5747,
        2506, 1030, 469, 251, 124, 58, 48, 35, 17, 12, 7, 6, 5, 0,
    ],
    [
        16383, 16375, 16374, 16366, 16336, 16250, 16107, 15852, 15398, 14251, 12117, 8796, 5016,
        2288, 998, 431, 236, 132, 89, 37, 16, 12, 4, 3, 2, 1, 0,
    ],
    [
        16375, 16357, 16312, 16294, 16276, 16222, 16133, 15999, 15515, 14655, 13123, 10667, 7324,
        4098, 2073, 1141, 630, 370, 209, 93, 48, 39, 12, 11, 10, 9, 0,
    ],
    [
        16343, 16312, 16281, 16179, 16067, 15730, 15464, 15025, 14392, 13258, 11889, 10224, 7824,
        5761, 3902, 2349, 1419, 837, 520, 285, 183, 122, 71, 61, 40, 20, 0,
    ],
];

static CF_SE20: [[i16; 27]; 7] = [
    [
        16351, 16344, 16317, 16283, 16186, 16061, 15855, 15477, 14832, 13832, 12286, 10056, 7412,
        4889, 2996, 1739, 1071, 716, 496, 383, 296, 212, 149, 109, 82, 59, 0,
    ],
    [
        16368, 16352, 16325, 16291, 16224, 16081, 15788, 15228, 14074, 12059, 9253, 5952, 3161,
        1655, 1006, 668, 479, 357, 254, 199, 154, 115, 88, 67, 51, 45, 0,
    ],
    [
        16372, 16357, 16339, 16314, 16263, 16169, 15984, 15556, 14590, 12635, 9475, 5625, 2812,
        1488, 913, 641, 467, 347, 250, 191, 155, 117, 89, 72, 59, 46, 0,
    ],
    [
        16371, 16362, 16352, 16326, 16290, 16229, 16067, 15675, 14715, 12655, 9007, 5114, 2636,
        1436, 914, 650, 477, 357, 287, 227, 182, 132, 105, 79, 58, 48, 0,
    ],
    [
        16364, 16348, 16318, 16269, 16192, 16033, 15637, 14489, 12105, 8407, 4951, 2736, 1669,
        1156, 827, 615, 465, 348, 269, 199, 162, 125, 99, 73, 51, 37, 0,
    ],
    [
        16326, 16297, 16257, 16136, 15923, 15450, 14248, 11907, 8443, 5432, 3396, 2226, 1561, 1201,
        909, 699, 520, 423, 323, 255, 221, 163, 121, 87, 71, 50, 0,
    ],
    [
        16317, 16280, 16203, 16047, 15838, 15450, 14749, 13539, 11868, 9790, 7789, 5956, 4521,
        3400, 2513, 1926, 1483, 1100, 816, 590, 431, 306, 214, 149, 105, 60, 0,
    ],
];

static CF_SE11: [[[i16; 27]; 7]; 7] = [
    [
        [
            16375, 16372, 16367, 16356, 16326, 16249, 16009, 15318, 13710, 10910, 7311, 3989, 1850,
            840, 380, 187, 103, 66, 46, 36, 26, 20, 15, 12, 8, 6, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16374, 16363, 16323, 16171, 15649,
            14281, 11398, 7299, 3581, 1336, 428, 135, 49, 17, 7, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16375, 16367, 16347, 16267, 15969, 15044,
            12765, 9094, 5087, 2234, 787, 251, 89, 29, 13, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16379, 16376, 16359, 16313, 16124, 15490, 13752, 10641, 6693, 3409, 1499,
            567, 208, 76, 34, 17, 10, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16375, 16367, 16336, 16220, 15772, 14485, 12105, 8736,
            5367, 2833, 1387, 581, 239, 98, 46, 24, 12, 9, 7, 6, 5, 2, 0,
        ],
        [
            16383, 16382, 16380, 16379, 16377, 16375, 16347, 16269, 16004, 15265, 13542, 10823,
            7903, 5214, 3145, 1692, 847, 365, 139, 47, 14, 9, 8, 5, 4, 3, 0,
        ],
        [
            16381, 16378, 16375, 16372, 16336, 16274, 16039, 15643, 14737, 13185, 11186, 8836,
            6501, 4198, 2444, 1270, 615, 281, 153, 93, 63, 48, 42, 33, 24, 21, 0,
        ],
    ],
    [
        [
            16383, 16382, 16381, 16380, 16379, 16377, 16376, 16373, 16369, 16357, 16316, 16205,
            15866, 14910, 12674, 8962, 4857, 1970, 632, 204, 75, 34, 15, 9, 5, 3, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16376, 16375, 16374, 16370, 16356,
            16298, 16139, 15598, 14050, 10910, 6488, 2627, 701, 138, 38, 12, 6, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16375, 16358, 16292, 15999, 15070,
            12735, 8772, 4549, 1595, 376, 95, 26, 10, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16376, 16375, 16373, 16361, 16309,
            16153, 15563, 13983, 10829, 6716, 3004, 1002, 267, 74, 19, 5, 4, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16353, 16250, 15897, 14810, 12582,
            9100, 5369, 2494, 884, 281, 87, 31, 12, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16379, 16378, 16377, 16371, 16348, 16282, 16042, 15416, 13942, 11431,
            8296, 5101, 2586, 1035, 328, 68, 15, 9, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16380, 16379, 16373, 16340, 16267, 16130, 15773, 14969, 13751, 11722, 9172,
            6092, 3329, 1507, 563, 186, 86, 26, 23, 10, 7, 6, 5, 4, 1, 0,
        ],
    ],
    [
        [
            16382, 16381, 16380, 16379, 16377, 16370, 16359, 16312, 16141, 15591, 14168, 11084,
            6852, 3124, 1105, 354, 124, 48, 25, 14, 7, 6, 5, 4, 3, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16374, 16357, 16301, 16076, 15343,
            13341, 9379, 4693, 1476, 324, 67, 18, 9, 7, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16369, 16349, 16265, 15937, 14834,
            12076, 7587, 3123, 769, 152, 44, 13, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16376, 16367, 16324, 16160, 15574, 13854,
            10306, 5601, 1880, 436, 113, 34, 18, 9, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16372, 16348, 16267, 15929, 14858, 12426,
            8315, 4098, 1412, 384, 112, 40, 16, 11, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16376, 16367, 16310, 16123, 15532, 13965, 11248,
            7655, 3910, 1573, 491, 141, 43, 18, 9, 8, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16381, 16379, 16378, 16377, 16373, 16371, 16367, 16347, 16280, 16132, 15778,
            14963, 13688, 11380, 8072, 4680, 2140, 774, 193, 63, 33, 15, 7, 5, 4, 0,
        ],
    ],
    [
        [
            16382, 16381, 16380, 16379, 16378, 16377, 16373, 16360, 16339, 16250, 15927, 14873,
            12393, 8549, 4645, 2000, 748, 271, 109, 48, 19, 9, 5, 4, 3, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16371, 16351, 16244, 15876, 14627,
            11604, 6836, 2711, 772, 210, 54, 21, 8, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16376, 16372, 16341, 16209, 15686, 13965,
            10150, 5099, 1594, 333, 74, 27, 12, 8, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16376, 16369, 16321, 16091, 15261, 12834,
            8160, 3248, 821, 187, 59, 22, 11, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16372, 16350, 16249, 15838, 14425, 11097,
            6138, 2238, 628, 180, 53, 21, 13, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16377, 16365, 16308, 16026, 15269, 13352, 9583,
            5246, 2223, 754, 202, 57, 26, 9, 8, 7, 6, 4, 3, 2, 1, 0,
        ],
        [
            16379, 16378, 16377, 16376, 16375, 16370, 16365, 16338, 16270, 16120, 15723, 14760,
            12783, 9474, 5727, 2713, 977, 296, 93, 39, 14, 12, 10, 7, 4, 3, 0,
        ],
    ],
    [
        [
            16383, 16382, 16379, 16378, 16377, 16370, 16364, 16342, 16267, 16032, 15272, 13475,
            10375, 6652, 3685, 1813, 805, 358, 152, 61, 33, 20, 9, 7, 5, 3, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16376, 16361, 16311, 16096, 15280, 13085,
            9315, 5003, 1992, 647, 170, 60, 25, 17, 7, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16377, 16376, 16375, 16372, 16355, 16288,
            15990, 14926, 12076, 7449, 3161, 981, 302, 78, 24, 7, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16377, 16373, 16351, 16264, 15836, 14299, 10534,
            5358, 1777, 499, 145, 44, 17, 11, 8, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16377, 16366, 16324, 16155, 15416, 13055, 8332,
            3423, 1080, 304, 97, 39, 16, 9, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16377, 16373, 16359, 16258, 15905, 14720, 11631, 6834,
            2911, 1022, 345, 116, 49, 24, 14, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16380, 16379, 16378, 16377, 16376, 16375, 16370, 16365, 16338, 16236, 15960,
            15302, 13685, 10788, 6853, 3314, 1213, 417, 149, 59, 25, 8, 3, 2, 1, 0,
        ],
    ],
    [
        [
            16378, 16377, 16376, 16374, 16373, 16368, 16349, 16303, 16149, 15653, 14445, 12326,
            9581, 6707, 4156, 2251, 1062, 460, 202, 93, 53, 25, 12, 8, 3, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16378, 16374, 16365, 16317, 16146, 15685, 14441,
            11949, 8459, 4949, 2280, 874, 300, 86, 29, 20, 10, 7, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16377, 16358, 16306, 16114, 15474, 13793, 10641,
            6491, 3116, 1219, 382, 135, 62, 26, 17, 11, 6, 5, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16376, 16361, 16305, 16051, 15112, 12593, 8234,
            4130, 1583, 552, 182, 59, 25, 10, 9, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16381, 16380, 16379, 16376, 16346, 16245, 15837, 14409, 10881, 5964,
            2333, 798, 279, 100, 41, 14, 9, 7, 6, 5, 4, 3, 2, 1, 0,
        ],
        [
            16383, 16382, 16380, 16379, 16377, 16361, 16331, 16156, 15454, 13155, 8820, 4256, 1671,
            610, 218, 84, 42, 14, 10, 9, 8, 6, 5, 4, 3, 2, 0,
        ],
        [
            16382, 16380, 16378, 16377, 16367, 16352, 16241, 16077, 15536, 14352, 11787, 7926,
            4119, 1726, 638, 233, 91, 28, 16, 9, 8, 6, 5, 4, 3, 1, 0,
        ],
    ],
    [
        [
            16369, 16361, 16352, 16340, 16315, 16284, 16223, 16091, 15848, 15385, 14573, 13396,
            11681, 9316, 6613, 4037, 2144, 1033, 491, 213, 100, 55, 34, 18, 12, 6, 0,
        ],
        [
            16382, 16381, 16379, 16376, 16371, 16359, 16345, 16306, 16198, 16002, 15534, 14580,
            12881, 10271, 6793, 3572, 1467, 504, 152, 60, 23, 14, 5, 4, 2, 1, 0,
        ],
        [
            16383, 16382, 16380, 16379, 16378, 16376, 16367, 16360, 16344, 16292, 16183, 15902,
            15224, 13793, 11340, 7866, 4409, 1916, 689, 225, 80, 34, 16, 6, 3, 1, 0,
        ],
        [
            16381, 16380, 16379, 16377, 16376, 16372, 16366, 16353, 16325, 16266, 16097, 15632,
            14551, 12346, 9014, 5262, 2439, 920, 324, 126, 50, 20, 9, 6, 4, 1, 0,
        ],
        [
            16383, 16380, 16379, 16377, 16375, 16373, 16369, 16360, 16338, 16283, 16183, 15892,
            15109, 13313, 10173, 6308, 3103, 1264, 457, 169, 75, 30, 15, 5, 2, 1, 0,
        ],
        [
            16379, 16377, 16372, 16370, 16365, 16347, 16296, 16186, 15988, 15448, 14083, 11465,
            7678, 4215, 1961, 900, 431, 193, 87, 37, 21, 13, 8, 5, 2, 1, 0,
        ],
        [
            16373, 16368, 16360, 16342, 16320, 16294, 16230, 16123, 15884, 15548, 14801, 13380,
            11064, 7909, 4654, 2378, 1114, 490, 235, 135, 74, 40, 21, 11, 6, 1, 0,
        ],
    ],
];

static CF_OFF_SE02: [i16; 7] = [1, 1, 1, 0, 0, 1, 2];

static CF_OFF_SE20: [i16; 7] = [0, -2, -2, -2, -3, -4, -3];

static CF_OFF_SE11: [[i16; 7]; 7] = [
    [-5, 0, 0, -3, -1, 0, -1],
    [1, 3, 0, 3, 0, 0, -1],
    [-2, 0, 0, 0, 0, 0, 3],
    [0, 0, 0, 0, 0, 0, 2],
    [0, 0, 3, 0, 0, 0, 4],
    [0, 1, 0, 0, 0, 0, 1],
    [0, 1, 3, 3, 4, 2, 4],
];

/*****************   end generated tables from sfe_tables.c *****************/
