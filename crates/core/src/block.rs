// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/block.h, libMpeghDec/src/block.cpp and
// libMpeghDec/src/arm/block_arm.cpp. Of the ARM file only CBlock_ScaleSpectralData_func1 is
// compiled in the reference configuration (FUNCTION_CBlock_ScaleSpectralData_func1 is defined
// unconditionally); its result equals the generic scaling loop.

//! Long/short block decoding: scale factors, arithmetic coded spectral data, inverse
//! quantization, noise filling, IGF stereo filling, the tool chain of `ApplyTools()` and the
//! frequency to time transform.
//!
//! See the [crate documentation](crate) for the memory passed to these functions.

use crate::aac_rom::*;
use crate::aacdec_tns::ctns_apply;
use crate::aacdecoder_lib::*;
use crate::channelinfo::*;
use crate::overlapadd::spec;
use mpeghdec_arith::ac_arith_coder::{carco_decode_arith_data, ARITH_CODER_ERROR, ARITH_CODER_OK};
use mpeghdec_fdk::audio::{
    AC_EL_ENHANCED_NOISE, AC_EL_IGF_AFTER_TNS, AC_EL_IGF_INDEP_TILING, AC_EL_IGF_USE_ENF, AC_INDEP,
    AC_MPEGH3DA, AC_RSVD50, AC_USAC,
};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{f_div_norm, sqrt_fixp_lookup_e};
use mpeghdec_fdk::mdct::{
    imlt_block, BLOCK_SHORT, BLOCK_START, BLOCK_STOP, MDCT_OUT_HEADROOM,
    MLT_FLAG_CURR_ALIAS_SYMMETRY,
};
use mpeghdec_fdk::scale::{get_scalefactor, scale_value, scale_values_saturate_to};
use mpeghdec_fdk::tools_rom::fdk_get_window_slope;
use mpeghdec_igf::igf_dec::*;

/// `EvaluatePower43(pValue, lsb)`: calculate `2^(lsb/4) * value^(4/3)`.
///
/// `p_value` is the quantized value; the inverse quantized result is stored back. `lsb` are the
/// 2 LSBs of the scale factor (`scaleFactor % 4`) applied as power 2 factor to the resulting
/// inverse quantized value. Returns the exponent of the result (mantissa) stored into `p_value`.
#[inline]
pub fn evaluate_power43(p_value: &mut FixpDbl, lsb: u32) -> i32 {
    let q = *p_value;
    let mut value_exp = A2_TO_POW0P25_EXP;

    if q < INV_QUANT_TABLESIZE {
        *p_value = f_mult(INVERSE_QUANT_TABLE_MANT[q as usize], A2_TO_POW0P25_MANT[lsb as usize]);
        value_exp += INVERSE_QUANT_TABLE_EXP[q as usize] as i32;
    } else {
        let idx = (q >> 3) as usize;
        let e0 = INVERSE_QUANT_TABLE_EXP[idx] as i32;
        let e1 = INVERSE_QUANT_TABLE_EXP[idx + 1] as i32;

        let t0 = INVERSE_QUANT_TABLE_MANT[idx] >> (e1 - e0);
        let t1 = INVERSE_QUANT_TABLE_MANT[idx + 1];

        let td = f_mult(t1 - t0, INVERSE_QUANT_INTERPOLATION_TABLE[(q & 7) as usize]);

        *p_value = f_mult(t0 + td, A2_TO_POW0P25_MANT[lsb as usize]);
        value_exp += e1 + INVERSE_QUANT_INTERPOLATION_EXP;
    }

    value_exp
}

/// `InverseQuantize(pSignedValue, lsb, scale)`
#[inline]
pub fn inverse_quantize(p_signed_value: &mut FixpDbl, lsb: u32, mut scale: i32) {
    let signed_value = *p_signed_value;
    if signed_value == 0 {
        return;
    }

    let mut value = if signed_value < 0 { signed_value.wrapping_neg() } else { signed_value };

    scale += evaluate_power43(&mut value, lsb);

    value = scale_value(value, scale);

    *p_signed_value = if signed_value < 0 { value.wrapping_neg() } else { value };
}

/// `CBlock_UnpackIndex(idx, qp, hcb)`: conversion of index values to quantized coefficients.
/// Returns the dimension.
#[inline]
pub fn cblock_unpack_index(mut idx: i32, qp: &mut [FixpDbl], hcb: &CodeBookDescription) -> i32 {
    let dim = hcb.dimension as i32;
    let offset = hcb.offset as i32;
    let bits = hcb.num_bits as i32;
    let mask = (1 << bits) - 1;

    for q in qp.iter_mut().take(dim as usize) {
        *q = (idx & mask) - offset;
        idx >>= bits;
    }

    dim
}

/// `CBlock_DecodeHuffmanWord(bs, hcb)`: reads the huffman codeword from the bitstream and returns
/// the index value.
#[inline]
pub fn cblock_decode_huffman_word(bs: &mut FdkBitStream, hcb: &CodeBookDescription) -> i32 {
    let code_book = hcb.code_book;
    let mut index: u32 = 0;
    let mut val: u32;

    loop {
        val = code_book[index as usize][bs.read_bits(HUFFMAN_BITS) as usize] as u32; /* Expensive memory access */

        if (val & 1) == 0 {
            index = val >> 2;
            continue;
        } else {
            if val & 2 != 0 {
                bs.push_back_cache(1);
            }

            val >>= 2;
            break;
        }
    }

    val as i32
}

/// `CBlock_DecodeHuffmanWordCB(bs, CodeBook)`
#[inline]
pub fn cblock_decode_huffman_word_cb(
    bs: &mut FdkBitStream,
    code_book: &[[u16; HUFFMAN_ENTRIES]],
) -> i32 {
    let mut index: u32 = 0;
    loop {
        index = code_book[index as usize][bs.read_2bits() as usize] as u32; /* Expensive memory access */
        if index & 1 != 0 {
            break;
        }
        index >>= 2;
    }
    if index & 2 != 0 {
        bs.push_back_cache(1);
    }
    (index >> 2) as i32
}

/// `CBlock_ReadScaleFactorData(pAacDecoderChannelInfo, bs, flags)`: read scale factor data. See
/// chapter 4.6.2.3.2 of ISO/IEC 14496-3. The SF_OFFSET = 100 value referenced in chapter
/// 4.6.2.3.3 is already substracted from the scale factor values. Also includes PNS data reading.
pub fn cblock_read_scale_factor_data(
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    bs: &mut FdkBitStream,
    flags: u32,
) -> AacDecoderError {
    let ch = p_aac_decoder_channel_info;
    let mut position: i32 = 0; /* accu for intensity delta coding */
    let mut factor: i32 = ch.p_dyn_data.raw_data_info.global_gain as i32; /* accu for scale factor delta coding */
    let hcb = &AAC_CODE_BOOK_DESCRIPTION_SCL;
    let code_book = hcb.code_book;

    let scale_factor_bands_transmitted = get_scale_factor_bands_transmitted(&ch.ics_info) as usize;
    let window_groups = get_window_groups(&ch.ics_info) as usize;
    let d = &mut *ch.p_dyn_data;
    for group in 0..window_groups {
        let off = group * 16;
        for band in 0..scale_factor_bands_transmitted {
            match d.a_code_book[off + band] {
                ZERO_HCB => {
                    /* zero book */
                    d.a_scale_factor[off + band] = 0;
                }
                INTENSITY_HCB | INTENSITY_HCB2 => {
                    /* intensity steering */
                    let temp = cblock_decode_huffman_word_cb(bs, code_book);
                    position += temp - 60;
                    d.a_scale_factor[off + band] = (position - 100) as i16;
                }
                _ => {
                    /* decode scale factor */
                    if !((flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA)) != 0
                        && band == 0
                        && group == 0)
                    {
                        let temp = cblock_decode_huffman_word_cb(bs, code_book);
                        factor += temp - 60; /* MIDFAC 1.5 dB */
                    }
                    if 255 < factor || 0 > factor {
                        return AAC_DEC_PARSE_ERROR;
                    }
                    d.a_scale_factor[off + band] = (factor - 100) as i16;
                }
            }
        }
    }

    AAC_DEC_OK
}

/// `CBlock_ScaleSpectralData_func1(pSpectrum, maxSfbs, BandOffsets, SpecScale_window, pSfbScale,
/// window)` (ARM variant of the scaling loop; `runs` is always a multiple of 4).
#[inline]
fn cblock_scale_spectral_data_func1(
    p_spectrum: &mut [FixpDbl],
    max_sfbs: i32,
    band_offsets: &[i16],
    spec_scale_window: i32,
    p_sfb_scale: &[i16],
    window: i32,
) {
    for band in 0..max_sfbs.max(0) as usize {
        let start = band_offsets[band] as usize;
        let stop = band_offsets[band + 1] as usize;
        let scale = f_min(
            DFRACT_BITS - 1,
            spec_scale_window - p_sfb_scale[window as usize * 16 + band] as i32,
        );

        if scale != 0 {
            for v in &mut p_spectrum[start..stop] {
                *v >>= scale;
            }
        }
    }
}

/// Scaling of one window of `CBlock_ScaleSpectralData()` / `CBlock_ScaleTileData()`: finds the
/// scaling of the window (including the TNS headroom) and scales the spectrum. Returns the scale.
fn scale_window(
    p_spectrum: &mut [FixpDbl],
    p_sfb_scale: &[i16],
    spec_scale_window_init: i32,
    window: i32,
    max_sfbs: u8,
    tns: &crate::aacdec_tns::CTnsData,
    ics_info: &CIcsInfo,
    band_offsets: &[i16],
    sampling_rate_index: u32,
) -> i32 {
    let w = window as usize;
    let mut spec_scale_window = spec_scale_window_init;

    /* find scaling for current window */
    for band in 0..max_sfbs as usize {
        spec_scale_window = f_max(spec_scale_window, p_sfb_scale[w * 16 + band] as i32);
    }

    if tns.active != 0 && tns.number_of_filters[w] > 0 {
        /* Find max scale of TNS bands */
        let mut spec_scale_window_tns = 0;
        let mut tns_start = get_maximum_tns_bands(ics_info, sampling_rate_index) as i32;
        let mut tns_stop = 0;
        for filter_index in 0..tns.number_of_filters[w] as usize {
            let f = &tns.filter[w][filter_index];
            for band in f.start_band as usize..f.stop_band as usize {
                spec_scale_window_tns =
                    f_max(spec_scale_window_tns, p_sfb_scale[w * 16 + band] as i32);
            }
            /* Find TNS line boundaries for all TNS filters */
            tns_start = f_min(tns_start, f.start_band as i32);
            tns_stop = f_max(tns_stop, f.stop_band as i32);
        }
        spec_scale_window_tns += tns.gain_ld as i32;
        debug_assert!(tns_stop >= tns_start);
        /* Consider existing headroom of all MDCT lines inside the TNS bands. */
        let s = band_offsets[tns_start as usize] as i32;
        let e = band_offsets[tns_stop as usize] as i32;
        spec_scale_window_tns -= get_scalefactor(&p_spectrum[s as usize..s.max(e) as usize]);
        if spec_scale_window <= 17 {
            spec_scale_window_tns += 1;
        }
        /* Add enough mantissa head room such that the spectrum is still representable
        after applying TNS. */
        spec_scale_window = f_max(spec_scale_window, spec_scale_window_tns);
    }

    spec_scale_window
}

/// `CBlock_ScaleSpectralData(pAacDecoderChannelInfo, maxSfbs, pSamplingRateInfo)`: find a common
/// exponent (shift factor) for all sfb in each spectral window, and store them into
/// `CAacDecoderChannelInfo::specScale`. `maxSfbs` is the maximum number of SFBs to be processed
/// (might differ from `icsInfo.MaxSfBands`).
///
/// `p_spectral_coefficient` is `pAacDecoderChannelInfo->pSpectralCoefficient`.
pub fn cblock_scale_spectral_data(
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    p_spectral_coefficient: &mut [FixpDbl],
    max_sfbs: u8,
    p_sampling_rate_info: &SamplingRateInfo,
) {
    let ch = p_aac_decoder_channel_info;
    let band_offsets = get_scale_factor_band_offsets(&ch.ics_info, p_sampling_rate_info);

    ch.spec_scale = [0; 8];

    let mut window = 0;
    for group in 0..get_window_groups(&ch.ics_info) as i32 {
        for _groupwin in 0..get_window_group_length(&ch.ics_info, group) {
            let spec_scale_window_init = ch.spec_scale[window as usize] as i32;
            let p_spectrum = &mut p_spectral_coefficient[spec(window, ch.granule_length)..];

            let spec_scale_window = scale_window(
                p_spectrum,
                &ch.p_dyn_data.a_sfb_scale,
                spec_scale_window_init,
                window,
                max_sfbs,
                &ch.p_dyn_data.tns_data,
                &ch.ics_info,
                band_offsets,
                p_sampling_rate_info.sampling_rate_index,
            );

            /* store scaling of current window */
            ch.spec_scale[window as usize] = spec_scale_window as i16;

            cblock_scale_spectral_data_func1(
                p_spectrum,
                max_sfbs as i32,
                band_offsets,
                spec_scale_window,
                &ch.p_dyn_data.a_sfb_scale,
                window,
            );
            window += 1;
        }
    }
}

/// `CBlock_ScaleTileData(pAacDecoderStaticChannelInfo, pAacDecoderChannelInfo, maxSfbs,
/// pSamplingRateInfo)`: [`cblock_scale_spectral_data`] for the IGF source spectra of all tiles
/// (INF).
pub fn cblock_scale_tile_data(
    p_aac_decoder_static_channel_info: &mut CAacDecoderStaticChannelInfo,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    max_sfbs: u8,
    p_sampling_rate_info: &SamplingRateInfo,
) {
    let st = p_aac_decoder_static_channel_info;
    let ch = p_aac_decoder_channel_info;
    let band_offsets = get_scale_factor_band_offsets(&ch.ics_info, p_sampling_rate_info);
    let is_short = (get_window_sequence(&ch.ics_info) == BLOCK_SHORT) as i32;

    /* Get the number of tiles */
    let number_of_tiles = iis_igf_dec_lib_get_number_of_tiles(&st.igf_static_data, is_short);

    /* Over all tiles */
    for tile_idx in 0..number_of_tiles {
        let mut window = 0;
        for group in 0..get_window_groups(&ch.ics_info) as i32 {
            for _groupwin in 0..get_window_group_length(&ch.ics_info, group) {
                let spec_scale_window_init = 0;

                let tile_spectrum = iis_igf_dec_lib_access_source_spectrum(
                    &st.igf_static_data,
                    &mut ch.igf_data,
                    tile_idx,
                    is_short,
                    window,
                )
                .expect("tile");

                let sfb_scale = *iis_igf_dec_lib_access_source_spectrum_exponent(
                    &mut st.igf_static_data,
                    tile_idx,
                    is_short,
                );

                let spec_scale_window = scale_window(
                    tile_spectrum,
                    &sfb_scale,
                    spec_scale_window_init,
                    window,
                    max_sfbs,
                    &ch.p_dyn_data.tns_data,
                    &ch.ics_info,
                    band_offsets,
                    p_sampling_rate_info.sampling_rate_index,
                );

                /* store scaling of current window */
                iis_igf_dec_lib_access_source_spectrum_scale(
                    &mut st.igf_static_data,
                    tile_idx,
                    is_short,
                )[window as usize] = spec_scale_window as i16;

                cblock_scale_spectral_data_func1(
                    tile_spectrum,
                    max_sfbs as i32,
                    band_offsets,
                    spec_scale_window,
                    &sfb_scale,
                    window,
                );
                window += 1;
            }
        }
    }
}

// The branchless inverse quantization in `inverse_quantize_band()` relies on this.
const _: () = assert!(INVERSE_QUANT_TABLE_MANT[0] == 0);

/// `InverseQuantizeBand(spectrum, sfbScalefactor, maxVal, noLines, scf)`: inverse quantize one
/// sfb. Each value of the sfb is processed according to the formula:
/// `spectrum[i] = Sign(spectrum[i]) * Matissa(spectrum[i])^(4/3) * 2^(lsb/4)`.
#[inline]
fn inverse_quantize_band(
    spectrum: &mut [FixpDbl],
    sfb_scalefactor: &mut i16,
    mut max_val: FixpDbl,
    no_lines: i32,
    scf: i32,
) {
    let msb = scf >> 2;
    let lsb = scf & 3;
    let fac = A2_TO_POW0P25_MANT[lsb as usize];
    let n = no_lines as usize;

    if max_val < INV_QUANT_TABLESIZE {
        let q = max_val;
        max_val = f_mult(INVERSE_QUANT_TABLE_MANT[q as usize], fac);
        let scale = INVERSE_QUANT_TABLE_EXP[q as usize] as i32 - (cnt_leading_zeros(max_val) - 2);
        *sfb_scalefactor = (scale + A2_TO_POW0P25_EXP + msb) as i16;

        // Branchless form of: if q < 0 { s = fMult(-mant[-q], fac) >> (scale - exp[-q]) }
        // else if q > 0 { s = fMult(mant[q], fac) >> (scale - exp[q]) }. All |q| are at most
        // max_val < 1024 (the mask only avoids bounds checks); q == 0 yields 0 as mant[0] == 0.
        for s in &mut spectrum[..n] {
            let q = *s;
            let a = (q.unsigned_abs() & (INV_QUANT_TABLESIZE as u32 - 1)) as usize;
            let mant = INVERSE_QUANT_TABLE_MANT[a];
            let sp = f_mult(if q < 0 { -mant } else { mant }, fac);
            *s = sp.wrapping_shr((scale - INVERSE_QUANT_TABLE_EXP[a] as i32) as u32);
        }
    } else {
        let mut scale = evaluate_power43(&mut max_val, lsb as u32);
        scale -= f_normz(max_val) - 2;
        *sfb_scalefactor = (scale + A2_TO_POW0P25_EXP + msb) as i16;

        for s in &mut spectrum[..n] {
            let mut q = *s;
            let abs_q = f_abs(q);
            if abs_q < INV_QUANT_TABLESIZE {
                if q < 0 {
                    q = -q;
                    let sp = f_mult(-INVERSE_QUANT_TABLE_MANT[q as usize], fac);
                    *s = sp >> (scale - INVERSE_QUANT_TABLE_EXP[q as usize] as i32);
                } else if q > 0 {
                    let sp = f_mult(INVERSE_QUANT_TABLE_MANT[q as usize], fac);
                    *s = sp >> (scale - INVERSE_QUANT_TABLE_EXP[q as usize] as i32);
                }
            } else {
                let idx = (abs_q >> 3) as usize;
                let e0 = INVERSE_QUANT_TABLE_EXP[idx] as i32;
                let e1 = INVERSE_QUANT_TABLE_EXP[idx + 1] as i32;

                let t0 = INVERSE_QUANT_TABLE_MANT[idx] >> (e1 - e0);
                let t1 = INVERSE_QUANT_TABLE_MANT[idx + 1];

                let td = f_mult(t1 - t0, INVERSE_QUANT_INTERPOLATION_TABLE[(abs_q & 7) as usize]);

                let mut sp = f_mult(t0 + td, fac);

                if q < 0 {
                    sp = -sp;
                }

                *s = sp >> (scale - e1 - INVERSE_QUANT_INTERPOLATION_EXP);
            }
        }
    }
}

/// `maxabs_D(pSpectralCoefficient, noLines)`: find max spectral line value of the current sfb.
#[inline]
fn maxabs_d(p_spectral_coefficient: &[FixpDbl]) -> FixpDbl {
    let mut loc_max: FixpDbl = 0;
    for &v in p_spectral_coefficient.iter().rev() {
        /* Expensive memory access */
        loc_max = f_max(f_abs(v), loc_max);
    }
    loc_max
}

/// `CBlock_InverseQuantizeSpectralData(pAacDecoderChannelInfo, pSamplingRateInfo, band_is_noise,
/// active_band_search)`.
///
/// `band_is_noise` is always `pAacDecoderChannelInfo->pDynData->band_is_noise` in the C++ code;
/// the parameter is omitted and the channel's array is used.
pub fn cblock_inverse_quantize_spectral_data(
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    p_spectral_coefficient: &mut [FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    active_band_search: u8,
) -> AacDecoderError {
    let ch = p_aac_decoder_channel_info;
    let scale_factor_bands_transmitted = get_scale_factor_bands_transmitted(&ch.ics_info) as usize;
    let band_offsets = get_scale_factor_band_offsets(&ch.ics_info, p_sampling_rate_info);
    let total_bands = get_scale_factor_bands_total(&ch.ics_info) as usize;
    let d = &mut *ch.p_dyn_data;

    d.a_sfb_scale = [0; 8 * 16];

    let mut window = 0;
    for group in 0..get_window_groups(&ch.ics_info) as i32 {
        for _groupwin in 0..get_window_group_length(&ch.ics_info, group) {
            let win_off = spec(window, ch.granule_length);
            /* inverse quantization */
            for band in 0..scale_factor_bands_transmitted {
                let start = win_off + band_offsets[band] as usize;
                let no_lines = band_offsets[band + 1] as i32 - band_offsets[band] as i32;
                let bnds = group as usize * 16 + band;

                if (d.a_code_book[bnds] == ZERO_HCB)
                    || (d.a_code_book[bnds] == INTENSITY_HCB)
                    || (d.a_code_book[bnds] == INTENSITY_HCB2)
                {
                    continue;
                }

                let p_spectral = &mut p_spectral_coefficient[start..start + no_lines as usize];
                let loc_max = maxabs_d(p_spectral);

                if active_band_search != 0 && loc_max != 0 {
                    d.band_is_noise[group as usize * 16 + band] = 0;
                }

                /* Cheap robustness improvement - Do not remove!!! */
                if f_abs(loc_max) > MAX_QUANTIZED_VALUE {
                    return AAC_DEC_PARSE_ERROR;
                }

                /* The inverse quantization operation is given by (ISO/IEC 14496-3:2009(E)):
                x_invquant = Sign(x_quant) . abs(x_quant)^(4/3)
                A gain derived from the scale factor of the sfb is applied: gain = 2^(0.25*sf).
                With msb = (sf >> 2) and lsb = (sf % 4) the rescaled output is represented by
                mantissa Sign(x_quant) * (2^(lsb/4) * abs(x_quant)^(4/3)) and exponent 2^msb. */

                let msb = d.a_scale_factor[bnds] >> 2;

                /* Inverse quantize band only if it is not empty */
                if loc_max != 0 {
                    inverse_quantize_band(
                        p_spectral,
                        &mut d.a_sfb_scale[window as usize * 16 + band],
                        loc_max,
                        no_lines,
                        d.a_scale_factor[bnds] as i32,
                    );
                } else {
                    d.a_sfb_scale[window as usize * 16 + band] = msb;
                }
            } /* for (band=0; band < ScaleFactorBandsTransmitted; band++) */

            /* Make sure the array is cleared to the end */
            let start_clear = band_offsets[scale_factor_bands_transmitted] as i32;
            let end_clear = band_offsets[total_bands] as i32;
            let diff_clear = end_clear - start_clear;
            if diff_clear > 0 {
                let s = win_off + start_clear as usize;
                p_spectral_coefficient[s..s + diff_clear as usize].fill(0);
            }
            window += 1;
        }
    }

    AAC_DEC_OK
}

/// `noise_level_tab[8]`: `FDKpow(2, (float)(noise_level-14)/3.0f) * 2` (`*2` to compensate for
/// fMultDiv2), `noise_level_tab(noise_level==0) == 0` by definition.
/* Generated with scripts/codegen/dump_table.py from libMpeghDec/src/block.cpp. */
static NOISE_LEVEL_TAB: [FixpSgl; 8] =
    [0x0000, 0x0CB3, 0x1000, 0x1429, 0x1966, 0x2000, 0x2851, 0x32CC];

/// Inlined `UsacRandomSign()`: `seed = (ULONG)((UINT64)seed * 69069 + 5)`.
#[inline(always)]
fn usac_random(seed: u32) -> u32 {
    seed.wrapping_mul(69069).wrapping_add(5)
}

/// `CBlock_ApplyNoise(pAacDecoderStaticChannelInfo, pAacDecoderChannelInfo, pSamplingRateInfo,
/// nfRandomSeed, band_is_noise, flag_INFactive, igf_StereoFilling, max_noise_sfb)`: noise filling
/// (and the independent noise filling of the IGF tiles if INF is active).
///
/// `nfRandomSeed` is always `&pAacDecoderStaticChannelInfo->nfRandomSeed` and `band_is_noise`
/// always `pAacDecoderChannelInfo->pDynData->band_is_noise` in the C++ code; both parameters are
/// omitted.
pub fn cblock_apply_noise(
    p_aac_decoder_static_channel_info: &mut CAacDecoderStaticChannelInfo,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    p_spectral_coefficient: &mut [FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    flag_inf_active: i32,
    igf_stereo_filling: i32,
    max_noise_sfb: i32,
) {
    let st = p_aac_decoder_static_channel_info;
    let ch = p_aac_decoder_channel_info;
    let swb_offset = get_scale_factor_band_offsets(&ch.ics_info, p_sampling_rate_info);
    let is_short = (get_window_sequence(&ch.ics_info) == BLOCK_SHORT) as i32;
    let granule_length = ch.granule_length;

    /* Obtain noise level and scale factor offset. */
    let noise_level = (ch.p_dyn_data.specific_to_usac.fd_noise_level_and_offset >> 5) as usize;
    let noise_val_pos: FixpSgl = NOISE_LEVEL_TAB[noise_level];

    /* noise_offset can change even when noise_level=0. Neccesary for IGF stereo filling */
    let noise_offset =
        (ch.p_dyn_data.specific_to_usac.fd_noise_level_and_offset & 0x1f) as i32 - 16;

    let mut noise_filling_start_offset = if is_short != 0 { 20 } else { 160 };
    if granule_length == 96 {
        noise_filling_start_offset = (3 * noise_filling_start_offset) / 4; /* scale offset with 3/4 for coreCoderFrameLength == 768 */
    }

    /* determine sfb from where on noise filling is applied */
    let mut sfb = 0usize;
    while (swb_offset[sfb] as i32) < noise_filling_start_offset {
        sfb += 1;
    }
    let nf_start_offset_sfb = sfb;

    let igf_num_tile = iis_igf_dec_lib_get_number_of_tiles(&st.igf_static_data, is_short);
    /* Tile_pointer_Array: begin of the source spectrum of the tiles (SPEC(tile, 0, gl)) */
    let tiles_active =
        flag_inf_active != 0 && ch.igf_data.bitstream_data[0].igf_all_zero == 0 && igf_num_tile > 0;

    /* if (noise_level!=0) */
    {
        let mut win = 0i32;
        for g in 0..get_window_groups(&ch.ics_info) as i32 {
            let window_group_length = get_window_group_length(&ch.ics_info, g) as i32;
            for sfb in nf_start_offset_sfb..max_noise_sfb.max(0) as usize {
                let bin_start = swb_offset[sfb] as usize;
                let bin_stop = swb_offset[sfb + 1] as usize;
                let gs = g as usize * 16 + sfb;
                let d = &mut *ch.p_dyn_data;

                let flag_n = d.band_is_noise[gs] != 0;

                /* if all bins of one sfb in one window group are zero modify the scale factor by
                 * noise_offset */
                if flag_n {
                    /* Change scaling factors for empty signal bands */
                    d.a_scale_factor[gs] = (d.a_scale_factor[gs] as i32 + noise_offset) as i16;
                    /* scale factor "sf" implied gain "g" is g = 2^(sf/4) */
                    for gwin in 0..window_group_length {
                        let i = ((win + gwin) * 16) as usize + sfb;
                        d.a_sfb_scale[i] = (d.a_sfb_scale[i] as i32 + (noise_offset >> 2)) as i16;
                    }
                }

                let mut seed = st.nf_random_seed;
                /* + 1 because exponent of MantissaTable[lsb][0] is always 1. */
                let scale = (d.a_scale_factor[gs] >> 2) as i32 + 1;
                let lsb = (d.a_scale_factor[gs] & 3) as usize;
                let mantissa = MANTISSA_TABLE[lsb][0];

                for gwin in 0..window_group_length {
                    let w_off = spec(win + gwin, granule_length);
                    let p_spec = &mut p_spectral_coefficient[w_off..];

                    let scale1 = scale - d.a_sfb_scale[((win + gwin) * 16) as usize + sfb] as i32;
                    let mut scaled_noise_val_pos =
                        scale_value(f_mult_div2(noise_val_pos, mantissa), scale1);
                    let mut scaled_noise_val_neg = scaled_noise_val_pos.wrapping_neg();

                    /* If the whole band is zero, just fill without checking */
                    if flag_n {
                        if igf_stereo_filling != 0 {
                            /* If Stereo Filling is active and this band has been zeroed-out, then
                             * do not scale noise data */
                            scaled_noise_val_pos = fx_sgl2fx_dbl(noise_val_pos);
                            scaled_noise_val_neg = -scaled_noise_val_pos;
                        }
                        for v in &mut p_spec[bin_start..bin_stop] {
                            seed = usac_random(seed); /* Inlined: UsacRandomSign - origin in usacdec_lpd.h */
                            *v = if seed & 0x10000 != 0 {
                                scaled_noise_val_neg
                            } else {
                                scaled_noise_val_pos
                            };
                        }
                    }
                    /*If band is sparsely filled, check for 0 and fill */
                    else {
                        for v in &mut p_spec[bin_start..bin_stop] {
                            if *v == 0 {
                                seed = usac_random(seed); /* Inlined: UsacRandomSign - origin in usacdec_lpd.h */
                                *v = if seed & 0x10000 != 0 {
                                    scaled_noise_val_neg
                                } else {
                                    scaled_noise_val_pos
                                };
                            }
                        }
                    }

                    /* Apply Independent noise filling of tiles  or just copy them if INF is not
                     * active */
                    if tiles_active {
                        let tiles = &mut ch.igf_data.spectrum_tab_array;
                        /* Win_Tile_pointer_Array[tileIdx] = SPEC(Tile_pointer_Array[tileIdx], win +
                         * gwin, granuleLength) + bin_start */
                        let tile_pos = |t: usize, bin: usize| t * IGF_TILE_LEN + w_off + bin;

                        /* If the whole band is zero, just fill without checking */
                        if flag_n {
                            if igf_stereo_filling != 0 {
                                /* If Stereo Filling is active and this band has been zeroed-out,
                                 * then do not scale noise data */
                                scaled_noise_val_pos = fx_sgl2fx_dbl(noise_val_pos);
                                scaled_noise_val_neg = -scaled_noise_val_pos;
                            }

                            for bin in bin_start..bin_stop {
                                for tile_idx in 0..igf_num_tile as usize {
                                    seed = usac_random(seed); /* Inlined: UsacRandomSign - origin in usacdec_lpd.h */
                                    tiles[tile_pos(tile_idx, bin)] = if seed & 0x10000 != 0 {
                                        scaled_noise_val_neg
                                    } else {
                                        scaled_noise_val_pos
                                    };
                                }
                            }
                        }
                        /*If band is sparsely filled, check for 0 and fill */
                        else {
                            for bin in bin_start..bin_stop {
                                if tiles[tile_pos(0, bin)] == 0 {
                                    for tile_idx in 0..igf_num_tile as usize {
                                        seed = usac_random(seed);
                                        tiles[tile_pos(tile_idx, bin)] = if seed & 0x10000 != 0 {
                                            scaled_noise_val_neg
                                        } else {
                                            scaled_noise_val_pos
                                        };
                                    }
                                }
                            }
                        }
                    } /* if(flag_INFactive) */
                } /* for (gwin...) */
                st.nf_random_seed = seed;
            } /* for (sfb...) */
            win += window_group_length;
        } /* for (g...) */

        if flag_inf_active != 0 {
            for tile_idx in 0..igf_num_tile as usize {
                /* If the scale factors have changed due to empty signal bands, reflect these
                 * changes in each tile  */
                st.igf_static_data.f_spectrum_tab_sfb_exp[tile_idx] = ch.p_dyn_data.a_sfb_scale;

                if ch.igf_data.bitstream_data[0].igf_all_zero != 0 {
                    /* copy noise filled spectrum to the internal representation: */
                    /* hMap->fSpectrumTab[0] is the source spectrum of tile tileIdx */
                    let t0 = tile_idx * IGF_TILE_LEN;
                    ch.igf_data.spectrum_tab_array[t0..t0 + 1024]
                        .copy_from_slice(&p_spectral_coefficient[..1024]);
                }
            } /* for (tileIdx) */
        }
    } /* ... */
}

/// Finds the first sfb whose offset is at least `offset` (the noise filling start band).
#[inline]
fn start_sfb(swb_offset: &[i16], offset: i32) -> usize {
    let mut sfb = 0usize;
    while (swb_offset[sfb] as i32) < offset {
        sfb += 1;
    }
    sfb
}

/// `IGF_StereoFillingPrepare(pAacDecoderChannelInfo, pSamplingRateInfo, max_noise_sfb,
/// dmx_prev_modified, dmx_prev_win_exp, band_is_noise)`: stereo filling parameter calculation.
///
/// `p_spectral_coefficient` is `pAacDecoderChannelInfo->pSpectralCoefficient` (read only).
pub fn igf_stereo_filling_prepare(
    p_aac_decoder_channel_info: &CAacDecoderChannelInfo,
    p_spectral_coefficient: &[FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    max_noise_sfb: i32,
    dmx_prev_modified: &mut [FixpDbl],
    dmx_prev_modified_exp: &mut [i16],
    band_is_noise: &[u8],
) {
    let swb_offset =
        get_scale_factor_band_offsets(&p_aac_decoder_channel_info.ics_info, p_sampling_rate_info);

    /* Fixed number for long windows */
    let noise_filling_start_offset = 160;

    /* determine sfb from where on noise filling is applied */
    let nf_start_offset_sfb = start_sfb(swb_offset, noise_filling_start_offset);

    for sfb in nf_start_offset_sfb..max_noise_sfb.max(0) as usize {
        /* Run algorithm only if the particular band has been fully noise filled, i.e. that it
        has been empty before the noise-filling process */
        if band_is_noise[sfb] == 0 {
            continue;
        }

        let bin_start = swb_offset[sfb];
        let bin_stop = swb_offset[sfb + 1];

        /* Calculate the widht of the particular sfb */
        let sfb_width: i16 = bin_stop - bin_start;

        /* Left-allign "sfb_width", so that it is suitable for multiplication calculations */
        let bits = f_normz(sfb_width) - 1;
        let sfb_width_modified: FixpSgl = ((sfb_width as i32) << bits) as FixpSgl;

        /* Find a suitable shift for dmx addition */
        let mut shift = 0;
        while (1i32 << shift) < sfb_width as i32 {
            shift += 1;
        }

        /* Get the spectrum of the particular window */
        let p_spec = p_spectral_coefficient;

        /* Get the previous downmix of the particular window */
        let dmx_prev = &mut *dmx_prev_modified;
        let bs = bin_start as usize;
        let be = bin_stop as usize;

        /* Find the available headroom of the downmix signal and a suitable shift value*/
        let head_shift = get_scalefactor(&dmx_prev[bs..be]);

        /* Avoiding noise boosting. Cleaning noise*/
        if head_shift > 25 {
            dmx_prev_modified_exp[sfb] = -31;
            dmx_prev[bs..be].fill(0);
            continue;
        }

        /* Calculate the energy of the previous downmix */
        /* Make sure energy is never zero, so that the division later is possible */
        let mut energy_dmx: FixpDbl = 1;
        for &v in &dmx_prev[bs..be] {
            energy_dmx = energy_dmx.wrapping_add(f_pow2_div2(v << head_shift) >> shift);
        }

        /* Calculate "energy_dmx" exponent of the particular window */
        let energy_dmx_e = ((dmx_prev_modified_exp[sfb] as i32 - head_shift) << 1) + 1 + shift;

        /* We need to calculate the energy per line (constant) of the scaled, noise-filled
        spectrum. Then energy in the whole band is obtained by multiplying the energy per line by
        the widht of the band */

        /* When we Pow, we get Q4.27->noise_energy_per_line */
        let noise_energy_per_line_sfb = f_pow2(p_spec[bs]);

        /* pSpec is in Q-1.32, but since we multiply it by 4 (2 bits;see algo) it becomes Q1.30.
        The power is in Q2.29. In short: exp= ((-1 + 2) << 1); */
        let noise_energy_per_line_sfb_exp = 2;

        /* We need to compute "sfbWidth - factor", which is equal to "sfbWidth*(1-factor)" */

        /* Get the value of one with the same exponent. It is:" 1<< (31 -
         * noise_energy_per_line_sfb_exp)" */
        let value_of_one: FixpDbl = 0x20000000;

        /* Calculate "(1-factor)" */
        let intermediate_result = value_of_one.wrapping_sub(noise_energy_per_line_sfb);

        /* We multiply by sfbWidth, i.e. calculate "sfbWidth*(1-factor)" */
        let factor = f_mult(intermediate_result, sfb_width_modified);
        let factor_exp = noise_energy_per_line_sfb_exp + (15 - bits);

        /* Divide by energy_sfb. Calculate (sfb_width * (1 - factor))/energy_dmx[sfb] */
        let mut temp_exp = 0;
        let mut temp_fixp_dbl = f_div_norm(factor, energy_dmx, &mut temp_exp);

        /* Correct the output exponent */
        temp_exp += factor_exp - energy_dmx_e;

        /* Calculate sqrt((sfb_width * (1 - factor))/energy_dmx[sfb]) */
        temp_fixp_dbl = sqrt_fixp_lookup_e(temp_fixp_dbl, &mut temp_exp);

        /* Allign max left */
        let tmp_scale = f_normz(temp_fixp_dbl) - 1;
        let mut tmp = temp_fixp_dbl.wrapping_shl(tmp_scale as u32);
        temp_exp -= tmp_scale;
        /* Limit the output value to 10 */
        if temp_exp > 3 {
            /* limit factor to 10.0f */
            tmp = if temp_exp > 4 {
                fl2fxconst_dbl(0.625)
            } else {
                f_min(tmp, fl2fxconst_dbl(0.625))
            };
            temp_exp = 4;
        }

        /* Modify the downmix so that it can be added to the spectrum later: "dmx_prev*tmp" */
        for v in &mut dmx_prev[bs..be] {
            *v = f_mult(*v << head_shift, tmp);
        }

        /* "dmx_prev*tmp" exponent for every sfb and window */
        dmx_prev_modified_exp[sfb] =
            (dmx_prev_modified_exp[sfb] as i32 - head_shift + temp_exp) as i16;
    } /* for (sfb...) */
}

/// `IGF_StereoFillingApply(pAacDecoderChannelInfo, pSpectralCoefficient, p2_tile_spectrum_exp,
/// pSamplingRateInfo, max_noise_sfb, dmx_prev_modified, dmx_prev_modified_e, band_is_noise)`:
/// stereo filling application to the spectrum or an IGF tile.
///
/// Of `pAacDecoderChannelInfo` only `icsInfo` and `pDynData->aScaleFactor` are used; they are
/// passed separately because `p2_tile_spectrum_exp` may be the channel's `pDynData->aSfbScale`.
pub fn igf_stereo_filling_apply(
    ics_info: &CIcsInfo,
    a_scale_factor: &[i16],
    p_spectral_coefficient: &mut [FixpDbl],
    p2_tile_spectrum_exp: &mut [i16],
    p_sampling_rate_info: &SamplingRateInfo,
    max_noise_sfb: i32,
    dmx_prev_modified: &[FixpDbl],
    dmx_prev_modified_e: &[i16],
    band_is_noise: &[u8],
) {
    let swb_offset = get_scale_factor_band_offsets(ics_info, p_sampling_rate_info);

    /*Constant for long blocks*/
    let noise_filling_start_offset = 160;

    /* determine sfb from where on noise filling is applied */
    let nf_start_offset_sfb = start_sfb(swb_offset, noise_filling_start_offset);

    for sfb in nf_start_offset_sfb..max_noise_sfb.max(0) as usize {
        /* if all bins of one sfb in one window group are zero modify the scale factor by
         * noise_offset */
        if band_is_noise[sfb] == 0 {
            continue;
        }

        let bin_start = swb_offset[sfb] as usize;
        let bin_stop = swb_offset[sfb + 1] as usize;

        /* Calculate the width of the particular sfb */
        let sfb_width = (bin_stop - bin_start) as i16;

        /*Find suitable headroom for subsequent additions */
        let mut shift = 0;
        while (1i32 << shift) < sfb_width as i32 {
            shift += 1;
        }

        /* Get the spectrum of the particular window */
        let p_spec = &mut *p_spectral_coefficient;

        /* Get the previous downmix of the particular window */
        let dmx_tmp_pointer = dmx_prev_modified;

        /* Get the scale value of dmx */
        let scale_const = dmx_prev_modified_e[sfb] as i32;

        /* Add the scaled and noise filled values to the scaled downmix from the previous frame.
        For that purpose we need to adjust the binary point position */

        /* pSpec is in Q-1.32, but since we multiply it by 4 (2 bits;see algo) it becomes Q1.30.*/
        let scale_p_spec = 1;

        /* Resulting exponent */

        /* Set to some minimum value to prevent divison by 0 */
        let mut factor: FixpDbl = 1;
        let mut diff_1 = scale_const - scale_p_spec;
        /* Adjust binary point and add */
        let p_spec_exp = if diff_1 >= 0 {
            diff_1 = f_min(diff_1 + 1, 31);
            for bin in bin_start..bin_stop {
                let temp = (dmx_tmp_pointer[bin] >> 1) + (p_spec[bin] >> diff_1);
                p_spec[bin] = temp;
                factor = factor.wrapping_add(f_pow2(temp) >> shift);
            }
            scale_const + 1
        } else {
            diff_1 = f_min(-diff_1 + 1, 31);
            for bin in bin_start..bin_stop {
                let temp = (dmx_tmp_pointer[bin] >> diff_1) + (p_spec[bin] >> 1);
                p_spec[bin] = temp;
                factor = factor.wrapping_add(f_pow2(temp) >> shift);
            }
            scale_p_spec + 1
        };

        /* Stereo filling changes the dynamic range of the signal,despite keeping the energy of
        the signal constant. */
        let head_shift = get_scalefactor(&p_spec[bin_start..bin_stop]);

        /* Do not calculate zeros or -1s */
        if head_shift == 31 {
            p2_tile_spectrum_exp[sfb] = -31;
            continue;
        }

        /* Exponent of the resulting power value */
        let factor_exp = (p_spec_exp << 1) + shift;

        /* Left-allign "sfb_width", so that it is suitable for multiplication calculations */
        let bits = f_normz(sfb_width as FixpDbl) - 1;
        let sfb_width_modified: FixpDbl = (sfb_width as FixpDbl) << bits;
        let sfb_width_modified_exp = 31 - bits;

        /*Calculate Scaled_Width/factor */
        let mut temp_int = 0;
        let temp_fixp_dbl = f_div_norm(sfb_width_modified, factor, &mut temp_int);
        temp_int += sfb_width_modified_exp - factor_exp;

        /*Calculate sqrt(Scaled_Width/factor) */
        factor = sqrt_fixp_lookup_e(temp_fixp_dbl, &mut temp_int);

        let tmp_scale = f_normz(factor) - 1;
        factor = factor.wrapping_shl(tmp_scale as u32);
        temp_int -= tmp_scale;
        if temp_int > 3 {
            /* limit factor to 10.0f */
            factor = if temp_int > 4 {
                fl2fxconst_dbl(0.625)
            } else {
                f_min(factor, fl2fxconst_dbl(0.625))
            };
            temp_int = 4;
        }

        /* Generate scaling factor */
        let lsb = (a_scale_factor[sfb] & 3) as usize;
        let msb = (a_scale_factor[sfb] >> 2) as i32;
        let scaled_one = MANTISSA_TABLE[lsb][0];
        let scaled_one_exp = (msb + 1) as i16;

        /* Modify spectrum and scale it with the orginal scaling factor*/
        for v in &mut p_spec[bin_start..bin_stop] {
            let temp = f_mult(*v << head_shift, factor);
            *v = f_mult(temp, scaled_one);
        }

        /* Calculate output exponent including the output scaling which was avoided in the noise
         * filling stage */
        p2_tile_spectrum_exp[sfb] =
            ((p_spec_exp - head_shift + temp_int) as i16 as i32 + scaled_one_exp as i32) as i16;
    } /* for (sfb...) */
}

/// `CBlock_ReadAcSpectralData(hBs, pAacDecoderChannelInfo, pAacDecoderStaticChannelInfo,
/// pSamplingRateInfo, frame_length, flags)`: read arithmetic encoded spectral data.
///
/// `p_spectral_coefficient` is `pAacDecoderChannelInfo->pSpectralCoefficient`.
pub fn cblock_read_ac_spectral_data(
    hbs: &mut FdkBitStream,
    p_aac_decoder_channel_info: &CAacDecoderChannelInfo,
    p_spectral_coefficient: &mut [FixpDbl],
    p_aac_decoder_static_channel_info: &mut CAacDecoderStaticChannelInfo,
    p_sampling_rate_info: &SamplingRateInfo,
    frame_length: u32,
    flags: u32,
) -> AacDecoderError {
    let ch = p_aac_decoder_channel_info;
    let mut error_aac = AAC_DEC_OK;
    let mut error = ARITH_CODER_OK;

    /* number of transmitted spectral coefficients */
    let band_offsets = get_scale_factor_band_offsets(&ch.ics_info, p_sampling_rate_info);
    let lg = band_offsets[get_scale_factor_bands_transmitted(&ch.ics_info) as usize] as i32;

    let num_win = get_windows_per_frame(&ch.ics_info);
    let win_len = if is_long_block(&ch.ics_info) != 0 {
        frame_length as i32
    } else {
        frame_length as i32 / num_win
    };

    let arith_reset_flag = if flags & AC_INDEP != 0 { 1 } else { hbs.read_bit() as u16 as i32 };

    let h_ar_co = p_aac_decoder_static_channel_info.h_ar_co.as_mut().expect("hArCo");
    for win in 0..num_win {
        let off = spec(win, ch.granule_length);
        error = carco_decode_arith_data(
            h_ar_co,
            hbs,
            &mut p_spectral_coefficient[off..],
            lg,
            win_len,
            (arith_reset_flag != 0 && win == 0) as i32,
        );
        if error != ARITH_CODER_OK {
            break;
        }
    }

    if error == ARITH_CODER_ERROR {
        error_aac = AAC_DEC_PARSE_ERROR;
    }

    error_aac
}

/// `getLowPassLine(pAacDecoderChannelInfo, pAacDecoderStaticChannelInfo, pSamplingRateInfo,
/// elFlags)`
fn get_low_pass_line(
    ch: &CAacDecoderChannelInfo,
    st: &CAacDecoderStaticChannelInfo,
    p_sampling_rate_info: &SamplingRateInfo,
    el_flags: u32,
) -> i32 {
    let mut lowpass_sfb = get_scale_factor_bands_transmitted(&ch.ics_info) as i32;

    if ch.p_dyn_data.raw_data_info.common_window != 0 {
        lowpass_sfb = ch.ics_info.max_sfb_ste as i32;
    }
    if el_flags & AC_EL_ENHANCED_NOISE != 0 {
        lowpass_sfb = f_max(lowpass_sfb, st.igf_static_data.igf_stop_sfb_lb as i32);
    }
    get_scale_factor_band_offsets(&ch.ics_info, p_sampling_rate_info)[lowpass_sfb as usize] as i32
        / 2
}

/// Runs `CIgf_apply()` and (for long blocks) `CIgf_TNF_apply()` on one channel (the two IGF
/// blocks of `ApplyTools()`).
fn apply_tools_igf(
    st: &mut CAacDecoderStaticChannelInfo,
    ch: &mut CAacDecoderChannelInfo,
    spectrum: &mut [FixpDbl],
    igf_common: &mut IgfPrivateDataCommon,
    virtual_spec: &mut [FixpDbl],
    el_flags: u32,
) {
    let mut tnf_mask = [0u8; 1024];
    let is_long = is_long_block(&ch.ics_info) == 1;

    /* Run main IGF routine */
    cigf_apply(
        &mut st.igf_static_data,
        &mut ch.igf_data,
        igf_common,
        virtual_spec,
        spectrum,
        &mut ch.spec_scale,
        if is_long { IGF_GRID_LONG_WINDOW } else { IGF_GRID_SHORT_WINDOW },
        get_window_groups(&ch.ics_info) as i32,
        get_windows_per_frame(&ch.ics_info),
        get_window_group_length_table(&ch.ics_info),
        &mut st.nf_random_seed,
        &mut tnf_mask,
        (el_flags & AC_EL_IGF_USE_ENF != 0) as u8,
        IGF_FRAME_DIVISION_AAC_OR_TCX_LONG,
    );

    if is_long_block(&ch.ics_info) != 0 {
        /* Apply IGF - TNF */
        cigf_tnf_apply(
            &st.igf_static_data,
            &ch.igf_data,
            virtual_spec,
            spectrum,
            &ch.spec_scale,
            &tnf_mask,
            0,
            0,
        );
    }
}

/// `ApplyTools(pAacDecoderStaticChannelInfo[], pAacDecoderChannelInfo[], pSamplingRateInfo,
/// flags, elFlags, channel, common_window)`: apply IGF and TNS.
///
/// The channel arrays and `spec` start at the first channel of the element; `igf_common` is
/// `igf_private_data_common` and `time_data2` is `pTimeData2` (the IGF `virtualSpec` of channel
/// `channel` is `time_data2[1024 * channel..]`), see the [crate documentation](crate).
pub fn apply_tools(
    p_aac_decoder_static_channel_info: &mut [Box<CAacDecoderStaticChannelInfo>],
    p_aac_decoder_channel_info: &mut [Box<CAacDecoderChannelInfo>],
    spec_el: &mut [FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    flags: u32,
    el_flags: u32,
    channel: i32,
    common_window: i32,
    igf_common: &mut [IgfPrivateDataCommon; 2],
    time_data2: &mut [FixpDbl],
) {
    let c = channel as usize;
    apply_tools_channel(
        &mut p_aac_decoder_static_channel_info[c],
        &mut p_aac_decoder_channel_info[c],
        &mut spec_el[c * 1024..(c + 1) * 1024],
        &mut igf_common[c],
        &mut time_data2[1024 * c..1024 * (c + 1)],
        p_sampling_rate_info,
        flags,
        el_flags,
        common_window,
    );
}

/// [`apply_tools`] of one channel with the memory of the channel: its static channel info,
/// channel info, spectrum, `IGF_Common_channel_data_handle` and IGF `virtualSpec` (not in C; the
/// C++ code reaches them through the channel index).
#[allow(clippy::too_many_arguments)]
pub fn apply_tools_channel(
    st: &mut CAacDecoderStaticChannelInfo,
    ch: &mut CAacDecoderChannelInfo,
    spectrum: &mut [FixpDbl],
    igf_common: &mut IgfPrivateDataCommon,
    virtual_spec: &mut [FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    flags: u32,
    el_flags: u32,
    common_window: i32,
) {
    if flags & AC_MPEGH3DA != 0 {
        /* IGF processing before TNS */

        if (el_flags & AC_EL_ENHANCED_NOISE != 0)
            && (el_flags & AC_EL_IGF_AFTER_TNS == 0)
            && ((el_flags & AC_EL_IGF_INDEP_TILING != 0) || common_window == 0)
        {
            apply_tools_igf(st, ch, spectrum, igf_common, virtual_spec, el_flags);
        }
    }

    let mut nbands = get_scale_factor_bands_transmitted(&ch.ics_info);

    if el_flags & AC_EL_ENHANCED_NOISE != 0 {
        let islong = is_long_block(&ch.ics_info);
        let igf = &st.igf_static_data;

        if el_flags & AC_EL_IGF_AFTER_TNS == 0 {
            if islong != 0 {
                if igf.igf_stop_sfb_lb > nbands {
                    nbands = igf.igf_stop_sfb_lb;
                }
            } else if igf.igf_stop_sfb_sb > nbands {
                nbands = igf.igf_stop_sfb_sb;
            }
        } else if islong != 0 {
            if igf.igf_start_sfb_lb < nbands {
                nbands = igf.igf_start_sfb_lb;
            }
        } else if igf.igf_start_sfb_sb < nbands {
            nbands = igf.igf_start_sfb_sb;
        }
    }

    /* Deinterleave */
    if ch.transform_splitting_active != 0 {
        let low_pass_line = get_low_pass_line(ch, st, p_sampling_rate_info, el_flags) as usize;
        let mut tmp = [0 as FixpDbl; 512];

        debug_assert!(low_pass_line <= 512);
        for i in 0..low_pass_line {
            tmp[i] = spectrum[i * 2 + 1];
        }
        for i in 0..low_pass_line {
            spectrum[i] = spectrum[i * 2];
        }
        spectrum[low_pass_line..2 * low_pass_line].copy_from_slice(&tmp[..low_pass_line]);
    }

    ctns_apply(
        &ch.p_dyn_data.tns_data,
        &ch.ics_info,
        spectrum,
        p_sampling_rate_info,
        ch.granule_length,
        nbands,
        (el_flags & AC_EL_ENHANCED_NOISE != 0) as u8,
        flags,
    );

    /* Interleave again */
    if ch.transform_splitting_active != 0 {
        let low_pass_line = get_low_pass_line(ch, st, p_sampling_rate_info, el_flags) as usize;
        let mut tmp = [0 as FixpDbl; 512];

        debug_assert!(low_pass_line <= 512);
        tmp[..low_pass_line].copy_from_slice(&spectrum[low_pass_line..2 * low_pass_line]);
        for i in (0..low_pass_line).rev() {
            spectrum[i * 2] = spectrum[i];
        }
        for i in 0..low_pass_line {
            spectrum[i * 2 + 1] = tmp[i];
        }
        for v in &mut spectrum[(low_pass_line << 1)..1024] {
            *v = 0;
        }
    }

    if (el_flags & AC_EL_ENHANCED_NOISE != 0)
        && (el_flags & AC_EL_IGF_AFTER_TNS != 0)
        && (el_flags & AC_EL_IGF_USE_ENF != 0)
    {
        let is_short = (get_window_sequence(&ch.ics_info) == BLOCK_SHORT) as i32;
        let number_of_tiles = iis_igf_dec_lib_get_number_of_tiles(&st.igf_static_data, is_short);

        /* Apply TNS over tiles */
        for tile_idx in 0..number_of_tiles {
            /* Get tile spectrum */
            let tile_spectrum = iis_igf_dec_lib_access_source_spectrum(
                &st.igf_static_data,
                &mut ch.igf_data,
                tile_idx,
                is_short,
                0,
            )
            .expect("tile");

            /* Apply TNS over the tiles */
            ctns_apply(
                &ch.p_dyn_data.tns_data,
                &ch.ics_info,
                tile_spectrum,
                p_sampling_rate_info,
                ch.granule_length,
                nbands,
                (el_flags & AC_EL_ENHANCED_NOISE != 0) as u8,
                flags,
            );
        } /* for(int tileIdx=0;tileIdx<NumberOfTiles;tileIdx++) */
    }

    if flags & AC_MPEGH3DA != 0 {
        /* When IGF is run after TNS */

        if (el_flags & AC_EL_ENHANCED_NOISE != 0)
            && (el_flags & AC_EL_IGF_AFTER_TNS != 0)
            && ((el_flags & AC_EL_IGF_INDEP_TILING != 0) || common_window == 0)
        {
            apply_tools_igf(st, ch, spectrum, igf_common, virtual_spec, el_flags);
        }
    }
}

/// `getWindow2Nr(length, shape)`
fn get_window2_nr(length: i32, shape: i32) -> i32 {
    let mut nr = 0;

    if shape == 2 {
        /* Low Overlap, 3/4 zeroed */
        nr = (length * 3) >> 2;
    }

    nr
}

/// `CBlock_FrequencyToTime(pAacDecoderStaticChannelInfo, pAacDecoderChannelInfo, outSamples,
/// frameLen, frameOk, pWorkBuffer1, aacOutDataHeadroom, elFlags, elCh)`: transform MDCT spectral
/// data into time domain.
///
/// `p_spectral_coefficient` is `pAacDecoderChannelInfo->pSpectralCoefficient`. The C++ code uses
/// `pAacDecoderChannelInfo->pComStaticData->pWorkBufferCore1->mdctOutTemp` as IMDCT output
/// buffer, which is the memory the caller passes as `pWorkBuffer1` (otherwise unused); here the
/// buffer is `p_work_buffer1` (at least `frameLen` values).
pub fn cblock_frequency_to_time(
    p_aac_decoder_static_channel_info: &mut CAacDecoderStaticChannelInfo,
    p_aac_decoder_channel_info: &mut CAacDecoderChannelInfo,
    p_spectral_coefficient: &mut [FixpDbl],
    out_samples: &mut [PcmDec],
    frame_len: i16,
    _frame_ok: i32,
    p_work_buffer1: &mut [FixpDbl],
    aac_out_data_headroom: i32,
    _el_flags: u32,
    _el_ch: i32,
) {
    let st = p_aac_decoder_static_channel_info;
    let ch = p_aac_decoder_channel_info;
    let frame_len = frame_len as i32;
    let fr;
    let mut fl;

    /* Determine left slope length (fl), right slope length (fr) and transform length (tl).
       USAC: The slope length may mismatch with the previous frame in case of LPD / FD
             transitions. The adjustment is handled by the imdct implementation.
    */
    let mut tl = frame_len;
    let mut n_spec = 1;

    match ch.ics_info.window_sequence {
        BLOCK_STOP => {
            fl = frame_len >> 3;
            fr = frame_len;
        }
        BLOCK_START => {
            /* or StopStartSequence */
            fl = frame_len;
            fr = frame_len >> 3;
            if ch.transform_splitting_active != 0 {
                tl >>= 1;
                n_spec = 2;
                fl = fr;
                ch.spec_scale[0] -= 1;
                ch.spec_scale[1] = ch.spec_scale[0];

                /* Deinterleave */
                let mut tmp = [0 as FixpDbl; 512];
                for i in 0..512 {
                    tmp[i] = p_spectral_coefficient[i * 2 + 1];
                }
                for i in 0..512 {
                    p_spectral_coefficient[i] = p_spectral_coefficient[i * 2];
                }
                p_spectral_coefficient[512..1024].copy_from_slice(&tmp[..512]);
            }
        }
        BLOCK_SHORT => {
            fl = frame_len >> 3;
            fr = fl;
            tl >>= 3;
            n_spec = 8;
        }
        /* default, BLOCK_LONG */
        _ => {
            fl = frame_len;
            fr = frame_len - get_window2_nr(frame_len, get_window_shape(&ch.ics_info) as i32);
            /* New startup needs differentiation between sine shape and low overlap shape.
            This is a special case for the LD-AAC transformation windows, because
            the slope length can be different while using the same window sequence. */
            if st.i_mdct.prev_tl == 0 {
                fl = fr;
            }
        }
    }

    {
        let tmp = &mut p_work_buffer1[..frame_len as usize];
        let shape = get_window_shape(&ch.ics_info) as i32;
        imlt_block(
            &mut st.i_mdct,
            tmp,
            p_spectral_coefficient,
            &ch.spec_scale,
            n_spec,
            frame_len,
            tl,
            fdk_get_window_slope(fl, shape),
            fl,
            fdk_get_window_slope(fr, shape),
            fr,
            0,
            if ch.curr_aliasing_symmetry != 0 { MLT_FLAG_CURR_ALIAS_SYMMETRY } else { 0 },
        );

        scale_values_saturate_to(
            &mut out_samples[..frame_len as usize],
            tmp,
            MDCT_OUT_HEADROOM - aac_out_data_headroom,
        );
    }

    st.prev_window_shape = ch.ics_info.window_shape;
}
