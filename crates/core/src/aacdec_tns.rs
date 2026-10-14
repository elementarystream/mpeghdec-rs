// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/aacdec_tns.h and libMpeghDec/src/aacdec_tns.cpp (the TNS functions
// declared in block.h are defined there).

//! Temporal noise shaping tool.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::{FixpTcc, FDKAACDEC_TNS_COEFF3, FDKAACDEC_TNS_COEFF4};
use crate::aacdecoder_lib::*;
use crate::channelinfo::{
    get_maximum_tns_bands, get_scale_factor_band_offsets, get_scale_factor_bands_total,
    get_windows_per_frame, is_long_block, CIcsInfo, SamplingRateInfo,
};
use crate::overlapadd::spec;
use mpeghdec_fdk::audio::{
    AC_EL_ENHANCED_NOISE, AC_EL_IGF_AFTER_TNS, AC_MPEGH3DA, AC_RSVD50, AC_USAC,
};
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::common_fix::{f_min, FixpDbl, FixpQdl};
use mpeghdec_fdk::lpc::{clpc_parcor_to_lpc, clpc_synthesis};

/// `TNS_MAX_WINDOWS`
pub const TNS_MAX_WINDOWS: usize = 8;
/// `TNS_MAXIMUM_FILTERS`
pub const TNS_MAXIMUM_FILTERS: usize = 3;

/// `TNS_MAXIMUM_ORDER` (for memory allocation): 12 for AAC-LC and AAC-SSR, 20 for AAC-Main, 15
/// for USAC.
pub const TNS_MAXIMUM_ORDER: usize = 20;

/// `CFilter`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CFilter {
    /// `Coeff`
    pub coeff: [i8; TNS_MAXIMUM_ORDER],
    /// `StartBand`
    pub start_band: u8,
    /// `StopBand`
    pub stop_band: u8,
    /// `Direction`
    pub direction: i8,
    /// `Resolution`
    pub resolution: i8,
    /// `Order`
    pub order: u8,
}

/// `CTnsData`
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CTnsData {
    /// `Filter`
    pub filter: [[CFilter; TNS_MAXIMUM_FILTERS]; TNS_MAX_WINDOWS],
    /// `NumberOfFilters`
    pub number_of_filters: [u8; TNS_MAX_WINDOWS],
    /// `DataPresent`
    pub data_present: u8,
    /// `Active`
    pub active: u8,
    /// `GainLd`: log2 of the maximum total filter gains. The value is required to keep necessary
    /// mantissa headroom so that while applying the TNS predictor the mantissas do not overflow.
    pub gain_ld: u8,
}

/// `CTns_Reset(pTnsData)`: resets the tns data.
pub fn ctns_reset(p_tns_data: &mut CTnsData) {
    /* Note: the following FDKmemclear should not be required. */
    p_tns_data.filter = [[CFilter::default(); TNS_MAXIMUM_FILTERS]; TNS_MAX_WINDOWS];
    p_tns_data.number_of_filters = [0; TNS_MAX_WINDOWS];
    p_tns_data.data_present = 0;
    p_tns_data.active = 0;
}

/// `CTns_ReadDataPresentFlag(bs, pTnsData)`: reads the data-present flag for tns from the
/// bitstream.
pub fn ctns_read_data_present_flag(bs: &mut FdkBitStream, p_tns_data: &mut CTnsData) {
    p_tns_data.data_present = bs.read_bit() as u8;
}

/// `CTns_Read(bs, pTnsData, pIcsInfo, flags)`: reads the elements for tns from the bitstream.
pub fn ctns_read(
    bs: &mut FdkBitStream,
    p_tns_data: &mut CTnsData,
    p_ics_info: &CIcsInfo,
    flags: u32,
) -> AacDecoderError {
    let error_status = AAC_DEC_OK;

    if p_tns_data.data_present == 0 {
        return error_status;
    }

    let start_window: u8 = 0;
    let wins_per_frame = get_windows_per_frame(p_ics_info) as u8;
    let is_long_flag = is_long_block(p_ics_info);

    p_tns_data.gain_ld = 0;

    for window in start_window..wins_per_frame {
        let w = window as usize;
        let n_filt = bs.read_bits(if is_long_flag != 0 { 2 } else { 1 }) as u8;
        p_tns_data.number_of_filters[w] = n_filt;

        if n_filt != 0 {
            let coef_res = bs.read_bit() as u8;

            let mut nextstopband = get_scale_factor_bands_total(p_ics_info);

            for index in 0..n_filt as usize {
                let filter = &mut p_tns_data.filter[w][index];

                let mut length = bs.read_bits(if is_long_flag != 0 { 6 } else { 4 }) as u8;

                if length > nextstopband {
                    length = nextstopband;
                }

                filter.start_band = nextstopband - length;
                filter.stop_band = nextstopband;
                nextstopband = filter.start_band;

                let order;
                if flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0 {
                    /* max(Order) = 15 (long), 7 (short) */
                    order = bs.read_bits(if is_long_flag != 0 { 4 } else { 3 }) as u8;
                    filter.order = order;
                } else {
                    order = bs.read_bits(if is_long_flag != 0 { 5 } else { 3 }) as u8;
                    filter.order = order;

                    if filter.order as usize > TNS_MAXIMUM_ORDER {
                        return AAC_DEC_TNS_READ_ERROR;
                    }
                }

                if order != 0 {
                    const SGN_MASK: [u8; 3] = [0x2, 0x4, 0x8];
                    const NEG_MASK: [i8; 3] = [!0x3, !0x7, !0xF];

                    filter.direction = if bs.read_bit() != 0 { -1 } else { 1 };

                    let coef_compress = bs.read_bit() as u8;

                    filter.resolution = (coef_res + 3) as i8;

                    let s_mask = SGN_MASK[(coef_res + 1 - coef_compress) as usize];
                    let n_mask = NEG_MASK[(coef_res + 1 - coef_compress) as usize];

                    for i in 0..order as usize {
                        let coef =
                            bs.read_bits((filter.resolution as u8 - coef_compress) as u32) as u8;
                        filter.coeff[i] = if coef & s_mask != 0 {
                            (coef | n_mask as u8) as i8
                        } else {
                            coef as i8
                        };
                    }
                    p_tns_data.gain_ld = 4;
                }
            }
        }
    }

    p_tns_data.active = 1;

    error_status
}

/// `CTns_ReadDataPresentUsac(hBs, pTnsData0, pTnsData1, ptns_on_lr, pIcsInfo, flags, elFlags,
/// fCommonWindow)`.
pub fn ctns_read_data_present_usac(
    hbs: &mut FdkBitStream,
    p_tns_data0: &mut CTnsData,
    p_tns_data1: &mut CTnsData,
    ptns_on_lr: &mut u8,
    p_ics_info: &CIcsInfo,
    flags: u32,
    el_flags: u32,
    f_common_window: i32,
) {
    let mut common_tns = 0;

    if f_common_window != 0 {
        common_tns = hbs.read_bit();
    }
    /*if ( igFilling && !igFAfterTnsSynth) { */
    if (el_flags & (AC_EL_ENHANCED_NOISE | AC_EL_IGF_AFTER_TNS)) == AC_EL_ENHANCED_NOISE {
        *ptns_on_lr = 1;
    } else {
        *ptns_on_lr = hbs.read_bit() as u8;
    }
    if common_tns != 0 {
        p_tns_data0.data_present = 1;
        ctns_read(hbs, p_tns_data0, p_ics_info, flags);

        p_tns_data0.data_present = 0;
        p_tns_data0.active = 1;
        *p_tns_data1 = *p_tns_data0;
    } else {
        let tns_present_both = hbs.read_bit();
        if tns_present_both != 0 {
            p_tns_data0.data_present = 1;
            p_tns_data1.data_present = 1;
        } else {
            p_tns_data1.data_present = hbs.read_bit() as u8;
            p_tns_data0.data_present = (p_tns_data1.data_present == 0) as u8;
        }
    }
}

/// `CTns_Apply(pTnsData, pIcsInfo, pSpectralCoefficient, pSamplingRateInfo, granuleLength,
/// nbands, igf_active, flags)`: applies the tns to the spectrum.
pub fn ctns_apply(
    p_tns_data: &CTnsData,
    p_ics_info: &CIcsInfo,
    p_spectral_coefficient: &mut [FixpDbl],
    p_sampling_rate_info: &SamplingRateInfo,
    granule_length: i32,
    nbands: u8,
    igf_active: u8,
    flags: u32,
) {
    if p_tns_data.active != 0 {
        let mut coeff = [0 as FixpTcc; TNS_MAXIMUM_ORDER];
        let mut temp = [0 as FixpQdl; TNS_MAXIMUM_ORDER];
        let mut lpc_coeff_m = [0 as FixpTcc; TNS_MAXIMUM_ORDER];

        let start_window = 0;
        let wins_per_frame = get_windows_per_frame(p_ics_info);

        for window in start_window..wins_per_frame {
            let p_spectrum = &mut p_spectral_coefficient[spec(window, granule_length)..];

            for index in 0..p_tns_data.number_of_filters[window as usize] as usize {
                let mut syn_state_index = 0;
                let filter = &p_tns_data.filter[window as usize][index];

                if filter.order > 0 {
                    let order = filter.order as usize;
                    if filter.resolution == 3 {
                        for i in 0..order {
                            coeff[i] = FDKAACDEC_TNS_COEFF3[(filter.coeff[i] as i32 + 4) as usize];
                        }
                    } else {
                        for i in 0..order {
                            coeff[i] = FDKAACDEC_TNS_COEFF4[(filter.coeff[i] as i32 + 8) as usize];
                        }
                    }

                    let mut tns_max_bands =
                        get_maximum_tns_bands(p_ics_info, p_sampling_rate_info.sampling_rate_index);
                    /* See redefinition of TNS_MAX_BANDS table */
                    if (flags & (AC_USAC | AC_RSVD50 | AC_MPEGH3DA) != 0)
                        && (p_sampling_rate_info.sampling_rate_index > 5)
                    {
                        tns_max_bands += 1;
                    }

                    let offsets = get_scale_factor_band_offsets(p_ics_info, p_sampling_rate_info);

                    let start = f_min(f_min(filter.start_band, tns_max_bands), nbands);
                    let start = offsets[start as usize] as i32;

                    let stop = if (flags & AC_MPEGH3DA != 0) && igf_active != 0 {
                        f_min(filter.stop_band, nbands)
                    } else {
                        f_min(f_min(filter.stop_band, tns_max_bands), nbands)
                    };
                    let stop = offsets[stop as usize] as i32;

                    let size = stop - start;

                    if size != 0 {
                        let mut state = [0 as FixpDbl; TNS_MAXIMUM_ORDER];

                        let lpc_coeff_e =
                            clpc_parcor_to_lpc(&coeff, &mut lpc_coeff_m, order as i32, &mut temp);
                        clpc_synthesis(
                            &mut p_spectrum[start as usize..stop as usize],
                            filter.direction as i32,
                            &lpc_coeff_m,
                            lpc_coeff_e,
                            order as i32,
                            &mut state,
                            &mut syn_state_index,
                        );
                    }
                }
            }
        }
    }
}
