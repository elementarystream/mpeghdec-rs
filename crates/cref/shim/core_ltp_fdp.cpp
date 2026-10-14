// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libMpeghDec ltp_post.cpp (LTP postfilter) and fdp.cpp (frequency domain
// prediction). The channel structures are allocated with the helpers of core_common.cpp
// (cref_core_ch_new(), cref_core_st_new()), whose first members are the C++ structures.

#include "channelinfo.h"
#include "ltp_post.h"
#include "fdp.h"

extern "C" {

/* readLtpParam() on the bitstream bs (from cref_bs_new()). */
INT cref_core_ltp_read_ltp_param(void* bs, INT* ltp_param, INT isLFE) {
  return readLtpParam((HANDLE_FDK_BITSTREAM)bs, ltp_param, isLFE);
}

/* ltp_post() with the state passed as scalars and arrays. */
void cref_core_ltp_post(FIXP_DBL* sig, INT L_frame, INT fs, INT* ltp_param, INT* pitch_int_past, INT* pitch_fr_past, FIXP_SGL* gain_past, INT* gainIdx_past, FIXP_SGL* mem_in, FIXP_SGL* mem_out) {
  ltp_post(sig, L_frame, fs, ltp_param, pitch_int_past, pitch_fr_past, gain_past, gainIdx_past,
           mem_in, mem_out);
}

/* FDP_DecodeBins() on a channel info from cref_core_ch_new() (pSpectralCoefficient set with
   cref_core_ch_set_spec()) and a static channel info from cref_core_st_new(). */
void cref_core_fdp_decode_bins(void* ch, void* st, UINT flags, const FIXP_DBL* quantSpecCurr, const SHORT* sfbOffsets, INT predictionBandwidth, INT isTcx, FIXP_DBL i_gain_m, INT i_gain_e, INT g_div64, INT lg) {
  FDP_DecodeBins((CAacDecoderChannelInfo*)ch, (CAacDecoderStaticChannelInfo*)st, flags,
                 quantSpecCurr, sfbOffsets, predictionBandwidth, isTcx, i_gain_m, i_gain_e,
                 g_div64, lg);
}

}  // extern "C"
