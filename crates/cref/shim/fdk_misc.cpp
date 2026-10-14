// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK TD_upsampler.h and the STFT filterbank (FDK_stftfilterbank_api.h).

#include "TD_upsampler.h"
#include "FDK_stftfilterbank.h"

extern "C" {

// TD_upsampler.h
SHORT cref_TD_upsampler_init(INT fac, INT* states) { return TD_upsampler_init((TD_FAC_UPSAMPLE)fac, states); }
SHORT cref_TD_upsampler(INT fac, const INT* in, SHORT lenIn, INT* out, INT* states) {
  return TD_upsampler((TD_FAC_UPSAMPLE)fac, in, lenIn, out, states);
}

// FDK_stftfilterbank_api.h
void* cref_stft_open(INT mode, UINT frameSize, UINT fftSize, INT* err) {
  STFT_FILTERBANK_CONFIG cfg;
  HANDLE_STFT_FILTERBANK h = NULL;
  cfg.stftFilterbankMode = (STFT_FILTERBANK_MODE)mode;
  cfg.frameSize = frameSize;
  cfg.fftSize = fftSize;
  *err = StftFilterbank_Open(&cfg, &h);
  return h;
}
void cref_stft_process(INT* in, INT* out, void* h, INT headroom) {
  StftFilterbank_Process(in, out, (HANDLE_STFT_FILTERBANK)h, headroom);
}
INT* cref_stft_prev(void* h) { return ((HANDLE_STFT_FILTERBANK)h)->prevAudioInput; }
INT cref_stft_zeros(void* h) { return ((HANDLE_STFT_FILTERBANK)h)->numZerosPaddedTwoSided; }
INT cref_stft_close(void* h) {
  HANDLE_STFT_FILTERBANK hh = (HANDLE_STFT_FILTERBANK)h;
  return StftFilterbank_Close(&hh);
}

}  // extern "C"
