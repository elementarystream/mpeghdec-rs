// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libPCMutils limiter.h and pcm_utils.h.

#include "limiter.h"
#include "pcm_utils.h"

extern "C" {

void* cref_lim_create(UINT maxAttackMs, UINT releaseMs, FIXP_DBL threshold, UINT maxChannels, UINT maxSampleRate) {
  return pcmLimiter_Create(maxAttackMs, releaseMs, threshold, maxChannels, maxSampleRate);
}
INT cref_lim_destroy(void* h) { return pcmLimiter_Destroy((TDLimiterPtr)h); }
INT cref_lim_reset(void* h) { return pcmLimiter_Reset((TDLimiterPtr)h); }
UINT cref_lim_getDelay(void* h) { return pcmLimiter_GetDelay((TDLimiterPtr)h); }
INT cref_lim_getMaxGainReduction(void* h) { return pcmLimiter_GetMaxGainReduction((TDLimiterPtr)h); }
INT cref_lim_setNChannels(void* h, UINT n) { return pcmLimiter_SetNChannels((TDLimiterPtr)h, n); }
INT cref_lim_setSampleRate(void* h, UINT fs) { return pcmLimiter_SetSampleRate((TDLimiterPtr)h, fs); }
INT cref_lim_setAttack(void* h, UINT ms) { return pcmLimiter_SetAttack((TDLimiterPtr)h, ms); }
INT cref_lim_setRelease(void* h, UINT ms) { return pcmLimiter_SetRelease((TDLimiterPtr)h, ms); }
INT cref_lim_setThreshold(void* h, FIXP_DBL t) { return pcmLimiter_SetThreshold((TDLimiterPtr)h, t); }
INT cref_lim_apply(void* h, INT* in, INT* out, INT* work, FIXP_DBL* gains, INT scaling, UINT n) {
  return pcmLimiter_Apply((TDLimiterPtr)h, in, out, work, gains, scaling, n);
}
/* Scalar state (21 values) and the maxBuf/delayBuf contents (lengths given by the caller). */
void cref_lim_state(void* h, INT* out, INT* maxBuf, UINT maxBufLen, INT* delayBuf, UINT delayBufLen) {
  TDLimiterPtr l = (TDLimiterPtr)h;
  out[0] = l->attack;
  out[1] = l->attackConst;
  out[2] = l->releaseConst;
  out[3] = l->attackMs;
  out[4] = l->releaseMs;
  out[5] = l->maxAttackMs;
  out[6] = l->threshold;
  out[7] = l->channels;
  out[8] = l->maxChannels;
  out[9] = l->sampleRate;
  out[10] = l->maxSampleRate;
  out[11] = l->cor;
  out[12] = l->max;
  out[13] = l->maxBufIdx;
  out[14] = l->delayBufIdx;
  out[15] = l->smoothState0;
  out[16] = l->minGain;
  out[17] = l->cleanSamples;
  out[18] = l->release;
  out[19] = l->previous_mode;
  out[20] = l->scaling;
  FDKmemcpy(maxBuf, l->maxBuf, maxBufLen * sizeof(INT));
  FDKmemcpy(delayBuf, l->delayBuf, delayBufLen * sizeof(INT));
}

void cref_FDK_interleave(const FIXP_DBL* in, INT* out, UINT channels, UINT frameSize, UINT length) {
  FDK_interleave(in, out, channels, frameSize, length);
}

}  // extern "C"
