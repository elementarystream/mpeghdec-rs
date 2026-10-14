// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libMpeghDec (conceal): the concealment functions of conceal.cpp.
//
// Handles: `params` is a CConcealParams from cref_core_conceal_params_new(), `st` a static channel
// info from cref_core_st_new() and `ch` a channel info from cref_core_ch_new() (see
// core_common.cpp). The static and dynamic channel infos are the first members of the structures
// allocated there, so the handles are cast directly.

#include "channelinfo.h"
#include "conceal.h"

static CAacDecoderStaticChannelInfo* st_of(void* st) { return (CAacDecoderStaticChannelInfo*)st; }
static CAacDecoderChannelInfo* ch_of(void* ch) { return (CAacDecoderChannelInfo*)ch; }

extern "C" {

void cref_core_conceal_init_common_data(void* params) { CConcealment_InitCommonData((CConcealParams*)params); }

/* Also sets concealmentInfo.pConcealParams to params. */
void cref_core_conceal_init_channel_data(void* st, void* params, INT initRenderMode, INT samplesPerFrame) {
  CConcealment_InitChannelData(&st_of(st)->concealmentInfo, (CConcealParams*)params,
                               (AACDEC_RENDER_MODE)initRenderMode, samplesPerFrame);
}

/* params may be NULL */
INT cref_core_conceal_get_method(void* params) { return (INT)CConcealment_GetMethod((CConcealParams*)params); }

/* params may be NULL */
UINT cref_core_conceal_get_delay(void* params) { return CConcealment_GetDelay((CConcealParams*)params); }

/* params may be NULL */
INT cref_core_conceal_set_params(void* params, INT method, INT fadeOutSlope, INT fadeInSlope, INT muteRelease, FIXP_DBL comfNoiseLevel) {
  return (INT)CConcealment_SetParams((CConcealParams*)params, method, fadeOutSlope, fadeInSlope,
                                     muteRelease, comfNoiseLevel);
}

/* params, fadeOut and fadeIn may be NULL */
INT cref_core_conceal_set_attenuation(void* params, const SHORT* fadeOut, const SHORT* fadeIn) {
  return (INT)CConcealment_SetAttenuation((CConcealParams*)params, fadeOut, fadeIn);
}

/* st may be NULL */
INT cref_core_conceal_get_state(void* st) {
  return (INT)CConcealment_GetState(st ? &st_of(st)->concealmentInfo : NULL);
}

/* st may be NULL */
INT cref_core_conceal_get_last_frame_ok(void* st, INT fBeforeApply) {
  return CConcealment_GetLastFrameOk(st ? &st_of(st)->concealmentInfo : NULL, fBeforeApply);
}

void cref_core_conceal_store(void* st, void* ch) {
  CConcealment_Store(&st_of(st)->concealmentInfo, ch_of(ch), st_of(st));
}

/* pSamplingRateInfo is not used by CConcealment_Apply(); a zeroed structure is passed. */
INT cref_core_conceal_apply(void* st, void* ch, INT samplesPerFrame, INT lastLpdMode, INT frameOk, UINT flags) {
  SamplingRateInfo sri;
  FDKmemclear(&sri, sizeof(sri));
  return CConcealment_Apply(&st_of(st)->concealmentInfo, ch_of(ch), st_of(st), &sri, samplesPerFrame,
                            (UCHAR)lastLpdMode, frameOk, flags);
}

INT cref_core_conceal_td_fading(INT len, void* st, INT aacOutDataHeadroom, FIXP_DBL* pcmdata) {
  CAacDecoderStaticChannelInfo* p = st_of(st);
  return CConcealment_TDFading(len, &p, aacOutDataHeadroom, pcmdata, NULL);
}

}  // extern "C"
