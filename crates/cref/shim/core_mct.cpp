// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libMpeghDec mct.cpp (multichannel coding tool): CMct allocation, state dump/load
// (mirrored by crates/cref/tests/core_mct.rs), bitstream parsing and MCT stereo filling on the
// channel structures of shim/core_common.cpp.

#include "channelinfo.h"
#include "aacdecoder.h"
#include "mct.h"
#include "FDK_igfDec.h"

#define DUMP(x) (out[n++] = (INT)(x))
#define DUMP_ARR(a, len)                                   \
  do {                                                     \
    for (int _i = 0; _i < (int)(len); _i++) DUMP((a)[_i]); \
  } while (0)
#define LOAD(x, T) ((x) = (T)in[n++])
#define LOAD_ARR(a, len, T)                                         \
  do {                                                              \
    for (int _i = 0; _i < (int)(len); _i++) (a)[_i] = (T)in[n++]; \
  } while (0)

#define MAX_CH 28

/* The channel handles of shim/core_common.cpp (CREF_CH / CREF_ST) start with the
   CAacDecoderChannelInfo / CAacDecoderStaticChannelInfo, so they are converted by a cast. */
static void get_channels(void** ch, void** st, INT nCh, CAacDecoderChannelInfo** chs,
                         CAacDecoderStaticChannelInfo** sts) {
  for (int i = 0; i < nCh && i < MAX_CH; i++) {
    chs[i] = (CAacDecoderChannelInfo*)ch[i];
    if (sts != NULL) sts[i] = (CAacDecoderStaticChannelInfo*)st[i];
  }
}

extern "C" {

/* CMct_Initialize(&handle, ...) with the handle in *pp (NULL or an existing CMct). If a new CMct
   is allocated, prevOutSpec/prevOutSpec_exp (FDKmalloc) are zeroed like in the Rust port. */
INT cref_core_mct_initialize(void** pp, UINT mask, INT firstSigIdx, INT signalsInGroup) {
  CMctPtr p = (CMctPtr)*pp;
  INT err = CMct_Initialize(&p, mask, firstSigIdx, signalsInGroup);
  if (*pp == NULL && p != NULL) {
    if (p->prevOutSpec != NULL) FDKmemclear(p->prevOutSpec, p->numMctChannels * 1024 * sizeof(FIXP_DBL));
    if (p->prevOutSpec_exp != NULL) FDKmemclear(p->prevOutSpec_exp, p->numMctChannels * 8 * 16 * sizeof(SHORT));
  }
  *pp = p;
  return err;
}

void cref_core_mct_destroy(void* p) { CMct_Destroy((CMctPtr)p); }

/* Complete CMct state including the work data and the prev buffers (mctMask as high and low
   word, a presence flag before each prev buffer). */
INT cref_core_mct_dump(void* p, INT* out) {
  CMct* m = (CMct*)p;
  CMctWork* w = m->mctWork;
  INT n = 0;
  DUMP(w->keepTree);
  DUMP(w->numPairs);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) DUMP_ARR(w->codePairs[i], 2);
  DUMP_ARR(w->bDeltaTime, MAX_NUM_MCT_BOXES);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) DUMP_ARR(w->pairCoeffDeltaSfb[i], MAX_NUM_MCT_BANDS);
  DUMP_ARR(w->pairCoeffDeltaFb, MAX_NUM_MCT_BOXES);
  DUMP_ARR(w->numMctMaskBands, MAX_NUM_MCT_BOXES);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) {
    DUMP((UINT)(w->mctMask[i] >> 32));
    DUMP((UINT)w->mctMask[i]);
  }
  DUMP_ARR(w->bHasMctMask, MAX_NUM_MCT_BOXES);
  DUMP_ARR(w->bHasBandwiseCoeffs, MAX_NUM_MCT_BOXES);
  DUMP_ARR(w->predDir, MAX_NUM_MCT_BOXES);
  DUMP_ARR(w->hasStereoFilling, MAX_NUM_MCT_CHANNELS);
  DUMP(m->MCCSignalingType);
  DUMP(m->MCTSignalingTypePrev);
  DUMP_ARR(m->windowSequenceIsShortPrev, MAX_NUM_MCT_BOXES);
  DUMP_ARR(m->channelMap, MAX_NUM_MCT_CHANNELS);
  DUMP(m->numMctChannels);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) DUMP_ARR(m->pairCoeffQSfbPrev[i], MAX_NUM_MCT_BANDS);
  DUMP_ARR(m->pairCoeffQFbPrev, MAX_NUM_MCT_BOXES);
  DUMP(m->prevOutSpec != NULL);
  if (m->prevOutSpec != NULL) DUMP_ARR(m->prevOutSpec, m->numMctChannels * 1024);
  DUMP(m->prevOutSpec_exp != NULL);
  if (m->prevOutSpec_exp != NULL) DUMP_ARR(m->prevOutSpec_exp, m->numMctChannels * 8 * 16);
  DUMP(m->Mct_group_parsed);
  return n;
}

/* Loads the layout of cref_core_mct_dump(); numMctChannels and the presence flags are skipped
   (the prev buffers keep their size). */
INT cref_core_mct_load(void* p, const INT* in) {
  CMct* m = (CMct*)p;
  CMctWork* w = m->mctWork;
  INT n = 0;
  LOAD(w->keepTree, UCHAR);
  LOAD(w->numPairs, UCHAR);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) LOAD_ARR(w->codePairs[i], 2, UCHAR);
  LOAD_ARR(w->bDeltaTime, MAX_NUM_MCT_BOXES, UCHAR);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) LOAD_ARR(w->pairCoeffDeltaSfb[i], MAX_NUM_MCT_BANDS, UCHAR);
  LOAD_ARR(w->pairCoeffDeltaFb, MAX_NUM_MCT_BOXES, UCHAR);
  LOAD_ARR(w->numMctMaskBands, MAX_NUM_MCT_BOXES, UCHAR);
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) {
    UINT64 hi = (UINT)in[n++];
    UINT64 lo = (UINT)in[n++];
    w->mctMask[i] = (hi << 32) | lo;
  }
  LOAD_ARR(w->bHasMctMask, MAX_NUM_MCT_BOXES, UCHAR);
  LOAD_ARR(w->bHasBandwiseCoeffs, MAX_NUM_MCT_BOXES, UCHAR);
  LOAD_ARR(w->predDir, MAX_NUM_MCT_BOXES, UCHAR);
  LOAD_ARR(w->hasStereoFilling, MAX_NUM_MCT_CHANNELS, UCHAR);
  LOAD(m->MCCSignalingType, UCHAR);
  LOAD(m->MCTSignalingTypePrev, UCHAR);
  LOAD_ARR(m->windowSequenceIsShortPrev, MAX_NUM_MCT_BOXES, UCHAR);
  LOAD_ARR(m->channelMap, MAX_NUM_MCT_CHANNELS, UCHAR);
  n++; /* numMctChannels */
  for (int i = 0; i < MAX_NUM_MCT_BOXES; i++) LOAD_ARR(m->pairCoeffQSfbPrev[i], MAX_NUM_MCT_BANDS, SHORT);
  LOAD_ARR(m->pairCoeffQFbPrev, MAX_NUM_MCT_BOXES, SHORT);
  n++;
  if (m->prevOutSpec != NULL) LOAD_ARR(m->prevOutSpec, m->numMctChannels * 1024, FIXP_DBL);
  n++;
  if (m->prevOutSpec_exp != NULL) LOAD_ARR(m->prevOutSpec_exp, m->numMctChannels * 8 * 16, SHORT);
  LOAD(m->Mct_group_parsed, UCHAR);
  return n;
}

INT cref_core_mct_parse(void* p, void* bs, INT usacIndepFlag, INT nChannels) {
  return CMct_inverseMctParseBS((CMctPtr)p, (HANDLE_FDK_BITSTREAM)bs, usacIndepFlag, nChannels);
}

/* CMct_MCT_StereoFilling() on the channel handles ch[0..nCh) / st[0..nCh) (the channels of the
   signal group, i.e. already offset by channelOffset). streamInfo only carries
   aacSamplesPerFrame; the sampling rate info is getSamplingRateInfo(1024, srIdx, sampleRate). */
INT cref_core_mct_stereo_filling(void* p, INT aacSamplesPerFrame, void** ch, void** st, INT nCh, UINT srIdx, UINT sampleRate, const UINT* elFlags, UINT usacIndepFrame) {
  CAacDecoderChannelInfo* chs[MAX_CH];
  CAacDecoderStaticChannelInfo* sts[MAX_CH];
  get_channels(ch, st, nCh, chs, sts);
  CStreamInfo si;
  FDKmemclear(&si, sizeof(si));
  si.aacSamplesPerFrame = aacSamplesPerFrame;
  SamplingRateInfo sri;
  FDKmemclear(&sri, sizeof(sri));
  getSamplingRateInfo(&sri, 1024, srIdx, sampleRate);
  return CMct_MCT_StereoFilling((CMctPtr)p, &si, chs, sts, &sri, elFlags, usacIndepFrame);
}

void cref_core_mct_save_prev(void* p, void** ch, INT nCh) {
  CAacDecoderChannelInfo* chs[MAX_CH];
  get_channels(ch, NULL, nCh, chs, NULL);
  CMct_StereoFilling_save_prev((CMctPtr)p, chs);
}

void cref_core_mct_clear_prev(void* p, void** ch, INT nCh) {
  CAacDecoderChannelInfo* chs[MAX_CH];
  get_channels(ch, NULL, nCh, chs, NULL);
  CMct_StereoFilling_clear_prev((CMctPtr)p, chs);
}

/* Sets the IGF data read by the MCT: fSpectrumTab_sfb_exp (IGF_MAX_TILES x 128 values) of the
   static data and the whitening levels bitstreamData[0].igfWhiteningLevel (IGF_MAX_TILES values). */
void cref_core_mct_igf_set(void* stIgf, void* chIgf, const SHORT* sfbExp, const INT* whitening) {
  IGF_PRIVATE_STATIC_DATA* s = (IGF_PRIVATE_STATIC_DATA*)stIgf;
  IGF_PRIVATE_DATA* d = (IGF_PRIVATE_DATA*)chIgf;
  FDKmemcpy(s->fSpectrumTab_sfb_exp, sfbExp, sizeof(s->fSpectrumTab_sfb_exp));
  for (int t = 0; t < IGF_MAX_TILES; t++) d->bitstreamData[0].igfWhiteningLevel[t] = (WHITENING_LEVEL)whitening[t];
}

}  // extern "C"
