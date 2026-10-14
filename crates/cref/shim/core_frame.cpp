// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for the libMpeghDec frame level: the aacDecoder_*() API (aacdecoder_lib.cpp), the
// earcon mixer PcmDataPayload() (aacdecoder.cpp) and the mpeghdecoder_*() API
// (mpeghdecoder.cpp).

#include "aacdecoder_lib.h"
#include "aacdecoder.h"
#include "mpeghdecoder.h"

extern "C" {

/* ---- aacDecoder_*() ---- */

void* cref_aacdec_open(INT transportFmt, UINT nrOfLayers) {
  return aacDecoder_Open((TRANSPORT_TYPE)transportFmt, nrOfLayers);
}

void cref_aacdec_close(void* h) { aacDecoder_Close((HANDLE_AACDECODER)h); }

INT cref_aacdec_set_param(void* h, INT param, INT value) {
  return aacDecoder_SetParam((HANDLE_AACDECODER)h, (AACDEC_PARAM)param, value);
}

INT cref_aacdec_config_raw(void* h, UCHAR* conf, UINT length) {
  UCHAR* c[1] = {conf};
  UINT l[1] = {length};
  return aacDecoder_ConfigRaw((HANDLE_AACDECODER)h, c, l);
}

INT cref_aacdec_fill(void* h, const UCHAR* buffer, UINT bufferSize, UINT* bytesValid) {
  const UCHAR* b[1] = {buffer};
  UINT s[1] = {bufferSize};
  return aacDecoder_Fill((HANDLE_AACDECODER)h, b, s, bytesValid);
}

INT cref_aacdec_decode_frame(void* h, INT* timeData, INT timeDataSize, UINT flags) {
  return aacDecoder_DecodeFrame((HANDLE_AACDECODER)h, (INT_PCM*)timeData, timeDataSize, flags);
}

/* CStreamInfo fields (see core_frame.rs for the order), 24 values */
void cref_aacdec_stream_info(void* h, INT64* out) {
  CStreamInfo* si = aacDecoder_GetStreamInfo((HANDLE_AACDECODER)h);
  INT64 v[] = {si->sampleRate,
               si->sampleRateBeforeRs,
               si->frameSize,
               si->mpeghAUSize,
               si->numChannels,
               si->numChannelsBeforeMix,
               si->aacSampleRate,
               si->aot,
               si->channelConfig,
               si->bitRate,
               si->aacSamplesPerFrame,
               si->aacNumChannels,
               si->outputDelay,
               si->flags,
               si->epConfig,
               si->referenceLayout,
               si->numLostAccessUnits,
               si->numTotalBytes,
               si->numBadBytes,
               si->numTotalAccessUnits,
               si->numBadAccessUnits,
               si->drcProgRefLev,
               si->outputLoudness,
               si->fBsRestartOk};
  for (UINT i = 0; i < sizeof(v) / sizeof(v[0]); i++) out[i] = v[i];
}

/* Internal decoder state (see core_frame.rs for the order), 32 values */
void cref_aacdec_state(void* h, INT* out) {
  HANDLE_AACDECODER self = (HANDLE_AACDECODER)h;
  INT v[] = {self->aacChannels,
             self->ascChannels[0],
             self->frameOK,
             (INT)self->flags[0],
             self->flushStatus,
             self->flushCnt,
             self->buildUpStatus,
             self->buildUpCnt,
             self->hasAudioPreRoll,
             self->applyCrossfade,
             self->truncateStartOffset[0],
             self->truncateStartOffset[1],
             self->truncateStartOffset[2],
             self->truncateStopOffset[0],
             self->truncateStopOffset[1],
             self->truncateStopOffset[2],
             self->truncateFrameSize,
             self->truncateSampleCount,
             self->discardSamplesAtStartCnt,
             self->targetLayout,
             self->targetLayout_config,
             self->uiManagerEnabled,
             self->uiManagerActive,
             self->uiStatusValid,
             self->drcStatusValid,
             self->uiSignalChanged,
             self->useElementSkipping,
             self->limiterEnableUser,
             self->limiterEnableCurr,
             self->multibandDrcPresent,
             self->aacChannelsPrev,
             self->downmixId};
  for (UINT i = 0; i < sizeof(v) / sizeof(v[0]); i++) out[i] = v[i];
}

/* ---- PcmDataPayload() ---- */

void* cref_earcon_new(void) {
  EarconDecoder* e = (EarconDecoder*)FDKcalloc(1, sizeof(EarconDecoder));
  EarconDecoder_Init(e);
  return e;
}

void cref_earcon_free(void* e) { FDKfree(e); }

/* EarconDecoder state without EarconData (see core_frame.rs for the order), 48 values */
void cref_earcon_get(void* p, INT* v) {
  EarconDecoder* e = (EarconDecoder*)p;
  EarconConfig* c = &e->earconConfig;
  INT i = 0;
  v[i++] = c->m_numPcmSignals;
  v[i++] = c->m_bsPcmFrameSize_index;
  v[i++] = c->m_bsPcmFrameSize;
  v[i++] = c->m_bsPcmLoudnessValue;
  v[i++] = c->m_pcmHasAttenuationGain;
  v[i++] = c->m_bsPcmAttenuationGain;
  v[i++] = c->m_bsPcmTruncationLength;
  v[i++] = c->m_bsMHASTruncationLength;
  v[i++] = c->sampling_frequency;
  v[i++] = c->TruncationFlag;
  v[i++] = c->EarconFlag;
  v[i++] = c->EarconParseError;
  v[i++] = e->earconInfo.m_numEarcons;
  v[i++] = e->First_Frame;
  v[i++] = e->target_loudness_old;
  v[i++] = e->m_bsPcmLoudnessValue_old;
  v[i++] = e->m_bsPcmAttenuationGain_old;
  v[i++] = e->EarconGain;
  v[i++] = e->EarconShift;
  v[i++] = e->AttGain_store_high_precision;
  v[i++] = e->AttGain_exp_store_high_precision;
  v[i++] = e->AttGain_intermediate;
  v[i++] = e->AttGainShift_intermediate;
  v[i++] = e->AttGain_increment_intermediate;
  v[i++] = e->AttGain_old;
  v[i++] = e->AttGainShift_old;
  v[i++] = e->AttGain_increment_old;
  v[i++] = e->AttGain_new;
  v[i++] = e->AttGainShift_new;
  v[i++] = e->AccumulatedFrameSize;
  v[i++] = e->BaseframeSize;
  v[i++] = e->TruncationPresent;
  v[i++] = e->numPcmSignalsInFrame;
  v[i++] = e->numPcmSignals_old;
  v[i++] = e->numSignalsMixed;
  v[i++] = e->NumberOfRestChannels;
  v[i++] = e->CurrentFrameHasEarcon;
  v[i++] = e->LastFrameHadEarcon;
  v[i++] = e->speakerPosIndices[0];
  v[i++] = e->speakerPosIndices[1];
  v[i++] = e->speakerGains[0];
  v[i++] = e->speakerGains[1];
  for (int k = 0; k < 6; k++) v[i++] = e->speakerPosIndices_Rest[k];
}

void cref_earcon_set(void* p, const INT* v) {
  EarconDecoder* e = (EarconDecoder*)p;
  EarconConfig* c = &e->earconConfig;
  INT i = 0;
  c->m_numPcmSignals = v[i++];
  c->m_bsPcmFrameSize_index = v[i++];
  c->m_bsPcmFrameSize = v[i++];
  c->m_bsPcmLoudnessValue = v[i++];
  c->m_pcmHasAttenuationGain = v[i++];
  c->m_bsPcmAttenuationGain = v[i++];
  c->m_bsPcmTruncationLength = v[i++];
  c->m_bsMHASTruncationLength = v[i++];
  c->sampling_frequency = v[i++];
  c->TruncationFlag = v[i++];
  c->EarconFlag = v[i++];
  c->EarconParseError = v[i++];
  e->earconInfo.m_numEarcons = v[i++];
  e->First_Frame = v[i++];
  e->target_loudness_old = v[i++];
  e->m_bsPcmLoudnessValue_old = v[i++];
  e->m_bsPcmAttenuationGain_old = v[i++];
  e->EarconGain = v[i++];
  e->EarconShift = v[i++];
  e->AttGain_store_high_precision = v[i++];
  e->AttGain_exp_store_high_precision = v[i++];
  e->AttGain_intermediate = v[i++];
  e->AttGainShift_intermediate = v[i++];
  e->AttGain_increment_intermediate = v[i++];
  e->AttGain_old = v[i++];
  e->AttGainShift_old = v[i++];
  e->AttGain_increment_old = v[i++];
  e->AttGain_new = v[i++];
  e->AttGainShift_new = v[i++];
  e->AccumulatedFrameSize = v[i++];
  e->BaseframeSize = v[i++];
  e->TruncationPresent = v[i++];
  e->numPcmSignalsInFrame = v[i++];
  e->numPcmSignals_old = v[i++];
  e->numSignalsMixed = v[i++];
  e->NumberOfRestChannels = v[i++];
  e->CurrentFrameHasEarcon = v[i++];
  e->LastFrameHadEarcon = v[i++];
  e->speakerPosIndices[0] = v[i++];
  e->speakerPosIndices[1] = v[i++];
  e->speakerGains[0] = v[i++];
  e->speakerGains[1] = v[i++];
  for (int k = 0; k < 6; k++) e->speakerPosIndices_Rest[k] = v[i++];
}

SHORT* cref_earcon_data(void* p) { return ((EarconDecoder*)p)->EarconData; }

INT cref_pcm_data_payload(void* p, INT* timeData, UINT baseframeSize, INT drcTargetLoudness, INT defaultTargetLoudness, INT targetLayout, INT lastFrameSamples, INT newFrameSamples) {
  return PcmDataPayload((EarconDecoder*)p, (FIXP_DBL*)timeData, baseframeSize,
                        (SCHAR)drcTargetLoudness, (SCHAR)defaultTargetLoudness, targetLayout,
                        (SHORT)lastFrameSamples, (SHORT)newFrameSamples);
}

/* ---- ui.cpp constants (the static functions are not reachable) ---- */

/* the fixed point constants of computeLoudnessCompensationGain() and applyUserInteractivity() */
void cref_ui_constants(INT* v) {
  v[0] = FL2FXCONST_DBL(0.3321928f * (float)(1 << 1));
  v[1] = FL2FXCONST_DBL(3.01029996 / (float)(1 << 2));
  v[2] = FL2FXCONST_SGL(1.5f / 180.f * 64);
  v[3] = FL2FXCONST_SGL(3.f / 180.f * 32);
  v[4] = FL2FXCONST_DBL(1.0f / (float)(1 << 9));
  v[5] = FL2FXCONST_DBL(1.0f / (float)(1 << 7));
  v[6] = FL2FXCONST_DBL(0.5f / 127.0f);
  v[7] = FL2FXCONST_SGL(1.0 / 127);
  v[8] = TDL_MPEGH3DA_DEFAULT_THRESHOLD;
}

/* ---- mpeghdecoder_*() ---- */

void* cref_mpegh_init(INT cicpSetup) { return mpeghdecoder_init(cicpSetup); }

void cref_mpegh_destroy(void* h) { mpeghdecoder_destroy((HANDLE_MPEGH_DECODER_CONTEXT)h); }

INT cref_mpegh_set_mha_config(void* h, const UCHAR* config, UINT configSize) {
  return mpeghdecoder_setMhaConfig((HANDLE_MPEGH_DECODER_CONTEXT)h, config, configSize);
}

INT cref_mpegh_process(void* h, const UCHAR* data, UINT length, UINT64 timestamp) {
  return mpeghdecoder_process((HANDLE_MPEGH_DECODER_CONTEXT)h, data, length, timestamp);
}

INT cref_mpegh_process_timescale(void* h, const UCHAR* data, UINT length, UINT64 timestamp, UINT timescale) {
  return mpeghdecoder_processTimescale((HANDLE_MPEGH_DECODER_CONTEXT)h, data, length, timestamp,
                                       timescale);
}

/* info: numSamplesPerChannel, numChannels, sampleRate, loudness, isConcealed; ts: ticks, pts */
INT cref_mpegh_get_samples(void* h, INT* out, UINT outLength, INT* info, UINT64* ts) {
  MPEGH_DECODER_OUTPUT_INFO oi;
  FDKmemclear(&oi, sizeof(oi));
  MPEGH_DECODER_ERROR err =
      mpeghdecoder_getSamples((HANDLE_MPEGH_DECODER_CONTEXT)h, (int32_t*)out, outLength, &oi);
  info[0] = oi.numSamplesPerChannel;
  info[1] = oi.numChannels;
  info[2] = oi.sampleRate;
  info[3] = oi.loudness;
  info[4] = oi.isConcealed;
  ts[0] = oi.ticks;
  ts[1] = oi.pts;
  return err;
}

INT cref_mpegh_flush_and_get(void* h) {
  return mpeghdecoder_flushAndGet((HANDLE_MPEGH_DECODER_CONTEXT)h);
}

INT cref_mpegh_flush(void* h) { return mpeghdecoder_flush((HANDLE_MPEGH_DECODER_CONTEXT)h); }

INT cref_mpegh_set_param(void* h, INT param, INT value) {
  return mpeghdecoder_setParam((HANDLE_MPEGH_DECODER_CONTEXT)h, (MPEGH_DECODER_PARAMETER)param,
                               value);
}

} /* extern "C" */
