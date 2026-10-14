// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libDRCdec: the bitstream readers (drcDec_reader.h) on standalone structures, the
// public decoder API (FDK_drcDecLib.h) and serializers of the internal state for differential
// testing. The serializers write every field (including union storage) as INT values in
// declaration order; tests/drc_common/mod.rs contains the matching Rust serializers.

#include "FDK_drcDecLib.h"
#include "drcDec_types.h"
#include "drcDec_reader.h"
#include "drcDec_gainDecoder.h"
#include "drcDec_selectionProcess.h"

/* Layout of struct s_drc_decoder in FDK_drcDecLib.cpp (the type is private to that file). */
typedef enum { SHIM_DRC_DEC_NOT_INITIALIZED = 0 } SHIM_DRC_DEC_STATUS;
struct shim_drc_decoder {
  DRC_DEC_CODEC_MODE codecMode;
  DRC_DEC_FUNCTIONAL_RANGE functionalRange;
  SHIM_DRC_DEC_STATUS status;
  HANDLE_DRC_GAIN_DECODER hGainDec;
  HANDLE_DRC_SELECTION_PROCESS hSelectionProc;
  int selProcInputDiff;
  UNI_DRC_CONFIG uniDrcConfig;
  LOUDNESS_INFO_SET loudnessInfoSet;
  UNI_DRC_GAIN uniDrcGain;
  SEL_PROC_OUTPUT selProcOutput;
};

/* standalone reader state */
struct shim_reader {
  UNI_DRC_CONFIG cfg;
  LOUDNESS_INFO_SET lis;
  UNI_DRC_GAIN gain;
};

#define P(x) (out[n++] = (INT)(x))

static int ser_gain_set(const GAIN_SET* g, INT* out, int n) {
  int i;
  P(g->gainCodingProfile);
  P(g->gainInterpolationType);
  P(g->fullFrame);
  P(g->timeAlignment);
  P(g->timeDeltaMinPresent);
  P(g->timeDeltaMin);
  P(g->bandCount);
  P(g->drcBandType);
  for (i = 0; i < 4; i++) P(g->gainSequenceIndex[i]);
  for (i = 0; i < 4; i++) {
    P(g->drcCharacteristic[i].present);
    P(g->drcCharacteristic[i].isCICP);
    P(g->drcCharacteristic[i].custom.left);
    P(g->drcCharacteristic[i].custom.right);
  }
  for (i = 0; i < 4; i++) P(g->bandBorder[i].startSubBandIndex);
  return n;
}

static int ser_coef(const DRC_COEFFICIENTS_UNI_DRC* c, INT* out, int n) {
  int i;
  P(c->drcLocation);
  P(c->drcFrameSizePresent);
  P(c->drcFrameSize);
  P(c->gainSequenceCount);
  P(c->gainSetCount);
  for (i = 0; i < 4; i++) P(c->streamGainSequenceCount[i]);
  for (i = 0; i < 4; i++) P(c->streamGainSetCount[i]);
  for (i = 0; i < 48; i++) n = ser_gain_set(&c->gainSet[i], out, n);
  for (i = 0; i < 48; i++) P(c->gainSetIndexForGainSequence[i]);
  return n;
}

static int ser_inst(const DRC_INSTRUCTIONS_UNI_DRC* p, INT* out, int n) {
  int i;
  const SHORT* u = (const SHORT*)&p->gainModificationForChannelGroup[0][0];
  P(p->drcInstructionsType);
  P(p->mae_groupID);
  P(p->drcSetId);
  P(p->drcSetComplexityLevel);
  P(p->drcLocation);
  P(p->drcApplyToDownmix);
  P(p->downmixIdCount);
  for (i = 0; i < 8; i++) P(p->downmixId[i]);
  P(p->drcSetEffect);
  P(p->limiterPeakTargetPresent);
  P(p->limiterPeakTarget);
  P(p->drcSetTargetLoudnessPresent);
  P(p->drcSetTargetLoudnessValueUpper);
  P(p->drcSetTargetLoudnessValueLower);
  P(p->dependsOnDrcSetPresent);
  P(p->dependsOnDrcSet);
  P(p->requiresEq);
  for (i = 0; i < 28 * 4 * 3; i++) P(u[i]);
  for (i = 0; i < 2 * 28; i++) P(p->gainSetIndex[i]);
  P(p->drcChannelCount);
  for (i = 0; i < 4; i++) P(p->streamNDrcChannelGroups[i]);
  P(p->nDrcChannelGroups);
  for (i = 0; i < 28; i++) P(p->gainSetIndexForChannelGroup[i]);
  return n;
}

static int ser_config(const UNI_DRC_CONFIG* c, INT* out, int n) {
  int i;
  P(c->downmixInstructionsCountV0);
  P(c->downmixInstructionsCountV1);
  P(c->downmixInstructionsCount);
  for (i = 0; i < 4; i++) P(c->drcCoefficientsUniDrcCountV0[i]);
  P(c->drcCoefficientsUniDrcCountV1);
  P(c->drcCoefficientsUniDrcCount);
  for (i = 0; i < 4; i++) P(c->drcInstructionsUniDrcCountV0[i]);
  P(c->drcInstructionsUniDrcCountV1);
  P(c->drcInstructionsUniDrcCount);
  for (i = 0; i < 4; i++) P(c->streamChannelCount[i]);
  P(c->channelLayout.baseChannelCount);
  P(c->channelLayout.layoutSignalingPresent);
  for (i = 0; i < 32; i++) {
    P(c->downmixInstructions[i].downmixId);
    P(c->downmixInstructions[i].targetChannelCount);
    P(c->downmixInstructions[i].targetLayout);
    P(c->downmixInstructions[i].downmixCoefficientsPresent);
  }
  for (i = 0; i < 2; i++) n = ser_coef(&c->drcCoefficientsUniDrc[i], out, n);
  for (i = 0; i < 32 + 1; i++) n = ser_inst(&c->drcInstructionsUniDrc[i], out, n);
  P(c->loudnessInfoSetPresent);
  P(c->uniDrcConfigExtPresent);
  for (i = 0; i < 8; i++) P(c->uniDrcConfigExt.uniDrcConfigExtType[i]);
  for (i = 0; i < 7; i++) P(c->uniDrcConfigExt.extBitSize[i]);
  P(c->drcInstructionsCountInclVirtual);
  P(c->diff);
  return n;
}

static int ser_loudness_info(const LOUDNESS_INFO* l, INT* out, int n) {
  int i;
  P(l->loudnessInfoType);
  P(l->mae_groupID);
  P(l->drcSetId);
  P(l->downmixId);
  P(l->samplePeakLevelPresent);
  P(l->truePeakLevelPresent);
  P(l->measurementCount);
  P(l->samplePeakLevel);
  P(l->truePeakLevel);
  for (i = 0; i < 16; i++) {
    P(l->loudnessMeasurement[i].methodValue);
    P(l->loudnessMeasurement[i].methodDefinition);
    P(l->loudnessMeasurement[i].measurementSystem);
    P(l->loudnessMeasurement[i].reliability);
  }
  return n;
}

static int ser_loudness_set(const LOUDNESS_INFO_SET* s, INT* out, int n) {
  int i;
  for (i = 0; i < 4; i++) P(s->loudnessInfoAlbumCountV0[i]);
  P(s->loudnessInfoAlbumCountV1);
  P(s->loudnessInfoAlbumCount);
  for (i = 0; i < 4; i++) P(s->loudnessInfoCountV0[i]);
  P(s->loudnessInfoCountV1);
  P(s->loudnessInfoCount);
  for (i = 0; i < 32; i++) n = ser_loudness_info(&s->loudnessInfoAlbum[i], out, n);
  for (i = 0; i < 32; i++) n = ser_loudness_info(&s->loudnessInfo[i], out, n);
  P(s->loudnessInfoSetExtPresent);
  for (i = 0; i < 8; i++) P(s->loudnessInfoSetExt.loudnessInfoSetExtType[i]);
  for (i = 0; i < 7; i++) P(s->loudnessInfoSetExt.extBitSize[i]);
  P(s->diff);
  return n;
}

static int ser_gain(const UNI_DRC_GAIN* g, INT* out, int n) {
  int i, j;
  for (i = 0; i < 48; i++) P(g->nNodes[i]);
  for (i = 0; i < 48; i++)
    for (j = 0; j < 32; j++) {
      P(g->gainNode[i][j].gainDb);
      P(g->gainNode[i][j].time);
    }
  P(g->uniDrcGainExtPresent);
  for (i = 0; i < 8; i++) P(g->uniDrcGainExtension.uniDrcGainExtType[i]);
  for (i = 0; i < 7; i++) P(g->uniDrcGainExtension.extBitSize[i]);
  for (i = 0; i < 4; i++) P(g->nDecodedSequences[i]);
  P(g->status);
  return n;
}

static int ser_sel_proc_output(const SEL_PROC_OUTPUT* o, INT* out, int n) {
  int i;
  P(o->outputPeakLevelDb);
  P(o->loudnessNormalizationGainDb);
  P(o->outputLoudness);
  P(o->numSelectedDrcSets);
  for (i = 0; i < 3; i++) P(o->selectedDrcSetIds[i]);
  for (i = 0; i < 3; i++) P(o->selectedDownmixIds[i]);
  P(o->activeDownmixId);
  P(o->baseChannelCount);
  P(o->targetChannelCount);
  P(o->targetLayout);
  P(o->downmixMatrixPresent);
  P(o->boost);
  P(o->compress);
  P(o->groupIdLoudnessCount);
  for (i = 0; i < 32; i++) P(o->groupId[i]);
  for (i = 0; i < 32; i++) P(o->groupIdLoudness[i]);
  P(o->mixingLevel);
  return n;
}

static int ser_lnb(const LINEAR_NODE_BUFFER* l, INT* out, int n) {
  int i, j;
  P(l->gainInterpolationType);
  for (i = 0; i < NUM_LNB_FRAMES; i++) P(l->nNodes[i]);
  for (i = 0; i < NUM_LNB_FRAMES; i++)
    for (j = 0; j < 32; j++) P(l->linearNodeGain[i][j]);
  for (i = 0; i < NUM_LNB_FRAMES; i++)
    for (j = 0; j < 32; j++) P(l->linearNodeTime[i][j]);
  return n;
}

static int ser_gain_dec(const DRC_GAIN_DECODER* h, INT* out, int n, int withOverlap) {
  int i, j, k, l;
  P(h->deltaTminDefault);
  P(h->frameSize);
  P(h->sampleRate);
  P(h->msPerFrame);
  P(h->loudnessNormalisationGainDb);
  P(h->delayMode);
  n = ser_coef(&h->drcCoef, out, n);
  for (l = 0; l < 2; l++) P(h->nActiveDrcs[l]);
  for (l = 0; l < 2; l++)
    for (i = 0; i < 3; i++) {
      const ACTIVE_DRC* a = &h->activeDrc[l][i];
      P(a->activeDrcOffset);
      n = ser_inst(&a->drcInst, out, n);
      for (j = 0; j < 28; j++) P(a->duckingModificationForChannelGroup[j].duckingScaling);
      for (j = 0; j < 56; j++) P(a->channelGroupForChannel[j]);
      for (j = 0; j < 28; j++) P(a->bandCountForChannelGroup[j]);
      for (j = 0; j < 28; j++) P(a->gainElementForGroup[j]);
      for (j = 0; j < 28; j++) P(a->channelGroupIsParametricDrc[j]);
      P(a->gainElementCount);
      for (j = 0; j < 28; j++)
        for (k = 0; k < NUM_LNB_FRAMES; k++) P(a->lnbIndexForChannel[j][k]);
      P(a->subbandGainsReady);
    }
  P(h->multiBandActiveDrcIndex);
  P(h->channelGainActiveDrcIndex);
  for (j = 0; j < 28; j++) P(h->channelGain[j]);
  for (j = 0; j < 28; j++) P(h->channelGainPrev[j]);
  P(h->drcGainBuffers.lnbPointer);
  for (i = 0; i < 48; i++) n = ser_lnb(&h->drcGainBuffers.linearNodeBuffer[i], out, n);
  n = ser_lnb(&h->drcGainBuffers.dummyLnb, out, n);
  if (withOverlap) {
    for (i = 0; i < 28; i++)
      for (j = 0; j < 4; j++) {
        P(h->overlap[i][j].start_index);
        P(h->overlap[i][j].stop_index);
        for (k = 0; k < MAX_NUM_SUBBANDS; k++) P(h->overlap[i][j].overlapWeight[k]);
      }
  }
  for (i = 0; i < 48; i++)
    for (j = 0; j < 16; j++) P(h->subbandGains[i][j]);
  for (j = 0; j < 16; j++) P(h->dummySubbandGains[j]);
  P(h->status);
  P(h->timeDomainSupported);
  P(h->startupMs);
  P(h->subbandDomainSupported);
  return n;
}

extern "C" {

/* ---- standalone readers ---- */

void* cref_drc_reader_new() {
  shim_reader* r = (shim_reader*)FDKcalloc(1, sizeof(shim_reader));
  UINT sz = sizeof(DRC_INSTRUCTIONS_UNI_DRC);
  if (sizeof(DRC_COEFFICIENTS_UNI_DRC) > sz) sz = sizeof(DRC_COEFFICIENTS_UNI_DRC);
  if (sizeof(DOWNMIX_INSTRUCTIONS) > sz) sz = sizeof(DOWNMIX_INSTRUCTIONS);
  r->cfg.p_scratch = (UINT*)FDKcalloc(1, sz);
  return r;
}
void cref_drc_reader_free(void* h) {
  shim_reader* r = (shim_reader*)h;
  FDKfree(r->cfg.p_scratch);
  FDKfree(r);
}
INT cref_drc_reader_config(void* h, UCHAR* buf, UINT size, UINT validBits, INT sub, INT* bitsLeft) {
  shim_reader* r = (shim_reader*)h;
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = drcDec_readMpegh3daUniDrcConfig(&bs, &r->cfg, &r->lis, sub);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_drc_reader_loudness(void* h, UCHAR* buf, UINT size, UINT validBits, INT sub, INT* bitsLeft) {
  shim_reader* r = (shim_reader*)h;
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = drcDec_readMpegh3daLoudnessInfoSet(&bs, &r->lis, sub);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_drc_reader_gain(void* h, UCHAR* buf, UINT size, UINT validBits, INT coefIndex, INT frameSize, INT deltaTminDefault, INT sub, INT* bitsLeft) {
  shim_reader* r = (shim_reader*)h;
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  DRC_COEFFICIENTS_UNI_DRC* pCoef = (coefIndex >= 0) ? &r->cfg.drcCoefficientsUniDrc[coefIndex] : NULL;
  INT err = drcDec_readUniDrcGain(&bs, pCoef, frameSize, deltaTminDefault, sub, &r->gain);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_drc_reader_ser(void* h, INT* out) {
  shim_reader* r = (shim_reader*)h;
  int n = 0;
  n = ser_config(&r->cfg, out, n);
  n = ser_loudness_set(&r->lis, out, n);
  n = ser_gain(&r->gain, out, n);
  return n;
}

/* ---- decoder API ---- */

void* cref_drcdec_open(INT range) {
  HANDLE_DRC_DECODER h = NULL;
  if (FDK_drcDec_Open(&h, (DRC_DEC_FUNCTIONAL_RANGE)range) != DRC_DEC_OK) return NULL;
  return h;
}
void cref_drcdec_close(void* h) {
  HANDLE_DRC_DECODER d = (HANDLE_DRC_DECODER)h;
  FDK_drcDec_Close(&d);
}
INT cref_drcdec_set_codec_mode(void* h, INT mode) { return FDK_drcDec_SetCodecMode((HANDLE_DRC_DECODER)h, (DRC_DEC_CODEC_MODE)mode); }
INT cref_drcdec_init(void* h, INT frameSize, INT sampleRate, INT baseChannelCount) { return FDK_drcDec_Init((HANDLE_DRC_DECODER)h, frameSize, sampleRate, baseChannelCount); }
INT cref_drcdec_set_param(void* h, INT type, INT value) { return FDK_drcDec_SetParam((HANDLE_DRC_DECODER)h, (DRC_DEC_USERPARAM)type, (FIXP_DBL)value); }
INT cref_drcdec_get_param(void* h, INT type) { return (INT)FDK_drcDec_GetParam((HANDLE_DRC_DECODER)h, (DRC_DEC_USERPARAM)type); }
INT cref_drcdec_set_mpegh_params(void* h, INT nG, const INT* g, INT nP, const INT* p, const INT* nm, INT pref) { return FDK_drcDec_SetSelectionProcessMpeghParameters((HANDLE_DRC_DECODER)h, nG, g, nP, p, nm, pref); }
INT cref_drcdec_set_mpegh_params_simple(void* h, INT preset, INT nG, const INT* g) { return FDK_drcDec_SetSelectionProcessMpeghParameters_simple((HANDLE_DRC_DECODER)h, preset, nG, g); }
INT cref_drcdec_set_downmix_instructions(void* h, INT n, const INT* id, const INT* layout, const INT* count) { return FDK_drcDec_SetDownmixInstructions((HANDLE_DRC_DECODER)h, n, id, layout, count); }
INT cref_drcdec_get_group_loudness(void* h, INT groupId, INT* avail) { return (INT)FDK_drcDec_GetGroupLoudness(FDK_drcDec_GetSelectionProcessOutput((HANDLE_DRC_DECODER)h), groupId, avail); }
void cref_drcdec_set_channel_gains(void* h, INT upd, INT nch, INT frameSize, INT* gains, INT* audio, INT offset) { FDK_drcDec_SetChannelGains((HANDLE_DRC_DECODER)h, upd, nch, frameSize, (FIXP_DBL*)gains, (FIXP_DBL*)audio, offset); }
INT cref_drcdec_read_config(void* h, UCHAR* buf, UINT size, UINT validBits, INT sub, INT* bitsLeft) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = FDK_drcDec_ReadUniDrcConfig((HANDLE_DRC_DECODER)h, &bs, sub);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_drcdec_read_loudness(void* h, UCHAR* buf, UINT size, UINT validBits, INT sub, INT* bitsLeft) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = FDK_drcDec_ReadLoudnessInfoSet((HANDLE_DRC_DECODER)h, &bs, sub);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_drcdec_read_gain(void* h, UCHAR* buf, UINT size, UINT validBits, INT sub, INT* bitsLeft) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = FDK_drcDec_ReadUniDrcGain((HANDLE_DRC_DECODER)h, &bs, sub);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_drcdec_preprocess(void* h) { return FDK_drcDec_Preprocess((HANDLE_DRC_DECODER)h); }
INT cref_drcdec_process_time(void* h, INT delay, INT loc, INT chOff, INT drcChOff, INT n, INT* buf, INT stride) {
  return FDK_drcDec_ProcessTime((HANDLE_DRC_DECODER)h, delay, (DRC_DEC_LOCATION)loc, chOff, drcChOff, n, (FIXP_DBL*)buf, stride);
}
/* real/imag: numChannels buffers of `stride` values each, stored contiguously; imag may be NULL */
INT cref_drcdec_process_freq(void* h, INT delay, INT loc, INT chOff, INT drcChOff, INT n, INT single, INT* real, INT* imag, INT stride) {
  FIXP_DBL* r[28];
  FIXP_DBL* i[28];
  for (int c = 0; c < n && c < 28; c++) {
    r[c] = (FIXP_DBL*)real + c * stride;
    if (imag) i[c] = (FIXP_DBL*)imag + c * stride;
  }
  return FDK_drcDec_ProcessFreq((HANDLE_DRC_DECODER)h, delay, (DRC_DEC_LOCATION)loc, chOff, drcChOff, n, single, r, imag ? i : NULL);
}
INT cref_drcdec_status(void* h) {
  shim_drc_decoder* d = (shim_drc_decoder*)h;
  return (INT)d->status * 1000 + (INT)d->selProcInputDiff;
}
/* what: bit 0 config+loudness+gain, bit 1 sel proc output, bit 2 gain decoder, bit 3 overlap */
INT cref_drcdec_ser(void* h, INT what, INT* out) {
  shim_drc_decoder* d = (shim_drc_decoder*)h;
  int n = 0;
  P(d->codecMode);
  P(d->functionalRange);
  P(d->status);
  P(d->selProcInputDiff);
  if (what & 1) {
    n = ser_config(&d->uniDrcConfig, out, n);
    n = ser_loudness_set(&d->loudnessInfoSet, out, n);
    n = ser_gain(&d->uniDrcGain, out, n);
  }
  if (what & 2) n = ser_sel_proc_output(&d->selProcOutput, out, n);
  if ((what & 4) && d->hGainDec) n = ser_gain_dec(d->hGainDec, out, n, what & 8);
  return n;
}

}  // extern "C"
