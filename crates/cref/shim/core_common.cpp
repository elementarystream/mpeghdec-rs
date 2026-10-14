// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Common C shims for the libMpeghDec differential tests: allocation of the per channel and per
// element structures of the core decoder, wired like CAacDecoder_Init() does, and conversion of
// their state from/to INT arrays. The array layout is mirrored by
// crates/cref/tests/core_common/mod.rs.

#include "core_common.h"

extern "C" INT cref_rom_table_id(const SHORT* p);

static void dump_ics(const CIcsInfo* ics, INT* out, INT& n) {
  DUMP_ARR(ics->WindowGroupLength, 8);
  DUMP(ics->WindowGroups);
  DUMP(ics->Valid);
  DUMP(ics->WindowShape);
  DUMP(ics->WindowSequence);
  DUMP(ics->MaxSfBands);
  DUMP(ics->max_sfb_ste);
  DUMP(ics->ScaleFactorGrouping);
  DUMP(ics->TotalSfBands);
}

static void load_ics(CIcsInfo* ics, const INT* in, INT& n) {
  LOAD_ARR(ics->WindowGroupLength, 8, UCHAR);
  LOAD(ics->WindowGroups, UCHAR);
  LOAD(ics->Valid, UCHAR);
  LOAD(ics->WindowShape, UCHAR);
  LOAD(ics->WindowSequence, BLOCK_TYPE);
  LOAD(ics->MaxSfBands, UCHAR);
  LOAD(ics->max_sfb_ste, UCHAR);
  LOAD(ics->ScaleFactorGrouping, UCHAR);
  LOAD(ics->TotalSfBands, UCHAR);
}

static void dump_tns(const CTnsData* t, INT* out, INT& n) {
  for (int w = 0; w < TNS_MAX_WINDOWS; w++) {
    for (int f = 0; f < TNS_MAXIMUM_FILTERS; f++) {
      const CFilter* fi = &t->Filter[w][f];
      DUMP_ARR(fi->Coeff, TNS_MAXIMUM_ORDER);
      DUMP(fi->StartBand);
      DUMP(fi->StopBand);
      DUMP(fi->Direction);
      DUMP(fi->Resolution);
      DUMP(fi->Order);
    }
  }
  DUMP_ARR(t->NumberOfFilters, TNS_MAX_WINDOWS);
  DUMP(t->DataPresent);
  DUMP(t->Active);
  DUMP(t->GainLd);
}

static void load_tns(CTnsData* t, const INT* in, INT& n) {
  for (int w = 0; w < TNS_MAX_WINDOWS; w++) {
    for (int f = 0; f < TNS_MAXIMUM_FILTERS; f++) {
      CFilter* fi = &t->Filter[w][f];
      LOAD_ARR(fi->Coeff, TNS_MAXIMUM_ORDER, SCHAR);
      LOAD(fi->StartBand, UCHAR);
      LOAD(fi->StopBand, UCHAR);
      LOAD(fi->Direction, SCHAR);
      LOAD(fi->Resolution, SCHAR);
      LOAD(fi->Order, UCHAR);
    }
  }
  LOAD_ARR(t->NumberOfFilters, TNS_MAX_WINDOWS, UCHAR);
  LOAD(t->DataPresent, UCHAR);
  LOAD(t->Active, UCHAR);
  LOAD(t->GainLd, UCHAR);
}

static void dump_conceal(const CConcealmentInfo* c, INT* out, INT& n) {
  DUMP_ARR(c->spectralCoefficient, 1024);
  DUMP_ARR(c->specScale, 8);
  DUMP(c->iRandomPhase);
  DUMP_ARR(c->prevFrameOk, 2);
  DUMP(c->cntValidFrames);
  DUMP(c->cntFadeFrames);
  DUMP_ARR(c->winGrpOffset, 2);
  DUMP_ARR(c->attGrpOffset, 2);
  DUMP(c->lastRenderMode);
  DUMP(c->windowShape);
  DUMP(c->windowSequence);
  DUMP(c->lastWinGrpLen);
  DUMP(c->concealState);
  DUMP(c->concealState_old);
  DUMP(c->fade_old);
  DUMP(c->lastFadingType);
  DUMP(c->lastWindowGroups);
  DUMP_ARR(c->lastWindowGroupLength, 8);
  DUMP(c->TDNoiseSeed);
  DUMP_ARR(c->TDNoiseStates, 3);
  DUMP_ARR(c->TDNoiseCoef, 3);
  DUMP(c->TDNoiseAtt);
}

static void load_conceal(CConcealmentInfo* c, const INT* in, INT& n) {
  LOAD_ARR(c->spectralCoefficient, 1024, FIXP_DBL);
  LOAD_ARR(c->specScale, 8, SHORT);
  LOAD(c->iRandomPhase, INT);
  LOAD_ARR(c->prevFrameOk, 2, INT);
  LOAD(c->cntValidFrames, INT);
  LOAD(c->cntFadeFrames, INT);
  LOAD_ARR(c->winGrpOffset, 2, int);
  LOAD_ARR(c->attGrpOffset, 2, int);
  LOAD(c->lastRenderMode, SCHAR);
  LOAD(c->windowShape, UCHAR);
  LOAD(c->windowSequence, BLOCK_TYPE);
  LOAD(c->lastWinGrpLen, UCHAR);
  LOAD(c->concealState, CConcealmentState);
  LOAD(c->concealState_old, CConcealmentState);
  LOAD(c->fade_old, FIXP_DBL);
  LOAD(c->lastFadingType, TDfadingType);
  LOAD(c->lastWindowGroups, UCHAR);
  LOAD_ARR(c->lastWindowGroupLength, 8, UCHAR);
  LOAD(c->TDNoiseSeed, ULONG);
  LOAD_ARR(c->TDNoiseStates, 3, FIXP_DBL);
  LOAD_ARR(c->TDNoiseCoef, 3, FIXP_SGL);
  LOAD(c->TDNoiseAtt, FIXP_SGL);
}

extern "C" {

/* ---------------------------------------------------------------------------------------------
   CAacDecoderChannelInfo (+ pDynData)
   --------------------------------------------------------------------------------------------- */

void* cref_core_ch_new(void) {
  CREF_CH* c = (CREF_CH*)FDKcalloc(1, sizeof(CREF_CH));
  c->ch.pDynData = &c->dyn;
  return c;
}
void cref_core_ch_free(void* p) { FDKfree(p); }
/* Sets pSpectralCoefficient (memory owned by the caller). */
void cref_core_ch_set_spec(void* p, FIXP_DBL* spec) { ((CREF_CH*)p)->ch.pSpectralCoefficient = spec; }
/* Sets pComStaticData / pComData (element common data from cref_core_com_new()). */
void cref_core_ch_set_com(void* p, void* com) {
  ((CREF_CH*)p)->ch.pComStaticData = &((CREF_COM*)com)->com;
  ((CREF_CH*)p)->ch.pComData = NULL;
}
void* cref_core_ch_igf(void* p) { return &((CREF_CH*)p)->ch.IGFdata; }

/* Dump of CAacDecoderChannelInfo without pointers and IGFdata, followed by pDynData. */
INT cref_core_ch_dump(void* p, INT* out) {
  CAacDecoderChannelInfo* c = &((CREF_CH*)p)->ch;
  CAacDecoderDynamicData* d = c->pDynData;
  INT n = 0;
  DUMP_ARR(c->specScale, 8);
  dump_ics(&c->icsInfo, out, n);
  DUMP(c->granuleLength);
  DUMP(c->ElementInstanceTag);
  DUMP(c->renderMode);
  DUMP(c->currAliasingSymmetry);
  DUMP(c->transform_splitting_active);
  DUMP(c->fdp_data_present);
  DUMP(c->fdp_spacing_index);
  DUMP_ARR(d->aScaleFactor, 8 * 16);
  DUMP_ARR(d->aSfbScale, 8 * 16);
  DUMP_ARR(d->aCodeBook, 8 * 16);
  DUMP_ARR(d->band_is_noise, 8 * 16);
  dump_tns(&d->TnsData, out, n);
  DUMP(d->RawDataInfo.CommonWindow);
  DUMP(d->RawDataInfo.GlobalGain);
  DUMP(d->specificTo.usac.fd_noise_level_and_offset);
  DUMP(d->specificTo.usac.tns_active);
  return n;
}

INT cref_core_ch_load(void* p, const INT* in) {
  CAacDecoderChannelInfo* c = &((CREF_CH*)p)->ch;
  CAacDecoderDynamicData* d = c->pDynData;
  INT n = 0;
  LOAD_ARR(c->specScale, 8, SHORT);
  load_ics(&c->icsInfo, in, n);
  LOAD(c->granuleLength, INT);
  LOAD(c->ElementInstanceTag, UCHAR);
  LOAD(c->renderMode, AACDEC_RENDER_MODE);
  LOAD(c->currAliasingSymmetry, int);
  LOAD(c->transform_splitting_active, UCHAR);
  LOAD(c->fdp_data_present, int);
  LOAD(c->fdp_spacing_index, int);
  LOAD_ARR(d->aScaleFactor, 8 * 16, SHORT);
  LOAD_ARR(d->aSfbScale, 8 * 16, SHORT);
  LOAD_ARR(d->aCodeBook, 8 * 16, UCHAR);
  LOAD_ARR(d->band_is_noise, 8 * 16, UCHAR);
  load_tns(&d->TnsData, in, n);
  LOAD(d->RawDataInfo.CommonWindow, UCHAR);
  LOAD(d->RawDataInfo.GlobalGain, UCHAR);
  LOAD(d->specificTo.usac.fd_noise_level_and_offset, UCHAR);
  LOAD(d->specificTo.usac.tns_active, UCHAR);
  return n;
}

/* ---------------------------------------------------------------------------------------------
   CAacDecoderStaticChannelInfo
   --------------------------------------------------------------------------------------------- */

/* Allocates the static channel info with concealment buffer, overlap buffer (mdct_init()) and
   arithmetic decoder data. */
void* cref_core_st_new(void) {
  CREF_ST* s = (CREF_ST*)FDKcalloc(1, sizeof(CREF_ST));
  s->st.concealmentInfo.spectralCoefficient = s->concealSpec;
  s->st.pOverlapBuffer = s->overlap;
  s->st.hArCo = &s->arco;
  mdct_init(&s->st.IMdct, s->st.pOverlapBuffer, OverlapBufferSize);
  return s;
}
void cref_core_st_free(void* p) { FDKfree(p); }
void* cref_core_st_igf(void* p) { return &((CREF_ST*)p)->st.IGF_StaticData; }
/* Sets pCpeStaticData (from cref_core_cpe_new()) or NULL. */
void cref_core_st_set_cpe(void* p, void* cpe) {
  ((CREF_ST*)p)->st.pCpeStaticData = (CpePersistentData*)cpe;
}

/* Dump of CAacDecoderStaticChannelInfo without IGF_StaticData: IMdct state (prev_wrs as ROM
   table id), overlap buffer, hArCo, nfRandomSeed, concealmentInfo, LTP and FDP data. */
INT cref_core_st_dump(void* p, INT* out) {
  CAacDecoderStaticChannelInfo* s = &((CREF_ST*)p)->st;
  INT n = 0;
  DUMP(s->IMdct.prev_tl);
  DUMP(s->IMdct.prev_nr);
  DUMP(s->IMdct.prev_fr);
  DUMP(s->IMdct.ov_offset);
  DUMP(s->IMdct.ov_size);
  DUMP(s->IMdct.prevAliasSymmetry);
  DUMP(s->IMdct.prevPrevAliasSymmetry);
  DUMP(cref_rom_table_id((const SHORT*)s->IMdct.prev_wrs));
  DUMP_ARR(s->pOverlapBuffer, OverlapBufferSize);
  DUMP(s->hArCo->m_numberLinesPrev);
  DUMP_ARR(s->hArCo->c_prev, (1024 / 2) + 4);
  DUMP(s->nfRandomSeed);
  dump_conceal(&s->concealmentInfo, out, n);
  DUMP_ARR(s->ltp_param, 3);
  DUMP(s->ltp_pitch_int_past);
  DUMP(s->ltp_pitch_fr_past);
  DUMP(s->ltp_gain_past);
  DUMP(s->ltp_gainIdx_past);
  DUMP_ARR(s->ltp_mem_in, LTP_MEM_IN_SIZE);
  DUMP_ARR(s->ltp_mem_out, LTP_MEM_OUT_SIZE);
  DUMP_ARR(s->quantSpecPrev1, 160);
  DUMP_ARR(s->quantSpecPrev2, 160);
  DUMP(s->prevWindowShape);
  return n;
}

/* Loads the same layout as cref_core_st_dump(); the ROM table id of prev_wrs is ignored (the
   window slope pointer is not changed). */
INT cref_core_st_load(void* p, const INT* in) {
  CAacDecoderStaticChannelInfo* s = &((CREF_ST*)p)->st;
  INT n = 0;
  LOAD(s->IMdct.prev_tl, int);
  LOAD(s->IMdct.prev_nr, int);
  LOAD(s->IMdct.prev_fr, int);
  LOAD(s->IMdct.ov_offset, int);
  LOAD(s->IMdct.ov_size, int);
  LOAD(s->IMdct.prevAliasSymmetry, int);
  LOAD(s->IMdct.prevPrevAliasSymmetry, int);
  n++;
  LOAD_ARR(s->pOverlapBuffer, OverlapBufferSize, FIXP_DBL);
  LOAD(s->hArCo->m_numberLinesPrev, SHORT);
  LOAD_ARR(s->hArCo->c_prev, (1024 / 2) + 4, UCHAR);
  LOAD(s->nfRandomSeed, ULONG);
  load_conceal(&s->concealmentInfo, in, n);
  LOAD_ARR(s->ltp_param, 3, int);
  LOAD(s->ltp_pitch_int_past, int);
  LOAD(s->ltp_pitch_fr_past, int);
  LOAD(s->ltp_gain_past, FIXP_SGL);
  LOAD(s->ltp_gainIdx_past, int);
  LOAD_ARR(s->ltp_mem_in, LTP_MEM_IN_SIZE, FIXP_SGL);
  LOAD_ARR(s->ltp_mem_out, LTP_MEM_OUT_SIZE, FIXP_SGL);
  LOAD_ARR(s->quantSpecPrev1, 160, short);
  LOAD_ARR(s->quantSpecPrev2, 160, short);
  LOAD(s->prevWindowShape, UCHAR);
  return n;
}

/* ---------------------------------------------------------------------------------------------
   CConcealParams
   --------------------------------------------------------------------------------------------- */

void* cref_core_conceal_params_new(void) { return FDKcalloc(1, sizeof(CConcealParams)); }
void cref_core_conceal_params_free(void* p) { FDKfree(p); }
/* Sets concealmentInfo.pConcealParams of a static channel info. */
void cref_core_st_set_conceal_params(void* st, void* params) {
  ((CREF_ST*)st)->st.concealmentInfo.pConcealParams = (CConcealParams*)params;
}
INT cref_core_conceal_params_dump(void* p, INT* out) {
  CConcealParams* c = (CConcealParams*)p;
  INT n = 0;
  DUMP_ARR(c->fadeOutFactor, CONCEAL_MAX_NUM_FADE_FACTORS);
  DUMP_ARR(c->fadeInFactor, CONCEAL_MAX_NUM_FADE_FACTORS);
  DUMP(c->method);
  DUMP(c->numFadeOutFrames);
  DUMP(c->numFadeInFrames);
  DUMP(c->numMuteReleaseFrames);
  DUMP(c->comfortNoiseLevel);
  return n;
}
INT cref_core_conceal_params_load(void* p, const INT* in) {
  CConcealParams* c = (CConcealParams*)p;
  INT n = 0;
  LOAD_ARR(c->fadeOutFactor, CONCEAL_MAX_NUM_FADE_FACTORS, FIXP_SGL);
  LOAD_ARR(c->fadeInFactor, CONCEAL_MAX_NUM_FADE_FACTORS, FIXP_SGL);
  LOAD(c->method, CConcealmentMethod);
  LOAD(c->numFadeOutFrames, int);
  LOAD(c->numFadeInFrames, int);
  LOAD(c->numMuteReleaseFrames, int);
  LOAD(c->comfortNoiseLevel, FIXP_DBL);
  return n;
}

/* ---------------------------------------------------------------------------------------------
   Element data: pComStaticData (CJointStereoData, CCplxPredictionData, pWorkBufferCore1) and
   CpePersistentData
   --------------------------------------------------------------------------------------------- */

/* Allocates pComStaticData with pJointStereoData (if hasJsd) and cplxPredictionData (if hasCpd).
   pWorkBufferCore1 is set to wb (memory owned by the caller, WB_SECTION_SIZE values). */
void* cref_core_com_new(INT hasJsd, INT hasCpd, FIXP_DBL* wb) {
  CREF_COM* c = (CREF_COM*)FDKcalloc(1, sizeof(CREF_COM));
  c->com.pJointStereoData = hasJsd ? &c->jsd : NULL;
  c->com.cplxPredictionData = hasCpd ? &c->cpd : NULL;
  c->com.pWorkBufferCore1 = (CWorkBufferCore1*)wb;
  return c;
}
void cref_core_com_free(void* p) { FDKfree(p); }

INT cref_core_jsd_dump(void* p, INT* out) {
  CJointStereoData* j = &((CREF_COM*)p)->jsd;
  INT n = 0;
  DUMP(j->MsMaskPresent);
  DUMP_ARR(j->MsUsed, JointStereoMaximumBands);
  DUMP(j->cplx_pred_flag);
  DUMP(j->IGF_MsMaskPresent);
  DUMP(j->igf_cplx_pred_flag);
  DUMP_ARR(j->store_dmx_re_prev, 1024);
  DUMP(j->store_dmx_re_prev_e);
  return n;
}
INT cref_core_jsd_load(void* p, const INT* in) {
  CJointStereoData* j = &((CREF_COM*)p)->jsd;
  INT n = 0;
  LOAD(j->MsMaskPresent, UCHAR);
  LOAD_ARR(j->MsUsed, JointStereoMaximumBands, UCHAR);
  LOAD(j->cplx_pred_flag, UCHAR);
  LOAD(j->IGF_MsMaskPresent, UCHAR);
  LOAD(j->igf_cplx_pred_flag, UCHAR);
  LOAD_ARR(j->store_dmx_re_prev, 1024, FIXP_DBL);
  LOAD(j->store_dmx_re_prev_e, SHORT);
  return n;
}

INT cref_core_cpd_dump(void* p, INT* out) {
  CCplxPredictionData* c = &((CREF_COM*)p)->cpd;
  INT n = 0;
  DUMP(c->pred_dir);
  DUMP(c->igf_pred_dir);
  DUMP(c->complex_coef);
  DUMP(c->use_prev_frame);
  for (int g = 0; g < JointStereoMaximumGroups; g++) DUMP_ARR(c->alpha_q_re[g], JointStereoMaximumBands);
  for (int g = 0; g < JointStereoMaximumGroups; g++) DUMP_ARR(c->alpha_q_im[g], JointStereoMaximumBands);
  return n;
}
INT cref_core_cpd_load(void* p, const INT* in) {
  CCplxPredictionData* c = &((CREF_COM*)p)->cpd;
  INT n = 0;
  LOAD(c->pred_dir, UCHAR);
  LOAD(c->igf_pred_dir, UCHAR);
  LOAD(c->complex_coef, UCHAR);
  LOAD(c->use_prev_frame, UCHAR);
  for (int g = 0; g < JointStereoMaximumGroups; g++) LOAD_ARR(c->alpha_q_re[g], JointStereoMaximumBands, SHORT);
  for (int g = 0; g < JointStereoMaximumGroups; g++) LOAD_ARR(c->alpha_q_im[g], JointStereoMaximumBands, SHORT);
  return n;
}

/* CpePersistentData; the pointers of jointStereoPersistentData are set by
   cref_core_cpe_wire(). */
void* cref_core_cpe_new(void) { return FDKcalloc(1, sizeof(CpePersistentData)); }
void cref_core_cpe_free(void* p) { FDKfree(p); }
/* Sets the pointers of jointStereoPersistentData like CAacDecoder_Init(): spectralCoeffs[ch2] and
   specScale[ch2] to the concealment data of the static channel infos st0/st1 (st1 may be NULL),
   scratchBuffer to wb (pWorkBufferCore1->workBuffer) and scratchBuffer2 to timeData2 + 2048. */
void cref_core_cpe_wire(void* p, void* st0, void* st1, FIXP_DBL* wb, FIXP_DBL* timeData2) {
  CJointStereoPersistentData* j = &((CpePersistentData*)p)->jointStereoPersistentData;
  j->spectralCoeffs[0] = ((CREF_ST*)st0)->st.concealmentInfo.spectralCoefficient;
  j->specScale[0] = ((CREF_ST*)st0)->st.concealmentInfo.specScale;
  if (st1 != NULL) {
    j->spectralCoeffs[1] = ((CREF_ST*)st1)->st.concealmentInfo.spectralCoefficient;
    j->specScale[1] = ((CREF_ST*)st1)->st.concealmentInfo.specScale;
  }
  j->scratchBuffer = wb;
  j->scratchBuffer2 = timeData2 + 2 * 1024;
}
INT cref_core_cpe_dump(void* p, INT* out) {
  CJointStereoPersistentData* j = &((CpePersistentData*)p)->jointStereoPersistentData;
  INT n = 0;
  DUMP(j->clearSpectralCoeffs);
  for (int g = 0; g < JointStereoMaximumGroups; g++) DUMP_ARR(j->alpha_q_re_prev[g], JointStereoMaximumBands);
  for (int g = 0; g < JointStereoMaximumGroups; g++) DUMP_ARR(j->alpha_q_im_prev[g], JointStereoMaximumBands);
  DUMP(j->winSeqPrev);
  DUMP(j->winShapePrev);
  DUMP(j->winGroupsPrev);
  return n;
}
INT cref_core_cpe_load(void* p, const INT* in) {
  CJointStereoPersistentData* j = &((CpePersistentData*)p)->jointStereoPersistentData;
  INT n = 0;
  LOAD(j->clearSpectralCoeffs, UCHAR);
  for (int g = 0; g < JointStereoMaximumGroups; g++) LOAD_ARR(j->alpha_q_re_prev[g], JointStereoMaximumBands, SHORT);
  for (int g = 0; g < JointStereoMaximumGroups; g++) LOAD_ARR(j->alpha_q_im_prev[g], JointStereoMaximumBands, SHORT);
  LOAD(j->winSeqPrev, UCHAR);
  LOAD(j->winShapePrev, UCHAR);
  LOAD(j->winGroupsPrev, UCHAR);
  return n;
}

}  // extern "C"
