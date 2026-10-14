// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for the libMpeghDec channel element functions (channel.cpp, block.cpp, stereo.cpp,
// aacdec_tns.cpp): a channel element context allocated and wired like CAacDecoder_Init() does,
// and wrappers of the element and block level functions.

#include "core_common.h"
#include "channel.h"
#include "block.h"
#include "stereo.h"
#include "aacdec_tns.h"
#include "FDK_igfDec.h"
#include "conceal.h"

/* Channel element context: the memory CAacDecoder_Init() assigns to the channels of one element
   (element channels ch = 0, 1). */
typedef struct {
  CREF_CH* ch[2];
  CREF_ST* st[2];
  CAacDecoderChannelInfo* pCh[2];
  CAacDecoderStaticChannelInfo* pSt[2];
  CREF_COM* com;
  CpePersistentData* cpe;
  FIXP_DBL spec[2 * 1024];     /* workBufferCore2 of the element's channels */
  FIXP_DBL wb[WB_SECTION_SIZE]; /* CWorkBufferCore1 */
  FIXP_DBL timeData2[3 * 1024]; /* pTimeData2 (virtualSpec, scratchBuffer2) */
  IGF_PRIVATE_DATA_COMMON igfCommon[2];
  SamplingRateInfo sri;
  CConcealParams conceal;
} CREF_EL;

extern "C" {

/* Allocates an element with nch channels (1 or 2) like CAacDecoder_Init(): pComStaticData with
   pWorkBufferCore1, pJointStereoData (CPE) and cplxPredictionData (CPE with cpPossible),
   pSpectralCoefficient, pCpeStaticData (cpPossible), IGF (enf != 0), nfRandomSeed, granuleLength,
   renderMode, mdct_init() and concealment init. Returns NULL if getSamplingRateInfo() fails. */
void* cref_core_el_new(INT nch, INT srIdx, UINT sampleRate, INT cpPossible, INT enf, INT igfStart, INT igfStop, INT igfHighRes, INT igfWhitening, INT igfAfterTns, INT igfIndepTiling) {
  CREF_EL* e = (CREF_EL*)FDKcalloc(1, sizeof(CREF_EL));
  if (getSamplingRateInfo(&e->sri, 1024, srIdx, sampleRate) != AAC_DEC_OK) {
    FDKfree(e);
    return NULL;
  }
  CConcealment_InitCommonData(&e->conceal);
  e->com = (CREF_COM*)FDKcalloc(1, sizeof(CREF_COM));
  e->com->com.pWorkBufferCore1 = (CWorkBufferCore1*)e->wb;
  if (nch == 2) {
    e->com->com.pJointStereoData = &e->com->jsd;
    if (cpPossible) e->com->com.cplxPredictionData = &e->com->cpd;
  }
  if (cpPossible) {
    e->cpe = (CpePersistentData*)FDKcalloc(1, sizeof(CpePersistentData));
    e->cpe->jointStereoPersistentData.winGroupsPrev = 1;
  }
  for (int ch = 0; ch < nch; ch++) {
    e->ch[ch] = (CREF_CH*)FDKcalloc(1, sizeof(CREF_CH));
    e->ch[ch]->ch.pDynData = &e->ch[ch]->dyn;
    e->st[ch] = (CREF_ST*)FDKcalloc(1, sizeof(CREF_ST));
    e->pCh[ch] = &e->ch[ch]->ch;
    e->pSt[ch] = &e->st[ch]->st;
    CAacDecoderChannelInfo* c = e->pCh[ch];
    CAacDecoderStaticChannelInfo* s = e->pSt[ch];
    c->pComStaticData = &e->com->com;
    c->pComData = NULL;
    c->pSpectralCoefficient = &e->spec[ch * 1024];
    s->concealmentInfo.spectralCoefficient = e->st[ch]->concealSpec;
    s->pOverlapBuffer = e->st[ch]->overlap;
    s->hArCo = &e->st[ch]->arco;

    e->igfCommon[ch].virtualSpec = &e->timeData2[1024 * ch];
    if (enf) {
      iisIGFDecLibInit(&s->IGF_StaticData, &c->IGFdata, &e->igfCommon[ch], igfStart, igfStop,
                       igfHighRes, igfWhitening, e->sri.samplingRate, 1024,
                       e->sri.ScaleFactorBands_Long, e->sri.NumberOfScaleFactorBands_Long,
                       e->sri.ScaleFactorBands_Short, e->sri.NumberOfScaleFactorBands_Short, enf,
                       igfAfterTns, igfIndepTiling);
    }
    s->pCpeStaticData = e->cpe;
    if (e->cpe != NULL) {
      e->cpe->jointStereoPersistentData.spectralCoeffs[ch] = s->concealmentInfo.spectralCoefficient;
      e->cpe->jointStereoPersistentData.specScale[ch] = s->concealmentInfo.specScale;
      e->cpe->jointStereoPersistentData.scratchBuffer = e->wb;
      e->cpe->jointStereoPersistentData.scratchBuffer2 = &e->timeData2[2 * 1024];
    }
  }
  e->pSt[0]->nfRandomSeed = (ULONG)0x3039;
  if (nch == 2) e->pSt[1]->nfRandomSeed = (ULONG)0x10932;
  for (int ch = 0; ch < nch; ch++) {
    e->pCh[ch]->granuleLength = 1024 / 8;
    e->pCh[ch]->renderMode = AACDEC_RENDER_IMDCT;
    mdct_init(&e->pSt[ch]->IMdct, e->pSt[ch]->pOverlapBuffer, OverlapBufferSize);
    CConcealment_InitChannelData(&e->pSt[ch]->concealmentInfo, &e->conceal, AACDEC_RENDER_IMDCT, 1024);
  }
  return e;
}

void cref_core_el_free(void* p) {
  CREF_EL* e = (CREF_EL*)p;
  for (int ch = 0; ch < 2; ch++) {
    if (e->ch[ch]) FDKfree(e->ch[ch]);
    if (e->st[ch]) FDKfree(e->st[ch]);
  }
  if (e->cpe) FDKfree(e->cpe);
  FDKfree(e->com);
  FDKfree(e);
}

/* Accessors (for the dump functions of core_common.cpp and igf.cpp). */
void* cref_core_el_ch(void* p, INT ch) { return ((CREF_EL*)p)->ch[ch]; }
void* cref_core_el_st(void* p, INT ch) { return ((CREF_EL*)p)->st[ch]; }
void* cref_core_el_com(void* p) { return ((CREF_EL*)p)->com; }
void* cref_core_el_cpe(void* p) { return ((CREF_EL*)p)->cpe; }
void* cref_core_el_igf_common(void* p, INT ch) { return &((CREF_EL*)p)->igfCommon[ch]; }
FIXP_DBL* cref_core_el_spec(void* p) { return ((CREF_EL*)p)->spec; }
FIXP_DBL* cref_core_el_wb(void* p) { return ((CREF_EL*)p)->wb; }
FIXP_DBL* cref_core_el_time_data2(void* p) { return ((CREF_EL*)p)->timeData2; }

/* CChannelElement_Read() */
INT cref_core_el_read(void* p, void* bs, INT aot, UINT flags, UINT elFlags, UINT frameLength, INT nch) {
  CREF_EL* e = (CREF_EL*)p;
  return CChannelElement_Read((HANDLE_FDK_BITSTREAM)bs, e->pCh, e->pSt, (AUDIO_OBJECT_TYPE)aot, &e->sri,
                              flags, elFlags, frameLength, (UCHAR)nch, 0, NULL);
}

/* CChannelElement_Decode() */
void cref_core_el_decode(void* p, UINT flags, UINT elFlags, INT nch) {
  CREF_EL* e = (CREF_EL*)p;
  CChannelElement_Decode(e->pCh, e->pSt, &e->sri, flags, elFlags, nch);
}

/* CBlock_FrequencyToTime() of channel ch of the element, pWorkBuffer1 = mdctOutTemp. */
void cref_core_el_f2t(void* p, INT ch, FIXP_DBL* out, INT frameLen, INT frameOk, INT headroom, UINT elFlags) {
  CREF_EL* e = (CREF_EL*)p;
  CBlock_FrequencyToTime(e->pSt[ch], e->pCh[ch], out, (SHORT)frameLen, frameOk,
                         ((CWorkBufferCore1*)e->wb)->mdctOutTemp, headroom, elFlags, ch);
}

/* CConcealment_Apply() of channel ch of the element (as called by CAacDecoder_DecodeFrame()). */
INT cref_core_el_conceal_apply(void* p, INT ch, INT frameOk, UINT flags) {
  CREF_EL* e = (CREF_EL*)p;
  return CConcealment_Apply(&e->pSt[ch]->concealmentInfo, e->pCh[ch], e->pSt[ch], &e->sri, 1024, 0,
                            frameOk, flags);
}

/* ---------------------------------------------------------------------------------------------
   Block level functions on an element context
   --------------------------------------------------------------------------------------------- */

INT cref_core_el_read_sf(void* p, INT ch, void* bs, UINT flags) {
  CREF_EL* e = (CREF_EL*)p;
  return CBlock_ReadScaleFactorData(e->pCh[ch], (HANDLE_FDK_BITSTREAM)bs, flags);
}

INT cref_core_el_inverse_quantize(void* p, INT ch, INT activeBandSearch) {
  CREF_EL* e = (CREF_EL*)p;
  return CBlock_InverseQuantizeSpectralData(e->pCh[ch], &e->sri, e->pCh[ch]->pDynData->band_is_noise,
                                            (UCHAR)activeBandSearch);
}

void cref_core_el_scale_spectral_data(void* p, INT ch, INT maxSfbs) {
  CREF_EL* e = (CREF_EL*)p;
  CBlock_ScaleSpectralData(e->pCh[ch], (UCHAR)maxSfbs, &e->sri);
}

void cref_core_el_scale_tile_data(void* p, INT ch, INT maxSfbs) {
  CREF_EL* e = (CREF_EL*)p;
  CBlock_ScaleTileData(e->pSt[ch], e->pCh[ch], (UCHAR)maxSfbs, &e->sri);
}

void cref_core_el_apply_noise(void* p, INT ch, INT infActive, INT stereoFilling, INT maxNoiseSfb) {
  CREF_EL* e = (CREF_EL*)p;
  CBlock_ApplyNoise(e->pSt[ch], e->pCh[ch], &e->sri, &e->pSt[ch]->nfRandomSeed,
                    e->pCh[ch]->pDynData->band_is_noise, infActive, stereoFilling, maxNoiseSfb);
}

INT cref_core_el_read_ac(void* p, INT ch, void* bs, UINT frameLength, UINT flags) {
  CREF_EL* e = (CREF_EL*)p;
  return CBlock_ReadAcSpectralData((HANDLE_FDK_BITSTREAM)bs, e->pCh[ch], e->pSt[ch], &e->sri, frameLength,
                                   flags);
}

void cref_core_el_apply_tools(void* p, UINT flags, UINT elFlags, INT ch, INT commonWindow) {
  CREF_EL* e = (CREF_EL*)p;
  ApplyTools(e->pSt, e->pCh, &e->sri, flags, elFlags, ch, commonWindow);
}

/* IGF_StereoFillingPrepare() + IGF_StereoFillingApply() of the right channel on the main spectrum
   (and the tiles if tiles != 0), with dmx_prev_modified/exp given by the caller (1024 / 64 values,
   modified). */
void cref_core_el_stereo_filling(void* p, INT maxNoiseSfb, FIXP_DBL* dmx, SHORT* dmxExp, INT tiles) {
  CREF_EL* e = (CREF_EL*)p;
  CAacDecoderChannelInfo* c = e->pCh[1];
  CAacDecoderStaticChannelInfo* s = e->pSt[1];
  IGF_StereoFillingPrepare(c, &e->sri, maxNoiseSfb, dmx, dmxExp, c->pDynData->band_is_noise);
  if (tiles) {
    INT n = iisIGFDecLibGetNumberOfTiles(&s->IGF_StaticData, 0);
    for (int t = 0; t < n; t++) {
      FIXP_DBL* ts = iisIGFDecLibAccessSourceSpectrum(&s->IGF_StaticData, &c->IGFdata, t, 0, 0);
      SHORT* te = iisIGFDecLibAccessSourceSpectrum_exponent(&s->IGF_StaticData, t, 0);
      IGF_StereoFillingApply(c, ts, te, &e->sri, maxNoiseSfb, dmx, dmxExp, c->pDynData->band_is_noise);
    }
  }
  IGF_StereoFillingApply(c, c->pSpectralCoefficient, c->pDynData->aSfbScale, &e->sri, maxNoiseSfb, dmx,
                         dmxExp, c->pDynData->band_is_noise);
}

/* IGF_StereoFilling_GetPreviousDmx() of the element (CPE with cpPossible). */
void cref_core_el_get_previous_dmx(void* p, FIXP_DBL* dmx, SHORT* dmxExp, INT maxNoiseSfb) {
  CREF_EL* e = (CREF_EL*)p;
  CIcsInfo* ics = &e->pCh[0]->icsInfo;
  IGF_StereoFilling_GetPreviousDmx(e->pSt, e->pCh, dmx, dmxExp, GetScaleFactorBandOffsets(ics, &e->sri),
                                   GetWindowGroupLengthTable(ics), GetWindowGroups(ics), maxNoiseSfb,
                                   e->pCh[1]->pDynData->band_is_noise);
}

/* CJointStereo_ApplyMS() on the main spectra of the element (mainband_flag 1). */
void cref_core_el_apply_ms(void* p, INT maxSfbSte, INT sfbL, INT sfbR) {
  CREF_EL* e = (CREF_EL*)p;
  CAacDecoderChannelInfo** c = e->pCh;
  CJointStereo_ApplyMS(c, e->pSt, c[0]->pSpectralCoefficient, c[1]->pSpectralCoefficient,
                       c[0]->pDynData->aSfbScale, c[1]->pDynData->aSfbScale, c[0]->specScale,
                       c[1]->specScale, GetScaleFactorBandOffsets(&c[0]->icsInfo, &e->sri),
                       GetWindowGroupLengthTable(&c[0]->icsInfo), GetWindowGroups(&c[0]->icsInfo),
                       maxSfbSte, sfbL, sfbR, c[0]->pComStaticData->pJointStereoData->store_dmx_re_prev,
                       &(c[0]->pComStaticData->pJointStereoData->store_dmx_re_prev_e), 1);
}

/* CJointStereo_ApplyMSIGFcore() of the element. */
void cref_core_el_apply_ms_igf_core(void* p, INT maxSfbSte, INT sfbL, INT sfbR) {
  CREF_EL* e = (CREF_EL*)p;
  CAacDecoderChannelInfo** c = e->pCh;
  CJointStereo_ApplyMSIGFcore(c, e->pSt, GetScaleFactorBandOffsets(&c[0]->icsInfo, &e->sri),
                              GetWindowGroupLengthTable(&c[0]->icsInfo), GetWindowGroups(&c[0]->icsInfo),
                              maxSfbSte, sfbL, sfbR);
}

/* CJointStereo_ApplyMS_IGF() on the main spectra (virtual != 0: on the virtualSpec buffers). */
void cref_core_el_apply_ms_igf(void* p, INT virt) {
  CREF_EL* e = (CREF_EL*)p;
  CAacDecoderChannelInfo** c = e->pCh;
  FIXP_DBL* l = virt ? &e->timeData2[0] : c[0]->pSpectralCoefficient;
  FIXP_DBL* r = virt ? &e->timeData2[1024] : c[1]->pSpectralCoefficient;
  CJointStereo_ApplyMS_IGF(e->pSt, c, l, r, GetScaleFactorBandOffsets(&c[0]->icsInfo, &e->sri),
                           GetWindowGroupLengthTable(&c[0]->icsInfo), GetWindowGroups(&c[0]->icsInfo));
}

/* CJointStereo_Read() (+ CJointStereo_ReadIGF() if igf) as in the "ms" case of
   CChannelElement_Read(). */
INT cref_core_el_read_ms(void* p, void* bs, INT maxSfbSte, INT maxSfbSteClear, INT cplxPred, UINT flags, INT igf, INT igfStart, INT igfStop, INT highRes) {
  CREF_EL* e = (CREF_EL*)p;
  CAacDecoderChannelInfo** c = e->pCh;
  INT r = CJointStereo_Read((HANDLE_FDK_BITSTREAM)bs, c[0]->pComStaticData->pJointStereoData,
                            GetWindowGroups(&c[0]->icsInfo), maxSfbSte, maxSfbSteClear,
                            cplxPred ? &e->cpe->jointStereoPersistentData : NULL,
                            cplxPred ? c[0]->pComStaticData->cplxPredictionData : NULL, cplxPred,
                            GetScaleFactorBandsTotal(&c[0]->icsInfo), GetWindowSequence(&c[0]->icsInfo),
                            flags);
  if (igf) {
    r = r * 16 + CJointStereo_ReadIGF((HANDLE_FDK_BITSTREAM)bs, c[0]->pComStaticData->pJointStereoData,
                                      GetWindowGroups(&c[0]->icsInfo), igfStart, igfStop, highRes,
                                      &e->cpe->jointStereoPersistentData,
                                      c[0]->pComStaticData->cplxPredictionData,
                                      GetWindowSequence(&c[0]->icsInfo), flags);
  }
  return r;
}

/* CTns_Read() / CTns_ReadDataPresentUsac() / CTns_Apply() on the channels of the element. */
INT cref_core_el_tns_read(void* p, INT ch, void* bs, UINT flags) {
  CREF_EL* e = (CREF_EL*)p;
  return CTns_Read((HANDLE_FDK_BITSTREAM)bs, &e->pCh[ch]->pDynData->TnsData, &e->pCh[ch]->icsInfo, flags);
}
INT cref_core_el_tns_read_usac(void* p, void* bs, UINT flags, UINT elFlags, INT commonWindow) {
  CREF_EL* e = (CREF_EL*)p;
  UCHAR tns_on_lr = 0xAA;
  CTns_ReadDataPresentUsac((HANDLE_FDK_BITSTREAM)bs, &e->pCh[0]->pDynData->TnsData, &e->pCh[1]->pDynData->TnsData,
                           &tns_on_lr, &e->pCh[0]->icsInfo, flags, elFlags, commonWindow);
  return tns_on_lr;
}
void cref_core_el_tns_apply(void* p, INT ch, INT nbands, INT igfActive, UINT flags) {
  CREF_EL* e = (CREF_EL*)p;
  CTns_Apply(&e->pCh[ch]->pDynData->TnsData, &e->pCh[ch]->icsInfo, e->pCh[ch]->pSpectralCoefficient,
             &e->sri, e->pCh[ch]->granuleLength, (UCHAR)nbands, (UCHAR)igfActive, flags);
}

/* The inline functions of block.h. */
INT cref_core_evaluate_power43(FIXP_DBL* v, UINT lsb) { return EvaluatePower43(v, lsb); }
void cref_core_inverse_quantize(FIXP_DBL* v, UINT lsb, INT scale) { InverseQuantize(v, lsb, scale); }
INT cref_core_unpack_index(INT idx, FIXP_DBL* qp) { return CBlock_UnpackIndex(idx, qp, &AACcodeBookDescriptionSCL); }
INT cref_core_decode_huffman_word(void* bs, INT cb) {
  if (cb) return CBlock_DecodeHuffmanWordCB((HANDLE_FDK_BITSTREAM)bs, AACcodeBookDescriptionSCL.CodeBook);
  return CBlock_DecodeHuffmanWord((HANDLE_FDK_BITSTREAM)bs, &AACcodeBookDescriptionSCL);
}
void cref_core_codebook_table_init(void* p, INT ch) {
  CChannel_CodebookTableInit(((CREF_EL*)p)->pCh[ch]);
}

}  // extern "C"
