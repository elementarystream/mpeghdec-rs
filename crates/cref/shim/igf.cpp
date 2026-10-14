// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libIGFdec (FDK_igfDec.h, FDK_igfSCFDecoder.h).

#include "FDK_igfDec.h"
#include "aac_rom.h"

/* Global function of FDK_igfDec.cpp without declaration in a header. */
FIXP_DBL Autocorrelation(FIXP_DBL* x, INT* x_exp, INT n, INT lag);

/* Appends the elements of an array to the dump buffer. */
#define DUMP(x) (out[n++] = (INT)(x))
#define DUMP_ARR(a, len)                              \
  do {                                                \
    for (int _i = 0; _i < (int)(len); _i++) DUMP((a)[_i]); \
  } while (0)

extern "C" {

/* Scale factor band offset table of sfbOffsetTables[0][srIdx] (1024 frame length). */
const SHORT* cref_igf_sfb_table(INT srIdx, INT isShort, INT* len) {
  const SFB_INFO* info = &sfbOffsetTables[0][srIdx];
  *len = isShort ? info->numberOfSfbShort : info->numberOfSfbLong;
  return isShort ? info->sfbOffsetShort : info->sfbOffsetLong;
}

void* cref_igf_static_new() { return FDKcalloc(1, sizeof(IGF_PRIVATE_STATIC_DATA)); }
void* cref_igf_data_new() { return FDKcalloc(1, sizeof(IGF_PRIVATE_DATA)); }
void cref_igf_free(void* p) { FDKfree(p); }
void* cref_igf_common_new() {
  IGF_PRIVATE_DATA_COMMON* c = (IGF_PRIVATE_DATA_COMMON*)FDKcalloc(1, sizeof(IGF_PRIVATE_DATA_COMMON));
  c->virtualSpec = (FIXP_DBL*)FDKcalloc(1024, sizeof(FIXP_DBL));
  return c;
}
void cref_igf_common_free(void* p) {
  IGF_PRIVATE_DATA_COMMON* c = (IGF_PRIVATE_DATA_COMMON*)p;
  FDKfree(c->virtualSpec);
  FDKfree(c);
}
FIXP_DBL* cref_igf_common_virtual_spec(void* c) { return ((IGF_PRIVATE_DATA_COMMON*)c)->virtualSpec; }
/* Sets the common data handle of hPrivateData (the core decoder shares it between elements). */
void cref_igf_set_common(void* d, void* c) {
  ((IGF_PRIVATE_DATA*)d)->IGF_Common_channel_data_handle = (IGF_PRIVATE_DATA_COMMON*)c;
}

void cref_igf_init(void* s, void* d, void* c, UCHAR startIdx, UCHAR stopIdx, UCHAR highRes, UCHAR whitening, UINT sampleRate, INT frameLength, const SHORT* sfbLB, INT lenLB, const SHORT* sfbSB, INT lenSB, UCHAR enf, UCHAR afterTns, UCHAR indepTiling) {
  iisIGFDecLibInit((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d, (IGF_PRIVATE_DATA_COMMON*)c,
                   startIdx, stopIdx, highRes, whitening, sampleRate, frameLength, sfbLB, lenLB,
                   sfbSB, lenSB, enf, afterTns, indepTiling);
}

void cref_igf_read_scf(void* s, void* d, void* bs, INT indepFlag, INT winMode, INT winCount, INT frameType) {
  iisIGFDecLibReadSCF((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d, (HANDLE_FDK_BITSTREAM)bs,
                      indepFlag, winMode, winCount, frameType);
}

INT cref_igf_read_igf(void* s, void* d, void* bs, INT ws, INT indepFlag, INT useEnf, INT frameType) {
  return iisIGFDecLibReadIGF((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d,
                             (HANDLE_FDK_BITSTREAM)bs, ws, indepFlag, useEnf, frameType);
}

void cref_igf_sync(void* l, void* r) { iisIGF_Sync_Data((IGF_PRIVATE_DATA*)l, (IGF_PRIVATE_DATA*)r); }

void cref_igf_apply(void* s, void* d, FIXP_DBL* spec, SHORT* scale, INT ws, INT groups, INT nSpectra, const UCHAR* groupLength, UINT* seed, UCHAR* tnfMask, UCHAR inf, INT frameType) {
  CIgf_apply((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d, spec, scale, ws, groups, nSpectra,
             groupLength, seed, tnfMask, inf, frameType);
}

void cref_igf_apply_stereo(void* sL, void* sR, void* dL, void* dR, FIXP_DBL* specL, FIXP_DBL* specR, SHORT* scaleL, SHORT* scaleR, INT ws, INT groups, INT nSpectra, const UCHAR* groupLength, UINT* seedL, UINT* seedR, const UCHAR* msUsed, UCHAR* tnfMaskL, UCHAR* tnfMaskR, UCHAR inf, INT frameType) {
  CIgf_apply_stereo((IGF_PRIVATE_STATIC_DATA*)sL, (IGF_PRIVATE_STATIC_DATA*)sR, (IGF_PRIVATE_DATA*)dL,
                    (IGF_PRIVATE_DATA*)dR, specL, specR, scaleL, scaleR, ws, groups, nSpectra,
                    groupLength, seedL, seedR, msUsed, tnfMaskL, tnfMaskR, inf, frameType);
}

void cref_igf_tnf_apply(void* s, void* d, FIXP_DBL* spec, SHORT* scale, const UCHAR* tnfMask, INT ws, INT frameType) {
  CIgf_TNF_apply((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d, spec, scale, tnfMask, ws, frameType);
}

void cref_igf_inject_new(void* s, void* d, FIXP_DBL* spec, SHORT* specExp, INT ws, INT frameType) {
  iisIGFDecLibInjectSourceSpectrumNew((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d, spec,
                                      specExp, ws, frameType);
}

INT cref_igf_num_tiles(void* s, INT ws) {
  return iisIGFDecLibGetNumberOfTiles((IGF_PRIVATE_STATIC_DATA*)s, ws);
}

/* Offset (in FIXP_DBL) of the tile spectrum pointer from Spectrum_tab_array, -1 for NULL. */
INT cref_igf_access_spectrum(void* s, void* d, INT tileIdx, INT ws, INT win) {
  FIXP_DBL* p = iisIGFDecLibAccessSourceSpectrum((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d,
                                                 tileIdx, ws, win);
  return p ? (INT)(p - ((IGF_PRIVATE_DATA*)d)->Spectrum_tab_array) : -1;
}

/* Offset (in SHORT) of the exponent pointer from fSpectrumTab_sfb_exp. */
INT cref_igf_access_exponent(void* s, INT tileIdx, INT ws) {
  SHORT* p = iisIGFDecLibAccessSourceSpectrum_exponent((IGF_PRIVATE_STATIC_DATA*)s, tileIdx, ws);
  return (INT)(p - &((IGF_PRIVATE_STATIC_DATA*)s)->fSpectrumTab_sfb_exp[0][0]);
}

/* Offset (in SHORT) of the scale pointer from fSpectrumScale. */
INT cref_igf_access_scale(void* s, void* d, INT tileIdx, INT ws) {
  SHORT* p = iisIGFDecLibAccessSourceSpectrum_Scale((IGF_PRIVATE_STATIC_DATA*)s, (IGF_PRIVATE_DATA*)d,
                                                    tileIdx, ws);
  return (INT)(p - &((IGF_PRIVATE_STATIC_DATA*)s)->fSpectrumScale[0][0]);
}

void cref_igf_set_spectrum_scale(void* s, const SHORT* scale) {
  FDKmemcpy(((IGF_PRIVATE_STATIC_DATA*)s)->fSpectrumScale, scale,
            sizeof(((IGF_PRIVATE_STATIC_DATA*)s)->fSpectrumScale));
}

void cref_igf_set_tile_spectrum(void* d, const FIXP_DBL* spec) {
  FDKmemcpy(((IGF_PRIVATE_DATA*)d)->Spectrum_tab_array, spec,
            sizeof(((IGF_PRIVATE_DATA*)d)->Spectrum_tab_array));
}

FIXP_DBL cref_igf_autocorrelation(FIXP_DBL* x, INT* xExp, INT n, INT lag) {
  return Autocorrelation(x, xExp, n, lag);
}

/* Dump of IGF_PRIVATE_STATIC_DATA; fSpectrumTab pointers as offsets into d->Spectrum_tab_array. */
INT cref_igf_dump_static(void* sp, void* dp, INT* out) {
  IGF_PRIVATE_STATIC_DATA* s = (IGF_PRIVATE_STATIC_DATA*)sp;
  IGF_PRIVATE_DATA* d = (IGF_PRIVATE_DATA*)dp;
  INT n = 0;
  IGFSCFDEC_PRIVATE_DATA* h = &s->hIGFSCFDecoder;
  DUMP(h->st.low);
  DUMP(h->st.high);
  DUMP(h->st.vobf);
  DUMP(h->st.bits_to_follow);
  DUMP_ARR(h->prevBuffer, 112);
  DUMP(h->prevSB - h->prevBuffer);
  DUMP(h->prevLB - h->prevBuffer);
  DUMP(h->prevTCXmedium - h->prevBuffer);
  DUMP(h->prevTCXlong - h->prevBuffer);
  DUMP(h->prevDLB);
  DUMP(h->prevDSB);
  DUMP(h->prevDTCXmedium);
  DUMP(h->prevDTCXlong);
  DUMP(h->scfCountLongBlock);
  DUMP(h->scfCountShortBlock);
  DUMP(h->scfCountTCXBlock);
  DUMP(h->t);
  DUMP(h->lastFrameBlockType);
  DUMP(h->arith_decode_first_symbol);
  for (int g = 0; g < IGF_MAX_GRIDS; g++) {
    IGF_GRID_INFO* gi = &s->sGridInfoTab[g];
    for (int t = 0; t < IGF_MAX_TILES; t++) {
      IGF_MAP_INFO* m = &gi->sIGFMapInfoTab[t];
      DUMP(m->fSpectrumTab[0] ? (INT)(m->fSpectrumTab[0] - d->Spectrum_tab_array) : -1);
      DUMP(m->iWhiteningLevel);
      DUMP_ARR(m->iSfbWidthTab, IGF_MAX_SFB_BANDS);
      DUMP(m->iSfbCnt);
      DUMP(m->iDesStart);
      DUMP(m->iWidth);
      DUMP(m->iIdx);
      DUMP(m->iSfbSplit);
    }
    DUMP(gi->iIGFStartLine);
    DUMP(gi->iIGFStopLine);
    DUMP(gi->iIGFMinLine);
    DUMP_ARR(gi->iIGFTileIdxTab, IGF_MAX_TILES);
    DUMP(gi->iIGFNumSfb);
    DUMP(gi->iIGFStartSfb);
    DUMP(gi->iIGFStopSfb);
    DUMP(gi->iIGFNumTile);
  }
  for (int t = 0; t < IGF_MAX_TILES; t++) DUMP_ARR(s->fSpectrumScale[t], IGF_MAX_WIN);
  for (int t = 0; t < IGF_MAX_TILES; t++)
    DUMP_ARR(s->fSpectrumTab_sfb_exp[t], IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
  DUMP_ARR(s->igfWhiteningLevelPrev, IGF_MAX_TILES);
  DUMP(s->igfMinSubbandLB);
  DUMP(s->igfMinSubbandSB);
  DUMP_ARR(s->igfTilesLB, IGF_MAX_TILES);
  DUMP_ARR(s->igfTilesSB, IGF_MAX_TILES);
  DUMP_ARR(s->prevPatchNum, IGF_MAX_TILES);
  DUMP(s->numSfbLB);
  if (s->sfbOffsetLB) DUMP_ARR(s->sfbOffsetLB, s->numSfbLB + 1);
  DUMP(s->numSfbSB);
  if (s->sfbOffsetSB) DUMP_ARR(s->sfbOffsetSB, s->numSfbSB + 1);
  DUMP(s->enhancedNoiseFilling);
  DUMP(s->igfUseWhitening);
  DUMP(s->igfAfterTnsSynth);
  DUMP(s->igfIndependentTiling);
  DUMP(s->useHighRes);
  DUMP(s->igfStartSfbLB);
  DUMP(s->igfStopSfbLB);
  DUMP(s->igfStartSfbSB);
  DUMP(s->igfStopSfbSB);
  DUMP(s->igfNTilesLB);
  DUMP(s->igfNTilesSB);
  return n;
}

/* Dump of IGF_PRIVATE_DATA (without the common data handle). */
INT cref_igf_dump_data(void* dp, INT* out) {
  IGF_PRIVATE_DATA* d = (IGF_PRIVATE_DATA*)dp;
  INT n = 0;
  DUMP_ARR(d->Spectrum_tab_array, IGF_MAX_TILES * 1024);
  for (int i = 0; i < N_IGF_FRAME_DIVISION; i++) {
    IGF_BITSTREAM_DATA* b = &d->bitstreamData[i];
    DUMP_ARR(b->sfe, 256);
    DUMP_ARR(b->iCurrTileIdxTab, IGF_MAX_TILES);
    DUMP_ARR(b->igfWhiteningLevel, IGF_MAX_TILES);
    DUMP(b->igf_allZero);
    DUMP(b->igfUseEnfFlat);
  }
  DUMP_ARR(d->tileNum, IGF_MAX_TILES);
  return n;
}

/* Dump of IGF_PRIVATE_DATA_COMMON (without virtualSpec). */
INT cref_igf_dump_common(void* cp, INT* out) {
  IGF_PRIVATE_DATA_COMMON* c = (IGF_PRIVATE_DATA_COMMON*)cp;
  INT n = 0;
  for (int t = 0; t < IGF_MAX_TILES; t++) {
    IGF_WORK_MEMORY* w = &c->IGF_WorkingMem[t];
    DUMP_ARR(w->fSfbGainTab, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->fSfbDestinEnergyTab, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->fSfbTileEnergyTab, IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
    DUMP_ARR(w->fSfbSurvivedEnergyTab, IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
    DUMP_ARR(w->sE, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->tE, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->fSfbGainTab_exp, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->fSfbDestinEnergyTab_exp, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->fSfbTileEnergyTab_exp, IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
    DUMP_ARR(w->fSfbSurvivedEnergyTab_exp, IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
    DUMP_ARR(w->sE_exp, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->tE_exp, IGF_MAX_SFB_BANDS);
    DUMP_ARR(w->intermediate_shift, IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
    DUMP_ARR(w->Initial_exp, IGF_MAX_WIN * IGF_MAX_SFB_SHORT);
  }
  return n;
}

}  // extern "C"
