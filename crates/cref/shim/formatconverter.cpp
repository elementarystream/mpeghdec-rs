// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFormatConverter: format converter API (FDK_formatConverterLib.h), the downmix
// matrix parser (FDK_dmxMatrixParser.h), peak_filter() and a serializer of the complete converter
// state for differential testing. The serializer writes the fields as INT values in a fixed
// order; tests/formatconverter.rs contains the matching Rust serializer.
//
// Scratch buffers handed to the library are filled with a garbage pattern, so that reads of
// uninitialized scratch memory show up as differences.

#include <stdlib.h>
#include <string.h>

#include "FDK_formatConverterLib.h"
#include "FDK_formatConverter_api.h"
#include "FDK_formatConverter_data.h"
#include "FDK_formatConverter_init.h"
#include "FDK_formatConverter_intern.h"
#include "FDK_formatConverter_activeDmx_stft.h"
#include "FDK_dmxMatrixParser.h"
#include "FDK_stftfilterbank.h"

#define P(x) (out[n++] = (INT)(x))

#define SCRATCH_GARBAGE 0x5A

/* geo: n entries of 10 INTs (cicpLoudspeakerIndex, Az, El, LFE, screenRelative, hasDistance,
 * distance, hasLoudspeakerCalibrationGain, loudspeakerCalibrationGain, loudspeakerType) */
static void geo_from_ints(CICP2GEOMETRY_CHANNEL_GEOMETRY* g, const INT* geo, int n) {
  memset(g, 0, sizeof(*g) * n);
  for (int i = 0; i < n; i++) {
    g[i].cicpLoudspeakerIndex = geo[10 * i + 0];
    g[i].Az = geo[10 * i + 1];
    g[i].El = geo[10 * i + 2];
    g[i].LFE = geo[10 * i + 3];
    g[i].screenRelative = geo[10 * i + 4];
    g[i].hasDistance = geo[10 * i + 5];
    g[i].distance = (UINT)geo[10 * i + 6];
    g[i].hasLoudspeakerCalibrationGain = geo[10 * i + 7];
    g[i].loudspeakerCalibrationGain = geo[10 * i + 8];
    g[i].loudspeakerType = (CICP2GEOMETRY_LOUDSPEAKER_TYPE)geo[10 * i + 9];
  }
}

static int ser_geo(const CICP2GEOMETRY_CHANNEL_GEOMETRY* g, INT* out, int n) {
  P(g->cicpLoudspeakerIndex);
  P(g->Az);
  P(g->El);
  P(g->LFE);
  P(g->screenRelative);
  P(g->hasDistance);
  P(g->distance);
  P(g->hasLoudspeakerCalibrationGain);
  P(g->loudspeakerCalibrationGain);
  P(g->loudspeakerType);
  return n;
}

static int ser_stft_config(const STFT_FILTERBANK_CONFIG* c, INT* out, int n) {
  P(c->stftFilterbankMode);
  P(c->frameSize);
  P(c->fftSize);
  return n;
}

static int ser_filterbank(HANDLE_STFT_FILTERBANK f, INT* out, int n) {
  P(f != NULL);
  if (f == NULL) return n;
  n = ser_stft_config(&f->stftFilterbankConfig, out, n);
  P(f->numZerosPaddedTwoSided);
  for (UINT i = 0; i < f->stftFilterbankConfig.frameSize; i++) P(f->prevAudioInput[i]);
  return n;
}

static int dmx_sel(HANDLE_FORMAT_CONVERTER_PARAMS p, const FIXP_DMX_H* x) {
  if (x == NULL) return 0;
  if (x == p->dmxMtx_sorted) return 1;
  if (x == p->dmxMtxL_sorted) return 2;
  if (x == p->dmxMtx2_sorted) return 3;
  return 99;
}

static int eq_sel(HANDLE_FORMAT_CONVERTER_PARAMS p, const INT* x) {
  if (x == NULL) return 0;
  if (x == p->eqIndexVec_sorted) return 1;
  if (x == p->eqIndexVec2_sorted) return 2;
  return 99;
}

extern "C" {

/* ---- format converter API ---- */

void* cref_fc_create(INT mode, const INT* outGeo, UINT numOut, UINT sampleRate, UINT frameSize, INT* err) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[32];
  geo_from_ints(g, outGeo, (int)numOut);
  IIS_FORMATCONVERTER_HANDLE h = NULL;
  *err = IIS_FormatConverter_Create(&h, (IIS_FORMATCONVERTER_MODE)mode, g, numOut, sampleRate, frameSize);
  if (*err != 0) {
    if (h != NULL) IIS_FormatConverter_Close(&h);
    return NULL;
  }
  return h;
}
void cref_fc_close(void* h) {
  IIS_FORMATCONVERTER_HANDLE f = (IIS_FORMATCONVERTER_HANDLE)h;
  IIS_FormatConverter_Close(&f);
}
INT cref_fc_add_input(void* h, const INT* geo, UINT numChannels) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[32];
  geo_from_ints(g, geo, (int)numChannels);
  return IIS_FormatConverter_Config_AddInputSetup((IIS_FORMATCONVERTER_HANDLE)h, g, numChannels, 0, 0);
}
void cref_fc_set_aes(void* h, UINT aes) { IIS_FormatConverter_Config_SetAES((IIS_FORMATCONVERTER_HANDLE)h, aes); }
void cref_fc_set_pas(void* h, UINT pas) { IIS_FormatConverter_Config_SetPAS((IIS_FORMATCONVERTER_HANDLE)h, pas); }
INT cref_fc_set_immersive(void* h, UINT flag) { return IIS_FormatConverter_Config_SetImmersiveDownmixFlag((IIS_FORMATCONVERTER_HANDLE)h, flag); }
INT cref_fc_set_rendering3d(void* h, UINT flag) { return IIS_FormatConverter_Config_SetRendering3DTypeFlag((IIS_FORMATCONVERTER_HANDLE)h, flag); }
INT cref_fc_is_open(void* h) { return IIS_FormatConverter_IsOpen((IIS_FORMATCONVERTER_HANDLE)h); }
INT cref_fc_get_delay(void* h) {
  UINT d = 0;
  IIS_FormatConverter_GetDelay((IIS_FORMATCONVERTER_HANDLE)h, &d);
  return (INT)d;
}
void cref_fc_add_signals(void* h, UINT n) { ((IIS_FORMATCONVERTER_HANDLE)h)->numSignalsTotal += n; }

INT cref_fc_open(void* h, UINT bufSize) {
  INT* buf = (INT*)malloc(bufSize + 16);
  memset(buf, SCRATCH_GARBAGE, bufSize + 16);
  INT err = IIS_FormatConverter_Open((IIS_FORMATCONVERTER_HANDLE)h, buf, bufSize);
  free(buf);
  return err;
}

/* in: numTotalInputChannels blocks of inStride samples, isNull[ch] != 0 for a NULL channel;
   out: numOutputChannels blocks of frameSize samples (accumulated); drc may be NULL. */
INT cref_fc_process(void* h, INT* in, INT inStride, const INT* isNull, INT* out, void* drc) {
  IIS_FORMATCONVERTER_HANDLE f = (IIS_FORMATCONVERTER_HANDLE)h;
  IIS_FORMATCONVERTER_INTERNAL* p = (IIS_FORMATCONVERTER_INTERNAL*)f->member;
  FIXP_DBL* dein[32];
  for (UINT ch = 0; ch < 32; ch++) dein[ch] = NULL;
  for (UINT ch = 0; ch < p->numTotalInputChannels; ch++) {
    dein[ch] = isNull[ch] ? NULL : (FIXP_DBL*)&in[ch * inStride];
  }
  return IIS_FormatConverter_Process(f, (HANDLE_DRC_DECODER)drc, dein, (FIXP_DBL*)out, inStride);
}

INT cref_fc_frame(void* h, UCHAR* buf, UINT size, UINT validBits, INT* bitsLeft) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT r = FormatConverterFrame((IIS_FORMATCONVERTER_HANDLE)h, &bs);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return r;
}

/* The part of applyDownmixMatrixPerSignalGroup() (aacdecoder.cpp) for one signal group with a
   custom downmix matrix. spk: count triplets (Az, El, LFE). Returns 0, 1 (DecodeDownmixMatrix
   failed), 2 (formatConverterSetEQs failed) or 3 (invalid target layout). */
INT cref_fc_apply_dmx(void* h, INT grp, INT layout, INT count, const INT* spk, INT targetLayout, UCHAR* mem, INT sizeBits, INT offset) {
  IIS_FORMATCONVERTER_INTERNAL* _p = (IIS_FORMATCONVERTER_INTERNAL*)((IIS_FORMATCONVERTER_HANDLE)h)->member;
  SpeakerInformation inputConfig[FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS];
  CICP2GEOMETRY_CHANNEL_GEOMETRY outputConfig_geo[FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS];
  INT numOutputCh = 0, numOutLfes = 0;
  SpeakerInformation outputConfig[FDK_MPEGHAUDIO_DEC_MAX_OUTPUT_CHANNELS];
  FDK_BITSTREAM bitbuf;
  size_t scratchSize = sizeof(eqConfigStruct) + 64 * 1024;
  FIXP_DBL* p_buffer = (FIXP_DBL*)malloc(scratchSize);
  memset(p_buffer, SCRATCH_GARBAGE, scratchSize);
  eqConfigStruct* eqConfig = (eqConfigStruct*)p_buffer;
  FIXP_DBL* scratchBuf = (FIXP_DBL*)ALIGN_PTR(p_buffer + (sizeof(eqConfigStruct) / sizeof(FIXP_DBL) + 1));
  INT err = 0;

  FDKinitBitStream(&bitbuf, mem, 256, sizeBits, BS_READER);
  memset(inputConfig, 0, sizeof(inputConfig));
  memset(outputConfig, 0, sizeof(outputConfig));
  for (INT n = 0; n < count; n++) {
    inputConfig[n].azimuth = (SHORT)spk[3 * n];
    inputConfig[n].elevation = (SHORT)spk[3 * n + 1];
    inputConfig[n].isLFE = (SHORT)spk[3 * n + 2];
  }
  if (cicp2geometry_get_geometry_from_cicp(targetLayout, outputConfig_geo, &numOutputCh, &numOutLfes) != 0) {
    free(p_buffer);
    return 3;
  }
  for (INT n = 0; n < (numOutputCh + numOutLfes); n++) {
    outputConfig[n].azimuth = outputConfig_geo[n].Az;
    outputConfig[n].elevation = outputConfig_geo[n].El;
    outputConfig[n].isLFE = outputConfig_geo[n].LFE;
  }
  err = DecodeDownmixMatrix(layout, count, inputConfig, targetLayout, (numOutputCh + numOutLfes), outputConfig, &bitbuf,
                            _p->fcParams->dmxMtx_sorted + offset, eqConfig, scratchBuf);
  if (err != 0) {
    free(p_buffer);
    return 1;
  }
  formatConverterDmxMatrixExponent(_p);
  err = formatConverterSetEQs(_p->fcParams->eqIndexVec_sorted + offset, eqConfig->numEQs, eqConfig->eqParams, eqConfig->eqMap, grp, _p, scratchBuf);
  free(p_buffer);
  if (err != 0) return 2;
  _p->amountOfAddedDmxMatricesAndEqualizers += _p->numInputChannels[grp];
  return 0;
}

INT cref_fc_ser(void* hv, INT* out) {
  IIS_FORMATCONVERTER_HANDLE f = (IIS_FORMATCONVERTER_HANDLE)hv;
  IIS_FORMATCONVERTER_INTERNAL* p = (IIS_FORMATCONVERTER_INTERNAL*)f->member;
  int n = 0, i, j, k;
  P(f->numLocalSpeaker);
  P(f->pas);
  P(f->aes);
  P(f->numSignalsTotal);
  P(f->fCMode);

  P(p->fcNumFreqBands);
  P(p->fcCenterFrequencies != NULL);
  for (i = 0; i < 28; i++) n = ser_geo(&p->inputChannelGeo[i], out, n);
  for (i = 0; i < 28; i++) P(p->numInputChannels[i]);
  P(p->numInputChannelGroups);
  P(p->amountOfAddedDmxMatricesAndEqualizers);
  for (i = 0; i < 28; i++) {
    P(p->eqGains[i] != NULL);
    if (p->eqGains[i] != NULL)
      for (k = 0; k < 58; k++) P(p->eqGains[i][k]);
  }
  P(p->numTotalInputChannels);
  P(p->frameSize);
  P(p->STFT_headroom_prescaling);
  P(p->numTimeSlots);
  P(p->samplingRate);
  P(p->immersiveDownmixFlag);
  P(p->rendering3DTypeFlag);
  P(p->mode);
  P(p->fcInputFormat);
  P(p->fcOutputFormat);
  P(p->aes);
  P(p->numOutputChannels);
  for (i = 0; i < 24; i++) n = ser_geo(&p->outChannelGeo[i], out, n);
  P(p->cicpLayoutIndex);
  P(p->outChannelVbapFlag);
  P(p->inChannelVbapFlag);

  /* fcParams */
  HANDLE_FORMAT_CONVERTER_PARAMS pp = p->fcParams;
  P(pp != NULL);
  if (pp != NULL) {
    UINT nin = p->numTotalInputChannels, nout = p->numOutputChannels;
    P(pp->samplingRate);
    P(pp->phaseAlignStrength);
    P(pp->adaptiveEQStrength);
    P(pp->formatConverterInputFormat_internal);
    P(pp->formatConverterOutputFormat_internal);
    P(pp->formatConverterParams_internal != NULL);
    if (pp->formatConverterParams_internal != NULL)
      for (i = 0; i < N_EQ; i++)
        for (k = 0; k < MAXBANDS; k++) P(pp->formatConverterParams_internal->eq[i][k]);
    P(pp->genericIOFmt);
    for (i = 0; i < (INT)nin; i++)
      for (j = 0; j < (INT)nout; j++) {
        P(pp->dmxMtx[i][j]);
        P(pp->dmxMtxL[i][j]);
        P(pp->dmxMtx2[i][j]);
        P(pp->eqIndexVec[i][j]);
        P(pp->eqIndexVec2[i][j]);
      }
    for (i = 0; i < (INT)(nin * nout); i++) {
      P(pp->dmxMtx_sorted[i]);
      P(pp->dmxMtxL_sorted[i]);
      P(pp->dmxMtx2_sorted[i]);
      P(pp->eqIndexVec_sorted[i]);
      P(pp->eqIndexVec2_sorted[i]);
    }
    P(pp->immersiveMode);
    P(pp->Mode3Drendering);
    P(pp->Mode2Drendering);
    P(pp->rendering3DTypeFlag_internal);
    for (i = 0; i < 24; i++) P(pp->chOut_count[i]);
    for (i = 0; i < 24; i++) P(pp->chOut_exp[i]);
    P(pp->dmx_iterations);
    P(dmx_sel(pp, pp->dmxMatrixL_FDK));
    P(dmx_sel(pp, pp->dmxMatrixH_FDK));
    P(eq_sel(pp, pp->eqIndex_FDK));
    P(pp->dmxMtxIsSet);
    P(pp->formatConverterDelay);
    P(pp->centerFrequenciesNormalized != NULL);
  }

  /* fcState */
  P(p->fcState != NULL);
  if (p->fcState != NULL) {
    activeDownmixer* h = (activeDownmixer*)p->fcState->handleActiveDmxStft;
    P(h != NULL);
    if (h != NULL) {
      P(h->numInChans);
      P(h->numOutChans);
      P(h->fixpAES);
      for (i = 0; i < (INT)h->numOutChans; i++)
        for (k = 0; k < 58; k++) {
          P(h->targetEnePrev[i][k]);
          P(h->realizedEnePrev[i][k]);
        }
      for (k = 0; k < 58; k++) {
        P(h->targetEnePrevExp[k]);
        P(h->inBufStftHeadroomPrev[k]);
        P(h->realizedSigHeadroomPrev[k]);
      }
    }
  }

  /* STFT buffers and filterbanks */
  P(p->inputBufferStft != NULL);
  if (p->inputBufferStft != NULL)
    for (i = 0; i < (INT)p->numTotalInputChannels; i++)
      for (k = 0; k < (INT)p->stftLength; k++) P(p->inputBufferStft[i][k]);
  P(p->prevInputBufferStft != NULL);
  if (p->prevInputBufferStft != NULL)
    for (i = 0; i < TFRA + 1; i++)
      for (k = 0; k < (INT)p->stftLength; k++) P(p->prevInputBufferStft[i][k]);
  P(p->outputBufferStft != NULL);
  if (p->outputBufferStft != NULL)
    for (i = 0; i < (INT)p->numOutputChannels; i++)
      for (k = 0; k < (INT)p->stftLength; k++) P(p->outputBufferStft[i][k]);
  for (i = 0; i < 28; i++) n = ser_filterbank(p->stftFilterbankAnalysis[i], out, n);
  for (i = 0; i < 24; i++) n = ser_filterbank(p->stftFilterbankSynthesis[i], out, n);
  n = ser_stft_config(&p->stftFilterbankConfigAnalysis, out, n);
  n = ser_stft_config(&p->stftFilterbankConfigSynthesis, out, n);
  P(p->stftFrameSize);
  P(p->stftLength);
  P(p->stftNumErbBands);
  for (i = 0; i < 28; i++) P(p->format_in_listOfChannels[i]);
  P(p->format_in_listOfChannels_nchan);
  for (i = 0; i < 24; i++) P(p->format_out_listOfChannels[i]);
  P(p->format_out_listOfChannels_nchan);
  for (i = 0; i < 13; i++)
    for (j = 0; j < 6; j++) {
      P(p->GVH[i][j]);
      P(p->GVH_e[i][j]);
      P(p->GVL[i][j]);
      P(p->GVL_e[i][j]);
    }
  for (i = 0; i < 58; i++) P(p->is4GVH_StftErb[i]);
  for (i = 0; i < 13; i++) P(p->topIn[i]);
  for (i = 0; i < 6; i++) P(p->midOut[i]);
  P(p->erb_is4GVH_L);
  P(p->erb_is4GVH_H);
  P(p->openSuccess);
  return n;
}

/* ---- internal functions ---- */

INT cref_fc_match_channels(const INT* geo, INT inout, INT numChannels, INT* channelVec, UINT* numUnknown, INT* unknownVec) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[32];
  AFC_FORMAT_CONVERTER_CHANNEL_ID v[32];
  geo_from_ints(g, geo, numChannels);
  INT r = formatConverterMatchChConfig2Channels(g, inout, numChannels, v, numUnknown, unknownVec);
  for (int i = 0; i < numChannels; i++) channelVec[i] = v[i];
  return r;
}

FIXP_DBL cref_peak_filter(FIXP_DBL f, INT f_e, FIXP_DBL q, INT q_e, FIXP_DBL g, INT g_e, FIXP_DBL G, INT G_e, FIXP_DBL b, INT b_e, INT* result_e) {
  return peak_filter(f, f_e, q, q_e, g, g_e, G, G_e, b, b_e, result_e);
}

/* matrix: rows of nout values (FIXP_SGL in INT) */
INT cref_fc_postprocess(INT* matrix, UINT nin, UINT nout) {
  FIXP_DMX_H rows[32][32];
  FIXP_DMX_H* m[32];
  for (UINT i = 0; i < nin; i++) {
    for (UINT j = 0; j < nout; j++) rows[i][j] = (FIXP_DMX_H)matrix[i * nout + j];
    m[i] = rows[i];
  }
  INT r = formatConverterPostprocessDmxMtx(m, nin, nout);
  for (UINT i = 0; i < nin; i++)
    for (UINT j = 0; j < nout; j++) matrix[i * nout + j] = rows[i][j];
  return r;
}

/* ---- downmix matrix parser ---- */

static int ser_eq_config(const eqConfigStruct* e, INT* out, int n) {
  P(e->numEQs);
  for (int i = 0; i < FDK_DOWNMIX_MATRIX_DEC_MAX_INPUT_CHANNELS; i++) P(e->eqMap[i]);
  for (int i = 0; i < FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_EQ; i++) {
    const eqParamsStruct* q = &e->eqParams[i];
    P(q->nPkFilter);
    P(q->G);
    P(q->G_e);
    for (int j = 0; j < FDK_DOWNMIX_MATRIX_DEC_MAX_NUM_PEAK_FILTER_PARAMS; j++) {
      P(q->pkFilterParams[j].f);
      P(q->pkFilterParams[j].q);
      P(q->pkFilterParams[j].g);
      P(q->pkFilterParams[j].f_e);
      P(q->pkFilterParams[j].q_e);
      P(q->pkFilterParams[j].g_e);
    }
  }
  return n;
}

static int ser_speakers(const SpeakerInformation* s, int count, INT* out, int n) {
  for (int i = 0; i < count; i++) {
    P(s[i].elevation);
    P(s[i].azimuth);
    P(s[i].isLFE);
    P(s[i].originalPosition);
    P(s[i].isAlreadyUsed);
    P(s[i].symmetricPair ? (INT)(s[i].symmetricPair - s) : -1);
    P(s[i].pairType);
  }
  return n;
}

/* inSpk/outSpk: triplets (Az, El, LFE). dmx: inCount * outCount values. ser: eq config and the
   speaker lists after the call. */
INT cref_decode_dmx_matrix(INT inIdx, INT inCount, const INT* inSpk, INT outIdx, INT outCount, const INT* outSpk, UCHAR* buf, UINT size, UINT validBits, INT* dmx, INT* ser, INT* serLen, INT* bitsLeft) {
  SpeakerInformation inputConfig[32], outputConfig[32];
  eqConfigStruct eqConfig;
  FIXP_SGL m[32 * 32];
  memset(inputConfig, 0, sizeof(inputConfig));
  memset(outputConfig, 0, sizeof(outputConfig));
  memset(&eqConfig, 0, sizeof(eqConfig));
  for (int i = 0; i < 32 * 32; i++) m[i] = (FIXP_SGL)0x1234;
  for (int i = 0; i < inCount; i++) {
    inputConfig[i].azimuth = (SHORT)inSpk[3 * i];
    inputConfig[i].elevation = (SHORT)inSpk[3 * i + 1];
    inputConfig[i].isLFE = (SHORT)inSpk[3 * i + 2];
  }
  for (int i = 0; i < outCount; i++) {
    outputConfig[i].azimuth = (SHORT)outSpk[3 * i];
    outputConfig[i].elevation = (SHORT)outSpk[3 * i + 1];
    outputConfig[i].isLFE = (SHORT)outSpk[3 * i + 2];
  }
  size_t scratchSize = 64 * 1024;
  FIXP_DBL* scratch = (FIXP_DBL*)malloc(scratchSize);
  memset(scratch, SCRATCH_GARBAGE, scratchSize);
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT r = DecodeDownmixMatrix(inIdx, inCount, inputConfig, outIdx, outCount, outputConfig, &bs, m, &eqConfig, scratch);
  free(scratch);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  for (int i = 0; i < inCount * outCount; i++) dmx[i] = m[i];
  int n = 0;
  n = ser_eq_config(&eqConfig, ser, n);
  n = ser_speakers(inputConfig, inCount, ser, n);
  n = ser_speakers(outputConfig, outCount, ser, n);
  *serLen = n;
  return r;
}

/* set: serialized FDK_DOWNMIX_GROUPS_MATRIX_SET (9*256 bytes, 9 sizes, 56 indices). drc may be
   NULL. */
INT cref_dmx_matrix_set(UCHAR* buf, UINT size, UINT validBits, INT targetLayout, INT downmixConfigType, INT* downmixId, INT* set, INT* bitsLeft, void* drc) {
  FDK_DOWNMIX_GROUPS_MATRIX_SET s;
  memset(&s, 0, sizeof(s));
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT r = DownmixMatrixSet(&bs, &s, targetLayout, downmixConfigType, downmixId, (HANDLE_DRC_DECODER)drc);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  int n = 0;
  INT* out = set;
  for (int l = 0; l < FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID; l++)
    for (int b = 0; b < FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE; b++) P(s.downmixMatrixMemory[l][b]);
  for (int l = 0; l < FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID; l++) P(s.downmixMatrixSize[l]);
  for (int g = 0; g < FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS; g++) P(s.downmixMatrix[g]);
  return r;
}

}  // extern "C"
