// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libMpeghDec aac_rom.cpp and channelinfo.cpp: table dumps, getSamplingRateInfo(),
// IcsRead() and IcsReadMaxSfb().

#include "channelinfo.h"
#include "aac_rom.h"

#define DUMP(x) (out[n++] = (INT)(x))
#define DUMP_ARR(a, len)                                   \
  do {                                                     \
    for (int _i = 0; _i < (int)(len); _i++) DUMP((a)[_i]); \
  } while (0)

extern "C" {

/* Dumps a table of aac_rom.cpp as INT values:
   0 MantissaTable, 1 InverseQuantTableMant, 2 InverseQuantTableExp,
   3 InverseQuantInterpolationTable, 4 a2ToPow0p25Mant (+ a2ToPow0p25Exp,
   InverseQuantInterpolationExp), 5 sfbOffsetTables (all 5x16 entries: numbers and offsets),
   6 AACcodeBookDescriptionSCL (description and codebook), 7 tns_max_bands_tbl,
   8 FDKaacDec_tnsCoeff3, 9 FDKaacDec_tnsCoeff4, 10 AacDec_randomSign, 11 mdst_filt_coef_curr,
   12 mdst_filt_coef_prev. Returns the number of values. */
INT cref_core_rom_dump(INT id, INT* out) {
  INT n = 0;
  switch (id) {
    case 0:
      for (int i = 0; i < 4; i++) DUMP_ARR(MantissaTable[i], 14);
      break;
    case 1:
      DUMP_ARR(InverseQuantTableMant, INV_QUANT_TABLESIZE + 1);
      break;
    case 2:
      DUMP_ARR(InverseQuantTableExp, INV_QUANT_TABLESIZE + 1);
      break;
    case 3:
      DUMP_ARR(InverseQuantInterpolationTable, 8);
      break;
    case 4:
      DUMP_ARR(a2ToPow0p25Mant, 4);
      DUMP(a2ToPow0p25Exp);
      DUMP(InverseQuantInterpolationExp);
      break;
    case 5:
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 16; j++) {
          const SFB_INFO* s = &sfbOffsetTables[i][j];
          DUMP(s->numberOfSfbLong);
          DUMP(s->numberOfSfbShort);
          DUMP(s->sfbOffsetLong != NULL);
          DUMP(s->sfbOffsetShort != NULL);
          if (s->sfbOffsetLong) DUMP_ARR(s->sfbOffsetLong, s->numberOfSfbLong + 1);
          if (s->sfbOffsetShort) DUMP_ARR(s->sfbOffsetShort, s->numberOfSfbShort + 1);
        }
      }
      break;
    case 6:
      DUMP(AACcodeBookDescriptionSCL.Dimension);
      DUMP(AACcodeBookDescriptionSCL.numBits);
      DUMP(AACcodeBookDescriptionSCL.Offset);
      for (int i = 0; i < 65; i++) DUMP_ARR(AACcodeBookDescriptionSCL.CodeBook[i], 4);
      break;
    case 7:
      for (int i = 0; i < 13; i++) DUMP_ARR(tns_max_bands_tbl[i], 2);
      break;
    case 8:
      DUMP_ARR(FDKaacDec_tnsCoeff3, 8);
      break;
    case 9:
      DUMP_ARR(FDKaacDec_tnsCoeff4, 16);
      break;
    case 10:
      DUMP_ARR(AacDec_randomSign, AAC_NF_NO_RANDOM_VAL / 16);
      break;
    case 11:
      for (int i = 0; i < 20; i++) DUMP_ARR(mdst_filt_coef_curr[i], 3);
      break;
    case 12:
      for (int i = 0; i < 6; i++) DUMP_ARR(mdst_filt_coef_prev[i], 4);
      break;
  }
  return n;
}

/* getSamplingRateInfo(); out: error, samplingRateIndex, samplingRate, numbers of bands, offset
   tables (numberOfBands + 1 values each, if not NULL). */
INT cref_core_get_sampling_rate_info(UINT samplesPerFrame, UINT samplingRateIndex, UINT samplingRate, INT* out) {
  SamplingRateInfo t;
  FDKmemclear(&t, sizeof(t));
  INT n = 0;
  AAC_DECODER_ERROR err = getSamplingRateInfo(&t, samplesPerFrame, samplingRateIndex, samplingRate);
  DUMP(err);
  DUMP(t.samplingRateIndex);
  DUMP(t.samplingRate);
  DUMP(t.NumberOfScaleFactorBands_Long);
  DUMP(t.NumberOfScaleFactorBands_Short);
  DUMP(t.ScaleFactorBands_Long != NULL);
  DUMP(t.ScaleFactorBands_Short != NULL);
  if (t.ScaleFactorBands_Long) DUMP_ARR(t.ScaleFactorBands_Long, t.NumberOfScaleFactorBands_Long + 1);
  if (t.ScaleFactorBands_Short) DUMP_ARR(t.ScaleFactorBands_Short, t.NumberOfScaleFactorBands_Short + 1);
  return n;
}

/* Builds the SamplingRateInfo of sfbOffsetTables[0][srIdx]. */
static void sr_info(SamplingRateInfo* t, INT srIdx) {
  FDKmemclear(t, sizeof(*t));
  getSamplingRateInfo(t, 1024, srIdx, 48000);
}

/* IcsRead() on bitstream bs with the ics info loaded from ics[12] (CIcsInfo field order of
   cref_core_ch_dump()); the resulting ics info is written back. Returns the error code. */
INT cref_core_ics_read(void* bs, INT* ics, INT srIdx, UINT flags, INT maxSfbOnly) {
  SamplingRateInfo t;
  sr_info(&t, srIdx);
  CIcsInfo info;
  for (int i = 0; i < 8; i++) info.WindowGroupLength[i] = (UCHAR)ics[i];
  info.WindowGroups = (UCHAR)ics[8];
  info.Valid = (UCHAR)ics[9];
  info.WindowShape = (UCHAR)ics[10];
  info.WindowSequence = (BLOCK_TYPE)ics[11];
  info.MaxSfBands = (UCHAR)ics[12];
  info.max_sfb_ste = (UCHAR)ics[13];
  info.ScaleFactorGrouping = (UCHAR)ics[14];
  info.TotalSfBands = (UCHAR)ics[15];
  AAC_DECODER_ERROR err = maxSfbOnly ? IcsReadMaxSfb((HANDLE_FDK_BITSTREAM)bs, &info, &t)
                                     : IcsRead((HANDLE_FDK_BITSTREAM)bs, &info, &t, flags);
  for (int i = 0; i < 8; i++) ics[i] = info.WindowGroupLength[i];
  ics[8] = info.WindowGroups;
  ics[9] = info.Valid;
  ics[10] = info.WindowShape;
  ics[11] = info.WindowSequence;
  ics[12] = info.MaxSfBands;
  ics[13] = info.max_sfb_ste;
  ics[14] = info.ScaleFactorGrouping;
  ics[15] = info.TotalSfBands;
  return err;
}

}  // extern "C"
