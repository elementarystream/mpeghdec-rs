// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Shared definitions of the libMpeghDec C shims (see core_common.cpp).

#ifndef CREF_CORE_COMMON_H
#define CREF_CORE_COMMON_H

#include "channelinfo.h"
#include "aac_rom.h"

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

/* Channel info together with its dynamic data (pDynData is allocated separately in C++). */
typedef struct {
  CAacDecoderChannelInfo ch;
  CAacDecoderDynamicData dyn;
} CREF_CH;

/* Static channel info with the buffers allocated for it in CAacDecoder_Init(). */
typedef struct {
  CAacDecoderStaticChannelInfo st;
  FIXP_DBL concealSpec[1024];
  FIXP_DBL overlap[OverlapBufferSize];
  CArcoData arco;
} CREF_ST;

/* Element common data (pComStaticData and what it points to). */
typedef struct {
  CAacDecoderCommonStaticData com;
  CJointStereoData jsd;
  CCplxPredictionData cpd;
} CREF_COM;

#endif /* CREF_CORE_COMMON_H */
