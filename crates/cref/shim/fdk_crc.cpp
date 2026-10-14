// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK FDK_crc.h. Bitstream handles come from shim/fdk_bitstream.cpp.

#include "FDK_crc.h"

extern "C" {

void* cref_crc_new(void) { return FDKcalloc(1, sizeof(FDK_CRCINFO)); }
void cref_crc_free(void* h) { FDKfree(h); }
void cref_crc_init(void* h, UINT poly, UINT startValue, UINT len) { FDKcrcInit((FDK_CRCINFO*)h, poly, startValue, len); }
void cref_crc_reset(void* h) { FDKcrcReset((FDK_CRCINFO*)h); }
INT cref_crc_startReg(void* h, void* bs, INT mBits) { return FDKcrcStartReg((FDK_CRCINFO*)h, (FDK_BITSTREAM*)bs, mBits); }
INT cref_crc_endReg(void* h, void* bs, INT reg) { return FDKcrcEndReg((FDK_CRCINFO*)h, (FDK_BITSTREAM*)bs, reg); }
UINT cref_crc_getCRC(void* h) { return FDKcrcGetCRC((FDK_CRCINFO*)h); }
/* out: crcValue, crcPoly, crcMask, startValue, crcLen, regStart, regStop, hasLookup,
        then 4 values per region (isActive, maxBits, bitBufCntBits, validBits) */
void cref_crc_state(void* h, INT* out) {
  FDK_CRCINFO* c = (FDK_CRCINFO*)h;
  out[0] = c->crcValue;
  out[1] = c->crcPoly;
  out[2] = c->crcMask;
  out[3] = c->startValue;
  out[4] = c->crcLen;
  out[5] = (INT)c->regStart;
  out[6] = (INT)c->regStop;
  out[7] = c->pCrcLookup ? (INT)c->pCrcLookup[1] : -1;
  for (int i = 0; i < MAX_CRC_REGS; i++) {
    out[8 + 4 * i + 0] = c->crcRegData[i].isActive;
    out[8 + 4 * i + 1] = c->crcRegData[i].maxBits;
    out[8 + 4 * i + 2] = c->crcRegData[i].bitBufCntBits;
    out[8 + 4 * i + 3] = c->crcRegData[i].validBits;
  }
}

}  // extern "C"
