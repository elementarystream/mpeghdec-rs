// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK FDK_bitstream.h / FDK_bitbuffer.h.

#include "FDK_bitstream.h"

extern "C" {

void* cref_bs_new(UCHAR* buf, UINT size, UINT validBits, INT config) {
  FDK_BITSTREAM* bs = (FDK_BITSTREAM*)FDKcalloc(1, sizeof(FDK_BITSTREAM));
  FDKinitBitStream(bs, buf, size, validBits, (FDK_BS_CFG)config);
  return bs;
}
void cref_bs_free(void* h) { FDKfree(h); }
void cref_bs_state(void* h, UINT* out) {
  FDK_BITSTREAM* bs = (FDK_BITSTREAM*)h;
  out[0] = bs->CacheWord;
  out[1] = bs->BitsInCache;
  out[2] = bs->hBitBuf.BitNdx;
  out[3] = (UINT)bs->hBitBuf.ValidBits;
  out[4] = bs->hBitBuf.ReadOffset;
  out[5] = bs->hBitBuf.WriteOffset;
}
UINT cref_bs_readBits(void* h, UINT n) { return FDKreadBits((FDK_BITSTREAM*)h, n); }
UINT cref_bs_readBit(void* h) { return FDKreadBit((FDK_BITSTREAM*)h); }
UINT cref_bs_read2Bits(void* h) { return FDKread2Bits((FDK_BITSTREAM*)h); }
UINT cref_bs_readBitsBwd(void* h, UINT n) { return FDKreadBitsBwd((FDK_BITSTREAM*)h, n); }
UINT cref_bs_escapedValue(void* h, INT a, INT b, INT c) { return escapedValue((FDK_BITSTREAM*)h, a, b, c); }
UINT cref_bs_getBits(void* h, UINT n) { return FDKgetBits((FDK_BITSTREAM*)h, n); }
UINT cref_bs_writeBits(void* h, UINT v, UINT n) { return FDKwriteBits((FDK_BITSTREAM*)h, v, n); }
UINT cref_bs_writeBitsBwd(void* h, UINT v, UINT n) { return FDKwriteBitsBwd((FDK_BITSTREAM*)h, v, n); }
UINT cref_bs_writeEscapedValue(void* h, UINT v, UINT a, UINT b, UINT c) { return FDKwriteEscapedValue((FDK_BITSTREAM*)h, v, a, b, c); }
void cref_bs_syncCache(void* h) { FDKsyncCache((FDK_BITSTREAM*)h); }
void cref_bs_syncCacheBwd(void* h) { FDKsyncCacheBwd((FDK_BITSTREAM*)h); }
void cref_bs_byteAlign(void* h, UINT anchor) { FDKbyteAlign((FDK_BITSTREAM*)h, anchor); }
void cref_bs_pushBack(void* h, UINT n) { FDKpushBack((FDK_BITSTREAM*)h, n); }
void cref_bs_pushFor(void* h, UINT n) { FDKpushFor((FDK_BITSTREAM*)h, n); }
void cref_bs_pushBiDirectional(void* h, INT n) { FDKpushBiDirectional((FDK_BITSTREAM*)h, n); }
INT cref_bs_getValidBits(void* h) { return FDKgetValidBits((FDK_BITSTREAM*)h); }
INT cref_bs_getFreeBits(void* h) { return FDKgetFreeBits((FDK_BITSTREAM*)h); }
void cref_bs_feedBuffer(void* h, const UCHAR* in, UINT size, UINT* valid) { FDKfeedBuffer((FDK_BITSTREAM*)h, in, size, valid); }
void cref_bs_fetchBuffer(void* h, UCHAR* out, UINT* n) { FDKfetchBuffer((FDK_BITSTREAM*)h, out, n); }
void cref_bs_copyBuffer(void* dst, void* src, UINT* valid) { FDKcopyBuffer((FDK_BITSTREAM*)dst, (FDK_BITSTREAM*)src, valid); }

}  // extern "C"
