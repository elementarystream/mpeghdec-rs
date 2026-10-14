// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libArithCoding ac_arith_coder.h.

#include "ac_arith_coder.h"

extern "C" {

void* cref_arco_create() { return CArco_Create(); }
void cref_arco_destroy(void* h) { CArco_Destroy((CArcoData*)h); }
void cref_arco_get_state(void* h, SHORT* numberLinesPrev, UCHAR* c_prev) {
  CArcoData* p = (CArcoData*)h;
  *numberLinesPrev = p->m_numberLinesPrev;
  FDKmemcpy(c_prev, p->c_prev, sizeof(p->c_prev));
}
void cref_arco_set_state(void* h, SHORT numberLinesPrev, const UCHAR* c_prev) {
  CArcoData* p = (CArcoData*)h;
  p->m_numberLinesPrev = numberLinesPrev;
  FDKmemcpy(p->c_prev, c_prev, sizeof(p->c_prev));
}
INT cref_arco_decode(void* h, void* bs, FIXP_DBL* spectrum, INT lg, INT lg_max, INT reset) {
  return CArco_DecodeArithData((CArcoData*)h, (HANDLE_FDK_BITSTREAM)bs, spectrum, lg, lg_max, reset);
}
INT cref_ari_decode_14bits(void* bs, INT* st, const SHORT* c_freq, INT cfl) {
  Tastat s;
  s.low = st[0];
  s.high = st[1];
  s.vobf = st[2];
  s.bits_to_follow = st[3];
  INT r = ari_decode_14bits((HANDLE_FDK_BITSTREAM)bs, &s, c_freq, cfl);
  st[0] = s.low;
  st[1] = s.high;
  st[2] = s.vobf;
  st[3] = s.bits_to_follow;
  return r;
}

}  // extern "C"
