// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK FDK_lpc.h.

#include "FDK_lpc.h"

// Defined in FDK_lpc.cpp (arm variant) but not declared in FDK_lpc.h.
void CLpc_SynthesisLattice(FIXP_DBL* signal, const int signal_size, const int signal_e,
                           const int signal_e_out, const int inc, const FIXP_SGL* coeff,
                           const int order, FIXP_DBL* state);

extern "C" {

void cref_CLpc_SynthesisLattice_SGL(INT* signal, INT size, INT signal_e, INT signal_e_out, INT inc, const SHORT* coeff, INT order, INT* state) {
  CLpc_SynthesisLattice(signal, size, signal_e, signal_e_out, inc, (const FIXP_SGL*)coeff, order, state);
}
void cref_CLpc_SynthesisLattice_DBL(INT* signal, INT size, INT signal_e, INT signal_e_out, INT inc, const INT* coeff, INT order, INT* state) {
  CLpc_SynthesisLattice(signal, size, signal_e, signal_e_out, inc, (const FIXP_DBL*)coeff, order, state);
}
void cref_CLpc_Synthesis_DBL(INT* signal, INT size, INT inc, const INT* coeff, INT coeff_e, INT order, INT* state, INT* stateIndex) {
  CLpc_Synthesis(signal, size, inc, (const FIXP_LPC_TNS*)coeff, coeff_e, order, state, stateIndex);
}
void cref_CLpc_Synthesis_SGL(INT* signal, INT size, INT inc, const SHORT* coeff, INT coeff_e, INT order, INT* state, INT* stateIndex) {
  CLpc_Synthesis(signal, size, inc, (const FIXP_LPC*)coeff, coeff_e, order, state, stateIndex);
}
INT cref_CLpc_AutoCorr(const INT* signal, INT signal_e, INT* acorr, INT* acorr_e, INT size, INT order) {
  return CLpc_AutoCorr(signal, signal_e, acorr, acorr_e, size, order);
}
INT cref_CLpc_ParcorToLpc_DBL(const INT* refl, INT* lpc, INT n, INT64* work) {
  return CLpc_ParcorToLpc((const FIXP_LPC_TNS*)refl, (FIXP_LPC_TNS*)lpc, n, work);
}
INT cref_CLpc_ParcorToLpc_SGL(const SHORT* refl, SHORT* lpc, INT n, INT64* work) {
  return CLpc_ParcorToLpc((const FIXP_LPC*)refl, (FIXP_LPC*)lpc, n, work);
}
void cref_CLpc_AutoToParcor(INT* acorr, INT acorr_e, SHORT* refl, INT order, INT* gain, INT useGain) {
  CLpc_AutoToParcor(acorr, acorr_e, (FIXP_LPC*)refl, order, useGain ? &gain[0] : NULL, useGain ? &gain[1] : NULL);
}
void cref_CLpc_AutoToLpcIGF(INT* lpc, INT* lpc_e, const INT* R, INT R_e, INT m) {
  CLpc_AutoToLpcIGF(lpc, lpc_e, R, R_e, m, NULL);
}
void cref_CLpc_AutoToLpc(SHORT* lpc, INT* lpc_e, const INT* R, INT R_e, INT m, INT is_ltpf) {
  CLpc_AutoToLpc((FIXP_LPC*)lpc, lpc_e, R, R_e, m, NULL, is_ltpf);
}

}  // extern "C"
