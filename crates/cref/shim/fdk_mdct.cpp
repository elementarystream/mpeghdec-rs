// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK mdct.h. Window slopes are passed as (length, shape) and looked up with
// FDKgetWindowSlope(); window slope pointers are reported as table ids (see cref_rom_table()).

#include "mdct.h"
#include "FDK_tools_rom.h"

extern "C" INT cref_rom_table_id(const SHORT* p);

typedef struct {
  mdct_t mdct;
  FIXP_DBL overlap[2048];
} CREF_MDCT;

static const FIXP_WTP* win(INT length, INT shape) {
  return (length > 0) ? FDKgetWindowSlope(length, shape) : NULL;
}

extern "C" {

void* cref_mdct_new(void) { return FDKcalloc(1, sizeof(CREF_MDCT)); }
void cref_mdct_free(void* h) { FDKfree(h); }
void cref_mdct_init(void* h, INT ovSize) {
  CREF_MDCT* m = (CREF_MDCT*)h;
  mdct_init(&m->mdct, m->overlap, ovSize);
}
INT* cref_mdct_overlap(void* h) { return ((CREF_MDCT*)h)->overlap; }
void cref_mdct_state(void* h, INT* out) {
  mdct_t* m = &((CREF_MDCT*)h)->mdct;
  out[0] = m->prev_tl;
  out[1] = m->prev_nr;
  out[2] = m->prev_fr;
  out[3] = m->ov_offset;
  out[4] = m->ov_size;
  out[5] = m->prevAliasSymmetry;
  out[6] = m->prevPrevAliasSymmetry;
  out[7] = cref_rom_table_id((const SHORT*)m->prev_wrs);
  out[8] = m->pAsymOvlp != NULL;
}
void cref_mdct_set(void* h, INT prevAliasSymmetry, INT prevPrevAliasSymmetry, INT ovOffset) {
  mdct_t* m = &((CREF_MDCT*)h)->mdct;
  m->prevAliasSymmetry = prevAliasSymmetry;
  m->prevPrevAliasSymmetry = prevPrevAliasSymmetry;
  m->ov_offset = ovOffset;
}
void cref_mdct_set_asym(void* h, INT* asym) { ((CREF_MDCT*)h)->mdct.pAsymOvlp = asym; }

INT cref_mdct_block(void* h, const INT* timeData, INT noInSamples, INT* mdctData, INT nSpec, INT tl, INT wrsLen, INT wrsShape, INT fr, SHORT* e) {
  return mdct_block(&((CREF_MDCT*)h)->mdct, timeData, noInSamples, mdctData, nSpec, tl, win(wrsLen, wrsShape), fr, e);
}
void cref_imdct_gain(INT* gain_m, INT* gain_e, INT tl) { imdct_gain(gain_m, gain_e, tl); }
INT cref_imdct_drain(void* h, INT* out, INT room) { return imdct_drain(&((CREF_MDCT*)h)->mdct, out, room); }
INT cref_imdct_copy_ov_and_nr(void* h, INT* out, INT n) { return imdct_copy_ov_and_nr(&((CREF_MDCT*)h)->mdct, out, n); }
INT cref_imdct_copy_full(void* h, INT* out, INT n) { return imdct_copy_full(&((CREF_MDCT*)h)->mdct, out, n); }
void cref_imdct_adapt_parameters(void* h, INT* fl, INT* nl, INT tl, INT wlsLen, INT wlsShape, INT noOutSamples) {
  imdct_adapt_parameters(&((CREF_MDCT*)h)->mdct, fl, nl, tl, win(wlsLen, wlsShape), noOutSamples);
}
INT cref_imlt_block(void* h, INT* output, INT* spectrum, const SHORT* sf, INT nSpec, INT noOutSamples, INT tl, INT wlsLen, INT wlsShape, INT fl, INT wrsLen, INT wrsShape, INT fr, INT gain, INT flags) {
  return imlt_block(&((CREF_MDCT*)h)->mdct, output, spectrum, sf, nSpec, noOutSamples, tl, win(wlsLen, wlsShape), fl, win(wrsLen, wrsShape), fr, gain, flags);
}
INT cref_imdct_block(void* h, INT* output, INT* spectrum, const SHORT* sf, INT nSpec, INT noOutSamples, INT tl, INT wlsLen, INT wlsShape, INT fl, INT wrsLen, INT wrsShape, INT fr, INT gain) {
  return imdct_block(&((CREF_MDCT*)h)->mdct, output, spectrum, sf, nSpec, noOutSamples, tl, win(wlsLen, wlsShape), fl, win(wrsLen, wrsShape), fr, gain);
}
INT cref_imdst_block(void* h, INT* output, INT* spectrum, const SHORT* sf, INT nSpec, INT noOutSamples, INT tl, INT wlsLen, INT wlsShape, INT fl, INT wrsLen, INT wrsShape, INT fr, INT gain) {
  return imdst_block(&((CREF_MDCT*)h)->mdct, output, spectrum, sf, nSpec, noOutSamples, tl, win(wlsLen, wlsShape), fl, win(wrsLen, wrsShape), fr, gain);
}

}  // extern "C"
