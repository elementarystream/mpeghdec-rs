// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK FDK_tools_rom.h, FDK_trigFcts.h, scramble.h, fft_rad2.h, fft.h, rfft.h and
// dct.h.

#include "FDK_tools_rom.h"
#include "FDK_trigFcts.h"
#include "scramble.h"
#include "fft_rad2.h"
#include "fft.h"
#include "rfft.h"
#include "dct.h"

// Identifies a FIXP_SPK table by its address (see cref_rom_table()). -1 for NULL, -2 for unknown.
static INT fdk_rom_table_id(const FIXP_SPK* p);

extern "C" {

// Table access by id: 0 SineTable512, 1 SineTable1024, 2 SineWindow128, 3 SineWindow256,
// 4 SineWindow512, 5 SineWindow1024, 6 KBDWindow64, 7 KBDWindow128, 8 KBDWindow256,
// 9 KBDWindow1024, 10 sin_twiddle_L64. KBDWindow64/256 have internal linkage and are reached via
// windowSlopes.
const SHORT* cref_rom_table(INT id) {
  const FIXP_SPK* t = NULL;
  switch (id) {
    case 0: t = SineTable512; break;
    case 1: t = SineTable1024; break;
    case 2: t = SineWindow128; break;
    case 3: t = SineWindow256; break;
    case 4: t = SineWindow512; break;
    case 5: t = SineWindow1024; break;
    case 6: t = windowSlopes[1][0][0]; break;
    case 7: t = KBDWindow128; break;
    case 8: t = windowSlopes[1][0][2]; break;
    case 9: t = KBDWindow1024; break;
    case 10: t = sin_twiddle_L64; break;
  }
  return (const SHORT*)t;
}
INT cref_rom_table_id(const SHORT* p) { return fdk_rom_table_id((const FIXP_SPK*)p); }
INT cref_windowSlopes(INT shape, INT raster, INT idx) { return fdk_rom_table_id(windowSlopes[shape][raster][idx]); }
INT cref_FDKgetWindowSlope(INT length, INT shape) { return fdk_rom_table_id(FDKgetWindowSlope(length, shape)); }

// Bitstream element lists
void* cref_getBitstreamElementList(INT aot, INT epConfig, INT nChannels, INT layer, UINT elFlags) {
  return (void*)getBitstreamElementList((AUDIO_OBJECT_TYPE)aot, (SCHAR)epConfig, (UCHAR)nChannels, (UCHAR)layer, elFlags);
}
INT cref_el_id(void* el, INT i) { return (INT)((const element_list_t*)el)->id[i]; }
void* cref_el_next(void* el, INT k) { return (void*)((const element_list_t*)el)->next[k]; }

// FDK_trigFcts.h
INT cref_fixp_cos(INT x, INT scale) { return fixp_cos(x, scale); }
INT cref_fixp_sin(INT x, INT scale) { return fixp_sin(x, scale); }
void cref_fixp_cos_sin(INT* out, INT x, INT scale) { fixp_cos_sin(x, scale, &out[0], &out[1]); }
void cref_inline_fixp_cos_sin(INT* out, INT x1, INT x2, INT scale) { inline_fixp_cos_sin(x1, x2, scale, out); }

// scramble.h, fft_rad2.h, fft.h, rfft.h
void cref_scramble(INT* x, INT length) { scramble(x, length); }
void cref_dit_fft(INT* x, INT ldn, INT table) {
  if (table == 0) {
    dit_fft(x, ldn, SineTable512, 512);
  } else {
    dit_fft(x, ldn, SineTable1024, 1024);
  }
}
void cref_fft(INT length, INT* x, INT* scalefactor) { fft(length, x, scalefactor); }
INT cref_rfft(UINT fftLength, INT* x, UINT bufferLen, INT* scalefactor) { return rfft(fftLength, x, bufferLen * sizeof(FIXP_DBL), scalefactor); }
INT cref_irfft(UINT fftLength, INT* x, UINT bufferLen, INT* scalefactor) { return irfft(fftLength, x, bufferLen * sizeof(FIXP_DBL), scalefactor); }
INT cref_getSineTab(UINT length, UINT* step) {
  const FIXP_SPK* tab;
  getSineTab(length, &tab, step);
  return fdk_rom_table_id(tab);
}

// dct.h
void cref_dct_getTables(INT* out, INT length) {
  const FIXP_SPK *twiddle, *sin_twiddle;
  int sin_step;
  dct_getTables(&twiddle, &sin_twiddle, &sin_step, length);
  out[0] = fdk_rom_table_id(twiddle);
  out[1] = fdk_rom_table_id(sin_twiddle);
  out[2] = sin_step;
}
void cref_dct_II(INT* x, INT* tmp, INT L, INT* e) { dct_II(x, tmp, L, e); }
void cref_dct_III(INT* x, INT* tmp, INT L, INT* e) { dct_III(x, tmp, L, e); }
void cref_dst_III(INT* x, INT* tmp, INT L, INT* e) { dst_III(x, tmp, L, e); }
void cref_dct_IV(INT* x, INT L, INT* e) { dct_IV(x, L, e); }
void cref_dst_IV(INT* x, INT L, INT* e) { dst_IV(x, L, e); }

}  // extern "C"

static INT fdk_rom_table_id(const FIXP_SPK* p) {
  if (p == NULL) return -1;
  for (INT id = 0; id <= 10; id++) {
    if ((const SHORT*)p == cref_rom_table(id)) return id;
  }
  return -2;
}
