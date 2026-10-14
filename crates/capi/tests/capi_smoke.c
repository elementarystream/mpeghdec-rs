// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: a smoke test of the C ABI (scripts/capi-test.sh) against the unchanged headers.

#include <mpeghdec/mpeghUIManager.h>
#include <mpeghdec/mpeghdecoder.h>
#include <stddef.h>
#include <stdio.h>

#define CHECK(c)                                                  \
  do {                                                            \
    if (!(c)) {                                                   \
      fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #c); \
      return 1;                                                   \
    }                                                             \
  } while (0)

int main(void) {
  /* the layout the Rust side uses (repr(C)): uint64_t is 4 byte aligned on 32 bit x86 */
  const size_t a = _Alignof(uint64_t);
  CHECK(offsetof(MPEGH_DECODER_OUTPUT_INFO, ticks) == (12 + a - 1) / a * a);
  CHECK(offsetof(MPEGH_DECODER_OUTPUT_INFO, loudness) ==
        offsetof(MPEGH_DECODER_OUTPUT_INFO, ticks) + 16);
  CHECK(offsetof(MPEGH_DECODER_OUTPUT_INFO, isConcealed) ==
        offsetof(MPEGH_DECODER_OUTPUT_INFO, loudness) + 4);
  CHECK(sizeof(MPEGH_DECODER_OUTPUT_INFO) ==
        (offsetof(MPEGH_DECODER_OUTPUT_INFO, isConcealed) + 1 + a - 1) / a * a);

  /* unsupported layout, NULL pointers */
  CHECK(mpeghdecoder_init(8) == NULL);
  CHECK(mpeghdecoder_process(NULL, (const uint8_t*)"", 0, 0) == MPEGH_DEC_NULLPTR_ERROR);
  CHECK(mpeghdecoder_flush(NULL) == MPEGH_DEC_NULLPTR_ERROR);
  mpeghdecoder_destroy(NULL);

  HANDLE_MPEGH_DECODER_CONTEXT d = mpeghdecoder_init(2);
  CHECK(d != NULL);
  CHECK(mpeghdecoder_process(d, NULL, 1, 0) == MPEGH_DEC_NULLPTR_ERROR);
  CHECK(mpeghdecoder_setParam(d, MPEGH_DEC_PARAM_TARGET_REFERENCE_LEVEL, 200) ==
        MPEGH_DEC_UNSUPPORTED_PARAM);
  CHECK(mpeghdecoder_setParam(d, MPEGH_DEC_PARAM_TARGET_REFERENCE_LEVEL, 96) == MPEGH_DEC_OK);
  static int32_t out[24 * 3072];
  MPEGH_DECODER_OUTPUT_INFO info = {0};
  info.ticks = 1234;
  CHECK(mpeghdecoder_getSamples(d, out, 1, &info) == MPEGH_DEC_BUFFER_ERROR);
  CHECK(mpeghdecoder_getSamples(d, out, 2 * 3072, &info) == MPEGH_DEC_FEED_DATA);
  CHECK(info.numSamplesPerChannel == 0 && info.ticks == 1234);
  mpeghdecoder_destroy(d);

  HANDLE_MPEGH_UI_MANAGER ui = mpegh_UI_Manager_Open();
  CHECK(ui != NULL);
  unsigned int flags = 0;
  char xml[2048];
  CHECK(mpegh_UI_GetXmlSceneState(ui, NULL, 0, 0, &flags) == MPEGH_UI_INVALID_PARAM);
  CHECK(mpegh_UI_GetXmlSceneState(ui, xml, sizeof(xml), 0, &flags) == MPEGH_UI_OK);
  static unsigned short mem[1024];
  CHECK(mpegh_UI_SetPersistenceMemory(ui, (char*)mem + 1, 512) == MPEGH_UI_INVALID_PARAM);
  CHECK(mpegh_UI_SetPersistenceMemory(ui, mem, sizeof(mem)) == MPEGH_UI_OK_BUT_NO_VALID_DATA);
  CHECK(mpegh_UI_SetPersistenceMemory(ui, mem, sizeof(mem)) == MPEGH_UI_OK);
  void* block = NULL;
  unsigned short size = 0;
  CHECK(mpegh_UI_GetPersistenceMemory(ui, &block, &size) == MPEGH_UI_OK);
  CHECK(block == mem && size == sizeof(mem));
  mpegh_UI_Manager_Close(ui);

  printf("capi smoke test passed\n");
  return 0;
}
