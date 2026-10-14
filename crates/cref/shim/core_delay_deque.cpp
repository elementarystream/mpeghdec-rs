// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libMpeghDec FDK_delay.cpp and deque.cpp.

#include "channelinfo.h"
#include "FDK_delay.h"
#include "deque.h"

extern "C" {

/* ---------------------------------------------------------------------------------------------
   FDK_delay
   --------------------------------------------------------------------------------------------- */

/* FDK_Delay_Create() on a structure filled with garbage, then FDK_Delay_Destroy(). out receives
   the return value, delay_line != NULL, delay, num_channels and the number of non-zero values
   of the delay line after the creation, and delay_line != NULL, delay, num_channels after the
   destruction. */
void cref_core_delay_create_destroy(UINT delay, UINT num_channels, INT* out) {
  FDK_SignalDelay d;
  FDKmemset(&d, 0xA5, sizeof(d));
  out[0] = FDK_Delay_Create(&d, (USHORT)delay, (UCHAR)num_channels);
  out[1] = d.delay_line != NULL;
  out[2] = d.delay;
  out[3] = d.num_channels;
  INT nz = 0;
  if (d.delay_line != NULL) {
    for (INT i = 0; i < (INT)d.num_channels * (INT)d.delay; i++) nz += d.delay_line[i] != 0;
  }
  out[4] = nz;
  FDK_Delay_Destroy(&d);
  out[5] = d.delay_line != NULL;
  out[6] = d.delay;
  out[7] = d.num_channels;
}

/* ---------------------------------------------------------------------------------------------
   deque (a deque structure allocated with cref_core_deque_new(); element positions returned
   as block indices, -1 for NULL)
   --------------------------------------------------------------------------------------------- */

static INT deque_block_index(const deque* q, const void* p) {
  if (p == NULL) return -1;
  return (INT)(((const unsigned char*)p - (const unsigned char*)q->data) / q->block_size);
}

void* cref_core_deque_new(void) { return FDKcalloc(1, sizeof(deque)); }
/* Frees the structure itself (call cref_core_deque_free() first to free the data). */
void cref_core_deque_delete(void* q) { FDKfree(q); }

/* State: data != NULL, block_size, first, last, size, max, full. */
void cref_core_deque_state(void* p, UINT* out) {
  deque* q = (deque*)p;
  out[0] = q->data != NULL;
  out[1] = q->block_size;
  out[2] = q->first;
  out[3] = q->last;
  out[4] = q->size;
  out[5] = q->max;
  out[6] = q->full;
}
/* Copies the whole buffer (max * block_size bytes). */
void cref_core_deque_data(void* p, UCHAR* out) {
  deque* q = (deque*)p;
  FDKmemcpy(out, q->data, q->max * q->block_size);
}

INT cref_core_deque_alloc(void* q, UINT length, UINT block_size) { return deque_alloc((deque*)q, length, block_size); }
void cref_core_deque_free(void* q) { deque_free((deque*)q); }
INT cref_core_deque_push_back(void* q, UCHAR* data) { return deque_push_back((deque*)q, data); }
INT cref_core_deque_bulk_push_back(void* q, UCHAR* data, UINT numData) { return deque_bulk_push_back((deque*)q, data, numData); }
INT cref_core_deque_bulk_push_back_zeros(void* q, UINT numZeros) { return deque_bulk_push_back_zeros((deque*)q, numZeros); }
/* deque_pop_back(): returns the block index of the result, the block is copied to out. */
INT cref_core_deque_pop_back(void* q, UCHAR* out) {
  void* r = deque_pop_back((deque*)q);
  if (r != NULL) FDKmemcpy(out, r, ((deque*)q)->block_size);
  return deque_block_index((deque*)q, r);
}
/* deque_pop_front(): returns the block index of the result, the block is copied to out. */
INT cref_core_deque_pop_front(void* q, UCHAR* out) {
  void* r = deque_pop_front((deque*)q);
  if (r != NULL) FDKmemcpy(out, r, ((deque*)q)->block_size);
  return deque_block_index((deque*)q, r);
}
INT cref_core_deque_bulk_pop_front_copy(void* q, UCHAR* destination, UINT numData) { return deque_bulk_pop_front_copy((deque*)q, destination, numData); }
INT cref_core_deque_bulk_pop_front(void* q, UINT numData) { return deque_bulk_pop_front((deque*)q, numData); }
INT cref_core_deque_full(void* q) { return deque_full((deque*)q); }
INT cref_core_deque_empty(void* q) { return deque_empty((deque*)q); }
UINT cref_core_deque_size(void* q) { return deque_size((deque*)q); }
UINT cref_core_deque_space(void* q) { return deque_space((deque*)q); }
void cref_core_deque_clear(void* q) { deque_clear((deque*)q); }
INT cref_core_deque_at(void* q, UINT index) { return deque_block_index((deque*)q, deque_at((deque*)q, index)); }
INT cref_core_deque_front(void* q) { return deque_block_index((deque*)q, deque_front((deque*)q)); }
INT cref_core_deque_back(void* q) { return deque_block_index((deque*)q, deque_back((deque*)q)); }

}  // extern "C"
