// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Reference values of the floating point computations of the 32 bit x86 build (x87), for
// scripts/x87/x87_check.py. Built with the compiler and options of the reference (clang -m32 -O3), it
// prints the results of the fixed-point primitives that use floating point (sqrtFixp(),
// invSqrtNorm2(), invFixp()) and of the 64 bit multiplications for random and edge-case inputs
// ("P" lines), and of the crossfade of mpeghdecoder.cpp (a copy of fade(), "F" lines).

#include "common_fix.h"
#include "fixpoint_math.h"

#include <cmath>
#include <cstdio>

static unsigned long long rng = 0x9E3779B97F4A7C15ull;
static unsigned rnd() {
  rng ^= rng << 13;
  rng ^= rng >> 7;
  rng ^= rng << 17;
  return (unsigned)(rng >> 11);
}

__attribute__((noinline)) static void primitives(int op, long long q) {
  INT e1;
  FIXP_DBL r1 = invSqrtNorm2((FIXP_DBL)op, &e1);
  int e2 = (int)(rnd() % 64) - 32;
  int e2_in = e2;
  FIXP_DBL r2 = invFixp((FIXP_DBL)op, &e2);
  FIXP_DBL r3 = (op >= 0) ? sqrtFixp((FIXP_DBL)op) : 0;
  FIXP_DBL r4 = invFixp((FIXP_DBL)op);
  INT64 r5 = fMultDiv2((INT64)q, (INT64)op * 7919);
  INT64 r6 = fMultDiv2((INT64)q, (FIXP_DBL)op);
  printf("P %d %lld %d %d %d %d %d %d %d %lld %lld\n", op, q, e2_in, (int)r1, (int)e1, (int)r2,
         e2, (int)r3, (int)r4, r5, r6);
}

/* fade() of mpeghdecoder.cpp */
__attribute__((noinline)) static void fade(int32_t* sample, int index, int fadelen, bool fadein) {
  float factor = 0;
  if (fadein) {
    factor = (float)(-cos(M_PI * (float)index / (float)fadelen) + 1.0f) / 2.0f;
  } else {
    factor = (float)(cos(M_PI * (float)(index + 1) / (float)fadelen) + 1) / 2.0f;
  }
  *sample = (int32_t)(*sample * factor);
}

int main() {
  const int edge[] = {0, 1, 2, 3, -1, -2, 0x7fffffff, (int)0x80000000, 0x40000000, 0x3fffffff,
                      46341, 65536, 12345};
  for (int v : edge) primitives(v, 0x123456789abcdefll);
  for (int i = 0; i < 200000; i++) {
    int op = (int)rnd() >> (rnd() % 31);
    long long q = ((long long)rnd() << 32) ^ rnd() ^ ((long long)(int)rnd() << 40);
    primitives(op, q);
  }
  const int32_t samples[] = {8388607, -8388608, 123457, -7777777, 4194305};
  for (int ch = 1; ch <= 24; ch++) {
    int fadelen = 128 * ch;
    for (int i = 0; i < fadelen; i++)
      for (int fadein = 0; fadein < 2; fadein++)
        for (int32_t v : samples) {
          int32_t s = v;
          fade(&s, i, fadelen, fadein);
          printf("F %d %d %d %d %d\n", v, i, fadelen, fadein, s);
        }
  }
  return 0;
}
