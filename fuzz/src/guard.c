// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

/* Crash, hang and heap guard for the reference side of the differential fuzz targets
 * (fuzz/README.md).
 *
 * The C++ reference may trap or crash on corrupt inputs where the port has defined behavior
 * (divisions by zero, NULL dereferences, out-of-bounds accesses, endless loops). Such inputs are
 * not findings of the port, and they must not disturb the fuzzer process.
 *
 * Signals. Every reference API call runs through fz_guard_call(): a fatal signal inside the
 * call jumps back out of the reference (siglongjmp) and the call reports the signal number; the
 * target then destroys the handle under the guard. Only C frames and Rust frames without
 * destructors (the trampolines of src/guard.rs) lie between sigsetjmp() and siglongjmp().
 * Signals outside a guarded call go to the handler installed before (libFuzzer's), so crashes of
 * the port are reported as usual.
 *
 * Heap (Linux). The reference allocates from an arena of its own while it runs a guarded call:
 * malloc/calloc/realloc/free are interposed and route the calls of the reference thread inside a
 * guarded call to a bump allocator that is reset for every reference run (fz_ref_begin). Heap
 * overruns of the reference then only reach its own arena, a jump out of the reference never
 * leaves a malloc lock held, and heap memory that the reference reads without writing it (malloc
 * without initialization, the redzone after each block) holds the run's byte pattern, so the
 * recheck with other patterns also recognizes reads of uninitialized and out-of-allocation heap
 * memory; each pattern also moves the arena's start, so results that depend on heap addresses
 * change too. Freed blocks are not reused within a run. Elsewhere (macOS replays) the reference
 * uses the system heap.
 *
 * Budget. A CPU-time budget (ITIMER_VIRTUAL) bounds the reference part of one fuzz input. With
 * the arena, a call that exceeds it is left with siglongjmp (-1). Without the arena the budget is
 * soft (a jump could leave a malloc lock held): the call completes and the next call reports -1.
 *
 * Stack. Before each guarded call the stack below the caller is filled with the run's byte
 * pattern, so that reads of uninitialized stack memory by the reference give reproducible
 * values; on a difference the target reruns the reference with other patterns. */

#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#if defined(__linux__)
#include <sys/mman.h>
#define FZ_ARENA 1
#else
#define FZ_ARENA 0
#endif

static sigjmp_buf g_env;
/* The thread inside a guarded call: only it jumps out of a signal handler (a process-directed
   SIGVTALRM or a fault may arrive on another thread), and only its allocations go to the arena. */
static __thread volatile sig_atomic_t t_active;
static volatile sig_atomic_t g_expired = 0;
static int g_installed = 0;
static int g_pattern = 0;

static const int g_sigs[] = {SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT, SIGTRAP, SIGVTALRM};
#define NSIGS ((int)(sizeof(g_sigs) / sizeof(g_sigs[0])))
static struct sigaction g_old[NSIGS];

/* Alternate signal stack: a stack overflow in the reference must still reach the handler. */
static char g_altstack[1 << 18];

/* Codes of fz_guard_call besides signal numbers. */
#define FZ_TIMEOUT (-1)
#define FZ_ARENA_FULL (-2)

/* --- reference heap ----------------------------------------------------------------------- */
#if FZ_ARENA
extern void *__libc_malloc(size_t);
extern void *__libc_calloc(size_t, size_t);
extern void *__libc_realloc(void *, size_t);
extern void __libc_free(void *);

#define ARENA_SIZE ((size_t)4 << 30) /* reserved address space (MAP_NORESERVE) */
/* Every block is written when it is allocated (pattern or zeros), so the arena's use is resident
   memory; a reference run may use at most this much (libFuzzer's -rss_limit_mb is 4096 in
   fuzz/run.sh). Larger requests end the run as "arena full". */
#define ARENA_LIMIT ((size_t)1 << 30)
#define ARENA_KEEP ((size_t)64 << 20) /* resident memory kept between runs */
#define REDZONE 64
static char *g_arena;
static size_t g_used, g_peak, g_max_used;

struct hdr {
  size_t size;
  size_t pad;
};

static int in_arena(const void *p) {
  return g_arena && (const char *)p >= g_arena && (const char *)p < g_arena + ARENA_SIZE;
}

static void *arena_alloc(size_t n, int zero) {
  size_t need;
  struct hdr *h;
  char *p;
  if (!g_arena) {
    void *m = mmap(NULL, ARENA_SIZE, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (m == MAP_FAILED) return NULL;
    g_arena = (char *)m;
  }
  if (n > ARENA_LIMIT ||
      g_used + ((sizeof(struct hdr) + n + REDZONE + 15) & ~(size_t)15) > ARENA_LIMIT) {
    /* Not a finding: the reference asked for more than the arena holds. */
    if (t_active) {
      t_active = 0;
      siglongjmp(g_env, FZ_ARENA_FULL);
    }
    return NULL;
  }
  need = (sizeof(struct hdr) + n + REDZONE + 15) & ~(size_t)15;
  h = (struct hdr *)(g_arena + g_used);
  g_used += need;
  if (g_used > g_peak) g_peak = g_used;
  if (g_used > g_max_used) g_max_used = g_used;
  h->size = n;
  p = (char *)(h + 1);
  memset(p, zero ? 0 : g_pattern, n);
  memset(p + n, g_pattern, need - sizeof(struct hdr) - n);
  return p;
}

/* A fresh arena. Runs with another pattern also start at another offset (a multiple of 16
   bytes, 4112 per pattern step), so that results which depend on heap addresses change as well
   and are recognized. */
static void arena_reset(void) {
  if (g_arena && g_peak > ARENA_KEEP) {
    madvise(g_arena + ARENA_KEEP, g_peak - ARENA_KEEP, MADV_DONTNEED);
    g_peak = ARENA_KEEP;
  }
  g_used = (size_t)g_pattern * 4112;
}

void *malloc(size_t n) {
  if (t_active) return arena_alloc(n, 0);
  return __libc_malloc(n);
}

void *calloc(size_t a, size_t b) {
  if (t_active) {
    if (b && a > (size_t)-1 / b) return NULL;
    return arena_alloc(a * b, 1);
  }
  return __libc_calloc(a, b);
}

void free(void *p) {
  if (!p || in_arena(p)) return;
  __libc_free(p);
}

void *realloc(void *p, size_t n) {
  if (in_arena(p) || (!p && t_active)) {
    size_t old = p ? ((struct hdr *)p - 1)->size : 0;
    void *q = t_active ? arena_alloc(n, 0) : __libc_malloc(n);
    if (q && p) memcpy(q, p, old < n ? old : n);
    return q;
  }
  return __libc_realloc(p, n);
}
#endif

/* --- signals ------------------------------------------------------------------------------ */
static void chain(int sig, siginfo_t *si, void *uc) {
  int i;
  for (i = 0; i < NSIGS; i++) {
    if (g_sigs[i] != sig) continue;
    /* Hand the signal on for good: restore the previous action. */
    sigaction(sig, &g_old[i], NULL);
    if (g_old[i].sa_flags & SA_SIGINFO) {
      if (g_old[i].sa_sigaction) g_old[i].sa_sigaction(sig, si, uc);
      return;
    }
    if (g_old[i].sa_handler == SIG_IGN) return;
    if (g_old[i].sa_handler == SIG_DFL) {
      /* A synchronous fault repeats when the handler returns and then takes the default
         action; anything else is raised again (it is delivered after this handler). */
      if (sig != SIGSEGV && sig != SIGBUS && sig != SIGFPE && sig != SIGILL) raise(sig);
      return;
    }
    g_old[i].sa_handler(sig);
    return;
  }
}

static void leave(int code) {
  t_active = 0;
  siglongjmp(g_env, code);
}

static void handler(int sig, siginfo_t *si, void *uc) {
  if (sig == SIGVTALRM) {
    g_expired = 1;
    /* hard budget only where no jump can leave a lock held (see the top of the file) */
    if (FZ_ARENA && t_active) leave(FZ_TIMEOUT);
    return;
  }
  if (t_active) leave(sig);
  chain(sig, si, uc);
}

static void install(void) {
  struct sigaction sa;
  stack_t ss;
  int i;
  if (g_installed) return;
  g_installed = 1;
  memset(&ss, 0, sizeof(ss));
  ss.ss_sp = g_altstack;
  ss.ss_size = sizeof(g_altstack);
  ss.ss_flags = 0;
  sigaltstack(&ss, NULL);
  memset(&sa, 0, sizeof(sa));
  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO | SA_ONSTACK;
  sigemptyset(&sa.sa_mask);
  for (i = 0; i < NSIGS; i++) sigaction(g_sigs[i], &sa, &g_old[i]);
}

/* --- API ---------------------------------------------------------------------------------- */

/* Whether the reference has a heap of its own (1) or uses the system heap (0). */
int fz_ref_arena(void) { return FZ_ARENA; }

/* The largest arena use of a reference run so far, in bytes. */
uint64_t fz_ref_arena_max(void) {
#if FZ_ARENA
  return g_max_used;
#else
  return 0;
#endif
}

/* Starts one reference run: the byte pattern of uninitialized memory, the CPU-time budget
   (milliseconds; 0: none), a fresh arena. */
void fz_ref_begin(int64_t budget_ms, int pattern) {
  struct itimerval it;
  install();
  g_pattern = pattern & 0xff;
#if FZ_ARENA
  arena_reset();
#endif
  g_expired = 0;
  memset(&it, 0, sizeof(it));
  if (budget_ms > 0) {
    it.it_value.tv_sec = (time_t)(budget_ms / 1000);
    it.it_value.tv_usec = (suseconds_t)((budget_ms % 1000) * 1000);
  }
  setitimer(ITIMER_VIRTUAL, &it, NULL);
}

/* Ends the reference run: disarms the budget. */
void fz_ref_end(void) {
  struct itimerval it;
  memset(&it, 0, sizeof(it));
  setitimer(ITIMER_VIRTUAL, &it, NULL);
  g_expired = 0;
}

/* A copy of `n` bytes of `src` for a reference call, followed by `extra` bytes of the run's
   pattern. Inside a guarded call it comes from the arena (with its redzone), so reads past the
   end of an input buffer see the pattern; it is freed by fz_ref_free() (a no-op in the arena). */
void *fz_ref_copy(const void *src, size_t n, size_t extra) {
  unsigned char *p = (unsigned char *)malloc(n + extra ? n + extra : 1);
  if (!p) return NULL;
  if (n) memcpy(p, src, n);
  memset(p + n, g_pattern, extra);
  return p;
}

void fz_ref_free(void *p) { free(p); }

/* Fills 256 KiB of the stack below the caller with `pattern`. */
__attribute__((noinline)) static void poison_stack(int pattern) {
  volatile unsigned char buf[256 * 1024];
  memset((void *)buf, pattern, sizeof(buf));
  __asm__ volatile("" : : "r"(buf) : "memory");
}

/* Calls fn(ctx) under the guard. Returns 0 when the call returned, the signal number when it
   crashed, -1 when the budget ran out (before the call if `budget` is non-zero; during it with
   the arena), -2 when the arena is full. */
int fz_guard_call(void (*fn)(void *), void *ctx, int budget) {
  int code;
  if (budget && g_expired) return FZ_TIMEOUT;
  poison_stack(g_pattern);
  code = sigsetjmp(g_env, 1);
  if (code == 0) {
    t_active = 1;
    fn(ctx);
    t_active = 0;
    return 0;
  }
  return code;
}
