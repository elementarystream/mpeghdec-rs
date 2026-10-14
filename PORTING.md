# Porting guide

How the C++ reference implementation (Fraunhofer mpeghdec r4.0.1) is ported to Rust, and how the
port is organized. The port is a **bit-exact** translation; every optimization keeps the output
bit-exact (except the opt-in [fast mode](#fast-mode-opt-in-not-bit-exact)).

## Reference material

`scripts/setup-reference.sh` creates the following git-ignored directories (`MPEGHDEC_WORK_DIR`
overrides the location of `work/`):

| Path | Content |
| --- | --- |
| `work/reference/src` | Original C++ sources (pinned commit `4448b69`, r4.0.1). |
| `work/reference/build-<arch>` | CMake release build for the host (`aarch64`, `x86_64`; `i686` with `--i686`); `bin/mpeghDecoder` is the reference decoder. |
| `work/reference/resolved` | Sources with the architecture conditionals resolved for Apple arm64 without NEON, license headers stripped. **Read these.** |
| `work/reference/pp` | Preprocessed translation units (`*.ii`) of the host build. **Ground truth** of what is compiled. |

### Which code path is ported

The primary target is the **AArch64 build on Apple** (`__arm__`, `__ARM_ARCH_8__`, `__aarch64__`,
`__APPLE__`, NEON disabled, `SINETABLE_16BIT`, `WINDOWTABLE_16BIT`, `POW2COEFF_16BIT`,
`LDCOEFF_16BIT`, `ARCH_PREFER_MULT_32x16`). The x86 builds produce different LSBs and are
reproduced on x86 targets (see [Other architectures](#other-architectures-x86)).

- Many primitives have AArch64 inline assembly variants that round differently from the generic C
  fallback (e.g. `fixmul_DD` is `(a*b)>>31`, not `((a*b)>>32)<<1`; `schur_div`, `sqrtFixp`,
  `invFixp`, `invSqrtNorm2` use single precision float instructions). They are ported in
  `mpeghdec-fdk`.
- Files in `arm/` directories are `#include`d by the generic `.cpp` files; most only contain NEON
  code, which is disabled. Implementations are selected with `FUNCTION_xxx` macros that `unifdef`
  cannot resolve: **when a function has an `arm/` variant, check the `.ii` file** for the
  definition that is compiled.
- The reference build force-includes `mpeghdec_symbol_prefix.h`, so some global function names in
  the `.ii` files carry a `_mpeghdec_` prefix.

### Other architectures (x86)

The port reproduces the **x86-64 build** (`__x86__`, the generic C code and `x86/*.h`, built with
clang) on `x86_64` targets, the **32 bit x86 build** on `x86` targets and the AArch64 build on all
others. `mpeghdec_fdk::ARCH_X86` and `ARCH_X86_32` select the variant at compile time
(`crates/fdk/build.rs`); `MPEGHDEC_ARCH=x86|x86_32|arm` at build time overrides the target
architecture, e.g. to test the AArch64 variant on an x86-64 host. Both builds use the same tables.
The differences, found by comparing the preprocessed translation units function by function:

| Primitive | AArch64 | x86 |
| --- | --- | --- |
| `fMult()` (`fixmul_DD`, `_SD`), `fPow2()` | `(a*b) >> 31` (`>> 15` for `SHORT`) | `((a*b) >> 32) << 1` |
| `fMultAdd()`, `fMultSub()` | `2x ± ((a*b) >> 31)` | `(x ± ((a*b) >> 32)) << 1` |
| `cplxMult*()` | sums of products in 64 bits, one shift | individually shifted products |
| `scaleValueSaturate()` | `sqshl` (low byte of the amount; right shifts floor) | generic (right shifts that shift out all significant bits give 0) |
| `sqrtFixp()` | `fsqrt` in single precision | `sqrt((double)(float)op) * 46340.9492f` |
| `invSqrtNorm2()`, `invFixp()` | single precision division of the normalized input | double precision division, `frexpf()` |
| `schur_div()` | single precision division | 64 bit integer division |
| `fLog2()` | unrolled polynomial (`flog2_func1()`) | loop with `fMult()` |
| `scaleValuesWithFactor()` | right shifts not limited | right shifts limited to 31 |
| FDP: `scaleValue()` by 32 or more | NEON loop for `width & ~15` values | SSE2 loop for `width & ~7` values |

The `Div2` multiplications are the same in both builds. Conversions of `float`/`double` to `INT`
follow the x86 semantics on x86 (`cvttsd2si`: `INT_MIN` for NaN and out of range values). The
NEON code paths of the port reproduce the AArch64 arithmetic and are only used there.

On x86, AVX2 code paths are selected at run time (`mpeghdec_fdk::simd::avx2()`);
`MPEGHDEC_NO_SIMD` (any value) disables them. They compute exactly the values of the portable
code:

- Hand-written kernels with the x86 arithmetic: the radix-2 FFT and its radix-4 first stages
  (`fft_rad2.rs`), the real FFT pre/post processing (`rfft.rs`), the DCT-IV twiddles (`dct.rs`)
  and `computeEQAndClip()` of the active downmix (x86-64).
- AVX2 clones (`avx2_clone!`): the same Rust code compiled a second time with AVX2 enabled, so
  the compiler vectorizes it with 256 bit registers (active downmix, STFT windowing, IMDCT window
  crossing, scaling loops, limiter maxima).

The order of equal elements of `qsort()` (gVBAP loudspeaker sorting) depends on the C library:
the port reproduces Apple Libc on Apple targets and the merge sort of glibc on all others
(`mpeghdec_gvbap::qsort`).

The reference is built with clang: gcc gives the same output for the test streams but differs in
corner cases of undefined shifts (other vectorization), which the differential tests find.

#### 32 bit x86

The 32 bit x86 build (`scripts/setup-reference.sh --i686`) differs from the x86-64 build in three
ways:

- `fixmuldiv2_QQ()` / `fixmuldiv2_QD()`: without a 128 bit type, the generic code of `fixmul.h`
  approximates the product.
- No SSE: the FDP loops are not vectorized (all shifts take the amount modulo 32).
- Floating point uses the x87 unit: operations round to the 64 bit significand of the extended
  format, and values are only rounded to `float`/`double` where the compiled code stores them.
  This affects `sqrtFixp()`, `invSqrtNorm2()`, `invFixp()`, the crossfade `fade()` and the
  timestamp conversions of `mpeghdecoder.cpp`. `mpeghdec_fdk::x87::F80` models the extended
  format in software (so the variant builds on any host); the call sites follow the instruction
  sequences of the reference binary. `scripts/x87/x87_check.py` compares the model with the
  reference compiler.

The differential tests (`crates/cref`) do not compile for 32 bit targets; the variant is verified
end to end (`MPEGHDEC_ARCH=x86_32 scripts/e2e.py`) and with `scripts/x87/x87_check.py`.

## Crates and modules

One crate per C++ library; dependencies mirror the C++ include graph:

```
fdk  <-  arith, drc, pcmutils, tpdec
fdk, arith  <-  igf
fdk  <-  gvbap
fdk, drc, gvbap  <-  formatconverter
fdk, tpdec  <-  uimanager
all of the above  <-  core (libMpeghDec)  <-  mpeghdec (public API)  <-  cli
```

- One Rust module per C++ source file pair (`foo.h` + `foo.cpp` + `arm/foo_arm.cpp`), named after
  the file in snake_case with a redundant `FDK_` prefix dropped: `FDK_bitstream.h` →
  `bitstream.rs`, `drcDec_reader.cpp` → `drc_dec_reader.rs`.
- Every source file starts with the license notice header (copy it from an existing file) and a
  `// Ported from ...` line listing the C++ files.

## Naming

| C++ | Rust |
| --- | --- |
| functions `CBlock_ReadSpectralData`, `fMultDiv2` | snake_case: `cblock_read_spectral_data`, `f_mult_div2` |
| struct typedefs `CAacDecoderChannelInfo`, `FDK_BITSTREAM` | CamelCase: `CAacDecoderChannelInfo`, `FdkBitStream` |
| struct fields `WindowSequence`, `pDynData` | snake_case: `window_sequence`, `p_dyn_data` (a `p`/`h` prefix may be dropped when it no longer is a pointer) |
| macros / constants `MAX_SFB_LONG` | same name, `pub const` |
| `typedef enum` used as integers | `pub type Name = i32;` plus `pub const` values (see `fdk::audio`) |
| `typedef enum` that is a closed set, never used arithmetically | `#[repr(i32)] enum` |

Keep the original C++ name in the doc comment of every public item (`` /// `fMultDiv2(a, b)` ``)
so that `grep` finds it. Overloaded C++ functions get suffixes:

- `_sgl` for the `FIXP_SGL` variant (`scaleValue(FIXP_SGL)` → `scale_value_sgl`),
- `_noexp` for the variant without an exponent out-parameter (`fDivNorm(a, b)` →
  `f_div_norm_noexp`),
- `_to` for `(dst, src)` variants and fixed result exponents (`scaleValues(dst, src, ...)` →
  `scale_values_to`).
- The `fMult`/`fMultDiv2`/`fMultAdd(Div2)`/`fMultSub(Div2)`/`fPow2`/`fAbs`/`fNorm` overload sets
  are generic functions dispatching on the operand types (`i32` = `FIXP_DBL`, `i16` =
  `FIXP_SGL`, `i64` = `FIXP_QDL`), like C++ overload resolution. The Rust operand types must
  match the C++ static types, otherwise another overload (with different rounding) is used.
- `cplxMult*(&re, &im, a_re, a_im, b_re, b_im)` returns a tuple: `let (re, im) = cplx_mult(...)`.

## Types

| C++ | Rust |
| --- | --- |
| `FIXP_DBL`, `LONG`, `INT`, `INT_PCM` | `FixpDbl` = `i32` (use `i32` for integers) |
| `FIXP_SGL`, `SHORT` | `FixpSgl` = `i16` |
| `FIXP_QDL`, `INT64` | `i64` |
| `UINT`, `ULONG` / `USHORT` / `UCHAR` / `SCHAR` | `u32` / `u16` / `u8` / `i8` |
| `FIXP_SPK` / `FIXP_DPK` | `FixpSpk` / `FixpDpk` (`.re`, `.im`) |
| `float` / `double` | `f32` / `f64` |

Use the exact width and signedness of the C++ variable. Arithmetic must reproduce C semantics:

- **Integer promotion**: `UCHAR`/`SHORT` operands are promoted to `int`; mixing `int` and `UINT`
  converts to unsigned. Convert explicitly in Rust.
- **Overflow**: signed overflow wraps in the reference build. Plain Rust arithmetic is the
  default; where the C++ code may overflow, use `wrapping_*`. The `checked` profile (overflow
  checks on) finds such places.
- **Shifts** by a negative amount or by the bit width or more: the AArch64 build masks the amount
  modulo 32 (64 for 64 bit values). Use `wrapping_shl`/`wrapping_shr` where the amount can be out
  of range.
- **Casts**: `(SHORT)x` → `x as i16`; `(INT)f` → `f as i32` (Rust saturates where C is undefined;
  x86 conversions are reproduced explicitly).
- **Floating point**: keep `float` vs `double` precision; `FL2FXCONST_DBL(0.5f * x)` evaluates its
  argument in the precision of the C++ expression. clang contracts `a*b+c` into fused
  multiply-add on arm64: check the disassembly of the reference object file and use `mul_add`
  where clang emitted `fmadd`/`fmsub`.

## Tables

Do not retype constant tables. `scripts/codegen/dump_table.py` compiles a small program against
the C++ source and prints the exact values:

```sh
scripts/codegen/dump_table.py libFDK/src/FDK_tools_rom.cpp SineTable512 SHORT --name SINE_TABLE_512 --per-line 8
scripts/codegen/dump_table.py libFDK/src/fixpoint_math.cpp exp2_tab_long UINT --hex --name EXP2_TAB_LONG
```

Struct tables (e.g. `FIXP_SPK` arrays) are dumped as their scalar fields in memory order.

## Memory, pointers and ownership

- Prefer owned data (`Vec`, `Box<[T]>`, arrays) and slices with indices over raw pointers. A
  pointer walking through an array becomes an index.
- `HANDLE_FOO` parameters become `&mut Foo` / `&Foo`; the static memory pools (`C_ALLOC_MEM`,
  `aac_ram.cpp`) become ordinary allocations owned by the respective structs.
- Scratch buffers become local arrays or buffers owned by the caller. **Unions used to share
  memory** become separate storage, unless the code reads data through another member than the
  one written (then replicate that explicitly). Check whether code relies on stale content of
  reused buffers.
- `FDK_BITBUF` points into memory that several bit buffers share (the transport decoder's
  input buffer, copies made to save and restore a position). Here that memory is a `BitMem`, a
  reference-counted byte array: a bitstream is `Clone` (C's struct assignment) and shares the
  memory, which lives as long as any bitstream uses it. A reader over a temporary buffer gets a
  copy (`FdkBitStream::reader`).
- Where raw pointers are unavoidable, confine them to small functions with a `// SAFETY:` comment.
- Out-parameters (`INT* result_e`) become `&mut i32` parameters; error codes stay error codes.

## Multi-core decoding

The reference decoder is single-threaded. The port uses two kinds of parallelism, both with an
output that is identical for every thread count; with one thread neither is used:

- **Frame pipelining** (`mpeghdec_core::render`, `mpeghdec_core::pipeline`): the rendering of
  access unit N runs on a render thread while access unit N+1 is decoded.
- **Fork-join parallelism** within a frame with a thread pool
  (`mpeghdec_fdk::thread_pool::ThreadPool`).

`MpeghDecoderContext` with `n > 1` threads owns the render thread (`RenderWorker`) and, for `n > 2`,
a pool of `n - 1` threads, handed to the core decoder with `set_render_worker()` /
`set_thread_pool()`.

Rules for parallel code:

- Only distribute independent work items: each item writes memory no other item of the same fork
  reads or writes. State the sequential code passes from item to item (random seeds, `frame_ok`,
  stale buffer content) is computed before or after the parallel part, in the sequential order.
- Keep the sequential path (a `pool: Option<&ThreadPool>` / `_pool` variant next to the original
  function) and document at the parallel variant why it is exact.
- Scratch buffers shared by the items become one buffer per item; if later code may read the
  stale content, reproduce the final content of the sequential run.
- A fork-join round costs a fraction of a microsecond and a frame a few hundred microseconds
  (docs/optimization.md): only fork for work items of several microseconds.
- Test the parallel variant against the sequential one (`crates/cref`), end to end with several
  thread counts (`scripts/e2e.py --args "-j N"`) and with `scripts/stress_threads.py`.

| Parallel part | Items | Why it is exact |
| --- | --- | --- |
| Concealment, IMDCT, LTP post filter (`frequency_to_time_parallel()`) | channels | A pre-pass walks the channel loop without processing (order, offsets, the `frame_ok` each channel sees). Each item has its own IMDCT output buffer; the last one is copied to `mdctOutTemp`. The time domain fading, which passes the noise seed on, runs afterwards in channel order. |
| IGF and TNS of a CPE (`cchannel_element_decode_pool()`) | 2 channels | Disjoint channel infos, spectra and IGF buffers. |
| Format converter STFT analysis / synthesis (`iis_format_converter_process_pool()`) | channels, time slots | The headroom prescaling of a time slot depends only on its samples and is computed first; every time slot has its own spectra; DRC and active downmix run per time slot in order. |
| WAV output of the command line decoder | writer thread | Batches are written in order. |

Not parallelized: the active downmix (sequential energy smoothing; splitting the ERB bands did
not pay off), gVBAP (a few microseconds per frame), the limiter (one gain envelope), element
decoding across elements (shared work buffers, MCT) and the bitstream parsing (the element
lengths that would allow it are optional and not sent by the test streams).

### Frame pipelining

`aacDecoder_DecodeFrame()` consists of two stages:

- **core**: transport decoder (`transportDec_ReadAccessUnit()` with its callbacks), pre-roll
  parsing, `updateUiStatus()` and `CAacDecoder_DecodeFrame()` (parsing, element decoding,
  concealment, IMDCT, LTP post filter, TD fading) into the time signal in `pTimeData2`;
- **render**: the channel gains of `applyUserInteractivity()`, DRC 1 with its delay
  compensation, format converter, gVBAP, DRC 2/3, sample rate conversion, MPEG-H AU size,
  truncation, cross-fade, earcon mixing, output loudness, interleaving, limiter and the discarding
  of the start delay.

The control part of `applyUserInteractivity()` runs between the stages on the decoding thread.

With a render thread, `aac_decoder_decode_frame_pipelined()` runs the core stage of access unit
N+1 while the render stage of N runs on the render thread. The state only the render stage uses
is `RenderState` (DRC decoder, format converter, gVBAP renderers, STFT filterbanks, delay lines,
sample rate converters, truncation, cross-fade, earcon decoder, limiter). The render functions
take it together with a `RenderCtx` (a snapshot of the core state they read), the stream info and
the time buffers, and cannot touch the core state. The core stage reaches the render state only
through the `rs!` macro, which first waits for a running render stage (`join_render()`): the
compiler guarantees that render state is never used concurrently, and every access happens in the
sequential order. The sequential decoder calls the same render functions in the original order.

State written by the core stage and read by the render stage:

| State | Resolution |
| --- | --- |
| Time signal (`pTimeData2`) | Two buffers exchanged at the hand-over. |
| Configuration, `signalSkipped[]`, flags, target layout, `flushStatus`, loudness and limiter settings | Snapshot in the `RenderCtx`. The transport decoder rewrites its configurations at every configuration packet, so the render stage never reads them directly. |
| Stream info (`CStreamInfo`) | Snapshot; the render stage writes its copy, returned with the frame. |
| Channel gains of the UI | Computed by the control part, passed in the `RenderCtx`. |
| DRC gain payloads (every frame), object metadata (object streams) | Parsed twice: the core stage parses on a copy of the parser state for the bit position and error status it needs and records the payload; before the next render stage the payload is parsed again on the real state, in the sequential order. Both parses must give the same result (checked). A parse error waits for the rendering and parses on the real state. |
| Production metadata, format converter frame data | Parsed on the real state after waiting for the rendering. |
| DRC configuration and loudness, configuration changes, downmix matrices, `CtrlCFGChange`, truncation, earcon configuration and PCM data | The transport decoder callbacks use `rs!` and wait for the rendering. |
| Parameters (`aacDecoder_SetParam()`) | Through `rs!`, after the rendering of the previous frame. |

State written by the render stage and read by the core stage:

| State | Resolution |
| --- | --- |
| `pTimeData2` and `workBufferCore2` | Separate buffers per stage. **Buffer poisoning** (`MPEGHDEC_POISON`): `MPEGHDEC_POISON=3` fills both buffers with pseudo random values before the core decoding and `workBufferCore2` before the rendering of every access unit in the sequential decoder; all end-to-end outputs stay identical, so neither stage reads content the other left. `MPEGHDEC_POISON=6` (also destroying the time signal) is the negative control. |
| IGF `virtualSpec` (in `pTimeData2`), which the IGF may read stale | Only where the tiles of an IGF grid do not cover the IGF range; `igf_reads_stale_data()` checks the configuration, and such configurations are not pipelined. |
| Downmix matrix set stored in `pTimeData2` | Written and read by the configuration callbacks of the core stage. |
| Stream info fields of the rendered frame | The render stage writes its own copy; the core stage only reads `numChannels`, which is known before the rendering. |

An access unit is rendered at once, as in the sequential decoder, if there is no render thread or
the buffer poisoning is on, if `aacDecoder_DecodeFrame()` is called with flags (concealment,
flushing, interruption), if it has pre-roll access units or the decoder is flushing or building
up, if the rendering could fail (its error handling influences the following calls), without
MPEG-H rendering (target layout -1), or if the IGF may read stale `pTimeData2` content.

#### `MpeghDecoderContext` (`mpeghdecoder_*()`)

Because of the decoder delay, the output frame that `get_samples()` returns after `process()` of
access unit N contains samples rendered from N, so a decoder that returns the same frames call by
call cannot overlap the rendering of N with the decoding of N+1. The pipelined `MpeghDecoderContext`
returns the output frames one `process()` call later: a `get_samples()` call that needs the output
of a frame still being rendered returns `Ok(None)` and records that the sequential caller would
have called `get_samples()` until `Ok(None)` at this point; these calls are executed on the state
of that point when the rendering has finished, and their frames are returned by the next
`get_samples()` calls. Queue operations that do not commute with the pending ones (full queues,
restarts, flushes, errors) wait for the rendering. The return values of `process()` are those of
the sequential decoder. The sequence of output frames is identical if `get_samples()` is called
until it returns `Ok(None)` after every `process()`, `flush()` and `flush_and_get()` call.

## Fast mode (opt-in, not bit-exact)

The fast mode (`mpeghdec_core::fast::FastOptions`, `Decoder::set_fast_mode()`, CLI `-fast`)
enables shortcuts whose output differs from the reference by the reference's own rounding noise (a
few LSB of 16 bit PCM, see the README). It
costs nothing when off. Background: [docs/research.md](docs/research.md#relaxed-exactness). The
only option is the format converter bypass.

The reference runs every channel signal through the STFT format converter (STFT analysis, active
downmix, synthesis), also when the target layout is the channel layout of the stream. Then the
downmix matrix maps every input channel to one output channel with the coefficient 1.0, the
equalizer gain is 1 and the sine-windowed STFT with 50 % overlap reconstructs the input delayed by
one STFT frame (256 samples). The bypass (`mpeghdec_formatconverter::format_converter_fast`)
replaces the STFT processing of such a frame by `out[o][t] += fMult(in[i][t - 256], dmx[i][o])`.

Conditions, checked every frame (`format_converter_bypass_map()`); otherwise the frame takes the
bit-exact path:

- the fast option is set for the access unit (a snapshot at the end of its core stage, so it
  applies in the sequential order also with frame pipelining);
- no STFT domain DRC;
- the converter is in the STFT mode with `fftSize == 2 * stftFrameSize` and `frameSize` a
  multiple of `stftFrameSize`;
- no immersive downmix and one downmix matrix for all ERB bands;
- at most one non-zero downmix coefficient per input and per output channel, each 1.0;
- the equalizer of every non-zero coefficient is flat.

Output channels without an input channel stay silent, so e.g. stereo to 5.1 is bypassed as well.
The delay line is the `prevAudioInput` buffer of the STFT analysis, which both paths keep up to
date. When the STFT path follows bypassed frames, `format_converter_leave_bypass()` sets the
overlap-add buffers to the windowed tail of the delayed copy and resets the active downmix state;
the output stays continuous but is not bit-exact afterwards.

Verification: `scripts/fast_accuracy.py` checks every case against the reference (bit-exact
without bypass; with bypass max |error| ≤ 1024 LSB and RMS ≤ 64 LSB of 24 bit per channel); the
unit tests of `format_converter_fast` compare the bypass with the STFT path; the differential
tests `fast_mode_*` (`crates/cref/tests/core_frame.rs`) decode with concealment, flushes,
parameter changes, seeks and random switching of the fast mode.

## Chunked parallel decoding

The decoding of a single stream is sequential by nature, which limits pipelining and fork-join
parallelism to about 2x. A whole file can be decoded faster in chunks, each with its own decoder
that starts some access units earlier (warm-up). Its output is only identical to the sequential
decoding once its state equals the state of the sequential decoder, which no fixed warm-up
guarantees. `mpeghdec::chunked` therefore **verifies every chunk boundary** by comparing the
complete decoder states and falls back to the sequential decoding where they differ.

### State digest (`mpeghdec_fdk::digest`)

The decoder is deterministic: two decoders with equal states return the same output for the same
input. States are compared by a 128 bit digest of all state that can influence the future output:

- `StateDigest` feeds a value to a `Digester` (two `wyhash`-style multiply-fold chains). It is
  implemented for primitives, arrays, `Vec`, `Box`, `Option`, `VecDeque`, tuples and references
  (by address: references in the decoder state point to constant tables).
- `#[derive(StateDigest)]` (crate `mpeghdec-digest-derive`) feeds every field; it is derived for
  all structures of the decoder, so a new field is part of the digest automatically.
- `#[digest(skip)]` excludes a field, always with a comment why it cannot influence the future
  output; `#[digest(with = f)]` / `#[digest(extra = f)]` feed a normalized form.
- `MpeghDecoderContext::state_digest()` / `state_trace(depth)` return the digest and the
  digests of all fields (`diff_traces()` lists the fields in which two states differ).

Not part of the digest:

| State | Why |
| --- | --- |
| Scratch memory (`workBufferCore1/2`, `pTimeData2` unless the IGF may read it stale, IGF working memory, joint stereo data, parallel IMDCT buffers, pipelining buffers) | Written before it is read within an access unit. Checked with `MPEGHDEC_POISON=8`, which fills all of it with garbage before every access unit: all end-to-end outputs stay identical. |
| The per-frame channel data (`pAacDecoderChannelInfo[]`) | Parsed with the element before it is decoded. Only the concealment uses values of an earlier frame, which is a hazard. |
| Concealment state only read while concealing (noise seed, counters) | Concealment is a hazard. The noise seed depends on the number of frames since the decoder was opened and never converges. |
| Fields that are initialized before every use or never read; statistics, counters, thread handles | Cannot influence the output. |

Normalized state: ring buffers (`MpeghDecoderContext` queues, limiter delay line, DRC node buffers)
are digested relative to their read position; IGF grid caches by the fact that they match their tile
indices; the transport decoder input buffer is set by a decoder that starts in the middle of a
stream to what the sequential decoder has (`TransportDec::set_bs_buffer()`).

**Hazards** (`mpeghdec_fdk::digest::hazard()`): the decoder signals a hazard wherever excluded
state or the physical layout of a ring buffer could influence the decoding (concealment, a DRC
interpolation over a node buffer without nodes or not written since its initialization,
`deque_pop_back()` with `last == 0`). Two decoders with equal digests decode identically up to the
first hazard.

The unit test `state_digest_covers_the_components` checks that equal inputs give equal digests,
that changing any of about 25 components changes the digest, that scratch memory does not, and
that rotating the ring buffers does not.

### Protocol (`decode_chunked()`)

- The access units are split into chunks (four per thread, at least 1024 access units each, and
  so many that one chunk per thread fits into the buffer limit). Every chunk after the first
  starts at a random access point at least `warmup` (216) access units earlier: an access unit
  with a configuration packet (with `-cra indep` also an independent frame). Without
  configuration repetition the input is decoded sequentially.
- The **true run** decodes from the first access unit on the calling thread. Every other chunk is
  decoded by a single-threaded **speculative run** on a worker, which discards the output of the
  warm-up and computes its digest at the start of its chunk (and at `verify_offsets` 64 and 192)
  and before its end.
- At the end of chunk k the digest of the current run is compared with the one of the speculative
  run of chunk k+1. If they are equal, the output is taken from the speculative run from there
  on; if not, the comparison is repeated at the next verification point, and finally the current
  run continues through chunk k+1.
- A hazard in the speculative run that produces the output stops it: the true run, paused where
  it handed over, decodes up to the hazard without output and continues from there.
- After 3 consecutive rejected boundaries the true run decodes the rest with all threads.
- Errors are those of the sequential loop. Runs that finish before their turn keep their output
  and wait while it exceeds `max_buffered_bytes` (256 MiB).

Tests: `crates/mpeghdec/tests/chunked.rs` compares the chunked with the sequential decoding frame
by frame (chunk and thread counts, timestamp gaps around boundaries, lost and corrupt access
units, configuration changes, errors, giving up); end to end with `-chunks 0`, `-chunks 16 -cra
indep` and a stress setting with far too short warm-ups (`-chunks 24 -cw 16 -cv 0,8,32,128 -cra
indep -j 3`): all outputs identical. Without the format converter energies in the digest, the
stress setting produces different outputs, so the verification detects state that matters.

Limitations: streams without configuration repetition are decoded sequentially by default (with
`-cra indep` the states rarely become equal); some boundaries are rejected because rarely read
state differs; a hazard late in a file costs up to a sequential decoding; the chunks decode
single-threaded; `-y`, `-z`, `-sf`, `-st` and the UI manager options use the sequential decoding.

## API layers

- **Straight port of the C API**: `mpeghdec_core::mpeghdecoder` (`mpeghdecoder.h`: the
  `MpeghDecoderContext` and the functions `mpeghdecoder_init()` ... `mpeghdecoder_set_param()`,
  the C names in snake case, with the C error codes) and `mpeghdec_uimanager::mpegh_ui_manager` (`mpeghUIManager.h`).
  Handles become `&mut`, buffers slices; additions not in C (threads, fast mode, state digest)
  are methods of the context. The differential tests and the fuzz targets use this layer.
- **Safe API** (`mpeghdec`): `Decoder`, `UiManager` and `chunked`, with `Result`s and enums
  that keep the C codes. Every method calls the straight port.
- **C ABI** (`mpeghdec-capi`): the 17 functions of the C API as `extern "C"` functions over the
  straight port, with the unchanged headers. Pointers are checked like in C; where C would
  dereference NULL, the C ABI returns an error code instead (documented in `crates/capi/src`).
  Every function catches panics. `scripts/capi-test.sh` checks the exported symbols, the
  structure layout and the behaviour against the reference library, and links the reference demo
  decoder against the C ABI for the end-to-end tests.

## What is not ported

- NEON/ARMv7 assembly and intrinsics (not used by the reference configurations; the arithmetic of
  the AArch64 and x86 inline assembly is reproduced).
- Memory allocation wrappers and library info boilerplate without effect on decoding.
- Code that is compiled out in the reference configuration (check the `.ii` files).

## Testing

### Differential tests (`crates/cref`)

`crates/cref/build.rs` compiles the complete C++ library (`$MPEGHDEC_C_DIR`, default
`work/reference/src`) together with the shims in `crates/cref/shim/`, with clang unless `CXX`
selects another compiler. Without the sources the tests are skipped. To test a ported module:

1. Add `crates/cref/shim/<module>.cpp` with `extern "C"` wrappers named `cref_*` (one function
   definition per line start: `RET cref_name(ARGS) {`). Use `void*` for opaque handles and scalar
   pointers for arrays.
2. Run `scripts/codegen/gen_cref_bindings.py <module>` to generate `crates/cref/src/<module>.rs`
   and add `#[cfg(not(cref_missing))] pub mod <module>;` to `crates/cref/src/lib.rs`.
3. Add `crates/cref/tests/<module>.rs` (starting with `#![cfg(not(cref_missing))]`) comparing Rust
   and C++ on random and edge-case inputs (`mpeghdec_cref::testutil::Rng`), including internal
   state.
4. Run `cargo test -p mpeghdec-cref --release --test <module>` and with `--profile checked`.

Static C++ functions are tested through the nearest non-static function.

### End-to-end tests

`scripts/e2e.py` decodes the test material with the command line decoder and compares the SHA-256
of the WAV output with the reference: `SHA256SUMS.ref-pcm` (AArch64) of the test streams,
`work/vectors/manifest.json` and `work/conformance/ref.<arch>.json`. Missing hashes of the x86
variants are created with the reference decoder of the variant (`scripts/refsums.py`).
`MPEGHDEC_ARCH` selects the variant to check, e.g. on an x86-64 host:

```sh
MPEGHDEC_ARCH=arm CARGO_TARGET_DIR=target/arm cargo build --release
MPEGHDEC_ARCH=arm scripts/e2e.py --exe target/arm/release/mpeghdec --samples all
cargo build --release --target i686-unknown-linux-gnu -p mpeghdec-cli
MPEGHDEC_ARCH=x86_32 scripts/e2e.py --exe target/i686-unknown-linux-gnu/release/mpeghdec --samples all
```

Further checks: `MPEGHDEC_POISON=3|8 scripts/e2e.py --vectors --args "-j 1"` (stale buffers),
`MPEGHDEC_NO_SIMD=1 scripts/e2e.py` (without AVX2), `scripts/stress_threads.py` (thread counts
under load), `scripts/fast_accuracy.py` (fast mode deviation) and the differential fuzz targets in
[fuzz/](fuzz/README.md).

## Style

- Documentation, comments and commit messages in English.
- Keep the C++ comments that explain the algorithm. Do not add comments that merely restate code.
- `cargo fmt` before committing; clippy lints about C-style code are allowed in the workspace
  `Cargo.toml`.
