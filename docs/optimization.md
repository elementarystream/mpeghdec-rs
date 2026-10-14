# Performance optimizations

The Rust port is several times faster than the Fraunhofer C++ reference decoder while its default
output stays **bit-exact**. This note describes what was optimized, why it is still exact, and
which ideas were rejected. Implementation details are in [PORTING.md](../PORTING.md) ("Multi-core
decoding", "Fast mode", "Chunked parallel decoding"); background measurements are in
[research.md](research.md).

## Exactness and verification

Every optimization is an exactly equivalent transformation: same fixed-point rounding, saturation
and wrap-around, same order of operations where it is observable, same handling of corrupt input.

`crates/cref` tests every ported function against the C++ code (release and `checked` profile),
and SIMD kernels against the generic code on random and extreme inputs. `scripts/e2e.py` compares
the decoded output with the reference for 26 test streams (5.9 hours) and 36 generated vectors at
6 target layouts (242 cases), at several thread counts and decoding modes;
`scripts/stress_threads.py` repeats decodes on an oversubscribed machine. Buffer poisoning
(`MPEGHDEC_POISON`) checks the pipeline stages and the chunk state digest, each with a negative
control that must fail.

## Results on Apple M1 Max

8 performance + 2 efficiency cores; wall clock, best of 3 runs of `scripts/bench.py`; reference:
the `mpeghDecoder` demo, CMake release build (single-threaded). All outputs are byte-identical.

| Case | Reference | Rust `-j 1` | Rust default (`-j 2`) | Rust `-j 8` |
| --- | --- | --- | --- | --- |
| 30 min stereo, stereo output | 13.18 s | 4.44 s (3.0x) | 2.72 s (4.8x) | 0.74 s (17.8x, chunked) |
| 22.2 to stereo (60 s) | 2.29 s | 1.15 s (2.0x) | 0.80 s (2.9x) | 0.74 s (3.1x) |
| 22.2 to 22.2 (60 s) | 4.73 s | 1.60 s (3.0x) | 0.86 s (5.5x) | 0.76 s (6.2x) |
| 7.1.4 to 7.1.4 (60 s) | 2.34 s | 0.80 s (2.9x) | 0.46 s (5.1x) | 0.38 s (6.1x) |
| 7.1.4 to stereo (60 s) | 1.21 s | 0.57 s (2.1x) | 0.40 s (3.0x) | 0.35 s (3.4x) |
| 4 objects to 7.1.4 (60 s) | 1.32 s | 0.28 s (4.7x) | 0.18 s (7.3x) | 0.18 s (7.3x) |
| 5.1 with MCT to 5.1 (60 s) | 1.12 s | 0.38 s (2.9x) | 0.21 s (5.2x) | 0.19 s (6.0x) |

The 30 min stream repeats its configuration about every 0.75 s; the 60 s generated vectors do
not, so chunked decoding cannot split them.

## Single-threaded hot spots

- **Arithmetic decoder** (`crates/arith`): parsing is sequential and bounds all frame-level
  parallelism. The bit-by-bit renormalization is a closed form (leading ones of `~high | low`,
  then the count of `low & ~high` below bit 15) followed by one `read_bits(n)`, which refills the
  bit cache at the same positions; checked for all 2^32 `(low, high)` pairs. The decoder is
  latency-bound, so the symbol search tests symbol 0 first and counts the remaining table entries
  above the threshold with SIMD (SSE2; `umull`, compares and `addv` on AArch64). The rows are
  strictly decreasing, so the count equals the binary search. `get_pk_v2()` is a 128 KiB table
  filled on first use by the original function.
- **FFT** (`crates/fdk/src/fft_rad2.rs`, NEON): four butterflies per vector, the bit reversal
  fused into an out-of-place radix-4 first pass, pairs of radix-2 stages fused, the four C++
  complex multiply variants mapped to one form with the same 64-bit product truncation, and signs
  folded into the coefficients via `-(A >> 16) == (-A + 0xFFFF) >> 16`. 512 points: 3.56 µs to
  1.13 µs. Real FFT and DCT-IV twiddles are vectorized the same way.
- **Format converter**: STFT windows in natural order (element-wise loops); the active downmix
  as per-bin loops over all 256 bins with wrapping (order-independent) sums, and its EQ/clipping
  vectorized with the same IEEE single precision division, square root and conversions.
- **TNS / LPC synthesis**: a linear history instead of a ring buffer and a shorter dependency
  chain (wrapping subtractions may be reordered); the ring state is rebuilt exactly.
- Branchless sign decoding, small-value inverse quantization, saturating scaling and IMDCT window
  crossing; bounds-check-free slice loops in IGF, limiter and interleaving; WAV packing into a
  reused buffer.

## Multi-core decoding

**Thread pool** (`crates/fdk/src/thread_pool.rs`): a fork-join pool with rounds of about 0.2 µs,
since parallel sections are only microseconds long. Idle workers spin for 100 µs, then park; 1 ms
cost up to 28% more CPU time for the same wall time, 30 µs or immediate parking slowed
multichannel decoding by 8–12% and stereo by up to 32%.

**Within a frame**: concealment, IMDCT and LTP per channel (the noise seed of the time-domain
fading stays sequential), STFT analysis and synthesis per channel, IGF and TNS of a channel pair,
WAV output on its own thread. This saturates at about 1.2–1.3x: parsing and the active downmix
stay sequential.

**Frame pipelining** (two or more threads): rendering of access unit N (interactivity, DRC,
format converter, object rendering, earcons, limiter, ...) runs while N+1 is decoded.
Render-only state lives in a `RenderState` that the decoding stage reaches only through a macro
that first waits for the running render, so the compiler enforces the sequential order. The time
signal is double-buffered and per-frame settings are snapshotted; DRC and object metadata
payloads are parsed on a copy of the parser state during decoding, again before rendering, and
checked to agree. Frames with concealment, flush, pre-roll, configuration changes or a fallible
render are rendered sequentially. `Decoder::get_samples()` then returns each frame one
`process()` call later; the sequence is identical if the caller drains it after every call.
Decoding (half of it parsing) becomes the critical path, so more than 3–4 threads barely help.

**Chunked decoding** of seekable files: chunks start at access units with a configuration packet
and are decoded by speculative single-threaded decoders that start at least 216 access units
earlier. The output switches decoders only where a **128-bit digest of the complete decoder
state** matches. `#[derive(StateDigest)]` covers every decoder structure; each excluded field
(scratch, per-frame data, statistics) is justified, and scratch exclusion is validated by
poisoning. The decoder is deterministic, so equal state means equal output; state that may depend
on excluded data (concealment, some out-of-range reads of the reference) raises a hazard flag and
the sequential decoder continues. It is used for inputs longer than 2048 access units with at
least four threads. With 8 threads, three 30 min test streams take 0.75–0.92 s (31 or 30 of 31
boundaries accepted), two others 1.4 s and 2.0 s because their format converter converges late.

## CPU time and memory

The default is **two threads**: most of the speedup at little extra CPU time. `/usr/bin/time -l`,
median of 3:

| Case | Decoder | Wall | CPU time | Max RSS |
| --- | --- | --- | --- | --- |
| 30 min stereo | reference | 13.39 s | 13.32 s | 14.9 MB |
| | Rust `-j 1` / `-j 2` | 4.44 s / 2.71 s | 4.41 s / 5.52 s | 14.1 MB / 14.6 MB |
| | Rust `-j 8`, no chunks | 2.60 s | 7.82 s | 15.2 MB |
| | Rust `-j 8`, chunked | 0.75 s | 5.94 s | 204 MB |
| 22.2 to 22.2 | reference | 4.73 s | 4.72 s | 11.2 MB |
| | Rust `-j 1` / `-j 2` | 1.60 s / 0.86 s | 1.58 s / 1.76 s | 12.8 MB / 14.9 MB |
| | Rust `-j 8` | 0.75 s | 4.99 s | 15.8 MB |

Single-threaded, the port needs 20–33% of the reference's CPU time. At `-j 8` the workers wait
for parsing and CPU time grows 2–3x; chunked decoding keeps all threads busy. On macOS the
performance core count is a better `-j` than all cores (3–10%). Input is read on demand, so memory
does not grow with the file length; the decoder uses 1.5–5 MB more than the reference (thread
stacks, pipeline buffers, tables). Chunked decoding buffers finished chunks (default limit
256 MiB, `-cmem`).

## Fast mode (opt-in, not bit-exact)

`-fast` / `Decoder::set_fast_mode(true)` bypasses the STFT format converter where it is an
identity (one input per output channel, gain 1.0, flat EQ, no STFT-domain DRC, no immersive
downmix) with the same 256-sample delay. The reference spends 39–47% of such decodes there, and
its per-band gains jitter around 1. Deviation over the 185 bypassed test cases: at most 842 LSB,
52 LSB rms of 24 bit (tests enforce 1024 / 64). Gain: 1.7–1.8x single-threaded, 1.6x chunked,
only 1–18% with the frame pipeline, whose critical path is decoding.

## x86-64 with AVX2

Generic x86-64 code vectorizes only for SSE2, which lacks 64-bit products of 32-bit lanes, so
`fMult()` loops stay scalar. Runtime-selected, exact AVX2 paths (`MPEGHDEC_NO_SIMD=1` disables
them): hand-written radix-2 and radix-4 FFT stages (bit reversal fused via 4x4 complex
transposes), real FFT and DCT-IV twiddles, and `computeEQAndClip()` (double precision as in the
reference, exact `schur_div()` quotients with a one-sided correction); `avx2_clone!` copies of
the active downmix, STFT and IMDCT windowing, scaling, limiter and gVBAP gain ramps; an x86-64-v3
(BMI/LZCNT) arithmetic decoder. AVX-512 is not used (see below).

Intel Xeon @ 2.10 GHz, 4 vCPUs of a shared cloud VM (runs vary by up to 5–10%), reference built
with clang 18; best of 3 runs; all outputs byte-identical to the x86-64 reference.

| Case | Reference | `-j 1` | `-j 1`, no SIMD | Default (`-j 2`) | `-j 4` |
| --- | --- | --- | --- | --- | --- |
| 30 min stereo, stereo output | 20.23 s | 7.29 s (2.8x) | 15.22 s | 4.26 s (4.8x) | 2.33 s (8.7x, chunked) |
| 22.2 to stereo (60 s) | 4.21 s | 1.72 s (2.4x) | 3.28 s | 1.24 s (3.4x) | 1.13 s (3.7x) |
| 22.2 to 22.2 (60 s) | 7.30 s | 2.66 s (2.7x) | 5.49 s | 1.45 s (5.0x) | 1.33 s (5.5x) |
| 7.1.4 to 7.1.4 (60 s) | 3.57 s | 1.30 s (2.8x) | 2.77 s | 0.71 s (5.0x) | 0.65 s (5.5x) |
| 7.1.4 to stereo (60 s) | 2.27 s | 0.87 s (2.6x) | 1.73 s | 0.57 s (4.0x) | 0.55 s (4.1x) |
| 4 objects to 7.1.4 (60 s) | 1.39 s | 0.48 s (2.9x) | 0.63 s | 0.30 s (4.7x) | 0.32 s (4.3x) |
| 5.1 with MCT to 5.1 (60 s) | 1.76 s | 0.61 s (2.9x) | 1.36 s | 0.35 s (5.1x) | 0.36 s (4.9x) |

"No SIMD" keeps only the SSE2 parts of the arithmetic decoder.

## Rejected ideas

See [relaxed exactness](research.md#relaxed-exactness) and [hardware
acceleration](research.md#hardware-acceleration) for the measurements.

- Float IMDCT / STFT (vDSP, rustfft): ≤ 8%, not exact; the exact NEON real FFT beats vDSP once
  conversions are counted. Whole format converter in `f32`: 1.2x on downmixes, not worth a mode.
- Passive time-domain downmix: 1.4x, but a different downmix (11–14 dB SNR to the reference).
- `sqdmulh` for `fMult`: no gain on the kernels that matter.
- Metal GPU: bit-exact, but a round trip (180–210 µs) costs more than several CPU frames;
  batching needs a parse/DSP split, which idle CPU cores exploit just as well.
- Accelerate / AMX / BNNS, Neural Engine, SME: float or int8 formats, no large matrix products.
- Further NEON FFT restructuring: near NEON throughput, ≤ 3% overall. Thread QoS: no effect.
- AVX-512: one FFT stage 18% faster in isolation, the whole decoder 1–6% slower.
- Branchless symbol search including entry 0: 3–9% slower. gVBAP ramp clone on AArch64: no
  measurable effect. Exact band split of the active downmix: slower than the fixed-size kernel.

## Remaining opportunities

Parallel element parsing needs `elementLengthPresent`, which the test streams do not send. Moving
IMDCT, LTP and fading into the render stage would balance the pipeline, but the fading updates
concealment state that the next decode reads. A longer warm-up would accept more chunk boundaries
where the format converter converges late (about 120–220 access units). On x86 (estimates): a
local 64-bit bit reader in the arithmetic decoder (≤ 5% of the decoder), a zero-copy output queue
(memmove 4–9%), vectorized 24-bit WAV packing (1–2%).

## Reproducing the measurements

```sh
scripts/make_vectors.py --out work/bench --seconds 60 --only c22_2_1024k c7_1_4_512k obj4_256k c5_1_mct_160k
cargo build --release -p mpeghdec-cli
scripts/bench.py                                  # default threads against the reference
scripts/bench.py --no-ref --args "-j 1"           # other thread counts
scripts/e2e.py --samples all --vectors            # exactness
scripts/e2e.py --samples all --vectors --args "-chunks 16 -cra indep"
scripts/stress_threads.py                         # determinism under load
scripts/fast_accuracy.py --samples all --vectors  # fast mode tolerance
MPEGHDEC_NO_SIMD=1 scripts/bench.py --no-ref --args "-j 1"   # x86: without the AVX2 code paths
```
