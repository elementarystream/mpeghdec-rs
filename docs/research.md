# Research notes

Three investigations that shaped the verification and the performance work of the port: the ISO
conformance bitstreams, how much a decoder that is only numerically close to the reference could
gain, and whether hardware accelerators are worth using. The performance measurements were made on
an Apple M1 Max (8 performance + 2 efficiency cores, 24-core GPU); multi-threaded timings are noisy
(about ±5%); single-threaded kernel timings are stable. The prototypes used for the last two
sections are not part of the repository.

## Conformance bitstreams

ISO publishes the electronic inserts of ISO/IEC 23008-9 (MPEG-H 3D Audio conformance) free of
charge on `standards.iso.org`. The 2nd edition archive (1 605 MB) holds 161 MHAS bitstreams
(108 MB), 196 reference waveforms (1.9 GB) and a coverage table. The licence allows their use for
the purpose of the standard, testing a decoder, but not redistribution, so they are downloaded
into the git-ignored `work/conformance` together with the derived test cases and output hashes.
The inner archive is stored without compression, so single files can be fetched with HTTP range
requests instead of downloading the whole archive.

```sh
scripts/conformance.py fetch [--waveforms]   # bitstreams (108 MB), optionally the waveforms (770 MB)
scripts/conformance.py prepare               # MP4 wrapping, cases.json
scripts/e2e.py --samples none --conformance  # bit-exact against the reference build
scripts/conformance.py iso                   # reference output against the ISO waveforms
```

`fetch` checks the CRC-32 of every file against the archive directory and records its SHA-256
(ISO publishes no checksums). `prepare` wraps the bitstreams into MP4, because the reference
decoder reads MP4 only, and derives the test cases. The access units are timed at the sampling
rate of the stream, one frame length apart: the decoder API compares timestamp differences with
the access unit length and inserts or removes samples when they disagree.

### Applicability

The reference decoder supports the Baseline profile up to level 4 with an FD core only (no LPD,
no HOA). **82 bitstreams** (33 MB) use only these tools and come with **108 reference waveform
cases** (769 MB). They cover window switching, noise filling, TNS, M/S and complex prediction,
IGF (including stereo filling), MCT, tonal component coding, MPEG-D DRC, ducking and loudness
normalization, object metadata (position, rate, spread), pre-roll and target layouts 2, 6 and 19.
They come from other encoders than the rest of the test material and are built to hit extreme
values of each tool. The other **79 bitstreams** (9 LPD, 24 FD/LPD switching, 46 HOA) serve as
negative tests: the port must fail exactly like the reference.

Each reference waveform names its decoder settings in a suffix of its file name:

| Setting | Meaning | `mpeghdec` option | Cases |
|---|---|---|---|
| none | reference layout, no loudness normalization, no DRC effect, no preset | `-tl <reference layout>` | 48 |
| `Lay-<x>` | target layout, CICP index | `-tl <x>` | 8 |
| `Lou-<x>` | target loudness −x LKFS (x = 10, 16, 24, 31) | `-rl <4·x>` | 42 |
| `Eff-<x>` | DRC effect type request | `-dse <x>` | 16 |
| `Pr-<x>` | group preset | UI manager `-script` with a preset selection (`actionType="95"`) | 8 |
| `Cpo-<x>` | conformance point inside the decoder | none; the final output is used | 14 |

Limitations of the mapping:

- `mpeghdec` defaults to `-tl 6`, so the reference layout is read from `mpegh3daConfig`. Two
  streams (`C0_3_FD`, `C104_3_FD`) give it in a form that matches no CICP layout and are decoded to
  layout 6.
- Loudness normalization cannot be disabled (the target level range 40–127 has no "off"), which
  matters only for the comparison with the ISO waveforms, not for the bit-exact comparison.
- The exact criteria of ISO/IEC 23008-9 are not in the free inserts. `scripts/conformance.py iso`
  assumes the RMS/LSB definition of ISO/IEC 11172-4 generalized to K bits (RMS of the difference
  below 2^-(K-1)/√12, maximum at most 2 LSB at K bit) and a segmental SNR over 1024-sample
  segments capped at 100 dB, so its verdicts are indicative only. Failing an ISO criterion is a
  property of the reference decoder as long as the port is bit-exact with it.

### Results

**Bit-exact comparison**: 214 cases, 124 applicable (108 waveform cases and 16 extra default
settings) and 90 negative (57 HOA, 33 LPD/switching). For every case the port matches the
reference build in exit code, output hash (or absence of output) and error message, with the
default settings, `-j 1`, `-j 4`, `-j 8`, chunked decoding and buffer poisoning. The reference
decodes 115 applicable cases; it writes no output for 8 (`M1_3_FD`, the tonal component coding
streams, four 64–96 kHz streams) and rejects `C6_3_FD_Sfb` (extreme varying `max_sfb`). LPD and
switching streams fail at access unit 10; HOA streams produce no output.

**ISO criteria** (reference output against the ISO waveforms; holds for the port too): 91 of the
108 cases are comparable, 87 of them have a criterion. 63 pass; 86 pass when the first 1024
samples are ignored (the other 23 exceed the maximum difference only in the first frame, by up to
0.6 LSB at 16 bit). The RMS difference is 0.02–0.08 LSB at 16 bit against a limit of 0.29, the
segmental SNR 53–77 dB against 40 or 46 dB. `C6_3_FD_M-Mec` reaches a maximum difference of
2.59 LSB at 16 bit throughout the signal; `C6_3_FD_Nmf` (non-meaningful window transitions)
deviates far more, but its criterion only asks for decoding the whole stream.

## Relaxed exactness

How much faster could the decoder be if its output only had to be very close to the reference (a
few LSB of 24 bit)? Single-threaded profile of the bit-exact port (`-j 1`):

| Part | Stereo to stereo | 22.2 to 22.2 | 22.2 to stereo | 7.1.4 to 7.1.4 |
| --- | --- | --- | --- | --- |
| Parsing, arithmetic decoding, inverse quantization, noise filling | 31% | 32% | 47% | 27% |
| Format converter STFT analysis + synthesis | 23% | 23% | 17% | 28% |
| Format converter active downmix | 17% | 17% | 18% | 20% |
| IMDCT (including its FFT) | 11% | 11% | 19% | 10% |
| IGF, TNS, other tools | 8% | 2% | 2% | 1% |
| Limiter, interleaving, copies, WAV output | 10% | 12% | 5% | 14% |

The reference runs the format converter on every stream, including stereo to stereo, where its
downmix matrix is the identity.

**Conclusion.** Relaxing the arithmetic is worth little: the hot DSP kernels are already exact
NEON code, and about half of the rest is parsing, where exactness costs nothing. The one large
single-threaded win is algorithmic: **bypass the format converter when it is an identity**
(1.60–1.73x in the prototype). The largest win overall needs no relaxation at all: **chunked
parallel decoding** was bit-exact on every tested stream with enough warm-up. Both are
implemented: the fast mode and chunked decoding (see [PORTING.md](../PORTING.md) and
[optimization.md](optimization.md)).

### Variants measured

Speed relative to the bit-exact `-j 1` run (CPU time, best of 3); error against the bit-exact
output, worst channel, in LSB of 24 bit:

| Variant | Speed | Max error | RMS error | Notes |
| --- | --- | --- | --- | --- |
| Format converter bypass (identity matrix) | 1.60–1.73x | 528–672 | 32–46 | ≈ −106 dBFS rms; the fast mode |
| Format converter in `f32` (vDSP) | 1.11–1.20x | 501–673 | 33–54 | also for real downmixes |
| Bypass + `f32` converter + `f32` IMDCT | 1.66–1.80x (downmix: 1.23–1.24x) | | | |
| `f32` STFT only | no measurable gain | 480–553 | 41–54 | conversions eat the gain |
| `f32` IMDCT only | ≤ 3% | 113–164 | 9.4–10 | |
| Passive time-domain downmix | 1.4x | 1.1–1.3 M | 133–264 k | a different, louder downmix (11–14 dB SNR) |

Every variant that touches the format converter lands near 40 LSB rms for the same reason: the
fixed-point STFT round trip alone is accurate to below 1 LSB rms, but the reference's **active
downmix** computes its per-band EQ gain √(T/R) with a 16-bit division, single precision
`invSqrtNorm2` and per-slot headroom renormalization. For an identity matrix T = R, so the exact
gain is 1, yet the reference applies gains that fluctuate around 1 by about 1e-5 (about −100 dB).
The bypass is therefore closer to the mathematically exact result than the reference; its error is
about 0.15 LSB rms and at most 2.6 LSB of 16 bit.

Kernels, single thread:

| Kernel | Fixed point (exact NEON) | `f32` vDSP incl. conversions | `f32` portable |
| --- | --- | --- | --- |
| DCT-IV 1024 (IMDCT) | 1.71 µs | 0.90 µs (1.9x) | rustdct 3.56 µs |
| DCT-IV 128 (short blocks) | 0.17 µs | 0.12 µs (1.5x) | |
| Real FFT 512 (STFT) | 0.62 µs | 0.82 µs (0.75x) | realfft 0.88 µs |
| STFT analysis + synthesis, one channel and slot | 1.50 µs | 0.85 µs (1.8x) | |

Float pays off only when data stays in float across several stages, and the portable crates are
slower than the fixed-point code, so float DSP would not help on non-Apple targets.

Cheaper fixed-point primitives are not the obstacle either: `sqdmulh` reproduces `fMult` exactly
except for `-1·-1`, wrapping sums may be re-associated exactly, and the remaining sequential loops
(TNS lattice, limiter envelope, energy smoothing) are true recursions that float would not remove.

### Chunked decoding

Each range of a file is decoded by its own decoder that starts K access units earlier at a random
access point, with the warm-up output discarded. Boundaries identical to the continuous decode
(32 ranges of a 30 min stream):

| Stream | K = 0 | K = 72 | K = 144 | K = 216 |
| --- | --- | --- | --- | --- |
| s1_30min | 0/31 (≤ 403 LSB) | 0/31 (≤ 252 LSB) | 31/31 | 31/31 |
| s2_30min | 0/31 (≤ 362 LSB) | 0/31 | 29/31 | 31/31 |
| s4_30min | 0/31 (≤ 269 LSB) | 1/31 | 27/31 | 31/31 |
| s1_30min, bypass | 31/31 | 31/31 | | |

The core decoder is exact from a random access point with configuration and pre-roll. The slowly
converging state is the active downmix (smoothed energies, headroom of the previous slot), which
needs 120–220 access units and depends on the signal; with the bypass it disappears. Concealed
frames are not reproducible, because concealment draws random phases from state that depends on the
whole history. Because convergence is empirical, the implementation verifies every boundary with a
state digest instead of trusting K.

## Hardware acceleration

**Conclusion:** GPU, AMX, Neural Engine and SME offload are not worth it. The decoder is bound by
sequential parsing and frame-to-frame dependencies; its parallel DSP parts are small and far
below the fixed cost of a GPU round trip. Splitting parsing from DSP (the frame pipeline) is a
CPU change, and once it exists idle CPU cores absorb the DSP work.

Everything a GPU or a float transform library could take over (FFT, IMDCT and STFT) is 31–37% of the
single-threaded time for channel-based content and 8% for objects. With frame-level threads but
without a frame pipeline, parsing was 40.7–58.5% of the main thread's time at `-j 8`; removing the
transforms from it entirely would give at most 1.06x (objects) to 1.28x (22.2 to 22.2).

### GPU (Metal)

- **Exactness is possible.** Metal has exact 32-bit integer arithmetic, `mulhi` and 64-bit
  multiplies; a prototype FFT kernel reproduced the decoder's fixed-point FFT bit-exactly. GPU
  division is correctly rounded and `precise::sqrt()` matches IEEE (plain `sqrt()` does not).
- **Latency kills it.** A command buffer round trip has a median of 182–210 µs (24 µs when
  pipelined), 3–4 complete stereo frames on the CPU (55 µs each single-threaded).
- **Batching does not help enough.** The zero-copy GPU FFT breaks even with one core at about 200
  512-point transforms per dispatch and with 8 cores at 1000–1500, i.e. the transforms of 50–100
  frames of 22.2 content (1–2 s of audio). That needs the parse/DSP split; with it, the upper
  bound on top of a CPU frame pipeline is about 1.0x.

### Accelerate, AMX, SME, Neural Engine

- **vDSP** has float transforms only: FFT 3.1–3.6x faster than the exact NEON FFT, but 1.1x once
  strided int/float conversion is included; DCT-IV 1024 1.9–2.0x including conversion. Its
  output is two orders of magnitude more accurate than the reference, so it can never be
  bit-exact. Estimated end-to-end gain of a float path: 1.2x single-threaded, 1.05–1.1x at `-j 8`.
- **AMX / BNNS** (through Accelerate) work in fp16/fp32/fp64 or int8/int16 and need large dense
  matrix products; the decoder truncates 32x16 and 32x32 products and has no such products.
- **SME** is not available on the M1 Max; entering streaming mode costs more than a 256-point FFT,
  and its outer products do not match per-product truncation.
- **Neural Engine**: Core ML only, fp16, latency of hundreds of microseconds or more.

### CPU features and scheduling

- **NEON FFT**: 1145 ns for 512 points (about 1.6 cycles per radix-2 butterfly, 3.1x the scalar
  code), close to the throughput of its widening multiplies; radix-8 passes could give at most
  about 1.2x on the FFT, about 3% of a decode.
- **`SQDMULH` for `fMult`**: exact with a once-per-call check for `0x80000000`, but the active
  downmix kernel is bound by loads, stores and shifts (166 ns for 256 bins either way).
- **`sdot` / `smmla`** work on 8-bit operands: no use for 32x16 fixed point.
- **Efficiency cores** are 3.5–4.2x slower on the transform kernels, and a fork-join round waits
  for its slowest task: `-j 10` was 3–10% slower than `-j 8` except for objects, so on macOS the
  performance core count is the better thread count. Thread QoS classes made no difference.
- **File-level parallelism**: 8 concurrent single-threaded decodes give 7.1x the throughput of
  one `-j 1` decode (5.6x one `-j 8` decode); no accelerator comes close for batch transcoding.

Not measured: GPU energy for many concurrent streams, newer GPUs and Metal 4 command queues, SME on
M4, and the efficiency core trade-off on chips with more efficiency cores.
