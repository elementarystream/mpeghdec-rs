# Differential fuzzing

`cargo-fuzz` targets that run the C++ reference (mpeghdec r4.0.1, linked through `mpeghdec-cref`)
and the port on the same input and compare the results of every public API call. A panic or hang
of the port is a finding as well. This directory is a separate cargo workspace, built with nightly
Rust in the container of `fuzz/run.sh`.

## Targets

| Target | Input | Calls |
|---|---|---|
| `decode_mhas` | MHAS access units (`mhm1`) | `mpeghdecoder_init`, `process` / `processTimescale` (split, joined, truncated, repeated and skipped access units; timestamp jitter, gaps and jumps), `getSamples` (also with too small buffers), `flush` with seeks, `flushAndGet`, `setParam`, `setMhaConfig`, `destroy` |
| `decode_mha1` | `mpegh3daConfig()` and raw access units (`mha1`) | as above, starting with `setMhaConfig` |
| `ui_decode` | MHAS access units and a UUID | `mpegh_UI_FeedMHAS`, `ApplyXmlAction`, `UpdateMHAS`, `GetXmlSceneState`, `Set`/`GetPersistenceMemory`, then the decoder calls on the updated access units |

Every return value is compared, after `getSamples` also the output info, timestamps and output
buffer, in `ui_decode` also the updated MHAS bytes, XML output and persistence memory. The input
format is documented in [`src/dec.rs`](src/dec.rs); its first byte selects the thread count of the
port. With two or four threads the port returns frames one `process()` call later (PORTING.md,
"MpeghDecoderContext"), so output frames are compared as a sequence and `getSamples` is drained
after every call.

## Running

Fuzzing needs Docker, git and the reference sources in `work/reference/src`; everything else is
in the container:

```sh
git clone https://github.com/Fraunhofer-IIS/mpeghdec.git work/reference/src
git -C work/reference/src checkout 4448b69738da2fa5f2f2f2b0ce29eea32509e046
```

```sh
fuzz/seeds.sh                        # seed corpora from work/ into fuzz/corpus/<target>/
fuzz/run.sh decode_mhas 600 4        # fuzz one target for 600 s with 4 processes
fuzz/run.sh all 300 4                # every target in turn
FUZZ_TAG=checked FUZZ_CHECKED=1 fuzz/run.sh decode_mhas 3600 8   # overflow checks
FUZZ_TAG=nosimd FUZZ_NO_SIMD=1 fuzz/run.sh decode_mhas 3600 8    # portable code paths
fuzz/replay.sh --docker fuzz/artifacts/decode_mhas/crash-...     # replay in the container (x86-64)
fuzz/replay.sh fuzz/artifacts/decode_mhas/minimized-from-...     # replay natively
fuzz/replay.sh --trace decode_mhas FILE                          # with both call traces
MPEGHDEC_FUZZ_ECHO=1 fuzz/replay.sh FILE                         # print every call as it runs
```

`fuzz/seeds.sh` (stable Rust on the host; `MPEGHDEC_WORK_DIR`, `MPEGHDEC_TEST_STREAMS`) seeds
from the generated vectors, the test streams and all ISO conformance bitstreams with the settings
of their [test cases](../docs/research.md#conformance-bitstreams), at several target layouts and
thread counts, with seeks, timestamp gaps and parameter changes. `fuzz/corpus/` can be copied to
a machine without the test streams.

`fuzz/run.sh` builds without sanitizer (release semantics), runs libFuzzer in fork mode, then
minimizes and replays new findings in `fuzz/artifacts/<target>[-<tag>]/` (log next to it). The
first run builds the reference, so start one target alone first.

| Variable | Meaning |
|---|---|
| `FUZZ_PLATFORM` | `linux/amd64` (default, x86-64 reference variant) or `linux/arm64` (AArch64 variant) |
| `FUZZ_MAX_LEN`, `FUZZ_TIMEOUT` | libFuzzer `-max_len` (65536) and `-timeout` in seconds (60) |
| `FUZZ_REF_BUDGET_MS` | CPU budget of the reference part of one input (10000); slower inputs are skipped |
| `FUZZ_TMIN` | `0`: do not minimize findings |
| `FUZZ_CHECKED` | `1`: debug assertions and overflow checks in the port |
| `FUZZ_NO_SIMD` | `1`: the port without its runtime-selected SIMD paths |
| `FUZZ_TAG` | suffix of the artifact directory, for parallel campaigns of one target |
| `FUZZ_ARGS` | more libFuzzer arguments |

## Triage

A **difference** panics with a report (`mpeghdec-rs differential fuzzing: DIFFERENCE in ...`)
naming the first differing call or output frame, both results, the first differing sample or byte
and the preceding calls; `fuzz/replay.sh` prints the same report. Every panic, also in a worker
thread, is saved with its input to `fuzz/artifacts/reports/` (`MPEGHDEC_FUZZ_REPORT_DIR`), since
fork mode drops process output and timing-dependent findings may not replay. For a `timeout-*`
artifact, `fuzz/replay.sh` runs each side alone and reports which one hangs.

Undefined behaviour of the reference must not be reported as a port bug (`src/guard.c`,
`src/dec.rs`):

- **Crashes and hangs**: every reference call runs under a signal guard and a CPU budget; the run
  stops at that call and the traces are compared up to it.
- **Heap**: on Linux the reference allocates from its own arena, so overruns cannot corrupt the
  fuzzer.
- **Uninitialized memory**: stack, `malloc` blocks and a 64-byte redzone are filled with a pattern;
  if the reference trace changes with other patterns, the input is skipped (`RefNondeterministic`).
- **Deviation sites**: where the C++ code is undefined, the port returns an error or uses defined
  values and calls `mpeghdec_fdk::deviation::hit()`; both runs stop before that API call.
- **API misuse undefined in C** (persistence blocks of 2 to 9 bytes, timestamps of 2^63 ns or
  more, timescale 0) is not generated.

The deviation sites: MCT with more than 64 bands (the C++ code writes past `pairCoeffQSfb` and
`pairCoeffQSfbPrev[pair]`); MCT stereo filling without MCT channels; negative shifts in complex
prediction; negative switch group and group indices of the UI state, and signals beyond its
on/off array; DRC gain sets, sequences, node buffers, subband gains, MPEG-H requests and downmix
ID requests out of range; gVBAP vertex and triangle lists beyond the C++ arrays, uninitialized
`subsetIndex` entries, imaginary speakers without a horizontal speaker, empty triangulations, OAM
frame and interpolation lengths out of range; format converter setups with more than 24 outputs,
overflowing conversion vectors, compact downmix matrices and templates out of range, missing
equalizers; UI persistence blocks of 2 to 9 bytes and inconsistent `UpdateMHAS` lengths. Where the
port reproduces an out-of-bounds access exactly (the members it reads or writes), it is compared
as usual. No seed input reaches a deviation site.

What remains is analysed by hand: out-of-bounds reads that reach initialized memory, undefined
behaviour that the reference compiler resolves differently at a call site, and results specific to
one variant (replay an x86 finding with `fuzz/replay.sh --docker` on an arm64 machine).
