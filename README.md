# mpeghdec-rs

A Rust port of the [Fraunhofer MPEG-H decoder (mpeghdec)](https://github.com/Fraunhofer-IIS/mpeghdec),
an implementation of the MPEG-H 3D Audio standard (ISO/IEC 23008-3), Baseline profile.

> **Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.**
>
> This project is a modified version of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1, commit
> `4448b69`): the C++ sources were translated to Rust and modified from 2026-09-27 to 2026-10-13. It
> is not endorsed by Fraunhofer. The complete license text is in [LICENSE.txt](LICENSE.txt); it
> applies to this port unchanged. Note that the license grants **no patent rights**: using MPEG-H
> Audio requires patent licenses from the respective patent owners (see LICENSE.txt, section 3).

## Features

- **Bit-exact.** The decoded PCM output is identical to the reference decoder built for the same
  architecture. The reference differs between architectures in the least significant bits; the
  port reproduces the AArch64 build (Apple arm64, NEON disabled), the x86-64 build and the 32 bit
  x86 build, selected by the target (see [PORTING.md](PORTING.md#other-architectures-x86)).
  Verified end to end on test streams, generated test vectors at all tested target layouts and
  the ISO/IEC 23008-9 conformance bitstreams, and function by function against the C++ code.
- **Fast.** About 3x faster than the reference single-threaded. With more threads the decoder
  pipelines frames and parallelizes within frames, and long files are decoded in parallel chunks.
  The output is identical for every thread count.
- **Complete.** All libraries of the reference decoder are ported, including DRC, the format
  converter, object rendering (gVBAP), the UI manager and earcons, plus a command line decoder
  with the options of the reference demo.
- **Three APIs.** A safe Rust API (`mpeghdec`), the straight port of the C API
  (`mpeghdecoder_*()`, `mpegh_UI_*()`) and a C library (`mpeghdec-capi`) that replaces the
  reference `mpeghdec` library without changes to its users.
- **Safe Rust** apart from a few documented places (SIMD intrinsics, the thread pool, byte views
  of integer buffers for the state digest and where the C++ code reuses a work buffer as bytes)
  and the C ABI.

| C++ library | Rust crate |
| --- | --- |
| libFDK, libSYS | `mpeghdec-fdk` |
| libArithCoding | `mpeghdec-arith` |
| libDRCdec | `mpeghdec-drc` |
| libPCMutils | `mpeghdec-pcmutils` |
| libMpegTPDec | `mpeghdec-tpdec` |
| libIGFdec | `mpeghdec-igf` |
| libgVBAPRenderer | `mpeghdec-gvbap` |
| libFormatConverter | `mpeghdec-formatconverter` |
| libUIManager, libMpeghUIMan | `mpeghdec-uimanager` |
| libMpeghDec | `mpeghdec-core` |
| mpeghdecoder / mpeghUIManager API | `mpeghdec` |
| demo decoder | `mpeghdec-cli` (binary `mpeghdec`) |
| C API (`libmpeghdec`) | `mpeghdec-capi` (not published) |

## Performance

Apple M1 Max, wall clock, best of 3 (`scripts/bench.py`). The reference is the single-threaded
C++ `mpeghDecoder` in a release build. Every output is byte-identical to the reference output.

| Case | Reference | `-j 1` | `-j 2` (default) | `-j 8` |
| --- | --- | --- | --- | --- |
| 30 min stereo, stereo output | 13.18 s | 4.44 s (3.0x) | 2.72 s (4.8x) | **0.74 s (17.8x)** |
| 22.2 to stereo (60 s) | 2.29 s | 1.15 s (2.0x) | 0.80 s (2.9x) | 0.74 s (3.1x) |
| 22.2 to 22.2 (60 s) | 4.73 s | 1.60 s (3.0x) | 0.86 s (5.5x) | 0.76 s (6.2x) |
| 7.1.4 to 7.1.4 (60 s) | 2.34 s | 0.80 s (2.9x) | 0.46 s (5.1x) | 0.38 s (6.1x) |
| 4 objects to 7.1.4 (60 s) | 1.32 s | 0.28 s (4.7x) | 0.18 s (7.3x) | 0.18 s (7.3x) |
| 5.1 with MCT to 5.1 (60 s) | 1.12 s | 0.38 s (2.9x) | 0.21 s (5.2x) | 0.19 s (6.0x) |

- `-j 2`: frame pipelining (the rendering of an access unit runs while the next one is decoded),
  with about the memory use of the reference.
- `-j 8`: additionally a thread pool within frames and, for long files that repeat their
  configuration, chunked parallel decoding (about 200 MB of memory). The 60 s vectors have a
  single configuration and are decoded sequentially.

On x86-64 the decoder selects AVX2 code paths at run time (same output); it is 2.4–2.9x faster
than the x86-64 reference single-threaded and 3.4–5.1x with two threads. See
[docs/optimization.md](docs/optimization.md) for details.

## Command line decoder

```sh
cargo install mpeghdec-cli
mpeghdec -if input.mp4 -of output.wav -tl 2
```

The options are those of the reference demo `mpeghDecoder`:

- `-if` input: the first MPEG-H track (`mhm1` or `mha1`) of an MP4 file, or a raw MHAS stream
  (`.mhas`; with timestamps from a `.pts.txt` side file of `byte_offset size pts` lines at 90 kHz
  if present, otherwise 1024 samples at 48 kHz per access unit)
- `-of` output WAV file (24 bit)
- `-tl` CICP index of the target loudspeaker layout (default 6, i.e. 5.1; 2 is stereo)
- `-rl` target loudness, `-dse` DRC effect type, `-db` / `-dc` DRC boost / attenuation, `-dam`
  album mode
- `-y` / `-z` first / last access unit, `-sf` / `-st` seek from / to access unit
- `-script`, `-xmlSceneState`, `-persistFile`: MPEG-H UI manager (XML interactivity script, scene
  state output, persistence data); `-bsof` writes the processed bitstream as raw MHAS

Options not in the reference demo:

- `-j N` number of threads (default 2, at most the number of CPU cores; `-j 1` decodes on the
  calling thread only)
- `-chunks N` chunked parallel decoding with N chunks (default: automatic for long inputs from
  `-j 4`; `-chunks 0` decodes sequentially). Each chunk boundary is verified by comparing the
  complete decoder states, so the output is identical to the sequential decoding. `-cw`, `-cv`,
  `-cmem` and `-cra` tune the warm-up, the verification points, the memory limit and the random
  access points (see [PORTING.md](PORTING.md#chunked-parallel-decoding)). `MPEGHDEC_CHUNK_LOG=1`
  logs the chunk decisions, `MPEGHDEC_CHUNK_DEBUG=1` also the state fields that differ at
  rejected boundaries.
- `-fast` [fast mode](#fast-mode-not-bit-exact) (not bit-exact)

## Rust library

The `mpeghdec` crate provides `Decoder` (decoding with timestamp handling) and `UiManager` (user
interactivity):

```rust
let mut dec = mpeghdec::Decoder::new(2)?; // stereo output
let mut out = vec![0i32; dec.output_buffer_len()];
for (au, pts) in access_units {
    dec.process_timescale(au, pts, 48000)?;
    while let Some(info) = dec.get_samples(&mut out)? {
        let samples = &out[..info.samples()]; // interleaved, 24 bit in the upper bits
    }
}
dec.flush_and_get()?;
while let Some(info) = dec.get_samples(&mut out)? { /* ... */ }
```

A decoder runs on the calling thread by default; `Decoder::with_threads(layout, n)` /
`set_threads(n)` add frame pipelining and parallelism within frames. With more than one thread
`get_samples()` returns the output frames one `process()` call later; the sequence of frames is
the same if `get_samples()` is called until it returns `None` after every `process()` / `flush()`
call, as above. `mpeghdec::chunked::decode_chunked()` decodes a whole seekable input in parallel
chunks.

The straight port of the C API, with the functions, arguments and error codes of
`mpeghdecoder.h` and `mpeghUIManager.h`, is in `mpeghdec_core::mpeghdecoder` and
`mpeghdec_uimanager::mpegh_ui_manager`. The other `mpeghdec-*` crates are internals without
stability guarantees.

## C library

`mpeghdec-capi` builds a drop-in replacement for the `mpeghdec` library of the reference
implementation: the 17 functions of its unchanged headers (`crates/capi/include/mpeghdec/`), with
the same behaviour and error codes.

```sh
cargo build --release -p mpeghdec-capi   # target/release/libmpeghdec_capi.{a,so,dylib}
```

Install the library as `libmpeghdec.*` together with the headers and `mpeghdec.pc`
(`crates/capi/mpeghdec.pc.in`); `scripts/capi-test.sh` shows the installation and links the
reference demo decoder against it. `MPEGHDEC_THREADS=n` enables multi-threading for C users (the
output frames then come one `mpeghdecoder_process()` call later, see above).

## Fast mode (not bit-exact)

The opt-in fast mode (`-fast`, `Decoder::set_fast_mode(true)`) skips the STFT format
converter where it is equivalent to a delayed copy of the channels (a channel layout rendered to
itself or copied into a larger one, e.g. stereo to stereo or stereo to 5.1). The output then
differs from the reference by the rounding noise of its active downmix: at most 842 LSB and
52 LSB rms of 24 bit over all test material (about 3 LSB of 16 bit). Single-threaded decoding
is 1.7–1.8x faster; with frame pipelining the gain is small. See
[PORTING.md](PORTING.md#fast-mode-opt-in-not-bit-exact) for the exact conditions.

## Development

```sh
cargo build --release
cargo test --workspace --release
```

The differential tests (`crates/cref`) compile the C++ reference implementation and compare
individual functions and the complete decoder with the port. They and the end-to-end tests need
local, git-ignored material in `work/` (`MPEGHDEC_WORK_DIR` overrides the location) and are
skipped without it:

| Path | Content | Source |
| --- | --- | --- |
| `work/reference/` | C++ sources (`src`), reference builds (`build-<arch>`), resolved and preprocessed sources | `scripts/setup-reference.sh` |
| `work/vectors/` | generated test vectors | `scripts/make_vectors.py` (needs the [libmpeghe](https://github.com/ittiam-systems/libmpeghe) encoder) |
| `work/conformance/` | ISO/IEC 23008-9 conformance bitstreams (not redistributable) | `scripts/conformance.py fetch` |
| `work/streams/` | further test streams with reference hashes (`MPEGHDEC_TEST_STREAMS`) | not distributed |

`scripts/setup-reference.sh` needs git, CMake, clang, `unifdef` and Python 3.

```sh
scripts/setup-reference.sh                  # clone and build the reference into work/reference
cargo test -p mpeghdec-cref --release       # differential tests
cargo test -p mpeghdec-cref --profile checked   # with overflow checks
scripts/e2e.py --samples short --vectors    # end to end against the reference hashes
scripts/e2e.py --samples none --conformance # ISO conformance bitstreams
scripts/bench.py                            # decoding speed
scripts/capi-test.sh                        # C library as a drop-in replacement
```

The repository layout:

- `crates/` – one crate per C++ library, the API crate `mpeghdec`, the command line decoder and
  the C library (`crates/capi`)
- `crates/cref` – differential tests against the C++ reference (not published)
- `fuzz/` – differential fuzzing against the C++ reference ([fuzz/README.md](fuzz/README.md))
- `scripts/` – reference setup, test vector generation, end-to-end tests and benchmarks
- `docs/` – [optimizations](docs/optimization.md) and [research notes](docs/research.md)

See [PORTING.md](PORTING.md) for the porting conventions, the architecture variants and the
multi-core design.

## License

[Software License for The Fraunhofer FDK MPEG-H Software](LICENSE.txt). The license grants no
patent rights; see section 3.
