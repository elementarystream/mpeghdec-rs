#!/bin/bash
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
# Runs a differential fuzz target (fuzz/, fuzz/README.md) in the fuzz container
# (fuzz/Dockerfile), so that nothing has to be installed on the host.
#
#   fuzz/run.sh TARGET [SECONDS] [JOBS]   fuzz TARGET for SECONDS (default 600) with JOBS
#                                           processes (default 4; libFuzzer fork mode), then
#                                           minimize the new findings and replay them
#   fuzz/run.sh all [SECONDS] [JOBS]      every target in turn, SECONDS each
#   fuzz/run.sh exec COMMAND...           run a command in the container (from fuzz/)
#
# TARGET: decode_mhas decode_mha1 ui_decode
#
# The repository is mounted at the same absolute path (with the main checkout, whose work/ holds
# the reference sources), the container runs as the calling user, and cargo uses target/fuzz
# (target/fuzz-arm64 for the arm64 image). The corpus is fuzz/corpus/TARGET (build it with
# fuzz/seeds.sh), findings go to fuzz/artifacts/TARGET; both are git-ignored.
#
# Environment:
#   FUZZ_PLATFORM   linux/amd64 (default: the x86 reference variant) or linux/arm64 (the AArch64
#                   variant; native on Apple silicon)
#   FUZZ_MAX_LEN    libFuzzer -max_len (default 65536)
#   FUZZ_TIMEOUT    libFuzzer -timeout in seconds for one input (default 60)
#   FUZZ_REF_BUDGET_MS  CPU budget of the reference part of one input (default 10000; a
#                   reference that needs more is treated as hanging and the input skipped)
#   FUZZ_TMIN       0: do not minimize findings (default 1)
#   FUZZ_CHECKED    1: build the port with debug assertions and overflow checks (as the checked
#                   profile; target/fuzz*-checked); an overflow panic then shows arithmetic that
#                   relies on wrap-around. Default: release semantics (cargo fuzz -O).
#   FUZZ_NO_SIMD    1: run the port without its runtime-selected SIMD code paths
#                   (MPEGHDEC_NO_SIMD)
#   FUZZ_TAG        suffix of the artifact directory and the log (fuzz/artifacts/TARGET-TAG), to
#                   run several campaigns of one target at the same time
#   FUZZ_ARGS       more libFuzzer arguments
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PLATFORM="${FUZZ_PLATFORM:-linux/amd64}"
case "$PLATFORM" in
  linux/amd64) SUFFIX="" ;;
  linux/arm64) SUFFIX="-arm64" ;;
  *) echo "unsupported FUZZ_PLATFORM $PLATFORM" >&2; exit 2 ;;
esac
IMAGE="mpeghdec-rs-fuzz$SUFFIX"
TARGETS=(decode_mhas decode_mha1 ui_decode)

[ $# -ge 1 ] || { sed -n '5,36p' "$0"; exit 2; }

if [ "${MPEGHDEC_FUZZ_IN_CONTAINER:-}" != 1 ]; then
  docker build --platform "$PLATFORM" -q -t "$IMAGE" - < "$REPO/fuzz/Dockerfile" >/dev/null
  MAIN="$(dirname "$(git -C "$REPO" rev-parse --path-format=absolute --git-common-dir)")"
  mounts=(-v "$MAIN:$MAIN")
  case "$REPO/" in "$MAIN"/*) ;; *) mounts+=(-v "$REPO:$REPO") ;; esac
  # crates.io downloads persist in the target directory
  REG="$REPO/target/fuzz$SUFFIX/cargo-registry"
  mkdir -p "$REG"
  mounts+=(-v "$REG:/opt/cargo/registry")
  envs=()
  [ "${FUZZ_NO_SIMD:-0}" = 1 ] && envs+=(-e MPEGHDEC_NO_SIMD=1)
  tty=()
  [ -t 1 ] && tty=(-t)
  exec docker run --rm ${tty[@]+"${tty[@]}"} --init --platform "$PLATFORM" --user "$(id -u):$(id -g)" -e HOME=/tmp \
    "${mounts[@]}" ${envs[@]+"${envs[@]}"} -e MPEGHDEC_FUZZ_IN_CONTAINER=1 \
    -e CARGO_TARGET_DIR="$REPO/target/fuzz$SUFFIX" \
    -e MPEGHDEC_WORK_DIR="$MAIN/work" \
    -e MPEGHDEC_FUZZ_REF_BUDGET_MS="${FUZZ_REF_BUDGET_MS:-10000}" \
    -e FUZZ_MAX_LEN -e FUZZ_TIMEOUT -e FUZZ_TMIN -e FUZZ_CHECKED -e FUZZ_TAG -e FUZZ_ARGS \
    -e FUZZ_PLATFORM="$PLATFORM" \
    -w "$REPO/fuzz" "$IMAGE" "$REPO/fuzz/run.sh" "$@"
fi

# --- inside the container ------------------------------------------------------------------
cd "$REPO/fuzz"
# cargo fuzz build options: release semantics (-O: no debug assertions, no overflow checks), no
# sanitizer (the port is safe Rust; the reference is not instrumented).
FZ=(-O -s none)
if [ "${FUZZ_CHECKED:-0}" = 1 ]; then
  FZ=(-O -a -s none)
  export CARGO_TARGET_DIR="$CARGO_TARGET_DIR-checked"
fi
if [ "$1" = exec ]; then
  shift
  exec "$@"
fi
TARGET="$1"
SECONDS_PER="${2:-600}"
JOBS="${3:-4}"
if [ "$TARGET" = all ]; then
  for t in "${TARGETS[@]}"; do "$0" "$t" "$SECONDS_PER" "$JOBS"; done
  exit 0
fi
case " ${TARGETS[*]} " in *" $TARGET "*) ;; *) echo "unknown target $TARGET" >&2; exit 2 ;; esac

NAME="$TARGET${FUZZ_TAG:+-$FUZZ_TAG}"
ART="$REPO/fuzz/artifacts/$NAME"
mkdir -p "$ART" "$REPO/fuzz/corpus/$TARGET"
before="$(ls "$ART" | sort)"
CARGO_BUILD_JOBS="$JOBS" cargo fuzz build "${FZ[@]}" "$TARGET"
cargo build -q --release -j "$JOBS" --features tools --bin replay
echo "== $NAME: $SECONDS_PER s, $JOBS jobs, $PLATFORM${FUZZ_CHECKED:+ (checked)}${MPEGHDEC_NO_SIMD:+ (no SIMD)}"
# Fork mode: the run continues after findings; libFuzzer writes them to $ART.
# shellcheck disable=SC2086
cargo fuzz run "${FZ[@]}" "$TARGET" "$REPO/fuzz/corpus/$TARGET" -- \
  -fork="$JOBS" -ignore_crashes=1 -ignore_timeouts=1 -ignore_ooms=1 \
  -max_total_time="$SECONDS_PER" -max_len="${FUZZ_MAX_LEN:-65536}" -timeout="${FUZZ_TIMEOUT:-60}" \
  -rss_limit_mb=4096 -print_final_stats=1 -artifact_prefix="$ART/" ${FUZZ_ARGS:-} \
  2>&1 | tee "$ART/../$NAME.log" | grep --line-buffered -E '^#|INFO: -fork|DONE|CRASH|ERROR|DIFFERENCE|panicked|Test unit written|stat::' || true

new="$(comm -13 <(echo "$before") <(ls "$ART" | sort) | grep -E '^(crash|timeout|oom|leak)-' || true)"
if [ -z "$new" ]; then
  echo "== $NAME: no new findings"
  exit 0
fi
echo "== $NAME: new findings:"
echo "$new"
for f in $new; do
  if [ "${FUZZ_TMIN:-1}" = 1 ] && [[ "$f" == crash-* ]]; then
    cargo fuzz tmin "${FZ[@]}" -r 2000 "$TARGET" "$ART/$f" >"$ART/$f.tmin.log" 2>&1 || true
  fi
done
# Replay every new finding (and its minimized form): the report names the first differing call.
for f in $new; do
  # cargo fuzz tmin writes to the untagged artifact directory
  m="$REPO/fuzz/artifacts/$TARGET/minimized-from-${f#crash-}"
  for g in "$ART/$f" "$m"; do
    [ -f "$g" ] || continue
    echo "== replay $g"
    "$CARGO_TARGET_DIR/release/replay" "$TARGET" "$g" || true
  done
done
