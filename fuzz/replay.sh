#!/bin/bash
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
# Replays fuzz inputs (findings in fuzz/artifacts/<target>/, or any corpus file) and prints what
# each does: match, an aborted reference call, a reference that reads uninitialized memory, or a
# DIFFERENCE with the first differing call; a port panic shows its message (fuzz/README.md).
#
#   fuzz/replay.sh [--docker] [--trace] [TARGET|auto] FILE|DIR...
#
# TARGET defaults to `auto` (taken from the path: fuzz/artifacts/<target>[-<tag>]/...).
# --docker  replay in the fuzz container (fuzz/run.sh; FUZZ_PLATFORM selects the variant, by
#           default linux/amd64 = the x86-64 reference); without it, replay natively on this host
#           (its own reference variant).
# --trace   also print both call traces.
set -euo pipefail
FUZZ="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$FUZZ/.." && pwd)"
docker=0
args=()
for a in "$@"; do
  case "$a" in
    --docker) docker=1 ;;
    *) args+=("$a") ;;
  esac
done
[ ${#args[@]} -ge 1 ] || { sed -n '5,15p' "$0"; exit 2; }
case "${args[0]}" in
  --trace) first="${args[1]:-}" ;;
  *) first="${args[0]}" ;;
esac
# Without a target name, infer it from the paths.
case "$first" in
  decode_mhas|decode_mha1|ui_decode|auto) ;;
  *)
    if [ "${args[0]}" = --trace ]; then args=(--trace auto "${args[@]:1}"); else args=(auto "${args[@]}"); fi
    ;;
esac
# Absolute paths (the container sees the repository at the same path).
for i in "${!args[@]}"; do
  if [ -e "${args[$i]}" ]; then args[$i]="$(cd "$(dirname "${args[$i]}")" && pwd)/$(basename "${args[$i]}")"; fi
done
if [ "$docker" = 1 ]; then
  exec "$REPO/fuzz/run.sh" exec bash -c \
    'cargo build -q --release --features tools --bin replay && exec "$CARGO_TARGET_DIR/release/replay" "$@"' \
    replay "${args[@]}"
fi
TARGET_DIR="${CARGO_TARGET_DIR:-$REPO/target/fuzz-host}"
CARGO_TARGET_DIR="$TARGET_DIR" cargo build -q --release --manifest-path "$FUZZ/Cargo.toml" --features tools --bin replay
exec "$TARGET_DIR/release/replay" "${args[@]}"
