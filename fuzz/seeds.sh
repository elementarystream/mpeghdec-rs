#!/bin/bash
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
# Builds the seed corpora fuzz/corpus/<target>/ with src/bin/seedgen.rs from the generated
# vectors, the ISO conformance bitstreams (with the settings of work/conformance/cases.json, see
# docs/research.md#conformance-bitstreams) and the test streams in work/, as far as present.
# Existing corpus files are kept.
#
#   fuzz/seeds.sh [FILE.mp4|FILE.mhas...]
set -euo pipefail
FUZZ="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$FUZZ/.." && pwd)"
WORK="${MPEGHDEC_WORK_DIR:-$REPO/work}"
STREAMS="${MPEGHDEC_TEST_STREAMS:-$WORK/streams}"
TARGET_DIR="${CARGO_TARGET_DIR:-$REPO/target/fuzz-host}"
CARGO_TARGET_DIR="$TARGET_DIR" cargo build -q --release --manifest-path "$FUZZ/Cargo.toml" --features tools --bin seedgen
if [ $# -eq 0 ]; then
  shopt -s nullglob
  set -- "$WORK"/vectors/*/*.mp4 "$WORK"/conformance/mp4/*.mp4 "$STREAMS"/*.mhas
fi
[ $# -gt 0 ] || { echo "no test streams in $WORK" >&2; exit 1; }
cases=()
if [ -f "$WORK/conformance/cases.json" ]; then
  CASES="$TARGET_DIR/conformance-cases.txt"
  # NAME STREAM TL RL DSE PRESET (- if unused); the preset of a -script is its paramInt
  python3 - "$WORK/conformance" > "$CASES" <<'PY'
import json, os, re, sys
d = sys.argv[1]
for name, c in json.load(open(os.path.join(d, "cases.json"))).items():
    if not c.get("applicable"):
        continue
    a = dict(zip(c["args"][::2], c["args"][1::2]))
    preset = "-"
    if "-script" in a:
        m = re.search(r'paramInt="(\d+)"', open(os.path.join(d, a["-script"])).read())
        preset = m.group(1) if m else "-"
    print(name, os.path.join(d, c["stream"]), a.get("-tl", "6"), a.get("-rl", "-"), a.get("-dse", "-"), preset)
PY
  cases=(--cases "$CASES")
fi
exec "$TARGET_DIR/release/seedgen" "$FUZZ/corpus" ${cases[@]+"${cases[@]}"} "$@"
