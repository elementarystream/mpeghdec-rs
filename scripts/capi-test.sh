#!/bin/bash
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
# Checks the C ABI (crates/capi) as a drop-in replacement for the mpeghdec library:
#
#   1. builds mpeghdec-capi and installs it into target/capi/prefix like the reference install
#      (lib/libmpeghdec.{a,dylib|so}, include/mpeghdec/*.h, lib/pkgconfig/mpeghdec.pc);
#   2. checks that the shared library exports exactly the 17 functions of the C API;
#   3. builds crates/capi/tests/capi_smoke.c with pkg-config against the installed library and
#      against the reference library, and runs both;
#   4. links the object files of the reference demo decoder (work/reference/build-<arch>) with
#      the installed static library instead of the reference library (only the demo's command
#      line parser and WAV writer come from the reference library) and runs scripts/e2e.py with
#      it, with one and with four decoder threads (MPEGHDEC_THREADS).
#
# Usage: scripts/capi-test.sh [e2e.py arguments, default: --samples all --vectors --conformance]
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${MPEGHDEC_WORK_DIR:-$REPO/work}"
TARGET_DIR="${CARGO_TARGET_DIR:-$REPO/target}"
OUT="$TARGET_DIR/capi"
PREFIX="$OUT/prefix"
case "$(uname -m)" in
  arm64|aarch64) BUILD="$WORK/reference/build-aarch64" ;;
  x86_64|amd64) BUILD="$WORK/reference/build-x86_64" ;;
  *) echo "unsupported host architecture $(uname -m)" >&2; exit 1 ;;
esac
[ $# -gt 0 ] || set -- --samples all --vectors --conformance

echo "== build and install"
cargo build -q --release -p mpeghdec-capi --manifest-path "$REPO/Cargo.toml"
NATIVE="$(cargo rustc -q --release -p mpeghdec-capi --manifest-path "$REPO/Cargo.toml" \
  --crate-type staticlib -- --print native-static-libs 2>&1 \
  | sed -n 's/.*native-static-libs: //p' | tail -1)"
rm -rf "$PREFIX"
mkdir -p "$PREFIX/lib/pkgconfig" "$PREFIX/include/mpeghdec" "$OUT/bin"
cp "$REPO"/crates/capi/include/mpeghdec/*.h "$PREFIX/include/mpeghdec/"
cp "$TARGET_DIR/release/libmpeghdec_capi.a" "$PREFIX/lib/libmpeghdec.a"
if [ "$(uname)" = Darwin ]; then SO=dylib; else SO=so; fi
cp "$TARGET_DIR/release/libmpeghdec_capi.$SO" "$PREFIX/lib/libmpeghdec.$SO"
sed -e "s|@PREFIX@|$PREFIX|" -e "s|@NATIVE_STATIC_LIBS@|$NATIVE|" \
  "$REPO/crates/capi/mpeghdec.pc.in" > "$PREFIX/lib/pkgconfig/mpeghdec.pc"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig"

echo "== exports"
EXPECTED="mpegh_UI_ApplyXmlAction mpegh_UI_FeedMHAS mpegh_UI_GetPersistenceMemory
mpegh_UI_GetXmlSceneState mpegh_UI_Manager_Close mpegh_UI_Manager_Open
mpegh_UI_SetPersistenceMemory mpegh_UI_UpdateMHAS mpeghdecoder_destroy mpeghdecoder_flush
mpeghdecoder_flushAndGet mpeghdecoder_getSamples mpeghdecoder_init mpeghdecoder_process
mpeghdecoder_processTimescale mpeghdecoder_setMhaConfig mpeghdecoder_setParam"
if [ "$SO" = dylib ]; then
  ACTUAL="$(nm -gU "$PREFIX/lib/libmpeghdec.dylib" | awk '$2 == "T" {print substr($3, 2)}')"
else
  ACTUAL="$(nm -D --defined-only "$PREFIX/lib/libmpeghdec.so" | awk '$2 == "T" {print $3}')"
fi
if [ "$(echo $ACTUAL | tr ' ' '\n' | sort)" != "$(echo $EXPECTED | tr ' ' '\n' | sort)" ]; then
  echo "unexpected exports:" $ACTUAL >&2
  exit 1
fi
echo "17 symbols"

echo "== smoke test"
CC="${CC:-cc}"
CXX="${CXX:-c++}"
# shellcheck disable=SC2046,SC2086
"$CC" -O1 "$REPO/crates/capi/tests/capi_smoke.c" -o "$OUT/bin/capi_smoke_static" \
  $(pkg-config --cflags mpeghdec) "$PREFIX/lib/libmpeghdec.a" $NATIVE
echo -n "static: "
"$OUT/bin/capi_smoke_static"
# shellcheck disable=SC2046
"$CC" -O1 "$REPO/crates/capi/tests/capi_smoke.c" -o "$OUT/bin/capi_smoke_shared" \
  $(pkg-config --cflags --libs mpeghdec) -Wl,-rpath,"$PREFIX/lib"
echo -n "shared: "
"$OUT/bin/capi_smoke_shared"
if [ -f "$BUILD/lib/libmpeghdec.a" ]; then
  mkdir -p "$OUT/refinc/mpeghdec"
  cp "$WORK"/reference/src/include/*.h "$OUT/refinc/mpeghdec/"
  "$CC" -O1 -c -I"$OUT/refinc" "$REPO/crates/capi/tests/capi_smoke.c" -o "$OUT/capi_smoke_ref.o"
  "$CXX" "$OUT/capi_smoke_ref.o" "$BUILD/lib/libmpeghdec.a" -lm -o "$OUT/bin/capi_smoke_ref"
  echo -n "reference: "
  "$OUT/bin/capi_smoke_ref"
fi

echo "== reference demo linked with the C ABI"
OBJ="$BUILD/demo/CMakeFiles/mpeghDecoder.dir"
if [ ! -d "$OBJ" ]; then
  echo "no reference build in $BUILD (scripts/setup-reference.sh), skipped"
  exit 0
fi
# The demo also uses the command line parser and the WAV writer of the reference library
# (libSYS, not part of the API): they are taken from the reference library.
SYS="$OUT/demo-sys"
rm -rf "$SYS" && mkdir -p "$SYS"
(cd "$SYS" && ar x "$BUILD/lib/libmpeghdec.a" wav_file.cpp.o cmdl_parser.cpp.o genericStds.cpp.o)
# shellcheck disable=SC2046,SC2086
"$CXX" -O3 "$OBJ"/mpeghDecoder/main_mpeghDecoder.cpp.o \
  "$OBJ"/mpeghUiManager/mpeghUiManagerProcessor.cpp.o "$SYS"/*.o -o "$OUT/bin/mpeghDecoder" \
  "$PREFIX/lib/libmpeghdec.a" "$BUILD/lib/libmmtisobmff.a" "$BUILD/lib/libilo.a" $NATIVE -lm
for threads in 1 4; do
  echo "-- MPEGHDEC_THREADS=$threads"
  MPEGHDEC_THREADS=$threads "$REPO/scripts/e2e.py" --exe "$OUT/bin/mpeghDecoder" "$@" | tail -1
done
