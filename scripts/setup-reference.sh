#!/bin/bash
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
# Sets up the C++ reference implementation used for porting and differential testing:
#
#   work/reference/src            Fraunhofer mpeghdec checkout (pinned commit)
#   work/reference/build-<arch>   CMake release build for the host (aarch64 or x86_64;
#                                 bin/mpeghDecoder is the reference decoder)
#   work/reference/build-i686     with --i686 on x86-64 hosts: 32 bit x86 build (clang -m32, needs
#                                 the 32 bit C++ libraries, Debian/Ubuntu: g++-multilib)
#   work/reference/resolved       sources with the AArch64/Apple (no NEON) conditionals resolved
#                                 and license headers stripped (for reading)
#   work/reference/pp             preprocessed translation units (*.ii) of the host build: the
#                                 ground truth of which code is actually compiled
#
# MPEGHDEC_WORK_DIR overrides the location of work/.
#
# The reference is built with clang (Apple clang on macOS) unless CC/CXX say otherwise: some of its
# results depend on the compiler (undefined shifts differ between scalar and vectorized code). The
# build follows the code paths of the host architecture (AArch64 or x86, see PORTING.md).
#
# Usage: scripts/setup-reference.sh [--skip-clone] [--skip-build] [--i686]
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${MPEGHDEC_WORK_DIR:-$REPO/work}"
REF="$WORK/reference"
C_DIR="$REF/src"
case "$(uname -m)" in
  arm64|aarch64) BUILD="$REF/build-aarch64" ;;
  x86_64|amd64) BUILD="$REF/build-x86_64" ;;
  *) echo "unsupported host architecture $(uname -m)" >&2; exit 1 ;;
esac
URL="https://github.com/Fraunhofer-IIS/mpeghdec.git"
COMMIT="4448b69738da2fa5f2f2f2b0ce29eea32509e046" # r4.0.1

skip_clone=0; skip_build=0; i686=0
for a in "$@"; do
  case "$a" in
    --skip-clone) skip_clone=1 ;;
    --skip-build) skip_build=1 ;;
    --i686) i686=1 ;;
  esac
done

mkdir -p "$REF"
if [ $skip_clone -eq 0 ] && [ ! -d "$C_DIR/.git" ]; then
  git clone "$URL" "$C_DIR"
  git -C "$C_DIR" checkout "$COMMIT"
fi

if [ $skip_build -eq 0 ]; then
  CC="${CC:-clang}" CXX="${CXX:-clang++}" \
    cmake -S "$C_DIR" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  cmake --build "$BUILD" -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
fi
if [ $i686 -eq 1 ]; then
  CC="${CC:-clang}" CXX="${CXX:-clang++}" CFLAGS="-m32" CXXFLAGS="-m32" LDFLAGS="-m32" \
    cmake -S "$C_DIR" -B "$REF/build-i686" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$REF/build-i686" -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
fi

# Resolve architecture conditionals for Apple arm64 without NEON (the x86 code paths differ in
# the primitives of libFDK, see PORTING.md).
SYMS=(-D__arm__ -D__ARM_ARCH_8__ -D__aarch64__ -D__GNUC__ -D__clang__ -D__llvm__ -D__APPLE__ -D__LP64__
  -DSINETABLE_16BIT -DWINDOWTABLE_16BIT -DPOW2COEFF_16BIT -DLDCOEFF_16BIT -DARCH_PREFER_MULT_32x16 -DNDEBUG
  -U__ARM_NEON__ -U__ARM_NEON -U__TARGET_FEATURE_NEON -U__ARM_AARCH64_NEON__ -U__ARM_ARCH_5TE__
  -U__ARM_ARCH_6__ -U__ARM_ARCH_7_A__ -U__ARM_ARCH_7M__ -U__ARM_ARCH_7EM__ -U__ARM_ARCH_7_R__ -U__x86__
  -U_MSC_VER -U_M_ARM64 -U__riscv64__ -U__riscv32__ -U__riscv -U__WASM__ -U__wasm__ -U__wasm32__
  -U__THUMBEL__ -U__TARGET_FEATURE_DSPMUL -U__TARGET_ARCH_7E_M -U__x86_64__ -U__i386__ -U__i686__
  -U__i586__ -U_M_IX86 -U_M_X64 -U__MINGW32__ -U__gnu_linux__ -UFDK_ASSERT_ENABLE
  -UUSE_ARMV8_64BIT_DIVISION -UUSE_ARMV8_64BIT_SQUAREROOT -UUSE_ARMV8_64BIT_INVDIV -U_WIN32 -U_WIN64
  -U__TARGET_ARCH_7_A -U__TARGET_ARCH_7_R -U__TARGET_ARCH_7_M -U__ARM_ARCH_5TEJ__ -U__ARM_ARCH_6J__
  -U__ARM_ARCH_6ZK__ -U__ARM_ARCH_6K__ -U__ARM_ARCH_6T2__ -U__ARM_ARCH_7A__ -U__ARM_ARCH_7R__
  -U__ARM_ARCH_7_M__ -U__ARM_ARCH_7E_M__ -U_M_ARM)
DST="$REF/resolved"
rm -rf "$DST"
( cd "$C_DIR"
  find src include demo -type f \( -name '*.cpp' -o -name '*.h' \) | while read -r f; do
    mkdir -p "$DST/$(dirname "$f")"
    unifdef "${SYMS[@]}" "$f" > "$DST/$f.tmp" || [ $? -le 1 ]
    awk 'NR==1 && /^\/\*-{5,}/{skip=1} skip && /^-{5,}\*\/$/{skip=0; next} !skip' "$DST/$f.tmp" > "$DST/$f"
    rm "$DST/$f.tmp"
  done )

# Preprocessed translation units with the exact flags of the reference build.
python3 - "$BUILD" "$REF/pp" "$C_DIR/src/" <<'PY'
import json, os, shlex, subprocess, sys
cc = json.load(open(os.path.join(sys.argv[1], 'compile_commands.json')))
out, src = sys.argv[2], os.path.realpath(sys.argv[3]) + '/'
for e in cc:
    f = os.path.realpath(e['file'])
    if not f.startswith(src):
        continue
    args = shlex.split(e['command']); a = []; skip = False
    for x in args:
        if skip: skip = False; continue
        if x == '-o': skip = True; continue
        if x == '-c': continue
        a.append(x)
    rel = f[len(src):]
    dst = os.path.join(out, rel.replace('.cpp', '.ii'))
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    subprocess.run(a + ['-E', '-P', '-o', dst], cwd=e['directory'], check=True)
PY
echo "reference set up in $REF"
