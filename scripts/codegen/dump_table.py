#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Dump constant tables of the C++ reference implementation as Rust array literals.

The C++ sources define many tables with macros (FL2FXCONST_*, STC, WTC, ...). Instead of
re-implementing those macros, this tool compiles a tiny program that #includes the C++ source
file (so file-static tables are visible), and prints the raw table values with the exact
compiler semantics of the reference build.

Usage:
  scripts/codegen/dump_table.py SOURCE SYMBOL ELEMTYPE [--rust-type T] [--name NAME] [--hex] [--per-line N]
                      [--count EXPR] [--prelude CODE]

  SOURCE    path to a .cpp/.h file relative to the C++ source root (e.g. libFDK/src/FDK_tools_rom.cpp)
  SYMBOL    C expression of the table (e.g. SineTable512 or &pow2Coeff[0])
  ELEMTYPE  scalar C type used to reinterpret the table memory (e.g. SHORT, LONG, UCHAR).
            Struct tables (e.g. FIXP_SPK) are dumped as their scalar fields in memory order.
  --count   number of ELEMTYPE elements (default: sizeof(SYMBOL)/sizeof(ELEMTYPE))

Environment:
  MPEGHDEC_C_DIR   C++ source checkout (default: work/reference/src)
"""
import argparse, os, subprocess, sys, tempfile, glob

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
CDIR = os.environ.get('MPEGHDEC_C_DIR', os.path.join(WORK, 'reference', 'src'))

RUST = {'SHORT': 'i16', 'FIXP_SGL': 'i16', 'USHORT': 'u16', 'LONG': 'i32', 'INT': 'i32',
        'FIXP_DBL': 'i32', 'UINT': 'u32', 'ULONG': 'u32', 'UCHAR': 'u8', 'SCHAR': 'i8',
        'char': 'i8', 'INT64': 'i64', 'UINT64': 'u64', 'float': 'f32', 'double': 'f64'}

def include_dirs():
    dirs = [os.path.join(CDIR, 'include'), os.path.join(CDIR, 'include', 'sys')]
    dirs += sorted(glob.glob(os.path.join(CDIR, 'src', '*', 'include')))
    dirs += sorted(glob.glob(os.path.join(CDIR, 'src', '*', 'src')))
    return dirs

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source'); ap.add_argument('symbol'); ap.add_argument('elemtype')
    ap.add_argument('--rust-type'); ap.add_argument('--name'); ap.add_argument('--hex', action='store_true')
    ap.add_argument('--per-line', type=int, default=8); ap.add_argument('--count')
    ap.add_argument('--prelude', default='')
    a = ap.parse_args()
    src = os.path.join(CDIR, 'src', a.source) if not os.path.isabs(a.source) else a.source
    count = a.count or f'(sizeof({a.symbol}) / sizeof({a.elemtype}))'
    isfloat = a.elemtype in ('float', 'double')
    prog = f'''#include "{src}"
#include <stdio.h>
{a.prelude}
int main() {{
  const {a.elemtype}* p = (const {a.elemtype}*)(const void*)({ '&' if not a.symbol.startswith('&') else ''}{a.symbol.lstrip('&')});
  long n = (long)({count});
  printf("%ld\\n", n);
  for (long i = 0; i < n; i++) {{
    {'printf("%.17g\\n", (double)p[i]);' if isfloat else 'printf("%lld\\n", (long long)p[i]);'}
  }}
  return 0;
}}
'''
    with tempfile.TemporaryDirectory() as td:
        cpp = os.path.join(td, 'dump.cpp'); exe = os.path.join(td, 'dump')
        open(cpp, 'w').write(prog)
        cmd = ['c++', '-std=c++11', '-O0', '-w', '-Wl,-dead_strip', '-DMPEGHDEC_INTERNAL=1', '-DMPEGHDEC_STATIC=1']
        cmd += ['-I' + d for d in include_dirs()] + [cpp, '-o', exe]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stderr.write(r.stderr); sys.exit(1)
        out = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split()
    n = int(out[0]); vals = out[1:1 + n]
    rt = a.rust_type or RUST.get(a.elemtype, a.elemtype)
    def fmt(v):
        if isfloat: return v if ('.' in v or 'e' in v or 'n' in v) else v + '.0'
        iv = int(v)
        if a.hex:
            bits = {'i8': 8, 'u8': 8, 'i16': 16, 'u16': 16, 'i32': 32, 'u32': 32, 'i64': 64, 'u64': 64}[rt]
            h = f'0x{iv & ((1 << bits) - 1):0{bits // 4}X}'
            if rt.startswith('i') and iv < 0: return f'{h}u{bits} as {rt}'
            return h
        return str(iv)
    items = [fmt(v) for v in vals]
    lines = [', '.join(items[i:i + a.per_line]) + ',' for i in range(0, len(items), a.per_line)]
    name = a.name or a.symbol.lstrip('&').split('[')[0]
    print(f'pub static {name}: [{rt}; {n}] = [')
    for l in lines: print('    ' + l)
    print('];')

if __name__ == '__main__':
    main()
