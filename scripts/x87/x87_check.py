#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Checks the x87 model of the 32 bit x86 variant against the reference compiler.

Compiles scripts/x87/x87_reference.cpp with the compiler and options of the 32 bit x86 reference build
(clang -m32 -O3, x87 floating point) against the libFDK headers of work/reference/src, runs it and
compares its output with the 32 bit x86 variant of the Rust code (test
`x87_reference_tests` of the mpeghdec-core crate, built with MPEGHDEC_ARCH=x86_32 into
target/x86_32).

Requirements: an x86 host with clang and the 32 bit C++ libraries (Debian/Ubuntu: g++-multilib),
scripts/setup-reference.sh.

Usage: scripts/x87/x87_check.py
"""
import os, subprocess, sys, tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
C_DIR = os.environ.get('MPEGHDEC_C_DIR', os.path.join(WORK, 'reference', 'src'))


def main():
    tmp = tempfile.mkdtemp(prefix='mpeghdec-x87-')
    exe, out = os.path.join(tmp, 'x87_reference'), os.path.join(tmp, 'x87_reference.txt')
    inc = [f'-I{os.path.join(C_DIR, d)}' for d in
           ('src/libFDK/include', 'src/libSYS/include', 'include', 'include/sys')]
    subprocess.run([os.environ.get('CXX', 'clang++'), '-m32', '-O3', '-DNDEBUG', '-w', *inc,
                    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'x87_reference.cpp'), '-o', exe], check=True)
    with open(out, 'w') as f:
        subprocess.run([exe], stdout=f, check=True)
    env = dict(os.environ, MPEGHDEC_ARCH='x86_32', MPEGHDEC_X87_REFERENCE=out,
               CARGO_TARGET_DIR=os.path.join(REPO, 'target', 'x86_32'))
    r = subprocess.run(['cargo', 'test', '--release', '-p', 'mpeghdec-core', '--lib',
                        'x87_reference_tests', '--', '--nocapture'], cwd=REPO, env=env)
    sys.exit(r.returncode)


if __name__ == '__main__':
    main()
