#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Decoding speed benchmark of the Rust decoder against the reference decoder.

Every case is decoded by both decoders; the wall-clock times (best of N runs) and whether the
outputs are identical are reported.

Cases: the 30 minute test stream s1_30min (stereo) and the 60 s multichannel/object
vectors in work/bench (create them with
`scripts/make_vectors.py --out work/bench --seconds 60 --only c22_2_1024k c7_1_4_512k obj4_256k c5_1_mct_160k`).

Usage: scripts/bench.py [--exe PATH] [--ref PATH] [--runs N] [--no-ref] [--filter REGEX] [--args "..."]
Environment: MPEGHDEC_WORK_DIR (default <repo>/work), MPEGHDEC_ARCH (the variant of the Rust
decoder, selects the default reference decoder, see scripts/refsums.py)
"""
import argparse, os, re, subprocess, sys, tempfile, time, filecmp

sys.dont_write_bytecode = True  # no __pycache__ in scripts/
import refsums  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
CASES = [
    ('s1_30min.tl2', 'streams/s1_30min_a.mp4', 2),
    ('c22_2.tl2', 'bench/c22_2_1024k/c22_2_1024k.mp4', 2),
    ('c22_2.tl13', 'bench/c22_2_1024k/c22_2_1024k.mp4', 13),
    ('c7_1_4.tl19', 'bench/c7_1_4_512k/c7_1_4_512k.mp4', 19),
    ('c7_1_4.tl2', 'bench/c7_1_4_512k/c7_1_4_512k.mp4', 2),
    ('obj4.tl19', 'bench/obj4_256k/obj4_256k.mp4', 19),
    ('c5_1_mct.tl6', 'bench/c5_1_mct_160k/c5_1_mct_160k.mp4', 6),
]


def timed(cmd, runs):
    best = None
    for _ in range(runs):
        t = time.perf_counter()
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        d = time.perf_counter() - t
        best = d if best is None else min(best, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exe', default=os.path.join(REPO, 'target', 'release', 'mpeghdec'))
    ap.add_argument('--ref', default=refsums.ref_decoder(WORK),
                    help='reference decoder (default: the one of the variant, see scripts/refsums.py)')
    ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--no-ref', action='store_true')
    ap.add_argument('--filter')
    ap.add_argument('--args', default='', help='extra arguments for the Rust decoder')
    a = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix='mpeghdec-bench-')
    print(f'{"case":14} {"ref [s]":>8} {"rust [s]":>9} {"speedup":>8}  output')
    for name, path, tl in CASES:
        if a.filter and not re.search(a.filter, name):
            continue
        src = os.path.join(WORK, path)
        if not os.path.exists(src):
            print(f'{name:14} (missing {path})')
            continue
        out_r, out_c = os.path.join(tmp, 'r.wav'), os.path.join(tmp, 'c.wav')
        tr = timed([a.exe, '-tl', str(tl), '-if', src, '-of', out_r] + a.args.split(), a.runs)
        if a.no_ref:
            print(f'{name:14} {"-":>8} {tr:9.3f}')
            continue
        tc = timed([a.ref, '-tl', str(tl), '-if', src, '-of', out_c], a.runs)
        same = filecmp.cmp(out_r, out_c, shallow=False)
        print(f'{name:14} {tc:8.3f} {tr:9.3f} {tc / tr:7.2f}x  {"identical" if same else "DIFFERENT"}')


if __name__ == '__main__':
    main()
