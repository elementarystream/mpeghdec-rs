#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Determinism stress test of the multi-core decoding.

Decodes the same streams many times with different thread counts, several decoders at once (so
that the machine is oversubscribed and the worker threads are preempted at random points), and
checks that every output is identical to the single-threaded output.

Usage: scripts/stress_threads.py [--exe PATH] [--rounds N] [--parallel N] [--threads 2,3,4,8,16]
                               [--streams stream.mp4:tl ...]
Environment: MPEGHDEC_WORK_DIR (default <repo>/work)
"""
import argparse, concurrent.futures as cf, hashlib, itertools, os, random, subprocess, sys, tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
DEFAULT_STREAMS = [
    'vectors/c22_2_1024k/c22_2_1024k.mp4:13',
    'vectors/c7_1_4_mct_320k/c7_1_4_mct_320k.mp4:2',
    'vectors/c5_1_all_tools_128k/c5_1_all_tools_128k.mp4:6',
    'vectors/stereo_cplx_96k/stereo_cplx_96k.mp4:2',
    'vectors/obj4_256k/obj4_256k.mp4:19',
    'streams/s4_short_b.mp4:2',
]


def decode(exe, stream, tl, threads, out):
    r = subprocess.run([exe, '-tl', str(tl), '-if', stream, '-of', out, '-j', str(threads)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode != 0:
        return f'decoder failed ({r.returncode})'
    h = hashlib.sha256(open(out, 'rb').read()).hexdigest()
    os.remove(out)
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exe', default=os.path.join(REPO, 'target', 'release', 'mpeghdec'))
    ap.add_argument('--rounds', type=int, default=4)
    ap.add_argument('--parallel', type=int, default=(os.cpu_count() or 4))
    ap.add_argument('--threads', default='2,3,4,8,16')
    ap.add_argument('--streams', nargs='*', default=DEFAULT_STREAMS)
    a = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix='mpeghdec-stress-')
    streams = []
    for s in a.streams:
        path, tl = s.rsplit(':', 1)
        path = path if os.path.isabs(path) else os.path.join(WORK, path)
        if os.path.exists(path):
            streams.append((path, int(tl)))
        else:
            print(f'skipping missing {path}')
    ref = {}
    for i, (path, tl) in enumerate(streams):
        ref[(path, tl)] = decode(a.exe, path, tl, 1, os.path.join(tmp, f'ref{i}.wav'))
    jobs = [(p, tl, j) for (p, tl), j in itertools.product(streams, [int(x) for x in a.threads.split(',')])]
    jobs = jobs * a.rounds
    random.shuffle(jobs)
    failed = 0
    with cf.ThreadPoolExecutor(a.parallel) as ex:
        futs = {ex.submit(decode, a.exe, p, tl, j, os.path.join(tmp, f'{n}.wav')): (p, tl, j)
                for n, (p, tl, j) in enumerate(jobs)}
        for f in cf.as_completed(futs):
            p, tl, j = futs[f]
            if f.result() != ref[(p, tl)]:
                failed += 1
                print(f'MISMATCH {os.path.basename(p)} -tl {tl} -j {j}', flush=True)
    print(f'{len(jobs) - failed}/{len(jobs)} decodes identical to -j 1')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
