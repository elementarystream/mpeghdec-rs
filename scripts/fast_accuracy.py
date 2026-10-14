#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Accuracy check of the fast mode (`mpeghdec -fast`, not bit-exact) against the reference.

For every case the Rust decoder decodes the stream twice, bit-exact and with `-fast`:

  * the bit-exact output must match the known reference hash or, without one, the output of the
    reference decoder (work/reference/build-<arch>) run with the same arguments;
  * the fast output must have the same format and length as the reference;
  * without bypassed frames the fast output must be bit-exact, otherwise every channel must stay
    within max |error| <= --max-lsb and RMS error <= --max-rms (LSB of 24 bit), sample by sample.

Cases: the test streams (target layout 2, and 6 with --extra), the vectors at the usual target
layouts and with --extra also at their own channel layout (the identity case), with --seek the
identity cases with a seek (`-sf/-st`), and with --bench the streams of scripts/bench.py.

Usage: scripts/fast_accuracy.py [--exe PATH] [--samples short|all|none] [--vectors] [--extra]
                              [--seek] [--bench] [--args "-j 1"] [--jobs N] [--filter REGEX]
                              [--max-lsb 1024] [--max-rms 64] [--json FILE]
Environment: MPEGHDEC_WORK_DIR (default <repo>/work)
"""
import argparse, concurrent.futures as cf, hashlib, json, os, re, subprocess, sys, tempfile

sys.dont_write_bytecode = True  # no __pycache__ in scripts/ (wavdiff, make_vectors are imported)
import refsums, wavdiff  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
STREAMS = os.environ.get('MPEGHDEC_TEST_STREAMS', os.path.join(WORK, 'streams'))
VECTORS = os.path.join(WORK, 'vectors')
REF_DEC = refsums.ref_decoder(WORK)
SHORT = ['s1_clip', 's3_dvs', 's4_dvs', 's5_short', 's1_short',
         's2_short', 's3_short', 's4_short']
BENCH = [
    ('bench.s1_30min.tl2', 'streams/s1_30min_a.mp4', 2),
    ('bench.c22_2.tl13', 'bench/c22_2_1024k/c22_2_1024k.mp4', 13),
    ('bench.c7_1_4.tl19', 'bench/c7_1_4_512k/c7_1_4_512k.mp4', 19),
    ('bench.c5_1_mct.tl6', 'bench/c5_1_mct_160k/c5_1_mct_160k.mp4', 6),
    ('bench.c22_2.tl2', 'bench/c22_2_1024k/c22_2_1024k.mp4', 2),
    ('bench.obj4.tl19', 'bench/obj4_256k/obj4_256k.mp4', 19),
]
SEEK_ARGS = '-sf 100 -st 300'


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def vector_layouts():
    """name -> CICP index of the channel layout of the vectors (scripts/make_vectors.py)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import make_vectors
    return {v[0]: v[1] for v in make_vectors.VECTORS if v[1]}


def cases(a):
    """(name, stream, target layout, extra arguments, expected SHA-256 or None)"""
    out = []
    if a.samples != 'none':
        for mp4, h in sorted(refsums.load(STREAMS, WORK).items()):
            if a.samples == 'short' and not any(mp4.startswith(s + '.') for s in SHORT):
                continue
            base = mp4.replace('.mp4', '')
            out.append((base + '.tl2', os.path.join(STREAMS, mp4), 2, '', h))
            if a.extra:
                out.append((base + '.tl6', os.path.join(STREAMS, mp4), 6, '', None))
    manifest = refsums.vector_manifest(VECTORS, WORK) if a.vectors else None
    if manifest:
        natural = vector_layouts() if (a.extra or a.seek) else {}
        for name, e in sorted(manifest.items()):
            stream = os.path.join(VECTORS, e['stream'])
            for tl, o in sorted(e['outputs'].items(), key=lambda x: int(x[0])):
                out.append((f'{name}.tl{tl}', stream, int(tl), '', o['sha256']))
            tl = natural.get(name)
            if tl is None:
                continue
            if a.extra and str(tl) not in e['outputs']:
                out.append((f'{name}.tl{tl}', stream, tl, '', None))
            if a.seek:
                out.append((f'{name}.tl{tl}.seek', stream, tl, SEEK_ARGS, None))
    if a.bench:
        for name, path, tl in BENCH:
            if os.path.exists(os.path.join(WORK, path)):
                out.append((name, os.path.join(WORK, path), tl, '', None))
        if a.seek:
            out.append(('bench.s1_30min.tl2.seek', os.path.join(WORK, BENCH[0][1]), 2,
                        '-sf 2000 -st 30000', None))
    if a.filter:
        out = [c for c in out if re.search(a.filter, c[0])]
    return out


class TooShortToSeek(Exception):
    """The stream has fewer access units than the seek target (both decoders refuse it)."""


def decode(exe, stream, tl, args, out):
    r = subprocess.run([exe, '-tl', str(tl), '-if', stream, '-of', out] + args.split(),
                       capture_output=True, text=True)
    if r.returncode != 0 and '-st ' in args and 'Too few ISOBMFF/MP4 Samples' in r.stdout:
        raise TooShortToSeek(r.stdout.strip().splitlines()[-1])
    if r.returncode != 0:
        raise RuntimeError(f'{os.path.basename(exe)} failed ({r.returncode}): {r.stdout[-300:]}')
    return r.stdout


def run_case(case, a, tmp):
    name, stream, tl, extra, expected = case
    exact, fast = os.path.join(tmp, name + '.exact.wav'), os.path.join(tmp, name + '.fast.wav')
    res = dict(name=name, ok=False, skipped=False, bypassed=0, frames=0, max_lsb=0, rms_lsb=0.0,
               snr_db=None, msg='')
    try:
        decode(a.exe, stream, tl, f'{a.args} {extra}', exact)
        if expected is None:
            if not os.path.exists(REF_DEC):
                raise RuntimeError(f'reference decoder missing ({REF_DEC})')
            ref = os.path.join(tmp, name + '.ref.wav')
            # the reference decoder prints its progress with carriage returns: discard stdout
            subprocess.run([REF_DEC, '-tl', str(tl), '-if', stream, '-of', ref] + extra.split(),
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            expected = sha256(ref)
            os.remove(ref)
        if sha256(exact) != expected:
            raise RuntimeError('bit-exact decode differs from the reference')
        log = decode(a.exe, stream, tl, f'{a.args} {extra} -fast', fast)
        m = re.search(r'Fast mode: format converter bypassed in (\d+) frames', log)
        if not m:
            raise RuntimeError('no fast mode report in the decoder output')
        res['bypassed'] = int(m.group(1))
        m = re.search(r'Decoded MPEG-H audio frames: (\d+)', log)
        res['frames'] = int(m.group(1)) if m else 0
        if open(exact, 'rb').read(80) != open(fast, 'rb').read(80):
            raise RuntimeError('WAV header (format or length) differs')
        if os.path.getsize(exact) != os.path.getsize(fast):
            raise RuntimeError('output length differs')
        if res['bypassed'] == 0:
            if sha256(fast) != expected:
                raise RuntimeError('not bypassed, but the output is not bit-exact')
            res['ok'] = True
            res['msg'] = 'bit-exact'
            return res
        d = wavdiff.compare(exact, fast)
        chans = d['channels']
        res['max_lsb'] = max(c['max_lsb'] for c in chans)
        res['rms_lsb'] = max(c['rms_lsb'] for c in chans)
        loud = [c['snr_db'] for c in chans if c['ref_rms_lsb'] > 2 ** 16]  # above -42 dBFS rms
        res['snr_db'] = min(loud) if loud else None
        if res['max_lsb'] > a.max_lsb or res['rms_lsb'] > a.max_rms:
            raise RuntimeError(f'outside the tolerance (max {res["max_lsb"]} LSB, '
                               f'rms {res["rms_lsb"]:.1f} LSB)')
        res['ok'] = True
    except TooShortToSeek as e:
        res['ok'] = res['skipped'] = True
        res['msg'] = f'stream too short for the seek ({e})'
    except (RuntimeError, ValueError, OSError) as e:
        res['msg'] = str(e)
    finally:
        if not a.keep:
            for f in (exact, fast):
                if os.path.exists(f):
                    os.remove(f)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exe', default=os.path.join(REPO, 'target', 'release', 'mpeghdec'))
    ap.add_argument('--samples', default='short', choices=['short', 'all', 'none'])
    ap.add_argument('--vectors', action='store_true')
    ap.add_argument('--extra', action='store_true', help='identity layouts without known hashes')
    ap.add_argument('--seek', action='store_true', help='identity cases with a seek')
    ap.add_argument('--bench', action='store_true', help='the streams of scripts/bench.py')
    ap.add_argument('--args', default='', help='extra arguments for the Rust decoder')
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument('--filter')
    ap.add_argument('--max-lsb', type=int, default=1024, help='max |error| [LSB of 24 bit]')
    ap.add_argument('--max-rms', type=float, default=64, help='max RMS error [LSB of 24 bit]')
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--json', help='write the results to this file')
    a = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix='mpeghdec-fast-')
    todo = cases(a)
    results = []
    print(f'{"case":36} {"bypassed":>12} {"max[LSB24]":>10} {"rms[LSB24]":>10} {"SNR[dB]":>8}  result')
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        for r in ex.map(lambda c: run_case(c, a, tmp), todo):
            results.append(r)
            snr = f'{r["snr_db"]:8.1f}' if r['snr_db'] is not None else f'{"-":>8}'
            byp = f'{r["bypassed"]}/{r["frames"]}'
            print(f'{r["name"]:36} {byp:>12} {r["max_lsb"]:10d} {r["rms_lsb"]:10.2f} {snr}  '
                  f'{"SKIP" if r["skipped"] else "PASS" if r["ok"] else "FAIL"} {r["msg"]}', flush=True)
    failed = sum(not r['ok'] for r in results)
    byp = [r for r in results if r['bypassed'] and r['ok']]
    skipped = sum(r['skipped'] for r in results)
    print(f'{len(results) - failed - skipped}/{len(results)} passed'
          + (f', {skipped} skipped' if skipped else '') + f'; {len(byp)} cases bypassed'
          + (f' (worst: max {max(r["max_lsb"] for r in byp)} LSB, '
             f'rms {max(r["rms_lsb"] for r in byp):.2f} LSB of 24 bit)' if byp else ''))
    if a.json:
        json.dump(results, open(a.json, 'w'), indent=1)
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
