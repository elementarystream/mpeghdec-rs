#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""End-to-end comparison of the Rust decoder with the reference decoder output.

Test cases (expected hashes per reference variant, see scripts/refsums.py):
  * the test streams in work/streams at target layout 2 (SHA256SUMS.ref-pcm[.x86|.x86_32]),
  * the vectors of scripts/make_vectors.py (work/vectors/manifest[.<arch>].json),
  * the ISO/IEC 23008-9 conformance bitstreams (scripts/conformance.py,
    work/conformance/ref.<arch>.json): exit code, output hash or absence of output and the last
    error message must match, also for bitstreams the reference cannot decode (LPD, HOA, ...).

MPEGHDEC_ARCH=arm|x86|x86_32 selects the variant the Rust decoder was built for when it is not the
host's (e.g. x86_32 for an i686 build on an x86-64 host).

The Rust CLI (`target/<profile>/mpeghdec`) decodes every case and the SHA-256 of its WAV output is
compared; on a mismatch the first differing sample is reported, using a reference WAV made with
the reference decoder.

Usage: scripts/e2e.py [--profile release] [--exe PATH] [--jobs N] [--filter REGEX] [--samples short|all|none]
                    [--vectors] [--conformance] [--keep] [--args "..."]

--args passes extra arguments to the Rust decoder (e.g. --args "-j 4" for the number of threads).
"""
import argparse, concurrent.futures as cf, hashlib, json, os, re, subprocess, sys, tempfile

sys.dont_write_bytecode = True  # no __pycache__ in scripts/
import conformance, refsums  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
STREAMS = os.environ.get('MPEGHDEC_TEST_STREAMS', os.path.join(WORK, 'streams'))
VECTORS = os.path.join(WORK, 'vectors')
REF_DEC = refsums.ref_decoder(WORK)
SHORT = ['s1_clip', 's3_dvs', 's4_dvs', 's5_short', 's1_short',
         's2_short', 's3_short', 's4_short']


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def cases(args):
    out = []
    if args.samples != 'none':
        sums = refsums.load(STREAMS, WORK)
        for mp4, h in sorted(sums.items()):
            if args.samples == 'short' and not any(mp4.startswith(s + '.') for s in SHORT):
                continue
            out.append((mp4.replace('.mp4', '') + '.tl2', os.path.join(STREAMS, mp4), ['-tl', '2'],
                        dict(exit=0, sha256=h)))
    manifest = refsums.vector_manifest(VECTORS, WORK) if args.vectors else None
    if manifest:
        for name, e in sorted(manifest.items()):
            for tl, o in sorted(e['outputs'].items(), key=lambda x: int(x[0])):
                out.append((f'{name}.tl{tl}', os.path.join(VECTORS, e['stream']), ['-tl', tl],
                            dict(exit=0, sha256=o['sha256'])))
    if args.conformance:
        confs = conformance.load_cases()
        if confs is None:
            sys.exit('no conformance cases, run scripts/conformance.py fetch and prepare')
        refs = conformance.ref_results(args.jobs)
        for name, c in sorted(confs.items()):
            a = [conformance.CONF + '/' + x if x.startswith('scripts/') else x for x in c['args']]
            out.append(('conf.' + name, os.path.join(conformance.CONF, c['stream']), a, refs[name]))
    if args.filter:
        out = [c for c in out if re.search(args.filter, c[0])]
    return out


def wav_layout(d):
    """(channels, bytes per sample, data offset) of a RIFF/RF64 WAV file, None if not parseable.

    The chunks are walked (our writer puts a JUNK/ds64 chunk before 'fmt ', so the format is not
    at the offsets of the canonical 44 byte header)."""
    if len(d) < 12 or d[:4] not in (b'RIFF', b'RF64') or d[8:12] != b'WAVE':
        return None
    pos, fmt = 12, None
    while pos + 8 <= len(d):
        cid, size = d[pos:pos + 4], int.from_bytes(d[pos + 4:pos + 8], 'little')
        if cid == b'fmt ' and pos + 24 <= len(d):
            fmt = (int.from_bytes(d[pos + 10:pos + 12], 'little'),
                   int.from_bytes(d[pos + 22:pos + 24], 'little') // 8)
        elif cid == b'data':
            if fmt is None or fmt[0] == 0 or fmt[1] == 0:
                return None
            return fmt[0], fmt[1], pos + 8
        pos += 8 + size + (size & 1)
    return None


def first_difference(a, b):
    """Returns a description of the first differing sample of two WAV files."""
    da, db = open(a, 'rb').read(), open(b, 'rb').read()
    la, lb = wav_layout(da), wav_layout(db)
    if la is None or lb is None:
        return 'not a WAV file' + ('' if la is not None else f' ({a})') + ('' if lb is not None else f' ({b})')
    if la[:2] != lb[:2]:
        return f'WAV formats differ (channels, bytes per sample: {la[:2]} vs {lb[:2]})'
    ch, bps = la[0], la[1]
    pa, pb = da[la[2]:], db[lb[2]:]
    n = min(len(pa), len(pb))
    for i in range(0, n, 4096):
        if pa[i:i + 4096] != pb[i:i + 4096]:
            for j in range(i, min(i + 4096, n)):
                if pa[j] != pb[j]:
                    s = j // bps
                    return f'first difference at sample {s // ch} (frame {s // ch // 1024}), channel {s % ch}'
    return f'outputs differ in length ({len(pa)} vs {len(pb)} bytes of samples)'


def last_error(text):
    errors = [l.strip() for l in text.splitlines() if 'Error' in l]
    return errors[-1] if errors else ''


def run_case(case, args, tmp):
    """expected: exit code, SHA-256 of the output (None: no output written) and, for a failure,
    the last error message of the reference decoder."""
    name, stream, dec_args, expected = case
    out = os.path.join(tmp, name + '.wav')
    exe = args.exe or os.path.join(REPO, 'target', args.profile, 'mpeghdec')
    r = subprocess.run([exe] + dec_args + ['-if', stream, '-of', out] + args.args.split(),
                       capture_output=True, text=True, errors='replace')
    if r.returncode != expected['exit']:
        return name, False, (f'decoder exit code {r.returncode}, reference {expected["exit"]}: '
                             f'{(r.stdout + r.stderr)[-300:]}')
    if r.returncode != 0 and last_error(r.stdout + r.stderr) != expected.get('error', ''):
        return name, False, (f'error message {last_error(r.stdout + r.stderr)!r}, '
                             f'reference {expected.get("error")!r}')
    got = sha256(out) if os.path.exists(out) else None
    if got == expected['sha256']:
        if not args.keep and got is not None:
            os.remove(out)
        return name, True, ''
    if got is None or expected['sha256'] is None:
        return name, False, 'no output written' + (' by the Rust decoder' if got is None else ' by the reference')
    ref = os.path.join(tmp, name + '.ref.wav')
    subprocess.run([REF_DEC] + dec_args + ['-if', stream, '-of', ref], capture_output=True)
    return name, False, first_difference(out, ref)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='release')
    ap.add_argument('--exe', help='decoder executable (default: target/<profile>/mpeghdec)')
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    ap.add_argument('--filter')
    ap.add_argument('--samples', default='short', choices=['short', 'all', 'none'])
    ap.add_argument('--vectors', action='store_true')
    ap.add_argument('--conformance', action='store_true', help='ISO conformance bitstreams')
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--args', default='', help='extra arguments for the Rust decoder')
    args = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix='mpeghdec-e2e-')
    todo = cases(args)
    failed = 0
    with cf.ThreadPoolExecutor(args.jobs) as ex:
        for name, ok, msg in ex.map(lambda c: run_case(c, args, tmp), todo):
            print(f'{"PASS" if ok else "FAIL"} {name} {msg}', flush=True)
            failed += not ok
    print(f'{len(todo) - failed}/{len(todo)} passed (outputs in {tmp})')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
