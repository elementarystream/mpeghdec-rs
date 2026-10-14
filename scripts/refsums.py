# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.

"""Reference decoder and expected SHA-256 hashes per architecture.

The reference output depends on the architecture it is built for (see PORTING.md). Hashes of the
test streams: work/streams/SHA256SUMS.ref-pcm (AArch64), .x86 (x86-64), .x86_32 (32 bit x86).
The Rust decoder follows its target architecture or MPEGHDEC_ARCH=arm|x86|x86_32 at build time;
set MPEGHDEC_ARCH the same way when running the scripts for a variant other than the host's.

Reference decoders: work/reference/build-<arch>/bin/mpeghDecoder, <arch> = aarch64, x86_64 or
i686 (scripts/setup-reference.sh, i686 with --i686 on an x86-64 host). Missing hash files of the
x86 variants (SHA256SUMS.ref-pcm.x86[_32]) are created with them from the arguments in
SHA256SUMS.ref-pcm. work/vectors/manifest.json holds the hashes of the reference that made the
vectors (`ref_arch` of its entries, AArch64 if missing); manifest.<arch>.json of another variant is
created the same way.
"""
import concurrent.futures as cf, hashlib, json, os, platform, subprocess, sys, tempfile

X86_64_MACHINES = ('x86_64', 'amd64')
X86_32_MACHINES = ('i386', 'i486', 'i586', 'i686', 'x86')


def host_arch():
    m = platform.machine().lower()
    if m in X86_64_MACHINES:
        return 'x86'
    if m in X86_32_MACHINES:
        return 'x86_32'
    return 'arm'


def arch():
    """The reference build that the Rust decoder reproduces: 'arm', 'x86' or 'x86_32'."""
    env = os.environ.get('MPEGHDEC_ARCH')
    if env in ('x86', 'x86_32', 'arm'):
        return env
    return host_arch()


BUILD_DIRS = {'arm': 'build-aarch64', 'x86': 'build-x86_64', 'x86_32': 'build-i686'}


def ref_decoder(work, a=None):
    """Path of the reference decoder of the variant `a` (default: arch())."""
    return os.path.join(work, 'reference', BUILD_DIRS[a or arch()], 'bin', 'mpeghDecoder')


def sums_file(samples, a=None):
    a = a or arch()
    return os.path.join(samples, 'SHA256SUMS.ref-pcm' + ('' if a == 'arm' else '.' + a))


def parse(path):
    """{mp4 name: (sha256, reference decoder arguments)}"""
    out = {}
    for line in open(path):
        p = line.split()
        if len(p) >= 2:
            out[p[1]] = (p[0], p[2:])
    return out


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _check_ref(ref_dec, a):
    if a == 'arm' or host_arch() == 'arm':
        sys.exit(f'the hashes of the {a} reference build can only be created on an x86 host')
    if not os.path.exists(ref_dec):
        sys.exit(f'reference decoder missing ({ref_dec}), run scripts/setup-reference.sh'
                 + (' --i686' if a == 'x86_32' else ''))


def _decode_all(jobs, ref_dec, items):
    """items: (key, stream, args) -> {key: sha256}"""
    tmp = tempfile.mkdtemp(prefix='mpeghdec-refsums-')

    def run(item):
        key, stream, args = item
        wav = os.path.join(tmp, f'{abs(hash(key))}.wav')
        subprocess.run([ref_dec] + args + ['-if', stream, '-of', wav],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        h = sha256(wav)
        os.remove(wav)
        return key, h

    with cf.ThreadPoolExecutor(jobs or os.cpu_count()) as ex:
        out = dict(ex.map(run, items))
    os.rmdir(tmp)
    return out


def load(samples, work, jobs=None):
    """{mp4 name: sha256} of the reference build that the Rust decoder reproduces."""
    a = arch()
    path = sums_file(samples, a)
    if not os.path.exists(path):
        ref_dec = ref_decoder(work, a)
        _check_ref(ref_dec, a)
        print(f'creating {path} with {ref_dec}', file=sys.stderr)
        entries = parse(sums_file(samples, 'arm'))
        hashes = _decode_all(jobs, ref_dec, [(mp4, os.path.join(samples, mp4), args)
                                             for mp4, (_, args) in entries.items()])
        with open(path, 'w') as f:
            f.write(''.join(f'{hashes[mp4]}  {mp4}  {" ".join(args)}\n'
                            for mp4, (_, args) in entries.items()))
    return {mp4: h for mp4, (h, _) in parse(path).items()}


def vector_manifest(vectors, work, jobs=None):
    """The manifest of the vectors (scripts/make_vectors.py) for the reproduced reference build:
    manifest.json (made with the reference of its `ref_arch`) or manifest.<arch>.json for
    another variant, which is created if missing."""
    base = os.path.join(vectors, 'manifest.json')
    if not os.path.exists(base):
        return None
    manifest = json.load(open(base))
    a = arch()
    made_with = {e.get('ref_arch', 'arm') for e in manifest.values()}
    if made_with == {a}:
        return manifest
    path = os.path.join(vectors, f'manifest.{a}.json')
    if not os.path.exists(path):
        ref_dec = ref_decoder(work, a)
        _check_ref(ref_dec, a)
        print(f'creating {path} with {ref_dec}', file=sys.stderr)
        items = [((name, tl), os.path.join(vectors, e['stream']), ['-tl', tl])
                 for name, e in manifest.items() for tl in e['outputs']]
        hashes = _decode_all(jobs, ref_dec, items)
        for name, e in manifest.items():
            e['ref_arch'] = a
            for tl, o in e['outputs'].items():
                o['sha256'] = hashes[(name, tl)]
                o.pop('log', None)
        json.dump(manifest, open(path, 'w'), indent=1, sort_keys=True)
    return json.load(open(path))
