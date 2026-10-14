#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""ISO/IEC 23008-9 conformance bitstreams of MPEG-H 3D Audio (docs/research.md#conformance-bitstreams).

Subcommands:

  fetch [--waveforms]   Downloads the bitstreams (with --waveforms also the ISO reference waveforms
                        of the applicable cases) into work/conformance/ by HTTP range requests on the
                        stored inner zip. Checks each CRC-32 and records the SHA-256 in
                        work/conformance/SHA256SUMS; work/conformance/INDEX lists the archive.
  prepare               Wraps the bitstreams into MP4 (work/conformance/mp4/; the reference decoder
                        reads MP4 only) and writes the test cases to work/conformance/cases.json.
  refs                  Records exit code, output hash and error message of the reference decoder
                        in work/conformance/ref.<arch>.json (scripts/e2e.py --conformance does
                        this when the file is missing).
  iso                   Compares the reference decoder output with the ISO reference waveforms
                        (criteria: iso_verdict()) and writes work/conformance/iso-report.json.

The ISO licence does not allow redistribution, so the files stay in the git-ignored work/.

Usage: scripts/conformance.py fetch [--waveforms] [--jobs N]
       scripts/conformance.py prepare
       scripts/conformance.py refs [-v]
       scripts/conformance.py iso [--exe PATH] [--all]

Requirements: numpy (iso), openpyxl (the criteria of the coverage table in cases.json).
"""
import argparse, concurrent.futures as cf, hashlib, json, os, struct, subprocess, sys, tempfile, urllib.request, zlib

sys.dont_write_bytecode = True  # no __pycache__ in scripts/
import refsums  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
CONF = os.path.join(WORK, 'conformance')
BASE_URL = 'https://standards.iso.org/iso-iec/23008/-9/ed-2/en/'
ARCHIVE_URL = BASE_URL + 'MPEG-H_3D_audio_conformanceData.zip'
TABLES_URL = BASE_URL + 'ConformanceTables.xlsx'
INNER = 'MPEG-H_3D_Audio_ConformanceData/MPEG-H_3D_Audio_ConformanceData.zip'
SUMS = os.path.join(CONF, 'SHA256SUMS')
CASES = os.path.join(CONF, 'cases.json')
INDEX = os.path.join(CONF, 'INDEX')  # CRC-32, size and name of the files of the archive


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


# --- bitstream names ---------------------------------------------------------------------------

def core_mode(stream):
    """'FD', 'LPD' or 'Cct' (FD/LPD switching) from a name <ElemConf>_<SampFreqIdx>_<CoreMode>..."""
    return stream.split('_')[2]


def applicable(stream):
    """Whether the bitstream uses only tools of the reference decoder (FD core, no HOA)."""
    return not stream.startswith('H') and core_mode(stream) == 'FD'


# --- archive access by range requests ----------------------------------------------------------

def fetch_range(url, start, end):
    """Bytes [start, end) of `url`."""
    req = urllib.request.Request(url, headers={'Range': f'bytes={start}-{end - 1}'})
    for attempt in range(5):
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                data = r.read()
            if len(data) == end - start:
                return data
        except OSError as e:
            if attempt == 4:
                raise
            print(f'retrying range {start}-{end} of {url}: {e}', file=sys.stderr)
    raise IOError(f'short read of range {start}-{end} of {url}')


def content_length(url):
    req = urllib.request.Request(url, method='HEAD')
    with urllib.request.urlopen(req, timeout=60) as r:
        return int(r.headers['Content-Length'])


def central_directory(url, base, size):
    """{name: (method, crc32, compressed size, size, local header offset)} of the zip archive at
    [base, base + size) of `url` (no zip64, no archive comment longer than 64 KB)."""
    tail = fetch_range(url, base + size - min(size, 65536 + 22), base + size)
    i = tail.rfind(b'PK\x05\x06')
    if i < 0:
        raise IOError('end of central directory not found')
    count, cd_size, cd_off = struct.unpack('<HII', tail[i + 10:i + 20])
    cd = fetch_range(url, base + cd_off, base + cd_off + cd_size)
    out, p = {}, 0
    for _ in range(count):
        (sig, _, _, _, method, _, _, crc, csize, usize, nlen, elen, clen, _, _, _,
         off) = struct.unpack('<IHHHHHHIIIHHHHHII', cd[p:p + 46])
        if sig != 0x02014b50:
            raise IOError('bad central directory entry')
        name = cd[p + 46:p + 46 + nlen].decode()
        out[name] = (method, crc, csize, usize, off)
        p += 46 + nlen + elen + clen
    return out


def data_offset(url, base, local_header_off):
    """Offset of the data of the entry whose local header is at `local_header_off`."""
    h = fetch_range(url, base + local_header_off, base + local_header_off + 30)
    nlen, elen = struct.unpack('<HH', h[26:30])
    return base + local_header_off + 30 + nlen + elen


class Archive:
    """The inner archive of the conformance data, read by range requests."""

    def __init__(self):
        outer = central_directory(ARCHIVE_URL, 0, content_length(ARCHIVE_URL))
        method, _, csize, _, off = outer[INNER]
        if method != 0:
            raise IOError('the inner archive is not stored, range access impossible')
        self.base = data_offset(ARCHIVE_URL, 0, off)
        self.entries = central_directory(ARCHIVE_URL, self.base, csize)

    def names(self, folder, ext):
        return sorted(n[len(folder) + 1:-len(ext)] for n in self.entries
                      if n.startswith(folder + '/') and n.endswith(ext))

    def extract(self, name, dst):
        method, crc, csize, usize, off = self.entries[name]
        start = data_offset(ARCHIVE_URL, self.base, off)
        data = fetch_range(ARCHIVE_URL, start, start + csize)
        if method == 8:
            data = zlib.decompress(data, -15)
        elif method != 0:
            raise IOError(f'{name}: compression method {method}')
        if len(data) != usize or zlib.crc32(data) != crc:
            raise IOError(f'{name}: CRC or size mismatch')
        tmp = dst + '.part'
        with open(tmp, 'wb') as f:
            f.write(data)
        os.replace(tmp, dst)


# --- fetch -------------------------------------------------------------------------------------

def read_sums():
    if not os.path.exists(SUMS):
        return {}
    return {p[1]: p[0] for p in (l.split() for l in open(SUMS)) if len(p) == 2}


def write_sums(sums):
    with open(SUMS, 'w') as f:
        f.write(''.join(f'{h}  {rel}\n' for rel, h in sorted(sums.items())))


def cmd_fetch(args):
    os.makedirs(os.path.join(CONF, 'compressedMhas'), exist_ok=True)
    os.makedirs(os.path.join(CONF, 'referencesWav'), exist_ok=True)
    sums = read_sums()
    print('reading the archive directory', flush=True)
    arc = Archive()
    with open(INDEX, 'w') as f:
        f.write(''.join(f'{crc:08x} {usize:10d}  {n}\n' for n, (_, crc, _, usize, _) in
                        sorted(arc.entries.items()) if not n.endswith('/')))
    streams = arc.names('compressedMhas', '.mhas')
    todo = [f'compressedMhas/{s}.mhas' for s in streams]
    if args.waveforms:
        todo += [f'referencesWav/{w}.wav' for w in arc.names('referencesWav', '.wav')
                 if applicable(w.split('__')[0])]
    tables = 'ConformanceTables.xlsx'

    def get(rel):
        dst = os.path.join(CONF, rel)
        if rel in sums and os.path.exists(dst):
            if sha256(dst) != sums[rel]:
                raise IOError(f'{rel}: SHA-256 differs from SHA256SUMS, delete the file to fetch it again')
            return rel, sums[rel], False
        if rel == tables:
            data = urllib.request.urlopen(TABLES_URL, timeout=60).read()
            with open(dst, 'wb') as f:
                f.write(data)
        else:
            arc.extract(rel, dst)
        return rel, sha256(dst), True

    fetched = 0
    with cf.ThreadPoolExecutor(args.jobs) as ex:
        for rel, h, new in ex.map(get, [tables] + todo):
            sums[rel] = h
            fetched += new
            if new:
                print(f'fetched {rel}', flush=True)
    write_sums(sums)
    n_app = sum(applicable(s) for s in streams)
    print(f'{fetched} files fetched, {len(todo) + 1 - fetched} present; {len(streams)} bitstreams '
          f'({n_app} applicable, {len(streams) - n_app} negative tests) in {CONF}')


# --- MHAS streams ------------------------------------------------------------------------------

PACTYP_MPEGH3DACFG, PACTYP_MPEGH3DAFRAME = 1, 2

# usacSamplingFrequencyIndex (ISO/IEC 23003-3, table 69); 31: explicit 24 bit value
SAMPLING_FREQUENCIES = {0: 96000, 1: 88200, 2: 64000, 3: 48000, 4: 44100, 5: 32000, 6: 24000,
                        7: 22050, 8: 16000, 9: 12000, 10: 11025, 11: 8000, 12: 7350, 15: 57600,
                        16: 51200, 17: 40000, 18: 38400, 19: 34150, 20: 28800, 21: 25600,
                        22: 20000, 23: 19200, 24: 17075, 25: 14400, 26: 12800, 27: 9600}
# coreSbrFrameLengthIndex -> output frame length
FRAME_LENGTHS = {0: 768, 1: 1024, 2: 2048, 3: 2048, 4: 4096}


class Bits:
    def __init__(self, d, p=0):
        self.d, self.p = d, p

    def u(self, n):
        v = 0
        for _ in range(n):
            v = (v << 1) | ((self.d[self.p >> 3] >> (7 - (self.p & 7))) & 1)
            self.p += 1
        return v

    def esc(self, n1, n2, n3):
        v = self.u(n1)
        if v == (1 << n1) - 1:
            e = self.u(n2)
            v += e
            if e == (1 << n2) - 1:
                v += self.u(n3)
        return v


def packets(d):
    """(type, payload offset, payload length) of the MHAS packets of `d`; stops at a truncated
    packet."""
    off = 0
    while off < len(d):
        try:
            b = Bits(d, off * 8)
            t, _, n = b.esc(3, 8, 8), b.esc(2, 8, 32), b.esc(11, 24, 24)
        except IndexError:
            return
        body = (b.p + 7) // 8
        if body + n > len(d):
            return
        yield t, body, n
        off = body + n


def access_units(d):
    """(offset, size) of the access units: the packets up to and including the next frame packet
    (like the MHAS splitter of the Rust CLI, crates/cli/src/mhas.rs)."""
    aus, start, end = [], 0, 0
    for t, body, n in packets(d):
        end = body + n
        if t == PACTYP_MPEGH3DAFRAME:
            aus.append((start, end - start))
            start = end
    if start < len(d):
        aus.append((start, len(d) - start))
    return aus


def stream_config(d):
    """Sampling rate, frame length and reference layout (CICP index, or None for another
    speakerLayoutType) of the first mpegh3daConfig of the stream."""
    for t, body, _ in packets(d):
        if t == PACTYP_MPEGH3DACFG:
            b = Bits(d, body * 8)
            profile_level = b.u(8)
            sf_idx = b.u(5)
            rate = b.u(24) if sf_idx == 31 else SAMPLING_FREQUENCIES[sf_idx]
            frame_length = FRAME_LENGTHS[b.u(3)]
            b.u(1), b.u(1)  # cfg_reserved, receiverDelayCompensation
            layout_type = b.u(2)
            cicp = b.u(6) if layout_type == 0 else None
            return dict(profile_level=profile_level, rate=rate, frame_length=frame_length,
                        layout_type=layout_type, cicp=cicp)
    raise ValueError('no mpegh3daConfig packet')


def box(t, *parts):
    body = b''.join(parts)
    return struct.pack('>I', 8 + len(body)) + t + body


def full_box(t, version, flags, *parts):
    return box(t, struct.pack('>I', (version << 24) | flags), *parts)


def mhas_to_mp4(src, dst):
    """Wraps a raw MHAS stream into a plain mhm1 MP4 for the reference decoder, which reads MP4
    only. The access units are timed at the sampling rate of the stream, one frame length apart:
    the decoder compares the timestamp difference with the access unit length and inserts or
    removes samples when they disagree. Access units with a config packet are sync samples."""
    data = open(src, 'rb').read()
    cfg = stream_config(data)
    aus = access_units(data)
    n, delta, ts = len(aus), cfg['frame_length'], cfg['rate']
    dur = n * delta
    sync = [i + 1 for i, (o, s) in enumerate(aus)
            if any(t == PACTYP_MPEGH3DACFG for t, _, _ in packets(data[o:o + s]))]
    ident = struct.pack('>9I', 0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000)
    mhm1 = box(b'mhm1', b'\0' * 6, struct.pack('>H', 1), b'\0' * 8,
               struct.pack('>HHHHI', 2, 16, 0, 0, (ts & 0xffff) << 16))

    def moov(mdat_off):
        stbl = box(b'stbl',
                   full_box(b'stsd', 0, 0, struct.pack('>I', 1), mhm1),
                   full_box(b'stts', 0, 0, struct.pack('>III', 1, n, delta)),
                   full_box(b'stss', 0, 0, struct.pack('>I', len(sync)),
                            *[struct.pack('>I', i) for i in sync]),
                   full_box(b'stsc', 0, 0, struct.pack('>IIII', 1, 1, n, 1)),
                   full_box(b'stsz', 0, 0, struct.pack('>II', 0, n),
                            *[struct.pack('>I', s) for _, s in aus]),
                   full_box(b'stco', 0, 0, struct.pack('>II', 1, mdat_off)))
        minf = box(b'minf', full_box(b'smhd', 0, 0, b'\0' * 4),
                   box(b'dinf', full_box(b'dref', 0, 0, struct.pack('>I', 1), full_box(b'url ', 0, 1))),
                   stbl)
        mdia = box(b'mdia',
                   full_box(b'mdhd', 0, 0, struct.pack('>IIIIHH', 0, 0, ts, dur, 0x55c4, 0)),
                   full_box(b'hdlr', 0, 0, b'\0' * 4, b'soun', b'\0' * 12, b'SoundHandler\0'), minf)
        trak = box(b'trak', full_box(b'tkhd', 0, 3, struct.pack('>IIIII', 0, 0, 1, 0, dur), b'\0' * 8,
                                     struct.pack('>HHHH', 0, 0, 0x0100, 0), ident,
                                     struct.pack('>II', 0, 0)), mdia)
        mvhd = full_box(b'mvhd', 0, 0, struct.pack('>IIII', 0, 0, ts, dur),
                        struct.pack('>IH', 0x10000, 0x0100), b'\0' * 10, ident, b'\0' * 24,
                        struct.pack('>I', 2))
        return box(b'moov', mvhd, trak)

    ftyp = box(b'ftyp', b'mp42', struct.pack('>I', 0), b'mp42isom')
    head = len(ftyp) + len(moov(0))
    payload = b''.join(data[o:o + s] for o, s in aus)
    tmp = dst + '.part'
    with open(tmp, 'wb') as f:
        f.write(ftyp + moov(head + 8) + struct.pack('>I', 8 + len(payload)) + b'mdat' + payload)
    os.replace(tmp, dst)
    return cfg


# --- test cases --------------------------------------------------------------------------------

def preset_script(preset):
    """UI manager interactivity script selecting a group preset (without UUID check)."""
    return ('<ActionEvent uuid="00000000-0000-0000-0000-000000000000" version="11.0" '
            f'actionType="95" paramInt="{preset}" />\n')


def decoder_args(settings, cfg):
    """Decoder options of a decoder-setting suffix (e.g. 'Lay-2_Lou-24_Eff-6'), see the note.

    Without Lay the target layout is the reference layout of the stream (layout 6, the decoder
    default, if it is not given as CICP index). Lou-x is the target loudness -x LKFS (-rl 4x),
    Eff-x the DRC effect type request. Pr-x needs a preset selection script, made by the caller.
    Cpo-x (a conformance point inside the decoder) has no equivalent: the final output is used."""
    s = dict(p.split('-', 1) for p in settings.split('_')) if settings else {}
    args = ['-tl', s.get('Lay', str(cfg['cicp'] or 6))]
    if 'Lou' in s:
        args += ['-rl', str(4 * int(s['Lou']))]  # the suffix value is the negated loudness
    if 'Eff' in s:
        args += ['-dse', s['Eff']]
    return args, s.get('Pr')


def cmd_prepare(args):
    """Wraps the bitstreams into MP4 files and writes the test cases to cases.json."""
    mhas_dir = os.path.join(CONF, 'compressedMhas')
    mp4_dir = os.path.join(CONF, 'mp4')
    script_dir = os.path.join(CONF, 'scripts')
    os.makedirs(mp4_dir, exist_ok=True)
    os.makedirs(script_dir, exist_ok=True)
    streams = sorted(f[:-5] for f in os.listdir(mhas_dir) if f.endswith('.mhas'))
    if not streams:
        sys.exit(f'no bitstreams in {mhas_dir}, run scripts/conformance.py fetch')
    # The decoder settings of the ISO reference waveforms (also without the waveforms fetched).
    waves = {}
    for line in open(INDEX):
        rel = line.split()[2]
        if rel.startswith('referencesWav/'):
            w = rel[len('referencesWav/'):-4]
            waves.setdefault(w.split('__')[0], []).append(w.split('__')[1] if '__' in w else '')
    criteria = read_criteria()
    cases = {}
    for s in streams:
        cfg = mhas_to_mp4(os.path.join(mhas_dir, s + '.mhas'), os.path.join(mp4_dir, s + '.mp4'))
        stream_cases = {}
        for st in sorted(set(waves.get(s, [])) | {''}):
            a, preset = decoder_args(st, cfg)
            if preset is not None:
                script = os.path.join('scripts', f'preset-{preset}.xml')
                open(os.path.join(CONF, script), 'w').write(preset_script(preset))
                a += ['-script', script]
            name = s + ('__' + st if st else '')
            has_wav = st in waves.get(s, [])
            stream_cases[name] = dict(stream=f'mp4/{s}.mp4', args=a, applicable=applicable(s),
                                      iso_wav=f'referencesWav/{name}.wav' if has_wav else None,
                                      criteria=criteria.get(name) if has_wav else None, config=cfg)
        # the default settings only if no reference waveform case decodes with the same options
        if s not in waves.get(s, []) and any(c['args'] == stream_cases[s]['args']
                                             for n, c in stream_cases.items() if n != s):
            del stream_cases[s]
        cases.update(stream_cases)
    json.dump(cases, open(CASES, 'w'), indent=1, sort_keys=True)
    n_app = sum(c['applicable'] for c in cases.values())
    print(f'{len(cases)} cases ({n_app} applicable, {len(cases) - n_app} negative) in {CASES}')


def read_criteria():
    """{case: criteria} from the coverage table: the conformance criteria column holds
    '<K> bit / <K> bit' (RMS and maximum difference at K bit resolution) or
    '<x> db / <K> bit / <K> bit' (additionally a segmental SNR of x dB); {} without openpyxl."""
    try:
        import openpyxl
    except ImportError:
        print('openpyxl missing: no conformance criteria in cases.json', file=sys.stderr)
        return {}
    out = {}
    wb = openpyxl.load_workbook(os.path.join(CONF, 'ConformanceTables.xlsx'), read_only=True, data_only=True)
    for ws in wb.worksheets:
        col = None
        for row in ws.iter_rows(values_only=True):
            if col is None:
                if 'Sequence Base Name' in row and 'Conformance criteria' in row:
                    col = (row.index('Sequence Base Name'), row.index('Conformance criteria'))
                continue
            name, text = row[col[0]], row[col[1]]
            if not name or not text:
                continue
            parts = [p.strip().lower() for p in str(text).split('/')]
            if not all(p.endswith(('bit', 'db')) for p in parts):
                out[name] = dict(text=' '.join(str(text).split()))
                continue
            bits = [int(p.split()[0]) for p in parts if p.endswith('bit')]
            db = [float(p.split()[0]) for p in parts if p.endswith('db')]
            out[name] = dict(text=str(text), rms_bits=bits[0], max_bits=bits[1],
                             segsnr_db=db[0] if db else None)
    return out


def run_decoder(exe, case, out, extra=()):
    """Decodes a case; returns (exit code, SHA-256 of the output or None, last 'Error' line)."""
    if os.path.exists(out):
        os.remove(out)
    a = [x if not x.startswith('scripts/') else os.path.join(CONF, x) for x in case['args']]
    r = subprocess.run([exe] + a + ['-if', os.path.join(CONF, case['stream']), '-of', out] + list(extra),
                       capture_output=True, text=True, errors='replace')
    errors = [l.strip() for l in (r.stdout + r.stderr).splitlines() if 'Error' in l]
    h = sha256(out) if os.path.exists(out) else None
    return r.returncode, h, errors[-1] if errors else ''


def ref_results_path(a=None):
    return os.path.join(CONF, f'ref.{a or refsums.arch()}.json')


def load_cases():
    if not os.path.exists(CASES):
        return None
    return json.load(open(CASES))


def ref_results(jobs=None, a=None):
    """{case: {exit, sha256, error}} of the reference decoder of the variant `a` (default: the one
    the Rust decoder reproduces); created with that reference decoder if missing."""
    a = a or refsums.arch()
    path = ref_results_path(a)
    cases = load_cases()
    if cases is None:
        return None
    old = json.load(open(path)) if os.path.exists(path) else {}
    old = {n: r for n, r in old.items() if n in cases and r.get('args') == cases[n]['args']}
    todo = [n for n in cases if n not in old]
    if todo:
        ref_dec = refsums.ref_decoder(WORK, a)
        if a != refsums.host_arch() and not (a == 'x86_32' and refsums.host_arch() == 'x86'):
            sys.exit(f'the results of the {a} reference build can only be created on that host')
        if not os.path.exists(ref_dec):
            sys.exit(f'reference decoder missing ({ref_dec}), run scripts/setup-reference.sh')
        print(f'decoding {len(todo)} conformance cases with {ref_dec}', file=sys.stderr)
        tmp = tempfile.mkdtemp(prefix='mpeghdec-conf-')

        def run(n):
            out = os.path.join(tmp, n + '.wav')
            code, h, err = run_decoder(ref_dec, cases[n], out)
            if os.path.exists(out):
                os.remove(out)
            return n, dict(exit=code, sha256=h, error=err, args=cases[n]['args'])

        with cf.ThreadPoolExecutor(jobs or os.cpu_count()) as ex:
            old.update(ex.map(run, todo))
        os.rmdir(tmp)
        json.dump(old, open(path, 'w'), indent=1, sort_keys=True)
    return old


def cmd_refs(args):
    res = ref_results(args.jobs)
    ok = sum(r['exit'] == 0 for r in res.values())
    print(f'{len(res)} cases: {ok} decoded, {len(res) - ok} failed; {ref_results_path()}')
    for n, r in sorted(res.items()):
        if args.verbose or (r['exit'] != 0) == load_cases()[n]['applicable']:
            print(f'  {n}: exit {r["exit"]}, output {"yes" if r["sha256"] else "no"} {r["error"]}')


# --- comparison with the ISO reference waveforms -----------------------------------------------

def read_wav(path):
    """(samples as float array [frames, channels] in [-1, 1), sample rate) of a 16/24/32 bit PCM
    WAV file."""
    import numpy as np
    d = open(path, 'rb').read()
    pos, fmt = 12, None
    while pos + 8 <= len(d):
        cid, size = d[pos:pos + 4], int.from_bytes(d[pos + 4:pos + 8], 'little')
        if cid == b'fmt ':
            _, ch, rate, _, _, bits = struct.unpack('<HHIIHH', d[pos + 8:pos + 24])
            fmt = ch, rate, bits
        elif cid == b'data':
            ch, rate, bits = fmt
            raw = np.frombuffer(d, np.uint8, (min(size, len(d) - pos - 8) // (ch * bits // 8)) * ch * bits // 8, pos + 8)
            if bits == 24:
                b = raw.reshape(-1, 3).astype(np.int32)
                x = (b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)) << 8 >> 8  # sign extension
            else:
                x = raw.view({16: '<i2', 32: '<i4'}[bits]).astype(np.int64)
            return x.reshape(-1, ch) / float(1 << (bits - 1)), rate
        pos += 8 + size + (size & 1)
    raise ValueError(f'{path}: no PCM data')


SEGMENT = 1024  # samples per channel of a segment of the segmental SNR


def compare_iso(test, ref):
    """Differences of the decoder output `test` from the ISO waveform `ref` (full scale 1):
    RMS and maximum of the difference over all channels (max_tail: without the first SEGMENT
    samples, where the largest differences occur), and the segmental SNR (mean over the
    segments of SEGMENT samples of each channel whose reference is not silent of the segment SNR
    in dB, capped at 100 dB)."""
    import numpy as np
    d = test - ref
    rms = float(np.sqrt(np.mean(d * d)))
    mx = float(np.max(np.abs(d))) if d.size else 0.0
    mx_tail = float(np.max(np.abs(d[SEGMENT:]))) if len(d) > SEGMENT else 0.0
    n = len(ref) // SEGMENT * SEGMENT
    es = (ref[:n] ** 2).reshape(-1, SEGMENT, ref.shape[1]).sum(1)
    ed = (d[:n] ** 2).reshape(-1, SEGMENT, ref.shape[1]).sum(1)
    sel = es > 0
    snr = np.minimum(100.0, 10 * np.log10(es[sel] / np.maximum(ed[sel], 1e-30)))
    return dict(rms=rms, max=mx, max_tail=mx_tail, segsnr=float(snr.mean()) if snr.size else None,
                segsnr_min=float(snr.min()) if snr.size else None)


def iso_verdict(m, crit, max_key='max'):
    """Pass/fail by the criteria of the coverage table, with the RMS/LSB definition of the MPEG
    audio conformance specifications (ISO/IEC 11172-4 for K = 16): RMS of the difference below
    2^-(K-1)/sqrt(12) and maximum absolute difference at most 2^-(K-2) (2 LSB at K bit); plus the
    segmental SNR where specified."""
    if not crit or 'rms_bits' not in crit:
        return None
    ok = m['rms'] < 2.0 ** -(crit['rms_bits'] - 1) / 12 ** 0.5 and m[max_key] <= 2.0 ** -(crit['max_bits'] - 2)
    if crit['segsnr_db'] is not None:
        ok = ok and m['segsnr'] is not None and m['segsnr'] >= crit['segsnr_db']
    return ok


def cmd_iso(args):
    """Compares the reference decoder output with the ISO reference waveforms."""
    cases = load_cases()
    todo = [n for n, c in sorted(cases.items())
            if c['iso_wav'] and os.path.exists(os.path.join(CONF, c['iso_wav']))
            and (args.all or c['applicable'])]
    ref_dec = refsums.ref_decoder(WORK)
    tmp = tempfile.mkdtemp(prefix='mpeghdec-iso-')

    def run(n):
        c = cases[n]
        out = os.path.join(tmp, n + '.wav')
        code, h, err = run_decoder(args.exe or ref_dec, c, out)
        r = dict(case=n, criteria=c['criteria'], exit=code, error=err)
        if h is None:
            r['status'] = 'no output' + (f' ({err})' if err else '')
            return r
        test, rate = read_wav(out)
        os.remove(out)
        ref, iso_rate = read_wav(os.path.join(CONF, c['iso_wav']))
        r.update(channels=(test.shape[1], ref.shape[1]), rate=(rate, iso_rate), frames=(len(test), len(ref)))
        if test.shape[1] != ref.shape[1] or rate != iso_rate:
            r['status'] = 'format differs'
            return r
        n_frames = min(len(test), len(ref))
        r.update(compare_iso(test[:n_frames], ref[:n_frames]))
        r['verdict'] = iso_verdict(r, c['criteria'])
        r['verdict_tail'] = iso_verdict(r, c['criteria'], 'max_tail')
        r['status'] = 'compared'
        return r

    with cf.ThreadPoolExecutor(args.jobs or os.cpu_count()) as ex:
        res = list(ex.map(run, todo))
    os.rmdir(tmp)
    json.dump(res, open(os.path.join(CONF, 'iso-report.json'), 'w'), indent=1)

    def lsb(v, bits):
        return f'{v * 2 ** (bits - 1):.3f}'

    print(f'{"case":56} {"criteria":24} {"RMS":>7} {"max":>7} {"max>1k":>7} {"segSNR":>7}  result')
    for r in res:
        crit = r['criteria'] or {}
        ctext = ' '.join((crit.get('text') or '-').split())[:24]
        if r['status'] != 'compared':
            info = r['status']
            if r['status'] == 'format differs':
                info += f' (channels {r["channels"][0]} vs {r["channels"][1]}, rate {r["rate"][0]} vs {r["rate"][1]})'
            print(f'{r["case"]:56} {ctext:24} {"":7} {"":7} {"":7} {"":7}  {info}')
            continue
        bits = crit.get('rms_bits', 16)
        snr = f'{r["segsnr"]:.1f}' if r['segsnr'] is not None else '-'
        v = {True: 'pass', False: 'FAIL', None: '-'}[r['verdict']]
        if r['verdict'] is False and r['verdict_tail']:
            v += ' (pass without the first 1024 samples)'
        lens = '' if r['frames'][0] == r['frames'][1] else f' (frames {r["frames"][0]} vs {r["frames"][1]})'
        mb = crit.get('max_bits', 16)
        print(f'{r["case"]:56} {ctext:24} {lsb(r["rms"], bits):>7} {lsb(r["max"], mb):>7} '
              f'{lsb(r["max_tail"], mb):>7} {snr:>7}  {v}{lens}')
    cnt = {s: sum(r['status'] == s for r in res) for s in ('compared', 'no output', 'format differs')}
    passed = sum(r.get('verdict') is True for r in res)
    passed_tail = sum(r.get('verdict_tail') is True for r in res)
    judged = sum(r.get('verdict') is not None for r in res)
    print(f'{len(res)} cases: {cnt["compared"]} compared, {judged} with a criterion: {passed} pass '
          f'({passed_tail} without the first 1024 samples); {len(res) - cnt["compared"]} not comparable. '
          f'RMS and max in LSB at the resolution of the criteria (16 bit if none), max>1k: maximum '
          f'without the first 1024 samples. Report: {os.path.join(CONF, "iso-report.json")}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('fetch')
    p.add_argument('--waveforms', action='store_true', help='also the ISO reference waveforms')
    p.add_argument('--jobs', type=int, default=8)
    sub.add_parser('prepare')
    p = sub.add_parser('refs')
    p.add_argument('--jobs', type=int)
    p.add_argument('-v', '--verbose', action='store_true')
    p = sub.add_parser('iso')
    p.add_argument('--jobs', type=int)
    p.add_argument('--exe', help='decoder (default: the reference decoder)')
    p.add_argument('--all', action='store_true', help='also the bitstreams of the negative tests')
    args = ap.parse_args()
    {'fetch': cmd_fetch, 'prepare': cmd_prepare, 'refs': cmd_refs, 'iso': cmd_iso}[args.cmd](args)


if __name__ == '__main__':
    main()
