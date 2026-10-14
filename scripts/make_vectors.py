#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Create MPEG-H test vectors with the ittiam libmpeghe encoder and decode them with the
Fraunhofer reference decoder.

The vectors cover configurations that the test streams do not: mono, multichannel
layouts up to 22.2, 44.1 kHz, low bitrates (IGF), objects with moving positions (gVBAP) and the
`mha1` MP4 format. Each vector is decoded to several target layouts; the reference outputs and
their SHA-256 hashes are written next to the vectors.

Usage: scripts/make_vectors.py [--out DIR] [--seconds N] [--only NAME...]

Requirements (all local, git-ignored):
  work/libmpeghe/build/bin/MPEGHEncoder   ittiam encoder (https://github.com/ittiam-systems/libmpeghe)
  work/reference/build-<host>/bin/mpeghDecoder  reference decoder (scripts/setup-reference.sh)
  work/streams/*.mp4                            source material (decoded to WAV first)
  sox, numpy

PATCHED_OPTIONS: vectors with an 'enc' entry use the encoder options -mct:, -cplx_pred:, -fdp:,
-ltpf:, -nf:, -enf: and -kernel:, which need scripts/libmpeghe-testbench.patch applied to the
libmpeghe checkout (commit 603275b) and a rebuild.
"""
import argparse, hashlib, json, os, struct, subprocess, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import refsums  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.environ.get('MPEGHDEC_WORK_DIR', os.path.join(REPO, 'work'))
STREAMS = os.environ.get('MPEGHDEC_TEST_STREAMS', os.path.join(WORK, 'streams'))
ENC = os.path.join(WORK, 'libmpeghe', 'build', 'bin', 'MPEGHEncoder')
DEC = refsums.ref_decoder(WORK, refsums.host_arch())
OUT = os.path.join(WORK, 'vectors')
SRC_STREAMS = ['s1_clip_a', 's3_dvs_a', 's5_short_a',
               's2_short_a']

# CICP layout index -> number of channels (encoder input), see ISO/IEC 23091-3.
CICP_CHANNELS = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 8, 9: 3, 10: 4, 11: 7, 12: 8, 13: 24, 14: 8,
                 15: 12, 16: 10, 17: 12, 18: 14, 19: 12, 20: 14}

# name, cicp, bitrate, sample rate, options
VECTORS = [
    ('mono_64k', 1, 64000, 48000, {}),
    ('mono_32k', 1, 32000, 48000, {}),
    ('stereo_64k', 2, 64000, 48000, {}),
    ('stereo_128k', 2, 128000, 48000, {}),
    ('stereo_256k', 2, 256000, 48000, {}),
    ('stereo_96k_44k', 2, 96000, 44100, {}),
    ('stereo_128k_mha1', 2, 128000, 48000, {'op_fmt': 2}),
    ('c3_0_128k', 3, 128000, 48000, {}),
    ('c5_0_192k', 5, 192000, 48000, {}),
    ('c5_1_256k', 6, 256000, 48000, {}),
    ('c5_1_160k', 6, 160000, 48000, {}),
    ('c7_1_320k', 12, 320000, 48000, {}),
    ('c7_1_front_320k', 7, 320000, 48000, {}),
    ('c5_1_2_384k', 14, 384000, 48000, {}),
    ('c5_1_4_448k', 16, 448000, 48000, {}),
    ('c7_1_4_512k', 19, 512000, 48000, {}),
    ('c9_1_4_640k', 20, 640000, 48000, {}),
    ('c22_2_1024k', 13, 1024000, 48000, {}),
    ('obj4_256k', 0, 256000, 48000, {'objects': 4}),
    # A stereo bed plus objects is not decodable (the libmpeghe output yields no frames).
    # Coding tools that the encoder disables by default. These options need a small local patch
    # of the libmpeghe test bench (see PATCHED_OPTIONS).
    ('c5_1_mct_160k', 6, 160000, 48000, {'enc': ['-mct:1']}),
    ('c7_1_4_mct_320k', 19, 320000, 48000, {'enc': ['-mct:1']}),
    ('stereo_cplx_96k', 2, 96000, 48000, {'enc': ['-cplx_pred:1']}),
    ('stereo_fdp_96k', 2, 96000, 48000, {'enc': ['-fdp:1']}),
    ('stereo_ltpf_64k', 2, 64000, 48000, {'enc': ['-ltpf:1']}),
    ('stereo_nf_96k', 2, 96000, 48000, {'enc': ['-nf:1']}),
    ('stereo_kernel_128k', 2, 128000, 48000, {'enc': ['-kernel:1']}),
    ('mono_ltpf_fdp_48k', 1, 48000, 48000, {'enc': ['-ltpf:1', '-fdp:1']}),
    ('c5_1_all_tools_128k', 6, 128000, 48000,
     {'enc': ['-mct:1', '-cplx_pred:1', '-fdp:1', '-ltpf:1', '-nf:1']}),
    # Lower sampling rates (the decoder resamples the output to 48 or 44.1 kHz).
    ('stereo_64k_32k', 2, 64000, 32000, {}),
    ('stereo_48k_24k', 2, 48000, 24000, {}),
    ('mono_32k_16k', 1, 32000, 16000, {}),
    ('c5_1_128k_32k', 6, 128000, 32000, {}),
    ('stereo_64k_22k', 2, 64000, 22050, {}),
    ('stereo_64k_29k', 2, 64000, 29400, {}),
    ('stereo_48k_14k', 2, 48000, 14700, {}),
    ('obj4_128k_24k', 0, 128000, 24000, {'objects': 4}),
]

# Target layouts every vector is decoded to.
TARGET_LAYOUTS = [1, 2, 6, 12, 19, 13]


def run(cmd, **kw):
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        sys.stderr.write(f'command failed: {" ".join(cmd)}\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}\n')
        raise SystemExit(1)
    return r.stdout


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def read_wav(path):
    """Reads a PCM WAV (16/24/32 bit) into float64 [-1, 1), shape (samples, channels)."""
    d = open(path, 'rb').read()
    pos = 12
    fmt = None
    while pos + 8 <= len(d):
        cid, size = d[pos:pos + 4], struct.unpack('<I', d[pos + 4:pos + 8])[0]
        body = d[pos + 8:pos + 8 + size] if cid != b'data' else d[pos + 8:]
        if cid == b'fmt ':
            fmt = struct.unpack('<HHIIHH', body[:16])
        elif cid == b'data':
            ch, bits = fmt[1], fmt[5]
            if bits == 24:
                raw = np.frombuffer(body[:len(body) // 3 * 3], dtype=np.uint8).reshape(-1, 3)
                x = (raw[:, 0].astype(np.int32) | (raw[:, 1].astype(np.int32) << 8) |
                     (raw[:, 2].astype(np.int32) << 16))
                x = np.where(x >= 1 << 23, x - (1 << 24), x) / float(1 << 23)
            elif bits == 16:
                x = np.frombuffer(body[:len(body) // 2 * 2], dtype='<i2') / 32768.0
            else:
                x = np.frombuffer(body[:len(body) // 4 * 4], dtype='<i4') / float(1 << 31)
            n = len(x) // ch * ch
            return x[:n].reshape(-1, ch), fmt[2]
        pos += 8 + size + (size & 1)
    raise ValueError(f'{path}: no data chunk')


def write_wav(path, x, rate, bits=24):
    x = np.clip(x, -1.0, 1.0 - 1.0 / (1 << (bits - 1)))
    ch = x.shape[1]
    q = np.round(x * (1 << (bits - 1))).astype(np.int32)
    if bits == 24:
        b = np.stack([q & 0xff, (q >> 8) & 0xff, (q >> 16) & 0xff], axis=-1).astype(np.uint8).tobytes()
    else:
        b = q.astype('<i2').tobytes()
    hdr = b'RIFF' + struct.pack('<I', 36 + len(b)) + b'WAVE' + b'fmt ' + struct.pack(
        '<IHHIIHH', 16, 1, ch, rate, rate * ch * bits // 8, ch * bits // 8, bits) + b'data' + struct.pack('<I', len(b))
    open(path, 'wb').write(hdr + b)


def source_material(seconds):
    """Stereo reference decodes of test streams, used as audio content."""
    srcs = []
    os.makedirs(os.path.join(OUT, 'src'), exist_ok=True)
    for s in SRC_STREAMS:
        mp4 = os.path.join(STREAMS, s + '.mp4')
        wav = os.path.join(OUT, 'src', s + '.wav')
        if not os.path.exists(mp4):
            continue
        if not os.path.exists(wav):
            run([DEC, '-tl', '2', '-z', str(int(seconds * 48000 / 1024) + 200), '-if', mp4, '-of', wav])
        x, rate = read_wav(wav)
        srcs.append(x)
    if not srcs:
        raise SystemExit(f'no source material found in {STREAMS}')
    return srcs


def make_channels(srcs, n, seconds, rate):
    """n channels of distinct content: source streams, left/right channels and time offsets."""
    length = int(seconds * rate)
    out = np.zeros((length, n))
    for c in range(n):
        s = srcs[(c // 2) % len(srcs)]
        off = int((c * 0.37 % 5.0) * 48000)
        x = s[off:off + int(seconds * 48000) + 1, c % 2]
        if rate != 48000:
            t = np.arange(length) * (48000.0 / rate)
            x = np.interp(t, np.arange(len(x)), x)
        x = x[:length]
        out[:len(x), c] = x * (0.9 if c % 3 else 0.6)
    return out


def write_oam(path, n_objects, n_frames, bed_channels=0, bed_names=()):
    """OAM file version 1 (w13412): header, descriptions, then per 1024-sample frame and object
    (sample index u64, object id u16, azimuth, elevation, radius, gain as float32)."""
    d = b'OAM ' + struct.pack('<H', 1) + struct.pack('<HH', bed_channels, n_objects)
    d += b'Created by make_vectors.py'.ljust(32, b' ')
    for name in bed_names:
        d += name.encode().ljust(64, b'\0')
    for i in range(n_objects):
        d += f'Object #{i:03d}'.encode().ljust(64, b'\0')
    for f in range(n_frames):
        for i in range(n_objects):
            t = f / n_frames
            az = ((i * 360.0 / n_objects) + 540.0 * t) % 360.0 - 180.0
            el = 30.0 * np.sin(2 * np.pi * (t * 3 + i / n_objects))
            gain = 0.5 + 0.5 * abs(np.cos(np.pi * (t * 2 + i / 7)))
            d += struct.pack('<QHffff', (f + 1) * 1024, i, az, el, 1.0, gain)
    open(path, 'wb').write(d)


def encode(name, cicp, br, rate, opts, srcs, seconds):
    vdir = os.path.join(OUT, name)
    os.makedirs(vdir, exist_ok=True)
    op_fmt = opts.get('op_fmt', 3)
    ext = {1: 'mhas', 2: 'mp4', 3: 'mp4'}[op_fmt]
    out = os.path.join(vdir, f'{name}.{ext}')
    if os.path.exists(out):
        return out
    n_obj = opts.get('objects', 0)
    if n_obj:
        # The encoder test bench derives the channel file names from the first 6 characters of
        # the OAM file name, so a 6 character item name is used.
        item = 'vector'
        n_bed = CICP_CHANNELS[cicp] if cicp else 0
        x = make_channels(srcs, n_obj + n_bed, seconds, rate)
        bed_names = []
        if n_bed:
            # Stereo bed: channel files named with azimuth/elevation.
            for c, (az, el) in enumerate([(30, 0), (-30, 0)][:n_bed]):
                suffix = f'_A{az:+04d}_E{el:+03d}.wav'
                write_wav(os.path.join(vdir, item + suffix), x[:, c:c + 1], rate)
                bed_names.append(suffix)
        for i in range(n_obj):
            write_wav(os.path.join(vdir, f'{item}_{i:03d}.wav'), x[:, n_bed + i:n_bed + i + 1], rate)
        oam = os.path.join(vdir, f'{item}.oam')
        write_oam(oam, n_obj, int(seconds * rate / 1024), n_bed, bed_names)
        cmd = [ENC, f'-oam_file:{oam}', f'-ofile:{out}', f'-op_fmt:{op_fmt}', f'-br:{br}']
        if cicp:
            cmd.append(f'-cicp:{cicp}')
    else:
        wav = os.path.join(vdir, f'{name}_input.wav')
        write_wav(wav, make_channels(srcs, CICP_CHANNELS[cicp], seconds, rate), rate)
        cmd = [ENC, f'-ifile:{wav}', f'-ofile:{out}', f'-op_fmt:{op_fmt}', f'-br:{br}', f'-cicp:{cicp}']
    cmd += opts.get('enc', [])
    log = run(cmd, cwd=vdir)
    open(os.path.join(vdir, 'encoder.log'), 'w').write(log)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--out', help='output directory (default: work/vectors)')
    a = ap.parse_args()
    global OUT
    if a.out:
        OUT = os.path.abspath(a.out)
    os.makedirs(OUT, exist_ok=True)
    srcs = source_material(a.seconds + 6)
    manifest_path = os.path.join(OUT, 'manifest.json')
    manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else {}
    for name, cicp, br, rate, opts in VECTORS:
        if a.only and name not in a.only:
            continue
        stream = encode(name, cicp, br, rate, opts, srcs, a.seconds)
        entry = {'stream': os.path.relpath(stream, OUT), 'outputs': {}, 'ref_arch': refsums.host_arch()}
        if stream.endswith('.mp4'):
            for tl in TARGET_LAYOUTS:
                wav = os.path.join(os.path.dirname(stream), f'{name}.tl{tl}.ref.wav')
                log = run([DEC, '-tl', str(tl), '-if', stream, '-of', wav])
                frames = [l for l in log.splitlines() if 'Decoded MPEG-H audio frames' in l]
                entry['outputs'][str(tl)] = {'sha256': sha256(wav), 'log': frames[-1] if frames else ''}
        manifest[name] = entry
        print(name, entry['stream'], ' '.join(f'{k}:{v["sha256"][:8]}' for k, v in entry['outputs'].items()))
        json.dump(manifest, open(manifest_path, 'w'), indent=1, sort_keys=True)


if __name__ == '__main__':
    main()
