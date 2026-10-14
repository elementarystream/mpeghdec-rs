#!/usr/bin/env python3
# Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
# Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
# See LICENSE.txt for the complete license text.
"""Numerical comparison of two 24 bit PCM WAV files, e.g. a fast mode (not bit-exact) decode
against the bit-exact reference output.

Reports per channel: the maximum absolute difference in LSB of 24 bit, the RMS difference in LSB,
the SNR of the test file against the reference (reference signal power / difference power, in dB)
and the share of samples that are identical / within 1 / 4 / 16 LSB.

Usage: scripts/wavdiff.py REF.wav TEST.wav [--offset N] [--skip N] [--json]
  --offset N  the test file is delayed by N samples relative to the reference (compare
              ref[t] with test[t + N]; negative values: test is early)
  --skip N    ignore the first N samples of the reference (start-up transients)

As a module: compare(ref, test, offset=0, skip=0) returns the same numbers as a dict.
"""
import argparse, json, struct, sys

import numpy as np


def open_wav(path):
    """Returns (file, data offset, channels, sample rate, frames) of a 24 bit PCM WAV file."""
    f = open(path, 'rb')
    riff, _, wave = struct.unpack('<4sI4s', f.read(12))
    if riff != b'RIFF' or wave != b'WAVE':
        raise ValueError(f'{path}: not a WAV file')
    fmt = None
    while True:
        hdr = f.read(8)
        if len(hdr) < 8:
            raise ValueError(f'{path}: no data chunk')
        cid, size = struct.unpack('<4sI', hdr)
        if cid == b'fmt ':
            d = f.read(size)
            _, ch, rate, _, align, bits = struct.unpack('<HHIIHH', d[:16])
            fmt = (ch, rate, bits, align)
        elif cid == b'data':
            ch, rate, bits, align = fmt
            if bits != 24:
                raise ValueError(f'{path}: only 24 bit PCM is supported')
            data_off = f.tell()
            f.seek(0, 2)
            avail = (f.tell() - data_off) // align
            # streaming writers may leave the data size at 0: use the file size then
            n = min(size // align, avail) if size else avail
            return f, data_off, ch, rate, n
        else:
            f.seek(size + (size & 1), 1)


def read_frames(f, data_off, ch, start, count, total):
    """Frames [start, start + count) as an int32 array (count, ch); frames outside [0, total)
    are zero."""
    out = np.zeros((count, ch), dtype=np.int32)
    lo, hi = max(start, 0), min(start + count, total)
    if hi <= lo:
        return out
    f.seek(data_off + lo * ch * 3)
    raw = np.frombuffer(f.read((hi - lo) * ch * 3), dtype=np.uint8).reshape(-1, 3).astype(np.int32)
    v = raw[:, 0] | (raw[:, 1] << 8) | (raw[:, 2] << 16)
    v = np.where(v & 0x800000, v - 0x1000000, v)
    out[lo - start:hi - start] = v.reshape(hi - lo, ch)
    return out


def compare(ref, test, offset=0, skip=0):
    """Compares two WAV files, see the module documentation."""
    fr, orr, chr_, rate_r, nr = open_wav(ref)
    ft, ot, cht, rate_t, nt = open_wav(test)
    try:
        if chr_ != cht:
            raise ValueError(f'channel count differs: {chr_} vs {cht}')
        ch = chr_
        n = max(min(nr, nt - offset) - skip, 0)
        maxabs = np.zeros(ch, np.int64)
        se = np.zeros(ch, np.float64)
        ss = np.zeros(ch, np.float64)
        hist = np.zeros((4, ch), np.int64)
        first_diff = -1
        block = 1 << 18
        pos = skip
        while pos < skip + n:
            cnt = min(block, skip + n - pos)
            r = read_frames(fr, orr, ch, pos, cnt, nr).astype(np.int64)
            t = read_frames(ft, ot, ch, pos + offset, cnt, nt).astype(np.int64)
            d = np.abs(t - r)
            if first_diff < 0:
                nz = np.nonzero(d.any(axis=1))[0]
                if len(nz):
                    first_diff = pos + int(nz[0])
            maxabs = np.maximum(maxabs, d.max(axis=0))
            se += (d.astype(np.float64) ** 2).sum(axis=0)
            ss += (r.astype(np.float64) ** 2).sum(axis=0)
            for i, lim in enumerate((0, 1, 4, 16)):
                hist[i] += (d <= lim).sum(axis=0)
            pos += cnt
        channels = []
        for c in range(ch):
            snr = float('inf') if se[c] == 0 else 10 * np.log10(max(ss[c], 1e-30) / se[c])
            channels.append(dict(ch=c, max_lsb=int(maxabs[c]),
                                 rms_lsb=float(np.sqrt(se[c] / n)) if n else 0.0,
                                 ref_rms_lsb=float(np.sqrt(ss[c] / n)) if n else 0.0,
                                 snr_db=float(snr),
                                 eq=hist[0][c] / max(n, 1), le1=hist[1][c] / max(n, 1),
                                 le4=hist[2][c] / max(n, 1), le16=hist[3][c] / max(n, 1)))
        return dict(frames=int(n), ref_frames=int(nr), test_frames=int(nt), rate=(rate_r, rate_t),
                    first_diff=first_diff, channels=channels)
    finally:
        fr.close()
        ft.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ref')
    ap.add_argument('test')
    ap.add_argument('--offset', type=int, default=0)
    ap.add_argument('--skip', type=int, default=0)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    try:
        res = compare(a.ref, a.test, a.offset, a.skip)
    except ValueError as e:
        sys.exit(str(e))
    if a.json:
        print(json.dumps(res))
        return
    print(f'{res["frames"]} frames x {len(res["channels"])} ch compared (offset {a.offset}, '
          f'skip {a.skip}; lengths {res["ref_frames"]} / {res["test_frames"]}); '
          f'first difference at frame {res["first_diff"]}')
    print(f'{"ch":>3} {"max[LSB]":>9} {"rms[LSB]":>9} {"SNR[dB]":>8} {"==":>7} {"<=1":>7} {"<=4":>7} {"<=16":>7}')
    for r in res['channels']:
        print(f'{r["ch"]:3d} {r["max_lsb"]:9d} {r["rms_lsb"]:9.3f} {r["snr_db"]:8.1f} '
              f'{100 * r["eq"]:6.2f}% {100 * r["le1"]:6.2f}% {100 * r["le4"]:6.2f}% {100 * r["le16"]:6.2f}%')


if __name__ == '__main__':
    main()
