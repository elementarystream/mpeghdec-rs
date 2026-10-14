// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! `qsort()` of the C library of the reference build.
//!
//! `quickHull.cpp` sorts loudspeakers with `qsort()` and a comparison function that reports many
//! elements as equal (e.g. symmetric left/right speakers). `qsort()` is not stable, so the order
//! of equal elements, which determines the triangulation, depends on the C library. [`qsort`]
//! reproduces the C library of the target: the Apple Libc implementation on Apple targets
//! ([`qsort_bsd`]) and the GNU C library implementation on all others ([`qsort_glibc`]).
//!
//! The Apple Libc implementation is Bentley & McIlroy's
//! "Engineering a Sort Function" as in FreeBSD's `qsort.c` (Copyright (c) 1992, 1993 The Regents
//! of the University of California, BSD license) with these properties (verified against the
//! disassembly of `libsystem_c` and by differential tests):
//!
//! - introsort style recursion limit `2 * (flsl(n) - 1)`, then BSD `heapsort()`,
//! - insertion sort for `n <= 7`,
//! - pseudomedian of 9 for `n > 40`, median of 3 otherwise,
//! - if a partitioning pass did not swap anything, an insertion sort is tried after the
//!   partitions have been moved into place and abandoned after `1 + n / 4` swaps,
//! - recursion into the smaller partition, iteration on the larger one.

use core::cmp::Ordering;

/// `qsort(base, nmemb, size, compar)` on a slice; `cmp` returns the C comparison result.
///
/// The `qsort()` of the target's C library: [`qsort_bsd`] on Apple targets, [`qsort_glibc`]
/// otherwise.
pub fn qsort<T: Copy, F: FnMut(&T, &T) -> i32>(a: &mut [T], cmp: F) {
    if cfg!(target_vendor = "apple") {
        qsort_bsd(a, cmp)
    } else {
        qsort_glibc(a, cmp)
    }
}

/// `qsort()` of the GNU C library (`msort.c`, glibc 2.39): a top-down merge sort, i.e. a stable
/// sort, with the comparisons in the order of glibc (`msort_with_tmp()`: the first half, then
/// the second half, then the merge, which takes the element of the first half if `cmp <= 0`).
///
/// glibc falls back to an unstable quicksort only if it cannot allocate the temporary buffer
/// (not for the small arrays sorted here).
pub fn qsort_glibc<T: Copy, F: FnMut(&T, &T) -> i32>(a: &mut [T], mut cmp: F) {
    let mut tmp = a.to_vec();
    msort_with_tmp(a, &mut tmp, &mut cmp);
}

/// `msort_with_tmp()`: sorts `b` with the scratch buffer `tmp` (at least as long as `b`).
fn msort_with_tmp<T: Copy, F: FnMut(&T, &T) -> i32>(b: &mut [T], tmp: &mut [T], cmp: &mut F) {
    let n = b.len();
    if n <= 1 {
        return;
    }
    let n1 = n / 2;
    msort_with_tmp(&mut b[..n1], tmp, cmp);
    msort_with_tmp(&mut b[n1..], tmp, cmp);
    let (mut i1, mut i2, mut t) = (0, n1, 0);
    while i1 < n1 && i2 < n {
        if cmp(&b[i1], &b[i2]) <= 0 {
            tmp[t] = b[i1];
            i1 += 1;
        } else {
            tmp[t] = b[i2];
            i2 += 1;
        }
        t += 1;
    }
    // the rest of the first half follows the merged part; the rest of the second half is
    // already in place
    tmp[t..t + n1 - i1].copy_from_slice(&b[i1..n1]);
    t += n1 - i1;
    b[..t].copy_from_slice(&tmp[..t]);
}

/// `qsort()` of Apple Libc (the BSD implementation described above).
pub fn qsort_bsd<T: Copy, F: FnMut(&T, &T) -> i32>(a: &mut [T], mut cmp: F) {
    let n = a.len();
    let depth_limit = 2 * (fls(n) - 1);
    qsort_rec(a, 0, n, &mut cmp, depth_limit);
}

/// `flsl(n)`: index of the most significant set bit (1-based), 0 for 0.
fn fls(n: usize) -> i32 {
    (usize::BITS - n.leading_zeros()) as i32
}

/// `_isort()`: insertion sort. With `swap_limit != 0`, it gives up (returning `false`) as soon as
/// more than `swap_limit` swaps would be needed; the swaps done so far are kept.
fn isort<T: Copy, F: FnMut(&T, &T) -> i32>(
    a: &mut [T],
    base: usize,
    n: usize,
    cmp: &mut F,
    swap_limit: usize,
) -> bool {
    let mut swap_cnt = 0;
    for pm in base + 1..base + n {
        let mut pl = pm;
        while pl > base && cmp(&a[pl - 1], &a[pl]) > 0 {
            a.swap(pl, pl - 1);
            if swap_limit != 0 {
                if swap_cnt >= swap_limit {
                    return false;
                }
                swap_cnt += 1;
            }
            pl -= 1;
        }
    }
    true
}

fn med3<T: Copy, F: FnMut(&T, &T) -> i32>(
    a: &[T],
    x: usize,
    y: usize,
    z: usize,
    cmp: &mut F,
) -> usize {
    if cmp(&a[x], &a[y]) < 0 {
        if cmp(&a[y], &a[z]) < 0 {
            y
        } else if cmp(&a[x], &a[z]) < 0 {
            z
        } else {
            x
        }
    } else if cmp(&a[y], &a[z]) > 0 {
        y
    } else if cmp(&a[x], &a[z]) < 0 {
        x
    } else {
        z
    }
}

fn vecswap<T: Copy>(a: &mut [T], i: usize, j: usize, n: usize) {
    for k in 0..n {
        a.swap(i + k, j + k);
    }
}

fn qsort_rec<T: Copy, F: FnMut(&T, &T) -> i32>(
    a: &mut [T],
    mut base: usize,
    mut n: usize,
    cmp: &mut F,
    mut depth_limit: i32,
) {
    loop {
        if depth_limit <= 0 {
            heapsort(&mut a[base..base + n], cmp);
            return;
        }
        depth_limit -= 1;

        if n <= 7 {
            isort(a, base, n, cmp, 0);
            return;
        }
        let mut pm = base + n / 2;
        let mut pl = base;
        let mut pn = base + n - 1;
        if n > 40 {
            let d = n / 8;
            pl = med3(a, pl, pl + d, pl + 2 * d, cmp);
            pm = med3(a, pm - d, pm, pm + d, cmp);
            pn = med3(a, pn - 2 * d, pn - d, pn, cmp);
        }
        pm = med3(a, pl, pm, pn, cmp);
        a.swap(base, pm);

        let mut swap_cnt = false;
        let mut pa = base + 1;
        let mut pb = base + 1;
        let mut pc = base + n - 1;
        let mut pd = base + n - 1;
        loop {
            while pb <= pc {
                let r = cmp(&a[pb], &a[base]);
                if r > 0 {
                    break;
                }
                if r == 0 {
                    swap_cnt = true;
                    a.swap(pa, pb);
                    pa += 1;
                }
                pb += 1;
            }
            while pb <= pc {
                let r = cmp(&a[pc], &a[base]);
                if r < 0 {
                    break;
                }
                if r == 0 {
                    swap_cnt = true;
                    a.swap(pc, pd);
                    pd -= 1;
                }
                pc -= 1;
            }
            if pb > pc {
                break;
            }
            a.swap(pb, pc);
            swap_cnt = true;
            pb += 1;
            pc -= 1;
        }

        let pn = base + n;
        let d = (pa - base).min(pb - pa);
        vecswap(a, base, pb - d, d);
        let d = (pd - pc).min(pn - pd - 1);
        vecswap(a, pb, pn - d, d);

        let r1 = pb - pa;
        let r2 = pd - pc;

        if !swap_cnt {
            /* Switch to insertion sort */
            if isort(a, base, n, cmp, 1 + n / 4) {
                return;
            }
        }

        match r1.cmp(&r2) {
            Ordering::Less | Ordering::Equal => {
                /* Recurse on left partition, then iterate on right partition */
                if r1 > 1 {
                    qsort_rec(a, base, r1, cmp, depth_limit);
                }
                if r2 > 1 {
                    base = pn - r2;
                    n = r2;
                    continue;
                }
                return;
            }
            Ordering::Greater => {
                /* Recurse on right partition, then iterate on left partition */
                if r2 > 1 {
                    qsort_rec(a, pn - r2, r2, cmp, depth_limit);
                }
                if r1 > 1 {
                    n = r1;
                    continue;
                }
                return;
            }
        }
    }
}

/// BSD `heapsort(vbase, nmemb, size, compar)`.
fn heapsort<T: Copy, F: FnMut(&T, &T) -> i32>(v: &mut [T], cmp: &mut F) {
    let mut nmemb = v.len();
    if nmemb <= 1 {
        return;
    }
    /* Items are numbered from 1 to nmemb, so offset from size bytes below the starting address. */
    let idx = |i: usize| i - 1;

    /* Build the list into a heap, where a heap is defined such that for the records
     * K1 ... KN, Kj/2 >= Kj for 1 <= j/2 <= j <= N. */
    let mut l = nmemb / 2 + 1;
    loop {
        l -= 1;
        if l == 0 {
            break;
        }
        /* CREATE(l, nmemb, i, j, t, p, size, cnt, tmp) */
        let mut par_i = l;
        loop {
            let mut child_i = par_i * 2;
            if child_i > nmemb {
                break;
            }
            if child_i < nmemb && cmp(&v[idx(child_i)], &v[idx(child_i + 1)]) < 0 {
                child_i += 1;
            }
            if cmp(&v[idx(child_i)], &v[idx(par_i)]) <= 0 {
                break;
            }
            v.swap(idx(par_i), idx(child_i));
            par_i = child_i;
        }
    }

    /* For each element of the heap, save the largest element into its final slot, save the
     * displaced element (k), then recreate the heap. */
    while nmemb > 1 {
        let k = v[idx(nmemb)];
        v[idx(nmemb)] = v[idx(1)];
        nmemb -= 1;
        /* SELECT(i, j, nmemb, t, p, size, k, cnt, tmp1, tmp2) */
        let mut par_i = 1;
        loop {
            let mut child_i = par_i * 2;
            if child_i > nmemb {
                break;
            }
            if child_i < nmemb && cmp(&v[idx(child_i)], &v[idx(child_i + 1)]) < 0 {
                child_i += 1;
            }
            v[idx(par_i)] = v[idx(child_i)];
            par_i = child_i;
        }
        loop {
            let child_i = par_i;
            par_i = child_i / 2;
            if child_i == 1 || cmp(&k, &v[idx(par_i)]) < 0 {
                v[idx(child_i)] = k;
                break;
            }
            v[idx(child_i)] = v[idx(par_i)];
        }
    }
}
