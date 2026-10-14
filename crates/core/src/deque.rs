// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/deque.h and libMpeghDec/src/deque.cpp.

//! Double ended queue (ring buffer) of fixed capacity.
//!
//! The C++ ring buffer stores untyped blocks of `block_size` bytes (`void*`); here it is generic
//! over the element type `T`, which takes the role of the block. Functions returning a `void*`
//! into the buffer (`NULL` if there is no element) return `Option<&T>` (or `Option<&mut T>` for
//! the `_mut` variants, since the C++ callers also write through these pointers).
//!
//! The quirks of the C++ code are kept: `deque_pop_back()` computes the position from `size`
//! if `last` is 0, and the bulk pops clear `full` even for 0 elements, after which
//! `deque_push_back()` succeeds on a full queue (`size` then exceeds `max`).

use mpeghdec_fdk::digest::StateDigest;

/// `deque` (`struct ring_buf`)
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Deque<T> {
    /// `data`: `max` elements (empty for `NULL`)
    pub data: Vec<T>,
    /// `first`
    pub first: u32,
    /// `last`
    pub last: u32,
    /// `size`
    pub size: u32,
    /// `max`
    pub max: u32,
    /// `full`
    pub full: bool,
}

/// The state digest of a queue is its logical content: `max`, `size`, `full`, the elements in
/// queue order and `last - first - size` (0 except after the `deque_pop_back()` quirk), not the
/// position of the elements in the ring buffer (elements outside `first..first + size` are never
/// read). Every operation addresses the elements relative to `first` (and `last`), so queues with
/// the same logical content behave the same, except `deque_pop_back()` with `last == 0`, which
/// computes the position from `size`. The decoder pops back only on the error paths of
/// `mpeghdecoder_process()` (`aacDecoder_Fill()` failed or the decoder could not be reopened),
/// after which the caller stops decoding.
impl<T: StateDigest> StateDigest for Deque<T> {
    fn digest(&self, d: &mut mpeghdec_fdk::digest::Digester) {
        d.u32(self.max);
        d.u32(self.size);
        d.bytes(&[self.full as u8]);
        if self.max > 0 {
            let rel = (self.last as u64 + 2 * self.max as u64
                - self.first as u64
                - (self.size % self.max) as u64)
                % self.max as u64;
            d.u64(rel);
            let n = self.size.min(self.max) as usize;
            let first = self.first as usize;
            let a = &self.data[first..(first + n).min(self.max as usize)];
            let b = &self.data[..n - a.len()];
            d.field_with("data", |d| {
                T::digest_slice(a, d);
                T::digest_slice(b, d);
            });
        }
    }
}

/// `deque_alloc(queue, length, block_size)`: allocate a queue of `length` zero-initialized
/// elements. Returns -1 on out of memory, else 0.
pub fn deque_alloc<T: Copy + Default>(q: &mut Deque<T>, length: u32) -> i32 {
    let mut v: Vec<T> = Vec::new();
    if v.try_reserve_exact(length as usize).is_err() {
        return -1;
    }
    v.resize(length as usize, T::default());
    q.data = v;

    q.first = 0;
    q.last = 0;
    q.size = 0;
    q.max = length;
    q.full = false;
    0
}

/// `deque_free(queue)`
pub fn deque_free<T>(q: &mut Deque<T>) {
    q.data = Vec::new();
    deque_clear(q);
}

/// `deque_push_back(queue, data)`: returns -1 if the queue is full, else 0.
pub fn deque_push_back<T: Copy>(q: &mut Deque<T>, data: &T) -> i32 {
    if q.full {
        return -1;
    }

    q.data[q.last as usize] = *data;
    q.last = q.last.wrapping_add(1) % q.max;
    q.size = q.size.wrapping_add(1);
    if q.size == q.max {
        q.full = true;
    }
    0
}

/// `deque_bulk_push_back(q, data, numData)` with `numData = data.len()`: returns -1 if there is
/// not enough space, else 0.
pub fn deque_bulk_push_back<T: Copy>(q: &mut Deque<T>, data: &[T]) -> i32 {
    let num_data = data.len() as u32;
    if q.full || q.size.wrapping_add(num_data) > q.max {
        return -1;
    }

    let space = q.max - q.last;
    let left;
    if space > 0 {
        let to_copy;
        if num_data >= space {
            to_copy = space;
            left = num_data - space;
        } else {
            to_copy = num_data;
            left = 0;
        }
        let last = q.last as usize;
        q.data[last..last + to_copy as usize].copy_from_slice(&data[..to_copy as usize]);
    } else {
        left = num_data;
    }

    if left > 0 {
        let src = (num_data - left) as usize;
        q.data[..left as usize].copy_from_slice(&data[src..src + left as usize]);
    }
    q.last = q.last.wrapping_add(num_data) % q.max;
    q.size = q.size.wrapping_add(num_data);
    if q.size == q.max {
        q.full = true;
    }
    0
}

/// `deque_bulk_push_back_zeros(q, numZeros)`: push `num_zeros` zero (default) elements. Returns
/// -1 if there is not enough space, else 0.
pub fn deque_bulk_push_back_zeros<T: Copy + Default>(q: &mut Deque<T>, num_zeros: u32) -> i32 {
    if q.full || q.size.wrapping_add(num_zeros) > q.max {
        return -1;
    }

    let space = q.max - q.last;
    let left;
    if space > 0 {
        let to_copy;
        if num_zeros >= space {
            to_copy = space;
            left = num_zeros - space;
        } else {
            to_copy = num_zeros;
            left = 0;
        }
        let last = q.last as usize;
        q.data[last..last + to_copy as usize].fill(T::default());
    } else {
        left = num_zeros;
    }

    if left > 0 {
        q.data[..left as usize].fill(T::default());
    }
    q.last = q.last.wrapping_add(num_zeros) % q.max;
    q.size = q.size.wrapping_add(num_zeros);
    if q.size == q.max {
        q.full = true;
    }
    0
}

/// `deque_pop_back(queue)`: remove the last element and return it (`None` if the queue is
/// empty).
///
/// Like the C++ code, the position of the last element is computed from `size` if `last` is 0,
/// which is only correct if the elements start at index 0 or the queue is full.
pub fn deque_pop_back<T>(q: &mut Deque<T>) -> Option<&T> {
    if q.size == 0 {
        return None;
    }

    let mut tmp = q.last;
    if tmp == 0 {
        if q.first != 0 {
            /* the position depends on the physical layout, which is not part of the state
            digest */
            mpeghdec_fdk::digest::hazard();
        }
        tmp = q.size;
    }
    let pos = tmp.wrapping_sub(1) % q.max;
    q.last = pos;
    q.size -= 1;
    q.full = false;
    Some(&q.data[pos as usize])
}

/// `deque_pop_front(queue)`: remove the first element and return it (`None` if the queue is
/// empty).
pub fn deque_pop_front<T>(q: &mut Deque<T>) -> Option<&T> {
    if q.size == 0 {
        return None;
    }

    let pos = q.first;
    q.first = q.first.wrapping_add(1) % q.max;
    q.size -= 1;
    q.full = false;
    Some(&q.data[pos as usize])
}

/// `deque_bulk_pop_front_copy(q, destination, numData)` with `numData = destination.len()`:
/// remove the first `numData` elements and copy them to `destination`. Returns -1 if the queue
/// holds less than `numData` elements, else 0.
pub fn deque_bulk_pop_front_copy<T: Copy>(q: &mut Deque<T>, destination: &mut [T]) -> i32 {
    let num_data = destination.len() as u32;
    if q.size < num_data {
        return -1;
    }

    let mut left = num_data as usize;
    let mut data = q.first as usize;
    let mut dest = 0usize;
    if q.first.wrapping_add(num_data) >= q.max {
        let to_copy = (q.max - q.first) as usize;
        destination[..to_copy].copy_from_slice(&q.data[data..data + to_copy]);
        left -= to_copy;
        data = 0;
        dest = to_copy;
    }
    destination[dest..dest + left].copy_from_slice(&q.data[data..data + left]);
    q.first = q.first.wrapping_add(num_data) % q.max;
    q.size -= num_data;
    q.full = false;
    0
}

/// `deque_bulk_pop_front_copy(src, tmp[..n_pop])` followed by
/// `deque_bulk_push_back(dst, tmp[..n_push])` (`n_push <= n_pop`) without the intermediate
/// buffer: moves the first `n_push` elements of `src` to the back of `dst` (unless `dst` has not
/// enough space) and removes the first `n_pop` elements of `src`. Returns `false` and does
/// nothing if `src` holds less than `n_pop` elements (the C++ code would then push the stale
/// content of `tmp`).
pub fn deque_bulk_move<T: Copy>(
    src: &mut Deque<T>,
    dst: &mut Deque<T>,
    n_pop: u32,
    n_push: u32,
) -> bool {
    debug_assert!(n_push <= n_pop);
    if src.size < n_pop {
        return false;
    }
    if !(dst.full || dst.size.wrapping_add(n_push) > dst.max) {
        let (mut from, mut to, mut left) = (src.first as usize, dst.last as usize, n_push as usize);
        let (smax, dmax) = (src.max as usize, dst.max as usize);
        while left > 0 {
            if from == smax {
                from = 0;
            }
            if to == dmax {
                to = 0;
            }
            let n = left.min(smax - from).min(dmax - to);
            dst.data[to..to + n].copy_from_slice(&src.data[from..from + n]);
            (from, to, left) = (from + n, to + n, left - n);
        }
        dst.last = dst.last.wrapping_add(n_push) % dst.max;
        dst.size = dst.size.wrapping_add(n_push);
        if dst.size == dst.max {
            dst.full = true;
        }
    }
    deque_bulk_pop_front(src, n_pop);
    true
}

/// `deque_bulk_pop_front(q, numData)`: remove the first `num_data` elements. Returns -1 if the
/// queue holds less than `num_data` elements, else 0.
pub fn deque_bulk_pop_front<T>(q: &mut Deque<T>, num_data: u32) -> i32 {
    if q.size < num_data {
        return -1;
    }

    q.first = q.first.wrapping_add(num_data) % q.max;
    q.size -= num_data;
    q.full = false;
    0
}

/// `deque_full(queue)`
pub fn deque_full<T>(q: &Deque<T>) -> bool {
    q.full
}

/// `deque_empty(queue)`
pub fn deque_empty<T>(q: &Deque<T>) -> bool {
    q.size == 0
}

/// `deque_size(queue)`
pub fn deque_size<T>(q: &Deque<T>) -> u32 {
    q.size
}

/// `deque_space(queue)`
pub fn deque_space<T>(q: &Deque<T>) -> u32 {
    q.max.wrapping_sub(q.size)
}

/// `deque_clear(queue)`
pub fn deque_clear<T>(q: &mut Deque<T>) {
    q.first = 0;
    q.last = 0;
    q.size = 0;
    q.full = false;
}

/// Position of element `index` (`None` if `index >= size`).
fn deque_pos<T>(q: &Deque<T>, index: u32) -> Option<usize> {
    if index >= q.size {
        return None;
    }
    Some((q.first.wrapping_add(index) % q.max) as usize)
}

/// `deque_at(queue, index)`: element `index` (`None` if `index >= size`).
pub fn deque_at<T>(q: &Deque<T>, index: u32) -> Option<&T> {
    deque_pos(q, index).map(|pos| &q.data[pos])
}

/// `deque_at(queue, index)` for writing through the returned pointer.
pub fn deque_at_mut<T>(q: &mut Deque<T>, index: u32) -> Option<&mut T> {
    deque_pos(q, index).map(|pos| &mut q.data[pos])
}

/// `deque_front(queue)`
pub fn deque_front<T>(q: &Deque<T>) -> Option<&T> {
    deque_at(q, 0)
}

/// `deque_front(queue)` for writing through the returned pointer.
pub fn deque_front_mut<T>(q: &mut Deque<T>) -> Option<&mut T> {
    deque_at_mut(q, 0)
}

/// `deque_back(queue)`
pub fn deque_back<T>(q: &Deque<T>) -> Option<&T> {
    if q.size == 0 {
        return None;
    }
    deque_at(q, q.size - 1)
}

/// `deque_back(queue)` for writing through the returned pointer.
pub fn deque_back_mut<T>(q: &mut Deque<T>) -> Option<&mut T> {
    if q.size == 0 {
        return None;
    }
    let index = q.size - 1;
    deque_at_mut(q, index)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `deque_bulk_move()` against the pop into a buffer and the push of the C++ code.
    #[test]
    fn bulk_move_equals_pop_and_push() {
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        let mut next = move |m: u64| {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x % m
        };
        for _ in 0..2000 {
            let (smax, dmax) = (1 + next(40) as u32, 1 + next(40) as u32);
            let (mut a, mut b) = (Deque::<i32>::default(), Deque::<i32>::default());
            deque_alloc(&mut a, smax);
            deque_alloc(&mut b, dmax);
            let mut val = 0;
            for _ in 0..50 {
                // random fill levels and ring positions
                for _ in 0..next(3) {
                    let n = next(smax as u64 + 1) as usize;
                    let v: Vec<i32> = (0..n).map(|i| val + i as i32).collect();
                    val += n as i32;
                    deque_bulk_push_back(&mut a, &v);
                    deque_bulk_pop_front(&mut b, next(dmax as u64 + 1) as u32);
                    let v: Vec<i32> = (0..next(dmax as u64 + 1)).map(|i| -(i as i32)).collect();
                    deque_bulk_push_back(&mut b, &v);
                }
                let n_pop = next(smax as u64 + 2) as u32;
                let n_push = next(n_pop as u64 + 1) as u32;
                let (mut a2, mut b2) = (a.clone(), b.clone());
                let mut tmp = vec![0i32; n_pop as usize];
                if deque_bulk_pop_front_copy(&mut a2, &mut tmp) == 0 {
                    deque_bulk_push_back(&mut b2, &tmp[..n_push as usize]);
                    assert!(deque_bulk_move(&mut a, &mut b, n_pop, n_push));
                } else {
                    assert!(!deque_bulk_move(&mut a, &mut b, n_pop, n_push));
                }
                assert_eq!((&a, &b), (&a2, &b2));
            }
        }
    }
}
