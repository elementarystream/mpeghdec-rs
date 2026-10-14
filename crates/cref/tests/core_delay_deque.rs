// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Differential tests of libMpeghDec FDK_delay.cpp (FDK_Delay_Create()/FDK_Delay_Destroy()) and
//! deque.cpp (random operation sequences on queues of several block sizes and capacities).
#![cfg(not(cref_missing))]

use mpeghdec_core::delay::*;
use mpeghdec_core::deque::*;
use mpeghdec_cref::core_delay_deque::*;
use mpeghdec_cref::testutil::Rng;

// ---------------------------------------------------------------------------------------------
// FDK_delay
// ---------------------------------------------------------------------------------------------

#[test]
fn delay_create_destroy() {
    let mut rng = Rng::new(0xde1a);
    let delays = [0u16, 1, 2, 3, 100, 257, 1024, 3000, 65535];
    let channels = [1u8, 2, 3, 24, 28, 255];
    for &delay in &delays {
        for &num_channels in &channels {
            let mut c = [0i32; 8];
            unsafe {
                cref_core_delay_create_destroy(delay as u32, num_channels as u32, c.as_mut_ptr())
            };

            // start from a structure holding other data
            let mut d = FdkSignalDelay {
                delay_line: vec![rng.i32(); rng.range(0, 10) as usize],
                delay: rng.next_u32() as u16,
                num_channels: rng.next_u32() as u8,
            };
            let mut r = [0i32; 8];
            r[0] = fdk_delay_create(&mut d, delay, num_channels);
            r[1] = !d.delay_line.is_empty() as i32;
            r[2] = d.delay as i32;
            r[3] = d.num_channels as i32;
            r[4] = d.delay_line.iter().filter(|&&v| v != 0).count() as i32;
            if delay > 0 {
                assert_eq!(d.delay_line.len(), delay as usize * num_channels as usize);
            }
            fdk_delay_destroy(&mut d);
            r[5] = !d.delay_line.is_empty() as i32;
            r[6] = d.delay as i32;
            r[7] = d.num_channels as i32;
            assert_eq!(c, r, "delay {delay} channels {num_channels}");
            assert_eq!(d, FdkSignalDelay::default());
        }
    }
}

// ---------------------------------------------------------------------------------------------
// deque
// ---------------------------------------------------------------------------------------------

/// A block type of the queue with its memory representation (the C deque stores bytes).
trait Block: Copy + Default + PartialEq + std::fmt::Debug {
    const SIZE: usize;
    fn put(&self, out: &mut Vec<u8>);
    fn get(b: &[u8]) -> Self;
    fn random(rng: &mut Rng) -> Self;
}

impl Block for u8 {
    const SIZE: usize = 1;
    fn put(&self, out: &mut Vec<u8>) {
        out.push(*self);
    }
    fn get(b: &[u8]) -> Self {
        b[0]
    }
    fn random(rng: &mut Rng) -> Self {
        rng.next_u32() as u8 | 1
    }
}

impl Block for i32 {
    const SIZE: usize = 4;
    fn put(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.to_le_bytes());
    }
    fn get(b: &[u8]) -> Self {
        i32::from_le_bytes(b[..4].try_into().unwrap())
    }
    fn random(rng: &mut Rng) -> Self {
        rng.i32() | 1
    }
}

impl Block for u64 {
    const SIZE: usize = 8;
    fn put(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.to_le_bytes());
    }
    fn get(b: &[u8]) -> Self {
        u64::from_le_bytes(b[..8].try_into().unwrap())
    }
    fn random(rng: &mut Rng) -> Self {
        rng.next_u64() | 1
    }
}

impl Block for [u8; 12] {
    const SIZE: usize = 12;
    fn put(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(self);
    }
    fn get(b: &[u8]) -> Self {
        b[..12].try_into().unwrap()
    }
    fn random(rng: &mut Rng) -> Self {
        let mut a = [0u8; 12];
        for v in a.iter_mut() {
            *v = rng.next_u32() as u8;
        }
        a[0] |= 1;
        a
    }
}

fn to_bytes<T: Block>(v: &[T]) -> Vec<u8> {
    let mut o = Vec::with_capacity(v.len() * T::SIZE);
    for x in v {
        x.put(&mut o);
    }
    o
}

fn from_bytes<T: Block>(b: &[u8]) -> Vec<T> {
    b.chunks(T::SIZE).map(T::get).collect()
}

/// A C deque (with the given block size) and a Rust deque.
struct DequePair<T: Block> {
    c: *mut (),
    r: Deque<T>,
}

impl<T: Block> DequePair<T> {
    fn new() -> Self {
        DequePair { c: unsafe { cref_core_deque_new() }, r: Deque::default() }
    }

    fn check(&self, what: &str) {
        let mut st = [0u32; 7];
        unsafe { cref_core_deque_state(self.c, st.as_mut_ptr()) };
        let r = &self.r;
        let st_r = [
            !r.data.is_empty() as u32,
            T::SIZE as u32,
            r.first,
            r.last,
            r.size,
            r.max,
            r.full as u32,
        ];
        assert_eq!(st, st_r, "{what}: state (data, block_size, first, last, size, max, full)");
        if st[0] != 0 {
            let mut data = vec![0u8; st[5] as usize * T::SIZE];
            unsafe { cref_core_deque_data(self.c, data.as_mut_ptr()) };
            assert_eq!(data, to_bytes(&r.data), "{what}: data");
        }
    }

    /// Block index of an element reference of the Rust deque.
    fn index_of(&self, p: Option<usize>) -> i32 {
        match p {
            None => -1,
            Some(p) => ((p - self.r.data.as_ptr() as usize) / std::mem::size_of::<T>()) as i32,
        }
    }
}

impl<T: Block> Drop for DequePair<T> {
    fn drop(&mut self) {
        unsafe {
            cref_core_deque_free(self.c);
            cref_core_deque_delete(self.c);
        }
    }
}

fn deque_random_ops<T: Block>(seed: u64) {
    let mut rng = Rng::new(seed);
    for &cap in &[1u32, 2, 3, 5, 8, 16, 33, 100] {
        let mut q = DequePair::<T>::new();
        assert_eq!(
            unsafe { cref_core_deque_alloc(q.c, cap, T::SIZE as u32) },
            deque_alloc(&mut q.r, cap)
        );
        q.check(&format!("cap {cap} alloc"));
        let mut max = cap;
        for op_idx in 0..3000 {
            let op = rng.next_u32() % 22;
            let mut next_max = max;
            let what = format!("block {} cap {max} op {op_idx} ({op})", T::SIZE);
            // Number of elements for the bulk operations, biased towards the edge cases.
            // Note: a bulk pop of 0 elements clears `full` of a full queue, after which
            // push_back() succeeds and `size` exceeds `max` (like in C). The values are chosen
            // such that the C code does not access memory outside of the buffer then.
            let rnd_n = |rng: &mut Rng, size: u32| -> u32 {
                match rng.next_u32() % 5 {
                    0 => 0,
                    1 => size,
                    2 => max.saturating_sub(size),
                    3 => max + (rng.next_u32() % 3),
                    _ => rng.next_u32() % (max + 2),
                }
            };
            let cur_max = max;
            match op {
                0..=3 => {
                    let v = T::random(&mut rng);
                    let mut b = Vec::new();
                    v.put(&mut b);
                    let rc = unsafe { cref_core_deque_push_back(q.c, b.as_mut_ptr()) };
                    let rr = deque_push_back(&mut q.r, &v);
                    assert_eq!(rc, rr, "{what}: push_back");
                }
                4 | 5 => {
                    let n = rnd_n(&mut rng, q.r.size);
                    let v: Vec<T> = (0..n).map(|_| T::random(&mut rng)).collect();
                    let mut b = to_bytes(&v);
                    b.push(0);
                    let rc = unsafe { cref_core_deque_bulk_push_back(q.c, b.as_mut_ptr(), n) };
                    let rr = deque_bulk_push_back(&mut q.r, &v);
                    assert_eq!(rc, rr, "{what}: bulk_push_back {n}");
                }
                6 => {
                    let n = rnd_n(&mut rng, q.r.size);
                    let rc = unsafe { cref_core_deque_bulk_push_back_zeros(q.c, n) };
                    let rr = deque_bulk_push_back_zeros(&mut q.r, n);
                    assert_eq!(rc, rr, "{what}: bulk_push_back_zeros {n}");
                }
                7..=9 => {
                    let back = op == 7 || (op == 9 && rng.next_u32() % 2 == 0);
                    let mut b = vec![0u8; T::SIZE];
                    let (ic, res) = if back {
                        let ic = unsafe { cref_core_deque_pop_back(q.c, b.as_mut_ptr()) };
                        (ic, deque_pop_back(&mut q.r).map(|r| (r as *const T as usize, *r)))
                    } else {
                        let ic = unsafe { cref_core_deque_pop_front(q.c, b.as_mut_ptr()) };
                        (ic, deque_pop_front(&mut q.r).map(|r| (r as *const T as usize, *r)))
                    };
                    let ir = q.index_of(res.map(|(p, _)| p));
                    assert_eq!(ic, ir, "{what}: pop (back {back}) position");
                    if let Some((_, v)) = res {
                        assert_eq!(T::get(&b), v, "{what}: pop (back {back}) value");
                    }
                }
                10 | 11 => {
                    let mut n = rnd_n(&mut rng, q.r.size);
                    if q.r.size > max {
                        n = n.min(max);
                    }
                    let mut bc = vec![0u8; n as usize * T::SIZE + 1];
                    let rc =
                        unsafe { cref_core_deque_bulk_pop_front_copy(q.c, bc.as_mut_ptr(), n) };
                    let mut dr = vec![T::default(); n as usize];
                    let rr = deque_bulk_pop_front_copy(&mut q.r, &mut dr);
                    assert_eq!(rc, rr, "{what}: bulk_pop_front_copy {n}");
                    if rc == 0 {
                        assert_eq!(
                            from_bytes::<T>(&bc[..n as usize * T::SIZE]),
                            dr,
                            "{what}: bulk_pop_front_copy data"
                        );
                    }
                }
                12 => {
                    let n = rnd_n(&mut rng, q.r.size);
                    let rc = unsafe { cref_core_deque_bulk_pop_front(q.c, n) };
                    let rr = deque_bulk_pop_front(&mut q.r, n);
                    assert_eq!(rc, rr, "{what}: bulk_pop_front {n}");
                }
                13 | 14 => {
                    let size = q.r.size;
                    let index = match rng.next_u32() % 4 {
                        0 => size,
                        1 => size.wrapping_sub(1),
                        2 => rng.next_u32(),
                        _ => rng.next_u32() % (size + 2),
                    };
                    let ic = unsafe { cref_core_deque_at(q.c, index) };
                    let ir = q.index_of(deque_at(&q.r, index).map(|r| r as *const T as usize));
                    assert_eq!(ic, ir, "{what}: at {index}");
                    let im = deque_at_mut(&mut q.r, index).map(|r| r as *mut T as usize);
                    assert_eq!(ic, q.index_of(im), "{what}: at_mut {index}");
                }
                15 => {
                    let ic = unsafe { cref_core_deque_front(q.c) };
                    let ir = q.index_of(deque_front(&q.r).map(|r| r as *const T as usize));
                    assert_eq!(ic, ir, "{what}: front");
                    let im = deque_front_mut(&mut q.r).map(|r| r as *mut T as usize);
                    assert_eq!(ic, q.index_of(im), "{what}: front_mut");
                    let ic = unsafe { cref_core_deque_back(q.c) };
                    let ir = q.index_of(deque_back(&q.r).map(|r| r as *const T as usize));
                    assert_eq!(ic, ir, "{what}: back");
                    let im = deque_back_mut(&mut q.r).map(|r| r as *mut T as usize);
                    assert_eq!(ic, q.index_of(im), "{what}: back_mut");
                }
                16..=18 => {
                    let c = unsafe {
                        [
                            cref_core_deque_full(q.c),
                            cref_core_deque_empty(q.c),
                            cref_core_deque_size(q.c) as i32,
                            cref_core_deque_space(q.c) as i32,
                        ]
                    };
                    let r = [
                        deque_full(&q.r) as i32,
                        deque_empty(&q.r) as i32,
                        deque_size(&q.r) as i32,
                        deque_space(&q.r) as i32,
                    ];
                    assert_eq!(c, r, "{what}: full/empty/size/space");
                }
                19 => {
                    if rng.next_u32() % 4 == 0 {
                        unsafe { cref_core_deque_clear(q.c) };
                        deque_clear(&mut q.r);
                    }
                }
                _ => {
                    if rng.next_u32() % 8 == 0 {
                        // free and allocate again (possibly with another capacity)
                        unsafe { cref_core_deque_free(q.c) };
                        deque_free(&mut q.r);
                        q.check(&format!("{what}: free"));
                        let new_max =
                            if rng.next_u32() % 2 == 0 { rng.range(1, 40) as u32 } else { cur_max };
                        assert_eq!(
                            unsafe { cref_core_deque_alloc(q.c, new_max, T::SIZE as u32) },
                            deque_alloc(&mut q.r, new_max),
                            "{what}: alloc"
                        );
                        next_max = new_max;
                    }
                }
            }
            q.check(&what);
            max = next_max;
        }
    }
}

#[test]
fn deque_random_ops_u8() {
    deque_random_ops::<u8>(1);
}

#[test]
fn deque_random_ops_i32() {
    deque_random_ops::<i32>(2);
}

#[test]
fn deque_random_ops_u64() {
    deque_random_ops::<u64>(3);
}

#[test]
fn deque_random_ops_12_bytes() {
    deque_random_ops::<[u8; 12]>(4);
}
