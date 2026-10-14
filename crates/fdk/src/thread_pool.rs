// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not a port: the reference decoder is single-threaded.

//! Fork-join thread pool of the multi-core decoding.
//!
//! The decoder splits parts of a frame into independent tasks (for example the inverse
//! transform of every channel) and runs them with [`ThreadPool::for_each`] /
//! [`ThreadPool::for_each_mut`]; the calling thread takes part in the work and the call returns
//! when all tasks are done. Frames decode quickly, so the parallel sections are short and closely
//! spaced: the workers spin for a while ([`DEFAULT_SPIN`]) after their last task before they park,
//! which keeps fork-join rounds cheap while frames are decoded back to back. Parked workers are
//! woken only when a job has enough tasks for them.
//!
//! The pool never changes results: the tasks of a job must be independent of each other (every
//! caller documents why), so their order and thread do not matter. Nested use (a task calling the
//! pool again) and concurrent use from several threads run the tasks sequentially on the calling
//! thread.

use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Default time a worker spins waiting for the next job before it parks.
pub const DEFAULT_SPIN: Duration = Duration::from_micros(100);

/// Idle time after which a spinning worker yields its core between polls.
const YIELD_AFTER: Duration = Duration::from_micros(50);

/// Stack size of the worker threads (the decoder keeps some larger arrays on the stack).
const WORKER_STACK_SIZE: usize = 8 << 20;

/// Aligns a value to its own cache line pair (false sharing between the spinning workers and
/// the counters written by the tasks).
#[repr(align(128))]
struct Padded<T>(T);

/// One fork-join round, owned by the stack frame of [`ThreadPool::for_each`].
struct Job {
    epoch: u64,
    n: usize,
    next: AtomicUsize,
    /// The task function; the lifetime is erased; `for_each` does not return (or unwind) before
    /// no worker references the job anymore.
    f: &'static (dyn Fn(usize) + Sync),
    /// Set if a task panicked in a worker.
    panicked: AtomicBool,
    /// The payload of the first panic of a task in a worker.
    panic: Mutex<Option<Box<dyn std::any::Any + Send>>>,
}

struct Shared {
    /// The current job, null if none. Workers only dereference it while they are counted in
    /// `active`.
    job: Padded<AtomicPtr<Job>>,
    /// Incremented for every job (after `job` is set).
    epoch: Padded<AtomicU64>,
    /// Number of workers that may access the current job.
    active: Padded<AtomicUsize>,
    /// Per worker: set while the worker is parked (or about to park).
    parked: Box<[Padded<AtomicBool>]>,
    shutdown: AtomicBool,
    spin: Duration,
}

/// A fork-join thread pool with `threads - 1` worker threads (the thread calling
/// [`ThreadPool::for_each`] is the remaining one).
pub struct ThreadPool {
    shared: Arc<Shared>,
    workers: Vec<JoinHandle<()>>,
    /// Set while a job runs (nested or concurrent calls run sequentially).
    busy: AtomicBool,
}

impl std::fmt::Debug for ThreadPool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ThreadPool").field("threads", &self.threads()).finish()
    }
}

impl ThreadPool {
    /// Creates a pool for `threads` threads in total (at least 2), i.e. spawns `threads - 1`
    /// workers, which spin [`DEFAULT_SPIN`] before they park.
    pub fn new(threads: usize) -> std::io::Result<ThreadPool> {
        ThreadPool::with_spin(threads, DEFAULT_SPIN)
    }

    /// Like [`ThreadPool::new`] with the time `spin` the workers wait actively for the next job.
    pub fn with_spin(threads: usize, spin: Duration) -> std::io::Result<ThreadPool> {
        let n_workers = threads.max(2) - 1;
        let shared = Arc::new(Shared {
            job: Padded(AtomicPtr::new(std::ptr::null_mut())),
            epoch: Padded(AtomicU64::new(0)),
            active: Padded(AtomicUsize::new(0)),
            parked: (0..n_workers).map(|_| Padded(AtomicBool::new(false))).collect(),
            shutdown: AtomicBool::new(false),
            spin,
        });
        let mut pool = ThreadPool { shared, workers: Vec::new(), busy: AtomicBool::new(false) };
        for idx in 0..n_workers {
            let shared = pool.shared.clone();
            let handle = thread::Builder::new()
                .name(format!("mpeghdec-worker-{idx}"))
                .stack_size(WORKER_STACK_SIZE)
                .spawn(move || worker_main(&shared, idx))?;
            pool.workers.push(handle);
        }
        Ok(pool)
    }

    /// The number of threads (workers and the calling thread).
    pub fn threads(&self) -> usize {
        self.workers.len() + 1
    }

    /// Runs `f(i)` for every `i` in `0..n` on the pool and the calling thread and returns when
    /// all calls returned. The calls must be independent of each other. A panic of a task is
    /// propagated after all tasks finished.
    pub fn for_each<F: Fn(usize) + Sync>(&self, n: usize, f: F) {
        if n <= 1 || self.workers.is_empty() {
            (0..n).for_each(f);
            return;
        }
        if self.busy.swap(true, Ordering::Acquire) {
            /* nested or concurrent use */
            (0..n).for_each(f);
            return;
        }
        let f: &(dyn Fn(usize) + Sync) = &f;
        // SAFETY: only the lifetime is changed. The job (holding the reference) is retracted and
        // all workers have left it before this function returns or unwinds (`JobGuard`).
        let f: &'static (dyn Fn(usize) + Sync) = unsafe { std::mem::transmute(f) };
        let shared = &*self.shared;
        let epoch = shared.epoch.0.load(Ordering::Relaxed) + 1;
        let job = Job {
            epoch,
            n,
            next: AtomicUsize::new(0),
            f,
            panicked: AtomicBool::new(false),
            panic: Mutex::new(None),
        };
        let guard = JobGuard { pool: self };
        shared.job.0.store(&job as *const Job as *mut Job, Ordering::SeqCst);
        shared.epoch.0.store(epoch, Ordering::SeqCst);
        /* wake parked workers that have a task (worker w takes part if n > w + 1) */
        for (w, handle) in self.workers.iter().enumerate().take(n - 1) {
            let flag = &shared.parked[w].0;
            if flag.load(Ordering::SeqCst) && flag.swap(false, Ordering::SeqCst) {
                handle.thread().unpark();
            }
        }
        loop {
            let i = job.next.fetch_add(1, Ordering::Relaxed);
            if i >= n {
                break;
            }
            f(i);
        }
        drop(guard);
        if job.panicked.load(Ordering::Relaxed) {
            let payload = job.panic.lock().unwrap_or_else(|e| e.into_inner()).take();
            if let Some(payload) = payload {
                resume_unwind(payload);
            }
        }
    }

    /// Runs `f(i, &mut items[i])` for every item on the pool and the calling thread (see
    /// [`ThreadPool::for_each`]).
    pub fn for_each_mut<T: Send, F: Fn(usize, &mut T) + Sync>(&self, items: &mut [T], f: F) {
        let base = SyncMutPtr(items.as_mut_ptr());
        self.for_each(items.len(), |i| {
            // SAFETY: `for_each` calls the closure exactly once per index `i < items.len()`, so
            // the mutable references are disjoint; `items` is borrowed for the whole call.
            let item = unsafe { base.at(i) };
            f(i, item)
        });
    }
}

/// Runs `f(i, &mut items[i])` for every item, on `pool` if given, otherwise sequentially in
/// index order.
#[inline]
pub fn for_each_mut<T: Send, F: Fn(usize, &mut T) + Sync>(
    pool: Option<&ThreadPool>,
    items: &mut [T],
    f: F,
) {
    match pool {
        Some(pool) => pool.for_each_mut(items, f),
        None => {
            for (i, item) in items.iter_mut().enumerate() {
                f(i, item);
            }
        }
    }
}

struct SyncMutPtr<T>(*mut T);

// SAFETY: used only to hand out disjoint `&mut T` to other threads (requires `T: Send`).
unsafe impl<T: Send> Sync for SyncMutPtr<T> {}

impl<T> SyncMutPtr<T> {
    /// # Safety
    /// `i` must be in bounds and no other reference to the element may exist.
    #[inline(always)]
    unsafe fn at<'a>(&self, i: usize) -> &'a mut T {
        &mut *self.0.add(i)
    }
}

/// Retracts the job and waits until no worker accesses it (also when a task of the calling
/// thread panics).
struct JobGuard<'a> {
    pool: &'a ThreadPool,
}

impl Drop for JobGuard<'_> {
    fn drop(&mut self) {
        let shared = &*self.pool.shared;
        shared.job.0.store(std::ptr::null_mut(), Ordering::SeqCst);
        let mut spins = 0u32;
        while shared.active.0.load(Ordering::SeqCst) != 0 {
            spins += 1;
            if spins < 1 << 12 {
                std::hint::spin_loop();
            } else {
                /* a worker was descheduled while running a task */
                thread::yield_now();
            }
        }
        self.pool.busy.store(false, Ordering::Release);
    }
}

impl Drop for ThreadPool {
    fn drop(&mut self) {
        let shared = &*self.shared;
        shared.shutdown.store(true, Ordering::SeqCst);
        for (w, handle) in self.workers.iter().enumerate() {
            shared.parked[w].0.store(false, Ordering::SeqCst);
            handle.thread().unpark();
        }
        for handle in self.workers.drain(..) {
            let _ = handle.join();
        }
    }
}

fn worker_main(shared: &Shared, idx: usize) {
    let mut seen = 0u64;
    let mut last_work = Instant::now();
    loop {
        /* wait for the next job */
        let mut spins = 0u32;
        let epoch = loop {
            let e = shared.epoch.0.load(Ordering::Acquire);
            if e != seen {
                break e;
            }
            spins = spins.wrapping_add(1);
            if spins % 64 == 0 {
                if shared.shutdown.load(Ordering::Relaxed) {
                    return;
                }
                let idle = last_work.elapsed();
                if idle >= shared.spin {
                    park(shared, idx, seen);
                    if shared.shutdown.load(Ordering::SeqCst) {
                        return;
                    }
                    last_work = Instant::now();
                } else if idle >= YIELD_AFTER {
                    /* let other threads run on this core if the machine is oversubscribed */
                    thread::yield_now();
                }
            }
            std::hint::spin_loop();
        };

        /* join the job */
        shared.active.0.fetch_add(1, Ordering::SeqCst);
        let p = shared.job.0.load(Ordering::SeqCst);
        if p.is_null() {
            /* the job of `epoch` is already finished */
            seen = epoch;
        } else {
            // SAFETY: the job stays valid while this worker is counted in `active` and the
            // pointer was loaded after the increment (see `JobGuard`).
            let job = unsafe { &*p };
            seen = job.epoch;
            if job.n > idx + 1 {
                last_work = Instant::now();
                loop {
                    let i = job.next.fetch_add(1, Ordering::Relaxed);
                    if i >= job.n {
                        break;
                    }
                    if let Err(payload) = catch_unwind(AssertUnwindSafe(|| (job.f)(i))) {
                        let mut slot = job.panic.lock().unwrap_or_else(|e| e.into_inner());
                        slot.get_or_insert(payload);
                        job.panicked.store(true, Ordering::Relaxed);
                    }
                }
            }
        }
        shared.active.0.fetch_sub(1, Ordering::Release);
    }
}

/// Parks the worker until a job newer than `seen` is published (or the pool is dropped).
fn park(shared: &Shared, idx: usize, seen: u64) {
    let flag = &shared.parked[idx].0;
    flag.store(true, Ordering::SeqCst);
    while flag.load(Ordering::SeqCst) {
        if shared.epoch.0.load(Ordering::SeqCst) != seen || shared.shutdown.load(Ordering::SeqCst) {
            flag.store(false, Ordering::SeqCst);
            break;
        }
        thread::park();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_every_task_once() {
        let pool = ThreadPool::new(4).unwrap();
        for n in [0usize, 1, 2, 3, 7, 100] {
            let mut v = vec![0u32; n];
            pool.for_each_mut(&mut v, |i, x| *x += i as u32 + 1);
            assert!(v.iter().enumerate().all(|(i, &x)| x == i as u32 + 1));
        }
    }

    #[test]
    fn many_rounds_and_parking() {
        let pool = ThreadPool::with_spin(3, Duration::from_micros(10)).unwrap();
        let mut v = vec![0u64; 5];
        for round in 0..20000u64 {
            pool.for_each_mut(&mut v, |i, x| *x += i as u64 * round);
            if round % 1000 == 0 {
                thread::sleep(Duration::from_micros(200));
            }
        }
        let s: u64 = (0..20000u64).sum();
        assert!(v.iter().enumerate().all(|(i, &x)| x == i as u64 * s));
    }

    #[test]
    fn nested_calls_run_sequentially() {
        let pool = ThreadPool::new(4).unwrap();
        let mut v = vec![[0u32; 4]; 4];
        pool.for_each_mut(&mut v, |_, row| {
            pool.for_each_mut(row, |j, x| *x = j as u32);
        });
        assert!(v.iter().all(|r| *r == [0, 1, 2, 3]));
    }

    #[test]
    fn panics_propagate() {
        let pool = ThreadPool::new(4).unwrap();
        let r = catch_unwind(AssertUnwindSafe(|| {
            pool.for_each(64, |i| {
                if i == 63 {
                    panic!("task panic");
                }
            })
        }));
        assert!(r.is_err());
        /* the pool stays usable */
        let mut v = vec![0u8; 16];
        pool.for_each_mut(&mut v, |_, x| *x = 1);
        assert!(v.iter().all(|&x| x == 1));
    }

    #[test]
    fn optional_pool() {
        let mut v = vec![0usize; 10];
        for_each_mut(None, &mut v, |i, x| *x = i);
        assert!(v.iter().enumerate().all(|(i, &x)| x == i));
    }
}
