// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not a port: the reference decoder is single-threaded.

//! Background thread of the frame pipelining (not in C).
//!
//! With frame pipelining (see [`crate::render`]) the rendering of access unit N runs on a
//! [`RenderWorker`] while the calling thread decodes access unit N+1. Frames decode
//! quickly, so the hand-over has to be fast: like the workers of the thread pool
//! ([`mpeghdec_fdk::thread_pool`]), the render worker spins for a while ([`DEFAULT_SPIN`]) after
//! its last job before it parks, and [`Pending::wait`] spins (yielding) until the job is done.

use std::collections::VecDeque;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Default time the render worker spins waiting for the next job before it parks.
pub const DEFAULT_SPIN: Duration = Duration::from_micros(100);

/// Idle time after which the spinning worker yields its core between polls.
const YIELD_AFTER: Duration = Duration::from_micros(50);

/// Stack size of the render thread (the decoder keeps some larger arrays on the stack).
const WORKER_STACK_SIZE: usize = 8 << 20;

type Job = Box<dyn FnOnce() + Send + 'static>;

struct Shared {
    queue: Mutex<VecDeque<Job>>,
    /// Number of jobs in `queue`.
    queued: AtomicUsize,
    /// Set while the worker is parked (or about to park).
    parked: AtomicBool,
    shutdown: AtomicBool,
    spin: Duration,
}

/// A background thread running jobs in submission order (the render stage of the frame
/// pipeline).
pub struct RenderWorker {
    shared: Arc<Shared>,
    handle: Option<JoinHandle<()>>,
}

impl std::fmt::Debug for RenderWorker {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RenderWorker").finish()
    }
}

/// The result of a job submitted with [`RenderWorker::run`].
pub struct Pending<T> {
    slot: Arc<Slot<T>>,
}

struct Slot<T> {
    done: AtomicBool,
    value: Mutex<Option<thread::Result<T>>>,
}

impl RenderWorker {
    /// Creates the render thread, which spins [`DEFAULT_SPIN`] after its last job before it
    /// parks.
    pub fn new() -> std::io::Result<RenderWorker> {
        RenderWorker::with_spin(DEFAULT_SPIN)
    }

    /// Like [`RenderWorker::new`] with the time `spin` the thread waits actively for the next
    /// job.
    pub fn with_spin(spin: Duration) -> std::io::Result<RenderWorker> {
        let shared = Arc::new(Shared {
            queue: Mutex::new(VecDeque::new()),
            queued: AtomicUsize::new(0),
            parked: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            spin,
        });
        let s = shared.clone();
        let handle = thread::Builder::new()
            .name("mpeghdec-render".to_string())
            .stack_size(WORKER_STACK_SIZE)
            .spawn(move || worker_main(&s))?;
        Ok(RenderWorker { shared, handle: Some(handle) })
    }

    /// Runs `f` on the render thread (after the jobs submitted before). A panic of `f` is
    /// propagated by [`Pending::wait`].
    pub fn run<T: Send + 'static, F: FnOnce() -> T + Send + 'static>(&self, f: F) -> Pending<T> {
        let slot = Arc::new(Slot { done: AtomicBool::new(false), value: Mutex::new(None) });
        let s = slot.clone();
        let job: Job = Box::new(move || {
            let r = catch_unwind(AssertUnwindSafe(f));
            *s.value.lock().unwrap_or_else(|e| e.into_inner()) = Some(r);
            s.done.store(true, Ordering::Release);
        });
        let shared = &*self.shared;
        shared.queue.lock().unwrap_or_else(|e| e.into_inner()).push_back(job);
        shared.queued.fetch_add(1, Ordering::SeqCst);
        if shared.parked.load(Ordering::SeqCst) && shared.parked.swap(false, Ordering::SeqCst) {
            if let Some(h) = &self.handle {
                h.thread().unpark();
            }
        }
        Pending { slot }
    }
}

impl Drop for RenderWorker {
    fn drop(&mut self) {
        let shared = &*self.shared;
        shared.shutdown.store(true, Ordering::SeqCst);
        shared.parked.store(false, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            h.thread().unpark();
            let _ = h.join();
        }
    }
}

impl<T> Pending<T> {
    /// Whether the job has finished ([`Pending::wait`] returns without waiting).
    pub fn is_done(&self) -> bool {
        self.slot.done.load(Ordering::Acquire)
    }

    /// Waits for the job and returns its result (spinning, then yielding the core).
    pub fn wait(self) -> T {
        let mut spins = 0u32;
        while !self.slot.done.load(Ordering::Acquire) {
            spins = spins.saturating_add(1);
            if spins < 1 << 14 {
                std::hint::spin_loop();
            } else {
                thread::yield_now();
            }
        }
        let r = self.slot.value.lock().unwrap_or_else(|e| e.into_inner()).take();
        match r.expect("render job result") {
            Ok(v) => v,
            Err(payload) => resume_unwind(payload),
        }
    }
}

fn worker_main(shared: &Shared) {
    let mut last_work = Instant::now();
    let mut spins = 0u32;
    loop {
        if shared.queued.load(Ordering::Acquire) > 0 {
            let job = shared.queue.lock().unwrap_or_else(|e| e.into_inner()).pop_front();
            if let Some(job) = job {
                shared.queued.fetch_sub(1, Ordering::SeqCst);
                job();
                last_work = Instant::now();
                continue;
            }
        }
        if shared.shutdown.load(Ordering::SeqCst) {
            return;
        }
        spins = spins.wrapping_add(1);
        if spins % 64 == 0 {
            let idle = last_work.elapsed();
            if idle >= shared.spin {
                park(shared);
                last_work = Instant::now();
            } else if idle >= YIELD_AFTER {
                /* let other threads run on this core if the machine is oversubscribed */
                thread::yield_now();
            }
        }
        std::hint::spin_loop();
    }
}

/// Parks the worker until a job is queued (or the worker is dropped).
fn park(shared: &Shared) {
    shared.parked.store(true, Ordering::SeqCst);
    while shared.parked.load(Ordering::SeqCst) {
        if shared.queued.load(Ordering::SeqCst) > 0 || shared.shutdown.load(Ordering::SeqCst) {
            shared.parked.store(false, Ordering::SeqCst);
            break;
        }
        thread::park();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_jobs_in_order() {
        let w = RenderWorker::with_spin(Duration::from_micros(20)).unwrap();
        let log = Arc::new(Mutex::new(Vec::new()));
        let mut pending = Vec::new();
        for i in 0..1000u32 {
            let l = log.clone();
            pending.push(w.run(move || {
                l.lock().unwrap().push(i);
                i * 2
            }));
            if i % 100 == 0 {
                thread::sleep(Duration::from_micros(300));
            }
        }
        for (i, p) in pending.into_iter().enumerate() {
            assert_eq!(p.wait(), i as u32 * 2);
        }
        assert_eq!(*log.lock().unwrap(), (0..1000).collect::<Vec<_>>());
    }

    #[test]
    fn panics_propagate() {
        let w = RenderWorker::new().unwrap();
        let p = w.run(|| -> u32 { panic!("render panic") });
        assert!(catch_unwind(AssertUnwindSafe(|| p.wait())).is_err());
        /* the worker stays usable */
        assert_eq!(w.run(|| 7).wait(), 7);
    }
}
