// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The async runtime executor, plus inter-task communication tools.
//!
//! **Note:** for our purposes, a _task_ is an independent top-level future
//! managed by the executor polling loop. There is a fixed set of tasks,
//! provided to the executor at startup. This is distinct from the casual use
//! of "task" to mean a piece of code that runs concurrently with other code;
//! we'll use the term "concurrent process" for this. The fixed set of tasks
//! managed by the scheduler can execute an _arbitrary number_ of concurrent
//! processes using operations like `join` and `select`.
//!
//!
//! # Starting the executor / operating system
//!
//! The mechanism for "starting the OS" is [`run_tasks`]. That's the right
//! choice for most applications.
//!
//! `run_tasks` is a wrapper around fancier API, which you can use directly in
//! special circumstances:
//!
//! - If you need faster interrupt response, consider allowing some interrupts
//!   to preempt task code using [`run_tasks_with_preemption`].
//! - If you need code to run when no other tasks are ready -- which can be
//!   useful for putting the CPU into a low power state, or toggling a pin to
//!   signal CPU load on a logic analyzer -- see [`run_tasks_with_idle`]
//! - Finally, if you want to turn on all the bells and whistles, you can use
//!   [`run_tasks_with_preemption_and_idle`] which combines the previous two.
//!
//!
//! # Interrupts, wait, and notify
//!
//! So, you've given the OS an array of tasks that need to each be polled
//! forever. The OS could simply poll every task in a big loop (a pattern known
//! in embedded development as a "superloop"), but this has some problems:
//!
//! 1. By constantly checking whether each task can make progress, we keep the
//!    CPU running full-tilt, burning power needlessly.
//!
//! 2. Because any given task may have to wait for *every other task* to be
//!    polled before it gets control, the minimum response latency to events is
//!    increased, possibly by a lot.
//!
//! We can do better.
//!
//! There are, in practice, two reasons why a task might yield.
//!
//! 1. Because it has more work to do immediately, but wants to leave room for
//!    other tasks to execute during a long-running operation. In this case, we
//!    actually *do* want to come right back and poll the task. (To do this, use
//!    [`yield_cpu`].)
//!
//! 2. Because it is waiting for an event -- a particular timer tick, an
//!    interrupt from a peripheral, a signal from another task, etc. In this
//!    case, we don't need to poll the task again *until that event occurs.*
//!
//! The OS tracks a *wake bit* per task. When this bit is set, it means that
//! the task should be polled. Each time through the outer poll loop, the OS
//! will determine which tasks have their wake bits set, *clear the wake bits*,
//! and then poll the tasks.
//!
//! (Tasks might be polled even when their bit isn't set -- this is a waste of
//! energy, but is also something that Rust `Future`s are expected to tolerate.
//! Giving the OS some slack on this dramatically simplifies the implementation.
//! However, the OS tries to poll the smallest feasible set of tasks each time
//! it polls.)
//!
//! The need to set and check wake bits is embodied by the [`Notify`] type,
//! which provides a kind of event broadcast. Tasks can subscribe to a `Notify`,
//! and when it is signaled, all subscribed tasks get their wake bits set -- so
//! they will be polled at the next opportunity.
//!
//! `Notify` is very low level -- the more pleasant abstractions of
//! [`spsc::Queue`][crate::spsc], [`mutex`][crate::mutex], and even
//! [`sleep_until`][crate::time::sleep_until]/[`sleep_for`][crate::time::sleep_for]
//! are built on top of it. However, `Notify` is the only OS facility that's
//! safe to use from interrupt service routines, making it an ideal way to wake
//! tasks when hardware events occur. See the [`Notify`] docs for an example of
//! using this to handle events from a UART.
//!
//!
//! # Idle behavior
//!
//! When no tasks have their wake bits set, the default behavior is to idle the
//! processor using the `WFI` instruction. You can override this behavior by
//! starting the scheduler with [`run_tasks_with_idle`] or (if you're using
//! preemption, below) [`run_tasks_with_preemption_and_idle`], which let you
//! substitute a custom "idle hook" to execute when no tasks are ready.
//!
//! A common use for such an idle hook is to toggle a pin to indicate CPU usage
//! on a logic analyzer, enter a vendor-specific deep-sleep mode, or feed a
//! watchdog.
//!
//!
//! # Building your own task notification mechanism
//!
//! If `Notify` doesn't meet your needs, you can use the [`wake_task_by_index`]
//! and [`wake_tasks_by_mask`] functions to explicitly wake one or more tasks.
//! Because tasks are required to tolerate spurious wakeups, both of these
//! functions are safe: spamming tasks with wakeup requests merely wastes
//! energy and time.
//!
//! Both of these functions expose the fact that the scheduler tracks wake bits
//! in a single `usize`. When waking a task with index 0 (mask `1 << 0`), we're
//! actually waking any task where `index % 32 == 0`. Very complex systems with
//! greater than 32 top-level tasks will thus experience more spurious wakeups.
//! The advantage of this "lossy" technique is that wake bit manipulation is
//! very, very cheap, and can be done entirely with processor atomic operations.
//!
//! For an example of how to do this, read the source code for `Notify` -- it's
//! written entirely in terms of public API, so if you want to do something
//! similar that `Notify` itself doesn't support, you can start by copying it.
//!
//!
//! # Adding preemption
//!
//! By default, the scheduler does not preempt task code: task poll routines are
//! run cooperatively, and ISRs are allowed only in between polls. This
//! increases interrupt response latency, because if an event occurs while
//! polling tasks, all polling must complete before the ISR is run. However, it
//! makes the program much easier to reason about, because code is simply never
//! preempted.
//!
//! Applications can change this by starting the scheduler with
//! [`run_tasks_with_preemption`] or [`run_tasks_with_preemption_and_idle`].
//! These entry points let you set a _preemption policy_, which allows ISRs
//! above some priority level to preempt task code. (Tasks still cannot preempt
//! one another.)
//!
//! The more basic [`run_tasks`] operation is written in terms of
//! [`run_tasks_with_preemption_and_idle`], so if you would like to see how to
//! convert your use of `run_tasks` to the more complex form, start by copying
//! the code from `run_tasks`.

use core::convert::Infallible;
use core::future::Future;
use core::mem;
use core::mem::MaybeUninit;
use core::ops::{Index, IndexMut};
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use portable_atomic::{AtomicUsize, Ordering};

use pin_project::pin_project;

use crate::util::Captures;

/// Trait used to implement multicore operations. This is mainly for
///
pub trait Multicore {
    /// Returns currently running core
    fn cpuid() -> usize;
    /// During scheduling, cores regularly park themselves using the `wfi` instruction.
    /// This method should perform platform-specific interrupt to wake up the given core.
    fn wake_cpu(core: usize);
}

extern "C" {
    // Allows pac crates to transparently provide cpuid implementation
    fn _lilos_cpuid() -> usize;
    fn _lilos_wake_cpu(cpu: usize);
}

/// Provide implementation of cpuid for multicore environments
#[macro_export]
macro_rules! provide_multicore_impl {
    ($name:ident) => {
        #[no_mangle]
        fn _lilos_cpuid() -> usize {
            <$name as $crate::exec::Multicore>::cpuid()
        }
        #[no_mangle]
        fn _lilos_wake_cpu(cpu: usize) {
            <$name as $crate::exec::Multicore>::wake_cpu(cpu)
        }
    };
}

/// Type used to store wake masks. [usize] is used because it guarantees to be one word sized.
/// When `multicore` feature is enabled, we split the wake mask into two 16 bit masks, one for each
/// core. [lilos::exec] implementation is written to support multicore operation transparently
pub type WakeMask = usize;

/// Atomic variant of [WakeMask]
pub type AtomicWakeMask = AtomicUsize;

/// Returns the currently running core.
///
/// Internally just calls the `_lilos_cpuid` PAC hook.
pub fn cpuid() -> usize {
    unsafe { _lilos_cpuid() }
}

/// Wakes up the given core (if it was put to sleep using `wfi`).
///
/// Internally just calls `_lilos_wake_cpu` PAC hook
pub fn wake_core(cpu: usize) {
    unsafe { _lilos_wake_cpu(cpu) }
}

/// How many cores are we running with. This constant can be changed by the `multicore` feature.
pub const CORES: usize = if cfg!(feature = "multicore") { 2 } else { 1 };

// TODO: This works for 1 or 2 cores, does not work for 4 cores.
/// How many bits in the mask are per one core
pub const CORE_NBITS: usize = 1 << (usize::BITS.ilog2() - CORES.ilog2());
/// Mask used to implement X mod (number of cores).
pub const CORE_INDEX_MASK: usize = CORE_NBITS - 1;
/// Bitmask for selecting wake bits of one core.
pub const CORE_BITS_MASK: usize = 1 << (CORE_NBITS - 1) | ((1 << (CORE_NBITS - 1)) - 1);


/// Accumulates bitmasks from wakers as they are invoked. The executor
/// atomically checks and clears this at each iteration.
static WAKE_BITS: AtomicWakeMask = AtomicWakeMask::new(0);

/// Computes the wake bit mask for the task with the given index, which is
/// equivalent to `1 << (index % USIZE_BITS)`.
const fn wake_mask_for_index(core: usize, index: usize) -> WakeMask {
    // Lossy as-cast used here because rotate_left implies a mod by small power
    // of 2 (32, 64) and `as u32` implies mod by 2**32, which doesn't change the
    // result.
    // CORE_INDEX_MASK performs index mod 32 in single core system, and mod 16 in
    // dualcore system (we have 16 bits per core in that case).
    // second shift (core << CORE_NBITS) shifts the mask into place.
    // With multicore: when waking core 0, this is a noop, when waking core 1,
    // this shifts the mask bits into the upper 16 bits.
    (1_usize << (index & CORE_INDEX_MASK) as u32) << (core * CORE_NBITS) as u32
}
/// Computes the wake mask for core. This wake mask wakes all tasks on a given core,
/// or can be used to select wake bits ONLY for a given core.
const fn wake_mask_for_core(core: usize) -> WakeMask {
    CORE_BITS_MASK << (core * CORE_NBITS) as u32
}

/// VTable for our wakers. Our wakers store a task notification bitmask in their
/// "pointer" member, and atomically OR it into `WAKE_BITS` when invoked.
static VTABLE: RawWakerVTable = RawWakerVTable::new(
    // clone
    |p| RawWaker::new(p, &VTABLE),
    // wake
    |p| wake_tasks_by_mask(p as usize),
    // wake_by_ref
    |p| wake_tasks_by_mask(p as usize),
    // drop
    |_| (),
);

/// Produces a `Waker` that will wake *at least* task `index` on `core`.
///
/// Waker will wake any task `n` where `n % (32 / CORES) == index % (32 / CORES)`.
fn waker_for_task(core: usize, index: usize) -> Waker {
    let mask = wake_mask_for_index(core, index);
    // Safety: Waker::from_raw is unsafe because bad things happen if the
    // combination of this particular pointer and the functions in the vtable
    // don't meet the Waker contract or are incompatible. In our case, our
    // vtable functions are actually entirely safe, since we're passing an
    // integer as a pointer.
    unsafe { Waker::from_raw(RawWaker::new(mask as *const (), &VTABLE)) }
}

/// Exploits our known Waker structure to extract the notification mask from a
/// Waker.
///
/// If this is applied to a Waker that isn't from this executor (specifically,
/// one not generated by `waker_for_task`), this will cause spurious and almost
/// certainly incorrect wakeups. Currently I don't feel like that risk is great
/// enough to mark this unsafe -- it can't violate *memory* safety for certain.
///
/// In practice this function compiles down to a single inlined load
/// instruction.
fn extract_mask(waker: &Waker) -> WakeMask {
    // Determine whether the pointer member comes first or second within the
    // representation of RawWaker. This is currently compile-time simplified
    // and goes away.
    //
    // Safety: we are using `transmute` to inspect the raw composition of a
    // Waker. That direction is safe -- it's a fancy version of casting a
    // pointer to an integer. Transmuting the _other_ direction would be very
    // unsafe.
    let ptr_first = unsafe {
        let (cell0, _) =
            mem::transmute::<Waker, (usize, usize)>(Waker::from_raw(RawWaker::new(1234 as *const (), &VTABLE)));
        cell0 == 1234usize
    };

    let waker: *const Waker = waker;
    // Safety: at the moment, `Waker` consists exactly of a `*const ()` and a
    // `&'static RawWakerVTable` (or equivalent pointer), and this is unlikely
    // to change. We've already verified above that we can find the parameter
    // word, which is what we care about. Extracting it cannot violate memory
    // safety, since we're just reading initialized memory.
    unsafe {
        let parts = &*(waker as *const (usize, usize));
        if ptr_first {
            parts.0
        } else {
            parts.1
        }
    }
}

/// Notifies the executor that any tasks whose wake bits are set in `mask`
/// should be polled on the next iteration.
///
/// This is a very low-level operation and is rarely what you want to use. See
/// `Notify`.
#[inline(never)]
pub fn wake_tasks_by_mask(mask: WakeMask) {
    WAKE_BITS.fetch_or(mask, Ordering::SeqCst);
    for core in 0..CORES {
        if mask & wake_mask_for_core(core) != 0 && core != cpuid(){
            wake_core(core)
        }
    }
}

/// Notifies the executor that the task with the given `index` should be polled
/// on the next iteration.
///
/// This operation isn't precise: it may wake other tasks, but it is guaranteed
/// to at least wake the desired task.
#[inline(always)]
pub fn wake_task_by_index(core: usize, index: usize) {
    wake_tasks_by_mask(wake_mask_for_index(core, index));
}

/// Structure that allows us to work on a per-core basis. This allows for certain things
/// (Like executor wake bits) to be sharded. If you have some core-specific peripherals,
/// it might be useful to store state in [PerCore], and only operate core-local copy.
///
/// Lilos is intended to be run on embedded devices, so no padding to prevent false cache sharing
/// is added.
#[derive(Debug, Clone, Copy)]
pub struct PerCore<T>(pub [T; CORES]);

impl<T> Index<usize> for PerCore<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<T> IndexMut<usize> for PerCore<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

impl<T> PerCore<T> {
    /// Executes closure on elements belonging to each core
    pub fn on_all<R>(&self, f: impl Fn(&T) -> R) -> [R; CORES] {
        let mut res: [MaybeUninit<R>; CORES] = [const { MaybeUninit::uninit() }; CORES];
        for i in 0..CORES {
            res[i].write(f(&self.0[i]));
        }
        unsafe { res.map(|r| r.assume_init()) }
    }

    /// Executes closure on elements belonging to each core.
    pub fn on_all_mut<R>(&mut self, f: impl Fn(&mut T) -> R) -> [R; CORES] {
        let mut res: [MaybeUninit<R>; CORES] = [const { MaybeUninit::uninit() }; CORES];
        for i in 0..CORES {
            res[i].write(f(&mut self.0[i]));
        }
        unsafe { res.map(|r| r.assume_init()) }
    }

    /// Executes a closure on an element belonging to a single core
    pub fn on_core<R>(&self, core: usize, f: impl Fn(&T) -> R) -> R {
        f(&self.0[core])
    }

    /// Mutably executes a closure on an element belonging to a single core
    pub fn on_core_mut<R>(&mut self, core: usize, f: impl Fn(&mut T) -> R) -> R {
        f(&mut self.0[core])
    }

    /// Executes a closure on an element belonging to current core.
    /// current core is determined using the `_lilos_cpuid` hook
    pub fn on_current<R>(&self, f: impl Fn(&T) -> R) -> R {
        self.on_core(cpuid(), f)
    }
    ///  Mutably executes a closure on an element belonging to current core.
    /// current core is determined using the `_lilos_cpuid` hook
    pub fn on_current_mut<R>(&mut self, f: impl Fn(&mut T) -> R) -> R {
        self.on_core_mut(cpuid(), f)
    }
}

/// Allows hooking into the scheduling process.
///
/// A single [Hooks] instance will be shared between all cores. If you need
/// to store per-core data, use a [PerCore] wrapper.
pub trait Hooks {
    /// Run before scheduler is started on `core`
    fn init_core(&self, core: usize);
    /// Run before we poll woken up tasks on `core`
    fn before_poll(&self, core: usize);
    /// Run before we poll woken up tasks on `core`
    fn after_poll(&self, core: usize);
}

/// Polls `future` in a context where the `Waker` will signal the task with
/// index `index`.
fn poll_task(index: usize, future: Pin<&mut dyn Future<Output = Infallible>>) {
    #![allow(unreachable_patterns)]
    match future.poll(&mut Context::from_waker(&waker_for_task(cpuid(), index))) {
        Poll::Pending => (),
        Poll::Ready(never) => match never {},
    }
}

/// Selects an interrupt control strategy for the scheduler.
///
/// This is used as an argument to [`run_tasks_with_preemption`] and
/// [`run_tasks_with_preemption_and_idle`].
#[derive(Copy, Clone, Debug)]
pub enum Interrupts {
    /// Use PRIMASK to completely disable interrupts while task code is running.
    Masked,
    /// Use BASEPRI to mask interrupts of the given priority and lower (i.e.
    /// numerically greater, since 0 is "most important" and `0xFF` is "least
    /// important").
    ///
    /// `Filtered(0)` is basically equivalent to `Masked`.
    ///
    /// Be careful when using this: note that the "next" priority past 0 is
    /// vendor-dependent, because implementations can choose not to implement
    /// some of the least significant interrupt priority bits to save space on
    /// the chip. If what you want is to divide interrupts into a set that can
    /// preempt everything (often timer interrupts) and a set that cannot,
    /// `Filtered(0x80)` should work.
    ///
    /// When using the `systick` feature, note that the SysTick IRQ is
    /// configured at priority 0 by default, so setting this to `Filtered(0x80)`
    /// is enough to stop losing ticks during long-running sequences. (You can
    /// adjust this priority in the NVIC.)
    ///
    /// This is not available on ARMv6-M, which lacks the `BASEPRI` feature.
    #[cfg(lilos_has_basepri)]
    Filtered(u8),
}

impl Interrupts {
    fn scope<R>(self, body: impl FnOnce() -> R) -> R {
        let r = match self {
            Interrupts::Masked => {
                let prev = cortex_m::register::primask::read();
                cortex_m::interrupt::disable();

                let r = body();

                if prev == cortex_m::register::primask::Primask::Active {
                    // Safety: interrupts were just on, so this won't compromise
                    // memory safety.
                    unsafe {
                        cortex_m::interrupt::enable();
                    }
                }

                r
            }
            #[cfg(lilos_has_basepri)]
            Interrupts::Filtered(priority) => {
                let prev = cortex_m::register::basepri::read();
                cortex_m::register::basepri_max::write(priority);

                let r = body();

                // Safety: just restoring state
                unsafe {
                    cortex_m::register::basepri::write(prev);
                }

                r
            }
        };

        // Make sure newly-enabled interrupt handlers fire.
        cortex_m::asm::isb();

        r
    }
}

/// Runs the given futures forever, sleeping when possible. Each future acts as
/// a task, in the sense of `core::task` -- that is, it is a top-level entity
/// that can wake up separately from the other tasks.
///
/// Task futures must not ever resolve/complete -- they need to be infinite
/// loops or equivalent. Due to limitations in the language, this requires their
/// return type to be [`Infallible`], which is an awkward way of writing `!`.
///
/// Not all tasks are polled every time through the loop. On the first
/// iteration, only the tasks with a corresponding bit set in `initial_mask` are
/// polled; on subsequent passes, only tasks awoken by the *previous* iteration
/// are called.
///
/// Any time polling completes with *no* tasks awoken, code will never run again
/// unless an interrupt handler wakes tasks using `Notify`. And so, when we
/// detect this condition, we use the `WFI` instruction to idle the processor
/// until an interrupt arrives. This has the advantages of using less power and
/// having more predictable response latency than spinning. If you'd like to
/// override this behavior, see [`run_tasks_with_idle`].
pub fn schedule(
    futures: &mut [Pin<&mut dyn Future<Output = Infallible>>],
    hooks: &[&(dyn Hooks)],
    initial_mask: WakeMask,
) -> ! {
    // Safety: we're passing Interrupts::Masked, the always-safe option
    unsafe {
        schedule_with_preemption_and_idle(futures, hooks, initial_mask, Interrupts::Masked, || {
            cortex_m::asm::wfi();
            // This works around an undocumented erratum on STM32 processors
            // when WFI is set to go to "Sleep" level, and a debug agent has
            // set the DBGMCU bits to cause clocks to continue to run during
            // sleep. In this situation, it appears that the pipeline state
            // after the WFI can be corrupted in the specific case where the
            // WFI happens _without_ an interrupt service routine occurring
            // (i.e. our default configuration of interrupts masked). An ISB
            // appears to fix it, independent of alignment etc.
            //
            // Hard to tell, though, since this isn't in the errata sheet.
            //
            // On non-STM32 Cortex processors this will cost a few cycles.
            cortex_m::asm::isb();
        })
    }
}

/// Extended version of `run_tasks` that replaces the default idle behavior
/// (sleeping until the next interrupt) with code of your choosing.
///
/// If you would like the processor to sleep when idle, you will need to call
/// WFI yourself from within the implementation of `idle_hook`.
///
/// See [`run_tasks`] for more details.
pub fn schedule_with_idle(
    futures: &mut [Pin<&mut dyn Future<Output = Infallible>>],
    hooks: &mut [&(dyn Hooks)],
    initial_mask: WakeMask,
    idle_hook: impl FnMut(),
) -> ! {
    // Safety: we're passing Interrupts::Masked, the always-safe option
    unsafe { schedule_with_preemption_and_idle(futures, hooks, initial_mask, Interrupts::Masked, idle_hook) }
}

/// Extended version of `run_tasks` that configures the scheduler with a custom
/// interrupt policy.
///
/// Passing `Interrupts::Masked` here gets the same behavior as `run_tasks`.
///
/// Passing `Interrupts::Filtered(p)` causes the scheduler to only disable
/// interrupts with priority equal to or numerically greater than `p`. See the
/// docs for the [`Interrupts`] type for more details.
///
/// # Safety
///
/// This can be used safely as long as ISRs and task code that share data
/// structures use appropriate critical sections. If the only preemption you're
/// enabling is the OS's built-in SysTick ISR, it's intrinsically safe and you
/// can meet this contract trivially -- just make sure you've set the priority
/// levels for your other interrupts appropriately!
///
/// Note that none of the top-level functions in this module are safe to use
/// from a custom ISR. Only operations on types that are specifically described
/// as being ISR safe, such as `Notify::notify`, can be used from ISRs.
pub unsafe fn schedule_with_preemption(
    futures: &mut [Pin<&mut dyn Future<Output = Infallible>>],
    hooks: &mut [&(dyn Hooks)],
    initial_mask: WakeMask,
    interrupts: Interrupts,
) -> ! {
    // Safety: this is safe if our own contract is upheld.
    unsafe { schedule_with_preemption_and_idle(futures, hooks, initial_mask, interrupts, || cortex_m::asm::wfi()) }
}

/// Extended version of `run_tasks` that configures the scheduler with a custom
/// interrupt policy and idle hook. See [`run_tasks`] for more information about
/// the basic behavior.
///
/// Passing `Interrupts::Masked` here gets the same behavior as
/// `run_tasks_with_idle`.
///
/// Passing `Interrupts::Filtered(p)` causes the scheduler to only disable
/// interrupts with priority equal to or numerically greater than `p`. This can
/// be used to ensure that the OS systick ISR (priority 0, by default) can
/// preempt long-running tasks.
///
/// # Safety
///
/// This can be used safely as long as ISRs and task code that share data
/// structures use appropriate critical sections. If the only preemption you're
/// enabling is the OS's built-in SysTick ISR, it's intrinsically safe and you
/// can meet this contract trivially -- just make sure you've set the priority
/// levels for your other interrupts appropriately!
///
/// Note that none of the top-level functions in this module are safe to use
/// from a custom ISR. Only operations on types that are specifically described
/// as being ISR safe, such as `Notify::notify`, can be used from ISRs.
pub unsafe fn schedule_with_preemption_and_idle(
    futures: &mut [Pin<&mut dyn Future<Output = Infallible>>],
    hooks: &[&(dyn Hooks)],
    initial_mask: WakeMask,
    interrupts: Interrupts,
    mut idle_hook: impl FnMut(),
) -> ! {
    let current_core = cpuid();
    let current_core_mask = wake_mask_for_core(current_core);

    WAKE_BITS.or(initial_mask & current_core_mask, Ordering::SeqCst);

    let futures_ptr: *mut [Pin<&mut dyn Future<Output = Infallible>>] = futures;
    let futures_ptr: *mut [Pin<*mut dyn Future<Output = Infallible>>] = futures_ptr as _;

    unsafe { TASK_FUTURES[current_core] = Some(futures_ptr); }

    hooks.iter().for_each(|hook| hook.init_core(current_core));

    loop {
        interrupts.scope(|| {
            // Run scheduler hooks
            hooks.iter().for_each(|hook| hook.before_poll(current_core));
            // Fetch the old mask, and remove our activated bits.
            let mask = WAKE_BITS.fetch_and(!current_core_mask, Ordering::SeqCst);
            for (i, f) in futures.iter_mut().enumerate() {
                if mask & wake_mask_for_index(current_core, i) != 0 {
                    poll_task(i, f.as_mut());
                }
            }
            hooks.iter().for_each(|hook| hook.after_poll(current_core));
            // If none of the futures woke each other, we're relying on an
            // interrupt to set bits -- so we can sleep waiting for it.
            if WAKE_BITS.load(Ordering::SeqCst) & current_core_mask  == 0 {
                idle_hook();
            }
        });

        // Now interrupts are enabled for a brief period before diving back in.
        // Note that we allow interrupt-wake even when some wake bits are set;
        // this prevents ISR starvation by polling tasks.
    }
}

/// This `static` variable is only written by the OS, and never read. It exists
/// to be observed from a debugger.
///
/// Without this, it's really hard to figure out where the official list of
/// tasks is. We don't put the list of tasks *itself* in a `static` because we
/// can't predict its size (it's up to the client). We don't use this `static`
/// as _our_ sense of the task list because, well, we don't have to.
///
/// The fact that we don't _read_ this variable dodges most lifetime/safety
/// issues.
///
/// Note that the `#[used]` annotation is load-bearing here -- without it the
/// compiler will happily throw the variable away, confusing the debugger.
#[used]
static mut TASK_FUTURES: PerCore<Option<*mut [Pin<*mut dyn Future<Output = Infallible>>]>> = PerCore([None; CORES]);

/// Constant that can be passed to `run_tasks` and `wake_tasks_by_mask` to mean
/// "all tasks."
pub const ALL_TASKS: usize = !0;

/// A lightweight task notification scheme that can be used to safely route
/// events from interrupt handlers to task code.
///
/// This is the lowest level inter-task communication type in `lilos`, and is
/// appropriate if you're building your own higher-level mechanism, or if you
/// want to signal events from interrupt service routines.
///
/// Any number of tasks can [`subscribe`][Notify::subscribe] to a `Notify`. When
/// [`notify`][Notify::notify] is called on it, all those tasks will be awoken
/// (i.e. their `Waker` will be triggered so that they become eligible for
/// polling), and their subscription is atomically ended. Because spurious wakes
/// are possible, the subscribed tasks may wake before `notify` is called. To
/// check if the desired condition has truly occurred, you'll generally want to
/// call [`until`][Notify::until] instead of using `subscribe` directly.
///
/// A `Notify` is very small (the size of a pointer), so feel free to create as
/// many as you like.
///
/// It is safe to call `notify` from an ISR, so this is the usual method by
/// which interrupt handlers inform task code of events. Normally a `Notify`
/// used in this way is stored in a `static`:
///
/// ```ignore
/// static EVENT: Notify = Notify::new();
/// ```
///
/// You can use that style of `static` `Notify` to sleep waiting for interrupt
/// conditions in async code. Here's an example for a made-up but typical UART
/// driver:
///
/// ```ignore
/// /// Event signal for waking task(s) when data arrives.
/// static RX_NOT_EMPTY: Notify = Notify::new();
///
/// /// UART interrupt handler.
/// #[interrupt]
/// fn UART() {
///     let uart = get_uart_peripheral_somehow();
///
///     let control = uart.control.read();
///     let status = uart.status.read();
///
///     if control.rx_irq_enabled() && status.rx_not_empty() {
///         // Shut off the interrupt source to keep this from reoccurring.
///         uart.control.modify(|_, w| w.rx_irq_enabled().clear());
///         // Wake up the task that requested this.
///         RX_NOT_EMPTY.notify();
///     }
/// }
///
/// async fn uart_recv(uart: &Uart) -> u8 {
///     // Enable the rx data interrupt so we get notified.
///     uart.control.modify(|_, w| w.rx_irq_enabled().set());
///     // Listen for data, using a predicate to filter out spurious wakes.
///     RX_NOT_EMPTY.until(|| uart.status.read().rx_not_empty()).await;
///
///     UART.data.read()
/// }
/// ```
///
/// # Waker coalescing
///
/// A `Notify` collects any number of task `Waker`s into a fixed-size structure
/// without heap allocation. It does this by coalescing the `Waker`s such that
/// they may become *imprecise*: firing the waker for task N may also spuriously
/// wake task M, if `M % 32 == N % 32`. (Implementation-wise, this is a matter
/// of collecting a wake bits mask from the wakers using secret knowledge.)
///
/// While this is often not the *ideal* strategy, it has the advantage that it
/// can be built up cheaply and torn down atomically from interrupt context.
/// (Contrast with e.g. a list of waiting tasks, which is more precise but
/// harder to get right and more expensive at runtime.)
///
/// For more nuanced use cases -- precisely waking a single task, waking tasks
/// in a particular order, waking groups of tasks together, etc. -- see the
/// [`list`][crate::list] module -- though note that `list` cannot be used from
/// an ISR.
#[derive(Debug, Default)]
pub struct Notify {
    mask: AtomicWakeMask,
}

impl Notify {
    /// Creates a new `Notify` with no tasks waiting.
    pub const fn new() -> Self {
        Self {
            mask: AtomicWakeMask::new(0),
        }
    }

    /// Adds the `Waker` to the set of waiters.
    ///
    /// This is a low-level operation. For using a `Notify` in practice, you
    /// probably want [`until`][Notify::until] instead.
    pub fn subscribe(&self, waker: &Waker) {
        self.mask.fetch_or(extract_mask(waker), Ordering::SeqCst);
    }

    /// Wakes tasks, at least all those whose waiters have been passed to
    /// `subscribe` since the last `notify`, possibly more.
    ///
    /// As with any wake, this makes the tasks eligible for polling in the next
    /// iteration of the executor, and does not cause any code to run
    /// immediately.
    pub fn notify(&self) {
        wake_tasks_by_mask(self.mask.swap(0, Ordering::SeqCst))
    }

    /// Waits for a condition to become true, checking only when signaled by
    /// this `Notify`. This is generally the right way to synchronize with an
    /// event through a `Notify`.
    ///
    /// `until` repeatedly calls the `cond` function you provide, completing
    /// when it "passes" (see below). In between calls, it subscribes to this
    /// `Notify`, so that the task won't waste time checking `cond` when no
    /// event has occurred.
    ///
    /// This is appropriate if you know that any change to `cond`'s result will
    /// be preceded by some task calling `self.notify()`.
    ///
    /// The meaning of `cond` "passing" is defined by the [`TestResult`] trait.
    ///
    /// - In the easiest case, `cond` should return a `bool`. `until` will
    ///   resolve when `cond` returns `true`.
    ///
    /// - For more interesting use cases, `cond` can also return an `Option<T>`.
    ///   In this case, `until` will resolve when `cond` returns `Some(value)`,
    ///   producing `value`.
    ///
    /// # Cancellation
    ///
    /// **Cancel safety:** Strict, if no data is moved into `cond`.
    ///
    /// Dropping this future will drop `cond`, and may leave the current task
    /// subscribed to `self` (meaning one potential spurious wakeup in the
    /// future is possible).
    ///
    /// If creating `cond` consumes some data, dropping the future produced by
    /// `until` will drop that data, creating a potential for data loss on
    /// cancellation. In practice it's hard to do this by accident; it usually
    /// requires one of the following patterns:
    ///
    /// 1. Writing `cond` as an explicit `move` closure. In this case you can
    ///    improve cancel safety by avoiding the move, if possible.
    /// 2. Passing a closure you received as `cond` instead of making a new one.
    ///    In this case, consider passing the closure by reference.
    pub fn until<F, T: TestResult>(&self, cond: F) -> Until<'_, F>
    where
        F: FnMut() -> T,
    {
        Until { cond, notify: self }
    }

    /// Waits for a condition to become true, in a way that tolerates race
    /// conditions (e.g. preempting interrupt handlers).
    ///
    /// This is a variation on [`until`][Notify::until] that is slightly more
    /// expensive, but won't miss events if the `Notify` may be signaled by a
    /// preempting interrupt handler. The only difference between this and
    /// `until` is that `until_racy` subscribes to the `Notify` _before_
    /// checking the condition `cond`. This means if `cond` is immediately true,
    /// you may see a spurious wakeup down the line (since the `subscribe`
    /// cannot be undone). This will waste a bit of CPU but should have no other
    /// ill effects.
    ///
    /// See [`until`][Notify::until] for more details.
    ///
    /// # Cancellation
    ///
    /// **Cancel safety:** Strict, if no data is moved into `cond`.
    ///
    /// Dropping this future will drop `cond`, and may leave the current task
    /// subscribed to `self` (meaning one potential spurious wakeup in the
    /// future is possible).
    ///
    /// If creating `cond` consumes some data, dropping the future produced by
    /// `until` will drop that data, creating a potential for data loss on
    /// cancellation. In practice it's hard to do this by accident; it usually
    /// requires one of the following patterns:
    ///
    /// 1. Writing `cond` as an explicit `move` closure. In this case you can
    ///    improve cancel safety by avoiding the move, if possible.
    /// 2. Passing a closure you received as `cond` instead of making a new one.
    ///    In this case, consider passing the closure by reference.
    pub fn until_racy<F, T: TestResult>(&self, cond: F) -> UntilRacy<'_, F>
    where
        F: FnMut() -> T,
    {
        UntilRacy { cond, notify: self }
    }

    /// Subscribes to `notify` and blocks until the task is awoken. This may
    /// produces spurious wakeups, and is appropriate only when you're checking
    /// some condition separately. Otherwise, use [`until`][Notify::until].
    ///
    /// # Cancellation
    ///
    /// **Cancel safety:** Strict.
    ///
    /// Dropping this future will leave the current task subscribed to `self`
    /// (meaning one potential spurious wakeup in the future is possible).
    pub fn until_next(&self) -> impl Future<Output = ()> + Captures<&'_ Self> {
        let mut setup = false;
        self.until(move || mem::replace(&mut setup, true))
    }
}

/// Trait implemented by things that indicate success or failure, to be used
/// with [`Notify::until`] and friends.
///
/// In practice this is `bool` (if there's no output associated with success) or
/// `Option<T>` (if there is).
///
/// This is used by the various polling functions in this module.
pub trait TestResult {
    /// Type of content produced on success.
    type Output;
    /// Converts `self` into an `Option` that is `Some` on success, `None` on
    /// failure.
    fn into_test_result(self) -> Option<Self::Output>;
}

impl TestResult for bool {
    type Output = ();
    fn into_test_result(self) -> Option<Self::Output> {
        if self {
            Some(())
        } else {
            None
        }
    }
}

impl<T> TestResult for Option<T> {
    type Output = T;
    fn into_test_result(self) -> Option<Self::Output> {
        self
    }
}

/// Internal future type used to implement `Notify::until`. This makes it
/// much easier to recognize the future in a debugger.
#[derive(Debug)]
#[must_use = "futures do nothing unless you `.await` or poll them"]
#[pin_project]
pub struct Until<'n, F> {
    cond: F,
    notify: &'n Notify,
}

impl<F, T> Future for Until<'_, F>
where
    F: FnMut() -> T,
    T: TestResult,
{
    type Output = T::Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let p = self.project();
        if let Some(x) = (p.cond)().into_test_result() {
            Poll::Ready(x)
        } else {
            p.notify.subscribe(cx.waker());
            Poll::Pending
        }
    }
}

/// Internal future type used to implement `Notify::until_racy`. This makes
/// it much easier to recognize the future in a debugger.
#[derive(Debug)]
#[must_use = "futures do nothing unless you `.await` or poll them"]
#[pin_project]
pub struct UntilRacy<'n, F> {
    cond: F,
    notify: &'n Notify,
}

impl<F, T> Future for UntilRacy<'_, F>
where
    F: FnMut() -> T,
    T: TestResult,
{
    type Output = T::Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let p = self.project();
        p.notify.subscribe(cx.waker());
        if let Some(x) = (p.cond)().into_test_result() {
            Poll::Ready(x)
        } else {
            Poll::Pending
        }
    }
}

/// Returns a future that will be pending exactly once before resolving.
///
/// This can be used to give up CPU to any other tasks that are currently ready
/// to run, and then take it back without waiting for an event.
///
/// # Cancellation
///
/// **Cancel safety:** Strict.
///
/// Dropping this future does nothing in particular.
pub fn yield_cpu() -> impl Future<Output = ()> {
    YieldCpu { polled: false }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct YieldCpu {
    polled: bool,
}

impl Future for YieldCpu {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if mem::replace(&mut self.polled, true) {
            Poll::Ready(())
        } else {
            // Ensure that we get called next round.
            cx.waker().wake_by_ref();

            Poll::Pending
        }
    }
}
