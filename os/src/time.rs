// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Timekeeping using the SysTick Timer.
//!
//! **Note:** this entire module is only available if the `systick` feature is
//! present; it is on by default.
//!
//! The OS uses the Cortex-M SysTick Timer to maintain a monotonic counter
//! recording the number of microseconds ("ticks") since boot. This module
//! provides ways to read that timer, and also to arrange for tasks to be woken
//! at specific times (such as [`sleep_until`] and [`sleep_for`]).
//!
//! To use this facility in an application, you need to call
//! [`initialize_sys_tick`] to inform the OS of the system clock speed.
//! Otherwise, no operations in this module will work properly.
//!
//! You can get the value of tick counter using [`TickTime::now`].
//!
//! # Types for describing time
//!
//! This module uses three main types for describing time, in slightly different
//! roles.
//!
//! `TickTime` represents a specific point in time, measured as a number of
//! ticks since boot (or, really, since the executor was started). It's a
//! 64-bit count of microseconds, which means it overflows every 584 thousand
//! years. This lets us ignore overflows in timestamps, making everything
//! simpler. `TickTime` is analogous to `std::time::Instant` from the Rust
//! standard library.
//!
//! `Micros` represents a relative time interval in microseconds. This uses the
//! same representation as `TickTime`, so adding them together is cheap.
//!
//! `core::time::Duration` is similar to `Micros` but with a lot more bells and
//! whistles. It's the type used to measure time intervals in the Rust standard
//! library. It can be used with most time-related API in the OS, but you might
//! not want to do so on a smaller CPU: `Duration` uses a mixed-number-base
//! format internally that means almost all operations require a 64-bit multiply
//! or divide. On machines lacking such instructions, this can become quite
//! costly (in terms of both program size and time required).
//!
//! Cases where the OS won't accept `Duration` are mostly around things like
//! sleeps, where the operation will always be performed in units of whole
//! ticks, so being able to pass (say) nanoseconds is misleading.
//!
//! # Imposing a timeout on an operation
//!
//! If you want to stop a concurrent process if it's not done by a certain time,
//! see the [`with_deadline`] function (and its relative friend,
//! [`with_timeout`]). These let you impose a deadline on any future, such that
//! if it hasn't resolved by a certain time, it will be dropped (cancelled).
//!
//! # Fixing "lost ticks"
//!
//! If the longest sequence in your application between any two `await` points
//! takes less than a microsecond, the standard timer configuration will work
//! fine and keep reliable time.
//!
//! However, if you sometimes need to do more work than that -- or if you're
//! concerned you might do so by accident due to a bug -- the systick IRQ can be
//! configured to preempt task code. The OS is designed to handle this safely.
//! For more information, see
//! [`run_tasks_with_preemption`][crate::exec::run_tasks_with_preemption].
//!
//! # Getting higher precision
//!
//! For many applications, microsecond are a fine unit of time, but sometimes
//! you need something more precise. Currently, the easiest way to do this is to
//! enlist a different hardware timer. The `time` module has no special
//! privileges that you can't make use of, and adding your own alternate
//! timekeeping module is explictly supported in the design (this is why the
//! `"systick"` feature exists).
//!
//! This can also be useful on processors like the Nordic nRF52 series, where
//! the best sleep mode to use when idling the CPU also stops the systick timer.
//! On platforms like that, the systick isn't useful as a monotonic clock, and
//! you'll want to use some other vendor-specific timer.
//!
//! Currently there's no example of how to do this in the repo. If you need
//! this, please file an issue.

use core::future::Future;
use core::mem::MaybeUninit;
use core::ops::{Add, AddAssign};
use core::pin::Pin;
use core::ptr::{addr_of, addr_of_mut};
use core::task::{Context, Poll};
use core::time::Duration;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::exception;
use lilos_list::List;
use pin_project::pin_project;
use crate::exec::{cpuid, PerCore, CORES};

struct State {
    tick: AtomicU32,
    epoch: AtomicU32,
    incr: AtomicU32,
    list_ready: AtomicBool,
    timer_list: MaybeUninit<List<TickTime>>
}

static mut STATE: PerCore<State> = PerCore([const {
    State {
        tick: AtomicU32::new(0),
        epoch: AtomicU32::new(0),
        incr: AtomicU32::new(0),
        list_ready: AtomicBool::new(false),
        timer_list: MaybeUninit::uninit(),
    }
}; CORES]);

// /// Bottom 32 bits of the tick counter. Updated by ISR.
// static TICK: PerCore<AtomicU32> =  PerCore(const {} AtomicU32::new(0));
// /// Top 32 bits of the tick counter. Updated by ISR.
// static EPOCH: PerCore<AtomicU32> = PerCore([const { AtomicU32::new(0) }; CORES]);
// /// How much to increment the tick counter by.
// static INCR: PerCore<AtomicU32> = PerCore([const { AtomicU32::new(0) }; CORES]);
//
// /// Tracks whether the timer list has been initialized.
// static TIMER_LIST_READY: PerCore<AtomicBool> = AtomicBool::new(false);
//
// /// Storage for the timer list.
// static mut TIMER_LIST: MaybeUninit<List<TickTime>> = MaybeUninit::uninit();

/// Panics if called from an interrupt service routine (ISR). This is used to
/// prevent OS features that are unavailable to ISRs from being used in ISRs.
#[cfg(feature = "systick")]
fn assert_not_in_isr() {
    let psr_value = cortex_m::register::apsr::read().bits();
    // Bottom 9 bits are the exception number, which are 0 in Thread mode.
    if psr_value & 0x1FF != 0 {
        panic!();
    }
}

/// Nabs a reference to the global timer list.
///
/// # Preconditions
///
/// - Must not be called from an interrupt.
/// - Must only be called once the timer list has been initialized, which is to
///   say, from within a task.
#[cfg(feature = "systick")]
pub(crate) fn get_timer_list() -> Pin<&'static List<TickTime>> {
    // Prevent this from being used from interrupt context.

    assert_not_in_isr();

    let state = unsafe { &STATE[cpuid()] };

    // Ensure that the timer list has been initialized.
    cheap_assert!(state.list_ready.load(Ordering::Acquire));

    // Since we know we're not running concurrently with the scheduler setup, we
    // can freely vend pin references to the now-immortal timer list.
    //
    // Safety: &mut references to TIMER_LIST have stopped after initialization.
    let list_ref = unsafe { &*addr_of!(state.timer_list) };
    // Safety: we know the list has been initialized because we checked
    // TIMER_LIST_READY, above. We also know that the list trivially meets the
    // pin criterion, since it's immortal and always referenced by shared
    // reference at this point.
    unsafe { Pin::new_unchecked(list_ref.assume_init_ref()) }
}

/// Implements executor hooks to aallow asynchronous delays using the systick timer. Accuracy should be in milliseconds.
///
/// WARNING: This will call [cortex_m::Peripherals::steal] on initialization.
/// If you use [cortex_m::Peripherals::take] afterward, your application will crash
#[derive(Debug)]
pub struct SysTick { clock_hz: u32 }

impl SysTick {
    /// Initializes systick timer, and returns [crate::exec::Hooks] that should be
    /// provided to the executor in order to drive the timer scheduling.
    ///
    /// SysTick is not a reliable timing mechanism when the core is put into deep sleep
    /// If you can, prefer using the individual chip timing mechanisms.
    pub fn initialize(clock_hz: u32) -> Self {
        SysTick { clock_hz }
    }
}

impl crate::exec::Hooks for SysTick {
    fn init_core(&self, _: usize) {
        let state = unsafe { &mut STATE[cpuid()] };
        let mut syst = unsafe { cortex_m::Peripherals::steal().SYST };
        let cycles = self.clock_hz / 1_000;
        state.incr.store(1_000, Ordering::Relaxed);

        syst.disable_counter();
        syst.set_clock_source(SystClkSource::Core);

        syst.set_reload(cycles - 1);
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();

        let already_initialized = state.list_ready.swap(true, Ordering::SeqCst);
        // Catch any attempt to do this twice. Would doing this twice be bad?
        // Not necessarily. But it would be damn suspicious.
        cheap_assert!(!already_initialized);

        // Safety: by successfully flipping the initialization flag, we know
        // we can do this without aliasing; being in a single-threaded context
        // right now means we can do it without racing.
        let timer_list = unsafe { &mut *addr_of_mut!(state.timer_list) };

        // Initialize the list node itself.
        timer_list.write(List::new());
    }

    fn before_poll(&self, _: usize) {
        // Scan for any expired timers.
        let now = TickTime::now();
        get_timer_list().wake_while(|&t| t <= now);
    }

    fn after_poll(&self, _: usize) {}
}

/// Represents a moment in time by the value of the system tick counter.
/// System-specific analog of `std::time::Instant`.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Default)]
pub struct TickTime(u64);

impl TickTime {
    /// Retrieves the current value of the tick counter.
    pub fn now() -> Self {
        let state = unsafe { &STATE[cpuid()] };
        // This loop will only repeat if e != e2, which means we raced the
        // systick ISR. Since that ISR only occurs once per millisecond, this
        // loop should repeat at most twice.
        loop {
            let e = state.epoch.load(Ordering::SeqCst);
            let t = state.tick.load(Ordering::SeqCst);
            let e2 = state.epoch.load(Ordering::SeqCst);
            if e == e2 {
                break TickTime(((e as u64) << 32) | (t as u64));
            }
        }
    }

    /// Constructs a `TickTime` value describing a certain number of
    /// milliseconds since the executor booted.
    pub fn from_micros_since_boot(m: u64) -> Self {
        Self(m)
    }

    /// Subtracts this time from an earlier time, giving the `Duration` between
    /// them.
    ///
    /// # Panics
    ///
    /// If this time is not actually `>= earlier`.
    pub fn duration_since(self, earlier: TickTime) -> Duration {
        Duration::from_micros(self.micros_since(earlier).0)
    }

    /// Subtracts this time from an earlier time, giving the amount of time
    /// between them measured in `Micros`.
    ///
    /// # Panics
    ///
    /// If this time is not actually `>= earlier`.
    pub fn micros_since(self, earlier: TickTime) -> Micros {
        Micros(self.0.checked_sub(earlier.0).unwrap())
    }

    /// Checks the clock to determine how much time has elapsed since the
    /// instant recorded by `self`.
    pub fn elapsed(self) -> Micros {
        Self::now().micros_since(self)
    }

    /// Checks the clock to determine how much time has elapsed since the
    /// instant recorded by `self`. Convenience version that returns the result
    /// as a `Duration`.
    pub fn elapsed_duration(self) -> Duration {
        Duration::from_micros(self.elapsed().0)
    }

    /// Adds some microseconds to `self`, checking for overflow. Note that since
    /// we use 64 bit ticks, overflow is unlikely in practice.
    pub fn checked_add(self, micros: Micros) -> Option<Self> {
        self.0.checked_add(micros.0).map(TickTime)
    }

    /// Subtracts some microseconds from `self`, checking for overflow. Overflow
    /// can occur if `micros` is longer than the time from boot to `self`.
    pub fn checked_sub(self, micros: Micros) -> Option<Self> {
        self.0.checked_sub(micros.0).map(TickTime)
    }
}

/// Add a `Duration` to a `Ticks` with normal `+` overflow behavior (i.e.
/// checked in debug builds, optionally not checked in release builds).
impl Add<Duration> for TickTime {
    type Output = Self;
    fn add(self, other: Duration) -> Self::Output {
        TickTime(self.0 + other.as_micros() as u64)
    }
}

impl AddAssign<Duration> for TickTime {
    fn add_assign(&mut self, other: Duration) {
        self.0 += other.as_micros() as u64
    }
}

impl From<TickTime> for u64 {
    fn from(t: TickTime) -> Self {
        t.0
    }
}

/// A period of time measured in microseconds.
///
/// This plays a role similar to `core::time::Duration` but is designed to be
/// cheaper to use. In particular, as of this writing, `Duration` insists on
/// converting times to and from a Unix-style (seconds, nanoseconds)
/// representation internally. This means that converting to or from any simple
/// monotonic time -- even in nanoseconds! -- requires a 64-bit division or
/// multiplication. Many useful processors, such as Cortex-M0, don't have 32-bit
/// division, much less 64-bit division.
///
/// `Micros` wraps a `u64` and records a number of microseconds. Since
/// microseconds are `lilos`'s unit used for internal timekeeping, this ensures
/// that a `Micros` can be used for any deadline or timeout computation without
/// any unit conversions or expensive arithmetic operations.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Default)]
pub struct Micros(pub u64);

/// Adds a number of microseconds to a `TickTime` with normal `+` overflow
/// behavior (i.e. checked in debug builds, optionally not checked in release
/// builds).
impl Add<Micros> for TickTime {
    type Output = Self;
    fn add(self, other: Micros) -> Self::Output {
        TickTime(self.0 + other.0)
    }
}

/// Adds a number of microseconds to a `TickTime` with normal `+=` overflow
/// behavior (i.e. checked in debug builds, optionally not checked in release
/// builds).
impl AddAssign<Micros> for TickTime {
    fn add_assign(&mut self, other: Micros) {
        self.0 += other.0;
    }
}

impl From<Micros> for u64 {
    fn from(x: Micros) -> Self {
        x.0
    }
}

impl From<u64> for Micros {
    fn from(x: u64) -> Self {
        Self(x)
    }
}

/// Sleeps until the system time is equal to or greater than `deadline`.
///
/// More precisely, `sleep_until(d)` returns a `Future` that will poll as
/// `Pending` until `TickTime::now() >= deadline`; then it will poll `Ready`.
///
/// If `deadline` is already in the past, this will instantly become `Ready`.
///
/// Other tools you might consider:
///
/// - If you want to sleep for a relative time interval, consider [`sleep_for`].
/// - If you want to make an action periodic by sleeping in a loop,
///   [`PeriodicGate`] helps avoid common mistakes that cause timing drift and
///   jitter.
/// - If you want to impose a deadline/timeout on async code, see
///   [`with_deadline`].
///
/// # Preconditions
///
/// This can only be used within a task.
///
/// # Cancellation
///
/// **Cancel safety:** Strict.
///
/// Dropping this future does nothing in particular.
pub async fn sleep_until(deadline: TickTime) {
    if TickTime::now() >= deadline {
        return;
    }

    // Insert our node into the pending timer list. If we get cancelled, the
    // node will detach itself as it's being dropped.
    get_timer_list().join(deadline).await
}

/// Sleeps until the system time has increased by `d`.
///
/// More precisely, `sleep_for(d)` captures the system time, `t`, and returns a
/// `Future` that will poll as `Pending` until `TickTime::now() >= t + d`; then
/// it will poll `Ready`.
///
/// If `d` is 0, this will instantly become `Ready`.
///
/// `d` can be any type that can be added to a `TickTime`, which in practice
/// means either [`Micros`] or [`Duration`].
///
/// This function is a thin wrapper around [`sleep_until`]. See that function's
/// docs for examples, details, and alternatives.
///
/// # Cancellation
///
/// **Cancel safety:** Strict.
///
/// Dropping this future does nothing in particular.
pub fn sleep_for<D>(d: D) -> impl Future<Output = ()>
where
    TickTime: Add<D, Output = TickTime>,
{
    sleep_until(TickTime::now() + d)
}

/// Alters a future to impose a deadline on its completion.
///
/// Concretely,
/// - The output type is changed from `T` to `Option<T>`.
/// - If the future resolves on any polling that starts before `deadline`, its
///   result will be produced, wrapped in `Some`.
/// - If poll is called at or after `deadline`, the future resolves to `None`.
///
/// The wrapped future is _not_ immediately dropped if the timeout expires. It
/// will be dropped when you drop the wrapped version. Under normal
/// circumstances this happens automatically, e.g. if you do:
///
/// ```ignore
/// with_deadline(MY_DEADLINE, some_operation()).await;
/// ```
///
/// In this case, `await` drops the future as soon as it resolves (as always),
/// which means the nested `some_operation()` future will be promptly dropped
/// when we notice that the deadline has been met or exceeded.
pub fn with_deadline<F>(deadline: TickTime, code: F) -> impl Future<Output = Option<F::Output>>
where
    F: Future,
{
    TimeLimited {
        limiter: sleep_until(deadline),
        process: code,
    }
}

/// Alters a future to impose a timeout on its completion.
///
/// This is equivalent to [`with_deadline`] using a deadline of `TickTime::now()
/// \+ timeout`. That is, the current time is captured when `with_timeout` is
/// called (_not_ at first poll), the provided timeout is added, and that's used
/// as the deadline for the returned future.
///
/// See [`with_deadline`] for more details.
pub fn with_timeout<D, F>(timeout: D, code: F) -> impl Future<Output = Option<F::Output>>
where
    F: Future,
    TickTime: Add<D, Output = TickTime>,
{
    with_deadline(TickTime::now() + timeout, code)
}

/// A future-wrapper that gates polling a future `B` on whether another
/// future `A` has resolved.
///
/// Once `A` resolved, `B` is no longer polled and the combined future
/// resolves to `None`. If `B` resolves first, its result is produced
/// wrapped in `Some`.
#[derive(Debug)]
#[pin_project]
struct TimeLimited<A, B> {
    #[pin]
    limiter: A,
    #[pin]
    process: B,
}

impl<A, B> Future for TimeLimited<A, B>
where
    A: Future<Output = ()>,
    B: Future,
{
    type Output = Option<B::Output>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let p = self.project();
        // We always check the limiter first. If the limiter's condition has
        // occurred, we bail, even if the limited process is also ready.
        if let Poll::Ready(()) = p.limiter.poll(cx) {
            return Poll::Ready(None);
        }
        p.process.poll(cx).map(Some)
    }
}

/// Helper for doing something periodically, accurately.
///
/// A `PeriodicGate` can be used to *gate* (pause) execution of a task until a
/// point in time arrives; that point in time is *periodic*, meaning it repeats
/// at regular intervals. For example, to call the function `f` every 30
/// milliseconds, you would write:
///
/// ```ignore
/// let mut gate = PeriodicGate::from(Micros(30_000));
/// loop {
///     f();
///     gate.next_time().await;
/// }
/// ```
///
/// This will maintain the 30-millisecond interval consistently, even if `f()`
/// takes several milliseconds to run, and even if `f()` is sometimes fast and
/// sometimes slow. (If `f` sometimes takes more than 30 milliseconds, the next
/// execution will happen later than normal -- there's not a lot we can do about
/// that. However, as soon as calls to `f` take less than 30 milliseconds, we'll
/// return to the normal periodic timing.)
///
/// This is often, but not always, what you want in a timing loop.
///
/// - `PeriodicGate` has "catch-up" behavior that might not be what you want: if
///   one execution takes (say) 5 times longer than the chosen period, it will
///   frantically run 4 more just after it to "catch up." This attempts to
///   maintain a constant number of executions per unit time, but that might not
///   be what you want. (Note that you can _detect_ the "catch-up" behavior by
///   calling [`PeriodicGate::is_lagging`].)
///
/// - [`sleep_for`] can ensure a minimum delay _between_ operations, which is
///   different from `PeriodicGate`'s behavior.
#[derive(Debug)]
pub struct PeriodicGate {
    interval: Micros,
    next: TickTime,
}

impl From<Duration> for PeriodicGate {
    fn from(d: Duration) -> Self {
        PeriodicGate {
            interval: Micros(d.as_micros() as u64),
            next: TickTime::now(),
        }
    }
}

impl From<Micros> for PeriodicGate {
    /// Creates a periodic gate that can be used to release execution every
    /// `interval`, starting right now.
    fn from(interval: Micros) -> Self {
        PeriodicGate {
            interval,
            next: TickTime::now(),
        }
    }
}

impl PeriodicGate {
    /// Creates a periodic gate that can be used to release execution every
    /// `interval`, starting `delay` ticks in the future.
    ///
    /// This can be useful for creating multiple periodic gates that operate out
    /// of phase with respect to each other.
    pub fn new_shift(interval: Micros, delay: Micros) -> Self {
        PeriodicGate {
            interval,
            next: TickTime::now() + delay,
        }
    }

    /// Checks if this gate is lagging behind because it isn't being called
    /// often enough.
    pub fn is_lagging(&self) -> bool {
        self.next <= TickTime::now()
    }

    /// Returns a future that will resolve when it's time to execute again.
    ///
    /// # Cancellation
    ///
    /// **Cancel safety:** Strict.
    ///
    /// Dropping this future does nothing in particular.
    pub async fn next_time(&mut self) {
        sleep_until(self.next).await;
        self.next += self.interval;
    }
}

/// [embedded_hal_async::delay::DelayNs] implementation using our SysTick timer
#[derive(Debug)]
pub struct Delay;

impl embedded_hal_async::delay::DelayNs for Delay {
    async fn delay_ns(&mut self, mut ns: u32) {
        while ns > 1_000_000 {
            sleep_for(Micros(1)).await;
            ns -= 1_000_000;
        }
    }

    async fn delay_us(&mut self, us: u32) {
        sleep_for(Micros(us as u64)).await;
    }

    async fn delay_ms(&mut self, ms: u32) {
        sleep_for(Micros(ms as u64 * 1000)).await
    }
}

/// System tick ISR. Advances the tick counter. This doesn't wake any tasks; see
/// code in `exec` for that.
#[doc(hidden)]
#[exception]
fn SysTick() {
    let state = unsafe { &STATE[cpuid()] };

    let incr = state.incr.load(Ordering::Acquire);
    if state.tick.fetch_add(incr, Ordering::Release) < incr {
        state.epoch.fetch_add(1, Ordering::Release);
    }
}
