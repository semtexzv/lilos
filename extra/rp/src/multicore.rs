#![allow(dead_code)]
use crate::interrupt;
use crate::peripherals::CORE1;
use core::convert::Infallible;
use core::mem::ManuallyDrop;
use core::sync::atomic::{compiler_fence, Ordering};
use lilos::portable_atomic::AtomicBool;
use lilos_hal::interrupt::InterruptExt;

// const PAUSE_TOKEN: u32 = 0xDEADBEEF;
// const RESUME_TOKEN: u32 = !0xDEADBEEF;
static IS_CORE1_INIT: AtomicBool = AtomicBool::new(false);

// Push a value to the inter-core FIFO, block until space is available
#[inline(always)]
fn fifo_write(value: u32) {
    let sio = pac::SIO;
    // Wait for the FIFO to have enough space
    while !sio.fifo().st().read().rdy() {
        cortex_m::asm::nop();
    }
    sio.fifo().wr().write_value(value);
    // Fire off an event to the other core.
    // This is required as the other core may be `wfe` (waiting for event)
    cortex_m::asm::sev();
}

// Pop a value from inter-core FIFO, block until available
#[inline(always)]
fn fifo_read() -> u32 {
    let sio = pac::SIO;
    // Wait until FIFO has data
    while !sio.fifo().st().read().vld() {
        cortex_m::asm::nop();
    }
    sio.fifo().rd().read()
}

// Pop a value from inter-core FIFO, `wfe` until available
#[inline(always)]
#[allow(unused)]
fn fifo_read_wfe() -> u32 {
    let sio = pac::SIO;
    // Wait until FIFO has data
    while !sio.fifo().st().read().vld() {
        cortex_m::asm::wfe();
    }
    sio.fifo().rd().read()
}

// Drain inter-core FIFO
#[inline(always)]
fn fifo_drain() {
    let sio = pac::SIO;
    while sio.fifo().st().read().vld() {
        let _ = sio.fifo().rd().read();
    }
}

/// Data type for a properly aligned stack of N bytes
#[repr(C, align(32))]
pub struct Stack<const SIZE: usize> {
    /// Memory to be used for the stack
    pub mem: [u8; SIZE],
}

impl<const SIZE: usize> Stack<SIZE> {
    /// Construct a stack of length SIZE, initialized to 0
    pub const fn new() -> Stack<SIZE> {
        Stack { mem: [0_u8; SIZE] }
    }
}

pub fn install_core0_stack_guard() -> Result<(), ()> {
    extern "C" {
        static mut _stack_end: usize;
    }
    unsafe { install_stack_guard(core::ptr::addr_of_mut!(_stack_end)) }
}

#[cfg(all(feature = "rp2040", not(feature = "_test")))]
#[inline(always)]
fn install_stack_guard(stack_bottom: *mut usize) -> Result<(), ()> {
    let core = unsafe { cortex_m::Peripherals::steal() };

    // Fail if MPU is already configured
    if core.MPU.ctrl.read() != 0 {
        return Err(());
    }

    // The minimum we can protect is 32 bytes on a 32 byte boundary, so round up which will
    // just shorten the valid stack range a tad.
    let addr = (stack_bottom as u32 + 31) & !31;
    // Mask is 1 bit per 32 bytes of the 256 byte range... clear the bit for the segment we want
    let subregion_select = 0xff ^ (1 << ((addr >> 5) & 7));
    unsafe {
        core.MPU.ctrl.write(5); // enable mpu with background default map
        core.MPU.rbar.write((addr & !0xff) | (1 << 4)); // set address and update RNR
        core.MPU.rasr.write(
            1 // enable region
                | (0x7 << 1) // size 2^(7 + 1) = 256
                | (subregion_select << 8)
                | 0x10000000, // XN = disable instruction fetch; no other bits means no permissions
        );
    }
    Ok(())
}

#[cfg(all(feature = "rp235x", not(feature = "_test")))]
#[inline(always)]
fn install_stack_guard(stack_bottom: *mut usize) -> Result<(), ()> {
    let core = unsafe { cortex_m::Peripherals::steal() };

    // Fail if MPU is already configured
    if core.MPU.ctrl.read() != 0 {
        return Err(());
    }

    unsafe {
        core.MPU.ctrl.write(5); // enable mpu with background default map
        core.MPU.rbar.write(stack_bottom as u32 & !0xff); // set address
        core.MPU.rlar.write(1); // enable region
    }
    Ok(())
}

// This is to hack around cortex_m defaulting to ARMv7 when building tests,
// so the compile fails when we try to use ARMv8 peripherals.
#[cfg(feature = "_test")]
#[inline(always)]
fn install_stack_guard(_stack_bottom: *mut usize) -> Result<(), ()> {
    Ok(())
}

struct Multicore;
impl lilos::exec::Multicore for Multicore {
    fn cpuid() -> usize {
        rp_pac::SIO.cpuid().read() as _
    }

    fn wake_cpu(id: usize) {
        if id == 0 {
            interrupt::SIO_IRQ_PROC0.pend()
        } else {
            interrupt::SIO_IRQ_PROC1.pend()
        }
    }
}

#[inline(always)]
fn core1_setup(stack_bottom: *mut usize) {
    if install_stack_guard(stack_bottom).is_err() {
        // currently only happens if the MPU was already set up, which
        // would indicate that the core is already in use from outside
        // embassy, somehow. trap if so since we can't deal with that.
        cortex_m::asm::udf();
    }
    unsafe {
        crate::gpio::init();
    }
}

// The first two ignored `u64` parameters are there to take up all of the registers,
// which means that the rest of the arguments are taken from the stack,
// where we're able to put them from core 0.
extern "C" fn core1_startup<F: FnOnce() -> Infallible>(
    _: u64,
    _: u64,
    entry: *mut ManuallyDrop<F>,
    stack_bottom: *mut usize,
) -> Infallible {
    core1_setup(stack_bottom);

    let entry = unsafe { ManuallyDrop::take(&mut *entry) };

    // make sure the preceding read doesn't get reordered past the following fifo write
    compiler_fence(Ordering::SeqCst);

    // Signal that it's safe for core 0 to get rid of the original value now.
    fifo_write(1);

    IS_CORE1_INIT.store(true, Ordering::Release);
    // Enable fifo interrupt on CORE1 for `pause` functionality.
    #[cfg(feature = "rp2040")]
    unsafe {
        interrupt::SIO_IRQ_PROC1.enable()
    };
    #[cfg(feature = "rp235x")]
    unsafe {
        interrupt::SIO_IRQ_FIFO.enable()
    };

    entry()
}
/// Fork the execution into core0 and core1. This method will never return.
///
/// WARNING: Requires `multicore` feature on the `lilos`
pub fn fork<const SIZE: usize>(
    c1: CORE1,
    stack: &'static mut Stack<SIZE>,
    core0: impl FnOnce() -> Infallible,
    core1: impl FnOnce() -> Infallible + Send,
) -> ! {
    #![allow(unreachable_code)]
    spawn_core1(c1, stack, core1);
    core0();
    loop {
    }
}

/// Spawn execution on core1. Safety - function is not static, but we enforce stack frame on the [fork] method
pub(crate) fn spawn_core1<F, const SIZE: usize>(_: CORE1, stack: &'static mut Stack<SIZE>, entry: F)
where
    F: FnOnce() -> Infallible + Send,
{
    // Reset the core
    let psm = pac::PSM;
    psm.frce_off().modify(|w| w.set_proc1(true));
    while !psm.frce_off().read().proc1() {
        cortex_m::asm::nop();
    }
    psm.frce_off().modify(|w| w.set_proc1(false));

    // The ARM AAPCS ABI requires 8-byte stack alignment.
    // #[align] on `struct Stack` ensures the bottom is aligned, but the top could still be
    // unaligned if the user chooses a stack size that's not multiple of 8.
    // So, we round down to the next multiple of 8.
    let stack_words = stack.mem.len() / 8 * 2;
    let mem = unsafe { core::slice::from_raw_parts_mut(stack.mem.as_mut_ptr() as *mut usize, stack_words) };

    // Set up the stack
    let mut stack_ptr = unsafe { mem.as_mut_ptr().add(mem.len()) };

    // We don't want to drop this, since it's getting moved to the other core.
    let mut entry = ManuallyDrop::new(entry);

    // Push the arguments to `core1_startup` onto the stack.
    unsafe {
        // Push `stack_bottom`.
        stack_ptr = stack_ptr.sub(1);
        stack_ptr.cast::<*mut usize>().write(mem.as_mut_ptr());

        // Push `entry`.
        stack_ptr = stack_ptr.sub(1);
        stack_ptr.cast::<*mut ManuallyDrop<F>>().write(&mut entry);
    }

    // Make sure the compiler does not reorder the stack writes after to after the
    // below FIFO writes, which would result in them not being seen by the second
    // core.
    //
    // From the compiler perspective, this doesn't guarantee that the second core
    // actually sees those writes. However, we know that the RP2040 doesn't have
    // memory caches, and writes happen in-order.
    compiler_fence(Ordering::Release);

    let p = unsafe { cortex_m::Peripherals::steal() };
    let vector_table = p.SCB.vtor.read();

    // After reset, core 1 is waiting to receive commands over FIFO.
    // This is the sequence to have it jump to some code.
    let cmd_seq = [
        0,
        0,
        1,
        vector_table as usize,
        stack_ptr as usize,
        core1_startup::<F> as usize,
    ];

    let mut seq = 0;
    let mut fails = 0;
    loop {
        let cmd = cmd_seq[seq] as u32;
        if cmd == 0 {
            fifo_drain();
            cortex_m::asm::sev();
        }
        fifo_write(cmd);

        let response = fifo_read();
        if cmd == response {
            seq += 1;
        } else {
            seq = 0;
            fails += 1;
            if fails > 16 {
                // The second core isn't responding, and isn't going to take the entrypoint
                panic!("CORE1 not responding");
            }
        }
        if seq >= cmd_seq.len() {
            break;
        }
    }

    // Wait until the other core has copied `entry` before returning.
    fifo_read();
}

#[interrupt]
unsafe fn SIO_IRQ_PROC0() {
    let sio = pac::SIO;
    // Clear IRQ
    sio.fifo().st().write(|w| w.set_wof(false));
}

#[interrupt]
unsafe fn SIO_IRQ_PROC1() {
    let sio = pac::SIO;
    // Clear IRQ
    sio.fifo().st().write(|w| w.set_wof(false));

    // while sio.fifo().st().read().vld() {
    //     // Pause CORE1 execution and disable interrupts
    //     if fifo_read_wfe() == PAUSE_TOKEN {
    //         cortex_m::interrupt::disable();
    //         // Signal to CORE0 that execution is paused
    //         fifo_write(PAUSE_TOKEN);
    //         // Wait for `resume` signal from CORE0
    //         while fifo_read_wfe() != RESUME_TOKEN {
    //             cortex_m::asm::nop();
    //         }
    //         cortex_m::interrupt::enable();
    //         // Signal to CORE0 that execution is resumed
    //         fifo_write(RESUME_TOKEN);
    //     }
    // }
}

#[cfg(all(feature = "rt", feature = "rp235x"))]
#[interrupt]
unsafe fn SIO_IRQ_FIFO() {
    let sio = pac::SIO;
    // Clear IRQ
    sio.fifo().st().write(|w| w.set_wof(false));

    while sio.fifo().st().read().vld() {
        // Pause CORE1 execution and disable interrupts
        if fifo_read_wfe() == PAUSE_TOKEN {
            cortex_m::interrupt::disable();
            // Signal to CORE0 that execution is paused
            fifo_write(PAUSE_TOKEN);
            // Wait for `resume` signal from CORE0
            while fifo_read_wfe() != RESUME_TOKEN {
                cortex_m::asm::nop();
            }
            cortex_m::interrupt::enable();
            // Signal to CORE0 that execution is resumed
            fifo_write(RESUME_TOKEN);
        }
    }
}

lilos::provide_multicore_impl!(Multicore);
