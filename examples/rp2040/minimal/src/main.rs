// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Minimal example of using `lilos` to blink an LED at 1Hz on the
//! Raspberry Pi Pico board.
//!
//! This starts a single task, which uses the scheduler and timer to
//! periodically toggle a GPIO pin (pin 25, which is an LED on the Pi Pico
//! board).
//!
//! This demonstrates
//!
//! 1. How to start the `lilos` executor and configure timekeeping.
//! 2. How to perform periodic actions and delays.
//! 3. How to safely share data on the stack with a task.

// We won't be using the standard library.
#![no_std]
// We don't have a conventional `main` (`cortex_m_rt::entry` is different).
#![no_main]

// Pull in a panic handling crate. We have to `extern crate` this explicitly
// because it isn't otherwise referenced in code!
extern crate panic_halt;

use lilos_rp::gpio::Level;

// For RP2040, we need to include a bootloader. The general Cargo build process
// doesn't have great support for this, so we included it as a binary constant.
#[link_section = ".boot_loader"]
#[used]
static BOOT: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// How often our blinky task wakes up (1/2 our blink frequency).
const PERIOD: lilos::time::Micros = lilos::time::Micros(500);

#[cortex_m_rt::entry]
fn main() -> ! {
    // Check out peripherals from the runtime.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let p = lilos_rp::Peripherals::take();

    let mut output = lilos_rp::gpio::Output::new(p.PIN_25, Level::High);

    // Create a task to blink the LED. You could also write this as an `async
    // fn` but we've inlined it as an `async` block for simplicity.
    let blink = core::pin::pin!(async {
        // PeriodicGate is a `lilos` tool for implementing low-jitter periodic
        // actions. It opens once per PERIOD.
        let mut gate = lilos::time::PeriodicGate::from(PERIOD);

        // Loop forever, blinking things. Note that this borrows the device
        // peripherals `p` from the enclosing stack frame.
        loop {
            output.set_level(!output.get_output_level());
            gate.next_time().await;
        }
    });

    // Configure the systick timer for 1kHz ticks at the default ROSC speed of
    // _roughly_ 6 MHz.
    lilos::time::initialize_sys_tick(&mut cp.SYST, 6_000_000);
    // Set up and run the scheduler with a single task.
    lilos::exec::run_tasks(
        &mut [blink],           // <-- array of tasks
        lilos::exec::ALL_TASKS, // <-- which to start initially
    )
}
