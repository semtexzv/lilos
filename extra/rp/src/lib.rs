#![deny(dead_code)]
#![cfg_attr(not(test), no_std)]

pub mod clocks;
pub mod dma;
pub mod gpio;
pub mod spi;

mod critical_section_impl;
#[cfg(feature = "multicore")]
pub mod multicore;
pub(crate) mod reset;

extern crate rp_pac as pac;

pub use lilos_hal::{into_ref, Peripheral, PeripheralRef};

lilos_hal::interrupt_mod!(
    TIMER_IRQ_0,
    TIMER_IRQ_1,
    TIMER_IRQ_2,
    TIMER_IRQ_3,
    PWM_IRQ_WRAP,
    USBCTRL_IRQ,
    XIP_IRQ,
    PIO0_IRQ_0,
    PIO0_IRQ_1,
    PIO1_IRQ_0,
    PIO1_IRQ_1,
    DMA_IRQ_0,
    DMA_IRQ_1,
    IO_IRQ_BANK0,
    IO_IRQ_QSPI,
    SIO_IRQ_PROC0,
    SIO_IRQ_PROC1,
    CLOCKS_IRQ,
    SPI0_IRQ,
    SPI1_IRQ,
    UART0_IRQ,
    UART1_IRQ,
    ADC_IRQ_FIFO,
    I2C0_IRQ,
    I2C1_IRQ,
    RTC_IRQ,
    SWI_IRQ_0,
    SWI_IRQ_1,
    SWI_IRQ_2,
    SWI_IRQ_3,
    SWI_IRQ_4,
    SWI_IRQ_5,
);

lilos_hal::peripherals! {
    PIN_0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7,
    PIN_8,
    PIN_9,
    PIN_10,
    PIN_11,
    PIN_12,
    PIN_13,
    PIN_14,
    PIN_15,
    PIN_16,
    PIN_17,
    PIN_18,
    PIN_19,
    PIN_20,
    PIN_21,
    PIN_22,
    PIN_23,
    PIN_24,
    PIN_25,
    PIN_26,
    PIN_27,
    PIN_28,
    PIN_29,
    PIN_QSPI_SCLK,
    PIN_QSPI_SS,
    PIN_QSPI_SD0,
    PIN_QSPI_SD1,
    PIN_QSPI_SD2,
    PIN_QSPI_SD3,

    UART0,
    UART1,

    SPI0,
    SPI1,

    I2C0,
    I2C1,

    DMA_CH0,
    DMA_CH1,
    DMA_CH2,
    DMA_CH3,
    DMA_CH4,
    DMA_CH5,
    DMA_CH6,
    DMA_CH7,
    DMA_CH8,
    DMA_CH9,
    DMA_CH10,
    DMA_CH11,

    PWM_SLICE0,
    PWM_SLICE1,
    PWM_SLICE2,
    PWM_SLICE3,
    PWM_SLICE4,
    PWM_SLICE5,
    PWM_SLICE6,
    PWM_SLICE7,

    USB,

    RTC,

    FLASH,

    ADC,
    ADC_TEMP_SENSOR,

    CORE1,

    PIO0,
    PIO1,

    WATCHDOG,
    BOOTSEL,
}

pub unsafe fn init() {
    // SIO does not get reset when core0 is reset with either `scb::sys_reset()` or with SWD.
    // Since we're using SIO spinlock 31 for the critical-section impl, this causes random
    // hangs if we reset in the middle of a CS, because the next boot sees the spinlock
    // as locked and waits forever.
    //
    // See https://github.com/embassy-rs/embassy/issues/1736
    // and https://github.com/rp-rs/rp-hal/issues/292
    // and https://matrix.to/#/!vhKMWjizPZBgKeknOo:matrix.org/$VfOkQgyf1PjmaXZbtycFzrCje1RorAXd8BQFHTl4d5M
    //
    // According to Raspberry Pi, this is considered Working As Intended, and not an errata,
    // even though this behavior is different from every other ARM chip (sys_reset usually resets
    // the *system* as its name implies, not just the current core).
    //
    // A similar thing could happen with PROC1. It is unclear whether it's possible for PROC1
    // to stay unreset through a PROC0 reset, so we reset it anyway just in case.
    //
    // Important info from PSM logic (from Luke Wren in above Matrix thread)
    //
    //     The logic is, each PSM stage is reset if either of the following is true:
    //     - The previous stage is in reset and FRCE_ON is false
    //     - FRCE_OFF is true
    //
    // The PSM order is SIO -> PROC0 -> PROC1.
    // So, we have to force-on PROC0 to prevent it from getting reset when resetting SIO.
    pac::PSM.frce_on().write_and_wait(|w| {
        w.set_proc0(true);
    });
    // Then reset SIO and PROC1.
    pac::PSM.frce_off().write_and_wait(|w| {
        w.set_sio(true);
        w.set_proc1(true);
    });
    // clear force_off first, force_on second. The other way around would reset PROC0.
    pac::PSM.frce_off().write_and_wait(|_| {});
    pac::PSM.frce_on().write_and_wait(|_| {});
}

/// Extension trait for PAC regs, adding atomic xor/bitset/bitclear writes.
#[allow(unused)]
trait RegExt<T: Copy> {
    #[allow(unused)]
    fn write_xor<R>(&self, f: impl FnOnce(&mut T) -> R) -> R;
    fn write_set<R>(&self, f: impl FnOnce(&mut T) -> R) -> R;
    fn write_clear<R>(&self, f: impl FnOnce(&mut T) -> R) -> R;
    fn write_and_wait<R>(&self, f: impl FnOnce(&mut T) -> R) -> R
    where
        T: PartialEq;
}

impl<T: Default + Copy, A: pac::common::Write> RegExt<T> for pac::common::Reg<T, A> {
    fn write_xor<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let mut val = Default::default();
        let res = f(&mut val);
        unsafe {
            let ptr = (self.as_ptr() as *mut u8).add(0x1000) as *mut T;
            ptr.write_volatile(val);
        }
        res
    }

    fn write_set<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let mut val = Default::default();
        let res = f(&mut val);
        unsafe {
            let ptr = (self.as_ptr() as *mut u8).add(0x2000) as *mut T;
            ptr.write_volatile(val);
        }
        res
    }

    fn write_clear<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let mut val = Default::default();
        let res = f(&mut val);
        unsafe {
            let ptr = (self.as_ptr() as *mut u8).add(0x3000) as *mut T;
            ptr.write_volatile(val);
        }
        res
    }

    fn write_and_wait<R>(&self, f: impl FnOnce(&mut T) -> R) -> R
    where
        T: PartialEq,
    {
        let mut val = Default::default();
        let res = f(&mut val);
        unsafe {
            self.as_ptr().write_volatile(val);
            while self.as_ptr().read_volatile() != val {}
        }
        res
    }
}
