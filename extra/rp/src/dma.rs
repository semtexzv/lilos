use crate::interrupt;
use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::{Context, Poll};
use lilos::exec::Notify;
use lilos_hal::interrupt::InterruptExt;
use lilos_hal::{impl_peripheral, into_ref, Peripheral, PeripheralRef};
use rp_pac::dma::vals::{DataSize, TreqSel};

pub const CHANNEL_COUNT: usize = 12;
const NEW_NOTIFY: Notify = Notify::new();
pub static mut CHANNEL_WAKERS: [Notify; CHANNEL_COUNT] = [NEW_NOTIFY; CHANNEL_COUNT];

#[interrupt]
unsafe fn DMA_IRQ_0() {
    let ints0 = pac::DMA.ints(0).read();
    for channel in 0..CHANNEL_COUNT {
        let ctrl_trig = pac::DMA.ch(channel).ctrl_trig().read();
        if ctrl_trig.ahb_error() {
            panic!("DMA: error on DMA_0 channel {}", channel);
        }

        if ints0 & (1 << channel) == (1 << channel) {
            CHANNEL_WAKERS[channel].notify();
        }
    }
    pac::DMA.ints(0).write_value(ints0);
}

/// Initialize the dma subsystem. Required if you use DMA or the SPI interfaces
pub unsafe fn init() {
    interrupt::DMA_IRQ_0.disable();
    interrupt::DMA_IRQ_0.set_priority(interrupt::Priority::P3);

    pac::DMA.inte(0).write_value(0xFFFF);

    interrupt::DMA_IRQ_0.enable();
}

trait SealedChannel {}
trait SealedWord {}

/// DMA channel interface.
#[allow(private_bounds)]
pub trait Channel: Peripheral<P = Self> + SealedChannel + Into<AnyChannel> + Sized + 'static {
    /// Channel number.
    fn number(&self) -> u8;

    /// Channel registry block.
    fn regs(&self) -> rp_pac::dma::Channel {
        rp_pac::DMA.ch(self.number() as _)
    }

    /// Convert into type-erased [AnyChannel].
    fn degrade(self) -> AnyChannel {
        AnyChannel { number: self.number() }
    }
}

/// DMA word.
#[allow(private_bounds)]
pub trait Word: SealedWord {
    /// Word size.
    fn size() -> DataSize;
}

impl SealedWord for u8 {}
impl Word for u8 {
    fn size() -> DataSize {
        DataSize::SIZE_BYTE
    }
}

impl SealedWord for u16 {}
impl Word for u16 {
    fn size() -> DataSize {
        DataSize::SIZE_HALFWORD
    }
}

impl SealedWord for u32 {}
impl Word for u32 {
    fn size() -> DataSize {
        DataSize::SIZE_WORD
    }
}

/// Type erased DMA channel.
pub struct AnyChannel {
    number: u8,
}

impl_peripheral!(AnyChannel);

impl SealedChannel for AnyChannel {}
impl Channel for AnyChannel {
    fn number(&self) -> u8 {
        self.number
    }
}

macro_rules! channel {
    ($name:ident, $num:expr) => {
        impl SealedChannel for crate::peripherals::$name {}
        impl Channel for crate::peripherals::$name {
            fn number(&self) -> u8 {
                $num
            }
        }

        impl From<crate::peripherals::$name> for crate::dma::AnyChannel {
            fn from(val: crate::peripherals::$name) -> Self {
                crate::dma::Channel::degrade(val)
            }
        }
    };
}

channel!(DMA_CH0, 0);
channel!(DMA_CH1, 1);
channel!(DMA_CH2, 2);
channel!(DMA_CH3, 3);
channel!(DMA_CH4, 4);
channel!(DMA_CH5, 5);
channel!(DMA_CH6, 6);
channel!(DMA_CH7, 7);
channel!(DMA_CH8, 8);
channel!(DMA_CH9, 9);
channel!(DMA_CH10, 10);
channel!(DMA_CH11, 11);

/// DMA read.
///
/// SAFETY: Slice must point to a valid location reachable by DMA.
pub unsafe fn read<'a, C: Channel, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    from: *const W,
    to: *mut [W],
    dreq: u8,
) -> Transfer<'a, C> {
    copy_inner(
        ch,
        from as *const u32,
        to as *mut W as *mut u32,
        (&*to).len(),
        W::size(),
        false,
        true,
        dreq,
    )
}

/// DMA write.
///
/// SAFETY: Slice must point to a valid location reachable by DMA.
pub unsafe fn write<'a, C: Channel, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    from: *const [W],
    to: *mut W,
    dreq: u8,
) -> Transfer<'a, C> {
    copy_inner(
        ch,
        from as *const W as *const u32,
        to as *mut u32,
        (&*from).len(),
        W::size(),
        true,
        false,
        dreq,
    )
}

// static mut so that this is allocated in RAM.
static mut DUMMY: u32 = 0;

/// DMA repeated write.
///
/// SAFETY: Slice must point to a valid location reachable by DMA.
pub unsafe fn write_repeated<'a, C: Channel, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    to: *mut W,
    len: usize,
    dreq: u8,
) -> Transfer<'a, C> {
    copy_inner(
        ch,
        core::ptr::addr_of_mut!(DUMMY) as *const u32,
        to as *mut u32,
        len,
        W::size(),
        false,
        false,
        dreq,
    )
}

/// DMA copy between slices.
///
/// SAFETY: Slices must point to locations reachable by DMA.
pub unsafe fn copy<'a, C: Channel, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    from: &[W],
    to: &mut [W],
) -> Transfer<'a, C> {
    let from_len = from.len();
    let to_len = to.len();
    assert_eq!(from_len, to_len);
    copy_inner(
        ch,
        from.as_ptr() as *const u32,
        to.as_mut_ptr() as *mut u32,
        from_len,
        W::size(),
        true,
        true,
        TreqSel::PERMANENT.to_bits(),
    )
}

fn copy_inner<'a, C: Channel>(
    ch: impl Peripheral<P = C> + 'a,
    from: *const u32,
    to: *mut u32,
    len: usize,
    data_size: DataSize,
    incr_read: bool,
    incr_write: bool,
    dreq: u8,
) -> Transfer<'a, C> {
    into_ref!(ch);

    let p = ch.regs();

    p.read_addr().write_value(from as u32);
    p.write_addr().write_value(to as u32);
    p.trans_count().write_value(len as u32);

    compiler_fence(Ordering::SeqCst);

    p.ctrl_trig().write(|w| {
        // TODO: Add all DREQ options to pac vals::TreqSel, and use
        // `set_treq:sel`
        w.0 = ((dreq as u32) & 0x3f) << 15usize;
        w.set_data_size(data_size);
        w.set_incr_read(incr_read);
        w.set_incr_write(incr_write);
        w.set_chain_to(ch.number());
        w.set_en(true);
    });

    compiler_fence(Ordering::SeqCst);
    Transfer::new(ch)
}

/// DMA transfer driver.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a, C: Channel> {
    channel: PeripheralRef<'a, C>,
}

impl<'a, C: Channel> Transfer<'a, C> {
    pub(crate) fn new(channel: impl Peripheral<P = C> + 'a) -> Self {
        into_ref!(channel);

        Self { channel }
    }
}

impl<'a, C: Channel> Drop for Transfer<'a, C> {
    fn drop(&mut self) {
        let p = self.channel.regs();
        rp_pac::DMA
            .chan_abort()
            .modify(|m| m.set_chan_abort(1 << self.channel.number()));
        while p.ctrl_trig().read().busy() {}
    }
}

impl<'a, C: Channel> Unpin for Transfer<'a, C> {}
impl<'a, C: Channel> Future for Transfer<'a, C> {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // We need to register/re-register the waker for each poll because any
        // calls to wake will deregister the waker.
        unsafe {
            CHANNEL_WAKERS[self.channel.number() as usize].subscribe(cx.waker());
        }

        if self.channel.regs().ctrl_trig().read().busy() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}
