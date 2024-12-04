use core::{
    future::{poll_fn, PollFn},
    marker::PhantomData,
    task::{Poll, Waker},
};

use crate::interrupt::typelevel::Interrupt;
use ch32_metapac::usbhs::vals::SpeedType;
use defmt::Format;
use embassy_hal_internal::Peripheral;
use embassy_sync::waitqueue::AtomicWaker;

use crate::{
    gpio::{AFType, Speed},
    interrupt,
};

use super::Instance;

const MAX_PACKET_SIZE: usize = 64;
static EP_TX_BUF: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];
static EP_RX_BUF: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];

static BUS_WAKER: AtomicWaker = AtomicWaker::new();

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

pub struct USBHsHostDriver<T: Instance> {
    _phantom: PhantomData<T>,
}

pub struct Bus<T: Instance> {
    _phantom: PhantomData<T>,
}

#[cfg_attr(feature = "defmt", derive(Format))]
pub enum Event {
    DeviceAttach,
    DeviceDetach,
    Suspend,
    Resume,
}

impl<T: Instance> Bus<T> {
    pub async fn poll(&mut self) -> Event {
        poll_fn(|ctx| {
            BUS_WAKER.register(ctx.waker());

            let regs = T::hregs();

            let flags = regs.int_fg().read();
            if flags.detect() {
                let res = if regs.mis_st().read().dev_attach() {
                    Event::DeviceAttach
                } else {
                    Event::DeviceDetach
                };

                regs.int_fg().write(|v| v.set_detect(true));
                critical_section::with(|_| regs.int_en().modify(|v| v.set_detect(true)));

                Poll::Ready(res)
            } else if flags.suspend() {
                let res = if regs.mis_st().read().suspend() {
                    Event::Suspend
                } else {
                    Event::Resume
                };

                regs.int_fg().write(|v| v.set_suspend(true));
                critical_section::with(|_| regs.int_en().modify(|v| v.set_suspend(true)));

                Poll::Ready(res)
            } else {
                Poll::Pending
            }

            // TODO more flags
        })
        .await
    }
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let flag = r.int_fg().read();

        if flag.detect() {
            r.int_en().modify(|v| v.set_detect(false));
            BUS_WAKER.wake();
        } 
        if flag.suspend() {
            r.int_en().modify(|v| v.set_suspend(false));
            BUS_WAKER.wake();
        }

        trace!("irq: {:x}", flag.0);
    }
}

pub fn start<'d, T: Instance>(
    dp: impl Peripheral<P = impl super::DpPin<T, 0> + 'd>,
    dm: impl Peripheral<P = impl super::DmPin<T, 0> + 'd>,
) -> (Bus<T>, USBHsHostDriver<T>) {
    let dp = dp.into_ref();
    let dm = dm.into_ref();

    dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
    dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

    T::enable_and_reset();

    let r = T::regs();
    let h = T::hregs();

    r.ctrl().write(|w| {
        w.set_clr_all(true);
        w.set_reset_sie(true);
    });

    // Sleep for 10uS from WCH C code
    embassy_time::block_for(embassy_time::Duration::from_micros(10));

    r.ctrl().write(|_| {});

    r.ctrl().write(|w| {
        w.set_host_mode(true);
        w.set_speed_type(SpeedType::HIGHSPEED);
        w.set_int_busy(true);
        w.set_dma_en(true);
    });

    h.config().write(|w| {
        w.set_h_tx_en(true);
        w.set_h_rx_en(true);
    });
    h.rx_dma().write_value((&EP_RX_BUF).as_ptr() as u32);
    h.tx_dma().write_value((&EP_TX_BUF).as_ptr() as u32);

    h.ctrl().write(|w| w.set_phy_suspendm(true));

    h.rx_max_len().write(|w| w.set_len(MAX_PACKET_SIZE as u16));

    r.int_en().write(|w| {
        w.set_dev_nak(false);
        w.set_fifo_ov(true);
        w.set_setup_act(true);
        w.set_suspend(true);
        w.set_transfer(true);
        w.set_detect(true);
    });

    unsafe { T::Interrupt::enable() };

    (Bus { _phantom: PhantomData }, USBHsHostDriver { _phantom: PhantomData })
}
