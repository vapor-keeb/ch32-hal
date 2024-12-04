use core::{
    future::{poll_fn, PollFn},
    marker::PhantomData,
    task::{Poll, Waker},
};

use crate::{interrupt::typelevel::Interrupt, usb::EndpointDataBuffer};
use ch32_metapac::usbhs::vals::SpeedType;
use defmt::Format;
use embassy_hal_internal::Peripheral;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use embassy_usb_driver::EndpointError;

use crate::{
    gpio::{AFType, Speed},
    interrupt,
};

use super::Instance;

const MAX_PACKET_SIZE: usize = 64;

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static TX_WAKER: AtomicWaker = AtomicWaker::new();
// static RX_WAKER: AtomicWaker = AtomicWaker::new();

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

pub struct USBHsHostDriver<'d, T: Instance> {
    _phantom: PhantomData<T>,
    tx_buf: &'d mut EndpointDataBuffer,
    rx_buf: &'d mut EndpointDataBuffer,
}

impl<'d, T: Instance> USBHsHostDriver<'d, T> {
    pub async fn setup(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let h = T::hregs();

        self.tx_buf.write_volatile(buf);
        h.ep_pid().write(|v| {
            v.set_endp(0);
            v.set_token(0b1101);
        });

        h.tx_len().write(|v| v.set_len(buf.len() as u16));
        h.rx_ctrl().write(|_| {});
        h.tx_ctrl().write(|_| {});

        assert!(!h.int_fg().read().transfer());

        poll_fn(|ctx| {
            TX_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();

            if transfer {
                trace!("status h_res {}", status.0);
                h.ep_pid().write(|_| {});
                h.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| h.int_en().modify(|w| w.set_transfer(true)));
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        Ok(())
    }
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
    pub async fn reset(&mut self) {
        // follow example code from openwch
        T::hregs().dev_ad().write(|v| v.set_addr(0));
        critical_section::with(|_| {
            T::hregs().ctrl().modify(|v| {
                v.set_tx_bus_reset(true);
            });
        });
        Timer::after_millis(15).await;
        critical_section::with(|_| {
            T::hregs().ctrl().modify(|v| {
                v.set_tx_bus_reset(false);
            });
        });
        Timer::after_millis(2).await;
        // copied from openwch
        if T::hregs().int_fg().read().detect() {
            if T::hregs().mis_st().read().dev_attach() {
                panic!("attach after reset");
                T::hregs().int_fg().write(|v| v.set_detect(true));
                critical_section::with(|_| T::hregs().int_en().modify(|v| v.set_detect(true)));
            }
        }
    }

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
                // small problem
                // currently suspend fires after device attach
                // unclear what this means
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
        if flag.transfer() {
            r.int_en().modify(|v| v.set_transfer(false));
            TX_WAKER.wake();
            // RX_WAKER.wake();
        }

        trace!("irq: {:x}", flag.0);
    }
}

pub fn start<'d, T: Instance>(
    dp: impl Peripheral<P = impl super::DpPin<T, 0> + 'd>,
    dm: impl Peripheral<P = impl super::DmPin<T, 0> + 'd>,
    tx_buf: &'d mut EndpointDataBuffer,
    rx_buf: &'d mut EndpointDataBuffer,
) -> (Bus<T>, USBHsHostDriver<'d, T>) {
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
    h.rx_dma().write_value(rx_buf.addr() as u32);
    h.tx_dma().write_value(tx_buf.addr() as u32);

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

    (
        Bus { _phantom: PhantomData },
        USBHsHostDriver {
            tx_buf,
            rx_buf,
            _phantom: PhantomData,
        },
    )
}
