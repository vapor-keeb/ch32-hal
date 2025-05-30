use core::{
    future::{poll_fn, Future},
    marker::PhantomData,
    task::Poll,
};

use async_usb_host::{types::UsbSpeed, Event};
use defmt::todo;
use embassy_time::Timer;

use crate::usbhs::Instance;

use super::BUS_WAKER;

pub struct Bus<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Bus<T> {
    pub(crate) fn new() -> Self {
        Bus { _phantom: PhantomData }
    }
}

impl<T: Instance> async_usb_host::Bus for Bus<T> {
    async fn reset(&mut self) {
        // follow example code from openwch (`USBHSH_ResetRootHubPort`)
        T::hregs().dev_ad().write(|v| v.set_addr(0));

        critical_section::with(|_| {
            T::hregs().ctrl().modify(|v| {
                v.set_tx_bus_reset(true);
            });
        });
        Timer::after_millis(11).await;
        critical_section::with(|_| {
            T::hregs().ctrl().modify(|v| {
                v.set_tx_bus_reset(false);
            });
        });

        // magic wait also from openwch
        Timer::after_millis(2).await;

        // copied from openwch
        if T::hregs().int_fg().read().detect() {
            if T::hregs().mis_st().read().dev_attach() {
                todo!("attach after reset");
            }
        }

        if T::hregs().ctrl().read().sof_en() {
            info!("sof_en already enabled.");
        }
        critical_section::with(|_| {
            T::hregs().ctrl().modify(|v| v.set_sof_en(true));
        });
    }

    fn poll(&mut self) -> impl Future<Output = Event> {
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
    }

    async fn speed(&mut self) -> Option<UsbSpeed> {
        let hregs = T::hregs();
        use ch32_metapac::usbhs::vals::SpeedType;
        match hregs.speed_type().read().speed_type() {
            SpeedType::FULLSPEED => Some(UsbSpeed::FullSpeed),
            SpeedType::HIGHSPEED => Some(UsbSpeed::HighSpeed),
            SpeedType::LOWSPEED => Some(UsbSpeed::LowSpeed),
            SpeedType::_RESERVED_3 => None,
        }
    }
}
