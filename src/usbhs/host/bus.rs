use core::{future::poll_fn, marker::PhantomData, task::Poll};

use async_usb_host::{errors::UsbHostError, types::Pid, Event};
use ch32_metapac::usbhs::vals::{HostTxResponse, Tog};
use embassy_time::Timer;

use crate::usbhs::{Instance};

use super::{BUS_WAKER, MAX_PACKET_SIZE};

pub struct Bus<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Bus<T> {
    pub(crate) fn new() -> Self {
        Bus {
            _phantom: PhantomData,
        }
    }
}

impl<T: Instance> async_usb_host::Bus for Bus<T> {
    async fn reset(&mut self) {
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
            }
        }
        // don't let the bus sleep (NOT GOOD)
        critical_section::with(|_| {
            T::hregs().ctrl().modify(|w| w.set_sof_en(true));
        });
    }

    async fn poll(&mut self) -> Event {
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
