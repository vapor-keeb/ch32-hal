use core::{future::poll_fn, marker::PhantomData, task::Poll};

use async_usb_host::{errors::UsbHostError, types::Pid, Event};
use ch32_metapac::usbhs::vals::{HostTxResponse, Tog};
use embassy_time::Timer;

use crate::{
    usb::EndpointDataBuffer,
    usbhs::{host::TX_WAKER, Instance},
};

use super::{BUS_WAKER, MAX_PACKET_SIZE};

pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<T>,
    tx_buf: &'d mut EndpointDataBuffer,
    rx_buf: &'d mut EndpointDataBuffer,
}

impl<'d, T: Instance> Bus<'d, T> {
    pub(crate) fn new(tx_buf: &'d mut EndpointDataBuffer, rx_buf: &'d mut EndpointDataBuffer) -> Self {
        Bus {
            _phantom: PhantomData,
            tx_buf,
            rx_buf,
        }
    }
}

impl<'d, T: Instance> async_usb_host::Bus for Bus<'d, T> {
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
    /// Send the 8 byte setup
    async fn setup(&mut self, buf: &[u8; 8]) -> Result<(), UsbHostError> {
        let h = T::hregs();

        self.tx_buf.write_volatile(buf);
        h.tx_len().write(|v| v.set_len(buf.len() as u16));
        h.rx_ctrl().write(|v| {
            v.set_r_tog(Tog::DATA0);
        });
        h.tx_ctrl().write(|v| {
            v.set_t_tog(Tog::DATA0);
            v.set_t_res(HostTxResponse::ACK);
        });

        h.ep_pid().write(|v| {
            v.set_endp(0);
            v.set_token(Pid::SETUP as u8);
        });

        poll_fn(|ctx| {
            TX_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();

            if transfer {
                // First stop sending more setup
                h.ep_pid().write(|_| {});

                // Check what the device responded
                let device_response = unwrap!(TryInto::<Pid>::try_into(status.h_res()));
                let res = match device_response {
                    Pid::ACK => Ok(()),
                    Pid::NAK => Err(UsbHostError::NAK),
                    Pid::STALL => Err(UsbHostError::STALL),
                    _ => panic!("??? {:?}", device_response),
                };

                // Mark transfer as complete
                h.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| h.int_en().modify(|w| w.set_transfer(true)));
                Poll::Ready(res)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    async fn data_in(&mut self, buf: &mut [u8]) -> Result<usize, UsbHostError> {
        let h = T::hregs();
        // Send IN token to allow the bytes to come in
        critical_section::with(|_| {
            h.rx_ctrl().modify(|v| {
                v.set_r_tog(match v.r_tog() {
                    Tog::DATA0 => Tog::DATA1,
                    Tog::DATA1 => Tog::DATA0,
                    _ => panic!(),
                });
            });
        });
        h.ep_pid().write(|v| {
            v.set_endp(0);
            v.set_token(Pid::IN as u8);
        });

        poll_fn(|ctx| {
            TX_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();
            if transfer {
                // First stop sending
                h.ep_pid().write(|_| {});
                trace!("status: {}", status.0);
                let res = match unwrap!(TryInto::<Pid>::try_into(status.h_res())) {
                    Pid::DATA0 | Pid::DATA1 => {
                        if status.tog_ok() {
                            let bytes_read = h.rx_len().read() as usize;
                            debug_assert!(bytes_read <= 64); // TODO: FIX THIS when we have a size for self.rx_buf
                            if bytes_read > buf.len() {
                                Err(UsbHostError::BufferOverflow)
                            } else {
                                self.rx_buf.read_volatile(&mut buf[..bytes_read]);
                                Ok(bytes_read)
                            }
                        } else {
                            error!("Wrong TOG");
                            Err(UsbHostError::WrongTog)
                        }
                    }
                    Pid::NAK => panic!("NAK"),
                    Pid::STALL => Err(UsbHostError::STALL),
                    pid => {
                        panic!("Unexpected pid: {}", pid)
                    }
                };

                // Mark transfer as complete
                h.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| h.int_en().modify(|w| w.set_transfer(true)));

                Poll::Ready(res)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    async fn data_out(&mut self, buf: &[u8]) -> Result<(), UsbHostError> {
        if buf.len() > MAX_PACKET_SIZE {
            return Err(UsbHostError::BufferOverflow);
        }

        let h = T::hregs();

        self.tx_buf.write_volatile(buf);
        h.tx_len().write(|v| v.set_len(buf.len() as u16));
        critical_section::with(|_| {
            h.tx_ctrl().modify(|v| {
                v.set_t_tog(match v.t_tog() {
                    Tog::DATA0 => Tog::DATA1,
                    Tog::DATA1 => Tog::DATA0,
                    _ => panic!(),
                });
            });
        });
        h.ep_pid().write(|v| {
            v.set_endp(0);
            v.set_token(Pid::OUT as u8);
        });

        poll_fn(|ctx| {
            TX_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();

            if transfer {
                // First stop sending
                h.ep_pid().write(|_| {});

                // Check what the device responded
                let device_response = unwrap!(TryInto::<Pid>::try_into(status.h_res()));
                let res = match device_response {
                    Pid::ACK => Ok(()),
                    Pid::NAK => Err(UsbHostError::NAK),
                    Pid::STALL => Err(UsbHostError::STALL),
                    _ => panic!("??? {:?}", device_response),
                };

                // Mark transfer as complete
                h.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| h.int_en().modify(|w| w.set_transfer(true)));
                Poll::Ready(res)
            } else {
                Poll::Pending
            }
        })
        .await
    }
}
