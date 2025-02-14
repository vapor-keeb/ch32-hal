use core::{future::poll_fn, marker::PhantomData, task::Poll};

use async_usb_host::{
    errors::UsbHostError,
    types::{DataTog, Pid},
};
use ch32_metapac::usbhs::vals::{HostTxResponse, Tog};

use crate::{
    usb::EndpointDataBuffer,
    usbhs::{host::PIPE_WAKER, Instance},
};

use super::MAX_PACKET_SIZE;

pub struct Pipe<'d, T: Instance> {
    _phantom: PhantomData<T>,
    tx_buf: &'d mut EndpointDataBuffer,
    rx_buf: &'d mut EndpointDataBuffer,
}

impl<'d, T: Instance> Pipe<'d, T> {
    pub(crate) fn new(tx_buf: &'d mut EndpointDataBuffer, rx_buf: &'d mut EndpointDataBuffer) -> Self {
        Pipe {
            _phantom: PhantomData,
            tx_buf,
            rx_buf,
        }
    }
}

impl<'d, T: Instance> async_usb_host::Pipe for Pipe<'d, T> {
    fn set_addr(&mut self, addr: u8) {
        let h = T::hregs();
        h.dev_ad().write(|v| v.set_addr(addr));
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
            PIPE_WAKER.register(ctx.waker());
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
                    r => panic!("??? {:?} {:x}", r, status.h_res()),
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

    async fn data_in(&mut self, endpoint: u8, tog: DataTog, buf: &mut [u8]) -> Result<usize, UsbHostError> {
        let h = T::hregs();
        // Send IN token to allow the bytes to come in
        critical_section::with(|_| {
            h.rx_ctrl().modify(|v| {
                v.set_r_tog(match tog {
                    DataTog::DATA0 => Tog::DATA0,
                    DataTog::DATA1 => Tog::DATA1,
                });
            });
        });
        h.ep_pid().write(|v| {
            v.set_endp(endpoint);
            v.set_token(Pid::IN as u8);
        });

        poll_fn(|ctx| {
            PIPE_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();
            if transfer {
                // First stop sending
                h.ep_pid().write(|_| {});
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
                    Pid::NAK => Err(UsbHostError::NAK),
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

    async fn data_out(&mut self, endpoint: u8, tog: DataTog, buf: &[u8]) -> Result<(), UsbHostError> {
        if buf.len() > MAX_PACKET_SIZE {
            return Err(UsbHostError::BufferOverflow);
        }

        let h = T::hregs();

        self.tx_buf.write_volatile(buf);
        h.tx_len().write(|v| v.set_len(buf.len() as u16));
        critical_section::with(|_| {
            h.tx_ctrl().modify(|v| {
                v.set_t_tog(match tog {
                    DataTog::DATA0 => Tog::DATA0,
                    DataTog::DATA1 => Tog::DATA1,
                });
            });
        });
        h.ep_pid().write(|v| {
            // TODO: questionable
            v.set_endp(endpoint);
            v.set_token(Pid::OUT as u8);
        });

        poll_fn(|ctx| {
            PIPE_WAKER.register(ctx.waker());
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
