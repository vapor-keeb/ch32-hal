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

    fn handle_device_response(status: ch32_metapac::usbhs::regs::IntSt) -> Result<Pid, UsbHostError> {
        let hres = status.h_res();
        TryInto::<Pid>::try_into(hres).or_else(|_| {
            #[cfg(feature = "defmt")]
            error!("Invalid PID value: {:x}", hres);
            Err(UsbHostError::UnexpectedPID)
        })
    }
}

impl<'d, T: Instance> async_usb_host::Pipe for Pipe<'d, T> {
    fn set_addr(&mut self, addr: u8) {
        let h = T::hregs();
        h.dev_ad().write(|v| v.set_addr(addr));
    }

    /// Send the 8 byte setup
    async fn setup(&mut self, buf: &[u8]) -> Result<(), UsbHostError> {
        assert!(buf.len() == 8 || buf.is_empty(), "Setup packet must be 8 bytes long or empty (CSPLIT)");
        let h = T::hregs();

        self.tx_buf.write_volatile(buf);
        h.tx_len().write(|v| v.set_len(buf.len() as u16));
        h.rx_ctrl().write(|v| {
            v.set_r_tog(Tog::DATA0);
        });
        h.tx_ctrl().write(|v| {
            v.set_t_tog(Tog::DATA0);
            v.set_t_res(HostTxResponse::ACK);
            v.set_t_data_no(false); // Expect to write data packets
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
                let device_response = Self::handle_device_response(status)?;
                let res = match device_response {
                    Pid::ACK => Ok(()),
                    Pid::NAK => Err(UsbHostError::NAK),
                    Pid::STALL => Err(UsbHostError::STALL),
                    Pid::NYET => Err(UsbHostError::NYET),
                    r => {
                        #[cfg(feature = "defmt")]
                        error!("Unexpected PID: {:?}", r);
                        Err(UsbHostError::UnexpectedPID)
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

    async fn split(&mut self, complete: bool, port: u8, ep_type: u8) -> Result<(), UsbHostError> {
        let hregs = T::hregs();
        defmt::assert!(hregs.mis_st().read().split_can(), "can't split");

        critical_section::with(|_| {
            hregs.tx_ctrl().modify(|v| v.set_t_data_no(true));
        });

        // ET 2b | E(0) ??? 1b | S (0) 1b | Port 7b | C(1)/S(0) 1b
        let split_data = ((ep_type as u16 & 0x3) << 10) | ((port as u16 & 0x7F) << 1) | complete as u16;
        hregs.split_data().write(|v| v.set_split_data(split_data));
        hregs.ep_pid().write(|v| v.set_token(Pid::SPLIT as u8));

        poll_fn(|ctx| {
            PIPE_WAKER.register(ctx.waker());
            let transfer = hregs.int_fg().read().transfer();

            if transfer {
                // First stop sending more setup
                hregs.ep_pid().write(|_| {});

                hregs.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| hregs.int_en().modify(|w| w.set_transfer(true)));
                Poll::Ready(Ok(()))
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

                let device_response = Self::handle_device_response(status)?;
                let res = match device_response {
                    Pid::DATA0 | Pid::DATA1 => {
                        if status.tog_ok() {
                            let bytes_read = h.rx_len().read() as usize;
                            defmt::debug_assert!(bytes_read <= 64); // TODO: FIX THIS when we have a size for self.rx_buf
                            if bytes_read > buf.len() {
                                Err(UsbHostError::BufferOverflow)
                            } else {
                                self.rx_buf.read_volatile(&mut buf[..bytes_read]);
                                Ok(bytes_read)
                            }
                        } else {
                            #[cfg(feature = "defmt")]
                            error!("Wrong TOG");
                            Err(UsbHostError::WrongTog)
                        }
                    }
                    Pid::NAK => Err(UsbHostError::NAK),
                    Pid::STALL => Err(UsbHostError::STALL),
                    Pid::NYET => Err(UsbHostError::NYET),
                    pid => {
                        #[cfg(feature = "defmt")]
                        error!("Unexpected PID: {:?}", pid);
                        Err(UsbHostError::UnexpectedPID)
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
                v.set_t_data_no(false); // Expect to write data packets
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
                let device_response = Self::handle_device_response(status)?;
                let res = match device_response {
                    Pid::ACK => Ok(()),
                    Pid::NAK => Err(UsbHostError::NAK),
                    Pid::STALL => Err(UsbHostError::STALL),
                    Pid::NYET => Err(UsbHostError::NYET),
                    pid => {
                        #[cfg(feature = "defmt")]
                        error!("Unexpected PID: {:?}", pid);
                        Err(UsbHostError::UnexpectedPID)
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
}
