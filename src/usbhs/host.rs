use core::{
    future::{poll_fn, PollFn},
    marker::PhantomData,
    task::{Poll, Waker},
};

use crate::{interrupt::typelevel::Interrupt, usb::EndpointDataBuffer};
use ch32_metapac::usbhs::vals::{HostRxResponse, HostTxResponse, SpeedType, Tog};
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

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(Format))]
#[cfg_attr(not(feature = "defmt"), derive(Debug))]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PID {
    OUT = 0b0001,
    IN = 0b1001,
    SOF = 0b0101,
    SETUP = 0b1101,

    DATA0 = 0b0011,
    DATA1 = 0b1011,
    DATA2 = 0b0111,
    MDATA = 0b1111,

    ACK = 0b0010,
    NAK = 0b1010,
    STALL = 0b1110,
    NYET = 0b0110,

    // PRE and ERR are both 0b1100???
    SPLIT = 0b1000,
    PING = 0b0100,
    Rsvd = 0b0000,
}

impl TryFrom<u8> for PID {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b0001 => Ok(PID::OUT),
            0b1001 => Ok(PID::IN),
            0b0101 => Ok(PID::SOF),
            0b1101 => Ok(PID::SETUP),

            0b0011 => Ok(PID::DATA0),
            0b1011 => Ok(PID::DATA1),
            0b0111 => Ok(PID::DATA2),
            0b1111 => Ok(PID::MDATA),

            0b0010 => Ok(PID::ACK),
            0b1010 => Ok(PID::NAK),
            0b1110 => Ok(PID::STALL),
            0b0110 => Ok(PID::NYET),

            0b1000 => Ok(PID::SPLIT),
            0b0100 => Ok(PID::PING),
            0b0000 => Ok(PID::Rsvd),

            _ => Err(()),
        }
    }
}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

pub struct USBHsHostDriver<'d, T: Instance> {
    _phantom: PhantomData<T>,
    tx_buf: &'d mut EndpointDataBuffer,
    rx_buf: &'d mut EndpointDataBuffer,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[cfg_attr(not(feature = "defmt"), derive(Debug))]
pub enum UsbHostError {
    NAK,
    WrongTog,
    STALL,
    Unknown,
    BufferOverflow,
}

impl<'d, T: Instance> USBHsHostDriver<'d, T> {
    /// Send the 8 byte setup
    pub async fn setup(&mut self, buf: &[u8]) -> Result<(), UsbHostError> {
        let h = T::hregs();

        self.tx_buf.write_volatile(buf);
        h.ep_pid().write(|v| {
            v.set_endp(0);
            v.set_token(PID::SETUP as u8);
        });

        h.tx_len().write(|v| v.set_len(buf.len() as u16));
        h.rx_ctrl().write(|v| {
            v.set_r_tog(Tog::DATA0);
        });
        h.tx_ctrl().write(|v| {
            v.set_t_tog(Tog::DATA0);
            v.set_t_res(HostTxResponse::ACK);
        });

        assert!(!h.int_fg().read().transfer());

        poll_fn(|ctx| {
            TX_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();

            if transfer {
                // First stop sending more setup
                h.ep_pid().write(|_| {});

                // Check what the device responded
                let device_response = unwrap!(TryInto::<PID>::try_into(status.h_res()));
                let res = match device_response {
                    PID::ACK => Ok(()),
                    PID::NAK => Err(UsbHostError::NAK),
                    PID::STALL => Err(UsbHostError::STALL),
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

    pub async fn data_in(&mut self, buf: &mut [u8]) -> Result<usize, UsbHostError> {
        let h = T::hregs();
        // Send IN token to allow the bytes to come in
        critical_section::with(|_| {
            h.rx_ctrl().modify(|v| {
                v.set_r_tog(match v.r_tog() {
                    Tog::DATA0 => Tog::DATA1,
                    Tog::DATA1 => Tog::DATA0,
                    _ => panic!()
                });
            });
        });
        h.ep_pid().write(|v| {
            v.set_endp(0);
            v.set_token(PID::IN as u8);
        });

        poll_fn(|ctx| {
            TX_WAKER.register(ctx.waker());
            let transfer = h.int_fg().read().transfer();
            let status = h.int_st().read();
            if transfer {
                // First stop sending more setup
                h.ep_pid().write(|_| {});
                trace!("status: {}", status.0);
                let res = match unwrap!(TryInto::<PID>::try_into(status.h_res())) {
                    PID::DATA0 |
                    PID::DATA1 => {
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
                    },
                    PID::NAK => panic!("NAK"),
                    PID::STALL => {
                        Err(UsbHostError::STALL)
                    },
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
