use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::panic;
use core::sync::atomic::{fence, AtomicBool, Ordering};
use core::task::Poll;

use ch32_metapac::otg::vals::{EpRxResponse, EpTxResponse, UsbToken};
use defmt::{debug, error, info, trace, warn};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    self as driver, Direction, EndpointAddress, EndpointError, EndpointIn, EndpointInfo, EndpointType, Event,
};
use marker::{In, Out};

use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::peripherals::OTG_FS;
use crate::{interrupt, peripherals, Peripheral, RccPeripheral};

pub mod marker;

const NR_EP: usize = 16;
const EP_MAX_PACKET_SIZE: u16 = 64;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
// TODO well... not like this
static EP0_WAKER: AtomicWaker = NEW_AW;

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

#[qingke_rt::interrupt]
unsafe fn OTG_FS() {
    InterruptHandler::<OTG_FS>::on_interrupt();
}

/* Interrupt Handlers

   AFAICT, LP is triggered for **ALL** events, and HP is only triggered on
   "double buffered" bulk transfers and iso transfers. I think we can safely
   ignore that for now. WKUP is for well USB WakeUp events
*/
impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        // Clear the host sof, we are in device mode.... It's garbage
        let int_fg = regs.int_fg().read();

        if int_fg.fifo_ov() {
            panic!("overflow");
        }

        // All the interrupt we handle
        if int_fg.suspend() || int_fg.transfer() || int_fg.bus_rst() {
            T::Interrupt::disable();

            // Bus stuff we wakup bus
            if int_fg.bus_rst() || int_fg.suspend() {
                BUS_WAKER.wake();
            }

            if int_fg.transfer() {
                let status = regs.int_st().read();

                let token = status.mask_token();
                match token {
                    UsbToken::IN | UsbToken::OUT => {
                        let ep = status.mask_uis_endp();
                        if ep == 0 {
                            EP0_WAKER.wake();
                        } else {
                            trace!("[IRQ USBFS] Transfer Token: {:#x}", token.to_bits());
                            error!("Unexpected EP: {}", ep);
                        }
                    }
                    UsbToken::SETUP => {
                        EP0_WAKER.wake();
                    }
                    UsbToken::RSVD => panic!("rsvd token"),
                }
            }
        }

        if int_fg.hst_sof() {
            regs.int_fg().write(|v| v.set_hst_sof(true));
        }
    }
}

struct ControlPipeSetupState {
    setup_data: UnsafeCell<[u8; 8]>,
    setup_ready: AtomicBool,
}

struct State {
    control_state: ControlPipeSetupState,
}

impl State {
    const fn new() -> Self {
        State {
            control_state: ControlPipeSetupState {
                setup_data: UnsafeCell::new([0u8; 8]),
                setup_ready: AtomicBool::new(false),
            },
        }
    }
}

unsafe impl Send for State {}
unsafe impl Sync for State {}

pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    ep_0_in: EndpointData,
    ep_0_out: EndpointData,
    ep_alloc: [Option<EndpointData>; NR_EP],
    ep_out_buffer: &'d mut [EndpointDataBuffer],
    ep_out_buffer_next: usize,
}

#[derive(Debug, Clone, Copy)]
struct EndpointData {
    ep_type: EndpointType,
    max_packet_size: u16,
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct EndpointDataBuffer {
    data: [u8; EP_MAX_PACKET_SIZE as usize],
}

impl Default for EndpointDataBuffer {
    fn default() -> Self {
        unsafe {
            EndpointDataBuffer {
                data: core::mem::zeroed(),
            }
        }
    }
}

impl<'d, T> Driver<'d, T>
where
    T: Instance,
{
    pub fn new(
        _usb: impl Peripheral<P = T> + 'd,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: impl Peripheral<P = impl crate::gpio::Pin> + 'd,
        dm: impl Peripheral<P = impl crate::gpio::Pin> + 'd,
        ep_out_buffer: &'d mut [EndpointDataBuffer],
    ) -> Self {
        assert!(ep_out_buffer.len() > 0);
        let dp = dp.into_ref();
        let dm = dm.into_ref();

        dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
        dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

        T::enable_and_reset();

        ep_out_buffer[0].data[0..16].copy_from_slice(&[69u8; 16]);

        Self {
            phantom: PhantomData,
            ep_alloc: [None; NR_EP],
            ep_out_buffer,
            ep_out_buffer_next: 1,
            ep_0_in: EndpointData {
                ep_type: EndpointType::Control,
                max_packet_size: EP_MAX_PACKET_SIZE,
            },
            ep_0_out: EndpointData {
                ep_type: EndpointType::Control,
                max_packet_size: EP_MAX_PACKET_SIZE,
            },
        }
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;

    type EndpointIn = Endpoint<'d, T, In>;

    type ControlPipe = ControlPipe<'d, T>;

    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: driver::EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
        todo!();
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: driver::EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        todo!()
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size <= EP_MAX_PACKET_SIZE);
        trace!("start");
        let regs = T::regs();

        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        // Clear all
        regs.ctrl().write(|_| {});

        regs.int_en().write(|w| {
            // w.set_dev_nak(true);
            // w.set_fifo_ov(true);

            // Host SOF is ignored, not our usecase here

            w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });

        regs.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_dev_pu_en(true);
        });

        regs.uep_dma(0)
            .write_value(self.ep_out_buffer[0].data.as_mut_ptr() as u32);

        regs.uep_rx_ctrl(0).write(|w| w.set_mask_r_res(EpRxResponse::ACK));
        regs.uep_tx_ctrl(0).write(|w| w.set_mask_t_res(EpTxResponse::NAK));

        // Hookup the bus on start?
        regs.udev_ctrl().write(|w| {
            // pd is for HOST
            w.set_pd_dis(true);
            w.set_port_en(true);
        });

        // Initialize the bus so that it signals that power is available
        // usbd.rs does BUS_WAKER.wake(), but it doesn't seem necessary
        BUS_WAKER.wake();

        critical_section::with(|_cs| {
            T::Interrupt::unpend();
            unsafe {
                T::Interrupt::enable();
            }
        });

        let ep_out = Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(0, Direction::Out),
                ep_type: EndpointType::Control,
                max_packet_size: 64,
                interval_ms: 0,
            },
            buffer: None,
        };

        let ep_in = Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(0, Direction::In),
                ep_type: EndpointType::Control,
                max_packet_size: 64,
                interval_ms: 0,
            },
            buffer: None,
        };

        (
            Bus {
                _phantom: PhantomData,
                inited: false,
            },
            ControlPipe {
                ep_in,
                ep_out,
                buffer: &mut self.ep_out_buffer[0],
            },
        )
    }
}

/// USB endpoint.
pub struct Endpoint<'d, T, D> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
    buffer: Option<&'d mut EndpointDataBuffer>,
    // state: &'d EpState,
}

impl<'d, T: Instance> driver::Endpoint for Endpoint<'d, T, In> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        todo!()
    }
}

impl<'d, T: Instance> driver::EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        todo!()
    }
}

impl<'d, T: Instance> driver::Endpoint for Endpoint<'d, T, Out> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        todo!()
    }
}

impl<'d, T: Instance> driver::EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        todo!()
    }
}

pub struct ControlPipe<'d, T> {
    ep_in: Endpoint<'d, T, In>,
    ep_out: Endpoint<'d, T, Out>,
    buffer: &'d mut EndpointDataBuffer,
}

pub struct Bus<'d, T> {
    _phantom: PhantomData<&'d T>,
    inited: bool,
}

impl<'d, T> driver::Bus for Bus<'d, T>
where
    T: Instance,
{
    async fn enable(&mut self) {
        trace!("Enable")
    }

    async fn disable(&mut self) {
        // TODO: ???
        trace!("Disable")
    }

    async fn poll(&mut self) -> embassy_usb_driver::Event {
        // TODO: VBUS detection
        if !self.inited {
            self.inited = true;
            return Event::PowerDetected;
        }

        poll_fn(|ctx| {
            BUS_WAKER.register(ctx.waker());
            trace!("bus poll");

            critical_section::with(|_| {
                unsafe { T::Interrupt::enable() };

                let regs = T::regs();
                let interrupt_flags = regs.int_fg().read();

                // Either Suspend or Resume has happened
                if interrupt_flags.suspend() {
                    // Clear suspend flag
                    regs.int_fg().write(|v| v.set_suspend(true));

                    if regs.mis_st().read().suspend() {
                        return Poll::Ready(Event::Suspend);
                    } else {
                        return Poll::Ready(Event::Resume);
                    }
                }

                if interrupt_flags.bus_rst() {
                    trace!("bus: reset");
                    regs.dev_ad().write(|v| {
                        v.set_mask_usb_addr(0);
                    });

                    // Reset endpoint state on bus reset
                    regs.uep_rx_ctrl(0).write(|v| v.set_mask_r_res(EpRxResponse::ACK));
                    regs.uep_tx_ctrl(0).write(|v| v.set_mask_t_res(EpTxResponse::NAK));

                    regs.int_fg().write(|v| {
                        v.set_bus_rst(true);
                        // Also clear transfer?
                        // v.set_transfer(true);
                    });

                    return Poll::Ready(Event::Reset);
                }

                Poll::Pending
            })
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        todo!()
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        todo!()
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        todo!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        todo!()
    }
}

impl<'d, T> embassy_usb_driver::ControlPipe for ControlPipe<'d, T>
where
    T: Instance,
{
    fn max_packet_size(&self) -> usize {
        usize::from(EP_MAX_PACKET_SIZE)
    }

    async fn setup(&mut self) -> [u8; 8] {
        trace!("setup");
        poll_fn(move |ctx| {
            EP0_WAKER.register(ctx.waker());
            critical_section::with(|_| {
                unsafe { T::Interrupt::enable() };

                let regs = T::regs();
                let int_flags = regs.int_fg().read();
                if int_flags.transfer() {
                    let int_status = regs.int_st().read();

                    match int_status.mask_token() {
                        UsbToken::SETUP => {
                            // SETUP packet token
                            regs.uep_tx_ctrl(0).write(|w| {
                                w.set_t_tog(true);
                                w.set_mask_t_res(EpTxResponse::NAK);
                            });
                            regs.uep_rx_ctrl(0).write(|w| {
                                w.set_r_tog(true);
                                w.set_mask_r_res(EpRxResponse::NAK);
                            });
                            let mut data = [0u8; 8];

                            // Should be acquire
                            fence(Ordering::SeqCst);
                            data.copy_from_slice(&self.buffer.data[0..8]);
                            fence(Ordering::SeqCst);

                            // Clear Flag
                            regs.int_fg().write(|v| v.set_transfer(true));
                            return Poll::Ready(data);
                        }
                        _ => {
                            error!(
                                "Unexpected packet: {}, len: {}",
                                int_status.mask_token().to_bits(),
                                regs.rx_len().read().rx_len()
                            );
                            panic!()
                        }
                    }
                }
                Poll::Pending
            })
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, EndpointError> {
        trace!("data_out");
        assert!(buf.len() <= EP_MAX_PACKET_SIZE as usize, "out buf too large");
        assert!(first && last, "TODO support chunking");

        let regs = T::regs();

        // Tx Ctrl should be NAK

        assert!(!regs.int_fg().read().transfer(), "transfer should not be pending");

        // Really.... unnessrary but lollll
        // Should be release
        fence(Ordering::SeqCst);

        regs.uep_rx_ctrl(0).write(|v| {
            v.set_r_tog(true);
            v.set_mask_r_res(EpRxResponse::ACK);
        });

        // poll for packet
        let bytes_read = poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() != 0 {
                    // Unexpected
                    regs.int_fg().write(|v| v.set_transfer(true));
                    error!("Expected STATUS stage saw non ep0, aborting");
                    Poll::Ready(Err(EndpointError::Disabled))
                } else {
                    let token = status.mask_token();
                    trace!("[IRQ USBFS] Transfer Token: {:#x}", token.to_bits());

                    let ret = match token {
                        UsbToken::RSVD | UsbToken::IN => unreachable!(),
                        UsbToken::SETUP => Poll::Ready(Err(EndpointError::Disabled)),
                        UsbToken::OUT => {
                            // upper bits are reserved (0)
                            // TODO fix metapac
                            let len = regs.rx_len().read().0 as usize;
                            // TODO debug assert
                            // https://github.com/embassy-rs/embassy/blob/6e0b08291b63a0da8eba9284869d1d046bc5dabb/embassy-usb/src/lib.rs#L408
                            assert_eq!(len, buf.len());
                            if len != buf.len() {
                                Poll::Ready(Err(EndpointError::BufferOverflow))
                            } else {
                                // Should be Acquire
                                fence(Ordering::SeqCst);
                                buf.copy_from_slice(&self.buffer.data[..len]);
                                Poll::Ready(Ok(len))
                            }
                        }
                    };

                    regs.int_fg().write(|v| v.set_transfer(true));
                    ret
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            ret
        })
        .await?;

        regs.uep_rx_ctrl(0).write(|v| {
            v.set_r_tog(true);
            v.set_mask_r_res(EpRxResponse::NAK);
        });

        regs.uep_t_len(0).write_value(0);
        regs.uep_tx_ctrl(0).write(|v| {
            v.set_t_tog(true);
            v.set_mask_t_res(EpTxResponse::ACK);
        });

        // TODO clean up/decide critical section
        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());
            critical_section::with(|_| {
                unsafe { T::Interrupt::enable() };

                if regs.int_fg().read().transfer() {
                    let status = regs.int_st().read();
                    if status.mask_uis_endp() != 0 {
                        // Unexpected
                        regs.int_fg().write(|v| v.set_transfer(true));
                        error!("Expected STATUS stage saw non ep0, aborting");
                        return Poll::Ready(Err(EndpointError::Disabled));
                    }
                    if status.mask_token() == UsbToken::IN {
                        if regs.rx_len().read().0 != 0 {
                            error!("Expected 0 len OUT stage, found non-zero len, aborting");
                            return Poll::Ready(Err(EndpointError::Disabled));
                        }
                        // Set the EP back to NAK so that we are "not ready to recieve"
                        regs.uep_tx_ctrl(0).write(|v| {
                            v.set_t_tog(true);
                            v.set_mask_t_res(EpTxResponse::NAK);
                        });
                        regs.int_fg().write(|v| v.set_transfer(true));
                        Poll::Ready(Ok(()))
                    } else {
                        regs.int_fg().write(|v| v.set_transfer(true));
                        Poll::Ready(Err(EndpointError::Disabled))
                    }
                } else {
                    Poll::Pending
                }
            })
        })
        .await?;

        trace!("data_out: (hex) {:x}", buf[..bytes_read]);
        Ok(bytes_read)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        trace!("data_in: (hex) {:x}", data);
        let regs = T::regs();

        if data.len() > self.ep_in.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        assert!(first && last, "TODO, handle other cases");

        // AFAIK this is already true
        // regs.uep_rx_ctrl(0).write(|v| {
        //     // Mark the RX endpoint as "NAK", because we can't really do
        //     // anything, the EP buffer will be occupied by the out going data.
        //     v.set_mask_r_res(EpRxResponse::NAK);
        // });

        // Deposit data in dma buffer
        self.buffer.data[..data.len()].copy_from_slice(data);

        // Should be release
        fence(Ordering::SeqCst);

        // TODO: manual is wrong here, t_len(3) should be a u16
        regs.uep_t_len(0).write_value(data.len() as u8);

        regs.uep_tx_ctrl(0).write(|v| {
            v.set_mask_t_res(EpTxResponse::ACK);
            // ? not sure why but following their example
            v.set_t_tog(true);
        });

        // Last packet we need to wait for it to finish
        if last {
            // Poll for last packet to finsh transfer
            poll_fn(|ctx| {
                EP0_WAKER.register(ctx.waker());
                let poll_result = {
                    let int_flags = regs.int_fg().read();
                    if int_flags.transfer() {
                        let status = regs.int_st().read();

                        if status.mask_uis_endp() != 0 {
                            // Unexpected
                            error!("Expected STATUS stage saw non ep0, aborting");
                            Poll::Ready(Err(EndpointError::Disabled))
                        } else {
                            match status.mask_token() {
                                UsbToken::OUT | UsbToken::RSVD => unreachable!(),
                                UsbToken::IN => {
                                    fence(Ordering::SeqCst);
                                    trace!("huh: {:X}", self.buffer.data[..data.len()]);
                                    regs.uep_tx_ctrl(0).write(|v| {
                                        v.set_t_tog(true);
                                        v.set_mask_t_res(EpTxResponse::NAK);
                                    });
                                    regs.uep_rx_ctrl(0).write(|v| {
                                        // Set RX to true to expect the STATUS (OUT) packet
                                        v.set_mask_r_res(EpRxResponse::ACK);
                                        v.set_r_tog(true);
                                    });
                                    regs.int_fg().write(|v| v.set_transfer(true));
                                    Poll::Ready(Ok(()))
                                }
                                UsbToken::SETUP => {
                                    error!("SETUP while data_in, aborting");
                                    // TODO: unsure
                                    // regs.int_fg().write(|v| v.set_transfer(true));
                                    Poll::Ready(Err(EndpointError::Disabled))
                                }
                            }
                        }
                    } else {
                        Poll::Pending
                    }
                };
                unsafe { T::Interrupt::enable() };
                poll_result
            })
            .await?;

            // Expect the empty OUT token for status
            poll_fn(|ctx| {
                EP0_WAKER.register(ctx.waker());

                let poll_res = if regs.int_fg().read().transfer() {
                    let status = regs.int_st().read();
                    let res = if status.mask_uis_endp() != 0 {
                        // Unexpected
                        error!("Expected STATUS stage saw non ep0, aborting");
                        Poll::Ready(Err(EndpointError::Disabled))
                    } else if status.mask_token() == UsbToken::OUT {
                        if regs.rx_len().read().0 != 0 {
                            error!("Expected 0 len OUT stage, found non-zero len, aborting");
                            return Poll::Ready(Err(EndpointError::Disabled));
                        }
                        // Set the EP back to NAK so that we are "not ready to recieve"
                        regs.uep_rx_ctrl(0).write(|v| {
                            v.set_r_tog(true);
                            v.set_mask_r_res(EpRxResponse::NAK);
                        });
                        Poll::Ready(Ok(()))
                    } else {
                        error!("Got {} instead of OUT token", status.mask_token().to_bits());
                        Poll::Ready(Err(EndpointError::Disabled))
                    };
                    regs.int_fg().write(|v| v.set_transfer(true));
                    res
                } else {
                    Poll::Pending
                };

                unsafe { T::Interrupt::enable() };

                poll_res
            })
            .await?;
        }

        trace!("[CP] data_in complete");

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("accept");
        let regs = T::regs();

        // Rx should already be NAK
        regs.uep_t_len(0).write_value(0);
        regs.uep_tx_ctrl(0).write(|v| {
            v.set_t_tog(true);
            v.set_mask_t_res(EpTxResponse::ACK);
        });

        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());

            let res = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                assert_eq!(status.mask_uis_endp(), 0);
                match status.mask_token() {
                    UsbToken::IN => {
                        // Maybe stall? and unstall when we actually set addr
                        regs.uep_tx_ctrl(0).write(|v| {
                            v.set_mask_t_res(EpTxResponse::NAK);
                            v.set_t_tog(true);
                        });
                        regs.int_fg().write(|v| v.set_transfer(true));
                    }
                    UsbToken::OUT | UsbToken::RSVD | UsbToken::SETUP => panic!(),
                }
                Poll::Ready(())
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            res
        })
        .await;
    }

    async fn reject(&mut self) {
        // Untested, unsure how to test
        let regs = T::regs();
        regs.uep_rx_ctrl(0).write(|v| {
            v.set_mask_r_res(EpRxResponse::STALL);
        });
        regs.uep_tx_ctrl(0).write(|v| {
            v.set_mask_t_res(EpTxResponse::STALL);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        // That's it ?
        trace!("Accept addr: {}", addr);
        self.accept().await;
        T::regs().dev_ad().write(|v| {
            v.set_mask_usb_addr(addr);
        });
        trace!("address accepted");
    }
}

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

trait SealedInstance: RccPeripheral {
    fn regs() -> crate::pac::otg::Usbd;
    fn state() -> &'static State;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (otg, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::otg::Usbd {
                // datasheet
                unsafe { crate::pac::otg::Usbd::from_ptr(crate::pac::OTG_FS.as_ptr()) }
            }

            fn state() -> &'static State {
                static STATE: State = State::new();

                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);
