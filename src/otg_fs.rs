use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use defmt::trace;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    self as driver, Direction, EndpointAddress, EndpointError, EndpointInfo, EndpointType, Event,
};

use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::peripherals::OTG_FS;
use crate::{interrupt, peripherals, Peripheral, RccPeripheral};

const NR_EP: usize = 16;
const MAX_EP_OUT_BUFFER: u16 = 64;

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
        let int_fg = regs.int_fg().read();
        trace!(
            "int fg {:#b}: RST:{1=0..1:b}, XFER: {1=1..2:b}, SUSPEND: {1=2..3:b}, FIFO_OV: {1=4..5:b}",
            int_fg.0,
            int_fg.0
        );

        if int_fg.bus_rst() || int_fg.suspend() {
            trace!("Interrupt Upper IRQ: {}", T::Interrupt::is_enabled());
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
    ep_out_buffer: &'d mut [EpOutBuffer],
    ep_out_buffer_next: usize,
}

#[derive(Debug, Clone, Copy)]
struct EndpointData {
    ep_type: EndpointType,
    max_packet_size: u16,
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct EpOutBuffer {
    data: [u8; MAX_EP_OUT_BUFFER as usize],
}

impl Default for EpOutBuffer {
    fn default() -> Self {
        unsafe {
            EpOutBuffer {
                data: core::mem::zeroed(),
            }
        }
    }
}

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;

impl<'d, T> Driver<'d, T>
where
    T: Instance,
{
    pub fn new(
        _usb: impl Peripheral<P = T> + 'd,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: impl Peripheral<P = impl crate::gpio::Pin> + 'd,
        dm: impl Peripheral<P = impl crate::gpio::Pin> + 'd,
        ep_out_buffer: &'d mut [EpOutBuffer],
    ) -> Self {
        assert!(ep_out_buffer.len() > 0);
        let dp = dp.into_ref();
        let dm = dm.into_ref();

        dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
        dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

        T::enable_and_reset();

        Self {
            phantom: PhantomData,
            ep_alloc: [None; NR_EP],
            ep_out_buffer,
            ep_out_buffer_next: 1,
            ep_0_in: EndpointData {
                ep_type: EndpointType::Control,
                max_packet_size: 64,
            },
            ep_0_out: EndpointData {
                ep_type: EndpointType::Control,
                max_packet_size: 64,
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
        trace!("start");
        let regs = T::regs();

        regs.ctrl().modify(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        regs.ctrl().modify(|w| {
            w.0 = 0;
        });

        regs.int_en().write(|w| {
            w.set_dev_nak(true);
            w.set_fifo_ov(true);

            // Host SOF is ignored, not our usecase here

            // w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });

        regs.ctrl().modify(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_dev_pu_en(true);
        });

        critical_section::with(|_cs| {
            T::Interrupt::unpend();
            unsafe {
                T::Interrupt::enable();
            }
        });

        regs.uep_dma(0)
            .write_value(self.ep_out_buffer[0].data.as_mut_ptr() as u32);

        // RX(0) = ACK
        regs.uep_rx_ctrl(0).write(|w| w.set_mask_r_res(0b00));
        // TX(0) = NAK
        regs.uep_tx_ctrl(0).write(|w| w.set_mask_t_res(0b10));

        // Initialize the bus so that it signals that power is available
        // usbd.rs does BUS_WAKER.wake(), but it doesn't seem necessary

        let ep_out = Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(0, Direction::Out),
                ep_type: EndpointType::Control,
                max_packet_size: 64,
                interval_ms: 0,
            },
        };

        let ep_in = Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(0, Direction::In),
                ep_type: EndpointType::Control,
                max_packet_size: 64,
                interval_ms: 0,
            },
        };

        // Hookup the bus on start?
        regs.udev_ctrl().write(|w| {
            // different from reference code
            // board has no pulldown
            w.set_pd_dis(false);
            w.set_port_en(true);
        });

        (
            Bus {
                _phantom: PhantomData,
                inited: false,
            },
            ControlPipe {
                ep_in,
                ep_out,
                setup_state: &T::state().control_state,
            },
        )
    }
}

/// USB endpoint direction.
trait Dir {
    /// Returns the direction value.
    fn dir() -> Direction;
}

/// Marker type for the "IN" direction.
pub struct In;
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for the "OUT" direction.
pub struct Out;
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// USB endpoint.
pub struct Endpoint<'d, T, D> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
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
    setup_state: &'d ControlPipeSetupState,
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
        poll_fn(|ctx| {
            BUS_WAKER.register(ctx.waker());
            trace!("poll");

            // LOL VBUS detection
            if !self.inited {
                self.inited = true;
                return Poll::Ready(Event::PowerDetected);
            }

            let regs = T::regs();
            let interrupt_flags = regs.int_fg().read();
            let misc_status = regs.mis_st().read();

            // Either Suspend or Resume has happened
            if interrupt_flags.suspend() {
                // Clear suspend flag
                regs.int_fg().write(|v| v.set_suspend(true));

                if misc_status.suspend() {
                    trace!("Suspend");
                    return Poll::Ready(Event::Suspend);
                } else {
                    trace!("resume");
                    return Poll::Ready(Event::Resume);
                }
            }

            // Somehow detect a bus reset needs to happen
            // todo!();

            // unsafe { T::Interrupt::enable() };
            critical_section::with(|_| {
                regs.int_fg().modify(|_| {});
                trace!("Interrupt Enabled? {}", T::Interrupt::is_enabled());
                trace!(
                    "Pend? {}, IntFG: {:#x}, IntSt: {:#x}",
                    T::Interrupt::is_pending(),
                    regs.int_fg().read().0,
                    regs.int_st().read().0,
                );
                unsafe { T::Interrupt::enable() };
            });

            trace!("Bus pend");
            Poll::Pending
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
        usize::from(MAX_EP_OUT_BUFFER)
    }

    async fn setup(&mut self) -> [u8; 8] {
        trace!("setup");
        /*
        loop {
            let intfg = T::regs().int_fg().read();
            let status = T::regs().int_st().read();

            loop {
                if !intfg.transfer() {
                    break;
                }
                // setup packet
                if status.mask_token() != 0b11 {
                    break;
                }

                T::regs().uep_tx_ctrl(0).write(|w| {
                    w.set_t_tog(true);
                    // NAK
                    w.set_mask_t_res(0b10);
                });
                T::regs().uep_rx_ctrl(0).write(|w| {
                    w.set_r_tog(true);
                    // NAK
                    w.set_mask_r_res(0b10);
                });
            }
            T::regs().int_fg().write_value(intfg);
        }
         */

        let mut x = false;
        poll_fn(move |ctx| {
            BUS_WAKER.register(ctx.waker());

            if !self.setup_state.setup_ready.load(Ordering::Acquire) {
                self.setup_state.setup_ready.store(true, Ordering::Relaxed);
            } else {
            }

            let regs = T::regs();
            trace!(
                "Interrupt?: {}, {}",
                T::Interrupt::is_enabled(),
                T::Interrupt::is_pending()
            );
            trace!(
                "Pend? {}, IntFG: {:#x}, IntSt: {:#x}",
                T::Interrupt::is_pending(),
                regs.int_fg().read().0,
                regs.int_st().read().0,
            );

            trace!("setup Pend");
            Poll::Pending
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        todo!()
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        todo!()
    }

    async fn accept(&mut self) {
        todo!()
    }

    async fn reject(&mut self) {
        todo!()
    }

    async fn accept_set_address(&mut self, addr: u8) {
        todo!()
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
