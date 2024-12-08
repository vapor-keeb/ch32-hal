use core::marker::PhantomData;

use bus::Bus;
use ch32_metapac::usbhs::vals::SpeedType;
use embassy_hal_internal::Peripheral;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::{
    gpio::{AFType, Speed},
    interrupt,
    usb::EndpointDataBuffer,
    usbhs::Instance,
};

pub mod bus;

const MAX_PACKET_SIZE: usize = 64;

pub(crate) static BUS_WAKER: AtomicWaker = AtomicWaker::new();
pub(crate) static TX_WAKER: AtomicWaker = AtomicWaker::new();
// static RX_WAKER: AtomicWaker = AtomicWaker::new();

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
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

pub struct USBHsHostDriver<'d, T: Instance> {
    _phantom: PhantomData<T>,
    tx_buf: &'d mut EndpointDataBuffer,
    rx_buf: &'d mut EndpointDataBuffer,
}

impl<'d, T: Instance> USBHsHostDriver<'d, T> {
    pub fn new(
        dp: impl Peripheral<P = impl super::DpPin<T, 0> + 'd>,
        dm: impl Peripheral<P = impl super::DmPin<T, 0> + 'd>,
        tx_buf: &'d mut EndpointDataBuffer,
        rx_buf: &'d mut EndpointDataBuffer,
    ) -> USBHsHostDriver<'d, T> {
        let dp = dp.into_ref();
        let dm = dm.into_ref();

        dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
        dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

        USBHsHostDriver {
            tx_buf,
            rx_buf,
            _phantom: PhantomData,
        }
    }
}

impl<'d, T: Instance> async_usb_host::Driver for USBHsHostDriver<'d, T> {
    type Bus = Bus<'d, T>;

    fn start(self) -> Self::Bus {
        let USBHsHostDriver {
            _phantom,
            tx_buf,
            rx_buf,
        } = self;
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

        h.ctrl().write(|w|  w.set_phy_suspendm(true));

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

        Bus::new(tx_buf, rx_buf)
    }
}
