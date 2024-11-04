use core::marker::PhantomData;

use crate::interrupt::typelevel::Interrupt;
use ch32_metapac::usbhs::vals::SpeedType;
use embassy_hal_internal::Peripheral;

use crate::{
    gpio::{AFType, Speed},
    interrupt,
};

use super::{DmPin, DpPin};

const MAX_PACKET_SIZE: usize = 64;
static EP_TX_BUF: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];
static EP_RX_BUF: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let h = T::hregs();
        let flag = r.int_fg().read();

        if flag.detect() {
            let misc_status = r.mis_st().read();
            if misc_status.dev_attach() {
                trace!("dev attach");
            } else {
                trace!("dev disconnect");
            }
            r.int_fg().write(|w| w.set_detect(true));
        }
    }
}

pub fn start<'d, T: Instance>(
    dp: impl Peripheral<P = impl DpPin<T, 0> + 'd>,
    dm: impl Peripheral<P = impl DmPin<T, 0> + 'd>,
) {
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

    r.ctrl().write(|w| {});

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
    h.rx_dma().write_value((&EP_RX_BUF).as_ptr() as u32);
    h.tx_dma().write_value((&EP_TX_BUF).as_ptr() as u32);

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

    loop {
        let r = T::regs();
        let h = T::hregs();
        let flag = r.int_fg().read();

        if flag.detect() {
            let misc_status = r.mis_st().read();
            if misc_status.dev_attach() {
                trace!("dev attach");
            } else {
                trace!("dev disconnect");
            }
            r.int_fg().write(|w| w.set_detect(true));
        }
    }
}

/// USB endpoint.
trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::usbhs::Usb;
    fn dregs() -> crate::pac::usbhs::Usbd;
    fn hregs() -> crate::pac::usbhs::Usbh;
}

/// UsbHs peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Regular interrupt for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
    /// Wakeup interrupt for this instance
    type WakeupInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usbhs, $inst:ident) => {
        use crate::peripherals;
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usbhs::Usb {
                crate::pac::$inst
            }

            fn dregs() -> crate::pac::usbhs::Usbd {
                unsafe {
                    crate::pac::usbhs::Usbd::from_ptr(crate::pac::$inst.as_ptr())
                }
            }

            fn hregs() -> crate::pac::usbhs::Usbh {
                unsafe {
                    crate::pac::usbhs::Usbh::from_ptr(crate::pac::$inst.as_ptr())
                }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type WakeupInterrupt = crate::_generated::peripheral_interrupts::$inst::WAKEUP;
        }
    };
);
