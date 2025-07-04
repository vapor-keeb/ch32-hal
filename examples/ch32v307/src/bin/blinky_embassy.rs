#![no_std]
#![no_main]

use core::panic::PanicInfo;

use ch32_hal::gpio::{AnyPin, Level, Output, Pin, Speed};
use ch32_hal::time::Hertz;
use ch32_hal::{rcc, Config};
use embassy_executor::Spawner;
use embassy_time::Timer;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    critical_section::with(|_| {

        loop {}
    })
}


#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    const RCC_CFG: rcc::Config = {
        use rcc::*;

        rcc::Config {
            hse: Some(Hse {
                freq: Hertz(8_000_000),
                mode: HseMode::Oscillator,
            }),
            sys: Sysclk::PLL,
            pll_src: PllSource::HSI,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV4,
                mul: PllMul::MUL4,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: LsConfig::default_lsi(),
            hspll_src: HsPllSource::HSE,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV4,
            }),
        }
    };
    let cfg = Config {
        rcc: RCC_CFG,
        ..Default::default()
    };
    let p = ch32_hal::init(Default::default());

    let mut pa15 = Output::new(p.PA15, Level::Low, Speed::High);

    loop {
        pa15.set_low();
        Timer::after_millis(100).await;
        pa15.set_high();
        Timer::after_secs(1).await;
    }
}
