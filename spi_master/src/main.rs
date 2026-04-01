#![no_std]
#![no_main]

#[unsafe(link_section = ".boot2")]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use panic_probe as _;
//use panic_halt as _;
//use defmt::*;
use defmt as _;
use defmt::info;
use defmt_rtt as _;
use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1_000);

#[rtic::app(device = rp2040_hal::pac, 
    peripherals = true, 
    dispatchers = [RTC_IRQ],
)]
mod app {
    use super::*;
    use cortex_m::asm::bkpt;
    use rp2040_hal as hal;
    use hal::{
        Clock,
        clocks::init_clocks_and_plls,
        sio::Sio,
        spi::{Spi, Enabled},
        gpio::{Pins, Pin, FunctionSpi, FunctionSioOutput, PullDown},
        gpio::bank0::{Gpio3, Gpio0, Gpio2, Gpio1},
        fugit::RateExtU32
    };
    use embedded_hal::{digital::OutputPin, spi::SpiBus};
    //use heapless::String;
    
    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        cs : Pin<Gpio1, FunctionSioOutput, PullDown>,
        spi : Spi<Enabled, you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::SPI0, 
            (Pin<Gpio3, FunctionSpi, PullDown>, Pin<Gpio0, FunctionSpi, PullDown>, Pin<Gpio2, FunctionSpi, PullDown>)>,
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        bkpt();

        let mut pac = ctx.device;
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
        let clocks = init_clocks_and_plls(12_000_000, 
            pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, 
            &mut pac.RESETS, &mut watchdog,).ok().unwrap();

        Mono::start(ctx.core.SYST, 125_000_000);
        
        let sio = Sio::new(pac.SIO);
        let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0,
            sio.gpio_bank0, &mut pac.RESETS,);

        let mut cs = pins.gpio1.into_function::<FunctionSioOutput>();
        let sck = pins.gpio2.into_function::<FunctionSpi>();
        let miso = pins.gpio0.into_function::<FunctionSpi>();
        let mosi = pins.gpio3.into_function::<FunctionSpi>();
        let spi_pins = (mosi, miso, sck);
        cs.set_high().ok();
        let spi : Spi<_, _, _, 8> = Spi::new(pac.SPI0, spi_pins)
            .init(&mut pac.RESETS, clocks.peripheral_clock.freq(), 1_000_000u32.Hz(), embedded_hal::spi::MODE_0);

        task::spawn().ok();

        (Shared {
        }, Local {
            cs, spi
        })
    }

    #[task(priority = 1, local = [cs, spi])]
    async fn task(ctx : task::Context) {
        loop {
            let msg = b"hello world";
            ctx.local.cs.set_low().ok();
            ctx.local.spi.write(msg).unwrap();
            ctx.local.cs.set_high().ok();
            info!("hello world");
            Mono::delay(1_000.millis()).await;
        }
    }
}