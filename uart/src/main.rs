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
        gpio::{Pins, Pin, FunctionUart, PullDown},
        gpio::bank0::{Gpio0, Gpio1},
        fugit::RateExtU32,
        sio::Sio,
        uart::{UartPeripheral, UartConfig, DataBits, StopBits, Enabled}
    };
    
    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        uart : UartPeripheral<Enabled, you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::UART0, 
            (Pin<Gpio0, FunctionUart, PullDown>, Pin<Gpio1, FunctionUart, PullDown>)>
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

        let tx = pins.gpio0.into_function::<FunctionUart>();
        let rx = pins.gpio1.into_function::<FunctionUart>();
        let uart_pins = (tx, rx);
        let uart_config = UartConfig::new(115_200u32.Hz(), DataBits::Eight, None, StopBits::One);
        let uart = UartPeripheral::<_, _, _>::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(uart_config, clocks.peripheral_clock.freq()).unwrap();

        task::spawn().ok();

        (Shared {
        }, Local {
            uart
        })
    }

    #[task(priority = 1, local = [uart])]
    async fn task(ctx : task::Context) {
        let mut msg = [0u8; 32];
        let start = b"hello world\n";
        msg[..start.len()].copy_from_slice(start);
        let mut buffer = [0u8; 32];
        loop {
            let msg_len = msg.iter().position(|&b| b==0).unwrap_or(msg.len());
            ctx.local.uart.write_raw(&msg[..msg_len]).unwrap();
            if ctx.local.uart.read_raw(&mut buffer).is_ok() {
                msg[..buffer.len()].copy_from_slice(&buffer);
                info!("tx:{}, rx:{}", msg, buffer);
            }
            Mono::delay(1_000.millis()).await;
        }
    }
}