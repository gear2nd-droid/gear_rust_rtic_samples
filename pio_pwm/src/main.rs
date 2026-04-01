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
        clocks::init_clocks_and_plls,
        sio::Sio,
        pio::{StateMachine, PIOExt, PIO0SM0, PIOBuilder, Running, Rx, Tx, PinDir},
        gpio::{Pins, Pin, FunctionPio0, PullDown},
        gpio::bank0::Gpio25,
    };
    use pio_proc::pio_file;
    
    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        led : Pin<Gpio25, FunctionPio0, PullDown>,
        sm : StateMachine<PIO0SM0, Running>,
        rx : Rx<PIO0SM0>,
        tx : Tx<PIO0SM0>,
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

        Mono::start(ctx.core.SYST, 125_000_000);

        let mut pac = ctx.device;
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
        let _ = init_clocks_and_plls(12_000_000, 
            pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, 
            &mut pac.RESETS, &mut watchdog,).ok().unwrap();
        let sio = Sio::new(pac.SIO);
        let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0,
            sio.gpio_bank0, &mut pac.RESETS,);
        
        let led = pins.gpio25.into_function::<FunctionPio0>();
        let (mut pio, pio0sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let program = pio_file!("src/pio_pwm.pio");
        let installed = pio.install(&program.program).unwrap();
        let (mut sm_stop, rx, tx) = PIOBuilder::from_installed_program(installed)
            .set_pins(25, 1).clock_divisor_fixed_point(125, 0).build(pio0sm0);
        sm_stop.set_pindirs([(25,PinDir::Output)]);
        let sm = sm_stop.start();

        task::spawn().ok();

        (Shared {
        }, Local {
            led, sm, rx, tx
        })
    }

    #[task(priority = 1, local = [led, tx])]
    async fn task(ctx : task::Context) {
        const PERIOD: u32 = 1_000;
        loop {
            ctx.local.tx.write(PERIOD);
            ctx.local.tx.write(PERIOD);
            for duty in (0..PERIOD).step_by(8) {
                ctx.local.tx.write(duty);
                ctx.local.tx.write(PERIOD - duty);
                info!("{}", duty);
                Mono::delay(10.millis()).await;
            }
            for duty in (0..PERIOD).rev().step_by(8) {
                ctx.local.tx.write(duty);
                ctx.local.tx.write(PERIOD - duty);
                info!("{}", duty);
                Mono::delay(10.millis()).await;
            }
        }
    }
}