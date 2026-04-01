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
use rp2040_hal::Timer;

systick_monotonic!(Mono, 1_000);

#[rtic::app(device = rp2040_hal::pac, 
    peripherals = true, 
    dispatchers = [RTC_IRQ]
)]
mod app {
    use super::*;
    use cortex_m::asm::bkpt;
    use rp2040_hal::{self as hal, timer::{Alarm, Alarm0}};
    use hal::{
        clocks::init_clocks_and_plls,
    };
    use hal::fugit::ExtU32;
    
    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        alarm0 : Alarm0,
        alarm0_val : u32,
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

        let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
        let mut alarm0 = timer.alarm_0().unwrap();
        alarm0.enable_interrupt();
        alarm0.schedule(100u32.millis()).unwrap();
        let alarm0_val = 0;

        task::spawn().ok();

        (Shared {
        }, Local {
            alarm0, alarm0_val
        })
    }

    #[task(priority = 1)]
    async fn task(_ : task::Context) {
        let mut val = 0;
        loop {
            val += 1;
            info!("task:{}", val);
            Mono::delay(100.millis()).await;
        }
    }    

    #[task(priority = 2, binds = TIMER_IRQ_0, local = [alarm0, alarm0_val])]
    fn alarm0_irq(ctx : alarm0_irq::Context) {
        ctx.local.alarm0.clear_interrupt();
        ctx.local.alarm0.schedule(100u32.millis()).unwrap();
        *ctx.local.alarm0_val += 1;
        info!("tim_irq:{}", *ctx.local.alarm0_val);
    }
}