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
use rp2040_hal as hal;
use rtic_monotonics::systick::prelude::*;
use static_cell::StaticCell;
use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::asm::bkpt;
use hal::{
    pac,
    Timer,
    sio::{Sio, SioFifo},
    clocks::init_clocks_and_plls,
    multicore::{Multicore, Stack},
};

systick_monotonic!(Mono, 1_000);

static CORE1_STACK: StaticCell<Stack<4096>> = StaticCell::new();
static TIMER_CELL: StaticCell<Timer> = StaticCell::new();
static TIMER_PTR: AtomicPtr<Timer> = AtomicPtr::new(core::ptr::null_mut());

#[repr(C)]
struct SharedPacket {
    len: u32,
    data: [u8; 2048],
}

static mut SHARED_PKT: SharedPacket = SharedPacket {
    len: 0,
    data: [0; 2048],
};

#[rtic::app(device = rp2040_hal::pac, 
    peripherals = true, 
    dispatchers = [RTC_IRQ]
)]
mod app {
    use super::*;
    
    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        siofifo: SioFifo,
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

        let timer: &'static mut Timer = TIMER_CELL.init(Timer::new(pac.TIMER, &mut pac.RESETS, &clocks));
        TIMER_PTR.store(timer as *mut _, Ordering::Release);

        let stack = CORE1_STACK.init(Stack::new());
        let mut siofifo = sio.fifo;

        let mut multicore = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut siofifo);
        let cores = multicore.cores();
        let _core1 = cores[1].spawn(
            stack.take().unwrap(),
            core1_task,
        ).unwrap();

        core0_task::spawn().ok();

        (Shared {
        }, Local {
            siofifo,
        })
    }

    #[task(priority = 1, local = [siofifo])]
    async fn core0_task(ctx : core0_task::Context) {
        let mut cnt = 0;
        let siofifo = ctx.local.siofifo;
        loop {
            Mono::delay(100.millis()).await;
            cnt += 1;
            siofifo.write(cnt);
            unsafe{
                SHARED_PKT.len += 1;
                SHARED_PKT.data[0] = (cnt & 0xFF) as u8;
            }
            let buf = siofifo.read();
            match buf {
                None => {
                    info!("core0:{}, read:None", cnt);
                },
                Some(v) => {
                    info!("core0:{}, read:{}, shared_send:{}", cnt, v, unsafe{SHARED_PKT.data[0]});
                }
            }
        }
    }

    fn core1_task() {
        let pac = unsafe{
            pac::Peripherals::steal()
        };
        let timer: &'static Timer = unsafe {
            &*TIMER_PTR.load(Ordering::Acquire)
        };
        let mut siofifo = Sio::new(pac.SIO).fifo;
        let mut cnt = 1000;
        loop {
            core1_delay(&timer, 100_000);
            cnt += 1;
            siofifo.write(cnt);
            let buf = siofifo.read();
            match buf {
                None => {
                    info!("core1:{}, read:None", cnt);
                },
                Some(v) => {
                    info!("core1:{}, read:{}, shared_receive:{}", cnt, v, unsafe{SHARED_PKT.data[0]});
                }
            }
        }
    }

    fn core1_delay(timer: &Timer, time: u64) {
        let start = timer.get_counter().ticks();
        loop {
            let now = timer.get_counter().ticks();
            if start + time < now {
                break;
            }
        }
    }
}