#![no_std]
#![no_main]

#[unsafe(link_section = ".boot2")]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

mod core1;
mod common;
mod ethernet_ip;
use crate::common::*;

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
use hal::{
        sio::{Sio, SioFifo},
        gpio::{FunctionPio0, FunctionSioInput, FunctionSioOutput, Pins},
        multicore::{Multicore, Stack},
        clocks::init_clocks_and_plls,
        Timer,
        dma::DMAExt,
        };
use cortex_m::asm::dmb;

systick_monotonic!(Mono, 1_000);

static CORE1_STACK: StaticCell<Stack<8192>> = StaticCell::new();

// CELL
//static RESETS_CELL: StaticCell<hal::pac::RESETS> = StaticCell::new();
static CLOCKS_CELL: StaticCell<hal::clocks::ClocksManager> = StaticCell::new();
static TIMER_CELL: StaticCell<hal::timer::Timer> = StaticCell::new();
static TIMER_PTR: AtomicPtr<hal::timer::Timer> = AtomicPtr::new(core::ptr::null_mut());

#[rtic::app(device = rp2040_hal::pac, 
    peripherals = true, 
    dispatchers = [RTC_IRQ],
)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        siofifo: SioFifo,
    }

    #[local]
    struct Local {
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        info!("boot");
        cortex_m::asm::delay(5_000_000);
        // bkpt() はデバッガ未接続時に HardFault を起こすため削除。
        // probe-rs run でデバッグする場合は必要に応じてここに再追加すること。

        Mono::start(ctx.core.SYST, 125_000_000);

        let mut pac = ctx.device;
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
        let clocks = init_clocks_and_plls(12_000_000, 
            pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, 
            &mut pac.RESETS, &mut watchdog,).ok().unwrap();
        let sio = Sio::new(pac.SIO);
        let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0,
            sio.gpio_bank0, &mut pac.RESETS,);
        let stack = CORE1_STACK.init(Stack::new());
        let mut siofifo = sio.fifo;

        // core0/1 common
        let cell_clocks: &mut hal::clocks::ClocksManager = CLOCKS_CELL.init(clocks);
        let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &cell_clocks);
        let cell_timer: &mut hal::timer::Timer = TIMER_CELL.init(timer);

        // core1:DMA-PIO-SPI用ピン定義
        let w5500_sck = pins.gpio21.into_function::<FunctionPio0>();
        let w5500_mosi = pins.gpio23.into_function::<FunctionPio0>();
        let w5500_miso = pins.gpio22.into_function::<FunctionPio0>();
        let w5500_cs = pins.gpio20.into_function::<FunctionSioOutput>();
        let w5500_int = pins.gpio24.into_function::<FunctionSioInput>();
        let w5500_rst = pins.gpio25.into_function::<FunctionSioOutput>();

        // core0/1:dma
        let dmas = pac.DMA.split(&mut pac.RESETS);
        let dma_tx = dmas.ch0;
        let dma_rx = dmas.ch1;
        
        // core1:run
        let mut multicore = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut siofifo);
        let cores = multicore.cores();
        core1::Core1Task::core0_init(w5500_sck, w5500_mosi, w5500_miso, w5500_cs, w5500_int, w5500_rst,
            pac.PIO0, &mut pac.RESETS, dma_tx, dma_rx);
        let _core1 = cores[1].spawn(
            stack.take().unwrap(),
            core1::core1_task,
        ).unwrap();

        info!("core0 and core1 start");

        // UDP I/O ポーリングタスクを起動
        io_task::spawn().ok();

        // core0->core1
        TIMER_PTR.store(cell_timer as *mut _, Ordering::Release);
        siofifo.write_blocking(1);

        // SIO_IRQ_PROC0 はハードウェア割込みタスクのため spawn 不要

        (Shared {
            siofifo
        }, Local {
            
        })
    }

    /// SIO FIFO 受信割込みタスク（EtherNet/IP 版）
    ///
    /// core1 が FIFO に書き込むと SIO_IRQ_PROC0 が発生し即座に呼ばれる。
    /// レベルトリガのため FIFO を空にするまでループする。
    #[task(binds = SIO_IRQ_PROC0, priority = 2, shared = [siofifo])]
    fn fifo_irq(mut ctx: fifo_irq::Context) {
        ctx.shared.siofifo.lock(|fifo| {
            while let Some(raw) = fifo.read() {
                // FIFO 読み出し後に dmb() を入れ、core1 が EIP_SHARED に書いた
                // データがこのコアから正しく見えることを保証する。
                dmb();

                if raw == FifoMsg::EipConnected as u32 {
                    info!("core0: EIP TCP connected");

                } else if raw == FifoMsg::EipDisconnected as u32 {
                    info!("core0: EIP TCP disconnected");

                } else {
                    info!("core0: unknown FIFO message {}", raw);
                }
            }
        });
    }

    /// UDP 暗黙的 I/O ポーリングタスク（コア 1 完結方針）
    ///
    /// core1 が udp_rx_ready をセットするのを 1ms ごとにポーリングし、
    /// アプリケーション I/O 処理を行って udp_tx_buf へ書き込む。
    /// FIFO 割り込みを使わないため、fifo_irq の処理を邪魔しない。
    #[task(priority = 1)]
    async fn io_task(_: io_task::Context) {
        loop {
            Mono::delay(1.millis()).await;

            if !EIP_SISO.udp_rx_ready.load(core::sync::atomic::Ordering::Acquire) {
                continue;
            }
            EIP_SISO.udp_rx_ready.store(false, core::sync::atomic::Ordering::Release);

            let rx_len = with_eip(|s| s.udp_rx_len);
            if rx_len == 0 {
                continue;
            }

            let mut buf = [0u8; EIP_MAX_PAYLOAD];
            with_eip(|s| buf[..rx_len].copy_from_slice(&s.udp_rx_buf[..rx_len]));

            // ─── アプリケーション I/O 処理 ─────────────────────────────────
            // ここで buf[0..rx_len] の受信データ（O→T）を処理し、
            // 送信データ（T→O）を buf に書き込む。
            // （デモ: エコー — 受信データをそのまま返す）
            // ───────────────────────────────────────────────────────────────

            with_eip(|s| {
                s.udp_tx_buf[..rx_len].copy_from_slice(&buf[..rx_len]);
                s.udp_tx_len = rx_len;
            });
            dmb();
            EIP_SISO.udp_tx_ready.store(true, core::sync::atomic::Ordering::Release);
            if rx_len >= 4 {
                info!("core0: I/O rx={:02x}{:02x}{:02x}{:02x} len={}", buf[0], buf[1], buf[2], buf[3], rx_len);
            }
        }
    }

    fn core0_delay(timer: &Timer, time: u64) {
        let start = timer.get_counter().ticks();
        loop {
            let now = timer.get_counter().ticks();
            if start + time < now {
                break;
            }
        }
    }    
}