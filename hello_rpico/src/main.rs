#![no_std]      // 標準ライブラリを使用しない
#![no_main]     // Rustの通常のmain関数を使用しない

// RP2040のセカンダリブートローダ、このセクションはROMブートローダから実行される
#[unsafe(link_section = ".boot2")]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// panic時の動作
use panic_probe as _;
//use panic_halt as _;

// defmt(軽量ロギング)
use defmt as _;
use defmt::info;
use defmt_rtt as _;

// RTIC monitonic(Systickを時間基準に使用)
use rtic_monotonics::systick::prelude::*;

// Cortex-M用ユーティリティ
use cortex_m::asm::bkpt;

// HAL
use embedded_hal::digital::OutputPin;
use rp2040_hal as hal;
use hal::{
    gpio::{Pin, Pins, PullDown, FunctionSioOutput,},
    gpio::bank0::{Gpio25},
    sio::Sio,
    clocks::init_clocks_and_plls,
};

// Systickを1kHzのmonotonicタイマとして定義
systick_monotonic!(Mono, 1_000);

// RTICのアプリの実行
#[rtic::app(device = rp2040_hal::pac, 
    peripherals = true, 
    dispatchers = [RTC_IRQ, XIP_IRQ]
)]
mod app {
    use super::*;
    
    // 共有リソース
    // 複数タスクからアクセスされる、RICが排他制御を行う
    #[shared]
    struct Shared {
        shared_val : i32,
    }

    // ローカルリソース
    // 単一のタスクでアクセスされる、RTICでの排他制御が不要
    #[local]
    struct Local {
        local_val : i32,
        led : Pin<Gpio25, FunctionSioOutput, PullDown>,
    }

    // アイドルタスク
    // 実行可能なタスクがないときに呼ばれる
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // 割込み待ち(省電力)
            cortex_m::asm::wfi();
        }
    }

    // 初期化処理
    // システム起動時に1度だけ呼ばれる
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // デバッグ実行時にここで一時停止
        bkpt();

        // PAC(Peripheral Access Crate)の取得
        let mut pac = ctx.device;

        // ウォッチドック初期化(クロック初期化に必要)
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // クロックとPLLの初期化
        // 外部水晶12MHzからシステムクロック125MHzを作る
        let _ = init_clocks_and_plls(12_000_000, 
            pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, 
            &mut pac.RESETS, &mut watchdog,).ok().unwrap();

        // Systick monotonicの開始
        // RP2040のシステムクロック125MHzを使用
        Mono::start(ctx.core.SYST, 125_000_000);

        // SIO(GPIO制御用)
        let sio = Sio::new(pac.SIO);
        
        // GPIO初期化
        let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0,
            sio.gpio_bank0, &mut pac.RESETS,);

        // 初期値
        let shared_val = 0;
        let local_val = 0;

        // GPIO25をpush-pull出力(LED)
        let led = pins.gpio25.into_push_pull_output();

        // 非同期タスクを起動
        task1::spawn().ok();
        task2::spawn().ok();

        // リソースの登録
        (Shared {
            shared_val
        }, Local {
            local_val, led
        })
    }

    // タスク1、低優先度タスク
    // ローカルリソースで値をカウントアップ
    // カウントアップした値を共有リソースに入れる
    #[task(priority = 1, shared = [shared_val], local = [local_val])]
    async fn task1(mut ctx : task1::Context) {
        let local_val = ctx.local.local_val;
        loop {
            *local_val = *local_val + 1;
            ctx.shared.shared_val.lock(|shared_val|{
                *shared_val += 1;
            });
            Mono::delay(1_000.millis()).await;
        }
    }

    // タスク2、高優先度タスク
    // 共有リソースの値でもって、LEDを制御
    #[task(priority = 2, shared = [shared_val], local = [led])]
    async fn task2(mut ctx : task2::Context) {
        let led = ctx.local.led;
        loop {
            ctx.shared.shared_val.lock(|shared_val|{
                info!("{}", *shared_val);
                if *shared_val % 2 == 1 {
                    led.set_high().ok();
                } else {
                    led.set_low().ok();
                }
            });
            Mono::delay(100.millis()).await;
        }
    }
}