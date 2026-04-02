use crate::common::*;
use crate::ethernet_ip::{eip_build_io_cpf, eip_parse_io_cpf, eip_process_send_rr_data, eip_process_simple};

use panic_probe as _;
//use panic_halt as _;
//use defmt::*;
use core::ptr::addr_of;
use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::asm::dmb;
use defmt as _;
use defmt::info;
use defmt_rtt as _;
use hal::{
    dma::{CH0, CH1, Channel, SingleChannel},
    gpio::bank0::{Gpio20, Gpio21, Gpio22, Gpio23, Gpio24, Gpio25},
    gpio::{FunctionPio0, FunctionSioInput, FunctionSioOutput, Pin, PullDown},
    pac,
    pio::{PIO0SM0, PIOBuilder, PIOExt, PinDir, Running, Rx, ShiftDirection, StateMachine, Tx},
    sio::Sio,
};
use pio_proc::pio_file;
use rp2040_hal as hal;
use static_cell::StaticCell;

// CELL
static DMA_TX_CELL: StaticCell<Channel<CH0>> = StaticCell::new();
static DMA_RX_CELL: StaticCell<Channel<CH1>> = StaticCell::new();
static PIO_TX_CELL: StaticCell<Tx<PIO0SM0>> = StaticCell::new();
static PIO_RX_CELL: StaticCell<Rx<PIO0SM0>> = StaticCell::new();
static PIO_SM_CELL: StaticCell<StateMachine<PIO0SM0, Running>> = StaticCell::new();
// PTR
static DMA_TX_PTR: AtomicPtr<Channel<CH0>> = AtomicPtr::new(core::ptr::null_mut());
static DMA_RX_PTR: AtomicPtr<Channel<CH1>> = AtomicPtr::new(core::ptr::null_mut());
static PIO_TX_PTR: AtomicPtr<Tx<PIO0SM0>> = AtomicPtr::new(core::ptr::null_mut());
static PIO_RX_PTR: AtomicPtr<Rx<PIO0SM0>> = AtomicPtr::new(core::ptr::null_mut());
static PIO_SM_PTR: AtomicPtr<StateMachine<PIO0SM0, Running>> =
    AtomicPtr::new(core::ptr::null_mut());

static mut TX_VALS: [u8; W5500_LEN_TX] = [0u8; W5500_LEN_TX];
static mut RX_VALS: [u8; W5500_LEN_RX] = [0u8; W5500_LEN_RX];
static CS_PIN: u8 = 20;
static SCK_PIN: u8 = 21;
static MOSI_PIN: u8 = 23;
static MISO_PIN: u8 = 22;
static INT_PIN: u8 = 24;
static RST_PIN: u8 = 25;

/// Control Phase: Selects Common Register.
static CONTROL_COM: u8 = 0b00000;
/// Gateway IP Address Register
static REG_GAR: u16 = 0x0001;
/// Subnet Mask Register
static REG_SUBR: u16 = 0x0005;
/// Source Hardware Address Register
static REG_SHAR: u16 = 0x0009;
/// Source IP Address Register
static REG_SIPR: u16 = 0x000F;
/// W5500 PHY Configuration Register
static REG_PHYCFGR: u16 = 0x002E;
/// Control Phase: Selects Socket 0 Register
static CONTROL_TCP: u8 = 0b00001;
static TX_TCP: u8 = 0b00010;
static RX_TCP: u8 = 0b00011;
static CONTROL_UDP: u8 = 0b00101;
static TX_UDP: u8 = 0b00110;
static RX_UDP: u8 = 0b00111;
/// Socket n Mode Register
static REG_SN_MR: u16 = 0x0000;
/// Socket n Command Register
static REG_SN_CR: u16 = 0x0001;
/// Socket n Interrupt Register
static REG_SN_IR: u16 = 0x0002;
/// Socket n Status Register
static REG_SN_SR: u16 = 0x0003;
/// Socket n Source Port Register
static REG_SN_PORT: u16 = 0x0004;
/// Socket n Destination IP Address Register
static REG_SN_DIPR: u16 = 0x000C;
/// Socket n Destination Port Register
static REG_SN_DPORT: u16 = 0x0010;
/// Socket n RX Buffer Size Register
static REG_RXBUF_SIZE: u16 = 0x001E;
/// Socket n TX Buffer Size Register
static REG_TXBUF_SIZE: u16 = 0x001F;
/// Socket n TX Free Size Register
static REG_TX_FSR: u16 = 0x0020;
/// Socket n TX Read Pointer Register
//static REG_TX_RD: u16 = 0x0022;
/// Socket n TX Write Pointer Register
static REG_TX_WR: u16 = 0x0024;
/// Socket n Received Size Register
static REG_RX_RSR: u16 = 0x0026;
/// Socket n RX Read Data Pointer Register
static REG_RX_RD: u16 = 0x0028;
/// Socket n RX Write Pointer Register
//static REG_RX_WR: u16 = 0x002A;

static SN_CR_OPEN: u8 = 0x01;
static SN_CR_LISTEN: u8 = 0x02;
static SN_CR_DISCON: u8 = 0x08;
static SN_CR_CLOSE: u8 = 0x10;
static SN_CR_SEND: u8 = 0x20;
static SN_CR_RECV: u8 = 0x40;
static SN_MR_TCP: u8 = 0x01;
static SN_MR_UDP: u8 = 0x02;
static SN_SR_SOCK_CLOSED: u8 = 0x00;
static SN_SR_SOCK_INIT: u8 = 0x13;
static SN_SR_SOCK_LISTEN: u8 = 0x14;
static SN_SR_SOCK_SYNRECV: u8 = 0x16;
static SN_SR_ESTABLISHED: u8 = 0x17;
static SN_SR_UDP: u8 = 0x22;
static SN_IR_SENDOK: u8 = 0x10;
static SN_IR_TIMEOUT: u8 = 0x08;
static SN_IR_RECV: u8 = 0x04;

static GATEWAY: [u8; 4] = [192u8, 168u8, 10u8, 1u8];
static SUBNET: [u8; 4] = [255u8, 255u8, 255u8, 0u8];
static MAC_ADDRESS: [u8; 6] = [0x8c, 0x1f, 0x64, 0x2f, 0x01, 0x65];
static SOURCE_IP: [u8; 4] = [192u8, 168u8, 10u8, 31u8];
static SOURCE_PORT_TCP: u16 = EIP_TCP_PORT; // 44818: EIP 明示的メッセージ
static DESTINATION_IP_TCP: [u8; 4] = [192u8, 168u8, 10u8, 17u8];
static DESTINATION_PORT_TCP: u16 = EIP_TCP_PORT;
static SOURCE_PORT_UDP: u16 = EIP_UDP_PORT; // 2222: EIP 暗黙的 I/O
static DESTINATION_IP_UDP: [u8; 4] = [192u8, 168u8, 10u8, 17u8];
static DESTINATION_PORT_UDP: u16 = EIP_UDP_PORT;

pub struct Core1Task {
    dma_tx: &'static Channel<CH0>,
    dma_rx: &'static Channel<CH1>,
    _pio_tx: &'static mut Tx<PIO0SM0>,
    _pio_rx: &'static mut Rx<PIO0SM0>,
    _pio_sm: &'static mut StateMachine<PIO0SM0, Running>,
}

impl Core1Task {
    pub fn core0_init(
        _sck_pin: Pin<Gpio21, FunctionPio0, PullDown>,
        _mosi_pin: Pin<Gpio23, FunctionPio0, PullDown>,
        _miso_pin: Pin<Gpio22, FunctionPio0, PullDown>,
        _cs_pin: Pin<Gpio20, FunctionSioOutput, PullDown>,
        _int_pin: Pin<Gpio24, FunctionSioInput, PullDown>,
        _rst_pin: Pin<Gpio25, FunctionSioOutput, PullDown>,
        pio0: hal::pac::PIO0,
        resets: &mut hal::pac::RESETS,
        dma_tx: Channel<CH0>,
        dma_rx: Channel<CH1>,
    ) {
        // PIOの設定
        let (mut pio, pio0sm0, _, _, _) = pio0.split(resets);
        let program = pio_file!("src/pio_spi.pio");
        let installed = pio.install(&program.program).unwrap();
        let (mut sm_stop, pio_rx, pio_tx) = PIOBuilder::from_installed_program(installed)
            .out_pins(MOSI_PIN, 1)
            .in_pin_base(MISO_PIN)
            .side_set_pin_base(SCK_PIN)
            .clock_divisor_fixed_point(3, 0)
            .out_shift_direction(ShiftDirection::Left)
            .in_shift_direction(ShiftDirection::Left)
            .pull_threshold(8)
            .autopull(false)
            .push_threshold(8)
            .autopush(false)
            .build(pio0sm0);
        sm_stop.set_pindirs([
            (SCK_PIN, PinDir::Output),
            (MOSI_PIN, PinDir::Output),
            (MISO_PIN, PinDir::Input),
        ]);
        let pio_sm = sm_stop.start();

        // DMAの設定
        let tx_from: *const [u8; SPI_LEN] = addr_of!(TX_VALS) as *const [u8; SPI_LEN];
        dma_tx
            .ch()
            .ch_read_addr()
            .write(|w| unsafe { w.bits(tx_from as *const u8 as u32) });
        dma_tx
            .ch()
            .ch_write_addr()
            .write(|w| unsafe { w.bits(pio_tx.fifo_address() as u32) });
        dma_tx.ch().ch_ctrl_trig().write(|w| unsafe {
            w.incr_read()
                .set_bit()
                .data_size()
                .size_byte()
                .treq_sel()
                .bits(0)
                .en()
                .set_bit()
        });
        let rx_to: *const [u8; SPI_LEN] = addr_of!(RX_VALS) as *const [u8; SPI_LEN];
        dma_rx
            .ch()
            .ch_read_addr()
            .write(|w| unsafe { w.bits(pio_rx.fifo_address() as u32) });
        dma_rx
            .ch()
            .ch_write_addr()
            .write(|w| unsafe { w.bits(rx_to as *const u8 as u32) });
        dma_rx.ch().ch_ctrl_trig().write(|w| unsafe {
            w.incr_write()
                .set_bit()
                .data_size()
                .size_byte()
                .treq_sel()
                .bits(4)
                .en()
                .set_bit()
        });

        // DMAとPIOをコア間共有
        let dma_tx_buf = DMA_TX_CELL.init(dma_tx);
        DMA_TX_PTR.store(dma_tx_buf as *mut _, Ordering::Release);
        let dma_rx_buf: &mut Channel<CH1> = DMA_RX_CELL.init(dma_rx);
        DMA_RX_PTR.store(dma_rx_buf as *mut _, Ordering::Release);
        let pio_tx_buf = PIO_TX_CELL.init(pio_tx);
        PIO_TX_PTR.store(pio_tx_buf as *mut _, Ordering::Release);
        let pio_rx_buf = PIO_RX_CELL.init(pio_rx);
        PIO_RX_PTR.store(pio_rx_buf as *mut _, Ordering::Release);
        let pio_sm_buf = PIO_SM_CELL.init(pio_sm);
        PIO_SM_PTR.store(pio_sm_buf as *mut _, Ordering::Release);
    }

    pub fn core1_init() -> Self {
        let dma_tx = unsafe { &*DMA_TX_PTR.load(Ordering::Acquire) };
        let dma_rx = unsafe { &*DMA_RX_PTR.load(Ordering::Acquire) };
        let _pio_tx = unsafe { &mut *PIO_TX_PTR.load(Ordering::Acquire) };
        let _pio_rx = unsafe { &mut *PIO_RX_PTR.load(Ordering::Acquire) };
        let _pio_sm = unsafe { &mut *PIO_SM_PTR.load(Ordering::Acquire) };
        Self {
            dma_tx: dma_tx,
            dma_rx: dma_rx,
            _pio_tx: _pio_tx,
            _pio_rx: _pio_rx,
            _pio_sm: _pio_sm,
        }
    }

    fn w5500_transfer(&mut self, tx_buf: &[u8], rx_buf: &mut [u8], len: usize) {
        self.dma_tx
            .ch()
            .ch_ctrl_trig()
            .modify(|_, w| w.en().clear_bit());
        self.dma_rx
            .ch()
            .ch_ctrl_trig()
            .modify(|_, w| w.en().clear_bit());
        let tx_from: *const [u8; SPI_LEN] = addr_of!(TX_VALS) as *const [u8; SPI_LEN];
        self.dma_tx
            .ch()
            .ch_read_addr()
            .write(|w| unsafe { w.bits(tx_from as *const u8 as u32) });
        let rx_to: *const [u8; SPI_LEN] = addr_of!(RX_VALS) as *const [u8; SPI_LEN];
        self.dma_rx
            .ch()
            .ch_write_addr()
            .write(|w| unsafe { w.bits(rx_to as *const u8 as u32) });
        unsafe {
            self.dma_tx
                .ch()
                .ch_trans_count()
                .write(|w| w.bits(len as u32));
            self.dma_rx
                .ch()
                .ch_trans_count()
                .write(|w| w.bits(len as u32));
            TX_VALS[0..tx_buf.len()].copy_from_slice(tx_buf);
        }
        Self::cs_low();
        self.dma_tx
            .ch()
            .ch_ctrl_trig()
            .modify(|_, w| w.en().set_bit());
        self.dma_rx
            .ch()
            .ch_ctrl_trig()
            .modify(|_, w| w.en().set_bit());
        // RX DMA の完了を待つ。
        // DMA TX busy=0 は「PIO TXキューに全バイトを積んだ」だけで、PIOが全バイトを
        // SPI クロックし終わった保証ではない。RX DMA が全バイトを PIO RX FIFO から
        // 読み終わった時点で、PIO の送受信が完全に完了したことが保証される。
        // 固定 100μs 待ちは SPI が遅い場合に不足するため、RX DMA 完了待ちに置き換える。⇒バグになった。
        // txとrxのwhileループから抜け出ない事態が発生しているため、固定10usに戻す。
        // dma_tx+100usでは途中で止まる、dma_rx+100usでは途中で止まらない。
        //while self.dma_tx.ch().ch_ctrl_trig().read().busy().bit_is_set() {}
        while self.dma_rx.ch().ch_ctrl_trig().read().busy().bit_is_set() {}
        Self::core1_delay(10u64);
        Self::cs_high();
        self._pio_sm.drain_tx_fifo();
        self._pio_sm.clear_fifos();
        unsafe {
            rx_buf[0..len].copy_from_slice(&RX_VALS[0..len]);
        }
    }

    fn core1_time() -> u64 {
        let th = unsafe { (*pac::TIMER::ptr()).timerawh().read().bits() };
        let tl = unsafe { (*pac::TIMER::ptr()).timerawl().read().bits() };
        let t = ((th as u64) << 32) | (tl as u64);
        t
    }

    fn _core1_await(start: u64, time: u64) -> bool {
        let now = Self::core1_time();
        if start + time >= now { true } else { false }
    }

    fn core1_delay(time: u64) {
        let start = Self::core1_time();
        loop {
            let now = Self::core1_time();
            if start + time < now {
                break;
            }
            //cortex_m::asm::wfi();
        }
    }

    #[inline(always)]
    fn cs_low() {
        unsafe {
            (*pac::SIO::ptr())
                .gpio_out_clr()
                .write(|w| w.bits(1 << CS_PIN));
        }
    }

    #[inline(always)]
    fn cs_high() {
        unsafe {
            (*pac::SIO::ptr())
                .gpio_out_set()
                .write(|w| w.bits(1 << CS_PIN));
        }
    }

    #[inline(always)]
    fn w5500_reset() {
        unsafe {
            (*pac::SIO::ptr())
                .gpio_out_clr()
                .write(|w| w.bits(1 << RST_PIN));
        }
        Self::core1_delay(500);
        unsafe {
            (*pac::SIO::ptr())
                .gpio_out_set()
                .write(|w| w.bits(1 << RST_PIN));
        }
        Self::core1_delay(4_000);
    }

    pub fn core1_w5500_reset(&mut self) {
        // 起動時 GPIO は LOW（CS アクティブ）のため、リセット前に確実に非アクティブにする。
        // CS が LOW のまま W5500 がリセットされると SPI 状態機械が壊れ全レジスタ読み書きが失敗する。
        Self::cs_high();
        Self::core1_delay(1_000); // 16μs @ 62.5 MHz
        Self::w5500_reset();
    }

    fn w5500_write(&mut self, address: u16, block_select: u8, tx_data: &[u8], data_len: usize) {
        let high = ((address >> 8) & 0x00FF) as u8;
        let low = (address & 0x00FF) as u8;
        // write
        let mut tx_buf = [0u8; W5500_LEN_TX];
        let mut rx_buf = [0u8; W5500_LEN_RX];
        tx_buf[0] = high;
        tx_buf[1] = low;
        if data_len == 1 {
            tx_buf[2] = block_select << 3 | 1 << 2 | 1 << 0;
        } else if data_len == 2 {
            tx_buf[2] = block_select << 3 | 1 << 2 | 2 << 0;
        } else if data_len == 4 {
            tx_buf[2] = block_select << 3 | 1 << 2 | 3 << 0;
        } else {
            tx_buf[2] = block_select << 3 | 1 << 2 | 0 << 0;
        }
        for i in 0..data_len {
            tx_buf[i + 3] = tx_data[i];
        }
        self.w5500_transfer(&tx_buf, &mut rx_buf, data_len + 3);
    }

    fn _w5500_write_verify(
        &mut self,
        address: u16,
        block_select: u8,
        tx_data: &[u8],
        rx_data: &mut [u8],
        data_len: usize,
    ) -> bool {
        let high = ((address >> 8) & 0x00FF) as u8;
        let low = (address & 0x00FF) as u8;
        // write
        let mut tx_buf = [0u8; W5500_LEN_TX];
        let mut rx_buf = [0u8; W5500_LEN_RX];
        tx_buf[0] = high;
        tx_buf[1] = low;
        if data_len == 1 {
            tx_buf[2] = block_select << 3 | 0b1 << 2 | 0b01 << 0;
        } else if data_len == 2 {
            tx_buf[2] = block_select << 3 | 0b1 << 2 | 0b10 << 0;
        } else if data_len == 4 {
            tx_buf[2] = block_select << 3 | 0b1 << 2 | 0b11 << 0;
        } else {
            tx_buf[2] = block_select << 3 | 0b1 << 2 | 0b00 << 0;
        }
        for i in 0..data_len {
            tx_buf[i + 3] = tx_data[i];
        }
        self.w5500_transfer(&tx_buf, &mut rx_buf, data_len + 3);
        // read
        let mut tx_buf = [0u8; W5500_LEN_TX];
        let mut rx_buf = [0u8; W5500_LEN_RX];
        tx_buf[0] = high;
        tx_buf[1] = low;
        tx_buf[2] = block_select << 3 | 0 << 2 | 0 << 0;
        for i in 0..data_len {
            tx_buf[i + 3] = 0;
        }
        self.w5500_transfer(&tx_buf, &mut rx_buf, data_len + 3);
        for i in 0..data_len {
            rx_data[i] = rx_buf[i + 3];
        }
        // verify
        let mut ret = true;
        for i in 0..data_len {
            if rx_data[i] != tx_data[i] {
                ret = false;
            }
        }
        ret
    }

    fn w5500_read(&mut self, address: u16, block_select: u8, rx_data: &mut [u8], data_len: usize) {
        let high = ((address >> 8) & 0x00FF) as u8;
        let low = (address & 0x00FF) as u8;
        // read
        let mut tx_buf = [0u8; W5500_LEN_TX];
        let mut rx_buf = [0u8; W5500_LEN_RX];
        tx_buf[0] = high;
        tx_buf[1] = low;
        if data_len == 1 {
            tx_buf[2] = block_select << 3 | 0b0 << 2 | 0b01 << 0;
        } else if data_len == 2 {
            tx_buf[2] = block_select << 3 | 0b0 << 2 | 0b10 << 0;
        } else if data_len == 4 {
            tx_buf[2] = block_select << 3 | 0b0 << 2 | 0b11 << 0;
        } else {
            tx_buf[2] = block_select << 3 | 0b0 << 2 | 0b00 << 0;
        }
        for i in 0..data_len {
            tx_buf[i + 3] = 0;
        }
        self.w5500_transfer(&tx_buf, &mut rx_buf, data_len + 3);
        for i in 0..data_len {
            rx_data[i] = rx_buf[i + 3];
        }
    }

    /// TCP（ソケット 0, ポート SOURCE_PORT_TCP）と
    /// UDP（ソケット 1, ポート SOURCE_PORT_UDP）を同時に初期化する。
    /// PHY リセット・ネットワーク設定・リンク確立待ちを含む一回限りの初期化。
    pub fn core1_w5500_init(&mut self) {
        let mut tx_buf;
        let mut rx_buf = [0u8; SPI_LEN];

        info!("w5500: init start");

        // PHY リセット → 再起動
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = 0b00111000;
        self.w5500_write(REG_PHYCFGR, CONTROL_COM, &tx_buf, 1);
        Self::core1_delay(10_000);
        tx_buf[0] = 0b10111000;
        self.w5500_write(REG_PHYCFGR, CONTROL_COM, &tx_buf, 1);

        // ネットワーク設定
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0..4].copy_from_slice(&GATEWAY);
        self.w5500_write(REG_GAR, CONTROL_COM, &tx_buf, 4);
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0..4].copy_from_slice(&SUBNET);
        self.w5500_write(REG_SUBR, CONTROL_COM, &tx_buf, 4);
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0..6].copy_from_slice(&MAC_ADDRESS);
        self.w5500_write(REG_SHAR, CONTROL_COM, &tx_buf, 6);
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0..4].copy_from_slice(&SOURCE_IP);
        self.w5500_write(REG_SIPR, CONTROL_COM, &tx_buf, 4);
        // ── 書き込み確認（読み返し） ──────────────────────────────────────────
        self.w5500_read(REG_SIPR, CONTROL_COM, &mut rx_buf, 4);
        info!(
            "w5500: SIPR readback={}.{}.{}.{} (expect {}.{}.{}.{})",
            rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
            SOURCE_IP[0], SOURCE_IP[1], SOURCE_IP[2], SOURCE_IP[3]
        );
        self.w5500_read(REG_SUBR, CONTROL_COM, &mut rx_buf, 4);
        info!(
            "w5500: SUBR readback={}.{}.{}.{} GAR={}",
            rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
            GATEWAY[3]
        );
        info!(
            "w5500: network configured ({}.{}.{}.{})",
            SOURCE_IP[0], SOURCE_IP[1], SOURCE_IP[2], SOURCE_IP[3]
        );

        // 全ソケット（0–7）のバッファサイズを 2 KB に設定
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = 2;
        for i in 0..8u8 {
            self.w5500_write(REG_RXBUF_SIZE, CONTROL_TCP + 4 * i, &tx_buf, 1);
            self.w5500_write(REG_TXBUF_SIZE, CONTROL_TCP + 4 * i, &tx_buf, 1);
        }

        // ── ソケット 0: TCP ──────────────────────────────────────────────────
        // TCP モード設定 + CLOSE で確実に初期状態へ
        info!("w5500: TCP socket0 CLOSE...");
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_CR_CLOSE; // SN_CR
        self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_CLOSED {
                break;
            }
        }

        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = ((SOURCE_PORT_TCP >> 8) & 0xFF) as u8; // SN_PORT high
        tx_buf[1] = (SOURCE_PORT_TCP & 0xFF) as u8; // SN_PORT low
        self.w5500_write(REG_SN_PORT, CONTROL_TCP, &tx_buf, 2);

        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_MR_TCP; // SN_MR: TCP モード
        self.w5500_write(REG_SN_MR, CONTROL_TCP, &tx_buf, 1); //6);
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_CLOSED {
                break;
            }
        }
        // OPEN → SOCK_INIT 待ち（タイムアウト付き診断）
        info!("w5500: TCP socket0 OPEN...");
        // SN_MR 確認ログ（OPEN前にTCPモード設定確認）
        self.w5500_read(REG_SN_MR, CONTROL_TCP, &mut rx_buf, 1);
        info!(
            "w5500: SN_MR before OPEN = 0x{:02x} (expect 0x01)",
            rx_buf[0]
        );
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_CR_OPEN;
        self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);
        let mut open_cnt: u32 = 0;
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_INIT {
                break;
            }
            //info!("SR=0x{:02x}", rx_buf[0]);
            open_cnt += 1;
            if open_cnt == 100 || open_cnt % 500_000 == 0 {
                info!("w5500: OPEN wait SR=0x{:02x} count={}", rx_buf[0], open_cnt);
            }
            if open_cnt >= 2_000_000 {
                info!("w5500: OPEN timeout! SR=0x{:02x} giving up", rx_buf[0]);
                break;
            }
        }
        info!("w5500: TCP socket0 INIT done (SR=0x{:02x})", rx_buf[0]);

        // ── ソケット 1: UDP ──────────────────────────────────────────────────
        // UDP モード設定 + CLOSE で確実に初期状態へ
        info!("w5500: UDP socket1 CLOSE...");
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_MR_UDP; // SN_MR: UDP モード
        tx_buf[1] = SN_CR_CLOSE; // SN_CR
        self.w5500_write(REG_SN_MR, CONTROL_UDP, &tx_buf, 2); //6);
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_UDP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_CLOSED {
                break;
            }
        }

        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = ((SOURCE_PORT_UDP >> 8) & 0xFF) as u8; // SN_PORT high
        tx_buf[1] = (SOURCE_PORT_UDP & 0xFF) as u8; // SN_PORT low
        self.w5500_write(REG_SN_PORT, CONTROL_UDP, &tx_buf, 2);

        // 送信先 IP + ポートを設定（REG_SN_DIPR から 6 バイト連続書き込み）
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0..4].copy_from_slice(&DESTINATION_IP_UDP);
        tx_buf[4] = ((DESTINATION_PORT_UDP >> 8) & 0xFF) as u8;
        tx_buf[5] = (DESTINATION_PORT_UDP & 0xFF) as u8;
        self.w5500_write(REG_SN_DIPR, CONTROL_UDP, &tx_buf, 6);
        // OPEN → SN_SR_UDP 待ち（タイムアウト付き診断）
        info!("w5500: UDP socket1 OPEN...");
        self.w5500_read(REG_SN_MR, CONTROL_UDP, &mut rx_buf, 1);
        info!(
            "w5500: UDP SN_MR before OPEN = 0x{:02x} (expect 0x02)",
            rx_buf[0]
        );
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_CR_OPEN;
        self.w5500_write(REG_SN_CR, CONTROL_UDP, &tx_buf, 1); //5);
        let mut udp_open_cnt: u32 = 0;
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_UDP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_UDP {
                break;
            }
            udp_open_cnt += 1;
            if udp_open_cnt == 100 || udp_open_cnt % 500_000 == 0 {
                info!(
                    "w5500: UDP OPEN wait SR=0x{:02x} count={}",
                    rx_buf[0], udp_open_cnt
                );
            }
            if udp_open_cnt >= 2_000_000 {
                info!("w5500: UDP OPEN timeout! SR=0x{:02x} giving up", rx_buf[0]);
                break;
            }
        }
        info!("w5500: UDP socket1 OPEN done (SR=0x{:02x})", rx_buf[0]);

        // リンク確立待ち（LNK ビット = 1 になるまで）
        info!("w5500: waiting for PHY link...");
        let mut link_wait_count: u32 = 0;
        loop {
            self.w5500_read(REG_PHYCFGR, CONTROL_COM, &mut rx_buf, 1);
            if rx_buf[0] & 0x01 != 0 {
                break;
            }
            link_wait_count += 1;
            if link_wait_count % 10 == 0 {
                info!(
                    "w5500: PHY link down, waiting... (PHYCFGR=0x{:02x}, {}s)",
                    rx_buf[0],
                    link_wait_count / 10
                );
            }
            Self::core1_delay(100_000); // 100 ms
        }
        info!("w5500: PHY link UP (PHYCFGR=0x{:02x})", rx_buf[0]);
    }

    pub fn core1_w5500_udp_tx(&mut self, tx_data: &[u8], tx_len: usize) -> bool {
        let mut tx_buf;
        let mut rx_buf = [0u8; SPI_LEN];
        let mut ret_val;

        // 送信可能サイズの確認
        self.w5500_read(REG_TX_FSR, CONTROL_UDP, &mut rx_buf, 2);
        let free_size: u16 = ((rx_buf[0] as u16) << 8) | ((rx_buf[1] as u16) << 0);
        if free_size as usize >= tx_len {
            // 書き込み先アドレスを取得
            self.w5500_read(REG_TX_WR, CONTROL_UDP, &mut rx_buf, 2);
            let address: u16 = ((rx_buf[0] as u16) << 8) | ((rx_buf[1] as u16) << 0);
            let next_address = address.wrapping_add(tx_len as u16);

            // 送信データの書き込み
            tx_buf = [0u8; SPI_LEN];
            tx_buf[0..tx_len].copy_from_slice(&tx_data[0..tx_len]);
            self.w5500_write(address, TX_UDP, &tx_buf, tx_len);

            // 書き込み先アドレスの更新
            tx_buf = [0u8; SPI_LEN];
            tx_buf[0] = ((next_address >> 8) & 0x00FF) as u8;
            tx_buf[1] = ((next_address >> 0) & 0x00FF) as u8;
            self.w5500_write(REG_TX_WR, CONTROL_UDP, &tx_buf, 2);

            // 送信
            tx_buf = [0u8; SPI_LEN];
            tx_buf[0] = SN_CR_SEND; // REG_SN_CR
            self.w5500_write(REG_SN_CR, CONTROL_UDP, &tx_buf, 1); //5);

            // SENDOK 待ち（タイムアウト付き 200ms）
            let deadline = Self::core1_time() + 200_000;
            ret_val = loop {
                self.w5500_read(REG_SN_IR, CONTROL_UDP, &mut rx_buf, 1);
                if rx_buf[0] & SN_IR_SENDOK != 0 {
                    // SENDOK クリア
                    tx_buf = [0u8; SPI_LEN];
                    tx_buf[0] = SN_IR_SENDOK;
                    self.w5500_write(REG_SN_IR, CONTROL_UDP, &tx_buf, 1);
                    break true;
                }
                if Self::core1_time() > deadline {
                    info!("udp_tx: SENDOK timeout");
                    break false;
                }
            };
        } else {
            info!("send_error_free_size:{}", free_size);
            ret_val = false;
        }

        ret_val
    }

    /// UDP 受信（ペイロードのみ返す）
    ///
    /// W5500 UDP RX バッファの各パケットは先頭 8 バイトのヘッダ
    /// （送信元 IP 4B + ポート 2B + データ長 2B）を持つ。
    ///
    /// RX_RSR はバッファ内の合計バイト数（複数パケット分）を返す。
    /// 1 回の読み出しで最大 SPI_LEN バイトを読み込み、
    /// 先頭ヘッダの data_len[6..7] フィールドで正確な 1 パケット分のサイズを確定し、
    /// ポインタをそのサイズ分だけ進める（= 1 パケットのみ消費）。
    /// W5500 は RECV コマンド後にバッファに残パケットがあれば SN_IR_RECV を再セットするため、
    /// 次のループ反復で残りのパケットが読み出される。
    ///
    /// 戻り値: ペイロードのバイト数（0 = データなし）
    pub fn core1_w5500_udp_rx(&mut self, rx_data: &mut [u8]) -> usize {
        let mut tx_buf;
        let mut rx_buf = [0u8; SPI_LEN];

        // RECV 割込みビット確認（SENDOK 等の他ビットは無視）
        self.w5500_read(REG_SN_IR, CONTROL_UDP, &mut rx_buf, 1);
        if rx_buf[0] & SN_IR_RECV == 0 {
            return 0;
        }

        // 受信可能バイト数確認（ヘッダ 8B 未満はスキップ）
        self.w5500_read(REG_RX_RSR, CONTROL_UDP, &mut rx_buf, 2);
        let rx_available: usize = (((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16)) as usize;
        if rx_available < 8 {
            return 0;
        }
        // 読み出しアドレス取得
        self.w5500_read(REG_RX_RD, CONTROL_UDP, &mut rx_buf, 2);
        let address: u16 = ((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16);

        // バッファから SPI_LEN バイト以内を 1 回で読み込む
        // rx_buf[0..8]  = W5500 UDP ヘッダ（先頭パケット分）
        // rx_buf[8..]   = 先頭パケットのペイロード（および後続パケット）
        let read_len = rx_available.min(SPI_LEN);
        self.w5500_read(address, RX_UDP, &mut rx_buf, read_len);

        // ヘッダの data_len[6..7] から先頭パケットの正確なペイロード長を取得
        // W5500 は完全なパケット単位でバッファに書くため必ず 8B 以上存在する
        let payload_len: usize = (((rx_buf[6] as u16) << 8) | (rx_buf[7] as u16)) as usize;

        // ポインタを先頭パケット 1 つ分だけ進める（後続パケットは次回以降で処理）
        let next_address = address.wrapping_add((8 + payload_len) as u16);

        if payload_len == 0 || payload_len > rx_data.len() || 8 + payload_len > read_len {
            // 不正なペイロード長、またはバッファに収まっていない → スキップ
            tx_buf = [0u8; SPI_LEN];
            tx_buf[0] = ((next_address >> 8) & 0xFF) as u8;
            tx_buf[1] = (next_address & 0xFF) as u8;
            self.w5500_write(REG_RX_RD, CONTROL_UDP, &tx_buf, 2);
            tx_buf = [0u8; SPI_LEN];
            tx_buf[0] = SN_CR_RECV;
            self.w5500_write(REG_SN_CR, CONTROL_UDP, &tx_buf, 1); //5);
            return 0;
        }

        // ヘッダ直後（オフセット 8）から payload_len バイトをコピー
        rx_data[0..payload_len].copy_from_slice(&rx_buf[8..8 + payload_len]);

        // 読み出しポインタを 1 パケット分のみ更新
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = ((next_address >> 8) & 0xFF) as u8;
        tx_buf[1] = (next_address & 0xFF) as u8;
        self.w5500_write(REG_RX_RD, CONTROL_UDP, &tx_buf, 2);

        // RECV コマンド発行（残パケットがあれば W5500 が SN_IR_RECV を再セット）
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_CR_RECV;
        self.w5500_write(REG_SN_CR, CONTROL_UDP, &tx_buf, 1); //5);

        // RECV 割込みクリア
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_IR_RECV;
        self.w5500_write(REG_SN_IR, CONTROL_UDP, &tx_buf, 1);

        payload_len
    }

    // -------------------------------------------------------------------------
    // TCP ドライバ
    // -------------------------------------------------------------------------

    /// TCP LISTEN 開始。LISTEN コマンドを発行し SOCK_LISTEN 状態まで待つ。
    /// 接続確立（ESTABLISHED）は待たない → core1_task() でポーリングする。
    /// core1_w5500_init() または core1_w5500_tcp_reopen() の後に呼ぶこと。
    pub fn core1_w5500_tcp_start_listen(&mut self) {
        let mut tx_buf = [0u8; SPI_LEN];
        let mut rx_buf = [0u8; SPI_LEN];
        let deadline = Self::core1_time() + 500_000;
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_LISTEN {
                break;
            }
            if Self::core1_time() > deadline {
                info!("tcp_start_listen: timeout (SR=0x{:02x})", rx_buf[0]);
                break;
            }
            if rx_buf[0] == SN_SR_SOCK_INIT {
                // SOCK_INIT のとき LISTEN コマンドを再発行（初回含む）
                tx_buf[0] = SN_CR_LISTEN;
                self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);
            }
            Self::core1_delay(5_000); // 5ms 待ち後に再ポーリング
        }
    }

    /// TCP 切断後の再 OPEN。CLOSE → SOCK_CLOSED → OPEN → SOCK_INIT の順に遷移する。
    /// この後 core1_w5500_tcp_start_listen() を呼ぶこと。
    pub fn core1_w5500_tcp_reopen(&mut self) {
        let mut tx_buf = [0u8; SPI_LEN];
        let mut rx_buf = [0u8; SPI_LEN];

        // CLOSE（タイムアウト付き: 最大 500ms）
        tx_buf[0] = SN_CR_CLOSE;
        self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);
        let deadline = Self::core1_time() + 500_000;
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_CLOSED {
                break;
            }
            if Self::core1_time() > deadline {
                info!(
                    "tcp_reopen: CLOSE timeout (SR=0x{:02x}), forcing",
                    rx_buf[0]
                );
                break;
            }
            Self::core1_delay(1_000);
        }

        // TCP モード設定 + ソースポート再設定 + OPEN（タイムアウト付き: 最大 500ms）
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_MR_TCP; // SN_MR: TCP モード
        tx_buf[1] = SN_CR_OPEN; // SN_CR
        self.w5500_write(REG_SN_MR, CONTROL_TCP, &tx_buf, 2); //6);
        let deadline = Self::core1_time() + 500_000;
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_INIT {
                break;
            }
            if Self::core1_time() > deadline {
                info!("tcp_reopen: OPEN timeout (SR=0x{:02x})", rx_buf[0]);
                break;
            }
            Self::core1_delay(1_000);
        }
    }

    /// 現在のソケットステータスを返す
    pub fn core1_w5500_tcp_socket_sr(&mut self) -> u8 {
        let mut rx_buf = [0u8; SPI_LEN];
        self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
        rx_buf[0]
    }

    /// TCP ソケットの SN_IR（割込みフラグ）を返す（診断用）
    /// ビット: b0=CON, b1=DISCON, b2=RECV, b3=TIMEOUT, b4=SENDOK
    pub fn core1_w5500_tcp_sn_ir(&mut self) -> u8 {
        let mut rx_buf = [0u8; SPI_LEN];
        self.w5500_read(REG_SN_IR, CONTROL_TCP, &mut rx_buf, 1);
        rx_buf[0]
    }

    /// PHYCFGR を返す（診断用）
    /// ビット: b7=RST, b6=OPMD, b5:3=OPMDC, b2=DPX, b1=SPD, b0=LNK
    pub fn core1_w5500_phycfgr(&mut self) -> u8 {
        let mut rx_buf = [0u8; SPI_LEN];
        self.w5500_read(REG_PHYCFGR, CONTROL_COM, &mut rx_buf, 1);
        rx_buf[0]
    }

    /// DISCON コマンドを発行してソケットをクローズする
    pub fn core1_w5500_tcp_discon(&mut self) {
        let mut tx_buf = [0u8; SPI_LEN];
        let mut rx_buf = [0u8; SPI_LEN];

        // DISCON コマンド
        tx_buf[0] = SN_CR_DISCON;
        self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);

        // CLOSED になるまで待つ（最大 500ms）
        let deadline = Self::core1_time() + 500_000;
        loop {
            self.w5500_read(REG_SN_SR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] == SN_SR_SOCK_CLOSED {
                break;
            }
            if Self::core1_time() > deadline {
                // タイムアウト時は強制 CLOSE
                tx_buf = [0u8; SPI_LEN];
                tx_buf[0] = SN_CR_CLOSE;
                self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);
                break;
            }
        }
    }

    /// TCP送信（最大 SPI_LEN バイト）
    /// 戻り値: true = 送信成功, false = 失敗
    pub fn core1_w5500_tcp_tx(&mut self, tx_data: &[u8], tx_len: usize) -> bool {
        let mut tx_buf;
        let mut rx_buf = [0u8; SPI_LEN];

        // 送信可能サイズの確認
        self.w5500_read(REG_TX_FSR, CONTROL_TCP, &mut rx_buf, 2);
        let free_size: u16 = ((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16);
        if (free_size as usize) < tx_len {
            info!("tcp_tx: no free size (free={})", free_size);
            return false;
        }

        // 書き込み先アドレスを取得
        self.w5500_read(REG_TX_WR, CONTROL_TCP, &mut rx_buf, 2);
        let address: u16 = ((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16);
        let next_address = address.wrapping_add(tx_len as u16);

        // 送信データの書き込み
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0..tx_len].copy_from_slice(&tx_data[0..tx_len]);
        self.w5500_write(address, TX_TCP, &tx_buf, tx_len);

        // 書き込みポインタの更新
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = ((next_address >> 8) & 0x00FF) as u8;
        tx_buf[1] = (next_address & 0x00FF) as u8;
        self.w5500_write(REG_TX_WR, CONTROL_TCP, &tx_buf, 2);

        // SEND コマンド発行
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_CR_SEND;
        self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);

        // SENDOK 割込み待ち（タイムアウト付き）
        let deadline = Self::core1_time() + 1_000_000; // 1秒
        loop {
            self.w5500_read(REG_SN_IR, CONTROL_TCP, &mut rx_buf, 1);
            if rx_buf[0] & SN_IR_SENDOK != 0 {
                // SENDOK 割込みクリア
                tx_buf = [0u8; SPI_LEN];
                tx_buf[0] = SN_IR_SENDOK;
                self.w5500_write(REG_SN_IR, CONTROL_TCP, &tx_buf, 1);
                return true;
            }
            if rx_buf[0] & SN_IR_TIMEOUT != 0 {
                // タイムアウト割込みクリア
                tx_buf = [0u8; SPI_LEN];
                tx_buf[0] = SN_IR_TIMEOUT;
                self.w5500_write(REG_SN_IR, CONTROL_TCP, &tx_buf, 1);
                info!("tcp_tx: timeout");
                return false;
            }
            if Self::core1_time() > deadline {
                info!("tcp_tx: deadline exceeded");
                return false;
            }
        }
    }

    /// TCP受信（最大 SPI_LEN バイト）
    /// 戻り値: 受信バイト数（0 = データなし）
    pub fn core1_w5500_tcp_rx(&mut self, rx_data: &mut [u8]) -> usize {
        let mut tx_buf;
        let mut rx_buf = [0u8; SPI_LEN];

        // RECV 割込み確認
        self.w5500_read(REG_SN_IR, CONTROL_TCP, &mut rx_buf, 1);
        if rx_buf[0] & SN_IR_RECV == 0 {
            return 0;
        }

        // 受信サイズ確認
        self.w5500_read(REG_RX_RSR, CONTROL_TCP, &mut rx_buf, 2);
        let rx_len: usize = (((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16)) as usize;
        if rx_len == 0 || rx_len > SPI_LEN {
            return 0;
        }

        // 読み込みアドレスを取得
        self.w5500_read(REG_RX_RD, CONTROL_TCP, &mut rx_buf, 2);
        let address: u16 = ((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16);
        let next_address = address.wrapping_add(rx_len as u16);

        // 受信データの読み込み（TCP はヘッダなし）
        self.w5500_read(address, RX_TCP, &mut rx_buf, rx_len);
        rx_data[0..rx_len].copy_from_slice(&rx_buf[0..rx_len]);

        // 読み込みポインタの更新
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = ((next_address >> 8) & 0x00FF) as u8;
        tx_buf[1] = (next_address & 0x00FF) as u8;
        self.w5500_write(REG_RX_RD, CONTROL_TCP, &tx_buf, 2);

        // RECV コマンド発行
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_CR_RECV;
        self.w5500_write(REG_SN_CR, CONTROL_TCP, &tx_buf, 1); //5);

        // RECV 割込みクリア
        tx_buf = [0u8; SPI_LEN];
        tx_buf[0] = SN_IR_RECV;
        self.w5500_write(REG_SN_IR, CONTROL_TCP, &tx_buf, 1);

        rx_len
    }
}

/// コア 1 メインタスク。EtherNet/IP TCP（ソケット 0, ポート 44818）と
/// UDP（ソケット 1, ポート 2222）を同時に処理する。
///
/// # ループの構造
/// init (TCP socket0 + UDP socket1)
/// tcp_start_listen()          // SOCK_LISTEN まで待つ
/// loop {
///     FIFO チェック（ノンブロッキング）
///     TCP 状態確認            // 接続検出 / 切断検出 → reopen + re-listen
///     TCP 受信ポーリング      // ESTABLISHED 時のみ → eip_process_simple() / eip_process_send_rr_data()
///     UDP 受信ポーリング      // 常時 → EIP_SHARED.udp_rx_buf に格納、udp_rx_ready をセット
/// }
pub fn core1_task() {
    info!("core1: task start");
    let mut core1 = Core1Task::core1_init();
    info!("core1: core1_init() done");
    core1.core1_w5500_reset();
    info!("core1: w5500 reset done");
    // TCP（ソケット 0）と UDP（ソケット 1）を同時に初期化
    core1.core1_w5500_init();

    let pac = unsafe { pac::Peripherals::steal() };
    let mut siofifo = Sio::new(pac.SIO).fifo;

    // core0 からの起動確認シグナルを受け取る
    loop {
        if siofifo.read().is_some() {
            break;
        }
    }

    info!(
        "core1 started: EIP TCP port={}, UDP port={}",
        SOURCE_PORT_TCP, SOURCE_PORT_UDP
    );

    // 最初の LISTEN（SOCK_LISTEN まで待つ、接続確立は待たない）
    core1.core1_w5500_tcp_start_listen();
    {
        let sr = core1.core1_w5500_tcp_socket_sr();
        if sr == SN_SR_SOCK_LISTEN {
            info!(
                "EIP TCP: listening on port {} (SR=0x{:02x})",
                SOURCE_PORT_TCP, sr
            );
        } else {
            info!(
                "EIP TCP: start_listen failed (SR=0x{:02x}), recovery in main loop",
                sr
            );
        }
    }

    let mut tcp_connected = false;
    let mut session_handle: u32 = 0;
    // T->O UDP シーケンスカウンタ（CPF Sequenced Address Item / CIP Sequence Count）
    let mut udp_seq: u32 = 0;
    // ローカル TX バッファ（共有メモリへの送受信中書き込みを防ぐダブルバッファ）
    // Core0 が udp_tx_buf を更新したら即座にここへコピーし、送信はこちらから行う。
    // Forward_Open 直後から T->O データを送れるよう、初期値として 4 バイトのゼロを設定する
    // （CODESYS の T->O サイズ設定 = 4 バイトに合わせる）。
    let mut local_tx_buf = [0u8; EIP_MAX_PAYLOAD];
    let mut local_tx_len: usize = 4; // T->O 初期データ: 4 バイトのゼロ
    // T→O 定周期送信タイマー（デフォルト 10ms、Forward_Open 後に RPI へ更新）
    let mut rpi_us: u64 = 10_000u64;
    let mut next_io_time = Core1Task::core1_time() + rpi_us;
    // 定期ステータスログ（1 秒ごとに TCP 状態を出力）
    let mut next_status_time = Core1Task::core1_time() + 1_000_000u64;
    // 前回ログした TCP SR（変化時のみ追加ログ）
    let mut last_tcp_sr: u8 = 0xFF;

    loop {
        // ── FIFO メッセージ処理（ノンブロッキング） ──────────────────────────
        // core1 は基本的に FIFO を受信しない（core0 からの EipSendIo は
        // SISO フラグで通知されるため、FIFO を空にするだけでよい）
        while let Some(_) = siofifo.read() {
            dmb();
        }

        // ── RPI 更新チェック（Forward_Open で Core0 が設定） ────────────────
        if EIP_SISO.io_rpi_updated.load(Ordering::Acquire) {
            EIP_SISO.io_rpi_updated.store(false, Ordering::Release);
            let new_rpi = with_eip(|s| s.io_rpi_us) as u64;
            if new_rpi > 0 {
                rpi_us = new_rpi;
                next_io_time = Core1Task::core1_time() + rpi_us;
                info!("core1: RPI updated to {}us", rpi_us);
            }
        }

        // ── Core0 が TX データを更新したらローカルバッファへコピー ──────────
        // 送信処理中に共有メモリが書き換えられないようダブルバッファ化する。
        // 実際の送信はループ末尾のタイマーブロックで定周期に行う。
        if EIP_SISO.udp_tx_ready.load(Ordering::Acquire) {
            EIP_SISO.udp_tx_ready.store(false, Ordering::Release);
            let new_len = with_eip(|s| s.udp_tx_len);
            if new_len > 0 && new_len <= EIP_MAX_PAYLOAD {
                with_eip(|s| {
                    local_tx_buf[..new_len].copy_from_slice(&s.udp_tx_buf[..new_len]);
                });
                dmb();
                local_tx_len = new_len;
            }
        }

        // ── TCP ソケット状態確認 ─────────────────────────────────────────────
        let tcp_sr = core1.core1_w5500_tcp_socket_sr();

        // 新規接続検出（LISTEN → ESTABLISHED）
        if tcp_sr == SN_SR_ESTABLISHED && !tcp_connected {
            tcp_connected = true;
            session_handle = 0;
            info!("EIP TCP: connected");
            siofifo.write(FifoMsg::EipConnected as u32);
        }

        // 切断検出（接続中かつ ESTABLISHED 以外 → 全ての切断状態を捕捉）
        // CLOSE_WAIT(0x1C), CLOSED(0x00), FIN_WAIT(0x18), CLOSING(0x1A), TIME_WAIT(0x1B) 等
        if tcp_connected && tcp_sr != SN_SR_ESTABLISHED {
            tcp_connected = false;
            session_handle = 0;
            info!("EIP TCP: disconnected (SR=0x{:02x})", tcp_sr);
            core1.core1_w5500_tcp_discon();
            siofifo.write(FifoMsg::EipDisconnected as u32);
            // 再 OPEN → LISTEN（UDP ソケットはそのまま継続）
            core1.core1_w5500_tcp_reopen();
            core1.core1_w5500_tcp_start_listen();
            info!("EIP TCP: re-listening on port {}", SOURCE_PORT_TCP);
        }

        // LISTEN 状態でも ESTABLISHED でもない場合のリカバリ
        // （初回 tcp_start_listen() が失敗した場合や異常終了からの自動回復）
        // SN_SR_SOCK_SYNRECV(0x16) は TCP 3-way ハンドシェイク途中の正常遷移なので除外する
        if !tcp_connected
            && tcp_sr != SN_SR_SOCK_LISTEN
            && tcp_sr != SN_SR_ESTABLISHED
            && tcp_sr != SN_SR_SOCK_SYNRECV
        {
            if tcp_sr == SN_SR_SOCK_INIT {
                // SOCK_INIT のまま → LISTEN コマンドのみ再試行
                core1.core1_w5500_tcp_start_listen();
            } else {
                // CLOSED 等 → reopen してから LISTEN
                info!("EIP TCP: unexpected SR=0x{:02x}, reopen+listen", tcp_sr);
                core1.core1_w5500_tcp_reopen();
                core1.core1_w5500_tcp_start_listen();
            }
        }

        // ── TCP 受信（接続中のみ） ───────────────────────────────────────────
        // TCP はストリームプロトコルのため、1 回の読み出しに複数の EIP メッセージが
        // 含まれる場合がある（例: RegisterSession + ForwardOpen が同一セグメント到着）。
        // EIP ヘッダの length フィールド（バイト 2-3）でメッセージ境界を判定してループする。
        if tcp_sr == SN_SR_ESTABLISHED {
            let mut rx_buf = [0u8; SPI_LEN];
            let rx_len = core1.core1_w5500_tcp_rx(&mut rx_buf);
            if rx_len > 0 {
                let mut offset = 0usize;
                while offset + EIP_ENCAP_LEN <= rx_len {
                    // EIP ヘッダからこのメッセージのペイロード長を取得
                    let payload_len = u16::from_le_bytes([
                        rx_buf[offset + 2],
                        rx_buf[offset + 3],
                    ]) as usize;
                    let msg_end = offset + EIP_ENCAP_LEN + payload_len;
                    if msg_end > rx_len {
                        // 不完全なメッセージ（TCP 分割）→ このループを抜ける
                        info!("EIP TCP: partial message at offset={} msg_end={} rx_len={}", offset, msg_end, rx_len);
                        break;
                    }
                    let msg_len = msg_end - offset;

                    let mut tx_buf = [0u8; SPI_LEN];
                    let tx_len = eip_process_simple(
                        &rx_buf[offset..],
                        msg_len,
                        &mut tx_buf,
                        &mut session_handle,
                    );

                    if tx_len == usize::MAX {
                        // SendRRData → コア 1 で CIP を直接処理（コア 1 完結方針）
                        let mut resp_buf = [0u8; SPI_LEN];
                        let resp_len = eip_process_send_rr_data(
                            &rx_buf[offset..],
                            msg_len,
                            &mut resp_buf,
                        );
                        if resp_len > 0 {
                            core1.core1_w5500_tcp_tx(&resp_buf, resp_len);
                        }
                    } else if tx_len > 0 {
                        // RegisterSession / NOP など直接応答できるコマンド
                        core1.core1_w5500_tcp_tx(&tx_buf, tx_len);
                    }

                    offset = msg_end;
                }
            }
        }

        // ── UDP 受信（常時ポーリング、TCP 状態に依存しない） ─────────────────
        {
            let mut udp_buf = [0u8; SPI_LEN];
            let udp_len = core1.core1_w5500_udp_rx(&mut udp_buf);
            //let udp_len = 0;
            if udp_len > 0 {
                // CPF パケットを解析して I/O データのオフセットを取得する
                // CPF 構造: Sequenced Address Item(0x8002) + Connected Data Item(0x00B1)
                if let Some(io_start) = eip_parse_io_cpf(&udp_buf, udp_len) {
                    let io_len = udp_len.saturating_sub(io_start);
                    // I/O データのみを EIP_SHARED に格納し core0 へ通知
                    with_eip(|s| {
                        let copy_len = io_len.min(s.udp_rx_buf.len());
                        s.udp_rx_buf[..copy_len]
                            .copy_from_slice(&udp_buf[io_start..io_start + copy_len]);
                        s.udp_rx_len = copy_len;
                    });
                    dmb();
                    EIP_SISO.udp_rx_ready.store(true, Ordering::Release);
                    // コア 0 へは FIFO 通知しない。コア 0 は udp_rx_ready をポーリングで検知する。
                } else {
                    info!("UDP: CPF parse failed, len={}", udp_len);
                }
            }
        }

        // ── 定期ステータスログ（1 秒ごと） ───────────────────────────────────
        // RTT バッファ溢れを防ぐため T→O タイマーとは独立した低頻度ログ。
        // TCP SR が変化したときも即座に出力する（接続試行の検出）。
        let now = Core1Task::core1_time();
        if tcp_sr != last_tcp_sr {
            last_tcp_sr = tcp_sr;
            let sn_ir = core1.core1_w5500_tcp_sn_ir();
            info!("TCP SR changed: 0x{:02x} connected={} sn_ir=0x{:02x}", tcp_sr, tcp_connected, sn_ir);
        }
        if now >= next_status_time {
            next_status_time = now + 1_000_000u64;
            let io_conn = EIP_SISO.io_connected.load(Ordering::Acquire);
            let sn_ir = core1.core1_w5500_tcp_sn_ir();
            let phycfgr = core1.core1_w5500_phycfgr();
            info!(
                "status: tcp_sr=0x{:02x} tcp_conn={} io_conn={} sn_ir=0x{:02x} phy=0x{:02x}(lnk={}) rpi={}us",
                tcp_sr, tcp_connected, io_conn, sn_ir, phycfgr, phycfgr & 0x01, rpi_us
            );
        }

        // ── T→O 定周期送信（タイマーベース、Core0 の応答待ちなし） ──────────
        // local_tx_buf には Core0 が最後に更新した送信データが格納済み。
        // Core0 の処理タイミングに依存せず RPI 周期で送信するため、
        // Core0 が低速・一時停止しても T→O が止まらない。
        // io_connected は Forward_Open 成功後に true となり、
        // Forward_Close または TCP 切断後に false となる。
        if now >= next_io_time {
            next_io_time += rpi_us;
            if EIP_SISO.io_connected.load(Ordering::Acquire) && local_tx_len > 0 {
                udp_seq = udp_seq.wrapping_add(1);
                let mut cpf_buf = [0u8; SPI_LEN];
                let cpf_len = eip_build_io_cpf(
                    &mut cpf_buf,
                    EIP_TO_CONN_ID,
                    udp_seq,
                    &local_tx_buf,
                    local_tx_len,
                );
                if cpf_len > 0 {
                    core1.core1_w5500_udp_tx(&cpf_buf, cpf_len);
                }
            }
        }
    }
}
