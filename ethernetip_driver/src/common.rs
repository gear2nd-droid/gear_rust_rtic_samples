use core::cell::UnsafeCell;
use core::sync::atomic::AtomicBool;

pub static SPI_LEN: usize = EIP_BUF_LEN; // 528: EIP最大メッセージ長に合わせる
pub static W5500_LEN_TX: usize = EIP_BUF_LEN + 3;
pub static W5500_LEN_RX: usize = EIP_BUF_LEN + 3 + 8;

// ─── EtherNet/IP 定数 ────────────────────────────────────────────────────────
/// EIP カプセル化ヘッダ長（バイト）
pub const EIP_ENCAP_LEN: usize = 24;
/// EIP ペイロード最大長（バイト）
pub const EIP_MAX_PAYLOAD: usize = 504;
/// EIP メッセージ最大長（ヘッダ + ペイロード）
pub const EIP_BUF_LEN: usize = EIP_ENCAP_LEN + EIP_MAX_PAYLOAD; // 528
/// EIP TCP ポート（明示的メッセージ）
pub const EIP_TCP_PORT: u16 = 44818;
/// EIP UDP ポート（暗黙的 I/O）
pub const EIP_UDP_PORT: u16 = 2222;

// EtherNet/IP コマンドコード
pub const EIP_CMD_NOP: u16 = 0x0000;
pub const EIP_CMD_LIST_SERVICES: u16 = 0x0004;
pub const EIP_CMD_LIST_IDENTITY: u16 = 0x0063;
pub const EIP_CMD_LIST_INTERFACES: u16 = 0x0064;
pub const EIP_CMD_REGISTER_SESSION: u16 = 0x0065;
pub const EIP_CMD_UNREGISTER_SESSION: u16 = 0x0066;
pub const EIP_CMD_SEND_RR_DATA: u16 = 0x006F;
pub const EIP_CMD_SEND_UNIT_DATA: u16 = 0x0070;

// EtherNet/IP ステータスコード
pub const EIP_STATUS_OK: u32 = 0x0000;
pub const EIP_STATUS_INVALID_CMD: u32 = 0x0001;
pub const EIP_STATUS_INVALID_SESSION: u32 = 0x0064;
pub const EIP_STATUS_INVALID_LENGTH: u32 = 0x0065;

// CPF アイテムタイプ
pub const CPF_NULL: u16 = 0x0000;
pub const CPF_CONNECTED_ADDR: u16 = 0x00A1;
pub const CPF_CONNECTED_DATA: u16 = 0x00B1;
pub const CPF_UNCONNECTED_DATA: u16 = 0x00B2;
/// UDP 暗黙的 I/O で使用するシーケンスアドレスアイテム（Sequenced Address Item）
pub const CPF_SEQUENCED_ADDR: u16 = 0x8002;

/// Forward_Open でアダプタが割り当てる T->O ネットワーク接続 ID
pub const EIP_TO_CONN_ID: u32 = 0x00C1_0001;

// ─── EIP コア間共有データ ─────────────────────────────────────────────────────
/// コア 1 ↔ コア 0 間で EtherNet/IP メッセージを受け渡す共有メモリ
///
/// TCP CIP 処理はコア 1 完結のため tcp_rx/tx バッファは不要。
/// UDP I/O データのみコア間で共有する。
#[repr(C)]
pub struct EipSharedData {
    /// コア 1 が受信した UDP 暗黙的 I/O データ（コア 0 が読む）
    pub udp_rx_buf: [u8; EIP_MAX_PAYLOAD],
    pub udp_rx_len: usize,
    /// コア 0 が用意した UDP 送信データ（コア 1 が送信する）
    pub udp_tx_buf: [u8; EIP_MAX_PAYLOAD],
    pub udp_tx_len: usize,
    /// Forward_Open で設定された T→O RPI（マイクロ秒）。コア 1 が書き、コア 1 が読む
    pub io_rpi_us: u32,
}

/// EIP 共有データの Sync 対応ラッパー
pub struct EipShared {
    pub(crate) inner: UnsafeCell<EipSharedData>,
}
unsafe impl Sync for EipShared {}

impl EipShared {
    const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(EipSharedData {
                udp_rx_buf: [0u8; EIP_MAX_PAYLOAD],
                udp_rx_len: 0,
                udp_tx_buf: [0u8; EIP_MAX_PAYLOAD],
                udp_tx_len: 0,
                io_rpi_us: 10_000,
            }),
        }
    }
}

/// グローバル EIP 共有データ
pub static EIP_SHARED: EipShared = EipShared::new();

/// EIP 共有データへのクロージャアクセス（unsafe を隠蔽）
pub fn with_eip<F, R>(f: F) -> R
where
    F: FnOnce(&mut EipSharedData) -> R,
{
    unsafe { f(&mut *EIP_SHARED.inner.get()) }
}

// ─── SISO フラグ（コア間同期） ────────────────────────────────────────────────
/// SISO（Single Input Single Output）フラグ
///
/// コア 1 ↔ コア 0 間のデータレディ通知に使用する AtomicBool セット。
/// データは EIP_SHARED で受け渡し、フラグで読み書き完了を通知する。
pub struct EipSisoFlags {
    /// コア 1 → コア 0：UDP I/O データが EIP_SHARED.udp_rx_buf に準備できた（コア 0 がポーリングで読む）
    pub udp_rx_ready: AtomicBool,
    /// コア 0 → コア 1：UDP 送信データが EIP_SHARED.udp_tx_buf に準備できた（コア 1 がポーリングで読む）
    pub udp_tx_ready: AtomicBool,
    /// コア 1 内部：EIP I/O 接続確立（Forward_Open 成功後 true、Forward_Close で false）
    pub io_connected: AtomicBool,
    /// コア 1 内部：T→O RPI が EIP_SHARED.io_rpi_us に書き込まれた
    pub io_rpi_updated: AtomicBool,
}

/// グローバル SISO フラグ
pub static EIP_SISO: EipSisoFlags = EipSisoFlags {
    udp_rx_ready: AtomicBool::new(false),
    udp_tx_ready: AtomicBool::new(false),
    io_connected: AtomicBool::new(false),
    io_rpi_updated: AtomicBool::new(false),
};

/// コア 0 ↔ コア 1 間の FIFO メッセージ
///
/// TCP CIP 処理はコア 1 完結のため接続イベント通知のみに使用する。
#[repr(u32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FifoMsg {
    /// コア 1 → コア 0：EIP TCP 接続確立（ポート 44818）
    EipConnected = 5,
    /// コア 1 → コア 0：EIP TCP 切断
    EipDisconnected = 6,
}
