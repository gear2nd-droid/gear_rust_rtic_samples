/// EtherNet/IP プロトコル処理モジュール（コア 0 / コア 1 共用）
///
/// このモジュールはネットワーク I/O を持たない純粋なプロトコル処理のみを担う。
/// 実際の TCP/UDP 送受信は core1.rs が行い、その前後でここの関数を呼ぶ。
///
/// # アーキテクチャ
/// コア 1 (W5500 + EIP 通信層)        コア 0 (RTIC 高速アプリ)
///   TCP受信 → eip_process_simple()      fifo_irq: EipRequest受信
///           ↘ RegisterSession等は        → eip_process_send_rr_data()
///             コア 1 が即座に応答          → レスポンス書込み
///           ↘ SendRRData は              → EIP_SISO.tcp_tx_ready = true
///             EIP_SHARED に書き込み
///             EIP_SISO.tcp_rx_ready
///             FIFO EipRequest
///             → コア 0 待ち
///             ← EIP_SISO.tcp_tx_ready
///             TCP送信
use crate::common::*;
use core::sync::atomic::Ordering;
use cortex_m::asm::dmb;
use defmt::info;

// ─── ヘッダ組み立て ───────────────────────────────────────────────────────────

/// EIP カプセル化ヘッダ（24 バイト）を buf[0..24] に書き込む
pub fn eip_build_header(
    buf: &mut [u8],
    command: u16,
    length: u16,
    session: u32,
    status: u32,
    sender_ctx: &[u8; 8],
) {
    buf[0..2].copy_from_slice(&command.to_le_bytes());
    buf[2..4].copy_from_slice(&length.to_le_bytes());
    buf[4..8].copy_from_slice(&session.to_le_bytes());
    buf[8..12].copy_from_slice(&status.to_le_bytes());
    buf[12..20].copy_from_slice(sender_ctx);
    buf[20..24].copy_from_slice(&0u32.to_le_bytes()); // options = 0
}

/// EIP ヘッダを解析して (command, length, session, status, sender_ctx) を返す
pub fn eip_parse_header(buf: &[u8]) -> Option<(u16, u16, u32, u32, [u8; 8])> {
    if buf.len() < EIP_ENCAP_LEN {
        return None;
    }
    let command = u16::from_le_bytes([buf[0], buf[1]]);
    let length = u16::from_le_bytes([buf[2], buf[3]]);
    let session = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    let status = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
    let mut ctx = [0u8; 8];
    ctx.copy_from_slice(&buf[12..20]);
    Some((command, length, session, status, ctx))
}

// ─── コア 1 側：シンプルコマンド処理 ──────────────────────────────────────────

/// コア 1 が受信した EIP TCP メッセージを処理する。
///
/// コア 1 自身が完結できるコマンド（RegisterSession, ListServices, ListIdentity 等）を
/// その場で処理し、tx_buf にレスポンスを書いてバイト数を返す。
///
/// 戻り値：
///   - `0`         : レスポンス不要（UnRegisterSession, NOP 等）
///   - `usize::MAX`: コア 0 への SendRRData 転送が必要
///   - その他       : tx_buf に書いたレスポンスバイト数
pub fn eip_process_simple(
    rx_buf: &[u8],
    rx_len: usize,
    tx_buf: &mut [u8],
    session_handle: &mut u32,
) -> usize {
    let Some((command, _length, session, _status, ctx)) = eip_parse_header(rx_buf) else {
        return 0;
    };
    if rx_len < EIP_ENCAP_LEN {
        return 0;
    }

    info!("eip_simple: cmd=0x{:04x} rx_len={}", command, rx_len);

    match command {
        EIP_CMD_NOP => 0,

        EIP_CMD_REGISTER_SESSION => {
            // ペイロード: プロトコルバージョン(2) + オプションフラグ(2) = 4 バイト
            // 入力を簡易検証（長さ 4 未満は不正）
            if rx_len < EIP_ENCAP_LEN + 4 {
                eip_build_header(tx_buf, command, 0, 0, EIP_STATUS_INVALID_LENGTH, &ctx);
                return EIP_ENCAP_LEN;
            }
            // セッションハンドル発行（0 は無効なので 1 以上を保証）
            *session_handle = session_handle.wrapping_add(1).max(1);
            eip_build_header(tx_buf, command, 4, *session_handle, EIP_STATUS_OK, &ctx);
            tx_buf[EIP_ENCAP_LEN] = 0x01; // プロトコルバージョン (1)
            tx_buf[EIP_ENCAP_LEN + 1] = 0x00;
            tx_buf[EIP_ENCAP_LEN + 2] = 0x00; // オプションフラグ
            tx_buf[EIP_ENCAP_LEN + 3] = 0x00;
            EIP_ENCAP_LEN + 4
        }

        EIP_CMD_UNREGISTER_SESSION => {
            if session == *session_handle {
                *session_handle = 0;
            }
            0 // レスポンスなし
        }

        EIP_CMD_LIST_SERVICES => {
            // 1 アイテム (Communications) を返す
            // ペイロード: item_count(2) + type(2) + len(2) + ver(2) + cap(2) + name(16) = 26
            eip_build_header(tx_buf, command, 26, 0, EIP_STATUS_OK, &ctx);
            let p = EIP_ENCAP_LEN;
            tx_buf[p] = 0x01;
            tx_buf[p + 1] = 0x00; // item count = 1
            tx_buf[p + 2] = 0x00;
            tx_buf[p + 3] = 0x01; // type = Communications (0x0100)
            tx_buf[p + 4] = 0x14;
            tx_buf[p + 5] = 0x00; // length = 20
            tx_buf[p + 6] = 0x01;
            tx_buf[p + 7] = 0x00; // version = 1
            tx_buf[p + 8] = 0x20;
            tx_buf[p + 9] = 0x00; // capability: TCP over EIP
            tx_buf[p + 10..p + 26].copy_from_slice(b"Communications\0\0");
            EIP_ENCAP_LEN + 26
        }

        EIP_CMD_LIST_IDENTITY => {
            // CIP Identity Item (0x000C): ver(2)+sockaddr(16)+vendor(2)+devtype(2)+
            //   prodcode(2)+revision(2)+status(2)+serial(4)+namelen(1)+name(3)+state(1) = 37
            eip_build_header(tx_buf, command, 6 + 37, 0, EIP_STATUS_OK, &ctx);
            let p = EIP_ENCAP_LEN;
            tx_buf[p] = 0x01;
            tx_buf[p + 1] = 0x00; // item count = 1
            tx_buf[p + 2] = 0x0C;
            tx_buf[p + 3] = 0x00; // item type = Identity
            tx_buf[p + 4] = 0x25;
            tx_buf[p + 5] = 0x00; // item length = 37
            let q = p + 6;
            tx_buf[q] = 0x01;
            tx_buf[q + 1] = 0x00; // protocol version
            for i in 0..16 {
                tx_buf[q + 2 + i] = 0;
            } // socket addr (dummy)
            tx_buf[q + 18] = 0x59;
            tx_buf[q + 19] = 0x00; // vendor ID (generic)
            tx_buf[q + 20] = 0x0C;
            tx_buf[q + 21] = 0x00; // device type (Comm Adapter)
            tx_buf[q + 22] = 0x01;
            tx_buf[q + 23] = 0x00; // product code
            tx_buf[q + 24] = 0x01;
            tx_buf[q + 25] = 0x00; // major/minor revision
            tx_buf[q + 26] = 0x00;
            tx_buf[q + 27] = 0x00; // status
            tx_buf[q + 28] = 0x01;
            tx_buf[q + 29] = 0x00;
            tx_buf[q + 30] = 0x00;
            tx_buf[q + 31] = 0x00; // serial number
            tx_buf[q + 32] = 3; // product name length
            tx_buf[q + 33] = b'R';
            tx_buf[q + 34] = b'P';
            tx_buf[q + 35] = b'2';
            tx_buf[q + 36] = 0x03; // state = operational
            EIP_ENCAP_LEN + 6 + 37
        }

        EIP_CMD_LIST_INTERFACES => {
            // インタフェースなし（空リスト）
            eip_build_header(tx_buf, command, 2, 0, EIP_STATUS_OK, &ctx);
            tx_buf[EIP_ENCAP_LEN] = 0x00;
            tx_buf[EIP_ENCAP_LEN + 1] = 0x00; // item count = 0
            EIP_ENCAP_LEN + 2
        }

        EIP_CMD_SEND_RR_DATA => {
            // コア 0 で CIP を処理する必要がある → 呼び出し元に通知
            usize::MAX
        }

        EIP_CMD_SEND_UNIT_DATA => {
            // 接続済みデータ（Forward Open 後）→ 今回は未サポート
            eip_build_header(tx_buf, command, 0, session, EIP_STATUS_INVALID_CMD, &ctx);
            EIP_ENCAP_LEN
        }

        _ => {
            eip_build_header(tx_buf, command, 0, session, EIP_STATUS_INVALID_CMD, &ctx);
            EIP_ENCAP_LEN
        }
    }
}

// ─── コア 0 側：SendRRData / CIP 処理 ────────────────────────────────────────

/// コア 0 が SendRRData を処理してレスポンスを構築する。
///
/// rx_buf には EIP_SHARED.tcp_rx_buf の内容（EIP ヘッダ + CPF + CIP）が入っている。
/// tx_buf に EIP レスポンス全体を書き込み、バイト数を返す（0 = エラー）。
pub fn eip_process_send_rr_data(rx_buf: &[u8], rx_len: usize, tx_buf: &mut [u8]) -> usize {
    info!("send_rr_data: enter rx_len={}", rx_len);
    // 最小長: EIP ヘッダ(24) + interface_handle(4) + timeout(2) + item_count(2) +
    //          null_item(4) + data_item_hdr(4) = 40 バイト
    if rx_len < EIP_ENCAP_LEN + 16 {
        info!("send_rr_data: too short");
        return 0;
    }
    let Some((_cmd, _len, session, _status, ctx)) = eip_parse_header(rx_buf) else {
        info!("send_rr_data: parse_header failed");
        return 0;
    };

    // SendRRData ペイロード解析
    let payload = &rx_buf[EIP_ENCAP_LEN..rx_len];
    // payload[0..4] = interface handle
    // payload[4..6] = timeout
    // payload[6..8] = item count
    let item_count = u16::from_le_bytes([payload[6], payload[7]]);
    info!("send_rr_data: item_count={}", item_count);
    if item_count < 2 {
        info!("send_rr_data: item_count<2");
        return 0;
    }

    // Item 0: Null アドレスアイテム (type=0x0000, len=0)
    let it0_type = u16::from_le_bytes([payload[8], payload[9]]);
    let it0_len = u16::from_le_bytes([payload[10], payload[11]]) as usize;
    info!(
        "send_rr_data: it0_type=0x{:04x} it0_len={}",
        it0_type, it0_len
    );
    if it0_type != CPF_NULL || it0_len != 0 {
        info!("send_rr_data: item0 not NULL");
        return 0;
    }

    // Item 1: Unconnected Data アイテム (type=0x00B2)
    // payload[8..12] = item0 (NULL, 4 bytes)
    // payload[12..]  = item1 start
    let it1_off = 12;
    if it1_off + 4 > payload.len() {
        info!("send_rr_data: payload too short for item1 hdr");
        return 0;
    }
    let it1_type = u16::from_le_bytes([payload[it1_off], payload[it1_off + 1]]);
    let it1_len = u16::from_le_bytes([payload[it1_off + 2], payload[it1_off + 3]]) as usize;
    info!(
        "send_rr_data: it1_type=0x{:04x} it1_len={}",
        it1_type, it1_len
    );
    if it1_type != CPF_UNCONNECTED_DATA {
        info!("send_rr_data: item1 not 0x00B2");
        return 0;
    }

    let cip_offset = it1_off + 4;
    if cip_offset + it1_len > payload.len() || it1_len == 0 {
        info!(
            "send_rr_data: cip range error cip_off={} it1_len={} payload_len={}",
            cip_offset,
            it1_len,
            payload.len()
        );
        return 0;
    }
    let cip_req = &payload[cip_offset..cip_offset + it1_len];

    // CIP リクエスト処理
    let mut cip_resp = [0u8; 128];
    let cip_resp_len = process_cip(cip_req, &mut cip_resp);
    info!("send_rr_data: cip_resp_len={}", cip_resp_len);
    if cip_resp_len == 0 {
        return 0;
    }

    // ── CIP Forward_Open / Forward_Close に応じて接続状態を Core1 へ通知 ──
    // process_cip は純粋なプロトコル処理のみ行うため、接続状態の管理はここで行う。
    if cip_resp_len >= 3 {
        match cip_resp[0] {
            0xD4 if cip_resp_len >= 28 && cip_resp[2] == 0x00 => {
                // Forward_Open 成功応答: T→O 実際 RPI は cip_resp[24..28]
                let to_rpi =
                    u32::from_le_bytes([cip_resp[24], cip_resp[25], cip_resp[26], cip_resp[27]]);
                with_eip(|s| s.io_rpi_us = to_rpi);
                dmb();
                EIP_SISO.io_rpi_updated.store(true, Ordering::Release);
                EIP_SISO.io_connected.store(true, Ordering::Release);
                info!("Forward_Open OK: T->O RPI={}us", to_rpi);
            }
            0xDB if cip_resp_len >= 28 && cip_resp[2] == 0x00 => {
                // Large_Forward_Open 成功応答: T→O 実際 RPI は cip_resp[24..28]
                let to_rpi =
                    u32::from_le_bytes([cip_resp[24], cip_resp[25], cip_resp[26], cip_resp[27]]);
                with_eip(|s| s.io_rpi_us = to_rpi);
                dmb();
                EIP_SISO.io_rpi_updated.store(true, Ordering::Release);
                EIP_SISO.io_connected.store(true, Ordering::Release);
                info!("Large_Forward_Open OK: T->O RPI={}us", to_rpi);
            }
            0xCE if cip_resp_len >= 4 && cip_resp[2] == 0x00 => {
                // Forward_Close 成功応答
                EIP_SISO.io_connected.store(false, Ordering::Release);
                info!("Forward_Close OK: I/O connection closed");
            }
            _ => {}
        }
    }

    // SendRRData レスポンス組み立て
    // ペイロード: interface_handle(4) + timeout(2) + item_count(2) +
    //             null_item(4) + data_item_hdr(4) + cip_resp
    let resp_payload_len = 4 + 2 + 2 + 4 + 4 + cip_resp_len;
    eip_build_header(
        tx_buf,
        EIP_CMD_SEND_RR_DATA,
        resp_payload_len as u16,
        session,
        EIP_STATUS_OK,
        &ctx,
    );
    let p = EIP_ENCAP_LEN;
    // interface handle
    tx_buf[p] = 0x00;
    tx_buf[p + 1] = 0x00;
    tx_buf[p + 2] = 0x00;
    tx_buf[p + 3] = 0x00;
    // timeout
    tx_buf[p + 4] = 0x00;
    tx_buf[p + 5] = 0x00;
    // item count = 2
    tx_buf[p + 6] = 0x02;
    tx_buf[p + 7] = 0x00;
    // Null address item
    tx_buf[p + 8] = 0x00;
    tx_buf[p + 9] = 0x00; // type
    tx_buf[p + 10] = 0x00;
    tx_buf[p + 11] = 0x00; // length = 0
    // Unconnected data item
    tx_buf[p + 12] = 0xB2;
    tx_buf[p + 13] = 0x00; // type = 0x00B2
    tx_buf[p + 14] = (cip_resp_len & 0xFF) as u8;
    tx_buf[p + 15] = ((cip_resp_len >> 8) & 0xFF) as u8;
    tx_buf[p + 16..p + 16 + cip_resp_len].copy_from_slice(&cip_resp[..cip_resp_len]);

    EIP_ENCAP_LEN + resp_payload_len
}

// ─── CIP 基本サービス処理（コア 0） ──────────────────────────────────────────

/// CIP リクエストを処理してレスポンスを response バッファに書き込む。
/// 戻り値: response に書いたバイト数（0 = 失敗）
pub fn process_cip(req: &[u8], response: &mut [u8]) -> usize {
    if req.is_empty() || response.len() < 4 {
        return 0;
    }
    let service = req[0];
    let path_len = if req.len() > 1 {
        req[1] as usize * 2
    } else {
        0
    }; // ワード数→バイト数
    let data_off = 2 + path_len;

    // CIP レスポンス: service_reply = service | 0x80
    response[0] = service | 0x80;
    response[1] = 0x00; // reserved

    match service {
        // Get_Attribute_All (0x01)
        0x01 => {
            response[2] = 0x00; // general status = success
            response[3] = 0x00; // additional status size = 0
            // 最小限のアイデンティティ属性
            response[4] = 0x59;
            response[5] = 0x00; // vendor ID
            response[6] = 0x0C;
            response[7] = 0x00; // device type
            response[8] = 0x01;
            response[9] = 0x00; // product code
            response[10] = 0x01;
            response[11] = 0x00; // revision
            response[12] = 0x00;
            response[13] = 0x00; // status
            response[14] = 0x01;
            response[15] = 0x00;
            response[16] = 0x00;
            response[17] = 0x00; // serial number
            response[18] = 3;
            response[19] = b'R';
            response[20] = b'P';
            response[21] = b'2';
            22
        }

        // Get_Attribute_Single (0x0E)
        0x0E => {
            // パス解析: class / instance / attribute
            if data_off + 0 <= req.len() && path_len >= 4 {
                let _class = req[3] as u16;
                let _instance = req[5] as u16;
                // 属性値として 0 を返す（実装依存で拡張可能）
                response[2] = 0x00;
                response[3] = 0x00;
                response[4] = 0x00;
                response[5] = 0x00;
                6
            } else {
                // パス不正
                response[2] = 0x08; // general status: not supported
                response[3] = 0x00;
                4
            }
        }

        // Set_Attribute_Single (0x10)
        0x10 => {
            if data_off < req.len() {
                // 書き込み成功（実際の適用はアプリ層に委ねる）
                response[2] = 0x00;
                response[3] = 0x00;
                let write_len = req.len() - data_off;
                // 受け取ったデータをそのまま格納（デモ用）
                let copy_len = write_len.min(response.len() - 4);
                response[4..4 + copy_len].copy_from_slice(&req[data_off..data_off + copy_len]);
                4 + copy_len
            } else {
                response[2] = 0x08;
                response[3] = 0x00;
                4
            }
        }

        // Forward_Open (0x54) — I/O 接続確立
        //
        // リクエスト形式（process_cip 入力時点）:
        //   req[0]        = 0x54 (service)
        //   req[1]        = path_size (words)
        //   req[2..2+p*2] = EPATH (Connection Manager 0x06/0x01)
        //   data[0..1]    = Priority/Time_Tick, Timeout_Ticks
        //   data[2..6]    = O->T Network Connection ID
        //   data[6..10]   = T->O Network Connection ID (originator は 0 を送る)
        //   data[10..12]  = Connection Serial Number
        //   data[12..14]  = Originator Vendor ID
        //   data[14..18]  = Originator Serial Number
        //   data[18]      = Connection Timeout Multiplier
        //   data[19..22]  = Reserved
        //   data[22..26]  = O->T RPI (μs)
        //   data[26..28]  = O->T Connection Parameters (16-bit)
        //   data[28..32]  = T->O RPI (μs)
        //   data[32..34]  = T->O Connection Parameters (16-bit)
        //   data[34]      = Transport Type/Trigger
        //   data[35]      = Connection Path Size (words)
        //   data[36..]    = Connection Path
        0x54 => {
            let path_words = if req.len() > 1 { req[1] as usize } else { 0 };
            let data_start = 2 + path_words * 2;
            // データ部分が最低 36 バイト必要
            if req.len() < data_start + 36 || response.len() < 30 {
                response[0] = 0xD4;
                response[1] = 0x00;
                response[2] = 0x01;
                response[3] = 0x00; // error
                return 4;
            }
            let d = &req[data_start..];
            let ot_conn_id = u32::from_le_bytes([d[2], d[3], d[4], d[5]]);
            let conn_serial = u16::from_le_bytes([d[10], d[11]]);
            let orig_vendor = u16::from_le_bytes([d[12], d[13]]);
            let orig_serial = u32::from_le_bytes([d[14], d[15], d[16], d[17]]);
            let ot_rpi = u32::from_le_bytes([d[22], d[23], d[24], d[25]]);
            let to_rpi = u32::from_le_bytes([d[28], d[29], d[30], d[31]]);

            // T->O ネットワーク接続 ID（アダプタ側が割り当て）
            const TO_CONN_ID: u32 = EIP_TO_CONN_ID;

            // Forward Open 成功レスポンス (service reply = 0xD4)
            response[0] = 0xD4;
            response[1] = 0x00;
            response[2] = 0x00;
            response[3] = 0x00; // success
            response[4..8].copy_from_slice(&ot_conn_id.to_le_bytes()); // O->T Net Conn ID
            response[8..12].copy_from_slice(&TO_CONN_ID.to_le_bytes()); // T->O Net Conn ID
            response[12..14].copy_from_slice(&conn_serial.to_le_bytes()); // Conn Serial
            response[14..16].copy_from_slice(&orig_vendor.to_le_bytes()); // Orig Vendor ID
            response[16..20].copy_from_slice(&orig_serial.to_le_bytes()); // Orig Serial
            response[20..24].copy_from_slice(&ot_rpi.to_le_bytes()); // O->T Actual RPI
            response[24..28].copy_from_slice(&to_rpi.to_le_bytes()); // T->O Actual RPI
            response[28] = 0; // App reply size
            response[29] = 0; // Reserved
            30
        }

        // Large_Forward_Open (0x5B) — CODESYS/CC100 等の新しい EIP スキャナが送る拡張版
        //
        // 通常の Forward_Open (0x54) との違いは接続パラメータが 16-bit → 32-bit に拡張された点のみ:
        //   data[22..26]  = O->T RPI (μs)
        //   data[26..30]  = O->T Connection Parameters (32-bit) ← 4 バイト
        //   data[30..34]  = T->O RPI (μs)                      ← オフセット +2
        //   data[34..38]  = T->O Connection Parameters (32-bit) ← 4 バイト
        //   data[38]      = Transport Type/Trigger              ← オフセット +4
        //   data[39]      = Connection Path Size (words)        ← オフセット +4
        //   data[40..]    = Connection Path                     ← オフセット +4
        //
        // レスポンス形式はサービスコードのみ異なり (0xDB)、フィールド構造は 0xD4 と同一。
        0x5B => {
            let path_words = if req.len() > 1 { req[1] as usize } else { 0 };
            let data_start = 2 + path_words * 2;
            // Large FO のデータ部分は通常 FO より 4 バイト大きい（最低 40 バイト必要）
            if req.len() < data_start + 40 || response.len() < 30 {
                response[0] = 0xDB;
                response[1] = 0x00;
                response[2] = 0x01;
                response[3] = 0x00; // error
                return 4;
            }
            let d = &req[data_start..];
            let ot_conn_id = u32::from_le_bytes([d[2], d[3], d[4], d[5]]);
            let conn_serial = u16::from_le_bytes([d[10], d[11]]);
            let orig_vendor = u16::from_le_bytes([d[12], d[13]]);
            let orig_serial = u32::from_le_bytes([d[14], d[15], d[16], d[17]]);
            let ot_rpi = u32::from_le_bytes([d[22], d[23], d[24], d[25]]);
            // T->O RPI は O->T 接続パラメータが 32-bit になった分 +2 ずれた d[30..34]
            let to_rpi = u32::from_le_bytes([d[30], d[31], d[32], d[33]]);

            const TO_CONN_ID: u32 = EIP_TO_CONN_ID;

            // Large_Forward_Open 成功レスポンス (service reply = 0xDB)
            response[0] = 0xDB;
            response[1] = 0x00;
            response[2] = 0x00;
            response[3] = 0x00; // success
            response[4..8].copy_from_slice(&ot_conn_id.to_le_bytes());
            response[8..12].copy_from_slice(&TO_CONN_ID.to_le_bytes());
            response[12..14].copy_from_slice(&conn_serial.to_le_bytes());
            response[14..16].copy_from_slice(&orig_vendor.to_le_bytes());
            response[16..20].copy_from_slice(&orig_serial.to_le_bytes());
            response[20..24].copy_from_slice(&ot_rpi.to_le_bytes());
            response[24..28].copy_from_slice(&to_rpi.to_le_bytes());
            response[28] = 0; // App reply size
            response[29] = 0; // Reserved
            info!(
                "Large_Forward_Open: ot_conn_id=0x{:08x} to_rpi={}us",
                ot_conn_id, to_rpi
            );
            30
        }

        // Forward_Close (0x4E) — I/O 接続切断
        //
        //   req[0]        = 0x4E (service)
        //   req[1]        = path_size (words)
        //   req[2..2+p*2] = EPATH (Connection Manager 0x06/0x01)
        //   data[0..1]    = Priority/Time_Tick, Timeout_Ticks
        //   data[2..4]    = Connection Serial Number
        //   data[4..6]    = Originator Vendor ID
        //   data[6..10]   = Originator Serial Number
        //   data[10]      = Connection Path Size (words)
        //   data[11]      = Reserved
        //   data[12..]    = Connection Path
        0x4E => {
            let path_words = if req.len() > 1 { req[1] as usize } else { 0 };
            let data_start = 2 + path_words * 2;
            if req.len() < data_start + 12 || response.len() < 14 {
                response[0] = 0xCE;
                response[1] = 0x00;
                response[2] = 0x01;
                response[3] = 0x00;
                return 4;
            }
            let d = &req[data_start..];
            let conn_serial = u16::from_le_bytes([d[2], d[3]]);
            let orig_vendor = u16::from_le_bytes([d[4], d[5]]);
            let orig_serial = u32::from_le_bytes([d[6], d[7], d[8], d[9]]);

            // Forward Close 成功レスポンス (service reply = 0xCE)
            response[0] = 0xCE;
            response[1] = 0x00;
            response[2] = 0x00;
            response[3] = 0x00; // success
            response[4..6].copy_from_slice(&conn_serial.to_le_bytes());
            response[6..8].copy_from_slice(&orig_vendor.to_le_bytes());
            response[8..12].copy_from_slice(&orig_serial.to_le_bytes());
            response[12] = 0; // App reply size
            response[13] = 0; // Reserved
            14
        }

        // その他のサービス
        _ => {
            response[2] = 0x08; // general status: service not supported
            response[3] = 0x00;
            4
        }
    }
}

// ─── UDP 暗黙的 I/O ───────────────────────────────────────────────────────────

/// UDP O->T CPF パケットを解析し、I/O データの開始オフセットを返す。
///
/// EtherNet/IP 暗黙的メッセージの CPF 構造（EIP Vol.2 Section 2-6.3）:
/// [0-1]   item_count = 2 (LE)
/// [2-3]   Sequenced Address Item type = 0x8002 (LE)
/// [4-5]   addr item length = 8 (LE)
/// [6-9]   Network Connection ID / O->T Connection ID (LE)
/// [10-13] Encapsulation Sequence Count (LE)
/// [14-15] Connected Data Item type = 0x00B1 (LE)
/// [16-17] data item length = 2 + io_data_len (LE)
/// [18-19] CIP Sequence Count (LE)
/// [20+]   I/O データ (MODELESS フォーマット)
///
/// 戻り値: I/O データの開始オフセット（= 20）、パース失敗時は None
pub fn eip_parse_io_cpf(rx: &[u8], rx_len: usize) -> Option<usize> {
    // 最小サイズ: CPF ヘッダ 20 バイト
    if rx_len < 20 {
        return None;
    }
    let item_count = u16::from_le_bytes([rx[0], rx[1]]);
    if item_count != 2 {
        return None;
    }
    let seq_addr_type = u16::from_le_bytes([rx[2], rx[3]]);
    if seq_addr_type != CPF_SEQUENCED_ADDR {
        return None;
    }
    let data_type = u16::from_le_bytes([rx[14], rx[15]]);
    if data_type != CPF_CONNECTED_DATA {
        return None;
    }
    // I/O データはオフセット 20 以降（CIP シーケンスカウント 2 バイトの次）
    Some(20)
}

/// T->O UDP CPF パケットを構築する。
///
/// [0-1]   item_count = 2
/// [2-3]   Sequenced Address Item type = 0x8002
/// [4-5]   addr item length = 8
/// [6-9]   T->O Network Connection ID
/// [10-13] Encapsulation Sequence Count (seq32)
/// [14-15] Connected Data Item type = 0x00B1
/// [16-17] data item length = 2 + io_len
/// [18-19] CIP Sequence Count (seq32 の下位 16 ビット)
/// [20+]   I/O データ
///
/// 戻り値: tx に書いたバイト数（バッファ不足時は 0）
pub fn eip_build_io_cpf(
    tx: &mut [u8],
    to_conn_id: u32,
    seq32: u32,
    io_data: &[u8],
    io_len: usize,
) -> usize {
    let total = 20 + io_len;
    if tx.len() < total {
        return 0;
    }
    let data_item_len = (2 + io_len) as u16; // CIP seq(2) + I/O data
    // Sequenced Address Item
    tx[0] = 0x02;
    tx[1] = 0x00; // item_count = 2
    tx[2] = (CPF_SEQUENCED_ADDR & 0xFF) as u8;
    tx[3] = ((CPF_SEQUENCED_ADDR >> 8) & 0xFF) as u8;
    tx[4] = 0x08;
    tx[5] = 0x00; // addr item length = 8
    tx[6..10].copy_from_slice(&to_conn_id.to_le_bytes());
    tx[10..14].copy_from_slice(&seq32.to_le_bytes());
    // Connected Data Item
    tx[14] = (CPF_CONNECTED_DATA & 0xFF) as u8;
    tx[15] = ((CPF_CONNECTED_DATA >> 8) & 0xFF) as u8;
    tx[16..18].copy_from_slice(&data_item_len.to_le_bytes());
    // CIP Sequence Count（16 ビット）
    tx[18] = (seq32 & 0xFF) as u8;
    tx[19] = ((seq32 >> 8) & 0xFF) as u8;
    // I/O データ
    if io_len > 0 {
        tx[20..20 + io_len].copy_from_slice(&io_data[..io_len]);
    }
    total
}
