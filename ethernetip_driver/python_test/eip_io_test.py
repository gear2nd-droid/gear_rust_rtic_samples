# EtherNet/IP Implicit（I/O スキャナ）テスト for W5500 driver on RP2040 (Pico)
# 
# eeip ライブラリ（rossmann-engineering/eeip.py）を使用して、
# Pico を EtherNet/IP アダプタとして I/O スキャンを行うテストプログラム。
# 
# インストール:
#   pip install eeip
# 
# 接続先:
#   Pico IP       : 192.168.10.31
#   TCP port      : 44818  (EIP 明示的メッセージ: RegisterSession / Forward Open)
#   UDP port      : 2222   (EIP 暗黙的 I/O: 周期データ交換)
# 
# アセンブリオブジェクト（CIP Class 0x04）:
#   O->T (PC → Pico, 出力) : Instance 0x64  4 バイト
#   T->O (Pico → PC, 入力) : Instance 0x65  4 バイト
# 
# プロトコルシーケンス:
#   1. TCP 接続
#   2. RegisterSession (EIP TCP)
#   3. Forward Open   (CIP SendRRData → I/O 接続確立)
#   4. UDP 周期送受信  (O->T: PC→Pico, T->O: Pico→PC)
#   5. Forward Close  (I/O 接続切断)
#   6. UnregisterSession
# 
# UDP パケット形式（MODELESS):
#   [シーケンス番号(2 bytes, LE)] + [データ(可変)]
# 
# 使い方:
#   python eip_io_test.py
#   python eip_io_test.py --ip 192.168.10.31 --count 20 --interval 0.5

import argparse
import platform
import socket
import subprocess
import sys
import time
import threading
import traceback
from datetime import datetime

# eeip ライブラリのインポート
try:
    from eeip import EEIPClient, RealTimeFormat, ConnectionType
except ImportError:
    print("[ERROR] eeip ライブラリが見つかりません。")
    print("        pip install eeip  でインストールしてください。")
    sys.exit(1)


def check_tcp_port(ip: str, port: int, timeout: float = 2.0) -> bool:
    # TCP ポートに接続できるか確認する（診断用）
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except (ConnectionRefusedError, socket.timeout, OSError):
        return False


def ping_host(ip: str, timeout: int = 1) -> bool:
    # OS の ping コマンドで ICMP 疎通を確認する
    if platform.system().lower() == "windows":
        cmd = ["ping", "-n", "1", "-w", str(timeout * 1000), ip]
    else:
        cmd = ["ping", "-c", "1", "-W", str(timeout), ip]
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=timeout + 2)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def diagnose(ip: str) -> None:
    # ファームウェアの動作状態を診断する
    print("[診断] ネットワーク疎通を確認中...")

    # ── Step 1: ping（ICMP）で Pico が IP 応答するか確認 ─────────────────────
    ping_ok = ping_host(ip)
    if not ping_ok:
        print(f"[診断] ping {ip}: 応答なし")
        print("[診断] → Pico が見つかりません。次を確認してください:")
        print(f"[診断]   ① Pico の電源が入っているか")
        print(f"[診断]   ② Ethernet ケーブルが接続されているか")
        print(f"[診断]   ③ Pico の IP が {ip} であるか（core1.rs の SOURCE_IP）")
        print("[診断]   ④ 新しいファームウェアがフラッシュされているか")
        print("[診断]      cargo build --release")
        print("[診断]      probe-rs run --chip RP2040 --protocol swd \\")
        print("[診断]        target/thumbv6m-none-eabi/release/w5500_driver")
        return

    print(f"[診断] ping {ip}: OK（Pico は起動・ネットワーク接続中）")

    # ── Step 2: TCP ポートチェック ────────────────────────────────────────────
    print(f"[診断] port {EIP_TCP_PORT} (EIP TCP) を確認中...")
    ok = check_tcp_port(ip, EIP_TCP_PORT)

    if ok:
        print(f"[診断] port {EIP_TCP_PORT}: 応答あり → ファームウェアは正常動作中です。")
        print("[診断]   RegisterSession が失敗した理由を確認してください。")
    else:
        print(f"[診断] port {EIP_TCP_PORT}: 応答なし（TCP LISTEN していません）")
        print("[診断] → 考えられる原因と対処:")
        print("[診断]   ① 最新ファームウェアをビルド＆フラッシュしてください:")
        print("[診断]      cargo build --release")
        print("[診断]      probe-rs run --chip RP2040 --protocol swd \\")
        print("[診断]        target/thumbv6m-none-eabi/release/w5500_driver")
        print("[診断]   ② フラッシュ後も同じなら RTT ログを確認してください:")
        print("[診断]      probe-rs run で起動して 'EIP TCP: listening' が出るか確認")

# ── デフォルト設定 ────────────────────────────────────────────────────────────
DEFAULT_PICO_IP    = "192.168.10.31"
DEFAULT_COUNT      = 0          # 0 = 無制限（Ctrl+C で停止）
#DEFAULT_INTERVAL   = 0.5        # 秒
DEFAULT_INTERVAL   = 0.01        # 秒

# EIP / CIP 定数
EIP_TCP_PORT       = 44818
EIP_UDP_PORT       = 2222

# アセンブリインスタンス
O_T_INSTANCE       = 0x64       # 出力: PC → Pico
T_O_INSTANCE       = 0x65       # 入力: Pico → PC
O_T_LENGTH         = 4          # O->T データ長（バイト）
T_O_LENGTH         = 4          # T->O データ長（バイト）
RPI_US             = 100_000    # 要求パケットレート: 100 ms（μs 単位）
EIP_TO_CONN_ID     = 0x00C10001 # ファームウェアが割り当てる T->O ネットワーク接続 ID


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="EtherNet/IP Implicit I/O スキャナ テスト")
    parser.add_argument("--ip",       default=DEFAULT_PICO_IP, help="Pico の IP アドレス")
    parser.add_argument("--count",    type=int, default=DEFAULT_COUNT,
                        help="送受信サイクル数（0 = 無制限）")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL,
                        help="送信間隔（秒）")
    return parser.parse_args()


def print_banner(ip: str, interval: float, count: int) -> None:
    print("=" * 60)
    print("  EtherNet/IP Implicit I/O スキャナ テスト")
    print(f"  Pico IP       : {ip}")
    print(f"  TCP port      : {EIP_TCP_PORT}  (RegisterSession / Forward Open)")
    print(f"  UDP port      : {EIP_UDP_PORT}   (I/O 周期データ)")
    print(f"  O->T instance : 0x{O_T_INSTANCE:02X}  {O_T_LENGTH} bytes  (PC → Pico)")
    print(f"  T->O instance : 0x{T_O_INSTANCE:02X}  {T_O_LENGTH} bytes  (Pico → PC)")
    print(f"  RPI           : {RPI_US // 1000} ms")
    print(f"  送信間隔      : {interval} s")
    print(f"  サイクル数    : {'無制限 (Ctrl+C で停止)' if count == 0 else count}")
    print("=" * 60)


def run_io_scan(ip: str, count: int, interval: float) -> None:
    # EtherNet/IP Implicit I/O スキャンを実行する。
    # 1. RegisterSession で TCP セッションを確立
    # 2. Forward Open で I/O 接続を開く
    # 3. O->T データを周期送信し、T->O データを受信して表示
    # 4. Forward Close → UnregisterSession で接続を閉じる

    client = EEIPClient()

    # ── 1. TCP セッション登録 ────────────────────────────────────────────────
    print(f"[EIP] TCP 接続 → RegisterSession ({ip}:{EIP_TCP_PORT}) ...")
    try:
        client.register_session(ip)
    except (ConnectionRefusedError, OSError) as e:
        print(f"[ERROR] RegisterSession 失敗: {e}")
        diagnose(ip)
        return
    print("[EIP] Session 登録完了")

    # ── 2. O->T（出力）パラメータ設定 ────────────────────────────────────────
    client.o_t_instance_id         = O_T_INSTANCE
    client.o_t_length               = O_T_LENGTH
    client.o_t_requested_packet_rate = RPI_US
    client.o_t_realtime_format      = RealTimeFormat.MODELESS
    client.o_t_owner_redundant      = False
    client.o_t_variable_length      = False
    client.o_t_connection_type      = ConnectionType.POINT_TO_POINT

    # ── 3. T->O（入力）パラメータ設定 ────────────────────────────────────────
    client.t_o_instance_id          = T_O_INSTANCE
    client.t_o_length               = T_O_LENGTH
    client.t_o_requested_packet_rate = RPI_US
    client.t_o_realtime_format      = RealTimeFormat.MODELESS
    client.t_o_owner_redundant      = False
    client.t_o_variable_length      = False
    client.t_o_connection_type      = ConnectionType.POINT_TO_POINT

    # ── 4. Forward Open（I/O 接続確立） ──────────────────────────────────────
    print("[EIP] Forward Open（I/O 接続確立）...")
    try:
        client.forward_open()
    except Exception as e:
        print(f"[ERROR] Forward Open 失敗: {e}")
        traceback.print_exc()
        try:
            client.unregister_session()
        except Exception:
            pass
        return
    # ファームウェアが割り当てる T->O 接続 ID を明示的に設定
    client._EEIPClient__connection_id_t_o = EIP_TO_CONN_ID
    print("[EIP] I/O 接続確立完了 ✓")
    print()
    print("  cycle |  O->T (PC→Pico)      |  T->O (Pico→PC)")
    print("  ------+----------------------+--------------------")

    # ── 5. 周期データ送受信ループ ─────────────────────────────────────────────
    stop_event = threading.Event()
    cycle = 0

    def io_loop() -> None:
        nonlocal cycle
        try:
            while not stop_event.is_set():
                cycle_num = cycle + 1

                # O->T 出力データ作成（4 バイト: サイクルカウンタをリトルエンディアンで格納）
                out = [
                    (cycle_num      ) & 0xFF,
                    (cycle_num >>  8) & 0xFF,
                    (cycle_num >> 16) & 0xFF,
                    (cycle_num >> 24) & 0xFF,
                ]

                # 出力データ送信（eeip は forward_open 後にバックグラウンドで UDP 送信する）
                client.o_t_iodata = out

                # 入力データ受信（T->O データ: Pico から受信した O_T_LENGTH バイト分）
                in_data = client.t_o_iodata[:O_T_LENGTH]  # list[int]

                # 表示
                out_hex = " ".join(f"{b:02X}" for b in out)
                now = datetime.now()
                str_date = now.strftime("%H:%M:%S")
                if any(in_data):
                    in_hex  = " ".join(f"{b:02X}" for b in in_data)
                    print(f" {str_date}: {cycle_num:5d} |  {out_hex:<20s}|  {in_hex}")
                else:
                    print(f" {str_date}: {cycle_num:5d} |  {out_hex:<20s}|  (no data)")

                cycle = cycle_num
                if count > 0 and cycle >= count:
                    stop_event.set()
                    break

                stop_event.wait(interval)

        except Exception as e:
            print(f"\n[ERROR] I/O ループ中にエラー: {e}")
            stop_event.set()

    io_thread = threading.Thread(target=io_loop, daemon=True)
    io_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C — 停止中...")
        stop_event.set()

    io_thread.join(timeout=3.0)
    print()

    # ── 6. Forward Close + UnregisterSession ─────────────────────────────────
    print(f"[EIP] {cycle} サイクル完了。接続を閉じます...")
    try:
        client.forward_close()
        print("[EIP] Forward Close 完了")
    except Exception as e:
        print(f"[WARN] Forward Close エラー: {e}")

    try:
        client.unregister_session()
        print("[EIP] Session 解除完了")
    except Exception as e:
        print(f"[WARN] UnregisterSession エラー: {e}")

    print("[EIP] テスト完了")


def main() -> None:
    args = parse_args()
    print_banner(args.ip, args.interval, args.count)
    run_io_scan(args.ip, args.count, args.interval)


if __name__ == "__main__":
    main()