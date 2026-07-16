"""
実機IMU検証スクリプト (ESP32 + BNO08x)

Sample/IMU/IMU_logger.py および Control/sensor.py (ImuStream) と同じプロトコル:
  - 本スクリプトが TCP サーバとして待ち受け (デフォルト 0.0.0.0:5007)
  - ESP32 (IMU_CALIB_AND_REC.ino) が接続してくる
  - 接続後に MEASURE を送り、CSV行を受信する
  - データ行フォーマット (13フィールド):
      Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Qx,Qy,Qz,Qw
      Ax..Az [m/s^2], Gx..Gz [rad/s], Mx..Mz [uT], Qx..Qw 姿勢クォータニオン

取得値は test/testAssets/ に CSV 保存する。
Ctrl+C で停止。

使い方:
  python test/testIMU.py
  python test/testIMU.py --port 5007 --duration 30
"""

from __future__ import annotations

import argparse
import csv
import datetime
import os
import socket
import sys
import threading
import time

# --- 接続設定 ---
# ESP32側(IMU_CALIB_AND_REC.ino)の host/port と一致させること
LISTEN_IP = "0.0.0.0"   # 全IFで待ち受け (特定NICに縛らない)
LISTEN_PORT = 5007

# --- 保存先 ---
# スクリプト位置基準にすることで、実行カレントに依存せず test/testAssets へ書く
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR = os.path.join(SCRIPT_DIR, "testAssets")

# コンソール表示の間引き間隔 [s]
# IMUは~50Hzだが、全行printすると読めないため表示だけ間引く(CSVは全件保存)
PRINT_INTERVAL_S = 0.5


def now_stamp() -> str:
    """
    目的: ファイル名用の現在時刻文字列を返す。
    意義: 接続・再実行ごとに一意なCSV名を付け、過去ログを上書きしない。
    """
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def open_csv(addr) -> tuple:
    """
    目的: 新規CSVを開き、ヘッダを書いて (file, writer, path) を返す。
    意義: 受信ループ開始前に保存先を確定し、取得値の事後検証・QEKF入力確認を可能にする。
    Args:
        addr: accept() が返す (ip, port)。ファイル名に接続元IPを残す。
    """
    os.makedirs(SAVE_DIR, exist_ok=True)
    host = addr[0].replace(".", "-")
    path = os.path.join(SAVE_DIR, f"IMU_log_{now_stamp()}_{host}.csv")
    f = open(path, mode="w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    # ヘッダは Sample/IMU/IMU_logger.py と揃える (後で突き合わせしやすい)
    writer.writerow(
        [
            "PC_Timestamp",
            "Ax", "Ay", "Az",
            "Gx", "Gy", "Gz",
            "Mx", "My", "Mz",
            "Qx", "Qy", "Qz", "Qw",
        ]
    )
    f.flush()
    print(f"[Save] {path}")
    return f, writer, path


def is_data_line(line: str) -> bool:
    """
    目的: 受信1行が「13数値のIMUデータ行」かどうかを判定する。
    意義: HELLO / >> MEASURE / ACCURACY 等の制御メッセージをCSVに混入させない。
          ImuStream._parse_and_store と同じ選別方針。
    """
    if not line or "," not in line:
        return False
    if line.startswith(("HELLO", "CALIB", "STATUS", ">>", "ACCURACY", "MODE")):
        return False
    parts = line.split(",")
    if len(parts) != 13:
        return False
    try:
        [float(p) for p in parts]
    except ValueError:
        return False
    return True


def format_sample(values: list[float]) -> str:
    """
    目的: 13フィールドの生値を、単位付きの読みやすい1行文字列にする。
    意義: 実機確認時に「何が取れているか」をコンソールで即判断できるようにする。
    """
    ax, ay, az, gx, gy, gz, mx, my, mz, qx, qy, qz, qw = values
    return (
        f"accel[m/s^2]=({ax:7.3f},{ay:7.3f},{az:7.3f})  "
        f"gyro[rad/s]=({gx:7.4f},{gy:7.4f},{gz:7.4f})  "
        f"mag[uT]=({mx:6.1f},{my:6.1f},{mz:6.1f})  "
        f"quat(x,y,z,w)=({qx:6.3f},{qy:6.3f},{qz:6.3f},{qw:6.3f})"
    )


def handle_connection(
    conn: socket.socket,
    addr,
    stop_event: threading.Event,
    duration_s: float | None,
):
    """
    目的: 1本のESP32接続について、計測開始→受信→CSV保存→終了処理を行う。
    意義: 本スクリプトの中核。プロトコル(MEASURE/STOP)とデータ保存をここで完結させる。

    流れ:
      1) MEASURE 送信でESP側を計測モードへ
      2) 改行区切りで行を組み立て、データ行のみCSVへ追記
      3) 一定間隔でコンソール表示 (全件はCSV側)
      4) 切断・時間切れ・停止時に STOP 送信とファイルクローズ
    """
    csv_file = None
    csv_writer = None
    save_path = None
    n_samples = 0
    t0 = time.time()
    last_print = 0.0
    buffer = ""  # TCPはメッセージ境界が保証されないため、改行まで蓄積する

    print(f"[Info] Connected from {addr}")
    # timeout付きrecvで、duration/Ctrl+C をブロック無しに監視できるようにする
    conn.settimeout(1.0)

    try:
        conn.sendall(b"MEASURE\n")
        print("[Cmd] MEASURE")
        csv_file, csv_writer, save_path = open_csv(addr)

        while not stop_event.is_set():
            if duration_s is not None and (time.time() - t0) >= duration_s:
                print(f"[Info] duration {duration_s:.1f}s reached")
                break

            try:
                data = conn.recv(4096)
                if not data:
                    print("[Info] peer closed connection")
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    # キャリブ精度ワンショット (データではないが検証に有用)
                    if line.startswith("ACCURACY_ONCE:"):
                        print(f"[Accuracy] {line}")
                        continue

                    if is_data_line(line):
                        values = [float(p) for p in line.split(",")]
                        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        csv_writer.writerow([ts] + [f"{v:.6f}" for v in values])
                        n_samples += 1

                        now = time.time()
                        if now - last_print >= PRINT_INTERVAL_S:
                            last_print = now
                            print(f"[{n_samples:6d}] {format_sample(values)}")
                            csv_file.flush()  # 途中停止でも直近まで残す
                    else:
                        print(f"[Msg] {line}")

            except socket.timeout:
                continue  # タイムアウトは正常: ループ先頭で停止条件を再評価
            except OSError as e:
                print(f"[Recv Error] {e}")
                break
    finally:
        # 異常終了でもESPをIDLEへ戻し、CSVを閉じる
        try:
            conn.sendall(b"STOP\n")
        except OSError:
            pass
        try:
            conn.close()
        except OSError:
            pass
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        elapsed = time.time() - t0
        rate = n_samples / elapsed if elapsed > 0 else 0.0
        print(
            f"[Done] samples={n_samples}, elapsed={elapsed:.1f}s, "
            f"rate≈{rate:.1f} Hz, saved={save_path}"
        )


def parse_args():
    """
    目的: CLI引数 (--ip / --port / --duration) を解釈する。
    意義: コードを編集せずに待ち受け先や計測時間を変えられるようにする。
    """
    p = argparse.ArgumentParser(description="ESP32 BNO08x IMU hardware test -> test/testAssets")
    p.add_argument("--ip", default=LISTEN_IP, help="listen address (default 0.0.0.0)")
    p.add_argument("--port", type=int, default=LISTEN_PORT, help="listen port (default 5007)")
    p.add_argument(
        "--duration",
        type=float,
        default=None,
        help="recording seconds (default: until Ctrl+C)",
    )
    return p.parse_args()


def main():
    """
    目的: TCPサーバを起動し、ESP32接続を待ち受けて計測セッションを開始する。
    意義: エントリポイント。接続待ちと1セッション実行の外側ループを担う。
          --duration 指定時は1接続で終了、未指定時は再接続も受け付ける。
    """
    args = parse_args()
    os.makedirs(SAVE_DIR, exist_ok=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.ip, args.port))
    server.listen(1)
    # acceptもtimeout付き: Ctrl+C を確実に拾うため
    server.settimeout(1.0)

    stop_event = threading.Event()

    print("=" * 60)
    print(f"IMU hardware test  listen={args.ip}:{args.port}")
    print(f"save dir           {SAVE_DIR}")
    print("ESP32 側: host=このPCのIP, port=5007 で接続すること")
    print("Ctrl+C で終了")
    print("=" * 60)

    try:
        while not stop_event.is_set():
            print("[Info] Waiting for ESP32 connection...")
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            handle_connection(conn, addr, stop_event, args.duration)
            if args.duration is not None:
                break
    except KeyboardInterrupt:
        print("\n[Exit] KeyboardInterrupt")
        stop_event.set()
    finally:
        server.close()


if __name__ == "__main__":
    main()
    sys.exit(0)
