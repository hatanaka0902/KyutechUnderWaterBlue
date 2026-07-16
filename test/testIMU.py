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

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5007

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR = os.path.join(SCRIPT_DIR, "testAssets")

# コンソールに最新値を出す間隔 [s]
PRINT_INTERVAL_S = 0.5


def now_stamp() -> str:
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def open_csv(addr) -> tuple:
    os.makedirs(SAVE_DIR, exist_ok=True)
    host = addr[0].replace(".", "-")
    path = os.path.join(SAVE_DIR, f"IMU_log_{now_stamp()}_{host}.csv")
    f = open(path, mode="w", newline="", encoding="utf-8")
    writer = csv.writer(f)
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
    ax, ay, az, gx, gy, gz, mx, my, mz, qx, qy, qz, qw = values
    return (
        f"accel[m/s^2]=({ax:7.3f},{ay:7.3f},{az:7.3f})  "
        f"gyro[rad/s]=({gx:7.4f},{gy:7.4f},{gz:7.4f})  "
        f"mag[uT]=({mx:6.1f},{my:6.1f},{mz:6.1f})  "
        f"quat(x,y,z,w)=({qx:6.3f},{qy:6.3f},{qz:6.3f},{qw:6.3f})"
    )


def handle_connection(conn: socket.socket, addr, stop_event: threading.Event, duration_s: float | None):
    csv_file = None
    csv_writer = None
    save_path = None
    n_samples = 0
    t0 = time.time()
    last_print = 0.0
    buffer = ""

    print(f"[Info] Connected from {addr}")
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
                            csv_file.flush()
                    else:
                        print(f"[Msg] {line}")

            except socket.timeout:
                continue
            except OSError as e:
                print(f"[Recv Error] {e}")
                break
    finally:
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
    args = parse_args()
    os.makedirs(SAVE_DIR, exist_ok=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.ip, args.port))
    server.listen(1)
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
