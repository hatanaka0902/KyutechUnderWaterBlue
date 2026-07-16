import socket
import threading
import csv
import datetime
import time
import os

# --- 設定 ---
SERVER_IP = "0.0.0.0"
SERVER_PORT = 5007

SAVE_DIR = "imu_server_save_csv"
os.makedirs(SAVE_DIR, exist_ok=True)

# --- 状態 ---
state_lock = threading.Lock()
is_logging = False          # REC状態（STOPまで保持）
csv_file = None
csv_writer = None
current_filename = None

# 接続ソケット共有（送信用）
conn_lock = threading.Lock()
conn_global = None

# 受信スレッド停止（接続ごと）
recv_stop_event = threading.Event()


def now_stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def set_conn(conn):
    global conn_global
    with conn_lock:
        conn_global = conn


def get_conn():
    with conn_lock:
        return conn_global


def safe_send(cmd: str) -> bool:
    """接続中なら送信。切れてたら False（状態は維持）"""
    conn = get_conn()
    if conn is None:
        return False
    try:
        conn.sendall((cmd.strip().upper() + "\n").encode("utf-8"))
        return True
    except Exception:
        return False


def close_log_file_nolock():
    """lock取得済み前提でクローズ"""
    global csv_file, csv_writer, current_filename
    if csv_file:
        try:
            csv_file.flush()
            csv_file.close()
        except Exception:
            pass
    csv_file = None
    csv_writer = None
    current_filename = None


def open_new_log_file_for_connection(addr):
    """
    ★再接続のたびに新しいファイルにする
    REC中に新接続が来たら必ず呼ぶ
    """
    global csv_file, csv_writer, current_filename

    # 古いファイルが開いていたら閉じる（＝接続ごとに新規）
    close_log_file_nolock()

    filename = f"IMU_log_{now_stamp()}_{addr[0].replace('.', '-')}.csv"
    path = os.path.join(SAVE_DIR, filename)

    csv_file = open(path, mode="w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    current_filename = path

    header = [
        "PC_Timestamp",
        "Ax", "Ay", "Az",
        "Gx", "Gy", "Gz",
        "Mx", "My", "Mz",
        "Qx", "Qy", "Qz", "Qw"
    ]
    csv_writer.writerow(header)
    csv_file.flush()

    print(f">> [REC] New file for this connection: {path}")


def handle_client_receive(conn, addr, stop_event: threading.Event):
    """
    受信スレッド：切れたら終了（メインはacceptへ戻る）
    """
    buffer = ""
    print(f"[Info] Connection established from: {addr}")

    try:
        conn.settimeout(1.0)
        while not stop_event.is_set():
            try:
                data = conn.recv(4096)
                if not data:
                    print("[Info] Connection closed by peer.")
                    break

                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    # --- 1) ワンショット精度 ---
                    if line.startswith("ACCURACY_ONCE:"):
                        print(f"[Info] Accuracy: {line}")

                        with state_lock:
                            if is_logging and csv_writer:
                                csv_writer.writerow(["# Initial Calibration Accuracy", line.split(":", 1)[1]])
                                csv_file.flush()

                        # ★REC中ならMEASUREへ（ESPに計測再開させる）
                        with state_lock:
                            do_measure = is_logging
                        if do_measure:
                            ok = safe_send("MEASURE")
                            print("[Cmd] Auto-trigger: MEASURE" if ok else "[Warn] Auto MEASURE failed")

                    # --- 2) データCSV行 ---
                    elif "," in line and not line.startswith("CALIB") and not line.startswith("STATUS") and not line.startswith(">>") and not line.startswith("HELLO"):
                        with state_lock:
                            if not (is_logging and csv_writer):
                                # REC中でなければ捨てる（または表示）
                                continue

                            now = datetime.datetime.now()
                            timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            row = [timestamp_str] + line.split(",")
                            csv_writer.writerow(row)

                            # フラッシュは安全側（重いなら間引き）
                            csv_file.flush()

                    # --- 3) その他メッセージ ---
                    else:
                        print(f"[Msg] {line}")

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Recv Error] {e}")
                break

    finally:
        try:
            conn.close()
        except Exception:
            pass
        print("[Info] Receiver thread ended.")


def on_new_connection(conn, addr):
    """
    新しい接続が来たときの処理：
    - conn共有を更新
    - 受信スレッドを起動
    - REC中なら：新しいファイル作成 → GET_ONCE を送る（→受信側がMEASURE自動）
    """
    global recv_stop_event

    # 接続差し替え
    set_conn(conn)

    # 旧受信スレッド停止
    recv_stop_event.set()
    recv_stop_event = threading.Event()

    # 受信スレッド開始
    t = threading.Thread(target=handle_client_receive, args=(conn, addr, recv_stop_event), daemon=True)
    t.start()

    # REC状態なら、新規ファイルへ切り替えて再開準備
    with state_lock:
        rec = is_logging
        if rec:
            open_new_log_file_for_connection(addr)

    if rec:
        print("[Auto] REC is active -> GET_ONCE (then auto MEASURE)")
        if not safe_send("GET_ONCE"):
            print("[Warn] GET_ONCE send failed (not connected?)")


def accept_loop(server: socket.socket):
    while True:
        print("Waiting for ESP32 connection...")
        conn, addr = server.accept()
        on_new_connection(conn, addr)


def main():
    global is_logging

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((SERVER_IP, SERVER_PORT))
        server.listen(1)
        print(f"[Start] Listening on port {SERVER_PORT}...")

        threading.Thread(target=accept_loop, args=(server,), daemon=True).start()

        print("==================================================")
        print(" Commands:")
        print("  REC   -> Start logging (kept until STOP). New file on each reconnect.")
        print("  STOP  -> Stop logging & close file. Send STOP to ESP.")
        print("  CALIB -> Calibration mode")
        print("  SAVE  -> Save calibration to flash")
        print("  GET_ONCE -> Request accuracy once")
        print("  EXIT  -> Exit program")
        print("==================================================")

        while True:
            cmd = input("Cmd > ").strip().upper()
            if not cmd:
                continue

            if cmd == "EXIT":
                break

            if cmd == "REC":
                with state_lock:
                    is_logging = True
                print(">> [REC] Logging state ON (will persist until STOP)")
                # 接続中ならこの接続用にファイル作成して開始
                conn = get_conn()
                if conn is not None:
                    with state_lock:
                        open_new_log_file_for_connection(conn.getpeername())
                    if not safe_send("GET_ONCE"):
                        print("[Warn] Not connected now. Will auto-resume on next connect.")
                else:
                    print("[Warn] Not connected now. Will create new file & start when ESP connects.")

            elif cmd == "STOP":
                with state_lock:
                    is_logging = False
                    close_log_file_nolock()
                print(">> [STOP] Logging state OFF")
                if not safe_send("STOP"):
                    print("[Warn] Not connected now. (ESP will stop when you send STOP after reconnect.)")

            else:
                if not safe_send(cmd):
                    print("[Warn] Not connected now. Command not sent.")

    except KeyboardInterrupt:
        print("\n[Exit]")
    finally:
        recv_stop_event.set()
        c = get_conn()
        if c:
            try:
                c.close()
            except Exception:
                pass
        with state_lock:
            close_log_file_nolock()
        server.close()


if __name__ == "__main__":
    main()