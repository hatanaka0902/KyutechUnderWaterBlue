import socket

import json

import datetime

import threading

import sys



# ==============================

# 設定

# ==============================

UDP_PORT = 27000

BIND_IP = "192.168.2.10"





def make_out_path():

    # RECした瞬間の時刻で新規ファイル名を作る

    return datetime.datetime.now().strftime("dvl_%Y%m%d_%H%M%S.jsonl")





# DVEXT: 公式順に合わせたフィールド列（あなたのDVEXTログに合わせる）

DVEXT_FIELDS = [

    "dvl_lock", "gps_status", "imu_cal_status", "roll_deg", "pitch_deg", "heading_deg",

    "data_skips", "vel_up_mps", "altitude_m", "vel_north_mps", "vel_east_mps",

    "latitude_deg", "longitude_deg", "ekf_dt_s",

    "quat_w", "quat_x", "quat_y", "quat_z",

    "gain_a_db", "gain_b_db", "gain_c_db", "gain_d_db",

    "lock_a", "lock_b", "lock_c", "lock_d",

    "vel_a_mps", "vel_b_mps", "vel_c_mps", "vel_d_mps",

    "range_a_m", "range_b_m", "range_c_m", "range_d_m",

]



# DVPDL: 公式順

# $DVPDL, tu, dtu, adr, adp, ady, pdx, pdy, pdz, c *hh

DVPDL_FIELDS = [

    "tu_usec",

    "dtu_usec",

    "adr_rad",

    "adp_rad",

    "ady_rad",

    "pdx_m",

    "pdy_m",

    "pdz_m",

    "confidence",

]



# ---- 「最重要」だけ保存/表示 ----

DVEXT_MIN_KEYS = ["dvl_lock", "vel_north_mps", "vel_east_mps", "vel_up_mps", "heading_deg"]

DVPDL_MIN_KEYS = ["tu_usec", "dtu_usec", "pdx_m", "pdy_m", "pdz_m", "confidence"]





# ==============================

# パース関連

# ==============================

def _to_num(s: str):

    s = s.strip()

    if s == "":

        return None

    try:

        return float(s)

    except ValueError:

        return s





def _split_checksum(line: str):

    if "*" in line:

        main, cs = line.split("*", 1)

        return main, cs.strip()

    return line, None





def parse_dvext_min(line: str):

    line = line.strip()

    if line.startswith("$"):

        line = line[1:]

    main, _cs = _split_checksum(line)

    parts = main.split(",")

    if not parts or parts[0] != "DVEXT":

        return None

    vals = parts[1:]

    vals = (vals + [""] * len(DVEXT_FIELDS))[:len(DVEXT_FIELDS)]



    full = {}

    for k, v in zip(DVEXT_FIELDS, vals):

        if k == "dvl_lock":

            full[k] = v.strip()  # "T"/"F"

        else:

            full[k] = _to_num(v)



    return {k: full.get(k) for k in DVEXT_MIN_KEYS}





def parse_dvpdl_min(line: str):

    line = line.strip()

    if line.startswith("$"):

        line = line[1:]

    main, _cs = _split_checksum(line)

    parts = main.split(",")

    if not parts or parts[0] != "DVPDL":

        return None

    vals = parts[1:]

    vals = (vals + [""] * len(DVPDL_FIELDS))[:len(DVPDL_FIELDS)]

    full = {k: _to_num(v) for k, v in zip(DVPDL_FIELDS, vals)}

    return {k: full.get(k) for k in DVPDL_MIN_KEYS}





# ==============================

# ログ制御（REC/STOP）

# ==============================

logging_enabled = False

out_path = None

state_lock = threading.Lock()





def append_jsonl(obj: dict, path: str):

    with open(path, "a", encoding="utf-8") as f:

        f.write(json.dumps(obj, ensure_ascii=False) + "\n")





def print_menu():

    print("==================================================")

    print(" Commands:")

    print("  REC   -> Create file, Start Logging (print+save)")

    print("  STOP  -> Stop Logging (silent)")

    print("==================================================")





def input_thread():

    global logging_enabled, out_path

    print_menu()

    while True:

        cmd = sys.stdin.readline()

        if not cmd:

            continue

        cmd = cmd.strip().upper()



        if cmd == "REC":

            with state_lock:

                out_path = make_out_path()

                logging_enabled = True

                path = out_path

            print(f"[REC] Logging START -> {path}")



        elif cmd == "STOP":

            with state_lock:

                logging_enabled = False

                closed_path = out_path

                out_path = None

            print(f"[STOP] Logging STOP -> {closed_path}")



        elif cmd == "":

            continue

        else:

            print(f"[WARN] Unknown command: {cmd}")

            print_menu()





# ==============================

# main

# ==============================

def main():

    # 入力受付スレッド開始

    th = threading.Thread(target=input_thread, daemon=True)

    th.start()



    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind((BIND_IP, UDP_PORT))

    sock.settimeout(0.2)  # 入力と両立させるためタイムアウト



    print(f"Listening on UDP {UDP_PORT}... (type REC to start logging)")



    while True:

        try:

            payload, _addr = sock.recvfrom(4096)

        except socket.timeout:

            continue



        text = payload.decode("ascii", errors="ignore").strip()

        if not text:

            continue



        # ★STOP中は完全サイレント：何も表示しない・保存もしない

        with state_lock:

            do_log = logging_enabled

            path = out_path

        if not do_log or not path:

            continue



        ts = datetime.datetime.now().isoformat()



        # REC中のみ：表示＆保存

        if text.startswith("$DVPDL"):

            p = parse_dvpdl_min(text)

            if p:

                rec = {"timestamp": ts, "msg": "DVPDL", **p}

                append_jsonl(rec, path)

                print(f"[DVPDL] d=({p['pdx_m']},{p['pdy_m']},{p['pdz_m']}) conf={p['confidence']}")



        elif text.startswith("$DVEXT"):

            p = parse_dvext_min(text)

            if p:

                rec = {"timestamp": ts, "msg": "DVEXT", **p}

                append_jsonl(rec, path)

                print(

                    f"[DVEXT] lock={p['dvl_lock']} "

                    f"Vn={p['vel_north_mps']} "

                    f"Ve={p['vel_east_mps']} "

                    f"Vu={p['vel_up_mps']} "

                    f"hdg={p['heading_deg']}"

                )



        else:

            # REC中のみ：DVEXT/DVPDL以外も見たいならここを有効化

            # print(f"[RAW] {text[:200]}")

            pass





if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("\nCtrl+C を受け取ったので終了します。")