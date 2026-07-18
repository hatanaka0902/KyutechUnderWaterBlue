"""
DvlStream / function.get_latest_dvext() の実ソケット統合テスト。
モックではなく、実際にUDPソケットでパケットを送受信して検証する。
"""
import socket
import time
import sys

import params
params.DVL_BIND_IP = "127.0.0.1"
params.DVL_UDP_PORT = 27099  # テスト用の空きポート

import function  # ここで実際にDvlStream/ImuStreamの受信スレッドが起動する


def make_dvext_sentence(dvl_lock="T", vel_north=0.3, vel_east=0.1, vel_up=0.05,
                          altitude=3.2, heading=90.0, pitch=2.5,
                          qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """正しいチェックサム付きの$DVEXTテスト文字列を作る(parse_dvext()と同じ計算式)"""
    fields = [
        "DVEXT", dvl_lock, "1", "3333", "1.5", str(pitch), str(heading),
        "0", str(vel_up), str(altitude), str(vel_north), str(vel_east),
        "0.0", "0.0", "0.02",
        str(qw), str(qx), str(qy), str(qz),
        "0", "0", "0", "0",
        "1", "1", "1", "1",
        "0.1", "0.1", "0.1", "0.1",
        "3.0", "3.0", "3.0", "3.0",
    ]
    payload = ",".join(fields)
    checksum = 0
    for ch in payload:
        checksum ^= ord(ch)
    return f"${payload}*{checksum:02X}"


sentence = make_dvext_sentence()
print(f"送信するテスト文字列:\n  {sentence}\n")

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.sendto(sentence.encode("ascii"), (params.DVL_BIND_IP, params.DVL_UDP_PORT))
time.sleep(0.2)  # 受信スレッドが拾うのを待つ

print("=== テスト1: get_latest_dvext()が正しくパースできるか ===")
result = function.get_latest_dvext()
print(f"結果: {result}")
assert result is not None, "Noneが返ってきた(受信できていない)"
assert result["dvl_lock"] is True
assert abs(result["vel_north"] - 0.3) < 1e-6
assert abs(result["altitude"] - 3.2) < 1e-6
print("  -> OK: 送った値が正しくパースされている\n")

print("=== テスト2: get_velocity_data()が機体座標系速度を返すか ===")
v_body = function.get_velocity_data()
print(f"結果: {v_body}")
assert v_body is not None
print("  -> OK\n")

print("=== テスト3: 鮮度チェック(stale_timeout超過でNoneになるか) ===")
print("0.6秒待機します(stale_timeout=0.5秒)...")
time.sleep(0.6)
result_stale = function.get_latest_dvext()
print(f"結果: {result_stale}")
assert result_stale is None, "stale_timeoutを超えたのにNoneが返ってこなかった"
print("  -> OK: 古いデータは破棄されている\n")

print("=== テスト4: dvl_lock=Falseのときget_velocity_data()がNoneを返すか ===")
sentence_unlocked = make_dvext_sentence(dvl_lock="F")
client.sendto(sentence_unlocked.encode("ascii"), (params.DVL_BIND_IP, params.DVL_UDP_PORT))
time.sleep(0.2)
v_body_unlocked = function.get_velocity_data()
print(f"結果: {v_body_unlocked}")
assert v_body_unlocked is None
print("  -> OK: ロスト時はNoneを返す\n")

print("全テスト成功")
client.close()
