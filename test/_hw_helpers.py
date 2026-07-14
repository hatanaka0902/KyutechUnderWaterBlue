"""
test/ 配下の各 test_*.py が共通で使う実機テスト用ヘルパー。

このモジュール自体はテストケースを持たない(ファイル名が test_*.py ではないため
unittest discover / pytest からは収集されない)。

含まれるもの:
    - confirm_hardware_action(): 実機を動かす前に人間の確認を取るためのプロンプト
    - safe_stop_and_disarm():   実機デモの try/finally で必ず呼び、停止・Disarmする
    - hardware_flag_present():  `python test/test_xxx.py --hardware` のように
                                 明示的なフラグが付いた時だけ実機デモを走らせるための判定
    - FakeMavHandle / FakeVehicle / patched_mavlink_connection():
                                 pymavlink.mavutil.mavlink_connection を実機なしで
                                 差し替えるためのフェイク実装(mavlink_io.py の単体テスト用)
"""

from __future__ import annotations

import sys
from contextlib import contextmanager


def hardware_flag_present() -> bool:
    """コマンドライン引数に --hardware が付いているかどうかを返す。

    実機を動かすデモは、これが True の時だけ実行する(誤って `python -m unittest`
    等の一括実行で実機が動いてしまう事故を防ぐためのガード)。
    """
    return "--hardware" in sys.argv


def confirm_hardware_action(message: str) -> bool:
    """実機のスラスタ・グリッパー等を実際に動かす直前に人間の確認を取る。

    Args:
        message: 確認内容の説明("前進1秒動かします" など)
    Returns:
        ユーザーが 'y' または 'yes' (大文字小文字無視) と答えたら True。
        それ以外(何も入力せずEnter含む)は False = 安全側にキャンセル。
    """
    try:
        answer = input(f"[実機注意] {message} 実行しますか? (y/N): ").strip().lower()
    except EOFError:
        # 標準入力が無い環境(CI等)では安全側に倒して常に拒否する
        return False
    return answer in ("y", "yes")


def safe_stop_and_disarm(mavlink_io_module, note: str = "") -> None:
    """実機デモの try/finally で必ず呼ぶ安全停止処理。

    ManualControl(0,0,500,0) で全軸ニュートラルに戻し、続けて Disarm() する。
    どちらかが例外を出しても、もう片方の停止処理は必ず試みる(実機を
    動かしっぱなしにしないことを最優先する)。
    """
    if note:
        print(f"[安全停止] {note}")
    try:
        mavlink_io_module.ManualControl(0, 0, 500, 0)
        print("[安全停止] ManualControl(0,0,500,0) 送信済み(全軸ニュートラル)")
    except Exception as e:
        print(f"[安全停止] ManualControl送信に失敗: {e}")
    try:
        mavlink_io_module.Disarm()
    except Exception as e:
        print(f"[安全停止] Disarmに失敗: {e}")


# ---------------------------------------------------------------------------
# mavlink_io.py をハードウェア無しで単体テストするためのフェイク接続。
# 実際の pymavlink.mavutil.mavlink_connection() の戻り値(vehicle)が持つ
# メソッド/属性のうち、mavlink_io.py が使うものだけを最小限real実装せずに再現する。
# ---------------------------------------------------------------------------

class _Recorded:
    """送信された1回分のMAVLinkコマンドの記録(引数をそのまま保持)。"""

    def __init__(self, name, args, kwargs):
        self.name = name
        self.args = args
        self.kwargs = kwargs

    def __repr__(self):
        return f"_Recorded({self.name}, args={self.args}, kwargs={self.kwargs})"


class FakeMavHandle:
    """vehicle.mav.xxx_send() 呼び出しを記録するだけのフェイク。"""

    def __init__(self, calls: list):
        self._calls = calls

    def command_long_send(self, *args, **kwargs):
        self._calls.append(_Recorded("command_long_send", args, kwargs))

    def set_mode_send(self, *args, **kwargs):
        self._calls.append(_Recorded("set_mode_send", args, kwargs))

    def manual_control_send(self, *args, **kwargs):
        self._calls.append(_Recorded("manual_control_send", args, kwargs))


class FakeVehicle:
    """mavutil.mavlink_connection() の戻り値を模したフェイク。実ネットワーク通信をしない。"""

    def __init__(self):
        self.calls: list = []
        self.mav = FakeMavHandle(self.calls)
        self.target_system = 1
        self.target_component = 1
        self.armed_wait_count = 0
        self.disarmed_wait_count = 0
        self.reboot_count = 0
        self._mode_mapping = {"MANUAL": 19, "STABILIZE": 0, "ALT_HOLD": 2}

    def recv_match(self, *args, **kwargs):
        """接続直後のheartbeat待ち受けを模す。実際の待機はせず即時ダミーを返す。"""
        return "FAKE_HEARTBEAT"

    def motors_armed_wait(self):
        self.armed_wait_count += 1

    def motors_disarmed_wait(self):
        self.disarmed_wait_count += 1

    def mode_mapping(self):
        return dict(self._mode_mapping)

    def reboot_autopilot(self):
        self.reboot_count += 1


@contextmanager
def patched_mavlink_connection(monkeypatch_target="pymavlink.mavutil.mavlink_connection"):
    """`with patched_mavlink_connection() as fake_vehicle:` で使う。

    pymavlink.mavutil.mavlink_connection をこの with 節の間だけ FakeVehicle を返す
    ものに差し替える。呼び出し元は差し替え後に (再)import した mavlink_io モジュールが
    このフェイク接続に対して命令を送るので、実機・実ネットワークなしで
    Arm/Disarm/ChangeMode/SetPwm/ManualControl/CameraTilt の引数組み立てロジックを検証できる。
    """
    from unittest.mock import patch as _patch

    fake_vehicle = FakeVehicle()
    with _patch(monkeypatch_target, return_value=fake_vehicle):
        yield fake_vehicle
