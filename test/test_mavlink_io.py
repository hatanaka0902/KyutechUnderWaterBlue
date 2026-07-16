"""
Control/mavlink_io.py のテスト。

対象関数(MAVLink低レベルI/O):
    - Arm() / Disarm()            : 機体のアーム/ディスアーム
    - ChangeMode(mode)            : 操縦モード変更
    - SetPwm(channel_id, pwm)     : RCチャンネルPWM出力
    - ManualControl(x,y,z,yaw)    : 手動操縦指令送信
    - CameraTilt(tilt,roll,pan)   : カメラジンバル制御
    - GainUp()/GainDown()         : 操作ゲイン調整(既知バグ: GAIN未初期化でNameError)

mavlink_io.py は import された瞬間に実機/SITLへ接続してheartbeatをブロッキング
待機する(CLAUDE.md参照)。そのため:
    - 単体テスト(TestMavlinkIoWithFakeConnection)は pymavlink.mavutil.mavlink_connection
      をフェイク実装に差し替えた状態でモジュールをimportし、実機なしで
      「引数の組み立てが正しいか」を検証する。
    - 実機デモ(run_hardware_demo)は差し替えを行わず、実際にBlueROVへ接続して
      Arm/カメラチルト/前後進を試す。安全のため各操作の前に確認プロンプトを出し、
      終了時(正常終了・例外・Ctrl+C問わず)に必ずManualControl(0,0,500,0)+Disarm()する。

実行方法:
    python test/test_mavlink_io.py             # 単体テスト(フェイク接続、実機不要)
    python test/test_mavlink_io.py --hardware   # 実機デモ(要: 実機BlueROVに接続済み)
"""

import os
import sys
import time
import unittest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROL_DIR = os.path.join(os.path.dirname(_TEST_DIR), "Control")
for _dir in (_CONTROL_DIR, _TEST_DIR):
    if _dir not in sys.path:
        sys.path.insert(0, _dir)

from unittest.mock import patch  # noqa: E402

from _hw_helpers import (  # noqa: E402
    FakeVehicle, hardware_flag_present, confirm_hardware_action, safe_stop_and_disarm,
)


def _import_fresh_mavlink_io_with_fake(fake_vehicle):
    """pymavlink.mavutil.mavlink_connection をフェイクに差し替えた状態で
    mavlink_io.py をキャッシュから外して再importする(モジュール読み込み時の
    接続コードがフェイクに対して実行される)。"""
    sys.modules.pop("mavlink_io", None)
    with patch("pymavlink.mavutil.mavlink_connection", return_value=fake_vehicle):
        import mavlink_io  # noqa: F401
    return mavlink_io


class TestMavlinkIoWithFakeConnection(unittest.TestCase):
    """実機なしで mavlink_io.py の各関数が「正しい引数でMAVLinkコマンドを組み立てて
    いるか」を検証する。実際のソケット通信は行わず、FakeVehicleが呼び出しを記録する。"""

    @classmethod
    def setUpClass(cls):
        cls.fake = FakeVehicle()
        cls.mavlink_io = _import_fresh_mavlink_io_with_fake(cls.fake)

    def setUp(self):
        # 各テストの記録を独立させるため、前のテストの呼び出し履歴をクリアする
        self.fake.calls.clear()
        self.fake.armed_wait_count = 0
        self.fake.disarmed_wait_count = 0

    def test_arm_sends_arm_command_and_waits(self):
        """目的: Arm()がMAV_CMD_COMPONENT_ARM_DISARMをparam1=1(アーム)で送り、
        motors_armed_wait()で確認待ちすることを確認する。
        役割: Arm()は制御ループ開始前に必ず呼ばれる、機体を動作可能にする関数。
        param1を誤ると実機がアームされず、以降のManualControl等が全て無視される。
        結果が示すこと: command_long_sendが1回呼ばれ、コマンドIDと param1=1 が正しい。"""
        from pymavlink import mavutil
        self.mavlink_io.Arm()
        self.assertEqual(len(self.fake.calls), 1)
        call = self.fake.calls[0]
        self.assertEqual(call.name, "command_long_send")
        self.assertEqual(call.args[2], mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(call.args[4], 1)  # param1=1 -> arm
        self.assertEqual(self.fake.armed_wait_count, 1)

    def test_disarm_sends_disarm_command_and_waits(self):
        """目的: Disarm()が同じコマンドをparam1=0(ディスアーム)で送り、
        motors_disarmed_wait()で確認待ちすることを確認する。
        役割: 実機デモ・実運用の終了時に必ず呼ぶ安全確保の関数。ここが機能しないと
        デモ終了後もスラスタに制御権が残った状態になりうる。
        結果が示すこと: command_long_sendが1回呼ばれ、param1=0。"""
        from pymavlink import mavutil
        self.mavlink_io.Disarm()
        self.assertEqual(len(self.fake.calls), 1)
        call = self.fake.calls[0]
        self.assertEqual(call.args[2], mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(call.args[4], 0)
        self.assertEqual(self.fake.disarmed_wait_count, 1)

    def test_change_mode_to_known_mode_sends_correct_mode_id(self):
        """目的: ChangeMode('MANUAL')が mode_mapping() から解決した正しい mode_id で
        set_mode_send を呼ぶことを確認する。
        役割: main.py起動時に最初に呼ばれ、以降のManualControlが機体に反映される
        前提となる操縦モード切替。モードIDを誤ると指令自体が無視される。
        結果が示すこと: set_mode_sendが呼ばれ、3番目の引数(mode_id)が
        FakeVehicle.mode_mapping()['MANUAL']と一致する。"""
        self.mavlink_io.ChangeMode("MANUAL")
        self.assertEqual(len(self.fake.calls), 1)
        call = self.fake.calls[0]
        self.assertEqual(call.name, "set_mode_send")
        self.assertEqual(call.args[2], self.fake.mode_mapping()["MANUAL"])

    def test_change_mode_to_unknown_mode_exits(self):
        """目的: 未知のモード名を渡すとChangeMode()がsys.exit(1)で終了することを確認する。
        役割: 存在しないモード名でのMAVLinkコマンド送信を未然に防ぐ入力検証の確認。
        結果が示すこと: ChangeMode('NOT_A_REAL_MODE')はSystemExitを発生させ、
        set_mode_sendは呼ばれない(=不正なコマンドを送信しない)。"""
        with self.assertRaises(SystemExit):
            self.mavlink_io.ChangeMode("NOT_A_REAL_MODE")
        self.assertEqual(len(self.fake.calls), 0)

    def test_set_pwm_rejects_out_of_range_channel(self):
        """目的: channel_idが1〜14の範囲外の場合、command_long_sendが送信されないことを確認する。
        役割: ArduSubのRCチャンネル範囲外への誤送信を防ぐガード(既存コードのif文)の確認。
        結果が示すこと: channel_id=0 と channel_id=15 のどちらもコマンドを送らない。"""
        self.mavlink_io.SetPwm(0, 1500)
        self.mavlink_io.SetPwm(15, 1500)
        self.assertEqual(len(self.fake.calls), 0)

    def test_set_pwm_valid_channel_sends_correct_command(self):
        """目的: 有効なchannel_idではMAV_CMD_DO_SET_SERVOをchannel_id・pwm値付きで
        送信することを確認する。
        役割: LEDやグリッパー等、ManualControl以外のアクチュエータ制御の基礎関数。
        結果が示すこと: command_long_sendの引数にチャンネル7・PWM1600が正しく含まれる。"""
        from pymavlink import mavutil
        self.mavlink_io.SetPwm(7, 1600)
        self.assertEqual(len(self.fake.calls), 1)
        call = self.fake.calls[0]
        self.assertEqual(call.args[2], mavutil.mavlink.MAV_CMD_DO_SET_SERVO)
        self.assertEqual(call.args[4], 7)
        self.assertEqual(call.args[5], 1600)

    def test_manual_control_sends_exact_axis_values(self):
        """目的: ManualControl(x,y,z,yaw)が manual_control_send にそのままx,y,z,yawを
        渡していることを確認する(docstringに記載のMANUAL_CONTROLチャンネル規約)。
        役割: controlfunction.run_control_loop()が毎周期呼ぶ、実機を実際に動かす
        最終出口。軸の順序・値を誤ると意図と異なる方向にROVが動く。
        結果が示すこと: manual_control_sendの引数(x,y,z,yaw)が呼び出し時の値と一致する。"""
        self.mavlink_io.ManualControl(100, -50, 600, 10)
        self.assertEqual(len(self.fake.calls), 1)
        call = self.fake.calls[0]
        self.assertEqual(call.name, "manual_control_send")
        # 署名: manual_control_send(target_system, x, y, z, yaw, buttons)
        self.assertEqual(call.args[1:5], (100, -50, 600, 10))

    def test_camera_tilt_sends_mount_control_with_given_angles(self):
        """目的: CameraTilt(tilt,roll,pan)がMAV_CMD_DO_MOUNT_CONTROLをtilt/roll/pan付きで
        送信することを確認する。
        役割: カメラジンバルの姿勢制御。実機デモでの動作確認(±45度スイープ)の
        土台となる関数。
        結果が示すこと: command_long_sendの引数にtilt=4500,roll=100,pan=-100が
        正しい位置に入っている。"""
        from pymavlink import mavutil
        self.mavlink_io.CameraTilt(4500, roll=100, pan=-100)
        self.assertEqual(len(self.fake.calls), 1)
        call = self.fake.calls[0]
        self.assertEqual(call.args[2], mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL)
        self.assertEqual(call.args[4], 4500)
        self.assertEqual(call.args[5], 100)
        self.assertEqual(call.args[6], -100)

    def test_gain_up_raises_name_error_known_bug(self):
        """目的: GainUp()が「GAINが未初期化のグローバル変数」バグによりNameErrorを
        発生させることを、回帰テストとして固定する。
        役割: CLAUDE.md/コードコメントに記載された既知バグ(修正せず現状維持と
        されている)を、将来偶発的に直って動作が変わった場合に気付けるようにする
        ドキュメント的テスト。
        結果が示すこと: 現状のコードではGainUp()呼び出しがNameErrorになる。"""
        with self.assertRaises(NameError):
            self.mavlink_io.GainUp()

    def test_gain_down_raises_name_error_known_bug(self):
        """目的: GainDown()も同じ既知バグ(GAIN未初期化)でNameErrorになることを確認する。
        役割: GainUpと対になる関数で同種のバグが存在することの回帰テスト。
        結果が示すこと: 現状のコードではGainDown()呼び出しがNameErrorになる。"""
        with self.assertRaises(NameError):
            self.mavlink_io.GainDown()


# ---------------------------------------------------------------------------
# 実機デモ: 実際のBlueROVに接続し、Arm/カメラチルト/前後進/Disarmを試す。
# --hardware フラグ付きで実行した時だけ動く(通常のテスト実行では動かない)。
# ---------------------------------------------------------------------------
def run_hardware_demo():
    print("=== mavlink_io.py 実機デモ ===")
    print(f"実機のBlueROVに接続します(pid未指定・{__name__} からの直接接続)。")
    print("接続先はparams.MAVLINK_CONNECTION_STRING。BlueROVの電源・ネットワークを確認してください。")

    sys.modules.pop("mavlink_io", None)
    import mavlink_io  # フェイクを挟まない実接続。import時にheartbeat待ちでブロックする

    try:
        if not confirm_hardware_action("ChangeMode('MANUAL') に切り替え、Arm() します。"):
            print("キャンセルしました。")
            return
        mavlink_io.ChangeMode("MANUAL")
        mavlink_io.Arm()

        if confirm_hardware_action("カメラチルトを +45度 → -45度 → 0度 の順にスイープします。"):
            mavlink_io.CameraTilt(45 * 100)
            time.sleep(0.5)
            mavlink_io.CameraTilt(-45 * 100)
            time.sleep(0.5)
            mavlink_io.CameraTilt(0)
            time.sleep(0.5)

        if confirm_hardware_action("前進(x=200)を1秒、後進(x=-200)を1秒、ManualControlで送ります。"):
            mavlink_io.ManualControl(200, 0, 500, 0)
            time.sleep(1)
            mavlink_io.ManualControl(-200, 0, 500, 0)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[中断] Ctrl+Cを検知しました。安全停止します。")
    finally:
        safe_stop_and_disarm(mavlink_io, note="mavlink_io実機デモ終了")


if __name__ == "__main__":
    if hardware_flag_present():
        run_hardware_demo()
    else:
        sys.argv = [a for a in sys.argv if a != "--hardware"]
        unittest.main(verbosity=2)
