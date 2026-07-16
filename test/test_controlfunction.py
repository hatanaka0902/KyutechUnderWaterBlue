"""
Control/controlfunction.py のテスト。

対象関数(誘導則・カスケードPID制御ループ):
    - EmergencyProblem(current_state)                                : 緊急停止判定
    - AzimuthControl(target_x, target_y)                              : 目標方位角の計算
    - OuterLoopControl(dx,dy,dz,current_yaw_deg,dt)                   : 外側ループ(位置/yaw誤差→速度setpoint)
    - get_target_position()/is_target_reached(...)                    : 目標位置取得/到達判定
    - get_body_frame_velocity(current_state)/get_yaw_rate_deg(...)    : 内側ループ用の実測値取得
    - InnerLoopControl(...)                                            : 内側ループ(速度setpoint→推力指令)
    - reset_all_pids()                                                 : 全6PIDのリセット
    - run_control_loop()                                               : メイン制御ループ

controlfunction.py は import 時に `from mavlink_io import ManualControl` を実行するため、
そのままimportすると実機/SITLへの接続・heartbeat待ちが発生する(CLAUDE.md参照)。
そのため:
    - 単体テストは、sys.modules['mavlink_io'] にダミーモジュール(呼び出しを記録するだけの
      ManualControl)を差し込んだ状態でcontrolfunction.pyをimportし、実機なしで
      誘導則の計算ロジックのみを検証する。
    - 実機走行デモ(run_hardware_demo)はダミーを使わず、実際にBlueROVへ接続してArmし、
      OuterLoopControl→InnerLoopControl→ManualControlの一連の流れを実際に動かして確認する。
      get_target_position()は現状[0,0,0]固定のスタブなので、デモでは仮の目標オフセットを
      直接指定してカスケードPIDを動作させる。安全のため確認プロンプト・時間制限・
      Ctrl+C安全停止・終了時の強制停止&Disarmを備える。

実行方法:
    python test/test_controlfunction.py             # 単体テスト(フェイクmavlink_io、実機不要)
    python test/test_controlfunction.py --hardware   # 実機走行デモ(要: 実機BlueROVに接続済み)
"""

import math
import os
import sys
import time
import unittest
from unittest.mock import patch

import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROL_DIR = os.path.join(os.path.dirname(_TEST_DIR), "Control")
for _dir in (_CONTROL_DIR, _TEST_DIR):
    if _dir not in sys.path:
        sys.path.insert(0, _dir)

from _hw_helpers import hardware_flag_present, confirm_hardware_action, safe_stop_and_disarm  # noqa: E402


def _make_fake_mavlink_io():
    """controlfunction.py の `from mavlink_io import ManualControl` を実機なしで
    満たすための最小限のダミーモジュール。ManualControl呼び出しをcallsに記録する。"""
    import types
    module = types.ModuleType("mavlink_io")
    calls = []

    def ManualControl(x, y, z, yaw):
        calls.append((x, y, z, yaw))

    module.ManualControl = ManualControl
    module.calls = calls
    return module


def _import_fresh_controlfunction_with_fake_mavlink():
    fake_mavlink_io = _make_fake_mavlink_io()
    sys.modules["mavlink_io"] = fake_mavlink_io
    sys.modules.pop("controlfunction", None)
    import controlfunction  # noqa: F401 (importと同時にfunction.py経由でImuStreamも起動する)
    return controlfunction, fake_mavlink_io


class TestEmergencyProblem(unittest.TestCase):
    """EmergencyProblem(current_state): 現在は深度<0.1mのみで緊急停止を判定する。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()

    def test_shallow_depth_triggers_emergency(self):
        """目的: position z(current_state[2])が0.1m未満の時にTrueを返すことを確認する。
        役割: 水面付近でスラスタを動かし続けると転覆・空転する危険があるための、
        現状唯一実装されている緊急停止条件。
        結果が示すこと: z=0.05mを与えるとEmergencyProblem()はTrue。"""
        state = np.zeros(19)
        state[2] = 0.05
        self.assertTrue(self.cf.EmergencyProblem(state))

    def test_sufficient_depth_does_not_trigger_emergency(self):
        """目的: 十分な深度がある場合にFalseを返すことを確認する(誤検知しないことの確認)。
        役割: 通常運用中に緊急停止が誤って発火し、制御ループが毎回止まってしまう
        ことを防ぐための回帰テスト。
        結果が示すこと: z=1.0mを与えるとEmergencyProblem()はFalse。"""
        state = np.zeros(19)
        state[2] = 1.0
        self.assertFalse(self.cf.EmergencyProblem(state))

    def test_boundary_depth_is_not_emergency(self):
        """目的: 境界値z=0.1m(閾値と同値)がFalse(緊急停止しない)側になることを確認する。
        役割: current_state[2] < 0.1 という不等号(<, <=ではない)の実装を固定する回帰テスト。
        結果が示すこと: z=0.1mではEmergencyProblem()はFalse。"""
        state = np.zeros(19)
        state[2] = 0.1
        self.assertFalse(self.cf.EmergencyProblem(state))


class TestAzimuthControl(unittest.TestCase):
    """AzimuthControl(target_x, target_y): 目標座標へのyaw角[deg]、x軸正を0度、反時計回り正。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()

    def test_target_along_positive_x_is_zero_degrees(self):
        """目的: 目標が自機の正面(x軸正方向)にある時、方位角が0度になることを確認する。
        役割: NED座標系でのx軸正=前方という規約が正しく角度に変換されることの基礎確認。
        結果が示すこと: AzimuthControl(1,0)==0.0度。"""
        self.assertAlmostEqual(self.cf.AzimuthControl(1.0, 0.0), 0.0)

    def test_target_along_positive_y_is_90_degrees(self):
        """目的: 目標がy軸正方向(右方向)にある時、方位角が+90度になることを確認する。
        役割: OuterLoopControlのyaw誤差計算・InnerLoopControlのyawレート制御が
        期待する回転方向の規約を固定する。
        結果が示すこと: AzimuthControl(0,1)==90.0度。"""
        self.assertAlmostEqual(self.cf.AzimuthControl(0.0, 1.0), 90.0)

    def test_target_along_negative_x_is_180_degrees(self):
        """目的: 目標が真後ろにある時、方位角が180度(atan2の仕様上180か-180)になることを確認する。
        役割: 後方目標に対して正しく大きな旋回角を要求することの確認。
        結果が示すこと: |AzimuthControl(-1,0)| == 180.0度。"""
        self.assertAlmostEqual(abs(self.cf.AzimuthControl(-1.0, 0.0)), 180.0)

    def test_target_along_negative_y_is_minus_90_degrees(self):
        """目的: 目標がy軸負方向(左方向)にある時、方位角が-90度になることを確認する。
        役割: 90度方向と対称のケースを確認し、符号の反転漏れがないことを保証する。
        結果が示すこと: AzimuthControl(0,-1)==-90.0度。"""
        self.assertAlmostEqual(self.cf.AzimuthControl(0.0, -1.0), -90.0)


class TestIsTargetReached(unittest.TestCase):
    """is_target_reached(target_x,target_y,target_z): 水平距離<POSITION_TOLERANCEかつ
    |target_z|<DEPTH_TOLERANCEで到達とみなす。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()
        import params
        cls.params = params

    def test_within_both_tolerances_is_reached(self):
        """目的: 水平・深度誤差が両方とも許容範囲内の場合にTrueを返すことを確認する。
        役割: run_control_loop()がホールドモード(ManualControl(0,0,500,0))に入る
        条件を正しく判定できることの確認。
        結果が示すこと: 許容値の半分程度の誤差ではTrue。"""
        half_pos = self.params.POSITION_TOLERANCE / 2.0
        half_depth = self.params.DEPTH_TOLERANCE / 2.0
        self.assertTrue(self.cf.is_target_reached(half_pos, 0.0, half_depth))

    def test_horizontal_error_too_large_is_not_reached(self):
        """目的: 水平誤差が許容範囲を超える場合にFalseを返すことを確認する。
        役割: 水平方向にまだ距離がある間はホールドに入らず、追従を続けるべきという
        設計の確認。
        結果が示すこと: POSITION_TOLERANCEの2倍の水平誤差ではFalse。"""
        too_far = self.params.POSITION_TOLERANCE * 2.0
        self.assertFalse(self.cf.is_target_reached(too_far, 0.0, 0.0))

    def test_depth_error_too_large_is_not_reached(self):
        """目的: 深度誤差だけが許容範囲を超える場合もFalseを返すことを確認する
        (水平・深度の両方を満たす必要がある、AND条件であることの確認)。
        役割: 水平位置には到達していても深度が合っていない状態でホールドに
        入ってしまう誤判定を防ぐことの確認。
        結果が示すこと: 水平誤差0でもDEPTH_TOLERANCEの2倍の深度誤差ではFalse。"""
        too_deep = self.params.DEPTH_TOLERANCE * 2.0
        self.assertFalse(self.cf.is_target_reached(0.0, 0.0, too_deep))


class TestGetBodyFrameVelocity(unittest.TestCase):
    """get_body_frame_velocity(current_state): QEKFのNED速度をbody座標系に回転する。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()

    def test_identity_attitude_returns_same_velocity(self):
        """目的: 姿勢が単位クォータニオン(無回転)の時、body速度がworld速度と一致することを確認する。
        役割: InnerLoopControlのsurge/heave速度PIDが使う実測値の入口。回転行列の
        向きを誤ると全く違う軸の速度をsurgeとして扱ってしまう。
        結果が示すこと: current_state[3:6]=[1,2,3], quat=[1,0,0,0]でbody速度も[1,2,3]。"""
        state = np.zeros(19)
        state[6] = 1.0  # qw
        state[3:6] = [1.0, 2.0, 3.0]
        np.testing.assert_allclose(self.cf.get_body_frame_velocity(state), [1.0, 2.0, 3.0], atol=1e-9)

    def test_yaw_90deg_rotates_world_velocity_into_body_frame(self):
        """目的: yaw+90度の姿勢下で、world前方(x軸)速度がbody座標系では-y方向に見えることを確認する。
        役割: function.get_velocity_data()のDVL変換と対になる、逆方向(world→body)の
        変換規約が一貫していることの確認(同じ回転行列の転置を使う設計)。
        結果が示すこと: world速度[1,0,0]がyaw90度でbody≈[0,-1,0]になる。"""
        state = np.zeros(19)
        half = math.radians(90.0) / 2.0
        state[6] = math.cos(half)   # qw
        state[9] = math.sin(half)   # qz
        state[3:6] = [1.0, 0.0, 0.0]
        np.testing.assert_allclose(self.cf.get_body_frame_velocity(state), [0.0, -1.0, 0.0], atol=1e-9)


class TestGetYawRateDeg(unittest.TestCase):
    """get_yaw_rate_deg(current_state): 生ジャイロ - QEKF推定バイアス から yawレート[deg/s]。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()

    def test_subtracts_estimated_gyro_bias_from_raw_gyro(self):
        """目的: 生ジャイロz成分からQEKF推定のgyro_bias_z(current_state[15])を引いた値が
        度に変換されて返ることを確認する。
        役割: バイアス補正をしないと、ジャイロの定常誤差がyawレート制御の恒常的な
        オフセット誤差(旋回し続ける/止まらない等)として現れてしまう。
        結果が示すこと: raw_gyro_z=0.2rad/s, bias_z=0.05rad/sなら、
        degrees(0.2-0.05)が返る。"""
        state = np.zeros(19)
        state[15] = 0.05  # gyro_bias_z
        imu_data = np.array([[0.0, 0.0, 9.81], [0.0, 0.0, 0.2]])
        with patch.object(self.cf, "get_last_imu_data", return_value=imu_data):
            result = self.cf.get_yaw_rate_deg(state)
        self.assertAlmostEqual(result, math.degrees(0.2 - 0.05))

    def test_returns_zero_when_no_imu_data_available(self):
        """目的: get_last_imu_data()がNone(まだstate_estimation()が呼ばれていない等)の場合、
        0.0を安全側の既定値として返すことを確認する。
        役割: 起動直後のInnerLoopControl呼び出しで例外にならず、yawレート誤差0
        (何もしない)にフォールバックすることの確認。
        結果が示すこと: get_last_imu_data()がNoneの時get_yaw_rate_deg()は0.0。"""
        state = np.zeros(19)
        with patch.object(self.cf, "get_last_imu_data", return_value=None):
            self.assertEqual(self.cf.get_yaw_rate_deg(state), 0.0)


class TestOuterLoopControl(unittest.TestCase):
    """OuterLoopControl(dx,dy,dz,current_yaw_deg,dt): 位置/yaw誤差から速度setpointを計算する。
    params.py上、surge/heave/yaw outerは全てki=kd=0(純P制御)なので出力はkp*error(クランプ後)で決定的。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()
        import params
        cls.params = params

    def setUp(self):
        self.cf.reset_all_pids()

    def test_surge_setpoint_equals_kp_times_horizontal_distance(self):
        """目的: surge_setpointが水平距離(dx,dyのノルム)にsurge outerのkpを掛けた値
        (クランプ後)になることを確認する。
        役割: 「距離が大きいほど速く前進する」という外側ループの基本的な誘導則の確認。
        結果が示すこと: dx=0.1,dy=0(距離0.1)、kp=0.15なら surge_setpoint≈0.015 m/s
        (output_limits=(0,0.3)の範囲内なのでクランプされない)。"""
        surge_sp, _heave_sp, _yaw_rate_sp, _az = self.cf.OuterLoopControl(
            dx=0.1, dy=0.0, dz=0.0, current_yaw_deg=0.0, dt=0.1)
        expected = self.params.PID_SURGE_OUTER["kp"] * 0.1
        self.assertAlmostEqual(surge_sp, expected, places=6)

    def test_heave_setpoint_equals_kp_times_dz_ned_sign(self):
        """目的: heave_setpointがdz(NED、正=より深く)にheave outerのkpを掛けた値になり、
        符号反転されていないことを確認する(符号反転はManualControl送信直前のみで行う設計)。
        役割: controlfunction.pyのコメントに明記された「NEDのまま(符号反転しない)」という
        設計意図が実装と一致していることの回帰テスト。
        結果が示すこと: dz=0.1(下方向)、kp=0.3なら heave_setpoint≈+0.03 (正のまま)。"""
        _surge_sp, heave_sp, _yaw_rate_sp, _az = self.cf.OuterLoopControl(
            dx=1.0, dy=0.0, dz=0.1, current_yaw_deg=0.0, dt=0.1)
        expected = self.params.PID_HEAVE_OUTER["kp"] * 0.1
        self.assertAlmostEqual(heave_sp, expected, places=6)

    def test_yaw_rate_setpoint_uses_azimuth_error(self):
        """目的: yaw_rate_setpointが「目標方位角 - 現在yaw」の誤差にyaw outerのkpを掛けた値
        になることを確認する。
        役割: 目標座標の方向を向くための旋回レート指令が正しく計算されていることの確認。
        結果が示すこと: dx=0,dy=1(目標方位90度)、current_yaw_deg=0のとき、
        誤差90度×kp(=0.5)=45deg/sになる(output_limits=(-20,20)なので実際は20にクランプ)。"""
        _surge_sp, _heave_sp, yaw_rate_sp, target_az = self.cf.OuterLoopControl(
            dx=0.0, dy=1.0, dz=0.0, current_yaw_deg=0.0, dt=0.1)
        self.assertAlmostEqual(target_az, 90.0)
        lo, hi = self.params.PID_YAW_OUTER["output_limits"]
        self.assertAlmostEqual(yaw_rate_sp, hi)  # 45*kp相当は上限20でクランプされる

    def test_large_distance_clamps_surge_setpoint_to_output_limit(self):
        """目的: 非常に大きな距離誤差を与えても、surge_setpointがoutput_limitsの上限を
        超えないことを確認する。
        役割: 目標が遠い時に無制限の速度指令を出さない安全装置(output_limits)が
        OuterLoopControl経由でも機能していることの確認。
        結果が示すこと: dx=100mでもsurge_setpointはPID_SURGE_OUTERのoutput_limits上限以下。"""
        surge_sp, _h, _y, _a = self.cf.OuterLoopControl(
            dx=100.0, dy=0.0, dz=0.0, current_yaw_deg=0.0, dt=0.1)
        _lo, hi = self.params.PID_SURGE_OUTER["output_limits"]
        self.assertAlmostEqual(surge_sp, hi)


class TestInnerLoopControl(unittest.TestCase):
    """InnerLoopControl(surge_sp,heave_sp,yaw_rate_sp,current_state,dt): 速度setpointと
    実測値の誤差から推力/トルク指令を計算する内側ループ。全てki=kd=0(純P制御)。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()
        import params
        cls.params = params

    def setUp(self):
        self.cf.reset_all_pids()

    def _make_state(self, vx=0.0, vy=0.0, vz=0.0):
        state = np.zeros(19)
        state[6] = 1.0  # 単位クォータニオン(無回転)
        state[3:6] = [vx, vy, vz]
        return state

    def test_x_cmd_reacts_to_surge_error(self):
        """目的: 実測surge速度が0で、surge_setpointが正の値の時、x_cmdが正の値
        (kp*誤差、クランプ後)になることを確認する。
        役割: ManualControlのx(前後方向)に渡す最終的な指令値の計算根拠。ここが
        機能しないとPID制御を通した速度追従ができない。
        結果が示すこと: surge_setpoint=0.05, 実測0, kp=1000なら理論値50、
        output_limits=(0,400)なので50のまま。"""
        state = self._make_state(vx=0.0)
        with patch.object(self.cf, "get_last_imu_data", return_value=np.zeros((2, 3))):
            x_cmd, _z, _r = self.cf.InnerLoopControl(
                surge_setpoint=0.05, heave_setpoint=0.0, yaw_rate_setpoint=0.0,
                current_state=state, dt=0.1)
        expected = self.params.PID_SURGE_INNER["kp"] * 0.05
        self.assertAlmostEqual(x_cmd, expected, places=3)

    def test_z_cmd_offset_reacts_to_heave_error(self):
        """目的: heave_setpointと実測heave速度(current_state[5])の誤差にkpを掛けた値が
        z_cmd_offsetとして返ることを確認する(NED符号のまま、反転はしない)。
        役割: run_control_loop()側で最終的に符号反転して500に加算する前段の値が
        正しいことの確認。
        結果が示すこと: heave_setpoint=0.02, 実測vz=0, kp=800なら理論値16、
        output_limits=(-300,300)なので16のまま。"""
        state = self._make_state(vz=0.0)
        with patch.object(self.cf, "get_last_imu_data", return_value=np.zeros((2, 3))):
            _x, z_cmd_offset, _r = self.cf.InnerLoopControl(
                surge_setpoint=0.0, heave_setpoint=0.02, yaw_rate_setpoint=0.0,
                current_state=state, dt=0.1)
        expected = self.params.PID_HEAVE_INNER["kp"] * 0.02
        self.assertAlmostEqual(z_cmd_offset, expected, places=3)

    def test_r_cmd_reacts_to_yaw_rate_error(self):
        """目的: yaw_rate_setpointと実測yawレート(get_yaw_rate_deg)の誤差にkpを掛けた値が
        r_cmdとして返ることを確認する。
        役割: ManualControlのyaw(旋回)に渡す最終指令の計算根拠。
        結果が示すこと: yaw_rate_setpoint=2.0deg/s, 実測0(ジャイロ0・バイアス0)、kp=20なら
        理論値40、output_limits=(-500,500)なので40のまま。"""
        state = self._make_state()
        with patch.object(self.cf, "get_last_imu_data", return_value=np.zeros((2, 3))):
            _x, _z, r_cmd = self.cf.InnerLoopControl(
                surge_setpoint=0.0, heave_setpoint=0.0, yaw_rate_setpoint=2.0,
                current_state=state, dt=0.1)
        expected = self.params.PID_YAW_INNER["kp"] * 2.0
        self.assertAlmostEqual(r_cmd, expected, places=3)


class TestResetAllPids(unittest.TestCase):
    """reset_all_pids(): controlfunction内の6つのPIDインスタンス全てをリセットする。"""

    @classmethod
    def setUpClass(cls):
        cls.cf, cls.fake_mavlink_io = _import_fresh_controlfunction_with_fake_mavlink()

    def test_all_six_pids_are_reset(self):
        """目的: reset_all_pids()呼び出し後、6つ全てのPIDインスタンスの積分項が
        クリアされ、initializedフラグがFalseに戻ることを確認する。
        役割: run_control_loop()がホールド突入時に呼ぶ、ワインドアップ/微分キック
        対策。1つでも漏れていると、その軸だけ前回の状態を持ち越して不自然な動きになる。
        結果が示すこと: 6つのPIDインスタンス全てで integral==0.0, initialized==False。"""
        pid_names = ["_surge_outer_pid", "_heave_outer_pid", "_yaw_outer_pid",
                     "_surge_inner_pid", "_heave_inner_pid", "_yaw_inner_pid"]
        for name in pid_names:
            pid = getattr(self.cf, name)
            pid.update_from_error(1.0, dt=0.1)

        self.cf.reset_all_pids()

        for name in pid_names:
            pid = getattr(self.cf, name)
            with self.subTest(name=name):
                self.assertEqual(pid.integral, 0.0)
                self.assertFalse(pid.initialized)


# ---------------------------------------------------------------------------
# 実機走行デモ: 実際にBlueROVへ接続し、OuterLoopControl→InnerLoopControl→
# ManualControl のカスケードPID制御を、仮の目標オフセットに対して一定時間動かす。
# get_target_position()は現状[0,0,0]固定スタブのため、このデモでは直接オフセットを与える。
# --hardware フラグ付きで実行した時だけ動く。
# ---------------------------------------------------------------------------
def run_hardware_demo(duration_sec: float = 8.0, dx: float = 0.3, dy: float = 0.0, dz: float = 0.0):
    print("=== controlfunction.py 実機走行デモ ===")
    print("get_target_position()は現状[0,0,0]固定スタブのため、下記の仮目標オフセットを")
    print(f"直接与えます: dx={dx}m, dy={dy}m, dz={dz}m (NED)")
    print("OuterLoopControl -> InnerLoopControl -> ManualControl の一連の流れを実機で確認します。")

    if not confirm_hardware_action(
        f"MANUALモードに切り替えてArmし、{duration_sec}秒間カスケードPIDでManualControlを送ります。"
    ):
        print("キャンセルしました。")
        return

    # 単体テストで差し込んだフェイクmavlink_ioが残っていれば外し、実接続版を使う
    sys.modules.pop("mavlink_io", None)
    sys.modules.pop("controlfunction", None)
    import mavlink_io
    import controlfunction as cf
    import utility_functions

    cf.reset_all_pids()

    try:
        mavlink_io.ChangeMode("MANUAL")
        mavlink_io.Arm()

        start = time.time()
        while time.time() - start < duration_sec:
            current_state = cf.state_estimation()
            if cf.EmergencyProblem(current_state):
                print("[緊急停止] EmergencyProblem()がTrueを返しました。停止します。")
                break

            dt = cf.get_last_dt()
            _, _, yaw_rad = utility_functions.quaternion_to_euler(current_state[6:10])
            yaw_deg = math.degrees(yaw_rad)

            surge_sp, heave_sp, yaw_rate_sp, target_az = cf.OuterLoopControl(dx, dy, dz, yaw_deg, dt)
            x_cmd, z_cmd_offset, r_cmd = cf.InnerLoopControl(surge_sp, heave_sp, yaw_rate_sp, current_state, dt)

            print(
                f"yaw={yaw_deg:+6.1f}deg target_az={target_az:+6.1f}deg  "
                f"surge_sp={surge_sp:+.3f} heave_sp={heave_sp:+.3f} yaw_rate_sp={yaw_rate_sp:+.2f}  "
                f"-> x={x_cmd:+.1f} z_off={z_cmd_offset:+.1f} r={r_cmd:+.1f}"
            )
            mavlink_io.ManualControl(int(x_cmd), 0, int(-z_cmd_offset + 500), int(r_cmd))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[中断] Ctrl+Cを検知しました。安全停止します。")
    finally:
        safe_stop_and_disarm(mavlink_io, note="controlfunction実機デモ終了")
        import function
        function._imu_stream.stop()


if __name__ == "__main__":
    if hardware_flag_present():
        run_hardware_demo()
    else:
        sys.argv = [a for a in sys.argv if a != "--hardware"]
        try:
            unittest.main(verbosity=2)
        finally:
            import function
            function._imu_stream.stop()
