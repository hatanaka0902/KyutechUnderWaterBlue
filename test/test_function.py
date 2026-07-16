"""
Control/function.py のテスト。

対象関数(センサ取得〜状態推定):
    - create_qekf()/initialize()                 : QEKFの生成・初期化
    - get_acceleration_data()/get_gyro_data()      : IMU(ESP32/BNO08x)取得
    - parse_dvext()/get_velocity_data()            : DVL-75 ($DVEXT) 取得
    - get_orientation_measurement()                 : IMU姿勢クォータニオン取得
    - get_sensor_data()                             : IMU+DVLをQEKF入力形式にまとめる
    - state_estimation()                            : predict→integrate→DVL/姿勢/深度更新→
                                                        inject→resetの1周期を実行
    - get_last_dt()/get_last_imu_data()             : 直近dt/IMU生値の取得(PIDループ用)

function.py は import 時に ImuStream (ESP32用TCPサーバ、ポート5007) を起動するが、
BlueROV本体(MAVLink)には一切触れない。そのため:
    - 単体テスト(TestXxx)は get_latest_dvext() をモックし、_imu_stream の内部状態に
      直接ダミー値を注入することで、実際のESP32/DVL-75なしにロジックを検証する。
    - 実機読み取りデモ(run_hardware_demo)はモックを外し、実際に接続されたESP32/DVL-75
      から state_estimation() を一定時間実行して結果を表示する「読み取り専用」デモ。
      スラスタ等は一切動かさないため、mavlink_io.py 系のデモよりリスクは低いが、
      実機接続を前提とした長時間ループになるため確認プロンプトを設けている。

実行方法:
    python test/test_function.py             # 単体テスト(モック、実機不要)
    python test/test_function.py --hardware   # 実機読み取りデモ(要: ESP32/DVL-75接続)
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

import function  # noqa: E402 (importと同時にImuStreamサーバが起動する)
import utility_functions  # noqa: E402
from _hw_helpers import hardware_flag_present, confirm_hardware_action  # noqa: E402


def _nmea_checksum(payload: str) -> str:
    """NMEAペイロード(先頭$なし、*以降なし)のXORチェックサムを2桁hexで返す。"""
    csum = 0
    for ch in payload:
        csum ^= ord(ch)
    return f"{csum:02X}"


def make_dvext_sentence(
    *, dvl_lock=True, imu_cal="3333", roll_deg=0.0, pitch_deg=0.0, heading_deg=0.0,
    vel_up=0.0, altitude=1.0, vel_north=0.0, vel_east=0.0, elapsed_time=0.05,
    qw=1.0, qx=0.0, qy=0.0, qz=0.0,
):
    """parse_dvext() が期待するインデックス(f[1],f[3]..f[19])に合わせたテスト用$DVEXT文を生成する。"""
    lock = "T" if dvl_lock else "F"
    fields = [
        "DVEXT", lock, "", imu_cal, f"{roll_deg}", f"{pitch_deg}", f"{heading_deg}", "",
        f"{vel_up}", f"{altitude}", f"{vel_north}", f"{vel_east}", "", "",
        f"{elapsed_time}", f"{qw}", f"{qx}", f"{qy}", f"{qz}",
    ]
    payload = ",".join(fields)
    return f"${payload}*{_nmea_checksum(payload)}"


def _inject_imu_latest(quat, accel=None, gyro=None):
    """function._imu_stream の内部状態に直接ダミー値を書き込む(実ESP32接続の代替)。"""
    accel = np.zeros(3) if accel is None else np.asarray(accel, dtype=float)
    gyro = np.zeros(3) if gyro is None else np.asarray(gyro, dtype=float)
    with function._imu_stream._lock:
        function._imu_stream._latest = {
            "accel": accel, "gyro": gyro, "mag": np.zeros(3),
            "quat": np.asarray(quat, dtype=float),
        }
        function._imu_stream._latest_time = time.time()


def _clear_imu_latest():
    with function._imu_stream._lock:
        function._imu_stream._latest = None
        function._imu_stream._latest_time = 0.0


class TestParseDvext(unittest.TestCase):
    """parse_dvext(sentence): DVL-75の$DVEXT NMEA文字列パーサ。get_velocity_data()の入口。"""

    def test_valid_sentence_is_parsed_with_correct_fields(self):
        """目的: 正しいチェックサムを持つ$DVEXT文が、フィールドごとに正しい型・値で
        パースされることを確認する。
        役割: DVL-75からのシリアル/Ethernet生データをPython dictに変換する、
        速度取得の最初のステップ。ここでの取り違えはDVL速度更新全体を狂わせる。
        結果が示すこと: dvl_lock=True(文字列'T')、heading等の数値フィールドがfloatに
        変換され、指定した値と一致する。"""
        sentence = make_dvext_sentence(dvl_lock=True, heading_deg=90.0, vel_up=0.1,
                                        vel_north=1.0, vel_east=0.2, qw=1.0)
        parsed = function.parse_dvext(sentence)
        self.assertIsNotNone(parsed)
        self.assertTrue(parsed["dvl_lock"])
        self.assertAlmostEqual(parsed["heading_deg"], 90.0)
        self.assertAlmostEqual(parsed["vel_north"], 1.0)
        self.assertAlmostEqual(parsed["vel_east"], 0.2)
        self.assertAlmostEqual(parsed["vel_up"], 0.1)
        self.assertAlmostEqual(parsed["qw"], 1.0)

    def test_bad_checksum_returns_none(self):
        """目的: チェックサムが壊れた文字列(ノイズ/伝送誤り)がNoneとして拒否されることを確認する。
        役割: 誤ったデータをそのまま速度推定に使ってしまう(サイレントな破損データ受理)
        事故を防ぐ、DVL-75プロトコルの整合性チェック。
        結果が示すこと: チェックサムを1文字だけ壊した文字列はparse_dvext()でNoneになる。"""
        sentence = make_dvext_sentence(vel_north=1.0)
        bad = sentence[:-2] + ("00" if sentence[-2:] != "00" else "FF")
        self.assertIsNone(function.parse_dvext(bad))

    def test_non_dvext_sentence_returns_none(self):
        """目的: $DVEXT以外のNMEA文(例: $GPGGA)がNoneとして無視されることを確認する。
        役割: DVL-75以外の機器から紛れ込んだ文字列でクラッシュ・誤動作しないことの確認。
        結果が示すこと: '$GPGGA,...'を渡すとparse_dvext()はNoneを返す。"""
        self.assertIsNone(function.parse_dvext("$GPGGA,123456,,,,,0,00,,,M,,M,,*00"))


class TestGetVelocityData(unittest.TestCase):
    """get_velocity_data(): DVL対地速度(NED)を機体座標系(body)に変換して返す。
    get_latest_dvext()は実機シリアル/Ethernet受信のスタブ(NotImplementedError)なので、
    ここでは patch して既知のdvext dictを差し込む。"""

    def test_identity_attitude_body_velocity_equals_ned_velocity(self):
        """目的: 姿勢が単位クォータニオン(無回転)の時、body速度がNED速度とそのまま
        一致することを確認する(回転が無回転なので変換前後で値が変わらないケース)。
        役割: qekf.QEKF.update_dvl()に渡す前段のbody→NED方向の変換が正しいことの
        基礎確認。ここでの符号ミスは全てのDVL速度更新に影響する。
        結果が示すこと: vel_north=1.0,vel_east=0.5,vel_up=0.2(上向き)を与えると、
        body速度は[1.0, 0.5, -0.2](NED z正=下方向なので上向き0.2はz=-0.2)になる。"""
        dvext = function.parse_dvext(make_dvext_sentence(
            dvl_lock=True, vel_north=1.0, vel_east=0.5, vel_up=0.2, qw=1.0))
        with patch.object(function, "get_latest_dvext", return_value=dvext):
            v_body = function.get_velocity_data()
        self.assertIsNotNone(v_body)
        np.testing.assert_allclose(v_body, [1.0, 0.5, -0.2], atol=1e-9)

    def test_yaw_90deg_rotates_velocity_into_body_frame(self):
        """目的: yaw+90度の姿勢の時、NEDのx方向(北)速度がbody座標系では-y方向(右方向の逆)
        に変換されることを確認する。
        役割: 実際にROVが旋回している状態でも正しくbody速度が得られる(=InnerLoopControlの
        surge/heave速度PIDが誤った速度で追従しない)ことの確認。
        結果が示すこと: heading分の回転を持つ観測から、world=[1,0,0]がbody≈[0,-1,0]になる。"""
        yaw = math.radians(90.0)
        qw, qz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        dvext = function.parse_dvext(make_dvext_sentence(
            dvl_lock=True, vel_north=1.0, vel_east=0.0, vel_up=0.0, qw=qw, qz=qz))
        with patch.object(function, "get_latest_dvext", return_value=dvext):
            v_body = function.get_velocity_data()
        self.assertIsNotNone(v_body)
        np.testing.assert_allclose(v_body, [0.0, -1.0, 0.0], atol=1e-9)

    def test_dvl_unlocked_returns_none(self):
        """目的: DVLがロックを失っている(dvl_lock=False)場合、速度としてNoneが返る
        ことを確認する。
        役割: ロスト中の不正な速度データをQEKFに投入してしまう(誤った更新)事故を
        防ぐガード。function.get_sensor_data()はこのNoneを見てDVL更新をスキップする。
        結果が示すこと: dvl_lock=Falseの$DVEXTからはget_velocity_data()がNone。"""
        dvext = function.parse_dvext(make_dvext_sentence(dvl_lock=False, vel_north=1.0))
        with patch.object(function, "get_latest_dvext", return_value=dvext):
            self.assertIsNone(function.get_velocity_data())

    def test_no_dvext_available_returns_none(self):
        """目的: get_latest_dvext()自体がNone(受信途絶)を返す場合もNoneが伝播することを確認する。
        役割: DVL-75との通信断が起きた時に、古い/存在しないデータで処理を続けない
        ことの確認。
        結果が示すこと: get_latest_dvext()がNoneならget_velocity_data()もNone。"""
        with patch.object(function, "get_latest_dvext", return_value=None):
            self.assertIsNone(function.get_velocity_data())


class TestGetOrientationMeasurement(unittest.TestCase):
    """get_orientation_measurement(): BNO08x(ESP32経由)の姿勢クォータニオンを
    QEKF.update_orientation()用の(4,1)配列で返す。"""

    def tearDown(self):
        _clear_imu_latest()

    def test_returns_quaternion_reshaped_to_4x1(self):
        """目的: ImuStreamが保持する'quat'(shape(4,))が(4,1)にreshapeされて
        返ることを確認する。
        役割: qekf.QEKF.update_orientation()は(4,1)前提の行列演算をしているため、
        形状不一致はブロードキャストエラーに直結する。
        結果が示すこと: 注入したクォータニオンと同じ値が(4,1)形状で返る。"""
        q = np.array([0.7071, 0.0, 0.0, 0.7071])
        _inject_imu_latest(q)
        meas = function.get_orientation_measurement()
        self.assertIsNotNone(meas)
        self.assertEqual(meas.shape, (4, 1))
        np.testing.assert_allclose(meas.flatten(), q, atol=1e-6)

    def test_stale_imu_data_returns_none(self):
        """目的: ImuStreamのデータが古くなった(stale_timeout超過)場合、Noneが
        返ることを確認する。
        役割: ESP32接続が切れた際に古い姿勢で誤った補正をQEKFに与えないための
        ガードの確認。
        結果が示すこと: _latest_timeを1秒前に設定するとget_orientation_measurement()はNone。"""
        _inject_imu_latest([1.0, 0.0, 0.0, 0.0])
        with function._imu_stream._lock:
            function._imu_stream._latest_time = time.time() - 1.0
        self.assertIsNone(function.get_orientation_measurement())

    def test_no_imu_data_returns_none(self):
        """目的: 一度もIMUデータを受信していない場合にNoneが返ることを確認する。
        役割: 起動直後、ESP32未接続の状態で例外を出さずに安全にNoneを返すことの確認。
        結果が示すこと: _latestがNoneの状態でget_orientation_measurement()はNone。"""
        _clear_imu_latest()
        self.assertIsNone(function.get_orientation_measurement())


class TestGetSensorData(unittest.TestCase):
    """get_sensor_data(): IMU(加速度・角速度)とDVL速度をQEKF入力形式にまとめる。"""

    def test_combines_accel_gyro_into_2x3_array(self):
        """目的: get_acceleration_data()とget_gyro_data()の結果が(2,3)のimu_dataに
        正しく積み上げられることを確認する(1行目=加速度、2行目=角速度)。
        役割: QEKF.integrate/predict は u[0]を加速度、u[1]を角速度として使うため、
        行の順序を誤ると加速度と角速度が入れ替わり、状態推定が完全に破綻する。
        結果が示すこと: imu_data[0]==accel, imu_data[1]==gyro、shape==(2,3)。"""
        with patch.object(function, "get_acceleration_data", return_value=[1.0, 2.0, 3.0]), \
             patch.object(function, "get_gyro_data", return_value=[0.1, 0.2, 0.3]), \
             patch.object(function, "get_velocity_data", return_value=None):
            imu_data, (v, cov) = function.get_sensor_data()
        self.assertEqual(imu_data.shape, (2, 3))
        np.testing.assert_allclose(imu_data[0], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(imu_data[1], [0.1, 0.2, 0.3])
        self.assertIsNone(v)
        self.assertIsNone(cov)

    def test_dvl_available_returns_3x1_velocity_and_covariance(self):
        """目的: DVL速度が取得できる場合、(3,1)形状のv_bodyと(3,3)の共分散が
        返ることを確認する。
        役割: qekf.QEKF.update_dvl(dvl_measurement_raw_body, dvl_covariance_body)の
        期待する形状と一致していることの確認。
        結果が示すこと: get_velocity_data()が[0.1,0.2,0.3]を返すと、
        v_body.shape==(3,1)でその値が入り、covはdiag([0.02,0.02,0.03])**2に一致する。"""
        with patch.object(function, "get_acceleration_data", return_value=[0.0, 0.0, 9.81]), \
             patch.object(function, "get_gyro_data", return_value=[0.0, 0.0, 0.0]), \
             patch.object(function, "get_velocity_data", return_value=[0.1, 0.2, 0.3]):
            imu_data, (v, cov) = function.get_sensor_data()
        self.assertEqual(v.shape, (3, 1))
        np.testing.assert_allclose(v.flatten(), [0.1, 0.2, 0.3])
        np.testing.assert_allclose(cov, np.diag([0.02, 0.02, 0.03]) ** 2)


class TestStateEstimation(unittest.TestCase):
    """state_estimation(): predict→integrate→DVL/姿勢/深度更新→inject→resetの1周期。
    function.py全体のセンサ融合の主入口であり、controlfunction.run_control_loop()が
    毎周期呼ぶ。"""

    def setUp(self):
        # 各テストをQEKF・_last_timeの初期状態から始める(他テストの残留状態を排除)
        function.initialize()
        _clear_imu_latest()

    def _patch_sensors(self, accel, gyro, velocity=None, depth=None):
        return (
            patch.object(function, "get_acceleration_data", return_value=accel),
            patch.object(function, "get_gyro_data", return_value=gyro),
            patch.object(function, "get_velocity_data", return_value=velocity),
            patch.object(function, "get_depth_data", return_value=depth),
        )

    def test_first_call_skips_predict_and_returns_initial_state(self):
        """目的: 一度もstate_estimation()を呼んでいない状態(_last_time is None)での
        初回呼び出しが、predict/integrateをスキップして初期状態をそのまま返すことを確認する。
        役割: 起動直後、dtが未計測(0や負の異常値になりうる)状態でIMU積分してしまう
        事故を防ぐための、function.pyのコメントに明記された分岐。
        結果が示すこと: 初回呼び出しの戻り値が create_qekf() 直後の初期状態と一致し、
        get_last_dt()は0.0になる。"""
        p_accel, p_gyro, p_vel, p_depth = self._patch_sensors(
            accel=[0.0, 0.0, 9.81], gyro=[0.0, 0.0, 0.0])
        with p_accel, p_gyro, p_vel, p_depth:
            state = function.state_estimation()
        np.testing.assert_allclose(state, function._qekf.get_state())
        self.assertEqual(function.get_last_dt(), 0.0)

    def test_second_call_computes_positive_dt_and_updates_last_imu_data(self):
        """目的: 2回目以降の呼び出しで、実際の経過時間に基づく正のdtが計算され、
        get_last_dt()/get_last_imu_data()がその周期の値に更新されることを確認する。
        役割: controlfunction.OuterLoopControl/InnerLoopControlはget_last_dt()を
        「state_estimation()と同じdt」として再利用する設計になっており、ここが
        ズレると外側/内側ループが異なる時間軸でPID計算してしまう。
        結果が示すこと: 1回目呼び出し後に少し待って2回目を呼ぶと、get_last_dt()>0、
        get_last_imu_data()が2回目に与えた加速度・角速度と一致する。"""
        p_accel, p_gyro, p_vel, p_depth = self._patch_sensors(
            accel=[0.0, 0.0, 9.81], gyro=[0.0, 0.0, 0.0])
        with p_accel, p_gyro, p_vel, p_depth:
            function.state_estimation()
            time.sleep(0.05)
            function.state_estimation()
        self.assertGreater(function.get_last_dt(), 0.0)
        np.testing.assert_allclose(function.get_last_imu_data()[0], [0.0, 0.0, 9.81])
        np.testing.assert_allclose(function.get_last_imu_data()[1], [0.0, 0.0, 0.0])

    def test_stationary_run_keeps_position_near_zero(self):
        """目的: 静止相当の入力(重力のみの加速度、角速度0、DVL速度0)を数周期与えても、
        推定位置が原点付近から大きく発散しないことを確認する。
        役割: IMU単独では小さなドリフトは避けられないが、DVL速度0による毎周期の
        update_dvlが位置の暴走を防いでいることの、結合動作としての健全性確認
        (qekf.py単体テストでは見えない、function.py経由の統合的な振る舞い)。
        結果が示すこと: 10周期(合計約0.1秒分)実行後も位置の各成分が小さい範囲に収まる。"""
        p_accel, p_gyro, p_vel, p_depth = self._patch_sensors(
            accel=[0.0, 0.0, 9.81], gyro=[0.0, 0.0, 0.0], velocity=[0.0, 0.0, 0.0])
        with p_accel, p_gyro, p_vel, p_depth:
            for _ in range(10):
                state = function.state_estimation()
                time.sleep(0.01)
        np.testing.assert_allclose(state[0:3], [0.0, 0.0, 0.0], atol=0.05)


# ---------------------------------------------------------------------------
# 実機読み取りデモ: 実際のESP32(IMU)/DVL-75に接続した状態でstate_estimation()を
# 繰り返し呼び、推定された位置・速度・姿勢を表示する。スラスタ等は一切動かさない
# (読み取り専用)ため、mavlink_io.pyの実機デモよりリスクは低い。
# ---------------------------------------------------------------------------
def run_hardware_demo(duration_sec: float = 10.0, interval_sec: float = 0.2):
    print("=== function.py 実機読み取りデモ(スラスタは動かしません) ===")
    print("ESP32(IMU_CALIB_AND_REC.ino)がこのPCの5007番ポートに接続してくるのを待ちます。")
    print("DVL-75は get_latest_dvext() が未実装(NotImplementedError)のため、この関数を")
    print("実装済みの場合のみDVL更新が有効になります(未実装ならDVL部分は自動的にスキップされます)。")

    if not confirm_hardware_action(f"state_estimation()を{duration_sec}秒間、{interval_sec}秒間隔で実行します。"):
        print("キャンセルしました。")
        return

    function.initialize()

    def _safe_get_depth():
        try:
            return function.get_depth_data()
        except NotImplementedError:
            return None

    def _safe_get_velocity():
        try:
            return function.get_velocity_data()
        except NotImplementedError:
            return None

    with patch.object(function, "get_depth_data", side_effect=_safe_get_depth), \
         patch.object(function, "get_velocity_data", side_effect=_safe_get_velocity):
        start = time.time()
        while time.time() - start < duration_sec:
            try:
                state = function.state_estimation()
            except RuntimeError as e:
                print(f"[警告] {e}")
                time.sleep(interval_sec)
                continue
            pos = state[0:3]
            vel = state[3:6]
            roll, pitch, yaw = utility_functions.quaternion_to_euler(state[6:10])
            print(
                f"pos(NED)=[{pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}]m  "
                f"vel(NED)=[{vel[0]:+.3f},{vel[1]:+.3f},{vel[2]:+.3f}]m/s  "
                f"rpy=[{math.degrees(roll):+.1f},{math.degrees(pitch):+.1f},{math.degrees(yaw):+.1f}]deg  "
                f"dt={function.get_last_dt():.3f}s"
            )
            time.sleep(interval_sec)

    print("=== 実機読み取りデモ終了 ===")


if __name__ == "__main__":
    if hardware_flag_present():
        run_hardware_demo()
    else:
        sys.argv = [a for a in sys.argv if a != "--hardware"]
        try:
            unittest.main(verbosity=2)
        finally:
            function._imu_stream.stop()
