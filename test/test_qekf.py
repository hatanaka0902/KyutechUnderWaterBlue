"""
Control/qekf.py の単体テスト。

対象クラス: QEKF (誤差状態クォータニオンEKF本体、Joan Sola arXiv:1711.02508 準拠)
    - integrate(u, dt)              : IMU入力を公称状態(位置・速度・姿勢)に積分
    - predict(u, dt)                 : 誤差状態の共分散を伝播
    - update_orientation(q)          : 姿勢観測による補正
    - update_depth(depth)            : 深度観測による補正
    - update_dvl(v, cov)             : DVL速度観測による補正
    - inject()                       : 誤差状態を公称状態へ反映
    - reset()                        : 誤差状態・共分散をリセット
    - get_state/get_position/get_velocity/get_quaternion/get_covariance: アクセサ

qekf.py は utility_functions.py にのみ依存する純粋なnumpy計算であり、実機は不要。
function.state_estimation() が predict→integrate→update→inject→reset の順で
毎周期これらを呼ぶため、各メソッド単独の振る舞いをここで固定しておく。

実行方法:
    python test/test_qekf.py
"""

import os
import sys
import unittest

import numpy as np

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from qekf import QEKF  # noqa: E402


def make_qekf(x_0=None):
    """テスト用にQEKFを組み立てる。オフセットは全て0、ノイズは適度な値に固定し、
    テストごとの再現性(乱数を使わない決定的な挙動)を優先する。"""
    default_x0 = [
        0.0, 0.0, 0.0,        # position (NED)
        0.0, 0.0, 0.0,        # velocity (NED)
        1.0, 0.0, 0.0, 0.0,   # quaternion [qw,qx,qy,qz] (無回転)
        0.0, 0.0, 0.0,        # accel bias
        0.0, 0.0, 0.0,        # gyro bias
        0.0, 0.0, 9.81,       # gravity (NED, z下向き)
    ]
    return QEKF(
        x_0=x_0 if x_0 is not None else default_x0,
        dx_0=np.zeros((18, 1)),
        P_0=np.eye(18) * 0.1,
        std_a=0.1, std_gyro=0.01, std_dvl=0.02, std_depth=0.05, std_orientation=0.01,
        std_a_bias=1e-4, std_gyro_bias=1e-5,
        dvl_offset=np.zeros(3), barometer_offset=np.zeros(3), imu_offset=np.zeros(3),
    )


class TestIntegrate(unittest.TestCase):
    """integrate(u,dt): IMUの生データ(比力・角速度)から位置・速度・姿勢を予測積分する。
    function.state_estimation()が毎周期呼ぶ、状態推定の中核。"""

    def test_stationary_imu_keeps_velocity_and_position_unchanged(self):
        """目的: 静止状態(加速度計が重力9.81だけを読む、角速度0)ではIMU積分によって
        速度・位置が変化しないことを確認する。
        役割: 「重力を差し引いた比力(a - a_bias - g)」が正しく0になり、余計な
        見かけの加速度で位置がドリフトしないことの検証。センサ融合前のIMU単独
        推定における最重要ケース。
        結果が示すこと: a_m=[0,0,9.81](NED, 重力と一致)・omega_m=[0,0,0]なら、
        1ステップ後も position, velocity は初期値からほぼ変化しない。"""
        qekf = make_qekf()
        u = np.array([[0.0, 0.0, 9.81], [0.0, 0.0, 0.0]])
        qekf.integrate(u, dt=0.1)
        np.testing.assert_allclose(qekf.get_position(), [0.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(qekf.get_velocity(), [0.0, 0.0, 0.0], atol=1e-9)

    def test_net_upward_acceleration_increases_velocity_upward(self):
        """目的: 重力を上回る比力(上向きの合力)を与えた時、NED z方向の速度が
        負に(=上方向に)増加することを確認する。
        役割: 座標系の符号規約(NED: z正=下)がintegrate()内で正しく扱われている
        ことの確認。符号が反転していると、ROVが浮上しようとして逆に沈む重大な
        バグになる。
        結果が示すこと: a_m=[0,0,8.81](重力より1 m/s^2小さい=上向き合力)を
        与えると、速度のz成分が負(上向き)になる。"""
        qekf = make_qekf()
        u = np.array([[0.0, 0.0, 8.81], [0.0, 0.0, 0.0]])
        qekf.integrate(u, dt=0.1)
        self.assertLess(qekf.get_velocity()[2], 0.0)

    def test_constant_yaw_rate_rotates_quaternion(self):
        """目的: 一定のz軸角速度を積分すると、姿勢クォータニオンから復元されるyawが
        時間×角速度だけ進むことを確認する。
        役割: ジャイロ角速度から姿勢を推定する部分の基本動作確認。yawレートPID
        (controlfunction)のフィードバックが正しいyaw推定に依存しているため重要。
        結果が示すこと: omega_m=[0,0,1.0] rad/s を dt=0.1 で10ステップ積分すると
        yaw ≈ 1.0 rad(角速度×経過時間)に近づく。"""
        qekf = make_qekf()
        u = np.array([[0.0, 0.0, 9.81], [0.0, 0.0, 1.0]])
        for _ in range(10):
            qekf.integrate(u, dt=0.1)
        import utility_functions as uf
        _, _, yaw = uf.quaternion_to_euler(qekf.get_quaternion())
        self.assertAlmostEqual(yaw, 1.0, places=2)


class TestPredict(unittest.TestCase):
    """predict(u,dt): 誤差状態の共分散Pを伝播する(状態そのものは変えない)。"""

    def test_predict_does_not_change_nominal_state(self):
        """目的: predict()は公称状態(x)を変更せず、共分散(P)のみを更新することを確認する。
        役割: qekf.pyの設計上、状態の伝播はintegrate()の責務であり、predict()は
        誤差の不確かさだけを扱う分離になっている。責務混在によるバグを防ぐ回帰テスト。
        結果が示すこと: predict()呼び出し前後で get_state() の値が変化しない。"""
        qekf = make_qekf()
        before = qekf.get_state().copy()
        u = np.array([[0.1, 0.0, 9.81], [0.01, 0.0, 0.0]])
        qekf.predict(u, dt=0.1)
        after = qekf.get_state()
        np.testing.assert_allclose(before, after, atol=1e-12)

    def test_predict_increases_uncertainty_over_time(self):
        """目的: predict()を繰り返すほど共分散(不確かさ)が大きくなっていくことを確認する。
        役割: プロセスノイズ(std_a, std_gyro等)が正しく共分散に加算されていることの
        確認。ここが機能しないと、更新(update_dvl等)が過信した推定を出し続け、
        フィルタが観測を無視するようになる(発散/過信バグ)。
        結果が示すこと: 1回のpredict後よりも10回のpredict後の方が、共分散行列の
        トレース(対角成分の総和)が大きい。"""
        qekf = make_qekf()
        u = np.array([[0.0, 0.0, 9.81], [0.0, 0.0, 0.0]])
        qekf.predict(u, dt=0.1)
        trace_after_1 = np.trace(qekf.get_covariance())

        qekf2 = make_qekf()
        for _ in range(10):
            qekf2.predict(u, dt=0.1)
        trace_after_10 = np.trace(qekf2.get_covariance())

        self.assertGreater(trace_after_10, trace_after_1)


class TestUpdateDvl(unittest.TestCase):
    """update_dvl(v_body, cov_body): DVL対地速度による誤差状態の補正。
    function.state_estimation()がDVLロック時に毎周期呼ぶ主要な更新経路。"""

    def test_matching_measurement_gives_near_zero_correction(self):
        """目的: DVL観測値が現在の推定速度と(NED変換後)一致する場合、補正量(dx)が
        ほぼゼロになることを確認する。
        役割: 「観測と推定が一致していれば何も直さない」というカルマンフィルタの
        基本的な整合性の確認。ここが崩れると、正しい観測が来ても無駄に状態が動く。
        結果が示すこと: 現在推定速度と全く同じbody速度をupdate_dvlに与えると、
        dx[3:5](水平速度の誤差成分)がほぼ0になる。"""
        qekf = make_qekf()
        v_body = np.zeros((3, 1))  # 姿勢が単位クォータニオンなのでNEDでも[0,0,0]
        cov_body = np.diag([0.02, 0.02, 0.03]) ** 2
        qekf.update_dvl(v_body, cov_body)
        dx, _ = qekf.dx, qekf.P
        np.testing.assert_allclose(dx[3:5].flatten(), [0.0, 0.0], atol=1e-9)

    def test_mismatched_measurement_pulls_state_toward_measurement(self):
        """目的: 推定速度と異なるDVL観測を与えた時、inject後の速度推定が観測値に
        近づく(0から離れて観測方向に動く)ことを確認する。
        役割: フィルタが実際に観測を信頼して状態を補正する、更新ステップの
        本来の目的(推定値の追従)が機能していることの確認。
        結果が示すこと: 観測=[1.0, 0.0, 0.0] m/sを与えてupdate_dvl→inject→reset
        すると、更新後のvxが0より大きくなる(観測方向に補正される)。"""
        qekf = make_qekf()
        v_body = np.array([[1.0], [0.0], [0.0]])
        cov_body = np.diag([0.02, 0.02, 0.03]) ** 2
        qekf.update_dvl(v_body, cov_body)
        qekf.inject()
        qekf.reset()
        self.assertGreater(qekf.get_velocity()[0], 0.0)


class TestUpdateOrientation(unittest.TestCase):
    """update_orientation(q): BNO08x等の姿勢観測による誤差状態の補正。"""

    def test_matching_quaternion_gives_near_zero_correction(self):
        """目的: 観測クォータニオンが現在の推定姿勢と一致する場合、補正量(dx)の
        姿勢成分(dx[6:9])がほぼゼロになることを確認する。
        役割: update_dvlと同様、観測と推定が一致していれば補正しないという
        整合性の確認。
        結果が示すこと: 現在の推定と同じ[1,0,0,0]をupdate_orientationに渡すと、
        姿勢誤差成分がほぼ0になる。"""
        qekf = make_qekf()
        qekf.update_orientation(np.array([[1.0], [0.0], [0.0], [0.0]]))
        np.testing.assert_allclose(qekf.dx[6:9].flatten(), [0.0, 0.0, 0.0], atol=1e-6)

    def test_mismatched_quaternion_produces_nonzero_correction(self):
        """目的: 現在推定と異なる姿勢観測を与えた時、姿勢の誤差状態成分が
        非ゼロになることを確認する(=補正が発生する)。
        役割: 実際にBNO08xの姿勢観測がQEKFの姿勢推定を補正できることの確認。
        結果が示すこと: yaw90度回転を表す観測クォータニオンを与えると、
        dx[6:9]のいずれかの成分がほぼゼロでなくなる。"""
        qekf = make_qekf()
        import math
        half = math.radians(90.0) / 2.0
        q_obs = np.array([[math.cos(half)], [0.0], [0.0], [math.sin(half)]])
        qekf.update_orientation(q_obs)
        self.assertGreater(np.linalg.norm(qekf.dx[6:9]), 1e-6)


class TestUpdateDepth(unittest.TestCase):
    """update_depth(depth): 気圧センサによる深度(z)の誤差状態補正。"""

    def test_matching_depth_gives_near_zero_correction(self):
        """目的: 観測深度が現在の推定位置z(=0)と一致する場合、補正量がほぼゼロに
        なることを確認する。
        役割: 深度観測が推定と一致している時に不要な補正が入らないことの確認。
        結果が示すこと: depth=0.0を与えるとdx[2](深度方向誤差)がほぼ0になる。"""
        qekf = make_qekf()
        qekf.update_depth(0.0)
        self.assertAlmostEqual(float(qekf.dx[2, 0]), 0.0, places=6)

    def test_deeper_measurement_produces_positive_position_correction(self):
        """目的: 現在推定より深い(NED z正方向)観測を与えた時、位置zの誤差状態
        補正が正の値になることを確認する。
        役割: 気圧センサが実際に深度推定を補正できる方向性(符号)の確認。
        結果が示すこと: depth=2.0(現在推定z=0より深い)を与えると、
        dx[2](z方向誤差)が正になる。"""
        qekf = make_qekf()
        qekf.update_depth(2.0)
        self.assertGreater(float(qekf.dx[2, 0]), 0.0)


class TestInject(unittest.TestCase):
    """inject(): 誤差状態(dx)を公称状態(x)へ反映する。update_*とreset()の間で必ず呼ばれる。"""

    def test_inject_adds_position_and_velocity_error(self):
        """目的: dxの位置・速度成分がそのままxに加算されることを確認する。
        役割: 誤差状態カルマンフィルタの中心的な操作(公称+誤差=真の推定)が
        正しく実装されていることの確認。
        結果が示すこと: dx[0:3]=[1,2,3], dx[3:6]=[0.1,0.2,0.3]を設定してinject()
        すると、position/velocityにそれぞれその値が加算される。"""
        qekf = make_qekf()
        qekf.dx[0:3] = np.array([[1.0], [2.0], [3.0]])
        qekf.dx[3:6] = np.array([[0.1], [0.2], [0.3]])
        qekf.inject()
        np.testing.assert_allclose(qekf.get_position(), [1.0, 2.0, 3.0], atol=1e-9)
        np.testing.assert_allclose(qekf.get_velocity(), [0.1, 0.2, 0.3], atol=1e-9)

    def test_inject_keeps_quaternion_unit_norm(self):
        """目的: 姿勢誤差(dx[6:9])を注入した後も、クォータニオンが単位ノルムを
        保つことを確認する。
        役割: inject()が誤差角をクォータニオン積(quaternion_product)で正しく
        合成しており、単純な加算による正規化崩れが起きていないことの確認。
        結果が示すこと: 小さな姿勢誤差を注入した後もget_quaternion()のノルムが1。"""
        qekf = make_qekf()
        qekf.dx[6:9] = np.array([[0.05], [0.0], [0.0]])
        qekf.inject()
        self.assertAlmostEqual(float(np.linalg.norm(qekf.get_quaternion())), 1.0, places=6)


class TestReset(unittest.TestCase):
    """reset(): 誤差状態(dx)をゼロに戻し、共分散を再パラメータ化する(注入後に毎周期呼ぶ)。"""

    def test_reset_zeros_error_state(self):
        """目的: reset()呼び出し後にdxが完全にゼロベクトルになることを確認する。
        役割: inject()で公称状態に反映した誤差を、次周期のために必ずクリアする
        という誤差状態カルマンフィルタの手順(predict-integrate-update-inject-reset)
        の一貫性を保証する。
        結果が示すこと: 非ゼロのdxを設定してreset()すると、全要素が0になる。"""
        qekf = make_qekf()
        qekf.dx = np.ones((18, 1))
        qekf.reset()
        np.testing.assert_allclose(qekf.dx.flatten(), np.zeros(18), atol=1e-12)

    def test_reset_with_zero_dtheta_keeps_covariance_unchanged(self):
        """目的: 姿勢誤差(dtheta=dx[6:9])がゼロの時、reset()による共分散の
        変換行列GがほぼI(単位行列)となり、共分散が変化しないことを確認する。
        役割: reset()の共分散変換(G@P@G.T)が、姿勢誤差が無い通常ケースで
        余計な変形を加えないことの確認。
        結果が示すこと: dtheta=0でreset()した前後でPがほぼ変化しない。"""
        qekf = make_qekf()
        qekf.dx = np.zeros((18, 1))
        before = qekf.get_covariance().copy()
        qekf.reset()
        after = qekf.get_covariance()
        np.testing.assert_allclose(before, after, atol=1e-9)


class TestAccessors(unittest.TestCase):
    """get_state/get_position/get_velocity/get_quaternion/get_covariance: 状態取得用アクセサ。
    function.py/controlfunction.pyはQEKFの内部(self.x等)に直接触らずこれらを使う想定。"""

    def test_get_state_returns_flat_19_vector(self):
        """目的: get_state()が19要素の1次元配列を返すことを確認する。
        役割: function.state_estimation()の戻り値の形をcontrolfunction.py側が
        current_state[2]やcurrent_state[6:10]のようにインデックスで前提にしている
        ため、形状の逸脱は誘導則全体を壊す。
        結果が示すこと: get_state().shape == (19,)。"""
        qekf = make_qekf()
        state = qekf.get_state()
        self.assertEqual(state.shape, (19,))

    def test_accessors_are_consistent_with_get_state_slices(self):
        """目的: get_position/get_velocity/get_quaternionが、get_state()の対応する
        スライスと一致することを確認する。
        役割: 複数のアクセサ経路(個別メソッド vs 生の状態ベクトル)で値がズレて
        いないことの確認(実装の重複によるバグを防ぐ回帰テスト)。
        結果が示すこと: get_state()[0:3]==get_position() 等が全て成立する。"""
        qekf = make_qekf()
        state = qekf.get_state()
        np.testing.assert_allclose(state[0:3], qekf.get_position())
        np.testing.assert_allclose(state[3:6], qekf.get_velocity())
        np.testing.assert_allclose(state[6:10], qekf.get_quaternion())

    def test_get_covariance_returns_copy_not_reference(self):
        """目的: get_covariance()が内部Pのコピーを返し、呼び出し元での変更が
        QEKF内部状態に影響しないことを確認する。
        役割: ログ出力等でget_covariance()の戻り値を書き換えてしまっても、
        フィルタ本体の共分散が意図せず汚染されないことの安全性確認。
        結果が示すこと: 戻り値を変更してもqekf.Pは変化しない。"""
        qekf = make_qekf()
        cov = qekf.get_covariance()
        cov[0, 0] = 999.0
        self.assertNotAlmostEqual(qekf.get_covariance()[0, 0], 999.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
