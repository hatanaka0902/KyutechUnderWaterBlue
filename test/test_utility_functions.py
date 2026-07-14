"""
Control/utility_functions.py の単体テスト。

対象関数(すべてクォータニオン/回転演算、Hamilton記法 q=[qw,qx,qy,qz]):
    - cross(v)                              : 外積行列(skew-symmetric matrix)
    - quaternion_to_euler(q)                : クォータニオン -> (roll,pitch,yaw)[rad]
    - quaternion_to_rotation_matrix(q)      : クォータニオン -> 回転行列(body->world)
    - quaternion_product(q1, q2)            : Hamilton積
    - rotation_vector_to_quaternion(v, ang) : 回転ベクトル+角度 -> クォータニオン(指数写像)
    - rodrigues(omega, angle)               : Rodriguesの回転公式 -> 回転行列

qekf.py の予測・更新式が全てこれらの関数に依存しているため、ここでの符号や
軸の誤りはQEKFの状態推定全体を誤らせる。純粋なnumpy計算のみで実機は不要。

実行方法:
    python test/test_utility_functions.py
"""

import math
import os
import sys
import unittest

import numpy as np

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import utility_functions as uf  # noqa: E402


class TestCross(unittest.TestCase):
    """cross(v): 3次元ベクトルの外積行列。QEKFの姿勢ヤコビアン(F_x)や
    DVLオフセット補正(update_dvl)で「ベクトル×ベクトル」の代わりに行列積として使われる。"""

    def test_cross_matrix_is_skew_symmetric(self):
        """目的: cross(v) が反対称行列(A = -A.T)であることを確認する。
        役割: 外積行列の数学的定義上の必須性質。ここが崩れるとヤコビアンの
        対称性を前提にした後続の共分散演算(P = F P F.T 等)が破綻する。
        結果が示すこと: cross(v) + cross(v).T が全てゼロ行列になる。"""
        v = np.array([1.0, 2.0, 3.0])
        M = uf.cross(v)
        np.testing.assert_allclose(M + M.T, np.zeros((3, 3)), atol=1e-12)

    def test_cross_matrix_times_v_equals_zero(self):
        """目的: cross(v) @ v == 0 (自分自身との外積は常にゼロベクトル)を確認する。
        役割: v×v=0 という外積の基本性質が行列表現でも保たれていることの検証。
        結果が示すこと: 任意のvについてcross(v)@vがゼロベクトルになる。"""
        v = np.array([1.0, -2.0, 0.5])
        np.testing.assert_allclose(uf.cross(v) @ v, np.zeros(3), atol=1e-12)

    def test_cross_matches_numpy_cross_product(self):
        """目的: cross(v)@w が np.cross(v,w) と一致することを確認する。
        役割: 外積行列が「外積を行列積として表現したもの」であることの直接検証。
        結果が示すこと: 任意の v,w で cross(v)@w == np.cross(v,w)。"""
        v = np.array([1.0, 0.0, 0.0])
        w = np.array([0.0, 1.0, 0.0])
        np.testing.assert_allclose(uf.cross(v) @ w, np.cross(v, w), atol=1e-12)


class TestQuaternionToEuler(unittest.TestCase):
    """quaternion_to_euler(q): controlfunction.run_control_loop() が現在yawを
    取得するために使う。姿勢デバッグ表示にも使われる基礎関数。"""

    def test_identity_quaternion_is_zero_euler(self):
        """目的: 単位クォータニオン[1,0,0,0](無回転)がroll=pitch=yaw=0になることを確認する。
        役割: 回転の基準(原点)が正しく定義されていることの確認。
        結果が示すこと: 3軸すべて0radになる。"""
        roll, pitch, yaw = uf.quaternion_to_euler([1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_yaw_90deg_quaternion_recovers_yaw(self):
        """目的: Z軸まわり+90度回転を表すクォータニオンから yaw=+90度 が復元されることを確認する。
        役割: AzimuthControl(目標方位)とQEKF推定yawを比較するcontrolfunction.pyの
        前提(同じ角度規約であること)を保証する。
        結果が示すこと: q=[cos45,0,0,sin45] のyawはπ/2[rad](=90度)。"""
        half = math.radians(90.0) / 2.0
        q = [math.cos(half), 0.0, 0.0, math.sin(half)]
        _, _, yaw = uf.quaternion_to_euler(q)
        self.assertAlmostEqual(math.degrees(yaw), 90.0, places=5)

    def test_roll_90deg_quaternion_recovers_roll(self):
        """目的: X軸まわり+90度回転(roll)がrollチャンネルにのみ現れることを確認する。
        役割: roll/pitch/yawの軸割り当て(ZYX順)がquaternion_to_rotation_matrixと
        整合していることの回帰保証。
        結果が示すこと: q=[cos45,sin45,0,0] はroll=90度、pitch/yawはほぼ0度。"""
        half = math.radians(90.0) / 2.0
        q = [math.cos(half), math.sin(half), 0.0, 0.0]
        roll, pitch, yaw = uf.quaternion_to_euler(q)
        self.assertAlmostEqual(math.degrees(roll), 90.0, places=5)
        self.assertAlmostEqual(math.degrees(pitch), 0.0, places=5)
        self.assertAlmostEqual(math.degrees(yaw), 0.0, places=5)


class TestQuaternionToRotationMatrix(unittest.TestCase):
    """quaternion_to_rotation_matrix(q): DVL速度のbody→NED変換、QEKFの積分・
    ヤコビアン計算など、qekf.py全体で最も多用される関数。"""

    def test_identity_quaternion_gives_identity_matrix(self):
        """目的: 無回転クォータニオンが単位行列を返すことを確認する。
        役割: 回転の基準が正しいことの最も基本的な確認。
        結果が示すこと: R([1,0,0,0]) == np.eye(3)。"""
        R = uf.quaternion_to_rotation_matrix([1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_rotation_matrix_is_orthonormal(self):
        """目的: 得られた行列が直交行列(R@R.T==I, det(R)==1)であることを確認する。
        役割: 回転行列としての妥当性(スケーリングや反転を含まないこと)の保証。
        速度ベクトルの変換で長さが変わってしまうと物理的に誤った速度になる。
        結果が示すこと: 任意の単位クォータニオンに対しR@R.T=I、det(R)=1。"""
        half = math.radians(37.0) / 2.0
        q = [math.cos(half), 0.1, 0.2, 0.3]
        q = np.asarray(q) / np.linalg.norm(q)
        R = uf.quaternion_to_rotation_matrix(q)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)

    def test_yaw_90deg_rotates_x_axis_to_y_axis(self):
        """目的: yaw+90度の回転行列が body の [1,0,0](前方)を world の [0,1,0]に写すことを確認する。
        役割: DVLのbody対地速度をNEDへ変換する際の軸の向き(どちらに回るか)の規約を固定する。
        controlfunction.get_body_frame_velocity や function.get_velocity_data の
        回転方向がこれと一致している前提でテストが書かれている。
        結果が示すこと: R(yaw=90) @ [1,0,0] ≈ [0,1,0]。"""
        half = math.radians(90.0) / 2.0
        q = [math.cos(half), 0.0, 0.0, math.sin(half)]
        R = uf.quaternion_to_rotation_matrix(q)
        np.testing.assert_allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)


class TestQuaternionProduct(unittest.TestCase):
    """quaternion_product(q1,q2): Hamilton積。QEKF.integrate()での姿勢積分
    (q ⊗ q_rot)、QEKF.inject()での誤差反映に使われる。"""

    def test_identity_is_neutral_element(self):
        """目的: 単位クォータニオンを掛けても元のクォータニオンが変化しないことを確認する。
        役割: 積分・注入処理で「今回の回転が無い(dtheta=0)」場合に状態が変化しない
        ことを保証する回帰テスト。
        結果が示すこと: q ⊗ [1,0,0,0] == q。"""
        q = np.array([0.5, 0.5, 0.5, 0.5])
        result = uf.quaternion_product(q, [1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(result.flatten(), q, atol=1e-12)

    def test_product_of_two_90deg_yaw_gives_180deg_yaw(self):
        """目的: yaw+90度を2回合成すると yaw+180度になることを確認する。
        役割: 姿勢を少しずつ積分していくQEKF.integrate()の正しさの基礎となる、
        回転の合成則(積の順序を含む)を検証する。
        結果が示すこと: 合成後のクォータニオンから復元したyawが180度(または-180度)になる。"""
        half90 = math.radians(90.0) / 2.0
        q90 = [math.cos(half90), 0.0, 0.0, math.sin(half90)]
        combined = uf.quaternion_product(q90, q90)
        _, _, yaw = uf.quaternion_to_euler(combined)
        self.assertAlmostEqual(abs(math.degrees(yaw)), 180.0, places=4)

    def test_output_shape_is_column_vector(self):
        """目的: 戻り値の形が (4,1) であることを確認する。
        役割: qekf.py内で self.x[6:10] = quaternion_product(...) のように
        (4,1)形状のスロットへ直接代入されるため、形状不一致はブロードキャスト
        エラーや意図しない値の破損につながる。
        結果が示すこと: quaternion_product(...).shape == (4,1)。"""
        result = uf.quaternion_product([1, 0, 0, 0], [1, 0, 0, 0])
        self.assertEqual(result.shape, (4, 1))


class TestRotationVectorToQuaternion(unittest.TestCase):
    """rotation_vector_to_quaternion(vector, angle): 指数写像。QEKF.integrate()で
    角速度(omega_m-omega_b)*dtを姿勢の増分クォータニオンに変換するために使う。"""

    def test_zero_vector_gives_identity_quaternion(self):
        """目的: ゼロベクトル(回転無し)が単位クォータニオンになることを確認する。
        役割: 角速度がゼロ(静止)の時に姿勢が変化しないことを保証する、ゼロ除算対策の分岐。
        結果が示すこと: rotation_vector_to_quaternion([0,0,0], angle)==[1,0,0,0]。"""
        q = uf.rotation_vector_to_quaternion([0.0, 0.0, 0.0], 1.23)
        np.testing.assert_allclose(q.flatten(), [1.0, 0.0, 0.0, 0.0], atol=1e-12)

    def test_output_is_unit_quaternion(self):
        """目的: 出力が常に単位クォータニオン(ノルム1)であることを確認する。
        役割: 姿勢を表すクォータニオンは単位でなければ回転行列変換等で不正な結果になる。
        結果が示すこと: 任意の軸・角度でノルムが1になる。"""
        q = uf.rotation_vector_to_quaternion([0.3, -0.1, 0.2], 0.7)
        self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=9)

    def test_z_axis_rotation_matches_known_quaternion(self):
        """目的: Z軸まわりの回転ベクトルから、既知の解析解(yaw用クォータニオン)と一致することを確認する。
        役割: 軸の割り当て(どの成分がどの軸に対応するか)がquaternion_to_rotation_matrix等の
        他関数と一貫していることの確認。
        結果が示すこと: vector=[0,0,1], angle=90度 のとき q ≈ [cos45,0,0,sin45]。"""
        angle = math.radians(90.0)
        q = uf.rotation_vector_to_quaternion([0.0, 0.0, 1.0], angle)
        expected = [math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]
        np.testing.assert_allclose(q.flatten(), expected, atol=1e-9)


class TestRodrigues(unittest.TestCase):
    """rodrigues(omega, angle): QEKF.predict()の姿勢ヤコビアンブロック(R(ω dt)^T相当)で使用。"""

    def test_zero_omega_gives_identity(self):
        """目的: 角速度ゼロの時に単位行列(回転なし)を返すことを確認する。
        役割: ゼロ除算を避けるための分岐が正しく単位行列にフォールバックすることの確認。
        結果が示すこと: rodrigues([0,0,0], angle) == I。"""
        R = uf.rodrigues([0.0, 0.0, 0.0], 0.5)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_rodrigues_matches_quaternion_rotation_for_same_axis_angle(self):
        """目的: rodriguesとquaternion_to_rotation_matrixが、同じ軸・角度に対して
        同じ回転行列を生成することを相互検証する(クロスチェック)。
        役割: QEKFの積分(クォータニオン経由)と予測ヤコビアン(rodrigues経由)が、
        同じ物理的回転に対して整合した数値を使っていることの保証。
        結果が示すこと: 軸[0,0,1]・角度30度について、両関数の結果がほぼ一致する。"""
        axis = np.array([0.0, 0.0, 1.0])
        angle = math.radians(30.0)
        R_rodrigues = uf.rodrigues(axis, angle)

        q = uf.rotation_vector_to_quaternion(axis, angle)
        R_quat = uf.quaternion_to_rotation_matrix(q)

        np.testing.assert_allclose(R_rodrigues, R_quat, atol=1e-9)

    def test_rodrigues_output_is_orthonormal(self):
        """目的: rodriguesの出力が有効な回転行列(直交・det=1)であることを確認する。
        役割: QEKFの共分散伝播(P_hat = F_x P F_x.T)にそのまま使われるブロックなので、
        直交性が崩れると共分散が非物理的な値(負の分散等)に発散する恐れがある。
        結果が示すこと: R@R.T==I、det(R)==1。"""
        R = uf.rodrigues([0.2, 0.4, -0.1], 1.1)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
