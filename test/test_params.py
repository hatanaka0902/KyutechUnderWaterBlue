"""
Control/params.py の健全性テスト。

params.py は定数のみを定義するファイルで関数を持たない。ここでは値そのものの
形式・範囲(例えば output_limits の下限<=上限、MAX_DTが正であること等)が
崩れていないかを確認する。値のチューニング自体(ゲインの良し悪し)は対象外。

実機(BlueROV)は不要。実行方法:
    python test/test_params.py
"""

import os
import re
import sys
import unittest

import numpy as np

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import params  # noqa: E402


class TestMavlinkConnectionString(unittest.TestCase):
    """MAVLINK_CONNECTION_STRING: mavlink_io.py が起動時に接続する接続文字列。"""

    def test_connection_string_matches_pymavlink_udp_format(self):
        """目的: 接続文字列が pymavlink の期待する 'udpin:host:port' 形式であることを確認する。
        役割: ここが誤っていると mavlink_io.py の import 時点(mavutil.mavlink_connection)
        で接続確立に失敗し、機体と全く通信できなくなる。
        結果が示すこと: 文字列が 'udpin:<IPv4>:<port>' の形式に一致する。"""
        pattern = r"^udpin:\d{1,3}(\.\d{1,3}){3}:\d+$"
        self.assertRegex(params.MAVLINK_CONNECTION_STRING, pattern)


class TestQekfInitialState(unittest.TestCase):
    """QEKF初期パラメータ(_X0, _DX0, _P0等)。function.create_qekf()にそのまま渡る。"""

    def test_x0_has_19_elements(self):
        """目的: 公称状態の初期値が19要素(位置3+速度3+quat4+accelbias3+gyrobias3+g3)であることを確認する。
        役割: QEKFクラスは19要素であることを前提に固定インデックス(self.x[6:10]等)で
        アクセスするため、要素数がズレると全ての状態推定が破綻する。
        結果が示すこと: len(_X0) == 19。"""
        self.assertEqual(len(params._X0), 19)

    def test_x0_quaternion_part_is_unit_quaternion(self):
        """目的: 初期姿勢クォータニオン(_X0[6:10])が単位クォータニオンであることを確認する。
        役割: 非単位クォータニオンで初期化すると、quaternion_to_rotation_matrix等の
        回転演算が不正なスケールを持つ行列を返し、初回から推定が破綻する。
        結果が示すこと: [1,0,0,0]のノルムが1、かつ無回転を表す。"""
        q0 = params._X0[6:10]
        self.assertAlmostEqual(float(np.linalg.norm(q0)), 1.0, places=9)
        self.assertEqual(q0, [1.0, 0.0, 0.0, 0.0])

    def test_dx0_and_p0_shapes_match_18_dim_error_state(self):
        """目的: 誤差状態の初期値・初期共分散が18次元(位置3+速度3+姿勢3+accelbias3+gyrobias3+g3)であることを確認する。
        役割: QEKF.predict/inject/resetは全て(18,)または(18,18)の形状を前提にしており、
        ここがズレると行列演算がブロードキャストエラーになる。
        結果が示すこと: _DX0.shape==(18,1)、_P0.shape==(18,18)。"""
        self.assertEqual(params._DX0.shape, (18, 1))
        self.assertEqual(params._P0.shape, (18, 18))

    def test_std_values_are_positive(self):
        """目的: 各ノイズ標準偏差(_STD_*)が全て正の値であることを確認する。
        役割: 標準偏差が0や負だと共分散行列が特異になり、逆行列計算(update_dvl等の
        inv(H@P@H.T+V))がゼロ除算やnanを生む恐れがある。
        結果が示すこと: _STD_A, _STD_GYRO, _STD_DVL, _STD_DEPTH, _STD_ORIENTATION,
        _STD_A_BIAS, _STD_GYRO_BIAS が全て0より大きい。"""
        for name in ("_STD_A", "_STD_GYRO", "_STD_DVL", "_STD_DEPTH", "_STD_ORIENTATION",
                     "_STD_A_BIAS", "_STD_GYRO_BIAS"):
            with self.subTest(name=name):
                self.assertGreater(getattr(params, name), 0.0)


class TestToleranceAndMaxDt(unittest.TestCase):
    """POSITION_TOLERANCE/DEPTH_TOLERANCE/MAX_DT: 到達判定・dtクランプに使う閾値。"""

    def test_tolerances_are_positive(self):
        """目的: 到達判定に使う許容誤差が正の値であることを確認する。
        役割: 0や負だと is_target_reached() が常にFalse(0の場合は理論上到達不能、
        負の場合は比較が無意味)になり、ホールドモードに入れなくなる。
        結果が示すこと: POSITION_TOLERANCE>0、DEPTH_TOLERANCE>0。"""
        self.assertGreater(params.POSITION_TOLERANCE, 0.0)
        self.assertGreater(params.DEPTH_TOLERANCE, 0.0)

    def test_max_dt_is_positive_and_reasonably_small(self):
        """目的: MAX_DTが正の値で、かつ制御ループとして非常識に大きくないことを確認する。
        役割: MAX_DTはIMU積分・PID微分項のdtクランプ上限。大きすぎると発散防止の意味を
        成さず、0以下だと常にdt=0扱いになりpredict/integrateが実質無効化される。
        結果が示すこと: 0 < MAX_DT <= 5.0[s]。"""
        self.assertGreater(params.MAX_DT, 0.0)
        self.assertLessEqual(params.MAX_DT, 5.0)


class TestPidGainDicts(unittest.TestCase):
    """PID_{SURGE,HEAVE,YAW}_{OUTER,INNER}: pid.PID(**...)にそのまま渡される6つの辞書。"""

    PID_DICT_NAMES = [
        "PID_SURGE_OUTER", "PID_HEAVE_OUTER", "PID_YAW_OUTER",
        "PID_SURGE_INNER", "PID_HEAVE_INNER", "PID_YAW_INNER",
    ]

    def test_all_six_pid_dicts_exist(self):
        """目的: controlfunction.pyが期待する6つのPIDゲイン辞書が全て定義されていることを確認する。
        役割: controlfunction.pyはこれらをPID(**PID_XXX)の形でモジュール読み込み時に
        インスタンス化するため、1つでも欠けるとimport時にNameErrorで制御ループ全体が起動不能になる。
        結果が示すこと: 6辞書すべてがparamsモジュールの属性として存在する。"""
        for name in self.PID_DICT_NAMES:
            with self.subTest(name=name):
                self.assertTrue(hasattr(params, name), f"{name} が定義されていません")

    def test_each_pid_dict_has_required_keys(self):
        """目的: 各PID辞書が pid.PID.__init__ の必須/任意引数と一致するキーを持つことを確認する。
        役割: キー名の typo は PID(**dict) 呼び出し時にTypeError(unexpected keyword)を
        引き起こし、これもimport時に発生するため即座に制御ループ全体が起動不能になる。
        結果が示すこと: 各辞書が {kp, ki, kd, output_limits, windup_limit, angle_error_deg} の
        キーのみを持つ(必須のkp/ki/kdは必ず含む)。"""
        required = {"kp", "ki", "kd"}
        allowed = required | {"output_limits", "windup_limit", "angle_error_deg"}
        for name in self.PID_DICT_NAMES:
            d = getattr(params, name)
            with self.subTest(name=name):
                self.assertTrue(required.issubset(d.keys()))
                self.assertTrue(set(d.keys()).issubset(allowed))

    def test_output_limits_lower_is_not_greater_than_upper(self):
        """目的: output_limits=(lo,hi) の lo<=hi が成立していることを確認する。
        役割: 逆転していると common.clamp(value,lo,hi) が常に lo (実質hi<lo なので
        max(lo,min(hi,value))は常にloになる)を返し、PIDが機能しなくなる。
        結果が示すこと: 6つ全てのPID辞書でoutput_limitsのlo<=hi。"""
        for name in self.PID_DICT_NAMES:
            d = getattr(params, name)
            limits = d.get("output_limits")
            if limits is None:
                continue
            lo, hi = limits
            with self.subTest(name=name):
                self.assertLessEqual(lo, hi)

    def test_surge_outer_lower_limit_is_nonnegative(self):
        """目的: PID_SURGE_OUTERの出力下限が0以上であることを確認する(controlfunction.pyの
        コメント「距離は常に0以上」という前提の裏付け)。
        役割: OuterLoopControlは水平距離(常に非負)を誤差として使うsurge outer PIDの
        設計意図(前進のみ、後退はしない)を守っていることの確認。
        結果が示すこと: PID_SURGE_OUTER['output_limits'][0] >= 0。"""
        lo, _hi = params.PID_SURGE_OUTER["output_limits"]
        self.assertGreaterEqual(lo, 0.0)

    def test_yaw_pids_use_angle_error_deg_only_where_intended(self):
        """目的: yaw系PIDのうち、角度そのもの(方位)を扱うouterはangle_error_deg=True、
        角速度([deg/s])を扱うinnerはangle_error_deg=Falseになっていることを確認する。
        役割: CLAUDE.md/コード内コメントに明記された設計意図(「誤差は既にdeg/sなので
        正規化不要」)が実際の設定値と一致しているかの回帰テスト。ここがずれると
        yawレート誤差が誤って-180~180に丸められ、大きな旋回指令が意図せず削られる。
        結果が示すこと: PID_YAW_OUTER['angle_error_deg'] is True、
        PID_YAW_INNER['angle_error_deg'] is False。"""
        self.assertTrue(params.PID_YAW_OUTER["angle_error_deg"])
        self.assertFalse(params.PID_YAW_INNER["angle_error_deg"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
