"""
Control/common.py の単体テスト。

対象関数:
    - clamp(value, lo, hi)      : 値を [lo, hi] にクランプする
    - normalize_deg(angle)      : 角度を -180〜180度の範囲に正規化する

common.py は他モジュールに依存しない純粋関数のみなので、実機(BlueROV)は不要。
実行方法:
    python test/test_common.py
"""

import os
import sys
import unittest

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from common import clamp, normalize_deg  # noqa: E402


class TestClamp(unittest.TestCase):
    """clamp(value, lo, hi): 制御ループ全体でPID出力やdtの範囲制限に使われる、
    最も基本的な安全装置。ここが壊れると出力上限を超えた指令が実機に送られる
    恐れがあるため、境界値を含めて厳密に検証する。"""

    def test_value_within_range_is_unchanged(self):
        """目的: 範囲内の値はそのまま素通しされることを確認する。
        役割: clamp は範囲外の値だけを丸めるべきで、範囲内の値を変えてはいけない。
        結果が示すこと: 3 は [0,10] の内側なので clamp(3,0,10)==3 となるべき。"""
        self.assertEqual(clamp(3, 0, 10), 3)

    def test_value_below_lower_bound_is_raised_to_lo(self):
        """目的: 下限を下回る値が lo に丸められることを確認する。
        役割: PIDの出力下限(例: output_limits の下側)を守るための丸め処理。
        結果が示すこと: -5 は下限0未満なので clamp(-5,0,10)==0 となるべき。"""
        self.assertEqual(clamp(-5, 0, 10), 0)

    def test_value_above_upper_bound_is_lowered_to_hi(self):
        """目的: 上限を上回る値が hi に丸められることを確認する。
        役割: PIDの出力上限(例: スラスタ最大出力相当)を守るための丸め処理。
        結果が示すこと: 15 は上限10を超えるので clamp(15,0,10)==10 となるべき。"""
        self.assertEqual(clamp(15, 0, 10), 10)

    def test_boundary_values_are_unchanged(self):
        """目的: 境界値そのもの(lo, hi)がクランプされずに保持されることを確認する。
        役割: 境界の扱いを >= / <= として実装しているか(閉区間)を保証する。
        結果が示すこと: clamp(0,0,10)==0 かつ clamp(10,0,10)==10。"""
        self.assertEqual(clamp(0, 0, 10), 0)
        self.assertEqual(clamp(10, 0, 10), 10)

    def test_negative_range(self):
        """目的: lo/hi が負の範囲でも正しく機能することを確認する。
        役割: heave(深度方向)PIDのように負の出力レンジを持つケースを想定。
        結果が示すこと: -0.5 は [-0.3, 0.3] の下限未満なので -0.3 に丸められる。"""
        self.assertAlmostEqual(clamp(-0.5, -0.3, 0.3), -0.3)


class TestNormalizeDeg(unittest.TestCase):
    """normalize_deg(angle): yaw誤差などの角度量を -180〜180度に正規化する。
    AzimuthControl や PID の angle_error_deg フラグが依存する基礎関数であり、
    ここが誤っていると旋回制御が反対方向に回る等の重大な誤動作につながる。"""

    def test_angle_within_range_is_unchanged(self):
        """目的: 既に -180〜180 の範囲内にある角度は変化しないことを確認する。
        役割: 正規化は「範囲外の時だけ」360度単位でずらす処理であるべき。
        結果が示すこと: normalize_deg(90) == 90。"""
        self.assertAlmostEqual(normalize_deg(90.0), 90.0)

    def test_angle_above_180_wraps_negative(self):
        """目的: 180度を超える角度が負側に折り返されることを確認する。
        役割: 例えばyaw誤差200度は「-160度回転すればよい」という意味に変換する必要がある。
        結果が示すこと: normalize_deg(200) == -160。"""
        self.assertAlmostEqual(normalize_deg(200.0), -160.0)

    def test_angle_below_minus_180_wraps_positive(self):
        """目的: -180度未満の角度が正側に折り返されることを確認する。
        役割: 上のテストと対称のケース(反対回りの誤差)を保証する。
        結果が示すこと: normalize_deg(-200) == 160。"""
        self.assertAlmostEqual(normalize_deg(-200.0), 160.0)

    def test_boundary_180_maps_to_minus_180(self):
        """目的: 境界値180度がどちらに丸められるか(実装の (angle+180)%360-180 に基づく)を固定する。
        役割: PIDのangle_error_deg正規化と組み合わせた時の符号の一貫性を保証する回帰テスト。
        結果が示すこと: 実装上、180度は -180度に写る((180+180)%360-180 == -180)。"""
        self.assertAlmostEqual(normalize_deg(180.0), -180.0)

    def test_multiple_wraps(self):
        """目的: 360度を大きく超える角度(複数回転分の誤差)でも正しく正規化されることを確認する。
        役割: 旋回を続けてyaw角が何周も蓄積した場合でも誤差計算が破綻しないことの保証。
        結果が示すこと: 720+45=765度は45度と等価であるべき。"""
        self.assertAlmostEqual(normalize_deg(765.0), 45.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
