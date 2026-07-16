"""
Control/pid.py の単体テスト。

対象クラス: PID(kp, ki, kd, output_limits=None, windup_limit=None, angle_error_deg=False)
    - update(setpoint, measurement, dt)      : setpoint-measurement から誤差を計算して更新
    - update_from_error(error, dt)           : 誤差が既に計算済みの場合に使用
    - reset()                                 : 積分項・前回誤差・前回出力をクリア

pid.py は common.clamp にのみ依存する純粋なロジックであり、実機(BlueROV)は不要。
controlfunction.py の6つのPIDインスタンス(surge/heave/yaw × outer/inner)は
全てこのクラスから生成されるため、ここでの不具合は誘導制御全体に影響する。

実行方法:
    python test/test_pid.py
"""

import os
import sys
import unittest

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from pid import PID  # noqa: E402


class TestPidProportional(unittest.TestCase):
    """P項単体の振る舞い(ki=kd=0)。実際に params.py の6つのPIDは全てki=kd=0の
    P制御器として運用されているため、まずここが正しくないと制御全体が成立しない。"""

    def test_pure_p_output_equals_kp_times_error(self):
        """目的: kp のみを持つPIDの出力が kp*error に一致することを確認する。
        役割: update_from_error() は誤差に比例した出力を返す最も基本的な機能。
        結果が示すこと: kp=2, error=3 なら出力は 6 になるはず。"""
        pid = PID(kp=2.0, ki=0.0, kd=0.0)
        output = pid.update_from_error(3.0, dt=0.1)
        self.assertAlmostEqual(output, 6.0)

    def test_update_computes_error_as_setpoint_minus_measurement(self):
        """目的: update() が setpoint-measurement を誤差として使っていることを確認する。
        役割: 速度PID(InnerLoopControl)はこのupdate()経由で使われるため、符号を誤ると
        実機が目標と逆方向に加速する重大な誤動作になる。
        結果が示すこと: setpoint=10, measurement=4 → error=6 → kp=1なら出力6。"""
        pid = PID(kp=1.0, ki=0.0, kd=0.0)
        output = pid.update(setpoint=10.0, measurement=4.0, dt=0.1)
        self.assertAlmostEqual(output, 6.0)


class TestPidOutputLimits(unittest.TestCase):
    """output_limits: 出力クランプ。実機に送るコマンド値の上限/下限を守る安全装置。"""

    def test_output_is_clamped_to_upper_limit(self):
        """目的: 大きな誤差でも出力が output_limits の上限を超えないことを確認する。
        役割: 例えばsurge_innerのoutput_limits=(0,400)のように、実機コマンドの
        レンジを超えないようにするための保護。
        結果が示すこと: kp*error=1000でもoutput_limits=(0,400)なら出力は400。"""
        pid = PID(kp=100.0, ki=0.0, kd=0.0, output_limits=(0.0, 400.0))
        output = pid.update_from_error(10.0, dt=0.1)
        self.assertAlmostEqual(output, 400.0)

    def test_output_is_clamped_to_lower_limit(self):
        """目的: 負に大きい誤差でも出力が output_limits の下限を下回らないことを確認する。
        役割: 上のテストと対称。下限側の保護が機能していることを保証する。
        結果が示すこと: kp*error=-1000でもoutput_limits=(-50,50)なら出力は-50。"""
        pid = PID(kp=100.0, ki=0.0, kd=0.0, output_limits=(-50.0, 50.0))
        output = pid.update_from_error(-10.0, dt=0.1)
        self.assertAlmostEqual(output, -50.0)


class TestPidWindup(unittest.TestCase):
    """windup_limit: 積分項自体のアンチワインドアップ。ki>0の場合に、飽和した誤差が
    積分項を無限に増大させて出力が過大化・発振する(ワインドアップ)のを防ぐ。"""

    def test_integral_term_accumulates_with_ki(self):
        """目的: ki>0の時、繰り返し呼ぶことで積分項の寄与が出力に加算されていくことを確認する。
        役割: 定常誤差を打ち消すための積分制御が機能していることの確認。
        結果が示すこと: kp=0のまま複数回updateすると、出力(=i_term)は単調に増加する。"""
        pid = PID(kp=0.0, ki=1.0, kd=0.0)
        out1 = pid.update_from_error(1.0, dt=1.0)
        out2 = pid.update_from_error(1.0, dt=1.0)
        self.assertGreater(out2, out1)

    def test_integral_is_clamped_by_windup_limit(self):
        """目的: windup_limit を設定すると内部の積分値がその範囲でクランプされることを確認する。
        役割: 誤差が飽和し続けても積分項が無限に増大しないようにする安全装置。
        結果が示すこと: windup_limit=2.0のとき、大量に積分してもintegral属性は2.0を超えない。"""
        pid = PID(kp=0.0, ki=1.0, kd=0.0, windup_limit=2.0)
        for _ in range(100):
            pid.update_from_error(10.0, dt=1.0)
        self.assertLessEqual(pid.integral, 2.0)


class TestPidDerivative(unittest.TestCase):
    """kd: 微分項。誤差の変化率に応じた出力(急な誤差変化を減衰させる項)。"""

    def test_derivative_is_zero_on_first_call(self):
        """目的: 初回呼び出し(前回誤差が無い状態)ではD項が寄与しないことを確認する。
        役割: last_error が未初期化な状態で微分を計算すると誤った急激な出力が出る
        恐れがあるため、初回はD項を0として扱う実装になっている。
        結果が示すこと: kp=0,ki=0,kd任意でも初回の出力は0になる。"""
        pid = PID(kp=0.0, ki=0.0, kd=5.0)
        output = pid.update_from_error(10.0, dt=0.1)
        self.assertAlmostEqual(output, 0.0)

    def test_derivative_reacts_to_error_change(self):
        """目的: 2回目以降の呼び出しでD項が (error - last_error)/dt に比例して効くことを確認する。
        役割: 誤差が急増した場合に出力を先読みして反応する、Dゲインの基本動作の確認。
        結果が示すこと: error 0→10 (dt=1) なら D項は kd*10 になるはず(P,Iは0なので)。"""
        pid = PID(kp=0.0, ki=0.0, kd=2.0)
        pid.update_from_error(0.0, dt=1.0)
        output = pid.update_from_error(10.0, dt=1.0)
        self.assertAlmostEqual(output, 20.0)


class TestPidDtGuard(unittest.TestCase):
    """dt<=0 のガード: pid.py は内部で time.time() を呼ばず、dtを毎回外部から
    受け取る設計になっている。dtが取得できない/異常な呼び出しに対する挙動を保証する。"""

    def test_non_positive_dt_holds_last_output_after_init(self):
        """目的: 一度出力した後に dt<=0 で呼ばれた場合、前回出力を保持することを確認する。
        役割: dt計測に失敗した回でも出力が急に0や異常値に飛ばないようにする安全策。
        結果が示すこと: 1回目の出力が6.0の後、dt=0で呼んでも6.0のまま変化しない。"""
        pid = PID(kp=2.0, ki=0.0, kd=0.0)
        first = pid.update_from_error(3.0, dt=0.1)
        held = pid.update_from_error(999.0, dt=0.0)
        self.assertAlmostEqual(held, first)

    def test_non_positive_dt_before_any_init_returns_zero(self):
        """目的: reset直後・一度も有効なdtで呼ばれていない状態で dt<=0 が来た場合、
        安全な既定値である0.0を返すことを確認する。
        役割: 制御ループ起動直後の異常入力に対するフェイルセーフ。
        結果が示すこと: 初期化直後に dt=-1 で呼んでも出力は0.0。"""
        pid = PID(kp=2.0, ki=0.0, kd=0.0)
        output = pid.update_from_error(5.0, dt=-1.0)
        self.assertAlmostEqual(output, 0.0)


class TestPidAngleErrorNormalization(unittest.TestCase):
    """angle_error_deg=True: yaw誤差を-180〜180度に正規化するフラグ。
    yaw系PID(PID_YAW_OUTER)がこれを使う。誤ると180度反対方向に旋回する事故に繋がる。"""

    def test_large_positive_error_is_wrapped(self):
        """目的: 200度のような「一周未満だが180度を超える」誤差が-160度に正規化されることを確認する。
        役割: 目標方位と現在yawの差が280度のような場合、実際には短い方向(-80度相当)に
        旋回すべきところを、正規化なしでは長い方向に回ってしまう問題を防ぐ。
        結果が示すこと: angle_error_deg=Trueでerror=200を渡すと、実際に使われる誤差は-160、
        出力はkp*(-160)になる。"""
        pid = PID(kp=1.0, ki=0.0, kd=0.0, angle_error_deg=True)
        output = pid.update_from_error(200.0, dt=0.1)
        self.assertAlmostEqual(output, -160.0)

    def test_update_setpoint_measurement_wraps_across_boundary(self):
        """目的: setpoint/measurementがまたがる±180度境界でも、短い方の回転方向が
        選ばれることを確認する(例: 現在yaw=170度, 目標yaw=-170度)。
        役割: AzimuthControlの返り値(-180〜180)とQEKFのyaw推定値を直接
        update()に渡すcontrolfunction.pyのOuterLoopControlの前提を保証する。
        結果が示すこと: 170度から-170度への誤差は本来20度分の回転で済むはずで、
        正規化なしの-340度ではなく20度が使われる。"""
        pid = PID(kp=1.0, ki=0.0, kd=0.0, angle_error_deg=True)
        output = pid.update(setpoint=-170.0, measurement=170.0, dt=0.1)
        self.assertAlmostEqual(output, 20.0)


class TestPidReset(unittest.TestCase):
    """reset(): 目標到達時やモード切替時にPIDの内部状態(積分項・前回誤差・前回出力)を
    クリアする。controlfunction.reset_all_pids() が全6PIDに対して呼ぶ。"""

    def test_reset_clears_integral_and_initialized_flag(self):
        """目的: 積分が蓄積した状態からresetすると内部状態が初期値に戻ることを確認する。
        役割: ホールドモードに入る/出る際に前回の積分やD項の影響(ワインドアップ・
        微分キック)を持ち込まないようにするための機能。
        結果が示すこと: reset後は integral==0.0, initialized==False, last_error==0.0。"""
        pid = PID(kp=1.0, ki=1.0, kd=1.0)
        pid.update_from_error(5.0, dt=1.0)
        pid.update_from_error(5.0, dt=1.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertFalse(pid.initialized)
        self.assertEqual(pid.last_error, 0.0)

    def test_after_reset_first_call_behaves_like_fresh_pid(self):
        """目的: reset後の最初のupdate呼び出しが、生成直後のPIDと同じ挙動(D項無視)になることを確認する。
        役割: reset()が「新品のPIDに入れ替える」のと等価であることの回帰保証。
        結果が示すこと: reset後最初の呼び出しの出力は、kd項を含まないP項のみの値になる。"""
        pid = PID(kp=2.0, ki=0.0, kd=5.0)
        pid.update_from_error(1.0, dt=0.1)
        pid.reset()
        output = pid.update_from_error(3.0, dt=0.1)
        self.assertAlmostEqual(output, 6.0)  # kd*(error-last_error)/dt は初回無視されP項のみ


if __name__ == "__main__":
    unittest.main(verbosity=2)
