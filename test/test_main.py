"""
Control/main.py のテスト。

main.py はエントリポイントで、関数は持たず以下の手続きのみを行う:
    if __name__ == "__main__":
        initialize()          # function.py: QEKF初期化
        run_control_loop()    # controlfunction.py: メイン制御ループ(実質無限ループ)
        ManualControl(0,0,300,0)  # mavlink_io.py: 終了時の最終停止コマンド

このファイルでは:
    - 単体テスト: main.pyを実際に実行(runpy)しつつ initialize/run_control_loop/
      ManualControl をモックに差し替え、「呼び出し順序」と「`if __name__=="__main__"`
      ガードが正しく機能している(単純importでは何も実行されない)」ことを検証する。
      実機は不要。
    - 実機起動デモ(run_hardware_demo): main.pyと全く同じ呼び出し順序
      (initialize→run_control_loop→ManualControl停止)を実際のBlueROVに対して実行する。
      run_control_loop()は本来無限ループ(EmergencyProblemの深度判定以外に終了条件が無い)
      なので、デモでは一定時間経過で強制的に終了させるラッパーを被せて安全に区切る。

実行方法:
    python test/test_main.py             # 単体テスト(モック、実機不要)
    python test/test_main.py --hardware   # 実機起動デモ(要: 実機BlueROVに接続済み、最長時間で自動停止)
"""

import importlib
import os
import runpy
import sys
import time
import types
import unittest
from unittest.mock import patch

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROL_DIR = os.path.join(os.path.dirname(_TEST_DIR), "Control")
for _dir in (_CONTROL_DIR, _TEST_DIR):
    if _dir not in sys.path:
        sys.path.insert(0, _dir)

from _hw_helpers import hardware_flag_present, confirm_hardware_action, safe_stop_and_disarm  # noqa: E402

_MAIN_PY_PATH = os.path.join(_CONTROL_DIR, "main.py")


def _make_fake_mavlink_io():
    module = types.ModuleType("mavlink_io")
    calls = []

    def ManualControl(x, y, z, yaw):
        calls.append(("ManualControl", (x, y, z, yaw)))

    module.ManualControl = ManualControl
    module.calls = calls
    return module


def _prepare_modules_with_fake_mavlink():
    """function/controlfunctionを(必要なら)フェイクmavlink_ioの下でimportしておく。
    main.pyの `from controlfunction import run_control_loop` 等が実機接続なしで
    解決できるようにする準備。"""
    fake_mavlink_io = _make_fake_mavlink_io()
    sys.modules["mavlink_io"] = fake_mavlink_io
    sys.modules.pop("controlfunction", None)
    import function  # noqa: F401
    import controlfunction  # noqa: F401
    return fake_mavlink_io


class TestMainCallOrder(unittest.TestCase):
    """main.pyの `if __name__=="__main__":` ブロックが initialize() -> run_control_loop() ->
    ManualControl(0,0,300,0) の順に、かつそれ以外の余計な呼び出しをせずに実行することを確認する。"""

    @classmethod
    def setUpClass(cls):
        cls.fake_mavlink_io = _prepare_modules_with_fake_mavlink()
        import function
        import controlfunction
        cls.function = function
        cls.controlfunction = controlfunction

    def test_main_calls_initialize_then_run_control_loop_then_manual_control_stop(self):
        """目的: main.pyを実行すると、initialize()→run_control_loop()→
        ManualControl(0,0,300,0)の順で、かつ他の呼び出しを挟まずに実行されることを確認する。
        役割: main.pyはプロジェクトの実行入口であり、「初期化してから制御ループに入り、
        ループが終わったら安全な低出力(z=300、waterから浮くよう軽く上向きにする程度)で
        停止する」という設計上必須の順序を保証する。順序が入れ替わると、初期化前に
        制御ループが走って未初期化のQEKFを使ってしまう等の重大な事故になる。
        結果が示すこと: 記録された呼び出し順序が
        ['initialize', 'run_control_loop', ('ManualControl', (0,0,300,0))] と一致する。"""
        calls = []
        with patch.object(self.function, "initialize", side_effect=lambda: calls.append("initialize")), \
             patch.object(self.controlfunction, "run_control_loop",
                           side_effect=lambda: calls.append("run_control_loop")), \
             patch.object(self.fake_mavlink_io, "ManualControl",
                           side_effect=lambda x, y, z, yaw: calls.append(("ManualControl", (x, y, z, yaw)))):
            runpy.run_path(_MAIN_PY_PATH, run_name="__main__")

        self.assertEqual(calls, ["initialize", "run_control_loop", ("ManualControl", (0, 0, 300, 0))])

    def test_importing_main_as_module_does_not_execute_body(self):
        """目的: main.pyを(スクリプト実行ではなく)通常のモジュールとしてimportした場合、
        `if __name__=="__main__":` ブロックの内容が一切実行されないことを確認する。
        役割: 他のモジュールやテストコードがmain.pyを誤ってimportしてしまっても、
        制御ループやManualControlが意図せず起動しない(実機が暴走しない)ことの
        安全確認。
        結果が示すこと: run_name="main"(非"__main__")で実行すると、
        initialize/run_control_loop/ManualControlのいずれも呼ばれない。"""
        calls = []
        with patch.object(self.function, "initialize", side_effect=lambda: calls.append("initialize")), \
             patch.object(self.controlfunction, "run_control_loop",
                           side_effect=lambda: calls.append("run_control_loop")), \
             patch.object(self.fake_mavlink_io, "ManualControl",
                           side_effect=lambda x, y, z, yaw: calls.append("ManualControl")):
            runpy.run_path(_MAIN_PY_PATH, run_name="main_imported_as_module")

        self.assertEqual(calls, [])


# ---------------------------------------------------------------------------
# 実機起動デモ: main.py と全く同じ呼び出し順序(initialize→run_control_loop→
# ManualControl停止)を実際のBlueROVに対して実行する。run_control_loop()自体には
# 時間による終了条件が無いため、デモでは一定時間で強制終了するラッパーを被せる。
# --hardware フラグ付きで実行した時だけ動く。
# ---------------------------------------------------------------------------
def run_hardware_demo(duration_sec: float = 8.0):
    print("=== main.py 実機起動デモ ===")
    print("main.pyと同じ手順(initialize -> run_control_loop -> ManualControl停止)を実行します。")
    print("get_target_position()は[0,0,0]固定スタブのため、通常はホールド"
          "(ManualControl(0,0,500,0)の送信)を繰り返すだけの安全な動作になります。")
    print(f"run_control_loop()自体には時間制限が無いため、このデモでは最長{duration_sec}秒で"
          "強制終了するようにしています。")

    if not confirm_hardware_action(f"main.pyと同じ制御ループを最長{duration_sec}秒実行します。"):
        print("キャンセルしました。")
        return

    sys.modules.pop("mavlink_io", None)
    sys.modules.pop("controlfunction", None)
    import mavlink_io
    import function
    import controlfunction

    original_emergency = controlfunction.EmergencyProblem
    start = time.time()

    def _timeboxed_emergency(current_state):
        if time.time() - start > duration_sec:
            print(f"[デモ終了] {duration_sec}秒経過したため制御ループを終了します。")
            return True
        return original_emergency(current_state)

    try:
        function.initialize()
        mavlink_io.ChangeMode("MANUAL")
        # Arm()はmain.py本体には無いが、ManualControlが実際に機体へ反映されるために必要
        mavlink_io.Arm()
        with patch.object(controlfunction, "EmergencyProblem", side_effect=_timeboxed_emergency):
            controlfunction.run_control_loop()
        mavlink_io.ManualControl(0, 0, 300, 0)
    except KeyboardInterrupt:
        print("\n[中断] Ctrl+Cを検知しました。")
    finally:
        safe_stop_and_disarm(mavlink_io, note="main.py実機デモ終了")
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
