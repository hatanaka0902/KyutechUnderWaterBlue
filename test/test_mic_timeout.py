"""
マイク確認待ちの45秒タイムアウトが正しく動くかを検証する。
シナリオA: マイクが一切確認できない -> 45秒でEMERGENCYへ強制遷移するはず
シナリオB: マイクがすぐ確認できる -> 今まで通り正常にSURFACEまで進むはず(タイムアウトに巻き込まれない)
"""
import os
import sys, types, time as time_module
import numpy as np

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

_ctx_ref = {"ctx": None}
_fake_pos_ref = {"pos": np.array([0.0, 0.0, 0.5])}

fake_function = types.ModuleType("function")
fake_function.initialize = lambda: None
def fake_state_estimation():
    ctx = _ctx_ref["ctx"]
    pos = _fake_pos_ref["pos"]
    if ctx is not None and ctx.target_position is not None:
        remaining = ctx.target_position - pos
        pos = pos + remaining * 0.5
    else:
        pos = pos + np.array([0.05, 0.0, 0.05])
    _fake_pos_ref["pos"] = pos
    return np.array([pos[0], pos[1], pos[2], 0,0,0, 1,0,0,0, 0,0,0, 0,0,0, 0,0,9.81], dtype=float)
fake_function.state_estimation = fake_state_estimation
fake_function.get_last_dt = lambda: 0.1
fake_function.get_last_imu_data = lambda: np.zeros((2, 3))
sys.modules["function"] = fake_function

fake_mavlink_io = types.ModuleType("mavlink_io")
fake_mavlink_io.ManualControl = lambda x, y, z, yaw: None
sys.modules["mavlink_io"] = fake_mavlink_io

import flow, perception, params, controlfunction, main
controlfunction.state_estimation = fake_state_estimation

# 安全装置: time.sleepを高速化しつつ、想定外の暴走を検知する
_real_sleep = time_module.sleep
_guard = {"n": 0}
def guarded_sleep(seconds):
    _guard["n"] += 1
    if _guard["n"] > 5000:
        raise RuntimeError("想定外の長時間ループ")
    _real_sleep(seconds)
time_module.sleep = guarded_sleep

params.INIT_WAIT_SEC = 0.02
params.SEARCH_TIMEOUT_SEC = 0.02
params.MAIN_LOOP_HZ = 200.0
params.MIC_CONFIRM_TIMEOUT_SEC = 0.3  # テスト用に45秒 -> 0.3秒に短縮(比率は本番と同じ考え方)


# pitch_deg=50.0: flow.handle_search_hydrophone は pitch_deg > HYDROPHONE_PITCH_THRESHOLD_DEG(30)
# で「発見」と判定する(Bug B対処後の向き)。旧仕様の10.0だと発見成立せず、このテストが検証したい
# マイク確認待ちタイムアウト経路(HOLD_TARGET -> HYDRO_DIVE以降)に入れなくなるため50.0に変更。
perception.get_latest_hydrophone_bearing = lambda: perception.HydrophoneBearing(pitch_deg=50.0, yaw_deg=0.0, timestamp=0.0)
flow._image_processing_available_stub = lambda: False  # HYDRO経路で検証


def run_scenario(label, mic_stub):
    flow._previous_state = None
    controlfunction._was_holding = False
    _fake_pos_ref["pos"] = np.array([0.0, 0.0, 0.5])
    _guard["n"] = 0
    flow._mic_data_from_above_stub = mic_stub

    ctx = flow.MissionContext()
    _ctx_ref["ctx"] = ctx

    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        final_state = main.run_mission(ctx=ctx)
    print(f"[{label}] 最終State: {final_state}  (ticks={_guard['n']})")
    return final_state


print("=== シナリオA: マイクが一切確認できない ===")
result_a = run_scenario("A", lambda: False)
assert result_a == flow.State.EMERGENCY, f"期待値EMERGENCYだが{result_a}だった"
print("  -> 期待通りEMERGENCYへ強制遷移した\n")

print("=== シナリオB: マイクがすぐ確認できる(回帰チェック) ===")
result_b = run_scenario("B", lambda: True)
assert result_b == flow.State.DONE, f"期待値DONEだが{result_b}だった"
print("  -> 期待通りDONEまで正常に進んだ(タイムアウトに巻き込まれていない)")

time_module.sleep = _real_sleep
