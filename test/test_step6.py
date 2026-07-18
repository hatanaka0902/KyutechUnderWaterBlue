"""
main.run_mission()を、乱数で挙動を変えるスタブを差し込みながら多数回実行し、
状態遷移グラフに抜け・無限ループが無いかを検証する。
"""
import os
import sys
import types
import random
import time as time_module
import numpy as np

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

# ---- センサ/MAVLinkのハードウェア依存部分を差し替える(Step1〜5と同じ理由) ----
_ctx_ref = {"ctx": None}
_fake_pos_ref = {"pos": None}

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

fake_function.state_estimation = fake_state_estimation  # import時点で必要(後でcontrolfunction側にも直接パッチする)
fake_function.get_last_dt = lambda: 0.1
fake_function.get_last_imu_data = lambda: np.zeros((2, 3))
sys.modules["function"] = fake_function

fake_mavlink_io = types.ModuleType("mavlink_io")
fake_mavlink_io.ManualControl = lambda x, y, z, yaw: None
sys.modules["mavlink_io"] = fake_mavlink_io

# Control/ 配下のモジュール（flow, perception, params, controlfunction, main）
import flow
import perception
import params
import controlfunction
import main

controlfunction.state_estimation = fake_state_estimation  # 実際に参照される場所へパッチ(Step5で学んだ通り)

# ---- time.sleepを横取りして高速化 + 無限ループ検知の安全装置にする ----
_real_sleep = time_module.sleep
_tick_guard = {"n": 0, "max_ticks": 3000}

def guarded_sleep(seconds):
    _tick_guard["n"] += 1
    if _tick_guard["n"] > _tick_guard["max_ticks"]:
        raise RuntimeError(f"MAX_TICKS({_tick_guard['max_ticks']})超過。無限ループの疑いあり")
    _real_sleep(seconds)

time_module.sleep = guarded_sleep

params.INIT_WAIT_SEC = 0.02
params.SEARCH_TIMEOUT_SEC = 0.02
params.MAIN_LOOP_HZ = 1000.0


def make_random_stubs(rng):
    """呼ばれるたびに条件へ徐々に近づく乱数スタブ群(いつかは必ず条件を満たすように設計)"""
    hydro_calls = {"n": 0}
    def random_hydro_bearing():
        hydro_calls["n"] += 1
        pitch = 60.0 - hydro_calls["n"] * rng.uniform(2.0, 10.0)
        yaw = rng.uniform(-180, 180)
        return perception.HydrophoneBearing(pitch_deg=pitch, yaw_deg=yaw, timestamp=0.0)

    image_available = rng.random() < 0.5

    yolo_calls = {"n": 0}
    yolo_confirm_after = rng.randint(1, 4)
    def random_yolo_detection():
        yolo_calls["n"] += 1
        detected = yolo_calls["n"] > yolo_confirm_after
        return perception.YoloDetection(
            detected=detected,
            x_m=rng.uniform(0.5, 3.0), y_m=rng.uniform(-1, 1), z_m=rng.uniform(-1, 1),
            confidence=rng.uniform(0.6, 1.0) if detected else rng.uniform(0.0, 0.4),
            timestamp=time_module.time(),  # 実時刻にする(0.0だとstale判定で常に棄却されてしまう)
        )

    mic_calls = {"n": 0}
    mic_confirm_after = rng.randint(0, 4)  # 0〜4回リトライさせてから確定させる
    def random_mic_stub():
        mic_calls["n"] += 1
        return mic_calls["n"] > mic_confirm_after

    return random_hydro_bearing, image_available, random_yolo_detection, random_mic_stub


N_TRIALS = 40
results = {}
errors = []

for seed in range(N_TRIALS):
    rng = random.Random(seed)

    # モジュールレベルの永続変数を毎回リセットする(前回の実行結果を引きずらないように)
    flow._previous_state = None
    controlfunction._was_holding = False
    _tick_guard["n"] = 0
    _fake_pos_ref["pos"] = np.array([0.0, 0.0, 0.5])

    bearing_fn, image_available, yolo_fn, mic_fn = make_random_stubs(rng)
    perception.get_latest_hydrophone_bearing = bearing_fn
    perception.get_latest_yolo_detection = yolo_fn
    flow._image_processing_available_stub = lambda ia=image_available: ia
    flow._mic_data_from_above_stub = mic_fn

    ctx = flow.MissionContext()
    _ctx_ref["ctx"] = ctx

    try:
        import io
        import contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            final_state = main.run_mission(ctx=ctx)
        results[final_state] = results.get(final_state, 0) + 1
    except Exception as e:
        errors.append((seed, type(e).__name__, str(e)))
        results["EXCEPTION"] = results.get("EXCEPTION", 0) + 1

time_module.sleep = _real_sleep  # 後片付け

print(f"=== {N_TRIALS}回試行した結果 ===\n")
for k, v in results.items():
    print(f"  {k}: {v}回")

if errors:
    print(f"\n異常終了したseed: {len(errors)}件")
    for seed, exc_type, msg in errors[:5]:
        print(f"  seed={seed}: {exc_type}: {msg}")
else:
    print("\n異常終了は0件でした")
