# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Control software for a BlueROV (underwater ROV) used by a Kyutech (Kyushu Institute of Technology) team. It talks to
the vehicle's ArduSub autopilot over MAVLink (via `pymavlink`) and estimates vehicle state with a custom error-state
quaternion EKF (QEKF). The QEKF design follows Joan Sola's "Quaternion kinematics for the error-state Kalman filter"
(arXiv:1711.02508) — equation numbers in `qekf.py` comments refer to that paper. It is also adapted from
https://github.com/bjornrho/Navigation-brov2/tree/main (referenced in README.md).

Code comments and docstrings are largely in Japanese.

Per README.md: all sensor values (IMU, DVL, microphone) are treated as relative to the robot at each time step —
i.e. body-frame readings, not absolute/world-frame. `qekf.py`/`function.py` rotate these into the NED world frame
during the predict/update cycle.

## Environment / running

There is no `requirements.txt`/`pyproject.toml` in this repo. Third-party dependencies used across the code:
`pymavlink`, `numpy`, `keyboard`. Install manually, e.g.:

```
pip install pymavlink numpy keyboard
```

`Control/mavlink_io.py` opens a MAVLink UDP connection (`udpin:192.168.2.1:14550`, the default BlueROV2/ArduSub
companion-computer address, held in `params.MAVLINK_CONNECTION_STRING`) **at module import time** and blocks on
`recv_match(..., blocking=True)` waiting for a heartbeat. Importing `mavlink_io` (directly, or transitively via
`controlfunction`/`main`) requires a live vehicle/SITL reachable at that address — it will hang otherwise. Keep
this in mind when writing scripts that import these modules.

Run the main control loop with:
```
python Control/main.py
```

## Tests

There is no configured test runner (no pytest config, no CI) and no test files currently exist (`test/` is an
empty directory). `utility_functions.py` is pure numpy with no side effects, so it's the easiest module to write
standalone tests against; everything else either raises `NotImplementedError` (the `function.py` sensor stubs) or
imports `mavlink_io`, which connects to real hardware/SITL at import time (see above).

## Architecture

All active code lives in `Control/`. The old monolithic `BlueRovStabilityControl.py` has been split by dependency
layer; each file only imports from files at or below its own layer (no circular imports):

```
params.py, common.py, qekf.py + utility_functions.py   (leaves — no intra-Control deps)
        ↓                    ↓
function.py (params, qekf)  pid.py (common)     mavlink_io.py (params)
        ↓                        ↓                     ↓
        └──────────── controlfunction.py (params, common, function, pid, mavlink_io) ──────────┘
                              ↓
                           main.py (function, controlfunction, mavlink_io)
```

- **`params.py`** — constants only, no dependencies on other `Control/` modules: QEKF initial state/covariance/noise
  (`_X0`, `_DX0`, `_P0`, `_STD_*`, `*_OFFSET`), `POSITION_TOLERANCE`/`DEPTH_TOLERANCE`/`MAX_DT`, and
  `MAVLINK_CONNECTION_STRING` (`'udpin:192.168.2.1:14550'`). Also six cascade-PID gain dicts consumed by
  `pid.PID(**...)` in `controlfunction.py`: `PID_{SURGE,HEAVE,YAW}_{OUTER,INNER}`. All calibration-sensitive values
  are marked as needing real-hardware tuning.

- **`common.py`** — generic, BlueROV-agnostic helpers: `clamp(value, lo, hi)`, `normalize_deg(angle)`.

- **`pid.py`** — a `PID` class (depends only on `common.clamp`) with configurable `output_limits` (output clamp)
  and `windup_limit` (integral-term anti-windup clamp), and an `angle_error_deg` flag that normalizes the error to
  -180…180° for angular setpoints (used by the yaw PIDs). `update(setpoint, measurement, dt)` computes the error
  internally; `update_from_error(error, dt)` takes a pre-computed error (used for the surge/heave outer loops,
  where the error is a distance/offset rather than a plain setpoint-minus-measurement). `dt` is always passed in
  (no internal clock) — if `dt <= 0`, the previous output is held rather than recomputed. `reset()` clears the
  integral/derivative state; `controlfunction.py` calls this on all six PID instances whenever the vehicle enters
  target-hold, to avoid integral windup/derivative kick on resume.

- **`function.py`** — sensor-to-state-estimate pipeline. Sensor accessors (`get_acceleration_data`, `get_gyro_data`,
  `get_velocity_data`, `get_position_data`) — **currently stubs that `raise NotImplementedError`**; these must be
  wired to real sensors before the control loop can run end-to-end (barometer/depth accessor and the QEKF depth
  update step were removed — state estimation is now IMU-predict + DVL-update only). `get_sensor_data()` packages
  IMU+DVL readings into the shape QEKF expects. `create_qekf()`/`initialize()` build/reset the module-level `_qekf`
  (a `QEKF` instance from `qekf.py`). `state_estimation()` drives `_qekf` through predict → integrate → DVL update
  → inject → reset each loop iteration, using wall-clock `dt` (clamped to `MAX_DT` via `common.clamp`).
  `get_last_dt()`/`get_last_imu_data()` exist so `controlfunction.py`'s cascade PID loop can reuse the same `dt`/IMU
  reading `state_estimation()` just computed, instead of re-measuring; both are updated via a `global _last_time,
  _last_dt, _last_imu_data` declaration at the top of `state_estimation()` (a previous version of this declaration
  omitted `_last_dt`/`_last_imu_data`, silently making them local and leaving the getters stuck at `0.0`/`None` —
  fixed).

- **`controlfunction.py`** — guidance/navigation math and the control loop, built as a cascade PID (outer
  position/yaw loop → inner velocity/rate loop). The single-stage control functions this replaced
  (`YawRateControl`, `VelocitySpeed`, `VelocityControl`, and the `params.py` constants only they used) have been
  deleted. Defined in call order (callee before caller): `EmergencyProblem` → `AzimuthControl` → `OuterLoopControl`
  → `get_target_position`/`is_target_reached` → `get_body_frame_velocity`/`get_yaw_rate_deg` → `InnerLoopControl`
  → `reset_all_pids` → `run_control_loop`. The six module-level `PID` instances (`_surge_outer_pid`,
  `_heave_outer_pid`, `_yaw_outer_pid`, `_surge_inner_pid`, `_heave_inner_pid`, `_yaw_inner_pid`, constructed from
  the `params.PID_*` dicts) are grouped immediately after the imports.
  - `EmergencyProblem(current_state)` — currently checks only depth (`current_state[2]`) < 0.1 m; wall-distance and
    sensor-fault checks are marked as not yet implemented.
  - `AzimuthControl` — bearing to a target in the XY plane.
  - `OuterLoopControl(dx, dy, dz, current_yaw_deg, dt)` — turns a position/yaw error into setpoints: forward speed
    (`_surge_outer_pid`), NED heave rate (`_heave_outer_pid`, positive = sinking — deliberately *not*
    sign-flipped here; the NED→`ManualControl` (500-neutral, smaller-z=sink) convention conversion happens only in
    the final `ManualControl` call in `run_control_loop()`), and yaw rate (`_yaw_outer_pid`, angle-aware).
  - `get_target_position()` (stub returning a fixed `[0, 0, 0]`), `is_target_reached()`.
  - `get_body_frame_velocity`/`get_yaw_rate_deg` — measured-value helpers for `InnerLoopControl` (rotate QEKF NED
    velocity into body frame; gyro-z minus QEKF's estimated gyro bias-z for yaw rate).
  - `InnerLoopControl(surge_sp, heave_sp, yaw_rate_sp, current_state, dt)` — tracks those setpoints against the
    measured values above; returns `(x_cmd, z_cmd_offset, r_cmd)` for `ManualControl`, where `z_cmd_offset` is a
    NED-convention offset from 500 (positive = sink) that the caller must sign-flip before adding to 500.
  - `reset_all_pids()` — resets all six PID instances; called once when entering target-hold.
  - `run_control_loop()` — the main loop: `state_estimation()` → `EmergencyProblem` check → `get_target_position()`
    → hold-in-place (`ManualControl(0,0,500,0)`, resetting PIDs on hold-entry) or `OuterLoopControl` →
    `InnerLoopControl` → `ManualControl(x_cmd, 0, -z_cmd_offset + 500, r_cmd)` (the sign flip here is what converts
    `InnerLoopControl`'s NED-convention `z_cmd_offset` to `ManualControl`'s smaller-z=sink convention).
  - Imports are ordered stdlib → local (no third-party deps in this file), with the `sys.path` insert for
    `utility_functions` grouped with the other local imports, and each of `function`/`params` imported once.

- **`mavlink_io.py`** — low-level MAVLink I/O: opens the `BLUEROV` connection at import time (see above), plus
  `Arm`/`Disarm`, `ChangeMode`, `SetPwm`, `ManualControl` (docstring documents the MANUAL_CONTROL channel
  convention: `x`/`y` -1000…1000, `z` 0…1000 with 500 neutral and smaller-z=sink, `yaw` -1000…1000), `CameraTilt`,
  `GainUp`/`GainDown`. **Known bug:** `GainUp`/`GainDown` reference a global `GAIN` that is never initialized
  anywhere — calling either raises `NameError`. Left as-is (out of scope for recent cleanups).

- **`main.py`** — the entry point (`if __name__ == "__main__"`): imports `mavlink_io` first (so the MAVLink
  connection/heartbeat wait happens before anything else, matching the original file's behavior), then calls
  `initialize()` (from `function.py`) → `run_control_loop()` (from `controlfunction.py`) → a final `ManualControl`
  stop command.

- **`qekf.py`** — the `QEKF` class implementing the error-state quaternion EKF. Nominal state `x` is 19-dim:
  position(3, NED) · velocity(3, NED) · quaternion(4, Hamilton `[qw,qx,qy,qz]`) · accel bias(3) · gyro bias(3) ·
  gravity(3). Error state `dx`/covariance `P` are 18-dim. Cycle structure: `predict` (propagate covariance from
  IMU) → `integrate` (propagate nominal state from IMU) → a measurement-correction step (`update_dvl`, called
  every cycle by `function.py`; `update_depth`/`update_orientation` also exist on the class but currently have no
  caller — see the `function.py` note above about the depth pipeline being removed) → `inject` (fold error-state
  into nominal state) → `reset` (zero the error-state).

- **`utility_functions.py`** — quaternion/rotation math shared by `qekf.py`: `cross` (skew-symmetric matrix),
  `quaternion_to_euler`, `quaternion_to_rotation_matrix`, `quaternion_product` (Hamilton product),
  `rotation_vector_to_quaternion` (exponential map), `rodrigues` (Rodrigues' rotation formula). Pure numpy, no
  side effects — safe to import/test in isolation, unlike the MAVLink-connecting modules above.

- **`BlueRovSpeedControl.py`** — an earlier, unrelated prototype: a standalone manual/keyboard teleop script (`Arm`
  → demo choreography of tilt/LED/gripper/thrust moves → WASD+arrow-key keyboard control loop via the `keyboard`
  package), not the same guidance/PID logic as `controlfunction.py`. All top-level code (including opening its own
  MAVLink connection to `udpin:192.168.2.2:14550`, note the different final octet from
  `params.MAVLINK_CONNECTION_STRING`) runs unconditionally at import time, not gated behind `if __name__`. Not
  imported by anything else in `Control/`; kept as reference/history rather than active code.

Import order: `qekf.py`, `function.py`, and `controlfunction.py` each insert `Control/`'s own directory onto
`sys.path` before importing `qekf`/`utility_functions` as top-level modules (not a package), so all files in
`Control/` must stay siblings in the same directory.
