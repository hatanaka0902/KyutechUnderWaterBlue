# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Control software for a BlueROV (underwater ROV) used by a Kyutech (Kyushu Institute of Technology) team. It talks to
the vehicle's ArduSub autopilot over MAVLink (via `pymavlink`) and estimates vehicle state with a custom error-state
quaternion EKF (QEKF). The QEKF design follows Joan Sola's "Quaternion kinematics for the error-state Kalman filter"
(arXiv:1711.02508) — equation numbers in `qekf.py` comments refer to that paper. It is also adapted from
https://github.com/bjornrho/Navigation-brov2/tree/main (referenced in README.md).

Code comments and docstrings are largely in Japanese.

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

`test/testAzimuthControl.py` currently exists but is empty — there is no configured test runner (no pytest config,
no CI). Earlier revisions of the control functions (see `Control/BlueRovSpeedControl.py`, now fully commented out)
carried worked examples as doctest-style docstrings for functions like `AzimuthControl`/`VelocityControl`; that is
the intended pattern for adding tests to this file (`python -m doctest <file> -v`) or in `test/testAzimuthControl.py`.

## Architecture

All active code lives in `Control/`. The old monolithic `BlueRovStabilityControl.py` has been split by dependency
layer; each file only imports from files at or below its own layer (no circular imports):

```
params.py, common.py, qekf.py + utility_functions.py   (leaves — no intra-Control deps)
        ↓
function.py           (params, qekf)
        ↓                              mavlink_io.py   (params)
controlfunction.py     (params, common, function, mavlink_io)
        ↓
main.py                (function, controlfunction, mavlink_io)
```

- **`params.py`** — constants only, no dependencies on other `Control/` modules: QEKF initial state/covariance/noise
  (`_X0`, `_DX0`, `_P0`, `_STD_*`, `*_OFFSET`), control gains/thresholds (`YAW_KP`, `YAW_RATE_MAX`, `VEL_XY_MAX`,
  `VEL_Z_MIN`/`VEL_Z_MAX`, `POSITION_TOLERANCE`, `DEPTH_TOLERANCE`, `MAX_DT`), and `MAVLINK_CONNECTION_STRING`
  (`'udpin:192.168.2.1:14550'`). All marked as needing real-hardware calibration.

- **`common.py`** — generic, BlueROV-agnostic helpers: `clamp(value, lo, hi)`, `normalize_deg(angle)`.

- **`function.py`** — sensor-to-state-estimate pipeline. Sensor accessors (`get_acceleration_data`, `get_gyro_data`,
  `get_velocity_data`, `get_depth_data`, `get_position_data`) — **currently stubs that `raise NotImplementedError`**;
  these must be wired to real sensors before the control loop can run end-to-end. `get_sensor_data()` packages
  IMU+DVL readings into the shape QEKF expects. `create_qekf()`/`initialize()` build/reset the module-level `_qekf`
  (a `QEKF` instance from `qekf.py`). `state_estimation()` drives `_qekf` through predict → integrate → DVL/depth
  update → inject → reset each loop iteration, using wall-clock `dt` (clamped to `MAX_DT` via `common.clamp`).

- **`controlfunction.py`** — guidance/navigation math and the control loop. `AzimuthControl` (bearing to a target
  in the XY plane), `VelocitySpeed`/`VelocityControl` (converts a target position + current depth into a MAVLink
  `manual_control` command: forward speed, fixed lateral speed, throttle around a 500 neutral, yaw). **Known bug:**
  `VelocityControl`'s 4th parameter is named `current_z`, but the function body references `current_yaw_deg` (not
  a parameter or global) when calling `YawRateControl` — this raises `NameError` at runtime. `run_control_loop()`
  already computes and passes a yaw-in-degrees value there, so the parameter name is the likely bug, not the call
  site. Also here: `EmergencyProblem(current_state)` (currently checks only depth < 0.1 m; wall-distance and
  sensor-fault checks are marked as not yet implemented), `get_target_position()` (stub returning a fixed
  `[0, 0, 0]`), `is_target_reached()`, and `run_control_loop()` — the main loop, calling into `function.py` for
  state estimation and `mavlink_io.py` for vehicle commands, exiting when `EmergencyProblem` trips.

- **`mavlink_io.py`** — low-level MAVLink I/O: opens the `BLUEROV` connection at import time (see above), plus
  `Arm`/`Disarm`, `ChangeMode`, `SetPwm`, `ManualControl`, `CameraTilt`, `GainUp`/`GainDown`. **Known bug:**
  `GainUp`/`GainDown` reference a global `GAIN` that is never initialized anywhere — calling either raises
  `NameError`. Left as-is (marked with a `TODO` in the file) since fixing it wasn't in scope for the module split.

- **`main.py`** — the entry point (`if __name__ == "__main__"`): imports `mavlink_io` first (so the MAVLink
  connection/heartbeat wait happens before anything else, matching the original file's behavior), then calls
  `initialize()` (from `function.py`) → `run_control_loop()` (from `controlfunction.py`) → a final `ManualControl`
  stop command.

- **`qekf.py`** — the `QEKF` class implementing the error-state quaternion EKF. Nominal state `x` is 19-dim:
  position(3, NED) · velocity(3, NED) · quaternion(4, Hamilton `[qw,qx,qy,qz]`) · accel bias(3) · gyro bias(3) ·
  gravity(3). Error state `dx`/covariance `P` are 18-dim. Key methods, called in this order each control cycle:
  `predict` (propagate covariance from IMU) → `integrate` (propagate nominal state from IMU) →
  `update_dvl`/`update_depth`/`update_orientation` (measurement corrections) → `inject` (fold error-state into
  nominal state) → `reset` (zero the error-state).

- **`utility_functions.py`** — quaternion/rotation math shared by `qekf.py`: `cross` (skew-symmetric matrix),
  `quaternion_to_euler`, `quaternion_to_rotation_matrix`, `quaternion_product` (Hamilton product),
  `rotation_vector_to_quaternion` (exponential map), `rodrigues` (Rodrigues' rotation formula). Pure numpy, no
  side effects — safe to import/test in isolation, unlike the MAVLink-connecting modules above.

- **`BlueRovSpeedControl.py`** — an earlier, now fully-commented-out prototype of the same vehicle-control logic.
  Not imported by anything; kept as reference/history rather than active code.

Import order: `qekf.py`, `function.py`, and `controlfunction.py` each insert `Control/`'s own directory onto
`sys.path` before importing `qekf`/`utility_functions` as top-level modules (not a package), so all files in
`Control/` must stay siblings in the same directory.
