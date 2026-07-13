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

`Control/BlueRovStabilityControl.py` opens a MAVLink UDP connection (`udpin:192.168.2.1:14550`, the default
BlueROV2/ArduSub companion-computer address) **at module import time** and blocks on `recv_match(..., blocking=True)`
waiting for a heartbeat. Importing or running this file requires a live vehicle/SITL reachable at that address —
it will hang otherwise. Keep this in mind when writing scripts that import this module.

Run the main control loop with:
```
python Control/BlueRovStabilityControl.py
```

## Tests

`test/testAzimuthControl.py` currently exists but is empty — there is no configured test runner (no pytest config,
no CI). Earlier revisions of the control functions (see `Control/BlueRovSpeedControl.py`, now fully commented out)
carried worked examples as doctest-style docstrings for functions like `AzimuthControl`/`VelocityControl`; that is
the intended pattern for adding tests to this file (`python -m doctest <file> -v`) or in `test/testAzimuthControl.py`.

## Architecture

All active code lives in `Control/`:

- **`BlueRovStabilityControl.py`** — the main entry point (`run_control_loop()`, invoked under
  `if __name__ == "__main__"`). Responsibilities:
  - Low-level vehicle commands over MAVLink: `Arm`/`Disarm`, `ChangeMode`, `SetPwm`, `ManualControl`, `CameraTilt`,
    `GainUp`/`GainDown`.
  - Sensor accessors (`get_acceleration_data`, `get_gyro_data`, `get_velocity_data`, `get_depth_data`,
    `get_position_data`) — **currently stubs that `raise NotImplementedError`**; these must be wired to real sensors
    before the control loop can run end-to-end. `get_sensor_data()` packages IMU+DVL readings into the shape QEKF
    expects.
  - State estimation: `state_estimation()` drives the module-level `_qekf` (a `QEKF` instance from `qekf.py`)
    through predict → integrate → DVL/depth update → inject → reset each loop iteration, using wall-clock `dt`.
  - Guidance/navigation math: `AzimuthControl` (bearing to a target in the XY plane), `VelocitySpeed`/
    `VelocityControl` (converts a target position + current depth into a MAVLink `manual_control` command:
    forward speed, fixed lateral speed, throttle around a 500 neutral, yaw).
  - Safety: `EmergencyProblem(current_state)` currently checks only for depth < 0.1 m; comments mark wall-distance
    and sensor-fault checks as not yet implemented. `run_control_loop()` exits the loop when this trips.
  - `get_target_position()` is a stub returning a fixed `[0, 0, 0]` — the real target-feeding mechanism is not yet
    implemented.

- **`qekf.py`** — the `QEKF` class implementing the error-state quaternion EKF. Nominal state `x` is 19-dim:
  position(3, NED) · velocity(3, NED) · quaternion(4, Hamilton `[qw,qx,qy,qz]`) · accel bias(3) · gyro bias(3) ·
  gravity(3). Error state `dx`/covariance `P` are 18-dim. Key methods, called in this order each control cycle:
  `predict` (propagate covariance from IMU) → `integrate` (propagate nominal state from IMU) →
  `update_dvl`/`update_depth`/`update_orientation` (measurement corrections) → `inject` (fold error-state into
  nominal state) → `reset` (zero the error-state). `create_qekf()`/module-level constants in
  `BlueRovStabilityControl.py` hold the default initial state/covariance/noise parameters, marked as needing
  real-hardware calibration.

- **`utility_functions.py`** — quaternion/rotation math shared by `qekf.py`: `cross` (skew-symmetric matrix),
  `quaternion_to_rotation_matrix`, `quaternion_product` (Hamilton product), `rotation_vector_to_quaternion`
  (exponential map), `rodrigues` (Rodrigues' rotation formula). Pure numpy, no side effects — safe to import/test
  in isolation, unlike `BlueRovStabilityControl.py`.

- **`BlueRovSpeedControl.py`** — an earlier, now fully-commented-out prototype of the same vehicle-control logic.
  Not imported by anything; kept as reference/history rather than active code.

Import order: `BlueRovStabilityControl.py` inserts `Control/`'s own directory onto `sys.path` before importing
`qekf` and `utility_functions` as top-level modules (not a package), so these files must stay siblings in the same
directory.
