import math
import os
import sys

_CONTROL_DIR = os.path.dirname(os.path.abspath(__file__))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import utility_functions
from function import state_estimation, get_last_dt, get_last_imu_data
from mavlink_io import ManualControl
from params import (
    POSITION_TOLERANCE, DEPTH_TOLERANCE,
    PID_SURGE_OUTER, PID_HEAVE_OUTER, PID_YAW_OUTER,
    PID_SURGE_INNER, PID_HEAVE_INNER, PID_YAW_INNER,
)
from pid import PID


_surge_outer_pid = PID(**PID_SURGE_OUTER)
_heave_outer_pid = PID(**PID_HEAVE_OUTER)
_yaw_outer_pid = PID(**PID_YAW_OUTER)
_surge_inner_pid = PID(**PID_SURGE_INNER)
_heave_inner_pid = PID(**PID_HEAVE_INNER)
_yaw_inner_pid = PID(**PID_YAW_INNER)


# 緊急問題用code
def EmergencyProblem(current_state):
    """
    current_state: QEKFの公称状態(19要素のnp.ndarray, function.state_estimation()の戻り値)。
    現在は current_state[2] (position z [m], NED, 正=下方向) のみで判定する。
    """
    # 現在の深度が0.1m以下の場合は緊急問題としてTrueを返す
    if current_state[2] < 0.1:
        return True
    # 壁との距離が0.5m以下の場合は緊急問題としてTrueを返す

    # センサが異常な場合は緊急問題としてTrueを返す

    return False


def AzimuthControl(target_x, target_y):
    """
    目標座標 (target_x, target_y) [m, NED] へのyaw角を返す（度）
    x軸正方向を0度として反時計回りを正とする
    返り値は -180〜180 の範囲（math.atan2 の結果）
    """
    target_azimuth_yaw_degree = math.degrees(math.atan2(target_y, target_x))

    return target_azimuth_yaw_degree


def OuterLoopControl(dx, dy, dz, current_yaw_deg, dt):
    """
    位置/yaw誤差から速度指令(setpoint)を計算する外側ループ。

    Args:
        dx, dy, dz: 現在位置からの目標オフセット [m] (NED)
        current_yaw_deg: 現在のyaw角 [deg]
        dt: state_estimation()と同じdt(get_last_dt()で取得)

    Returns:
        surge_setpoint      [m/s]:   前進方向の目標速度(常に0以上)
        heave_setpoint      [m/s]:   深度方向の目標速度(NED, 正=より深く沈む方向)
        yaw_rate_setpoint   [deg/s]: 目標旋回レート
        target_azimuth_deg  [deg]:   目標方位角(ログ・デバッグ用)
    """
    horizontal_distance = math.sqrt(dx**2 + dy**2)
    surge_setpoint = _surge_outer_pid.update_from_error(horizontal_distance, dt)

    # NEDのまま(符号反転しない)。ManualControlの500中立表現への変換は内側ループの
    # 出力を使う最終段(run_control_loopのManualControl呼び出し)でのみ行う。
    heave_setpoint = _heave_outer_pid.update_from_error(dz, dt)

    target_azimuth_deg = AzimuthControl(dx, dy)
    yaw_rate_setpoint = _yaw_outer_pid.update(target_azimuth_deg, current_yaw_deg, dt)

    return surge_setpoint, heave_setpoint, yaw_rate_setpoint, target_azimuth_deg


def get_target_position():
    # return [dx, dy, dz](m) 現在位置からのオフセット(NED)
    return [0, 0, 0]


def is_target_reached(target_x, target_y, target_z):
    """
    target_x, target_y, target_z: get_target_position() と同じ NED オフセット [m]。
    水平距離が POSITION_TOLERANCE 未満、かつ |target_z| が DEPTH_TOLERANCE 未満で到達とみなす。
    """
    return math.sqrt(target_x**2 + target_y**2) < POSITION_TOLERANCE and abs(target_z) < DEPTH_TOLERANCE


def get_body_frame_velocity(current_state):
    """
    QEKFのworld(NED)速度を機体座標系に回転させる。
    current_state: 19要素のnp.ndarray(function.state_estimation()の戻り値)。
    """
    q = current_state[6:10]
    v_world = current_state[3:6]
    R = utility_functions.quaternion_to_rotation_matrix(q)
    return R.T @ v_world  # [surge, sway, heave_body] (機体座標系)


def get_yaw_rate_deg(current_state):
    """
    生ジャイロ - QEKF推定バイアス から yawレート[deg/s]を求める。
    current_state: 19要素のnp.ndarray(function.state_estimation()の戻り値)。
    """
    imu_data = get_last_imu_data()
    if imu_data is None:
        return 0.0
    gyro_z_raw = imu_data[1][2]        # 機体角速度zの生値 [rad/s]
    gyro_bias_z = current_state[15]    # QEKFのgyro_bias z成分 [rad/s]
    return math.degrees(gyro_z_raw - gyro_bias_z)


def InnerLoopControl(surge_setpoint, heave_setpoint, yaw_rate_setpoint, current_state, dt):
    """
    速度指令(setpoint)と実測値の誤差から、推力/トルク指令を計算する内側ループ。

    Args:
        surge_setpoint, heave_setpoint, yaw_rate_setpoint: OuterLoopControl()の戻り値。
        current_state: 19要素のnp.ndarray(function.state_estimation()の戻り値)。
        dt: state_estimation()と同じdt(get_last_dt()で取得)。

    Returns:
        x_cmd          : ManualControlのxにそのまま渡せる値
        z_cmd_offset    : 500からのオフセット量(NED, 正=沈む方向)。heave_setpoint/heave_measured
                           はNED、ManualControlのzは500中立・値が小さいほど沈む方向という逆の慣習
                           なので、送信直前に符号を反転してから +500 すること
        r_cmd          : ManualControlのrにそのまま渡せる値
    """
    v_body = get_body_frame_velocity(current_state)
    surge_measured = v_body[0]
    heave_measured = current_state[5]      # world/NEDのvzをそのまま使う(outerと同じ座標系)
    yaw_rate_measured = get_yaw_rate_deg(current_state)

    x_cmd = _surge_inner_pid.update(surge_setpoint, surge_measured, dt)
    z_cmd_offset = _heave_inner_pid.update(heave_setpoint, heave_measured, dt)
    r_cmd = _yaw_inner_pid.update(yaw_rate_setpoint, yaw_rate_measured, dt)

    return x_cmd, z_cmd_offset, r_cmd


def reset_all_pids():
    """目標到達後や緊急停止後など、PIDの内部状態をクリアしたい時に呼ぶ"""
    _surge_outer_pid.reset()
    _heave_outer_pid.reset()
    _yaw_outer_pid.reset()
    _surge_inner_pid.reset()
    _heave_inner_pid.reset()
    _yaw_inner_pid.reset()


def run_control_loop():
    """
    メイン制御ループ。状態推定 → 緊急停止判定 → 目標到達判定 → 外側/内側PIDループの順に
    毎周期実行し、ManualControl (MAVLink) で機体へ速度・旋回指令を送信する。
    """
    Flag = True
    was_holding = False

    while Flag:
        current_state = state_estimation()
        if EmergencyProblem(current_state):
            Flag = False
            break

        dx, dy, dz = get_target_position()

        if is_target_reached(dx, dy, dz):
            if not was_holding:
                reset_all_pids()   # ホールドに入る瞬間に一度だけリセット
                was_holding = True
            ManualControl(0, 0, 500, 0)
            continue
        was_holding = False

        dt = get_last_dt()
        _, _, current_yaw_rad = utility_functions.quaternion_to_euler(current_state[6:10])
        current_yaw_deg = math.degrees(current_yaw_rad)

        surge_sp, heave_sp, yaw_rate_sp, _ = OuterLoopControl(dx, dy, dz, current_yaw_deg, dt)
        x_cmd, z_cmd_offset, r_cmd = InnerLoopControl(surge_sp, heave_sp, yaw_rate_sp, current_state, dt)

        # heave_setpoint/heave_measuredはNED(正=沈む方向)、ManualControlのzは500中立・
        # 値が小さいほど沈む方向という逆の慣習のため、ここで符号を反転する。
        ManualControl(x_cmd, 0, -z_cmd_offset + 500, r_cmd)
