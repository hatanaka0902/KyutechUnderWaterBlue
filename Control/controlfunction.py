import math
import os
import sys
from pid import PID
from params import PID_SURGE_OUTER, PID_HEAVE_OUTER, PID_YAW_OUTER
from function import get_last_dt


# 同一ディレクトリの utility_functions を確実に import する
_surge_outer_pid = PID(**PID_SURGE_OUTER)
_heave_outer_pid = PID(**PID_HEAVE_OUTER)
_yaw_outer_pid = PID(**PID_YAW_OUTER)
_CONTROL_DIR = os.path.dirname(os.path.abspath(__file__))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import utility_functions

from params import (
    YAW_KP, YAW_RATE_MAX,
    VEL_XY_MAX, VEL_Z_MIN, VEL_Z_MAX,
    POSITION_TOLERANCE, DEPTH_TOLERANCE,
)
from common import clamp, normalize_deg
from function import state_estimation
from mavlink_io import ManualControl


# 緊急問題用code
def EmergencyProblem(current_state):
    # 現在の深度が0.1m以下の場合は緊急問題としてTrueを返す
    if current_state[2] < 0.1:
        return True
    # 壁との距離が0.5m以下の場合は緊急問題としてTrueを返す

    # センサが異常な場合は緊急問題としてTrueを返す

    return False

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
    # 出力を使う最終段(手順5)でのみ行う。ここで反転すると#2と同じ符号バグを再演する。
    heave_setpoint = _heave_outer_pid.update_from_error(dz, dt)

    target_azimuth_deg = AzimuthControl(dx, dy)
    yaw_rate_setpoint = _yaw_outer_pid.update(target_azimuth_deg, current_yaw_deg, dt)

    return surge_setpoint, heave_setpoint, yaw_rate_setpoint, target_azimuth_deg

def YawRateControl(target_azimuth_deg, current_yaw_deg):
    """目標方位角と現在yawの差から、旋回レート指令(-1000〜1000)を返す"""
    yaw_error_deg = normalize_deg(target_azimuth_deg - current_yaw_deg)
    r_cmd = YAW_KP * yaw_error_deg
    return max(-YAW_RATE_MAX, min(YAW_RATE_MAX, r_cmd))


def AzimuthControl(target_x, target_y):
    """
    目標座標 (target_x, target_y) へのyaw角を返す（度）
    x軸正方向を0度として反時計回りを正とする
    返り値は -180〜180 の範囲（math.atan2 の結果）
    """
    target_azimuth_yaw_degree = math.degrees(math.atan2(target_y, target_x))

    return target_azimuth_yaw_degree

def VelocitySpeed(target_x, target_y, target_z):
    """
    戻り値: (velocity_x, velocity_y, velocity_z)
      いずれもManualControlが受け付ける範囲にクランプ済み
    """
    horizontal_distance = math.sqrt(target_x**2 + target_y**2)
    target_velocity_x = clamp(horizontal_distance * 100, 0, VEL_XY_MAX)
    target_velocity_y = 0
    target_velocity_z = clamp(-target_z * 100 + 500, VEL_Z_MIN, VEL_Z_MAX)
    return target_velocity_x, target_velocity_y, target_velocity_z

def VelocityControl(target_x, target_y, target_z, current_z):
    '''
    target_x: 目標x座標
    target_y: 目標y座標
    target_z: 目標z座標
    戻り値: (velocity_x, velocity_y, velocity_z)
      velocity_x: 目標への水平速度 (0〜1000)
      velocity_y: 横方向速度 (固定0)
      velocity_z: 上下方向速度 (500が中立, 0〜1000)
      azimuth_yaw_degree: 目標へのyaw角 (-180〜180)
    '''
    target_velocity_x, target_velocity_y, target_velocity_z = VelocitySpeed(target_x, target_y, target_z)
    target_azimuth_yaw_degree = AzimuthControl(target_x, target_y)
    yaw_rate_cmd = YawRateControl(target_azimuth_yaw_degree, current_yaw_deg)
    ManualControl(target_velocity_x, target_velocity_y, target_velocity_z, yaw_rate_cmd)
    return target_velocity_x, target_velocity_y, target_velocity_z, yaw_rate_cmd


def get_target_position():
    # return [dx, dy, dz](m) 現在位置からのオフセット(NED)
    return [0, 0, 0]


def is_target_reached(target_x, target_y, target_z):
    return math.sqrt(target_x**2 + target_y**2) < POSITION_TOLERANCE and abs(target_z) < DEPTH_TOLERANCE

def run_control_loop():
    Flag = True
    while Flag:
        current_state = state_estimation()
        if EmergencyProblem(current_state):
            Flag = False
            break

        target_x, target_y, target_z = get_target_position()

        if is_target_reached(target_x, target_y, target_z):
            ManualControl(0, 0, 500, 0)
            continue

        _, _, current_yaw_rad = utility_functions.quaternion_to_euler(current_state[6:10])
        current_yaw_deg = math.degrees(current_yaw_rad)
        VelocityControl(target_x, target_y, target_z, current_yaw_deg)
