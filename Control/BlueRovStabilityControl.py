import math
from pymavlink import mavutil
import os
import sys
import time
import numpy as np
import keyboard
import csv
import datetime

# 同一ディレクトリの QEKF / utility_functions を import
_CONTROL_DIR = os.path.dirname(os.path.abspath(__file__))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from qekf import QEKF
import utility_functions


# ---------- QEKF 初期パラメータ (実機キャリブレーション後に要チューニング) ----------
# 公称状態 x: [p(3), v(3), q(4), a_bias(3), gyro_bias(3), g(3)] = 19
_X0 = [
    0.0, 0.0, 0.0,          # position (NED) [m]
    0.0, 0.0, 0.0,          # velocity (NED) [m/s]
    1.0, 0.0, 0.0, 0.0,     # quaternion [qw, qx, qy, qz]
    0.0, 0.0, 0.0,          # accel bias
    0.0, 0.0, 0.0,          # gyro bias
    0.0, 0.0, 9.81,         # gravity (NED, z下向き)
]
_DX0 = np.zeros((18, 1))
_P0 = np.eye(18) * 0.1

_STD_A = 0.1               # m/s^2
_STD_GYRO = 0.01           # rad/s
_STD_DVL = 0.02            # m/s
_STD_DEPTH = 0.05          # m
_STD_ORIENTATION = 0.01    # quaternion units
_STD_A_BIAS = 1e-4
_STD_GYRO_BIAS = 1e-5

_DVL_OFFSET = np.zeros(3)
_BAROMETER_OFFSET = np.zeros(3)
_IMU_OFFSET = np.zeros(3)


def create_qekf(
    x_0=None, dx_0=None, P_0=None,
    std_a=None, std_gyro=None, std_dvl=None, std_depth=None,
    std_orientation=None, std_a_bias=None, std_gyro_bias=None,
    dvl_offset=None, barometer_offset=None, imu_offset=None,
):
    """QEKF インスタンスを生成して返す。引数省略時はモジュール既定値を使用。"""
    return QEKF(
        x_0 if x_0 is not None else _X0,
        dx_0 if dx_0 is not None else _DX0.copy(),
        P_0 if P_0 is not None else _P0.copy(),
        std_a if std_a is not None else _STD_A,
        std_gyro if std_gyro is not None else _STD_GYRO,
        std_dvl if std_dvl is not None else _STD_DVL,
        std_depth if std_depth is not None else _STD_DEPTH,
        std_orientation if std_orientation is not None else _STD_ORIENTATION,
        std_a_bias if std_a_bias is not None else _STD_A_BIAS,
        std_gyro_bias if std_gyro_bias is not None else _STD_GYRO_BIAS,
        dvl_offset if dvl_offset is not None else _DVL_OFFSET,
        barometer_offset if barometer_offset is not None else _BAROMETER_OFFSET,
        imu_offset if imu_offset is not None else _IMU_OFFSET,
    )


# 初期化用の関数
def initialize(qekf=None):
    """キャリブレーション・フィルタ初期化。qekf 未指定時は既定パラメータで生成。"""
    global _qekf, _last_time
    _qekf = qekf if qekf is not None else create_qekf()
    _last_time = None
    return _qekf


# Start a connection listening on a UDP port
BLUEROV = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
# Wait for the first heartbeat
heartbeat = BLUEROV.recv_match(type='HEARTBEAT', condition=f'HEARTBEAT.get_srcComponent() == {mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1}', blocking=True)
print(heartbeat)



def Arm():
    # master.arducopter_arm() or:
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system,
        BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    BLUEROV.motors_armed_wait()
    print('Armed!')


def Disarm():
    # master.arducopter_disarm() or:
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system,
        BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    # wait until disarming confirmed
    print("Waiting for the vehicle to disarm")
    BLUEROV.motors_disarmed_wait()
    print('Disarmed!')


def ChangeMode(mode='MANUAL'):
    """
    :param mode: 'MANUAL' or 'STABILIZE' or 'ALT_HOLD'
    """
    if mode not in BLUEROV.mode_mapping():
        print("Unknown mode : {}".format(mode))
        print('Try: ', list(BLUEROV.mode_mapping().keys()))
        sys.exit(1)
    mode_id = BLUEROV.mode_mapping()[mode]
    BLUEROV.mav.set_mode_send(
        BLUEROV.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def SetPwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    # https://www.ardusub.com/developers/rc-input-and-output.html
    if channel_id < 1 or 14 < channel_id:
        print("Channel does not exist.")
        return
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system, BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel_id,
        pwm,
        0, 0, 0, 0, 0
    )


def ManualControl(x, y, z, yaw):
    BLUEROV.mav.manual_control_send(
        BLUEROV.target_system,
        x,
        y,
        z,
        yaw,
        0)


def CameraTilt(tilt, roll=0, pan=0):
    """
    Moves gimbal to given position
    Args:
        tilt (float): tilt angle in centidegrees (0 is forward)
        roll (float, optional): pan angle in centidegrees (0 is forward)
        pan  (float, optional): pan angle in centidegrees (0 is forward)
    """
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system,
        BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)


def GainUp():
    global GAIN
    GAIN += 100
    if 1000 < GAIN:
        GAIN = 1000
    print("Gain is", GAIN)


def GainDown():
    global GAIN
    GAIN -= 100
    if GAIN < 0:
        GAIN = 0
    print("Gain is", GAIN)

# 緊急問題用code
def EmergencyProblem(current_state):
    # 現在の深度が0.1m以下の場合は緊急問題としてTrueを返す
    if current_state[2] < 0.1:
        return True
    # 壁との距離が0.5m以下の場合は緊急問題としてTrueを返す

    # センサが異常な場合は緊急問題としてTrueを返す

    return False


# ---------- センサ取得 (実センサ読み出しに置き換え) ----------
def get_acceleration_data():
    """比力 [ax, ay, az] (m/s^2, body)"""
    raise NotImplementedError

def get_gyro_data():
    """角速度 [gx, gy, gz] (rad/s, body)"""
    raise NotImplementedError

def get_velocity_data():
    """DVL対地速度 [vx, vy, vz] (m/s, body)。途絶時は None"""
    raise NotImplementedError

def get_depth_data():
    """気圧深度 [m] (NED z)。途絶時は None"""
    raise NotImplementedError

def get_position_data():
    """位置観測用スタブ (現状QEKFはDVL速度更新を使用)"""
    raise NotImplementedError


def get_sensor_data():
    """IMU + DVL を取得し、QEKF 入力形式で返す。

    Returns:
        imu_data: shape (2, 3) — [accel(3,), gyro(3,)]
        dvl: (v_body, R) — v_body は shape (3,), R は (3,3)。途絶時は (None, None)
    """
    accel = np.asarray(get_acceleration_data(), dtype=float).flatten()
    gyro = np.asarray(get_gyro_data(), dtype=float).flatten()
    imu_data = np.vstack([accel, gyro])  # shape (2, 3)

    DVL_COVARIANCE_BODY = np.diag([0.02, 0.02, 0.03]) ** 2
    v_body = get_velocity_data()
    if v_body is None:
        return imu_data, (None, None)
    v_body = np.asarray(v_body, dtype=float).reshape(3, 1)
    return imu_data, (v_body, DVL_COVARIANCE_BODY)


# ---------- 状態推定 (QEKF) ----------
_qekf = create_qekf()
_last_time = None


def state_estimation():
    """IMU予測 + DVL/深度更新で状態を推定し、公称状態(19,)を返す。"""
    global _last_time
    imu_data, (dvl_v, dvl_cov) = get_sensor_data()
    depth = get_depth_data()

    now = time.time()  # TODO: センサヘッダ or 同期済みクロックに置き換え
    dt = 0.0 if _last_time is None else now - _last_time
    _last_time = now
    if dt <= 0.0:
        return _qekf.get_state()

    _qekf.predict(imu_data, dt)
    _qekf.integrate(imu_data, dt)

    if dvl_v is not None:
        _qekf.update_dvl(dvl_v, dvl_cov)
        _qekf.inject()
        _qekf.reset()

    if depth is not None:
        _qekf.update_depth(depth)
        _qekf.inject()
        _qekf.reset()

    return _qekf.get_state()

def normalize_deg(angle):
    """角度を -180〜180 度に正規化"""
    return (angle + 180.0) % 360.0 - 180.0

YAW_KP = 15.0        # 旋回レートPゲイン(実機で要チューニング)
YAW_RATE_MAX = 1000  # ManualControlのr(yaw)フィールドの飽和値

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
    目標座標から速度指令値を計算する。

    target_x: 目標x座標
    target_y: 目標y座標
    target_z: 目標z座標

    戻り値: (velocity_x, velocity_y, velocity_z)
      velocity_x: 目標への水平速度 (0〜1000)
      velocity_y: 横方向速度 (固定0)
      velocity_z: 上下方向速度 (500が中立, 0〜1000)
    """
    target_velocity_x = math.sqrt(target_x**2 + target_y**2) * 100
    target_velocity_y = 0
    target_velocity_z = -target_z * 100 + 500
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

POSITION_TOLERANCE = 0.2  # m, 水平方向の到達とみなす距離
DEPTH_TOLERANCE = 0.1     # m, 深度方向の到達とみなす距離

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



if __name__ == "__main__":
    initialize()
    run_control_loop()
    ManualControl(0, 0, 300, 0)

