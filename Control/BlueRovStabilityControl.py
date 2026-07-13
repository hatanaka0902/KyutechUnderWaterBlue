import math
from pymavlink import mavutil
import sys
import time
import numpy as np
import keyboard
import csv
import datetime

# 初期化用の関数
def initialize():
    ## キャリブレーション用のcode
    ## センサデータ取得用の関数を初期化
    return 0

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


# ---------- センサ取得(元コードのバグ修正版) ----------
def get_acceleration_data():
    raise NotImplementedError  # 実センサ読み出しに置き換え

def get_gyro_data():
    raise NotImplementedError

def get_position_data():
    raise NotImplementedError
# ----------------------------------------------------

def get_sensor_data():
    def get_imu_data():
        # u[0] = [ax, ay, az](m/s^2), u[1] = [gx, gy, gz](rad/s)
        accel = get_acceleration_data()
        gyro = get_gyro_data()
        return np.array([accel, gyro])  # shape (2,3)

    DVL_COVARIANCE_BODY = np.diag([0.02, 0.02, 0.03]) ** 2  # DVLスペックシートから設定

    def get_dvl_data():
        # return (velocity_body(3,), covariance(3,3)) / 信号途絶時は (None, None)
        v_body = get_velocity_data()
        return v_body, DVL_COVARIANCE_BODY

    return get_imu_data(), get_dvl_data()


# ---------- 状態推定(EKFを使った実装) ----------

_qekf = QEKF(x_0, dx_0, P_0, std_a, std_gyro, std_dvl, std_depth,
             std_orientation, std_a_bias, std_gyro_bias,
             dvl_offset, barometer_offset, imu_offset)
_last_time = None

def state_estimation():
    global _last_time
    imu_data, (dvl_v, dvl_cov) = get_sensor_data()
    depth = get_depth_data()

    now = time.time()
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


def AzimuthControl(target_x, target_y):
    """
    目標座標 (target_x, target_y) へのyaw角を返す（度）
    x軸正方向を0度として反時計回りを正とする
    返り値は 0 ~ 360 の範囲に正規化

    """
    target_azimuth_yaw_degree = math.degrees(math.atan2(target_y, target_x))

    return target_azimuth_yaw_degree

def VelocitySpeed(target_x, target_y, target_z,current_z):
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
    target_velocity_x = math.sqrt(target_x**2 + target_y**2 + target_z**2) * 100
    target_velocity_y = 0
    target_velocity_z = (target_z-current_z) * 100 + 500
    return target_velocity_x, target_velocity_y, target_velocity_z

def VelocityControl(target_x, target_y, target_z,current_z):
    '''
    target_x: 目標x座標
    target_y: 目標y座標
    target_z: 目標z座標
    戻り値: (velocity_x, velocity_y, velocity_z)
      velocity_x: 目標への水平速度 (0〜1000)
      velocity_y: 横方向速度 (固定0)
      velocity_z: 上下方向速度 (500が中立, 0〜1000)
      azimuth_yaw_degree: 目標へのyaw角 (0〜360)
    '''
    target_velocity_x, target_velocity_y, target_velocity_z = VelocitySpeed(target_x, target_y, target_z, current_z)
    target_azimuth_yaw_degree = AzimuthControl(target_x, target_y)
    ManualControl(target_velocity_x, target_velocity_y, target_velocity_z, target_azimuth_yaw_degree)
    return target_velocity_x, target_velocity_y, target_velocity_z, target_azimuth_yaw_degree

def get_target_position():
    # return [x, y, z](m)
    return [0, 0, 0]



# def pi_Control():
#     attitude_control()
#     velocity_control()
Flag = True
while Flag:
    current_state = state_estimation()
    if EmergencyProblem(current_state):
        Flag = False
        break
    # 目標位置取得用の関数->[x, y, z](m)
    target_position = get_target_position()

    VelocityControl(target_position[0], target_position[1], target_position[2], current_state[2])

