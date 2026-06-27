import math
from pymavlink import mavutil
import sys
import time
import keyboard

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


# # フライトモード変更
# ChangeMode("MANUAL")
# 制御権取得
Arm()
# カメラチルト
CameraTilt(45*100)
time.sleep(0.5)
CameraTilt(-45*100)
time.sleep(0.5)
CameraTilt(0*100)
time.sleep(0.5)
# LED点灯
for brightness in range(1100, 1900, 10):
    SetPwm(7, brightness)
    time.sleep(0.01)
for brightness in range(1900, 1100, -10):
    SetPwm(7, brightness)
    time.sleep(0.01)
# グリッパー開閉
SetPwm(11, 1900)
time.sleep(2)
SetPwm(11, 1100)
time.sleep(2)
# 前進
ManualControl(200, 0, 500, 0)
time.sleep(1)
# 後進
ManualControl(-200, 0, 500, 0)
time.sleep(1)
# 浮上
ManualControl(0, 0, 700, 0)
time.sleep(1)
# 潜水
ManualControl(0, 0, 300, 0)
time.sleep(1)
# 停止
ManualControl(0, 0, 500, 0)

def AzimuthControl(target_x, target_y):
    """
    目標座標 (target_x, target_y) へのyaw角を返す（度）
    x軸正方向を0度として反時計回りを正とする
    返り値は 0 ~ 360 の範囲に正規化

    >>> AzimuthControl(1, 0)
    0.0
    >>> AzimuthControl(0, 1)
    90.0
    >>> AzimuthControl(-1, 1)
    135.0
    >>> AzimuthControl(-1, 0)
    180.0
    >>> AzimuthControl(1, 1)
    45.0
    >>> AzimuthControl(0, -1)
    -90.0
    >>> AzimuthControl(0, 0)
    0.0
    >>> AzimuthControl(1, -1)
    135.0
    >>> AzimuthControl(-1, -1)
    -45.0
    >>> AzimuthControl(-1, 1)
    -135.0
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

    >>> VelocityControl(0, 0, 0)
    (0.0, 0, 500)
    >>> VelocityControl(1, 0, 0)
    (100.0, 0, 500)
    >>> VelocityControl(3, 4, 0)
    (500.0, 0, 500)
    >>> VelocityControl(0, 0, 1)
    (100.0, 0, 600)
    >>> VelocityControl(0, 0, -1)
    (100.0, 0, 400)
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
    >>> VelocityControl(0, 0, 0, 0)
    (0.0, 0, 500)
    >>> VelocityControl(1, 0, 0, 0)
    (100.0, 0, 500)
    >>> VelocityControl(0, 1, 0, 0)
    (100.0, 0, 500)
    >>> VelocityControl(0, 0, 1, 0)
    (100.0, 0, 600)
    >>> VelocityControl(0, 0, -1, 0)
    (100.0, 0, 400)
    >>> VelocityControl(1, 1, 0, 0)
    (141.4213562373095, 0, 500)
    >>> VelocityControl(1, 1, 1, 0)
    (141.4213562373095, 0, 600)
    >>> VelocityControl(1, 1, -1, 0)
    (141.4213562373095, 0, 400)
    >>> VelocityControl(1, 1, 0, 1)
    (141.4213562373095, 0, 500)
    >>> VelocityControl(1, 1, 0, -1)
    (141.4213562373095, 0, 500)
    >>> VelocityControl(1, 1, 1, 1)
    (141.4213562373095, 0, 600)
    >>> VelocityControl(1, 1, 1, -1)
    (141.4213562373095, 0, 600)
    '''
    target_velocity_x, target_velocity_y, target_velocity_z = VelocitySpeed(target_x, target_y, target_z,current_z)
    target_azimuth_yaw_degree = AzimuthControl(target_x, target_y)
    ManualControl(target_velocity_x, target_velocity_y, target_velocity_z, target_azimuth_yaw_degree)
    return 0

if __name__ == '__main__':
    import doctest
    doctest.testmod()
