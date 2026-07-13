import math
from pymavlink import mavutil
import sys
import time
import keyboard
import csv
import datetime

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
    target_velocity_x, target_velocity_y, target_velocity_z = VelocitySpeed(target_x, target_y, target_z, current_z)
    target_azimuth_yaw_degree = AzimuthControl(target_x, target_y)
    ManualControl(target_velocity_x, target_velocity_y, target_velocity_z, target_azimuth_yaw_degree)
    return target_velocity_x, target_velocity_y, target_velocity_z, target_azimuth_yaw_degree


VIRTUAL_MAX_SPEED = 0.3  # m/s（指令1000のときの想定速度）
LOOP_DT = 1.0


def compute_virtual_delta(cmd_x, cmd_y, cmd_z, heading_deg, dt=LOOP_DT):
    """MANUAL_CONTROL指令から仮想移動量 [m] を推定する。"""
    speed_x = (cmd_x / 1000.0) * VIRTUAL_MAX_SPEED
    speed_y = (cmd_y / 1000.0) * VIRTUAL_MAX_SPEED
    speed_z = ((cmd_z - 500) / 500.0) * VIRTUAL_MAX_SPEED
    heading = math.radians(heading_deg)
    dx = (speed_x * math.cos(heading) - speed_y * math.sin(heading)) * dt
    dy = (speed_x * math.sin(heading) + speed_y * math.cos(heading)) * dt
    dz = speed_z * dt
    return dx, dy, dz


def compute_displacement(x, y, z, start_x, start_y, start_z):
    """開始点からの移動量 [m] を返す。"""
    dx = x - start_x
    dy = y - start_y
    dz = z - start_z
    horizontal = math.sqrt(dx**2 + dy**2)
    total = math.sqrt(dx**2 + dy**2 + dz**2)
    return dx, dy, dz, horizontal, total

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


# if __name__ == '__main__':
#     import doctest
#     doctest.testmod()

target_x = 100
target_y = 100
target_z = 100
current_z = 0.0
current_yaw = 0.0

virtual_x = 0.0
virtual_y = 0.0
virtual_z = 0.0
start_x = 0.0
start_y = 0.0
start_z = 0.0
start_initialized = False

log_file = open("rov_movement_log.csv", "w", newline="", encoding="utf-8")
writer = csv.writer(log_file)
writer.writerow([
    "timestamp",
    "cmd_x", "cmd_y", "cmd_z", "cmd_yaw",
    "virtual_x", "virtual_y", "virtual_z",
    "virtual_dx", "virtual_dy", "virtual_dz",
    "virtual_horizontal_m", "virtual_total_m",
    "real_x", "real_y", "real_z", "real_total_m",
])

try:
    while True:
        cmd_x, cmd_y, cmd_z, cmd_yaw = VelocityControl(
            target_x, target_y, target_z, current_z)

        delta_x, delta_y, delta_z = compute_virtual_delta(cmd_x, cmd_y, cmd_z, cmd_yaw)
        virtual_x += delta_x
        virtual_y += delta_y
        virtual_z += delta_z

        vdx, vdy, vdz, virtual_horizontal, virtual_total = compute_displacement(
            virtual_x, virtual_y, virtual_z, start_x, start_y, start_z)

        pos_msg = BLUEROV.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        real_x = real_y = real_z = real_total = None
        if pos_msg:
            real_x = pos_msg.x
            real_y = pos_msg.y
            real_z = pos_msg.z
            current_z = real_z
            if not start_initialized:
                start_x, start_y, start_z = real_x, real_y, real_z
                start_initialized = True
            _, _, _, _, real_total = compute_displacement(
                real_x, real_y, real_z, start_x, start_y, start_z)

        att_msg = BLUEROV.recv_match(type='ATTITUDE', blocking=False)
        if att_msg:
            current_yaw = math.degrees(att_msg.yaw)

        writer.writerow([
            datetime.datetime.now().isoformat(),
            cmd_x, cmd_y, cmd_z, cmd_yaw,
            virtual_x, virtual_y, virtual_z,
            vdx, vdy, vdz,
            virtual_horizontal, virtual_total,
            real_x, real_y, real_z, real_total,
        ])
        log_file.flush()

        print(
            f"仮想移動: dx={vdx:.2f} dy={vdy:.2f} dz={vdz:.2f}m | "
            f"水平={virtual_horizontal:.2f}m 合計={virtual_total:.2f}m",
            end="",
        )
        if real_total is not None:
            print(f" | 実測合計={real_total:.2f}m")
        else:
            print(" | 実測=未取得")

        time.sleep(LOOP_DT)
except KeyboardInterrupt:
    print("\n停止")
finally:
    log_file.close()