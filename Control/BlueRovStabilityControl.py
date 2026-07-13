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

class ImuDvlEkf:
    G = np.array([0.0, 0.0, 9.81])  # NED: 重力はz正方向(下向き)

    def __init__(self, x0=None, P0=None, Q=None, R_dvl=None):
        self.x = np.zeros(9) if x0 is None else np.asarray(x0, dtype=float)
        self.P = np.eye(9) * 0.1 if P0 is None else np.asarray(P0, dtype=float)

        # プロセスノイズ(IMUのノイズ特性に応じて要チューニング)
        self.Q = np.diag([
            1e-4, 1e-4, 1e-4,   # 位置
            1e-5, 1e-5, 1e-5,   # 姿勢
            1e-3, 1e-3, 1e-3,   # 速度
        ]) if Q is None else np.asarray(Q, dtype=float)

        # 観測ノイズ(DVLの位置精度に応じて要チューニング)
        self.R_dvl = np.diag([0.05, 0.05, 0.08]) ** 2 if R_dvl is None else np.asarray(R_dvl, dtype=float)

    # ---------- 運動学モデル ----------
    @staticmethod
    def _euler_rate_matrix(roll, pitch):
        """機体角速度[p,q,r] -> オイラー角速度[roll_dot,pitch_dot,yaw_dot]
        pitch=±90degで特異点(ジンバルロック)。フルレンジの姿勢変化が
        想定される場合はクォータニオン状態への変更を検討してください。
        """
        cr, sr = np.cos(roll), np.sin(roll)
        cp, tp = np.cos(pitch), np.tan(pitch)
        return np.array([
            [1.0, sr * tp, cr * tp],
            [0.0, cr, -sr],
            [0.0, sr / cp, cr / cp],
        ])

    @staticmethod
    def _rotation_matrix(roll, pitch, yaw):
        """機体座標系 -> world座標系 (R = Rz(yaw) Ry(pitch) Rx(roll))"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    def _f(self, x, imu_data, dt):
        """状態遷移関数 f(x, u, dt) -> x_next"""
        p, euler, v = x[0:3], x[3:6], x[6:9]
        accel_b, omega_b = imu_data[0:3], imu_data[3:6]

        R = self._rotation_matrix(*euler)
        Wr = self._euler_rate_matrix(euler[0], euler[1])

        p_next = p + v * dt
        euler_next = euler + (Wr @ omega_b) * dt
        a_world = R @ accel_b + self.G
        v_next = v + a_world * dt

        return np.concatenate([p_next, euler_next, v_next])

    def _jacobian_f(self, x, imu_data, dt, eps=1e-6):
        """中心差分によるヤコビアン F = df/dx (9x9)"""
        n = len(x)
        F = np.zeros((n, n))
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = eps
            F[:, i] = (self._f(x + dx, imu_data, dt) - self._f(x - dx, imu_data, dt)) / (2 * eps)
        return F

    @staticmethod
    def _wrap_angles(x):
        x[3:6] = (x[3:6] + np.pi) % (2 * np.pi) - np.pi
        return x

    # ---------- 予測ステップ(IMU) ----------
    def predict(self, imu_data, dt):
        """imu_data: [ax, ay, az, gx, gy, gz] (get_imu_data()の出力そのまま)"""
        imu_data = np.asarray(imu_data, dtype=float)
        if dt <= 0.0:
            return  # 初回呼び出し等、dtが取れない場合は予測をスキップ

        F = self._jacobian_f(self.x, imu_data, dt)
        self.x = self._wrap_angles(self._f(self.x, imu_data, dt))
        self.P = F @ self.P @ F.T + self.Q * dt

    # ---------- 更新ステップ(DVL) ----------
    def update_dvl(self, dvl_data):
        """dvl_data: [x, y, z] (get_dvl_data()の出力そのまま、位置観測)"""
        z = np.asarray(dvl_data[:3], dtype=float)
        H = np.zeros((3, 9))
        H[0:3, 0:3] = np.eye(3)

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_dvl
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self._wrap_angles(self.x + K @ y)
        self.P = (np.eye(9) - K @ H) @ self.P

    def get_state(self):
        """[x, y, z, roll, pitch, yaw, vx, vy, vz]"""
        return self.x.copy()

    def get_covariance(self):
        """収束判定(確信度)に使う共分散行列"""
        return self.P.copy()


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
        # return [ax, ay, az, gx, gy, gz](m/s^2, rad/s)
        imu_data = np.zeros(6)
        imu_data[:3] = get_acceleration_data()
        imu_data[3:] = get_gyro_data()

        return imu_data

    def get_dvl_data():
        # return [x, y, z](m)
        dvl_data = np.zeros(3)
        dvl_data[:3] = get_position_data()

        return dvl_data

    return get_imu_data(), get_dvl_data()


# ---------- 状態推定(EKFを使った実装) ----------
_ekf = ImuDvlEkf()
_last_time = None

def state_estimation():
    global _last_time
    imu_data, dvl_data = get_sensor_data()

    now = time.time()
    dt = 0.0 if _last_time is None else now - _last_time
    _last_time = now

    _ekf.predict(imu_data, dt)

    if dvl_data is not None:
        _ekf.update_dvl(dvl_data)

    current_state = _ekf.get_state()
    return current_state


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


# def pi_Control():
#     attitude_control()
#     velocity_control()

# 目標位置取得用の関数->[x, y, z](m)
target_position = get_target_position()
# 目標速度取得用の関数->[x, y, z](m/s)
target_velocity = get_target_velocity()
current_state = state_estimation()

VelocityControl(target_position[0], target_position[1], target_position[2], current_state[2])

