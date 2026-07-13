import math
from pymavlink import mavutil
import sys
import time
import keyboard
import csv
import datetime

# 初期化用の関数
def initialize():
    ## キャリブレーション用のcode
    ## センサデータ取得用の関数を初期化
    get_sensor_data()
    ## 状態推定用の関数を初期化
    state_estimation()
    ## 姿勢制御用の関数を初期化
    attitude_control()
    ## 速度制御用の関数を初期化
    velocity_control()
    ## PI制御用の関数を初期化
    pi_Control()

"""
IMU/DVLセンサ融合による状態推定モジュール(EKF)

状態ベクトル: x = [px, py, pz, roll, pitch, yaw, vx, vy, vz]
    - 位置      px, py, pz [m]   (NED座標系, z軸下向き。Fossenモデルとの整合を優先)
    - 姿勢      roll, pitch, yaw [rad] (ZYXオイラー角)
    - 速度      vx, vy, vz [m/s] (NED座標系)

前提:
    - IMU加速度は比力(specific force)として与えられる(重力成分を含まない)
    - DVLは対地"位置"を観測する(get_dvl_data()の命名に合わせた実装)
      ※ 一般的なDVLは対地"速度"を出力することが多いです。速度観測の場合は
        update_dvl() の H 行列を速度成分(6:9)に差し替えてください。
    - ENU(z軸上向き)を使う場合は G の符号を反転してください。

DVL途絶時のデッドレコニング:
    predict() のみを呼び、update_dvl() を呼ばなければ、その周期は
    IMU積分のみの予測結果がそのまま状態推定値として使われます。
    get_dvl_data() 側でタイムアウトやチェックサム異常を検知したら
    None を返すよう改修し、state_estimation() 側で None のときは
    update_dvl() をスキップしてください。

分散システムでの時刻同期に関する注意:
    Zenoh経由でDesktop/notebook/lab PC間で時刻がずれていると dt の計算が
    狂い、予測ステップが不安定になります。dt はローカルの time.time() では
    なく、センサメッセージのヘッダタイムスタンプ(またはPTP等で同期した
    ROS2クロック)から計算することを推奨します。
"""

import time
import numpy as np


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

def get_sensor_data():
    def get_imu_data():
        imu_data = np.zeros(6)
        imu_data[:3] = get_acceleration_data()
        imu_data[3:] = get_gyro_data()
        return imu_data

    def get_dvl_data():
        dvl_data = np.zeros(3)
        dvl_data[:3] = get_position_data()
        # TODO: 信号タイムアウト/チェックサム異常時は None を返す
        return dvl_data

    return get_imu_data(), get_dvl_data()


# ---------- 状態推定(EKFを使った実装) ----------
_ekf = ImuDvlEkf()
_last_time = None

def state_estimation():
    global _last_time
    imu_data, dvl_data = get_sensor_data()

    now = time.time()  # TODO: センサヘッダのタイムスタンプ or 同期済みROS2クロックに置き換え
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


def pi_Control():
    attitude_control()
    velocity_control()

# 目標位置取得用の関数->[x, y, z](m)
target_position = get_target_position()
# 目標速度取得用の関数->[x, y, z](m/s)
target_velocity = get_target_velocity()

VelocityControl(target_position[0], target_position[1], target_position[2], current_state[2])

