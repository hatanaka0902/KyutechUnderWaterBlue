import os
import sys
import time

import numpy as np

# 同一ディレクトリの QEKF を確実に import する
_CONTROL_DIR = os.path.dirname(os.path.abspath(__file__))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from qekf import QEKF

from params import (
    _X0, _DX0, _P0,
    _STD_A, _STD_GYRO, _STD_DVL, _STD_DEPTH, _STD_ORIENTATION,
    _STD_A_BIAS, _STD_GYRO_BIAS,
    _DVL_OFFSET, _BAROMETER_OFFSET, _IMU_OFFSET,
    MAX_DT,
)
from common import clamp


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
    """IMU予測 + DVL更新で状態を推定し、公称状態(19,)を返す。"""
    global _last_time
    imu_data, (dvl_v, dvl_cov) = get_sensor_data()
    now = time.time()

    if _last_time is None:
        _last_time = now
        return _qekf.get_state()  # 初回はpredictをスキップ

    dt = clamp(now - _last_time, 0.0, MAX_DT)  # 負値・異常大値の両方をガード
    _last_time = now

    _qekf.predict(imu_data, dt)
    _qekf.integrate(imu_data, dt)

    if dvl_v is not None:
        _qekf.update_dvl(dvl_v, dvl_cov)
        _qekf.inject()
        _qekf.reset()

    return _qekf.get_state()
