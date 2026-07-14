import numpy as np

# ---------- MAVLink 接続 ----------
MAVLINK_CONNECTION_STRING = 'udpin:192.168.2.1:14550'

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

# ---------- 制御ゲイン・閾値 ----------
# 旧・単段制御用(手順6でカスケードPIDに置き換え後、削除予定)
# TODO: controlfunction.py の YawRateControl/VelocitySpeed を
#       カスケードPID(PID_YAW_*, PID_HEAVE_*, PID_SURGE_*)に置き換えたら
#       この5つは不要になるので削除する
YAW_KP = 15.0        # 旋回レートPゲイン(実機で要チューニング)
YAW_RATE_MAX = 1000  # ManualControlのr(yaw)フィールドの飽和値

VEL_XY_MAX = 1000        # ManualControlのx,yフィールドの飽和値(-1000〜1000)
VEL_Z_MIN, VEL_Z_MAX = 0, 1000  # zフィールドの範囲(500が中立)

POSITION_TOLERANCE = 0.2  # m, 水平方向の到達とみなす距離
DEPTH_TOLERANCE = 0.1     # m, 深度方向の到達とみなす距離

MAX_DT = 0.5  # s, これを超えるdtは発散防止のためクランプ(要チューニング)

PID_YAW_OUTER = dict(
    kp=0.5, ki=0.0, kd=0.0,
    output_limits=(-20.0, 20.0),   # deg/s。初回は控えめな最大旋回速度から
    windup_limit=40.0,             # Ki=0の間は無効。後でIを入れる時のため仮置き
    angle_error_deg=True,          # 誤差を-180~180に正規化(角度なので必須)
)

PID_YAW_INNER = dict(
    kp=20.0, ki=0.0, kd=0.0,
    output_limits=(-500.0, 500.0),  # rコマンド。フルレンジ(±1000)よりまず控えめに
    windup_limit=25.0,
    angle_error_deg=False,          # 誤差は既にdeg/s(角度ではない)ので正規化不要
)

PID_HEAVE_OUTER = dict(
    kp=0.3, ki=0.0, kd=0.0,
    output_limits=(-0.3, 0.3),      # m/s。小型ROVの初回テストとして控えめな昇降速度
    windup_limit=0.6,
    angle_error_deg=False,
)

PID_HEAVE_INNER = dict(
    kp=800.0, ki=0.0, kd=0.0,
    output_limits=(-300.0, 300.0),  # 500中立からのオフセット量(送信時に+500する)
    windup_limit=0.4,
    angle_error_deg=False,
)

PID_SURGE_OUTER = dict(
    kp=0.15, ki=0.0, kd=0.0,
    output_limits=(0.0, 0.3),       # m/s。距離は常に0以上なので下限も0
    windup_limit=0.6,
    angle_error_deg=False,
)

PID_SURGE_INNER = dict(
    kp=1000.0, ki=0.0, kd=0.0,
    output_limits=(0.0, 400.0),     # xコマンド。初回は後退方向を使わない(0始まり)
    windup_limit=0.4,
    angle_error_deg=False,
)