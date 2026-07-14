# KyutechUnderWaterBlue
## センサから得られる値は，すべて各時刻でのロボットから見た相対値とする
- IMU
- DVL
- マイクロフォン


カルマンフィルタ利用github
https://github.com/bjornrho/Navigation-brov2/tree/main

## Control/ 関数一覧(役割・依存関係)

モジュール間の依存方向(左が右に依存されうる):

```
params.py, common.py, qekf.py, utility_functions.py, imu_stream.py   (最下層・相互依存なし)
        ↓                              ↓
function.py (params, qekf,  pid.py (common)     mavlink_io.py (params)
    utility_functions, imu_stream)
        ↓                        ↓                     ↓
        └──────────── controlfunction.py (params, common, function, pid, mavlink_io, utility_functions) ──────────┘
                              ↓
                           main.py (function, controlfunction, mavlink_io)
```

### params.py — 定数のみ
QEKF初期パラメータ、到達判定閾値(`POSITION_TOLERANCE`/`DEPTH_TOLERANCE`/`MAX_DT`)、カスケードPID用ゲイン辞書
(`PID_{SURGE,HEAVE,YAW}_{OUTER,INNER}`、`pid.PID(**...)`にそのまま渡す形式)、MAVLink接続文字列。関数なし、他モジュールに依存しない。

### common.py — 汎用ヘルパー
| 関数 | 役割 | 依存 |
|---|---|---|
| `clamp(value, lo, hi)` | 値を `[lo, hi]` にクランプ | なし |
| `normalize_deg(angle)` | 角度を -180〜180 度に正規化 | なし |

### pid.py — PID制御器
`common.clamp` にのみ依存。`output_limits`(出力クランプ)、`windup_limit`(積分項アンチワインドアップ用クランプ)、
`angle_error_deg`(yaw等の角度誤差を-180〜180度に正規化するフラグ)を持つ`PID`クラス。`dt`は毎回外部から渡す(内部で
`time.time()`を呼ばない)。

| メソッド | 役割 | 依存 |
|---|---|---|
| `PID(kp, ki, kd, output_limits=None, windup_limit=None, angle_error_deg=False)` | ゲイン・出力クランプ・アンチワインドアップ・角度正規化の設定、`reset()`呼び出し | `common.clamp` |
| `update(setpoint, measurement, dt)` | setpoint/measurementから誤差を計算して更新(速度PIDなど) | なし |
| `update_from_error(error, dt)` | 誤差が上流で計算済みの場合に使用(位置PIDなど) | なし |
| `reset()` | 積分項・前回誤差・前回出力をクリア | なし |

### function.py — センサ取得〜状態推定
モジュール読み込み時に`_qekf = create_qekf()`と並べて`_imu_stream = ImuStream()` / `_imu_stream.start()`を実行し、
IMU受信用TCPサーバをバックグラウンドで起動する。

| 関数 | 役割 | 依存 |
|---|---|---|
| `create_qekf(...)` | `QEKF` インスタンス生成(省略時は `params.py` の初期値) | `qekf.QEKF`, `params` |
| `initialize(qekf=None)` | フィルタ初期化、モジュール変数 `_qekf`/`_last_time` をリセット | `create_qekf` |
| `get_acceleration_data()` | 比力 [ax,ay,az] (m/s^2, body) を`ImuStream`から取得。途絶時は`RuntimeError` | `imu_stream.ImuStream` |
| `get_gyro_data()` | 角速度 [gx,gy,gz] (rad/s, body) を`ImuStream`から取得。途絶時は`RuntimeError` | `imu_stream.ImuStream` |
| `get_depth_data()` | 気圧深度 [m] (NED z) 取得(**未実装スタブ**)。DVEXTの`altitude`(底面までの距離)とは別物 | なし |
| `get_position_data()` | 位置取得用スタブ(**未実装**。現状QEKFはDVL速度更新のみ使用しており未使用) | なし |
| `parse_dvext(sentence)` | `$DVEXT`のNMEA文字列をパース(チェックサム検証込み)、dictまたはNoneを返す | なし |
| `get_latest_dvext()` | DVL-75からの最新`$DVEXT`文字列を受信して`parse_dvext()`に通す(**未実装スタブ**、シリアル/Ethernet受信部) | なし |
| `get_velocity_data()` | DVL対地速度を機体座標系で返す。DVLロスト時は`None` | `get_latest_dvext`, `utility_functions.quaternion_to_rotation_matrix` |
| `get_orientation_measurement()` | BNO08xのクォータニオンを`QEKF.update_orientation()`用に返す。取得不可なら`None` | `imu_stream.ImuStream` |
| `get_sensor_data()` | IMU+DVLをQEKF入力形式にまとめる | 上記 `get_acceleration_data`/`get_gyro_data`/`get_velocity_data` |
| `state_estimation()` | predict→integrate→DVL更新→姿勢更新→深度更新(各々inject→reset)を1周期実行し公称状態を返す | `get_sensor_data`, `get_orientation_measurement`, `get_depth_data`, `common.clamp`, `params.MAX_DT`, `qekf.QEKF` |
| `get_last_dt()` | 直近の`state_estimation()`で使われた`dt`を返す(`controlfunction.py`のPIDループでの再利用用) | なし |
| `get_last_imu_data()` | 直近の`state_estimation()`で使われたIMU生データ(shape (2,3))を返す | なし |

### controlfunction.py — 誘導則・カスケードPID制御ループ
外側ループ(位置/yaw誤差→速度setpoint)と内側ループ(速度setpoint→推力/トルク指令)からなるカスケードPID構成。
モジュール読み込み時に`params.PID_*`辞書から6つの`PID`インスタンス(`_surge_outer_pid`/`_heave_outer_pid`/`_yaw_outer_pid`/
`_surge_inner_pid`/`_heave_inner_pid`/`_yaw_inner_pid`)を生成する。旧来の単段制御関数(`YawRateControl`/`VelocitySpeed`/
`VelocityControl`)はこのカスケード構成への移行に伴い削除済み。

| 関数 | 役割 | 依存 |
|---|---|---|
| `EmergencyProblem(current_state)` | 緊急停止判定(現状は深度 `current_state[2]` < 0.1m のみ、壁距離/センサ異常は未実装) | なし |
| `AzimuthControl(target_x, target_y)` | 目標座標(NED)への方位角[度](-180〜180)を算出 | なし |
| `OuterLoopControl(dx, dy, dz, current_yaw_deg, dt)` | 位置/yaw誤差から外側ループの速度setpoint(forward speed, NED heave rate, yaw rate)を算出 | `AzimuthControl`, `_surge_outer_pid`/`_heave_outer_pid`/`_yaw_outer_pid` |
| `get_target_position()` | 目標位置取得(**現状は固定 `[0,0,0]`(NEDオフセット)のスタブ**) | なし |
| `is_target_reached(target_x, target_y, target_z)` | 目標到達判定(水平距離・深度差がそれぞれ閾値未満か) | `params.POSITION_TOLERANCE`/`DEPTH_TOLERANCE` |
| `get_body_frame_velocity(current_state)` | QEKFのNED速度を機体座標系に回転 | `utility_functions.quaternion_to_rotation_matrix` |
| `get_yaw_rate_deg(current_state)` | 生ジャイロ値 - QEKF推定ジャイロバイアスからyawレート[deg/s]を算出 | `function.get_last_imu_data` |
| `InnerLoopControl(surge_sp, heave_sp, yaw_rate_sp, current_state, dt)` | 外側ループのsetpointと実測値(上記2関数)の誤差から`ManualControl`向けの`(x_cmd, z_cmd_offset, r_cmd)`を算出 | `get_body_frame_velocity`, `get_yaw_rate_deg`, `_surge_inner_pid`/`_heave_inner_pid`/`_yaw_inner_pid` |
| `reset_all_pids()` | 6つのPIDインスタンスをすべてリセット(目標到達によるホールド突入時などに呼ぶ) | なし |
| `run_control_loop()` | メイン制御ループ(状態推定→緊急判定→目標到達判定→ホールド or 外側/内側PIDループ→`ManualControl`送信を繰り返す) | `function.state_estimation`/`get_last_dt`, `EmergencyProblem`, `get_target_position`, `is_target_reached`, `OuterLoopControl`, `InnerLoopControl`, `reset_all_pids`, `mavlink_io.ManualControl`, `utility_functions.quaternion_to_euler` |

### mavlink_io.py — MAVLink低レベルI/O
モジュール読み込み時に `BLUEROV` 接続を確立し、heartbeatをブロッキング待機(実機/SITL必須)。

| 関数 | 役割 | 依存 |
|---|---|---|
| `Arm()` | 機体をアーム | `BLUEROV` 接続 |
| `Disarm()` | 機体をディスアーム | `BLUEROV` 接続 |
| `ChangeMode(mode)` | 操縦モード変更(`MANUAL`/`STABILIZE`/`ALT_HOLD`) | `BLUEROV` 接続 |
| `SetPwm(channel_id, pwm)` | RCチャンネルPWM出力 | `BLUEROV` 接続 |
| `ManualControl(x, y, z, yaw)` | 手動操縦指令を送信。`x`/`y`は-1000〜1000、`z`は0〜1000(500が中立、値が小さいほど沈む方向)、`yaw`は-1000〜1000 | `BLUEROV` 接続 |
| `CameraTilt(tilt, roll, pan)` | カメラジンバル姿勢制御 | `BLUEROV` 接続 |
| `GainUp()` / `GainDown()` | 操作ゲイン調整(**既知バグ**: グローバル `GAIN` が未初期化のため呼ぶと`NameError`) | なし |

### main.py — エントリポイント
`mavlink_io` → `function` → `controlfunction` の順にimportし(MAVLink接続を最初に確立)、`initialize()` → `run_control_loop()`
→ 停止用`ManualControl`を実行。

### qekf.py — QEKFクラス
`utility_functions.py` に依存。誤差状態クォータニオンEKFの本体。

| メソッド | 役割 |
|---|---|
| `__init__` / `filter_reset` | 公称状態・誤差状態・共分散・ノイズパラメータの初期化 |
| `integrate(u, dt)` | IMU入力を公称状態(位置・速度・姿勢)に積分 |
| `predict(u, dt)` | 誤差状態の共分散を伝播 |
| `update_orientation(q)` | 姿勢観測による補正(`function.state_estimation()`から毎周期呼ばれる) |
| `update_depth(depth)` | 深度観測による補正(`function.state_estimation()`から毎周期呼ばれる) |
| `update_dvl(v, cov)` | DVL速度観測による補正(`function.state_estimation()`から毎周期呼ばれる) |
| `inject()` | 誤差状態を公称状態へ反映 |
| `reset()` | 誤差状態・共分散をリセット |
| `get_state` / `get_position` / `get_velocity` / `get_quaternion` / `get_covariance` | 状態取得用アクセサ |

### imu_stream.py — IMUストリーム受信
Adafruit BNO08x搭載ESP32(`IMU_CALIB_AND_REC.ino`)からTCP経由でIMUデータを受信するモジュール。このモジュールがサーバとして
ESP32からの接続を待ち受け、接続確立後に`"MEASURE"`コマンドを送ってCSV形式(Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Qx,Qy,Qz,Qw、
末尾`\n`)のデータ行送信を開始させる。受信はバックグラウンドスレッドで行い、直近値をロックで保護しつつ保持する
(制御ループから`get_latest()`で毎周期ポーリングする用途)。姿勢クォータニオンはESP32側`Qx,Qy,Qz,Qw`の順序から
QEKFが使う`[qw,qx,qy,qz]`の順に並べ替えて保持する。ESP32の送信は姿勢推定(`SH2_ROTATION_VECTOR`)イベント発火時のみ
行われるため、姿勢推定が止まるとIMUデータ全体(加速度・角速度含む)が途絶する点に注意。

| メソッド | 役割 | 依存 |
|---|---|---|
| `ImuStream(listen_ip="0.0.0.0", listen_port=5007, stale_timeout=0.5)` | 待受アドレス・鮮度タイムアウトの設定 | なし |
| `start()` | バックグラウンドスレッドでTCPサーバを起動し、ESP32の接続を待つ | `socket`, `threading` |
| `stop()` | サーバを停止 | なし |
| `get_latest()` | 直近のIMU値を`dict`(`accel`/`gyro`/`mag`/`quat`)で返す。`stale_timeout`を超えて更新が無ければ`None` | なし |

### utility_functions.py — クォータニオン/回転演算(変更なし・純粋関数)
| 関数 | 役割 |
|---|---|
| `cross(v)` | 外積行列(skew-symmetric matrix) |
| `quaternion_to_euler(q)` | クォータニオン→オイラー角(roll, pitch, yaw) |
| `quaternion_to_rotation_matrix(q)` | クォータニオン→回転行列 |
| `quaternion_product(q1, q2)` | クォータニオンのHamilton積 |
| `rotation_vector_to_quaternion(vector, angle)` | 回転ベクトル→クォータニオン(指数写像) |
| `rodrigues(omega, angle)` | Rodriguesの回転公式による回転行列 |

### BlueRovSpeedControl.py — 旧プロトタイプ(手動キーボード操縦スクリプト)
`controlfunction.py`のカスケードPID制御とは別系統で、PID/QEKFを使わない独立した手動操縦デモ。`Arm`→カメラチルト/LED/
グリッパー/前後上下移動のデモ動作→WASD+矢印キーによるキーボード操縦ループ(`keyboard`パッケージ使用)、という構成。
モジュール読み込み時に無条件で独自のMAVLink接続(`udpin:192.168.2.2:14550`。末尾が`params.MAVLINK_CONNECTION_STRING`
と異なる)を確立し、トップレベルのコードが`if __name__`等で保護されずそのまま実行される。他モジュールから未使用
(参考・履歴用途のみ)。
