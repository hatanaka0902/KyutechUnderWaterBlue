import os
os.environ['MAVLINK20'] = '1'  # MAVLink 2.0 強制
from pymavlink import mavutil
import time

# 1. 接続と初期化
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.source_system = 255  # GCSとして認識させる
master.wait_heartbeat()

# 2. テレメトリ要求送信
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

# 3. 安全ロック解除 (ARM命令)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, # 1=ARM, 0=DISARM
    0, 0, 0, 0, 0, 0
)

# # 4. 操作信号の送信ループ (例: 前進+浮上)
# rc_inputs = [1500] * 18
# rc_inputs[4] = 1600  # RC5: 前進 (Forward)
# rc_inputs[2] = 1550  # RC3: 上下推進 (Heave)

# try:
#     while True:
#         # GCSとして生存信号を送信
#         master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

#         # 操縦信号のオーバーライド送信
#         master.mav.rc_channels_override_send(
#             master.target_system, master.target_component,
#             *rc_inputs
#         )
#         time.sleep(0.1)  # 10Hz周期で送信を維持
# except KeyboardInterrupt:
#     pass

print("Sensor data saved to sensor_data.csv")