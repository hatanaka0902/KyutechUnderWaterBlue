from mimetypes import init
from operator import truediv
from sensor import ImuStream


def FlagControl(params):
    """
    params: 初期化時のパラメータ
    """
    if EmergencyProblem(state):
        return True
    if timeControl(initial_params, time, deadline_time):
        return True
    return False

def SensorGetData():
    """
    センサから得られた値を返す
    """
    return hydrophone_data_yaw,hydrophone_data_pitch



if __name__ == "__main__":
    ini
    initial_params =initialControl()
    Now_filename = initialSetting()
    stream = ImuStream()   # function.py ではモジュール読込時に start() 済み
    stream.start()
    latest_imu_data = stream.get_latest()  # dict または None

    while Flag:
        sensor_data = SensorGetData()
        save_sensor_data(sensor_data)
        if FlagControl(params,sensor_data):
            Flag = False
            break
        hydrophone_data_yaw,hydrophone_data_pitch = hydrophoneSensorControl()
        if GetSerachFloatingBall(hydrophone_data_pitch,hydrophone_data_yaw,target_FloatingBall_pitch,target_FloatingBall_yaw):
           # BlueRovがビンガーに近づいているかのチェック関数
           GetBinger_Flag = True

        if GetBinger_Flag:
            target_position_x,target_position_y,target_position_z = BingerSearch(hydrophone_data_pitch,hydrophone_data_yaw,params.assumed_distance)



