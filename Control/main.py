from mimetypes import init
from operator import truediv
from sensor import ImuStream


def FlagEmergencyControl(params):
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

    initial_params =initialControl()
    Now_filename = initialSetting()
    stream = ImuStream()   # function.py ではモジュール読込時に start() 済み
    stream.start()
    latest_imu_data = stream.get_latest()  # dict または None
    Flag_Emergency = False
    Flag_Search_Binger = False
    Flag_Image_Processing = False
    Binger_Attack = False
    Flag_Get_Binger = False
    get_binger_pitch_position = 30
    Binger_Try_Attack = False


    while not (Flag_Emergency or Flag_Get_Binger):
        # 最初にセンサからデータを取得
        # --------------------------------
        sensor_data = SensorGetData()
        # データを保存
        # --------------------------------
        save_sensor_data(sensor_data)
        # 緊急制御のフラグをチェック
        # --------------------------------
        if FlagEmergencyControl(params,sensor_data):
            Flag_Emergency = True
            break
        # ビンガーの位置を取得
        # --------------------------------
        hydrophone_data_yaw,hydrophone_data_pitch = hydrophoneSensorControl()
        if GetSerachFloatingBall(hydrophone_data_pitch,hydrophone_data_yaw,target_FloatingBaller_pitch,target_FloatingBaller_yaw):
           # BlueRovがビンガーに近づいているかのチェック関数
           Flag_Search_Binger = True

        if Flag_Search_Binger and Flag_Image_Processing and not Flag_Get_Binger:
            # 画像処理が利用できる場合の処理
            # --------------------------------
        else:
            # 画像処理が利用できない場合の処理

        if Binger_Attack:
            # Binger衝突の処理
            # --------------------------------
            # ホバリング
            # --------------------------------
            # Bingerの位置がどこかを取得
            if hydrophone_data_pitch > get_binger_pitch_position:
            # Bingerが一定時間上にあるかのチェック
                GetBinger_Flag = True

            else:
                Binger_Try_Attack = True
        if Binger_Try_Attack or Flag_Search_Binger:
        

