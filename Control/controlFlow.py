


def initialControl():
    """
    起動時の内部時間を計測する

    """
    start_time = time.time()
    return start_time

def timeControl(start_time, time,deadline_time):
    """
    返し値はTrueの場合は時間が期限を超えている場合はFalseの場合は時間が期限を超えていない場合である
    start_time: 起動時の内部時間 [s]
    time: 現在の内部時間 [s]
    deadline_time: 期限時間 [s]

    """
    if time - start_time < deadline_time:
        return False
    return True

def EmergencyProblem(current_state):
    """
    current_state: QEKFの公称状態(19要素のnp.ndarray, function.state_estimation()の戻り値)。
    現在は current_state[2] (position z [m], NED, 正=下方向) のみで判定する。
    """
    # 現在の深度が0.1m以下の場合は緊急問題としてTrueを返す
    if current_state[2] < 0.1:
        return True
    # 壁との距離が0.5m以下の場合は緊急問題としてTrueを返す

    # センサが異常な場合は緊急問題としてTrueを返す

    return False

def GetSerachFloatingBall(hydrophone_data_pitch,hydrophone_data_yaw,target_FloatingBall_pitch,target_FloatingBall_yaw):
    """
    ハイドロフォンから得られた値のpitchが一定以下の場合はTrueを返す
    hydrophone_data_pitch: ハイドロフォンから得られた値のpitch
    hydrophone_data_yaw: ハイドロフォンから得られた値のyaw
    target_FloatingBall_pitch: 浮遊球のpitch
    target_FloatingBall_yaw: 浮遊球のyaw
    """
    if hydrophone_data_pitch < target_FloatingBall_pitch and hydrophone_data_yaw < target_FloatingBall_yaw:
        return True
    return False

def hydrophoneSensorControl(yaw_angle,pitch_angle):
    """
    yaw_angle: ハイドロフォンから得られた値のyaw
    pitch_angle: ハイドロフォンから得られた値のpitch
    """
    if yaw_angle < target_FloatingBall_yaw and pitch_angle < target_FloatingBall_pitch:
        return True
    return False

