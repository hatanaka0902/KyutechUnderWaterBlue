def initialSetting():
    """
    センサデータ保存用のファイル名を設定てん作成する
    """
    filename = datetime.now().strftime('%Y%m%d%H%M%S') + '.csv'
    with open(filename, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'hydrophone_data_yaw', 'hydrophone_data_pitch'])
    return filename

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



def BingerSearch(pitch_deg, yaw_deg, assumed_distance=BINGER_SEARCH_ASSUMED_DISTANCE):
    """
    ハイドロフォンの pitch, yaw [deg] から、AUV現在位置を原点とした
    相対目標座標 (dx, dy, dz) [m, NED] を返す。

    距離は未知のため、assumed_distance [m] を仮定して固定値として扱う
    (球面座標 -> 直交座標の変換)。

    Args:
        pitch_deg: 仰角/俯角 [deg]。正 = 対象が下方向にある(NEDのz正方向と対応)。
        yaw_deg:   水平面内の方位角 [deg]。x軸正方向を0度とし反時計回りが正
                   (AzimuthControl()のatan2(y, x)と対応する定義)。
        assumed_distance: 対象までの距離の仮定値 [m]。

    Returns:
        dx, dy, dz: 現在位置からの目標オフセット [m] (NED)
    """
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    horizontal_distance = assumed_distance * math.cos(pitch)
    dx = horizontal_distance * math.cos(yaw)
    dy = horizontal_distance * math.sin(yaw)
    dz = assumed_distance * math.sin(pitch)

    return dx, dy, dz

