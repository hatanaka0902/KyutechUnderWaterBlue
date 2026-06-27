import math


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
    '''
    target_velocity_x, target_velocity_y, target_velocity_z = VelocitySpeed(target_x, target_y, target_z,current_z)
    target_azimuth_yaw_degree = AzimuthControl(target_x, target_y)
    ManualControl(target_velocity_x, target_velocity_y, target_velocity_z, target_azimuth_yaw_degree)


if __name__ == '__main__':
    import doctest
    doctest.testmod()
