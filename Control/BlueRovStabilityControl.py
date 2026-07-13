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

# センサデータ取得用の関数
def get_sensor_data():
    ## IMUセンサデータ取得
    def get_imu_data():
        ## 加速度センサデータ取得(x m/s^2, y m/s^2, z m/s^2)
        imu_data[:3] = get_acceleration_data()
        ## ジャイロセンサデータ取得(x rad/s, y rad/s, z rad/s)
        imu_data[3:] = get_gyro_data()
        return imu_data
    ## DVLセンサデータ取得
    def get_dvl_data():
        ## DVLセンサデータ取得(x, y, z)(m)
        dvl_data[:3] = get_position_data()
        return dvl_data
    return get_imu_data(), get_dvl_data()

# 状態推定用の関数
def state_estimation():



# 姿勢制御用の関数
def attitude_control():

# 速度制御用の関数
def velocity_control():

def pi_Control():
    attitude_control()
    velocity_control()

# 目標位置取得用の関数->[x, y, z](m)
target_position = get_target_position()
# 目標速度取得用の関数->[x, y, z](m/s)
target_velocity = get_target_velocity()

