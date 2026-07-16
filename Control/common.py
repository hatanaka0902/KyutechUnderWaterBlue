import datetime
import csv
from pathlib import Path

def SensorGetData():
    """
    センサから得られた値を返す
    """
    return hydrophone_data_yaw,hydrophone_data_pitch

def save_sensor_data(sensor_data,Now_filename):
    """
    センサから得られた値を保存する
    """
    filename = Path('Assets/Sensor_data') / Now_filename + '.csv'
    if not filename.parent.exists():
        filename.parent.mkdir(parents=True)
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(sensor_data.tolist())
    return 0

def get_target_position(target_position_x,target_position_y,target_position_z):
    dx = target_position_x
    dy = target_position_y
    dz = target_position_z
    return [dx, dy, dz]


