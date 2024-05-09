
from postprocess import IMUData
import numpy as np
from scipy.spatial.transform import Rotation as R
import os


import serial

ser = serial.Serial('/dev/tty.usbmodem1301', 115200)

foot_transform = None
shank_transform = None

calibration_point = None

def read_data():
    global foot_transform, shank_transform
    global calibration_point
    ser.reset_input_buffer()
    line = ser.readline().decode('UTF-8').replace('\n', '')
    if len(line.split(',')) != 16:
        return None
    cycle_count, time_elapsed, qr, qi, qj, qk, pr, pi, pj, pk, qa, qb, qc, pa, pb, pc = line.split(',')
    datapoint = IMUData(cycle_count, [float(qr), float(qi), float(qj), float(qk)], [float(pr), float(pi), float(pj), float(pk)])
    if foot_transform is None:
        sagittal_foot = R.from_euler('xyz', [0, 0, 0], degrees=True)
        sagittal_shank = R.from_euler('xyz', [0, 0, 0], degrees=True)
        foot_transform = sagittal_foot * datapoint.IMU_1.inv()
        shank_transform = sagittal_shank * datapoint.IMU_2.inv()
    datapoint.calibrate(foot_transform, shank_transform)
    if calibration_point is None:
        calibration_point = datapoint
    return datapoint

def get_angle_diff(data: IMUData):
    r_ab = np.dot(data.IMU_1.as_matrix().T, data.IMU_2.as_matrix())
    return np.arccos((np.trace(r_ab)-1)/2)*360/2/np.pi


def main():
    while True:
        datapoint = read_data()
        if datapoint is None:
            continue
        # print(datapoint.IMU_2.as_euler('xyz', degrees=True))
        diff = get_angle_diff(datapoint)
        # os.system('clear')
        print(np.round(diff, 2))
        # get pitch
        # print(np.arcsin(-diff[2, 0]) * 180 / np.pi)

if __name__ == '__main__':
    main()