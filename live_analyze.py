from postprocess import IMUData
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import serial
import threading
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/tty.usbmodem1201', 115200)

foot_transform = None
shank_transform = None

calibration_point = None

def rotation_matrix_to_3d_vector(R):
    return np.array([R[0, 2], R[1, 2], R[2, 2]])/np.linalg.norm([R[0, 2], R[1, 2], R[2, 2]])

def read_data():
    global foot_transform, shank_transform
    global calibration_point
    ser.reset_input_buffer()
    line = ser.readline().decode('UTF-8').replace('\n', '')
    if len(line.split(',')) != 16:
        return None
    cycle_count, time_elapsed, qr, qi, qj, qk, pr, pi, pj, pk, qa, qb, qc, pa, pb, pc = line.split(',')
    datapoint = IMUData(time_elapsed, [float(qr), float(qi), float(qj), float(qk)], [float(pr), float(pi), float(pj), float(pk)])
    if foot_transform is None:
        sagittal_foot = R.from_euler('xyz', [0, 0, 0], degrees=True)
        sagittal_shank = R.from_euler('xyz', [0, 0, 0], degrees=True)
        foot_transform = sagittal_foot * datapoint.IMU_1.inv()
        shank_transform = sagittal_shank * datapoint.IMU_2.inv()
    # datapoint.calibrate(foot_transform, shank_transform)
    if calibration_point is None:
        calibration_point = datapoint
    return datapoint

def get_angle_diff(data: IMUData):
    r_ab = np.dot(data.IMU_1.as_matrix().T, data.IMU_2.as_matrix())
    return np.arccos((np.trace(r_ab)-1)/2)*360/2/np.pi

def angle_to_ground(vector):
    # project vector to x-z frame then calculate angle between the vector and the x-axis
    # Extract components of the vector
    v_x, v_y, v_z = vector

    # Project the vector onto the x-z plane
    v_xz = np.array([v_x, 0, v_z])

    # Calculate the magnitude of the projected vector
    magnitude_v_xz = np.linalg.norm(v_xz)

    # Calculate the cosine of the angle
    cos_theta = v_x / magnitude_v_xz

    # Calculate the angle in radians
    theta_radians = np.arccos(cos_theta)

    # Convert the angle to degrees
    theta_degrees = np.degrees(theta_radians)

    return theta_degrees

def angle_between_3d_vectors(v1, v2):
    return np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))

def print_roll_pitch_yaw(R):
    print(f"Roll: {np.arctan2(R[2, 1], R[2, 2])*180/np.pi}")
    print(f"Pitch: {np.arcsin(-R[2, 0])*180/np.pi}")
    print(f"Yaw: {np.arctan2(R[1, 0], R[0, 0])*180/np.pi}")

data = []

ax = plt.axes(projection='3d')

def collect_data():
    global data
    while True:
        datapoint = read_data()
        if datapoint is None:
            continue
        # print(datapoint.IMU_2.as_euler('xyz', degrees=True))
        # diff = get_angle_diff(datapoint)
        v = np.dot(datapoint.IMU_1.as_matrix(), [0, 0, 1])
        w = np.dot(datapoint.IMU_2.as_matrix(), [0, 0, 1])
        # def print_vector(v):
        #     print(f"vector((0,0,0),{tuple([float(i) for i in v])})")
        # print_vector(v)
        # print_vector(w)
        # print_roll_pitch_yaw(datapoint.IMU_1.as_matrix())
        # print_roll_pitch_yaw(datapoint.IMU_2.as_matrix())
        angle_diff = angle_between_3d_vectors(v, w)
        print(angle_diff*180/np.pi)
        data.append([datapoint.timestep, angle_diff*180/np.pi, v, w])

def visualize_data(i):
    # update the plot with the latest datapoint
    global data, ax
    if len(data) == 0:
        return
    ax.clear()
    ts, angle_diff, v, w = data[-1]
    ax.quiver(0, 0, 0, v[0], v[1], v[2], color='r')
    ax.quiver(0, 0, 0, w[0], w[1], w[2], color='b')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])


def main():
    # start a thead to collect data
    t = threading.Thread(target=collect_data)
    t.start()
    ani = FuncAnimation(plt.gcf(), visualize_data, interval=1)
    plt.show()


if __name__ == '__main__':
    main()