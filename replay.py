"""
Performs a 3D replay of ankle angles. More of a sanity check
"""

import time

from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
from tqdm.auto import tqdm
from matplotlib.animation import FuncAnimation
import threading

file_selector = 'data/walk_hall_v2.TXT'
# calibration_timestep = 1000
cutoff_start = 40
cutoff_end = 60

def set_yaw_to_zero_matrix(R):
    # Set the yaw of the rotation matrix to zero
    # Get the euler angles
    euler_angles = R.as_euler('xyz', degrees=True)

    # Set the yaw to zero
    euler_angles[2] = 0

    # Convert the euler angles back to a rotation matrix
    return R.from_euler('xyz', euler_angles, degrees=True)


class IMUData:
    def __init__(self, timestep, quat1, quat2, accel1=None, accel2=None):
        self.timestep = float(timestep)
        self.IMU_1 = R.from_quat([float(q) for q in quat1])
        self.IMU_2 = R.from_quat([float(q) for q in quat2])
        self.accel1 = np.array([float(a) for a in accel1]) if accel1 else None
        self.accel2 = np.array([float(a) for a in accel2]) if accel2 else None

    def get_ankle_angle(self):
        v1 = np.dot(self.IMU_1.as_matrix(), [0, 0, 1])
        v2 = np.dot(self.IMU_2.as_matrix(), [0, 0, 1])
        ankle_angle = angle_between_3d_vectors(v1, v2) * 180 / np.pi
        return ankle_angle

    def __str__(self):
        return f"IMU1: {self.IMU_1.as_euler('xyz', degrees=True)} IMU2: {self.IMU_2.as_euler('xyz', degrees=True)}"


def angle_between_3d_vectors(v1, v2):
    return np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))



def angle_to_ground(vector):
    vector = np.array(vector)

    # Calculate the dot product of the vector and the ground normal (0, 1, 0)
    dot_product = np.dot(vector, np.array([0, 0, 1]))

    # Calculate the magnitudes of the vector and the ground normal
    vector_magnitude = np.linalg.norm(vector)
    ground_normal_magnitude = 1  # Magnitude of (0, 1, 0) is 1

    # Calculate the angle using the dot product formula
    angle_radians = np.arccos(dot_product / (vector_magnitude * ground_normal_magnitude))
    angle_degrees = np.degrees(angle_radians)

    return angle_degrees


data = []
ax = plt.axes(projection='3d')

def visualize_data(i):
    # update the plot with the latest datapoint
    global data, ax
    if len(data) == 0:
        return
    ax.clear()
    ts, angle_diff, v, w = data
    ax.quiver(0, 0, 0, v[0], v[1], v[2], color='r')
    ax.quiver(0, 0, 0, w[0], w[1], w[2], color='b')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

def replay_thread(imu_data):
    global data
    for i, d in enumerate(imu_data):
        ankle_angle = d.get_ankle_angle()
        print(f"{d.timestep:.2f}: {ankle_angle}")
        data = [d.timestep, ankle_angle, np.dot(d.IMU_1.as_matrix(), [0, 0, 1]), np.dot(d.IMU_2.as_matrix(), [0, 0, 1])]
        if i < len(imu_data) - 1:
            time.sleep(imu_data[i+1].timestep - d.timestep)

def main():
    prev_time = -1
    with open(file_selector, 'r') as f:
        data_read = f.readlines()
        # Parse the data into IMUData objects
        print("Reading data...")
        imu_data = []
        for x in tqdm(data_read):
            ts = float(x.split(',')[1])
            if ts < prev_time:
                print(f"Jump in time detected at {ts}")
                continue
            if ts < cutoff_start or (ts > cutoff_end and cutoff_end != -1):
                continue
            imu_data.append(IMUData(x.split(',')[1],
                    x.split(',')[2:6],
                    x.split(',')[6:10],
                    x.split(',')[10:13],
                    x.split(',')[13:16]))
            prev_time = ts
    print(f"Average Sampling Rate: {len(imu_data) / ((cutoff_end if cutoff_end != -1 else float(data_read[-1].split(',')[1])) - cutoff_start)} Hz")
    print("Starting replay...")
    start_time = time.time()
    ani = FuncAnimation(plt.gcf(), visualize_data, interval=1)
    threading.Thread(target=replay_thread, args=(imu_data,)).start()
    plt.show()



if __name__ == '__main__':
    main()
