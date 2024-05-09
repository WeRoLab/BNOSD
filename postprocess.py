from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
from tqdm.auto import tqdm

file_selector = 'data/IMU44.TXT'
calibration_timestep = 1000
cutoff_start = 0
cutoff_end = -2

sagittal_foot = R.from_euler('xyz', [0, 0, 0], degrees=True)
sagittal_shank = R.from_euler('xyz', [0, 90, 0], degrees=True)


class IMUData:
    def __init__(self, timestep, quat1, quat2,accel1=None,accel2=None):
        self.IMU_1 = R.from_quat(quat1)
        self.IMU_2 = R.from_quat(quat2)
        self.accel1 = accel1
        self.accel2 = accel2
        self.timestep = timestep

    def calibrate(self, foot_transform, shank_transform):
        # self.IMU_1 = R.from_matrix(np.dot(foot_transform, self.IMU_1.as_matrix()))
        # self.IMU_2 = R.from_matrix(np.dot(shank_transform, self.IMU_2.as_matrix()))
        self.IMU_1 = foot_transform * self.IMU_1
        self.IMU_2 = shank_transform * self.IMU_2

    def __str__(self):
        return f"IMU1: {self.IMU_1.as_euler('xyz', degrees=True)} IMU2: {self.IMU_2.as_euler('xyz', degrees=True)}"


def main():
    with open(file_selector, 'r') as f:
        data = f.readlines()
        # to dataframe
        print("Reading data...")
        data = [IMUData(x.split(',')[0:2], x.split(',')[2:6], x.strip().split(',')[6:10]) for x in data[cutoff_start:cutoff_end]]
        # data = [IMUData(x.split(',')[0:1], x.split(',')[1:5], x.strip().split(',')[5:9]) for x in data[cutoff:]]

    # Calibrate
    # calibration_point = data[calibration_timestep]
    # foot_transform = np.dot(sagittal_foot.as_matrix(), calibration_point.IMU_1.as_matrix().T)
    # shank_transform = np.dot(sagittal_shank.as_matrix(), calibration_point.IMU_2.as_matrix().T)
    # foot_transform = sagittal_foot * calibration_point.IMU_1.inv()
    # shank_transform = sagittal_shank * calibration_point.IMU_2.inv()
    # for x in tqdm(data, desc="Calibrating..."):
    #     x.calibrate(foot_transform, shank_transform)

    # Plot
    #
    # plt.figure()
    # for i in range(3):
    #     plt.subplot(3, 1, i + 1)
    #     # plt.plot([x.IMU_1.as_euler('xyz', degrees=True)[i] for x in data], label='IMU 1')
    #     plt.plot([x.IMU_2.as_euler('xyz', degrees=True)[i] for x in data], label='IMU 2')
    #     plt.title(['roll', 'pitch', 'yaw'][i])
    # plt.show()

    # Compute R_ab
    r_ab = [np.dot(x.IMU_1.as_matrix().T, x.IMU_2.as_matrix()) for x in data]
    # pitch angle:
    pitch = [np.arccos((np.trace(x)-1)/2)*360/2/np.pi for x in r_ab]
    # plot
    plt.figure()
    plt.plot(pitch)
    plt.title('Angle Diff')
    # manually set y limits
    # plt.ylim(170, 180)
    plt.show()

    # plot raw data
    # plt.figure()
    # plt.plot([x.IMU_1.as_quat()[3] for x in data], label='IMU 1')
    # plt.plot([x.IMU_2.as_quat()[3] for x in data], label='IMU 2')
    # plt.title('Quaternion W')
    # plt.show()


if __name__ == '__main__':
    main()
