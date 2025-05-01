from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
from tqdm.auto import tqdm
from scipy.signal import savgol_filter

file_selector = 'data/fsr/ramp_down_up_walk.TXT'
# treadmill_file = 'data/michael_fsr_1.csv'
# calibration_timestep = 1000
cutoff_start = 66.5
cutoff_end = 73

def set_yaw_to_zero_matrix(R):
    # Set the yaw of the rotation matrix to zero
    # Get the euler angles
    euler_angles = R.as_euler('xyz', degrees=True)

    # Set the yaw to zero
    euler_angles[2] = 0

    # Convert the euler angles back to a rotation matrix
    return R.from_euler('xyz', euler_angles, degrees=True)

def plot_against_treadmill():
    import pandas as pd
    # read a csv file with treadmill data, select the 1:Fz column and 1:Timestamp column
    time_offset_s = -281
    treadmill_data = pd.read_csv(treadmill_file)
    treadmill_data = treadmill_data[['1:Timestamp', '1:Fz']]
    # convert the timestamp to seconds
    treadmill_data['1:Timestamp'] = treadmill_data['1:Timestamp'] / 1000 + time_offset_s

    # limit data to the same time range as the IMU data (cutoff_start to cutoff_end if cutoff_end is not -1)
    treadmill_data = treadmill_data[(treadmill_data['1:Timestamp'] > cutoff_start) & (treadmill_data['1:Timestamp'] < cutoff_end) if cutoff_end != -1 else (treadmill_data['1:Timestamp'] > cutoff_start)]
    # return a lists of time and force data
    return list(treadmill_data['1:Timestamp']), list(treadmill_data['1:Fz'])


class IMUData:
    def __init__(self, timestep, quat1, quat2, accel1=None, accel2=None, fsr_voltage=None):
        self.timestep = float(timestep)
        self.IMU_1 = R.from_quat([float(q) for q in quat1])
        self.IMU_2 = R.from_quat([float(q) for q in quat2])
        # self.IMU_1 = set_yaw_to_zero_matrix(self.IMU_1)
        # self.IMU_2 = set_yaw_to_zero_matrix(self.IMU_2)
        self.accel1 = np.array([float(a) for a in accel1]) if accel1 else None
        self.accel2 = np.array([float(a) for a in accel2]) if accel2 else None
        self.fsr_voltage = float(fsr_voltage)
        self.heel_striking = False

    # def calibrate(self, foot_transform, shank_transform):
    #     self.IMU_1 = foot_transform * self.IMU_1
    #     self.IMU_2 = shank_transform * self.IMU_2

    def get_ankle_angle(self):
        v1 = np.dot(self.IMU_1.as_matrix(), [0, 0, 1])
        v2 = np.dot(self.IMU_2.as_matrix(), [0, 0, 1])
        ankle_angle = angle_between_3d_vectors(v1, v2) * 180 / np.pi
        # if self.timestep in [300, 500]:
        #     print(self.timestep)
        #     print(f"vector((0,0,0),{tuple(v1)})")
        #     print(f"vector((0,0,0),{tuple(v2)})")
        #     print(f"angle: {ankle_angle}")
        return ankle_angle

    def __str__(self):
        return f"IMU1: {self.IMU_1.as_euler('xyz', degrees=True)} IMU2: {self.IMU_2.as_euler('xyz', degrees=True)}"



def angle_to_ground(vector):
    vector = np.array(vector)

    # Calculate the dot product of the vector and the ground normal (0, 1, 0)
    dot_product = np.dot(vector, np.array([0, 0, 1]))

    # Calculate the magnitudes of the vector and the ground normal
    vector_magnitude = np.linalg.norm(vector)
    ground_normal_magnitude = 1  # Magnitude of (0, 1, 0) is 1

    # Calculate the angle using the dot product formula
    angle_radians = np.arccos(dot_product / (vector_magnitude * ground_normal_magnitude))

    # Convert the angle from radians to degrees
    angle_degrees = np.degrees(angle_radians)

    return angle_degrees


def angle_between_3d_vectors(v1, v2):
    return np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))





def main():
    prev_time = -1
    with open(file_selector, 'r') as f:
        data = f.readlines()
        # Parse the data into IMUData objects
        print("Reading data...")
        imu_data = []
        for x in tqdm(data):
            x = x.strip()
            ts = float(x.split(',')[1])
            if ts < prev_time:
                print(f"Jump in time detected at {ts}")
                continue
            if ts < cutoff_start or (ts > cutoff_end and cutoff_end != -1):
                continue
            val = x.split(',')[16] if len(x.split(',')) > 16 else None
            threasold = 0.008
            reached_threshold = 0.15 if val and float(val) > threasold else 0
            imu_data.append(IMUData(x.split(',')[1],
                    x.split(',')[2:6],
                    x.split(',')[6:10],
                    x.split(',')[10:13],
                    x.split(',')[13:16], val))
            prev_time = ts
    heel_striking = False
    heel_strike_threshold = 0.05
    for data_pt in imu_data:
        if data_pt.fsr_voltage > heel_strike_threshold:
            if not heel_striking:
                data_pt.heel_striking = True
            heel_striking = True
        else:
            heel_striking = False
    print(f"Average Sampling Rate: {len(imu_data) / ((cutoff_end if cutoff_end != -1 else float(data[-1].split(',')[1])) - cutoff_start)} Hz")

    print(f"Maximum angle difference: {max([x.get_ankle_angle() for x in imu_data])-min([x.get_ankle_angle() for x in imu_data])}")

    # Plot ankle angle over time
    time_axis = [x.timestep for x in imu_data]
    ankle_angles = [x.get_ankle_angle() for x in imu_data]
    # smooth the ankle angles with respect to time
    time_axis = np.array(time_axis)
    ankle_angles = np.array(ankle_angles)
    window_size = 10  # Odd number, representing the number of data points used for smoothing
    polyorder = 3  # Polynomial order for the smoothing fit
    ankle_angles = savgol_filter(ankle_angles, window_size, polyorder)

    fig, ax1 = plt.subplots()
    ax1.plot(time_axis, ankle_angles)
    ax1.set_ylabel("Ankle Angle (degrees)")
    # a not dotted line at 90 degrees
    # ax1.axhline(y=90, color='r', linestyle='-')
    # set y limit to 85-95
    # ax1.set_ylim([105, 115])

    # overlay with acceleration
    # ax2 = ax1.twinx()
    # vertical_accel = [x.accel1[2] for x in imu_data]
    # ax2.plot(time_axis, vertical_accel, color='r')
    # ax2.set_ylabel("Acceleration (m/s^2)", color='r')

    # overlay with fsr voltage
    # ax3 = ax1.twinx()
    # fsr_voltage = [x.fsr_voltage for x in imu_data]
    # ax3.plot(time_axis, fsr_voltage, color='g')
    # ax3.set_ylabel("FSR Voltage (V)")
    # ax3.set_ylim([0, 0.2])

    # ax4 = ax1.twinx()
    # treadmill_time, treadmill_force = plot_against_treadmill()
    # ax4.plot(treadmill_time, treadmill_force, color='r')
    # ax4.set_ylabel("Treadmill Force (N)")
    # ax4.set_ylim([0, 1200])

    # vertical lines for heel strikes
    for data in imu_data:
        if data.heel_striking:
            plt.axvline(x=data.timestep, color='r', linestyle='--')

    # # plot moving average of vertical_accel
    # window_size = 20
    # vertical_accel = np.convolve(vertical_accel, np.ones(window_size) / window_size, mode='valid')
    # ax2.plot(np.arange(len(vertical_accel)), vertical_accel, color='g')

    plt.xlabel("Time (s)")
    plt.title("Walking")
    plt.show()


if __name__ == '__main__':
    main()
