from scipy.spatial.transform import Rotation as R
import numpy as np
from tqdm.auto import tqdm
import pandas as pd
import os
from concurrent.futures import ThreadPoolExecutor


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
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))


class IMUData:
    def __init__(self, timestep, quat1, quat2, accel1=None, accel2=None, fsr_voltage=None, angle=None, activity=None):
        self.ankle_angle = angle
        self.timestep = float(timestep)
        if len(quat1) != 4:
            self.IMU_1 = R.from_euler('xyz', [float(q) for q in quat1], degrees=False)
            self.IMU_2 = R.from_euler('xyz', [float(q) for q in quat2], degrees=False)
        else:
            self.IMU_1 = R.from_quat([float(q) for q in quat1])
            self.IMU_2 = R.from_quat([float(q) for q in quat2])
        self.accel1 = np.array([float(a) for a in accel1]) if accel1 is not None else None
        self.accel2 = np.array([float(a) for a in accel2]) if accel2 is not None else None
        self.fsr_voltage = float(fsr_voltage) if fsr_voltage else None
        self.heel_striking = False
        self.foot_contact = False
        self.force = 0
        self.activity = activity

    def get_ankle_angle(self):
        if self.ankle_angle:
            return self.ankle_angle
        v1 = np.dot(self.IMU_1.as_matrix(), [0, 0, 1])
        v2 = np.dot(self.IMU_2.as_matrix(), [0, 0, 1])
        self.ankle_angle = angle_between_3d_vectors(v1, v2) * 180 / np.pi
        return self.ankle_angle

    def __str__(self):
        return f"IMU1: {self.IMU_1.as_euler('xyz', degrees=True)} IMU2: {self.IMU_2.as_euler('xyz', degrees=True)}"


def load_data(file_selector, cutoff_start: float = 0, cutoff_end: float = -1):
    prev_time = -1
    with open(file_selector, 'r') as f:
        data = f.readlines()
        # Parse the data into IMUData objects
        # print("Reading data...")
        imu_data = []
        for x in data:
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
    heel_strike_threshold = 0.03
    # for data_pt in imu_data:
    #     if data_pt.fsr_voltage > heel_strike_threshold:
    #         if not heel_striking:
    #             data_pt.heel_striking = True
    #         heel_striking = True
    #     else:
    #         heel_striking = False
    # print(
    # f"Average Sampling Rate: {len(imu_data) / ((cutoff_end if cutoff_end != -1 else float(data[-1].split(',')[1])) - cutoff_start)} Hz")
    return imu_data


def load_moticon(filename, moticon_offset=0):
    times = []
    forces = []
    imu_z = []
    with open(filename, 'r') as file:
        # Skip the header lines that start with #
        for line in file:
            if not line.startswith('#'):
                # Split the line by tabs and get the right total force (24th column, index 23)
                data = line.strip().split('\t')
                times.append(float(data[0]))
                forces.append(float(data[23]))
                imu_z.append(float(data[19]))
    # downsampling to 50 Hz
    times = times[::2]
    forces = forces[::2]
    imu_z = imu_z[::2]
    # apply offset
    times = [time + moticon_offset for time in times]
    return [times, forces, imu_z]


def load_treadmill(treadmill_file, time_offset_s):
    import pandas as pd
    treadmill_data = pd.read_csv(treadmill_file)
    treadmill_data = treadmill_data[['1:Timestamp', '1:Fz']]
    # downsample to 50 Hz
    treadmill_data = treadmill_data.iloc[::20, :]
    # convert the timestamp to seconds
    print(time_offset_s)
    treadmill_data['1:Timestamp'] = treadmill_data['1:Timestamp'] / 1000 + time_offset_s
    return [list(treadmill_data['1:Timestamp']), list(treadmill_data['1:Fz'])]


def load_data_from_folder(data_folder, cutoff_start=0, cutoff_end=-1, treadmill_offset=0, moticon_offset=0):
    import os
    files = os.listdir(data_folder)
    ankle_data = None
    insole_data = None
    treadmill_data = None

    # Convert single values to lists for consistency
    if not isinstance(cutoff_start, list):
        cutoff_start = [cutoff_start]
    if not isinstance(cutoff_end, list):
        cutoff_end = [cutoff_end]

    # Ensure cutoff_start and cutoff_end have the same length
    if len(cutoff_start) != len(cutoff_end):
        raise ValueError("cutoff_start and cutoff_end must have the same length")

    for file in files:
        if 'treadmill' in file:
            treadmill_data = load_treadmill(data_folder + file, treadmill_offset)
        elif 'ankle' in file:
            ankle_data = load_data(data_folder + file)
        elif 'insole' in file:
            insole_data = load_moticon(data_folder + file, moticon_offset)

    # Helper function to check if a time is within any segment
    def is_in_segments(time):
        for start, end in zip(cutoff_start, cutoff_end):
            if (start < time) and (end == -1 or time < end):
                return True
        return False

    # apply cutoffs
    if ankle_data:
        ankle_data = [data for data in ankle_data if is_in_segments(data.timestep)]
    if insole_data:
        insole_data[2] = [imu_z for time, imu_z in zip(insole_data[0], insole_data[2]) if is_in_segments(time)]
        insole_data[1] = [force for time, force in zip(insole_data[0], insole_data[1]) if is_in_segments(time)]
        insole_data[0] = [time for time in insole_data[0] if is_in_segments(time)]
    if treadmill_data:
        treadmill_data[1] = [force for time, force in zip(treadmill_data[0], treadmill_data[1]) if is_in_segments(time)]
        treadmill_data[0] = [time for time in treadmill_data[0] if is_in_segments(time)]

    return ankle_data, insole_data, treadmill_data


def integrate_ankle_insole(ankle_data, insole_data=None):
    # replace ankle's heel_strike data
    heel_striking = True
    heel_lifting = True
    heel_striking_times = []
    heel_lifting_times = []
    heel_strike_threshold = 50
    if insole_data is not None:
        # merge insole's force into ankle data
        insole_ind = 0
        for data in ankle_data:
            while (insole_ind < len(insole_data[0]) - 1 and
                   abs(insole_data[0][insole_ind + 1] - data.timestep) < abs(
                        insole_data[0][insole_ind] - data.timestep)):
                insole_ind += 1
            data.force = insole_data[1][insole_ind]
        for time, force in zip(insole_data[0], insole_data[1]):
            if force > heel_strike_threshold:
                if not heel_striking:
                    heel_striking = True
                    heel_striking_times.append(time)
            else:
                heel_striking = False
            if force < heel_strike_threshold:
                if not heel_lifting:
                    heel_lifting = True
                    heel_lifting_times.append(time)
            else:
                heel_lifting = False
    else:
        for data in ankle_data:
            if data.force > heel_strike_threshold:
                if not heel_striking:
                    heel_striking = True
                    heel_striking_times.append(data.timestep)
            else:
                heel_striking = False
            if data.force < heel_strike_threshold:
                if not heel_lifting:
                    heel_lifting = True
                    heel_lifting_times.append(data.timestep)
            else:
                heel_lifting = False
    # find closest ankle data point to each heel strike, reset the heel striking flag
    for data in ankle_data:
        data.heel_striking = False
        data.heel_lifting = False
    ind = 0
    for t in heel_striking_times:
        while ind < len(ankle_data) - 1 and abs(ankle_data[ind].timestep - t) > abs(t - ankle_data[ind + 1].timestep):
            ind += 1
        ankle_data[ind].heel_striking = True
    ind = 0
    for t in heel_lifting_times:
        while ind < len(ankle_data) - 1 and abs(ankle_data[ind].timestep - t) > abs(t - ankle_data[ind + 1].timestep):
            ind += 1
        ankle_data[ind].heel_lifting = True


def gyro_to_euler(gyro_data, sampling_rate, initial_angles=None):
    """
    Convert gyroscope readings to euler angles using numerical integration

    Args:
        gyro_data: numpy array of shape (n, 3) containing gyroscope readings [wx, wy, wz]
        sampling_rate: sampling frequency in Hz
        initial_angles: initial euler angles [phi, theta, psi] in radians. Defaults to [0,0,0]

    Returns:
        euler_angles: numpy array of shape (n, 3) containing euler angles [phi, theta, psi]
    """
    dt = 1.0 / sampling_rate

    if initial_angles is None:
        initial_angles = np.array([0.0, 0.0, 0.0])

    # Ensure gyro_data is numpy array and reshape if needed
    gyro_data = np.array(gyro_data)
    if len(gyro_data.shape) == 1:
        gyro_data = gyro_data.reshape(1, 3)

    n_samples = len(gyro_data)

    # Initialize euler angles array
    euler_angles = np.zeros((n_samples, 3))
    euler_angles[0] = initial_angles

    # Convert gyroscope data to euler angle rates
    for i in range(1, n_samples):
        phi = euler_angles[i - 1, 0]  # roll
        theta = euler_angles[i - 1, 1]  # pitch

        # Rotation matrix to convert body rates to euler rates
        W = np.array([
            [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
        ])

        # Calculate euler rates - ensure proper matrix multiplication
        euler_rates = W @ gyro_data[i]

        # Integrate euler rates to get euler angles
        euler_angles[i] = euler_angles[i - 1] + euler_rates * dt

        # Normalize angles to [-pi, pi]
        euler_angles[i] = np.mod(euler_angles[i] + np.pi, 2 * np.pi) - np.pi

    return euler_angles


def detect_peaks(data, threshold=90):
    """ Vectorized peak detection """
    data = np.array(data)
    prev = np.roll(data, 1)
    next = np.roll(data, -1)
    peaks = (data > prev) & (data > next) & (data > threshold)
    peaks[0] = peaks[-1] = False  # First and last cannot be peaks
    return np.where(peaks)[0]


def process_file(file, data_dir):
    imu_path = os.path.join(data_dir, "imu", file)
    gon_path = os.path.join(data_dir, "gon", file)
    strike_path = os.path.join(data_dir, "gcRight", file)
    cond_path = os.path.join(data_dir, "conditions", file)

    imu_df = pd.read_csv(imu_path)[[
        "data_Header", "data_foot_Accel_X", "data_foot_Accel_Y", "data_foot_Accel_Z",
        "data_shank_Accel_X", "data_shank_Accel_Y", "data_shank_Accel_Z"]]
    gon_df = pd.read_csv(gon_path)[["data_Header", "data_ankle_sagittal"]]
    strike_df = pd.read_csv(strike_path)[["data_Header", "data_HeelStrike", "data_ToeOff"]]
    conditions_df = pd.read_csv(cond_path)[["labels_Header", "labels_Label"]].rename(
        columns={"labels_Header": "data_Header"})

    imu_df = downsample_to_50hz(imu_df)
    gon_df = downsample_to_50hz(gon_df)
    strike_df = downsample_to_50hz(strike_df)

    merged_df = pd.merge_asof(
        imu_df.sort_values('data_Header'),
        gon_df.sort_values('data_Header'),
        on='data_Header', direction='nearest')
    merged_df = pd.merge_asof(
        merged_df.sort_values('data_Header'),
        strike_df.sort_values('data_Header'),
        on='data_Header', direction='nearest')
    merged_df = pd.merge_asof(
        merged_df.sort_values('data_Header'),
        conditions_df.sort_values('data_Header'),
        on='data_Header', direction='nearest')

    features = merged_df[[
        'data_foot_Accel_X', 'data_foot_Accel_Y', 'data_foot_Accel_Z',
        'data_shank_Accel_X', 'data_shank_Accel_Y', 'data_shank_Accel_Z',
        'data_ankle_sagittal', 'data_Header', 'data_HeelStrike', 'data_ToeOff', 'labels_Label'
    ]].values

    heel_strikes = detect_peaks(merged_df["data_HeelStrike"])
    toe_offs = detect_peaks(merged_df["data_ToeOff"])

    imu_data = []
    current_phase = 1
    heel_strike_set = set(heel_strikes)
    toe_off_set = set(toe_offs)

    for i in range(len(features)):
        data = IMUData(
            timestep=features[i][7],
            quat1=[1, 0, 0, 0], quat2=[1, 0, 0, 0],
            accel1=features[i][:3] * 9.8, accel2=features[i][3:6] * 9.8,
            angle=features[i][6], activity=features[i][10]
        )
        if i in heel_strike_set:
            current_phase = 1
        elif i in toe_off_set:
            current_phase = 0
        data.force = 300 if current_phase == 1 else 0
        imu_data.append(data)

    return normalize_ankle_angle(imu_data)


def process_camargo_data(data_dir, load_portion=1):
    files = os.listdir(os.path.join(data_dir, "gcRight"))
    files = files[:int(len(files) * load_portion)]

    with ThreadPoolExecutor() as executor:
        results = list(
            tqdm(executor.map(lambda f: process_file(f, data_dir), files), total=len(files), desc="Processing camargo"))
    return results


def downsample_to_50hz(df, header_column='data_Header'):
    if df.empty or header_column not in df.columns:
        return pd.DataFrame()
    try:
        df[header_column] = pd.to_numeric(df[header_column], errors='coerce')
        df = df.dropna(subset=[header_column])  # Remove rows where header is NaN
        if df.empty:
            return pd.DataFrame()
    except ValueError:
        print(f"Error: Could not convert '{header_column}' to numeric.")
        return pd.DataFrame()
    sampling_rate = 1 / (df[header_column].diff().mean())
    downsample_factor = int(round(sampling_rate / 50))

    if downsample_factor <= 0:
        print("Warning: Original sampling rate is already at or below 50Hz. No downsampling performed.")
        return df
    downsampled_df = df.iloc[::downsample_factor, :].reset_index(drop=True)
    return downsampled_df


def normalize_ankle_angle(ankle_data):
    # find mean ankle angle and subtract it from all angles
    mean_angle = np.mean([data.get_ankle_angle() for data in ankle_data])
    for data in ankle_data:
        data.ankle_angle -= mean_angle
    return ankle_data


from scipy.signal import detrend
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


def find_best_moticon_offset(data_folder,
                             fs=200,
                             cutoff_start=None,
                             cutoff_end=None,
                             plot_result=True):
    # --- Load raw data ---
    ankle_data = load_data(data_folder + 'ankle.TXT',
                           cutoff_start or 0,
                           cutoff_end or -1)
    mot_times, mot_forces, mot_imu_z = load_moticon(data_folder + 'insole.txt')

    ankle_time = np.array([pt.timestep for pt in ankle_data])
    ankle_sig = np.array([pt.accel2[0] for pt in ankle_data])

    mot_time = np.array(mot_times)
    mot_sig = np.array(mot_imu_z) / -9.81

    # --- Define full time grid to capture entire lag range ---
    t_min = min(ankle_time.min(), mot_time.min())
    t_max = max(ankle_time.max(), mot_time.max())
    t_uniform = np.arange(t_min, t_max, 1.0 / fs)

    # --- Interpolate onto uniform grid ---
    ankle_interp = interp1d(ankle_time, ankle_sig, bounds_error=False, fill_value=0.0)
    mot_interp = interp1d(mot_time, mot_sig, bounds_error=False, fill_value=0.0)
    a = detrend(ankle_interp(t_uniform))
    m = detrend(mot_interp(t_uniform))

    # --- FFT-based cross-correlation ---
    n = len(t_uniform)
    N = 2 ** int(np.ceil(np.log2(2 * n - 1)))
    A = np.fft.rfft(a, n=N)
    M = np.fft.rfft(m, n=N)
    corr = np.fft.irfft(A * np.conj(M), n=N)
    corr = np.concatenate((corr[-(n - 1):], corr[:n]))
    lags = np.arange(-n + 1, n) / fs

    # --- Peak detection & sub-sample refinement ---
    idx = np.argmax(np.abs(corr))
    offset = lags[idx]
    if 1 <= idx < len(corr) - 1:
        y0, y1, y2 = corr[idx - 1], corr[idx], corr[idx + 1]
        p = (y0 - y2) / (2 * (y0 - 2 * y1 + y2))
        offset += p / fs

    # --- Plot results ---
    if plot_result:
        plt.figure(figsize=(8, 4))
        plt.plot(lags, corr)
        plt.axvline(offset, linestyle='--', label=f'Offset = {offset:.4f}s')
        plt.xlabel('Lag (s)')
        plt.ylabel('Cross-correlation')
        plt.title('Moticon vs Ankle Signal Cross-correlation')
        plt.legend()
        plt.tight_layout()
        plt.show()

    print(f"Estimated moticon -> ankle offset: {offset:.4f} s")
    return offset
