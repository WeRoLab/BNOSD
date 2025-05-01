from torch.utils.data import Dataset
import torch
import numpy as np
import random

def collect_features(ankle_data):
    features = []
    for data_point in ankle_data:
        # imu1_euler = data_point.IMU_1.as_euler('xyz', degrees=True)
        # imu2_euler = data_point.IMU_2.as_euler('xyz', degrees=True)

        # Combine features
        features.append(np.concatenate([
            # imu1_euler,
            # imu2_euler,
            data_point.accel1,
            data_point.accel2,
            [data_point.get_ankle_angle()]
        ]))

    return features

class StrideDataset(Dataset):
    def __init__(self, ankle_data, sequence_length=100):
        self.half_length = sequence_length // 2
        self.features = []
        self.labels = []

        seq_features = collect_features(ankle_data)
        for i in range(self.half_length, len(ankle_data) - self.half_length):
            label = 1 if (ankle_data[i].force > 200) else 0

            self.features.append(seq_features[i-self.half_length:i+self.half_length])
            self.labels.append(label)

        self.features = np.array(self.features)
        self.features = torch.FloatTensor(self.features)
        
        self.labels = torch.LongTensor(self.labels)
        self.pos_weight = sum(self.labels == 0) / sum(self.labels == 1)

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        return self.features[idx], self.labels[idx]

from sklearn.preprocessing import LabelEncoder

class StrideClassificationDataset(Dataset):
    def __init__(self, segments, max_length=100, num_features=7):
        self.max_length = max_length
        self.num_features = num_features

        # Ensure activity is detected
        for segment in segments:
            if segment.activity is None:
                segment.detect_activity()

        self.valid_segments = [s for s in segments if s.activity is not None]
        self.label_encoder = LabelEncoder()
        self.y = torch.tensor(
            self.label_encoder.fit_transform([s.activity for s in self.valid_segments]), dtype=torch.long
        )

        self.X = torch.zeros((len(self.valid_segments), num_features, max_length), dtype=torch.float32)

        for i, segment in enumerate(self.valid_segments):
            data = np.array(collect_features(segment.data))
            length = min(data.shape[0], max_length)
            self.X[i, :, :length] = torch.tensor(data[:length].T, dtype=torch.float32)  # shape: (7, T)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

    def get_label_encoder(self):
        return self.label_encoder