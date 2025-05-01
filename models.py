from torch import nn
import torch
import torch.nn.functional as F


class StrideLSTM(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, dropout_rate=0.5):
        super().__init__()

        # Increased complexity in LSTM:  Added batch normalization and dropout.
        self.lstm = nn.LSTM(
            input_size, hidden_size, num_layers,
            bidirectional=True,  # Capture temporal context
            batch_first=True,
            dropout=dropout_rate if num_layers > 1 else 0  # Apply dropout between LSTM layers (if > 1 layer)
        )
        self.lstm_dropout = nn.Dropout(dropout_rate)  # Dropout after LSTM
        self.batch_norm_lstm = nn.BatchNorm1d(hidden_size * 2)  # Batch norm after LSTM (and before attention)

        # Increased complexity in Attention:  Added more layers and ReLU.
        self.attention = nn.Sequential(
            nn.Linear(hidden_size * 2, 128),  # Increased hidden size
            nn.ReLU(),  # Changed to ReLU
            nn.Dropout(dropout_rate),  # Dropout in attention
            nn.Linear(128, 64),
            nn.Tanh(),
            nn.Dropout(dropout_rate),  # Dropout in attention
            nn.Linear(64, 1),
            nn.Softmax(dim=1)
        )

        # Increased complexity in FC layers:  Added a hidden layer and dropout.
        self.fc1 = nn.Linear(hidden_size * 2, hidden_size)  # Added a hidden layer
        self.fc_dropout = nn.Dropout(dropout_rate)  # Dropout before final layer
        self.fc2 = nn.Linear(hidden_size, 2)  # Output layer

    def forward(self, x):
        lstm_out, _ = self.lstm(x)  # [batch, seq_len, hidden*2]
        lstm_out = self.lstm_dropout(lstm_out)

        # Batch Normalization *after* the sequence dimension (important!)
        #  Reshape for batch norm, apply, then reshape back.
        batch_size, seq_len, hidden_dim = lstm_out.shape
        lstm_out = lstm_out.reshape(batch_size * seq_len, hidden_dim)
        lstm_out = self.batch_norm_lstm(lstm_out)
        lstm_out = lstm_out.reshape(batch_size, seq_len, hidden_dim)

        attn_weights = self.attention(lstm_out)  # [batch, seq_len, 1]
        context = torch.sum(lstm_out * attn_weights, dim=1)  # [batch, hidden*2]

        # Apply the first FC layer, ReLU, and dropout
        out = self.fc1(context)
        out = torch.relu(out)
        out = self.fc_dropout(out)

        # Apply the final FC layer
        out = self.fc2(out)
        return out

class StrideClassificationCNN(nn.Module):
    def __init__(self, input_channels: int = 7, num_classes: int = 4):
        super(StrideClassificationCNN, self).__init__()
        self.conv1 = nn.Conv1d(input_channels, 64, kernel_size=5, padding=2)
        self.bn1 = nn.BatchNorm1d(64)
        self.pool1 = nn.AdaptiveMaxPool1d(50)

        self.conv2 = nn.Conv1d(64, 128, kernel_size=5, padding=2)
        self.bn2 = nn.BatchNorm1d(128)
        self.pool2 = nn.AdaptiveMaxPool1d(25)

        self.dropout = nn.Dropout(0.5)
        self.fc1 = nn.Linear(128 * 25, 128)
        self.fc2 = nn.Linear(128, num_classes)

    def forward(self, x):
        # x shape: (batch, channels=7, time_steps<=100)
        x = self.pool1(F.relu(self.bn1(self.conv1(x))))
        x = self.pool2(F.relu(self.bn2(self.conv2(x))))
        x = x.view(x.size(0), -1)
        x = self.dropout(F.relu(self.fc1(x)))
        x = self.fc2(x)
        return x