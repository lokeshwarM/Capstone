"""
=============================================================================
Deep LSTM Trajectory Sequence Model & Predictive Dead-Reckoning
Autonomous Swarm UAV Delivery System | Academic Capstone Research
=============================================================================
This module provides a recurrent neural network (LSTM) for:
  1. Learning temporal-spatial patterns of 3D UAV trajectories in urban canyons.
  2. Next-waypoint displacement forecasting for flight planning and collision avoidance.
  3. Real-time dead-reckoning during GPS dropouts / sensor denial events.
=============================================================================
"""

import os
import math
# pyrefly: ignore [missing-import]
import numpy as np
# pyrefly: ignore [missing-import]
import torch
# pyrefly: ignore [missing-import]
import torch.nn as nn


class TrajectoryLSTM(nn.Module):
    """
    Multi-layer Long Short-Term Memory (LSTM) network designed for 3D
    UAV spatial velocity and displacement delta (ΔX, ΔY, ΔZ) sequential forecasting.
    Predicting displacements ensures scale invariance, rapid convergence,
    and physics-consistent dead-reckoning during GPS denial.
    """
    def __init__(self, input_size=3, hidden_size=64, num_layers=2, output_size=3, dropout=0.05):
        super(TrajectoryLSTM, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.output_size = output_size

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )

        self.head = nn.Sequential(
            nn.Linear(hidden_size, 32),
            nn.LeakyReLU(0.1),
            nn.Linear(32, output_size)
        )

    def forward(self, x, hidden=None):
        """
        Forward pass.
        Args:
            x: Tensor of shape (batch_size, seq_len, input_size) containing displacement deltas
            hidden: Optional tuple (h_0, c_0)
        Returns:
            out: Predicted next displacement delta (batch_size, output_size)
            hidden: Updated LSTM hidden states
        """
        lstm_out, hidden = self.lstm(x, hidden)
        last_step = lstm_out[:, -1, :]
        out = self.head(last_step)
        return out, hidden

    def dead_reckon_autoregressive(self, seed_seq, steps=12):
        """
        Autoregressively rolls predictions forward when external GPS/sensors drop out.
        Args:
            seed_seq: (seq_len + 1, 3) historical 3D positions before GPS denial
            steps: Number of steps to forecast in dead-reckoning mode
        Returns:
            forecasts: np.ndarray of shape (steps, 3) of predicted 3D waypoints
        """
        self.eval()
        with torch.no_grad():
            pts = list(seed_seq)
            # Compute input displacement deltas from historical positions
            diffs = [pts[i] - pts[i - 1] for i in range(1, len(pts))]
            curr_diffs = torch.tensor(np.array(diffs), dtype=torch.float32).unsqueeze(0)

            forecasts = []
            curr_pos = pts[-1].copy()

            # Prime the recurrent hidden state with the historical sequence
            lstm_out, (h, c) = self.lstm(curr_diffs)
            curr_in = curr_diffs[:, -1:, :]

            for _ in range(steps):
                lstm_step, (h, c) = self.lstm(curr_in, (h, c))
                pred_delta = self.head(lstm_step[:, -1, :])
                delta_np = pred_delta.squeeze(0).cpu().numpy()
                curr_pos = curr_pos + delta_np
                forecasts.append(curr_pos.copy())

                # Step forward with recurrent prediction
                curr_in = pred_delta.unsqueeze(1)

            return np.array(forecasts)


def generate_realistic_uav_trajectory(num_steps=100, truck_pos=(0.0, -37.5, 1.5), target_pos=(30.0, -45.0, 35.1), cruise_alt=50.0):
    """
    Synthesizes a realistic 3D UAV delivery trajectory through urban street canyons:
      1. Vertical takeoff from truck deck (0 <= t < 0.15)
      2. Longitudinal street corridor flight (0.15 <= t < 0.45)
      3. Smooth banking corridor turn around building corners (0.45 <= t < 0.75)
      4. Deceleration & vertical descent to rooftop pad (0.75 <= t <= 1.0)
    """
    t = np.linspace(0, 1, num_steps)
    points = np.zeros((num_steps, 3))
    
    start_x, start_y, start_z = truck_pos
    end_x, end_y, end_z = target_pos
    mid_x = (start_x + end_x) / 2.0 + 8.0 * np.sign(end_x - start_x + 0.1)

    for i, s in enumerate(t):
        if s < 0.15:
            # Takeoff
            frac = s / 0.15
            points[i, 0] = start_x
            points[i, 1] = start_y
            points[i, 2] = start_z + frac * (cruise_alt - start_z)
        elif s < 0.45:
            # Flight segment 1: Along Avenue
            frac = (s - 0.15) / 0.30
            points[i, 0] = start_x + frac * (mid_x - start_x)
            points[i, 1] = start_y + np.sin(frac * np.pi) * 1.2
            points[i, 2] = cruise_alt
        elif s < 0.75:
            # Flight segment 2: Corridor Turn into Cross Street around building
            frac = (s - 0.45) / 0.30
            points[i, 0] = mid_x + frac * (end_x - mid_x)
            points[i, 1] = start_y + (0.5 * (1.0 - np.cos(frac * np.pi))) * (end_y - start_y)
            points[i, 2] = cruise_alt + np.sin(frac * np.pi) * 0.5
        else:
            # Final descent to rooftop
            frac = (s - 0.75) / 0.25
            points[i, 0] = end_x
            points[i, 1] = end_y
            points[i, 2] = cruise_alt - frac * (cruise_alt - end_z)

    return points


def create_training_dataset(destinations, num_trajectories=75, seq_len=10, noise_std=0.02):
    """
    Generates a diverse set of UAV trajectories across all building targets
    and slices them into displacement sliding windows:
      (X: seq_len displacement deltas -> y: next displacement delta).
    """
    truck_start = (0.0, -37.5, 1.5)
    all_trajectories = []

    np.random.seed(42)
    for _ in range(num_trajectories):
        dest = destinations[np.random.randint(len(destinations))]
        noisy_dest = (dest[0] + np.random.normal(0, 0.8),
                      dest[1] + np.random.normal(0, 0.8),
                      dest[2])
        traj = generate_realistic_uav_trajectory(
            num_steps=100,
            truck_pos=truck_start,
            target_pos=noisy_dest,
            cruise_alt=50.0 + np.random.uniform(-1.5, 1.5)
        )
        traj += np.random.normal(0, noise_std, traj.shape)
        all_trajectories.append(traj)

    X_list, y_list = [], []
    for traj in all_trajectories:
        deltas = traj[1:] - traj[:-1]
        for i in range(len(deltas) - seq_len):
            X_list.append(deltas[i : i + seq_len])
            y_list.append(deltas[i + seq_len])

    X = torch.tensor(np.array(X_list), dtype=torch.float32)
    y = torch.tensor(np.array(y_list), dtype=torch.float32)
    return X, y, all_trajectories


def train_lstm_model(model, X, y, epochs=40, batch_size=64, lr=0.003):
    """
    Trains the TrajectoryLSTM network on sliding displacement sequence pairs.
    """
    dataset = torch.utils.data.TensorDataset(X, y)
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=15, gamma=0.5)

    loss_history = []
    model.train()
    for epoch in range(epochs):
        epoch_loss = 0.0
        for batch_X, batch_y in loader:
            optimizer.zero_grad()
            pred, _ = model(batch_X)
            loss = criterion(pred, batch_y)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item() * len(batch_X)

        scheduler.step()
        epoch_loss /= len(dataset)
        loss_history.append(epoch_loss)

    return loss_history


def evaluate_dead_reckoning(model, test_trajectory, dropout_start=55, dropout_len=12, seq_len=10):
    """
    Simulates a mission scenario where GPS is lost as drone navigates a street corner turn.
    Compares:
      1. Ground Truth 3D path
      2. LSTM Autoregressive Dead-Reckoning
      3. Naive Linear Velocity Extrapolation
    """
    seed_seq = test_trajectory[dropout_start - (seq_len + 1) : dropout_start]
    gt_dropout = test_trajectory[dropout_start : dropout_start + dropout_len]

    # 1. LSTM Autoregressive Prediction
    lstm_preds = model.dead_reckon_autoregressive(seed_seq, steps=dropout_len)

    # 2. Baseline: Linear Velocity Extrapolation from last 2 GPS points
    p_last = test_trajectory[dropout_start - 1]
    p_prev = test_trajectory[dropout_start - 2]
    velocity = p_last - p_prev
    linear_preds = np.array([p_last + velocity * (k + 1) for k in range(dropout_len)])

    # Compute Error Metrics
    lstm_rmse = float(np.sqrt(np.mean((lstm_preds - gt_dropout) ** 2)))
    lstm_mae = float(np.mean(np.abs(lstm_preds - gt_dropout)))
    max_lstm_drift = float(np.max(np.linalg.norm(lstm_preds - gt_dropout, axis=1)))

    linear_rmse = float(np.sqrt(np.mean((linear_preds - gt_dropout) ** 2)))
    linear_mae = float(np.mean(np.abs(linear_preds - gt_dropout)))
    max_linear_drift = float(np.max(np.linalg.norm(linear_preds - gt_dropout, axis=1)))

    drift_reduction = ((linear_rmse - lstm_rmse) / linear_rmse) * 100.0

    return {
        'lstm_preds': lstm_preds,
        'linear_preds': linear_preds,
        'gt_dropout': gt_dropout,
        'dropout_indices': (dropout_start, dropout_start + dropout_len),
        'lstm_rmse': lstm_rmse,
        'lstm_mae': lstm_mae,
        'max_lstm_drift': max_lstm_drift,
        'linear_rmse': linear_rmse,
        'linear_mae': linear_mae,
        'max_linear_drift': max_linear_drift,
        'drift_reduction_pct': drift_reduction
    }
